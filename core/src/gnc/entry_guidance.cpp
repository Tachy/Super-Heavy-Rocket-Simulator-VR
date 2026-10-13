#include "starsim/gnc/entry_guidance.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/gnc/belly_flop.hpp"
#include "starsim/frames/geodetic.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double converged_downrange_m = 500.0;
constexpr double converged_bank_rad = 0.1 * constants::deg_to_rad;
constexpr double bracket_step_rad = 10.0 * constants::deg_to_rad;
constexpr int max_illinois_iterations = 8;
constexpr int constraint_bisections = 6;
constexpr double belly_flop_max_height_m = 40000.0;  // the hand-over Mach number is only trusted in dense air

double sign_of(double x) { return x < 0.0 ? -1.0 : 1.0; }

// Bank of the body now (entry_attitude convention): lift direction = the body axis's component perpendicular to the
// air velocity, measured from the local vertical plane.
double measured_bank_rad(const Vec3& axis, const Vec3& air_velocity, const Vec3& up) {
    const Vec3 u = math::normalized(air_velocity);
    Vec3 l0 = up - u * dot(up, u);
    const Vec3 l_raw = axis - u * dot(axis, u);
    if (norm(l0) < 1e-9 || norm(l_raw) < 1e-9) return 0.0;
    l0 = math::normalized(l0);
    const Vec3 l = math::normalized(l_raw);
    const Vec3 side = cross(u, l0);
    return std::atan2(-dot(l, side), dot(l, l0));
}

}  // namespace

const char* to_string(EntryPhase phase) {
    switch (phase) {
        case EntryPhase::Entry: return "Eintritt";
        case EntryPhase::BellyFlop: return "Bauchflug";
        case EntryPhase::Gate: return "Tor";
    }
    return "?";
}

EntryGuidance::EntryGuidance(const vehicle::FlightBody& ship, const AlphaSchedule& schedule, EntryTarget target,
                             EntryGuidanceParameters parameters)
    : target_(target), parameters_(parameters) {
    const auto& stage = ship.vehicle().stages().front();
    model_.aero = EntryAeroTable(ship.vehicle(), schedule);
    model_.mass_kg = ship.vehicle().mass_properties().mass_kg;
    model_.nose_radius_m = stage.config.heat_shield.nose_radius_m;
    model_.bank_rate_radps = parameters_.bank_rate_radps;
    if (!parameters_.predictor_j2) model_.j2 = 0.0;
    limits_ = stage.config.heat_shield;
    belly_azimuth_rad_ = stage.config.aero.belly_azimuth_rad;
}

bool EntryGuidance::violates(const EntryPrediction& p) const {
    const double f = parameters_.constraint_fraction;
    if (limits_.max_heat_rate_w_m2 > 0.0 && p.peak_heat_rate_w_m2 > f * limits_.max_heat_rate_w_m2) return true;
    if (limits_.max_load_g > 0.0 && p.peak_load_g > f * limits_.max_load_g) return true;
    return p.peak_dynamic_pressure_pa > parameters_.max_dynamic_pressure_pa;
}

EntryGuidance::Solution EntryGuidance::solve(const EntryPredictorModel& model, const EntryPredictorState& state,
                                             double heat_load_j_m2, double sign, double warm_start_rad) const {
    Solution s;
    const double max_bank = parameters_.max_bank_rad;
    const auto predict = [&](double magnitude) {
        ++s.predictions;
        return predict_entry(model, state, sign * magnitude, target_, parameters_.gate_height_m, heat_load_j_m2);
    };
    // f(m) = downrange error, falling with the bank magnitude m. Bracket from the warm start.
    double a = std::clamp(warm_start_rad, 0.0, max_bank);
    EntryPrediction pa = predict(a);
    double b = a;
    EntryPrediction pb = pa;
    double step = bracket_step_rad;
    const bool more_bank = pa.downrange_error_m > 0.0;
    bool bracketed = false, saturated = false;
    while (!bracketed && !saturated) {
        b = std::clamp(more_bank ? b + step : b - step, 0.0, max_bank);
        pb = predict(b);
        if ((pb.downrange_error_m > 0.0) != more_bank) {
            bracketed = true;
        } else if (b <= 0.0 || b >= max_bank) {
            saturated = true;  // even the bank limit does not reach the target
        } else {
            a = b;
            pa = pb;
            step *= 2.0;
        }
    }
    if (saturated) {
        s.bank_rad = b;
        s.prediction = pb;
    } else {
        // Illinois (modified regula falsi) between a and b, whose errors have opposite signs.
        double fa = pa.downrange_error_m, fb = pb.downrange_error_m;
        s.bank_rad = std::abs(fa) < std::abs(fb) ? a : b;
        s.prediction = std::abs(fa) < std::abs(fb) ? pa : pb;
        int side = 0;
        for (int i = 0; i < max_illinois_iterations; ++i) {
            if (std::abs(s.prediction.downrange_error_m) < converged_downrange_m || std::abs(b - a) < converged_bank_rad) break;
            const double c = (a * fb - b * fa) / (fb - fa);
            const EntryPrediction pc = predict(c);
            const double fc = pc.downrange_error_m;
            s.bank_rad = c;
            s.prediction = pc;
            if ((fc > 0.0) == (fa > 0.0)) {
                a = c;
                fa = fc;
                if (side == -1) fb *= 0.5;
                side = -1;
            } else {
                b = c;
                fb = fc;
                if (side == 1) fa *= 0.5;
                side = 1;
            }
        }
    }
    // Corridor: heat rate, load and dynamic pressure peak higher with more bank (steeper) - cut it back to the
    // largest bank inside the limits.
    if (violates(s.prediction)) {
        s.constrained = true;
        double lo = 0.0, hi = s.bank_rad;
        EntryPrediction p_lo = predict(0.0);
        if (!violates(p_lo)) {
            for (int i = 0; i < constraint_bisections; ++i) {
                const double mid = 0.5 * (lo + hi);
                const EntryPrediction p_mid = predict(mid);
                if (violates(p_mid)) {
                    hi = mid;
                } else {
                    lo = mid;
                    p_lo = p_mid;
                }
            }
        }
        s.bank_rad = lo;
        s.prediction = p_lo;
    }
    return s;
}

void EntryGuidance::entry_cycle(const vehicle::FlightBody& body, const Vec3& r_itrf, const Vec3& v_itrf) {
    model_.mass_kg = body.vehicle().mass_properties().mass_kg;
    const EntryPredictorState state{r_itrf, v_itrf, bank_rad_};

    const Vec3 up = math::normalized(r_itrf);
    const Vec3 v_h = v_itrf - up * dot(v_itrf, up);
    const Vec3 t = math::normalized(target_.position_itrf_m);
    const Vec3 d = t - up * dot(t, up);
    const double heading_error = -std::atan2(dot(cross(v_h, d), up), dot(v_h, d));  // > 0: target to the right
    // Bank sign. A positive bank turns the lift to the left of the track (entry_attitude), so with the sign held the
    // predicted end lies left of the target (crossrange > 0) when it overshoots. First cycle: the better sign.
    Solution s;
    if (sign_ == 0.0) {
        const Solution left = solve(model_, state, body.heat_load_j_m2(), 1.0, bank_magnitude_rad_);
        const Solution right = solve(model_, state, body.heat_load_j_m2(), -1.0, bank_magnitude_rad_);
        const bool use_left = std::abs(left.prediction.crossrange_m) <= std::abs(right.prediction.crossrange_m);
        sign_ = use_left ? 1.0 : -1.0;
        s = use_left ? left : right;
    } else {
        s = solve(model_, state, body.heat_load_j_m2(), sign_, bank_magnitude_rad_);
        const double deadband = parameters_.reversal_crossrange_base_m +
                                parameters_.reversal_crossrange_fraction * ground_range_m(r_itrf, target_.position_itrf_m);
        if (norm(v_itrf) > parameters_.min_reversal_speed_mps && sign_ * s.prediction.crossrange_m > deadband) {
            sign_ = -sign_;
            s = solve(model_, state, body.heat_load_j_m2(), sign_, bank_magnitude_rad_);
        }
    }
    bank_magnitude_rad_ = s.bank_rad;
    bank_target_rad_ = sign_ * s.bank_rad;
    command_.downrange_error_m = s.prediction.downrange_error_m;
    command_.crossrange_m = s.prediction.crossrange_m;
    command_.heading_error_rad = heading_error;
    command_.constrained = s.constrained;
}

void EntryGuidance::belly_flop(const vehicle::FlightBody& body, const math::Mat3& itrf_to_gcrf, const Vec3& r_itrf,
                               const Vec3& v_itrf, double height_m, double dt_s) {
    const Vec3 up = math::normalized(r_itrf);
    const Vec3 offset = r_itrf - target_.position_itrf_m;
    const Vec3 offset_h = offset - up * dot(offset, up);
    const Vec3 v_h = v_itrf - up * dot(v_itrf, up);
    if (!flop_started_) {  // from the attitude flown now: heading of the nose (along the track after the entry)
        flop_started_ = true;
        flop_attitude_gcrf_ = body.state().attitude;
        const Vec3 x = transpose(itrf_to_gcrf) * body.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
        const Vec3 x_h = x - up * dot(x, up);
        nose_itrf_ = norm(x_h) > 1e-3 ? x_h / norm(x_h) : math::normalized(cross(Vec3{0.0, 0.0, 1.0}, up));
    }
    const double height_above_gate = height_m - target_.ground_height_m - parameters_.gate_height_m;
    const double sink = -dot(v_itrf, up);
    const Vec3 a = belly_flop_acceleration(offset_h, v_h, height_above_gate, sink, parameters_.zem_gain);
    const double a_norm = norm(a);
    // The hull is a cylinder: its cross-flow force lies in the plane of the axis and the (near vertical) air flow,
    // i.e. horizontally along the axis - rolling gives no side force. So the axis is yawed onto the wanted
    // acceleration (the nose keeps its hemisphere) and pitched: nose down pushes forward, nose up backward. The
    // heading turns at most at the bank rate limit (yaw about the belly axis is slow; faster turns spin up, b1).
    Vec3 nose = math::normalized(nose_itrf_ - up * dot(nose_itrf_, up));
    if (a_norm > parameters_.belly_flop_turn_min_acceleration_mps2 && phase_ != EntryPhase::Gate) {
        const Vec3 along = a / a_norm;
        const Vec3 wanted_nose = dot(along, nose) >= 0.0 ? along : -along;
        const double turn = std::atan2(dot(cross(nose, wanted_nose), up), dot(nose, wanted_nose));
        const double max_turn = parameters_.bank_rate_radps * dt_s;
        const double step = std::clamp(turn, -max_turn, max_turn);
        nose = math::normalized(nose * std::cos(step) + cross(up, nose) * std::sin(step));
    }
    nose_itrf_ = nose;
    // At the gate the flip takes over (b3): until then the belly stays level.
    const double tilt_magnitude =
        phase_ == EntryPhase::Gate
            ? 0.0
            : std::min(std::atan(std::abs(dot(a, nose)) / constants::standard_gravity_mps2), parameters_.max_tilt_rad);
    const double tilt = dot(a, nose) >= 0.0 ? tilt_magnitude : -tilt_magnitude;  // > 0: nose down
    const Vec3 direction = nose;
    const auto wanted = belly_flop_attitude(up, nose, direction, tilt, belly_azimuth_rad_);
    const math::Quat wanted_gcrf = attitude_from_axes(itrf_to_gcrf * wanted.axis_gcrf, itrf_to_gcrf * wanted.roll_reference_gcrf);
    flop_attitude_gcrf_ = slew_toward(flop_attitude_gcrf_, wanted_gcrf, parameters_.belly_flop_slew_rate_radps * dt_s);
    command_.axis_gcrf = flop_attitude_gcrf_.rotate(Vec3{1.0, 0.0, 0.0});
    command_.roll_reference_gcrf = flop_attitude_gcrf_.rotate(Vec3{0.0, 1.0, 0.0});
    command_.alpha_rad = 0.5 * constants::pi - tilt;
    command_.bank_rad = 0.0;
    // zero-effort miss at the gate: along the nose (> 0: beyond the gate) and to its right
    const double t_go = std::max(height_above_gate, 1.0) / std::max(sink, 20.0);
    const Vec3 zem = offset_h + v_h * t_go;
    command_.downrange_error_m = dot(zem, nose);
    command_.crossrange_m = -dot(zem, cross(nose, up));
}

EntryCommand EntryGuidance::update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                   time::Epoch epoch) {
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const auto& s = body.state();
    const Vec3 v_rel_gcrf = s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(transform), s.position_gcrf_m);
    const Vec3 r_itrf = transpose(transform.rotation) * s.position_gcrf_m;
    const Vec3 v_itrf = transpose(transform.rotation) * v_rel_gcrf;
    const Vec3 up_gcrf = math::normalized(s.position_gcrf_m);
    const double height = frames::fixed_to_geodetic(r_itrf).height_m;
    const environment::Us76Atmosphere atmosphere;
    const auto air = atmosphere.at(height);
    const double speed = norm(v_rel_gcrf);
    const double mach = air.speed_of_sound_mps > 0.0 ? speed / air.speed_of_sound_mps : 0.0;
    const Vec3 axis_gcrf = s.attitude.rotate(Vec3{1.0, 0.0, 0.0});

    if (!started_) {  // take over the bank the vehicle flies now (whoever flew it so far)
        started_ = true;
        bank_rad_ = measured_bank_rad(axis_gcrf, v_rel_gcrf, up_gcrf);
        last_update_ = epoch;
        last_cycle_ = epoch - parameters_.cycle_s;
    }
    const double dt = epoch - last_update_;
    last_update_ = epoch;

    // Online scale factors: the accelerometer's drag and lift against the model at this Mach number.
    const Vec3 a_aero = s.attitude.rotate(body.aero_acceleration_body_mps2());
    if (phase_ == EntryPhase::Entry && epoch - last_cycle_ >= parameters_.cycle_s &&
        norm(a_aero) > parameters_.scale_min_acceleration_mps2 && air.density_kg_m3 > 0.0) {
        const Vec3 u = math::normalized(v_rel_gcrf);
        const double q_over_m = 0.5 * air.density_kg_m3 * speed * speed / model_.mass_kg;
        const double drag = -dot(a_aero, u), lift = norm(a_aero + u * drag);
        const double drag_model = q_over_m * model_.aero.cd_area_m2(mach);
        const double lift_model = q_over_m * model_.aero.cl_area_m2(mach);
        const auto filter = [&](double& k, double measured) {
            k += parameters_.scale_filter_gain * (std::clamp(measured, 0.5, 2.0) - k);
        };
        if (drag_model > 0.0) filter(model_.drag_scale, drag / drag_model);
        if (lift_model > 0.0) filter(model_.lift_scale, lift / lift_model);
    }

    if (height - target_.ground_height_m <= parameters_.gate_height_m) phase_ = EntryPhase::Gate;
    if (phase_ == EntryPhase::Entry && mach < parameters_.belly_flop_mach && height < belly_flop_max_height_m) {
        phase_ = EntryPhase::BellyFlop;
    }

    if (phase_ == EntryPhase::Entry) {
        if (epoch - last_cycle_ >= parameters_.cycle_s) {
            last_cycle_ = epoch;
            entry_cycle(body, r_itrf, v_itrf);
        }
        const double max_change = parameters_.bank_rate_radps * dt;
        bank_rad_ += std::clamp(bank_target_rad_ - bank_rad_, -max_change, max_change);
        command_.alpha_rad = model_.aero.alpha_rad(mach);
        command_.bank_rad = bank_rad_;
        const auto attitude = entry_attitude(v_rel_gcrf, up_gcrf, command_.alpha_rad, bank_rad_, belly_azimuth_rad_);
        command_.axis_gcrf = attitude.axis_gcrf;
        command_.roll_reference_gcrf = attitude.roll_reference_gcrf;
    }
    if (phase_ != EntryPhase::Entry) belly_flop(body, transform.rotation, r_itrf, v_itrf, height, dt);
    command_.phase = phase_;
    command_.drag_scale = model_.drag_scale;
    command_.lift_scale = model_.lift_scale;
    return command_;
}

}  // namespace starsim::gnc
