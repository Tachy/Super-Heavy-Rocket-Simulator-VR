#include "starsim/gnc/entry_guidance.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/gnc/attitude_slew.hpp"
#include "starsim/frames/geodetic.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double converged_downrange_m = 500.0;
constexpr double converged_bank_rad = 0.1 * constants::deg_to_rad;
constexpr double bracket_step_rad = 10.0 * constants::deg_to_rad;
constexpr int max_illinois_iterations = 8;
constexpr int constraint_bisections = 6;
constexpr double terminal_max_height_m = 40000.0;  // the hand-over Mach number is only trusted in dense air
constexpr double max_terminal_prediction_s = 900.0;
constexpr double min_steering_fraction = 1.0e-3;  // below: the lift direction of the last cycle is kept  // the hand-over Mach number is only trusted in dense air

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
        case EntryPhase::Terminal: return "Endanflug";
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
    model_.fast_bank_rate_radps = parameters_.fast_bank_rate_radps;
    model_.fast_bank_mach_low = parameters_.fast_bank_mach_low;
    model_.fast_bank_mach_high = parameters_.fast_bank_mach_high;
    model_.terminal_min_alpha_rad = parameters_.terminal_min_alpha_rad;
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
        return predict_entry(model, state, sign * magnitude, target_, parameters_.gate_height_m,
                             heat_load_j_m2);
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
    time_since_reversal_s_ += parameters_.cycle_s;
    const EntryPredictorState state{r_itrf, v_itrf, bank_rad_};

    const Vec3 up = math::normalized(r_itrf);
    const Vec3 v_h = v_itrf - up * dot(v_itrf, up);
    const Vec3 t = math::normalized(target_.position_itrf_m);
    const Vec3 d = t - up * dot(t, up);
    const double heading_error = -std::atan2(dot(cross(v_h, d), up), dot(v_h, d));  // > 0: target to the right
    // Lateral: the corrector times the bank reversals. Holding the sign to the gate ends with the crossrange
    // c_hold, reversing now with c_now; when they lie on opposite sides of the target a reversal at some time ends on
    // it, and it is flown in the cycle that time falls into. Otherwise the sign whose prediction ends closer across
    // is taken. The terminal phase removes what is left.
    if (sign_ == 0.0) {
        const Solution left = solve(model_, state, body.heat_load_j_m2(), 1.0, bank_magnitude_rad_);
        const Solution right = solve(model_, state, body.heat_load_j_m2(), -1.0, bank_magnitude_rad_);
        sign_ = std::abs(left.prediction.crossrange_m) <= std::abs(right.prediction.crossrange_m) ? 1.0 : -1.0;
    }
    Solution s = solve(model_, state, body.heat_load_j_m2(), sign_, bank_magnitude_rad_);
    if (norm(v_itrf) > parameters_.min_reversal_speed_mps && time_since_reversal_s_ >= parameters_.min_reversal_interval_s) {
        EntryPredictorState now = state;
        now.reversal_time_s = 0.0;
        const Solution reverse_now = solve(model_, now, body.heat_load_j_m2(), sign_, bank_magnitude_rad_);
        const double c_hold = s.prediction.crossrange_m, c_now = reverse_now.prediction.crossrange_m;
        bool reverse = false;
        if ((c_hold > 0.0) != (c_now > 0.0)) {
            // The reversal time that ends on the target lies between now and never: it has come when a reversal
            // one cycle from now would already end on the side of holding the sign.
            EntryPredictorState next = state;
            next.reversal_time_s = parameters_.cycle_s;
            const Solution reverse_next = solve(model_, next, body.heat_load_j_m2(), sign_, bank_magnitude_rad_);
            const double c_next = reverse_next.prediction.crossrange_m;
            reverse = (c_next > 0.0) == (c_hold > 0.0);
            if (!reverse) {
                // the bank magnitude for the reversal still to come: its time interpolated between the next cycle
                // and the end of the prediction (the crossrange over the reversal time is nearly linear)
                EntryPredictorState planned = state;
                planned.reversal_time_s = parameters_.cycle_s + (s.prediction.time_s - parameters_.cycle_s) *
                                                                     c_next / (c_next - c_hold);
                const Solution at = solve(model_, planned, body.heat_load_j_m2(), sign_, bank_magnitude_rad_);
                s.bank_rad = at.bank_rad;
                s.prediction.downrange_error_m = at.prediction.downrange_error_m;
                s.constrained = at.constrained;
            }
        } else if (std::abs(c_now) + parameters_.reversal_crossrange_base_m < std::abs(c_hold)) {
            reverse = true;
        }
        if (reverse) {
            sign_ = -sign_;
            time_since_reversal_s_ = 0.0;
            s = reverse_now;
        }
    }
    bank_magnitude_rad_ = s.bank_rad;
    bank_target_rad_ = sign_ * s.bank_rad;
    command_.downrange_error_m = s.prediction.downrange_error_m;
    command_.crossrange_m = s.prediction.crossrange_m;
    command_.heading_error_rad = heading_error;
    command_.constrained = s.constrained;
}

Vec3 EntryGuidance::terminal_miss(const vehicle::FlightBody& body, const Vec3& r_itrf, const Vec3& v_itrf,
                                  const Vec3& steering, double* time_to_go_s) const {
    EntryPredictorModel m = model_;
    m.mass_kg = body.vehicle().mass_properties().mass_kg;
    EntryPredictorState state{r_itrf, v_itrf, 0.0};
    state.steered = true;
    state.steering_itrf = steering;
    const EntryPrediction p = predict_entry(m, state, 0.0, target_, parameters_.gate_height_m, 0.0,
                                            parameters_.terminal_prediction_step_s, max_terminal_prediction_s);
    if (time_to_go_s) *time_to_go_s = p.time_s;
    const Vec3 end = p.end_position_itrf_m - target_.position_itrf_m;
    const Vec3 end_up = math::normalized(p.end_position_itrf_m);
    return end - end_up * dot(end, end_up);
}

void EntryGuidance::terminal(const vehicle::FlightBody& body, const math::Mat3& itrf_to_gcrf, const Vec3& r_itrf,
                             const Vec3& v_itrf, double dt_s) {
    const Vec3 up = math::normalized(r_itrf);
    const Vec3 east = math::normalized(cross(Vec3{0.0, 0.0, 1.0}, up));
    const Vec3 north = cross(up, east);
    const Vec3 u = math::normalized(v_itrf);
    if (!terminal_started_) {  // the lift vector flown now: alpha and lift direction of the body axis
        terminal_started_ = true;
        terminal_attitude_gcrf_ = body.state().attitude;
        const Vec3 x = transpose(itrf_to_gcrf) * body.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
        const double alpha = std::acos(std::clamp(dot(x, u), -1.0, 1.0));
        const Vec3 l = x - u * dot(x, u);
        terminal_lift_direction_itrf_ = norm(l) > 1e-6 ? l / norm(l) : north;
        terminal_steering_itrf_ = terminal_lift_direction_itrf_ * model_.steering_fraction(alpha);
    }
    terminal_prediction_age_s_ += dt_s;
    if (phase_ == EntryPhase::Terminal &&
        (terminal_prediction_age_s_ >= parameters_.terminal_cycle_s || !terminal_predicted_)) {
        terminal_prediction_age_s_ = 0.0;
        terminal_predicted_ = true;
        // steering basis perpendicular to the air velocity: "up" and "side" (north when the fall is vertical)
        Vec3 e1 = up - u * dot(up, u);
        if (norm(e1) < 0.1) e1 = north - u * dot(north, u);
        e1 = math::normalized(e1);
        const Vec3 e2 = cross(u, e1);
        Vec3 p = terminal_steering_itrf_ - u * dot(terminal_steering_itrf_, u);
        const Vec3 z0 = terminal_miss(body, r_itrf, v_itrf, p, nullptr);
        const double d = parameters_.terminal_newton_step;
        const Vec3 z1 = terminal_miss(body, r_itrf, v_itrf, p + e1 * d, nullptr);
        const Vec3 z2 = terminal_miss(body, r_itrf, v_itrf, p + e2 * d, nullptr);
        // 2x2 Jacobian of the miss (east, north) over the steering (e1, e2)
        const double j11 = dot(z1 - z0, east) / d, j12 = dot(z2 - z0, east) / d;
        const double j21 = dot(z1 - z0, north) / d, j22 = dot(z2 - z0, north) / d;
        const double det = j11 * j22 - j12 * j21;
        if (std::abs(det) > 1e-9) {
            const double me = dot(z0, east), mn = dot(z0, north);
            const double d1 = -(j22 * me - j12 * mn) / det, d2 = -(-j21 * me + j11 * mn) / det;
            p = p + e1 * d1 + e2 * d2;
            if (norm(p) > 1.0) p = p / norm(p);
        }
        terminal_steering_itrf_ = p;
        terminal_zem_itrf_ = z0;
    }
    // At the gate the flip takes over (b3): until then the steering of the last cycle is held.
    const double fraction = std::min(norm(terminal_steering_itrf_), 1.0);
    if (fraction > min_steering_fraction) {
        terminal_lift_direction_itrf_ = terminal_steering_itrf_ / norm(terminal_steering_itrf_);
    }
    const double alpha = model_.steering_alpha(fraction);
    const auto wanted = entry_attitude_with_lift(itrf_to_gcrf * v_itrf, itrf_to_gcrf * terminal_lift_direction_itrf_, alpha,
                                                 belly_azimuth_rad_);
    const math::Quat wanted_gcrf = attitude_from_axes(wanted.axis_gcrf, wanted.roll_reference_gcrf);
    terminal_attitude_gcrf_ =
        slew_toward(terminal_attitude_gcrf_, wanted_gcrf, parameters_.terminal_slew_rate_radps * dt_s);
    command_.axis_gcrf = terminal_attitude_gcrf_.rotate(Vec3{1.0, 0.0, 0.0});
    command_.roll_reference_gcrf = terminal_attitude_gcrf_.rotate(Vec3{0.0, 1.0, 0.0});
    command_.alpha_rad = alpha;
    command_.bank_rad = 0.0;
    // predicted miss at the gate along the horizontal track (> 0: beyond) and to its right
    Vec3 track = v_itrf - up * dot(v_itrf, up);
    track = norm(track) > 1e-3 ? track / norm(track) : terminal_lift_direction_itrf_;
    command_.downrange_error_m = dot(terminal_zem_itrf_, track);
    command_.crossrange_m = -dot(terminal_zem_itrf_, cross(track, up));
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
    if (phase_ == EntryPhase::Entry && mach < parameters_.terminal_mach && height < terminal_max_height_m) {
        phase_ = EntryPhase::Terminal;
    }

    if (phase_ == EntryPhase::Entry) {
        if (epoch - last_cycle_ >= parameters_.cycle_s) {
            last_cycle_ = epoch;
            entry_cycle(body, r_itrf, v_itrf);
        }
        const double max_change = model_.bank_rate_at(mach) * dt;
        bank_rad_ += std::clamp(bank_target_rad_ - bank_rad_, -max_change, max_change);
        command_.alpha_rad = model_.aero.alpha_rad(mach);
        command_.bank_rad = bank_rad_;
        const auto attitude = entry_attitude(v_rel_gcrf, up_gcrf, command_.alpha_rad, bank_rad_, belly_azimuth_rad_);
        command_.axis_gcrf = attitude.axis_gcrf;
        command_.roll_reference_gcrf = attitude.roll_reference_gcrf;
    }
    if (phase_ != EntryPhase::Entry) terminal(body, transform.rotation, r_itrf, v_itrf, dt);
    command_.phase = phase_;
    command_.drag_scale = model_.drag_scale;
    command_.lift_scale = model_.lift_scale;
    return command_;
}

}  // namespace starsim::gnc
