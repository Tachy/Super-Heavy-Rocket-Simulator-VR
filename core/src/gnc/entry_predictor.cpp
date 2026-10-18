#include "starsim/gnc/entry_predictor.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/aero/entry_heating.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/frames/geodetic.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double entry_height_m = 80000.0;   // skip-out: below this, then above
constexpr double skip_out_height_m = 130000.0;
constexpr double min_track_speed_mps = 1.0;
constexpr double near_target_range_m = 50000.0;  // [E] miss metric switch (see predict_entry)

struct Exit {
    double apoapsis_height_m = 0.0;  // < 0: hyperbolic (no apoapsis)
    double periapsis_height_m = 0.0;
    double inclination_rad = 0.0;
};

// Osculating two-body orbit of an Earth-fixed state (inertial velocity v + w x r, the axes aligned at this instant).
Exit exit_orbit(const EntryPredictorModel& m, const Vec3& r, const Vec3& v_fixed) {
    const Vec3 v = v_fixed + cross(Vec3{0.0, 0.0, m.omega_radps}, r);
    const double rn = norm(r), energy = 0.5 * dot(v, v) - m.gm_m3ps2 / rn;
    const Vec3 h = cross(r, v);
    Exit e;
    e.inclination_rad = std::acos(std::clamp(h.z / norm(h), -1.0, 1.0));
    if (energy >= 0.0) {
        e.apoapsis_height_m = -1.0;
        return e;
    }
    const double a = -m.gm_m3ps2 / (2.0 * energy);
    const double ecc = std::sqrt(std::max(0.0, 1.0 - dot(h, h) / (m.gm_m3ps2 * a)));
    e.apoapsis_height_m = a * (1.0 + ecc) - m.earth_radius_m;
    e.periapsis_height_m = a * (1.0 - ecc) - m.earth_radius_m;
    return e;
}

struct Derivative {
    Vec3 dr, dv;
};

struct AeroSample {
    Vec3 acceleration;
    double heat_rate_w_m2 = 0.0;
    double dynamic_pressure_pa = 0.0;
    double height_m = 0.0;
    double mach = 0.0;
};

class Model {
public:
    Model(const EntryPredictorModel& m, const EntryPredictorState& start, const Vec3& nose, double gate_m)
        : m_(m), steered_(start.steered), steering_(start.steering), frozen_(start.basis_frozen),
          e1_(start.basis_e1), e2_(start.basis_e2), nose_(nose), gate_m_(gate_m) {
        const Vec3& d = norm(start.lift_direction) > 0.0 ? start.lift_direction : start.steering;
        lift_angle_ = std::atan2(d.y, d.x);
    }

    bool nose_mode() const { return steered_ && frozen_ && norm(nose_) > 0.0; }

    // Once per step: the lift direction turns towards the sector steering at the turn rate.
    void advance(const Vec3& r, double step_s) {
        if (!nose_mode()) return;
        const Vec3 target = sector_steering(frames::fixed_to_geodetic(r).height_m);
        const double turn = std::remainder(std::atan2(target.y, target.x) - lift_angle_, 2.0 * constants::pi);
        const double max_turn = m_.nose_turn_rate_radps * step_s;
        lift_angle_ += std::clamp(turn, -max_turn, max_turn);
    }

    // Steering basis: velocity frame until the path is steeper than the freeze angle, then fixed (decided per step).
    void update_basis(const Vec3& r, const Vec3& v) {
        if (!steered_ || frozen_) return;
        const Vec3 up = math::normalized(r), u = math::normalized(v);
        if (std::asin(std::clamp(dot(u, up), -1.0, 1.0)) < m_.steering_freeze_gamma_rad) {
            steering_basis(up, u, e1_, e2_);
            frozen_ = true;
            if (norm(nose_) > 0.0) {  // as the guidance
                lift_angle_ = std::atan2(steering_.y, steering_.x);  // the turn starts from the lift flown
                steering_ = nose_turn_steering(steering_, nose_, e1_, e2_,
                                               m_.nose_sector_at(frames::fixed_to_geodetic(r).height_m - gate_m_),
                                               m_.nose_fraction);
            }
        }
    }

    AeroSample aero(const Vec3& r, const Vec3& v, double bank_rad) const {
        AeroSample s;
        s.height_m = frames::fixed_to_geodetic(r).height_m;
        const auto air = atmosphere_.at(s.height_m);
        const double speed = norm(v);
        if (air.density_kg_m3 <= 0.0 || speed < 1e-3) return s;
        const double mach = speed / air.speed_of_sound_mps;
        s.mach = mach;
        s.dynamic_pressure_pa = 0.5 * air.density_kg_m3 * speed * speed;
        const Vec3 u = v / speed;
        const Vec3 up = math::normalized(r);
        Vec3 l0 = up - u * dot(up, u);
        if (norm(l0) < 1e-9) l0 = cross(u, std::abs(u.x) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0});
        l0 = math::normalized(l0);
        const Vec3 side = cross(u, l0);
        const Vec3 l = l0 * std::cos(bank_rad) - side * std::sin(bank_rad);
        const double q_over_m = s.dynamic_pressure_pa / m_.mass_kg;
        if (steered_) {
            Vec3 steering = steering_;
            if (nose_mode()) {  // the sector narrows towards the gate; the lift direction flown turns into it
                const Vec3 target = sector_steering(s.height_m);
                double size = norm(target);
                const double left = std::remainder(std::atan2(target.y, target.x) - lift_angle_, 2.0 * constants::pi);
                if (std::abs(left) > m_.nose_turned_rad) size = std::min(size, m_.nose_fraction);
                steering = Vec3{size * std::cos(lift_angle_), size * std::sin(lift_angle_), 0.0};
            }
            const double fraction = std::min(norm(steering), 1.0);
            const double alpha = m_.steering_alpha(fraction, mach);
            s.acceleration = u * (-q_over_m * m_.drag_scale * m_.aero.cd_area_m2(mach, alpha));
            Vec3 e1 = e1_, e2 = e2_;
            if (!frozen_) steering_basis(up, u, e1, e2);
            Vec3 lb = e1 * steering.x + e2 * steering.y;
            lb = lb - u * dot(lb, u);
            if (fraction > 0.0 && norm(lb) > 1e-9) {
                s.acceleration += lb / norm(lb) * (q_over_m * m_.lift_scale * m_.aero.cl_area_m2(mach, alpha));
            }
        } else {
            s.acceleration = u * (-q_over_m * m_.drag_scale * m_.aero.cd_area_m2(mach)) +
                             l * (q_over_m * m_.lift_scale * m_.aero.cl_area_m2(mach));
        }
        if (m_.nose_radius_m > 0.0) {
            s.heat_rate_w_m2 = aero::entry_heat_rate_w_m2(air.density_kg_m3, speed, m_.nose_radius_m, m_.radiative.get());
        }
        return s;
    }

    Derivative derivative(const Vec3& r, const Vec3& v, double bank_rad) const {
        const double rn = norm(r);
        const double z2 = r.z * r.z / (rn * rn);
        const double k = 1.5 * m_.j2 * (m_.earth_radius_m / rn) * (m_.earth_radius_m / rn);
        const double g = -m_.gm_m3ps2 / (rn * rn * rn);
        const Vec3 gravity{g * r.x * (1.0 - k * (5.0 * z2 - 1.0)), g * r.y * (1.0 - k * (5.0 * z2 - 1.0)),
                           g * r.z * (1.0 - k * (5.0 * z2 - 3.0))};
        const Vec3 w{0.0, 0.0, m_.omega_radps};
        const Vec3 a = gravity - cross(w, v) * 2.0 - cross(w, cross(w, r)) + aero(r, v, bank_rad).acceleration;
        return {v, a};
    }

private:
    const EntryPredictorModel& m_;
    bool steered_ = false;
    Vec3 steering_;
    bool frozen_ = false;
    Vec3 e1_, e2_;
    Vec3 sector_steering(double height_m) const {
        return nose_sector_steering(steering_, nose_, e1_, e2_, m_.nose_sector_at(height_m - gate_m_), m_.nose_min_fraction,
                                    m_.nose_max_fraction);
    }

    Vec3 nose_;
    double gate_m_ = 0.0;
    double lift_angle_ = 0.0;  // nose mode: lift direction flown, from e1 towards e2
    environment::Us76Atmosphere atmosphere_;
};

}  // namespace

void steering_basis(const Vec3& up, const Vec3& u, Vec3& e1, Vec3& e2) {
    e1 = up - u * dot(up, u);
    if (norm(e1) < 1.0e-2) {  // vertical fall (within ~0.6 deg): north instead
        const Vec3 east = math::normalized(cross(Vec3{0.0, 0.0, 1.0}, up));
        const Vec3 north = cross(up, east);
        e1 = north - u * dot(north, u);
    }
    e1 = math::normalized(e1);
    e2 = cross(u, e1);
}

Vec3 nose_steering(const Vec3& nose, const Vec3& e1, const Vec3& e2, double fraction) {
    const double heading = std::atan2(dot(nose, e2), dot(nose, e1));
    return {fraction * std::cos(heading), fraction * std::sin(heading), 0.0};
}

Vec3 nose_turn_steering(const Vec3& p, const Vec3& nose, const Vec3& e1, const Vec3& e2, double sector_rad,
                        double fraction) {
    if (norm(p) < 1e-9) return nose_steering(nose, e1, e2, fraction);
    return nose_sector_steering(p / norm(p) * fraction, nose, e1, e2, sector_rad, fraction, fraction);
}

Vec3 nose_sector_steering(const Vec3& p, const Vec3& nose, const Vec3& e1, const Vec3& e2, double sector_rad,
                          double min_fraction, double max_fraction) {
    const double heading = std::atan2(dot(nose, e2), dot(nose, e1));
    const double size = std::clamp(norm(p), min_fraction, max_fraction);
    const double offset = norm(p) > 1e-9 ? std::remainder(std::atan2(p.y, p.x) - heading, 2.0 * constants::pi) : 0.0;
    const double angle = heading + std::clamp(offset, -sector_rad, sector_rad);
    return {size * std::cos(angle), size * std::sin(angle), 0.0};
}

double ground_range_m(const Vec3& a_itrf_m, const Vec3& b_itrf_m) {
    const Vec3 a = math::normalized(a_itrf_m), b = math::normalized(b_itrf_m);
    return std::atan2(norm(cross(a, b)), dot(a, b)) * constants::wgs84_semi_major_axis_m;
}

namespace {

// The great-circle angle `gc_rad` (in [0, pi]) from the start to the end, unwrapped by the angle swept along the path:
// past the antipode the great circle shortens again (a skip and a glide near orbital speed fly more than half around
// the Earth). The branch closest to the swept angle; the great circle itself while the path stays within half a turn.
double unwrapped_range_rad(double gc_rad, double swept_rad) {
    const double turn = 2.0 * constants::pi;
    const double n = std::floor(swept_rad / turn);
    double best = gc_rad;
    for (const double c : {n * turn + gc_rad, (n + 1.0) * turn - gc_rad, n * turn - gc_rad}) {
        if (c >= 0.0 && std::abs(c - swept_rad) < std::abs(best - swept_rad)) best = c;
    }
    return best;
}

double angle_between(const Vec3& a, const Vec3& b) { return std::atan2(norm(cross(a, b)), dot(a, b)); }

}  // namespace

double bank_step(double bank_rad, double command_rad, double max_change_rad, bool shorter_way) {
    if (!shorter_way) return bank_rad + std::clamp(command_rad - bank_rad, -max_change_rad, max_change_rad);
    const double delta = std::remainder(command_rad - bank_rad, 2.0 * constants::pi);
    return std::remainder(bank_rad + std::clamp(delta, -max_change_rad, max_change_rad), 2.0 * constants::pi);
}

namespace {

// The signed way to go from `bank_rad` to `command_rad` by bank_step_held's rule (may exceed pi); updates
// `roll_direction`.
double held_delta(double bank_rad, double command_rad, int& roll_direction) {
    constexpr double ambiguous_rad = 150.0 * constants::deg_to_rad;   // [E] either way round is about as long
    constexpr double settled_rad = 5.0 * constants::deg_to_rad;       // [E] as the guidance's settled bank
    double delta = std::remainder(command_rad - bank_rad, 2.0 * constants::pi);
    if (std::abs(delta) > ambiguous_rad) {
        // a roll under way keeps its direction; a new one from a lift-down bank starts through lift down (away from 0,
        // as bank_step), one from a lift-up bank the shorter way
        const bool lift_down = std::abs(bank_rad) > 0.5 * constants::pi;
        const int wanted = roll_direction != 0 ? roll_direction
                           : lift_down         ? (bank_rad >= 0.0 ? 1 : -1)
                                               : (delta > 0.0 ? 1 : -1);
        if ((delta > 0.0) != (wanted > 0)) delta += wanted * 2.0 * constants::pi;
    }
    roll_direction = std::abs(delta) > settled_rad ? (delta > 0.0 ? 1 : -1) : 0;
    return delta;
}

}  // namespace

double bank_step_held(double bank_rad, double command_rad, double max_change_rad, int& roll_direction) {
    const double delta = held_delta(bank_rad, command_rad, roll_direction);
    return std::remainder(bank_rad + std::clamp(delta, -max_change_rad, max_change_rad), 2.0 * constants::pi);
}

double bank_step_smooth(double bank_rad, double command_rad, double max_rate_radps, double max_acceleration_radps2,
                        double dt_s, int& roll_direction, double& bank_rate_radps) {
    if (!(max_acceleration_radps2 > 0.0) || !(dt_s > 0.0)) {
        const double next = bank_step_held(bank_rad, command_rad, max_rate_radps * dt_s, roll_direction);
        bank_rate_radps = dt_s > 0.0 ? std::remainder(next - bank_rad, 2.0 * constants::pi) / dt_s : 0.0;
        return next;
    }
    const double delta = held_delta(bank_rad, command_rad, roll_direction);
    // the fastest rate that still stops on the command braking at the limit in steps of dt (v dt + (v - dv) dt + ...
    // <= |delta|), within the rate limit and without passing it in this step
    const double dv = max_acceleration_radps2 * dt_s;
    const double stopping = -0.5 * dv + std::sqrt(0.25 * dv * dv + 2.0 * max_acceleration_radps2 * std::abs(delta));
    const double wanted = std::copysign(std::min({max_rate_radps, stopping, std::abs(delta) / dt_s}), delta);
    bank_rate_radps = std::clamp(wanted, bank_rate_radps - dv, bank_rate_radps + dv);
    return std::remainder(bank_rad + bank_rate_radps * dt_s, 2.0 * constants::pi);
}

EntryPrediction predict_entry(const EntryPredictorModel& model, const EntryPredictorState& start,
                              double bank_command_rad, const EntryTarget& target, double gate_height_m,
                              double heat_load_so_far_j_m2, double step_s, double max_time_s) {
    const double gate_m = target.ground_height_m + gate_height_m;
    Model m(model, start, target.nose_direction_itrf, gate_m);
    EntryPrediction p;
    p.heat_load_j_m2 = heat_load_so_far_j_m2;
    Vec3 r = start.position_itrf_m, v = start.velocity_itrf_mps;
    double bank = start.bank_rad;
    double bank_rate = start.bank_rate_radps;
    int roll_direction = start.roll_direction;
    bool entered = false;
    double t = 0.0;
    double swept_rad = 0.0;  // ground track angle flown (unwraps the range past the antipode)
    AeroSample now = m.aero(r, v, bank);
    // bank profile (final_bank_rad): magnitude linear in energy from the command now to the final bank at the gate
    const bool profiled = model.final_bank_rad >= 0.0;
    const auto energy = [&](const Vec3& rr, const Vec3& vv) { return 0.5 * dot(vv, vv) - model.gm_m3ps2 / norm(rr); };
    const double energy_start = energy(r, v);
    const double energy_gate = -model.gm_m3ps2 / (model.earth_radius_m + gate_m);
    const double command_sign = std::copysign(1.0, bank_command_rad);
    while (t < max_time_s) {
        m.update_basis(r, v);
        m.advance(r, step_s);
        const double max_change = model.bank_rate_at(now.mach) * step_s;
        const bool reversed = start.reversal_time_s >= 0.0 && t >= start.reversal_time_s;
        double commanded = bank_command_rad;
        if (profiled && energy_start > energy_gate) {
            const double f = std::clamp((energy_start - energy(r, v)) / (energy_start - energy_gate), 0.0, 1.0);
            const double magnitude = std::abs(bank_command_rad);
            commanded = command_sign * (magnitude + (model.final_bank_rad - magnitude) * f);
        }
        const double command = reversed ? -commanded : commanded;
        const double next_bank = profiled ? bank_step_smooth(bank, command, model.bank_rate_at(now.mach),
                                                             model.bank_acceleration_radps2, step_s, roll_direction,
                                                             bank_rate)
                                          : bank_step(bank, command, max_change, false);
        const double mid_bank = bank + 0.5 * std::remainder(next_bank - bank, 2.0 * constants::pi);  // across 180 deg
        const Derivative k1 = m.derivative(r, v, bank);
        const Derivative k2 = m.derivative(r + k1.dr * (0.5 * step_s), v + k1.dv * (0.5 * step_s), mid_bank);
        const Derivative k3 = m.derivative(r + k2.dr * (0.5 * step_s), v + k2.dv * (0.5 * step_s), mid_bank);
        const Derivative k4 = m.derivative(r + k3.dr * step_s, v + k3.dv * step_s, next_bank);
        const Vec3 r1 = r + (k1.dr + k2.dr * 2.0 + k3.dr * 2.0 + k4.dr) * (step_s / 6.0);
        const Vec3 v1 = v + (k1.dv + k2.dv * 2.0 + k3.dv * 2.0 + k4.dv) * (step_s / 6.0);
        const AeroSample next = m.aero(r1, v1, next_bank);
        p.heat_load_j_m2 += 0.5 * (now.heat_rate_w_m2 + next.heat_rate_w_m2) * step_s;
        p.peak_heat_rate_w_m2 = std::max(p.peak_heat_rate_w_m2, next.heat_rate_w_m2);
        p.peak_load_g = std::max(p.peak_load_g, norm(next.acceleration) / constants::standard_gravity_mps2);
        p.peak_dynamic_pressure_pa = std::max(p.peak_dynamic_pressure_pa, next.dynamic_pressure_pa);
        if (next.height_m <= gate_m) {  // interpolate to the gate height
            const double f = std::clamp((now.height_m - gate_m) / (now.height_m - next.height_m), 0.0, 1.0);
            const Vec3 r_gate = r + (r1 - r) * f;
            swept_rad += angle_between(r, r_gate);
            r = r_gate;
            v = v + (v1 - v) * f;
            t += f * step_s;
            p.reached_gate = true;
            break;
        }
        swept_rad += angle_between(r, r1);
        r = r1;
        v = v1;
        bank = next_bank;
        now = next;
        t += step_s;
        if (now.height_m < entry_height_m) entered = true;
        if (entered && now.height_m > skip_out_height_m) {
            const Exit e = exit_orbit(model, r, v);
            if (model.carry_skips && e.apoapsis_height_m > 0.0 && e.apoapsis_height_m < model.max_skip_apoapsis_height_m) {
                entered = false;  // carried: the arc comes back down
                ++p.skips;
                continue;
            }
            p.exited = true;
            p.exit_apoapsis_height_m = e.apoapsis_height_m;
            p.exit_periapsis_height_m = e.periapsis_height_m;
            p.exit_inclination_rad = e.inclination_rad;
            break;
        }
    }
    p.end_position_itrf_m = r;
    p.end_velocity_itrf_mps = v;
    p.time_s = t;
    // Far from the target: difference of ranges from the start (monotone in the bank; the end's unwrapped past the
    // antipode, the target's within half a turn) and the target's distance
    // from the great circle start -> end. Close to it a range difference cannot tell a target behind the end from
    // one ahead: there the miss in the end's horizontal plane, along / across the track at the end (its velocity's
    // horizontal direction; the direction from the start when the fall is vertical).
    if (ground_range_m(start.position_itrf_m, target.position_itrf_m) > near_target_range_m) {
        const double end_range_m =
            unwrapped_range_rad(ground_range_m(start.position_itrf_m, r) / constants::wgs84_semi_major_axis_m, swept_rad) *
            constants::wgs84_semi_major_axis_m;
        // the target's range along the start's direction of motion: one behind the start is a turn less its distance
        // ahead (l5 Task 10: the NRHO return's planner took an interface past the site for one before it)
        double target_range_m = ground_range_m(start.position_itrf_m, target.position_itrf_m);
        const Vec3 start_track = start.velocity_itrf_mps - math::normalized(start.position_itrf_m) *
                                                              dot(start.velocity_itrf_mps, math::normalized(start.position_itrf_m));
        if (dot(target.position_itrf_m, start_track) < 0.0) {
            target_range_m = 2.0 * constants::pi * constants::wgs84_semi_major_axis_m - target_range_m;
        }
        p.downrange_error_m = end_range_m - target_range_m;
        const Vec3 n = cross(start.position_itrf_m, r);
        if (norm(n) > 1e-6 * norm(r) * norm(r)) {
            p.crossrange_m = -std::asin(std::clamp(dot(math::normalized(target.position_itrf_m), math::normalized(n)),
                                                   -1.0, 1.0)) *
                             constants::wgs84_semi_major_axis_m;
        }
        return p;
    }
    const Vec3 up = math::normalized(r);
    const auto horizontal = [&](const Vec3& x) { return x - up * dot(x, up); };
    Vec3 track = horizontal(v);
    if (norm(track) < min_track_speed_mps) track = horizontal(r - start.position_itrf_m);
    if (norm(track) > 0.0) {
        track = math::normalized(track);
        const Vec3 miss = horizontal(r - target.position_itrf_m);
        p.downrange_error_m = dot(miss, track);
        p.crossrange_m = -dot(miss, cross(track, up));
    }
    return p;
}

}  // namespace starsim::gnc
