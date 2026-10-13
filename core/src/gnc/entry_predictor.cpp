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
    Model(const EntryPredictorModel& m, bool steered, const Vec3& steering)
        : m_(m), steered_(steered), steering_(steering) {}

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
            const double fraction = std::min(norm(steering_), 1.0);
            const double alpha = m_.steering_alpha(fraction);
            s.acceleration = u * (-q_over_m * m_.drag_scale * m_.aero.cd_area_m2(mach, alpha));
            const Vec3 lb = steering_ - u * dot(steering_, u);
            if (fraction > 0.0 && norm(lb) > 1e-9) {
                s.acceleration += lb / norm(lb) * (q_over_m * m_.lift_scale * m_.aero.cl_area_m2(mach, alpha));
            }
        } else {
            s.acceleration = u * (-q_over_m * m_.drag_scale * m_.aero.cd_area_m2(mach)) +
                             l * (q_over_m * m_.lift_scale * m_.aero.cl_area_m2(mach));
        }
        if (m_.nose_radius_m > 0.0) {
            s.heat_rate_w_m2 = aero::stagnation_heat_rate_w_m2(air.density_kg_m3, speed, m_.nose_radius_m);
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
    environment::Us76Atmosphere atmosphere_;
};

}  // namespace

double ground_range_m(const Vec3& a_itrf_m, const Vec3& b_itrf_m) {
    const Vec3 a = math::normalized(a_itrf_m), b = math::normalized(b_itrf_m);
    return std::atan2(norm(cross(a, b)), dot(a, b)) * constants::wgs84_semi_major_axis_m;
}

EntryPrediction predict_entry(const EntryPredictorModel& model, const EntryPredictorState& start,
                              double bank_command_rad, const EntryTarget& target, double gate_height_m,
                              double heat_load_so_far_j_m2, double step_s, double max_time_s) {
    Model m(model, start.steered, start.steering_itrf);
    EntryPrediction p;
    p.heat_load_j_m2 = heat_load_so_far_j_m2;
    Vec3 r = start.position_itrf_m, v = start.velocity_itrf_mps;
    double bank = start.bank_rad;
    const double gate_m = target.ground_height_m + gate_height_m;
    bool entered = false;
    double t = 0.0;
    AeroSample now = m.aero(r, v, bank);
    while (t < max_time_s) {
        const double max_change = model.bank_rate_at(now.mach) * step_s;
        const bool reversed = start.reversal_time_s >= 0.0 && t >= start.reversal_time_s;
        const double command = reversed ? -bank_command_rad : bank_command_rad;
        const double next_bank = bank + std::clamp(command - bank, -max_change, max_change);
        const double mid_bank = 0.5 * (bank + next_bank);
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
            r = r + (r1 - r) * f;
            v = v + (v1 - v) * f;
            t += f * step_s;
            p.reached_gate = true;
            break;
        }
        r = r1;
        v = v1;
        bank = next_bank;
        now = next;
        t += step_s;
        if (now.height_m < entry_height_m) entered = true;
        if (entered && now.height_m > skip_out_height_m) break;
    }
    p.end_position_itrf_m = r;
    p.end_velocity_itrf_mps = v;
    p.time_s = t;
    // Far from the target: difference of ranges from the start (monotone in the bank) and the target's distance
    // from the great circle start -> end. Close to it a range difference cannot tell a target behind the end from
    // one ahead: there the miss in the end's horizontal plane, along / across the track at the end (its velocity's
    // horizontal direction; the direction from the start when the fall is vertical).
    if (ground_range_m(start.position_itrf_m, target.position_itrf_m) > near_target_range_m) {
        p.downrange_error_m =
            ground_range_m(start.position_itrf_m, r) - ground_range_m(start.position_itrf_m, target.position_itrf_m);
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
