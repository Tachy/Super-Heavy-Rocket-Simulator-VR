#include "starsim/gnc/entry_corridor.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

namespace {

constexpr double steepest_rad = -15.0 * constants::deg_to_rad;
constexpr double shallowest_rad = -0.5 * constants::deg_to_rad;
constexpr double tolerance_rad = 0.01 * constants::deg_to_rad;
constexpr double speed_tolerance_mps = 25.0;
constexpr double max_prediction_s = 6000.0;

// Earth-fixed interface state on the meridian of longitude 0 from speed, angle and heading.
EntryPredictorState interface_state(const EntryPredictorModel& m, const CorridorQuery& q, double gamma_rad,
                                    double bank_rad) {
    const double r = m.earth_radius_m + q.interface_height_m;
    const math::Vec3 up{std::cos(q.latitude_rad), 0.0, std::sin(q.latitude_rad)};
    const math::Vec3 east{0.0, 1.0, 0.0};
    const math::Vec3 north = cross(up, east);
    const math::Vec3 horizontal = north * std::cos(q.heading_rad) + east * std::sin(q.heading_rad);
    const math::Vec3 v = (horizontal * std::cos(gamma_rad) + up * std::sin(gamma_rad)) * q.speed_mps;
    EntryPredictorState s;
    s.position_itrf_m = up * r;
    s.velocity_itrf_mps = v;
    s.bank_rad = bank_rad;
    return s;
}

EntryPrediction fly(const EntryPredictorModel& m, const CorridorQuery& q, double gamma_rad, double bank_rad) {
    return predict_entry(m, interface_state(m, q, gamma_rad, bank_rad), bank_rad, EntryTarget{}, q.gate_height_m, 0.0,
                         1.0, max_prediction_s);
}

bool within_limits(const EntryPrediction& p, const CorridorQuery& q, std::string& limiting) {
    const auto& l = q.limits;
    if (l.max_heat_rate_w_m2 > 0.0 && p.peak_heat_rate_w_m2 > q.constraint_fraction * l.max_heat_rate_w_m2) {
        limiting = "Hitzestrom";
        return false;
    }
    if (l.max_heat_load_j_m2 > 0.0 && p.heat_load_j_m2 > q.constraint_fraction * l.max_heat_load_j_m2) {
        limiting = "Wärmelast";
        return false;
    }
    if (l.max_load_g > 0.0 && p.peak_load_g > q.constraint_fraction * l.max_load_g) {
        limiting = "Last";
        return false;
    }
    return true;
}

// Bisection for the boundary of a predicate that holds at `good` and fails at `bad`.
template <class F>
double boundary(double good, double bad, F holds) {
    while (std::abs(good - bad) > tolerance_rad) {
        const double mid = 0.5 * (good + bad);
        (holds(mid) ? good : bad) = mid;
    }
    return good;
}

}  // namespace

EntryCorridor entry_corridor(EntryPredictorModel model, const CorridorQuery& query) {
    model.carry_skips = false;  // a corridor boundary is about the first pass
    const bool capture = query.aerocapture_apoapsis_m > 0.0;
    constexpr double lift_up = 0.0;
    constexpr double lift_down = constants::pi;
    // Shallow side: lift down still captures (direct: reaches the gate; aerocapture: or exits at or below the
    // apoapsis). A pass that never dips below 80 km neither exits nor reaches the gate: not captured.
    const auto captures = [&](double gamma) {
        const auto p = fly(model, query, gamma, lift_down);
        if (p.reached_gate) return true;
        return capture && p.exited && p.exit_apoapsis_height_m >= 0.0 &&
               p.exit_apoapsis_height_m <= query.aerocapture_apoapsis_m;
    };
    // Steep side: lift up stays within the limits (aerocapture: and still exits at or above the apoapsis).
    std::string limiting;
    const auto admissible = [&](double gamma) {
        const auto p = fly(model, query, gamma, lift_up);
        if (!within_limits(p, query, limiting)) return false;
        // A pass that never dips below 80 km keeps its orbit: too shallow, i.e. high enough.
        const bool never_dipped = !p.exited && !p.reached_gate;
        const bool high_exit = never_dipped || (p.exited && (p.exit_apoapsis_height_m < 0.0 ||
                                                             p.exit_apoapsis_height_m >= query.aerocapture_apoapsis_m));
        if (capture && !high_exit) {
            limiting = "Austritt";
            return false;
        }
        return true;
    };
    EntryCorridor c;
    const bool shallowest_admissible = admissible(shallowest_rad);
    if (!shallowest_admissible || !captures(steepest_rad)) {
        c.limiting = shallowest_admissible ? "Kein Einfang" : limiting;
        return c;  // not even the shallowest entry stays within the limits, or no capture even steeply
    }
    c.shallow_rad = captures(shallowest_rad) ? shallowest_rad : boundary(steepest_rad, shallowest_rad, captures);
    c.steep_rad = admissible(steepest_rad) ? steepest_rad : boundary(shallowest_rad, steepest_rad, admissible);
    // the limiting cause just beyond the steep boundary
    admissible(c.steep_rad - 2.0 * tolerance_rad);
    c.limiting = limiting;
    // A corridor narrower than the bisection resolution (each boundary within one tolerance) is not resolved.
    c.exists = c.shallow_rad - c.steep_rad > 2.0 * tolerance_rad;
    c.width_rad = std::max(0.0, c.shallow_rad - c.steep_rad);
    c.middle_rad = 0.5 * (c.shallow_rad + c.steep_rad);
    return c;
}

double max_entry_speed(const EntryPredictorModel& model, CorridorQuery query, double min_width_rad, double low_mps,
                       double high_mps) {
    const auto wide_enough = [&](double v) {
        query.speed_mps = v;
        const auto c = entry_corridor(model, query);
        return c.exists && c.width_rad >= min_width_rad;
    };
    if (!wide_enough(low_mps)) return low_mps - 1.0;
    if (wide_enough(high_mps)) return high_mps;
    double good = low_mps, bad = high_mps;
    while (bad - good > speed_tolerance_mps) {
        const double mid = 0.5 * (good + bad);
        (wide_enough(mid) ? good : bad) = mid;
    }
    return good;
}

}  // namespace starsim::gnc
