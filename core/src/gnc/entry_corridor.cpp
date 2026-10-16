#include "starsim/gnc/entry_corridor.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/frames/geodetic.hpp"

namespace starsim::gnc {

namespace {

constexpr double steepest_rad = -15.0 * constants::deg_to_rad;
constexpr double shallowest_rad = -0.5 * constants::deg_to_rad;
constexpr double tolerance_rad = 0.01 * constants::deg_to_rad;
constexpr double speed_tolerance_mps = 25.0;
constexpr double max_prediction_s = 6000.0;

// Earth-fixed interface state on the meridian of longitude 0 from speed, angle and heading: at the geodetic interface
// height (the predictor judges geodetic heights; on a sphere of the equatorial radius the point at 26 deg sat ~4 km
// higher, Task 8), the angle against the geocentric horizontal (as EarthReturnPlanner measures it).
EntryPredictorState interface_state(const CorridorQuery& q, double gamma_rad, double bank_rad) {
    const math::Vec3 position = frames::geodetic_to_fixed({q.latitude_rad, 0.0, q.interface_height_m});
    const math::Vec3 up = math::normalized(position);
    const math::Vec3 east{0.0, 1.0, 0.0};
    const math::Vec3 north = cross(up, east);
    const math::Vec3 horizontal = north * std::cos(q.heading_rad) + east * std::sin(q.heading_rad);
    const math::Vec3 v = (horizontal * std::cos(gamma_rad) + up * std::sin(gamma_rad)) * q.speed_mps;
    EntryPredictorState s;
    s.position_itrf_m = position;
    s.velocity_itrf_mps = v;
    s.bank_rad = bank_rad;
    return s;
}

EntryPrediction fly(const EntryPredictorModel& m, const CorridorQuery& q, double gamma_rad, double bank_rad) {
    return predict_entry(m, interface_state(q, gamma_rad, bank_rad), bank_rad, EntryTarget{}, q.gate_height_m, 0.0,
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
    model.carry_skips = false;     // a corridor boundary is about the first pass
    model.final_bank_rad = -1.0;   // at constant full lift down / up (the classical corridor)
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

BankWindow entry_bank_window(const EntryPredictorModel& model, const CorridorQuery& query, double flight_path_rad,
                             double max_bank_rad) {
    std::string limiting;
    const bool capture = query.aerocapture_apoapsis_m > 0.0;
    const auto ok = [&](double bank_rad) {
        const auto p = fly(model, query, flight_path_rad, bank_rad);
        if (p.skips != 0) return false;
        if (capture && p.reached_gate) {
            // stays in: more bank than the apoapsis needs - the guidance's room, judged on the pass's heat rate only
            // (the load and heat load of the dive to the gate that follows are never flown: the bank comes back)
            const double max_rate = query.limits.max_heat_rate_w_m2;
            return max_rate <= 0.0 || p.peak_heat_rate_w_m2 <= query.constraint_fraction * max_rate;
        }
        if (!within_limits(p, query, limiting)) return false;
        if (p.reached_gate) return true;  // a direct entry, no skip
        return capture && p.exited && p.exit_apoapsis_height_m >= 0.0 &&
               p.exit_apoapsis_height_m <= query.aerocapture_apoapsis_m;
    };
    // the longest run of admissible banks on a 2 deg grid, its edges refined by bisection
    constexpr double grid_rad = 2.0 * constants::deg_to_rad;
    const int n = static_cast<int>(max_bank_rad / grid_rad);
    int best_first = -1, best_last = -1, first = -1;
    for (int i = 0; i <= n + 1; ++i) {
        const bool good = i <= n && ok(i * grid_rad);
        if (good && first < 0) first = i;
        if (!good && first >= 0) {
            if (i - 1 - first > best_last - best_first) {
                best_first = first;
                best_last = i - 1;
            }
            first = -1;
        }
    }
    BankWindow w;
    w.flight_path_rad = flight_path_rad;
    if (best_first < 0) return w;
    constexpr double edge_tolerance_rad = 0.25 * constants::deg_to_rad;
    const auto edge = [&](double good, double bad) {
        while (std::abs(good - bad) > edge_tolerance_rad) {
            const double mid = 0.5 * (good + bad);
            (ok(mid) ? good : bad) = mid;
        }
        return good;
    };
    w.low_rad = best_first > 0 ? edge(best_first * grid_rad, (best_first - 1) * grid_rad) : 0.0;
    w.high_rad = best_last < n ? edge(best_last * grid_rad, (best_last + 1) * grid_rad) : best_last * grid_rad;
    w.width_rad = w.high_rad - w.low_rad;
    w.middle_rad = 0.5 * (w.low_rad + w.high_rad);
    w.exists = true;
    return w;
}

BankWindow widest_bank_window(const EntryPredictorModel& model, const CorridorQuery& query, const EntryCorridor& corridor,
                              double max_bank_rad) {
    constexpr double step_rad = 0.1 * constants::deg_to_rad;
    BankWindow best;
    for (double gamma = corridor.steep_rad; gamma <= corridor.shallow_rad + 1e-12; gamma += step_rad) {
        const auto w = entry_bank_window(model, query, gamma, max_bank_rad);
        if (w.exists && (!best.exists || w.width_rad > best.width_rad)) best = w;
    }
    return best;
}

double max_entry_speed(const EntryPredictorModel& model, CorridorQuery query, double min_width_rad, double low_mps,
                       double high_mps) {
    const auto wide_enough = [&](double v) {
        query.speed_mps = v;
        const auto c = entry_corridor(model, query);
        if (!c.exists || c.width_rad < min_width_rad) return false;
        if (query.min_bank_window_rad <= 0.0) return true;
        const auto w = widest_bank_window(model, query, c, query.max_bank_rad);
        return w.exists && w.width_rad >= query.min_bank_window_rad;
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
