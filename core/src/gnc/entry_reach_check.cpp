#include "starsim/gnc/entry_reach_check.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "starsim/gnc/entry_guidance.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double bank_tolerance_m = 1000.0;  // mean downrange miss that counts as solved (twice the corrector's 500 m)
constexpr int bank_iterations = 8;
constexpr int reversal_iterations = 12;      // bisection of the reversal time: window / 4096 (~0.3 s)
constexpr double bank_probe_rad = 5.0 * constants::deg_to_rad;
constexpr double outside = -std::numeric_limits<double>::infinity();

struct Flight {
    const EntryPredictorModel& model;
    const EntryPredictorState& entry;
    const EntryTarget& target;
    double gate_height_m;
    double step_s;

    // `sign` x `bank` from the entry interface, reversed at `reversal_time_s` (< 0: held to the gate).
    EntryPrediction fly(double sign, double bank, double reversal_time_s) const {
        EntryPredictorState s = entry;
        s.bank_rad = sign * bank;
        s.reversal_time_s = reversal_time_s;
        return predict_entry(model, s, s.bank_rad, target, gate_height_m, 0.0, step_s);
    }

    // Mean downrange miss of the two signs held (the planner's shot): what the bank magnitude solves for.
    double mean_downrange(double bank, bool& valid) const {
        const auto a = fly(1.0, bank, -1.0), b = fly(-1.0, bank, -1.0);
        valid = a.reached_gate && b.reached_gate;
        return 0.5 * (a.downrange_error_m + b.downrange_error_m);
    }

    // Bank magnitude for the range (secant from `bank`; more bank, less range), within lift up.
    double solve_bank(double bank) const {
        bool valid = false;
        double b0 = bank, f0 = mean_downrange(b0, valid);
        if (!valid || std::abs(f0) < bank_tolerance_m) return b0;
        double b1 = std::clamp(b0 + (f0 > 0.0 ? bank_probe_rad : -bank_probe_rad), 0.0, 0.5 * constants::pi);
        for (int i = 0; i < bank_iterations; ++i) {
            const double f1 = mean_downrange(b1, valid);
            if (!valid || std::abs(f1) < bank_tolerance_m || f1 == f0) break;
            const double b2 = std::clamp(b1 - f1 * (b1 - b0) / (f1 - f0), 0.0, 0.5 * constants::pi);
            b0 = b1;
            f0 = f1;
            b1 = b2;
        }
        return b1;
    }
};

struct Bracket {
    double margin_m = outside;
    double sign = 1.0;
    double handover_time_s = 0.0;
    double crossrange_m = 0.0;  // mean of the two signs held
};

// The crossrange interval the guidance's single reversal covers at bank magnitude `bank`: from reversing at once (the
// other sign held) to reversing at the hand-over; the margin is how far the target lies inside it (< 0: outside).
Bracket bracket(const Flight& f, double bank, double deadband_m) {
    Bracket b;
    const auto plus = f.fly(1.0, bank, -1.0), minus = f.fly(-1.0, bank, -1.0);
    if (!plus.reached_gate || !minus.reached_gate) return b;
    b.crossrange_m = 0.5 * (plus.crossrange_m + minus.crossrange_m);
    b.sign = std::abs(plus.crossrange_m) <= std::abs(minus.crossrange_m) ? 1.0 : -1.0;  // as EntryGuidance::entry_cycle
    const auto& held = b.sign > 0.0 ? plus : minus;
    b.handover_time_s = held.mark_time_s >= 0.0 ? held.mark_time_s : held.time_s;
    const auto at_once = f.fly(b.sign, bank, 0.0);
    const auto at_handover = f.fly(b.sign, bank, b.handover_time_s);
    if (!at_once.reached_gate || !at_handover.reached_gate) return b;
    const double a = at_once.crossrange_m, c = at_handover.crossrange_m;
    const double nearer = std::min(std::abs(a), std::abs(c));
    b.margin_m = ((a > 0.0) != (c > 0.0) ? nearer : -nearer) - deadband_m;
    return b;
}

}  // namespace

EntryReachCheck entry_reach_check(const EntryGuidanceParameters& p, bool capsule) {
    EntryReachCheck check;
    check.handover_mach = p.terminal_phase ? p.terminal_mach : 0.0;
    check.min_reversal_speed_mps = p.min_reversal_speed_mps;
    check.reversal_deadband_m = p.reversal_crossrange_base_m;
    check.max_dynamic_pressure_pa = p.max_dynamic_pressure_pa;
    check.constraint_fraction = p.constraint_fraction;
    if (capsule) {  // [E] the capsule campaign's (test_capsule_entry.cpp): +-0.1 deg, mass +-3 %, aero +-10 %
        check.dispersion_flight_path_rad = 0.1 * constants::deg_to_rad;
        check.dispersion_speed_mps = 0.0;
        check.dispersion_aero_fraction = 0.1;
        check.dispersion_mass_fraction = 0.03;
    }
    return check;
}

EntryReachCheck entry_reach_check(const EntryGuidance& guidance) {
    EntryReachCheck check = entry_reach_check(guidance.parameters(), guidance.capsule());
    check.max_heat_rate_w_m2 = guidance.limits().max_heat_rate_w_m2;
    check.max_load_g = guidance.limits().max_load_g;
    return check;
}

EntryReachAssessment assess_entry_reach(const EntryPredictorModel& model, const EntryPredictorState& entry, double bank_rad,
                                        const EntryTarget& target, double gate_height_m, const EntryReachCheck& check) {
    EntryReachAssessment result;
    EntryPredictorModel marked = model;
    marked.mark_mach = check.handover_mach;
    marked.mark_speed_mps = check.min_reversal_speed_mps;

    // nominal: the planner's bank
    const Flight nominal{marked, entry, target, gate_height_m, check.step_s};
    const Bracket b = bracket(nominal, bank_rad, check.reversal_deadband_m);
    result.crossrange_m = b.crossrange_m;
    result.bank_rad = bank_rad;
    result.first_sign = b.sign;
    result.handover_time_s = b.handover_time_s;
    result.nominal_margin_m = b.margin_m;
    double smallest = b.margin_m;

    // dispersed, one at a time: steeper, slower, aerodynamics and mass up and down - each flies its own bank for the range
    const Vec3 up = math::normalized(entry.position_itrf_m);
    const double speed = norm(entry.velocity_itrf_mps);
    const Vec3 u = entry.velocity_itrf_mps / speed;
    Vec3 down_normal = u * dot(up, u) - up;  // perpendicular to u, towards down
    down_normal = norm(down_normal) > 1e-9 ? math::normalized(down_normal) : -up;
    const auto disperse = [&](const EntryPredictorModel& m, const EntryPredictorState& s, const char* name) {
        const Flight f{m, s, target, gate_height_m, check.step_s};
        const double margin = bracket(f, f.solve_bank(bank_rad), check.reversal_deadband_m).margin_m;
        if (margin < smallest) {
            smallest = margin;
            result.limiting_case = name;
        }
    };
    if (check.dispersion_flight_path_rad > 0.0) {
        EntryPredictorState steeper = entry;
        const double dg = check.dispersion_flight_path_rad;
        steeper.velocity_itrf_mps = (u * std::cos(dg) + down_normal * std::sin(dg)) * speed;
        disperse(marked, steeper, "steeper");
    }
    if (check.dispersion_speed_mps > 0.0) {
        EntryPredictorState slower = entry;
        slower.velocity_itrf_mps = u * (speed - check.dispersion_speed_mps);
        disperse(marked, slower, "slower");
    }
    for (const double sign : {-1.0, 1.0}) {
        if (check.dispersion_aero_fraction > 0.0) {
            EntryPredictorModel m = marked;
            m.lift_scale *= 1.0 + sign * check.dispersion_aero_fraction;
            m.drag_scale *= 1.0 + sign * check.dispersion_aero_fraction;
            disperse(m, entry, sign < 0.0 ? "aero low" : "aero high");
        }
        if (check.dispersion_mass_fraction > 0.0) {
            EntryPredictorModel m = marked;
            m.mass_kg *= 1.0 + sign * check.dispersion_mass_fraction;
            disperse(m, entry, sign < 0.0 ? "mass low" : "mass high");
        }
    }
    result.bracketed = smallest >= 0.0;
    result.reach_m = std::isfinite(smallest) ? std::max(0.0, std::abs(b.crossrange_m) + smallest) : 0.0;
    result.dispersion_reserve_m = std::isfinite(smallest) ? b.margin_m - smallest : 0.0;
    if (!std::isfinite(b.margin_m)) return result;

    // the planned entry: the reversal that ends on the target (bisection between at once and the hand-over), when
    // the target is inside the window; otherwise the end nearer to it
    double lo = 0.0, hi = b.handover_time_s;
    EntryPrediction planned = nominal.fly(b.sign, bank_rad, lo);
    const double c_lo = planned.crossrange_m;
    if (b.margin_m + check.reversal_deadband_m >= 0.0) {
        for (int i = 0; i < reversal_iterations; ++i) {
            const double mid = 0.5 * (lo + hi);
            planned = nominal.fly(b.sign, bank_rad, mid);
            ((planned.crossrange_m > 0.0) == (c_lo > 0.0) ? lo : hi) = mid;
        }
        result.reversal_time_s = 0.5 * (lo + hi);
        planned = nominal.fly(b.sign, bank_rad, result.reversal_time_s);
        // the speed at the reversal: the prediction stopped there
        EntryPredictorState s = entry;
        s.bank_rad = b.sign * bank_rad;
        const auto to_reversal = predict_entry(marked, s, s.bank_rad, target, gate_height_m, 0.0, check.step_s,
                                               std::max(result.reversal_time_s, check.step_s));
        result.reversal_speed_mps = norm(to_reversal.end_velocity_itrf_mps);
    }
    result.peak_heat_rate_w_m2 = planned.peak_heat_rate_w_m2;
    result.peak_load_g = planned.peak_load_g;
    result.peak_dynamic_pressure_pa = planned.peak_dynamic_pressure_pa;
    const double f = check.constraint_fraction;
    result.within_corridor =
        planned.reached_gate &&
        !(check.max_heat_rate_w_m2 > 0.0 && planned.peak_heat_rate_w_m2 > f * check.max_heat_rate_w_m2) &&
        !(check.max_load_g > 0.0 && planned.peak_load_g > f * check.max_load_g) &&
        !(check.max_dynamic_pressure_pa > 0.0 && planned.peak_dynamic_pressure_pa > check.max_dynamic_pressure_pa);
    return result;
}

}  // namespace starsim::gnc
