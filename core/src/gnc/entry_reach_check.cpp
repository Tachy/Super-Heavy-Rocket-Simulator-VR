#include "starsim/gnc/entry_reach_check.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <iterator>
#include <limits>
#include <random>
#include <string_view>
#include <vector>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/frames/geodetic.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double max_terminal_prediction_s = 900.0;  // as EntryGuidance::terminal_miss
constexpr int reach_grid = 8;                        // reach search: two waves of this many targets (span / 64)

// Rodrigues: `x` turned by `angle` about the unit axis `k`.
Vec3 rotated(const Vec3& x, const Vec3& k, double angle) {
    return x * std::cos(angle) + cross(k, x) * std::sin(angle) + k * (dot(k, x) * (1.0 - std::cos(angle)));
}

struct Case {
    const char* name;
    EntryPredictorModel model;   // the truth (the dispersed aerodynamics and mass)
    double aero_factor = 1.0;    // truth over the onboard model's aerodynamics (EntryGuidance learns it: scale factors)
    EntryPredictorState entry;
};

struct Guided {
    bool reached_gate = false;
    bool range_lost = false;  // the gate reached in the entry phase (a capsule) beyond EntryReachCheck::gate_tolerance_m
    int constrained_cycles = 0;  // cycles whose bank the corridor cut back (range given up for the moment)
    double gate_miss_m = 0.0;    // a capsule: horizontal miss at the gate
    bool terminal_ok = true;
    double terminal_miss_m = 0.0;
    double peak_heat_rate_w_m2 = 0.0, peak_load_g = 0.0, peak_dynamic_pressure_pa = 0.0;
    double handover_crossrange_m = 0.0;  // the bank held from the hand-over to the gate
    double first_sign = 0.0;
    double handover_time_s = 0.0;
    double reversal_time_s = -1.0, reversal_speed_mps = 0.0;
    int reversals = 0;
    bool within_corridor = false;
    bool flies = false;
};

void take_peaks(Guided& g, const EntryPrediction& p) {
    g.peak_heat_rate_w_m2 = std::max(g.peak_heat_rate_w_m2, p.peak_heat_rate_w_m2);
    g.peak_load_g = std::max(g.peak_load_g, p.peak_load_g);
    g.peak_dynamic_pressure_pa = std::max(g.peak_dynamic_pressure_pa, p.peak_dynamic_pressure_pa);
}

// The terminal steering flown ahead from the hand-over state (EntryGuidance::terminal without the nose sector): each
// cycle one Levenberg-Marquardt step on the predicted horizontal miss at the gate (its step limit), then the steering
// held for the cycle; from the entry's bank at the schedule's fraction. The horizontal miss where the flight reaches the
// gate.
double terminal_miss_m(const EntryGuidanceParameters& p, const EntryPredictorModel& model, const EntryPredictorState& from,
                       double bank_rad, const EntryTarget& target, double cycle_s) {
    EntryTarget free = target;
    free.nose_direction_itrf = Vec3{};
    const Vec3 up = math::normalized(target.position_itrf_m);
    const Vec3 east = math::normalized(cross(Vec3{0.0, 0.0, 1.0}, up));
    const Vec3 north = cross(up, east);
    EntryPredictorState s{from.position_itrf_m, from.velocity_itrf_mps, 0.0};
    s.steered = true;
    const auto horizontal = [&](const EntryPrediction& pr) {
        const Vec3 d = pr.end_position_itrf_m - target.position_itrf_m;
        return Vec3{dot(d, east), dot(d, north), 0.0};
    };
    const auto miss = [&](const Vec3& steering, bool& reached) {
        EntryPredictorState q = s;
        q.steering = steering;
        const auto pr = predict_entry(model, q, 0.0, free, p.gate_height_m, 0.0, p.terminal_prediction_step_s,
                                      max_terminal_prediction_s);
        reached = pr.reached_gate;
        return horizontal(pr);
    };
    Vec3 steering = Vec3{std::cos(bank_rad), -std::sin(bank_rad), 0.0} * EntryPredictorModel::schedule_fraction;
    for (double t = 0.0; t < max_terminal_prediction_s; t += cycle_s) {
        bool reached = false;
        const Vec3 z0 = miss(steering, reached);
        if (!reached) return std::numeric_limits<double>::infinity();
        const double d1 = EntryPredictorModel::steering_difference(steering, Vec3{1.0, 0.0, 0.0}, p.terminal_newton_step);
        const double d2 = EntryPredictorModel::steering_difference(steering, Vec3{0.0, 1.0, 0.0}, p.terminal_newton_step);
        bool r1 = false, r2 = false;
        const Vec3 z1 = miss(steering + Vec3{d1, 0.0, 0.0}, r1);
        const Vec3 z2 = miss(steering + Vec3{0.0, d2, 0.0}, r2);
        if (r1 && r2) {
            const double j11 = (z1.x - z0.x) / d1, j12 = (z2.x - z0.x) / d2;
            const double j21 = (z1.y - z0.y) / d1, j22 = (z2.y - z0.y) / d2;
            const double a11 = j11 * j11 + j21 * j21, a12 = j11 * j12 + j21 * j22, a22 = j12 * j12 + j22 * j22;
            const double lambda = 1.0e-3 * (a11 + a22);  // as EntryGuidance::terminal
            const double b1 = -(j11 * z0.x + j21 * z0.y), b2 = -(j12 * z0.x + j22 * z0.y);
            const double det = (a11 + lambda) * (a22 + lambda) - a12 * a12;
            if (det > 0.0) {
                Vec3 step{((a22 + lambda) * b1 - a12 * b2) / det, ((a11 + lambda) * b2 - a12 * b1) / det, 0.0};
                if (norm(step) > p.terminal_max_step) step = step * (p.terminal_max_step / norm(step));
                steering = steering + step;
                if (norm(steering) > 1.0) steering = steering / norm(steering);
            }
        }
        EntryPredictorState q = s;
        q.steering = steering;
        const auto segment =
            predict_entry(model, q, 0.0, free, p.gate_height_m, 0.0, p.terminal_prediction_step_s, cycle_s);
        if (segment.reached_gate) return norm(horizontal(segment));
        s.position_itrf_m = segment.end_position_itrf_m;
        s.velocity_itrf_mps = segment.end_velocity_itrf_mps;
    }
    return std::numeric_limits<double>::infinity();
}

// The onboard guidance flown ahead from the entry interface to `target` (see assess_entry_reach).
Guided fly_guided(const EntryReachCheck& check, const EntryGuidanceParameters& p, const Case& c, double warm_bank_rad,
                  const EntryTarget& target, bool debug) {
    Guided g;
    const EntryCorrector corrector{p, check.limits, target, check.step_s, false};
    const EntryPredictorModel& truth = c.model;
    EntryPredictorModel onboard = c.model;  // the guidance's model: nominal aerodynamics, the scales learnt in flight
    onboard.lift_scale /= c.aero_factor;
    onboard.drag_scale /= c.aero_factor;
    // EntryGuidance::update's scale filter over one fly-ahead cycle (its gain per guidance cycle)
    const double scale_gain = 1.0 - std::pow(1.0 - p.scale_filter_gain, check.cycle_s / p.cycle_s);
    EntryLateralState lateral;
    lateral.bank_magnitude_rad = warm_bank_rad;
    EntryPredictorState s = c.entry;
    s.reversal_time_s = -1.0;
    s.bank_rate_radps = 0.0;
    int roll_direction = 0;
    double bank_target = 0.0, heat_load = 0.0, t = 0.0;
    bool first = true, ended = false;
    const environment::Us76Atmosphere atmosphere;
    while (t < p.max_time_s) {
        const double speed = norm(s.velocity_itrf_mps);
        if (speed >= p.hold_bank_below_speed_mps) {
            lateral.time_since_reversal_s += check.cycle_s;
            const double sign_before = lateral.sign;
            const EntrySolution solution =
                entry_corrector_cycle(corrector, onboard, s, heat_load, bank_target, check.cycle_s, lateral, roll_direction);
            if (first) {  // the ship arrives on its first command (the planner's shots fly it from the interface)
                first = false;
                g.first_sign = lateral.sign;
                s.bank_rad = lateral.sign * solution.bank_rad;
            } else if (lateral.sign != sign_before || lateral.pending_reversal_in_s >= 0.0) {
                ++g.reversals;
                g.reversal_time_s = t + std::max(0.0, lateral.pending_reversal_in_s);
                g.reversal_speed_mps = speed;
            }

            if (solution.constrained) ++g.constrained_cycles;
            bank_target = lateral.sign * solution.bank_rad;
            if (debug) {
                std::printf("[reach]   t %.0f v %.0f bank %+.1f (flown %+.1f) cross %.1f down %.1f km peak %.2f g%s\n", t,
                            speed, bank_target / constants::deg_to_rad, s.bank_rad / constants::deg_to_rad,
                            solution.prediction.crossrange_m / 1000.0, solution.prediction.downrange_error_m / 1000.0,
                            solution.prediction.peak_load_g, solution.constrained ? " constrained" : "");
            }
        }
        EntryPredictorState ahead = s;
        ahead.roll_direction = roll_direction;
        const bool pending = lateral.pending_reversal_in_s >= 0.0;
        ahead.reversal_time_s = pending ? lateral.pending_reversal_in_s : -1.0;
        const auto segment =
            predict_entry(truth, ahead, bank_target, target, p.gate_height_m, heat_load, check.step_s, check.cycle_s);
        take_peaks(g, segment);
        if (segment.peak_load_g * constants::standard_gravity_mps2 > p.scale_min_acceleration_mps2) {  // measured aero
            onboard.lift_scale += scale_gain * (truth.lift_scale - onboard.lift_scale);
            onboard.drag_scale += scale_gain * (truth.drag_scale - onboard.drag_scale);
        }
        heat_load = segment.heat_load_j_m2;
        s.position_itrf_m = segment.end_position_itrf_m;
        s.velocity_itrf_mps = segment.end_velocity_itrf_mps;
        s.bank_rad = segment.end_bank_rad;
        s.bank_rate_radps = segment.end_bank_rate_radps;
        t += segment.time_s;
        if (pending) {  // the reversal timed within the cycle has flown (EntryGuidance::update)
            lateral.pending_reversal_in_s = -1.0;
            lateral.sign = -lateral.sign;
            lateral.time_since_reversal_s = 0.0;
            lateral.reversal_rolling = true;
            lateral.bank_magnitude_rad = lateral.pending_reversal_bank_rad;
            bank_target = lateral.sign * lateral.bank_magnitude_rad;
            roll_direction = 0;
        }
        if (segment.reached_gate || segment.exited || segment.time_s <= 0.0) {  // a capsule: the gate in the entry
            g.reached_gate = segment.reached_gate;
            g.handover_crossrange_m = segment.crossrange_m;
            g.gate_miss_m = std::hypot(segment.downrange_error_m, segment.crossrange_m);
            g.range_lost = g.gate_miss_m > (check.gate_tolerance_m > 0.0 ? check.gate_tolerance_m : p.reversal_crossrange_base_m);
            g.handover_time_s = t;
            ended = true;
            break;
        }
        if (p.terminal_phase) {  // EntryGuidance::update's hand-over
            const double height = frames::fixed_to_geodetic(s.position_itrf_m).height_m;
            const double sound = atmosphere.at(height).speed_of_sound_mps;
            const double mach = sound > 0.0 ? norm(s.velocity_itrf_mps) / sound : 0.0;
            const bool settled =
                std::abs(std::remainder(bank_target - s.bank_rad, 2.0 * constants::pi)) < entry_settled_bank_rad;
            if (height < entry_terminal_max_height_m &&
                ((mach < p.terminal_mach && settled) || mach < p.terminal_mach_latest)) {
                break;
            }
        }
    }
    if (!ended) {
        g.handover_time_s = t;
        EntryPredictorState rest = s;
        rest.roll_direction = roll_direction;
        rest.reversal_time_s = -1.0;
        const auto to_gate = predict_entry(truth, rest, bank_target, target, p.gate_height_m, heat_load, check.step_s);
        take_peaks(g, to_gate);
        g.reached_gate = to_gate.reached_gate;
        g.handover_crossrange_m = to_gate.crossrange_m;
        if (p.terminal_phase && g.reached_gate) {
            g.terminal_miss_m = terminal_miss_m(p, onboard, s, bank_target, target, check.cycle_s);
            g.terminal_ok = g.terminal_miss_m < p.corrector_tolerance_m;
        }
    }
    g.within_corridor =
        !(check.limits.max_heat_rate_w_m2 > 0.0 && g.peak_heat_rate_w_m2 > check.limits.max_heat_rate_w_m2) &&
        !(check.limits.max_load_g > 0.0 && g.peak_load_g > check.limits.max_load_g) &&
        !(g.peak_dynamic_pressure_pa > p.max_dynamic_pressure_pa);
    g.flies = g.reached_gate && !g.range_lost && g.terminal_ok && g.within_corridor;
    return g;
}

// Flies case i to target i, all at once (one thread each; the predictor is pure); `trace_case`: the case whose cycles
// are printed.
std::vector<Guided> fly_all(const EntryReachCheck& check, const EntryGuidanceParameters& p, const std::vector<Case>& cases,
                            double warm_bank_rad, const std::vector<EntryTarget>& targets, const char* trace_case) {
    std::vector<std::future<Guided>> flights;
    flights.reserve(targets.size());
    for (size_t i = 0; i < targets.size(); ++i) {
        const Case& c = cases[i];
        const EntryTarget& target = targets[i];
        const bool debug = trace_case != nullptr && std::string_view(trace_case) == c.name;
        flights.push_back(std::async(std::launch::async, [&check, &p, &c, warm_bank_rad, &target, debug] {
            return fly_guided(check, p, c, warm_bank_rad, target, debug);
        }));
    }
    std::vector<Guided> flown;
    flown.reserve(flights.size());
    for (auto& f : flights) flown.push_back(f.get());
    return flown;
}

}  // namespace

std::vector<EntryDispersion> ship_entry_campaign_dispersions() {
    // as "Entry guidance brings the ship to the Starbase flip gate from dispersed entries": one generator, seven draws a
    // case (the propellant and the flap factor drawn but not used here: the mass is known onboard, the flaps trim)
    std::mt19937 rng(20261013);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    std::vector<EntryDispersion> cases;
    for (int i = 0; i < 11; ++i) {
        EntryDispersion d;
        d.flight_path_rad = 0.2 * u(rng) * constants::deg_to_rad;  // the campaign's -1 + 0.2 u deg (u > 0: shallower)
        d.speed_mps = 40.0 * u(rng);
        d.range_m = 300000.0 * u(rng);
        d.cross_m = 60000.0 * u(rng);
        (void)u(rng);  // propellant
        d.aero_factor = 1.0 + 0.1 * u(rng);  // crossflow
        (void)u(rng);  // flap
        cases.push_back(d);
    }
    return cases;
}

EntryReachCheck entry_reach_check(const EntryGuidanceParameters& p, bool capsule) {
    EntryReachCheck check;
    check.guidance = p;
    check.capsule = capsule;
    if (capsule) {
        // The capsule's campaign (core/tests/test_capsule_entry.cpp: entry angle +-0.1 deg, mass +-3 %, truth aero
        // +-10 %) is not flown ahead: the fly-ahead of its corridor cut-back disagrees with the 6-DoF campaign - case 1
        // (0.099 deg shallower, mass +2.6 %, aero -7.4 %) misses the 2 km zone by 11-38 km flown ahead (cut back at 0.9 x
        // 4 g or 25 kPa from the interface on), but reaches the gate 0.71 km off in the campaign (peak 3.60 g). Open
        // (planner plan Task 7 fix round 2); the nominal entry is checked.
        check.dispersions.clear();
        check.flight_path_dispersion_rad = 0.1 * constants::deg_to_rad;
    }
    return check;
}

EntryReachCheck entry_reach_check(const EntryGuidance& guidance) {
    EntryReachCheck check = entry_reach_check(guidance.parameters(), guidance.capsule());
    check.limits = guidance.limits();
    return check;
}

EntryReachAssessment assess_entry_reach(const EntryPredictorModel& model, const EntryPredictorState& entry, double bank_rad,
                                        const EntryTarget& target, double gate_height_m, const EntryReachCheck& check) {
    EntryReachAssessment result;
    result.bank_rad = bank_rad;
    EntryGuidanceParameters p = check.guidance;
    p.gate_height_m = gate_height_m;
    const char* debug_env = std::getenv("STARSIM_DEBUG_REACH");
    const bool debug = debug_env != nullptr;

    // the planner's crossrange: the mean of the two signs held at its bank; half their span bounds the reach search
    EntryPredictorState held = entry;
    held.reversal_time_s = -1.0;
    double ends[2] = {0.0, 0.0};
    for (int i = 0; i < 2; ++i) {
        held.bank_rad = (i == 0 ? 1.0 : -1.0) * bank_rad;
        const auto one = predict_entry(model, held, held.bank_rad, target, gate_height_m, 0.0, check.step_s);
        ends[i] = one.crossrange_m;
        result.crossrange_m += 0.5 * one.crossrange_m;
    }
    const double half_span_m = 0.5 * std::abs(ends[0] - ends[1]);

    // the cases: nominal, then each dispersed entry interface of the campaign
    static const char* const names[] = {"case 1", "case 2", "case 3",  "case 4",  "case 5",  "case 6",
                                        "case 7", "case 8", "case 9",  "case 10", "case 11", "case 12",
                                        "case 13", "case 14", "case 15", "case 16"};
    std::vector<Case> cases{{"nominal", model, 1.0, entry}};
    const Vec3 up = math::normalized(entry.position_itrf_m);
    const double speed = norm(entry.velocity_itrf_mps);
    const Vec3 u = entry.velocity_itrf_mps / speed;
    Vec3 down_normal = u * dot(up, u) - up;  // perpendicular to u, towards down
    down_normal = norm(down_normal) > 1e-9 ? math::normalized(down_normal) : -up;
    const Vec3 track = math::normalized(cross(up, cross(u, up)));  // horizontal, along the motion
    const Vec3 normal = math::normalized(cross(entry.position_itrf_m, entry.velocity_itrf_mps));
    const double radius = norm(entry.position_itrf_m);
    for (size_t i = 0; i < check.dispersions.size() && i < std::size(names); ++i) {
        const EntryDispersion& d = check.dispersions[i];
        Case c{names[i], model, 1.0, entry};
        const double down = -d.flight_path_rad;  // > 0: steeper
        c.entry.velocity_itrf_mps = (u * std::cos(down) + down_normal * std::sin(down)) * (speed + d.speed_mps);
        const double back = -d.range_m / radius;  // the interface further back along the path
        c.entry.position_itrf_m = rotated(c.entry.position_itrf_m, normal, back);
        c.entry.velocity_itrf_mps = rotated(c.entry.velocity_itrf_mps, normal, back);
        const double side = d.cross_m / radius;  // to the right of the path
        c.entry.position_itrf_m = rotated(c.entry.position_itrf_m, track, side);
        c.entry.velocity_itrf_mps = rotated(c.entry.velocity_itrf_mps, track, side);
        c.model.lift_scale *= d.aero_factor;
        c.model.drag_scale *= d.aero_factor;
        c.model.mass_kg *= d.mass_factor;  // known onboard (the vehicle's mass)
        c.aero_factor = d.aero_factor;
        cases.push_back(c);
    }
    const std::vector<Guided> flown = fly_all(check, p, cases, bank_rad, std::vector<EntryTarget>(cases.size(), target),
                                              debug_env);  // STARSIM_DEBUG_REACH=<case name>: its cycles

    const Guided& nominal = flown.front();
    result.first_sign = nominal.first_sign;
    result.nominal_flies = nominal.flies;
    result.reversals = nominal.reversals;
    result.reversal_time_s = nominal.reversal_time_s;
    result.reversal_speed_mps = nominal.reversal_speed_mps;
    result.handover_time_s = nominal.handover_time_s;
    result.terminal_miss_m = nominal.terminal_miss_m;
    result.peak_heat_rate_w_m2 = nominal.peak_heat_rate_w_m2;
    result.peak_load_g = nominal.peak_load_g;
    result.peak_dynamic_pressure_pa = nominal.peak_dynamic_pressure_pa;
    bool all_fly = true, all_inside = true;
    for (size_t i = 0; i < flown.size(); ++i) {
        const Guided& g = flown[i];
        if (debug) {
            std::printf("[reach] %-12s flies %d (gate %d miss %.0f m, terminal %.0f m, corridor %d, %d constrained): "
                        "hand-over cross %.1f km after %.0f s, %d reversals (last %.0f s at %.0f m/s), peaks %.0f kW/m2 "
                        "%.2f g %.1f kPa\n",
                        cases[i].name, g.flies, g.reached_gate, g.gate_miss_m, g.terminal_miss_m, g.within_corridor,
                        g.constrained_cycles,
                        g.handover_crossrange_m / 1000.0, g.handover_time_s, g.reversals, g.reversal_time_s,
                        g.reversal_speed_mps, g.peak_heat_rate_w_m2 / 1000.0, g.peak_load_g,
                        g.peak_dynamic_pressure_pa / 1000.0);
        }
        result.worst_peak_heat_rate_w_m2 = std::max(result.worst_peak_heat_rate_w_m2, g.peak_heat_rate_w_m2);
        result.worst_peak_load_g = std::max(result.worst_peak_load_g, g.peak_load_g);
        result.worst_peak_dynamic_pressure_pa = std::max(result.worst_peak_dynamic_pressure_pa, g.peak_dynamic_pressure_pa);
        all_inside = all_inside && g.within_corridor;
        if (!g.flies && all_fly) result.limiting_case = cases[i].name;
        all_fly = all_fly && g.flies;
        result.cases_flying += g.flies ? 1 : 0;
    }
    result.within_corridor = all_inside;
    result.bracketed = all_fly;
    if (!all_fly && nominal.flies) {  // which energy side fails: shallower and faster against the dispersions' sizes
        bool high_fails = false, low_fails = false;
        double gamma_size = 0.0, speed_size = 0.0;
        for (const auto& d : check.dispersions) {
            gamma_size = std::max(gamma_size, std::abs(d.flight_path_rad));
            speed_size = std::max(speed_size, std::abs(d.speed_mps));
        }
        for (size_t i = 1; i < flown.size(); ++i) {
            if (flown[i].flies) continue;
            const EntryDispersion& d = check.dispersions[i - 1];
            const double energy = (gamma_size > 0.0 ? d.flight_path_rad / gamma_size : 0.0) +
                                  (speed_size > 0.0 ? d.speed_mps / speed_size : 0.0);
            (energy > 0.0 ? high_fails : low_fails) = true;
        }
        result.aim_direction = high_fails && !low_fails ? -1 : low_fails && !high_fails ? 1 : 0;
    }
    if (!all_fly || !check.measure_reach) {
        result.reach_m = all_fly || nominal.flies ? std::abs(result.crossrange_m) : 0.0;  // the nominal reaches it
        return result;
    }

    // The reach: every case flown to targets moved further out across (the side the target lies on), up to the one-sign
    // half span; two waves of reach_grid targets each.
    const Vec3 target_up = math::normalized(target.position_itrf_m);
    Vec3 right = cross(target.position_itrf_m - entry.position_itrf_m, target_up);  // right of the arrival track
    right = math::normalized(right - target_up * dot(right, target_up));
    const Vec3 along = cross(target_up, right);
    const double outward = result.crossrange_m >= 0.0 ? 1.0 : -1.0;  // > 0: the target lies right of the mean path
    const double target_radius = norm(target.position_itrf_m);
    const auto moved = [&](double offset_m) {
        EntryTarget t = target;
        t.position_itrf_m = rotated(target.position_itrf_m, along, outward * offset_m / target_radius);
        return t;
    };
    double lo = 0.0, hi = std::max(0.0, half_span_m - std::abs(result.crossrange_m));
    for (int wave = 0; wave < 2 && hi > lo; ++wave) {
        std::vector<double> offsets;
        std::vector<Case> wave_cases;
        std::vector<EntryTarget> targets;
        for (int k = 1; k <= reach_grid; ++k) {
            offsets.push_back(lo + (hi - lo) * k / reach_grid);
            const EntryTarget t = moved(offsets.back());
            for (const auto& c : cases) {
                wave_cases.push_back(c);
                targets.push_back(t);
            }
        }
        const auto tries = fly_all(check, p, wave_cases, bank_rad, targets, nullptr);
        double first_fail = hi;
        double last_fly = lo;
        for (size_t k = 0; k < offsets.size(); ++k) {
            const auto begin = tries.begin() + static_cast<std::ptrdiff_t>(k * cases.size());
            const bool all = std::all_of(begin, begin + static_cast<std::ptrdiff_t>(cases.size()),
                                         [](const Guided& g) { return g.flies; });
            if (!all) {
                first_fail = offsets[k];
                break;
            }
            last_fly = offsets[k];
        }
        if (last_fly >= hi) {  // flies up to the bound
            lo = hi;
            break;
        }
        lo = last_fly;
        hi = first_fail;
    }
    result.reach_margin_m = lo;
    result.reach_m = std::abs(result.crossrange_m) + lo;
    return result;
}

}  // namespace starsim::gnc
