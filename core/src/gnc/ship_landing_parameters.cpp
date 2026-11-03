// The ship's flip and landing burn parameters from its config: K values read, rate limits, the 3 -> 2 sink window and
// the gate lead derived from the engine sets and the landing masses (plan 2026-11-03 planner-vehicle-parameters Task 3).

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "starsim/gnc/ship_landing.hpp"
#include "starsim/vehicle/stage_inertia.hpp"

namespace starsim::gnc {

namespace {

constexpr double no_sink_limit_mps = 1.0e3;  // an engine-set switch without a sink window
constexpr int flip_push_steps = 2000;        // midpoint rule over the flip (smooth integrand: < 1e-6 relative)

std::vector<std::string> stage_ids(const config::StageConfig& stage, const std::vector<std::string>& groups) {
    std::vector<std::string> out;
    for (const auto& g : groups) out.push_back(stage.id + "/" + g);
    return out;
}

// Thrust of a set at a throttle: linear between the set's ends (EnginePerformance::thrust_n is linear in it).
double thrust_at(const EngineSetThrust& set, double throttle) {
    const double span = set.throttle_max - set.throttle_min;
    const double t = std::clamp(throttle, set.throttle_min, set.throttle_max);
    return span > 0.0 ? set.min_n + (t - set.throttle_min) / span * (set.max_n - set.min_n) : set.max_n;
}

// The throttle at which a set gives `thrust_n` (thrust_at's inverse).
double throttle_for(const EngineSetThrust& set, double thrust_n) {
    const double span = set.max_n - set.min_n;
    const double t = span > 0.0 ? set.throttle_min + (thrust_n - set.min_n) / span * (set.throttle_max - set.throttle_min)
                                : set.throttle_max;
    return std::clamp(t, set.throttle_min, set.throttle_max);
}

// The longest shutdown of the engines `groups` leaves running when the set steps down to `next`.
double shutdown_s(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                  const std::vector<std::string>& groups, const std::vector<std::string>& next) {
    double longest = 0.0;
    for (const auto& id : groups) {
        if (std::find(next.begin(), next.end(), id) != next.end()) continue;
        for (const auto& g : stage.engine_groups) {
            if (g.id == id) longest = std::max(longest, vehicle.engine(g.engine_id).shutdown_s);
        }
    }
    return longest;
}

// The longest startup of the engines of `groups` (a relight: thrust and authority ramp up over it).
double startup_s(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                 const std::vector<std::string>& groups) {
    double longest = 0.0;
    for (const auto& id : groups) {
        for (const auto& g : stage.engine_groups) {
            if (g.id == id) longest = std::max(longest, vehicle.engine(g.engine_id).startup_s);
        }
    }
    return longest;
}

}  // namespace

ShipLandingMasses ship_landing_masses(const config::VehicleConfig& vehicle, const config::StageConfig& ship) {
    // Main tanks empty, headers full: the configured landing load, whatever a scenario loaded.
    auto full = ship;
    double header_kg = 0.0;
    for (auto& t : full.tanks) {
        if (t.role != config::TankRole::Header) continue;
        t.mass_kg = t.capacity_kg;
        header_kg += t.capacity_kg;
    }
    const auto heavy = vehicle::settled_stage_mass_properties(vehicle, full, 0.0);
    return {heavy.mass_kg, heavy.mass_kg - header_kg + ship.unusable_propellant.header_kg, heavy.cg_station_m,
            heavy.pitch_inertia_kg_m2};
}

double ship_phase_rate_limit_radps(const EngineSetThrust& set, double tilt_max_rad, double inertia_kg_m2) {
    const double alpha = max_pitch_acceleration_radps2(set, set.throttle_min, inertia_kg_m2);
    return std::min(set.gimbal_rate_radps, std::sqrt(alpha * tilt_max_rad));
}

SinkWindow ship_switch_sink_window(const EngineSetThrust& outgoing, const EngineSetThrust& incoming, double light_mass_kg,
                                   double heavy_mass_kg, double height_m, double throttle_margin, double shutdown_s,
                                   double gravity_mps2) {
    SinkWindow w;
    const double out_decel = outgoing.min_n / light_mass_kg - gravity_mps2;
    w.min_sink_mps = out_decel > 0.0 ? out_decel * shutdown_s : 0.0;
    const double in_decel = incoming.max_n * (1.0 - throttle_margin) / heavy_mass_kg - gravity_mps2;
    w.max_sink_mps = in_decel > 0.0 ? std::sqrt(2.0 * in_decel * std::max(height_m, 0.0)) : 0.0;
    return w;
}

namespace {

// The spool s(t) = min(1, t / T) and its first two integrals (T = 0: full at once).
double spool_at(double t, double spool_s) { return spool_s > 0.0 ? std::min(1.0, t / spool_s) : 1.0; }
double spool_integral(double t, double spool_s) {
    if (spool_s <= 0.0) return t;
    if (t <= spool_s) return t * t / (2.0 * spool_s);
    return 0.5 * spool_s + (t - spool_s);
}
double spool_double_integral(double t, double spool_s) {
    if (spool_s <= 0.0) return 0.5 * t * t;
    if (t <= spool_s) return t * t * t / (6.0 * spool_s);
    const double u = t - spool_s;
    return spool_s * spool_s / 6.0 + 0.5 * spool_s * u + 0.5 * u * u;
}
// The time at which spool_integral reaches y.
double spool_integral_inverse(double y, double spool_s) {
    if (spool_s <= 0.0) return y;
    if (y <= 0.5 * spool_s) return std::sqrt(2.0 * spool_s * y);
    return spool_s + (y - 0.5 * spool_s);
}

// Bang-bang turn at alpha s(t): +alpha s up to t_switch, -alpha s after it until the rate is zero (t_end).
struct SpooledTurn {
    double alpha = 0.0;
    double spool_s = 0.0;
    double t_switch = 0.0;
    double end_s() const { return spool_integral_inverse(2.0 * spool_integral(t_switch, spool_s), spool_s); }
    double angle_at(double t) const {
        if (t <= t_switch) return alpha * spool_double_integral(t, spool_s);
        return alpha * (2.0 * spool_double_integral(t_switch, spool_s) +
                        2.0 * spool_integral(t_switch, spool_s) * (t - t_switch) - spool_double_integral(t, spool_s));
    }
};

// Rest-to-rest turn through `angle` at a constant alpha, the rate capped (bang-coast-bang; flip_duration_s's profile).
double capped_turn_angle(double t, double angle, double alpha, double rate_max) {
    const double t_peak = std::sqrt(angle / alpha);
    const double t_acc = rate_max > 0.0 ? std::min(rate_max / alpha, t_peak) : t_peak;
    const double rate = alpha * t_acc;
    const double t_coast = t_acc < t_peak ? (angle - alpha * t_acc * t_acc) / rate : 0.0;
    const double total = 2.0 * t_acc + t_coast;
    if (t <= t_acc) return 0.5 * alpha * t * t;
    if (t <= t_acc + t_coast) return 0.5 * alpha * t_acc * t_acc + rate * (t - t_acc);
    const double left = std::max(total - t, 0.0);
    return angle - 0.5 * alpha * left * left;
}

}  // namespace

FlipPush ship_flip_push(const FlipModel& m) {
    FlipPush f;
    const double angle = 0.5 * constants::pi;
    // Flip: the switch time that ends the turn at rest at 90 deg (the end angle grows with it), by bisection.
    SpooledTurn turn{m.flip_alpha_radps2, m.spool_s, 0.0};
    double lo = 0.0, hi = flip_duration_s(angle, m.flip_alpha_radps2, 0.0) + m.spool_s;
    for (int i = 0; i < 200 && hi - lo > 1e-15 * hi; ++i) {
        turn.t_switch = 0.5 * (lo + hi);
        (turn.angle_at(turn.end_s()) < angle ? lo : hi) = turn.t_switch;
    }
    turn.t_switch = 0.5 * (lo + hi);
    f.duration_s = turn.end_s();
    const double dt = f.duration_s / flip_push_steps;
    for (int i = 0; i < flip_push_steps; ++i) {
        const double t = (i + 0.5) * dt;
        const double a = m.flip_accel_mps2 * spool_at(t, m.spool_s) * std::cos(turn.angle_at(t));
        f.speed_mps += a * dt;
        f.travel_m += (f.duration_s - t) * a * dt;
    }
    // The gate's own horizontal speed along the line carries on through the flip.
    f.travel_m += m.initial_speed_mps * f.duration_s;
    // Tilt-in: the horizontal part of the braking set's thrust grows with the tilt; the braking ends at rest.
    double v = m.initial_speed_mps + f.speed_mps;
    if (m.brake_tilt_rad > 0.0 && m.brake_alpha_radps2 > 0.0) {
        f.tilt_in_s = flip_duration_s(m.brake_tilt_rad, m.brake_alpha_radps2, m.brake_rate_max_radps);
        const double h = f.tilt_in_s / flip_push_steps;
        for (int i = 0; i < flip_push_steps && v > 0.0; ++i) {
            const double t = (i + 0.5) * h;
            const double a = m.brake_accel_mps2 *
                             std::sin(capped_turn_angle(t, m.brake_tilt_rad, m.brake_alpha_radps2, m.brake_rate_max_radps));
            if (a * h >= v) {  // stops within this step: v^2 / (2 a)
                f.tilt_in_m += a > 0.0 ? v * v / (2.0 * a) : 0.0;
                v = 0.0;
                break;
            }
            f.tilt_in_m += (v - 0.5 * a * h) * h;
            v -= a * h;
        }
    }
    f.brake_speed_mps = std::max(v, 0.0);
    const double brake = m.brake_accel_mps2 * std::sin(m.brake_tilt_rad);
    f.brake_m = brake > 0.0 ? f.brake_speed_mps * f.brake_speed_mps / (2.0 * brake) : 0.0;
    return f;
}

double ship_approach_time_constant_s(const config::VehicleConfig& vehicle, const config::StageConfig& ship,
                                     const std::vector<std::string>& groups) {
    double tau = 0.0;
    for (const auto& id : groups) {
        for (const auto& g : ship.engine_groups) {
            if (g.id != id) continue;
            const auto& e = vehicle.engine(g.engine_id);
            if (e.throttle_rate_per_s > 0.0) tau = std::max(tau, (e.throttle_max - e.throttle_min) / e.throttle_rate_per_s);
        }
    }
    return tau;
}

ShipLandingParameters ship_landing_parameters(const config::VehicleConfig& vehicle, const config::StageConfig& ship) {
    if (!ship.ship_landing) {
        throw std::invalid_argument("ship_landing_parameters: stage '" + ship.id +
                                    "' has no ship landing keys ([stage.recovery], config::ShipLandingProfile)");
    }
    const auto& k = *ship.ship_landing;
    const auto& r = ship.return_profile;
    ShipLandingParameters p;
    p.flip_groups = ship_flip_groups(ship);
    p.flip_throttle = k.flip_throttle;
    p.flip_tolerance_rad = k.flip_tolerance_rad;
    p.flip_settled_rate_radps = k.flip_settled_rate_radps;
    p.flip_flaps_until_tilt_rad = k.flaps_until_tilt_rad;
    p.descent_propellant_margin = k.descent_propellant_margin;

    auto& s = p.scvx;
    for (int i = 0; i < descent_phases; ++i) {
        s.snapshot.phase_groups[static_cast<size_t>(i)] = stage_ids(ship, r.landing_engine_phases.at(static_cast<size_t>(i)));
    }
    // The header tanks carry the landing; the engines starve when they run dry. The unusable header propellant is the
    // plans' hard floor: no plan or re-plan draws on it.
    s.snapshot.reserve_kg = ship.unusable_propellant.header_kg;
    s.snapshot.structure_keep_out = true;
    s.replan_reserve_fraction = 0.0;
    s.abort_reserve_fraction = 0.0;
    s.lateral_effort_fraction = k.lateral_effort_fraction;
    auto& l = s.settings.limits;
    l.throttle_margin = 0.05;        // [E]
    l.terminal_thrust_margin = 1.0;  // no "gentle end" cap: two engines at minimum are below the weight

    const auto masses = ship_landing_masses(vehicle, ship);
    std::array<EngineSetThrust, descent_phases> sets{};
    for (int i = 0; i < descent_phases; ++i) {
        const auto u = static_cast<size_t>(i);
        l.tilt_max_rad[u] = r.landing_phase_max_tilt_rad.at(u);
        sets[u] = engine_set_thrust(vehicle, ship, r.landing_engine_phases[u], constants::standard_pressure_pa,
                                    masses.heavy_cg_station_m);
        l.rate_max_radps[u] = ship_phase_rate_limit_radps(sets[u], l.tilt_max_rad[u], masses.heavy_pitch_inertia_kg_m2);
    }
    // 3 -> 2: the minimum from the sets now, the maximum at the gate from its height (switch_window_from_gate). The
    // planner's third phase flies the same pair (no switch).
    const auto window = ship_switch_sink_window(
        sets[0], sets[1], masses.light_kg, masses.heavy_kg, 0.0, l.throttle_margin,
        shutdown_s(vehicle, ship, r.landing_engine_phases[0], r.landing_engine_phases[1]), constants::standard_gravity_mps2);
    l.switches = {{{0.0, 0.0, window.min_sink_mps, no_sink_limit_mps}, {0.0, 0.0, 0.0, no_sink_limit_mps}}};
    p.switch_window_from_gate = true;
    // The window is wide (no fitted operating point in its middle): the first plan's guess switches from the state.
    l.late_switch_guess = true;
    // Catch approach: the time the landing pair runs its throttle range.
    l.approach_time_constant_s = ship_approach_time_constant_s(vehicle, ship, r.landing_engine_phases.back());

    // Gate lead: the flip set's push (spooling up from the relight), braked by the first landing set after its tilt-in.
    const auto flip = engine_set_thrust(vehicle, ship, *r.flip_engine_groups, constants::standard_pressure_pa,
                                        masses.heavy_cg_station_m);
    FlipModel model;
    model.flip_accel_mps2 = thrust_at(flip, k.flip_throttle) / masses.heavy_kg;
    model.flip_alpha_radps2 = max_pitch_acceleration_radps2(flip, k.flip_throttle, masses.heavy_pitch_inertia_kg_m2);
    model.spool_s = startup_s(vehicle, ship, *r.flip_engine_groups);
    model.brake_accel_mps2 = sets[0].max_n * (1.0 - l.throttle_margin) / masses.heavy_kg;
    model.brake_alpha_radps2 = max_pitch_acceleration_radps2(
        sets[0], throttle_for(sets[0], sets[0].max_n * (1.0 - l.throttle_margin)), masses.heavy_pitch_inertia_kg_m2);
    model.brake_rate_max_radps = l.rate_max_radps[0];
    model.brake_tilt_rad = l.tilt_max_rad[0];
    p.gate_flip = model;
    p.gate_flip_mass_kg = masses.heavy_kg;
    p.gate_lead_m = ship_flip_push(model).lead_m();
    return p;
}

double ship_gate_lead_m(const ShipLandingParameters& p, double along_speed_mps, double mass_kg) {
    FlipModel m = p.gate_flip;
    if (p.gate_flip_mass_kg > 0.0 && mass_kg > 0.0) {  // same thrust on the live mass
        const double scale = p.gate_flip_mass_kg / mass_kg;
        m.flip_accel_mps2 *= scale;
        m.brake_accel_mps2 *= scale;
    }
    m.initial_speed_mps = along_speed_mps;
    return ship_flip_push(m).lead_m();
}

std::vector<std::string> ship_flip_groups(const config::StageConfig& ship) {
    if (!ship.ship_landing) {
        throw std::invalid_argument("ship_flip_groups: stage '" + ship.id + "' has no ship landing keys");
    }
    return stage_ids(ship, *ship.return_profile.flip_engine_groups);
}

ShipLandingParameters ship_landing_parameters(const vehicle::Vehicle& ship) {
    return ship_landing_parameters(ship.source_config(), ship.stages().front().config);
}

double ship_switch_max_sink_mps(const config::VehicleConfig& vehicle, const config::StageConfig& ship,
                                const ShipLandingParameters& p, double height_m) {
    const auto masses = ship_landing_masses(vehicle, ship);
    const auto& phases = ship.return_profile.landing_engine_phases;
    const auto set = [&](size_t i) {
        return engine_set_thrust(vehicle, ship, phases.at(i), constants::standard_pressure_pa, masses.heavy_cg_station_m);
    };
    return ship_switch_sink_window(set(0), set(1), masses.light_kg, masses.heavy_kg, height_m,
                                   p.scvx.settings.limits.throttle_margin, 0.0, constants::standard_gravity_mps2)
        .max_sink_mps;
}

}  // namespace starsim::gnc
