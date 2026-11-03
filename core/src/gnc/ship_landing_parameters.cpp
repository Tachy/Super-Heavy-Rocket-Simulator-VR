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

FlipPush ship_flip_push(double flip_accel_mps2, double flip_alpha_radps2, double brake_accel_mps2, double brake_tilt_rad) {
    FlipPush f;
    const double angle = 0.5 * constants::pi;
    f.duration_s = flip_duration_s(angle, flip_alpha_radps2, 0.0);
    const double half = 0.5 * f.duration_s;
    const double dt = f.duration_s / flip_push_steps;
    for (int i = 0; i < flip_push_steps; ++i) {
        const double t = (i + 0.5) * dt;
        const double theta = t < half ? 0.5 * flip_alpha_radps2 * t * t
                                      : angle - 0.5 * flip_alpha_radps2 * (f.duration_s - t) * (f.duration_s - t);
        const double a = flip_accel_mps2 * std::cos(theta);
        f.speed_mps += a * dt;
        f.travel_m += (f.duration_s - t) * a * dt;
    }
    const double brake = brake_accel_mps2 * std::sin(brake_tilt_rad);
    f.brake_m = brake > 0.0 ? f.speed_mps * f.speed_mps / (2.0 * brake) : 0.0;
    return f;
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
    p.descent_two_engine_sink_mps = window.min_sink_mps;
    p.switch_window_from_gate = true;

    // Gate lead: the flip set's push, braked by the first landing set.
    const auto flip = engine_set_thrust(vehicle, ship, *r.flip_engine_groups, constants::standard_pressure_pa,
                                        masses.heavy_cg_station_m);
    const auto push = ship_flip_push(
        thrust_at(flip, k.flip_throttle) / masses.heavy_kg,
        max_pitch_acceleration_radps2(flip, k.flip_throttle, masses.heavy_pitch_inertia_kg_m2),
        sets[0].max_n * (1.0 - l.throttle_margin) / masses.heavy_kg, l.tilt_max_rad[0]);
    p.gate_lead_m = push.lead_m();
    return p;
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
