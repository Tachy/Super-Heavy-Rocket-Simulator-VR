#include "starsim/gnc/catch_feasibility.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "starsim/gnc/descent_snapshot.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/propulsion/engine_performance.hpp"

namespace starsim::gnc {

namespace {

constexpr int need_iterations = 50;  // fixed point on the gate mass: contracts by ~dv / ve per step (< 0.1)

// The highest sea-level exhaust velocity among the flip and landing groups (a lower bound of the need).
double landing_exhaust_velocity_mps(const config::VehicleConfig& vehicle, const config::StageConfig& ship) {
    std::vector<std::string> groups = *ship.return_profile.flip_engine_groups;
    for (const auto& phase : ship.return_profile.landing_engine_phases) groups.insert(groups.end(), phase.begin(), phase.end());
    double ve = 0.0;
    for (const auto& id : groups) {
        for (const auto& g : ship.engine_groups) {
            if (g.id != id) continue;
            const propulsion::EnginePerformance performance(vehicle.engine(g.engine_id));
            ve = std::max(ve, performance.specific_impulse_s(1.0, constants::standard_pressure_pa) *
                                  constants::standard_gravity_mps2);
        }
    }
    return ve;
}

}  // namespace

CatchFeasibility catch_feasibility(const vehicle::Vehicle& ship, const std::vector<std::string>& flip_groups,
                                   double catch_floor_kg) {
    for (const auto& stage : ship.stages()) {
        for (const auto& e : stage.engines) {
            if (e.failed && std::find(flip_groups.begin(), flip_groups.end(), e.group_id) != flip_groups.end()) {
                return {false, report::make(report::Code::catch_engine_failed, {report::data_name("group", e.group_id)})};
            }
        }
    }
    const double usable_kg =
        landing_feed_propellant_kg(ship) - ship.stages().front().config.unusable_propellant.header_kg;
    if (usable_kg < catch_floor_kg) {
        return {false, report::make(report::Code::catch_propellant_short,
                                    {report::mass("usable", std::max(usable_kg, 0.0)), report::mass("needed", catch_floor_kg)})};
    }
    return {};
}

CatchPropellantNeed ship_catch_propellant_need(const config::VehicleConfig& vehicle, const config::StageConfig& ship,
                                               double drop_m, double gate_sink_mps, double catch_sink_mps) {
    const auto p = ship_landing_parameters(vehicle, ship);
    const auto masses = ship_landing_masses(vehicle, ship);
    const double g = constants::standard_gravity_mps2;
    const double brake_n = p.gate_flip.brake_accel_mps2 * masses.heavy_kg;
    const double flip_impulse = ship_flip_push(p.gate_flip).speed_mps * masses.heavy_kg;  // N s
    CatchPropellantNeed n;
    n.exhaust_velocity_mps = landing_exhaust_velocity_mps(vehicle, ship);
    n.end_mass_kg = masses.light_kg;
    const double fall = gate_sink_mps * gate_sink_mps + 2.0 * g * std::max(drop_m, 0.0);
    for (int i = 0; i < need_iterations; ++i) {
        const double mass = n.end_mass_kg + n.propellant_kg;
        const double a = brake_n / mass;
        if (!(a > g)) {  // the landing set cannot hold the ship: no catch
            n.stop_dv_mps = std::numeric_limits<double>::infinity();
            n.propellant_kg = std::numeric_limits<double>::infinity();
            return n;
        }
        const double v_i = std::sqrt((fall * (a - g) + g * catch_sink_mps * catch_sink_mps) / a);
        n.stop_dv_mps = std::max(v_i - catch_sink_mps, 0.0) * a / (a - g);
        n.flip_dv_mps = flip_impulse / mass;
        n.propellant_kg = n.end_mass_kg * std::expm1((n.stop_dv_mps + n.flip_dv_mps) / n.exhaust_velocity_mps);
    }
    return n;
}

double ship_catch_propellant_floor_kg(const config::VehicleConfig& vehicle, const config::StageConfig& ship,
                                      double gate_height_m, double gate_sink_mps, double arm_top_height_m, double margin) {
    const auto masses = ship_landing_masses(vehicle, ship);
    // the CG at the catch: the pins at the arm tops, the CG below them; no tower: the stage bottom on the ground
    const double catch_cg_height_m =
        arm_top_height_m > 0.0 ? arm_top_height_m - (ship.catch_pins.station_m - masses.heavy_cg_station_m)
                               : masses.heavy_cg_station_m;
    const double catch_sink_mps = ship_landing_parameters(vehicle, ship).scvx.snapshot.catch_sink_mps;
    const auto need =
        ship_catch_propellant_need(vehicle, ship, gate_height_m - catch_cg_height_m, gate_sink_mps, catch_sink_mps);
    return need.propellant_kg * (1.0 + margin);
}

}  // namespace starsim::gnc
