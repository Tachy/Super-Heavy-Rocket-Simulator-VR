#include "starsim/propulsion/engine_performance.hpp"

#include <algorithm>

#include "starsim/math/constants.hpp"

namespace starsim::propulsion {

EnginePerformance::EnginePerformance(const config::EngineConfig& config)
    : config_(config),
      full_mass_flow_kgps_(config.thrust_ref_n / (config.isp_ref_s * constants::standard_gravity_mps2)),
      vacuum_thrust_n_(config.thrust_ref_n + config.reference_pressure_pa * config.nozzle_exit_area_m2),
      max_ambient_pressure_pa_(config.max_ambient_pressure_pa) {}

double EnginePerformance::thrust_n(double throttle, double ambient_pressure_pa) const {
    return std::max(0.0, throttle * vacuum_thrust_n_ - ambient_pressure_pa * config_.nozzle_exit_area_m2);
}

double EnginePerformance::specific_impulse_s(double throttle, double ambient_pressure_pa) const {
    const double mdot = mass_flow_kgps(throttle);
    return mdot > 0.0 ? thrust_n(throttle, ambient_pressure_pa) / (mdot * constants::standard_gravity_mps2) : 0.0;
}

}  // namespace starsim::propulsion
