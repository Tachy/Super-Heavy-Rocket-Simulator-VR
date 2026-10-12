#pragma once

#include "starsim/config/vehicle_config.hpp"

namespace starsim::propulsion {

// Steady-state performance of one engine type (see config/README.md, "Engine thrust model"):
//   mdot(throttle)     = throttle * F_ref / (Isp_ref * g0)
//   F(throttle, p_amb) = throttle * F_vac - p_amb * A_exit,   F_vac = F_ref + p_ref * A_exit
// Exhaust velocity is taken as independent of throttle (chamber-pressure effects on Isp neglected).
class EnginePerformance {
public:
    explicit EnginePerformance(const config::EngineConfig& config);

    double mass_flow_kgps(double throttle) const { return throttle * full_mass_flow_kgps_; }
    double thrust_n(double throttle, double ambient_pressure_pa) const;
    double specific_impulse_s(double throttle, double ambient_pressure_pa) const;

    // Vacuum nozzles must not run where the flow would separate (e.g. an RVac at sea level).
    bool can_operate(double ambient_pressure_pa) const { return ambient_pressure_pa <= max_ambient_pressure_pa_; }

    const config::EngineConfig& config() const { return config_; }

private:
    config::EngineConfig config_;
    double full_mass_flow_kgps_;
    double vacuum_thrust_n_;
    double max_ambient_pressure_pa_;
};

}  // namespace starsim::propulsion
