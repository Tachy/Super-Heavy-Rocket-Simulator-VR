#include "starsim/vehicle/attitude_hold_gas.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::vehicle {

double gravity_gradient_hold_gas_kgps(double inertia_difference_kg_m2, double orbital_rate_radps, double deadband_rad,
                                      double torque_per_mass_flow_nm_per_kgps) {
    if (!(torque_per_mass_flow_nm_per_kgps > 0.0) || !(deadband_rad > 0.0)) return 0.0;
    const double mean_torque_nm = 1.5 * orbital_rate_radps * orbital_rate_radps * std::abs(inertia_difference_kg_m2) *
                                  (1.0 - std::cos(2.0 * deadband_rad)) / (2.0 * deadband_rad);
    return mean_torque_nm / torque_per_mass_flow_nm_per_kgps;
}

double pitch_torque_per_mass_flow(const Vehicle& v) {
    const math::Vec3 cg = v.mass_properties().cg_point_m();
    // the best pitch arm (torque per thrust, positive sense) forward and aft of the CG
    double forward_m = 0.0, aft_m = 0.0, forward_ve = 0.0, aft_ve = 0.0;
    for (const auto& s : v.stages()) {
        for (const auto& t : s.rcs) {
            if (!(t.thrust_n > 0.0) || !(t.isp_s > 0.0)) continue;
            const double arm_m = math::cross(t.position_m - cg, t.direction).y;
            const bool forward = t.position_m.x >= cg.x;
            double& best = forward ? forward_m : aft_m;
            if (arm_m > best) {
                best = arm_m;
                (forward ? forward_ve : aft_ve) = t.isp_s * constants::standard_gravity_mps2;
            }
        }
    }
    if (forward_m > 0.0 && aft_m > 0.0) {  // the couple: torque (a_f + a_a) F for a flow F / ve_f + F / ve_a
        return (forward_m + aft_m) / (1.0 / forward_ve + 1.0 / aft_ve);
    }
    return std::max(forward_m * forward_ve, aft_m * aft_ve);  // one side only: the single thruster
}

double lvlh_hold_gas_kgps(const Vehicle& v, double orbital_rate_radps, double deadband_rad) {
    const auto& inertia = v.mass_properties().inertia_kg_m2;  // (roll, pitch, yaw)
    return gravity_gradient_hold_gas_kgps(inertia.z - inertia.x, orbital_rate_radps, deadband_rad,
                                          pitch_torque_per_mass_flow(v));
}

}  // namespace starsim::vehicle
