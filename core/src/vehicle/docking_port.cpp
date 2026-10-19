#include "starsim/vehicle/docking_port.hpp"

#include <stdexcept>
#include <string>

namespace starsim::vehicle {

PortKinematics port_kinematics(const FlightBody& body, std::string_view port_id) {
    const auto& vehicle = body.vehicle();
    for (const auto& stage : vehicle.stages()) {
        for (const auto& port : stage.config.docking_ports) {
            if (port.id != port_id) continue;
            const auto& s = body.state();
            const math::Vec3 lever_body =
                math::Vec3{stage.base_station_m, 0.0, 0.0} + port.position_m - vehicle.mass_properties().cg_point_m();
            const math::Vec3 lever = s.attitude.rotate(lever_body);
            const math::Vec3 omega = s.attitude.rotate(s.angular_velocity_body_radps);
            const math::Vec3 x = s.attitude.rotate(port.axis), z = s.attitude.rotate(port.up);
            PortKinematics k;
            k.position_gcrf_m = s.position_gcrf_m + lever;
            k.velocity_gcrf_mps = s.velocity_gcrf_mps + cross(omega, lever);
            k.axes_gcrf = math::Mat3::from_columns(x, cross(z, x), z);
            k.angular_velocity_gcrf_radps = omega;
            k.covered = port.behind_nose_cone && vehicle.has_nose_cone() && !vehicle.nose_cone_open();
            return k;
        }
    }
    // Vehicle keeps no vehicle id: name the stages searched
    std::string stages;
    for (const auto& stage : vehicle.stages()) stages += (stages.empty() ? "" : ", ") + stage.config.id;
    throw std::invalid_argument("Unbekannter Andockport '" + std::string(port_id) + "' (Stufen: " + stages + ")");
}

}  // namespace starsim::vehicle
