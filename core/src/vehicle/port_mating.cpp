#include "starsim/vehicle/port_mating.hpp"

#include <stdexcept>

#include "starsim/vehicle/vehicle.hpp"

namespace starsim::vehicle {

namespace {

// Columns: the outward axis, y = up x axis, up (the port frame of vehicle/docking_port.hpp).
math::Mat3 port_axes(const config::DockingPortConfig& p) {
    return math::Mat3::from_columns(p.axis, math::cross(p.up, p.axis), p.up);
}

}  // namespace

StageTransform mated_stage_transform(const config::DockingPortConfig& chaser, const config::DockingPortConfig& target) {
    // Mated, the chaser's port axes are the target's (-x, -y, +z) (contact/docking_interface.hpp).
    const math::Mat3 flip = math::Mat3::from_columns({-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0});
    const math::Mat3 rotation = port_axes(target) * flip * math::transpose(port_axes(chaser));
    return {rotation, target.position_m - rotation * chaser.position_m};
}

std::optional<report::Report> port_mismatch(const config::DockingPortConfig& chaser,
                                            const config::DockingPortConfig& target) {
    if (chaser.mechanism != target.mechanism) {
        return report::make(report::Code::docking_ports_mismatch,
                            {report::data_name("chaser", chaser.id), report::data_name("target", target.id)});
    }
    if (chaser.mechanism == config::DockingMechanism::ProbeDrogue &&
        (chaser.soft_capture != config::SoftCaptureMode::Active ||
         target.soft_capture != config::SoftCaptureMode::Passive)) {
        return report::make(report::Code::docking_probe_drogue_roles,
                            {report::data_name("chaser", chaser.id), report::data_name("target", target.id)});
    }
    return std::nullopt;
}

const config::DockingPortConfig& port_config(const Vehicle& vehicle, std::string_view port_id) {
    for (const auto& stage : vehicle.stages()) {
        for (const auto& port : stage.config.docking_ports) {
            if (port.id == port_id) return port;
        }
    }
    throw std::invalid_argument("Unknown docking port '" + std::string(port_id) + "'");
}

}  // namespace starsim::vehicle
