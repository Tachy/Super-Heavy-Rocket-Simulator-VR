#include "starsim/gnc/relative_navigation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace starsim::gnc {

RelativeNavMode parse_relative_nav(std::string_view text) {
    if (text == "ideal") return RelativeNavMode::Ideal;
    throw std::invalid_argument("relative_nav must be \"ideal\", not \"" + std::string(text) + "\"");
}

RelativePortState relative_port_state(const vehicle::PortKinematics& chaser, const vehicle::PortKinematics& target,
                                      RelativeNavMode /*mode: only Ideal*/) {
    const math::Mat3 to_port = transpose(target.axes_gcrf);
    const math::Vec3 dr = chaser.position_gcrf_m - target.position_gcrf_m;
    // seen from the rotating target frame: remove the frame's omega x r
    const math::Vec3 dv =
        chaser.velocity_gcrf_mps - target.velocity_gcrf_mps - cross(target.angular_velocity_gcrf_radps, dr);
    RelativePortState s;
    s.position_m = to_port * dr;
    s.velocity_mps = to_port * dv;
    s.angular_rate_radps = to_port * (chaser.angular_velocity_gcrf_radps - target.angular_velocity_gcrf_radps);
    s.range_m = norm(dr);
    s.closing_speed_mps = s.range_m > 0.0 ? -dot(dr, dv) / s.range_m : 0.0;
    const math::Vec3 tx = math::column(target.axes_gcrf, 0), tz = math::column(target.axes_gcrf, 2);
    const math::Vec3 cx = math::column(chaser.axes_gcrf, 0), cz = math::column(chaser.axes_gcrf, 2);
    s.misalignment_rad = std::acos(std::clamp(-dot(cx, tx), -1.0, 1.0));
    const math::Vec3 cz_plane = cz - dot(cz, tx) * tx;  // chaser up projected onto the docking plane
    s.clocking_rad = std::atan2(dot(cross(tz, cz_plane), tx), dot(tz, cz_plane));
    s.chaser_port_covered = chaser.covered;
    return s;
}

}  // namespace starsim::gnc
