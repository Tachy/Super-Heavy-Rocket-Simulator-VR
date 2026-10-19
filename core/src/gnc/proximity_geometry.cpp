// Proximity geometry of the Crew Dragon approach to the ISS. Real pattern (Crew-2 / Demo-1 / Crew-5 timelines,
// Demo-2 press): Waypoint 0 400 m below, Waypoint 1 220 m out on the port axis, Waypoint 2 20 m out, final approach
// at 0.30 -> 0.10 m/s.
// Spec correction 2: IDA-3 (R-bar) gets a fly-around at 250 m [E] from the V-bar front up to Waypoint 1 above IDA-3.
// Spec correction 4: i4 ends at the contact plane; the capture envelope here is [E] until i5 checks the IDSS IDD.
#include "starsim/gnc/proximity_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace starsim::gnc {

using math::Vec3;

DockingApproach docking_approach(const Vec3& port_axis_lvlh) {
    const Vec3 u = normalized(port_axis_lvlh);
    const double dominant = std::cos(45.0 * constants::deg_to_rad);
    if (u.x > dominant) return DockingApproach::VBar;
    if (-u.z > dominant) return DockingApproach::RBar;
    throw std::invalid_argument("Kein Anflugweg für diesen Port");
}

std::vector<ProximityWaypoint> proximity_path(DockingApproach approach, const ProximityParameters& p) {
    std::vector<ProximityWaypoint> path;
    path.push_back({"Wegpunkt 0", WaypointFrame::LvlhFromCg, {0.0, 0.0, p.waypoint0_below_m}, p.waypoint0_hold_s, false});
    if (approach == DockingApproach::RBar) {
        int k = 1;  // front point, then the arc towards the zenith; the zenith leg ends at Waypoint 1
        for (double a = 0.0; a < 0.5 * constants::pi - 0.5 * p.flyaround_step_rad; a += p.flyaround_step_rad, ++k) {
            path.push_back({"Umflug " + std::to_string(k), WaypointFrame::LvlhFromCg,
                            {p.flyaround_radius_m * std::cos(a), 0.0, -p.flyaround_radius_m * std::sin(a)}, 0.0,
                            false});
        }
    }
    path.push_back({"Wegpunkt 1", WaypointFrame::TargetPort, {p.waypoint1_range_m, 0.0, 0.0}, p.waypoint1_hold_s, true});
    path.push_back({"Wegpunkt 2", WaypointFrame::TargetPort, {p.waypoint2_range_m, 0.0, 0.0}, p.waypoint2_hold_s, true});
    return path;
}

CorridorStatus corridor_status(const Vec3& position_port_m, double cg_range_m, const ProximityParameters& p) {
    if (cg_range_m >= p.keep_out_radius_m) return CorridorStatus::OutsideKos;
    if (position_port_m.x <= 0.0) return CorridorStatus::Retreat;
    const double bound =
        std::max(position_port_m.x * std::tan(p.corridor_half_angle_rad), p.corridor_min_half_width_m);
    const double lateral = std::hypot(position_port_m.y, position_port_m.z);
    if (lateral <= bound) return CorridorStatus::Inside;
    if (lateral <= p.corridor_retreat_factor * bound) return CorridorStatus::Hold;
    return CorridorStatus::Retreat;
}

math::Quat docking_attitude_gcrf(const math::Mat3& chaser_port_axes_body, const math::Mat3& target_port_axes_gcrf) {
    const math::Mat3& t = target_port_axes_gcrf;
    const math::Mat3 desired = math::Mat3::from_columns(-(t * Vec3{1, 0, 0}), -(t * Vec3{0, 1, 0}), t * Vec3{0, 0, 1});
    return math::Quat::from_matrix(desired * transpose(chaser_port_axes_body));
}

namespace {

EnvelopeCheck violated(const char* what, double value, const char* unit) {
    char text[96];
    std::snprintf(text, sizeof text, "%s (%.3f %s)", what, value, unit);
    return {false, text};
}

}  // namespace

EnvelopeCheck capture_envelope_check(const RelativePortState& state, const CaptureEnvelope& envelope) {
    const double axial_mps = -state.velocity_mps.x;
    if (axial_mps > envelope.axial_max_mps) return violated("axial zu schnell", axial_mps, "m/s");
    if (axial_mps < envelope.axial_min_mps) return violated("axial zu langsam", axial_mps, "m/s");
    const double lateral_mps = std::hypot(state.velocity_mps.y, state.velocity_mps.z);
    if (lateral_mps > envelope.lateral_max_mps) return violated("lateral zu schnell", lateral_mps, "m/s");
    const double offset_m = std::hypot(state.position_m.y, state.position_m.z);
    if (offset_m > envelope.offset_max_m) return violated("Versatz zu groß", offset_m, "m");
    if (state.misalignment_rad > envelope.angle_max_rad)
        return violated("Winkel zu groß", state.misalignment_rad / constants::deg_to_rad, "°");
    const math::Vec3& w = state.angular_rate_radps;  // target port frame: x = the docking axis (roll)
    const double rate_radps = std::hypot(w.y, w.z);
    if (rate_radps > envelope.rate_max_radps)
        return violated("Drehrate zu groß", rate_radps / constants::deg_to_rad, "°/s");
    if (std::abs(state.clocking_rad) > envelope.roll_max_rad)
        return violated("Rollwinkel zu groß", std::abs(state.clocking_rad) / constants::deg_to_rad, "°");
    if (std::abs(w.x) > envelope.roll_rate_max_radps)
        return violated("Rollrate zu groß", std::abs(w.x) / constants::deg_to_rad, "°/s");
    return {true, ""};
}

}  // namespace starsim::gnc
