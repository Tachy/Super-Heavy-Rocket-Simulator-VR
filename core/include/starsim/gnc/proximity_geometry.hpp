#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "starsim/gnc/relative_navigation.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// Proximity operations of a visiting vehicle at a station (ISS i4): the waypoint path per approach, the approach
// corridor inside the keep-out sphere, the docking attitude and the capture envelope. LVLH of frames/lvlh.hpp
// (x V-bar, y H-bar, z R-bar = nadir); port frame of vehicle/docking_port.hpp (x out of the port, z its up).

enum class DockingApproach { VBar, RBar };  // IDA-2 forward / IDA-3 zenith
// From the target port axis in the station LVLH frame: dominant +x -> VBar, dominant -z -> RBar; otherwise
// std::invalid_argument (German: "Kein Anflugweg für diesen Port").
DockingApproach docking_approach(const math::Vec3& port_axis_lvlh);

enum class WaypointFrame { LvlhFromCg, TargetPort };  // CG-relative in station LVLH / chaser port in target port frame
struct ProximityWaypoint {
    std::string name;  // German: "Wegpunkt 0", "Umflug 1", "Wegpunkt 1", "Wegpunkt 2"
    WaypointFrame frame;
    math::Vec3 position_m;
    double hold_s = 0.0;   // go/no-go hold after arrival (0: fly on)
    bool on_axis = false;  // TargetPort waypoints on the docking axis: reached by the axial approach
};

struct TranslationLoopParameters {
    double gain_time_s = 1.0;       // [E] fraction = error / (acceleration * gain_time)
    double start_error_mps = 0.02;  // [E] a pulse train starts above this velocity error
    double stop_error_mps = 0.005;  // [E] ... and stops below this
};

struct BreakoutParameters {
    std::vector<double> magnitudes_mps{0.3, 0.5, 1.0, 1.5};  // [E] tried in order, the smallest safe one wins
    double horizon_s = 86400.0;    // [E] passively safe for 24 h (ISS visiting-vehicle practice)
    double sample_s = 30.0;        // [E] CW sampling of the safety check
    double exit_limit_s = 1800.0;  // [E] the keep-out sphere must be left within this
    // Departure (ISS i5): the arc must also leave the approach ellipsoid (semi-axes in LVLH) within the limit.
    std::optional<math::Vec3> ellipsoid_semi_axes_m;
    double ellipsoid_exit_limit_s = 3600.0;  // [E] Demo-1 left the approach ellipsoid 17 min after undocking
};

struct ProximityParameters {
    double keep_out_radius_m = 200.0;  // [S: NASA] keep-out sphere about the station CG (= mission::keep_out_sphere_radius_m)
    double kos_margin_m = 10.0;        // [E] transfers outside the sphere keep this clearance
    double waypoint0_below_m = 400.0;  // [S: Crew-2 timeline] (i3 ends here)
    double waypoint1_range_m = 220.0;  // [S: Crew-2 / Demo-1 / Crew-5 timelines] on the port axis
    double waypoint2_range_m = 20.0;   // [S: same]
    double waypoint0_hold_s = 300.0;   // [E] go/no-go holds
    double waypoint1_hold_s = 300.0;   // [E]
    double waypoint2_hold_s = 300.0;   // [E] Crew-5: hold + docking go/no-go at Waypoint 2
    double transfer_s = 1140.0;        // [D] Crew-2 Waypoint 0 04:25 -> Waypoint 1 04:49 minus the 5 min hold
    double flyaround_radius_m = 250.0;                         // [E] IDA-3 fly-around about the station CG
    double flyaround_step_rad = 30.0 * constants::deg_to_rad;  // [E] legs of the arc
    double flyaround_leg_s = 300.0;                            // [E]
    double transfer_final_s = 60.0;         // [E] the CW re-solve stops this long before arrival; the hold law brakes
    double arrival_tolerance_m = 2.0;       // [E] LVLH waypoints
    double axis_arrival_tolerance_m = 0.3;  // [E] on-axis waypoints
    double arrival_speed_mps = 0.01;        // [E]
    double corridor_half_angle_rad = 10.0 * constants::deg_to_rad;  // [E] spec: cone about the port axis inside the sphere
    double corridor_min_half_width_m = 0.3;   // [E] the cone's lateral bound never drops below this near the port
    double corridor_retreat_factor = 2.0;     // [E] beyond this multiple of the bound: retreat instead of hold
    double approach_speed_far_mps = 0.30;     // [S: Demo-2 0.98 ft/s]
    double approach_time_constant_s = 600.0;  // [E] axial speed = range / this between the limits
    double final_speed_start_mps = 0.10;      // [S: Demo-2 0.33 ft/s before soft capture] at Waypoint 2
    double contact_speed_mps = 0.075;         // [E] middle of the axial capture band 0.05-0.10
    double contact_plane_m = 0.0;  // [E] contact when the chaser port reaches this axial distance (i5: the session sets the petal mesh plane)
    double lateral_gain_per_s = 0.1;        // [E] lateral velocity = -offset * gain (offset ~ deadband mean / gain)
    double lateral_speed_max_mps = 0.03;    // [E] below the lateral capture bound 0.04
    double hold_time_constant_s = 60.0;     // [E] position hold: velocity = -error / this
    double hold_speed_max_mps = 0.05;       // [E]
    double retreat_speed_mps = 0.10;        // [E] outward along the axis
    double overspeed_factor = 1.5;          // [E] closing faster than factor * schedule + margin: hold
    double overspeed_margin_mps = 0.02;     // [E]
    double deorbit_reserve_kg = 600.0;      // [E] d3: 598 kg deorbit from 420 km; below it: abort
    double max_start_range_m = 5000.0;      // [E] start within the approach ellipsoid's scale (4 x 2 x 2 km)
    double resume_max_range_m = 5000.0;     // [E] a resume after an abort flies back to Waypoint 0 from within this
    std::size_t start_waypoint = 0;         // first waypoint of the path to fly to (tests: 2 = start at Waypoint 2)
    double start_tolerance_m = 10.0;        // [E] a start within this of the first waypoint holds there (i3: 2-5 m)
    TranslationLoopParameters loop_far;     // outside the keep-out sphere
    TranslationLoopParameters loop_near{1.0, 0.005, 0.002};  // [E] inside: finer (contact offset < 0.1 m)
    BreakoutParameters breakout;
};

// Waypoint 0, then for VBar Waypoint 1 / 2; for RBar the fly-around (front point on V-bar at flyaround_radius_m,
// arc points every flyaround_step_rad towards -z, last one short of the zenith), then Waypoint 1 / 2.
std::vector<ProximityWaypoint> proximity_path(DockingApproach approach, const ProximityParameters& p);

enum class CorridorStatus { OutsideKos, Inside, Hold, Retreat };
// `position_port_m`: chaser port in the target port frame; `cg_range_m`: CG to CG. Inside the sphere: the lateral
// offset against max(x tan(half angle), min half width) - Inside / Hold (beyond) / Retreat (beyond factor x bound, or
// x <= 0: behind the port plane without contact).
CorridorStatus corridor_status(const math::Vec3& position_port_m, double cg_range_m, const ProximityParameters& p);

// The chaser attitude that mates its port with the target port: chaser port axes x = -x_t, y = -y_t, z = z_t
// (axes opposed, ups equal). `chaser_port_axes_body`: columns of the chaser port frame in its body frame.
math::Quat docking_attitude_gcrf(const math::Mat3& chaser_port_axes_body, const math::Mat3& target_port_axes_gcrf);

struct CaptureEnvelope {  // [S: IDSS IDD E Table 3.3.1.1-2, Initial Contact Conditions] (verified in i5)
    double axial_min_mps = 0.05, axial_max_mps = 0.10;
    double lateral_max_mps = 0.04;
    double offset_max_m = 0.10;
    double angle_max_rad = 4.0 * constants::deg_to_rad;   // pitch/yaw misalignment, vector sum
    double rate_max_radps = 0.2 * constants::deg_to_rad;  // pitch/yaw rate, vector sum
    double roll_max_rad = 4.0 * constants::deg_to_rad;
    double roll_rate_max_radps = 0.2 * constants::deg_to_rad;
};
struct EnvelopeCheck {
    bool inside = false;
    std::string reason;  // German, the first violated bound ("" when inside)
};
EnvelopeCheck capture_envelope_check(const RelativePortState& state, const CaptureEnvelope& envelope = {});

}  // namespace starsim::gnc
