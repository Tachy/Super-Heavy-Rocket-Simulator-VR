#pragma once

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::contact {

// Kinematics of a deployable lander leg (HLS, Task 6b; [stage.landing_legs] hinge_station_m, hinge_radius_m,
// knuckle_height_m), a function of the deployment only. In the leg's radial plane (station s, radius r): the rigid leg
// runs from the hinge H to the knuckle K, which stands knuckle_height_m above the deployed foot F (footprint radius,
// foot station). Deployed (1) it points from H to K at the angle phi from +s towards +r; it turns about the hinge's
// tangent axis by (1 - d) x (0 - phi), so stowed (0) it lies along the hull, tip up. The foot hangs from the knuckle
// along -s when deployed and folds about the knuckle by (1 - d) x (phi - 180 deg), so stowed it continues the leg's line.
// The same kinematics draw the legs (tools/structures/blender/hls_legs_layout.py, VehicleModels.hls_leg_pose).
struct LegPose {
    math::Vec3 hinge_m, knuckle_m, foot_m;  // stage body frame: x = station from the stage bottom
};

// True when the legs carry the kinematics (hinge_radius_m > 0).
bool has_leg_kinematics(const config::LandingLegConfig& legs);
// The deployed angle phi of the leg (rad, from +x towards the radial).
double leg_deployed_angle_rad(const config::LandingLegConfig& legs);
// Hinge, knuckle and foot of the leg at `azimuth_rad` for `deployment` in [0, 1] (clamped).
LegPose leg_pose(const config::LandingLegConfig& legs, double azimuth_rad, double deployment);

}  // namespace starsim::contact
