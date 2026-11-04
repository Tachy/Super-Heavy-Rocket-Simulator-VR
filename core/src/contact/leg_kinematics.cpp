#include "starsim/contact/leg_kinematics.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::contact {

using math::Vec3;

namespace {
// (s, r) turned by `angle` (a positive angle takes +s towards +r).
void turn(double angle, double& s, double& r) {
    const double c = std::cos(angle), n = std::sin(angle);
    const double s2 = s * c - r * n, r2 = s * n + r * c;
    s = s2;
    r = r2;
}
}  // namespace

bool has_leg_kinematics(const config::LandingLegConfig& legs) { return legs.hinge_radius_m > 0.0; }

double leg_deployed_angle_rad(const config::LandingLegConfig& legs) {
    return std::atan2(legs.footprint_radius_m - legs.hinge_radius_m,
                      legs.foot_station_m + legs.knuckle_height_m - legs.hinge_station_m);
}

LegPose leg_pose(const config::LandingLegConfig& legs, double azimuth_rad, double deployment) {
    const double d = std::clamp(deployment, 0.0, 1.0);
    const double phi = leg_deployed_angle_rad(legs);
    const double leg_turn = (1.0 - d) * (0.0 - phi);
    const double foot_turn = leg_turn + (1.0 - d) * (phi - constants::pi);
    double ks = legs.foot_station_m + legs.knuckle_height_m - legs.hinge_station_m;
    double kr = legs.footprint_radius_m - legs.hinge_radius_m;
    turn(leg_turn, ks, kr);
    double fs = -legs.knuckle_height_m, fr = 0.0;  // the foot below the knuckle, deployed
    turn(foot_turn, fs, fr);
    const Vec3 radial{0.0, std::cos(azimuth_rad), std::sin(azimuth_rad)};
    const auto at = [&](double s, double r) { return Vec3{s, 0.0, 0.0} + r * radial; };
    const double hs = legs.hinge_station_m, hr = legs.hinge_radius_m;
    return {at(hs, hr), at(hs + ks, hr + kr), at(hs + ks + fs, hr + kr + fr)};
}

}  // namespace starsim::contact
