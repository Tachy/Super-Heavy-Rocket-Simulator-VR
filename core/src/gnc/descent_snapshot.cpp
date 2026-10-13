#include "starsim/gnc/descent_snapshot.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/frames/geodetic.hpp"

namespace starsim::gnc {

using math::Mat3;
using math::Quat;
using math::Vec3;

namespace {

Mat3 from_columns(const Vec3& a, const Vec3& b, const Vec3& c) {
    Mat3 m;
    m(0, 0) = a.x; m(1, 0) = a.y; m(2, 0) = a.z;
    m(0, 1) = b.x; m(1, 1) = b.y; m(2, 1) = b.z;
    m(0, 2) = c.x; m(1, 2) = c.y; m(2, 2) = c.z;
    return m;
}

bool contains(const std::vector<std::string>& groups, const std::string& group) {
    return std::find(groups.begin(), groups.end(), group) != groups.end();
}

}  // namespace

PlanFrame plan_frame(double latitude_rad, double longitude_rad, double height_m) {
    const double sl = std::sin(latitude_rad), cl = std::cos(latitude_rad);
    const double so = std::sin(longitude_rad), co = std::cos(longitude_rad);
    return {from_columns({cl * co, cl * so, sl}, {-so, co, 0.0}, {-sl * co, -sl * so, cl}),
            frames::geodetic_to_fixed({latitude_rad, longitude_rad, height_m})};
}

DescentState plan_state(const vehicle::FlightBody& body, const frames::FixedToInertial& earth, const PlanFrame& frame) {
    const auto& s = body.state();
    const Mat3 to_fixed = transpose(earth.rotation);
    const Vec3 omega = vehicle::earth_rotation_vector(earth);  // GCRF
    const Vec3 r_fixed = to_fixed * s.position_gcrf_m;
    const Vec3 v_fixed = to_fixed * (s.velocity_gcrf_mps - cross(omega, s.position_gcrf_m));
    const Quat gcrf_to_plan = Quat::from_matrix(transpose(frame.plan_to_itrf) * to_fixed);
    return make_state(body.vehicle().mass_properties().mass_kg, frame.to_plan(r_fixed), frame.direction_to_plan(v_fixed),
                      (gcrf_to_plan * s.attitude).normalized(),
                      s.angular_velocity_body_radps - s.attitude.inverse_rotate(omega));
}

DescentSnapshot take_descent_snapshot(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                      time::Epoch epoch, const LandingSite& site, const PlanFrame& frame,
                                      const SnapshotSettings& settings) {
    const auto t = env.earth_orientation.fixed_to_gcrf(epoch);
    const Mat3 gcrf_to_plan = transpose(frame.plan_to_itrf) * transpose(t.rotation);
    const auto& vehicle = body.vehicle();
    const auto mass = vehicle.mass_properties();

    DescentModelInputs in;
    in.vehicle = &vehicle;
    in.ignition_groups = settings.ignition_groups;
    in.final_groups = settings.final_groups;
    in.atmosphere = &env.atmosphere;
    in.site_height_m = site.height_m;
    in.gravity_mps2 = gcrf_to_plan * env.gravity.acceleration_mps2(t.rotation * frame.origin_itrf_m, epoch);
    in.earth_rate_radps = gcrf_to_plan * vehicle::earth_rotation_vector(t);
    in.earth_center_m = frame.to_plan({0.0, 0.0, 0.0});

    DescentSnapshot snap;
    snap.model = make_descent_model(in);
    snap.initial = plan_state(body, t, frame);
    snap.epoch = epoch;
    snap.min_mass_kg = mass.mass_kg - vehicle.propellant_kg() + settings.reserve_kg;

    if (site.catch_target) {
        const auto& c = *site.catch_target;
        const Vec3 pins = frame.to_plan(c.pin_point_itrf_m) + Vec3{settings.catch_clearance_m, 0.0, 0.0};
        snap.catch_target.goal = DescentGoal::Catch;
        snap.catch_target.position_m = pins - Vec3{c.pin_station_m - mass.cg_station_m, 0.0, 0.0};
        snap.catch_target.velocity_mps = {-settings.catch_sink_mps, 0.0, 0.0};
        const double arm_top = frame.to_plan(c.pin_point_itrf_m).x;
        snap.catch_target.corridor_top_m = arm_top + mass.cg_station_m + settings.corridor_margin_m;
        snap.catch_target.corridor_radius_m = settings.corridor_radius_m;
        snap.catch_target.final_set_radius_m = settings.final_set_radius_m;
        const Vec3 tower = frame.to_plan(c.tower_center_itrf_m);
        snap.abort_target.keep_out_center_m = {0.0, tower.y, tower.z};
        snap.abort_target.keep_out_radius_m = c.tower_keep_out_radius_m;
        snap.abort_target.mount_keep_out_radius_m = c.mount_keep_out_radius_m;
        snap.abort_target.mount_top_cg_height_m = c.mount_height_m + mass.cg_station_m + settings.mount_margin_m;
    }
    snap.abort_target.goal = DescentGoal::Abort;
    snap.abort_target.ground_cg_height_m = mass.cg_station_m;  // bottom on the ground (flat at the pad height)
    if (site.sea) {
        const Vec3 seaward{0.0, std::sin(site.sea->direction_rad), std::cos(site.sea->direction_rad)};
        snap.abort_target.sea_target_m = seaward * (site.sea->shoreline_distance_m + settings.sea_margin_m);
    }

    double spool = 0.0;
    int engines = 0;
    for (const auto& e : vehicle.stages().front().engines) {
        if (e.starved || !contains(settings.ignition_groups, e.group_id)) continue;
        spool += e.spool;
        snap.startup_s = std::max(snap.startup_s, e.performance->config().startup_s);
        ++engines;
    }
    snap.phase0_spool = engines > 0 ? spool / engines : 1.0;
    return snap;
}

}  // namespace starsim::gnc
