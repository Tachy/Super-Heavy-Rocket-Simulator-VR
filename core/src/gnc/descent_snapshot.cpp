#include "starsim/gnc/descent_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

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

PlanFrame plan_frame(double latitude_rad, double longitude_rad, double height_m, const frames::Ellipsoid& shape) {
    const double sl = std::sin(latitude_rad), cl = std::cos(latitude_rad);
    const double so = std::sin(longitude_rad), co = std::cos(longitude_rad);
    return {from_columns({cl * co, cl * so, sl}, {-so, co, 0.0}, {-sl * co, -sl * so, cl}),
            frames::geodetic_to_fixed({latitude_rad, longitude_rad, height_m}, shape)};
}

PlanFrame plan_frame(const LandingSite& site) {
    // The Moon: origin on the terrain at the touchdown point, x along the sphere's radial (a sphere's geodetic
    // vertical), y east, z north in MOON_ME.
    const frames::Ellipsoid shape =
        site.body == ephemeris::Body::Moon ? frames::Ellipsoid{constants::moon_mean_radius_m, 0.0} : frames::wgs84;
    return plan_frame(site.latitude_rad, site.longitude_rad, site.height_m, shape);
}

environment::BodyFrame site_body_frame(const LandingSite& site, const vehicle::FlightEnvironment& env, time::Epoch epoch) {
    if (site.body != ephemeris::Body::Moon) {
        return environment::BodyFrame::centred_at_origin(env.earth_orientation.fixed_to_gcrf(epoch));
    }
    if (!env.moon) throw std::logic_error("lunar landing site without the Moon as a reference body");
    return env.moon->frame(epoch);
}

namespace {

// From the thrust centroid through the CG (see thrust_axis_body).
Vec3 axis_through_cg(Vec3 centroid, double thrust, double cg_station_m) {
    if (thrust <= 0.0) return {1.0, 0.0, 0.0};
    centroid = centroid / thrust;
    const double lever = cg_station_m - centroid.x;
    if (lever <= 0.0) return {1.0, 0.0, 0.0};
    return math::normalized(Vec3{lever, -centroid.y, -centroid.z});
}

}  // namespace

Vec3 thrust_axis_body(const vehicle::Vehicle& vehicle, double cg_station_m) {
    Vec3 centroid;
    double thrust = 0.0;
    for (const auto& e : vehicle.stages().front().engines) {
        if (!e.producing_thrust()) continue;
        const double f = e.spool * e.performance->thrust_n(e.throttle, 0.0);
        centroid += e.gimbal_point_m * f;
        thrust += f;
    }
    return axis_through_cg(centroid, thrust, cg_station_m);
}

Vec3 thrust_axis_body(const vehicle::Vehicle& vehicle, double cg_station_m, const std::vector<std::string>& groups) {
    Vec3 centroid;
    double thrust = 0.0;
    for (const auto& e : vehicle.stages().front().engines) {
        if (e.starved || !contains(groups, e.group_id)) continue;
        const double f = e.performance->thrust_n(1.0, 0.0);
        centroid += e.gimbal_point_m * f;
        thrust += f;
    }
    return axis_through_cg(centroid, thrust, cg_station_m);
}

DescentState plan_state(const vehicle::FlightBody& body, const environment::BodyFrame& frame, const PlanFrame& plan) {
    const auto& s = body.state();
    // Hull -> thrust axis: the shortest rotation from body x to the thrust direction.
    const Vec3 thrust_axis = thrust_axis_body(body.vehicle(), body.vehicle().mass_properties().cg_station_m);
    const Vec3 turn = cross(Vec3{1.0, 0.0, 0.0}, thrust_axis);
    const double turn_angle = std::atan2(norm(turn), thrust_axis.x);
    const Quat hull_to_thrust = turn_angle > 0.0 ? Quat::from_axis_angle(turn / norm(turn), turn_angle) : Quat{};
    const Quat attitude = s.attitude * hull_to_thrust;
    // Earth: centre zero, so every expression is the former ITRF one exactly (subtracting zero vectors is exact).
    const Mat3 to_fixed = transpose(frame.transform.rotation);
    const Vec3& omega = frame.rotation_gcrf_radps;
    const Vec3 r_fixed = frame.to_fixed(s.position_gcrf_m);
    const Vec3 v_fixed = to_fixed * frame.relative_velocity(s.position_gcrf_m, s.velocity_gcrf_mps);
    const Quat gcrf_to_plan = Quat::from_matrix(transpose(plan.plan_to_itrf) * to_fixed);
    return make_state(body.vehicle().mass_properties().mass_kg, plan.to_plan(r_fixed), plan.direction_to_plan(v_fixed),
                      (gcrf_to_plan * attitude).normalized(),
                      hull_to_thrust.inverse_rotate(s.angular_velocity_body_radps - s.attitude.inverse_rotate(omega)));
}

DescentSnapshot take_descent_snapshot(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                      time::Epoch epoch, const LandingSite& site, const PlanFrame& frame,
                                      const SnapshotSettings& settings) {
    const auto body_frame = site_body_frame(site, env, epoch);
    const auto& t = body_frame.transform;
    const Mat3 gcrf_to_plan = transpose(frame.plan_to_itrf) * transpose(t.rotation);
    const auto& vehicle = body.vehicle();
    const auto mass = vehicle.mass_properties();
    const bool lunar = site.body == ephemeris::Body::Moon;

    DescentModelInputs in;
    in.vehicle = &vehicle;
    in.phase_groups = settings.phase_groups;
    in.atmosphere = lunar ? nullptr : &env.atmosphere;
    in.site_height_m = site.height_m;
    // Gravitation over the touchdown point; the lunar plan frame falls with the Moon (its centre's acceleration out).
    const Vec3 gravity_gcrf = env.gravity.acceleration_mps2(body_frame.position_gcrf(frame.origin_itrf_m), epoch);
    in.gravity_mps2 = gcrf_to_plan * (lunar ? gravity_gcrf - env.moon->centre_acceleration_gcrf(epoch) : gravity_gcrf);
    in.earth_rate_radps = gcrf_to_plan * body_frame.rotation_gcrf_radps;
    in.earth_center_m = frame.to_plan({0.0, 0.0, 0.0});

    DescentSnapshot snap;
    snap.model = make_descent_model(in);
    snap.initial = plan_state(body, body_frame, frame);
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
    if (site.pad) {
        // Touchdown: the CG where the feet meet the pad, sinking at the end speed.
        snap.catch_target.goal = DescentGoal::Pad;
        snap.catch_target.position_m = {mass.cg_station_m + site.pad->contact_bottom_height_m, 0.0, 0.0};
        snap.catch_target.velocity_mps = {-settings.catch_sink_mps, 0.0, 0.0};
    }
    snap.abort_target.goal = DescentGoal::Abort;
    // Bottom on the ground (flat at the pad height), on the legs at a pad site.
    snap.abort_target.ground_cg_height_m = mass.cg_station_m + (site.pad ? site.pad->contact_bottom_height_m : 0.0);
    if (site.sea) {
        const Vec3 seaward{0.0, std::sin(site.sea->direction_rad), std::cos(site.sea->direction_rad)};
        snap.abort_target.sea_target_m = seaward * (site.sea->shoreline_distance_m + settings.sea_margin_m);
        snap.abort_target.sea_min_distance_m = site.sea->shoreline_distance_m + settings.sea_clearance_m;
    }

    double spool = 0.0;
    int engines = 0;
    for (const auto& e : vehicle.stages().front().engines) {
        if (e.starved || !contains(settings.phase_groups[0], e.group_id)) continue;
        spool += e.spool;
        snap.startup_s = std::max(snap.startup_s, e.performance->config().startup_s);
        ++engines;
    }
    snap.phase0_spool = engines > 0 ? spool / engines : 1.0;
    const double pressure =
        lunar ? 0.0 : env.atmosphere.at(site.height_m + snap.initial[descent_index::position]).pressure_pa;
    for (const auto& e : vehicle.stages().front().engines) {
        if (!e.starved) snap.thrust_n += e.spool * e.performance->thrust_n(e.throttle, pressure);
    }
    return snap;
}

}  // namespace starsim::gnc
