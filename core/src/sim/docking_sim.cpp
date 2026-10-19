#include "starsim/sim/docking_sim.hpp"

#include <algorithm>
#include <limits>

#include "starsim/gnc/relative_navigation.hpp"
#include "starsim/vehicle/docking_port.hpp"

namespace starsim::sim {

DockingSim::DockingSim(std::string chaser_port, std::string station_port, const config::VehicleConfig& chaser_config,
                       const config::VehicleConfig& station_config, contact::DockingMechanismParameters p)
    : chaser_port_(std::move(chaser_port)),
      station_port_(std::move(station_port)),
      mechanism_(p),
      shape_(contact::station_shape(station_config)),
      hull_body_m_(contact::hull_sample_points(chaser_config, collision_p_)) {}

double DockingSim::coast_limit_s(const vehicle::FlightBody& chaser, const vehicle::FlightBody& station) const {
    if (chaser.docked()) return std::numeric_limits<double>::infinity();
    const auto& cs = chaser.state();
    const auto& ss = station.state();
    const math::Vec3 d = cs.position_gcrf_m - ss.position_gcrf_m;
    const double range = norm(d);
    const math::Vec3 cg_body = chaser.vehicle().mass_properties().cg_point_m();
    double reach = 0.0;
    for (const auto& p : hull_body_m_) reach = std::max(reach, norm(p - cg_body));
    double clearance = 0.0;
    if (range > collision_p_.check_range_m) {
        clearance = range - collision_p_.check_range_m;  // to where the shape check takes over (review i5: no free coast)
    } else {
        const math::Vec3 cg_station = station.vehicle().mass_properties().cg_point_m() + ss.attitude.inverse_rotate(d);
        clearance = std::numeric_limits<double>::infinity();
        for (const auto& c : shape_.capsules) {
            const math::Vec3 ab = c.b_m - c.a_m;
            const double t = std::clamp(dot(cg_station - c.a_m, ab) / dot(ab, ab), 0.0, 1.0);
            clearance = std::min(clearance, norm(cg_station - (c.a_m + t * ab)) - c.radius_m);
        }
        for (const auto& b : shape_.boxes) clearance = std::min(clearance, contact::distance_to_box(b, cg_station));
        clearance -= reach;
        const auto& ip = mechanism_.parameters().interface;
        const auto ck = vehicle::port_kinematics(chaser, chaser_port_);
        const auto sk = vehicle::port_kinematics(station, station_port_);
        clearance = std::min(clearance, norm(ck.position_gcrf_m - sk.position_gcrf_m) -
                                            (ip.ring_extension_m + 2.0 * ip.petal_height_m + ip.interface_radius_m));
    }
    if (clearance <= 0.0) return 0.0;
    const math::Vec3 omega_s = ss.attitude.rotate(ss.angular_velocity_body_radps);
    const math::Vec3 omega_c = cs.attitude.rotate(cs.angular_velocity_body_radps);
    const double speed =
        norm(cs.velocity_gcrf_mps - ss.velocity_gcrf_mps - cross(omega_s, d)) + norm(omega_c - omega_s) * reach;
    // The relative motion accelerates (CW: 3 n^2 radially, -n^2 cross-track, Coriolis 2 n v): bound it by
    // 3 n^2 range + 2 n speed and solve clearance = speed t + accel t^2 / 2 (review i5: a parked chaser coasted deep in).
    const double n = norm(omega_s);
    const double accel = 3.0 * n * n * range + 2.0 * n * speed;
    if (accel <= 0.0) return clearance / std::max(speed, 1e-6);
    return (-speed + std::sqrt(speed * speed + 2.0 * accel * clearance)) / accel;
}

contact::DockingPose DockingSim::pose(const vehicle::FlightBody& chaser, const vehicle::FlightBody& station) const {
    const auto ck = vehicle::port_kinematics(chaser, chaser_port_);
    const auto sk = vehicle::port_kinematics(station, station_port_);
    const auto rel = gnc::relative_port_state(ck, sk);
    return {rel.position_m, rel.velocity_mps, transpose(sk.axes_gcrf) * ck.axes_gcrf, rel.angular_rate_radps};
}

void DockingSim::before_tick(vehicle::FlightBody& chaser, vehicle::FlightBody& station, time::Epoch epoch,
                             double dt_s) {
    if (!t0_) t0_ = epoch;
    balance_n_ = last_force_n_ = {};
    hull_contact_ = false;
    if (chaser.docked()) {  // held on the station: the mechanism only runs its clock (hooks)
        mechanism_.update(epoch - *t0_, {}, 0.0, 0.0, dt_s);
        return;
    }
    const auto& cs = chaser.state();
    const auto& ss = station.state();
    const math::Mat3 rc = cs.attitude.to_matrix();  // body -> GCRF
    const math::Mat3 rs = ss.attitude.to_matrix();
    const auto mp = chaser.vehicle().mass_properties();

    // mechanism wrench (passive port frame) -> GCRF force at its point, chaser body torque
    const auto sk = vehicle::port_kinematics(station, station_port_);
    const auto ck = vehicle::port_kinematics(chaser, chaser_port_);
    const math::Vec3 lever_body = transpose(rc) * (ck.position_gcrf_m - cs.position_gcrf_m);
    const double inertia = std::max({mp.inertia_kg_m2.x, mp.inertia_kg_m2.y, mp.inertia_kg_m2.z}) +
                           mp.mass_kg * dot(lever_body, lever_body);
    const auto w = mechanism_.update(epoch - *t0_, pose(chaser, station), mp.mass_kg, inertia, dt_s);
    math::Vec3 force = sk.axes_gcrf * w.force_port_n;
    const math::Vec3 point = sk.position_gcrf_m + sk.axes_gcrf * w.point_port_m;
    math::Vec3 torque_gcrf = cross(point - cs.position_gcrf_m, force) + sk.axes_gcrf * w.torque_port_nm;

    // hull contact with the station primitives (station body frame), within the check range
    if (norm(cs.position_gcrf_m - ss.position_gcrf_m) < collision_p_.check_range_m) {
        const math::Mat3 to_station = transpose(rs);
        // the station primitives live in its vehicle frame (origin at the stage bottom): body offsets + its CG point
        const math::Vec3 station_cg = station.vehicle().mass_properties().cg_point_m();
        const math::Vec3 omega_c = rc * cs.angular_velocity_body_radps;
        const math::Vec3 omega_s = rs * ss.angular_velocity_body_radps;
        std::vector<math::Vec3> points, velocities;
        points.reserve(hull_body_m_.size());
        velocities.reserve(hull_body_m_.size());
        const math::Vec3 cg_body = mp.cg_point_m();
        for (const auto& p : hull_body_m_) {
            const math::Vec3 r = rc * (p - cg_body);
            const math::Vec3 at = cs.position_gcrf_m + r;
            const math::Vec3 v = cs.velocity_gcrf_mps + cross(omega_c, r);
            const math::Vec3 v_station = ss.velocity_gcrf_mps + cross(omega_s, at - ss.position_gcrf_m);
            points.push_back(station_cg + to_station * (at - ss.position_gcrf_m));
            velocities.push_back(to_station * (v - v_station));
        }
        const auto h = contact::hull_contact(shape_, points, velocities,
                                             station_cg + to_station * (cs.position_gcrf_m - ss.position_gcrf_m), collision_p_);
        if (h.any) {
            hull_contact_ = true;
            const math::Vec3 f = rs * h.force_station_n;
            force += f;
            torque_gcrf += rs * h.torque_station_nm;
            collision_impulse_ns_ = collision_impulse_ns_.value_or(0.0) + norm(f) * dt_s;
        }
    }
    if (norm(force) == 0.0 && norm(torque_gcrf) == 0.0) return;
    const math::Vec3 on_station = -force;
    chaser.apply_external_wrench(force, transpose(rc) * torque_gcrf);
    station.apply_external_wrench(on_station, {});  // ideal attitude: the torque is dropped
    balance_n_ = force + on_station;
    last_force_n_ = force;
}

}  // namespace starsim::sim
