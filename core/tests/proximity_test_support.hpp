#pragma once
// ISS i4 tests: a CW point-mass chaser with an ideal port-aligned attitude around a station in ideal LVLH attitude.
// The chaser CG is `chaser_lever` behind its port; the target port sits at `port_lever` from the station CG with the
// axes `port_to_lvlh`. Translation fractions act with the Dragon's i2 pure authority.
//
// DragonNearIss: a session with the ISS from the OEM fixture (body 1) and a Crew Dragon (body 0, LVLH hold) placed by a
// function of the station's ports and state (i2 rendezvous and i4 proximity session tests).
#include <algorithm>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/orbital_attitude.hpp"
#include "starsim/gnc/proximity_geometry.hpp"
#include "starsim/gnc/proximity_guidance.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "starsim/orbit/oem.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/station_start.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/docking_port.hpp"

namespace starsim::proximity_test {

struct CwSim {
    double n = 1.128e-3;
    math::Mat3 port_to_lvlh;      // target port frame in LVLH
    math::Vec3 port_lever_lvlh;   // station CG -> target port
    double chaser_lever_m = 4.0;  // chaser port ahead of its CG along its docking axis (= -x_t)
    orbit::RelativeState cg;      // chaser CG, LVLH
    gnc::AxisAcceleration authority{{0.1245, 0.1245, 0.0453, 0.0453, 0.0453, 0.0453}};  // target port axes
    double propellant_kg = 2000.0;
    double kg_per_mps = 16.2;  // [D] i2 lateral cost (conservative for every axis)
    time::Epoch epoch;

    static CwSim ida2() {
        CwSim s;
        s.port_to_lvlh = math::Mat3::from_columns({1, 0, 0}, {0, -1, 0}, {0, 0, -1});  // axis +x, up -z (zenith)
        s.port_lever_lvlh = {35.0, 0.0, 0.0};
        return s;
    }
    static CwSim ida3() {
        CwSim s;
        s.port_to_lvlh = math::Mat3::from_columns({0, 0, -1}, {0, 1, 0}, {1, 0, 0});  // axis -z, up +x; y = z x x
        s.port_lever_lvlh = {28.0, 0.0, -5.0};
        return s;
    }
    math::Vec3 chaser_port_lvlh() const {  // the chaser port leads its CG along -x_t (towards the target port)
        return cg.position_m - chaser_lever_m * (port_to_lvlh * math::Vec3{1, 0, 0});
    }
    gnc::ProximityInput input() const {
        gnc::ProximityInput in;
        in.epoch = epoch;
        in.cg_lvlh = cg;
        const math::Mat3 lvlh_to_port = transpose(port_to_lvlh);
        in.port.position_m = lvlh_to_port * (chaser_port_lvlh() - port_lever_lvlh);
        in.port.velocity_mps = lvlh_to_port * cg.velocity_mps;
        in.port.range_m = norm(in.port.position_m);
        in.port.closing_speed_mps = -dot(in.port.position_m, in.port.velocity_mps) / std::max(in.port.range_m, 1e-9);
        in.port_to_lvlh = port_to_lvlh;
        in.mean_motion_radps = n;
        in.authority_port = authority;
        in.propellant_kg = propellant_kg;
        return in;
    }
    // One step: CW drift plus the commanded acceleration (the fraction is in the target port frame; applied as a
    // velocity change at the start of the step).
    void step(const math::Vec3& fraction_port, double dt) {
        const math::Vec3 fp = fraction_port;
        const math::Vec3 a_port{fp.x * authority.along(0, fp.x), fp.y * authority.along(1, fp.y),
                                fp.z * authority.along(2, fp.z)};
        const math::Vec3 dv = port_to_lvlh * a_port * dt;
        propellant_kg -= norm(dv) * kg_per_mps;
        cg.velocity_mps += dv;
        cg = orbit::cw_propagate(cg, n, dt);
        epoch = epoch + dt;
    }
};

// Runs the guidance until contact, abort completion or `limit_s`; returns the simulated seconds.
inline double run(gnc::ProximityGuidance& g, CwSim& s, double limit_s, double dt = 0.05) {
    double t = 0.0;
    for (; t < limit_s; t += dt) {
        const auto out = g.update(s.input());
        if (g.phase() == gnc::ProximityPhase::Contact) break;
        if (g.phase() == gnc::ProximityPhase::Abort && !out.attitude_hold) break;
        s.step(out.translation_fraction_port, dt);
    }
    return t;
}

inline const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
inline const std::filesystem::path fixture_dir = std::filesystem::path(STARSIM_FIXTURE_DIR) / "iss_oem";

// (IDA-2, IDA-3, ISS CG state) -> the Dragon CG state, all GCRF.
using Placement =
    std::function<math::StateVector(const vehicle::PortKinematics&, const vehicle::PortKinematics&, const math::StateVector&)>;

// The Dragon CG at LVLH `position_m` from the ISS CG with the LVLH relative velocity `velocity_mps`.
inline Placement lvlh_from_cg(const math::Vec3& position_m, const math::Vec3& velocity_mps = {}) {
    return [=](const vehicle::PortKinematics&, const vehicle::PortKinematics&, const math::StateVector& iss) {
        return orbit::chaser_from_lvlh(iss, {position_m, velocity_mps});
    };
}
// The Dragon CG `range_m` out along the IDA-2 axis, co-moving with the port (no omega x r drift in the port frame).
inline Placement in_front_of_ida2(double range_m) {
    return [=](const vehicle::PortKinematics& port, const vehicle::PortKinematics&, const math::StateVector&) {
        const math::Vec3 position = port.position_gcrf_m + port.axes_gcrf * math::Vec3{range_m, 0.0, 0.0};
        const math::Vec3 velocity =
            port.velocity_gcrf_mps + cross(port.angular_velocity_gcrf_radps, position - port.position_gcrf_m);
        return math::StateVector{position, velocity};
    };
}

// (IDA-2, IDA-3, ISS CG state) -> the full Dragon state (attitude and rate included).
using RigidPlacement = std::function<vehicle::RigidBodyState(const vehicle::PortKinematics&, const vehicle::PortKinematics&,
                                                             const math::StateVector&)>;

// The Dragon port (idss) at `position_port_m` in the frame of the station port `port_id` with the velocity
// `velocity_port_mps` relative to that rotating frame, in the docking attitude turned by `attitude_offset` (body
// frame), co-rotating with the station.
inline RigidPlacement at_port(const std::string& port_id, const math::Vec3& position_port_m,
                              const math::Vec3& velocity_port_mps = {}, const math::Quat& attitude_offset = {}) {
    return [=](const vehicle::PortKinematics& ida2, const vehicle::PortKinematics& ida3, const math::StateVector&) {
        const auto& target = port_id == "ida2" ? ida2 : ida3;
        // Dragon port axes in the body: x = (1,0,0), up z = (0,0,-1), y = z x x (config crew_dragon idss)
        const math::Mat3 chaser_axes = math::Mat3::from_columns({1, 0, 0}, {0, -1, 0}, {0, 0, -1});
        const math::Quat q = gnc::docking_attitude_gcrf(chaser_axes, target.axes_gcrf) * attitude_offset;
        const math::Vec3 omega = target.angular_velocity_gcrf_radps;
        const math::Vec3 port = target.position_gcrf_m + target.axes_gcrf * position_port_m;
        const math::Vec3 port_velocity = target.velocity_gcrf_mps + cross(omega, port - target.position_gcrf_m) +
                                         target.axes_gcrf * velocity_port_mps;
        // the lever CG -> idss in GCRF, read from a free body in that attitude
        const vehicle::FlightBody probe(
            std::make_unique<vehicle::Vehicle>(config::load_vehicle(config_dir, "crew_dragon"), config_dir),
            vehicle::RigidBodyState{{7.0e6, 0.0, 0.0}, {0.0, 7.5e3, 0.0}, q, {}});
        const math::Vec3 lever = vehicle::port_kinematics(probe, "idss").position_gcrf_m - probe.state().position_gcrf_m;
        return vehicle::RigidBodyState{port - lever, port_velocity - cross(omega, lever), q, q.inverse_rotate(omega)};
    };
}

struct DragonNearIss {
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;  // not "time": that would hide the namespace for time::Epoch below
    sim::FlightSession session{ephemeris, earth, config_dir};
    time::Epoch t0;
    // The Dragon placed by place_in_orbit (body x along the velocity) at the CG state from `place`.
    explicit DragonNearIss(const Placement& place, bool with_station = true, bool cone_open = true,
                           std::vector<sim::StationImpulse> extra_impulses = {}) {
        init([&](const vehicle::PortKinematics& a, const vehicle::PortKinematics& b, const math::StateVector& iss) {
            session.place_in_orbit(config::load_vehicle(config_dir, "crew_dragon"), place(a, b, iss), t0);
        }, with_station, cone_open, std::move(extra_impulses));
    }
    // The Dragon placed by place_body with the full state from `place`.
    explicit DragonNearIss(const RigidPlacement& place, bool with_station = true, bool cone_open = true,
                           std::vector<sim::StationImpulse> extra_impulses = {}) {
        init([&](const vehicle::PortKinematics& a, const vehicle::PortKinematics& b, const math::StateVector& iss) {
            session.place_body(config::load_vehicle(config_dir, "crew_dragon"), place(a, b, iss), t0);
        }, with_station, cone_open, std::move(extra_impulses));
    }

private:
    using Placer = std::function<void(const vehicle::PortKinematics&, const vehicle::PortKinematics&, const math::StateVector&)>;
    void init(const Placer& placer, bool with_station, bool cone_open, std::vector<sim::StationImpulse> extra_impulses) {
        const auto archive = orbit::OemArchive::scan(fixture_dir, clock);
        t0 = clock.from_utc({2026, 10, 16, 0, 0, 0.0});
        auto iss = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, clock, earth, t0,
                                  sim::IssOrbitSource::Oem);
        sim::FlightSession probe(ephemeris, earth, config_dir);  // reads the ports of the ideal-attitude station
        probe.add_station(iss.vehicle, iss.state_gcrf, iss.epoch);
        const auto& station = probe.body(0).state();
        placer(vehicle::port_kinematics(probe.body(0), "ida2"), vehicle::port_kinematics(probe.body(0), "ida3"),
               {station.position_gcrf_m, station.velocity_gcrf_mps});
        session.set_nose_cone(0, cone_open);
        for (auto& impulse : extra_impulses) iss.impulses.push_back(std::move(impulse));
        if (with_station) session.add_station(iss.vehicle, iss.state_gcrf, iss.epoch, iss.impulses);
        session.set_attitude_mode(gnc::lvlh_hold());
    }
};

// Every probe meets its own drogue within tol_m (refuelling): the problem, or nullopt when they pair one to one.
inline std::optional<std::string> probe_pairing_problem(const std::vector<math::Vec3>& probes_m,
                                                        const std::vector<math::Vec3>& drogues_m, double tol_m) {
    std::set<std::size_t> used;
    for (const auto& probe : probes_m) {
        std::optional<std::size_t> hit;
        for (std::size_t k = 0; k < drogues_m.size(); ++k) {
            if (math::norm(probe - drogues_m[k]) < tol_m) hit = k;
        }
        if (!hit) return "probe without drogue";
        if (!used.insert(*hit).second) return "two probes in one drogue";
    }
    return std::nullopt;
}

}  // namespace starsim::proximity_test
