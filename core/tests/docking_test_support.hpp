#pragma once
// ISS i5 tests: a Crew Dragon placed at an ISS port (OEM fixture station, body 1) with the docking mechanism armed by
// the rendezvous target, drifting freely (no attitude hold) unless a proximity flight is started.
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "proximity_test_support.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "test_support.hpp"

namespace starsim::docking_test {

// A station reboost `after_s` seconds after the fixture's start (2026-10-16T00:00 UTC).
struct Reboost {
    double after_s = 0.0;
    double delta_v_mps = 0.0;
};

// IDA-2 as configured (iss.toml, station frame): the fixtures that mean "station x = N" convert through it, so they
// follow the port when its measured position changes.
inline math::Vec3 ida2_station_position_m() {
    const auto iss = config::load_vehicle(STARSIM_CONFIG_DIR, "iss");  // a named copy: the range-for must not bind a temporary
    for (const auto& p : iss.stages.front().docking_ports) {
        if (p.id == "ida2") return p.position_m;
    }
    throw std::logic_error("iss.toml has no port ida2");
}

struct DragonAtPort {
    std::unique_ptr<proximity_test::DragonNearIss> near;
    sim::FlightSession& session;
    time::Epoch t0;

    explicit DragonAtPort(std::unique_ptr<proximity_test::DragonNearIss> d)
        : near(std::move(d)), session(near->session), t0(near->t0) {}

    const vehicle::FlightBody& dragon() const { return session.body(0); }
    const vehicle::FlightBody& station() const { return session.body(1); }

    // The Dragon port at (axial, offset, 0) in the station port frame, moving at (axial_mps, lateral_mps, 0) in it, in
    // the docking attitude turned by `attitude_offset` (body frame); the mechanism armed, free drift.
    static DragonAtPort place(const std::string& port, double axial_m, double offset_m, double axial_mps,
                              double lateral_mps, const math::Quat& attitude_offset = {},
                              const std::vector<Reboost>& reboosts = {}) {
        test::spice_session();  // kernels for the session (SPICE)
        std::vector<sim::StationImpulse> impulses;
        if (!reboosts.empty()) {
            const spice::SpiceTimeConverter clock;
            const auto start = clock.from_utc({2026, 10, 16, 0, 0, 0.0});  // = DragonNearIss::t0
            for (const auto& r : reboosts) impulses.push_back({start + r.after_s, r.delta_v_mps, "Bahnanhebung (Test)"});
        }
        DragonAtPort d(std::make_unique<proximity_test::DragonNearIss>(
            proximity_test::at_port(port, {axial_m, offset_m, 0.0}, {axial_mps, lateral_mps, 0.0}, attitude_offset), true,
            true, std::move(impulses)));
        d.session.set_rendezvous_target(1, "idss", port);
        d.session.set_attitude_mode({});
        return d;
    }
    // Station axes (x, -y, -z) -> the IDA-2 port frame of the config (x out, z = up, y = up x x). The fixtures below say where the Dragon stands relative to the station's
    // structure; the port's clocking (`up`, no longer the zenith since Task 8) must not move it.
    static math::Vec3 station_offset_to_port(const math::Vec3& v) {
        const math::Vec3 station{v.x, -v.y, -v.z};
        const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "iss");
        for (const auto& port : cfg.stages.front().docking_ports)
            if (port.id == "ida2") {
                const math::Vec3 y = cross(port.up, port.axis);
                return {dot(station, port.axis), dot(station, y), dot(station, port.up)};
            }
        throw std::logic_error("iss.toml has no port ida2");
    }
    // The Dragon port at `position_m` (station axes (x, -y, -z), origin at the configured IDA-2 position), at rest
    // in the station frame unless `velocity_mps` (same axes), docking attitude, not turning (so the session may coast).
    static DragonAtPort place_at(const math::Vec3& position_m, const math::Vec3& velocity_mps = {}) {
        test::spice_session();
        DragonAtPort e(std::make_unique<proximity_test::DragonNearIss>(
            proximity_test::at_port("ida2", station_offset_to_port(position_m), station_offset_to_port(velocity_mps))));
        e.session.set_rendezvous_target(1, "idss", "ida2");
        e.session.set_attitude_mode({});
        e.session.apply_angular_rate(0, {});
        return e;
    }
    // Docked at `port` (placed 5 mm before the ring faces meet at 0.075 m/s, flown to hard capture).
    static DragonAtPort docked(const std::string& port, const std::vector<Reboost>& reboosts = {}) {
        auto d = place(port, 0.305, 0.0, -0.075, 0.0, {}, reboosts);
        d.fly_until(contact::DockingPhase::Docked, 1200.0);
        return d;
    }
    // Beside the US module stack (station x = 30 m, clear of Columbus / Kibo at 46.7 m and of the truss at 35.5 m, Dragon axis along -x like at IDA-2), its 4 m heat shield `gap_m`
    // off the 2.2 m module surface on the station +y side, drifting towards it at `speed_mps`.
    static DragonAtPort beside_modules(double gap_m, double speed_mps) {
        test::spice_session();
        DragonAtPort d(std::make_unique<proximity_test::DragonNearIss>(
            proximity_test::at_port("ida2", station_offset_to_port({30.0 - ida2_station_position_m().x, -(2.2 + 2.0 + gap_m), 0.0}),
                                    station_offset_to_port({0.0, speed_mps, 0.0}), {}), true, true, std::vector<sim::StationImpulse>{}));
        d.session.set_rendezvous_target(1, "idss", "ida2");
        d.session.set_attitude_mode({});
        return d;
    }

    // advance_to in 1 s calls until the docking phase is `phase` or `limit_s` after the current epoch.
    bool fly_until(contact::DockingPhase phase, double limit_s) {
        return wait_for(session, t0, [&] { return session.docking_phase() == phase; }, limit_s);
    }
    // advance_to in `step_s` calls (on the t0 grid) until `done` or `limit_s` after the current epoch.
    static bool wait_for(sim::FlightSession& s, time::Epoch t0, const std::function<bool()>& done, double limit_s,
                         double step_s = 1.0) {
        const double start = s.epoch() - t0;
        for (double t = std::round(start); t < start + limit_s; t += step_s) {
            if (done()) return true;
            s.advance_to(t0 + t + step_s, 10000000);
        }
        return done();
    }
};

}  // namespace starsim::docking_test
