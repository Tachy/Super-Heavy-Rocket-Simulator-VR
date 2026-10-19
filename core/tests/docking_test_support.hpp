#pragma once
// ISS i5 tests: a Crew Dragon placed at an ISS port (OEM fixture station, body 1) with the docking mechanism armed by
// the rendezvous target, drifting freely (no attitude hold) unless a proximity flight is started.
#include <memory>
#include <string>

#include "proximity_test_support.hpp"
#include "test_support.hpp"

namespace starsim::docking_test {

struct DragonAtPort {
    std::unique_ptr<proximity_test::DragonNearIss> near;
    sim::FlightSession& session;
    time::Epoch t0;

    explicit DragonAtPort(std::unique_ptr<proximity_test::DragonNearIss> d)
        : near(std::move(d)), session(near->session), t0(near->t0) {}

    const vehicle::FlightBody& dragon() const { return session.body(0); }
    const vehicle::FlightBody& station() const { return session.body(1); }

    // The Dragon port at (axial, offset, 0) in the station port frame, moving at (axial_mps, lateral_mps, 0) in it, in
    // the docking attitude; the mechanism armed, free drift.
    static DragonAtPort place(const std::string& port, double axial_m, double offset_m, double axial_mps,
                              double lateral_mps, const math::Quat& attitude_offset = {}) {
        test::spice_session();  // kernels for the session (SPICE)
        DragonAtPort d(std::make_unique<proximity_test::DragonNearIss>(proximity_test::at_port(
            port, {axial_m, offset_m, 0.0}, {axial_mps, lateral_mps, 0.0}, attitude_offset)));
        d.session.set_rendezvous_target(1, "idss", port);
        d.session.set_attitude_mode({});
        return d;
    }
    // Docked at `port` (placed 5 mm before the ring faces meet at 0.075 m/s, flown to hard capture).
    static DragonAtPort docked(const std::string& port) {
        auto d = place(port, 0.305, 0.0, -0.075, 0.0);
        d.fly_until(contact::DockingPhase::Docked, 1200.0);
        return d;
    }
    // Beside the US module stack (station x = 45 m, Dragon axis along -x like at IDA-2), its 4 m heat shield `gap_m`
    // off the 2.2 m module surface on the station +y side, drifting towards it at `speed_mps`.
    static DragonAtPort beside_modules(double gap_m, double speed_mps) {
        // IDA-2 port frame = station (x, -y, -z), port at station x = 73 m
        return place("ida2", 45.0 - 73.0, -(2.2 + 2.0 + gap_m), 0.0, speed_mps);
    }

    // advance_to in 1 s calls until the docking phase is `phase` or `limit_s` after the current epoch.
    bool fly_until(contact::DockingPhase phase, double limit_s) {
        const double start = session.epoch() - t0;
        for (double t = std::round(start); t < start + limit_s; t += 1.0) {
            if (session.docking_phase() == phase) return true;
            session.advance_to(t0 + t + 1.0, 1000000);
        }
        return session.docking_phase() == phase;
    }
};

}  // namespace starsim::docking_test
