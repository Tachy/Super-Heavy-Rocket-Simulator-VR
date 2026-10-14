// Crew Dragon under its parachutes in the flight session (d4 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md):
// from the drogue gate to the water; review-focus cases; the return mission from orbit with the published timeline.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/config/splashdown_zone_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/parachute_sequencer.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

const spice::SpiceEphemeris& spice_ephemeris() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& spice_earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

config::VehicleConfig capsule_config(double propellant_kg = 1400.0) {
    auto c = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
    c.stages.erase(c.stages.begin());
    for (auto& t : c.stages.front().tanks) t.mass_kg = propellant_kg;
    return c;
}

// The capsule `height_m` above the zone centre, `speed_mps` relative to the Earth, `gamma_rad` below the horizontal
// heading north, heat shield first (body -x along the motion).
vehicle::RigidBodyState falling_state(const config::SplashdownZoneConfig& zone, double height_m, double speed_mps,
                                      double gamma_rad, time::Epoch epoch) {
    const Vec3 r = frames::geodetic_to_fixed({zone.latitude_rad, zone.longitude_rad, height_m});
    const Vec3 up = math::normalized(r);
    const Vec3 east{-std::sin(zone.longitude_rad), std::cos(zone.longitude_rad), 0.0};
    const Vec3 north = cross(up, east);
    const Vec3 v_rel = (north * std::cos(gamma_rad) - up * std::sin(gamma_rad)) * speed_mps;
    const auto t = spice_earth().fixed_to_gcrf(epoch);
    vehicle::RigidBodyState s;
    s.position_gcrf_m = t.rotation * r;
    s.velocity_gcrf_mps = t.rotation * v_rel + cross(vehicle::earth_rotation_vector(t), s.position_gcrf_m);
    const Vec3 x = math::normalized(t.rotation * v_rel) * -1.0;  // body +x against the motion
    const Vec3 y = math::normalized(cross(t.rotation * east, x));
    const Vec3 z = cross(x, y);
    math::Mat3 m;
    const Vec3 columns[3] = {x, y, z};
    for (int c = 0; c < 3; ++c) {
        m(0, c) = columns[c].x;
        m(1, c) = columns[c].y;
        m(2, c) = columns[c].z;
    }
    s.attitude = math::Quat::from_matrix(m).normalized();
    return s;
}

struct Descent {
    double drogue_height_m = -1.0, drogue_speed_mps = -1.0, drogue_s = -1.0;
    double main_height_m = -1.0, main_speed_mps = -1.0, main_s = -1.0;
    double splashdown_s = -1.0, sink_mps = -1.0, miss_m = -1.0, peak_g = 0.0;
    sim::LandingOutcome outcome = sim::LandingOutcome::Flying;
};

// Flies the controlled body (the capsule) of `session` until it is down or `max_s` passed, in `step_s` steps; records
// the events (deployment heights as the sequencer saw them).
Descent fly_down(sim::FlightSession& session, double max_s = 900.0, double step_s = 0.05) {
    Descent d;
    const auto t0 = session.epoch();
    const size_t i = session.controlled_index();
    while (session.epoch() - t0 < max_s) {
        session.advance_to(session.epoch() + step_s, 1000000);
        const auto& body = session.body(i);
        const auto tel = session.telemetry(i);
        d.peak_g = std::max(d.peak_g, norm(body.aero_acceleration_body_mps2()) / constants::standard_gravity_mps2);
        const auto* seq = session.parachute_sequencer(i);
        if (seq && d.drogue_s < 0.0 && seq->drogue_height_m()) {
            d.drogue_s = session.epoch() - t0;
            d.drogue_height_m = *seq->drogue_height_m();
            d.drogue_speed_mps = tel.speed_relative_mps;
        }
        if (seq && d.main_s < 0.0 && seq->main_height_m()) {
            d.main_s = session.epoch() - t0;
            d.main_height_m = *seq->main_height_m();
            d.main_speed_mps = tel.speed_relative_mps;
        }
        if (body.resting() || body.impacted()) {
            d.splashdown_s = session.epoch() - t0;
            break;
        }
    }
    const auto& body = session.body(i);
    if (body.touchdown_sink_mps()) d.sink_mps = *body.touchdown_sink_mps();
    d.outcome = session.landing_outcome(i);
    if (session.splashdown_zone()) d.miss_m = session.splashdown_miss_m(i);
    return d;
}

struct Scenario {
    sim::FlightSession session{spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR};
    config::SplashdownZoneConfig zone = config::load_splashdown_zone(STARSIM_CONFIG_DIR, "pacific_san_diego");
    time::Epoch t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 14, 6, 0, 0.0});
    Scenario(double height_m, double speed_mps, double gamma_rad, const config::VehicleConfig& cfg = capsule_config()) {
        session.set_splashdown_zone(zone);
        session.place_body(cfg, falling_state(zone, height_m, speed_mps, gamma_rad, t0), t0);
    }
};

}  // namespace

TEST_CASE("From the drogue gate the capsule splashes down under its mains", "[dragon][splashdown][session]") {
    Scenario s(5600.0, 166.0, 60.0 * deg);  // d3's gate: 166 m/s, steep
    const auto d = fly_down(s.session);
    std::printf("[validation] gate to water: drogues %.2f km at %.0f m/s, mains %.2f km at %.0f m/s after %.0f s, "
                "splashdown %.0f s after the mains at %.1f m/s, %.2f km from the zone centre, peak %.2f g\n",
                d.drogue_height_m / 1000.0, d.drogue_speed_mps, d.main_height_m / 1000.0, d.main_speed_mps,
                d.main_s - d.drogue_s, d.splashdown_s - d.main_s, d.sink_mps, d.miss_m / 1000.0, d.peak_g);
    CHECK(d.drogue_height_m == Catch::Approx(5500.0).epsilon(0.02));
    CHECK(d.main_height_m == Catch::Approx(1800.0).epsilon(0.02));
    CHECK(d.main_speed_mps == Catch::Approx(53.0).epsilon(0.15));          // [S] 119 mph
    CHECK(d.main_s - d.drogue_s < 90.0);                                    // [S] "less than a minute later";
    CHECK(d.main_s - d.drogue_s > 40.0);                                    //     Demo-2 2:44 -> 2:45 (minutes)
    CHECK(d.splashdown_s - d.main_s == Catch::Approx(180.0).epsilon(0.35));  // [S] Demo-2 2:45 -> 2:48
    CHECK(d.sink_mps == Catch::Approx(7.0).epsilon(0.30));                  // [S] 15-16 mph, spec tolerance 30 %
    CHECK(d.outcome == sim::LandingOutcome::SplashedDown);
    CHECK(d.peak_g < 3.6);
    CHECK_FALSE(s.session.body(0).crew_load_warning());
    CHECK_FALSE(s.session.body(0).vehicle().parachute_torn());
    CHECK(d.miss_m < 2000.0);  // the drift from 5.6 km over the zone stays inside the 2 km zone
}

TEST_CASE("The Dracos stay off under the parachutes", "[dragon][splashdown][session]") {
    Scenario s(5600.0, 166.0, 60.0 * deg);
    double rcs_after_drogues = -1.0;
    const auto t0 = s.session.epoch();
    while (s.session.epoch() - t0 < 120.0) {
        s.session.advance_to(s.session.epoch() + 0.05, 1000000);
        const auto* seq = s.session.parachute_sequencer(0);
        if (seq && seq->phase() != gnc::ParachutePhase::Armed && rcs_after_drogues < 0.0) {
            rcs_after_drogues = s.session.body(0).vehicle().rcs_propellant_kg();
        }
    }
    REQUIRE(rcs_after_drogues > 0.0);
    CHECK(s.session.body(0).vehicle().rcs_propellant_kg() == Catch::Approx(rcs_after_drogues).margin(1e-9));
}

TEST_CASE("Drogues open at their height under time warp", "[dragon][splashdown][session]") {
    Scenario s(12000.0, 200.0, 80.0 * deg);
    const auto d = fly_down(s.session, 900.0, 30.0);  // 30 s steps: coast segments wherever the session allows them
    const auto* seq = s.session.parachute_sequencer(0);
    REQUIRE(seq != nullptr);
    REQUIRE(seq->drogue_height_m());
    CHECK(*seq->drogue_height_m() == Catch::Approx(5500.0).margin(50.0));  // one tick of fall at most, not a coast
    REQUIRE(seq->main_height_m());
    CHECK(*seq->main_height_m() == Catch::Approx(1800.0).margin(10.0));
    CHECK(d.outcome == sim::LandingOutcome::SplashedDown);
}

TEST_CASE("A capsule placed below the main height opens its mains at once", "[dragon][splashdown][session]") {
    Scenario s(1200.0, 45.0, 85.0 * deg);
    s.session.advance_to(s.session.epoch() + 0.1);
    const auto& chutes = s.session.body(0).vehicle().stages().front().parachutes;
    CHECK(chutes[0].state == vehicle::ParachuteState::Stowed);
    CHECK(chutes[1].state == vehicle::ParachuteState::Open);
    const auto d = fly_down(s.session);
    CHECK(d.outcome == sim::LandingOutcome::SplashedDown);
}

TEST_CASE("Drogues held back by dynamic pressure: the mains tear and the capsule hits the water",
          "[dragon][splashdown][session]") {
    auto cfg = capsule_config();
    cfg.stages.front().parachutes[0].max_deploy_dynamic_pressure_pa = 100.0;  // never reached: the drogues never go
    Scenario s(5600.0, 166.0, 60.0 * deg, cfg);
    const auto d = fly_down(s.session);
    const auto& mains = s.session.body(0).vehicle().stages().front().parachutes[1];
    std::printf("no drogues: mains at %.0f m / %.0f m/s, peak %.0f kN per canopy, peak %.2f g, %s at %.1f m/s\n",
                d.main_height_m, d.main_speed_mps, mains.peak_load_n / 1000.0, d.peak_g, sim::to_string(d.outcome), d.sink_mps);
    CHECK(d.drogue_s < 0.0);
    CHECK(s.session.body(0).vehicle().parachute_torn());
    CHECK(d.sink_mps > 9.0);
    CHECK(d.outcome == sim::LandingOutcome::Impact);
}

TEST_CASE("A capsule on its trunk has no parachute sequencer", "[dragon][splashdown][session]") {
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    const auto zone = config::load_splashdown_zone(STARSIM_CONFIG_DIR, "pacific_san_diego");
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 14, 6, 0, 0.0});
    session.set_splashdown_zone(zone);
    session.place_body(config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon"),
                       falling_state(zone, 5000.0, 150.0, 60.0 * deg, t0), t0);
    session.advance_to(t0 + 1.0);
    CHECK(session.parachute_sequencer(0) == nullptr);
    CHECK_FALSE(session.body(0).vehicle().parachutes_out());
}
