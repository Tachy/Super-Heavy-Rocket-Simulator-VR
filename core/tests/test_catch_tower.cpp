// Catch tower scenarios: a booster placed above the arms with its engines off, full force model, real tower
// contact. The arms are closed beforehand where the scenario needs it (no guidance involved).

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/primitives.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/catch_tower_sim.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const spice::SpiceEphemeris& spice_ephemeris() {
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& spice_earth() {
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

time::Epoch start() {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    return time.from_utc({2026, 10, 12, 12, 0, 0.0});
}

// FlightSession is not movable in practice (its environment refers to its own members): set it up in place.
void use_starbase(sim::FlightSession& session) {
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
}

// Block 2 booster with 4 t of propellant left, upright over the catch axis, pins facing the arms. `pin_offset_m`
// (site frame) moves the pins' midpoint from the catch point (arm tops on the catch axis); velocity in the site
// frame, relative to the ground.
void place_booster(sim::FlightSession& session, time::Epoch epoch, const Vec3& pin_offset_m, const Vec3& velocity_mps) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block2");
    cfg.stages.resize(1);
    for (auto& tank : cfg.stages[0].tanks) tank.mass_kg = 2000.0;
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const auto& tower = session.tower()->geometry();
    const Vec3 up{0.0, 0.0, 1.0};
    const double azimuth = cfg.stages[0].catch_pins.azimuths_rad.front();
    // Body +y such that the first pin (azimuth from +y towards +z) points along +across.
    const Vec3 y = tower.across() * std::cos(azimuth) - cross(up, tower.across()) * std::sin(azimuth);
    const math::Mat3 body_to_site = contact::from_columns(up, y, cross(up, y));
    const double pin_station = cfg.stages[0].catch_pins.station_m;
    const Vec3 cg_site = up * tower.arm_top_height_m(0.0) + pin_offset_m - up * (pin_station - cg);
    const auto t = spice_earth().fixed_to_gcrf(epoch);
    const Vec3 r_itrf = tower.to_itrf(cg_site);
    vehicle::RigidBodyState s;
    s.position_gcrf_m = t.position(r_itrf);
    s.velocity_gcrf_mps = t.velocity(r_itrf, tower.site_to_itrf() * velocity_mps);
    s.attitude = math::Quat::from_matrix(t.rotation * tower.site_to_itrf() * body_to_site).normalized();
    s.angular_velocity_body_radps = s.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    session.place_body(cfg, s, epoch);
    session.set_control_mode(sim::ControlMode::DirectGimbal);  // no stability augmentation: engines and RCS off
}

// Advances in 0.1 s steps until `done` or `seconds`.
template <class Done>
void run(sim::FlightSession& session, time::Epoch t0, double seconds, Done done) {
    for (double t = 0.1; t <= seconds && !done(); t += 0.1) session.advance_to(t0 + t, 1000);
}

}  // namespace

TEST_CASE("Centred slow arrival: the booster is caught and hangs on the arms", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    session.tower()->close_arms(true);
    place_booster(session, t0, {0.0, 0.0, 0.02}, {0.0, 0.0, -1.0});
    const auto& booster = session.body(0);
    run(session, t0, 30.0, [&] { return booster.caught() || booster.impacted(); });
    INFO("max pin load " << session.tower()->max_pin_load_n());
    REQUIRE(booster.caught());
    CHECK(session.tower()->phase() == sim::CatchPhase::Caught);
    const auto at = [&](time::Epoch e) {
        return transpose(spice_earth().fixed_to_gcrf(e).rotation) * booster.state().position_gcrf_m;
    };
    const auto caught_at = session.epoch();
    const Vec3 r0 = at(caught_at);
    session.advance_to(caught_at + 60.0, 100000);
    CHECK(norm(at(session.epoch()) - r0) < 1e-6);  // fixed in ITRF
}

TEST_CASE("Off-centre arrival beyond the trigger limit: the arms stay half open", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    const auto& tower = session.tower()->geometry();
    place_booster(session, t0, tower.across() * 2.5 + Vec3{0.0, 0.0, 3.0}, {0.0, 0.0, -1.0});
    const auto& booster = session.body(0);
    run(session, t0, 2.5, [&] { return booster.caught() || booster.impacted(); });
    CHECK_FALSE(booster.caught());
    CHECK(session.tower()->arms().opening_rad() > 0.0);
    CHECK(session.tower()->phase() != sim::CatchPhase::Closing);
}

TEST_CASE("Fast arrival overloads the pins", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    session.tower()->close_arms(true);
    place_booster(session, t0, {0.0, 0.0, 0.02}, {0.0, 0.0, -8.0});
    const auto& booster = session.body(0);
    run(session, t0, 10.0, [&] { return booster.caught() || booster.impacted(); });
    CHECK(booster.impacted());
    CHECK(session.tower()->phase() == sim::CatchPhase::Missed);
}

TEST_CASE("Drifting into the tower is an impact", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    const auto& tower = session.tower()->geometry();
    place_booster(session, t0, tower.tower_direction() * 12.0 + Vec3{0.0, 0.0, 20.0}, tower.tower_direction() * 5.0);
    const auto& booster = session.body(0);
    run(session, t0, 10.0, [&] { return booster.impacted(); });
    CHECK(booster.impacted());
    CHECK_FALSE(booster.caught());
}

namespace {

// Booster velocity relative to the ground in the site frame.
Vec3 site_velocity(const sim::FlightSession& session, const vehicle::FlightBody& body) {
    const auto t = spice_earth().fixed_to_gcrf(session.epoch());
    const Vec3 omega = vehicle::earth_rotation_vector(t);
    const auto& s = body.state();
    return transpose(session.tower()->geometry().site_to_itrf()) * transpose(t.rotation) *
           (s.velocity_gcrf_mps - cross(omega, s.position_gcrf_m));
}

}  // namespace

TEST_CASE("The carriage keeps carrying a caught booster", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    session.tower()->close_arms(true);
    place_booster(session, t0, {0.0, 0.0, 0.02}, {0.0, 0.0, -1.0});
    const auto& booster = session.body(0);
    run(session, t0, 30.0, [&] { return booster.caught(); });
    REQUIRE(booster.caught());
    const double sag = booster.vehicle().mass_properties().mass_kg * constants::standard_gravity_mps2 /
                       session.tower()->geometry().config().carriage_stiffness_n_m;
    const auto caught_at = session.epoch();
    for (double t = 0.5; t <= 30.0; t += 0.5) session.advance_to(caught_at + t, 1000);  // ticks, no coast
    CHECK(std::abs(session.tower()->arms().carriage_offset_m() + sag) < 0.05 * sag);
}

TEST_CASE("Pin friction does not chatter from tick to tick", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    session.tower()->close_arms(true);
    place_booster(session, t0, {0.0, 0.0, 0.02}, {0.0, 0.0, -1.0});
    const auto& booster = session.body(0);
    session.advance_to(t0 + 0.5, 1000);  // pins on the arms
    Vec3 previous = site_velocity(session, booster);
    double largest_change = 0.0;
    for (int k = 1; k <= 200 && !booster.caught(); ++k) {
        session.advance_to(t0 + 0.5 + k * sim::FlightSession::tick_s, 10);
        const Vec3 v = site_velocity(session, booster);
        largest_change = std::max(largest_change, std::hypot(v.x - previous.x, v.y - previous.y));
        previous = v;
    }
    INFO("largest tick-to-tick change of the horizontal velocity " << largest_change << " m/s");
    CHECK(largest_change < 1e-3);
}

TEST_CASE("A booster coasting down from above the tower zone is judged by the tower", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    place_booster(session, t0, {0.0, 0.0, 1200.0}, {0.0, 0.0, 0.0});
    const auto& booster = session.body(0);
    session.advance_to(t0 + 90.0, 100000);  // one call: coast segments allowed (each > 1 s)
    CHECK((booster.impacted() || booster.resting()));
    CHECK(session.tower()->phase() == sim::CatchPhase::Missed);
}

TEST_CASE("A stack released without thrust stays on the launch mount", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    session.launch_from_pad("starship_block3",
                            sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")),
                            90.0 * constants::deg_to_rad, t0);
    session.release_clamps();
    for (double t = 0.5; t <= 10.0; t += 0.5) session.advance_to(t0 + t, 1000);
    CHECK_FALSE(session.body(0).impacted());
    CHECK(session.body(0).resting());
}

TEST_CASE("An aborted catch keeps the arms open", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    session.tower()->abort_catch();
    place_booster(session, t0, {0.0, 0.0, 3.0}, {0.0, 0.0, -1.0});  // would be caught (see the centred arrival)
    const auto& booster = session.body(0);
    // 3 s: the pins pass the arm tops (an engines-off fall onto the mount below would strike the tower).
    run(session, t0, 3.0, [&] { return booster.caught() || booster.impacted() || booster.resting(); });
    CHECK_FALSE(booster.caught());
    CHECK(session.tower()->phase() == sim::CatchPhase::Aborted);
    CHECK(session.tower()->arms().opening_rad() > 0.0);
}

TEST_CASE("Striking the tower is recorded", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    const auto& tower = session.tower()->geometry();
    place_booster(session, t0, tower.tower_direction() * 12.0 + Vec3{0.0, 0.0, 20.0}, tower.tower_direction() * 5.0);
    const auto& booster = session.body(0);
    run(session, t0, 10.0, [&] { return booster.impacted(); });
    CHECK(session.tower()->tower_struck());
}
