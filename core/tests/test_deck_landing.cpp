// Landing on the moving drone-ship deck (sea state and deck motion, Task 3): the legs' contact and judgement in the
// deck frame, the landed stage riding the deck, and the unchanged behaviour of a calm sea, the flag off and a pad.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "return_scenario.hpp"
#include "starsim/config/drone_ship_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/deck_motion.hpp"
#include "starsim/contact/landing_legs.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

const spice::SpiceEphemeris& de440() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

time::Epoch start() {
    test::spice_session();
    return spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 10, 40, 0.0});
}

// The drone ship of test_falcon9_asds.cpp: ~600 km north-east of the Cape, long axis north.
sim::LaunchSite ship() {
    return sim::drone_ship_site(config::load_drone_ship(STARSIM_CONFIG_DIR, "asds_atlantic"), 32.0 * deg, -76.0 * deg,
                                -35.0, 0.0);
}

sim::LaunchSite lz40() {
    return sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_lz40"));
}

config::VehicleConfig stage_config() { return test::booster_config("falcon9_block5", 1000.0); }

// The drop of test_falcon9_asds.cpp ("A stage set down on the deck lands..."); `configure` runs after the landing
// zone is set (the sea state, the flag).
void drop_on_site(sim::FlightSession& session, const sim::LaunchSite& site, double feet_height_m, double sink_mps,
                  const std::function<void(sim::FlightSession&)>& configure) {
    const auto cfg = stage_config();
    const auto t0 = start();
    session.set_launch_site(site);
    session.set_landing_zone(site);
    configure(session);
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const double foot = cfg.stages.front().landing_legs.foot_station_m;
    session.place_body(cfg, test::state_over_site(session.launch_site(), earth(), t0, {0.0, 0.0, feet_height_m + cg - foot},
                                                  {0.0, 0.0, -sink_mps}),
                       t0);
    session.set_leg_deployment(0, 1.0);
}

sim::LegTouchdown run_drop(sim::FlightSession& session) {
    const auto t0 = session.epoch();
    while (session.leg_touchdown(0) == sim::LegTouchdown::None && session.epoch() - t0 < 20.0) {
        session.advance_to(session.epoch() + 0.1);
    }
    return session.leg_touchdown(0);
}

struct DropEnd {
    sim::LegTouchdown touchdown;
    vehicle::RigidBodyState state;
};

// Drops, judges, then holds 5 s more (the landed hold).
DropEnd drop_and_hold(const sim::LaunchSite& site, const std::function<void(sim::FlightSession&)>& configure) {
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    drop_on_site(session, site, 0.5, 2.0, configure);
    const auto touchdown = run_drop(session);
    session.advance_to(session.epoch() + 5.0);
    return {touchdown, session.body(0).state()};
}

bool same_state(const vehicle::RigidBodyState& a, const vehicle::RigidBodyState& b) {
    return a.position_gcrf_m == b.position_gcrf_m && a.velocity_gcrf_mps == b.velocity_gcrf_mps &&
           a.attitude.w == b.attitude.w && a.attitude.x == b.attitude.x && a.attitude.y == b.attitude.y &&
           a.attitude.z == b.attitude.z && a.angular_velocity_body_radps == b.angular_velocity_body_radps;
}

// Body state seen from the leg site: CG and velocity relative to the Earth, site frame.
struct SiteState {
    Vec3 cg_m;
    Vec3 velocity_mps;
    math::Mat3 body_to_site;
};

SiteState site_state(const sim::FlightSession& session, const vehicle::FlightBody& body) {
    const auto t = earth().fixed_to_gcrf(session.epoch());
    const auto& legs = session.leg_landing();
    const auto& s = body.state();
    const math::Mat3 gcrf_to_site = transpose(legs.site_to_fixed()) * transpose(t.rotation);
    const Vec3 v_rel = s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), s.position_gcrf_m);
    return {legs.to_site(transpose(t.rotation) * s.position_gcrf_m), gcrf_to_site * v_rel,
            gcrf_to_site * s.attitude.to_matrix()};
}

// Largest distance of a foot from the deck plane and the velocity error against the deck point at the CG, now.
struct RideError {
    double foot_m = 0.0;
    double velocity_mps = 0.0;
};

RideError ride_error(const sim::FlightSession& session) {
    const auto& body = session.body(0);
    const auto& stage = body.vehicle().stages().front();
    const auto geometry =
        contact::make_leg_geometry(stage.config.landing_legs, stage.base_station_m, 0.5 * stage.config.diameter_m);
    const double cg_station = body.vehicle().mass_properties().cg_station_m;
    const auto pose = session.deck_pose();
    REQUIRE(pose);
    const auto deck = contact::deck_frame(*pose, ship().deck->heading_rad);
    const Vec3 normal = column(deck.deck_to_site, 2);
    const auto s = site_state(session, body);
    // a crushed core shortens its leg: the foot point is the deployed one moved up by the crush stroke (along the
    // deck normal as in contact::leg_contact)
    const auto crushed = session.leg_crushed_m(0);
    RideError e;
    for (size_t i = 0; i < geometry.feet.size(); ++i) {
        const Vec3 p = s.cg_m + s.body_to_site * (geometry.feet[i] - Vec3{cg_station, 0.0, 0.0});
        const double crush = i < crushed.size() ? crushed[i] : 0.0;
        e.foot_m = std::max(e.foot_m, std::abs(dot(normal, p - deck.origin_site_m) + crush));
    }
    const Vec3 deck_point = deck.velocity_site_mps + cross(deck.angular_velocity_site_radps, s.cg_m - deck.origin_site_m);
    e.velocity_mps = norm(s.velocity_mps - deck_point);
    return e;
}

}  // namespace

TEST_CASE("A fixed deck gives exactly today's contact", "[legs][deck_motion][asds]") {
    const auto today = drop_and_hold(ship(), [](sim::FlightSession&) {});
    const auto calm = drop_and_hold(ship(), [](sim::FlightSession& s) {
        s.set_sea_state(environment::SeaStateConfig{});
        CHECK_FALSE(s.deck_pose());  // calm: no deck motion
    });
    const auto flag_off = drop_and_hold(ship(), [](sim::FlightSession& s) {
        s.set_sea_state(environment::SeaStateConfig{2.0, 8.0, 45.0 * deg, 1});
        s.set_deck_motion(false);
        CHECK_FALSE(s.deck_motion());
        CHECK_FALSE(s.deck_pose());
    });
    CHECK(today.touchdown == sim::LegTouchdown::Landed);
    CHECK(calm.touchdown == today.touchdown);
    CHECK(flag_off.touchdown == today.touchdown);
    CHECK(same_state(calm.state, today.state));
    CHECK(same_state(flag_off.state, today.state));
}

namespace {

// Hs 2 m / Tp 8 s, waves 45 deg off the bow; the stage dropped 1 m above the deck centre at 1 m/s relative sink,
// engines off, until it has landed.
void land_on_heaving_deck(sim::FlightSession& session) {
    const auto t0 = start();
    session.advance_to(t0);  // no bodies: the clock jumps (the sea's time origin is taken from it)
    session.set_launch_site(ship());
    session.set_landing_zone(ship());
    session.set_sea_state(environment::SeaStateConfig{2.0, 8.0, 45.0 * deg, 1});
    CHECK(session.deck_motion());
    REQUIRE(session.sea_state());
    const auto pose = session.deck_pose();
    REQUIRE(pose);
    // the stage 1 m above the deck centre, sinking at 1 m/s relative to it, engines off
    const auto cfg = stage_config();
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const double foot = cfg.stages.front().landing_legs.foot_station_m;
    session.place_body(cfg, test::state_over_site(session.launch_site(), earth(), t0,
                                                  {0.0, 0.0, pose->heave_m + 1.0 + cg - foot},
                                                  {0.0, 0.0, pose->heave_rate_mps - 1.0}),
                       t0);
    session.set_leg_deployment(0, 1.0);
    REQUIRE(run_drop(session) == sim::LegTouchdown::Landed);
    CHECK(session.landing_outcome(0) == sim::LandingOutcome::Landed);
    const auto crushed = session.leg_crushed_m(0);
    std::printf("[deck] landed after %.2f s, crush strokes", session.epoch() - t0);
    for (double c : crushed) std::printf(" %.3f", c);
    std::printf(" m\n");
}

}  // namespace

TEST_CASE("A stage set down on a heaving deck lands and rides it", "[legs][deck_motion][asds]") {
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    land_on_heaving_deck(session);

    RideError worst;
    double heave_min = 1e9, heave_max = -1e9;
    auto check = [&] {
        const auto e = ride_error(session);
        worst.foot_m = std::max(worst.foot_m, e.foot_m);
        worst.velocity_mps = std::max(worst.velocity_mps, e.velocity_mps);
        heave_min = std::min(heave_min, session.deck_pose()->heave_m);
        heave_max = std::max(heave_max, session.deck_pose()->heave_m);
        CHECK(e.foot_m < 0.05);
        CHECK(e.velocity_mps < 0.01);
    };
    const auto landed = session.epoch();
    while (session.epoch() - landed < 120.0) {  // tick by tick
        session.advance_to(session.epoch() + 0.5);
        check();
    }
    std::printf("[deck] ticks: foot %.4f m, velocity %.5f m/s, heave %.2f .. %.2f m\n", worst.foot_m,
                worst.velocity_mps, heave_min, heave_max);
    for (int chunk = 0; chunk < 4; ++chunk) {  // the time-warp path: one coast per call
        session.advance_to(session.epoch() + 30.0);
        check();
    }
    std::printf("[deck] with coasts: foot %.4f m, velocity %.5f m/s\n", worst.foot_m, worst.velocity_mps);
    CHECK(heave_max - heave_min > 0.3);  // the deck did move
    CHECK(session.body(0).resting());
}

TEST_CASE("The touchdown is judged relative to the deck", "[legs][deck_motion][asds]") {
    // Abeam waves roll the deck; the stage is set down slowly at a roll peak of ~3 deg.
    const auto cfg = stage_config();
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const double foot = cfg.stages.front().landing_legs.foot_station_m;
    const double radius = cfg.stages.front().landing_legs.footprint_radius_m;
    auto set_down = [&](double tilt_to_deck_rad) {
        sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
        session.advance_to(start());
        session.set_launch_site(ship());
        session.set_landing_zone(ship());
        session.set_sea_state(environment::SeaStateConfig{2.5, 8.0, 90.0 * deg, 1});
        // the first roll peak between 2.7 and 3.3 deg
        double previous = 0.0;
        bool rising = false;
        for (int i = 0; i < 20000; ++i) {
            const double roll = std::abs(session.deck_pose()->roll_rad);
            if (rising && roll < previous && previous > 2.7 * deg && previous < 3.3 * deg) break;
            rising = roll > previous;
            previous = roll;
            session.advance_to(session.epoch() + 0.05);
        }
        const auto pose = *session.deck_pose();
        REQUIRE(std::abs(pose.roll_rad) > 2.6 * deg);
        REQUIRE(std::abs(pose.roll_rad) < 3.4 * deg);
        const auto deck = contact::deck_frame(pose, ship().deck->heading_rad);
        const Vec3 n = column(deck.deck_to_site, 2);
        const Vec3 d = math::normalized(Vec3{n.x, n.y, 0.0});  // where the deck normal leans
        // tilted away from the lean: tilt_to_deck - roll from the vertical
        const Vec3 e = math::normalized(-1.0 * d - dot(-1.0 * d, n) * n);
        const Vec3 axis = std::cos(tilt_to_deck_rad) * n + std::sin(tilt_to_deck_rad) * e;
        const double tilt_from_vertical = std::acos(axis.z);
        const auto attitude = tilt_from_vertical > 0.0
                                  ? math::Quat::from_axis_angle(math::normalized(cross(Vec3{0.0, 0.0, 1.0}, axis)),
                                                                tilt_from_vertical)
                                  : math::Quat{};
        // lowest foot 0.05 m above the deck plane, sinking at 0.3 m/s relative to the deck
        const double above = 0.05 + (cg - foot) * std::cos(tilt_to_deck_rad) + radius * std::sin(tilt_to_deck_rad);
        const Vec3 cg_site = deck.origin_site_m + above * n;
        const Vec3 velocity = deck.velocity_site_mps + cross(deck.angular_velocity_site_radps, cg_site - deck.origin_site_m) -
                              0.3 * n;
        session.place_body(cfg, test::state_over_site(session.launch_site(), earth(), session.epoch(), cg_site, velocity,
                                                      attitude),
                           session.epoch());
        session.set_leg_deployment(0, 1.0);
        const auto placed = session.epoch();
        const auto result = run_drop(session);
        std::printf("[deck] roll %.2f deg, stage %.1f deg to the deck (%.1f deg from the vertical): %s after %.2f s\n",
                    pose.roll_rad / deg, tilt_to_deck_rad / deg, tilt_from_vertical / deg, sim::to_string(result),
                    session.epoch() - placed);
        return std::pair{result, session.epoch() - placed};
    };
    // upright to the deck: not tipped over although ~3 deg from the vertical
    CHECK(set_down(0.0).first != sim::LegTouchdown::TippedOver);
    // 32 deg to the deck normal (beyond the 30 deg tip-over angle), only ~29 deg from the vertical: tipped over at the
    // first foot contact (an Earth-vertical judgement would not end it there)
    const auto [tipped, after_s] = set_down(32.0 * deg);
    CHECK(tipped == sim::LegTouchdown::TippedOver);
    CHECK(after_s < 1.0);
}

TEST_CASE("A pad landing ignores the sea state", "[legs][deck_motion]") {
    const auto today = drop_and_hold(lz40(), [](sim::FlightSession&) {});
    const auto rough = drop_and_hold(lz40(), [](sim::FlightSession& s) {
        s.set_sea_state(environment::SeaStateConfig{3.0, 8.0, 0.0, 1});
        CHECK_FALSE(s.deck_pose());  // no deck
    });
    CHECK(today.touchdown == sim::LegTouchdown::Landed);
    CHECK(rough.touchdown == today.touchdown);
    CHECK(same_state(rough.state, today.state));
}

TEST_CASE("The sea beside a moving deck stays at the mean sea level", "[legs][deck_motion]") {
    // Deck heading north, heaved 0.5 m and rolled 3 deg starboard (east) down: 50 m west of the centre line the
    // deck-attached frame stands ~2.6 m higher than the site frame, 50 m east ~2.6 m lower.
    const auto deck = *ship().deck;
    contact::DeckPose pose;
    pose.heave_m = 0.5;
    pose.roll_rad = 3.0 * deg;
    const auto f = contact::deck_frame(pose, deck.heading_rad);
    const auto level = contact::deck_frame(contact::DeckPose{}, deck.heading_rad).deck_to_site;
    const math::Mat3 carrier_to_site = f.deck_to_site * transpose(level);
    contact::LandingSurface moving;
    moving.field = std::make_shared<contact::DeckSeaField>(deck, carrier_to_site, f.origin_site_m);
    contact::LandingSurface fixed;  // the deck's own water, at -freeboard in whatever frame it is evaluated
    fixed.deck = deck;
    // water at a point given in site coordinates: a base-ring point there (leg_contact's water flag)
    auto water = [&](const contact::LandingSurface& surface, const Vec3& site_m) {
        contact::LegGeometry g;
        g.base_ring = {Vec3{}};
        contact::SitePose p;
        p.cg_m = transpose(carrier_to_site) * (site_m - f.origin_site_m);  // carrier frame
        std::vector<double> crushed;
        return contact::leg_contact(g, crushed, p, surface).water;
    };
    const double sea = -deck.freeboard_m;
    {  // west: 0.1 m above the mean sea, but below -freeboard in the carrier frame
        const Vec3 site{-50.0, 0.0, sea + 0.1};
        REQUIRE((transpose(carrier_to_site) * (site - f.origin_site_m)).z < sea);
        CHECK_FALSE(water(moving, site));
        CHECK(water(fixed, site));  // the sea moving with the deck would have caught it
    }
    {  // east: 0.1 m below the mean sea, but above -freeboard in the carrier frame
        const Vec3 site{50.0, 0.0, sea - 0.1};
        REQUIRE((transpose(carrier_to_site) * (site - f.origin_site_m)).z > sea);
        CHECK(water(moving, site));
        CHECK_FALSE(water(fixed, site));
    }
    // on the deck the support is the deck itself (z = 0 of the carrier frame)
    const auto on_deck = moving.below(10.0, 20.0);
    CHECK_FALSE(on_deck.water);
    CHECK(on_deck.height_m == 0.0);
}

TEST_CASE("Changing the sea under a landed stage moves it without a jump", "[legs][deck_motion][asds]") {
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    land_on_heaving_deck(session);
    session.advance_to(session.epoch() + 10.0);
    auto cg_site = [&] { return site_state(session, session.body(0)).cg_m; };
    auto ride_velocity_error = [&] {
        double worst = 0.0;
        for (int i = 0; i < 20; ++i) {
            session.advance_to(session.epoch() + 0.5);
            worst = std::max(worst, ride_error(session).velocity_mps);
        }
        return worst;
    };
    // the flag off: held fixed where it is, at rest relative to the Earth (tolerances: the test sees the Earth through
    // SPICE, the session through its interpolated orientation, sub-mm)
    Vec3 before = cg_site();
    session.set_deck_motion(false);
    CHECK_FALSE(session.deck_pose());
    CHECK(norm(cg_site() - before) < 1e-6);
    CHECK(norm(site_state(session, session.body(0)).velocity_mps) < 1e-5);
    before = cg_site();
    session.advance_to(session.epoch() + 10.0);
    CHECK(norm(cg_site() - before) < 1e-3);
    // on again: no jump, then it rides the deck from its pose relative to the deck now
    before = cg_site();
    session.set_deck_motion(true);
    REQUIRE(session.deck_pose());
    CHECK(norm(cg_site() - before) < 1e-6);
    double error = ride_velocity_error();
    std::printf("[deck] after the flag on: velocity error %.6f m/s\n", error);
    CHECK(error < 0.01);
    // a new sea (another time origin and spectrum): no jump, rides the new deck motion
    before = cg_site();
    session.set_sea_state(environment::SeaStateConfig{1.5, 7.0, 90.0 * deg, 2});
    CHECK(norm(cg_site() - before) < 1e-6);
    error = ride_velocity_error();
    std::printf("[deck] after the new sea: velocity error %.6f m/s\n", error);
    CHECK(error < 0.01);
    // no sea: held fixed where it is
    before = cg_site();
    session.set_sea_state(std::nullopt);
    CHECK(norm(cg_site() - before) < 1e-6);
    CHECK(norm(site_state(session, session.body(0)).velocity_mps) < 1e-5);
    before = cg_site();
    session.advance_to(session.epoch() + 30.0);  // a coast
    CHECK(norm(cg_site() - before) < 1e-3);
    CHECK(session.body(0).resting());
}
