// Engine groups placed up the hull and canted towards the axis, and landing-only groups (lunar l2: the HLS landing
// thrusters).

#include <algorithm>
#include <cmath>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

config::VehicleConfig ship() {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    return cfg;
}

// The ship with one extra canted ring of four Raptors at 40 m (a test vehicle, not the HLS).
config::VehicleConfig canted_ship(bool landing_only) {
    auto cfg = ship();
    config::EngineGroupConfig g;
    g.id = "canted";
    g.engine_id = "raptor3";
    g.count = 4;
    g.ring_radius_m = 4.6;
    g.station_m = 40.0;
    g.cant_rad = 15.0 * deg;
    g.relight = true;
    g.landing_only = landing_only;
    cfg.stages.front().engine_groups.push_back(g);
    return cfg;
}

// A session with `cfg` in a 200 km circular orbit (two-body), nose prograde.
struct OrbitSession {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    sim::FlightSession session;
    time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    math::StateVector orbit_state;

    static environment::PointMassGravityConfig two_body() {
        environment::PointMassGravityConfig c;
        c.moon = c.sun = c.earth_j2 = false;
        return c;
    }
    explicit OrbitSession(const config::VehicleConfig& cfg)
        : session(ephemeris, earth, STARSIM_CONFIG_DIR, two_body()) {
        orbit_state = orbit::state_from_elements({6378137.0 + 200e3, 0.0, 26.5 * deg, 0.0, 0.0, 0.0},
                                                 session.gm_earth_m3ps2());
        const Vec3 x = math::normalized(orbit_state.velocity_mps);
        const Vec3 z = -math::normalized(orbit_state.position_m);
        const Vec3 y = cross(z, x);
        math::Mat3 m;
        const Vec3 columns[3] = {x, y, z};
        for (int c = 0; c < 3; ++c) {
            m(0, c) = columns[c].x;
            m(1, c) = columns[c].y;
            m(2, c) = columns[c].z;
        }
        session.place_body(cfg, {orbit_state.position_m, orbit_state.velocity_mps, math::Quat::from_matrix(m), {}}, t0);
    }
};

}  // namespace

TEST_CASE("A canted ring up the hull thrusts along the axis without a moment", "[vehicle][engines]") {
    const vehicle::Vehicle v(canted_ship(false), STARSIM_CONFIG_DIR);
    Vec3 force, moment;
    const double cg = v.mass_properties().cg_station_m;
    int n = 0;
    for (const auto& e : v.stages().front().engines) {
        if (e.group_id != "ship/canted") continue;
        ++n;
        CHECK(e.gimbal_point_m.x == Catch::Approx(40.0));
        CHECK(std::hypot(e.gimbal_point_m.y, e.gimbal_point_m.z) == Catch::Approx(4.6));
        const Vec3 d = e.thrust_direction();
        CHECK(d.x == Catch::Approx(std::cos(15.0 * deg)));
        CHECK(d.y * e.gimbal_point_m.y + d.z * e.gimbal_point_m.z < 0.0);  // towards the axis
        force += d;
        moment += cross(e.gimbal_point_m - Vec3{cg, 0.0, 0.0}, d);
    }
    CHECK(n == 4);
    CHECK(force.x == Catch::Approx(4.0 * std::cos(15.0 * deg)));
    CHECK(std::hypot(force.y, force.z) < 1e-12);
    CHECK(norm(moment) < 1e-9);
}

TEST_CASE("Engines without a station or cant are placed as before", "[vehicle][engines]") {
    const vehicle::Vehicle v(config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3"), STARSIM_CONFIG_DIR);
    for (const auto& e : v.stages().back().engines) {
        CHECK(e.gimbal_point_m.x == Catch::Approx(v.stages().back().base_station_m));
        CHECK(e.thrust_direction().x == 1.0);
        CHECK(!e.auxiliary());
    }
}

TEST_CASE("A canted engine group cannot gimbal", "[vehicle][engines][config]") {
    auto cfg = canted_ship(false);
    cfg.stages.front().engine_groups.back().gimbal = true;
    CHECK_THROWS(vehicle::Vehicle(cfg, STARSIM_CONFIG_DIR));
}

TEST_CASE("Landing-only engines are never chosen for burns or lit by ignite", "[sim][engines]") {
    OrbitSession plain(ship());
    OrbitSession ringed(canted_ship(true));
    OrbitSession counted(canted_ship(false));  // the same ring as a main group: chosen (the test discriminates)
    const auto node_epoch = plain.t0 + 300.0;
    const auto at_node = orbit::propagate_two_body(plain.orbit_state, plain.session.gm_earth_m3ps2(), 300.0);
    const gnc::BurnNode node{node_epoch, math::normalized(at_node.velocity_mps) * 200.0};
    REQUIRE(plain.session.execute_burn(node));
    REQUIRE(ringed.session.execute_burn(node));
    REQUIRE(counted.session.execute_burn(node));
    CHECK(ringed.session.burn_configuration().groups == plain.session.burn_configuration().groups);
    const auto& with_main = counted.session.burn_configuration().groups;
    CHECK(std::find(with_main.begin(), with_main.end(), "ship/canted") != with_main.end());

    ringed.session.ignite();
    ringed.session.advance_to(ringed.t0 + 5.0);
    int landing = 0;
    for (const auto& e : ringed.session.body(0).vehicle().stages().front().engines) {
        if (!e.landing_only) continue;
        ++landing;
        CHECK(!e.producing_thrust());
    }
    CHECK(landing == 4);
}

TEST_CASE("A forced burn naming a landing-only group lights no landing engine", "[sim][engines]") {
    OrbitSession ringed(canted_ship(true));
    const auto at_node = orbit::propagate_two_body(ringed.orbit_state, ringed.session.gm_earth_m3ps2(), 300.0);
    const gnc::BurnNode node{ringed.t0 + 300.0, math::normalized(at_node.velocity_mps) * 200.0};
    CHECK_FALSE(ringed.session.execute_burn(node, {"canted"}));  // only a landing-only group: no configuration
    REQUIRE(ringed.session.execute_burn(node, {"vacuum", "canted"}));
    const auto& groups = ringed.session.burn_configuration().groups;
    CHECK(std::find(groups.begin(), groups.end(), "ship/canted") == groups.end());
    ringed.session.advance_to(node.epoch + 5.0, 100000);
    int vacuum = 0;
    for (const auto& e : ringed.session.body(0).vehicle().stages().front().engines) {
        if (e.landing_only) CHECK(!e.producing_thrust());
        if (e.group_id == "ship/vacuum" && e.producing_thrust()) ++vacuum;
    }
    CHECK(vacuum == 3);
}
