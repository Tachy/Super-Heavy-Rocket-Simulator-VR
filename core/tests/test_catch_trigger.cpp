#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/catch_trigger.hpp"
#include "starsim/contact/tower_collision.hpp"
#include "starsim/contact/tower_geometry.hpp"
#include "starsim/gnc/landing_site.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/catch_tower_sim.hpp"

#include <cmath>

using namespace starsim;
using namespace starsim::contact;
using math::Vec3;

TEST_CASE("The tower waits half open and closes the arms just before the pins arrive", "[contact]") {
    const CatchTrigger t;
    const double final_close = 0.65;  // standby -> closed
    // Arguments: pin height above the arms, axis offset, sink rate, final closing time.
    CHECK(arm_command(t, 150.0, 5.0, 10.0, final_close) == ArmCommand::Open);      // 15 s to contact
    CHECK(arm_command(t, 50.0, 5.0, 10.0, final_close) == ArmCommand::Standby);    // 5 s: wait half open
    CHECK(arm_command(t, 5.0, 0.5, 5.0, final_close) == ArmCommand::Closed);       // 1 s <= 0.65 + 0.5 s
    CHECK(arm_command(t, 5.0, 2.5, 5.0, final_close) == ArmCommand::Standby);      // too far off: no catch
    CHECK(arm_command(t, 10.0, 0.5, 5.0, final_close) == ArmCommand::Standby);     // 2 s: not yet
    // Still braking hard (sink above the slow final approach): "1 s at this speed" would close far too early.
    CHECK(arm_command(t, 140.0, 0.5, 140.0, final_close) == ArmCommand::Standby);
    CHECK(arm_command(t, 1.0, 0.5, 0.0, final_close) == ArmCommand::Closed);       // hovering just above the arms
    CHECK(arm_command(t, 20.0, 0.5, -1.0, final_close) == ArmCommand::Open);       // climbing
    CHECK(arm_command(t, -1.0, 0.5, 1.0, final_close) == ArmCommand::Open);        // pins already below the arms
}

// The final closing is allowed only where the pins still land on the arms: at the limit, displaced away from the tower
// (the only direction that runs off the tips), every pin point lies on a closed arm; just beyond the tip none does.
TEST_CASE("The arms close only where the pins land on the tips", "[contact]") {
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1");
    const sim::CatchTowerSim sim(*site.tower, site.latitude_rad, site.longitude_rad, site.height_m);
    const auto& tower = sim.geometry();
    const double limit_m = sim.trigger().max_axis_offset_m;
    CHECK(limit_m < tower.arm_tip_beyond_axis_m());
    const double final_close = 0.65;
    CHECK(arm_command(sim.trigger(), 3.0, limit_m, 5.0, final_close) == ArmCommand::Closed);
    CHECK(arm_command(sim.trigger(), 3.0, limit_m + 0.01, 5.0, final_close) == ArmCommand::Standby);
    const Vec3 up{0.0, 0.0, 1.0};
    const double z = tower.arm_top_height_m(0.0) - 0.05;
    for (const int side : {1, -1}) {
        const auto arm = tower.arm_box(side, 0.0, 0.0);
        for (const double r : {4.9, 5.2, 5.45}) {  // pin (hull radius 4.5 + 1 m)
            const Vec3 across = tower.across() * (side * r) + up * z;
            CHECK(point_in_box(arm, across - tower.tower_direction() * limit_m).inside);
            CHECK_FALSE(point_in_box(arm, across - tower.tower_direction() * (tower.arm_tip_beyond_axis_m() + 0.01)).inside);
        }
    }
}

TEST_CASE("Hull collisions with the tower and the launch mount", "[contact]") {
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1");
    const TowerGeometry tower(*site.tower, site.latitude_rad, site.longitude_rad, site.height_m);
    const Vec3 up{0.0, 0.0, 1.0};
    const double mount = site.tower->mount_height_m;
    CHECK_FALSE(hits_tower(tower, up * mount, up * (mount + 121.0), 4.5));        // stack on the mount
    CHECK_FALSE(hits_tower(tower, up * 29.5, up * 100.5, 4.5));                   // caught booster
    // Hull reaches 27.5 m: the box corner toward the mount (22.4 m along, 1.2 m across, parallel to the squares) is inside.
    const Vec3 near_tower = tower.tower_direction() * 23.0;
    CHECK(hits_tower(tower, near_tower + up * 50.0, near_tower + up * 121.0, 4.5));
    CHECK(hits_tower(tower, Vec3{3.0, 0.0, 10.0}, Vec3{3.0, 0.0, 81.0}, 4.5));     // fell into the mount
}

TEST_CASE("Hull penetration by an arm box", "[contact]") {
    const OrientedBox arm{{6.0, 0.0, 0.0}, math::Mat3::identity(), {1.0, 18.0, 1.25}};  // inner face at x = 5
    const Vec3 bottom{0.0, 0.0, -30.0}, top{0.0, 0.0, 40.0};
    CHECK(hull_penetration_m(arm, bottom, top, 4.5) <= 0.0);                                   // 0.5 m clear
    CHECK(std::abs(hull_penetration_m(arm, bottom + Vec3{1.0, 0, 0}, top + Vec3{1.0, 0, 0}, 4.5) - 0.5) < 1e-9);
}

// The hull on the catch axis never touches the rotated tower box or an arm, at any opening and carriage offset the arms
// pass through while it comes down (plan Review Focus 2; a 0.25 m axis offset stays inside the 0.3 m per side).
TEST_CASE("Arm sweep clears the rotated tower corner", "[contact]") {
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1");
    const TowerGeometry tower(*site.tower, site.latitude_rad, site.longitude_rad, site.height_m);
    const auto& c = *site.tower;
    const double hull_radius_m = 4.5;
    for (const double offset_m : {0.0, 0.25, -0.25}) {
        const Vec3 axis = tower.across() * offset_m;
        const Vec3 bottom = axis + Vec3{0.0, 0.0, c.catch_height_m - 63.0};
        const Vec3 top = axis + Vec3{0.0, 0.0, c.catch_height_m + 9.3};
        CHECK_FALSE(hits_tower(tower, bottom, top, hull_radius_m));
        for (double opening_deg = 0.0; opening_deg <= c.arm_open_rad / constants::deg_to_rad + 1e-9; opening_deg += 1.0) {
            for (const double carriage_m : {0.0, -0.5 * c.carriage_stroke_m, -c.carriage_stroke_m}) {
                for (const int side : {1, -1}) {
                    const auto arm = tower.arm_box(side, opening_deg * constants::deg_to_rad, carriage_m);
                    CHECK(hull_penetration_m(arm, bottom, top, hull_radius_m) <= 0.0);  // negative: the clearance
                }
            }
        }
    }
}

// The roll of the booster's catch is one-way: fins 0, 90, 180, the 90 deg fin away from the tower, the pins across the
// arms; a ship (no such azimuth) keeps the nearer of the two rolls.
TEST_CASE("Catch ends with the 90 deg fin away from the tower", "[contact]") {
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1");
    const TowerGeometry tower(*site.tower, site.latitude_rad, site.longitude_rad, site.height_m);
    const auto vehicle = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto& pins = vehicle.stages.front().catch_pins;
    REQUIRE(pins.away_from_tower_azimuth_rad.has_value());
    const Vec3 up{0.0, 0.0, 1.0};
    gnc::CatchTarget target;
    target.across_itrf = tower.across();
    target.pin_azimuth_rad = pins.azimuths_rad.front();
    target.away_from_tower_azimuth_rad = pins.away_from_tower_azimuth_rad;
    for (const Vec3& current : {Vec3{1.0, 0.0, 0.0}, Vec3{-1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, -1.0, 0.0}}) {
        const Vec3 y = gnc::catch_roll_reference(target, up, current);
        const Vec3 fin_90 = cross(up, y);  // body +z, azimuth 90
        CHECK(dot(fin_90, tower.tower_direction()) == Catch::Approx(-1.0).margin(1e-12));
        const Vec3 first_pin = y * std::cos(target.pin_azimuth_rad) + cross(up, y) * std::sin(target.pin_azimuth_rad);
        CHECK(std::abs(dot(first_pin, tower.across())) == Catch::Approx(1.0).margin(1e-12));
    }
    const auto& ship_pins = vehicle.stages.back().catch_pins;
    CHECK_FALSE(ship_pins.away_from_tower_azimuth_rad.has_value());
    target.away_from_tower_azimuth_rad = ship_pins.away_from_tower_azimuth_rad;
    CHECK(norm(gnc::catch_roll_reference(target, up, tower.across()) - tower.across()) < 1e-12);
    CHECK(norm(gnc::catch_roll_reference(target, up, -tower.across()) + tower.across()) < 1e-12);
}
