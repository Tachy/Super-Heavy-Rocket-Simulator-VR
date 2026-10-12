#include <catch2/catch_test_macros.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/catch_trigger.hpp"
#include "starsim/contact/tower_collision.hpp"

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

TEST_CASE("Hull collisions with the tower and the launch mount", "[contact]") {
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1");
    const TowerGeometry tower(*site.tower, site.latitude_rad, site.longitude_rad, site.height_m);
    const Vec3 up{0.0, 0.0, 1.0};
    const double mount = site.tower->mount_height_m;
    CHECK_FALSE(hits_tower(tower, up * mount, up * (mount + 121.0), 4.5));        // stack on the mount
    CHECK_FALSE(hits_tower(tower, up * 29.5, up * 100.5, 4.5));                   // caught booster
    const Vec3 near_tower = tower.tower_direction() * 20.0;                       // hull reaches 24.5 m: face at 24 m
    CHECK(hits_tower(tower, near_tower + up * 50.0, near_tower + up * 121.0, 4.5));
    CHECK(hits_tower(tower, Vec3{3.0, 0.0, 10.0}, Vec3{3.0, 0.0, 81.0}, 4.5));     // fell into the mount
}

TEST_CASE("Hull penetration by an arm box", "[contact]") {
    const OrientedBox arm{{6.0, 0.0, 0.0}, math::Mat3::identity(), {1.0, 18.0, 1.25}};  // inner face at x = 5
    const Vec3 bottom{0.0, 0.0, -30.0}, top{0.0, 0.0, 40.0};
    CHECK(hull_penetration_m(arm, bottom, top, 4.5) <= 0.0);                                   // 0.5 m clear
    CHECK(std::abs(hull_penetration_m(arm, bottom + Vec3{1.0, 0, 0}, top + Vec3{1.0, 0, 0}, 4.5) - 0.5) < 1e-9);
}
