#include <catch2/catch_test_macros.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/catch_trigger.hpp"
#include "starsim/contact/tower_collision.hpp"

#include <cmath>

using namespace starsim;
using namespace starsim::contact;
using math::Vec3;

TEST_CASE("The tower closes the arms just in time", "[contact]") {
    const CatchTrigger t;
    CHECK(should_close_arms(t, 30.0, 0.5, 10.0, 4.0));         // 3 s to contact <= 4 + 1 s
    CHECK_FALSE(should_close_arms(t, 60.0, 0.5, 10.0, 4.0));   // 6 s
    CHECK_FALSE(should_close_arms(t, 30.0, 4.0, 10.0, 4.0));   // too far off the catch axis
    CHECK(should_close_arms(t, 1.0, 0.5, 0.0, 4.0));           // hovering just above the arms
    CHECK_FALSE(should_close_arms(t, 20.0, 0.5, -1.0, 4.0));   // climbing
    CHECK_FALSE(should_close_arms(t, -1.0, 0.5, 1.0, 4.0));    // pins already below the arms
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
