#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/tower_geometry.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::contact;
using Catch::Matchers::WithinAbs;
using math::Vec3;

namespace {

const config::LaunchSiteConfig& starbase() {
    static const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1");
    return site;
}

TowerGeometry starbase_tower() {
    const auto& s = starbase();
    return TowerGeometry(*s.tower, s.latitude_rad, s.longitude_rad, s.height_m);
}

constexpr Vec3 up{0.0, 0.0, 1.0};

}  // namespace

TEST_CASE("Site frame is east-north-up at the pad and round-trips through ITRF", "[contact]") {
    const auto tower = starbase_tower();
    const Vec3 p{12.3, -4.5, 67.8};
    const Vec3 back = tower.to_site(tower.to_itrf(p));
    CHECK(norm(back - p) < 1e-6);
    const auto above = frames::fixed_to_geodetic(tower.to_itrf({0.0, 0.0, 100.0}));
    CHECK_THAT(above.height_m, WithinAbs(starbase().height_m + 100.0, 1e-6));
    CHECK_THAT(above.latitude_rad, WithinAbs(starbase().latitude_rad, 1e-12));
    CHECK_THAT(dot(tower.across(), tower.tower_direction()), WithinAbs(0.0, 1e-15));
}

TEST_CASE("Closed arms hold the pins but not the hull", "[contact]") {
    const auto tower = starbase_tower();
    const double top = tower.arm_top_height_m(0.0);
    for (const int side : {1, -1}) {
        const auto arm = tower.arm_box(side, 0.0, 0.0);
        for (const double along : {-5.0, 0.0, 5.0}) {  // anywhere along the arms
            for (const double r : {4.9, 5.2, 5.45}) {   // pin (hull radius 4.5 + 1 m)
                CHECK(point_in_box(arm, tower.across() * (side * r) + tower.tower_direction() * along + up * (top - 0.05)).inside);
            }
            CHECK_FALSE(point_in_box(arm, tower.across() * (side * 4.5) + tower.tower_direction() * along + up * (top - 0.05)).inside);
        }
    }
}

TEST_CASE("Open arms leave room for the hull and the pins", "[contact]") {
    const auto tower = starbase_tower();
    const double open = tower.config().arm_open_rad;
    const double z = tower.arm_top_height_m(0.0) - 0.5;
    for (int deg = 0; deg < 360; deg += 10) {
        const double a = deg * constants::deg_to_rad;
        for (const double r : {4.5, 5.5}) {
            const Vec3 p{r * std::cos(a), r * std::sin(a), z};
            CHECK_FALSE(point_in_box(tower.arm_box(1, open, 0.0), p).inside);
            CHECK_FALSE(point_in_box(tower.arm_box(-1, open, 0.0), p).inside);
        }
    }
}

TEST_CASE("Tower and mount stay clear of a booster on the catch axis", "[contact]") {
    const auto tower = starbase_tower();
    const auto& c = tower.config();
    CHECK(point_in_box(tower.tower_box(), tower.tower_direction() * c.distance_m + up * 50.0).inside);
    CHECK_FALSE(capsule_overlaps_box({0.0, 0.0, c.mount_height_m}, {0.0, 0.0, 120.0}, 4.5, tower.tower_box()));
    CHECK_FALSE(capsule_overlaps_annulus({0.0, 0.0, c.mount_height_m}, {0.0, 0.0, 120.0}, 4.5, tower.mount()));
    CHECK(c.catch_height_m - 66.5 > c.mount_height_m);  // caught booster's bottom clears the mount (block 3 pins)
    // Carriage offset moves the arm down.
    CHECK_THAT(tower.arm_box(1, 0.0, -1.0).center.z - tower.arm_box(1, 0.0, 0.0).center.z, WithinAbs(-1.0, 1e-12));
}
