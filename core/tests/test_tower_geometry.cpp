#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/pin_contact.hpp"
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

TEST_CASE("Tower box is parallel to the measured squares", "[contact]") {
    const auto tower = starbase_tower();
    const auto& c = tower.config();
    const double yaw = c.plan_yaw_rad;
    CHECK_THAT(yaw, WithinAbs(37.4 * constants::deg_to_rad, 1e-12));
    const auto box = tower.tower_box();
    const Vec3 x_expected = tower.tower_direction() * std::cos(yaw) - tower.across() * std::sin(yaw);
    const Vec3 y_expected = tower.tower_direction() * std::sin(yaw) + tower.across() * std::cos(yaw);
    CHECK(norm(math::column(box.axes, 0) - x_expected) < 1e-12);
    CHECK(norm(math::column(box.axes, 1) - y_expected) < 1e-12);
    CHECK(norm(box.center - (tower.tower_direction() * c.distance_m + up * (0.5 * c.height_m))) < 1e-9);
}

TEST_CASE("Hinges straddle the corner toward the mount", "[contact]") {
    const auto tower = starbase_tower();
    const auto& c = tower.config();
    CHECK_THAT(c.hinge_distance_m, WithinAbs(22.3, 1e-12));
    const double lateral = 0.5 * c.arm_closed_gap_m + 0.5 * c.arm_width_m;
    for (const int side : {1, -1}) {
        const Vec3 h = tower.hinge(side);
        CHECK_THAT(dot(h, tower.tower_direction()), WithinAbs(22.3, 1e-9));
        CHECK_THAT(dot(h, tower.across()), WithinAbs(side * lateral, 1e-9));
        CHECK_THAT(h.z, WithinAbs(c.catch_height_m, 1e-9));
    }
    // The box corner nearest the catch axis lies within 2 m of the line at 22.3 +- 0.1 m.
    const auto box = tower.tower_box();
    double best = 1e9;
    Vec3 corner{};
    for (const double sx : {-1.0, 1.0}) {
        for (const double sy : {-1.0, 1.0}) {
            const Vec3 p = box.center + math::column(box.axes, 0) * (sx * box.half_extents.x) +
                           math::column(box.axes, 1) * (sy * box.half_extents.y);
            const double d = std::hypot(p.x, p.y);
            if (d < best) {
                best = d;
                corner = p;
            }
        }
    }
    CHECK_THAT(dot(corner, tower.tower_direction()), WithinAbs(22.3, 0.1));
    CHECK(std::abs(dot(corner, tower.across())) < 2.0);
}

TEST_CASE("Closed arms hold the pins but not the hull", "[contact]") {
    const auto tower = starbase_tower();
    const double top = tower.arm_top_height_m(0.0);
    for (const int side : {1, -1}) {
        const auto arm = tower.arm_box(side, 0.0, 0.0);
        for (const double along : {-1.5, 0.0, 5.0}) {  // along the arms: the tip lies 1.9 m beyond the axis
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
    const auto block3 = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    CHECK(c.catch_height_m - block3.stages.front().catch_pins.station_m > c.mount_height_m);  // caught bottom clears the mount
    // Carriage offset moves the arm down.
    CHECK_THAT(tower.arm_box(1, 0.0, -1.0).center.z - tower.arm_box(1, 0.0, 0.0).center.z, WithinAbs(-1.0, 1e-12));
}

TEST_CASE("Pins at 0 and 180 rest on both closed arms", "[contact]") {
    const auto tower = starbase_tower();
    const auto vehicle = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto& stage = vehicle.stages.front();
    CHECK(stage.catch_pins.azimuths_rad == std::vector<double>{0.0, constants::pi});
    CHECK_THAT(stage.catch_pins.station_m, WithinAbs(stage.grid_fins.station_m - 1.0, 1e-12));
    const double hull_radius_m = 4.5;
    const auto geometry = make_catch_geometry(stage.catch_pins, 0.0, hull_radius_m, tower.config().arm_thickness_m);
    std::array<ArmContactState, 2> arms;
    for (int i = 0; i < 2; ++i) {
        const int side = i == 0 ? 1 : -1;
        arms[static_cast<size_t>(i)] = {tower.arm_box(side, 0.0, 0.0), tower.hinge(side), 0.0, 0.0, tower.across() * side};
    }
    const double sink_in_m = 0.01;  // pins that far into the arm tops
    SitePose pose;
    pose.cg_station_m = 24.0;
    pose.cg_m = up * (tower.arm_top_height_m(0.0) - sink_in_m - (geometry.pin_station_m - pose.cg_station_m));
    pose.body_to_site = from_columns(up, tower.across(), cross(up, tower.across()));
    const auto c = catch_contact(geometry, pose, arms);
    REQUIRE(c.pin_points_in_contact.size() == 2);
    CHECK(c.all_pins_in_contact());
    CHECK(c.force_n.z > 0.0);
    CHECK(c.hull_push_n[0] == 0.0);  // the 9 m hull between the arms, 0.3 m each side
    CHECK(c.hull_push_n[1] == 0.0);
}
