#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "starsim/contact/primitives.hpp"

using namespace starsim;
using namespace starsim::contact;
using Catch::Matchers::WithinAbs;
using math::Vec3;

namespace {

OrientedBox axis_aligned() { return {{10.0, 0.0, 0.0}, math::Mat3::identity(), {2.0, 1.0, 0.5}}; }

}  // namespace

TEST_CASE("Point in box: depth and normal of the nearest face", "[contact]") {
    const auto top = point_in_box(axis_aligned(), {10.5, 0.2, 0.45});
    REQUIRE(top.inside);
    CHECK_THAT(top.depth_m, WithinAbs(0.05, 1e-12));
    CHECK(top.normal == Vec3{0.0, 0.0, 1.0});
    const auto side = point_in_box(axis_aligned(), {11.9, 0.0, 0.0});
    REQUIRE(side.inside);
    CHECK_THAT(side.depth_m, WithinAbs(0.1, 1e-12));
    CHECK(side.normal == Vec3{1.0, 0.0, 0.0});
    CHECK_FALSE(point_in_box(axis_aligned(), {13.0, 0.0, 0.0}).inside);
    CHECK_THAT(distance_to_box(axis_aligned(), {13.0, 0.0, 0.0}), WithinAbs(1.0, 1e-12));
    CHECK(distance_to_box(axis_aligned(), {10.0, 0.0, 0.0}) == 0.0);
}

TEST_CASE("Point in a rotated box reports the face normal in the frame", "[contact]") {
    // Local x along frame y.
    const OrientedBox box{{0.0, 0.0, 0.0}, from_columns({0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}), {2.0, 1.0, 0.5}};
    const auto p = point_in_box(box, {0.0, 1.9, 0.0});
    REQUIRE(p.inside);
    CHECK_THAT(p.depth_m, WithinAbs(0.1, 1e-12));
    CHECK_THAT(p.normal.y, WithinAbs(1.0, 1e-12));
    CHECK(column(box.axes, 1) == Vec3{-1.0, 0.0, 0.0});
}

TEST_CASE("Capsules against a box", "[contact]") {
    CHECK(capsule_overlaps_box({10.0, 0.0, -5.0}, {10.0, 0.0, 5.0}, 0.2, axis_aligned()));
    CHECK_FALSE(capsule_overlaps_box({10.0, 3.0, -5.0}, {10.0, 3.0, 5.0}, 1.0, axis_aligned()));  // 2 m clear
    CHECK(capsule_overlaps_box({10.0, 3.0, -5.0}, {10.0, 3.0, 5.0}, 2.1, axis_aligned()));
}

TEST_CASE("Capsules against the launch mount annulus", "[contact]") {
    const Annulus mount{5.5, 13.0, 20.0};
    CHECK_FALSE(capsule_overlaps_annulus({0.0, 0.0, 20.0}, {0.0, 0.0, 90.0}, 4.5, mount));  // stands on it
    CHECK(capsule_overlaps_annulus({3.0, 0.0, 10.0}, {3.0, 0.0, 80.0}, 4.5, mount));        // fell into the ring
    CHECK_FALSE(capsule_overlaps_annulus({8.0, 0.0, 25.0}, {8.0, 0.0, 90.0}, 4.5, mount));  // 5 m above it
    CHECK_THAT(distance_to_annulus(mount, {0.0, 0.0, 0.0}), WithinAbs(5.5, 1e-12));
    CHECK_THAT(distance_to_annulus(mount, {20.0, 0.0, 30.0}), WithinAbs(std::hypot(7.0, 10.0), 1e-12));
}
