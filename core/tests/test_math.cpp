#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

using starsim::math::Mat3;
using starsim::math::Vec3;

TEST_CASE("Vec3 arithmetic, dot and cross product", "[math]") {
    const Vec3 a{1.0, 2.0, 3.0};
    const Vec3 b{-2.0, 0.5, 4.0};
    CHECK(a + b == Vec3{-1.0, 2.5, 7.0});
    CHECK(a - b == Vec3{3.0, 1.5, -1.0});
    CHECK(2.0 * a == Vec3{2.0, 4.0, 6.0});
    CHECK(dot(a, b) == Catch::Approx(11.0));
    CHECK(cross(Vec3{1, 0, 0}, Vec3{0, 1, 0}) == Vec3{0, 0, 1});
    CHECK(norm(Vec3{3.0, 4.0, 12.0}) == Catch::Approx(13.0));
}

TEST_CASE("Vec3 keeps sub-millimetre resolution at lunar distance", "[math]") {
    // float64 requirement: a 0.1 mm offset at 4e8 m must survive a round trip.
    const Vec3 moon{4.0e8, -1.5e8, 3.0e7};
    const Vec3 shifted = moon + Vec3{1e-4, 1e-4, 1e-4};
    const Vec3 delta = shifted - moon;
    CHECK(delta.x == Catch::Approx(1e-4).epsilon(1e-3));
    CHECK(delta.y == Catch::Approx(1e-4).epsilon(1e-3));
}

TEST_CASE("Mat3 multiplication and transpose", "[math]") {
    Mat3 rot_z;  // +90 deg about z
    rot_z(0, 1) = -1.0;
    rot_z(1, 0) = 1.0;
    rot_z(2, 2) = 1.0;
    CHECK(rot_z * Vec3{1, 0, 0} == Vec3{0, 1, 0});
    const Mat3 back = transpose(rot_z) * rot_z;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK(back(i, j) == Mat3::identity()(i, j));
        }
    }
}

TEST_CASE("Mat3 determinant and linear solve", "[math]") {
    // Rows [2 1 0; 0 3 1; 1 0 4]: det 25.
    const Mat3 a = Mat3::from_columns({2.0, 0.0, 1.0}, {1.0, 3.0, 0.0}, {0.0, 1.0, 4.0});
    CHECK(a(0, 1) == 1.0);
    CHECK(starsim::math::column(a, 2).z == 4.0);
    CHECK(starsim::math::determinant(a) == Catch::Approx(25.0));
    const auto x = starsim::math::solve(a, {0.0, -3.0, 13.0});
    REQUIRE(x);
    CHECK(norm(*x - Vec3{1.0, -2.0, 3.0}) < 1e-12);
    // Dependent columns: refused.
    CHECK_FALSE(starsim::math::solve(Mat3::from_columns({1.0, 2.0, 3.0}, {0.0, 1.0, 1.0}, {1.0, 3.0, 4.0}),
                                     {1.0, 0.0, 0.0}));
    // Badly scaled but regular columns (a Jacobian in m per m/s next to one in m per m): solved.
    const auto y =
        starsim::math::solve(Mat3::from_columns({1e6, 0.0, 0.0}, {0.0, 1e-3, 0.0}, {0.0, 0.0, 1.0}), {1e6, 1e-3, 1.0});
    REQUIRE(y);
    CHECK(norm(*y - Vec3{1.0, 1.0, 1.0}) < 1e-12);
}
