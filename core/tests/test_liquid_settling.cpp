#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/math/constants.hpp"
#include "starsim/vehicle/liquid_settling.hpp"

using namespace starsim;
using Catch::Approx;
using vehicle::settle_direct;
using vehicle::settling_table;

namespace {
constexpr double deg = constants::pi / 180.0;
}

TEST_CASE("Settled liquid along the axis is the aft column", "[settling]") {
    const double aspect = 3.6;  // the LOX main tank: 16.4 m long, radius 4.5 m
    const auto table = settling_table(aspect);
    for (double fill : {0.02, 0.3, 0.9}) {
        const double h = fill * aspect;
        for (const auto& m : {table->query(fill, 0.0), settle_direct(aspect, fill, 0.0)}) {
            CHECK(m.axial == Approx(-0.5 * aspect + 0.5 * h).margin(1e-9));
            CHECK(m.axial_var == Approx(h * h / 12.0).margin(1e-9));
            CHECK(m.lateral == Approx(0.0).margin(1e-9));
            CHECK(m.lateral_var == Approx(0.25).margin(1e-4));  // disk: r^2 / 4
            CHECK(m.transverse_var == Approx(0.25).margin(1e-4));
        }
    }
    // Down along +x (tilt 180 deg): the same column against the forward end.
    const auto forward = table->query(0.3, constants::pi);
    CHECK(forward.axial == Approx(0.5 * aspect - 0.15 * aspect).margin(1e-9));
}

TEST_CASE("A horizontal half-full cylinder has the half-disk centroid", "[settling]") {
    const double aspect = 3.6;
    for (const auto& m : {settling_table(aspect)->query(0.5, 90.0 * deg), settle_direct(aspect, 0.5, 90.0 * deg)}) {
        CHECK(m.axial == Approx(0.0).margin(1e-9));
        CHECK(m.lateral == Approx(4.0 / (3.0 * constants::pi)).margin(1e-4));
        CHECK(m.lateral_var == Approx(0.25 - std::pow(4.0 / (3.0 * constants::pi), 2)).margin(1e-4));
        CHECK(m.transverse_var == Approx(0.25).margin(1e-4));
        CHECK(m.axial_var == Approx(aspect * aspect / 12.0).epsilon(1e-4));
    }
}

TEST_CASE("Settled liquid is mirror-symmetric and a full tank has the cylinder's moments", "[settling]") {
    const double aspect = 1.8;
    const auto table = settling_table(aspect);
    for (double tilt : {20.0 * deg, 60.0 * deg, 85.0 * deg}) {
        const auto a = table->query(0.1, tilt), b = table->query(0.1, constants::pi - tilt);
        CHECK(a.axial == Approx(-b.axial).margin(1e-9));
        CHECK(a.lateral == Approx(b.lateral).margin(1e-9));
        const auto full = table->query(1.0, tilt);
        CHECK(full.axial == Approx(0.0).margin(1e-9));
        CHECK(full.lateral == Approx(0.0).margin(1e-6));
        CHECK(full.axial_var == Approx(aspect * aspect / 12.0).epsilon(1e-3));
    }
}

// The table (1 deg rows, 121 plane offsets, 100-point quadrature) against the direct solution between grid points.
// Tolerance: centroid within 2 % of the radius, variances within 5 % or 0.01 r^2 (shallow residues).
TEST_CASE("The settling table matches the direct solution between grid points", "[settling]") {
    for (double aspect : {3.6, 0.6}) {  // a main tank, a header
        const auto table = settling_table(aspect);
        for (double tilt : {3.3 * deg, 44.6 * deg, 71.2 * deg, 177.5 * deg}) {
            for (double fill : {0.037, 0.21, 0.66}) {
                const auto t = table->query(fill, tilt);
                const auto d = settle_direct(aspect, fill, tilt);
                INFO("aspect " << aspect << " tilt " << tilt / deg << " fill " << fill);
                CHECK(t.axial == Approx(d.axial).margin(0.02));
                CHECK(t.lateral == Approx(d.lateral).margin(0.02));
                CHECK(t.axial_var == Approx(d.axial_var).epsilon(0.05).margin(0.01));
                CHECK(t.lateral_var == Approx(d.lateral_var).epsilon(0.05).margin(0.01));
                CHECK(t.transverse_var == Approx(d.transverse_var).epsilon(0.05).margin(0.01));
            }
        }
    }
}

TEST_CASE("A slightly tilted surface pulls a shallow residue towards the low end", "[settling]") {
    // 2 % LOX in the 16.4 m main tank, down 3 deg off +x (tilt 177 deg; entry: the axial deceleration drives the
    // liquid forward): the residue lies in the forward part, towards the lateral side.
    const auto m = settling_table(3.6)->query(0.02, 177.0 * deg);
    CHECK(m.axial > 0.5 * 3.6 - 1.0);  // within one radius of the forward end
    CHECK(m.lateral > 0.0);
    CHECK(settling_table(3.6) == settling_table(3.6));  // cached
}
