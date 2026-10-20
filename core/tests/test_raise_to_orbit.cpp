// Refuelling f1: apsis burns from the insertion orbit to a phasing orbit.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"

using namespace starsim;

namespace {
constexpr double gm = constants::earth_gm_m3ps2;
const double re = constants::wgs84_semi_major_axis_m;
const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);

math::StateVector at_periapsis(double rp, double ra) {
    orbit::KeplerianElements e;
    e.semi_major_axis_m = 0.5 * (rp + ra);
    e.eccentricity = (ra - rp) / (ra + rp);
    e.inclination_rad = 26.5 * constants::deg_to_rad;
    return orbit::state_from_elements(e, gm);
}

// Flies the node impulsively and returns the state just after it.
math::StateVector fly(const math::StateVector& s, time::Epoch now, const gnc::BurnNode& node) {
    auto at = orbit::propagate_two_body(s, gm, node.epoch - now);
    at.velocity_mps = at.velocity_mps + node.delta_v_gcrf_mps;
    return at;
}

double periapsis(const math::StateVector& s) {
    const auto el = orbit::elements_from_state(s, gm);
    return el.semi_major_axis_m * (1.0 - el.eccentricity);
}
double apoapsis(const math::StateVector& s) {
    const auto el = orbit::elements_from_state(s, gm);
    return el.semi_major_axis_m * (1.0 + el.eccentricity);
}
}  // namespace

TEST_CASE("next_raise_node: below the depot one burn at apoapsis", "[burn_node]") {
    auto s = at_periapsis(re + 120e3, re + 190e3);
    const auto node = gnc::next_raise_node(s, t0, gm, re + 160e3, re + 190e3, 1e3);
    REQUIRE(node);
    s = fly(s, t0, *node);
    CHECK(periapsis(s) == Catch::Approx(re + 160e3).margin(100.0));
    CHECK(apoapsis(s) == Catch::Approx(re + 190e3).margin(100.0));
    CHECK_FALSE(gnc::next_raise_node(s, node->epoch, gm, re + 160e3, re + 190e3, 1e3));  // done
}

TEST_CASE("next_raise_node: above the depot two burns", "[burn_node]") {
    auto s = at_periapsis(re + 120e3, re + 190e3);
    time::Epoch now = t0;
    int burns = 0;
    while (const auto node = gnc::next_raise_node(s, now, gm, re + 250e3, re + 270e3, 1e3)) {
        s = fly(s, now, *node);
        now = node->epoch;
        REQUIRE(++burns <= 2);
    }
    CHECK(burns == 2);
    CHECK(periapsis(s) == Catch::Approx(re + 250e3).margin(1e3));
    CHECK(apoapsis(s) == Catch::Approx(re + 270e3).margin(1e3));
}

TEST_CASE("next_raise_node: nothing to do at or above the target; open orbit refused", "[burn_node]") {
    CHECK_FALSE(gnc::next_raise_node(at_periapsis(re + 200e3, re + 210e3), t0, gm, re + 170e3, re + 190e3, 1e3));
    orbit::KeplerianElements h;
    h.semi_major_axis_m = -2.0e7;
    h.eccentricity = 1.5;
    CHECK_FALSE(gnc::next_raise_node(orbit::state_from_elements(h, gm), t0, gm, re + 170e3, re + 190e3, 1e3));
}
