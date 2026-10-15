// Sun hold of the lunar coast: ends when pointed and settled, or after the timeout.

#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/sun_hold.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;

TEST_CASE("Sun hold settles, or gives up after its timeout", "[gnc][sun]") {
    constexpr double deg = constants::deg_to_rad;
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    gnc::SunHold hold;
    CHECK_FALSE(hold.active());
    hold.start(t0);
    REQUIRE(hold.active());
    CHECK(hold.update(5.0 * deg, 0.0, t0 + 10.0) == gnc::SunHoldResult::Holding);             // not pointed
    CHECK(hold.update(0.5 * deg, 0.1 * deg, t0 + 20.0) == gnc::SunHoldResult::Holding);       // still turning
    CHECK(hold.update(0.5 * deg, 0.001 * deg, t0 + 30.0) == gnc::SunHoldResult::Settled);
    CHECK_FALSE(hold.active());
    hold.start(t0);
    CHECK(hold.update(5.0 * deg, 0.1 * deg, t0 + 1799.0) == gnc::SunHoldResult::Holding);
    CHECK(hold.update(5.0 * deg, 0.1 * deg, t0 + 1800.0) == gnc::SunHoldResult::TimedOut);  // never settles
    CHECK_FALSE(hold.active());
}
