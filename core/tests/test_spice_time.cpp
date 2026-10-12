#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using time::UtcDateTime;

TEST_CASE("J2000.0 epoch: 2000-01-01 11:58:55.816 UTC is TDB 0", "[time][spice]") {
    test::spice_session();
    const spice::SpiceTimeConverter converter;
    // TT = UTC + 32.184 s + 32 leap seconds; |TDB - TT| < 2 ms.
    const auto epoch = converter.from_utc({2000, 1, 1, 11, 58, 55.816});
    CHECK(epoch.tdb_seconds_past_j2000() == Catch::Approx(0.0).margin(2e-3));
}

TEST_CASE("TDB - UTC reflects the leap-second table", "[time][spice]") {
    test::spice_session();
    const spice::SpiceTimeConverter converter;
    CHECK(converter.tdb_minus_utc_s(converter.from_utc({2000, 1, 1, 0, 0, 0.0})) ==
          Catch::Approx(32.0 + 32.184).margin(2e-3));
    CHECK(converter.tdb_minus_utc_s(converter.from_utc({2017, 1, 1, 0, 0, 0.0})) ==
          Catch::Approx(37.0 + 32.184).margin(2e-3));
}

TEST_CASE("The leap second 2016-12-31 23:59:60 lasts one second", "[time][spice]") {
    test::spice_session();
    const spice::SpiceTimeConverter converter;
    const auto before = converter.from_utc({2016, 12, 31, 23, 59, 59.0});
    const auto leap = converter.from_utc({2016, 12, 31, 23, 59, 60.0});
    const auto after = converter.from_utc({2017, 1, 1, 0, 0, 0.0});
    CHECK(leap - before == Catch::Approx(1.0).margin(1e-6));
    CHECK(after - before == Catch::Approx(2.0).margin(1e-6));
}

TEST_CASE("UTC -> TDB -> UTC round trip to the microsecond", "[time][spice]") {
    test::spice_session();
    const spice::SpiceTimeConverter converter;
    const UtcDateTime utc{2026, 10, 12, 14, 3, 7.123456};
    const auto back = converter.to_utc(converter.from_utc(utc));
    CHECK(back.year == utc.year);
    CHECK(back.month == utc.month);
    CHECK(back.day == utc.day);
    CHECK(back.hour == utc.hour);
    CHECK(back.minute == utc.minute);
    CHECK(back.second == Catch::Approx(utc.second).margin(1.5e-6));
}

TEST_CASE("SPICE errors become exceptions and do not poison later calls", "[spice]") {
    test::spice_session();
    const spice::SpiceTimeConverter converter;
    CHECK_THROWS_AS(converter.from_utc({2026, 2, 30, 0, 0, 0.0}), spice::SpiceError);
    CHECK_NOTHROW(converter.from_utc({2026, 2, 28, 0, 0, 0.0}));
}
