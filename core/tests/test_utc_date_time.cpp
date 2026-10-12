#include <catch2/catch_test_macros.hpp>

#include "starsim/time/utc_date_time.hpp"

using starsim::time::UtcDateTime;

TEST_CASE("UtcDateTime parses and formats ISO 8601", "[time]") {
    const auto parsed = UtcDateTime::parse_iso("2026-10-12T14:03:07.25Z");
    REQUIRE(parsed);
    CHECK(*parsed == UtcDateTime{2026, 10, 12, 14, 3, 7.25});
    CHECK(parsed->to_iso_string() == "2026-10-12T14:03:07.250000");
}

TEST_CASE("UtcDateTime accepts a leap second", "[time]") {
    const auto parsed = UtcDateTime::parse_iso("2016-12-31T23:59:60.5");
    REQUIRE(parsed);
    CHECK(parsed->second == 60.5);
}

TEST_CASE("UtcDateTime rejects malformed input", "[time]") {
    CHECK_FALSE(UtcDateTime::parse_iso("2026-13-01T00:00:00"));
    CHECK_FALSE(UtcDateTime::parse_iso("2026-10-12 25:00:00"));
    CHECK_FALSE(UtcDateTime::parse_iso("2026-10-12T10:00"));
    CHECK_FALSE(UtcDateTime::parse_iso("garbage"));
}
