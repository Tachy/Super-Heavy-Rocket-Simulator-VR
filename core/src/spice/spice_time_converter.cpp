#include <cstdio>

#include "spice_check.hpp"
#include "starsim/spice/spice_adapters.hpp"

namespace starsim::spice {

time::Epoch SpiceTimeConverter::from_utc(const time::UtcDateTime& utc) const {
    const auto lock = detail::lock_spice();
    // Nanosecond formatting keeps the conversion well below the double resolution of the epoch.
    char text[48];
    std::snprintf(text, sizeof(text), "%04d-%02d-%02dT%02d:%02d:%012.9f", utc.year, utc.month, utc.day,
                  utc.hour, utc.minute, utc.second);
    SpiceDouble et = 0.0;
    str2et_c(text, &et);
    detail::throw_if_failed("str2et_c");
    return time::Epoch::from_tdb_seconds_past_j2000(et);
}

time::UtcDateTime SpiceTimeConverter::to_utc(time::Epoch epoch) const {
    const auto lock = detail::lock_spice();
    SpiceChar text[48];
    et2utc_c(epoch.tdb_seconds_past_j2000(), "ISOC", 6, sizeof(text), text);
    detail::throw_if_failed("et2utc_c");
    const auto parsed = time::UtcDateTime::parse_iso(text);
    if (!parsed) {
        throw SpiceError(std::string("cannot parse SPICE UTC string: ") + text);
    }
    return *parsed;
}

double SpiceTimeConverter::tdb_minus_utc_s(time::Epoch epoch) const {
    const auto lock = detail::lock_spice();
    SpiceDouble delta = 0.0;
    deltet_c(epoch.tdb_seconds_past_j2000(), "ET", &delta);
    detail::throw_if_failed("deltet_c");
    return delta;
}

}  // namespace starsim::spice
