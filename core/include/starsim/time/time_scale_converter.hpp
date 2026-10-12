#pragma once

#include "starsim/time/epoch.hpp"
#include "starsim/time/utc_date_time.hpp"

namespace starsim::time {

// Converts between UTC (with leap seconds) and the simulation time scale TDB.
class TimeScaleConverter {
public:
    virtual ~TimeScaleConverter() = default;

    virtual Epoch from_utc(const UtcDateTime& utc) const = 0;
    virtual UtcDateTime to_utc(Epoch epoch) const = 0;

    // TDB - UTC in seconds at the given epoch (leap seconds + 32.184 s + periodic TDB-TT terms).
    virtual double tdb_minus_utc_s(Epoch epoch) const = 0;
};

}  // namespace starsim::time
