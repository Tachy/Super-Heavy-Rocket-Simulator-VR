#include "starsim/ephemeris/interpolated_ephemeris.hpp"

#include <cmath>

namespace starsim::ephemeris {

InterpolatedEphemeris::InterpolatedEphemeris(const Ephemeris& source, double node_spacing_s)
    : source_(source), spacing_s_(node_spacing_s) {}

const StateVector& InterpolatedEphemeris::node(int body, Body target, std::int64_t index) const {
    auto& nodes = nodes_[body];
    auto it = nodes.find(index);
    if (it == nodes.end()) {
        const auto epoch = time::Epoch::from_tdb_seconds_past_j2000(static_cast<double>(index) * spacing_s_);
        it = nodes.emplace(index, source_.state_gcrf(target, epoch)).first;
    }
    return it->second;
}

StateVector InterpolatedEphemeris::state_gcrf(Body target, time::Epoch epoch) const {
    if (target == Body::Earth) return {};
    if (!(spacing_s_ > 0.0)) return source_.state_gcrf(target, epoch);
    const int body = static_cast<int>(target);
    const double t = epoch.tdb_seconds_past_j2000();
    const auto index = static_cast<std::int64_t>(std::floor(t / spacing_s_));
    Interval& interval = last_[body];
    if (!interval.valid || interval.index != index) {
        interval.start = node(body, target, index);
        interval.end = node(body, target, index + 1);
        interval.index = index;
        interval.valid = true;
    }
    // cubic Hermite on [t0, t0 + h] in s = (t - t0) / h
    const double h = spacing_s_;
    const double s = (t - static_cast<double>(index) * h) / h;
    const double s2 = s * s, s3 = s2 * s;
    const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0, h10 = s3 - 2.0 * s2 + s, h01 = -2.0 * s3 + 3.0 * s2,
                 h11 = s3 - s2;
    const double d00 = 6.0 * s2 - 6.0 * s, d10 = 3.0 * s2 - 4.0 * s + 1.0, d01 = -d00, d11 = 3.0 * s2 - 2.0 * s;
    const auto& a = interval.start;
    const auto& b = interval.end;
    return {a.position_m * h00 + a.velocity_mps * (h10 * h) + b.position_m * h01 + b.velocity_mps * (h11 * h),
            (a.position_m * d00 + b.position_m * d01) / h + a.velocity_mps * d10 + b.velocity_mps * d11};
}

}  // namespace starsim::ephemeris
