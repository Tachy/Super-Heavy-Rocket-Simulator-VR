#pragma once

#include <array>
#include <cstdint>
#include <unordered_map>

#include "starsim/ephemeris/ephemeris.hpp"

namespace starsim::ephemeris {

// An ephemeris that samples another one on a fixed grid of epochs and interpolates between the nodes: cubic Hermite in
// position and velocity on each interval (C1 across nodes). l6 Task 1: the flight and its planners asked SPICE for the
// Moon and the Sun at every force evaluation - 75 % of a lunar return's plan time (~1.2 us per call).
// The geocentric Moon and Sun are smooth on the grid: the interpolation error scales with h^4 (Moon: 14 mm at 1 h,
// 0.2 mm at 20 min; test_interpolated_ephemeris checks 1 mm over a month). Apparent positions and GM pass through.
// Nodes are filled on demand and kept: not thread-safe (like the CSPICE source).
class InterpolatedEphemeris final : public Ephemeris {
public:
    // [E] node spacing: 20 min (the test measured the Moon at 14 mm with 1 h, 0.2 mm with 20 min: h^4).
    static constexpr double default_node_spacing_s = 1200.0;

    // `node_spacing_s` <= 0: every call passes through to `source` (the fidelity flag's off state).
    explicit InterpolatedEphemeris(const Ephemeris& source, double node_spacing_s = default_node_spacing_s);

    StateVector state_gcrf(Body target, time::Epoch epoch) const override;
    double gm_m3ps2(Body body) const override { return source_.gm_m3ps2(body); }
    math::Vec3 apparent_position_gcrf(Body target, time::Epoch epoch,
                                      const math::StateVector& observer_gcrf) const override {
        return source_.apparent_position_gcrf(target, epoch, observer_gcrf);
    }

private:
    static constexpr int body_count = 4;
    // the interval a query fell into last time, per body (consecutive queries mostly share it)
    struct Interval {
        std::int64_t index = 0;
        bool valid = false;
        StateVector start, end;
    };
    const StateVector& node(int body, Body target, std::int64_t index) const;

    const Ephemeris& source_;
    double spacing_s_;
    mutable std::array<std::unordered_map<std::int64_t, StateVector>, body_count> nodes_;
    mutable std::array<Interval, body_count> last_;
};

}  // namespace starsim::ephemeris
