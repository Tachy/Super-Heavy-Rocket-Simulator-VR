#include "starsim/gnc/proximity_transfer.hpp"

#include <algorithm>
#include <limits>
#include <vector>

namespace starsim::gnc {

namespace {

constexpr double transfer_sample_s = 5.0;      // [E] CW sampling of the transfer clearance check
constexpr double breakout_closing_factor = 0.9;  // [E] before the exit the range never drops below this x start

double min_range_on_arc(const orbit::RelativeState& s, double n, double duration_s, double step_s) {
    double closest = norm(s.position_m);
    for (double t = step_s; t <= duration_s + 0.5 * step_s; t += step_s) {
        closest = std::min(closest, norm(orbit::cw_propagate(s, n, std::min(t, duration_s)).position_m));
    }
    return closest;
}

std::vector<math::Vec3> breakout_directions(const math::Vec3& position_m) {
    std::vector<math::Vec3> directions;
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            for (int k = -1; k <= 1; ++k) {
                if (i == 0 && j == 0 && k == 0) continue;
                directions.push_back(normalized(math::Vec3{double(i), double(j), double(k)}));
            }
        }
    }
    directions.push_back(normalized(position_m));
    return directions;
}

// Exit time and minimum range after the exit of one candidate arc; nullopt-like (exit < 0) when rejected.
struct Candidate {
    bool ok = false;
    double exit_s = 0.0;
    double min_after_exit_m = 0.0;
    double ellipsoid_exit_s = -1.0;  // < 0: not left (or not asked for)
};

// (x/a)^2 + (y/b)^2 + (z/c)^2: > 1 outside the ellipsoid.
double ellipsoid_metric(const math::Vec3& r, const math::Vec3& axes) {
    const double x = r.x / axes.x, y = r.y / axes.y, z = r.z / axes.z;
    return x * x + y * y + z * z;
}

Candidate check_breakout_arc(const orbit::RelativeState& s, double n, const ProximityParameters& p) {
    const double clearance = p.keep_out_radius_m + p.kos_margin_m;
    const double start_range = norm(s.position_m);
    const auto& b = p.breakout;
    bool exited = start_range >= clearance;
    Candidate c;
    c.min_after_exit_m = exited ? start_range : std::numeric_limits<double>::infinity();
    for (double t = b.sample_s; t <= b.horizon_s + 0.5 * b.sample_s; t += b.sample_s) {
        const math::Vec3 at = orbit::cw_propagate(s, n, t).position_m;
        const double range = norm(at);
        if (b.ellipsoid_semi_axes_m && c.ellipsoid_exit_s < 0.0 && ellipsoid_metric(at, *b.ellipsoid_semi_axes_m) > 1.0) {
            c.ellipsoid_exit_s = t;
        }
        if (!exited) {
            if (range < breakout_closing_factor * start_range) return {};
            if (range >= clearance) {
                exited = true;
                c.exit_s = t;
                c.min_after_exit_m = range;
            } else if (t > b.exit_limit_s) {
                return {};
            }
            continue;
        }
        if (range < clearance) return {};
        c.min_after_exit_m = std::min(c.min_after_exit_m, range);
    }
    c.ok = exited;
    if (b.ellipsoid_semi_axes_m && (c.ellipsoid_exit_s < 0.0 || c.ellipsoid_exit_s > b.ellipsoid_exit_limit_s)) c.ok = false;
    return c;
}

}  // namespace

ProximityTransfer plan_proximity_transfer(const orbit::RelativeState& from, const math::Vec3& to_cg_m, double n_radps,
                                          double nominal_s, const ProximityParameters& p) {
    // a start inside the margin (but outside the sphere) only needs an arc that does not come closer than it starts
    const double clearance =
        std::max(p.keep_out_radius_m, std::min(p.keep_out_radius_m + p.kos_margin_m, norm(from.position_m)));
    for (const double f : {1.0, 0.75, 1.25, 0.5, 1.5}) {
        const double duration = f * nominal_s;
        const auto v = orbit::cw_transfer_velocity(from.position_m, to_cg_m, n_radps, duration);
        if (!v) continue;
        const double closest = min_range_on_arc({from.position_m, *v}, n_radps, duration, transfer_sample_s);
        if (closest >= clearance) return {true, duration, *v, closest};
    }
    return {};
}

Breakout breakout_delta_v(const orbit::RelativeState& from, double n_radps, const ProximityParameters& p) {
    const auto directions = breakout_directions(from.position_m);
    for (const double magnitude : p.breakout.magnitudes_mps) {
        Breakout best;
        for (const auto& d : directions) {
            const auto c = check_breakout_arc({from.position_m, from.velocity_mps + magnitude * d}, n_radps, p);
            if (c.ok && (!best.safe || c.min_after_exit_m > best.min_range_after_exit_m)) {
                best = {true, magnitude * d, c.exit_s, c.min_after_exit_m, std::max(c.ellipsoid_exit_s, 0.0)};
            }
        }
        if (best.safe) return best;
    }
    Breakout fallback;
    fallback.delta_v_lvlh_mps = p.breakout.magnitudes_mps.back() * directions.back();
    return fallback;
}

}  // namespace starsim::gnc
