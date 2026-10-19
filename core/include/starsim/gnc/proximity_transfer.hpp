#pragma once

#include "starsim/gnc/proximity_geometry.hpp"
#include "starsim/math/vec3.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"

namespace starsim::gnc {

// Proximity transfers outside the keep-out sphere and the breakout burn (ISS i4), both in the Clohessy-Wiltshire
// model about the station CG (LVLH of frames/lvlh.hpp: x V-bar, y H-bar, z R-bar = nadir). Over the few hundred
// metres and ~20 min of a proximity leg CW stays within decimetres of the full model (validated in the tests);
// the translation loop re-solves the transfer every tick, so the residual never accumulates.

struct ProximityTransfer {
    bool feasible = false;
    double duration_s = 0.0;
    math::Vec3 departure_velocity_lvlh_mps;  // relative velocity needed now (CW)
    double min_cg_range_m = 0.0;             // closest approach to the station CG on the CW arc
};
// CW transfer of the chaser CG from `from` (current relative state, LVLH about the station CG) to `to_cg_m` (at rest
// there). Tries nominal_s, then 0.75, 1.25, 0.5, 1.5 x nominal_s; the first whose sampled arc (every 5 s) keeps
// >= keep_out_radius_m + kos_margin_m from the CG wins. Infeasible (all inside, or CW singular) otherwise.
ProximityTransfer plan_proximity_transfer(const orbit::RelativeState& from, const math::Vec3& to_cg_m,
                                          double n_radps, double nominal_s, const ProximityParameters& p);

struct Breakout {
    bool safe = false;
    math::Vec3 delta_v_lvlh_mps;
    double exit_s = 0.0;               // when the CW arc leaves the sphere (+ margin)
    double min_range_after_exit_m = 0.0;
};
// Smallest magnitude of p.breakout.magnitudes_mps with a direction (26 cube directions + the radial-away direction)
// whose CW arc leaves the sphere within exit_limit_s without first closing in (range never below 0.9 x the start
// range before the exit) and stays outside keep_out + margin until horizon_s; among those the largest minimum range
// after the exit. Unsafe (best effort: the largest magnitude, away direction) when none qualifies.
Breakout breakout_delta_v(const orbit::RelativeState& from, double n_radps, const ProximityParameters& p);

}  // namespace starsim::gnc
