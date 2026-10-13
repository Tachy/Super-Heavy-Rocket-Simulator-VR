#pragma once

#include <memory>
#include <string>
#include <vector>

#include "starsim/gnc/booster_return.hpp"
#include "starsim/gnc/scvx_landing.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

// Flip and landing of the ship at the tower (b3 of docs/superpowers/specs/2026-10-13-ship-return-design.md).
struct ShipLandingParameters {
    std::vector<std::string> flip_groups{"ship/sea_level_1", "ship/sea_level_2", "ship/sea_level_3"};
    double flip_throttle = 0.8;                                  // [E]
    double flip_tolerance_rad = 15.0 * constants::deg_to_rad;    // [E] axis from the vertical: the burn plans start
    double flip_settled_rate_radps = 0.2;                        // [E] ... once the flip's pitch/yaw rate is this low
    // The flip starts aerodynamically: aft flaps folded, forward flaps out - the tail loses its drag and falls (engines
    // down) while the engines spool up; below this tilt the attitude controller takes the flaps back [E].
    double flip_flaps_until_tilt_rad = 30.0 * constants::deg_to_rad;
    double first_plan_retry_s = 1.0;                             // a failed first plan is tried again this often
    ScvxLandingParameters scvx;                                  // ship preset: ship_landing_parameters()
};

// The ship's preset: engine phases 3 -> 2 -> 1 (user 2026-10-13) and planning limits for a ship whose last engine
// barely holds it (one Raptor 3 2.75 MN at sea level vs ~265 t: thrust/weight ~1.04).
ShipLandingParameters ship_landing_parameters();

// From the gate (belly down, ~100 m/s sink):
//  Flip        - the sea-level engines relight at the flip throttle and the attitude controller turns the axis to the
//                local vertical (engines down; gimbals, flaps and RCS).
//  LandingBurn - once the axis is within the flip tolerance: SCvx plans to the catch on the ship's pins (re-plans,
//                abort to the sea after repeated unreachable catches - ScvxLandingGuidance); a failed first plan is
//                retried every first_plan_retry_s while the flip attitude and thrust are held.
//  Landed      - pins on the arms, touchdown or impact: engines off.
// Commands as the booster's return guidance (ReturnCommand), so the session flies both the same way.
class ShipLandingGuidance {
public:
    explicit ShipLandingGuidance(LandingSite site, ShipLandingParameters parameters = ship_landing_parameters());

    ReturnCommand update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);

    ReturnPhase phase() const { return phase_; }
    void set_catch_contact(bool contact) { catch_contact_ = contact; }
    void set_realtime(bool realtime);
    const ScvxLandingGuidance* scvx() const { return scvx_.get(); }
    int first_plan_attempts() const { return first_plan_attempts_; }
    // The flip: from the gate until the axis is within the flip tolerance; height of the CG above the site then.
    double flip_duration_s() const { return flip_duration_s_; }
    double flip_end_height_m() const { return flip_end_height_m_; }

private:
    LandingSite site_;
    ShipLandingParameters parameters_;
    ReturnPhase phase_ = ReturnPhase::Flip;
    std::unique_ptr<ScvxLandingGuidance> scvx_;
    bool started_ = false;
    time::Epoch next_attempt_;
    time::Epoch flip_start_;
    double flip_duration_s_ = 0.0;
    double flip_end_height_m_ = 0.0;
    int first_plan_attempts_ = 0;
    bool catch_contact_ = false;
};

}  // namespace starsim::gnc
