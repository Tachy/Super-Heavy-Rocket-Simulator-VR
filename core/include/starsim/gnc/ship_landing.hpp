#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "starsim/gnc/booster_return.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/gnc/scvx_landing.hpp"
#include "starsim/gnc/upright_descent.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

// Flip and landing of the ship at the tower (b3 of docs/superpowers/archive/specs/2026-10-13-ship-return-design.md).
struct ShipLandingParameters {
    std::vector<std::string> flip_groups{"ship/sea_level_1", "ship/sea_level_2", "ship/sea_level_3"};
    double flip_throttle = 0.8;                                  // [E]
    double flip_tolerance_rad = 15.0 * constants::deg_to_rad;    // [E] axis from the vertical: the burn plans start
    double flip_settled_rate_radps = 0.2;                        // [E] ... once the flip's pitch/yaw rate is this low
    // The flip starts aerodynamically: aft flaps folded, forward flaps out - the tail loses its drag and falls (engines
    // down) while the engines spool up; below this tilt the attitude controller takes the flaps back [E].
    double flip_flaps_until_tilt_rad = 30.0 * constants::deg_to_rad;
    double first_plan_retry_s = 1.0;                             // a failed first plan is tried again this often
    // [E] the flip pushes the ship ~100-200 m along its nose (thrust across the path while it turns): the entry's gate
    // lies this far back from the catch axis on the line tower -> pad, the nose pointing to the tower
    double gate_lead_m = 220.0;
    // Ship catch robustness (2026-10-21): the last-resort abort (gnc::upright_descent) after this many failed first
    // plans (neither the catch nor the sea in reach), or when an abort has no converged abort plan after the grace
    // time or leans the hull beyond the guard tilt.
    UprightDescentParameters upright_descent;
    int first_plans_before_descent = 3;                               // [E]
    double abort_plan_grace_s = 2.0;                                  // [E] two re-plan cycles
    // [E] abort plans fly up to ~24 deg (20 deg limit + thrust axis + tracking); seed 18 tumbled from 45 deg on
    double upright_guard_tilt_rad = 35.0 * constants::deg_to_rad;
    double descent_two_engine_sink_mps = 20.0;                        // [D] the 3 -> 2 switch window starts at 20 m/s
    // [E] an abort switches to the descent once the usable propellant is below this times a vertical stop's need
    double descent_propellant_margin = 1.3;
    ScvxLandingParameters scvx;                                  // ship preset: ship_landing_parameters()
};

// The ship's preset: engine phases 3 -> 2 (user 2026-10-13, flight 13/14 videos: the belly engine shuts down, the
// leeward pair lands; its thrust centroid lies in the symmetry plane) and planning limits for the ship.
ShipLandingParameters ship_landing_parameters();

// Where the ship's entry guidance aims: with a catch tower `gate_lead_m` back from the catch axis on the line
// tower -> pad, the nose to the tower: the flip pushes the ship towards the arms, the final approach comes from the open
// side between them and the side pins lie across them; else the pad centre.
EntryTarget ship_entry_target(const LandingSite& site, const ShipLandingParameters& parameters);

// From the gate (belly down, ~100 m/s sink):
//  Flip        - the sea-level engines relight at the flip throttle and the attitude controller turns the axis to the
//                local vertical (engines down; gimbals, flaps and RCS).
//  LandingBurn - once the axis is within the flip tolerance: SCvx plans to the catch on the ship's pins (re-plans,
//                abort to the sea after repeated unreachable catches - ScvxLandingGuidance); a failed first plan is
//                retried every first_plan_retry_s while the flip attitude and thrust are held.
//  Upright descent (either phase, sticky) - the last-resort abort: no plan reaches the catch or the sea, or an abort
//                plan is missing / leans the hull too far: gnc::upright_descent keeps the hull upright and slows it.
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
    // Gives the catch up (pilot or tower command): the landing burn plans for the sea. False (refused) without a sea,
    // once down or with the pins on the arms.
    bool request_abort();
    int first_plan_attempts() const { return first_plan_attempts_; }
    // The catch the landing burn's plan flies to (its end); none before the first plan, once landed, while aborting
    // and in the upright descent (onboard prediction for the mission timeline).
    std::optional<time::Epoch> catch_epoch() const {
        if (phase_ != ReturnPhase::LandingBurn || !scvx_ || upright_descent_ || scvx_->aborting()) return std::nullopt;
        return scvx_->plan_end_epoch();
    }
    // The flip: from the gate until the axis is within the flip tolerance; height of the CG above the site then.
    double flip_duration_s() const { return flip_duration_s_; }
    double flip_end_height_m() const { return flip_end_height_m_; }
    // The last-resort abort flies (sticky): upright descent instead of plans.
    bool upright_descent_active() const { return upright_descent_; }
    // First tick resting or impacted: axis tilt from the local vertical and sink speed (Earth-fixed).
    bool has_contact() const { return contact_; }
    double contact_tilt_rad() const { return contact_tilt_rad_; }
    double contact_sink_mps() const { return contact_sink_mps_; }

private:
    void print_gate_state(const char* label, const vehicle::FlightBody& body, const frames::FixedToInertial& transform,
                          time::Epoch epoch) const;
    // The upright descent's engines: three while sinking faster than the 3 -> 2 window, then the landing pair.
    const std::vector<std::string>& descent_groups(double sink_mps) const {
        return sink_mps > parameters_.descent_two_engine_sink_mps ? parameters_.flip_groups
                                                                  : parameters_.scvx.snapshot.phase_groups[1];
    }
    void fly_upright_descent(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                             const frames::FixedToInertial& transform, const math::Vec3& site_up, ReturnCommand& cmd) const;

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
    bool abort_requested_ = false;
    bool upright_descent_ = false;
    std::optional<time::Epoch> abort_since_;
    bool contact_ = false;
    double contact_tilt_rad_ = 0.0;
    double contact_sink_mps_ = 0.0;
};

}  // namespace starsim::gnc
