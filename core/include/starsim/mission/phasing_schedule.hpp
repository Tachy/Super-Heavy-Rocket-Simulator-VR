#pragma once

#include <string>

#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

// Coelliptic rendezvous after the Crew Dragon pattern (spec i3; plan docs/superpowers/plans/2026-10-19-iss-i3-phasing.md
// lists the Crew-2 timeline the values are taken from).
struct PhasingParameters {
    double coelliptic_height_m = 2900.0;  // [D] Crew-2: 4.9 m/s closing between 30 and 10 km = 1.5 n dh
    double approach_initiation_range_m = 7500.0;  // [S] Crew-2: approach initiation burn at 7.5 km
    double approach_transfer_s = 2700.0;   // [S] Crew-2: approach initiation 03:40 -> Waypoint 0 04:25
    double mid_course_after_s = 1500.0;    // [S] Crew-2: approach initiation midcourse burn 04:05
    double waypoint0_below_m = 400.0;      // [S] Crew-2: Waypoint 0, 400 m below the ISS
    double coelliptic_coast_s = 10080.0;   // [S] Crew-2: coelliptic burn 00:53 -> approach initiation 03:40
    double first_burn_lead_s = 1800.0;     // [E] checkout after the separation (Crew-2 phase burn 37 min, Demo-2 49 min)
    double mission_limit_s = 28.0 * 3600.0;  // [S] crew missions dock 24-28 h after the launch
    double earliest_arrival_s = 0.0;       // Waypoint 0 not before (a crew timeline); 0: the earliest window
    // [D] smaller corrections are skipped: one skipped carries ~0.02 m/s x 1200 s = 24 m to Waypoint 0 (0.05 left 60 m)
    double mid_course_threshold_mps = 0.02;
    double reserve_kg = 900.0;  // [E] deorbit 598 kg (d3: 139 m/s from 420 km) + proximity operations and abort
    // C_D A / m of the planner's coasts (gnc::coast_with_drag); the session fills them from its tracking.
    double chaser_drag_area_per_mass_m2pkg = 0.0;
    double target_drag_area_per_mass_m2pkg = 0.0;
};

// A phasing orbit a chaser raises to after its insertion (refuelling f1: chosen per launch window).
struct PhasingOrbit {
    double periapsis_radius_m = 0.0;
    double apoapsis_radius_m = 0.0;
};

// Burn epochs of the coelliptic profile from mean motions with the secular J2 rates (two-body otherwise): the Dragon
// either waits on its orbit until the phase is right and boosts (phasing_revolutions = 0, the earliest window) or, when
// that would arrive before earliest_arrival_s, slows the catch-up with a prograde phase burn at first_burn_lead_s (an
// apoapsis raise) and boosts after `phasing_revolutions` revolutions at the burn point. A phase that needs more than
// one full catch-up lap adds `extra_laps`. Boost at the start (or phasing) orbit -> half a transfer ellipse -> coelliptic
// burn coelliptic_height_m below the target -> coelliptic coast -> approach initiation -> Waypoint 0.
struct PhasingSchedule {
    bool feasible = false;
    std::string reason;  // German when refused
    int phasing_revolutions = 0;
    int extra_laps = 0;
    double phase_orbit_apoapsis_radius_m = 0.0;  // the phase burn's target apoapsis (0 without one)
    bool above = false;  // the chaser above the target: the target catches up, the boost descends (refuelling f1)
    time::Epoch phase_epoch, boost_epoch, coelliptic_epoch, approach_initiation_epoch, waypoint0_epoch;
};

PhasingSchedule phasing_schedule(const math::StateVector& chaser, const math::StateVector& target, time::Epoch epoch,
                                 const PhasingParameters& parameters, double gm_m3ps2);

// Along-track distance behind the target at the coelliptic burn: the approach initiation point's horizontal distance
// plus the coelliptic closing (1.5 n dh) over the coelliptic coast.
double coelliptic_behind_m(const PhasingParameters& parameters, double target_radius_m, double gm_m3ps2);

}  // namespace starsim::mission
