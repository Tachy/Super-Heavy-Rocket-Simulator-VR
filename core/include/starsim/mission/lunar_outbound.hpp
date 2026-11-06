#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "starsim/environment/gravity.hpp"
#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/report/report.hpp"
#include "starsim/mission/lunar_transfer.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/mission/position_targeter.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

enum class LunarOrbitKind { Llo, Nrho };

// The LLO: a plane chosen by its normal (Moon-centred, GCRF axes), else by the inclination (the node then follows the
// arrival: polar by default, explicit missions).
struct LloTarget {
    double altitude_m = 100e3;  // [S] spec: 100 km circular
    std::optional<math::Vec3> plane_normal_moon_inertial;
    double inclination_rad = 0.5 * constants::pi;  // relative to the MOON_ME equator; used without a plane normal
};

// The landing the LLO is chosen for (user 2026-11-06: "die Neigung des Orbits kann frei gewaehlt werden"): the plane
// holds the site at the touchdown epoch (llo_plane_through_site); MOON_ME latitude / longitude.
struct LloSiteTarget {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    std::optional<time::Epoch> touchdown;     // planned (a window, the mission's touchdown_after_epoch_h) ...
    double first_pass_after_arrival_s = 0.0;  // ... else this long after the transfer's arrival
};

// The plane that holds the site at `touchdown` and the arrival axis `arrival_asymptote_moon_inertial` (the incoming
// asymptote: every plane through it is reached at the LOI cost of the arrival, so this one costs no plane change). Of
// its two senses the one of `arrival_normal_moon_inertial` (the arrival plane it is turned from: the other sense passes
// the Moon on its other side - another transfer), without one the prograde (normal on the Moon's north side). A site
// (nearly) on the axis is in every plane through it: then the plane through the axis closest to the sense plane (the
// arrival plane itself: no turn; without one the plane of least inclination). Unit normal, Moon-centred GCRF axes. The
// same holds for a transfer from a lunar orbit with its burn point as the axis (nrho_site_departure).
math::Vec3 llo_plane_through_site(const frames::BodyOrientation& moon, double latitude_rad, double longitude_rad,
                                  time::Epoch touchdown, const math::Vec3& arrival_asymptote_moon_inertial,
                                  const std::optional<math::Vec3>& arrival_normal_moon_inertial = std::nullopt);

// Approach azimuth over the site (rad from north, east positive) of a circular orbit of radius `llo_radius_m` in the
// plane `plane_normal_moon_inertial` (its sense gives the direction of motion) at `touchdown`: the ground velocity
// (inertial minus the Moon's rotation) on the site's horizontal, as LunarDescentPlanner computes it at a pass. The
// mission start checks the touchdown point for this approach (plan 2026-11-06 ruling R13).
double llo_approach_azimuth(const frames::BodyOrientation& moon, double latitude_rad, double longitude_rad,
                            time::Epoch touchdown, const math::Vec3& plane_normal_moon_inertial, double llo_radius_m,
                            double gm_moon_m3ps2);

struct LunarOutboundParameters {
    LunarOrbitKind orbit = LunarOrbitKind::Llo;
    LloTarget llo;
    // LLO without a plane normal: the plane is chosen through this site at its touchdown (plan() records it in
    // OutboundPlan::llo_plane_normal_moon_inertial; the later solves of that plan - finite TLI, MCCs - aim at it and
    // hold the plan's arrival epoch).
    std::optional<LloSiteTarget> llo_site;
    std::shared_ptr<const NrhoReference> nrho;  // required for Nrho
    // LLO: planned periselene. NRHO: the reference's perilune the flight joins: the powered flyby happens about then
    // and the insertion follows on the arc after it.
    time::Epoch arrival_epoch;
    double earliest_departure_s = 1800.0;  // after the plan's epoch
    std::vector<double> mcc_after_tli_s{86400.0, 2.5 * 86400.0};  // [E] spec: ~1 and ~2.5 days
    double mcc_threshold_mps = 0.05;      // [E] smaller corrections are skipped
    double llo_trim_tolerance_m = 200.0;  // [E] apsis errors below are not trimmed
    double reserve_fraction = 0.05;       // [E] propellant margin on top of the planned burns
    // NRHO in-direct transfer [S: Kikuchi, Matsumoto & Ueda, ISSFD 2024]: powered lunar swing-by at 100 km, NRHO
    // insertion at true anomaly 160 deg (about a day after the swing-by).
    double flyby_altitude_m = 100e3;
    double nrho_insertion_true_anomaly_rad = 160.0 * constants::deg_to_rad;
    // Pilot-flown mid-course corrections (user 2026-10-15; Apollo: the computer planned, the crew flew the burn and
    // trimmed the residuals): the correction is announced `pilot_call_lead_s` before its epoch and the window closes
    // `pilot_window_s` after it.
    bool pilot_corrections = false;
    double pilot_call_lead_s = 600.0;  // [E]
    double pilot_window_s = 1800.0;    // [E]
};

struct OutboundPlan {
    bool feasible = false;
    report::Report reason;  // ok when feasible; else why it is refused
    gnc::BurnNode tli;
    // NRHO only: powered swing-by at the flyby periselene, and the periselene point and epoch the MCCs aim at.
    std::optional<gnc::BurnNode> powered_flyby;
    PositionTarget flyby_target;
    gnc::BurnNode insertion;  // LLO: LOI at periselene; NRHO: insertion on the reference (re-planned in flight)
    // LLO: the plane the TLI aims at (the parameters' normal, or the one chosen through llo_site); empty when the
    // inclination is targeted.
    std::optional<math::Vec3> llo_plane_normal_moon_inertial;
    time::Epoch llo_arrival_epoch;           // with a plane: the arrival the later solves hold (its asymptote)
    std::optional<time::Epoch> llo_touchdown;  // with llo_site: the touchdown the plane holds the site at
    double total_delta_v_mps = 0.0;
    double propellant_kg = 0.0;  // rocket equation on the vacuum Raptors
};

// Plans the outbound flight from an Earth parking orbit to the 100 km LLO or the NRHO in the full force model
// (the gravity model the flight uses): TLI, mid-course corrections, (NRHO) powered swing-by, insertion and LLO apsis
// trims. The burns are impulsive nodes; the session flies them as finite burns and re-plans at every step.
//
// NRHO: a direct transfer meets the NRHO perilune head-on (a low-energy arrival is overtaken by the Moon and passes
// against its motion; the southern L2 NRHO's perilune runs with it: 160 deg apart, 3.5 km/s insertion). The flight
// therefore swings by the Moon with a braking burn and joins the NRHO a day later (in-direct transfer).
class LunarOutboundPlanner {
public:
    LunarOutboundPlanner(const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                         const frames::BodyOrientation& moon_orientation);

    // TLI from the parking-orbit state; checks the propellant (mass, Isp, propellant on board).
    OutboundPlan plan(const math::StateVector& state, time::Epoch epoch, const LunarOutboundParameters& p,
                      double mass_kg, double isp_s, double propellant_kg) const;

    // The plan's TLI re-solved for the finite burn the session flies (`engines` at the current mass), from the parking
    // orbit state at `epoch`; same node epoch, the impulsive TLI as the first guess. LLO: periselene altitude and
    // inclination; NRHO: the flyby point at its epoch. Empty if not converged.
    std::optional<gnc::BurnNode> finite_tli(const math::StateVector& state, time::Epoch epoch,
                                            const LunarOutboundParameters& p, const OutboundPlan& plan,
                                            const FiniteBurnModel& engines) const;

    // Mid-course correction at `burn_epoch` (zero first guess): LLO - B-plane to the periselene; NRHO - the plan's
    // flyby point at its epoch. Empty if below the threshold or not converged.
    std::optional<gnc::BurnNode> mid_course(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                            const LunarOutboundParameters& p, const OutboundPlan& plan) const;

    // NRHO: the powered swing-by re-planned from the current state (burn at the plan's flyby epoch, reaching the
    // reference at the insertion epoch). Empty for the LLO or when not converged.
    std::optional<gnc::BurnNode> powered_flyby(const math::StateVector& state, time::Epoch epoch,
                                               const LunarOutboundParameters& p, const OutboundPlan& plan) const;

    // Insertion: LLO - circular at the predicted periselene; NRHO - reference velocity minus the arrival velocity at
    // the plan's insertion epoch.
    std::optional<gnc::BurnNode> insertion(const math::StateVector& state, time::Epoch epoch,
                                           const LunarOutboundParameters& p, const OutboundPlan& plan) const;

    // LLO trim at the first apsis at least `min_lead_s` ahead (shorter than one period): sets the opposite apsis to
    // the target radius (Moon two-body); empty when both apsides are within the tolerance.
    std::optional<gnc::BurnNode> llo_trim(const math::StateVector& state, time::Epoch epoch,
                                          const LunarOutboundParameters& p, double min_lead_s = 0.0) const;

private:
    // The LLO arrival the solves aim at: the parameters' plane normal, else the plan's chosen plane, else the inclination.
    LunarArrivalTarget llo_target(const LunarOutboundParameters& p, const OutboundPlan* plan = nullptr) const;
    // The TLI into the plane through p.llo_site (fixed point on the arrival asymptote, arrival held); records the plane,
    // the held arrival and the touchdown in `plan`.
    TransferSolution solve_through_site(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                        const LunarOutboundParameters& p, OutboundPlan& plan) const;
    bool plan_nrho(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                   const LunarOutboundParameters& p, OutboundPlan& plan) const;
    std::optional<time::Epoch> nrho_insertion_epoch(const LunarOutboundParameters& p) const;

    const ephemeris::Ephemeris& ephemeris_;
    const frames::BodyOrientation& moon_orientation_;
    LunarTransferTargeter transfer_;
    PositionTargeter position_;
    double gm_moon_;
};

// Moon-relative state (GCRF axes) of an Earth-centred state.
math::StateVector moon_relative(const ephemeris::Ephemeris& ephemeris, const math::StateVector& state, time::Epoch t);

}  // namespace starsim::mission
