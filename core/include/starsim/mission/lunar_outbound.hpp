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
#include "starsim/mission/lunar_transfer.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/mission/position_targeter.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

enum class LunarOrbitKind { Llo, Nrho };

struct LloTarget {
    double altitude_m = 100e3;                     // [S] spec: 100 km circular
    double inclination_rad = 0.5 * constants::pi;  // polar, relative to the MOON_ME equator
};

struct LunarOutboundParameters {
    LunarOrbitKind orbit = LunarOrbitKind::Llo;
    LloTarget llo;
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
    std::string reason;  // German when refused
    gnc::BurnNode tli;
    // NRHO only: powered swing-by at the flyby periselene, and the periselene point and epoch the MCCs aim at.
    std::optional<gnc::BurnNode> powered_flyby;
    PositionTarget flyby_target;
    gnc::BurnNode insertion;  // LLO: LOI at periselene; NRHO: insertion on the reference (re-planned in flight)
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

    // LLO trim at the next apsis: sets the opposite apsis to the target radius (Moon two-body); empty when both
    // apsides are within the tolerance.
    std::optional<gnc::BurnNode> llo_trim(const math::StateVector& state, time::Epoch epoch,
                                          const LunarOutboundParameters& p) const;

private:
    LunarArrivalTarget llo_target(const LunarOutboundParameters& p) const;
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
