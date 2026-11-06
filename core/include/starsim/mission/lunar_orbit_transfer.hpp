#pragma once

#include <optional>
#include <string>

#include "starsim/environment/gravity.hpp"
#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/mission/lunar_transfer.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/mission/position_targeter.hpp"
#include "starsim/report/report.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

struct TwoBurnTransfer {
    bool feasible = false;
    report::Report reason;  // ok when feasible; else why it is refused
    gnc::BurnNode first;   // departure
    gnc::BurnNode second;  // predicted arrival burn (re-planned on arrival)
    double total_delta_v_mps = 0.0;
};

// The NRHO -> LLO descent's cost for a landing site (lunar sites plan, ruling R12). The NRHO stays as it is; the
// flown transfer (nrho_to_llo, the mission's [budget] nrho_to_llo leg) departs at a perilune into the polar LLO through
// that point (2026-11-13 mission, first perilune after the insertion: 35.5 deg turned from the NRHO plane, 969 + 276
// m/s in the full model, 977 m/s two-body for the first burn), whose inertial plane the site turns through with the
// Moon (~13 deg a day). A site not in that plane at a pass within the wait costs a plane change, the cheaper of
// (two-body about the Moon):
// (a) in the departure burn: the plane through the perilune point and the site (llo_plane_through_site with the
//     perilune direction as the axis); the burn is the difference of the velocity vectors - the perilune velocity and
//     the transfer's aposelene speed for the LLO radius, horizontal, in the plane - minus the polar one (a combined
//     change of plane and speed, for a horizontal velocity the law of cosines dv = sqrt(v1^2 + v2^2 - 2 v1 v2 cos(di))
//     [S: Vallado, Fundamentals of Astrodynamics and Applications, 4th ed., sec. 6.4 "Combined Changes to Inclination
//     and Speed"]); not credited below the polar burn (the flight flies the polar transfer). A site near the antipode
//     of the perilune point needs a large turn about it (the turn axis passes through the site);
// (b) in the polar LLO: the least turn that holds the site, di = asin(|n . s|), as a pure plane change at a node,
//     dv = 2 v_llo sin(di / 2) [S: Vallado, sec. 6.4 "Inclination Only"].
struct NrhoSiteDeparture {
    time::Epoch departure;              // the reference perilune of the burn
    time::Epoch touchdown;              // the pass the cost is for
    double plane_change_rad = 0.0;      // the turn of the cheaper option
    bool in_llo = false;                // (b) rather than (a)
    double extra_delta_v_mps = 0.0;     // over the polar transfer
};

// `after`: the NRHO is joined then (the outbound plan's insertion); the departure is its first perilune at least the
// burn lead later (as LunarOrbitTransfer::nrho_to_llo). `touchdown`: planned (taken as it is), else the pass of least
// cost from the transfer's arrival + `first_pass_after_arrival_s` to `site_wait_s` later. Empty when the reference
// holds no such perilune.
std::optional<NrhoSiteDeparture> nrho_site_departure(const NrhoReference& nrho, const ephemeris::Ephemeris& ephemeris,
                                                     const frames::BodyOrientation& moon, time::Epoch after,
                                                     double latitude_rad, double longitude_rad, double llo_altitude_m,
                                                     std::optional<time::Epoch> touchdown,
                                                     double first_pass_after_arrival_s, double site_wait_s);

// Transfers between the NRHO reference and the low lunar orbit, and NRHO station keeping, in the full force model
// (the gravity model the flight uses). Burns are impulsive nodes; the session flies them as finite burns.
class LunarOrbitTransfer {
public:
    LunarOrbitTransfer(const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                       const frames::BodyOrientation& moon_orientation);

    // NRHO -> LLO: first burn at the next perilune of the reference after `epoch` (retrograde: transfer ellipse down to
    // the LLO altitude on the far side, targeted on periselene altitude and inclination), second = circularisation.
    TwoBurnTransfer nrho_to_llo(const math::StateVector& state, time::Epoch epoch, const NrhoReference& nrho,
                                const LloTarget& llo) const;

    // LLO -> NRHO: arrival anywhere on the reference within `search_s` (the NRHO plane turns with the Moon's orbit,
    // ~86 deg per revolution, the LLO plane stays inertial: a perilune is cheap only when the planes line up, every
    // ~27 days). Scanned on two-body Lambert about the Moon (arrival epoch, time of flight 0.25-4 d, departure phase on
    // the LLO) in one-day arrival windows; the window's cheapest candidates are refined by the position targeter in
    // the full force model; the first window whose refined total is <= max_total_mps is taken, else the cheapest
    // refined transfer. Second burn = velocity match on the reference.
    TwoBurnTransfer llo_to_nrho(const math::StateVector& state, time::Epoch epoch, const NrhoReference& nrho,
                                double search_s = 14.0 * 86400.0, double max_total_mps = 1000.0) const;

    // Station keeping in the NRHO at apolune: a burn `lead_s` after `epoch` aims the next perilune of the reference
    // (position targeter); empty when the predicted perilune miss is below `deadband_m` or the reference does not
    // cover the next perilune.
    std::optional<gnc::BurnNode> station_keeping(const math::StateVector& state, time::Epoch epoch,
                                                 const NrhoReference& nrho, double deadband_m = 1000.0,
                                                 double lead_s = 60.0) const;  // [E] burn delay after the decision

    // Second burn of a transfer flown so far: LLO circularisation at the predicted periselene, or the NRHO velocity
    // match at the targeted perilune.
    std::optional<gnc::BurnNode> arrival_burn(const math::StateVector& state, time::Epoch epoch,
                                              const TwoBurnTransfer& transfer, LunarOrbitKind target,
                                              const NrhoReference* nrho, const LloTarget& llo) const;

private:
    const ephemeris::Ephemeris& ephemeris_;
    LunarTransferTargeter transfer_;
    PositionTargeter position_;
    double gm_moon_;
};

}  // namespace starsim::mission
