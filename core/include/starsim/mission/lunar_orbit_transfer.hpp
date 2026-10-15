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
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

struct TwoBurnTransfer {
    bool feasible = false;
    std::string reason;    // German when refused
    gnc::BurnNode first;   // departure
    gnc::BurnNode second;  // predicted arrival burn (re-planned on arrival)
    double total_delta_v_mps = 0.0;
};

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
