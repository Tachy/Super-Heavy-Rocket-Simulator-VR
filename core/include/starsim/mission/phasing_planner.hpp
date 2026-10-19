#pragma once

#include <optional>
#include <string>
#include <vector>

#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/mission/finite_burn.hpp"
#include "starsim/mission/phasing_schedule.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

enum class PhasingBurnKind { Phase, Boost, Coelliptic, ApproachInitiation, MidCourse, Braking };
const char* german_name(PhasingBurnKind kind);

struct PhasingBurn {
    PhasingBurnKind kind = PhasingBurnKind::Boost;
    gnc::BurnNode node;
};

struct PhasingPlan {
    bool feasible = false;
    std::string reason;  // German when refused
    PhasingSchedule schedule;
    std::vector<PhasingBurn> burns;  // in order; the mid-course correction is planned in flight only
    double total_delta_v_mps = 0.0;
    double propellant_kg = 0.0;      // rocket equation at the engines' Isp
    double min_ellipsoid_metric_before_ai = 0.0;  // of the planned arc, sampled every minute; > 1: outside
    PhasingParameters parameters;
};

// Coelliptic phasing in the full force model (the flight's gravity model) with orbital drag on both bodies
// (gnc::coast_with_drag with the parameters' C_D A / m). Timing from phasing_schedule; then each burn is targeted from
// the state the previous one leaves, the burn flown as a finite burn of `engines` (apply_finite_burn):
// - Phase: prograde magnitude (secant) so that the chaser is half a transfer behind the coelliptic aim point at the boost;
// - Boost: prograde magnitude (secant) so that the radius at the coelliptic epoch is the coelliptic radius (a
//   180 deg position target would be singular out of plane); waiting plans solve the boost epoch for the phase first;
// - Coelliptic: position targeter to the approach initiation point (range 7.5 km, dh below) at its epoch (3D: removes
//   the residual plane error);
// - Approach initiation: position targeter (CW first guess) to Waypoint 0 at its epoch;
// - Mid-course (in flight only): the same target, zero first guess;
// - Braking at Waypoint 0: the velocity that leaves the chaser at rest in the target's LVLH frame.
class PhasingPlanner {
public:
    explicit PhasingPlanner(const environment::GravityModel& gravity);

    // `engines`: the burn engines at the current mass; `propellant_kg`: on board (budget against reserve_kg).
    PhasingPlan plan(const math::StateVector& chaser, const math::StateVector& target, time::Epoch epoch,
                     const PhasingParameters& parameters, const FiniteBurnModel& engines, double propellant_kg) const;

    // The burn of `kind` re-targeted from the live states at `epoch` (before the burn), on the plan's epochs. Empty when
    // the targeting does not converge, the plan has no such burn, or a mid-course correction is below its threshold.
    std::optional<gnc::BurnNode> retarget(PhasingBurnKind kind, const math::StateVector& chaser,
                                          const math::StateVector& target, time::Epoch epoch, const PhasingPlan& plan,
                                          const FiniteBurnModel& engines) const;

    // Along-track miss (m) of the boost geometry predicted from the live states: the chaser's angle at the boost epoch
    // against the coelliptic aim point minus half a transfer, times the target radius; > 0: the chaser is late.
    double boost_phase_error_m(const math::StateVector& chaser, const math::StateVector& target, time::Epoch epoch,
                               const PhasingPlan& plan) const;

private:
    math::StateVector coast(const math::StateVector& s, time::Epoch from, time::Epoch to, double beta) const;
    double phase_error_rad(const math::StateVector& chaser_at_boost, const math::StateVector& target_at_boost,
                           time::Epoch boost_epoch, const PhasingParameters& p, double boost_to_coelliptic_s) const;
    std::optional<gnc::BurnNode> solve_phase(const math::StateVector& chaser, const math::StateVector& target,
                                             time::Epoch epoch, const PhasingPlan& plan,
                                             const FiniteBurnModel& engines) const;
    std::optional<gnc::BurnNode> solve_boost(const math::StateVector& chaser, const math::StateVector& target,
                                             time::Epoch epoch, const PhasingPlan& plan,
                                             const FiniteBurnModel& engines) const;
    std::optional<gnc::BurnNode> solve_position(PhasingBurnKind kind, const math::StateVector& chaser,
                                                const math::StateVector& target, time::Epoch epoch,
                                                const PhasingPlan& plan, const FiniteBurnModel& engines) const;
    gnc::BurnNode braking(const math::StateVector& chaser, const math::StateVector& target, time::Epoch epoch,
                          const PhasingPlan& plan) const;

    const environment::GravityModel& gravity_;
};

}  // namespace starsim::mission
