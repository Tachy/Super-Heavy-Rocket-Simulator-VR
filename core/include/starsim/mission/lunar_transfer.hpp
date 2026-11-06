#pragma once

#include <optional>

#include "starsim/environment/gravity.hpp"
#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/mission/finite_burn.hpp"

namespace starsim::mission {

// Desired closest approach to the Moon.
struct LunarArrivalTarget {
    double periselene_altitude_m = 100e3;
    double inclination_rad = 90.0 * 3.14159265358979 / 180.0;  // relative to the lunar equator (MOON_ME)
    time::Epoch periselene_epoch;  // planned closest approach: used for the first guess, not a constraint
    // The arrival plane by its normal (Moon-centred, GCRF axes) instead of the inclination: the plane is turned about
    // the arrival axis (arrival_axis) onto the plane holding that axis closest to this normal - the B-plane angle;
    // the tilt of the axis itself is not a constraint (it would cost the transfer kilometres per second).
    std::optional<math::Vec3> plane_normal_moon_inertial;
    // The periselene epoch is a constraint too (three constraints, three unknowns): the arrival time - free otherwise
    // and drifting along a nearly flat family (a day for ~2 m/s of TLI) - fixes the asymptote, so a plane chosen
    // through a site at a touchdown stays the plane through it.
    bool hold_arrival_epoch = false;
};

// The direction an arrival plane turns about without a plane change (unit, Moon-centred GCRF axes): the incoming
// asymptote of a hyperbolic arrival; for a closed arc (a transfer from a lunar orbit) the aposelene direction, where
// such a transfer starts.
math::Vec3 arrival_axis(const math::StateVector& relative_state_at_periselene, double gm_moon_m3ps2);

// Predicted closest approach for a given trajectory.
struct LunarArrival {
    bool reached = false;          // false if the trajectory never comes close to the Moon
    time::Epoch periselene_epoch;
    double periselene_altitude_m = 0.0;
    double inclination_rad = 0.0;
    math::StateVector relative_state;  // Moon-centred, GCRF axes, at periselene
};

struct TransferSolution {
    bool converged = false;
    int iterations = 0;
    gnc::BurnNode burn;            // impulsive manoeuvre (executed as a finite burn)
    LunarArrival arrival;
};

// Targets a lunar arrival in the full force model (Earth-centred gravity with Moon and Sun, as used by
// the flight). Unknowns: the three components of an impulsive delta-v at `burn_epoch`; constraints:
// periselene altitude and lunar inclination or the plane's turn about the arrival axis (B-plane style, arrival time
// free). Minimum-norm Newton
// steps with a finite-difference Jacobian and line search; first guess from a Lambert arc to the Moon
// (or the given guess, e.g. zero for a mid-course correction).
class LunarTransferTargeter {
public:
    LunarTransferTargeter(const environment::GravityModel& earth_centred_gravity, const ephemeris::Ephemeris& ephemeris,
                          const frames::BodyOrientation& moon_orientation);

    // Coasts `state` (Earth-centred, at `epoch`) to `burn_epoch`, applies `delta_v` and finds the closest
    // approach to the Moon.
    LunarArrival evaluate(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                          const math::Vec3& delta_v_mps) const;

    // Without `initial_delta_v` the Lambert guess is used; when its Newton iteration stalls (the guessed arc hits the
    // Moon and the steps creep along the family of impacting arcs) the guess past the opposite limb is tried.
    TransferSolution solve(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                           const LunarArrivalTarget& target, std::optional<math::Vec3> initial_delta_v = {}) const;

    // Lambert-based first guess (Earth two-body) aimed beside the Moon at the target epoch: past the limb on the
    // `side` (+1 / -1) of the transfer plane's normal x Moon direction.
    math::Vec3 lambert_guess(const math::StateVector& state_at_burn, time::Epoch burn_epoch,
                             const LunarArrivalTarget& target, double side = 1.0) const;
    // Lambert-based first guess for a target plane: aimed past the Moon at the B-vector of that plane (asymptote from
    // the arc to the Moon's centre x the plane normal). Empty without a plane or a Lambert arc.
    std::optional<math::Vec3> plane_guess(const math::StateVector& state_at_burn, time::Epoch burn_epoch,
                                          const LunarArrivalTarget& target) const;

    // Searches one parking-orbit period after `earliest` (in `step_s` steps) for the burn time with the
    // smallest Lambert delta-v that arrives at `arrival_epoch`. Returns the burn epoch.
    time::Epoch best_departure(const math::StateVector& state, time::Epoch epoch, time::Epoch earliest,
                               time::Epoch arrival_epoch, double step_s = 60.0) const;

    // Launch-window helper: right ascension of the ascending node for a parking orbit of the given
    // inclination whose plane contains the Moon's direction at `arrival_epoch` (TLI then needs no plane
    // change). Empty if the Moon's declination exceeds the inclination.
    std::optional<double> parking_orbit_raan(time::Epoch arrival_epoch, double inclination_rad) const;

    // Lunar orbit insertion: retrograde burn at periselene to a circular orbit.
    static gnc::BurnNode orbit_insertion(const LunarArrival& arrival, double gm_moon_m3ps2);

    // The burn is flown as a finite burn of these engines (apply_finite_burn); empty: impulsive.
    void set_finite_burn(std::optional<FiniteBurnModel> model) { finite_ = model; }

private:
    // The Newton iteration from the first guess `dv`.
    TransferSolution newton(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                            const LunarArrivalTarget& target, math::Vec3 dv) const;
    math::StateVector coast(const math::StateVector& state, time::Epoch from, time::Epoch to) const;

    const environment::GravityModel& gravity_;
    const ephemeris::Ephemeris& ephemeris_;
    const frames::BodyOrientation& moon_orientation_;
    double gm_earth_;
    double gm_moon_;
    std::optional<FiniteBurnModel> finite_;
};

}  // namespace starsim::mission
