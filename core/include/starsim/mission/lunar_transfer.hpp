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
};

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
// periselene altitude and lunar inclination (B-plane style, arrival time free). Minimum-norm Newton
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

    TransferSolution solve(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                           const LunarArrivalTarget& target, std::optional<math::Vec3> initial_delta_v = {}) const;

    // Lambert-based first guess (Earth two-body) aimed beside the Moon at the target epoch.
    math::Vec3 lambert_guess(const math::StateVector& state_at_burn, time::Epoch burn_epoch,
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
    math::StateVector coast(const math::StateVector& state, time::Epoch from, time::Epoch to) const;

    const environment::GravityModel& gravity_;
    const ephemeris::Ephemeris& ephemeris_;
    const frames::BodyOrientation& moon_orientation_;
    double gm_earth_;
    double gm_moon_;
    std::optional<FiniteBurnModel> finite_;
};

}  // namespace starsim::mission
