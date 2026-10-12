#pragma once

#include "starsim/math/state_vector.hpp"

namespace starsim::orbit {

// Classical osculating elements. For circular or equatorial orbits the undefined angles are
// set to 0 and the remaining angle absorbs them (argument of latitude / true longitude).
struct KeplerianElements {
    double semi_major_axis_m = 0.0;  // negative for hyperbolic orbits
    double eccentricity = 0.0;
    double inclination_rad = 0.0;
    double raan_rad = 0.0;
    double argument_of_periapsis_rad = 0.0;
    double true_anomaly_rad = 0.0;
};

KeplerianElements elements_from_state(const math::StateVector& state, double gm_m3ps2);
math::StateVector state_from_elements(const KeplerianElements& elements, double gm_m3ps2);

// Specific orbital energy v^2/2 - mu/r.
double specific_energy(const math::StateVector& state, double gm_m3ps2);

// Exact two-body propagation by universal variables (all conic types, forward or backward).
math::StateVector propagate_two_body(const math::StateVector& state, double gm_m3ps2, double dt_s);

// Orbital period of an elliptic orbit.
double orbital_period_s(double semi_major_axis_m, double gm_m3ps2);

}  // namespace starsim::orbit
