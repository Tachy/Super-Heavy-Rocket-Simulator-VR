#pragma once

#include <optional>

#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/integrators/integrate.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/math/vec3.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::orbit {

// Circular restricted three-body problem, Earth-Moon, nondimensional (length: Earth-Moon distance, time: 1/n).
// Rotating frame: origin at the barycentre, x from Earth to Moon, z along the orbital angular momentum.
struct Cr3bpState {
    math::Vec3 position;
    math::Vec3 velocity;
};

integrators::State<6> cr3bp_derivative(double mu, const integrators::State<6>& y);
Cr3bpState cr3bp_propagate(double mu, const Cr3bpState& s, double duration);
double jacobi_constant(double mu, const Cr3bpState& s);

// Symmetric periodic orbit through the x-z plane with y'(0) only: corrects (x0, z0, vy0) so that after half the
// given period the orbit crosses y = 0 perpendicularly (vx = vz = 0). Empty when Newton does not converge.
std::optional<Cr3bpState> correct_periodic_orbit(double mu, const Cr3bpState& guess, double period);

// Instantaneous Earth-Moon rotating frame at an epoch (DE440): converts a nondimensional rotating state to an
// Earth-centred GCRF state (instantaneous distance and angular rate as units). The barycentre moves with the Moon
// (mu * r_moon); the frame's x is along the Earth -> Moon line, and the frame pulsates with the distance.
class EarthMoonRotatingFrame {
public:
    EarthMoonRotatingFrame(const ephemeris::Ephemeris& ephemeris, time::Epoch epoch);
    double mu() const;
    math::StateVector to_gcrf(const Cr3bpState& rotating) const;
    Cr3bpState from_gcrf(const math::StateVector& gcrf) const;
    double time_unit_s() const;    // 1 / instantaneous angular rate
    double length_unit_m() const;  // instantaneous Earth-Moon distance

private:
    math::Vec3 barycentre_velocity() const;

    double mu_ = 0.0;
    double length_m_ = 0.0;
    double omega_radps_ = 0.0;
    double radial_rate_mps_ = 0.0;  // d(length)/dt (eccentric lunar orbit)
    math::Vec3 x_, y_, z_;
    math::Vec3 barycentre_m_;  // relative to the Earth, GCRF
};

}  // namespace starsim::orbit
