#pragma once

#include "starsim/math/vec3.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::ephemeris {

enum class Body { Sun, Earth, Moon, EarthMoonBarycenter };

// Position and velocity of a body in GCRF (Earth-centred, ICRF-aligned axes).
struct StateVector {
    math::Vec3 position_m;
    math::Vec3 velocity_mps;
};

// Positions of natural bodies from a planetary ephemeris (DE440 by default).
class Ephemeris {
public:
    virtual ~Ephemeris() = default;

    // Geometric state of `target` relative to the Earth's centre, GCRF axes. No light-time correction.
    virtual StateVector state_gcrf(Body target, time::Epoch epoch) const = 0;

    // Gravitational parameter GM of the body, consistent with the ephemeris.
    virtual double gm_m3ps2(Body body) const = 0;
};

}  // namespace starsim::ephemeris
