#pragma once

#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::ephemeris {

enum class Body { Sun, Earth, Moon, EarthMoonBarycenter };

using StateVector = math::StateVector;

// Positions of natural bodies from a planetary ephemeris (DE440 by default).
class Ephemeris {
public:
    virtual ~Ephemeris() = default;

    // Geometric state of `target` relative to the Earth's centre, GCRF axes. No light-time correction.
    virtual StateVector state_gcrf(Body target, time::Epoch epoch) const = 0;

    // Gravitational parameter GM of the body, consistent with the ephemeris.
    virtual double gm_m3ps2(Body body) const = 0;

    // Apparent position of `target` relative to an observer (observer state relative to the Earth's
    // centre, GCRF axes), corrected for light time and stellar aberration. Use for everything that is
    // *seen*: rendering, eclipses, shadows. Geometric positions misplace e.g. the Moon's shadow on the
    // Earth by ~40 km (20.5" annual aberration).
    virtual math::Vec3 apparent_position_gcrf(Body target, time::Epoch epoch,
                                              const math::StateVector& observer_gcrf) const = 0;
};

}  // namespace starsim::ephemeris
