#pragma once

#include <string>

#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/time/time_scale_converter.hpp"

// SPICE-backed implementations of the core interfaces. Require a live spice::SpiceSession.

namespace starsim::spice {

class SpiceTimeConverter final : public time::TimeScaleConverter {
public:
    time::Epoch from_utc(const time::UtcDateTime& utc) const override;
    time::UtcDateTime to_utc(time::Epoch epoch) const override;
    double tdb_minus_utc_s(time::Epoch epoch) const override;
};

class SpiceEphemeris final : public ephemeris::Ephemeris {
public:
    ephemeris::StateVector state_gcrf(ephemeris::Body target, time::Epoch epoch) const override;
    double gm_m3ps2(ephemeris::Body body) const override;
    math::Vec3 apparent_position_gcrf(ephemeris::Body target, time::Epoch epoch,
                                      const math::StateVector& observer_gcrf) const override;
};

// Orientation of a SPICE body-fixed frame relative to J2000 (= ICRF / GCRF axes in DE4xx kernels).
class SpiceOrientation final : public frames::BodyOrientation {
public:
    explicit SpiceOrientation(std::string frame_name) : frame_name_(std::move(frame_name)) {}

    // Earth: ITRF93 from the NAIF high-precision binary PCKs (IERS EOP based).
    static SpiceOrientation earth() { return SpiceOrientation("ITRF93"); }
    // Moon: principal-axes frame from the DE440 lunar libration solution.
    static SpiceOrientation moon() { return SpiceOrientation("MOON_PA"); }
    // Moon: mean-Earth/polar-axis frame, the reference frame of lunar maps and textures (differs from
    // MOON_PA by ~0.8 km on the surface).
    static SpiceOrientation moon_mean_earth() { return SpiceOrientation("MOON_ME"); }

    frames::FixedToInertial fixed_to_gcrf(time::Epoch epoch) const override;

private:
    std::string frame_name_;
};

}  // namespace starsim::spice
