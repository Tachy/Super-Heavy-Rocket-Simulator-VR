#pragma once

#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/math/vec3.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::environment {

// Gravitational acceleration of a spacecraft whose position is given relative to the model's
// central body, in GCRF axes. The central body's own acceleration is subtracted (indirect term),
// so the result is valid in the non-rotating, body-centred frame.
class GravityModel {
public:
    virtual ~GravityModel() = default;

    virtual ephemeris::Body central_body() const = 0;
    virtual math::Vec3 acceleration_mps2(const math::Vec3& position_m, time::Epoch epoch) const = 0;
};

// Fidelity flags of the default gravity model (all switchable, see CLAUDE.md).
struct PointMassGravityConfig {
    ephemeris::Body central_body = ephemeris::Body::Earth;  // Earth or Moon
    bool earth = true;     // Earth as third body (only relevant when central body is the Moon)
    bool moon = true;      // Moon as third body (only relevant when central body is the Earth)
    bool sun = true;       // Sun as third body
    bool earth_j2 = true;  // Earth oblateness; applied only when the Earth is the central body
};

// Point masses of Earth, Moon and Sun (DE440 positions and GM values) plus Earth J2.
// Earth J2 uses the true pole direction (ITRF z-axis) from the Earth orientation model.
class PointMassGravity final : public GravityModel {
public:
    PointMassGravity(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& earth_orientation,
                     PointMassGravityConfig config = {});

    ephemeris::Body central_body() const override { return config_.central_body; }
    math::Vec3 acceleration_mps2(const math::Vec3& position_m, time::Epoch epoch) const override;

    const PointMassGravityConfig& config() const { return config_; }

private:
    math::Vec3 third_body(ephemeris::Body body, double gm, const math::Vec3& position_m, time::Epoch epoch) const;
    math::Vec3 earth_j2(const math::Vec3& position_m, time::Epoch epoch) const;

    const ephemeris::Ephemeris& ephemeris_;
    const frames::BodyOrientation& earth_orientation_;
    PointMassGravityConfig config_;
    double gm_earth_;
    double gm_moon_;
    double gm_sun_;
    double gm_central_;
};

// Reference radius belonging to constants::earth_j2 (EGM2008).
inline constexpr double earth_j2_reference_radius_m = 6378136.3;

}  // namespace starsim::environment
