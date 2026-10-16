#pragma once

#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::environment {

// A reference body's body-fixed frame at one epoch: GCRF axes, centre relative to the Earth's centre.
struct BodyFrame {
    frames::FixedToInertial transform;      // body-fixed axes -> GCRF axes
    math::Vec3 centre_gcrf_m;               // the body's centre (Earth: zero)
    math::Vec3 centre_velocity_gcrf_mps;
    math::Vec3 rotation_gcrf_radps;         // the body's rotation vector

    static BodyFrame centred_at_origin(const frames::FixedToInertial& transform) {
        return {transform, {}, {}, frames::rotation_vector(transform)};
    }
    math::Vec3 to_fixed(const math::Vec3& position_gcrf_m) const {
        return transpose(transform.rotation) * (position_gcrf_m - centre_gcrf_m);
    }
    math::Vec3 position_gcrf(const math::Vec3& fixed_m) const { return centre_gcrf_m + transform.position(fixed_m); }
    math::Vec3 velocity_gcrf(const math::Vec3& fixed_m, const math::Vec3& fixed_velocity_mps) const {
        return centre_velocity_gcrf_mps + transform.velocity(fixed_m, fixed_velocity_mps);
    }
    // Velocity relative to the rotating body (for the Earth: relative to the co-rotating air).
    math::Vec3 relative_velocity(const math::Vec3& position_gcrf_m, const math::Vec3& velocity_gcrf_mps) const {
        return velocity_gcrf_mps - centre_velocity_gcrf_mps - cross(rotation_gcrf_radps, position_gcrf_m - centre_gcrf_m);
    }
    // Inertial state relative to the centre (GCRF axes): orbit elements, LVLH.
    math::StateVector centred(const math::StateVector& gcrf) const {
        return {gcrf.position_m - centre_gcrf_m, gcrf.velocity_mps - centre_velocity_gcrf_mps};
    }
};

// Everything that is relative to a celestial body (lunar spec l1). Earth and Moon are instances; nothing in it is
// specific to either. A small value type: it refers to the orientation, atmosphere and ephemeris it was made from.
class ReferenceBody {
public:
    // The Earth: ITRF93, WGS84, the given atmosphere; centre at the GCRF origin (no ephemeris call).
    static ReferenceBody earth(const frames::BodyOrientation& itrf, const Atmosphere& atmosphere, double gm_m3ps2);
    // The Moon: MOON_ME (the frame of lunar maps), sphere R = 1737.4 km, no atmosphere; centre from the ephemeris.
    static ReferenceBody moon(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& moon_me);

    ephemeris::Body id() const { return id_; }
    double gm_m3ps2() const { return gm_m3ps2_; }
    const frames::Ellipsoid& shape() const { return shape_; }
    bool has_atmosphere() const { return atmosphere_ != nullptr; }
    // Air at a height above the shape; vacuum_state() without an atmosphere.
    AtmosphereState air(double height_m) const {
        if (!atmosphere_) return vacuum_state();
        auto s = atmosphere_->at(height_m);
        s.density_kg_m3 *= air_density_factor_;  // truth dispersion (lunar l5 campaigns); 1 otherwise
        s.mean_free_path_m /= air_density_factor_;
        return s;  // pressure and temperature stay: the engines' back-pressure is not dispersed
    }
    void set_air_density_factor(double f) { air_density_factor_ = f; }
    math::StateVector centre_gcrf(time::Epoch epoch) const;
    // Acceleration of the body's centre in the GCRF (Earth: zero; Moon: central difference of the ephemeris velocity
    // - the DE440 Moon's own acceleration, exact to ~1e-9 m/s^2). The plan frame of a lunar landing falls with it.
    math::Vec3 centre_acceleration_gcrf(time::Epoch epoch) const;
    BodyFrame frame(time::Epoch epoch) const;

    // Terrain over the shape (lunar l2; the owner keeps it alive). nullptr: the bare shape.
    void set_terrain(const LunarTerrain* terrain) { terrain_ = terrain; }
    const LunarTerrain* terrain() const { return terrain_; }
    // Surface height above the shape below a body-fixed point (0 without terrain), and its outward unit normal
    // (without terrain: the shape's vertical).
    double surface_height_m(const math::Vec3& fixed_m) const;
    math::Vec3 surface_normal_fixed(const math::Vec3& fixed_m) const;

private:
    ReferenceBody(ephemeris::Body id, double gm_m3ps2, frames::Ellipsoid shape, const frames::BodyOrientation& orientation,
                  const Atmosphere* atmosphere, const ephemeris::Ephemeris* ephemeris)
        : id_(id), gm_m3ps2_(gm_m3ps2), shape_(shape), orientation_(&orientation), atmosphere_(atmosphere),
          ephemeris_(ephemeris) {}

    ephemeris::Body id_;
    double gm_m3ps2_;
    frames::Ellipsoid shape_;
    const frames::BodyOrientation* orientation_;
    const Atmosphere* atmosphere_;           // nullptr: vacuum
    const ephemeris::Ephemeris* ephemeris_;  // nullptr: centred at the GCRF origin (the Earth)
    double air_density_factor_ = 1.0;
    const LunarTerrain* terrain_ = nullptr;
};

}  // namespace starsim::environment
