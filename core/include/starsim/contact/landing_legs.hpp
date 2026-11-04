#pragma once

#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/contact/pin_contact.hpp"

namespace starsim::contact {

// Contact points of a stage with deployed landing legs, vehicle frame (station, y, z).
struct LegGeometry {
    std::vector<math::Vec3> feet;       // deployed feet
    std::vector<math::Vec3> base_ring;  // the stage's bottom edge (engine section): must not touch the surface
    config::LandingLegConfig config;
};

// Empty when the stage has no legs.
LegGeometry make_leg_geometry(const config::LandingLegConfig& legs, double stage_base_station_m, double hull_radius_m);

struct LegContact {
    math::Vec3 force_n;      // on the body, site frame
    math::Vec3 torque_nm;    // about the CG, site frame
    int feet_in_contact = 0;
    bool water = false;            // a foot or the base below the surface over water
    bool stroke_exhausted = false; // a crush core used up: the leg fails
    bool base_contact = false;     // the base ring below the surface
    double max_foot_force_n = 0.0;
    // Per foot (one entry per foot, 0 where it does not touch): how far the foot is pushed in along the leg's stroke axis,
    // the stage's station axis (the feet only move along it, towards the hull): the penetration along the surface normal
    // over the cosine between the normal and the station axis. A drawn foot rises by it, so it stands on the surface.
    // Read only (the models); nothing in the force depends on it.
    std::vector<double> foot_compression_m;
};

// The foot compression for a penetration depth along the surface normal `n` (site frame) and the stage's station axis
// `axis` (site frame): depth / (n . axis); 0 for depth <= 0 or a surface almost parallel to the axis (cosine < 0.1).
double foot_compression_along_axis_m(double depth_m, const math::Vec3& n, const math::Vec3& axis);

// The body's inertia for friction that stays stable with explicit (start-of-tick) forces: a foot's friction is capped
// at what stops its slip within the tick (effective mass at the foot incl. rotation, shared by the feet in contact).
// Without it, a stage with a small roll inertia (Falcon 9: ~45 t m^2, feet 9 m out) reversed its spin every tick.
struct FrictionLimit {
    double mass_kg = 0.0;
    math::Vec3 inertia_body_kg_m2;  // principal moments (roll, pitch, yaw)
    double dt_s = 0.0;
};

// Penalty contact of the feet with the surface. Each foot is a spring-damper (stiffness, damping) in series with a
// crush core: once the force reaches the crush force the core crushes plastically (constant force, the stroke used so
// far in `crushed_m`, one per foot, grows; no rebound). Regularised Coulomb friction against the slip. A foot over
// water gets no support (it sinks: `water` once below the sea). The base ring only reports contact (no force).
LegContact leg_contact(const LegGeometry& legs, std::vector<double>& crushed_m, const SitePose& pose,
                       const LandingSurface& surface, const FrictionLimit* friction_limit = nullptr);

}  // namespace starsim::contact
