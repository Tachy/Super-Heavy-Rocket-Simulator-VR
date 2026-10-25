#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "starsim/environment/atmosphere.hpp"

namespace starsim::environment {

// Refractive index profile of a spherically symmetric atmosphere: n - 1 proportional to the density (Gladstone-Dale,
// the standard-air refractivity at 550 nm), tabulated over the altitude. One wavelength: no dispersion (green flash).
class RefractionProfile {
public:
    // Samples `atmosphere` every `step_m` from 0 to `top_altitude_m`; above that the index is 1.
    explicit RefractionProfile(const Atmosphere& atmosphere, double top_altitude_m = 120.0e3, double step_m = 25.0);

    double refractivity(double altitude_m) const;           // n - 1
    double refractivity_gradient_per_m(double altitude_m) const;  // d(n - 1) / d altitude
    double top_altitude_m() const { return top_altitude_m_; }

private:
    double top_altitude_m_;
    double step_m_;
    std::vector<double> refractivity_;
};

// Astronomical refraction: the angle (rad, >= 0) by which a source at infinity appears raised for an observer
// `observer_altitude_m` above a sphere of `surface_radius_m` looking at `apparent_elevation_rad` (above the local
// horizontal). The ray is followed through the whole atmosphere, also through a tangent point below the observer
// (an observer in orbit looking at the limb). Bouguer invariant n r sin z = p and the nonsingular zenith-angle integral
// R = -int r n' / (n + r n') dz (Auer & Standish 2000, AJ 119, 2472), Simpson's rule. nullopt: the ray meets
// the surface.
std::optional<double> refraction_bending_rad(const RefractionProfile& profile, double surface_radius_m,
                                             double observer_altitude_m, double apparent_elevation_rad);

// The lift (apparent minus true elevation) over the TRUE elevation for one observer, what a renderer applies to a
// source's geometric direction. Grid: true elevation min + span (i / (n - 1))^2, dense near the horizon / limb.
// Below min the lift of min holds (the part is behind the surface); above min + span it is 0 (rays above the
// atmosphere's top for an observer above it). Where the bending grows faster than the elevation (an observer in orbit,
// rays grazing the lowest ~12 km of the limb) the image would fold over: the table keeps the upper branch, the true
// elevations of the fold are below min. Empty (no lift) when the limb rays would bend past the nadir (a distant
// observer, beyond ~330 000 km).
struct RefractionLiftTable {
    double true_elevation_min_rad = 0.0;
    double true_elevation_span_rad = 0.0;
    std::vector<double> lift_rad;

    double lift_at(double true_elevation_rad) const;
};

RefractionLiftTable refraction_lift_table(const RefractionProfile& profile, double surface_radius_m,
                                          double observer_altitude_m, std::size_t samples = 64);

}  // namespace starsim::environment
