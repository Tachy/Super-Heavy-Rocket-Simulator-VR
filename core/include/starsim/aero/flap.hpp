#pragma once

#include <vector>

#include "starsim/aero/grid_fin.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::aero {

// Maximum normal-force coefficient of a flap (flat plate broadside to the flow) versus Mach, reference = flap
// area. Linear interpolation, clamped at the table ends.
class FlapTable {
public:
    FlapTable() = default;
    explicit FlapTable(const config::CsvTable& table);  // columns: mach, cn_max

    double cn_max(double mach) const;
    bool empty() const { return mach_.empty(); }

private:
    std::vector<double> mach_;
    std::vector<double> cn_max_;
};

// Flat plate hinged along a line parallel to the body axis at the hull (radius hull_radius_m, hinge azimuth from
// body +y towards +z), e.g. a Starship flap. u_stow: hull tangent at the hinge pointing away from the belly
// (leeward), e_r: radial. The plate spans e_x and s(d) = cos(d) u_stow + sin(d) e_r: d = 0 folded leeward
// (edge-on to a belly-first flow), d = 90 deg radial (broadside); normal n = e_x x s.
struct FlapGeometry {
    double station_fwd_m = 0.0;  // body frame
    double station_aft_m = 0.0;
    double hinge_azimuth_rad = 0.0;
    double hull_radius_m = 0.0;
    double span_m = 0.0;
    double area_m2 = 0.0;
    double belly_azimuth_rad = 0.0;  // body direction of the heat shield (windward side in entry)
};

math::Vec3 flap_span_direction(const FlapGeometry& flap, double deflection_rad);
math::Vec3 flap_normal(const FlapGeometry& flap, double deflection_rad);
math::Vec3 flap_centroid(const FlapGeometry& flap, double deflection_rad);  // body frame (station, y, z)

struct FlapForce {
    math::Vec3 force_body_n;
    math::Vec3 application_point_m;  // body frame
};

// Force of one flap. `velocity_body_mps`: flap centroid relative to the air (vehicle air velocity + omega x r).
// Newtonian flat plate: |C_N| = cn_max(M) sin^2(theta), theta = incidence of the flow on the plate, along the
// normal against the normal motion; continuum and free-molecular (2.2 sin^2, diffuse [E]) bridged by the
// rarefaction weight. Only the windward side carries pressure: a flap whose centroid does not move towards the
// belly lies in the hull's lee and gets no force.
FlapForce flap_force(const FlapGeometry& flap, double deflection_rad, const math::Vec3& velocity_body_mps,
                     const FinAirData& air, const FlapTable& table);

}  // namespace starsim::aero
