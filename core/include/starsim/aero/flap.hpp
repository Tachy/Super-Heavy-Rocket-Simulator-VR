#pragma once

#include <optional>
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

// Flat plate hinged along a line at the hull (hinge azimuth from body +y towards +z), e.g. a Starship flap. The hinge
// starts at (station_aft_m, radius hinge_radius_aft_m) and leans towards the axis going forward by hinge_lean_rad, in
// the meridian plane: h = cos(l) e_x - sin(l) e_r, its perpendicular in that plane w = sin(l) e_x + cos(l) e_r (l = 0:
// e_x and e_r). u_stow: hull tangent at the hinge pointing away from the belly (leeward). The plate spans h and
// s(d) = cos(d) u_stow + sin(d) w: d = 0 folded leeward (edge-on to a belly-first flow), d = 90 deg in the meridian
// plane (broadside, edge-on to the axial flow at any lean); normal n = h x s.
struct FlapGeometry {
    double station_fwd_m = 0.0;  // body frame
    double station_aft_m = 0.0;
    double hinge_azimuth_rad = 0.0;
    double hull_radius_m = 0.0;
    double span_m = 0.0;
    double area_m2 = 0.0;
    double belly_azimuth_rad = 0.0;  // body direction of the heat shield (windward side in entry)
    double hinge_lean_rad = 0.0;
    std::optional<double> hinge_radius_aft_m;  // default hull_radius_m
    std::optional<double> centroid_u_m;        // planform centroid along h from the aft end; default mid-hinge
    std::optional<double> centroid_v_m;        // planform centroid along s; default mid-span
};

math::Vec3 flap_hinge_direction(const FlapGeometry& flap);  // h, towards the forward end
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
