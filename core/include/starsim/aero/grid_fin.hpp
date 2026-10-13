#pragma once

#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::aero {

// Normal-force (and axial-force) characteristic of a grid (lattice) fin versus Mach number: slope of the normal-force
// coefficient (per rad, on the fin's frontal lattice area) and the effective angle beyond which the
// force no longer grows (cell flow separation). Transonic "choking" of the cells shows as a dip of the
// slope. Linear interpolation, clamped at the table ends.
class GridFinTable {
public:
    GridFinTable() = default;
    // columns: mach, cn_alpha_per_rad, limit_deg, optional ca_axial
    explicit GridFinTable(const config::CsvTable& table);

    double normal_force_slope_per_rad(double mach) const;
    double limit_rad(double mach) const;
    // Axial force coefficient on the frontal lattice area (0 without the column); it is added to the vehicle's
    // tail-first drag curve (Vehicle), not to the fin force.
    double axial_coefficient(double mach) const;
    const std::vector<double>& mach() const { return mach_; }
    const std::vector<double>& axial_coefficients() const { return axial_; }
    bool empty() const { return mach_.empty(); }

private:
    std::vector<double> mach_;
    std::vector<double> slope_;
    std::vector<double> limit_rad_;
    std::vector<double> axial_;  // empty: no axial data
};

// One fin in body axes (x along the vehicle axis towards the nose, see vehicle.hpp). At zero deflection
// the fin plane contains the body axis and the radial direction e_r; its normal is the tangential
// direction t = e_x x e_r = (0, -sin a, cos a) for the azimuth a from +y towards +z. A deflection d
// rotates the fin about e_r: n = t cos d + e_x sin d.
struct GridFinGeometry {
    double station_m = 0.0;    // hinge station (vehicle body frame)
    double azimuth_rad = 0.0;  // from body +y towards +z
    double radius_m = 0.0;     // radius of the fin's centre of pressure
    double area_m2 = 0.0;      // frontal lattice area
};

struct GridFinForce {
    math::Vec3 force_body_n;
    math::Vec3 application_point_m;  // body frame (station, y, z)
};

// Flow state at the vehicle, enough for the fin model.
struct FinAirData {
    double density_kg_m3 = 0.0;
    double speed_of_sound_mps = 0.0;
    double rarefaction = 0.0;  // 0 continuum ... 1 free-molecular (fins ineffective)
};

// Force of one fin. `fin_velocity_body_mps`: velocity of the fin relative to the air (vehicle air
// velocity + omega x r). Flat-plate style: the force acts along the fin normal and opposes the normal
// component of the motion; |C_N| = slope * sin(min(|alpha_eff|, limit)).
GridFinForce grid_fin_force(const GridFinGeometry& fin, double deflection_rad, const math::Vec3& fin_velocity_body_mps,
                            const FinAirData& air, const GridFinTable& table);

// Tangential force per radian of deflection for small deflections (allocation): dF_t/dd.
double grid_fin_effectiveness(const GridFinGeometry& fin, const math::Vec3& fin_velocity_body_mps,
                              const FinAirData& air, const GridFinTable& table);

math::Vec3 grid_fin_radial(const GridFinGeometry& fin);
math::Vec3 grid_fin_tangential(const GridFinGeometry& fin);

}  // namespace starsim::aero
