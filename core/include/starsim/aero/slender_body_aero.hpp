#pragma once

#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::aero {

// Axial drag coefficient versus Mach number (linear interpolation, clamped at the table ends).
class AxialDragCurve {
public:
    AxialDragCurve() = default;
    explicit AxialDragCurve(const config::CsvTable& table);  // columns: mach, cd_axial_power_on, cd_axial_power_off

    double cd(double mach, bool powered) const;
    bool empty() const { return mach_.empty(); }

private:
    std::vector<double> mach_;
    std::vector<double> cd_power_on_;
    std::vector<double> cd_power_off_;
};

struct SlenderBodyGeometry {
    double length_m = 0.0;
    double diameter_m = 0.0;
    double nose_length_m = 0.0;  // ogive nose at the top; 0 = flat top
};

struct AeroResult {
    math::Vec3 force_body_n;       // total aerodynamic force, body axes (x = towards the nose)
    double normal_force_station_m = 0.0;  // where the normal force acts (from the bottom of the body)
    double mach = 0.0;
    double dynamic_pressure_pa = 0.0;
    double angle_of_attack_rad = 0.0;  // total angle between body axis and the oncoming flow
    double knudsen = 0.0;              // mean free path / body length
    double rarefaction = 0.0;          // 0 = continuum, 1 = free-molecular flow
};

// Bridging weight continuum -> free-molecular flow over 0.01 < Kn < 10 (sine-squared in log Kn).
double rarefaction_weight(double knudsen);

// Axisymmetric slender body:
//   axial force  C_A = C_A0(M) cos^2(alpha)                      (table, power on / off; a separate
//                                                                 table for tail-first flight if given)
//   normal force C_N = sin(2a) cos(a/2) + eta * C_dc * (L D / A_ref) sin^2(a)   (Jorgensen, NASA TR R-474)
// The potential term acts at 0.466 of the nose length from the tip (ogive, Barrowman), the cross-flow
// term at mid-length. [E] eta = 0.68, C_dc = 1.2 - generic, to be calibrated (no public Starship data).
// Rarefied flow (above ~90 km): bridged by Knudsen number to free-molecular coefficients
// (C_A = 2.2 cos^2 a, cross-flow C_dc = 2.2 with eta = 1 [E], typical diffuse-reflection values).
// No damping derivatives yet.
class SlenderBodyAero {
public:
    SlenderBodyAero(SlenderBodyGeometry geometry, AxialDragCurve drag, AxialDragCurve tail_first_drag = {});

    // air_velocity_body_mps: velocity of the vehicle relative to the air, body axes.
    AeroResult compute(const math::Vec3& air_velocity_body_mps, double density_kg_m3, double speed_of_sound_mps,
                       bool powered, double mean_free_path_m = 0.0) const;

    const SlenderBodyGeometry& geometry() const { return geometry_; }
    const AxialDragCurve& drag_curve() const { return drag_; }
    // Table for the given flight direction (tail first falls back to the nose-first table).
    const AxialDragCurve& drag_curve(bool tail_first) const {
        return tail_first && !tail_first_drag_.empty() ? tail_first_drag_ : drag_;
    }

private:
    SlenderBodyGeometry geometry_;
    AxialDragCurve drag_;
    AxialDragCurve tail_first_drag_;
    double reference_area_m2_;
};

}  // namespace starsim::aero
