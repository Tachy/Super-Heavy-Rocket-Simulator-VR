#pragma once

#include <filesystem>
#include <vector>

#include "starsim/aero/slender_body_aero.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::aero {

// Blunt capsule (Crew Dragon), axisymmetric, heat shield first. Body x runs from the shield apex (station
// apex_station_m) towards the nose. u: velocity relative to the air (body axes); theta: angle between u and -x;
// rho = unit radial part of u (+y when u is axial), the windward side. Coefficients from a (mach, theta) table
// (tools/aero/gen_capsule_aero.py, modified Newtonian [D] / [E] below Mach 2), reference area pi D^2 / 4:
//   F = q S (C_A x - C_N rho),   M_apex = q S D C_m (x cross rho).
// No rarefied bridging (the highest Mach row is used in thin air) and no damping derivatives (d3 plan rulings 1-3).
struct CapsuleAeroTable {
    std::vector<double> mach;       // ascending
    std::vector<double> theta_rad;  // ascending
    std::vector<std::vector<double>> ca, cn, cm;  // [mach][theta]
};

// CSV columns mach, alpha_deg (theta), ca, cn, cm; rows grouped by Mach, the same theta grid in each group.
CapsuleAeroTable load_capsule_aero_table(const std::filesystem::path& csv);

struct CapsuleTrim {
    bool found = false;
    double theta_rad = 0.0;
    double cd = 0.0;  // along -u
    double cl = 0.0;  // perpendicular to u
    double lift_to_drag = 0.0;
    math::Vec3 air_velocity_body_unit;  // u at trim (in the plane of the CG's radial offset)
    math::Vec3 lift_body_unit;          // lift direction at trim, body axes
};

class CapsuleAero {
public:
    struct Coefficients {
        double ca = 0.0, cn = 0.0, cm = 0.0;
    };
    CapsuleAero(CapsuleAeroTable table, double reference_diameter_m, double apex_station_m);

    // AeroResult::normal_force_station_m = apex station; moment_body_nm = M_apex.
    AeroResult compute(const math::Vec3& air_velocity_body_mps, double density_kg_m3, double speed_of_sound_mps) const;
    Coefficients coefficients(double mach, double theta_rad) const;  // bilinear, clamped at the grid ends
    // Pitching moment coefficient about a CG (body point: station from the vehicle bottom, y, z) in the plane of the CG's
    // radial offset (+y without an offset); > 0 turns theta up. The trim: theta in [0, 60 deg] where it crosses zero
    // with a negative slope.
    double pitch_moment_about(double mach, double theta_rad, const math::Vec3& cg_body_m) const;
    CapsuleTrim trim(double mach, const math::Vec3& cg_body_m) const;
    double reference_area_m2() const { return reference_area_m2_; }
    double reference_diameter_m() const { return diameter_m_; }
    double apex_station_m() const { return apex_station_m_; }
    const CapsuleAeroTable& table() const { return table_; }

private:
    CapsuleAeroTable table_;
    double diameter_m_;
    double apex_station_m_;
    double reference_area_m2_;
};

}  // namespace starsim::aero
