#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "starsim/vehicle/vehicle.hpp"

namespace starsim::gnc {

// Nominal angle of attack of the belly-first entry over Mach (linear, clamped at the ends).
struct AlphaSchedule {
    std::vector<double> mach;
    std::vector<double> alpha_rad;
    double at(double mach_number) const;
    bool empty() const { return mach.empty(); }
};

// CSV with columns mach, alpha_deg (path relative to config/).
AlphaSchedule load_alpha_schedule(const std::filesystem::path& config_dir, const std::string& table);

// Aerodynamic coefficients of the ship for the entry guidance's point-mass prediction: drag and lift area
// (force / dynamic pressure) over Mach at the scheduled angle of attack, hull plus flaps in pitch trim about the
// current CG. Continuum flow (the rarefied upper entry carries little force; the guidance's online scale factors
// absorb the difference). Built on the simulation thread; afterwards immutable and safe to copy to a worker.
class EntryAeroTable {
public:
    EntryAeroTable() = default;
    // `ship`: a single-stage vehicle with flaps (its first stage carries them).
    EntryAeroTable(const vehicle::Vehicle& ship, const AlphaSchedule& schedule);
    // A capsule (Vehicle::capsule_aero): drag and lift area at its trim over Mach about the current CG; the alpha
    // schedule is pi - theta_trim (the angle between the nose and the air velocity). It has one angle of attack, so the
    // alpha-grid accessors return the trim's values at any alpha (the capsule flies no terminal phase).
    static EntryAeroTable capsule(const vehicle::Vehicle& capsule);

    double cd_area_m2(double mach) const;
    double cl_area_m2(double mach) const;
    double alpha_rad(double mach) const { return schedule_.at(mach); }
    bool empty() const { return mach_.empty(); }

    // Drag and lift area at any angle of attack (40..140 deg, flaps in trim; bilinear over Mach and alpha): the
    // terminal phase steers by the lift's direction about the air velocity and by alpha.
    double cd_area_m2(double mach, double alpha_rad) const;
    double cl_area_m2(double mach, double alpha_rad) const;

    // Flap deflections (the ship's first-stage flaps in order) of the pitch trim at the scheduled angle of attack;
    // empty for a capsule.
    std::vector<double> trim_flap_rad(double mach) const;

private:
    AlphaSchedule schedule_;
    std::vector<double> mach_;
    std::vector<double> cd_area_m2_;
    std::vector<double> cl_area_m2_;
    std::vector<double> alpha_grid_rad_;
    std::vector<std::vector<double>> cd_grid_m2_;  // [mach][alpha]
    std::vector<std::vector<double>> cl_grid_m2_;
    std::vector<double> trim_delta_rad_;  // per mach_: forward pair at mid + delta, aft pair at mid - delta
    std::vector<bool> flap_forward_;
    std::vector<double> flap_max_rad_;
    double bilinear(const std::vector<std::vector<double>>& grid, double mach, double alpha_rad) const;
};

}  // namespace starsim::gnc
