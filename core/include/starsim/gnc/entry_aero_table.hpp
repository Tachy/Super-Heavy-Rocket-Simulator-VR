#pragma once

#include <filesystem>
#include <limits>
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
// The trim is the one the flap allocation settles in (AttitudeController: pitch cancelled, then the moment-neutral
// drift towards mid-range): of all pitch trims of the two pairs the one closest to mid-range, where each pair's offset
// from mid goes with its own pitch slope. (Before planner plan Task 4 the pairs moved by equal and opposite offsets: the
// forward pair 14-33 deg where the flown flaps sat at 41-44, 0.4-0.5 % too little drag below Mach 3.)
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
    // The same at any angle of attack of the grid (bilinear).
    std::vector<double> trim_flap_rad(double mach, double alpha_rad) const;
    // The lowest angle of attack the flaps can trim in pitch below the schedule at this Mach (most lift the ship can hold;
    // 0: no flaps, no such bound).
    double lowest_trimmed_alpha_rad(double mach) const;
    // How far the flaps' pitch trim along the schedule stays inside their stops (planner plan Task 12b): over the Mach
    // grid, the smallest distance of any flap of the trim the table holds (the settled one, else the antisymmetric) from
    // its stops - the upper `max_deflection_rad` and the lower the lowest usable deflection in that flow (edge-on /
    // the hull's shadow, aero::flap_lowest_usable_deflection_rad). <= 0: somewhere the flaps alone cannot hold the
    // schedule in pitch. +infinity without flaps (a capsule).
    double schedule_trim_margin_rad() const { return schedule_trim_margin_rad_; }

private:
    double schedule_trim_margin_rad_ = std::numeric_limits<double>::infinity();
    std::vector<double> lowest_trimmed_alpha_rad_;  // per mach_
    AlphaSchedule schedule_;
    std::vector<double> mach_;
    std::vector<double> cd_area_m2_;
    std::vector<double> cl_area_m2_;
    std::vector<double> alpha_grid_rad_;
    std::vector<std::vector<double>> cd_grid_m2_;  // [mach][alpha]
    std::vector<std::vector<double>> cl_grid_m2_;
    // Trim offsets from mid-range of the aft and the forward pair: [mach][alpha] and per mach_ (schedule)
    std::vector<std::vector<double>> trim_aft_grid_rad_, trim_forward_grid_rad_;
    std::vector<double> trim_aft_rad_, trim_forward_rad_;
    std::vector<bool> flap_forward_;
    std::vector<double> flap_max_rad_;
    double bilinear(const std::vector<std::vector<double>>& grid, double mach, double alpha_rad) const;
};

}  // namespace starsim::gnc
