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

    double cd_area_m2(double mach) const;
    double cl_area_m2(double mach) const;
    double alpha_rad(double mach) const { return schedule_.at(mach); }
    bool empty() const { return mach_.empty(); }

private:
    AlphaSchedule schedule_;
    std::vector<double> mach_;
    std::vector<double> cd_area_m2_;
    std::vector<double> cl_area_m2_;
};

}  // namespace starsim::gnc
