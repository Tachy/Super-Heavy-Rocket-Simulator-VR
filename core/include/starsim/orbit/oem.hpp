#pragma once

#include <cstddef>
#include <filesystem>
#include <functional>
#include <istream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"
#include "starsim/time/time_scale_converter.hpp"

namespace starsim::orbit {

struct OemError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// One row of the OEM "TRAJECTORY EVENT SUMMARY" comment table (NASA ISS trajectory data).
struct OemEvent {
    std::string name;                 // e.g. "SPX-35 Docking", "Reboost"
    time::Epoch tig;                  // time of ignition (TDB)
    double delta_v_mps = 0.0;         // magnitude as listed
    double apoapsis_height_m = 0.0;   // HA after the event
    double periapsis_height_m = 0.0;  // HP after the event
};

struct OemHeader {
    std::string object_name;
    std::string creation_date;  // as written (UTC)
    time::Epoch usable_start;
    time::Epoch usable_stop;
    std::optional<double> mass_kg, drag_area_m2, drag_coefficient, solar_radiation_area_m2,
        solar_radiation_coefficient;
};

// CCSDS Orbit Ephemeris Message (KVN, version 2), as published by NASA for the ISS: one EME2000 / UTC / Earth
// segment of state rows. States between rows come from Lagrange interpolation inside runs of equal time steps
// ("segments"): where the output grid restarts (the step changes), a window never reaches across the seam.
class Oem {
public:
    static constexpr int interpolation_degree = 8;  // CCSDS OEM recommendation for Lagrange interpolation

    static Oem parse(std::istream& in, const time::TimeScaleConverter& time, const std::string& name = "OEM");
    static Oem load(const std::filesystem::path& file, const time::TimeScaleConverter& time);
    // For tests: a subset of rows (keep_row(i) true); the usable window is clipped to the kept rows.
    Oem filtered(const std::function<bool(size_t)>& keep_row) const;

    const OemHeader& header() const { return header_; }
    const std::vector<OemEvent>& events() const { return events_; }
    size_t row_count() const { return epochs_.size(); }
    time::Epoch row_epoch(size_t i) const { return epochs_[i]; }
    const math::StateVector& row_state(size_t i) const { return states_[i]; }
    bool covers(time::Epoch epoch) const;  // inside the usable window
    // GCRF (EME2000) state; throws OemError outside the usable window.
    math::StateVector state_gcrf(time::Epoch epoch) const;

private:
    void find_segments();

    OemHeader header_;
    std::vector<OemEvent> events_;
    std::vector<time::Epoch> epochs_;
    std::vector<math::StateVector> states_;
    // First row of each run of equal steps. The seam row also ends the previous segment (shared row).
    std::vector<size_t> segment_start_;
};

// All OEM files of a directory (the archive data/iss_oem/: one file per CREATION_DATE).
class OemArchive {
public:
    // Files that fail to parse are skipped and listed in skipped(); a missing directory gives an empty archive.
    static OemArchive scan(const std::filesystem::path& dir, const time::TimeScaleConverter& time);
    // The newest file (by CREATION_DATE) whose usable window contains `epoch`, or nullptr.
    const Oem* for_epoch(time::Epoch epoch) const;
    // The newest file of all (for the synthetic orbit's mean elements), or nullptr.
    const Oem* latest() const;
    size_t size() const { return files_.size(); }
    const std::vector<std::string>& skipped() const { return skipped_; }

private:
    std::vector<Oem> files_;  // sorted by creation date, oldest first
    std::vector<std::string> skipped_;
};

}  // namespace starsim::orbit
