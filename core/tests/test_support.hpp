#pragma once

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_session.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::test {

// One SPICE session shared by all tests of the executable (CSPICE is process-global).
inline spice::SpiceSession& spice_session() {
    static spice::SpiceSession session = spice::SpiceSession::open_default(STARSIM_KERNEL_DIR);
    return session;
}

inline time::Epoch epoch_from_jd_tdb(double jd_tdb) {
    return time::Epoch::from_tdb_seconds_past_j2000((jd_tdb - constants::j2000_julian_date) *
                                                    constants::seconds_per_day);
}

// Reads a numeric CSV fixture (first line = header). Returns rows of doubles.
inline std::vector<std::vector<double>> read_fixture(const std::string& name) {
    std::ifstream file(std::string(STARSIM_FIXTURE_DIR) + "/" + name);
    if (!file) {
        throw std::runtime_error("missing fixture " + name + " (run tools/validation/fetch_horizons.py)");
    }
    std::vector<std::vector<double>> rows;
    std::string line;
    std::getline(file, line);  // header
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream stream(line);
        std::string cell;
        while (std::getline(stream, cell, ',')) {
            row.push_back(std::stod(cell));
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

}  // namespace starsim::test
