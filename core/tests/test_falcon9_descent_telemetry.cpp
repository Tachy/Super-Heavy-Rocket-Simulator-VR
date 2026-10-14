// Tail-first descent of the Falcon 9 first stage against our webcast telemetry (milestone c1): the stage is placed
// unguided in each flight's state after its entry burn and must follow the recorded speed until the landing burn.

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

struct Sample {
    double t_s = 0.0;
    double speed_mps = -1.0;  // < 0: no reading
    double altitude_m = -1.0;
    int engines_lit = -1;     // first stage's engine icon (entry and landing burns), < 0: not shown
};

std::vector<Sample> load_stage1(const std::string& file) {
    std::ifstream in(std::string(STARSIM_TELEMETRY_DIR) + "/" + file);
    std::vector<Sample> out;
    std::string line;
    bool header = true;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (header) {
            header = false;
            continue;
        }
        std::vector<std::string> f;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) f.push_back(cell);
        f.resize(10);
        Sample s;
        s.t_s = std::stod(f[0]);
        if (!f[1].empty()) s.speed_mps = std::stod(f[1]) / 3.6;
        if (!f[2].empty()) s.altitude_m = std::stod(f[2]) * 1e3;
        if (!f[6].empty()) s.engines_lit = std::stoi(f[6]);
        out.push_back(s);
    }
    return out;
}

// Least-squares line through the altitude readings within +-half of t: value and slope.
std::pair<double, double> altitude_fit(const std::vector<Sample>& d, double t, double half) {
    double n = 0, st = 0, sh = 0, stt = 0, sth = 0;
    for (const auto& s : d) {
        if (s.altitude_m < 0.0 || std::abs(s.t_s - t) > half) continue;
        n += 1;
        st += s.t_s;
        sh += s.altitude_m;
        stt += s.t_s * s.t_s;
        sth += s.t_s * s.altitude_m;
    }
    const double slope = (n * sth - st * sh) / (n * stt - st * st);
    return {(sh - slope * st) / n + slope * t, slope};
}

}  // namespace

TEST_CASE("Falcon 9 descent drag reproduces the webcast telemetry of CRS-35 and Starlink 15-25",
          "[gnc][falcon9][validation]") {
    // Propellant in the descent from the stage's gauge after the entry burn (% of 425 t); horizontal direction of
    // flight (east, north): CRS-35 flies back west to LZ-40, Starlink 15-25 south along the Californian coast.
    const auto [file, propellant_kg, heading_east, heading_north] = GENERATE(table<std::string, double, double, double>({
        {"f9_crs35_2026-10-13.csv", 0.030 * 425.0e3, -1.0, 0.0},
        {"f9_starlink15-25_2026-10-12.csv", 0.039 * 425.0e3, 0.0, -1.0},
    }));
    CAPTURE(file);
    const auto data = load_stage1(file);
    REQUIRE(data.size() > 500);
    // Entry burn: the engine icon replaces the speed dial; the comparison starts at the first speed reading after
    // it and ends at the last one before the landing burn's icon.
    double entry_end = 0.0, landing_start = 0.0;
    for (const auto& s : data) {
        if (s.t_s < 300.0) continue;
        if (s.engines_lit > 0 && landing_start == 0.0) {
            if (entry_end == 0.0 || s.t_s - entry_end < 20.0) entry_end = s.t_s;
            else landing_start = s.t_s;
        }
    }
    REQUIRE(entry_end > 300.0);
    REQUIRE(landing_start > entry_end);
    const Sample* start = nullptr;
    const Sample* last = nullptr;
    for (const auto& s : data) {
        if (s.t_s <= entry_end + 2.0 || s.t_s >= landing_start || s.speed_mps < 0.0) continue;
        if (!start) start = &s;
        last = &s;
    }
    REQUIRE(start != nullptr);
    const auto [h0, vz0] = altitude_fit(data, start->t_s, 6.0);
    const double vh0 = std::sqrt(std::max(start->speed_mps * start->speed_mps - vz0 * vz0, 0.0));

    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 10, 33, 0.0});
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_lz40")));
    session.place_body(test::booster_config("falcon9_block5", propellant_kg),
                       test::state_over_site(session.launch_site(), earth, t0, {0.0, 0.0, h0},
                                             {vh0 * heading_east, vh0 * heading_north, vz0}),
                       t0);

    double max_error = 0.0, sum_sq = 0.0;
    int n = 0;
    for (const auto& s : data) {
        if (s.t_s <= start->t_s + 1.0 || s.t_s > last->t_s || s.speed_mps < 0.0) continue;
        session.advance_to(t0 + (s.t_s - start->t_s), 1000);
        const auto tr = earth.fixed_to_gcrf(session.epoch());
        const auto& st = session.body(0).state();
        const double speed = norm(st.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), st.position_gcrf_m));
        const double error = speed / s.speed_mps - 1.0;
        if (std::getenv("STARSIM_VERBOSE") && n % 8 == 0) {
            const double height = frames::fixed_to_geodetic(transpose(tr.rotation) * st.position_gcrf_m).height_m;
            std::printf("  T+%.1f s  recorded %.0f m/s at %.1f km  simulated %.0f m/s at %.1f km\n", s.t_s, s.speed_mps,
                        s.altitude_m / 1e3, speed, height / 1e3);
        }
        max_error = std::max(max_error, std::abs(error));
        sum_sq += error * error;
        ++n;
    }
    // Below the last speed reading the 0.1 km altitude continues to the landing burn's first lit icon; compared as
    // the altitude lost since that reading (the webcast's speed and altitude graphics may be offset in time).
    double max_altitude_error_m = 0.0, offset_m = 0.0;
    int n_altitude = 0;
    for (const auto& s : data) {
        if (s.t_s < last->t_s || s.t_s > landing_start || s.altitude_m < 0.0) continue;
        session.advance_to(t0 + (s.t_s - start->t_s), 1000);
        const auto tr = earth.fixed_to_gcrf(session.epoch());
        const double height =
            frames::fixed_to_geodetic(transpose(tr.rotation) * session.body(0).state().position_gcrf_m).height_m -
            session.launch_site().height_m;
        if (n_altitude == 0) offset_m = height - s.altitude_m;
        max_altitude_error_m = std::max(max_altitude_error_m, std::abs(height - s.altitude_m - offset_m));
        ++n_altitude;
    }
    const double rms = std::sqrt(sum_sq / n);
    std::printf("[validation] %s: from T+%.1f s at %.1f km, %.0f m/s (vertical %.0f); %d speed samples to T+%.1f s: "
                "rms %.1f %%, max %.1f %%; altitude lost to the landing burn (T+%.1f s): max error %.0f m (%d samples)\n",
                file.c_str(), start->t_s, h0 / 1e3, start->speed_mps, vz0, n, last->t_s, 100.0 * rms, 100.0 * max_error,
                landing_start, max_altitude_error_m, n_altitude);
    CHECK(n > 30);
    CHECK(rms < 0.03);
    CHECK(max_error < 0.06);
    CHECK(max_altitude_error_m < 300.0);
}
