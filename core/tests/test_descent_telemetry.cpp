// Tail-first descent of the booster against our webcast telemetry tables (tests/validation/telemetry): the booster
// of the configuration that flew (block 1 on flight 5, block 2 on flights 7 and 8) is placed unguided in each flight's state at its peak descent speed (~26 km) and must follow the
// recorded speed until the landing burn.

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
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
    double speed_mps = -1.0;    // < 0: no reading
    double altitude_m = -1.0;
};

std::vector<Sample> load_booster(const std::string& file) {
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
        f.resize(5);
        Sample s;
        s.t_s = std::stod(f[0]);
        if (!f[1].empty()) s.speed_mps = std::stod(f[1]) / 3.6;
        if (!f[2].empty()) s.altitude_m = std::stod(f[2]) * 1e3;
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

TEST_CASE("Booster descent drag reproduces the flight 5/7/8 telemetry", "[gnc][booster][validation]") {
    // Table and the vehicle configuration that flew (tests/validation/telemetry/flights.toml).
    const auto [file, vehicle_id] = GENERATE(table<std::string, std::string>({
        {"flight5_2024-10-13.csv", "starship_block1"},
        {"flight7_2025-01-16.csv", "starship_block2"},
        {"flight8_2025-03-06.csv", "starship_block2"},
    }));
    CAPTURE(file, vehicle_id);
    const auto data = load_booster(file);
    REQUIRE(data.size() > 500);
    // Peak descent speed (after the boostback) and the landing-burn ignition (speed falling > 120 km/h per s).
    const Sample* peak = nullptr;
    for (const auto& s : data) {
        if (s.t_s > 300.0 && s.speed_mps > 0.0 && (!peak || s.speed_mps > peak->speed_mps)) peak = &s;
    }
    REQUIRE(peak != nullptr);
    double t_ignition = 0.0;
    const Sample* last = nullptr;
    for (const auto& s : data) {
        if (s.t_s <= peak->t_s || s.speed_mps < 0.0) continue;
        if (last && s.speed_mps < 1500.0 / 3.6 && (last->speed_mps - s.speed_mps) / (s.t_s - last->t_s) > 120.0 / 3.6) {
            t_ignition = last->t_s;
            break;
        }
        last = &s;
    }
    REQUIRE(t_ignition > peak->t_s);
    const auto [h0, vz0] = altitude_fit(data, peak->t_s, 6.0);
    const double vh0 = std::sqrt(std::max(peak->speed_mps * peak->speed_mps - vz0 * vz0, 0.0));

    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    const double landing_propellant_kg = 72.0e3;  // as our block 1/2 returns at ignition
    session.place_body(test::booster_config(vehicle_id, landing_propellant_kg),
                       test::state_over_site(session.launch_site(), earth, t0, {20.0e3, 0.0, h0}, {-vh0, 0.0, vz0}), t0);

    double max_error = 0.0, sum_sq = 0.0;
    int n = 0;
    for (const auto& s : data) {
        if (s.t_s <= peak->t_s + 1.0 || s.t_s > t_ignition - 2.0 || s.speed_mps < 0.0) continue;
        session.advance_to(t0 + (s.t_s - peak->t_s), 1000);
        const auto tr = earth.fixed_to_gcrf(session.epoch());
        const auto& st = session.body(0).state();
        const double speed = norm(st.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), st.position_gcrf_m));
        const double error = speed / s.speed_mps - 1.0;
        max_error = std::max(max_error, std::abs(error));
        sum_sq += error * error;
        ++n;
    }
    const double rms = std::sqrt(sum_sq / n);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("%s: start T+%.1f s at %.1f km, %.0f m/s (vertical %.0f); %d samples to T+%.1f s: rms %.1f %%, "
                    "max %.1f %%\n", file.c_str(), peak->t_s, h0 / 1e3, peak->speed_mps, vz0, n, t_ignition - 2.0,
                    100.0 * rms, 100.0 * max_error);
    }
    CHECK(n > 40);
    CHECK(rms < 0.03);
    CHECK(max_error < 0.06);
}
