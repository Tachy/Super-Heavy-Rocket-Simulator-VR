// Ship entry against the webcast telemetry of flights 12-14 (tests/validation/telemetry). The real bank history is
// unknown, so the replay steers the bank to follow the recorded altitude over time (vertical lift fraction from an
// altitude / altitude-rate feedback); the drag model then decides how fast the ship slows down. Checked: the speed
// over time (drag, i.e. the hull cross-flow and flap tables) and that the model's lift suffices to fly the recorded
// path (vertical lift fraction rarely saturated); in the belly flop the speed over altitude (terminal speed).

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ship_entry_scenario.hpp"
#include "starsim/gnc/entry_aero_table.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

struct Reading {
    double t_s = 0.0;
    double speed_mps = -1.0;  // < 0: no reading
    double altitude_m = -1.0;
};

std::vector<Reading> load_ship(const std::string& file) {
    std::ifstream in(std::string(STARSIM_TELEMETRY_DIR) + "/" + file);
    std::vector<Reading> out;
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
        f.resize(9);
        Reading r;
        r.t_s = std::stod(f[0]);
        if (!f[3].empty()) r.speed_mps = std::stod(f[3]) / 3.6;
        if (!f[4].empty()) r.altitude_m = std::stod(f[4]) * 1e3;
        // OCR error: during camera cuts a stray '7' / '1' km/h is read; the ship is that slow only near the ground.
        if (r.speed_mps >= 0.0 && r.speed_mps < 100.0 && !(r.altitude_m >= 0.0 && r.altitude_m < 5000.0)) r.speed_mps = -1.0;
        out.push_back(r);
    }
    return out;
}

// Least-squares line through the readings (speed or altitude) within +-half of t: value, slope, count.
struct Fit {
    double value = 0.0, slope = 0.0;
    int n = 0;
};

// Readings off the window's median by more than 10 % are OCR errors the table's cleaning let through.
Fit fit(const std::vector<Reading>& d, double t, double half, bool altitude) {
    std::vector<double> window;
    for (const auto& r : d) {
        const double y = altitude ? r.altitude_m : r.speed_mps;
        if (y >= 0.0 && std::abs(r.t_s - t) <= half) window.push_back(y);
    }
    if (window.size() < 3) return {};
    std::nth_element(window.begin(), window.begin() + static_cast<long>(window.size() / 2), window.end());
    const double median = window[window.size() / 2];
    double n = 0, st = 0, sy = 0, stt = 0, sty = 0;
    for (const auto& r : d) {
        const double y = altitude ? r.altitude_m : r.speed_mps;
        if (y < 0.0 || std::abs(r.t_s - t) > half) continue;
        if (std::abs(y - median) > 0.1 * median + (altitude ? 2000.0 : 20.0)) continue;
        const double x = r.t_s - t;
        n += 1;
        st += x;
        sy += y;
        stt += x * x;
        sty += x * y;
    }
    Fit f;
    f.n = static_cast<int>(n);
    if (n < 3) return f;
    f.slope = (n * sty - st * sy) / (n * stt - st * st);
    f.value = (sy - f.slope * st) / n;
    return f;
}

double interpolate(const std::vector<double>& x, const std::vector<double>& y, double at) {
    if (at <= x.front()) return y.front();
    if (at >= x.back()) return y.back();
    const size_t i = static_cast<size_t>(std::upper_bound(x.begin(), x.end(), at) - x.begin());
    const double f = (at - x[i - 1]) / (x[i] - x[i - 1]);
    return y[i - 1] + f * (y[i] - y[i - 1]);
}

gnc::AlphaSchedule alpha_schedule() { return gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv"); }

struct ReplayResult {
    double speed_rms = 0.0;      // relative, speed over time above Mach 3 (the bank holds the recorded altitude)
    int speed_points = 0;
    double low_rms = 0.0;        // relative, speed over altitude from Mach 3 to 1 (below 40 km, steep descent)
    int low_points = 0;
    double saturated_fraction = 0.0;  // of the control steps with the vertical lift fraction at a limit
    double subsonic_ratio = 0.0;      // mean model / telemetry speed between 10 and 3 km (0: no readings)
};

constexpr double nominal_lift_fraction = 0.5;  // vertical share of the lift on the recorded path [E]

ReplayResult replay(const char* file, double t_start, const char* name) {
    const auto d = load_ship(file);
    const auto schedule = alpha_schedule();
    const Fit v0 = fit(d, t_start, 3.0, false);
    const Fit h0 = fit(d, t_start, 20.0, true);
    REQUIRE(v0.n >= 3);
    REQUIRE(h0.n >= 3);
    test::EntryStart start;
    start.height_m = h0.value;
    start.speed_relative_mps = v0.value;
    start.flight_path_angle_rad = std::asin(std::clamp(h0.slope / v0.value, -1.0, 1.0));
    start.bank_rad = std::acos(nominal_lift_fraction);  // on the recorded path: the feedback's mid value
    test::ShipEntryScenario scenario(start);

    int step = 0, saturated = 0;
    double mach_now = 30.0;
    const auto law = [&](const vehicle::FlightTelemetry& t, double& alpha, double& bank) {
        const double tt = t_start + step++;
        if (t.knudsen < 0.01) mach_now = t.mach;
        alpha = schedule.at(mach_now);
        const Fit h = fit(d, tt, 20.0, true);
        if (h.n < 3) return;  // no altitude readings: keep the bank
        // Vertical lift fraction: altitude and altitude-rate feedback around a mid value [E gains].
        const double fraction = nominal_lift_fraction + 0.3 * (h.value - t.altitude_m) / 1000.0 + 0.3 * (h.slope - t.vertical_speed_mps) / 50.0;
        const double clamped = std::clamp(fraction, -1.0, 1.0);
        if (clamped != fraction && t.dynamic_pressure_pa > 500.0) ++saturated;
        // Bank changes at most 1 deg/s [E]: faster reversals couple into the slow yaw axis (belly axis, large
        // inertia, weak flap authority) and the angle of attack departs (3 deg/s: excursions to 90-150 deg).
        constexpr double bank_rate = 1.0 * deg;
        bank = std::clamp(std::acos(clamped), bank - bank_rate, bank + bank_rate);
    };
    const auto samples = scenario.run(law, 0.0, 2500.0, 3000.0);

    // Recorded speed over altitude below 40 km (ascending altitude), for the steep supersonic descent.
    std::vector<double> low_altitude, low_speed;
    {
        std::vector<std::pair<double, double>> pts;
        for (const auto& x : d) {
            if (x.t_s <= t_start || x.altitude_m < 0.0 || x.altitude_m > 40000.0 || x.speed_mps < 0.0) continue;
            pts.emplace_back(x.altitude_m, x.speed_mps);
        }
        std::sort(pts.begin(), pts.end());
        for (const auto& [h, v] : pts) {
            if (!low_altitude.empty() && h <= low_altitude.back()) continue;  // 0.1 km steps: one reading per step
            low_altitude.push_back(h);
            low_speed.push_back(v);
        }
    }
    ReplayResult r;
    double sum = 0.0, low_sum = 0.0;
    double model_sub = 0.0, tel_sub = 0.0;
    int n_model_sub = 0, n_tel_sub = 0;
    const bool verbose = std::getenv("STARSIM_VERBOSE") != nullptr;
    if (verbose) std::cout << name << " start h " << start.height_m / 1e3 << " km v " << start.speed_relative_mps
                           << " gamma " << start.flight_path_angle_rad / deg << "\n";
    for (const auto& s : samples) {
        const auto& t = s.telemetry;
        if (t.altitude_m <= 10000.0 && t.altitude_m >= 3000.0) {
            model_sub += t.speed_relative_mps;
            ++n_model_sub;
        }
        const bool continuum = t.altitude_m < 40000.0;
        if (continuum && t.mach < 1.0) continue;
        if (continuum && t.mach < 3.0) {
            // Decelerating at ~14 m/s^2 a few seconds of timing shift are many percent of speed: compare over altitude.
            if (low_altitude.size() >= 5 && t.altitude_m > low_altitude.front() && t.altitude_m < low_altitude.back()) {
                const double v_tel = interpolate(low_altitude, low_speed, t.altitude_m);
                const double e = (t.speed_relative_mps - v_tel) / v_tel;
                low_sum += e * e;
                ++r.low_points;
                if (verbose && r.low_points % 5 == 1)
                    std::cout << "  low h " << t.altitude_m / 1e3 << " v " << t.speed_relative_mps << " tel " << v_tel
                              << " mach " << t.mach << "\n";
            }
            continue;
        }
        const Fit v = fit(d, t_start + s.t_s, 3.0, false);
        if (v.n < 3) continue;
        const double e = (t.speed_relative_mps - v.value) / v.value;
        sum += e * e;
        ++r.speed_points;
        if (verbose && static_cast<int>(s.t_s) % 50 == 0) {
            const Fit h = fit(d, t_start + s.t_s, 20.0, true);
            std::cout << "  t " << s.t_s << " h " << t.altitude_m / 1e3 << " (" << h.value / 1e3 << ") v "
                      << t.speed_relative_mps << " (" << v.value << ") aoa " << t.angle_of_attack_rad / deg << " drag "
                      << s.drag_mps2 << " mach " << t.mach << "\n";
        }
    }
    for (const auto& x : d) {
        if (x.altitude_m >= 3000.0 && x.altitude_m <= 10000.0 && x.speed_mps > 0.0 && x.t_s > t_start) {
            tel_sub += x.speed_mps;
            ++n_tel_sub;
        }
    }
    r.speed_rms = r.speed_points > 0 ? std::sqrt(sum / r.speed_points) : 1.0;
    r.low_rms = r.low_points > 0 ? std::sqrt(low_sum / r.low_points) : 0.0;
    r.saturated_fraction = step > 0 ? static_cast<double>(saturated) / step : 0.0;
    if (n_model_sub > 0 && n_tel_sub >= 5) r.subsonic_ratio = (model_sub / n_model_sub) / (tel_sub / n_tel_sub);
    if (verbose) std::cout << name << ": speed rms " << r.speed_rms * 100 << " % over " << r.speed_points
                           << " s, below Mach 3 over altitude " << r.low_rms * 100 << " % over " << r.low_points
                           << " s, lift saturated " << r.saturated_fraction * 100 << " %, subsonic ratio "
                           << r.subsonic_ratio << ", breakup '" << scenario.body().breakup_reason() << "'\n";
    CHECK_FALSE(scenario.body().impacted());
    return r;
}

}  // namespace

TEST_CASE("Ship entry reproduces the webcast telemetry of flights 12-14", "[ship][entry][validation]") {
    struct Flight {
        const char* file;
        double t_start;  // first readings below ~88 km
        const char* name;
        bool subsonic;   // telemetry covers 10-3 km
    };
    const Flight flights[] = {{"flight13_2026-07-24.csv", 2800.5, "flight 13", true},
                              {"flight12_2026-05-22.csv", 2820.5, "flight 12", false},
                              {"flight14_2026-09-28.csv", 9990.5, "flight 14", true}};
    for (const auto& f : flights) {
        CAPTURE(f.name);
        const ReplayResult r = replay(f.file, f.t_start, f.name);
        CHECK(r.speed_points > 200);
        CHECK(r.speed_rms < 0.05);
        if (r.low_points > 0) CHECK(r.low_rms < 0.10);
        CHECK(r.saturated_fraction < 0.1);
        if (f.subsonic) {
            CHECK(r.subsonic_ratio > 0.9);
            CHECK(r.subsonic_ratio < 1.1);
        }
    }
}
