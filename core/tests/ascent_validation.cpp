#include "ascent_validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <tuple>

#include "starsim/config/mission_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

namespace starsim::test {
namespace {

constexpr double sample_step_s = 0.25;      // the tables' 2 readings/s, offset by a quarter second
constexpr double flight_limit_s = 1000.0;   // beyond every SECO
constexpr double speed_half_window_s = 1.0;
constexpr double altitude_half_window_s = 2.0;  // the dials show 0.1 km

double stage_propellant_t(const vehicle::Stage& stage) {
    return (stage.propellant_kg(config::TankContents::Oxidizer) + stage.propellant_kg(config::TankContents::Fuel)) / 1e3;
}

FlownAscent::Sample sample_of(const sim::FlightSession& session) {
    const auto tel = session.telemetry(session.controlled_index());
    FlownAscent::Sample s;
    s.t_s = session.mission_time_s();
    s.altitude_km = tel.altitude_m / 1e3;
    s.speed_kmh = tel.speed_relative_mps * 3.6;
    s.gamma_deg = tel.flight_path_angle_rad / constants::deg_to_rad;
    s.mass_t = tel.mass_kg / 1e3;
    s.thrust_mn = tel.thrust_n / 1e6;
    s.sensed_g = tel.sensed_acceleration_g;
    s.q_kpa = tel.dynamic_pressure_pa / 1e3;
    s.mach = tel.mach;
    return s;
}

void dump(const std::string& mission_id, const FlownAscent& a) {
    const char* dir = std::getenv("STARSIM_ASCENT_DUMP");
    if (!dir) return;
    std::ofstream out(std::filesystem::path(dir) / (mission_id + ".csv"));
    out << "t_s,altitude_km,speed_kmh,gamma_deg,mass_t,thrust_mn,sensed_g,q_kpa,mach\n";
    for (const auto& s : a.samples) {
        out << s.t_s << ',' << s.altitude_km << ',' << s.speed_kmh << ',' << s.gamma_deg << ',' << s.mass_t << ','
            << s.thrust_mn << ',' << s.sensed_g << ',' << s.q_kpa << ',' << s.mach << '\n';
    }
}

// Least-squares line through (t, y) within +-half of t0: value and slope at t0 (NaN: fewer than 2 readings).
std::pair<double, double> line_at(const std::vector<std::pair<double, double>>& readings, double t0, double half) {
    double n = 0, st = 0, sy = 0, stt = 0, sty = 0;
    for (const auto& [t, y] : readings) {
        if (std::abs(t - t0) > half) continue;
        const double u = t - t0;
        n += 1;
        st += u;
        sy += y;
        stt += u * u;
        sty += u * y;
    }
    if (n < 2) return {std::nan(""), std::nan("")};
    const double slope = (n * sty - st * sy) / (n * stt - st * st);
    return {(sy - slope * st) / n, slope};
}

}  // namespace

FlownAscent::Sample FlownAscent::at(double t_s) const {
    if (samples.empty()) throw std::out_of_range("no samples");
    auto it = std::lower_bound(samples.begin(), samples.end(), t_s,
                               [](const Sample& s, double t) { return s.t_s < t; });
    if (it == samples.begin()) return samples.front();
    if (it == samples.end()) return samples.back();
    const auto& b = *it;
    const auto& a = *(it - 1);
    const double f = (t_s - a.t_s) / (b.t_s - a.t_s);
    Sample s = a;
    s.t_s = t_s;
    s.altitude_km += f * (b.altitude_km - a.altitude_km);
    s.speed_kmh += f * (b.speed_kmh - a.speed_kmh);
    s.gamma_deg += f * (b.gamma_deg - a.gamma_deg);
    return s;
}

FlownAscent fly_validation_ascent(const std::string& mission_id, time::Epoch liftoff) {
    spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto mission = config::load_ascent_mission(STARSIM_CONFIG_DIR, mission_id);
    FlownAscent a;
    a.plan = sim::start_ascent_mission(session, mission, STARSIM_CONFIG_DIR, liftoff);
    if (!a.plan.feasible) return a;
    bool booster_phase_seen = false;
    for (double t = sample_step_s; t < flight_limit_s; t += sample_step_s) {
        session.advance_to(liftoff + t, 1000);
        const auto state = session.autopilot().state;
        const auto index = session.controlled_index();
        const auto& body = session.body(index);
        if (body.impacted()) break;
        if (state == sim::AutopilotState::PreLaunch) continue;
        const auto s = sample_of(session);
        if (state == sim::AutopilotState::Booster) {
            booster_phase_seen = true;
            a.meco = s;
            a.booster_meco_propellant_t = stage_propellant_t(body.vehicle().stages().front());
        }
        if (state == sim::AutopilotState::UpperStage) {
            for (size_t i = 0; i < session.body_count(); ++i) {
                if (i != index) session.end_booster_return(i);
            }
        }
        a.samples.push_back(s);
        if (booster_phase_seen && a.seco_s < 0.0 && state == sim::AutopilotState::Orbit) {
            a.seco_s = s.t_s;
            a.seco = s;
        }
        if (a.seco_s > 0.0 && !body.vehicle().any_engine_producing_thrust()) {
            const auto& st = body.state();
            const auto el = orbit::elements_from_state({st.position_gcrf_m, st.velocity_gcrf_mps}, session.gm_earth_m3ps2());
            a.periapsis_km = (el.semi_major_axis_m * (1.0 - el.eccentricity) - constants::wgs84_semi_major_axis_m) / 1e3;
            a.apoapsis_km = (el.semi_major_axis_m * (1.0 + el.eccentricity) - constants::wgs84_semi_major_axis_m) / 1e3;
            a.inclination_deg = session.telemetry(index).inclination_rad / constants::deg_to_rad;
            a.upper_propellant_at_seco_t = body.vehicle().propellant_kg() / 1e3;
            break;
        }
    }
    dump(mission_id, a);
    return a;
}

RecordedReading recorded_stack(const std::string& table, double t_s, const std::string& vehicle) {
    std::ifstream in(std::string(STARSIM_TELEMETRY_DIR) + "/" + table);
    if (!in) throw std::runtime_error("missing telemetry table " + table);
    std::vector<std::pair<double, double>> speed, altitude;
    std::string line;
    std::vector<std::string> header;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> f;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) f.push_back(cell);
        if (header.empty()) {
            header = f;
            continue;
        }
        f.resize(header.size());
        const auto column = [&](const std::string& name) {
            const auto it = std::find(header.begin(), header.end(), name);
            if (it == header.end()) throw std::runtime_error(table + ": no column " + name);
            return f[static_cast<size_t>(it - header.begin())];
        };
        const double t = std::stod(f[0]);
        if (std::abs(t - t_s) > altitude_half_window_s) continue;
        if (const auto v = column(vehicle + "_speed_kmh"); !v.empty()) speed.emplace_back(t, std::stod(v));
        if (const auto h = column(vehicle + "_altitude_km"); !h.empty()) altitude.emplace_back(t, std::stod(h));
    }
    RecordedReading r;
    r.t_s = t_s;
    std::tie(r.speed_kmh, r.speed_rate_kmh_s) = line_at(speed, t_s, speed_half_window_s);
    std::tie(r.altitude_km, r.altitude_rate_km_s) = line_at(altitude, t_s, altitude_half_window_s);
    return r;
}

void print(const std::string& flight, const Comparison& c) {
    std::printf("[validation] %s %-28s recorded %9.2f  simulated %9.2f  (%+6.1f %%)  bound %7.2f%s\n", flight.c_str(),
                c.what.c_str(), c.recorded, c.simulated, 100.0 * (c.simulated / c.recorded - 1.0), c.bound(),
                c.within() ? "" : "  MISS");
}

Comparison compare_speed(const std::string& table, const FlownAscent& flown, double t_s, double time_base_s,
                         double allowance) {
    const auto r = recorded_stack(table, t_s);
    char what[64];
    std::snprintf(what, sizeof what, "T+%.0f s speed [km/h]", t_s);
    return {what, r.speed_kmh, flown.at(t_s - time_base_s).speed_kmh,
            speed_resolution_kmh + std::abs(r.speed_rate_kmh_s) * (reading_time_s + time_base_uncertainty_s), allowance};
}

Comparison compare_altitude(const std::string& table, const FlownAscent& flown, double t_s, double time_base_s,
                            double allowance) {
    const auto r = recorded_stack(table, t_s);
    char what[64];
    std::snprintf(what, sizeof what, "T+%.0f s altitude [km]", t_s);
    return {what, r.altitude_km, flown.at(t_s - time_base_s).altitude_km,
            altitude_resolution_km + std::abs(r.altitude_rate_km_s) * (reading_time_s + time_base_uncertainty_s),
            allowance};
}

}  // namespace starsim::test
