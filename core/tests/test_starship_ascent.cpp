// Starship (Block 3) ascent mission against the webcast telemetry of flights 13 and 14: booster throttle schedule from
// the propellant gauge, MECO planned for the booster's return to the tower.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double recorded_meco_propellant_kg = 500e3;  // [D] gauge ~13.5 % of 3,650 t at MECO, flights 13 / 14

struct StackSample {
    double t_s = 0.0, altitude_km = 0.0, speed_kmh = 0.0;
};

struct StarshipAscent {
    std::vector<StackSample> samples;  // at the requested times, until MECO
    double meco_s = -1.0, meco_altitude_km = 0.0, meco_speed_kmh = 0.0;
};

// Flies the mission until the stack separates; samples the stack at `times` (mission time). `meco_propellant_kg` > 0:
// no booster return, MECO with this much left in the booster (as recorded) instead of the return guidance's plan.
StarshipAscent fly_mission(const config::AscentMission& mission, const std::vector<double>& times,
                           double meco_propellant_kg = 0.0) {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
    if (meco_propellant_kg > 0.0) {
        auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, mission.vehicle_id);
        cfg.stages.back().payload_mass_kg = mission.payload_kg;
        cfg.stages.front().reserve_propellant_kg = meco_propellant_kg;
        session.launch_from_pad(cfg, sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, mission.site_id)),
                                mission.heading_rad, t0);
        gnc::PitchProgram program;
        program.kick_angle_rad = mission.kick_angle_rad;
        program.booster_throttle = mission.booster_throttle;
        session.engage_ascent_autopilot(
            {mission.periapsis_height_m, mission.apoapsis_height_m, mission.inclination_rad, mission.northerly}, program);
    } else {
        sim::start_ascent_mission(session, mission, STARSIM_CONFIG_DIR, t0);
    }
    StarshipAscent a;
    size_t next = 0;
    for (double t = 0.0; t < 300.0; t += 0.5) {
        session.advance_to(t0 + t, 1000);
        const auto& b = session.body(session.controlled_index());
        const auto tel = session.telemetry(session.controlled_index());
        const double t_mission = session.mission_time_s();
        if (std::getenv("STARSIM_DEBUG_ASCENT_G") && std::fmod(t, 5.0) < 0.25) {
            std::printf("  [g] T+%3.0f  %6.2f km  %6.0f km/h  %4.2f g  m %7.1f t  prop %7.1f t  thrust %6.1f MN\n", t_mission,
                        tel.altitude_m / 1e3, tel.speed_relative_mps * 3.6, tel.sensed_acceleration_g, tel.mass_kg / 1e3,
                        tel.propellant_kg / 1e3, tel.thrust_n / 1e6);
        }
        if (session.autopilot().state == sim::AutopilotState::Booster && next < times.size() && t_mission >= times[next]) {
            a.samples.push_back({times[next++], tel.altitude_m / 1e3, tel.speed_relative_mps * 3.6});
        }
        if (a.meco_s < 0.0 && session.autopilot().state != sim::AutopilotState::Booster && t_mission > 10.0) {
            a.meco_s = t_mission;
            a.meco_altitude_km = tel.altitude_m / 1e3;
            a.meco_speed_kmh = tel.speed_relative_mps * 3.6;
        }
        if (a.meco_s > 0.0 || b.impacted()) break;
    }
    return a;
}

}  // namespace

TEST_CASE("Starship ascent follows the webcast telemetry of flights 13 and 14", "[gnc][validation][ascent]") {
    // Recorded (tests/validation/telemetry, flights 13 / 14, displayed altitude above the pad and speed relative to the
    // ground): T+30 s 1.6 km 518 / 513 km/h, T+60 s 8.5 / 8.3 km 1404 / 1408 km/h, T+90 s 20.8 / 20.1 km 2787 / 2758 km/h,
    // T+120 s 39.0 / 37.4 km 4878 / 4817 km/h; MECO ~T+140 s at ~53 km, ~5900-6000 km/h with ~13.5 % = ~500 t left in
    // the booster (gauge): flown with that MECO, not the return guidance's plan (our return needs less: known gap).
    struct Recorded {
        double t_s, altitude_km, speed_kmh;
    };
    const Recorded recorded[] = {{30.0, 1.6, 515.0}, {60.0, 8.4, 1406.0}, {90.0, 20.5, 2772.0}, {120.0, 38.2, 4848.0}};
    std::vector<double> times;
    for (const auto& r : recorded) times.push_back(r.t_s);
    const auto mission = config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo");
    const auto a = fly_mission(mission, times, recorded_meco_propellant_kg);
    REQUIRE(a.samples.size() == times.size());
    for (size_t i = 0; i < times.size(); ++i) {
        std::printf("[validation] Starship T+%.0f s: recorded %.1f km %.0f km/h, simulated %.1f km %.0f km/h\n",
                    recorded[i].t_s, recorded[i].altitude_km, recorded[i].speed_kmh, a.samples[i].altitude_km,
                    a.samples[i].speed_kmh);
        CHECK(std::abs(a.samples[i].speed_kmh / recorded[i].speed_kmh - 1.0) < 0.12);
    }
    std::printf("[validation] Starship MECO: recorded ~T+140 s, ~53 km, ~5950 km/h; simulated T+%.0f s, %.1f km, %.0f km/h\n",
                a.meco_s, a.meco_altitude_km, a.meco_speed_kmh);
    CHECK(std::abs(a.meco_s - 140.0) < 12.0);
    CHECK(std::abs(a.meco_speed_kmh / 5950.0 - 1.0) < 0.1);
    CHECK(std::abs(a.meco_altitude_km / 53.0 - 1.0) < 0.15);
}

TEST_CASE("Starship pitch kick sweep", "[.tuning]") {
    auto mission = config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo");
    for (double kick_deg : {10.0, 11.0, 12.0, 13.0, 14.0}) {
        mission.kick_angle_rad = kick_deg * deg;
        const auto a = fly_mission(mission, {60.0, 90.0, 120.0}, recorded_meco_propellant_kg);
        std::printf("kick %4.1f deg: T+60 %.1f km %.0f km/h, T+90 %.1f km %.0f km/h, T+120 %.1f km %.0f km/h; MECO T+%.0f s %.1f "
                    "km %.0f km/h\n", kick_deg, a.samples[0].altitude_km, a.samples[0].speed_kmh, a.samples[1].altitude_km,
                    a.samples[1].speed_kmh, a.samples[2].altitude_km, a.samples[2].speed_kmh, a.meco_s, a.meco_altitude_km,
                    a.meco_speed_kmh);
    }
}
