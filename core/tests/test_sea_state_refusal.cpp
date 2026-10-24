// Mission sea state ([sea_state]) and the onboard computer's refusal of a drone-ship landing in a rough sea
// (sim::sea_state_refusal inside sim::plan_launch).

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "starsim/config/drone_ship_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/deck_motion.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/ascent_plan_cache.hpp"
#include "starsim/sim/ascent_profile_planner.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

const spice::SpiceEphemeris& de440() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

// The liftoff of test_ascent_profile_planner.cpp's requests (shares their disk-cached plans).
time::Epoch liftoff() {
    test::spice_session();
    return spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
}

std::string file_text(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// A missions directory (unique per test) with one mission file `id` whose text is `text`.
std::filesystem::path mission_dir(const std::string& name, const std::string& id, const std::string& text) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_sea_state_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "missions");
    std::ofstream(dir / "missions" / (id + ".toml"), std::ios::binary) << text;
    return dir;
}

std::string with_sea_state(const std::string& sea_state_table) {
    return file_text(std::filesystem::path(STARSIM_CONFIG_DIR) / "missions" / "falcon9_asds.toml") + "\n[sea_state]\n" +
           sea_state_table;
}

environment::SeaStateConfig sea(double hs_m, double tp_s, double heading_rad) {
    environment::SeaStateConfig c;
    c.significant_wave_height_m = hs_m;
    c.peak_period_s = tp_s;
    c.heading_rad = heading_rad;
    c.seed = 1;
    return c;
}

// Falcon 9 with the payload of ascent mission `mission_id` (as test_ascent_profile_planner.cpp's falcon9_request).
sim::AscentProfileRequest falcon9_request(const char* mission_id) {
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, mission_id);
    sim::AscentProfileRequest r;
    r.vehicle_id = m.vehicle_id;
    r.site_id = m.site_id;
    r.heading_rad = m.heading_rad;
    r.liftoff = liftoff();
    r.target = {m.periapsis_height_m, m.apoapsis_height_m, m.inclination_rad, m.northerly};
    r.payload_kg = m.payload_kg;
    r.fairing = m.fairing;
    r.payload = m.payload;
    return r;
}

bool starts_with(const std::string& text, const std::string& prefix) { return text.rfind(prefix, 0) == 0; }

}  // namespace

TEST_CASE("A mission's [sea_state] loads", "[config][asds][sea_state]") {
    const auto dir = mission_dir("loads", "rough",
                                 with_sea_state("significant_wave_height_m = 2.5\npeak_period_s = 8.0\n"
                                                "heading_deg = 60.0\nseed = 7\n"));
    const auto s = config::mission_sea_state(dir, "rough");
    REQUIRE(s);
    CHECK(s->significant_wave_height_m == 2.5);
    CHECK(s->peak_period_s == 8.0);
    CHECK(std::abs(s->heading_rad - 60.0 * deg) < 1e-12);
    CHECK(s->seed == 7u);
    // the ascent mission carries it too
    const auto m = config::load_ascent_mission(dir, "rough");
    REQUIRE(m.sea_state);
    CHECK(m.sea_state->significant_wave_height_m == 2.5);
    // a mission without the table: calm
    CHECK_FALSE(config::mission_sea_state(STARSIM_CONFIG_DIR, "falcon9_asds"));
    CHECK_FALSE(config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_asds").sea_state);
}

TEST_CASE("A bad [sea_state] is a load error naming the key", "[config][asds][sea_state]") {
    auto error_of = [](const std::string& name, const std::string& table) -> std::string {
        const auto dir = mission_dir(name, "bad", with_sea_state(table));
        try {
            config::mission_sea_state(dir, "bad");
        } catch (const config::ConfigError& e) {
            return e.what();
        }
        return {};
    };
    CHECK(error_of("no_tp", "significant_wave_height_m = 2.0\n").find("peak_period_s") != std::string::npos);
    CHECK(error_of("tp_zero", "significant_wave_height_m = 2.0\npeak_period_s = 0.0\n").find("peak_period_s") !=
          std::string::npos);
    CHECK(error_of("hs_negative", "significant_wave_height_m = -1.0\npeak_period_s = 8.0\n")
              .find("significant_wave_height_m") != std::string::npos);
    CHECK(error_of("seed_negative", "significant_wave_height_m = 1.0\npeak_period_s = 8.0\nseed = -1\n").find("seed") !=
          std::string::npos);
}

TEST_CASE("sea_state_refusal: within the limits, above them, flag off", "[sim][asds][sea_state]") {
    const auto ship = config::load_drone_ship(STARSIM_CONFIG_DIR, "asds_atlantic");
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    CHECK_FALSE(sim::sea_state_refusal(session, ship, 0.0));  // calm sea

    // Hs 1.5 m / Tp 8 s from astern (waves travel along the deck's long axis): tilt ~0.9 deg, heave rate ~0.1 m/s
    session.set_sea_state(sea(1.5, 8.0, 30.0 * deg));
    CHECK_FALSE(sim::sea_state_refusal(session, ship, 30.0 * deg));

    // Hs 6 m / Tp 10 s abeam: far above both limits
    session.set_sea_state(sea(6.0, 10.0, 90.0 * deg));
    const auto reason = sim::sea_state_refusal(session, ship, 0.0);
    REQUIRE(reason);
    std::printf("[sea_state] %s\n", reason->c_str());
    CHECK(starts_with(*reason, "Seegang im Landegebiet zu hoch"));
    const auto stats = contact::DeckMotion(*session.sea_state(), ship.hull, 0.0).statistics();
    char value[32], limit[32];
    std::snprintf(value, sizeof value, "%.1f", stats.significant_tilt_rad / deg);
    std::snprintf(limit, sizeof limit, "%.1f", ship.max_deck_tilt_rad / deg);
    CHECK(reason->find(value) != std::string::npos);
    CHECK(reason->find(limit) != std::string::npos);

    session.set_deck_motion(false);  // fidelity flag off: the old behaviour, no refusal
    CHECK_FALSE(sim::sea_state_refusal(session, ship, 0.0));
}

TEST_CASE("A rough-sea drone-ship launch is refused before its drone-ship trials", "[sim][asds][sea_state]") {
    // The Starlink-class request (falcon9_asds.toml): the RTLS return leaves the upper stage short of this orbit, so the
    // drone ship would be chosen. Its RTLS plan is answered first (memo, from the disk cache when another process has
    // planned it); then the disk cache is switched off so that any drone-ship trial would have to be flown.
    const auto request = falcon9_request("falcon9_asds");
    sim::FlightSession like(de440(), earth(), STARSIM_CONFIG_DIR);
    auto rtls = request;
    const auto zone = config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_lz40");
    rtls.recovery = {sim::RecoveryKind::LandingZone, zone.id, zone.latitude_rad, zone.longitude_rad, 0.0};
    REQUIRE_FALSE(sim::plan_ascent_profile(like, rtls).feasible);
    sim::disable_plan_cache();

    like.set_sea_state(sea(6.0, 10.0, 60.0 * deg));
    const auto plan = sim::plan_launch(like, request);
    std::printf("[sea_state] %s\n", plan.reason.c_str());
    CHECK_FALSE(plan.feasible);
    CHECK(starts_with(plan.reason, "Seegang im Landegebiet zu hoch"));
    CHECK(plan.trial_flights == 0);  // RTLS from the memo, no drone-ship trial
    CHECK_FALSE(plan.booster_impact);
}

TEST_CASE("An RTLS launch ignores the sea state", "[sim][asds][sea_state]") {
    const auto request = falcon9_request("falcon9_rtls");  // CRS-35 class: back to LZ-40
    sim::FlightSession like(de440(), earth(), STARSIM_CONFIG_DIR);
    like.set_sea_state(sea(6.0, 10.0, 60.0 * deg));
    const auto plan = sim::plan_launch(like, request);
    REQUIRE(plan.feasible);
    CHECK(plan.recovery.kind == sim::RecoveryKind::LandingZone);
    CHECK(plan.recovery.zone_or_ship_id == "ccsfs_lz40");
}

TEST_CASE("A mission start sets its sea state; the next start without one resets it", "[sim][asds][sea_state]") {
    // The mission file lives in a copy of the config (unique directory); the session keeps the repository config, so
    // its plans come from the planner's memo / disk cache (keyed by the session's config directory).
    const auto dir = std::filesystem::temp_directory_path() / "starsim_sea_state_mission_start";
    std::filesystem::remove_all(dir);
    std::filesystem::copy(STARSIM_CONFIG_DIR, dir, std::filesystem::copy_options::recursive);
    std::ofstream(dir / "missions" / "falcon9_asds_swell.toml", std::ios::binary)
        << with_sea_state("significant_wave_height_m = 1.5\npeak_period_s = 8.0\nheading_deg = 45.0\nseed = 3\n");
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto t0 = time.from_utc({2026, 10, 13, 10, 40, 0.0});  // test_falcon9_asds.cpp's start
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    const auto swell = sim::start_mission(session, dir, "falcon9_asds_swell", t0, time);
    REQUIRE(swell.refusal.empty());
    REQUIRE(session.sea_state());
    CHECK(session.sea_state()->config().significant_wave_height_m == 1.5);
    REQUIRE(session.landing_zone());
    REQUIRE(session.landing_zone()->deck);
    CHECK(session.deck_pose());  // the drone ship set after the sea state moves with it

    const auto calm = sim::start_mission(session, STARSIM_CONFIG_DIR, "falcon9_asds", t0, time);
    REQUIRE(calm.refusal.empty());
    CHECK_FALSE(session.sea_state());
    CHECK_FALSE(session.deck_pose());
}

TEST_CASE("Sea-state limits of the drone ship in Hs", "[.][sea_state_limits]") {
    // Prints the significant wave height at which the deck reaches a landing limit (docs/subsystems.md, drone_ships.toml).
    const auto ship = config::load_drone_ship(STARSIM_CONFIG_DIR, "asds_atlantic");
    for (const double tp : {8.0, 10.0}) {
        for (const double angle_deg : {0.0, 90.0}) {
            const auto s = contact::DeckMotion(environment::SeaState(sea(1.0, tp, angle_deg * deg)), ship.hull, 0.0)
                               .statistics();  // linear: the statistics scale with Hs
            const double hs_tilt = ship.max_deck_tilt_rad / s.significant_tilt_rad;
            const double hs_rate = ship.max_heave_rate_mps / s.significant_heave_rate_mps;
            std::printf("[sea_state] Tp %.0f s, waves %.0f deg off the long axis: Hs limit %.2f m (tilt) / %.2f m "
                        "(heave rate)\n",
                        tp, angle_deg, hs_tilt, hs_rate);
        }
    }
}
