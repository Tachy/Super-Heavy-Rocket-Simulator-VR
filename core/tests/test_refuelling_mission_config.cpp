// Refuelling r5: the mission configurations of the reference flights.

#include <filesystem>
#include <fstream>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/refuelling_mission_config.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
constexpr double deg = constants::deg_to_rad;

// A directory with one mission file (unique directory per test).
std::filesystem::path with_mission(const std::string& name, const std::string& id, const std::string& text) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_r5_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "missions");
    std::ofstream(dir / "missions" / (id + ".toml")) << text;
    return dir;
}

}  // namespace

TEST_CASE("depot_launch and tanker_flight configurations load", "[refuelling][config][mission]") {
    CHECK(config::mission_kind(config_dir, "depot_launch") == config::MissionKind::DepotLaunch);
    const auto d = config::load_depot_launch_mission(config_dir, "depot_launch");
    CHECK(d.ascent_id == "depot_ascent");
    const auto a = config::load_ascent_mission(config_dir, "depot_ascent");
    CHECK(a.vehicle_id == "starship_depot");
    CHECK(a.periapsis_height_m == Catch::Approx(200.0e3));
    CHECK(a.inclination_rad == Catch::Approx(26.5 * deg));

    const auto t = config::load_refuelling_flight_mission(config_dir, "tanker_flight");
    CHECK(t.kind == config::MissionKind::TankerFlight);
    CHECK(t.ascent_id == "tanker_ascent");
    CHECK(t.start == config::RefuellingStart::Pad);
    CHECK(t.depot.height_m == Catch::Approx(200.0e3));
    CHECK(t.depot.inclination_rad == Catch::Approx(26.5 * deg));
    CHECK(t.depot.propellant_kg == Catch::Approx(600.0e3));
    CHECK(t.mission_limit_s == Catch::Approx(48.0 * 3600.0));
    CHECK(t.window.search_s == Catch::Approx(12.0 * 86400.0));
    CHECK(t.return_id == "ship_return_starbase");
    const auto ta = config::load_ascent_mission(config_dir, "tanker_ascent");
    CHECK(ta.vehicle_id == "starship_tanker");
    CHECK(ta.periapsis_height_m == Catch::Approx(170.0e3));
    CHECK(ta.apoapsis_height_m == Catch::Approx(190.0e3));

    const auto o = config::load_refuelling_flight_mission(config_dir, "tanker_flight_orbit");
    CHECK(o.start == config::RefuellingStart::PhasingOrbit);
    CHECK(o.phasing_behind_rad == Catch::Approx(8.0 * deg));
    CHECK(o.start_propellant_kg > 0.0);
}

TEST_CASE("Refuelling mission configs refuse bad input", "[refuelling][config][mission]") {
    const std::string base = R"([mission]
name = "x"
kind = "tanker_flight"
ascent = "tanker_ascent"
return = "ship_return_starbase"
[depot]
height_km = 200.0
inclination_deg = 26.5
propellant_t = 600.0
[start]
mode = "%MODE%"
behind_deg = 8.0
propellant_t = 150.0
[launch_window]
earliest_utc = "2026-10-20T00:00:00"
search_days = 12.0
insertion_s = 520.0
insertion_arc_deg = 20.0
[phasing]
mission_limit_h = 48.0
earliest_arrival_h = 0.0
)";
    const auto mode = [&](const std::string& m) {
        std::string text = base;
        text.replace(text.find("%MODE%"), 6, m);
        return text;
    };
    const auto ok = with_mission("cfg_ok", "t", mode("pad"));
    CHECK_NOTHROW(config::load_refuelling_flight_mission(ok, "t"));
    const auto bad = with_mission("cfg_mode", "t", mode("balloon"));
    CHECK_THROWS_WITH(config::load_refuelling_flight_mission(bad, "t"), Catch::Matchers::ContainsSubstring("mode"));
    std::string no_return = mode("pad");
    no_return.erase(no_return.find("return ="), std::string("return = \"ship_return_starbase\"\n").size());
    const auto nr = with_mission("cfg_return", "t", no_return);
    CHECK_THROWS_WITH(config::load_refuelling_flight_mission(nr, "t"), Catch::Matchers::ContainsSubstring("return"));
    std::string ascent = mode("pad");
    ascent.replace(ascent.find("kind = \"tanker_flight\""), 22, "kind = \"ascent\"      ");
    const auto wrong = with_mission("cfg_kind", "t", ascent);
    CHECK_THROWS(config::load_refuelling_flight_mission(wrong, "t"));
    std::string hls = mode("pad");
    hls.replace(hls.find("kind = \"tanker_flight\""), 22, "kind = \"hls_refuel\"  ");
    const auto no_lunar = with_mission("cfg_lunar", "t", hls);
    CHECK_THROWS_WITH(config::load_refuelling_flight_mission(no_lunar, "t"), Catch::Matchers::ContainsSubstring("lunar"));
}
