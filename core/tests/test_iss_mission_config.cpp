// ISS i6: the three ISS mission kinds load from config/missions and refuse bad values.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <string>

#include "starsim/config/mission_config.hpp"

using namespace starsim;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
constexpr double deg = constants::deg_to_rad;

// A directory with one mission file (unique per test case: ctest runs in parallel).
std::filesystem::path config_with(const std::string& tag, const std::string& id, const std::string& text) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_iss_cfg_" + tag);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "missions");
    std::ofstream(dir / "missions" / (id + ".toml")) << text;
    return dir;
}

const std::string base = R"([mission]
name = "x"
kind = "iss_rendezvous"
vehicle = "crew_dragon"
station = "iss"
port = "ida3"
iss_orbit = "synthetic"
[start]
epoch_utc = "2026-10-16T00:00:00"
launch = "falcon9_crew_dragon_iss"
phase_deg = 30.0
iss_argument_of_latitude_deg = 10.0
[phasing]
mission_limit_h = 28.0
earliest_arrival_h = 0.0
)";
}  // namespace

TEST_CASE("ISS mission config: the three kinds load", "[iss][iss_mission][config]") {
    CHECK(config::mission_kind(config_dir, "crew_dragon_iss") == config::MissionKind::IssRendezvous);
    CHECK(config::mission_kind(config_dir, "falcon9_crew_dragon_iss") == config::MissionKind::IssLaunch);
    CHECK(config::mission_kind(config_dir, "crew_dragon_iss_departure") == config::MissionKind::IssDeparture);

    const auto r = config::load_iss_rendezvous_mission(config_dir, "crew_dragon_iss");
    CHECK(r.vehicle_id == "crew_dragon");
    CHECK(r.station_id == "iss");
    CHECK(r.port == "ida2");
    CHECK(r.iss_orbit == "auto");
    CHECK(r.launch_id == "falcon9_crew_dragon_iss");
    CHECK_FALSE(r.phase_rad);
    CHECK(r.mission_limit_s == Catch::Approx(28.0 * 3600.0));
    CHECK(r.earliest_arrival_s == 0.0);

    const auto l = config::load_iss_launch_mission(config_dir, "falcon9_crew_dragon_iss");
    CHECK(l.ascent_id == "falcon9_crew_dragon");
    CHECK(l.port == "ida2");
    CHECK(l.window.search_s == Catch::Approx(3.0 * 86400.0));
    CHECK(l.window.insertion_s > 0.0);
    CHECK(l.window.insertion_arc_rad > 0.0);
    CHECK(l.mission_limit_s == Catch::Approx(28.0 * 3600.0));

    const auto d = config::load_iss_departure_mission(config_dir, "crew_dragon_iss_departure");
    CHECK(d.return_id == "crew_dragon_return");
    CHECK(d.undock_to_deorbit_s == Catch::Approx(16.1 * 3600.0));
    CHECK(d.search_window_s == Catch::Approx(168.0 * 3600.0));
    CHECK(d.rcs_propellant_kg == Catch::Approx(2000.0));
    CHECK_FALSE(d.iss_argument_of_latitude_rad);
}

TEST_CASE("ISS mission config: optional phase and refusals", "[iss][iss_mission][config]") {
    const auto ok = config::load_iss_rendezvous_mission(config_with("phase", "m", base), "m");
    REQUIRE(ok.phase_rad);
    CHECK(*ok.phase_rad == Catch::Approx(30.0 * deg));
    REQUIRE(ok.iss_argument_of_latitude_rad);
    CHECK(*ok.iss_argument_of_latitude_rad == Catch::Approx(10.0 * deg));

    std::string bad_orbit = base;
    bad_orbit.replace(bad_orbit.find("\"synthetic\""), 11, "\"sgp4\"");
    CHECK_THROWS(config::load_iss_rendezvous_mission(config_with("orbit", "m", bad_orbit), "m"));

    std::string no_port = base;
    no_port.erase(no_port.find("port = \"ida3\"\n"), 14);
    CHECK_THROWS(config::load_iss_rendezvous_mission(config_with("port", "m", no_port), "m"));

    // a rendezvous file is not a launch mission
    CHECK_THROWS(config::load_iss_launch_mission(config_with("kind", "m", base), "m"));
}
