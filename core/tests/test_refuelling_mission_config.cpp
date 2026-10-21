// Refuelling r5: the mission configurations of the reference flights.

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/refuelling_mission_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/refuelling_mission.hpp"

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
    CHECK(ta.periapsis_height_m == Catch::Approx(120.0e3));  // user 2026-10-20: insertion at 120 km, then the raise
    REQUIRE(t.phasing_periapsis_height_m);
    CHECK(*t.phasing_periapsis_height_m == Catch::Approx(170.0e3));
    CHECK(ta.apoapsis_height_m == Catch::Approx(190.0e3));

    // f1: the candidate phasing orbits and the planning values of their cost
    REQUIRE(t.phasing_candidates_m.size() == 5u);  // f2: 180 x 190 km dropped
    CHECK(t.phasing_candidates_m.front().first == Catch::Approx(150.0e3));
    CHECK(t.phasing_candidates_m.front().second == Catch::Approx(190.0e3));
    CHECK(t.phasing_candidates_m.back().first == Catch::Approx(250.0e3));
    CHECK(t.phasing_candidates_m.back().second == Catch::Approx(270.0e3));
    CHECK(t.planning_mass_kg == Catch::Approx(308.0e3));
    CHECK(t.planning_boil_off_kgps == Catch::Approx(15.9e3 / 86400.0));
    const auto h = config::load_refuelling_flight_mission(config_dir, "hls_refuel");
    CHECK(h.phasing_candidates_m.size() == 5u);
    CHECK(h.planning_mass_kg == Catch::Approx(328.0e3));

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
    // r5 minor: the chaser docks with its first port; a [mission] port would be silently ignored
    std::string port = mode("pad");
    port.insert(port.find("ascent = "), "port = \"transfer_dock\"\n");
    const auto with_port = with_mission("cfg_port", "t", port);
    CHECK_THROWS_WITH(config::load_refuelling_flight_mission(with_port, "t"),
                      Catch::Matchers::ContainsSubstring("[mission] port"));
    // f1: candidate phasing orbits
    const auto candidates = [&](const std::string& name, const std::string& lines) {
        return with_mission(name, "t", mode("pad") + lines);
    };
    const auto good = candidates("cfg_cand_ok", "candidates_km = [[150.0, 190.0], [250.0, 270.0]]\n"
                                                "planning_mass_t = 308.0\nplanning_boil_off_t_per_day = 12.0\n");
    CHECK(config::load_refuelling_flight_mission(good, "t").phasing_candidates_m.size() == 2u);
    const auto inverted = candidates("cfg_cand_inv", "candidates_km = [[200.0, 190.0]]\n"
                                                     "planning_mass_t = 308.0\nplanning_boil_off_t_per_day = 12.0\n");
    CHECK_THROWS_WITH(config::load_refuelling_flight_mission(inverted, "t"),
                      Catch::Matchers::ContainsSubstring("candidates_km"));
    const auto low = candidates("cfg_cand_low", "candidates_km = [[90.0, 190.0]]\n"
                                                "planning_mass_t = 308.0\nplanning_boil_off_t_per_day = 12.0\n");
    CHECK_THROWS_WITH(config::load_refuelling_flight_mission(low, "t"), Catch::Matchers::ContainsSubstring("candidates_km"));
    const auto triple = candidates("cfg_cand_3", "candidates_km = [[150.0, 190.0, 200.0]]\n"
                                                 "planning_mass_t = 308.0\nplanning_boil_off_t_per_day = 12.0\n");
    CHECK_THROWS_WITH(config::load_refuelling_flight_mission(triple, "t"), Catch::Matchers::ContainsSubstring("pairs"));
    const auto no_mass = candidates("cfg_cand_mass", "candidates_km = [[150.0, 190.0]]\nplanning_boil_off_t_per_day = 12.0\n");
    CHECK_THROWS_WITH(config::load_refuelling_flight_mission(no_mass, "t"),
                      Catch::Matchers::ContainsSubstring("planning_mass_t"));
    // f1 minor M5: an empty list is an error, not "no candidates but planning values required"; the window margin is
    // read without candidates too
    const auto empty = candidates("cfg_cand_empty", "candidates_km = []\n");
    CHECK_THROWS_WITH(config::load_refuelling_flight_mission(empty, "t"),
                      Catch::Matchers::ContainsSubstring("candidates_km must not be empty"));
    // f4 final review: nan passed number_or and every 'must not be negative' check
    const auto nan_margin = candidates("cfg_margin_nan", "window_margin_h = nan\n");
    CHECK_THROWS_WITH(config::load_refuelling_flight_mission(nan_margin, "t"), Catch::Matchers::ContainsSubstring("finite"));
    const auto margin = candidates("cfg_margin", "window_margin_h = 2.0\n");
    CHECK(config::load_refuelling_flight_mission(margin, "t").window_margin_s == Catch::Approx(7200.0));
    // the reference phasing (a calculator lever) needs the single phasing orbit's periapsis
    const auto m = config::load_refuelling_flight_mission(ok, "t");
    const auto tanker_ascent = config::load_ascent_mission(config_dir, "tanker_ascent");
    CHECK_THROWS_WITH(sim::refuelling_window_parameters(m, tanker_ascent, 3300.0, 0.0, true),
                      Catch::Matchers::ContainsSubstring("periapsis_km"));
}

TEST_CASE("Refuelling window parameters: the insertion orbit and the candidates", "[refuelling][config][mission]") {
    const double re = constants::wgs84_semi_major_axis_m;
    const auto t = config::load_refuelling_flight_mission(config_dir, "tanker_flight");
    const auto a = config::load_ascent_mission(config_dir, t.ascent_id);
    const auto p = sim::refuelling_window_parameters(t, a, 3400.0, 0.0);
    CHECK(p.separation_periapsis_radius_m == Catch::Approx(re + 120.0e3));  // the insertion, the raise follows
    CHECK(p.separation_apoapsis_radius_m == Catch::Approx(re + 190.0e3));
    REQUIRE(p.phasing_orbits.size() == 5u);  // f2: 180 x 190 km dropped
    CHECK(p.phasing_orbits.front().periapsis_radius_m == Catch::Approx(re + 150.0e3));
    CHECK(p.phasing_orbits.back().apoapsis_radius_m == Catch::Approx(re + 270.0e3));
    CHECK(p.cost.mass_kg == Catch::Approx(308.0e3));
    CHECK(p.cost.exhaust_velocity_mps == Catch::Approx(3400.0));
    CHECK(p.cost.boil_off_kgps == Catch::Approx(15.9e3 / 86400.0));
    // the window closes window_margin_h before the limit (the flown re-plans get the full limit)
    CHECK(t.window_margin_s == Catch::Approx(4.0 * 3600.0));
    CHECK(p.phasing.mission_limit_s == Catch::Approx(t.mission_limit_s - t.window.insertion_s - t.window_margin_s));
    // the reference set: the single r5 phasing orbit
    const auto r = sim::refuelling_window_parameters(t, a, 3400.0, 0.0, true);
    REQUIRE(r.phasing_orbits.size() == 1u);
    CHECK(r.phasing_orbits.front().periapsis_radius_m == Catch::Approx(re + 170.0e3));
    CHECK(r.phasing_orbits.front().apoapsis_radius_m == Catch::Approx(re + 190.0e3));
}

TEST_CASE("refuelling_campaign_flown configs load", "[refuelling][config][campaign_flown]") {
    CHECK(config::mission_kind(config_dir, "refuelling_campaign_flown") ==
          config::MissionKind::RefuellingCampaignFlown);
    const auto m = config::load_refuelling_campaign_flown_mission(config_dir, "refuelling_campaign_flown");
    CHECK_FALSE(m.depot_placed);
    CHECK(m.keeping_min_height_m == Catch::Approx(195.0e3));
    CHECK(m.keeping_target_height_m == Catch::Approx(200.0e3));
    CHECK(m.keeping_min_lead_s == Catch::Approx(86400.0));
    CHECK(m.keeping_engine_groups == std::vector<std::string>{"sea_level_3"});
    CHECK(m.return_first_tanker);
    CHECK(m.fill_tolerance == Catch::Approx(0.02));
    CHECK(m.duration_tolerance_s == Catch::Approx(86400.0));
    const auto p = config::load_refuelling_campaign_flown_mission(config_dir, "refuelling_campaign_flown_placed");
    CHECK(p.depot_placed);
    CHECK(p.placed_shortfall_tankers == Catch::Approx(1.5));
    CHECK_THROWS(config::load_refuelling_campaign_flown_mission(config_dir, "tanker_flight"));  // wrong kind
}

TEST_CASE("refuelling_campaign_flown refuses bad values", "[refuelling][config][campaign_flown]") {
    const std::string base = R"([mission]
name = "x"
kind = "refuelling_campaign_flown"
[depot]
start = "%START%"
%SHORTFALL%
[station_keeping]
min_height_km = 195.0
target_height_km = %TARGET%
min_lead_h = 24.0
engine_groups = %GROUPS%
[flights]
return_first_tanker = true
[comparison]
fill_tolerance = %FILL%
duration_days = 1.0
)";
    const auto text = [&](std::string start, std::string shortfall, std::string target, std::string groups,
                          std::string fill) {
        std::string t = base;
        const auto put = [&](const std::string& key, const std::string& value) {
            t.replace(t.find(key), key.size(), value);
        };
        put("%START%", start);
        put("%SHORTFALL%", shortfall);
        put("%TARGET%", target);
        put("%GROUPS%", groups);
        put("%FILL%", fill);
        return t;
    };
    const auto load = [&](const std::string& name, const std::string& t) {
        return config::load_refuelling_campaign_flown_mission(with_mission("f2_" + name, "c", t), "c");
    };
    CHECK_NOTHROW(load("ok", text("launch", "", "200.0", "[\"sea_level_3\"]", "0.02")));
    CHECK_THROWS(load("target", text("launch", "", "195.0", "[\"sea_level_3\"]", "0.02")));  // not above the minimum
    CHECK_THROWS(load("fill", text("launch", "", "200.0", "[\"sea_level_3\"]", "-0.02")));
    CHECK_THROWS(load("start", text("orbit", "", "200.0", "[\"sea_level_3\"]", "0.02")));
    CHECK_THROWS(load("shortfall", text("placed", "", "200.0", "[\"sea_level_3\"]", "0.02")));  // placed needs it
    CHECK_NOTHROW(load("placed", text("placed", "shortfall_tankers = 1.5", "200.0", "[\"sea_level_3\"]", "0.02")));
    CHECK_THROWS(load("groups", text("launch", "", "200.0", "[]", "0.02")));
}
