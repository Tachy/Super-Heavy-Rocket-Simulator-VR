// Mission selection panel, Task 2: every selectable mission declares where it starts and with which vehicle
// ([mission] start_place / start_vehicle); the declarations must agree with what sim::start_mission puts in front of
// the player. Reads the configs and the kind loaders only - no flights.

#include <algorithm>
#include <filesystem>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <toml++/toml.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/refuelling_mission_config.hpp"

using namespace starsim;

namespace {

struct Declared {
    std::string id, place, vehicle;
};

const std::set<std::string> known_follow_ons = {"ship_return_starbase", "ship_return_starbase_crewed", "leo_ascent"};
const std::set<std::string> not_a_start_vehicle = {"ejection_seat", "parachutist", "iss"};

std::filesystem::path config_dir() { return std::filesystem::path(STARSIM_CONFIG_DIR); }

toml::table parse(const std::filesystem::path& path) {
    REQUIRE(std::filesystem::exists(path));
    return toml::parse_file(path.string());
}

std::vector<std::string> mission_ids() {
    std::vector<std::string> ids;
    for (const auto& e : std::filesystem::directory_iterator(config_dir() / "missions"))
        if (e.path().extension() == ".toml") ids.push_back(e.path().stem().string());
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<Declared> declared_starts() {
    std::vector<Declared> out;
    for (const auto& id : mission_ids()) {
        const auto t = parse(config_dir() / "missions" / (id + ".toml"));
        const auto place = t["mission"]["start_place"].value<std::string>();
        const auto vehicle = t["mission"]["start_vehicle"].value<std::string>();
        if (place && vehicle) out.push_back({id, *place, *vehicle});
    }
    return out;
}

std::set<std::string> pad_ids() {
    std::set<std::string> ids;
    for (const auto& s : config::load_launch_sites(STARSIM_CONFIG_DIR))
        if (s.kind != "landing_zone") ids.insert(s.id);
    return ids;
}

std::set<std::string> place_ids() {
    std::set<std::string> ids;
    const auto t = parse(config_dir() / "start_places.toml");
    const auto* places = t["place"].as_array();
    REQUIRE(places != nullptr);
    for (const auto& n : *places) ids.insert(*n.as_table()->at("id").value<std::string>());
    return ids;
}

// (site, vehicle) of an ascent mission
std::pair<std::string, std::string> ascent_start(const std::string& ascent_id) {
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, ascent_id);
    return {m.site_id, m.vehicle_id};
}

}  // namespace

TEST_CASE("Every mission file declares a start or is a known follow-on", "[mission][config]") {
    for (const auto& id : mission_ids()) {
        const auto t = parse(config_dir() / "missions" / (id + ".toml"));
        const bool place = t["mission"]["start_place"].is_string();
        const bool vehicle = t["mission"]["start_vehicle"].is_string();
        INFO(id);
        if (known_follow_ons.count(id)) {
            CHECK_FALSE(place);
            CHECK_FALSE(vehicle);
        } else {
            CHECK(place);
            CHECK(vehicle);
        }
    }
}

TEST_CASE("Start places and vehicles exist", "[mission][config]") {
    const auto pads = pad_ids();
    const auto places = place_ids();
    for (const auto& d : declared_starts()) {
        INFO(d.id);
        CHECK((pads.count(d.place) == 1 || places.count(d.place) == 1));
        CHECK(std::filesystem::exists(config_dir() / "vehicles" / (d.vehicle + ".toml")));
        CHECK(not_a_start_vehicle.count(d.vehicle) == 0);
    }
}

TEST_CASE("The declared start matches what start_mission starts", "[mission][config]") {
    using config::MissionKind;
    for (const auto& d : declared_starts()) {
        INFO(d.id);
        std::string place, vehicle;
        switch (config::mission_kind(STARSIM_CONFIG_DIR, d.id)) {
            case MissionKind::Ascent: {
                const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, d.id);
                place = m.site_id;
                vehicle = m.vehicle_id;
                break;
            }
            case MissionKind::IssLaunch:
                std::tie(place, vehicle) =
                    ascent_start(config::load_iss_launch_mission(STARSIM_CONFIG_DIR, d.id).ascent_id);
                break;
            case MissionKind::DepotLaunch:
                std::tie(place, vehicle) =
                    ascent_start(config::load_depot_launch_mission(STARSIM_CONFIG_DIR, d.id).ascent_id);
                break;
            case MissionKind::TankerFlight:
            case MissionKind::HlsRefuel: {
                const auto m = config::load_refuelling_flight_mission(STARSIM_CONFIG_DIR, d.id);
                std::tie(place, vehicle) = ascent_start(m.ascent_id);
                if (m.start != config::RefuellingStart::Pad) place = "earth_orbit";
                break;
            }
            case MissionKind::RefuellingCampaignFlown: {
                const auto m = config::load_refuelling_campaign_flown_mission(STARSIM_CONFIG_DIR, d.id);
                if (m.depot_placed) {
                    place = "earth_orbit";
                    vehicle = "starship_depot";
                } else {  // the depot of the campaign is the reference flight's depot_launch
                    const auto launch = config::load_depot_launch_mission(STARSIM_CONFIG_DIR, "depot_launch");
                    std::tie(place, vehicle) = ascent_start(launch.ascent_id);
                }
                break;
            }
            case MissionKind::CapsuleReturn:
                place = "earth_orbit";
                vehicle = config::load_capsule_return_mission(STARSIM_CONFIG_DIR, d.id).vehicle_id;
                break;
            case MissionKind::IssRendezvous:
                place = "earth_orbit";
                vehicle = config::load_iss_rendezvous_mission(STARSIM_CONFIG_DIR, d.id).vehicle_id;
                break;
            case MissionKind::IssDeparture:
                place = "iss_docked";
                vehicle = config::load_iss_departure_mission(STARSIM_CONFIG_DIR, d.id).vehicle_id;
                break;
            case MissionKind::LunarLanding:
                place = "earth_orbit";
                vehicle = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, d.id).vehicle_id;
                break;
            case MissionKind::LunarReturn:
                place = "lunar_orbit";
                vehicle = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, d.id).vehicle_id;
                break;
            case MissionKind::ShipReturn:
                FAIL("a ship return is not started, it declares no start");
        }
        CHECK(d.place == place);
        CHECK(d.vehicle == vehicle);
    }
}

TEST_CASE("Every pad lists vehicles that exist and may stand there", "[mission][config]") {
    const std::set<std::string> never = {"ejection_seat", "parachutist", "iss", "crew_dragon", "starship_hls"};
    const auto t = parse(config_dir() / "launch_sites.toml");
    const auto* sites = t["site"].as_array();
    REQUIRE(sites != nullptr);
    for (const auto& n : *sites) {
        const auto& site = *n.as_table();
        INFO(*site["id"].value<std::string>());
        const auto* list = site["vehicles"].as_array();
        if (site["kind"].is_string()) {  // a landing zone: no vehicles start there
            CHECK(list == nullptr);
            continue;
        }
        REQUIRE(list != nullptr);
        CHECK_FALSE(list->empty());
        for (const auto& v : *list) {
            const auto vehicle = *v.value<std::string>();
            INFO(vehicle);
            CHECK(std::filesystem::exists(config_dir() / "vehicles" / (vehicle + ".toml")));
            CHECK(never.count(vehicle) == 0);
        }
    }
}
