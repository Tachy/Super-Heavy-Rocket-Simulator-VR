#pragma once

// Refuelling r5: the reference flights' mission files - depot_launch, tanker_flight, hls_refuel (config/missions/).

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "starsim/config/mission_config.hpp"
#include "starsim/time/utc_date_time.hpp"

namespace starsim::config {

// depot_launch: the ascent of the depot stack (its ascent mission: vehicle, site, target orbit) at epoch_utc.
struct DepotLaunchMission {
    std::string id, name, ascent_id;
    time::UtcDateTime epoch_utc;
};

// The depot of a refuelling flight, placed on a synthetic circular orbit (mean elements advanced by J2).
struct DepotOrbitConfig {
    std::string vehicle_id = "starship_depot";
    std::string port = "transfer_dock";
    double height_m = 0.0;                  // circular
    double inclination_rad = 0.0;           // true equator frame (as the synthetic ISS)
    double raan_rad = 0.0;                  // at earliest_utc; hls_refuel: computed from the lunar plane
    double argument_of_latitude_rad = 0.0;  // at earliest_utc
    double propellant_kg = 0.0;             // main tanks, split at the ratio of their capacities
};

enum class RefuellingStart { Pad, PhasingOrbit };

// tanker_flight / hls_refuel: the chaser from the pad (launch window) or from the phasing orbit behind the depot.
// The phasing orbit's apsides are the ascent mission's target, so both starts share one orbit.
struct RefuellingFlightMission {
    std::string id, name;
    MissionKind kind = MissionKind::TankerFlight;  // or HlsRefuel
    std::string ascent_id;                         // the chaser's ascent mission (vehicle, site, target orbit)
    std::string port = "transfer_dock";            // the chaser's port
    DepotOrbitConfig depot;
    time::UtcDateTime earliest_utc;
    RefuellingStart start = RefuellingStart::Pad;
    double phasing_behind_rad = 0.0;   // PhasingOrbit: behind the depot, in its plane
    double start_propellant_kg = 0.0;  // PhasingOrbit: main tanks
    IssLaunchWindowConfig window;      // Pad (insertion = liftoff -> orbit insertion)
    double mission_limit_s = 0.0;
    double earliest_arrival_s = 0.0;
    // [phasing] periapsis_km: the phasing orbit's perigee when the ascent inserts lower (user 2026-10-20: the tanker
    // at 120 km) - raised at the first apoapsis; absent: the ascent's target is the phasing orbit.
    std::optional<double> phasing_periapsis_height_m;
    // [phasing] candidates_km (refuelling f1): phasing orbits (periapsis, apoapsis heights m) the launch window chooses
    // from by their cost with planning_mass_t / planning_boil_off_t_per_day (required with the candidates)
    std::vector<std::pair<double, double>> phasing_candidates_m;
    double planning_mass_kg = 0.0;
    double planning_boil_off_kgps = 0.0;
    double window_margin_s = 0.0;  // [phasing] window_margin_h: the window closes this much before the limit (f1)
    std::string return_id;            // tanker_flight: the ship_return mission whose [deorbit] block is flown
    std::string lunar_mission_id;      // hls_refuel: the lunar landing mission continued after the refuelling
};

DepotLaunchMission load_depot_launch_mission(const std::filesystem::path& config_dir, std::string_view mission_id);
// Kinds tanker_flight and hls_refuel; ConfigError (via detail::Document) for anything else or bad values.
RefuellingFlightMission load_refuelling_flight_mission(const std::filesystem::path& config_dir,
                                                       std::string_view mission_id);

}  // namespace starsim::config
