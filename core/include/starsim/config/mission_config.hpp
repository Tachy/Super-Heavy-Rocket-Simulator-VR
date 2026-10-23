#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"
#include "starsim/time/utc_date_time.hpp"

namespace starsim::config {

// [mission] kind: what the mission file describes (FlightSim.start_mission dispatches on it). Absent: "ascent".
enum class MissionKind {
    Ascent, ShipReturn, CapsuleReturn, LunarLanding, LunarReturn, IssRendezvous, IssLaunch, IssDeparture,
    DepotLaunch, TankerFlight, HlsRefuel,  // refuelling r5 (refuelling_mission_config.hpp)
    RefuellingCampaignFlown                // refuelling f2
};
MissionKind mission_kind(const std::filesystem::path& config_dir, std::string_view mission_id);
// [mission] display_name (German, no trademarks: the webcast overlay); empty when the file has none.
std::string mission_display_name(const std::filesystem::path& config_dir, std::string_view mission_id);

// [deorbit] of a ship return from orbit (b4): the deorbit planning settings (gnc::DeorbitParameters).
struct ShipReturnDeorbit {
    double entry_interface_height_m = 0.0;
    double entry_flight_path_rad = 0.0;
    double nominal_bank_rad = 0.0;
    double max_crossrange_m = 0.0;
    double search_window_s = 0.0;
};

// Ship return from orbit (config/missions/<id>.toml): the scenario's start orbit and the deorbit planning settings.
struct ShipReturnMission {
    std::string id;
    std::string name;
    std::string vehicle_id;
    std::string stage_id;  // the ship's stage (stages below it are left out)
    std::string site_id;
    time::UtcDateTime epoch_utc;
    double periapsis_height_m = 0.0;  // above the equatorial radius
    double apoapsis_height_m = 0.0;
    double inclination_rad = 0.0;
    double raan_rad = 0.0;
    double argument_of_latitude_rad = 0.0;
    double propellant_kg = 0.0;  // in orbit
    ShipReturnDeorbit deorbit;
    std::string crew_escape_id;  // [mission] crew_escape (config/crew_escape/): crewed - ejection seats; empty: uncrewed
};

ShipReturnMission load_ship_return_mission(const std::filesystem::path& config_dir, std::string_view mission_id);

// Capsule return from orbit (config/missions/<id>.toml, d4): the start orbit, the splashdown zone and the deorbit
// planning settings (FlightSession::plan_capsule_return).
struct CapsuleReturnMission {
    std::string id;
    std::string name;
    std::string vehicle_id;
    std::string zone_id;  // config/splashdown_zones.toml
    time::UtcDateTime epoch_utc;
    double periapsis_height_m = 0.0;  // above the equatorial radius
    double apoapsis_height_m = 0.0;
    double inclination_rad = 0.0;
    double raan_rad = 0.0;
    double argument_of_latitude_rad = 0.0;
    double rcs_propellant_kg = 0.0;  // in the capsule's RCS tank at the start
    double entry_interface_height_m = 0.0;
    double entry_flight_path_rad = 0.0;
    double nominal_bank_rad = 0.0;
    double max_crossrange_m = 0.0;
    double search_window_s = 0.0;
};

CapsuleReturnMission load_capsule_return_mission(const std::filesystem::path& config_dir, std::string_view mission_id);

// HLS lunar landing (config/missions/<id>.toml, lunar l3: the outbound part; l4 adds descent, surface stay and
// ascent): the circular Earth parking orbit at the start (its RAAN is computed: plane through the Moon at arrival), the
// transfer settings and the lunar orbit (LLO or NRHO) the flight inserts into.
struct LunarLandingMission {
    std::string id;
    std::string name;
    std::string vehicle_id;
    std::string site_id;  // config/lunar_sites.toml (l4 lands there)
    time::UtcDateTime epoch_utc;  // start in the parking orbit
    double parking_height_m = 0.0;  // circular, above the equatorial radius
    double parking_inclination_rad = 0.0;
    double argument_of_latitude_rad = 0.0;
    double flight_time_s = 0.0;  // start -> arrival (LLO periselene / NRHO perilune the flight joins), planned
    double earliest_departure_s = 0.0;
    std::vector<double> mcc_after_tli_s;
    std::string lunar_orbit;  // "llo" | "nrho"
    double llo_altitude_m = 0.0;
    double llo_inclination_rad = 0.0;
    math::Vec3 nrho_apolune_position{}, nrho_apolune_velocity{};  // CR3BP seed (nondimensional)
    int nrho_revolutions = 0, nrho_synodic_months = 0, nrho_reference_revolutions = 0;
    // [descent] (lunar l4; defaults as mission::LunarDescentParameters)
    double pdi_altitude_above_site_m = 15.0e3;
    double braking_range_m = 450.0e3;
    double max_crossrange_m = 20.0e3;
    double slope_limit_rad = 8.0 * constants::deg_to_rad;
    double touchdown_search_radius_m = 1000.0;
    // [surface]
    double surface_stay_s = 0.0;
    // [ascent] (defaults as gnc::LunarAscentParameters)
    double insertion_altitude_above_site_m = 15.0e3;
    double ascent_aposelene_m = 100.0e3;
    // [budget]: nominal delta-v of the legs not planned at the start (the outbound legs come from its plan)
    double descent_delta_v_mps = 2100.0;
    double ascent_delta_v_mps = 1900.0;
    double nrho_to_llo_delta_v_mps = 876.0;
    double llo_to_nrho_delta_v_mps = 725.0;
    double reserve_kg = 10000.0;
};

LunarLandingMission load_lunar_landing_mission(const std::filesystem::path& config_dir, std::string_view mission_id);

// Ship return from lunar orbit (config/missions/<id>.toml, lunar l5): the start orbit (LLO or NRHO), the variant
// (direct entry / aerocapture) and the return planning settings. The propellant is computed by the start.
struct LunarReturnMission {
    std::string id, name, vehicle_id, stage_id, site_id;
    time::UtcDateTime epoch_utc;
    std::string orbit;  // "llo" | "nrho"
    double llo_altitude_m = 100e3;
    double llo_inclination_rad = 0.5 * constants::pi;  // Moon equator; the node is placed through the asymptote
    math::Vec3 nrho_apolune_position{}, nrho_apolune_velocity{};  // CR3BP seed (nondimensional)
    int nrho_revolutions = 0, nrho_synodic_months = 0, nrho_reference_revolutions = 0;
    bool aerocapture = false;  // [return] variant
    double flight_time_s = 3.5 * 86400.0;
    double search_s = 86400.0;
    std::vector<double> mcc_after_tei_s;
    double entry_interface_height_m = 120e3;
    double braking_height_m = 2000e3;
    double min_corridor_width_rad = 1.0 * constants::deg_to_rad;
    double nominal_bank_rad = 110.0 * constants::deg_to_rad;
    double aerocapture_apoapsis_m = 400e3;
    double perigee_raise_height_m = 200e3;
    ShipReturnDeorbit deorbit;  // [deorbit] (aerocapture): b4 after the capture
    double mcc_allowance_mps = 30.0;
    double coast_attitude_kg_per_day = 1000.0;
    double reserve_kg = 10000.0;
    // [mission] boil_off (refuelling r2): the session's boil-off fidelity flag for this mission (default true). The
    // Block 3 lunar missions set false until r5 (a bare hull cannot keep its propellant for days; user 2026-10-20).
    bool boil_off = true;
    std::string crew_escape_id;  // [mission] crew_escape (config/crew_escape/): crewed - ejection seats; empty: uncrewed
};

LunarReturnMission load_lunar_return_mission(const std::filesystem::path& config_dir, std::string_view mission_id);

// A payload vehicle on top of an ascent mission's stack ([payload]; config::stack_payload), released after the upper
// stage's cutoff.
struct MissionPayload {
    std::string vehicle_id;
    double separation_delay_s = 30.0;  // after the upper stage's cutoff
    double separation_dv_mps = 0.3;    // spring push (relative speed)
    double nose_cone_open_s = 600.0;   // after the separation; < 0: stays closed
};

// Launch to orbit on the ascent autopilot (config/missions/<id>.toml). The pitch program and the first stage's recovery
// (tower, landing zone or drone ship) are not configured: the onboard computer plans them at the start
// (sim::plan_launch); the loader refuses [pitch_program], kick_deg, booster_throttle, landing_zone and [drone_ship].
struct AscentMission {
    std::string id;
    std::string name;
    std::string vehicle_id;
    std::string site_id;
    double heading_rad = 0.0;     // body +y on the pad
    double payload_kg = 0.0;
    bool fairing = true;          // false: the payload flies without the upper stage's fairing (Dragon)
    std::optional<MissionPayload> payload;  // a payload vehicle on top of the stack (exclusive with payload_kg)
    double periapsis_height_m = 0.0;
    double apoapsis_height_m = 0.0;
    double inclination_rad = 0.0;
    bool northerly = true;
    // [mission] countdown_s: requested epoch -> T-0 when the mission waits on the pad (sim::StartOptions); none: the
    // start option's default
    std::optional<double> countdown_s;
};

AscentMission load_ascent_mission(const std::filesystem::path& config_dir, std::string_view mission_id);

// ISS missions (ISS i6). Common [mission] keys: station (vehicle id), port (docking port id on it), iss_orbit
// ("oem" | "synthetic" | "auto", sim::parse_iss_orbit_source).

// [launch_window] of an ISS launch (mission::LaunchWindowParameters).
struct IssLaunchWindowConfig {
    double insertion_s = 0.0;        // [D] liftoff -> payload separation
    double insertion_arc_rad = 0.0;  // [D] pad at liftoff -> separation point, inertial in-plane angle
    double search_s = 0.0;           // from earliest_utc
};

// Falcon 9 + Crew Dragon from the pad to the ISS: the launch window, the ascent mission `ascent_id`, then the docking.
struct IssLaunchMission {
    std::string id;
    std::string name;
    std::string ascent_id;  // an ascent mission with the Dragon as payload
    std::string station_id;
    std::string port;
    std::string iss_orbit;
    time::UtcDateTime earliest_utc;
    IssLaunchWindowConfig window;
    double mission_limit_s = 0.0;     // liftoff -> Waypoint 0
    double earliest_arrival_s = 0.0;  // 0: the earliest window
};

// Crew Dragon from its separation orbit to the ISS: the separation of the first window of `launch_id` after epoch_utc,
// or (phase_rad) a start that far behind the ISS at epoch_utc.
struct IssRendezvousMission {
    std::string id;
    std::string name;
    std::string vehicle_id;
    std::string station_id;
    std::string port;
    std::string iss_orbit;
    std::string launch_id;
    time::UtcDateTime epoch_utc;
    std::optional<double> phase_rad;
    std::optional<double> iss_argument_of_latitude_rad;  // synthetic orbit only
    double mission_limit_s = 0.0;
    double earliest_arrival_s = 0.0;
};

// Crew Dragon docked at the ISS: undocking timed to a return pass, departure, the capsule return `return_id`.
struct IssDepartureMission {
    std::string id;
    std::string name;
    std::string vehicle_id;
    std::string station_id;
    std::string port;
    std::string iss_orbit;
    std::string return_id;  // a capsule_return mission (zone, deorbit settings)
    time::UtcDateTime epoch_utc;  // the earliest undocking
    double rcs_propellant_kg = 0.0;
    double undock_to_deorbit_s = 0.0;
    double search_window_s = 0.0;  // of the return-pass search
    std::optional<double> iss_argument_of_latitude_rad;
};

IssLaunchMission load_iss_launch_mission(const std::filesystem::path& config_dir, std::string_view mission_id);
IssRendezvousMission load_iss_rendezvous_mission(const std::filesystem::path& config_dir, std::string_view mission_id);
IssDepartureMission load_iss_departure_mission(const std::filesystem::path& config_dir, std::string_view mission_id);

}  // namespace starsim::config
