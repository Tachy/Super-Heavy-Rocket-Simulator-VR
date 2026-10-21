#include "starsim/config/mission_config.hpp"
#include "starsim/config/refuelling_mission_config.hpp"

#include <cmath>
#include <cstdint>

#include "starsim/math/constants.hpp"
#include "toml_document.hpp"

namespace starsim::config {

namespace {

MissionKind kind_of(const detail::Document& doc) {
    const auto* mission = doc.root["mission"].as_table();
    if (!mission) doc.fail("missing [mission]");
    const auto kind = doc.text_or(*mission, "kind", "ascent");
    if (kind == "ascent") return MissionKind::Ascent;
    if (kind == "ship_return") return MissionKind::ShipReturn;
    if (kind == "capsule_return") return MissionKind::CapsuleReturn;
    if (kind == "lunar_landing") return MissionKind::LunarLanding;
    if (kind == "lunar_return") return MissionKind::LunarReturn;
    if (kind == "iss_rendezvous") return MissionKind::IssRendezvous;
    if (kind == "iss_launch") return MissionKind::IssLaunch;
    if (kind == "iss_departure") return MissionKind::IssDeparture;
    if (kind == "depot_launch") return MissionKind::DepotLaunch;
    if (kind == "tanker_flight") return MissionKind::TankerFlight;
    if (kind == "hls_refuel") return MissionKind::HlsRefuel;
    doc.fail("mission: unknown kind '" + kind +
             "' (ascent, ship_return, capsule_return, lunar_landing, lunar_return, iss_rendezvous, iss_launch, "
             "iss_departure, depot_launch, tanker_flight, hls_refuel)");
}

// [mission] station / port / iss_orbit of an ISS mission.
void iss_common(const detail::Document& doc, const toml::table& mission, std::string& station, std::string& port,
                std::string& iss_orbit) {
    station = doc.text(mission, "station");
    port = doc.text(mission, "port");
    iss_orbit = doc.text(mission, "iss_orbit");
    if (iss_orbit != "oem" && iss_orbit != "synthetic" && iss_orbit != "auto") {
        doc.fail("mission: iss_orbit '" + iss_orbit + "' (oem, synthetic, auto)");
    }
}

time::UtcDateTime utc(const detail::Document& doc, const toml::table& t, std::string_view key) {
    const auto parsed = time::UtcDateTime::parse_iso(doc.text(t, key));
    if (!parsed) doc.fail(std::string(key) + " is not YYYY-MM-DDTHH:MM:SS");
    return *parsed;
}

std::optional<double> optional_deg(const detail::Document& doc, const toml::table& t, std::string_view key) {
    if (!t.contains(key)) return std::nullopt;
    return doc.number(t, key) * constants::deg_to_rad;
}

void require_kind(const detail::Document& doc, MissionKind kind, const char* name) {
    if (kind_of(doc) != kind) doc.fail(std::string("not a ") + name + " mission ([mission] kind)");
}

// [E] refuelling f1: a phasing orbit below this decays within hours
constexpr double min_phasing_height_km = 100.0;

// A TOML array of numbers (`size` entries when size > 0).
std::vector<double> numbers(const detail::Document& doc, const toml::table& t, std::string_view key, size_t size = 0) {
    const auto* array = t[key].as_array();
    if (!array) doc.fail("missing number array '" + std::string(key) + "'");
    std::vector<double> values;
    for (const auto& node : *array) {
        const auto v = node.value<double>();
        if (!v) doc.fail("'" + std::string(key) + "' must contain numbers only");
        values.push_back(*v);
    }
    if (size > 0 && values.size() != size) doc.fail("'" + std::string(key) + "' needs " + std::to_string(size) + " numbers");
    return values;
}

math::Vec3 vec3(const detail::Document& doc, const toml::table& t, std::string_view key) {
    const auto v = numbers(doc, t, key, 3);
    return {v[0], v[1], v[2]};
}

int positive_integer(const detail::Document& doc, const toml::table& t, std::string_view key) {
    const auto v = t[key].value<int64_t>();
    if (!v || *v <= 0) doc.fail("'" + std::string(key) + "' must be a positive integer");
    return static_cast<int>(*v);
}

// [deorbit] of a ship return (b4)
ShipReturnDeorbit ship_return_deorbit(const detail::Document& doc, const toml::table& deorbit) {
    const double deg = constants::deg_to_rad;
    ShipReturnDeorbit d;
    d.entry_interface_height_m = doc.positive(deorbit, "entry_interface_km") * 1000.0;
    d.entry_flight_path_rad = doc.number(deorbit, "entry_flight_path_deg") * deg;
    if (!(d.entry_flight_path_rad < 0.0)) doc.fail("deorbit: entry_flight_path_deg must be negative");
    d.nominal_bank_rad = doc.positive(deorbit, "nominal_bank_deg") * deg;
    d.max_crossrange_m = doc.positive(deorbit, "max_crossrange_km") * 1000.0;
    d.search_window_s = doc.positive(deorbit, "search_window_h") * 3600.0;
    return d;
}

// Burn times in days after a reference, positive and increasing (seconds).
std::vector<double> increasing_days(const detail::Document& doc, const toml::table& t, std::string_view key) {
    std::vector<double> seconds;
    for (const double days : numbers(doc, t, key)) {
        if (!(days > 0.0) || (!seconds.empty() && days * 86400.0 <= seconds.back())) {
            doc.fail("'" + std::string(key) + "' must be positive and increasing");
        }
        seconds.push_back(days * 86400.0);
    }
    return seconds;
}

}  // namespace

MissionKind mission_kind(const std::filesystem::path& config_dir, std::string_view mission_id) {
    return kind_of(detail::Document(config_dir / "missions" / (std::string(mission_id) + ".toml")));
}

ShipReturnMission load_ship_return_mission(const std::filesystem::path& config_dir, std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    require_kind(doc, MissionKind::ShipReturn, "ship return");
    const double deg = constants::deg_to_rad;
    const auto* mission = doc.root["mission"].as_table();
    const auto* orbit = doc.root["orbit"].as_table();
    const auto* deorbit = doc.root["deorbit"].as_table();
    if (!mission || !orbit || !deorbit) doc.fail("missing [mission], [orbit] or [deorbit]");
    ShipReturnMission m;
    m.id = std::string(mission_id);
    m.name = doc.text(*mission, "name");
    m.vehicle_id = doc.text(*mission, "vehicle");
    m.stage_id = doc.text(*mission, "stage");
    m.site_id = doc.text(*mission, "site");
    const auto epoch = time::UtcDateTime::parse_iso(doc.text(*orbit, "epoch_utc"));
    if (!epoch) doc.fail("orbit: epoch_utc is not YYYY-MM-DDTHH:MM:SS");
    m.epoch_utc = *epoch;
    m.periapsis_height_m = doc.positive(*orbit, "periapsis_km") * 1000.0;
    m.apoapsis_height_m = doc.positive(*orbit, "apoapsis_km") * 1000.0;
    if (m.apoapsis_height_m < m.periapsis_height_m) doc.fail("orbit: apoapsis below periapsis");
    m.inclination_rad = doc.number(*orbit, "inclination_deg") * deg;
    m.raan_rad = doc.number(*orbit, "raan_deg") * deg;
    m.argument_of_latitude_rad = doc.number(*orbit, "argument_of_latitude_deg") * deg;
    m.propellant_kg = doc.positive(*orbit, "propellant_t") * 1000.0;
    m.deorbit = ship_return_deorbit(doc, *deorbit);
    return m;
}

CapsuleReturnMission load_capsule_return_mission(const std::filesystem::path& config_dir, std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    require_kind(doc, MissionKind::CapsuleReturn, "capsule return");
    const double deg = constants::deg_to_rad;
    const auto* mission = doc.root["mission"].as_table();
    const auto* orbit = doc.root["orbit"].as_table();
    const auto* deorbit = doc.root["deorbit"].as_table();
    if (!mission || !orbit || !deorbit) doc.fail("missing [mission], [orbit] or [deorbit]");
    CapsuleReturnMission m;
    m.id = std::string(mission_id);
    m.name = doc.text(*mission, "name");
    m.vehicle_id = doc.text(*mission, "vehicle");
    m.zone_id = doc.text(*mission, "zone");
    const auto epoch = time::UtcDateTime::parse_iso(doc.text(*orbit, "epoch_utc"));
    if (!epoch) doc.fail("orbit: epoch_utc is not YYYY-MM-DDTHH:MM:SS");
    m.epoch_utc = *epoch;
    m.periapsis_height_m = doc.positive(*orbit, "periapsis_km") * 1000.0;
    m.apoapsis_height_m = doc.positive(*orbit, "apoapsis_km") * 1000.0;
    if (m.apoapsis_height_m < m.periapsis_height_m) doc.fail("orbit: apoapsis below periapsis");
    m.inclination_rad = doc.number(*orbit, "inclination_deg") * deg;
    m.raan_rad = doc.number(*orbit, "raan_deg") * deg;
    m.argument_of_latitude_rad = doc.number(*orbit, "argument_of_latitude_deg") * deg;
    m.rcs_propellant_kg = doc.positive(*orbit, "rcs_propellant_kg");
    m.entry_interface_height_m = doc.positive(*deorbit, "entry_interface_km") * 1000.0;
    m.entry_flight_path_rad = doc.number(*deorbit, "entry_flight_path_deg") * deg;
    if (!(m.entry_flight_path_rad < 0.0)) doc.fail("deorbit: entry_flight_path_deg must be negative");
    m.nominal_bank_rad = doc.positive(*deorbit, "nominal_bank_deg") * deg;
    m.max_crossrange_m = doc.positive(*deorbit, "max_crossrange_km") * 1000.0;
    m.search_window_s = doc.positive(*deorbit, "search_window_h") * 3600.0;
    return m;
}

LunarLandingMission load_lunar_landing_mission(const std::filesystem::path& config_dir, std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    require_kind(doc, MissionKind::LunarLanding, "lunar landing");
    const double deg = constants::deg_to_rad;
    const auto& mission = doc.table(doc.root, "mission");
    const auto& orbit = doc.table(doc.root, "orbit");
    const auto& transfer = doc.table(doc.root, "transfer");
    const auto& lunar_orbit = doc.table(doc.root, "lunar_orbit");
    LunarLandingMission m;
    m.id = std::string(mission_id);
    m.name = doc.text(mission, "name");
    m.vehicle_id = doc.text(mission, "vehicle");
    m.site_id = doc.text(mission, "site");
    const auto epoch = time::UtcDateTime::parse_iso(doc.text(orbit, "epoch_utc"));
    if (!epoch) doc.fail("orbit: epoch_utc is not YYYY-MM-DDTHH:MM:SS");
    m.epoch_utc = *epoch;
    m.parking_height_m = doc.positive(orbit, "height_km") * 1000.0;
    m.parking_inclination_rad = doc.number(orbit, "inclination_deg") * deg;
    m.argument_of_latitude_rad = doc.number(orbit, "argument_of_latitude_deg") * deg;
    m.flight_time_s = doc.positive(transfer, "flight_time_days") * 86400.0;
    m.earliest_departure_s = doc.number(transfer, "earliest_departure_h") * 3600.0;
    if (m.earliest_departure_s < 0.0) doc.fail("transfer: earliest_departure_h must be >= 0");
    for (const double days : numbers(doc, transfer, "mcc_after_tli_days")) {
        if (!(days > 0.0) || (!m.mcc_after_tli_s.empty() && days * 86400.0 <= m.mcc_after_tli_s.back())) {
            doc.fail("transfer: mcc_after_tli_days must be positive and increasing");
        }
        m.mcc_after_tli_s.push_back(days * 86400.0);
    }
    m.lunar_orbit = doc.text(lunar_orbit, "kind");
    if (m.lunar_orbit == "llo") {
        m.llo_altitude_m = doc.positive(lunar_orbit, "altitude_km") * 1000.0;
        m.llo_inclination_rad = doc.number(lunar_orbit, "inclination_deg") * deg;
    } else if (m.lunar_orbit == "nrho") {
        const auto& nrho = doc.table(doc.root, "nrho");
        m.nrho_apolune_position = vec3(doc, nrho, "apolune_position");
        m.nrho_apolune_velocity = vec3(doc, nrho, "apolune_velocity");
        m.nrho_revolutions = positive_integer(doc, nrho, "revolutions");
        m.nrho_synodic_months = positive_integer(doc, nrho, "synodic_months");
        m.nrho_reference_revolutions = positive_integer(doc, nrho, "reference_revolutions");
    } else {
        doc.fail("lunar_orbit: unknown kind '" + m.lunar_orbit + "' (llo, nrho)");
    }
    const auto& descent = doc.table(doc.root, "descent");
    m.pdi_altitude_above_site_m = doc.positive(descent, "pdi_altitude_above_site_km") * 1000.0;
    m.braking_range_m = doc.positive(descent, "braking_range_km") * 1000.0;
    m.max_crossrange_m = doc.positive(descent, "max_crossrange_km") * 1000.0;
    m.slope_limit_rad = doc.positive(descent, "slope_limit_deg") * deg;
    m.touchdown_search_radius_m = doc.positive(descent, "touchdown_search_radius_m");
    m.surface_stay_s = doc.number(doc.table(doc.root, "surface"), "stay_hours") * 3600.0;
    if (m.surface_stay_s < 0.0) doc.fail("surface: stay_hours must be >= 0");
    const auto& ascent = doc.table(doc.root, "ascent");
    m.insertion_altitude_above_site_m = doc.positive(ascent, "insertion_altitude_above_site_km") * 1000.0;
    m.ascent_aposelene_m = doc.positive(ascent, "aposelene_km") * 1000.0;
    const auto& budget = doc.table(doc.root, "budget");
    m.descent_delta_v_mps = doc.positive(budget, "descent_delta_v_mps");
    m.ascent_delta_v_mps = doc.positive(budget, "ascent_delta_v_mps");
    m.nrho_to_llo_delta_v_mps = doc.positive(budget, "nrho_to_llo_delta_v_mps");
    m.llo_to_nrho_delta_v_mps = doc.positive(budget, "llo_to_nrho_delta_v_mps");
    m.reserve_kg = doc.positive(budget, "reserve_kg");
    return m;
}

LunarReturnMission load_lunar_return_mission(const std::filesystem::path& config_dir, std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    require_kind(doc, MissionKind::LunarReturn, "lunar return");
    const double deg = constants::deg_to_rad;
    const auto& mission = doc.table(doc.root, "mission");
    const auto& orbit = doc.table(doc.root, "orbit");
    const auto& ret = doc.table(doc.root, "return");
    const auto& budget = doc.table(doc.root, "budget");
    LunarReturnMission m;
    m.id = std::string(mission_id);
    m.name = doc.text(mission, "name");
    m.vehicle_id = doc.text(mission, "vehicle");
    m.stage_id = doc.text(mission, "stage");
    m.site_id = doc.text(mission, "site");
    m.boil_off = mission["boil_off"].value_or(true);
    const auto epoch = time::UtcDateTime::parse_iso(doc.text(orbit, "epoch_utc"));
    if (!epoch) doc.fail("orbit: epoch_utc is not YYYY-MM-DDTHH:MM:SS");
    m.epoch_utc = *epoch;
    m.orbit = doc.text(orbit, "kind");
    if (m.orbit == "llo") {
        m.llo_altitude_m = doc.positive(orbit, "altitude_km") * 1000.0;
        m.llo_inclination_rad = doc.number(orbit, "inclination_deg") * deg;
        if (m.llo_inclination_rad < 0.0 || m.llo_inclination_rad > constants::pi) {
            doc.fail("orbit: inclination_deg must lie in [0, 180]");
        }
    } else if (m.orbit == "nrho") {
        const auto& nrho = doc.table(doc.root, "nrho");
        m.nrho_apolune_position = vec3(doc, nrho, "apolune_position");
        m.nrho_apolune_velocity = vec3(doc, nrho, "apolune_velocity");
        m.nrho_revolutions = positive_integer(doc, nrho, "revolutions");
        m.nrho_synodic_months = positive_integer(doc, nrho, "synodic_months");
        m.nrho_reference_revolutions = positive_integer(doc, nrho, "reference_revolutions");
    } else {
        doc.fail("orbit: unknown kind '" + m.orbit + "' (llo, nrho)");
    }
    const auto variant = doc.text(ret, "variant");
    if (variant != "direct" && variant != "aerocapture") {
        doc.fail("return: unknown variant '" + variant + "' (direct, aerocapture)");
    }
    m.aerocapture = variant == "aerocapture";
    m.flight_time_s = doc.positive(ret, "flight_time_days") * 86400.0;
    m.search_s = doc.positive(ret, "search_days") * 86400.0;
    m.mcc_after_tei_s = increasing_days(doc, ret, "mcc_after_tei_days");
    m.entry_interface_height_m = doc.positive(ret, "entry_interface_km") * 1000.0;
    m.braking_height_m = doc.positive(ret, "braking_height_km") * 1000.0;
    m.min_corridor_width_rad = doc.positive(ret, "min_corridor_width_deg") * deg;
    m.nominal_bank_rad = doc.positive(ret, "nominal_bank_deg") * deg;
    if (m.aerocapture) {
        m.aerocapture_apoapsis_m = doc.positive(ret, "aerocapture_apoapsis_km") * 1000.0;
        m.perigee_raise_height_m = doc.positive(ret, "perigee_raise_km") * 1000.0;
        m.deorbit = ship_return_deorbit(doc, doc.table(doc.root, "deorbit"));
    }
    m.mcc_allowance_mps = doc.number(budget, "mcc_allowance_mps");
    if (m.mcc_allowance_mps < 0.0) doc.fail("budget: mcc_allowance_mps must be >= 0");
    m.coast_attitude_kg_per_day = doc.number(budget, "coast_attitude_kg_per_day");
    if (m.coast_attitude_kg_per_day < 0.0) doc.fail("budget: coast_attitude_kg_per_day must be >= 0");
    m.reserve_kg = doc.positive(budget, "reserve_kg");
    return m;
}

AscentMission load_ascent_mission(const std::filesystem::path& config_dir, std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    require_kind(doc, MissionKind::Ascent, "ascent");
    const double deg = constants::deg_to_rad;
    const auto* mission = doc.root["mission"].as_table();
    const auto* target = doc.root["target"].as_table();
    const auto* program = doc.root["pitch_program"].as_table();
    if (!mission || !target || !program) doc.fail("missing [mission], [target] or [pitch_program]");
    AscentMission m;
    m.id = std::string(mission_id);
    m.name = doc.text(*mission, "name");
    m.vehicle_id = doc.text(*mission, "vehicle");
    m.site_id = doc.text(*mission, "site");
    m.landing_zone_id = doc.text_or(*mission, "landing_zone");
    if (const auto* ship = doc.root["drone_ship"].as_table()) {
        if (!m.landing_zone_id.empty()) doc.fail("a mission has either a landing_zone or a [drone_ship]");
        MissionDroneShip d;
        d.id = doc.text(*ship, "id");
        d.latitude_rad = doc.number(*ship, "latitude_deg") * deg;
        d.longitude_rad = doc.number(*ship, "longitude_deg") * deg;
        d.heading_rad = doc.number(*ship, "heading_deg") * deg;
        d.sea_height_m = doc.number(*ship, "sea_height_m");
        m.drone_ship = d;
    }
    m.heading_rad = doc.number(*mission, "heading_deg") * deg;
    m.payload_kg = doc.number_or(*mission, "payload_t", 0.0) * 1000.0;
    m.fairing = (*mission)["fairing"].value_or(true);
    if (const auto* payload = doc.root["payload"].as_table()) {
        if (m.payload_kg > 0.0) doc.fail("a mission has either payload_t or a [payload] vehicle");
        MissionPayload p;
        p.vehicle_id = doc.text(*payload, "vehicle");
        p.separation_delay_s = doc.number_or(*payload, "separation_delay_s", p.separation_delay_s);
        p.separation_dv_mps = doc.number_or(*payload, "separation_dv_mps", p.separation_dv_mps);
        p.nose_cone_open_s = doc.number_or(*payload, "nose_cone_open_s", p.nose_cone_open_s);
        if (p.separation_delay_s < 0.0 || p.separation_dv_mps < 0.0) doc.fail("[payload] times and speeds must be >= 0");
        m.payload = p;
    }
    m.periapsis_height_m = doc.positive(*target, "periapsis_km") * 1000.0;
    m.apoapsis_height_m = doc.positive(*target, "apoapsis_km") * 1000.0;
    if (m.apoapsis_height_m < m.periapsis_height_m) doc.fail("target: apoapsis below periapsis");
    m.inclination_rad = doc.number(*target, "inclination_deg") * deg;
    m.northerly = (*target)["northerly"].value_or(true);
    m.kick_angle_rad = doc.positive(*program, "kick_deg") * deg;
    if (const auto* points = (*program)["booster_throttle"].as_array()) {
        for (const auto& node : *points) {
            const auto* pair = node.as_array();
            const auto t = pair && pair->size() == 2 ? (*pair)[0].value<double>() : std::nullopt;
            const auto f = pair && pair->size() == 2 ? (*pair)[1].value<double>() : std::nullopt;
            if (!t || !f || *f <= 0.0 || *f > 1.0) doc.fail("booster_throttle entries must be [t_s, fraction in (0, 1]]");
            if (!m.booster_throttle.empty() && *t <= m.booster_throttle.back().first) {
                doc.fail("booster_throttle times must increase");
            }
            m.booster_throttle.emplace_back(*t, *f);
        }
    }
    return m;
}

IssRendezvousMission load_iss_rendezvous_mission(const std::filesystem::path& config_dir, std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    require_kind(doc, MissionKind::IssRendezvous, "ISS rendezvous");
    const auto* mission = doc.root["mission"].as_table();
    const auto* start = doc.root["start"].as_table();
    const auto* phasing = doc.root["phasing"].as_table();
    if (!mission || !start || !phasing) doc.fail("missing [mission], [start] or [phasing]");
    IssRendezvousMission m;
    m.id = std::string(mission_id);
    m.name = doc.text(*mission, "name");
    m.vehicle_id = doc.text(*mission, "vehicle");
    iss_common(doc, *mission, m.station_id, m.port, m.iss_orbit);
    m.epoch_utc = utc(doc, *start, "epoch_utc");
    m.launch_id = doc.text(*start, "launch");
    m.phase_rad = optional_deg(doc, *start, "phase_deg");
    m.iss_argument_of_latitude_rad = optional_deg(doc, *start, "iss_argument_of_latitude_deg");
    m.mission_limit_s = doc.positive(*phasing, "mission_limit_h") * 3600.0;
    m.earliest_arrival_s = doc.number(*phasing, "earliest_arrival_h") * 3600.0;
    return m;
}

IssLaunchMission load_iss_launch_mission(const std::filesystem::path& config_dir, std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    require_kind(doc, MissionKind::IssLaunch, "ISS launch");
    const auto* mission = doc.root["mission"].as_table();
    const auto* window = doc.root["launch_window"].as_table();
    const auto* phasing = doc.root["phasing"].as_table();
    if (!mission || !window || !phasing) doc.fail("missing [mission], [launch_window] or [phasing]");
    IssLaunchMission m;
    m.id = std::string(mission_id);
    m.name = doc.text(*mission, "name");
    m.ascent_id = doc.text(*mission, "ascent");
    iss_common(doc, *mission, m.station_id, m.port, m.iss_orbit);
    m.earliest_utc = utc(doc, *window, "earliest_utc");
    m.window.search_s = doc.positive(*window, "search_days") * 86400.0;
    m.window.insertion_s = doc.positive(*window, "insertion_s");
    m.window.insertion_arc_rad = doc.positive(*window, "insertion_arc_deg") * constants::deg_to_rad;
    m.mission_limit_s = doc.positive(*phasing, "mission_limit_h") * 3600.0;
    m.earliest_arrival_s = doc.number(*phasing, "earliest_arrival_h") * 3600.0;
    return m;
}

IssDepartureMission load_iss_departure_mission(const std::filesystem::path& config_dir, std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    require_kind(doc, MissionKind::IssDeparture, "ISS departure");
    const auto* mission = doc.root["mission"].as_table();
    const auto* start = doc.root["start"].as_table();
    if (!mission || !start) doc.fail("missing [mission] or [start]");
    IssDepartureMission m;
    m.id = std::string(mission_id);
    m.name = doc.text(*mission, "name");
    m.vehicle_id = doc.text(*mission, "vehicle");
    iss_common(doc, *mission, m.station_id, m.port, m.iss_orbit);
    m.return_id = doc.text(*mission, "return");
    m.epoch_utc = utc(doc, *start, "epoch_utc");
    m.rcs_propellant_kg = doc.positive(*start, "rcs_propellant_kg");
    m.undock_to_deorbit_s = doc.positive(*start, "undock_to_deorbit_h") * 3600.0;
    m.search_window_s = doc.positive(*start, "search_window_h") * 3600.0;
    m.iss_argument_of_latitude_rad = optional_deg(doc, *start, "iss_argument_of_latitude_deg");
    return m;
}

// Refuelling r5 (declared in refuelling_mission_config.hpp; here for the file's helpers).
DepotLaunchMission load_depot_launch_mission(const std::filesystem::path& config_dir, std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    require_kind(doc, MissionKind::DepotLaunch, "depot launch");
    const auto* mission = doc.root["mission"].as_table();
    const auto* start = doc.root["start"].as_table();
    if (!mission || !start) doc.fail("missing [mission] or [start]");
    DepotLaunchMission m;
    m.id = std::string(mission_id);
    m.name = doc.text(*mission, "name");
    m.ascent_id = doc.text(*mission, "ascent");
    m.epoch_utc = utc(doc, *start, "epoch_utc");
    return m;
}

RefuellingFlightMission load_refuelling_flight_mission(const std::filesystem::path& config_dir,
                                                       std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    const double deg = constants::deg_to_rad;
    RefuellingFlightMission m;
    m.id = std::string(mission_id);
    m.kind = kind_of(doc);
    if (m.kind != MissionKind::TankerFlight && m.kind != MissionKind::HlsRefuel) {
        doc.fail("not a tanker_flight / hls_refuel mission ([mission] kind)");
    }
    const auto* mission = doc.root["mission"].as_table();
    const auto* depot = doc.root["depot"].as_table();
    const auto* start = doc.root["start"].as_table();
    const auto* window = doc.root["launch_window"].as_table();
    const auto* phasing = doc.root["phasing"].as_table();
    if (!depot || !start || !window || !phasing) doc.fail("missing [depot], [start], [launch_window] or [phasing]");
    m.name = doc.text(*mission, "name");
    m.ascent_id = doc.text(*mission, "ascent");
    if (mission->contains("port")) {  // r5 minor: it was parsed and never used
        doc.fail("[mission] port is not used: the chaser docks with its first docking port ([depot] port names the "
                 "depot's)");
    }
    m.depot.vehicle_id = doc.text_or(*depot, "vehicle", "starship_depot");
    m.depot.port = doc.text_or(*depot, "port", "transfer_dock");
    m.depot.height_m = doc.positive(*depot, "height_km") * 1e3;
    m.depot.inclination_rad = doc.positive(*depot, "inclination_deg") * deg;
    m.depot.raan_rad = doc.number_or(*depot, "raan_deg", 0.0) * deg;
    m.depot.argument_of_latitude_rad = doc.number_or(*depot, "argument_of_latitude_deg", 0.0) * deg;
    m.depot.propellant_kg = doc.positive(*depot, "propellant_t") * 1e3;
    const auto mode = doc.text(*start, "mode");
    if (mode == "pad") {
        m.start = RefuellingStart::Pad;
    } else if (mode == "phasing_orbit") {
        m.start = RefuellingStart::PhasingOrbit;
        m.phasing_behind_rad = doc.positive(*start, "behind_deg") * deg;
        m.start_propellant_kg = doc.positive(*start, "propellant_t") * 1e3;
    } else {
        doc.fail("[start] mode must be \"pad\" or \"phasing_orbit\"");
    }
    m.earliest_utc = utc(doc, *window, "earliest_utc");
    m.window.search_s = doc.positive(*window, "search_days") * 86400.0;
    m.window.insertion_s = doc.positive(*window, "insertion_s");
    m.window.insertion_arc_rad = doc.positive(*window, "insertion_arc_deg") * deg;
    m.mission_limit_s = doc.positive(*phasing, "mission_limit_h") * 3600.0;
    m.earliest_arrival_s = doc.number(*phasing, "earliest_arrival_h") * 3600.0;
    if (phasing->contains("periapsis_km")) m.phasing_periapsis_height_m = doc.positive(*phasing, "periapsis_km") * 1e3;
    if (const auto* candidates = (*phasing)["candidates_km"].as_array()) {
        for (const auto& node : *candidates) {
            const auto* pair = node.as_array();
            const auto p = pair && pair->size() == 2 ? (*pair)[0].value<double>() : std::nullopt;
            const auto a = pair && pair->size() == 2 ? (*pair)[1].value<double>() : std::nullopt;
            if (!p || !a) doc.fail("[phasing] candidates_km must hold [periapsis, apoapsis] pairs");
            if (!(*p >= min_phasing_height_km) || !(*a >= *p)) {
                doc.fail("[phasing] candidates_km: periapsis below " + std::to_string(int(min_phasing_height_km)) +
                         " km or above the apoapsis");
            }
            m.phasing_candidates_m.emplace_back(*p * 1e3, *a * 1e3);
        }
        m.planning_mass_kg = doc.positive(*phasing, "planning_mass_t") * 1e3;
        m.planning_boil_off_kgps = doc.number(*phasing, "planning_boil_off_t_per_day") * 1e3 / 86400.0;
        if (m.planning_boil_off_kgps < 0.0) doc.fail("[phasing] planning_boil_off_t_per_day must not be negative");
        m.window_margin_s = doc.number_or(*phasing, "window_margin_h", 0.0) * 3600.0;
        if (m.window_margin_s < 0.0) doc.fail("[phasing] window_margin_h must not be negative");
    }
    if (m.kind == MissionKind::TankerFlight) {
        m.return_id = doc.text(*mission, "return");
    } else {
        const auto* lunar = doc.root["lunar"].as_table();
        if (!lunar) doc.fail("missing [lunar]");
        m.lunar_mission_id = doc.text(*lunar, "mission");
    }
    return m;
}

}  // namespace starsim::config
