#include "starsim/config/mission_config.hpp"

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
    doc.fail("mission: unknown kind '" + kind + "' (ascent, ship_return, capsule_return, lunar_landing)");
}

void require_kind(const detail::Document& doc, MissionKind kind, const char* name) {
    if (kind_of(doc) != kind) doc.fail(std::string("not a ") + name + " mission ([mission] kind)");
}

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
    m.entry_interface_height_m = doc.positive(*deorbit, "entry_interface_km") * 1000.0;
    m.entry_flight_path_rad = doc.number(*deorbit, "entry_flight_path_deg") * deg;
    if (!(m.entry_flight_path_rad < 0.0)) doc.fail("deorbit: entry_flight_path_deg must be negative");
    m.nominal_bank_rad = doc.positive(*deorbit, "nominal_bank_deg") * deg;
    m.max_crossrange_m = doc.positive(*deorbit, "max_crossrange_km") * 1000.0;
    m.search_window_s = doc.positive(*deorbit, "search_window_h") * 3600.0;
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

}  // namespace starsim::config
