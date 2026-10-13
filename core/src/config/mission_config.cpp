#include "starsim/config/mission_config.hpp"

#include "starsim/math/constants.hpp"
#include "toml_document.hpp"

namespace starsim::config {

ShipReturnMission load_ship_return_mission(const std::filesystem::path& config_dir, std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
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

AscentMission load_ascent_mission(const std::filesystem::path& config_dir, std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
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
    m.heading_rad = doc.number(*mission, "heading_deg") * deg;
    m.payload_kg = doc.number_or(*mission, "payload_t", 0.0) * 1000.0;
    m.fairing = (*mission)["fairing"].value_or(true);
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
