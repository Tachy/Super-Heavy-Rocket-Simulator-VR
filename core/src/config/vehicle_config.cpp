#include "starsim/config/vehicle_config.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

#include <toml++/toml.hpp>

#include "starsim/math/constants.hpp"
#include "toml_document.hpp"

namespace starsim::config {

namespace {

using detail::Document;

PropellantConfig load_propellant(const std::filesystem::path& config_dir, const std::string& id) {
    const Document doc(config_dir / "propellants.toml");
    if (auto list = doc.root["propellant"].as_array()) {
        for (const auto& node : *list) {
            const auto& t = *node.as_table();
            if (doc.text(t, "id") == id) {
                return {id, doc.number(t, "mixture_ratio"), doc.number(t, "oxidizer_density_kg_m3"),
                        doc.number(t, "fuel_density_kg_m3")};
            }
        }
    }
    doc.fail("propellant '" + id + "' not defined");
}

EngineConfig load_engine(const std::filesystem::path& config_dir, const std::string& id) {
    const Document doc(config_dir / "engines" / (id + ".toml"));
    const auto& engine = doc.table(doc.root, "engine");
    const auto& performance = doc.table(doc.root, "performance");
    const auto& throttle = doc.table(doc.root, "throttle");
    const auto& gimbal = doc.table(doc.root, "gimbal");
    const auto& mass = doc.table(doc.root, "mass");
    const auto& timing = doc.table(doc.root, "timing");

    EngineConfig e;
    e.id = id;
    e.name = doc.text(engine, "name");
    e.mixture_ratio = doc.number(engine, "mixture_ratio");
    e.reference_pressure_pa = doc.number(performance, "reference_pressure_pa");
    e.thrust_ref_n = doc.number(performance, "thrust_ref_n");
    e.isp_ref_s = doc.number(performance, "isp_ref_s");
    const double exit_diameter = doc.number(performance, "nozzle_exit_diameter_m");
    e.nozzle_exit_area_m2 = constants::pi * 0.25 * exit_diameter * exit_diameter;
    e.max_ambient_pressure_pa = doc.number(performance, "max_ambient_pressure_pa");
    e.throttle_min = doc.number(throttle, "min");
    e.throttle_max = doc.number(throttle, "max");
    e.throttle_rate_per_s = doc.number(throttle, "rate_per_s");
    e.gimbal_max_rad = doc.number(gimbal, "max_angle_deg") * constants::deg_to_rad;
    e.gimbal_rate_rad_s = doc.number(gimbal, "rate_deg_s") * constants::deg_to_rad;
    e.dry_mass_kg = doc.number(mass, "dry_kg");
    e.startup_s = doc.number(timing, "startup_s");
    e.shutdown_s = doc.number(timing, "shutdown_s");
    if (e.thrust_ref_n <= 0.0 || e.isp_ref_s <= 0.0) {
        doc.fail("thrust_ref_n and isp_ref_s must be positive");
    }
    return e;
}

StageConfig load_stage(const Document& doc, const toml::table& t) {
    StageConfig s;
    s.id = doc.text(t, "id");
    s.name = doc.text(t, "name");
    s.length_m = doc.number(t, "length_m");
    s.diameter_m = doc.number(t, "diameter_m");
    s.dry_mass_kg = doc.number(t, "dry_mass_kg");
    s.dry_cg_station_m = doc.number(t, "dry_cg_station_m");
    s.payload_mass_kg = doc.number_or(t, "payload_mass_kg", 0.0);
    s.fairing_mass_kg = doc.number_or(t, "fairing_mass_kg", 0.0);
    s.jettison_after_separation_kg = doc.number_or(t, "jettison_after_separation_kg", 0.0);
    s.radiation_pressure_coefficient = doc.number_or(t, "radiation_pressure_coefficient", 1.3);
    s.separation = doc.text_or(t, "separation");

    if (auto tanks = t["tank"].as_array()) {
        for (const auto& node : *tanks) {
            const auto& tank = *node.as_table();
            const auto contents = doc.text(tank, "contents");
            if (contents != "oxidizer" && contents != "fuel") {
                doc.fail("tank contents must be 'oxidizer' or 'fuel'");
            }
            s.tanks.push_back({doc.text(tank, "id"),
                               contents == "oxidizer" ? TankContents::Oxidizer : TankContents::Fuel,
                               doc.number(tank, "mass_kg"), doc.number(tank, "base_station_m")});
        }
    }
    if (auto groups = t["engine_group"].as_array()) {
        for (const auto& node : *groups) {
            const auto& g = *node.as_table();
            s.engine_groups.push_back({doc.text(g, "id"), doc.text(g, "engine"),
                                       static_cast<int>(doc.number(g, "count")), doc.number(g, "ring_radius_m"),
                                       doc.boolean(g, "gimbal"), doc.boolean(g, "relight"), {}});
            if (const auto* angles = g["angles_deg"].as_array()) {
                for (const auto& a : *angles) {
                    const auto value = a.value<double>();
                    if (!value) doc.fail("engine_group.angles_deg must be numbers");
                    s.engine_groups.back().angles_rad.push_back(*value * constants::deg_to_rad);
                }
                if (static_cast<int>(s.engine_groups.back().angles_rad.size()) != s.engine_groups.back().count) {
                    doc.fail("engine_group.angles_deg must list one angle per engine");
                }
            }
        }
    }
    if (auto aero = t["aero"].as_table()) {
        s.aero.reference_diameter_m = doc.number_or(*aero, "reference_diameter_m", s.diameter_m);
        s.aero.ascent_drag_table = doc.text_or(*aero, "ascent_drag_table");
        s.aero.descent_drag_table = doc.text_or(*aero, "descent_drag_table");
        s.aero.crossflow_table = doc.text_or(*aero, "crossflow_table");
        s.aero.nose_length_m = doc.number_or(*aero, "nose_length_m", 0.0);
        s.aero.flap_table = doc.text_or(*aero, "flap_table");
        s.aero.entry_alpha_table = doc.text_or(*aero, "entry_alpha_table");
        s.aero.belly_azimuth_rad = doc.number_or(*aero, "belly_azimuth_deg", 270.0) * constants::deg_to_rad;
    } else {
        s.aero.reference_diameter_m = s.diameter_m;
    }
    if (auto fins = t["grid_fins"].as_table()) {
        const auto* azimuths = (*fins)["azimuths_deg"].as_array();
        if (!azimuths || azimuths->empty()) {
            doc.fail("grid_fins.azimuths_deg must list one azimuth per fin");
        }
        for (const auto& a : *azimuths) {
            const auto value = a.value<double>();
            if (!value) doc.fail("grid_fins.azimuths_deg must be numbers");
            s.grid_fins.azimuths_rad.push_back(*value * constants::deg_to_rad);
        }
        s.grid_fins.area_m2 = doc.number(*fins, "area_m2");
        s.grid_fins.station_m = doc.number(*fins, "station_m");
        s.grid_fins.radius_m = doc.number(*fins, "radius_m");
        s.grid_fins.max_deflection_rad = doc.number(*fins, "max_deflection_deg") * constants::deg_to_rad;
        s.grid_fins.rate_rad_s = doc.number(*fins, "rate_deg_s") * constants::deg_to_rad;
        s.grid_fins.table = doc.text(*fins, "table");
    }
    if (auto flaps = t["flap"].as_array()) {
        for (auto& node : *flaps) {
            const auto* ft = node.as_table();
            if (!ft) doc.fail("stage.flap entries must be tables");
            config::FlapConfig f;
            f.id = doc.text(*ft, "id");
            f.hinge_azimuth_rad = doc.number(*ft, "hinge_azimuth_deg") * constants::deg_to_rad;
            f.station_fwd_m = doc.number(*ft, "station_fwd_m");
            f.station_aft_m = doc.number(*ft, "station_aft_m");
            f.span_m = doc.number(*ft, "span_m");
            f.area_m2 = doc.number(*ft, "area_m2");
            f.max_deflection_rad = doc.number(*ft, "max_deflection_deg") * constants::deg_to_rad;
            f.rate_rad_s = doc.number(*ft, "rate_deg_s") * constants::deg_to_rad;
            f.initial_deflection_rad = doc.number_or(*ft, "initial_deflection_deg", 0.0) * constants::deg_to_rad;
            s.flaps.push_back(f);
        }
        if (!s.flaps.empty() && s.aero.flap_table.empty()) doc.fail("stage.flap needs [stage.aero] flap_table");
    }
    if (auto shield = t["heat_shield"].as_table()) {
        s.heat_shield.nose_radius_m = doc.number(*shield, "nose_radius_m");
        s.heat_shield.max_heat_rate_w_m2 = doc.number(*shield, "max_heat_rate_w_m2");
        s.heat_shield.max_heat_load_j_m2 = doc.number(*shield, "max_heat_load_j_m2");
        s.heat_shield.max_load_g = doc.number_or(*shield, "max_load_g", 0.0);
    }
    if (auto pins = t["catch_pins"].as_table()) {
        const auto* azimuths = (*pins)["azimuths_deg"].as_array();
        if (!azimuths || azimuths->empty()) {
            doc.fail("catch_pins.azimuths_deg must list one azimuth per pin");
        }
        for (const auto& a : *azimuths) {
            const auto value = a.value<double>();
            if (!value) doc.fail("catch_pins.azimuths_deg must be numbers");
            s.catch_pins.azimuths_rad.push_back(*value * constants::deg_to_rad);
        }
        s.catch_pins.station_m = doc.number(*pins, "station_m");
        s.catch_pins.protrusion_m = doc.positive(*pins, "protrusion_m");
        s.catch_pins.max_load_n = doc.positive(*pins, "max_load_n");
    }
    if (auto legs = t["landing_legs"].as_table()) {
        const auto* azimuths = (*legs)["azimuths_deg"].as_array();
        if (!azimuths || azimuths->size() < 3) doc.fail("landing_legs.azimuths_deg must list at least three legs");
        for (const auto& a : *azimuths) {
            const auto value = a.value<double>();
            if (!value) doc.fail("landing_legs.azimuths_deg must be numbers");
            s.landing_legs.azimuths_rad.push_back(*value * constants::deg_to_rad);
        }
        s.landing_legs.footprint_radius_m = doc.positive(*legs, "footprint_radius_m");
        s.landing_legs.foot_station_m = doc.number(*legs, "foot_station_m");
        s.landing_legs.deploy_time_s = doc.positive(*legs, "deploy_time_s");
        s.landing_legs.stroke_m = doc.positive(*legs, "stroke_m");
        s.landing_legs.crush_force_n = doc.positive(*legs, "crush_force_n");
        s.landing_legs.stiffness_n_m = doc.positive(*legs, "stiffness_n_m");
        s.landing_legs.damping_n_s_m = doc.positive(*legs, "damping_n_s_m");
        s.landing_legs.friction = doc.positive(*legs, "friction");
    }
    if (auto rcs = t["rcs"].as_table()) {
        s.rcs_thruster_count = static_cast<int>(doc.number_or(*rcs, "thruster_count", 0.0));
        s.rcs_thrust_n = doc.number_or(*rcs, "thrust_per_thruster_n", 0.0);
    }
    if (auto recovery = t["recovery"].as_table()) {
        s.reserve_propellant_kg = doc.number_or(*recovery, "reserve_propellant_kg", 0.0);
        s.landing_propellant_kg = doc.number_or(*recovery, "landing_propellant_kg", 0.0);
        s.recovery_method = doc.text_or(*recovery, "method");
        auto group_list = [&](const toml::array& list, const std::string& key) {
            std::vector<std::string> ids;
            for (const auto& node : list) {
                const auto id = node.value<std::string>().value_or("");
                const bool known = std::any_of(s.engine_groups.begin(), s.engine_groups.end(),
                                               [&](const auto& g) { return g.id == id; });
                if (!known) doc.fail(key + " group '" + id + "' is not an engine group of stage '" + s.id + "'");
                ids.push_back(id);
            }
            return ids;
        };
        if (const auto* groups = (*recovery)["deorbit_engine_groups"].as_array()) {
            s.deorbit_engine_groups = group_list(*groups, "deorbit engine");
        }
        // Return profile: `base` with the keys of `table` that are set.
        auto read_profile = [&](const toml::table& table, config::ReturnProfile p) {
            p.boostback = table["boostback"].value_or(p.boostback);
            if (const auto* groups = table["boostback_engine_groups"].as_array()) {
                p.boostback_engine_groups = group_list(*groups, "boostback engine");
            }
            if (const auto* groups = table["entry_engine_groups"].as_array()) {
                p.entry_engine_groups = group_list(*groups, "entry engine");
            }
            if (const auto* phases = table["landing_engine_phases"].as_array()) {
                p.landing_engine_phases.clear();
                for (const auto& node : *phases) {
                    const auto* list = node.as_array();
                    if (!list) doc.fail("landing_engine_phases must be a list of engine group lists");
                    p.landing_engine_phases.push_back(group_list(*list, "landing engine"));
                }
                if (p.landing_engine_phases.size() != 3) doc.fail("landing_engine_phases must list 3 engine sets");
            }
            if (const auto* throttles = table["landing_phase_throttles"].as_array()) {
                p.landing_phase_throttles.clear();
                for (const auto& node : *throttles) {
                    const auto value = node.value<double>();
                    if (!value || *value <= 0.0 || *value > 1.0) doc.fail("landing_phase_throttles must be in (0, 1]");
                    p.landing_phase_throttles.push_back(*value);
                }
                if (p.landing_phase_throttles.size() != 3) doc.fail("landing_phase_throttles must list 3 throttles");
            }
            if (const auto* tilts = table["landing_phase_max_tilt_deg"].as_array()) {
                p.landing_phase_max_tilt_rad.clear();
                for (const auto& node : *tilts) {
                    const auto value = node.value<double>();
                    if (!value || *value <= 0.0 || *value >= 90.0) doc.fail("landing_phase_max_tilt_deg must be in (0, 90)");
                    p.landing_phase_max_tilt_rad.push_back(*value * constants::deg_to_rad);
                }
                if (p.landing_phase_max_tilt_rad.size() != 3) doc.fail("landing_phase_max_tilt_deg must list 3 limits");
            }
            if (const auto* windows = table["landing_switch_sink_mps"].as_array()) {
                p.landing_switch_sink_mps.clear();
                for (const auto& node : *windows) {
                    const auto* pair = node.as_array();
                    const auto low = pair && pair->size() == 2 ? (*pair)[0].value<double>() : std::nullopt;
                    const auto high = pair && pair->size() == 2 ? (*pair)[1].value<double>() : std::nullopt;
                    if (!low || !high || *low < 0.0 || *high < *low) {
                        doc.fail("landing_switch_sink_mps entries must be [min, max] sink rates");
                    }
                    p.landing_switch_sink_mps.emplace_back(*low, *high);
                }
                if (p.landing_switch_sink_mps.size() != 2) doc.fail("landing_switch_sink_mps must list 2 switch windows");
            }
            p.return_flight_time_s = doc.number_or(table, "return_flight_time_s", p.return_flight_time_s);
            p.boostback_loss_fraction = doc.number_or(table, "boostback_loss_fraction", p.boostback_loss_fraction);
            p.entry_burn_altitude_m = doc.number_or(table, "entry_burn_altitude_m", p.entry_burn_altitude_m);
            p.entry_burn_delta_v_mps = doc.number_or(table, "entry_burn_delta_v_mps", p.entry_burn_delta_v_mps);
            p.entry_burn_throttle = doc.number_or(table, "entry_burn_throttle", p.entry_burn_throttle);
            if (p.entry_burn_altitude_m > 0.0 && p.entry_engine_groups.empty()) {
                doc.fail("an entry burn (entry_burn_altitude_m) needs entry_engine_groups");
            }
            return p;
        };
        s.return_profile = read_profile(*recovery, {});
        if (const auto* drone_ship = (*recovery)["drone_ship"].as_table()) {
            s.drone_ship_return = read_profile(*drone_ship, s.return_profile);
        }
    }
    return s;
}

}  // namespace

const EngineConfig& VehicleConfig::engine(const std::string& engine_id) const {
    const auto it = engines.find(engine_id);
    if (it == engines.end()) {
        throw ConfigError("engine '" + engine_id + "' not loaded for vehicle " + id);
    }
    return it->second;
}

VehicleConfig load_vehicle(const std::filesystem::path& config_dir, std::string_view vehicle_id) {
    const Document doc(config_dir / "vehicles" / (std::string(vehicle_id) + ".toml"));
    const auto& vehicle = doc.table(doc.root, "vehicle");

    VehicleConfig v;
    v.id = doc.text(vehicle, "id");
    v.name = doc.text(vehicle, "name");
    v.display_name = doc.text_or(vehicle, "display_name", v.name);
    v.propellant = load_propellant(config_dir, doc.text(vehicle, "propellant"));

    std::map<std::string, StageConfig> stages_by_id;
    if (auto stages = doc.root["stage"].as_array()) {
        for (const auto& node : *stages) {
            auto stage = load_stage(doc, *node.as_table());
            stages_by_id[stage.id] = std::move(stage);
        }
    }
    const auto* order = vehicle["stages_bottom_to_top"].as_array();
    if (!order) {
        doc.fail("missing stages_bottom_to_top");
    }
    for (const auto& node : *order) {
        const auto id = node.value<std::string>().value_or("");
        const auto it = stages_by_id.find(id);
        if (it == stages_by_id.end()) {
            doc.fail("stage '" + id + "' listed in stages_bottom_to_top is not defined");
        }
        v.stages.push_back(it->second);
    }
    for (const auto& stage : v.stages) {
        for (const auto& group : stage.engine_groups) {
            if (!v.engines.contains(group.engine_id)) {
                v.engines[group.engine_id] = load_engine(config_dir, group.engine_id);
            }
        }
    }
    return v;
}

std::vector<double> CsvTable::column(std::string_view name) const {
    for (size_t c = 0; c < columns.size(); ++c) {
        if (columns[c] == name) {
            std::vector<double> values;
            values.reserve(rows.size());
            for (const auto& row : rows) values.push_back(row.at(c));
            return values;
        }
    }
    throw ConfigError("CSV column '" + std::string(name) + "' not found");
}

CsvTable load_csv_table(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        throw ConfigError("CSV table not found: " + path.string());
    }
    CsvTable table;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> cells;
        std::stringstream stream(line);
        std::string cell;
        while (std::getline(stream, cell, ',')) cells.push_back(cell);
        if (table.columns.empty()) {
            table.columns = std::move(cells);
            continue;
        }
        if (cells.size() != table.columns.size()) {
            throw ConfigError(path.string() + ": row with " + std::to_string(cells.size()) + " cells, expected " +
                              std::to_string(table.columns.size()));
        }
        std::vector<double> row;
        for (const auto& c : cells) row.push_back(std::stod(c));
        table.rows.push_back(std::move(row));
    }
    return table;
}

}  // namespace starsim::config
