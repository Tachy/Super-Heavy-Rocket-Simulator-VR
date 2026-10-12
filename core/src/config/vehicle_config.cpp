#include "starsim/config/vehicle_config.hpp"

#include <fstream>
#include <sstream>

#include <toml++/toml.hpp>

#include "starsim/math/constants.hpp"

namespace starsim::config {

namespace {

// Reads a TOML file and remembers its path for error messages.
struct Document {
    std::filesystem::path path;
    toml::table root;

    explicit Document(std::filesystem::path p) : path(std::move(p)) {
        if (!std::filesystem::exists(path)) {
            throw ConfigError("config file not found: " + path.string());
        }
        try {
            root = toml::parse_file(path.string());
        } catch (const toml::parse_error& e) {
            std::ostringstream message;
            message << path.string() << ": " << e.description() << " (line " << e.source().begin.line << ")";
            throw ConfigError(message.str());
        }
    }

    [[noreturn]] void fail(const std::string& what) const { throw ConfigError(path.string() + ": " + what); }

    double number(const toml::table& t, std::string_view key) const {
        if (auto v = t[key].value<double>()) return *v;
        fail("missing number '" + std::string(key) + "'");
    }
    double number_or(const toml::table& t, std::string_view key, double fallback) const {
        return t[key].value<double>().value_or(fallback);
    }
    std::string text(const toml::table& t, std::string_view key) const {
        if (auto v = t[key].value<std::string>()) return *v;
        fail("missing string '" + std::string(key) + "'");
    }
    std::string text_or(const toml::table& t, std::string_view key, std::string fallback = {}) const {
        return t[key].value<std::string>().value_or(std::move(fallback));
    }
    bool boolean(const toml::table& t, std::string_view key) const {
        if (auto v = t[key].value<bool>()) return *v;
        fail("missing boolean '" + std::string(key) + "'");
    }
    const toml::table& table(const toml::table& t, std::string_view key) const {
        if (auto v = t[key].as_table()) return *v;
        fail("missing table [" + std::string(key) + "]");
    }
};

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
                                       doc.boolean(g, "gimbal"), doc.boolean(g, "relight")});
        }
    }
    if (auto aero = t["aero"].as_table()) {
        s.aero.reference_diameter_m = doc.number_or(*aero, "reference_diameter_m", s.diameter_m);
        s.aero.ascent_drag_table = doc.text_or(*aero, "ascent_drag_table");
        s.aero.entry_table = doc.text_or(*aero, "entry_table");
        s.aero.nose_length_m = doc.number_or(*aero, "nose_length_m", 0.0);
    } else {
        s.aero.reference_diameter_m = s.diameter_m;
    }
    if (auto recovery = t["recovery"].as_table()) {
        s.reserve_propellant_kg = doc.number_or(*recovery, "reserve_propellant_kg", 0.0);
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
