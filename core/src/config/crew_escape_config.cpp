#include "starsim/config/crew_escape_config.hpp"

#include <cmath>

#include "starsim/math/constants.hpp"
#include "toml_document.hpp"

namespace starsim::config {

namespace {

double non_negative(const detail::Document& doc, const toml::table& t, std::string_view key) {
    const double v = doc.number(t, key);
    if (v < 0.0) doc.fail("'" + std::string(key) + "' must not be negative");
    return v;
}

math::Vec3 vec3(const detail::Document& doc, const toml::table& t, std::string_view key) {
    const auto* a = t[key].as_array();
    if (!a || a->size() != 3) doc.fail("'" + std::string(key) + "' must be an array of three numbers");
    math::Vec3 v;
    double* out[3] = {&v.x, &v.y, &v.z};
    for (size_t i = 0; i < 3; ++i) {
        const auto x = (*a)[i].value<double>();
        if (!x) doc.fail("'" + std::string(key) + "' must be an array of three numbers");
        *out[i] = doc.finite(*x, key);
    }
    return v;
}

std::vector<ThrustSample> thrust_table(const detail::Document& doc, const toml::table& t, std::string_view key) {
    const auto* a = t[key].as_array();
    if (!a || a->empty()) doc.fail("'" + std::string(key) + "' must be a non-empty array of [time_s, thrust_n]");
    std::vector<ThrustSample> table;
    for (const auto& node : *a) {
        const auto* pair = node.as_array();
        const auto time = pair && pair->size() == 2 ? (*pair)[0].value<double>() : std::nullopt;
        const auto thrust = pair && pair->size() == 2 ? (*pair)[1].value<double>() : std::nullopt;
        if (!time || !thrust) doc.fail("'" + std::string(key) + "' entries are [time_s, thrust_n]");
        const ThrustSample s{doc.finite(*time, key), doc.finite(*thrust, key)};
        if (s.thrust_n < 0.0) doc.fail("'" + std::string(key) + "': thrust must not be negative");
        if (!table.empty() && !(s.time_s > table.back().time_s)) doc.fail("'" + std::string(key) + "': times must ascend");
        table.push_back(s);
    }
    return table;
}

std::string vehicle_id(const detail::Document& doc, const std::filesystem::path& config_dir, const toml::table& t,
                       std::string_view key) {
    auto id = doc.text(t, key);
    if (!std::filesystem::exists(config_dir / "vehicles" / (id + ".toml"))) {
        doc.fail("'" + std::string(key) + "': unknown vehicle '" + id + "'");
    }
    return id;
}

}  // namespace

CrewEscapeConfig load_crew_escape(const std::filesystem::path& config_dir, std::string_view id) {
    const detail::Document doc(config_dir / "crew_escape" / (std::string(id) + ".toml"));
    const auto& t = doc.table(doc.root, "crew_escape");
    CrewEscapeConfig c;
    c.id = doc.text(t, "id");
    c.seat_vehicle_id = vehicle_id(doc, config_dir, t, "seat_vehicle");
    c.parachutist_vehicle_id = vehicle_id(doc, config_dir, t, "parachutist_vehicle");
    const auto rail = vec3(doc, t, "rail_direction");
    if (!(norm(rail) > 0.0)) doc.fail("'rail_direction' must not be zero");
    c.rail_direction_body = math::normalized(rail);
    c.rail_length_m = doc.positive(t, "rail_length_m");
    c.catapult = thrust_table(doc, t, "catapult");
    c.rocket = thrust_table(doc, t, "rocket");
    c.stagger_s = non_negative(doc, t, "stagger_s");
    c.envelope_max_height_m = doc.positive(t, "envelope_max_height_m");
    c.envelope_max_eas_mps = doc.positive(t, "envelope_max_eas_mps");
    c.mode1_max_eas_mps = doc.positive(t, "mode1_max_eas_mps");
    c.mode_low_height_m = doc.positive(t, "mode_low_height_m");
    c.mode1_main_delay_s = non_negative(doc, t, "mode1_main_delay_s");
    c.mode2_drogue_delay_s = non_negative(doc, t, "mode2_drogue_delay_s");
    c.mode2_separation_delay_s = non_negative(doc, t, "mode2_separation_delay_s");
    if (!(c.mode2_separation_delay_s > c.mode2_drogue_delay_s)) {
        doc.fail("'mode2_separation_delay_s' must follow 'mode2_drogue_delay_s'");
    }
    c.min_firing_height_m = doc.positive(t, "min_firing_height_m");
    c.sea_target_margin_m = non_negative(doc, t, "sea_target_margin_m");
    c.min_catch_propellant_kg = non_negative(doc, t, "min_catch_propellant_kg");
    c.max_load_g = doc.positive(t, "max_load_g");
    c.max_load_duration_s = non_negative(doc, t, "max_load_duration_s");
    c.min_clearance_m = non_negative(doc, t, "min_clearance_m");
    c.plume_half_angle_rad = doc.positive(t, "plume_half_angle_deg") * constants::deg_to_rad;
    c.plume_length_m = doc.positive(t, "plume_length_m");
    c.max_touchdown_sink_mps = doc.positive(t, "max_touchdown_sink_mps");

    const auto* seats = doc.root["seat"].as_array();
    if (!seats || seats->empty()) doc.fail("at least one [[seat]] is required");
    for (const auto& node : *seats) {
        const auto* st = node.as_table();
        if (!st) doc.fail("[[seat]] entries must be tables");
        c.seats.push_back({vec3(doc, *st, "position_m")});
    }
    c.crew_count = static_cast<int>(c.seats.size());
    return c;
}

double thrust_at(const std::vector<ThrustSample>& table, double t_s) {
    if (table.empty() || t_s < table.front().time_s || t_s > table.back().time_s) return 0.0;
    for (size_t i = 1; i < table.size(); ++i) {
        if (t_s <= table[i].time_s) {
            const auto& a = table[i - 1];
            const auto& b = table[i];
            return a.thrust_n + (b.thrust_n - a.thrust_n) * (t_s - a.time_s) / (b.time_s - a.time_s);
        }
    }
    return table.back().thrust_n;
}

}  // namespace starsim::config
