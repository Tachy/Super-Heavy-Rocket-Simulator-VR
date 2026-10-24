#include "starsim/config/drone_ship_config.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "starsim/math/constants.hpp"
#include "toml_document.hpp"

namespace starsim::config {

namespace {

constexpr double deg = constants::deg_to_rad;

DroneShipConfig read_ship(const detail::Document& doc, const toml::table& t) {
    DroneShipConfig s;
    s.id = doc.text(t, "id");
    s.name = doc.text(t, "name");
    s.deck_length_m = doc.positive(t, "deck_length_m");
    s.deck_width_m = doc.positive(t, "deck_width_m");
    s.freeboard_m = doc.positive(t, "freeboard_m");
    if (s.deck_width_m > s.deck_length_m) doc.fail("drone ship '" + s.id + "': deck_width_m > deck_length_m");
    s.basin = doc.text(t, "basin");
    s.home_port_latitude_rad = doc.number(t, "home_port_latitude_deg") * deg;
    s.home_port_longitude_rad = doc.number(t, "home_port_longitude_deg") * deg;
    s.area_min_latitude_rad = doc.number(t, "area_min_latitude_deg") * deg;
    s.area_max_latitude_rad = doc.number(t, "area_max_latitude_deg") * deg;
    s.area_min_longitude_rad = doc.number(t, "area_min_longitude_deg") * deg;
    s.area_max_longitude_rad = doc.number(t, "area_max_longitude_deg") * deg;
    if (s.area_min_latitude_rad >= s.area_max_latitude_rad || s.area_min_longitude_rad >= s.area_max_longitude_rad) {
        doc.fail("drone ship '" + s.id + "': the service area is empty");
    }

    auto& h = s.hull;
    h.length_m = doc.positive(t, "hull_length_m");
    h.beam_m = doc.positive(t, "hull_beam_m");
    if (h.beam_m > h.length_m) doc.fail("drone ship '" + s.id + "': hull_beam_m > hull_length_m");
    h.draft_m = doc.positive(t, "hull_depth_m") - s.freeboard_m;
    if (!(h.draft_m > 0.0)) doc.fail("drone ship '" + s.id + "': hull_depth_m must exceed freeboard_m (draft <= 0)");
    h.roll_gyration_m = doc.positive(t, "hull_roll_gyration_m");
    h.pitch_gyration_m = doc.positive(t, "hull_pitch_gyration_m");
    const auto non_negative = [&](std::string_view key) {
        const double v = doc.number(t, key);
        if (v < 0.0) doc.fail("drone ship '" + s.id + "': '" + std::string(key) + "' must not be negative");
        return v;
    };
    h.heave_added_mass_factor = non_negative("hull_heave_added_mass_factor");
    h.roll_added_inertia_factor = non_negative("hull_roll_added_inertia_factor");
    h.pitch_added_inertia_factor = non_negative("hull_pitch_added_inertia_factor");
    const auto damping = [&](std::string_view key) {
        const double v = doc.positive(t, key);
        if (v >= 1.0) doc.fail("drone ship '" + s.id + "': '" + std::string(key) + "' must be below 1");
        return v;
    };
    h.heave_damping_ratio = damping("hull_heave_damping_ratio");
    h.roll_damping_ratio = damping("hull_roll_damping_ratio");
    h.pitch_damping_ratio = damping("hull_pitch_damping_ratio");
    s.max_deck_tilt_rad = doc.positive(t, "max_deck_tilt_deg") * deg;
    s.max_heave_rate_mps = doc.positive(t, "max_heave_rate_mps");
    return s;
}

double wrapped_longitude(double lon_rad) {
    const double two_pi = 2.0 * constants::pi;
    double l = std::fmod(lon_rad, two_pi);
    if (l > constants::pi) l -= two_pi;
    if (l <= -constants::pi) l += two_pi;
    return l;
}

// Great-circle central angle (haversine).
double central_angle_rad(double lat1, double lon1, double lat2, double lon2) {
    const double s_lat = std::sin(0.5 * (lat2 - lat1));
    const double s_lon = std::sin(0.5 * (lon2 - lon1));
    const double a = s_lat * s_lat + std::cos(lat1) * std::cos(lat2) * s_lon * s_lon;
    return 2.0 * std::asin(std::min(1.0, std::sqrt(a)));
}

}  // namespace

DroneShipConfig load_drone_ship(const std::filesystem::path& config_dir, std::string_view ship_id) {
    const detail::Document doc(config_dir / "drone_ships.toml");
    if (auto list = doc.root["drone_ship"].as_array()) {
        for (const auto& node : *list) {
            const auto& t = *node.as_table();
            if (doc.text(t, "id") == ship_id) return read_ship(doc, t);
        }
    }
    doc.fail("drone ship '" + std::string(ship_id) + "' not defined");
}

std::vector<DroneShipConfig> load_drone_ships(const std::filesystem::path& config_dir) {
    const detail::Document doc(config_dir / "drone_ships.toml");
    std::vector<DroneShipConfig> ships;
    if (auto list = doc.root["drone_ship"].as_array()) {
        for (const auto& node : *list) ships.push_back(read_ship(doc, *node.as_table()));
    }
    return ships;
}

std::optional<DroneShipConfig> drone_ship_for(const std::vector<DroneShipConfig>& ships, double latitude_rad,
                                              double longitude_rad) {
    const double lon = wrapped_longitude(longitude_rad);
    std::optional<DroneShipConfig> best;
    double best_distance = 0.0;
    for (const auto& s : ships) {
        if (latitude_rad < s.area_min_latitude_rad || latitude_rad > s.area_max_latitude_rad) continue;
        if (lon < s.area_min_longitude_rad || lon > s.area_max_longitude_rad) continue;
        const double d = central_angle_rad(s.home_port_latitude_rad, s.home_port_longitude_rad, latitude_rad, lon);
        if (!best || d < best_distance) {
            best = s;
            best_distance = d;
        }
    }
    return best;
}

}  // namespace starsim::config
