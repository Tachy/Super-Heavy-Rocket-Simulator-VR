#include "starsim/config/drone_ship_config.hpp"

#include <algorithm>
#include <cmath>

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
