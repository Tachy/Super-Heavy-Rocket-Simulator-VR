#include "starsim/config/launch_site_config.hpp"

#include "starsim/math/constants.hpp"
#include "toml_document.hpp"

namespace starsim::config {

namespace {

using detail::Document;

TowerConfig load_tower(const Document& doc, const toml::table& t) {
    const double deg = constants::deg_to_rad;
    TowerConfig c;
    c.azimuth_rad = doc.number(t, "azimuth_deg") * deg;
    c.distance_m = doc.positive(t, "distance_m");
    c.width_m = doc.positive(t, "width_m");
    c.height_m = doc.positive(t, "height_m");
    c.mount_height_m = doc.positive(t, "mount_height_m");
    c.mount_outer_radius_m = doc.positive(t, "mount_outer_radius_m");
    c.mount_inner_radius_m = doc.positive(t, "mount_inner_radius_m");
    c.arm_length_m = doc.positive(t, "arm_length_m");
    c.arm_width_m = doc.positive(t, "arm_width_m");
    c.arm_thickness_m = doc.positive(t, "arm_thickness_m");
    c.arm_open_rad = doc.positive(t, "arm_open_deg") * deg;
    c.arm_closed_gap_m = doc.positive(t, "arm_closed_gap_m");
    c.arm_rate_rad_s = doc.positive(t, "arm_rate_deg_s") * deg;
    c.arm_force_limit_n = doc.positive(t, "arm_force_limit_n");
    c.catch_height_m = doc.positive(t, "catch_height_m");
    c.carriage_mass_kg = doc.positive(t, "carriage_mass_kg");
    c.carriage_stiffness_n_m = doc.positive(t, "carriage_stiffness_n_m");
    c.carriage_damping_n_s_m = doc.positive(t, "carriage_damping_n_s_m");
    c.carriage_stroke_m = doc.positive(t, "carriage_stroke_m");
    if (c.mount_inner_radius_m >= c.mount_outer_radius_m) doc.fail("tower: mount_inner_radius_m >= mount_outer_radius_m");
    if (c.catch_height_m <= c.mount_height_m) doc.fail("tower: catch_height_m must be above mount_height_m");
    if (c.distance_m <= 0.5 * c.width_m) doc.fail("tower: the pad centre lies inside the tower");
    return c;
}

SeaConfig load_sea(const Document& doc, const toml::table& t) {
    SeaConfig c;
    c.direction_rad = doc.number(t, "direction_deg") * constants::deg_to_rad;
    c.shoreline_distance_m = doc.positive(t, "shoreline_distance_m");
    return c;
}

}  // namespace

LaunchSiteConfig load_launch_site(const std::filesystem::path& config_dir, std::string_view site_id) {
    const Document doc(config_dir / "launch_sites.toml");
    if (auto list = doc.root["site"].as_array()) {
        for (const auto& node : *list) {
            const auto& t = *node.as_table();
            if (doc.text(t, "id") != site_id) continue;
            LaunchSiteConfig s;
            s.id = std::string(site_id);
            s.name = doc.text(t, "name");
            s.latitude_rad = doc.number(t, "latitude_deg") * constants::deg_to_rad;
            s.longitude_rad = doc.number(t, "longitude_deg") * constants::deg_to_rad;
            s.height_m = doc.number(t, "height_m");
            if (auto tower = t["tower"].as_table()) s.tower = load_tower(doc, *tower);
            if (auto sea = t["sea"].as_table()) s.sea = load_sea(doc, *sea);
            return s;
        }
    }
    doc.fail("launch site '" + std::string(site_id) + "' not defined");
}

}  // namespace starsim::config
