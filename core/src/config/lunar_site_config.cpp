#include "starsim/config/lunar_site_config.hpp"

#include <fstream>

#include "starsim/math/constants.hpp"
#include "toml_document.hpp"

namespace starsim::config {

namespace {

LunarSiteConfig parse_site(const detail::Document& doc, const toml::table& t) {
    LunarSiteConfig s;
    s.id = doc.text(t, "id");
    s.name = doc.text(t, "name");
    s.product = doc.text(t, "product");
    s.latitude_rad = doc.number(t, "latitude_deg") * constants::deg_to_rad;
    s.longitude_rad = doc.number(t, "longitude_deg") * constants::deg_to_rad;
    s.tile_half_size_m = doc.positive(t, "tile_half_size_m");
    return s;
}

[[noreturn]] void missing(const std::filesystem::path& path) {
    throw ConfigError("Geländedaten fehlen: " + path.string() + " - bitte python tools/data_pipeline/fetch.py ausführen");
}

}  // namespace

std::vector<LunarSiteConfig> load_lunar_sites(const std::filesystem::path& config_dir) {
    const detail::Document doc(config_dir / "lunar_sites.toml");
    std::vector<LunarSiteConfig> sites;
    if (auto list = doc.root["site"].as_array()) {
        for (const auto& node : *list) sites.push_back(parse_site(doc, *node.as_table()));
    }
    return sites;
}

LunarSiteConfig load_lunar_site(const std::filesystem::path& config_dir, std::string_view id) {
    for (auto& s : load_lunar_sites(config_dir)) {
        if (s.id == id) return s;
    }
    throw ConfigError((config_dir / "lunar_sites.toml").string() + ": lunar site '" + std::string(id) + "' not defined");
}

TerrainGridFile load_terrain_grid_file(const std::filesystem::path& toml_path) {
    if (!std::filesystem::exists(toml_path)) missing(toml_path);
    const detail::Document doc(toml_path);
    const auto& g = doc.table(doc.root, "grid");
    TerrainGridFile f;
    f.product = doc.text(g, "product");
    f.projection = doc.text(g, "projection");
    f.sphere_radius_m = doc.positive(g, "sphere_radius_m");
    f.dn_scale_m = doc.positive(g, "dn_scale_m");
    f.lines = static_cast<int>(doc.positive(g, "lines"));
    f.samples = static_cast<int>(doc.positive(g, "samples"));
    if (f.projection == "polar_stereographic_south") {
        f.pole_line = doc.number(g, "pole_line");
        f.pole_sample = doc.number(g, "pole_sample");
        f.scale_m = doc.positive(g, "scale_m");
        f.first_line = static_cast<int>(doc.number(g, "first_line"));
        f.first_sample = static_cast<int>(doc.number(g, "first_sample"));
    } else if (f.projection == "simple_cylindrical") {
        f.pixels_per_degree = doc.positive(g, "pixels_per_degree");
        f.zero_line = doc.number(g, "zero_line");
        f.zero_sample = doc.number(g, "zero_sample");
    } else if (f.projection == "geographic") {
        f.north_latitude_deg = doc.number(g, "north_latitude_deg");
        f.west_longitude_deg = doc.number(g, "west_longitude_deg");
        f.step_latitude_deg = doc.positive(g, "step_latitude_deg");
        f.step_longitude_deg = doc.positive(g, "step_longitude_deg");
        f.global_longitude = doc.boolean(g, "global_longitude");
        f.height_reference = doc.text(g, "height_reference");
        if (f.height_reference != "egm2008_undulation" && f.height_reference != "wgs84_ellipsoid") {
            doc.fail("unknown height_reference '" + f.height_reference + "'");
        }
        f.area = doc.text_or(g, "area");
        f.level = doc.text_or(g, "level");
        f.area_latitude_deg = doc.number_or(g, "area_latitude_deg", 0.0);
        f.area_longitude_deg = doc.number_or(g, "area_longitude_deg", 0.0);
        f.half_size_m = doc.number_or(g, "half_size_m", 0.0);
    } else {
        doc.fail("unknown projection '" + f.projection + "'");
    }
    const auto heights = toml_path.parent_path() / doc.text(g, "heights");
    if (!std::filesystem::exists(heights)) missing(heights);
    const auto count = static_cast<size_t>(f.lines) * static_cast<size_t>(f.samples);
    if (std::filesystem::file_size(heights) != 2 * count) doc.fail("heights file size does not match lines x samples");
    f.dn.resize(count);
    std::ifstream in(heights, std::ios::binary);
    in.read(reinterpret_cast<char*>(f.dn.data()), static_cast<std::streamsize>(2 * count));  // little endian (x86)
    if (!in) doc.fail("cannot read " + heights.string());
    if (auto list = doc.root["check"].as_array()) {
        for (const auto& node : *list) {
            const auto& c = *node.as_table();
            f.checks.push_back({doc.number(c, "latitude_deg") * constants::deg_to_rad,
                                doc.number(c, "longitude_deg") * constants::deg_to_rad, doc.number(c, "height_m")});
        }
    }
    return f;
}

}  // namespace starsim::config
