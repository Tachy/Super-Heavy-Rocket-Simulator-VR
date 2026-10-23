#include "starsim/config/earth_terrain_config.hpp"

#include "starsim/math/constants.hpp"
#include "toml_document.hpp"

namespace starsim::config {

std::vector<EarthTerrainArea> load_earth_terrain_areas(const std::filesystem::path& config_dir) {
    const detail::Document doc(config_dir / "earth_terrain.toml");
    std::vector<EarthTerrainArea> areas;
    if (auto list = doc.root["area"].as_array()) {
        for (const auto& node : *list) {
            const auto& t = *node.as_table();
            EarthTerrainArea a;
            a.id = doc.text(t, "id");
            a.latitude_rad = doc.number(t, "latitude_deg") * constants::deg_to_rad;
            a.longitude_rad = doc.number(t, "longitude_deg") * constants::deg_to_rad;
            a.local_half_size_m = doc.positive(t, "local_half_size_m");
            a.regional_half_size_m = doc.positive(t, "regional_half_size_m");
            if (a.regional_half_size_m <= a.local_half_size_m) {
                doc.fail("area '" + a.id + "': regional_half_size_m must exceed local_half_size_m");
            }
            areas.push_back(std::move(a));
        }
    }
    return areas;
}

}  // namespace starsim::config
