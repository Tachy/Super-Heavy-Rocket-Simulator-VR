#include "starsim/config/splashdown_zone_config.hpp"

#include "starsim/math/constants.hpp"
#include "toml_document.hpp"

namespace starsim::config {

SplashdownZoneConfig load_splashdown_zone(const std::filesystem::path& config_dir, std::string_view zone_id) {
    const detail::Document doc(config_dir / "splashdown_zones.toml");
    if (auto list = doc.root["zone"].as_array()) {
        for (const auto& node : *list) {
            const auto& t = *node.as_table();
            if (doc.text(t, "id") != zone_id) continue;
            SplashdownZoneConfig z;
            z.id = std::string(zone_id);
            z.name = doc.text(t, "name");
            z.latitude_rad = doc.number(t, "latitude_deg") * constants::deg_to_rad;
            z.longitude_rad = doc.number(t, "longitude_deg") * constants::deg_to_rad;
            z.radius_m = doc.positive(t, "radius_m");
            return z;
        }
    }
    doc.fail("splashdown zone '" + std::string(zone_id) + "' not defined");
}

}  // namespace starsim::config
