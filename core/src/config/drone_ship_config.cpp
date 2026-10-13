#include "starsim/config/drone_ship_config.hpp"

#include "toml_document.hpp"

namespace starsim::config {

DroneShipConfig load_drone_ship(const std::filesystem::path& config_dir, std::string_view ship_id) {
    const detail::Document doc(config_dir / "drone_ships.toml");
    if (auto list = doc.root["drone_ship"].as_array()) {
        for (const auto& node : *list) {
            const auto& t = *node.as_table();
            if (doc.text(t, "id") != ship_id) continue;
            DroneShipConfig s;
            s.id = std::string(ship_id);
            s.name = doc.text(t, "name");
            s.deck_length_m = doc.positive(t, "deck_length_m");
            s.deck_width_m = doc.positive(t, "deck_width_m");
            s.freeboard_m = doc.positive(t, "freeboard_m");
            if (s.deck_width_m > s.deck_length_m) doc.fail("drone ship '" + s.id + "': deck_width_m > deck_length_m");
            return s;
        }
    }
    doc.fail("drone ship '" + std::string(ship_id) + "' not defined");
}

}  // namespace starsim::config
