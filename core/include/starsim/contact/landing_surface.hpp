#pragma once

#include <optional>

#include "starsim/config/launch_site_config.hpp"

namespace starsim::contact {

// What carries a landing stage, in its site frame: origin on the support surface at the site (pad centre or deck
// centre), x east, y north, z up. Flat (terrain comes with M8).
struct LandingSurface {
    // A drone ship's deck: a rectangle at z = 0 over the sea at z = -freeboard_m. Held on station, level (no sea state).
    struct Deck {
        double half_length_m = 0.0;  // along the long axis
        double half_width_m = 0.0;
        double heading_rad = 0.0;    // long axis, clockwise from north
        double freeboard_m = 0.0;    // deck above the sea
    };
    std::optional<Deck> deck;              // none: a ground plane at z = 0
    std::optional<config::SeaConfig> sea;  // ground plane only: water beyond the shoreline (still at z = 0)

    struct Support {
        double height_m = 0.0;  // surface below the point
        bool water = false;
    };
    Support below(double east_m, double north_m) const;
};

}  // namespace starsim::contact
