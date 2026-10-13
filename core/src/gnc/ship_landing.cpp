#include "starsim/gnc/ship_landing.hpp"

namespace starsim::gnc {

namespace {

constexpr double deg = constants::deg_to_rad;

}  // namespace

ShipLandingParameters ship_landing_parameters() {
    ShipLandingParameters p;
    auto& s = p.scvx;
    s.snapshot.phase_groups = {std::vector<std::string>{"ship/sea_level_1", "ship/sea_level_2", "ship/sea_level_3"},
                               std::vector<std::string>{"ship/sea_level_1", "ship/sea_level_2"},
                               std::vector<std::string>{"ship/sea_level_1"}};
    auto& l = s.settings.limits;
    l.tilt_max_rad = {20.0 * deg, 15.0 * deg, 8.0 * deg};  // [E] the more engines, the more gimbal authority
    l.rate_max_radps = {0.3, 0.2, 0.1};                    // [E] the ship is shorter and lighter than the booster
    // [E] 3 -> 2 while still sinking fast, 2 -> 1 calm and slow: one engine barely holds the ship.
    l.switches = {{{0.0, 0.0, 15.0, 60.0}, {3.0 * deg, 0.03, 1.5, 8.0}}};
    l.throttle_margin = 0.03;          // [E] the last engine needs ~96 % to hover
    l.terminal_thrust_margin = 1.0;    // no "gentle end" cap: one engine at minimum is far below the weight
    return p;
}

}  // namespace starsim::gnc
