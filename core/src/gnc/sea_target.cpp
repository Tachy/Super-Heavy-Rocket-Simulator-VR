#include "starsim/gnc/sea_target.hpp"

#include <cmath>
#include <stdexcept>

#include "starsim/frames/geodetic.hpp"

namespace starsim::gnc {

EntryTarget sea_give_up_target(const LandingSite& site, double margin_m) {
    if (!site.sea) throw std::invalid_argument("Aufgabe des Schiffs: der Landeplatz hat keine Seeseite");
    const double sl = std::sin(site.latitude_rad), cl = std::cos(site.latitude_rad);
    const double so = std::sin(site.longitude_rad), co = std::cos(site.longitude_rad);
    const math::Vec3 east{-so, co, 0.0}, north{-sl * co, -sl * so, cl};
    const math::Vec3 seaward = north * std::cos(site.sea->direction_rad) + east * std::sin(site.sea->direction_rad);
    const math::Vec3 pad = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    auto ground = frames::fixed_to_geodetic(pad + seaward * (site.sea->shoreline_distance_m + margin_m));
    ground.height_m = site.height_m;
    return {frames::geodetic_to_fixed(ground), site.height_m, {}};
}

}  // namespace starsim::gnc
