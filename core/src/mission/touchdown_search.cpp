#include "starsim/mission/touchdown_search.hpp"

#include <stdexcept>
#include <utility>

namespace starsim::mission {

std::future<std::optional<TouchdownPoint>> select_touchdown_point_async(
    std::shared_ptr<const environment::DetailedLunarTerrain> surface, double latitude_rad, double longitude_rad,
    std::vector<double> approach_azimuths_rad, double footprint_radius_m, LunarDescentParameters parameters,
    TouchdownHazardLimits hazards) {
    if (!surface) throw std::invalid_argument("select_touchdown_point_async: keine Mondoberfläche");
    return std::async(std::launch::async, [surface = std::move(surface), latitude_rad, longitude_rad,
                                           azimuths = std::move(approach_azimuths_rad), footprint_radius_m,
                                           parameters = std::move(parameters), hazards = std::move(hazards)] {
        for (const double azimuth : azimuths) {
            if (auto point = select_touchdown_point(*surface, latitude_rad, longitude_rad, azimuth, footprint_radius_m,
                                                    parameters, hazards)) {
                return point;
            }
        }
        return std::optional<TouchdownPoint>{};
    });
}

}  // namespace starsim::mission
