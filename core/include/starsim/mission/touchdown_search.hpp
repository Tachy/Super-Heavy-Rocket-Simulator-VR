#pragma once

#include <future>
#include <memory>
#include <optional>
#include <vector>

#include "starsim/environment/detailed_lunar_terrain.hpp"
#include "starsim/mission/lunar_descent.hpp"

namespace starsim::mission {

// The touchdown search with the hazard check on a background thread (follow-up U-7: a refused site checks every
// candidate of the search, 16-30 s at IM-2, which held the mission start): select_touchdown_point(surface, ...) for each
// approach azimuth in turn, the first admissible point returned (std::nullopt: refused for every azimuth) - the same
// point as the synchronous calls, bit for bit. The search keeps the surface alive (shared ownership) and copies the
// parameters, so the caller may drop its references meanwhile. It touches only the surface (no CSPICE, no session):
// the session's thread goes on while it runs. The future's destructor waits for the search (std::async semantics), so
// a dropped search finishes before its thread is gone. Exceptions of the search come out of get().
std::future<std::optional<TouchdownPoint>> select_touchdown_point_async(
    std::shared_ptr<const environment::DetailedLunarTerrain> surface, double latitude_rad, double longitude_rad,
    std::vector<double> approach_azimuths_rad, double footprint_radius_m, LunarDescentParameters parameters,
    TouchdownHazardLimits hazards);

}  // namespace starsim::mission
