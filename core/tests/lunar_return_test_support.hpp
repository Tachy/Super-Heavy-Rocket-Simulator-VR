// Shared set-up of the lunar return tests (l5): a session with the Moon's orientation and the mission's vehicle with
// overrides.
#pragma once

#include <functional>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

namespace starsim::test {

// A flight session with the Earth and the Moon (MOON_ME orientation), as start_mission needs for lunar missions.
struct LunarReturnSession {
    bool kernels = (spice_session(), true);  // before the members that read them
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
};

// The Block 3 vehicle with `tweak` applied to its ship stage (a vehicle override for start_lunar_return_mission).
inline config::VehicleConfig block3_with_ship(const std::function<void(config::StageConfig&)>& tweak) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    for (auto& stage : cfg.stages) {
        if (stage.id == "ship") tweak(stage);
    }
    return cfg;
}

}  // namespace starsim::test
