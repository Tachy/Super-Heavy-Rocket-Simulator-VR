#pragma once

#include <span>

#include "starsim/environment/gravity.hpp"
#include "starsim/integrators/integrate.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::orbit {

// Equations of motion of an unpowered spacecraft: y = [r, v] relative to the gravity model's
// central body (GCRF axes); t is seconds after `reference_epoch`.
integrators::OdeFunction<6> ballistic_dynamics(const environment::GravityModel& gravity, time::Epoch reference_epoch);

// Default tolerances for coasting flight: 1 mm / 1 um/s absolute, 1e-12 relative.
integrators::Dop853Options<6> default_coast_options();

struct CoastResult {
    math::StateVector state;  // relative to the central body, GCRF axes
    time::Epoch epoch;
    int event_index = -1;
    long evaluations = 0;
};

// Propagates a coasting spacecraft from `epoch0` to `epoch_end` (forward or backward), stopping
// at the first event. Events receive t relative to epoch0 and the packed state.
CoastResult propagate_coast(const environment::GravityModel& gravity, const math::StateVector& state0,
                            time::Epoch epoch0, time::Epoch epoch_end,
                            std::span<const integrators::Event<6>> events = {},
                            const integrators::Dop853Options<6>& options = default_coast_options());

}  // namespace starsim::orbit
