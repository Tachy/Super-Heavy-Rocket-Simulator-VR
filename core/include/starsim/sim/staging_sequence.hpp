#pragma once

#include <cstdint>
#include <optional>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::vehicle {
class Vehicle;
}

namespace starsim::sim {

// The staging sequence under way (FlightSession::stage(); ascent trajectory and staging, Task 1): MECO, the
// separation and the upper stage's ignition, each at its own tick (config::StagingConfig).
struct StagingSequence {
    config::StagingConfig config;
    uint32_t stack_serial = 0;                        // the stack; after the separation its lower part
    time::Epoch meco_epoch;                           // the cutoff command (the sequence's start)
    std::optional<time::Epoch> upper_ignition_epoch;  // commanded at (hot: in the MECO's tick, joined)
    std::optional<time::Epoch> separation_epoch;
    uint32_t upper_serial = 0;                        // the upper part's body, from the separation on
};

// Seconds from MECO to the separation of `stack` (its lowest stage's [stage.staging]): cold - the configured delay;
// hot - the start-up time of the upper stage's slowest engine (the separation waits for its thrust). 0 without a
// staging config.
double separation_after_meco_s(const vehicle::Vehicle& stack);

// Seconds from MECO to the upper stage's ignition command: cold - the separation delay plus the ignition delay; hot - 0
// (lit in the MECO's tick). 0 without a staging config.
double upper_ignition_after_meco_s(const vehicle::Vehicle& stack);

}  // namespace starsim::sim
