#include "starsim/sim/staging_sequence.hpp"

#include <algorithm>

#include "starsim/vehicle/vehicle.hpp"

namespace starsim::sim {

namespace {

// Start-up time of the slowest main engine of the stage above the lowest (auxiliary thrusters do not count).
double upper_startup_s(const vehicle::Vehicle& stack) {
    const auto& stages = stack.stages();
    double startup_s = 0.0;
    if (stages.size() < 2) return startup_s;
    for (const auto& e : stages[1].engines) {
        if (!e.auxiliary()) startup_s = std::max(startup_s, e.performance->config().startup_s);
    }
    return startup_s;
}

}  // namespace

double separation_after_meco_s(const vehicle::Vehicle& stack) {
    const auto& staging = stack.stages().front().config.staging;
    if (!staging) return 0.0;
    return staging->kind == config::StagingConfig::Kind::Hot ? upper_startup_s(stack) : staging->separation_delay_s;
}

double upper_ignition_after_meco_s(const vehicle::Vehicle& stack) {
    const auto& staging = stack.stages().front().config.staging;
    if (!staging || staging->kind == config::StagingConfig::Kind::Hot) return 0.0;
    return staging->separation_delay_s + staging->upper_ignition_delay_s;
}

}  // namespace starsim::sim
