#pragma once

#include <optional>
#include <vector>

#include "starsim/config/vehicle_config.hpp"

namespace starsim::gnc {

enum class ParachutePhase { Armed, Drogues, Mains, Down };
const char* to_string(ParachutePhase phase);

struct ParachuteSequencerInput {
    double height_m = 0.0;             // above the ground / sea
    double dynamic_pressure_pa = 0.0;
    double sink_mps = 0.0;             // > 0 descending
    bool on_surface = false;           // resting on the ground / water
};

struct ParachuteActions {
    bool deploy_drogues = false;
    bool release_drogues = false;
    bool deploy_mains = false;
    bool release_mains = false;
};

// Parachute deployment of a descending capsule (d4; Crew Dragon [S]: drogues ~5.5 km, swapped for the mains at ~1.8 km).
// Triggers are the height plus, for a set with max_deploy_dynamic_pressure_pa, a dynamic-pressure check (the
// deployment waits above it). The mains are the backup: they go at their height whatever the drogues did (the
// drogues are released only if they are out). On the surface the mains are released. Only descending flight deploys.
class ParachuteSequencer {
public:
    explicit ParachuteSequencer(const std::vector<config::ParachuteConfig>& sets);
    ParachuteSequencer(const ParachuteSequencer&) = delete;
    ParachuteSequencer& operator=(const ParachuteSequencer&) = delete;

    ParachuteActions update(const ParachuteSequencerInput& in);
    ParachutePhase phase() const { return phase_; }
    // Below this height the session ticks (no analytic coast past a deployment height).
    double watch_height_m() const;
    std::optional<double> drogue_height_m() const { return drogue_height_m_; }  // where the drogues were deployed
    std::optional<double> main_height_m() const { return main_height_m_; }

private:
    std::vector<config::ParachuteConfig> sets_;
    const config::ParachuteConfig* drogue_ = nullptr;  // into sets_
    const config::ParachuteConfig* main_ = nullptr;
    ParachutePhase phase_ = ParachutePhase::Armed;
    bool drogues_out_ = false;
    std::optional<double> drogue_height_m_, main_height_m_;
};

}  // namespace starsim::gnc
