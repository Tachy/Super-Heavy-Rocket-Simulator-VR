#include "starsim/gnc/parachute_sequencer.hpp"

#include <algorithm>

namespace starsim::gnc {

namespace {

constexpr double watch_margin_m = 1000.0;  // [E] ticked from this far above the highest deployment height

bool q_allows(const config::ParachuteConfig& set, double q_pa) {
    return set.max_deploy_dynamic_pressure_pa <= 0.0 || q_pa <= set.max_deploy_dynamic_pressure_pa;
}

}  // namespace

const char* to_string(ParachutePhase phase) {
    switch (phase) {
        case ParachutePhase::Armed: return "bereit";
        case ParachutePhase::Drogues: return "Bremsschirme";
        case ParachutePhase::Mains: return "Hauptschirme";
        case ParachutePhase::Down: return "unten";
    }
    return "?";
}

ParachuteSequencer::ParachuteSequencer(const std::vector<config::ParachuteConfig>& sets) : sets_(sets) {
    for (const auto& s : sets_) (s.role == "drogue" ? drogue_ : main_) = &s;
}

double ParachuteSequencer::watch_height_m() const {
    double h = 0.0;
    for (const auto& s : sets_) h = std::max(h, s.deploy_height_m);
    return h + watch_margin_m;
}

ParachuteActions ParachuteSequencer::update(const ParachuteSequencerInput& in) {
    ParachuteActions a;
    if (phase_ == ParachutePhase::Down) return a;
    if (in.on_surface) {
        a.release_mains = phase_ == ParachutePhase::Mains;
        a.release_drogues = drogues_out_;
        drogues_out_ = false;
        phase_ = ParachutePhase::Down;
        return a;
    }
    if (in.sink_mps <= 0.0 || phase_ == ParachutePhase::Mains) return a;
    if (main_ && in.height_m <= main_->deploy_height_m && q_allows(*main_, in.dynamic_pressure_pa)) {
        a.deploy_mains = true;
        main_height_m_ = in.height_m;
        a.release_drogues = drogues_out_;
        drogues_out_ = false;
        phase_ = ParachutePhase::Mains;
        return a;
    }
    if (phase_ == ParachutePhase::Armed && drogue_ && in.height_m <= drogue_->deploy_height_m &&
        q_allows(*drogue_, in.dynamic_pressure_pa)) {
        a.deploy_drogues = true;
        drogue_height_m_ = in.height_m;
        drogues_out_ = true;
        phase_ = ParachutePhase::Drogues;
    }
    return a;
}

}  // namespace starsim::gnc
