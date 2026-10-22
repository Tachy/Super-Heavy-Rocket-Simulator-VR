#include "starsim/sim/mission_timeline.hpp"

#include <algorithm>
#include <utility>

namespace starsim::sim {

const char* to_string(MissionEventKind kind) {
    switch (kind) {
        case MissionEventKind::EngineStart: return "Triebwerksstart";
        case MissionEventKind::Liftoff: return "Liftoff";
        case MissionEventKind::MaxQ: return "Max Q";
        case MissionEventKind::Meco: return "MECO";
        case MissionEventKind::StageSeparation: return "Stufentrennung";
        case MissionEventKind::Seco: return "SECO";
        case MissionEventKind::Ignition: return "Zündung";
        case MissionEventKind::Cutoff: return "Brennschluss";
        case MissionEventKind::Boostback: return "Boostback";
        case MissionEventKind::LandingBurn: return "Landezündung";
        case MissionEventKind::EntryInterface: return "Wiedereintritt";
        case MissionEventKind::Parachutes: return "Fallschirme";
        case MissionEventKind::CrewEjection: return "Ausstieg Crew";
        case MissionEventKind::Docking: return "Andocken";
        case MissionEventKind::Undocking: return "Abkoppeln";
        case MissionEventKind::Caught: return "Gefangen";
        case MissionEventKind::Landed: return "Gelandet";
        case MissionEventKind::Splashdown: return "Wasserung";
        case MissionEventKind::Impact: return "Aufprall";
        case MissionEventKind::Launch: return "Start";
    }
    return "";
}

const char* id_of(MissionEventKind kind) {
    switch (kind) {
        case MissionEventKind::EngineStart: return "engine_start";
        case MissionEventKind::Liftoff: return "liftoff";
        case MissionEventKind::MaxQ: return "max_q";
        case MissionEventKind::Meco: return "meco";
        case MissionEventKind::StageSeparation: return "stage_separation";
        case MissionEventKind::Seco: return "seco";
        case MissionEventKind::Ignition: return "ignition";
        case MissionEventKind::Cutoff: return "cutoff";
        case MissionEventKind::Boostback: return "boostback";
        case MissionEventKind::LandingBurn: return "landing_burn";
        case MissionEventKind::EntryInterface: return "entry_interface";
        case MissionEventKind::Parachutes: return "parachutes";
        case MissionEventKind::CrewEjection: return "crew_ejection";
        case MissionEventKind::Docking: return "docking";
        case MissionEventKind::Undocking: return "undocking";
        case MissionEventKind::Caught: return "caught";
        case MissionEventKind::Landed: return "landed";
        case MissionEventKind::Splashdown: return "splashdown";
        case MissionEventKind::Impact: return "impact";
        case MissionEventKind::Launch: return "launch";
    }
    return "";
}

namespace {
bool is_action(MissionEventKind kind) {
    return kind == MissionEventKind::Ignition || kind == MissionEventKind::Undocking;
}

bool earlier(const MissionEvent& a, const MissionEvent& b) { return a.epoch < b.epoch; }
}  // namespace

void MissionTimeline::reset(bool starts_on_pad) {
    starts_on_pad_ = starts_on_pad;
    scheduled_liftoff_.reset();
    recorded_.clear();
    predicted_.clear();
}

void MissionTimeline::record(MissionEvent event) {
    event.predicted = false;
    const auto it = std::upper_bound(recorded_.begin(), recorded_.end(), event, earlier);
    recorded_.insert(it, std::move(event));
}

void MissionTimeline::set_predicted(std::vector<MissionEvent> events) {
    for (auto& e : events) e.predicted = true;
    std::stable_sort(events.begin(), events.end(), earlier);
    predicted_ = std::move(events);
}

void MissionTimeline::set_scheduled_liftoff(std::optional<time::Epoch> t0) { scheduled_liftoff_ = t0; }

std::optional<time::Epoch> MissionTimeline::t0() const {
    for (const auto& e : recorded_)
        if (e.kind == MissionEventKind::Liftoff) return e.epoch;
    if (starts_on_pad_) return scheduled_liftoff_;
    for (const auto& e : recorded_)
        if (is_action(e.kind)) return e.epoch;
    for (const auto& e : predicted_)
        if (is_action(e.kind)) return e.epoch;
    return std::nullopt;
}

std::optional<double> MissionTimeline::mission_clock_s(time::Epoch now) const {
    const auto zero = t0();
    if (!zero) return std::nullopt;
    return now - *zero;
}

}  // namespace starsim::sim
