#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "starsim/time/epoch.hpp"

namespace starsim::sim {

enum class MissionEventKind {
    EngineStart,
    Liftoff,
    MaxQ,
    Meco,
    StageSeparation,
    Seco,
    Ignition,
    Cutoff,
    Boostback,
    LandingBurn,
    EntryInterface,
    Parachutes,
    CrewEjection,
    Docking,
    Undocking,
    Caught,
    Landed,
    Splashdown,
    Impact,
    Launch  // predicted liftoff of a later launch, e.g. the next tanker
};

// German display label of an event kind.
const char* to_string(MissionEventKind kind);
// Stable snake_case identifier of an event kind ("max_q", "stage_separation"), for scripts and data.
const char* id_of(MissionEventKind kind);

struct MissionEvent {
    MissionEventKind kind = MissionEventKind::Liftoff;
    time::Epoch epoch;
    uint32_t body_serial = 0;  // 0: mission-wide
    std::string body_name;     // German display name of the body at the event (upper stage name)
    bool predicted = false;
};

// Past and predicted mission events plus the T-0 rules. Pure data; the flight session feeds it.
class MissionTimeline {
public:
    void reset(bool starts_on_pad);
    void record(MissionEvent event);  // past event (predicted forced false), kept sorted by epoch
    void set_predicted(std::vector<MissionEvent> events);  // replaces all predicted events
    void set_scheduled_liftoff(std::optional<time::Epoch> t0);

    const std::vector<MissionEvent>& recorded() const { return recorded_; }
    const std::vector<MissionEvent>& predicted() const { return predicted_; }

    // T-0 in order: first recorded Liftoff; on a pad mission the scheduled liftoff; otherwise the first recorded
    // Ignition / Undocking, then the earliest predicted one; else none.
    std::optional<time::Epoch> t0() const;
    std::optional<double> mission_clock_s(time::Epoch now) const;  // now - t0; negative = T-

private:
    bool starts_on_pad_ = false;
    std::optional<time::Epoch> scheduled_liftoff_;
    std::vector<MissionEvent> recorded_;
    std::vector<MissionEvent> predicted_;
};

class FlightSession;

// The future events of the session's active plans (predicted; FlightSession::advance_to stores them in its timeline after every
// call): the scheduled liftoff and a lunar ascent's liftoff (Liftoff), the pending burn's ignition (Ignition, from the
// burn executor), the ship / capsule deorbit burn and entry interface, the burns of the lunar outbound, descent and
// return plans and of the rendezvous plan not yet commanded (Ignition at the impulsive node; the return's entry
// interface), the docked pair's separation (Undocking: the undocking epoch plus the hooks' opening time) and the
// campaign supervisor's next launch (Launch, mission-wide). Only events after the session epoch.
std::vector<MissionEvent> predicted_mission_events(const FlightSession& s);

}  // namespace starsim::sim
