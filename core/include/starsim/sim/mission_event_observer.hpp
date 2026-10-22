#pragma once

#include <cstdint>
#include <map>
#include <optional>

#include "starsim/gnc/booster_return.hpp"
#include "starsim/sim/mission_timeline.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::sim {

class FlightSession;

// An event of body `index` of `s` now: its serial and the name of its upper (last) stage.
MissionEvent mission_event(const FlightSession& s, size_t index, MissionEventKind kind);
// `t` holds a past event of this kind for the body `serial`.
bool has_recorded(const MissionTimeline& t, MissionEventKind kind, uint32_t serial);

// Mission events seen from the outside of the flight (external view, Task 3): engine starts and cutoffs, the burns of a
// returning booster (named by its return guidance's phase), Max Q, the entry interface, the parachutes and the landing
// outcomes of every body (a catch, known once the body rests in the arms, stamped with the pins' first touch on them),
// recorded into a MissionTimeline. Events with an exact moment (liftoff, separations - with MECO when the stack's first
// stage separates under thrust (hot staging) -, docking, undocking, crew ejection) are recorded by FlightSession
// directly.
// FlightSession::advance_to calls observe() after every tick and every coast; it samples at most every
// observe_interval_s, so an event is recorded up to that late (after a coast: at the coast's end).
class MissionEventObserver {
public:
    static constexpr double observe_interval_s = 0.1;  // [E] sim seconds between samples while ticking
    static constexpr double max_q_drop_fraction = 0.9;  // [E] Max Q is past once q fell below this x its peak
    static constexpr double max_q_min_pa = 10e3;        // [E] smaller peaks are no Max Q (a ship's q after staging)
    static constexpr double entry_interface_altitude_m = 120e3;  // [S] conventional entry interface
    // [E] a descent through the entry interface is an entry when the orbit's periapsis (above the equatorial radius)
    // lies below this: entries aim at 40-80 km, the lowest insertion orbits (tanker: 120 km perigee) stay above.
    static constexpr double entry_max_periapsis_altitude_m = 100e3;

    void reset();  // forgets every body and the last sample
    // Samples every body (at most every observe_interval_s; always after a coast, which is longer) and records what
    // changed since its last sample. A body seen for the first time only sets its memory.
    void observe(const FlightSession& s, MissionTimeline& t);

private:
    struct BodyMemory {
        int engines_running = 0;
        bool burn_with_upper = false;   // the running burn was seen with an upper stage attached
        bool burn_ascent_upper = false; // the running burn was seen as the controlled body in AutopilotState::UpperStage
        bool seco_done = false;
        std::optional<gnc::ReturnPhase> return_phase;  // of the return guidance (none without)
        double q_peak_pa = 0.0;
        time::Epoch q_peak_epoch;
        bool max_q_done = false;
        bool above_entry_interface = false;
        bool been_above_entry_interface = false;  // ever (Max Q is only tracked before)
        uint64_t sample = 0;                      // the last observe() that saw the body
        bool chutes_out = false;
        bool flying = true;  // landing outcome Flying at the last sample
    };
    void observe_body(const FlightSession& s, size_t index, BodyMemory& m, bool first, MissionTimeline& t) const;

    std::map<uint32_t, BodyMemory> bodies_;  // by body serial
    std::optional<time::Epoch> last_;
    uint64_t sample_ = 0;
};

}  // namespace starsim::sim
