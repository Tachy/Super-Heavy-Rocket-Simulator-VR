// Mission events seen from the outside of the flight: sampled from the session's bodies (external view, Task 3).

#include "starsim/sim/mission_event_observer.hpp"

#include <algorithm>

#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"

namespace starsim::sim {

namespace {

// An engine start of body `index` by what the body is doing; none for a body with return guidance (its phases name
// the burns: after hot staging the centre engines run on through the flip, so the boostback is no relight).
// A ship on its landing guidance: the first start is its landing burn (flip and catch), a relight adds nothing.
std::optional<MissionEventKind> start_kind(const FlightSession& s, size_t index, const MissionTimeline& t) {
    if (s.body(index).clamped()) return MissionEventKind::EngineStart;
    if (s.return_status(index)) return std::nullopt;
    if (s.ship_landing(index)) {
        if (has_recorded(t, MissionEventKind::LandingBurn, s.body_serial(index))) return std::nullopt;
        return MissionEventKind::LandingBurn;
    }
    return MissionEventKind::Ignition;
}

// The name an event shows: the part that acts (controller ruling, external view review). Engine events and a separation
// name the lowest stage (its engines; the separated part; SECO: the lowest stage of the upper part is the launcher's
// upper stage, a stacked payload above it does not act); the stack's pad events and Max Q name the whole vehicle
// (display name, else its lowest stage); everything else the upper (last) stage - the crewed or docking part.
const std::string& event_name(const vehicle::Vehicle& v, MissionEventKind kind) {
    const auto& stages = v.stages();
    switch (kind) {
        case MissionEventKind::Meco:
        case MissionEventKind::Seco:
        case MissionEventKind::StageSeparation:
        case MissionEventKind::Ignition:
        case MissionEventKind::Cutoff:
        case MissionEventKind::Boostback:
        case MissionEventKind::LandingBurn: return stages.front().config.name;
        case MissionEventKind::EngineStart:
        case MissionEventKind::Liftoff:
        case MissionEventKind::MaxQ:
            if (stages.size() >= 2 && !v.display_name().empty()) return v.display_name();
            return stages.front().config.name;
        default: return stages.back().config.name;
    }
}

// The burn a return guidance phase starts.
std::optional<MissionEventKind> return_burn_kind(gnc::ReturnPhase phase) {
    switch (phase) {
        case gnc::ReturnPhase::Boostback: return MissionEventKind::Boostback;
        case gnc::ReturnPhase::EntryBurn: return MissionEventKind::Ignition;
        case gnc::ReturnPhase::LandingBurn: return MissionEventKind::LandingBurn;
        case gnc::ReturnPhase::Flip:
        case gnc::ReturnPhase::Coast:
        case gnc::ReturnPhase::Landed: return std::nullopt;
    }
    return std::nullopt;
}

std::optional<MissionEventKind> outcome_kind(LandingOutcome outcome) {
    switch (outcome) {
        case LandingOutcome::Flying: return std::nullopt;
        case LandingOutcome::Caught: return MissionEventKind::Caught;
        case LandingOutcome::Landed:
        case LandingOutcome::TippedOver: return MissionEventKind::Landed;
        case LandingOutcome::Splashdown:
        case LandingOutcome::SplashedDown: return MissionEventKind::Splashdown;
        case LandingOutcome::Impact:
        case LandingOutcome::BrokenUp: return MissionEventKind::Impact;
    }
    return std::nullopt;
}

}  // namespace

MissionEvent mission_event(const FlightSession& s, size_t index, MissionEventKind kind) {
    return MissionEvent{kind, s.epoch(), s.body_serial(index), event_name(s.body(index).vehicle(), kind), false};
}

bool has_recorded(const MissionTimeline& t, MissionEventKind kind, uint32_t serial) {
    return std::any_of(t.recorded().begin(), t.recorded().end(),
                       [&](const MissionEvent& e) { return e.kind == kind && e.body_serial == serial; });
}

void MissionEventObserver::reset() {
    bodies_.clear();
    last_.reset();
}

void MissionEventObserver::observe(const FlightSession& s, MissionTimeline& t) {
    const auto now = s.epoch();
    if (last_ && now - *last_ < observe_interval_s && now >= *last_) return;
    last_ = now;
    ++sample_;
    for (size_t i = 0; i < s.body_count(); ++i) {
        const auto [it, first] = bodies_.try_emplace(s.body_serial(i));
        it->second.sample = sample_;
        observe_body(s, i, it->second, first, t);
    }
    for (auto it = bodies_.begin(); it != bodies_.end();) {  // serials are never reused: a removed body is gone
        it = it->second.sample == sample_ ? std::next(it) : bodies_.erase(it);
    }
}

void MissionEventObserver::observe_body(const FlightSession& s, size_t index, BodyMemory& m, bool first,
                                        MissionTimeline& t) const {
    const auto& body = s.body(index);
    const auto tm = s.telemetry(index);
    const bool clamped = body.clamped();
    const bool with_upper = body.vehicle().stages().size() >= 2;
    const bool ascent_upper = index == s.controlled_index() && s.autopilot().state == AutopilotState::UpperStage;
    const uint32_t serial = s.body_serial(index);

    // Engines: starts and cutoffs.
    const int engines = tm.engines_running;
    if (!first && m.engines_running == 0 && engines > 0) {
        if (const auto kind = start_kind(s, index, t)) t.record(mission_event(s, index, *kind));
        m.burn_with_upper = false;
        m.burn_ascent_upper = false;
    } else if (!first && m.engines_running > 0 && engines == 0 && s.engines_commanded(index)) {
        // only an uncommanded shutdown (starved, failed, broken up): FlightSession records a commanded one at its command
        auto kind = MissionEventKind::Cutoff;
        // the stack lifted off with this serial: its lowest stage is the stack's first stage
        if (m.burn_with_upper && !clamped && has_recorded(t, MissionEventKind::Liftoff, serial) &&
            !has_recorded(t, MissionEventKind::Meco, serial)) {
            kind = MissionEventKind::Meco;
        } else if (!m.seco_done && m.burn_ascent_upper) {
            kind = MissionEventKind::Seco;
            m.seco_done = true;
        }
        t.record(mission_event(s, index, kind));
    }
    if (engines > 0) {  // what the running burn was seen as (the autopilot moves on before the engines are down)
        m.burn_with_upper = m.burn_with_upper || with_upper;
        m.burn_ascent_upper = m.burn_ascent_upper || ascent_upper;
    }
    m.engines_running = engines;

    // Return guidance: its phase names the burn that starts.
    if (const auto* r = s.return_status(index)) {
        if (!first && m.return_phase != r->phase) {
            if (const auto kind = return_burn_kind(r->phase)) t.record(mission_event(s, index, *kind));
        }
        m.return_phase = r->phase;
    } else {
        m.return_phase.reset();
    }

    // Max Q: the peak of q while climbing, recorded at its epoch once q has fallen clearly below it. Only on an ascent:
    // never above the entry interface and no entry guidance (a skip or an aerocapture pass climbs at entry q).
    const bool earth = tm.reference_body == ephemeris::Body::Earth;
    if (earth && tm.altitude_m > entry_interface_altitude_m) m.been_above_entry_interface = true;
    if (!m.max_q_done && !m.been_above_entry_interface && !s.entry_status(index)) {
        if (tm.vertical_speed_mps > 0.0 && tm.dynamic_pressure_pa > m.q_peak_pa) {
            m.q_peak_pa = tm.dynamic_pressure_pa;
            m.q_peak_epoch = s.epoch();
        }
        if (m.q_peak_pa >= max_q_min_pa && tm.dynamic_pressure_pa < max_q_drop_fraction * m.q_peak_pa) {
            auto e = mission_event(s, index, MissionEventKind::MaxQ);
            e.epoch = m.q_peak_epoch;
            t.record(std::move(e));
            m.max_q_done = true;
        }
    }

    // Entry interface: down through it after having been above (once per descent), coasting on an orbit whose periapsis
    // lies in the air - not the dip of a low insertion orbit to its perigee, not a powered descent.
    if (earth) {
        if (tm.altitude_m > entry_interface_altitude_m) {
            m.above_entry_interface = true;
        } else if (m.above_entry_interface) {
            m.above_entry_interface = false;
            if (engines == 0 && tm.periapsis_altitude_m < entry_max_periapsis_altitude_m) {
                t.record(mission_event(s, index, MissionEventKind::EntryInterface));
            }
        }
    }

    // Parachutes: the first set out.
    if (const auto* chutes = s.parachute_sequencer(index); chutes && chutes->phase() != gnc::ParachutePhase::Armed) {
        if (!m.chutes_out && !first) t.record(mission_event(s, index, MissionEventKind::Parachutes));
        m.chutes_out = true;
    }

    // Landing outcome: the change from Flying.
    const auto outcome = outcome_kind(s.landing_outcome(index));
    if (m.flying && outcome && !first) t.record(mission_event(s, index, *outcome));
    m.flying = !outcome;
}

}  // namespace starsim::sim
