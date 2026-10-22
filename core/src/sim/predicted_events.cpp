// Predicted mission events: the future events of the session's active plans (external view, Task 4).

#include <cmath>
#include <optional>
#include <vector>

#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/mission_event_observer.hpp"
#include "starsim/sim/mission_timeline.hpp"

namespace starsim::sim {

namespace {

// [E] A plan's burn and the commanded burn it became (re-targeted at the command) lie within this; distinct burns of
// one plan are further apart (half an orbit and more).
constexpr double same_burn_s = 600.0;

class Collector {
public:
    explicit Collector(const FlightSession& s) : s_(s) {}

    // An event of the controlled body at `epoch`, if it is still ahead (`pending`: of the pending burn).
    void add(MissionEventKind kind, time::Epoch epoch, bool pending = false) {
        if (!(epoch - s_.epoch() > 0.0)) return;
        auto e = pending ? s_.burn_event(kind) : mission_event(s_, s_.controlled_index(), kind);
        e.epoch = epoch;
        e.predicted = true;
        events_.push_back(std::move(e));
    }
    // A planned burn's ignition (impulsive node epoch) unless it is the pending burn or already commanded.
    void add_burn(time::Epoch node_epoch) {
        for (const auto& c : covered_) {
            if (std::abs(node_epoch - c) < same_burn_s) return;
        }
        add(MissionEventKind::Ignition, node_epoch);
    }
    void add_burn(const gnc::BurnNode& node) { add_burn(node.epoch); }
    void cover(time::Epoch node_epoch) { covered_.push_back(node_epoch); }
    std::vector<MissionEvent> take() { return std::move(events_); }
    void push(MissionEvent e) { events_.push_back(std::move(e)); }

private:
    const FlightSession& s_;
    std::vector<time::Epoch> covered_;  // node epochs of the pending and the commanded burns
    std::vector<MissionEvent> events_;
};

}  // namespace

std::vector<MissionEvent> predicted_mission_events(const FlightSession& s) {
    Collector c(s);
    // refuelling f2: the campaign driver's next launch (mission-wide: the vehicle does not exist yet)
    if (const auto* supervisor = s.supervisor()) {
        if (const auto wake = supervisor->wake_epoch(); wake && *wake - s.epoch() > 0.0) {
            MissionEvent e;
            e.kind = MissionEventKind::Launch;
            e.epoch = *wake;
            e.predicted = true;
            c.push(std::move(e));
        }
    }
    if (!s.active() || s.controlled_index() >= s.body_count()) return c.take();

    // the scheduled liftoff while the ascent autopilot is armed for it
    if (const auto& t0 = s.scheduled_liftoff(); t0 && s.autopilot().state == AutopilotState::PreLaunch) {
        c.add(MissionEventKind::Liftoff, *t0);
    }
    if (const auto liftoff = s.lunar_ascent_liftoff_epoch()) c.add(MissionEventKind::Liftoff, *liftoff);
    if (const auto separation = s.undock_separation_epoch()) c.add(MissionEventKind::Undocking, *separation);

    // The burn the autopilot flies covers the plan's node (pending or lit); a pending one shows its ignition from the
    // executor (a finite burn starts before its impulsive node).
    const auto& autopilot = s.autopilot();
    std::optional<time::Epoch> active_node;
    if (autopilot.state == AutopilotState::Burn && s.burn()) {
        active_node = s.burn()->node().epoch;
        c.cover(*active_node);
        if (!autopilot.burn.ignite && !autopilot.burn.complete) {
            c.add(MissionEventKind::Ignition, s.epoch() + autopilot.burn.time_to_ignition_s, true);
        }
    }

    // Ship / capsule deorbit: while the autopilot flies its burn (pending or lit) or the return is under way.
    if (const auto* plan = s.ship_return_plan(); plan && plan->feasible) {
        const bool flying_it = autopilot.state == AutopilotState::Return ||
                               (active_node && std::abs(*active_node - plan->node.epoch) < same_burn_s);
        if (flying_it) {
            c.add_burn(plan->node);
            c.add(MissionEventKind::EntryInterface, plan->entry_interface_epoch);
        }
    }

    // Lunar flight: the plans' burns not yet commanded.
    for (const auto& b : s.lunar_burns()) c.cover(b.node.epoch);
    if (s.lunar_step() != LunarStep::None) {
        if (const auto* plan = s.lunar_outbound_plan(); plan && plan->feasible) {
            c.add_burn(plan->tli);
            if (plan->powered_flyby) c.add_burn(*plan->powered_flyby);
            c.add_burn(plan->insertion);
        }
        if (const auto* plan = s.lunar_descent_plan(); plan && plan->feasible) {
            if (plan->plane_trim) c.add_burn(*plan->plane_trim);
            c.add_burn(plan->doi);
            c.add(MissionEventKind::Ignition, plan->pdi_epoch);
        }
    }
    if (const auto* plan = s.lunar_return_plan(); plan && plan->feasible && s.lunar_step() != LunarStep::None) {
        if (plan->nrho_departure) c.add_burn(*plan->nrho_departure);
        c.add_burn(plan->tei);
        if (plan->arrival.braked) c.add_burn(plan->arrival.braking_epoch);
        if (plan->arrival.reached) c.add(MissionEventKind::EntryInterface, plan->arrival.interface_epoch);
    }

    // Rendezvous phasing: the plan's burns after the flown ones.
    if (const auto* plan = s.rendezvous_plan(); plan && plan->feasible && s.rendezvous_step() != RendezvousStep::None) {
        const auto& flown = s.rendezvous_burns();
        for (const auto& b : flown) c.cover(b.node.epoch);
        for (const auto& b : plan->burns) {
            if (!flown.empty() && !(b.node.epoch - flown.back().node.epoch > 0.0)) continue;
            c.add_burn(b.node);
        }
    }
    return c.take();
}

}  // namespace starsim::sim
