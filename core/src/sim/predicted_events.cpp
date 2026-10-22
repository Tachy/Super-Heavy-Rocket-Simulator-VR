// Predicted mission events: the future events of the session's active plans (external view, Task 4).

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/sim/mission_event_observer.hpp"
#include "starsim/sim/mission_timeline.hpp"

namespace starsim::sim {

namespace {

// [E] A plan's burn and the commanded burn it became (re-targeted at the command) lie within this; distinct burns of
// one plan are further apart (half an orbit and more).
constexpr double same_burn_s = 600.0;

// Events a body has once: their prediction ends when the event is recorded for that body (a prediction that was a
// little late must not stay beside the recorded event).
bool once_per_body(MissionEventKind kind) {
    switch (kind) {
        case MissionEventKind::Meco:
        case MissionEventKind::StageSeparation:
        case MissionEventKind::Seco:
        case MissionEventKind::Boostback:
        case MissionEventKind::LandingBurn:
        case MissionEventKind::Parachutes:
        case MissionEventKind::Caught:
        case MissionEventKind::Landed:
        case MissionEventKind::Splashdown: return true;
        default: return false;
    }
}

class Collector {
public:
    explicit Collector(const FlightSession& s) : s_(s) {}

    // An event of the controlled body at `epoch`, if it is still ahead (`pending`: of the pending burn).
    void add(MissionEventKind kind, time::Epoch epoch, bool pending = false) {
        if (!pending) {
            add_for(s_.controlled_index(), kind, epoch);
            return;
        }
        if (!(epoch - s_.epoch() > 0.0)) return;
        auto e = s_.burn_event(kind);
        e.epoch = epoch;
        e.predicted = true;
        events_.push_back(std::move(e));
    }
    // An event of body `index` at `epoch`, if it is still ahead and (once per body) not recorded yet.
    // `serial`: the body the event will belong to when it is not body `index` itself (a part separated before it).
    // `past`: an event that has happened but is not decided yet (the catch between the pins' first touch and the body
    // at rest in the arms) - shown at its epoch although that lies behind.
    void add_for(size_t index, MissionEventKind kind, time::Epoch epoch, std::optional<uint32_t> serial = std::nullopt,
                 bool past = false) {
        if (!past && !(epoch - s_.epoch() > 0.0)) return;
        auto e = mission_event(s_, index, kind);
        if (serial) e.body_serial = *serial;
        if (once_per_body(kind) && has_recorded(s_.timeline(), kind, e.body_serial)) return;
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

// [E] MECO forecast: the booster's flow integrated along the throttle schedule in steps of this, at most this far ahead
constexpr double meco_forecast_step_s = 1.0;
constexpr double meco_forecast_horizon_s = 1000.0;

// Seconds until the booster has burned `above_kg`: its actual flow now, scaled along the open loop's throttle schedule
// (the engines running now at the scheduled throttle, within their throttle range). None without flow.
std::optional<double> booster_burn_time_s(const FlightSession& s, const vehicle::Vehicle& vehicle, double above_kg) {
    const auto* guidance = s.ascent_guidance();
    if (!guidance) return std::nullopt;
    double flow_now_kgps = 0.0;
    const auto flows_kgps = vehicle.tank_mass_flows_kgps();  // the engines' actual flow (throttle, spool, dispersion)
    for (const double f : flows_kgps.front()) flow_now_kgps += f;
    const auto& engines = vehicle.stages().front().engines;
    const auto engine_flow_kgps = [&](std::optional<double> throttle) {  // nullopt: each engine's throttle now
        double sum = 0.0;
        for (const auto& e : engines) {
            if (!e.producing_thrust()) continue;
            const auto& cfg = e.performance->config();
            const double f = throttle ? std::clamp(*throttle, cfg.throttle_min, cfg.throttle_max) : e.throttle;
            sum += e.performance->mass_flow_kgps(f);
        }
        return sum;
    };
    const double reference_kgps = engine_flow_kgps(std::nullopt);
    if (!(flow_now_kgps > 0.0) || !(reference_kgps > 0.0)) return std::nullopt;
    const double since_liftoff_s = s.epoch() - guidance->liftoff_epoch();
    double left_kg = std::max(above_kg, 0.0), t_s = 0.0;
    std::optional<double> last_throttle;  // the schedule holds its value over long spans: one engine sum per value
    double flow_kgps = 0.0;
    while (t_s < meco_forecast_horizon_s) {
        const double throttle = guidance->booster_throttle(since_liftoff_s + t_s + 0.5 * meco_forecast_step_s);
        if (!last_throttle || throttle != *last_throttle) {
            last_throttle = throttle;
            flow_kgps = flow_now_kgps * engine_flow_kgps(throttle) / reference_kgps;
        }
        if (!(flow_kgps > 0.0)) return std::nullopt;
        if (flow_kgps * meco_forecast_step_s >= left_kg) return t_s + left_kg / flow_kgps;
        left_kg -= flow_kgps * meco_forecast_step_s;
        t_s += meco_forecast_step_s;
    }
    return std::nullopt;
}

// Ascent: the booster's cutoff by the autopilot's own criterion run forward (the propellant above the cutoff
// propellant, burned along the throttle schedule; the autopilot stages in the MECO's tick), the upper stage's by the
// closed loop.
void add_ascent(const FlightSession& s, Collector& c) {
    const size_t i = s.controlled_index();
    const auto state = s.autopilot().state;
    if (state == AutopilotState::Booster) {
        const auto& cutoff_kg = s.booster_cutoff_propellant_kg();
        if (!cutoff_kg) return;
        const auto& vehicle = s.body(i).vehicle();
        const auto& booster = vehicle.stages().front();
        const double propellant_kg =
            booster.propellant_kg(config::TankContents::Oxidizer) + booster.propellant_kg(config::TankContents::Fuel);
        const auto burn_s = booster_burn_time_s(s, vehicle, propellant_kg - *cutoff_kg);
        if (!burn_s) return;
        const auto meco = s.epoch() + *burn_s;
        c.add_for(i, MissionEventKind::Meco, meco);
        c.add_for(i, MissionEventKind::StageSeparation, meco);
    } else if (state == AutopilotState::UpperStage) {
        if (const auto* g = s.ascent_guidance(); g && g->cutoff_epoch()) {
            c.add_for(i, MissionEventKind::Seco, *g->cutoff_epoch());
        }
    }
}

// Returns and landings of every body: what its return, entry and landing guidance predict.
void add_returns(const FlightSession& s, Collector& c) {
    for (size_t i = 0; i < s.body_count(); ++i) {
        if (const auto* r = s.return_guidance(i)) {
            const auto& f = r->forecast();
            if (f.boostback_cutoff) c.add_for(i, MissionEventKind::Cutoff, *f.boostback_cutoff);
            if (f.landing_ignition) c.add_for(i, MissionEventKind::LandingBurn, *f.landing_ignition);
            if (f.touchdown) {
                c.add_for(i, f.catch_target ? MissionEventKind::Caught : MissionEventKind::Landed, *f.touchdown);
            }
        }
        // pins on the arms, not yet at rest there: the catch at its moment until the tower decides and the observer has
        // recorded it (then the dedupe drops it); a miss or an impact ends it
        const auto outcome = s.landing_outcome(i);
        if (const auto contact = s.catch_contact_epoch(i);
            contact && (outcome == LandingOutcome::Flying || outcome == LandingOutcome::Caught)) {
            c.add_for(i, MissionEventKind::Caught, *contact, std::nullopt, true);
        }
        if (const auto* landing = s.ship_landing(i)) {
            if (const auto at = landing->catch_epoch()) c.add_for(i, MissionEventKind::Caught, *at);
        } else if (const auto* entry = s.entry_guidance(i)) {
            // the gate: a ship flips on its landing burn there, a capsule's drogues open
            if (const auto gate = entry->gate_epoch()) {
                c.add_for(i, entry->capsule() ? MissionEventKind::Parachutes : MissionEventKind::LandingBurn, *gate);
            }
        }
    }
}

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
    add_ascent(s, c);
    add_returns(s, c);

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
            // a capsule still on its trunk: the capsule enters - after the jettison a new body with the next serial (the
            // trunk keeps the stack's). Assumes no other body is created before the jettison (none in a capsule return:
            // the step machine runs the burn, then the jettison); otherwise the serial is off until the jettison.
            const auto step = s.capsule_return_step();
            const bool trunk_on = (step == CapsuleReturnStep::DeorbitBurn || step == CapsuleReturnStep::TrunkJettison) &&
                                  s.body(s.controlled_index()).vehicle().stages().size() > 1;
            c.add_for(s.controlled_index(), MissionEventKind::EntryInterface, plan->entry_interface_epoch,
                      trunk_on ? std::optional<uint32_t>(s.next_body_serial()) : std::nullopt);
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
