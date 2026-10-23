// Predicted mission events: the future events of the session's active plans (external view, Task 4).

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/math/constants.hpp"
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

// [E] MECO forecast: the booster's ascent run forward in steps of this (the max-Q limiter's law settles within its 1 s
// time constant: 0.5 s keeps the explicit steps stable), at most this far ahead
constexpr double meco_forecast_step_s = 0.5;
constexpr double meco_forecast_horizon_s = 1000.0;
// [E] the forecast's air speed at least this (the gravity-turn rate divides by it; on the pad the stack is at rest)
constexpr double meco_forecast_min_speed_mps = 10.0;
// [E] half step of the forecast's central difference of the air density with height
constexpr double meco_forecast_density_step_m = 50.0;

// Seconds until the booster has burned `above_kg`: its ascent run forward under the max-Q limiter
// (gnc::max_q_thrust_acceleration_mps2, the same law as the guidance) on a point mass in the plane of its air velocity
// - height, air speed, flight-path angle and mass from the state now; the open loop's thrust direction (vertical rise,
// the pitch kick's linear tilt, then the gravity turn along the air velocity); the vehicle's own drag table (power on,
// zero angle of attack), US76 air, central gravity. The limiter's throttle below the operational cap, within the
// engines' range; the flow is the actual flow now (throttle, spool, dispersion) scaled with the engines' flow at that
// throttle. Left out: the Earth rotation's terms (centrifugal ~0.03 m/s^2), lift. None without flow.
std::optional<double> booster_burn_time_s(const FlightSession& s, const vehicle::FlightBody& body, double above_kg) {
    const auto* guidance = s.ascent_guidance();
    if (!guidance) return std::nullopt;
    const auto& vehicle = body.vehicle();
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
    const auto thrust_n = [&](double throttle, double ambient_pa) {  // the engines producing thrust now, spool included
        double sum = 0.0;
        for (const auto& e : engines) {
            if (e.producing_thrust()) sum += e.spool * e.performance->thrust_n(throttle, ambient_pa);
        }
        return sum;
    };

    // the point mass from the state now: height, air speed, its flight-path angle, mass, radius
    const auto& env = s.environment();
    const auto& st = body.state();
    const auto earth = env.earth_orientation.fixed_to_gcrf(s.epoch());
    const math::Vec3 relative = st.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(earth), st.position_gcrf_m);
    const auto geodetic = frames::fixed_to_geodetic(transpose(earth.rotation) * st.position_gcrf_m);
    const math::Vec3 up = earth.rotation * math::Vec3{std::cos(geodetic.latitude_rad) * std::cos(geodetic.longitude_rad),
                                                      std::cos(geodetic.latitude_rad) * std::sin(geodetic.longitude_rad),
                                                      std::sin(geodetic.latitude_rad)};
    const auto& program = guidance->program();
    double h = geodetic.height_m, r = norm(st.position_gcrf_m), m = vehicle.mass_properties().mass_kg;
    const double speed_now = norm(relative);
    double v = std::max(speed_now, meco_forecast_min_speed_mps);
    // rising off the pad: vertical (the direction of a slow air velocity means little)
    double gamma = speed_now < meco_forecast_min_speed_mps ? 0.5 * constants::pi
                                                           : std::asin(std::clamp(dot(relative, up) / speed_now, -1.0, 1.0));
    // the open loop's pitch kick (forecast time; none: still rising vertically)
    std::optional<double> kick_start_s;
    if (const auto kick = guidance->kick_start_epoch()) kick_start_s = *kick - s.epoch();

    const double dt = meco_forecast_step_s;
    double left_kg = std::max(above_kg, 0.0), t_s = 0.0;
    while (t_s < meco_forecast_horizon_s) {
        // thrust elevation above the local horizontal: vertical, tilting through the kick, then along the air velocity
        if (!kick_start_s && v * std::sin(gamma) >= program.vertical_rise_speed_mps) kick_start_s = t_s;
        const double kick_progress = kick_start_s ? (t_s - *kick_start_s) / program.kick_duration_s : 0.0;
        const double thrust_elevation =
            kick_progress < 1.0 ? 0.5 * constants::pi - program.kick_angle_rad * std::max(kick_progress, 0.0) : gamma;
        const double thrust_along = std::cos(thrust_elevation - gamma);
        const double thrust_across = std::sin(thrust_elevation - gamma);
        const auto air = env.atmosphere.at(h);
        const double density = air.density_kg_m3 * env.air_density_factor;
        const double drag_n = density > 0.0 ? -vehicle.aero()
                                                   .compute({v, 0.0, 0.0}, density, air.speed_of_sound_mps, true,
                                                            air.mean_free_path_m)
                                                   .force_body_n.x
                                            : 0.0;
        const double g = s.gm_earth_m3ps2() / (r * r);
        const double density_rate = env.air_density_factor *
                                    (env.atmosphere.at(h + meco_forecast_density_step_m).density_kg_m3 -
                                     env.atmosphere.at(h - meco_forecast_density_step_m).density_kg_m3) /
                                    (2.0 * meco_forecast_density_step_m) * v * std::sin(gamma);
        const double other_mps2 = -drag_n / m - g * std::sin(gamma);
        const double acceleration = gnc::max_q_thrust_acceleration_mps2(program.max_dynamic_pressure_pa, density,
                                                                        density_rate, v, other_mps2);
        const double throttle = std::min(gnc::ascent_throttle_cap(program),
                                         std::isfinite(acceleration) && thrust_along > 0.0
                                             ? gnc::booster_throttle_for_thrust(vehicle, m * acceleration / thrust_along,
                                                                                air.pressure_pa)
                                             : 1.0);
        const double flow_kgps = flow_now_kgps * engine_flow_kgps(throttle) / reference_kgps;
        if (!(flow_kgps > 0.0)) return std::nullopt;
        if (flow_kgps * dt >= left_kg) return t_s + left_kg / flow_kgps;
        const double thrust_mps2 = thrust_n(throttle, air.pressure_pa) / m;
        const double speed_rate = thrust_mps2 * thrust_along + other_mps2;
        const double gamma_rate = (thrust_mps2 * thrust_across - g * std::cos(gamma)) / v + v / r * std::cos(gamma);
        h += v * std::sin(gamma) * dt;
        r += v * std::sin(gamma) * dt;
        v = std::max(v + speed_rate * dt, meco_forecast_min_speed_mps);
        gamma += gamma_rate * dt;
        m -= flow_kgps * dt;
        left_kg -= flow_kgps * dt;
        t_s += dt;
    }
    return std::nullopt;
}

// The upper stage's ignition in a staging sequence: named after the stage above the stack's lowest and belonging to the
// body it becomes at the separation (the next serial) - unless it is the separated upper part `index` itself.
void add_upper_ignition(const FlightSession& s, Collector& c, size_t index, time::Epoch epoch, bool joined) {
    if (!(epoch - s.epoch() > 0.0)) return;
    auto e = mission_event(s, index, MissionEventKind::Ignition);
    if (joined) {
        const auto& stages = s.body(index).vehicle().stages();
        if (stages.size() < 2) return;
        e.body_serial = s.next_body_serial();
        e.body_name = stages[1].config.name;
    }
    e.epoch = epoch;
    e.predicted = true;
    c.push(std::move(e));
}

// Ascent: the booster's cutoff by the autopilot's own criterion run forward (the propellant above the cutoff
// propellant, burned under the max-Q limiter), the staging sequence's separation and upper-stage ignition from it
// (StagingSequence), the upper stage's cutoff by the closed loop.
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
        const auto burn_s = booster_burn_time_s(s, s.body(i), propellant_kg - *cutoff_kg);
        if (!burn_s) return;
        const auto meco = s.epoch() + *burn_s;
        c.add_for(i, MissionEventKind::Meco, meco);
        c.add_for(i, MissionEventKind::StageSeparation, meco + separation_after_meco_s(vehicle));
        if (booster.config.staging) add_upper_ignition(s, c, i, meco + upper_ignition_after_meco_s(vehicle), true);
    } else if (state == AutopilotState::Staging) {
        const auto& seq = s.staging();
        if (!seq) return;
        if (!seq->separation_epoch) {  // joined: the controlled body is the stack
            const auto& vehicle = s.body(i).vehicle();
            const auto upper_ignition = seq->meco_epoch + upper_ignition_after_meco_s(vehicle);
            c.add_for(i, MissionEventKind::StageSeparation, seq->meco_epoch + separation_after_meco_s(vehicle));
            if (!seq->upper_ignition_epoch) add_upper_ignition(s, c, i, upper_ignition, true);
        } else if (!seq->upper_ignition_epoch) {  // cold, separated: the controlled body is the upper stage
            add_upper_ignition(s, c, i, *seq->separation_epoch + seq->config.upper_ignition_delay_s, false);
        }
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
