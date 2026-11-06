// FlightSession: ship entry, return and landing; booster return and catch.

#include "starsim/sim/flight_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <utility>

#include "starsim/config/mission_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/entry_residue_limit.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/sim/refuelling_mission.hpp"

namespace starsim::sim {

namespace {

constexpr double tower_keep_out_margin_m = 5.0;  // [E] aborted landings stay this far from the tower lattice

}  // namespace

void FlightSession::set_booster_return(bool enabled, gnc::BoosterReturnParameters parameters) {
    if (enabled) {
        booster_return_ = std::move(parameters);
    } else {
        booster_return_.reset();
    }
}

void FlightSession::end_booster_return(size_t index) {
    auto& flying = bodies_.at(index);
    if (!flying.return_guidance) return;
    flying.return_guidance.reset();
    flying.return_command = {};
    for (auto& [group, run] : flying.running_groups) run = false;
}

const gnc::ReturnCommand* FlightSession::return_status(size_t index) const {
    const auto& flying = bodies_.at(index);
    return flying.return_guidance ? &flying.return_command : nullptr;
}

std::optional<time::Epoch> FlightSession::catch_contact_epoch(size_t index) const {
    if (!tower_) return std::nullopt;
    return tower_->first_contact_epoch(*bodies_.at(index).body);
}

const gnc::EntryCommand* FlightSession::entry_status(size_t index) const {
    const auto& flying = bodies_.at(index);
    return flying.entry_guidance ? &flying.entry_command : nullptr;
}

void FlightSession::place_entering_ship(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                                        time::Epoch epoch, std::optional<gnc::EntryGuidanceParameters> parameters) {
    place_body(config, state, epoch);
    start_entry_guidance(bodies_.front(), parameters ? *parameters
                                                     : gnc::entry_guidance_parameters(config, config.stages.front(),
                                                                                      gnc::EntryKind::Orbital));
}

void FlightSession::start_entry_guidance(Flying& flying, const gnc::EntryGuidanceParameters& parameters) {
    flying.entry_hull_moment_scale = 1.0;  // a new pass estimates afresh
    flying.entry_attitude_bias_nm = {};
    const auto& stage = flying.body->vehicle().stages().front().config;
    if (stage.aero.entry_profile == "capsule") {
        flying.entry_guidance = std::make_unique<gnc::EntryGuidance>(*flying.body, capsule_entry_target(), parameters);
        return;
    }
    const auto& schedule = *alpha_schedule(stage.aero.entry_alpha_table);
    // gate ahead of the tower's catch axis (b3), else over the pad centre
    const auto site = return_site(flying.body->vehicle());
    auto landing = gnc::ship_landing_parameters(flying.body->vehicle());
    const auto target = gnc::ship_entry_target(site, landing);
    flying.entry_guidance =
        std::make_unique<gnc::EntryGuidance>(*flying.body, schedule, target, parameters, radiative_heating_);
    // the terminal phase moves the gate with the lead the predicted gate state needs: the flip flown forward with this
    // session's ship-landing controller and tick (gnc::ShipGateAim)
    flying.gate_aim.reset();
    if (site.catch_target) {
        auto aim = std::make_shared<gnc::ShipGateAim>(site, std::move(landing), ship_landing_controller_, tick_s);
        // real-time play: the prediction flies on a worker with an environment of its own (built here, on the sim thread)
        aim->set_worker_environment([this] { return worker_environment(); });
        aim->set_realtime(realtime_planning_);
        flying.entry_guidance->set_terminal_aim(
            [aim](const gnc::EntryGuidance::TerminalGate& gate) { return (*aim)(gate); });
        flying.gate_aim = std::move(aim);
    }
}

namespace {

FlightSession::MainPropellant main_tanks(const vehicle::Vehicle& vehicle) {
    FlightSession::MainPropellant p;
    for (const auto& stage : vehicle.stages()) {
        p.oxidizer_kg += stage.propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main);
        p.fuel_kg += stage.propellant_kg(config::TankContents::Fuel, config::TankRole::Main);
    }
    return p;
}

}  // namespace

FlightSession::MainPropellant FlightSession::main_propellant_at(const vehicle::FlightBody& body, time::Epoch at) const {
    return main_propellant_flown(body, env_, epoch_, at);
}

FlightSession::MainPropellant FlightSession::main_propellant_flown(const vehicle::FlightBody& body,
                                                                   const vehicle::FlightEnvironment& env,
                                                                   time::Epoch from, time::Epoch at) {
    if (!env.boil_off || env.ephemeris == nullptr) return main_tanks(body.vehicle());
    // Waiting for its burn the ship drifts free (burn_drifting) and turns under the air's torque - which side the Sun
    // heats changes the boil-off by up to ~2x (r2 campaigns), so no fixed-attitude average will do: a copy is coasted.
    vehicle::FlightBody ahead(body.vehicle().copy_upper(0), body.state());
    ahead.set_ground_height(body.ground_height_m());
    for (time::Epoch t = from; t < at;) {
        const double dt = std::min(max_coast_s, at - t);
        ahead.coast(env, t, dt);
        t = t + dt;
    }
    const auto later = main_tanks(ahead.vehicle());
    if (std::getenv("STARSIM_DEBUG_DEORBIT")) {
        const auto now = main_tanks(body.vehicle());
        std::printf("[deorbit] flown ahead %.1f h to %.0f s TDB: main LOX %.2f -> %.2f t, CH4 %.2f -> %.2f t\n",
                    (at - from) / 3600.0, at.tdb_seconds_past_j2000(), now.oxidizer_kg / 1000.0,
                    later.oxidizer_kg / 1000.0, now.fuel_kg / 1000.0, later.fuel_kg / 1000.0);
    }
    return later;
}

double FlightSession::drag_area_per_mass_ahead() const {
    // Fix round (planner Task 6): a drag-free first plan of b4 from the aircatch orbit took a pass 111 h ahead at 56.8 km
    // planned crossrange; with the ship's real drag (tracked 2.4-2.6e-3 m2/kg at 217 t) that pass lay 125 km across, out
    // of reach - the re-plans found none in their windows, the old burn was flown and the entry hit the ground 950 km
    // short. The planner measures the drag the tracker will measure: on the ship itself, flown ahead.
    return drag_area_per_mass_ahead(*bodies_[controlled_].body);
}

double FlightSession::drag_area_per_mass_ahead(const vehicle::FlightBody& body) const {
    return drag_area_per_mass_flown(body, env_, gravity_, epoch_);
}

double FlightSession::drag_area_per_mass_flown(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                               const environment::GravityModel& gravity, time::Epoch from) {
    vehicle::FlightEnvironment nominal = env;
    nominal.air_density_factor = 1.0;  // the planner's air (US76), not the truth's
    vehicle::FlightBody ahead(body.vehicle().copy_upper(0), body.state());
    ahead.set_ground_height(body.ground_height_m());
    const time::Epoch end = from + deorbit_track_interval_s;
    for (time::Epoch t = from; t < end;) {
        const double dt = std::min(max_coast_s, end - t);
        ahead.coast(nominal, t, dt);
        t = t + dt;
    }
    const auto& s0 = body.state();
    const auto& s1 = ahead.state();
    const double beta = gnc::estimate_drag_area_per_mass(gravity, {s0.position_gcrf_m, s0.velocity_gcrf_mps}, from,
                                                         {s1.position_gcrf_m, s1.velocity_gcrf_mps}, end, 0.0);
    if (std::getenv("STARSIM_DEBUG_DEORBIT")) {
        std::printf("[deorbit] drag flown ahead %.1f h: C_D A / m %.3e m2/kg (%.1f t)\n", deorbit_track_interval_s / 3600.0,
                    beta, body.vehicle().mass_properties().mass_kg / 1000.0);
    }
    return beta;
}

report::Report FlightSession::boil_off_shortfall(gnc::DeorbitPlan& plan, double margin) const {
    const auto& body = *bodies_[controlled_].body;
    return boil_off_shortfall(plan, margin, main_tanks(body.vehicle()), main_propellant_at(body, plan.node.epoch));
}

report::Report FlightSession::boil_off_shortfall(gnc::DeorbitPlan& plan, double margin, const MainPropellant& now,
                                                 const MainPropellant& at_burn) const {
    const auto& body = *bodies_[controlled_].body;
    plan.boil_off_kg = now.total() - at_burn.total();
    // LOX and CH4 boil off in another ratio than the engine burns them (~3.2 against 3.6): the one that runs dry first
    // limits the burn. The deorbit engine set's mixture ratio (r2 minor: not just the last engine's).
    const auto& stage = body.vehicle().stages().front();
    double mixture_ratio = 0.0;
    for (const auto& e : stage.engines) {
        const bool deorbit_engine = std::any_of(
            stage.config.deorbit_engine_groups.begin(), stage.config.deorbit_engine_groups.end(),
            [&](const std::string& g) { return e.group_id == stage.config.id + "/" + g; });
        if (deorbit_engine || (mixture_ratio == 0.0 && !e.auxiliary())) mixture_ratio = e.performance->config().mixture_ratio;
        if (deorbit_engine) break;
    }
    const double usable_kg = mixture_ratio > 0.0 ? std::min(at_burn.oxidizer_kg * (1.0 + mixture_ratio) / mixture_ratio,
                                                            at_burn.fuel_kg * (1.0 + mixture_ratio))
                                                 : at_burn.total();
    // f3: the settling push before the burn takes its gas from the main tanks
    const double settling_kg = settling_gas_kg();
    if (usable_kg >= margin * plan.propellant_kg + settling_kg) return {};
    return report::make(report::Code::deorbit_boil_off_short,
                        {report::mass("deorbit", plan.propellant_kg), report::mass("settling", settling_kg),
                         report::time("wait", plan.node.epoch - epoch_), report::mass("usable", usable_kg),
                         report::mass("boiled", plan.boil_off_kg)});
}

gnc::DeorbitPlan FlightSession::ship_deorbit_plan(const gnc::DeorbitParameters& given, double* drag_used_m2pkg) const {
    gnc::DeorbitPlan refused;
    if (bodies_.empty()) {
        refused.reason = report::make(report::Code::return_no_vehicle);
        return refused;
    }
    const auto& flying = bodies_[controlled_];
    if (flying.entry_guidance || flying.ship_landing) {
        refused.reason = report::make(report::Code::return_no_ship_in_orbit);
        return refused;
    }
    return ship_deorbit_plan(*flying.body, given, drag_used_m2pkg);
}

gnc::DeorbitPlan FlightSession::ship_deorbit_plan(const vehicle::FlightBody& body, const gnc::DeorbitParameters& given,
                                                  double* drag_used_m2pkg) const {
    gnc::DeorbitPlan refused;
    auto inputs = deorbit_plan_inputs(body, given, refused);
    if (!inputs) return refused;
    // no drag given (the first plan): the ship's own, flown ahead
    inputs->measure_drag = true;
    auto plan = plan_from_inputs(*inputs, gravity_, earth_, env_);
    if (drag_used_m2pkg) *drag_used_m2pkg = inputs->parameters.drag_area_per_mass_m2pkg;
    return plan;
}

std::optional<FlightSession::DeorbitReplanInputs> FlightSession::deorbit_plan_inputs(const vehicle::FlightBody& body,
                                                                                     const gnc::DeorbitParameters& given,
                                                                                     gnc::DeorbitPlan& refused) const {
    const auto& vehicle = body.vehicle();
    const auto& stage = vehicle.stages().front().config;
    if (stage.aero.entry_alpha_table.empty() || stage.deorbit_engine_groups.empty()) {
        refused.reason = report::make(report::Code::return_no_ship_in_orbit);
        return std::nullopt;
    }
    if (!stage.ship_landing) {  // Task 3 M5: a refusal, not an exception on the sim thread
        refused.reason = report::make(report::Code::return_no_landing_parameters);
        return std::nullopt;
    }
    const auto configurations = burn_configurations(&stage.deorbit_engine_groups);
    if (configurations.empty()) {
        refused.reason = report::make(report::Code::deorbit_engine_unavailable);
        return std::nullopt;
    }
    DeorbitReplanInputs in;
    const auto& s = body.state();
    in.epoch = epoch_;
    in.state = {s.position_gcrf_m, s.velocity_gcrf_mps};
    in.exhaust_velocity_mps = configurations.front().thrust_n / configurations.front().mass_flow_kgps;
    in.burn_thrust_n = configurations.front().thrust_n;
    in.burn_lead_s = burn_lead_s();
    in.schedule = alpha_schedule(stage.aero.entry_alpha_table);
    in.target = gnc::ship_entry_target(return_site(vehicle), gnc::ship_landing_parameters(vehicle));
    in.parameters = given;
    in.parameters.burn_dispersion = gnc::deorbit_burn_dispersion(stage);  // its cutoff residual and pointing error
    in.landing_propellant_kg = stage.landing_propellant_kg;
    set_inputs_body(in, body);
    in.generation = deorbit_generation_;
    return in;
}

void FlightSession::set_inputs_body(DeorbitReplanInputs& in, const vehicle::FlightBody& body) {
    const auto& vehicle = body.vehicle();
    const auto& stage = vehicle.stages().front().config;
    const auto guidance = gnc::entry_guidance_parameters(vehicle.source_config(), stage, gnc::EntryKind::Orbital);
    in.onboard = std::make_shared<const gnc::EntryGuidance>(body, *in.schedule, in.target, guidance);
    // Task 12b: main-tank residue the flaps cannot trim is vented before the interface; its limit and the vented ship's
    // onboard model are derived by plan_from_inputs (the planner's thread, final fix I1)
    in.vent_rate_kgps = stage.entry_vent_rate_kgps;
    in.max_main_propellant_kg = std::numeric_limits<double>::infinity();
    in.main_propellant_kg = vehicle.main_propellant_kg();
    in.parameters.reach_check = gnc::entry_reach_check(*in.onboard);  // the onboard guidance's rules and limits
    in.mass_kg = vehicle.mass_properties().mass_kg;
    in.propellant_kg = vehicle.propellant_kg();
    // the flights ahead (drag, boil-off) start from a copy of the ship as it is (its ground height)
    auto copy = std::make_shared<vehicle::FlightBody>(vehicle.copy_upper(0), body.state());
    copy->set_ground_height(body.ground_height_m());
    in.copy = std::move(copy);
}

gnc::DeorbitPlan FlightSession::plan_from_inputs(DeorbitReplanInputs& in, const environment::GravityModel& gravity,
                                                 const frames::BodyOrientation& earth,
                                                 const vehicle::FlightEnvironment& env) {
    if (in.measure_drag && !(in.parameters.drag_area_per_mass_m2pkg > 0.0)) {
        in.parameters.drag_area_per_mass_m2pkg = drag_area_per_mass_flown(*in.copy, env, gravity, in.epoch);
    }
    const auto as_is = [&] {
        return gnc::plan_deorbit(in.state, in.epoch, gravity, earth, in.onboard->model(), in.target, in.mass_kg,
                                 in.exhaust_velocity_mps, in.propellant_kg, in.landing_propellant_kg, in.parameters);
    };
    // Task 12b: the burn leaves more main-tank residue than the flaps trim - the ship vents the excess before the
    // interface, so the entry is planned again with the vented mass and the vented ship's trim. A return that stays
    // within the limit is planned as it is (one plan). The limit (a bisection over EntryAeroTable builds) and the vented
    // onboard model are derived here, on the planner's thread (final fix I1). The decision stays on the as-is plan's
    // burn: deciding on the vented plan's own burn (one plan fewer) moved the phase-45 return's gate from 35 to 305 m
    // (its burn leaves the main tanks just around the limit; final fix measure 2026-11-04).
    const auto plan = as_is();
    if (!(in.vent_rate_kgps > 0.0) || !(plan.feasible || plan.short_of_reserve)) return plan;
    const auto& ship = in.copy->vehicle();
    if (!(in.max_main_propellant_kg < std::numeric_limits<double>::infinity())) {
        in.max_main_propellant_kg = gnc::entry_residue_limit(ship, *in.schedule).max_main_propellant_kg;
    }
    if (in.main_propellant_kg - plan.propellant_kg <= in.max_main_propellant_kg) return plan;
    const auto& stage = ship.stages().front().config;
    const auto guidance = gnc::entry_guidance_parameters(ship.source_config(), stage, gnc::EntryKind::Orbital);
    auto vented_ship = ship.copy_upper(0);
    gnc::draw_main_tanks(*vented_ship, in.main_propellant_kg - in.max_main_propellant_kg);
    const vehicle::FlightBody vented_body(std::move(vented_ship), in.copy->state());
    const gnc::EntryGuidance onboard_vented(vented_body, *in.schedule, in.target, guidance);
    auto vented = in.parameters;
    vented.max_entry_mass_kg = in.mass_kg - (in.main_propellant_kg - in.max_main_propellant_kg);
    auto result = gnc::plan_deorbit(in.state, in.epoch, gravity, earth, onboard_vented.model(), in.target, in.mass_kg,
                                    in.exhaust_velocity_mps, in.propellant_kg, in.landing_propellant_kg, vented);
    if (!(result.feasible || result.short_of_reserve) || !in.check_vent_fit) return result;
    const double excess_kg = result.entry_vent_kg;
    // Final fix I2: the vent must end before the entry guidance starts. What the coast from the burn to it cannot take
    // is vented while the ship waits for its burn (set_entry_vent_plan); a pass whose wait is too short for that share
    // is passed over for a later one - the burn pushed back by at least the missing vent time.
    for (int attempt = 0; attempt < vent_pass_attempts && (result.feasible || result.short_of_reserve); ++attempt) {
        const double missing_s = vent_time_missing_s(in, result);
        if (!(missing_s > 0.0)) return result;
        vented.lead_s = (result.node.epoch - in.epoch) + missing_s;
        result = gnc::plan_deorbit(in.state, in.epoch, gravity, earth, onboard_vented.model(), in.target, in.mass_kg,
                                   in.exhaust_velocity_mps, in.propellant_kg, in.landing_propellant_kg, vented);
    }
    if ((result.feasible || result.short_of_reserve) && !(vent_time_missing_s(in, result) > 0.0)) return result;
    gnc::DeorbitPlan refused;
    refused.reason = report::make(report::Code::deorbit_vent_impossible,
                                  {report::mass("excess", excess_kg), report::number("rate", in.vent_rate_kgps)});
    return refused;
}

double FlightSession::vent_time_missing_s(const DeorbitReplanInputs& in, const gnc::DeorbitPlan& plan) {
    if (!(plan.entry_vent_kg > 0.0) || !(in.vent_rate_kgps > 0.0)) return 0.0;
    const double half_burn_s =
        in.burn_thrust_n > 0.0
            ? 0.5 * gnc::burn_duration_s(plan.delta_v_mps, in.mass_kg, in.burn_thrust_n, in.burn_thrust_n / in.exhaust_velocity_mps)
            : 0.0;
    // from the burn's end to the entry guidance (entry_start_epoch: the interface less entry_lead_s), and from the
    // snapshot to the burn's preparation
    const double after_s = std::max(0.0, plan.entry_interface_epoch - entry_lead_s - (plan.node.epoch + half_burn_s));
    const double before_s = std::max(0.0, plan.node.epoch - half_burn_s - in.burn_lead_s - in.epoch);
    return plan.entry_vent_kg / in.vent_rate_kgps - after_s - before_s;
}

void FlightSession::log_dispersion_note(const gnc::DeorbitPlan& plan) {
    if (plan.feasible && !plan.dispersion_robust) std::printf("[deorbit] %s\n", report::describe(plan.dispersion_note).c_str());
}

gnc::DeorbitPlan FlightSession::ship_return_preview(const gnc::DeorbitParameters& parameters) const {
    return ship_deorbit_plan(parameters);
}

std::optional<gnc::DeorbitParameters> FlightSession::ship_return_settings(report::Report* reason) const {
    if (iss_mission_ && iss_mission_->ship_return) return *iss_mission_->ship_return;  // the mission's own return
    return vehicle_return_settings(reason);
}

std::optional<gnc::DeorbitParameters> FlightSession::vehicle_return_settings(report::Report* reason) const {
    const std::string mission_id = bodies_.empty() ? std::string() : bodies_[controlled_].body->vehicle().stages().front().config.return_mission;
    if (mission_id.empty()) {
        if (reason) *reason = report::make(report::Code::return_no_return_mission);
        return std::nullopt;
    }
    const auto m = config::load_ship_return_mission(config_dir_, mission_id);
    auto parameters = ship_deorbit_parameters(m.deorbit);  // the one mapping (refuelling r5)
    parameters.search_window_s = m.deorbit.search_window_s;
    return parameters;
}

bool FlightSession::plan_ship_return() { return *begin_ship_return(false); }

bool FlightSession::plan_ship_return(const gnc::DeorbitParameters& parameters) {
    return *begin_ship_return(parameters, false);
}

std::optional<bool> FlightSession::begin_ship_return(bool background) {
    report::Report why;
    const auto parameters = vehicle_return_settings(&why);
    if (!parameters) {
        ship_return_ = false;
        ++deorbit_generation_;
        ship_return_task_.discard();
        ++ship_return_request_;
        ship_return_outcome_ = false;
        ship_return_then_ = {};
        deorbit_plan_ = gnc::DeorbitPlan{};
        deorbit_plan_->reason = why;
        reset_entry_vent();
        return false;
    }
    return begin_ship_return(*parameters, background);
}

std::optional<bool> FlightSession::begin_ship_return(const gnc::DeorbitParameters& parameters, bool background) {
    ++ship_return_request_;  // a pending earlier request is replaced (its poll reports it cancelled)
    ship_return_outcome_.reset();
    ship_return_then_ = {};
    const auto submitted = submit_ship_return(parameters, background);
    if (submitted) ship_return_outcome_ = *submitted;
    return submitted;
}

std::optional<bool> FlightSession::submit_ship_return(const gnc::DeorbitParameters& parameters, bool background) {
    const auto wall = std::chrono::steady_clock::now();  // diagnostics
    ship_return_ = false;
    deorbit_plan_.reset();
    reset_entry_vent();  // final review M6: nothing of an earlier return's vent carries into this one
    ++deorbit_generation_;  // a background plan or re-plan of an earlier return is dropped when it arrives
    ship_return_task_.discard();
    deorbit_parameters_ = parameters;
    if (bodies_.empty()) return false;
    gnc::DeorbitPlan refused;
    std::optional<DeorbitReplanInputs> inputs;
    const auto& flying = bodies_[controlled_];
    if (flying.entry_guidance || flying.ship_landing) {
        refused.reason = report::make(report::Code::return_no_ship_in_orbit);
    } else {
        inputs = deorbit_plan_inputs(*flying.body, parameters, refused);
    }
    if (!inputs) {
        deorbit_plan_ = refused;
        return false;
    }
    inputs->measure_drag = true;  // no drag given: the ship's own, flown ahead
    if (background) {
        // Real-time play (Task 11b): the drag flight, the planner and the boil-off flight on a worker; the ship coasts on.
        // Waited for once ship_return_budget_s of sim time has passed (time warp meanwhile).
        ship_return_parameters_ = parameters;
        ship_return_deadline_ = epoch_ + ship_return_budget_s_;
        ship_return_wall_start_ = wall;
        auto worker = worker_environment();
        ship_return_task_.submit([in = std::move(*inputs), worker] {
            return compute_deorbit_plan(in, worker->gravity(), worker->earth(), worker->env());
        });
        ship_return_begin_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall).count();
        return std::nullopt;
    }
    const bool applied = apply_ship_return(compute_deorbit_plan(std::move(*inputs), gravity_, earth_, env_));
    if (std::getenv("STARSIM_DEBUG_REALTIME")) {  // diagnostics: the sim thread's hold
        std::printf("[realtime] ship return at once: %.0f ms on the sim thread\n",
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall).count());
    }
    return applied;
}

std::optional<bool> FlightSession::poll_ship_return(bool wait) {
    const auto decide = [this](bool outcome) {
        ship_return_outcome_ = outcome;
        if (ship_return_then_) std::exchange(ship_return_then_, {})(outcome);
    };
    while (ship_return_task_.busy()) {
        const bool due = wait || !realtime_planning_ || epoch_ - ship_return_deadline_ >= 0.0;
        const auto poll_start = std::chrono::steady_clock::now();
        auto result = ship_return_task_.collect(due);
        if (!result) break;
        const auto& plan = result->plan;
        const auto& groups = bodies_.empty() ? std::vector<std::string>{}
                                             : bodies_[controlled_].body->vehicle().stages().front().config.deorbit_engine_groups;
        // only a snapshot older than now can be stale: a plan made from the state now is decided as the sync path would
        if (result->inputs.generation == deorbit_generation_ && plan.feasible && !bodies_.empty() &&
            result->inputs.epoch < epoch_ &&
            !burn_still_ahead(plan.node, groups)) {
            // Fix round 1 (I1): its burn came too close while it was planned (time warp meanwhile) - never armed;
            // planned again from the state now.
            ++ship_return_resubmits_;
            std::printf("[deorbit] background return plan stale (burn %.0f s ahead): planned again from the state now\n",
                        plan.node.epoch - epoch_);
            if (const auto refused = submit_ship_return(ship_return_parameters_, true)) decide(*refused);
            continue;
        }
        const auto apply_start = std::chrono::steady_clock::now();
        const bool applied = apply_ship_return(*result);
        if (std::getenv("STARSIM_DEBUG_REALTIME")) {  // diagnostics: the sim thread's share
            const auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
            const auto now = std::chrono::steady_clock::now();
            std::printf("[realtime] ship return: begin %.1f ms + wait %.0f ms + apply %.1f ms on the sim thread, worker "
                        "%.0f ms, applied %.1f s after its snapshot\n",
                        ship_return_begin_ms_, ms(poll_start, apply_start), ms(apply_start, now),
                        ms(ship_return_wall_start_, apply_start), epoch_ - result->inputs.epoch);
        }
        decide(applied);
    }
    return ship_return_outcome_;
}

std::optional<bool> FlightSession::ship_return_request_outcome(std::uint64_t id, report::Report* cancelled) {
    if (id != ship_return_request_) {
        if (cancelled) *cancelled = report::make(report::Code::return_planning_cancelled);
        return false;
    }
    return poll_ship_return(false);
}

bool FlightSession::burn_still_ahead(const gnc::BurnNode& node, const std::vector<std::string>& groups) const {
    // The configuration start_burn picks (no RCS fallback for a forced set); its ignition half the burn before the node.
    const double dv = norm(node.delta_v_gcrf_mps);
    const double mass_kg = bodies_[controlled_].body->vehicle().mass_properties().mass_kg;
    for (const auto& c : burn_configurations(&groups)) {
        if (c.minimum_dv_mps > 0.25 * dv) continue;
        const double to_ignition_s = node.epoch - epoch_ - 0.5 * gnc::burn_duration_s(dv, mass_kg, c.thrust_n, c.mass_flow_kgps);
        return to_ignition_s > burn_lead_s();
    }
    return true;  // no configuration: execute_burn refuses it itself
}

bool FlightSession::apply_ship_return(const DeorbitReplanResult& result) {
    const auto& in = result.inputs;
    if (in.generation != deorbit_generation_ || bodies_.empty()) return false;  // planned anew since: dropped
    const auto& stage = bodies_[controlled_].body->vehicle().stages().front().config;
    // the drag the plan coasted with: the tracking re-plans' first guess
    deorbit_parameters_.drag_area_per_mass_m2pkg = in.parameters.drag_area_per_mass_m2pkg;
    deorbit_plan_ = result.plan;
    if (!deorbit_plan_->feasible) return false;
    // The earliest pass is the best one with boil-off too (a later one only has less left): it alone is checked.
    if (const auto shortfall = boil_off_shortfall(*deorbit_plan_, commit_burn_margin, result.now, result.at_burn);
        !shortfall.ok()) {
        deorbit_plan_->feasible = false;
        deorbit_plan_->short_of_reserve = true;
        deorbit_plan_->reason = shortfall;
        return false;
    }
    if (!execute_burn(deorbit_plan_->node, stage.deorbit_engine_groups)) {
        deorbit_plan_->feasible = false;
        deorbit_plan_->reason = report::make(report::Code::deorbit_burn_too_small);
        return false;
    }
    ship_return_ = true;
    log_dispersion_note(*deorbit_plan_);
    set_entry_vent_plan(*deorbit_plan_, in.max_main_propellant_kg, result.now, result.at_burn);
    deorbit_replan_due_ = deorbit_plan_->node.epoch - in.epoch > deorbit_replan_age_s;
    deorbit_track_epoch_ = in.epoch;
    deorbit_track_state_ = in.state;
    deorbit_next_track_ = in.epoch + deorbit_track_interval_s;
    return true;
}

double FlightSession::deorbit_replan_in_s() const {
    if (!deorbit_replan_due_ || !ship_return_) return std::numeric_limits<double>::infinity();
    const double lead = autopilot_.burn.time_to_ignition_s - deorbit_replan_lead_s;
    return std::min(lead, deorbit_next_track_ - epoch_);
}

void FlightSession::replan_ship_deorbit() {
    if (realtime_planning_ && deorbit_replan_task_.busy()) {
        // a job of an earlier return still runs (its own is not due before it was applied): orphaned, reaped without
        // a wait when done; this re-plan stays due and starts on a tick after it (fix round 3: the sim thread never
        // joins a running job here)
        deorbit_replan_orphan_ = true;
        return;
    }
    deorbit_replan_due_ = false;
    auto inputs = prepare_deorbit_replan();
    if (!inputs) return;
    if (realtime_planning_) {
        // Real-time play (Task 11b): the planner and the boil-off flight run on a worker with an environment of their
        // own; the old plan stays armed until the result is applied (its burn is >= deorbit_replan_lead_s away).
        deorbit_replan_deadline_ = epoch_ + ship_return_budget_s_;  // waited for from here (time warp meanwhile)
        deorbit_replan_wall_start_ = std::chrono::steady_clock::now();
        auto worker = worker_environment();
        deorbit_replan_task_.submit([in = std::move(*inputs), worker] {
            return compute_deorbit_plan(in, worker->gravity(), worker->earth(), worker->env());
        });
        return;
    }
    apply_deorbit_replan(compute_deorbit_plan(std::move(*inputs), gravity_, earth_, env_));
}

void FlightSession::poll_deorbit_replan(bool wait) {
    if (deorbit_replan_orphan_) return;  // its owner ended: reaped, never applied
    const bool due = wait || !realtime_planning_ || epoch_ - deorbit_replan_deadline_ >= 0.0;
    const auto poll_start = std::chrono::steady_clock::now();
    if (auto result = deorbit_replan_task_.collect(due)) {
        if (std::getenv("STARSIM_DEBUG_REALTIME")) {  // diagnostics: the worker's time and the sim thread's wait
            const auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
            const auto now = std::chrono::steady_clock::now();
            std::printf("[realtime] deorbit re-plan: worker %.0f ms, waited %.0f ms on the sim thread (%s)\n",
                        ms(deorbit_replan_wall_start_, now), ms(poll_start, now), due ? "due" : "ready");
        }
        apply_deorbit_replan(*result);
    }
}

std::optional<FlightSession::DeorbitReplanInputs> FlightSession::prepare_deorbit_replan() {
    const auto& flying = bodies_[controlled_];
    const auto& stage = flying.body->vehicle().stages().front().config;
    if (stage.deorbit_engine_groups.empty() || burn_configurations(&stage.deorbit_engine_groups).empty() || !deorbit_plan_) {
        return std::nullopt;
    }
    const auto& s = flying.body->state();
    gnc::DeorbitParameters parameters = deorbit_parameters_;
    parameters.search_window_s = deorbit_plan_->node.epoch - epoch_ + deorbit_replan_window_s;
    parameters.drag_area_per_mass_m2pkg =
        gnc::estimate_drag_area_per_mass(gravity_, deorbit_track_state_, deorbit_track_epoch_,
                                         {s.position_gcrf_m, s.velocity_gcrf_mps}, epoch_,
                                         deorbit_parameters_.drag_area_per_mass_m2pkg);
    deorbit_parameters_.drag_area_per_mass_m2pkg = parameters.drag_area_per_mass_m2pkg;  // the next re-plan's guess
    gnc::DeorbitPlan refused;
    auto in = deorbit_plan_inputs(*flying.body, parameters, refused);
    if (!in) return std::nullopt;
    in->old_node = deorbit_plan_->node;
    in->old_plan = *deorbit_plan_;
    return in;
}

FlightSession::DeorbitReplanResult FlightSession::compute_deorbit_plan(DeorbitReplanInputs inputs,
                                                                       const environment::GravityModel& gravity,
                                                                       const frames::BodyOrientation& earth,
                                                                       const vehicle::FlightEnvironment& env) {
    DeorbitReplanResult r;
    r.plan = plan_from_inputs(inputs, gravity, earth, env);
    const auto& in = inputs;
    // The plan flown and its main tanks at its burn: a first plan (measure_drag) only when feasible (apply_ship_return);
    // a re-plan's new one, else the old one (apply_deorbit_replan).
    const bool first = in.measure_drag;
    if (!first || r.plan.feasible) {
        const auto& flown_node = first || r.plan.feasible || r.plan.short_of_reserve ? r.plan.node : in.old_plan.node;
        r.now = main_tanks(in.copy->vehicle());
        r.at_burn = main_propellant_flown(*in.copy, env, in.epoch, flown_node.epoch);
    }
    r.inputs = std::move(inputs);
    return r;
}

void FlightSession::apply_deorbit_replan(const DeorbitReplanResult& result) {
    const auto& in = result.inputs;
    // a re-plan of a return that has ended or been planned anew since its snapshot: dropped
    if (in.generation != deorbit_generation_ || !ship_return_ || !deorbit_plan_ || autopilot_.state != AutopilotState::Burn ||
        bodies_.empty()) {
        return;
    }
    const auto& stage = bodies_[controlled_].body->vehicle().stages().front().config;
    if (epoch_ != in.epoch) {
        // Fix round 1 (I1), a re-plan from the worker: the armed burn's preparation or the burn itself has begun (time
        // warp while it was planned) - the old plan flies on, never re-armed mid-burn; a new pass that came too close is
        // not taken (the old plan flies on; no further far re-plan is armed - deorbit_replan_due_ was cleared when this one began).
        const bool started = autopilot_.burn.ignite || autopilot_.burn.time_to_ignition_s <= burn_lead_s();
        if (started || (result.plan.feasible && !burn_still_ahead(result.plan.node, stage.deorbit_engine_groups))) {
            ++deorbit_replans_dropped_;
            std::printf("[deorbit] background re-plan dropped: %s\n",
                        started ? "the burn is being prepared or flown" : "its burn came too close while it was planned");
            return;
        }
    }
    ++deorbit_replans_;
    const auto& plan = result.plan;
    const auto& old_node = in.old_node;
    if (std::getenv("STARSIM_DEBUG_DEORBIT")) {
        std::printf("[deorbit] re-plan %.0f s before the old node (tracked %.1f h, CdA/m %.3e m2/kg): %s, burn %+.1f s "
                    "from it, %.2f m/s (old %.2f m/s), cross %.1f km%s\n",
                    old_node.epoch - in.epoch, (in.epoch - deorbit_track_epoch_) / 3600.0,
                    in.parameters.drag_area_per_mass_m2pkg, plan.feasible ? "feasible" : report::describe(plan.reason).c_str(),
                    plan.feasible ? plan.node.epoch - old_node.epoch : 0.0, norm(plan.node.delta_v_gcrf_mps),
                    norm(old_node.delta_v_gcrf_mps), plan.crossrange_m / 1000.0,
                    epoch_ == in.epoch ? "" : " (worker)");
    }
    deorbit_next_track_ = in.epoch + deorbit_track_interval_s;
    // A later pass is taken when drag moved this one out of reach (it is planned again before its burn); none in the
    // window: the old plan is flown (and planned again at the next tracking time or before its burn).
    // The landing reserve was checked when the deorbit was committed; a re-plan that only cuts into it is still flown -
    // the old plan's burn is known to miss now (l5 Task 10, aircatch seed 15: the tracked drag asked 64.4 instead of
    // 61.0 m/s, 0.3 t into the 35 t reserve; the old burn was flown and the entry missed the gate by 274 km).
    // Boil-off since the commitment (r2): when the main tanks will no longer hold the burn, it is called off - the
    // ship stays in orbit instead of running dry in the burn.
    gnc::DeorbitPlan flown = plan.feasible || plan.short_of_reserve ? plan : *deorbit_plan_;
    {
        if (const auto shortfall = boil_off_shortfall(flown, call_off_burn_margin, result.now, result.at_burn);
            !shortfall.ok()) {
            deorbit_plan_ = flown;
            deorbit_plan_->feasible = false;
            deorbit_plan_->short_of_reserve = true;
            deorbit_plan_->reason = shortfall;
            ship_return_ = false;
            disengage_autopilot();
            burn_abort_reason_ = shortfall;
            return;
        }
    }
    auto node = old_node;
    if (plan.feasible || plan.short_of_reserve) {
        deorbit_plan_ = flown;  // the plan with the boil-off until its burn (r2 minor: the planner's has none)
        if (execute_burn(plan.node, stage.deorbit_engine_groups)) {
            node = plan.node;
            log_dispersion_note(*deorbit_plan_);
            set_entry_vent_plan(*deorbit_plan_, in.max_main_propellant_kg, result.now, result.at_burn);
        } else {
            deorbit_plan_->node = old_node;
            execute_burn(old_node, stage.deorbit_engine_groups);
        }
    } else {
        deorbit_plan_->boil_off_kg = flown.boil_off_kg;  // the old plan, flown on: its boil-off from now
    }
    deorbit_replan_due_ = node.epoch - in.epoch > deorbit_replan_age_s;
}

time::Epoch FlightSession::entry_start_epoch(double horizon_s) const {
    const auto& s = bodies_[controlled_].body->state();
    const integrators::Event<6> interface{
        "entry interface",
        [&](double t_s, const integrators::State<6>& y) {
            const math::Vec3 r = transpose(earth_.fixed_to_gcrf(epoch_ + t_s).rotation) * math::Vec3{y[0], y[1], y[2]};
            return frames::fixed_to_geodetic(r).height_m - deorbit_parameters_.entry_interface_height_m;
        },
        integrators::Crossing::Falling};
    const auto coast = orbit::propagate_coast(gravity_, {s.position_gcrf_m, s.velocity_gcrf_mps}, epoch_, epoch_ + horizon_s,
                                              std::span(&interface, 1));
    return coast.event_index == 0 ? coast.epoch - entry_lead_s : epoch_;  // no entry ahead: guidance at once
}

void FlightSession::place_landing_ship(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                                       time::Epoch epoch, std::optional<gnc::ShipLandingParameters> parameters) {
    place_body(config, state, epoch);
    auto& flying = bodies_.front();
    const auto& vehicle = flying.body->vehicle();
    flying.ship_landing = std::make_unique<gnc::ShipLandingGuidance>(
        return_site(vehicle), parameters ? std::move(*parameters) : gnc::ship_landing_parameters(vehicle));
    flying.ship_landing->set_realtime(realtime_planning_);
}

// Flip and landing burn of a ship: engine groups and throttle for this tick (attitude in command_for).
void FlightSession::run_ship_landing(Flying& flying) {
    if (flying.body->impacted()) {
        if (!flying.ship_landing->has_contact()) flying.ship_landing->update(*flying.body, env_, epoch_);  // contact record
        return;
    }
    // From the belly flip on the header tanks feed the engines: the main-tank residue lies away from the outlets
    // (header-tank plan; a no-op without headers).
    flying.body->vehicle().select_feed(0, vehicle::FeedSource::Header);
    if (tower_) flying.ship_landing->set_catch_contact(tower_->in_contact(*flying.body));
    flying.return_command = flying.ship_landing->update(*flying.body, env_, epoch_);
    for (const auto& stage : flying.body->vehicle().stages()) {
        for (const auto& e : stage.engines) {
            const auto& groups = flying.return_command.run_groups;
            flying.running_groups[e.group_id] = std::find(groups.begin(), groups.end(), e.group_id) != groups.end();
        }
    }
    if (tower_ && flying.return_command.abort) tower_->abort_catch();
}

// Autonomous return of a separated booster: engine groups, throttle and steering axis for this tick.
void FlightSession::run_return_guidance(Flying& flying) {
    if (flying.body->impacted()) return;
    flying.return_command = flying.return_guidance->update(*flying.body, env_, epoch_);
    for (const auto& stage : flying.body->vehicle().stages()) {
        for (const auto& e : stage.engines) {
            const auto& groups = flying.return_command.run_groups;
            flying.running_groups[e.group_id] = std::find(groups.begin(), groups.end(), e.group_id) != groups.end();
        }
    }
    if (tower_ && flying.return_command.abort) tower_->abort_catch();
}

bool FlightSession::returns_after_staging(const vehicle::Vehicle& vehicle) const {
    // Return guidance for a booster that is caught by a tower at this site (Super Heavy has no legs, project decision
    // 2026-10-13: it is always caught) or lands on its legs on the mission's landing zone (Falcon 9).
    if (!booster_return_) return false;
    const auto& stage = vehicle.stages().front().config;
    if (stage.recovery_method == "tower_catch") return launch_site_.tower.has_value();
    return stage.recovery_method == "leg_landing" && landing_zone_ && !stage.landing_legs.azimuths_rad.empty();
}

gnc::LandingSite FlightSession::return_site(const vehicle::Vehicle& booster) const {
    const auto& stage = booster.stages().front();
    if (landing_zone_ && stage.config.recovery_method == "leg_landing" && !stage.config.landing_legs.azimuths_rad.empty()) {
        const auto& z = *landing_zone_;
        gnc::LandingSite landing{ephemeris::Body::Earth, z.latitude_rad, z.longitude_rad, z.height_m, {}, {}, {}};
        if (z.sea) landing.sea = gnc::SeaSide{z.sea->direction_rad, z.sea->shoreline_distance_m};
        // The feet sit -foot_station_m below the stage bottom when deployed.
        landing.pad = gnc::PadTarget{-stage.config.landing_legs.foot_station_m, z.pad_radius_m};
        return landing;
    }
    gnc::LandingSite landing{ephemeris::Body::Earth, launch_site_.latitude_rad, launch_site_.longitude_rad, launch_site_.height_m, {}, {}, {}};
    if (tower_ && !stage.config.catch_pins.azimuths_rad.empty()) {
        const auto& g = tower_->geometry();
        const auto& c = g.config();
        landing.catch_target = gnc::CatchTarget{g.to_itrf({0.0, 0.0, g.arm_top_height_m(0.0)}),
                                                g.site_to_itrf() * g.across(),
                                                stage.base_station_m + stage.config.catch_pins.station_m,
                                                stage.config.catch_pins.azimuths_rad.front(),
                                                g.to_itrf(g.tower_direction() * c.distance_m),
                                                0.5 * c.width_m * std::sqrt(2.0) + 0.5 * booster.diameter_m() +
                                                    tower_keep_out_margin_m,
                                                c.mount_outer_radius_m + 0.5 * booster.diameter_m() + tower_keep_out_margin_m,
                                                c.mount_height_m,
                                                c.height_m,
                                                stage.config.catch_pins.away_from_tower_azimuth_rad};
        const auto structure_box = [&](const contact::OrientedBox& box) {
            return gnc::StructureBox{g.to_itrf(box.center), g.site_to_itrf() * math::column(box.axes, 0), box.half_extents.x,
                                     box.half_extents.y, box.half_extents.z};
        };
        for (int i = 0; i < 2; ++i) {
            const int side = i == 0 ? 1 : -1;
            landing.catch_target->arms_standby[static_cast<size_t>(i)] = structure_box(g.arm_box(side, c.arm_standby_rad, 0.0));
            landing.catch_target->arms_open[static_cast<size_t>(i)] = structure_box(g.arm_box(side, c.arm_open_rad, 0.0));
        }
        landing.catch_target->tower = structure_box(g.tower_box());
    }
    if (launch_site_.sea) {
        landing.sea = gnc::SeaSide{launch_site_.sea->direction_rad, launch_site_.sea->shoreline_distance_m};
    }
    return landing;
}

void FlightSession::place_returning_booster(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                                            time::Epoch epoch, gnc::BoosterReturnParameters parameters,
                                            gnc::ReturnPhase start_phase) {
    place_body(config, state, epoch);
    auto& flying = bodies_.front();
    flying.return_guidance =
        std::make_unique<gnc::BoosterReturnGuidance>(return_site(flying.body->vehicle()), std::move(parameters));
    flying.body->set_accelerometer(true);  // the return guidance estimates the drag online
    flying.return_guidance->skip_to(start_phase);
    flying.return_guidance->set_realtime(realtime_planning_);
}

gnc::BoosterReturnParameters FlightSession::return_parameters(const vehicle::Vehicle& vehicle) const {
    const bool drone_ship = landing_zone_ && landing_zone_->deck;
    return gnc::booster_return_parameters(vehicle.source_config(), vehicle.stages().front().config, drone_ship,
                                          booster_return_.value_or(gnc::BoosterReturnParameters{}));
}

void FlightSession::set_landing_zone(const LaunchSite& zone) {
    landing_zone_ = zone;
    legs_ = earth_leg_site(zone, true);
}

void FlightSession::apply_return_command(const Flying& flying, const gnc::AttitudeController& controller,
                                         double ambient_pressure_pa, vehicle::VehicleCommand& command) const {
    const auto& r = flying.return_command;
    for (auto& group : command.groups) group.throttle = r.throttle;
    if (r.torque_body_nm) {
        // SCvx lateral LQR: the guidance's torque straight to gimbals / RCS. The grid fins feather into their local flow:
        // the planning model has none (neutral fins act as a tail and turn an engines-first stage into the air flow;
        // allocating torque to them fights the gimbals at low speed).
        auto output = gnc::AttitudeController::allocate(*flying.body, ambient_pressure_pa, *r.torque_body_nm, false);
        output.grid_fin_rad = gnc::AttitudeController::feathered_grid_fins(*flying.body);
        output.apply_to(command);
        if (std::getenv("STARSIM_DEBUG_BURN") && flying.ship_landing) {  // diagnostics
            static int n = 0;
            if (++n % 40 == 0) {
                double gp = 0.0, gy = 0.0;
                for (const auto& e : flying.body->vehicle().stages().front().engines) {
                    if (e.producing_thrust()) { gp = e.gimbal_pitch_rad; gy = e.gimbal_yaw_rad; }
                }
                const auto& t = *r.torque_body_nm;
                const auto& w = flying.body->state().angular_velocity_body_radps;
                const auto inertia = flying.body->vehicle().mass_properties().inertia_kg_m2;
                int running = 0;
                for (const auto& e : flying.body->vehicle().stages().front().engines) running += e.producing_thrust() ? 1 : 0;
                std::printf("[burn] torque %.2f/%.2f/%.2f MNm gimbal cmd %.2f/%.2f deg actual %.2f/%.2f deg w %.3f/%.3f/%.3f "
                            "I*w %.1f/%.1f engines %d flaps",
                            t.x / 1e6, t.y / 1e6, t.z / 1e6, command.gimbal_pitch_rad * 57.3, command.gimbal_yaw_rad * 57.3,
                            gp * 57.3, gy * 57.3, w.x, w.y, w.z, inertia.y * w.y / 1e6, inertia.z * w.z / 1e6, running);
                for (const auto& f : flying.body->vehicle().stages().front().flaps) std::printf(" %.0f", f.deflection_rad * 57.3);
                const auto mass = flying.body->vehicle().mass_properties();
                const auto& down = flying.body->vehicle().liquid_down_body();
                std::printf(" cg %.2f %.2f/%.2f m liquid %.2f/%.2f/%.2f", mass.cg_station_m, mass.cg_offset_m.y,
                            mass.cg_offset_m.z, down.x, down.y, down.z);
                std::printf("\n");
            }
        }
    } else {
        controller.steer(*flying.body, ambient_pressure_pa, r.axis_gcrf, r.roll_reference_gcrf,
                         r.feedforward ? &*r.feedforward : nullptr).apply_to(command);
    }
    command.rcs_force_fraction = flying.body->state().attitude.inverse_rotate(r.rcs_force_gcrf);
    if (!r.flap_rad.empty()) command.flap_rad = r.flap_rad;
}

std::shared_ptr<const gnc::AlphaSchedule> FlightSession::alpha_schedule(const std::string& table) const {
    auto& cached = alpha_schedules_[table];
    if (!cached) cached = std::make_shared<const gnc::AlphaSchedule>(gnc::load_alpha_schedule(config_dir_, table));
    return cached;
}

void FlightSession::reset_entry_vent() {
    entry_vent_remaining_kg_ = 0.0;
    entry_vented_kg_ = 0.0;
    entry_residue_limit_kg_ = std::numeric_limits<double>::infinity();
    entry_vent_floor_kg_ = std::numeric_limits<double>::infinity();
}

void FlightSession::set_entry_vent_plan(const gnc::DeorbitPlan& plan, double max_main_propellant_kg,
                                        const MainPropellant& now, const MainPropellant& at_burn) {
    entry_residue_limit_kg_ = max_main_propellant_kg;
    entry_vent_floor_kg_ = std::numeric_limits<double>::infinity();
    if (bodies_.empty() || !(plan.entry_vent_kg > 0.0)) return;
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    const double rate = vehicle.stages().front().config.entry_vent_rate_kgps;
    if (!(rate > 0.0) || !(max_main_propellant_kg < std::numeric_limits<double>::infinity())) return;
    // the coast from the burn's end to the entry guidance (vent_time_missing_s's, with the ship's mass now)
    double half_burn_s = 0.0;
    const auto configurations = burn_configurations(&vehicle.stages().front().config.deorbit_engine_groups);
    if (!configurations.empty()) {
        const auto& c = configurations.front();
        half_burn_s = 0.5 * gnc::burn_duration_s(plan.delta_v_mps, vehicle.mass_properties().mass_kg, c.thrust_n, c.mass_flow_kgps);
    }
    const double after_s = std::max(0.0, plan.entry_interface_epoch - entry_lead_s - (plan.node.epoch + half_burn_s));
    // Kept for the burn while venting before it: its propellant with the commit margin, the settling gas and the
    // boil-off until it (as boil_off_shortfall checks it); the coast after the burn vents at least that much again.
    const double keep_kg = (commit_burn_margin - 1.0) * plan.propellant_kg + settling_gas_kg() +
                           std::max(0.0, now.total() - at_burn.total());
    const double floor_kg = max_main_propellant_kg + plan.propellant_kg + std::max(rate * after_s, keep_kg);
    const double main_kg = vehicle.main_propellant_kg();
    if (!(main_kg > floor_kg)) return;  // the coast after the burn holds the whole vent: it starts at the cutoff
    entry_vent_floor_kg_ = floor_kg;
    std::printf("[entry vent] the coast after the burn (%.0f s) vents %.2f t of %.2f t: %.2f t vented before the burn "
                "(main tanks %.2f t -> %.2f t)\n",
                after_s, rate * after_s / 1000.0, plan.entry_vent_kg / 1000.0, (main_kg - floor_kg) / 1000.0,
                main_kg / 1000.0, floor_kg / 1000.0);
}

void FlightSession::arm_entry_vent() {
    entry_vent_remaining_kg_ = 0.0;
    entry_vent_floor_kg_ = std::numeric_limits<double>::infinity();  // the burn is done: the wait's share ends
    if (bodies_.empty()) return;
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    const auto& stage = vehicle.stages().front().config;
    if (!(stage.entry_vent_rate_kgps > 0.0) || !(entry_residue_limit_kg_ < std::numeric_limits<double>::infinity())) return;
    // the limit the armed plan derived on the planner's thread (final fix I1: no bisection, no file read here)
    const double main_kg = vehicle.main_propellant_kg();
    entry_vent_remaining_kg_ = std::max(0.0, main_kg - entry_residue_limit_kg_);
    if (entry_vent_remaining_kg_ > 0.0) {
        std::printf("[entry vent] main tanks %.2f t, the flaps trim the entry with %.2f t: venting %.2f t at %.0f kg/s, "
                    "%.0f s before the entry guidance\n",
                    main_kg / 1000.0, entry_residue_limit_kg_ / 1000.0, entry_vent_remaining_kg_ / 1000.0,
                    stage.entry_vent_rate_kgps, entry_start_ - epoch_);
    }
}

void FlightSession::vent_entry_residue(double dt_s) {
    if (bodies_.empty()) return;
    if (entry_vent_floor_kg_ < std::numeric_limits<double>::infinity()) {
        // the wait before the burn (final fix I2): down to the floor, never once its preparation or the burn began
        auto& vehicle = bodies_[controlled_].body->vehicle();
        if (!ship_return_ || autopilot_.state != AutopilotState::Burn) {
            entry_vent_floor_kg_ = std::numeric_limits<double>::infinity();
            return;
        }
        if (autopilot_.burn.ignite || autopilot_.burn.time_to_ignition_s <= burn_lead_s()) return;
        const double share_kg = vehicle.main_propellant_kg() - entry_vent_floor_kg_;
        if (!(share_kg > 0.0)) {
            entry_vent_remaining_kg_ = 0.0;
            return;
        }
        const double kg = std::min(share_kg, vehicle.stages().front().config.entry_vent_rate_kgps * dt_s);
        gnc::draw_main_tanks(vehicle, kg);
        entry_vented_kg_ += kg;
        entry_vent_remaining_kg_ = share_kg - kg;
        return;
    }
    if (!(entry_vent_remaining_kg_ > 0.0)) return;
    auto& flying = bodies_[controlled_];
    if (flying.entry_guidance || flying.ship_landing || flying.body->impacted() || autopilot_.state != AutopilotState::Return) {
        // the interface reached (or the return ended) before the vent: what is left stays on board
        std::printf("[entry vent] stopped with %.2f t still to vent (%.2f t vented)\n", entry_vent_remaining_kg_ / 1000.0,
                    entry_vented_kg_ / 1000.0);
        entry_vent_remaining_kg_ = 0.0;
        return;
    }
    auto& vehicle = flying.body->vehicle();
    const double kg = std::min({entry_vent_remaining_kg_, vehicle.stages().front().config.entry_vent_rate_kgps * dt_s,
                                vehicle.main_propellant_kg()});
    gnc::draw_main_tanks(vehicle, kg);
    entry_vent_remaining_kg_ = kg < entry_vent_remaining_kg_ && vehicle.main_propellant_kg() > 0.0
                                   ? entry_vent_remaining_kg_ - kg : 0.0;
    entry_vented_kg_ += kg;
}

}  // namespace starsim::sim
