// FlightSession: burn configurations of the controlled vehicle and the execution of planned burns.

#include "starsim/sim/flight_session.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/vehicle/propellant_transfer.hpp"

namespace starsim::sim {

// Candidate engine configurations of the lowest stage with engines, strongest first: all groups at full
// throttle, each group alone at full throttle, each group alone at minimum throttle.
std::vector<FlightSession::BurnConfiguration> FlightSession::burn_configurations(const std::vector<std::string>* forced) const {
    std::vector<BurnConfiguration> configurations;
    if (bodies_.empty()) return configurations;
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    const double mass = vehicle.mass_properties().mass_kg;
    for (const auto& stage : vehicle.stages()) {
        std::vector<std::string> groups;
        for (const auto& e : stage.engines) {
            if (e.auxiliary()) continue;  // launch escape or landing thrusters: never a burn engine
            if (std::find(groups.begin(), groups.end(), e.group_id) == groups.end()) groups.push_back(e.group_id);
        }
        if (groups.empty()) continue;
        // Burn sets: groups of the same engine on the same ring burn together (symmetric thrust; e.g. the ship's three
        // single-engine sea-level groups of the landing sequence - one off-axis Raptor would push the burn askew).
        std::vector<std::vector<std::string>> sets;
        std::vector<std::pair<std::string, double>> set_keys;
        for (const auto& g : groups) {
            const auto first = std::find_if(stage.engines.begin(), stage.engines.end(),
                                            [&](const vehicle::Engine& e) { return e.group_id == g; });
            const std::pair<std::string, double> key{first->performance->config().id,
                                                     std::round(std::hypot(first->gimbal_point_m.y, first->gimbal_point_m.z) * 100.0)};
            const auto it = std::find(set_keys.begin(), set_keys.end(), key);
            if (it == set_keys.end()) {
                set_keys.push_back(key);
                sets.push_back({g});
            } else {
                sets[static_cast<size_t>(it - set_keys.begin())].push_back(g);
            }
        }
        auto make = [&](const std::vector<std::string>& selected, bool minimum_throttle) {
            BurnConfiguration c;
            c.groups = selected;
            double transient = 0.0;
            for (const auto& e : stage.engines) {
                if (e.starved || e.auxiliary() || std::find(selected.begin(), selected.end(), e.group_id) == selected.end()) {
                    continue;
                }
                const auto& cfg = e.performance->config();
                c.throttle = minimum_throttle ? cfg.throttle_min : 1.0;
                c.thrust_n += e.performance->thrust_n(c.throttle, 0.0);
                c.mass_flow_kgps += e.performance->mass_flow_kgps(c.throttle);
                c.shutdown_s = std::max(c.shutdown_s, cfg.shutdown_s);
                transient = std::max(transient, 0.5 * (cfg.startup_s + cfg.shutdown_s));
            }
            c.minimum_dv_mps = c.thrust_n / mass * transient;
            c.thrust_axis_body = gnc::thrust_axis_body(vehicle, vehicle.mass_properties().cg_station_m, selected);
            return c;
        };
        if (forced) {
            std::vector<std::string> selected;
            for (const auto& g : *forced) {
                const std::string id = stage.config.id + "/" + g;
                // Only burn engines: a landing-only or launch-escape group is never lit by a burn (lunar l2).
                if (std::find(groups.begin(), groups.end(), id) != groups.end()) selected.push_back(id);
            }
            if (!selected.empty()) {
                configurations.push_back(make(selected, false));
                configurations.push_back(make(selected, true));
            }
            break;
        }
        configurations.push_back(make(groups, false));
        for (const auto& set : sets) configurations.push_back(make(set, false));
        for (const auto& set : sets) configurations.push_back(make(set, true));
        break;
    }
    std::erase_if(configurations, [](const BurnConfiguration& c) { return c.thrust_n <= 0.0; });
    std::stable_sort(configurations.begin(), configurations.end(),
                     [](const BurnConfiguration& a, const BurnConfiguration& b) { return a.thrust_n > b.thrust_n; });
    return configurations;
}

double FlightSession::minimum_burn_dv_mps() const {
    const auto configurations = burn_configurations();
    return configurations.empty() ? 0.0 : configurations.back().minimum_dv_mps;
}

bool FlightSession::execute_burn(const gnc::BurnNode& node) { return start_burn(node, burn_configurations()); }

bool FlightSession::execute_burn(const gnc::BurnNode& node, const std::vector<std::string>& groups) {
    return start_burn(node, burn_configurations(&groups), false);
}

bool FlightSession::start_burn(const gnc::BurnNode& node, const std::vector<BurnConfiguration>& configurations,
                               bool rcs_fallback, double max_transient_share) {
    // Strongest configuration whose start/shutdown impulse is at most a quarter (max_transient_share) of the burn, so
    // that the transients stay small against the closed-loop part; otherwise the weakest that still fits.
    const double dv = norm(node.delta_v_gcrf_mps);
    const BurnConfiguration* chosen = nullptr;
    for (const auto& c : configurations) {
        if (c.minimum_dv_mps <= max_transient_share * dv) {
            chosen = &c;
            break;
        }
    }
    BurnConfiguration rcs;
    if (chosen == nullptr && rcs_fallback && !bodies_.empty()) {
        // Below the main-engine minimum impulse (or no main engines): RCS translation (no transients) along +x when the
        // vehicle has thrusters along it (ullage thrusters), else along -x (Crew Dragon with only the Dracos under its
        // nose cone) - if the burn takes less than an hour. Lateral axes are not used: one-station layouts (Super Heavy)
        // push only with a torque, and lateral translation belongs to docking (M9).
        const auto& vehicle = bodies_[controlled_].body->vehicle();
        rcs.rcs = true;
        auto capability = vehicle.rcs_force_capability({1.0, 0.0, 0.0});
        rcs.thrust_axis_body = {1.0, 0.0, 0.0};
        if (!(capability.force_n > 0.0)) {
            capability = vehicle.rcs_force_capability({-1.0, 0.0, 0.0});
            rcs.thrust_axis_body = {-1.0, 0.0, 0.0};
        }
        rcs.thrust_n = capability.force_n;
        rcs.mass_flow_kgps = capability.mass_flow_kgps;
        if (rcs.thrust_n > 0.0 && dv / (rcs.thrust_n / vehicle.mass_properties().mass_kg) < 3600.0) {
            chosen = &rcs;
        }
    }
    if (chosen == nullptr) {
        return false;
    }
    burn_config_ = *chosen;
    burn_.emplace(node);
    settling_lead_s_ = chosen->rcs ? 0.0 : settling_lead_s();  // cold gas needs no settled liquid
    settling_push_ = false;
    settling_hold_start_.reset();
    burn_abort_reason_.clear();
    // the status until the burn's first tick: a coast before it read the previous burn's (l5 Task 10: the deorbit
    // re-plan saw the old burn's time to ignition and planned again on every coast step)
    autopilot_.burn = {};
    autopilot_.burn.time_to_ignition_s =
        node.epoch - epoch_ -
        0.5 * gnc::burn_duration_s(dv, bodies_[controlled_].body->vehicle().mass_properties().mass_kg, chosen->thrust_n,
                                   chosen->mass_flow_kgps);
    autopilot_.state = AutopilotState::Burn;
    attitude_mode_ = {};  // d2 minor: a mode set before the burn no longer turns the vehicle back after it
    capsule_return_step_ = CapsuleReturnStep::None;  // a pilot burn ends a capsule return (its plan sets it after this)
    return true;
}

std::optional<size_t> FlightSession::burn_stage_index() const {
    if (bodies_.empty()) return std::nullopt;
    const auto& stages = bodies_[controlled_].body->vehicle().stages();
    for (size_t s = 0; s < stages.size(); ++s) {
        for (const auto& e : stages[s].engines) {
            if (!e.auxiliary()) return s;
        }
    }
    return std::nullopt;
}

vehicle::SettlingPush FlightSession::settling_push_now() const {
    const auto stage = burn_stage_index();
    if (!stage || !env_.ignition_settling || !bodies_[controlled_].body->in_orbit()) return {};
    return vehicle::settling_push(bodies_[controlled_].body->vehicle(), *stage);
}

double FlightSession::settling_lead_s() const { return settling_push_now().lead_s; }

double FlightSession::settling_gas_kg() const { return settling_push_now().gas_kg; }

bool FlightSession::settle(bool on) {
    if (on && docking_busy()) return false;
    settle_manual_ = on;
    return true;
}

bool FlightSession::plan_circularization_at_apoapsis() {
    if (bodies_.empty()) return false;
    const auto& s = bodies_[controlled_].body->state();
    const auto node = gnc::circularize_at_apoapsis({s.position_gcrf_m, s.velocity_gcrf_mps}, epoch_, gm_earth_);
    return node && execute_burn(*node);
}

}  // namespace starsim::sim
