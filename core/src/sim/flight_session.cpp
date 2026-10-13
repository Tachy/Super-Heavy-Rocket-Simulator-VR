#include "starsim/sim/flight_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/gnc/descent_snapshot.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/sim/landing_outcome.hpp"

namespace starsim::sim {

namespace {

constexpr double tower_keep_out_margin_m = 5.0;  // [E] aborted landings stay this far from the tower lattice
// A freely falling body this close to its ground is ticked: a coast segment ends with one ground-contact check, so
// a long one would fall through the ground and judge the touchdown at the speed it has reached far below it.
constexpr double ground_tick_height_m = 5000.0;

}  // namespace

FlightSession::FlightSession(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& earth,
                             std::filesystem::path config_dir, environment::PointMassGravityConfig gravity_config)
    : config_dir_(std::move(config_dir)),
      earth_(earth),
      gravity_(ephemeris, earth, gravity_config),
      env_{gravity_, atmosphere_, earth, &ephemeris, true},
      gm_earth_(ephemeris.gm_m3ps2(ephemeris::Body::Earth)) {}

const char* to_string(AutopilotState state) {
    switch (state) {
        case AutopilotState::Off: return "aus";
        case AutopilotState::PreLaunch: return "Startvorbereitung";
        case AutopilotState::Booster: return "Booster";
        case AutopilotState::UpperStage: return "Oberstufe";
        case AutopilotState::Orbit: return "Orbit erreicht";
        case AutopilotState::Burn: return "Manöver";
        case AutopilotState::Return: return "Rückkehr";
    }
    return "";
}

void FlightSession::engage_ascent_autopilot(const gnc::AscentTarget& target, const gnc::PitchProgram& program) {
    guidance_ = std::make_unique<gnc::AscentGuidance>(target, program, gm_earth_);
    autopilot_ = {AutopilotState::PreLaunch, {}};
    if (!bodies_.empty() && !bodies_[controlled_].body->clamped()) {
        guidance_->start(*bodies_[controlled_].body, earth_, epoch_);
        autopilot_.state = bodies_[controlled_].body->vehicle().stages().size() > 1 ? AutopilotState::Booster
                                                                                    : AutopilotState::UpperStage;
    }
}

void FlightSession::set_booster_return(bool enabled, gnc::BoosterReturnParameters parameters) {
    if (enabled) {
        booster_return_ = std::move(parameters);
    } else {
        booster_return_.reset();
    }
}

const gnc::ReturnCommand* FlightSession::return_status(size_t index) const {
    const auto& flying = bodies_.at(index);
    return flying.return_guidance ? &flying.return_command : nullptr;
}

const gnc::EntryCommand* FlightSession::entry_status(size_t index) const {
    const auto& flying = bodies_.at(index);
    return flying.entry_guidance ? &flying.entry_command : nullptr;
}

void FlightSession::place_entering_ship(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                                        time::Epoch epoch, gnc::EntryGuidanceParameters parameters) {
    place_body(config, state, epoch);
    start_entry_guidance(bodies_.front(), parameters);
}

void FlightSession::start_entry_guidance(Flying& flying, const gnc::EntryGuidanceParameters& parameters) {
    const auto& stage = flying.body->vehicle().stages().front().config;
    const auto schedule = gnc::load_alpha_schedule(config_dir_, stage.aero.entry_alpha_table);
    // gate ahead of the tower's catch axis (b3), else over the pad centre
    const auto target = gnc::ship_entry_target(return_site(flying.body->vehicle()), gnc::ship_landing_parameters());
    flying.entry_guidance = std::make_unique<gnc::EntryGuidance>(*flying.body, schedule, target, parameters);
}

bool FlightSession::plan_ship_return(const gnc::DeorbitParameters& parameters) {
    ship_return_ = false;
    deorbit_plan_.reset();
    deorbit_parameters_ = parameters;
    if (bodies_.empty()) return false;
    auto& flying = bodies_[controlled_];
    const auto& vehicle = flying.body->vehicle();
    const auto& stage = vehicle.stages().front().config;
    gnc::DeorbitPlan refused;
    if (stage.aero.entry_alpha_table.empty() || stage.deorbit_engine_groups.empty() || flying.entry_guidance ||
        flying.ship_landing) {
        refused.reason = "Keine Rückkehr möglich: kein Schiff mit Hitzeschild und Deorbit-Triebwerk im Orbit";
        deorbit_plan_ = refused;
        return false;
    }
    const auto configurations = burn_configurations(&stage.deorbit_engine_groups);
    if (configurations.empty()) {
        refused.reason = "Deorbit-Triebwerk nicht verfügbar";
        deorbit_plan_ = refused;
        return false;
    }
    const double exhaust_velocity_mps = configurations.front().thrust_n / configurations.front().mass_flow_kgps;
    const auto schedule = gnc::load_alpha_schedule(config_dir_, stage.aero.entry_alpha_table);
    const auto target = gnc::ship_entry_target(return_site(vehicle), gnc::ship_landing_parameters());
    const gnc::EntryGuidance onboard(*flying.body, schedule, target);
    const auto& s = flying.body->state();
    deorbit_plan_ = gnc::plan_deorbit({s.position_gcrf_m, s.velocity_gcrf_mps}, epoch_, gravity_, earth_, onboard.model(),
                                      target, vehicle.mass_properties().mass_kg, exhaust_velocity_mps,
                                      vehicle.propellant_kg(), stage.landing_propellant_kg, parameters);
    if (!deorbit_plan_->feasible) return false;
    if (!execute_burn(deorbit_plan_->node, stage.deorbit_engine_groups)) {
        deorbit_plan_->feasible = false;
        deorbit_plan_->reason = "Deorbit-Brennvorgang zu klein für das Triebwerk";
        return false;
    }
    ship_return_ = true;
    return true;
}

time::Epoch FlightSession::entry_start_epoch() const {
    const auto& s = bodies_[controlled_].body->state();
    const integrators::Event<6> interface{
        "entry interface",
        [&](double t_s, const integrators::State<6>& y) {
            const math::Vec3 r = transpose(earth_.fixed_to_gcrf(epoch_ + t_s).rotation) * math::Vec3{y[0], y[1], y[2]};
            return frames::fixed_to_geodetic(r).height_m - deorbit_parameters_.entry_interface_height_m;
        },
        integrators::Crossing::Falling};
    const auto coast = orbit::propagate_coast(gravity_, {s.position_gcrf_m, s.velocity_gcrf_mps}, epoch_, epoch_ + 6000.0,
                                              std::span(&interface, 1));
    return coast.event_index == 0 ? coast.epoch - entry_lead_s : epoch_;  // no entry ahead: guidance at once
}

void FlightSession::place_landing_ship(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                                       time::Epoch epoch, gnc::ShipLandingParameters parameters) {
    place_body(config, state, epoch);
    auto& flying = bodies_.front();
    flying.ship_landing = std::make_unique<gnc::ShipLandingGuidance>(return_site(flying.body->vehicle()), std::move(parameters));
    flying.ship_landing->set_realtime(realtime_planning_);
}

// Flip and landing burn of a ship: engine groups and throttle for this tick (attitude in command_for).
void FlightSession::run_ship_landing(Flying& flying) {
    if (flying.body->impacted()) return;
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

void FlightSession::place_in_orbit(const std::string& vehicle_id, const math::StateVector& state, time::Epoch epoch,
                                   size_t first_stage) {
    auto config = config::load_vehicle(config_dir_, vehicle_id);
    config.stages.erase(config.stages.begin(), config.stages.begin() + static_cast<long>(first_stage));
    auto vehicle = std::make_unique<vehicle::Vehicle>(config, config_dir_);
    // Body x along the velocity, body z along the orbit normal.
    const math::Vec3 x = math::normalized(state.velocity_mps);
    const math::Vec3 z = math::normalized(cross(state.position_m, state.velocity_mps));
    const math::Vec3 y = cross(z, x);
    math::Mat3 m;
    const math::Vec3 columns[3] = {x, y, z};
    for (int c = 0; c < 3; ++c) {
        m(0, c) = columns[c].x;
        m(1, c) = columns[c].y;
        m(2, c) = columns[c].z;
    }
    const vehicle::RigidBodyState rigid{state.position_m, state.velocity_mps, math::Quat::from_matrix(m), {}};
    bodies_.clear();
    bodies_.push_back({std::make_unique<vehicle::FlightBody>(std::move(vehicle), rigid), {}, nullptr, {}});
    controlled_ = 0;
    epoch_ = epoch;
    launch_epoch_ = epoch;
    autopilot_ = {};
    burn_.reset();
}

// Candidate engine configurations of the lowest stage with engines, strongest first: all groups at full
// throttle, each group alone at full throttle, each group alone at minimum throttle.
std::vector<FlightSession::BurnConfiguration> FlightSession::burn_configurations(const std::vector<std::string>* forced) const {
    std::vector<BurnConfiguration> configurations;
    if (bodies_.empty()) return configurations;
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    const double mass = vehicle.mass_properties().mass_kg;
    for (const auto& stage : vehicle.stages()) {
        if (stage.engines.empty()) continue;
        std::vector<std::string> groups;
        for (const auto& e : stage.engines) {
            if (std::find(groups.begin(), groups.end(), e.group_id) == groups.end()) groups.push_back(e.group_id);
        }
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
                if (e.starved || std::find(selected.begin(), selected.end(), e.group_id) == selected.end()) continue;
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
            for (const auto& g : *forced) selected.push_back(stage.config.id + "/" + g);
            configurations.push_back(make(selected, false));
            configurations.push_back(make(selected, true));
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
    return start_burn(node, burn_configurations(&groups));
}

bool FlightSession::start_burn(const gnc::BurnNode& node, const std::vector<BurnConfiguration>& configurations) {
    // Strongest configuration whose start/shutdown impulse is at most a quarter of the burn, so that
    // the transients stay small against the closed-loop part; otherwise the weakest that still fits.
    const double dv = norm(node.delta_v_gcrf_mps);
    const BurnConfiguration* chosen = nullptr;
    for (const auto& c : configurations) {
        if (c.minimum_dv_mps <= 0.25 * dv) {
            chosen = &c;
            break;
        }
    }
    BurnConfiguration rcs;
    if (chosen == nullptr && !bodies_.empty()) {
        // Below the main-engine minimum impulse: RCS translation (no transients), if the vehicle has RCS
        // and the burn takes less than an hour.
        const auto& vehicle = bodies_[controlled_].body->vehicle();
        rcs.rcs = true;
        rcs.thrust_n = vehicle.max_rcs_force_n();
        rcs.mass_flow_kgps = 1e-9;  // cold gas (ullage) mass neglected [E]
        if (rcs.thrust_n > 0.0 && dv / (rcs.thrust_n / vehicle.mass_properties().mass_kg) < 3600.0) {
            chosen = &rcs;
        }
    }
    if (chosen == nullptr) {
        return false;
    }
    burn_config_ = *chosen;
    burn_.emplace(node);
    autopilot_.state = AutopilotState::Burn;
    return true;
}

bool FlightSession::plan_circularization_at_apoapsis() {
    if (bodies_.empty()) return false;
    const auto& s = bodies_[controlled_].body->state();
    const auto node = gnc::circularize_at_apoapsis({s.position_gcrf_m, s.velocity_gcrf_mps}, epoch_, gm_earth_);
    return node && execute_burn(*node);
}

std::vector<math::Vec3> FlightSession::predict_trajectory(size_t index, double duration_s, double step_s) const {
    std::vector<math::Vec3> points;
    const auto& s = bodies_.at(index).body->state();
    // Impact on a sphere of the polar radius (display only; the flight itself uses the ellipsoid).
    const std::vector<integrators::Event<6>> impact{
        {"impact", [](double, const integrators::State<6>& y) { return std::hypot(y[0], y[1], y[2]) - 6356752.0; },
         integrators::Crossing::Falling}};
    double next = 0.0;
    const integrators::StepObserver<6> observer = [&](const integrators::Dop853<6>& solver) {
        while (next <= solver.t()) {
            const auto y = solver.dense_output(next);
            points.push_back({y[0], y[1], y[2]});
            next += step_s;
        }
    };
    auto options = orbit::default_coast_options();
    options.relative_tolerance = 1e-9;  // display accuracy is plenty
    options.absolute_tolerance = integrators::State<6>{{1.0, 1.0, 1.0, 1e-3, 1e-3, 1e-3}};
    const auto result = integrators::integrate<6>(orbit::ballistic_dynamics(gravity_, epoch_), 0.0,
                                                  math::pack({s.position_gcrf_m, s.velocity_gcrf_mps}), duration_s,
                                                  options, impact, observer);
    points.push_back({result.y[0], result.y[1], result.y[2]});
    return points;
}

void FlightSession::fail_engine(size_t body_index, size_t stage_index, size_t engine_index) {
    bodies_.at(body_index).body->vehicle().fail_engine(stage_index, engine_index);
}

void FlightSession::apply_velocity_error(size_t body_index, const math::Vec3& delta_v_gcrf_mps) {
    auto& body = *bodies_.at(body_index).body;
    auto state = body.state();
    state.velocity_gcrf_mps += delta_v_gcrf_mps;
    body.set_state(state);
}

// Discrete autopilot decisions, once per tick before integration.
void FlightSession::run_autopilot() {
    auto& flying = bodies_[controlled_];
    auto& body = *flying.body;
    switch (autopilot_.state) {
        case AutopilotState::PreLaunch: {
            ignite();
            bool all_running = true;
            for (const auto& e : body.vehicle().stages().front().engines) {
                all_running = all_running && (e.mode == vehicle::EngineMode::Running || e.starved);
            }
            if (all_running) {
                release_clamps();
                guidance_->start(body, earth_, epoch_);
                autopilot_.state = AutopilotState::Booster;
            }
            break;
        }
        case AutopilotState::Booster: {
            const auto& booster = body.vehicle().stages().front();
            const double propellant = booster.propellant_kg(config::TankContents::Oxidizer) +
                                      booster.propellant_kg(config::TankContents::Fuel);
            // A returning booster cuts off when it is down to what its return needs (planned from the state now);
            // otherwise at the configured reserve.
            double reserve_kg = booster.config.reserve_propellant_kg;
            if (booster.config.landing_propellant_kg > 0.0 && returns_after_staging(body.vehicle())) {
                const auto& s = body.state();
                reserve_kg = gnc::BoosterReturnGuidance::meco_propellant_kg(
                    body.vehicle(), {s.position_gcrf_m, s.velocity_gcrf_mps}, earth_, epoch_,
                    return_site(body.vehicle()), *booster_return_, booster.config.landing_propellant_kg);
            }
            if (propellant <= reserve_kg) {
                stage();  // MECO + hot staging; the upper stage becomes the controlled body
                autopilot_.state = AutopilotState::UpperStage;
            }
            break;
        }
        case AutopilotState::UpperStage:
            if (autopilot_.guidance.cutoff) {
                cutoff();
                autopilot_.state = AutopilotState::Orbit;
            }
            break;
        case AutopilotState::Burn: {
            const auto& c = burn_config_;
            const auto& vehicle = body.vehicle();
            const math::Vec3 thrust_axis =
                c.rcs ? math::Vec3{1.0, 0.0, 0.0} : gnc::thrust_axis_body(vehicle, vehicle.mass_properties().cg_station_m, c.groups);
            autopilot_.burn = burn_->update(body, epoch_, tick_s, c.thrust_n, c.mass_flow_kgps, c.shutdown_s,
                                            last_rcs_acceleration_, thrust_axis);
            if (c.rcs) {
                if (autopilot_.burn.complete) autopilot_.state = AutopilotState::Off;
                break;
            }
            const bool running = std::any_of(flying.running_groups.begin(), flying.running_groups.end(),
                                             [](const auto& g) { return g.second; });
            if (autopilot_.burn.ignite && !running) {
                for (const auto& g : c.groups) flying.running_groups[g] = true;
            }
            if (autopilot_.burn.complete) {
                cutoff();
                autopilot_.state = ship_return_ ? AutopilotState::Return : AutopilotState::Off;
                if (ship_return_) entry_start_ = entry_start_epoch();  // from the state the burn actually left
            }
            break;
        }
        case AutopilotState::Return:
            if (!flying.entry_guidance && !flying.ship_landing && epoch_ >= entry_start_) {
                start_entry_guidance(flying, {});
            }
            break;
        default:
            break;
    }
}


bool FlightSession::returns_after_staging(const vehicle::Vehicle& vehicle) const {
    // Return guidance only for a booster that is caught by a tower at this site: Super Heavy has no legs (project
    // decision 2026-10-13: it is always caught); Falcon 9 pad / drone-ship landings are not modelled yet.
    return booster_return_ && vehicle.stages().front().config.recovery_method == "tower_catch" &&
           launch_site_.tower.has_value();
}

LaunchSite launch_site_from_config(const config::LaunchSiteConfig& site) {
    return {site.latitude_rad, site.longitude_rad, site.height_m, site.tower, site.sea};
}

gnc::LandingSite FlightSession::return_site(const vehicle::Vehicle& booster) const {
    gnc::LandingSite landing{launch_site_.latitude_rad, launch_site_.longitude_rad, launch_site_.height_m, {}, {}};
    const auto& stage = booster.stages().front();
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
                                                c.mount_height_m};
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
    flying.return_guidance->skip_to(start_phase);
    flying.return_guidance->set_realtime(realtime_planning_);
}

void FlightSession::set_realtime_planning(bool realtime) {
    realtime_planning_ = realtime;
    for (auto& flying : bodies_) {
        if (flying.return_guidance) flying.return_guidance->set_realtime(realtime);
        if (flying.ship_landing) flying.ship_landing->set_realtime(realtime);
    }
}

void FlightSession::set_launch_site(const LaunchSite& site) {
    launch_site_ = site;
    tower_.reset();
    if (towers_enabled_ && site.tower) {
        tower_ = std::make_unique<CatchTowerSim>(*site.tower, site.latitude_rad, site.longitude_rad, site.height_m);
    }
}

std::vector<vehicle::FlightBody*> FlightSession::body_pointers() const {
    std::vector<vehicle::FlightBody*> list;
    for (const auto& flying : bodies_) list.push_back(flying.body.get());
    return list;
}

void FlightSession::place_body(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                               time::Epoch epoch) {
    auto body = std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(config, config_dir_), state);
    body->set_ground_height(launch_site_.height_m);
    bodies_.clear();
    bodies_.push_back({std::move(body), {}, nullptr, {}});
    controlled_ = 0;
    epoch_ = epoch;
    launch_epoch_ = epoch;
    autopilot_ = {};
    burn_.reset();
    ship_return_ = false;
}

void FlightSession::launch_from_pad(const std::string& vehicle_id, const LaunchSite& site, double heading_rad,
                                    time::Epoch epoch) {
    launch_from_pad(config::load_vehicle(config_dir_, vehicle_id), site, heading_rad, epoch);
}

void FlightSession::launch_from_pad(const config::VehicleConfig& config, const LaunchSite& site, double heading_rad,
                                    time::Epoch epoch) {
    auto vehicle = std::make_unique<vehicle::Vehicle>(config, config_dir_);
    set_launch_site(site);
    const double stand_height = site.height_m + (tower_ ? site.tower->mount_height_m : 0.0);
    bodies_.clear();
    bodies_.push_back({vehicle::FlightBody::on_pad(std::move(vehicle), site.latitude_rad, site.longitude_rad,
                                                   stand_height, heading_rad, earth_, epoch),
                       {}, nullptr, {}});
    // The stack stands on the launch mount: its top is the ground until staging (a stack released without
    // thrust settles back onto it instead of dropping through the mount's open centre).
    bodies_.back().body->set_ground_height(stand_height);
    controlled_ = 0;
    epoch_ = epoch;
    launch_epoch_ = epoch;
}

void FlightSession::apply_return_command(const Flying& flying, const gnc::AttitudeController& controller,
                                         double ambient_pressure_pa, vehicle::VehicleCommand& command) const {
    const auto& r = flying.return_command;
    for (auto& group : command.groups) group.throttle = r.throttle;
    if (r.torque_body_nm) {
        // SCvx lateral LQR: the guidance's torque straight to gimbals / RCS (grid fins neutral: the planning model has
        // none, and at low speed their allocation fights the gimbals).
        gnc::AttitudeController::allocate(*flying.body, ambient_pressure_pa, *r.torque_body_nm, false).apply_to(command);
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

vehicle::VehicleCommand FlightSession::command_for(size_t index, double ambient_pressure_pa) const {
    vehicle::VehicleCommand command;
    const auto& flying = bodies_[index];
    for (const auto& [group, run] : flying.running_groups) {
        command.groups.push_back({group, run, index == controlled_ ? input_.throttle : 1.0});
    }
    if (flying.ship_landing) {
        if (!flying.body->impacted()) apply_return_command(flying, ship_landing_controller_, ambient_pressure_pa, command);
        return command;
    }
    if (flying.entry_guidance) {
        if (!flying.body->impacted()) {
            const auto& e = flying.entry_command;
            const gnc::AttitudeController::Feedforward feedforward{
                flying.body->state().attitude.inverse_rotate(e.rate_gcrf_radps), {}};
            entry_controller_.steer(*flying.body, ambient_pressure_pa, e.axis_gcrf, e.roll_reference_gcrf, &feedforward)
                .apply_to(command);
        }
        return command;
    }
    if (index != controlled_ || flying.return_guidance) {
        if (flying.return_guidance && !flying.body->impacted()) {
            apply_return_command(flying, return_controller_, ambient_pressure_pa, command);
        }
        return command;
    }
    const bool guided = autopilot_.state == AutopilotState::Booster || autopilot_.state == AutopilotState::UpperStage;
    if (guided || autopilot_.state == AutopilotState::Burn) {
        const double throttle = guided ? 1.0 : burn_config_.throttle;
        for (auto& group : command.groups) group.throttle = throttle;
        const auto& direction = guided ? autopilot_.guidance.thrust_direction_gcrf : autopilot_.burn.direction_gcrf;
        const auto& vehicle = flying.body->vehicle();
        const math::Vec3 thrust_axis = guided || burn_config_.rcs
                                           ? math::Vec3{1.0, 0.0, 0.0}
                                           : gnc::thrust_axis_body(vehicle, vehicle.mass_properties().cg_station_m,
                                                                   burn_config_.groups);
        if (thrust_axis.x < 1.0 - 1e-9) {
            // Asymmetric set (the ship's deorbit Raptor): its thrust goes through the CG along an axis tilted against the
            // hull; the hull turns so that this axis points along the burn, its roll kept where it is.
            const auto& attitude = flying.body->state().attitude;
            const math::Vec3 x0 = math::normalized(direction);
            const math::Vec3 y_now = attitude.rotate(math::Vec3{0.0, 1.0, 0.0});
            const math::Vec3 y0 = math::normalized(y_now - x0 * dot(y_now, x0));
            const math::Vec3 z0 = cross(x0, y0);
            math::Mat3 m;
            const math::Vec3 columns[3] = {x0, y0, z0};
            for (int c = 0; c < 3; ++c) {
                m(0, c) = columns[c].x;
                m(1, c) = columns[c].y;
                m(2, c) = columns[c].z;
            }
            const math::Vec3 x{1.0, 0.0, 0.0};
            const math::Quat to_x = math::Quat::from_axis_angle(cross(thrust_axis, x), std::acos(std::clamp(thrust_axis.x, -1.0, 1.0)));
            const math::Quat hull = math::Quat::from_matrix(m) * to_x;  // hull.rotate(thrust_axis) = x0
            controller_.steer(*flying.body, ambient_pressure_pa, hull.rotate(x), hull.rotate(math::Vec3{0.0, 1.0, 0.0}))
                .apply_to(command);
        } else {
            controller_.steer(*flying.body, ambient_pressure_pa, direction).apply_to(command);
        }
        if (!guided && burn_config_.rcs && autopilot_.burn.ignite && !autopilot_.burn.complete) {
            // Translation along the remaining delta-v, independent of the attitude.
            command.rcs_force_fraction = flying.body->state().attitude.inverse_rotate(autopilot_.burn.direction_gcrf);
        }
        return command;
    }
    // Axis convention: body +y points downrange (launch heading), z = x cross y. Pitch input > 0 tips
    // the nose towards +y (rotation about +z); yaw input > 0 turns it to the right of the heading, i.e.
    // towards -z (rotation about +y; z = up x heading points to the left).
    if (mode_ == ControlMode::RateCommand) {
        const math::Vec3 rates{input_.roll * max_roll_rate_radps, input_.yaw * max_pitch_yaw_rate_radps,
                               input_.pitch * max_pitch_yaw_rate_radps};
        controller_.steer_rates(*flying.body, ambient_pressure_pa, rates).apply_to(command);
    } else {
        double max_gimbal = 0.0;
        for (const auto& stage : flying.body->vehicle().stages()) {
            for (const auto& e : stage.engines) max_gimbal = std::max(max_gimbal, e.performance->config().gimbal_max_rad);
        }
        // Engines sit below the CG: a deflection towards +y turns the nose towards -y, towards +z the
        // nose towards -z (right).
        command.gimbal_yaw_rad = -input_.pitch * max_gimbal;
        command.gimbal_pitch_rad = input_.yaw * max_gimbal;
        command.rcs_torque_fraction = {input_.roll, input_.yaw, input_.pitch};  // same sense as the gimbals
        command.gimbal_roll_rad = input_.roll * max_gimbal;
    }
    return command;
}

// How long all bodies may coast from now: nothing firing or commanded, no ascent guidance, no pilot
// input, rotation settled, and far enough from a planned ignition.
double FlightSession::coast_allowance_s() const {
    if (bodies_.empty()) return 0.0;
    const bool guided = autopilot_.state == AutopilotState::PreLaunch || autopilot_.state == AutopilotState::Booster ||
                        autopilot_.state == AutopilotState::UpperStage;
    if (guided) return 0.0;
    if (input_.pitch != 0.0 || input_.yaw != 0.0 || input_.roll != 0.0) return 0.0;
    double allowance = max_coast_s;
    if (autopilot_.state == AutopilotState::Burn) {
        allowance = std::min(allowance, autopilot_.burn.time_to_ignition_s - burn_preparation_s);
    }
    if (autopilot_.state == AutopilotState::Return && !bodies_[controlled_].entry_guidance) {
        allowance = std::min(allowance, entry_start_ - epoch_);
    }
    if (tower_) allowance = std::min(allowance, tower_->coast_limit_s(body_pointers(), earth_, epoch_));
    for (const auto& flying : bodies_) {
        const auto& body = *flying.body;
        if (flying.return_guidance && flying.return_guidance->phase() != gnc::ReturnPhase::Landed &&
            !body.impacted()) {
            return 0.0;
        }
        if ((flying.entry_guidance || flying.ship_landing) && !body.impacted() && !body.resting() && !body.caught()) {
            return 0.0;  // steered every tick
        }
        if (body.impacted() || body.resting() || body.caught()) continue;  // no longer moving on its own
        if (!body.can_coast()) return 0.0;
        {
            const auto t = earth_.fixed_to_gcrf(epoch_);
            const double height = frames::fixed_to_geodetic(transpose(t.rotation) * body.state().position_gcrf_m).height_m -
                                  body.ground_height_m();
            if (height < ground_tick_height_m) return 0.0;
            const math::Vec3 up = math::normalized(body.state().position_gcrf_m);
            const double sink = -dot(body.state().velocity_gcrf_mps, up);
            if (sink > 0.0) allowance = std::min(allowance, (height - ground_tick_height_m) / sink);
        }
        for (const auto& [group, run] : flying.running_groups) {
            if (run) return 0.0;
        }
        if (norm(body.state().angular_velocity_body_radps) > 1e-3 && &flying == &bodies_[controlled_]) return 0.0;
    }
    return allowance > 1.0 ? allowance : 0.0;
}

time::Epoch FlightSession::advance_to(time::Epoch target, int max_ticks) {
    for (int tick = 0; tick < max_ticks && target - epoch_ >= tick_s; ++tick) {
        if (const double coast = std::min(coast_allowance_s(), target - epoch_); coast > 1.0) {
            for (auto& flying : bodies_) flying.body->coast(env_, epoch_, coast);
            epoch_ += coast;
            if (autopilot_.state == AutopilotState::Burn) {
                autopilot_.burn.time_to_ignition_s -= coast;  // refreshed on the next tick
            }
            continue;
        }
        if (autopilot_.state != AutopilotState::Off && autopilot_.state != AutopilotState::Orbit &&
            !bodies_.empty()) {
            run_autopilot();
            const bool closed_loop = autopilot_.state == AutopilotState::UpperStage;
            if (autopilot_.state == AutopilotState::Booster || closed_loop) {
                autopilot_.guidance = guidance_->update(*bodies_[controlled_].body, env_, epoch_, closed_loop);
            }
        }
        for (size_t i = 0; i < bodies_.size(); ++i) {
            if (bodies_[i].return_guidance) {
                if (tower_) bodies_[i].return_guidance->set_catch_contact(tower_->in_contact(*bodies_[i].body));
                run_return_guidance(bodies_[i]);
            }
            auto& flying = bodies_[i];
            if (flying.entry_guidance && !flying.ship_landing && !flying.body->impacted()) {
                flying.entry_command = flying.entry_guidance->update(*flying.body, env_, epoch_);
                if (flying.entry_command.phase == gnc::EntryPhase::Gate) {  // the flip and the catch take over
                    flying.ship_landing = std::make_unique<gnc::ShipLandingGuidance>(return_site(flying.body->vehicle()));
                    flying.ship_landing->set_realtime(realtime_planning_);
                }
            }
            if (flying.ship_landing) run_ship_landing(flying);
        }
        const auto pointers = body_pointers();
        if (tower_) tower_->before_tick(pointers, earth_, epoch_);
        for (size_t i = 0; i < bodies_.size(); ++i) {
            auto& body = *bodies_[i].body;
            const auto t = earth_.fixed_to_gcrf(epoch_);
            const double altitude =
                frames::fixed_to_geodetic(transpose(t.rotation) * body.state().position_gcrf_m).height_m;
            const auto command = command_for(i, atmosphere_.at(altitude).pressure_pa);
            if (i == controlled_) {
                const double force = norm(command.rcs_force_fraction) > 0.0 ? body.vehicle().max_rcs_force_n() : 0.0;
                last_rcs_acceleration_ = body.state().attitude.rotate(math::normalized_or_zero(command.rcs_force_fraction)) *
                                         (force / body.vehicle().mass_properties().mass_kg);
            }
            body.step(env_, epoch_, tick_s, command);
        }
        if (tower_) tower_->after_tick(pointers, earth_, epoch_ + tick_s, tick_s);
        epoch_ += tick_s;
    }
    if (bodies_.empty()) {
        epoch_ = target;
    }
    return epoch_;
}

void FlightSession::ignite() {
    if (bodies_.empty()) return;
    auto& flying = bodies_[controlled_];
    for (const auto& stage : flying.body->vehicle().stages()) {
        if (stage.engines.empty()) continue;
        for (const auto& e : stage.engines) flying.running_groups[e.group_id] = true;
        break;  // lowest stage with engines only
    }
}

void FlightSession::cutoff() {
    if (bodies_.empty()) return;
    for (auto& [group, run] : bodies_[controlled_].running_groups) run = false;
}

void FlightSession::release_clamps() {
    if (bodies_.empty() || !bodies_[controlled_].body->clamped()) return;
    bodies_[controlled_].body->release_clamps();
    launch_epoch_ = epoch_;  // T+0 = liftoff
}

void FlightSession::stage() {
    if (bodies_.empty() || bodies_[controlled_].body->vehicle().stages().size() < 2) return;
    auto& lower = bodies_[controlled_];
    auto upper = lower.body->separate(1);
    for (auto& [group, run] : lower.running_groups) run = false;  // booster engines off after separation
    lower.body->set_ground_height(launch_site_.height_m);  // separated stages: the real ground, not the mount
    upper->set_ground_height(launch_site_.height_m);
    if (returns_after_staging(lower.body->vehicle())) {
        const auto landing = return_site(lower.body->vehicle());
        lower.return_guidance = std::make_unique<gnc::BoosterReturnGuidance>(landing, *booster_return_);
        lower.return_guidance->set_realtime(realtime_planning_);
    }
    Flying flying{std::move(upper), {}, nullptr, {}};
    bodies_.push_back(std::move(flying));
    controlled_ = bodies_.size() - 1;
    ignite();  // hot staging: the upper stage lights immediately
}

LandingOutcome FlightSession::landing_outcome(size_t index) const {
    const auto t = earth_.fixed_to_gcrf(epoch_);
    const auto& body = *bodies_.at(index).body;
    return sim::landing_outcome(body, launch_site_, transpose(t.rotation) * body.state().position_gcrf_m);
}

vehicle::FlightTelemetry FlightSession::telemetry(size_t index) const {
    return bodies_.at(index).body->telemetry(env_, epoch_);
}

void FlightSession::deploy_legs(size_t body_index) { bodies_.at(body_index).body->vehicle().deploy_legs(); }

void FlightSession::set_leg_deployment(size_t body_index, double fraction) {
    bodies_.at(body_index).body->vehicle().set_leg_deployment(fraction);
}

}  // namespace starsim::sim
