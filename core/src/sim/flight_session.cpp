#include "starsim/sim/flight_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/sim/landing_outcome.hpp"

namespace starsim::sim {

namespace {

// A freely falling body this close to its ground is ticked: a coast segment ends with one ground-contact check, so
// a long one would fall through the ground and judge the touchdown at the speed it has reached far below it.
constexpr double ground_tick_height_m = 5000.0;

}  // namespace

FlightSession::FlightSession(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& earth,
                             std::filesystem::path config_dir, environment::PointMassGravityConfig gravity_config,
                             const frames::BodyOrientation* moon_orientation)
    : config_dir_(std::move(config_dir)),
      earth_(earth),
      gravity_(ephemeris, earth, gravity_config),
      moon_(moon_orientation ? std::optional<environment::ReferenceBody>(
                                   environment::ReferenceBody::moon(ephemeris, *moon_orientation))
                             : std::nullopt),
      env_{gravity_, atmosphere_, earth, &ephemeris, true, moon_ ? &*moon_ : nullptr},
      gm_earth_(ephemeris.gm_m3ps2(ephemeris::Body::Earth)) {
    legs_ = std::make_unique<LegLandingSim>(0.0, 0.0, 0.0, contact::LandingSurface{}, tick_s);
    moon_orientation_ = moon_orientation;
    lunar_debug_ = std::getenv("STARSIM_DEBUG_LUNAR_SESSION") != nullptr;
}

const char* to_string(AutopilotState state) {
    switch (state) {
        case AutopilotState::Off: return "aus";
        case AutopilotState::PreLaunch: return "Startvorbereitung";
        case AutopilotState::Booster: return "Booster";
        case AutopilotState::UpperStage: return "Oberstufe";
        case AutopilotState::PayloadSeparation: return "Nutzlasttrennung";
        case AutopilotState::Orbit: return "Orbit erreicht";
        case AutopilotState::Burn: return "Manöver";
        case AutopilotState::Return: return "Rückkehr";
        case AutopilotState::Lunar: return "Mondflug";
    }
    return "";
}

void FlightSession::engage_ascent_autopilot(const gnc::AscentTarget& target, const gnc::PitchProgram& program) {
    guidance_ = std::make_unique<gnc::AscentGuidance>(target, program, gm_earth_);
    autopilot_ = {AutopilotState::PreLaunch, {}};
    if (!bodies_.empty() && !bodies_[controlled_].body->clamped()) {
        guidance_->start(*bodies_[controlled_].body, earth_, epoch_);
        // Booster while more than one launcher stage with engines is left (a stacked payload's stages and its abort
        // engines do not count: [second stage, trunk, capsule] resumes as the upper stage).
        int powered = 0;
        for (const auto& stage : bodies_[controlled_].body->vehicle().stages()) {
            const bool engines = std::any_of(stage.engines.begin(), stage.engines.end(),
                                             [](const vehicle::Engine& e) { return !e.auxiliary(); });
            powered += !stage.config.payload && engines ? 1 : 0;
        }
        autopilot_.state = powered > 1 ? AutopilotState::Booster : AutopilotState::UpperStage;
    }
}

void FlightSession::place_in_orbit(const std::string& vehicle_id, const math::StateVector& state, time::Epoch epoch,
                                   size_t first_stage) {
    auto config = config::load_vehicle(config_dir_, vehicle_id);
    config.stages.erase(config.stages.begin(), config.stages.begin() + static_cast<long>(first_stage));
    place_in_orbit(config, state, epoch);
}

void FlightSession::place_in_orbit(const config::VehicleConfig& config, const math::StateVector& state,
                                   time::Epoch epoch) {
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
    reset_leg_landing();
    bodies_.push_back({std::make_unique<vehicle::FlightBody>(std::move(vehicle), rigid), {}, nullptr, {}});
    controlled_ = 0;
    epoch_ = epoch;
    launch_epoch_ = epoch;
    autopilot_ = {};
    burn_.reset();
    attitude_mode_ = {};
    payload_separation_.reset();  // d2 minor: an earlier flight's release and cone timer do not carry over
    nose_cone_epoch_.reset();
    capsule_return_step_ = CapsuleReturnStep::None;
    reset_lunar();
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

bool FlightSession::abort_landing(size_t body_index) {
    auto& flying = bodies_.at(body_index);
    if (body_index == controlled_ && autopilot_.state == AutopilotState::Lunar &&
        lunar_step_ == LunarStep::PoweredDescent && lunar_landing_) {
        return lunar_abort_to_orbit();  // lunar l4: abort to orbit, or refused and the landing continues
    }
    bool accepted = false;
    if (flying.return_guidance) accepted = flying.return_guidance->request_abort();
    if (flying.ship_landing) accepted = flying.ship_landing->request_abort();
    if (accepted && tower_) tower_->abort_catch();  // never open the arms under a body still aiming at them
    return accepted;
}

void FlightSession::apply_angular_rate(size_t body_index, const math::Vec3& rate_body_radps) {
    auto& body = *bodies_.at(body_index).body;
    auto state = body.state();
    state.angular_velocity_body_radps = rate_body_radps;
    body.set_state(state);
}

bool FlightSession::attitude_mode_active() const {
    return attitude_mode_.kind != gnc::AttitudeModeKind::Free &&
           (autopilot_.state == AutopilotState::Off || autopilot_.state == AutopilotState::Orbit ||
            autopilot_.state == AutopilotState::PayloadSeparation || autopilot_.state == AutopilotState::Lunar);
}

bool FlightSession::separate_payload() {
    if (bodies_.empty() || !payload_separation_) return false;
    auto& lower = bodies_[controlled_];
    const auto& stages = lower.body->vehicle().stages();
    size_t index = 0;
    for (size_t i = 1; i < stages.size(); ++i) {
        if (stages[i].config.id == payload_separation_->first_stage_id) index = i;
    }
    if (index == 0) return false;
    auto upper = lower.body->separate(index, payload_separation_->dv_mps);
    for (auto& [group, run] : lower.running_groups) run = false;
    upper->set_ground_height(launch_site_.height_m);
    bodies_.push_back({std::move(upper), {}, nullptr, {}});
    controlled_ = bodies_.size() - 1;
    attitude_mode_ = gnc::inertial_hold(bodies_[controlled_].body->state().attitude);
    if (payload_separation_->nose_cone_open_s >= 0.0) {
        nose_cone_epoch_ = epoch_ + payload_separation_->nose_cone_open_s;
        nose_cone_body_ = controlled_;
    }
    return true;
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
                all_running = all_running && (e.mode == vehicle::EngineMode::Running || e.starved || e.auxiliary());
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
                    return_site(body.vehicle()), return_parameters(booster.config),
                    booster.config.landing_propellant_kg);
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
                if (payload_separation_) {
                    autopilot_.state = AutopilotState::PayloadSeparation;
                    separation_epoch_ = epoch_ + payload_separation_->delay_s;
                    // the stack holds its attitude on the upper stage's thrusters until the release
                    attitude_mode_ = gnc::inertial_hold(body.state().attitude);
                }
            }
            break;
        case AutopilotState::PayloadSeparation:
            if (epoch_ >= separation_epoch_) {
                separate_payload();
                autopilot_.state = AutopilotState::Orbit;
            }
            break;
        case AutopilotState::Burn: {
            const auto& c = burn_config_;
            const auto& vehicle = body.vehicle();
            const math::Vec3 thrust_axis =
                c.rcs ? c.thrust_axis_body : gnc::thrust_axis_body(vehicle, vehicle.mass_properties().cg_station_m, c.groups);
            autopilot_.burn = burn_->update(body, epoch_, tick_s, c.thrust_n, c.mass_flow_kgps, c.shutdown_s,
                                            last_rcs_acceleration_, thrust_axis);
            if (c.rcs) {
                if (autopilot_.burn.complete) {
                    // a lunar flight's burn returns to its step machine
                    autopilot_.state = lunar_step_ != LunarStep::None ? AutopilotState::Lunar : AutopilotState::Off;
                    if (capsule_return_step_ == CapsuleReturnStep::DeorbitBurn) {
                        autopilot_.state = AutopilotState::Return;
                        capsule_return_step_ = CapsuleReturnStep::TrunkJettison;
                        capsule_step_epoch_ = epoch_ + trunk_jettison_delay_s;
                        capsule_hold_attitude_ = body.state().attitude;
                        entry_start_ = entry_start_epoch();  // from the state the burn actually left
                    }
                }
                break;
            }
            const bool running = std::any_of(flying.running_groups.begin(), flying.running_groups.end(),
                                             [](const auto& g) { return g.second; });
            if (autopilot_.burn.ignite && !running) {
                for (const auto& g : c.groups) flying.running_groups[g] = true;
            }
            if (autopilot_.burn.complete) {
                cutoff();
                autopilot_.state = lunar_step_ != LunarStep::None ? AutopilotState::Lunar
                                   : ship_return_                 ? AutopilotState::Return
                                                                  : AutopilotState::Off;
                if (ship_return_) entry_start_ = entry_start_epoch();  // from the state the burn actually left
            }
            break;
        }
        case AutopilotState::Return:
            if (capsule_return_step_ != CapsuleReturnStep::None) {
                run_capsule_return(flying);
                break;
            }
            if (!flying.entry_guidance && !flying.ship_landing && epoch_ >= entry_start_) {
                const bool capsule = flying.body->vehicle().stages().front().config.aero.entry_profile == "capsule";
                start_entry_guidance(flying, capsule ? gnc::EntryGuidanceParameters::capsule() : gnc::EntryGuidanceParameters{});
            }
            break;
        case AutopilotState::Lunar:
            run_lunar(flying);
            break;
        default:
            break;
    }
}

LaunchSite launch_site_from_config(const config::LaunchSiteConfig& site) {
    return {site.latitude_rad, site.longitude_rad, site.height_m, site.tower, site.sea, site.pad_radius_m, {}};
}

LaunchSite drone_ship_site(const config::DroneShipConfig& ship, double latitude_rad, double longitude_rad,
                           double sea_height_m, double heading_rad) {
    LaunchSite site;
    site.latitude_rad = latitude_rad;
    site.longitude_rad = longitude_rad;
    site.height_m = sea_height_m + ship.freeboard_m;
    site.pad_radius_m = 0.5 * ship.deck_width_m;
    site.deck = contact::LandingSurface::Deck{0.5 * ship.deck_length_m, 0.5 * ship.deck_width_m, heading_rad,
                                              ship.freeboard_m};
    site.sea = config::SeaConfig{heading_rad + 0.5 * constants::pi, 0.5 * ship.deck_width_m};
    return site;
}

void FlightSession::set_aero_dispersion(size_t body_index, double factor) {
    bodies_.at(body_index).body->vehicle().set_hull_aero_scale(factor);
}

void FlightSession::set_thrust_dispersion(size_t body_index, double factor) {
    bodies_.at(body_index).body->vehicle().set_thrust_factor(factor);
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
    landing_zone_.reset();
    contact::LandingSurface ground;
    ground.sea = site.sea;
    legs_ = std::make_unique<LegLandingSim>(site.latitude_rad, site.longitude_rad, site.height_m, std::move(ground), tick_s);
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
    body->set_ground_height(body->vehicle().splashdown_recovery() ? 0.0 : launch_site_.height_m);  // a capsule: the sea
    bodies_.clear();
    reset_leg_landing();
    bodies_.push_back({std::move(body), {}, nullptr, {}});
    controlled_ = 0;
    epoch_ = epoch;
    launch_epoch_ = epoch;
    reset_mission_state();
}

void FlightSession::reset_mission_state() {
    autopilot_ = {};
    guidance_.reset();
    burn_.reset();
    ship_return_ = false;
    deorbit_plan_.reset();
    attitude_mode_ = {};
    payload_separation_.reset();
    nose_cone_epoch_.reset();
    capsule_return_step_ = CapsuleReturnStep::None;  // review d3: a return does not carry into a new flight
    reset_lunar();
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
    reset_mission_state();  // a new flight never inherits the previous one's autopilot, burn or return plan
}

// How long all bodies may coast from now: nothing firing or commanded, no ascent guidance, no pilot
// input, rotation settled, and far enough from a planned ignition.
double FlightSession::coast_allowance_s() const {
    if (bodies_.empty()) return 0.0;
    const bool guided = autopilot_.state == AutopilotState::PreLaunch || autopilot_.state == AutopilotState::Booster ||
                        autopilot_.state == AutopilotState::UpperStage ||
                        autopilot_.state == AutopilotState::PayloadSeparation;
    if (guided) return 0.0;
    if (attitude_mode_active()) return 0.0;  // pulsed RCS hold: ticked
    if (autopilot_.state == AutopilotState::Return &&
        (capsule_return_step_ == CapsuleReturnStep::TrunkJettison ||
         capsule_return_step_ == CapsuleReturnStep::NoseConeClose || capsule_return_step_ == CapsuleReturnStep::Coast)) {
        return 0.0;  // capsule return: timed steps and the turn to the trim attitude, ticked (the entry: per body below)
    }
    // Under the parachutes the capsule is ticked by its watch height; once it is down (resting or impacted) one more tick
    // ends the return (gewassert / verloren) - review d5 I2: a time-warp call ending on the contact tick coasted past it.
    if (autopilot_.state == AutopilotState::Return &&
        (capsule_return_step_ == CapsuleReturnStep::Gate || capsule_return_step_ == CapsuleReturnStep::Drogues ||
         capsule_return_step_ == CapsuleReturnStep::Mains)) {
        return 0.0;
    }
    if (input_.pitch != 0.0 || input_.yaw != 0.0 || input_.roll != 0.0 || norm(input_.translation) > 0.0) return 0.0;
    double allowance = max_coast_s;
    if (autopilot_.state == AutopilotState::Lunar) {
        if (lunar_step_ == LunarStep::PilotCorrection) return 0.0;  // the crew flies: ticked
        if (lunar_step_ == LunarStep::PoweredDescent) return 0.0;   // the descent guidance flies: ticked
        if (lunar_step_ == LunarStep::Ascent || lunar_step_ == LunarStep::AbortAscent) return 0.0;  // ascent: ticked
        // the next timed step (ignition preparation then comes from the state Burn), and the next Sun re-point
        allowance = std::min(allowance, lunar_step_epoch_ - epoch_);
        if (lunar_step_ == LunarStep::Coast || lunar_step_ == LunarStep::DescentWait) allowance = std::min(allowance, lunar_next_repoint_ - epoch_);
    }
    if (nose_cone_epoch_) allowance = std::min(allowance, *nose_cone_epoch_ - epoch_);
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
            const auto clearance = body.ground_clearance(env_, epoch_);  // over the body's reference body (lunar l1)
            // a capsule ticks from above its parachute deployment heights (no coast past them)
            const double tick_height =
                flying.parachutes && flying.parachutes->phase() != gnc::ParachutePhase::Down
                    ? std::max(ground_tick_height_m, flying.parachutes->watch_height_m())
                    : ground_tick_height_m;
            if (clearance.height_m < tick_height) return 0.0;
            if (clearance.sink_mps > 0.0) {
                allowance = std::min(allowance, (clearance.height_m - tick_height) / clearance.sink_mps);
            }
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
        if (nose_cone_epoch_ && epoch_ >= *nose_cone_epoch_) {
            if (nose_cone_body_ < bodies_.size()) set_nose_cone(nose_cone_body_, true);
            nose_cone_epoch_.reset();
        }
        // Parachute sequencers before the coast check: a body that has not ticked yet (placed, staged, released)
        // must already limit its coast to the watch height (review d4: a placed capsule coasted past 5.5 km).
        for (auto& flying : bodies_) {
            if (!flying.parachutes && !flying.body->vehicle().stages().front().config.parachutes.empty()) {
                flying.parachutes = std::make_unique<gnc::ParachuteSequencer>(
                    flying.body->vehicle().stages().front().config.parachutes);
            }
        }
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
                bodies_[i].return_guidance->set_leg_contact(legs_->in_contact(*bodies_[i].body));
                run_return_guidance(bodies_[i]);
                auto& v = bodies_[i].body->vehicle();
                if (bodies_[i].return_command.deploy_legs && v.leg_deployment() == 0.0) v.deploy_legs();
            }
            auto& flying = bodies_[i];
            if (flying.entry_guidance && !flying.ship_landing && !flying.body->impacted()) {
                flying.entry_command = flying.entry_guidance->update(*flying.body, env_, epoch_);
                if (flying.entry_command.phase == gnc::EntryPhase::Gate && !flying.entry_guidance->capsule()) {
                    // the flip and the catch take over
                    flying.ship_landing = std::make_unique<gnc::ShipLandingGuidance>(return_site(flying.body->vehicle()));
                    flying.ship_landing->set_realtime(realtime_planning_);
                }
            }
            if (flying.ship_landing) run_ship_landing(flying);
            if (flying.parachutes && !flying.body->impacted()) run_parachutes(flying);
        }
        const auto pointers = body_pointers();
        if (tower_) tower_->before_tick(pointers, earth_, epoch_);
        legs_->before_tick(pointers, env_, epoch_);
        for (size_t i = 0; i < bodies_.size(); ++i) {
            auto& body = *bodies_[i].body;
            const auto t = earth_.fixed_to_gcrf(epoch_);
            const double altitude =
                frames::fixed_to_geodetic(transpose(t.rotation) * body.state().position_gcrf_m).height_m;
            const auto command = command_for(i, atmosphere_.at(altitude).pressure_pa);
            body.step(env_, epoch_, tick_s, command);
            if (i == controlled_) {
                // What the thrusters that fired gave (pulses), for the burn executor - only while a translation is
                // commanded: the net force of attitude pulses before ignition would otherwise start the burn early.
                last_rcs_acceleration_ =
                    norm(command.rcs_force_fraction) > 0.0
                        ? body.state().attitude.rotate(body.rcs_force_body_n()) / body.vehicle().mass_properties().mass_kg
                        : math::Vec3{};
            }
        }
        if (tower_) tower_->after_tick(pointers, earth_, epoch_ + tick_s, tick_s);
        legs_->after_tick(pointers, env_, epoch_ + tick_s, tick_s);
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
        bool lit = false;
        for (const auto& e : stage.engines) {
            if (e.auxiliary()) continue;  // launch escape or landing thrusters: never lit here
            flying.running_groups[e.group_id] = true;
            lit = true;
        }
        if (lit) break;  // lowest stage with engines only
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
        lower.body->set_ground_height(landing.height_m);
        lower.body->set_accelerometer(true);  // the return guidance estimates the drag online
        lower.return_guidance = std::make_unique<gnc::BoosterReturnGuidance>(
            landing, return_parameters(lower.body->vehicle().stages().front().config));
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
    switch (legs_->touchdown(body)) {  // a landing on legs is judged by the legs
        case LegTouchdown::Landed: return LandingOutcome::Landed;
        case LegTouchdown::TippedOver: return LandingOutcome::TippedOver;
        case LegTouchdown::Impact: return LandingOutcome::Impact;
        case LegTouchdown::Splashdown: return LandingOutcome::Splashdown;
        case LegTouchdown::None: break;
    }
    const auto& site = landing_zone_ && bodies_.at(index).return_guidance ? *landing_zone_ : launch_site_;
    return sim::landing_outcome(body, site, transpose(t.rotation) * body.state().position_gcrf_m);
}

ephemeris::Body FlightSession::reference_body(size_t index) const {
    return bodies_.at(index).body->reference_body(env_, epoch_);
}

vehicle::FlightTelemetry FlightSession::telemetry(size_t index) const {
    return bodies_.at(index).body->telemetry(env_, epoch_);
}

void FlightSession::set_landing_surface(double latitude_rad, double longitude_rad, double height_m,
                                        contact::LandingSurface surface) {
    legs_ = std::make_unique<LegLandingSim>(latitude_rad, longitude_rad, height_m, std::move(surface), tick_s);
}

void FlightSession::set_landing_surface(const LegSite& site, contact::LandingSurface surface) {
    if (site.body == ephemeris::Body::Moon && !moon_) {
        throw std::runtime_error("Landeplatz auf dem Mond braucht die Mondorientierung");
    }
    const auto shape = site.body == ephemeris::Body::Moon ? moon_->shape() : frames::wgs84;
    legs_ = std::make_unique<LegLandingSim>(site, shape, std::move(surface), tick_s);
}

void FlightSession::set_moon_terrain(std::shared_ptr<const environment::LunarTerrain> terrain) {
    if (!moon_) throw std::runtime_error("Mondgelände braucht die Mondorientierung");
    // Bodies resting on the Moon stand at the old surface height: the order is terrain first, then placement.
    for (size_t i = 0; i < bodies_.size(); ++i) {
        if (bodies_[i].body->resting() && reference_body(i) == ephemeris::Body::Moon) {
            throw std::runtime_error("Mondgelände muss vor dem Aufsetzen gesetzt werden");
        }
    }
    moon_terrain_ = std::move(terrain);
    moon_->set_terrain(moon_terrain_.get());
    // A lunar landing site was built on the old terrain (its height, its surface): it must be set again.
    if (legs_->site().body == ephemeris::Body::Moon) {
        legs_ = std::make_unique<LegLandingSim>(0.0, 0.0, 0.0, contact::LandingSurface{}, tick_s);
    }
}

void FlightSession::set_lunar_landing_site(const LunarSite& site) {
    if (!moon_ || !moon_terrain_) throw std::runtime_error("Mondlandeplatz braucht das Mondgelände (set_moon_terrain)");
    const LegSite leg_site{ephemeris::Body::Moon, site.config.latitude_rad, site.config.longitude_rad, site.height_m,
                           site.config.tile_half_size_m};
    const auto frame = LegLandingSim::site_frame(leg_site, moon_->shape());
    contact::LandingSurface surface;
    surface.field =
        std::make_shared<contact::TerrainField>(*moon_, frame.origin_fixed_m, frame.site_to_fixed, moon_terrain_);
    set_landing_surface(leg_site, std::move(surface));
}

void FlightSession::reset_leg_landing() {
    legs_->clear();
}

void FlightSession::set_nose_cone(size_t body_index, bool open) {
    bodies_.at(body_index).body->vehicle().set_nose_cone_open(open);
}

void FlightSession::deploy_legs(size_t body_index) { bodies_.at(body_index).body->vehicle().deploy_legs(); }

void FlightSession::set_leg_deployment(size_t body_index, double fraction) {
    bodies_.at(body_index).body->vehicle().set_leg_deployment(fraction);
}

}  // namespace starsim::sim
