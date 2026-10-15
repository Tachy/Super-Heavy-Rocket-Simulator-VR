// FlightSession: ship entry, return and landing; booster return and catch.

#include "starsim/sim/flight_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/frames/geodetic.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"

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
    if (stage.aero.entry_profile == "capsule") {
        flying.entry_guidance = std::make_unique<gnc::EntryGuidance>(*flying.body, capsule_entry_target(), parameters);
        return;
    }
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
    flying.body->set_accelerometer(true);  // the return guidance estimates the drag online
    flying.return_guidance->skip_to(start_phase);
    flying.return_guidance->set_realtime(realtime_planning_);
}

gnc::BoosterReturnParameters FlightSession::return_parameters(const config::StageConfig& stage) const {
    const bool drone_ship = landing_zone_ && landing_zone_->deck;
    return gnc::booster_return_parameters(stage.id, stage.return_profile_for(drone_ship),
                                          booster_return_.value_or(gnc::BoosterReturnParameters{}));
}

void FlightSession::set_landing_zone(const LaunchSite& zone) {
    landing_zone_ = zone;
    contact::LandingSurface ground;
    if (zone.deck) {
        ground.deck = zone.deck;
    } else {
        ground.sea = zone.sea;
    }
    legs_ = std::make_unique<LegLandingSim>(zone.latitude_rad, zone.longitude_rad, zone.height_m, std::move(ground), tick_s);
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

}  // namespace starsim::sim
