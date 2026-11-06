// FlightSession: capsule entry, deorbit planning, parachutes and splashdown.

#include "starsim/sim/flight_session.hpp"

#include "starsim/config/mission_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/capsule_return_mission.hpp"

namespace starsim::sim {

gnc::EntryTarget FlightSession::capsule_entry_target() const {
    if (!splashdown_zone_) throw std::logic_error("FlightSession: no splashdown zone set");
    // The zone's water: the EGM2008 geoid there (M8 Plan A Task 5; was the ellipsoid).
    const double sea_m = sea_height_m(splashdown_zone_->latitude_rad, splashdown_zone_->longitude_rad);
    gnc::EntryTarget target;
    target.position_itrf_m =
        frames::geodetic_to_fixed({splashdown_zone_->latitude_rad, splashdown_zone_->longitude_rad, sea_m});
    target.ground_height_m = sea_m;
    return target;
}

void FlightSession::place_entering_capsule(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                                           time::Epoch epoch, gnc::EntryGuidanceParameters parameters) {
    place_body(config, state, epoch);
    start_entry_guidance(bodies_.front(), parameters);
}

const char* id_of(CapsuleReturnStep step) {  // i18n-ns: capsule_return_step
    switch (step) {
        case CapsuleReturnStep::None: return "none";
        case CapsuleReturnStep::DeorbitBurn: return "deorbit_burn";
        case CapsuleReturnStep::TrunkJettison: return "trunk_jettison";
        case CapsuleReturnStep::NoseConeClose: return "nose_cone_close";
        case CapsuleReturnStep::Coast: return "coast";
        case CapsuleReturnStep::Entry: return "entry";
        case CapsuleReturnStep::Gate: return "gate";
        case CapsuleReturnStep::Drogues: return "drogues";
        case CapsuleReturnStep::Mains: return "mains";
        case CapsuleReturnStep::SplashedDown: return "splashed_down";
        case CapsuleReturnStep::Lost: return "lost";
    }
    return "?";
}

bool FlightSession::plan_capsule_return() {
    std::string mission_id;
    if (!bodies_.empty()) {
        for (const auto& stage : bodies_[controlled_].body->vehicle().stages()) {
            if (!stage.config.return_mission.empty()) mission_id = stage.config.return_mission;
        }
    }
    if (mission_id.empty()) {
        deorbit_plan_ = gnc::DeorbitPlan{};
        deorbit_plan_->reason = report::make(report::Code::return_no_return_mission);
        return false;
    }
    return plan_capsule_return(capsule_return_parameters(config::load_capsule_return_mission(config_dir_, mission_id)));
}

gnc::DeorbitPlan FlightSession::capsule_deorbit_plan(gnc::DeorbitParameters& parameters) const {
    gnc::DeorbitPlan refused;
    if (bodies_.empty()) {
        refused.reason = report::make(report::Code::return_no_capsule_in_orbit);
        return refused;
    }
    const auto& flying = bodies_[controlled_];
    const auto& vehicle = flying.body->vehicle();
    const auto& stages = vehicle.stages();
    if (stages.back().config.aero.entry_profile != "capsule" || flying.entry_guidance) {
        refused.reason = report::make(report::Code::return_no_capsule_in_orbit);
        return refused;
    }
    if (vehicle.payload_stacked()) {  // review d3 M2: the capsule must fly alone (released from the second stage)
        refused.reason = report::make(report::Code::return_capsule_on_upper_stage);
        return refused;
    }
    if (!splashdown_zone_) {
        refused.reason = report::make(report::Code::return_no_splashdown_zone);
        return refused;
    }
    const auto rcs = vehicle.rcs_force_capability({1.0, 0.0, 0.0});
    if (!(rcs.force_n > 0.0)) {
        refused.reason = report::make(report::Code::return_no_deorbit_thrusters);
        return refused;
    }
    parameters.burn_thrust_n = rcs.force_n;
    // The capsule after the trunk jettison: its entry model (aero at trim, heat shield, crew load limit).
    vehicle::FlightBody capsule(vehicle.copy_upper(stages.size() - 1), flying.body->state());
    const gnc::EntryGuidance onboard(capsule, capsule_entry_target());
    parameters.reach_check = gnc::entry_reach_check(onboard);  // its rules (no terminal phase) and the crew load limit
    parameters.reach_check.gate_tolerance_m = splashdown_zone_->radius_m;  // the gate over the zone
    parameters.burn_dispersion = gnc::deorbit_burn_dispersion(stages.back().config);  // its cutoff residual, pointing
    const double capsule_mass_kg = capsule.vehicle().mass_properties().mass_kg;
    const auto& s = flying.body->state();
    const auto plan = [&](double entry_mass_kg) {
        parameters.entry_mass_kg = entry_mass_kg;
        return gnc::plan_deorbit({s.position_gcrf_m, s.velocity_gcrf_mps}, epoch_, gravity_, earth_, onboard.model(),
                                 capsule_entry_target(), vehicle.mass_properties().mass_kg,
                                 rcs.force_n / rcs.mass_flow_kgps, vehicle.rcs_propellant_kg(),
                                 capsule_return_reserve_kg, parameters);
    };
    auto result = plan(capsule_mass_kg);
    if (result.feasible) result = plan(capsule_mass_kg - result.propellant_kg);
    return result;
}

gnc::DeorbitPlan FlightSession::capsule_return_preview(gnc::DeorbitParameters parameters) const {
    return capsule_deorbit_plan(parameters);
}

bool FlightSession::plan_capsule_return(gnc::DeorbitParameters parameters) {
    // A return in progress keeps its plan (review d3: a second call reset it mid-way).
    if (capsule_return_step_ != CapsuleReturnStep::None || autopilot_.state == AutopilotState::Return) return false;
    if (docking_busy()) {  // undock first (ISS i5)
        docking_status_ = report::make(report::Code::docking_return_docked);
        return false;
    }
    ship_return_ = false;
    capsule_return_step_ = CapsuleReturnStep::None;
    deorbit_plan_.reset();
    ++deorbit_generation_;  // a background ship return plan is dropped when it arrives
    if (bodies_.empty()) return false;
    deorbit_plan_ = capsule_deorbit_plan(parameters);
    deorbit_parameters_ = parameters;
    if (!deorbit_plan_->feasible) return false;
    if (!execute_burn(deorbit_plan_->node) || !burn_config_.rcs) {
        deorbit_plan_->feasible = false;
        deorbit_plan_->reason = report::make(report::Code::deorbit_burn_on_thrusters_impossible);
        autopilot_.state = AutopilotState::Off;
        return false;
    }
    capsule_return_step_ = CapsuleReturnStep::DeorbitBurn;
    log_dispersion_note(*deorbit_plan_);
    return true;
}

void FlightSession::run_capsule_return(Flying& flying) {
    if (flying.body->impacted() && capsule_return_step_ != CapsuleReturnStep::SplashedDown) {
        capsule_return_step_ = CapsuleReturnStep::Lost;  // d4 minor: the step stayed "Bremsschirme" after an impact
        autopilot_.state = AutopilotState::Off;
        return;
    }
    switch (capsule_return_step_) {
        case CapsuleReturnStep::TrunkJettison:
            if (epoch_ >= capsule_step_epoch_) {
                if (flying.body->vehicle().stages().size() > 1) {
                    auto capsule = flying.body->separate(flying.body->vehicle().stages().size() - 1,
                                                         trunk_separation_dv_mps);
                    record_event(MissionEventKind::StageSeparation, controlled_);  // the trunk
                    // the flat model's water below them: the geoid (run_parachutes follows it under the capsule)
                    capsule->set_ground_height(sea_below_m(capsule->state().position_gcrf_m, epoch_));
                    flying.body->set_ground_height(sea_below_m(flying.body->state().position_gcrf_m, epoch_));
                    bodies_.push_back(new_flying(std::move(capsule)));
                    controlled_ = bodies_.size() - 1;
                    clear_rendezvous_target();  // the target belongs to the controlled body's port
                    reset_rendezvous();
                }
                capsule_return_step_ = CapsuleReturnStep::NoseConeClose;
                capsule_step_epoch_ = epoch_ + nose_cone_close_delay_s;
            }
            break;
        case CapsuleReturnStep::NoseConeClose:
            if (epoch_ >= capsule_step_epoch_) {
                set_nose_cone(controlled_, false);
                capsule_return_step_ = CapsuleReturnStep::Coast;
            }
            break;
        case CapsuleReturnStep::Coast:
            if (epoch_ >= entry_start_) {
                start_entry_guidance(flying, gnc::EntryGuidanceParameters::capsule());
                capsule_return_step_ = CapsuleReturnStep::Entry;
            }
            break;
        case CapsuleReturnStep::Entry:
            if (flying.entry_guidance && flying.entry_guidance->phase() == gnc::EntryPhase::Gate) {
                capsule_return_step_ = CapsuleReturnStep::Gate;
            }
            break;
        case CapsuleReturnStep::Gate:
        case CapsuleReturnStep::Drogues:
        case CapsuleReturnStep::Mains:
            if (flying.parachutes) {
                switch (flying.parachutes->phase()) {
                    case gnc::ParachutePhase::Drogues: capsule_return_step_ = CapsuleReturnStep::Drogues; break;
                    case gnc::ParachutePhase::Mains: capsule_return_step_ = CapsuleReturnStep::Mains; break;
                    case gnc::ParachutePhase::Down:
                        capsule_return_step_ = CapsuleReturnStep::SplashedDown;
                        autopilot_.state = AutopilotState::Off;
                        break;
                    default: break;
                }
            }
            break;
        default:
            break;
    }
}

void FlightSession::run_parachutes(Flying& flying) {
    auto& body = *flying.body;
    const auto t = earth_.fixed_to_gcrf(epoch_);
    const auto& s = body.state();
    const auto geodetic = frames::fixed_to_geodetic(transpose(t.rotation) * s.position_gcrf_m);
    const double altitude = geodetic.height_m;
    // The ground below the capsule (controller ruling 5, M8 Plan A Task 5): the Earth terrain's surface when set (its
    // sea: the geoid), else the geoid there - capsules come down at sea, and the flat model's ground follows it.
    const double lat = geodetic.latitude_rad, lon = geodetic.longitude_rad;
    const double ground_m = earth_terrain_ ? earth_terrain_->height_ellipsoid_m(lat, lon) : sea_height_m(lat, lon);
    if (!earth_terrain_) body.set_ground_height(ground_m);
    const math::Vec3 air = s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), s.position_gcrf_m);
    const double q = 0.5 * atmosphere_.at(altitude).density_kg_m3 * dot(air, air);
    const double sink = -dot(air, math::normalized(s.position_gcrf_m));
    const auto a = flying.parachutes->update({altitude - ground_m, q, sink, body.resting()});
    auto& v = body.vehicle();
    if (a.release_drogues) v.release_parachutes("drogue");
    if (a.deploy_drogues) v.deploy_parachutes("drogue");
    if (a.deploy_mains) v.deploy_parachutes("main");
    if (a.release_mains) v.release_parachutes("main");
}

double FlightSession::splashdown_miss_m(size_t index) const {
    if (!splashdown_zone_) throw std::logic_error("FlightSession: no splashdown zone set");
    const auto t = earth_.fixed_to_gcrf(epoch_);
    const math::Vec3 r = transpose(t.rotation) * bodies_.at(index).body->state().position_gcrf_m;
    const math::Vec3 z = frames::geodetic_to_fixed(
        {splashdown_zone_->latitude_rad, splashdown_zone_->longitude_rad,
         sea_height_m(splashdown_zone_->latitude_rad, splashdown_zone_->longitude_rad)});
    const math::Vec3 up = math::normalized(r);
    const math::Vec3 d = r - z;
    return norm(d - up * dot(d, up));
}

double FlightSession::sea_below_m(const math::Vec3& position_gcrf_m, time::Epoch epoch) const {
    const auto t = earth_.fixed_to_gcrf(epoch);
    const auto g = frames::fixed_to_geodetic(transpose(t.rotation) * position_gcrf_m);
    return sea_height_m(g.latitude_rad, g.longitude_rad);
}

}  // namespace starsim::sim
