// FlightSession: capsule entry, deorbit planning, parachutes and splashdown.

#include "starsim/sim/flight_session.hpp"

#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::sim {

gnc::EntryTarget FlightSession::capsule_entry_target() const {
    if (!splashdown_zone_) throw std::logic_error("FlightSession: no splashdown zone set");
    gnc::EntryTarget target;
    target.position_itrf_m = frames::geodetic_to_fixed({splashdown_zone_->latitude_rad, splashdown_zone_->longitude_rad, 0.0});
    target.ground_height_m = 0.0;
    return target;
}

void FlightSession::place_entering_capsule(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                                           time::Epoch epoch, gnc::EntryGuidanceParameters parameters) {
    place_body(config, state, epoch);
    start_entry_guidance(bodies_.front(), parameters);
}

const char* to_string(CapsuleReturnStep step) {
    switch (step) {
        case CapsuleReturnStep::None: return "-";
        case CapsuleReturnStep::DeorbitBurn: return "Deorbit-Brennvorgang";
        case CapsuleReturnStep::TrunkJettison: return "Trunk-Abwurf";
        case CapsuleReturnStep::NoseConeClose: return "Bugklappe schließen";
        case CapsuleReturnStep::Coast: return "Freiflug";
        case CapsuleReturnStep::Entry: return "Wiedereintritt";
        case CapsuleReturnStep::Gate: return "Tor";
        case CapsuleReturnStep::Drogues: return "Bremsschirme";
        case CapsuleReturnStep::Mains: return "Hauptschirme";
        case CapsuleReturnStep::SplashedDown: return "gewassert";
        case CapsuleReturnStep::Lost: return "verloren";
    }
    return "?";
}

gnc::DeorbitParameters FlightSession::capsule_deorbit_parameters() {
    gnc::DeorbitParameters p;
    // [E] d3 ledger: an L/D 0.2 capsule under the crew's 4 g has range authority only between ~0 and ~45 deg of bank; at
    // -2.0 deg the range moves ~1000 km per degree of entry angle (-1.2 deg: ~2400 km)
    p.entry_flight_path_rad = -2.0 * constants::deg_to_rad;
    p.nominal_bank_rad = 30.0 * constants::deg_to_rad;
    p.max_crossrange_m = 40000.0;  // [E] the capsule's lateral reach at L/D 0.2 (reversals)
    p.gate_height_m = gnc::EntryGuidanceParameters::capsule().gate_height_m;
    return p;
}

bool FlightSession::plan_capsule_return(gnc::DeorbitParameters parameters) {
    // A return in progress keeps its plan (review d3: a second call reset it mid-way).
    if (capsule_return_step_ != CapsuleReturnStep::None || autopilot_.state == AutopilotState::Return) return false;
    ship_return_ = false;
    capsule_return_step_ = CapsuleReturnStep::None;
    deorbit_plan_.reset();
    gnc::DeorbitPlan refused;
    if (bodies_.empty()) return false;
    auto& flying = bodies_[controlled_];
    const auto& vehicle = flying.body->vehicle();
    const auto& stages = vehicle.stages();
    if (stages.back().config.aero.entry_profile != "capsule" || flying.entry_guidance) {
        refused.reason = "Keine Rückkehr möglich: keine Kapsel im Orbit";
        deorbit_plan_ = refused;
        return false;
    }
    if (vehicle.payload_stacked()) {  // review d3 M2: the capsule must fly alone (released from the second stage)
        refused.reason = "Keine Rückkehr möglich: Kapsel noch auf der Oberstufe";
        deorbit_plan_ = refused;
        return false;
    }
    if (!splashdown_zone_) {
        refused.reason = "Keine Rückkehr möglich: keine Wasserungszone";
        deorbit_plan_ = refused;
        return false;
    }
    const auto rcs = vehicle.rcs_force_capability({1.0, 0.0, 0.0});
    if (!(rcs.force_n > 0.0)) {
        refused.reason = "Keine Rückkehr möglich: keine Deorbit-Düsen";
        deorbit_plan_ = refused;
        return false;
    }
    parameters.burn_thrust_n = rcs.force_n;
    // The capsule after the trunk jettison: its entry model (aero at trim, heat shield, crew load limit).
    vehicle::FlightBody capsule(vehicle.copy_upper(stages.size() - 1), flying.body->state());
    const gnc::EntryGuidance onboard(capsule, capsule_entry_target());
    const double capsule_mass_kg = capsule.vehicle().mass_properties().mass_kg;
    const auto& s = flying.body->state();
    const auto plan = [&](double entry_mass_kg) {
        parameters.entry_mass_kg = entry_mass_kg;
        return gnc::plan_deorbit({s.position_gcrf_m, s.velocity_gcrf_mps}, epoch_, gravity_, earth_, onboard.model(),
                                 capsule_entry_target(), vehicle.mass_properties().mass_kg,
                                 rcs.force_n / rcs.mass_flow_kgps, vehicle.rcs_propellant_kg(),
                                 capsule_return_reserve_kg, parameters);
    };
    deorbit_plan_ = plan(capsule_mass_kg);
    if (deorbit_plan_->feasible) deorbit_plan_ = plan(capsule_mass_kg - deorbit_plan_->propellant_kg);
    deorbit_parameters_ = parameters;
    if (!deorbit_plan_->feasible) return false;
    if (!execute_burn(deorbit_plan_->node) || !burn_config_.rcs) {
        deorbit_plan_->feasible = false;
        deorbit_plan_->reason = "Deorbit-Brennvorgang auf den Düsen nicht möglich";
        autopilot_.state = AutopilotState::Off;
        return false;
    }
    capsule_return_step_ = CapsuleReturnStep::DeorbitBurn;
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
                    capsule->set_ground_height(0.0);
                    flying.body->set_ground_height(0.0);
                    bodies_.push_back({std::move(capsule), {}, nullptr, {}});
                    controlled_ = bodies_.size() - 1;
                    clear_rendezvous_target();  // the target belongs to the controlled body's port
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
    const double altitude = frames::fixed_to_geodetic(transpose(t.rotation) * s.position_gcrf_m).height_m;
    const math::Vec3 air = s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), s.position_gcrf_m);
    const double q = 0.5 * atmosphere_.at(altitude).density_kg_m3 * dot(air, air);
    const double sink = -dot(air, math::normalized(s.position_gcrf_m));
    const auto a = flying.parachutes->update({altitude - body.ground_height_m(), q, sink, body.resting()});
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
    const math::Vec3 z =
        frames::geodetic_to_fixed({splashdown_zone_->latitude_rad, splashdown_zone_->longitude_rad, 0.0});
    const math::Vec3 up = math::normalized(r);
    const math::Vec3 d = r - z;
    return norm(d - up * dot(d, up));
}

}  // namespace starsim::sim
