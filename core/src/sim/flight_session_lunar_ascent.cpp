// FlightSession: lunar ascent from the surface and abort to orbit from a powered descent (lunar l4).
//
// Design decisions:
// - plan_lunar_ascent needs the controlled vehicle standing landed (lunar step Landed / SurfaceStay, or judged
//   "gelandet" by the leg simulation) and checks the propellant (mission::lunar_budget on the ascent set's vacuum Isp;
//   the landing setup's ascent delta-v and reserve when one exists). A liftoff later than now waits in SurfaceStay.
// - Liftoff releases the landed body (FlightBody::release_landed: integrated again from its held pose, which moves with
//   the Moon - no jump) and its leg track (the judgement starts over; the legs carry the weight until the thrust does).
// - Ascent / AbortAscent fly gnc::LunarAscentGuidance's commands as the ship's landing burn (ReturnCommand: run groups,
//   throttle, axis; the ship-landing controller), ticked. At cutoff: AscentCoast to the aposelene (free coast, < 1 h:
//   no Sun hold), then circularize_at_apoapsis re-planned from the state at the burn's preparation, flown like the
//   descent's small burns (lunar_descent_burn: the session's choice, else the single sea-level Raptor), then InLlo.
// - Abort to orbit: LunarAscentGuidance::abort_delta_v_mps against the rocket equation on the ascent set's vacuum Isp,
//   with the landing setup's reserve; refused -> nothing changes (the landing continues).

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_budget.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

namespace {

constexpr double far_future_s = 1e9;  // "nothing timed"

std::string german(double value, int decimals) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.*f", decimals, value);
    std::string s = text;
    std::replace(s.begin(), s.end(), '.', ',');
    return s;
}

}  // namespace

bool FlightSession::plan_lunar_ascent(const gnc::LunarAscentParameters& parameters, double liftoff_delay_s) {
    if (bodies_.empty() || !moon_) {
        lunar_status_ = "Kein Aufstieg: keine Mondorientierung";
        return false;
    }
    auto& flying = bodies_[controlled_];
    const bool landed_step = lunar_step_ == LunarStep::Landed || lunar_step_ == LunarStep::SurfaceStay;
    const bool judged = legs_->touchdown(*flying.body) == LegTouchdown::Landed;
    if (!flying.body->resting() || flying.body->impacted() || (!landed_step && !judged) ||
        reference_body(controlled_) != ephemeris::Body::Moon) {
        lunar_status_ = "Kein Aufstieg: nicht gelandet";
        return false;
    }
    const auto& vehicle = flying.body->vehicle();
    const double isp_s = gnc::groups_vacuum_isp_s(vehicle, parameters.ascent_groups);
    if (isp_s <= 0.0) {
        lunar_status_ = "Kein Aufstieg: keine Aufstiegstriebwerke";
        return false;
    }
    const double delta_v_mps =
        lunar_landing_setup_ ? lunar_landing_setup_->ascent_delta_v_mps : default_ascent_delta_v_mps;
    const double reserve_kg = lunar_landing_setup_ ? lunar_landing_setup_->reserve_kg : default_ascent_reserve_kg;
    const double mass_kg = vehicle.mass_properties().mass_kg;
    const auto budget = mission::lunar_budget(mass_kg, mass_kg - vehicle.propellant_kg(), reserve_kg,
                                              {{"Aufstieg", delta_v_mps, isp_s}});
    if (!budget.closes) {
        lunar_status_ = "Kein Aufstieg: " + budget.reason;
        return false;
    }
    lunar_ascent_parameters_ = parameters;
    autopilot_.state = AutopilotState::Lunar;
    attitude_mode_ = {};
    if (liftoff_delay_s > 0.0) {
        lunar_step_ = LunarStep::SurfaceStay;
        lunar_step_epoch_ = epoch_ + liftoff_delay_s;
        lunar_status_ = "Oberflächenaufenthalt: Start in " + german(liftoff_delay_s / 3600.0, 1) + " h";
        return true;
    }
    lunar_liftoff(flying);
    return true;
}

void FlightSession::lunar_liftoff(Flying& flying) {
    auto& body = *flying.body;
    const auto frame = moon_->frame(epoch_);
    // Terrain height at the liftoff point: the feet below the CG along the hull axis.
    const auto& vehicle = body.vehicle();
    const auto& s = body.state();
    const double below_cg_m =
        vehicle.mass_properties().cg_station_m - vehicle.stages().front().config.landing_legs.foot_station_m;
    const math::Vec3 feet = s.position_gcrf_m - s.attitude.rotate(math::Vec3{1.0, 0.0, 0.0}) * below_cg_m;
    const double site_height_m = norm(feet - frame.centre_gcrf_m) - constants::moon_mean_radius_m;
    const auto parameters = lunar_ascent_parameters_ ? *lunar_ascent_parameters_ : gnc::LunarAscentParameters{};
    lunar_ascent_parameters_.reset();
    body.release_landed();
    legs_->release(body);
    lunar_ascent_ = std::make_unique<gnc::LunarAscentGuidance>(
        parameters, env_.ephemeris->gm_m3ps2(ephemeris::Body::Moon), site_height_m);
    lunar_ascent_->start(body, frame, epoch_, false);
    attitude_mode_ = {};
    autopilot_.state = AutopilotState::Lunar;
    lunar_step_ = LunarStep::Ascent;
    lunar_step_epoch_ = epoch_;
    lunar_status_ = "Aufstieg: Abheben";
}

bool FlightSession::lunar_abort_to_orbit() {
    auto& body = *bodies_[controlled_].body;
    const auto& vehicle = body.vehicle();
    const auto frame = moon_->frame(epoch_);
    const auto parameters = lunar_landing_setup_ && lunar_landing_setup_->ascent ? *lunar_landing_setup_->ascent
                                                                                 : gnc::LunarAscentParameters{};
    const double reserve_kg = lunar_landing_setup_ ? lunar_landing_setup_->reserve_kg : default_ascent_reserve_kg;
    auto guidance = std::make_unique<gnc::LunarAscentGuidance>(
        parameters, env_.ephemeris->gm_m3ps2(ephemeris::Body::Moon), lunar_touchdown_point_.height_m);
    const double delta_v_mps = guidance->abort_delta_v_mps(body, frame);
    const double isp_s = gnc::groups_vacuum_isp_s(vehicle, parameters.ascent_groups);
    const double mass_kg = vehicle.mass_properties().mass_kg;
    const double needed_kg =
        isp_s > 0.0 ? mass_kg * (1.0 - std::exp(-delta_v_mps / (isp_s * constants::standard_gravity_mps2))) : mass_kg;
    const double available_kg = vehicle.propellant_kg() - reserve_kg;
    if (needed_kg > available_kg) {
        lunar_status_ = "Abbruch nicht möglich: " + german(needed_kg / 1000.0, 1) + " t Treibstoff nötig, " +
                        german(std::max(0.0, available_kg) / 1000.0, 1) + " t verfügbar";
        return false;
    }
    guidance->start(body, frame, epoch_, true);
    lunar_ascent_ = std::move(guidance);
    lunar_landing_.reset();  // the descent guidance is dropped
    lunar_step_ = LunarStep::AbortAscent;
    lunar_step_epoch_ = epoch_;
    lunar_status_ = "Abbruch zum Orbit: " + german(delta_v_mps, 0) + " m/s nötig";
    return true;
}

void FlightSession::run_lunar_ascent_steps(Flying& flying) {
    auto& body = *flying.body;
    const double gm_moon = env_.ephemeris->gm_m3ps2(ephemeris::Body::Moon);
    const auto moon_relative = [&] {
        const auto& s = body.state();
        return moon_->frame(epoch_).centred({s.position_gcrf_m, s.velocity_gcrf_mps});
    };
    if (body.impacted()) {
        lunar_step_ = LunarStep::Lost;
        lunar_status_ = "verloren: Aufprall";
        autopilot_.state = AutopilotState::Off;
        for (auto& [group, run] : flying.running_groups) run = false;
        return;
    }
    switch (lunar_step_) {
        case LunarStep::Ascent:
        case LunarStep::AbortAscent: {
            const auto c = lunar_ascent_->update(body, moon_->frame(epoch_), epoch_);
            gnc::ReturnCommand command;
            command.phase = gnc::ReturnPhase::LandingBurn;
            command.run_groups = c.run_groups;
            command.throttle = c.throttle;
            command.axis_gcrf = c.axis_gcrf;
            command.time_to_go_s = c.time_to_go_s;
            flying.return_command = command;
            for (const auto& stage : body.vehicle().stages()) {
                for (const auto& e : stage.engines) {
                    flying.running_groups[e.group_id] =
                        std::find(c.run_groups.begin(), c.run_groups.end(), e.group_id) != c.run_groups.end();
                }
            }
            const std::string prefix = lunar_step_ == LunarStep::Ascent ? "Aufstieg: " : "Abbruch zum Orbit: ";
            const std::string status = prefix + (lunar_ascent_->status().empty() ? std::string(gnc::to_string(c.phase))
                                                                                 : lunar_ascent_->status());
            if (lunar_step_epoch_ - epoch_ < 0.0 || c.cutoff) lunar_status_ = status;  // the abort's reason first
            if (!c.cutoff) break;
            // Cutoff: coast to the aposelene, the circularisation re-planned at its preparation.
            const auto node = gnc::circularize_at_apoapsis(moon_relative(), epoch_, gm_moon);
            if (!node) {
                lunar_abort("Aufstieg beendet: keine geschlossene Bahn");
                break;
            }
            const auto e = orbit::elements_from_state(moon_relative(), gm_moon);
            lunar_step_ = LunarStep::AscentCoast;
            lunar_step_epoch_ = lunar_ignition_lead(*node) - small_burn_margin_s;
            lunar_status_ = "Freiflug zur Aposelene: " +
                            german((e.semi_major_axis_m * (1.0 - e.eccentricity) - constants::moon_mean_radius_m) / 1000.0, 1) +
                            " x " +
                            german((e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::moon_mean_radius_m) / 1000.0, 1) +
                            " km";
            if (lunar_debug_) std::printf("[lunar] ascent cutoff: %s\n", lunar_status_.c_str());
            break;
        }
        case LunarStep::AscentCoast: {
            if (epoch_ - lunar_step_epoch_ < 0.0) break;
            const auto node = gnc::circularize_at_apoapsis(moon_relative(), epoch_, gm_moon);
            if (!node) {
                lunar_abort("Aufstieg beendet: keine geschlossene Bahn");
                break;
            }
            // ~18 m/s at 150-200 t: lunar_small_burn (the single sea-level Raptor while its transients fit) - before the
            // session's RCS choice (a ~15-40 min cold-gas burn commanded at the main engines' ignition lead, far off its
            // centre: 93.7 x 101.9 km at 151 t, l4 C4). The landing thrusters are auxiliary engines, never lit by a burn.
            const bool flown = lunar_small_burn(*node);
            if (flown) {
                lunar_burns_.push_back({LunarStep::Circularize, *node, body.vehicle().mass_properties().mass_kg});
                if (lunar_debug_) {
                    std::printf("[lunar] circularisation: %.2f m/s on %s (%.0f kN)\n", norm(node->delta_v_gcrf_mps),
                                burn_config_.rcs ? "RCS" : burn_config_.groups.front().c_str(),
                                burn_config_.thrust_n / 1000.0);
                }
            }
            if (flown || lunar_descent_burn(*node, LunarStep::Circularize)) {
                lunar_step_ = LunarStep::Circularize;
                lunar_status_ = "Zirkularisierung: " + german(norm(node->delta_v_gcrf_mps), 1) + " m/s";
            }
            break;
        }
        case LunarStep::Circularize: {  // the burn is over
            const auto e = orbit::elements_from_state(moon_relative(), gm_moon);
            const double low = e.semi_major_axis_m * (1.0 - e.eccentricity) - constants::moon_mean_radius_m;
            const double high = e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::moon_mean_radius_m;
            lunar_step_ = LunarStep::InLlo;
            lunar_step_epoch_ = epoch_ + far_future_s;
            lunar_status_ = "in LLO: " + german(low / 1000.0, 1) + " x " + german(high / 1000.0, 1) + " km";
            break;
        }
        default:
            break;
    }
}

}  // namespace starsim::sim
