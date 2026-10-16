// FlightSession: lunar descent from the LLO, powered descent and the surface steps (lunar l4).
//
// Design decisions:
// - plan_lunar_landing plans once from the LLO (mission::LunarDescentPlanner) and checks the propellant for descent and
//   ascent (mission::lunar_budget: braking and ascent on the vacuum Raptors - user 2026-10-15 -, DOI and the rest of the
//   descent on the approach set; Isp from the vehicle's engine groups).
// - The plane trim and the DOI are flown as planned (closed-loop cutoff on the delta-v, lunar_small_burn: the single
//   sea-level Raptor, else the session's choice); the PDI uses the predicted epoch and the braking law absorbs the dispersion. A DOI re-plan after the
//   trim is a later refinement.
// - Waits for a burn (DescentWait, days) are Sun-pointed coasts like l3's Coast. After the DOI the vehicle coasts
//   freely (half a revolution) and turns to the braking attitude (retrograde, horizontal, LVLH hold)
//   descent_attitude_lead_s before PDI; the braking guidance's first update lights the vacuum Raptors their start-up
//   time before the PDI epoch.
// - The descent is a step of the lunar step machine (state Lunar): PoweredDescent flies gnc::LunarLandingGuidance's
//   commands as the ship's landing burn (apply_return_command with the ship-landing controller: gimbals + RCS), ticked.
// - The touchdown record is taken at the first leg contact (sink and lateral speed relative to the Moon); the legs'
//   judgement ends the step: "gelandet" -> Landed (via SurfaceStay when a stay is requested), "umgekippt" / "Aufprall"
//   (or an impact without legs) -> Lost (autopilot off).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>

#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_budget.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "flight_session_lunar_detail.hpp"

namespace starsim::sim {

using detail::far_future_s;
using detail::german;

namespace {

// Longest start-up of the engines in `groups`.
double group_startup_s(const vehicle::Vehicle& vehicle, const std::vector<std::string>& groups) {
    double startup_s = 0.0;
    for (const auto& stage : vehicle.stages()) {
        for (const auto& e : stage.engines) {
            if (std::find(groups.begin(), groups.end(), e.group_id) != groups.end()) {
                startup_s = std::max(startup_s, e.performance->config().startup_s);
            }
        }
    }
    return startup_s;
}

}  // namespace

bool FlightSession::plan_lunar_landing(const LunarLandingSetup& setup) {
    if (bodies_.empty()) return false;
    const bool idle = (lunar_step_ == LunarStep::None &&
                       (autopilot_.state == AutopilotState::Off || autopilot_.state == AutopilotState::Orbit)) ||
                      (lunar_step_ == LunarStep::InLlo && autopilot_.state == AutopilotState::Lunar);
    if (!idle) return false;  // a flight or a descent under way keeps its plan
    // A closed orbit about the Moon whose periselene clears the highest terrain.
    bool in_orbit = false;
    double aposelene_m = 0.0;
    const double gm_moon = env_.ephemeris->gm_m3ps2(ephemeris::Body::Moon);
    if (moon_ && moon_orientation_ && reference_body(controlled_) == ephemeris::Body::Moon) {
        const auto e = orbit::elements_from_state(mission::moon_relative(*env_.ephemeris, controlled_state(), epoch_),
                                                  gm_moon);
        in_orbit = e.eccentricity < 1.0 && e.semi_major_axis_m * (1.0 - e.eccentricity) >
                                               constants::moon_mean_radius_m + constants::moon_max_terrain_height_m;
        aposelene_m = e.semi_major_axis_m * (1.0 + e.eccentricity);
    }
    if (!in_orbit) {
        lunar_status_ = "Kein Abstieg: nicht in einer Mondumlaufbahn";
        return false;
    }
    if (!moon_terrain_ || legs_->site().body != ephemeris::Body::Moon) {
        lunar_status_ = "Kein Abstieg: keine Landestelle gesetzt";
        return false;
    }
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    const auto& legs = vehicle.stages().front().config.landing_legs;
    const auto& sets = setup.guidance.braking_sets;
    const double vacuum_isp_s = sets.empty() ? 0.0 : gnc::groups_vacuum_isp_s(vehicle, sets.front());
    const double approach_isp_s = sets.empty() ? 0.0 : gnc::groups_vacuum_isp_s(vehicle, sets.back());
    if (legs.azimuths_rad.empty() || vacuum_isp_s <= 0.0 || approach_isp_s <= 0.0) {
        lunar_status_ = "Kein Abstieg: Fahrzeug ohne Landebeine oder Abstiegstriebwerke";
        return false;
    }
    const double mass_kg = vehicle.mass_properties().mass_kg;
    const auto budget = mission::lunar_budget(
        mass_kg, mass_kg - vehicle.propellant_kg(), setup.reserve_kg,
        {{"Bremsphase", setup.braking_delta_v_mps, vacuum_isp_s},
         {"Anflug", setup.descent_delta_v_mps - setup.braking_delta_v_mps, approach_isp_s},
         {"Aufstieg", setup.ascent_delta_v_mps, vacuum_isp_s}});
    if (!budget.closes) {
        lunar_status_ = "Kein Abstieg: " + budget.reason;
        return false;
    }
    const auto& site = legs_->site();
    auto descent = setup.descent;
    if (setup.braking_range) {
        // PDI speed: periselene of the descent orbit (the LLO's aposelene, PDI height above the site).
        const double rp = constants::moon_mean_radius_m + moon_terrain_->height_m(site.latitude_rad, site.longitude_rad) +
                          descent.pdi_altitude_above_site_m;
        const double pdi_speed = std::sqrt(gm_moon * (2.0 / rp - 2.0 / (rp + std::max(aposelene_m, rp))));
        descent.braking_range_m = mission::braking_range_m(
            pdi_speed, mass_kg, gnc::set_max_thrust_n(vehicle, sets.front()), *setup.braking_range);
    }
    const mission::LunarDescentPlanner planner(gravity_, *env_.ephemeris, *moon_orientation_);
    mission::LunarDescentPlan plan;
    try {
        plan = planner.plan(controlled_state(), epoch_, *moon_terrain_, site.latitude_rad, site.longitude_rad,
                            legs.footprint_radius_m, descent);
    } catch (const std::exception&) {  // integrator step underflow on a trial arc, ephemeris range
        lunar_status_ = "Kein Abstieg: Planung fehlgeschlagen";
        return false;
    }
    if (!plan.feasible) {
        lunar_status_ = "Kein Abstieg: " + plan.reason;
        return false;
    }
    lunar_descent_plan_ = plan;
    lunar_landing_setup_ = setup;
    lunar_landing_setup_->descent = descent;
    lunar_descent_trimmed_ = false;
    lunar_landing_.reset();
    lunar_touchdown_.reset();
    lunar_pilot_.reset();
    autopilot_.state = AutopilotState::Lunar;
    lunar_descent_wait();
    if (lunar_debug_) {
        std::printf("[lunar] descent plan: pass %.3f d, trim %.2f m/s (crossrange %.0f -> %.1f m), DOI %.2f m/s, "
                    "periselene %.2f km, PDI in %.3f d, range %.1f km, budget margin %.1f t\n",
                    (plan.site_pass_epoch - epoch_) / 86400.0,
                    plan.plane_trim ? norm(plan.plane_trim->delta_v_gcrf_mps) : 0.0, plan.crossrange_before_trim_m,
                    plan.crossrange_m, norm(plan.doi.delta_v_gcrf_mps), plan.periselene_altitude_m / 1000.0,
                    (plan.pdi_epoch - epoch_) / 86400.0, plan.pdi_range_m / 1000.0, budget.margin_kg / 1000.0);
    }
    return true;
}

void FlightSession::lunar_descent_wait() {
    const auto& plan = *lunar_descent_plan_;
    const bool trim = plan.plane_trim && !lunar_descent_trimmed_;
    const auto& node = trim ? *plan.plane_trim : plan.doi;
    lunar_step_ = LunarStep::DescentWait;
    attitude_mode_ = {gnc::AttitudeModeKind::Sun, {}, {}, {}};
    lunar_sun_hold_.start(epoch_);
    lunar_next_repoint_ = epoch_ + sun_repoint_interval_s;
    // Ticks resume for the burn's preparation (as l3's trims: planned at least that far ahead)
    lunar_step_epoch_ = lunar_ignition_lead(node) - small_burn_margin_s;
    lunar_status_ = std::string("Warten auf Überflug: ") + (trim ? "Bahnebenenkorrektur" : "DOI") + " in " +
                    german((node.epoch - epoch_) / 3600.0, 1) + " h";
}

bool FlightSession::lunar_descent_burn(const gnc::BurnNode& node, LunarStep step) {
    if (!lunar_small_burn(node)) {
        lunar_abort(std::string(step == LunarStep::Circularize ? "Aufstieg" : "Abstieg") +
                    " abgebrochen: Brennvorgang nicht möglich");
        return false;
    }
    lunar_burns_.push_back({step, node, bodies_[controlled_].body->vehicle().mass_properties().mass_kg});
    if (lunar_debug_) {
        std::printf("[lunar] %s: %.2f m/s on %s (%.0f kN)\n", to_string(step), norm(node.delta_v_gcrf_mps),
                    burn_config_.rcs ? "RCS" : burn_config_.groups.front().c_str(), burn_config_.thrust_n / 1000.0);
    }
    return true;
}

bool FlightSession::lunar_small_burn(const gnc::BurnNode& node) {
    // A 10-20 m/s trim or DOI fits no Raptor set (their start/shutdown impulse is more than a quarter of the burn), and
    // at ~570 t (the LLO after a full-tank outbound flight) the session's choice is the cold-gas RCS (Isp ~66 s: ~15 t
    // per burn instead of ~2.5 t). The approach set's gimballed Raptor alone (as the SCvx approach) flies it at minimum
    // throttle while its transients are at most small_burn_transient_share of the burn; tiny burns (station keeping)
    // stay on the session's choice. Without a landing setup: the HLS's sets.
    const auto set = lunar_landing_setup_ ? lunar_landing_setup_->guidance.small_burn_set
                                          : gnc::hls_landing_parameters().small_burn_set;
    if (!set.empty()) {
        std::vector<std::string> groups;  // burn_configurations takes the group ids without the stage prefix
        for (const auto& id : set) groups.push_back(id.substr(id.find('/') + 1));
        if (start_burn(node, burn_configurations(&groups), false, small_burn_transient_share)) return true;
    }
    return execute_burn(node);
}

bool FlightSession::begin_powered_descent(const mission::TouchdownPoint& point, double approach_azimuth_rad,
                                          gnc::LunarLandingParameters parameters) {
    if (!moon_) {
        lunar_status_ = "Landeanflug nicht möglich: keine Mondorientierung";
        return false;
    }
    if (legs_->site().body != ephemeris::Body::Moon) {
        lunar_status_ = "Landeanflug nicht möglich: keine Landestelle auf dem Mond (set_lunar_landing_site)";
        return false;
    }
    if (bodies_.empty() || reference_body(controlled_) != ephemeris::Body::Moon) {
        lunar_status_ = "Landeanflug nicht möglich: Fahrzeug nicht am Mond";
        return false;
    }
    auto& vehicle = bodies_[controlled_].body->vehicle();
    const auto& legs = vehicle.stages().front().config.landing_legs;
    if (legs.azimuths_rad.empty()) {
        lunar_status_ = "Landeanflug nicht möglich: keine Landebeine";
        return false;
    }
    if (!legs.fixed && vehicle.leg_deployment() == 0.0) vehicle.deploy_legs();
    gnc::LandingSite site{ephemeris::Body::Moon, point.latitude_rad, point.longitude_rad, point.height_m, {}, {},
                          gnc::PadTarget{-legs.foot_station_m, 0.0}};  // feet -foot_station_m below the stage bottom
    lunar_landing_ = std::make_unique<gnc::LunarLandingGuidance>(site, approach_azimuth_rad, std::move(parameters));
    lunar_landing_->set_realtime(realtime_planning_);
    lunar_touchdown_.reset();
    lunar_touchdown_point_ = point;
    lunar_auto_abort_tried_ = false;
    attitude_mode_ = {};
    autopilot_.state = AutopilotState::Lunar;
    lunar_step_ = LunarStep::PoweredDescent;
    lunar_step_epoch_ = epoch_;
    lunar_status_ = "Landeanflug";
    return true;
}

void FlightSession::run_lunar_descent_steps(Flying& flying) {
    switch (lunar_step_) {
        case LunarStep::DescentWait: {
            lunar_sun_coast(flying);
            if (epoch_ - lunar_step_epoch_ < 0.0) break;
            const auto& plan = *lunar_descent_plan_;
            const bool trim = plan.plane_trim && !lunar_descent_trimmed_;
            const auto& node = trim ? *plan.plane_trim : plan.doi;
            attitude_mode_ = {};  // the burn executor turns to the burn attitude in its preparation
            if (lunar_descent_burn(node, trim ? LunarStep::PlaneTrim : LunarStep::Doi)) {
                lunar_step_ = trim ? LunarStep::PlaneTrim : LunarStep::Doi;
                lunar_status_ = std::string(trim ? "Bahnebenenkorrektur: " : "Abstiegsbahn (DOI): ") +
                                german(norm(node.delta_v_gcrf_mps), 1) + " m/s";
            }
            break;
        }
        case LunarStep::PlaneTrim:  // the trim burn is over
            lunar_descent_trimmed_ = true;
            lunar_descent_wait();
            break;
        case LunarStep::Doi: {  // the DOI burn is over: half a revolution of free coast to PDI
            const auto& plan = *lunar_descent_plan_;
            lunar_step_ = LunarStep::DescentCoast;
            attitude_mode_ = {};
            lunar_step_epoch_ = plan.pdi_epoch - descent_attitude_lead_s;
            lunar_status_ = "Freiflug zum Abstieg: PDI in " + german((plan.pdi_epoch - epoch_) / 60.0, 0) + " min";
            break;
        }
        case LunarStep::DescentCoast: {
            if (epoch_ - lunar_step_epoch_ < 0.0) break;
            const auto& plan = *lunar_descent_plan_;
            const auto& setup = *lunar_landing_setup_;
            if (attitude_mode_.kind != gnc::AttitudeModeKind::Lvlh) {
                // Braking attitude: nose (+x) against the velocity, +y up (LVLH: +x along the velocity, +z to nadir).
                attitude_mode_ = gnc::lvlh_hold(math::Quat::from_matrix(
                    math::Mat3::from_columns({-1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, -1.0, 0.0})));
                const auto& sets = setup.guidance.braking_sets;
                const double startup_s = sets.empty() ? 0.0 : group_startup_s(flying.body->vehicle(), sets.front());
                lunar_step_epoch_ = plan.pdi_epoch - startup_s;
                lunar_status_ = "Freiflug zum Abstieg: Bremslage, PDI in " +
                                german((plan.pdi_epoch - epoch_) / 60.0, 0) + " min";
                break;
            }
            if (!begin_powered_descent(plan.touchdown, plan.approach_azimuth_rad, setup.guidance)) {
                const std::string status = lunar_status_;
                lunar_abort(status);
            }
            break;
        }
        case LunarStep::SurfaceStay:
            if (epoch_ - lunar_step_epoch_ < 0.0) break;
            if (lunar_ascent_parameters_) {  // plan_lunar_ascent with a delay: liftoff now
                lunar_liftoff(flying);
                break;
            }
            if (lunar_landing_setup_ && lunar_landing_setup_->ascent && plan_lunar_ascent(*lunar_landing_setup_->ascent)) {
                break;
            }
            lunar_step_ = LunarStep::Landed;
            lunar_step_epoch_ = epoch_ + far_future_s;
            // a refused ascent keeps its reason
            if (!lunar_landing_setup_ || !lunar_landing_setup_->ascent) {
                lunar_status_ = "gelandet: Oberflächenaufenthalt beendet";
            } else if (lunar_mission_) {  // review l4 M3: the sequence ends with the refusal
                lunar_mission_leg_ = LunarMissionLeg::Ended;
                lunar_status_ = "Mission beendet: " + lunar_status_;
            }
            break;
        case LunarStep::PoweredDescent: {
            auto& body = *flying.body;
            const auto frame = moon_->frame(epoch_);
            const auto& s = body.state();
            const bool contact = legs_->in_contact(body);
            const auto record_contact = [&] {
                const math::Vec3 rel = frame.relative_velocity(s.position_gcrf_m, s.velocity_gcrf_mps);
                const math::Vec3 up = math::normalized(s.position_gcrf_m - frame.centre_gcrf_m);
                lunar_touchdown_ = LunarTouchdown{LegTouchdown::None, 0.0, -dot(rel, up), norm(rel - up * dot(rel, up))};
            };
            if (contact && !lunar_touchdown_) record_contact();
            flying.return_command = lunar_landing_->update(body, env_, epoch_, contact);
            // Spec: a landing out of reach (the first approach plan failed, or SCvx gives up after repeated unreachable
            // re-plans - on the Moon its abort plans only set the vehicle down near the point) aborts to orbit. Refused
            // (propellant): the landing guidance keeps flying.
            const bool unreachable = lunar_landing_->phase() == gnc::LunarDescentPhase::Aborted ||
                                     (lunar_landing_->approach() && lunar_landing_->approach()->aborting());
            if (unreachable && !lunar_auto_abort_tried_) {
                lunar_auto_abort_tried_ = true;
                if (lunar_abort_to_orbit()) {
                    run_lunar_ascent_steps(flying);  // this tick's command from the ascent guidance
                    break;
                }
                lunar_status_ = "Landepunkt nicht erreichbar, Abbruch nicht möglich";
                // Final review l4 I2: without an approach plan the guidance would cut the engines - set down with the
                // terminal law instead (towards the touchdown point, the sink rate from the height).
                if (lunar_landing_->land_without_approach()) {
                    lunar_status_ += ": Endanflug ohne Plan";
                    flying.return_command = lunar_landing_->update(body, env_, epoch_, contact);
                }
            }
            for (const auto& stage : body.vehicle().stages()) {
                for (const auto& e : stage.engines) {
                    const auto& groups = flying.return_command.run_groups;
                    flying.running_groups[e.group_id] = std::find(groups.begin(), groups.end(), e.group_id) != groups.end();
                }
            }
            if (!lunar_landing_->status().empty() && lunar_landing_->status() != lunar_status_ &&
                lunar_landing_->phase() == gnc::LunarDescentPhase::Aborted && !lunar_auto_abort_tried_) {
                lunar_status_ = "Landeanflug: " + lunar_landing_->status();
            }
            const auto judged = legs_->touchdown(body);
            if (judged == LegTouchdown::None && !body.impacted()) break;
            if (!lunar_touchdown_) record_contact();  // an impact before any leg contact
            lunar_touchdown_->outcome = body.impacted() ? LegTouchdown::Impact : judged;
            // Where it stands: the footprint centre (the hull axis at the feet's station - on a slope the 21 m high CG
            // leans ~0.9 m downhill) against the touchdown point, both in the leg site's east / north / up frame.
            const double r_point = constants::moon_mean_radius_m + lunar_touchdown_point_.height_m;
            const double lat = lunar_touchdown_point_.latitude_rad, lon = lunar_touchdown_point_.longitude_rad;
            const math::Vec3 point_fixed{r_point * std::cos(lat) * std::cos(lon), r_point * std::cos(lat) * std::sin(lon),
                                         r_point * std::sin(lat)};
            const auto& vehicle = body.vehicle();
            const double below_cg_m =
                vehicle.mass_properties().cg_station_m - vehicle.stages().front().config.landing_legs.foot_station_m;
            const math::Vec3 footprint = s.position_gcrf_m - s.attitude.rotate(math::Vec3{1.0, 0.0, 0.0}) * below_cg_m;
            const math::Vec3 d = legs_->to_site(frame.to_fixed(footprint)) - legs_->to_site(point_fixed);
            lunar_touchdown_->offset_m = std::hypot(d.x, d.y);
            if (lunar_touchdown_->outcome == LegTouchdown::Landed) {
                lunar_landing_->mark_landed();
                lunar_landed_epoch_ = epoch_;
                const double stay_s = lunar_landing_setup_ ? lunar_landing_setup_->surface_stay_s : 0.0;
                // A requested ascent lifts off at the stay's end, also after a stay of 0 h (final review l4 I1).
                const bool stays = stay_s > 0.0 || (lunar_landing_setup_ && lunar_landing_setup_->ascent);
                lunar_step_ = stays ? LunarStep::SurfaceStay : LunarStep::Landed;
                lunar_step_epoch_ = stays ? lunar_landed_epoch_ + stay_s : epoch_ + far_future_s;
                lunar_status_ = "gelandet: " + german(lunar_touchdown_->offset_m, 2) + " m vom Zielpunkt";
            } else {
                lunar_step_ = LunarStep::Lost;
                lunar_status_ = std::string("verloren: ") + to_string(lunar_touchdown_->outcome);
                autopilot_.state = AutopilotState::Off;
            }
            for (auto& [group, run] : flying.running_groups) run = false;
            break;
        }
        case LunarStep::Landed:
        case LunarStep::Lost:
        default:
            break;
    }
}

}  // namespace starsim::sim
