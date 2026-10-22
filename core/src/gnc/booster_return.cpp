#include "starsim/gnc/booster_return.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/lambert.hpp"

#include "booster_return_detail.hpp"

namespace starsim::gnc {

using math::Mat3;
using math::Vec3;
using namespace booster_return_detail;

namespace {

// Hinge station of the grid fins (vehicle frame), or a negative value without fins.
double grid_fin_station(const vehicle::Vehicle& v) {
    for (const auto& stage : v.stages()) {
        if (!stage.grid_fins.empty()) return stage.grid_fins.front().geometry.station_m;
    }
    return -1.0;
}

// Lateral acceleration (perpendicular to the flight path) of a booster falling engines first at the angle
// of attack alpha, with the grid fins holding that attitude (they cancel the aerodynamic moment about the
// CG). Positive: the force points away from the side the top leans to.
double lateral_acceleration(const vehicle::FlightBody& body, double speed, const environment::AtmosphereState& air,
                            double alpha, double fin_station) {
    const auto mass = body.vehicle().mass_properties();
    const Vec3 v_body{-speed * std::cos(alpha), speed * std::sin(alpha), 0.0};  // top leans towards +y
    const auto aero = body.vehicle().aero().compute(v_body, air.density_kg_m3, air.speed_of_sound_mps, false,
                                                     air.mean_free_path_m);
    const Vec3 moment = cross(Vec3{aero.normal_force_station_m - mass.cg_station_m, 0.0, 0.0}, aero.force_body_n);
    const double fin_force_y = -moment.z / (fin_station - mass.cg_station_m);
    const Vec3 total = aero.force_body_n + Vec3{0.0, fin_force_y, 0.0};
    const Vec3 u = v_body / speed;
    const Vec3 perpendicular = total - u * dot(total, u);
    return -perpendicular.y / mass.mass_kg;
}

}  // namespace

const char* to_string(ReturnPhase phase) {
    switch (phase) {
        case ReturnPhase::Flip: return "Wende";
        case ReturnPhase::Boostback: return "Boostback";
        case ReturnPhase::Coast: return "Abstieg";
        case ReturnPhase::EntryBurn: return "Wiedereintritt";
        case ReturnPhase::LandingBurn: return "Landebrennen";
        case ReturnPhase::Landed: return "Gelandet";
    }
    return "";
}

BoosterReturnParameters booster_return_parameters(const std::string& stage_id, const config::ReturnProfile& profile,
                                                  BoosterReturnParameters p) {
    p.boostback = profile.boostback;
    const auto ids = [&](const std::vector<std::string>& local) {
        std::vector<std::string> out;
        for (const auto& id : local) out.push_back(stage_id + "/" + id);
        return out;
    };
    if (!profile.boostback_engine_groups.empty()) p.boostback_groups = ids(profile.boostback_engine_groups);
    if (!profile.entry_engine_groups.empty()) p.entry_groups = ids(profile.entry_engine_groups);
    if (profile.landing_engine_phases.size() == 3) {
        p.landing_ignition_groups = ids(profile.landing_engine_phases[0]);
        p.landing_divert_groups = ids(profile.landing_engine_phases[1]);
        p.landing_groups = ids(profile.landing_engine_phases[2]);
    }
    if (profile.landing_phase_throttles.size() == 3) {
        p.scvx.ignition_throttle = profile.landing_phase_throttles[0];
        p.scvx.divert_throttle = profile.landing_phase_throttles[1];
        p.scvx.final_throttle = profile.landing_phase_throttles[2];
    }
    if (profile.landing_phase_max_tilt_rad.size() == 3) {
        for (size_t i = 0; i < 3; ++i) p.scvx.settings.limits.tilt_max_rad[i] = profile.landing_phase_max_tilt_rad[i];
    }
    if (profile.landing_switch_sink_mps.size() == 2) {
        auto& switches = p.scvx.settings.limits.switches;
        for (size_t i = 0; i < 2; ++i) {
            const auto [low, high] = profile.landing_switch_sink_mps[i];
            switches[i] = {0.0, 0.0, low, high};
        }
        // The burn prediction switches in the middle of each window.
        p.scvx.divert_speed_mps = 0.5 * (switches[0].min_sink_mps + switches[0].max_sink_mps);
        p.scvx.switch_speed_mps = 0.5 * (switches[1].min_sink_mps + switches[1].max_sink_mps);
    }
    if (profile.return_flight_time_s > 0.0) p.flight_time_s = profile.return_flight_time_s;
    if (profile.boostback_loss_fraction >= 0.0) p.boostback_loss_fraction = profile.boostback_loss_fraction;
    if (profile.rcs_propellant_kg >= 0.0) p.rcs_propellant_kg = profile.rcs_propellant_kg;
    if (profile.entry_burn_altitude_m > 0.0) {
        p.entry_altitude_m = profile.entry_burn_altitude_m;
        p.entry_delta_v_mps = profile.entry_burn_delta_v_mps;
        if (profile.entry_burn_throttle > 0.0) p.entry_throttle = profile.entry_burn_throttle;
    }
    return p;
}

BoosterReturnParameters booster_return_parameters(const config::StageConfig& stage, BoosterReturnParameters base) {
    return booster_return_parameters(stage.id, stage.return_profile, std::move(base));
}

BoosterReturnGuidance::BoosterReturnGuidance(LandingSite site, BoosterReturnParameters parameters)
    : site_(std::move(site)), parameters_(std::move(parameters)) {
    if (site_.catch_target && site_.sea && !parameters_.scvx_landing) {
        // The explicit laws cannot hit the arms (Super Heavy has no legs): the return aims at the sea.
        retarget_to_sea();
    } else if (site_.catch_target) {
        const auto pins = frames::fixed_to_geodetic(site_.catch_target->pin_point_itrf_m);
        target_bottom_height_m_ = pins.height_m - site_.height_m - site_.catch_target->pin_station_m;
    } else if (site_.pad) {
        target_bottom_height_m_ = site_.pad->contact_bottom_height_m;  // on the legs
    }
    if (!parameters_.boostback) phase_ = ReturnPhase::Coast;  // drone ship: coast from separation
}

void BoosterReturnGuidance::retarget_to_sea() {
    ditch_ = true;
    target_bottom_height_m_ = 0.0;  // the hull's bottom at sea level (flat at the site height)
    const Vec3 up = up_at(site_.latitude_rad, site_.longitude_rad);
    const Vec3 east = math::normalized(cross(Vec3{0.0, 0.0, 1.0}, up));
    const Vec3 north = cross(up, east);
    const Vec3 seaward = north * std::cos(site_.sea->direction_rad) + east * std::sin(site_.sea->direction_rad);
    const auto sea = frames::fixed_to_geodetic(
        frames::geodetic_to_fixed({site_.latitude_rad, site_.longitude_rad, site_.height_m}) +
        seaward * (site_.sea->shoreline_distance_m + parameters_.scvx.snapshot.sea_margin_m));
    site_.latitude_rad = sea.latitude_rad;
    site_.longitude_rad = sea.longitude_rad;
}

ReturnCommand BoosterReturnGuidance::update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                            time::Epoch epoch) {
    const auto& s = body.state();
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const Mat3 to_fixed = transpose(transform.rotation);
    const Vec3 omega = vehicle::earth_rotation_vector(transform);
    const Vec3 r_fixed = to_fixed * s.position_gcrf_m;
    const Vec3 v_rel_gcrf = s.velocity_gcrf_mps - cross(omega, s.position_gcrf_m);
    const Vec3 v_fixed = to_fixed * v_rel_gcrf;
    const auto mass = body.vehicle().mass_properties();
    const Vec3 site_up = up_at(site_.latitude_rad, site_.longitude_rad);
    const Vec3 site_fixed = frames::geodetic_to_fixed({site_.latitude_rad, site_.longitude_rad, site_.height_m});
    const auto geo = frames::fixed_to_geodetic(r_fixed);
    const double pressure = env.atmosphere.at(geo.height_m).pressure_pa;
    const Vec3 body_axis = s.attitude.rotate(Vec3{1.0, 0.0, 0.0});
    // Leg landing: the ballistic impact lies beyond the pad by the landing burn's lead (it stops short of it).
    auto horizontal_miss = [&](const Impact& impact) {
        const Vec3 miss = site_fixed + landing_burn_lead_itrf(body, env, impact) - impact.position_itrf_m;
        return miss - site_up * dot(miss, site_up);
    };

    if (!started_) {
        started_ = true;
        boostback_start_ = epoch;
        last_update_ = epoch;
    }
    const double dt = epoch - last_update_;
    last_update_ = epoch;
    ++tick_counter_;
    ReturnCommand cmd;
    // the forecasts of the phases left behind are void
    if (phase_ != ReturnPhase::Boostback) forecast_.boostback_cutoff.reset();
    if (phase_ != ReturnPhase::Coast) forecast_.landing_ignition.reset();
    if (phase_ != ReturnPhase::Coast && phase_ != ReturnPhase::LandingBurn) forecast_.touchdown.reset();
    forecast_.catch_target = site_.catch_target.has_value() && !abort_requested_ && !ditch_;

    if (phase_ == ReturnPhase::Flip || phase_ == ReturnPhase::Boostback) {
        const double time_left = std::max(boostback_start_ + parameters_.flight_time_s - epoch, 30.0);
        const auto at_landing = env.earth_orientation.fixed_to_gcrf(epoch + time_left);
        const Vec3 aim_fixed = site_fixed + aim_offset_itrf_ + site_up * mass.cg_station_m;
        const Vec3 aim_gcrf = at_landing.position(aim_fixed);
        const Vec3 normal = math::normalized(cross(s.position_gcrf_m, aim_gcrf));
        const auto arc = orbit::solve_lambert(s.position_gcrf_m, aim_gcrf, time_left, constants::earth_gm_m3ps2, normal);
        if (arc && tick_counter_ % 20 == 1) {
            // Shooting on the aim point: where would the Lambert arc really come down (drag, rotation)?
            const auto impact = predict(body, env, epoch, &arc->departure_velocity_mps);
            if (impact.valid) {
                const Vec3 miss = horizontal_miss(impact);
                aim_offset_itrf_ += miss;
                impact_miss_m_ = norm(miss);
            }
        }
        const Vec3 v_go = arc ? arc->departure_velocity_mps - s.velocity_gcrf_mps : -v_rel_gcrf;
        cmd.axis_gcrf = math::normalized(v_go);
        cmd.impact_miss_m = impact_miss_m_;

        const double pointing = std::acos(std::clamp(dot(body_axis, cmd.axis_gcrf), -1.0, 1.0));
        if (phase_ == ReturnPhase::Flip) {
            // Turn with the centre engines idling (gimbal authority), then go to full boostback thrust.
            cmd.run_groups = parameters_.landing_groups;
            cmd.throttle = 0.0;  // limited to the minimum throttle by the engines
            if (pointing < parameters_.flip_tolerance_rad) phase_ = ReturnPhase::Boostback;
        } else {
            cmd.run_groups = parameters_.boostback_groups;
            cmd.throttle = 1.0;
            // Cutoff when the remaining delta-v equals the shutdown tail-off impulse.
            const auto engines = engine_set(body.vehicle(), parameters_.boostback_groups, pressure);
            const double tail_off_mps = 0.5 * engines.thrust_n / mass.mass_kg * engines.shutdown_s;
            if (engines.thrust_n > 0.0 && engines.mass_flow_kgps > 0.0) {  // the cutoff criterion run forward
                const double to_gain_mps = std::max(norm(v_go) - tail_off_mps, 0.0);
                const double exhaust_mps = engines.thrust_n / engines.mass_flow_kgps;
                forecast_.boostback_cutoff =
                    epoch + mass.mass_kg / engines.mass_flow_kgps * (1.0 - std::exp(-to_gain_mps / exhaust_mps));
            }
            if (norm(v_go) < std::max(tail_off_mps, 0.5)) phase_ = ReturnPhase::Coast;
        }
        cmd.phase = phase_;
        if (phase_ == ReturnPhase::Coast) cmd.run_groups.clear();
        return cmd;
    }

    if (phase_ == ReturnPhase::Coast) {
        if (tick_counter_ % 40 == 0) estimate_drag_scale(body, env, epoch);
        // Steering: in the atmosphere with the grid fins (they hold an angle of attack, the body's cross-flow
        // force moves the impact point), above it by RCS translation.
        const auto air_now = env.atmosphere.at(geo.height_m);
        const double speed = norm(v_fixed);
        const double dynamic_pressure = 0.5 * air_now.density_kg_m3 * speed * speed;
        const double fin_station = grid_fin_station(body.vehicle());
        const bool aero_steering = fin_station > 0.0 && dynamic_pressure > aero_steering_min_q_pa;
        if (tick_counter_ % (aero_steering ? 40 : 200) == 0) {
            const auto impact = predict(body, env, epoch);
            impact_epoch_.reset();
            if (impact.valid) {
                impact_epoch_ = epoch + impact.time_s;
                const Vec3 miss = horizontal_miss(impact);
                impact_miss_m_ = norm(miss);
                trim_itrf_ = {};
                steer_itrf_ = {};
                if (aero_steering) {
                    // Constant lateral acceleration that removes the miss within a fraction of the time to
                    // impact (margin for the prediction model, which ignores the steering force itself); the
                    // angle of attack comes from the aerodynamic model by bisection. Its sign too: for the blunt tail-first
                    // booster the tilted axial drag outweighs the cross-flow force, so the force points to
                    // the side the top leans to.
                    const double t_correct = aero_steering_time_fraction * impact.time_s;
                    const Vec3 required = miss * (2.0 / std::max(t_correct * t_correct, 1.0));
                    auto lateral = [&](double alpha) {
                        return lateral_acceleration(body, speed, air_now, alpha, fin_station);
                    };
                    const double at_max = lateral(aero_steering_max_alpha_rad);
                    if (at_max != 0.0 && impact_miss_m_ > trim_deadband_m) {
                        double low = 0.0, high = aero_steering_max_alpha_rad;
                        if (std::abs(at_max) > norm(required)) {
                            for (int i = 0; i < 20; ++i) {
                                const double mid = 0.5 * (low + high);
                                (std::abs(lateral(mid)) < norm(required) ? low : high) = mid;
                            }
                        }
                        // Lean towards -d if the force points away from the lean, towards d otherwise.
                        const double lean = at_max > 0.0 ? -1.0 : 1.0;
                        steer_itrf_ = math::normalized(required) * (-lean * std::tan(high));
                    }
                } else if (impact_miss_m_ > trim_deadband_m) {
                    if (auto burn = pending_entry_burn(body.vehicle());
                        burn && parameters_.entry_max_altitude_change_m > 0.0) {
                        // Entry burn ahead: shoot its ignition altitude on the miss along the track (secant on a probe
                        // that ignites higher by a fixed amount).
                        const Vec3 track = math::normalized_or_zero(v_fixed - site_up * dot(v_fixed, site_up));
                        const double along_m = dot(miss, track);
                        burn->altitude_m += entry_altitude_probe_m;
                        const auto probe = predict_impact(body, env, epoch, site_.height_m, nullptr, &*burn, drag_scale_);
                        if (probe.valid) {
                            const double moved_m = dot(probe.position_itrf_m - impact.position_itrf_m, track);
                            if (std::abs(moved_m) > 1.0) {
                                const double limit = parameters_.entry_max_altitude_change_m;
                                entry_altitude_change_m_ = std::clamp(
                                    entry_altitude_change_m_ + along_m * entry_altitude_probe_m / moved_m, -limit, limit);
                            }
                        }
                    }
                    // Proportional trim: full RCS force from 100 m miss.
                    trim_itrf_ = miss * (std::min(impact_miss_m_ / 100.0, 1.0) / impact_miss_m_);
                }
            }
        }
        // Engines first; for a lateral push towards d the top leans towards -d.
        cmd.axis_gcrf = transform.rotation * math::normalized(-math::normalized(v_fixed) - steer_itrf_);
        cmd.impact_miss_m = impact_miss_m_;
        cmd.rcs_force_gcrf = transform.rotation * trim_itrf_;
        if (site_.catch_target) {
            // Roll the pins towards the arms already on the way down (grid fins, above the air RCS): recorded
            // boosters light the landing burn already rolled for the catch.
            const Vec3 body_y = to_fixed * s.attitude.rotate(Vec3{0.0, 1.0, 0.0});
            cmd.roll_reference_gcrf = transform.rotation * catch_roll_reference(site_.catch_target->across_itrf, site_up,
                                                                                site_.catch_target->pin_azimuth_rad, body_y);
        }
        const double height = geo.height_m - mass.cg_station_m - site_.height_m;
        const bool descending = dot(v_fixed, site_up) < 0.0;
        if (descending && !entry_done_ && parameters_.entry_altitude_m > 0.0 &&
            geo.height_m - site_.height_m < parameters_.entry_altitude_m + entry_altitude_change_m_) {
            phase_ = ReturnPhase::EntryBurn;
            entry_delta_v_mps_ = 0.0;
            entry_delta_v_change_mps_ = 0.0;
            entry_thrust_epoch_ = epoch;
            // The coast's steering (fin lean) does not carry over: the burn predicts its own miss at once.
            steer_itrf_ = {};
            impact_miss_m_ = 0.0;
            entry_predicted_ = false;
            forecast_.landing_ignition.reset();
            forecast_.touchdown.reset();
            impact_epoch_.reset();
        }
        if (phase_ == ReturnPhase::Coast && descending && (height < 30e3 || tick_counter_ % 20 == 0)) {
            const auto burn = predict_landing_burn(body, env, epoch);
            if (burn.stop_height_m <= 0.0) phase_ = ReturnPhase::LandingBurn;
            if (phase_ == ReturnPhase::LandingBurn) {
                // lit now: its own time to go takes over below
            } else if (pending_entry_burn(body.vehicle())) {  // the burn prediction does not know the entry burn ahead
                forecast_.landing_ignition.reset();
                forecast_.touchdown.reset();
            } else {
                forecast_landing(burn, height - target_bottom_height_m_, -dot(v_fixed, site_up));
            }
        }
        cmd.phase = phase_;
        if (phase_ == ReturnPhase::Coast) return cmd;
    }

    if (phase_ == ReturnPhase::EntryBurn) {
        if (body.vehicle().any_engine_producing_thrust()) {
            double thrust_n = 0.0;
            for (const auto& e : body.vehicle().stages().front().engines) {
                if (e.producing_thrust()) thrust_n += e.spool * e.performance->thrust_n(e.throttle, pressure);
            }
            entry_delta_v_mps_ += thrust_n / mass.mass_kg * dt;
            entry_thrust_epoch_ = epoch;
        }
        // Nothing burning for longer than an engine start (failed or starved engines): the burn is over, the coast and
        // its landing-burn check take over.
        const bool dead = epoch - entry_thrust_epoch_ > entry_no_thrust_s;
        if (entry_delta_v_mps_ >= parameters_.entry_delta_v_mps + entry_delta_v_change_mps_ || dead) {
            entry_done_ = true;
            steer_itrf_ = {};  // the burn's lean direction is not a fin lean
            phase_ = ReturnPhase::Coast;
            cmd.phase = phase_;
            cmd.axis_gcrf = transform.rotation * (-math::normalized(v_fixed));
            return cmd;
        }
        entry_burn(body, env, epoch, cmd);
        cmd.phase = phase_;
        return cmd;
    }

    if (phase_ == ReturnPhase::LandingBurn) {
        if (!landing_started_) start_landing(body, env, epoch);
        if (scvx_) {
            const auto c = scvx_->update(body, env, epoch);
            cmd.run_groups = c.run_groups;
            cmd.throttle = c.throttle;
            cmd.axis_gcrf = c.axis_gcrf;
            cmd.roll_reference_gcrf = c.roll_reference_gcrf;
            cmd.feedforward = c.feedforward;
            cmd.torque_body_nm = c.torque_body_nm;
            cmd.scvx = true;
            cmd.abort = c.abort;
            cmd.time_to_go_s = c.time_to_go_s;
            cmd.impact_miss_m = c.offset_m;
            bool over = false;
            if (site_.catch_target) {
                const Vec3 r_catch = site_.catch_target->pin_point_itrf_m -
                                     site_up * (site_.catch_target->pin_station_m - mass.cg_station_m);
                const double height = dot(r_fixed - r_catch, site_up);
                over = !c.abort && (catch_contact_ || height < -catch_missed_height_m);
            } else {
                // Pad: the legs go out before the touchdown; the engines stop on leg contact, or shortly after the plan's
                // end (a plan ending a little high: the stage drops onto its legs).
                const double deploy_s = body.vehicle().stages().front().config.landing_legs.deploy_time_s;
                if (c.time_to_go_s <= deploy_s + leg_deploy_margin_s) legs_deployed_ = true;
                if (c.time_to_go_s > 0.0 || !plan_end_) plan_end_ = epoch + c.time_to_go_s;
                over = leg_contact_ || epoch - *plan_end_ > pad_cutoff_after_plan_s;
            }
            if (body.resting() || body.impacted() || over) phase_ = ReturnPhase::Landed;
        } else {
            explicit_landing(body, env, epoch, cmd);
        }
        if (cmd.abort) forecast_.catch_target = false;
        if (phase_ == ReturnPhase::LandingBurn) forecast_.touchdown = epoch + cmd.time_to_go_s;
    }
    if (phase_ == ReturnPhase::Landed) {
        forecast_.touchdown.reset();
        cmd.run_groups.clear();
        cmd.axis_gcrf = transform.rotation * site_up;
    }
    cmd.deploy_legs = legs_deployed_;
    cmd.phase = phase_;
    return cmd;
}

void BoosterReturnGuidance::forecast_landing(const BurnPrediction& burn, double height_m, double sink_mps) {
    // The ignition from the guidance's two predictions: the ballistic impact less the fall, at the sink speed now, over
    // the height the landing burn takes (its start height above its stop height). At the ignition that height is the
    // height left. (The stop height alone run forward linearly was 26 s early in the fall's middle: the
    // stop height falls fast while the stage is fast and high; this stays within ~5 s, see the timeline test.)
    if (!impact_epoch_ || !(sink_mps > 0.0)) {
        forecast_.landing_ignition.reset();
        forecast_.touchdown.reset();
        return;
    }
    forecast_.landing_ignition = *impact_epoch_ - std::max(height_m - burn.stop_height_m, 0.0) / sink_mps;
    forecast_.touchdown = *forecast_.landing_ignition + burn.duration_s;
}

void BoosterReturnGuidance::set_realtime(bool realtime) {
    parameters_.scvx.realtime = realtime;
    if (scvx_) scvx_->set_realtime(realtime);
}

void BoosterReturnGuidance::start_landing(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                          time::Epoch epoch) {
    landing_started_ = true;
    if (!parameters_.scvx_landing || !(site_.catch_target || site_.pad)) return;
    auto p = parameters_.scvx;
    p.snapshot.phase_groups = {parameters_.landing_ignition_groups, parameters_.landing_divert_groups,
                               parameters_.landing_groups};
    // A pad landing ends on one engine whose minimum thrust exceeds the weight: no exponential approach (infeasible at
    // the end, see DescentLimits::approach_time_constant_s).
    if (site_.pad) p.settings.limits.approach_time_constant_s = 0.0;
    scvx_ = std::make_unique<ScvxLandingGuidance>(site_, p);
    if (abort_requested_) scvx_->request_abort();
    if (!scvx_->start(body, env, epoch)) {
        scvx_.reset();  // fallback: the explicit laws
        if (abort_requested_ && !ditch_) retarget_to_sea();
    }
}

bool BoosterReturnGuidance::request_abort() {
    if (!site_.sea || phase_ == ReturnPhase::Landed || catch_contact_) return false;
    if (scvx_) return abort_requested_ = scvx_->request_abort();
    abort_requested_ = true;
    if (landing_started_ && !ditch_) retarget_to_sea();  // explicit laws in charge: they must not aim at the arms
    return true;
}

}  // namespace starsim::gnc
