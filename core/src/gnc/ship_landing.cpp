#include "starsim/gnc/ship_landing.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/aero/flap.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/mat3.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double catch_missed_height_m = 5.0;  // CG this far below the catch point: the catch is over (as the booster)

// Thrust of the running engines of `groups` at minimum and full throttle, and their exhaust velocity.
struct GroupThrust {
    double min_n = 0.0;
    double max_n = 0.0;
    double exhaust_velocity_mps = 0.0;
};

GroupThrust group_thrust(const vehicle::Vehicle& vehicle, const std::vector<std::string>& groups, double pressure_pa) {
    GroupThrust t;
    double mass_flow_kgps = 0.0;
    for (const auto& e : vehicle.stages().front().engines) {
        if (e.starved || e.failed || std::find(groups.begin(), groups.end(), e.group_id) == groups.end()) continue;
        t.min_n += e.performance->thrust_n(e.performance->config().throttle_min, pressure_pa);
        t.max_n += e.performance->thrust_n(1.0, pressure_pa);
        mass_flow_kgps += e.performance->mass_flow_kgps(1.0);
    }
    t.exhaust_velocity_mps = mass_flow_kgps > 0.0 ? t.max_n / mass_flow_kgps : 0.0;
    return t;
}

Vec3 up_at(double latitude_rad, double longitude_rad) {
    return {std::cos(latitude_rad) * std::cos(longitude_rad), std::cos(latitude_rad) * std::sin(longitude_rad),
            std::sin(latitude_rad)};
}

}  // namespace

EntryTarget ship_entry_target(const LandingSite& site, const ShipLandingParameters& parameters) {
    return ship_entry_target(site, parameters.gate_lead_m);
}

EntryTarget ship_entry_target(const LandingSite& site, double gate_lead_m) {
    const Vec3 pad = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    EntryTarget target{pad, site.height_m, {}};
    if (!site.catch_target) return target;
    const auto& c = *site.catch_target;
    const Vec3 up = up_at(site.latitude_rad, site.longitude_rad);
    Vec3 to_tower = c.tower_center_itrf_m - c.pin_point_itrf_m;
    to_tower = math::normalized(to_tower - up * dot(to_tower, up));
    const Vec3 axis = c.pin_point_itrf_m - up * dot(c.pin_point_itrf_m - pad, up);  // catch axis at the pad height
    target.position_itrf_m = axis - to_tower * gate_lead_m;
    target.nose_direction_itrf = to_tower;
    return target;
}

// Diagnostics (STARSIM_DEBUG_GATE): the ship relative to the catch axis in the site's east/north/up, the axis relative
// to the pad centre, height of the CG above the site, Earth-fixed velocity and the propellant aboard.
void ShipLandingGuidance::print_gate_state(const char* label, const vehicle::FlightBody& body,
                                           const frames::FixedToInertial& transform, time::Epoch epoch) const {
    (void)epoch;
    const Vec3 up = up_at(site_.latitude_rad, site_.longitude_rad);
    const Vec3 east = math::normalized(cross(Vec3{0.0, 0.0, 1.0}, up));
    const Vec3 north = cross(up, east);
    const Vec3 r = transpose(transform.rotation) * body.state().position_gcrf_m;
    const Vec3 v = transpose(transform.rotation) *
                   (body.state().velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(transform), body.state().position_gcrf_m));
    const Vec3 pad = frames::geodetic_to_fixed({site_.latitude_rad, site_.longitude_rad, site_.height_m});
    const Vec3 axis = site_.catch_target ? site_.catch_target->pin_point_itrf_m : pad;
    const Vec3 to_axis = axis - r;
    std::printf("[%s] to-axis E %.1f N %.1f m, height %.1f m, v E %.2f N %.2f U %.2f m/s, axis from pad E %.2f N %.2f m, "
                "propellant %.0f kg (the feed can burn %.0f kg)\n",
                label, dot(to_axis, east), dot(to_axis, north), dot(r - pad, up), dot(v, east), dot(v, north), dot(v, up),
                dot(axis - pad, east), dot(axis - pad, north), body.vehicle().propellant_kg(),
                feed_propellant_kg(body.vehicle()));
}

bool ship_flip_ready(const ShipLandingParameters& parameters, const vehicle::FlightBody& body, const Vec3& up_gcrf) {
    const Vec3 axis = body.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
    const double tilt = std::acos(std::clamp(dot(axis, up_gcrf), -1.0, 1.0));
    const Vec3 w = body.state().angular_velocity_body_radps;
    // the plans' rate cone
    return tilt < parameters.flip_tolerance_rad && std::hypot(w.y, w.z) < parameters.flip_settled_rate_radps;
}

ShipLandingGuidance::ShipLandingGuidance(LandingSite site, ShipLandingParameters parameters)
    : site_(std::move(site)), parameters_(std::move(parameters)) {}

void ShipLandingGuidance::set_realtime(bool realtime) {
    parameters_.scvx.realtime = realtime;
    if (scvx_) scvx_->set_realtime(realtime);
    if (pending_) pending_->set_realtime(realtime);
}

ReturnCommand ShipLandingGuidance::update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                          time::Epoch epoch) {
    ReturnCommand cmd;
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const Vec3 site_up = up_at(site_.latitude_rad, site_.longitude_rad);
    const Vec3 up_gcrf = transform.rotation * site_up;
    if (!started_) {
        started_ = true;
        next_attempt_ = epoch;
        flip_start_ = epoch;
        if (parameters_.switch_window_from_gate) {
            // The 3 -> 2 window's maximum from the height left at the gate: the CG above the catch (or the ground).
            const auto& vehicle = body.vehicle();
            const Vec3 pad = frames::geodetic_to_fixed({site_.latitude_rad, site_.longitude_rad, site_.height_m});
            const Vec3 r_fixed = transpose(transform.rotation) * body.state().position_gcrf_m;
            double height = dot(r_fixed - pad, site_up);
            if (site_.catch_target) {
                const double cg = vehicle.mass_properties().cg_station_m;
                height -= dot(site_.catch_target->pin_point_itrf_m - pad, site_up) - (site_.catch_target->pin_station_m - cg);
            }
            parameters_.scvx.settings.limits.switches[0].max_sink_mps = ship_switch_max_sink_mps(
                vehicle.source_config(), vehicle.stages().front().config, parameters_, height);
        }
        if (diagnostics_ && std::getenv("STARSIM_DEBUG_GATE")) print_gate_state("gate", body, transform, epoch);
    }
    if (phase_ == ReturnPhase::Flip) {
        cmd.run_groups = parameters_.flip_groups;
        cmd.throttle = parameters_.flip_throttle;
        cmd.axis_gcrf = up_gcrf;  // engines down
        if (site_.catch_target) {
            // pins across the arms already in the flip, while three engines and the flaps can roll the ship (two
            // off-axis engines in the burn have little roll authority)
            const Vec3 body_y = transpose(transform.rotation) * body.state().attitude.rotate(Vec3{0.0, 1.0, 0.0});
            cmd.roll_reference_gcrf = transform.rotation * catch_roll_reference(*site_.catch_target, site_up, body_y);
        }
        const Vec3 axis = body.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
        const double tilt = std::acos(std::clamp(dot(axis, up_gcrf), -1.0, 1.0));
        const Vec3 w = body.state().angular_velocity_body_radps;
        if (diagnostics_ && std::getenv("STARSIM_DEBUG_FLIP")) {  // diagnostics
            double spool = 0.0, thrust = 0.0;
            for (const auto& e : body.vehicle().stages().front().engines) {
                if (e.producing_thrust()) thrust += e.spool * e.performance->thrust_n(e.throttle, 0.0);
                spool = std::max(spool, e.spool);
            }
            std::printf("[flip] t %.2f tilt %.1f rate %.3f spool %.2f thrust %.2f MN flaps", epoch - flip_start_,
                        tilt * constants::rad_to_deg, std::hypot(w.y, w.z), spool, thrust / 1e6);
            for (const auto& f : body.vehicle().stages().front().flaps) {
                std::printf(" %.0f", f.deflection_rad * constants::rad_to_deg);
            }
            std::printf(" w %.3f/%.3f\n", w.y, w.z);
        }
        if (tilt > parameters_.flip_flaps_until_tilt_rad) {
            const auto& stage = body.vehicle().stages().front();
            const double cg = body.vehicle().mass_properties().cg_station_m;
            for (const auto& f : stage.flaps) {
                const bool forward = aero::flap_centroid(f.geometry, f.deflection_rad).x > cg;
                cmd.flap_rad.push_back(forward ? f.max_deflection_rad : 0.0);
            }
        }
        if (tilt < parameters_.flip_tolerance_rad && flip_duration_s_ == 0.0) {
            flip_duration_s_ = epoch - flip_start_;
            const Vec3 r_fixed = transpose(transform.rotation) * body.state().position_gcrf_m;
            flip_end_height_m_ = frames::fixed_to_geodetic(r_fixed).height_m - site_.height_m;
            if (diagnostics_ && std::getenv("STARSIM_DEBUG_GATE")) print_gate_state("flip-end", body, transform, epoch);
        }
        if (ship_flip_ready(parameters_, body, up_gcrf) && epoch >= next_attempt_ && site_.catch_target &&
            !upright_descent_ && !pending_) {
            ++first_plan_attempts_;
            pending_ = std::make_unique<ScvxLandingGuidance>(site_, parameters_.scvx);
            if (abort_requested_) pending_->request_abort();
            attempt_epoch_ = epoch;
            // Real-time play (Task 11b): planned on the worker from the state the flip command above leads to over the
            // hold (its thrust along the axis), taking effect at the hand-over; else at once.
            FirstPlanHold hold;
            if (parameters_.scvx.realtime) hold.duration_s = first_plan_hold_s(FirstPlanKind::Ship);
            hold.throttle_down = true;  // the flip engines to minimum throttle meanwhile (below)
            pending_->begin(body, env, epoch, parameters_.scvx.realtime, hold);
        }
        if (pending_) {
            if (const auto started = pending_->poll(epoch)) {
                if (parameters_.scvx.realtime) record_first_plan_wall_s(FirstPlanKind::Ship, pending_->first_plan_wall_s());
                if (*started) {
                    scvx_ = std::move(pending_);
                    phase_ = ReturnPhase::LandingBurn;
                } else {
                    pending_.reset();
                    next_attempt_ = attempt_epoch_ + parameters_.first_plan_retry_s;
                    // neither the catch nor the sea in reach: holding the flip thrust would hover the headers dry
                    if (first_plan_attempts_ >= parameters_.first_plans_before_descent) upright_descent_ = true;
                }
            }
        }
        if (pending_ && pending_->first_plan_pending()) {
            // Real-time play: the hold the plan starts from - the flip engines down to their minimum throttle (3 engines
            // at the flip's 0.8 brake ~45 m/s^2: held for the plan's ~2 s the sink would turn into a climb).
            double minimum = 0.0;
            for (const auto& e : body.vehicle().stages().front().engines) {
                if (std::find(cmd.run_groups.begin(), cmd.run_groups.end(), e.group_id) != cmd.run_groups.end()) {
                    minimum = std::max(minimum, e.performance->config().throttle_min);
                }
            }
            cmd.throttle = minimum;
        }
    }
    if (phase_ == ReturnPhase::LandingBurn && !upright_descent_) {
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
        // [E] flaps folded against the hull in the burn: the plans' model has no flaps
        cmd.flap_rad.assign(body.vehicle().stages().front().flaps.size(), 0.0);
        const auto mass = body.vehicle().mass_properties();
        const Vec3 r_fixed = transpose(transform.rotation) * body.state().position_gcrf_m;
        const Vec3 r_catch = site_.catch_target->pin_point_itrf_m -
                             site_up * (site_.catch_target->pin_station_m - mass.cg_station_m);
        const bool catch_over = !c.abort && (catch_contact_ || dot(r_fixed - r_catch, site_up) < -catch_missed_height_m);
        if (catch_over) phase_ = ReturnPhase::Landed;
        // An abort without a converged abort plan after the grace time, or one that leans the hull beyond the guard:
        // the upright descent instead (sticky).
        if (c.abort) {
            if (!abort_since_) abort_since_ = epoch;
            const Vec3 axis = body.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
            const double tilt = std::acos(std::clamp(dot(axis, up_gcrf), -1.0, 1.0));
            const bool no_plan = !scvx_->has_abort_plan() && epoch - *abort_since_ > parameters_.abort_plan_grace_s;
            // Propellant left only for a vertical stop: upright and slow before the distance to the sea (an abort plan
            // flying for the water ran the headers dry and hit it at 9.5 m/s).
            const Vec3 v_fixed = transpose(transform.rotation) *
                                 (body.state().velocity_gcrf_mps -
                                  cross(vehicle::earth_rotation_vector(transform), body.state().position_gcrf_m));
            const double sink = -dot(v_fixed, site_up);
            const double pressure = env.atmosphere.at(site_.height_m).pressure_pa;
            const Vec3 pad = frames::geodetic_to_fixed({site_.latitude_rad, site_.longitude_rad, site_.height_m});
            const double engine_height = dot(r_fixed - pad, site_up) - mass.cg_station_m;
            const auto thrust = group_thrust(
                body.vehicle(), descent_groups(descent_pair_reached(body.vehicle(), sink, engine_height, pressure)), pressure);
            const double usable = feed_propellant_kg(body.vehicle()) - parameters_.scvx.snapshot.reserve_kg;
            const double need = upright_stop_propellant_kg(mass.mass_kg, sink, thrust.max_n / mass.mass_kg,
                                                           constants::standard_gravity_mps2, thrust.exhaust_velocity_mps,
                                                           parameters_.upright_descent);
            const bool short_of_propellant = usable < parameters_.descent_propellant_margin * need;
            if (no_plan || short_of_propellant || tilt > parameters_.upright_guard_tilt_rad) {
                upright_descent_ = true;
            }
        }
    }
    if (upright_descent_ && phase_ != ReturnPhase::Landed) fly_upright_descent(body, env, transform, site_up, cmd);
    if ((body.resting() || body.impacted()) && !contact_) {
        contact_ = true;
        const Vec3 axis = body.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
        contact_tilt_rad_ = std::acos(std::clamp(dot(axis, up_gcrf), -1.0, 1.0));
        const Vec3 v_fixed = body.state().velocity_gcrf_mps -
                             cross(vehicle::earth_rotation_vector(transform), body.state().position_gcrf_m);
        contact_sink_mps_ = -dot(v_fixed, up_gcrf);
    }
    if (body.resting() || body.impacted()) phase_ = ReturnPhase::Landed;
    if (phase_ == ReturnPhase::Landed) {
        cmd.run_groups.clear();
        cmd.axis_gcrf = up_gcrf;
    }
    cmd.phase = phase_;
    return cmd;
}

bool ShipLandingGuidance::descent_pair_reached(const vehicle::Vehicle& vehicle, double sink_mps, double height_m,
                                               double pressure_pa) const {
    const auto pair = group_thrust(vehicle, parameters_.scvx.snapshot.phase_groups[1], pressure_pa);
    const double mass_kg = vehicle.mass_properties().mass_kg;
    // the pair at its planned maximum (the plans' tracking margin below full) still stops the sink within the height:
    // the descent profile with that whole excess; the margin is left to close in on its own profile
    auto full = parameters_.upright_descent;
    full.deceleration_fraction = 1.0;
    const double planned = pair.max_n * (1.0 - parameters_.scvx.settings.limits.throttle_margin) / mass_kg;
    return sink_mps <= upright_descent_sink_mps(height_m, planned, constants::standard_gravity_mps2, full);
}

// The last-resort abort (gnc::upright_descent): the flip set brakes towards the landing pair's sink profile until the
// pair can stop the sink, then the pair follows the profile (sticky); thrust limits of the running engines at the local pressure; arms open, flaps folded,
// free roll.
void ShipLandingGuidance::fly_upright_descent(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                              const frames::FixedToInertial& transform, const Vec3& site_up,
                                              ReturnCommand& cmd) {
    const auto& vehicle = body.vehicle();
    const auto mass = vehicle.mass_properties();
    const Vec3 r_fixed = transpose(transform.rotation) * body.state().position_gcrf_m;
    const Vec3 v_fixed = transpose(transform.rotation) *
                         (body.state().velocity_gcrf_mps -
                          cross(vehicle::earth_rotation_vector(transform), body.state().position_gcrf_m));
    const Vec3 pad = frames::geodetic_to_fixed({site_.latitude_rad, site_.longitude_rad, site_.height_m});
    const double cg_height = dot(r_fixed - pad, site_up);
    const double pressure = env.atmosphere.at(site_.height_m + cg_height).pressure_pa;
    const double sink = -dot(v_fixed, site_up);
    const double engine_height = cg_height - mass.cg_station_m;
    if (!descent_on_pair_) descent_on_pair_ = descent_pair_reached(vehicle, sink, engine_height, pressure);
    const auto& groups = descent_groups(descent_on_pair_);
    const auto thrust = group_thrust(vehicle, groups, pressure);
    const auto pair = group_thrust(vehicle, parameters_.scvx.snapshot.phase_groups[1], pressure);
    const Vec3 tower = site_.catch_target ? site_.catch_target->tower_center_itrf_m : pad;
    const auto c = upright_descent(r_fixed - tower, v_fixed, site_up, engine_height, thrust.min_n / mass.mass_kg,
                                   thrust.max_n / mass.mass_kg, pair.max_n / mass.mass_kg,
                                   constants::standard_gravity_mps2, parameters_.upright_descent);
    cmd.run_groups = groups;
    cmd.throttle = landing_throttle(vehicle, groups, mass.mass_kg * c.acceleration_mps2, pressure);
    if (diagnostics_ && std::getenv("STARSIM_DEBUG_FLIP")) {  // diagnostics
        std::printf("[upright] h %.1f sink %.1f groups %zu accel %.2f (%.2f-%.2f) throttle %.2f propellant %.0f kg\n",
                    cg_height - mass.cg_station_m, sink, groups.size(), c.acceleration_mps2, thrust.min_n / mass.mass_kg,
                    thrust.max_n / mass.mass_kg, cmd.throttle, vehicle.propellant_kg());
    }
    cmd.axis_gcrf = transform.rotation * c.axis;
    cmd.roll_reference_gcrf = {};
    cmd.scvx = false;
    cmd.abort = true;
    cmd.feedforward.reset();
    cmd.torque_body_nm.reset();
    cmd.flap_rad.assign(vehicle.stages().front().flaps.size(), 0.0);
}

bool ShipLandingGuidance::request_abort() {
    if (!site_.sea || phase_ == ReturnPhase::Landed || catch_contact_) return false;
    if (scvx_) return abort_requested_ = scvx_->request_abort();
    abort_requested_ = true;  // still flipping: the first plan goes to the sea
    if (pending_) pending_->request_abort();  // a first plan under way: the abort follows it (its re-plans)
    return true;
}

}  // namespace starsim::gnc
