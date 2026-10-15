// Powered descent of a lunar lander (l4): Apollo braking law from PDI to the high gate, SCvx approach on the Raptor,
// terminal descent on the landing thrusters (Apollo P66), engines off on leg contact.
//
// Design decisions:
// - Braking frame: the touchdown point's plan frame (x up, y east, z north) turned about x by the approach azimuth:
//   downrange z = (0, sin az, cos az), crossrange y = z x x (right-handed). Positions and velocities relative to the
//   rotating Moon (plan_state). The effective gravity is evaluated at the vehicle (the braking phase spans ~450 km,
//   ~15 deg of the Moon): gravitation minus the Moon centre's acceleration, minus the Coriolis and centrifugal terms of
//   the Moon's rotation, all in the guidance frame.
// - Thrust: along the law's thrust acceleration, throttle = m |a| / T_max(set) clamped to the set's [minimum, 1]. A
//   smaller set takes over when the demand stays below set_drop_fraction x its maximum for set_drop_s; a disjoint set
//   (vacuum -> sea_level_1) is lit startup_s before the drop the demand's trend predicts, the old set is cut at the
//   drop. The hull turns so that the running set's thrust axis through the CG points along the command
//   (thrust_axis_body). With the fixed vacuum Raptors no gimbal acts: the attitude controller's torque goes to the RCS.
// - Engine-out in a fixed set (user 2026-10-15): the other engines of the set are cut at once (their off-axis thrust is
//   a torque the RCS cannot hold) and engine_out_set continues the law; the later sets follow by the same drop rule.
// - Hand-over at braking t_go <= approach_time_to_go_s (15 s: the law's gains 12/t_go^2, 6/t_go outran the attitude loop
//   of a ~700 t HLS on one Raptor below ~12 s and tumbled it), or when the law has no time to go left: the first SCvx
//   plan is made synchronously; if it fails the phase is Aborted ("Anflug nicht planbar", command abort, engines off).
//   The high gate is low and slow (700 m up, 800 m out, 25 m/s): from Apollo-like gates (2-3 km, 90-108 m/s) SCvx
//   found junk optima (overshooting the point) or none.
// - Terminal descent: once the approach lights its last set (the landing thrusters, planned within ~10 m of the point)
//   a P66-like law flies it (sink rate from the height, offset and lateral velocity nulled; thrust never below the set's
//   minimum). The thrusters are fixed and canted: the attitude controller turns them by differential throttling (their
//   thrust lines miss the CG by ~9 m each; user 2026-10-15) plus the RCS - the single off-axis Raptor before them leaves
//   the hull ~5 deg off the thrust axis at the switch, which the RCS alone needed ~20 s to remove.
// - Touchdown: engines off on the first leg contact; the session marks Landed when the legs judge "gelandet".

#include "starsim/gnc/lunar_landing_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "starsim/math/constants.hpp"

namespace starsim::gnc {

using math::Mat3;
using math::Quat;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double debug_interval_s = 10.0;

bool contains(const std::vector<std::string>& groups, const std::string& id) {
    return std::find(groups.begin(), groups.end(), id) != groups.end();
}

// Axial vacuum thrust of the set's usable engines at full throttle.
double set_max_thrust_n(const vehicle::Vehicle& vehicle, const std::vector<std::string>& groups) {
    double thrust = 0.0;
    for (const auto& e : vehicle.stages().front().engines) {
        if (!e.starved && contains(groups, e.group_id)) thrust += e.performance->thrust_n(1.0, 0.0) * e.axis_body.x;
    }
    return thrust;
}

double set_min_throttle(const vehicle::Vehicle& vehicle, const std::vector<std::string>& groups) {
    double minimum = 0.0;
    for (const auto& e : vehicle.stages().front().engines) {
        if (!e.starved && contains(groups, e.group_id)) minimum = std::max(minimum, e.performance->config().throttle_min);
    }
    return minimum;
}

double set_startup_s(const vehicle::Vehicle& vehicle, const std::vector<std::string>& groups) {
    double startup = 0.0;
    for (const auto& e : vehicle.stages().front().engines) {
        if (!e.starved && contains(groups, e.group_id)) startup = std::max(startup, e.performance->config().startup_s);
    }
    return startup;
}

// Thrust of all engines now (starting, running, running down), vacuum.
double vehicle_thrust_n(const vehicle::Vehicle& vehicle) {
    double thrust = 0.0;
    for (const auto& e : vehicle.stages().front().engines) {
        if (e.producing_thrust()) thrust += e.spool * e.performance->thrust_n(e.throttle, 0.0) * e.axis_body.x;
    }
    return thrust;
}

bool disjoint(const std::vector<std::string>& a, const std::vector<std::string>& b) {
    return std::none_of(a.begin(), a.end(), [&](const std::string& g) { return contains(b, g); });
}

}  // namespace

const char* to_string(LunarDescentPhase phase) {
    switch (phase) {
        case LunarDescentPhase::Braking: return "Bremsphase";
        case LunarDescentPhase::Approach: return "Anflug";
        case LunarDescentPhase::Terminal: return "Endanflug";
        case LunarDescentPhase::Touchdown: return "Aufsetzen";
        case LunarDescentPhase::Landed: return "gelandet";
        case LunarDescentPhase::Aborted: return "Abbruch";
    }
    return "";
}

LunarLandingParameters hls_landing_parameters() {
    LunarLandingParameters p;
    auto& a = p.approach;
    // One sea-level Raptor flies the approach (three at minimum throttle over-brake the gate), the landing thrusters the
    // last ~150 m (lunar l4 Task 3).
    a.snapshot.phase_groups = {std::vector<std::string>{"hls/sea_level_1"}, std::vector<std::string>{"hls/sea_level_1"},
                               std::vector<std::string>{"hls/landing"}};
    a.snapshot.reserve_kg = 5000.0;   // [E]
    a.snapshot.catch_sink_mps = 1.0;  // spec: ~1 m/s at touchdown
    // [E] the landing thrusters steer sideways only by tilting the hull on the RCS: they take over above the point
    a.snapshot.pad_final_set_radius_m = 10.0;
    auto& l = a.settings.limits;
    // [E] 45 deg: braking 90 m/s within 3 km needs ~1.35 m/s^2 horizontally against 1.62 m/s^2 gravity; the landing
    // thrusters are steered by the RCS only.
    l.tilt_max_rad = {45.0 * deg, 45.0 * deg, 3.0 * deg};
    l.rate_max_radps = {0.05, 0.05, 0.01};
    l.switches[0] = {0.0, 0.0, 5.0, 60.0, 0.0};
    l.switches[1] = {2.0 * deg, 0.01, 1.0, 10.0, 150.0};  // landing thrusters from >= 150 m CG height
    l.approach_time_constant_s = 0.0;                     // pad landing
    l.final_gimbal_fraction = 1.0;  // the landing thrusters' equivalent gimbal (RCS): 0.3 of it could not turn them back
    a.settings.kinematic_guess_s = {50.0, 10.0, 30.0};    // [E] ~1 min on the Raptor, short second phase, ~30 s
    a.lqr_lateral = true;
    // [E] the booster's 0.2 rocked the 700 t HLS on its single Raptor (rates up to 0.7 rad/s); 0.05 tracks within 0.07
    a.lateral_effort_fraction = 0.05;
    return p;
}

LunarLandingGuidance::LunarLandingGuidance(LandingSite touchdown, double approach_azimuth_rad,
                                           LunarLandingParameters parameters)
    : site_(std::move(touchdown)),
      parameters_(std::move(parameters)),
      plan_(plan_frame(site_)),
      braking_(parameters_.high_gate, parameters_.braking_time_to_go_guess_s),
      debug_(std::getenv("STARSIM_DEBUG_LUNAR_DESCENT") != nullptr) {
    const Vec3 up{1.0, 0.0, 0.0};
    const Vec3 downrange{0.0, std::sin(approach_azimuth_rad), std::cos(approach_azimuth_rad)};
    const Vec3 crossrange = cross(downrange, up);
    plan_to_guidance_ = transpose(Mat3::from_columns(up, crossrange, downrange));
    if (!parameters_.braking_sets.empty()) set_groups_ = parameters_.braking_sets.front();
}

void LunarLandingGuidance::set_realtime(bool realtime) {
    parameters_.approach.realtime = realtime;
    if (approach_) approach_->set_realtime(realtime);
}

std::vector<std::string> LunarLandingGuidance::braking_groups_now(const vehicle::Vehicle& vehicle, double demand_n,
                                                                  time::Epoch epoch) {
    const double dt = started_ ? epoch - last_update_ : 0.0;
    // Engine-out in a fixed set: cut it, the gimballed engine-out set continues.
    bool failed_fixed = false;
    for (const auto& e : vehicle.stages().front().engines) {
        failed_fixed = failed_fixed || (e.starved && !e.gimbaled && contains(set_groups_, e.group_id));
    }
    if (failed_fixed && set_groups_ != parameters_.engine_out_set) {
        set_groups_ = parameters_.engine_out_set;
        next_lit_ = false;
        below_set_s_ = 0.0;
        status_ = "Triebwerksausfall: Bremsen auf den Meeresspiegel-Raptoren";
        if (debug_) std::printf("[lunar_descent] engine out: braking on the engine-out set\n");
    }
    demand_history_.emplace_back(epoch, demand_n);
    while (demand_history_.size() > 2 && epoch - demand_history_.front().first > parameters_.demand_trend_s) {
        demand_history_.pop_front();
    }
    std::vector<std::string> groups = set_groups_;
    if (braking_set_ + 1 >= parameters_.braking_sets.size()) return groups;
    const auto& next = parameters_.braking_sets[braking_set_ + 1];
    const double threshold = parameters_.set_drop_fraction * set_max_thrust_n(vehicle, next);
    below_set_s_ = demand_n < threshold ? below_set_s_ + dt : 0.0;
    if (disjoint(set_groups_, next) && !next_lit_) {
        // The demand's trend predicts the drop; the next set starts its start-up ahead of it.
        const auto& [t_old, f_old] = demand_history_.front();
        const double span = epoch - t_old;
        const double slope = span > 1.0 ? (demand_n - f_old) / span : 0.0;
        const double to_drop = demand_n <= threshold ? 0.0
                               : slope < 0.0         ? (demand_n - threshold) / -slope
                                                     : std::numeric_limits<double>::infinity();
        if (to_drop + parameters_.set_drop_s <= set_startup_s(vehicle, next)) {
            next_lit_ = true;
            if (debug_) std::printf("[lunar_descent] lighting the next set ahead of the drop (%.1f s)\n", to_drop);
        }
    }
    if (below_set_s_ >= parameters_.set_drop_s) {
        ++braking_set_;
        set_groups_ = next;
        next_lit_ = false;
        below_set_s_ = 0.0;
        if (debug_) {
            std::printf("[lunar_descent] set drop to %s (demand %.2f MN)\n", set_groups_.front().c_str(), demand_n / 1e6);
        }
        return set_groups_;
    }
    if (next_lit_) groups.insert(groups.end(), next.begin(), next.end());
    return groups;
}

ReturnCommand LunarLandingGuidance::braking(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                            time::Epoch epoch) {
    const auto frame = site_body_frame(site_, env, epoch);
    const auto x = plan_state(body, frame, plan_);
    const Vec3 r_plan = state_position(x), v_plan = state_velocity(x);
    const Mat3 gcrf_to_plan = transpose(plan_.plan_to_itrf) * transpose(frame.transform.rotation);
    const auto& s = body.state();
    const Vec3 gravity = effective_gravity_plan(body, env, epoch, frame, x);
    const auto law = braking_.update(plan_to_guidance_ * r_plan, plan_to_guidance_ * v_plan, plan_to_guidance_ * gravity);
    braking_t_go_ = law.time_to_go_s;

    ReturnCommand cmd;
    cmd.phase = ReturnPhase::LandingBurn;
    if (!law.valid || law.time_to_go_s <= parameters_.approach_time_to_go_s) {
        start_approach(body, env, epoch);
        return phase_ == LunarDescentPhase::Approach ? approach_command(body, env, epoch) : cmd;
    }
    const auto& vehicle = body.vehicle();
    const double mass = vehicle.mass_properties().mass_kg;
    const double demand = mass * norm(law.thrust_acceleration_mps2);
    cmd.run_groups = braking_groups_now(vehicle, demand, epoch);
    cmd.throttle = std::max(landing_throttle(vehicle, cmd.run_groups, demand, 0.0), set_min_throttle(vehicle, set_groups_));
    cmd.time_to_go_s = law.time_to_go_s;
    const Vec3 direction = transpose(gcrf_to_plan) * (transpose(plan_to_guidance_) * law.thrust_acceleration_mps2);
    cmd.axis_gcrf = hull_axis_for(body, direction, set_groups_);
    if (debug_ && epoch - next_debug_ >= 0.0) {
        const Vec3 thrust_axis =
            s.attitude.rotate(thrust_axis_body(vehicle, vehicle.mass_properties().cg_station_m, set_groups_));
        const Vec3 wanted = math::normalized(direction);
        next_debug_ = epoch + (law.time_to_go_s < 30.0 ? 0.5 : debug_interval_s);
        const Vec3 rg = plan_to_guidance_ * r_plan, vg = plan_to_guidance_ * v_plan;
        std::printf("[lunar_descent] braking t_go %.1f r %.0f/%.0f/%.0f v %.1f/%.1f/%.1f demand %.2f MN throttle %.2f "
                    "m %.1f t axis error %.2f deg rate %.3f/%.3f/%.3f set %s%s\n",
                    law.time_to_go_s, rg.x, rg.y, rg.z, vg.x, vg.y, vg.z, demand / 1e6, cmd.throttle, mass / 1000.0,
                    std::acos(std::clamp(dot(thrust_axis, wanted), -1.0, 1.0)) / deg, s.angular_velocity_body_radps.x,
                    s.angular_velocity_body_radps.y, s.angular_velocity_body_radps.z, set_groups_.front().c_str(),
                    next_lit_ ? " (+next)" : "");
    }
    return cmd;
}

void LunarLandingGuidance::start_approach(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                          time::Epoch epoch) {
    approach_ = std::make_unique<ScvxLandingGuidance>(site_, parameters_.approach);
    if (debug_) {
        const auto frame = site_body_frame(site_, env, epoch);
        const auto x = plan_state(body, frame, plan_);
        const Vec3 rg = plan_to_guidance_ * state_position(x), vg = plan_to_guidance_ * state_velocity(x);
        const Vec3 axis = plan_to_guidance_ * state_attitude(x).rotate(Vec3{1.0, 0.0, 0.0});
        const Vec3 w = state_rate(x);
        std::printf("[lunar_descent] hand-over t_go %.1f r %.1f/%.1f/%.1f v %.2f/%.2f/%.2f m %.1f t thrust axis "
                    "%.3f/%.3f/%.3f rate %.4f/%.4f/%.4f\n",
                    braking_t_go_, rg.x, rg.y, rg.z, vg.x, vg.y, vg.z, body.vehicle().mass_properties().mass_kg / 1000.0,
                    axis.x, axis.y, axis.z, w.x, w.y, w.z);
    }
    if (approach_->start(body, env, epoch)) {
        phase_ = LunarDescentPhase::Approach;
        status_ = "Anflug";
    } else {
        phase_ = LunarDescentPhase::Aborted;
        status_ = "Anflug nicht planbar";
    }
}

ReturnCommand LunarLandingGuidance::approach_command(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                                     time::Epoch epoch) {
    ReturnCommand cmd;
    const auto c = approach_->update(body, env, epoch);
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
    cmd.phase = ReturnPhase::LandingBurn;
    if (c.run_groups == parameters_.approach.snapshot.phase_groups.back()) {
        // The last set is lit: the terminal descent flies it.
        phase_ = LunarDescentPhase::Terminal;
        status_ = "Endanflug";
        if (debug_) std::printf("[lunar_descent] terminal descent (offset %.1f m)\n", c.offset_m);
        return terminal(body, env, epoch);
    }
    return cmd;
}

Vec3 LunarLandingGuidance::effective_gravity_plan(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                                  time::Epoch epoch, const environment::BodyFrame& frame,
                                                  const DescentState& x) const {
    const Mat3 gcrf_to_plan = transpose(plan_.plan_to_itrf) * transpose(frame.transform.rotation);
    const Vec3 gravitation = gcrf_to_plan * (env.gravity.acceleration_mps2(body.state().position_gcrf_m, epoch) -
                                             env.moon->centre_acceleration_gcrf(epoch));
    const Vec3 omega = gcrf_to_plan * frame.rotation_gcrf_radps;
    const Vec3 r_centre = state_position(x) + plan_.direction_to_plan(plan_.origin_itrf_m);
    return gravitation - 2.0 * cross(omega, state_velocity(x)) - cross(omega, cross(omega, r_centre));
}

Vec3 LunarLandingGuidance::hull_axis_for(const vehicle::FlightBody& body, const Vec3& direction_gcrf,
                                         const std::vector<std::string>& groups) {
    const auto& s = body.state();
    const Vec3 hull_x = s.attitude.rotate(Vec3{1.0, 0.0, 0.0});
    if (norm(direction_gcrf) == 0.0) return hull_x;
    const Vec3 wanted = math::normalized(direction_gcrf);
    const auto& vehicle = body.vehicle();
    const Vec3 thrust_axis = s.attitude.rotate(thrust_axis_body(vehicle, vehicle.mass_properties().cg_station_m, groups));
    const Vec3 turn = cross(thrust_axis, wanted);
    const double angle = std::atan2(norm(turn), dot(thrust_axis, wanted));
    const Quat to_wanted = norm(turn) > 1e-12 ? Quat::from_axis_angle(turn / norm(turn), angle) : Quat{};
    return to_wanted.rotate(hull_x);
}

ReturnCommand LunarLandingGuidance::terminal(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                             time::Epoch epoch) {
    const auto& t = parameters_.terminal;
    const auto frame = site_body_frame(site_, env, epoch);
    const auto x = plan_state(body, frame, plan_);
    const Vec3 r = state_position(x), v = state_velocity(x);
    const auto& vehicle = body.vehicle();
    const auto mass = vehicle.mass_properties();
    const Vec3 g = effective_gravity_plan(body, env, epoch, frame, x);
    // Height of the feet above the touchdown point: the CG over its touchdown height.
    const double contact = site_.pad ? site_.pad->contact_bottom_height_m : 0.0;
    const double height = r.x - (mass.cg_station_m + contact);
    const double sink = std::clamp(height / t.sink_time_constant_s, t.touchdown_sink_mps, t.max_sink_mps);
    const double a_up = (-sink - v.x) / t.vertical_time_constant_s;
    // Horizontal: null the velocity and the offset (critically damped), within the tilt the thrust can take. The RCS
    // turns the thrust only slowly: the loop acts on the state predicted over lead_s with the thrust's present lateral
    // acceleration (its actual direction), so it does not overshoot while the hull is still turning.
    const Vec3 axis_now = state_attitude(x).rotate(Vec3{1.0, 0.0, 0.0});
    const double thrust_now = vehicle_thrust_n(vehicle) / mass.mass_kg;
    const Vec3 a_now{0.0, thrust_now * axis_now.y + g.y, thrust_now * axis_now.z + g.z};
    const Vec3 r_lead = r + v * t.lead_s + a_now * (0.5 * t.lead_s * t.lead_s);
    const Vec3 v_lead = v + a_now * t.lead_s;
    const double w = t.lateral_frequency_rad_s;
    Vec3 a_side{0.0, -w * w * r_lead.y - 2.0 * t.lateral_damping * w * v_lead.y,
                -w * w * r_lead.z - 2.0 * t.lateral_damping * w * v_lead.z};
    // The thrust cannot go below the set's minimum: the tilt is measured against at least that (a sink-rate demand
    // below it would otherwise leave only the lateral part and turn the thrust sideways).
    const auto& groups = parameters_.approach.snapshot.phase_groups.back();
    double min_thrust_n = 0.0;
    for (const auto& e : vehicle.stages().front().engines) {
        if (!e.starved && contains(groups, e.group_id)) {
            min_thrust_n += e.performance->thrust_n(e.performance->config().throttle_min, 0.0) * e.axis_body.x;
        }
    }
    const double vertical = std::max(a_up - g.x, min_thrust_n / mass.mass_kg);
    if (const double n = norm(a_side), limit = vertical * std::tan(t.max_tilt_rad); n > limit) a_side = a_side * (limit / n);
    const Vec3 thrust_plan = Vec3{vertical, a_side.y - g.y, a_side.z - g.z};
    ReturnCommand cmd;
    cmd.phase = ReturnPhase::LandingBurn;
    cmd.run_groups = groups;
    const double demand = mass.mass_kg * norm(thrust_plan);
    cmd.throttle = std::max(landing_throttle(vehicle, groups, demand, 0.0), set_min_throttle(vehicle, groups));
    const Vec3 direction = frame.transform.rotation * plan_.direction_to_itrf(thrust_plan);
    cmd.axis_gcrf = hull_axis_for(body, direction, groups);  // the attitude controller turns it (RCS)
    cmd.time_to_go_s = height / sink;
    cmd.impact_miss_m = std::hypot(r.y, r.z);
    if (debug_ && epoch - next_debug_ >= 0.0) {
        next_debug_ = epoch + 1.0;
        const Vec3 axis = state_attitude(x).rotate(Vec3{1.0, 0.0, 0.0});
        std::printf("[lunar_descent] terminal h %.1f v %.2f/%.2f/%.2f offset %.2f/%.2f thrust %.2f MN throttle %.2f "
                    "tilt cmd %.2f deg axis %.3f/%.3f\n",
                    height, v.x, v.y, v.z, r.y, r.z, demand / 1e6, cmd.throttle,
                    std::atan2(std::hypot(thrust_plan.y, thrust_plan.z), thrust_plan.x) / deg, axis.y, axis.z);
    }
    return cmd;
}

ReturnCommand LunarLandingGuidance::update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                           time::Epoch epoch, bool leg_contact) {
    const auto flying = [&] {
        return phase_ == LunarDescentPhase::Braking || phase_ == LunarDescentPhase::Approach ||
               phase_ == LunarDescentPhase::Terminal;
    };
    if (leg_contact && flying()) {
        phase_ = LunarDescentPhase::Touchdown;
        if (debug_) std::printf("[lunar_descent] leg contact: engines off\n");
    }
    ReturnCommand cmd;
    if (phase_ == LunarDescentPhase::Braking) {
        cmd = braking(body, env, epoch);
    } else if (phase_ == LunarDescentPhase::Approach) {
        cmd = approach_command(body, env, epoch);
    } else if (phase_ == LunarDescentPhase::Terminal) {
        cmd = terminal(body, env, epoch);
    }
    if (!flying()) {
        cmd = ReturnCommand{};
        cmd.phase = ReturnPhase::Landed;
        cmd.axis_gcrf = body.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});  // hold
        cmd.abort = phase_ == LunarDescentPhase::Aborted;
    }
    started_ = true;
    last_update_ = epoch;
    return cmd;
}

}  // namespace starsim::gnc
