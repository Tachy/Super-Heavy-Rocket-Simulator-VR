// Lunar ascent (l4): liftoff on the landing thrusters, vertical rise on the vacuum Raptors, PEG into the
// (site + 15 km) x 100 km ellipse; abort to orbit from a powered descent.
//
// Design decisions:
// - Liftoff: the liftoff set (landing thrusters) at full throttle (T/W ~1.5 at 662 t), the hull's thrust axis along the
//   local vertical. At raptor_start_height_m above the liftoff point the ascent set starts; the liftoff set stays on
//   (bridge) until the ascent set's thrust exceeds the weight (phase Vertical). At vertical_rise_height_m: PEG.
// - Ascent on the three fixed vacuum Raptors (user 2026-10-15: symmetric on the ring, thrust through the CG, Isp 380 s
//   instead of 350 s - ~10 t saved): landing -> vacuum is a disjoint switch as in the braking phase; with no gimbal the
//   attitude controller's torque goes to the RCS. The set throttles down to max_acceleration_mps2, never below its
//   minimum. A single throttle drives every running group: while a bridge set is on, both run at full throttle.
// - Vacuum-Raptor engine-out (user 2026-10-15): the other two are cut at once (their off-axis thrust is a torque no RCS
//   holds), the gimballed sea-level Raptors start and PEG continues on them (it sees their thrust and flow). Before PEG
//   the landing thrusters bridge the start-up.
// - PEG target: radius R + site height + insertion altitude, horizontal at the vis-viva speed of the ellipse to
//   R + aposelene, radial 0, terminal semi-major axis of that ellipse; plane: polar through the liftoff point (normal
//   cross(r_hat, pole), towards the north for `northerly`), inertial, fixed at start. A polar plane through the liftoff
//   point always passes over it: no launch window (a fixed target plane - rendezvous with a depot, M9 - would need one).
//   PEG state: Moon-relative inertial (BodyFrame::centred).
// - Abort: the plane is the current Moon-relative orbital plane; while the vertical speed is below abort_min_climb_mps
//   the ascent set thrusts along the local vertical tilted towards the downrange horizontal (at most abort_max_tilt_rad,
//   keeping 1.2 x the weight vertical) at full throttle (phase Arrest); engines running at the abort (the approach
//   Raptor, the landing thrusters) bridge the ascent set's start-up. PEG takes over once the vehicle climbs at
//   abort_min_climb_mps and a fresh PEG solution is not at its radial limit (abort_peg_max_radial).

#include "starsim/gnc/lunar_ascent_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/gnc/lunar_landing_guidance.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double arrest_weight_factor = 1.2;  // [E] the abort keeps this share of the weight in the vertical thrust

bool contains(const std::vector<std::string>& groups, const std::string& id) {
    return std::find(groups.begin(), groups.end(), id) != groups.end();
}

// Thrust of the engines of `groups` now (starting, running), vacuum, along the hull axis.
double groups_thrust_n(const vehicle::Vehicle& vehicle, const std::vector<std::string>& groups) {
    double thrust = 0.0;
    for (const auto& stage : vehicle.stages()) {
        for (const auto& e : stage.engines) {
            if (e.producing_thrust() && contains(groups, e.group_id)) {
                thrust += e.spool * e.performance->thrust_n(e.throttle, 0.0) * e.axis_body.x;
            }
        }
    }
    return thrust;
}

// Full and minimum thrust of the usable engines of `groups`.
std::pair<double, double> groups_thrust_range_n(const vehicle::Vehicle& vehicle, const std::vector<std::string>& groups) {
    double full = 0.0, minimum_throttle = 0.0;
    for (const auto& stage : vehicle.stages()) {
        for (const auto& e : stage.engines) {
            if (e.starved || !contains(groups, e.group_id)) continue;
            full += e.performance->thrust_n(1.0, 0.0) * e.axis_body.x;
            minimum_throttle = std::max(minimum_throttle, e.performance->config().throttle_min);
        }
    }
    return {full, minimum_throttle};
}

// Thrust and mass flow of the engines starting or running (at full spool), vacuum - PEG's vehicle model.
PegVehicle peg_vehicle(const vehicle::Vehicle& vehicle) {
    PegVehicle p;
    for (const auto& stage : vehicle.stages()) {
        for (const auto& e : stage.engines) {
            if (e.mode != vehicle::EngineMode::Starting && e.mode != vehicle::EngineMode::Running) continue;
            p.thrust_n += e.performance->thrust_n(e.throttle, 0.0);
            p.mass_flow_kgps += e.performance->mass_flow_kgps(e.throttle);
            p.shutdown_s = std::max(p.shutdown_s, e.performance->config().shutdown_s);
        }
    }
    p.mass_kg = vehicle.mass_properties().mass_kg;
    return p;
}

}  // namespace

double groups_vacuum_isp_s(const vehicle::Vehicle& vehicle, const std::vector<std::string>& groups) {
    double thrust_n = 0.0, mass_flow_kgps = 0.0;
    for (const auto& stage : vehicle.stages()) {
        for (const auto& e : stage.engines) {
            if (e.starved || !contains(groups, e.group_id)) continue;
            thrust_n += e.performance->thrust_n(1.0, 0.0);
            mass_flow_kgps += e.performance->mass_flow_kgps(1.0);
        }
    }
    return mass_flow_kgps > 0.0 ? thrust_n / mass_flow_kgps / constants::standard_gravity_mps2 : 0.0;
}

const char* to_string(LunarAscentPhase phase) {
    switch (phase) {
        case LunarAscentPhase::Liftoff: return "Abheben";
        case LunarAscentPhase::Vertical: return "Senkrechtaufstieg";
        case LunarAscentPhase::Arrest: return "Sinken stoppen";
        case LunarAscentPhase::Peg: return "PEG";
        case LunarAscentPhase::Cutoff: return "Brennschluss";
    }
    return "";
}

LunarAscentGuidance::LunarAscentGuidance(LunarAscentParameters parameters, double moon_gm_m3ps2, double site_height_m)
    : parameters_(std::move(parameters)), gm_(moon_gm_m3ps2), site_height_m_(site_height_m), peg_(moon_gm_m3ps2) {
    const double rp = constants::moon_mean_radius_m + site_height_m_ + parameters_.insertion_altitude_above_site_m;
    const double ra = constants::moon_mean_radius_m + parameters_.aposelene_altitude_m;
    target_.radius_m = rp;
    target_.radial_speed_mps = 0.0;
    target_.horizontal_speed_mps = std::sqrt(gm_ * (2.0 / rp - 2.0 / (rp + ra)));
    target_.terminal_semi_major_axis_m = 0.5 * (rp + ra);
    ascent_groups_ = parameters_.ascent_groups;
    debug_ = std::getenv("STARSIM_DEBUG_LUNAR_ASCENT") != nullptr;
}

void LunarAscentGuidance::start(const vehicle::FlightBody& body, const environment::BodyFrame& moon, time::Epoch,
                                bool abort) {
    const auto& s = body.state();
    const auto rel = moon.centred({s.position_gcrf_m, s.velocity_gcrf_mps});
    const Vec3 r_hat = math::normalized(rel.position_m);
    liftoff_radius_m_ = norm(rel.position_m);
    ascent_groups_ = parameters_.ascent_groups;
    engine_out_ = false;
    steering_ = true;
    status_.clear();
    peg_.reset();
    bridge_groups_.clear();
    if (abort) {
        target_.plane_normal = math::normalized(cross(rel.position_m, rel.velocity_mps));
        for (const auto& stage : body.vehicle().stages()) {
            for (const auto& e : stage.engines) {
                if (e.producing_thrust() && !contains(ascent_groups_, e.group_id) && !contains(bridge_groups_, e.group_id)) {
                    bridge_groups_.push_back(e.group_id);
                }
            }
        }
        phase_ = dot(rel.velocity_mps, r_hat) < parameters_.abort_min_climb_mps ? LunarAscentPhase::Arrest
                                                                                  : LunarAscentPhase::Peg;
    } else {
        const Vec3 pole = moon.transform.rotation * Vec3{0.0, 0.0, 1.0};
        const Vec3 normal = math::normalized(cross(r_hat, pole));
        target_.plane_normal = parameters_.northerly ? normal : -normal;
        bridge_groups_ = parameters_.liftoff_groups;
        phase_ = LunarAscentPhase::Liftoff;
    }
}

std::vector<std::string> LunarAscentGuidance::with_bridge(const vehicle::Vehicle& vehicle, double weight_n) {
    if (!bridge_groups_.empty() && groups_thrust_n(vehicle, ascent_groups_) > weight_n) bridge_groups_.clear();
    std::vector<std::string> groups = ascent_groups_;
    const auto add = [&](const std::vector<std::string>& more) {
        for (const auto& g : more) {
            if (!contains(groups, g)) groups.push_back(g);
        }
    };
    add(bridge_groups_);
    if (steering_) add(parameters_.steering_groups);
    return groups;
}

double LunarAscentGuidance::ascent_throttle(const vehicle::Vehicle& vehicle, const std::vector<std::string>& groups) const {
    if (!bridge_groups_.empty()) return 1.0;
    const auto [full_n, minimum] = groups_thrust_range_n(vehicle, groups);
    if (full_n <= 0.0) return 1.0;
    const double wanted = parameters_.max_acceleration_mps2 * vehicle.mass_properties().mass_kg / full_n;
    // while the steering set runs, differential throttling needs room both ways: off the minimum and off full throttle
    const double margin = steering_ ? parameters_.steering_throttle_margin : 0.0;
    return std::clamp(wanted, std::min(minimum + margin, 1.0 - margin), 1.0 - margin);
}

LunarAscentCommand LunarAscentGuidance::update(const vehicle::FlightBody& body, const environment::BodyFrame& moon,
                                               time::Epoch epoch) {
    const auto& s = body.state();
    const auto& vehicle = body.vehicle();
    const auto rel = moon.centred({s.position_gcrf_m, s.velocity_gcrf_mps});
    const double r = norm(rel.position_m);
    const Vec3 r_hat = rel.position_m / r;
    const double height_m = r - liftoff_radius_m_;
    const double weight_n = vehicle.mass_properties().mass_kg * gm_ / (r * r);

    // Engine-out in the fixed ascent set: cut it, the gimballed engine-out set continues (before PEG the liftoff set
    // bridges its start-up).
    if (!engine_out_) {
        bool failed_fixed = false;
        for (const auto& stage : vehicle.stages()) {
            for (const auto& e : stage.engines) {
                failed_fixed = failed_fixed || (e.starved && !e.gimbaled && contains(ascent_groups_, e.group_id));
            }
        }
        if (failed_fixed) {
            engine_out_ = true;
            ascent_groups_ = parameters_.engine_out_groups;
            if (phase_ == LunarAscentPhase::Liftoff || phase_ == LunarAscentPhase::Vertical) {
                bridge_groups_ = parameters_.liftoff_groups;
            }
            status_ = "Triebwerksausfall: Aufstieg auf den Meeresspiegel-Raptoren";
        }
    }

    LunarAscentCommand cmd;
    cmd.phase = phase_;
    if (phase_ == LunarAscentPhase::Cutoff) {
        cmd.axis_gcrf = s.attitude.rotate(Vec3{1.0, 0.0, 0.0});
        cmd.cutoff = true;
        return cmd;
    }
    if (phase_ == LunarAscentPhase::Liftoff) {
        if (height_m >= parameters_.raptor_start_height_m) {
            phase_ = LunarAscentPhase::Vertical;
        } else {
            cmd.run_groups = parameters_.liftoff_groups;
            for (const auto& g : parameters_.steering_groups) {
                if (!contains(cmd.run_groups, g)) cmd.run_groups.push_back(g);
            }
            cmd.throttle = 1.0;
            cmd.axis_gcrf = LunarLandingGuidance::hull_axis_for(body, r_hat, parameters_.liftoff_groups);
            return cmd;
        }
    }
    if (phase_ == LunarAscentPhase::Vertical && height_m >= parameters_.vertical_rise_height_m) {
        phase_ = LunarAscentPhase::Peg;
    }
    if (phase_ == LunarAscentPhase::Arrest && dot(rel.velocity_mps, r_hat) >= parameters_.abort_min_climb_mps) {
        const auto model = peg_vehicle(vehicle);
        // ... or once the ballistic apex reaches the insertion radius (PEG then has the horizontal speed left to gain)
        const double v_r = dot(rel.velocity_mps, r_hat);
        const double v_h = norm(rel.velocity_mps - r_hat * v_r);
        const double net_gravity = std::max(gm_ / (r * r) - v_h * v_h / r, 1e-3);  // less the centrifugal term
        const bool apex_reaches = v_r * v_r / (2.0 * net_gravity) >= target_.radius_m - r;
        if (model.thrust_n > 0.0) {
            peg_.reset();  // a fresh solve every tick: a degenerate one froze its time to go below 10 s
            const auto trial = peg_.update({rel.position_m, rel.velocity_mps}, model, target_, epoch);
            if ((dot(trial.thrust_direction, r_hat) < parameters_.abort_peg_max_radial && !trial.cutoff) || apex_reaches) {
                phase_ = LunarAscentPhase::Peg;  // continues from this converged solution
            }
        }
    }
    cmd.phase = phase_;
    cmd.run_groups = with_bridge(vehicle, weight_n);
    cmd.throttle = ascent_throttle(vehicle, cmd.run_groups);
    if (phase_ == LunarAscentPhase::Vertical) {
        cmd.axis_gcrf = LunarLandingGuidance::hull_axis_for(body, r_hat, ascent_groups_);
        return cmd;
    }
    if (phase_ == LunarAscentPhase::Arrest) {
        cmd.throttle = steering_ ? 1.0 - parameters_.steering_throttle_margin : 1.0;  // room for differential steering
        const Vec3 v_h = rel.velocity_mps - r_hat * dot(rel.velocity_mps, r_hat);
        const double full_n = groups_thrust_range_n(vehicle, ascent_groups_).first;
        const double cos_tilt = full_n > 0.0 ? std::min(1.0, arrest_weight_factor * weight_n / full_n) : 1.0;
        const double tilt = norm(v_h) > 1e-6 ? std::min(std::acos(cos_tilt), parameters_.abort_max_tilt_rad) : 0.0;
        const Vec3 downrange = norm(v_h) > 1e-6 ? v_h / norm(v_h) : Vec3{};
        cmd.axis_gcrf = LunarLandingGuidance::hull_axis_for(body, r_hat * std::cos(tilt) + downrange * std::sin(tilt),
                                                            ascent_groups_);
        return cmd;
    }
    // PEG
    const auto vehicle_model = peg_vehicle(vehicle);
    if (groups_thrust_n(vehicle, ascent_groups_) <= 0.0 || vehicle_model.thrust_n <= 0.0) {
        // the ascent set is still starting (engine-out switch): keep the attitude
        cmd.axis_gcrf = s.attitude.rotate(Vec3{1.0, 0.0, 0.0});
        return cmd;
    }
    const auto peg = peg_.update({rel.position_m, rel.velocity_mps}, vehicle_model, target_, epoch);
    cmd.time_to_go_s = peg.time_to_go_s;
    cmd.axis_gcrf = LunarLandingGuidance::hull_axis_for(body, peg.thrust_direction, ascent_groups_);
    if (steering_) {
        const Vec3 thrust_axis =
            s.attitude.rotate(thrust_axis_body(vehicle, vehicle.mass_properties().cg_station_m, ascent_groups_));
        const double error = std::acos(std::clamp(dot(thrust_axis, math::normalized(peg.thrust_direction)), -1.0, 1.0));
        const Vec3 rate = s.attitude.rotate(s.angular_velocity_body_radps) - moon.rotation_gcrf_radps;
        if (error < parameters_.steering_alignment_rad && norm(rate) < parameters_.steering_rate_radps) steering_ = false;
    }
    if (debug_ && epoch - next_debug_ >= 0.0) {
        next_debug_ = epoch + (peg.time_to_go_s < 20.0 ? 0.5 : 5.0);
        const Vec3 thrust_axis = s.attitude.rotate(thrust_axis_body(vehicle, vehicle.mass_properties().cg_station_m, ascent_groups_));
        const Vec3 wanted = math::normalized(peg.thrust_direction);
        const double v_r = dot(rel.velocity_mps, r_hat);
        std::printf("[lunar_ascent] h %.0f vr %.1f vh %.1f t_go %.1f fr %.3f thr %.2f m %.1f t axis err %.2f deg w %.4f%s\n",
                    height_m, v_r, norm(rel.velocity_mps - r_hat * v_r), peg.time_to_go_s, dot(wanted, r_hat), cmd.throttle,
                    vehicle.mass_properties().mass_kg / 1000.0,
                    std::acos(std::clamp(dot(thrust_axis, wanted), -1.0, 1.0)) / constants::deg_to_rad,
                    norm(s.angular_velocity_body_radps), peg.cutoff ? " CUTOFF" : "");
    }
    if (peg.cutoff) {
        phase_ = LunarAscentPhase::Cutoff;
        cmd.phase = phase_;
        cmd.cutoff = true;
        cmd.run_groups.clear();
    }
    return cmd;
}

double LunarAscentGuidance::abort_delta_v_mps(const vehicle::FlightBody& body, const environment::BodyFrame& moon) const {
    const auto& s = body.state();
    const auto rel = moon.centred({s.position_gcrf_m, s.velocity_gcrf_mps});
    const Vec3 r_hat = math::normalized(rel.position_m);
    const double v_r = dot(rel.velocity_mps, r_hat);
    const double v_h = norm(rel.velocity_mps - r_hat * v_r);
    const double to_gain = std::max(0.0, target_.horizontal_speed_mps - v_h) + std::max(0.0, -v_r);
    return to_gain * (1.0 + parameters_.abort_delta_v_margin);
}

}  // namespace starsim::gnc
