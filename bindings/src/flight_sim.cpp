#include "flight_sim.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"

namespace starsim::bindings {

namespace {

godot::Vector3 to_godot(const math::Vec3& v) { return {v.x, v.y, v.z}; }
time::Epoch epoch(double tdb_s) { return time::Epoch::from_tdb_seconds_past_j2000(tdb_s); }

}  // namespace

void FlightSim::setup(const godot::String& config_dir) {
    session_ = std::make_unique<sim::FlightSession>(ephemeris_, earth_,
                                                    std::filesystem::path(config_dir.utf8().get_data()));
}

bool FlightSim::launch_from_pad(const godot::String& vehicle_id, double latitude_deg, double longitude_deg,
                                double height_m, double heading_deg, double tdb_s) {
    if (!session_) {
        godot::UtilityFunctions::push_error("FlightSim: call setup() first");
        return false;
    }
    try {
        const double deg = constants::deg_to_rad;
        session_->launch_from_pad(vehicle_id.utf8().get_data(), {latitude_deg * deg, longitude_deg * deg, height_m},
                                  heading_deg * deg, epoch(tdb_s));
        return true;
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String("FlightSim.launch_from_pad: ") + e.what());
        return false;
    }
}

double FlightSim::advance_to(double tdb_s) {
    if (!is_active()) return tdb_s;
    try {
        return session_->advance_to(epoch(tdb_s)).tdb_seconds_past_j2000();
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String("FlightSim.advance_to: ") + e.what());
        return tdb_s;
    }
}

double FlightSim::mission_time_s() const { return is_active() ? session_->mission_time_s() : 0.0; }

void FlightSim::set_input(double pitch, double yaw, double roll, double throttle) {
    if (session_) session_->set_input({pitch, yaw, roll, throttle});
}

void FlightSim::set_rate_mode(bool enabled) {
    if (session_) {
        session_->set_control_mode(enabled ? sim::ControlMode::RateCommand : sim::ControlMode::DirectGimbal);
    }
}

void FlightSim::ignite() { if (is_active()) session_->ignite(); }
void FlightSim::cutoff() { if (is_active()) session_->cutoff(); }
void FlightSim::release_clamps() { if (is_active()) session_->release_clamps(); }
void FlightSim::stage() { if (is_active()) session_->stage(); }

int FlightSim::body_count() const { return is_active() ? static_cast<int>(session_->body_count()) : 0; }
int FlightSim::controlled_index() const { return is_active() ? static_cast<int>(session_->controlled_index()) : -1; }

bool FlightSim::check_index(int index) const {
    if (!is_active() || index < 0 || index >= static_cast<int>(session_->body_count())) {
        godot::UtilityFunctions::push_error("FlightSim: invalid body index");
        return false;
    }
    return true;
}

godot::Vector3 FlightSim::body_position_gcrf_m(int index) const {
    if (!check_index(index)) return {};
    return to_godot(session_->body(index).state().position_gcrf_m);
}

godot::Quaternion FlightSim::body_attitude_gcrf(int index) const {
    if (!check_index(index)) return {};
    const auto& q = session_->body(index).state().attitude;
    return {q.x, q.y, q.z, q.w};
}

double FlightSim::body_cg_station_m(int index) const {
    if (!check_index(index)) return 0.0;
    return session_->body(index).vehicle().mass_properties().cg_station_m;
}

godot::Array FlightSim::body_stages(int index) const {
    godot::Array stages;
    if (!check_index(index)) return stages;
    for (const auto& stage : session_->body(index).vehicle().stages()) {
        godot::Dictionary d;
        d["name"] = godot::String(stage.config.name.c_str());
        d["id"] = godot::String(stage.config.id.c_str());
        d["base_station_m"] = stage.base_station_m;
        d["length_m"] = stage.config.length_m;
        d["diameter_m"] = stage.config.diameter_m;
        d["nose_length_m"] = stage.config.aero.nose_length_m;
        stages.push_back(d);
    }
    return stages;
}

godot::Array FlightSim::body_engines(int index) const {
    godot::Array engines;
    if (!check_index(index)) return engines;
    for (const auto& stage : session_->body(index).vehicle().stages()) {
        for (const auto& e : stage.engines) {
            godot::Dictionary d;
            d["position_m"] = to_godot(e.gimbal_point_m);
            d["direction"] = to_godot(e.thrust_direction());
            d["firing"] = e.producing_thrust();
            d["throttle"] = e.throttle;
            d["spool"] = e.spool;
            d["vacuum"] = e.performance->config().reference_pressure_pa == 0.0;
            engines.push_back(d);
        }
    }
    return engines;
}

godot::Dictionary FlightSim::telemetry(int index) const {
    godot::Dictionary d;
    if (!check_index(index)) return d;
    const auto t = session_->telemetry(index);
    d["altitude_m"] = t.altitude_m;
    d["latitude_deg"] = t.latitude_rad * constants::rad_to_deg;
    d["longitude_deg"] = t.longitude_rad * constants::rad_to_deg;
    d["speed_inertial_mps"] = t.speed_inertial_mps;
    d["speed_relative_mps"] = t.speed_relative_mps;
    d["vertical_speed_mps"] = t.vertical_speed_mps;
    d["flight_path_angle_deg"] = t.flight_path_angle_rad * constants::rad_to_deg;
    d["mach"] = t.mach;
    d["knudsen"] = t.knudsen;
    d["dynamic_pressure_pa"] = t.dynamic_pressure_pa;
    d["angle_of_attack_deg"] = t.angle_of_attack_rad * constants::rad_to_deg;
    d["mass_kg"] = t.mass_kg;
    d["propellant_kg"] = t.propellant_kg;
    d["thrust_n"] = t.thrust_n;
    d["sensed_acceleration_g"] = t.sensed_acceleration_g;
    d["engines_running"] = t.engines_running;
    d["clamped"] = session_->body(index).clamped();
    d["impacted"] = session_->body(index).impacted();
    return d;
}

void FlightSim::engage_ascent_autopilot(double periapsis_km, double apoapsis_km, double inclination_deg,
                                        bool northerly) {
    if (!is_active()) return;
    session_->engage_ascent_autopilot(
        {periapsis_km * 1000.0, apoapsis_km * 1000.0, inclination_deg * constants::deg_to_rad, northerly});
}

void FlightSim::disengage_autopilot() {
    if (is_active()) session_->disengage_autopilot();
}

bool FlightSim::plan_circularization() { return is_active() && session_->plan_circularization_at_apoapsis(); }

bool FlightSim::plan_burn(double seconds_from_now, double prograde_mps, double normal_mps, double radial_mps) {
    if (!is_active()) return false;
    const auto& s = session_->body(session_->controlled_index()).state();
    const auto at_node = orbit::propagate_two_body({s.position_gcrf_m, s.velocity_gcrf_mps},
                                                   session_->gm_earth_m3ps2(), seconds_from_now);
    return session_->execute_burn(
        {session_->epoch() + seconds_from_now,
         gnc::delta_v_from_components(at_node, prograde_mps, normal_mps, radial_mps)});
}

godot::Dictionary FlightSim::autopilot_status() const {
    godot::Dictionary d;
    if (!is_active()) return d;
    const auto& ap = session_->autopilot();
    d["state"] = godot::String::utf8(sim::to_string(ap.state));
    d["engaged"] = ap.state != sim::AutopilotState::Off && ap.state != sim::AutopilotState::Orbit;
    d["phase"] = godot::String::utf8(gnc::to_string(ap.guidance.phase));
    d["time_to_go_s"] = ap.guidance.time_to_go_s;
    const bool burning = ap.state == sim::AutopilotState::Burn;
    d["direction"] = to_godot(burning ? ap.burn.direction_gcrf : ap.guidance.thrust_direction_gcrf);
    d["has_direction"] = ap.state == sim::AutopilotState::Booster || ap.state == sim::AutopilotState::UpperStage ||
                         burning;
    if (burning) {
        godot::Dictionary b;
        b["time_to_ignition_s"] = ap.burn.time_to_ignition_s;
        b["remaining_dv_mps"] = ap.burn.remaining_dv_mps;
        b["burn_duration_s"] = ap.burn.burn_duration_s;
        b["total_dv_mps"] = session_->burn() ? norm(session_->burn()->node().delta_v_gcrf_mps) : 0.0;
        d["burn"] = b;
    }
    return d;
}

godot::PackedVector3Array FlightSim::predict_trajectory(int index, double duration_s, double step_s) const {
    godot::PackedVector3Array points;
    if (!check_index(index)) return points;
    try {
        for (const auto& p : session_->predict_trajectory(static_cast<size_t>(index), duration_s, step_s)) {
            points.push_back(to_godot(p));
        }
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String("FlightSim.predict_trajectory: ") + e.what());
    }
    return points;
}

godot::PackedVector3Array FlightSim::target_orbit_points(int count) const {
    godot::PackedVector3Array points;
    const auto* guidance = is_active() ? session_->ascent_guidance() : nullptr;
    if (guidance == nullptr || count < 3) return points;
    const math::Vec3 n = guidance->target_plane_normal_gcrf();
    const math::Vec3 u = math::normalized(cross(math::Vec3{0.0, 0.0, 1.0}, n));  // ascending node
    const math::Vec3 w = cross(n, u);
    const double r = guidance->target_radius_m();
    for (int i = 0; i <= count; ++i) {
        const double t = constants::two_pi * i / count;
        points.push_back(to_godot(r * (std::cos(t) * u + std::sin(t) * w)));
    }
    return points;
}

godot::Vector3 FlightSim::body_velocity_gcrf_mps(int index) const {
    if (!check_index(index)) return {};
    return to_godot(session_->body(index).state().velocity_gcrf_mps);
}

void FlightSim::_bind_methods() {
    using godot::ClassDB;
    using godot::D_METHOD;
    ClassDB::bind_method(D_METHOD("setup", "config_dir"), &FlightSim::setup);
    ClassDB::bind_method(D_METHOD("launch_from_pad", "vehicle_id", "latitude_deg", "longitude_deg", "height_m",
                                  "heading_deg", "tdb_s"),
                         &FlightSim::launch_from_pad);
    ClassDB::bind_method(D_METHOD("is_active"), &FlightSim::is_active);
    ClassDB::bind_method(D_METHOD("advance_to", "tdb_s"), &FlightSim::advance_to);
    ClassDB::bind_method(D_METHOD("mission_time_s"), &FlightSim::mission_time_s);
    ClassDB::bind_method(D_METHOD("set_input", "pitch", "yaw", "roll", "throttle"), &FlightSim::set_input);
    ClassDB::bind_method(D_METHOD("set_rate_mode", "enabled"), &FlightSim::set_rate_mode);
    ClassDB::bind_method(D_METHOD("ignite"), &FlightSim::ignite);
    ClassDB::bind_method(D_METHOD("cutoff"), &FlightSim::cutoff);
    ClassDB::bind_method(D_METHOD("release_clamps"), &FlightSim::release_clamps);
    ClassDB::bind_method(D_METHOD("stage"), &FlightSim::stage);
    ClassDB::bind_method(D_METHOD("body_count"), &FlightSim::body_count);
    ClassDB::bind_method(D_METHOD("controlled_index"), &FlightSim::controlled_index);
    ClassDB::bind_method(D_METHOD("body_position_gcrf_m", "index"), &FlightSim::body_position_gcrf_m);
    ClassDB::bind_method(D_METHOD("body_attitude_gcrf", "index"), &FlightSim::body_attitude_gcrf);
    ClassDB::bind_method(D_METHOD("body_cg_station_m", "index"), &FlightSim::body_cg_station_m);
    ClassDB::bind_method(D_METHOD("body_stages", "index"), &FlightSim::body_stages);
    ClassDB::bind_method(D_METHOD("body_engines", "index"), &FlightSim::body_engines);
    ClassDB::bind_method(D_METHOD("telemetry", "index"), &FlightSim::telemetry);
    ClassDB::bind_method(D_METHOD("engage_ascent_autopilot", "periapsis_km", "apoapsis_km", "inclination_deg",
                                  "northerly"),
                         &FlightSim::engage_ascent_autopilot);
    ClassDB::bind_method(D_METHOD("disengage_autopilot"), &FlightSim::disengage_autopilot);
    ClassDB::bind_method(D_METHOD("plan_circularization"), &FlightSim::plan_circularization);
    ClassDB::bind_method(D_METHOD("plan_burn", "seconds_from_now", "prograde_mps", "normal_mps", "radial_mps"),
                         &FlightSim::plan_burn);
    ClassDB::bind_method(D_METHOD("autopilot_status"), &FlightSim::autopilot_status);
    ClassDB::bind_method(D_METHOD("predict_trajectory", "index", "duration_s", "step_s"),
                         &FlightSim::predict_trajectory);
    ClassDB::bind_method(D_METHOD("target_orbit_points", "count"), &FlightSim::target_orbit_points);
    ClassDB::bind_method(D_METHOD("body_velocity_gcrf_mps", "index"), &FlightSim::body_velocity_gcrf_mps);
}

}  // namespace starsim::bindings
