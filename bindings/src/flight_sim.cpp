#include "flight_sim.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "starsim/math/constants.hpp"

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
}

}  // namespace starsim::bindings
