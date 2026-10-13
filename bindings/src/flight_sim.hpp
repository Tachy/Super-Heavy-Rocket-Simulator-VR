#pragma once

#include <filesystem>
#include <memory>

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"

namespace starsim::bindings {

// Godot facade of sim::FlightSession. Requires an open StarsimCore (shared SPICE kernel pool).
// Positions are GCRF metres (float64), attitudes body -> GCRF.
class FlightSim : public godot::RefCounted {
    GDCLASS(FlightSim, godot::RefCounted)

public:
    void setup(const godot::String& config_dir);
    bool launch_from_pad(const godot::String& vehicle_id, double latitude_deg, double longitude_deg, double height_m,
                         double heading_deg, double tdb_s);
    // Launch from a site of config/launch_sites.toml (with its catch tower, if it has one).
    bool launch_from_site(const godot::String& vehicle_id, const godot::String& site_id, double heading_deg, double tdb_s);
    // Catch tower geometry of a site (no flight needed): {present, origin_itrf_m, east, north, up (ITRF units),
    // tower_direction, across, tower_center_m, tower_size_m, hinge_plus_m, hinge_minus_m (site frame), mount_*,
    // arm_* sizes, arm_open_rad}.
    godot::Dictionary site_tower(const godot::String& site_id) const;
    // Tower of the current flight: {present, opening_rad, carriage_offset_m, phase (German)}.
    godot::Dictionary tower_state() const;
    void set_booster_return(bool enabled);
    void set_realtime_planning(bool enabled);
    godot::Dictionary return_state(int64_t index) const;
    bool is_active() const { return session_ && session_->active(); }
    double advance_to(double tdb_s);
    double mission_time_s() const;

    void set_input(double pitch, double yaw, double roll, double throttle);
    void set_rate_mode(bool enabled);
    void ignite();
    void cutoff();
    void release_clamps();
    void stage();

    // Guidance and manoeuvres.
    void engage_ascent_autopilot(double periapsis_km, double apoapsis_km, double inclination_deg, bool northerly);
    void disengage_autopilot();
    bool plan_circularization();
    // Node `seconds_from_now` ahead with orbit-relative components (prograde, normal, radial-out), m/s.
    bool plan_burn(double seconds_from_now, double prograde_mps, double normal_mps, double radial_mps);
    // {state, phase, time_to_go_s, direction (GCRF), burn: {time_to_ignition_s, remaining_dv_mps,
    //  burn_duration_s, direction}}
    godot::Dictionary autopilot_status() const;
    godot::PackedVector3Array predict_trajectory(int index, double duration_s, double step_s) const;
    // Target orbit of the ascent guidance as a closed curve (GCRF), empty without guidance.
    godot::PackedVector3Array target_orbit_points(int count) const;
    godot::Vector3 body_velocity_gcrf_mps(int index) const;

    int body_count() const;
    int controlled_index() const;
    godot::Vector3 body_position_gcrf_m(int index) const;
    godot::Quaternion body_attitude_gcrf(int index) const;
    double body_cg_station_m(int index) const;
    // [{name, base_station_m, length_m, diameter_m, nose_length_m}] bottom to top
    godot::Array body_stages(int index) const;
    // [{position_m (body, relative to the bottom), firing, throttle, spool}]
    godot::Array body_engines(int index) const;
    godot::Dictionary telemetry(int index) const;

protected:
    static void _bind_methods();

private:
    bool check_index(int index) const;

    spice::SpiceEphemeris ephemeris_;
    spice::SpiceOrientation earth_ = spice::SpiceOrientation::earth();
    std::unique_ptr<sim::FlightSession> session_;
    std::filesystem::path config_dir_;
};

}  // namespace starsim::bindings
