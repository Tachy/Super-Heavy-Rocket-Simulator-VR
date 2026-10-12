#pragma once

#include <memory>

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
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
    bool is_active() const { return session_ && session_->active(); }
    double advance_to(double tdb_s);
    double mission_time_s() const;

    void set_input(double pitch, double yaw, double roll, double throttle);
    void set_rate_mode(bool enabled);
    void ignite();
    void cutoff();
    void release_clamps();
    void stage();

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
};

}  // namespace starsim::bindings
