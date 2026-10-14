#pragma once

#include <memory>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/time/epoch.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::vehicle {

// Rigid-body state of the vehicle's centre of gravity, Earth-centred GCRF.
struct RigidBodyState {
    math::Vec3 position_gcrf_m;
    math::Vec3 velocity_gcrf_mps;
    math::Quat attitude;                    // body -> GCRF
    math::Vec3 angular_velocity_body_radps;
};

// Everything outside the vehicle that acts on it. Gravity must be Earth-centred.
struct FlightEnvironment {
    const environment::GravityModel& gravity;
    const environment::Atmosphere& atmosphere;
    const frames::BodyOrientation& earth_orientation;
    const ephemeris::Ephemeris* ephemeris = nullptr;  // needed for solar radiation pressure
    bool solar_radiation_pressure = true;
};

// Air data at the start of the last tick (what an air-data system would report to the controllers).
struct AirData {
    math::Vec3 air_velocity_body_mps;  // vehicle CG relative to the air
    aero::FinAirData air;
    double mean_free_path_m = 0.0;
};

struct FlightTelemetry {
    double altitude_m = 0.0;              // above WGS84
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double speed_inertial_mps = 0.0;
    double speed_relative_mps = 0.0;      // relative to the rotating Earth (what webcasts show)
    double vertical_speed_mps = 0.0;      // relative, along the local vertical
    double flight_path_angle_rad = 0.0;   // relative velocity above the local horizontal
    double mach = 0.0;                    // meaningful only in continuum flow (knudsen < 0.01)
    double knudsen = 0.0;                 // mean free path / vehicle length
    double dynamic_pressure_pa = 0.0;
    double angle_of_attack_rad = 0.0;
    double mass_kg = 0.0;
    double propellant_kg = 0.0;
    double thrust_n = 0.0;
    double sensed_acceleration_g = 0.0;   // non-gravitational acceleration / g0
    int engines_running = 0;
};

// A vehicle moving under gravity, thrust, aerodynamics and solar radiation pressure.
// Fixed-tick integration (classical RK4, typically 200 Hz); engine and propellant state change
// discretely between ticks, mass properties vary linearly inside a tick.
// Simplifications [documented]: no jet damping, no propellant slosh, CG motion inside the body is
// quasi-static, no gravity-gradient torque, no aerodynamic damping derivatives.
class FlightBody {
public:
    FlightBody(std::unique_ptr<Vehicle> vehicle, const RigidBodyState& state);

    // Vehicle standing on a pad: its bottom at `pad_itrf_m`, axis along the local geodetic vertical,
    // body +y pointing towards `heading_rad` (clockwise from north). Held down until release_clamps().
    static std::unique_ptr<FlightBody> on_pad(std::unique_ptr<Vehicle> vehicle, double latitude_rad,
                                              double longitude_rad, double height_m, double heading_rad,
                                              const frames::BodyOrientation& earth, time::Epoch epoch);

    void step(const FlightEnvironment& env, time::Epoch epoch, double dt_s, const VehicleCommand& command);

    // Unpowered flight over a longer interval with adaptive DOP853 on the same 13-element state and the
    // same forces (gravity, aerodynamics, solar radiation pressure; no thrust, no RCS). Used instead of
    // fixed ticks when nothing is commanded, e.g. for a multi-day lunar transfer.
    void coast(const FlightEnvironment& env, time::Epoch epoch, double duration_s);
    bool can_coast() const;  // engines off, not clamped, not on the ground

    bool clamped() const { return clamped_; }
    void release_clamps() { clamped_ = false; }
    // Contact wrench for the next step() only: force through the CG (GCRF) and torque (body axes).
    void apply_external_wrench(const math::Vec3& force_gcrf_n, const math::Vec3& torque_body_nm);
    // Fixes the body to the Earth where it is (caught by the tower): from now on it co-rotates with the Earth
    // like a clamped vehicle and is no longer integrated.
    void hold_fixed_to_earth(const frames::BodyOrientation& earth, time::Epoch epoch);
    bool caught() const { return caught_; }
    // Fixes a stage standing on its legs to the Earth where it is (landed): resting() from now on.
    void hold_landed(const frames::BodyOrientation& earth, time::Epoch epoch);
    void mark_impacted() { impacted_ = true; }  // struck the tower or overloaded a catch pin
    bool impacted() const { return impacted_; }
    bool resting() const { return resting_ || landed_; }  // carried by the ground after a gentle touchdown / landed
    // Entry (a single stage with a heat shield): stagnation heat rate of the last tick, its integral and peak.
    // Exceeding a limit breaks the vehicle up (impacted() then true, breakup_reason() names the cause in German);
    // heat_warning() from 80 % of a limit. Heating is integrated in step() only (coast() is for vacuum arcs).
    double heat_rate_w_m2() const { return heat_rate_w_m2_; }
    double heat_load_j_m2() const { return heat_load_j_m2_; }
    double peak_heat_rate_w_m2() const { return peak_heat_rate_w_m2_; }
    bool heat_warning() const { return heat_warning_; }
    const std::string& breakup_reason() const { return breakup_reason_; }
    // Skip-out ("Abprall"): after dipping below 80 km the vehicle climbed back above 100 km (set once, stays set).
    bool skipped_out() const { return skipped_out_; }
    // Accelerometer: aerodynamic acceleration (hull, fins, flaps; body axes) at the end of the last tick. Set while the
    // entry limits are judged (single stage with a heat shield) or the accelerometer is on, zero otherwise.
    const math::Vec3& aero_acceleration_body_mps2() const { return aero_acceleration_body_mps2_; }
    // Guidance that reads the accelerometer turns it on (one more force evaluation per tick).
    void set_accelerometer(bool on) { accelerometer_ = on; }
    const AirData& air_data() const { return air_data_; }

    // Flat ground at this height above WGS84 (the pad elevation; terrain comes later).
    double ground_height_m() const { return ground_height_m_; }
    void set_ground_height(double height_m) { ground_height_m_ = height_m; }

    static constexpr double max_touchdown_sink_rate_mps = 3.0;  // [E] structural limit
    static constexpr double max_touchdown_tilt_rad = 10.0 * 3.14159265358979 / 180.0;

    // Splits the stack below stage `index`. Returns the upper part; both keep position, velocity and
    // rotation consistent with the rigid body before the split. `separation_dv_mps` pushes them apart
    // along the axis (split by mass).
    std::unique_ptr<FlightBody> separate(size_t index, double separation_dv_mps = 0.0);

    FlightTelemetry telemetry(const FlightEnvironment& env, time::Epoch epoch) const;

    const Vehicle& vehicle() const { return *vehicle_; }
    Vehicle& vehicle() { return *vehicle_; }
    const RigidBodyState& state() const { return state_; }
    void set_state(const RigidBodyState& state) { state_ = state; }
    // Force of the reaction-control thrusters that fired in the last tick, body axes.
    math::Vec3 rcs_force_body_n() const { return rcs_force_body_n_; }

private:
    void follow_pad(const frames::BodyOrientation& earth, time::Epoch epoch);
    void resolve_ground_contact(const frames::BodyOrientation& earth, time::Epoch epoch);
    void update_air_data(const FlightEnvironment& env, time::Epoch epoch);

    std::unique_ptr<Vehicle> vehicle_;
    RigidBodyState state_;
    bool clamped_ = false;
    bool impacted_ = false;
    bool resting_ = false;
    AirData air_data_;
    math::Vec3 pad_itrf_m_;
    math::Quat attitude_itrf_;  // body -> ITRF while clamped
    double ground_height_m_ = 0.0;
    math::Vec3 external_force_gcrf_n_;
    math::Vec3 external_torque_body_nm_;
    bool caught_ = false;
    bool landed_ = false;  // standing on its legs, held to the Earth (hold_landed)
    double heat_rate_w_m2_ = 0.0;
    double heat_load_j_m2_ = 0.0;
    double peak_heat_rate_w_m2_ = 0.0;
    bool heat_warning_ = false;
    std::string breakup_reason_;
    bool entered_atmosphere_ = false;
    bool skipped_out_ = false;
    math::Vec3 aero_acceleration_body_mps2_;
    bool accelerometer_ = false;
    math::Vec3 rcs_force_body_n_;
};

// Earth rotation vector in GCRF from the orientation's rotation rate (omega_x = dR/dt * R^T).
math::Vec3 earth_rotation_vector(const frames::FixedToInertial& transform);

}  // namespace starsim::vehicle
