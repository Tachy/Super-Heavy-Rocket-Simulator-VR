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

struct FlightTelemetry {
    double altitude_m = 0.0;              // above WGS84
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double speed_inertial_mps = 0.0;
    double speed_relative_mps = 0.0;      // relative to the rotating Earth (what webcasts show)
    double vertical_speed_mps = 0.0;      // relative, along the local vertical
    double flight_path_angle_rad = 0.0;   // relative velocity above the local horizontal
    double mach = 0.0;
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

    bool clamped() const { return clamped_; }
    void release_clamps() { clamped_ = false; }
    bool impacted() const { return impacted_; }

    // Splits the stack below stage `index`. Returns the upper part; both keep position, velocity and
    // rotation consistent with the rigid body before the split. `separation_dv_mps` pushes them apart
    // along the axis (split by mass).
    std::unique_ptr<FlightBody> separate(size_t index, double separation_dv_mps = 0.0);

    FlightTelemetry telemetry(const FlightEnvironment& env, time::Epoch epoch) const;

    const Vehicle& vehicle() const { return *vehicle_; }
    Vehicle& vehicle() { return *vehicle_; }
    const RigidBodyState& state() const { return state_; }
    void set_state(const RigidBodyState& state) { state_ = state; }

private:
    void follow_pad(const frames::BodyOrientation& earth, time::Epoch epoch);

    std::unique_ptr<Vehicle> vehicle_;
    RigidBodyState state_;
    bool clamped_ = false;
    bool impacted_ = false;
    math::Vec3 pad_itrf_m_;
    math::Quat attitude_itrf_;  // body -> ITRF while clamped
    double cg_height_on_pad_m_ = 0.0;
};

// Earth rotation vector in GCRF from the orientation's rotation rate (omega_x = dR/dt * R^T).
math::Vec3 earth_rotation_vector(const frames::FixedToInertial& transform);

}  // namespace starsim::vehicle
