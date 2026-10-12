#pragma once

#include <string>

#include "starsim/math/vec3.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

// Target orbit for the ascent: insertion at periapsis (flight-path angle 0).
struct AscentTarget {
    double periapsis_altitude_m = 200e3;
    double apoapsis_altitude_m = 200e3;
    double inclination_rad = 0.0;
    bool northerly = true;  // launch towards the ascending (true) or descending node
};

// Open-loop first-stage profile.
struct PitchProgram {
    double vertical_rise_speed_mps = 60.0;  // clear the tower before turning
    // Initial tilt from vertical towards the launch azimuth. 10 deg reproduces the IFT-3 flight-path angle
    // over the first 90 s (tuning test "[.tuning]"); later the faster V3 turns over more slowly.
    double kick_angle_rad = 10.0 * 3.14159265358979 / 180.0;
    double kick_duration_s = 12.0;
};

enum class AscentPhase { Vertical, PitchKick, GravityTurn, ClosedLoop, Cutoff };

const char* to_string(AscentPhase phase);

struct GuidanceCommand {
    math::Vec3 thrust_direction_gcrf;  // where the body x-axis should point
    AscentPhase phase = AscentPhase::Vertical;
    double time_to_go_s = 0.0;         // closed loop only
    bool cutoff = false;
};

// Inertial launch azimuth (clockwise from north) for a target inclination from a given latitude,
// corrected for the Earth's rotation (Vallado, "Fundamentals of Astrodynamics", sec. 6.4).
double launch_azimuth_rad(double latitude_rad, double inclination_rad, double orbital_speed_mps, bool northerly);

// Ascent guidance: vertical rise, pitch kick, gravity turn (zero angle of attack) on the booster;
// closed-loop Powered Explicit Guidance (linear-tangent steering, Space Shuttle PEG as formulated on
// OrbiterWiki "Powered Explicit Guidance") on the upper stage, with out-of-plane correction towards the
// target orbital plane fixed at liftoff.
class AscentGuidance {
public:
    AscentGuidance(AscentTarget target, PitchProgram program, double gm_earth_m3ps2);

    // Must be called once at liftoff (fixes launch azimuth and target plane).
    void start(const vehicle::FlightBody& body, const frames::BodyOrientation& earth, time::Epoch epoch);

    // `closed_loop`: the upper stage is flying (PEG); otherwise the open-loop booster profile.
    GuidanceCommand update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch,
                           bool closed_loop);

    double launch_azimuth_rad() const { return azimuth_rad_; }
    const math::Vec3& target_plane_normal_gcrf() const { return plane_normal_; }
    double target_radius_m() const { return target_radius_m_; }
    double target_speed_mps() const { return target_speed_mps_; }

private:
    GuidanceCommand open_loop(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);
    GuidanceCommand peg(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);

    AscentTarget target_;
    PitchProgram program_;
    double gm_;
    double azimuth_rad_ = 0.0;
    math::Vec3 plane_normal_;
    double target_radius_m_ = 0.0;
    double target_speed_mps_ = 0.0;
    double kick_start_s_ = -1.0;
    time::Epoch liftoff_;
    // PEG state
    double peg_a_ = 0.0;
    double peg_b_ = 0.0;
    double peg_t_ = 0.0;  // time to go
    bool peg_converged_ = false;
    time::Epoch peg_last_;
};

}  // namespace starsim::gnc
