#pragma once

#include <optional>
#include <utility>
#include <vector>

#include <string>

#include "starsim/gnc/peg_steering.hpp"
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
    // The first stage's dynamic-pressure limit over Mach ({mach, pa}, increasing Mach; [stage] max_dynamic_pressure_pa
    // of the vehicle's lowest stage, filled in by FlightSession::engage_ascent_autopilot when empty): the booster
    // throttles to hold q at or below it (empty: no limit). The upper stage always flies full (PEG).
    std::vector<std::pair<double, double>> max_dynamic_pressure_by_mach;
    // The booster's operational ascent throttle ([stage] max_ascent_throttle of the lowest stage, filled in like the
    // q limit when 0): the open loop never commands more; the q limit acts below it. 0 or 1: full thrust.
    double max_ascent_throttle = 0.0;
};

// The booster's ascent throttle cap of a program (max_ascent_throttle; 1 when unset).
double ascent_throttle_cap(const PitchProgram& program);

// The program's dynamic-pressure limit at `mach`: linear between its points, held beyond the ends; 0 without a limit.
double dynamic_pressure_limit_pa(const PitchProgram& program, double mach);

// [E] Time constant of the max-Q limiter: q is brought to the limit as dq/dt = (limit - q) / tau. 1 s: short against
// q's own build-up near max-Q (q / (dq/dt) ~ 20-30 s), so the full thrust holds until ~95 % of the limit; long enough
// that the commanded throttle changes by ~0.1-0.2 /s there, well inside the engines' throttle rate (Raptor 3 and
// Merlin 1D: 0.5 /s [E]) - the rate-limited engines follow the command without the lag that would let q overshoot.
inline constexpr double max_q_time_constant_s = 1.0;

// Max-Q limiter law (feedback linearisation of q = rho v^2 / 2 along the path): the thrust acceleration along the air
// velocity that makes dq/dt = rho v dv/dt + v^2/2 drho/dt equal (limit - q) / max_q_time_constant_s, with
// dv/dt = thrust acceleration + `other_acceleration_mps2` (drag, gravity and the frame's terms along the path).
// +infinity where q cannot be limited by thrust (no limit, no air, no speed).
double max_q_thrust_acceleration_mps2(double limit_pa, double density_kg_m3, double density_rate_kgpm3ps,
                                      double air_speed_mps, double other_acceleration_mps2);

// The throttle at which the lowest stage's engines producing thrust (spool included) deliver `thrust_n` at
// `ambient_pressure_pa`, within their common throttle range (the highest minimum, the lowest maximum). Their maximum
// for no engine.
double booster_throttle_for_thrust(const vehicle::Vehicle& vehicle, double thrust_n, double ambient_pressure_pa);

enum class AscentPhase { Vertical, PitchKick, GravityTurn, ClosedLoop, Cutoff };

const char* to_string(AscentPhase phase);

struct GuidanceCommand {
    math::Vec3 thrust_direction_gcrf;  // where the body x-axis should point
    AscentPhase phase = AscentPhase::Vertical;
    double time_to_go_s = 0.0;         // closed loop only
    bool cutoff = false;
    double throttle = 1.0;             // open loop: the max-Q limiter (booster_throttle); closed loop: full
};

// Inertial launch azimuth (clockwise from north) for a target inclination from a given latitude,
// corrected for the Earth's rotation (Vallado, "Fundamentals of Astrodynamics", sec. 6.4).
double launch_azimuth_rad(double latitude_rad, double inclination_rad, double orbital_speed_mps, bool northerly);

// The inertial target plane AscentGuidance fixes at liftoff: through the Earth's centre, containing the site (local
// geodetic up) and the inertial launch direction of `inclination_rad` (northerly: towards the ascending node).
math::Vec3 ascent_plane_normal(const math::Vec3& position_gcrf_m, const frames::BodyOrientation& earth,
                               time::Epoch epoch, double inclination_rad, bool northerly);

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
    // The upper stage's cutoff as the closed loop predicts it (onboard prediction for the mission timeline): the epoch
    // of its last update plus PEG's time to go; none before the closed loop has run with thrust.
    const std::optional<time::Epoch>& cutoff_epoch() const { return cutoff_epoch_; }
    // The booster's throttle now: the operational cap (ascent_throttle_cap) while the dynamic pressure is below
    // PitchProgram::max_dynamic_pressure_by_mach at the Mach number now, throttled to hold it at the limit (max_q_thrust_acceleration_mps2 on the air
    // data and accelerometer of the last tick; the engines' minimum respected).
    double booster_throttle(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                            time::Epoch epoch) const;
    const PitchProgram& program() const { return program_; }
    // The open loop's pitch kick began here (none: still rising vertically, or not started).
    std::optional<time::Epoch> kick_start_epoch() const {
        return kick_start_s_ < 0.0 ? std::nullopt : std::optional<time::Epoch>(liftoff_ + kick_start_s_);
    }

private:
    double max_q_throttle(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch) const;
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
    // J2-aware insertion into a circular target (gnc/j2_insertion): radius, radial and horizontal speed at the predicted
    // cutoff point, re-solved when that point moves by more than j2_resolve_angle; inactive: the Kepler target.
    bool insertion_active_ = false;
    double insertion_u_rad_ = 0.0;
    double insertion_radius_m_ = 0.0;
    double insertion_radial_mps_ = 0.0;
    double insertion_horizontal_mps_ = 0.0;
    void update_insertion(const math::Vec3& r_hat, double r, double vt, double t_go, const vehicle::FlightEnvironment& env,
                          time::Epoch epoch);
    PegSteering peg_;
    std::optional<time::Epoch> cutoff_epoch_;
};

}  // namespace starsim::gnc
