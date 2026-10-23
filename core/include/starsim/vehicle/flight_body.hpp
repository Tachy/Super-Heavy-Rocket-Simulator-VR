#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"
#include "starsim/vehicle/boil_off.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::aero {
class RadiativeHeatingTable;
}

namespace starsim::vehicle {

// Rigid-body state of the vehicle's centre of gravity, Earth-centred GCRF.
struct RigidBodyState {
    math::Vec3 position_gcrf_m;
    math::Vec3 velocity_gcrf_mps;
    math::Quat attitude;                    // body -> GCRF
    math::Vec3 angular_velocity_body_radps;
};

// The default of FlightEnvironment::ignition_settling (refuelling f3; the campaign plans with it - f3 minor M7).
inline constexpr bool ignition_settling_default = true;

// Everything outside the vehicle that acts on it. Gravity must be Earth-centred; the Earth is the reference body unless
// `moon` is given and the vehicle is near it.
struct FlightEnvironment {
    const environment::GravityModel& gravity;
    const environment::Atmosphere& atmosphere;
    const frames::BodyOrientation& earth_orientation;
    const ephemeris::Ephemeris* ephemeris = nullptr;  // needed for solar radiation pressure
    bool solar_radiation_pressure = true;
    const environment::ReferenceBody* moon = nullptr;  // lunar l1: the Moon as a reference body (nullptr: Earth only)
    const aero::RadiativeHeatingTable* radiative_heating = nullptr;  // lunar l5: Earth air radiation (nullptr: none)
    double air_density_factor = 1.0;  // lunar l5 campaigns: truth air density (Earth) over US76
    // Fidelity flag (header-tank plan): tank liquid settles perpendicular to the apparent gravity
    // (Vehicle::settle_propellant); off: always a column on the tank bottom (settled aft).
    bool propellant_settling = true;
    // Fidelity flag (refuelling f3): main-tank liquid must be settled over the outlets before an in-orbit ignition
    // (Vehicle::update_settling / set_ignition_check); off: never checked, the progress is not moved.
    bool ignition_settling = ignition_settling_default;
    // Fidelity flag (header-tank plan review): engine, grid fin and flap moments act about the full CG point (pitch /
    // yaw; roll about MassProperties::roll_offset_m) as the hull's do. Off [E]: about the axial CG - with the full point
    // the Falcon 9 / Crew Dragon ascent apoapsis moved 210 -> 223 km (header-tank plan Task 2); not yet validated on.
    bool actuator_moments_about_cg_point = false;
    // Fidelity flag (refuelling r2): cryogenic propellant in the hull tanks of stages with [stage.thermal] boils off
    // by the hull heat balance (vehicle/boil_off.hpp) in free flight above FlightBody::boil_off_min_altitude_m.
    bool boil_off = true;
    // M8: the Earth's surface (reference_body_of sets it on the Earth). nullptr: the flat model - the ground at each
    // body's ground_height_m (the old model, kept for comparison runs and the guidance planners' predictions).
    const environment::Terrain* earth_terrain = nullptr;
};

// Air data at the start of the last tick (what an air-data system would report to the controllers).
struct AirData {
    math::Vec3 air_velocity_body_mps;  // vehicle CG relative to the air
    aero::FinAirData air;
    double mean_free_path_m = 0.0;
};

struct FlightTelemetry {
    // Position and relative velocity refer to the reference body (Earth: WGS84 / ITRF; Moon: sphere / MOON_ME).
    ephemeris::Body reference_body = ephemeris::Body::Earth;
    double altitude_m = 0.0;              // above the reference body's shape
    // Above the ground below (Moon: the terrain if loaded; Earth: the Earth terrain if set, else = altitude_m). Set by
    // FlightSession::telemetry.
    double altitude_terrain_m = 0.0;
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    // Relative to the reference body's centre, non-rotating ICRF-aligned axes (Earth: |v_gcrf|; Moon: |v - v_moon|).
    double speed_inertial_mps = 0.0;
    double speed_relative_mps = 0.0;      // relative to the rotating reference body (what webcasts show)
    // Both bodies whatever the reference (external view, transfer gauges), inertial about their centres. The Moon's
    // two are NaN without a Moon in the environment (FlightEnvironment::moon == nullptr).
    double distance_earth_m = 0.0;        // from the Earth's centre
    double speed_earth_mps = 0.0;         // |v_gcrf|
    double distance_moon_m = 0.0;         // from the Moon's centre
    double speed_moon_mps = 0.0;          // |v_gcrf - v_moon_gcrf|
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
    double periapsis_altitude_m = 0.0;  // osculating orbit about the reference body, above its equatorial radius
    double apoapsis_altitude_m = 0.0;   // infinity on an open orbit
    double inclination_rad = 0.0;       // to the reference body's equator of date (body-fixed z)
};

// The reference body `id` of an environment (the Earth from its orientation and atmosphere; the Moon if given).
environment::ReferenceBody reference_body_of(const FlightEnvironment& env, ephemeris::Body id);

struct GroundClearance {
    double height_m = 0.0;  // CG above the ground of the reference body
    double sink_mps = 0.0;  // inertial velocity relative to the body's centre, towards it
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

    // Reference body (lunar l1): the Moon within moon_capture_distance_m of it, the Earth again beyond
    // moon_release_distance_m (hysteresis on the choice of the last step / coast); bodies on the ground keep theirs.
    static constexpr double moon_capture_distance_m = 60.0e6;   // [E]
    static constexpr double moon_release_distance_m = 100.0e6;  // [E] the NRHO's apolune (~70,000 km) stays inside
    ephemeris::Body reference_body(const FlightEnvironment& env, time::Epoch epoch) const;
    GroundClearance ground_clearance(const FlightEnvironment& env, time::Epoch epoch) const;

    // Unpowered flight over a longer interval with adaptive DOP853 on the same 13-element state and the
    // same forces (gravity, aerodynamics, solar radiation pressure; no thrust, no RCS). Used instead of
    // fixed ticks when nothing is commanded, e.g. for a multi-day lunar transfer. The reference body (air, drag) is the
    // one chosen at the segment's start, its frame frozen at that epoch: keep segments short where it can change (the
    // session coasts <= 600 s; a body that starts Moon-referenced must not coast into the Earth's air in one segment).
    void coast(const FlightEnvironment& env, time::Epoch epoch, double duration_s);
    bool can_coast() const;  // engines off, not clamped, not on the ground
    // In orbit (refuelling f3): the osculating periapsis about the reference body of the last step / coast (the Earth
    // before the first) lies above its equatorial radius on a closed orbit.
    bool in_orbit() const;

    bool clamped() const { return clamped_; }
    void release_clamps() { clamped_ = false; }
    // Contact wrench for the next step() only: force through the CG (GCRF) and torque (body axes).
    void apply_external_wrench(const math::Vec3& force_gcrf_n, const math::Vec3& torque_body_nm);
    // Fixes the body to the Earth where it is (caught by the tower): from now on it co-rotates with the Earth
    // like a clamped vehicle and is no longer integrated.
    void hold_fixed_to_earth(const frames::BodyOrientation& earth, time::Epoch epoch);
    // Fixes the body where it is to the body-fixed frame `frame` of `body` (generic form of hold_fixed_to_earth).
    void hold_fixed(const environment::BodyFrame& frame, ephemeris::Body body);
    bool caught() const { return caught_; }
    // Fixes a stage standing on its legs to the Earth where it is (landed): resting() from now on.
    void hold_landed(const frames::BodyOrientation& earth, time::Epoch epoch);
    void hold_landed(const environment::BodyFrame& frame, ephemeris::Body body);  // on any reference body
    // Lets a landed body go (liftoff): it is integrated again from its held pose (moving with its body), not resting.
    void release_landed();
    // Leg landing simulations set this every tick for bodies with legs: true while one owns the body's ground contact
    // (its site); elsewhere the body's own ground contact applies even with its legs out (lunar l2).
    void set_legs_on_site(bool on_site) { legs_on_site_ = on_site; }
    void mark_impacted() { impacted_ = true; }  // struck the tower or overloaded a catch pin
    bool impacted() const { return impacted_; }
    bool resting() const { return resting_ || landed_; }  // carried by the ground after a gentle touchdown / landed
    // Entry (a single stage with a heat shield): stagnation heat rate of the last tick, its integral and peak.
    // Exceeding a limit breaks the vehicle up (impacted() then true, breakup_reason() names the cause in German);
    // heat_warning() from 80 % of a limit. Heating is integrated in step() only (coast() is for vacuum arcs).
    // heat_rate_w_m2() is convective plus radiative (env.radiative_heating, Earth air only); the radiative share alone:
    // radiative_heat_rate_w_m2().
    double heat_rate_w_m2() const { return heat_rate_w_m2_; }
    double radiative_heat_rate_w_m2() const { return radiative_heat_rate_w_m2_; }
    double heat_load_j_m2() const { return heat_load_j_m2_; }
    double peak_heat_rate_w_m2() const { return peak_heat_rate_w_m2_; }
    bool heat_warning() const { return heat_warning_; }
    // The aerodynamic load exceeded the crew limit ([stage.heat_shield] crew_load_limit_g) at some point (stays set).
    bool crew_load_warning() const { return crew_load_warning_; }
    const std::string& breakup_reason() const { return breakup_reason_; }
    // Skip-out ("Abprall"): after dipping below 80 km the vehicle climbed back above 100 km (set once, stays set).
    bool skipped_out() const { return skipped_out_; }
    // Accelerometer: aerodynamic acceleration (hull, fins, flaps; body axes) at the end of the last tick. Set while the
    // entry limits are judged (single stage with a heat shield) or the accelerometer is on, zero otherwise.
    const math::Vec3& aero_acceleration_body_mps2() const { return aero_acceleration_body_mps2_; }
    // Non-gravitational acceleration at the start of the last free-flight tick (thrust, aero, RCS, contact), body axes.
    const math::Vec3& specific_force_body_mps2() const { return specific_force_body_mps2_; }
    // Guidance that reads the accelerometer turns it on (one more force evaluation per tick).
    void set_accelerometer(bool on) { accelerometer_ = on; }
    const AirData& air_data() const { return air_data_; }
    // Air velocity (body frame) of a point on the axis at `station_m`: the CG's plus omega x r (last tick's air data).
    math::Vec3 point_air_velocity_body_mps(double station_m) const;

    // Flat ground at this height above WGS84 when the reference body is the Earth without terrain (the pad elevation);
    // with the Earth terrain (FlightEnvironment::earth_terrain) and at the Moon the ground is the body's terrain (the
    // shape without one; lunar l2, M8).
    double ground_height_m() const { return ground_height_m_; }
    // Ground below a body-fixed point: the terrain (or the bare shape) of the body; the Earth without terrain: the flat
    // ground_height_m.
    double ground_height_at(const environment::ReferenceBody& body, const math::Vec3& fixed_m) const {
        return body.id() == ephemeris::Body::Earth && !body.terrain() ? ground_height_m_ : body.surface_height_m(fixed_m);
    }
    void set_ground_height(double height_m) { ground_height_m_ = height_m; }
    // Sink rate (m/s, relative to the rotating Earth) at the first ground / water contact.
    std::optional<double> touchdown_sink_mps() const { return touchdown_sink_mps_; }

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

    // Boil-off (refuelling r2): the heat balance is evaluated every boil_off_interval_s of free flight and after every
    // coast; the vented mass is the trapezoid of the last two evaluations' flows per tank. Below
    // boil_off_min_altitude_m in air (convection and entry heating are not modelled), clamped, caught, landed, resting,
    // docked or carrying a docked visitor nothing boils off here: a docked pair is vented by the session in the
    // compound's environment (refuelling r4, FlightSession::set_tank_mass keeps the compound CG).
    static constexpr double boil_off_interval_s = 10.0;         // [E]
    static constexpr double boil_off_min_altitude_m = 100.0e3;  // [E]
    const BoilOffRate& boil_off_rate() const { return boil_off_rate_; }  // at the last evaluation
    void set_boil_off_rate(const BoilOffRate& rate) { boil_off_rate_ = rate; }  // the session's docked evaluation
    // The rates now in `liquid` (not applied; empty with the flag off or without an ephemeris).
    std::vector<TankBoilOff> boil_off_rates_now(const FlightEnvironment& env, time::Epoch epoch,
                                                const LiquidEnvironment& liquid) const;
    // The specific force of the last tick or at the end of the last coast (moves the liquid).
    const math::Vec3& liquid_specific_force_body_mps2() const { return boil_off_specific_force_body_mps2_; }

    // A station in ideal attitude (i1): the body axes follow LVLH (times body_in_lvlh) exactly after every step and
    // coast; contact torques do not turn it (its CMGs are taken as strong enough - documented simplification). The
    // translation stays free. The attitude enters no force of such a body (fixed-area drag and SRP).
    void set_ideal_lvlh_attitude(std::optional<math::Quat> body_in_lvlh) {
        ideal_lvlh_ = body_in_lvlh;
        snap_ideal_attitude();
    }
    bool ideal_attitude() const { return ideal_lvlh_.has_value(); }

    // Docked (ISS i5): held on a station by the session (sim/docking_sim) - not integrated in step() / coast(), no
    // thrust, no RCS, no propellant use; the session sets its state from the station every tick and after every coast.
    void hold_docked() {
        docked_ = true;
        forget_boil_off();
        vehicle_->stop_rcs();
        rcs_force_body_n_ = {};
        external_force_gcrf_n_ = {};
        external_torque_body_nm_ = {};
    }
    void release_docked() {
        docked_ = false;
        forget_boil_off();
    }
    bool docked() const { return docked_; }

private:
    void snap_ideal_attitude();
    std::optional<math::Quat> ideal_lvlh_;
    void follow_pad(const environment::BodyFrame& frame);
    void resolve_ground_contact(const environment::ReferenceBody& body, const environment::BodyFrame& frame);
    void update_air_data(const environment::ReferenceBody& body, const environment::BodyFrame& frame);
    void judge_parachute_loads();

    std::unique_ptr<Vehicle> vehicle_;
    RigidBodyState state_;
    bool clamped_ = false;
    bool impacted_ = false;
    bool resting_ = false;
    std::optional<double> touchdown_sink_mps_;
    AirData air_data_;
    math::Vec3 pad_fixed_m_;    // bottom point in the body-fixed frame of held_on_ while clamped/caught/landed/resting
    math::Quat attitude_fixed_;  // body -> body-fixed axes of held_on_
    // Chosen at the last step / coast; undecided (a body just built or placed): the Moon within the release distance.
    std::optional<ephemeris::Body> reference_;
    // The reference body's centre, GM and equatorial radius at the last step / coast (in_orbit); the Earth before.
    math::StateVector orbit_centre_;
    double orbit_gm_m3ps2_ = constants::earth_gm_m3ps2;
    double orbit_radius_m_ = constants::wgs84_semi_major_axis_m;
    void track_reference(const environment::ReferenceBody& body, const environment::BodyFrame& frame);
    LiquidEnvironment liquid_now() const;
    bool wreck_on_ground_ = false;  // struck the ground: the wreck stays where it struck (pad_fixed_m_ on held_on_)
    void hold_wreck(const environment::BodyFrame& frame, ephemeris::Body body);
    ephemeris::Body held_on_ = ephemeris::Body::Earth;    // the body pad_fixed_m_ / attitude_fixed_ refer to
    double ground_height_m_ = 0.0;
    math::Vec3 external_force_gcrf_n_;
    math::Vec3 external_torque_body_nm_;
    bool caught_ = false;
    bool docked_ = false;
    bool landed_ = false;  // standing on its legs, held to its body (hold_landed)
    bool legs_on_site_ = false;  // set every tick by the leg site for bodies on it
    double heat_rate_w_m2_ = 0.0;
    double radiative_heat_rate_w_m2_ = 0.0;
    double heat_load_j_m2_ = 0.0;
    double peak_heat_rate_w_m2_ = 0.0;
    bool heat_warning_ = false;
    bool crew_load_warning_ = false;
    std::string breakup_reason_;
    bool entered_atmosphere_ = false;
    bool skipped_out_ = false;
    math::Vec3 aero_acceleration_body_mps2_;
    math::Vec3 specific_force_body_mps2_;
    bool accelerometer_ = false;
    math::Vec3 rcs_force_body_n_;
    void boil_off(const FlightEnvironment& env, time::Epoch epoch, double dt_s);
    void reset_boil_off() {
        forget_boil_off();
        boil_off_rate_ = {};
    }
    // A hold (docked, caught, landed, resting) ends the trapezoid: the first evaluation after it starts afresh instead
    // of averaging with the flow from before the hold (r2 deferred minor). The last rate stays for the display.
    void forget_boil_off() {
        boil_off_elapsed_s_ = 0.0;
        boil_off_last_.clear();
    }
    double boil_off_elapsed_s_ = 0.0;
    math::Vec3 boil_off_specific_force_body_mps2_;  // of the last tick or at the end of the last coast
    std::vector<TankBoilOff> boil_off_last_;
    BoilOffRate boil_off_rate_;
};

// Earth rotation vector in GCRF from the orientation's rotation rate (omega_x = dR/dt * R^T).
math::Vec3 earth_rotation_vector(const frames::FixedToInertial& transform);

}  // namespace starsim::vehicle
