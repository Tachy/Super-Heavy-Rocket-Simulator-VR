#include "starsim/vehicle/flight_body.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "starsim/aero/entry_heating.hpp"
#include "starsim/aero/parachute.hpp"
#include "starsim/aero/radiative_heating.hpp"
#include "starsim/environment/eclipse.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/frames/lvlh.hpp"
#include "starsim/integrators/integrate.hpp"
#include "starsim/integrators/rk4.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"

namespace starsim::vehicle {

using math::Quat;
using math::Vec3;

namespace {

constexpr double solar_pressure_1au_pa = 1361.0 / 299792458.0;  // solar constant / c
constexpr double astronomical_unit_m = 149597870700.0;
constexpr double earth_mean_radius_m = 6371000.0;

using State13 = integrators::State<13>;

State13 pack(const RigidBodyState& s) {
    return {{s.position_gcrf_m.x, s.position_gcrf_m.y, s.position_gcrf_m.z, s.velocity_gcrf_mps.x,
             s.velocity_gcrf_mps.y, s.velocity_gcrf_mps.z, s.attitude.w, s.attitude.x, s.attitude.y, s.attitude.z,
             s.angular_velocity_body_radps.x, s.angular_velocity_body_radps.y, s.angular_velocity_body_radps.z}};
}

RigidBodyState unpack(const State13& y) {
    return {{y[0], y[1], y[2]},
            {y[3], y[4], y[5]},
            Quat{y[6], y[7], y[8], y[9]}.normalized(),
            {y[10], y[11], y[12]}};
}

// Moment of `force` acting on the axis-relative `point_m` (station, y, z): pitch and yaw about the full CG, roll about the
// lateral offset of the parts that turn with the hull (settled liquid does not: MassProperties::roll_offset_m).
Vec3 moment_about_cg(const MassProperties& mass, const Vec3& point_m, const Vec3& force) {
    Vec3 moment = cross(point_m - mass.cg_point_m(), force);
    moment.x = (point_m.y - mass.roll_offset_m.y) * force.z - (point_m.z - mass.roll_offset_m.z) * force.y;
    return moment;
}

Vec3 local_up_fixed(const frames::GeodeticPosition& g) {
    return {std::cos(g.latitude_rad) * std::cos(g.longitude_rad), std::cos(g.latitude_rad) * std::sin(g.longitude_rad),
            std::sin(g.latitude_rad)};
}

// Quantities that are constant during one tick.
struct TickContext {
    const FlightEnvironment& env;
    const Vehicle& vehicle;
    time::Epoch epoch;
    const environment::ReferenceBody& body;  // reference body of the tick
    environment::BodyFrame frame;            // its frame at the start of the tick (air only: the Moon is airless)
    std::vector<std::vector<double>> tank_flows;
    Vec3 srp_acceleration_gcrf;  // evaluated at the start of the tick
    Vec3 rcs_torque_nm;          // commanded reaction-control couple, body axes
    Vec3 rcs_force_n;            // commanded reaction-control translation force, body axes
    Vec3 external_force_gcrf_n;   // contact (tower arms), through the CG
    Vec3 external_torque_body_nm;
    Vec3* first_specific_force_body = nullptr;  // out: the specific force of the first RK4 stage (tau = 0)
};

// Moment of an engine, grid fin or flap force; `arm_m` runs from the axial CG (station, 0, 0) to the point of action.
Vec3 actuator_moment(const TickContext& ctx, const MassProperties& mass, const Vec3& arm_m, const Vec3& force) {
    if (!ctx.env.actuator_moments_about_cg_point) return cross(arm_m, force);
    Vec3 moment = cross(arm_m - Vec3{0.0, mass.cg_offset_m.y, mass.cg_offset_m.z}, force);
    moment.x = (arm_m.y - mass.roll_offset_m.y) * force.z - (arm_m.z - mass.roll_offset_m.z) * force.y;
    return moment;
}

struct Forces {
    Vec3 acceleration_gcrf;        // total
    Vec3 nongravitational_body_n;  // thrust + aero, body axes
    Vec3 aero_body_n;              // aerodynamic part (hull, grid fins, flaps), body axes
    Vec3 moment_body_nm;
    double thrust_n = 0.0;
    aero::AeroResult aero;
    MassProperties mass;
};

Forces evaluate(const TickContext& ctx, double tau_s, const RigidBodyState& s) {
    Forces f;
    f.mass = ctx.vehicle.mass_properties(&ctx.tank_flows, tau_s);
    const Vec3 r_fixed = ctx.frame.to_fixed(s.position_gcrf_m);
    const auto geodetic = frames::fixed_to_geodetic(r_fixed, ctx.body.shape());
    const auto air = ctx.body.air(geodetic.height_m);

    // Aerodynamics relative to the co-rotating atmosphere (no wind model yet).
    const Vec3 air_velocity = ctx.frame.relative_velocity(s.position_gcrf_m, s.velocity_gcrf_mps);
    const bool powered = ctx.vehicle.any_engine_producing_thrust();
    f.aero = ctx.vehicle.hull_aero(s.attitude.inverse_rotate(air_velocity), air, powered);
    f.aero.force_body_n = f.aero.force_body_n * ctx.vehicle.hull_aero_scale();
    f.aero.moment_body_nm = f.aero.moment_body_nm * ctx.vehicle.hull_aero_scale();
    f.nongravitational_body_n = f.aero.force_body_n;
    f.aero_body_n = f.aero.force_body_n;
    // About the CG, which sits off the axis by cg_offset_m on a capsule or with settled liquid (roll: roll_offset_m).
    f.moment_body_nm = moment_about_cg(f.mass, {f.aero.normal_force_station_m, 0.0, 0.0}, f.aero.force_body_n) +
                       f.aero.moment_body_nm;

    for (const auto& stage : ctx.vehicle.stages()) {
        for (const auto& e : stage.engines) {
            if (!e.producing_thrust()) continue;
            const double thrust =
                ctx.vehicle.thrust_factor() * e.spool * e.performance->thrust_n(e.throttle, air.pressure_pa);
            const Vec3 force = thrust * e.thrust_direction();
            f.thrust_n += thrust;
            f.nongravitational_body_n += force;
            // About the axial CG unless FlightEnvironment::actuator_moments_about_cg_point [E].
            f.moment_body_nm += actuator_moment(ctx, f.mass, e.gimbal_point_m - Vec3{f.mass.cg_station_m, 0.0, 0.0}, force);
        }
    }
    // Grid fins: local flow at each fin including the rotation (gives aerodynamic damping).
    const Vec3 air_velocity_body = s.attitude.inverse_rotate(air_velocity);
    const aero::FinAirData fin_air{air.density_kg_m3, air.speed_of_sound_mps, f.aero.rarefaction};
    for (const auto& stage : ctx.vehicle.stages()) {
        for (const auto& fin : stage.grid_fins) {
            const Vec3 arm = Vec3{fin.geometry.station_m - f.mass.cg_station_m, 0.0, 0.0} +
                             aero::grid_fin_radial(fin.geometry) * fin.geometry.radius_m;
            const Vec3 fin_velocity = air_velocity_body + cross(s.angular_velocity_body_radps, arm);
            const auto force = aero::grid_fin_force(fin.geometry, fin.deflection_rad, fin_velocity, fin_air,
                                                    *stage.grid_fin_table);
            f.nongravitational_body_n += force.force_body_n;
            f.aero_body_n += force.force_body_n;
            f.moment_body_nm += actuator_moment(ctx, f.mass, arm, force.force_body_n);
        }
    }
    // Flaps: local flow at each flap centroid including the rotation.
    for (const auto& stage : ctx.vehicle.stages()) {
        if (!stage.flap_table) continue;
        for (const auto& flap : stage.flaps) {
            const Vec3 arm =
                aero::flap_centroid(flap.geometry, flap.deflection_rad) - Vec3{f.mass.cg_station_m, 0.0, 0.0};
            const Vec3 flap_velocity = air_velocity_body + cross(s.angular_velocity_body_radps, arm);
            const auto force =
                aero::flap_force(flap.geometry, flap.deflection_rad, flap_velocity, fin_air, *stage.flap_table);
            f.nongravitational_body_n += force.force_body_n;
            f.aero_body_n += force.force_body_n;
            f.moment_body_nm += actuator_moment(ctx, f.mass, arm, force.force_body_n);
        }
    }
    // Parachutes: drag of the riser point's local flow (incl. omega x r: the hanging capsule's damping), massless canopies.
    for (const auto& stage : ctx.vehicle.stages()) {
        for (const auto& chute : stage.parachutes) {
            if (chute.state != ParachuteState::Open || chute.drag_area_m2 <= 0.0) continue;
            const Vec3 arm{chute.attach_station_m - f.mass.cg_station_m, -f.mass.cg_offset_m.y, -f.mass.cg_offset_m.z};
            const Vec3 point_velocity = air_velocity_body + cross(s.angular_velocity_body_radps, arm);
            const Vec3 force = aero::parachute_force_body_n(chute.drag_area_m2, point_velocity, air.density_kg_m3);
            f.nongravitational_body_n += force;
            f.aero_body_n += force;
            f.moment_body_nm += moment_about_cg(f.mass, {chute.attach_station_m, 0.0, 0.0}, force);
        }
    }
    f.moment_body_nm += ctx.rcs_torque_nm;
    f.nongravitational_body_n += ctx.rcs_force_n;
    f.moment_body_nm += ctx.external_torque_body_nm;
    f.acceleration_gcrf = ctx.env.gravity.acceleration_mps2(s.position_gcrf_m, ctx.epoch + tau_s) +
                          s.attitude.rotate(f.nongravitational_body_n) / f.mass.mass_kg + ctx.srp_acceleration_gcrf +
                          ctx.external_force_gcrf_n / f.mass.mass_kg;
    return f;
}

State13 derivative(const TickContext& ctx, double tau_s, const State13& y) {
    const RigidBodyState s = unpack(y);
    const Forces f = evaluate(ctx, tau_s, s);
    if (tau_s == 0.0 && ctx.first_specific_force_body) {
        *ctx.first_specific_force_body =
            (f.nongravitational_body_n + s.attitude.inverse_rotate(ctx.external_force_gcrf_n)) / f.mass.mass_kg;
    }
    const Vec3& w = s.angular_velocity_body_radps;
    const Vec3& inertia = f.mass.inertia_kg_m2;
    const Vec3 h{inertia.x * w.x, inertia.y * w.y, inertia.z * w.z};
    const Vec3 torque = f.moment_body_nm - cross(w, h);
    const Vec3 w_dot{inertia.x > 0.0 ? torque.x / inertia.x : 0.0, torque.y / inertia.y, torque.z / inertia.z};
    const Quat q_dot = math::attitude_rate(s.attitude, w);
    return {{s.velocity_gcrf_mps.x, s.velocity_gcrf_mps.y, s.velocity_gcrf_mps.z, f.acceleration_gcrf.x,
             f.acceleration_gcrf.y, f.acceleration_gcrf.z, q_dot.w, q_dot.x, q_dot.y, q_dot.z, w_dot.x, w_dot.y,
             w_dot.z}};
}

// Cannonball-cylinder solar radiation pressure with Earth and Moon shadow.
Vec3 solar_radiation_acceleration(const FlightEnvironment& env, const Vehicle& vehicle, const RigidBodyState& s,
                                  double mass_kg, time::Epoch epoch) {
    if (!env.solar_radiation_pressure || env.ephemeris == nullptr) {
        return {};
    }
    const Vec3 to_sun = env.ephemeris->state_gcrf(ephemeris::Body::Sun, epoch).position_m - s.position_gcrf_m;
    const Vec3 to_moon = env.ephemeris->state_gcrf(ephemeris::Body::Moon, epoch).position_m - s.position_gcrf_m;
    const double earth_shadow = environment::occulted_fraction(to_sun, constants::sun_radius_m, -s.position_gcrf_m,
                                                               earth_mean_radius_m);
    const double moon_shadow = environment::occulted_fraction(to_sun, constants::sun_radius_m, to_moon,
                                                              constants::moon_mean_radius_m);
    const double sunlit = (1.0 - earth_shadow) * (1.0 - moon_shadow);
    if (sunlit <= 0.0) {
        return {};
    }
    const double distance = norm(to_sun);
    const Vec3 sun_dir = to_sun / distance;
    const Vec3 sun_body = s.attitude.inverse_rotate(sun_dir);
    const double radius = 0.5 * vehicle.diameter_m();
    // A fixed solar area (a station's sun-tracking arrays, attitude-independent [E]), else the cylinder seen from the Sun.
    const double solar_area_m2 = vehicle.stages().front().config.aero.solar_area_m2;
    const double area =
        solar_area_m2 > 0.0
            ? solar_area_m2
            : std::abs(sun_body.x) * constants::pi * radius * radius +
                  std::sqrt(std::max(0.0, 1.0 - sun_body.x * sun_body.x)) * vehicle.length_m() * vehicle.diameter_m();
    double cr = 0.0;  // mass-weighted is overkill: use the stack average
    for (const auto& stage : vehicle.stages()) cr += stage.config.radiation_pressure_coefficient;
    cr /= static_cast<double>(vehicle.stages().size());
    const double pressure = solar_pressure_1au_pa * (astronomical_unit_m / distance) * (astronomical_unit_m / distance);
    return sun_dir * (-pressure * cr * area * sunlit / mass_kg);
}

}  // namespace

Vec3 earth_rotation_vector(const frames::FixedToInertial& transform) { return frames::rotation_vector(transform); }

environment::ReferenceBody reference_body_of(const FlightEnvironment& env, ephemeris::Body id) {
    if (id == ephemeris::Body::Moon && env.moon) return *env.moon;
    auto earth = environment::ReferenceBody::earth(
        env.earth_orientation, env.atmosphere,
        env.ephemeris ? env.ephemeris->gm_m3ps2(ephemeris::Body::Earth) : constants::earth_gm_m3ps2);
    earth.set_air_density_factor(env.air_density_factor);
    return earth;
}

ephemeris::Body FlightBody::reference_body(const FlightEnvironment& env, time::Epoch epoch) const {
    if (clamped_ || caught_ || landed_ || resting_ || wreck_on_ground_) return held_on_;
    if (env.moon == nullptr) return ephemeris::Body::Earth;
    const double distance = norm(state_.position_gcrf_m - env.moon->centre_gcrf(epoch).position_m);
    // Undecided (just built or placed, e.g. an NRHO near its apolune): as if it came from the Moon.
    const bool from_moon = !reference_ || *reference_ == ephemeris::Body::Moon;
    const double limit = from_moon ? moon_release_distance_m : moon_capture_distance_m;
    return distance < limit ? ephemeris::Body::Moon : ephemeris::Body::Earth;
}

GroundClearance FlightBody::ground_clearance(const FlightEnvironment& env, time::Epoch epoch) const {
    const auto id = reference_body(env, epoch);
    const auto body = reference_body_of(env, id);
    const auto frame = body.frame(epoch);
    const auto centred = frame.centred({state_.position_gcrf_m, state_.velocity_gcrf_mps});
    const Vec3 fixed = frame.to_fixed(state_.position_gcrf_m);
    const double height = frames::fixed_to_geodetic(fixed, body.shape()).height_m - ground_height_at(body, fixed);
    return {height, -dot(centred.velocity_mps, math::normalized(centred.position_m))};
}

FlightBody::FlightBody(std::unique_ptr<Vehicle> vehicle, const RigidBodyState& state)
    : vehicle_(std::move(vehicle)), state_(state) {}

std::unique_ptr<FlightBody> FlightBody::on_pad(std::unique_ptr<Vehicle> vehicle, double latitude_rad,
                                               double longitude_rad, double height_m, double heading_rad,
                                               const frames::BodyOrientation& earth, time::Epoch epoch) {
    const frames::GeodeticPosition pad{latitude_rad, longitude_rad, height_m};
    const Vec3 up = local_up_fixed(pad);
    const Vec3 east{-std::sin(longitude_rad), std::cos(longitude_rad), 0.0};
    const Vec3 north = cross(up, east);
    const Vec3 body_y = std::cos(heading_rad) * north + std::sin(heading_rad) * east;
    const Vec3 body_z = cross(up, body_y);
    math::Mat3 columns;
    for (int i = 0; i < 3; ++i) {
        columns(i, 0) = (i == 0 ? up.x : i == 1 ? up.y : up.z);
        columns(i, 1) = (i == 0 ? body_y.x : i == 1 ? body_y.y : body_y.z);
        columns(i, 2) = (i == 0 ? body_z.x : i == 1 ? body_z.y : body_z.z);
    }
    auto body = std::make_unique<FlightBody>(std::move(vehicle), RigidBodyState{});
    body->clamped_ = true;
    body->ground_height_m_ = height_m;
    body->pad_fixed_m_ = frames::geodetic_to_fixed(pad);
    body->attitude_fixed_ = Quat::from_matrix(columns);
    body->held_on_ = ephemeris::Body::Earth;
    body->follow_pad(environment::BodyFrame::centred_at_origin(earth.fixed_to_gcrf(epoch)));
    return body;
}

void FlightBody::follow_pad(const environment::BodyFrame& frame) {
    const double cg = vehicle_->mass_properties().cg_station_m;
    const Vec3 cg_fixed = pad_fixed_m_ + attitude_fixed_.rotate(Vec3{cg, 0.0, 0.0});
    state_.position_gcrf_m = frame.position_gcrf(cg_fixed);
    state_.velocity_gcrf_mps = frame.velocity_gcrf(cg_fixed, {});
    state_.attitude = (Quat::from_matrix(frame.transform.rotation) * attitude_fixed_).normalized();
    state_.angular_velocity_body_radps = state_.attitude.inverse_rotate(frame.rotation_gcrf_radps);
}

void FlightBody::apply_external_wrench(const Vec3& force_gcrf_n, const Vec3& torque_body_nm) {
    external_force_gcrf_n_ += force_gcrf_n;
    external_torque_body_nm_ += torque_body_nm;
}

void FlightBody::hold_fixed_to_earth(const frames::BodyOrientation& earth, time::Epoch epoch) {
    hold_fixed(environment::BodyFrame::centred_at_origin(earth.fixed_to_gcrf(epoch)), ephemeris::Body::Earth);
}

// A ground impact: the wreck stays where it struck, on its body (lunar l1 review: a frozen GCRF state left a wreck on
// the Moon behind at ~1 km/s).
void FlightBody::hold_wreck(const environment::BodyFrame& frame, ephemeris::Body body) {
    const double cg = vehicle_->mass_properties().cg_station_m;
    pad_fixed_m_ = frame.to_fixed(state_.position_gcrf_m - state_.attitude.rotate(Vec3{cg, 0.0, 0.0}));
    attitude_fixed_ = (Quat::from_matrix(transpose(frame.transform.rotation)) * state_.attitude).normalized();
    held_on_ = body;
    reference_ = body;
    impacted_ = true;
    wreck_on_ground_ = true;
}

void FlightBody::hold_fixed(const environment::BodyFrame& frame, ephemeris::Body body) {
    const double cg = vehicle_->mass_properties().cg_station_m;
    pad_fixed_m_ = frame.to_fixed(state_.position_gcrf_m - state_.attitude.rotate(Vec3{cg, 0.0, 0.0}));
    attitude_fixed_ = (Quat::from_matrix(transpose(frame.transform.rotation)) * state_.attitude).normalized();
    held_on_ = body;
    reference_ = body;
    caught_ = true;
    resting_ = false;
    follow_pad(frame);
}

void FlightBody::hold_landed(const frames::BodyOrientation& earth, time::Epoch epoch) {
    hold_landed(environment::BodyFrame::centred_at_origin(earth.fixed_to_gcrf(epoch)), ephemeris::Body::Earth);
}

void FlightBody::hold_landed(const environment::BodyFrame& frame, ephemeris::Body body) {
    hold_fixed(frame, body);
    caught_ = false;
    landed_ = true;
}

// The held pose (follow_pad) already moves with the body: the integration starts from it without a jump.
void FlightBody::release_landed() {
    landed_ = false;
    resting_ = false;
}

void FlightBody::snap_ideal_attitude() {
    if (!ideal_lvlh_) return;
    // LVLH about the GCRF origin: a station is Earth-referenced (the Moon's LVLH is not needed before a lunar station)
    const math::StateVector sv{state_.position_gcrf_m, state_.velocity_gcrf_mps};
    state_.attitude = frames::lvlh_to_inertial(sv) * *ideal_lvlh_;
    state_.angular_velocity_body_radps = state_.attitude.inverse_rotate(frames::lvlh_rate_inertial_radps(sv));
}

void FlightBody::step(const FlightEnvironment& env, time::Epoch epoch, double dt_s, const VehicleCommand& command) {
    if (impacted_) {
        vehicle_->stop_rcs();
        rcs_force_body_n_ = {};
        if (wreck_on_ground_) follow_pad(reference_body_of(env, held_on_).frame(epoch + dt_s));
        return;
    }
    reference_ = reference_body(env, epoch);
    const auto body = reference_body_of(env, *reference_);
    const auto frame = body.frame(epoch);
    const auto geodetic = frames::fixed_to_geodetic(frame.to_fixed(state_.position_gcrf_m), body.shape());
    vehicle_->update_engines(dt_s, command, body.air(geodetic.height_m).pressure_pa);
    vehicle_->update_grid_fins(dt_s, command);
    vehicle_->update_landing_legs(dt_s);
    vehicle_->update_flaps(dt_s, command);
    vehicle_->update_parachutes(dt_s);
    // Held, caught or landed: the thrusters stay off.
    vehicle_->update_rcs(dt_s, clamped_ || caught_ || landed_ ? VehicleCommand{} : command);
    rcs_force_body_n_ = vehicle_->rcs_force_n();
    update_air_data(body, frame);
    judge_parachute_loads();

    if (clamped_ || caught_ || landed_) {
        vehicle_->consume_propellant(dt_s);
        follow_pad(reference_body_of(env, held_on_).frame(epoch + dt_s));
        external_force_gcrf_n_ = {};
        external_torque_body_nm_ = {};
        return;
    }
    resting_ = false;  // re-established by the ground contact below

    const TickContext ctx{env,
                          *vehicle_,
                          epoch,
                          body,
                          frame,
                          vehicle_->tank_mass_flows_kgps(),
                          solar_radiation_acceleration(env, *vehicle_, state_, vehicle_->mass_properties().mass_kg, epoch),
                          vehicle_->rcs_torque_nm(),
                          vehicle_->rcs_force_n(),
                          external_force_gcrf_n_,
                          external_torque_body_nm_,
                          &specific_force_body_mps2_};
    const integrators::OdeFunction<13> f = [&ctx](double tau, const State13& y) { return derivative(ctx, tau, y); };
    state_ = unpack(integrators::rk4_step<13>(f, 0.0, pack(state_), dt_s));
    snap_ideal_attitude();
    if (env.propellant_settling) vehicle_->settle_propellant(specific_force_body_mps2_, dt_s);
    // Entry limits of a single stage with a heat shield (the ship): heating from the tick's air data, load from
    // the aerodynamic forces at the end of the tick (thrust loads are the engines' business).
    if (vehicle_->stages().size() == 1 && vehicle_->stages().front().config.heat_shield.nose_radius_m > 0.0) {
        const auto& shield = vehicle_->stages().front().config.heat_shield;
        const double speed = norm(air_data_.air_velocity_body_mps);
        const double density = air_data_.air.density_kg_m3;
        const bool earth_air = body.id() == ephemeris::Body::Earth;  // the radiative tables are Earth air
        radiative_heat_rate_w_m2_ = earth_air && env.radiative_heating
                                        ? env.radiative_heating->heat_rate_w_m2(density, speed, shield.nose_radius_m)
                                        : 0.0;
        heat_rate_w_m2_ =
            aero::stagnation_heat_rate_w_m2(density, speed, shield.nose_radius_m) + radiative_heat_rate_w_m2_;
        heat_load_j_m2_ += heat_rate_w_m2_ * dt_s;
        peak_heat_rate_w_m2_ = std::max(peak_heat_rate_w_m2_, heat_rate_w_m2_);
        const Forces end = evaluate(ctx, dt_s, state_);
        aero_acceleration_body_mps2_ = end.aero_body_n / end.mass.mass_kg;
        const double load_g = norm(aero_acceleration_body_mps2_) / constants::standard_gravity_mps2;
        if (heat_rate_w_m2_ > shield.max_heat_rate_w_m2 || heat_load_j_m2_ > shield.max_heat_load_j_m2) {
            breakup_reason_ = "Hitzeschild versagt";
        } else if (shield.max_load_g > 0.0 && load_g > shield.max_load_g) {
            breakup_reason_ = "Strukturüberlast";
        }
        if (shield.crew_load_limit_g > 0.0 && load_g > shield.crew_load_limit_g) crew_load_warning_ = true;
        constexpr double warning_fraction = 0.8;
        heat_warning_ = heat_rate_w_m2_ >= warning_fraction * shield.max_heat_rate_w_m2 ||
                        heat_load_j_m2_ >= warning_fraction * shield.max_heat_load_j_m2 ||
                        (shield.max_load_g > 0.0 && load_g >= warning_fraction * shield.max_load_g);
        if (!breakup_reason_.empty()) impacted_ = true;
        constexpr double entry_height_m = 80000.0, skip_height_m = 100000.0;  // [E] event thresholds
        if (body.has_atmosphere()) {
            if (geodetic.height_m < entry_height_m) entered_atmosphere_ = true;
            if (entered_atmosphere_ && geodetic.height_m > skip_height_m) skipped_out_ = true;
        }
    } else if (accelerometer_) {
        const Forces end = evaluate(ctx, dt_s, state_);
        aero_acceleration_body_mps2_ = end.aero_body_n / end.mass.mass_kg;
    }
    external_force_gcrf_n_ = {};
    external_torque_body_nm_ = {};
    vehicle_->consume_propellant(dt_s);

    resolve_ground_contact(body, body.frame(epoch + dt_s));
}

// Ground = flat at ground_height_m_ (the pad elevation) on the Earth (terrain: M8); the terrain on the Moon (lunar l2).
// Gentle contact: the ground carries the vehicle (it rests, co-rotating with the Earth, and lifts off as
// soon as thrust exceeds weight). Hard contact (sink rate or tilt too large): impact.
void FlightBody::resolve_ground_contact(const environment::ReferenceBody& body, const environment::BodyFrame& frame) {
    // Legs moving or out on a leg landing site: that simulation owns the contact.
    if (vehicle_->leg_deployment() > 0.0 && legs_on_site_) return;
    const double cg = vehicle_->mass_properties().cg_station_m;
    const Vec3 bottom_fixed = frame.to_fixed(state_.position_gcrf_m + state_.attitude.rotate(Vec3{-cg, 0.0, 0.0}));
    const Vec3 top_fixed =
        frame.to_fixed(state_.position_gcrf_m + state_.attitude.rotate(Vec3{vehicle_->length_m() - cg, 0.0, 0.0}));
    const auto bottom = frames::fixed_to_geodetic(bottom_fixed, body.shape());
    const double ground_height = ground_height_at(body, bottom_fixed);
    // Clearance of the bottom and the top point over their ground (the same as the lowest height against a flat
    // ground's height).
    const double lowest = std::min(bottom.height_m - ground_height, frames::fixed_to_geodetic(top_fixed, body.shape()).height_m -
                                                                         ground_height_at(body, top_fixed));
    if (lowest >= 0.0) {
        return;
    }
    const Vec3 up = frame.transform.rotation * local_up_fixed(bottom);
    const Vec3 relative = frame.relative_velocity(state_.position_gcrf_m, state_.velocity_gcrf_mps);
    const double sink_rate = -dot(relative, up);
    const double tilt = std::acos(std::clamp(dot(state_.attitude.rotate(Vec3{1.0, 0.0, 0.0}), up), -1.0, 1.0));
    if (!touchdown_sink_mps_) touchdown_sink_mps_ = sink_rate;
    if (vehicle_->splashdown_recovery()) {
        // On the water (d4): any tilt, sink up to the vehicle's limit; no buoyancy (it rests at the surface).
        if (sink_rate > vehicle_->max_splashdown_speed_mps()) {
            hold_wreck(frame, body.id());
            return;
        }
    } else if (sink_rate > max_touchdown_sink_rate_mps || tilt > max_touchdown_tilt_rad) {
        hold_wreck(frame, body.id());
        return;
    }
    // Rest on the ground below the current bottom point, keeping the current attitude relative to the Earth.
    pad_fixed_m_ = frames::geodetic_to_fixed({bottom.latitude_rad, bottom.longitude_rad, ground_height}, body.shape());
    attitude_fixed_ = (Quat::from_matrix(transpose(frame.transform.rotation)) * state_.attitude).normalized();
    held_on_ = body.id();
    follow_pad(frame);
    resting_ = true;
}

// Opening loads: q at each open set's riser point (air data of this tick) times one canopy's drag area. The drag area
// already grew this tick, so the load is that of the area the tick will fly with.
Vec3 FlightBody::point_air_velocity_body_mps(double station_m) const {
    const auto mass = vehicle_->mass_properties();
    const Vec3 arm{station_m - mass.cg_station_m, -mass.cg_offset_m.y, -mass.cg_offset_m.z};
    return air_data_.air_velocity_body_mps + cross(state_.angular_velocity_body_radps, arm);
}

void FlightBody::judge_parachute_loads() {
    const auto& stages = vehicle_->stages();
    for (size_t i = 0; i < stages.size(); ++i) {
        for (size_t k = 0; k < stages[i].parachutes.size(); ++k) {
            const auto& chute = stages[i].parachutes[k];
            if (chute.state != ParachuteState::Open) continue;
            const Vec3 v = point_air_velocity_body_mps(chute.attach_station_m);
            const double q = 0.5 * air_data_.air.density_kg_m3 * dot(v, v);
            vehicle_->record_parachute_load(i, k, q * chute.drag_area_m2 / chute.config.count);
        }
    }
}

void FlightBody::update_air_data(const environment::ReferenceBody& body, const environment::BodyFrame& frame) {
    const auto geodetic = frames::fixed_to_geodetic(frame.to_fixed(state_.position_gcrf_m), body.shape());
    const auto air = body.air(geodetic.height_m);
    const Vec3 air_velocity = frame.relative_velocity(state_.position_gcrf_m, state_.velocity_gcrf_mps);
    air_data_.air_velocity_body_mps = state_.attitude.inverse_rotate(air_velocity);
    air_data_.air = {air.density_kg_m3, air.speed_of_sound_mps,
                     aero::rarefaction_weight(air.mean_free_path_m / vehicle_->length_m())};
    air_data_.mean_free_path_m = air.mean_free_path_m;
}

bool FlightBody::can_coast() const {
    return !clamped_ && !impacted_ && !vehicle_->any_engine_producing_thrust();
}

void FlightBody::coast(const FlightEnvironment& env, time::Epoch epoch, double duration_s) {
    if (impacted_ || duration_s <= 0.0) {
        if (impacted_ && wreck_on_ground_ && duration_s > 0.0) {
            follow_pad(reference_body_of(env, held_on_).frame(epoch + duration_s));
        }
        return;
    }
    vehicle_->stop_rcs();  // a pulse in progress ends: its mass flow must not run through the whole segment
    if (resting_ || caught_ || landed_) {  // stays on the ground / on the tower, co-rotating with its body
        follow_pad(reference_body_of(env, held_on_).frame(epoch + duration_s));
        return;
    }
    reference_ = reference_body(env, epoch);
    const auto body = reference_body_of(env, *reference_);
    const TickContext ctx{env,
                          *vehicle_,
                          epoch,
                          body,
                          body.frame(epoch),
                          vehicle_->tank_mass_flows_kgps(),
                          solar_radiation_acceleration(env, *vehicle_, state_, vehicle_->mass_properties().mass_kg, epoch),
                          {},
                          {},
                          {},
                          {}};
    const integrators::OdeFunction<13> f = [&ctx](double tau, const State13& y) { return derivative(ctx, tau, y); };
    integrators::Dop853Options<13> options;
    options.relative_tolerance = 1e-12;
    options.absolute_tolerance =
        State13{{1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12}};
    const auto result = integrators::integrate<13>(f, 0.0, pack(state_), duration_s, options);
    state_ = unpack(result.y);
    snap_ideal_attitude();
    reference_ = reference_body(env, epoch + duration_s);
    const auto end_body = reference_body_of(env, *reference_);
    const auto end_frame = end_body.frame(epoch + duration_s);
    resolve_ground_contact(end_body, end_frame);
    update_air_data(end_body, end_frame);
}

std::unique_ptr<FlightBody> FlightBody::separate(size_t index, double separation_dv_mps) {
    const MassProperties combined = vehicle_->mass_properties();
    const double shift = vehicle_->stages().at(index).base_station_m;
    auto upper_vehicle = vehicle_->separate(index);
    const MassProperties lower = vehicle_->mass_properties();
    const MassProperties upper = upper_vehicle->mass_properties();

    auto rigid_part = [&](double cg_station_in_combined, double dv) {
        const Vec3 offset_body{cg_station_in_combined - combined.cg_station_m, 0.0, 0.0};
        RigidBodyState part = state_;
        part.position_gcrf_m += state_.attitude.rotate(offset_body);
        part.velocity_gcrf_mps += state_.attitude.rotate(cross(state_.angular_velocity_body_radps, offset_body) +
                                                         Vec3{dv, 0.0, 0.0});
        return part;
    };
    const double total = lower.mass_kg + upper.mass_kg;
    const RigidBodyState lower_state = rigid_part(lower.cg_station_m, -separation_dv_mps * upper.mass_kg / total);
    const RigidBodyState upper_state = rigid_part(shift + upper.cg_station_m, separation_dv_mps * lower.mass_kg / total);
    state_ = lower_state;
    auto upper_body = std::make_unique<FlightBody>(std::move(upper_vehicle), upper_state);
    upper_body->ground_height_m_ = ground_height_m_;
    upper_body->reference_ = reference_;
    return upper_body;
}

FlightTelemetry FlightBody::telemetry(const FlightEnvironment& env, time::Epoch epoch) const {
    FlightTelemetry t;
    const auto id = reference_body(env, epoch);
    const auto body = reference_body_of(env, id);
    const auto frame = body.frame(epoch);
    const auto geodetic = frames::fixed_to_geodetic(frame.to_fixed(state_.position_gcrf_m), body.shape());
    t.reference_body = id;
    t.altitude_m = geodetic.height_m;
    t.latitude_rad = geodetic.latitude_rad;
    t.longitude_rad = geodetic.longitude_rad;
    const Vec3 up = frame.transform.rotation * local_up_fixed(geodetic);
    const Vec3 relative = frame.relative_velocity(state_.position_gcrf_m, state_.velocity_gcrf_mps);
    t.speed_inertial_mps = norm(state_.velocity_gcrf_mps);
    t.speed_relative_mps = norm(relative);
    t.vertical_speed_mps = dot(relative, up);
    t.flight_path_angle_rad = std::atan2(t.vertical_speed_mps, norm(relative - t.vertical_speed_mps * up));

    const auto centred = frame.centred({state_.position_gcrf_m, state_.velocity_gcrf_mps});
    const auto el = orbit::elements_from_state(centred, body.gm_m3ps2());
    const double radius = body.shape().semi_major_axis_m;
    t.periapsis_altitude_m = el.semi_major_axis_m * (1.0 - el.eccentricity) - radius;
    t.apoapsis_altitude_m = el.eccentricity < 1.0 ? el.semi_major_axis_m * (1.0 + el.eccentricity) - radius
                                                  : std::numeric_limits<double>::infinity();
    const Vec3 h = cross(centred.position_m, centred.velocity_mps);
    const Vec3 pole = frame.transform.rotation * Vec3{0.0, 0.0, 1.0};
    t.inclination_rad = std::acos(std::clamp(dot(h, pole) / norm(h), -1.0, 1.0));

    const TickContext ctx{env, *vehicle_, epoch, body, frame, vehicle_->tank_mass_flows_kgps(), {}, {}, {}};
    const Forces f = evaluate(ctx, 0.0, state_);
    t.mach = f.aero.mach;
    t.knudsen = f.aero.knudsen;
    t.dynamic_pressure_pa = f.aero.dynamic_pressure_pa;
    t.angle_of_attack_rad = f.aero.angle_of_attack_rad;
    t.mass_kg = f.mass.mass_kg;
    t.propellant_kg = vehicle_->propellant_kg();
    t.thrust_n = f.thrust_n;
    t.sensed_acceleration_g = norm(f.nongravitational_body_n) / f.mass.mass_kg / constants::standard_gravity_mps2;
    for (const auto& stage : vehicle_->stages()) {
        for (const auto& e : stage.engines) t.engines_running += e.producing_thrust() ? 1 : 0;
    }
    return t;
}

}  // namespace starsim::vehicle
