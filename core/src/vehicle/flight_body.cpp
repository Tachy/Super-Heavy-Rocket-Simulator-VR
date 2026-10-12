#include "starsim/vehicle/flight_body.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/environment/eclipse.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/integrators/integrate.hpp"
#include "starsim/integrators/rk4.hpp"
#include "starsim/math/constants.hpp"

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

Vec3 local_up_fixed(const frames::GeodeticPosition& g) {
    return {std::cos(g.latitude_rad) * std::cos(g.longitude_rad), std::cos(g.latitude_rad) * std::sin(g.longitude_rad),
            std::sin(g.latitude_rad)};
}

// Quantities that are constant during one tick.
struct TickContext {
    const FlightEnvironment& env;
    const Vehicle& vehicle;
    time::Epoch epoch;
    math::Mat3 earth_fixed_to_gcrf;
    Vec3 earth_rotation_gcrf;
    std::vector<std::vector<double>> tank_flows;
    Vec3 srp_acceleration_gcrf;  // evaluated at the start of the tick
    Vec3 rcs_torque_nm;          // commanded reaction-control couple, body axes
    Vec3 rcs_force_n;            // commanded reaction-control translation force, body axes
    Vec3 external_force_gcrf_n;   // contact (tower arms), through the CG
    Vec3 external_torque_body_nm;
};

struct Forces {
    Vec3 acceleration_gcrf;        // total
    Vec3 nongravitational_body_n;  // thrust + aero, body axes
    Vec3 moment_body_nm;
    double thrust_n = 0.0;
    aero::AeroResult aero;
    MassProperties mass;
};

Forces evaluate(const TickContext& ctx, double tau_s, const RigidBodyState& s) {
    Forces f;
    f.mass = ctx.vehicle.mass_properties(&ctx.tank_flows, tau_s);
    const Vec3 r_fixed = transpose(ctx.earth_fixed_to_gcrf) * s.position_gcrf_m;
    const auto geodetic = frames::fixed_to_geodetic(r_fixed);
    const auto air = ctx.env.atmosphere.at(geodetic.height_m);

    // Aerodynamics relative to the co-rotating atmosphere (no wind model yet).
    const Vec3 air_velocity = s.velocity_gcrf_mps - cross(ctx.earth_rotation_gcrf, s.position_gcrf_m);
    const bool powered = ctx.vehicle.any_engine_producing_thrust();
    f.aero = ctx.vehicle.aero().compute(s.attitude.inverse_rotate(air_velocity), air.density_kg_m3,
                                        air.speed_of_sound_mps, powered, air.mean_free_path_m);
    f.nongravitational_body_n = f.aero.force_body_n;
    f.moment_body_nm = cross(Vec3{f.aero.normal_force_station_m - f.mass.cg_station_m, 0.0, 0.0}, f.aero.force_body_n);

    for (const auto& stage : ctx.vehicle.stages()) {
        for (const auto& e : stage.engines) {
            if (!e.producing_thrust()) continue;
            const double thrust = e.spool * e.performance->thrust_n(e.throttle, air.pressure_pa);
            const Vec3 force = thrust * e.thrust_direction();
            f.thrust_n += thrust;
            f.nongravitational_body_n += force;
            f.moment_body_nm += cross(e.gimbal_point_m - Vec3{f.mass.cg_station_m, 0.0, 0.0}, force);
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
            f.moment_body_nm += cross(arm, force.force_body_n);
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
    const double area = std::abs(sun_body.x) * constants::pi * radius * radius +
                        std::sqrt(std::max(0.0, 1.0 - sun_body.x * sun_body.x)) * vehicle.length_m() * vehicle.diameter_m();
    double cr = 0.0;  // mass-weighted is overkill: use the stack average
    for (const auto& stage : vehicle.stages()) cr += stage.config.radiation_pressure_coefficient;
    cr /= static_cast<double>(vehicle.stages().size());
    const double pressure = solar_pressure_1au_pa * (astronomical_unit_m / distance) * (astronomical_unit_m / distance);
    return sun_dir * (-pressure * cr * area * sunlit / mass_kg);
}

}  // namespace

Vec3 earth_rotation_vector(const frames::FixedToInertial& transform) {
    const math::Mat3 w = transform.rotation_rate_per_s * transpose(transform.rotation);
    return {0.5 * (w(2, 1) - w(1, 2)), 0.5 * (w(0, 2) - w(2, 0)), 0.5 * (w(1, 0) - w(0, 1))};
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
    body->pad_itrf_m_ = frames::geodetic_to_fixed(pad);
    body->attitude_itrf_ = Quat::from_matrix(columns);
    body->follow_pad(earth, epoch);
    return body;
}

void FlightBody::follow_pad(const frames::BodyOrientation& earth, time::Epoch epoch) {
    const auto transform = earth.fixed_to_gcrf(epoch);
    const double cg = vehicle_->mass_properties().cg_station_m;
    const Vec3 cg_fixed = pad_itrf_m_ + attitude_itrf_.rotate(Vec3{cg, 0.0, 0.0});
    state_.position_gcrf_m = transform.position(cg_fixed);
    state_.velocity_gcrf_mps = transform.velocity(cg_fixed, {});
    state_.attitude = (Quat::from_matrix(transform.rotation) * attitude_itrf_).normalized();
    state_.angular_velocity_body_radps = state_.attitude.inverse_rotate(earth_rotation_vector(transform));
}

void FlightBody::apply_external_wrench(const Vec3& force_gcrf_n, const Vec3& torque_body_nm) {
    external_force_gcrf_n_ += force_gcrf_n;
    external_torque_body_nm_ += torque_body_nm;
}

void FlightBody::hold_fixed_to_earth(const frames::BodyOrientation& earth, time::Epoch epoch) {
    const auto transform = earth.fixed_to_gcrf(epoch);
    const math::Mat3 to_fixed = transpose(transform.rotation);
    const double cg = vehicle_->mass_properties().cg_station_m;
    pad_itrf_m_ = to_fixed * (state_.position_gcrf_m - state_.attitude.rotate(Vec3{cg, 0.0, 0.0}));
    attitude_itrf_ = (Quat::from_matrix(to_fixed) * state_.attitude).normalized();
    caught_ = true;
    resting_ = false;
    follow_pad(earth, epoch);
}

Vec3 FlightBody::rcs_torque(const VehicleCommand& command) const {
    const Vec3 max = vehicle_->max_rcs_torque_nm();
    const Vec3& f = command.rcs_torque_fraction;
    return {std::clamp(f.x, -1.0, 1.0) * max.x, std::clamp(f.y, -1.0, 1.0) * max.y, std::clamp(f.z, -1.0, 1.0) * max.z};
}

Vec3 FlightBody::rcs_force(const VehicleCommand& command) const {
    const Vec3& f = command.rcs_force_fraction;
    const double magnitude = norm(f);
    const Vec3 fraction = magnitude > 1.0 ? f / magnitude : f;
    return fraction * vehicle_->max_rcs_force_n();
}

void FlightBody::step(const FlightEnvironment& env, time::Epoch epoch, double dt_s, const VehicleCommand& command) {
    if (impacted_) {
        return;
    }
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const auto geodetic = frames::fixed_to_geodetic(transpose(transform.rotation) * state_.position_gcrf_m);
    vehicle_->update_engines(dt_s, command, env.atmosphere.at(geodetic.height_m).pressure_pa);
    vehicle_->update_grid_fins(dt_s, command);
    update_air_data(env, epoch);

    if (clamped_ || caught_) {
        vehicle_->consume_propellant(dt_s);
        follow_pad(env.earth_orientation, epoch + dt_s);
        external_force_gcrf_n_ = {};
        external_torque_body_nm_ = {};
        return;
    }
    resting_ = false;  // re-established by the ground contact below

    const TickContext ctx{env,
                          *vehicle_,
                          epoch,
                          transform.rotation,
                          earth_rotation_vector(transform),
                          vehicle_->tank_mass_flows_kgps(),
                          solar_radiation_acceleration(env, *vehicle_, state_, vehicle_->mass_properties().mass_kg, epoch),
                          rcs_torque(command),
                          rcs_force(command),
                          external_force_gcrf_n_,
                          external_torque_body_nm_};
    const integrators::OdeFunction<13> f = [&ctx](double tau, const State13& y) { return derivative(ctx, tau, y); };
    state_ = unpack(integrators::rk4_step<13>(f, 0.0, pack(state_), dt_s));
    external_force_gcrf_n_ = {};
    external_torque_body_nm_ = {};
    vehicle_->consume_propellant(dt_s);

    resolve_ground_contact(env.earth_orientation, epoch + dt_s);
}

// Ground = flat at ground_height_m_ (the pad elevation) until terrain exists (M8).
// Gentle contact: the ground carries the vehicle (it rests, co-rotating with the Earth, and lifts off as
// soon as thrust exceeds weight). Hard contact (sink rate or tilt too large): impact.
void FlightBody::resolve_ground_contact(const frames::BodyOrientation& earth, time::Epoch epoch) {
    const auto transform = earth.fixed_to_gcrf(epoch);
    const math::Mat3 to_fixed = transpose(transform.rotation);
    const double cg = vehicle_->mass_properties().cg_station_m;
    const Vec3 bottom_fixed = to_fixed * (state_.position_gcrf_m + state_.attitude.rotate(Vec3{-cg, 0.0, 0.0}));
    const Vec3 top_fixed =
        to_fixed * (state_.position_gcrf_m + state_.attitude.rotate(Vec3{vehicle_->length_m() - cg, 0.0, 0.0}));
    const auto bottom = frames::fixed_to_geodetic(bottom_fixed);
    const double lowest = std::min(bottom.height_m, frames::fixed_to_geodetic(top_fixed).height_m);
    if (lowest >= ground_height_m_) {
        return;
    }
    const Vec3 up = transform.rotation * local_up_fixed(bottom);
    const Vec3 relative = state_.velocity_gcrf_mps - cross(earth_rotation_vector(transform), state_.position_gcrf_m);
    const double sink_rate = -dot(relative, up);
    const double tilt = std::acos(std::clamp(dot(state_.attitude.rotate(Vec3{1.0, 0.0, 0.0}), up), -1.0, 1.0));
    if (sink_rate > max_touchdown_sink_rate_mps || tilt > max_touchdown_tilt_rad) {
        impacted_ = true;
        return;
    }
    // Rest on the ground below the current bottom point, keeping the current attitude relative to the Earth.
    pad_itrf_m_ = frames::geodetic_to_fixed({bottom.latitude_rad, bottom.longitude_rad, ground_height_m_});
    attitude_itrf_ = (Quat::from_matrix(to_fixed) * state_.attitude).normalized();
    follow_pad(earth, epoch);
    resting_ = true;
}

void FlightBody::update_air_data(const FlightEnvironment& env, time::Epoch epoch) {
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const auto geodetic = frames::fixed_to_geodetic(transpose(transform.rotation) * state_.position_gcrf_m);
    const auto air = env.atmosphere.at(geodetic.height_m);
    const Vec3 air_velocity =
        state_.velocity_gcrf_mps - cross(earth_rotation_vector(transform), state_.position_gcrf_m);
    air_data_.air_velocity_body_mps = state_.attitude.inverse_rotate(air_velocity);
    air_data_.air = {air.density_kg_m3, air.speed_of_sound_mps,
                     aero::rarefaction_weight(air.mean_free_path_m / vehicle_->length_m())};
}

bool FlightBody::can_coast() const {
    return !clamped_ && !impacted_ && !vehicle_->any_engine_producing_thrust();
}

void FlightBody::coast(const FlightEnvironment& env, time::Epoch epoch, double duration_s) {
    if (impacted_ || duration_s <= 0.0) {
        return;
    }
    if (resting_ || caught_) {  // stays on the ground / on the tower, co-rotating with the Earth
        follow_pad(env.earth_orientation, epoch + duration_s);
        return;
    }
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const TickContext ctx{env,
                          *vehicle_,
                          epoch,
                          transform.rotation,
                          earth_rotation_vector(transform),
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
    resolve_ground_contact(env.earth_orientation, epoch + duration_s);
    update_air_data(env, epoch + duration_s);
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
    return upper_body;
}

FlightTelemetry FlightBody::telemetry(const FlightEnvironment& env, time::Epoch epoch) const {
    FlightTelemetry t;
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const auto geodetic = frames::fixed_to_geodetic(transpose(transform.rotation) * state_.position_gcrf_m);
    t.altitude_m = geodetic.height_m;
    t.latitude_rad = geodetic.latitude_rad;
    t.longitude_rad = geodetic.longitude_rad;
    const Vec3 up = transform.rotation * local_up_fixed(geodetic);
    const Vec3 relative = state_.velocity_gcrf_mps - cross(earth_rotation_vector(transform), state_.position_gcrf_m);
    t.speed_inertial_mps = norm(state_.velocity_gcrf_mps);
    t.speed_relative_mps = norm(relative);
    t.vertical_speed_mps = dot(relative, up);
    t.flight_path_angle_rad = std::atan2(t.vertical_speed_mps, norm(relative - t.vertical_speed_mps * up));

    const TickContext ctx{env, *vehicle_, epoch, transform.rotation, earth_rotation_vector(transform),
                          vehicle_->tank_mass_flows_kgps(), {}, {}, {}};
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
