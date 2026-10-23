#include "starsim/gnc/descent_model.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::gnc {

using math::Quat;
using math::Vec3;

namespace {

constexpr double table_height_m = 15000.0;  // air data tabulated up to this height above the pad
constexpr double table_mach = 4.0;          // drag tabulated up to this Mach number
constexpr double jacobian_relative_step = 1e-6;

// Linear interpolation in an equally spaced table starting at 0; clamped at both ends.
double interpolate(const std::vector<double>& table, double step, double at) {
    if (table.size() < 2) return table.empty() ? 0.0 : table[0];
    const double s = std::clamp(at / step, 0.0, static_cast<double>(table.size() - 1));
    const size_t i = std::min(static_cast<size_t>(s), table.size() - 2);
    const double f = s - static_cast<double>(i);
    return table[i] + (table[i + 1] - table[i]) * f;
}

}  // namespace

DescentState make_state(double mass_kg, const Vec3& r, const Vec3& v, const Quat& q, const Vec3& w) {
    return {{mass_kg, r.x, r.y, r.z, v.x, v.y, v.z, q.w, q.x, q.y, q.z, w.x, w.y, w.z}};
}
Vec3 state_position(const DescentState& x) { return {x[1], x[2], x[3]}; }
Vec3 state_velocity(const DescentState& x) { return {x[4], x[5], x[6]}; }
Quat state_attitude(const DescentState& x) { return {x[7], x[8], x[9], x[10]}; }
Vec3 state_rate(const DescentState& x) { return {x[11], x[12], x[13]}; }

DescentModel::HullNormalForce DescentModel::hull_normal_force(const DescentState& x) const {
    HullNormalForce result;
    const Vec3 v = state_velocity(x);
    const double speed = norm(v);
    if (!crossflow_enabled || speed <= 1e-6) return result;
    const double height = std::max(state_position(x).x, 0.0);
    const double rho = normal_force_scale * interpolate(density_kg_m3, height_step_m, height);
    if (rho <= 0.0) return result;
    const double mach = speed / std::max(interpolate(speed_of_sound_mps, height_step_m, height), 1.0);
    // Air velocity in body axes (still air); its cross component c sets the angle of attack:
    // sin a = |c| / V. Written in c so that it stays smooth at a = 0 / pi (the Jacobians difference it).
    const Vec3 air_body = state_attitude(x).normalized().inverse_rotate(v);
    const Vec3 c{0.0, air_body.y, air_body.z};
    const double c_norm = norm(c);
    // Cross-flow term: q S_p cd_x sin^2 a along -c / |c| = -c (rho / 2) S_p cd_x |c|.
    const Vec3 crossflow = c * (-0.5 * rho * planform_area_m2 * interpolate(crossflow_cd_by_mach, mach_step, mach) * c_norm);
    result.force_body_n = crossflow;
    result.moment_body_nm = cross(Vec3{crossflow_station_above_cg_m, 0.0, 0.0}, crossflow);
    if (potential_term) {
        // q A |sin 2a| cos(a/2) along -c / |c| = -c rho A |V_x| cos(a/2); 1 + cos a in its cancellation-free
        // form tail first (cos a < 0): sin^2 a / (1 - cos a).
        const double one_plus_cos = air_body.x >= 0.0 ? 1.0 + air_body.x / speed
                                                       : c_norm * c_norm / (speed * (speed - air_body.x));
        const Vec3 potential = c * (-rho * reference_area_m2 * std::abs(air_body.x) * std::sqrt(0.5 * one_plus_cos));
        result.force_body_n += potential;
        result.moment_body_nm += cross(Vec3{potential_station_above_cg_m, 0.0, 0.0}, potential);
    }
    return result;
}

DescentState DescentModel::derivative(const DescentState& x, const Vec3& u, int phase) const {
    const double m = x[descent_index::mass];
    const Vec3 r = state_position(x);
    const Vec3 v = state_velocity(x);
    const Quat q = state_attitude(x);
    const Vec3 w = state_rate(x);
    Vec3 drag;
    const double speed = norm(v);
    if (speed > 1e-6) {
        const double height = std::max(r.x, 0.0);
        const double rho = interpolate(density_kg_m3, height_step_m, height);
        const double mach = speed / std::max(interpolate(speed_of_sound_mps, height_step_m, height), 1.0);
        drag = v * (-0.5 * rho * speed * reference_area_m2 * interpolate(cd_by_mach, mach_step, mach));
    }
    const HullNormalForce hull = hull_normal_force(x);
    const Vec3& omega = earth_rate_radps;
    const Vec3 acceleration = q.rotate(u) / m + drag / m + q.normalized().rotate(hull.force_body_n) / m + gravity_mps2 -
                              cross(omega, v) * 2.0 - cross(omega, cross(omega, r - earth_center_m));
    const Quat q_dot = math::attitude_rate(q, w);
    const Vec3 lever{-gimbal_lever_m, 0.0, 0.0};  // gimbal point relative to the CG, body axes
    const Vec3 h{inertia_kg_m2.x * w.x, inertia_kg_m2.y * w.y, inertia_kg_m2.z * w.z};
    const Vec3 torque = cross(lever, u) + hull.moment_body_nm - cross(w, h);
    const auto& set = engine_sets[static_cast<size_t>(phase)];
    return make_state(-set.mass_flow_per_thrust_s_m * (norm(u) + set.pressure_thrust_loss_n), v, acceleration, q_dot,
                      {torque.x / inertia_kg_m2.x, torque.y / inertia_kg_m2.y, torque.z / inertia_kg_m2.z});
}

void DescentModel::jacobians(const DescentState& x, const Vec3& u, int phase, Mat14x14& a, Mat14x3& b) const {
    for (int j = 0; j < 14; ++j) {
        const double h = jacobian_relative_step * std::max(1.0, std::abs(x[j]));
        DescentState plus = x, minus = x;
        plus[j] += h;
        minus[j] -= h;
        const auto df = derivative(plus, u, phase) - derivative(minus, u, phase);
        for (int i = 0; i < 14; ++i) a[static_cast<size_t>(i * 14 + j)] = df[i] / (2.0 * h);
    }
    const double h = jacobian_relative_step * std::max(1.0, norm(u));
    const Vec3 steps[3] = {{h, 0.0, 0.0}, {0.0, h, 0.0}, {0.0, 0.0, h}};
    for (int j = 0; j < 3; ++j) {
        const auto df = derivative(x, u + steps[j], phase) - derivative(x, u - steps[j], phase);
        for (int i = 0; i < 14; ++i) b[static_cast<size_t>(i * 3 + j)] = df[i] / (2.0 * h);
    }
}

DescentModel make_descent_model(const DescentModelInputs& in) {
    const auto& vehicle = *in.vehicle;
    const auto& stage = vehicle.stages().front();
    const double pressure = in.atmosphere ? in.atmosphere->at(in.site_height_m).pressure_pa : 0.0;  // nullptr: vacuum
    const auto in_phase = [&](const vehicle::Engine& e, int p) {
        const auto& groups = in.phase_groups[static_cast<size_t>(p)];
        return !e.starved && std::find(groups.begin(), groups.end(), e.group_id) != groups.end();
    };
    // Lever of the planning model's single gimbal point: mean station of the gimballed engines over all phases.
    DescentModel m;
    double gimbal_station = 0.0;
    int gimbal_count = 0;
    for (int p = 0; p < descent_phases; ++p) {
        for (const auto& e : stage.engines) {
            if (!in_phase(e, p) || !e.gimbaled) continue;
            gimbal_station += e.gimbal_point_m.x;
            ++gimbal_count;
        }
    }
    const auto mass = vehicle.mass_properties();
    m.gimbal_lever_m = mass.cg_station_m - (gimbal_count > 0 ? gimbal_station / gimbal_count : 0.0);
    for (int p = 0; p < descent_phases; ++p) {
        EngineSetModel set;
        double flow = 0.0;
        double vacuum_thrust = 0.0;
        for (const auto& e : stage.engines) {
            if (!in_phase(e, p)) continue;
            const auto& c = e.performance->config();
            // Canted engines (the HLS landing thrusters) push along their own axis: only the axial part counts.
            const double axial = e.axis_body.x;
            set.thrust_max_n += axial * e.performance->thrust_n(1.0, pressure);
            vacuum_thrust += axial * e.performance->thrust_n(1.0, 0.0);
            set.thrust_min_n += axial * e.performance->thrust_n(c.throttle_min, pressure);
            flow += e.performance->mass_flow_kgps(1.0);
            if (c.throttle_rate_per_s > 0.0) {
                set.throttle_rate_per_s = set.throttle_rate_per_s > 0.0 ? std::min(set.throttle_rate_per_s, c.throttle_rate_per_s)
                                                                        : c.throttle_rate_per_s;
            }
            if (e.gimbaled) {
                set.gimbal_max_rad = set.gimbal_max_rad > 0.0 ? std::min(set.gimbal_max_rad, c.gimbal_max_rad) : c.gimbal_max_rad;
            }
        }
        set.mass_flow_per_thrust_s_m = vacuum_thrust > 0.0 ? flow / vacuum_thrust : 0.0;
        set.pressure_thrust_loss_n = vacuum_thrust - set.thrust_max_n;
        if (set.thrust_max_n > 0.0 && set.gimbal_max_rad == 0.0) {
            // A set without gimbals (the landing thrusters) is steered by differential throttling (canted engines,
            // trims up to half their throttle range [E]) and the RCS. The model has no torque input, so it gets an
            // equivalent gimbal that gives that torque about the CG (documented simplification: the implied lateral
            // force torque / lever is a few % of the thrust; the re-plans every second absorb it).
            Vec3 torque = vehicle.max_rcs_torque_nm();
            for (const auto& e : stage.engines) {
                if (!in_phase(e, p) || !e.throttle_steered()) continue;
                const auto& c = e.performance->config();
                const Vec3 t = vehicle::throttle_torque_nm(e, mass.cg_station_m);
                torque += Vec3{std::abs(t.x), std::abs(t.y), std::abs(t.z)} * (0.5 * (c.throttle_max - c.throttle_min));
            }
            const double lever = std::abs(m.gimbal_lever_m);
            const double pitch_yaw = std::min(torque.y, torque.z);
            if (lever > 0.0 && pitch_yaw > 0.0) set.gimbal_max_rad = std::atan(pitch_yaw / (lever * set.thrust_max_n));
        }
        m.engine_sets[static_cast<size_t>(p)] = set;
    }
    m.inertia_kg_m2 = mass.inertia_kg_m2;
    m.reference_area_m2 = constants::pi * 0.25 * vehicle.diameter_m() * vehicle.diameter_m();
    const auto& drag = vehicle.aero().drag_curve(true);
    for (double mach = 0.0; mach <= table_mach + 1e-9; mach += m.mach_step) m.cd_by_mach.push_back(drag.cd(mach, true));
    // Hull normal force: the flight model's slender body (its identified cross-flow curve or the generic value).
    const auto& hull = vehicle.aero();
    const auto& geometry = hull.geometry();
    for (double mach = 0.0; mach <= table_mach + 1e-9; mach += m.mach_step) {
        m.crossflow_cd_by_mach.push_back(hull.continuum_crossflow_cd(mach));
    }
    m.planform_area_m2 = geometry.length_m * geometry.diameter_m;
    m.crossflow_station_above_cg_m = 0.5 * geometry.length_m - mass.cg_station_m;
    m.potential_term = geometry.nose_length_m > 0.0;
    m.potential_station_above_cg_m = hull.potential_station_m() - mass.cg_station_m;
    if (!in.atmosphere) {  // vacuum: one entry each (interpolate returns it)
        m.density_kg_m3 = {0.0};
        m.speed_of_sound_mps = {0.0};
    }
    for (double h = 0.0; in.atmosphere && h <= table_height_m + 1e-9; h += m.height_step_m) {
        const auto air = in.atmosphere->at(in.site_height_m + h);
        m.density_kg_m3.push_back(air.density_kg_m3);
        m.speed_of_sound_mps.push_back(air.speed_of_sound_mps);
    }
    m.gravity_mps2 = in.gravity_mps2;
    m.earth_rate_radps = in.earth_rate_radps;
    m.earth_center_m = in.earth_center_m;
    return m;
}

}  // namespace starsim::gnc
