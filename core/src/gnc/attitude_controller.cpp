#include "starsim/gnc/attitude_controller.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <cstdlib>
#include <limits>

#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

struct Authority {
    double lever_moment = 0.0;  // sum of thrust * lever arm of the gimbaled engines (pitch/yaw)
    double roll_moment = 0.0;   // sum of thrust * ring radius
    double gimbal_max_rad = 0.0;  // smallest deflection limit of the running gimbaled engines
};

Authority gimbal_authority(const vehicle::FlightBody& body, double cg_station_m, double ambient_pressure_pa) {
    Authority a;
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& e : stage.engines) {
            if (!e.gimbaled || !e.producing_thrust()) continue;
            const double thrust = e.spool * e.performance->thrust_n(e.throttle, ambient_pressure_pa);
            a.lever_moment += thrust * (cg_station_m - e.gimbal_point_m.x);
            a.roll_moment += thrust * std::hypot(e.gimbal_point_m.y, e.gimbal_point_m.z);
            const double limit = e.performance->config().gimbal_max_rad;
            a.gimbal_max_rad = a.gimbal_max_rad > 0.0 ? std::min(a.gimbal_max_rad, limit) : limit;
        }
    }
    return a;
}

double fraction(double torque, double max) { return max > 0.0 ? torque / max : 0.0; }

// Differential throttling of the running fixed, canted engines (vehicle::Engine::throttle_steered): each one's torque per
// unit throttle and how far its throttle may move either way - from its group's mean throttle (the trims sum to zero on
// a symmetric ring, so the mean is the commanded throttle) to the nearer limit.
struct ThrottleAuthority {
    std::vector<size_t> index;  // engine index, stage order over all stages
    std::vector<Vec3> torque_per_unit;
    std::vector<double> range;
    size_t engine_count = 0;
    Vec3 max_torque_nm;  // per axis: every engine at its range with the favourable sign
};

ThrottleAuthority throttle_authority(const vehicle::FlightBody& body, double cg_station_m) {
    ThrottleAuthority a;
    std::map<std::string, std::pair<double, int>> group_throttle;  // sum, count
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& e : stage.engines) {
            if (e.throttle_steered() && e.mode == vehicle::EngineMode::Running && !e.starved) {
                auto& [sum, count] = group_throttle[e.group_id];
                sum += e.throttle;
                ++count;
            }
        }
    }
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& e : stage.engines) {
            const size_t i = a.engine_count++;
            if (!e.throttle_steered() || e.mode != vehicle::EngineMode::Running || e.starved) continue;
            const auto& [sum, count] = group_throttle[e.group_id];
            const double base = sum / count;
            const auto& c = e.performance->config();
            const double range = std::max(0.0, std::min(base - c.throttle_min, c.throttle_max - base));
            const Vec3 t = vehicle::throttle_torque_nm(e, cg_station_m);
            a.index.push_back(i);
            a.torque_per_unit.push_back(t);
            a.range.push_back(range);
            a.max_torque_nm += Vec3{std::abs(t.x), std::abs(t.y), std::abs(t.z)} * range;
        }
    }
    return a;
}

// Grid fins: body torque per radian of deflection of each fin, from the latest air data.
struct FinAuthority {
    std::vector<Vec3> torque_per_rad;
    std::vector<double> max_deflection_rad;
    Vec3 max_torque_nm;  // per axis, all fins at full deflection with the favourable sign
};

FinAuthority fin_authority(const vehicle::FlightBody& body, double cg_station_m) {
    FinAuthority a;
    const auto& air = body.air_data();
    const Vec3& w = body.state().angular_velocity_body_radps;
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& fin : stage.grid_fins) {
            const Vec3 arm = Vec3{fin.geometry.station_m - cg_station_m, 0.0, 0.0} +
                             aero::grid_fin_radial(fin.geometry) * fin.geometry.radius_m;
            const double k = aero::grid_fin_effectiveness(fin.geometry, air.air_velocity_body_mps + cross(w, arm),
                                                          air.air, *stage.grid_fin_table);
            const Vec3 per_rad = cross(arm, aero::grid_fin_tangential(fin.geometry)) * k;
            a.torque_per_rad.push_back(per_rad);
            a.max_deflection_rad.push_back(fin.max_deflection_rad);
            a.max_torque_nm += Vec3{std::abs(per_rad.x), std::abs(per_rad.y), std::abs(per_rad.z)} *
                               fin.max_deflection_rad;
        }
    }
    return a;
}

// Minimum-norm deflections for a torque: d = A^T (A A^T + eps I)^-1 tau (A: 3 x n, columns = torque per
// rad), scaled down uniformly if a fin would exceed its limit. Regularisation handles axes the fins
// cannot reach (e.g. roll with a degenerate layout).
std::vector<double> allocate_fins(const FinAuthority& a, const Vec3& torque) {
    const size_t n = a.torque_per_rad.size();
    math::Mat3 m;
    for (const auto& c : a.torque_per_rad) {
        const double v[3] = {c.x, c.y, c.z};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) m(i, j) += v[i] * v[j];
    }
    const double trace = m(0, 0) + m(1, 1) + m(2, 2);
    if (trace <= 0.0) return std::vector<double>(n, 0.0);
    for (int i = 0; i < 3; ++i) m(i, i) += 1e-6 * trace;
    const auto solved = math::solve(m, torque);
    if (!solved) return std::vector<double>(n, 0.0);  // refused (cannot happen with the regularisation): fins neutral
    const double y[3] = {solved->x, solved->y, solved->z};
    std::vector<double> d(n);
    double scale = 1.0;
    for (size_t i = 0; i < n; ++i) {
        const Vec3& c = a.torque_per_rad[i];
        d[i] = c.x * y[0] + c.y * y[1] + c.z * y[2];
        if (std::abs(d[i]) > a.max_deflection_rad[i]) scale = std::min(scale, a.max_deflection_rad[i] / std::abs(d[i]));
    }
    for (auto& v : d) v *= scale;
    return d;
}

// Minimum-norm solution d = A^T (A A^T + eps I)^-1 b of A d = b (A: 3 x n, given by its columns).
std::vector<double> minimum_norm(const std::vector<Vec3>& columns, const Vec3& b) {
    FinAuthority a;
    a.torque_per_rad = columns;
    a.max_deflection_rad.assign(columns.size(), std::numeric_limits<double>::infinity());
    return allocate_fins(a, b);
}

// Flaps: modelled aerodynamic moment about the CG (hull + flaps at their current deflections) and the flap
// moments' derivative per radian of deflection (numeric), from the latest air data.
struct FlapAuthority {
    bool active = false;
    std::vector<Vec3> torque_per_rad;
    std::vector<double> deflection_rad;
    std::vector<double> max_deflection_rad;
    std::vector<double> lower_rad, upper_rad;  // reachable within flap_reach_horizon_s (rate limit), inside the stops
    Vec3 aero_moment_nm;
    Vec3 max_torque_nm;  // per axis, half of each flap's range with the favourable sign
};

constexpr double flap_difference_step_rad = 1.0 * constants::deg_to_rad;
constexpr double flap_stop_secant_span_rad = 20.0 * constants::deg_to_rad;  // [E] a third of the peak derivative at a stop
constexpr double flap_at_stop_rad = 0.5 * constants::deg_to_rad;           // [E] a flap this close to a stop is at it
// [E] At a stop the 1 deg difference of sin^2 is ~5 % of the 20 deg secant; a quarter separates "vanishing" from a
// flap whose incidence is not at the sin^2 extremum (flow at an angle to the flap's stop position).
constexpr double flap_tiny_column_fraction = 0.25;
// [E] Newton step limit per call: with little authority (thin air) the minimum-norm step asked for far more than the
// range and the flaps banged between their stops every cycle; half the range keeps large trim changes (a 1e6 N m yaw
// demand at 65 km moves the aft flaps by more than 20 deg).
constexpr double flap_max_step_rad = 45.0 * constants::deg_to_rad;
// [E] The allocation's box: what the rate-limited actuator reaches within this horizon (~5 deg at 20 deg/s). Steps of
// 40-80 deg that the flaps could not follow made the remainder for the RCS wrong and limit-cycled the aft pair in the
// terminal phase (l5 Task 10, active-set allocation). Sweep (LEO campaigns 14 flights + lunar fixtures): full range
// 7/14 LEO gates missed by 0.3-1.9 km; 0.25 s all 14 gates 0 m and the lunar nominal caught 13.2 t; 0.5 s / 1 s / 2 s
// lost the lunar nominal (dry 0.6 km off, impact, dry) - the terminal phase sits at a stability margin.
// (A damped least-squares step, 2 % of the trace, was tried against a 2-cycle in an instant-follow unit iteration:
// it broke 11 of 14 LEO return gates - not used.)
constexpr double flap_reach_horizon_s = 0.25;

double dynamic_pressure_pa(const vehicle::FlightBody& body) {
    const auto& air = body.air_data();
    const double v = norm(air.air_velocity_body_mps);
    return 0.5 * air.air.density_kg_m3 * v * v;
}

Vec3 flap_moment(const vehicle::Flap& flap, double deflection_rad, const aero::FlapTable& table,
                 const vehicle::AirData& air, const Vec3& rate_body, double cg_station_m) {
    const Vec3 arm = aero::flap_centroid(flap.geometry, deflection_rad) - Vec3{cg_station_m, 0.0, 0.0};
    const auto force =
        aero::flap_force(flap.geometry, deflection_rad, air.air_velocity_body_mps + cross(rate_body, arm), air.air, table);
    return cross(arm, force.force_body_n);
}

FlapAuthority flap_authority(const vehicle::FlightBody& body, double cg_station_m) {
    FlapAuthority a;
    const auto& air = body.air_data();
    if (air.air.density_kg_m3 <= 0.0 || body.vehicle().flap_count() == 0) return a;
    const Vec3& w = body.state().angular_velocity_body_radps;
    a.aero_moment_nm = AttitudeController::modelled_hull_moment_nm(body);
    for (const auto& stage : body.vehicle().stages()) {
        if (!stage.flap_table) continue;
        for (const auto& flap : stage.flaps) {
            const double d = flap.deflection_rad;
            const double max = flap.max_deflection_rad;
            a.aero_moment_nm += flap_moment(flap, d, *stage.flap_table, air, w, cg_station_m);
            const auto column = [&](double lo, double hi) {
                return (flap_moment(flap, hi, *stage.flap_table, air, w, cg_station_m) -
                        flap_moment(flap, lo, *stage.flap_table, air, w, cg_station_m)) /
                       (hi - lo);
            };
            Vec3 per_rad = column(std::max(0.0, d - flap_difference_step_rad), std::min(max, d + flap_difference_step_rad));
            // The flap force goes with sin^2 of its incidence: its derivative vanishes at both stops (folded, broadside).
            // The local difference of a flap AT a stop sees almost no authority and the flap never comes back (l5 Task 10:
            // flaps held 90/0/0/90 from 105 km, their roll moment outgrew the RCS at 83 km and the ship rolled away) -
            // there the secant over a wide span into the range is used. Only at the stop: a flap trimmed near it (the
            // aerocapture pass holds the forward flaps at ~82 deg) keeps its local column, a secant there chattered 0/90.
            const bool at_lower = d <= flap_at_stop_rad, at_upper = d >= max - flap_at_stop_rad;
            if (at_lower || at_upper) {
                const Vec3 secant = at_lower ? column(0.0, std::min(max, flap_stop_secant_span_rad))
                                             : column(std::max(0.0, max - flap_stop_secant_span_rad), max);
                if (norm(per_rad) < flap_tiny_column_fraction * norm(secant)) per_rad = secant;
            }
            a.torque_per_rad.push_back(per_rad);
            a.deflection_rad.push_back(d);
            a.max_deflection_rad.push_back(flap.max_deflection_rad);
            const double reach = flap.rate_rad_s > 0.0 ? flap.rate_rad_s * flap_reach_horizon_s : max;
            a.lower_rad.push_back(std::max(0.0, d - reach));
            a.upper_rad.push_back(std::min(max, d + reach));
            a.max_torque_nm += Vec3{std::abs(per_rad.x), std::abs(per_rad.y), std::abs(per_rad.z)} *
                               (0.5 * flap.max_deflection_rad);
        }
    }
    a.active = !a.torque_per_rad.empty();
    return a;
}

// Fraction of the null-space distance to mid-range removed per call (redundant flap setting, slow drift).
constexpr double flap_centring_gain = 0.2;

// Box-constrained flap step for the moment `needed` (linear model: sum of column_i delta_i): active set - the
// minimum-norm step over the free flaps, every flap it would push beyond its box (a stop, or what its rate reaches
// within flap_reach_horizon_s) is fixed at that bound and the rest
// re-solved for what the fixed ones leave. Then the flaps drift towards mid-range in the columns' null space, scaled
// to stay inside the range, so the drift is moment-neutral (a clamp after the step or the drift was not: flaps at a
// stop took large redundant moves and the allocation chattered 0/90 deg at moderate q - l5 Task 10).
std::vector<double> allocate_flaps(const FlapAuthority& fa, const Vec3& needed) {
    const size_t n = fa.torque_per_rad.size();
    std::vector<double> delta(n, 0.0);
    std::vector<bool> fixed(n, false);
    std::vector<size_t> free;
    for (size_t pass = 0; pass <= n; ++pass) {
        free.clear();
        std::vector<Vec3> columns;
        Vec3 rest = needed;
        for (size_t i = 0; i < n; ++i) {
            if (fixed[i]) {
                rest -= fa.torque_per_rad[i] * delta[i];
            } else {
                free.push_back(i);
                columns.push_back(fa.torque_per_rad[i]);
            }
        }
        if (free.empty()) break;
        std::vector<double> step = minimum_norm(columns, rest);
        double largest = 0.0;
        for (const double x : step) largest = std::max(largest, std::abs(x));
        if (largest > flap_max_step_rad) {
            for (auto& x : step) x *= flap_max_step_rad / largest;  // same direction, inside the linear reach
        }
        bool clamped = false;
        for (size_t k = 0; k < free.size(); ++k) {
            const size_t i = free[k];
            const double target = fa.deflection_rad[i] + step[k];
            const double bounded = std::clamp(target, fa.lower_rad[i], fa.upper_rad[i]);
            delta[i] = bounded - fa.deflection_rad[i];
            if (bounded != target) {
                fixed[i] = true;
                clamped = true;
            }
        }
        if (!clamped) break;
    }
    // Moment-neutral drift towards mid-range over ALL flaps (a flap fixed at a stop comes back this way):
    // z = to_mid - A^T (A A^T)^-1 A to_mid; a flap at a stop that z would push outward is left out and z solved again.
    std::vector<bool> drifting(n, true);
    std::vector<double> drift(n, 0.0);
    for (size_t pass = 0; pass <= n; ++pass) {
        std::vector<Vec3> columns;
        std::vector<size_t> index;
        std::vector<double> to_mid;
        Vec3 mid_moment;
        for (size_t i = 0; i < n; ++i) {
            if (!drifting[i]) continue;
            const double v = 0.5 * fa.max_deflection_rad[i] - (fa.deflection_rad[i] + delta[i]);
            columns.push_back(fa.torque_per_rad[i]);
            index.push_back(i);
            to_mid.push_back(v);
            mid_moment += fa.torque_per_rad[i] * v;
        }
        std::fill(drift.begin(), drift.end(), 0.0);
        if (columns.empty()) break;
        const std::vector<double> range_part = minimum_norm(columns, mid_moment);
        bool excluded = false;
        for (size_t k = 0; k < index.size(); ++k) {
            const size_t i = index[k];
            drift[i] = to_mid[k] - range_part[k];
            const double at = fa.deflection_rad[i] + delta[i];
            const bool outward = (at <= fa.lower_rad[i] && drift[i] < 0.0) || (at >= fa.upper_rad[i] && drift[i] > 0.0);
            if (outward) {
                drifting[i] = false;
                excluded = true;
            }
        }
        if (!excluded) break;
    }
    double scale = flap_centring_gain;
    for (size_t i = 0; i < n; ++i) {
        const double at = fa.deflection_rad[i] + delta[i];
        if (drift[i] > 0.0) scale = std::min(scale, (fa.upper_rad[i] - at) / drift[i]);
        if (drift[i] < 0.0) scale = std::min(scale, (fa.lower_rad[i] - at) / drift[i]);
    }
    scale = std::max(0.0, scale);
    for (size_t i = 0; i < n; ++i) delta[i] += scale * drift[i];
    return delta;
}

// Thrust centroid this far off the axis: the engine set is asymmetric (roll by RCS, see allocate).
constexpr double asymmetric_thrust_offset_m = 0.05;

// Fraction of the maximum angular deceleration planned for stopping a rotation (margin for lag and
// thrust changes).
constexpr double braking_margin = 0.5;

// Rate command for an attitude error: linear (natural frequency / damping) for small errors, limited for
// large ones to the rate that can still be stopped with the available angular acceleration, so a
// saturated actuator does not overshoot.
// With an actuator delay d (e.g. the gimbals' slew to the opposite stop) the rate w still turns w d before braking
// starts: w d + w^2 / (2 b) <= |error|.
double rate_command(double error_rad, double linear_gain, double max_angular_acceleration, double actuator_delay_s = 0.0) {
    const double linear = linear_gain * error_rad;
    if (max_angular_acceleration <= 0.0) return linear;
    const double b = braking_margin * max_angular_acceleration;
    const double d = actuator_delay_s;
    const double braking = d > 0.0 ? b * (std::sqrt(d * d + 2.0 * std::abs(error_rad) / b) - d)
                                   : std::sqrt(2.0 * braking_margin * max_angular_acceleration * std::abs(error_rad));
    return std::copysign(std::min(std::abs(linear), braking), error_rad);
}

}  // namespace

std::vector<double> AttitudeController::feathered_grid_fins(const vehicle::FlightBody& body) {
    std::vector<double> deflections;
    const auto& air = body.air_data();
    const Vec3& w = body.state().angular_velocity_body_radps;
    const double cg = body.vehicle().mass_properties().cg_station_m;
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& fin : stage.grid_fins) {
            // Fin normal n(d) = t cos d + e_x sin d; no normal flow when v_t cos d + v_x sin d = 0.
            const Vec3 arm = Vec3{fin.geometry.station_m - cg, 0.0, 0.0} +
                             aero::grid_fin_radial(fin.geometry) * fin.geometry.radius_m;
            const Vec3 v = air.air_velocity_body_mps + cross(w, arm);
            const double v_t = dot(v, aero::grid_fin_tangential(fin.geometry));
            const double d = std::abs(v.x) > 1e-6 ? std::atan(-v_t / v.x) : 0.0;
            deflections.push_back(std::clamp(d, -fin.max_deflection_rad, fin.max_deflection_rad));
        }
    }
    return deflections;
}

ControlOutput AttitudeController::allocate(const vehicle::FlightBody& body, double ambient_pressure_pa,
                                           const Vec3& desired_torque, bool grid_fins, bool flaps) {
    const auto mass = body.vehicle().mass_properties();
    const Authority a = gimbal_authority(body, mass.cg_station_m, ambient_pressure_pa);
    const Vec3 rcs_max = body.vehicle().max_rcs_torque_nm();
    ControlOutput out;
    Vec3 torque = desired_torque;
    if (grid_fins && body.vehicle().grid_fin_count() > 0) {
        // Fins first (aerodynamic, no propellant); the remainder goes to gimbals / RCS.
        const FinAuthority fins = fin_authority(body, mass.cg_station_m);
        out.grid_fin_rad = allocate_fins(fins, torque);
        for (size_t i = 0; i < out.grid_fin_rad.size(); ++i) torque -= fins.torque_per_rad[i] * out.grid_fin_rad[i];
    }
    if (!flaps && body.vehicle().flap_count() > 0) {
        // Flaps not steering (e.g. the ship's landing burn): the modelled aerodynamic moment (hull + flaps as they
        // are) is cancelled by the gimbals / RCS instead.
        const FlapAuthority fa = flap_authority(body, mass.cg_station_m);
        if (fa.active) torque -= fa.aero_moment_nm;
    }
    if (flaps) {
        const FlapAuthority fa = flap_authority(body, mass.cg_station_m);
        if (fa.active) {
            // Newton step: the flaps add what the modelled aero moment lacks of the wanted net torque.
            const Vec3 needed = torque - fa.aero_moment_nm;
            const std::vector<double> delta = allocate_flaps(fa, needed);
            const size_t n = delta.size();
            out.flap_rad.resize(n);
            Vec3 achieved;
            for (size_t i = 0; i < n; ++i) {
                out.flap_rad[i] = std::clamp(fa.deflection_rad[i] + delta[i], 0.0, fa.max_deflection_rad[i]);
                achieved += fa.torque_per_rad[i] * (out.flap_rad[i] - fa.deflection_rad[i]);
            }
            torque = needed - achieved;  // remainder for gimbals / RCS
        }
    }
    // Off-axis thrust (an asymmetric engine set, e.g. two of the ship's three centre Raptors): the moment of the
    // running engines at neutral gimbal is cancelled first, so the gimbals deliver the wanted net torque.
    // The same set's thrust centroid off the axis also makes differential (roll) gimbaling push sideways: then roll
    // goes to the RCS.
    bool asymmetric = false;
    if (a.lever_moment > 0.0) {
        Vec3 moment_sum;
        double thrust_sum = 0.0;
        for (const auto& stage : body.vehicle().stages()) {
            for (const auto& e : stage.engines) {
                if (!e.producing_thrust()) continue;
                const double thrust = e.spool * e.performance->thrust_n(e.throttle, ambient_pressure_pa);
                const Vec3 arm = e.gimbal_point_m - Vec3{mass.cg_station_m, 0.0, 0.0};
                torque -= cross(arm, Vec3{thrust, 0.0, 0.0});
                moment_sum += e.gimbal_point_m * thrust;
                thrust_sum += thrust;
            }
        }
        asymmetric = thrust_sum > 0.0 && std::hypot(moment_sum.y, moment_sum.z) / thrust_sum > asymmetric_thrust_offset_m;
    }
    // Moments of a deflection (see Vehicle::update_engines): pitch -> +tau_y, yaw -> -tau_z, roll -> +tau_x.
    if (a.lever_moment > 0.0) {
        out.gimbal.pitch_rad = torque.y / a.lever_moment;
        out.gimbal.yaw_rad = -torque.z / a.lever_moment;
    } else {
        // No gimbals: differential throttling of fixed, canted engines first (pitch/yaw only - their thrust has no
        // tangential part), the rest to the RCS.
        Vec3 rest{0.0, torque.y, torque.z};
        const ThrottleAuthority ta = throttle_authority(body, mass.cg_station_m);
        if (!ta.index.empty()) {
            FinAuthority fa;
            fa.torque_per_rad = ta.torque_per_unit;
            fa.max_deflection_rad = ta.range;
            const std::vector<double> trims = allocate_fins(fa, rest);
            out.throttle_trim.assign(ta.engine_count, 0.0);
            for (size_t k = 0; k < trims.size(); ++k) {
                out.throttle_trim[ta.index[k]] = trims[k];
                rest -= ta.torque_per_unit[k] * trims[k];
            }
        }
        out.rcs_torque_fraction.y = fraction(rest.y, rcs_max.y);
        out.rcs_torque_fraction.z = fraction(rest.z, rcs_max.z);
    }
    if (a.roll_moment > 0.0 && !asymmetric) {
        out.gimbal.roll_rad = torque.x / a.roll_moment;
    } else {
        out.rcs_torque_fraction.x = fraction(torque.x, rcs_max.x);
    }
    return out;
}

ControlOutput AttitudeController::steer_rates(const vehicle::FlightBody& body, double ambient_pressure_pa,
                                              const Vec3& rates_body_radps) const {
    const auto mass = body.vehicle().mass_properties();
    const Vec3& w = body.state().angular_velocity_body_radps;
    const Vec3& inertia = mass.inertia_kg_m2;
    const double k = 2.0 * gains_.damping_ratio * gains_.natural_frequency_rad_s;
    const Vec3 torque{inertia.x * k * (rates_body_radps.x - w.x), inertia.y * k * (rates_body_radps.y - w.y),
                      inertia.z * k * (rates_body_radps.z - w.z)};
    return allocate(body, ambient_pressure_pa, torque, gains_.grid_fins, gains_.flaps);
}

ControlOutput AttitudeController::steer(const vehicle::FlightBody& body, double ambient_pressure_pa,
                                        const Vec3& axis_gcrf, const Vec3& roll_reference_gcrf,
                                        const Feedforward* feedforward) const {
    const auto& state = body.state();
    const auto mass = body.vehicle().mass_properties();

    const Vec3 error = pointing_error(body, axis_gcrf, roll_reference_gcrf);
    const Vec3 rotation{0.0, error.y, error.z};
    const double roll_error = error.x;

    // Torque = I * kd * (rate_command - rate); for small errors this is I * (wn^2 e - kd w).
    const double wn = gains_.natural_frequency_rad_s;
    const double kd = 2.0 * gains_.damping_ratio * wn;
    const Vec3& w = state.angular_velocity_body_radps;
    const Vec3& inertia = mass.inertia_kg_m2;
    const Authority a = gimbal_authority(body, mass.cg_station_m, ambient_pressure_pa);
    // Without gimbals the differentially throttled engines add to the RCS.
    const Vec3 rcs_max = body.vehicle().max_rcs_torque_nm() +
                         (a.lever_moment > 0.0 ? Vec3{} : throttle_authority(body, mass.cg_station_m).max_torque_nm);
    Vec3 fin_max =
        gains_.grid_fins && body.vehicle().grid_fin_count() > 0 ? fin_authority(body, mass.cg_station_m).max_torque_nm : Vec3{};
    if (gains_.flaps) fin_max += flap_authority(body, mass.cg_station_m).max_torque_nm;
    auto max_acceleration = [&](double gimbal_moment, double rcs_torque, double fin_torque, double moment_of_inertia) {
        const double torque = (gimbal_moment > 0.0 ? gimbal_moment * std::sin(a.gimbal_max_rad) : rcs_torque) + fin_torque;
        return moment_of_inertia > 0.0 ? torque / moment_of_inertia : 0.0;
    };
    const double gain = wn * wn / kd;
    const Vec3 ff_rate = feedforward ? Vec3{feedforward->roll_rate ? feedforward->rate_body_radps.x : 0.0,
                                            feedforward->rate_body_radps.y, feedforward->rate_body_radps.z}
                                     : Vec3{};
    const Vec3 ff_acceleration =
        feedforward ? Vec3{0.0, feedforward->acceleration_body_radps2.y, feedforward->acceleration_body_radps2.z} : Vec3{};
    Vec3 rate{rate_command(roll_error, gain, max_acceleration(a.roll_moment, rcs_max.x, fin_max.x, inertia.x)) + ff_rate.x,
              rate_command(rotation.y, gain, max_acceleration(a.lever_moment, rcs_max.y, fin_max.y, inertia.y),
                           gains_.actuator_delay_s) + ff_rate.y,
              rate_command(rotation.z, gain, max_acceleration(a.lever_moment, rcs_max.z, fin_max.z, inertia.z),
                           gains_.actuator_delay_s) + ff_rate.z};
    if (const double limit = gains_.rcs_max_rate_radps; limit > 0.0) {
        if (!(a.lever_moment > 0.0)) {
            // Pitch and yaw together: the slew about the combined axis stays within the limit.
            const double r = std::hypot(rate.y, rate.z);
            if (r > limit) {
                rate.y *= limit / r;
                rate.z *= limit / r;
            }
        }
    }
    if (const double limit = gains_.rcs_max_roll_rate_radps; limit > 0.0 && !(a.roll_moment > 0.0)) {
        rate.x = std::clamp(rate.x, -limit, limit);
    }
    Vec3 torque{inertia.x * kd * (rate.x - w.x), inertia.y * (kd * (rate.y - w.y) + ff_acceleration.y),
                inertia.z * (kd * (rate.z - w.z) + ff_acceleration.z)};
    if (gains_.hold_deadband_rad > 0.0) {
        const double errors[3] = {roll_error, rotation.y, rotation.z};
        const double rate_errors[3] = {w.x - ff_rate.x, w.y - ff_rate.y, w.z - ff_rate.z};
        double* components[3] = {&torque.x, &torque.y, &torque.z};
        for (int i = 0; i < 3; ++i) {
            if (std::abs(errors[i]) < gains_.hold_deadband_rad &&
                std::abs(rate_errors[i]) < gains_.hold_rate_deadband_radps) {
                *components[i] = 0.0;
            }
        }
    }
    if (feedforward) torque += feedforward->torque_body_nm;
    if (gains_.flaps && feedforward && feedforward->flap_rest_rad.size() == body.vehicle().flap_count() &&
        !feedforward->flap_rest_rad.empty() &&
        dynamic_pressure_pa(body) < gains_.flap_rest_dynamic_pressure_pa) {
        // no authority: the flaps rest (e.g. in the trim of the coming entry), gimbals / RCS cancel the modelled moment
        ControlOutput out = allocate(body, ambient_pressure_pa, torque, gains_.grid_fins, false);
        out.flap_rad = feedforward->flap_rest_rad;
        return out;
    }
    return allocate(body, ambient_pressure_pa, torque, gains_.grid_fins, gains_.flaps);
}

Vec3 AttitudeController::modelled_hull_moment_nm(const vehicle::FlightBody& body) {
    const auto& air = body.air_data();
    if (air.air.density_kg_m3 <= 0.0) return {};
    const auto hull = body.vehicle().aero().compute(air.air_velocity_body_mps, air.air.density_kg_m3,
                                                    air.air.speed_of_sound_mps,
                                                    body.vehicle().any_engine_producing_thrust(), air.mean_free_path_m);
    const double cg_station_m = body.vehicle().mass_properties().cg_station_m;
    return cross(Vec3{hull.normal_force_station_m - cg_station_m, 0.0, 0.0}, hull.force_body_n);
}

Vec3 AttitudeController::pointing_error(const vehicle::FlightBody& body, const Vec3& axis_gcrf,
                                        const Vec3& roll_reference_gcrf) {
    const auto& state = body.state();
    // the axis error as a rotation vector in body axes
    const Vec3 target_body = state.attitude.inverse_rotate(math::normalized(axis_gcrf));
    const Vec3 axis_error = cross(Vec3{1.0, 0.0, 0.0}, target_body);
    const double sin_error = norm(axis_error);
    const double angle = std::atan2(sin_error, target_body.x);
    const Vec3 rotation = sin_error > 1e-12 ? axis_error * (angle / sin_error) : Vec3{};
    double roll_error = 0.0;
    if (norm(roll_reference_gcrf) > 0.0) {
        const Vec3 ref_body = state.attitude.inverse_rotate(roll_reference_gcrf);
        roll_error = std::atan2(ref_body.z, ref_body.y);  // rotate +y towards the reference about +x
    }
    return {roll_error, rotation.y, rotation.z};
}

Vec3 AttitudeController::integrate_error(const Vec3& integral, const Vec3& error, double dt) const {
    if (!(gains_.integral_gain > 0.0)) return integral;
    const auto step = [&](double sum, double e) { return std::abs(e) < gains_.integral_window_rad ? sum + e * dt : sum; };
    return {step(integral.x, error.x), step(integral.y, error.y), step(integral.z, error.z)};
}

Vec3 AttitudeController::integral_torque(const vehicle::FlightBody& body, const Vec3& integral) const {
    const Vec3& inertia = body.vehicle().mass_properties().inertia_kg_m2;
    const double ki = gains_.integral_gain;
    return {inertia.x * ki * integral.x, inertia.y * ki * integral.y, inertia.z * ki * integral.z};
}

}  // namespace starsim::gnc
