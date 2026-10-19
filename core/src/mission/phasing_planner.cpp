#include "starsim/mission/phasing_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "starsim/frames/lvlh.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/position_targeter.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"

namespace starsim::mission {

using math::StateVector;
using math::Vec3;

namespace {
constexpr double gm = constants::earth_gm_m3ps2;  // the planner is Earth-centred, as PositionTargeter
constexpr int secant_iterations = 20;
constexpr double phase_tolerance_m = 50.0;      // [E] along-track at the boost (the coelliptic burn targets the rest)
constexpr double radius_tolerance_m = 20.0;     // [E] boost: coelliptic radius at the coelliptic epoch
constexpr double position_tolerance_m = 5.0;    // [E] coelliptic / approach initiation / mid-course aim points
constexpr double ellipsoid_sample_s = 60.0;
constexpr double max_boost_shift_s = 6.0 * 3600.0;  // [E] a waiting plan's boost epoch moves at most this far
constexpr double boost_epoch_step_s = 60.0;         // second point of the boost-epoch secant

double hohmann_departure_mps(double r1, double r2) {
    return std::sqrt(gm / r1) * (std::sqrt(2.0 * r2 / (r1 + r2)) - 1.0);
}

double wrap_pi(double angle) {
    double a = std::remainder(angle, constants::two_pi);
    if (a <= -constants::pi) a += constants::two_pi;
    return a;
}

// Secant on f(x) = 0 from x0, x1; empty when it does not reach |f| <= tolerance.
template <class F>
std::optional<double> secant(F f, double x0, double x1, double tolerance) {
    double f0 = f(x0), f1 = f(x1);
    for (int i = 0; i < secant_iterations; ++i) {
        if (std::abs(f1) <= tolerance) return x1;
        if (f1 == f0) return std::nullopt;
        const double x2 = x1 - f1 * (x1 - x0) / (f1 - f0);
        x0 = x1;
        f0 = f1;
        x1 = x2;
        f1 = f(x1);
    }
    return std::abs(f1) <= tolerance ? std::optional<double>(x1) : std::nullopt;
}

std::string limit_text(const PhasingParameters& p) {
    return "Kein Rendezvous: Phasenwinkel nicht innerhalb von " +
           std::to_string(static_cast<int>(std::lround(p.mission_limit_s / 3600.0))) + " h erreichbar";
}

// The coelliptic aim point at the coelliptic burn: behind and below the target.
Vec3 coelliptic_point(const StateVector& target_at_coelliptic, const PhasingParameters& p) {
    const double r = norm(target_at_coelliptic.position_m);
    return point_behind_below(target_at_coelliptic, coelliptic_behind_m(p, r, gm), p.coelliptic_height_m);
}

// The approach initiation point: approach_initiation_range_m from the target, coelliptic_height_m below it.
Vec3 approach_initiation_point(const StateVector& target_at_ai, const PhasingParameters& p) {
    const double range = p.approach_initiation_range_m, dh = p.coelliptic_height_m;
    return point_behind_below(target_at_ai, std::sqrt(std::max(0.0, range * range - dh * dh)), dh);
}
}  // namespace

const char* german_name(PhasingBurnKind kind) {
    switch (kind) {
        case PhasingBurnKind::Phase: return "Phasenmanöver";
        case PhasingBurnKind::Boost: return "Anhebung";
        case PhasingBurnKind::Coelliptic: return "Koelliptisches Manöver";
        case PhasingBurnKind::ApproachInitiation: return "Annäherungsbeginn";
        case PhasingBurnKind::MidCourse: return "Kurskorrektur";
        case PhasingBurnKind::Braking: return "Bremsmanöver";
    }
    return "?";
}

PhasingPlanner::PhasingPlanner(const environment::GravityModel& gravity) : gravity_(gravity) {}

StateVector PhasingPlanner::coast(const StateVector& s, time::Epoch from, time::Epoch to, double beta) const {
    if (to - from == 0.0) return s;
    return gnc::coast_with_drag(gravity_, s, from, to, beta);
}

// All angles in the target's orbit plane at the boost epoch, measured from the target there (no epoch mixing, no wrap
// at 180 deg). The schedule's required_at_boost = pi + behind / r - n_i t_h is the same condition in mean motions.
double PhasingPlanner::phase_error_rad(const StateVector& chaser_b, const StateVector& target_b, time::Epoch t_b,
                                       const PhasingParameters& p, double t_h) const {
    const StateVector target_c = coast(target_b, t_b, t_b + t_h, p.target_drag_area_per_mass_m2pkg);
    const Vec3 c = coelliptic_point(target_c, p);
    // the chaser should sit half a transfer (pi) before the aim point
    const double required = wrap_pi(in_plane_angle_rad(target_b, c) - constants::pi);
    return wrap_pi(required - in_plane_angle_rad(target_b, chaser_b.position_m));  // > 0: the chaser is late
}

double PhasingPlanner::boost_phase_error_m(const StateVector& chaser, const StateVector& target, time::Epoch epoch,
                                           const PhasingPlan& plan) const {
    const auto& p = plan.parameters;
    const time::Epoch t_b = plan.schedule.boost_epoch;
    const StateVector chaser_b = coast(chaser, epoch, t_b, p.chaser_drag_area_per_mass_m2pkg);
    const StateVector target_b = coast(target, epoch, t_b, p.target_drag_area_per_mass_m2pkg);
    const double t_h = plan.schedule.coelliptic_epoch - t_b;
    return phase_error_rad(chaser_b, target_b, t_b, p, t_h) * norm(target_b.position_m);
}

std::optional<gnc::BurnNode> PhasingPlanner::solve_phase(const StateVector& chaser, const StateVector& target,
                                                         time::Epoch epoch, const PhasingPlan& plan,
                                                         const FiniteBurnModel& engines) const {
    const auto& p = plan.parameters;
    const auto& s = plan.schedule;
    if (s.phasing_revolutions == 0) return std::nullopt;
    const time::Epoch t_p = s.phase_epoch, t_b = s.boost_epoch;
    const StateVector at_phase = coast(chaser, epoch, t_p, p.chaser_drag_area_per_mass_m2pkg);
    const StateVector target_b = coast(target, epoch, t_b, p.target_drag_area_per_mass_m2pkg);
    const Vec3 prograde = normalized(at_phase.velocity_mps);
    const double t_h = s.coelliptic_epoch - t_b;
    const auto f = [&](double dv) {
        const StateVector burned = apply_finite_burn(gravity_, at_phase, t_p, dv * prograde, engines);
        const StateVector chaser_b = coast(burned, t_p, t_b, p.chaser_drag_area_per_mass_m2pkg);
        return phase_error_rad(chaser_b, target_b, t_b, p, t_h);
    };
    const double dv0 = hohmann_departure_mps(norm(at_phase.position_m), s.phase_orbit_apoapsis_radius_m);
    const auto dv = secant(f, dv0, 1.05 * dv0, phase_tolerance_m / norm(target_b.position_m));
    if (!dv) return std::nullopt;
    return gnc::BurnNode{t_p, *dv * prograde};
}

std::optional<gnc::BurnNode> PhasingPlanner::solve_boost(const StateVector& chaser, const StateVector& target,
                                                         time::Epoch epoch, const PhasingPlan& plan,
                                                         const FiniteBurnModel& engines) const {
    const auto& p = plan.parameters;
    const time::Epoch t_b = plan.schedule.boost_epoch, t_c = plan.schedule.coelliptic_epoch;
    const StateVector at_boost = coast(chaser, epoch, t_b, p.chaser_drag_area_per_mass_m2pkg);
    const StateVector target_c = coast(target, epoch, t_c, p.target_drag_area_per_mass_m2pkg);
    const double r_c = norm(coelliptic_point(target_c, p));
    const Vec3 prograde = normalized(at_boost.velocity_mps);
    const auto f = [&](double dv) {
        const StateVector burned = apply_finite_burn(gravity_, at_boost, t_b, dv * prograde, engines);
        return norm(coast(burned, t_b, t_c, p.chaser_drag_area_per_mass_m2pkg).position_m) - r_c;
    };
    const double dv0 = hohmann_departure_mps(norm(at_boost.position_m), r_c);
    const auto dv = secant(f, dv0, 1.02 * dv0, radius_tolerance_m);
    if (!dv) return std::nullopt;
    return gnc::BurnNode{t_b, *dv * prograde};
}

std::optional<gnc::BurnNode> PhasingPlanner::solve_position(PhasingBurnKind kind, const StateVector& chaser,
                                                            const StateVector& target, time::Epoch epoch,
                                                            const PhasingPlan& plan,
                                                            const FiniteBurnModel& engines) const {
    const auto& p = plan.parameters;
    const auto& s = plan.schedule;
    const double beta_c = p.chaser_drag_area_per_mass_m2pkg, beta_t = p.target_drag_area_per_mass_m2pkg;
    PositionTargeter targeter{gravity_};
    targeter.set_drag_area_per_mass(beta_c);
    targeter.set_finite_burn(engines);
    time::Epoch burn_epoch, aim_epoch;
    Vec3 aim, guess;
    if (kind == PhasingBurnKind::Coelliptic) {
        burn_epoch = s.coelliptic_epoch;
        aim_epoch = s.approach_initiation_epoch;
        aim = approach_initiation_point(coast(target, epoch, aim_epoch, beta_t), p);
        // first guess: circular along the local horizontal
        const StateVector at_burn = coast(chaser, epoch, burn_epoch, beta_c);
        const Vec3 radial = normalized(at_burn.position_m);
        const Vec3 horizontal = normalized(at_burn.velocity_mps - dot(at_burn.velocity_mps, radial) * radial);
        guess = std::sqrt(gm / norm(at_burn.position_m)) * horizontal - at_burn.velocity_mps;
    } else {
        burn_epoch = kind == PhasingBurnKind::MidCourse ? s.approach_initiation_epoch + p.mid_course_after_s
                                                        : s.approach_initiation_epoch;
        aim_epoch = s.waypoint0_epoch;
        aim = point_behind_below(coast(target, epoch, aim_epoch, beta_t), 0.0, p.waypoint0_below_m);
        if (kind == PhasingBurnKind::ApproachInitiation) {
            // first guess: the CW two-impulse transfer to Waypoint 0
            const StateVector target_ai = coast(target, epoch, burn_epoch, beta_t);
            const StateVector chaser_ai = coast(chaser, epoch, burn_epoch, beta_c);
            const auto rel = orbit::lvlh_relative_state(target_ai, chaser_ai);
            const double r = norm(target_ai.position_m);
            const double n = std::sqrt(gm / (r * r * r));
            const auto v = orbit::cw_transfer_velocity(rel.position_m, {0.0, 0.0, p.waypoint0_below_m}, n,
                                                       aim_epoch - burn_epoch);
            if (v) guess = frames::lvlh_to_inertial(target_ai).rotate(*v - rel.velocity_mps);
        }
    }
    const auto solution = targeter.solve(chaser, epoch, burn_epoch, {aim, aim_epoch}, guess, position_tolerance_m);
    if (!solution.converged) return std::nullopt;
    if (kind == PhasingBurnKind::MidCourse && norm(solution.burn.delta_v_gcrf_mps) < p.mid_course_threshold_mps) {
        return std::nullopt;
    }
    return solution.burn;
}

gnc::BurnNode PhasingPlanner::braking(const StateVector& chaser, const StateVector& target, time::Epoch epoch,
                                      const PhasingPlan& plan) const {
    const auto& p = plan.parameters;
    const time::Epoch t = plan.schedule.waypoint0_epoch;
    const StateVector c = coast(chaser, epoch, t, p.chaser_drag_area_per_mass_m2pkg);
    const StateVector tg = coast(target, epoch, t, p.target_drag_area_per_mass_m2pkg);
    const Vec3 omega = frames::lvlh_rate_inertial_radps(tg);
    return {t, tg.velocity_mps + cross(omega, c.position_m - tg.position_m) - c.velocity_mps};
}

std::optional<gnc::BurnNode> PhasingPlanner::retarget(PhasingBurnKind kind, const StateVector& chaser,
                                                      const StateVector& target, time::Epoch epoch,
                                                      const PhasingPlan& plan, const FiniteBurnModel& engines) const {
    switch (kind) {
        case PhasingBurnKind::Phase: return solve_phase(chaser, target, epoch, plan, engines);
        case PhasingBurnKind::Boost: return solve_boost(chaser, target, epoch, plan, engines);
        case PhasingBurnKind::Coelliptic:
        case PhasingBurnKind::ApproachInitiation:
        case PhasingBurnKind::MidCourse: return solve_position(kind, chaser, target, epoch, plan, engines);
        case PhasingBurnKind::Braking: return braking(chaser, target, epoch, plan);
    }
    return std::nullopt;
}

PhasingPlan PhasingPlanner::plan(const StateVector& chaser, const StateVector& target, time::Epoch epoch,
                                 const PhasingParameters& p, const FiniteBurnModel& engines,
                                 double propellant_kg) const {
    PhasingPlan out;
    out.parameters = p;
    out.schedule = phasing_schedule(chaser, target, epoch, p, gm);
    if (!out.schedule.feasible) {
        out.reason = out.schedule.reason;
        return out;
    }
    auto& s = out.schedule;
    const double beta_c = p.chaser_drag_area_per_mass_m2pkg, beta_t = p.target_drag_area_per_mass_m2pkg;
    const double t_h = s.coelliptic_epoch - s.boost_epoch;

    // 1. A waiting plan: the boost epoch in the full model (the schedule's mean motions miss drag and short periods).
    if (s.phasing_revolutions == 0) {
        const double r = norm(target.position_m);
        const auto g = [&](double offset_s) {
            const time::Epoch t_b = epoch + offset_s;
            return phase_error_rad(coast(chaser, epoch, t_b, beta_c), coast(target, epoch, t_b, beta_t), t_b, p, t_h);
        };
        const double scheduled_s = s.boost_epoch - epoch;
        const auto solved = secant(g, scheduled_s, scheduled_s + boost_epoch_step_s, phase_tolerance_m / r);
        if (!solved || std::abs(*solved - scheduled_s) > max_boost_shift_s) {
            out.reason = "Kein Rendezvous: Zielführung konvergiert nicht (Anhebung)";
            return out;
        }
        const double shift = *solved - scheduled_s;
        s.boost_epoch = s.boost_epoch + shift;
        s.coelliptic_epoch = s.coelliptic_epoch + shift;
        s.approach_initiation_epoch = s.approach_initiation_epoch + shift;
        s.waypoint0_epoch = s.waypoint0_epoch + shift;
        if (s.boost_epoch - epoch < p.first_burn_lead_s || s.waypoint0_epoch - epoch > p.mission_limit_s) {
            out.reason = limit_text(p);
            return out;
        }
    }

    // 2. Chain the burns from the states the previous burn leaves; sample the arc against the approach ellipsoid.
    std::vector<PhasingBurnKind> kinds;
    if (s.phasing_revolutions > 0) kinds.push_back(PhasingBurnKind::Phase);
    for (const auto k : {PhasingBurnKind::Boost, PhasingBurnKind::Coelliptic, PhasingBurnKind::ApproachInitiation,
                         PhasingBurnKind::Braking}) {
        kinds.push_back(k);
    }
    const double exhaust_mps = engines.thrust_n / engines.mass_flow_kgps;
    FiniteBurnModel model = engines;
    StateVector c = chaser, tg = target;
    time::Epoch t = epoch;
    double min_metric = std::numeric_limits<double>::infinity();
    for (const auto kind : kinds) {
        const auto node = retarget(kind, c, tg, t, out, model);
        if (!node) {
            out.reason = std::string("Kein Rendezvous: Zielführung konvergiert nicht (") + german_name(kind) + ")";
            return out;
        }
        // coast to the node in sample steps, keeping the closest approach to the ellipsoid before the approach initiation
        while (t < node->epoch) {
            const time::Epoch next = std::min(node->epoch, t + ellipsoid_sample_s);
            c = coast(c, t, next, beta_c);
            tg = coast(tg, t, next, beta_t);
            t = next;
            if (t <= s.approach_initiation_epoch) min_metric = std::min(min_metric, approach_ellipsoid_metric(tg, c.position_m));
        }
        c = apply_finite_burn(gravity_, c, node->epoch, node->delta_v_gcrf_mps, model);
        const double dv = norm(node->delta_v_gcrf_mps);
        model.mass_kg *= std::exp(-dv / exhaust_mps);
        out.burns.push_back({kind, *node});
        out.total_delta_v_mps += dv;
    }
    out.min_ellipsoid_metric_before_ai = min_metric;
    if (min_metric <= 1.0) {
        out.reason = "Kein Rendezvous: Bahn tritt vor dem Annäherungsbeginn in das Annäherungsellipsoid ein";
        return out;
    }

    // 3. Budget against the propellant on board and the reserve.
    out.propellant_kg = engines.mass_kg * (1.0 - std::exp(-out.total_delta_v_mps / exhaust_mps));
    if (out.propellant_kg + p.reserve_kg > propellant_kg) {
        out.reason = "Kein Rendezvous: Treibstoff reicht nicht (" + std::to_string(std::lround(out.propellant_kg)) +
                     " kg + Reserve " + std::to_string(std::lround(p.reserve_kg)) + " kg, an Bord " +
                     std::to_string(std::lround(propellant_kg)) + " kg)";
        return out;
    }
    out.feasible = true;
    return out;
}

}  // namespace starsim::mission
