#include "starsim/mission/lunar_outbound.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/lambert.hpp"

namespace starsim::mission {

using math::StateVector;
using math::Vec3;

namespace {
constexpr double nrho_flyby_tolerance_m = 1000.0;  // [E] swing-by aim at the NRHO insertion point: MCCs refine it
constexpr double nrho_mcc_tolerance_m = 100.0;     // [E] MCC aim at the flyby point; re-planned swing-by aim
// Swing-by inclinations searched (MOON_ME): the B-plane node is free, so the inclination picks the swing-by geometry.
constexpr double flyby_inclination_step_rad = 15.0 * constants::deg_to_rad;       // [E] coarse grid
constexpr double flyby_inclination_tolerance_rad = 0.25 * constants::deg_to_rad;  // [E] golden-section refinement
constexpr double min_flyby_to_insertion_s = 0.25 * 86400.0;  // [E] shorter arcs are not considered
// Search for the insertion's true anomaly on the reference: step and resolution.
constexpr double anomaly_search_step_s = 600.0;
constexpr double anomaly_search_tolerance_s = 1.0;
constexpr double apsis_now_s = 1.0;  // an apsis less than this ahead or behind counts as "now"
constexpr double far_future_s = 1e30;  // "no apsis found yet"
// LLO plane through a site: a site this close to the arrival axis (sine of the angle) lies in every plane through it.
constexpr double site_on_axis_sin = 0.01;  // [E] ~0.6 deg
// Fixed point between the TLI into a plane and the asymptote it arrives on: done when the plane moves less than this
// (1e-5 rad: ~17 m at the lunar surface), at most this many solves.
constexpr double plane_fixed_point_rad = 1e-5;
constexpr int plane_fixed_point_solves = 6;

bool debug_enabled() { return std::getenv("STARSIM_DEBUG_OUTBOUND") != nullptr; }

// Time from the current point to the next passage of mean anomaly `target` (elliptic orbit).
double time_to_mean_anomaly(double mean_anomaly, double target, double mean_motion) {
    double delta = std::fmod(target - mean_anomaly, 2.0 * constants::pi);
    if (delta < 0.0) delta += 2.0 * constants::pi;
    double dt = delta / mean_motion;
    const double period = 2.0 * constants::pi / mean_motion;
    if (period - dt < apsis_now_s) dt = 0.0;  // just passed: take it now, not one revolution later
    return dt;
}
}  // namespace

StateVector moon_relative(const ephemeris::Ephemeris& ephemeris, const StateVector& state, time::Epoch t) {
    const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t);
    return {state.position_m - moon.position_m, state.velocity_mps - moon.velocity_mps};
}

LunarOutboundPlanner::LunarOutboundPlanner(const environment::GravityModel& gravity,
                                           const ephemeris::Ephemeris& ephemeris,
                                           const frames::BodyOrientation& moon_orientation)
    : ephemeris_(ephemeris),
      moon_orientation_(moon_orientation),
      transfer_(gravity, ephemeris, moon_orientation),
      position_(gravity),
      gm_moon_(ephemeris.gm_m3ps2(ephemeris::Body::Moon)) {}

Vec3 llo_plane_through_site(const frames::BodyOrientation& moon, double latitude_rad, double longitude_rad,
                            time::Epoch touchdown, const Vec3& arrival_asymptote_moon_inertial,
                            const std::optional<Vec3>& arrival_normal_moon_inertial) {
    const auto rotation = moon.fixed_to_gcrf(touchdown).rotation;
    const Vec3 site = rotation * Vec3{std::cos(latitude_rad) * std::cos(longitude_rad),
                                      std::cos(latitude_rad) * std::sin(longitude_rad), std::sin(latitude_rad)};
    const Vec3 pole = rotation * Vec3{0.0, 0.0, 1.0};
    const Vec3 a = math::normalized(arrival_asymptote_moon_inertial);
    const Vec3 sense = arrival_normal_moon_inertial ? *arrival_normal_moon_inertial : pole;  // else prograde
    Vec3 n = cross(a, site);
    if (norm(n) < site_on_axis_sin) {
        // The site lies in every plane through the axis: the one closest to the sense plane (no turn), else - the
        // sense parallel to the axis - any plane through it.
        n = sense - a * dot(a, sense);
        if (norm(n) < site_on_axis_sin) n = cross(a, std::abs(a.x) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0});
    }
    n = math::normalized(n);
    return dot(n, sense) < 0.0 ? -n : n;
}

double llo_approach_azimuth(const frames::BodyOrientation& moon, double latitude_rad, double longitude_rad,
                            time::Epoch touchdown, const Vec3& plane_normal_moon_inertial, double llo_radius_m,
                            double gm_moon_m3ps2) {
    const auto transform = moon.fixed_to_gcrf(touchdown);
    const Vec3 s = transform.rotation * Vec3{std::cos(latitude_rad) * std::cos(longitude_rad),
                                             std::cos(latitude_rad) * std::sin(longitude_rad), std::sin(latitude_rad)};
    const Vec3 pole = transform.rotation * Vec3{0.0, 0.0, 1.0};
    // circular speed along the plane over the site, minus the Moon's rotation (as LunarDescentPlanner's pass azimuth)
    const Vec3 inertial_v = math::normalized(cross(plane_normal_moon_inertial, s)) * std::sqrt(gm_moon_m3ps2 / llo_radius_m);
    const Vec3 ground_v = inertial_v - cross(frames::rotation_vector(transform), s * llo_radius_m);
    Vec3 east = cross(pole, s);
    if (norm(east) < site_on_axis_sin) east = cross(Vec3{1.0, 0.0, 0.0}, s);  // at a pole: any horizontal frame
    east = math::normalized(east);
    const Vec3 north = cross(s, east);
    return std::atan2(dot(ground_v, east), dot(ground_v, north));
}

LunarArrivalTarget LunarOutboundPlanner::llo_target(const LunarOutboundParameters& p, const OutboundPlan* plan) const {
    LunarArrivalTarget target{p.llo.altitude_m, p.llo.inclination_rad, p.arrival_epoch};
    target.plane_normal_moon_inertial = p.llo.plane_normal_moon_inertial;
    if (plan && plan->llo_plane_normal_moon_inertial) {
        // the plan's plane holds its site only on the plan's asymptote: the solves after the plan hold its arrival
        target.plane_normal_moon_inertial = plan->llo_plane_normal_moon_inertial;
        target.periselene_epoch = plan->llo_arrival_epoch;
        target.hold_arrival_epoch = true;
    }
    return target;
}

TransferSolution LunarOutboundPlanner::solve_through_site(const StateVector& state, time::Epoch epoch,
                                                          time::Epoch burn_epoch, const LunarOutboundParameters& p,
                                                          OutboundPlan& plan) const {
    const auto& site = *p.llo_site;
    // The inclination-targeted transfer first (polar by default: the plan without a site); its arrival epoch is held
    // and its plane turned about its asymptote onto the site - the least change from that transfer. The arrival time
    // is free in that solve and drifts along a nearly flat family (Mons Mouton 2026-11-13: +19 h for the polar plane,
    // -12 h for a plane solved freely, 2 m/s of TLI apart), and a day turns the asymptote by ~13 deg: held, the plane
    // stays the plane through the site, also in the mid-course corrections after the plan.
    TransferSolution solution = transfer_.solve(state, epoch, burn_epoch, llo_target(p));
    std::optional<Vec3> warm;
    Vec3 axis;
    // The sense of the plane: the first transfer's (turned, not reversed). Measured against the other sense (TLI + LOI,
    // 2026-11-02 + 4 d, touchdown arrival + 8 h): Apollo 11 4,005.8 (i 167.1 deg) vs 4,000.7 m/s (i 13.5 deg), Apollo 15
    // 3,998.3 (i 31.3 deg) vs 4,008.0 m/s (i 148.4 deg), Mons Mouton (hls_lunar_llo) 3,240 vs 3,488 m/s TLI - the kept
    // sense is never more than ~5 m/s dearer, the other can be 250 m/s dearer; choosing by cost would double the solves.
    std::optional<Vec3> sense;
    time::Epoch arrival = p.arrival_epoch;
    if (solution.converged) {
        axis = arrival_axis(solution.arrival.relative_state, gm_moon_);
        sense = cross(solution.arrival.relative_state.position_m, solution.arrival.relative_state.velocity_mps);
        warm = solution.burn.delta_v_gcrf_mps;
        arrival = solution.arrival.periselene_epoch;
    } else {  // the Earth two-body arc from the burn to the Moon's centre at the planned arrival
        const StateVector at_burn = position_.coast(state, epoch, burn_epoch);
        const auto moon = ephemeris_.state_gcrf(ephemeris::Body::Moon, p.arrival_epoch);
        const auto centre = orbit::solve_lambert(at_burn.position_m, moon.position_m, p.arrival_epoch - burn_epoch,
                                                 ephemeris_.gm_m3ps2(ephemeris::Body::Earth),
                                                 cross(at_burn.position_m, at_burn.velocity_mps));
        axis = centre ? math::normalized(centre->arrival_velocity_mps - moon.velocity_mps)
                      : -math::normalized(moon.velocity_mps);
    }
    const time::Epoch touchdown = site.touchdown ? *site.touchdown : arrival + site.first_pass_after_arrival_s;
    Vec3 normal = llo_plane_through_site(moon_orientation_, site.latitude_rad, site.longitude_rad, touchdown, axis, sense);
    for (int i = 0; i < plane_fixed_point_solves; ++i) {
        auto target = llo_target(p);
        target.plane_normal_moon_inertial = normal;
        target.periselene_epoch = arrival;
        // turned with the arrival free first (a large turn with the arrival held creeps: Apollo 11 77 deg, 40 Newton
        // steps), then held where it arrived
        target.hold_arrival_epoch = i > 0;
        solution = transfer_.solve(state, epoch, burn_epoch, target, warm);
        if (!solution.converged && warm) solution = transfer_.solve(state, epoch, burn_epoch, target);
        if (!solution.converged) return solution;
        warm = solution.burn.delta_v_gcrf_mps;
        arrival = solution.arrival.periselene_epoch;
        axis = arrival_axis(solution.arrival.relative_state, gm_moon_);
        const Vec3 next = llo_plane_through_site(moon_orientation_, site.latitude_rad, site.longitude_rad,
                                                 touchdown, axis, normal);
        const double moved = std::acos(std::clamp(dot(next, normal), -1.0, 1.0));
        if (debug_enabled()) {
            std::printf("[outbound] LLO plane through the site, solve %d: TLI %.1f m/s, i %.3f deg, plane moved %.2e rad\n",
                        i + 1, norm(solution.burn.delta_v_gcrf_mps), solution.arrival.inclination_rad / constants::deg_to_rad,
                        moved);
        }
        if (moved < plane_fixed_point_rad) break;
        normal = next;
    }
    plan.llo_plane_normal_moon_inertial = normal;
    plan.llo_arrival_epoch = arrival;
    plan.llo_touchdown = touchdown;
    return solution;
}

std::optional<time::Epoch> LunarOutboundPlanner::nrho_insertion_epoch(const LunarOutboundParameters& p) const {
    // Moon-relative osculating true anomaly of the reference, unwrapped to (-pi, pi] (0 at the perilune).
    auto anomaly = [&](time::Epoch t) {
        const double nu =
            orbit::elements_from_state(moon_relative(ephemeris_, p.nrho->state_gcrf(t), t), gm_moon_).true_anomaly_rad;
        return nu > constants::pi ? nu - 2.0 * constants::pi : nu;
    };
    const double target = p.nrho_insertion_true_anomaly_rad;
    time::Epoch a = p.arrival_epoch;
    if (!p.nrho->covers(a) || anomaly(a) >= target) return std::nullopt;
    // Coarse steps from the perilune until the anomaly passes the target (it rises monotonically up to the apolune).
    for (time::Epoch b = a + anomaly_search_step_s; p.nrho->covers(b); b = b + anomaly_search_step_s) {
        const double nu = anomaly(b);
        if (nu < 0.0) return std::nullopt;  // past the apolune without reaching the target
        if (nu >= target) {
            while (b - a > anomaly_search_tolerance_s) {
                const time::Epoch m = a + 0.5 * (b - a);
                (anomaly(m) >= target ? b : a) = m;
            }
            return b;
        }
        a = b;
    }
    return std::nullopt;
}

bool LunarOutboundPlanner::plan_nrho(const StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                     const LunarOutboundParameters& p, OutboundPlan& plan) const {
    const auto t_insert = nrho_insertion_epoch(p);
    if (!t_insert) {
        plan.reason = report::make(report::Code::lunar_nrho_not_covering_arrival);
        return false;
    }
    const auto reference = p.nrho->state_gcrf(*t_insert);
    const auto reference_rel = moon_relative(ephemeris_, reference, *t_insert);

    // One candidate per swing-by inclination (the TLI's B-plane target): its braking burn is found by fixed-time
    // position targeting onto the insertion point, from a Moon two-body Lambert first guess. Returns the total
    // delta-v (TLI + swing-by + insertion), infinite when a step fails; the cheapest candidate is kept in `plan`.
    double best_cost = std::numeric_limits<double>::infinity();
    auto candidate = [&](double inclination) {
        const auto tli = transfer_.solve(state, epoch, burn_epoch, {p.flyby_altitude_m, inclination, p.arrival_epoch});
        if (!tli.converged) return std::numeric_limits<double>::infinity();
        const auto& flyby = tli.arrival;
        const double coast_s = *t_insert - flyby.periselene_epoch;
        if (coast_s < min_flyby_to_insertion_s) return std::numeric_limits<double>::infinity();
        const auto moon = ephemeris_.state_gcrf(ephemeris::Body::Moon, flyby.periselene_epoch);
        const StateVector at_flyby{flyby.relative_state.position_m + moon.position_m,
                                   flyby.relative_state.velocity_mps + moon.velocity_mps};
        // The braking burn keeps the flyby's sense of motion: near the insertion the NRHO moves almost radially, so
        // the arc meets it either way, but reversing the swing-by would cost kilometres per second.
        const auto lambert = orbit::solve_lambert(
            flyby.relative_state.position_m, reference_rel.position_m, coast_s, gm_moon_,
            cross(flyby.relative_state.position_m, flyby.relative_state.velocity_mps));
        const Vec3 guess = lambert ? lambert->departure_velocity_mps - flyby.relative_state.velocity_mps : Vec3{};
        const auto swing = position_.solve(at_flyby, flyby.periselene_epoch, flyby.periselene_epoch,
                                           {reference.position_m, *t_insert}, guess, nrho_flyby_tolerance_m);
        const Vec3 insertion = reference.velocity_mps - swing.arrival_gcrf.velocity_mps;
        const double cost = norm(tli.burn.delta_v_gcrf_mps) + norm(swing.burn.delta_v_gcrf_mps) + norm(insertion);
        if (debug_enabled()) {
            std::fprintf(stderr,
                         "outbound NRHO swing-by i %6.2f deg: TLI %.1f m/s, flyby %.2f d before insertion, "
                         "swing-by %s%.1f m/s (miss %.0f m), insertion %.1f m/s\n",
                         inclination / constants::deg_to_rad, norm(tli.burn.delta_v_gcrf_mps), coast_s / 86400.0,
                         swing.converged ? "" : "(not converged) ", norm(swing.burn.delta_v_gcrf_mps), swing.miss_m,
                         norm(insertion));
        }
        if (!swing.converged) return std::numeric_limits<double>::infinity();
        if (cost < best_cost) {
            best_cost = cost;
            plan.tli = tli.burn;
            plan.powered_flyby = swing.burn;
            plan.flyby_target = {at_flyby.position_m, flyby.periselene_epoch};
            plan.insertion = {*t_insert, insertion};
        }
        return cost;
    };

    // Coarse grid over 0..180 deg, then golden-section refinement around the best grid point (the cost changes by
    // hundreds of m/s per grid step).
    double best_grid = 0.0;
    for (double inclination = 0.0; inclination <= constants::pi + 1e-9; inclination += flyby_inclination_step_rad) {
        const double before = best_cost;
        candidate(inclination);
        if (best_cost < before) best_grid = inclination;
    }
    if (!std::isfinite(best_cost)) {
        plan.reason = report::make(report::Code::lunar_no_nrho_transfer);
        return false;
    }
    const double golden = 0.5 * (std::sqrt(5.0) - 1.0);
    double lo = std::max(0.0, best_grid - flyby_inclination_step_rad);
    double hi = std::min(constants::pi, best_grid + flyby_inclination_step_rad);
    double x1 = hi - golden * (hi - lo), x2 = lo + golden * (hi - lo);
    double f1 = candidate(x1), f2 = candidate(x2);
    while (hi - lo > flyby_inclination_tolerance_rad) {
        if (f1 < f2) {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - golden * (hi - lo);
            f1 = candidate(x1);
        } else {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + golden * (hi - lo);
            f2 = candidate(x2);
        }
    }
    return true;
}

OutboundPlan LunarOutboundPlanner::plan(const StateVector& state, time::Epoch epoch, const LunarOutboundParameters& p,
                                        double mass_kg, double isp_s, double propellant_kg) const {
    OutboundPlan plan;
    const time::Epoch burn_epoch =
        transfer_.best_departure(state, epoch, epoch + p.earliest_departure_s, p.arrival_epoch);

    if (p.orbit == LunarOrbitKind::Llo) {
        TransferSolution solution;
        if (!p.llo.plane_normal_moon_inertial && p.llo_site) {
            solution = solve_through_site(state, epoch, burn_epoch, p, plan);
        } else {
            solution = transfer_.solve(state, epoch, burn_epoch, llo_target(p));
            if (solution.converged && p.llo.plane_normal_moon_inertial) {  // a given plane: its arrival is held after
                plan.llo_plane_normal_moon_inertial = p.llo.plane_normal_moon_inertial;
                plan.llo_arrival_epoch = solution.arrival.periselene_epoch;
            }
        }
        if (!solution.converged) {
            plan.reason = report::make(report::Code::lunar_no_moon_transfer);
            return plan;
        }
        plan.tli = solution.burn;
        plan.insertion = LunarTransferTargeter::orbit_insertion(solution.arrival, gm_moon_);
    } else {
        if (!p.nrho || !p.nrho->covers(p.arrival_epoch)) {
            plan.reason = report::make(report::Code::lunar_nrho_not_covering_arrival);
            return plan;
        }
        if (!plan_nrho(state, epoch, burn_epoch, p, plan)) return plan;
    }

    // Propellant on the vacuum Raptors for all planned burns (rocket equation), plus the reserve.
    plan.total_delta_v_mps = norm(plan.tli.delta_v_gcrf_mps) + norm(plan.insertion.delta_v_gcrf_mps) +
                             (plan.powered_flyby ? norm(plan.powered_flyby->delta_v_gcrf_mps) : 0.0);
    plan.propellant_kg =
        mass_kg * (1.0 - std::exp(-plan.total_delta_v_mps / (isp_s * constants::standard_gravity_mps2)));
    if (plan.propellant_kg * (1.0 + p.reserve_fraction) > propellant_kg) {
        plan.reason = report::make(report::Code::lunar_outbound_insufficient_propellant,
                                   {report::mass("needed", plan.propellant_kg * (1.0 + p.reserve_fraction)),
                                    report::mass("onboard", propellant_kg)});
        return plan;
    }
    plan.feasible = true;
    return plan;
}

std::optional<gnc::BurnNode> LunarOutboundPlanner::finite_tli(const StateVector& state, time::Epoch epoch,
                                                              const LunarOutboundParameters& p, const OutboundPlan& plan,
                                                              const FiniteBurnModel& engines) const {
    // Continuation on the burn duration: the impulse is too far off as a first guess for the full ~7 min burn (its arc
    // no longer comes near the Moon), so thrust and mass flow are scaled down from 8x (same exhaust velocity) and each
    // solution seeds the next.
    gnc::BurnNode burn = plan.tli;
    for (const double scale : {8.0, 4.0, 2.0, 1.0}) {
        const FiniteBurnModel scaled{engines.mass_kg, engines.thrust_n * scale, engines.mass_flow_kgps * scale};
        bool converged = false;
        if (p.orbit == LunarOrbitKind::Llo) {
            auto targeter = transfer_;
            targeter.set_finite_burn(scaled);
            const auto solution =
                targeter.solve(state, epoch, plan.tli.epoch, llo_target(p, &plan), burn.delta_v_gcrf_mps);
            converged = solution.converged;
            if (converged) burn = solution.burn;
        } else {
            auto targeter = position_;
            targeter.set_finite_burn(scaled);
            const auto solution = targeter.solve(state, epoch, plan.tli.epoch, plan.flyby_target, burn.delta_v_gcrf_mps,
                                                 nrho_mcc_tolerance_m);
            converged = solution.converged;
            if (converged) burn = solution.burn;
        }
        if (debug_enabled()) {
            std::printf("[outbound] finite TLI x%.0f: converged %d, %.1f m/s\n", scale, converged,
                        norm(burn.delta_v_gcrf_mps));
        }
        if (!converged) return std::nullopt;
    }
    if (debug_enabled()) {
        std::printf("[outbound] finite TLI: %.1f m/s (impulsive %.1f m/s, %.1f m/s apart)\n", norm(burn.delta_v_gcrf_mps),
                    norm(plan.tli.delta_v_gcrf_mps), norm(burn.delta_v_gcrf_mps - plan.tli.delta_v_gcrf_mps));
    }
    return burn;
}

std::optional<gnc::BurnNode> LunarOutboundPlanner::mid_course(const StateVector& state, time::Epoch epoch,
                                                              time::Epoch burn_epoch, const LunarOutboundParameters& p,
                                                              const OutboundPlan& plan) const {
    gnc::BurnNode burn;
    if (p.orbit == LunarOrbitKind::Llo) {
        auto solution = transfer_.solve(state, epoch, burn_epoch, llo_target(p, &plan), Vec3{});
        // A finite TLI can leave an arc that no longer comes near the Moon: no periselene to start Newton from zero;
        // then the Lambert guess aimed beside the Moon at the planned arrival.
        if (!solution.converged) solution = transfer_.solve(state, epoch, burn_epoch, llo_target(p, &plan));
        if (debug_enabled()) {
            const auto free = transfer_.evaluate(state, epoch, burn_epoch, Vec3{});
            std::printf("[outbound] MCC: free arc reached %d periselene %.0f km i %.2f deg; solve %d after %d it, "
                        "%.2f m/s -> %.0f km i %.2f deg\n",
                        free.reached, free.periselene_altitude_m / 1000.0, free.inclination_rad / constants::deg_to_rad,
                        solution.converged, solution.iterations, norm(solution.burn.delta_v_gcrf_mps),
                        solution.arrival.periselene_altitude_m / 1000.0,
                        solution.arrival.inclination_rad / constants::deg_to_rad);
        }
        if (!solution.converged) return std::nullopt;
        burn = solution.burn;
    } else {
        // NRHO: MCCs aim at the swing-by point; after it there is nothing to aim at (the insertion is re-planned).
        if (burn_epoch >= plan.flyby_target.epoch) return std::nullopt;
        const auto solution =
            position_.solve(state, epoch, burn_epoch, plan.flyby_target, Vec3{}, nrho_mcc_tolerance_m);
        if (!solution.converged) return std::nullopt;
        burn = solution.burn;
    }
    if (norm(burn.delta_v_gcrf_mps) < p.mcc_threshold_mps) return std::nullopt;
    return burn;
}

std::optional<gnc::BurnNode> LunarOutboundPlanner::powered_flyby(const StateVector& state, time::Epoch epoch,
                                                                 const LunarOutboundParameters& p,
                                                                 const OutboundPlan& plan) const {
    if (p.orbit != LunarOrbitKind::Nrho || !plan.powered_flyby || !p.nrho || !p.nrho->covers(plan.insertion.epoch)) {
        return std::nullopt;
    }
    const auto solution =
        position_.solve(state, epoch, plan.powered_flyby->epoch,
                        {p.nrho->state_gcrf(plan.insertion.epoch).position_m, plan.insertion.epoch},
                        plan.powered_flyby->delta_v_gcrf_mps, nrho_mcc_tolerance_m);
    if (!solution.converged) return std::nullopt;
    return solution.burn;
}

std::optional<gnc::BurnNode> LunarOutboundPlanner::insertion(const StateVector& state, time::Epoch epoch,
                                                             const LunarOutboundParameters& p,
                                                             const OutboundPlan& plan) const {
    if (p.orbit == LunarOrbitKind::Llo) {
        const auto arrival = transfer_.evaluate(state, epoch, epoch, Vec3{});
        if (!arrival.reached) return std::nullopt;
        return LunarTransferTargeter::orbit_insertion(arrival, gm_moon_);
    }
    const time::Epoch t = plan.insertion.epoch;
    if (!p.nrho || !p.nrho->covers(t)) return std::nullopt;
    const auto arrived = position_.coast(state, epoch, t);
    // Velocity match only: the position error at the perilune is left to the station keeping.
    return gnc::BurnNode{t, p.nrho->state_gcrf(t).velocity_mps - arrived.velocity_mps};
}

std::optional<gnc::BurnNode> LunarOutboundPlanner::llo_trim(const StateVector& state, time::Epoch epoch,
                                                            const LunarOutboundParameters& p,
                                                            double min_lead_s) const {
    // Two-body about the Moon: the closed-loop burn absorbs the Earth's and the Sun's pull.
    const auto rel = moon_relative(ephemeris_, state, epoch);
    const auto el = orbit::elements_from_state(rel, gm_moon_);
    if (el.eccentricity >= 1.0) return std::nullopt;
    const double target = constants::moon_mean_radius_m + p.llo.altitude_m;
    const double rp = el.semi_major_axis_m * (1.0 - el.eccentricity);
    const double ra = el.semi_major_axis_m * (1.0 + el.eccentricity);
    if (std::abs(rp - target) < p.llo_trim_tolerance_m && std::abs(ra - target) < p.llo_trim_tolerance_m) {
        return std::nullopt;
    }
    const double e = el.eccentricity;
    const double ecc_anomaly =
        2.0 * std::atan(std::sqrt((1.0 - e) / (1.0 + e)) * std::tan(0.5 * el.true_anomaly_rad));
    const double mean_anomaly = ecc_anomaly - e * std::sin(ecc_anomaly);
    const double mean_motion = std::sqrt(gm_moon_ / std::pow(el.semi_major_axis_m, 3));
    const double period = 2.0 * constants::pi / mean_motion;
    if (min_lead_s >= period) throw std::invalid_argument("llo_trim: lead longer than one revolution");
    const double to_periapsis = time_to_mean_anomaly(mean_anomaly, 0.0, mean_motion);
    const double to_apoapsis = time_to_mean_anomaly(mean_anomaly, constants::pi, mean_motion);
    double dt = far_future_s;  // the earliest apsis at least min_lead_s ahead
    for (double c : {to_periapsis, to_apoapsis, to_periapsis + period, to_apoapsis + period}) {
        if (c >= min_lead_s) dt = std::min(dt, c);
    }
    const auto apsis = orbit::propagate_two_body(rel, gm_moon_, dt);
    const double r = norm(apsis.position_m);
    // Speed at radius r for the orbit with apsides r and target.
    const double speed = std::sqrt(gm_moon_ * 2.0 * target / (r * (r + target)));
    const Vec3 direction = math::normalized(apsis.velocity_mps);
    return gnc::BurnNode{epoch + dt, direction * (speed - norm(apsis.velocity_mps))};
}

}  // namespace starsim::mission
