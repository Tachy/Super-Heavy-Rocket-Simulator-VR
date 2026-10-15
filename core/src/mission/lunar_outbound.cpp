#include "starsim/mission/lunar_outbound.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

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

LunarArrivalTarget LunarOutboundPlanner::llo_target(const LunarOutboundParameters& p) const {
    return {p.llo.altitude_m, p.llo.inclination_rad, p.arrival_epoch};
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
        plan.reason = "NRHO-Referenz deckt die Ankunft nicht ab";
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
        plan.reason = "Kein Transfer zur NRHO gefunden";
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
        const auto solution = transfer_.solve(state, epoch, burn_epoch, llo_target(p));
        if (!solution.converged) {
            plan.reason = "Kein Transfer zum Mond gefunden";
            return plan;
        }
        plan.tli = solution.burn;
        plan.insertion = LunarTransferTargeter::orbit_insertion(solution.arrival, gm_moon_);
    } else {
        if (!p.nrho || !p.nrho->covers(p.arrival_epoch)) {
            plan.reason = "NRHO-Referenz deckt die Ankunft nicht ab";
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
        char buffer[160];
        std::snprintf(buffer, sizeof buffer, "Zu wenig Treibstoff für den Hinflug: %.0f t nötig, %.0f t an Bord",
                      plan.propellant_kg * (1.0 + p.reserve_fraction) / 1000.0, propellant_kg / 1000.0);
        plan.reason = buffer;
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
            const auto solution = targeter.solve(state, epoch, plan.tli.epoch, llo_target(p), burn.delta_v_gcrf_mps);
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
        auto solution = transfer_.solve(state, epoch, burn_epoch, llo_target(p), Vec3{});
        // A finite TLI can leave an arc that no longer comes near the Moon: no periselene to start Newton from zero;
        // then the Lambert guess aimed beside the Moon at the planned arrival.
        if (!solution.converged) solution = transfer_.solve(state, epoch, burn_epoch, llo_target(p));
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
    return gnc::BurnNode{t, p.nrho->state_gcrf(t).velocity_mps - arrived.velocity_mps};
}

std::optional<gnc::BurnNode> LunarOutboundPlanner::llo_trim(const StateVector& state, time::Epoch epoch,
                                                            const LunarOutboundParameters& p) const {
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
    const double dt = std::min(time_to_mean_anomaly(mean_anomaly, 0.0, mean_motion),
                               time_to_mean_anomaly(mean_anomaly, constants::pi, mean_motion));
    const auto apsis = orbit::propagate_two_body(rel, gm_moon_, dt);
    const double r = norm(apsis.position_m);
    // Speed at radius r for the orbit with apsides r and target.
    const double speed = std::sqrt(gm_moon_ * 2.0 * target / (r * (r + target)));
    const Vec3 direction = math::normalized(apsis.velocity_mps);
    return gnc::BurnNode{epoch + dt, direction * (speed - norm(apsis.velocity_mps))};
}

}  // namespace starsim::mission
