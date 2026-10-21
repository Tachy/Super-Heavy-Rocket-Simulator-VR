#include "starsim/mission/iss_launch_window.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"

namespace starsim::mission {

namespace {

using math::Vec3;

constexpr int max_raise_burns = 2;  // an apoapsis and a periapsis raise (refuelling f1)
// [E] review I1: the J2 swing of the osculating apsides at ~200 km (flown: 190 x 190 km read over 197.1 km)
constexpr double phasing_clearance_m = 10.0e3;

double raan_rate_radps(double a_m, double inclination_rad) {
    orbit::StationMeanElements mean;
    mean.semi_major_axis_m = a_m;
    mean.inclination_rad = inclination_rad;
    return orbit::j2_secular_rates(mean, constants::earth_gm_m3ps2, constants::earth_j2,
                                   environment::earth_j2_reference_radius_m)
        .raan_radps;
}

// Node of the ascent plane minus node of the biased station plane, about the Earth's pole, wrapped to (-pi, pi]; it
// grows with the Earth's rotation (~7.3e-5 rad/s), its upward zero is the window.
double node_difference_rad(const Vec3& pad_itrf_m, const frames::BodyOrientation& earth, const StationStateAt& station,
                           time::Epoch t, const LaunchWindowParameters& p, double bias_rad) {
    const auto transform = earth.fixed_to_gcrf(t);
    const Vec3 pole = transform.rotation * Vec3{0.0, 0.0, 1.0};
    const Vec3 pad = transform.position(pad_itrf_m);
    const Vec3 n_ascent = gnc::ascent_plane_normal(pad, earth, t, p.inclination_rad, p.northerly);
    const auto s = station(t);
    const Vec3 n_target =
        math::Quat::from_axis_angle(pole, bias_rad).rotate(math::normalized(cross(s.position_m, s.velocity_mps)));
    const Vec3 node_a = cross(pole, n_ascent), node_t = cross(pole, n_target);
    return std::atan2(dot(cross(node_t, node_a), pole), dot(node_t, node_a));
}

// An upward zero of `f` in [lo, hi] (f(lo) <= 0 < f(hi)) by bisection.
template <class F>
time::Epoch bisect(F f, time::Epoch lo, time::Epoch hi) {
    for (int k = 0; k < 40; ++k) {  // 600 s / 2^40: far below a microsecond
        const time::Epoch mid = lo + 0.5 * (hi - lo);
        if (f(mid) > 0.0) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    return lo + 0.5 * (hi - lo);
}

}  // namespace

double plane_bias_rad(double chaser_semi_major_axis_m, double station_semi_major_axis_m, double inclination_rad,
                      double match_s) {
    return (raan_rate_radps(station_semi_major_axis_m, inclination_rad) -
            raan_rate_radps(chaser_semi_major_axis_m, inclination_rad)) *
           match_s;
}

math::StateVector separation_state_estimate(const Vec3& pad_gcrf_m, const Vec3& plane_normal_gcrf,
                                            const LaunchWindowParameters& p, double gm_m3ps2) {
    const Vec3 n = math::normalized(plane_normal_gcrf);
    const Vec3 in_plane = math::normalized(pad_gcrf_m - dot(pad_gcrf_m, n) * n);
    const Vec3 u = math::Quat::from_axis_angle(n, p.insertion_arc_rad).rotate(in_plane);
    const double rp = p.separation_periapsis_radius_m, ra = p.separation_apoapsis_radius_m;
    const double speed = std::sqrt(gm_m3ps2 * (2.0 / rp - 2.0 / (rp + ra)));  // vis-viva at periapsis
    return {rp * u, speed * cross(n, u)};
}

RaiseEstimate raise_estimate(const math::StateVector& separation, const PhasingOrbit& orbit, double gm_m3ps2) {
    RaiseEstimate r;
    const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(0.0);
    time::Epoch t = t0;
    math::StateVector s = separation;
    for (int burn = 0; burn <= max_raise_burns; ++burn) {
        const auto node = gnc::next_raise_node(s, t, gm_m3ps2, orbit.periapsis_radius_m, orbit.apoapsis_radius_m,
                                               raise_tolerance_m);
        if (!node) {
            r.feasible = true;
            r.state = s;
            r.elapsed_s = t - t0;
            return r;
        }
        if (burn == max_raise_burns) break;  // more burns than an apoapsis and a periapsis raise
        s = orbit::propagate_two_body(s, gm_m3ps2, node->epoch - t);
        s.velocity_mps = s.velocity_mps + node->delta_v_gcrf_mps;
        r.delta_v_mps += math::norm(node->delta_v_gcrf_mps);
        ++r.burns;
        t = node->epoch;
    }
    return r;
}

PhasingCost phasing_cost(const RaiseEstimate& raise, const PhasingSchedule& schedule,
                         double separation_to_waypoint0_s, const PhasingParameters& p, double target_radius_m,
                         const PhasingCostModel& model, double gm_m3ps2) {
    const double r1 = orbit::elements_from_state(raise.state, gm_m3ps2).semi_major_axis_m;
    const double r_c = target_radius_m - p.coelliptic_height_m;
    const auto departure = [gm_m3ps2](double from, double to) {
        return std::sqrt(gm_m3ps2 / from) * (std::sqrt(2.0 * to / (from + to)) - 1.0);
    };
    const auto arrival = [gm_m3ps2](double from, double to) {
        return std::sqrt(gm_m3ps2 / to) * (1.0 - std::sqrt(2.0 * from / (from + to)));
    };
    double dv = raise.delta_v_mps + std::abs(departure(r1, r_c)) + std::abs(arrival(r1, r_c));
    int burns = raise.burns + 2;
    if (schedule.phasing_revolutions > 0) {  // the phase burn raises the apoapsis; the boost takes it back
        dv += 2.0 * std::abs(departure(r1, schedule.phase_orbit_apoapsis_radius_m));
        burns += 2;
    }
    PhasingCost c;
    if (model.exhaust_velocity_mps > 0.0) c.propellant_kg = model.mass_kg * (1.0 - std::exp(-dv / model.exhaust_velocity_mps));
    c.propellant_kg += model.gas_per_burn_kg * burns;
    c.boil_off_kg = model.boil_off_kgps * separation_to_waypoint0_s;
    return c;
}

LaunchWindow plan_launch_window(const Vec3& pad_itrf_m, const frames::BodyOrientation& earth,
                                const StationStateAt& station, time::Epoch earliest, const LaunchWindowParameters& p,
                                double gm_m3ps2) {
    LaunchWindow w;
    const double a_separation = 0.5 * (p.separation_periapsis_radius_m + p.separation_apoapsis_radius_m);
    const double a_station = orbit::elements_from_state(station(earliest), gm_m3ps2).semi_major_axis_m;
    std::string last_reason = "keine Ebenenquerung";
    // no candidates: the separation orbit is the phasing orbit (the ISS missions, unchanged)
    std::vector<std::optional<PhasingOrbit>> candidates;
    if (p.phasing_orbits.empty()) candidates.emplace_back();
    for (const auto& o : p.phasing_orbits) candidates.emplace_back(o);

    // The window at the crossing near t1 for one candidate: the root with the candidate's plane bias, re-aimed at its
    // boost; std::nullopt when the crossing lies before the earliest liftoff with the aimed bias.
    const auto evaluate = [&](time::Epoch t1, const std::optional<PhasingOrbit>& orbit) -> std::optional<LaunchWindow> {
        const double a_chaser = orbit ? 0.5 * (orbit->periapsis_radius_m + orbit->apoapsis_radius_m) : a_separation;
        LaunchWindow c;
        c.phasing_orbit = orbit;
        double bias = plane_bias_rad(a_chaser, a_station, p.inclination_rad, 0.5 * p.phasing.mission_limit_s);
        for (int iteration = 0; iteration < p.bias_iterations; ++iteration) {
            const auto f = [&](time::Epoch e) { return node_difference_rad(pad_itrf_m, earth, station, e, p, bias); };
            // the bias moves the zero by well under a sample: a widened bracket around the first one, never before
            // the earliest liftoff (review i6: the re-aimed bias moved a crossing just after it to before it)
            const time::Epoch lo = std::max(t1 - p.sample_s, earliest), hi = t1 + 2.0 * p.sample_s;
            if (!(f(lo) <= 0.0 && f(hi) > 0.0)) return std::nullopt;
            const time::Epoch root = bisect(f, lo, hi);
            const Vec3 pad = earth.fixed_to_gcrf(root).position(pad_itrf_m);
            const Vec3 n = gnc::ascent_plane_normal(pad, earth, root, p.inclination_rad, p.northerly);
            c.liftoff = root;
            c.separation = root + p.insertion_s;
            c.separation_state_gcrf = separation_state_estimate(pad, n, p, gm_m3ps2);
            c.plane_bias_rad = bias;
            c.raised = c.separation;
            c.raised_state_gcrf = c.separation_state_gcrf;
            if (!orbit) {
                c.schedule = phasing_schedule(c.separation_state_gcrf, station(c.separation), c.separation, p.phasing,
                                              gm_m3ps2);
            } else {
                const auto raise = raise_estimate(c.separation_state_gcrf, *orbit, gm_m3ps2);
                auto q = p.phasing;  // the limits count from the separation
                q.mission_limit_s -= raise.elapsed_s;
                q.earliest_arrival_s = std::max(0.0, q.earliest_arrival_s - raise.elapsed_s);
                c.schedule = PhasingSchedule{};
                // review I1: the flown plan reads the osculating apsides, which J2 swings by up to ~10 km - a phasing
                // orbit whose mean radius lies within that of the coelliptic height (below) or of the target's orbit
                // plus the coelliptic height (above) passes here but is refused in flight
                const double r_c = a_station - p.phasing.coelliptic_height_m;
                const bool clear = a_chaser <= r_c - phasing_clearance_m ||
                                   a_chaser >= a_station + p.phasing.coelliptic_height_m + phasing_clearance_m;
                if (!clear) {
                    c.schedule.reason = "Kein Rendezvous: Phasing-Bahn zu nah an der Zielbahn";
                } else if (!raise.feasible || q.mission_limit_s <= 0.0) {
                    c.schedule.reason = "Kein Rendezvous: Phasing-Bahn nicht erreichbar";
                } else {
                    c.raised = c.separation + raise.elapsed_s;
                    c.raised_state_gcrf = raise.state;
                    c.schedule = phasing_schedule(raise.state, station(c.raised), c.raised, q, gm_m3ps2);
                    if (c.schedule.feasible) {
                        c.cost = phasing_cost(raise, c.schedule, c.schedule.waypoint0_epoch - c.separation, p.phasing,
                                              a_station, p.cost, gm_m3ps2);
                    }
                }
            }
            if (!c.schedule.feasible) break;
            // re-aim the bias at the boost (i6 correction 2)
            bias = plane_bias_rad(a_chaser, a_station, p.inclination_rad, c.schedule.boost_epoch - c.separation);
        }
        return c;
    };

    time::Epoch t = earliest;
    while (t - earliest < p.search_s) {
        // the first bias guess: the plane match half way through the mission limit
        const double bias = plane_bias_rad(a_separation, a_station, p.inclination_rad, 0.5 * p.phasing.mission_limit_s);
        const auto f = [&](time::Epoch e) { return node_difference_rad(pad_itrf_m, earth, station, e, p, bias); };
        // the next upward zero after t: sample, then bisect
        double f0 = f(t);
        time::Epoch t1 = t;
        bool found = false;
        while (t1 - earliest < p.search_s) {
            const double f2 = f(t1 + p.sample_s);
            if (f0 <= 0.0 && f2 > 0.0 && f2 - f0 < 1.0) {  // not the +-pi wrap
                found = true;
                break;
            }
            t1 = t1 + p.sample_s;
            f0 = f2;
        }
        if (!found) break;
        // every candidate at this crossing; the cheapest feasible one (ties: the first in the list)
        std::optional<LaunchWindow> best, any;
        std::vector<std::string> reasons;  // every candidate's (f1 minor M4: only the first one's was reported)
        for (const auto& orbit : candidates) {
            const auto c = evaluate(t1, orbit);
            if (!c) continue;
            if (!any) any = c;
            if (c->schedule.feasible && (!best || c->cost.total_kg() < best->cost.total_kg())) best = c;
            if (!c->schedule.feasible &&
                std::find(reasons.begin(), reasons.end(), c->schedule.reason) == reasons.end()) {
                reasons.push_back(c->schedule.reason);
            }
        }
        if (!any) {  // this crossing lies before the earliest liftoff with the aimed bias: the next one
            t = t1 + p.sample_s;
            continue;
        }
        if (best) {
            best->feasible = true;
            best->rejected = w.rejected;
            return *best;
        }
        last_reason.clear();
        for (const auto& reason : reasons) last_reason += (last_reason.empty() ? "" : "; ") + reason;
        ++w.rejected;
        t = any->liftoff + 3600.0;  // past this crossing: the next one is ~a day later
    }
    char text[64];
    std::snprintf(text, sizeof text, "Kein Startfenster in %g Tagen: ", p.search_s / 86400.0);
    w.feasible = false;
    w.reason = text + last_reason;
    return w;
}

}  // namespace starsim::mission
