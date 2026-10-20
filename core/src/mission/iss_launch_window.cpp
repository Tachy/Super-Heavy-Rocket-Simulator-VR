#include "starsim/mission/iss_launch_window.hpp"

#include <cmath>
#include <cstdio>

#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"

namespace starsim::mission {

namespace {

using math::Vec3;

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

LaunchWindow plan_launch_window(const Vec3& pad_itrf_m, const frames::BodyOrientation& earth,
                                const StationStateAt& station, time::Epoch earliest, const LaunchWindowParameters& p,
                                double gm_m3ps2) {
    LaunchWindow w;
    const double a_chaser = 0.5 * (p.separation_periapsis_radius_m + p.separation_apoapsis_radius_m);
    const double a_station = orbit::elements_from_state(station(earliest), gm_m3ps2).semi_major_axis_m;
    std::string last_reason = "keine Ebenenquerung";
    time::Epoch t = earliest;
    while (t - earliest < p.search_s) {
        // the first bias guess: the plane match half way through the mission limit
        double bias = plane_bias_rad(a_chaser, a_station, p.inclination_rad, 0.5 * p.phasing.mission_limit_s);
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
        time::Epoch root = t1;
        for (int iteration = 0; iteration < p.bias_iterations; ++iteration) {
            // the bias moves the zero by well under a sample: a widened bracket around the first one
            time::Epoch lo = t1 - p.sample_s, hi = t1 + 2.0 * p.sample_s;
            if (!(f(lo) <= 0.0 && f(hi) > 0.0)) {
                lo = t1;
                hi = t1 + p.sample_s;
            }
            root = bisect(f, lo, hi);
            const Vec3 pad = earth.fixed_to_gcrf(root).position(pad_itrf_m);
            const Vec3 n = gnc::ascent_plane_normal(pad, earth, root, p.inclination_rad, p.northerly);
            w.liftoff = root;
            w.separation = root + p.insertion_s;
            w.separation_state_gcrf = separation_state_estimate(pad, n, p, gm_m3ps2);
            w.plane_bias_rad = bias;
            w.schedule =
                phasing_schedule(w.separation_state_gcrf, station(w.separation), w.separation, p.phasing, gm_m3ps2);
            if (!w.schedule.feasible) break;
            // re-aim the bias at the boost (i6 correction 2)
            bias = plane_bias_rad(a_chaser, a_station, p.inclination_rad, w.schedule.boost_epoch - w.separation);
        }
        if (w.schedule.feasible) {
            w.feasible = true;
            return w;
        }
        last_reason = w.schedule.reason;
        ++w.rejected;
        t = root + 3600.0;  // past this crossing: the next one is ~a day later
    }
    char text[64];
    std::snprintf(text, sizeof text, "Kein Startfenster in %g Tagen: ", p.search_s / 86400.0);
    w.feasible = false;
    w.reason = text + last_reason;
    return w;
}

}  // namespace starsim::mission
