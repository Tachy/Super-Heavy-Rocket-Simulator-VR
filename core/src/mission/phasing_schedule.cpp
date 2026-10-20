#include "starsim/mission/phasing_schedule.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/environment/gravity.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"

namespace starsim::mission {

namespace {
constexpr int max_extra_laps = 3;
constexpr int bisection_steps = 60;

// Mean argument-of-latitude rate of an orbit with semi-major axis a (Kepler + secular J2).
double latitude_rate(double a, double eccentricity, double inclination, double gm) {
    orbit::StationMeanElements mean;
    mean.semi_major_axis_m = a;
    mean.eccentricity = eccentricity;
    mean.inclination_rad = inclination;
    return orbit::j2_secular_rates(mean, gm, constants::earth_j2, environment::earth_j2_reference_radius_m)
        .latitude_radps;
}

double half_transfer_s(double r1, double r2, double gm) {
    const double a = 0.5 * (r1 + r2);
    return constants::pi * std::sqrt(a * a * a / gm);
}
}  // namespace

double coelliptic_behind_m(const PhasingParameters& p, double target_radius_m, double gm) {
    const double n = std::sqrt(gm / (target_radius_m * target_radius_m * target_radius_m));
    const double range = p.approach_initiation_range_m, dh = p.coelliptic_height_m;
    return std::sqrt(std::max(0.0, range * range - dh * dh)) + 1.5 * n * dh * p.coelliptic_coast_s;
}

PhasingSchedule phasing_schedule(const math::StateVector& chaser, const math::StateVector& target, time::Epoch epoch,
                                 const PhasingParameters& p, double gm) {
    PhasingSchedule s;
    const auto ec = orbit::elements_from_state(chaser, gm);
    const auto et = orbit::elements_from_state(target, gm);
    if (!(ec.semi_major_axis_m > 0.0) || ec.eccentricity >= 1.0 || !(et.semi_major_axis_m > 0.0) ||
        et.eccentricity >= 1.0) {
        s.reason = "Kein Rendezvous: keine geschlossene Bahn";
        return s;
    }
    const double r_d = ec.semi_major_axis_m, r_i = et.semi_major_axis_m;
    const double r_c = r_i - p.coelliptic_height_m;
    // below: the apoapsis under the coelliptic height (an eccentric start must not cross it); above (refuelling f1):
    // the periapsis keeps coelliptic_height_m [E] above the target's orbit while the target catches up
    const bool above = r_d * (1.0 - ec.eccentricity) > r_i + p.coelliptic_height_m;
    if (r_d * (1.0 + ec.eccentricity) >= r_c && !above) {
        s.reason = "Kein Rendezvous: Bahn kreuzt die Zielbahn";
        return s;
    }
    s.above = above;
    const double n_d = latitude_rate(r_d, ec.eccentricity, ec.inclination_rad, gm);
    const double n_i = latitude_rate(r_i, et.eccentricity, et.inclination_rad, gm);
    const double t_h = half_transfer_s(r_d, r_c, gm);
    // Phase (target ahead of the chaser) the boost needs: the chaser half a transfer behind the coelliptic aim point.
    const double behind_c = coelliptic_behind_m(p, r_i, gm);
    const double required_at_boost = constants::pi + behind_c / r_i - n_i * t_h;
    double lead = -in_plane_angle_rad(target, chaser.position_m);  // > 0: target ahead
    if (lead < 0.0) lead += constants::two_pi;
    const double tail_s = t_h + p.coelliptic_coast_s + p.approach_transfer_s;
    // rad/s of catch-up on the start orbit (the coelliptic closing is in behind_c): below, the chaser gains on the
    // target (the lead shrinks); above, the target gains (the lead grows)
    const double closing = std::abs(n_d - n_i);
    const double above_catch_up = std::fmod(required_at_boost - lead + 2.0 * constants::two_pi, constants::two_pi);

    for (int m = 0; m <= max_extra_laps; ++m) {
        const double catch_up =
            above ? above_catch_up + constants::two_pi * m : lead + constants::two_pi * m - required_at_boost;
        if (catch_up < 0.0) continue;
        // 1. Wait on the start orbit: the earliest window of this lap.
        const double wait_s = catch_up / closing;
        if (wait_s >= p.first_burn_lead_s && wait_s + tail_s <= p.mission_limit_s &&
            wait_s + tail_s >= p.earliest_arrival_s) {
            s.feasible = true;
            s.extra_laps = m;
            s.boost_epoch = epoch + wait_s;
            break;
        }
        if (above) continue;  // the phase burn only slows a chaser below the target
        // 2. Phase burn at first_burn_lead_s raising the apoapsis to r_a, boost after k revolutions at its point:
        //    catch-up = closing * lead_s + k (2 pi - n_i P(a)) with P = 2 pi / n(a), a = (r_d + r_a) / 2.
        bool found = false;
        for (int k = 1; !found; ++k) {
            const double base = closing * p.first_burn_lead_s;
            const auto caught = [&](double r_a) {
                const double a = 0.5 * (r_d + r_a);
                const double period = constants::two_pi / latitude_rate(a, (r_a - r_d) / (r_a + r_d), ec.inclination_rad, gm);
                return base + k * (constants::two_pi - n_i * period);
            };
            const double period_low = constants::two_pi / n_d;
            if (p.first_burn_lead_s + k * period_low + tail_s > p.mission_limit_s) break;
            if (caught(r_d) < catch_up || caught(r_c) > catch_up) continue;  // this k cannot close this lap
            double lo = r_d, hi = r_c;  // caught() decreases with r_a
            for (int b = 0; b < bisection_steps; ++b) {
                const double mid = 0.5 * (lo + hi);
                (caught(mid) > catch_up ? lo : hi) = mid;
            }
            const double r_a = 0.5 * (lo + hi);
            const double a = 0.5 * (r_d + r_a);
            const double period = constants::two_pi / latitude_rate(a, (r_a - r_d) / (r_a + r_d), ec.inclination_rad, gm);
            const double boost_s = p.first_burn_lead_s + k * period;
            if (boost_s + tail_s > p.mission_limit_s || boost_s + tail_s < p.earliest_arrival_s) continue;
            s.feasible = true;
            s.extra_laps = m;
            s.phasing_revolutions = k;
            s.phase_orbit_apoapsis_radius_m = r_a;
            s.phase_epoch = epoch + p.first_burn_lead_s;
            s.boost_epoch = epoch + boost_s;
            found = true;
        }
        if (found) break;
    }
    if (!s.feasible) {
        s.reason = "Kein Rendezvous: Phasenwinkel nicht innerhalb von " +
                   std::to_string(static_cast<int>(std::lround(p.mission_limit_s / 3600.0))) + " h erreichbar";
        return s;
    }
    s.coelliptic_epoch = s.boost_epoch + t_h;
    s.approach_initiation_epoch = s.coelliptic_epoch + p.coelliptic_coast_s;
    s.waypoint0_epoch = s.approach_initiation_epoch + p.approach_transfer_s;
    return s;
}

}  // namespace starsim::mission
