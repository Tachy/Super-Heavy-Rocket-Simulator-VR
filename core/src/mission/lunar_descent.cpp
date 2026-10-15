#include "starsim/mission/lunar_descent.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "starsim/integrators/integrate.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"

namespace starsim::mission {

using math::StateVector;
using math::Vec3;

namespace {
constexpr double sample_step_s = 60.0;      // pass search samples
constexpr double search_chunk_s = 86400.0;  // the pass search propagates one day at a time and stops at the first pass
constexpr double phase_tolerance_s = 1e-3;  // epoch of an in-plane angle
constexpr int phase_iterations = 60;
constexpr int trim_iterations = 6;
constexpr int doi_iterations = 8;

bool debug_enabled() { return std::getenv("STARSIM_DEBUG_LUNAR_DESCENT") != nullptr; }

Vec3 fixed_unit(double latitude_rad, double longitude_rad) {
    return {std::cos(latitude_rad) * std::cos(longitude_rad), std::cos(latitude_rad) * std::sin(longitude_rad),
            std::sin(latitude_rad)};
}

double wrap_pi(double a) { return std::remainder(a, constants::two_pi); }

// An Earth-centred GCRF state the predictions coast from.
struct Arc {
    StateVector state;
    time::Epoch epoch;
};

// Moon-relative geometry of a point on the surface against the vehicle's orbit plane: the in-plane angle the vehicle
// still has to travel to the point's projection on the plane, phi = atan2((r x s_p).n, r.s_p) (positive before the
// pass), and the sine of the point's angle out of the plane (s.n, positive on the side of the orbit normal).
struct Geometry {
    double phi_rad;
    double sin_out_of_plane;
};

Geometry geometry(const StateVector& rel, const Vec3& s) {
    const Vec3 n = math::normalized(cross(rel.position_m, rel.velocity_mps));
    const Vec3 s_p = s - n * dot(s, n);
    return {std::atan2(dot(cross(rel.position_m, s_p), n), dot(rel.position_m, s_p)), dot(s, n)};
}

class Predictor {
public:
    Predictor(const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
              const frames::BodyOrientation& moon)
        : gravity_(gravity), ephemeris_(ephemeris), moon_(moon) {}

    StateVector at(const Arc& arc, time::Epoch t) const {
        if (t - arc.epoch == 0.0) return arc.state;
        return orbit::propagate_coast(gravity_, arc.state, arc.epoch, t).state;
    }
    StateVector relative(const StateVector& state, time::Epoch t) const { return moon_relative(ephemeris_, state, t); }
    Vec3 direction(const Vec3& fixed_unit_vector, time::Epoch t) const {
        return moon_.fixed_to_gcrf(t).rotation * fixed_unit_vector;
    }
    Geometry geometry_at(const StateVector& state, time::Epoch t, const Vec3& fixed) const {
        return geometry(relative(state, t), direction(fixed, t));
    }

    // Samples every sample_step_s over (arc.epoch, arc.epoch + duration] from one DOP853 run (dense output).
    std::vector<Arc> samples(const Arc& arc, double duration_s) const {
        std::vector<Arc> out;
        double next = sample_step_s;
        const auto observer = [&](const integrators::Dop853<6>& solver) {
            while (next <= solver.t() + 1e-9) {
                out.push_back({math::unpack(solver.dense_output(next)), arc.epoch + next});
                next += sample_step_s;
            }
        };
        integrators::integrate<6>(orbit::ballistic_dynamics(gravity_, arc.epoch), 0.0, math::pack(arc.state),
                                  duration_s, orbit::default_coast_options(), {}, observer);
        return out;
    }

    // Epoch in [lo, hi] where the angle still to travel to the point (phi mod 2 pi) equals `to_go_rad`; it falls with
    // time, so g = wrap(phi - to_go) changes sign from + to - (Illinois false position). Empty when not bracketed.
    std::optional<time::Epoch> phase_epoch(const Arc& arc, const Vec3& fixed, double to_go_rad, time::Epoch lo,
                                           time::Epoch hi) const {
        const Arc base{at(arc, lo), lo};
        const auto g = [&](double dt) {
            const auto t = lo + dt;
            return wrap_pi(geometry_at(at(base, t), t, fixed).phi_rad - to_go_rad);
        };
        double a = 0.0, b = hi - lo;
        double ga = g(a), gb = g(b);
        if (!(ga > 0.0 && gb <= 0.0)) return std::nullopt;
        int side = 0;
        for (int i = 0; i < phase_iterations && b - a > phase_tolerance_s; ++i) {
            const double c = b - gb * (b - a) / (gb - ga);
            const double gc = g(c);
            if (gc > 0.0) {
                a = c;
                ga = gc;
                if (side == -1) gb *= 0.5;
                side = -1;
            } else {
                b = c;
                gb = gc;
                if (side == 1) ga *= 0.5;
                side = 1;
            }
        }
        return lo + 0.5 * (a + b);
    }

    // Closest approach to the Moon after `arc` (radial velocity rising through zero) within `span_s`.
    std::optional<double> periselene_radius(const Arc& arc, double span_s) const {
        const integrators::Event<6> periselene{
            "periselene",
            [&](double t, const integrators::State<6>& y) {
                const auto rel = relative(math::unpack(y), arc.epoch + t);
                return dot(rel.position_m, rel.velocity_mps);
            },
            integrators::Crossing::Rising};
        const auto result = orbit::propagate_coast(gravity_, arc.state, arc.epoch, arc.epoch + span_s, {&periselene, 1});
        if (result.event_index != 0) return std::nullopt;
        return norm(relative(result.state, result.epoch).position_m);
    }

private:
    const environment::GravityModel& gravity_;
    const ephemeris::Ephemeris& ephemeris_;
    const frames::BodyOrientation& moon_;
};

Arc with_impulse(const Arc& arc, const Vec3& delta_v) {
    return {{arc.state.position_m, arc.state.velocity_mps + delta_v}, arc.epoch};
}

// Secant iterations on f(x) = 0 from (x0, f0) and x1; stops below `tolerance`. Returns the last x and f(x).
template <class F>
std::pair<double, double> secant(F&& f, double x0, double f0, double x1, double tolerance, int iterations,
                                 const char* label) {
    double f1 = f(x1);
    for (int i = 0; i < iterations && std::abs(f1) >= tolerance && f1 != f0; ++i) {
        const double x2 = x1 - f1 * (x1 - x0) / (f1 - f0);
        x0 = x1;
        f0 = f1;
        x1 = x2;
        f1 = f(x1);
        if (debug_enabled()) std::printf("[lunar_descent] %s %d: %.4f m/s -> %.1f m\n", label, i, x1, f1);
    }
    return {x1, f1};
}

std::string refusal(const char* format, double a, double b = 0.0) {
    char text[200];
    std::snprintf(text, sizeof(text), format, a, b);
    return text;
}
}  // namespace

double braking_range_m(double pdi_speed_mps, double mass_kg, double thrust_n, const BrakingRangeParameters& p) {
    const double acceleration = p.thrust_fraction * thrust_n / mass_kg;
    const double range = acceleration > 0.0 ? p.range_factor * pdi_speed_mps * pdi_speed_mps / (2.0 * acceleration)
                                            : p.max_range_m;
    return std::clamp(range, p.min_range_m, p.max_range_m);
}

LunarDescentPlanner::LunarDescentPlanner(const environment::GravityModel& gravity,
                                         const ephemeris::Ephemeris& ephemeris,
                                         const frames::BodyOrientation& moon_orientation)
    : gravity_(gravity),
      ephemeris_(ephemeris),
      moon_orientation_(moon_orientation),
      gm_moon_(ephemeris.gm_m3ps2(ephemeris::Body::Moon)) {}

LunarDescentPlan LunarDescentPlanner::plan(const StateVector& state_gcrf, time::Epoch epoch,
                                           const environment::LunarTerrain& terrain, double site_latitude_rad,
                                           double site_longitude_rad, double footprint_radius_m,
                                           const LunarDescentParameters& p) const {
    LunarDescentPlan plan;
    const Predictor predict(gravity_, ephemeris_, moon_orientation_);
    if (!terrain.in_tile(site_latitude_rad, site_longitude_rad)) {
        plan.reason = "Landestelle außerhalb der Geländekachel (tools/data_pipeline/fetch.py)";
        return plan;
    }
    const auto rel0 = predict.relative(state_gcrf, epoch);
    const double r0 = norm(rel0.position_m);
    const double sma = 1.0 / (2.0 / r0 - dot(rel0.velocity_mps, rel0.velocity_mps) / gm_moon_);
    const double period = constants::two_pi * std::sqrt(sma * sma * sma / gm_moon_);
    const double radius_m = terrain.sphere_radius_m();
    const Vec3 site_fixed = fixed_unit(site_latitude_rad, site_longitude_rad);
    const time::Epoch earliest_pass = epoch + p.min_lead_s + 1.25 * period;

    // Pass search: samples of the coasting LLO, a pass where phi crosses zero from + to - (|phi| small: not the
    // antipode's wrap from +pi to -pi); its epoch refined on the samples' bracket.
    Arc cursor{state_gcrf, epoch};
    Geometry previous = predict.geometry_at(state_gcrf, epoch, site_fixed);
    while (cursor.epoch - epoch < p.search_s) {
        const auto samples = predict.samples(cursor, std::min(search_chunk_s, p.search_s - (cursor.epoch - epoch)));
        if (samples.empty()) break;
        for (const auto& sample : samples) {
            const Arc before = cursor;
            cursor = sample;
            const Geometry now = predict.geometry_at(sample.state, sample.epoch, site_fixed);
            const bool pass = previous.phi_rad > 0.0 && now.phi_rad <= 0.0 && previous.phi_rad < 0.5 * constants::pi;
            previous = now;
            if (!pass) continue;
            const auto pass_epoch = predict.phase_epoch(before, site_fixed, 0.0, before.epoch, sample.epoch);
            if (!pass_epoch) continue;
            const auto at_pass = predict.at(before, *pass_epoch);
            const double crossrange =
                radius_m * std::asin(predict.geometry_at(at_pass, *pass_epoch, site_fixed).sin_out_of_plane);
            if (debug_enabled()) {
                std::printf("[lunar_descent] pass at +%.0f s, crossrange %.1f m\n", *pass_epoch - epoch, crossrange);
            }
            if (std::abs(crossrange) > p.max_crossrange_m || *pass_epoch < earliest_pass) continue;

            // Approach azimuth: the ground velocity (Moon-relative minus the Moon's rotation) on the site's horizontal.
            const auto rel = predict.relative(at_pass, *pass_epoch);
            const auto transform = moon_orientation_.fixed_to_gcrf(*pass_epoch);
            const Vec3 s = transform.rotation * site_fixed;
            const Vec3 pole = transform.rotation * Vec3{0.0, 0.0, 1.0};
            const Vec3 ground_v = rel.velocity_mps - cross(frames::rotation_vector(transform), rel.position_m);
            const Vec3 east = math::normalized(cross(pole, s));
            const Vec3 north = cross(s, east);
            plan.approach_azimuth_rad = std::atan2(dot(ground_v, east), dot(ground_v, north));

            const auto touchdown = select_touchdown_point(terrain, site_latitude_rad, site_longitude_rad,
                                                          plan.approach_azimuth_rad, footprint_radius_m, p);
            if (!touchdown) {
                plan.reason = refusal("Kein Landepunkt mit höchstens %.0f Grad Hangneigung im Umkreis von %.0f m",
                                      p.slope_limit_rad / constants::deg_to_rad, p.touchdown_search_radius_m);
                return plan;
            }
            plan.touchdown = *touchdown;
            const Vec3 td_fixed = fixed_unit(touchdown->latitude_rad, touchdown->longitude_rad);
            const double td_radius_m = radius_m + touchdown->height_m;

            // The same pass of the touchdown point (shifted by <= 1 km).
            const double window_s = 0.05 * period;
            const auto td_pass = predict.phase_epoch(before, td_fixed, 0.0, *pass_epoch - window_s, *pass_epoch + window_s);
            if (!td_pass) continue;
            plan.site_pass_epoch = *td_pass;
            plan.crossrange_before_trim_m =
                td_radius_m *
                std::asin(predict.geometry_at(predict.at(before, *td_pass), *td_pass, td_fixed).sin_out_of_plane);

            // Plane trim along the orbit normal a quarter revolution before the point on the previous revolution:
            // the out-of-plane offset of the impulse, dv r / v, peaks at the pass (sin of 450 deg). Secant on the
            // predicted crossrange from the first guess dv = v asin(s.n) (moves the track towards the point).
            const double trim_window_s = 0.15 * period;
            const auto trim_epoch = predict.phase_epoch(before, td_fixed, 0.5 * constants::pi,
                                                        *td_pass - 1.25 * period - trim_window_s,
                                                        *td_pass - 1.25 * period + trim_window_s);
            if (!trim_epoch || *trim_epoch < epoch + p.min_lead_s) continue;
            const Arc trim_arc{predict.at(before, *trim_epoch), *trim_epoch};
            const auto trim_rel = predict.relative(trim_arc.state, *trim_epoch);
            const Vec3 normal = math::normalized(cross(trim_rel.position_m, trim_rel.velocity_mps));
            const auto crossrange_after = [&](double dv) {
                const Arc trimmed = with_impulse(trim_arc, normal * dv);
                const auto t = predict.phase_epoch(trimmed, td_fixed, 0.0, *td_pass - window_s, *td_pass + window_s);
                if (!t) return plan.crossrange_before_trim_m;  // no pass found: treated as no change
                return td_radius_m *
                       std::asin(predict.geometry_at(predict.at(trimmed, *t), *t, td_fixed).sin_out_of_plane);
            };
            const double guess = norm(trim_rel.velocity_mps) * plan.crossrange_before_trim_m / td_radius_m;
            Arc descent_orbit = trim_arc;
            plan.crossrange_m = plan.crossrange_before_trim_m;
            if (std::abs(guess) >= p.trim_threshold_mps) {
                const auto [dv, miss] = secant(crossrange_after, 0.0, plan.crossrange_before_trim_m, guess,
                                               p.trim_tolerance_m, trim_iterations, "trim");
                if (std::abs(dv) >= p.trim_threshold_mps) {
                    plan.plane_trim = gnc::BurnNode{*trim_epoch, normal * dv};
                    descent_orbit = with_impulse(trim_arc, normal * dv);
                    plan.crossrange_m = miss;
                }
            }

            // DOI half a revolution before the PDI point (braking range before the touchdown point): retrograde,
            // Hohmann first guess down to 15 km above the touchdown point, secant on the predicted periselene radius.
            const double braking_angle = p.braking_range_m / td_radius_m;
            const double to_go_doi = constants::pi + braking_angle;
            const auto doi_guess = *td_pass - to_go_doi / constants::two_pi * period;
            const auto doi_epoch = predict.phase_epoch(descent_orbit, td_fixed, to_go_doi, doi_guess - trim_window_s,
                                                       doi_guess + trim_window_s);
            if (!doi_epoch || *doi_epoch < epoch + p.min_lead_s) continue;
            const Arc doi_arc{predict.at(descent_orbit, *doi_epoch), *doi_epoch};
            const auto doi_rel = predict.relative(doi_arc.state, *doi_epoch);
            const double r1 = norm(doi_rel.position_m), v1 = norm(doi_rel.velocity_mps);
            const double rp = td_radius_m + p.pdi_altitude_above_site_m;
            const Vec3 retrograde = -math::normalized(doi_rel.velocity_mps);
            const auto periselene_miss = [&](double dv) {
                const auto radius = predict.periselene_radius(with_impulse(doi_arc, retrograde * dv), 0.75 * period);
                return radius ? *radius - rp : r1 - rp;  // no periselene: the orbit stays near circular
            };
            const double hohmann = v1 - std::sqrt(gm_moon_ * 2.0 * rp / (r1 * (r1 + rp)));
            const auto [doi_dv, doi_miss] = secant(periselene_miss, 0.0, periselene_miss(0.0), hohmann,
                                                   p.doi_tolerance_m, doi_iterations, "DOI");
            if (std::abs(doi_miss) > p.doi_tolerance_m) {
                plan.reason = "Kein Abstiegsmanöver (DOI) gefunden";
                return plan;
            }
            plan.doi = {*doi_epoch, retrograde * doi_dv};
            plan.periselene_altitude_m = rp + doi_miss - radius_m;

            // PDI on the post-DOI prediction where the angle to the touchdown point equals the braking range's.
            const Arc descent = with_impulse(doi_arc, retrograde * doi_dv);
            const auto pdi_guess = *td_pass - braking_angle / constants::two_pi * period;
            const auto pdi = predict.phase_epoch(descent, td_fixed, braking_angle, pdi_guess - trim_window_s,
                                                 pdi_guess + trim_window_s);
            if (!pdi) {
                plan.reason = "Kein Beginn des Abstiegs (PDI) gefunden";
                return plan;
            }
            plan.pdi_epoch = *pdi;
            const auto pdi_rel = predict.relative(predict.at(descent, *pdi), *pdi);
            const double cos_arc = dot(math::normalized(pdi_rel.position_m), predict.direction(td_fixed, *pdi));
            plan.pdi_range_m = td_radius_m * std::acos(std::clamp(cos_arc, -1.0, 1.0));
            if (debug_enabled()) {
                std::printf("[lunar_descent] trim %.3f m/s, DOI %.3f m/s, periselene %.1f m, PDI +%.0f s, %.1f km\n",
                            plan.plane_trim ? norm(plan.plane_trim->delta_v_gcrf_mps) : 0.0, doi_dv,
                            plan.periselene_altitude_m, *pdi - epoch, plan.pdi_range_m / 1000.0);
            }
            plan.feasible = true;
            return plan;
        }
    }
    plan.reason = refusal("Kein Überflug der Landestelle innerhalb von %.1f Tagen (Bahnebene zu weit entfernt)",
                          p.search_s / 86400.0);
    return plan;
}

}  // namespace starsim::mission
