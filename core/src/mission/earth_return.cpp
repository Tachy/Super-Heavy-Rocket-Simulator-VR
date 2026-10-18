#include "starsim/mission/earth_return.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <span>
#include <utility>

#include "starsim/frames/geodetic.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/lambert.hpp"

namespace starsim::mission {

using math::StateVector;
using math::Vec3;

namespace {

constexpr double guess_perigee_height_m = 50e3;     // [E] vacuum perigee of the patched-conic first guess
constexpr double max_return_s = 6.0 * 86400.0;      // inbound coasts end here
constexpr double max_interface_coast_s = 7200.0;    // braking height (or perigee) -> interface
constexpr double far_from_earth_m = 1.0e8;          // a closest approach farther out is the lunar swing, not perigee
constexpr double past_false_perigee_s = 60.0;
constexpr double at_braking_height_m = 1.0;         // a state this close to the braking height is "there"
constexpr int braking_passes = 3;                   // fixed: the arrival stays smooth in the burn (finite differences)
constexpr double finite_difference_mps = 1e-3;
// with the entry predictor in the residual: its fixed-step integration and gate interpolation are noisy at ~0.1 km, a
// 1 mm/s difference would be dominated by that noise
constexpr double predictor_difference_mps = 0.05;
constexpr double max_newton_step_mps = 100.0;
constexpr int max_halvings = 8;
constexpr double predictor_step_s = 1.0;
constexpr double predictor_max_time_s = 6000.0;
constexpr double exited_downrange_m = 2.0e7;        // a skip beyond the carry limit reads as a huge overshoot
// convergence
constexpr double perigee_tolerance_m = 200.0;
constexpr double crossrange_tolerance_m = 500.0;
constexpr double downrange_tolerance_m = 500.0;
constexpr double arrival_time_tolerance_s = 0.1;
constexpr double braking_speed_tolerance_mps = 0.5;
// gate search (direct entry): the interface epoch is scanned both ways from the plane solution and a downrange sign
// change refined until the gate stage can take over
constexpr double gate_scan_step_s = 2.0 * 3600.0;     // [E] the site turns ~3,000 km per step
constexpr double gate_scan_span_s = 12.0 * 3600.0;    // +-12 h: every site pass
constexpr double gate_scan_min_step_s = 900.0;        // [E] continuation: the shortest step before a way is closed
constexpr double gate_bracket_jump_m = 10000e3;       // larger: the target range wrapped past the antipode
constexpr int gate_refine_iterations = 12;
// [E] A gate the secant cannot close (the predicted range jumps at a reach boundary next to it: Task 8's MCC stalled at
// -21 km, the secant's denominator went to zero) is accepted this close: the profiled entry guidance moves its range by
// ~300 km per degree of initial bank.
constexpr double gate_accept_m = 50e3;
constexpr double gate_perigee_tolerance_m = 20.0;
constexpr double gate_crossrange_tolerance_m = 50.0;
constexpr double max_gate_step_s = 2.0 * 3600.0;  // secant steps outside a bracket
constexpr double gate_min_probe_s = 600.0;
// [E] one bank sign's track aimed at the site: the lateral offset refined until the predicted crossrange is below this
// (the guidance nulls the rest with small reversals below their speed limit)
constexpr double lateral_tolerance_m = 2000.0;
constexpr int lateral_iterations = 6;
constexpr double braking_angle_tolerance_rad = 0.005 * constants::deg_to_rad;
// patched-conic departure search
constexpr double llo_sample_s = 60.0;
constexpr double llo_window_periods = 1.1;
// NRHO return flyby point on the flyby sphere: coarse grid step (then 1/5 and 1/25 of it around the best point)
constexpr double flyby_point_step_rad = 10.0 * constants::deg_to_rad;
constexpr double flyby_position_tolerance_m = 100.0;  // [E] the departure burn's aim at the periselene point
// [E] NRHO departure searched 0.5-2.5 d before the perilune, the flyby within +-12 h of it (patched conic, cheap)
constexpr double nrd_lead_min_s = 0.5 * 86400.0;
constexpr double nrd_lead_max_s = 2.5 * 86400.0;
constexpr double nrd_lead_step_s = 0.25 * 86400.0;
constexpr double flyby_offset_max_s = 12.0 * 3600.0;
constexpr double flyby_offset_step_s = 3.0 * 3600.0;  // a search window reaches past one crossing
constexpr size_t max_nrho_options = 8;  // [E] departure options tried in the full model per perilune
constexpr double nrho_estimate_bound_fraction = 0.9;  // [E] the patched-conic estimate times this bounds the option
constexpr double departure_tolerance_s = 0.5;
constexpr double transfer_angle_min_rad = 90.0 * constants::deg_to_rad;
constexpr double transfer_angle_max_rad = 179.5 * constants::deg_to_rad;
constexpr double transfer_angle_step_rad = 2.5 * constants::deg_to_rad;

bool debug_enabled() { return std::getenv("STARSIM_DEBUG_RETURN") != nullptr; }

// Two-body periapsis radius p / (1 + e) (ellipse or hyperbola).
double periapsis_radius(const StateVector& s, double gm) {
    const double r = norm(s.position_m), v = norm(s.velocity_mps);
    const double h = norm(cross(s.position_m, s.velocity_mps));
    const double energy = 0.5 * v * v - gm / r;
    const double e = std::sqrt(std::max(0.0, 1.0 + 2.0 * energy * h * h / (gm * gm)));
    return h * h / (gm * (1.0 + e));
}

// Periapsis radius of the conic through radius r with speed v and flight-path angle gamma.
double periapsis_for(double r, double v, double gamma, double gm) {
    const double h = r * v * std::cos(gamma);
    const double energy = 0.5 * v * v - gm / r;
    const double e = std::sqrt(std::max(0.0, 1.0 + 2.0 * energy * h * h / (gm * gm)));
    return h * h / (gm * (1.0 + e));
}

// Position and velocity relative to the rotating Earth (ITRF).
std::pair<Vec3, Vec3> to_itrf(const frames::BodyOrientation& earth, const StateVector& s, time::Epoch t) {
    const auto x = earth.fixed_to_gcrf(t);
    const math::Mat3 rt = transpose(x.rotation);
    const Vec3 r = rt * s.position_m;
    return {r, rt * (s.velocity_mps - x.rotation_rate_per_s * r)};
}

// Velocity at `r` (Moon-relative) on the hyperbola with excess velocity `v_inf` whose outgoing asymptote is v_inf's
// direction: in the plane of r and the asymptote, eccentricity e from r = p / (1 + e cos(nu)) with nu = nu_inf - theta
// (theta the angle from r to the asymptote). Empty if no such hyperbola passes through r.
std::optional<Vec3> departure_velocity(const Vec3& r, const Vec3& v_inf, double gm) {
    const double r_m = norm(r), v2 = dot(v_inf, v_inf);
    if (!(v2 > 0.0)) return std::nullopt;
    const Vec3 r_hat = r / r_m, a_hat = v_inf / std::sqrt(v2);
    const double theta = std::acos(std::clamp(dot(r_hat, a_hat), -1.0, 1.0));
    const double semi_major = gm / v2;  // |a|
    auto radius_error = [&](double e) {
        const double nu = std::acos(-1.0 / e) - theta;
        const double denominator = 1.0 + e * std::cos(nu);
        return denominator > 0.0 ? semi_major * (e * e - 1.0) / denominator - r_m : -r_m;
    };
    // the radius grows with e from ~0 (at e -> 1 the asymptote swings back over r); scan for the crossing
    double lo = 1.0 + 1e-9;
    double f_lo = radius_error(lo);
    for (double hi = 1.001; hi < 1e4; hi *= 1.2) {
        const double f_hi = radius_error(hi);
        if ((f_lo < 0.0) != (f_hi < 0.0)) {
            for (int i = 0; i < 100; ++i) {
                const double mid = 0.5 * (lo + hi);
                ((radius_error(mid) < 0.0) == (f_lo < 0.0) ? lo : hi) = mid;
            }
            const double e = 0.5 * (lo + hi);
            const double nu = std::acos(-1.0 / e) - theta;
            const double p = semi_major * (e * e - 1.0);
            const double k = std::sqrt(gm / p);
            const Vec3 normal = math::normalized(cross(r_hat, a_hat));
            const Vec3 t_hat = cross(normal, r_hat);  // towards the asymptote
            return r_hat * (k * e * std::sin(nu)) + t_hat * (k * (1.0 + e * std::cos(nu)));
        }
        lo = hi;
        f_lo = f_hi;
    }
    return std::nullopt;
}

Vec3 rotated_about(const Vec3& u, const Vec3& unit_axis, double angle) {
    return u * std::cos(angle) + cross(unit_axis, u) * std::sin(angle) +
           unit_axis * (dot(unit_axis, u) * (1.0 - std::cos(angle)));
}

// Hyperbolic excess velocity (Moon-relative, GCRF) of the patched-conic return leaving the Moon at `departure`.
Vec3 excess_velocity(const ephemeris::Ephemeris& ephemeris, time::Epoch departure, double flight_time_s,
                     const Vec3& site_itrf_m, const frames::BodyOrientation& earth) {
    const double gm = ephemeris.gm_m3ps2(ephemeris::Body::Earth);
    const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, departure);
    const Vec3 site = earth.fixed_to_gcrf(departure + flight_time_s).rotation * site_itrf_m;
    const Vec3 rm = moon.position_m;
    Vec3 n = math::normalized(cross(rm, site));
    if (n.z < 0.0) n = -n;  // eastward arrival: the lower relative entry speed
    const double rp = constants::wgs84_semi_major_axis_m + guess_perigee_height_m;
    // The transfer angle whose arrival at the perigee radius is horizontal (that point is the perigee).
    struct Arc { bool ok; double radial; Vec3 departure; };
    auto arc = [&](double angle) -> Arc {
        const Vec3 d = rotated_about(math::normalized(rm), n, angle);
        const auto lambert = orbit::solve_lambert(rm, d * rp, flight_time_s, gm, n);
        if (!lambert) return {false, 0.0, {}};
        return {true, dot(d, lambert->arrival_velocity_mps), lambert->departure_velocity_mps};
    };
    Arc best{false, std::numeric_limits<double>::infinity(), {}};
    Arc previous = arc(transfer_angle_min_rad);
    double previous_angle = transfer_angle_min_rad;
    for (double angle = transfer_angle_min_rad + transfer_angle_step_rad; angle <= transfer_angle_max_rad + 1e-12;
         angle += transfer_angle_step_rad) {
        const Arc current = arc(angle);
        if (current.ok && (!best.ok || std::abs(current.radial) < std::abs(best.radial))) best = current;
        if (previous.ok && current.ok && (previous.radial < 0.0) != (current.radial < 0.0)) {
            double lo = previous_angle, hi = angle;
            Arc a_lo = previous;
            for (int i = 0; i < 40; ++i) {
                const double mid = 0.5 * (lo + hi);
                const Arc m = arc(mid);
                if (!m.ok) break;
                if ((m.radial < 0.0) == (a_lo.radial < 0.0)) {
                    lo = mid;
                    a_lo = m;
                } else {
                    hi = mid;
                }
                best = m;
            }
            break;
        }
        previous = current;
        previous_angle = angle;
    }
    return best.ok ? best.departure - moon.velocity_mps : Vec3{};
}

}  // namespace

Vec3 return_asymptote(const ephemeris::Ephemeris& ephemeris, time::Epoch departure, double flight_time_s,
                      const Vec3& site_itrf_m, const frames::BodyOrientation& earth) {
    const Vec3 v = excess_velocity(ephemeris, departure, flight_time_s, site_itrf_m, earth);
    return norm(v) > 0.0 ? math::normalized(v) : Vec3{};
}

EarthReturnPlanner::EarthReturnPlanner(const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                                       const frames::BodyOrientation& earth_orientation)
    : gravity_(gravity),
      ephemeris_(ephemeris),
      earth_(earth_orientation),
      position_(gravity),
      gm_earth_(ephemeris.gm_m3ps2(ephemeris::Body::Earth)),
      gm_moon_(ephemeris.gm_m3ps2(ephemeris::Body::Moon)) {}

StateVector EarthReturnPlanner::burned(const StateVector& at_burn, time::Epoch burn_epoch, const Vec3& delta_v) const {
    return finite_ ? apply_finite_burn(gravity_, at_burn, burn_epoch, delta_v, *finite_)
                   : StateVector{at_burn.position_m, at_burn.velocity_mps + delta_v};
}

void EarthReturnPlanner::arrive(const StateVector& state, time::Epoch epoch, const EarthReturnTarget& target,
                                EarthArrival& a) const {
    const double R = constants::wgs84_semi_major_axis_m;
    // osculating perigee of the braked conic (the constraint: continuous even when the interface is missed)
    a.perigee_height_m = periapsis_radius(state, gm_earth_) - R;
    // to the interface, or to the perigee when it stays above it: the arrival plane and the speeds are then read at the
    // perigee (a later point would let the Earth turn under the plane and make the crossrange jump at the boundary)
    const std::array<integrators::Event<6>, 2> events{
        integrators::Event<6>{"entry interface",
                              [&](double t_s, const integrators::State<6>& y) {
                                  const Vec3 r =
                                      transpose(earth_.fixed_to_gcrf(epoch + t_s).rotation) * Vec3{y[0], y[1], y[2]};
                                  return frames::fixed_to_geodetic(r).height_m - target.interface_height_m;
                              },
                              integrators::Crossing::Falling},
        integrators::Event<6>{"perigee",
                              [](double, const integrators::State<6>& y) {
                                  return y[0] * y[3] + y[1] * y[4] + y[2] * y[5];
                              },
                              integrators::Crossing::Rising}};
    orbit::CoastResult in{state, epoch, 1, 0};
    if (dot(state.position_m, state.velocity_mps) < 0.0) {
        in = orbit::propagate_coast(gravity_, state, epoch, epoch + max_interface_coast_s, events);
    }
    const auto [r_itrf, v_itrf] = to_itrf(earth_, in.state, in.epoch);
    a.reached = in.event_index == 0;
    a.interface_epoch = in.epoch;
    a.interface_gcrf = in.state;
    a.speed_mps = norm(v_itrf);
    a.flight_path_rad = std::asin(std::clamp(dot(math::normalized(r_itrf), v_itrf) / a.speed_mps, -1.0, 1.0));
    // crossrange: the site's distance from the arrival plane (plane through the Earth's centre, the interface point and
    // its relative velocity) - Earth rotation during the entry is the predictor's business
    const Vec3 n = math::normalized(cross(r_itrf, v_itrf));
    // (measured from the aimed track's lateral offset)
    a.crossrange_m =
        -std::asin(std::clamp(dot(math::normalized(target.site_itrf_m), n), -1.0, 1.0)) * R - target.lateral_offset_m;
    a.lateral_offset_m = target.lateral_offset_m;
    if (a.reached && target.entry_model) {
        const double bank = target.bank_sign * target.nominal_bank_rad;
        const auto p = gnc::predict_entry(*target.entry_model, gnc::EntryPredictorState{r_itrf, v_itrf, bank}, bank,
                                          target.gate, target.gate_height_m, 0.0, predictor_step_s,
                                          predictor_max_time_s);
        a.downrange_error_m = p.exited ? exited_downrange_m : p.downrange_error_m;
        a.predicted_crossrange_m = p.crossrange_m - target.bank_sign * target.lateral_margin_m;  // from the aim point
    }
}

EarthArrival EarthReturnPlanner::evaluate(const StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                          const Vec3& delta_v_mps, const EarthReturnTarget& target) const {
    EarthArrival a;
    const StateVector at_burn =
        burn_epoch - epoch == 0.0 ? state : orbit::propagate_coast(gravity_, state, epoch, burn_epoch).state;
    StateVector s = burned(at_burn, burn_epoch, delta_v_mps);
    time::Epoch t = burn_epoch;
    const double r_brake = constants::wgs84_semi_major_axis_m + target.braking_height_m;

    bool at_brake = norm(s.position_m) <= r_brake + at_braking_height_m;
    if (!at_brake) {
        const std::array<integrators::Event<6>, 2> events{
            integrators::Event<6>{"braking height",
                                  [&](double, const integrators::State<6>& y) {
                                      return std::hypot(y[0], y[1], y[2]) - r_brake;
                                  },
                                  integrators::Crossing::Falling},
            // a perigee above the braking height ends the inbound coast too (the constraints stay defined)
            integrators::Event<6>{"perigee",
                                  [](double, const integrators::State<6>& y) {
                                      return y[0] * y[3] + y[1] * y[4] + y[2] * y[5];
                                  },
                                  integrators::Crossing::Rising}};
        const time::Epoch horizon = burn_epoch + max_return_s;
        for (;;) {
            const auto coast = orbit::propagate_coast(gravity_, s, t, horizon, events);
            if (coast.event_index < 0) {
                // never came near the Earth: the osculating Earth conic at the horizon steers back (its perigee and
                // plane; the site where it stands then)
                const StateVector& end = coast.state;
                a.perigee_height_m = periapsis_radius(end, gm_earth_) - constants::wgs84_semi_major_axis_m;
                a.interface_epoch = coast.epoch;
                a.interface_gcrf = end;
                const Vec3 n = math::normalized(cross(end.position_m, end.velocity_mps));
                const Vec3 site = earth_.fixed_to_gcrf(coast.epoch).rotation * target.site_itrf_m;
                a.crossrange_m = -std::asin(std::clamp(dot(math::normalized(site), n), -1.0, 1.0)) *
                                     constants::wgs84_semi_major_axis_m -
                                 target.lateral_offset_m;
                a.lateral_offset_m = target.lateral_offset_m;
                return a;
            }
            s = coast.state;
            t = coast.epoch;
            if (coast.event_index == 1 && norm(s.position_m) > far_from_earth_m) {
                // the swing about the Moon, not the Earth's perigee: step past it and go on
                s = orbit::propagate_coast(gravity_, s, t, t + past_false_perigee_s).state;
                t = t + past_false_perigee_s;
                continue;
            }
            at_brake = coast.event_index == 0;
            break;
        }
    }
    if (at_brake && target.speed_mps > 0.0) {
        // Retrograde to the inertial interface speed that gives the target RELATIVE speed: the difference (Earth
        // rotation along the track) is measured on the braked arc and fed back, a fixed number of passes.
        const double gm = gm_earth_;
        const double r_now = norm(s.position_m), v_now = norm(s.velocity_mps);
        double r_interface = constants::wgs84_semi_major_axis_m + target.interface_height_m;
        double offset = 0.0;  // inertial - relative speed at the interface
        const double v_interface_now = std::sqrt(v_now * v_now + 2.0 * gm * (1.0 / r_interface - 1.0 / r_now));
        if (v_interface_now > target.speed_mps) {
            StateVector braked = s;
            for (int pass = 0; pass < braking_passes; ++pass) {
                const double v_target = target.speed_mps + offset;
                const double v_new =
                    std::sqrt(std::max(0.0, v_target * v_target - 2.0 * gm * (1.0 / r_interface - 1.0 / r_now)));
                braked.velocity_mps = s.velocity_mps * (v_new / v_now);
                if (pass + 1 == braking_passes) break;
                EarthArrival probe;
                EarthReturnTarget plain = target;
                plain.entry_model.reset();
                arrive(braked, t, plain, probe);
                if (!probe.reached) break;
                offset += target.speed_mps - probe.speed_mps;
                r_interface = norm(probe.interface_gcrf.position_m);
            }
            a.braking_delta_v_mps = norm(s.velocity_mps - braked.velocity_mps);
            a.braked = true;
            a.braking_epoch = t;
            s = braked;
        }
    }
    arrive(s, t, target, a);
    return a;
}

double EarthReturnPlanner::perigee_target_m(const EarthArrival& a, const EarthReturnTarget& target) const {
    const double R = constants::wgs84_semi_major_axis_m;
    if (!a.reached) {
        // above the interface (the arrival read at the perigee): the conic's own speed at the interface height, the
        // target angle taken as inertial
        const double r_i = R + target.interface_height_m;
        const auto& s = a.interface_gcrf;
        const double energy = 0.5 * dot(s.velocity_mps, s.velocity_mps) - gm_earth_ / norm(s.position_m);
        const double v_i = std::sqrt(std::max(0.0, 2.0 * (energy + gm_earth_ / r_i)));
        return periapsis_for(r_i, v_i, target.flight_path_rad, gm_earth_) - R;
    }
    // The interface conditions in inertial terms: the radial speed is the same in both frames, the inertial speed
    // differs from the relative one by what the current arrival measures (Earth rotation along the track).
    const double r_i = norm(a.interface_gcrf.position_m);
    const double v_in_now = norm(a.interface_gcrf.velocity_mps);
    const double v_rel_target = target.speed_mps > 0.0 ? target.speed_mps : a.speed_mps;
    const double v_in_target = v_rel_target + (v_in_now - a.speed_mps);
    const double radial = v_rel_target * std::sin(target.flight_path_rad);
    const double gamma_in = std::asin(std::clamp(radial / v_in_target, -1.0, 1.0));
    // the braked conic's perigee differs from the interface state's two-body one by the perturbations on the way
    const double bias = (a.perigee_height_m + R) - periapsis_radius(a.interface_gcrf, gm_earth_);
    return periapsis_for(r_i, v_in_target, gamma_in, gm_earth_) + bias - R;
}

namespace {

// Minimum-norm Newton step dx = J^T (J J^T)^-1 (-f) for 2 or 3 constraints (rows beyond `n` unused).
std::optional<Vec3> newton_step(const std::array<Vec3, 3>& rows, const Vec3& f, int n) {
    math::Mat3 m{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) m.m[i][j] = (i < n && j < n) ? dot(rows[i], rows[j]) : (i == j ? 1.0 : 0.0);
    }
    const auto y = math::solve(m, Vec3{-f.x, -f.y, n > 2 ? -f.z : 0.0});
    if (!y) return std::nullopt;
    return rows[0] * y->x + rows[1] * y->y + (n > 2 ? rows[2] * y->z : Vec3{});
}

double component(const Vec3& v, int i) { return i == 0 ? v.x : i == 1 ? v.y : v.z; }

Vec3 unit(int i) { return i == 0 ? Vec3{1.0, 0.0, 0.0} : i == 1 ? Vec3{0.0, 1.0, 0.0} : Vec3{0.0, 0.0, 1.0}; }

// A point of the gate search: the interface epoch targeted, the delta-v found and the predictor's downrange error.
struct GateSample {
    bool ok = false;
    time::Epoch t;
    Vec3 dv;
    double downrange_m = 0.0;
};

// Newton on the delta-v: `residual` (scaled, n components), `done` the convergence test.
template <class Evaluate, class Residual, class Done>
bool newton(Vec3& dv, int n, int max_iterations, int& iterations, EarthArrival& arrival, Evaluate evaluate,
            Residual residual, Done done, const char* what, double step_mps = finite_difference_mps) {
    auto size = [n](const Vec3& f) { return std::sqrt(f.x * f.x + f.y * f.y + (n > 2 ? f.z * f.z : 0.0)); };
    for (int iteration = 1; iteration <= max_iterations; ++iteration) {
        ++iterations;
        arrival = evaluate(dv);
        const Vec3 f = residual(arrival);
        if (debug_enabled()) {
            std::fprintf(stderr,
                         "return %s %2d: dv %9.3f m/s reached %d braked %.1f m/s v %.1f m/s gamma %.4f deg perigee "
                         "%.3f km cross %.3f km down %.3f km |f| %.3e\n",
                         what, iteration, norm(dv), arrival.reached ? 1 : 0, arrival.braking_delta_v_mps,
                         arrival.speed_mps, arrival.flight_path_rad / constants::deg_to_rad,
                         arrival.perigee_height_m / 1000.0, arrival.crossrange_m / 1000.0,
                         arrival.downrange_error_m / 1000.0, size(f));
        }
        if (done(arrival)) return true;
        std::array<Vec3, 3> rows{};
        // the fine difference for the trajectory rows (perigee and crossrange are nearly collinear: a coarse step bends
        // them), `step_mps` for the predictor's row 2 when it differs
        for (int c = 0; c < 3; ++c) {
            const Vec3 fine = residual(evaluate(dv + unit(c) * finite_difference_mps));
            const Vec3 coarse =
                step_mps == finite_difference_mps ? fine : residual(evaluate(dv + unit(c) * step_mps));
            for (int r = 0; r < 3; ++r) {
                Vec3& row = rows[r];
                const double d = r == 2 ? (component(coarse, r) - component(f, r)) / step_mps
                                        : (component(fine, r) - component(f, r)) / finite_difference_mps;
                (c == 0 ? row.x : c == 1 ? row.y : row.z) = d;
            }
        }
        const auto step = newton_step(rows, f, n);
        if (!step) return false;
        Vec3 delta = *step;
        if (norm(delta) > max_newton_step_mps) delta = delta * (max_newton_step_mps / norm(delta));
        double lambda = 1.0;
        for (int halving = 0; halving < max_halvings; ++halving, lambda *= 0.5) {
            if (size(residual(evaluate(dv + delta * lambda))) < size(f)) break;
        }
        dv += delta * lambda;
    }
    arrival = evaluate(dv);
    return done(arrival);
}

}  // namespace

ReturnBurn EarthReturnPlanner::solve(const StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                     const EarthReturnTarget& target, const Vec3& initial_delta_v_mps,
                                     int max_iterations) const {
    ReturnBurn out;
    const StateVector at_burn =
        burn_epoch - epoch == 0.0 ? state : orbit::propagate_coast(gravity_, state, epoch, burn_epoch).state;
    // stage 1: perigee and plane (the predictor's downrange is defined only once the interface is reached)
    EarthReturnTarget plane = target;
    plane.entry_model.reset();
    auto evaluate_plane = [&](const Vec3& dv) { return evaluate(at_burn, burn_epoch, burn_epoch, dv, plane); };
    auto residual = [&](const EarthArrival& a) {
        return Vec3{(a.perigee_height_m - perigee_target_m(a, target)) / 1e3, a.crossrange_m / 1e3, 0.0};
    };
    auto plane_done = [&](const EarthArrival& a) {
        return a.reached && std::abs(a.perigee_height_m - perigee_target_m(a, target)) < perigee_tolerance_m &&
               std::abs(a.crossrange_m) < crossrange_tolerance_m;
    };
    Vec3 dv = initial_delta_v_mps;
    out.converged =
        newton(dv, 2, max_iterations, out.iterations, out.arrival, evaluate_plane, residual, plane_done, "plane");
    if (out.converged && target.entry_model) {
        // stage 2: the gate, along the plane solutions by the interface epoch
        const auto aimed = aim_at_gate(at_burn, burn_epoch, target, dv, false);
        out.converged = aimed.has_value();
        EarthReturnTarget measured = target;
        if (aimed) {
            dv = aimed->delta_v_mps;
            measured.lateral_offset_m = aimed->lateral_offset_m;
        }
        out.arrival = evaluate(at_burn, burn_epoch, burn_epoch, dv, measured);
    }
    out.burn = {burn_epoch, dv};
    return out;
}

auto EarthReturnPlanner::aim_at_gate(const StateVector& at_burn, time::Epoch burn_epoch,
                                     const EarthReturnTarget& target, const Vec3& plane_delta_v_mps, bool scan) const
    -> std::optional<GateAim> {
    EarthReturnTarget shifted = target;
    auto dv = aim_at_gate_once(at_burn, burn_epoch, shifted, plane_delta_v_mps, scan);
    for (int k = 0; dv; ++k) {
        const auto a = evaluate(at_burn, burn_epoch, burn_epoch, *dv, shifted);
        if (debug_enabled()) {
            std::fprintf(stderr, "return track: lateral offset %.3f km, predicted crossrange %.3f km, downrange %.3f km\n",
                         shifted.lateral_offset_m / 1000.0, a.predicted_crossrange_m / 1000.0,
                         a.downrange_error_m / 1000.0);
        }
        if (!a.reached) return std::nullopt;
        if (std::abs(a.predicted_crossrange_m) < lateral_tolerance_m) return GateAim{*dv, shifted.lateral_offset_m};
        if (k == lateral_iterations) break;
        // the site lies that far beside the track's end: move the aimed plane by it (the arrival time is kept; the
        // predicted crossrange moves one to one with the offset)
        shifted.lateral_offset_m -= a.predicted_crossrange_m;
        dv = aim_at_gate_once(at_burn, burn_epoch, shifted, *dv, false);
    }
    return std::nullopt;
}

std::optional<Vec3> EarthReturnPlanner::aim_at_gate_once(const StateVector& at_burn, time::Epoch burn_epoch,
                                                         const EarthReturnTarget& target,
                                                         const Vec3& plane_delta_v_mps, bool scan) const {
    EarthReturnTarget no_gate = target;
    no_gate.entry_model.reset();
    auto evaluate_plane = [&](const Vec3& dv) { return evaluate(at_burn, burn_epoch, burn_epoch, dv, no_gate); };
    auto downrange = [&](const Vec3& dv) { return evaluate(at_burn, burn_epoch, burn_epoch, dv, target); };
    // perigee, crossrange and the interface epoch `t` from `dv` (warm start), tighter than the plane stage (the
    // downrange moves ~0.4 km per second of arrival and ~2 km per 200 m of perigee); the predictor's downrange there
    auto timed = [&](time::Epoch t, const Vec3& from) {
        auto residual = [&](const EarthArrival& x) {
            return Vec3{(x.perigee_height_m - perigee_target_m(x, target)) / 1e3, x.crossrange_m / 1e3,
                        (x.interface_epoch - t) / 100.0};
        };
        auto done = [&](const EarthArrival& x) {
            return x.reached &&
                   std::abs(x.perigee_height_m - perigee_target_m(x, target)) < gate_perigee_tolerance_m &&
                   std::abs(x.crossrange_m) < gate_crossrange_tolerance_m &&
                   std::abs(x.interface_epoch - t) < arrival_time_tolerance_s;
        };
        GateSample s{false, t, from, 0.0};
        int iterations = 0;
        EarthArrival a;
        if (!newton(s.dv, 3, 30, iterations, a, evaluate_plane, residual, done, "timed")) return s;
        const auto g = downrange(s.dv);
        s.ok = g.reached && g.downrange_error_m != exited_downrange_m;
        s.downrange_m = g.downrange_error_m;
        if (debug_enabled()) {
            std::fprintf(stderr, "return gate: interface %+.4f h, downrange %.3f km, delta-v %.3f m/s\n",
                         (t - burn_epoch) / 3600.0, s.downrange_m / 1000.0, norm(s.dv));
        }
        return s;
    };
    // a sign change between neighbours that is not the target range wrapping past the antipode
    auto brackets = [](const GateSample& a, const GateSample& b) {
        return a.ok && b.ok && (a.downrange_m < 0.0) != (b.downrange_m < 0.0) &&
               std::abs(a.downrange_m - b.downrange_m) < gate_bracket_jump_m;
    };
    // secant on the interface epoch, regula falsi (Illinois) once bracketed, to the gate tolerance
    auto converge = [&](GateSample a, GateSample b) -> std::optional<GateSample> {
        int kept = 0;  // Illinois: the same bracket end kept twice halves its value
        GateSample best = std::abs(a.downrange_m) < std::abs(b.downrange_m) ? a : b;
        const auto accepted = [&]() -> std::optional<GateSample> {
            return std::abs(best.downrange_m) < gate_accept_m ? std::optional<GateSample>(best) : std::nullopt;
        };
        for (int k = 0; k < gate_refine_iterations; ++k) {
            if (std::abs(b.downrange_m) < downrange_tolerance_m) return b;
            if (std::abs(b.downrange_m) < std::abs(best.downrange_m)) best = b;
            if (a.downrange_m == b.downrange_m || a.t - b.t == 0.0) return accepted();  // no secant
            const bool bracketed = brackets(a, b);
            const double f = a.downrange_m / (a.downrange_m - b.downrange_m);
            double t_new = (a.t - burn_epoch) + f * (b.t - a.t);
            if (!bracketed) {
                t_new = std::clamp(t_new, (b.t - burn_epoch) - max_gate_step_s, (b.t - burn_epoch) + max_gate_step_s);
            }
            const double w = (t_new - (a.t - burn_epoch)) / (b.t - a.t);  // warm start along the line a -> b
            const GateSample m = timed(burn_epoch + t_new, a.dv + (b.dv - a.dv) * w);
            if (!m.ok) return accepted();
            if (!bracketed) {
                a = b;
                b = m;
                continue;
            }
            if ((m.downrange_m < 0.0) == (b.downrange_m < 0.0)) {
                b = m;
                if (++kept >= 2) a.downrange_m *= 0.5;
            } else {
                a = b;
                b = m;
                kept = 0;
            }
        }
        if (std::abs(b.downrange_m) < downrange_tolerance_m) return b;
        if (std::abs(b.downrange_m) < std::abs(best.downrange_m)) best = b;
        return accepted();
    };

    // the plane re-aimed at the arrival time (the lateral offset may have moved since `plane_delta_v_mps`)
    const auto first = evaluate_plane(plane_delta_v_mps);
    if (!first.reached) return std::nullopt;
    const GateSample centre = timed(first.interface_epoch, plane_delta_v_mps);
    if (!centre.ok) return std::nullopt;
    if (std::abs(centre.downrange_m) < downrange_tolerance_m) return centre.dv;
    if (!scan) {
        // a nearby gate (mid-course corrections, the session's re-plans): secant from a probe beside the arrival; the
        // site moving downrange shortens the remaining range (later arrival: smaller error)
        const double latitude = frames::fixed_to_geodetic(target.site_itrf_m).latitude_rad;
        const double site_speed =
            constants::earth_rotation_rate_radps * constants::wgs84_semi_major_axis_m * std::cos(latitude);
        const double probe = std::clamp(centre.downrange_m / site_speed, -max_gate_step_s, max_gate_step_s);
        const GateSample second = timed(centre.t + (std::abs(probe) < gate_min_probe_s
                                                        ? std::copysign(gate_min_probe_s, probe)
                                                        : probe),
                                        centre.dv);
        if (!second.ok) return std::nullopt;
        const auto r = converge(centre, second);
        return r ? std::optional<Vec3>(r->dv) : std::nullopt;
    }
    // Walk the interface epoch both ways from the plane solution over half a day each (every site pass: the site's
    // along-track position turns once a day); the cheapest converged crossing wins. A step that does not solve is
    // halved (continuation: l5 Task 10, a 2 h step of the NRHO return's powered flyby crept 30 Newton iterations and
    // closed the scan at most epochs); the warm start follows the last two solutions.
    std::optional<GateSample> best;
    for (const double direction : {1.0, -1.0}) {
        GateSample previous = centre;
        std::optional<GateSample> before;
        double walked_s = 0.0, step_s = gate_scan_step_s;
        while (walked_s < gate_scan_span_s - 1e-6) {
            const double dt = direction * std::min(step_s, gate_scan_span_s - walked_s);
            Vec3 guess = previous.dv;
            if (before) guess += (previous.dv - before->dv) * (dt / (previous.t - before->t));
            const GateSample s = timed(previous.t + dt, guess);
            if (!s.ok) {  // not solved or skipped out: shorter steps, until the way is closed
                step_s *= 0.5;
                if (step_s < gate_scan_min_step_s) break;
                continue;
            }
            walked_s += std::abs(dt);
            if (brackets(previous, s)) {
                if (const auto r = converge(previous, s); r && (!best || norm(r->dv) < norm(best->dv))) best = r;
                break;
            }
            before = previous;
            previous = s;
            step_s = std::min(2.0 * step_s, gate_scan_step_s);
        }
    }
    if (!best) return std::nullopt;
    return best->dv;
}

auto EarthReturnPlanner::departures(const StateVector& state, time::Epoch epoch, const EarthReturnParameters& p) const
    -> std::vector<Departure> {
    std::vector<Departure> out;
    const time::Epoch first = epoch + p.earliest_departure_s;
    const time::Epoch last = epoch + p.search_s;
    const StateVector start = orbit::propagate_coast(gravity_, state, epoch, first).state;
    auto guess_at = [&](const StateVector& s, time::Epoch t, const Vec3& v_inf) {
        const StateVector rel = moon_relative(ephemeris_, s, t);
        const double r = norm(rel.position_m);
        const double v_p = std::sqrt(dot(v_inf, v_inf) + 2.0 * gm_moon_ / r);
        return (v_p - norm(rel.velocity_mps)) * math::normalized(rel.velocity_mps);
    };

    // LLO: per revolution, the point whose outgoing hyperbola (periapsis there, in the orbit plane) leaves along the
    // patched-conic asymptote.
    const StateVector rel0 = moon_relative(ephemeris_, start, first);
    const auto el = orbit::elements_from_state(rel0, gm_moon_);
    const double period = orbit::orbital_period_s(el.semi_major_axis_m, gm_moon_);
    StateVector s = start;
    time::Epoch t = first;
    while (t < last) {
        const time::Epoch revolution_end = t + llo_window_periods * period;
        const Vec3 v_inf =
            excess_velocity(ephemeris_, t + 0.5 * period, p.flight_time_guess_s, p.target.site_itrf_m, earth_);
        if (!(norm(v_inf) > 0.0)) break;
        const StateVector rel = moon_relative(ephemeris_, s, t);
        const Vec3 h = math::normalized(cross(rel.position_m, rel.velocity_mps));
        const Vec3 a = math::normalized(v_inf - h * dot(h, v_inf));
        const double e = 1.0 + norm(rel.position_m) * dot(v_inf, v_inf) / gm_moon_;
        const double nu = std::acos(-1.0 / e);
        const Vec3 d = rotated_about(a, h, -nu);
        // in-plane angle of the position past d (rising through zero at the departure point)
        auto angle = [&](const StateVector& x, time::Epoch tx) {
            const Vec3 r = moon_relative(ephemeris_, x, tx).position_m;
            return std::atan2(dot(h, cross(d, r)), dot(d, r));
        };
        double g_old = angle(s, t);
        bool found = false;
        while (t < revolution_end && t < last) {
            const StateVector next = orbit::propagate_coast(gravity_, s, t, t + llo_sample_s).state;
            const double g_new = angle(next, t + llo_sample_s);
            if (g_old < 0.0 && g_new >= 0.0 && g_new - g_old < constants::pi) {
                double lo = 0.0, hi = llo_sample_s;
                while (hi - lo > departure_tolerance_s) {
                    const double mid = 0.5 * (lo + hi);
                    const StateVector m = orbit::propagate_coast(gravity_, s, t, t + mid).state;
                    (angle(m, t + mid) < 0.0 ? lo : hi) = mid;
                }
                const time::Epoch td = t + hi;
                const StateVector sd = orbit::propagate_coast(gravity_, s, t, td).state;
                out.push_back({td, guess_at(sd, td, v_inf)});
                found = true;
            }
            s = next;
            t = t + llo_sample_s;
            g_old = g_new;
            if (found) break;
        }
        if (found) {
            // half a revolution on: the next window spans the next crossing, not this one again
            s = orbit::propagate_coast(gravity_, s, t, t + 0.5 * period).state;
            t = t + 0.5 * period;
        }
    }
    return out;
}

ReturnBurn EarthReturnPlanner::depart(const StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                      const EarthReturnTarget& target, const Vec3& guess, bool scan_gate) const {
    if (!target.entry_model) return solve(state, epoch, burn_epoch, target, guess);
    // direct entry: the plane first, then (scan) the arrival time searched for the site's pass, both ways
    EarthReturnTarget no_gate = target;
    no_gate.entry_model.reset();
    ReturnBurn sol = solve(state, epoch, burn_epoch, no_gate, guess);
    if (!sol.converged || !scan_gate) return sol;
    const StateVector at_burn =
        burn_epoch - epoch == 0.0 ? state : orbit::propagate_coast(gravity_, state, epoch, burn_epoch).state;
    const auto aimed = aim_at_gate(at_burn, burn_epoch, target, sol.burn.delta_v_gcrf_mps, true);
    sol.converged = aimed.has_value();
    if (aimed) {
        sol.burn.delta_v_gcrf_mps = aimed->delta_v_mps;
        EarthReturnTarget measured = target;
        measured.lateral_offset_m = aimed->lateral_offset_m;
        sol.arrival = evaluate(at_burn, burn_epoch, burn_epoch, aimed->delta_v_mps, measured);
    }
    return sol;
}

EarthReturnPlanner::NrhoReturn EarthReturnPlanner::nrho_return(const StateVector& state, time::Epoch epoch,
                                                               time::Epoch perilune,
                                                               const EarthReturnParameters& p) const {
    NrhoReturn best;
    const double r_p = constants::moon_mean_radius_m + p.flyby_altitude_m;
    const time::Epoch earliest = epoch + p.earliest_departure_s;

    // Patched-conic estimate for a departure at t_nrd and a flyby at the periselene point P (Moon-relative) at t_flyby:
    // departure burn = Moon two-body Lambert arc from the departure point to P (either sense), powered flyby = the
    // hyperbola through P with the return's excess velocity (patched conic through the site's plane; the targeting trims
    // it) minus the arc's arrival velocity.
    struct Option {
        double cost = std::numeric_limits<double>::infinity();
        time::Epoch t_nrd, t_flyby;
        StateVector at_nrd;
        Vec3 periselene, nrd, rpf_velocity;
    };
    std::vector<Option> options;  // the best flyby point per departure and flyby time
    auto search = [&](time::Epoch t_nrd, const StateVector& at_nrd, time::Epoch t_flyby) {
        const StateVector nrd_rel = moon_relative(ephemeris_, at_nrd, t_nrd);
        const Vec3 v_inf = excess_velocity(ephemeris_, t_flyby, p.flight_time_guess_s, p.target.site_itrf_m, earth_);
        if (!(norm(v_inf) > 0.0)) return;
        const Vec3 h = cross(nrd_rel.position_m, nrd_rel.velocity_mps);
        auto estimate = [&](const Vec3& periselene, Option& o) {
            const auto v_req = departure_velocity(periselene, v_inf, gm_moon_);
            if (!v_req) return false;
            bool found = false;
            for (const Vec3& normal : {h, -h}) {
                const auto arc = orbit::solve_lambert(nrd_rel.position_m, periselene, t_flyby - t_nrd, gm_moon_, normal);
                if (!arc) continue;
                const Vec3 nrd = arc->departure_velocity_mps - nrd_rel.velocity_mps;
                const double cost = norm(nrd) + norm(*v_req - arc->arrival_velocity_mps);
                if (cost < o.cost) {
                    o = {cost, t_nrd, t_flyby, at_nrd, periselene, nrd, *v_req};
                    found = true;
                }
            }
            return found;
        };
        // the flyby point on the sphere at the flyby height, in the frame of the return asymptote: coarse grid, then
        // two finer grids around the best point (a 100 km flyby turns the path by up to ~150 deg: the needed turn
        // depends on the approach)
        const Vec3 ax = math::normalized(v_inf);
        const Vec3 ay = math::normalized(cross(Vec3{0.0, 0.0, 1.0}, ax));
        const Vec3 az = cross(ax, ay);
        auto point = [&](double lat, double lon) {
            return (ax * (std::cos(lat) * std::cos(lon)) + ay * (std::cos(lat) * std::sin(lon)) + az * std::sin(lat)) *
                   r_p;
        };
        Option local;
        double b_lat = 0.0, b_lon = constants::pi;
        for (const double half : {0.5 * constants::pi, flyby_point_step_rad, flyby_point_step_rad / 5.0}) {
            const double step = half >= 0.5 * constants::pi ? flyby_point_step_rad : half / 5.0;
            const double lat0 = b_lat, lon0 = b_lon;
            for (double lat = lat0 - half; lat <= lat0 + half + 1e-9; lat += step) {
                if (std::abs(lat) > 0.5 * constants::pi) continue;
                for (double lon = lon0 - half; lon <= lon0 + half + 1e-9; lon += step) {
                    if (estimate(point(lat, lon), local)) {
                        b_lat = lat;
                        b_lon = lon;
                    }
                }
            }
        }
        if (std::isfinite(local.cost)) options.push_back(local);
    };
    // the departure time before the perilune and the flyby time about it [E]: both shape the approach and the turn
    StateVector s = state;
    time::Epoch t = epoch;
    for (double lead = nrd_lead_max_s; lead >= nrd_lead_min_s - 1e-6; lead -= nrd_lead_step_s) {
        // walk forward: the earliest departure (longest lead) first
        const time::Epoch t_nrd = perilune - lead;
        if (t_nrd < earliest) continue;
        s = orbit::propagate_coast(gravity_, s, t, t_nrd).state;
        t = t_nrd;
        for (double offset = -flyby_offset_max_s; offset <= flyby_offset_max_s + 1e-6; offset += flyby_offset_step_s) {
            if (lead + offset < nrd_lead_min_s) continue;  // the arc to the flyby stays at least this long
            search(t_nrd, s, perilune + offset);
        }
    }
    // Full force model, the options cheapest first (the estimate ignores the gate): the departure burn by position
    // targeting onto P at the flyby epoch, the powered flyby there by the return targeting (plane, perigee, and the gate
    // for a direct entry). The departure and flyby times move the arrival by hours - where the cheapest option's arrival
    // cannot bring the entry to the site (the site's place along the arrival track spans only part of a turn: l5 Task
    // 10), a later or earlier departure can, as a mission planner trades the departure for the landing site. The
    // cheapest converged option wins.
    std::sort(options.begin(), options.end(), [](const Option& a, const Option& b) { return a.cost < b.cost; });
    if (options.size() > max_nrho_options) options.resize(max_nrho_options);
    double best_mps = std::numeric_limits<double>::infinity();
    for (const Option& chosen : options) {
        // the estimate nearly bounds an option from below (aiming at the gate only adds; the full model came out up to
        // ~4 % below the patched conic): none left can beat the best found
        if (nrho_estimate_bound_fraction * chosen.cost >= best_mps) break;
        const auto moon = ephemeris_.state_gcrf(ephemeris::Body::Moon, chosen.t_flyby);
        const auto drop = position_.solve(chosen.at_nrd, chosen.t_nrd, chosen.t_nrd,
                                          {moon.position_m + chosen.periselene, chosen.t_flyby}, chosen.nrd,
                                          flyby_position_tolerance_m);
        if (debug_enabled()) {
            std::fprintf(stderr,
                         "return NRHO: departure %.2f d before the perilune, flyby %+.1f h from it: estimate %.1f m/s "
                         "(NRD %.1f), drop %s %.1f m/s\n",
                         (perilune - chosen.t_nrd) / 86400.0, (chosen.t_flyby - perilune) / 3600.0, chosen.cost,
                         norm(chosen.nrd), drop.converged ? "converged" : "failed", norm(drop.burn.delta_v_gcrf_mps));
        }
        if (!drop.converged) continue;
        const StateVector& at_flyby = drop.arrival_gcrf;
        const Vec3 rpf_guess = moon.velocity_mps + chosen.rpf_velocity - at_flyby.velocity_mps;
        const ReturnBurn flyby = depart(at_flyby, chosen.t_flyby, chosen.t_flyby, p.target, rpf_guess, true);
        if (debug_enabled()) {
            std::fprintf(stderr, "return NRHO powered flyby: guess %.1f m/s -> %s %.1f m/s\n", norm(rpf_guess),
                         flyby.converged ? "converged" : "failed", norm(flyby.burn.delta_v_gcrf_mps));
        }
        const double cost = norm(drop.burn.delta_v_gcrf_mps) + norm(flyby.burn.delta_v_gcrf_mps);
        if (flyby.converged && cost < best_mps) {
            best_mps = cost;
            best = {true, drop.burn, flyby};
        }
    }
    return best;
}

ReturnPlan EarthReturnPlanner::plan(const StateVector& state, time::Epoch epoch, const EarthReturnParameters& p,
                                    double mass_kg, double isp_s, double propellant_kg) const {
    ReturnPlan plan;
    if (p.orbit == LunarOrbitKind::Nrho && !p.nrho) {
        plan.reason = "Kein Rückflug: NRHO-Referenz fehlt";
        return plan;
    }
    double best = std::numeric_limits<double>::infinity();
    bool over_limit = false;
    // keeps a converged candidate: `departure_mps` the Moon-side delta-v (TEI, or departure burn + powered flyby)
    auto consider = [&](const ReturnBurn& sol, double departure_mps, std::optional<gnc::BurnNode> nrd) {
        if (!sol.converged) return;
        if (departure_mps > p.max_tei_mps) {
            over_limit = true;
            return;
        }
        const double cost = departure_mps + sol.arrival.braking_delta_v_mps;
        if (cost < best) {
            best = cost;
            plan.nrho_departure = nrd;
            plan.tei = sol.burn;
            plan.arrival = sol.arrival;
            plan.tei_delta_v_mps = departure_mps;
            plan.braking_delta_v_mps = sol.arrival.braking_delta_v_mps;
        }
    };
    if (p.orbit == LunarOrbitKind::Nrho) {
        for (const time::Epoch t : p.nrho->perilune_epochs()) {
            if (t - nrd_lead_min_s < epoch + p.earliest_departure_s || t > epoch + p.search_s) continue;
            const auto r = nrho_return(state, epoch, t, p);
            if (!r.found) continue;
            consider(r.flyby, norm(r.departure.delta_v_gcrf_mps) + norm(r.flyby.burn.delta_v_gcrf_mps), r.departure);
        }
    } else {
        for (const auto& c : departures(state, epoch, p)) {
            const auto sol = depart(state, epoch, c.epoch, p.target, c.delta_v_mps, true);
            if (debug_enabled()) {
                std::fprintf(stderr, "return departure %+.2f h: guess %.1f m/s -> %s TEI %.1f m/s, braking %.1f m/s (%d it)\n",
                             (c.epoch - epoch) / 3600.0, norm(c.delta_v_mps), sol.converged ? "converged" : "failed",
                             norm(sol.burn.delta_v_gcrf_mps), sol.arrival.braking_delta_v_mps, sol.iterations);
            }
            consider(sol, norm(sol.burn.delta_v_gcrf_mps), std::nullopt);
        }
    }
    char text[160];
    if (!std::isfinite(best)) {
        if (over_limit) {
            std::snprintf(text, sizeof text, "Kein Rückflug: TEI über %.0f m/s (Bahnebene passt nicht)", p.max_tei_mps);
            plan.reason = text;
        } else {
            plan.reason = "Kein Rückflug: TEI nicht planbar";
        }
        return plan;
    }
    const double total = plan.tei_delta_v_mps + plan.braking_delta_v_mps + p.mcc_allowance_mps;
    const double needed = mass_kg * (1.0 - std::exp(-total / (isp_s * constants::standard_gravity_mps2)));
    if (needed > propellant_kg) {
        std::snprintf(text, sizeof text, "Kein Rückflug: Treibstoff reicht nicht (%.1f t fehlen)",
                      (needed - propellant_kg) / 1000.0);
        plan.reason = text;
        return plan;
    }
    plan.feasible = true;
    return plan;
}

std::optional<time::Epoch> EarthReturnPlanner::braking_epoch(const StateVector& state, time::Epoch epoch,
                                                             const EarthReturnTarget& target) const {
    const double r_brake = constants::wgs84_semi_major_axis_m + target.braking_height_m;
    if (norm(state.position_m) <= r_brake) return std::nullopt;
    const integrators::Event<6> braking{
        "braking height",
        [&](double, const integrators::State<6>& y) { return std::hypot(y[0], y[1], y[2]) - r_brake; },
        integrators::Crossing::Falling};
    const auto coast = orbit::propagate_coast(gravity_, state, epoch, epoch + max_return_s, std::span(&braking, 1));
    if (coast.event_index != 0) return std::nullopt;
    return coast.epoch;
}

std::optional<gnc::BurnNode> EarthReturnPlanner::mid_course(const StateVector& state, time::Epoch epoch,
                                                            time::Epoch burn_epoch,
                                                            const EarthReturnParameters& p) const {
    if (const auto brake = braking_epoch(state, epoch, p.target);
        brake && burn_epoch > *brake - p.mcc_min_before_braking_s) {
        return std::nullopt;
    }
    const auto sol = solve(state, epoch, burn_epoch, p.target, Vec3{});
    if (!sol.converged || norm(sol.burn.delta_v_gcrf_mps) < p.mcc_threshold_mps) return std::nullopt;
    return sol.burn;
}

std::optional<gnc::BurnNode> EarthReturnPlanner::braking(const StateVector& state, time::Epoch epoch,
                                                         const EarthReturnTarget& target) const {
    const auto tb = braking_epoch(state, epoch, target);
    if (!tb) return std::nullopt;
    const StateVector at = orbit::propagate_coast(gravity_, state, epoch, *tb).state;
    // first guess: the vis-viva retrograde burn
    const auto sized = evaluate(at, *tb, *tb, Vec3{}, [&] {
        auto t = target;
        t.entry_model.reset();
        return t;
    }());
    const Vec3 dv = -math::normalized(at.velocity_mps) * sized.braking_delta_v_mps;
    const auto aimed = aim_interface(at, *tb, target, dv, "braking");
    if (!aimed) return std::nullopt;
    return gnc::BurnNode{*tb, *aimed};
}

std::optional<gnc::BurnNode> EarthReturnPlanner::entry_trim(const StateVector& state, time::Epoch epoch,
                                                            time::Epoch burn_epoch, const EarthReturnTarget& target) const {
    const StateVector at = orbit::propagate_coast(gravity_, state, epoch, burn_epoch).state;
    const auto aimed = aim_interface(at, burn_epoch, target, Vec3{}, "entry trim");
    if (!aimed || norm(*aimed) < entry_trim_threshold_mps) return std::nullopt;
    return gnc::BurnNode{burn_epoch, *aimed};
}

std::optional<Vec3> EarthReturnPlanner::aim_interface(const StateVector& at, time::Epoch burn_epoch,
                                                      const EarthReturnTarget& target, const Vec3& guess,
                                                      const char* what) const {
    // l5 Task 10: speed, angle and (direct) the predicted crossrange - the entry guidance nulls the downrange (it held
    // ~0 km through the nominal entry) but did not reverse for ~25 km across, and the terminal phase missed the gate.
    // The former downrange row was badly conditioned (stalled at 1.6-1.9 km in every campaign case) and left the
    // crossrange free.
    EarthReturnTarget coasting = target;
    coasting.speed_mps = 0.0;  // this burn is the last braking: none modelled after it
    const int n = target.entry_model ? 3 : 2;
    auto evaluate_burn = [&](const Vec3& x) { return evaluate(at, burn_epoch, burn_epoch, x, coasting); };
    auto residual = [&](const EarthArrival& a) {
        return Vec3{(a.speed_mps - target.speed_mps) / 10.0,
                    (a.flight_path_rad - target.flight_path_rad) / constants::deg_to_rad * 10.0,
                    a.predicted_crossrange_m / 1e4};
    };
    const auto speed_and_angle = [&](const EarthArrival& a) {
        return a.reached && std::abs(a.speed_mps - target.speed_mps) < braking_speed_tolerance_mps &&
               std::abs(a.flight_path_rad - target.flight_path_rad) < braking_angle_tolerance_rad;
    };
    auto done = [&](const EarthArrival& a) {
        return speed_and_angle(a) && (n < 3 || std::abs(a.predicted_crossrange_m) < lateral_tolerance_m);
    };
    int iterations = 0;
    EarthArrival arrival;
    // Speed and angle first: from a start far off the corridor (a pass that skips out in the predictor) the crossrange
    // is meaningless and the 3-row Newton wandered to 830 m/s before it found a 13 m/s trim (campaign seed 9).
    Vec3 corridor = guess;
    const bool on_corridor = newton(corridor, 2, 30, iterations, arrival, evaluate_burn, residual, speed_and_angle, what);
    if (n < 3) return on_corridor ? std::optional<Vec3>(corridor) : std::nullopt;
    Vec3 dv = on_corridor ? corridor : guess;
    if (newton(dv, n, 30, iterations, arrival, evaluate_burn, residual, done, what, predictor_difference_mps)) return dv;
    // not converged with the crossrange: speed and angle alone (the corridor; the guidance reverses for the rest)
    if (on_corridor) return corridor;
    return std::nullopt;
}

}  // namespace starsim::mission
