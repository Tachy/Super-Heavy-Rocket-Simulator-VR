#include "starsim/mission/lunar_transfer.hpp"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/lambert.hpp"

namespace starsim::mission {

using math::StateVector;
using math::Vec3;

namespace {

constexpr double max_transfer_s = 10.0 * 86400.0;
constexpr double lunar_vicinity_m = 1.0e8;     // closest approaches farther away do not count
constexpr double finite_difference_mps = 0.01;  // Jacobian step
constexpr double max_newton_step_mps = 100.0;

}  // namespace

LunarTransferTargeter::LunarTransferTargeter(const environment::GravityModel& gravity,
                                             const ephemeris::Ephemeris& ephemeris,
                                             const frames::BodyOrientation& moon_orientation)
    : gravity_(gravity),
      ephemeris_(ephemeris),
      moon_orientation_(moon_orientation),
      gm_earth_(ephemeris.gm_m3ps2(ephemeris::Body::Earth)),
      gm_moon_(ephemeris.gm_m3ps2(ephemeris::Body::Moon)) {}

StateVector LunarTransferTargeter::coast(const StateVector& state, time::Epoch from, time::Epoch to) const {
    if (to - from == 0.0) return state;
    return orbit::propagate_coast(gravity_, state, from, to).state;
}

LunarArrival LunarTransferTargeter::evaluate(const StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                             const Vec3& delta_v) const {
    StateVector start = coast(state, epoch, burn_epoch);
    if (finite_) {
        start = apply_finite_burn(gravity_, start, burn_epoch, delta_v, *finite_);
    } else {
        start.velocity_mps += delta_v;
    }

    auto relative = [&](double t, const integrators::State<6>& y) {
        const auto moon = ephemeris_.state_gcrf(ephemeris::Body::Moon, burn_epoch + t);
        return StateVector{Vec3{y[0], y[1], y[2]} - moon.position_m, Vec3{y[3], y[4], y[5]} - moon.velocity_mps};
    };
    const std::vector<integrators::Event<6>> events{
        // Closest approach: radial velocity relative to the Moon turns positive.
        {"periselene",
         [&](double t, const integrators::State<6>& y) {
             const auto rel = relative(t, y);
             return dot(rel.position_m, rel.velocity_mps) / norm(rel.position_m);
         },
         integrators::Crossing::Rising},
        {"impact",
         [&](double t, const integrators::State<6>& y) {
             return norm(relative(t, y).position_m) - constants::moon_mean_radius_m;
         },
         integrators::Crossing::Falling}};
    const auto result =
        orbit::propagate_coast(gravity_, start, burn_epoch, burn_epoch + max_transfer_s, events);

    LunarArrival arrival;
    if (result.event_index < 0) return arrival;
    const auto moon = ephemeris_.state_gcrf(ephemeris::Body::Moon, result.epoch);
    const StateVector rel{result.state.position_m - moon.position_m, result.state.velocity_mps - moon.velocity_mps};
    if (norm(rel.position_m) > lunar_vicinity_m) return arrival;

    // Osculating periselene (exact at the closest-approach event, smooth extrapolation after an impact).
    const auto el = orbit::elements_from_state(rel, gm_moon_);
    const Vec3 h = cross(rel.position_m, rel.velocity_mps);
    const auto pole_matrix = moon_orientation_.fixed_to_gcrf(result.epoch).rotation;
    const Vec3 pole{pole_matrix(0, 2), pole_matrix(1, 2), pole_matrix(2, 2)};
    arrival.reached = true;
    arrival.periselene_epoch = result.epoch;
    arrival.periselene_altitude_m = el.semi_major_axis_m * (1.0 - el.eccentricity) - constants::moon_mean_radius_m;
    arrival.inclination_rad = std::acos(std::clamp(dot(h, pole) / norm(h), -1.0, 1.0));
    arrival.relative_state = rel;
    return arrival;
}

Vec3 LunarTransferTargeter::lambert_guess(const StateVector& at_burn, time::Epoch burn_epoch,
                                          const LunarArrivalTarget& target) const {
    const Vec3 moon = ephemeris_.state_gcrf(ephemeris::Body::Moon, target.periselene_epoch).position_m;
    const Vec3 normal = math::normalized(cross(at_burn.position_m, at_burn.velocity_mps));
    // Aim beside the Moon (in the transfer plane) instead of at its centre.
    const Vec3 side = math::normalized(cross(normal, moon));
    const Vec3 aim = moon + side * (constants::moon_mean_radius_m + target.periselene_altitude_m);
    const auto lambert = orbit::solve_lambert(at_burn.position_m, aim, target.periselene_epoch - burn_epoch,
                                              gm_earth_, normal);
    return lambert ? lambert->departure_velocity_mps - at_burn.velocity_mps : Vec3{};
}

time::Epoch LunarTransferTargeter::best_departure(const StateVector& state, time::Epoch epoch, time::Epoch earliest,
                                                  time::Epoch arrival_epoch, double step_s) const {
    const auto el = orbit::elements_from_state(state, gm_earth_);
    const double period = orbit::orbital_period_s(el.semi_major_axis_m, gm_earth_);
    LunarArrivalTarget target;
    target.periselene_epoch = arrival_epoch;
    StateVector s = coast(state, epoch, earliest);
    time::Epoch best = earliest;
    double best_dv = INFINITY;
    for (double t = 0.0; t <= period; t += step_s) {
        const time::Epoch burn = earliest + t;
        if (t > 0.0) s = coast(s, burn - step_s, burn);
        const Vec3 moon = ephemeris_.state_gcrf(ephemeris::Body::Moon, arrival_epoch).position_m;
        const double angle = std::acos(std::clamp(dot(s.position_m, moon) / (norm(s.position_m) * norm(moon)), -1.0, 1.0));
        if (angle > 175.0 * constants::deg_to_rad || angle < 90.0 * constants::deg_to_rad) continue;
        const Vec3 dv = lambert_guess(s, burn, target);
        if (std::getenv("STARSIM_VERBOSE") && static_cast<int>(t) % 600 == 0) {
            std::printf("  departure +%5.0f s: angle %6.1f deg, lambert dv %8.1f m/s\n", t, angle / constants::deg_to_rad, norm(dv));
        }
        if (norm(dv) > 0.0 && norm(dv) < best_dv) {
            best_dv = norm(dv);
            best = burn;
        }
    }
    return best;
}

TransferSolution LunarTransferTargeter::solve(const StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                              const LunarArrivalTarget& target,
                                              std::optional<Vec3> initial_delta_v) const {
    TransferSolution solution;
    Vec3 dv = initial_delta_v ? *initial_delta_v : lambert_guess(coast(state, epoch, burn_epoch), burn_epoch, target);

    // B-plane style targeting: periselene altitude and lunar inclination; the arrival time stays free
    // (it follows from the energy). Two constraints, three unknowns: minimum-norm Newton steps find the
    // smallest correction. Scaled residuals: 100 km, 0.1 rad.
    auto residual = [&](const LunarArrival& a) {
        return std::array<double, 2>{(a.periselene_altitude_m - target.periselene_altitude_m) / 1.0e5,
                                     (a.inclination_rad - target.inclination_rad) / 0.1};
    };
    auto size = [](const std::array<double, 2>& r) { return std::hypot(r[0], r[1]); };
    for (int iteration = 1; iteration <= 40; ++iteration) {
        solution.iterations = iteration;
        const auto arrival = evaluate(state, epoch, burn_epoch, dv);
        solution.arrival = arrival;
        solution.burn = {burn_epoch, dv};
        if (!arrival.reached) return solution;
        const auto f = residual(arrival);
        if (std::getenv("STARSIM_VERBOSE")) {
            std::printf("  newton %2d: dv %8.1f m/s  alt %10.3f km  i %8.4f deg  arrival %+9.1f s\n", iteration,
                        norm(dv), arrival.periselene_altitude_m / 1000.0, arrival.inclination_rad * 180.0 / constants::pi,
                        arrival.periselene_epoch - target.periselene_epoch);
        }
        if (std::abs(arrival.periselene_altitude_m - target.periselene_altitude_m) < 50.0 &&
            std::abs(arrival.inclination_rad - target.inclination_rad) < 1e-4) {
            solution.converged = true;
            return solution;
        }
        // Jacobian (2 x 3) by forward differences.
        std::array<std::array<double, 3>, 2> j{};
        for (int c = 0; c < 3; ++c) {
            Vec3 perturbed = dv;
            (c == 0 ? perturbed.x : c == 1 ? perturbed.y : perturbed.z) += finite_difference_mps;
            const auto a = evaluate(state, epoch, burn_epoch, perturbed);
            if (!a.reached) return solution;
            const auto fp = residual(a);
            for (int r = 0; r < 2; ++r) j[r][c] = (fp[r] - f[r]) / finite_difference_mps;
        }
        // Minimum-norm step: delta = J^T (J J^T)^-1 (-f).
        const double m00 = j[0][0] * j[0][0] + j[0][1] * j[0][1] + j[0][2] * j[0][2];
        const double m01 = j[0][0] * j[1][0] + j[0][1] * j[1][1] + j[0][2] * j[1][2];
        const double m11 = j[1][0] * j[1][0] + j[1][1] * j[1][1] + j[1][2] * j[1][2];
        const double det = m00 * m11 - m01 * m01;
        const double y0 = (m11 * -f[0] - m01 * -f[1]) / det;
        const double y1 = (m00 * -f[1] - m01 * -f[0]) / det;
        Vec3 delta{j[0][0] * y0 + j[1][0] * y1, j[0][1] * y0 + j[1][1] * y1, j[0][2] * y0 + j[1][2] * y1};
        if (norm(delta) > max_newton_step_mps) delta = delta * (max_newton_step_mps / norm(delta));
        // Backtracking line search: the lunar flyby is strongly non-linear.
        double lambda = 1.0;
        for (int halving = 0; halving < 8; ++halving, lambda *= 0.5) {
            const auto trial = evaluate(state, epoch, burn_epoch, dv + delta * lambda);
            if (trial.reached && size(residual(trial)) < size(f)) break;
        }
        dv += delta * lambda;
    }
    return solution;
}

std::optional<double> LunarTransferTargeter::parking_orbit_raan(time::Epoch arrival_epoch,
                                                                double inclination_rad) const {
    const Vec3 m = math::normalized(ephemeris_.state_gcrf(ephemeris::Body::Moon, arrival_epoch).position_m);
    // Orbit normal n: perpendicular to m with n_z = cos(i). Basis of the plane perpendicular to m:
    // e1 (largest z component) and e2 = m x e1 (no z component).
    const Vec3 e1 = math::normalized(Vec3{0.0, 0.0, 1.0} - m * m.z);
    const Vec3 e2 = cross(m, e1);
    const double a = std::cos(inclination_rad) / e1.z;
    if (std::abs(a) > 1.0) return std::nullopt;
    const Vec3 n = a * e1 + std::sqrt(1.0 - a * a) * e2;
    return std::atan2(n.x, -n.y);  // node line = z x n
}

gnc::BurnNode LunarTransferTargeter::orbit_insertion(const LunarArrival& arrival, double gm_moon) {
    const auto& rel = arrival.relative_state;
    const double r = norm(rel.position_m);
    const Vec3 circular = std::sqrt(gm_moon / r) * math::normalized(rel.velocity_mps);
    return {arrival.periselene_epoch, circular - rel.velocity_mps};
}

}  // namespace starsim::mission
