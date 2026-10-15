#include "starsim/mission/nrho_reference.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"

namespace starsim::mission {

using math::StateVector;
using math::Vec3;

namespace {
constexpr double position_step_m = 1.0;  // finite-difference steps of the state transition matrix
constexpr double velocity_step_mps = 1e-3;
// Velocities enter the minimum-norm step scaled by the CR3BP time unit, so that 1 m of position and 1 m/s of velocity
// weigh as in nondimensional Earth-Moon units (unscaled, the step would put the whole correction into velocity).
constexpr double velocity_scale_s = constants::sidereal_month_s / (2.0 * constants::pi);
constexpr int line_search_steps = 10;
// Patch points per half revolution, crowded towards the perilune where the motion is fastest.
constexpr int patches_per_half_revolution = 4;
// Perilune search: window around the expected passage, and the resolution of the closest-approach epoch.
constexpr double perilune_window_s = 0.5 * 86400.0;
constexpr double closest_approach_tolerance_s = 1.0;

using Vec6 = std::array<double, 6>;

double norm2(const std::vector<double>& f) {
    double s = 0.0;
    for (double v : f) s += v * v;
    return std::sqrt(s);
}

Vec6 to_scaled(const StateVector& s) {
    const Vec3 v = velocity_scale_s * s.velocity_mps;
    return {s.position_m.x, s.position_m.y, s.position_m.z, v.x, v.y, v.z};
}

StateVector from_scaled(const Vec6& y) { return {{y[0], y[1], y[2]}, Vec3{y[3], y[4], y[5]} / velocity_scale_s}; }

// Dense symmetric positive definite solve (Cholesky) in place; false when the matrix is not positive definite.
bool solve_spd(std::vector<double> a, std::vector<double>& b, size_t n) {
    for (size_t j = 0; j < n; ++j) {
        double d = a[j * n + j];
        for (size_t k = 0; k < j; ++k) d -= a[j * n + k] * a[j * n + k];
        if (!(d > 0.0)) return false;
        a[j * n + j] = std::sqrt(d);
        for (size_t i = j + 1; i < n; ++i) {
            double s = a[i * n + j];
            for (size_t k = 0; k < j; ++k) s -= a[i * n + k] * a[j * n + k];
            a[i * n + j] = s / a[j * n + j];
        }
    }
    for (size_t i = 0; i < n; ++i) {  // L z = b
        for (size_t k = 0; k < i; ++k) b[i] -= a[i * n + k] * b[k];
        b[i] /= a[i * n + i];
    }
    for (size_t i = n; i-- > 0;) {  // L^T x = z
        for (size_t k = i + 1; k < n; ++k) b[i] -= a[k * n + i] * b[k];
        b[i] /= a[i * n + i];
    }
    return true;
}

// STARSIM_DEBUG_NRHO=1 prints every shooting iteration.
bool debug_enabled() {
    static const bool enabled = std::getenv("STARSIM_DEBUG_NRHO") != nullptr;
    return enabled;
}

// Fraction of a perilune-to-perilune revolution at patch r = 0..2n (apolune at r = n), crowded towards the perilunes.
double phase_after_perilune(int r) {
    constexpr int n = patches_per_half_revolution;
    return r <= n ? 0.5 - 0.5 * std::sin(0.5 * constants::pi * (n - r) / n)
                  : 0.5 + 0.5 * std::sin(0.5 * constants::pi * (r - n) / n);
}

StateVector coast(const environment::GravityModel& gravity, const StateVector& s, time::Epoch from, time::Epoch to) {
    if (to - from == 0.0) return s;
    return orbit::propagate_coast(gravity, s, from, to).state;
}

// Epoch of the closest approach to the Moon within +-window_s of `guess` on the coast from `state` at `epoch`
// (golden-section search; the perilune passage is the only minimum of the distance in the window).
time::Epoch closest_approach(const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                             const StateVector& state, time::Epoch epoch, time::Epoch guess, double window_s) {
    const auto lo = guess - window_s;
    const StateVector at_lo = coast(gravity, state, epoch, lo);
    auto distance = [&](double dt) {
        const auto t = lo + dt;
        return norm(coast(gravity, at_lo, lo, t).position_m - ephemeris.state_gcrf(ephemeris::Body::Moon, t).position_m);
    };
    const double ratio = 0.5 * (std::sqrt(5.0) - 1.0);
    double a = 0.0, b = 2.0 * window_s;
    double c = b - ratio * (b - a), d = a + ratio * (b - a);
    double fc = distance(c), fd = distance(d);
    while (b - a > closest_approach_tolerance_s) {
        if (fc < fd) {
            b = d, d = c, fd = fc;
            c = b - ratio * (b - a), fc = distance(c);
        } else {
            a = c, c = d, fc = fd;
            d = a + ratio * (b - a), fd = distance(d);
        }
    }
    return lo + 0.5 * (a + b);
}

// Step 1: the CR3BP periodic orbit of the resonant period, state at perilune.
orbit::Cr3bpState cr3bp_perilune(double mu, const NrhoSeed& seed, double period) {
    const auto apolune = orbit::correct_periodic_orbit(mu, seed.apolune, period);
    if (!apolune) throw std::runtime_error("NRHO: keine periodische Bahn im Dreikörperproblem gefunden");
    return orbit::cr3bp_propagate(mu, *apolune, 0.5 * period);
}

// CR3BP state converted at an epoch. Rotating-frame velocities are scaled with sqrt(GM / L^3) of the instantaneous
// distance L (not with the instantaneous angular rate), so that the energy relative to the Moon is consistent.
StateVector convert(const ephemeris::Ephemeris& ephemeris, const orbit::Cr3bpState& s, time::Epoch epoch) {
    const orbit::EarthMoonRotatingFrame frame(ephemeris, epoch);
    const double gm = ephemeris.gm_m3ps2(ephemeris::Body::Earth) + ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    const double l = frame.length_unit_m();
    const double k = std::sqrt(gm / (l * l * l)) * frame.time_unit_s();
    return frame.to_gcrf({s.position, k * s.velocity});
}

// Multiple shooting on all patches: minimum-norm Newton with backtracking, epochs fixed.
class Shooter {
public:
    Shooter(const environment::GravityModel& gravity, const std::vector<time::Epoch>& epochs,
            const NrhoReferenceOptions& options)
        : gravity_(gravity), epochs_(epochs), options_(options) {}

    // Returns the iterations; throws when it does not converge.
    int solve(std::vector<Vec6>& x) {
        const size_t count = x.size(), rows = 6 * (count - 1), cols = 6 * count;
        std::vector<double> f, jac(rows * cols), step(cols);
        for (int it = 0;; ++it) {
            const bool ok = residual(x, f);
            if (debug_enabled())
                std::fprintf(stderr, "NRHO shooting: %zu patches, iteration %d, max jump %.3f m, |F| %.4g\n", count,
                             it, max_jump_m_, norm2(f));
            if (ok) return it;
            if (it >= options_.max_iterations) throw std::runtime_error("NRHO: Mehrfachschießverfahren konvergiert nicht");
            jacobian(x, jac);
            const double before = norm2(f);
            minimum_norm_step(jac, f, step);
            // Backtracking: halve the step until the residual drops.
            const auto x0 = x;
            double alpha = 1.0;
            for (int halving = 0;; ++halving, alpha *= 0.5) {
                for (size_t c = 0; c < cols; ++c) x[c / 6][c % 6] = x0[c / 6][c % 6] + alpha * step[c];
                residual(x, f);
                if (norm2(f) < before || halving == line_search_steps) break;
            }
        }
    }

    double max_jump_m() const { return max_jump_m_; }

private:
    Vec6 scaled_coast(const Vec6& y, size_t j) const {
        return to_scaled(coast(gravity_, from_scaled(y), epochs_[j], epochs_[j + 1]));
    }

    // Continuity residual of every segment; true when all jumps are within the tolerances.
    bool residual(const std::vector<Vec6>& x, std::vector<double>& f) {
        f.assign(6 * (x.size() - 1), 0.0);
        bool ok = true;
        max_jump_m_ = 0.0;
        for (size_t j = 0; j + 1 < x.size(); ++j) {
            const Vec6 end = scaled_coast(x[j], j);
            for (int k = 0; k < 6; ++k) f[6 * j + k] = end[k] - x[j + 1][k];
            const double dr = std::hypot(f[6 * j], f[6 * j + 1], f[6 * j + 2]);
            const double dv = std::hypot(f[6 * j + 3], f[6 * j + 4], f[6 * j + 5]) / velocity_scale_s;
            max_jump_m_ = std::max(max_jump_m_, dr);
            ok = ok && dr < options_.continuity_tolerance_m && dv < options_.continuity_tolerance_mps;
        }
        return ok;
    }

    // dF_j/dX_j = state transition matrix (central differences), dF_j/dX_{j+1} = -I.
    void jacobian(const std::vector<Vec6>& x, std::vector<double>& jac) const {
        const size_t cols = 6 * x.size();
        std::fill(jac.begin(), jac.end(), 0.0);
        for (size_t j = 0; j + 1 < x.size(); ++j) {
            for (int k = 0; k < 6; ++k) {
                const double h = k < 3 ? position_step_m : velocity_step_mps * velocity_scale_s;
                Vec6 plus = x[j], minus = x[j];
                plus[k] += h;
                minus[k] -= h;
                const Vec6 a = scaled_coast(plus, j), b = scaled_coast(minus, j);
                for (int r = 0; r < 6; ++r) jac[(6 * j + r) * cols + 6 * j + k] = (a[r] - b[r]) / (2.0 * h);
            }
            for (int r = 0; r < 6; ++r) jac[(6 * j + r) * cols + 6 * (j + 1) + r] = -1.0;
        }
    }

    // dX = -J^T (J J^T)^-1 F (J J^T formed densely: <= ~300 rows).
    static void minimum_norm_step(const std::vector<double>& jac, std::vector<double> f, std::vector<double>& step) {
        const size_t rows = f.size(), cols = step.size();
        std::vector<double> jjt(rows * rows);
        for (size_t a = 0; a < rows; ++a)
            for (size_t b = 0; b <= a; ++b) {
                double s = 0.0;
                for (size_t c = 0; c < cols; ++c) s += jac[a * cols + c] * jac[b * cols + c];
                jjt[a * rows + b] = jjt[b * rows + a] = s;
            }
        if (!solve_spd(jjt, f, rows)) throw std::runtime_error("NRHO: Mehrfachschießverfahren konvergiert nicht");
        std::fill(step.begin(), step.end(), 0.0);
        for (size_t c = 0; c < cols; ++c)
            for (size_t a = 0; a < rows; ++a) step[c] -= jac[a * cols + c] * f[a];
    }

    const environment::GravityModel& gravity_;
    const std::vector<time::Epoch>& epochs_;
    const NrhoReferenceOptions& options_;
    double max_jump_m_ = 0.0;
};
}  // namespace

NrhoReference::NrhoReference(const environment::GravityModel& earth_centred_gravity,
                             const ephemeris::Ephemeris& ephemeris, const NrhoSeed& seed, time::Epoch first_perilune,
                             NrhoReferenceOptions options)
    : gravity_(earth_centred_gravity) {
    constexpr int n = patches_per_half_revolution;
    const double mu = orbit::EarthMoonRotatingFrame(ephemeris, first_perilune).mu();
    const double period_s = seed.synodic_months * constants::synodic_month_s / seed.revolutions;
    const double period = period_s / (constants::sidereal_month_s / (2.0 * constants::pi));
    const auto perilune = cr3bp_perilune(mu, seed, period);

    // Step 2: only the perilune states come from the CR3BP: the perilune passage is so sensitive that converted
    // states elsewhere miss it by thousands of km. The half revolution before the first perilune is coasted backward
    // from it.
    const auto p0 = convert(ephemeris, perilune, first_perilune);
    for (int r = n; r < 2 * n; ++r) {
        const auto t = first_perilune - (1.0 - phase_after_perilune(r)) * period_s;
        patch_epochs_.push_back(t);
        patch_states_.push_back(coast(earth_centred_gravity, p0, first_perilune, t));
    }
    patch_epochs_.push_back(first_perilune);
    patch_states_.push_back(p0);
    perilune_epochs_.push_back(first_perilune);

    // Step 3, continued revolution by revolution. The next perilune epoch is the closest approach of the coast from
    // the converged perilune before it: the free NRHO's perilune passages drift by hours from the resonant grid, and a
    // CR3BP perilune pinned to the grid leaves a jump Newton cannot close. The new revolution is seeded forward from
    // the previous perilune up to the apolune and backward from the CR3BP perilune at that epoch after it, so the jump
    // sits just after the apolune where the motion is slow and the Newton step nearly linear; then the whole span is
    // corrected again.
    // The initial jump grows with every revolution (the free orbit drifts from the CR3BP shape); when Newton cannot
    // close it, the revolution is seeded again from the free coast itself (anchor weight 0: already continuous). Partial
    // anchors were tried: a blend of two perilune states is far off both orbits (jumps of 40,000 km).
    Shooter shooter(earth_centred_gravity, patch_epochs_, options);
    std::vector<Vec6> x;
    for (int k = 1; k <= options.revolutions; ++k) {
        const size_t previous = patch_epochs_.size() - 1;
        const auto t_previous = patch_epochs_[previous];
        const auto t_next = closest_approach(earth_centred_gravity, ephemeris, patch_states_[previous], t_previous,
                                             t_previous + period_s, perilune_window_s);
        const auto anchor = convert(ephemeris, perilune, t_next);
        const auto free = coast(earth_centred_gravity, patch_states_[previous], t_previous, t_next);
        for (int r = 1; r <= 2 * n; ++r) {
            patch_epochs_.push_back(t_previous + phase_after_perilune(r) * (t_next - t_previous));
            if (r == n) apolune_epochs_.push_back(patch_epochs_.back());
        }
        perilune_epochs_.push_back(t_next);
        const std::vector<StateVector> converged = patch_states_;
        for (double weight = 1.0;; weight = 0.0) {
            const StateVector& next_state = weight > 0.0 ? anchor : free;
            patch_states_ = converged;
            for (int r = 1; r <= 2 * n; ++r) {
                const auto t = patch_epochs_[previous + r];
                patch_states_.push_back(r == 2 * n ? next_state
                                        : r <= n   ? coast(earth_centred_gravity, converged[previous], t_previous, t)
                                                   : coast(earth_centred_gravity, next_state, t_next, t));
            }
            x.clear();
            for (const auto& s : patch_states_) x.push_back(to_scaled(s));
            try {
                iterations_ += shooter.solve(x);
            } catch (const std::runtime_error&) {
                if (weight == 0.0) throw;
                if (debug_enabled()) std::fprintf(stderr, "NRHO shooting: revolution %d, CR3BP anchor failed\n", k);
                continue;
            }
            break;
        }
        for (size_t m = 0; m < x.size(); ++m) patch_states_[m] = from_scaled(x[m]);
    }
    max_continuity_error_m_ = shooter.max_jump_m();
}

bool NrhoReference::covers(time::Epoch epoch) const {
    return epoch - patch_epochs_.front() >= 0.0 && patch_epochs_.back() - epoch >= 0.0;
}

StateVector NrhoReference::state_gcrf(time::Epoch epoch) const {
    if (!covers(epoch)) throw std::out_of_range("NRHO: Epoche außerhalb der Referenzbahn");
    size_t i = patch_epochs_.size() - 1;
    while (epoch - patch_epochs_[i] < 0.0) --i;  // last patch at or before the epoch
    if (epoch - patch_epochs_[i] == 0.0) return patch_states_[i];
    return orbit::propagate_coast(gravity_, patch_states_[i], patch_epochs_[i], epoch).state;
}

}  // namespace starsim::mission
