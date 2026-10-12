#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "starsim/integrators/dop853_tableau.hpp"
#include "starsim/integrators/ode.hpp"

namespace starsim::integrators {

template <std::size_t N>
struct Dop853Options {
    double relative_tolerance = 1e-12;
    State<N> absolute_tolerance = State<N>::filled(1e-9);
    double max_step_s = std::numeric_limits<double>::infinity();
    double first_step_s = 0.0;  // 0 = automatic (Hairer's heuristic)
};

// Adaptive Dormand-Prince 8(5,3) integrator with 7th-order dense output.
// Step-size control and error norm follow Hairer et al. / SciPy's DOP853 implementation.
// Integrates forward or backward (t_bound < t0).
template <std::size_t N>
class Dop853 {
public:
    Dop853(OdeFunction<N> f, double t0_s, const State<N>& y0, double t_bound_s, Dop853Options<N> options = {})
        : f_(std::move(f)), options_(std::move(options)), t_(t0_s), t_old_(t0_s), t_bound_(t_bound_s), y_(y0),
          y_old_(y0), direction_(t_bound_s >= t0_s ? 1.0 : -1.0) {
        f_value_ = evaluate(t_, y_);
        h_abs_ = options_.first_step_s > 0.0 ? options_.first_step_s : initial_step();
    }

    bool finished() const { return t_ == t_bound_; }
    double t() const { return t_; }
    double t_previous() const { return t_old_; }
    const State<N>& y() const { return y_; }
    const State<N>& y_previous() const { return y_old_; }
    long evaluations() const { return evaluations_; }
    long accepted_steps() const { return accepted_; }
    long rejected_steps() const { return rejected_; }

    // Performs one accepted step towards t_bound. Throws if the step size underflows.
    void step() {
        if (finished()) {
            return;
        }
        const double min_step = 10.0 * std::abs(std::nextafter(t_, direction_ * std::numeric_limits<double>::infinity()) - t_);
        double h_abs = std::clamp(h_abs_, min_step, options_.max_step_s);
        bool step_rejected = false;

        while (true) {
            if (h_abs < min_step) {
                throw std::runtime_error("DOP853: step size underflow");
            }
            double t_new = t_ + direction_ * h_abs;
            if (direction_ * (t_new - t_bound_) > 0.0) {
                t_new = t_bound_;
            }
            const double h = t_new - t_;
            h_abs = std::abs(h);

            const State<N> y_new = rk_step(h);
            const double error = error_norm(h, y_new);

            if (error < 1.0) {
                double factor = error == 0.0 ? max_factor : std::min(max_factor, safety * std::pow(error, -1.0 / 8.0));
                if (step_rejected) {
                    factor = std::min(1.0, factor);
                }
                h_abs_ = h_abs * factor;
                t_old_ = t_;
                y_old_ = y_;
                f_old_ = f_value_;
                h_previous_ = h;
                t_ = t_new;
                y_ = y_new;
                f_value_ = k_[n_main];
                dense_ready_ = false;
                ++accepted_;
                return;
            }
            h_abs *= std::max(min_factor, safety * std::pow(error, -1.0 / 8.0));
            step_rejected = true;
            ++rejected_;
        }
    }

    // Interpolated state inside the last accepted step [t_previous, t] (7th order).
    State<N> dense_output(double t_s) const {
        if (!dense_ready_) {
            prepare_dense_output();
        }
        const double x = (t_s - t_old_) / h_previous_;
        State<N> y{};
        for (int i = n_interpolant - 1, count = 0; i >= 0; --i, ++count) {
            y += interpolant_[i];
            y *= (count % 2 == 0) ? x : (1.0 - x);
        }
        return y + y_old_;
    }

private:
    static constexpr int n_main = dop853::n_stages;            // 12 stages + FSAL slot
    static constexpr int n_interpolant = 3 + dop853::n_dense;  // F0..F6
    static constexpr double safety = 0.9;
    static constexpr double min_factor = 0.2;
    static constexpr double max_factor = 10.0;

    State<N> evaluate(double t_s, const State<N>& y) const {
        ++evaluations_;
        return f_(t_s, y);
    }

    State<N> rk_step(double h) {
        k_[0] = f_value_;
        for (int s = 1; s < n_main; ++s) {
            State<N> y_stage = y_;
            for (int j = 0; j < s; ++j) {
                if (dop853::a[s][j] != 0.0) {
                    y_stage.add_scaled(h * dop853::a[s][j], k_[j]);
                }
            }
            k_[s] = evaluate(t_ + dop853::c[s] * h, y_stage);
        }
        State<N> y_new = y_;
        for (int j = 0; j < n_main; ++j) {
            if (dop853::b[j] != 0.0) {
                y_new.add_scaled(h * dop853::b[j], k_[j]);
            }
        }
        k_[n_main] = evaluate(t_ + h, y_new);
        return y_new;
    }

    double error_norm(double h, const State<N>& y_new) const {
        double err5_sq = 0.0;
        double err3_sq = 0.0;
        for (std::size_t i = 0; i < N; ++i) {
            const double scale = options_.absolute_tolerance[i] +
                                 std::max(std::abs(y_[i]), std::abs(y_new[i])) * options_.relative_tolerance;
            double e5 = 0.0;
            double e3 = 0.0;
            for (int j = 0; j <= n_main; ++j) {
                e5 += k_[j][i] * dop853::e5[j];
                e3 += k_[j][i] * dop853::e3[j];
            }
            err5_sq += (e5 / scale) * (e5 / scale);
            err3_sq += (e3 / scale) * (e3 / scale);
        }
        if (err5_sq == 0.0 && err3_sq == 0.0) {
            return 0.0;
        }
        const double denominator = err5_sq + 0.01 * err3_sq;
        return std::abs(h) * err5_sq / std::sqrt(denominator * static_cast<double>(N));
    }

    double initial_step() const {
        const State<N> f0 = f_value_;
        auto rms_scaled = [&](const State<N>& v) {
            double sum = 0.0;
            for (std::size_t i = 0; i < N; ++i) {
                const double scale = options_.absolute_tolerance[i] + std::abs(y_[i]) * options_.relative_tolerance;
                sum += (v[i] / scale) * (v[i] / scale);
            }
            return std::sqrt(sum / static_cast<double>(N));
        };
        const double d0 = rms_scaled(y_);
        const double d1 = rms_scaled(f0);
        const double h0 = (d0 < 1e-5 || d1 < 1e-5) ? 1e-6 : 0.01 * d0 / d1;
        const State<N> y1 = State<N>(y_).add_scaled(direction_ * h0, f0);
        const State<N> f1 = evaluate(t_ + direction_ * h0, y1);
        const double d2 = rms_scaled(f1 - f0) / h0;
        const double h1 = std::max(d1, d2) <= 1e-15 ? std::max(1e-6, h0 * 1e-3)
                                                     : std::pow(0.01 / std::max(d1, d2), 1.0 / 8.0);
        return std::min({100.0 * h0, h1, options_.max_step_s});
    }

    void prepare_dense_output() const {
        const double h = h_previous_;
        // The step's stages k_[0..12] are still valid; add the three extra stages.
        for (int s = n_main + 1; s < dop853::n_stages_extended; ++s) {
            State<N> y_stage = y_old_;
            for (int j = 0; j < s; ++j) {
                if (dop853::a[s][j] != 0.0) {
                    y_stage.add_scaled(h * dop853::a[s][j], k_[j]);
                }
            }
            k_[s] = evaluate(t_old_ + dop853::c[s] * h, y_stage);
        }
        const State<N> delta_y = y_ - y_old_;
        interpolant_[0] = delta_y;
        interpolant_[1] = h * f_old_ - delta_y;
        interpolant_[2] = 2.0 * delta_y - h * (f_value_ + f_old_);
        for (int i = 0; i < dop853::n_dense; ++i) {
            State<N> sum{};
            for (int j = 0; j < dop853::n_stages_extended; ++j) {
                if (dop853::d[i][j] != 0.0) {
                    sum.add_scaled(dop853::d[i][j], k_[j]);
                }
            }
            interpolant_[3 + i] = h * sum;
        }
        dense_ready_ = true;
    }

    OdeFunction<N> f_;
    Dop853Options<N> options_;
    double t_;
    double t_old_;
    double t_bound_;
    State<N> y_;
    State<N> y_old_;
    State<N> f_value_{};
    State<N> f_old_{};
    double direction_;
    double h_abs_ = 0.0;
    double h_previous_ = 0.0;

    mutable std::array<State<N>, dop853::n_stages_extended> k_{};
    mutable std::array<State<N>, n_interpolant> interpolant_{};
    mutable bool dense_ready_ = false;
    mutable long evaluations_ = 0;
    long accepted_ = 0;
    long rejected_ = 0;
};

}  // namespace starsim::integrators
