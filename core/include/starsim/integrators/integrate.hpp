#pragma once

#include <cmath>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include "starsim/integrators/dop853.hpp"

namespace starsim::integrators {

enum class Crossing { Any, Rising, Falling };

// A terminal event: integration stops where g(t, y) crosses zero in the requested direction
// (Rising = from negative to positive). Examples: impact (altitude), periapsis (r.v), SOI entry.
template <std::size_t N>
struct Event {
    std::string name;
    std::function<double(double t_s, const State<N>& y)> g;
    Crossing crossing = Crossing::Any;
};

template <std::size_t N>
struct IntegrationResult {
    double t_s = 0.0;
    State<N> y{};
    int event_index = -1;  // index into the events span, -1 if t_end was reached
    long accepted_steps = 0;
    long rejected_steps = 0;
    long evaluations = 0;

    bool event_triggered() const { return event_index >= 0; }
};

// Called after each accepted step; dense_output() of the solver is valid in [t_previous, t].
template <std::size_t N>
using StepObserver = std::function<void(const Dop853<N>& solver)>;

namespace detail {

inline bool crossed(double g_old, double g_new, Crossing crossing) {
    const bool rising = g_old < 0.0 && g_new >= 0.0;
    const bool falling = g_old > 0.0 && g_new <= 0.0;
    switch (crossing) {
        case Crossing::Rising: return rising;
        case Crossing::Falling: return falling;
        case Crossing::Any: return rising || falling;
    }
    return false;
}

}  // namespace detail

// Integrates from (t0, y0) to t_end with DOP853, stopping at the first terminal event.
// Event times are located on the dense output to `event_tolerance_s`.
template <std::size_t N>
IntegrationResult<N> integrate(const OdeFunction<N>& f, double t0_s, const State<N>& y0, double t_end_s,
                               const Dop853Options<N>& options, std::span<const Event<N>> events = {},
                               const StepObserver<N>& observer = {}, double event_tolerance_s = 1e-9) {
    Dop853<N> solver(f, t0_s, y0, t_end_s, options);
    std::vector<double> g_old(events.size());
    for (std::size_t i = 0; i < events.size(); ++i) {
        g_old[i] = events[i].g(t0_s, y0);
    }

    auto finish = [&](double t, const State<N>& y, int index) {
        return IntegrationResult<N>{t, y, index, solver.accepted_steps(), solver.rejected_steps(),
                                    solver.evaluations()};
    };

    while (!solver.finished()) {
        solver.step();
        if (observer) {
            observer(solver);
        }
        // Find the earliest event crossing inside this step.
        int first_index = -1;
        double first_t = solver.t();
        for (std::size_t i = 0; i < events.size(); ++i) {
            const double g_new = events[i].g(solver.t(), solver.y());
            if (detail::crossed(g_old[i], g_new, events[i].crossing)) {
                // Bisection on the dense output; g_old and g_new bracket the root.
                double lo = solver.t_previous();
                double hi = solver.t();
                const double g_lo_sign = g_old[i];
                while (std::abs(hi - lo) > event_tolerance_s) {
                    const double mid = 0.5 * (lo + hi);
                    const double g_mid = events[i].g(mid, solver.dense_output(mid));
                    if ((g_mid < 0.0) == (g_lo_sign < 0.0) && g_mid != 0.0) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                const double t_root = hi;
                const bool earlier = (solver.t() >= solver.t_previous()) ? t_root < first_t : t_root > first_t;
                if (first_index < 0 || earlier) {
                    first_index = static_cast<int>(i);
                    first_t = t_root;
                }
            }
            g_old[i] = g_new;
        }
        if (first_index >= 0) {
            return finish(first_t, solver.dense_output(first_t), first_index);
        }
    }
    return finish(solver.t(), solver.y(), -1);
}

}  // namespace starsim::integrators
