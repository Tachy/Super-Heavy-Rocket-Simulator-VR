#pragma once

#include "starsim/integrators/ode.hpp"

namespace starsim::integrators {

// Classical 4th-order Runge-Kutta step with fixed step size h.
// Used for powered/atmospheric flight with pilot input at a fixed simulation tick.
template <std::size_t N>
State<N> rk4_step(const OdeFunction<N>& f, double t_s, const State<N>& y, double h_s) {
    const State<N> k1 = f(t_s, y);
    const State<N> k2 = f(t_s + 0.5 * h_s, State<N>(y).add_scaled(0.5 * h_s, k1));
    const State<N> k3 = f(t_s + 0.5 * h_s, State<N>(y).add_scaled(0.5 * h_s, k2));
    const State<N> k4 = f(t_s + h_s, State<N>(y).add_scaled(h_s, k3));
    State<N> result = y;
    result.add_scaled(h_s / 6.0, k1).add_scaled(h_s / 3.0, k2).add_scaled(h_s / 3.0, k3).add_scaled(h_s / 6.0, k4);
    return result;
}

}  // namespace starsim::integrators
