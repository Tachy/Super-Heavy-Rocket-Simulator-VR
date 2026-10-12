#pragma once

#include <cstddef>
#include <functional>

#include "starsim/math/vec_n.hpp"

namespace starsim::integrators {

template <std::size_t N>
using State = math::VecN<N>;

// Right-hand side dy/dt = f(t, y). t is in seconds relative to the integration's reference epoch,
// which keeps the time argument small and precise.
template <std::size_t N>
using OdeFunction = std::function<State<N>(double t_s, const State<N>& y)>;

}  // namespace starsim::integrators
