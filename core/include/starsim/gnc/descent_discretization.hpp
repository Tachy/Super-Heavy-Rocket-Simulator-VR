#pragma once

#include <array>

#include "starsim/gnc/descent_model.hpp"

namespace starsim::gnc {

// One interval of the first-order-hold discretisation and its sensitivities: for neighbouring trajectories
// x(end) ~ propagated + phi dx0 + gamma_minus du0 + gamma_plus du1 + sigma ds.
struct IntervalLinearization {
    Mat14x14 phi{};
    Mat14x3 gamma_minus{};
    Mat14x3 gamma_plus{};
    std::array<double, 14> sigma{};
    DescentState propagated;
};

// Propagates an interval of normalised length `dtau` of a phase lasting `phase_duration_s` with the thrust
// changing linearly from u0 to u1 (RK4, `substeps` steps), integrating the variational equations along.
IntervalLinearization linearize_interval(const DescentModel& model, int phase, const DescentState& x0,
                                         const math::Vec3& u0, const math::Vec3& u1, double phase_duration_s,
                                         double dtau, int substeps);

}  // namespace starsim::gnc
