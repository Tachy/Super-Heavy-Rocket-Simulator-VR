#pragma once

#include <array>

namespace starsim::gnc {

// Lateral dynamics of a gimbaled rocket for small angles, per horizontal axis:
//   p' = v,  v' = a theta - c u,  theta' = w,  w' = u
// p: lateral position, theta: tilt of the body axis towards that axis, u: angular acceleration command,
// a: thrust acceleration, c = I / (m L) (inertia over mass times gimbal lever): the gimbal that turns the body
// first pushes it the other way (non-minimum phase). Cascaded position/attitude loops that ignore this go
// unstable on a slow-turning vehicle; LQR state feedback on all four states does not.
struct TvcLateralWeights {
    double position_m = 1.0;          // [E] Bryson's rule: acceptable error scales
    double velocity_mps = 1.0;        // [E]
    double tilt_rad = 0.05;           // [E] ~3 deg
    double tilt_rate_radps = 0.05;    // [E]
    double angular_accel_radps2 = 0.1;  // [E] control effort
};

struct TvcLateralGains {
    std::array<double, 4> k{};  // u = -(k0 p + k1 v + k2 theta + k3 w)
};

// Discrete-time LQR (design step 0.05 s, Riccati iteration) for thrust acceleration a and c.
TvcLateralGains tvc_lateral_gains(double thrust_acceleration_mps2, double inertia_over_mass_lever_m,
                                  const TvcLateralWeights& weights);

}  // namespace starsim::gnc
