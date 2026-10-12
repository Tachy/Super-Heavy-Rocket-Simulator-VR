#include "starsim/contact/catch_arms.hpp"

#include <algorithm>

namespace starsim::contact {

namespace {
constexpr double hard_stop_speed_mps = 1.0;  // [E] faster at the end of the stroke: structural failure
}

CatchArms::CatchArms(const config::TowerConfig& config) : config_(config), opening_rad_(config.arm_open_rad) {}

void CatchArms::set_closed() {
    close_ = true;
    opening_rad_ = 0.0;
    opening_rate_rad_s_ = 0.0;
}

void CatchArms::step(double dt_s, double hull_push_n, double vertical_load_n) {
    const double previous = opening_rad_;
    const double target = close_ ? 0.0 : config_.arm_open_rad;
    stalled_ = close_ && hull_push_n > config_.arm_force_limit_n;
    if (!stalled_) {
        const double max_change = config_.arm_rate_rad_s * dt_s;
        opening_rad_ += std::clamp(target - opening_rad_, -max_change, max_change);
    }
    opening_rate_rad_s_ = (opening_rad_ - previous) / dt_s;

    // Semi-implicit Euler (stable for the stiff absorbers at 200 Hz).
    const double acceleration = (-config_.carriage_stiffness_n_m * carriage_offset_m_ -
                                 config_.carriage_damping_n_s_m * carriage_velocity_mps_ - vertical_load_n) /
                                config_.carriage_mass_kg;
    carriage_velocity_mps_ += acceleration * dt_s;
    carriage_offset_m_ += carriage_velocity_mps_ * dt_s;
    if (carriage_offset_m_ < -config_.carriage_stroke_m) {
        if (carriage_velocity_mps_ < -hard_stop_speed_mps) hit_hard_stop_ = true;
        carriage_offset_m_ = -config_.carriage_stroke_m;
        carriage_velocity_mps_ = std::max(carriage_velocity_mps_, 0.0);
    } else if (carriage_offset_m_ > config_.carriage_stroke_m) {
        carriage_offset_m_ = config_.carriage_stroke_m;
        carriage_velocity_mps_ = std::min(carriage_velocity_mps_, 0.0);
    }
}

}  // namespace starsim::contact
