#include "starsim/propulsion/rcs_modulator.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::propulsion {

RcsModulator::RcsModulator(size_t thrusters, double minimum_on_time_s)
    : minimum_on_time_s_(minimum_on_time_s), owed_s_(thrusters, 0.0), pulse_ticks_left_(thrusters, 0),
      firing_(thrusters, false) {}

const std::vector<bool>& RcsModulator::update(const std::vector<double>& duty, double dt_s) {
    const int pulse_ticks = std::max(1, static_cast<int>(std::lround(minimum_on_time_s_ / dt_s)));
    for (size_t i = 0; i < firing_.size(); ++i) {
        const double d = i < duty.size() ? std::clamp(duty[i], 0.0, 1.0) : 0.0;
        if (d <= 0.0) {
            owed_s_[i] = 0.0;
            pulse_ticks_left_[i] = 0;
            firing_[i] = false;
            continue;
        }
        // Anti-windup: at most one pulse plus a tick owed (a duty of 1 then fires every tick).
        owed_s_[i] = std::min(owed_s_[i] + d * dt_s, minimum_on_time_s_ + dt_s);
        bool fire = false;
        if (pulse_ticks_left_[i] > 0) {
            fire = true;
            --pulse_ticks_left_[i];
        } else if (owed_s_[i] + d * (minimum_on_time_s_ - dt_s) >= minimum_on_time_s_ - 0.5 * dt_s) {
            // What is owed plus what the duty adds during the pulse covers it (a full demand fires at once).
            fire = true;
            pulse_ticks_left_[i] = pulse_ticks - 1;
        } else if (firing_[i] && owed_s_[i] >= 0.5 * dt_s) {
            fire = true;
        }
        if (fire) owed_s_[i] -= dt_s;
        firing_[i] = fire;
    }
    return firing_;
}

}  // namespace starsim::propulsion
