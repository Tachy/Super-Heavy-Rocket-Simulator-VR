#pragma once

#include <vector>

namespace starsim::propulsion {

// On/off pulses of fixed-thrust thrusters from duties (sigma-delta per thruster, whole ticks): the duty is accumulated
// as owed on-time; a pulse starts once what is owed plus what the duty adds during it covers a minimum on-time,
// and lasts at least that long; it continues
// while at least half a tick is owed. A duty of zero clears what is owed. Not modelled: thrust rise / tail-off within
// a pulse and the Isp loss of short pulses.
class RcsModulator {
public:
    RcsModulator() = default;
    RcsModulator(size_t thrusters, double minimum_on_time_s);

    // Firing flags for this tick.
    const std::vector<bool>& update(const std::vector<double>& duty, double dt_s);
    const std::vector<bool>& firing() const { return firing_; }

private:
    double minimum_on_time_s_ = 0.0;
    std::vector<double> owed_s_;
    std::vector<int> pulse_ticks_left_;
    std::vector<bool> firing_;
};

}  // namespace starsim::propulsion
