#include "starsim/gnc/sun_hold.hpp"

namespace starsim::gnc {

void SunHold::start(time::Epoch epoch) {
    started_ = epoch;
    active_ = true;
}

SunHoldResult SunHold::update(double error_rad, double rate_radps, time::Epoch epoch) {
    if (!active_) return SunHoldResult::Settled;
    if (error_rad < limits_.settled_error_rad && rate_radps < limits_.settled_rate_radps) {
        active_ = false;
        return SunHoldResult::Settled;
    }
    if (epoch - started_ >= limits_.timeout_s) {
        active_ = false;
        return SunHoldResult::TimedOut;
    }
    return SunHoldResult::Holding;
}

}  // namespace starsim::gnc
