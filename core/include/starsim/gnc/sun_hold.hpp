#pragma once

// Sun pointing during a lunar coast: held until pointed and settled, given up after a timeout (a hold that never
// settles would spend cold gas until the next re-point). The session owns the attitude mode; this only decides.

#include "starsim/math/constants.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::gnc {

struct SunHoldLimits {
    double settled_error_rad = 1.0 * constants::deg_to_rad;     // [E]
    double settled_rate_radps = 0.005 * constants::deg_to_rad;  // [E] ~100 deg drift in 6 h at most
    double timeout_s = 1800.0;                                   // [E] slews take minutes; 30 min is a stuck hold
};

enum class SunHoldResult { Holding, Settled, TimedOut };

class SunHold {
public:
    explicit SunHold(SunHoldLimits limits = {}) : limits_(limits) {}
    void start(time::Epoch epoch);
    bool active() const { return active_; }
    // Called every tick while active; Settled and TimedOut end the hold.
    SunHoldResult update(double error_rad, double rate_radps, time::Epoch epoch);

private:
    SunHoldLimits limits_;
    time::Epoch started_{};
    bool active_ = false;
};

}  // namespace starsim::gnc
