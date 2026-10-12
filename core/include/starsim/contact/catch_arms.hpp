#pragma once

#include "starsim/config/launch_site_config.hpp"

namespace starsim::contact {

// The two catch arms (moving together) on their carriage. Opening angle: rate limited towards the command;
// closing stalls while the hull pushes back harder than the actuators allow. Carriage: mass on shock
// absorbers about its rest position (its weight is carried by the spring preload), with a hard stop at the
// end of the stroke.
class CatchArms {
public:
    explicit CatchArms(const config::TowerConfig& config);

    void command_close(bool close) { close_ = close; }
    bool close_commanded() const { return close_; }
    void set_closed();  // instant (scenario setup)
    // `hull_push_n`: force of the body against the closing arms; `vertical_load_n`: downward force of the
    // body on the arms.
    void step(double dt_s, double hull_push_n, double vertical_load_n);

    double opening_rad() const { return opening_rad_; }
    double opening_rate_rad_s() const { return opening_rate_rad_s_; }
    double carriage_offset_m() const { return carriage_offset_m_; }  // negative: below rest
    double carriage_velocity_mps() const { return carriage_velocity_mps_; }
    bool closed() const { return opening_rad_ <= 0.0; }
    bool stalled() const { return stalled_; }
    bool hit_hard_stop() const { return hit_hard_stop_; }  // reached the stop faster than it can take
    double closing_time_s() const { return config_.arm_open_rad / config_.arm_rate_rad_s; }

private:
    config::TowerConfig config_;
    bool close_ = false;
    double opening_rad_ = 0.0;
    double opening_rate_rad_s_ = 0.0;
    bool stalled_ = false;
    double carriage_offset_m_ = 0.0;
    double carriage_velocity_mps_ = 0.0;
    bool hit_hard_stop_ = false;
};

}  // namespace starsim::contact
