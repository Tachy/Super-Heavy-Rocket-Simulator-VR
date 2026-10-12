#include "starsim/contact/catch_trigger.hpp"

namespace starsim::contact {

bool should_close_arms(const CatchTrigger& trigger, double pin_height_above_arms_m, double axis_offset_m,
                       double sink_rate_mps, double closing_time_s) {
    if (axis_offset_m > trigger.max_axis_offset_m || pin_height_above_arms_m < 0.0) return false;
    if (pin_height_above_arms_m <= trigger.hover_height_m) return true;
    return sink_rate_mps > 0.0 && pin_height_above_arms_m / sink_rate_mps <= closing_time_s + trigger.margin_s;
}

}  // namespace starsim::contact
