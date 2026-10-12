#pragma once

namespace starsim::contact {

// Automatic tower: close the arms when the pins will reach the arm tops within the closing time (plus a
// margin) and the booster axis is near the catch axis, or when the pins hover just above the arms.
struct CatchTrigger {
    double max_axis_offset_m = 3.0;  // [E] farther off the arms cannot catch the pins
    double margin_s = 1.0;           // [E] arms closed this long before the pins arrive
    double hover_height_m = 2.0;     // [E]
};

bool should_close_arms(const CatchTrigger& trigger, double pin_height_above_arms_m, double axis_offset_m,
                       double sink_rate_mps, double closing_time_s);

}  // namespace starsim::contact
