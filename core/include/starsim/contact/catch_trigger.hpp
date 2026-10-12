#pragma once

#include "starsim/contact/catch_arms.hpp"

namespace starsim::contact {

// Automatic tower: the arms wait half open (standby) while the booster comes down between them and close only
// just before the pins arrive - from standby within a fraction of a second - if the booster axis is near the
// catch axis; climbing or below the arms: open.
struct CatchTrigger {
    double standby_lead_s = 10.0;    // [E] arms to standby this long before the pins arrive
    double max_axis_offset_m = 2.0;  // [E] final closing only this close to the catch axis (standby: ~3 m clear)
    double margin_s = 0.5;           // [E] arms closed this long before the pins arrive
    double hover_height_m = 2.0;     // [E]
};

ArmCommand arm_command(const CatchTrigger& trigger, double pin_height_above_arms_m, double axis_offset_m,
                       double sink_rate_mps, double final_closing_time_s);

}  // namespace starsim::contact
