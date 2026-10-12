#include "starsim/contact/catch_trigger.hpp"

namespace starsim::contact {

ArmCommand arm_command(const CatchTrigger& trigger, double pin_height_above_arms_m, double axis_offset_m,
                       double sink_rate_mps, double final_closing_time_s) {
    if (pin_height_above_arms_m < 0.0) return ArmCommand::Open;
    const bool hovering = pin_height_above_arms_m <= trigger.hover_height_m;
    if (!hovering && sink_rate_mps <= 0.0) return ArmCommand::Open;
    const double time_to_contact = hovering ? 0.0 : pin_height_above_arms_m / sink_rate_mps;
    if (time_to_contact > trigger.standby_lead_s) return ArmCommand::Open;
    const bool close = time_to_contact <= final_closing_time_s + trigger.margin_s && axis_offset_m <= trigger.max_axis_offset_m;
    return close ? ArmCommand::Closed : ArmCommand::Standby;
}

}  // namespace starsim::contact
