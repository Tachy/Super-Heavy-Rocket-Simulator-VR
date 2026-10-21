#pragma once

// Refuelling r6: the refuelling_report command line - levers from flags, the report as text.

#include <string>
#include <vector>

#include "starsim/sim/refuelling_campaign_flight.hpp"
#include "starsim/sim/refuelling_campaign_setup.hpp"
#include "starsim/time/time_scale_converter.hpp"

namespace starsim::sim {

// Command-line levers: --height-km, --interval-h, --phasing-limit-h, --tanker-dry-t, --reserve-t, --initial-t,
// --insulation, --hls-wait-d (each followed by a number). Throws std::invalid_argument naming the flag on unknown
// flags, a missing value, a malformed number or a value out of range (> 0; --reserve-t, --hls-wait-d >= 0).
CampaignLevers campaign_levers_from_args(const std::vector<std::string>& args);

// The report as text: inputs, the per-tanker table (launch / arrival in days and UTC, delivered, depot after), the
// losses and the answer line "Tanker flights: N" - or the reason when the campaign does not close.
std::string campaign_report_text(const CampaignReport& report, const time::TimeScaleConverter& time);

// Refuelling f2 (refuelling_report --fly / --fly-placed): the flown campaign as text - one row per flight (launch,
// arrival, phasing orbit, delivered, depot after: flown and the calculator's), the losses, the reboosts, the first
// tanker's return, then "Comparison: passed" or "Comparison: FAILED" with the misses (comparison_misses).
std::string campaign_flight_text(const CampaignFlightState& state, const std::vector<std::string>& misses,
                                 const time::TimeScaleConverter& time);

}  // namespace starsim::sim
