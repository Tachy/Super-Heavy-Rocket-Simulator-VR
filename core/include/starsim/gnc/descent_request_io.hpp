#pragma once

#include <string>

#include "starsim/gnc/descent_replanner.hpp"

namespace starsim::gnc {

// Text file of a landing-burn plan request (model tables, start state, target, settings, warm start): replays one
// re-plan from a flight outside the simulation (diagnostics, planner fixtures). One "name value..." line per field,
// doubles at full precision; reading keeps the defaults of fields the file does not name (older files stay readable
// when fields are added). The snapshot epoch is not stored.
void write_plan_request(const std::string& path, const PlanRequest& request);
PlanRequest read_plan_request(const std::string& path);  // std::runtime_error if unreadable

// Diagnostics: with STARSIM_DUMP_SCVX=<directory> every plan request is written there as scvx_request_<n>.txt
// (n counts the requests of the process); returns the file name, empty when not dumping.
std::string dump_plan_request_if_enabled(const PlanRequest& request);

}  // namespace starsim::gnc
