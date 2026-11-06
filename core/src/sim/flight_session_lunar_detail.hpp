// Private helpers shared by the lunar FlightSession files (flight_session_lunar*.cpp). Not part of the public API.
#pragma once

#include <cstdio>
#include <exception>
#include <optional>
#include <string>
#include <vector>

#include "starsim/config/data_missing.hpp"
#include "starsim/report/report.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::sim::detail {

inline constexpr double far_future_s = 1e9;  // "nothing timed"

// The vacuum Raptors (config group id) of the big lunar burns (TLI, insertion, transfers, TEI, braking).
inline const std::vector<std::string> vacuum_groups{"vacuum"};

// First epoch of `epochs` after t.
inline std::optional<time::Epoch> first_after(const std::vector<time::Epoch>& epochs, time::Epoch t) {
    for (const auto& e : epochs) {
        if (e - t > 0.0) return e;
    }
    return std::nullopt;
}

// A lunar step as a table-key reference (phase.lunar_step.<id>).
inline report::Param step_ref(const char* name, LunarStep step) {
    return report::ref(name, std::string("phase.lunar_step.") + id_of(step));
}

// The report an exception stands for: a refusal's own report, else the generic internal-error code (the English developer message goes to stderr, never into the shown text).
inline report::Report exception_report(const std::exception& e) {
    if (const auto* refusal = dynamic_cast<const report::Refusal*>(&e)) return refusal->report();
    if (const auto* missing = dynamic_cast<const config::DataMissing*>(&e)) return missing->report();
    std::fprintf(stderr, "internal error: %s\n", e.what());
    return report::make(report::Code::internal_error);
}

}  // namespace starsim::sim::detail
