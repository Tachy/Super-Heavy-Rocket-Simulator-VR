// Webcast agreement as a report, not a gate (user 2026-11-03): rows outside the tolerance print a WARN line and land in
// ${STARSIM_OUTPUT_DIR}/webcast_report/<case_id>.md. Never fails a test; the physical gates live in the normal-tier cases.
#pragma once

#include <string>
#include <vector>

namespace starsim::test {

struct WebcastRow {
    std::string quantity;
    double ours = 0.0;
    double reference = 0.0;
    double tolerance = 0.0;  // |ours - reference| <= tolerance is in agreement
    std::string reference_source;
    std::string parameters;  // the vehicle parameters this quantity depends on
};

// case_id names the file (one per case, so parallel ctest processes never share one).
void webcast_report(const std::string& case_id, const std::vector<WebcastRow>& rows);

}  // namespace starsim::test
