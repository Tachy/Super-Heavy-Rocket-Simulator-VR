#pragma once
// Small helpers to test outcome reports by code (the core carries no text).
#include <string>

#include "starsim/report/report.hpp"

namespace starsim::test {

// The report's code id (for INFO / printf output).
inline std::string code_id(const report::Report& r) { return std::string(report::id_of(r.code)); }

// The report's first cause has the code `code`.
inline bool cause_is(const report::Report& r, report::Code code) {
    return !r.causes.empty() && r.causes.front().code == code;
}

// A refusal `outer` whose first cause is `cause`.
inline bool refused(const report::Report& r, report::Code outer, report::Code cause) {
    return r.code == outer && cause_is(r, cause);
}

// A proximity status that names waypoint `number` (a "Wegpunkt" or, with flyaround, an "Umflug" name) as its cause.
inline bool names_waypoint(const report::Report& r, int number, bool flyaround = false) {
    if (r.causes.empty()) return false;
    const report::Report& name = r.causes.front();
    const auto expected = flyaround ? report::Code::proximity_flyaround_name : report::Code::proximity_waypoint_name;
    return name.code == expected && !name.params.empty() && name.params.front().value_si == number;
}

}  // namespace starsim::test
