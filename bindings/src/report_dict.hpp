#pragma once

#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include <string>

#include "starsim/report/report.hpp"

namespace starsim::bindings {

// {"code": String, "params": {name: float | String}, "kinds": {name: String}, "causes": Array of reports}.
// ok is {"code": "ok", "params": {}, "kinds": {}, "causes": []}. No text is composed here; the game resolves the code.
godot::Dictionary report_dict(const report::Report& report);
godot::Array report_array(const report::Reports& reports);

// Transitional (removed with the last conversion task): a core string that is not a report yet; empty -> ok.
godot::Dictionary legacy_report(const std::string& text);

}  // namespace starsim::bindings
