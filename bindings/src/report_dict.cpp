#include "report_dict.hpp"

#include <godot_cpp/variant/string.hpp>

namespace starsim::bindings {

namespace {

godot::String text_of(std::string_view s) { return godot::String::utf8(s.data(), static_cast<int64_t>(s.size())); }

}  // namespace

godot::Dictionary report_dict(const report::Report& report) {
    godot::Dictionary params;
    godot::Dictionary kinds;
    for (const auto& p : report.params) {
        const godot::String name = godot::String::utf8(p.name);
        if (p.kind == report::Kind::ref || p.kind == report::Kind::legacy_text ||
            p.kind == report::Kind::data_name) {
            params[name] = godot::String::utf8(p.text.c_str());
        } else {
            params[name] = p.value_si;
        }
        kinds[name] = godot::String::utf8(report::id_of(p.kind));
    }
    godot::Dictionary d;
    d["code"] = text_of(report::id_of(report.code));
    d["params"] = params;
    d["kinds"] = kinds;
    d["causes"] = report_array(report.causes);
    return d;
}

godot::Array report_array(const report::Reports& reports) {
    godot::Array a;
    for (const auto& r : reports) a.push_back(report_dict(r));
    return a;
}

godot::String qualified_id(const char* ns, const char* id) {
    return godot::String::utf8(ns) + "." + godot::String::utf8(id);
}

godot::Dictionary legacy_report(const std::string& text) {
    if (text.empty()) return report_dict(report::Report{});
    return report_dict(report::make(report::Code::legacy, {report::legacy_text("text", text)}));
}

}  // namespace starsim::bindings
