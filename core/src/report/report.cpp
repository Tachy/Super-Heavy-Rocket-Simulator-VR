#include "starsim/report/report.hpp"

#include <array>
#include <utility>

namespace starsim::report {

namespace {

constexpr CodeInfo kTable[] = {
#define STARSIM_CODE(name, id, params) {Code::name, #name, id, params},
#include "starsim/report/codes.def"
#undef STARSIM_CODE
};

Param make_param(const char* name, Kind kind, double value, std::string text = {}) {
    return Param{name, kind, value, std::move(text)};
}

}  // namespace

Report make(Code code, std::initializer_list<Param> params) {
    Report report;
    report.code = code;
    report.params.assign(params.begin(), params.end());
    return report;
}

Report with_cause(Report report, Report cause) {
    report.causes.push_back(std::move(cause));
    return report;
}

bool same(const Report& a, const Report& b) {
    if (a.code != b.code || a.params.size() != b.params.size() || a.causes.size() != b.causes.size()) return false;
    for (size_t i = 0; i < a.params.size(); ++i) {
        const Param& p = a.params[i];
        const Param& q = b.params[i];
        if (std::string_view(p.name) != std::string_view(q.name) || p.kind != q.kind || p.text != q.text) return false;
        if (!(p.value_si == q.value_si)) return false;
    }
    for (size_t i = 0; i < a.causes.size(); ++i) {
        if (!same(a.causes[i], b.causes[i])) return false;
    }
    return true;
}

Param angle(const char* name, double rad) { return make_param(name, Kind::angle, rad); }
Param mass(const char* name, double kg) { return make_param(name, Kind::mass, kg); }
Param length(const char* name, double m) { return make_param(name, Kind::length, m); }
Param speed(const char* name, double mps) { return make_param(name, Kind::speed, mps); }
Param time(const char* name, double s) { return make_param(name, Kind::time, s); }
Param count(const char* name, long long value) {
    return make_param(name, Kind::count, static_cast<double>(value));
}
Param number(const char* name, double value) { return make_param(name, Kind::number, value); }
Param ref(const char* name, std::string key) {
    return make_param(name, Kind::ref, 0.0, std::move(key));
}
Param legacy_text(const char* name, std::string text) {
    return make_param(name, Kind::legacy_text, 0.0, std::move(text));
}

Param utc(const char* name, double unix_s) { return make_param(name, Kind::utc, unix_s); }
Param data_name(const char* name, std::string id) {
    return make_param(name, Kind::data_name, 0.0, std::move(id));
}

std::span<const CodeInfo> code_table() { return std::span<const CodeInfo>(kTable); }

std::string_view id_of(Code code) {
    for (const CodeInfo& info : kTable) {
        if (info.code == code) return info.id;
    }
    return "ok";
}

std::optional<Code> code_from_id(std::string_view id) {
    for (const CodeInfo& info : kTable) {
        if (info.id == id) return info.code;
    }
    return std::nullopt;
}

const char* id_of(Kind kind) {
    switch (kind) {
        case Kind::angle: return "angle";
        case Kind::mass: return "mass";
        case Kind::length: return "length";
        case Kind::speed: return "speed";
        case Kind::time: return "time";
        case Kind::count: return "count";
        case Kind::number: return "number";
        case Kind::ref: return "ref";
        case Kind::legacy_text: return "legacy_text";
        case Kind::utc: return "utc";
        case Kind::data_name: return "data_name";
    }
    return "number";
}

Report legacy(std::string text) { return make(Code::legacy, {legacy_text("text", std::move(text))}); }

std::string transitional_text(const Report& report) {
    if (report.code == Code::legacy && !report.params.empty()) return report.params.front().text;
    std::string out(id_of(report.code));
    if (report.params.empty() && report.causes.empty()) return out;
    out += '(';
    bool first = true;
    for (const Param& p : report.params) {
        if (!first) out += ", ";
        first = false;
        out += p.name;
        out += '=';
        out += (p.kind == Kind::ref || p.kind == Kind::legacy_text || p.kind == Kind::data_name) ? p.text : std::to_string(p.value_si);
    }
    for (const Report& c : report.causes) {
        if (!first) out += ", ";
        first = false;
        out += "cause=" + transitional_text(c);
    }
    return out + ')';
}

Refusal::Refusal(Report report)
    : std::runtime_error(std::string(id_of(report.code))), report_(std::move(report)) {}

}  // namespace starsim::report
