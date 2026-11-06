#pragma once

#include <cstdint>
#include <initializer_list>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

// Outcome reports: a code plus parameters in SI units. The core never produces UI text; the game
// resolves codes through config/i18n/<lang>.toml.
namespace starsim::report {

enum class Code : std::uint16_t {
#define STARSIM_CODE(name, id, params) name,
#include "starsim/report/codes.def"
#undef STARSIM_CODE
};

// Physical kind of a parameter value (all values are SI: rad, kg, m, m/s, s).
enum class Kind : std::uint8_t { angle, mass, length, speed, time, count, number, ref, legacy_text, utc, data_name };

struct Param {
    const char* name;
    Kind kind;
    double value_si = 0.0;
    std::string text;  // ref: table key; legacy_text: transitional text; data_name: verbatim data identifier
};

struct Report {
    Code code = Code::ok;
    std::vector<Param> params;
    std::vector<Report> causes;

    bool ok() const { return code == Code::ok; }
};

using Reports = std::vector<Report>;

Report make(Code code, std::initializer_list<Param> params = {});
Report with_cause(Report report, Report cause);
// Same code, same parameters (name, kind, value, text) and same causes.
bool same(const Report& a, const Report& b);

Param angle(const char* name, double rad);
Param mass(const char* name, double kg);
Param length(const char* name, double m);
Param speed(const char* name, double mps);
Param time(const char* name, double s);
Param count(const char* name, long long value);
Param number(const char* name, double value);
Param ref(const char* name, std::string key);
Param legacy_text(const char* name, std::string text);
// UTC seconds since the Unix epoch (computed in the core from an Epoch); NaN when unknown.
Param utc(const char* name, double unix_s);
// A verbatim ASCII identifier read from an external data file (never text written in the code); shown as is.
Param data_name(const char* name, std::string id);

struct CodeInfo {
    Code code;
    std::string_view name;    // enumerator name
    std::string_view id;      // dotted id
    std::string_view params;  // comma-separated parameter names
};

std::span<const CodeInfo> code_table();
std::string_view id_of(Code code);
std::optional<Code> code_from_id(std::string_view id);
const char* id_of(Kind kind);

// transitional: removed by Task 16 (with Code::legacy): a not-yet-converted producer's text as a nested cause.
Report legacy(std::string text);

// Developer rendering (logs, test output, the campaign text report; not UI text): the code id with its parameters and
// causes, e.g. "iss.ended(cause=iss.collision)". transitional_text: removed by Task 10 (mission start), same function.
std::string describe(const Report& report);
inline std::string transitional_text(const Report& report) { return describe(report); }

// Thrown where a core operation refuses; what() is the code id.
class Refusal : public std::runtime_error {
public:
    explicit Refusal(Report report);
    const Report& report() const noexcept { return report_; }

private:
    Report report_;
};

}  // namespace starsim::report
