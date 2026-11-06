#include <catch2/catch_test_macros.hpp>

#include <regex>
#include <set>
#include <string>

#include "starsim/report/report.hpp"

using namespace starsim;

TEST_CASE("A default report is ok", "[report]") {
    report::Report r;
    CHECK(r.ok());
    CHECK(report::id_of(r.code) == "ok");
}

TEST_CASE("Parameters carry kind and SI value", "[report]") {
    auto r = report::make(report::Code::legacy, {report::legacy_text("text", "x")});
    CHECK_FALSE(r.ok());
    CHECK(r.params[0].kind == report::Kind::legacy_text);
    CHECK(r.params[0].text == "x");
    CHECK(report::angle("slope", 0.1449).value_si == 0.1449);
    CHECK(report::angle("slope", 0.1449).kind == report::Kind::angle);
    CHECK(report::ref("site", "lunar_site.mons_mouton.name").text == "lunar_site.mons_mouton.name");
    CHECK(report::count("n", 3).value_si == 3.0);
}

TEST_CASE("Code table: unique dotted ASCII ids, enumerator = id with _", "[report]") {
    const std::regex id_re("^[a-z][a-z0-9_]*(\\.[a-z0-9_]+)*$");
    const std::regex params_re("^([a-z_][a-z0-9_]*(,[a-z_][a-z0-9_]*)*)?$");
    std::set<std::string> ids;
    for (const report::CodeInfo& info : report::code_table()) {
        const std::string id(info.id);
        CHECK(std::regex_match(id, id_re));
        CHECK(ids.insert(id).second);
        std::string expected = id;
        for (char& c : expected) {
            if (c == '.') c = '_';
        }
        CHECK(expected == info.name);
        CHECK(std::regex_match(std::string(info.params), params_re));
    }
}

TEST_CASE("code_from_id inverts id_of", "[report]") {
    for (const report::CodeInfo& info : report::code_table()) {
        auto code = report::code_from_id(info.id);
        REQUIRE(code.has_value());
        CHECK(*code == info.code);
        CHECK(report::id_of(info.code) == info.id);
    }
    CHECK_FALSE(report::code_from_id("nope").has_value());
}

TEST_CASE("Refusal carries its report", "[report]") {
    try {
        throw report::Refusal(report::make(report::Code::legacy, {report::legacy_text("text", "x")}));
    } catch (const report::Refusal& e) {
        CHECK(e.report().code == report::Code::legacy);
        CHECK(std::string(e.what()) == "legacy");
        return;
    }
    FAIL("not thrown");
}

TEST_CASE("with_cause nests a report", "[report]") {
    auto r = report::with_cause(report::make(report::Code::legacy),
                                report::make(report::Code::command_internal_error));
    REQUIRE(r.causes.size() == 1);
    CHECK(r.causes[0].code == report::Code::command_internal_error);
}
