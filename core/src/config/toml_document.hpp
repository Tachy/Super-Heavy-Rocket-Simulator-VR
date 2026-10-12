#pragma once

#include <filesystem>
#include <sstream>
#include <string>
#include <string_view>

#include <toml++/toml.hpp>

#include "starsim/config/vehicle_config.hpp"

namespace starsim::config::detail {

// Reads a TOML file and remembers its path for error messages.
struct Document {
    std::filesystem::path path;
    toml::table root;

    explicit Document(std::filesystem::path p) : path(std::move(p)) {
        if (!std::filesystem::exists(path)) {
            throw ConfigError("config file not found: " + path.string());
        }
        try {
            root = toml::parse_file(path.string());
        } catch (const toml::parse_error& e) {
            std::ostringstream message;
            message << path.string() << ": " << e.description() << " (line " << e.source().begin.line << ")";
            throw ConfigError(message.str());
        }
    }

    [[noreturn]] void fail(const std::string& what) const { throw ConfigError(path.string() + ": " + what); }

    double number(const toml::table& t, std::string_view key) const {
        if (auto v = t[key].value<double>()) return *v;
        fail("missing number '" + std::string(key) + "'");
    }
    double number_or(const toml::table& t, std::string_view key, double fallback) const {
        return t[key].value<double>().value_or(fallback);
    }
    double positive(const toml::table& t, std::string_view key) const {
        const double v = number(t, key);
        if (!(v > 0.0)) fail("'" + std::string(key) + "' must be positive");
        return v;
    }
    std::string text(const toml::table& t, std::string_view key) const {
        if (auto v = t[key].value<std::string>()) return *v;
        fail("missing string '" + std::string(key) + "'");
    }
    std::string text_or(const toml::table& t, std::string_view key, std::string fallback = {}) const {
        return t[key].value<std::string>().value_or(std::move(fallback));
    }
    bool boolean(const toml::table& t, std::string_view key) const {
        if (auto v = t[key].value<bool>()) return *v;
        fail("missing boolean '" + std::string(key) + "'");
    }
    const toml::table& table(const toml::table& t, std::string_view key) const {
        if (auto v = t[key].as_table()) return *v;
        fail("missing table [" + std::string(key) + "]");
    }
};

}  // namespace starsim::config::detail
