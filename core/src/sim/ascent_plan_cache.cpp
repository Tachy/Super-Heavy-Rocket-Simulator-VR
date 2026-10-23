#include "starsim/sim/ascent_plan_cache.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <mutex>
#include <sstream>
#include <system_error>
#include <vector>

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

namespace starsim::sim {

namespace {

constexpr const char* format_header = "starsim-plan-cache 1";
constexpr const char* profile_end = "end";

std::atomic<bool> cache_disabled{false};

// FNV-1a 64 (offset basis and prime: Fowler/Noll/Vo, http://www.isthe.com/chongo/tech/comp/fnv/).
constexpr std::uint64_t fnv_offset = 14695981039346656037ull;
constexpr std::uint64_t fnv_prime = 1099511628211ull;

void fnv_add(std::uint64_t& h, const char* data, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        h ^= static_cast<unsigned char>(data[i]);
        h *= fnv_prime;
    }
}

std::string hex64(std::uint64_t v) {
    char buf[17];
    std::snprintf(buf, sizeof buf, "%016llx", static_cast<unsigned long long>(v));
    return buf;
}

std::string to_hex(const std::string& s) {
    static const char* digits = "0123456789abcdef";
    std::string out;
    out.reserve(s.size() * 2);
    for (const unsigned char c : s) {
        out += digits[c >> 4];
        out += digits[c & 15];
    }
    return out;
}

std::optional<std::string> from_hex(const std::string& s) {
    if (s.size() % 2 != 0) return std::nullopt;
    auto nibble = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    };
    std::string out;
    for (std::size_t i = 0; i < s.size(); i += 2) {
        const int hi = nibble(s[i]), lo = nibble(s[i + 1]);
        if (hi < 0 || lo < 0) return std::nullopt;
        out += static_cast<char>(hi * 16 + lo);
    }
    return out;
}

std::string hexfloat(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%a", v);
    return buf;
}

std::optional<double> parse_double(const std::string& s) {
    if (s.empty()) return std::nullopt;
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end != s.c_str() + s.size()) return std::nullopt;
    return v;
}

std::optional<long> parse_long(const std::string& s) {
    if (s.empty()) return std::nullopt;
    char* end = nullptr;
    const long v = std::strtol(s.c_str(), &end, 10);
    if (end != s.c_str() + s.size()) return std::nullopt;
    return v;
}

int process_id() {
#ifdef _WIN32
    return _getpid();
#else
    return static_cast<int>(getpid());
#endif
}

// The directory the environment names, whether or not the planner's cache is switched off (load / store themselves
// work on it; the planner asks plan_cache_dir() first).
std::optional<std::filesystem::path> env_cache_dir() {
    const char* value = std::getenv("STARSIM_PLAN_CACHE_DIR");
    if (!value || !*value) return std::nullopt;
    return std::filesystem::path(value);
}

std::string cache_name(const std::string& key, const std::filesystem::path& config_dir) {
    std::uint64_t h = fnv_offset;
    const std::string text = key + '|' + config_contents_hash(config_dir);
    fnv_add(h, text.data(), text.size());
    return hex64(h);
}

}  // namespace

std::optional<std::filesystem::path> plan_cache_dir() {
    if (cache_disabled) return std::nullopt;
    return env_cache_dir();
}

void disable_plan_cache() { cache_disabled = true; }

std::string plan_cache_key(const std::string& memo_key, time::Epoch liftoff) {
    return memo_key + "|liftoff " + hexfloat(liftoff.tdb_seconds_past_j2000());
}

std::string config_contents_hash(const std::filesystem::path& config_dir) {
    static std::mutex mutex;
    static std::map<std::string, std::string> done;
    std::lock_guard lock(mutex);
    const auto id = config_dir.string();
    if (const auto it = done.find(id); it != done.end()) return it->second;

    std::vector<std::pair<std::string, std::filesystem::path>> files;
    std::error_code ec;
    for (std::filesystem::recursive_directory_iterator it(config_dir, ec), end; !ec && it != end; it.increment(ec)) {
        if (!it->is_regular_file(ec)) continue;
        files.emplace_back(std::filesystem::relative(it->path(), config_dir, ec).generic_string(), it->path());
    }
    std::sort(files.begin(), files.end());
    std::uint64_t h = fnv_offset;
    for (const auto& [relative, path] : files) {
        fnv_add(h, relative.data(), relative.size());
        fnv_add(h, "\0", 1);
        std::ifstream in(path, std::ios::binary);
        char buf[1 << 16];
        while (in.read(buf, sizeof buf) || in.gcount() > 0) fnv_add(h, buf, static_cast<std::size_t>(in.gcount()));
        fnv_add(h, "\0", 1);
    }
    return done[id] = hex64(h);
}

std::string encode_profile(const AscentProfile& p) {
    std::ostringstream out;
    out << format_header << '\n';
    out << "feasible " << (p.feasible ? 1 : 0) << '\n';
    out << "reason " << to_hex(p.reason) << '\n';
    out << "kick " << hexfloat(p.kick_angle_rad) << '\n';
    out << "propellant " << hexfloat(p.upper_propellant_at_seco_kg) << '\n';
    out << "trials " << p.trial_flights << '\n';
    out << "recovery_kind " << static_cast<int>(p.recovery.kind) << '\n';
    out << "recovery_id " << to_hex(p.recovery.zone_or_ship_id) << '\n';
    out << "recovery_lat " << hexfloat(p.recovery.latitude_rad) << '\n';
    out << "recovery_lon " << hexfloat(p.recovery.longitude_rad) << '\n';
    out << "recovery_heading " << hexfloat(p.recovery.heading_rad) << '\n';
    out << "target_peri " << hexfloat(p.target.periapsis_altitude_m) << '\n';
    out << "target_apo " << hexfloat(p.target.apoapsis_altitude_m) << '\n';
    out << "target_incl " << hexfloat(p.target.inclination_rad) << '\n';
    out << "target_northerly " << (p.target.northerly ? 1 : 0) << '\n';
    out << "impact " << (p.booster_impact ? 1 : 0) << '\n';
    const BoosterImpact impact = p.booster_impact.value_or(BoosterImpact{});
    out << "impact_lat " << hexfloat(impact.latitude_rad) << '\n';
    out << "impact_lon " << hexfloat(impact.longitude_rad) << '\n';
    out << "impact_heading " << hexfloat(impact.heading_rad) << '\n';
    out << profile_end << '\n';
    return out.str();
}

std::optional<AscentProfile> decode_profile(std::string_view text) {
    std::istringstream in{std::string(text)};
    std::string line;
    if (!std::getline(in, line) || line != format_header) return std::nullopt;
    std::map<std::string, std::string> fields;
    bool ended = false;
    while (std::getline(in, line)) {
        if (line == profile_end) {
            ended = true;
            break;
        }
        const auto space = line.find(' ');
        if (space == std::string::npos) return std::nullopt;
        fields[line.substr(0, space)] = line.substr(space + 1);
    }
    if (!ended) return std::nullopt;

    auto num = [&](const char* name) -> std::optional<double> {
        const auto it = fields.find(name);
        return it == fields.end() ? std::nullopt : parse_double(it->second);
    };
    auto integer = [&](const char* name) -> std::optional<long> {
        const auto it = fields.find(name);
        return it == fields.end() ? std::nullopt : parse_long(it->second);
    };
    auto str = [&](const char* name) -> std::optional<std::string> {
        const auto it = fields.find(name);
        return it == fields.end() ? std::nullopt : from_hex(it->second);
    };

    const auto feasible = integer("feasible"), trials = integer("trials"), kind = integer("recovery_kind"),
               northerly = integer("target_northerly"), impact = integer("impact");
    const auto reason = str("reason"), recovery_id = str("recovery_id");
    const auto kick = num("kick"), propellant = num("propellant"), r_lat = num("recovery_lat"), r_lon = num("recovery_lon"),
               r_head = num("recovery_heading"), peri = num("target_peri"), apo = num("target_apo"),
               incl = num("target_incl"), i_lat = num("impact_lat"), i_lon = num("impact_lon"), i_head = num("impact_heading");
    if (!feasible || !trials || !kind || !northerly || !impact || !reason || !recovery_id || !kick || !propellant ||
        !r_lat || !r_lon || !r_head || !peri || !apo || !incl || !i_lat || !i_lon || !i_head) {
        return std::nullopt;
    }
    if (*kind < 0 || *kind > static_cast<long>(RecoveryKind::DroneShip)) return std::nullopt;

    AscentProfile p;
    p.feasible = *feasible != 0;
    p.reason = *reason;
    p.kick_angle_rad = *kick;
    p.upper_propellant_at_seco_kg = *propellant;
    p.trial_flights = static_cast<int>(*trials);
    p.recovery.kind = static_cast<RecoveryKind>(*kind);
    p.recovery.zone_or_ship_id = *recovery_id;
    p.recovery.latitude_rad = *r_lat;
    p.recovery.longitude_rad = *r_lon;
    p.recovery.heading_rad = *r_head;
    p.target.periapsis_altitude_m = *peri;
    p.target.apoapsis_altitude_m = *apo;
    p.target.inclination_rad = *incl;
    p.target.northerly = *northerly != 0;
    if (*impact != 0) p.booster_impact = BoosterImpact{*i_lat, *i_lon, *i_head};
    return p;
}

std::optional<AscentProfile> load_cached_plan(const std::string& key, const std::filesystem::path& config_dir) {
    try {
        const auto dir = env_cache_dir();
        if (!dir) return std::nullopt;
        std::ifstream in(*dir / (cache_name(key, config_dir) + ".plan"), std::ios::binary);
        if (!in) return std::nullopt;
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        // the profile, then the full key (a hash collision is a miss)
        const std::string marker = std::string(profile_end) + "\nkey ";
        const auto at = text.find(marker);
        if (at == std::string::npos) return std::nullopt;
        auto stored = text.substr(at + marker.size());
        while (!stored.empty() && (stored.back() == '\n' || stored.back() == '\r')) stored.pop_back();
        const auto stored_key = from_hex(stored);
        if (!stored_key || *stored_key != key) return std::nullopt;
        return decode_profile(text.substr(0, at + std::string(profile_end).size() + 1));
    } catch (...) {
        return std::nullopt;
    }
}

void store_cached_plan(const std::string& key, const std::filesystem::path& config_dir, const AscentProfile& profile) {
    try {
        const auto dir = env_cache_dir();
        if (!dir) return;
        std::error_code ec;
        std::filesystem::create_directories(*dir, ec);
        static std::atomic<unsigned> counter{0};
        const auto name = cache_name(key, config_dir);
        const auto tmp = *dir / (name + "." + std::to_string(process_id()) + "." + std::to_string(counter++) + ".tmp");
        {
            std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
            if (!out) return;
            out << encode_profile(profile) << "key " << to_hex(key) << '\n';
            if (!out.flush()) {
                out.close();
                std::filesystem::remove(tmp, ec);
                return;
            }
        }
        std::filesystem::rename(tmp, *dir / (name + ".plan"), ec);
        if (ec) std::filesystem::remove(tmp, ec);  // another process won the race: the content is the same
    } catch (...) {
    }
}

}  // namespace starsim::sim
