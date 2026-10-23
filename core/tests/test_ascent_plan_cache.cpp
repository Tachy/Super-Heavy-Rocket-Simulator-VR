// Persistent write-once plan cache shared by the test processes (follow-ups plan, Task 5).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "starsim/sim/ascent_plan_cache.hpp"

using namespace starsim;

namespace {

// A fresh, empty temp directory named by the test case (cases run as parallel processes: never share one).
struct TempDir {
    std::filesystem::path path;
    explicit TempDir(const std::string& name) : path(std::filesystem::temp_directory_path() / ("starsim_plan_cache_test_" + name)) {
        std::filesystem::remove_all(path);
        std::filesystem::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

// STARSIM_PLAN_CACHE_DIR set for a scope, the previous value restored.
struct EnvGuard {
    std::string previous;
    bool had = false;
    explicit EnvGuard(const std::string& value) {
        if (const char* old = std::getenv("STARSIM_PLAN_CACHE_DIR")) {
            previous = old;
            had = true;
        }
        _putenv_s("STARSIM_PLAN_CACHE_DIR", value.c_str());
    }
    ~EnvGuard() { _putenv_s("STARSIM_PLAN_CACHE_DIR", had ? previous.c_str() : ""); }
};

sim::AscentProfile sample_profile() {
    sim::AscentProfile p;
    p.feasible = true;
    p.reason = "Ziel erreicht \xC3\xA4 mit\nZeilenumbruch";
    p.kick_angle_rad = 0.2617993877991494;
    p.upper_propellant_at_seco_kg = 123456.789;
    p.trial_flights = 11;
    p.recovery.kind = sim::RecoveryKind::DroneShip;
    p.recovery.zone_or_ship_id = "asds_atlantic";
    p.recovery.latitude_rad = 0.4812345678901234;
    p.recovery.longitude_rad = -1.3987654321098765;
    p.recovery.heading_rad = 0.7853981633974483;
    p.target.periapsis_altitude_m = 200e3;
    p.target.apoapsis_altitude_m = 280e3;
    p.target.inclination_rad = 0.9250245035569946;
    p.target.northerly = false;
    p.booster_impact = sim::BoosterImpact{0.4801, -1.3901, 0.7801};
    return p;
}

void check_equal(const sim::AscentProfile& a, const sim::AscentProfile& b) {
    CHECK(a.feasible == b.feasible);
    CHECK(a.reason == b.reason);
    CHECK(a.kick_angle_rad == b.kick_angle_rad);
    CHECK(a.upper_propellant_at_seco_kg == b.upper_propellant_at_seco_kg);
    CHECK(a.trial_flights == b.trial_flights);
    CHECK(a.recovery.kind == b.recovery.kind);
    CHECK(a.recovery.zone_or_ship_id == b.recovery.zone_or_ship_id);
    CHECK(a.recovery.latitude_rad == b.recovery.latitude_rad);
    CHECK(a.recovery.longitude_rad == b.recovery.longitude_rad);
    CHECK(a.recovery.heading_rad == b.recovery.heading_rad);
    CHECK(a.target.periapsis_altitude_m == b.target.periapsis_altitude_m);
    CHECK(a.target.apoapsis_altitude_m == b.target.apoapsis_altitude_m);
    CHECK(a.target.inclination_rad == b.target.inclination_rad);
    CHECK(a.target.northerly == b.target.northerly);
    REQUIRE(a.booster_impact.has_value() == b.booster_impact.has_value());
    if (a.booster_impact) {
        CHECK(a.booster_impact->latitude_rad == b.booster_impact->latitude_rad);
        CHECK(a.booster_impact->longitude_rad == b.booster_impact->longitude_rad);
        CHECK(a.booster_impact->heading_rad == b.booster_impact->heading_rad);
    }
}

void write_file(const std::filesystem::path& path, const std::string& text) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path, std::ios::binary) << text;
}

}  // namespace

TEST_CASE("encode/decode round trip is bit-exact", "[sim][plan_cache]") {
    const auto p = sample_profile();
    const auto decoded = sim::decode_profile(sim::encode_profile(p));
    REQUIRE(decoded);
    check_equal(*decoded, p);

    sim::AscentProfile plain;  // defaults: no impact, empty reason, a non-finite value survives too
    plain.upper_propellant_at_seco_kg = std::nan("");
    const auto again = sim::decode_profile(sim::encode_profile(plain));
    REQUIRE(again);
    CHECK_FALSE(again->booster_impact);
    CHECK(std::isnan(again->upper_propellant_at_seco_kg));
}

TEST_CASE("corrupt cache file is ignored", "[sim][plan_cache]") {
    CHECK_FALSE(sim::decode_profile(""));
    CHECK_FALSE(sim::decode_profile("starsim-plan-cache 1\n"));
    CHECK_FALSE(sim::decode_profile("starsim-plan-cache 2\nend\n"));
    const auto text = sim::encode_profile(sample_profile());
    CHECK_FALSE(sim::decode_profile(text.substr(0, text.size() / 2)));

    TempDir dir("corrupt");
    TempDir config("corrupt_config");
    write_file(config.path / "a.toml", "x");
    EnvGuard env(dir.path.string());
    sim::store_cached_plan("some key", config.path, sample_profile());
    std::filesystem::path plan_file;
    for (const auto& entry : std::filesystem::directory_iterator(dir.path)) plan_file = entry.path();
    REQUIRE(plan_file.extension() == ".plan");
    REQUIRE(sim::load_cached_plan("some key", config.path));
    write_file(plan_file, "garbage \x01\x02 not a plan");
    CHECK_FALSE(sim::load_cached_plan("some key", config.path));
    write_file(plan_file, text.substr(0, text.size() / 2));
    CHECK_FALSE(sim::load_cached_plan("some key", config.path));
}

TEST_CASE("store then load through the directory", "[sim][plan_cache]") {
    TempDir dir("store_load");
    TempDir config("store_load_config");
    write_file(config.path / "a.toml", "x");
    EnvGuard env(dir.path.string());
    REQUIRE(sim::plan_cache_dir());

    CHECK_FALSE(sim::load_cached_plan("key A", config.path));
    sim::store_cached_plan("key A", config.path, sample_profile());
    const auto loaded = sim::load_cached_plan("key A", config.path);
    REQUIRE(loaded);
    check_equal(*loaded, sample_profile());
    CHECK_FALSE(sim::load_cached_plan("key B", config.path));

    // no temp files are left behind
    int tmp_files = 0;
    for (const auto& entry : std::filesystem::directory_iterator(dir.path)) tmp_files += entry.path().extension() == ".tmp";
    CHECK(tmp_files == 0);
}

// The memo key leaves the liftoff epoch out (one process reuses its plan for any hour), but a plan another process made
// at another epoch differs in the last bits of the kick: the disk key holds the epoch, so a cache hit is bit-identical to
// the plan the process would have made itself (the tanker flight's rendezvous moved by 1.6 t without it).
TEST_CASE("the disk key holds the liftoff epoch", "[sim][plan_cache]") {
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.5e8);
    const auto key = sim::plan_cache_key("memo key", t0);
    CHECK(key == sim::plan_cache_key("memo key", t0));
    CHECK(key != sim::plan_cache_key("memo key", t0 + 1e-6));
    CHECK(key != sim::plan_cache_key("other key", t0));

    TempDir dir("epoch_key");
    TempDir config("epoch_key_config");
    write_file(config.path / "a.toml", "x");
    EnvGuard env(dir.path.string());
    sim::store_cached_plan(key, config.path, sample_profile());
    CHECK(sim::load_cached_plan(key, config.path));
    CHECK_FALSE(sim::load_cached_plan(sim::plan_cache_key("memo key", t0 + 3600.0), config.path));
}

TEST_CASE("without the environment variable nothing is cached", "[sim][plan_cache]") {
    EnvGuard env("");
    CHECK_FALSE(sim::plan_cache_dir());
    CHECK_FALSE(sim::load_cached_plan("key", "."));
}

TEST_CASE("plan cache key changes with the config contents", "[sim][plan_cache]") {
    TempDir dir("config_hash");
    write_file(dir.path / "a.toml", "alpha = 1\n");
    write_file(dir.path / "sub" / "b.csv", "1,2,3\n");
    const auto h0 = sim::config_contents_hash(dir.path);
    CHECK(h0.size() == 16);

    // another directory with the same content gives the same hash (paths are relative)
    TempDir same("config_hash_same");
    write_file(same.path / "a.toml", "alpha = 1\n");
    write_file(same.path / "sub" / "b.csv", "1,2,3\n");
    CHECK(sim::config_contents_hash(same.path) == h0);

    TempDir changed("config_hash_changed");
    write_file(changed.path / "a.toml", "alpha = 2\n");  // one byte
    write_file(changed.path / "sub" / "b.csv", "1,2,3\n");
    CHECK(sim::config_contents_hash(changed.path) != h0);

    TempDir renamed("config_hash_renamed");
    write_file(renamed.path / "c.toml", "alpha = 1\n");  // a file renamed
    write_file(renamed.path / "sub" / "b.csv", "1,2,3\n");
    CHECK(sim::config_contents_hash(renamed.path) != h0);
}
