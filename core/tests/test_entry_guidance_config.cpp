// Ship entry guidance parameters from the vehicle config (plan 2026-11-03 planner-vehicle-parameters Task 4): K values
// read from [stage.entry], the RCS-derived rates equal to their hand formulas, the lunar q limit, refusal without the
// keys, inheritance by reused ship stages.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/entry_guidance.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/gnc/vehicle_capability.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Approx;

namespace {

constexpr double deg = constants::deg_to_rad;

const config::StageConfig& stage_of(const config::VehicleConfig& cfg, const std::string& id) {
    const auto it = std::find_if(cfg.stages.begin(), cfg.stages.end(), [&](const auto& s) { return s.id == id; });
    REQUIRE(it != cfg.stages.end());
    return *it;
}

// The RCS's angular acceleration at the entry mass (dry + full headers), the weaker transverse axis.
double rcs_acceleration(const config::VehicleConfig& cfg, const config::StageConfig& ship) {
    const auto m = gnc::ship_landing_masses(cfg, ship);
    return gnc::rcs_pitch_acceleration_radps2(ship, m.heavy_pitch_inertia_kg_m2, m.heavy_cg_station_m);
}

}  // namespace

TEST_CASE("Entry guidance parameters follow the vehicle config", "[ship][entry][guidance][config]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto& ship = stage_of(cfg, "ship");
    REQUIRE(ship.entry.has_value());
    const auto& e = *ship.entry;
    const auto p = gnc::entry_guidance_parameters(cfg, ship, gnc::EntryKind::Orbital);
    // K: as configured
    CHECK(p.max_dynamic_pressure_pa == e.max_dynamic_pressure_pa);
    CHECK(p.max_dynamic_pressure_pa == 25000.0);
    CHECK(p.terminal_mach == 5.0);
    CHECK(p.terminal_mach_latest == 3.0);
    CHECK(p.terminal_min_alpha_rad == Approx(50.0 * deg).epsilon(1e-12));
    CHECK(p.terminal_alpha_authority_rad == Approx(20.0 * deg).epsilon(1e-12));
    CHECK(p.fast_bank_rate_radps == Approx(3.0 * deg).epsilon(1e-12));
    CHECK(p.terminal_slew_rate_radps == Approx(3.0 * deg).epsilon(1e-12));
    // D: the RCS at the entry mass. Hand value: the yaw pairs (2 x 1500 N tangential per side at azimuth 0 / 180, forward
    // pod 46 m, aft pod 6 m) about the CG ~25 m: 1.2e5 N m; I ~5.5e7 kg m^2 -> ~2.2e-3 rad/s^2.
    const double a = rcs_acceleration(cfg, ship);
    const auto m = gnc::ship_landing_masses(cfg, ship);
    CHECK(a * m.heavy_pitch_inertia_kg_m2 == Approx(1.2e5).epsilon(0.02));
    CHECK(p.terminal_nose_slew_acceleration_radps2 == Approx(std::min(e.nose_slew_acceleration_max_radps2, a)).epsilon(1e-12));
    CHECK(p.terminal_nose_slew_acceleration_radps2 < e.nose_slew_acceleration_max_radps2);  // the RCS binds: ~0.13 deg/s^2
    const double stop_rate = std::sqrt(2.0 * a * gnc::entry_settled_bank_rad);
    CHECK(p.bank_rate_radps == Approx(std::min(e.bank_rate_max_radps, stop_rate)).epsilon(1e-12));
    CHECK(stop_rate > e.bank_rate_max_radps);  // ~1.1 deg/s: the 1 deg/s cap binds
    // the rest of the preset stays: an orbital entry carries no skips
    CHECK_FALSE(p.carry_skips);
    CHECK(p.gate_height_m == gnc::EntryGuidanceParameters{}.gate_height_m);
}

TEST_CASE("Lunar return entry uses the lunar q limit from config", "[ship][entry][guidance][config][lunar]") {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    auto& ship = cfg.stages.back();
    REQUIRE(ship.id == "ship");
    const auto lunar = gnc::entry_guidance_parameters(cfg, ship, gnc::EntryKind::LunarReturn);
    CHECK(lunar.max_dynamic_pressure_pa == 32000.0);
    CHECK(lunar.carry_skips);  // the lunar_return() preset underneath
    CHECK(lunar.final_bank_rad == gnc::EntryGuidanceParameters::lunar_return().final_bank_rad);
    // the limit is the config's, not the preset's
    ship.entry->lunar_max_dynamic_pressure_pa = 30000.0;
    CHECK(gnc::entry_guidance_parameters(cfg, ship, gnc::EntryKind::LunarReturn).max_dynamic_pressure_pa == 30000.0);
    CHECK(gnc::entry_guidance_parameters(cfg, ship, gnc::EntryKind::Orbital).max_dynamic_pressure_pa == 25000.0);
    // an aerocapture pass takes the orbital limit on its own preset
    const auto pass = gnc::entry_guidance_parameters(cfg, ship, gnc::EntryKind::Orbital,
                                                     gnc::EntryGuidanceParameters::aerocapture(400e3, -1.0));
    CHECK(pass.aerocapture_apoapsis_m == 400e3);
    CHECK(pass.max_dynamic_pressure_pa == 25000.0);
}

TEST_CASE("Ship without entry keys is refused", "[ship][entry][config]") {
    namespace fs = std::filesystem;
    const fs::path source(STARSIM_CONFIG_DIR);
    for (const std::string key : {"lunar_max_dynamic_pressure_pa", "nose_slew_acceleration_max_deg_s2"}) {
        const auto dir = fs::temp_directory_path() / ("starsim_entry_keys_" + key);
        fs::remove_all(dir);
        fs::create_directories(dir / "vehicles");
        fs::copy(source / "engines", dir / "engines", fs::copy_options::recursive);
        fs::copy_file(source / "propellants.toml", dir / "propellants.toml");
        std::ifstream in(source / "vehicles" / "starship_block3.toml");
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const auto at = text.find("\n" + key + " =");
        REQUIRE(at != std::string::npos);
        text.insert(at + 1, "# ");
        std::ofstream(dir / "vehicles" / "starship_block3.toml") << text;
        try {
            config::load_vehicle(dir, "starship_block3");
            FAIL("loaded without " << key);
        } catch (const config::ConfigError& e) {
            const std::string what = e.what();
            CHECK(what.find("Eintrittsführung") != std::string::npos);  // German, naming the key and the file
            CHECK(what.find(key) != std::string::npos);
            CHECK(what.find("starship_block3.toml") != std::string::npos);
        }
        fs::remove_all(dir);
    }
    // vehicles without a belly-first entry load as before (the capsule flies its own preset)
    for (const char* id : {"crew_dragon", "starship_depot", "falcon9_block5", "starship_hls"}) {
        const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, id);
        for (const auto& s : cfg.stages) CHECK_FALSE(s.entry.has_value());
    }
}

TEST_CASE("Tanker inherits the ship entry profile", "[ship][entry][config]") {
    const auto block3 = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto tanker = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_tanker");
    const auto& own_stage = stage_of(block3, "ship");
    const auto& tanker_stage = stage_of(tanker, "ship");
    REQUIRE(tanker_stage.entry.has_value());
    CHECK(tanker_stage.entry->max_dynamic_pressure_pa == own_stage.entry->max_dynamic_pressure_pa);
    CHECK(tanker_stage.entry->nose_slew_acceleration_max_radps2 == own_stage.entry->nose_slew_acceleration_max_radps2);
    const auto own = gnc::entry_guidance_parameters(block3, own_stage, gnc::EntryKind::Orbital);
    const auto inherited = gnc::entry_guidance_parameters(tanker, tanker_stage, gnc::EntryKind::Orbital);
    CHECK(inherited.terminal_mach == own.terminal_mach);
    CHECK(inherited.fast_bank_rate_radps == own.fast_bank_rate_radps);
    // the derived acceleration follows the tanker's own entry mass and inertia
    CHECK(inherited.terminal_nose_slew_acceleration_radps2 ==
          Approx(rcs_acceleration(tanker, tanker_stage)).epsilon(1e-12));
    if (tanker_stage.dry_mass_kg != own_stage.dry_mass_kg) {
        CHECK(inherited.terminal_nose_slew_acceleration_radps2 != own.terminal_nose_slew_acceleration_radps2);
    }
}
