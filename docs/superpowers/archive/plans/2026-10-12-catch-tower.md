# Catch Tower and Arms Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A launch tower with two automatic catch arms that physically catch (or fail to catch) the returning booster through pin-on-arm contact forces, visible in Godot.

**Architecture:** New core module `contact` (geometry primitives, tower geometry, arm/carriage dynamics, penalty pin contact, catch trigger, tower collision) below `sim`. `sim::CatchTowerSim` applies contact wrenches to `FlightBody`s each 200 Hz tick and judges catch/impact; `FlightSession` owns it per launch site. The booster return guidance aims at the catch point and rolls its pins towards the arms. Godot draws the tower procedurally from the same core geometry.

**Tech Stack:** C++20 (MSVC 2022, CMake, Ninja), toml++, Catch2 v3.8.1, godot-cpp GDExtension, Godot 4.7.2 double precision (GDScript).

**Spec:** `docs/superpowers/specs/2026-10-12-catch-tower-design.md`

## Global Constraints

- Module order: `contact` may use `math`, `frames`, `config`; never `vehicle`, `gnc`, `sim`. `sim` and `gnc` may use `contact`.
- SI units, radians internally; frame and unit in names (`_site`, `_itrf`, `_gcrf`, `_m`, `_mps`, `_rad`, `_n`).
- Site frame: origin at the pad centre on the ground (site `height_m`), x east, y north, z up (geodetic vertical).
- All tower and pin dimensions are config values with provenance tags ([E] estimate); no geometry numbers in code.
- Contact constants (stiffness, damping, friction, rest thresholds) are named constants with a comment; tuned for the 200 Hz RK4 tick.
- Existing tests keep passing unchanged: sites built as `sim::LaunchSite{lat, lon, h}` have no tower.
- UI strings German, code/comments English. Commits: Conventional Commits, ending with the session's attribution lines.
- Build/test (PowerShell, repo root): `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests` then `build/core/tests/starsim_core_tests.exe "<tags>"`; full suite `tools/build.ps1` at the end of each core task.

## Review Focus

- A booster that drifts sideways while hanging on the arms (one pin slides off the arm edge) -> must not be declared caught; the at-rest check requires both pins in contact; tested in Task 8 scenario (b).
- A stack standing on the launch mount (bottom exactly at the mount top) must not be reported as colliding with the mount; tested in Task 6.
- Coast segments must never skip the contact physics while a body is near the tower (falling booster after cutoff with engines off); tested in Task 8 (scenario (a) runs with the default coast logic).
- Pins pointing along the tower direction instead of across (wrong roll) -> pins pass between the arms; the guidance's roll reference must be sign-safe for both opposite pins; tested in Task 9 (unit test of the roll reference).
- The ascent from a tower site starts 20 m higher on the mount and must not hit the tower; tested in Task 9 integration.

---

## File Structure

| File | Responsibility |
|------|----------------|
| `core/src/config/toml_document.hpp` | shared TOML document helper (moved out of `vehicle_config.cpp`) |
| `core/include/starsim/config/launch_site_config.hpp`, `core/src/config/launch_site_config.cpp` | `TowerConfig`, `LaunchSiteConfig`, `load_launch_site` |
| `core/include/starsim/config/vehicle_config.hpp` (+ `.cpp`) | `CatchPinConfig` in `StageConfig` |
| `core/include/starsim/contact/primitives.hpp`, `core/src/contact/primitives.cpp` | oriented box, annulus, point/capsule tests |
| `core/include/starsim/contact/tower_geometry.hpp`, `.cpp` | site frame, tower box, mount, hinges, arm boxes |
| `core/include/starsim/contact/catch_arms.hpp`, `.cpp` | arm opening (rate limit, stall), carriage (spring-damper, stroke) |
| `core/include/starsim/contact/pin_contact.hpp`, `.cpp` | body sample points, penalty contact with friction |
| `core/include/starsim/contact/catch_trigger.hpp`, `.cpp` | automatic close decision |
| `core/include/starsim/contact/tower_collision.hpp`, `.cpp` | hull capsule vs tower and mount |
| `core/include/starsim/vehicle/flight_body.hpp` (+ `.cpp`) | external wrench, `caught()`, `mark_impacted()` |
| `core/include/starsim/sim/catch_tower_sim.hpp`, `core/src/sim/catch_tower_sim.cpp` | tower during a flight: contact, trigger, collision, catch |
| `core/include/starsim/sim/flight_session.hpp` (+ `.cpp`) | owns the tower, stands stacks on the mount, `place_body`, coast gating |
| `core/include/starsim/gnc/booster_return.hpp` (+ `.cpp`) | catch target, pin roll reference, cutoff on contact |
| `bindings/src/flight_sim.{hpp,cpp}` | `launch_from_site`, `site_tower`, `tower_state` |
| `game/world/tower_view.gd`, `game/shaders/lattice.gdshader` | procedural tower |
| `game/world/world.gd`, `game/vehicle/flight_controller.gd`, `game/ui/flight_hud.gd`, `game/tests/render_check.gd`, `game/tests/smoke_test.gd` | wiring |
| `config/launch_sites.toml`, `config/vehicles/starship_block{1,2,3}.toml` | tower and pin data |

Spec refinements decided while planning (recorded in the spec in Task 12): hinge spacing is derived (`arm_closed_gap_m / 2 + arm_width_m / 2` from the catch axis), so `arm_hinge_spacing_m` is dropped; pins are listed by `azimuths_deg` (count implied) and get `max_load_n` (pin overload = impact, which is how an 8 m/s arrival fails); the automatic trigger closes the arms when the predicted time to contact is within the closing time + 1 s (a fixed height window would close too late at catch speeds); `CatchTowerSim::close_arms(instantly)` exists for scenario tests.

---

### Task 1: Launch site and catch pin configuration

**Files:**
- Create: `core/src/config/toml_document.hpp`, `core/include/starsim/config/launch_site_config.hpp`, `core/src/config/launch_site_config.cpp`, `core/tests/test_launch_site_config.cpp`
- Modify: `core/src/config/vehicle_config.cpp` (use the shared document, parse `[stage.catch_pins]`), `core/include/starsim/config/vehicle_config.hpp` (`CatchPinConfig`), `config/launch_sites.toml`, `config/vehicles/starship_block1.toml`, `starship_block2.toml`, `starship_block3.toml`, `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Produces: `config::TowerConfig` (fields below), `config::LaunchSiteConfig { id, name, latitude_rad, longitude_rad, height_m, std::optional<TowerConfig> tower }`, `config::LaunchSiteConfig config::load_launch_site(const std::filesystem::path&, std::string_view)`, `config::CatchPinConfig { std::vector<double> azimuths_rad; double station_m, protrusion_m, max_load_n; }` as `StageConfig::catch_pins`.

- [ ] **Step 1: Write the failing test** - `core/tests/test_launch_site_config.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Matchers::WithinAbs;

TEST_CASE("Launch sites load with their catch tower", "[config]") {
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1");
    CHECK(site.name == "Starbase OLP-1");
    CHECK_THAT(site.latitude_rad, WithinAbs(25.99686 * constants::deg_to_rad, 1e-12));
    CHECK(site.height_m == 10.0);
    REQUIRE(site.tower.has_value());
    const auto& t = *site.tower;
    CHECK(t.catch_height_m > t.mount_height_m);
    CHECK(t.arm_closed_gap_m > 9.0);  // wider than the 9 m booster hull
    CHECK(t.mount_inner_radius_m < t.mount_outer_radius_m);
    CHECK_THAT(t.arm_open_rad, WithinAbs(40.0 * constants::deg_to_rad, 1e-12));
    CHECK(config::load_launch_site(STARSIM_CONFIG_DIR, "ksc_lc39a").tower.has_value());
    CHECK_THROWS_AS(config::load_launch_site(STARSIM_CONFIG_DIR, "nowhere"), config::ConfigError);
}

TEST_CASE("Boosters define two opposite catch pins", "[config]") {
    for (const char* id : {"starship_block1", "starship_block2", "starship_block3"}) {
        CAPTURE(id);
        const auto v = config::load_vehicle(STARSIM_CONFIG_DIR, id);
        const auto& booster = v.stages.front();
        const auto& pins = booster.catch_pins;
        REQUIRE(pins.azimuths_rad.size() == 2);
        CHECK_THAT(std::abs(pins.azimuths_rad[1] - pins.azimuths_rad[0]), WithinAbs(constants::pi, 1e-12));
        CHECK(pins.station_m > booster.grid_fins.station_m);
        CHECK(pins.station_m < booster.length_m);
        CHECK(pins.protrusion_m > 0.0);
        CHECK(pins.max_load_n > 0.0);
        CHECK(v.stages.back().catch_pins.azimuths_rad.empty());  // ship: none yet
    }
}
```

Add `test_launch_site_config.cpp` to `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`
Expected: compile error, `launch_site_config.hpp` not found.

- [ ] **Step 3: Move the TOML helper into a shared private header** - `core/src/config/toml_document.hpp`

```cpp
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
```

In `core/src/config/vehicle_config.cpp`: delete the `struct Document { ... };` definition from the anonymous namespace, add `#include "toml_document.hpp"` after the other includes, and put `using detail::Document;` as the first line inside the anonymous namespace. Remove the now unused `#include <fstream>`/`<sstream>` only if nothing else in the file uses them.

- [ ] **Step 4: Catch pins in the vehicle config**

In `core/include/starsim/config/vehicle_config.hpp`, after `GridFinConfig`:

```cpp
// Catch pins of a stage: hardpoints the tower arms catch (the returning booster).
struct CatchPinConfig {
    std::vector<double> azimuths_rad;  // one pin per entry, from body +y towards +z (like the grid fins)
    double station_m = 0.0;            // from the bottom of the stage
    double protrusion_m = 0.0;         // beyond the hull radius
    double max_load_n = 0.0;           // structural limit per pin
};
```

and in `StageConfig` after `grid_fins`: `CatchPinConfig catch_pins;  // empty azimuths: none`.

In `vehicle_config.cpp`, right after the `grid_fins` block of the stage loader:

```cpp
    if (auto pins = t["catch_pins"].as_table()) {
        const auto* azimuths = (*pins)["azimuths_deg"].as_array();
        if (!azimuths || azimuths->empty()) {
            doc.fail("catch_pins.azimuths_deg must list one azimuth per pin");
        }
        for (const auto& a : *azimuths) {
            const auto value = a.value<double>();
            if (!value) doc.fail("catch_pins.azimuths_deg must be numbers");
            s.catch_pins.azimuths_rad.push_back(*value * constants::deg_to_rad);
        }
        s.catch_pins.station_m = doc.number(*pins, "station_m");
        s.catch_pins.protrusion_m = doc.positive(*pins, "protrusion_m");
        s.catch_pins.max_load_n = doc.positive(*pins, "max_load_n");
    }
```

- [ ] **Step 5: Launch site loader** - `core/include/starsim/config/launch_site_config.hpp`

```cpp
#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include "starsim/config/vehicle_config.hpp"

namespace starsim::config {

// Catch tower of a launch site ([site.tower] in config/launch_sites.toml). Lengths in the site frame: origin
// at the pad centre on the ground, east/north/up.
struct TowerConfig {
    double azimuth_rad = 0.0;           // pad centre -> tower centre, clockwise from north
    double distance_m = 0.0;            // pad centre -> tower centre, horizontal
    double width_m = 0.0;               // square cross-section
    double height_m = 0.0;
    double mount_height_m = 0.0;        // top of the launch mount: a stack on the pad stands here
    double mount_outer_radius_m = 0.0;
    double mount_inner_radius_m = 0.0;
    double arm_length_m = 0.0;          // hinge -> tip
    double arm_width_m = 0.0;
    double arm_thickness_m = 0.0;
    double arm_open_rad = 0.0;          // each arm swung outward from closed (parallel arms)
    double arm_closed_gap_m = 0.0;      // clear gap between the closed arms
    double arm_rate_rad_s = 0.0;
    double arm_force_limit_n = 0.0;     // closing stalls above this push of the hull
    double catch_height_m = 0.0;        // arm top surface (carriage at rest) above the pad ground
    double carriage_mass_kg = 0.0;
    double carriage_stiffness_n_m = 0.0;
    double carriage_damping_n_s_m = 0.0;
    double carriage_stroke_m = 0.0;
};

struct LaunchSiteConfig {
    std::string id;
    std::string name;
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;  // ground (pad) height above WGS84
    std::optional<TowerConfig> tower;
};

// Reads site `site_id` from config/launch_sites.toml.
LaunchSiteConfig load_launch_site(const std::filesystem::path& config_dir, std::string_view site_id);

}  // namespace starsim::config
```

`core/src/config/launch_site_config.cpp`:

```cpp
#include "starsim/config/launch_site_config.hpp"

#include "starsim/math/constants.hpp"
#include "toml_document.hpp"

namespace starsim::config {

namespace {

using detail::Document;

TowerConfig load_tower(const Document& doc, const toml::table& t) {
    const double deg = constants::deg_to_rad;
    TowerConfig c;
    c.azimuth_rad = doc.number(t, "azimuth_deg") * deg;
    c.distance_m = doc.positive(t, "distance_m");
    c.width_m = doc.positive(t, "width_m");
    c.height_m = doc.positive(t, "height_m");
    c.mount_height_m = doc.positive(t, "mount_height_m");
    c.mount_outer_radius_m = doc.positive(t, "mount_outer_radius_m");
    c.mount_inner_radius_m = doc.positive(t, "mount_inner_radius_m");
    c.arm_length_m = doc.positive(t, "arm_length_m");
    c.arm_width_m = doc.positive(t, "arm_width_m");
    c.arm_thickness_m = doc.positive(t, "arm_thickness_m");
    c.arm_open_rad = doc.positive(t, "arm_open_deg") * deg;
    c.arm_closed_gap_m = doc.positive(t, "arm_closed_gap_m");
    c.arm_rate_rad_s = doc.positive(t, "arm_rate_deg_s") * deg;
    c.arm_force_limit_n = doc.positive(t, "arm_force_limit_n");
    c.catch_height_m = doc.positive(t, "catch_height_m");
    c.carriage_mass_kg = doc.positive(t, "carriage_mass_kg");
    c.carriage_stiffness_n_m = doc.positive(t, "carriage_stiffness_n_m");
    c.carriage_damping_n_s_m = doc.positive(t, "carriage_damping_n_s_m");
    c.carriage_stroke_m = doc.positive(t, "carriage_stroke_m");
    if (c.mount_inner_radius_m >= c.mount_outer_radius_m) doc.fail("tower: mount_inner_radius_m >= mount_outer_radius_m");
    if (c.catch_height_m <= c.mount_height_m) doc.fail("tower: catch_height_m must be above mount_height_m");
    if (c.distance_m <= 0.5 * c.width_m) doc.fail("tower: the pad centre lies inside the tower");
    return c;
}

}  // namespace

LaunchSiteConfig load_launch_site(const std::filesystem::path& config_dir, std::string_view site_id) {
    const Document doc(config_dir / "launch_sites.toml");
    if (auto list = doc.root["site"].as_array()) {
        for (const auto& node : *list) {
            const auto& t = *node.as_table();
            if (doc.text(t, "id") != site_id) continue;
            LaunchSiteConfig s;
            s.id = std::string(site_id);
            s.name = doc.text(t, "name");
            s.latitude_rad = doc.number(t, "latitude_deg") * constants::deg_to_rad;
            s.longitude_rad = doc.number(t, "longitude_deg") * constants::deg_to_rad;
            s.height_m = doc.number(t, "height_m");
            if (auto tower = t["tower"].as_table()) s.tower = load_tower(doc, *tower);
            return s;
        }
    }
    doc.fail("launch site '" + std::string(site_id) + "' not defined");
}

}  // namespace starsim::config
```

Add `src/config/launch_site_config.cpp` to `core/CMakeLists.txt` (after `src/config/vehicle_config.cpp`).

- [ ] **Step 6: Data** - `config/launch_sites.toml`: append a `[site.tower]` table under each `[[site]]` (directly after its `height_m` line).

Starbase OLP-1:

```toml
[site.tower]                  # catch tower ("chopsticks"); all values [E] from public imagery and reporting
azimuth_deg = 315.0           # [E] pad centre -> tower centre, clockwise from north (verify against imagery)
distance_m = 30.0             # [E]
width_m = 12.0                # [E] square lattice tower
height_m = 146.0              # [E]
mount_height_m = 20.0         # [E] top of the orbital launch mount: a stack stands here
mount_outer_radius_m = 13.0   # [E]
mount_inner_radius_m = 5.5    # [E] open centre for the engine exhaust
arm_length_m = 36.0           # [E] hinge -> tip
arm_width_m = 2.0             # [E] top surface the pins land on
arm_thickness_m = 2.5         # [E]
arm_open_deg = 40.0           # [E] each arm swung outward
arm_closed_gap_m = 9.6        # [E] hull 9 m + 0.3 m each side; pins (4.5-5.5 m) overlap the arm tops by 0.7 m
arm_rate_deg_s = 10.0         # [E]
arm_force_limit_n = 2.0e6     # [E] closing stalls when the hull pushes back harder
catch_height_m = 95.0         # [E] arm tops above the pad ground; caught booster bottom clears the mount
carriage_mass_kg = 1.5e5      # [E] carriage + arms
carriage_stiffness_n_m = 2.0e6    # [E] shock absorbers: ~1.4 m static travel under a 275 t booster
carriage_damping_n_s_m = 1.2e6    # [E]
carriage_stroke_m = 4.0       # [E] travel to the hard stop
```

KSC LC-39A: the same table with `azimuth_deg = 0.0` and the comment `# [E] tower north of the mount (verify against imagery)`.

`config/vehicles/starship_block1.toml` and `starship_block2.toml`: in the booster stage (first `[[stage]]`), directly before its `[stage.recovery]` line:

```toml
[stage.catch_pins]
azimuths_deg = [90.0, 270.0]           # [E] two hardpoints between the grid fins, caught by the tower arms
station_m = 65.5                       # [E] above the grid fins, below the hot-staging ring
protrusion_m = 1.0                     # [E] beyond the hull
max_load_n = 5.0e6                     # [E] structural limit per pin (~1.9 g of a 275 t booster on two pins)
```

`starship_block3.toml`: the same with `azimuths_deg = [45.0, 225.0]` (between the fins at 0/90/180) and `station_m = 66.5`.

- [ ] **Step 7: Run tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests; build/core/tests/starsim_core_tests.exe "[config]"`
Expected: PASS. Then `python tools/vehicle_report.py starship_block2` still runs (pins add no mass).

- [ ] **Step 8: Commit**

```bash
git add core/src/config core/include/starsim/config core/tests/test_launch_site_config.cpp core/CMakeLists.txt core/tests/CMakeLists.txt config/launch_sites.toml config/vehicles
git commit -m "feat: launch site towers and booster catch pins in the configuration"
```

---

### Task 2: Contact primitives

**Files:**
- Create: `core/include/starsim/contact/primitives.hpp`, `core/src/contact/primitives.cpp`, `core/tests/test_contact_primitives.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Produces: `contact::from_columns(Vec3, Vec3, Vec3) -> Mat3`, `contact::column(const Mat3&, int) -> Vec3`, `struct OrientedBox { Vec3 center; Mat3 axes; Vec3 half_extents; }`, `struct Penetration { bool inside; double depth_m; Vec3 normal; }`, `Penetration point_in_box(const OrientedBox&, const Vec3&)`, `double distance_to_box(...)`, `struct Annulus { double inner_radius_m, outer_radius_m, height_m; }`, `double distance_to_annulus(const Annulus&, const Vec3&)`, `bool capsule_overlaps_box(Vec3 a, Vec3 b, double radius_m, const OrientedBox&)`, `bool capsule_overlaps_annulus(Vec3 a, Vec3 b, double radius_m, const Annulus&)`.

- [ ] **Step 1: Write the failing test** - `core/tests/test_contact_primitives.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "starsim/contact/primitives.hpp"

using namespace starsim;
using namespace starsim::contact;
using Catch::Matchers::WithinAbs;
using math::Vec3;

namespace {

OrientedBox axis_aligned() { return {{10.0, 0.0, 0.0}, math::Mat3::identity(), {2.0, 1.0, 0.5}}; }

}  // namespace

TEST_CASE("Point in box: depth and normal of the nearest face", "[contact]") {
    const auto top = point_in_box(axis_aligned(), {10.5, 0.2, 0.45});
    REQUIRE(top.inside);
    CHECK_THAT(top.depth_m, WithinAbs(0.05, 1e-12));
    CHECK(top.normal == Vec3{0.0, 0.0, 1.0});
    const auto side = point_in_box(axis_aligned(), {11.9, 0.0, 0.0});
    REQUIRE(side.inside);
    CHECK_THAT(side.depth_m, WithinAbs(0.1, 1e-12));
    CHECK(side.normal == Vec3{1.0, 0.0, 0.0});
    CHECK_FALSE(point_in_box(axis_aligned(), {13.0, 0.0, 0.0}).inside);
    CHECK_THAT(distance_to_box(axis_aligned(), {13.0, 0.0, 0.0}), WithinAbs(1.0, 1e-12));
    CHECK(distance_to_box(axis_aligned(), {10.0, 0.0, 0.0}) == 0.0);
}

TEST_CASE("Point in a rotated box reports the face normal in the frame", "[contact]") {
    // Local x along frame y.
    const OrientedBox box{{0.0, 0.0, 0.0}, from_columns({0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}), {2.0, 1.0, 0.5}};
    const auto p = point_in_box(box, {0.0, 1.9, 0.0});
    REQUIRE(p.inside);
    CHECK_THAT(p.depth_m, WithinAbs(0.1, 1e-12));
    CHECK_THAT(p.normal.y, WithinAbs(1.0, 1e-12));
    CHECK(column(box.axes, 1) == Vec3{-1.0, 0.0, 0.0});
}

TEST_CASE("Capsules against a box", "[contact]") {
    CHECK(capsule_overlaps_box({10.0, 0.0, -5.0}, {10.0, 0.0, 5.0}, 0.2, axis_aligned()));
    CHECK_FALSE(capsule_overlaps_box({10.0, 3.0, -5.0}, {10.0, 3.0, 5.0}, 1.0, axis_aligned()));  // 2 m clear
    CHECK(capsule_overlaps_box({10.0, 3.0, -5.0}, {10.0, 3.0, 5.0}, 2.1, axis_aligned()));
}

TEST_CASE("Capsules against the launch mount annulus", "[contact]") {
    const Annulus mount{5.5, 13.0, 20.0};
    CHECK_FALSE(capsule_overlaps_annulus({0.0, 0.0, 20.0}, {0.0, 0.0, 90.0}, 4.5, mount));  // stands on it
    CHECK(capsule_overlaps_annulus({3.0, 0.0, 10.0}, {3.0, 0.0, 80.0}, 4.5, mount));        // fell into the ring
    CHECK_FALSE(capsule_overlaps_annulus({8.0, 0.0, 25.0}, {8.0, 0.0, 90.0}, 4.5, mount));  // 5 m above it
    CHECK_THAT(distance_to_annulus(mount, {0.0, 0.0, 0.0}), WithinAbs(5.5, 1e-12));
    CHECK_THAT(distance_to_annulus(mount, {20.0, 0.0, 30.0}), WithinAbs(std::hypot(7.0, 10.0), 1e-12));
}
```

(Include `<cmath>` for `std::hypot`.) Add the test file to `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`
Expected: compile error, `contact/primitives.hpp` not found.

- [ ] **Step 3: Implement** - `core/include/starsim/contact/primitives.hpp`

```cpp
#pragma once

#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::contact {

inline math::Mat3 from_columns(const math::Vec3& a, const math::Vec3& b, const math::Vec3& c) {
    math::Mat3 m = math::Mat3::identity();
    const math::Vec3 cols[3] = {a, b, c};
    for (int j = 0; j < 3; ++j) {
        m(0, j) = cols[j].x;
        m(1, j) = cols[j].y;
        m(2, j) = cols[j].z;
    }
    return m;
}

inline math::Vec3 column(const math::Mat3& m, int c) { return {m(0, c), m(1, c), m(2, c)}; }

// Box with centre, orthonormal axes (columns: local x, y, z in the frame) and half extents.
struct OrientedBox {
    math::Vec3 center;
    math::Mat3 axes = math::Mat3::identity();
    math::Vec3 half_extents;
};

struct Penetration {
    bool inside = false;
    double depth_m = 0.0;  // distance to the nearest face
    math::Vec3 normal;     // outward normal of that face
};

Penetration point_in_box(const OrientedBox& box, const math::Vec3& point);
double distance_to_box(const OrientedBox& box, const math::Vec3& point);  // 0 inside

// Solid ring around the vertical axis through the origin: inner..outer radius, from z = 0 to height.
struct Annulus {
    double inner_radius_m = 0.0;
    double outer_radius_m = 0.0;
    double height_m = 0.0;
};

double distance_to_annulus(const Annulus& ring, const math::Vec3& point);  // 0 inside

// Capsule = segment a-b swept by a sphere of `radius_m`; tested by sampling the segment (see the .cpp).
bool capsule_overlaps_box(const math::Vec3& a, const math::Vec3& b, double radius_m, const OrientedBox& box);
bool capsule_overlaps_annulus(const math::Vec3& a, const math::Vec3& b, double radius_m, const Annulus& ring);

}  // namespace starsim::contact
```

`core/src/contact/primitives.cpp`:

```cpp
#include "starsim/contact/primitives.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace starsim::contact {

using math::Vec3;

namespace {

// Segment samples every half radius: the sampled clearance overestimates the true one by at most
// r * (1 - sqrt(1 - 1/16)) = 3 % of the radius.
constexpr double sample_spacing_per_radius = 0.5;

template <class Distance>
bool capsule_overlaps(const Vec3& a, const Vec3& b, double radius_m, Distance distance) {
    const double length = norm(b - a);
    const int samples = std::max(2, static_cast<int>(std::ceil(length / (sample_spacing_per_radius * radius_m))) + 1);
    for (int i = 0; i < samples; ++i) {
        const Vec3 p = a + (b - a) * (static_cast<double>(i) / (samples - 1));
        if (distance(p) < radius_m) return true;
    }
    return false;
}

}  // namespace

Penetration point_in_box(const OrientedBox& box, const Vec3& point) {
    const Vec3 local = transpose(box.axes) * (point - box.center);
    const double l[3] = {local.x, local.y, local.z};
    const double h[3] = {box.half_extents.x, box.half_extents.y, box.half_extents.z};
    double best = std::numeric_limits<double>::infinity();
    int axis = 0;
    for (int i = 0; i < 3; ++i) {
        const double depth = h[i] - std::abs(l[i]);
        if (depth < 0.0) return {};
        if (depth < best) {
            best = depth;
            axis = i;
        }
    }
    return {true, best, column(box.axes, axis) * (l[axis] >= 0.0 ? 1.0 : -1.0)};
}

double distance_to_box(const OrientedBox& box, const Vec3& point) {
    const Vec3 local = transpose(box.axes) * (point - box.center);
    const Vec3 outside{std::max(std::abs(local.x) - box.half_extents.x, 0.0),
                       std::max(std::abs(local.y) - box.half_extents.y, 0.0),
                       std::max(std::abs(local.z) - box.half_extents.z, 0.0)};
    return norm(outside);
}

double distance_to_annulus(const Annulus& ring, const Vec3& point) {
    const double rho = std::hypot(point.x, point.y);
    const double dr = std::max({ring.inner_radius_m - rho, rho - ring.outer_radius_m, 0.0});
    const double dz = std::max({point.z - ring.height_m, -point.z, 0.0});
    return std::hypot(dr, dz);
}

bool capsule_overlaps_box(const Vec3& a, const Vec3& b, double radius_m, const OrientedBox& box) {
    return capsule_overlaps(a, b, radius_m, [&](const Vec3& p) { return distance_to_box(box, p); });
}

bool capsule_overlaps_annulus(const Vec3& a, const Vec3& b, double radius_m, const Annulus& ring) {
    return capsule_overlaps(a, b, radius_m, [&](const Vec3& p) { return distance_to_annulus(ring, p); });
}

}  // namespace starsim::contact
```

Add `src/contact/primitives.cpp` to `core/CMakeLists.txt`.

- [ ] **Step 4: Run tests** - `build/core/tests/starsim_core_tests.exe "[contact]"` after the build. Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/contact core/src/contact core/tests/test_contact_primitives.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat: contact primitives (boxes, launch mount ring, capsules)"
```

---

### Task 3: Tower geometry

**Files:**
- Create: `core/include/starsim/contact/tower_geometry.hpp`, `core/src/contact/tower_geometry.cpp`, `core/tests/test_tower_geometry.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `config::TowerConfig`, `load_launch_site` (Task 1), primitives (Task 2), `frames::geodetic_to_fixed`.
- Produces: `class contact::TowerGeometry` with `TowerGeometry(const config::TowerConfig&, double latitude_rad, double longitude_rad, double height_m)`, `config()`, `to_itrf(site)`, `to_site(itrf)`, `site_to_itrf()` (Mat3, columns east/north/up), `origin_itrf_m()`, `tower_direction()`, `across()` (site, horizontal units, `across = up x tower_direction`), `tower_box()`, `mount()` (Annulus), `hinge(int side)`, `arm_top_height_m(double carriage_offset_m)`, `arm_direction(int side, double opening_rad)`, `arm_box(int side, double opening_rad, double carriage_offset_m)`. `side` is +1 (on the +across side) or -1.

- [ ] **Step 1: Write the failing test** - `core/tests/test_tower_geometry.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/tower_geometry.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::contact;
using Catch::Matchers::WithinAbs;
using math::Vec3;

namespace {

const config::LaunchSiteConfig& starbase() {
    static const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1");
    return site;
}

TowerGeometry starbase_tower() {
    const auto& s = starbase();
    return TowerGeometry(*s.tower, s.latitude_rad, s.longitude_rad, s.height_m);
}

constexpr Vec3 up{0.0, 0.0, 1.0};

}  // namespace

TEST_CASE("Site frame is east-north-up at the pad and round-trips through ITRF", "[contact]") {
    const auto tower = starbase_tower();
    const Vec3 p{12.3, -4.5, 67.8};
    const Vec3 back = tower.to_site(tower.to_itrf(p));
    CHECK(norm(back - p) < 1e-6);
    const auto above = frames::fixed_to_geodetic(tower.to_itrf({0.0, 0.0, 100.0}));
    CHECK_THAT(above.height_m, WithinAbs(starbase().height_m + 100.0, 1e-6));
    CHECK_THAT(above.latitude_rad, WithinAbs(starbase().latitude_rad, 1e-12));
    CHECK_THAT(dot(tower.across(), tower.tower_direction()), WithinAbs(0.0, 1e-15));
}

TEST_CASE("Closed arms hold the pins but not the hull", "[contact]") {
    const auto tower = starbase_tower();
    const double top = tower.arm_top_height_m(0.0);
    for (const int side : {1, -1}) {
        const auto arm = tower.arm_box(side, 0.0, 0.0);
        for (const double along : {-5.0, 0.0, 5.0}) {  // anywhere along the arms
            for (const double r : {4.9, 5.2, 5.45}) {   // pin (hull radius 4.5 + 1 m)
                CHECK(point_in_box(arm, tower.across() * (side * r) + tower.tower_direction() * along + up * (top - 0.05)).inside);
            }
            CHECK_FALSE(point_in_box(arm, tower.across() * (side * 4.5) + tower.tower_direction() * along + up * (top - 0.05)).inside);
        }
    }
}

TEST_CASE("Open arms leave room for the hull and the pins", "[contact]") {
    const auto tower = starbase_tower();
    const double open = tower.config().arm_open_rad;
    const double z = tower.arm_top_height_m(0.0) - 0.5;
    for (int deg = 0; deg < 360; deg += 10) {
        const double a = deg * constants::deg_to_rad;
        for (const double r : {4.5, 5.5}) {
            const Vec3 p{r * std::cos(a), r * std::sin(a), z};
            CHECK_FALSE(point_in_box(tower.arm_box(1, open, 0.0), p).inside);
            CHECK_FALSE(point_in_box(tower.arm_box(-1, open, 0.0), p).inside);
        }
    }
}

TEST_CASE("Tower and mount stay clear of a booster on the catch axis", "[contact]") {
    const auto tower = starbase_tower();
    const auto& c = tower.config();
    CHECK(point_in_box(tower.tower_box(), tower.tower_direction() * c.distance_m + up * 50.0).inside);
    CHECK_FALSE(capsule_overlaps_box({0.0, 0.0, c.mount_height_m}, {0.0, 0.0, 120.0}, 4.5, tower.tower_box()));
    CHECK_FALSE(capsule_overlaps_annulus({0.0, 0.0, c.mount_height_m}, {0.0, 0.0, 120.0}, 4.5, tower.mount()));
    CHECK(c.catch_height_m - 66.5 > c.mount_height_m);  // caught booster's bottom clears the mount (block 3 pins)
    // Carriage offset moves the arm down.
    CHECK_THAT(tower.arm_box(1, 0.0, -1.0).center.z - tower.arm_box(1, 0.0, 0.0).center.z, WithinAbs(-1.0, 1e-12));
}
```

- [ ] **Step 2: Run to verify it fails** - build. Expected: compile error, `tower_geometry.hpp` not found.

- [ ] **Step 3: Implement** - `core/include/starsim/contact/tower_geometry.hpp`

```cpp
#pragma once

#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/primitives.hpp"

namespace starsim::contact {

// Catch tower in the site frame: origin at the pad centre on the ground, x east, y north, z up (geodetic
// vertical); the catch axis is the vertical through the origin. Closed arms are parallel, pointing from the
// tower over the pad, their inner edges `arm_closed_gap_m / 2` from the catch axis; an arm swings outward
// about a vertical hinge on the tower face.
class TowerGeometry {
public:
    TowerGeometry(const config::TowerConfig& config, double latitude_rad, double longitude_rad, double height_m);

    const config::TowerConfig& config() const { return config_; }
    math::Vec3 to_itrf(const math::Vec3& site_m) const { return origin_itrf_m_ + site_to_itrf_ * site_m; }
    math::Vec3 to_site(const math::Vec3& itrf_m) const { return transpose(site_to_itrf_) * (itrf_m - origin_itrf_m_); }
    const math::Mat3& site_to_itrf() const { return site_to_itrf_; }  // columns: east, north, up
    const math::Vec3& origin_itrf_m() const { return origin_itrf_m_; }
    const math::Vec3& tower_direction() const { return toward_tower_; }  // pad -> tower, horizontal unit
    const math::Vec3& across() const { return across_; }                 // up x tower_direction

    OrientedBox tower_box() const;
    Annulus mount() const;
    // Hinge of arm `side` (+1: on the +across side, -1: opposite) at the arm top height, carriage at rest.
    math::Vec3 hinge(int side) const;
    double arm_top_height_m(double carriage_offset_m) const { return config_.catch_height_m + carriage_offset_m; }
    // Horizontal unit from the hinge towards the tip; opening 0 = closed.
    math::Vec3 arm_direction(int side, double opening_rad) const;
    OrientedBox arm_box(int side, double opening_rad, double carriage_offset_m) const;

private:
    config::TowerConfig config_;
    math::Mat3 site_to_itrf_;
    math::Vec3 origin_itrf_m_;
    math::Vec3 toward_tower_;
    math::Vec3 across_;
};

}  // namespace starsim::contact
```

`core/src/contact/tower_geometry.cpp`:

```cpp
#include "starsim/contact/tower_geometry.hpp"

#include <cmath>

#include "starsim/frames/geodetic.hpp"

namespace starsim::contact {

using math::Vec3;

namespace {
constexpr Vec3 up{0.0, 0.0, 1.0};
}

TowerGeometry::TowerGeometry(const config::TowerConfig& config, double latitude_rad, double longitude_rad,
                             double height_m)
    : config_(config) {
    const double sl = std::sin(latitude_rad), cl = std::cos(latitude_rad);
    const double so = std::sin(longitude_rad), co = std::cos(longitude_rad);
    site_to_itrf_ = from_columns({-so, co, 0.0}, {-sl * co, -sl * so, cl}, {cl * co, cl * so, sl});
    origin_itrf_m_ = frames::geodetic_to_fixed({latitude_rad, longitude_rad, height_m});
    toward_tower_ = {std::sin(config.azimuth_rad), std::cos(config.azimuth_rad), 0.0};
    across_ = cross(up, toward_tower_);
}

OrientedBox TowerGeometry::tower_box() const {
    return {toward_tower_ * config_.distance_m + up * (0.5 * config_.height_m), from_columns(toward_tower_, across_, up),
            {0.5 * config_.width_m, 0.5 * config_.width_m, 0.5 * config_.height_m}};
}

Annulus TowerGeometry::mount() const {
    return {config_.mount_inner_radius_m, config_.mount_outer_radius_m, config_.mount_height_m};
}

Vec3 TowerGeometry::hinge(int side) const {
    const double lateral = 0.5 * config_.arm_closed_gap_m + 0.5 * config_.arm_width_m;
    return toward_tower_ * (config_.distance_m - 0.5 * config_.width_m) + across_ * (side * lateral) +
           up * config_.catch_height_m;
}

Vec3 TowerGeometry::arm_direction(int side, double opening_rad) const {
    return -toward_tower_ * std::cos(opening_rad) + across_ * (side * std::sin(opening_rad));
}

OrientedBox TowerGeometry::arm_box(int side, double opening_rad, double carriage_offset_m) const {
    const Vec3 u = arm_direction(side, opening_rad);
    const Vec3 center = hinge(side) + u * (0.5 * config_.arm_length_m) +
                        up * (carriage_offset_m - 0.5 * config_.arm_thickness_m);
    return {center, from_columns(u, cross(up, u), up),
            {0.5 * config_.arm_length_m, 0.5 * config_.arm_width_m, 0.5 * config_.arm_thickness_m}};
}

}  // namespace starsim::contact
```

Add `src/contact/tower_geometry.cpp` to `core/CMakeLists.txt`.

- [ ] **Step 4: Run tests** - `"[contact]"`. Expected: PASS.

- [ ] **Step 5: Commit** - `git commit -m "feat: catch tower geometry in the launch site frame"`

---

### Task 4: Catch arms and carriage

**Files:**
- Create: `core/include/starsim/contact/catch_arms.hpp`, `core/src/contact/catch_arms.cpp`, `core/tests/test_catch_arms.cpp`
- Modify: CMake lists

**Interfaces:**
- Consumes: `config::TowerConfig`.
- Produces: `class contact::CatchArms` with `explicit CatchArms(const config::TowerConfig&)` (open, carriage at rest), `command_close(bool)`, `close_commanded()`, `set_closed()` (instant, for tests), `step(double dt_s, double hull_push_n, double vertical_load_n)`, `opening_rad()`, `opening_rate_rad_s()`, `carriage_offset_m()`, `carriage_velocity_mps()`, `closed()`, `stalled()`, `hit_hard_stop()`, `closing_time_s()`.

- [ ] **Step 1: Write the failing test** - `core/tests/test_catch_arms.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/catch_arms.hpp"

using namespace starsim;
using Catch::Matchers::WithinAbs;

namespace {

config::TowerConfig tower() { return *config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1").tower; }

void run(contact::CatchArms& arms, double seconds, double push_n, double load_n) {
    for (int i = 0; i < static_cast<int>(seconds / 0.005 + 0.5); ++i) arms.step(0.005, push_n, load_n);
}

}  // namespace

TEST_CASE("Arms close at their rate limit", "[contact]") {
    const auto config = tower();
    contact::CatchArms arms(config);
    CHECK_THAT(arms.opening_rad(), WithinAbs(config.arm_open_rad, 1e-15));
    CHECK_THAT(arms.closing_time_s(), WithinAbs(4.0, 1e-12));
    arms.command_close(true);
    run(arms, 2.0, 0.0, 0.0);
    CHECK_THAT(arms.opening_rad(), WithinAbs(0.5 * config.arm_open_rad, 1e-9));
    CHECK_THAT(arms.opening_rate_rad_s(), WithinAbs(-config.arm_rate_rad_s, 1e-9));
    run(arms, 2.1, 0.0, 0.0);
    CHECK(arms.closed());
    CHECK(arms.opening_rad() == 0.0);
}

TEST_CASE("Arms stall when the hull pushes back", "[contact]") {
    const auto config = tower();
    contact::CatchArms arms(config);
    arms.command_close(true);
    run(arms, 1.0, 0.0, 0.0);
    const double before = arms.opening_rad();
    run(arms, 1.0, 2.0 * config.arm_force_limit_n, 0.0);
    CHECK(arms.stalled());
    CHECK(arms.opening_rad() == before);
}

TEST_CASE("Carriage shock absorbers carry a static load and stop at the end of their stroke", "[contact]") {
    const auto config = tower();
    contact::CatchArms arms(config);
    run(arms, 30.0, 0.0, 1.0e6);
    CHECK_THAT(arms.carriage_offset_m(), WithinAbs(-1.0e6 / config.carriage_stiffness_n_m, 1e-3));
    CHECK_FALSE(arms.hit_hard_stop());
    contact::CatchArms overloaded(config);
    run(overloaded, 5.0, 0.0, 5.0e7);
    CHECK(overloaded.hit_hard_stop());
    CHECK(overloaded.carriage_offset_m() == -config.carriage_stroke_m);
}

TEST_CASE("Arms can be set closed instantly", "[contact]") {
    contact::CatchArms arms(tower());
    arms.set_closed();
    CHECK(arms.closed());
    CHECK(arms.close_commanded());
}
```

- [ ] **Step 2: Run to verify it fails** - build. Expected: compile error.

- [ ] **Step 3: Implement** - `core/include/starsim/contact/catch_arms.hpp`

```cpp
#pragma once

#include "starsim/config/launch_site_config.hpp"

namespace starsim::contact {

// The two catch arms (moving together) on their carriage. Opening angle: rate limited towards the command;
// closing stalls while the hull pushes back harder than the actuators allow. Carriage: mass on shock
// absorbers about its rest position (its weight is carried by the spring preload), with a hard stop at the
// end of the stroke.
class CatchArms {
public:
    explicit CatchArms(const config::TowerConfig& config);

    void command_close(bool close) { close_ = close; }
    bool close_commanded() const { return close_; }
    void set_closed();  // instant (scenario setup)
    // `hull_push_n`: force of the body against the closing arms; `vertical_load_n`: downward force of the
    // body on the arms.
    void step(double dt_s, double hull_push_n, double vertical_load_n);

    double opening_rad() const { return opening_rad_; }
    double opening_rate_rad_s() const { return opening_rate_rad_s_; }
    double carriage_offset_m() const { return carriage_offset_m_; }  // negative: below rest
    double carriage_velocity_mps() const { return carriage_velocity_mps_; }
    bool closed() const { return opening_rad_ <= 0.0; }
    bool stalled() const { return stalled_; }
    bool hit_hard_stop() const { return hit_hard_stop_; }  // reached the stop faster than it can take
    double closing_time_s() const { return config_.arm_open_rad / config_.arm_rate_rad_s; }

private:
    config::TowerConfig config_;
    bool close_ = false;
    double opening_rad_ = 0.0;
    double opening_rate_rad_s_ = 0.0;
    bool stalled_ = false;
    double carriage_offset_m_ = 0.0;
    double carriage_velocity_mps_ = 0.0;
    bool hit_hard_stop_ = false;
};

}  // namespace starsim::contact
```

`core/src/contact/catch_arms.cpp`:

```cpp
#include "starsim/contact/catch_arms.hpp"

#include <algorithm>

namespace starsim::contact {

namespace {
constexpr double hard_stop_speed_mps = 1.0;  // [E] faster at the end of the stroke: structural failure
}

CatchArms::CatchArms(const config::TowerConfig& config) : config_(config), opening_rad_(config.arm_open_rad) {}

void CatchArms::set_closed() {
    close_ = true;
    opening_rad_ = 0.0;
    opening_rate_rad_s_ = 0.0;
}

void CatchArms::step(double dt_s, double hull_push_n, double vertical_load_n) {
    const double previous = opening_rad_;
    const double target = close_ ? 0.0 : config_.arm_open_rad;
    stalled_ = close_ && hull_push_n > config_.arm_force_limit_n;
    if (!stalled_) {
        const double max_change = config_.arm_rate_rad_s * dt_s;
        opening_rad_ += std::clamp(target - opening_rad_, -max_change, max_change);
    }
    opening_rate_rad_s_ = (opening_rad_ - previous) / dt_s;

    // Semi-implicit Euler (stable for the stiff absorbers at 200 Hz).
    const double acceleration = (-config_.carriage_stiffness_n_m * carriage_offset_m_ -
                                 config_.carriage_damping_n_s_m * carriage_velocity_mps_ - vertical_load_n) /
                                config_.carriage_mass_kg;
    carriage_velocity_mps_ += acceleration * dt_s;
    carriage_offset_m_ += carriage_velocity_mps_ * dt_s;
    if (carriage_offset_m_ < -config_.carriage_stroke_m) {
        if (carriage_velocity_mps_ < -hard_stop_speed_mps) hit_hard_stop_ = true;
        carriage_offset_m_ = -config_.carriage_stroke_m;
        carriage_velocity_mps_ = std::max(carriage_velocity_mps_, 0.0);
    } else if (carriage_offset_m_ > config_.carriage_stroke_m) {
        carriage_offset_m_ = config_.carriage_stroke_m;
        carriage_velocity_mps_ = std::min(carriage_velocity_mps_, 0.0);
    }
}

}  // namespace starsim::contact
```

- [ ] **Step 4: Run tests** - `"[contact]"`. Expected: PASS.

- [ ] **Step 5: Commit** - `git commit -m "feat: catch arm actuators and carriage shock absorbers"`

---

### Task 5: Pin and hull contact

**Files:**
- Create: `core/include/starsim/contact/pin_contact.hpp`, `core/src/contact/pin_contact.cpp`, `core/tests/test_pin_contact.cpp`
- Modify: CMake lists

**Interfaces:**
- Consumes: primitives, `config::CatchPinConfig`.
- Produces:
  - `struct CatchBodyGeometry { std::vector<std::vector<Vec3>> pins; std::vector<Vec3> hull; double pin_station_m; double max_pin_load_n; }`
  - `CatchBodyGeometry make_catch_geometry(const config::CatchPinConfig&, double stage_base_station_m, double hull_radius_m, double arm_thickness_m)`
  - `struct SitePose { Vec3 cg_m; Vec3 velocity_mps; Mat3 body_to_site; Vec3 angular_velocity_radps; double cg_station_m; }`
  - `struct ArmContactState { OrientedBox box; Vec3 hinge_m; double yaw_rate_radps; double carriage_velocity_mps; Vec3 outward; }`
  - `struct CatchContact { Vec3 force_n; Vec3 torque_nm; std::vector<int> pin_points_in_contact; double max_pin_load_n; std::array<double,2> hull_push_n; double vertical_load_n; bool all_pins_in_contact() const; bool any_contact() const; }`
  - `CatchContact catch_contact(const CatchBodyGeometry&, const SitePose&, const std::array<ArmContactState, 2>&)`

- [ ] **Step 1: Write the failing test** - `core/tests/test_pin_contact.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "starsim/contact/pin_contact.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::contact;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using math::Vec3;

namespace {

// Arm: top surface at z = 0, 36 m long along x, 2 m wide; the second arm far away.
std::array<ArmContactState, 2> arms() {
    ArmContactState near{{{0.0, 0.0, -1.25}, math::Mat3::identity(), {18.0, 1.0, 1.25}}, {-18.0, 0.0, 0.0}, 0.0, 0.0,
                         {0.0, 1.0, 0.0}};
    ArmContactState far = near;
    far.box.center = {0.0, 100.0, -1.25};
    return {near, far};
}

// One pin of three points 1 cm inside the arm top; body frame = site frame, CG at the origin.
CatchBodyGeometry one_pin() { return {{{{0.0, 0.0, -0.01}, {0.0, 0.3, -0.01}, {0.0, 0.6, -0.01}}}, {}, 0.0, 5.0e6}; }

SitePose at_rest() { return {{}, {}, math::Mat3::identity(), {}, 0.0}; }

constexpr double pin_stiffness = 4.0e7;  // must match pin_contact.cpp

}  // namespace

TEST_CASE("Pin contact pushes the body out of the arm", "[contact]") {
    const auto c = catch_contact(one_pin(), at_rest(), arms());
    REQUIRE(c.pin_points_in_contact.size() == 1);
    CHECK(c.pin_points_in_contact[0] == 3);
    CHECK(c.all_pins_in_contact());
    CHECK_THAT(c.force_n.z, WithinRel(pin_stiffness * 0.01, 1e-12));
    CHECK_THAT(c.force_n.x, WithinAbs(0.0, 1e-9));
    CHECK_THAT(c.vertical_load_n, WithinRel(pin_stiffness * 0.01, 1e-12));
    CHECK_THAT(c.max_pin_load_n, WithinRel(pin_stiffness * 0.01, 1e-12));
    // Torque about the CG: r x F summed over the points (y = 0, 0.3, 0.6).
    CHECK_THAT(c.torque_nm.x, WithinRel(0.9 * pin_stiffness * 0.01 / 3.0, 1e-9));
}

TEST_CASE("Contact never pulls and friction opposes sliding", "[contact]") {
    SitePose leaving = at_rest();
    leaving.velocity_mps = {0.0, 0.0, 1.0};  // separating fast: damping exceeds the spring force
    CHECK(norm(catch_contact(one_pin(), leaving, arms()).force_n) == 0.0);

    SitePose sliding = at_rest();
    sliding.velocity_mps = {1.0, 0.0, 0.0};
    const auto c = catch_contact(one_pin(), sliding, arms());
    CHECK_THAT(c.force_n.x, WithinRel(-0.3 * pin_stiffness * 0.01, 1e-9));  // mu = 0.3
}

TEST_CASE("No contact above the arm", "[contact]") {
    SitePose above = at_rest();
    above.cg_m = {0.0, 0.0, 0.5};
    const auto c = catch_contact(one_pin(), above, arms());
    CHECK_FALSE(c.any_contact());
    CHECK(c.pin_points_in_contact[0] == 0);
}

TEST_CASE("Hull against the inner face pushes the arm outward", "[contact]") {
    const CatchBodyGeometry hull_only{{}, {{0.0, -0.95, -1.0}}, 0.0, 5.0e6};
    const auto c = catch_contact(hull_only, at_rest(), arms());
    CHECK(c.force_n.y < 0.0);         // body pushed towards -y
    CHECK(c.hull_push_n[0] > 0.0);    // arm pushed along its outward direction (+y)
}

TEST_CASE("Catch geometry from the pin config", "[contact]") {
    config::CatchPinConfig pins;
    pins.azimuths_rad = {90.0 * constants::deg_to_rad, 270.0 * constants::deg_to_rad};
    pins.station_m = 65.5;
    pins.protrusion_m = 1.0;
    pins.max_load_n = 5.0e6;
    const auto g = make_catch_geometry(pins, 0.0, 4.5, 2.5);
    REQUIRE(g.pins.size() == 2);
    REQUIRE(g.pins[0].size() == 3);
    CHECK_THAT(g.pin_station_m, WithinAbs(65.5, 1e-12));
    CHECK_THAT(g.pins[0][0].x, WithinAbs(65.5, 1e-12));
    CHECK_THAT(g.pins[0][1].z, WithinAbs(5.0, 1e-12));   // azimuth 90 deg: body +z, mid-pin radius 4.5 + 0.5
    CHECK_THAT(g.pins[1][1].z, WithinAbs(-5.0, 1e-12));
    CHECK(g.hull.size() == 16);
    CHECK_THAT(g.hull[0].x, WithinAbs(65.5 - 1.25, 1e-12));
    CHECK_THAT(std::hypot(g.hull[3].y, g.hull[3].z), WithinAbs(4.5, 1e-12));
    CHECK(make_catch_geometry({}, 0.0, 4.5, 2.5).pins.empty());
}
```

- [ ] **Step 2: Run to verify it fails** - build. Expected: compile error.

- [ ] **Step 3: Implement** - `core/include/starsim/contact/pin_contact.hpp`

```cpp
#pragma once

#include <array>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/primitives.hpp"

namespace starsim::contact {

// Contact sample points of a catchable body in its body frame (x along the axis from the vehicle origin).
struct CatchBodyGeometry {
    std::vector<std::vector<math::Vec3>> pins;  // per pin: points along the pin axis
    std::vector<math::Vec3> hull;               // hull ring at the height of the arms' inner faces
    double pin_station_m = 0.0;                 // vehicle frame
    double max_pin_load_n = 0.0;
};

// Empty if `pins` lists none. The hull ring sits half an arm thickness below the pins.
CatchBodyGeometry make_catch_geometry(const config::CatchPinConfig& pins, double stage_base_station_m,
                                      double hull_radius_m, double arm_thickness_m);

// Body pose in the site frame, relative to the ground.
struct SitePose {
    math::Vec3 cg_m;
    math::Vec3 velocity_mps;
    math::Mat3 body_to_site = math::Mat3::identity();  // columns: body x, y, z
    math::Vec3 angular_velocity_radps;                   // site frame
    double cg_station_m = 0.0;                           // vehicle frame
};

// One arm as the contact sees it.
struct ArmContactState {
    OrientedBox box;
    math::Vec3 hinge_m;
    double yaw_rate_radps = 0.0;  // rotation of the arm about the site vertical
    double carriage_velocity_mps = 0.0;
    math::Vec3 outward;           // horizontal unit away from the catch axis
};

struct CatchContact {
    math::Vec3 force_n;                       // on the body, site frame
    math::Vec3 torque_nm;                     // about the CG, site frame
    std::vector<int> pin_points_in_contact;   // per pin
    double max_pin_load_n = 0.0;
    std::array<double, 2> hull_push_n{};      // per arm: push against closing
    double vertical_load_n = 0.0;             // downward load on the arms

    bool all_pins_in_contact() const;
    bool any_contact() const;
};

// Penalty contact of the sample points with the arm boxes: normal force k d - c v_n (never pulling) and
// regularised Coulomb friction along the slip.
CatchContact catch_contact(const CatchBodyGeometry& body, const SitePose& pose, const std::array<ArmContactState, 2>& arms);

}  // namespace starsim::contact
```

`core/src/contact/pin_contact.cpp`:

```cpp
#include "starsim/contact/pin_contact.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::contact {

using math::Vec3;

namespace {

// Contact parameters, tuned for the 200 Hz RK4 tick (contact period ~0.4 s for a 275 t booster on two pins).
constexpr double pin_stiffness_n_m = 4.0e7;    // [E] per pin: ~3.4 cm compression under a 275 t booster
constexpr double pin_damping_n_s_m = 1.0e6;    // [E] per pin: damping ratio ~0.2
constexpr double hull_stiffness_n_m = 4.0e7;   // [E] whole hull ring
constexpr double hull_damping_n_s_m = 1.0e6;   // [E]
constexpr double friction_coefficient = 0.3;   // [E] dry steel on steel
constexpr double friction_regularization_mps = 0.01;  // below: friction linear in the slip (no stick-slip chatter)
constexpr int pin_samples = 3;
constexpr int hull_samples = 16;

Vec3 radial(double azimuth_rad) { return {0.0, std::cos(azimuth_rad), std::sin(azimuth_rad)}; }

}  // namespace

bool CatchContact::all_pins_in_contact() const {
    return !pin_points_in_contact.empty() &&
           std::all_of(pin_points_in_contact.begin(), pin_points_in_contact.end(), [](int n) { return n > 0; });
}

bool CatchContact::any_contact() const { return norm(force_n) > 0.0; }

CatchBodyGeometry make_catch_geometry(const config::CatchPinConfig& pins, double stage_base_station_m,
                                      double hull_radius_m, double arm_thickness_m) {
    CatchBodyGeometry g;
    if (pins.azimuths_rad.empty()) return g;
    g.pin_station_m = stage_base_station_m + pins.station_m;
    g.max_pin_load_n = pins.max_load_n;
    for (const double azimuth : pins.azimuths_rad) {
        std::vector<Vec3> points;
        for (int k = 0; k < pin_samples; ++k) {
            const double r = hull_radius_m + pins.protrusion_m * (k + 0.5) / pin_samples;
            points.push_back(Vec3{g.pin_station_m, 0.0, 0.0} + radial(azimuth) * r);
        }
        g.pins.push_back(points);
    }
    const double hull_station = g.pin_station_m - 0.5 * arm_thickness_m;
    for (int k = 0; k < hull_samples; ++k) {
        g.hull.push_back(Vec3{hull_station, 0.0, 0.0} + radial(constants::two_pi * k / hull_samples) * hull_radius_m);
    }
    return g;
}

CatchContact catch_contact(const CatchBodyGeometry& body, const SitePose& pose, const std::array<ArmContactState, 2>& arms) {
    CatchContact result;
    result.pin_points_in_contact.assign(body.pins.size(), 0);
    auto site_point = [&](const Vec3& b) { return pose.cg_m + pose.body_to_site * (b - Vec3{pose.cg_station_m, 0.0, 0.0}); };
    // Contact force of one sample point on arm `i` (zero if not touching).
    auto force_at = [&](const Vec3& p, double k, double c, const ArmContactState& arm) -> Vec3 {
        const auto pen = point_in_box(arm.box, p);
        if (!pen.inside) return {};
        const Vec3 body_velocity = pose.velocity_mps + cross(pose.angular_velocity_radps, p - pose.cg_m);
        const Vec3 arm_velocity = Vec3{0.0, 0.0, arm.carriage_velocity_mps} +
                                  cross(Vec3{0.0, 0.0, arm.yaw_rate_radps}, p - arm.hinge_m);
        const Vec3 v_rel = body_velocity - arm_velocity;
        const double v_n = dot(v_rel, pen.normal);
        const double f_n = k * pen.depth_m - c * v_n;
        if (f_n <= 0.0) return {};
        const Vec3 slip = v_rel - pen.normal * v_n;
        return pen.normal * f_n - slip * (friction_coefficient * f_n / std::max(norm(slip), friction_regularization_mps));
    };
    auto apply = [&](const Vec3& p, const Vec3& f, size_t arm_index) {
        result.force_n += f;
        result.torque_nm += cross(p - pose.cg_m, f);
        result.vertical_load_n += f.z;
        result.hull_push_n[arm_index] += std::max(dot(-f, arms[arm_index].outward), 0.0);
    };
    for (size_t j = 0; j < body.pins.size(); ++j) {
        double load = 0.0;
        for (const auto& b : body.pins[j]) {
            const Vec3 p = site_point(b);
            for (size_t i = 0; i < arms.size(); ++i) {
                const Vec3 f = force_at(p, pin_stiffness_n_m / pin_samples, pin_damping_n_s_m / pin_samples, arms[i]);
                if (norm(f) == 0.0) continue;
                ++result.pin_points_in_contact[j];
                load += norm(f);
                apply(p, f, i);
            }
        }
        result.max_pin_load_n = std::max(result.max_pin_load_n, load);
    }
    for (const auto& b : body.hull) {
        const Vec3 p = site_point(b);
        for (size_t i = 0; i < arms.size(); ++i) {
            const Vec3 f = force_at(p, hull_stiffness_n_m / hull_samples, hull_damping_n_s_m / hull_samples, arms[i]);
            if (norm(f) > 0.0) apply(p, f, i);
        }
    }
    return result;
}

}  // namespace starsim::contact
```

- [ ] **Step 4: Run tests** - `"[contact]"`. Expected: PASS.

- [ ] **Step 5: Commit** - `git commit -m "feat: penalty contact of catch pins and hull with the tower arms"`

---

### Task 6: Catch trigger and tower collision

**Files:**
- Create: `core/include/starsim/contact/catch_trigger.hpp`, `core/src/contact/catch_trigger.cpp`, `core/include/starsim/contact/tower_collision.hpp`, `core/src/contact/tower_collision.cpp`, `core/tests/test_catch_trigger.cpp`
- Modify: CMake lists

**Interfaces:**
- Produces: `struct CatchTrigger { double max_axis_offset_m = 3.0; double margin_s = 1.0; double hover_height_m = 2.0; }`, `bool should_close_arms(const CatchTrigger&, double pin_height_above_arms_m, double axis_offset_m, double sink_rate_mps, double closing_time_s)`, `bool hits_tower(const TowerGeometry&, const Vec3& bottom_m, const Vec3& top_m, double hull_radius_m)`.

- [ ] **Step 1: Write the failing test** - `core/tests/test_catch_trigger.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/catch_trigger.hpp"
#include "starsim/contact/tower_collision.hpp"

using namespace starsim;
using namespace starsim::contact;
using math::Vec3;

TEST_CASE("The tower closes the arms just in time", "[contact]") {
    const CatchTrigger t;
    CHECK(should_close_arms(t, 30.0, 0.5, 10.0, 4.0));         // 3 s to contact <= 4 + 1 s
    CHECK_FALSE(should_close_arms(t, 60.0, 0.5, 10.0, 4.0));   // 6 s
    CHECK_FALSE(should_close_arms(t, 30.0, 4.0, 10.0, 4.0));   // too far off the catch axis
    CHECK(should_close_arms(t, 1.0, 0.5, 0.0, 4.0));           // hovering just above the arms
    CHECK_FALSE(should_close_arms(t, 20.0, 0.5, -1.0, 4.0));   // climbing
    CHECK_FALSE(should_close_arms(t, -1.0, 0.5, 1.0, 4.0));    // pins already below the arms
}

TEST_CASE("Hull collisions with the tower and the launch mount", "[contact]") {
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1");
    const TowerGeometry tower(*site.tower, site.latitude_rad, site.longitude_rad, site.height_m);
    const Vec3 up{0.0, 0.0, 1.0};
    const double mount = site.tower->mount_height_m;
    CHECK_FALSE(hits_tower(tower, up * mount, up * (mount + 121.0), 4.5));        // stack on the mount
    CHECK_FALSE(hits_tower(tower, up * 29.5, up * 100.5, 4.5));                   // caught booster
    const Vec3 near_tower = tower.tower_direction() * 20.0;                       // hull reaches 24.5 m: face at 24 m
    CHECK(hits_tower(tower, near_tower + up * 50.0, near_tower + up * 121.0, 4.5));
    CHECK(hits_tower(tower, Vec3{3.0, 0.0, 10.0}, Vec3{3.0, 0.0, 81.0}, 4.5));     // fell into the mount
}
```

- [ ] **Step 2: Run to verify it fails** - build. Expected: compile error.

- [ ] **Step 3: Implement**

`core/include/starsim/contact/catch_trigger.hpp`:

```cpp
#pragma once

namespace starsim::contact {

// Automatic tower: close the arms when the pins will reach the arm tops within the closing time (plus a
// margin) and the booster axis is near the catch axis, or when the pins hover just above the arms.
struct CatchTrigger {
    double max_axis_offset_m = 3.0;  // [E] farther off the arms cannot catch the pins
    double margin_s = 1.0;           // [E] arms closed this long before the pins arrive
    double hover_height_m = 2.0;     // [E]
};

bool should_close_arms(const CatchTrigger& trigger, double pin_height_above_arms_m, double axis_offset_m,
                       double sink_rate_mps, double closing_time_s);

}  // namespace starsim::contact
```

`core/src/contact/catch_trigger.cpp`:

```cpp
#include "starsim/contact/catch_trigger.hpp"

namespace starsim::contact {

bool should_close_arms(const CatchTrigger& trigger, double pin_height_above_arms_m, double axis_offset_m,
                       double sink_rate_mps, double closing_time_s) {
    if (axis_offset_m > trigger.max_axis_offset_m || pin_height_above_arms_m < 0.0) return false;
    if (pin_height_above_arms_m <= trigger.hover_height_m) return true;
    return sink_rate_mps > 0.0 && pin_height_above_arms_m / sink_rate_mps <= closing_time_s + trigger.margin_s;
}

}  // namespace starsim::contact
```

`core/include/starsim/contact/tower_collision.hpp`:

```cpp
#pragma once

#include "starsim/contact/tower_geometry.hpp"

namespace starsim::contact {

// True if a vehicle hull (capsule from `bottom_m` to `top_m`, site frame) touches the tower structure or the
// launch mount. The arms are not part of this test: they act through the contact model.
bool hits_tower(const TowerGeometry& tower, const math::Vec3& bottom_m, const math::Vec3& top_m, double hull_radius_m);

}  // namespace starsim::contact
```

`core/src/contact/tower_collision.cpp`:

```cpp
#include "starsim/contact/tower_collision.hpp"

namespace starsim::contact {

bool hits_tower(const TowerGeometry& tower, const math::Vec3& bottom_m, const math::Vec3& top_m, double hull_radius_m) {
    return capsule_overlaps_box(bottom_m, top_m, hull_radius_m, tower.tower_box()) ||
           capsule_overlaps_annulus(bottom_m, top_m, hull_radius_m, tower.mount());
}

}  // namespace starsim::contact
```

- [ ] **Step 4: Run tests** - `"[contact]"`. Expected: PASS (the stack-on-mount case: bottom point on the axis is 5.5 m from the ring > 4.5 m).

- [ ] **Step 5: Commit** - `git commit -m "feat: automatic catch trigger and tower collision test"`

---

### Task 7: FlightBody external wrench and catch hold

**Files:**
- Modify: `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp`, `core/tests/test_flight_body.cpp`

**Interfaces:**
- Produces: `FlightBody::apply_external_wrench(const Vec3& force_gcrf_n, const Vec3& torque_body_nm)` (applied during the next `step()` only), `FlightBody::hold_fixed_to_earth(const frames::BodyOrientation&, time::Epoch)`, `FlightBody::caught()`, `FlightBody::mark_impacted()`.

- [ ] **Step 1: Write the failing tests** - append to `core/tests/test_flight_body.cpp` (uses the file's `DeepSpace`, `ship_in_deep_space`, `RotatingEarth`, `t0`):

```cpp
TEST_CASE("An external wrench acts for one step", "[vehicle]") {
    DeepSpace space;
    auto body = ship_in_deep_space({});
    const double m = body->vehicle().mass_properties().mass_kg;
    const double iyy = body->vehicle().mass_properties().inertia_kg_m2.y;
    body->apply_external_wrench({1.0e6, 0.0, 0.0}, {0.0, 1.0e6, 0.0});
    body->step(space.env, t0, 0.1, {});
    CHECK(body->state().velocity_gcrf_mps.x == Catch::Approx(1.0e6 / m * 0.1).epsilon(1e-6));
    CHECK(body->state().angular_velocity_body_radps.y == Catch::Approx(1.0e6 / iyy * 0.1).epsilon(1e-6));
    const Vec3 v = body->state().velocity_gcrf_mps;
    body->step(space.env, t0 + 0.1, 0.1, {});
    CHECK(body->state().velocity_gcrf_mps.x == Catch::Approx(v.x).epsilon(1e-12));  // cleared after use
}

TEST_CASE("A caught body co-rotates with the Earth", "[vehicle]") {
    ZeroGravity gravity;
    environment::Us76Atmosphere atmosphere;
    RotatingEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
    RigidBodyState s;
    s.position_gcrf_m = {6.45e6, 0.0, 0.0};
    s.velocity_gcrf_mps = {0.0, 3.0, -2.0};
    auto body = ship_in_deep_space(s);
    body->hold_fixed_to_earth(earth, t0);
    CHECK(body->caught());
    for (int i = 0; i < 100; ++i) body->step(env, t0 + i, 1.0, {});
    const auto r = earth.fixed_to_gcrf(t0 + 100.0).rotation * Vec3{6.45e6, 0.0, 0.0};
    CHECK(norm(body->state().position_gcrf_m - r) < 1e-6);
    body->mark_impacted();
    CHECK(body->impacted());
}
```

- [ ] **Step 2: Run to verify it fails** - build. Expected: compile errors (`apply_external_wrench` not a member).

- [ ] **Step 3: Implement**

Header (`flight_body.hpp`), public section after `release_clamps()`:

```cpp
    // Contact wrench for the next step() only: force through the CG (GCRF) and torque (body axes).
    void apply_external_wrench(const math::Vec3& force_gcrf_n, const math::Vec3& torque_body_nm);
    // Fixes the body to the Earth where it is (caught by the tower): from now on it co-rotates with the Earth
    // like a clamped vehicle and is no longer integrated.
    void hold_fixed_to_earth(const frames::BodyOrientation& earth, time::Epoch epoch);
    bool caught() const { return caught_; }
    void mark_impacted() { impacted_ = true; }  // struck the tower or overloaded a catch pin
```

Private members: `math::Vec3 external_force_gcrf_n_; math::Vec3 external_torque_body_nm_; bool caught_ = false;`

`flight_body.cpp`:
- `TickContext`: add `Vec3 external_force_gcrf_n;` and `Vec3 external_torque_body_nm;` after `rcs_force_n`.
- `evaluate`: after `f.nongravitational_body_n += ctx.rcs_force_n;` add `f.moment_body_nm += ctx.external_torque_body_nm;`, and in the acceleration sum add `+ ctx.external_force_gcrf_n / f.mass.mass_kg`.
- `step`: replace `if (clamped_) {` with `if (clamped_ || caught_) {` and clear the wrench there before `return`; append `external_force_gcrf_n_, external_torque_body_nm_` to the `TickContext` initialiser; after the RK4 step add:

```cpp
    external_force_gcrf_n_ = {};
    external_torque_body_nm_ = {};
```
- `coast`: `if (resting_ || caught_) {` (follow the Earth); append `{}, {}` to its `TickContext` initialiser.
- New functions:

```cpp
void FlightBody::apply_external_wrench(const Vec3& force_gcrf_n, const Vec3& torque_body_nm) {
    external_force_gcrf_n_ += force_gcrf_n;
    external_torque_body_nm_ += torque_body_nm;
}

void FlightBody::hold_fixed_to_earth(const frames::BodyOrientation& earth, time::Epoch epoch) {
    const auto transform = earth.fixed_to_gcrf(epoch);
    const math::Mat3 to_fixed = transpose(transform.rotation);
    const double cg = vehicle_->mass_properties().cg_station_m;
    pad_itrf_m_ = to_fixed * (state_.position_gcrf_m - state_.attitude.rotate(Vec3{cg, 0.0, 0.0}));
    attitude_itrf_ = (Quat::from_matrix(to_fixed) * state_.attitude).normalized();
    caught_ = true;
    resting_ = false;
    follow_pad(earth, epoch);
}
```

- [ ] **Step 4: Run tests** - `build/core/tests/starsim_core_tests.exe "[vehicle]"`. Expected: PASS (all old `[vehicle]` cases too).

- [ ] **Step 5: Commit** - `git commit -m "feat: external contact wrench and catch hold for flight bodies"`

---

### Task 8: Tower in the flight session

**Files:**
- Create: `core/include/starsim/sim/catch_tower_sim.hpp`, `core/src/sim/catch_tower_sim.cpp`, `core/tests/test_catch_tower.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`, CMake lists

**Interfaces:**
- Consumes: Tasks 1-7.
- Produces:
  - `sim::LaunchSite` gains `std::optional<config::TowerConfig> tower;` (last member; brace init `{lat, lon, h}` unchanged) and `sim::LaunchSite sim::launch_site_from_config(const config::LaunchSiteConfig&)`.
  - `enum class sim::CatchPhase { Open, Closing, Caught, Missed }`, `const char* sim::to_string(CatchPhase)` (German: "offen", "schließt", "gefangen", "verfehlt").
  - `class sim::CatchTowerSim` with `CatchTowerSim(const config::TowerConfig&, double latitude_rad, double longitude_rad, double height_m)`, `geometry()`, `arms()`, `phase()`, `close_arms(bool instantly)`, `before_tick(const std::vector<vehicle::FlightBody*>&, const frames::BodyOrientation&, time::Epoch)`, `after_tick(const std::vector<vehicle::FlightBody*>&, const frames::BodyOrientation&, time::Epoch epoch_end, double dt_s)`, `in_contact(const vehicle::FlightBody&)`, `needs_ticks(const std::vector<vehicle::FlightBody*>&, const frames::BodyOrientation&, time::Epoch)`, `max_pin_load_n()`.
  - `FlightSession`: `set_towers(bool)`, `set_launch_site(const LaunchSite&)`, `place_body(const config::VehicleConfig&, const vehicle::RigidBodyState&, time::Epoch)`, `const CatchTowerSim* tower() const`, `CatchTowerSim* tower()`, `const LaunchSite& launch_site() const`.

- [ ] **Step 1: Write the failing scenario tests** - `core/tests/test_catch_tower.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/primitives.hpp"
#include "starsim/sim/catch_tower_sim.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const spice::SpiceEphemeris& ephemeris() { static const spice::SpiceEphemeris e; return e; }
const frames::BodyOrientation& earth() { static const auto e = spice::SpiceOrientation::earth(); return e; }

// Block 2 booster with 4 t of propellant left, upright over the catch axis, pins facing the arms. `pin_offset_m`
// (site frame) moves the pins' midpoint from the catch point (arm tops on the catch axis); velocity in the site
// frame, relative to the ground.
void place_booster(sim::FlightSession& session, time::Epoch epoch, const Vec3& pin_offset_m, const Vec3& velocity_mps) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block2");
    cfg.stages.resize(1);
    for (auto& tank : cfg.stages[0].tanks) tank.mass_kg = 2000.0;
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const auto& tower = session.tower()->geometry();
    const Vec3 up{0.0, 0.0, 1.0};
    const double azimuth = cfg.stages[0].catch_pins.azimuths_rad.front();
    // Body +y such that the first pin (azimuth from +y towards +z) points along +across.
    const Vec3 y = tower.across() * std::cos(azimuth) - cross(up, tower.across()) * std::sin(azimuth);
    const math::Mat3 body_to_site = contact::from_columns(up, y, cross(up, y));
    const double pin_station = cfg.stages[0].catch_pins.station_m;
    const Vec3 cg_site = up * tower.arm_top_height_m(0.0) + pin_offset_m - up * (pin_station - cg);
    const auto t = earth().fixed_to_gcrf(epoch);
    const Vec3 r_itrf = tower.to_itrf(cg_site);
    vehicle::RigidBodyState s;
    s.position_gcrf_m = t.position(r_itrf);
    s.velocity_gcrf_mps = t.velocity(r_itrf, tower.site_to_itrf() * velocity_mps);
    s.attitude = math::Quat::from_matrix(t.rotation * tower.site_to_itrf() * body_to_site).normalized();
    s.angular_velocity_body_radps = s.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    session.place_body(cfg, s, epoch);
    session.set_control_mode(sim::ControlMode::DirectGimbal);  // no stability augmentation: engines and RCS off
}

// FlightSession is not movable in practice (its environment refers to its own members): set it up in place.
void use_starbase(sim::FlightSession& session) {
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
}

time::Epoch start() {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    return time.from_utc({2026, 10, 12, 12, 0, 0.0});
}

// Advances in 0.1 s steps until `done` or `seconds`.
template <class Done>
void run(sim::FlightSession& session, time::Epoch t0, double seconds, Done done) {
    for (double t = 0.1; t <= seconds && !done(); t += 0.1) session.advance_to(t0 + t, 1000);
}

}  // namespace

TEST_CASE("Centred slow arrival: the booster is caught and hangs on the arms", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    session.tower()->close_arms(true);
    place_booster(session, t0, {0.0, 0.0, 0.02}, {0.0, 0.0, -1.0});
    const auto& booster = session.body(0);
    run(session, t0, 30.0, [&] { return booster.caught() || booster.impacted(); });
    INFO("max pin load " << session.tower()->max_pin_load_n());
    REQUIRE(booster.caught());
    CHECK(session.tower()->phase() == sim::CatchPhase::Caught);
    const auto at = [&](time::Epoch e) {
        return transpose(earth().fixed_to_gcrf(e).rotation) * booster.state().position_gcrf_m;
    };
    const auto caught_at = session.epoch();
    const Vec3 r0 = at(caught_at);
    session.advance_to(caught_at + 60.0, 100000);
    CHECK(norm(at(session.epoch()) - r0) < 1e-6);  // fixed in ITRF
}

TEST_CASE("Off-centre arrival: one pin misses and the booster falls", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    session.tower()->close_arms(true);
    const auto& tower = session.tower()->geometry();
    place_booster(session, t0, tower.across() * 1.5 + Vec3{0.0, 0.0, 2.0}, {0.0, 0.0, -1.0});
    const auto& booster = session.body(0);
    run(session, t0, 120.0, [&] { return booster.caught() || booster.impacted() || booster.resting(); });
    CHECK_FALSE(booster.caught());
    CHECK((booster.impacted() || booster.resting()));
    CHECK(session.tower()->phase() == sim::CatchPhase::Missed);
}

TEST_CASE("Fast arrival overloads the pins", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    session.tower()->close_arms(true);
    place_booster(session, t0, {0.0, 0.0, 0.02}, {0.0, 0.0, -8.0});
    const auto& booster = session.body(0);
    run(session, t0, 10.0, [&] { return booster.caught() || booster.impacted(); });
    CHECK(booster.impacted());
    CHECK(session.tower()->phase() == sim::CatchPhase::Missed);
}

TEST_CASE("Drifting into the tower is an impact", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    const auto& tower = session.tower()->geometry();
    place_booster(session, t0, tower.tower_direction() * 12.0 + Vec3{0.0, 0.0, 20.0}, tower.tower_direction() * 5.0);
    const auto& booster = session.body(0);
    run(session, t0, 10.0, [&] { return booster.impacted(); });
    CHECK(booster.impacted());
    CHECK_FALSE(booster.caught());
}
```

(`spice_adapters.hpp` provides `SpiceEphemeris`, `SpiceOrientation`, `SpiceTimeConverter` as in `test_booster_return.cpp`; copy its include lines if the names differ. Remove the unused `Scenario` struct if the compiler warns.) Add the file to `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Run to verify it fails** - build. Expected: compile error (`catch_tower_sim.hpp` not found).

- [ ] **Step 3: `CatchTowerSim`** - `core/include/starsim/sim/catch_tower_sim.hpp`

```cpp
#pragma once

#include <map>
#include <vector>

#include "starsim/contact/catch_arms.hpp"
#include "starsim/contact/catch_trigger.hpp"
#include "starsim/contact/pin_contact.hpp"
#include "starsim/contact/tower_geometry.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::sim {

enum class CatchPhase { Open, Closing, Caught, Missed };

const char* to_string(CatchPhase phase);  // German UI text

// The catch tower of the launch site during a flight: automatic arm control, contact forces on catchable
// bodies (single stages with catch pins), tower collisions of all bodies, and the catch itself.
class CatchTowerSim {
public:
    CatchTowerSim(const config::TowerConfig& config, double latitude_rad, double longitude_rad, double height_m);

    const contact::TowerGeometry& geometry() const { return geometry_; }
    const contact::CatchArms& arms() const { return arms_; }
    CatchPhase phase() const { return phase_; }
    void close_arms(bool instantly);

    // Contact wrenches for the coming tick (from the current states) and the automatic tower.
    void before_tick(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth, time::Epoch epoch);
    // Moves the arms over the tick, then judges collisions, pin overloads and the catch at its end.
    void after_tick(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                    time::Epoch epoch_end, double dt_s);
    bool in_contact(const vehicle::FlightBody& body) const;  // pins on the arms in the last tick
    // A body near the tower still moves on its own: fixed ticks are needed (no coast segments).
    bool needs_ticks(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                     time::Epoch epoch) const;
    double max_pin_load_n() const { return max_pin_load_n_; }  // largest so far (diagnostics)

private:
    struct Track {
        contact::CatchBodyGeometry geometry;
        contact::CatchContact contact;
        double at_rest_s = 0.0;
    };

    static bool catchable(const vehicle::FlightBody& body);
    contact::SitePose site_pose(const vehicle::FlightBody& body, const frames::FixedToInertial& t) const;
    std::array<contact::ArmContactState, 2> arm_states() const;
    Track& track(const vehicle::FlightBody& body);

    contact::TowerGeometry geometry_;
    contact::CatchArms arms_;
    contact::CatchTrigger trigger_;
    CatchPhase phase_ = CatchPhase::Open;
    std::map<const vehicle::FlightBody*, Track> tracks_;
    double hull_push_n_ = 0.0;
    double vertical_load_n_ = 0.0;
    double max_pin_load_n_ = 0.0;
};

}  // namespace starsim::sim
```

`core/src/sim/catch_tower_sim.cpp`:

```cpp
#include "starsim/sim/catch_tower_sim.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/contact/tower_collision.hpp"

namespace starsim::sim {

using math::Vec3;

namespace {

constexpr double rest_speed_mps = 0.1;      // caught: body and carriage this slow ...
constexpr double rest_rate_radps = 0.01;
constexpr double rest_time_s = 2.0;         // ... for this long, engines off, both pins on the arms
constexpr double missed_drop_m = 5.0;       // pins this far below the arm tops: the catch is lost
constexpr double tower_zone_m = 1000.0;     // bodies closer than this need fixed ticks

}  // namespace

const char* to_string(CatchPhase phase) {
    switch (phase) {
        case CatchPhase::Open: return "offen";
        case CatchPhase::Closing: return "schließt";
        case CatchPhase::Caught: return "gefangen";
        case CatchPhase::Missed: return "verfehlt";
    }
    return "";
}

CatchTowerSim::CatchTowerSim(const config::TowerConfig& config, double latitude_rad, double longitude_rad, double height_m)
    : geometry_(config, latitude_rad, longitude_rad, height_m), arms_(config) {}

void CatchTowerSim::close_arms(bool instantly) {
    if (instantly) arms_.set_closed();
    arms_.command_close(true);
    if (phase_ == CatchPhase::Open) phase_ = CatchPhase::Closing;
}

bool CatchTowerSim::catchable(const vehicle::FlightBody& body) {
    const auto& stages = body.vehicle().stages();
    return stages.size() == 1 && !stages.front().config.catch_pins.azimuths_rad.empty() && !body.clamped() &&
           !body.impacted() && !body.caught();
}

contact::SitePose CatchTowerSim::site_pose(const vehicle::FlightBody& body, const frames::FixedToInertial& t) const {
    const math::Mat3 to_fixed = transpose(t.rotation);
    const math::Mat3 to_site = transpose(geometry_.site_to_itrf()) * to_fixed;
    const Vec3 omega = vehicle::earth_rotation_vector(t);
    const auto& s = body.state();
    contact::SitePose p;
    p.cg_m = geometry_.to_site(to_fixed * s.position_gcrf_m);
    p.velocity_mps = to_site * (s.velocity_gcrf_mps - cross(omega, s.position_gcrf_m));
    p.body_to_site = to_site * s.attitude.to_matrix();
    p.angular_velocity_radps = to_site * (s.attitude.rotate(s.angular_velocity_body_radps) - omega);
    p.cg_station_m = body.vehicle().mass_properties().cg_station_m;
    return p;
}

std::array<contact::ArmContactState, 2> CatchTowerSim::arm_states() const {
    std::array<contact::ArmContactState, 2> states;
    for (int i = 0; i < 2; ++i) {
        const int side = i == 0 ? 1 : -1;
        auto& st = states[static_cast<size_t>(i)];
        st.box = geometry_.arm_box(side, arms_.opening_rad(), arms_.carriage_offset_m());
        st.hinge_m = geometry_.hinge(side) + Vec3{0.0, 0.0, arms_.carriage_offset_m()};
        st.yaw_rate_radps = -side * arms_.opening_rate_rad_s();  // arm direction turns by -side * dopening/dt
        st.carriage_velocity_mps = arms_.carriage_velocity_mps();
        st.outward = geometry_.across() * side;
    }
    return states;
}

CatchTowerSim::Track& CatchTowerSim::track(const vehicle::FlightBody& body) {
    auto it = tracks_.find(&body);
    if (it == tracks_.end()) {
        const auto& stage = body.vehicle().stages().front();
        Track t;
        t.geometry = contact::make_catch_geometry(stage.config.catch_pins, stage.base_station_m,
                                                  0.5 * body.vehicle().diameter_m(), geometry_.config().arm_thickness_m);
        it = tracks_.emplace(&body, t).first;
    }
    return it->second;
}

void CatchTowerSim::before_tick(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                                time::Epoch epoch) {
    const auto t = earth.fixed_to_gcrf(epoch);
    const auto arms = arm_states();
    const math::Mat3 site_to_gcrf = t.rotation * geometry_.site_to_itrf();
    hull_push_n_ = 0.0;
    vertical_load_n_ = 0.0;
    for (auto* body : bodies) {
        if (!catchable(*body)) continue;
        auto& tr = track(*body);
        const auto pose = site_pose(*body, t);
        tr.contact = contact::catch_contact(tr.geometry, pose, arms);
        if (tr.contact.any_contact()) {
            body->apply_external_wrench(site_to_gcrf * tr.contact.force_n,
                                        body->state().attitude.inverse_rotate(site_to_gcrf * tr.contact.torque_nm));
        }
        hull_push_n_ = std::max({hull_push_n_, tr.contact.hull_push_n[0], tr.contact.hull_push_n[1]});
        vertical_load_n_ += tr.contact.vertical_load_n;
        max_pin_load_n_ = std::max(max_pin_load_n_, tr.contact.max_pin_load_n);
        if (phase_ == CatchPhase::Open) {
            const Vec3 pin_axis = pose.cg_m + pose.body_to_site * Vec3{tr.geometry.pin_station_m - pose.cg_station_m, 0.0, 0.0};
            const double pin_height = pin_axis.z - geometry_.arm_top_height_m(arms_.carriage_offset_m());
            if (contact::should_close_arms(trigger_, pin_height, std::hypot(pose.cg_m.x, pose.cg_m.y),
                                           -pose.velocity_mps.z, arms_.closing_time_s())) {
                close_arms(false);
            }
        }
    }
}

void CatchTowerSim::after_tick(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                               time::Epoch epoch_end, double dt_s) {
    arms_.step(dt_s, hull_push_n_, vertical_load_n_);
    const auto t = earth.fixed_to_gcrf(epoch_end);
    for (auto* body : bodies) {
        if (body->clamped() || body->caught()) continue;
        const bool was_tracked = tracks_.count(body) > 0;
        if (!body->impacted()) {
            const auto pose = site_pose(*body, t);
            const Vec3 bottom = pose.cg_m + pose.body_to_site * Vec3{-pose.cg_station_m, 0.0, 0.0};
            const Vec3 top = pose.cg_m + pose.body_to_site * Vec3{body->vehicle().length_m() - pose.cg_station_m, 0.0, 0.0};
            if (contact::hits_tower(geometry_, bottom, top, 0.5 * body->vehicle().diameter_m())) body->mark_impacted();
            if (was_tracked) {
                auto& tr = tracks_.at(body);
                if (tr.contact.max_pin_load_n > tr.geometry.max_pin_load_n || arms_.hit_hard_stop()) body->mark_impacted();
                const Vec3 pin_axis = pose.cg_m + pose.body_to_site * Vec3{tr.geometry.pin_station_m - pose.cg_station_m, 0.0, 0.0};
                if (pin_axis.z < geometry_.arm_top_height_m(arms_.carriage_offset_m()) - missed_drop_m && phase_ != CatchPhase::Caught) {
                    phase_ = CatchPhase::Missed;
                }
                const bool at_rest = tr.contact.all_pins_in_contact() && !body->vehicle().any_engine_producing_thrust() &&
                                     norm(pose.velocity_mps) < rest_speed_mps &&
                                     norm(pose.angular_velocity_radps) < rest_rate_radps &&
                                     std::abs(arms_.carriage_velocity_mps()) < rest_speed_mps;
                tr.at_rest_s = at_rest ? tr.at_rest_s + dt_s : 0.0;
                if (tr.at_rest_s >= rest_time_s && !body->impacted()) {
                    body->hold_fixed_to_earth(earth, epoch_end);
                    phase_ = CatchPhase::Caught;
                }
            }
        }
        if (was_tracked && (body->impacted() || body->resting()) && phase_ != CatchPhase::Caught) phase_ = CatchPhase::Missed;
    }
}

bool CatchTowerSim::in_contact(const vehicle::FlightBody& body) const {
    const auto it = tracks_.find(&body);
    return it != tracks_.end() && it->second.contact.any_contact();
}

bool CatchTowerSim::needs_ticks(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                                time::Epoch epoch) const {
    const auto t = earth.fixed_to_gcrf(epoch);
    for (const auto* body : bodies) {
        if (body->clamped() || body->impacted() || body->caught() || body->resting()) continue;
        if (norm(geometry_.to_site(transpose(t.rotation) * body->state().position_gcrf_m)) < tower_zone_m) return true;
    }
    return false;
}

}  // namespace starsim::sim
```

(`Vehicle::any_engine_producing_thrust()`, `length_m()`, `diameter_m()` exist; `Quat::to_matrix()` gives body -> GCRF.)

- [ ] **Step 4: `FlightSession` integration**

`flight_session.hpp`: `#include <optional>`, `"starsim/config/launch_site_config.hpp"`, `"starsim/sim/catch_tower_sim.hpp"`. `LaunchSite`:

```cpp
struct LaunchSite {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    std::optional<config::TowerConfig> tower;  // catch tower (stacks then stand on its launch mount)
};

LaunchSite launch_site_from_config(const config::LaunchSiteConfig& site);
```

Public `FlightSession` members:

```cpp
    // Fidelity flag: catch towers of launch sites (default on). Takes effect at the next set_launch_site().
    void set_towers(bool enabled) { towers_enabled_ = enabled; }
    // Site of the mission (tower, landing target). launch_from_pad() sets it too.
    void set_launch_site(const LaunchSite& site);
    const LaunchSite& launch_site() const { return launch_site_; }
    // Places a vehicle with a free state (scenarios, tests); the body becomes the controlled one.
    void place_body(const config::VehicleConfig& config, const vehicle::RigidBodyState& state, time::Epoch epoch);
    const CatchTowerSim* tower() const { return tower_.get(); }
    CatchTowerSim* tower() { return tower_.get(); }
```

Private: `bool towers_enabled_ = true; std::unique_ptr<CatchTowerSim> tower_; std::vector<vehicle::FlightBody*> body_pointers() const;`

`flight_session.cpp`:

```cpp
LaunchSite launch_site_from_config(const config::LaunchSiteConfig& site) {
    return {site.latitude_rad, site.longitude_rad, site.height_m, site.tower};
}

void FlightSession::set_launch_site(const LaunchSite& site) {
    launch_site_ = site;
    tower_.reset();
    if (towers_enabled_ && site.tower) {
        tower_ = std::make_unique<CatchTowerSim>(*site.tower, site.latitude_rad, site.longitude_rad, site.height_m);
    }
}

std::vector<vehicle::FlightBody*> FlightSession::body_pointers() const {
    std::vector<vehicle::FlightBody*> list;
    for (const auto& flying : bodies_) list.push_back(flying.body.get());
    return list;
}

void FlightSession::place_body(const config::VehicleConfig& config, const vehicle::RigidBodyState& state, time::Epoch epoch) {
    auto body = std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(config, config_dir_), state);
    body->set_ground_height(launch_site_.height_m);
    bodies_.clear();
    bodies_.push_back({std::move(body), {}, nullptr, {}});
    controlled_ = 0;
    epoch_ = epoch;
    launch_epoch_ = epoch;
}
```

`launch_from_pad`: replace the body creation and `launch_site_ = site;` with

```cpp
    set_launch_site(site);
    const double stand_height = site.height_m + (tower_ ? site.tower->mount_height_m : 0.0);
    bodies_.clear();
    bodies_.push_back({vehicle::FlightBody::on_pad(std::move(vehicle), site.latitude_rad, site.longitude_rad,
                                                   stand_height, heading_rad, earth_, epoch),
                       {}, nullptr, {}});
    bodies_.back().body->set_ground_height(site.height_m);
```

`advance_to`, fixed-tick branch: before the loop that steps the bodies add

```cpp
        const auto pointers = body_pointers();
        if (tower_) tower_->before_tick(pointers, earth_, epoch_);
```

and after that loop, before `epoch_ += tick_s;`:

```cpp
        if (tower_) tower_->after_tick(pointers, earth_, epoch_ + tick_s, tick_s);
```

`coast_allowance_s`: after the `guided` check add `if (tower_ && tower_->needs_ticks(body_pointers(), earth_, epoch_)) return 0.0;`, and change the skip line to `if (body.impacted() || body.resting() || body.caught()) continue;`.

Add `src/sim/catch_tower_sim.cpp` and the contact sources to `core/CMakeLists.txt` if not yet there.

- [ ] **Step 5: Run tests**

Run: build, then `build/core/tests/starsim_core_tests.exe "[catch]"` and `"[flight]","[gnc]","[vehicle]"` (tag names as used in the repo).
Expected: the four scenarios pass; all previous tests pass unchanged. If scenario (a) ends `impacted` with a high pin load, check the contact sign conventions first (normal points out of the arm; load is `f.z`), then the at-rest thresholds; do not raise `max_load_n`.

- [ ] **Step 6: Full suite and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings` -> 100 % passed.

```bash
git add core tests 2>/dev/null; git commit -m "feat: catch tower in the flight session with automatic arms, contact and catch detection"
```

---

### Task 9: Booster return to the catch point

**Files:**
- Modify: `core/include/starsim/gnc/booster_return.hpp`, `core/src/gnc/booster_return.cpp`, `core/src/sim/flight_session.cpp`, `core/tests/test_booster_return.cpp`

**Interfaces:**
- Produces: `gnc::CatchTarget { Vec3 pin_point_itrf_m; Vec3 across_itrf; double pin_station_m; double pin_azimuth_rad; }`, `LandingSite::catch_target` (`std::optional`), `ReturnCommand::roll_reference_gcrf`, `BoosterReturnGuidance::set_catch_contact(bool)`, free function `math::Vec3 gnc::catch_roll_reference(const math::Vec3& across, const math::Vec3& up, double pin_azimuth_rad, const math::Vec3& current_body_y)` (direction of body +y that turns the first pin towards +-across; sign nearest the current body y).

- [ ] **Step 1: Write the failing tests**

Unit test, append to `core/tests/test_booster_return.cpp`:

```cpp
TEST_CASE("Catch roll reference turns the pins towards the arms", "[gnc][booster]") {
    const math::Vec3 up{0.0, 0.0, 1.0};
    const math::Vec3 across{1.0, 0.0, 0.0};
    for (const double azimuth_deg : {90.0, 45.0}) {
        const double a = azimuth_deg * deg;
        const auto y = gnc::catch_roll_reference(across, up, a, {0.0, 1.0, 0.0});
        const auto z = cross(up, y);
        const math::Vec3 pin = y * std::cos(a) + z * std::sin(a);  // first pin direction in the site frame
        CHECK(std::abs(std::abs(dot(pin, across)) - 1.0) < 1e-12);
        // The opposite current roll gives the opposite reference (shortest roll, pins symmetric).
        const auto y2 = gnc::catch_roll_reference(across, up, a, -y);
        CHECK(norm(y2 + y) < 1e-12);
    }
}
```

Integration test, append to the same file:

```cpp
TEST_CASE("Booster return to the catch tower ends caught or missed", "[gnc][booster][catch][validation]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const auto site = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1"));
    const std::string vehicle_id = GENERATE(as<std::string>{}, "starship_block1", "starship_block2", "starship_block3");
    CAPTURE(vehicle_id);
    session.launch_from_pad(vehicle_id, site, 90.0 * deg, t0);
    session.set_booster_return(true);
    gnc::AscentTarget target;
    target.periapsis_altitude_m = 200e3;
    target.apoapsis_altitude_m = 200e3;
    target.inclination_rad = 26.6 * deg;
    session.engage_ascent_autopilot(target);

    bool landing = false;
    for (double t = 0.0; t < 1200.0;) {
        t += landing ? sim::FlightSession::tick_s : 0.5;
        session.advance_to(t0 + t, 1000);
        CHECK_FALSE(session.body(session.controlled_index()).impacted());  // clears the tower on the way up
        if (session.body_count() < 2) continue;
        const auto& booster = session.body(0);
        const auto* status = session.return_status(0);
        landing = status && status->phase >= gnc::ReturnPhase::LandingBurn;
        if (booster.caught() || booster.impacted() || booster.resting()) break;
    }
    const auto* tower = session.tower();
    REQUIRE(tower != nullptr);
    const auto phase = tower->phase();
    CHECK((phase == sim::CatchPhase::Caught || phase == sim::CatchPhase::Missed));
    const auto t_end = earth.fixed_to_gcrf(session.epoch());
    const auto p = tower->geometry().to_site(transpose(t_end.rotation) * session.body(0).state().position_gcrf_m);
    std::printf("%s: tower %s, booster %.1f m from the catch axis, max pin load %.2f MN\n", vehicle_id.c_str(),
                sim::to_string(phase), std::hypot(p.x, p.y), tower->max_pin_load_n() / 1e6);
}
```

(Add `#include "starsim/config/launch_site_config.hpp"`, `<cstdio>` if missing.)

- [ ] **Step 2: Run to verify it fails** - build. Expected: compile errors (`catch_roll_reference`, `tower()` usage compiles after Task 8, `catch_target` missing).

- [ ] **Step 3: Guidance**

`booster_return.hpp`:

```cpp
// Catch on the tower arms instead of a touchdown on the pad.
struct CatchTarget {
    math::Vec3 pin_point_itrf_m;  // pins' midpoint on a centred catch: on the catch axis at the arm tops
    math::Vec3 across_itrf;       // horizontal unit; the pins must point along +-this direction
    double pin_station_m = 0.0;   // vehicle frame
    double pin_azimuth_rad = 0.0; // first pin, from body +y towards +z
};

// Body +y direction (same frame as the inputs) that turns the first pin along +-across, upright body (x = up);
// of the two solutions the one nearer `current_body_y` (the pins are symmetric).
math::Vec3 catch_roll_reference(const math::Vec3& across, const math::Vec3& up, double pin_azimuth_rad,
                                const math::Vec3& current_body_y);
```

`LandingSite`: add `std::optional<CatchTarget> catch_target;` (include `<optional>`). `ReturnCommand`: `math::Vec3 roll_reference_gcrf;  // desired body +y (zero: free roll)`. Class: `void set_catch_contact(bool contact) { catch_contact_ = contact; }`, member `bool catch_contact_ = false;` and `double target_bottom_height_m_ = 0.0;  // bottom above the site ground at touchdown / catch`.

`booster_return.cpp`:

```cpp
constexpr double catch_missed_height_m = 5.0;  // CG this far below the catch height: the arms were missed

math::Vec3 catch_roll_reference(const Vec3& across, const Vec3& up, double pin_azimuth_rad, const Vec3& current_body_y) {
    const Vec3 y = across * std::cos(pin_azimuth_rad) - cross(up, across) * std::sin(pin_azimuth_rad);
    return dot(y, current_body_y) < 0.0 ? -y : y;
}
```

Constructor: after storing the members,

```cpp
    if (site_.catch_target) {
        const auto pins = frames::fixed_to_geodetic(site_.catch_target->pin_point_itrf_m);
        target_bottom_height_m_ = pins.height_m - site_.height_m - site_.catch_target->pin_station_m;
    }
```

`predict_landing_burn`: `double h = geo.height_m - mass.cg_station_m - site_.height_m - target_bottom_height_m_;`.

Landing burn: replace `const Vec3 r_pad = site_fixed + site_up * mass.cg_station_m;` with

```cpp
    const Vec3 r_pad = site_.catch_target
                           ? site_.catch_target->pin_point_itrf_m - site_up * (site_.catch_target->pin_station_m - mass.cg_station_m)
                           : site_fixed + site_up * mass.cg_station_m;  // CG position at touchdown / catch
```

after `cmd.axis_gcrf = ...` in the landing burn:

```cpp
        if (site_.catch_target) {
            const Vec3 body_y = to_fixed * s.attitude.rotate(Vec3{0.0, 1.0, 0.0});
            cmd.roll_reference_gcrf = transform.rotation * catch_roll_reference(site_.catch_target->across_itrf, site_up,
                                                                                site_.catch_target->pin_azimuth_rad, body_y);
        }
```

and the landed condition becomes

```cpp
        const bool catch_over = site_.catch_target && (catch_contact_ || height < -catch_missed_height_m);
        if (body.resting() || body.impacted() || catch_over ||
            (!site_.catch_target && final_engines_ && height < touchdown_cutoff_height_m)) {
            phase_ = ReturnPhase::Landed;
        }
```

- [ ] **Step 4: Session wiring** (`flight_session.cpp`)
- `command_for`: `return_controller_.steer(*flying.body, ambient_pressure_pa, r.axis_gcrf, r.roll_reference_gcrf).apply_to(command);`
- `advance_to`: in the loop that runs the return guidance, before `run_return_guidance(bodies_[i]);`: `if (tower_) bodies_[i].return_guidance->set_catch_contact(tower_->in_contact(*bodies_[i].body));`
- `stage()`: build the landing site with the catch target:

```cpp
    if (booster_return_) {
        gnc::LandingSite landing{launch_site_.latitude_rad, launch_site_.longitude_rad, launch_site_.height_m};
        const auto& booster_stage = lower.body->vehicle().stages().front();
        if (tower_ && !booster_stage.config.catch_pins.azimuths_rad.empty()) {
            const auto& g = tower_->geometry();
            landing.catch_target = gnc::CatchTarget{g.to_itrf({0.0, 0.0, g.arm_top_height_m(0.0)}), g.site_to_itrf() * g.across(),
                                                    booster_stage.base_station_m + booster_stage.config.catch_pins.station_m,
                                                    booster_stage.config.catch_pins.azimuths_rad.front()};
        }
        lower.return_guidance = std::make_unique<gnc::BoosterReturnGuidance>(landing, *booster_return_);
    }
```

- [ ] **Step 5: Run tests**

Run: build, `STARSIM_VERBOSE=1 build/core/tests/starsim_core_tests.exe "[booster]"`.
Expected: all pass; the catch test prints one line per block (caught or missed, miss distance). Record the three lines in the commit message. The pad-landing test (no tower) keeps its assertions.

- [ ] **Step 6: Full suite and commit**

Run: `tools/build.ps1 -NoBindings` -> 100 %.

```bash
git commit -am "feat: booster return aims at the catch arms and rolls its pins towards them"
```

---

### Task 10: Bindings

**Files:**
- Modify: `bindings/src/flight_sim.hpp`, `bindings/src/flight_sim.cpp`, `game/tests/smoke_test.gd`

**Interfaces:**
- Produces (GDScript): `FlightSim.launch_from_site(vehicle_id: String, site_id: String, heading_deg: float, tdb_s: float) -> bool`, `FlightSim.site_tower(site_id: String) -> Dictionary`, `FlightSim.tower_state() -> Dictionary`.
  - `site_tower`: `present` (bool) and, if present: `origin_itrf_m`, `east`, `north`, `up` (Vector3, ITRF), `tower_direction`, `across` (Vector3, site), `tower_center_m` (site), `tower_size_m` (Vector3: width, width, height along tower_direction, across, up), `mount_height_m`, `mount_inner_radius_m`, `mount_outer_radius_m`, `arm_length_m`, `arm_width_m`, `arm_thickness_m`, `arm_open_rad`, `hinge_plus_m`, `hinge_minus_m` (Vector3 site, arm top, carriage at rest).
  - `tower_state`: `present`, `opening_rad`, `carriage_offset_m`, `phase` (German).

- [ ] **Step 1: Failing smoke check** - in `game/tests/smoke_test.gd`, inside `_initialize()` before the final `quit(...)`, add (using the file's existing `_check`):

```gdscript
	var tower_sim := FlightSim.new()
	tower_sim.setup(ProjectSettings.globalize_path("res://").path_join("../config"))
	var tower: Dictionary = tower_sim.site_tower("starbase_olp1")
	_check(tower.get("present", false), "Starbase has a catch tower")
	_check(absf(float(tower.get("mount_height_m", 0.0)) - 20.0) < 1e-9, "tower geometry from the core config")
```

(If the smoke test already resolves the config directory differently, reuse that expression.)

Run: `tools/build.ps1` then `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd`
Expected: FAIL (`site_tower` does not exist).

- [ ] **Step 2: Implement** - `flight_sim.hpp`: declare the three methods and add `std::filesystem::path config_dir_;` (set in `setup`). `flight_sim.cpp`:

```cpp
bool FlightSim::launch_from_site(const godot::String& vehicle_id, const godot::String& site_id, double heading_deg,
                                 double tdb_s) {
    if (!session_) {
        godot::UtilityFunctions::push_error("FlightSim: call setup() first");
        return false;
    }
    try {
        const auto site = config::load_launch_site(config_dir_, site_id.utf8().get_data());
        session_->launch_from_pad(vehicle_id.utf8().get_data(), sim::launch_site_from_config(site),
                                  heading_deg * constants::deg_to_rad, epoch(tdb_s));
        return true;
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String("FlightSim.launch_from_site: ") + e.what());
        return false;
    }
}

godot::Dictionary FlightSim::site_tower(const godot::String& site_id) const {
    godot::Dictionary d;
    d["present"] = false;
    try {
        const auto site = config::load_launch_site(config_dir_, site_id.utf8().get_data());
        if (!site.tower) return d;
        const contact::TowerGeometry g(*site.tower, site.latitude_rad, site.longitude_rad, site.height_m);
        const auto& c = *site.tower;
        auto v = [](const math::Vec3& x) { return godot::Vector3(x.x, x.y, x.z); };
        d["present"] = true;
        d["origin_itrf_m"] = v(g.origin_itrf_m());
        d["east"] = v(contact::column(g.site_to_itrf(), 0));
        d["north"] = v(contact::column(g.site_to_itrf(), 1));
        d["up"] = v(contact::column(g.site_to_itrf(), 2));
        d["tower_direction"] = v(g.tower_direction());
        d["across"] = v(g.across());
        d["tower_center_m"] = v(g.tower_box().center);
        d["tower_size_m"] = godot::Vector3(c.width_m, c.width_m, c.height_m);
        d["mount_height_m"] = c.mount_height_m;
        d["mount_inner_radius_m"] = c.mount_inner_radius_m;
        d["mount_outer_radius_m"] = c.mount_outer_radius_m;
        d["arm_length_m"] = c.arm_length_m;
        d["arm_width_m"] = c.arm_width_m;
        d["arm_thickness_m"] = c.arm_thickness_m;
        d["arm_open_rad"] = c.arm_open_rad;
        d["hinge_plus_m"] = v(g.hinge(1));
        d["hinge_minus_m"] = v(g.hinge(-1));
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String("FlightSim.site_tower: ") + e.what());
    }
    return d;
}

godot::Dictionary FlightSim::tower_state() const {
    godot::Dictionary d;
    const auto* tower = session_ ? session_->tower() : nullptr;
    d["present"] = tower != nullptr;
    if (!tower) return d;
    d["opening_rad"] = tower->arms().opening_rad();
    d["carriage_offset_m"] = tower->arms().carriage_offset_m();
    d["phase"] = godot::String::utf8(sim::to_string(tower->phase()));
    return d;
}
```

Bind them in `_bind_methods`:

```cpp
    ClassDB::bind_method(D_METHOD("launch_from_site", "vehicle_id", "site_id", "heading_deg", "tdb_s"), &FlightSim::launch_from_site);
    ClassDB::bind_method(D_METHOD("site_tower", "site_id"), &FlightSim::site_tower);
    ClassDB::bind_method(D_METHOD("tower_state"), &FlightSim::tower_state);
```

Includes: `starsim/config/launch_site_config.hpp`, `starsim/contact/tower_geometry.hpp`, `starsim/sim/catch_tower_sim.hpp`.

- [ ] **Step 3: Run** - `tools/build.ps1`, then the smoke test. Expected: PASS (all checks).

- [ ] **Step 4: Commit** - `git commit -am "feat: tower geometry and catch state for the game"`

---

### Task 11: Tower in Godot

**Files:**
- Create: `game/world/tower_view.gd`, `game/shaders/lattice.gdshader`
- Modify: `game/world/world.gd`, `game/vehicle/flight_controller.gd`, `game/ui/flight_hud.gd`, `game/tests/render_check.gd`

**Interfaces:**
- Consumes: `FlightSim.site_tower`, `FlightSim.tower_state`, `FlightSim.launch_from_site`.
- Produces: `TowerView` (`site_id`, `preview_opening_rad`, `build(geometry: Dictionary)`, `update_state(state: Dictionary)`); `FlightController.launch_site_id`.

- [ ] **Step 1: Lattice shader** - `game/shaders/lattice.gdshader`

```glsl
shader_type spatial;
render_mode cull_disabled;
// Open steel lattice: members along the bay edges and both diagonals of every bay, gaps cut out.

uniform vec4 steel : source_color = vec4(0.32, 0.33, 0.35, 1.0);
uniform float bay_m = 6.0;
uniform float member_fraction = 0.035;

varying vec3 local_position;

void vertex() {
	local_position = VERTEX;
}

void fragment() {
	vec2 p = vec2(local_position.x + local_position.z, local_position.y) / bay_m;
	vec2 f = fract(p);
	float edge = min(min(f.x, 1.0 - f.x), min(f.y, 1.0 - f.y));
	float diagonal = min(abs(f.x - f.y), abs(f.x + f.y - 1.0)) * 0.7071;
	ALBEDO = steel.rgb;
	METALLIC = 0.6;
	ROUGHNESS = 0.5;
	ALPHA = step(min(edge, diagonal), member_fraction);
	ALPHA_SCISSOR_THRESHOLD = 0.5;
}
```

- [ ] **Step 2: TowerView** - `game/world/tower_view.gd`

```gdscript
class_name TowerView
extends Node3D
## Catch tower of a launch site (tower, launch mount, two arms on their carriage), built procedurally from
## the core's geometry (FlightSim.site_tower): what is drawn is what the physics collides with.
## Child of the Earth body, placed at the site origin in Earth-fixed coordinates. Local coordinates are the
## site frame (east, north, up) mapped to render axes like every body frame (Frames.to_render).

const MOUNT_SEGMENTS := 24

var site_id := ""
## render_check: fixed arm opening (radians) instead of the flight state.
var preview_opening_rad := NAN
var _geometry: Dictionary = {}
var _arms: Array[Node3D] = []  ## pivots at the hinges: [+across, -across]
var _steel := StandardMaterial3D.new()
var _concrete := StandardMaterial3D.new()
var _lattice := ShaderMaterial.new()


func build(geometry: Dictionary) -> void:
	_geometry = geometry
	_steel.albedo_color = Color(0.55, 0.56, 0.58)
	_steel.metallic = 0.6
	_steel.roughness = 0.45
	_concrete.albedo_color = Color(0.62, 0.61, 0.58)
	_concrete.roughness = 0.9
	_lattice.shader = load("res://shaders/lattice.gdshader")
	position = Frames.to_render(geometry["origin_itrf_m"])
	basis = Frames.basis_to_render(Basis(geometry["east"], geometry["north"], geometry["up"]))
	var toward: Vector3 = geometry["tower_direction"]
	var across: Vector3 = geometry["across"]
	var up := Vector3(0.0, 0.0, 1.0)
	add_child(_box(geometry["tower_size_m"], Basis(toward, across, up), geometry["tower_center_m"], _lattice))
	_build_mount(geometry)
	for hinge_key in ["hinge_plus_m", "hinge_minus_m"]:
		var pivot := Node3D.new()
		var length: float = geometry["arm_length_m"]
		var thickness: float = geometry["arm_thickness_m"]
		pivot.add_child(_box(Vector3(length, geometry["arm_width_m"], thickness), Basis(),
				Vector3(0.5 * length, 0.0, -0.5 * thickness), _steel))
		add_child(pivot)
		_arms.append(pivot)
	update_state({})


## state: FlightSim.tower_state() of the flight at this site, or {} (arms open, carriage at rest).
func update_state(state: Dictionary) -> void:
	if _geometry.is_empty():
		return
	var opening: float = state.get("opening_rad", _geometry["arm_open_rad"])
	if not is_nan(preview_opening_rad):
		opening = preview_opening_rad
	var offset: float = state.get("carriage_offset_m", 0.0)
	var toward: Vector3 = _geometry["tower_direction"]
	var across: Vector3 = _geometry["across"]
	var up := Vector3(0.0, 0.0, 1.0)
	for i in range(_arms.size()):
		var side := 1.0 if i == 0 else -1.0
		var hinge: Vector3 = _geometry["hinge_plus_m" if i == 0 else "hinge_minus_m"]
		var u := -toward * cos(opening) + across * (side * sin(opening))  # mirrors TowerGeometry::arm_direction
		_arms[i].basis = Frames.basis_to_render(Basis(u, up.cross(u), up))
		_arms[i].position = Frames.to_render(hinge + up * offset)


## Box with `size_local` along the columns of `axes` (site frame), centred at `center_site`.
func _box(size_local: Vector3, axes: Basis, center_site: Vector3, material: Material) -> MeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = Vector3(size_local.y, size_local.z, size_local.x)  # local (a, b, c) -> render (b, c, a)
	var node := MeshInstance3D.new()
	node.mesh = mesh
	node.material_override = material
	node.basis = Frames.basis_to_render(axes)
	node.position = Frames.to_render(center_site)
	return node


func _build_mount(geometry: Dictionary) -> void:
	var inner: float = geometry["mount_inner_radius_m"]
	var outer: float = geometry["mount_outer_radius_m"]
	var height: float = geometry["mount_height_m"]
	var radius := 0.5 * (inner + outer)
	var segment := TAU * radius / MOUNT_SEGMENTS
	for k in range(MOUNT_SEGMENTS):
		var a := TAU * k / MOUNT_SEGMENTS
		var radial := Vector3(cos(a), sin(a), 0.0)
		var tangent := Vector3(-sin(a), cos(a), 0.0)
		add_child(_box(Vector3(outer - inner, segment * 1.02, height), Basis(radial, tangent, Vector3(0.0, 0.0, 1.0)),
				radial * radius + Vector3(0.0, 0.0, 0.5 * height), _concrete))
```

- [ ] **Step 3: Wiring**
- `flight_controller.gd`: add `var launch_site_id := ""`; in `launch()` replace the `sim.launch_from_pad(...)` call with `var ok := sim.launch_from_site(vehicle_id, site["id"], heading_deg, tdb_s)` and set `launch_site_id = site["id"] if ok else ""`.
- `world.gd`: add `var towers: Array[TowerView] = []`; at the end of `_setup_bodies` (after the pads), add:

```gdscript
	for site: Dictionary in sites:
		var geometry: Dictionary = flight.sim.site_tower(site["id"])
		if geometry.get("present", false):
			var view := TowerView.new()
			view.site_id = site["id"]
			earth.add_child(view)
			view.build(geometry)
			towers.append(view)
```

  and in `update_world()` after the flight advance:

```gdscript
	for view in towers:
		view.update_state(flight.sim.tower_state() if flight.sim.is_active() and view.site_id == flight.launch_site_id else {})
```
- `flight_hud.gd`: after the autopilot block add

```gdscript
	var tower: Dictionary = sim.tower_state()
	if tower.get("present", false) and sim.body_count() > 1:
		lines.append("Turm            %s" % tower["phase"])
```
- `render_check.gd`: document `--tower_arms_deg=<deg>` in the header comment; after the world is set up and before rendering, add:

```gdscript
		if _args.has("tower_arms_deg"):
			for view in world.towers:
				view.preview_opening_rad = deg_to_rad(float(_args["tower_arms_deg"]))
				view.update_state({})
```

- [ ] **Step 4: Visual check (needs GPU)**

Run: `external/godot/bin/godot.windows.editor.double.x86_64.exe --path game --script res://tests/render_check.gd -- --utc=2026-10-12T15:00:00 --vehicle=starship_block3 --site=starbase_olp1 --distance=400 --elevation=15 --azimuth=200 --out=C:/tmp/tower_open.png`, then the same with `--tower_arms_deg=0 --out=C:/tmp/tower_closed.png`.
Expected: read both PNGs: tower lattice next to the stack on the ring-shaped mount; arms swung out in the first, parallel over the pad in the second; the vehicle stands on the mount, not inside it. Adjust camera arguments until the tower is in view (the options are in the header of `render_check.gd`).

Run the smoke test again. Expected: PASS.

- [ ] **Step 5: Commit** - `git commit -am "feat: procedural catch tower in the game with live arm state and HUD status"` (add the two new files explicitly).

---

### Task 12: Documentation and verification

**Files:**
- Modify: `CLAUDE.md`, `config/README.md`, `docs/superpowers/specs/2026-10-12-catch-tower-design.md`, `docs/ansichten.md`

- [ ] **Step 1: CLAUDE.md**
- Architecture: add `contact (catch tower: geometry, arms, pin contact, collisions)` to "Implemented so far".
- Game structure: `game/world/tower_view.gd` (procedural catch tower from `FlightSim.site_tower`, arms from `tower_state`).
- Booster return paragraph: with a tower site (`launch_from_site` / `launch_site_from_config`) the landing burn aims at the catch point (pins at the arm tops), rolls the pins towards the arms and cuts off on pin contact; `sim::CatchTowerSim` closes the arms automatically (time to contact <= closing time + 1 s), applies penalty contact forces (pins and hull vs arm boxes, friction 0.3) and declares caught (both pins, at rest 2 s), impact (tower/mount hit, pin load > `max_load_n`, carriage hard stop > 1 m/s) or missed. Fidelity flag `FlightSession::set_towers`.
- Known issues: replace the "Booster return lands on the pad ..." sentence's conclusion with the measured catch outcomes of Task 9 (caught/missed per block, miss distance) and note that tower azimuths are [E] to be checked against imagery.

- [ ] **Step 2: config/README.md** - document `[site.tower]` and `[stage.catch_pins]` (keys, units, tags) next to the existing tables.

- [ ] **Step 3: Spec** - in the catch-tower spec, apply the refinements listed at the top of this plan (derived hinge position, `azimuths_deg` + `max_load_n`, time-to-contact trigger, `close_arms(instantly)`).

- [ ] **Step 4: docs/ansichten.md** - in the HUD section add the booster line "Turm: offen / schließt / gefangen / verfehlt".

- [ ] **Step 5: Full verification**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` -> 100 % tests passed; smoke test -> PASS.

- [ ] **Step 6: Commit** - `git commit -am "docs: catch tower"`
