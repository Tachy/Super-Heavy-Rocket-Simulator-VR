# Crew Dragon d4: Parachutes and Splashdown Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Crew Dragon capsule opens its drogues at the 5.5 km gate and its mains at ~1.8 km, both reefed, with
opening loads judged. It splashes down in the Pacific zone ("gewassert" at <= 9 m/s), and the full return mission from
orbit (`crew_dragon_return.toml`) reproduces the published event timeline.

**Architecture:**
- **Parachute model.** A parachute set is configured in `[[stage.parachute]]` and modelled in `aero/parachute`: a drag
  area that grows through its reefing stages over time, and a drag force on a riser point. The canopy is massless.
- **Physics.** `Vehicle` holds the per-set state (stowed / open / released / torn). `FlightBody` applies the force and
  moment at the attachment point, and judges each canopy's opening load against its limit.
- **Sequencer.** `gnc/parachute_sequencer` is a pure state machine. It fires the drogues on altitude plus a
  dynamic-pressure check, then swaps them for the mains on altitude, then releases the mains on the water.
- **Session.** `FlightSession` runs one sequencer per body whose lowest stage carries parachutes. It stops the Dracos
  under the canopies (user 2026-10-14: no steering under the parachutes) and ticks below the deployment height.
- **Splashdown.** A body whose lowest stage recovers by `"splashdown"` rests on the water at any tilt if its sink rate
  is <= `max_splashdown_speed_mps`, and is judged "gewassert". Otherwise it is an impact.
- **Mission.** The return mission is a config file. A small `sim::place_capsule_return_mission` places the capsule in
  its orbit with the zone set, and `plan_capsule_return` flies it to the water.

**Tech Stack:** C++20 core (`starsim_core`), Catch2 tests, TOML config (toml++), CMake/Ninja (MSVC 2022).

**Spec:** `docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md`, section "d4 design: parachutes and splashdown"
(plus Decomposition row d4 and "Choices made for the review" item 5).

## Global Constraints

- Code, comments, commit messages and docs in English. UI strings in the core are German (`"gewassert"`,
  `"Fallschirm gerissen"`, `"Bremsschirme"`, `"Hauptschirme"`).
- SI units internally, angles in radians. Frame and unit go into the name (`..._body_n`, `..._m2`, `..._mps`).
- Every config value carries a provenance tag: [S] sourced, [E] estimate, [D] derived.
- No magic numbers in code: thresholds come from config or named `constexpr` with a comment and tag.
- `core` modules depend downwards only. `aero` may use `config`, `math`. `gnc` may use `config`, `vehicle`, `aero`.
  `sim` uses everything.
- Spec, Model: "a canopy is a drag area C_D S(t) that grows from zero through reefing stages to full ... It acts through
  a riser attachment point on the capsule. The force goes along the air-relative velocity of that point, and the moment
  comes from the attachment offset. The canopy is massless, with no pendulum body of its own (a deliberate
  simplification, documented; added mass and pendulum swing are left out). Opening load = q x C_D S. It is judged
  against a limit [E] ("Fallschirm gerissen" beyond it)."
- Spec, Crew Dragon [S]: "2 drogues at ~5.5 km (~560 km/h), released when the 4 mains deploy at ~1.8 km, splashdown at
  ~25 km/h. The deployment triggers are altitude plus a dynamic-pressure check (`gnc/parachute_sequencer`)."
- Spec, Splashdown: "a vehicle with `[stage.recovery] method = "splashdown"` that reaches sea level gives "gewassert"
  (success) when the vertical speed is <= 9 m/s [E] and its mains are open. Otherwise the outcome is "Aufprall". The
  capsule rests at the surface (no buoyancy model)."
- Spec, Evidence tolerances: "event altitudes +-15 %, splashdown speed +-30 % [E]".
- User 2026-10-14: "Die Kapsel ist ja nach dem Eintritt sowieso nicht steuerbar ... unter den Schirmen gemeint". There is
  no steering under the parachutes, so the Dracos stay off once a parachute is out.
- User 2026-10-14: the splashdown zone radius is 2 km (`pacific_san_diego`).
- User 2026-10-14: RCS is on/off pulses only. No drawing and no UI in this part.
- Build: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`. Single test:
  `build/core/tests/starsim_core_tests.exe "<name|[tag]>"`. Full suite:
  `ctest --test-dir build -j 12` (`"/c/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/ctest.exe"`).
  It must stay green: 386/386 before this plan, plus the new tests.

## Evidence (published, [S])

| Event | Published | Source |
|---|---|---|
| Drogues | ~18,000 ft (5.5 km), ~350 mph (156 m/s) | NASA Demo-2 return timeline, `blogs.nasa.gov/commercialcrew` 2020-08-02; NASA "Top 10 things to know for Demo-2 return" |
| Mains | ~6,000 ft (1.83 km; SpaceX Crew-1: ~6,500 ft), ~119 mph (53 m/s) | same; spacex.com "Crew-1 returns to Earth" |
| Drogues -> mains | "less than a minute later" (Demo-2: 2:44 -> 2:45 p.m.) | NASA commercial crew blog |
| Mains -> splashdown | Demo-2: 2:45 -> 2:48 p.m. (~3 min, minute resolution) | same |
| Splashdown speed | ~15-16 mph (6.7-7.2 m/s) | Crew-7 coverage (Spectrum News 2024-03-11), TechCrunch Demo-2 preview |
| Deorbit burn start -> splashdown | 10:39 -> 11:33 a.m. EDT = 54 min (Crew-10, 2025-08-09, off San Diego; deorbit before the trunk jettison) | NASA Crew-10 return coverage schedule; nasaspaceflight.com 2025-08 |

Webcast telemetry with a Dragon layout in `webcast_telemetry.py` (spec: "if its overlay shows speed and altitude") is
not part of this plan. The published figures above cover every tolerance the spec names, and SpaceX webcasts carry no
free licence. The plan records this as a ruling in the ledger at Task 6.

Derived canopy sizes [D], from those speeds with a 10.0 t capsule [E] (dry 8.4 t + ~1.4 t of MMH/NTO left + crew in the
dry mass) and the capsule's own subsonic C_A x S = 0.97 x 12.57 = 12.2 m^2:
- Drogues: terminal 53 m/s at 1.8 km (rho 1.027) means C_D S total = 98.1 kN / 1442 Pa = 68.0 m^2. Without the
  capsule that is 55.8 m^2, so **28 m^2 per drogue**.
- Mains: terminal 7.0 m/s at sea level means C_D S total = 98.1 kN / 30.0 Pa = 3270 m^2, so **815 m^2 per main**. That
  is consistent with the commonly cited 116 ft (35 m) Mark 3 canopy at C_D ~0.85.

## Review Focus

1. **Drogues held back by a high dynamic pressure all the way down.** The sequencer must still fire the mains at their
   height as a backup. At high q they tear ("Fallschirm gerissen") and the capsule hits the water hard: "Aufprall", not
   "gewassert". Test in Task 5: "Drogues held back by dynamic pressure: the mains tear and the capsule hits the water".
2. **Time warp across the deployment height.** An analytic coast segment from above the drogue height must not jump
   past it; the session must tick there. Test in Task 5: "Drogues open at their height under time warp".
3. **A capsule placed below the main height (scenario).** It skips the drogues and opens the mains at once. Test in
   Task 5: "A capsule placed below the main height opens its mains at once".
4. **Mains torn or never opened before the water.** Sink > 9 m/s must give "Aufprall" even though the vehicle is a
   splashdown vehicle. Test in Task 4: "A capsule hitting the water faster than its limit is an impact".
5. **A capsule still stacked (launch, trunk on) must never open its parachutes,** because only the lowest stage's sets
   are sequenced and an ascending vehicle never triggers. Tests in Task 3 ("An ascending vehicle never deploys") and
   Task 5 ("A capsule on its trunk has no parachute sequencer").

---

## File Structure

| File | Responsibility |
|---|---|
| `core/include/starsim/config/vehicle_config.hpp`, `core/src/config/vehicle_config.cpp` (modify) | `ParachuteConfig`, `ReefingStageConfig`, `StageConfig::parachutes`, `StageConfig::max_splashdown_speed_mps`; loader + validation (incl. d3 minor M3) |
| `core/include/starsim/aero/parachute.hpp`, `core/src/aero/parachute.cpp` (create) | drag area over time (reefing, t^2 fill), riser-point drag force |
| `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp` (modify) | `Parachute` state per stage, deploy / release / update / load record, station shift on `separate`, `splashdown_recovery()`; d3 minor M5 |
| `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp` (modify) | parachute force + moment in `evaluate`, opening-load judgement in `step`, splashdown contact + `touchdown_sink_mps()` |
| `core/include/starsim/gnc/parachute_sequencer.hpp`, `core/src/gnc/parachute_sequencer.cpp` (create) | pure deployment state machine |
| `core/include/starsim/sim/landing_outcome.hpp`, `core/src/sim/landing_outcome.cpp` (modify) | `LandingOutcome::SplashedDown` ("gewassert") |
| `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (modify) | sequencer per body, RCS off under canopies, tick near deploy height, capsule return steps after the gate, `splashdown_miss_m`, `place_in_orbit(config)`; d3 minor M2 |
| `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp` (modify) | `CapsuleReturnMission`, `load_capsule_return_mission` |
| `core/include/starsim/sim/capsule_return_mission.hpp`, `core/src/sim/capsule_return_mission.cpp` (create) | `place_capsule_return_mission`, `capsule_return_parameters` |
| `config/vehicles/crew_dragon.toml` (modify) | drogue and main sets, `max_splashdown_speed_mps`, `nasa_*` sources |
| `config/missions/crew_dragon_return.toml` (create) | the return mission |
| `core/tests/test_parachute.cpp` (create) | model, config, vehicle and body unit tests, sequencer |
| `core/tests/test_capsule_splashdown.cpp` (create) | session tests from the gate, review-focus cases, full mission + timeline + campaign |
| `core/tests/test_capsule_entry.cpp`, `core/tests/test_landing_outcome.cpp` (modify) | d3 return test follows the new steps; outcome string |
| `core/CMakeLists.txt`, `core/tests/CMakeLists.txt` (modify) | new sources |
| `CLAUDE.md`, `config/README.md` (modify) | docs |

---

### Task 1: Parachute configuration and drag model

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp` (after `HeatShieldConfig`; `StageConfig` fields)
- Modify: `core/src/config/vehicle_config.cpp` (stage loader, after the `heat_shield` block; recovery block)
- Create: `core/include/starsim/aero/parachute.hpp`, `core/src/aero/parachute.cpp`
- Modify: `core/CMakeLists.txt` (add `src/aero/parachute.cpp` after `src/aero/capsule_aero.cpp`)
- Modify: `config/vehicles/crew_dragon.toml`
- Create: `core/tests/test_parachute.cpp`; Modify: `core/tests/CMakeLists.txt` (add `test_parachute.cpp` after `test_capsule_entry.cpp`)

**Interfaces:**
- Produces:
  - `config::ReefingStageConfig { double area_fraction; double hold_s; }`
  - `config::ParachuteConfig { std::string id; std::string role; int count; double drag_area_m2; double attach_station_m; double fill_time_s; std::vector<ReefingStageConfig> reefing; double max_load_n; double deploy_height_m; double max_deploy_dynamic_pressure_pa; }`
  - `StageConfig::parachutes` (`std::vector<ParachuteConfig>`)
  - `StageConfig::max_splashdown_speed_mps` (`double`, 0 unless method `"splashdown"`)
  - `double aero::parachute_drag_area_m2(const config::ParachuteConfig&, double time_since_deploy_s)` (one canopy)
  - `math::Vec3 aero::parachute_force_body_n(double drag_area_m2, const math::Vec3& point_air_velocity_body_mps, double density_kg_m3)`

- [ ] **Step 1: Write the failing tests** (`core/tests/test_parachute.cpp`)

```cpp
// Parachutes (d4 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md): configuration, drag area over the
// reefing stages, riser-point force; vehicle state, body physics, opening loads; the deployment sequencer.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>

#include "starsim/aero/parachute.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

namespace {

config::ParachuteConfig two_stage_chute() {
    config::ParachuteConfig p;
    p.id = "test";
    p.role = "main";
    p.count = 1;
    p.drag_area_m2 = 100.0;
    p.fill_time_s = 2.0;
    p.reefing = {{0.1, 3.0}, {0.4, 3.0}};
    p.max_load_n = 1e6;
    p.deploy_height_m = 1000.0;
    return p;
}

}  // namespace

TEST_CASE("Parachute drag area grows through its reefing stages", "[parachute]") {
    const auto p = two_stage_chute();
    CHECK(aero::parachute_drag_area_m2(p, -1.0) == 0.0);
    CHECK(aero::parachute_drag_area_m2(p, 0.0) == 0.0);
    CHECK(aero::parachute_drag_area_m2(p, 1.0) == Catch::Approx(100.0 * 0.1 * 0.25));  // t^2 fill: half time, quarter
    CHECK(aero::parachute_drag_area_m2(p, 2.0) == Catch::Approx(10.0));                // first stage reached
    CHECK(aero::parachute_drag_area_m2(p, 4.9) == Catch::Approx(10.0));                // held 3 s
    CHECK(aero::parachute_drag_area_m2(p, 6.0) == Catch::Approx(10.0 + 30.0 * 0.25));  // filling to stage 2
    CHECK(aero::parachute_drag_area_m2(p, 7.0) == Catch::Approx(40.0));
    CHECK(aero::parachute_drag_area_m2(p, 11.0) == Catch::Approx(40.0 + 60.0 * 0.25)); // full opening
    CHECK(aero::parachute_drag_area_m2(p, 12.0) == Catch::Approx(100.0));
    CHECK(aero::parachute_drag_area_m2(p, 1000.0) == Catch::Approx(100.0));
    auto unreefed = p;
    unreefed.reefing.clear();
    CHECK(aero::parachute_drag_area_m2(unreefed, 1.0) == Catch::Approx(25.0));
    CHECK(aero::parachute_drag_area_m2(unreefed, 2.0) == Catch::Approx(100.0));
}

TEST_CASE("Parachute force opposes the riser point's air velocity with q C_D S", "[parachute]") {
    const Vec3 v{-50.0, 3.0, -4.0};
    const auto f = aero::parachute_force_body_n(20.0, v, 1.2);
    const double q = 0.5 * 1.2 * dot(v, v);
    CHECK(norm(f) == Catch::Approx(q * 20.0));
    CHECK(dot(math::normalized(f), math::normalized(v)) == Catch::Approx(-1.0));
    CHECK(norm(aero::parachute_force_body_n(20.0, {}, 1.2)) == 0.0);
}

TEST_CASE("Crew Dragon carries two drogues and four mains", "[parachute][config]") {
    const auto c = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
    const auto& capsule = c.stages.back();
    REQUIRE(capsule.parachutes.size() == 2);
    const auto& drogue = capsule.parachutes[0];
    const auto& main = capsule.parachutes[1];
    CHECK(drogue.role == "drogue");
    CHECK(drogue.count == 2);
    CHECK(drogue.deploy_height_m == Catch::Approx(5500.0));
    CHECK(main.role == "main");
    CHECK(main.count == 4);
    CHECK(main.deploy_height_m == Catch::Approx(1800.0));
    CHECK(main.deploy_height_m < drogue.deploy_height_m);
    CHECK(capsule.max_splashdown_speed_mps == Catch::Approx(9.0));
    CHECK(c.stages.front().parachutes.empty());  // the trunk
}

namespace {

// A copy of crew_dragon.toml with `from` replaced by `to` in a temp config dir (engines and propellants copied along).
std::filesystem::path patched_config(const std::string& from, const std::string& to) {
    namespace fs = std::filesystem;
    const fs::path src = STARSIM_CONFIG_DIR;
    const fs::path dir = fs::temp_directory_path() / "starsim_parachute_config";
    fs::remove_all(dir);
    fs::create_directories(dir);
    fs::copy(src, dir, fs::copy_options::recursive);
    std::ifstream in(src / "vehicles" / "crew_dragon.toml");
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.replace(at, from.size(), to);
    std::ofstream(dir / "vehicles" / "crew_dragon.toml") << text;
    return dir;
}

}  // namespace

TEST_CASE("Parachute config is validated", "[parachute][config]") {
    CHECK_THROWS_AS(config::load_vehicle(patched_config("role = \"drogue\"", "role = \"ballute\""), "crew_dragon"),
                    config::ConfigError);
    CHECK_THROWS_AS(config::load_vehicle(patched_config("reefing = [[0.35, 4.0]]", "reefing = [[1.2, 4.0]]"), "crew_dragon"),
                    config::ConfigError);
    CHECK_THROWS_AS(config::load_vehicle(patched_config("max_splashdown_speed_mps = 9.0", ""), "crew_dragon"),
                    config::ConfigError);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`
Expected: compile error, `starsim/aero/parachute.hpp` not found.

- [ ] **Step 3: Add the configuration** (`vehicle_config.hpp`, after `HeatShieldConfig`)

```cpp
// One reefing stage of a parachute: the canopy fills to `area_fraction` of its full drag area and holds there.
struct ReefingStageConfig {
    double area_fraction = 0.0;
    double hold_s = 0.0;
};

// A set of identical parachutes ([[stage.parachute]]; d4): massless drag areas on a riser point on the axis.
struct ParachuteConfig {
    std::string id;
    std::string role;                  // "drogue" or "main" (gnc::ParachuteSequencer)
    int count = 0;                     // canopies in the set
    double drag_area_m2 = 0.0;         // C_D S0 of one canopy, fully open
    double attach_station_m = 0.0;     // riser attachment on the axis, from the bottom of the stage
    double fill_time_s = 0.0;          // inflation of each reefing stage and of the final opening
    std::vector<ReefingStageConfig> reefing;  // ascending area fractions in (0, 1); empty: opens to full at once
    double max_load_n = 0.0;           // per canopy: beyond it the set tears ("Fallschirm gerissen")
    double deploy_height_m = 0.0;      // above the ground / sea
    double max_deploy_dynamic_pressure_pa = 0.0;  // the deployment waits above it; 0 = no check
};
```

In `StageConfig`, after `HeatShieldConfig heat_shield;`:

```cpp
    std::vector<ParachuteConfig> parachutes;  // [[stage.parachute]] (d4: Crew Dragon drogues and mains)
    // [stage.recovery] max_splashdown_speed_mps (method "splashdown", required there): sink rate at the water up to
    // which the capsule is "gewassert"; faster is an impact.
    double max_splashdown_speed_mps = 0.0;
```

- [ ] **Step 4: Load and validate** (`vehicle_config.cpp`, after the `heat_shield` block)

```cpp
    if (auto chutes = t["parachute"].as_array()) {
        for (auto& node : *chutes) {
            const auto* pt = node.as_table();
            if (!pt) doc.fail("stage.parachute entries must be tables");
            config::ParachuteConfig p;
            p.id = doc.text(*pt, "id");
            p.role = doc.text(*pt, "role");
            if (p.role != "drogue" && p.role != "main") doc.fail("stage.parachute.role must be \"drogue\" or \"main\"");
            p.count = static_cast<int>(doc.positive(*pt, "count"));
            p.drag_area_m2 = doc.positive(*pt, "drag_area_m2");
            p.attach_station_m = doc.number(*pt, "attach_station_m");
            p.fill_time_s = doc.positive(*pt, "fill_time_s");
            p.max_load_n = doc.positive(*pt, "max_load_n");
            p.deploy_height_m = doc.positive(*pt, "deploy_height_m");
            p.max_deploy_dynamic_pressure_pa = doc.number_or(*pt, "max_deploy_dynamic_pressure_pa", 0.0);
            if (const auto* reefing = (*pt)["reefing"].as_array()) {
                double previous = 0.0;
                for (const auto& r : *reefing) {
                    const auto* pair = r.as_array();
                    if (!pair || pair->size() != 2) doc.fail("stage.parachute.reefing entries are [area_fraction, hold_s]");
                    const auto fraction = (*pair)[0].value<double>();
                    const auto hold = (*pair)[1].value<double>();
                    if (!fraction || !hold || *fraction <= previous || *fraction >= 1.0 || *hold < 0.0) {
                        doc.fail("stage.parachute.reefing: area fractions must ascend in (0, 1), holds >= 0");
                    }
                    p.reefing.push_back({*fraction, *hold});
                    previous = *fraction;
                }
            }
            s.parachutes.push_back(p);
        }
        int drogues = 0, mains = 0;
        for (const auto& p : s.parachutes) (p.role == "drogue" ? drogues : mains)++;
        if (drogues > 1 || mains > 1) doc.fail("stage.parachute: at most one drogue set and one main set");
        for (const auto& d : s.parachutes) {
            for (const auto& m : s.parachutes) {
                if (d.role == "drogue" && m.role == "main" && !(m.deploy_height_m < d.deploy_height_m)) {
                    doc.fail("stage.parachute: the mains must deploy below the drogues");
                }
            }
        }
    }
```

In the `recovery` block, after `s.recovery_method = ...`:

```cpp
        if (s.recovery_method == "splashdown") {
            s.max_splashdown_speed_mps = doc.positive(*recovery, "max_splashdown_speed_mps");
        }
```

(`doc.positive` already fails on a missing key. Check `detail::Document` before relying on it; if it returns 0 for a
missing key, add `if (!(*recovery)["max_splashdown_speed_mps"]) doc.fail(...)`.)

- [ ] **Step 5: Implement the drag model**

`core/include/starsim/aero/parachute.hpp`:

```cpp
#pragma once

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::aero {

// Drag area C_D S (m^2) of ONE canopy `time_since_deploy_s` after line stretch. Each reefing stage, and then the final
// opening, fills from the previous area in fill_time_s with the area growing as t^2 (the filling-time model of Knacke,
// Parachute Recovery Systems Design Manual, NWC TP 6575 [public domain]) and holds for its hold_s. Massless canopy (spec
// d4: no added mass, no pendulum body), so the opening load is q C_D S with no opening-shock factor.
double parachute_drag_area_m2(const config::ParachuteConfig& parachute, double time_since_deploy_s);

// Drag of `drag_area_m2` (all canopies of a set together) on a riser point moving through the air at
// `point_air_velocity_body_mps`: -1/2 rho |v| v C_D S.
math::Vec3 parachute_force_body_n(double drag_area_m2, const math::Vec3& point_air_velocity_body_mps,
                                  double density_kg_m3);

}  // namespace starsim::aero
```

`core/src/aero/parachute.cpp`:

```cpp
#include "starsim/aero/parachute.hpp"

namespace starsim::aero {

double parachute_drag_area_m2(const config::ParachuteConfig& p, double time_since_deploy_s) {
    if (time_since_deploy_s <= 0.0) return 0.0;
    double t = time_since_deploy_s;
    double from = 0.0;
    const auto filling = [&](double to) {
        const double x = t / p.fill_time_s;
        return p.drag_area_m2 * (from + (to - from) * x * x);
    };
    for (const auto& stage : p.reefing) {
        if (t < p.fill_time_s) return filling(stage.area_fraction);
        t -= p.fill_time_s;
        if (t < stage.hold_s) return p.drag_area_m2 * stage.area_fraction;
        t -= stage.hold_s;
        from = stage.area_fraction;
    }
    if (t < p.fill_time_s) return filling(1.0);
    return p.drag_area_m2;
}

math::Vec3 parachute_force_body_n(double drag_area_m2, const math::Vec3& v, double density_kg_m3) {
    return v * (-0.5 * density_kg_m3 * norm(v) * drag_area_m2);
}

}  // namespace starsim::aero
```

- [ ] **Step 6: Configure Crew Dragon** (`config/vehicles/crew_dragon.toml`)

Add these to `[sources]`:

```toml
nasa_demo2 = "NASA, Demo-2 return timeline (2020-08-02): drogues ~18,000 ft at ~350 mph, mains ~6,000 ft at ~119 mph, splashdown ~3 min later"
spacex_crew1 = "SpaceX, Crew-1 return (2021-05-02): two drogues, four mains at ~6,500 ft"
splashdown_speed = "Crew-7 coverage (Spectrum News 2024-03-11): ~16 mph at splashdown; TechCrunch Demo-2 preview: ~15 mph"
```

In the capsule's `[stage.recovery]`:

```toml
[stage.recovery]
method = "splashdown"                    # d3: guided entry to the drogue gate; d4: parachutes and splashdown
max_splashdown_speed_mps = 9.0           # [E: spec] faster sink at the water is an impact
```

After `[stage.heat_shield]`:

```toml
# Parachutes [S:nasa_demo2, spacex_crew1]: two drogues at the gate, released when the four mains open. Canopy drag areas
# [D] from the published speeds with a 10.0 t capsule [E] and its own subsonic C_A S 12.2 m^2: drogues 53 m/s (119 mph) at
# 1.8 km -> 68 m^2 in all; mains 7.0 m/s (15-16 mph [S:splashdown_speed]) at sea level -> 3270 m^2 in all (a 35 m Mark 3
# canopy at C_D ~0.85). Reefing, fill times and load limits [E] (no public data): sized so that the crew's sustained load
# stays under 3.6 g (0.9 x crew_load_limit_g).
[[stage.parachute]]
id = "drogue"
role = "drogue"
count = 2
drag_area_m2 = 28.0                      # [D] per canopy
attach_station_m = 3.6                   # [E] riser bridle under the nose cone, on the axis
fill_time_s = 1.0                        # [E]
reefing = [[0.35, 4.0]]                  # [E] one reefed stage
max_load_n = 200000.0                    # [E] per canopy
deploy_height_m = 5500.0                 # [S:nasa_demo2] ~18,000 ft
max_deploy_dynamic_pressure_pa = 15000.0 # [E] gate dispersion (d3: 160-178 m/s at 5.5 km) gives <= 11 kPa

[[stage.parachute]]
id = "main"
role = "main"
count = 4
drag_area_m2 = 815.0                     # [D] per canopy
attach_station_m = 3.6                   # [E]
fill_time_s = 2.0                        # [E]
reefing = [[0.06, 4.0], [0.20, 4.0], [0.45, 4.0]]  # [E] three reefed stages
max_load_n = 150000.0                    # [E] per canopy
deploy_height_m = 1800.0                 # [S:nasa_demo2] ~6,000 ft (Crew-1: ~6,500 ft)
```

- [ ] **Step 7: Add the sources to CMake and run the tests**

Run: build, then `build/core/tests/starsim_core_tests.exe "[parachute]"`
Expected: all 5 test cases pass. Then run `python tools/vehicle_report.py crew_dragon`. Expected: it prints the
report without an error. If it rejects unknown keys, teach it to skip `parachute`.

- [ ] **Step 8: Commit**

```bash
git add core/include/starsim/config/vehicle_config.hpp core/src/config/vehicle_config.cpp core/include/starsim/aero/parachute.hpp core/src/aero/parachute.cpp core/CMakeLists.txt core/tests/CMakeLists.txt core/tests/test_parachute.cpp config/vehicles/crew_dragon.toml
git commit -m "feat(aero): parachute sets with reefed drag area and riser-point drag; Crew Dragon drogues and mains"
```

---

### Task 2: Parachutes on the vehicle and in the body physics

**Files:**
- Modify: `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp`
- Modify: `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp`
- Test: `core/tests/test_parachute.cpp`

**Interfaces:**
- Consumes: `config::ParachuteConfig`, `aero::parachute_drag_area_m2`, `aero::parachute_force_body_n` (Task 1)
- Produces:
  - `enum class vehicle::ParachuteState { Stowed, Open, Released, Torn }`
  - `struct vehicle::Parachute { config::ParachuteConfig config; double attach_station_m; ParachuteState state; double deployed_s; double drag_area_m2; double peak_load_n; }` (the drag area covers all canopies; the peak load is per canopy)
  - `Stage::parachutes` (`std::vector<Parachute>`)
  - `bool Vehicle::deploy_parachutes(const std::string& role)`: stowed sets of that role open; true if any did
  - `void Vehicle::release_parachutes(const std::string& role)`
  - `void Vehicle::update_parachutes(double dt_s)`
  - `bool Vehicle::parachutes_out() const`: any set open
  - `bool Vehicle::parachute_torn() const`
  - `void Vehicle::record_parachute_load(size_t stage, size_t set, double load_per_canopy_n)`: records the peak, and tears the set above `max_load_n`

- [ ] **Step 1: Write the failing tests** (append to `core/tests/test_parachute.cpp`; add the includes
  `starsim/vehicle/flight_body.hpp`, `starsim/vehicle/vehicle.hpp`, `starsim/environment/us76_atmosphere.hpp`,
  `starsim/environment/point_mass_gravity.hpp`, `starsim/frames/geodetic.hpp`, and the fixed-Earth orientation and
  environment helpers from `test_flight_body.cpp`. Copy the small helper that builds a
  `vehicle::FlightEnvironment` with a non-rotating Earth from that file; do not include another test's .cpp.)

```cpp
namespace {

// The capsule alone, RCS tank at `propellant_kg` (10.0 t with 1.4 t: the parachute sizing mass).
config::VehicleConfig capsule_only(double propellant_kg = 1400.0) {
    auto c = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
    c.stages.erase(c.stages.begin());
    for (auto& t : c.stages.front().tanks) t.mass_kg = propellant_kg;
    return c;
}

// Capsule `height_m` above the (non-rotating, spherical-gravity) test Earth's surface at the equator / 0 deg, falling
// vertically at `sink_mps`, heat shield down (body +x up).
std::unique_ptr<vehicle::FlightBody> falling_capsule(double height_m, double sink_mps, double propellant_kg = 1400.0) {
    const Vec3 r = frames::geodetic_to_fixed({0.0, 0.0, height_m});
    const Vec3 up = math::normalized(r);
    vehicle::RigidBodyState s;
    s.position_gcrf_m = r;
    s.velocity_gcrf_mps = up * -sink_mps;
    s.attitude = math::Quat::from_axis_angle({0.0, 1.0, 0.0}, -constants::pi / 2.0);  // body +x along GCRF +x (= up here)
    auto body = std::make_unique<vehicle::FlightBody>(
        std::make_unique<vehicle::Vehicle>(capsule_only(propellant_kg), STARSIM_CONFIG_DIR), s);
    body->set_ground_height(0.0);
    return body;
}

// Steps `body` for `seconds` at 200 Hz without commands; returns the peak aerodynamic load (g).
double fall(vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch& t, double seconds) {
    double peak = 0.0;
    for (int i = 0; i < static_cast<int>(seconds * 200.0) && !body.impacted() && !body.resting(); ++i) {
        body.step(env, t, 0.005, {});
        t += 0.005;
        peak = std::max(peak, norm(body.aero_acceleration_body_mps2()) / constants::standard_gravity_mps2);
    }
    return peak;
}

double sink_of(const vehicle::FlightBody& body) {
    return -dot(body.state().velocity_gcrf_mps, math::normalized(body.state().position_gcrf_m));
}

}  // namespace

TEST_CASE("Vehicle opens, releases and tears parachute sets", "[parachute][vehicle]") {
    vehicle::Vehicle v(capsule_only(), STARSIM_CONFIG_DIR);
    const auto& sets = v.stages().front().parachutes;
    REQUIRE(sets.size() == 2);
    CHECK_FALSE(v.parachutes_out());
    CHECK(v.deploy_parachutes("drogue"));
    CHECK_FALSE(v.deploy_parachutes("drogue"));  // already out
    CHECK(v.parachutes_out());
    v.update_parachutes(1.0);
    CHECK(sets[0].drag_area_m2 == Catch::Approx(2 * 28.0 * 0.35));
    CHECK(sets[1].drag_area_m2 == 0.0);
    v.release_parachutes("drogue");
    CHECK(sets[0].state == vehicle::ParachuteState::Released);
    CHECK(sets[0].drag_area_m2 == 0.0);
    CHECK_FALSE(v.parachutes_out());
    CHECK(v.deploy_parachutes("main"));
    v.record_parachute_load(0, 1, 100000.0);
    CHECK(sets[1].peak_load_n == 100000.0);
    CHECK_FALSE(v.parachute_torn());
    v.record_parachute_load(0, 1, 160000.0);  // > 150 kN per canopy
    CHECK(sets[1].state == vehicle::ParachuteState::Torn);
    CHECK(v.parachute_torn());
    CHECK_FALSE(v.parachutes_out());
}

TEST_CASE("Parachute attachment stations follow the capsule when the trunk separates", "[parachute][vehicle]") {
    vehicle::Vehicle stack(config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon"), STARSIM_CONFIG_DIR);
    const double in_stack = stack.stages().back().parachutes.front().attach_station_m;
    const double trunk_length = stack.stages().front().config.length_m;
    CHECK(in_stack == Catch::Approx(trunk_length + 3.6));
    auto capsule = stack.separate(1);
    CHECK(capsule->stages().front().parachutes.front().attach_station_m == Catch::Approx(3.6));
}

TEST_CASE("Under its drogues the capsule falls at the published 119 mph at 6,000 ft", "[parachute][body]") {
    const auto env = test_environment();  // copied helper: non-rotating Earth, point-mass gravity, US76
    time::Epoch t;
    auto body = falling_capsule(3000.0, 60.0);
    body->vehicle().deploy_parachutes("drogue");
    fall(*body, env, t, 25.0);  // settles to its terminal speed long before 1.8 km
    while (body->telemetry(env, t).altitude_m > 1800.0) fall(*body, env, t, 0.1);
    CHECK(sink_of(*body) == Catch::Approx(53.0).epsilon(0.03));  // [S:nasa_demo2] ~119 mph
    // hangs heat shield down: body +x within 5 deg of up
    const Vec3 x = body->state().attitude.rotate({1.0, 0.0, 0.0});
    CHECK(dot(x, math::normalized(body->state().position_gcrf_m)) > std::cos(5.0 * constants::deg_to_rad));
}

TEST_CASE("Under its mains the capsule reaches the water at the published ~16 mph", "[parachute][body]") {
    const auto env = test_environment();
    time::Epoch t;
    auto body = falling_capsule(400.0, 8.0);
    body->vehicle().deploy_parachutes("main");
    fall(*body, env, t, 30.0);  // through the reefing stages to full
    while (body->telemetry(env, t).altitude_m > 5.0) fall(*body, env, t, 0.1);
    CHECK(sink_of(*body) == Catch::Approx(7.0).epsilon(0.03));  // [S:splashdown_speed] 15-16 mph
}

TEST_CASE("Reefed openings keep the crew under 3.6 g and the canopies under their limits", "[parachute][body]") {
    const auto env = test_environment();
    time::Epoch t;
    auto body = falling_capsule(5500.0, 175.0);  // the fast end of the d3 gate dispersion
    body->vehicle().deploy_parachutes("drogue");
    double peak = fall(*body, env, t, 20.0);
    const auto& sets = body->vehicle().stages().front().parachutes;
    CHECK(sets[0].state == vehicle::ParachuteState::Open);
    CHECK(sets[0].peak_load_n < 0.8 * sets[0].config.max_load_n);
    while (body->telemetry(env, t).altitude_m > 1800.0) fall(*body, env, t, 0.1);
    body->vehicle().release_parachutes("drogue");
    body->vehicle().deploy_parachutes("main");
    peak = std::max(peak, fall(*body, env, t, 40.0));
    CHECK(sets[1].state == vehicle::ParachuteState::Open);
    CHECK(sets[1].peak_load_n < 0.8 * sets[1].config.max_load_n);
    CHECK(peak < 3.6);
    CHECK_FALSE(body->crew_load_warning());
    std::printf("[validation] parachute openings: peak %.2f g, drogue %.0f kN, main %.0f kN per canopy\n", peak,
                sets[0].peak_load_n / 1000.0, sets[1].peak_load_n / 1000.0);
}

TEST_CASE("Mains opened at drogue speed tear", "[parachute][body]") {
    const auto env = test_environment();
    time::Epoch t;
    auto body = falling_capsule(5500.0, 160.0);
    body->vehicle().deploy_parachutes("main");
    fall(*body, env, t, 5.0);
    CHECK(body->vehicle().parachute_torn());
    CHECK(body->vehicle().stages().front().parachutes[1].state == vehicle::ParachuteState::Torn);
}
```

- [ ] **Step 2: Run them and confirm they fail.** Expected: compile errors (`deploy_parachutes`, `ParachuteState`, ...).

- [ ] **Step 3: Vehicle state** (`vehicle.hpp`)

After `struct Flap`:

```cpp
enum class ParachuteState { Stowed, Open, Released, Torn };  // Open: inflating, reefed or full

// A parachute set on a stage (d4): `drag_area_m2` covers all its canopies.
struct Parachute {
    config::ParachuteConfig config;
    double attach_station_m = 0.0;   // vehicle body frame
    ParachuteState state = ParachuteState::Stowed;
    double deployed_s = 0.0;         // since line stretch
    double drag_area_m2 = 0.0;       // now, all canopies
    double peak_load_n = 0.0;        // per canopy, since deployment
};
```

In `struct Stage`, after `flap_table`: `std::vector<Parachute> parachutes;`.

Add these public `Vehicle` members, next to the landing legs:

```cpp
    // Parachute sets ([[stage.parachute]], d4). deploy opens the stowed sets of `role` ("drogue" / "main"); a released
    // or torn set stays so. update grows the open sets' drag areas (aero::parachute_drag_area_m2).
    bool deploy_parachutes(const std::string& role);
    void release_parachutes(const std::string& role);
    void update_parachutes(double dt_s);
    bool parachutes_out() const;
    bool parachute_torn() const;
    // Opening-load judgement (FlightBody): the peak per canopy; above the set's max_load_n the set tears.
    void record_parachute_load(size_t stage, size_t set, double load_per_canopy_n);
    // The lowest stage recovers by splashdown ([stage.recovery] method = "splashdown").
    bool splashdown_recovery() const;
    double max_splashdown_speed_mps() const;
```

- [ ] **Step 4: Implement them** (`vehicle.cpp`)

In the constructor where flaps are built (`config.flaps` loop), `station` is the stage's base station:

```cpp
        for (const auto& p : stage_config.parachutes) {
            vehicle::Parachute chute;
            chute.config = p;
            chute.attach_station_m = station + p.attach_station_m;
            stage.parachutes.push_back(chute);
        }
```

In `separate()`, in the per-stage shift loop: `for (auto& chute : stage.parachutes) chute.attach_station_m -= shift;`

The methods, with `#include "starsim/aero/parachute.hpp"`:

```cpp
bool Vehicle::deploy_parachutes(const std::string& role) {
    bool any = false;
    for (auto& stage : stages_) {
        for (auto& chute : stage.parachutes) {
            if (chute.config.role != role || chute.state != ParachuteState::Stowed) continue;
            chute.state = ParachuteState::Open;
            chute.deployed_s = 0.0;
            any = true;
        }
    }
    return any;
}

void Vehicle::release_parachutes(const std::string& role) {
    for (auto& stage : stages_) {
        for (auto& chute : stage.parachutes) {
            if (chute.config.role != role || chute.state != ParachuteState::Open) continue;
            chute.state = ParachuteState::Released;
            chute.drag_area_m2 = 0.0;
        }
    }
}

void Vehicle::update_parachutes(double dt_s) {
    for (auto& stage : stages_) {
        for (auto& chute : stage.parachutes) {
            if (chute.state != ParachuteState::Open) continue;
            chute.deployed_s += dt_s;
            chute.drag_area_m2 = chute.config.count * aero::parachute_drag_area_m2(chute.config, chute.deployed_s);
        }
    }
}

bool Vehicle::parachutes_out() const {
    for (const auto& stage : stages_) {
        for (const auto& chute : stage.parachutes) {
            if (chute.state == ParachuteState::Open) return true;
        }
    }
    return false;
}

bool Vehicle::parachute_torn() const {
    for (const auto& stage : stages_) {
        for (const auto& chute : stage.parachutes) {
            if (chute.state == ParachuteState::Torn) return true;
        }
    }
    return false;
}

void Vehicle::record_parachute_load(size_t stage, size_t set, double load_per_canopy_n) {
    auto& chute = stages_.at(stage).parachutes.at(set);
    if (chute.state != ParachuteState::Open) return;
    chute.peak_load_n = std::max(chute.peak_load_n, load_per_canopy_n);
    if (load_per_canopy_n > chute.config.max_load_n) {  // all canopies of a set carry the same load: they tear together
        chute.state = ParachuteState::Torn;
        chute.drag_area_m2 = 0.0;
    }
}

bool Vehicle::splashdown_recovery() const {
    return !stages_.empty() && stages_.front().config.recovery_method == "splashdown";
}

double Vehicle::max_splashdown_speed_mps() const {
    return stages_.empty() ? 0.0 : stages_.front().config.max_splashdown_speed_mps;
}
```

- [ ] **Step 5: Body physics** (`flight_body.cpp`)

In `evaluate()`, after the flap loop and before `f.moment_body_nm += ctx.rcs_torque_nm;`:

```cpp
    // Parachutes: drag of the riser point's local flow (incl. omega x r: the hanging capsule's damping), massless canopies.
    for (const auto& stage : ctx.vehicle.stages()) {
        for (const auto& chute : stage.parachutes) {
            if (chute.state != ParachuteState::Open || chute.drag_area_m2 <= 0.0) continue;
            const Vec3 arm{chute.attach_station_m - f.mass.cg_station_m, -f.mass.cg_offset_m.y, -f.mass.cg_offset_m.z};
            const Vec3 point_velocity = air_velocity_body + cross(s.angular_velocity_body_radps, arm);
            const Vec3 force = aero::parachute_force_body_n(chute.drag_area_m2, point_velocity, air.density_kg_m3);
            f.nongravitational_body_n += force;
            f.aero_body_n += force;
            f.moment_body_nm += cross(arm, force);
        }
    }
```

In `step()`, after `vehicle_->update_flaps(dt_s, command);`, add `vehicle_->update_parachutes(dt_s);`. After
`update_air_data(env, epoch);`, add `judge_parachute_loads();`, a new private member:

```cpp
// Opening loads: q at each open set's riser point (air data of this tick) times one canopy's drag area. The drag area
// already grew this tick, so the load is that of the area the tick will fly with.
void FlightBody::judge_parachute_loads() {
    const auto mass = vehicle_->mass_properties();
    const auto& stages = vehicle_->stages();
    for (size_t i = 0; i < stages.size(); ++i) {
        for (size_t k = 0; k < stages[i].parachutes.size(); ++k) {
            const auto& chute = stages[i].parachutes[k];
            if (chute.state != ParachuteState::Open) continue;
            const Vec3 arm{chute.attach_station_m - mass.cg_station_m, -mass.cg_offset_m.y, -mass.cg_offset_m.z};
            const Vec3 v = air_data_.air_velocity_body_mps + cross(state_.angular_velocity_body_radps, arm);
            const double q = 0.5 * air_data_.air.density_kg_m3 * dot(v, v);
            vehicle_->record_parachute_load(i, k, q * chute.drag_area_m2 / chute.config.count);
        }
    }
}
```

The body's `aero_acceleration_body_mps2_` and `crew_load_warning_` are evaluated only for single stages with a heat
shield. The capsule is such a stage, so the parachute forces count towards the crew load.

- [ ] **Step 6: Run the tests.** Build, then run `starsim_core_tests.exe "[parachute]"`.
  Expected: all pass. The reefing / fill / load values in `crew_dragon.toml` are [E] and may be tuned until
  "Reefed openings ..." passes. Keep the area fractions ascending, change the 815 / 28 m^2 [D] sizes only if their
  speed tests fail, and ledger every changed value as a ruling. Then run `starsim_core_tests.exe "[dragon]"`.
  Expected: the d3 tests are unchanged, because no parachute is deployed there.

- [ ] **Step 7: Commit**

```bash
git add core/include/starsim/vehicle core/src/vehicle core/tests/test_parachute.cpp config/vehicles/crew_dragon.toml
git commit -m "feat(vehicle): parachute sets pull at their riser point; opening loads tear a set beyond its limit"
```

---

### Task 3: Parachute sequencer

**Files:**
- Create: `core/include/starsim/gnc/parachute_sequencer.hpp`, `core/src/gnc/parachute_sequencer.cpp`
- Modify: `core/CMakeLists.txt` (after `src/gnc/capsule_entry_control.cpp`)
- Test: `core/tests/test_parachute.cpp`

**Interfaces:**
- Consumes: `config::ParachuteConfig` (Task 1)
- Produces:
  - `enum class gnc::ParachutePhase { Armed, Drogues, Mains, Down }`
  - `const char* gnc::to_string(ParachutePhase)`: "bereit", "Bremsschirme", "Hauptschirme", "unten"
  - `struct gnc::ParachuteSequencerInput { double height_m; double dynamic_pressure_pa; double sink_mps; bool on_surface; }`
  - `struct gnc::ParachuteActions { bool deploy_drogues; bool release_drogues; bool deploy_mains; bool release_mains; }`
  - `class gnc::ParachuteSequencer { explicit ParachuteSequencer(const std::vector<config::ParachuteConfig>&); ParachuteActions update(const ParachuteSequencerInput&); ParachutePhase phase() const; double watch_height_m() const; std::optional<double> drogue_height_m() const; std::optional<double> main_height_m() const; }` (heights at which the sets were deployed)

- [ ] **Step 1: Write the failing tests** (append; include `starsim/gnc/parachute_sequencer.hpp`)

```cpp
namespace {

std::vector<config::ParachuteConfig> dragon_sets() {
    return config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon").stages.back().parachutes;
}

}  // namespace

TEST_CASE("Sequencer: drogues at their height, mains below, release on the water", "[parachute][gnc]") {
    gnc::ParachuteSequencer seq(dragon_sets());
    CHECK(seq.watch_height_m() == Catch::Approx(5500.0 + 1000.0));
    auto a = seq.update({6000.0, 9000.0, 160.0, false});
    CHECK_FALSE(a.deploy_drogues);
    a = seq.update({5490.0, 9000.0, 160.0, false});
    CHECK(a.deploy_drogues);
    CHECK(seq.phase() == gnc::ParachutePhase::Drogues);
    a = seq.update({3000.0, 2000.0, 60.0, false});
    CHECK_FALSE(a.deploy_mains);
    a = seq.update({1795.0, 1500.0, 53.0, false});
    CHECK(a.release_drogues);
    CHECK(a.deploy_mains);
    CHECK(seq.phase() == gnc::ParachutePhase::Mains);
    a = seq.update({0.0, 30.0, 7.0, true});
    CHECK(a.release_mains);
    CHECK(seq.phase() == gnc::ParachutePhase::Down);
    CHECK(seq.drogue_height_m() == 5490.0);
    CHECK(seq.main_height_m() == 1795.0);
    a = seq.update({0.0, 0.0, 0.0, true});
    CHECK_FALSE(a.release_mains);  // once
}

TEST_CASE("Sequencer: the drogues wait for the dynamic pressure; the mains still come at their height", "[parachute][gnc]") {
    gnc::ParachuteSequencer seq(dragon_sets());
    CHECK_FALSE(seq.update({5400.0, 20000.0, 200.0, false}).deploy_drogues);  // > 15 kPa
    CHECK(seq.phase() == gnc::ParachutePhase::Armed);
    const auto a = seq.update({1790.0, 18000.0, 190.0, false});
    CHECK(a.deploy_mains);  // backup: the mains go at their height whatever the drogues did
    CHECK_FALSE(a.release_drogues);
    CHECK(seq.phase() == gnc::ParachutePhase::Mains);
}

TEST_CASE("Sequencer: below the main height at once the mains open", "[parachute][gnc]") {
    gnc::ParachuteSequencer seq(dragon_sets());
    const auto a = seq.update({1000.0, 1000.0, 40.0, false});
    CHECK(a.deploy_mains);
    CHECK_FALSE(a.deploy_drogues);
}

TEST_CASE("Sequencer: an ascending vehicle never deploys", "[parachute][gnc]") {
    gnc::ParachuteSequencer seq(dragon_sets());
    for (double h = 0.0; h < 10000.0; h += 100.0) {
        const auto a = seq.update({h, 5000.0, -100.0, false});
        CHECK_FALSE(a.deploy_drogues);
        CHECK_FALSE(a.deploy_mains);
    }
    CHECK(seq.phase() == gnc::ParachutePhase::Armed);
}

TEST_CASE("Sequencer: resting on the surface before any deployment ends the sequence", "[parachute][gnc]") {
    gnc::ParachuteSequencer seq(dragon_sets());
    const auto a = seq.update({0.0, 0.0, 0.0, true});
    CHECK_FALSE(a.deploy_mains);
    CHECK(seq.phase() == gnc::ParachutePhase::Down);
}
```

- [ ] **Step 2: Run them and confirm they fail.** Expected: compile error, `parachute_sequencer.hpp` missing.

- [ ] **Step 3: Implement**

`parachute_sequencer.hpp`:

```cpp
#pragma once

#include <optional>
#include <vector>

#include "starsim/config/vehicle_config.hpp"

namespace starsim::gnc {

enum class ParachutePhase { Armed, Drogues, Mains, Down };
const char* to_string(ParachutePhase phase);

struct ParachuteSequencerInput {
    double height_m = 0.0;             // above the ground / sea
    double dynamic_pressure_pa = 0.0;
    double sink_mps = 0.0;             // > 0 descending
    bool on_surface = false;           // resting on the ground / water
};

struct ParachuteActions {
    bool deploy_drogues = false;
    bool release_drogues = false;
    bool deploy_mains = false;
    bool release_mains = false;
};

// Parachute deployment of a descending capsule (d4; Crew Dragon [S]: drogues ~5.5 km, swapped for the mains at ~1.8 km).
// Triggers are the height plus, for a set with max_deploy_dynamic_pressure_pa, a dynamic-pressure check (the
// deployment waits above it). The mains are the backup: they go at their height whatever the drogues did (the
// drogues are released only if they are out). On the surface the mains are released. Only descending flight deploys.
class ParachuteSequencer {
public:
    explicit ParachuteSequencer(const std::vector<config::ParachuteConfig>& sets);
    ParachuteActions update(const ParachuteSequencerInput& in);
    ParachutePhase phase() const { return phase_; }
    // Below this height the session ticks (no analytic coast past a deployment height).
    double watch_height_m() const;
    std::optional<double> drogue_height_m() const { return drogue_height_m_; }  // where the drogues were deployed
    std::optional<double> main_height_m() const { return main_height_m_; }

private:
    std::optional<double> drogue_height_m_, main_height_m_;
    const config::ParachuteConfig* drogue_ = nullptr;
    const config::ParachuteConfig* main_ = nullptr;
    std::vector<config::ParachuteConfig> sets_;
    ParachutePhase phase_ = ParachutePhase::Armed;
    bool drogues_out_ = false;
};

}  // namespace starsim::gnc
```

`parachute_sequencer.cpp`:

```cpp
#include "starsim/gnc/parachute_sequencer.hpp"

#include <algorithm>

namespace starsim::gnc {

namespace {

constexpr double watch_margin_m = 1000.0;  // [E] ticked from this far above the highest deployment height

bool q_allows(const config::ParachuteConfig& set, double q_pa) {
    return set.max_deploy_dynamic_pressure_pa <= 0.0 || q_pa <= set.max_deploy_dynamic_pressure_pa;
}

}  // namespace

const char* to_string(ParachutePhase phase) {
    switch (phase) {
        case ParachutePhase::Armed: return "bereit";
        case ParachutePhase::Drogues: return "Bremsschirme";
        case ParachutePhase::Mains: return "Hauptschirme";
        case ParachutePhase::Down: return "unten";
    }
    return "?";
}

ParachuteSequencer::ParachuteSequencer(const std::vector<config::ParachuteConfig>& sets) : sets_(sets) {
    for (const auto& s : sets_) (s.role == "drogue" ? drogue_ : main_) = &s;
}

double ParachuteSequencer::watch_height_m() const {
    double h = 0.0;
    for (const auto& s : sets_) h = std::max(h, s.deploy_height_m);
    return h + watch_margin_m;
}

ParachuteActions ParachuteSequencer::update(const ParachuteSequencerInput& in) {
    ParachuteActions a;
    if (phase_ == ParachutePhase::Down) return a;
    if (in.on_surface) {
        a.release_mains = phase_ == ParachutePhase::Mains;
        a.release_drogues = drogues_out_ && phase_ == ParachutePhase::Drogues;
        phase_ = ParachutePhase::Down;
        return a;
    }
    if (in.sink_mps <= 0.0 || phase_ == ParachutePhase::Mains) return a;
    if (main_ && in.height_m <= main_->deploy_height_m && q_allows(*main_, in.dynamic_pressure_pa)) {
        a.deploy_mains = true;
        main_height_m_ = in.height_m;
        a.release_drogues = drogues_out_;
        drogues_out_ = false;
        phase_ = ParachutePhase::Mains;
        return a;
    }
    if (phase_ == ParachutePhase::Armed && drogue_ && in.height_m <= drogue_->deploy_height_m &&
        q_allows(*drogue_, in.dynamic_pressure_pa)) {
        a.deploy_drogues = true;
        drogue_height_m_ = in.height_m;
        drogues_out_ = true;
        phase_ = ParachutePhase::Drogues;
    }
    return a;
}

}  // namespace starsim::gnc
```

Note: `drogue_` / `main_` point into `sets_`, so the class must not be copied. Delete the copy constructor and copy
assignment, since the session holds it in a `unique_ptr`.

- [ ] **Step 4: Run** `starsim_core_tests.exe "[parachute][gnc]"`. Expected: 5 test cases pass.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/gnc/parachute_sequencer.hpp core/src/gnc/parachute_sequencer.cpp core/CMakeLists.txt core/tests/test_parachute.cpp
git commit -m "feat(gnc): parachute sequencer - drogues on height and dynamic pressure, mains on height, release on the water"
```

---

### Task 4: Splashdown contact and outcome

**Files:**
- Modify: `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp` (`resolve_ground_contact`)
- Modify: `core/include/starsim/sim/landing_outcome.hpp`, `core/src/sim/landing_outcome.cpp`
- Test: `core/tests/test_parachute.cpp`, `core/tests/test_landing_outcome.cpp`

**Interfaces:**
- Consumes: `Vehicle::splashdown_recovery()`, `Vehicle::max_splashdown_speed_mps()` (Task 2)
- Produces:
  - `std::optional<double> FlightBody::touchdown_sink_mps() const` (first contact)
  - `LandingOutcome::SplashedDown` with the string "gewassert"
  - `sim::landing_outcome`: a splashdown vehicle on the surface gives `SplashedDown` if it is resting, else `Impact`

- [ ] **Step 1: Write the failing tests** (append to `test_parachute.cpp`; include `starsim/sim/landing_outcome.hpp`)

```cpp
TEST_CASE("A capsule under its mains rests on the water: gewassert", "[parachute][splashdown]") {
    const auto env = test_environment();
    time::Epoch t;
    auto body = falling_capsule(30.0, 7.0);
    body->vehicle().deploy_parachutes("main");
    body->vehicle().update_parachutes(60.0);  // fully open
    fall(*body, env, t, 30.0);
    REQUIRE(body->touchdown_sink_mps());
    CHECK(*body->touchdown_sink_mps() == Catch::Approx(7.0).epsilon(0.05));
    CHECK(body->resting());
    CHECK_FALSE(body->impacted());
    sim::LaunchSite site;  // any site: a splashdown vehicle is judged by its sink rate, not by the shoreline
    CHECK(sim::landing_outcome(*body, site, {}) == sim::LandingOutcome::SplashedDown);
    CHECK(std::string(sim::to_string(sim::LandingOutcome::SplashedDown)) == "gewassert");
}

TEST_CASE("A capsule hitting the water faster than its limit is an impact", "[parachute][splashdown]") {
    const auto env = test_environment();
    time::Epoch t;
    auto body = falling_capsule(200.0, 50.0);  // drogue speed, no mains
    fall(*body, env, t, 30.0);
    REQUIRE(body->touchdown_sink_mps());
    CHECK(*body->touchdown_sink_mps() > 9.0);
    CHECK(body->impacted());
    sim::LaunchSite site;
    CHECK(sim::landing_outcome(*body, site, {}) == sim::LandingOutcome::Impact);
}

TEST_CASE("A tilted capsule still splashes down", "[parachute][splashdown]") {
    const auto env = test_environment();
    time::Epoch t;
    auto body = falling_capsule(1.0, 6.0);
    auto s = body->state();
    s.attitude = (math::Quat::from_axis_angle({0.0, 0.0, 1.0}, 30.0 * constants::deg_to_rad) * s.attitude).normalized();
    body->set_state(s);  // use the existing state setter; add `set_state` if FlightBody has none
    fall(*body, env, t, 5.0);
    CHECK(body->resting());  // the leg rule (<= 10 deg tilt) does not apply on water
}
```

In `test_landing_outcome.cpp`, extend the existing to-string test (or add one) with
`CHECK(std::string(sim::to_string(sim::LandingOutcome::SplashedDown)) == "gewassert");`.

- [ ] **Step 2: Run them and confirm they fail.** Expected: compile errors (`touchdown_sink_mps`, `SplashedDown`).

- [ ] **Step 3: Implement the contact** (`flight_body.cpp`, `resolve_ground_contact`, after `sink_rate` and `tilt` are
  computed, before the hard-contact check):

```cpp
    if (!touchdown_sink_mps_) touchdown_sink_mps_ = sink_rate;
    if (vehicle_->splashdown_recovery()) {
        // On the water (d4): any tilt, sink up to the vehicle's limit; no buoyancy (it rests at the surface).
        if (sink_rate > vehicle_->max_splashdown_speed_mps()) {
            impacted_ = true;
            return;
        }
    } else if (sink_rate > max_touchdown_sink_rate_mps || tilt > max_touchdown_tilt_rad) {
        impacted_ = true;
        return;
    }
```

(This replaces the existing `if (sink_rate > max_touchdown_sink_rate_mps || tilt > ...) { impacted_ = true; return; }`.)
In `flight_body.hpp` add the member `std::optional<double> touchdown_sink_mps_;` and the public accessor:

```cpp
    // Sink rate (m/s, relative to the rotating Earth) at the first ground / water contact.
    std::optional<double> touchdown_sink_mps() const { return touchdown_sink_mps_; }
```

The resting capsule's later contacts happen at ~0 m/s and keep it resting. `touchdown_sink_mps_` holds the first one.

- [ ] **Step 4: Implement the outcome** (`landing_outcome.hpp`: add `SplashedDown` to the enum, documented as "a
  splashdown vehicle (capsule) rests on the water"; `landing_outcome.cpp`: `case LandingOutcome::SplashedDown: return
  "gewassert";` and in `landing_outcome()` after the `Flying` check):

```cpp
    if (body.vehicle().splashdown_recovery()) {  // capsule: judged by its sink at the water (FlightBody), not the site
        return body.resting() ? LandingOutcome::SplashedDown : LandingOutcome::Impact;
    }
```

- [ ] **Step 5: Run** `starsim_core_tests.exe "[parachute]"` and `"[outcome]"`, using the tag
  `test_landing_outcome.cpp` uses. Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add core/include/starsim/vehicle/flight_body.hpp core/src/vehicle/flight_body.cpp core/include/starsim/sim/landing_outcome.hpp core/src/sim/landing_outcome.cpp core/tests/test_parachute.cpp core/tests/test_landing_outcome.cpp
git commit -m "feat(sim): splashdown - a capsule rests on the water up to its sink limit and is judged gewassert"
```

---

### Task 5: Parachutes in the flight session

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`
- Create: `core/tests/test_capsule_splashdown.cpp`; Modify: `core/tests/CMakeLists.txt`
- Modify: `core/tests/test_capsule_entry.cpp` (the d3 return test loop and its M1 check)

**Interfaces:**
- Consumes: `gnc::ParachuteSequencer` (Task 3), `Vehicle::deploy_parachutes` / `release_parachutes` /
  `parachutes_out` (Task 2), `LandingOutcome::SplashedDown` (Task 4)
- Produces:
  - `Flying::parachutes` (`std::unique_ptr<gnc::ParachuteSequencer>`), created when the body's lowest stage has parachute sets
  - `const gnc::ParachuteSequencer* FlightSession::parachute_sequencer(size_t index) const` (nullptr: none)
  - `CapsuleReturnStep::{Drogues, Mains, SplashedDown}` (after `Gate`), strings "Bremsschirme", "Hauptschirme", "gewassert"
  - `double FlightSession::splashdown_miss_m(size_t index) const`: horizontal distance (m) from the zone centre; throws `std::logic_error` without a zone
  - `place_body`: a splashdown vehicle's ground is the sea (height 0)

- [ ] **Step 1: Write the failing session tests** (`core/tests/test_capsule_splashdown.cpp`)

```cpp
// Crew Dragon under its parachutes in the flight session (d4): from the drogue gate to the water; review-focus cases.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/config/splashdown_zone_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/parachute_sequencer.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

const spice::SpiceEphemeris& spice_ephemeris() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& spice_earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

config::VehicleConfig capsule_config(double propellant_kg = 1400.0) {
    auto c = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
    c.stages.erase(c.stages.begin());
    for (auto& t : c.stages.front().tanks) t.mass_kg = propellant_kg;
    return c;
}

// The capsule `height_m` above the zone centre, `speed_mps` relative to the Earth, `gamma_rad` below the horizontal
// heading north, heat shield first (body -x along the motion).
vehicle::RigidBodyState falling_state(const config::SplashdownZoneConfig& zone, double height_m, double speed_mps,
                                      double gamma_rad, time::Epoch epoch) {
    const Vec3 r = frames::geodetic_to_fixed({zone.latitude_rad, zone.longitude_rad, height_m});
    const Vec3 up = math::normalized(r);
    const Vec3 east{-std::sin(zone.longitude_rad), std::cos(zone.longitude_rad), 0.0};
    const Vec3 north = cross(up, east);
    const Vec3 v_rel = (north * std::cos(gamma_rad) - up * std::sin(gamma_rad)) * speed_mps;
    const auto t = spice_earth().fixed_to_gcrf(epoch);
    vehicle::RigidBodyState s;
    s.position_gcrf_m = t.rotation * r;
    s.velocity_gcrf_mps = t.rotation * v_rel + cross(vehicle::earth_rotation_vector(t), s.position_gcrf_m);
    const Vec3 x = math::normalized(t.rotation * v_rel) * -1.0;  // body +x against the motion
    const Vec3 y = math::normalized(cross(t.rotation * east, x));
    const Vec3 z = cross(x, y);
    math::Mat3 m;
    const Vec3 columns[3] = {x, y, z};
    for (int c = 0; c < 3; ++c) {
        m(0, c) = columns[c].x;
        m(1, c) = columns[c].y;
        m(2, c) = columns[c].z;
    }
    s.attitude = math::Quat::from_matrix(m).normalized();
    return s;
}

struct Descent {
    double drogue_height_m = -1.0, drogue_speed_mps = -1.0, drogue_s = -1.0;
    double main_height_m = -1.0, main_speed_mps = -1.0, main_s = -1.0;
    double splashdown_s = -1.0, sink_mps = -1.0, miss_m = -1.0, peak_g = 0.0;
    sim::LandingOutcome outcome = sim::LandingOutcome::Flying;
};

// Flies body 0 (the capsule) of `session` until it is down or `max_s` passed, in `step_s` steps; records the events.
Descent fly_down(sim::FlightSession& session, double max_s = 900.0, double step_s = 0.05) {
    Descent d;
    const auto t0 = session.epoch();
    const size_t i = session.controlled_index();
    while (session.epoch() - t0 < max_s) {
        session.advance_to(session.epoch() + step_s, 1000000);
        const auto& body = session.body(i);
        const auto tel = session.telemetry(i);
        d.peak_g = std::max(d.peak_g, norm(body.aero_acceleration_body_mps2()) / constants::standard_gravity_mps2);
        const auto* seq = session.parachute_sequencer(i);
        if (seq && d.drogue_s < 0.0 && seq->drogue_height_m()) {
            d.drogue_s = session.epoch() - t0;
            d.drogue_height_m = *seq->drogue_height_m();
            d.drogue_speed_mps = tel.speed_relative_mps;
        }
        if (seq && d.main_s < 0.0 && seq->main_height_m()) {
            d.main_s = session.epoch() - t0;
            d.main_height_m = *seq->main_height_m();
            d.main_speed_mps = tel.speed_relative_mps;
        }
        if (body.resting() || body.impacted()) {
            d.splashdown_s = session.epoch() - t0;
            break;
        }
    }
    const auto& body = session.body(i);
    if (body.touchdown_sink_mps()) d.sink_mps = *body.touchdown_sink_mps();
    d.outcome = session.landing_outcome(i);
    if (session.splashdown_zone()) d.miss_m = session.splashdown_miss_m(i);
    return d;
}

struct Scenario {
    sim::FlightSession session{spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR};
    config::SplashdownZoneConfig zone = config::load_splashdown_zone(STARSIM_CONFIG_DIR, "pacific_san_diego");
    time::Epoch t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 14, 6, 0, 0.0});
    Scenario(double height_m, double speed_mps, double gamma_rad, config::VehicleConfig cfg = capsule_config()) {
        session.set_splashdown_zone(zone);
        session.place_body(cfg, falling_state(zone, height_m, speed_mps, gamma_rad, t0), t0);
    }
};

}  // namespace

TEST_CASE("From the drogue gate the capsule splashes down under its mains", "[dragon][splashdown][session]") {
    Scenario s(5600.0, 166.0, 60.0 * deg);  // d3's gate: 166 m/s, steep
    const auto d = fly_down(s.session);
    std::printf("[validation] gate to water: drogues %.2f km at %.0f m/s, mains %.2f km at %.0f m/s after %.0f s, "
                "splashdown %.0f s after the mains at %.1f m/s, peak %.2f g\n",
                d.drogue_height_m / 1000.0, d.drogue_speed_mps, d.main_height_m / 1000.0, d.main_speed_mps,
                d.main_s - d.drogue_s, d.splashdown_s - d.main_s, d.sink_mps, d.peak_g);
    CHECK(d.drogue_height_m == Catch::Approx(5500.0).epsilon(0.02));
    CHECK(d.main_height_m == Catch::Approx(1800.0).epsilon(0.02));
    CHECK(d.main_speed_mps == Catch::Approx(53.0).epsilon(0.15));         // [S] 119 mph
    CHECK(d.main_s - d.drogue_s < 90.0);                                   // [S] "less than a minute later" (+-15 %... and
    CHECK(d.main_s - d.drogue_s > 40.0);                                   //      Demo-2 2:44 -> 2:45 at minute resolution)
    CHECK(d.splashdown_s - d.main_s == Catch::Approx(180.0).epsilon(0.35)); // [S] Demo-2 2:45 -> 2:48
    CHECK(d.sink_mps == Catch::Approx(7.0).epsilon(0.30));                 // [S] 15-16 mph, spec tolerance 30 %
    CHECK(d.outcome == sim::LandingOutcome::SplashedDown);
    CHECK(d.peak_g < 3.6);
    CHECK_FALSE(s.session.body(0).crew_load_warning());
    CHECK_FALSE(s.session.body(0).vehicle().parachute_torn());
    CHECK(d.miss_m < 2000.0);  // the drift from 5.6 km over the zone stays inside the 2 km zone
}

TEST_CASE("The Dracos stay off under the parachutes", "[dragon][splashdown][session]") {
    Scenario s(5600.0, 166.0, 60.0 * deg);
    double rcs_after_drogues = -1.0;
    const auto t0 = s.session.epoch();
    while (s.session.epoch() - t0 < 120.0) {
        s.session.advance_to(s.session.epoch() + 0.05, 1000000);
        const auto* seq = s.session.parachute_sequencer(0);
        if (seq && seq->phase() != gnc::ParachutePhase::Armed && rcs_after_drogues < 0.0) {
            rcs_after_drogues = s.session.body(0).vehicle().rcs_propellant_kg();
        }
    }
    REQUIRE(rcs_after_drogues > 0.0);
    CHECK(s.session.body(0).vehicle().rcs_propellant_kg() == Catch::Approx(rcs_after_drogues).margin(1e-9));
}

TEST_CASE("Drogues open at their height under time warp", "[dragon][splashdown][session]") {
    Scenario s(12000.0, 200.0, 80.0 * deg);
    const auto d = fly_down(s.session, 900.0, 30.0);  // 30 s steps: coast segments wherever the session allows them
    const auto* seq = s.session.parachute_sequencer(0);
    REQUIRE(seq != nullptr);
    REQUIRE(seq->drogue_height_m());
    CHECK(*seq->drogue_height_m() == Catch::Approx(5500.0).margin(50.0));  // one tick of fall at most, not a coast
    REQUIRE(seq->main_height_m());
    CHECK(*seq->main_height_m() == Catch::Approx(1800.0).margin(10.0));
    CHECK(d.outcome == sim::LandingOutcome::SplashedDown);
}

TEST_CASE("A capsule placed below the main height opens its mains at once", "[dragon][splashdown][session]") {
    Scenario s(1200.0, 45.0, 85.0 * deg);
    s.session.advance_to(s.session.epoch() + 0.1);
    const auto& chutes = s.session.body(0).vehicle().stages().front().parachutes;
    CHECK(chutes[0].state == vehicle::ParachuteState::Stowed);
    CHECK(chutes[1].state == vehicle::ParachuteState::Open);
    const auto d = fly_down(s.session);
    CHECK(d.outcome == sim::LandingOutcome::SplashedDown);
}

TEST_CASE("Drogues held back by dynamic pressure: the mains tear and the capsule hits the water", "[dragon][splashdown][session]") {
    auto cfg = capsule_config();
    cfg.stages.front().parachutes[0].max_deploy_dynamic_pressure_pa = 100.0;  // never reached: the drogues never go
    Scenario s(5600.0, 166.0, 60.0 * deg, cfg);
    const auto d = fly_down(s.session);
    CHECK(d.drogue_s < 0.0);
    CHECK(s.session.body(0).vehicle().parachute_torn());
    CHECK(d.sink_mps > 9.0);
    CHECK(d.outcome == sim::LandingOutcome::Impact);
}

TEST_CASE("A capsule on its trunk has no parachute sequencer", "[dragon][splashdown][session]") {
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    const auto zone = config::load_splashdown_zone(STARSIM_CONFIG_DIR, "pacific_san_diego");
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 14, 6, 0, 0.0});
    session.set_splashdown_zone(zone);
    session.place_body(config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon"), falling_state(zone, 5000.0, 150.0, 60.0 * deg, t0), t0);
    session.advance_to(t0 + 1.0);
    CHECK(session.parachute_sequencer(0) == nullptr);
    CHECK_FALSE(session.body(0).vehicle().parachutes_out());
}
```

- [ ] **Step 2: Run them and confirm they fail.** Expected: compile errors (`parachute_sequencer`, `splashdown_miss_m`).

- [ ] **Step 3: Session changes** (`flight_session.hpp` / `.cpp`)

1. `#include "starsim/gnc/parachute_sequencer.hpp"`. In `Flying`, add
   `std::unique_ptr<gnc::ParachuteSequencer> parachutes;  // lowest stage with parachute sets (capsule)`.
2. `CapsuleReturnStep`: append `Drogues, Mains, SplashedDown` after `Gate`. Strings in `to_string`: `"Bremsschirme"`,
   `"Hauptschirme"`, `"gewassert"`.
3. Public accessors:

```cpp
    // Parachute sequence of body `index` (a capsule flying alone with [[stage.parachute]] sets), nullptr if none.
    const gnc::ParachuteSequencer* parachute_sequencer(size_t index) const { return bodies_.at(index).parachutes.get(); }
    // Horizontal distance (m) of body `index` from the splashdown zone's centre; throws std::logic_error without a zone.
    double splashdown_miss_m(size_t index) const;
```

4. Private: `void run_parachutes(Flying& flying);`.
5. In `advance_to`, inside the per-body loop, after `if (flying.ship_landing) run_ship_landing(flying);`:

```cpp
            if (!flying.parachutes && !flying.body->vehicle().stages().front().config.parachutes.empty()) {
                flying.parachutes = std::make_unique<gnc::ParachuteSequencer>(
                    flying.body->vehicle().stages().front().config.parachutes);
            }
            if (flying.parachutes && !flying.body->impacted()) run_parachutes(flying);
```

6. `run_parachutes`:

```cpp
void FlightSession::run_parachutes(Flying& flying) {
    auto& body = *flying.body;
    const auto t = earth_.fixed_to_gcrf(epoch_);
    const auto& s = body.state();
    const double height = frames::fixed_to_geodetic(transpose(t.rotation) * s.position_gcrf_m).height_m - body.ground_height_m();
    const math::Vec3 air = s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), s.position_gcrf_m);
    const double q = 0.5 * atmosphere_.at(height + body.ground_height_m()).density_kg_m3 * dot(air, air);
    const double sink = -dot(air, math::normalized(s.position_gcrf_m));
    const auto a = flying.parachutes->update({height, q, sink, body.resting()});
    auto& v = body.vehicle();
    if (a.release_drogues) v.release_parachutes("drogue");
    if (a.deploy_drogues) v.deploy_parachutes("drogue");
    if (a.deploy_mains) v.deploy_parachutes("main");
    if (a.release_mains) v.release_parachutes("main");
}
```

7. `command_for`, at the top of the capsule branch (`flying.entry_guidance && flying.entry_guidance->capsule()`), and
   for any body with a sequencer past `Armed`: no RCS command. Insert at the very start of `command_for`, after the
   running-groups loop:

```cpp
    if (flying.parachutes && flying.parachutes->phase() != gnc::ParachutePhase::Armed) {
        return command;  // no steering under the parachutes (user 2026-10-14): the Dracos stay off
    }
```

8. `coast_allowance_s`: in the per-body loop, replace `ground_tick_height_m` in the height check with
   `const double tick_height = flying.parachutes && flying.parachutes->phase() != gnc::ParachutePhase::Down ?
   std::max(ground_tick_height_m, flying.parachutes->watch_height_m()) : ground_tick_height_m;` and use `tick_height`
   in both places of that block.
9. `splashdown_miss_m`:

```cpp
double FlightSession::splashdown_miss_m(size_t index) const {
    if (!splashdown_zone_) throw std::logic_error("FlightSession: no splashdown zone set");
    const auto t = earth_.fixed_to_gcrf(epoch_);
    const math::Vec3 r = transpose(t.rotation) * bodies_.at(index).body->state().position_gcrf_m;
    const math::Vec3 z = frames::geodetic_to_fixed({splashdown_zone_->latitude_rad, splashdown_zone_->longitude_rad, 0.0});
    const math::Vec3 up = math::normalized(r);
    const math::Vec3 d = r - z;
    return norm(d - up * dot(d, up));
}
```

10. `place_body`: `body->set_ground_height(body->vehicle().splashdown_recovery() ? 0.0 : launch_site_.height_m);`
    (a capsule alone lands on the sea).
11. `run_capsule_return`: replace the `Entry` case's target, and add the cases:

```cpp
        case CapsuleReturnStep::Gate:
        case CapsuleReturnStep::Drogues:
        case CapsuleReturnStep::Mains:
            if (flying.parachutes) {
                switch (flying.parachutes->phase()) {
                    case gnc::ParachutePhase::Drogues: capsule_return_step_ = CapsuleReturnStep::Drogues; break;
                    case gnc::ParachutePhase::Mains: capsule_return_step_ = CapsuleReturnStep::Mains; break;
                    case gnc::ParachutePhase::Down:
                        capsule_return_step_ = CapsuleReturnStep::SplashedDown;
                        autopilot_.state = AutopilotState::Off;
                        break;
                    default: break;
                }
            }
            if (flying.body->impacted()) autopilot_.state = AutopilotState::Off;
            break;
```

    Check the ordering assumption. `AutopilotState::Off` leaves `capsule_return_step_` at its last value, so the
    caller can read it. `reset_mission_state()` / `set_autopilot_off` still reset it.

- [ ] **Step 4: Update the d3 return test** (`test_capsule_entry.cpp`)

- In `fly_capsule_return`, the loop condition `session.capsule_return_step() != sim::CapsuleReturnStep::Gate` becomes
  `static_cast<int>(session.capsule_return_step()) < static_cast<int>(sim::CapsuleReturnStep::Gate)`. The drogues now
  follow the gate within one 1 s step.
- In "Crew Dragon returns from orbit to the drogue gate", the M1 block expected an impact. Change it to:

```cpp
    // After the gate the capsule comes down under its parachutes (d4) and is gewassert; then the session coasts again
    s.advance_to(s.epoch() + 600.0, 1000000);
    CHECK(s.landing_outcome(s.controlled_index()) == sim::LandingOutcome::SplashedDown);
```

  Keep the coast check (`reached - before > 3000.0`).
- `r.reached_gate` stays `step >= Gate` by the same cast comparison.

- [ ] **Step 5: Run** `starsim_core_tests.exe "[splashdown]"`, then `"[dragon]"`, then the full suite with ctest.
  Expected: all green. If "From the drogue gate ..." misses a published figure, judge it by the tolerance. The reefing
  and fill values are [E] and may move. The drag areas [D] may move only if the speed checks of Task 2 still hold.
  Ledger every change. A miss outside the tolerances that cannot be traced to an [E] value is a finding: report it
  rather than fit it away.

- [ ] **Step 6: Commit**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/tests/test_capsule_splashdown.cpp core/tests/CMakeLists.txt core/tests/test_capsule_entry.cpp
git commit -m "feat(sim): parachute sequence per capsule - drogues at the gate, mains at 1.8 km, no RCS under canopies, splashdown"
```

---

### Task 6: Return mission, timeline evidence and campaign

**Files:**
- Modify: `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp`
- Create: `core/include/starsim/sim/capsule_return_mission.hpp`, `core/src/sim/capsule_return_mission.cpp`
- Modify: `core/CMakeLists.txt` (after `src/sim/ascent_mission.cpp`)
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (`place_in_orbit` overload)
- Create: `config/missions/crew_dragon_return.toml`
- Test: `core/tests/test_capsule_splashdown.cpp`
- Docs: `CLAUDE.md`, `config/README.md`

**Interfaces:**
- Consumes: everything above; `FlightSession::plan_capsule_return(gnc::DeorbitParameters)` (d3)
- Produces:
  - `config::CapsuleReturnMission { id, name, vehicle_id, zone_id, epoch_utc, periapsis_height_m, apoapsis_height_m, inclination_rad, raan_rad, argument_of_latitude_rad, rcs_propellant_kg, entry_interface_height_m, entry_flight_path_rad, nominal_bank_rad, max_crossrange_m, search_window_s }`
  - `config::CapsuleReturnMission config::load_capsule_return_mission(const std::filesystem::path&, std::string_view)`
  - `void FlightSession::place_in_orbit(const config::VehicleConfig&, const math::StateVector&, time::Epoch)`; the string overload delegates to it
  - `void sim::place_capsule_return_mission(FlightSession&, const config::CapsuleReturnMission&, const std::filesystem::path& config_dir, time::Epoch epoch)`
  - `gnc::DeorbitParameters sim::capsule_return_parameters(const config::CapsuleReturnMission&)`

- [ ] **Step 1: Write the failing loader test** (append to `test_capsule_splashdown.cpp`)

```cpp
TEST_CASE("Crew Dragon return mission loads", "[dragon][mission][config]") {
    const auto m = config::load_capsule_return_mission(STARSIM_CONFIG_DIR, "crew_dragon_return");
    CHECK(m.vehicle_id == "crew_dragon");
    CHECK(m.zone_id == "pacific_san_diego");
    CHECK(m.periapsis_height_m == 420000.0);
    CHECK(m.inclination_rad == Catch::Approx(51.6 * deg));
    CHECK(m.entry_flight_path_rad == Catch::Approx(-2.0 * deg));
    CHECK(m.nominal_bank_rad == Catch::Approx(30.0 * deg));
    CHECK(m.max_crossrange_m == 40000.0);
    const auto p = sim::capsule_return_parameters(m);
    CHECK(p.entry_flight_path_rad == m.entry_flight_path_rad);
    CHECK(p.gate_height_m == Catch::Approx(5500.0));
}
```

- [ ] **Step 2: Run it and confirm it fails.** Expected: compile error.

- [ ] **Step 3: Loader** (`mission_config.hpp` after `ShipReturnMission`)

```cpp
// Capsule return from orbit (config/missions/<id>.toml, d4): the start orbit, the splashdown zone and the deorbit
// planning settings (FlightSession::plan_capsule_return).
struct CapsuleReturnMission {
    std::string id;
    std::string name;
    std::string vehicle_id;
    std::string zone_id;  // config/splashdown_zones.toml
    time::UtcDateTime epoch_utc;
    double periapsis_height_m = 0.0;
    double apoapsis_height_m = 0.0;
    double inclination_rad = 0.0;
    double raan_rad = 0.0;
    double argument_of_latitude_rad = 0.0;
    double rcs_propellant_kg = 0.0;  // in the capsule's RCS tank at the start
    double entry_interface_height_m = 0.0;
    double entry_flight_path_rad = 0.0;
    double nominal_bank_rad = 0.0;
    double max_crossrange_m = 0.0;
    double search_window_s = 0.0;
};

CapsuleReturnMission load_capsule_return_mission(const std::filesystem::path& config_dir, std::string_view mission_id);
```

`mission_config.cpp`: implement it as `load_ship_return_mission` does, with these differences. `[mission]` has `zone`
instead of `stage` / `site`. `[orbit]` has `rcs_propellant_kg` (`doc.positive`) instead of `propellant_t`. The
`[deorbit]` keys and checks are the same. Reuse the same `detail::Document` calls.

- [ ] **Step 4: Placement** (`capsule_return_mission.hpp` / `.cpp`)

```cpp
#pragma once

#include <filesystem>

#include "starsim/config/mission_config.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

// Places the mission's capsule (on its trunk) in its orbit at `epoch` with the RCS tank at rcs_propellant_kg, the
// nose cone open and the splashdown zone set; plan_capsule_return(capsule_return_parameters(m)) then flies it home.
void place_capsule_return_mission(FlightSession& session, const config::CapsuleReturnMission& mission,
                                  const std::filesystem::path& config_dir, time::Epoch epoch);

gnc::DeorbitParameters capsule_return_parameters(const config::CapsuleReturnMission& mission);

}  // namespace starsim::sim
```

```cpp
#include "starsim/sim/capsule_return_mission.hpp"

#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"

namespace starsim::sim {

void place_capsule_return_mission(FlightSession& session, const config::CapsuleReturnMission& m,
                                  const std::filesystem::path& config_dir, time::Epoch epoch) {
    auto cfg = config::load_vehicle(config_dir, m.vehicle_id);
    for (auto& stage : cfg.stages) {
        for (auto& tank : stage.tanks) {
            if (tank.contents == config::TankContents::Rcs) tank.mass_kg = m.rcs_propellant_kg;
        }
    }
    const double re = constants::wgs84_semi_major_axis_m;
    const double rp = re + m.periapsis_height_m, ra = re + m.apoapsis_height_m;
    const orbit::KeplerianElements elements{0.5 * (rp + ra), (ra - rp) / (ra + rp), m.inclination_rad, m.raan_rad, 0.0,
                                            m.argument_of_latitude_rad};
    session.place_in_orbit(cfg, orbit::state_from_elements(elements, session.gm_earth_m3ps2()), epoch);
    session.set_splashdown_zone(config::load_splashdown_zone(config_dir, m.zone_id));
    session.set_nose_cone(0, true);
}

gnc::DeorbitParameters capsule_return_parameters(const config::CapsuleReturnMission& m) {
    auto p = FlightSession::capsule_deorbit_parameters();
    p.entry_interface_height_m = m.entry_interface_height_m;
    p.entry_flight_path_rad = m.entry_flight_path_rad;
    p.nominal_bank_rad = m.nominal_bank_rad;
    p.max_crossrange_m = m.max_crossrange_m;
    p.search_window_s = m.search_window_s;
    return p;
}

}  // namespace starsim::sim
```

Check the exact name of the RCS tank contents enumerator in `vehicle_config.hpp` (`TankContents`). Also check the
order of `KeplerianElements` fields against the ship test's `place_mission`, which used
`{a, e, i, raan, argp = 0, argument of latitude as the anomaly}`.

`FlightSession::place_in_orbit(const config::VehicleConfig& config, const math::StateVector& state, time::Epoch epoch)`:
move the body of the string overload into it (from `auto vehicle = std::make_unique<vehicle::Vehicle>(config, ...)`
on). The string overload becomes: load the config, erase `first_stage` stages, call the new overload.

- [ ] **Step 5: Mission file** (`config/missions/crew_dragon_return.toml`)

```toml
# Return of Crew Dragon from an ISS-like orbit to the Pacific off San Diego (d4). The onboard computer plans the Draco
# deorbit burn (FlightSession::plan_capsule_return) for the first pass over the zone within the capsule's crossrange,
# jettisons the trunk after the burn and closes the nose cone [S: West Coast order since 2025], flies the bank-guided
# entry (d3) to the drogue gate, and comes down under drogues and mains (d4). [orbit] is the scenario start.

[mission]
name = "Crew-Dragon-Rückkehr"
vehicle = "crew_dragon"
zone = "pacific_san_diego"

[orbit]
epoch_utc = "2026-10-14T06:00:00"
periapsis_km = 420.0            # [E] ISS altitude after undocking
apoapsis_km = 420.0
inclination_deg = 51.6          # [S] ISS
raan_deg = 0.0                  # [D] replaced in Step 6 by the first feasible plane; a plane with a pass over the zone within reach in 48 h (test "... finds the plane")
argument_of_latitude_deg = 0.0
rcs_propellant_kg = 2000.0      # [E] after the ISS mission; the return spends ~650 kg (d3: 598 kg deorbit, ~30 kg entry)

[deorbit]
entry_interface_km = 120.0
entry_flight_path_deg = -2.0    # [E] d3: inside the crew's 4 g at L/D 0.2
nominal_bank_deg = 30.0         # [E] d3
max_crossrange_km = 40.0        # [E] d3
search_window_h = 48.0
```

- [ ] **Step 6: Find the plane.** Add a hidden test that prints the first feasible RAAN, as d3's `dragon_raan_rad()`
  does: 10 deg steps, `plan_capsule_return` feasible from the mission's other values. Run it, put the value into
  `raan_deg` with the [D] comment, and keep the test hidden (`[.]`).

```cpp
TEST_CASE("Crew Dragon return: the first feasible orbit plane", "[.][dragon][mission]") {
    auto m = config::load_capsule_return_mission(STARSIM_CONFIG_DIR, "crew_dragon_return");
    const auto t0 = spice::SpiceTimeConverter{}.from_utc(m.epoch_utc);
    for (double r = 0.0; r < 360.0; r += 10.0) {
        m.raan_rad = r * deg;
        sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
        sim::place_capsule_return_mission(session, m, STARSIM_CONFIG_DIR, t0);
        if (session.plan_capsule_return(sim::capsule_return_parameters(m))) {
            std::printf("first feasible RAAN %.0f deg\n", r);
            return;
        }
    }
    FAIL("no plane");
}
```

  Note: the d3 search used `raan` 0-350 deg at the same epoch and 420 km orbit, and found a value
  (`dragon_raan_rad()`). Expect the same plane. Run with `"Crew Dragon return: the first feasible orbit plane"`.

- [ ] **Step 7: Write the full mission test with the timeline evidence**

```cpp
namespace {

struct MissionRun {
    bool planned = false;
    double wait_h = 0.0;
    double ignition_s = -1.0, cutoff_s = -1.0, trunk_s = -1.0, interface_s = -1.0;
    Descent descent;          // times relative to the gate step (fly_down)
    double drogue_after_ignition_s = -1.0;
    double splashdown_after_ignition_s = -1.0;
};

// Places the mission (argument of latitude `phase_deg` if >= 0), plans and flies it to the water.
MissionRun fly_mission(double phase_deg = -1.0) {
    auto m = config::load_capsule_return_mission(STARSIM_CONFIG_DIR, "crew_dragon_return");
    if (phase_deg >= 0.0) m.argument_of_latitude_rad = phase_deg * deg;
    const auto t0 = spice::SpiceTimeConverter{}.from_utc(m.epoch_utc);
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    sim::place_capsule_return_mission(session, m, STARSIM_CONFIG_DIR, t0);
    MissionRun run;
    run.planned = session.plan_capsule_return(sim::capsule_return_parameters(m));
    if (!run.planned) return run;
    run.wait_h = (session.ship_return_plan()->node.epoch - t0) / 3600.0;
    // to the gate in the d3 manner: big steps to the burn, 1 s after
    while (static_cast<int>(session.capsule_return_step()) < static_cast<int>(sim::CapsuleReturnStep::Gate) &&
           session.epoch() - t0 < 3.0 * 86400.0 && !session.body(session.controlled_index()).impacted()) {
        double step = 1.0;
        if (session.autopilot().state == sim::AutopilotState::Burn) {
            const double to_ignition = session.autopilot().burn.time_to_ignition_s;
            if (to_ignition > 900.0) step = to_ignition - 600.0;
            if (run.ignition_s < 0.0 && session.autopilot().burn.ignite) run.ignition_s = session.epoch() - t0;
        } else if (run.cutoff_s < 0.0 && run.ignition_s >= 0.0) {
            run.cutoff_s = session.epoch() - t0;
        }
        session.advance_to(session.epoch() + step, 1000000);
        if (run.trunk_s < 0.0 && session.body_count() == 2) run.trunk_s = session.epoch() - t0;
        if (run.interface_s < 0.0 && run.cutoff_s >= 0.0 &&
            session.telemetry(session.controlled_index()).altitude_m < m.entry_interface_height_m) {
            run.interface_s = session.epoch() - t0;
        }
    }
    const double gate_s = session.epoch() - t0;
    run.descent = fly_down(session);
    run.drogue_after_ignition_s = gate_s + run.descent.drogue_s - run.ignition_s;
    run.splashdown_after_ignition_s = gate_s + run.descent.splashdown_s - run.ignition_s;
    return run;
}

}  // namespace

TEST_CASE("Crew Dragon return mission splashes down in the zone on the published timeline", "[dragon][mission][splashdown]") {
    const auto r = fly_mission();
    REQUIRE(r.planned);
    const auto& d = r.descent;
    std::printf("[validation] Crew Dragon return: waited %.1f h; burn %.0f s, trunk %.0f s after cutoff, interface %.0f s "
                "after cutoff; drogues %.2f km / %.0f m/s, mains %.2f km / %.0f m/s %.0f s later, splashdown %.0f s after "
                "the mains at %.1f m/s, %.2f km from the zone centre; ignition -> splashdown %.1f min (Crew-10: 54); "
                "peak under canopies %.2f g\n",
                r.wait_h, r.cutoff_s - r.ignition_s, r.trunk_s - r.cutoff_s, r.interface_s - r.cutoff_s,
                d.drogue_height_m / 1000.0, d.drogue_speed_mps, d.main_height_m / 1000.0, d.main_speed_mps,
                d.main_s - d.drogue_s, d.splashdown_s - d.main_s, d.sink_mps, d.miss_m / 1000.0,
                r.splashdown_after_ignition_s / 60.0, d.peak_g);
    CHECK(d.outcome == sim::LandingOutcome::SplashedDown);
    CHECK(d.miss_m < 2000.0);                                       // user 2026-10-14: zone radius 2 km
    CHECK(d.drogue_height_m == Catch::Approx(5500.0).epsilon(0.15));  // [S] ~18,000 ft
    CHECK(d.drogue_speed_mps == Catch::Approx(156.0).epsilon(0.15));  // [S] ~350 mph
    CHECK(d.main_height_m == Catch::Approx(1830.0).epsilon(0.15));    // [S] ~6,000 ft
    CHECK(d.main_speed_mps == Catch::Approx(53.0).epsilon(0.15));     // [S] ~119 mph
    CHECK(d.sink_mps == Catch::Approx(7.0).epsilon(0.30));            // [S] 15-16 mph
    CHECK(r.splashdown_after_ignition_s / 60.0 == Catch::Approx(54.0).epsilon(0.15));  // [S] Crew-10 10:39 -> 11:33 EDT
    CHECK(d.peak_g < 3.6);
}
```

  The 54 min check is the timeline evidence. If it fails, the cause is likely the known gap (our Draco burn is 1058 s
  against 7.5-12 min published). Do not tune the burn to make it pass: ledger it as a finding, loosen nothing, and
  report it to the user.

- [ ] **Step 8: Campaign** (orbit phase as for b4: 8 phases in two halves)

```cpp
namespace {

void fly_phase_campaign(int first, int count) {
    for (int i = first; i < first + count; ++i) {
        const double phase = 45.0 * i;
        const auto r = fly_mission(phase);
        std::printf("dragon return phase %.0f: planned %d, waited %.1f h, %s, %.2f km from the centre, %.1f m/s, %.1f min "
                    "ignition -> splashdown\n",
                    phase, r.planned, r.wait_h, sim::to_string(r.descent.outcome), r.descent.miss_m / 1000.0,
                    r.descent.sink_mps, r.splashdown_after_ignition_s / 60.0);
        INFO("phase " << phase);
        CHECK(r.planned);
        CHECK(r.descent.outcome == sim::LandingOutcome::SplashedDown);
        CHECK(r.descent.miss_m < 2000.0);
    }
}

}  // namespace

TEST_CASE("Crew Dragon return campaign: orbit phase 0-135 deg", "[dragon][mission][validation]") { fly_phase_campaign(0, 4); }
TEST_CASE("Crew Dragon return campaign: orbit phase 180-315 deg", "[dragon][mission][validation]") { fly_phase_campaign(4, 4); }
```

  If a phase finds no pass within 48 h, raise `search_window_h` [E] and ledger the ruling. Never drop the case.

- [ ] **Step 9: Run** `starsim_core_tests.exe "[dragon][mission]"`, then the full suite. Expected: all green. Record the
  `[validation]` lines in the ledger for the docs.

- [ ] **Step 10: Docs.** Add a `CLAUDE.md` Known-issues bullet, "Crew Dragon parachutes and splashdown (d4 ...)". Cover
  the model, the config figures with their tags, the sequencer rules, no RCS under the canopies, splashdown judging,
  the mission file, the validation figures from the ledger, and the known gaps (massless canopy: no opening-shock
  factor, no pendulum; no wind; the timeline gap if any). Update the `aero` list in the architecture paragraph
  ("parachutes"). `config/README.md`: document `[[stage.parachute]]`, `max_splashdown_speed_mps`, and the
  capsule-return mission format.

- [ ] **Step 11: Commit**

```bash
git add core/include/starsim/config/mission_config.hpp core/src/config/mission_config.cpp core/include/starsim/sim/capsule_return_mission.hpp core/src/sim/capsule_return_mission.cpp core/CMakeLists.txt core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp config/missions/crew_dragon_return.toml core/tests/test_capsule_splashdown.cpp CLAUDE.md config/README.md
git commit -m "feat(sim): Crew Dragon return mission from orbit to splashdown; timeline evidence and orbit-phase campaign"
```

---

### Task 7: d3 deferred minors M2, M3, M5

**Files:**
- Modify: `core/src/sim/flight_session.cpp` (`plan_capsule_return`), `core/src/config/vehicle_config.cpp`, `core/src/vehicle/vehicle.cpp` (`separate`)
- Test: `core/tests/test_capsule_entry.cpp`, `core/tests/test_parachute.cpp`

**Interfaces:**
- Consumes: d3 API. Produces: nothing new.

M4, the tumble test stopping at 1 kPa, stays deferred. It is a test-length choice, not a defect, and flying it to the
gate costs runtime.

- [ ] **Step 1: Write the failing tests**

```cpp
// test_capsule_entry.cpp
TEST_CASE("A capsule still on the second stage refuses the capsule return", "[dragon][mission]") {
    DragonDeorbit d(dragon_raan_rad());
    auto stack = config::stack_payload(config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5"),
                                       config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon"));
    stack.stages.erase(stack.stages.begin());  // the second stage with Dragon on top, in orbit
    const auto& s = d.session.body(0).state();
    d.session.place_body(stack, s, d.session.epoch());
    d.session.set_splashdown_zone(d.zone);
    CHECK_FALSE(d.session.plan_capsule_return());
    REQUIRE(d.session.ship_return_plan() != nullptr);
    CHECK(d.session.ship_return_plan()->reason == "Keine Rückkehr möglich: Kapsel noch auf der Oberstufe");
}

TEST_CASE("Splitting a body keeps its truth aero dispersion on both parts", "[vehicle][dragon]") {
    vehicle::Vehicle stack(config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon"), STARSIM_CONFIG_DIR);
    stack.set_hull_aero_scale(1.1);
    auto capsule = stack.separate(1);
    CHECK(capsule->hull_aero_scale() == 1.1);
    CHECK(stack.hull_aero_scale() == 1.1);
}
```

```cpp
// test_parachute.cpp (uses patched_config from Task 1)
TEST_CASE("A capsule entry profile needs a capsule table", "[dragon][config]") {
    CHECK_THROWS_AS(config::load_vehicle(patched_config("capsule_table = \"aero/crew_dragon_aero.csv\"", ""), "crew_dragon"),
                    config::ConfigError);
}
```

- [ ] **Step 2: Run them and confirm they fail.** Expected: the return is planned (or refused for another reason), the
  scale is 1.0, and nothing is thrown.

- [ ] **Step 3: Fixes**
  - M2, in `plan_capsule_return` after the "keine Kapsel im Orbit" check:

```cpp
    if (stages.back().config.payload) {  // review d3 M2: the capsule must fly alone (released from the second stage)
        refused.reason = "Keine Rückkehr möglich: Kapsel noch auf der Oberstufe";
        deorbit_plan_ = refused;
        return false;
    }
```

    (`stack_payload` marks the payload's stages `payload`. A capsule placed alone has `payload == false`.)
  - M3, in the stage loader after `[stage.aero]` is read:
    `if (s.aero.entry_profile == "capsule" && s.aero.capsule_table.empty()) doc.fail("stage.aero: entry_profile = \"capsule\" needs capsule_table");`
  - M5, in `Vehicle::separate`, next to the other `upper->` settings: `upper->hull_aero_scale_ = hull_aero_scale_;`.

- [ ] **Step 4: Run** `"[dragon]"`, `"[parachute]"`, then the full suite. Expected: green.

- [ ] **Step 5: Commit**

```bash
git add core/src/sim/flight_session.cpp core/src/config/vehicle_config.cpp core/src/vehicle/vehicle.cpp core/tests/test_capsule_entry.cpp core/tests/test_parachute.cpp
git commit -m "fix(dragon): refuse a return while the capsule is stacked, validate capsule entry profiles, keep the aero dispersion on separation"
```

---

## Self-review notes

- Spec coverage:
  - Model: Tasks 1-2.
  - Opening load and "Fallschirm gerissen": Task 2. The UI string lives with the outcome; the vehicle exposes
    `parachute_torn()`. Task 6 docs name the string, and d5 shows it.
  - Crew Dragon figures [S]: Task 1 config, checked in Tasks 2, 5 and 6.
  - Sequencer (altitude + q): Task 3.
  - No wind: documented in Task 6 docs.
  - Splashdown <= 9 m/s, "gewassert" / "Aufprall", resting without buoyancy: Task 4.
  - "Mains open": the speed implies it (drogues alone hit at ~50 m/s, no canopy at ~free fall); test "faster than its
    limit". Ruling to ledger at Task 4: no separate mains flag, because the sink limit cannot be met without mains.
  - Full mission + phase campaign: Task 6.
  - Evidence: Task 6, with the webcast-telemetry ruling.
- d5 (bindings) is out of scope. The accessors (`parachute_sequencer`, `Parachute` state, `splashdown_miss_m`) are what
  d5 will read.
- "Fallschirm gerissen" needs a German string accessor for d5. Add
  `const char* vehicle::parachute_failure_text() { return "Fallschirm gerissen"; }` in d5, not here (YAGNI).
