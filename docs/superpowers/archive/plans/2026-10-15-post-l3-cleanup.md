# Post-l3 Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Pay down the debt collected during l1-l3 before l4 (descent, surface stay, ascent) builds on it: one shared
3x3 solver, the two largest source files split by responsibility, and the deferred l2 / l3 review minors fixed.

**Architecture:** Tasks 1-3 change no behaviour (one solver in `math`, pure moves of whole functions into new
translation units). Tasks 4-5 are small behavioural fixes, each pinned by a test. No new module; one new small `gnc`
unit (`SunHold`).

**Tech Stack:** C++20, CMake/Ninja (MSVC 2022), Catch2, `tools/build.ps1`.

**Spec:** none (cleanup). Sources: the l3 plan's ledger (`docs/superpowers/plans/2026-10-15-lunar-l3-outbound.md`,
Task 8 entry: "Deferred minors") and the deferred l2 minors recorded in the memory `project-lunar-mission`.

## Global Constraints

- SI units, radians; frame and unit in names (`position_gcrf_m`); no magic numbers (constants with `[E]`/`[S]` tags).
- Code, comments, commits in English; UI / status / error strings in the core are German.
- Work on `main`; Conventional Commits; push after every task once `-Fast` is green.
- During a task run only the touched tags (`-TestFilter`); `-Fast` before every push; the full suite in Task 5 (the
  plan touches guidance and flight sessions).
- Tests must not share temp files or directories (unique names).
- Moves in Tasks 2-3 are verbatim: no edits to moved code except includes and the `detail` namespace qualifier.

## Out of scope (stay deferred, recorded in the ledger)

- l4: canted landing thrusters in the attitude / descent / burn models (l4 builds the descent models).
- l6: `MissionStart::refusal` and `lunar_status()` in the bindings; telemetry altitude at the Moon above the terrain
  instead of the sphere; synchronous planning in the tick (NRHO ~40 s) - needs the planner thread of l6 / M6.

## Review Focus

1. A Newton step on a (nearly) singular Jacobian: the shared solver must refuse it exactly as the old local copies did
   (targeter breaks, CR3BP corrector returns nullopt) - Task 1 tests a dependent and a badly scaled matrix.
2. A forced burn group list that names a landing-only group: no landing thruster may light - Task 4 test.
3. An upper stage with fixed legs separated from a stack: its legs are out - Task 4 test.
4. A small burn (trim, station keeping) planned closer than the 120 s preparation: it must be planned after it -
   Task 5 tests.
5. A Sun hold that never settles must end (cold gas) - Task 5 `SunHold` test.

---

## File Structure

| File | Responsibility |
|---|---|
| `core/include/starsim/math/mat3.hpp` | + `Mat3::from_columns`, `column`, `determinant`, `solve` (Task 1) |
| `core/src/mission/position_targeter.cpp`, `core/src/orbit/cr3bp.cpp`, `core/src/gnc/attitude_controller.cpp`, `core/src/gnc/j2_insertion.cpp` | use `math::solve` (Task 1) |
| `core/src/sim/flight_session.cpp` | session core: construction, placement, autopilot loop, tick, pilot actions (Task 2) |
| `core/src/sim/flight_session_capsule.cpp` (new) | capsule entry, deorbit, parachutes, splashdown (Task 2) |
| `core/src/sim/flight_session_return.cpp` (new) | ship entry / return / landing, booster return and catch (Task 2) |
| `core/src/sim/flight_session_burns.cpp` (new) | burn configurations, burn execution, circularisation (Task 2) |
| `core/src/sim/flight_session_command.cpp` (new) | `command_for`: the per-tick vehicle command (Task 2) |
| `core/src/gnc/booster_return_detail.hpp` (new, private) | constants and helpers shared by the booster-return units (Task 3) |
| `core/src/gnc/booster_return.cpp` | parameters, construction, `update` state machine (Task 3) |
| `core/src/gnc/booster_return_prediction.cpp` (new) | impact / landing-burn / MECO predictions, drag estimate (Task 3) |
| `core/src/gnc/booster_return_burns.cpp` (new) | entry burn and the explicit (ZEM/ZEV) landing burn (Task 3) |
| `core/src/sim/flight_session_burns.cpp`, `core/src/vehicle/vehicle.cpp`, `core/include/starsim/vehicle/flight_body.hpp`, `core/src/sim/lunar_site.cpp`, `core/src/sim/flight_session.cpp` | l2 minors (Task 4) |
| `core/include/starsim/gnc/sun_hold.hpp`, `core/src/gnc/sun_hold.cpp` (new) | Sun hold with settle check and timeout (Task 5) |
| `core/src/mission/lunar_outbound.cpp`, `core/src/mission/lunar_orbit_transfer.cpp`, `core/src/sim/flight_session_lunar.cpp`, `core/include/starsim/sim/flight_session.hpp` | l3 minors (Task 5) |
| `docs/subsystems.md` | lunar section: velocity-only matches, leads, Sun hold, file map (Task 5) |

---

### Task 1: One 3x3 solver in `math`

Four Cramer's-rule copies exist: `solve_columns` in `position_targeter.cpp:19` and `cr3bp.cpp:19` (identical), an
inline one in `attitude_controller.cpp:72-91` and `det3` in `j2_insertion.cpp:94-114`. Also `booster_return.cpp:18`
defines its own `gm_earth = 3.986004418e14` although `constants::earth_gm_m3ps2` exists.

**Files:**
- Modify: `core/include/starsim/math/mat3.hpp`
- Modify: `core/src/mission/position_targeter.cpp:18-25,65-66`, `core/src/orbit/cr3bp.cpp:18-25,82-83`,
  `core/src/gnc/attitude_controller.cpp:71-91`, `core/src/gnc/j2_insertion.cpp:93-114`, `core/src/gnc/booster_return.cpp:18,435,441,491`
- Test: `core/tests/test_math.cpp`

**Interfaces:**
- Produces: `math::Mat3::from_columns(const Vec3&, const Vec3&, const Vec3&)`, `math::column(const Mat3&, int)`,
  `math::determinant(const Mat3&)`, `std::optional<Vec3> math::solve(const Mat3& a, const Vec3& y)` (nullopt when the
  determinant is not finite or `|det| <= 1e-14 * |c0| |c1| |c2|`, the old copies' test).

- [ ] **Step 1: Write the failing test** (append to `core/tests/test_math.cpp`; add `#include <catch2/catch_approx.hpp>`
  if missing)

```cpp
TEST_CASE("Mat3 determinant and linear solve", "[math]") {
    using math::Mat3;
    using math::Vec3;
    // Rows [2 1 0; 0 3 1; 1 0 4]: det 25.
    const Mat3 a = Mat3::from_columns({2.0, 0.0, 1.0}, {1.0, 3.0, 0.0}, {0.0, 1.0, 4.0});
    CHECK(a(0, 1) == 1.0);
    CHECK(math::column(a, 2).z == 4.0);
    CHECK(math::determinant(a) == Catch::Approx(25.0));
    const auto x = math::solve(a, {0.0, -3.0, 13.0});
    REQUIRE(x);
    CHECK(norm(*x - Vec3{1.0, -2.0, 3.0}) < 1e-12);
    // Dependent columns: refused.
    CHECK_FALSE(math::solve(Mat3::from_columns({1.0, 2.0, 3.0}, {0.0, 1.0, 1.0}, {1.0, 3.0, 4.0}), {1.0, 0.0, 0.0}));
    // Badly scaled but regular columns (a Jacobian in m per m/s next to one in m per m): solved.
    const auto y = math::solve(Mat3::from_columns({1e6, 0.0, 0.0}, {0.0, 1e-3, 0.0}, {0.0, 0.0, 1.0}), {1e6, 1e-3, 1.0});
    REQUIRE(y);
    CHECK(norm(*y - Vec3{1.0, 1.0, 1.0}) < 1e-12);
}
```

- [ ] **Step 2: Run it to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[math]"`
Expected: compile error `'from_columns': is not a member of 'starsim::math::Mat3'`.

- [ ] **Step 3: Implement in `mat3.hpp`** (add `#include <cmath>` and `#include <optional>`; inside `struct Mat3`
  after `identity()`:)

```cpp
    static constexpr Mat3 from_columns(const Vec3& c0, const Vec3& c1, const Vec3& c2) {
        Mat3 r;
        r.m[0] = {c0.x, c1.x, c2.x};
        r.m[1] = {c0.y, c1.y, c2.y};
        r.m[2] = {c0.z, c1.z, c2.z};
        return r;
    }
```

and after `transpose`:

```cpp
constexpr Vec3 column(const Mat3& a, int k) { return {a.m[0][k], a.m[1][k], a.m[2][k]}; }

constexpr double determinant(const Mat3& a) { return dot(column(a, 0), cross(column(a, 1), column(a, 2))); }

// Solves a x = y by Cramer's rule; nullopt when the columns are (nearly) dependent.
inline std::optional<Vec3> solve(const Mat3& a, const Vec3& y) {
    const Vec3 c0 = column(a, 0), c1 = column(a, 1), c2 = column(a, 2);
    const double det = dot(c0, cross(c1, c2));
    if (!std::isfinite(det) || std::abs(det) <= 1e-14 * norm(c0) * norm(c1) * norm(c2)) return std::nullopt;
    return Vec3{dot(y, cross(c1, c2)), dot(c0, cross(y, c2)), dot(c0, cross(c1, y))} / det;
}
```

(If `dot` / `cross` in `vec3.hpp` are not `constexpr`, make `column` constexpr and `determinant` `inline`.)

- [ ] **Step 4: Run `[math]`** - expected PASS.

- [ ] **Step 5: Replace the copies**

`position_targeter.cpp`: delete `solve_columns` (lines 18-25) and replace lines 65-66 with

```cpp
        auto step = math::solve(math::Mat3::from_columns(column[0], column[1], column[2]), -1.0 * miss);
        if (!step) break;
```

then use `*step` where `step` was used below (or bind `Vec3 step = *solved;` after the check - keep the following
lines unchanged). Same in `cr3bp.cpp` (delete lines 18-25; at line 82-83:
`const auto step = math::solve(math::Mat3::from_columns(column[0], column[1], column[2]), -g); if (!step) return std::nullopt; u += *step;`).
Add `#include "starsim/math/mat3.hpp"` where missing.

`attitude_controller.cpp` (lines 71-91): build `math::Mat3 m` instead of `double m[3][3]` (same accumulation and
`1e-6 * trace` regularisation), then

```cpp
    const auto solved = math::solve(m, torque);
    if (!solved) return std::vector<double>(n, 0.0);
    const double y[3] = {solved->x, solved->y, solved->z};
```

and keep the rest of the function unchanged.

`j2_insertion.cpp` (lines 93-114): accumulate into `math::Mat3 a` and `Vec3 b`, delete `det3` and the column loop:

```cpp
        const auto step = math::solve(a, b);
        if (!step) break;
        p[0] += step->x;
        p[1] += step->y;
        p[2] += step->z;
```

`booster_return.cpp`: delete line 18, use `constants::earth_gm_m3ps2` at lines 435, 441, 491.

- [ ] **Step 6: Run the touched tags**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[math],[cr3bp],[nrho],[outbound],[transfer],[attitude],[j2],[mission]"`
Expected: PASS (the solver is the same formula; a `[j2]` or `[attitude]` value moving in the last digits is
fine, a failed case is not - then compare the old and new threshold for that call).

- [ ] **Step 7: `-Fast`, commit, push**

```bash
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast
git add core/include/starsim/math/mat3.hpp core/src/mission/position_targeter.cpp core/src/orbit/cr3bp.cpp core/src/gnc/attitude_controller.cpp core/src/gnc/j2_insertion.cpp core/src/gnc/booster_return.cpp core/tests/test_math.cpp
git commit -m "refactor(math): one 3x3 solver instead of four Cramer copies"
git push origin main
```

---

### Task 2: Split `flight_session.cpp` (1354 lines) by responsibility

Pure move of whole function definitions; `flight_session.hpp` is unchanged. Function -> new file (line numbers as of
4264952):

- `flight_session_capsule.cpp`: `capsule_entry_target` (100), `place_entering_capsule` (108),
  `to_string(CapsuleReturnStep)` (126), `capsule_deorbit_parameters` (143), `plan_capsule_return` (154),
  `run_capsule_return` (213), `run_parachutes` (271), `splashdown_miss_m` (287).
- `flight_session_return.cpp`: `set_booster_return` (76), `return_status` (84), `entry_status` (89),
  `place_entering_ship` (94), `start_entry_guidance` (114), `plan_ship_return` (298), `entry_start_epoch` (337),
  `place_landing_ship` (351), `run_ship_landing` (360), `run_return_guidance` (374), `returns_after_staging` (741),
  `return_site` (767), `place_returning_booster` (797), `return_parameters` (809), `set_landing_zone` (815),
  `apply_return_command` (905). The anonymous-namespace constant `tower_keep_out_margin_m` (only used by
  `return_site`) moves with it.
- `flight_session_burns.cpp`: `burn_configurations` (425), `minimum_burn_dv_mps` (491), both `execute_burn` (496,
  498), `start_burn` (502), `plan_circularization_at_apoapsis` (544).
- `flight_session_command.cpp`: `command_for` (944-1090).
- Stays in `flight_session.cpp`: constructor, `to_string(AutopilotState)`, ascent autopilot, placement, telemetry,
  `run_autopilot`, `coast_allowance_s` (keeps `ground_tick_height_m`), `advance_to`, pilot actions, legs / terrain
  setters.

**Files:**
- Create: `core/src/sim/flight_session_capsule.cpp`, `core/src/sim/flight_session_return.cpp`,
  `core/src/sim/flight_session_burns.cpp`, `core/src/sim/flight_session_command.cpp`
- Modify: `core/src/sim/flight_session.cpp`, `core/CMakeLists.txt:57-58`

**Interfaces:** none change (member functions of `sim::FlightSession`).

- [ ] **Step 1: Create each new file** with a one-line header comment naming its part, `#include
  "starsim/sim/flight_session.hpp"`, the original include block of `flight_session.cpp` (lines 1-16), `namespace
  starsim::sim {`, the moved definitions in their original order and with their preceding comments, `}  // namespace
  starsim::sim`. Example head of `flight_session_burns.cpp`:

```cpp
// FlightSession: burn configurations of the controlled vehicle and the execution of planned burns.

#include "starsim/sim/flight_session.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::sim {
```

- [ ] **Step 2: Delete the moved definitions from `flight_session.cpp`** and add the four files after
  `src/sim/flight_session.cpp` in `core/CMakeLists.txt`.

- [ ] **Step 3: Build and trim includes** - build; then remove from each file the includes it does not need (remove
  one, build, keep it removed when the build stays green).

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[sim]"`
Expected: build green, `[sim]` cases PASS.

- [ ] **Step 4: Check sizes** - `wc -l core/src/sim/flight_session*.cpp`: expected `flight_session.cpp` ~650 lines,
  each new file < 450. `git diff --stat` must show (almost) only moved lines: `git diff -M --color-moved=zebra` shows
  no changed lines inside moved blocks.

- [ ] **Step 5: `-Fast`, commit, push**

```bash
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast
git add core/src/sim/flight_session*.cpp core/CMakeLists.txt
git commit -m "refactor(sim): split flight_session.cpp into capsule, return, burn and command parts"
git push origin main
```

---

### Task 3: Split `booster_return.cpp` (930 lines)

Pure move. Shared constants and helpers go into a private header in `core/src/gnc/` (not in `include/`: no other
module may use them).

**Files:**
- Create: `core/src/gnc/booster_return_detail.hpp`, `core/src/gnc/booster_return_prediction.cpp`,
  `core/src/gnc/booster_return_burns.cpp`
- Modify: `core/src/gnc/booster_return.cpp`, `core/CMakeLists.txt:32`

Placement (line numbers as of 4264952, after Task 1 removed line 18):

- `booster_return_detail.hpp`, `namespace starsim::gnc::booster_return_detail`: all constants of lines 19-45 as
  `inline constexpr double`, `up_at`, `rotation_z`, `EngineSet` + `engine_set` (lines 47-81) as `inline` functions.
  Guard with `#pragma once`; include `starsim/math/constants.hpp`, `starsim/math/mat3.hpp`,
  `starsim/vehicle/vehicle.hpp`, `<cmath>`, `<string>`, `<vector>`.
- `booster_return.cpp`: `grid_fin_station`, `lateral_acceleration` (only used by `update`, stay in its anonymous
  namespace), `to_string(ReturnPhase)`, both `booster_return_parameters`, constructor, `retarget_to_sea`, `update`,
  `set_realtime`, `start_landing`, `request_abort`.
- `booster_return_prediction.cpp`: `predict_impact`, `predict_landing_burn`, `landing_burn_lead_itrf`,
  `meco_propellant_kg`, `predict`, `estimate_drag_scale`.
- `booster_return_burns.cpp`: `entry_burn`, `entry_burn_ahead`, `pending_entry_burn`, `explicit_landing`.

Each `.cpp` adds `using namespace booster_return_detail;` inside `namespace starsim::gnc {` (the moved code then
compiles unchanged).

- [ ] **Step 1: Create the detail header and the two new units, delete the moved code, add both `.cpp` files after
  `src/gnc/booster_return.cpp` in `core/CMakeLists.txt`.**

Head of `booster_return_detail.hpp`:

```cpp
#pragma once

// Constants and helpers shared by the booster-return guidance units (booster_return*.cpp). Private to core/src/gnc.

#include <cmath>
#include <string>
#include <vector>

#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::gnc::booster_return_detail {

inline constexpr double touchdown_speed_mps = 1.0;
// ... (the remaining constants of booster_return.cpp lines 20-45, verbatim with their comments)

}  // namespace starsim::gnc::booster_return_detail
```

- [ ] **Step 2: Build, trim includes, run the return tags**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[gnc],[catch],[booster]"`
Expected: PASS. Then `wc -l core/src/gnc/booster_return*.cpp`: each < 450 lines; `git diff -M --color-moved=zebra`
shows no edits inside moved blocks.

- [ ] **Step 3: `-Fast`, commit, push**

```bash
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast
git add core/src/gnc/booster_return* core/CMakeLists.txt
git commit -m "refactor(gnc): split booster_return.cpp into state machine, predictions and burns"
git push origin main
```

---

### Task 4: Deferred l2 minors

Five small fixes, each with its test:

1. `burn_configurations(&forced)` (`flight_session_burns.cpp`, the `if (forced)` branch) puts every forced group into
   `c.groups`, also a landing-only one; `make()` skips auxiliary engines for the thrust sum, but the burn executor
   lights `c.groups`. Fix: drop forced groups whose engines are auxiliary (`landing_only` / launch escape); when none is
   left there is no configuration.
2. `Vehicle::separate` builds the upper part with `leg_deployment_ = 0`: fixed legs of the upper stage read stowed.
   Fix: at the end of `separate`, on both parts, `if (has_landing_legs() && stages_.front().config.landing_legs.fixed)
   set_leg_deployment(1.0);`.
3. `FlightBody::legs_on_site_` defaults to `true`: a body with legs out that no leg site has claimed yet skips its own
   ground contact. Fix: default `false` (the leg site sets it every tick for bodies on it).
4. `load_lunar_terrain` does not check a tile against `lunar_sites.toml`: a site moved or resized after the download
   uses the old tile silently. Fix: the tile's product must equal the site's, the site must lie within one pixel of
   the tile centre and `(lines - 1) / 2 * scale_m` must equal `tile_half_size_m` within one pixel; otherwise a German
   error naming `tools/data_pipeline/fetch.py`.
5. `set_moon_terrain` after bodies came to rest on the Moon leaves them at their sphere / old-terrain height. Fix:
   refuse (throw, German) while a body rests on the Moon; the order is terrain first, then placement.

**Files:**
- Modify: `core/src/sim/flight_session_burns.cpp`, `core/src/vehicle/vehicle.cpp` (`separate`),
  `core/include/starsim/vehicle/flight_body.hpp:200`, `core/src/sim/lunar_site.cpp`, `core/src/sim/flight_session.cpp`
  (`set_moon_terrain`)
- Test: `core/tests/test_engine_placement.cpp`, `core/tests/test_hls_vehicle.cpp`, `core/tests/test_lunar_flight_body.cpp`,
  `core/tests/test_lunar_terrain.cpp`, `core/tests/test_lunar_session.cpp`

**Interfaces:** unchanged signatures. `FlightSession::set_moon_terrain` may now throw `std::runtime_error`.

- [ ] **Step 1: Write the failing tests**

`test_engine_placement.cpp` (next to "Landing-only engines are never chosen for burns or lit by ignite", which defines
`canted_ship(bool landing_only)`; reuse its session setup - read that test first and place the ship the same way):

```cpp
TEST_CASE("A forced burn naming a landing-only group lights no landing engine", "[sim][engines]") {
    // ... session with canted_ship(true) placed in orbit, exactly as in the landing-only test above ...
    const auto node = gnc::BurnNode{session.epoch() + 600.0, math::Vec3{10.0, 0.0, 0.0}};
    CHECK_FALSE(session.execute_burn(node, {"canted"}));  // only a landing-only group: no configuration
    REQUIRE(session.execute_burn(node, {"vacuum", "canted"}));
    session.advance_to(node.epoch + 5.0, 100000);
    for (const auto& e : session.body(0).vehicle().stages().front().engines) {
        if (e.group_id == "ship/canted") CHECK_FALSE(e.firing);
    }
}
```

(Use the engine-state member the landing-only test already checks instead of `firing` if it is named differently.)

`test_hls_vehicle.cpp`:

```cpp
TEST_CASE("An upper stage with fixed legs keeps them out after separation", "[vehicle][hls][legs]") {
    auto stack = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto hls = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    stack.stages[1] = hls.stages[0];  // Super Heavy + HLS
    vehicle::Vehicle v(stack, STARSIM_CONFIG_DIR);
    CHECK(v.leg_deployment() == 0.0);  // the booster's front stage has no legs
    const auto upper = v.separate(1);
    REQUIRE(upper->has_landing_legs());
    CHECK(upper->leg_deployment() == 1.0);
}
```

`test_lunar_flight_body.cpp` (uses the file's `LunarFixture`, `above_moon`, `lunar_orbit`):

```cpp
TEST_CASE("A body with fixed legs and no leg site keeps its own ground contact on the Moon", "[vehicle][lunar][legs]") {
    LunarFixture fx;
    auto hls = [&](const RigidBodyState& s) {
        return std::make_unique<FlightBody>(
            std::make_unique<vehicle::Vehicle>(config::load_vehicle(config_dir, "starship_hls"), config_dir), s);
    };
    const double cg = hls(lunar_orbit(fx.moon, fx.t0))->vehicle().mass_properties().cg_station_m;
    auto body = hls(above_moon(fx.moon, fx.t0, -85.0 * deg, 30.0 * deg, cg + 0.5, 1.0));
    REQUIRE(body->vehicle().leg_deployment() == 1.0);
    time::Epoch t = fx.t0;
    for (int i = 0; i < 2000 && !body->resting() && !body->impacted(); ++i, t = t + 0.005) {
        body->step(fx.env, t, 0.005, {});
    }
    CHECK((body->resting() || body->impacted()));  // not fallen through the ground
}
```

`test_lunar_terrain.cpp` (tags `[terrain][data]`, real data like the LOLA test; unique temp dir):

```cpp
TEST_CASE("A tile that no longer matches lunar_sites.toml is refused", "[terrain][data]") {
    const auto dir = std::filesystem::temp_directory_path() / "starsim_stale_tile_config";
    std::filesystem::create_directories(dir);
    std::ifstream in(std::filesystem::path(STARSIM_CONFIG_DIR) / "lunar_sites.toml");
    std::stringstream text;
    text << in.rdbuf();
    std::string s = text.str();
    const auto at = s.find("latitude_deg = -84.7906");
    REQUIRE(at != std::string::npos);
    s.replace(at, 23, "latitude_deg = -84.6906");  // moved 3 km: the downloaded tile is stale
    std::ofstream(dir / "lunar_sites.toml") << s;
    try {
        sim::load_lunar_terrain(dir, STARSIM_DATA_DIR);
        FAIL("expected an error");
    } catch (const std::runtime_error& e) {
        CHECK(std::string(e.what()).find("tools/data_pipeline/fetch.py") != std::string::npos);
        CHECK(std::string(e.what()).find("mons_mouton_im2") != std::string::npos);
    }
    std::filesystem::remove_all(dir);
}
```

(`#include <fstream>`, `<sstream>`.)

`test_lunar_session.cpp` (uses its `Fixture`; a flat synthetic terrain like `flat_global` in `test_lunar_terrain.cpp`,
copied into this file's anonymous namespace):

```cpp
TEST_CASE("Lunar terrain is refused once a body rests on the Moon", "[sim][lunar]") {
    Fixture fx;
    const auto frame = fx.moon.frame(fx.t0);
    const auto cfg = ship_config();
    const double cg = vehicle::Vehicle(cfg, STARSIM_CONFIG_DIR).mass_properties().cg_station_m;
    const Vec3 fixed = frames::geodetic_to_fixed({20.0 * deg, 40.0 * deg, cg + 0.5}, fx.moon.shape());
    // Upright is not needed for the refusal: a gentle drop from 0.5 m (tilt checks may make it an impact - then use
    // the upright attitude helper of test_lunar_flight_body.cpp).
    vehicle::RigidBodyState s{frame.position_gcrf(fixed), frame.velocity_gcrf(fixed, {}), {}, {}};
    fx.session.place_body(cfg, s, fx.t0);
    fx.session.advance_to(fx.t0 + 30.0, 100000);
    REQUIRE(fx.session.body(0).resting());
    auto terrain = std::make_shared<const environment::LunarTerrain>(
        flat_global(0.0), std::vector<environment::PolarStereographicTile>{}, constants::moon_mean_radius_m);
    CHECK_THROWS_AS(fx.session.set_moon_terrain(terrain), std::runtime_error);
}
```

(The attitude must make the drop a rest: copy `above_moon` from `test_lunar_flight_body.cpp` if the identity attitude
tips it over.)

- [ ] **Step 2: Run them to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[engines],[hls],[legs],[terrain],[lunar]"`
Expected: the five new cases FAIL (landing group lit / deployment 0 / fell through / no error / no throw).

- [ ] **Step 3: Implement**

(1) In the `if (forced)` branch of `burn_configurations`:

```cpp
        if (forced) {
            std::vector<std::string> selected;
            for (const auto& g : *forced) {
                const std::string id = stage.config.id + "/" + g;
                // Only burn engines: a landing-only or launch-escape group is never lit by a burn (lunar l2).
                if (std::find(groups.begin(), groups.end(), id) != groups.end()) selected.push_back(id);
            }
            if (!selected.empty()) {
                configurations.push_back(make(selected, false));
                configurations.push_back(make(selected, true));
            }
            break;
        }
```

(2) End of `Vehicle::separate`, before `return upper;`:

```cpp
    // Fixed legs are always out, also on a part split off (the upper part starts with them stowed otherwise).
    for (Vehicle* part : {this, upper.get()}) {
        if (part->has_landing_legs() && part->stages_.front().config.landing_legs.fixed) part->set_leg_deployment(1.0);
    }
```

(3) `flight_body.hpp:200`: `bool legs_on_site_ = false;  // set every tick by the leg site for bodies on it`.

(4) `lunar_site.cpp`, in the site loop after the projection check:

```cpp
        // A site moved or resized after the download would use the old tile silently.
        const double half_px = 0.5 * (f.lines - 1);
        const auto [line, sample] = t.position(site.latitude_rad, site.longitude_rad);  // after t is filled
        if (f.product != site.product || std::abs(line - half_px) > 1.0 || std::abs(sample - half_px) > 1.0 ||
            std::abs(half_px * f.scale_m - site.tile_half_size_m) > f.scale_m) {
            throw config::ConfigError("Mondgelände: Kachel " + site.id +
                                      " passt nicht mehr zu lunar_sites.toml - python tools/data_pipeline/fetch.py erneut ausführen");
        }
```

(Move the check below the field assignments of `t`, before `tiles.push_back`; use the exception type that
`load_terrain_grid_file` throws - `config::ConfigError` - with its header.)

(5) `set_moon_terrain`, first line after the orientation check:

```cpp
    for (size_t i = 0; i < bodies_.size(); ++i) {
        if (bodies_[i].body->resting() && reference_body(i) == ephemeris::Body::Moon) {
            throw std::runtime_error("Mondgelände muss vor dem Aufsetzen gesetzt werden");
        }
    }
```

- [ ] **Step 4: Run the touched tags, then the leg and landing tags**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[engines],[hls],[legs],[terrain],[lunar],[landing],[falcon9]"`
Expected: PASS. If a Falcon 9 or HLS leg landing fails after (3), the leg site claims bodies too late: make
`LegLandingSim` set `on_site` before the body's first step on the site (fix there, not by restoring the default).

- [ ] **Step 5: `-Fast`, commit, push**

```bash
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast
git add core/src core/include core/tests
git commit -m "fix: deferred l2 minors (forced landing groups, fixed legs after separation, leg-site default, stale tiles, terrain order)"
git push origin main
```

---

### Task 5: Deferred l3 minors, docs, full suite, review

1. **Small-burn leads.** `station_keeping` plans the burn 60 s after the decision (`station_keeping_lead_s`) and
   `llo_trim` at the nearest apsis (possibly ~0 s ahead), both inside the session's 120 s `burn_preparation_s`. Fix:
   both take a minimum lead; the session passes `burn_preparation_s + small_burn_margin_s` (60 s [E]).
2. **Departure lead.** `earliest_departure_h = 0` lets the TLI centre come before its ignition lead. Fix: the session
   plans with `earliest_departure_s >= min_departure_lead_s` (1800 s [E]: 120 s preparation + half a ~7 min TLI with
   margin); the config keeps accepting 0 ("as early as possible") and `config/README.md` says so.
3. **Sun hold timeout.** A Sun hold that never settles spends cold gas until the next re-point. New `gnc::SunHold`
   (settle check from `flight_session_lunar.cpp:36-37,376-390` plus a timeout).
4. **Moon-bound start.** `plan_lunar_outbound` from an orbit already about the Moon must refuse.
5. **Debug block.** `run_lunar` reads `getenv` every tick and keeps a function-local `static` shared by all sessions:
   read the flag once in the constructor into `lunar_debug_`, keep the last status in a member.
6. **Docs.** NRHO insertion and the LLO -> NRHO arrival burn match the reference velocity only (the position error is
   left to the station keeping): comment at both `arrival_burn` / `insertion` returns and in `docs/subsystems.md`.

**Files:**
- Create: `core/include/starsim/gnc/sun_hold.hpp`, `core/src/gnc/sun_hold.cpp`, `core/tests/test_sun_hold.cpp`
- Modify: `core/include/starsim/mission/lunar_outbound.hpp:104`, `core/src/mission/lunar_outbound.cpp:308-333`,
  `core/include/starsim/mission/lunar_orbit_transfer.hpp:50`, `core/src/mission/lunar_orbit_transfer.cpp:31,160-170`,
  `core/include/starsim/sim/flight_session.hpp` (constants, members), `core/src/sim/flight_session_lunar.cpp`,
  `core/src/sim/flight_session.cpp` (constructor), `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`,
  `config/README.md`, `docs/subsystems.md`
- Test: `core/tests/test_lunar_outbound.cpp`, `core/tests/test_lunar_orbit_transfer.cpp`,
  `core/tests/test_lunar_outbound_session.cpp`, `core/tests/test_sun_hold.cpp`

**Interfaces:**
- `std::optional<gnc::BurnNode> LunarOutboundPlanner::llo_trim(const StateVector&, time::Epoch, const LunarOutboundParameters&, double min_lead_s = 0.0) const` - the first apsis at least `min_lead_s` ahead.
- `std::optional<gnc::BurnNode> LunarOrbitTransfer::station_keeping(const StateVector&, time::Epoch, const NrhoReference&, double deadband_m = 1000.0, double lead_s = 60.0) const`.
- `gnc::SunHoldLimits`, `gnc::SunHoldResult { Holding, Settled, TimedOut }`, `gnc::SunHold::start(time::Epoch)`,
  `active()`, `SunHoldResult update(double error_rad, double rate_radps, time::Epoch)`.
- `FlightSession` constants: `small_burn_margin_s = 60.0`, `min_departure_lead_s = 1800.0` (both [E]).

- [ ] **Step 1: Write the failing tests**

`test_sun_hold.cpp` (add to `core/tests/CMakeLists.txt` like the other test files):

```cpp
// Sun hold of the lunar coast: ends when pointed and settled, or after the timeout.

#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/sun_hold.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;

TEST_CASE("Sun hold settles, or gives up after its timeout", "[gnc][sun]") {
    constexpr double deg = constants::deg_to_rad;
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    gnc::SunHold hold;
    CHECK_FALSE(hold.active());
    hold.start(t0);
    REQUIRE(hold.active());
    CHECK(hold.update(5.0 * deg, 0.0, t0 + 10.0) == gnc::SunHoldResult::Holding);             // not pointed
    CHECK(hold.update(0.5 * deg, 0.1 * deg, t0 + 20.0) == gnc::SunHoldResult::Holding);       // still turning
    CHECK(hold.update(0.5 * deg, 0.001 * deg, t0 + 30.0) == gnc::SunHoldResult::Settled);
    CHECK_FALSE(hold.active());
    hold.start(t0);
    CHECK(hold.update(5.0 * deg, 0.1 * deg, t0 + 1799.0) == gnc::SunHoldResult::Holding);
    CHECK(hold.update(5.0 * deg, 0.1 * deg, t0 + 1800.0) == gnc::SunHoldResult::TimedOut);  // never settles
    CHECK_FALSE(hold.active());
}
```

`test_lunar_outbound.cpp` (next to the existing trim test at line ~155, same fixture and orbit):

```cpp
TEST_CASE("LLO trim keeps the minimum lead: the next apsis after it", "[mission][lunar][outbound]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    const double gm_moon = fx.ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    const double r_moon = constants::moon_mean_radius_m;
    const double rp = r_moon + 95e3, ra = r_moon + 104e3;
    const auto rel =
        orbit::state_from_elements({0.5 * (rp + ra), (ra - rp) / (ra + rp), 90.0 * deg, 0.3, 0.0, 0.0}, gm_moon);
    const auto moon = fx.ephemeris.state_gcrf(ephemeris::Body::Moon, fx.t0);
    const math::StateVector state{rel.position_m + moon.position_m, rel.velocity_mps + moon.velocity_mps};
    const double half_period = constants::pi * std::sqrt(std::pow(0.5 * (rp + ra), 3) / gm_moon);
    const auto trim = fx.planner.llo_trim(state, fx.t0, p, 180.0);  // at periselene now: the aposelene next
    REQUIRE(trim);
    CHECK(trim->epoch - fx.t0 >= 180.0);
    CHECK(std::abs(trim->epoch - fx.t0 - half_period) < 60.0);
    CHECK(norm(trim->delta_v_gcrf_mps) < 5.0);
}
```

`test_lunar_orbit_transfer.cpp`:

```cpp
TEST_CASE("NRHO station keeping honours the lead", "[mission][lunar][nrho]") {
    test::spice_session();
    Fixture fx;
    auto s = fx.nrho.state_gcrf(fx.first_perilune);
    s.velocity_mps += math::Vec3{0.3, -0.3, 0.2};
    const auto apolune = fx.nrho.apolune_epochs()[0];
    s = fx.coast.coast(s, fx.first_perilune, apolune);
    const auto burn = fx.transfer.station_keeping(s, apolune, fx.nrho, 1000.0, 300.0);
    REQUIRE(burn);
    CHECK(burn->epoch - apolune == Catch::Approx(300.0).margin(1e-6));
}
```

`test_lunar_outbound_session.cpp` (uses its `Fixture`):

```cpp
TEST_CASE("Lunar outbound: Moon-bound start refused, immediate departure keeps its lead", "[sim][lunar][outbound]") {
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    p.earliest_departure_s = 0.0;
    fx.place(p.arrival_epoch);
    REQUIRE(fx.session.plan_lunar_outbound(p));
    CHECK(fx.session.lunar_outbound_plan()->tli.epoch - fx.t0 >= sim::FlightSession::min_departure_lead_s);

    Fixture moon_fx;
    const double gm_moon = moon_fx.ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    const auto rel = orbit::state_from_elements({constants::moon_mean_radius_m + 100e3, 0.0, 1.5, 0.0, 0.0, 0.0}, gm_moon);
    const auto moon = moon_fx.ephemeris.state_gcrf(ephemeris::Body::Moon, moon_fx.t0);
    moon_fx.session.place_in_orbit("starship_hls", {rel.position_m + moon.position_m, rel.velocity_mps + moon.velocity_mps},
                                   moon_fx.t0);
    CHECK_FALSE(moon_fx.session.plan_lunar_outbound(p));
    CHECK(moon_fx.session.lunar_status().find("Mond") != std::string::npos);
    CHECK(moon_fx.session.autopilot().state == sim::AutopilotState::Off);
}
```

- [ ] **Step 2: Run them to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[sun],[outbound],[nrho]"`
Expected: compile errors (`sun_hold.hpp` missing, `llo_trim` / `station_keeping` take no lead, no
`min_departure_lead_s`).

- [ ] **Step 3: Implement `SunHold`**

`sun_hold.hpp`:

```cpp
#pragma once

// Sun pointing during a lunar coast: held until pointed and settled, given up after a timeout (a hold that never
// settles would spend cold gas until the next re-point). The session owns the attitude mode; this only decides.

#include "starsim/math/constants.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::gnc {

struct SunHoldLimits {
    double settled_error_rad = 1.0 * constants::deg_to_rad;     // [E]
    double settled_rate_radps = 0.005 * constants::deg_to_rad;  // [E] ~100 deg drift in 6 h at most
    double timeout_s = 1800.0;                                   // [E] slews take minutes; 30 min is a stuck hold
};

enum class SunHoldResult { Holding, Settled, TimedOut };

class SunHold {
public:
    explicit SunHold(SunHoldLimits limits = {}) : limits_(limits) {}
    void start(time::Epoch epoch);
    bool active() const { return active_; }
    // Called every tick while active; Settled and TimedOut end the hold.
    SunHoldResult update(double error_rad, double rate_radps, time::Epoch epoch);

private:
    SunHoldLimits limits_;
    time::Epoch started_{};
    bool active_ = false;
};

}  // namespace starsim::gnc
```

`sun_hold.cpp`:

```cpp
#include "starsim/gnc/sun_hold.hpp"

namespace starsim::gnc {

void SunHold::start(time::Epoch epoch) {
    started_ = epoch;
    active_ = true;
}

SunHoldResult SunHold::update(double error_rad, double rate_radps, time::Epoch epoch) {
    if (!active_) return SunHoldResult::Settled;
    if (error_rad < limits_.settled_error_rad && rate_radps < limits_.settled_rate_radps) {
        active_ = false;
        return SunHoldResult::Settled;
    }
    if (epoch - started_ >= limits_.timeout_s) {
        active_ = false;
        return SunHoldResult::TimedOut;
    }
    return SunHoldResult::Holding;
}

}  // namespace starsim::gnc
```

Add `src/gnc/sun_hold.cpp` to `core/CMakeLists.txt` (gnc block).

- [ ] **Step 4: Implement the planner leads**

`llo_trim` gains `double min_lead_s` (header default `0.0`, comment "the first apsis at least `min_lead_s` ahead").
Replace the `dt` line:

```cpp
    const double period = 2.0 * constants::pi / mean_motion;
    const double to_periapsis = time_to_mean_anomaly(mean_anomaly, 0.0, mean_motion);
    const double to_apoapsis = time_to_mean_anomaly(mean_anomaly, constants::pi, mean_motion);
    double dt = far_future_s;  // the earliest apsis at least min_lead_s ahead
    for (double c : {to_periapsis, to_apoapsis, to_periapsis + period, to_apoapsis + period}) {
        if (c >= min_lead_s) dt = std::min(dt, c);
    }
```

(define `constexpr double far_future_s = 1e30;` in the file's anonymous namespace if none exists; a `min_lead_s`
longer than one period is a caller error - assert `min_lead_s < period` with `std::invalid_argument`.)

`station_keeping` gains `double lead_s = 60.0` after `deadband_m`; replace `station_keeping_lead_s` in the solve call by
`lead_s` and delete the constant (its value moves into the default with its `[E]` comment in the header).

- [ ] **Step 5: Implement the session changes** (`flight_session.hpp`, `flight_session_lunar.cpp`, `flight_session.cpp`)

`flight_session.hpp`, next to `burn_preparation_s`:

```cpp
    static constexpr double small_burn_margin_s = 60.0;    // [E] trims / station keeping: planned after the preparation
    static constexpr double min_departure_lead_s = 1800.0; // [E] TLI centre after 120 s preparation + half a ~7 min burn
```

and members `gnc::SunHold lunar_sun_hold_;`, `bool lunar_debug_ = false;`, `std::string lunar_debug_last_;`
(`#include "starsim/gnc/sun_hold.hpp"`). Constructor (`flight_session.cpp`):
`lunar_debug_ = std::getenv("STARSIM_DEBUG_LUNAR_SESSION") != nullptr;`.

`flight_session_lunar.cpp`:
- delete `sun_settled_error_rad` / `sun_settled_rate_radps`; in `lunar_enter_coast` and at the re-point
  (`attitude_mode_ = {gnc::AttitudeModeKind::Sun, ...}`) add `lunar_sun_hold_.start(epoch_);`. In `Coast`:

```cpp
            if (attitude_mode_.kind == gnc::AttitudeModeKind::Sun && lunar_sun_hold_.active()) {
                // ... error as now ...
                const auto held = lunar_sun_hold_.update(error, norm(s.angular_velocity_body_radps), epoch_);
                if (held != gnc::SunHoldResult::Holding) {
                    attitude_mode_ = {};  // pointed (or stuck: coast freely instead of spending cold gas)
                    if (lunar_debug_) {
                        std::printf("[lunar] Sun %s: %.2f deg, %.4f deg/s, %.0f kg\n",
                                    held == gnc::SunHoldResult::Settled ? "pointed" : "hold timed out",
                                    error / constants::deg_to_rad,
                                    norm(s.angular_velocity_body_radps) / constants::deg_to_rad,
                                    flying.body->vehicle().mass_properties().mass_kg);
                    }
                }
            }
```

- every `std::getenv("STARSIM_DEBUG_LUNAR_SESSION")` -> `lunar_debug_`; the `static std::string last` ->
  `lunar_debug_last_`.
- `Trim` step: `planner.llo_trim(controlled_state(), epoch_, *lunar_parameters_, burn_preparation_s + small_burn_margin_s)`.
- `InNrho` step: `transfer.station_keeping(controlled_state(), epoch_, *lunar_nrho_, 1000.0, burn_preparation_s + small_burn_margin_s)`.
- `plan_lunar_outbound`, after the orientation check:

```cpp
    if (reference_body(controlled_) == ephemeris::Body::Moon) {
        lunar_status_ = "Kein Mondflug: schon beim Mond";
        return false;
    }
    auto planned = parameters;  // the TLI needs its preparation and half its burn before the centre
    planned.earliest_departure_s = std::max(planned.earliest_departure_s, min_departure_lead_s);
```

and pass `planned` instead of `parameters` to `planner.plan` and wherever `parameters` is stored afterwards
(`lunar_parameters_`).

- Comments at the NRHO insertion return in `LunarOutboundPlanner::insertion` and the NRHO branch of
  `LunarOrbitTransfer::arrival_burn`: `// Velocity match only: the position error at the perilune is left to the station keeping.`

- [ ] **Step 6: Run the touched tags**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[sun],[outbound],[nrho],[transfer],[lunar]"`
Expected: PASS (the `[slow]` lunar flights included: trims now one apsis later, station keeping 2 min later - LLO
within 1 km and the NRHO bands of l3 must still hold; record the new numbers).

- [ ] **Step 7: Docs**

`config/README.md`, `lunar_landing` keys: `earliest_departure_h` - "0 = as early as possible; the session never
plans the TLI centre earlier than 30 min after the start". `docs/subsystems.md`, lunar section: the small-burn lead,
the departure lead, `SunHold` (timeout 30 min), velocity-only matches, the Moon-bound refusal; in the flight-session /
booster-return notes the new file map of Tasks 2-3. CLAUDE.md is unchanged (no new module).

- [ ] **Step 8: Full suite, bindings, smoke test**

```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1
external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd
```

Expected: all tests PASS (507 + 11 new), smoke test OK. Output to a file; read its tail.

- [ ] **Step 9: Commit, push, final review**

```bash
git add core config/README.md docs/subsystems.md
git commit -m "fix(sim): deferred l3 minors (small-burn and departure leads, Sun hold timeout, Moon-bound refusal)"
git push origin main
```

Then one final review (opus) of `4264952..HEAD`; fix Critical / Important, list Minor in the ledger.

---

## Ledger

Rulings:
- Task 1: Ruling: contact/primitives.hpp had an identical column(Mat3,int) that became ambiguous via ADL with math::column — removed it (math's is the same) — cost if wrong: none, same formula.
- Task 1: Ruling: the plan's tag run (incl. [outbound],[mission]) runs serially in one process (>27 min, pulled in Dragon campaigns) — aborted; verified with -Fast (parallel, 484/484, 1:48); slow lunar flights wait for the Task 5 full suite — cost if wrong: a slow-only regression found in Task 5 instead of here.
- Task 2: Ruling: launch_site_from_config / drone_ship_site (free functions declared in flight_session.hpp, not in the plan's lists) stay in flight_session.cpp — cost if wrong: none, a later move.
- Task 2: Ruling: Step 3's "remove while the build stays green" would strip every include (all reach the TU through flight_session.hpp); kept the std/starsim headers each file uses directly (include-what-you-use), dropped only unused ones (descent_snapshot was dead) — cost if wrong: a few redundant includes.
- Task 3: Ruling: plan's tag run "[gnc],[catch],[booster]" pulled in slow campaigns serially again — aborted; ran "[gnc]~[slow],[catch]~[slow],[booster]~[slow]" (118 cases) + -Fast — cost if wrong: slow-only regression surfaces in Task 5 full suite.
- Task 3: Ruling: detail header also gets <algorithm> (engine_set uses std::find) and using math::Mat3/Vec3 inside booster_return_detail (moved code unchanged); prediction unit drops the unused Mat3 using; includes trimmed by direct use (burns: geodetic only; main: no kepler) — cost if wrong: none.
- Task 4: Ruling: test defects fixed in the tests, not the code - HLS separation test also merges hls.engines into the stack (Vehicle threw 'invalid map key'); forced-burn test uses advance_to(..., 100000) (default 4000 ticks stopped short of the node) and additionally checks the 3 RVacs fire and ship/canted is not in the configuration; lunar session test uses an upright attitude helper (upright_above_moon) from the start - cost if wrong: none.
- Task 5: Ruling: Moon-bound test checks "schon beim Mond" instead of "Mond" (every refusal says "Kein Mondflug") - cost if wrong: none.
- Task 5: Ruling: bindings/src/flight_sim.cpp still used contact::column removed in Task 1 (bindings not built until the full suite) - switched to math::column - cost if wrong: none, same formula.
- Task 5: Ruling: Step 6 'record the new numbers' - ctest hides passing output; the slow lunar flights pass their l3 bands (LLO < 1 km, NRHO bands) in the full suite, exact numbers not re-recorded - cost if wrong: docs keep the l3 numbers.

Executed 2026-10-15 inline (executing-plans): Tasks 1-5 complete (4a60a02, 260f7a4, db73a46, f35801e, 4300ef8);
full suite 518/518 + Godot smoke test. Final review (opus) of 4264952..4300ef8: 0 Critical, 0 Important.

Deferred minors (final review):
- Automatic MCCs are still planned only `burn_preparation_s` ahead (no `small_burn_margin_s`); harmless while the MCC
  burn is < 240 s - add the margin or document the exemption.
- `min_departure_lead_s` (1800 s) comment/docs say "120 s preparation + half a ~7 min burn" (~330 s): state the real
  reason (margin for the finite-TLI re-solve, equals the planner default); a TLI window < 30 min away now waits one
  parking-orbit revolution - say so in docs/subsystems.md.
- `j2_insertion.cpp` Gauss-Newton now refuses a nearly collinear Jacobian by the relative 1e-14 test (was |det| <
  1e-300); `allocate_fins` returns zero deflections on refusal (practically unreachable) - undocumented.
- Tests: `upright_above_moon` (test_lunar_session.cpp) could use `Mat3::from_columns`; `++vacuum;    }` in
  test_engine_placement.cpp needs formatting.
- Lunar tile staleness check accepts a site moved by < 1 px (comment it).
- subsystems.md l3 trim / station-keeping numbers were not re-measured after the new leads (bands pass).
