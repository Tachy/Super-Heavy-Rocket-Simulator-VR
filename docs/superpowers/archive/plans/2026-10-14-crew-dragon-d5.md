# Crew Dragon d5: Game Interface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The game can start both Dragon missions through `FlightSim.start_mission` and can read the state of every RCS
thruster, every parachute set and the capsule (nose cone, trunk, return step, splashdown zone and miss). No drawing and
no UI.

**Architecture:** Mission dispatch moves into the core (`sim/mission_start`): a `kind` key in `[mission]` selects the
loader. A capsule return starts at its own scenario epoch, because the orbit's RAAN is tied to it. The core gains three
small accessors that the bindings read:
- `Vehicle::rcs_duty()`
- `FlightBody::point_air_velocity_body_mps()`
- `vehicle::to_string(ParachuteState)`

The bindings stay thin: dictionaries and arrays built from core state. The capsule return gets a terminal step "verloren"
after an impact (d4 deferred minor).

**Tech Stack:** C++20 core (Catch2), godot-cpp GDExtension, GDScript smoke test.

**Spec:** `docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md` (section "d5 design: game interface")

## Global Constraints

- `core` has NO Godot dependency; `bindings` is thin: no physics or business logic (CLAUDE.md).
- No drawing and no UI work (user 2026-10-14: "die gesamte grafische Darstellung und die UI muss ich erst mal festlegen"). Bindings may be built ahead.
- UI strings in the core are German; code, comments, commits English.
- SI units internally; the bindings expose degrees only where the name says `_deg` (existing pattern).
- Positions in body dictionaries are vehicle body frame metres (as `body_engines`, `body_grid_fins`).
- Spec d5: "`FlightSim`: `body_rcs(i)` (thrusters: position, direction, firing, duty; for plumes), `body_parachutes(i)` (state, inflation, attachment, direction), capsule flags (nose cone, trunk attached), missions `falcon9_crew_dragon` and `crew_dragon_return` via `start_mission`. Covered by the smoke test. No drawing or UI."
- Commits: Conventional Commits with the session trailer lines.

## Review Focus

1. `start_mission("crew_dragon_return", tdb)` with any `tdb`. The game expects the flight to begin at the mission's own epoch (`mission_start_tdb_s()`), not at `tdb`. Pinned in Task 1 ("A capsule return mission starts at its own epoch ...").
2. Starting a mission of the wrong kind, such as `ship_return_starbase`. Expected: a clear refusal (`false` plus an error naming the kind), not a missing-key config error. Pinned in Task 1 (dispatcher test) and in the smoke test (Task 4).
3. A capsule that hits the water too fast, or breaks up. Expected: the return step ends ("verloren", autopilot off) and does not stay at "Bremsschirme". Pinned in Task 3.
4. `body_rcs` / `body_parachutes` / `capsule_state` called with an invalid index, before a flight, or on a vehicle without thrusters or parachutes. Expected: empty results (`present = false`), no crash. Pinned in the smoke test (Task 4).
5. `rcs_duty()` before any `update_rcs`, after `stop_rcs`, or on a body that is not controlled. Expected: empty, which reads as duty 0, never an out-of-range read. Pinned in Task 2 and guarded in Task 4.

---

## File Structure

- Create `core/include/starsim/sim/mission_start.hpp`, `core/src/sim/mission_start.cpp`: kind dispatch + start of any startable mission.
- Modify `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp`: `MissionKind`, `mission_kind()`, kind checks in the loaders.
- Modify `config/missions/crew_dragon_return.toml`, `config/missions/ship_return_starbase.toml`: `kind` key.
- Modify `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp`: `rcs_duty()`, `to_string(ParachuteState)`.
- Modify `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp`: `point_air_velocity_body_mps()`.
- Modify `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`: `CapsuleReturnStep::Lost`.
- Modify `core/include/starsim/sim/capsule_return_mission.hpp`, `core/src/sim/capsule_return_mission.cpp`: overload with a loaded `VehicleConfig`.
- Create `core/tests/test_mission_start.cpp`; modify `core/tests/test_rcs.cpp`, `core/tests/test_parachute.cpp`, `core/tests/test_capsule_splashdown.cpp`, `core/tests/CMakeLists.txt`, `core/CMakeLists.txt`.
- Modify `bindings/src/flight_sim.hpp`, `bindings/src/flight_sim.cpp`; `game/tests/smoke_test.gd`.
- Modify `CLAUDE.md`, `config/README.md`.

Branch: `feat/crew-dragon-game-interface` from `main` (a1c290f).

---

### Task 1: Mission kinds and the core mission start

**Files:**
- Create: `core/include/starsim/sim/mission_start.hpp`, `core/src/sim/mission_start.cpp`, `core/tests/test_mission_start.cpp`
- Modify: `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp`, `config/missions/crew_dragon_return.toml`, `config/missions/ship_return_starbase.toml`, `core/CMakeLists.txt` (after `src/sim/capsule_return_mission.cpp`), `core/tests/CMakeLists.txt` (after `test_crew_dragon.cpp`)

**Interfaces:**
- Produces:
  - `enum class config::MissionKind { Ascent, ShipReturn, CapsuleReturn };`
  - `config::MissionKind config::mission_kind(const std::filesystem::path& config_dir, std::string_view mission_id);` — reads `[mission] kind`, `"ascent"` when absent; throws `ConfigError` for an unknown kind or a missing file.
  - `struct sim::MissionStart { config::MissionKind kind; std::string site_id; time::Epoch epoch; bool return_planned = false; };`
  - `sim::MissionStart sim::start_mission(FlightSession& session, const std::filesystem::path& config_dir, std::string_view mission_id, time::Epoch requested, const time::TimeScaleConverter& time);`
    - Ascent: `start_ascent_mission` at `requested`.
    - CapsuleReturn: `place_capsule_return_mission` at `time.from_utc(m.epoch_utc)`, then `plan_capsule_return(capsule_return_parameters(m))`; `return_planned` is its result.
    - ShipReturn: throws `std::invalid_argument` ("... is flown from orbit with plan_ship_return (key R), not started").

- [ ] **Step 1: Write the failing tests** in `core/tests/test_mission_start.cpp`:

```cpp
// Mission kinds and the core mission start (d5 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <stdexcept>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

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

}  // namespace

TEST_CASE("Missions name their kind; ascent is the default", "[mission][config]") {
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "falcon9_crew_dragon") == config::MissionKind::Ascent);
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "starship_leo") == config::MissionKind::Ascent);
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "crew_dragon_return") == config::MissionKind::CapsuleReturn);
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "ship_return_starbase") == config::MissionKind::ShipReturn);
    CHECK_THROWS_AS(config::mission_kind(STARSIM_CONFIG_DIR, "no_such_mission"), config::ConfigError);
}

TEST_CASE("A loader refuses a mission of another kind", "[mission][config]") {
    CHECK_THROWS_AS(config::load_ascent_mission(STARSIM_CONFIG_DIR, "crew_dragon_return"), config::ConfigError);
    CHECK_THROWS_AS(config::load_capsule_return_mission(STARSIM_CONFIG_DIR, "falcon9_crew_dragon"), config::ConfigError);
    CHECK_THROWS_AS(config::load_ship_return_mission(STARSIM_CONFIG_DIR, "crew_dragon_return"), config::ConfigError);
}

TEST_CASE("An ascent mission starts at the requested epoch with its site", "[mission][sim]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    const auto t = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const auto s = sim::start_mission(session, STARSIM_CONFIG_DIR, "falcon9_crew_dragon", t, time);
    CHECK(s.kind == config::MissionKind::Ascent);
    CHECK(s.site_id == "ccsfs_slc40");
    CHECK(s.epoch == t);
    CHECK(session.epoch() == t);
    CHECK(session.autopilot().state == sim::AutopilotState::PreLaunch);
}

TEST_CASE("A capsule return mission starts at its own epoch and plans the deorbit", "[mission][sim][dragon]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    const auto requested = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);  // ignored: the RAAN belongs to the epoch
    const auto s = sim::start_mission(session, STARSIM_CONFIG_DIR, "crew_dragon_return", requested, time);
    const auto m = config::load_capsule_return_mission(STARSIM_CONFIG_DIR, "crew_dragon_return");
    CHECK(s.kind == config::MissionKind::CapsuleReturn);
    CHECK(s.site_id.empty());
    CHECK(s.epoch == time.from_utc(m.epoch_utc));
    CHECK(session.epoch() == s.epoch);
    CHECK(s.return_planned);
    CHECK(session.capsule_return_step() == sim::CapsuleReturnStep::DeorbitBurn);
    CHECK(session.splashdown_zone().has_value());
}

TEST_CASE("A ship return mission is not started, it is flown from orbit", "[mission][sim]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    CHECK_THROWS_AS(sim::start_mission(session, STARSIM_CONFIG_DIR, "ship_return_starbase",
                                       time::Epoch::from_tdb_seconds_past_j2000(8.4e8), time),
                    std::invalid_argument);
}
```

The SPICE helpers are file-local in every test file, as in `test_capsule_splashdown.cpp`. `Epoch` has a defaulted
`<=>`, so `==` works.

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests > "$L" 2>&1; grep "error C" $L`
Expected: compile errors, because `mission_kind`, `MissionKind` and `sim/mission_start.hpp` are not declared yet.

- [ ] **Step 3: Implement.**

In `mission_config.hpp` (before `ShipReturnMission`):

```cpp
// [mission] kind: what the mission file describes (FlightSim.start_mission dispatches on it). Absent: "ascent".
enum class MissionKind { Ascent, ShipReturn, CapsuleReturn };
MissionKind mission_kind(const std::filesystem::path& config_dir, std::string_view mission_id);
```

In `mission_config.cpp`:

```cpp
namespace {

MissionKind kind_of(const detail::Document& doc) {
    const auto* mission = doc.root["mission"].as_table();
    if (!mission) doc.fail("missing [mission]");
    const auto kind = doc.text_or(*mission, "kind", "ascent");
    if (kind == "ascent") return MissionKind::Ascent;
    if (kind == "ship_return") return MissionKind::ShipReturn;
    if (kind == "capsule_return") return MissionKind::CapsuleReturn;
    doc.fail("mission: unknown kind '" + kind + "' (ascent, ship_return, capsule_return)");
}

void require_kind(const detail::Document& doc, MissionKind kind, const char* name) {
    if (kind_of(doc) != kind) doc.fail(std::string("not a ") + name + " mission ([mission] kind)");
}

}  // namespace

MissionKind mission_kind(const std::filesystem::path& config_dir, std::string_view mission_id) {
    return kind_of(detail::Document(config_dir / "missions" / (std::string(mission_id) + ".toml")));
}
```

Add `require_kind(doc, MissionKind::ShipReturn, "ship return");` as the first line after the `Document` in
`load_ship_return_mission`. Add the matching call to the other two loaders, with `CapsuleReturn` / "capsule return" and
`Ascent` / "ascent".

TOML: in `[mission]` of `crew_dragon_return.toml` add `kind = "capsule_return"`. In `ship_return_starbase.toml` add
`kind = "ship_return"`. Each gets a comment: `# FlightSim.start_mission dispatches on it (absent: "ascent")`.

`core/include/starsim/sim/mission_start.hpp`:

```cpp
#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include "starsim/config/mission_config.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/time/time_scale_converter.hpp"

namespace starsim::sim {

struct MissionStart {
    config::MissionKind kind = config::MissionKind::Ascent;
    std::string site_id;      // launch site of an ascent; empty for a return from orbit
    time::Epoch epoch;        // where the flight begins: `requested` for an ascent, the mission's own epoch for a return
    bool return_planned = false;  // capsule return: the deorbit plan was feasible (else ship_return_plan()->reason)
};

// Starts mission `mission_id` of config/missions on `session` by its [mission] kind:
// - ascent: start_ascent_mission at `requested`;
// - capsule_return: place_capsule_return_mission at the mission's [orbit] epoch_utc (its RAAN belongs to that epoch),
//   then plan_capsule_return - the autopilot flies all steps to the water;
// - ship_return: refused (std::invalid_argument): the ship returns from the orbit it flew to (plan_ship_return).
MissionStart start_mission(FlightSession& session, const std::filesystem::path& config_dir, std::string_view mission_id,
                           time::Epoch requested, const time::TimeScaleConverter& time);

}  // namespace starsim::sim
```

`core/src/sim/mission_start.cpp`:

```cpp
#include "starsim/sim/mission_start.hpp"

#include <stdexcept>

#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/capsule_return_mission.hpp"

namespace starsim::sim {

MissionStart start_mission(FlightSession& session, const std::filesystem::path& config_dir, std::string_view mission_id,
                           time::Epoch requested, const time::TimeScaleConverter& time) {
    MissionStart s;
    s.kind = config::mission_kind(config_dir, mission_id);
    switch (s.kind) {
        case config::MissionKind::Ascent: {
            const auto m = config::load_ascent_mission(config_dir, mission_id);
            start_ascent_mission(session, m, config_dir, requested);
            s.site_id = m.site_id;
            s.epoch = requested;
            break;
        }
        case config::MissionKind::CapsuleReturn: {
            const auto m = config::load_capsule_return_mission(config_dir, mission_id);
            s.epoch = time.from_utc(m.epoch_utc);
            place_capsule_return_mission(session, m, config_dir, s.epoch);
            s.return_planned = session.plan_capsule_return(capsule_return_parameters(m));
            break;
        }
        case config::MissionKind::ShipReturn:
            throw std::invalid_argument("mission '" + std::string(mission_id) +
                                        "' is a ship return: it is flown from orbit with plan_ship_return (key R), not started");
    }
    return s;
}

}  // namespace starsim::sim
```

Add `src/sim/mission_start.cpp` to `core/CMakeLists.txt` and `test_mission_start.cpp` to `core/tests/CMakeLists.txt`.

- [ ] **Step 4: Run to verify it passes**

Run: build as in step 2, then `build/core/tests/starsim_core_tests.exe "[mission]"`
Expected: all `[mission]` tests pass, including the existing `[dragon][mission][config]` and `[ship][mission]` loads.
The capsule-return start plans a 48 h search; note its run time in the ledger, because the smoke test runs the same path.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/config/mission_config.hpp core/src/config/mission_config.cpp \
  core/include/starsim/sim/mission_start.hpp core/src/sim/mission_start.cpp core/tests/test_mission_start.cpp \
  core/CMakeLists.txt core/tests/CMakeLists.txt config/missions/crew_dragon_return.toml config/missions/ship_return_starbase.toml
git commit -m "feat(sim): missions name their kind; start_mission starts an ascent or a capsule return at its own epoch"
```

---

### Task 2: Core accessors for thrusters and parachutes

**Files:**
- Modify: `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp`, `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp`
- Test: `core/tests/test_rcs.cpp`, `core/tests/test_parachute.cpp`

**Interfaces:**
- Produces:
  - `const std::vector<double>& vehicle::Vehicle::rcs_duty() const;` — the allocator's duty (0..1) per thruster, in stage order, then thruster order within a stage (the order of `update_rcs`). Empty before the first `update_rcs` and after `stop_rcs`.
  - `const char* vehicle::to_string(ParachuteState state);` — "verstaut", "offen", "abgeworfen", "Fallschirm gerissen".
  - `math::Vec3 vehicle::FlightBody::point_air_velocity_body_mps(double station_m) const;` — the air velocity at a point on the axis at `station_m` (body frame), including omega x r about the CG, from the last tick's air data. `judge_parachute_loads` uses it too.

- [ ] **Step 1: Write the failing tests.**

Append to `core/tests/test_rcs.cpp`:

```cpp
TEST_CASE("The RCS duty is reported per thruster and empty when stopped", "[rcs][vehicle]") {
    DeepSpace space;
    auto body = stage_alone("falcon9_block5", 1);
    auto& v = body->vehicle();
    CHECK(v.rcs_duty().empty());  // nothing allocated yet
    vehicle::VehicleCommand roll;
    roll.rcs_torque_fraction = {0.3, 0.0, 0.0};
    body->step(space.env, t0, tick_s, roll);
    REQUIRE(v.rcs_duty().size() == v.stages()[0].rcs.size());
    double sum = 0.0;
    for (const double d : v.rcs_duty()) {
        CHECK(d >= 0.0);
        CHECK(d <= 1.0);
        sum += d;
    }
    CHECK(sum > 0.0);
    v.stop_rcs();
    CHECK(v.rcs_duty().empty());
}
```

Append to `core/tests/test_parachute.cpp`:

```cpp
TEST_CASE("Parachute states read in German; a torn set says so", "[parachute][vehicle]") {
    CHECK(std::string(vehicle::to_string(vehicle::ParachuteState::Stowed)) == "verstaut");
    CHECK(std::string(vehicle::to_string(vehicle::ParachuteState::Open)) == "offen");
    CHECK(std::string(vehicle::to_string(vehicle::ParachuteState::Released)) == "abgeworfen");
    CHECK(std::string(vehicle::to_string(vehicle::ParachuteState::Torn)) == "Fallschirm gerissen");
}

TEST_CASE("The air velocity at a riser point is the body's when it does not rotate", "[parachute][body]") {
    World w;
    time::Epoch t;
    auto body = falling_capsule(3000.0, 50.0);
    fall(*body, w.env, t, 0.1);
    const double station = body->vehicle().stages().front().parachutes.front().attach_station_m;
    const Vec3 v = body->point_air_velocity_body_mps(station);
    const Vec3 w_body = body->state().angular_velocity_body_radps;
    const auto mass = body->vehicle().mass_properties();
    const Vec3 arm{station - mass.cg_station_m, -mass.cg_offset_m.y, -mass.cg_offset_m.z};
    const Vec3 expected = body->air_data().air_velocity_body_mps + cross(w_body, arm);
    CHECK(norm(v - expected) < 1e-12);
    CHECK(norm(v) == Catch::Approx(50.0).epsilon(0.05));
}
```

- [ ] **Step 2: Run to verify it fails**

Run: core build (task 1 step 2 command)
Expected: compile errors for `rcs_duty`, `to_string(ParachuteState)` and `point_air_velocity_body_mps`.

- [ ] **Step 3: Implement.**

`vehicle.hpp`, after `enum class ParachuteState ...`:

```cpp
const char* to_string(ParachuteState state);  // German (UI): "Fallschirm gerissen" for a torn set
```

In `class Vehicle`, next to `rcs_force_n()`:

```cpp
    // Duty (0..1) the allocator gave each thruster this tick, stage by stage in thruster order; empty before the first
    // update_rcs and after stop_rcs (all off).
    const std::vector<double>& rcs_duty() const { return rcs_duty_; }
```

`vehicle.cpp` (namespace `starsim::vehicle`):

```cpp
const char* to_string(ParachuteState state) {
    switch (state) {
        case ParachuteState::Stowed: return "verstaut";
        case ParachuteState::Open: return "offen";
        case ParachuteState::Released: return "abgeworfen";
        case ParachuteState::Torn: return "Fallschirm gerissen";
    }
    return "?";
}
```

`flight_body.hpp`, next to `air_data()`:

```cpp
    // Air velocity (body frame) of a point on the axis at `station_m`: the CG's plus omega x r (last tick's air data).
    math::Vec3 point_air_velocity_body_mps(double station_m) const;
```

`flight_body.cpp`:

```cpp
Vec3 FlightBody::point_air_velocity_body_mps(double station_m) const {
    const auto mass = vehicle_->mass_properties();
    const Vec3 arm{station_m - mass.cg_station_m, -mass.cg_offset_m.y, -mass.cg_offset_m.z};
    return air_data_.air_velocity_body_mps + cross(state_.angular_velocity_body_radps, arm);
}
```

In `judge_parachute_loads`, replace the `arm` and `v` lines with
`const Vec3 v = point_air_velocity_body_mps(chute.attach_station_m);`. Drop the then-unused `mass` local.

- [ ] **Step 4: Run to verify it passes**

Run: `build/core/tests/starsim_core_tests.exe "[rcs],[parachute]"`
Expected: all pass. The existing parachute load tests are unchanged, which proves the refactor.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/vehicle/vehicle.hpp core/src/vehicle/vehicle.cpp core/include/starsim/vehicle/flight_body.hpp \
  core/src/vehicle/flight_body.cpp core/tests/test_rcs.cpp core/tests/test_parachute.cpp
git commit -m "feat(vehicle): per-thruster RCS duty, riser-point air velocity and German parachute states for the game"
```

---

### Task 3: A lost capsule ends its return ("verloren")

d4 deferred minor: after an impact under the drogues, the capsule return step stays at "Bremsschirme" or
"Hauptschirme". A breakup in the entry also sets `impacted()` and also leaves the step hanging.

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`, `core/include/starsim/sim/capsule_return_mission.hpp`, `core/src/sim/capsule_return_mission.cpp`
- Test: `core/tests/test_capsule_splashdown.cpp`

**Interfaces:**
- Produces:
  - `CapsuleReturnStep::Lost` (`to_string`: "verloren"), set when the controlled capsule has `impacted()` before splashdown; the autopilot then goes Off.
  - Overload `void sim::place_capsule_return_mission(FlightSession&, const config::CapsuleReturnMission&, config::VehicleConfig vehicle, const std::filesystem::path& config_dir, time::Epoch epoch);`, which takes a loaded and possibly modified vehicle. The existing id-based overload loads it and forwards.

- [ ] **Step 1: Write the failing test.** In `test_capsule_splashdown.cpp`:
  - Extend `MissionDispersion` with `bool without_mains = false;`.
  - Extend `MissionRun` with `sim::CapsuleReturnStep final_step = sim::CapsuleReturnStep::None;` and `sim::AutopilotState final_autopilot = sim::AutopilotState::Off;`.
  - In `fly_mission`, replace `sim::place_capsule_return_mission(session, m, STARSIM_CONFIG_DIR, t0);` with:

```cpp
    auto vehicle = config::load_vehicle(STARSIM_CONFIG_DIR, m.vehicle_id);
    if (dispersion.without_mains) {
        std::erase_if(vehicle.stages.back().parachutes, [](const auto& p) { return p.role == "main"; });
    }
    sim::place_capsule_return_mission(session, m, std::move(vehicle), STARSIM_CONFIG_DIR, t0);
```

  - At its end, before `return run;`:

```cpp
    session.advance_to(session.epoch() + 1.0, 1000000);  // the step machine sees the outcome
    run.final_step = session.capsule_return_step();
    run.final_autopilot = session.autopilot().state;
```

  - Add the test:

```cpp
TEST_CASE("A capsule return without mains ends lost, not under its drogues", "[dragon][mission][splashdown]") {
    MissionDispersion d;
    d.without_mains = true;
    const auto r = fly_mission(d);
    REQUIRE(r.planned);
    CHECK(r.descent.outcome == sim::LandingOutcome::Impact);   // ~50 m/s under the drogues alone
    CHECK(r.final_step == sim::CapsuleReturnStep::Lost);
    CHECK(std::string(sim::to_string(r.final_step)) == "verloren");
    CHECK(r.final_autopilot == sim::AutopilotState::Off);
}
```

  - Also add `CHECK(r.final_step == sim::CapsuleReturnStep::SplashedDown);` to the nominal mission test ("... splashes down in the zone on the published timeline").

`sim::LandingOutcome::Impact` is "Aufprall". `std::erase_if` needs `<vector>` (C++20).

- [ ] **Step 2: Run to verify it fails**

Run: core build; `build/core/tests/starsim_core_tests.exe "A capsule return without mains ends lost, not under its drogues"`
Expected: first a compile failure, because `Lost` and the overload are missing. With stubs in place, the test fails
with `final_step == Drogues`. Run time is about one mission, roughly a minute.

- [ ] **Step 3: Implement.**

`flight_session.hpp`: the enum becomes
`None, DeorbitBurn, TrunkJettison, NoseConeClose, Coast, Entry, Gate, Drogues, Mains, SplashedDown, Lost`, with the
comment `// Lost: the capsule impacted or broke up before resting on the water (autopilot off)`.

`flight_session.cpp` `to_string`: `case CapsuleReturnStep::Lost: return "verloren";`.

At the top of `run_capsule_return`:

```cpp
    if (flying.body->impacted() && capsule_return_step_ != CapsuleReturnStep::SplashedDown) {
        capsule_return_step_ = CapsuleReturnStep::Lost;  // d4 minor: the step stayed "Bremsschirme" after an impact
        autopilot_.state = AutopilotState::Off;
        return;
    }
```

Remove the now-redundant `if (flying.body->impacted()) autopilot_.state = AutopilotState::Off;` in the
Gate/Drogues/Mains case.

`capsule_return_mission.hpp/.cpp`: split `place_capsule_return_mission` so the body takes the vehicle:

```cpp
void place_capsule_return_mission(FlightSession& session, const config::CapsuleReturnMission& m,
                                  const std::filesystem::path& config_dir, time::Epoch epoch) {
    place_capsule_return_mission(session, m, config::load_vehicle(config_dir, m.vehicle_id), config_dir, epoch);
}
```

The overload (with `config_dir` for the zone) carries the current body: the RCS fill, `place_in_orbit`, the zone and the
nose cone.

- [ ] **Step 4: Run to verify it passes**

Run: `build/core/tests/starsim_core_tests.exe "[dragon][mission]"` (this includes the nominal mission and the 8-case
campaign; it takes several minutes, so run it in the background and wait with an until-loop).
Expected: all pass. The new test shows `Lost`; the nominal mission shows `SplashedDown`.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp \
  core/include/starsim/sim/capsule_return_mission.hpp core/src/sim/capsule_return_mission.cpp core/tests/test_capsule_splashdown.cpp
git commit -m "fix(sim): a capsule that impacts or breaks up ends its return as lost instead of staying under its drogues"
```

---

### Task 4: FlightSim game interface and smoke test

**Files:**
- Modify: `bindings/src/flight_sim.hpp`, `bindings/src/flight_sim.cpp`, `game/tests/smoke_test.gd`

**Interfaces:**
- Consumes: `sim::start_mission`, `sim::MissionStart` (Task 1); `Vehicle::rcs_duty()`, `vehicle::to_string(ParachuteState)`, `FlightBody::point_air_velocity_body_mps` (Task 2); `CapsuleReturnStep::Lost` (Task 3); the existing `FlightSession::capsule_return_step()`, `splashdown_zone()`, `splashdown_miss_m(i)`, `parachute_sequencer(i)`, `plan_capsule_return()`, `set_splashdown_zone()`, `set_nose_cone()`, `Vehicle::has_nose_cone()`, `nose_cone_open()`, `parachute_torn()`.
- Produces (GDScript API):
  - `start_mission(id, tdb_s) -> bool`: now any startable kind. False plus `push_error` for unknown, invalid or ship-return missions.
  - `mission_start_tdb_s() -> float`: the epoch the current mission began at. For a capsule return this is its own epoch; the world clock jumps there.
  - `body_rcs(i) -> Array[Dictionary]`, one entry per thruster: `{id, stage, position_m, direction, thrust_n, firing, duty, starved, blocked}`.
  - `body_parachutes(i) -> Array[Dictionary]`, one entry per set: `{id, role, state, count, attach_station_m, drag_area_m2, full_drag_area_m2, inflation, direction, peak_load_n, max_load_n}`.
    - `direction`: body-frame unit vector from the riser point towards the canopy, i.e. against the point's air velocity; body +x when the air speed is below 0.1 m/s.
    - `inflation` = `drag_area / full_drag_area`.
  - `capsule_state(i) -> Dictionary`: `{present}`; when present also `{nose_cone (has one), nose_cone_open, trunk_attached, return_step, parachute_phase, parachute_torn, splashdown_miss_m (when a zone is set), outcome}`.
    - `present` holds when the body's top stage has `entry_profile == "capsule"`.
    - `parachute_phase` is "" without a sequencer.
  - `splashdown_zone() -> Dictionary`: `{present}`; when present also `{id, name, latitude_deg, longitude_deg, radius_m, origin_itrf_m, east, north, up}`.
  - `set_splashdown_zone(zone_id) -> bool`, `plan_capsule_return() -> bool`.
  - `autopilot_status()["capsule_step"]` (German) while a capsule return runs.

- [ ] **Step 1: Write the failing smoke checks.** In `game/tests/smoke_test.gd`, call `_check_dragon(config_dir, tdb)`
  after `_check_falcon9(...)`, and add:

```gdscript
# Crew Dragon (d5 game interface): both missions via start_mission, thrusters, parachutes and the capsule state.
func _check_dragon(config_dir: String, tdb: float) -> void:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	_check(sim.body_rcs(0).is_empty() and sim.body_parachutes(0).is_empty()
			and not sim.capsule_state(0).get("present", true), "before a flight: empty thruster/parachute/capsule state")
	_check(sim.start_mission("falcon9_crew_dragon", tdb), "Falcon 9 with Crew Dragon starts")
	_check(absf(sim.mission_start_tdb_s() - tdb) < 1e-9, "an ascent starts at the requested time")
	var rcs: Array = sim.body_rcs(0)
	_check(rcs.size() == 8 + 8 + 16, "stack: 8 + 8 Falcon 9 thrusters and 16 Dracos")
	var dracos_blocked := true
	for t in rcs:
		if t.get("stage", 0) == 3 and not t.get("blocked", false):
			dracos_blocked = false
	_check(dracos_blocked, "stacked Dracos are blocked")
	_check((rcs[0] as Dictionary).has_all(["position_m", "direction", "firing", "duty", "thrust_n"]), "thruster fields")
	_check(not sim.capsule_state(0).get("present", true), "the stack's top stage is the capsule's: still not a capsule body")
	_check(sim.body_parachutes(0).size() == 2, "the stacked capsule carries its two parachute sets")
	_check(not sim.start_mission("ship_return_starbase", tdb), "a ship return is not started by start_mission")

	var ret := FlightSim.new()
	ret.setup(config_dir)
	_check(ret.start_mission("crew_dragon_return", tdb), "Crew Dragon return mission starts")
	_check(absf(ret.mission_start_tdb_s() - tdb) > 1.0, "the return starts at its own epoch, not the requested one")
	var capsule: Dictionary = ret.capsule_state(ret.controlled_index())
	_check(capsule.get("present", false) and capsule.get("trunk_attached", false) and capsule.get("nose_cone_open", false),
			"capsule on its trunk with the nose cone open")
	_check(capsule.get("return_step", "") == "Deorbit-Brennvorgang", "the deorbit burn is planned")
	_check(capsule.get("parachute_phase", "x") == "", "no parachute sequencer while the trunk is attached")
	_check(ret.autopilot_status().get("return", {}).get("feasible", false), "the return plan is feasible")
	var zone: Dictionary = ret.splashdown_zone()
	_check(zone.get("present", false) and absf(float(zone.get("radius_m", 0.0)) - 2000.0) < 1e-9, "splashdown zone radius 2 km")
	var chutes: Array = ret.body_parachutes(ret.controlled_index())
	_check(chutes.size() == 2 and chutes[0].get("state", "") == "verstaut" and float(chutes[0].get("inflation", 1.0)) == 0.0,
			"parachutes stowed")
	_check(ret.body_rcs(ret.controlled_index()).size() == 16, "Crew Dragon alone: 16 Dracos")
	ret.advance_to(ret.mission_start_tdb_s() + 10.0)
	_check(ret.is_active(), "the return mission advances")
	_check(ret.body_rcs(9).is_empty() and ret.body_parachutes(9).is_empty() and ret.capsule_state(9).is_empty(),
			"invalid body index: empty results")
```

Stack stages: `first_stage` (0), `second_stage` (1), `trunk` (2), `capsule` (3). Thrusters: 8 on the first stage,
8 on the second, 16 Dracos.

`capsule_state(i).present` = the top stage has `entry_profile == "capsule"` and is not a `payload` stage
(`StageConfig::payload`). A capsule still stacked on its launcher is part of a launch stack, not a capsule body.

- [ ] **Step 2: Run to verify it fails**

Run: `cmd //c "C:\\Users\\johan\\.claude\\jobs\\78816a1a\\tmp\\build_noscan.cmd"` (bindings build), then the smoke test
`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd`.
Expected: script errors for the missing methods `body_rcs`, `capsule_state` and so on, so the run is not "SMOKE TEST PASSED".

- [ ] **Step 3: Implement the bindings.**

`flight_sim.hpp`:
- Add `#include "starsim/spice/spice_adapters.hpp"` (it is already there) and the member `spice::SpiceTimeConverter time_;`.
- Add `double mission_start_tdb_s_ = 0.0;`.
- Declare, with doc comments copied from the Interfaces block:

```cpp
    double mission_start_tdb_s() const { return mission_start_tdb_s_; }
    godot::Array body_rcs(int index) const;
    godot::Array body_parachutes(int index) const;
    godot::Dictionary capsule_state(int index) const;
    godot::Dictionary splashdown_zone() const;
    bool set_splashdown_zone(const godot::String& zone_id);
    bool plan_capsule_return();
```

`flight_sim.cpp`:

```cpp
bool FlightSim::start_mission(const godot::String& mission_id, double tdb_s) {
    if (!session_) {
        godot::UtilityFunctions::push_error("FlightSim: call setup() first");
        return false;
    }
    try {
        const auto s = sim::start_mission(*session_, config_dir_, mission_id.utf8().get_data(), epoch(tdb_s), time_);
        site_id_ = godot::String::utf8(s.site_id.c_str());
        mission_start_tdb_s_ = s.epoch.tdb_seconds_past_j2000();
        return true;
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String("FlightSim.start_mission: ") + e.what());
        return false;
    }
}

godot::Array FlightSim::body_rcs(int index) const {
    godot::Array thrusters;
    if (!check_index(index)) return thrusters;
    const auto& vehicle = session_->body(index).vehicle();
    const auto& duty = vehicle.rcs_duty();
    size_t k = 0;
    const auto& stages = vehicle.stages();
    for (size_t s = 0; s < stages.size(); ++s) {
        for (const auto& t : stages[s].rcs) {
            godot::Dictionary d;
            d["id"] = godot::String::utf8(t.id.c_str());
            d["stage"] = static_cast<int64_t>(s);
            d["position_m"] = to_godot(t.position_m);
            d["direction"] = to_godot(t.direction);  // force on the vehicle; the plume points the other way
            d["thrust_n"] = t.thrust_n;
            d["firing"] = t.output > 0.0;
            d["duty"] = k < duty.size() ? duty[k] : 0.0;
            d["starved"] = t.starved;
            d["blocked"] = t.blocked;
            thrusters.push_back(d);
            ++k;
        }
    }
    return thrusters;
}

godot::Array FlightSim::body_parachutes(int index) const {
    godot::Array sets;
    if (!check_index(index)) return sets;
    const auto& body = session_->body(index);
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& p : stage.parachutes) {
            const double full = p.config.count * p.config.drag_area_m2;
            const math::Vec3 v = body.point_air_velocity_body_mps(p.attach_station_m);
            const double speed = norm(v);
            godot::Dictionary d;
            d["id"] = godot::String::utf8(p.config.id.c_str());
            d["role"] = godot::String::utf8(p.config.role.c_str());
            d["state"] = godot::String::utf8(vehicle::to_string(p.state));
            d["count"] = p.config.count;
            d["attach_station_m"] = p.attach_station_m;
            d["drag_area_m2"] = p.drag_area_m2;
            d["full_drag_area_m2"] = full;
            d["inflation"] = full > 0.0 ? p.drag_area_m2 / full : 0.0;
            d["direction"] = speed > 0.1 ? to_godot(v * (-1.0 / speed)) : godot::Vector3(1.0, 0.0, 0.0);
            d["peak_load_n"] = p.peak_load_n;
            d["max_load_n"] = p.config.max_load_n;
            sets.push_back(d);
        }
    }
    return sets;
}

godot::Dictionary FlightSim::capsule_state(int index) const {
    godot::Dictionary d;
    if (!check_index(index)) return d;
    const auto i = static_cast<size_t>(index);
    const auto& vehicle = session_->body(i).vehicle();
    const auto& top = vehicle.stages().back().config;
    d["present"] = top.aero.entry_profile == "capsule" && !top.payload;  // a capsule flying (not stacked on a launcher)
    if (!bool(d["present"])) return d;
    d["nose_cone"] = vehicle.has_nose_cone();
    d["nose_cone_open"] = vehicle.nose_cone_open();
    d["trunk_attached"] = vehicle.stages().size() > 1;
    d["return_step"] = godot::String::utf8(sim::to_string(session_->capsule_return_step()));
    const auto* seq = session_->parachute_sequencer(i);
    d["parachute_phase"] = seq ? godot::String::utf8(gnc::to_string(seq->phase())) : godot::String();
    d["parachute_torn"] = vehicle.parachute_torn();
    if (session_->splashdown_zone()) d["splashdown_miss_m"] = session_->splashdown_miss_m(i);
    d["outcome"] = godot::String::utf8(sim::to_string(session_->landing_outcome(i)));
    return d;
}

godot::Dictionary FlightSim::splashdown_zone() const {
    godot::Dictionary d;
    d["present"] = false;
    if (!session_ || !session_->splashdown_zone()) return d;
    const auto& z = *session_->splashdown_zone();
    const double lat = z.latitude_rad, lon = z.longitude_rad;
    d["present"] = true;
    d["id"] = godot::String::utf8(z.id.c_str());
    d["name"] = godot::String::utf8(z.name.c_str());
    d["latitude_deg"] = lat * constants::rad_to_deg;
    d["longitude_deg"] = lon * constants::rad_to_deg;
    d["radius_m"] = z.radius_m;
    d["origin_itrf_m"] = to_godot(frames::geodetic_to_fixed({lat, lon, 0.0}));  // sea level (WGS84 height 0)
    d["east"] = godot::Vector3(-std::sin(lon), std::cos(lon), 0.0);
    d["north"] = godot::Vector3(-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat));
    d["up"] = godot::Vector3(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
    return d;
}

bool FlightSim::set_splashdown_zone(const godot::String& zone_id) {
    if (!session_) return false;
    try {
        session_->set_splashdown_zone(config::load_splashdown_zone(config_dir_, zone_id.utf8().get_data()));
        return true;
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String("FlightSim.set_splashdown_zone: ") + e.what());
        return false;
    }
}

bool FlightSim::plan_capsule_return() {
    if (!is_active()) return false;
    try {
        return session_->plan_capsule_return();
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String("FlightSim.plan_capsule_return: ") + e.what());
        return false;
    }
}
```

In `autopilot_status()`, before `return d;`:

```cpp
    if (session_->capsule_return_step() != sim::CapsuleReturnStep::None) {
        d["capsule_step"] = godot::String::utf8(sim::to_string(session_->capsule_return_step()));
    }
```

Includes: `starsim/sim/mission_start.hpp`, `starsim/config/splashdown_zone_config.hpp`, `starsim/vehicle/vehicle.hpp`.
Remove the now-unused `starsim/sim/ascent_mission.hpp` include only if nothing else uses it. In `launch_from_site` and
`launch_from_pad`, also set `mission_start_tdb_s_ = tdb_s;`.

Bind all new methods in `_bind_methods`:

```cpp
    ClassDB::bind_method(D_METHOD("mission_start_tdb_s"), &FlightSim::mission_start_tdb_s);
    ClassDB::bind_method(D_METHOD("body_rcs", "index"), &FlightSim::body_rcs);
    ClassDB::bind_method(D_METHOD("body_parachutes", "index"), &FlightSim::body_parachutes);
    ClassDB::bind_method(D_METHOD("capsule_state", "index"), &FlightSim::capsule_state);
    ClassDB::bind_method(D_METHOD("splashdown_zone"), &FlightSim::splashdown_zone);
    ClassDB::bind_method(D_METHOD("set_splashdown_zone", "zone_id"), &FlightSim::set_splashdown_zone);
    ClassDB::bind_method(D_METHOD("plan_capsule_return"), &FlightSim::plan_capsule_return);
```

The comment on `start_mission` in the header becomes: "Mission of config/missions by its [mission] kind: an ascent
starts at `tdb_s` (payload, pad, landing zone or drone ship, booster return, ascent autopilot); a capsule return at its
own epoch (`mission_start_tdb_s()`) with the deorbit planned; false (error pushed) for an unknown, invalid or ship-return
mission."

- [ ] **Step 4: Run to verify it passes**

Run: the bindings build (`BUILD_OK`), then the smoke test.
Expected: `SMOKE TEST PASSED` with every new `ok` line. Record the run time of the return-mission start in the ledger.

- [ ] **Step 5: Commit**

```bash
git add bindings/src/flight_sim.hpp bindings/src/flight_sim.cpp game/tests/smoke_test.gd
git commit -m "feat(bindings): Crew Dragon game interface - thrusters, parachutes, capsule state, splashdown zone, Dragon missions"
```

---

### Task 5: Docs and the full suite

**Files:**
- Modify: `CLAUDE.md`, `config/README.md`

- [ ] **Step 1: Write the docs.**

In `CLAUDE.md`:
- Under "Game (Godot) structure", after the d4 bullet, add:

```
- Game interface for Crew Dragon (d5 of the same spec; the drawing and UI are specified by the user first):
  `FlightSim.start_mission` dispatches on `[mission] kind` (`sim::start_mission`: "ascent" default, "capsule_return" starts
  at the mission's own epoch - `mission_start_tdb_s()` - with the deorbit planned; "ship_return" is refused: key R from
  orbit). `body_rcs(i)` (per thruster: stage, position, direction, thrust, firing, duty, starved, blocked),
  `body_parachutes(i)` (state German incl. "Fallschirm gerissen", drag area, inflation, riser station, canopy direction
  against the riser point's air velocity, loads), `capsule_state(i)` (nose cone, trunk, return step incl. "verloren" after
  an impact, parachute phase, splashdown miss), `splashdown_zone()`, `set_splashdown_zone`, `plan_capsule_return`,
  `autopilot_status()["capsule_step"]`. Covered by the smoke test.
```

- In the "Known issues" Crew Dragon parachutes bullet, append: `A capsule that impacts or breaks up ends its return as "verloren" (CapsuleReturnStep::Lost, autopilot off).`

In `config/README.md`, in the missions section, document `[mission] kind = "ascent" | "ship_return" | "capsule_return"`
(default "ascent"; `FlightSim.start_mission` dispatches on it, each loader refuses another kind).

- [ ] **Step 2: Run the full suite and the smoke test**

Run: `ctest --test-dir build -j 12` (full path in the ledger conventions) in the background, plus the smoke test.
Expected: everything passes (419 + the new tests) and `SMOKE TEST PASSED`.

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md config/README.md
git commit -m "docs: d5 Crew Dragon game interface"
```
