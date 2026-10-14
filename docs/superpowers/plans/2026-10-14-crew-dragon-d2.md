# Crew Dragon and orbital flight (d2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Falcon 9 lifts a Crew Dragon (capsule + trunk, a real vehicle) from SLC-40 into its 51.6 deg insertion orbit.
The capsule separates, opens its nose cone, holds attitude on its 16 Dracos, and flies small translation burns on them
with the existing burn executor.

**Architecture:**
- A payload vehicle is stacked on top of the launcher's configuration (`config::stack_payload`). Its stages are marked
  `payload`, and their RCS stays inhibited until the payload flies alone.
- The session separates the payload a fixed time after the upper stage's cutoff (spring push). It opens the nose cone
  later, and the capsule becomes the controlled body in an attitude mode (`gnc/orbital_attitude`: inertial, LVLH,
  direction, sun).
- RCS burns choose the body axis with the most thrust (+x first, as today). The hull turns that axis along the burn.
- SuperDracos are an `abort_only` engine group: present as data, never lit by the session.

**Tech Stack:** C++20 core (`core/`), Catch2 tests, TOML config, CMake/Ninja via `tools/build.ps1`.

**Spec:** `docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md` (section "d2 design: Crew Dragon and orbital
flight"; d1 is done, see CLAUDE.md "RCS (d1 ...)").

## Global Constraints

- Answer the user in German. Code, identifiers, comments, commit messages and docs are in English. UI strings in the
  core are German (e.g. `to_string(AutopilotState)`).
- SI units internally, angles in radians, and the frame or unit in each name (`direction_gcrf`, `delay_s`). Degrees
  only in config (`_deg`).
- Every config value carries a provenance tag: `[S:<ref>]`, `[E]` or `[D]`.
- RCS thrusters are on/off only, with fixed thrust. There is no continuous mode, and the autopilots fly on the pulses
  (user 2026-10-14).
- `core` must not depend on Godot. Bindings, the game and the UI are not touched in d2 (d5 builds the interface; the
  user specifies the drawing and UI).
- Commits follow Conventional Commits and end with:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE
  ```
- Work on the branch `feat/crew-dragon`, created from `main` (`f32313f`). Do not merge or push without the user's
  explicit request.
- Bash quoting mangles `\n` and quotes in heredocs. Write multi-line edits with the Edit or Write tool, or with a
  python script in `C:\Users\johan\.claude\jobs\78816a1a\tmp\`.
- Build (core): `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests > <log> 2>&1`, then
  grep the log for `" error "`.
- Build (with bindings): `cmd //c "C:\\Users\\johan\\.claude\\jobs\\78816a1a\\tmp\\build_noscan.cmd"`, which prints
  `BUILD_OK`.
- Single test: `build/core/tests/starsim_core_tests.exe "<name or [tag]>"`. Use a wildcard for names that contain a
  comma.
- Full suite (~4 min, 345/345 before d2):
  `"/c/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/ctest.exe" --test-dir build -j 12`.
- The existing campaigns must keep their results:
  - Super Heavy catch 21/21
  - ship from the gate 20/21
  - ship from orbit 14/14
  - Falcon 9 RTLS 21/21 and ASDS 21/21 (from ignition and from MECO)

  d2 does not change their vehicles' thrusters or engines. The full suite run at the end of each task covers them.

## Decisions taken in this plan (rulings for the user's review)

1. **Masses.**
   - Launch mass 12.5 t [S: Wikipedia "SpaceX Dragon 2"].
   - Propellant 2,562 kg MMH/NTO [S].
   - Trunk 1.5 t [E].
   - Capsule dry incl. crew 8,438 kg [D] = 12,500 - 2,562 - 1,500.

   The spec's "~9.5 t dry" contradicts its own 12.5 t launch mass (Wikipedia also gives 7.7 t dry for the capsule
   alone).
2. **Heights.**
   - Capsule 4.4 m [S, spec].
   - Trunk 3.7 m [D]: 8.1 m with the trunk [S: Wikipedia] minus the capsule. The spec's 2.5 m [E] is replaced.
3. **Draco layout [E]** (no public drawing):
   - 4 Dracos under the nose cone, 400 N, pushing aft (plume forward). They fire only with the cone open.
   - 4 side pods at 0/90/180/270 deg, each with tangential +, tangential - and forward thrusters.

   This gives pure couples on all three axes (a "balanced" layout: turns do not push the capsule in orbit) and
   translation along +x and -x. Lateral (y/z) translation is left out until docking (M9).
4. **Minimum on-time of the Dracos 10 ms [E]** (2 ticks), against 20 ms for the cold-gas thrusters. Pulsed hold with a
   2 Ns impulse quantum (spec choice 1) costs about 1 kg/h, not "grams per hour". The test bounds it and the ledger
   records the measured figure.
5. **Attitude hold deadband** `hold_deadband_rad` 0.5 deg, `hold_rate_deadband_radps` 0.05 deg/s [E] (new controller
   gains, default 0 = off). Without them a PD loop on 400 N pulses chatters (tens of kg/h).
6. **Coasting with an attitude mode.**
   - Mode `Free` (the default): coasting is allowed as today.
   - Any other mode: the controlled body is ticked at 200 Hz.
   - Long waits (deorbit planning, d3) switch to `Free`.
7. **Payload separation:**
   - at cutoff + 30 s [E], push 0.3 m/s [E] along body +x (spec);
   - the nose cone opens 600 s [E] after the separation;
   - the capsule then holds its attitude inertially (`AttitudeModeKind::Inertial`) and the second stage coasts unguided.

   These values are mission keys under `[payload]`.
8. **Crew mission booster:** RTLS to LZ-40, as Crew-9 [S: B1085 landed at LZ-1]. Target 200 x 210 km [S, spec],
   51.6 deg, kick 12 deg (the RTLS mission's: same 12.5 t on top).

## Review Focus

1. **The payload's RCS must not fire while it is stacked.**
   - Risk: a roll on the upper stage allocates to the Dracos (longer lever, stronger) and spends Dragon propellant.
   - Pinned by Task 2's test "A stacked payload's thrusters stay inhibited until it flies alone".
2. **Session paths that would light the SuperDracos.**
   - Risk: `ignite()` / `burn_configurations()` take the "lowest stage with engines" and could pick the capsule's
     SuperDracos once it is alone. A 1 m/s burn would then fly on 570 kN.
   - Pinned by Task 2's test "Abort-only engines are never lit by the session".
3. **An RCS burn whose strongest axis is -x** (the capsule with its side thrusters lost, or a future vehicle).
   - Risk: the hull-turn code divides by `sin(180 deg)`.
   - Pinned by Task 4's test "An RCS burn along -x turns the hull round and keeps the delta-v".
4. **Separation clearance.**
   - Risk: the capsule's attitude hold right after the spring push could thrust it back towards the second stage
     (one-station push).
   - Pinned by Task 5's test: the distance grows monotonically for 60 s and exceeds 10 m.
5. **Sessions with nothing to separate.**
   - Risk: missions without a payload vehicle (all existing ones) must not enter `PayloadSeparation`.
   - Pinned by Task 5's test "A mission without a payload vehicle ends in orbit as before" (the RTLS mission's
     autopilot state is `Orbit` after cutoff).

---

### Task 0: Branch and ledger

- [ ] **Step 1: Create the branch**

Run: `git switch -c feat/crew-dragon`
Expected: `Switched to a new branch 'feat/crew-dragon'`

- [ ] **Step 2: Create the ledger.** Run the executing-plans setup (`sdd-workspace` on this plan file). Write the
  first line `# SDD ledger — plan: docs/superpowers/plans/2026-10-14-crew-dragon-d2.md`, then record the eight
  decisions above as `Ruling:` lines (they reach the user in the final report).

---

### Task 1: Crew Dragon configuration and payload stacking

**Files:**
- Create: `config/vehicles/crew_dragon.toml`, `config/engines/superdraco.toml`, `core/tests/test_crew_dragon.cpp`
- Modify:
  - `config/propellants.toml` (add `mmh_nto`)
  - `core/include/starsim/config/vehicle_config.hpp` (`EngineGroupConfig::abort_only`,
    `RcsThrusterConfig::behind_nose_cone`, `StageConfig::nose_cone`, `StageConfig::payload`, `stack_payload`)
  - `core/src/config/vehicle_config.cpp` (load the keys, `stack_payload`)
  - `core/tests/CMakeLists.txt` (add `test_crew_dragon.cpp`)
  - `config/README.md`

**Interfaces:**
- Produces:
  - `bool EngineGroupConfig::abort_only` (TOML `abort_only`, default false)
  - `bool RcsThrusterConfig::behind_nose_cone` (pod or thruster key `behind_nose_cone`, default false)
  - `bool StageConfig::nose_cone` (`[[stage]] nose_cone = true`)
  - `bool StageConfig::payload` (set only by `stack_payload`)
  - `VehicleConfig config::stack_payload(VehicleConfig launcher, const VehicleConfig& payload)`

- [ ] **Step 1: Write the failing tests** in `core/tests/test_crew_dragon.cpp`:

```cpp
// Crew Dragon (d2 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md): vehicle data, payload stacking,
// thruster availability, orbital attitude modes, Draco translation burns.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

}  // namespace

TEST_CASE("Crew Dragon loads as capsule and trunk", "[dragon][config]") {
    const auto cfg = config::load_vehicle(config_dir, "crew_dragon");
    REQUIRE(cfg.stages.size() == 2);
    CHECK(cfg.stages[0].id == "trunk");
    CHECK(cfg.stages[1].id == "capsule");
    const auto& capsule = cfg.stages[1];
    CHECK(capsule.rcs.thrusters.size() == 16);
    int behind_cone = 0;
    for (const auto& t : capsule.rcs.thrusters) {
        CHECK(t.thrust_n == Catch::Approx(400.0));
        behind_cone += t.behind_nose_cone ? 1 : 0;
    }
    CHECK(behind_cone == 4);
    CHECK(capsule.nose_cone);
    REQUIRE(capsule.engine_groups.size() == 1);
    CHECK(capsule.engine_groups[0].count == 8);
    CHECK(capsule.engine_groups[0].abort_only);
    // 12.5 t at launch [S], 2,562 kg MMH/NTO [S] in the RCS tank (out of the propellant gauge).
    const vehicle::Vehicle v(cfg, config_dir);
    CHECK(v.mass_properties().mass_kg == Catch::Approx(12500.0));
    CHECK(v.rcs_propellant_kg() == Catch::Approx(2562.0));
    CHECK(v.propellant_kg() == 0.0);
}

TEST_CASE("A payload vehicle stacks on top of its launcher", "[dragon][config]") {
    const auto f9 = config::load_vehicle(config_dir, "falcon9_block5");
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    const auto stack = config::stack_payload(f9, dragon);
    REQUIRE(stack.stages.size() == 4);
    CHECK(stack.stages[2].id == "trunk");
    CHECK(stack.stages[3].id == "capsule");
    CHECK_FALSE(stack.stages[1].payload);
    CHECK(stack.stages[2].payload);
    CHECK(stack.stages[3].payload);
    CHECK(stack.stages[1].payload_mass_kg == 0.0);  // the payload is the stages now, not a mass
    CHECK(stack.engines.contains("superdraco"));
    CHECK(stack.engines.contains("merlin1d"));
    auto bare = f9;
    bare.stages.back().payload_mass_kg = 0.0;
    const vehicle::Vehicle launcher(bare, config_dir), stacked(stack, config_dir);
    CHECK(stacked.mass_properties().mass_kg ==
          Catch::Approx(launcher.mass_properties().mass_kg + 12500.0));
    // Stage ids must stay unique (engine groups are addressed "<stage>/<group>").
    CHECK_THROWS_AS(config::stack_payload(f9, f9), config::ConfigError);
}
```

Add `test_crew_dragon.cpp` to the `starsim_core_tests` source list in `core/tests/CMakeLists.txt` (next to
`test_rcs.cpp`).

- [ ] **Step 2: Build and run them to see them fail**

Run the core build. Expected: compile errors (`abort_only`, `behind_nose_cone`, `nose_cone`, `payload`,
`stack_payload` not members/declared).

- [ ] **Step 3: Add the config data**

`config/propellants.toml`, append:

```toml
[[propellant]]
id = "mmh_nto"
oxidizer = "N2O4"
fuel = "MMH"
mixture_ratio = 1.65              # [E] typical for Draco-class thrusters
oxidizer_density_kg_m3 = 1440.0   # [S: NTO at 20 C]
fuel_density_kg_m3 = 880.0        # [S: MMH at 20 C]
```

`config/engines/superdraco.toml`:

```toml
# SpaceX SuperDraco - Crew Dragon launch escape engine (data only in d2: an abort_only group, never lit)
[sources]
wiki_superdraco = "https://en.wikipedia.org/wiki/SuperDraco (commonly cited values)"

[engine]
id = "superdraco"
name = "SuperDraco"
cycle = "pressure-fed"
propellant = "mmh_nto"
mixture_ratio = 1.65                 # [E]
chamber_pressure_pa = 69e5           # [S:wiki_superdraco]

[performance]
reference_pressure_pa = 101325.0
thrust_ref_n = 71e3                  # [S:wiki_superdraco] sea level
isp_ref_s = 235.0                    # [S:wiki_superdraco]
nozzle_exit_diameter_m = 0.25        # [E]
max_ambient_pressure_pa = 101325.0

[throttle]
min = 0.2                            # [S:wiki_superdraco] 20-100 %
max = 1.00
rate_per_s = 5.0                     # [E]

[gimbal]
max_angle_deg = 0.0                  # fixed, canted in pairs
rate_deg_s = 0.0

[mass]
dry_kg = 0.0                         # in the capsule's dry mass
vehicle_side_hardware_kg = 0.0

[timing]
startup_s = 0.1                      # [S:wiki_superdraco] full thrust in 100 ms
shutdown_s = 0.1                     # [E]
```

`config/vehicles/crew_dragon.toml`:

```toml
# Crew Dragon (SpaceX Dragon 2, crew version) - capsule on its trunk. Flies as the payload of falcon9_block5
# (config::stack_payload via a mission's [payload] table) or alone in orbit. Milestone d2: orbital flight on the Dracos;
# aerodynamics, heat shield and parachutes follow in d3/d4.

[sources]
wiki_dragon2 = "https://en.wikipedia.org/wiki/SpaceX_Dragon_2 (commonly cited values)"

[vehicle]
id = "crew_dragon"
name = "Crew Dragon"
display_name = "Crew Capsule"
propellant = "mmh_nto"
stages_bottom_to_top = ["trunk", "capsule"]

[[stage]]
id = "trunk"
name = "Crew Dragon trunk"
length_m = 3.7                           # [D] 8.1 m with the trunk [S:wiki_dragon2] minus the 4.4 m capsule
diameter_m = 3.7                         # [E] on the 3.66 m second stage
dry_mass_kg = 1500.0                     # [E] solar arrays and radiators included, not modelled separately
dry_cg_station_m = 1.6                   # [E]
separation = "spring"

[[stage]]
id = "capsule"
name = "Crew Dragon capsule"
length_m = 4.4                           # [S:wiki_dragon2] with the nose cone
diameter_m = 4.0                         # [S:wiki_dragon2]
dry_mass_kg = 8438.0                     # [D] 12.5 t launch mass [S:wiki_dragon2] - 2,562 kg propellant - 1.5 t trunk;
                                         # includes the crew of four and their seats
dry_cg_station_m = 1.3                   # [E] low in the blunt body (the lift offset follows in d3)
separation = "spring"                    # [E] from the trunk (d3)
nose_cone = true                         # [S] hinged; closed for ascent and entry, open in orbit

[[stage.tank]]
id = "mmh_nto"
contents = "rcs"
mass_kg = 2562.0                         # [S:wiki_dragon2] shared by Dracos and SuperDracos
density_kg_m3 = 1162.0                   # [D] NTO/MMH at O/F 1.65 (1440 / 880 kg/m^3)
base_station_m = 0.4                     # [E] tanks in the service section above the heat shield

[stage.rcs]
type = "hypergolic"                      # Draco, MMH/NTO
isp_s = 300.0                            # [S:wiki_dragon2] vacuum
propellant = "mmh_nto"
minimum_on_time_s = 0.01                 # [E] 2 ticks: Draco valves pulse shorter than cold-gas ones

# [E] layout: no public drawing. Four Dracos under the nose cone push aft (plume forward: the deorbit / -x set, usable
# only with the cone open); four side pods with tangential +/- and a forward-pushing thruster give pure couples on all
# axes (balanced layout) and +x translation. Lateral translation (docking) is left to M9.
[[stage.rcs.pod]]
id = "nose"
station_m = 3.9                          # [E] forward bulkhead
radius_m = 0.6                           # [E]
azimuths_deg = [45.0, 135.0, 225.0, 315.0]
thrust_n = 400.0                         # [S:wiki_dragon2]
directions = ["aft"]
behind_nose_cone = true

[[stage.rcs.pod]]
id = "side"
station_m = 2.4                          # [E] on the 15 deg sidewall
radius_m = 1.5                           # [E]
azimuths_deg = [0.0, 90.0, 180.0, 270.0]
thrust_n = 400.0                         # [S:wiki_dragon2]
directions = ["tangential_plus", "tangential_minus", "forward"]

[[stage.engine_group]]
id = "superdraco"                        # launch escape: data only (abort_only: never lit by the session in d2)
engine = "superdraco"
count = 8                                # [S:wiki_dragon2] four pairs
ring_radius_m = 1.8                      # [E]
angles_deg = [40.0, 50.0, 130.0, 140.0, 220.0, 230.0, 310.0, 320.0]
gimbal = false
relight = true
abort_only = true

[stage.aero]
reference_diameter_m = 4.0
nose_length_m = 4.0                      # [E] the blunt cone as the stack's nose (capsule aero: d3)
```

- [ ] **Step 4: Load the new keys and write `stack_payload`**

In `vehicle_config.hpp`:
- `EngineGroupConfig`: add after `relight`
  `bool abort_only = false;  // launch escape (SuperDraco): data only, never lit by the session`.
  Keep the aggregate order for the existing brace init in the loader, so add it after `angles_rad` and set it in the
  loader by name.
- `RcsThrusterConfig`: add `bool behind_nose_cone = false;  // fires only with the stage's nose cone open`.
- `StageConfig`: add
  ```cpp
  bool nose_cone = false;  // a hinged nose cone (Crew Dragon): thrusters behind it fire only while it is open
  bool payload = false;    // a payload vehicle's stage (config::stack_payload): its RCS waits until it flies alone
  ```
- After `load_vehicle`, declare:

```cpp
// The launcher with a payload vehicle on top (a mission's [payload]): the payload's stages follow the launcher's, marked
// `payload`; the top launcher stage carries no payload mass. Throws ConfigError if a stage id occurs twice.
VehicleConfig stack_payload(VehicleConfig launcher, const VehicleConfig& payload);
```

In `vehicle_config.cpp`:
- `load_rcs`: in the pod loop read `const bool behind = p["behind_nose_cone"].value_or(false);` and pass it into each
  pushed thruster. For `[[stage.rcs.thruster]]` read `q["behind_nose_cone"].value_or(false)`.
- `load_stage`:
  - after `separation`, read `s.nose_cone = t["nose_cone"].value_or(false);`;
  - after loading the engine groups, read `abort_only` per group: `s.engine_groups.back().abort_only = g["abort_only"].value_or(false);`;
  - after `s.rcs` is loaded, fail when a thruster is `behind_nose_cone` and `!s.nose_cone`:
    `doc.fail("stage.rcs: behind_nose_cone needs nose_cone = true on the stage")`.
- Define `stack_payload`:

```cpp
VehicleConfig stack_payload(VehicleConfig launcher, const VehicleConfig& payload) {
    for (const auto& p : payload.stages) {
        for (const auto& l : launcher.stages) {
            if (l.id == p.id) throw ConfigError("stack_payload: stage id '" + p.id + "' occurs in both vehicles");
        }
    }
    launcher.stages.back().payload_mass_kg = 0.0;
    for (auto stage : payload.stages) {
        stage.payload = true;
        launcher.stages.push_back(std::move(stage));
    }
    for (const auto& [id, engine] : payload.engines) launcher.engines.emplace(id, engine);
    launcher.id += "+" + payload.id;
    launcher.name += " / " + payload.name;
    return launcher;
}
```

- [ ] **Step 5: Run the tests**

Run: `build/core/tests/starsim_core_tests.exe "[dragon][config]"`
Expected: 2 test cases pass. If the 12.5 t check fails by the engine dry masses, the loader adds no engine mass
(dry_mass_kg includes engines) — check `superdraco` `dry_kg = 0` is what made it pass, not a tolerance.

- [ ] **Step 6: Vehicle report**

Run: `python tools/vehicle_report.py crew_dragon`
Expected: masses 12.5 t, no Tsiolkovsky budget for the stages without main propellant. If the script divides by zero
on a stage without oxidizer/fuel tanks or treats the `abort_only` group as a main engine, patch it to skip those
(stages without main tanks report "no main propellant"; `abort_only` groups are listed as "abort only"). Then run
`python tools/vehicle_report.py falcon9_block5` and check that its output is unchanged
(`git stash`-free check: compare with the output before the patch).

- [ ] **Step 7: Document the keys** in `config/README.md`:
  - under "Aerodynamics and control surfaces (per stage)", add to the `[stage.rcs]` item:
    "`behind_nose_cone = true` (pod or thruster): fires only while the stage's nose cone is open (`[[stage]] nose_cone = true`, Crew Dragon)";
  - a new bullet: "`[[stage.engine_group]] abort_only = true`: launch escape engines (SuperDraco), loaded as data and
    never lit by the session (milestone d2)";
  - a new bullet under "Missions": "Payload vehicles: `config::stack_payload` stacks a vehicle (e.g. `crew_dragon`)
    on the launcher; its stages are `payload` stages (see Task 5's `[payload]` table)".

- [ ] **Step 8: Full suite and commit**

Run the full suite. Expected: all pass (345 before d2 plus the new cases).

```bash
git add config/vehicles/crew_dragon.toml config/engines/superdraco.toml config/propellants.toml config/README.md \
  core/include/starsim/config/vehicle_config.hpp core/src/config/vehicle_config.cpp core/tests/test_crew_dragon.cpp \
  core/tests/CMakeLists.txt tools/vehicle_report.py
git commit -m "feat(config): Crew Dragon vehicle and payload stacking"
```

---

### Task 2: Thruster availability and abort-only engines

**Files:**
- Modify:
  - `core/include/starsim/vehicle/vehicle.hpp` (`RcsThruster::behind_nose_cone`, `RcsThruster::blocked`,
    `Engine::abort_only`, nose cone state)
  - `core/src/vehicle/vehicle.cpp` (availability in `refresh_rcs_supply` → `refresh_rcs_availability`)
  - `core/src/sim/flight_session.cpp` (`ignite()`, `burn_configurations()` skip abort-only engines)
  - `core/include/starsim/sim/flight_session.hpp` (`set_nose_cone`)
- Test: `core/tests/test_crew_dragon.cpp`

**Interfaces:**
- Consumes: Task 1's config fields.
- Produces:
  - `bool vehicle::Engine::abort_only`
  - `bool vehicle::RcsThruster::behind_nose_cone`, `bool vehicle::RcsThruster::blocked` (true while the nose cone is
    closed or the stage is a payload still stacked)
  - `bool Vehicle::has_nose_cone() const`, `bool Vehicle::nose_cone_open() const`, `void Vehicle::set_nose_cone_open(bool)`
  - `void FlightSession::set_nose_cone(size_t body_index, bool open)`

- [ ] **Step 1: Write the failing tests** (append to `test_crew_dragon.cpp`; add the includes
  `starsim/environment/atmosphere.hpp`, `starsim/vehicle/flight_body.hpp`, `starsim/sim/flight_session.hpp`,
  `starsim/spice/spice_adapters.hpp`, `starsim/orbit/kepler.hpp`, `test_support.hpp`):

```cpp
namespace {

constexpr double tick_s = 0.005;
const time::Epoch t_deep = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);

class ZeroGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3&, time::Epoch) const override { return {}; }
};

class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};

struct DeepSpace {
    ZeroGravity gravity;
    environment::Us76Atmosphere atmosphere;
    FixedEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
};

std::unique_ptr<vehicle::FlightBody> deep_space_body(const config::VehicleConfig& cfg) {
    return std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, config_dir),
                                                 vehicle::RigidBodyState{{1e10, 0.0, 0.0}, {}, {}, {}});
}

double rcs_kg(const vehicle::Vehicle& v, const std::string& stage_id) {
    for (const auto& s : v.stages()) {
        if (s.config.id != stage_id) continue;
        double kg = 0.0;
        for (const auto& t : s.tanks) kg += t.config.contents == config::TankContents::Rcs ? t.mass_kg : 0.0;
        return kg;
    }
    return 0.0;
}

}  // namespace

TEST_CASE("A stacked payload's thrusters stay inhibited until it flies alone", "[dragon][rcs]") {
    DeepSpace space;
    const auto stack = config::stack_payload(config::load_vehicle(config_dir, "falcon9_block5"),
                                             config::load_vehicle(config_dir, "crew_dragon"));
    auto body = deep_space_body(stack);
    auto upper = body->separate(1);  // second stage + trunk + capsule, as after staging
    vehicle::VehicleCommand roll;
    roll.rcs_torque_fraction = {1.0, 0.0, 0.0};
    const double dragon_before = rcs_kg(upper->vehicle(), "capsule");
    for (int k = 0; k < 200; ++k) upper->step(space.env, t_deep + k * tick_s, tick_s, roll);
    CHECK(upper->state().angular_velocity_body_radps.x > 0.0);   // the second stage's nitrogen pods roll it
    CHECK(rcs_kg(upper->vehicle(), "capsule") == dragon_before);  // no Draco fired
    for (const auto& t : upper->vehicle().stages().back().rcs) CHECK(t.blocked);
    // Alone (trunk + capsule) its Dracos fire.
    auto dragon = upper->separate(1);
    for (const auto& t : dragon->vehicle().stages().back().rcs) CHECK(t.blocked == t.behind_nose_cone);
    for (int k = 0; k < 200; ++k) dragon->step(space.env, t_deep + k * tick_s, tick_s, roll);
    CHECK(rcs_kg(dragon->vehicle(), "capsule") < dragon_before);
}

TEST_CASE("The Dracos behind the nose cone fire only with the cone open", "[dragon][rcs]") {
    auto cfg = config::load_vehicle(config_dir, "crew_dragon");
    vehicle::Vehicle v(cfg, config_dir);
    REQUIRE(v.has_nose_cone());
    CHECK_FALSE(v.nose_cone_open());
    CHECK(v.rcs_force_capability({-1.0, 0.0, 0.0}).force_n == Catch::Approx(0.0).margin(1e-6));
    CHECK(v.rcs_force_capability({1.0, 0.0, 0.0}).force_n == Catch::Approx(1600.0).epsilon(0.01));
    v.set_nose_cone_open(true);
    CHECK(v.rcs_force_capability({-1.0, 0.0, 0.0}).force_n == Catch::Approx(1600.0).epsilon(0.01));
}

TEST_CASE("Dragon turns on pure couples: a full torque on any axis does not push it", "[dragon][rcs]") {
    // Balanced layout (pure couples on every axis): with rcs_keep_force a saturated turn leaves the capsule where it is.
    DeepSpace space;
    for (const Vec3 axis : {Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, -1.0}}) {
        CAPTURE(axis.x, axis.y, axis.z);
        auto body = deep_space_body(config::load_vehicle(config_dir, "crew_dragon"));
        body->vehicle().set_nose_cone_open(true);
        vehicle::VehicleCommand turn;
        turn.rcs_torque_fraction = axis;
        turn.rcs_keep_force = true;
        for (int k = 0; k < 400; ++k) body->step(space.env, t_deep + k * tick_s, tick_s, turn);
        const Vec3 w = body->state().angular_velocity_body_radps;
        CHECK(dot(w, axis) > 0.005);                              // it turns (> 0.3 deg/s after 2 s)
        CHECK(norm(body->state().velocity_gcrf_mps) < 2e-3);     // < 2 mm/s from leaks
    }
}

TEST_CASE("Abort-only engines are never lit by the session", "[dragon][sim]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const double gm = session.gm_earth_m3ps2();
    const orbit::KeplerianElements circular{6378137.0 + 200e3, 0.0, 51.6 * constants::deg_to_rad, 0.0, 0.0, 0.0};
    session.place_in_orbit("crew_dragon", orbit::state_from_elements(circular, gm), t_deep);
    session.ignite();
    session.advance_to(t_deep + 1.0);
    CHECK_FALSE(session.body(0).vehicle().any_engine_producing_thrust());
    CHECK(session.minimum_burn_dv_mps() == 0.0);  // no burn configuration on main engines
}
```

- [ ] **Step 2: Build and run them to see them fail**

Expected: compile errors (`blocked`, `behind_nose_cone`, `has_nose_cone`, `set_nose_cone_open`, `nose_cone_open`).

- [ ] **Step 3: Implement availability in `Vehicle`**

`vehicle.hpp`:
- `Engine`: add `bool abort_only = false;  // launch escape (data only): never lit by the session`.
- `RcsThruster`: add
  ```cpp
  bool behind_nose_cone = false;
  bool blocked = false;  // cannot fire now: behind a closed nose cone, or a payload stage still stacked
  ```
- In the public part next to `stop_rcs()`, add:

```cpp
    // Hinged nose cone of a stage (Crew Dragon): closed at construction; thrusters behind it are blocked while closed.
    bool has_nose_cone() const;
    bool nose_cone_open() const { return nose_cone_open_; }
    void set_nose_cone_open(bool open);
```

- Private: `bool nose_cone_open_ = false;`. Rename `refresh_rcs_supply()` to `refresh_rcs_availability()`, with the
  comment `// starves thrusters whose supply ran dry, blocks those that cannot fire now`.

`vehicle.cpp`:
- In the constructor, pass `t.behind_nose_cone` into the pushed `RcsThruster`. Use named assignment after
  `push_back` if the aggregate order makes brace init fragile: `stage.rcs.back().behind_nose_cone = t.behind_nose_cone;`.
- Set `engine.abort_only = group.abort_only;` where the engines are built.
- Replace `refresh_rcs_supply`:

```cpp
void Vehicle::refresh_rcs_availability() {
    bool changed = false;
    // A payload's stages (config::stack_payload) wait until the payload flies alone: its lowest stage is the vehicle's.
    const bool stacked = !stages_.empty() && !stages_.front().config.payload;
    for (auto& stage : stages_) {
        const bool main_dry = stage.propellant_kg(config::TankContents::Oxidizer) <= 0.0 ||
                              stage.propellant_kg(config::TankContents::Fuel) <= 0.0;
        for (auto& t : stage.rcs) {
            const bool starved = t.tank >= 0 ? stage.tanks[static_cast<size_t>(t.tank)].mass_kg <= 0.0 : main_dry;
            const bool blocked = (stacked && stage.config.payload) || (t.behind_nose_cone && !nose_cone_open_);
            if (starved != t.starved || blocked != t.blocked) {
                t.starved = starved;
                t.blocked = blocked;
                changed = true;
            }
        }
    }
    if (changed) rcs_allocator_.reset();
}
```

- Call `refresh_rcs_availability()` at the end of the constructor too (after `rebuild_aero()`). Otherwise
  `rcs_force_capability` before the first tick would count blocked thrusters.
- In `rcs_allocator()`, use thrust `t.starved || t.blocked ? 0.0 : t.thrust_n`. In `update_rcs`, set the output to
  `firing[i] && !t.starved && !t.blocked ? 1.0 : 0.0`.
- `Vehicle::separate`: call `refresh_rcs_availability()` on both parts after the `rcs_allocator_.reset()` lines (the
  payload becomes free when it is split off). Also copy `nose_cone_open_` to the upper part:
  `upper->nose_cone_open_ = nose_cone_open_;`.
- Add:

```cpp
bool Vehicle::has_nose_cone() const {
    return std::any_of(stages_.begin(), stages_.end(), [](const Stage& s) { return s.config.nose_cone; });
}

void Vehicle::set_nose_cone_open(bool open) {
    if (!has_nose_cone()) return;
    nose_cone_open_ = open;
    refresh_rcs_availability();
}
```

- Replace the remaining callers of `refresh_rcs_supply()` (`consume_propellant`, `update_rcs`).

- [ ] **Step 4: Keep abort-only engines out of the session**

`flight_session.cpp`:
- `ignite()`: light only the engines with `!e.abort_only`. The "lowest stage" is the lowest one that has a
  non-abort engine:

```cpp
void FlightSession::ignite() {
    if (bodies_.empty()) return;
    auto& flying = bodies_[controlled_];
    for (const auto& stage : flying.body->vehicle().stages()) {
        bool lit = false;
        for (const auto& e : stage.engines) {
            if (e.abort_only) continue;  // launch escape: never lit here
            flying.running_groups[e.group_id] = true;
            lit = true;
        }
        if (lit) break;  // lowest stage with engines only
    }
}
```

- `burn_configurations()`: inside the stage loop, collect `groups` only from `!e.abort_only` engines, and
  `continue` when `groups` is empty (instead of `if (stage.engines.empty()) continue;`). In `make`, skip
  `e.abort_only` engines with the starved ones.
- `PreLaunch` in `run_autopilot()`: `all_running` ignores `abort_only` engines (`|| e.abort_only`). Only the
  launcher's first stage is checked, but keep it explicit.
- Add `void FlightSession::set_nose_cone(size_t body_index, bool open)` (header comment:
  `// Crew Dragon's nose cone (thrusters behind it fire only while it is open); no effect without one.`), which calls
  `bodies_.at(body_index).body->vehicle().set_nose_cone_open(open);`.

- [ ] **Step 5: Run the tests**

Run: `build/core/tests/starsim_core_tests.exe "[dragon]"`. Expected: all pass.
Run: `build/core/tests/starsim_core_tests.exe "[rcs]"`. Expected: all pass (unchanged behaviour for the existing
vehicles: nothing is blocked there).

- [ ] **Step 6: Full suite and commit**

Run the full suite. Expected: all pass.

```bash
git add core/include/starsim/vehicle/vehicle.hpp core/src/vehicle/vehicle.cpp core/include/starsim/sim/flight_session.hpp \
  core/src/sim/flight_session.cpp core/tests/test_crew_dragon.cpp
git commit -m "feat(vehicle): payload and nose-cone thrusters wait until they can fire; abort-only engines stay off"
```

---

### Task 3: Orbital attitude modes

**Files:**
- Create: `core/include/starsim/gnc/orbital_attitude.hpp`, `core/src/gnc/orbital_attitude.cpp`
- Modify:
  - `core/CMakeLists.txt` or the core source list (add `src/gnc/orbital_attitude.cpp` where the other `gnc` sources are listed)
  - `core/include/starsim/gnc/attitude_controller.hpp` / `.cpp` (hold deadband)
  - `core/include/starsim/sim/flight_session.hpp` / `.cpp` (mode, command, coasting)
- Test: `core/tests/test_crew_dragon.cpp`

**Interfaces:**
- Produces:

```cpp
namespace starsim::gnc {
enum class AttitudeModeKind { Free, Inertial, Lvlh, Direction, Sun };
struct AttitudeMode {
    AttitudeModeKind kind = AttitudeModeKind::Free;
    math::Quat attitude_gcrf;   // Inertial: the body attitude held
    math::Quat body_in_lvlh;    // Lvlh: body attitude in the LVLH frame (identity: nose along the velocity, +z nadir)
    math::Vec3 direction_gcrf;  // Direction: where body +x points (unit); Sun: filled by the session each tick
};
struct AttitudeTarget {
    math::Vec3 axis_gcrf;            // body +x
    math::Vec3 roll_reference_gcrf;  // body +y (projected by the controller)
    math::Vec3 rate_gcrf_radps;      // rotation of the target (LVLH: the orbit rate)
};
math::Quat lvlh_attitude_gcrf(const math::StateVector& state);
AttitudeTarget attitude_target(const AttitudeMode& mode, const math::StateVector& state);
AttitudeMode inertial_hold(const math::Quat& attitude_gcrf);
AttitudeMode lvlh_hold(const math::Quat& body_in_lvlh = {});
}
```

- `AttitudeController::Gains::hold_deadband_rad`, `hold_rate_deadband_radps` (default 0: off).
- `void FlightSession::set_attitude_mode(const gnc::AttitudeMode&)`, `const gnc::AttitudeMode& FlightSession::attitude_mode() const`.
- `gnc::AttitudeController FlightSession::orbit_controller_` with the deadbands.

- [ ] **Step 1: Write the failing tests** (append to `test_crew_dragon.cpp`; include `starsim/gnc/orbital_attitude.hpp`):

```cpp
TEST_CASE("The LVLH frame: x along the velocity, z to nadir, rotating at the orbit rate", "[dragon][attitude]") {
    const double gm = 3.986004418e14;
    const orbit::KeplerianElements circular{6578137.0, 0.0, 0.9, 0.3, 0.0, 1.0};
    const auto s = orbit::state_from_elements(circular, gm);
    const auto q = gnc::lvlh_attitude_gcrf(s);
    CHECK(norm(q.rotate({1.0, 0.0, 0.0}) - math::normalized(s.velocity_mps)) < 1e-9);  // circular: v is horizontal
    CHECK(norm(q.rotate({0.0, 0.0, 1.0}) + math::normalized(s.position_m)) < 1e-12);
    const auto target = gnc::attitude_target(gnc::lvlh_hold(), s);
    const double n = std::sqrt(gm / std::pow(circular.semi_major_axis_m, 3));
    CHECK(norm(target.rate_gcrf_radps) == Catch::Approx(n).epsilon(1e-9));
    CHECK(dot(target.rate_gcrf_radps, cross(s.position_m, s.velocity_mps)) > 0.0);
    // "Trunk forward": the body turned 180 deg about z in LVLH points its nose against the velocity.
    const auto back = gnc::attitude_target(gnc::lvlh_hold(math::Quat::from_axis_angle({0.0, 0.0, 1.0}, constants::pi)), s);
    CHECK(dot(back.axis_gcrf, math::normalized(s.velocity_mps)) == Catch::Approx(-1.0));
}

namespace {

struct DragonInOrbit {
    const spice::SpiceEphemeris* ephemeris;
    const frames::BodyOrientation* earth;
    sim::FlightSession session;
    time::Epoch t0;
    double gm;
    explicit DragonInOrbit(const spice::SpiceEphemeris& eph, const frames::BodyOrientation& orientation)
        : ephemeris(&eph), earth(&orientation), session(eph, orientation, STARSIM_CONFIG_DIR), t0(t_deep),
          gm(session.gm_earth_m3ps2()) {
        const orbit::KeplerianElements circular{6378137.0 + 200e3, 0.0, 51.6 * constants::deg_to_rad, 0.0, 0.0, 0.0};
        session.place_in_orbit("crew_dragon", orbit::state_from_elements(circular, gm), t0);
        session.set_nose_cone(0, true);
    }
};

double angle_between(const Vec3& a, const Vec3& b) {
    return std::acos(std::clamp(dot(math::normalized(a), math::normalized(b)), -1.0, 1.0));
}

}  // namespace

TEST_CASE("Dragon captures an inertial attitude from a tumble and holds it on pulses", "[dragon][attitude]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    DragonInOrbit o(ephemeris, earth);
    auto& s = o.session;
    const auto hold = s.body(0).state().attitude;
    s.set_attitude_mode(gnc::inertial_hold(hold));
    auto st = s.body(0).state();
    st.angular_velocity_body_radps = {0.5 * constants::deg_to_rad, -1.0 * constants::deg_to_rad, 0.7 * constants::deg_to_rad};
    s.apply_angular_rate(0, st.angular_velocity_body_radps);
    const double kg0 = s.body(0).vehicle().rcs_propellant_kg();
    double worst = 0.0;
    for (double t = 1.0; t <= 1800.0; t += 1.0) {
        s.advance_to(o.t0 + t, 1000);
        if (t > 120.0) {
            worst = std::max(worst, angle_between(s.body(0).state().attitude.rotate({1.0, 0.0, 0.0}), hold.rotate({1.0, 0.0, 0.0})));
        }
    }
    const double used = kg0 - s.body(0).vehicle().rcs_propellant_kg();
    std::printf("[validation] Dragon inertial hold 30 min after a 1.3 deg/s tumble: worst %.2f deg after 2 min, %.2f kg\n",
                worst / constants::deg_to_rad, used);
    CHECK(worst < 1.0 * constants::deg_to_rad);
    CHECK(used > 0.0);
    CHECK(used < 5.0);  // [E] ruling 4: ~1 kg/h hold + the capture
}

TEST_CASE("Dragon holds LVLH with its trunk forward for a quarter orbit", "[dragon][attitude]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    DragonInOrbit o(ephemeris, earth);
    auto& s = o.session;
    const auto mode = gnc::lvlh_hold(math::Quat::from_axis_angle({0.0, 0.0, 1.0}, constants::pi));
    s.set_attitude_mode(mode);
    double worst = 0.0;
    for (double t = 1.0; t <= 1330.0; t += 1.0) {
        s.advance_to(o.t0 + t, 1000);
        const auto& b = s.body(0).state();
        if (t > 300.0) {  // after the 180 deg turn
            const auto target = gnc::attitude_target(mode, {b.position_gcrf_m, b.velocity_gcrf_mps});
            worst = std::max(worst, angle_between(b.attitude.rotate({1.0, 0.0, 0.0}), target.axis_gcrf));
        }
    }
    std::printf("[validation] Dragon LVLH trunk-forward hold: worst %.2f deg\n", worst / constants::deg_to_rad);
    CHECK(worst < 1.5 * constants::deg_to_rad);
}

TEST_CASE("Dragon points its nose at the Sun", "[dragon][attitude]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    DragonInOrbit o(ephemeris, earth);
    auto& s = o.session;
    gnc::AttitudeMode sun;
    sun.kind = gnc::AttitudeModeKind::Sun;
    s.set_attitude_mode(sun);
    s.advance_to(o.t0 + 600.0, 200000);
    const auto& b = s.body(0).state();
    const Vec3 to_sun = ephemeris.state_gcrf(ephemeris::Body::Sun, s.epoch()).position_m - b.position_gcrf_m;
    CHECK(angle_between(b.attitude.rotate({1.0, 0.0, 0.0}), to_sun) < 1.0 * constants::deg_to_rad);
}
```

(`FlightSession::apply_angular_rate` is new in Step 5, next to `apply_velocity_error`.)

- [ ] **Step 2: Build and run them to see them fail**

Expected: compile errors (`gnc/orbital_attitude.hpp` missing, `set_attitude_mode`, `apply_angular_rate`).

- [ ] **Step 3: Implement `gnc/orbital_attitude`**

`core/include/starsim/gnc/orbital_attitude.hpp`:

```cpp
#pragma once

#include "starsim/math/quat.hpp"
#include "starsim/math/state_vector.hpp"

namespace starsim::gnc {

// Attitude modes of a vehicle in orbit, flown on RCS by the session's orbit controller (d2: Crew Dragon).
// Free: no attitude control (the vehicle coasts). Inertial: hold a body attitude fixed in GCRF. Lvlh: hold an attitude
// fixed in the local-vertical/local-horizontal frame. Direction: body +x along a fixed GCRF direction (roll: body +y
// towards the orbit normal). Sun: body +x at the Sun (the session fills direction_gcrf each tick).
enum class AttitudeModeKind { Free, Inertial, Lvlh, Direction, Sun };

struct AttitudeMode {
    AttitudeModeKind kind = AttitudeModeKind::Free;
    math::Quat attitude_gcrf;   // Inertial
    math::Quat body_in_lvlh;    // Lvlh (identity: nose along the velocity, +z to nadir)
    math::Vec3 direction_gcrf;  // Direction, Sun
};

// What the attitude controller steers to: body +x along axis_gcrf, body +y towards roll_reference_gcrf, while the
// target turns at rate_gcrf_radps (fed forward).
struct AttitudeTarget {
    math::Vec3 axis_gcrf;
    math::Vec3 roll_reference_gcrf;
    math::Vec3 rate_gcrf_radps;
};

// LVLH frame (CCSDS / ISS convention) as an attitude: z to nadir (-r), y against the orbit normal (-r x v), x = y x z
// (along the velocity on a circular orbit).
math::Quat lvlh_attitude_gcrf(const math::StateVector& state);
AttitudeTarget attitude_target(const AttitudeMode& mode, const math::StateVector& state);
AttitudeMode inertial_hold(const math::Quat& attitude_gcrf);
AttitudeMode lvlh_hold(const math::Quat& body_in_lvlh = {});

}  // namespace starsim::gnc
```

`core/src/gnc/orbital_attitude.cpp`:

```cpp
#include "starsim/gnc/orbital_attitude.hpp"

#include "starsim/math/mat3.hpp"

namespace starsim::gnc {

using math::Quat;
using math::Vec3;

namespace {

Quat from_columns(const Vec3& x, const Vec3& y, const Vec3& z) {
    math::Mat3 m;
    const Vec3 columns[3] = {x, y, z};
    for (int c = 0; c < 3; ++c) {
        m(0, c) = columns[c].x;
        m(1, c) = columns[c].y;
        m(2, c) = columns[c].z;
    }
    return Quat::from_matrix(m);
}

}  // namespace

Quat lvlh_attitude_gcrf(const math::StateVector& state) {
    const Vec3 z = -math::normalized(state.position_m);
    const Vec3 y = -math::normalized(cross(state.position_m, state.velocity_mps));
    return from_columns(cross(y, z), y, z);
}

AttitudeTarget attitude_target(const AttitudeMode& mode, const math::StateVector& state) {
    const Vec3 x{1.0, 0.0, 0.0}, y{0.0, 1.0, 0.0};
    switch (mode.kind) {
        case AttitudeModeKind::Inertial:
            return {mode.attitude_gcrf.rotate(x), mode.attitude_gcrf.rotate(y), {}};
        case AttitudeModeKind::Lvlh: {
            const Quat q = lvlh_attitude_gcrf(state) * mode.body_in_lvlh;
            const double r2 = dot(state.position_m, state.position_m);
            return {q.rotate(x), q.rotate(y), cross(state.position_m, state.velocity_mps) / r2};
        }
        case AttitudeModeKind::Direction:
        case AttitudeModeKind::Sun:
            return {math::normalized(mode.direction_gcrf), math::normalized(cross(state.position_m, state.velocity_mps)), {}};
        case AttitudeModeKind::Free:
            break;
    }
    return {};
}

AttitudeMode inertial_hold(const Quat& attitude_gcrf) {
    AttitudeMode m;
    m.kind = AttitudeModeKind::Inertial;
    m.attitude_gcrf = attitude_gcrf;
    return m;
}

AttitudeMode lvlh_hold(const Quat& body_in_lvlh) {
    AttitudeMode m;
    m.kind = AttitudeModeKind::Lvlh;
    m.body_in_lvlh = body_in_lvlh;
    return m;
}

}  // namespace starsim::gnc
```

(`cross(r, v) / r^2` is the rotation rate of the radius vector. On a circular orbit it equals the orbit rate, and
LVLH turns about +h, which is the test's check.)

Add `src/gnc/orbital_attitude.cpp` to the core library's source list. Find it with
`grep -n "entry_attitude.cpp" core/CMakeLists.txt` and insert alphabetically.

- [ ] **Step 4: Add the hold deadband to the controller**

`attitude_controller.hpp`, in `Gains` after `rcs_max_rate_radps`:

```cpp
        // Attitude hold on pulsed RCS (orbit): per axis no torque while the error is within hold_deadband_rad and the
        // rate error within hold_rate_deadband_radps - the pulses then form a slow limit cycle instead of chattering
        // (0: off).
        double hold_deadband_rad = 0.0;
        double hold_rate_deadband_radps = 0.0;
```

`attitude_controller.cpp`, in `steer()`, after `torque` is computed and before `allocate(...)`. Replace the `const`
torque with a mutable one:

```cpp
    Vec3 torque{inertia.x * kd * (rate.x - w.x), inertia.y * (kd * (rate.y - w.y) + ff_acceleration.y),
                inertia.z * (kd * (rate.z - w.z) + ff_acceleration.z)};
    if (gains_.hold_deadband_rad > 0.0) {
        const double errors[3] = {roll_error, rotation.y, rotation.z};
        const double rate_errors[3] = {w.x, w.y - ff_rate.y, w.z - ff_rate.z};
        double* components[3] = {&torque.x, &torque.y, &torque.z};
        for (int i = 0; i < 3; ++i) {
            if (std::abs(errors[i]) < gains_.hold_deadband_rad && std::abs(rate_errors[i]) < gains_.hold_rate_deadband_radps) {
                *components[i] = 0.0;
            }
        }
    }
```

(Roll's feedforward is not used by `steer` (roll follows the reference). In LVLH the roll-axis rate component is
small for the trunk-forward attitude, and the roll error loop corrects it. If the LVLH test shows a roll limit cycle
costing propellant, feed `feedforward->rate_body_radps.x` into the roll rate error only, and ledger it.)

- [ ] **Step 5: Wire the mode into the session**

`flight_session.hpp`:
- `#include "starsim/gnc/orbital_attitude.hpp"`.
- Public:

```cpp
    // Attitude mode of the controlled body while no autopilot phase steers it (state Off / Orbit; d2). Any mode but Free
    // keeps the session ticking (pulsed RCS hold); Free lets it coast. Pilot input is ignored while a mode is held.
    void set_attitude_mode(const gnc::AttitudeMode& mode) { attitude_mode_ = mode; }
    const gnc::AttitudeMode& attitude_mode() const { return attitude_mode_; }
    // Scenarios and tests: sets a body's angular rate (body axes).
    void apply_angular_rate(size_t body_index, const math::Vec3& rate_body_radps);
```

- Private: `gnc::AttitudeMode attitude_mode_;` and

```cpp
    // Orbit attitude modes on pulsed RCS: slower than the ascent controller, with a hold deadband [E] 0.5 deg /
    // 0.05 deg/s so the hold settles into a slow limit cycle (d2, ruling 5).
    gnc::AttitudeController orbit_controller_{gnc::AttitudeController::Gains{
        0.3, 0.9, false, false, 0.0, 1.0 * constants::deg_to_rad, 0.5 * constants::deg_to_rad,
        0.05 * constants::deg_to_rad}};
```

  (`constants` needs `starsim/math/constants.hpp` in the header; it is already used by `return_coast_controller_`.)

`flight_session.cpp`:
- `reset_mission_state()` and `place_in_orbit()`: reset `attitude_mode_ = {};`.
- `apply_angular_rate`:

```cpp
void FlightSession::apply_angular_rate(size_t body_index, const math::Vec3& rate_body_radps) {
    auto& body = *bodies_.at(body_index).body;
    auto state = body.state();
    state.angular_velocity_body_radps = rate_body_radps;
    body.set_state(state);
}
```

- `command_for()`: after the `guided || Burn` block returns, and before the pilot-input part, add:

```cpp
    if (attitude_mode_.kind != gnc::AttitudeModeKind::Free &&
        (autopilot_.state == AutopilotState::Off || autopilot_.state == AutopilotState::Orbit)) {
        const auto& s = flying.body->state();
        gnc::AttitudeMode mode = attitude_mode_;
        if (mode.kind == gnc::AttitudeModeKind::Sun && env_.ephemeris) {
            mode.direction_gcrf = env_.ephemeris->state_gcrf(ephemeris::Body::Sun, epoch_).position_m - s.position_gcrf_m;
        }
        const auto target = gnc::attitude_target(mode, {s.position_gcrf_m, s.velocity_gcrf_mps});
        const gnc::AttitudeController::Feedforward feedforward{s.attitude.inverse_rotate(target.rate_gcrf_radps), {}};
        orbit_controller_.steer(*flying.body, ambient_pressure_pa, target.axis_gcrf, target.roll_reference_gcrf,
                                &feedforward)
            .apply_to(command);
        command.rcs_keep_force = true;  // an orbit: turns must not push the vehicle
        return command;
    }
```

  `env_.ephemeris` is the `FlightEnvironment` pointer. Check how `state_gcrf` returns the position
  (`StateVector::position_m`) and adapt the member name if it differs.
- `coast_allowance_s()`: after the `guided` check, add
  `if (attitude_mode_.kind != gnc::AttitudeModeKind::Free && (autopilot_.state == AutopilotState::Off || autopilot_.state == AutopilotState::Orbit)) return 0.0;`
  (ruling 6).

Gains aggregate order: `{natural_frequency, damping, grid_fins, flaps, actuator_delay, rcs_max_rate, hold_deadband,
hold_rate_deadband}`. The rate limit of 1 deg/s [E] keeps capture slews cheap (spec: propellant use).

- [ ] **Step 6: Run the tests**

Run: `build/core/tests/starsim_core_tests.exe "[dragon][attitude]"`
Expected: 4 pass. Print lines show the worst errors and the propellant.
- If the inertial hold uses more than 5 kg, first measure the limit cycle (print pulses per minute per axis with
  `STARSIM_VERBOSE`). Then widen the rate deadband to 0.1 deg/s.
- If it still exceeds the bound, record the figure and the ruling in the ledger. Do not loosen the test past 10 kg
  without the user.

- [ ] **Step 7: Full suite and commit**

Run the full suite. Expected: all pass. The new `Gains` fields default to 0, so the existing controllers are
unchanged.

```bash
git add core/include/starsim/gnc/orbital_attitude.hpp core/src/gnc/orbital_attitude.cpp core/CMakeLists.txt \
  core/include/starsim/gnc/attitude_controller.hpp core/src/gnc/attitude_controller.cpp \
  core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/tests/test_crew_dragon.cpp
git commit -m "feat(gnc): orbital attitude modes (inertial, LVLH, direction, Sun) on pulsed RCS with a hold deadband"
```

---

### Task 4: Draco translation burns

**Files:**
- Modify: `core/src/sim/flight_session.cpp` (`start_burn`, `run_autopilot` Burn, `command_for` hull turn),
  `core/include/starsim/sim/flight_session.hpp` (doc comment of `execute_burn`)
- Test: `core/tests/test_crew_dragon.cpp`

**Interfaces:**
- Consumes: Task 2 (`set_nose_cone`, availability), Task 3 (`DragonInOrbit`, `angle_between` test helpers).
- Produces: `BurnConfiguration::thrust_axis_body` holds the RCS burn axis. It is +x when the vehicle has thrust along
  +x, else the body axis (±x, ±y, ±z) with the largest force capability.

- [ ] **Step 1: Write the failing tests** (append):

```cpp
namespace {

// Flies an RCS burn of `dv` m/s prograde from a 200 km orbit (node 300 s ahead); returns the delta-v error vector
// (achieved - planned, m/s) in GCRF and the propellant used.
struct BurnResult {
    Vec3 error_mps;
    double propellant_kg = 0.0;
    Vec3 axis_body;
};

BurnResult fly_prograde_burn(DragonInOrbit& o, double dv, bool drop_forward_thrusters = false) {
    auto& s = o.session;
    if (drop_forward_thrusters) {
        // The side pods' forward thrusters lost: the strongest axis is -x (the four Dracos under the nose cone).
        auto cfg = config::load_vehicle(config_dir, "crew_dragon");
        auto& thrusters = cfg.stages[1].rcs.thrusters;
        std::erase_if(thrusters, [](const config::RcsThrusterConfig& t) { return t.direction.x > 0.5; });
        const auto b = s.body(0).state();
        s.place_body(cfg, b, o.t0);
        s.set_nose_cone(0, true);
    }
    const auto& b0 = s.body(0).state();
    const math::StateVector start{b0.position_gcrf_m, b0.velocity_gcrf_mps};
    const double node_s = 300.0;
    // Two-body reference: the session's gravity has J2, Moon and Sun - compare against an identical session without the
    // burn instead (the difference of the two final velocities is the burn).
    const auto at_node = orbit::propagate_two_body(start, o.gm, node_s);
    const Vec3 planned = math::normalized(at_node.velocity_mps) * dv;
    const double kg0 = s.body(0).vehicle().rcs_propellant_kg();
    REQUIRE(s.execute_burn({o.t0 + node_s, planned}));
    BurnResult r;
    r.axis_body = s.burn_configuration().thrust_axis_body;
    REQUIRE(s.burn_configuration().rcs);
    for (double t = 1.0; t < 1200.0 && s.autopilot().state == sim::AutopilotState::Burn; t += 1.0) {
        s.advance_to(o.t0 + t, 1000);
    }
    REQUIRE(s.autopilot().state == sim::AutopilotState::Off);
    r.propellant_kg = kg0 - s.body(0).vehicle().rcs_propellant_kg();
    // Reference: the same orbit without the burn, coasted to the same epoch.
    sim::FlightSession reference(*o.ephemeris, *o.earth, STARSIM_CONFIG_DIR);
    reference.place_in_orbit("crew_dragon", start, o.t0);
    reference.advance_to(s.epoch(), 1000000);
    r.error_mps = (s.body(0).state().velocity_gcrf_mps - reference.body(0).state().velocity_gcrf_mps) - planned;
    return r;
}

}  // namespace

TEST_CASE("Dragon flies 1 m/s and 10 m/s burns on its Dracos", "[dragon][burn][validation]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    for (const double dv : {1.0, 10.0}) {
        CAPTURE(dv);
        DragonInOrbit o(ephemeris, earth);
        const auto r = fly_prograde_burn(o, dv);
        std::printf("[validation] Dragon %.0f m/s Draco burn: error %.3f m/s, %.2f kg, axis %.0f/%.0f/%.0f\n", dv,
                    norm(r.error_mps), r.propellant_kg, r.axis_body.x, r.axis_body.y, r.axis_body.z);
        CHECK(r.axis_body.x == Catch::Approx(1.0));   // the side pods' forward thrusters (+x)
        CHECK(norm(r.error_mps) < 0.01 + 0.01 * dv);   // [E] 1 % + the pulse quantum and the reference's propagation
        const double ideal_kg = 12500.0 * (1.0 - std::exp(-dv / (300.0 * constants::standard_gravity_mps2)));
        CHECK(r.propellant_kg > ideal_kg);
        CHECK(r.propellant_kg < 1.3 * ideal_kg + 0.5);  // turning and holding during the burn
    }
}

TEST_CASE("An RCS burn along -x turns the hull round and keeps the delta-v", "[dragon][burn]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    DragonInOrbit o(ephemeris, earth);
    const auto r = fly_prograde_burn(o, 2.0, true);
    CHECK(r.axis_body.x == Catch::Approx(-1.0));
    CHECK(norm(r.error_mps) < 0.05);
}
```

`DragonInOrbit` (Task 3) keeps the ephemeris and orientation pointers for the reference session. Include
`starsim/orbit/kepler.hpp` (`propagate_two_body`; check its header with `grep -rn "propagate_two_body" core/include`).

- [ ] **Step 2: Build and run them to see them fail**

Expected: the 1/10 m/s test may already pass (+x path exists). The -x test fails: `axis_body.x == 1.0` (+x is
always chosen), or the burn is refused for lack of +x thrust (`execute_burn` false). Record which.

- [ ] **Step 3: Choose the RCS axis and turn the hull**

`flight_session.cpp`, `start_burn`, replace the RCS block:

```cpp
    if (chosen == nullptr && !bodies_.empty()) {
        // Below the main-engine minimum impulse (or no main engines): RCS translation along the body axis with the most
        // thrust - +x when the vehicle has thrusters along it (ullage thrusters), else the strongest of -x, +-y, +-z (Crew
        // Dragon with the forward Dracos only) - if the burn takes less than an hour.
        const auto& vehicle = bodies_[controlled_].body->vehicle();
        rcs.rcs = true;
        auto capability = vehicle.rcs_force_capability({1.0, 0.0, 0.0});
        rcs.thrust_axis_body = {1.0, 0.0, 0.0};
        if (!(capability.force_n > 0.0)) {
            for (const math::Vec3 axis : {math::Vec3{-1.0, 0.0, 0.0}, math::Vec3{0.0, 1.0, 0.0}, math::Vec3{0.0, -1.0, 0.0},
                                          math::Vec3{0.0, 0.0, 1.0}, math::Vec3{0.0, 0.0, -1.0}}) {
                const auto c = vehicle.rcs_force_capability(axis);
                if (c.force_n > capability.force_n) {
                    capability = c;
                    rcs.thrust_axis_body = axis;
                }
            }
        }
        rcs.thrust_n = capability.force_n;
        rcs.mass_flow_kgps = capability.mass_flow_kgps;
        if (rcs.thrust_n > 0.0 && dv / (rcs.thrust_n / vehicle.mass_properties().mass_kg) < 3600.0) {
            chosen = &rcs;
        }
    }
```

`run_autopilot`, `case AutopilotState::Burn`: the `thrust_axis` for RCS becomes `c.thrust_axis_body` (it only
enters the executor's engine-thrust sensing, so this is for consistency):

```cpp
            const math::Vec3 thrust_axis =
                c.rcs ? c.thrust_axis_body : gnc::thrust_axis_body(vehicle, vehicle.mass_properties().cg_station_m, c.groups);
```

`command_for`, the steering block:
- `thrust_axis` for `burn_config_.rcs` becomes `burn_config_.thrust_axis_body` (guided ascent stays +x).
- In the tilted-axis branch, replace the `to_x` line with a version that handles the anti-parallel axis:

```cpp
            const math::Vec3 x{1.0, 0.0, 0.0};
            const math::Vec3 turn_axis = cross(thrust_axis, x);
            // Anti-parallel (an RCS burn along -x): half a turn about body z.
            const math::Quat to_x = norm(turn_axis) > 1e-9
                                        ? math::Quat::from_axis_angle(turn_axis, std::acos(std::clamp(thrust_axis.x, -1.0, 1.0)))
                                        : math::Quat::from_axis_angle({0.0, 0.0, 1.0}, constants::pi);
```

  A lateral axis (±y, ±z) goes through the general branch. `hull.rotate(thrust_axis) = x0` holds by construction.

Update the header comment of `execute_burn(const gnc::BurnNode&)`: "... fly on RCS translation along the body axis with
the most thrust (+x when the vehicle has thrusters along it; else the strongest of -x, +-y, +-z - Crew Dragon) ...".

- [ ] **Step 4: Run the tests**

Run: `build/core/tests/starsim_core_tests.exe "[dragon][burn]"`
Expected: both pass. The print lines show the errors and the propellant.
- If the 1 m/s error exceeds 0.02 m/s, check first whether the reference session differs (attitude pulses before the
  burn are attitude-only under `rcs_keep_force`, but a starting turn on a balanced layout leaks ~mm/s).
- If the error is the executor's cutoff quantum (4 thrusters × 400 N × 5 ms / 12.5 t = 0.6 mm/s per tick), it is
  well within the bound.

- [ ] **Step 5: Full suite and commit**

Run the full suite. Expected: all pass. The ship and Falcon 9 keep +x: they have +x thrusters.

```bash
git add core/src/sim/flight_session.cpp core/include/starsim/sim/flight_session.hpp core/tests/test_crew_dragon.cpp
git commit -m "feat(sim): RCS burns fly along the body axis with the most thrust (Draco translation burns)"
```

---

### Task 5: The crew mission: payload separation, nose cone, orbit

**Files:**
- Create: `config/missions/falcon9_crew_dragon.toml`
- Modify:
  - `core/include/starsim/config/mission_config.hpp` / `core/src/config/mission_config.cpp` (`MissionPayload`)
  - `core/src/sim/ascent_mission.cpp` (stack the payload, set the separation)
  - `core/include/starsim/sim/flight_session.hpp` / `.cpp` (`PayloadSeparation`, `AutopilotState::PayloadSeparation`,
    `separate_payload()`, nose-cone timer)
  - `bindings/src/flight_sim.cpp` only if a `switch` over `AutopilotState` fails to compile there (it uses
    comparisons today: no change expected)
  - `CLAUDE.md`, `config/README.md`
- Test: `core/tests/test_crew_dragon.cpp`

**Interfaces:**
- Consumes: Task 1 `stack_payload`, Task 2 `set_nose_cone`, Task 3 `set_attitude_mode` / `inertial_hold`.
- Produces:

```cpp
// config
struct MissionPayload {
    std::string vehicle_id;
    double separation_delay_s = 30.0;   // after the upper stage's cutoff
    double separation_dv_mps = 0.3;     // spring push (relative speed)
    double nose_cone_open_s = 600.0;    // after the separation; < 0: stays closed
};
// AscentMission::payload: std::optional<MissionPayload> (exclusive with payload_t)

// sim
struct PayloadSeparation {
    std::string first_stage_id;  // the payload's lowest stage (e.g. "trunk")
    double delay_s = 30.0;
    double dv_mps = 0.3;
    double nose_cone_open_s = 600.0;
};
void FlightSession::set_payload_separation(std::optional<PayloadSeparation>);
bool FlightSession::separate_payload();  // false if the controlled body carries no such stage above its lowest
// AutopilotState::PayloadSeparation, to_string "Nutzlasttrennung"
```

- [ ] **Step 1: Write the mission file** `config/missions/falcon9_crew_dragon.toml`:

```toml
# Falcon 9 from SLC-40 with a Crew Dragon (a real vehicle on top of the stack, milestone d2) and the first stage landing
# on LZ-40 (as Crew-9: B1085 landed at LZ-1 [S]). The second stage inserts the stack, the capsule separates on its
# springs, opens its nose cone and holds its attitude on the Dracos; the second stage coasts unguided.

[mission]
name = "Falcon 9 mit Crew Dragon"
vehicle = "falcon9_block5"
site = "ccsfs_slc40"                    # Crew-9 launched from SLC-40 [S]
landing_zone = "ccsfs_lz40"
heading_deg = 45.0                      # body +y on the pad (towards the ISS plane, north-east)
fairing = false

[payload]
vehicle = "crew_dragon"
separation_delay_s = 30.0               # [E] after second-stage cutoff
separation_dv_mps = 0.3                 # [E] spring push
nose_cone_open_s = 600.0                # [E] after the separation

[target]
periapsis_km = 200.0                    # [S] Crew Dragon insertion ~200 x 210 km (spec)
apoapsis_km = 210.0
inclination_deg = 51.6                  # ISS plane
northerly = true

[pitch_program]
kick_deg = 12.0                         # [E] as falcon9_rtls (the same 12.5 t on top)
booster_throttle = [[49.0, 1.0], [51.0, 0.72], [60.0, 0.72], [62.0, 0.95]]  # [D] CRS-35 bucket (see falcon9_rtls)
```

- [ ] **Step 2: Write the failing tests** (append):

```cpp
TEST_CASE("The crew mission loads its payload vehicle", "[dragon][config]") {
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_crew_dragon");
    REQUIRE(m.payload);
    CHECK(m.payload->vehicle_id == "crew_dragon");
    CHECK(m.payload->separation_delay_s == Catch::Approx(30.0));
    CHECK(m.payload->separation_dv_mps == Catch::Approx(0.3));
    CHECK(m.payload->nose_cone_open_s == Catch::Approx(600.0));
    CHECK(m.payload_kg == 0.0);
    CHECK_FALSE(config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_rtls").payload);
}

TEST_CASE("A mission without a payload vehicle ends in orbit as before", "[dragon][sim]") {
    // Review focus 5: no payload separation without a payload. The stack flies the upper-stage phase from a placed
    // state (second stage only, short burn) to cutoff and must end in state Orbit, not PayloadSeparation.
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const double gm = session.gm_earth_m3ps2();
    const orbit::KeplerianElements start{6378137.0 + 190e3, 0.01, 51.6 * constants::deg_to_rad, 0.0, 0.0, 0.5};
    session.place_in_orbit("falcon9_block5", orbit::state_from_elements(start, gm), t_deep, 1);
    session.engage_ascent_autopilot({200e3, 200e3, 51.6 * constants::deg_to_rad, true});
    session.ignite();
    for (double t = 1.0; t < 600.0 && session.autopilot().state != sim::AutopilotState::Orbit; t += 1.0) {
        session.advance_to(t_deep + t, 1000);
        CHECK(session.autopilot().state != sim::AutopilotState::PayloadSeparation);
    }
    CHECK(session.autopilot().state == sim::AutopilotState::Orbit);
    CHECK(session.body_count() == 1);
}

TEST_CASE("Falcon 9 lifts Crew Dragon into its insertion orbit and the capsule separates", "[dragon][mission][validation]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 13, 10, 33, 0.0});
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_crew_dragon");
    sim::start_ascent_mission(session, m, STARSIM_CONFIG_DIR, t0);
    double seco_s = -1.0, separation_s = -1.0;
    size_t stage2 = 0, capsule = 0;
    for (double t = 1.0; t < 1200.0; t += 1.0) {
        session.advance_to(t0 + t, 1000);
        const auto state = session.autopilot().state;
        if (seco_s < 0.0 && state == sim::AutopilotState::PayloadSeparation) seco_s = session.mission_time_s();
        if (separation_s < 0.0 && seco_s > 0.0 && state != sim::AutopilotState::PayloadSeparation) {
            separation_s = session.mission_time_s();
            capsule = session.controlled_index();
            stage2 = capsule - 1;
            break;
        }
    }
    REQUIRE(seco_s > 0.0);
    REQUIRE(separation_s > 0.0);
    CHECK(separation_s - seco_s == Catch::Approx(30.0).margin(0.1));
    const auto& dragon = session.body(capsule);
    CHECK(dragon.vehicle().stages().back().config.id == "capsule");
    CHECK(session.attitude_mode().kind == gnc::AttitudeModeKind::Inertial);
    // Insertion orbit of the capsule.
    const auto& s = dragon.state();
    const auto el = orbit::elements_from_state({s.position_gcrf_m, s.velocity_gcrf_mps}, session.gm_earth_m3ps2());
    const double peri_km = (el.semi_major_axis_m * (1.0 - el.eccentricity) - 6378137.0) / 1000.0;
    const double apo_km = (el.semi_major_axis_m * (1.0 + el.eccentricity) - 6378137.0) / 1000.0;
    std::printf("[validation] crew mission: SECO T+%.0f s, separation T+%.0f s, orbit %.1f x %.1f km, i %.2f deg\n", seco_s,
                separation_s, peri_km, apo_km, el.inclination_rad / constants::deg_to_rad);
    CHECK(peri_km == Catch::Approx(200.0).margin(5.0));   // [E] as the other ascent tests (J2 on osculating elements)
    CHECK(apo_km == Catch::Approx(210.0).margin(5.0));
    CHECK(el.inclination_rad / constants::deg_to_rad == Catch::Approx(51.6).margin(0.2));
    // Clearance: the distance grows for 60 s (attitude pulses must not push the capsule back) and exceeds 10 m.
    double last = norm(dragon.state().position_gcrf_m - session.body(stage2).state().position_gcrf_m);
    const double kg0 = dragon.vehicle().rcs_propellant_kg();
    const auto t_sep = session.epoch();
    for (double t = 1.0; t <= 60.0; t += 1.0) {
        session.advance_to(t_sep + t, 1000);
        const double d = norm(session.body(capsule).state().position_gcrf_m - session.body(stage2).state().position_gcrf_m);
        CHECK(d > last);
        last = d;
    }
    CHECK(last > 10.0);
    CHECK_FALSE(session.body(stage2).vehicle().any_engine_producing_thrust());
    // The nose cone opens 600 s after the separation; the capsule holds its attitude on the Dracos meanwhile.
    CHECK_FALSE(session.body(capsule).vehicle().nose_cone_open());
    session.advance_to(t_sep + 601.0, 200000);
    CHECK(session.body(capsule).vehicle().nose_cone_open());
    const double used = kg0 - session.body(capsule).vehicle().rcs_propellant_kg();
    std::printf("[validation] capsule: %.2f kg of MMH/NTO in the first 10 min\n", used);
    CHECK(used < 5.0);
    // The first stage landed on LZ-40 meanwhile (RTLS as in falcon9_rtls).
    CHECK(session.landing_outcome(0) == sim::LandingOutcome::Landed);
}
```

Include `starsim/config/mission_config.hpp`, `starsim/sim/ascent_mission.hpp`, `starsim/sim/landing_outcome.hpp`.

- [ ] **Step 3: Build and run them to see them fail**

Expected: compile errors (`payload` member of `AscentMission`, `AutopilotState::PayloadSeparation`).

- [ ] **Step 4: Load `[payload]`**

`mission_config.hpp`: add `struct MissionPayload` (as in Interfaces), and in `AscentMission`, after `fairing`:
`std::optional<MissionPayload> payload;  // a payload vehicle on top of the stack (exclusive with payload_kg)`.

`mission_config.cpp`, in `load_ascent_mission` after `fairing`:

```cpp
    if (const auto* payload = doc.root["payload"].as_table()) {
        if (m.payload_kg > 0.0) doc.fail("a mission has either payload_t or a [payload] vehicle");
        MissionPayload p;
        p.vehicle_id = doc.text(*payload, "vehicle");
        p.separation_delay_s = doc.number_or(*payload, "separation_delay_s", p.separation_delay_s);
        p.separation_dv_mps = doc.number_or(*payload, "separation_dv_mps", p.separation_dv_mps);
        p.nose_cone_open_s = doc.number_or(*payload, "nose_cone_open_s", p.nose_cone_open_s);
        if (p.separation_delay_s < 0.0 || p.separation_dv_mps < 0.0) doc.fail("[payload] times and speeds must be >= 0");
        m.payload = p;
    }
```

- [ ] **Step 5: Separation in the session**

`flight_session.hpp`:
- Add the `AutopilotState` value `PayloadSeparation` between `UpperStage` and `Orbit`, with this comment on the enum:
  `// PayloadSeparation: after the upper stage's cutoff, waiting to release a payload vehicle (d2).`
- Add `struct PayloadSeparation` (as in Interfaces) above `class FlightSession`.
- Public:

```cpp
    // Payload vehicle of the mission (config::stack_payload): released `delay_s` after the upper stage's cutoff with a
    // spring push; it becomes the controlled body, holds its attitude inertially and opens its nose cone after
    // `nose_cone_open_s` (< 0: closed). Takes effect at the next cutoff of the ascent autopilot; reset by a new launch.
    void set_payload_separation(std::optional<PayloadSeparation> separation) { payload_separation_ = std::move(separation); }
    // Releases the payload now (pilot action, or the autopilot): false if the controlled body carries no payload stage
    // above its lowest.
    bool separate_payload();
```

- Private: `std::optional<PayloadSeparation> payload_separation_; time::Epoch separation_epoch_; std::optional<time::Epoch> nose_cone_epoch_; size_t nose_cone_body_ = 0;`

`flight_session.cpp`:
- `to_string`: add `case AutopilotState::PayloadSeparation: return "Nutzlasttrennung";`.
- `run_autopilot`, `case AutopilotState::UpperStage`:

```cpp
        case AutopilotState::UpperStage:
            if (autopilot_.guidance.cutoff) {
                cutoff();
                autopilot_.state = AutopilotState::Orbit;
                if (payload_separation_) {
                    autopilot_.state = AutopilotState::PayloadSeparation;
                    separation_epoch_ = epoch_ + payload_separation_->delay_s;
                    // the stack holds its attitude on the second stage's thrusters until the release
                    attitude_mode_ = gnc::inertial_hold(body.state().attitude);
                }
            }
            break;
        case AutopilotState::PayloadSeparation:
            if (epoch_ >= separation_epoch_) {
                separate_payload();
                autopilot_.state = AutopilotState::Orbit;
            }
            break;
```

- `separate_payload()`:

```cpp
bool FlightSession::separate_payload() {
    if (bodies_.empty() || !payload_separation_) return false;
    auto& lower = bodies_[controlled_];
    const auto& stages = lower.body->vehicle().stages();
    size_t index = 0;
    for (size_t i = 1; i < stages.size(); ++i) {
        if (stages[i].config.id == payload_separation_->first_stage_id) index = i;
    }
    if (index == 0) return false;
    auto upper = lower.body->separate(index, payload_separation_->dv_mps);
    for (auto& [group, run] : lower.running_groups) run = false;
    upper->set_ground_height(launch_site_.height_m);
    bodies_.push_back({std::move(upper), {}, nullptr, {}});
    controlled_ = bodies_.size() - 1;
    attitude_mode_ = gnc::inertial_hold(bodies_[controlled_].body->state().attitude);
    if (payload_separation_->nose_cone_open_s >= 0.0) {
        nose_cone_epoch_ = epoch_ + payload_separation_->nose_cone_open_s;
        nose_cone_body_ = controlled_;
    }
    return true;
}
```

- `advance_to`: at the top of the tick loop, before the coast check:

```cpp
        if (nose_cone_epoch_ && epoch_ >= *nose_cone_epoch_) {
            set_nose_cone(nose_cone_body_, true);
            nose_cone_epoch_.reset();
        }
```

  In `coast_allowance_s()`, add `if (nose_cone_epoch_) allowance = std::min(allowance, *nose_cone_epoch_ - epoch_);`
  and make `PayloadSeparation` count as guided (ticked): add it to the `guided` expression there.
- `advance_to`'s autopilot dispatch already runs `run_autopilot()` for every state except Off and Orbit, so
  `PayloadSeparation` is covered. `command_for`'s `guided` must not include `PayloadSeparation`: the stack coasts with
  its engines off. In `PayloadSeparation` the stack gets no steering (pilot path). Add it to the attitude-mode
  condition instead: `autopilot_.state == AutopilotState::PayloadSeparation` with `attitude_mode_` set to
  `inertial_hold(current attitude)` when entering the state, so the stack holds its attitude on the second stage's
  nitrogen.
- `reset_mission_state()`: `payload_separation_.reset(); nose_cone_epoch_.reset();` — a new flight starts clean.
  Because `start_ascent_mission` calls `launch_from_pad` (which resets) before setting the separation, the order
  works.

`ascent_mission.cpp`:

```cpp
    auto vehicle = config::load_vehicle(config_dir, mission.vehicle_id);
    vehicle.stages.back().payload_mass_kg = mission.payload_kg;
    if (!mission.fairing) vehicle.stages.back().fairing_mass_kg = 0.0;
    std::optional<PayloadSeparation> separation;
    if (const auto& p = mission.payload) {
        const auto payload = config::load_vehicle(config_dir, p->vehicle_id);
        separation = PayloadSeparation{payload.stages.front().id, p->separation_delay_s, p->separation_dv_mps,
                                       p->nose_cone_open_s};
        vehicle = config::stack_payload(std::move(vehicle), payload);
    }
    session.launch_from_pad(vehicle, ...);   // unchanged
    ...
    session.set_payload_separation(separation);   // after launch_from_pad (it resets the mission state)
```

- [ ] **Step 6: Run the tests**

Run: `build/core/tests/starsim_core_tests.exe "[dragon]"`
Expected: all pass; the mission test prints SECO, separation, orbit and capsule propellant.
- If the orbit misses the bands, compare with the RTLS mission's orbit (same stack mass). A large difference points
  to the stacked aero (capsule `nose_length_m` 4.0 vs the second stage's 6.0) or to the CG. Ledger the finding before
  any change.
- If the first stage does not land, compare MECO with the RTLS mission (T+141 s): the stacked capsule must give the
  same liftoff mass as `payload_t = 12.5`.

- [ ] **Step 7: Docs**

`CLAUDE.md`, add a bullet after the "RCS (d1 ...)" bullet:

```markdown
- Crew Dragon (d2 of `docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md`, `config/vehicles/crew_dragon.toml`):
  - Structure: capsule + trunk, 12.5 t [S]. 16 Dracos (400 N, Isp 300 s, 10 ms pulses) [E layout]:
    - 4 under the nose cone push aft, usable only with the cone open (`Vehicle::set_nose_cone_open`,
      `FlightSession::set_nose_cone`);
    - 4 side pods (tangential +/-, forward) give pure couples on all axes.

    8 SuperDracos are an `abort_only` group: data only, never lit by the session.
  - Payload vehicles: a mission's `[payload]` stacks a vehicle on the launcher (`config::stack_payload`). Its stages
    are `payload` stages, whose thrusters are blocked until it flies alone.
  - Separation: `FlightSession::set_payload_separation` / `separate_payload`, at cutoff + 30 s with a 0.3 m/s push
    [E] (`AutopilotState::PayloadSeparation` "Nutzlasttrennung"). The capsule becomes the controlled body in an
    inertial hold, and its nose cone opens 600 s later [E].
  - Orbit attitude modes (`gnc/orbital_attitude`: Inertial, Lvlh, Direction, Sun; `FlightSession::set_attitude_mode`):
    - flown by `orbit_controller_` on pulses with a hold deadband [E] 0.5 deg / 0.05 deg/s;
    - any mode but Free keeps the session ticking.
  - RCS burns fly along +x when the vehicle has thrust along it, else along its strongest body axis (the hull turns
    it along the burn).
  - Mission `falcon9_crew_dragon` (SLC-40, 51.6 deg, 200 x 210 km, RTLS to LZ-40).
  - Validation figures: (fill in the printed figures of the tests: hold error and kg/h, burn errors, orbit,
    clearance).
```

`config/README.md`, under "Missions": add a `falcon9_crew_dragon.toml` bullet that lists the `[payload]` keys
(`vehicle`, `separation_delay_s`, `separation_dv_mps`, `nose_cone_open_s`) and says that the table is exclusive with
`payload_t`.

- [ ] **Step 8: Full suite, bindings build, smoke test, commit**

- Run the full suite. Expected: all pass, the existing campaigns unchanged.
- Run the bindings build. Expected: `BUILD_OK`.
- Run the smoke test. Expected: it passes (no interface change).

```bash
git add config/missions/falcon9_crew_dragon.toml core/include/starsim/config/mission_config.hpp \
  core/src/config/mission_config.cpp core/src/sim/ascent_mission.cpp core/include/starsim/sim/flight_session.hpp \
  core/src/sim/flight_session.cpp core/tests/test_crew_dragon.cpp CLAUDE.md config/README.md
git commit -m "feat(sim): Falcon 9 crew mission - Crew Dragon separates, opens its nose cone and holds attitude"
```

---

### Task 6: Final review and handover

- [ ] **Step 1:** Run the final whole-branch review per executing-plans (a fresh reviewer on the most capable model).
  Give it the package, this plan, the spec, the Review Focus and the ledger's rulings.
- [ ] **Step 2:** Run one fix pass for Critical/Important findings (each RED→GREEN, suite green). Ledger the minors as
  deferred.
- [ ] **Step 3:** Update memory `project-dragon-rcs-scope.md` with the d2 state (done, rulings, next: d3). Ask the
  user to merge/push (do not do it unasked), then ask for `/compact` before d3.
