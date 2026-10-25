# Refuelling r1: vehicles and docking geometry - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The vehicles of the refuelling milestone exist as configuration and can be built and mated on paper: a
tanker (Block 3 + four dorsal probes), a depot (Block 3 ship without entry hardware, better insulated, four dorsal
drogues), probes on the HLS, LOX/CH4 transfer connectors, a run-time tank-mass setter, and proof that the Starship
RCS can translate along the docking axis.

**Architecture:** Config first. A `[[stage]] from = "<vehicle>:<stage>"` entry reuses another vehicle's stage
unchanged (the tanker and the depot's Super Heavy are Block 3's), only docking ports and transfer connectors may be
added. `DockingPortConfig` gains a mechanism (`idss` | `probe_drogue`) with the probe/drogue positions;
`[[stage.transfer_connector]]` is new. A pure helper `vehicle/port_mating` maps a chaser's stage frame into the
target's for a mated port pair (the i5 convention: axes opposed, ups equal) and says whether two ports can mate at
all; `FlightSession::set_rendezvous_target` refuses mismatched ports and the IDSS mechanism is only armed for IDSS
pairs. `Vehicle::set_tank_mass` changes one tank at run time.

**Tech Stack:** C++20 core (float64, SI), Catch2, toml++ loader (`detail::Document`), Python 3.11 (`tomllib`) for
`tools/vehicle_report.py`.

**Spec:** `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md` (sections "r1 design", "What exists and
what changes", "Error handling", "Testing", "Documentation"). Subsystem notes: `docs/subsystems.md` (header tanks,
ISS i5 docking). Config conventions: `config/README.md` (provenance tags [S]/[E]/[D]).

## Plan decisions that need the user's approval

1. **`from` stages instead of copied TOML:** the tanker is Block 3 plus hardware, so `starship_tanker.toml` takes both
   stages with `from = "starship_block3:booster"` / `"starship_block3:ship"`; the depot takes the booster the same
   way. A reused stage may only add `docking_port` and `transfer_connector` entries - any other key is a config error
   (it would silently diverge from the source). The depot's ship stage is written out in full (it differs: dry mass,
   no flaps / heat shield / catch pins / recovery), as the HLS file already does.
2. **Depot dry mass 236.5 t [E]:** Block 3 ship 255 t - heat shield 10.5 t ([E] Block 1 figure, already in the
   config) - four flaps 10 t ([E] 2.5 t each; not published) - catch pins 1 t [E] + insulation 3 t ([E] ~2 kg/m^2 of
   multi-layer insulation on ~1,300 m^2 of tank hull). Dry CG kept at 23.516 m [E] (the removed masses sit fore, aft
   and along the belly). The insulation's thermal properties come with r2.
3. **Docking hardware layout [E]** (SpaceX publishes no positions): the port reference point is 0.5 m off the dorsal
   hull at station 20 m (`[20, 0, 5]`, axis `+z` = dorsal, up `+x` = nose); four probes / drogues in that interface
   plane at stations 12 and 28 m, `y = +-1.5 m` (symmetric, so a ship rolled 180 deg meets them); LOX connector at
   station 8 m (LOX main tank 3.5-20.4 m), CH4 connector at 24 m (CH4 main tank 20-32.6 m), both at `y = 0` in the
   interface plane. Mated, the centrelines lie 10 m apart (1 m between the hulls), both noses the same way.
4. **HLS legs:** the HLS's fixed legs (feet 12 m out, 1.5 m below the skirt) reach towards the depot when docked back
   to back. r1 only checks the hull cylinders; leg-strut clearance needs the leg geometry as collision primitives and
   belongs to r3 (where the hull collision of the pair is built). If it collides, r3 shifts the HLS port aft.
5. **Port compatibility rule:** two ports can mate when their mechanisms are equal; a probe/drogue pair additionally
   needs the probes on the chaser (`soft_capture = "active"`) and the drogues on the target (`"passive"`). IDSS keeps
   its i5 rule (the mechanism is armed only for an active chaser and a passive target).

## Global Constraints

- SI units internally, angles in radians; frame and unit in names (`position_m`, `mass_kg`).
- No magic numbers in code: physical constants in `core/include/starsim/math/constants.hpp`; vehicle numbers in
  `config/` with provenance tags [S] sourced, [E] estimate, [D] derived.
- C++ sources UTF-8; UI / error strings shown to the user are German, code and comments English.
- Small files with one responsibility; modules depend downwards only (`vehicle` may use `config` and `math`, never
  `sim` or `gnc`).
- ctest runs cases in parallel: tests must not share temp files or directories (unique names).
- Test workflow: run only the tags the change affects (`-TestFilter`), incl. `[slow]` cases of those tags.
- Commit on `main`, Conventional Commits, push after each task once its tests are green.
- Commit trailer:
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_012biTJd57gGHxfvAHbX6f2T`.
- Trademarks ("Starship") only internally; `display_name` in configs is neutral.

## Review Focus

1. A `from` stage that also sets a normal key (e.g. `dry_mass_kg`) must fail loudly, not quietly use the source's
   value - Task 2 error-case test.
2. A `from` chain that points at itself (cycle) must end in a config error, not a stack overflow - Task 2 test.
3. Refilling a ship whose tanks ran dry must bring its engines and its cold-gas RCS back (the tanker/HLS after a
   transfer) - Task 1 test.
4. The existing IDSS ports (ISS, Crew Dragon) must load and behave unchanged (default mechanism `idss`), and a Dragon
   must not be able to "dock" its IDSS port to a depot drogue - Task 3 tests plus the ISS tags.
5. A probe/drogue layout that does not line up when mated (one probe off its drogue, connectors of different
   contents meeting) must be caught by a test, not first in r3's contact model - Task 4 mating test.

---

### Task 1: Run-time tank mass

**Files:**
- Modify: `core/include/starsim/vehicle/vehicle.hpp` (after `main_propellant_kg()`, ~line 200)
- Modify: `core/src/vehicle/vehicle.cpp` (after `Vehicle::main_propellant_kg`, ~line 334)
- Create: `core/tests/test_refuelling_vehicles.cpp`
- Modify: `core/tests/CMakeLists.txt` (add the test file after `test_compound_mass.cpp`)

**Interfaces:**
- Produces: `void vehicle::Vehicle::set_tank_mass(size_t stage_index, std::string_view tank_id, double mass_kg);`
  clamps to `[0, capacity_kg]`; throws `std::invalid_argument` (German) for an unknown stage index or tank id;
  re-arms starved engines when the stage's selected feed source holds oxidizer and fuel; refreshes RCS availability.
- Produces (test helper, file-local in `test_refuelling_vehicles.cpp`, reused by Tasks 2-4):
  `std::unique_ptr<vehicle::Vehicle> top_stage_alone(const std::string& vehicle_id)`.

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_refuelling_vehicles.cpp`:

```cpp
// Refuelling r1: tanker, depot and HLS vehicles, run-time tank masses, probe/drogue mating geometry.

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

// The top stage of a vehicle flying alone (the ship after staging).
std::unique_ptr<vehicle::Vehicle> top_stage_alone(const std::string& vehicle_id) {
    auto cfg = config::load_vehicle(config_dir, vehicle_id);
    cfg.stages.erase(cfg.stages.begin(), cfg.stages.end() - 1);
    return std::make_unique<vehicle::Vehicle>(cfg, config_dir);
}

}  // namespace

TEST_CASE("set_tank_mass sets one tank at run time, clamped to its capacity", "[refuelling][vehicle]") {
    auto ship = top_stage_alone("starship_block3");
    const double full_kg = ship->mass_properties().mass_kg;
    const double lox_capacity_kg = 1224808.7;

    ship->set_tank_mass(0, "lox_main", 100000.0);
    CHECK(ship->mass_properties().mass_kg == Catch::Approx(full_kg - (lox_capacity_kg - 100000.0)));

    ship->set_tank_mass(0, "lox_main", 5.0e6);  // more than fits
    CHECK(ship->stages()[0].propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main) ==
          Catch::Approx(lox_capacity_kg));

    ship->set_tank_mass(0, "ch4_main", -1.0);
    CHECK(ship->stages()[0].propellant_kg(config::TankContents::Fuel, config::TankRole::Main) == 0.0);

    CHECK_THROWS_AS(ship->set_tank_mass(0, "no_such_tank", 1.0), std::invalid_argument);
    CHECK_THROWS_AS(ship->set_tank_mass(3, "lox_main", 1.0), std::invalid_argument);
}

TEST_CASE("A refilled dry ship has its engines and cold-gas RCS back", "[refuelling][vehicle]") {
    auto ship = top_stage_alone("starship_block3");
    for (const char* id : {"lox_main", "ch4_main", "lox_header", "ch4_header"}) ship->set_tank_mass(0, id, 0.0);
    ship->consume_propellant(0.0);  // the main feed is dry: the engines starve
    REQUIRE(ship->stages()[0].engines.front().starved);
    CHECK(ship->rcs_pure_force_capability({0.0, 0.0, 1.0}).force_n == 0.0);  // ullage gas comes from the tanks

    ship->set_tank_mass(0, "lox_main", 300000.0);
    ship->set_tank_mass(0, "ch4_main", 80000.0);
    CHECK_FALSE(ship->stages()[0].engines.front().starved);
    CHECK(ship->rcs_pure_force_capability({0.0, 0.0, 1.0}).force_n > 0.0);
}
```

Add `test_refuelling_vehicles.cpp` to `core/tests/CMakeLists.txt` right after `test_compound_mass.cpp`.

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[refuelling]" > tmp/r1_t1_red.txt 2>&1`
then read the tail. Expected: compile error "set_tank_mass is not a member of Vehicle".

- [ ] **Step 3: Implement**

`vehicle.hpp`, after `double main_propellant_kg() const;`:

```cpp
    // Sets one tank's mass at run time (refuelling r1: transfer, boil-off, the propellant test mode), clamped to
    // [0, capacity]. A refilled feed source re-arms starved engines; the RCS availability follows the tanks.
    // Throws std::invalid_argument (German) for an unknown stage index or tank id.
    void set_tank_mass(size_t stage_index, std::string_view tank_id, double mass_kg);
```

`vehicle.cpp`, after `Vehicle::main_propellant_kg`:

```cpp
void Vehicle::set_tank_mass(size_t stage_index, std::string_view tank_id, double mass_kg) {
    if (stage_index >= stages_.size()) {
        throw std::invalid_argument("Unbekannte Stufe " + std::to_string(stage_index));
    }
    auto& stage = stages_[stage_index];
    const auto tank = std::find_if(stage.tanks.begin(), stage.tanks.end(),
                                   [&](const Tank& t) { return t.config.id == tank_id; });
    if (tank == stage.tanks.end()) {
        throw std::invalid_argument("Unbekannter Tank '" + std::string(tank_id) + "'");
    }
    tank->mass_kg = std::clamp(mass_kg, 0.0, tank->config.capacity_kg);
    const auto role = stage.feed == FeedSource::Header ? config::TankRole::Header : config::TankRole::Main;
    if (stage.propellant_kg(config::TankContents::Oxidizer, role) > 0.0 &&
        stage.propellant_kg(config::TankContents::Fuel, role) > 0.0) {
        for (auto& e : stage.engines) e.starved = false;
    }
    refresh_rcs_availability();
}
```

Add `#include <string_view>` to `vehicle.hpp` and `<algorithm>`, `<stdexcept>` to `vehicle.cpp` if missing.

- [ ] **Step 4: Run to verify it passes**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[refuelling]" > tmp/r1_t1.txt 2>&1`
Expected: 2 test cases pass. Then the touched module: `-TestFilter "[vehicle],[header]"` > `tmp/r1_t1_vehicle.txt`,
all pass.

- [ ] **Step 5: Commit and push**

```bash
git add core/include/starsim/vehicle/vehicle.hpp core/src/vehicle/vehicle.cpp core/tests/test_refuelling_vehicles.cpp core/tests/CMakeLists.txt
git commit -m "feat(vehicle): run-time tank mass setter for refuelling"
git push origin main
```

---

### Task 2: Reused stages (`from`) and the tanker skeleton

**Files:**
- Modify: `core/src/config/vehicle_config.cpp` (`load_vehicle` ~535-572; new `load_stage_from` and
  `append_docking_ports` in the anonymous namespace before `load_vehicle`; the docking-port block of `load_stage`
  ~272-289 moves into `append_docking_ports`)
- Create: `config/vehicles/starship_tanker.toml`
- Modify: `tools/vehicle_report.py` (`report`, ~line 85: stages from `resolve_stages`)
- Test: `core/tests/test_refuelling_vehicles.cpp`

**Interfaces:**
- Consumes: `config::load_vehicle(config_dir, id)` (public, recursive use).
- Produces: TOML `[[stage]] from = "<vehicle>:<stage>"`; file-local
  `void append_docking_ports(const Document&, const toml::table&, StageConfig&)` (Task 3 extends it);
  vehicle `starship_tanker` with stages `booster`, `ship`.

- [ ] **Step 1: Write the failing tests**

Append to `test_refuelling_vehicles.cpp` (add `#include <fstream>`, `#include <utility>` at the top):

```cpp
namespace {

// A config directory of its own (ctest runs cases in parallel): engines, propellants, Block 3 and one test vehicle.
std::filesystem::path config_with(const std::string& name, const std::string& vehicle_toml) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_refuelling_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir / "engines", dir / "engines", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(config_dir / "propellants.toml", dir / "propellants.toml");
    std::filesystem::copy_file(config_dir / "vehicles" / "starship_block3.toml",
                               dir / "vehicles" / "starship_block3.toml");
    std::ofstream(dir / "vehicles" / "test_vehicle.toml") << vehicle_toml;
    return dir;
}

std::string vehicle_with_stage(const std::string& stage_body) {
    return "[vehicle]\nid = \"test_vehicle\"\nname = \"Test\"\npropellant = \"methalox\"\n"
           "stages_bottom_to_top = [\"ship\"]\n\n[[stage]]\n" +
           stage_body;
}

}  // namespace

TEST_CASE("A stage taken over with `from` is the source vehicle's stage", "[refuelling][config]") {
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    REQUIRE(tanker.stages.size() == block3.stages.size());
    for (size_t i = 0; i < block3.stages.size(); ++i) {
        const auto& a = block3.stages[i];
        const auto& b = tanker.stages[i];
        CAPTURE(a.id);
        CHECK(b.id == a.id);
        CHECK(b.dry_mass_kg == a.dry_mass_kg);
        CHECK(b.dry_cg_station_m == a.dry_cg_station_m);
        REQUIRE(b.tanks.size() == a.tanks.size());
        for (size_t k = 0; k < a.tanks.size(); ++k) {
            CHECK(b.tanks[k].id == a.tanks[k].id);
            CHECK(b.tanks[k].mass_kg == a.tanks[k].mass_kg);
            CHECK(b.tanks[k].capacity_kg == a.tanks[k].capacity_kg);
        }
        CHECK(b.engine_groups.size() == a.engine_groups.size());
        CHECK(b.rcs.thrusters.size() == a.rcs.thrusters.size());
        CHECK(b.flaps.size() == a.flaps.size());
    }
    CHECK(tanker.engines.size() == block3.engines.size());
}

TEST_CASE("Bad `from` stages are config errors", "[refuelling][config]") {
    const std::pair<const char*, const char*> cases[] = {
        {"from_override", "from = \"starship_block3:ship\"\ndry_mass_kg = 1.0\n"},
        {"from_no_stage", "from = \"starship_block3:nose\"\n"},
        {"from_no_vehicle", "from = \"no_such_vehicle:ship\"\n"},
        {"from_syntax", "from = \"starship_block3\"\n"},
        {"from_cycle", "from = \"test_vehicle:ship\"\n"},
    };
    for (const auto& [name, body] : cases) {
        CAPTURE(name);
        const auto dir = config_with(name, vehicle_with_stage(body));
        CHECK_THROWS_AS(config::load_vehicle(dir, "test_vehicle"), config::ConfigError);
    }
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[refuelling]" > tmp/r1_t2_red.txt 2>&1`
Expected: the `from` cases fail (`missing string 'id'` / tanker config file not found).

- [ ] **Step 3: Implement the loader**

In `vehicle_config.cpp`, move the `if (auto ports = t["docking_port"].as_array()) { ... }` block out of `load_stage`
into a new file-local function and call it from `load_stage` at the same place:

```cpp
// [[stage.docking_port]] entries of a stage table (also the additions to a reused stage, see load_stage_from).
void append_docking_ports(const Document& doc, const toml::table& t, StageConfig& s) {
    if (auto ports = t["docking_port"].as_array()) {
        // ... the existing block unchanged, pushing into s.docking_ports ...
    }
}
```

Before `load_vehicle`, declare the recursion guard and add `load_stage_from`:

```cpp
// [[stage]] from = "<vehicle>:<stage>": that vehicle's stage exactly as configured there (refuelling r1: the tanker
// and the depot's booster are Block 3's). Only docking ports and transfer connectors may be added; any other key would
// silently diverge from the source and is an error.
StageConfig load_stage_from(const std::filesystem::path& config_dir, const Document& doc, const toml::table& t,
                            const std::string& propellant_id) {
    constexpr int max_from_depth = 4;  // a longer chain is a cycle
    thread_local int depth = 0;
    if (depth >= max_from_depth) doc.fail("stage.from: chain too long (a cycle?)");
    struct DepthGuard {
        DepthGuard() { ++depth; }
        ~DepthGuard() { --depth; }
    } guard;

    const std::string from = doc.text(t, "from");
    const auto colon = from.find(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 == from.size()) {
        doc.fail("stage.from must be \"<vehicle>:<stage>\", got \"" + from + "\"");
    }
    for (const auto& [key, value] : t) {
        if (key != "from" && key != "docking_port" && key != "transfer_connector") {
            doc.fail("stage.from = \"" + from + "\": '" + std::string(key.str()) + "' cannot be set on a reused stage");
        }
    }
    const auto source = load_vehicle(config_dir, from.substr(0, colon));
    if (source.propellant.id != propellant_id) {
        doc.fail("stage.from = \"" + from + "\": propellant '" + source.propellant.id + "' differs from '" +
                 propellant_id + "'");
    }
    const std::string stage_id = from.substr(colon + 1);
    const auto it = std::find_if(source.stages.begin(), source.stages.end(),
                                 [&](const StageConfig& s) { return s.id == stage_id; });
    if (it == source.stages.end()) doc.fail("stage.from = \"" + from + "\": no such stage");
    StageConfig s = *it;
    append_docking_ports(doc, t, s);
    return s;
}
```

`transfer_connector` is accepted here already; Task 3 parses it. In `load_vehicle`, replace the stage loop body:

```cpp
        for (const auto& node : *stages) {
            const auto& table = *node.as_table();
            auto stage = table.contains("from") ? load_stage_from(config_dir, doc, table, v.propellant.id)
                                                : load_stage(doc, table);
            stages_by_id[stage.id] = std::move(stage);
        }
```

Note: a `ConfigError` thrown while loading the source vehicle (missing file) propagates unchanged - that is the
`from_no_vehicle` case.

- [ ] **Step 4: Create the tanker skeleton**

`config/vehicles/starship_tanker.toml`:

```toml
# Starship tanker (refuelling r1): a Block 3 ship that carries propellant to the depot. Same hull, tanks and engines as
# starship_block3.toml - its stages are taken over unchanged ([[stage]] from = "<vehicle>:<stage>"); the ship adds four
# docking probes on the dorsal side and the transfer connectors (spec 2026-10-20-orbital-refuelling-design.md).

[sources]
spacex_v3 = "SpaceX Starship V3 update, May 2026, via https://www.teslarati.com/spacex-unveils-sweeping-starship-v3-upgrades-ahead-may-19-launch/ (four docking drogues and dedicated propellant transfer connections)"

[vehicle]
id = "starship_tanker"
name = "Starship Tanker (Super Heavy Block 3 / Ship Block 3)"
display_name = "Tanker Mk3"
propellant = "methalox"
stages_bottom_to_top = ["booster", "ship"]

[[stage]]
from = "starship_block3:booster"

[[stage]]
from = "starship_block3:ship"
```

- [ ] **Step 5: vehicle_report.py follows `from`**

In `tools/vehicle_report.py`, add after `load_toml`:

```python
def resolve_stages(vehicle: dict) -> dict:
    """Stages by id; a `from = "<vehicle>:<stage>"` stage is the source vehicle's stage plus its added entries."""
    stages = {}
    for stage in vehicle["stage"]:
        if "from" in stage:
            source_id, stage_id = stage["from"].split(":")
            source = load_toml(CONFIG_DIR / "vehicles" / f"{source_id}.toml")
            added = {k: v for k, v in stage.items() if k != "from"}
            stage = {**resolve_stages(source)[stage_id], **added}
        stages[stage["id"]] = stage
    return stages
```

and in `report` replace `stages = {s["id"]: s for s in vehicle["stage"]}` with `stages = resolve_stages(vehicle)`.

- [ ] **Step 6: Run to verify it passes**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[refuelling]" > tmp/r1_t2.txt 2>&1`
Expected: all `[refuelling]` cases pass. Then the loader's other users:
`-TestFilter "[vehicle],[header],[hls],[iss],[dragon]"` > `tmp/r1_t2_config.txt`, all pass.
Run `python tools/vehicle_report.py starship_tanker` and `python tools/vehicle_report.py starship_block3`: identical
masses and delta-v (diff the numbers).

- [ ] **Step 7: Commit and push**

```bash
git add core/src/config/vehicle_config.cpp config/vehicles/starship_tanker.toml tools/vehicle_report.py core/tests/test_refuelling_vehicles.cpp
git commit -m "feat(config): reuse another vehicle's stage with from; Starship tanker skeleton"
git push origin main
```

---

### Task 3: Probe/drogue ports, transfer connectors, port mating

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp` (`DockingPortConfig` ~126; new `DockingMechanism`,
  `TransferConnectorConfig`; `StageConfig::transfer_connectors`)
- Modify: `core/src/config/vehicle_config.cpp` (`append_docking_ports`; new `append_transfer_connectors`, called from
  `load_stage` after the tanks are parsed and from `load_stage_from`)
- Create: `core/include/starsim/vehicle/port_mating.hpp`, `core/src/vehicle/port_mating.cpp`
- Modify: `core/CMakeLists.txt` (add `src/vehicle/port_mating.cpp` after `src/vehicle/docking_port.cpp`)
- Modify: `core/src/sim/flight_session.cpp` (`set_rendezvous_target` ~478)
- Modify: `core/src/sim/flight_session_docking.cpp` (`arm_docking` ~53)
- Test: `core/tests/test_refuelling_vehicles.cpp`

**Interfaces:**
- Consumes: `append_docking_ports` (Task 2), `config::load_vehicle`.
- Produces:
  - `enum class config::DockingMechanism { Idss, ProbeDrogue };`
    `DockingPortConfig::mechanism` (default `Idss`), `DockingPortConfig::elements_m` (`std::vector<math::Vec3>`,
    stage frame; probes on an `active` port, drogues on a `passive` one).
  - `struct config::TransferConnectorConfig { std::string id; std::string tank_id; TankContents contents;
    math::Vec3 position_m; };` and `StageConfig::transfer_connectors`.
  - `vehicle::StageTransform { math::Mat3 rotation; math::Vec3 translation_m; math::Vec3 apply(const math::Vec3&) const; }`
  - `vehicle::StageTransform vehicle::mated_stage_transform(const config::DockingPortConfig& chaser,
    const config::DockingPortConfig& target);` chaser stage frame -> target stage frame.
  - `std::optional<std::string> vehicle::port_mismatch(const config::DockingPortConfig& chaser,
    const config::DockingPortConfig& target);` German reason, or nullopt when they can mate.
  - `const config::DockingPortConfig& vehicle::port_config(const Vehicle&, std::string_view port_id);` throws
    `std::invalid_argument` (German) for an unknown id.

- [ ] **Step 1: Write the failing tests**

Append to `test_refuelling_vehicles.cpp` (add `#include "starsim/vehicle/port_mating.hpp"`):

```cpp
namespace {

config::DockingPortConfig dorsal_port(config::SoftCaptureMode side) {
    config::DockingPortConfig p;
    p.id = "transfer_dock";
    p.mechanism = config::DockingMechanism::ProbeDrogue;
    p.position_m = {20.0, 0.0, 5.0};
    p.axis = {0.0, 0.0, 1.0};
    p.up = {1.0, 0.0, 0.0};
    p.soft_capture = side;
    p.elements_m = {{12.0, -1.5, 5.0}, {12.0, 1.5, 5.0}, {28.0, -1.5, 5.0}, {28.0, 1.5, 5.0}};
    return p;
}

}  // namespace

TEST_CASE("Back-to-back mating puts the ships parallel, 10 m apart, rolled 180 deg", "[refuelling][docking]") {
    const auto t = vehicle::mated_stage_transform(dorsal_port(config::SoftCaptureMode::Active),
                                                  dorsal_port(config::SoftCaptureMode::Passive));
    const math::Vec3 nose = t.rotation * math::Vec3{1.0, 0.0, 0.0};
    CHECK(nose.x == Catch::Approx(1.0));                          // both noses the same way
    const math::Vec3 dorsal = t.rotation * math::Vec3{0.0, 0.0, 1.0};
    CHECK(dorsal.z == Catch::Approx(-1.0));                       // backs face each other
    const math::Vec3 axis_point = t.apply({20.0, 0.0, 0.0});
    CHECK(axis_point.x == Catch::Approx(20.0));
    CHECK(axis_point.y == Catch::Approx(0.0).margin(1e-12));
    CHECK(axis_point.z == Catch::Approx(10.0));                    // centrelines 10 m apart
}

TEST_CASE("Ports mate only with the same mechanism and probes against drogues", "[refuelling][docking]") {
    const auto probes = dorsal_port(config::SoftCaptureMode::Active);
    const auto drogues = dorsal_port(config::SoftCaptureMode::Passive);
    CHECK_FALSE(vehicle::port_mismatch(probes, drogues).has_value());
    CHECK(vehicle::port_mismatch(probes, probes).has_value());
    CHECK(vehicle::port_mismatch(drogues, probes).has_value());

    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    const auto iss = config::load_vehicle(config_dir, "iss");
    const auto& idss = dragon.stages.front().docking_ports.front();
    CHECK(idss.mechanism == config::DockingMechanism::Idss);      // existing ports default to IDSS
    CHECK(idss.elements_m.empty());
    CHECK_FALSE(vehicle::port_mismatch(idss, iss.stages.front().docking_ports.front()).has_value());
    CHECK(vehicle::port_mismatch(idss, drogues).has_value());     // a Dragon cannot dock to a depot drogue
}

TEST_CASE("Bad probe/drogue ports and transfer connectors are config errors", "[refuelling][config]") {
    const std::string ship = "from = \"starship_block3:ship\"\n";
    const std::string port_head = "[[stage.docking_port]]\nid = \"p\"\nname = \"P\"\nposition_m = [20.0, 0.0, 5.0]\n"
                                  "axis = [0.0, 0.0, 1.0]\nup = [1.0, 0.0, 0.0]\n";
    const std::pair<const char*, std::string> cases[] = {
        {"pd_no_elements", ship + port_head + "mechanism = \"probe_drogue\"\n"},
        {"pd_two_elements", ship + port_head +
                                "mechanism = \"probe_drogue\"\nelements_m = [[12.0, 0.0, 5.0], [28.0, 0.0, 5.0]]\n"},
        {"idss_elements", ship + port_head + "elements_m = [[12.0, 0.0, 5.0], [28.0, 0.0, 5.0], [20.0, 1.0, 5.0]]\n"},
        {"pd_mechanism", ship + port_head + "mechanism = \"magnet\"\n"},
        {"tc_no_tank", ship + "[[stage.transfer_connector]]\nid = \"c\"\ntank = \"nope\"\nposition_m = [8.0, 0.0, 5.0]\n"},
        {"tc_header", ship + "[[stage.transfer_connector]]\nid = \"c\"\ntank = \"lox_header\"\nposition_m = [8.0, 0.0, 5.0]\n"},
    };
    for (const auto& [name, body] : cases) {
        CAPTURE(name);
        const auto dir = config_with(name, vehicle_with_stage(body));
        CHECK_THROWS_AS(config::load_vehicle(dir, "test_vehicle"), config::ConfigError);
    }
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[refuelling]" > tmp/r1_t3_red.txt 2>&1`
Expected: compile errors (`DockingMechanism`, `port_mating.hpp` missing).

- [ ] **Step 3: Config types**

In `vehicle_config.hpp`, before `struct DockingPortConfig`:

```cpp
// The docking hardware of a port: the IDSS ring (ISS i5) or the Starship probe/drogue set (refuelling r1: four probes
// on the tanker / HLS into four drogues on the depot, dorsal side). TOML mechanism = "idss" (default) | "probe_drogue".
enum class DockingMechanism { Idss, ProbeDrogue };
```

Add to `DockingPortConfig`:

```cpp
    DockingMechanism mechanism = DockingMechanism::Idss;
    std::vector<math::Vec3> elements_m;  // probe_drogue: the probe tips (active port) / drogue mouths (passive), stage frame
```

After `DockingPortConfig`:

```cpp
// [[stage.transfer_connector]] (refuelling r1): a ship-to-ship propellant coupling on the hull, fed by one main tank;
// the coupling face's centre in the stage frame. Two docked ships' connectors of the same contents meet.
struct TransferConnectorConfig {
    std::string id;
    std::string tank_id;
    TankContents contents = TankContents::Oxidizer;  // the tank's, filled in by the loader
    math::Vec3 position_m;
};
```

Add `std::vector<TransferConnectorConfig> transfer_connectors;` to `StageConfig` after `docking_ports`.

- [ ] **Step 4: Loader**

In `append_docking_ports`, after the `soft_capture` parsing and before `push_back`:

```cpp
            const std::string mechanism = p["mechanism"].value_or(std::string("idss"));
            if (mechanism == "probe_drogue") {
                port.mechanism = DockingMechanism::ProbeDrogue;
            } else if (mechanism != "idss") {
                doc.fail("stage.docking_port '" + port.id + "': mechanism must be \"idss\" or \"probe_drogue\"");
            }
            if (const auto* elements = p["elements_m"].as_array()) {
                for (const auto& e : *elements) {
                    const auto* xyz = e.as_array();
                    if (!xyz || xyz->size() != 3) doc.fail("stage.docking_port '" + port.id + "': elements_m needs [x, y, z] entries");
                    const auto x = (*xyz)[0].value<double>(), y = (*xyz)[1].value<double>(), z = (*xyz)[2].value<double>();
                    if (!x || !y || !z) doc.fail("stage.docking_port '" + port.id + "': elements_m must be numbers");
                    port.elements_m.push_back({*x, *y, *z});
                }
            }
            constexpr size_t min_probe_drogue_elements = 3;  // fewer do not fix the mated pose
            if (port.mechanism == DockingMechanism::ProbeDrogue && port.elements_m.size() < min_probe_drogue_elements) {
                doc.fail("stage.docking_port '" + port.id + "': probe_drogue needs at least 3 elements_m");
            }
            if (port.mechanism == DockingMechanism::Idss && !port.elements_m.empty()) {
                doc.fail("stage.docking_port '" + port.id + "': elements_m belong to a probe_drogue port");
            }
```

New file-local function after `append_docking_ports`:

```cpp
// [[stage.transfer_connector]]: id, tank (a main tank of this stage), position_m. Needs the stage's tanks parsed.
void append_transfer_connectors(const Document& doc, const toml::table& t, StageConfig& s) {
    if (auto list = t["transfer_connector"].as_array()) {
        for (const auto& node : *list) {
            const auto& c = *node.as_table();
            TransferConnectorConfig connector{doc.text(c, "id"), doc.text(c, "tank"), TankContents::Oxidizer,
                                              vec3(doc, c, "position_m")};
            const auto tank = std::find_if(s.tanks.begin(), s.tanks.end(),
                                           [&](const TankConfig& k) { return k.id == connector.tank_id; });
            if (tank == s.tanks.end() || tank->role != TankRole::Main || tank->contents == TankContents::Rcs) {
                doc.fail("stage.transfer_connector '" + connector.id + "': tank must be a main propellant tank of the stage");
            }
            connector.contents = tank->contents;
            s.transfer_connectors.push_back(connector);
        }
    }
}
```

Call `append_transfer_connectors(doc, t, s);` in `load_stage` right after the tanks loop, and in `load_stage_from`
after `append_docking_ports(doc, t, s);`. (`TankConfig::id` and `TankConfig::role` are the existing fields.)

- [ ] **Step 5: port_mating**

`core/include/starsim/vehicle/port_mating.hpp`:

```cpp
#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::vehicle {

class Vehicle;

// A rigid map from one stage frame into another (x station from the stage bottom, y/z body axes).
struct StageTransform {
    math::Mat3 rotation;
    math::Vec3 translation_m;
    math::Vec3 apply(const math::Vec3& point_m) const { return rotation * point_m + translation_m; }
};

// Two mated ports (ISS i5 convention: the port axes opposed, the ups equal): maps points of the chaser's stage frame
// into the target's. Refuelling r1: two dorsal ports (axis +z, up +x) give parallel ships rolled 180 deg.
StageTransform mated_stage_transform(const config::DockingPortConfig& chaser, const config::DockingPortConfig& target);

// Why two ports cannot mate (German, shown to the user), or nullopt: the mechanisms must be equal, and a probe/drogue
// pair needs the probes on the chaser (soft_capture "active") and the drogues on the target ("passive").
std::optional<std::string> port_mismatch(const config::DockingPortConfig& chaser, const config::DockingPortConfig& target);

// The port `port_id` on any stage of the vehicle. Throws std::invalid_argument (German) for an unknown id.
const config::DockingPortConfig& port_config(const Vehicle& vehicle, std::string_view port_id);

}  // namespace starsim::vehicle
```

`core/src/vehicle/port_mating.cpp`:

```cpp
#include "starsim/vehicle/port_mating.hpp"

#include <stdexcept>

#include "starsim/vehicle/vehicle.hpp"

namespace starsim::vehicle {

namespace {

// Columns: the outward axis, y = up x axis, up (the port frame of vehicle/docking_port.hpp).
math::Mat3 port_axes(const config::DockingPortConfig& p) {
    return math::Mat3::from_columns(p.axis, math::cross(p.up, p.axis), p.up);
}

}  // namespace

StageTransform mated_stage_transform(const config::DockingPortConfig& chaser, const config::DockingPortConfig& target) {
    // Mated, the chaser's port axes are the target's (-x, -y, +z) (contact/docking_interface.hpp).
    const math::Mat3 flip = math::Mat3::from_columns({-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0});
    const math::Mat3 rotation = port_axes(target) * flip * math::transpose(port_axes(chaser));
    return {rotation, target.position_m - rotation * chaser.position_m};
}

std::optional<std::string> port_mismatch(const config::DockingPortConfig& chaser,
                                         const config::DockingPortConfig& target) {
    if (chaser.mechanism != target.mechanism) {
        return "Andocksysteme passen nicht zusammen ('" + chaser.id + "' und '" + target.id + "')";
    }
    if (chaser.mechanism == config::DockingMechanism::ProbeDrogue &&
        (chaser.soft_capture != config::SoftCaptureMode::Active ||
         target.soft_capture != config::SoftCaptureMode::Passive)) {
        return "Sonden gehören zum anfliegenden Schiff, Trichter zum Ziel ('" + chaser.id + "' -> '" + target.id + "')";
    }
    return std::nullopt;
}

const config::DockingPortConfig& port_config(const Vehicle& vehicle, std::string_view port_id) {
    for (const auto& stage : vehicle.stages()) {
        for (const auto& port : stage.config.docking_ports) {
            if (port.id == port_id) return port;
        }
    }
    throw std::invalid_argument("Unbekannter Andockport '" + std::string(port_id) + "'");
}

}  // namespace starsim::vehicle
```

Check the exact German text `port_kinematics` uses for an unknown port (`core/src/vehicle/docking_port.cpp`) and use
the same wording in `port_config`.

- [ ] **Step 6: Session guard**

`flight_session.cpp`, `set_rendezvous_target`, after the two `port_kinematics` validation calls:

```cpp
    if (const auto problem = vehicle::port_mismatch(
            vehicle::port_config(bodies_[controlled_].body->vehicle(), chaser_port),
            vehicle::port_config(bodies_[target_index].body->vehicle(), target_port))) {
        throw std::invalid_argument(*problem);
    }
```

(include `starsim/vehicle/port_mating.hpp`; update the header comment of `set_rendezvous_target` in
`flight_session.hpp`: "std::invalid_argument (German) for an unknown port or ports that cannot mate".)

`flight_session_docking.cpp`, `arm_docking`: extend the early return so the IDSS mechanism is only armed for IDSS
ports (the probe/drogue mechanism comes with r3):

```cpp
    if (!cp || !sp || cp->mechanism != config::DockingMechanism::Idss || sp->mechanism != config::DockingMechanism::Idss ||
        cp->soft_capture != config::SoftCaptureMode::Active || sp->soft_capture != config::SoftCaptureMode::Passive) {
        return;
    }
```

- [ ] **Step 7: Run to verify it passes**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[refuelling]" > tmp/r1_t3.txt 2>&1`
Expected: all pass. Then the IDSS paths: `-TestFilter "[docking],[iss],[rendezvous],[proximity],[vehicle]"`
(incl. `[slow]`) > `tmp/r1_t3_iss.txt`, all pass.

- [ ] **Step 8: Commit and push**

```bash
git add core/include/starsim/config/vehicle_config.hpp core/src/config/vehicle_config.cpp core/include/starsim/vehicle/port_mating.hpp core/src/vehicle/port_mating.cpp core/CMakeLists.txt core/src/sim/flight_session.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session_docking.cpp core/tests/test_refuelling_vehicles.cpp
git commit -m "feat(vehicle): probe/drogue docking ports, transfer connectors and port mating"
git push origin main
```

---

### Task 4: Tanker, depot and HLS hardware; Starship translation

**Files:**
- Modify: `config/vehicles/starship_tanker.toml` (ship stage: probes + connectors)
- Create: `config/vehicles/starship_depot.toml`
- Modify: `config/vehicles/starship_hls.toml` (probes + connectors)
- Test: `core/tests/test_refuelling_vehicles.cpp`

**Interfaces:**
- Consumes: `from` stages (Task 2), `mechanism`/`elements_m`/`transfer_connector` (Task 3),
  `vehicle::mated_stage_transform`, `vehicle::port_mismatch`, `top_stage_alone` (Task 1), `set_tank_mass` (Task 1).
- Produces: vehicles `starship_tanker` (ship port `transfer_dock`, active probes), `starship_depot` (stages
  `booster`, `depot`; port `transfer_dock`, passive drogues), `starship_hls` (port `transfer_dock`, active probes);
  connectors `lox_transfer` (tank `lox_main`) and `ch4_transfer` (tank `ch4_main`) on all three. r3-r5 use these ids.

- [ ] **Step 1: Write the failing tests**

Append to `test_refuelling_vehicles.cpp` (add `#include <cstdio>`):

```cpp
namespace {

const config::DockingPortConfig& transfer_dock(const config::StageConfig& s) {
    for (const auto& p : s.docking_ports) {
        if (p.id == "transfer_dock") return p;
    }
    throw std::runtime_error("no transfer_dock on stage " + s.id);
}

// Mates `chaser` (its top stage) to the depot's top stage: every probe on a drogue, every connector on the depot's
// connector of the same contents, and the hulls clear of each other.
void check_mates_with_depot(const std::string& chaser_id) {
    const auto depot = config::load_vehicle(config_dir, "starship_depot").stages.back();
    const auto chaser = config::load_vehicle(config_dir, chaser_id).stages.back();
    const auto& cp = transfer_dock(chaser);
    const auto& dp = transfer_dock(depot);
    REQUIRE_FALSE(vehicle::port_mismatch(cp, dp).has_value());
    const auto t = vehicle::mated_stage_transform(cp, dp);
    constexpr double match_m = 1e-9;
    REQUIRE(cp.elements_m.size() == dp.elements_m.size());
    for (const auto& probe : cp.elements_m) {
        const math::Vec3 at = t.apply(probe);
        double nearest_m = 1e9;
        for (const auto& drogue : dp.elements_m) nearest_m = std::min(nearest_m, math::norm(at - drogue));
        CHECK(nearest_m < match_m);
    }
    REQUIRE(chaser.transfer_connectors.size() == 2);
    for (const auto& c : chaser.transfer_connectors) {
        CAPTURE(c.id);
        const math::Vec3 at = t.apply(c.position_m);
        bool met = false;
        for (const auto& d : depot.transfer_connectors) {
            met = met || (d.contents == c.contents && math::norm(at - d.position_m) < match_m);
        }
        CHECK(met);
    }
    const math::Vec3 chaser_axis = t.apply({0.0, 0.0, 0.0});
    const double gap_m = std::hypot(chaser_axis.y, chaser_axis.z) - 0.5 * (chaser.diameter_m + depot.diameter_m);
    constexpr double min_hull_gap_m = 0.5;  // [E] the probes' reach
    CHECK(gap_m > min_hull_gap_m);
    CHECK((t.rotation * math::Vec3{1.0, 0.0, 0.0}).x == Catch::Approx(1.0));  // noses the same way
}

}  // namespace

TEST_CASE("Tanker and HLS mate back to back with the depot", "[refuelling][docking]") {
    check_mates_with_depot("starship_tanker");
    check_mates_with_depot("starship_hls");
}

TEST_CASE("The depot is a Block 3 ship without entry hardware, 236.5 t dry", "[refuelling][vehicle]") {
    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");
    REQUIRE(depot.stages.size() == 2);
    CHECK(depot.stages.front().dry_mass_kg == block3.stages.front().dry_mass_kg);  // the same Super Heavy
    const auto& s = depot.stages.back();
    CHECK(s.dry_mass_kg == 236500.0);
    double propellant_kg = 0.0;
    for (const auto& t : s.tanks) propellant_kg += t.mass_kg;
    CHECK(propellant_kg == Catch::Approx(1.6e6));
    CHECK(s.flaps.empty());
    CHECK(s.heat_shield.nose_radius_m == 0.0);
    CHECK(s.catch_pins.azimuths_rad.empty());
    CHECK(s.recovery_method.empty());
    CHECK(transfer_dock(s).soft_capture == config::SoftCaptureMode::Passive);
    CHECK(transfer_dock(s).elements_m.size() == 4);
}

TEST_CASE("A tanker after its transfer translates along the docking axis on its cold-gas RCS",
          "[refuelling][vehicle][rcs]") {
    auto tanker = top_stage_alone("starship_tanker");
    constexpr double residue_kg = 10000.0;  // [E] what is left in each main tank after the transfer (r4 computes it)
    tanker->set_tank_mass(0, "lox_main", residue_kg);
    tanker->set_tank_mass(0, "ch4_main", residue_kg);
    const double mass_kg = tanker->mass_properties().mass_kg;
    const math::Vec3 axes[] = {{0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}};
    for (const auto& u : axes) {
        const auto c = tanker->rcs_pure_force_capability(u);
        std::printf("[validation] tanker translation (%+.0f,%+.0f,%+.0f): %.0f N = %.5f m/s^2 at %.0f t\n", u.x, u.y,
                    u.z, c.force_n, c.force_n / mass_kg, mass_kg / 1000.0);
    }
    CHECK(tanker->rcs_pure_force_capability({0.0, 0.0, 1.0}).force_n > 0.0);   // dorsal: the docking axis
    CHECK(tanker->rcs_pure_force_capability({0.0, 0.0, -1.0}).force_n > 0.0);  // back off
    CHECK(tanker->rcs_pure_force_capability({0.0, 1.0, 0.0}).force_n > 0.0);
    CHECK(tanker->rcs_pure_force_capability({0.0, -1.0, 0.0}).force_n > 0.0);
    CHECK(tanker->rcs_pure_force_capability({1.0, 0.0, 0.0}).force_n > 0.0);   // settling thrust (r4)
    CHECK(tanker->rcs_pure_force_capability({-1.0, 0.0, 0.0}).force_n == Catch::Approx(0.0).margin(1.0));  // none aft
}
```

(add `#include <algorithm>`, `#include <cmath>` at the top.)

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[refuelling]" > tmp/r1_t4_red.txt 2>&1`
Expected: the new cases fail ("no transfer_dock", depot config not found).

- [ ] **Step 3: Tanker hardware**

Append to the ship entry of `starship_tanker.toml` (below `from = "starship_block3:ship"`):

```toml
# Docking probes and transfer connectors [E] (positions not published; spec r1, plan decision 3): the interface plane
# lies 0.5 m off the dorsal hull (z = +4.5 m; belly at -z). Mated with the depot's drogues the ships lie parallel,
# rolled 180 deg, 10 m between the centrelines. The layout is symmetric in y so the rolled ship meets it.
[[stage.docking_port]]
id = "transfer_dock"
name = "Transfer-Andockstelle (Rücken)"
mechanism = "probe_drogue"
position_m = [20.0, 0.0, 5.0]            # [E] centre of the four probes, in the interface plane
axis = [0.0, 0.0, 1.0]                   # dorsal normal
up = [1.0, 0.0, 0.0]                     # nose
soft_capture = "active"                  # the probes
elements_m = [[12.0, -1.5, 5.0], [12.0, 1.5, 5.0], [28.0, -1.5, 5.0], [28.0, 1.5, 5.0]]  # [E] probe tips

[[stage.transfer_connector]]
id = "lox_transfer"
tank = "lox_main"                        # LOX main tank 3.5-20.4 m
position_m = [8.0, 0.0, 5.0]             # [E]

[[stage.transfer_connector]]
id = "ch4_transfer"
tank = "ch4_main"                        # CH4 main tank 20-32.6 m
position_m = [24.0, 0.0, 5.0]            # [E]
```

- [ ] **Step 4: HLS hardware**

Append the same three tables (identical values, `soft_capture = "active"`) to the end of the `hls` stage in
`starship_hls.toml`, with the comment line `# Docking probes and transfer connectors [E] as the tanker's (refuelling
r1): the HLS fills up at the depot. Leg clearance at the depot: r3.` The HLS tank stations are the Block 3 ones
(3.5 / 20 m), so the connectors sit on its main tanks.

- [ ] **Step 5: Depot vehicle**

Create `config/vehicles/starship_depot.toml`. Header and vehicle table:

```toml
# Starship propellant depot (refuelling r1): a Block 3 ship that stays in Earth orbit and is filled by tankers (NASA /
# SpaceX HLS concept). Without heat shield, flaps, catch pins and recovery (it never returns), with an insulation layer
# against boil-off (thermal properties: r2). Four docking drogues on the dorsal side. Super Heavy as Block 3.
# dry_mass_kg always INCLUDES the installed engines.

[sources]
oig_2026 = "NASA OIG IG-26-004, March 2026 (LEO depot, >10 tankers, HLS fills up in LEO)"
wiki_hls = "https://en.wikipedia.org/wiki/Starship_HLS (retrieved 2026-10-20: depot without heat shield and fins)"
spacex_v3 = "SpaceX Starship V3 update, May 2026, via https://www.teslarati.com/spacex-unveils-sweeping-starship-v3-upgrades-ahead-may-19-launch/"

[vehicle]
id = "starship_depot"
name = "Starship Depot (Super Heavy Block 3 / Depot)"
display_name = "Depot Mk3"
propellant = "methalox"
stages_bottom_to_top = ["booster", "depot"]

[[stage]]
from = "starship_block3:booster"

[[stage]]
id = "depot"
name = "Starship Depot"
length_m = 52.1                          # [D] as the Block 3 ship
diameter_m = 9.0
dry_mass_kg = 236500.0                   # [E] Block 3 ship 255 t - heat shield 10.5 t - four flaps 10 t [E] - catch pins
                                         # 1 t [E] + insulation 3 t [E] (~2 kg/m^2 MLI on ~1,300 m^2 of tank hull)
dry_cg_station_m = 23.516                # [E] as Block 3: the removed masses sit fore, aft and along the belly
payload_mass_kg = 0.0
```

Then copy from `starship_block3.toml`'s ship stage, unchanged and with their comments: the four `[[stage.tank]]`
entries (lines 136-166), the six `[[stage.engine_group]]` entries (from line 172 to the end of the `vacuum` group,
~line 206), and `[stage.rcs]` with both `[[stage.rcs.pod]]` entries (lines 284-306). Add `[stage.aero]` without the
entry and flap keys:

```toml
[stage.aero]                             # ascent only (the depot never enters)
reference_diameter_m = 9.0
crossflow_table = "aero/ship_crossflow.csv"
ascent_drag_table = "aero/starship_stack_ascent.csv"
nose_length_m = 17.0
```

Leave out `[[stage.flap]]`, `[stage.catch_pins]`, `[stage.recovery]` and `[stage.heat_shield]`. Finish with the
drogues and connectors:

```toml
# Docking drogues and transfer connectors [E] (positions not published; spec r1, plan decision 3): mirror images of
# the tanker's / HLS's probes - interface plane 0.5 m off the dorsal hull, symmetric in y.
[[stage.docking_port]]
id = "transfer_dock"
name = "Transfer-Andockstelle (Rücken)"
mechanism = "probe_drogue"
position_m = [20.0, 0.0, 5.0]            # [E] centre of the four drogue mouths
axis = [0.0, 0.0, 1.0]                   # dorsal normal (belly at -z)
up = [1.0, 0.0, 0.0]                     # nose
soft_capture = "passive"                 # the drogues
elements_m = [[12.0, -1.5, 5.0], [12.0, 1.5, 5.0], [28.0, -1.5, 5.0], [28.0, 1.5, 5.0]]  # [E] drogue mouths

[[stage.transfer_connector]]
id = "lox_transfer"
tank = "lox_main"
position_m = [8.0, 0.0, 5.0]             # [E]

[[stage.transfer_connector]]
id = "ch4_transfer"
tank = "ch4_main"
position_m = [24.0, 0.0, 5.0]            # [E]
```

If a `[stage.recovery]`-less ship stage is refused by the loader or by `Vehicle` construction, stop and report it
(do not add a recovery block silently: the depot is not recovered).

- [ ] **Step 6: Run to verify it passes**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[refuelling]" > tmp/r1_t4.txt 2>&1`
Expected: all pass; note the `[validation]` translation lines (accelerations per axis) for the ledger. Then
`-TestFilter "[hls],[vehicle]"` > `tmp/r1_t4_hls.txt` (the HLS config changed), all pass.
Run `python tools/vehicle_report.py starship_depot` and `python tools/vehicle_report.py starship_tanker`, keep the
outputs in `tmp/r1_report_*.txt`; record the depot's Tsiolkovsky delta-v (it must exceed Block 3's by the 18.5 t
lighter ship) in the ledger.

- [ ] **Step 7: Commit and push**

```bash
git add config/vehicles/starship_tanker.toml config/vehicles/starship_depot.toml config/vehicles/starship_hls.toml core/tests/test_refuelling_vehicles.cpp
git commit -m "feat(config): Starship tanker, depot and HLS docking hardware for refuelling"
git push origin main
```

---

### Task 5: Documentation and final review

**Files:**
- Modify: `docs/subsystems.md` (new section "Orbital refuelling (r)" with a subsection r1)
- Modify: `config/README.md` (`from` stages, `mechanism` / `elements_m`, `[[stage.transfer_connector]]`)
- Modify: `CLAUDE.md` (Architecture paragraph: `vehicle` gains run-time tank masses and port mating; `config` reused
  stages - one clause each, in the style of the existing list)
- Modify: `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md` (Status line: r1 implemented, plan path;
  record plan decisions 1-5 under "r1 design" as approved corrections)
- Create: `.superpowers/sdd/2026-10-20-refuelling-r1-vehicles/progress.md` (ledger: tasks, commits, rulings, numbers)

- [ ] **Step 1: subsystems.md**

Add after the ISS sections:

```markdown
## Orbital refuelling (r)

Spec `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md`: depot + tankers + HLS, back-to-back docking,
boil-off by heat balance, a campaign calculator from one flown tanker flight.

### r1: vehicles and docking geometry
- `starship_tanker` = Block 3 (`[[stage]] from = "starship_block3:<stage>"`) + four dorsal probes (`transfer_dock`)
  and the transfer connectors `lox_transfer` / `ch4_transfer`; `starship_depot` = Block 3 booster + a ship without
  heat shield, flaps, catch pins, recovery: 236.5 t dry [E] (decision list in the r1 plan), four dorsal drogues;
  the HLS carries the tanker's probes.
- Layout [E]: interface plane 0.5 m off the dorsal hull, probes/drogues at stations 12 / 28 m, y +-1.5 m; connectors
  at 8 m (LOX) / 24 m (CH4). Mated (`vehicle::mated_stage_transform`): parallel, rolled 180 deg, centrelines 10 m
  apart, 1 m between the hulls.
- `Vehicle::set_tank_mass`: clamped to capacity, re-arms starved engines and the cold-gas RCS.
- Ports mate only with equal mechanisms, probes on the chaser (`vehicle::port_mismatch`; `set_rendezvous_target`
  refuses others); the IDSS mechanism is armed for IDSS pairs only - the probe/drogue mechanism comes with r3.
- Tanker translation after a transfer (main tanks 10 t each): <the [validation] numbers from Task 4>; no -x thruster.
- Open for r3: HLS leg-strut clearance at the depot.
```

Replace `<the [validation] numbers from Task 4>` with the measured per-axis accelerations from `tmp/r1_t4.txt`.

- [ ] **Step 2: config/README.md, CLAUDE.md, spec status** - as listed under Files.

- [ ] **Step 3: Verify**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -TestFilter "[refuelling],[vehicle],[hls],[header],[docking],[iss],[rendezvous],[proximity],[dragon]" > tmp/r1_final.txt 2>&1`
(with bindings: the session headers changed) - all pass. Smoke test:
`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd > tmp/r1_smoke.txt 2>&1` - passes.

- [ ] **Step 4: Commit and push docs**

```bash
git add docs/subsystems.md config/README.md CLAUDE.md docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md
git commit -m "docs: refuelling r1 vehicles and docking geometry"
git push origin main
```

- [ ] **Step 5: Final review**

Dispatch a fresh reviewer (most capable model) over the r1 commits against the spec and this plan
(superpowers:requesting-code-review). Fix Critical / Important findings in one fix pass (affected tags green, commit
`fix(...)`, push); record minors in the ledger. Update the memory (`project-orbital-refuelling`): r1 complete, next
step writing-plans r2.
