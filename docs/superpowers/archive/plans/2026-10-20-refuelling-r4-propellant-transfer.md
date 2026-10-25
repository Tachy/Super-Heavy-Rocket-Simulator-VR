# Refuelling r4: propellant transfer - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A tanker docked back to back at the depot settles its propellant with a small +x cold-gas push on the
compound, chills the lines and pushes LOX and CH4 through the transfer connectors by its ullage pressure until its main
tanks are down to the computed return reserve (or the depot is full); the depot fills an HLS the same way. Masses and CGs
of both ships follow live, boil-off continues for both, and the result (duration, delivered LOX / CH4, gas, chill-down,
boil-off) is reported.

**Architecture:** Three layers. `config` gets the transfer hardware (line size, discharge coefficient, chill-down mass
per connector; ullage pressures per stage). `vehicle/propellant_transfer` holds the physics as pure functions: the
orifice flow law, where the liquid lies relative to a tank outlet (apparent gravity + Bond number, reusing r2's
`LiquidEnvironment`), the settling time. `sim/propellant_transfer` is a pure state machine (Settling -> Chilldown ->
Flowing, Paused, Complete, Stopped) fed with tank masses and a "settled" flag. `FlightSession` (new
`flight_session_transfer.cpp`) wires it to a docked pair on a 10 s cadence: settling thrust as an impulse on the compound,
gas from the chaser's main tanks, tank moves through r3's `set_tank_mass` (compound refresh), and the docked pair's
boil-off evaluated by the session for both bodies in the compound's environment (r3 deferred M2). `mission/tanker_reserve`
computes the tanker's main-tank floor from the previewed deorbit burn.

**Tech Stack:** C++20 core (float64, SI), Catch2, toml++ loader (`detail::Document`), CSPICE through the `spice`
adapters (tests).

**Spec:** `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md` (sections "r4 design", "Error handling",
"Testing", "Documentation"; r1-r3 corrections). Notes: `docs/subsystems.md` "Orbital refuelling (milestone r)" (r1-r3).
Config conventions: `config/README.md`.

## Plan decisions that need the user's approval

1. **Transfer hardware [E] (SpaceX publishes nothing):** one line per propellant, inner diameter 0.15 m, discharge
   coefficient 0.6 (lumped line, valves, coupling: `mdot = C_d A sqrt(2 rho dp)`; the smaller `C_d A` of the coupled
   pair governs); the delivering ship holds its ullage at 3.5 bar (spec 3-4 bar), the receiving ship vents to 1.5 bar:
   dp = 2 bar. That gives LOX 232 kg/s, CH4 141 kg/s (densities of `propellants.toml`, subcooled): 145 t LOX in ~10 min.
   The hydrostatic head under 1e-4 g (~0.2 kPa) is left out. Chill-down: 150 kg LOX / 60 kg CH4 vented per connector
   (~300 kg of line and coupling steel from ~250 K, c 450 J/kgK, over the latent heat, +50 % [E]) over 120 s [E].
2. **Settling:** the chaser's +x ("forward") cold-gas thrusters push the compound at 1e-4 g [E, spec] (capped by their
   pure +x capability, 3 kN for the tanker: 1.3 kN needed for 1,360 t). Modelled as the average thrust, applied as a
   velocity impulse along the chaser's +x every 10 s step (0.01 m/s each), no pulses, no torque (the depot's ideal LVLH
   attitude takes it - documented simplification); gas at the RCS Isp (70 s) from the chaser's main tanks in proportion to
   their oxidizer and fuel. Sequence: Settling for `sqrt(2 L / a)` [E] (L = the longest involved tank's full length:
   186 s for 16.9 m at 1e-4 g) -> Chilldown (120 s) -> Flowing. A tank's outlet (aft dome, body -x) is covered when the
   apparent gravity at the tank centre (tide and rotation about the compound CG, minus drag and settling thrust) gives
   Bo >= 10 (r2's `settled_weight` = 1) and lies within 30 deg [E] of -x. An uncovered outlet pauses the transfer
   (German status), a covered one resumes it.
3. **Docked boil-off by the session (r3 deferred M2):** while a station carries a docked visitor its own `FlightBody`
   boil-off is off; the session evaluates both bodies every 10 s and after every coast in the compound's environment
   (specific force of the compound, tide and rotation about the compound CG) and vents through
   `FlightSession::set_tank_mass` (trapezoid as r2), so the compound CG follows. This runs whenever a pair is docked,
   not only during a transfer (the ISS pair has no `[stage.thermal]`: unchanged).
4. **Tanker reserve:** `mission::tanker_reserve`: the deorbit burn by the rocket equation with the delta-v of the
   deorbit plan previewed from the docked state (`FlightSession::ship_return_preview`, the existing planner without the
   boil-off check), from the mass after the burn (ship without main propellant, headers full, + gas + unusable), times
   the r2 commit margin 1.10; plus cold gas 1.5 t [E] (departure, turn to the burn, entry), boil-off until the burn 2 t
   [E] (r2: a 10 t residue loses ~1.4 t in 22 h), unusable 1 t [E]; split at the deorbit set's mixture ratio. ~12-13 t
   for 80 m/s. r5 flies the return with it and reports whether it holds (no tuning here).
5. **Floors:** a delivering line closes at its floor; settling gas and boil-off drawn after that (tens of kg until the
   other line closes) come out of the reserve's gas / boil-off items.
6. **One model both ways:** `TransferDirection::ChaserToStation` (tanker -> depot) and `StationToChaser` (depot -> HLS,
   with per-line limits). The settling thrust always comes from the chaser.
7. **Scope:** core only. `FlightSim` getters / commands (`transfer_state`, `start_transfer`) follow in r6 with the
   campaign (spec r6 "Game"); r4 keeps the bindings compiling.

## Global Constraints

- SI units internally, angles in radians; frame and unit in names (`delivery_pressure_pa`, `line_diameter_m`).
- No magic numbers in code: physical constants in `core/include/starsim/math/constants.hpp` with source; vehicle numbers
  in `config/` with provenance tags [S] / [E] / [D]; model estimates as commented `[E]` defaults in their parameter
  struct (as `TransferRequest`).
- C++ sources UTF-8; user-facing strings German, code and comments English.
- Modules depend downwards only: `vehicle` may use `config`, `environment`, `math`; `mission` never includes `sim`;
  `sim` may use all.
- The ISS / Crew Dragon behaviour must not change: tags `[iss]`, `[docking]`, `[proximity]`, `[iss_mission]` stay
  green including their `[slow]` cases (Task 4 touches the docked path, Task 5 `undock`).
- ctest runs cases in parallel: tests must not share temp files or directories (unique `starsim_r4_<name>` prefixes).
- Test workflow: run only the tags the change affects, `[slow]` cases of those tags included. Build with
  `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests > tmp/r4_build.txt 2>&1`, then run the
  labels with `ctest --test-dir build -L "<regex>" -j 24 --output-on-failure > tmp/r4_tN.txt 2>&1` and read the tail
  (CTest labels = Catch tags). A single new case for RED: `build/core/tests/starsim_core_tests.exe "<name>"`. Runs that
  may exceed 10 min go to the background, checked every 10 min, stopped early on failures.
- Commit on `main`, Conventional Commits, push after each task once its tests are green. Trailer:
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_012biTJd57gGHxfvAHbX6f2T`.

## Review Focus

1. A transfer whose receiving tank is already full or whose delivering tank is already at its floor: a German refusal
   ("nichts zu übertragen"), no zero-length run and no division by zero - Task 5 test "Refusals".
2. The chaser has no cold gas (main tanks empty, e.g. an HLS arriving dry) or the settling thrust is zero: the transfer
   pauses with a German reason and never flows unsettled - Task 3 test "no thrust pauses" and Task 5 test "zero
   settling acceleration pauses".
3. Time warp: the result must not depend on how `advance_to` is chunked (1 s calls vs one 20 min call) - Task 5 test
   "chunking".
4. Undocking, a second start or a stop during the transfer: undock refused ("Treibstofftransfer läuft"), stop closes the
   valves and ends the thrust, a second start refused - Task 5 test "Refusals" and "stop".
5. A docked depot's own boil-off must not bypass the compound (r3 M2): the hulls stay mated to 1e-5 m over hours of
   venting - Task 4 test "docked boil-off keeps the hulls mated".

---

### Task 1: Transfer hardware in the configuration

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp:160-167` (`TransferConnectorConfig` fields), `:333`
  (add `PropellantTransferConfig` + `StageConfig::propellant_transfer`)
- Modify: `core/src/config/vehicle_config.cpp:225-242` (`append_transfer_connectors`), `load_stage` (read the block),
  `:655` (`load_stage_from`: allow `propellant_transfer`), after `append_transfer_connectors` in `load_stage_from`
- Modify: `config/vehicles/starship_tanker.toml`, `config/vehicles/starship_depot.toml`, `config/vehicles/starship_hls.toml`
- Modify: `config/README.md` (the new keys)
- Create: `core/tests/test_refuelling_transfer.cpp`; Modify: `core/tests/CMakeLists.txt` (after
  `test_refuelling_rendezvous.cpp`)

**Interfaces:**
- Produces: `config::TransferConnectorConfig::{line_diameter_m, discharge_coefficient, chilldown_kg}` (required on
  every `[[stage.transfer_connector]]`: diameter and coefficient positive, chill-down >= 0).
- Produces: `struct config::PropellantTransferConfig { double delivery_pressure_pa; double receive_pressure_pa; }` and
  `std::optional<PropellantTransferConfig> StageConfig::propellant_transfer` (`[stage.propellant_transfer]`, both
  positive, receive below delivery; allowed on a reused `from` stage).

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_refuelling_transfer.cpp`:

```cpp
// Refuelling r4: propellant transfer between docked ships.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/vehicle_config.hpp"

using namespace starsim;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

const config::StageConfig& ship_stage(const std::string& id) {
    static std::map<std::string, config::VehicleConfig> cache;
    auto it = cache.find(id);
    if (it == cache.end()) it = cache.emplace(id, config::load_vehicle(config_dir, id)).first;
    return it->second.stages.back();
}

}  // namespace

TEST_CASE("Tanker, depot and HLS carry the transfer hardware", "[refuelling][prop_transfer][config]") {
    for (const std::string id : {"starship_tanker", "starship_depot", "starship_hls"}) {
        INFO(id);
        const auto& s = ship_stage(id);
        REQUIRE(s.propellant_transfer);
        CHECK(s.propellant_transfer->delivery_pressure_pa == 3.5e5);
        CHECK(s.propellant_transfer->receive_pressure_pa == 1.5e5);
        REQUIRE(s.transfer_connectors.size() == 2);
        for (const auto& c : s.transfer_connectors) {
            CHECK(c.line_diameter_m == 0.15);
            CHECK(c.discharge_coefficient == 0.6);
            CHECK(c.chilldown_kg == (c.contents == config::TankContents::Oxidizer ? 150.0 : 60.0));
        }
    }
    CHECK_FALSE(ship_stage("starship_block3").propellant_transfer);
}
```

Add the config-error cases with the existing temp-dir helper pattern of `test_refuelling_docking.cpp` (`config_with`,
own prefix `starsim_r4_`): copy that helper (it writes Block 3, engines, propellants and one test vehicle) into this file
under the name `config_with`, then:

```cpp
namespace {

// A config dir with Block 3, the engines and propellants and one test vehicle (own temp prefix: ctest runs cases in
// parallel).
std::filesystem::path config_with(const std::string& name, const std::string& vehicle_toml) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_r4_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir / "engines", dir / "engines", std::filesystem::copy_options::recursive);
    std::filesystem::copy(config_dir / "aero", dir / "aero", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(config_dir / "propellants.toml", dir / "propellants.toml");
    std::filesystem::copy_file(config_dir / "vehicles" / "starship_block3.toml", dir / "vehicles" / "starship_block3.toml");
    std::ofstream(dir / "vehicles" / (name + ".toml")) << vehicle_toml;
    return dir;
}

std::string tanker_with(const std::string& name, const std::string& ship_additions) {
    return "[vehicle]\nid = \"" + name + "\"\nname = \"t\"\ndisplay_name = \"t\"\npropellant = \"methalox\"\n"
           "stages_bottom_to_top = [\"ship\"]\n\n[[stage]]\nfrom = \"starship_block3:ship\"\n" + ship_additions;
}

}  // namespace

TEST_CASE("Transfer hardware errors are reported", "[refuelling][prop_transfer][config]") {
    const std::string connector =
        "[[stage.transfer_connector]]\nid = \"lox\"\ntank = \"lox_main\"\nposition_m = [8.0, 0.0, 5.0]\n";
    SECTION("a connector without its line") {
        const auto dir = config_with("r4_no_line", tanker_with("r4_no_line", connector));
        CHECK_THROWS_WITH(config::load_vehicle(dir, "r4_no_line"),
                          Catch::Matchers::ContainsSubstring("line_diameter_m"));
    }
    SECTION("receive pressure not below delivery") {
        const auto dir = config_with(
            "r4_pressure",
            tanker_with("r4_pressure", connector + "line_diameter_m = 0.15\ndischarge_coefficient = 0.6\n"
                                                   "chilldown_kg = 150.0\n\n[stage.propellant_transfer]\n"
                                                   "delivery_pressure_pa = 2.0e5\nreceive_pressure_pa = 2.0e5\n"));
        CHECK_THROWS_WITH(config::load_vehicle(dir, "r4_pressure"),
                          Catch::Matchers::ContainsSubstring("receive_pressure_pa"));
    }
}
```

(If the existing `config_with` in `test_refuelling_docking.cpp` copies a different set of directories, copy exactly
what it copies - it is the working template. Add `#include <map>` for the cache.)

Register the file in `core/tests/CMakeLists.txt` after `test_refuelling_rendezvous.cpp`.

- [ ] **Step 2: Run the tests to verify they fail**

Run: build (`tools/build.ps1 -NoBindings -NoTests > tmp/r4_build.txt 2>&1`) - expected: compile error, the fields do
not exist.

- [ ] **Step 3: Implement the config**

`vehicle_config.hpp`, extend the connector and add the block after it:

```cpp
// [[stage.transfer_connector]] (refuelling r1): a ship-to-ship propellant coupling on the hull, fed by one main tank;
// the coupling face's centre in the stage frame. Two docked ships' connectors of the same contents meet.
// r4: the line through it (the smaller C_d A of a coupled pair governs the flow) and the propellant vented to chill
// it before each transfer.
struct TransferConnectorConfig {
    std::string id;
    std::string tank_id;
    TankContents contents = TankContents::Oxidizer;  // the tank's, filled in by the loader
    math::Vec3 position_m;
    double line_diameter_m = 0.0;
    double discharge_coefficient = 0.0;  // lumped line, valve and coupling losses: mdot = C_d A sqrt(2 rho dp)
    double chilldown_kg = 0.0;
};

// [stage.propellant_transfer] (refuelling r4): the ullage pressures of a ship-to-ship transfer. The delivering ship
// holds its ullage at delivery_pressure_pa, the receiving ship vents down to receive_pressure_pa; their difference
// drives the flow.
struct PropellantTransferConfig {
    double delivery_pressure_pa = 0.0;
    double receive_pressure_pa = 0.0;
};
```

and in `StageConfig` after `transfer_connectors`:

```cpp
    std::optional<PropellantTransferConfig> propellant_transfer;  // [stage.propellant_transfer] (refuelling r4)
```

`vehicle_config.cpp`, in `append_transfer_connectors` after `connector.contents = tank->contents;`:

```cpp
            connector.line_diameter_m = doc.positive(c, "line_diameter_m");
            connector.discharge_coefficient = doc.positive(c, "discharge_coefficient");
            connector.chilldown_kg = doc.number(c, "chilldown_kg");
            if (connector.chilldown_kg < 0.0) {
                doc.fail("stage.transfer_connector '" + connector.id + "': chilldown_kg must not be negative");
            }
```

A reader beside it, called in `load_stage` next to `append_transfer_connectors(doc, t, s)` and in `load_stage_from`
after its `append_transfer_connectors(doc, t, s)` (a reused stage may add it; the source stage's block, if any, is
replaced):

```cpp
// [stage.propellant_transfer]: delivery_pressure_pa, receive_pressure_pa (both positive, receive below delivery).
void read_propellant_transfer(const Document& doc, const toml::table& t, StageConfig& s) {
    if (auto block = t["propellant_transfer"].as_table()) {
        PropellantTransferConfig p{doc.positive(*block, "delivery_pressure_pa"),
                                   doc.positive(*block, "receive_pressure_pa")};
        if (p.receive_pressure_pa >= p.delivery_pressure_pa) {
            doc.fail("stage.propellant_transfer: receive_pressure_pa must be below delivery_pressure_pa");
        }
        s.propellant_transfer = p;
    }
}
```

In `load_stage_from`'s key filter add `key != "propellant_transfer"`.

Config files - every `[[stage.transfer_connector]]` of tanker, depot and HLS gets (LOX / CH4):

```toml
line_diameter_m = 0.15                   # [E] one line per propellant (SpaceX publishes no size)
discharge_coefficient = 0.6              # [E] lumped line, valve and coupling losses
chilldown_kg = 150.0                     # [E] LOX: ~300 kg of line and coupling steel from ~250 K (c 450 J/kgK) over
                                         # 213 kJ/kg ~100 kg, +50 % (CH4: 60.0, the same over 510 kJ/kg ~37 kg)
```

(`chilldown_kg = 60.0` on the CH4 connectors.) And per ship stage, after the connectors:

```toml
# Transfer pressures [E] (refuelling r4; spec: pressure-driven, Ars Technica 2024): the delivering ship holds its ullage
# at 3.5 bar (spec 3-4 bar), the receiving ship vents to 1.5 bar - 2 bar drive LOX at ~230 kg/s, CH4 at ~140 kg/s.
[stage.propellant_transfer]
delivery_pressure_pa = 3.5e5
receive_pressure_pa = 1.5e5
```

`config/README.md`: document the three connector keys and `[stage.propellant_transfer]` (also allowed on a `from`
stage) in the refuelling section next to `[[stage.transfer_connector]]`.

- [ ] **Step 4: Run the tests to verify they pass**

Build, then `ctest --test-dir build -L "refuelling|config" -j 24 --output-on-failure > tmp/r4_t1.txt 2>&1`.
Expected: all pass (the r1-r3 `[refuelling]` cases load the same vehicles). Also
`python tools/vehicle_report.py starship_tanker > tmp/r4_t1_report.txt` runs without error (it must ignore the new
keys; if it chokes on them, teach its `from` handling to pass `propellant_transfer` through).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/config/vehicle_config.hpp core/src/config/vehicle_config.cpp config/vehicles/starship_tanker.toml config/vehicles/starship_depot.toml config/vehicles/starship_hls.toml config/README.md core/tests/test_refuelling_transfer.cpp core/tests/CMakeLists.txt
git commit -m "feat(config): transfer lines and ullage pressures for ship-to-ship propellant transfer"
git push origin main
```

---

### Task 2: Transfer physics - line flow, outlet coverage, settling time

**Files:**
- Modify: `core/include/starsim/vehicle/boil_off.hpp:56-67` (`LiquidEnvironment::cg_offset_body_m`),
  `core/src/vehicle/boil_off.cpp:57-66` (`apparent_gravity_body_mps2` uses it)
- Create: `core/include/starsim/vehicle/propellant_transfer.hpp`, `core/src/vehicle/propellant_transfer.cpp`
- Modify: `core/CMakeLists.txt` (add `src/vehicle/propellant_transfer.cpp` after `src/vehicle/port_mating.cpp`)
- Test: `core/tests/test_refuelling_transfer.cpp`

**Interfaces:**
- Consumes: `config::TransferConnectorConfig` (Task 1); `vehicle::LiquidEnvironment`, `apparent_gravity_body_mps2`,
  `settled_weight` (r2); `vehicle::Tank::{density_kg_m3, radius_m, length_m, surface_tension_n_m, mass_kg}`.
- Produces:
  - `math::Vec3 LiquidEnvironment::cg_offset_body_m` - this body's CG from the free-falling reference point (the
    compound CG of a docked pair), body axes; default zero (r2 callers unchanged). `position_gcrf_m` is then the
    reference point's position.
  - `double vehicle::line_mass_flow_kgps(double discharge_area_m2, double density_kg_m3, double pressure_difference_pa)`
  - `double vehicle::pair_discharge_area_m2(const config::TransferConnectorConfig& a, const config::TransferConnectorConfig& b)`
  - `struct vehicle::OutletCheck { bool covered; double bond_number; double tilt_rad; }`
  - `vehicle::OutletCheck vehicle::outlet_check(const Vehicle&, size_t stage_index, size_t tank_index, const LiquidEnvironment&, double max_tilt_rad)`
  - `double vehicle::settling_time_s(double length_m, double acceleration_mps2)`

- [ ] **Step 1: Write the failing tests**

Append to `core/tests/test_refuelling_transfer.cpp` (add includes `starsim/vehicle/propellant_transfer.hpp`,
`starsim/vehicle/vehicle.hpp`, `starsim/math/constants.hpp`, `<numbers>`):

```cpp
namespace {

std::unique_ptr<vehicle::Vehicle> ship_alone(const std::string& id) {
    return std::make_unique<vehicle::Vehicle>(config::top_stage_config(config::load_vehicle(config_dir, id)), config_dir);
}

size_t tank_index(const vehicle::Vehicle& v, const std::string& id) {
    const auto& tanks = v.stages()[0].tanks;
    for (size_t i = 0; i < tanks.size(); ++i) {
        if (tanks[i].config.id == id) return i;
    }
    throw std::invalid_argument(id);
}

// 200 km, body z radial (body x perpendicular to the radius, as the along-track nose), no rotation: along x the tide
// is compressive (~1.3e-6 m/s^2 per m), the radial offsets are stretched (2x that).
vehicle::LiquidEnvironment orbit_liquid(const math::Vec3& specific_force_body_mps2) {
    const double r = constants::wgs84_semi_major_axis_m + 200.0e3;
    return {specific_force_body_mps2, {}, {0.0, 0.0, r}, math::Quat::identity()};
}

}  // namespace

TEST_CASE("Line flow is the orifice law of the smaller C_d A", "[refuelling][prop_transfer]") {
    const double area = 0.6 * std::numbers::pi / 4.0 * 0.15 * 0.15;  // 0.0106029 m^2
    CHECK(vehicle::line_mass_flow_kgps(area, 1200.0, 2.0e5) == Catch::Approx(232.30).epsilon(1e-4));
    CHECK(vehicle::line_mass_flow_kgps(area, 440.0, 2.0e5) == Catch::Approx(140.66).epsilon(1e-4));
    CHECK(vehicle::line_mass_flow_kgps(area, 1200.0, 0.0) == 0.0);
    CHECK(vehicle::line_mass_flow_kgps(area, 1200.0, -1.0e5) == 0.0);  // no back-flow: the valves close
    config::TransferConnectorConfig big, small;
    big.line_diameter_m = 0.20;
    big.discharge_coefficient = 0.6;
    small.line_diameter_m = 0.15;
    small.discharge_coefficient = 0.6;
    CHECK(vehicle::pair_discharge_area_m2(big, small) == Catch::Approx(area));
    CHECK(vehicle::pair_discharge_area_m2(small, big) == Catch::Approx(area));
}

TEST_CASE("Settling time: the liquid crosses the tank from rest", "[refuelling][prop_transfer]") {
    CHECK(vehicle::settling_time_s(16.9, 9.80665e-4) == Catch::Approx(185.65).epsilon(1e-3));
    CHECK(vehicle::settling_time_s(16.9, 0.0) == std::numeric_limits<double>::infinity());
}

TEST_CASE("Outlet coverage: a +x push settles the liquid aft, drag alone moves it forward",
          "[refuelling][prop_transfer]") {
    auto tanker = ship_alone("starship_tanker");
    tanker->set_tank_mass(0, "lox_main", 50.0e3);
    const size_t lox = tank_index(*tanker, "lox_main");
    const double max_tilt = 30.0 * constants::deg_to_rad;
    // settling thrust 1e-4 g along +x: apparent gravity aft, Bo ~ 1200 * 1e-3 * 4.5^2 / 0.0132 ~ 1.8e3
    const auto pushed = vehicle::outlet_check(*tanker, 0, lox, orbit_liquid({9.80665e-4, 0.0, 0.0}), max_tilt);
    CHECK(pushed.covered);
    CHECK(pushed.bond_number > 1.0e3);
    CHECK(pushed.tilt_rad < 5.0 * constants::deg_to_rad);
    // drag alone (nose along +V): the specific force points aft (-x), the liquid goes to the nose ([test] 5e-5 m/s^2,
    // a low orbit's drag - clear of the tide along the tank offset)
    const auto drag = vehicle::outlet_check(*tanker, 0, lox, orbit_liquid({-5.0e-5, 0.0, 0.0}), max_tilt);
    CHECK_FALSE(drag.covered);
    CHECK(drag.tilt_rad > 90.0 * constants::deg_to_rad);
    // the tide 10 m off the compound CG (radial: stretched, ~2.5e-5 m/s^2) pulls sideways: settled (Bo ~ 50) but away
    // from the outlet
    auto docked = orbit_liquid({});
    docked.cg_offset_body_m = {0.0, 0.0, 10.0};
    const auto tide = vehicle::outlet_check(*tanker, 0, lox, docked, max_tilt);
    CHECK(tide.bond_number > 10.0);
    CHECK_FALSE(tide.covered);
    // an empty tank has nothing at its outlet
    tanker->set_tank_mass(0, "lox_main", 0.0);
    CHECK_FALSE(vehicle::outlet_check(*tanker, 0, lox, orbit_liquid({9.80665e-4, 0.0, 0.0}), max_tilt).covered);
}

TEST_CASE("The compound CG offset moves the tide and the rotation, not the specific force",
          "[refuelling][prop_transfer][boil_off]") {
    auto liquid = orbit_liquid({1.0e-3, 0.0, 0.0});
    const math::Vec3 g0 = vehicle::apparent_gravity_body_mps2(liquid, {0.0, 0.0, 10.0});
    liquid.cg_offset_body_m = {0.0, 0.0, 4.0};
    const math::Vec3 g1 = vehicle::apparent_gravity_body_mps2(liquid, {0.0, 0.0, 6.0});
    CHECK(norm(g1 - g0) < 1e-15);  // the same point seen from the reference
}
```

(Use the repo's `math::Quat::identity()` spelling; if the identity factory has another name, use it - check
`core/include/starsim/math/quat.hpp`.)

- [ ] **Step 2: Run the tests to verify they fail**

Build: compile error (`propellant_transfer.hpp` missing, `cg_offset_body_m` unknown).

- [ ] **Step 3: Implement**

`boil_off.hpp`, `LiquidEnvironment`:

```cpp
struct LiquidEnvironment {
    math::Vec3 specific_force_body_mps2;
    math::Vec3 angular_velocity_body_radps;
    math::Vec3 position_gcrf_m;    // of the free-falling reference point (the body's CG, or a docked pair's compound CG)
    math::Quat body_to_gcrf;
    math::Vec3 cg_offset_body_m;   // refuelling r4: this body's CG from the reference point (docked: from the compound CG)
};
```

`boil_off.cpp`, `apparent_gravity_body_mps2`: replace `offset_body_m` by `const Vec3 offset = liquid.cg_offset_body_m +
offset_body_m;` in the tide (`d`) and centrifugal terms; update the header comment ("offset from the CG; tide and
rotation act about the reference point").

`core/include/starsim/vehicle/propellant_transfer.hpp`:

```cpp
#pragma once

#include <cstddef>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/vehicle/boil_off.hpp"

namespace starsim::vehicle {

class Vehicle;

// Ship-to-ship propellant transfer physics (refuelling r4).

// Pressure-driven line flow: mdot = C_d A sqrt(2 rho dp) (orifice law; the valves close against back-flow: 0 for
// dp <= 0). The hydrostatic head under the settling push (~0.2 kPa over 20 m of LOX at 1e-4 g) is left out.
double line_mass_flow_kgps(double discharge_area_m2, double density_kg_m3, double pressure_difference_pa);

// C_d A of two coupled connectors: the smaller one governs.
double pair_discharge_area_m2(const config::TransferConnectorConfig& a, const config::TransferConnectorConfig& b);

// Where a tank's liquid lies relative to its outlet (the aft dome's low point, body -x).
struct OutletCheck {
    bool covered = false;
    double bond_number = 0.0;  // rho |a| r^2 / sigma at the tank centre
    double tilt_rad = 0.0;     // between the apparent gravity and -x
};

// The apparent gravity at the tank centre (on the axis, mid-way along its full-load length) settles the liquid when
// settled_weight(Bo) reaches 1 (Bo >= 10, r2); the outlet is covered when it is settled, within `max_tilt_rad` of -x
// and the tank holds liquid. A propellant without a surface tension counts as settled.
OutletCheck outlet_check(const Vehicle& vehicle, size_t stage_index, size_t tank_index, const LiquidEnvironment& liquid,
                         double max_tilt_rad);

// Time for liquid at rest to cross a tank of `length_m` under `acceleration_mps2`: sqrt(2 L / a) [E] (kinematic, no
// slosh or drainage); infinite without acceleration.
double settling_time_s(double length_m, double acceleration_mps2);

}  // namespace starsim::vehicle
```

`core/src/vehicle/propellant_transfer.cpp`:

```cpp
#include "starsim/vehicle/propellant_transfer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "starsim/vehicle/vehicle.hpp"

namespace starsim::vehicle {

using math::Vec3;

double line_mass_flow_kgps(double discharge_area_m2, double density_kg_m3, double pressure_difference_pa) {
    if (!(pressure_difference_pa > 0.0)) return 0.0;
    return discharge_area_m2 * std::sqrt(2.0 * density_kg_m3 * pressure_difference_pa);
}

double pair_discharge_area_m2(const config::TransferConnectorConfig& a, const config::TransferConnectorConfig& b) {
    const auto cda = [](const config::TransferConnectorConfig& c) {
        return c.discharge_coefficient * std::numbers::pi / 4.0 * c.line_diameter_m * c.line_diameter_m;
    };
    return std::min(cda(a), cda(b));
}

OutletCheck outlet_check(const Vehicle& vehicle, size_t stage_index, size_t tank_index, const LiquidEnvironment& liquid,
                         double max_tilt_rad) {
    const auto& stage = vehicle.stages().at(stage_index);
    const auto& tank = stage.tanks.at(tank_index);
    OutletCheck out;
    const double centre_station_m = stage.base_station_m + tank.config.base_station_m + 0.5 * tank.length_m;
    const Vec3 offset = Vec3{centre_station_m, 0.0, 0.0} - vehicle.mass_properties().cg_point_m();
    const Vec3 g = apparent_gravity_body_mps2(liquid, offset);
    const double a = norm(g);
    if (!(a > 0.0) || !(tank.mass_kg > 0.0)) return out;
    out.tilt_rad = std::acos(std::clamp(-g.x / a, -1.0, 1.0));
    out.bond_number = tank.surface_tension_n_m > 0.0
                          ? tank.density_kg_m3 * a * tank.radius_m * tank.radius_m / tank.surface_tension_n_m
                          : std::numeric_limits<double>::infinity();
    out.covered = settled_weight(out.bond_number) >= 1.0 && out.tilt_rad <= max_tilt_rad;
    return out;
}

double settling_time_s(double length_m, double acceleration_mps2) {
    if (!(acceleration_mps2 > 0.0)) return std::numeric_limits<double>::infinity();
    return std::sqrt(2.0 * length_m / acceleration_mps2);
}

}  // namespace starsim::vehicle
```

Add the source to `core/CMakeLists.txt`.

- [ ] **Step 4: Run the tests to verify they pass**

Build, then `ctest --test-dir build -L "refuelling|boil_off|settling" -j 24 --output-on-failure > tmp/r4_t2.txt 2>&1`
(the `boil_off` cases cover the unchanged r2 callers). Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/vehicle/boil_off.hpp core/src/vehicle/boil_off.cpp core/include/starsim/vehicle/propellant_transfer.hpp core/src/vehicle/propellant_transfer.cpp core/CMakeLists.txt core/tests/test_refuelling_transfer.cpp
git commit -m "feat(vehicle): transfer line flow, outlet coverage by the apparent gravity, settling time"
git push origin main
```

---

### Task 3: The transfer state machine

**Files:**
- Create: `core/include/starsim/sim/propellant_transfer.hpp`, `core/src/sim/propellant_transfer.cpp`
- Modify: `core/CMakeLists.txt` (add `src/sim/propellant_transfer.cpp` after `src/sim/mission_start.cpp`)
- Test: `core/tests/test_refuelling_transfer.cpp`

**Interfaces:**
- Consumes: `vehicle::line_mass_flow_kgps` (Task 2).
- Produces (namespace `starsim::sim`):

```cpp
enum class TransferDirection { ChaserToStation, StationToChaser };
enum class TransferPhase { Idle, Settling, Chilldown, Flowing, Paused, Complete, Stopped };
inline constexpr size_t transfer_oxidizer = 0;  // line indices
inline constexpr size_t transfer_fuel = 1;
struct TransferRequest;    // see Step 3
struct TransferLine;       // C_d A, density, chill-down of a coupled pair
struct TransferTanks;      // delivering / receiving masses and receiving capacities per line
struct TransferStepInput;  // dt, tanks, settled, thrust_available, settle_time_s, pressure_difference_pa
struct TransferStepOutput; // moved_kg, vented_kg per line, thrust
struct TransferState;      // phase, German status, elapsed, delivered, flow, chill-down, gas, boil-off, thrust
class PropellantTransfer {
public:
    PropellantTransfer(std::array<TransferLine, 2> lines, TransferRequest request, const TransferTanks& tanks);
    TransferStepOutput step(const TransferStepInput& in);
    void stop();
    void add_settling(double gas_kg, double thrust_n);
    void add_boil_off(double kg);
    bool active() const;  // Settling, Chilldown, Flowing or Paused: the valves armed, the settling thrust on
    const TransferState& state() const;
    const TransferRequest& request() const;
    static bool any_line_open(const TransferRequest& request, const TransferTanks& tanks);
};
```

- [ ] **Step 1: Write the failing tests**

Append (include `starsim/sim/propellant_transfer.hpp`):

```cpp
namespace {

sim::TransferLine lox_line() { return {0.6 * std::numbers::pi / 4.0 * 0.15 * 0.15, 1200.0, 300.0}; }
sim::TransferLine ch4_line() { return {0.6 * std::numbers::pi / 4.0 * 0.15 * 0.15, 440.0, 120.0}; }

// A pure run: the tanks move as the machine says, settled when `settled(t)`.
struct Bench {
    sim::TransferTanks tanks{{156.5e3, 43.5e3}, {469.6e3, 130.4e3}, {1224.8e3, 340.2e3}};
    sim::TransferRequest request;
    std::optional<sim::PropellantTransfer> transfer;
    Bench() { request.delivering_floor_kg = {10.0e3, 2.8e3}; }
    void start() { transfer.emplace(std::array{lox_line(), ch4_line()}, request, tanks); }
    template <class Settled>
    void run(double limit_s, Settled settled, bool thrust_available = true) {
        for (double t = 0.0; t < limit_s && transfer->active(); t += request.step_s) {
            sim::TransferStepInput in;
            in.dt_s = request.step_s;
            in.tanks = tanks;
            in.settled = settled(t);
            in.thrust_available = thrust_available;
            in.settle_time_s = 185.65;
            in.pressure_difference_pa = 2.0e5;
            const auto out = transfer->step(in);
            for (size_t k = 0; k < 2; ++k) {
                tanks.delivering_kg[k] -= out.moved_kg[k] + out.vented_kg[k];
                tanks.receiving_kg[k] += out.moved_kg[k];
            }
        }
    }
};

}  // namespace

TEST_CASE("Transfer phases: settling, chill-down, flow to the floors, complete", "[refuelling][prop_transfer]") {
    Bench b;
    b.start();
    CHECK(b.transfer->state().phase == sim::TransferPhase::Settling);
    CHECK(b.transfer->active());
    b.run(180.0, [](double) { return true; });
    CHECK(b.transfer->state().phase == sim::TransferPhase::Settling);  // 185.65 s not yet reached
    b.run(10.0, [](double) { return true; });
    CHECK(b.transfer->state().phase == sim::TransferPhase::Chilldown);
    b.run(120.0, [](double) { return true; });
    CHECK(b.transfer->state().phase == sim::TransferPhase::Flowing);
    CHECK(b.transfer->state().chilldown_kg[0] == Catch::Approx(300.0));
    CHECK(b.transfer->state().chilldown_kg[1] == Catch::Approx(120.0));
    b.run(3600.0, [](double) { return true; });
    const auto& s = b.transfer->state();
    CHECK(s.phase == sim::TransferPhase::Complete);
    CHECK(s.status == "Transfer abgeschlossen");
    CHECK_FALSE(b.transfer->active());
    CHECK(b.tanks.delivering_kg[0] == Catch::Approx(10.0e3).margin(1e-6));
    CHECK(b.tanks.delivering_kg[1] == Catch::Approx(2.8e3).margin(1e-6));
    CHECK(s.delivered_kg[0] == Catch::Approx(156.5e3 - 300.0 - 10.0e3));
    CHECK(s.delivered_kg[1] == Catch::Approx(43.5e3 - 120.0 - 2.8e3));
    // LOX at 232.3 kg/s: 146.2 t in ~630 s of flow after 190 + 120 s
    CHECK(s.elapsed_s == Catch::Approx(190.0 + 120.0 + 630.0).margin(10.0));
}

TEST_CASE("A full receiver or a limit closes a line; nothing to move opens none", "[refuelling][prop_transfer]") {
    Bench b;
    b.tanks.receiving_kg[1] = b.tanks.receiving_capacity_kg[1] - 1000.0;  // the depot's CH4 nearly full
    b.request.limit_kg[0] = 20.0e3;
    b.start();
    b.run(3600.0, [](double) { return true; });
    CHECK(b.transfer->state().phase == sim::TransferPhase::Complete);
    CHECK(b.transfer->state().delivered_kg[0] == Catch::Approx(20.0e3));
    CHECK(b.tanks.receiving_kg[1] == Catch::Approx(b.tanks.receiving_capacity_kg[1]));
    sim::TransferTanks done = b.tanks;
    done.delivering_kg = {10.0e3, 2.8e3};  // both at their floors
    CHECK_FALSE(sim::PropellantTransfer::any_line_open(b.request, done));
}

TEST_CASE("An uncovered outlet pauses the transfer and a covered one resumes it", "[refuelling][prop_transfer]") {
    Bench b;
    b.start();
    b.run(400.0, [](double) { return true; });  // flowing
    REQUIRE(b.transfer->state().phase == sim::TransferPhase::Flowing);
    const double before = b.transfer->state().delivered_kg[0];
    b.run(50.0, [](double) { return false; });
    CHECK(b.transfer->state().phase == sim::TransferPhase::Paused);
    CHECK(b.transfer->state().status == "Transfer unterbrochen: Treibstoff nicht am Auslass");
    CHECK(b.transfer->state().delivered_kg[0] == before);  // nothing flows unsettled, not even in the pausing step
    b.run(10.0, [](double) { return true; });
    CHECK(b.transfer->state().phase == sim::TransferPhase::Flowing);
}

TEST_CASE("Settling that never covers the outlets pauses; no thrust pauses with its own reason",
          "[refuelling][prop_transfer]") {
    {
        Bench b;
        b.start();
        b.run(200.0, [](double) { return false; });
        CHECK(b.transfer->state().phase == sim::TransferPhase::Paused);
        CHECK(b.transfer->state().delivered_kg[0] == 0.0);
    }
    {
        Bench b;
        b.start();
        b.run(30.0, [](double) { return false; }, false);
        CHECK(b.transfer->state().phase == sim::TransferPhase::Paused);
        CHECK(b.transfer->state().status == "Transfer unterbrochen: kein Kaltgas zum Setzen");
    }
}

TEST_CASE("Stop closes the valves", "[refuelling][prop_transfer]") {
    Bench b;
    b.start();
    b.run(400.0, [](double) { return true; });
    b.transfer->stop();
    CHECK(b.transfer->state().phase == sim::TransferPhase::Stopped);
    CHECK(b.transfer->state().status == "Transfer gestoppt");
    CHECK_FALSE(b.transfer->active());
    const auto out = b.transfer->step({});
    CHECK(out.moved_kg[0] == 0.0);
    CHECK_FALSE(out.thrust);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Build: compile error (`sim/propellant_transfer.hpp` missing).

- [ ] **Step 3: Implement**

`core/include/starsim/sim/propellant_transfer.hpp`:

```cpp
#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <string>

#include "starsim/math/constants.hpp"

namespace starsim::sim {

// Ship-to-ship propellant transfer (refuelling r4): the state machine of one transfer between a docked pair, fed by the
// session with tank masses and whether the outlets are covered. Settling (the settling push runs for the settling
// time) -> Chilldown (the lines are chilled, the propellant vented) -> Flowing (pressure-driven, per line until the
// delivering tank reaches its floor, the receiving tank is full or the line's limit is delivered) -> Complete. An
// uncovered outlet pauses Chilldown / Flowing (and ends a Settling that did not settle); a covered one resumes.
enum class TransferDirection { ChaserToStation, StationToChaser };
enum class TransferPhase { Idle, Settling, Chilldown, Flowing, Paused, Complete, Stopped };
inline constexpr size_t transfer_oxidizer = 0;  // line indices
inline constexpr size_t transfer_fuel = 1;

struct TransferRequest {
    TransferDirection direction = TransferDirection::ChaserToStation;
    std::array<double, 2> delivering_floor_kg{0.0, 0.0};  // the delivering main tanks keep this (the tanker reserve)
    std::array<double, 2> limit_kg{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    double settling_acceleration_mps2 = 1.0e-4 * constants::standard_gravity_mps2;  // [E] spec r4
    double max_settled_tilt_rad = 30.0 * constants::deg_to_rad;  // [E] the aft dome's outlet stays covered
    double chilldown_s = 120.0;                                  // [E]
    double step_s = 10.0;                                        // [E] update cadence (flow, thrust, boil-off)
};

struct TransferLine {
    double discharge_area_m2 = 0.0;  // C_d A of the coupled pair
    double density_kg_m3 = 0.0;      // of the delivered liquid
    double chilldown_kg = 0.0;       // both connectors' together, vented from the delivering tank
};

struct TransferTanks {
    std::array<double, 2> delivering_kg{};  // main tanks of the delivering ship
    std::array<double, 2> receiving_kg{};
    std::array<double, 2> receiving_capacity_kg{};
};

struct TransferStepInput {
    double dt_s = 0.0;
    TransferTanks tanks;
    bool settled = false;           // every involved tank's outlet covered (vehicle::outlet_check)
    bool thrust_available = false;  // the chaser can push (+x capability and cold gas left)
    double settle_time_s = 0.0;     // the longest involved tank's settling time at the achieved push
    double pressure_difference_pa = 0.0;
};

struct TransferStepOutput {
    std::array<double, 2> moved_kg{};   // from the delivering to the receiving tank
    std::array<double, 2> vented_kg{};  // chill-down, from the delivering tank
    bool thrust = false;                // the settling push stays on
};

struct TransferState {
    TransferPhase phase = TransferPhase::Idle;
    std::string status;  // German (UI)
    double elapsed_s = 0.0;
    std::array<double, 2> delivered_kg{};
    std::array<double, 2> flow_kgps{};     // of the last step
    std::array<double, 2> chilldown_kg{};  // vented
    std::array<bool, 2> line_open{};
    double settling_gas_kg = 0.0;
    double settling_thrust_n = 0.0;  // of the last step
    double boil_off_kg = 0.0;        // both ships, since the start
};

class PropellantTransfer {
public:
    PropellantTransfer(std::array<TransferLine, 2> lines, TransferRequest request, const TransferTanks& tanks);
    TransferStepOutput step(const TransferStepInput& in);
    void stop();
    void add_settling(double gas_kg, double thrust_n);
    void add_boil_off(double kg) { state_.boil_off_kg += kg; }
    bool active() const;
    const TransferState& state() const { return state_; }
    const TransferRequest& request() const { return request_; }
    // Whether a line can move anything (the session refuses a start without one).
    static bool any_line_open(const TransferRequest& request, const TransferTanks& tanks);

private:
    void pause(const TransferStepInput& in);
    void close_lines(const TransferTanks& tanks);

    std::array<TransferLine, 2> lines_;
    TransferRequest request_;
    TransferState state_;
    TransferPhase paused_from_ = TransferPhase::Settling;
    double phase_elapsed_s_ = 0.0;
};

}  // namespace starsim::sim
```

`core/src/sim/propellant_transfer.cpp`:

```cpp
#include "starsim/sim/propellant_transfer.hpp"

#include <algorithm>

#include "starsim/vehicle/propellant_transfer.hpp"

namespace starsim::sim {

namespace {

constexpr double closed_margin_kg = 1e-3;  // a line within a gram of its end is closed (rounding of the tank sums)

bool line_open(const TransferRequest& r, const TransferTanks& t, double delivered_kg, size_t k) {
    return t.delivering_kg[k] > r.delivering_floor_kg[k] + closed_margin_kg &&
           t.receiving_kg[k] < t.receiving_capacity_kg[k] - closed_margin_kg &&
           delivered_kg < r.limit_kg[k] - closed_margin_kg;
}

}  // namespace

bool PropellantTransfer::any_line_open(const TransferRequest& request, const TransferTanks& tanks) {
    return line_open(request, tanks, 0.0, transfer_oxidizer) || line_open(request, tanks, 0.0, transfer_fuel);
}

PropellantTransfer::PropellantTransfer(std::array<TransferLine, 2> lines, TransferRequest request,
                                       const TransferTanks& tanks)
    : lines_(lines), request_(request) {
    for (size_t k = 0; k < 2; ++k) state_.line_open[k] = line_open(request_, tanks, 0.0, k);
    state_.phase = TransferPhase::Settling;
    state_.status = "Treibstoff wird gesetzt";
}

bool PropellantTransfer::active() const {
    return state_.phase == TransferPhase::Settling || state_.phase == TransferPhase::Chilldown ||
           state_.phase == TransferPhase::Flowing || state_.phase == TransferPhase::Paused;
}

void PropellantTransfer::stop() {
    if (!active()) return;
    state_.phase = TransferPhase::Stopped;
    state_.status = "Transfer gestoppt";
    state_.flow_kgps = {};
}

void PropellantTransfer::add_settling(double gas_kg, double thrust_n) {
    state_.settling_gas_kg += gas_kg;
    state_.settling_thrust_n = thrust_n;
}

void PropellantTransfer::pause(const TransferStepInput& in) {
    if (state_.phase != TransferPhase::Paused) paused_from_ = state_.phase == TransferPhase::Flowing
                                                                  ? TransferPhase::Flowing
                                                                  : TransferPhase::Chilldown;
    state_.phase = TransferPhase::Paused;
    state_.status = in.thrust_available ? "Transfer unterbrochen: Treibstoff nicht am Auslass"
                                        : "Transfer unterbrochen: kein Kaltgas zum Setzen";
    state_.flow_kgps = {};
}

void PropellantTransfer::close_lines(const TransferTanks& tanks) {
    for (size_t k = 0; k < 2; ++k) {
        state_.line_open[k] = state_.line_open[k] && line_open(request_, tanks, state_.delivered_kg[k], k);
    }
    if (!state_.line_open[0] && !state_.line_open[1]) {
        state_.phase = TransferPhase::Complete;
        state_.status = "Transfer abgeschlossen";
        state_.flow_kgps = {};
    }
}

TransferStepOutput PropellantTransfer::step(const TransferStepInput& in) {
    TransferStepOutput out;
    if (!active()) return out;
    state_.elapsed_s += in.dt_s;
    phase_elapsed_s_ += in.dt_s;
    state_.flow_kgps = {};
    switch (state_.phase) {
    case TransferPhase::Settling:
        if (!in.thrust_available) {
            pause(in);
        } else if (phase_elapsed_s_ >= in.settle_time_s) {
            if (in.settled) {
                state_.phase = TransferPhase::Chilldown;
                state_.status = "Leitungen werden gekühlt";
                phase_elapsed_s_ = 0.0;
            } else {
                pause(in);
            }
        }
        break;
    case TransferPhase::Paused:
        if (in.settled && in.thrust_available) {
            state_.phase = paused_from_;
            state_.status = paused_from_ == TransferPhase::Flowing ? "Transfer läuft" : "Leitungen werden gekühlt";
            if (paused_from_ == TransferPhase::Chilldown) phase_elapsed_s_ = 0.0;  // chilled again from the start
        }
        break;
    case TransferPhase::Chilldown:
        if (!in.settled || !in.thrust_available) {
            pause(in);
            break;
        }
        for (size_t k = 0; k < 2; ++k) {
            if (!state_.line_open[k]) continue;
            const double left = lines_[k].chilldown_kg - state_.chilldown_kg[k];
            const double room = in.tanks.delivering_kg[k] - request_.delivering_floor_kg[k];
            const double vent = std::clamp(
                std::min(left, lines_[k].chilldown_kg * in.dt_s / request_.chilldown_s), 0.0, std::max(room, 0.0));
            out.vented_kg[k] = vent;
            state_.chilldown_kg[k] += vent;
        }
        if (phase_elapsed_s_ >= request_.chilldown_s - 1e-9) {
            state_.phase = TransferPhase::Flowing;
            state_.status = "Transfer läuft";
            phase_elapsed_s_ = 0.0;
        }
        break;
    case TransferPhase::Flowing: {
        if (!in.settled || !in.thrust_available) {
            pause(in);
            break;
        }
        TransferTanks after = in.tanks;
        for (size_t k = 0; k < 2; ++k) {
            if (!state_.line_open[k]) continue;
            const double flow = vehicle::line_mass_flow_kgps(lines_[k].discharge_area_m2, lines_[k].density_kg_m3,
                                                             in.pressure_difference_pa);
            const double moved = std::max(
                0.0, std::min({flow * in.dt_s, in.tanks.delivering_kg[k] - request_.delivering_floor_kg[k],
                               in.tanks.receiving_capacity_kg[k] - in.tanks.receiving_kg[k],
                               request_.limit_kg[k] - state_.delivered_kg[k]}));
            out.moved_kg[k] = moved;
            state_.delivered_kg[k] += moved;
            state_.flow_kgps[k] = moved / in.dt_s;
            after.delivering_kg[k] -= moved;
            after.receiving_kg[k] += moved;
        }
        close_lines(after);
        break;
    }
    default:
        break;
    }
    out.thrust = active();
    return out;
}

}  // namespace starsim::sim
```

(Check that `constants::standard_gravity_mps2` and `constants::deg_to_rad` are the names in `constants.hpp`; both are
used elsewhere in the core.) Note the `paused_from_` rule: a pause during Settling resumes into Chilldown (the settling
time has passed or the thrust returned - the outlets are covered when it resumes).

- [ ] **Step 4: Run the tests to verify they pass**

Build, `ctest --test-dir build -L "prop_transfer" -j 24 --output-on-failure > tmp/r4_t3.txt 2>&1`. Expected: pass.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/sim/propellant_transfer.hpp core/src/sim/propellant_transfer.cpp core/CMakeLists.txt core/tests/test_refuelling_transfer.cpp
git commit -m "feat(sim): propellant transfer state machine - settling, chill-down, flow to floor or full, pause"
git push origin main
```

---

### Task 4: Docked boil-off through the compound

**Files:**
- Modify: `core/include/starsim/vehicle/flight_body.hpp:199-206` (comment, `boil_off_rates_now`, `set_boil_off_rate`,
  `liquid_specific_force_body_mps2`), `core/src/vehicle/flight_body.cpp:343-376` (skip while carrying a visitor; factor
  the rate evaluation)
- Modify: `core/include/starsim/sim/flight_session.hpp:1068-1108` (docking members), `core/src/sim/flight_session.cpp`
  (`advance_to`: call `docked_update` after every coast and every tick)
- Create: `core/src/sim/flight_session_transfer.cpp` (the docked update; Task 5 adds the transfer); Modify:
  `core/CMakeLists.txt`
- Modify: `core/src/sim/flight_session_docking.cpp` (`merge_docked`, `detach_compound`: reset the docked boil-off state)
- Test: `core/tests/test_refuelling_transfer.cpp` (includes `refuelling_test_support.hpp`)

**Interfaces:**
- Consumes: `LiquidEnvironment::cg_offset_body_m` (Task 2); `FlightSession::set_tank_mass` (r3);
  `vehicle::boil_off_rates`, `vehicle::total_boil_off` (r2).
- Produces:
  - `std::vector<vehicle::TankBoilOff> FlightBody::boil_off_rates_now(const FlightEnvironment& env, time::Epoch epoch, const LiquidEnvironment& liquid) const`
    - the rates now (empty with the flag off or without an ephemeris); not applied.
  - `void FlightBody::set_boil_off_rate(const BoilOffRate& rate)` - the session's docked evaluation shows in
    `boil_off_rate()`.
  - `const math::Vec3& FlightBody::liquid_specific_force_body_mps2() const` - the specific force of the last tick or at
    the end of the last coast (r2's `boil_off_specific_force_body_mps2_`).
  - `vehicle::LiquidEnvironment FlightSession::docked_liquid(size_t body_index, const math::Vec3& push_station_mps2) const`
    (private) - the compound's liquid environment in that body's axes; `push_station_mps2` = extra specific force
    (the settling push) in station body axes.
  - `void FlightSession::docked_update(double dt_s)` (private) - called after every coast and tick; every
    `FlightBody::boil_off_interval_s` (and after a coast) while docked: both bodies' boil-off vented through
    `set_tank_mass` (trapezoid of the last two evaluations); Task 5 hangs the transfer step into it.

- [ ] **Step 1: Write the failing tests**

Append (includes `refuelling_test_support.hpp`, `starsim/vehicle/docking_port.hpp`, `test_support.hpp` or whatever
provides `test::spice_session()` in `test_refuelling_docking.cpp` - copy its includes):

```cpp
namespace {

double main_kg(const vehicle::FlightBody& body) { return body.vehicle().main_propellant_kg(); }

double port_gap_m(const sim::FlightSession& s) {
    return norm(vehicle::port_kinematics(s.body(0), "transfer_dock").position_gcrf_m -
                vehicle::port_kinematics(s.body(1), "transfer_dock").position_gcrf_m);
}

}  // namespace

TEST_CASE("Docked boil-off: both ships vent through the compound, the hulls stay mated",
          "[refuelling][prop_transfer][boil_off][docking]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0});
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    REQUIRE(f.session.docked());
    const double gap0 = port_gap_m(f.session);
    const double tanker0 = main_kg(f.session.body(0));
    const double depot0 = main_kg(f.session.body(1));
    f.session.advance_to(f.session.epoch() + 2.0 * 3600.0, 100000);
    const double tanker_loss = tanker0 - main_kg(f.session.body(0));
    const double depot_loss = depot0 - main_kg(f.session.body(1));
    INFO("tanker " << tanker_loss << " kg, depot " << depot_loss << " kg in 2 h");
    CHECK(tanker_loss > 100.0);                  // bare stainless: tonnes per day
    CHECK(depot_loss > 10.0);                    // r2: ~450 kg/day full -> ~37 kg in 2 h (600 t: less wall wetted)
    CHECK(depot_loss < 100.0);
    CHECK(std::abs(port_gap_m(f.session) - gap0) < 1e-5);  // r3 M2: the depot no longer vents past the compound
    CHECK(f.session.compound_mass()->mass_kg ==
          Catch::Approx(f.session.body(1).vehicle().mass_properties().mass_kg));
    CHECK(f.session.body(0).boil_off_rate().oxidizer_kgps > 0.0);  // shown for the held tanker too
}

TEST_CASE("Docked boil-off follows the fidelity flag", "[refuelling][prop_transfer][boil_off][docking]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0});
    f.session.set_boil_off(false);
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    const double tanker0 = main_kg(f.session.body(0));
    const double depot0 = main_kg(f.session.body(1));
    f.session.advance_to(f.session.epoch() + 3600.0, 100000);
    CHECK(main_kg(f.session.body(0)) == tanker0);
    CHECK(main_kg(f.session.body(1)) == depot0);
}
```


- [ ] **Step 2: Run the tests to verify they fail**

Build, run `starsim_core_tests.exe "Docked boil-off: both ships vent through the compound, the hulls stay mated"`.
Expected: FAIL - the tanker loses nothing (held docked: no boil-off) and/or the port gap drifts (the depot vents past
the compound).

- [ ] **Step 3: Implement**

`flight_body.hpp` (replace the comment's last sentence and add the accessors next to `boil_off_rate()`):

```cpp
    // Boil-off (refuelling r2): the heat balance is evaluated every boil_off_interval_s of free flight and after every
    // coast; the vented mass is the trapezoid of the last two evaluations' flows per tank. Below
    // boil_off_min_altitude_m in air (convection and entry heating are not modelled), clamped, caught, landed, resting,
    // docked or carrying a docked visitor nothing boils off here: a docked pair is vented by the session in the
    // compound's environment (refuelling r4, FlightSession::set_tank_mass keeps the compound CG).
    static constexpr double boil_off_interval_s = 10.0;         // [E]
    static constexpr double boil_off_min_altitude_m = 100.0e3;  // [E]
    const BoilOffRate& boil_off_rate() const { return boil_off_rate_; }  // at the last evaluation
    void set_boil_off_rate(const BoilOffRate& rate) { boil_off_rate_ = rate; }
    // The rates now in `liquid` (not applied; empty with the flag off or without an ephemeris).
    std::vector<TankBoilOff> boil_off_rates_now(const FlightEnvironment& env, time::Epoch epoch,
                                                const LiquidEnvironment& liquid) const;
    // The specific force of the last tick or at the end of the last coast (moves the liquid).
    const math::Vec3& liquid_specific_force_body_mps2() const { return boil_off_specific_force_body_mps2_; }
```

`flight_body.cpp`: at the top of `boil_off`:

```cpp
    if (docked_ || vehicle_->attached_mass()) {  // a docked pair is vented by the session (refuelling r4)
        reset_boil_off();
        return;
    }
```

and factor the sun / moon / `boil_off_rates` call into `boil_off_rates_now` (the altitude check stays in `boil_off`):

```cpp
std::vector<TankBoilOff> FlightBody::boil_off_rates_now(const FlightEnvironment& env, time::Epoch epoch,
                                                        const LiquidEnvironment& liquid) const {
    if (!env.boil_off || env.ephemeris == nullptr) return {};
    const Vec3 sun = env.ephemeris->state_gcrf(ephemeris::Body::Sun, epoch).position_m - state_.position_gcrf_m;
    const Vec3 moon = env.ephemeris->state_gcrf(ephemeris::Body::Moon, epoch).position_m - state_.position_gcrf_m;
    return boil_off_rates(*vehicle_, environment::earth_orbit_radiation(state_.position_gcrf_m, sun, &moon),
                          state_.attitude, &liquid);
}
```

`boil_off` then calls `boil_off_rates_now(env, epoch, LiquidEnvironment{boil_off_specific_force_body_mps2_,
state_.angular_velocity_body_radps, state_.position_gcrf_m, state_.attitude, {}})`.

`flight_session.hpp`, beside the docking members:

```cpp
    // Docked update (refuelling r4, flight_session_transfer.cpp): a docked pair's boil-off (both bodies, in the
    // compound's environment, through set_tank_mass) every FlightBody::boil_off_interval_s and after every coast, and
    // the propellant transfer's steps.
    double docked_elapsed_s_ = 0.0;
    std::array<std::vector<vehicle::TankBoilOff>, 2> docked_boil_off_last_;  // chaser, station
    vehicle::LiquidEnvironment docked_liquid(size_t body_index, const math::Vec3& push_station_mps2) const;
    double vent_docked_boil_off(double dt_s, const math::Vec3& push_station_mps2);  // returns the vented kg
    void docked_update(double dt_s, bool after_coast);
    void reset_docked_update();
```

`flight_session_transfer.cpp`:

```cpp
// Refuelling r4: a docked pair's boil-off and the propellant transfer between its ships.

#include <algorithm>

#include "starsim/sim/flight_session.hpp"
#include "starsim/vehicle/boil_off.hpp"

namespace starsim::sim {

using math::Vec3;

vehicle::LiquidEnvironment FlightSession::docked_liquid(size_t body_index, const Vec3& push_station_mps2) const {
    const auto& station = *bodies_[*station_index_].body;
    const auto& ss = station.state();
    // the compound's specific force (drag of the station body, which carries the compound's mass, plus the push)
    const Vec3 specific_station = station.liquid_specific_force_body_mps2() + push_station_mps2;
    if (body_index == *station_index_) {  // the station vehicle's CG is the compound CG (attached mass included)
        return {specific_station, ss.angular_velocity_body_radps, ss.position_gcrf_m, ss.attitude, {}};
    }
    const auto& chaser = *bodies_[body_index].body;
    return {docked_attitude_station_.inverse_rotate(specific_station),
            chaser.state().angular_velocity_body_radps, ss.position_gcrf_m, chaser.state().attitude,
            docked_attitude_station_.inverse_rotate(docked_position_station_m_)};
}

double FlightSession::vent_docked_boil_off(double dt_s, const Vec3& push_station_mps2) {
    double vented = 0.0;
    const std::array<size_t, 2> index{controlled_, *station_index_};
    for (size_t b = 0; b < 2; ++b) {
        auto& body = *bodies_[index[b]].body;
        auto now = body.boil_off_rates_now(env_, epoch_, docked_liquid(index[b], push_station_mps2));
        for (const auto& tank : now) {
            double flow = tank.mass_flow_kgps;
            for (const auto& before : docked_boil_off_last_[b]) {
                if (before.stage_index == tank.stage_index && before.tank_index == tank.tank_index) {
                    flow = 0.5 * (flow + before.mass_flow_kgps);
                }
            }
            const auto& t = body.vehicle().stages()[tank.stage_index].tanks[tank.tank_index];
            const double kg = std::min(t.mass_kg, flow * dt_s);
            if (kg <= 0.0) continue;
            set_tank_mass(index[b], tank.stage_index, t.config.id, t.mass_kg - kg);
            vented += kg;
        }
        body.set_boil_off_rate(vehicle::total_boil_off(body.vehicle(), now));
        docked_boil_off_last_[b] = std::move(now);
    }
    return vented;
}

void FlightSession::docked_update(double dt_s, bool after_coast) {
    if (!docked()) return;
    docked_elapsed_s_ += dt_s;
    if (!after_coast && docked_elapsed_s_ < vehicle::FlightBody::boil_off_interval_s - 1e-9) return;
    vent_docked_boil_off(docked_elapsed_s_, {});
    docked_elapsed_s_ = 0.0;
}

void FlightSession::reset_docked_update() {
    docked_elapsed_s_ = 0.0;
    docked_boil_off_last_ = {};
}

}  // namespace starsim::sim
```

(`set_tank_mass(body, stage, id, kg)` is r3's compound-refreshing setter; `total_boil_off` is r2's - check its exact
name and header in `boil_off.hpp`.) Note: no altitude check here - a docked pair is always in orbit.

`advance_to`: after the coast block's `sync_docked();` add `docked_update(coast, true);`, and after the tick's
`if (docking_) docking_after_tick();` add `docked_update(tick_s, false);`. `merge_docked` and `detach_compound` call
`reset_docked_update()`.

Add `src/sim/flight_session_transfer.cpp` to `core/CMakeLists.txt` after `flight_session_rendezvous.cpp`.

- [ ] **Step 4: Run the tests to verify they pass**

Build, then `ctest --test-dir build -L "refuelling|boil_off|docking|iss" -j 24 --output-on-failure > tmp/r4_t4.txt 2>&1`
(the docked path changed: the ISS pairs must be unchanged - they have no `[stage.thermal]`). This includes `[slow]`
ISS campaigns; if it may exceed 10 min run it in the background and check every 10 min. Expected: all pass; report the
2 h tanker / depot losses from the INFO line (`STARSIM_VERBOSE` not needed - run the single case with `-s` to see it).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/vehicle/flight_body.hpp core/src/vehicle/flight_body.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/src/sim/flight_session_docking.cpp core/src/sim/flight_session_transfer.cpp core/CMakeLists.txt core/tests/test_refuelling_transfer.cpp
git commit -m "feat(sim): docked pair boil-off through the compound - both ships vent in the compound's environment"
git push origin main
```

---

### Task 5: Propellant transfer in the session

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp` (public `start_transfer`, `stop_transfer`, `transfer_state`;
  private transfer members), `core/src/sim/flight_session_transfer.cpp` (the transfer), `core/src/sim/flight_session.cpp`
  (`coast_allowance_s`: the next transfer step), `core/src/sim/flight_session_docking.cpp` (`undock` refusal,
  `reset_docking` / `detach_compound` end the transfer)
- Test: `core/tests/test_refuelling_transfer.cpp`

**Interfaces:**
- Consumes: Tasks 1-4; `Vehicle::rcs_pure_force_capability`, `vehicle::port_config`.
- Produces (public `FlightSession`):

```cpp
    // Propellant transfer (refuelling r4, flight_session_transfer.cpp): between the docked pair's transfer connectors of
    // equal contents (main tanks), driven by the delivering ship's ullage pressure against the receiving ship's
    // ([stage.propellant_transfer]); the chaser's +x cold gas pushes the compound to settle the liquid (average thrust,
    // an impulse every request.step_s, gas from its main tanks). Masses and CGs follow (set_tank_mass), both ships keep
    // boiling off. False with a German transfer_state().status when refused: not docked, the autopilot active, a
    // transfer under way, no coupled connectors or no transfer pressures on either ship, nothing to move.
    bool start_transfer(const TransferRequest& request);
    void stop_transfer();
    const TransferState& transfer_state() const;  // the running or last transfer, or the refusal
```

- [ ] **Step 1: Write the failing tests**

Append:

```cpp
namespace {

struct DockedPair : refuelling_test::TankerAtPort {
    explicit DockedPair(const config::VehicleConfig& chaser = refuelling_test::tanker_config(200.0e3))
        : TankerAtPort({0.0, 0.0, 0.0}, {}, chaser) {
        session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
        session.dock_in_place();
    }
    double tank(size_t body, const std::string& id) const {
        for (const auto& k : session.body(body).vehicle().stages()[0].tanks) {
            if (k.config.id == id) return k.mass_kg;
        }
        return 0.0;
    }
    double all_propellant() const {
        double kg = 0.0;
        for (size_t b = 0; b < 2; ++b) {
            for (const auto& k : session.body(b).vehicle().stages()[0].tanks) kg += k.mass_kg;
        }
        return kg;
    }
    void run_to_end(double limit_s, double chunk_s) {
        for (double t = 0.0; t < limit_s && is_running(); t += chunk_s) {
            session.advance_to(session.epoch() + chunk_s, 1000000);
        }
    }
    bool is_running() const {
        const auto p = session.transfer_state().phase;
        return p == sim::TransferPhase::Settling || p == sim::TransferPhase::Chilldown ||
               p == sim::TransferPhase::Flowing || p == sim::TransferPhase::Paused;
    }
};

sim::TransferRequest tanker_request() {
    sim::TransferRequest r;
    r.delivering_floor_kg = {10.0e3, 2.8e3};  // [test] the reserve of Task 6 is ~this size
    return r;
}

}  // namespace

TEST_CASE("Tanker to depot: settle, chill, flow to the reserve; masses close, hulls stay mated",
          "[refuelling][prop_transfer][docking]") {
    test::spice_session();
    DockedPair f;
    REQUIRE(f.session.docked());
    const double lox0 = f.tank(0, "lox_main"), depot_lox0 = f.tank(1, "lox_main");
    const double total0 = f.all_propellant();
    const double gap0 = port_gap_m(f.session);
    const math::Vec3 v0 = f.session.body(1).state().velocity_gcrf_mps;
    REQUIRE(f.session.start_transfer(tanker_request()));
    CHECK(f.session.transfer_state().phase == sim::TransferPhase::Settling);
    f.run_to_end(3600.0, 60.0);
    const auto& s = f.session.transfer_state();
    INFO(s.status << ": " << s.elapsed_s << " s, LOX " << s.delivered_kg[0] << " CH4 " << s.delivered_kg[1]
                  << " gas " << s.settling_gas_kg << " chill " << s.chilldown_kg[0] + s.chilldown_kg[1]
                  << " boil-off " << s.boil_off_kg << " thrust " << s.settling_thrust_n);
    REQUIRE(s.phase == sim::TransferPhase::Complete);
    // the LOX line closed last at its floor; CH4 paid a little gas after its line closed
    CHECK(f.tank(0, "lox_main") == Catch::Approx(10.0e3).margin(50.0));
    CHECK(f.tank(0, "ch4_main") == Catch::Approx(2.8e3).margin(200.0));
    CHECK(f.tank(0, "ch4_main") > 2.5e3);
    // every kilogram is accounted for: delivered stays in the pair, the rest left it
    CHECK(total0 - f.all_propellant() ==
          Catch::Approx(s.settling_gas_kg + s.chilldown_kg[0] + s.chilldown_kg[1] + s.boil_off_kg).margin(1.0));
    CHECK(f.tank(1, "lox_main") - depot_lox0 > s.delivered_kg[0] - 50.0);  // minus the depot's boil-off
    CHECK(s.delivered_kg[0] > 140.0e3);
    CHECK(s.delivered_kg[0] < lox0 - 10.0e3);
    // ~190 s settling + 120 s chill-down + ~630 s LOX flow (232 kg/s)
    CHECK(s.elapsed_s == Catch::Approx(940.0).margin(40.0));
    // 1e-4 g on ~1,360 t: ~1.33 kN for ~940 s at Isp 70 s
    CHECK(s.settling_thrust_n == Catch::Approx(1.0e-4 * 9.80665 * f.session.compound_mass()->mass_kg).epsilon(0.02));
    CHECK(s.settling_gas_kg == Catch::Approx(1.8e3).epsilon(0.15));
    CHECK(std::abs(port_gap_m(f.session) - gap0) < 1e-5);
    CHECK(f.session.docked());
    (void)v0;
}

TEST_CASE("Transfer results do not depend on how the time is advanced", "[refuelling][prop_transfer][docking][slow]") {
    test::spice_session();
    DockedPair a, b;
    REQUIRE(a.session.start_transfer(tanker_request()));
    REQUIRE(b.session.start_transfer(tanker_request()));
    a.run_to_end(3600.0, 1.0);
    b.run_to_end(3600.0, 1200.0);
    REQUIRE(a.session.transfer_state().phase == sim::TransferPhase::Complete);
    REQUIRE(b.session.transfer_state().phase == sim::TransferPhase::Complete);
    for (size_t k = 0; k < 2; ++k) {
        CHECK(a.session.transfer_state().delivered_kg[k] ==
              Catch::Approx(b.session.transfer_state().delivered_kg[k]).epsilon(1e-4));
    }
    CHECK(a.session.transfer_state().elapsed_s == Catch::Approx(b.session.transfer_state().elapsed_s).margin(1e-6));
}

TEST_CASE("Depot to HLS: the limits end the transfer", "[refuelling][prop_transfer][docking][hls]") {
    test::spice_session();
    DockedPair f(refuelling_test::ship_with("starship_hls", 50.0e3));
    REQUIRE(f.session.docked());
    const double hls_lox0 = f.tank(0, "lox_main");
    sim::TransferRequest r;
    r.direction = sim::TransferDirection::StationToChaser;
    r.limit_kg = {20.0e3, 5.5e3};
    REQUIRE(f.session.start_transfer(r));
    f.run_to_end(3600.0, 60.0);
    const auto& s = f.session.transfer_state();
    REQUIRE(s.phase == sim::TransferPhase::Complete);
    CHECK(s.delivered_kg[0] == Catch::Approx(20.0e3).margin(1.0));
    CHECK(s.delivered_kg[1] == Catch::Approx(5.5e3).margin(1.0));
    // the HLS pays the settling gas from its own main tanks (it has no [stage.thermal]: no boil-off)
    CHECK(f.tank(0, "lox_main") - hls_lox0 == Catch::Approx(20.0e3 - s.settling_gas_kg * 3.6 / 4.6).epsilon(0.02));
}

TEST_CASE("Transfer refusals and stop", "[refuelling][prop_transfer][docking]") {
    test::spice_session();
    {
        refuelling_test::TankerAtPort f({20.0, 0.0, 0.0});
        CHECK_FALSE(f.session.start_transfer(tanker_request()));
        CHECK(f.session.transfer_state().status == "Transfer nicht möglich: nicht angedockt");
    }
    {
        DockedPair f(refuelling_test::tanker_config(10.0e3));  // 7.8 t LOX / 2.2 t CH4: below the floors
        CHECK_FALSE(f.session.start_transfer(tanker_request()));
        CHECK(f.session.transfer_state().status == "Transfer nicht möglich: nichts zu übertragen");
    }
    {
        DockedPair f;
        REQUIRE(f.session.start_transfer(tanker_request()));
        CHECK_FALSE(f.session.start_transfer(tanker_request()));
        CHECK(f.session.docking_status() == "Transfer läuft bereits");
        CHECK(f.session.transfer_state().phase == sim::TransferPhase::Settling);  // the running transfer is kept
        CHECK_FALSE(f.session.undock());
        CHECK(f.session.docking_status() == "Abkoppeln nicht möglich: Treibstofftransfer läuft");
        f.session.advance_to(f.session.epoch() + 400.0, 1000000);
        REQUIRE(f.session.transfer_state().phase == sim::TransferPhase::Flowing);
        f.session.stop_transfer();
        CHECK(f.session.transfer_state().phase == sim::TransferPhase::Stopped);
        const double lox = f.tank(0, "lox_main");
        const math::Vec3 v = f.session.body(1).state().velocity_gcrf_mps;
        f.session.advance_to(f.session.epoch() + 60.0, 1000000);
        CHECK(f.tank(0, "lox_main") > lox - 50.0);  // only boil-off, no flow, no gas
        CHECK(f.session.undock());
        (void)v;
    }
}

TEST_CASE("Zero settling push: the liquid stays forward, the transfer pauses", "[refuelling][prop_transfer][docking]") {
    test::spice_session();
    DockedPair f;
    auto r = tanker_request();
    r.settling_acceleration_mps2 = 0.0;
    REQUIRE(f.session.start_transfer(r));
    f.session.advance_to(f.session.epoch() + 60.0, 1000000);
    CHECK(f.session.transfer_state().phase == sim::TransferPhase::Paused);
    CHECK(f.session.transfer_state().delivered_kg[0] == 0.0);
}
```

A second start must not overwrite the running transfer's state: it returns false with the reason in
`docking_status()`. The zero-push case: `thrust_available` is false only when the chaser *cannot* push (no +x
capability or no main-tank gas); a requested push of 0 gives `a = 0`, a settle time of 0 and `settled = false` (drag
moves the liquid forward) -> "Treibstoff nicht am Auslass".

- [ ] **Step 2: Run the tests to verify they fail**

Build: compile error (`start_transfer` unknown).

- [ ] **Step 3: Implement**

`flight_session.hpp`: include `starsim/sim/propellant_transfer.hpp`; add the public block from the Interfaces; private:

```cpp
    std::optional<PropellantTransfer> transfer_;
    TransferState transfer_refusal_;  // shown while no transfer was started
    time::Epoch transfer_next_;       // the next transfer step
    double transfer_thrust_n_ = 0.0;  // the settling push of the running step
    struct TransferSides {            // resolved at the start
        size_t delivering = 0, receiving = 0;  // body indices
        std::array<size_t, 2> delivering_stage{}, receiving_stage{};
        std::array<std::string, 2> delivering_tank, receiving_tank;
        double pressure_difference_pa = 0.0;
        double longest_tank_m = 0.0;
    } transfer_sides_;
    TransferTanks transfer_tanks() const;
    void run_transfer_step(double dt_s);
    bool refuse_transfer(const char* reason);
```

`flight_session_transfer.cpp` additions:

```cpp
namespace {

const config::TransferConnectorConfig* connector_for(const vehicle::Vehicle& v, config::TankContents contents,
                                                     size_t* stage_index) {
    for (size_t s = 0; s < v.stages().size(); ++s) {
        for (const auto& c : v.stages()[s].config.transfer_connectors) {
            if (c.contents == contents) {
                *stage_index = s;
                return &c;
            }
        }
    }
    return nullptr;
}

double tank_mass(const vehicle::Vehicle& v, size_t stage, const std::string& id) {
    for (const auto& t : v.stages()[stage].tanks) {
        if (t.config.id == id) return t.mass_kg;
    }
    return 0.0;
}

double tank_capacity(const vehicle::Vehicle& v, size_t stage, const std::string& id) {
    for (const auto& t : v.stages()[stage].tanks) {
        if (t.config.id == id) return t.config.capacity_kg;
    }
    return 0.0;
}

}  // namespace

bool FlightSession::refuse_transfer(const char* reason) {
    transfer_refusal_ = {};
    transfer_refusal_.status = reason;
    if (!transfer_) return false;
    transfer_.reset();
    return false;
}

const TransferState& FlightSession::transfer_state() const {
    return transfer_ ? transfer_->state() : transfer_refusal_;
}

bool FlightSession::start_transfer(const TransferRequest& request) {
    if (transfer_ && transfer_->active()) {
        docking_status_ = "Transfer läuft bereits";
        return false;
    }
    if (!docked()) return refuse_transfer("Transfer nicht möglich: nicht angedockt");
    if (autopilot_.state != AutopilotState::Off && autopilot_.state != AutopilotState::Orbit) {
        return refuse_transfer("Transfer nicht möglich: Autopilot aktiv");
    }
    const size_t chaser = controlled_, station = *station_index_;
    TransferSides sides;
    sides.delivering = request.direction == TransferDirection::ChaserToStation ? chaser : station;
    sides.receiving = sides.delivering == chaser ? station : chaser;
    const auto& dv = bodies_[sides.delivering].body->vehicle();
    const auto& rv = bodies_[sides.receiving].body->vehicle();
    std::array<TransferLine, 2> lines;
    const std::array<config::TankContents, 2> contents{config::TankContents::Oxidizer, config::TankContents::Fuel};
    for (size_t k = 0; k < 2; ++k) {
        const auto* dc = connector_for(dv, contents[k], &sides.delivering_stage[k]);
        const auto* rc = connector_for(rv, contents[k], &sides.receiving_stage[k]);
        if (!dc || !rc) return refuse_transfer("Transfer nicht möglich: keine Transferkupplung");
        sides.delivering_tank[k] = dc->tank_id;
        sides.receiving_tank[k] = rc->tank_id;
        const auto& dt = dv.stages()[sides.delivering_stage[k]];
        const auto& dp = dt.config.propellant_transfer;
        const auto& rp = rv.stages()[sides.receiving_stage[k]].config.propellant_transfer;
        if (!dp || !rp) return refuse_transfer("Transfer nicht möglich: kein Transfersystem");
        sides.pressure_difference_pa = dp->delivery_pressure_pa - rp->receive_pressure_pa;
        double density = 0.0;
        for (const auto& t : dt.tanks) {
            if (t.config.id == dc->tank_id) density = t.density_kg_m3;
        }
        lines[k] = {vehicle::pair_discharge_area_m2(*dc, *rc), density, dc->chilldown_kg + rc->chilldown_kg};
        for (const auto* v : {&dv, &rv}) {  // the longest involved tank sets the settling time
            const size_t s = v == &dv ? sides.delivering_stage[k] : sides.receiving_stage[k];
            const auto& id = v == &dv ? dc->tank_id : rc->tank_id;
            for (const auto& t : v->stages()[s].tanks) {
                if (t.config.id == id) sides.longest_tank_m = std::max(sides.longest_tank_m, t.length_m);
            }
        }
    }
    transfer_sides_ = sides;
    const auto tanks = transfer_tanks();
    if (!PropellantTransfer::any_line_open(request, tanks)) {
        return refuse_transfer("Transfer nicht möglich: nichts zu übertragen");
    }
    transfer_.emplace(lines, request, tanks);
    transfer_next_ = epoch_ + request.step_s;
    docked_elapsed_s_ = 0.0;  // the boil-off now runs on the transfer's steps
    return true;
}

void FlightSession::stop_transfer() {
    if (transfer_) transfer_->stop();
}

TransferTanks FlightSession::transfer_tanks() const {
    TransferTanks t;
    const auto& dv = bodies_[transfer_sides_.delivering].body->vehicle();
    const auto& rv = bodies_[transfer_sides_.receiving].body->vehicle();
    for (size_t k = 0; k < 2; ++k) {
        t.delivering_kg[k] = tank_mass(dv, transfer_sides_.delivering_stage[k], transfer_sides_.delivering_tank[k]);
        t.receiving_kg[k] = tank_mass(rv, transfer_sides_.receiving_stage[k], transfer_sides_.receiving_tank[k]);
        t.receiving_capacity_kg[k] =
            tank_capacity(rv, transfer_sides_.receiving_stage[k], transfer_sides_.receiving_tank[k]);
    }
    return t;
}
```

The step (called from `docked_update` while a transfer is active, see below):

```cpp
void FlightSession::run_transfer_step(double dt_s) {
    auto& chaser = *bodies_[controlled_].body;
    auto& station = *bodies_[*station_index_].body;
    const auto& request = transfer_->request();
    const double compound_kg = station.vehicle().mass_properties().mass_kg;  // carries the chaser's mass
    // 1. the settling push of the step that just ended: average thrust as one impulse along the chaser's +x
    if (transfer_thrust_n_ > 0.0) {
        auto s = station.state();
        s.velocity_gcrf_mps += chaser.state().attitude.rotate({1.0, 0.0, 0.0}) * (transfer_thrust_n_ * dt_s / compound_kg);
        station.set_state(s);
        sync_docked();
        // gas from the chaser's main tanks in proportion to their oxidizer and fuel (as Vehicle::tank_mass_flows_kgps)
        const size_t stage = transfer_sides_.delivering == controlled_ ? transfer_sides_.delivering_stage[0]
                                                                       : transfer_sides_.receiving_stage[0];
        const auto& st = chaser.vehicle().stages()[stage];
        const double gas = transfer_thrust_n_ * dt_s / (st.config.rcs.isp_s * constants::standard_gravity_mps2);
        const double ox = st.propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main);
        const double fuel = st.propellant_kg(config::TankContents::Fuel, config::TankRole::Main);
        double drawn = 0.0;
        for (const auto& t : st.tanks) {
            if (t.config.role != config::TankRole::Main || ox + fuel <= 0.0) continue;
            const double share = t.config.contents == config::TankContents::Oxidizer ? ox / (ox + fuel)
                                 : t.config.contents == config::TankContents::Fuel  ? fuel / (ox + fuel)
                                                                                     : 0.0;
            const double kg = std::min(t.mass_kg, gas * share);
            if (kg <= 0.0) continue;
            set_tank_mass(controlled_, stage, t.config.id, t.mass_kg - kg);
            drawn += kg;
        }
        transfer_->add_settling(drawn, transfer_thrust_n_);
    }
    // 2. boil-off of both ships over the step, in the pushed compound's environment
    const double push = transfer_thrust_n_ / compound_kg;
    const Vec3 push_station = docked_attitude_station_.rotate(Vec3{push, 0.0, 0.0});
    transfer_->add_boil_off(vent_docked_boil_off(dt_s, push_station));
    // 3. the next step's push and the outlets under it
    const size_t gas_stage = transfer_sides_.delivering == controlled_ ? transfer_sides_.delivering_stage[0]
                                                                       : transfer_sides_.receiving_stage[0];
    const double capability = chaser.vehicle().rcs_pure_force_capability({1.0, 0.0, 0.0}).force_n;
    const bool gas_left = chaser.vehicle().stages()[gas_stage].propellant_kg(config::TankContents::Oxidizer,
                                                                             config::TankRole::Main) +
                              chaser.vehicle().stages()[gas_stage].propellant_kg(config::TankContents::Fuel,
                                                                                 config::TankRole::Main) >
                          0.0;
    const double thrust = gas_left ? std::min(capability, request.settling_acceleration_mps2 * compound_kg) : 0.0;
    const double a = thrust / compound_kg;
    const Vec3 next_push = docked_attitude_station_.rotate(Vec3{a, 0.0, 0.0});
    // only the delivering outlets of the open lines must be covered (the receiving tank takes liquid in)
    bool settled = a > 0.0;
    const size_t d = transfer_sides_.delivering;
    const auto& dv = bodies_[d].body->vehicle();
    const auto liquid = docked_liquid(d, next_push);
    for (size_t k = 0; k < 2 && settled; ++k) {
        if (!transfer_->state().line_open[k]) continue;
        const size_t stage = transfer_sides_.delivering_stage[k];
        const auto& tanks = dv.stages()[stage].tanks;
        for (size_t i = 0; i < tanks.size(); ++i) {
            if (tanks[i].config.id == transfer_sides_.delivering_tank[k]) {
                settled = vehicle::outlet_check(dv, stage, i, liquid, request.max_settled_tilt_rad).covered;
            }
        }
    }
    TransferStepInput in;
    in.dt_s = dt_s;
    in.tanks = transfer_tanks();
    in.settled = settled;
    in.thrust_available = capability > 0.0 && gas_left;
    in.settle_time_s = a > 0.0 ? vehicle::settling_time_s(transfer_sides_.longest_tank_m, a) : 0.0;
    in.pressure_difference_pa = transfer_sides_.pressure_difference_pa;
    const auto out = transfer_->step(in);
    // 4. move the propellant (the compound follows)
    for (size_t k = 0; k < 2; ++k) {
        const auto& d = transfer_sides_;
        if (out.moved_kg[k] + out.vented_kg[k] > 0.0) {
            set_tank_mass(d.delivering, d.delivering_stage[k], d.delivering_tank[k],
                          in.tanks.delivering_kg[k] - out.moved_kg[k] - out.vented_kg[k]);
        }
        if (out.moved_kg[k] > 0.0) {
            set_tank_mass(d.receiving, d.receiving_stage[k], d.receiving_tank[k], in.tanks.receiving_kg[k] + out.moved_kg[k]);
        }
    }
    transfer_thrust_n_ = out.thrust ? thrust : 0.0;
}
```

The `in.tanks` are read **after** the gas and boil-off of steps 1-2, so the floors hold against them. Include
`starsim/math/constants.hpp` and `starsim/vehicle/propellant_transfer.hpp` in `flight_session_transfer.cpp`.

`docked_update` becomes:

```cpp
void FlightSession::docked_update(double dt_s, bool after_coast) {
    if (!docked()) return;
    docked_elapsed_s_ += dt_s;
    if (transfer_ && transfer_->active()) {
        if (epoch_ < transfer_next_ - 1e-9) return;
        run_transfer_step(docked_elapsed_s_);
        transfer_next_ = epoch_ + transfer_->request().step_s;
        docked_elapsed_s_ = 0.0;
        return;
    }
    transfer_thrust_n_ = 0.0;
    if (!after_coast && docked_elapsed_s_ < vehicle::FlightBody::boil_off_interval_s - 1e-9) return;
    vent_docked_boil_off(docked_elapsed_s_, {});
    docked_elapsed_s_ = 0.0;
}
```

`coast_allowance_s` (before the final return): `if (transfer_ && transfer_->active()) allowance = std::min(allowance,
transfer_next_ - epoch_);` (a coast of exactly one step is allowed: the existing `coast > 1.0` test passes for 10 s).

`undock()`: first line after the `!docking_` check:

```cpp
    if (transfer_ && transfer_->active()) {
        docking_status_ = "Abkoppeln nicht möglich: Treibstofftransfer läuft";
        return false;
    }
```

`reset_docking()` and `detach_compound()`: `if (transfer_) transfer_->stop(); transfer_thrust_n_ = 0.0;` (a collision or
a cleared target ends a transfer).

- [ ] **Step 4: Run the tests to verify they pass**

Build, then `ctest --test-dir build -L "refuelling|docking|iss_mission" -j 24 --output-on-failure > tmp/r4_t5.txt 2>&1`
(`undock` and the docked path changed). Expected: pass; record the tanker -> depot numbers from the INFO line (run the
case with `-s`): duration, delivered LOX / CH4, gas, chill-down, boil-off, thrust - they go into `docs/subsystems.md`
in Task 7. If the chunking case takes > 40 s it keeps its `[slow]` tag, otherwise drop it.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/src/sim/flight_session_docking.cpp core/src/sim/flight_session_transfer.cpp core/tests/test_refuelling_transfer.cpp
git commit -m "feat(sim): ship-to-ship propellant transfer - settling push, chill-down, pressure-driven flow, live masses"
git push origin main
```

---

### Task 6: The tanker's return reserve

**Files:**
- Create: `core/include/starsim/mission/tanker_reserve.hpp`, `core/src/mission/tanker_reserve.cpp`; Modify:
  `core/CMakeLists.txt` (after `src/mission/phasing_planner.cpp`)
- Modify: `core/include/starsim/sim/flight_session.hpp:389` (`ship_return_preview`, `tanker_reserve`,
  `tanker_transfer_request`; private `ship_deorbit_plan`), `core/src/sim/flight_session_return.cpp:116-164` (factor the
  planning part of `plan_ship_return` into `ship_deorbit_plan`)
- Modify: `core/src/sim/flight_session_transfer.cpp` (`tanker_reserve`, `tanker_transfer_request`)
- Test: `core/tests/test_refuelling_transfer.cpp`

**Interfaces:**
- Consumes: `gnc::plan_deorbit`, `gnc::DeorbitPlan`, `gnc::DeorbitParameters`; `FlightSession::burn_configurations`.
- Produces:

```cpp
namespace starsim::mission {
struct TankerReserveParameters {
    double deorbit_delta_v_mps = 0.0;   // the previewed deorbit plan
    double exhaust_velocity_mps = 0.0;  // of the deorbit engine set
    double mixture_ratio = 3.6;         // oxidizer / fuel of the deorbit set
    double burn_margin = 1.10;          // [E] plan_ship_return's commit margin (r2)
    double gas_kg = 1500.0;             // [E] cold gas after the transfer: departure, turn to the burn, entry
    double boil_off_kg = 2000.0;        // [E] the residue boiled off until the burn (r2: ~1.4 t of 10 t in 22 h)
    double unusable_kg = 1000.0;        // [E] trapped in sumps and lines
};
struct TankerReserve {
    double burn_kg = 0.0;
    double oxidizer_kg = 0.0;
    double fuel_kg = 0.0;
    double total_kg = 0.0;
};
TankerReserve tanker_reserve(double dry_with_headers_kg, const TankerReserveParameters& p);
}
// FlightSession (public):
gnc::DeorbitPlan ship_return_preview(const gnc::DeorbitParameters& parameters = {}) const;
std::optional<mission::TankerReserve> tanker_reserve(mission::TankerReserveParameters base = {},
                                                     std::string* reason = nullptr) const;
std::optional<TransferRequest> tanker_transfer_request(mission::TankerReserveParameters base = {},
                                                       std::string* reason = nullptr) const;
```

- [ ] **Step 1: Write the failing tests**

Append (include `starsim/mission/tanker_reserve.hpp`):

```cpp
TEST_CASE("Tanker reserve: the deorbit burn by the rocket equation plus gas, boil-off and residue",
          "[refuelling][prop_transfer][mission]") {
    mission::TankerReserveParameters p;
    p.deorbit_delta_v_mps = 80.0;
    p.exhaust_velocity_mps = 3300.0;
    const auto r = mission::tanker_reserve(290.0e3, p);
    // after the burn 290 + 1.5 + 1 t; burn = 292.5 t (e^(80/3300) - 1) x 1.10 = 7.8955 t
    CHECK(r.burn_kg == Catch::Approx(292.5e3 * std::expm1(80.0 / 3300.0) * 1.10));
    CHECK(r.total_kg == Catch::Approx(r.burn_kg + 1500.0 + 2000.0 + 1000.0));
    CHECK(r.oxidizer_kg == Catch::Approx(r.total_kg * 3.6 / 4.6));
    CHECK(r.fuel_kg == Catch::Approx(r.total_kg / 4.6));
}

TEST_CASE("Docked tanker: its reserve from the previewed return, as a transfer request",
          "[refuelling][prop_transfer][docking][deorbit]") {
    test::spice_session();
    DockedPair f;
    std::string reason;
    const auto plan = f.session.ship_return_preview();
    INFO(plan.reason);
    REQUIRE(plan.feasible);
    const auto reserve = f.session.tanker_reserve({}, &reason);
    INFO(reason);
    REQUIRE(reserve);
    INFO("deorbit " << plan.delta_v_mps << " m/s, reserve " << reserve->total_kg << " kg");
    CHECK(plan.delta_v_mps > 30.0);
    CHECK(plan.delta_v_mps < 200.0);
    CHECK(reserve->total_kg > 8.0e3);
    CHECK(reserve->total_kg < 30.0e3);
    const auto request = f.session.tanker_transfer_request({}, &reason);
    REQUIRE(request);
    CHECK(request->direction == sim::TransferDirection::ChaserToStation);
    CHECK(request->delivering_floor_kg[0] == Catch::Approx(reserve->oxidizer_kg));
    CHECK(request->delivering_floor_kg[1] == Catch::Approx(reserve->fuel_kg));
    // not docked: no reserve
    refuelling_test::TankerAtPort free({20.0, 0.0, 0.0});
    CHECK_FALSE(free.session.tanker_reserve({}, &reason));
    CHECK(reason == "Keine Reserve: nicht angedockt");
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Build: compile error.

- [ ] **Step 3: Implement**

`core/include/starsim/mission/tanker_reserve.hpp`: the declarations from the Interfaces with this comment on the
function:

```cpp
// The main-tank residue a tanker keeps for its return (refuelling r4, spec "End"): the deorbit burn by the rocket
// equation from the mass after it (`dry_with_headers_kg`: the ship without main propellant, header tanks full - the
// landing reserve - plus the gas and the unusable residue), times the margin; plus the gas, the boil-off until the
// burn and the unusable residue. Split at the mixture ratio (the gas draws the main tanks in proportion, the burn at
// the engines' ratio; boil-off ratios differ - the margin covers it, r5 flies it).
```

`core/src/mission/tanker_reserve.cpp`:

```cpp
#include "starsim/mission/tanker_reserve.hpp"

#include <cmath>

namespace starsim::mission {

TankerReserve tanker_reserve(double dry_with_headers_kg, const TankerReserveParameters& p) {
    TankerReserve r;
    const double after_burn_kg = dry_with_headers_kg + p.gas_kg + p.unusable_kg;
    r.burn_kg = after_burn_kg * std::expm1(p.deorbit_delta_v_mps / p.exhaust_velocity_mps) * p.burn_margin;
    r.total_kg = r.burn_kg + p.gas_kg + p.boil_off_kg + p.unusable_kg;
    r.oxidizer_kg = r.total_kg * p.mixture_ratio / (1.0 + p.mixture_ratio);
    r.fuel_kg = r.total_kg / (1.0 + p.mixture_ratio);
    return r;
}

}  // namespace starsim::mission
```

`flight_session_return.cpp`: move the body of `plan_ship_return` from `const auto& stage = ...` through the
`gnc::plan_deorbit(...)` call into

```cpp
gnc::DeorbitPlan FlightSession::ship_deorbit_plan(const gnc::DeorbitParameters& parameters,
                                                  std::vector<std::string>* groups) const;
```

which returns the refused plan (same German reasons) or the planner's result; `plan_ship_return` calls it, stores it in
`deorbit_plan_` and continues with the boil-off check and `execute_burn` exactly as before. Note the refusal "kein
Schiff mit Hitzeschild ..." also checks `flying.entry_guidance` / `ship_landing` - keep it. `ship_return_preview` is
`return ship_deorbit_plan(parameters, nullptr);` (works while docked: it reads the held chaser's state).

`flight_session_transfer.cpp`:

```cpp
std::optional<mission::TankerReserve> FlightSession::tanker_reserve(mission::TankerReserveParameters base,
                                                                    std::string* reason) const {
    const auto refuse = [&](const char* why) -> std::optional<mission::TankerReserve> {
        if (reason) *reason = why;
        return std::nullopt;
    };
    if (!docked()) return refuse("Keine Reserve: nicht angedockt");
    const auto plan = ship_return_preview();
    if (!plan.feasible) {
        if (reason) *reason = "Keine Reserve: " + plan.reason;
        return std::nullopt;
    }
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    const auto& stage = vehicle.stages().front().config;
    const auto configurations = burn_configurations(&stage.deorbit_engine_groups);
    if (configurations.empty()) return refuse("Keine Reserve: Deorbit-Triebwerk nicht verfügbar");
    base.deorbit_delta_v_mps = plan.delta_v_mps;
    base.exhaust_velocity_mps = configurations.front().thrust_n / configurations.front().mass_flow_kgps;
    for (const auto& s : vehicle.stages()) {
        for (const auto& e : s.engines) {
            if (e.group_id == configurations.front().groups.front()) {
                base.mixture_ratio = e.performance->config().mixture_ratio;
            }
        }
    }
    const double dry_with_headers_kg = vehicle.mass_properties().mass_kg - vehicle.main_propellant_kg();
    return mission::tanker_reserve(dry_with_headers_kg, base);
}

std::optional<TransferRequest> FlightSession::tanker_transfer_request(mission::TankerReserveParameters base,
                                                                     std::string* reason) const {
    const auto reserve = tanker_reserve(base, reason);
    if (!reserve) return std::nullopt;
    TransferRequest r;
    r.direction = TransferDirection::ChaserToStation;
    r.delivering_floor_kg = {reserve->oxidizer_kg, reserve->fuel_kg};
    return r;
}
```

(`burn_configurations` is non-const today if it caches: make the preview path const-correct, or mark the helper
`const` if it only reads - check its declaration. `e.group_id` is the engine's group field name used in
`Vehicle::update_engines`' `find_group(command, e.group_id)`.)

- [ ] **Step 4: Run the tests to verify they pass**

Build, then `ctest --test-dir build -L "refuelling|deorbit|boil_off" -j 24 --output-on-failure > tmp/r4_t6.txt 2>&1`
and `build/core/tests/starsim_core_tests.exe "[ship][mission]" > tmp/r4_t6_ship.txt 2>&1` (the ship return code path
was refactored: its campaigns incl. `[slow]`; background + 10 min checks if needed). Expected: all pass; record the
previewed delta-v and the reserve for the docs.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/mission/tanker_reserve.hpp core/src/mission/tanker_reserve.cpp core/CMakeLists.txt core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session_return.cpp core/src/sim/flight_session_transfer.cpp core/tests/test_refuelling_transfer.cpp
git commit -m "feat(mission): tanker return reserve from the previewed deorbit burn, as the transfer floor"
git push origin main
```

---

### Task 7: Documentation, smoke test, final review

**Files:**
- Modify: `docs/subsystems.md` (new "### Propellant transfer (r4)" after the r3 section; r2 "Flight" bullet: docked
  boil-off now by the session; r3 "Deferred": docked boil-off done)
- Modify: `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md` (status line: r4 implemented; corrections
  from the plan decisions if the user changed any)
- Modify: `CLAUDE.md` (module summary: `vehicle` propellant transfer physics, `sim` propellant transfer, `mission`
  tanker reserve)
- Modify: `config/README.md` (if not complete from Task 1)

- [ ] **Step 1: Write the subsystems section**

Content (fill the numbers measured in Tasks 4-6):
- Model: orifice flow per line, the pair's smaller C_d A; dp = delivery - receive pressure; densities from the
  propellant; values [E] from the plan decisions.
- Settling: average +x push of the chaser at 1e-4 g, impulse per 10 s step, gas from the chaser's main tanks at Isp
  70 s; settle time sqrt(2L/a); outlet covered at Bo >= 10 and <= 30 deg; pause / resume; simplifications (no pulses,
  no torque, no slosh, no hydrostatic head, receiving inlet not checked).
- Sequence and end conditions; floors honoured at line closing; the tanker reserve (formula, [E] items, the measured
  delta-v and reserve).
- Docked boil-off by the session (both bodies in the compound's environment; FlightBody skips while docked or carrying
  a visitor).
- Numbers: tanker 200 t -> depot 600 t: duration, delivered LOX / CH4, gas, chill-down, boil-off, thrust; docked
  boil-off over 2 h for tanker and depot; depot -> HLS limits case.
- Tests: `[refuelling][prop_transfer]` (`test_refuelling_transfer.cpp`).
- Deferred to r5 / r6 (from this plan and reviews).

- [ ] **Step 2: Bindings build + smoke test**

`powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoTests > tmp/r4_bindings.txt 2>&1` (the GDExtension must
still compile against the changed headers), then
`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd > tmp/r4_smoke.txt 2>&1`.
Expected: both succeed.

- [ ] **Step 3: Commit the docs**

```bash
git add docs/subsystems.md docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md CLAUDE.md config/README.md
git commit -m "docs: refuelling r4 propellant transfer"
git push origin main
```

- [ ] **Step 4: Final review**

Dispatch one reviewer (most capable model) over `git diff <r4 plan commit>..HEAD` with the spec's r4 section, this plan
and the Review Focus. Fix Critical / Important findings in one pass (tests of the affected tags), commit
`fix(sim): r4 review fixes - ...`, push. Record deferred minors in the ledger
`.superpowers/sdd/2026-10-20-refuelling-r4-transfer/progress.md` and the memory `project-orbital-refuelling`; ask the
user for `/clear` naming r5 (reference flights) as the next step.
