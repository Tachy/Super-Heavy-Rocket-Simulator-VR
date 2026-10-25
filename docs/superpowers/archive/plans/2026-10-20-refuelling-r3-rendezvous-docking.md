# Refuelling r3: Starship rendezvous and probe/drogue docking - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Starship tanker (and later the HLS) in a low orbit below the depot flies the coelliptic phasing to
Waypoint 0 on its Raptor and cold-gas RCS, approaches the depot along the R-bar from below, engages its four probes in
the depot's four drogues, is pulled in and held as part of the compound, and can undock and depart again.

**Architecture:** The ISS stack is generalised, not copied (spec architecture A). `[stage.station]` becomes the target
block with the approach limits (`[stage.station.proximity]`), applied by `sim/station_limits`. `gnc` gets the R-bar
approach from below (nadir port) as a straight closed-loop approach. `contact` gets the probe/drogue interface as a
second interface of the existing `DockingMechanism` (same phase machine: contact -> soft capture -> attenuation ->
retraction -> latches -> docked), selected by the port mechanism. The session arms it, sizes contact plane, re-arm and
coast limit from the mechanism, samples the whole hull (and landing-leg struts) for the collision check, and refreshes
the compound when tank masses change. The phasing flies a main-engine chaser on its single-Raptor burn set with cold gas
below the Raptor's minimum impulse.

**Tech Stack:** C++20 core (float64, SI), Catch2, toml++ loader (`detail::Document`), CSPICE through the `spice`
adapters (tests).

**Spec:** `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md` (sections "r3 design", "Error handling",
"Testing", "Documentation"; r1 corrections). Notes: `docs/subsystems.md` "ISS rendezvous and docking (milestone i)" and
"Orbital refuelling". Config conventions: `config/README.md`.

## Plan decisions that need the user's approval

1. **-x cold-gas thrusters (user decision 2026-10-20, before this plan).** The spec and r1 assumed "+x only (no -x);
   docking only needs the axial axis held, not braked". That is wrong: in the docking attitude the ship's x axis lies
   along the V-bar, and without along-track thrust the quantity `v_along + 2 n x_radial` (CW) is conserved; +V thrust
   can only raise it. Approaching upward needs it raised (works), but every hold, retreat and departure downward needs
   it lowered: a 20 m retreat after a failed capture ends in a 4.7 cm/s forward drift the ship can never stop. The
   tanker therefore gets a second pod of two "aft" thrusters (thrust along -x) at the forward pod's station [E], same
   1,500 N as the aft pod's "forward" pair; the HLS gets "aft" in its forward pod. A reused stage (`from`) may now append
   `[[stage.rcs_pod]]` entries (like docking ports). The Block 3 ship itself is unchanged. The HLS's new thrusters can
   also give yaw torque; the lunar tags are re-run and changed numbers are reported, not tuned.
2. **Target block:** `[stage.station]` keeps its name and gets `[stage.station.proximity]` (`keep_out_radius_m`,
   `approach_ellipsoid_m`, `waypoint0_below_m`, `waypoint1_range_m`, `waypoint2_range_m`, `flyaround_radius_m`; every
   key optional, unset = the ISS default in code). The depot gets the ISS numbers explicitly [E: NASA's ISS rules are
   the only published set] and `body_in_lvlh_deg = [0, 0, 0]` (nose along +V-bar, dorsal drogues to nadir). A station
   is **one stage**: `add_station` refuses a stack (the depot is added without its booster, `config::top_stage_config`);
   r5 turns the launched depot into the station.
3. **R-bar from below:** a nadir-facing port gets `DockingApproach::RBarBelow` with the path Waypoint 0 -> Waypoint 1
   -> Waypoint 2 (no fly-around). A segment to an on-axis waypoint is flown with the closed-loop axial approach (not a
   CW transfer) whenever the chaser is already on the approach axis beyond it - Waypoint 0 lies on the depot's R-bar.
   ISS paths are unchanged (their Waypoint 0 is off the IDA axes). Capture envelope from the spec [E]: closing
   0.03-0.10 m/s, offset <= 0.10 m, angle <= 1 deg, roll <= 1 deg, rates as IDSS.
4. **Probe/drogue interface [E] (SpaceX publishes nothing):** drogue cone depth 0.50 m, mouth radius 0.35 m, seat radius
   0.06 m, probe head radius 0.05 m (capture radius 0.30 m = 3x the envelope offset, IDSS has 2.4x), probe stroke 0.30
   m. Soft capture on the **first** seated probe head (each probe has its own latch; the four attenuators are modelled
   as one 6-DOF joint at the port centre, as the IDSS joint), retraction of the probes, then four structural latches
   (180 s) -> docked. Joint: translation as IDSS (chaser mass x 0.5^2 rad^2/s^2, critically damped); rotation by the
   chaser's inertia x 0.5^2 (the IDSS rotational values are load limits of a 12 t pair). Load limits [E]: dynamic
   compression 100 kN, static 60 kN, tension 60 kN, shear 50 kN, bending / torsion 2 MN m: a 0.10 m/s contact of a
   490 t tanker peaks at ~49 kN (joint damper), 0.25 m/s overloads. Separation springs 150 J [E] (0.03 m/s for a 450 t
   / 1,600 t pair).
5. **Starship phasing:** a chaser with burn engines flies the phasing burns on its deorbit burn set
   (`deorbit_engine_groups`, the belly sea-level Raptor - **not a vacuum Raptor as the spec says**: the config's vacuum
   group fires its three together) with transients up to `small_burn_transient_share` (0.6); below that minimum on the
   cold-gas RCS (+x, nose along the delta-v). The planner is unchanged: it plans with the Raptor model; each burn is
   retargeted with the model it will be flown on (Raptor or RCS), so the long RCS burns are targeted as finite burns.
   The plan's propellant figure stays Raptor-based (r5 measures the real use from the tank masses). Budget: the main
   tanks (the headers are the landing reserve). Drag estimate (tracked in flight as for Dragon): Cd 2.2 on the end-on
   area for chaser and depot (`fixed_drag_*` when the target has it). Between burns a main-engine chaser holds the
   docking attitude's orientation in LVLH (nose along +V, belly to nadir), so its drag and boil-off are defined.
6. **Hull collision:** only a nose port (axis along +x) cuts the top stage's hull sampling (the dorsal ports sampled
   only 0-19 m); landing-leg struts (root 4 m above the stage bottom [E] -> foot) are sampled too. The depot gets two
   collision capsules [E] (tank section r 4.5 m, nose r 2.5 m). The HLS leg clearance at the depot (r1 open point) is
   checked statically in the mated pose.
7. **Compound refresh:** `FlightSession::set_tank_mass(body, stage, tank, kg)` sets a tank and, for a docked pair,
   moves the compound CG with the new masses (hull positions fixed, velocity kept: transfer and venting are internal /
   isotropic). r4 uses it for the transfer.

## Global Constraints

- SI units internally, angles in radians; frame and unit in names (`position_port_m`, `keep_out_radius_m`).
- No magic numbers in code: physical constants in `core/include/starsim/math/constants.hpp` with source; vehicle numbers
  in `config/` with provenance tags [S] / [E] / [D]; mechanism estimates as commented `[E]` defaults in their parameter
  struct (as `DockingInterfaceParameters`).
- C++ sources UTF-8; user-facing strings German, code and comments English.
- Modules depend downwards only: `contact` may use `config`, `math`, `vehicle` not; `gnc` never includes `sim`.
- The ISS / Crew Dragon behaviour must not change: tags `[iss]`, `[docking]`, `[proximity]`, `[rendezvous]`,
  `[iss_mission]`, `[launch_window]` stay green including their `[slow]` cases.
- ctest runs cases in parallel: tests must not share temp files or directories.
- Test workflow: run only the tags the change affects, `[slow]` cases of those tags included. Build with
  `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests > tmp/r3_build.txt 2>&1`, then run the
  labels in parallel with `ctest --test-dir build -L "<regex>" -j 24 --output-on-failure > tmp/r3_tN.txt 2>&1` and read
  the tail (CTest labels = Catch tags). A single new case for RED: `build/core/tests/starsim_core_tests.exe "<name>"`.
- Commit on `main`, Conventional Commits, push after each task once its tests are green. Trailer:
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_012biTJd57gGHxfvAHbX6f2T`.

## Review Focus

1. Every ISS / Dragon path is unchanged by the generalisation (IDSS mechanism values, contact plane 0.70 m, re-arm and
   coast distances, approach paths, departure) - existing `[iss]` suites incl. `[slow]` in Tasks 3, 5 and 8, plus Task 5
   test "IDSS reach and contact plane are unchanged".
2. A probe/drogue contact outside the envelope (overspeed, roll 180 deg with all four heads beside the mouths) must fail
   cleanly ("Überlast" -> retreat to Waypoint 2; tip strike -> bounce -> "Versatz zu groß"), never latch rotated - Task 4
   test "a head beside the mouth strikes the plate, no latch" and Task 5 test "0.4 m/s overloads and retreats".
3. Tank masses of a docked pair driven to zero must keep the compound finite (no NaN, attached mass >= dry mass) and
   the hulls in place - Task 6 test "emptied tanks keep the compound finite".
4. A Starship with (nearly) empty main tanks or without the -x thrusters asks for a rendezvous / proximity flight: a
   German refusal, no throw - Task 7 test "a tanker without propellant is refused" and Task 3 test "authority on all
   six axes" (a missing axis is reported in the start refusal).
5. The depot added as a full stack, a second station or a depot without `[stage.station]`: a clear `logic_error`
   (programming error), never a silent wrong attitude - Task 2 test.

---

### Task 1: Docking thrusters (-x) and the RCS supply

**Files:**
- Modify: `core/src/config/vehicle_config.cpp:57-110` (factor the pod expansion into `append_rcs_pods`), `:611-644`
  (`load_stage_from`: allow `rcs_pod`)
- Modify: `core/include/starsim/vehicle/vehicle.hpp:197-199` (declare `rcs_supply_kg`)
- Modify: `core/src/vehicle/vehicle.cpp:376-380` (define it after `rcs_propellant_kg`)
- Modify: `config/vehicles/starship_tanker.toml` (append the docking pod), `config/vehicles/starship_hls.toml:124-130`
  (forward pod gets "aft")
- Create: `core/tests/test_refuelling_docking.cpp`; Modify: `core/tests/CMakeLists.txt` (add after
  `test_boil_off.cpp`)
- Modify: `config/README.md` (`[[stage.rcs_pod]]` on a reused stage)

**Interfaces:**
- Produces: `double vehicle::Vehicle::rcs_supply_kg() const` - propellant the RCS can draw: per stage with thrusters,
  its main-role oxidizer + fuel when `rcs.propellant == "main"`, else its Rcs tanks.
- Produces: config key `[[stage.rcs_pod]]` on a `from` stage (same keys as `[[stage.rcs.pod]]`; the source stage must
  have `[stage.rcs]`).

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_refuelling_docking.cpp`:

```cpp
// Refuelling r3: Starship rendezvous and probe/drogue docking at the depot.

#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

std::unique_ptr<vehicle::Vehicle> ship_alone(const std::string& vehicle_id) {
    return std::make_unique<vehicle::Vehicle>(config::top_stage_config(config::load_vehicle(config_dir, vehicle_id)),
                                              config_dir);
}

}  // namespace

TEST_CASE("Tanker and HLS translate on all six axes; the Block 3 ship still has no -x", "[refuelling][rcs]") {
    for (const std::string id : {"starship_tanker", "starship_hls"}) {
        auto ship = ship_alone(id);
        ship->set_tank_mass(0, "lox_main", 7826.0);  // 10 t main propellant at O/F 3.6 (r1 translation case)
        ship->set_tank_mass(0, "ch4_main", 2174.0);
        INFO(id);
        for (const Vec3 u : {Vec3{1, 0, 0}, Vec3{-1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, -1, 0}, Vec3{0, 0, 1}, Vec3{0, 0, -1}}) {
            CHECK(ship->rcs_pure_force_capability(u).force_n > 1000.0);
        }
        CHECK(ship->rcs_pure_force_capability({-1, 0, 0}).force_n ==
              Catch::Approx(ship->rcs_pure_force_capability({1, 0, 0}).force_n).epsilon(0.01));
    }
    auto block3 = ship_alone("starship_block3");
    CHECK(block3->rcs_pure_force_capability({-1, 0, 0}).force_n == 0.0);
}

TEST_CASE("RCS supply: main-tank cold gas counts the main tanks, Dragon its RCS tank", "[refuelling][rcs]") {
    auto tanker = ship_alone("starship_tanker");
    tanker->set_tank_mass(0, "lox_main", 20000.0);
    tanker->set_tank_mass(0, "ch4_main", 6000.0);
    CHECK(tanker->rcs_supply_kg() == Catch::Approx(26000.0));  // the headers are not RCS supply (landing reserve)
    CHECK(tanker->rcs_propellant_kg() == 0.0);
    const vehicle::Vehicle dragon(config::load_vehicle(config_dir, "crew_dragon"), config_dir);
    CHECK(dragon.rcs_supply_kg() == Catch::Approx(dragon.rcs_propellant_kg()));
    CHECK(dragon.rcs_supply_kg() > 0.0);
}

namespace {

// A config dir with Block 3, the engines and propellants and one test vehicle (as test_refuelling_vehicles.cpp; own
// temp prefix - ctest runs cases in parallel).
std::filesystem::path config_with(const std::string& name, const std::string& vehicle_toml) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_r3_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir / "engines", dir / "engines", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(config_dir / "propellants.toml", dir / "propellants.toml");
    std::filesystem::copy_file(config_dir / "vehicles" / "starship_block3.toml", dir / "vehicles" / "starship_block3.toml");
    std::ofstream(dir / "vehicles" / "test_vehicle.toml") << vehicle_toml;
    return dir;
}

std::string reused_ship(const std::string& additions) {
    return "[vehicle]\nid = \"test_vehicle\"\nname = \"Test\"\npropellant = \"methalox\"\n"
           "stages_bottom_to_top = [\"booster\"]\n\n[[stage]]\nfrom = \"starship_block3:booster\"\n" + additions;
}

const std::string docking_pod = "[[stage.rcs_pod]]\nid = \"dock\"\nstation_m = 46.0\nradius_m = 4.5\n"
                                "azimuths_deg = [0.0, 180.0]\nthrust_n = 1500.0\ndirections = [\"aft\"]\n";

size_t aft_thrusters(const config::StageConfig& s) {
    return static_cast<size_t>(std::count_if(s.rcs.thrusters.begin(), s.rcs.thrusters.end(),
                                             [](const config::RcsThrusterConfig& t) { return t.direction.x < -0.99; }));
}

}  // namespace

TEST_CASE("A reused stage may add RCS pods, not other RCS keys", "[refuelling][config]") {
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    CHECK(aft_thrusters(tanker.stages.back()) == 2);
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");

    // the booster has [stage.rcs] (its forward pod): pods may be added
    auto dir = config_with("rcs_pod_ok", reused_ship(docking_pod));
    const auto added = config::load_vehicle(dir, "test_vehicle");
    CHECK(added.stages.back().rcs.thrusters.size() == block3.stages.front().rcs.thrusters.size() + 2);
    CHECK(aft_thrusters(added.stages.back()) == 2);
    std::filesystem::remove_all(dir);

    dir = config_with("rcs_table", reused_ship("[stage.rcs]\nisp_s = 70.0\n"));
    CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"),
                      Catch::Matchers::ContainsSubstring("cannot be set on a reused stage"));
    std::filesystem::remove_all(dir);
}
```

Also add a case "an rcs_pod on a reused stage without RCS is an error": the Block 3 stages all have `[stage.rcs]`, so
write a second source vehicle into the temp dir - a copy of `starship_block3.toml` with its booster's `[stage.rcs]`
table and `[[stage.rcs.pod]]` entries removed is too fragile; instead test the loader branch directly by giving
`append`-time a source stage without thrusters: `config_with("rcs_none", ...)` plus a minimal source vehicle file
`vehicles/bare.toml` (one stage `ship`, `length_m = 10.0`, `diameter_m = 9.0`, `dry_mass_kg = 1000.0`, no tanks, no
engines - check the loader's required keys in `load_stage` and add only those) and a test vehicle with
`from = "bare:ship"` plus `docking_pod`; expect `CHECK_THROWS_WITH(..., ContainsSubstring("rcs_pod needs"))`. If the
loader cannot load a stage without tanks or engines, drop this case and note it in the ledger (the branch is two lines).

Add the includes `<algorithm>`, `<fstream>` to the test file.

Add `test_refuelling_docking.cpp` to `core/tests/CMakeLists.txt` after `test_boil_off.cpp`.

- [ ] **Step 2: Run to verify they fail**

Build (`tools/build.ps1 -NoBindings -NoTests`): compile error `top_stage_config` / `rcs_supply_kg` not declared.

- [ ] **Step 3: Implement**

`core/include/starsim/config/vehicle_config.hpp`, after the `load_vehicle` declaration:

```cpp
// The top stage alone, as it flies after staging (a ship in orbit, the depot as a station).
VehicleConfig top_stage_config(VehicleConfig vehicle);
```

`core/src/config/vehicle_config.cpp` (public section, after `load_vehicle`):

```cpp
VehicleConfig top_stage_config(VehicleConfig vehicle) {
    if (vehicle.stages.size() > 1) vehicle.stages.erase(vehicle.stages.begin(), vehicle.stages.end() - 1);
    return vehicle;
}
```

Factor the pod loop of `load_rcs` into a file-local function above it and call it from `load_rcs`:

```cpp
// [[stage.rcs.pod]] / [[stage.rcs_pod]] entries: a set of directions at each azimuth of a ring -> thrusters.
void append_rcs_pods(const Document& doc, const toml::array& pods, std::vector<RcsThrusterConfig>& out) {
    for (const auto& node : pods) {
        // ... the existing body of the `for (const auto& node : *pods)` loop, unchanged, pushing into `out` ...
    }
}
```

In `load_rcs`: `if (auto pods = rcs["pod"].as_array()) append_rcs_pods(doc, *pods, out.thrusters);`.

In `load_stage_from`: accept the key and append after the ports:

```cpp
        if (key != "from" && key != "docking_port" && key != "transfer_connector" && key != "rcs_pod") {
```

```cpp
    if (auto pods = t["rcs_pod"].as_array()) {
        // refuelling r3: docking thrusters added to a reused ship (the tanker's -x pair)
        if (s.rcs.thrusters.empty()) doc.fail("stage.from = \"" + from + "\": rcs_pod needs [stage.rcs] on the source stage");
        append_rcs_pods(doc, *pods, s.rcs.thrusters);
    }
```

`vehicle.hpp`, after `rcs_propellant_kg()`:

```cpp
    // What the RCS can draw: main-tank cold gas (rcs.propellant "main") counts the stage's main oxidizer and fuel
    // tanks (not the headers: the landing reserve), otherwise its Rcs tanks (refuelling r3: Starship proximity budgets).
    double rcs_supply_kg() const;
```

`vehicle.cpp`:

```cpp
double Vehicle::rcs_supply_kg() const {
    double total = 0.0;
    for (const auto& s : stages_) {
        if (s.rcs.empty()) continue;
        if (s.config.rcs.propellant == "main") {
            total += s.propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main) +
                     s.propellant_kg(config::TankContents::Fuel, config::TankRole::Main);
        } else {
            total += s.propellant_kg(config::TankContents::Rcs);
        }
    }
    return total;
}
```

`config/vehicles/starship_tanker.toml`, append to the ship stage (after its transfer connectors):

```toml
# Docking thrusters [E] (refuelling r3, user 2026-10-20): two cold-gas thrusters pushing along -x at the forward pod.
# In the docking attitude the ship's x axis lies along the V-bar; without -x a hold or retreat below the depot cannot
# stop the Coriolis drift forward (v_along + 2 n x_radial is conserved without along-track thrust). Same size as the
# aft pod's "forward" pair. SpaceX publishes no thruster layout.
[[stage.rcs_pod]]
id = "docking"
station_m = 46.0
radius_m = 4.5
azimuths_deg = [0.0, 180.0]
thrust_n = 1500.0
directions = ["aft"]
```

`config/vehicles/starship_hls.toml` forward pod:

```toml
directions = ["tangential_plus", "tangential_plus", "tangential_minus", "tangential_minus", "inward", "inward", "aft"]  # aft: -x for docking at the depot [E] (refuelling r3)
```

`config/README.md`: in the `from` section, add `[[stage.rcs_pod]]` to the list of keys a reused stage may add.

- [ ] **Step 4: Run the tests**

Build, then `ctest --test-dir build -L "refuelling|rcs|hls|lunar|config" -j 24 --output-on-failure > tmp/r3_t1.txt
2>&1`; read the tail. New cases pass; the lunar campaigns (HLS) may change numbers - if a lunar case fails, report
the change with the numbers to the user before touching anything (physics, not tuning).

- [ ] **Step 5: Commit**

```bash
git add core/src/config/vehicle_config.cpp core/include/starsim/config/vehicle_config.hpp core/include/starsim/vehicle/vehicle.hpp core/src/vehicle/vehicle.cpp config/vehicles/starship_tanker.toml config/vehicles/starship_hls.toml config/README.md core/tests/test_refuelling_docking.cpp core/tests/CMakeLists.txt
git commit -m "feat(config): -x docking thrusters for tanker and HLS, RCS supply from the main tanks"
git push origin main
```

---

### Task 2: The target block - depot as a station with its approach limits

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp:121-126` (add `StationProximityConfig`), `:328-329`
  (`StageConfig::station_proximity`)
- Modify: `core/src/config/vehicle_config.cpp:345-353` (parse `[stage.station.proximity]`)
- Create: `core/include/starsim/sim/station_limits.hpp`, `core/src/sim/station_limits.cpp`; Modify:
  `core/CMakeLists.txt` (after `src/sim/station_start.cpp`)
- Modify: `core/src/sim/flight_session.cpp:444-470` (`add_station`: one stage, `stages.back()`)
- Modify: `core/src/sim/flight_session_proximity.cpp:44-129` (limits, supply), `:161-173` (`propellant_kg`)
- Modify: `core/src/sim/flight_session_rendezvous.cpp:84-150` (limits), `core/src/sim/flight_session_docking.cpp:311`
  (departure limits)
- Modify: `config/vehicles/starship_depot.toml` (target block + collision capsules of Task 5 come later)
- Create: `core/tests/refuelling_test_support.hpp`
- Test: `core/tests/test_refuelling_docking.cpp`

**Interfaces:**
- Consumes: `config::top_stage_config` (Task 1).
- Produces (`config`): `struct StationProximityConfig { std::optional<double> keep_out_radius_m; std::optional<math::Vec3> approach_ellipsoid_m; std::optional<double> waypoint0_below_m, waypoint1_range_m, waypoint2_range_m, flyaround_radius_m; };` and `StageConfig::station_proximity`.
- Produces (`sim`, `station_limits.hpp`):
  - `gnc::ProximityParameters with_station_limits(gnc::ProximityParameters p, const config::StationProximityConfig& s);`
  - `gnc::DepartureParameters with_station_limits(gnc::DepartureParameters d, const config::StationProximityConfig& s);`
  - `mission::PhasingParameters with_station_limits(mission::PhasingParameters p, const config::StationProximityConfig& s);`
- Produces (tests, `refuelling_test_support.hpp`, namespace `starsim::refuelling_test`):
  - `math::StateVector depot_state(const frames::BodyOrientation& earth, time::Epoch epoch, double argument_of_latitude_rad = 0.0)` - 200 km circular, 26.5 deg, RAAN 0 in the true equator frame.
  - `config::VehicleConfig depot_config(double main_propellant_kg)`, `config::VehicleConfig tanker_config(double main_propellant_kg)` - ship alone, main tanks at O/F 3.6.
  - `struct DepotSession` (see Step 1).

- [ ] **Step 1: Write the test support and the failing tests**

Create `core/tests/refuelling_test_support.hpp`:

```cpp
#pragma once
// Refuelling r3 tests: the depot as a station in its 200 km / 26.5 deg orbit and a tanker (or HLS) near it.
#include <filesystem>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/gnc/orbital_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/docking_port.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::refuelling_test {

inline const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
constexpr double oxidizer_fuel_ratio = 3.6;  // Block 3 main tanks [D]

// The depot orbit of the lunar missions' start (spec r3): 200 km circular, 26.5 deg, RAAN 0 (true equator frame).
inline math::StateVector depot_state(const frames::BodyOrientation& earth, time::Epoch epoch,
                                     double argument_of_latitude_rad = 0.0) {
    orbit::StationMeanElements m;
    m.epoch = epoch;
    m.semi_major_axis_m = constants::wgs84_semi_major_axis_m + 200.0e3;
    m.inclination_rad = 26.5 * constants::deg_to_rad;
    m.argument_of_latitude_rad = argument_of_latitude_rad;
    return orbit::synthetic_station_state(m, constants::earth_gm_m3ps2, earth);
}

// A ship alone with `main_propellant_kg` in its main tanks (O/F 3.6); headers as configured.
inline config::VehicleConfig ship_with(const std::string& vehicle_id, double main_propellant_kg) {
    auto cfg = config::top_stage_config(config::load_vehicle(config_dir, vehicle_id));
    for (auto& tank : cfg.stages.back().tanks) {
        if (tank.role != config::TankRole::Main) continue;
        const double share = tank.contents == config::TankContents::Oxidizer ? oxidizer_fuel_ratio / (1.0 + oxidizer_fuel_ratio)
                                                                             : 1.0 / (1.0 + oxidizer_fuel_ratio);
        tank.mass_kg = std::min(tank.capacity_kg, share * main_propellant_kg);
    }
    return cfg;
}
inline config::VehicleConfig depot_config(double main_propellant_kg) { return ship_with("starship_depot", main_propellant_kg); }
inline config::VehicleConfig tanker_config(double main_propellant_kg) { return ship_with("starship_tanker", main_propellant_kg); }

// A session with the depot (station) at t0 = 2026-10-20 00:00 UTC; the chaser is placed by the test.
struct DepotSession {
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    sim::FlightSession session{ephemeris, earth, config_dir};
    time::Epoch t0 = clock.from_utc({2026, 10, 20, 0, 0, 0.0});
    math::StateVector depot = depot_state(earth, t0);
    double depot_propellant_kg = 600.0e3;  // [test] a depot after a few tankers
};

}  // namespace starsim::refuelling_test
```

Append to `core/tests/test_refuelling_docking.cpp` (add includes `refuelling_test_support.hpp`, `test_support.hpp`,
`starsim/frames/lvlh.hpp`, `starsim/sim/station_limits.hpp`):

```cpp
TEST_CASE("The depot is a target: LVLH attitude nose forward, dorsal to nadir, its approach limits",
          "[refuelling][station][config]") {
    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    const auto& s = depot.stages.back();
    REQUIRE(s.body_in_lvlh_rad);
    CHECK(norm(*s.body_in_lvlh_rad) == 0.0);
    REQUIRE(s.station_proximity.keep_out_radius_m);
    CHECK(*s.station_proximity.keep_out_radius_m == 200.0);
    CHECK(s.station_proximity.approach_ellipsoid_m->x == 2000.0);
    CHECK(*s.station_proximity.waypoint0_below_m == 400.0);
    const auto iss = config::load_vehicle(config_dir, "iss");
    CHECK_FALSE(iss.stages.front().station_proximity.keep_out_radius_m);  // the ISS keeps the code defaults
}

TEST_CASE("Station limits replace only the keys the target sets", "[refuelling][station]") {
    config::StationProximityConfig s;
    s.keep_out_radius_m = 150.0;
    s.waypoint1_range_m = 180.0;
    s.waypoint0_below_m = 350.0;
    s.approach_ellipsoid_m = Vec3{1500.0, 800.0, 800.0};
    const gnc::ProximityParameters defaults;
    const auto p = sim::with_station_limits(gnc::ProximityParameters{}, s);
    CHECK(p.keep_out_radius_m == 150.0);
    CHECK(p.waypoint1_range_m == 180.0);
    CHECK(p.waypoint0_below_m == 350.0);
    CHECK(p.waypoint2_range_m == defaults.waypoint2_range_m);
    CHECK(p.flyaround_radius_m == defaults.flyaround_radius_m);
    CHECK(sim::with_station_limits(gnc::DepartureParameters{}, s).ellipsoid_semi_axes_m.x == 1500.0);
    CHECK(sim::with_station_limits(mission::PhasingParameters{}, s).waypoint0_below_m == 350.0);
    CHECK(sim::with_station_limits(gnc::ProximityParameters{}, config::StationProximityConfig{}).keep_out_radius_m ==
          defaults.keep_out_radius_m);
}

TEST_CASE("add_station takes the depot alone; its drogues face nadir, its nose the velocity", "[refuelling][station]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    CHECK_THROWS_AS(f.session.add_station(config::load_vehicle(config_dir, "starship_depot"), f.depot, f.t0),
                    std::logic_error);  // the stack: a station is one stage
    auto no_target = refuelling_test::tanker_config(100.0e3);
    CHECK_THROWS_AS(f.session.add_station(no_target, f.depot, f.t0), std::logic_error);  // no [stage.station]
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    CHECK_THROWS_AS(f.session.add_station(refuelling_test::depot_config(1.0), f.depot, f.t0), std::logic_error);
    f.session.advance_to(f.t0 + 1.0);
    const auto& st = f.session.body(0).state();
    const math::Mat3 to_lvlh =
        transpose(frames::lvlh_to_inertial({st.position_gcrf_m, st.velocity_gcrf_mps}).to_matrix());
    const auto port = vehicle::port_kinematics(f.session.body(0), "transfer_dock");
    CHECK((to_lvlh * column(port.axes_gcrf, 0)).z == Catch::Approx(1.0).margin(1e-6));  // drogues to nadir
    CHECK((to_lvlh * column(port.axes_gcrf, 2)).x == Catch::Approx(1.0).margin(1e-6));  // nose along +V-bar
}
```

- [ ] **Step 2: Run to verify they fail**

Build: compile errors (`StationProximityConfig`, `station_limits.hpp` missing).

- [ ] **Step 3: Implement**

`vehicle_config.hpp`, after `StationReferenceOrbitConfig`:

```cpp
// [stage.station.proximity] (refuelling r3): the target's approach limits. Unset keys keep the ISS values in code
// (gnc::ProximityParameters, gnc::DepartureParameters, mission::PhasingParameters).
struct StationProximityConfig {
    std::optional<double> keep_out_radius_m;
    std::optional<math::Vec3> approach_ellipsoid_m;  // semi-axes along LVLH x, y, z
    std::optional<double> waypoint0_below_m;
    std::optional<double> waypoint1_range_m;
    std::optional<double> waypoint2_range_m;
    std::optional<double> flyaround_radius_m;
};
```

`StageConfig`, after `reference_orbit`: `StationProximityConfig station_proximity;  // a station: approach limits`.

`vehicle_config.cpp`, inside `if (auto station = t["station"].as_table())` after the reference orbit:

```cpp
        if (auto prox = (*station)["proximity"].as_table()) {
            auto& sp = s.station_proximity;
            const auto optional_positive = [&](const char* key) -> std::optional<double> {
                if (!prox->contains(key)) return std::nullopt;
                return doc.positive(*prox, key);
            };
            sp.keep_out_radius_m = optional_positive("keep_out_radius_m");
            sp.waypoint0_below_m = optional_positive("waypoint0_below_m");
            sp.waypoint1_range_m = optional_positive("waypoint1_range_m");
            sp.waypoint2_range_m = optional_positive("waypoint2_range_m");
            sp.flyaround_radius_m = optional_positive("flyaround_radius_m");
            if (prox->contains("approach_ellipsoid_m")) {
                const math::Vec3 e = vec3(doc, *prox, "approach_ellipsoid_m");
                if (!(e.x > 0.0 && e.y > 0.0 && e.z > 0.0)) doc.fail("stage.station.proximity.approach_ellipsoid_m must be positive");
                sp.approach_ellipsoid_m = e;
            }
            if (sp.waypoint1_range_m && sp.waypoint2_range_m && !(*sp.waypoint2_range_m < *sp.waypoint1_range_m)) {
                doc.fail("stage.station.proximity: waypoint2_range_m must be below waypoint1_range_m");
            }
            if (sp.keep_out_radius_m && sp.waypoint1_range_m && !(*sp.keep_out_radius_m < *sp.waypoint1_range_m)) {
                doc.fail("stage.station.proximity: Waypoint 1 must lie outside the keep-out sphere");
            }
        }
```

`core/include/starsim/sim/station_limits.hpp`:

```cpp
#pragma once

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/departure_guidance.hpp"
#include "starsim/gnc/proximity_geometry.hpp"
#include "starsim/mission/phasing_schedule.hpp"

namespace starsim::sim {

// The target block's approach limits (refuelling r3) over the code defaults: only the keys the target sets replace
// the caller's values (the ISS sets none: its values are the defaults).
gnc::ProximityParameters with_station_limits(gnc::ProximityParameters p, const config::StationProximityConfig& s);
gnc::DepartureParameters with_station_limits(gnc::DepartureParameters d, const config::StationProximityConfig& s);
mission::PhasingParameters with_station_limits(mission::PhasingParameters p, const config::StationProximityConfig& s);

}  // namespace starsim::sim
```

`core/src/sim/station_limits.cpp`:

```cpp
#include "starsim/sim/station_limits.hpp"

namespace starsim::sim {

gnc::ProximityParameters with_station_limits(gnc::ProximityParameters p, const config::StationProximityConfig& s) {
    if (s.keep_out_radius_m) p.keep_out_radius_m = *s.keep_out_radius_m;
    if (s.waypoint0_below_m) p.waypoint0_below_m = *s.waypoint0_below_m;
    if (s.waypoint1_range_m) p.waypoint1_range_m = *s.waypoint1_range_m;
    if (s.waypoint2_range_m) p.waypoint2_range_m = *s.waypoint2_range_m;
    if (s.flyaround_radius_m) p.flyaround_radius_m = *s.flyaround_radius_m;
    return p;
}

gnc::DepartureParameters with_station_limits(gnc::DepartureParameters d, const config::StationProximityConfig& s) {
    if (s.approach_ellipsoid_m) d.ellipsoid_semi_axes_m = *s.approach_ellipsoid_m;
    return d;
}

mission::PhasingParameters with_station_limits(mission::PhasingParameters p, const config::StationProximityConfig& s) {
    if (s.waypoint0_below_m) p.waypoint0_below_m = *s.waypoint0_below_m;
    return p;
}

}  // namespace starsim::sim
```

`add_station` (flight_session.cpp) - replace the first check and the attitude source:

```cpp
    if (config.stages.size() != 1) {
        throw std::logic_error("add_station: a station is one stage (add the depot without its booster)");
    }
    if (!config.stages.back().body_in_lvlh_rad) throw std::logic_error("add_station: vehicle has no [stage.station]");
```

and `const math::Vec3 ypr = *config.stages.back().body_in_lvlh_rad;`. Add a private helper in `flight_session.hpp`
next to `station_state()`:

```cpp
    const config::StationProximityConfig& station_limits() const;  // the station stage's [stage.station.proximity]
```

defined in `flight_session_rendezvous.cpp` after `station_state()`:

```cpp
const config::StationProximityConfig& FlightSession::station_limits() const {
    return bodies_.at(*station_index_).body->vehicle().stages().back().config.station_proximity;
}
```

Apply the limits (include `starsim/sim/station_limits.hpp`):
- `start_proximity`: first line after the station check: `p = with_station_limits(std::move(p), station_limits());`
- `proximity_input`: `in.propellant_kg = bodies_[controlled_].body->vehicle().rcs_supply_kg();`
- `plan_rendezvous`: after the refusals, `p = with_station_limits(p, station_limits());`
- `separate_docked`: `departure_.emplace(with_station_limits(gnc::DepartureParameters{}, station_limits()),
  with_station_limits(gnc::ProximityParameters{}, station_limits()));`

`config/vehicles/starship_depot.toml`, append to the depot stage:

```toml
# Target block (refuelling r3): the depot flies an ideal LVLH attitude like the ISS - nose along +V-bar (end-on drag),
# dorsal side with the drogues to nadir: tankers approach from below along the R-bar.
[stage.station]
body_in_lvlh_deg = [0.0, 0.0, 0.0]

[stage.station.proximity]                # [E] the ISS rules (NASA; the only published set) for a 52 m depot
keep_out_radius_m = 200.0
approach_ellipsoid_m = [2000.0, 1000.0, 1000.0]
waypoint0_below_m = 400.0
waypoint1_range_m = 220.0
waypoint2_range_m = 20.0
```

- [ ] **Step 4: Run the tests**

`ctest --test-dir build -L "refuelling|station|iss|proximity|rendezvous|docking" -j 24 --output-on-failure >
tmp/r3_t2.txt 2>&1` - all green (the ISS sets no limits: unchanged).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/config/vehicle_config.hpp core/src/config/vehicle_config.cpp core/include/starsim/sim/station_limits.hpp core/src/sim/station_limits.cpp core/CMakeLists.txt core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/src/sim/flight_session_proximity.cpp core/src/sim/flight_session_rendezvous.cpp core/src/sim/flight_session_docking.cpp config/vehicles/starship_depot.toml core/tests/refuelling_test_support.hpp core/tests/test_refuelling_docking.cpp
git commit -m "feat(sim): target block - the depot as a one-stage station with its approach limits"
git push origin main
```

---

### Task 3: R-bar approach from below and the probe/drogue capture envelope

**Files:**
- Modify: `core/include/starsim/gnc/proximity_geometry.hpp:20` (`DockingApproach::RBarBelow`), `:50-90` (move
  `CaptureEnvelope` above `ProximityParameters`, add `envelope`), declare `probe_drogue_capture_envelope()`
- Modify: `core/src/gnc/proximity_geometry.cpp:17-23` (`docking_approach`), add the envelope function
- Modify: `core/src/gnc/proximity_guidance.cpp` (`start_segment`: on-axis rule; `transitions`: `p_.envelope`)
- Test: `core/tests/test_refuelling_docking.cpp`

**Interfaces:**
- Produces: `gnc::DockingApproach::RBarBelow` (port axis towards nadir: `u.z > cos 45 deg`), `CaptureEnvelope
  ProximityParameters::envelope` (default: the IDSS values), `gnc::CaptureEnvelope gnc::probe_drogue_capture_envelope()`.
- Consumes: `proximity_test::CwSim`, `proximity_test::run` (`core/tests/proximity_test_support.hpp`).

- [ ] **Step 1: Write the failing tests**

Append to `test_refuelling_docking.cpp` (includes `proximity_test_support.hpp`, `starsim/gnc/proximity_guidance.hpp`):

```cpp
namespace {

// CW model of the tanker under the depot: target port axes in LVLH x = nadir (dorsal normal), z = +V (nose),
// y = z x x = -y_lvlh. Authority along the target port axes (Task 1 tanker at ~490 t): +-x_t = the ship's +-z,
// +-y_t its +-y, +-z_t its +-x.
proximity_test::CwSim depot_cw() {
    proximity_test::CwSim s;
    s.n = std::sqrt(constants::earth_gm_m3ps2 / std::pow(constants::wgs84_semi_major_axis_m + 200.0e3, 3));
    s.port_to_lvlh = math::Mat3::from_columns({0, 0, 1}, {0, -1, 0}, {1, 0, 0});
    s.port_lever_lvlh = {2.0, 0.0, 5.0};  // [test] depot CG ~18 m up the hull, port at 20 m, 5 m off the axis
    s.chaser_lever_m = 5.0;
    const double m = 490.0e3;
    s.authority = gnc::AxisAcceleration{{11338.0 / m, 11338.0 / m, 5670.0 / m, 5670.0 / m, 3000.0 / m, 3000.0 / m}};
    s.propellant_kg = 200.0e3;
    s.kg_per_mps = m / (70.0 * constants::standard_gravity_mps2);  // cold gas Isp 70 s
    return s;
}

gnc::ProximityParameters depot_parameters() {
    gnc::ProximityParameters p;
    p.contact_plane_m = 0.80;  // probe/drogue first touch (Task 5: cone depth + stroke)
    p.envelope = gnc::probe_drogue_capture_envelope();
    return p;
}

bool saw(const gnc::ProximityGuidance& g, gnc::ProximityPhase phase) {
    for (const auto& e : g.events()) {
        if (e.phase == phase) return true;
    }
    return false;
}

}  // namespace

TEST_CASE("A nadir port is approached along the R-bar from below, no fly-around", "[refuelling][proximity]") {
    CHECK(gnc::docking_approach({0.0, 0.0, 1.0}) == gnc::DockingApproach::RBarBelow);
    CHECK(gnc::docking_approach({1.0, 0.0, 0.0}) == gnc::DockingApproach::VBar);   // ISS paths unchanged
    CHECK(gnc::docking_approach({0.0, 0.0, -1.0}) == gnc::DockingApproach::RBar);
    const auto path = gnc::proximity_path(gnc::DockingApproach::RBarBelow, gnc::ProximityParameters{});
    REQUIRE(path.size() == 3);
    CHECK(path[1].on_axis);
    CHECK(path[2].on_axis);
}

TEST_CASE("Tanker from Waypoint 0 to contact under the depot inside the probe/drogue envelope", "[refuelling][proximity]") {
    auto s = depot_cw();
    s.cg = {{0.0, 0.0, 400.0}, {}};  // Waypoint 0 at rest, 400 m below the depot CG
    gnc::ProximityGuidance g(gnc::DockingApproach::RBarBelow, depot_parameters());
    const double t = proximity_test::run(g, s, 3.0 * 3600.0);
    REQUIRE(g.contact());
    INFO(g.contact()->envelope.reason);
    CHECK(g.contact()->envelope.inside);
    CHECK(std::hypot(g.contact()->port.position_m.y, g.contact()->port.position_m.z) < 0.10);
    CHECK_FALSE(saw(g, gnc::ProximityPhase::Transfer));  // straight up the axis
    CHECK_FALSE(saw(g, gnc::ProximityPhase::Flyaround));
    CHECK_FALSE(saw(g, gnc::ProximityPhase::Abort));
    CHECK(saw(g, gnc::ProximityPhase::Approach));
    CHECK(t < 2.0 * 3600.0);
}

TEST_CASE("A retreat below the depot stops at Waypoint 2 - the -x thrusters hold the Coriolis drift", "[refuelling][proximity]") {
    auto s = depot_cw();
    s.cg = {{0.0, 0.0, 400.0}, {}};
    gnc::ProximityGuidance g(gnc::DockingApproach::RBarBelow, depot_parameters());
    proximity_test::run(g, s, 3.0 * 3600.0);
    REQUIRE(g.phase() == gnc::ProximityPhase::Contact);
    g.capture_failed(s.input(), "Überlast");
    for (double t = 0.0; t < 1800.0 && g.phase() != gnc::ProximityPhase::Waypoint; t += 0.05) {
        s.step(g.update(s.input()).translation_fraction_port, 0.05);
    }
    REQUIRE(g.phase() == gnc::ProximityPhase::Waypoint);  // Waypoint 2, waiting for the go
    for (double t = 0.0; t < 600.0; t += 0.05) s.step(g.update(s.input()).translation_fraction_port, 0.05);
    const auto in = s.input();
    CHECK(in.port.position_m.x == Catch::Approx(20.0).margin(1.0));
    CHECK(std::hypot(in.port.position_m.y, in.port.position_m.z) < 1.0);  // held, no forward drift
    CHECK(norm(in.port.velocity_mps) < 0.01);
}

TEST_CASE("The probe/drogue envelope is the spec's", "[refuelling][proximity]") {
    const auto e = gnc::probe_drogue_capture_envelope();
    CHECK(e.axial_min_mps == 0.03);
    CHECK(e.axial_max_mps == 0.10);
    CHECK(e.offset_max_m == 0.10);
    CHECK(e.angle_max_rad == Catch::Approx(1.0 * constants::deg_to_rad));
    CHECK(gnc::ProximityParameters{}.envelope.axial_min_mps == 0.05);  // IDSS default unchanged
}
```

- [ ] **Step 2: Run to verify they fail**

Build: compile errors (`RBarBelow`, `envelope`, `probe_drogue_capture_envelope`).

- [ ] **Step 3: Implement**

`proximity_geometry.hpp`: `enum class DockingApproach { VBar, RBar, RBarBelow };` with the comment "RBar: a zenith port
(approach from above after a fly-around, ISS IDA-3); RBarBelow: a nadir port (the depot's drogues, approach straight up
the R-bar from Waypoint 0)". Move `struct CaptureEnvelope` (unchanged) above `ProximityParameters`, add to
`ProximityParameters` after `contact_plane_m`:

```cpp
    CaptureEnvelope envelope;  // checked at the contact plane (IDSS default; probe/drogue: probe_drogue_capture_envelope)
```

and after `capture_envelope_check`:

```cpp
// Probe/drogue initial contact conditions (refuelling r3 spec) [E]: closing 0.03-0.10 m/s, offset <= 0.10 m, angle and
// roll <= 1 deg; lateral speed and rates as IDSS.
CaptureEnvelope probe_drogue_capture_envelope();
```

`proximity_geometry.cpp`:

```cpp
DockingApproach docking_approach(const Vec3& port_axis_lvlh) {
    const Vec3 u = normalized(port_axis_lvlh);
    const double dominant = std::cos(45.0 * constants::deg_to_rad);
    if (u.x > dominant) return DockingApproach::VBar;
    if (-u.z > dominant) return DockingApproach::RBar;
    if (u.z > dominant) return DockingApproach::RBarBelow;
    throw std::invalid_argument("Kein Anflugweg für diesen Port");
}

CaptureEnvelope probe_drogue_capture_envelope() {
    CaptureEnvelope e;
    e.axial_min_mps = 0.03;
    e.axial_max_mps = 0.10;
    e.offset_max_m = 0.10;
    e.angle_max_rad = 1.0 * constants::deg_to_rad;
    e.roll_max_rad = 1.0 * constants::deg_to_rad;
    return e;
}
```

Extend the file's header comment: "Refuelling r3: a nadir port (the depot) is approached straight up the R-bar from
Waypoint 0; no fly-around."

`proximity_guidance.cpp`, `start_segment`: fly the axial approach when the chaser is already on the axis beyond the
waypoint (the depot's Waypoint 0 is on its R-bar; the ISS's is not on an IDA axis):

```cpp
    const bool on_axis_beyond = w.on_axis && in.port.position_m.x > w.position_m.x &&
                                lateral_m(in.port.position_m) <= corridor_bound_m(in.port.position_m.x);
    if (w.on_axis && (norm(target_cg(in, target_)) < p_.keep_out_radius_m || on_axis_beyond)) {
        enter(ProximityPhase::Approach, "Anflug auf " + w.name, in.epoch);
        return;
    }
```

`transitions`, FinalApproach: `capture_envelope_check(in.port, p_.envelope)`.

- [ ] **Step 4: Run the tests**

`ctest --test-dir build -L "refuelling|proximity|iss|docking" -j 24 --output-on-failure > tmp/r3_t3.txt 2>&1` -
green. If the retreat case fails on the lateral hold, print the event list with `STARSIM_VERBOSE` and look at the
authority of `-z_t` first (it must be 3000 N / m).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/gnc/proximity_geometry.hpp core/src/gnc/proximity_geometry.cpp core/src/gnc/proximity_guidance.cpp core/tests/test_refuelling_docking.cpp
git commit -m "feat(gnc): R-bar approach from below for the depot, probe/drogue capture envelope"
git push origin main
```

---

### Task 4: Probe/drogue interface contact

**Files:**
- Modify: `core/include/starsim/contact/docking_interface.hpp:52-62` (`InterfaceContact`: `torque_port_nm`, `gap_m`)
- Modify: `core/src/contact/docking_interface.cpp` (IDSS sets `gap_m`)
- Create: `core/include/starsim/contact/probe_drogue_interface.hpp`, `core/src/contact/probe_drogue_interface.cpp`;
  Modify: `core/CMakeLists.txt` (after `src/contact/docking_interface.cpp`)
- Test: `core/tests/test_refuelling_docking.cpp`

**Interfaces:**
- Produces: `InterfaceContact::torque_port_nm` (pure couple on the chaser about `ring_centre_m`; IDSS: zero),
  `InterfaceContact::gap_m` (IDSS: ring face beyond the petal overlap, `ring_centre.x - 2 h`; probe/drogue: the lowest
  head over the mouth plane).
- Produces (`contact`): `struct ProbeDrogueParameters` (fields below), `InterfaceContact probe_drogue_contact(const
  DockingPose& pose, double extension_m, const ProbeDrogueParameters& p)`, `ProbeDrogueParameters
  probe_drogue_parameters(const config::DockingPortConfig& chaser, const config::DockingPortConfig& target)`.

- [ ] **Step 1: Write the failing tests**

Append (include `starsim/contact/probe_drogue_interface.hpp`):

```cpp
namespace {

contact::ProbeDrogueParameters depot_probes() {
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    return contact::probe_drogue_parameters(tanker.stages.back().docking_ports.front(),
                                            depot.stages.back().docking_ports.front());
}

const math::Mat3 mated_axes = math::Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1});

contact::DockingPose pose_at(const Vec3& position_m, const Vec3& velocity_mps = {}, const math::Mat3& axes = mated_axes) {
    return {position_m, velocity_mps, axes, {}};
}

}  // namespace

TEST_CASE("Probe/drogue geometry from the r1 ports: probes and drogues pair up when mated", "[refuelling][contact]") {
    const auto p = depot_probes();
    REQUIRE(p.probes_m.size() == 4);
    REQUIRE(p.drogues_m.size() == 4);
    for (const auto& probe : p.probes_m) {
        CHECK(probe.x == 0.0);  // in the interface plane
        const Vec3 mated{-probe.x, -probe.y, probe.z};
        double nearest = 1e9;
        for (const auto& d : p.drogues_m) nearest = std::min(nearest, std::hypot(mated.y - d.y, mated.z - d.z));
        CHECK(nearest < 1e-9);
    }
}

TEST_CASE("Probe/drogue contact: free above the mouths, centring in the cone, seated heads latch", "[refuelling][contact]") {
    const auto p = depot_probes();
    const double first = p.cone_depth_m + p.extension_m;  // port separation at the first touch
    auto c = contact::probe_drogue_contact(pose_at({first + 0.01, 0.0, 0.0}), p.extension_m, p);
    CHECK_FALSE(c.any_contact);
    CHECK(c.gap_m == Catch::Approx(0.01).margin(1e-9));

    // 0.08 m off-centre (along y), halfway down the cone: the walls push towards the axis and out
    c = contact::probe_drogue_contact(pose_at({first - 0.40, 0.08, 0.0}), p.extension_m, p);
    REQUIRE(c.any_contact);
    CHECK(c.petal_contact);
    CHECK(c.force_port_n.y < 0.0);
    CHECK(c.force_port_n.x > 0.0);
    CHECK_FALSE(c.latch_ready);

    // centred, heads in the seats at full extension: latch ready, no penetration force
    c = contact::probe_drogue_contact(pose_at({p.extension_m + 0.005, 0.0, 0.0}), p.extension_m, p);
    CHECK(c.latch_ready);
    CHECK(norm(c.force_port_n) < 1e-6);

    // mated (retracted): every head on its seat, no force
    c = contact::probe_drogue_contact(pose_at({0.0, 0.0, 0.0}), 0.0, p);
    CHECK(norm(c.force_port_n) < 1e-6);
}

TEST_CASE("Probe/drogue contact: a head beside the mouth strikes the plate, no latch; tilted heads give a torque",
          "[refuelling][contact]") {
    const auto p = depot_probes();
    const double first = p.cone_depth_m + p.extension_m;
    auto c = contact::probe_drogue_contact(pose_at({first - 0.05, 0.6, 0.0}, {-0.05, 0.0, 0.0}), p.extension_m, p);
    CHECK(c.tip_strike);
    CHECK(c.force_port_n.x > 0.0);
    CHECK_FALSE(c.latch_ready);

    // rolled 180 deg about the docking axis and pushed in: no head reaches a seat it could latch in (the roll limit)
    const math::Mat3 rolled = mated_axes * math::Mat3::from_columns({1, 0, 0}, {0, -1, 0}, {0, 0, -1});
    c = contact::probe_drogue_contact(pose_at({p.extension_m, 0.0, 0.0}, {}, rolled), p.extension_m, p);
    CHECK_FALSE(c.latch_ready);

    // pitched 2 deg about y: the deeper pair touches first - the contact force has a moment about the head centre
    const double a = 2.0 * constants::deg_to_rad;
    const math::Mat3 pitch = math::Mat3::from_columns({std::cos(a), 0, -std::sin(a)}, {0, 1, 0}, {std::sin(a), 0, std::cos(a)});
    c = contact::probe_drogue_contact(pose_at({p.extension_m + 0.10, 0.0, 0.0}, {}, mated_axes * pitch), p.extension_m, p);
    REQUIRE(c.any_contact);
    CHECK(std::abs(c.torque_port_nm.y) > 0.0);
}
```

- [ ] **Step 2: Run to verify they fail**

Build: `probe_drogue_interface.hpp` missing.

- [ ] **Step 3: Implement**

`docking_interface.hpp`, `InterfaceContact` - add after `force_port_n`:

```cpp
    math::Vec3 torque_port_nm;  // pure couple on the chaser about ring_centre_m (probe/drogue: the heads' forces)
```

and after `capture_radius_m`:

```cpp
    // How far the interface is from meshing (> 0: apart): IDSS the ring face beyond the petal overlap, probe/drogue the
    // lowest head over the mouth plane. The mechanism calls a capture failed when it stays above bounce_gap_m.
    double gap_m = std::numeric_limits<double>::infinity();
```

(`#include <limits>`.) In `docking_interface_contact` after `const double gap = c.ring_centre_m.x;` add
`c.gap_m = gap - 2.0 * p.petal_height_m;`.

`core/include/starsim/contact/probe_drogue_interface.hpp`:

```cpp
#pragma once

#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/docking_interface.hpp"

namespace starsim::contact {

// Four-probe interface of the refuelling ships (refuelling r3), all values [E] (SpaceX publishes none). The chaser's
// probes (active) reach into conical drogues on the target (passive). Passive port frame: x out of the target, the
// drogue mouths in the plane x = 0, each cone narrowing to its seat at x = -cone_depth_m. A probe stands
// cone_depth_m + extension_m on the chaser's interface plane, so with the extension retracted to 0 and the pose mated
// (0, axes (-x, -y, +z)) every head sits in its seat. Each head meets its nearest drogue: the cone wall (normal towards
// the axis and out), the seat (the probe's attenuator) and, beside the mouth, the drogue plate.
struct ProbeDrogueParameters {
    std::vector<math::Vec3> probes_m;   // probe roots, chaser port frame (x = 0)
    std::vector<math::Vec3> drogues_m;  // mouth centres, passive port frame (x = 0)
    double cone_depth_m = 0.50;         // [E] mouth to seat
    double mouth_radius_m = 0.35;       // [E]
    double seat_radius_m = 0.06;        // [E]
    double head_radius_m = 0.05;        // [E] capture radius = mouth - head = 0.30 m (3x the envelope offset)
    double extension_m = 0.30;          // [E] probe stroke beyond the seated length, retracted after soft capture
    double wall_stiffness_n_per_m = 2.0e5;  // [E] per head
    double wall_damping_ns_per_m = 4.0e4;   // [E]
    double seat_stiffness_n_per_m = 5.0e5;  // [E] per head (attenuator)
    double seat_damping_ns_per_m = 1.0e5;   // [E]
    double plate_stiffness_n_per_m = 2.0e5; // [E] a head beside the mouth
    double plate_damping_ns_per_m = 4.0e4;  // [E]
    double friction_coefficient = 0.1;      // [E] as IDSS
    double friction_slip_mps = 1e-3;
    double latch_gap_m = 0.01;              // [E] a head this close to its seat, centred: its capture latch closes
    double latch_tilt_max_rad = 2.0 * constants::deg_to_rad;  // [E]
    double latch_roll_max_rad = 2.0 * constants::deg_to_rad;  // [E]
};

// The contact of every probe head (sums: force, torque about the mean head position = ring_centre_m, compression = the
// +x part). latch_ready: one head seated and the tilt / roll within the latch limits (each probe latches alone; the
// mechanism's joint takes over). ring_contact = a head on a seat, petal_contact = a head on a cone wall, tip_strike = a
// head on the plate beside a mouth.
InterfaceContact probe_drogue_contact(const DockingPose& pose, double extension_m, const ProbeDrogueParameters& p);

// Probes from the chaser port's elements_m, drogues from the target's, in their port frames (x = axis, z = up).
ProbeDrogueParameters probe_drogue_parameters(const config::DockingPortConfig& chaser,
                                              const config::DockingPortConfig& target);

}  // namespace starsim::contact
```

`core/src/contact/probe_drogue_interface.cpp`:

```cpp
#include "starsim/contact/probe_drogue_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace starsim::contact {

using math::Vec3;

namespace {

math::Vec3 friction(const Vec3& v_tangential, double normal_n, const ProbeDrogueParameters& p) {
    const double speed = norm(v_tangential);
    if (speed <= 0.0 || normal_n <= 0.0) return {};
    const double scale = std::min(1.0, speed / p.friction_slip_mps);
    return -(p.friction_coefficient * normal_n * scale / speed) * v_tangential;
}

Vec3 in_port_frame(const Vec3& point_stage_m, const config::DockingPortConfig& port) {
    const Vec3 d = point_stage_m - port.position_m;
    return {dot(d, port.axis), dot(d, cross(port.up, port.axis)), dot(d, port.up)};
}

}  // namespace

ProbeDrogueParameters probe_drogue_parameters(const config::DockingPortConfig& chaser,
                                              const config::DockingPortConfig& target) {
    ProbeDrogueParameters p;
    for (const auto& e : chaser.elements_m) p.probes_m.push_back(in_port_frame(e, chaser));
    for (const auto& e : target.elements_m) p.drogues_m.push_back(in_port_frame(e, target));
    return p;
}

InterfaceContact probe_drogue_contact(const DockingPose& pose, double extension_m, const ProbeDrogueParameters& p) {
    InterfaceContact c;
    c.capture_radius_m = p.mouth_radius_m - p.head_radius_m;
    const Vec3 x{1.0, 0.0, 0.0};
    const Vec3 xc = column(pose.chaser_axes, 0);  // the probes point along it (towards the passive port)
    const double length = p.cone_depth_m + extension_m;
    const double tan_cone = (p.mouth_radius_m - p.seat_radius_m) / p.cone_depth_m;
    const double cos_cone = 1.0 / std::sqrt(1.0 + tan_cone * tan_cone), sin_cone = tan_cone * cos_cone;
    std::vector<Vec3> heads, forces;
    bool seated = false;
    for (const auto& probe : p.probes_m) {
        const Vec3 head = pose.position_m + pose.chaser_axes * probe + length * xc;
        const Vec3 v = pose.velocity_mps + cross(pose.angular_rate_radps, head - pose.position_m);
        heads.push_back(head);
        c.gap_m = std::min(c.gap_m, head.x);
        const Vec3* mouth = &p.drogues_m.front();
        for (const auto& d : p.drogues_m) {
            if (std::hypot(head.y - d.y, head.z - d.z) < std::hypot(head.y - mouth->y, head.z - mouth->z)) mouth = &d;
        }
        const Vec3 lateral{0.0, head.y - mouth->y, head.z - mouth->z};
        const double r = norm(lateral);
        const double depth = -head.x;
        Vec3 f;
        if (depth > 0.0 && r > c.capture_radius_m) {  // beside the mouth: the plate
            const double normal = std::max(0.0, p.plate_stiffness_n_per_m * depth - p.plate_damping_ns_per_m * v.x);
            f = normal * x + friction({0.0, v.y, v.z}, normal, p);
            c.tip_strike = c.tip_strike || normal > 0.0;
        } else if (depth > 0.0) {
            const double allowed = p.mouth_radius_m - tan_cone * std::min(depth, p.cone_depth_m) - p.head_radius_m;
            if (r > allowed && r > 0.0) {  // the cone wall: normal towards the axis and out of the cone
                const Vec3 radial = lateral / r;
                const Vec3 n = -cos_cone * radial + sin_cone * x;
                const double vn = dot(v, n);
                const double fn = std::max(0.0, p.wall_stiffness_n_per_m * (r - allowed) * cos_cone - p.wall_damping_ns_per_m * vn);
                f += fn * n + friction(v - vn * n, fn, p);
                c.petal_contact = c.petal_contact || fn > 0.0;
            }
            if (depth > p.cone_depth_m) {  // on the seat: the probe's attenuator
                const double fn = std::max(0.0, p.seat_stiffness_n_per_m * (depth - p.cone_depth_m) - p.seat_damping_ns_per_m * v.x);
                f += fn * x + friction({0.0, v.y, v.z}, fn, p);
                c.ring_contact = c.ring_contact || fn > 0.0;
            }
            seated = seated || (depth >= p.cone_depth_m - p.latch_gap_m &&
                                r <= p.seat_radius_m - p.head_radius_m + p.latch_gap_m);
        }
        c.any_contact = c.any_contact || norm(f) > 0.0;
        c.compression_n += std::max(0.0, f.x);
        forces.push_back(f);
    }
    Vec3 centre;
    for (const auto& h : heads) centre += h;
    c.ring_centre_m = centre / static_cast<double>(heads.size());
    for (size_t i = 0; i < heads.size(); ++i) {
        c.force_port_n += forces[i];
        c.torque_port_nm += cross(heads[i] - c.ring_centre_m, forces[i]);
    }
    const double tilt = std::acos(std::clamp(-xc.x, -1.0, 1.0));
    const Vec3 zc = column(pose.chaser_axes, 2);
    const double roll = std::atan2(-zc.y, zc.z);
    c.latch_ready = seated && tilt <= p.latch_tilt_max_rad && std::abs(roll) <= p.latch_roll_max_rad;
    return c;
}

}  // namespace starsim::contact
```

(`ProbeDrogueParameters` uses `constants::deg_to_rad`: include `starsim/math/constants.hpp` in the header.)

- [ ] **Step 4: Run the tests**

`ctest --test-dir build -L "refuelling|docking|contact" -j 24 --output-on-failure > tmp/r3_t4.txt 2>&1` - green
(IDSS unchanged: `gap_m` is new and not yet read).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/contact/docking_interface.hpp core/src/contact/docking_interface.cpp core/include/starsim/contact/probe_drogue_interface.hpp core/src/contact/probe_drogue_interface.cpp core/CMakeLists.txt core/tests/test_refuelling_docking.cpp
git commit -m "feat(contact): probe/drogue interface - four probe heads in conical drogues"
git push origin main
```

---

### Task 5: Probe/drogue mechanism, arming, hull sampling and the session docking

**Files:**
- Modify: `core/include/starsim/contact/docking_mechanism.hpp`, `core/src/contact/docking_mechanism.cpp`
- Create: `core/include/starsim/contact/probe_drogue_mechanism.hpp`, `core/src/contact/probe_drogue_mechanism.cpp`
  (`probe_drogue_mechanism` parameters); Modify: `core/CMakeLists.txt`
- Modify: `core/src/sim/docking_sim.cpp:41-44` (coast limit), `core/src/sim/flight_session_docking.cpp:53-67`
  (arming), `:166-170` (re-arm), `core/src/sim/flight_session_proximity.cpp:114-120` (contact plane, envelope)
- Modify: `core/src/contact/station_collision.cpp:10-32` (nose ports only; leg struts),
  `core/include/starsim/contact/station_collision.hpp` (comment, `leg_root_station_m`)
- Modify: `config/vehicles/starship_depot.toml` (collision capsules)
- Modify: `core/tests/refuelling_test_support.hpp` (`TankerAtPort`)
- Test: `core/tests/test_refuelling_docking.cpp`

**Interfaces:**
- Consumes: `ProbeDrogueParameters`, `probe_drogue_contact`, `probe_drogue_parameters` (Task 4);
  `probe_drogue_capture_envelope` (Task 3); `refuelling_test::DepotSession`, `tanker_config`, `depot_config` (Task 2).
- Produces (`DockingMechanismParameters`): `std::optional<ProbeDrogueParameters> probe_drogue;`,
  `double joint_rotation_frequency_radps = 0.0;`, `std::string limit_name = "IDSS-Grenze";`.
- Produces (`DockingMechanism`): `double full_extension_m() const;`, `double first_contact_m() const;`,
  `double reach_m() const;`.
- Produces (`contact`): `DockingMechanismParameters probe_drogue_mechanism(const config::DockingPortConfig& chaser, const config::DockingPortConfig& target);`
- Produces (`StationCollisionParameters`): `double leg_root_station_m = 4.0;` ([E] strut root above the stage bottom).
- Produces (tests): `refuelling_test::TankerAtPort` (Step 1).

- [ ] **Step 1: Write the test support and the failing tests**

Add to `refuelling_test_support.hpp` (include `starsim/frames/lvlh.hpp`):

```cpp
// The chaser (ship alone) with its transfer_dock at `position_port_m` in the depot's transfer_dock frame, in the
// docking attitude (axes (-x, -y, +z) of the depot port's: rolled 180 deg, dorsal up), moving with the depot's
// rotating frame plus `velocity_port_mps`. Body 0 = chaser, body 1 = depot.
struct TankerAtPort : DepotSession {
    TankerAtPort(const Vec3& position_port_m, const Vec3& velocity_port_mps = {},
                 const config::VehicleConfig& chaser = tanker_config(200.0e3)) {
        sim::FlightSession probe(ephemeris, earth, config_dir);  // the ideal-attitude depot's port
        probe.add_station(depot_config(depot_propellant_kg), depot, t0);
        const auto sk = vehicle::port_kinematics(probe.body(0), "transfer_dock");
        const vehicle::Vehicle v(chaser, config_dir);
        const auto& port = chaser.stages.back().docking_ports.front();
        const math::Mat3 port_axes_body = math::Mat3::from_columns(port.axis, cross(port.up, port.axis), port.up);
        const math::Mat3 desired = sk.axes_gcrf * math::Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1});
        const math::Mat3 r = desired * transpose(port_axes_body);
        const Vec3 lever_body = port.position_m - v.mass_properties().cg_point_m();
        const Vec3 port_gcrf = sk.position_gcrf_m + sk.axes_gcrf * position_port_m;
        const Vec3 cg = port_gcrf - r * lever_body;
        const Vec3 omega = sk.angular_velocity_gcrf_radps;
        const Vec3 velocity = sk.velocity_gcrf_mps + sk.axes_gcrf * velocity_port_mps + cross(omega, cg - sk.position_gcrf_m);
        const math::Quat q = math::Quat::from_matrix(r);
        session.place_body(chaser, {cg, velocity, q, q.inverse_rotate(omega)}, t0);
        session.add_station(depot_config(depot_propellant_kg), depot, t0);
        session.set_attitude_mode(gnc::lvlh_hold(gnc::lvlh_attitude_gcrf({cg, velocity}).conjugate() * q));
    }
    // Advances until `done` or `limit_s`; returns the elapsed seconds.
    template <class Done> double fly(Done done, double limit_s, double step_s = 1.0) {
        double t = 0.0;
        while (t < limit_s && !done()) {
            session.advance_to(session.epoch() + step_s);
            t += step_s;
        }
        return t;
    }
};
```

(`using math::Vec3;` inside the namespace; check that `gnc::lvlh_attitude_gcrf` is declared in
`starsim/gnc/orbital_attitude.hpp` - `flight_session_proximity.cpp:162` uses it.)

Append to `test_refuelling_docking.cpp` (includes `starsim/contact/probe_drogue_mechanism.hpp`,
`starsim/contact/station_collision.hpp`, `starsim/vehicle/port_mating.hpp`):

```cpp
TEST_CASE("IDSS reach and contact plane are unchanged by the generalised mechanism", "[refuelling][docking]") {
    const contact::DockingMechanism idss;
    CHECK(idss.full_extension_m() == 0.30);
    CHECK(idss.first_contact_m() == Catch::Approx(0.70));
    CHECK(idss.reach_m() == Catch::Approx(0.70 + 1.2));
}

TEST_CASE("Probe/drogue mechanism: 0.10 m/s soft capture within the loads, retraction, latches, docked",
          "[refuelling][docking]") {
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    contact::DockingMechanism m(contact::probe_drogue_mechanism(tanker.stages.back().docking_ports.front(),
                                                                depot.stages.back().docking_ports.front()));
    CHECK(m.first_contact_m() == Catch::Approx(0.80));
    // relative 1-D dynamics of a 490 t tanker against a 1,600 t depot at 200 Hz, centred, closing at 0.10 m/s
    const double mc = 490.0e3, ms = 1600.0e3, mu = mc * ms / (mc + ms), dt = 0.005;
    double x = m.first_contact_m() + 0.02, v = -0.10;
    double t = 0.0;
    for (; t < 1500.0 && m.phase() != contact::DockingPhase::Docked; t += dt) {
        const auto w = m.update(t, {{x, 0.0, 0.0}, {v, 0.0, 0.0}, math::Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1}), {}},
                                mc, 1.0e8, dt);
        v += w.force_port_n.x / mu * dt;
        x += v * dt;
    }
    INFO(m.status());
    CHECK(m.phase() == contact::DockingPhase::Docked);
    CHECK(m.peaks().compression_n < m.parameters().dynamic_compression_limit_n);
    CHECK(m.peaks().compression_n > 10.0e3);
    CHECK(t < 20.0 * 60.0);  // soft capture -> docked: attenuation 120 s + retraction 300 s + latches 180 s
}

TEST_CASE("Probe/drogue mechanism: 0.25 m/s overloads", "[refuelling][docking]") {
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    contact::DockingMechanism m(contact::probe_drogue_mechanism(tanker.stages.back().docking_ports.front(),
                                                                depot.stages.back().docking_ports.front()));
    const double mc = 490.0e3, mu = mc * 1600.0e3 / (mc + 1600.0e3), dt = 0.005;
    double x = m.first_contact_m() + 0.02, v = -0.25;
    for (double t = 0.0; t < 60.0 && m.phase() != contact::DockingPhase::Failed; t += dt) {
        const auto w = m.update(t, {{x, 0.0, 0.0}, {v, 0.0, 0.0}, math::Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1}), {}},
                                mc, 1.0e8, dt);
        v += w.force_port_n.x / mu * dt;
        x += v * dt;
    }
    CHECK(m.phase() == contact::DockingPhase::Failed);
    CHECK(m.failure() == "Überlast");
}

TEST_CASE("Hull sampling: the tanker's whole hull clears the mated depot by ~1 m, 1.5 m closer it collides",
          "[refuelling][docking]") {
    const auto tanker = config::top_stage_config(config::load_vehicle(config_dir, "starship_tanker"));
    const auto depot = config::top_stage_config(config::load_vehicle(config_dir, "starship_depot"));
    const contact::StationCollisionParameters cp;
    const auto points = contact::hull_sample_points(tanker, cp);
    double top = 0.0;
    for (const auto& q : points) top = std::max(top, q.x);
    CHECK(top > 50.0);  // not cut at the dorsal port (x = 20 m)
    const auto t = vehicle::mated_stage_transform(tanker.stages.back().docking_ports.front(),
                                                  depot.stages.back().docking_ports.front());
    const auto shape = contact::station_shape(depot);
    std::vector<Vec3> mated, closer, zero(points.size());
    for (const auto& q : points) {
        mated.push_back(t.apply(q));
        closer.push_back(t.apply(q) - Vec3{0.0, 0.0, 1.5});  // towards the depot axis (its dorsal side is +z)
    }
    CHECK_FALSE(contact::hull_contact(shape, mated, zero, {}, cp).any);
    CHECK(contact::hull_contact(shape, closer, zero, {}, cp).any);
}

TEST_CASE("HLS legs clear the depot when mated (r1 open point)", "[refuelling][docking][hls]") {
    const auto hls = config::top_stage_config(config::load_vehicle(config_dir, "starship_hls"));
    const auto depot = config::top_stage_config(config::load_vehicle(config_dir, "starship_depot"));
    const contact::StationCollisionParameters cp;
    const auto points = contact::hull_sample_points(hls, cp);
    double reach = 0.0;
    for (const auto& q : points) reach = std::max(reach, std::hypot(q.y, q.z));
    CHECK(reach == Catch::Approx(12.0).margin(0.01));  // the leg feet are sampled
    const auto t = vehicle::mated_stage_transform(hls.stages.back().docking_ports.front(),
                                                  depot.stages.back().docking_ports.front());
    std::vector<Vec3> mated, zero(points.size());
    for (const auto& q : points) mated.push_back(t.apply(q));
    const auto contact = contact::hull_contact(contact::station_shape(depot), mated, zero, {}, cp);
    CHECK_FALSE(contact.any);
}

TEST_CASE("Tanker from Waypoint 2 to hard capture at the depot", "[refuelling][docking][proximity]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({20.0, 0.0, 0.0});
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    REQUIRE(f.session.start_proximity("transfer_dock", p));
    const double t = f.fly([&] { return f.session.docked(); }, 3600.0);
    INFO(f.session.docking_status() << " / " << f.session.proximity_status());
    REQUIRE(f.session.docked());
    CHECK(t < 40.0 * 60.0);
    const auto loads = *f.session.docking_loads();
    CHECK(loads.compression_n < 1.0e5);
    const auto merge = f.session.docking_merge_record();
    CHECK(norm(merge.momentum_after - merge.momentum_before) < 1e-6 * norm(merge.momentum_before));
    const auto& attached = f.session.body(1).vehicle().attached_mass();
    REQUIRE(attached);
    CHECK(attached->mass_kg == Catch::Approx(f.session.body(0).vehicle().mass_properties().mass_kg));
}

TEST_CASE("Tanker at the depot: 0.4 m/s overloads and retreats to Waypoint 2", "[refuelling][docking][proximity]") {
    test::spice_session();
    // inside Waypoint 2 the guidance brakes (0.037 m/s^2): it still hits at ~0.36 m/s, joint damper ~175 kN
    refuelling_test::TankerAtPort f({1.0, 0.0, 0.0}, {-0.40, 0.0, 0.0});
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    REQUIRE(f.session.start_proximity("transfer_dock", p));
    f.fly([&] { return f.session.docking_phase() == contact::DockingPhase::Failed; }, 120.0, 0.1);
    REQUIRE(f.session.docking_phase() == contact::DockingPhase::Failed);
    f.fly([&] { return f.session.proximity_phase() == gnc::ProximityPhase::Waypoint; }, 1800.0);
    CHECK(f.session.proximity_phase() == gnc::ProximityPhase::Waypoint);
    CHECK_THAT(f.session.proximity_status(), Catch::Matchers::ContainsSubstring("Wegpunkt 2"));
}

TEST_CASE("Tanker undocks from the depot: springs, departure out of the ellipsoid", "[refuelling][docking][slow]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0});
    // docked at once (ISS i6 scenario start)
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    REQUIRE(f.session.docked());
    REQUIRE(f.session.undock());
    f.fly([&] { return f.session.departure_phase() == gnc::DeparturePhase::Complete; }, 4.0 * 3600.0, 10.0);
    CHECK(f.session.departure_phase() == gnc::DeparturePhase::Complete);
    const auto sep = f.session.docking_separation_record();
    CHECK(sep.relative_speed_mps == Catch::Approx(std::sqrt(2.0 * 150.0 *
                                                            (1.0 / f.session.body(0).vehicle().mass_properties().mass_kg +
                                                             1.0 / f.session.body(1).vehicle().mass_properties().mass_kg))).epsilon(0.01));
    CHECK(f.session.docking_status() != "Kollision");
}
```

(Check the exact names of the merge / separation record getters (`docking_merge_record`,
`docking_separation_record`), `dock_in_place` and `set_rendezvous_target` visibility in `flight_session.hpp:670-695`
- the i6 departure mission calls them through `place_docked`; if `dock_in_place` is private, use the test seam the i6
tests use (`grep -n "dock_in_place" core/tests`). Also check the separation speed formula with the masses at
separation: the compound is detached first.)

- [ ] **Step 2: Run to verify they fail**

Build: `probe_drogue_mechanism.hpp`, `full_extension_m` missing.

- [ ] **Step 3: Implement**

`docking_mechanism.hpp`: include `starsim/contact/probe_drogue_interface.hpp` and `<optional>`; add to
`DockingMechanismParameters` after `interface`:

```cpp
    // Refuelling r3: the four-probe interface instead of the IDSS ring; its extension is the probe stroke.
    std::optional<ProbeDrogueParameters> probe_drogue;
```

after `joint_damping_ratio`:

```cpp
    // > 0: bending / torsion stiffness = the chaser's inertia * frequency^2 (a heavy chaser: the IDSS values above
    // are load limits of a 12 t pair and would leave a 500 t ship swinging for minutes)
    double joint_rotation_frequency_radps = 0.0;
```

after `torsion_limit_nm`: `std::string limit_name = "IDSS-Grenze";  // in the load warnings`.

Class: public `double full_extension_m() const; double first_contact_m() const; double reach_m() const;`, private
`InterfaceContact contact(const DockingPose& pose, double extension_m) const;` and members `double kb_ = 0.0, kt_ = 0.0;`
(stiffness used by the joint). Change `warn(const char*)` to `warn(const std::string&)`.

`docking_mechanism.cpp`:

```cpp
double DockingMechanism::full_extension_m() const {
    return p_.probe_drogue ? p_.probe_drogue->extension_m : p_.interface.ring_extension_m;
}

double DockingMechanism::first_contact_m() const {
    if (p_.probe_drogue) return p_.probe_drogue->cone_depth_m + p_.probe_drogue->extension_m;
    return p_.interface.ring_extension_m + 2.0 * p_.interface.petal_height_m;
}

double DockingMechanism::reach_m() const {
    if (!p_.probe_drogue) return first_contact_m() + p_.interface.interface_radius_m;
    double lateral = 0.0;
    for (const auto& q : p_.probe_drogue->probes_m) lateral = std::max(lateral, std::hypot(q.y, q.z));
    return first_contact_m() + lateral + p_.probe_drogue->mouth_radius_m;
}

InterfaceContact DockingMechanism::contact(const DockingPose& pose, double extension_m) const {
    return p_.probe_drogue ? probe_drogue_contact(pose, extension_m, *p_.probe_drogue)
                           : docking_interface_contact(pose, extension_m, p_.interface);
}
```

Replace: the constructor's `extension_m_(p.interface.ring_extension_m)` by a body `extension_m_ = full_extension_m();`;
in `fail` `extension_m_ = full_extension_m();`; in `before_capture`:

```cpp
    extension_m_ = full_extension_m();  // before capture the ring / the probes are fully extended
    const auto c = contact(pose, extension_m_);
    DockingWrench w{c.force_port_n, c.ring_centre_m, c.torque_port_nm};
```

the gains at the latch:

```cpp
        const double wr = p_.joint_rotation_frequency_radps;
        kb_ = wr > 0.0 ? inertia * wr * wr : p_.joint_bending_stiffness_nm_per_rad;
        kt_ = wr > 0.0 ? inertia * wr * wr : p_.joint_torsion_stiffness_nm_per_rad;
        cb_ = 2.0 * p_.joint_damping_ratio * std::sqrt(kb_ * inertia);
        ct_ = 2.0 * p_.joint_damping_ratio * std::sqrt(kt_ * inertia);
```

the bounce: `if (t - last_contact_s_ > p_.bounce_confirm_s && c.gap_m > p_.bounce_gap_m) {`; in `joint`
`const double kb = kb_, kt = kt_;`; in `start_docked` set `kb_`/`kt_` from the parameters (no joint runs while docked,
but keep them defined). Warnings: `warn("Zuglast über " + p_.limit_name);` etc. (the IDSS texts stay identical).
Update the file header comment: "Refuelling r3: the same sequence with the four-probe interface (probe_drogue): the
probes' stroke plays the ring's extension; structural latches play the hooks."

`core/include/starsim/contact/probe_drogue_mechanism.hpp`:

```cpp
#pragma once

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/docking_mechanism.hpp"

namespace starsim::contact {

// Mechanism parameters of the refuelling ships' probe/drogue docking (refuelling r3), all [E]: sized for a ~500 t
// tanker against a depot up to ~1,600 t. A 0.10 m/s contact peaks at the joint damper's m_c * omega * v (~49 kN for
// 490 t), below the 100 kN dynamic limit; 0.25 m/s overloads.
DockingMechanismParameters probe_drogue_mechanism(const config::DockingPortConfig& chaser,
                                                  const config::DockingPortConfig& target);

}  // namespace starsim::contact
```

`core/src/contact/probe_drogue_mechanism.cpp`:

```cpp
#include "starsim/contact/probe_drogue_mechanism.hpp"

#include "starsim/contact/probe_drogue_interface.hpp"

namespace starsim::contact {

DockingMechanismParameters probe_drogue_mechanism(const config::DockingPortConfig& chaser,
                                                  const config::DockingPortConfig& target) {
    DockingMechanismParameters m;
    m.probe_drogue = probe_drogue_parameters(chaser, target);
    m.joint_rotation_frequency_radps = 0.5;  // [E] as the translation: time constant 2 s
    m.end_stop_stiffness_n_per_m = 2.0e7;    // [E] the probes' hard stop for a ~500 t chaser
    m.hook_count = 4;                        // [E] one structural latch per probe
    m.separation_energy_j = 150.0;           // [E] ~0.03 m/s for 450 t / 1,600 t
    m.dynamic_compression_limit_n = 1.0e5;   // [E] 25 kN per probe
    m.static_compression_limit_n = 6.0e4;    // [E]
    m.tension_limit_n = 6.0e4;               // [E]
    m.shear_limit_n = 5.0e4;                 // [E]
    m.bending_limit_nm = 2.0e6;              // [E] probes +-8 m apart
    m.torsion_limit_nm = 2.0e6;              // [E]
    m.limit_name = "Auslegungsgrenze";
    return m;
}

}  // namespace starsim::contact
```

Session:
- `arm_docking` (flight_session_docking.cpp): replace the IDSS-only condition and the emplace:

```cpp
    // Active chaser port against a passive station port of the same mechanism (vehicle::port_mismatch): IDSS (ISS i5)
    // or the four probes into four drogues (refuelling r3).
    if (!cp || !sp || cp->mechanism != sp->mechanism || cp->soft_capture != config::SoftCaptureMode::Active ||
        sp->soft_capture != config::SoftCaptureMode::Passive) {
        return;
    }
    const auto parameters = cp->mechanism == config::DockingMechanism::ProbeDrogue
                                ? contact::probe_drogue_mechanism(*cp, *sp)
                                : contact::DockingMechanismParameters{};
    docking_.emplace(rendezvous_->chaser_port, rendezvous_->target_port, stages_of(chaser), stages_of(station), parameters);
```

  Update the file's design comment (IDSS or probe/drogue).
- Re-arm (`docking_after_tick`): `if (pose.position_m.x > m.first_contact_m() + m.parameters().rearm_distance_m) m.rearm();`
- `start_proximity`: replace the refusal text "both ports need IDSS soft capture" comment by "an active and a passive
  port of the same mechanism"; contact plane and envelope:

```cpp
    // i4 records "Kontakt" where the interface starts to mesh: IDSS petals, or probe heads at the drogue mouths
    p.contact_plane_m = docking_->mechanism().first_contact_m();
    if (docking_->mechanism().parameters().probe_drogue) p.envelope = gnc::probe_drogue_capture_envelope();
```

- `DockingSim::coast_limit_s`: `norm(ck.position_gcrf_m - sk.position_gcrf_m) - mechanism_.reach_m()`.

Hull sampling (`station_collision.cpp`): cut only at a nose port, sample the leg struts:

```cpp
        // a nose port (axis along +x: Crew Dragon) ends the hull below it; dorsal ports (the ships) do not
        if (top_stage && !s.docking_ports.empty() && s.docking_ports.front().axis.x > 0.5) {
            top = base + s.docking_ports.front().position_m.x - p.port_clearance_m;
        }
```

and after the ring loop of a stage:

```cpp
        // landing-leg struts (HLS): from the hull at leg_root_station_m to the deployed foot (refuelling r3: the legs
        // must clear the depot)
        const auto& legs = s.landing_legs;
        for (const double az : legs.azimuths_rad) {
            const Vec3 dir{0.0, std::cos(az), std::sin(az)};
            const Vec3 root = Vec3{base + p.leg_root_station_m, 0.0, 0.0} + r0 * dir;
            const Vec3 foot = Vec3{base + legs.foot_station_m, 0.0, 0.0} + legs.footprint_radius_m * dir;
            const int n = std::max(1, static_cast<int>(std::ceil(norm(foot - root) / p.ring_spacing_m)));
            for (int k = 0; k <= n; ++k) points.push_back(root + (static_cast<double>(k) / n) * (foot - root));
        }
```

(`using math::Vec3;`.) `StationCollisionParameters`: `double leg_root_station_m = 4.0;  // [E] leg strut root above the
stage bottom`. Update the header comment of `hull_sample_points`.

`config/vehicles/starship_depot.toml`, after the target block:

```toml
# Hull for the docking collision check (refuelling r3) [E]: tank section and nose as capsules (stage frame).
[[stage.collision]]
id = "tank_section"
kind = "capsule"
a_m = [4.5, 0.0, 0.0]
b_m = [32.0, 0.0, 0.0]
radius_m = 4.5

[[stage.collision]]
id = "nose"
kind = "capsule"
a_m = [36.0, 0.0, 0.0]
b_m = [47.0, 0.0, 0.0]
radius_m = 2.5
```

- [ ] **Step 4: Run the tests**

`ctest --test-dir build -L "refuelling|docking|proximity|iss|contact" -j 24 --output-on-failure > tmp/r3_t5.txt 2>&1`
(the ISS `[slow]` docking campaigns included) - green. With `STARSIM_DEBUG_DOCKING=1` the session cases print the
phases and loads; report the hard-capture time and the peak compression in the ledger.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/contact core/src/contact core/CMakeLists.txt core/src/sim/docking_sim.cpp core/src/sim/flight_session_docking.cpp core/src/sim/flight_session_proximity.cpp config/vehicles/starship_depot.toml core/tests/refuelling_test_support.hpp core/tests/test_refuelling_docking.cpp
git commit -m "feat(contact): probe/drogue docking mechanism at the depot, whole-hull and leg collision sampling"
git push origin main
```

---

### Task 6: Compound refresh when tank masses change

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp` (public `set_tank_mass`; private `refresh_compound`,
  member `compound_cg_point_m_`)
- Modify: `core/src/sim/flight_session_docking.cpp` (`merge_docked` stores the CG point; `refresh_compound`)
- Modify: `core/src/sim/flight_session.cpp` (`set_tank_mass`)
- Test: `core/tests/test_refuelling_docking.cpp`

**Interfaces:**
- Produces: `void sim::FlightSession::set_tank_mass(size_t body_index, size_t stage_index, std::string_view tank_id, double mass_kg);`
  - sets the tank (clamped, `Vehicle::set_tank_mass`); when the body is the docked chaser or the station of a docked
  compound, refreshes the compound: attached mass = the chaser's mass at its CG (hull-fixed), the station state moved by
  the CG shift (velocity kept), the chaser re-synced. r4 calls it for every transfer step.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST_CASE("Tank changes of a docked pair move the compound CG, not the hulls", "[refuelling][docking]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0});
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    REQUIRE(f.session.docked());
    const auto port_before = vehicle::port_kinematics(f.session.body(1), "transfer_dock").position_gcrf_m;
    const auto chaser_port_before = vehicle::port_kinematics(f.session.body(0), "transfer_dock").position_gcrf_m;
    const Vec3 v_before = f.session.body(1).state().velocity_gcrf_mps;
    const double total_before = f.session.body(1).vehicle().mass_properties().mass_kg;

    // 50 t of LOX from the tanker into the depot (what r4 will do in steps)
    const auto& tt = f.session.body(0).vehicle().stages()[0];
    double tanker_lox = 0.0, depot_lox = 0.0;
    for (const auto& k : tt.tanks) if (k.config.id == "lox_main") tanker_lox = k.mass_kg;
    for (const auto& k : f.session.body(1).vehicle().stages()[0].tanks) if (k.config.id == "lox_main") depot_lox = k.mass_kg;
    f.session.set_tank_mass(0, 0, "lox_main", tanker_lox - 50.0e3);
    f.session.set_tank_mass(1, 0, "lox_main", depot_lox + 50.0e3);

    CHECK(f.session.body(1).vehicle().mass_properties().mass_kg == Catch::Approx(total_before));
    CHECK(norm(vehicle::port_kinematics(f.session.body(1), "transfer_dock").position_gcrf_m - port_before) < 1e-6);
    CHECK(norm(vehicle::port_kinematics(f.session.body(0), "transfer_dock").position_gcrf_m - chaser_port_before) < 1e-6);
    CHECK(norm(f.session.body(1).state().velocity_gcrf_mps - v_before) < 1e-12);
    CHECK(f.session.body(1).vehicle().attached_mass()->mass_kg ==
          Catch::Approx(f.session.body(0).vehicle().mass_properties().mass_kg));
}

TEST_CASE("Emptied tanks keep the compound finite", "[refuelling][docking]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0});
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    for (const char* tank : {"lox_main", "ch4_main", "lox_header", "ch4_header"}) {
        f.session.set_tank_mass(0, 0, tank, 0.0);
        f.session.set_tank_mass(1, 0, tank, 0.0);
    }
    const auto& attached = f.session.body(1).vehicle().attached_mass();
    REQUIRE(attached);
    CHECK(std::isfinite(attached->position_m.x));
    CHECK(attached->mass_kg == Catch::Approx(255000.0).margin(1.0));  // the tanker's dry mass
    CHECK(std::isfinite(f.session.body(1).state().position_gcrf_m.x));
    f.session.advance_to(f.session.epoch() + 60.0);
    CHECK(f.session.docked());
}
```

(Tank access: check the `vehicle::Tank` field names in `vehicle.hpp` (`config.id`, `mass_kg`) and adapt.)

- [ ] **Step 2: Run to verify they fail**

Build: `FlightSession::set_tank_mass` missing.

- [ ] **Step 3: Implement**

`flight_session.hpp`, public near `docked()`:

```cpp
    // A tank of body `body_index` at run time (clamped; refuelling r4 transfer, test mode). For a docked pair the
    // compound follows: the chaser's mass stays attached at its (hull-fixed) CG, the station body's state moves with
    // the compound CG (velocity kept: the transfer is internal, venting isotropic), the hulls stay in place.
    void set_tank_mass(size_t body_index, size_t stage_index, std::string_view tank_id, double mass_kg);
```

private: `void refresh_compound();` and member `math::Vec3 compound_cg_point_m_;  // station vehicle frame, at the last
merge / refresh` plus `math::Vec3 docked_chaser_cg_point_m_;  // chaser vehicle frame, at the last merge / refresh`.

`flight_session.cpp`:

```cpp
void FlightSession::set_tank_mass(size_t body_index, size_t stage_index, std::string_view tank_id, double mass_kg) {
    bodies_.at(body_index).body->vehicle().set_tank_mass(stage_index, tank_id, mass_kg);
    const bool in_compound = docked() && station_index_ && (body_index == controlled_ || body_index == *station_index_);
    if (in_compound) refresh_compound();
}
```

`flight_session_docking.cpp` - at the end of the compound part of `merge_docked` (after `docked_mass_kg_ = mc;`):

```cpp
    compound_cg_point_m_ = cg_new;
    docked_chaser_cg_point_m_ = chaser.vehicle().mass_properties().cg_point_m();
```

and:

```cpp
void FlightSession::refresh_compound() {
    auto& chaser = *bodies_[controlled_].body;
    auto& station = *bodies_[*station_index_].body;
    auto attached = *station.vehicle().attached_mass();
    // the chaser's CG moves in its own hull: carry that shift into the station frame (the pair's relative attitude)
    const math::Vec3 chaser_cg = chaser.vehicle().mass_properties().cg_point_m();
    attached.position_m += docked_attitude_station_.rotate(chaser_cg - docked_chaser_cg_point_m_);
    attached.mass_kg = chaser.vehicle().mass_properties().mass_kg;
    station.vehicle().set_attached_mass(attached);
    // the station body's state is the compound CG: move it with the new CG point, hulls fixed, velocity kept
    const math::Vec3 cg_new = station.vehicle().mass_properties().cg_point_m();
    auto ss = station.state();
    ss.position_gcrf_m += ss.attitude.rotate(cg_new - compound_cg_point_m_);
    station.set_state(ss);
    docked_position_station_m_ = attached.position_m - cg_new;
    docked_mass_kg_ = attached.mass_kg;
    compound_cg_point_m_ = cg_new;
    docked_chaser_cg_point_m_ = chaser_cg;
    sync_docked();
}
```

Check the frames before relying on them: `merge_docked` sets `docked_position_station_m_ = transpose(rs) * (cg_mated -
ss.position_gcrf_m)` (station body axes, from the compound CG) and the attached position `cg_old + transpose(rs) *
(cg_mated - ss.position_gcrf_m)` (station vehicle frame) - so `docked_position_station_m_ = attached.position_m -
cg_new` holds. A station tank change alone (r4's depot side) also shifts `cg_new`: covered by the same lines.

- [ ] **Step 4: Run the tests**

`ctest --test-dir build -L "refuelling|docking|iss" -j 24 --output-on-failure > tmp/r3_t6.txt 2>&1` - green.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/src/sim/flight_session_docking.cpp core/tests/test_refuelling_docking.cpp
git commit -m "feat(sim): docked compound follows run-time tank masses"
git push origin main
```

---

### Task 7: Starship phasing on Raptor and cold gas, depot launch window

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp:638` (rename comment of the Cd), `:1022` (`rendezvous_engines`
  with a delta-v), new private helpers
- Modify: `core/src/sim/flight_session_rendezvous.cpp` (refusal removed, budget, drag, engines per burn, coast attitude)
- Create: `core/tests/test_refuelling_rendezvous.cpp`; Modify: `core/tests/CMakeLists.txt`
- Test: launch window in the same file

**Interfaces:**
- Consumes: `Vehicle::rcs_supply_kg` (Task 1), the depot station (Task 2), `refuelling_test::DepotSession`.
- Produces (private, `FlightSession`): `bool rendezvous_main_engines() const;`,
  `mission::FiniteBurnModel rendezvous_engines(double delta_v_mps = std::numeric_limits<double>::infinity()) const;`,
  `double rendezvous_rcs_below_mps() const;`, `double rendezvous_budget_kg() const;`,
  `bool execute_rendezvous_burn(const gnc::BurnNode& node);`.

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_refuelling_rendezvous.cpp`:

```cpp
// Refuelling r3: the tanker's phasing to Waypoint 0 below the depot, and the depot launch window from Starbase.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "refuelling_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/mission/iss_launch_window.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "starsim/orbit/kepler.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

// The tanker in a 170 x 190 km orbit in the depot plane, `behind_deg` of argument of latitude behind it (perigee at
// the tanker's position).
math::StateVector tanker_behind(const refuelling_test::DepotSession& f, double behind_deg) {
    const auto depot_now = refuelling_test::depot_state(f.earth, f.t0, -behind_deg * deg);  // same plane, behind
    const double rp = constants::wgs84_semi_major_axis_m + 170.0e3, ra = constants::wgs84_semi_major_axis_m + 190.0e3;
    const double a = 0.5 * (rp + ra);
    const Vec3 r_hat = math::normalized(depot_now.position_m);
    const Vec3 v_hat = math::normalized(depot_now.velocity_mps - dot(depot_now.velocity_mps, r_hat) * r_hat);
    const double vp = std::sqrt(constants::earth_gm_m3ps2 * (2.0 / rp - 1.0 / a));
    return {rp * r_hat, vp * v_hat};
}

}  // namespace

TEST_CASE("A tanker plans the coelliptic phasing to the depot on Raptor and cold gas", "[refuelling][rendezvous]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::tanker_config(200.0e3), tanker_behind(f, 8.0), f.t0);
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    REQUIRE(f.session.plan_rendezvous({}));
    INFO(f.session.rendezvous_status());
    const auto* plan = f.session.rendezvous_plan();
    REQUIRE(plan);
    CHECK(plan->feasible);
    CHECK(plan->total_delta_v_mps > 10.0);
    CHECK(plan->total_delta_v_mps < 150.0);
    CHECK(plan->parameters.waypoint0_below_m == 400.0);  // the depot's target block
    CHECK(plan->parameters.chaser_drag_area_per_mass_m2pkg > 0.0);
    CHECK(plan->parameters.target_drag_area_per_mass_m2pkg > 0.0);  // the depot has no fixed drag area
}

TEST_CASE("A tanker without main propellant is refused the rendezvous, no throw", "[refuelling][rendezvous]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::tanker_config(0.0), tanker_behind(f, 8.0), f.t0);
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    bool planned = true;
    REQUIRE_NOTHROW(planned = f.session.plan_rendezvous({}));
    CHECK_FALSE(planned);
    CHECK_FALSE(f.session.rendezvous_status().empty());
}

TEST_CASE("The tanker flies the phasing to Waypoint 0 below the depot", "[refuelling][rendezvous][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::tanker_config(200.0e3), tanker_behind(f, 8.0), f.t0);
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    REQUIRE(f.session.plan_rendezvous({}));
    const double main_before = f.session.body(0).vehicle().rcs_supply_kg();
    for (double t = 0.0; t < 30.0 * 3600.0 && f.session.rendezvous_step() != sim::RendezvousStep::Waypoint0 &&
                         f.session.rendezvous_step() != sim::RendezvousStep::None;
         t += 60.0) {
        f.session.advance_to(f.session.epoch() + 60.0);
    }
    INFO(f.session.rendezvous_status());
    REQUIRE(f.session.rendezvous_step() == sim::RendezvousStep::Waypoint0);
    const auto& depot = f.session.body(1).state();
    const auto rel = orbit::lvlh_relative_state({depot.position_gcrf_m, depot.velocity_gcrf_mps},
                                                {f.session.body(0).state().position_gcrf_m,
                                                 f.session.body(0).state().velocity_gcrf_mps});
    CHECK(norm(rel.position_m - Vec3{0.0, 0.0, 400.0}) < 150.0);
    CHECK(norm(rel.velocity_mps) < 0.5);
    bool raptor = false, cold_gas = false;
    for (const auto& b : f.session.rendezvous_burns()) {
        (norm(b.node.delta_v_gcrf_mps) >= 7.0 ? raptor : cold_gas) = true;
    }
    CHECK(raptor);
    CHECK(cold_gas);
    const double used = main_before - f.session.body(0).vehicle().rcs_supply_kg();  // burns, RCS and boil-off
    CHECK(used > 0.0);
    std::printf("[r3] tanker phasing: %zu burns, main propellant used %.1f t\n", f.session.rendezvous_burns().size(),
                used / 1000.0);
}

TEST_CASE("Depot launch window from Starbase: the 26.5 deg plane passes the pad about once a day",
          "[refuelling][launch_window]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto site = config::load_launch_site(refuelling_test::config_dir, "starbase_olp1");
    const Vec3 pad = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    mission::LaunchWindowParameters p;
    p.inclination_rad = 26.5 * deg;
    p.insertion_s = 520.0;                 // [test] Starship to SECO
    p.insertion_arc_rad = 20.0 * deg;
    p.separation_periapsis_radius_m = constants::wgs84_semi_major_axis_m + 170.0e3;
    p.separation_apoapsis_radius_m = constants::wgs84_semi_major_axis_m + 190.0e3;
    const mission::StationStateAt depot = [&](time::Epoch t) {
        const double n = std::sqrt(constants::earth_gm_m3ps2 / std::pow(constants::wgs84_semi_major_axis_m + 200.0e3, 3));
        return refuelling_test::depot_state(f.earth, t, n * (t - f.t0));
    };
    const auto w1 = mission::plan_launch_window(pad, f.earth, depot, f.t0, p, constants::earth_gm_m3ps2);
    INFO(w1.reason);
    REQUIRE(w1.feasible);
    const auto w2 = mission::plan_launch_window(pad, f.earth, depot, w1.liftoff + 4.0 * 3600.0, p,
                                                constants::earth_gm_m3ps2);
    REQUIRE(w2.feasible);
    const double spacing_h = (w2.liftoff - w1.liftoff) / 3600.0;
    CHECK(spacing_h > 20.0);
    CHECK(spacing_h < 26.0);
    std::printf("[r3] depot windows: %.2f h apart\n", spacing_h);
}
```

(The depot test state ignores J2 regression of the RAAN; good enough for the window spacing. If
`plan_launch_window` fails because the pad latitude (26.0 deg) sits just under the inclination (the plane only grazes
the pad), the fix belongs in `iss_launch_window.cpp` (crossing search near tangency) - report it before changing.)

Add `test_refuelling_rendezvous.cpp` to `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Run to verify they fail**

`build/core/tests/starsim_core_tests.exe "A tanker plans the coelliptic phasing to the depot on Raptor and cold gas"` -
FAIL: "Kein Rendezvous: nur mit Manövern auf den Steuerdüsen".

- [ ] **Step 3: Implement**

`flight_session.hpp`: the constant's comment `// [E] blunt body / ship end-on in free molecular flow (Dragon, Starship)`
(name unchanged). Replace `mission::FiniteBurnModel rendezvous_engines() const;` by the helpers in the Interfaces block.

`flight_session_rendezvous.cpp`:

```cpp
bool FlightSession::rendezvous_main_engines() const { return !burn_configurations().empty(); }

// A main-engine chaser (refuelling r3: Starship) burns on its deorbit set - the single belly Raptor - with transients
// up to small_burn_transient_share of the burn; below that minimum on the cold-gas RCS along +x (nose along the
// delta-v), as start_burn chooses.
double FlightSession::rendezvous_rcs_below_mps() const {
    const auto& groups = bodies_[controlled_].body->vehicle().stages().front().config.deorbit_engine_groups;
    const auto set = groups.empty() ? burn_configurations() : burn_configurations(&groups);
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto& c : set) minimum = std::min(minimum, c.minimum_dv_mps);
    return minimum / small_burn_transient_share;
}

mission::FiniteBurnModel FlightSession::rendezvous_engines(double delta_v_mps) const {
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    const double mass = vehicle.mass_properties().mass_kg;
    if (rendezvous_main_engines() && delta_v_mps >= rendezvous_rcs_below_mps()) {
        const auto& groups = vehicle.stages().front().config.deorbit_engine_groups;
        const auto set = groups.empty() ? burn_configurations() : burn_configurations(&groups);
        return {mass, set.front().thrust_n, set.front().mass_flow_kgps};
    }
    // RCS translation as start_burn picks it: along +x, else along -x
    auto capability = vehicle.rcs_force_capability({1.0, 0.0, 0.0});
    if (!(capability.force_n > 0.0)) capability = vehicle.rcs_force_capability({-1.0, 0.0, 0.0});
    return {mass, capability.force_n, capability.mass_flow_kgps};
}

double FlightSession::rendezvous_budget_kg() const {
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    if (!rendezvous_main_engines()) return vehicle.rcs_propellant_kg();  // Crew Dragon: the Draco tanks
    return vehicle.rcs_supply_kg();  // the main tanks (engines and cold gas); the headers are the landing reserve
}

bool FlightSession::execute_rendezvous_burn(const gnc::BurnNode& node) {
    if (!rendezvous_main_engines()) return execute_burn(node);
    const auto& groups = bodies_[controlled_].body->vehicle().stages().front().config.deorbit_engine_groups;
    return start_burn(node, groups.empty() ? burn_configurations() : burn_configurations(&groups), true,
                      small_burn_transient_share);
}
```

(Note: `rcs_supply_kg` counts the main tanks only if the RCS draws from them; Starship's does.)

In `plan_rendezvous`:
- delete the two lines `// only Crew Dragon docks (spec)...` and `if (!burn_configurations().empty()) return refuse(...)`.
- after the refusals: `p = with_station_limits(p, station_limits());` (Task 2 may have put it already).
- target drag: when the station has no fixed area use the end-on estimate:

```cpp
    if (!(p.target_drag_area_per_mass_m2pkg > 0.0)) {
        const auto& station = bodies_[*station_index_].body->vehicle();
        const auto& aero = station.stages().front().config.aero;
        const double radius = 0.5 * station.diameter_m();
        const double cd_area = aero.fixed_drag_area_m2 > 0.0 ? aero.fixed_drag_area_m2 * aero.fixed_drag_coefficient
                                                             : dragon_free_molecular_cd * constants::pi * radius * radius;
        p.target_drag_area_per_mass_m2pkg = cd_area / station.mass_properties().mass_kg;  // the depot flies nose first
    }
```

- budget: `plan = ...plan(controlled_state(), station_state(), epoch_, p, engines, rendezvous_budget_kg());` and the
  same in `rendezvous_replan`.
- refuse an empty budget before planning: `if (!(rendezvous_budget_kg() > 0.0)) return refuse_closed("Kein Rendezvous:
  kein Treibstoff");`

`rendezvous_preparation_epoch`: `const auto engines = rendezvous_engines(norm(node.delta_v_gcrf_mps));`

`rendezvous_fly_due_burn`: retarget with the model the burn will be flown on:

```cpp
    const mission::PhasingPlanner planner(gravity_);
    auto node = planner.retarget(kind, controlled_state(), station_state(), epoch_, *rendezvous_plan_, rendezvous_engines());
    if (node && rendezvous_main_engines() && norm(node->delta_v_gcrf_mps) < rendezvous_rcs_below_mps()) {
        // below the Raptor's minimum impulse: the long cold-gas burn, targeted as such
        const auto rcs = rendezvous_engines(0.0);
        if (auto fine = planner.retarget(kind, controlled_state(), station_state(), epoch_, *rendezvous_plan_, rcs)) node = fine;
    }
```

and `if (!execute_rendezvous_burn(*node)) {`; the debug record uses `rendezvous_engines(norm(node->delta_v_gcrf_mps))`.

`rendezvous_enter_coast`: hold the docking orientation between burns for a main-engine chaser:

```cpp
    // Crew Dragon drifts between the burns; a Starship holds nose along +V, belly to nadir (the docking attitude's
    // orientation, rolled 180 deg from LVLH) - its drag and boil-off stay defined (refuelling r3)
    attitude_mode_ = rendezvous_main_engines() ? gnc::lvlh_hold(math::Quat::from_axis_angle({1.0, 0.0, 0.0}, constants::pi))
                                               : gnc::AttitudeMode{};
```

Update the "Kein Rendezvous" warning text "Bahnanhebung der ISS" -> "Bahnanhebung der Station" (only when the station
is not the ISS? keep it simple: "der Station"; check `grep -rn "Bahnanhebung der ISS" core/tests` first - if a test
matches the text, leave it).

- [ ] **Step 4: Run the tests**

`ctest --test-dir build -L "refuelling|rendezvous|phasing|iss|launch_window" -j 24 --output-on-failure >
tmp/r3_t7.txt 2>&1` - green, including the ISS `[slow]` mission campaigns (Dragon path unchanged: no burn engines).
Record the phasing numbers (burns, propellant, Waypoint 0 miss, runtime) and the window spacing in the ledger.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session_rendezvous.cpp core/tests/test_refuelling_rendezvous.cpp core/tests/CMakeLists.txt
git commit -m "feat(sim): Starship phasing on the belly Raptor and cold gas, depot launch window"
git push origin main
```

---

### Task 8: Tanker from orbit to docked at the depot, docs, final review

**Files:**
- Test: `core/tests/test_refuelling_rendezvous.cpp` (campaign)
- Modify: `docs/subsystems.md` ("Orbital refuelling": r3 section), `CLAUDE.md` (module summary), `config/README.md`
  (`[stage.station.proximity]`, `[[stage.rcs_pod]]`, depot collision), spec status line
  (`docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md` lines 3-5 + corrections under "r3 design")

**Interfaces:**
- Consumes: everything above; the mission sequence `IssMissionSequence` / `set_iss_mission` (generic legs:
  Rendezvous -> Proximity -> Docking -> Docked).

- [ ] **Step 1: Write the campaign test**

```cpp
TEST_CASE("Tanker from its phasing orbit to hard capture at the depot", "[refuelling][rendezvous][docking][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::tanker_config(200.0e3), tanker_behind(f, 8.0), f.t0);
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    REQUIRE(f.session.plan_rendezvous({}));
    sim::IssMissionSequence sequence;
    sequence.station_port = "transfer_dock";
    f.session.set_iss_mission(sequence);
    double t = 0.0;
    for (; t < 36.0 * 3600.0 && f.session.iss_mission_leg() != sim::IssMissionLeg::Docked &&
           f.session.iss_mission_leg() != sim::IssMissionLeg::Ended;
         t += 30.0) {
        f.session.advance_to(f.session.epoch() + 30.0);
    }
    INFO(f.session.iss_mission_status() << " | " << f.session.proximity_status() << " | " << f.session.docking_status());
    REQUIRE(f.session.iss_mission_leg() == sim::IssMissionLeg::Docked);
    REQUIRE(f.session.docked());
    std::printf("[r3] tanker campaign: docked after %.1f h, peak compression %.1f kN\n", t / 3600.0,
                f.session.docking_loads()->compression_n / 1000.0);
}
```

(Check how `set_iss_mission` expects to be started after an explicit `plan_rendezvous` - the i6 sequence tests
(`test_iss_mission_sequence.cpp`) show whether the leg starts at Rendezvous or Ascent; set the leg accordingly or let
`plan_rendezvous` run through the sequence. Adapt, do not add a seam.)

- [ ] **Step 2: Run it**

`ctest --test-dir build -L "refuelling" -j 24 --output-on-failure > tmp/r3_t8.txt 2>&1`. If the campaign fails, debug
with `STARSIM_DEBUG_RENDEZVOUS=1`, `STARSIM_DEBUG_PROXIMITY=1`, `STARSIM_DEBUG_DOCKING=1` (systematic-debugging; no
tolerance widening without a stated cause).

- [ ] **Step 3: Docs**

- `docs/subsystems.md`, "Orbital refuelling": an "r3 Starship rendezvous and docking" subsection - decisions 1-7 of
  this plan with their numbers (-x thrusters and the Coriolis argument; target block; R-bar from below; probe/drogue
  geometry, joint, limits, envelope; phasing on `sea_level_3` + cold gas and its measured burns / propellant; hull and
  leg sampling with the HLS leg clearance; compound refresh), the campaign numbers (Waypoint 0 miss, contact
  conditions, hard-capture time, peak loads, window spacing), the `STARSIM_DEBUG_*` switches, and the deferred items.
- `CLAUDE.md`: module summary - contact "probe/drogue interface (four probes into four drogues)", gnc "R-bar approach
  from below", sim "depot as a station target, Starship phasing on Raptor + cold gas, compound follows run-time tank
  masses".
- `config/README.md`: `[stage.station.proximity]` keys; `[[stage.rcs_pod]]` on reused stages; collision capsules on
  the depot.
- Spec: status line "r1-r3 implemented"; under "r3 design" a "Corrections approved with the r3 plan" paragraph (the
  -x thrusters, sea-level instead of vacuum Raptor, one-stage stations, the approach-from-axis rule).

- [ ] **Step 4: Affected suites, bindings, smoke**

`ctest --test-dir build -L "refuelling|iss|docking|proximity|rendezvous|phasing|launch_window|iss_mission|station|rcs|hls|lunar|contact|config" -j 24 --output-on-failure > tmp/r3_final.txt 2>&1`
- green. Then `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoTests > tmp/r3_bindings.txt 2>&1` and the
smoke test `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script
res://tests/smoke_test.gd > tmp/r3_smoke.txt 2>&1`.

- [ ] **Step 5: Commit, final review**

```bash
git add core/tests/test_refuelling_rendezvous.cpp docs/subsystems.md CLAUDE.md config/README.md docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md
git commit -m "docs: refuelling r3 Starship rendezvous and probe/drogue docking"
git push origin main
```

Then the whole-milestone review (most capable model) over `d41d56b..HEAD`, fix pass, ledger, memory, and ask the user
for `/clear` naming r4 (propellant transfer) as the next plan.
