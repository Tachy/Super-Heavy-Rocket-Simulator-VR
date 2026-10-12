# SCvx Landing Guidance in Flight (Plan 3b) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The returning booster flies its landing burn on the SCvx planner (re-planned every second on a worker thread, tracked by the attitude controller with feed-forward), is caught on the tower arms in >= 90 % of a fixed dispersion campaign without ever striking the tower, and aborts to the sea when the catch can no longer succeed.

**Architecture:** Four new `gnc` units sit on top of Plan 3a's planner: `descent_snapshot` (everything the planner needs, taken on the simulation thread: plan frame, model, start state, catch/abort targets), `descent_replanner` (one worker thread, fixed sim-time latency, wait or real-time mode, solver reuse via `DescentPlanner`), `descent_tracking` (plan sampling, position/velocity correction) and `scvx_landing` (the landing-burn driver: first plan, re-plans, abort decision, actuator commands). `BoosterReturnGuidance` hands the landing burn to `ScvxLandingGuidance` and keeps the explicit laws as fallback; `FlightSession` passes feed-forward to the attitude controller, tells the tower about an abort and can place a returning booster for scenarios. The site gets a configured shoreline; `sim/landing_outcome` classifies "Wasserung" / "gelandet" / "Aufprall". First task: Raptor 3 at 280 tf with re-validation against the FAA residual-propellant figure.

**Tech Stack:** C++20 (MSVC 2022, CMake, Ninja), `std::async`/`std::future`, Catch2 v3.8.1, own `optim` SOCP solver, Godot 4 GDScript (HUD only).

**Spec:** `docs/superpowers/specs/2026-10-12-scvx-landing-design.md` (this plan covers "Execution in flight", "Tracking", "Abort to the sea", "Fallback", the scenario/dispersion/integration tests and the "Follow-ups ... for Plan 3b"; Plan 3a covered the planner itself).

## Global Constraints

- Module order: `gnc/descent_*` and `gnc/scvx_landing` may use `math`, `time`, `frames`, `integrators`, `optim`, `config`, `vehicle`, `environment`, `aero`; `sim` may use `gnc`; never the reverse. `gnc` must not include `sim` or `contact` headers.
- **The worker thread never calls SPICE, `ephemeris`, `env.gravity`, `env.earth_orientation` or `env.atmosphere`:** the snapshot (taken on the simulation thread) carries Earth rotation, gravity at the site, the atmosphere tables and the vehicle data (`DescentModel` owns all its data).
- Re-plan every **1.0 s** of simulation time from a state snapshot; a plan takes effect at **snapshot + 0.5 s** sim time. Tests and time warp **wait** for it (deterministic: identical runs give identical results); real-time play (flag) **never waits** and keeps flying the previous plan.
- Catch terminal condition: pins **1 m above the arm tops**, sink rate **1 m/s**, lateral velocity 0, upright with any roll (roll by the tracking controller), angular rate 0.
- Robustness target: **20 dispersion cases** (lateral position +-50 m, velocity +-10 m/s, residual mass +-5 %, attitude +-3 deg at ignition) **plus one engine failure** in the 13-engine phase; **>= 90 % caught** with **< 1 m** offset and **< 0.5 m/s** lateral speed; **no tower impact in any case**; every abort ends in a splashdown or a soft touchdown without tower damage.
- If the first SCvx plan fails, the existing explicit landing-burn guidance takes over.
- Tower fully automatic (no manual arm control). On an abort the arms stay open; HUD tower line "Abbruch"; outcomes "Wasserung" / "gelandet" / "Aufprall" (German UI strings, English code).
- Vehicle data: Raptor 3 thrust **280 tf** (2.7459e6 N sea level); FAA Revised Draft EA (Nov 2024, https://www.faa.gov/media/87636, p. 7): "Up to approximately 74 MT of residual propellant ... on the Super Heavy vehicle for downrange landing, including expenditures" = [S] cap on Block 3 propellant at landing-burn ignition (an EA projection, not a rule). Do not use the FAA "Upgraded Super Heavy" figures (80 m, 35 engines).
- Config values carry provenance tags ([S] sourced, [E] estimate, [D] derived); run `python tools/vehicle_report.py <id>` after every vehicle/engine config change.
- SI units, radians, frame + unit in names; plan frame `plan`: origin at the pad centre on the ground, x up, y east, z north. No magic numbers: every constant has a comment ([E] for estimates/tuning values).
- Commits: Conventional Commits, ending with the session attribution lines:
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` and `Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE`.
- Build/test: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests` then `build/core/tests/starsim_core_tests.exe "<tag>"`; at the end of each task the full `powershell -ExecutionPolicy Bypass -File tools/build.ps1` (core + GDExtension + ctest). New test files are added to `core/tests/CMakeLists.txt`, new sources to `core/CMakeLists.txt`.

## Review Focus

- **Engine start-up at ignition** (Raptor `startup_s` 3 s, spool 0 -> 1, longer than a 1.6 s phase 0): the first plan must not assume full thrust at t = 0; expected: plan thrust bounded by the spool ramp, booster still caught. Pinned by Task 4's spool test and Task 9's scenario (first plan made at spool 0).
- **13 -> 3 switch transient** (ten middle engines run down over `shutdown_s` 0.5 s, extra thrust the plan does not know): expected: the next re-plans absorb it; the catch still meets < 1 m / < 0.5 m/s. Pinned by Task 9's scenario check of the offset/lateral speed at first pin contact.
- **Constant-CG planning model + no re-plans in the last 3 s**: the pins may arrive higher/lower or faster than planned; expected: sink rate at first pin contact < 2 m/s and no pin overload. Pinned by Task 9's scenario (`sink at contact` check) and the campaign (pin overload = impact = failure).
- **Real-time mode with a slow worker**: expected: `collect` never blocks, the previous plan stays active, the late plan is used once it arrives. Pinned by Task 6's slow-planner test.
- **Session or guidance destroyed while a re-plan is running** (game quits mid-landing): expected: no crash, no hang beyond the running plan. Pinned by Task 6's destroy-while-busy test.

---

## File Structure

| File | Responsibility |
|------|----------------|
| `config/engines/raptor3.toml` | 280 tf, re-derived nozzle exit |
| `config/vehicles/starship_block3.toml` | FAA source, reserve tuned to the 74 t cap if needed |
| `config/launch_sites.toml` | `[site.sea]` shoreline per site |
| `core/include/starsim/config/launch_site_config.hpp`, `core/src/config/launch_site_config.cpp` | `SeaConfig` |
| `core/include/starsim/sim/landing_outcome.hpp`, `core/src/sim/landing_outcome.cpp` | water/land classification, `LandingOutcome` |
| `core/include/starsim/sim/catch_tower_sim.hpp`, `core/src/sim/catch_tower_sim.cpp` | `CatchPhase::Aborted`, `abort_catch()`, `tower_struck()` |
| `core/include/starsim/gnc/descent_planner.hpp`, `core/src/gnc/descent_planner.cpp` | `DescentPlanner` (solver per start phase + goal), stall stop, `warm_start_reference` |
| `core/include/starsim/gnc/descent_problem.hpp`, `core/src/gnc/descent_problem.cpp` | tilt taper, spool ramp, abort ground constraint, `constants::deg_to_rad` |
| `core/include/starsim/gnc/landing_site.hpp`, `core/src/gnc/landing_site.cpp` | `LandingSite`, `CatchTarget`, `SeaSide`, `catch_roll_reference` (moved out of `booster_return`) |
| `core/include/starsim/gnc/descent_snapshot.hpp`, `core/src/gnc/descent_snapshot.cpp` | `PlanFrame`, `plan_state`, `DescentSnapshot`, `take_descent_snapshot` |
| `core/include/starsim/gnc/descent_replanner.hpp`, `core/src/gnc/descent_replanner.cpp` | `PlanRequest`, `PlanOutcome`, `DescentReplanner` (worker, latency, wait/real-time) |
| `core/include/starsim/gnc/descent_tracking.hpp`, `core/src/gnc/descent_tracking.cpp` | `sample_plan`, `commanded_acceleration` |
| `core/include/starsim/gnc/attitude_controller.hpp`, `core/src/gnc/attitude_controller.cpp` | `Feedforward` in `steer` |
| `core/include/starsim/gnc/scvx_landing.hpp`, `core/src/gnc/scvx_landing.cpp` | `ScvxLandingGuidance` (first plan, re-plans, abort, commands) |
| `core/include/starsim/gnc/booster_return.hpp`, `core/src/gnc/booster_return.cpp` | SCvx hand-over, fallback, `skip_to`, `set_realtime`, command fields |
| `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` | `return_site`, `environment`, `place_returning_booster`, real-time flag, abort -> tower, feed-forward, `landing_outcome` |
| `core/tests/return_scenario.hpp` | test helpers: booster config with given propellant, state over the site, ignition height, catch run |
| `core/tests/test_landing_outcome.cpp`, `test_descent_snapshot.cpp`, `test_descent_replanner.cpp`, `test_descent_tracking.cpp`, `test_scvx_landing.cpp`, `test_scvx_catch.cpp` | new tests |
| `bindings/src/flight_sim.hpp`, `bindings/src/flight_sim.cpp` | `set_booster_return`, `set_realtime_planning`, `return_state` |
| `game/ui/flight_hud.gd`, `game/vehicle/flight_controller.gd`, `game/world/world.gd` | HUD lines, enable return with the autopilot, real-time flag |
| `CLAUDE.md`, spec, `docs/ansichten.md` | documentation |

---

### Task 1: Raptor 3 at 280 tf and the FAA residual-propellant check

**Files:**
- Modify: `config/engines/raptor3.toml`
- Modify: `config/vehicles/starship_block3.toml` (source line; `reserve_propellant_kg` only if Step 5 requires it)
- Modify: `core/tests/test_booster_return.cpp:147-180`
- Modify: `CLAUDE.md` (Known issues: block 3 reserve)

**Interfaces:**
- Consumes: nothing new.
- Produces: `faa_landing_propellant_kg` check in the catch-return test (later tasks keep it).

- [ ] **Step 1: Write the failing check**

In `core/tests/test_booster_return.cpp`, add below `constexpr double deg = constants::deg_to_rad;` inside the anonymous namespace:

```cpp
// FAA Revised Draft EA, Nov 2024 (https://www.faa.gov/media/87636, p. 7): "Up to approximately 74 MT of residual
// propellant ... on the Super Heavy vehicle for downrange landing, including expenditures" [S]. An EA projection,
// used here as the cap on the Block 3 booster's propellant at landing-burn ignition.
constexpr double faa_landing_propellant_kg = 74.0e3;
```

In the test "Booster return to the catch tower ends caught or missed", record the propellant at ignition. Replace the loop body lines

```cpp
        landing = status && status->phase >= gnc::ReturnPhase::LandingBurn;
        if (booster.caught() || booster.impacted() || booster.resting()) break;
```

with

```cpp
        const bool burning = status && status->phase >= gnc::ReturnPhase::LandingBurn;
        if (burning && !landing) ignition_propellant_kg = booster.vehicle().propellant_kg();
        landing = burning;
        if (booster.caught() || booster.impacted() || booster.resting()) break;
```

declare `double ignition_propellant_kg = -1.0;` next to `bool landing = false;`, and append after the final `std::printf`:

```cpp
    std::printf("%s: propellant at landing-burn ignition %.1f t\n", vehicle_id.c_str(), ignition_propellant_kg / 1e3);
    REQUIRE(ignition_propellant_kg >= 0.0);
    if (vehicle_id == "starship_block3") CHECK(ignition_propellant_kg <= faa_landing_propellant_kg);
```

- [ ] **Step 2: Run it before the engine change**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests` then `build/core/tests/starsim_core_tests.exe "Booster return to the catch tower ends caught or missed"`
Expected: the block 3 line prints the ignition propellant; record the three values in the ledger. If block 3 is already <= 74 t the CHECK passes (it is a cap, not a behaviour switch) - note that in the ledger; the engine change in Step 3 must keep it.

- [ ] **Step 3: Switch Raptor 3 to 280 tf**

In `config/engines/raptor3.toml` add to `[sources]`:

```toml
block3_summary = "Project decision 2026-10-12 (Block 3 summary: 33 Raptor 3, ~280 tf per engine at sea level)"
```

and replace the two `[performance]` lines:

```toml
thrust_ref_n = 2.7459e6              # 280 tf [S:block3_summary]; Wikipedia's 250 tf is the earlier figure [S:wiki_raptor]
nozzle_exit_diameter_m = 1.446       # [D] from Isp_vac = 350 s [S:wiki_sh]: A_e = F_sl*(350/330-1)/p0 = 1.6424 m^2
```

In `config/vehicles/starship_block3.toml` add to its `[sources]` table:

```toml
faa_ea_2024 = "https://www.faa.gov/media/87636 (FAA Revised Draft EA, Nov 2024, p. 7): up to ~74 t residual propellant on Super Heavy for landing, incl. expenditures"
```

- [ ] **Step 4: Re-check the vehicles**

Run: `python tools/vehicle_report.py starship_block3` and `python tools/vehicle_report.py starship_block3 --payload-kg 100000`
Expected: booster thrust 33 x 2.75 MN = 90.6 MN sea level (+12 % vs before), liftoff T/W printed; record T/W, burn times and delta-v in the ledger.

- [ ] **Step 5: Run the affected validations**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`, then
`build/core/tests/starsim_core_tests.exe "[booster]"`, `build/core/tests/starsim_core_tests.exe "[ascent]"`, `build/core/tests/starsim_core_tests.exe "[flight]"`, `build/core/tests/starsim_core_tests.exe "[descent]"`
Expected: all pass, the block 3 ignition propellant <= 74 t.
If the block 3 ascent tests fail (higher thrust -> different max-Q / MECO), diagnose with `STARSIM_VERBOSE=1`; tolerances stay as written - fix the cause (pitch program parameters in the test's mission are inputs, not tolerances) and ledger a ruling.
If block 3's ignition propellant exceeds 74 t: lower `reserve_propellant_kg` in `config/vehicles/starship_block3.toml` in 10 t steps (comment: `# [E] tuned so that <= 74 t remain at landing-burn ignition [S:faa_ea_2024]`) until the check passes while the booster still reaches the tower (test line "caught" or "missed", not an impact before the tower zone); ledger every step.

- [ ] **Step 6: Update CLAUDE.md**

In the Known issues bullet "Booster return reserves (block 1/2: 330 t, block 3: 300 t; ...)" put the final block 3 reserve value and append: "Block 3 is checked against the FAA EA's ~74 t residual propellant at landing-burn ignition (Raptor 3 at 280 tf)."

- [ ] **Step 7: Full build and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1`
Expected: all tests pass.

```bash
git add config/engines/raptor3.toml config/vehicles/starship_block3.toml core/tests/test_booster_return.cpp CLAUDE.md
git commit -m "feat: Raptor 3 at 280 tf, Block 3 landing propellant checked against the FAA EA

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE"
```

---

### Task 2: Shoreline, landing outcome and the tower's abort state

**Files:**
- Modify: `config/launch_sites.toml`
- Modify: `core/include/starsim/config/launch_site_config.hpp`, `core/src/config/launch_site_config.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp` (`LaunchSite::sea`, `landing_outcome`), `core/src/sim/flight_session.cpp`
- Create: `core/include/starsim/sim/landing_outcome.hpp`, `core/src/sim/landing_outcome.cpp`
- Modify: `core/include/starsim/sim/catch_tower_sim.hpp`, `core/src/sim/catch_tower_sim.cpp`
- Create: `core/tests/return_scenario.hpp`, `core/tests/test_landing_outcome.cpp`
- Modify: `core/tests/test_launch_site_config.cpp`, `core/tests/test_catch_tower.cpp`, `core/tests/CMakeLists.txt`, `core/CMakeLists.txt`

**Interfaces:**
- Consumes: `TowerGeometry`, `FlightBody::{caught,impacted,resting}`.
- Produces:
  - `struct config::SeaConfig { double direction_rad; double shoreline_distance_m; };`, `LaunchSiteConfig::sea` and `sim::LaunchSite::sea` (`std::optional<config::SeaConfig>`, last member)
  - `enum class sim::LandingOutcome { Flying, Caught, Landed, Splashdown, Impact };`, `const char* to_string(LandingOutcome)`
  - `bool sim::over_water(const config::SeaConfig&, double east_m, double north_m)`
  - `math::Vec3 sim::site_enu_m(const LaunchSite&, const math::Vec3& itrf_m)`
  - `LandingOutcome sim::landing_outcome(const vehicle::FlightBody&, const LaunchSite&, const math::Vec3& position_itrf_m)`
  - `LandingOutcome FlightSession::landing_outcome(size_t index) const`
  - `CatchPhase::Aborted` ("Abbruch"), `void CatchTowerSim::abort_catch()`, `bool CatchTowerSim::tower_struck() const`
  - test helpers in `core/tests/return_scenario.hpp`: `config::VehicleConfig booster_config(const std::string& id, double propellant_kg)`, `vehicle::RigidBodyState state_over_site(const sim::LaunchSite&, const frames::BodyOrientation&, time::Epoch, const math::Vec3& cg_enu_m, const math::Vec3& velocity_enu_mps, const math::Quat& tilt_enu = {})`

- [ ] **Step 1: Write the failing tests**

`core/tests/return_scenario.hpp`:

```cpp
#pragma once

// Helpers for return/landing scenarios: a booster with a chosen propellant load, placed over the launch site.

#include <cmath>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/primitives.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::test {

// Single-stage booster of `vehicle_id` with its tanks scaled to `propellant_kg` in total.
inline config::VehicleConfig booster_config(const std::string& vehicle_id, double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, vehicle_id);
    cfg.stages.resize(1);
    double total = 0.0;
    for (const auto& tank : cfg.stages[0].tanks) total += tank.mass_kg;
    for (auto& tank : cfg.stages[0].tanks) tank.mass_kg *= propellant_kg / total;
    return cfg;
}

// Booster state over `site`: CG at `cg_enu_m` (east, north, up from the pad centre on the ground), velocity relative
// to the ground, body x up and body +y east, then rotated by `tilt_enu` (site frame); co-rotating with the Earth.
inline vehicle::RigidBodyState state_over_site(const sim::LaunchSite& site, const frames::BodyOrientation& earth,
                                               time::Epoch epoch, const math::Vec3& cg_enu_m,
                                               const math::Vec3& velocity_enu_mps, const math::Quat& tilt_enu = {}) {
    const double sl = std::sin(site.latitude_rad), cl = std::cos(site.latitude_rad);
    const double so = std::sin(site.longitude_rad), co = std::cos(site.longitude_rad);
    const math::Mat3 enu_to_itrf = contact::from_columns({-so, co, 0.0}, {-sl * co, -sl * so, cl}, {cl * co, cl * so, sl});
    const math::Vec3 origin = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const math::Mat3 body_to_enu =
        tilt_enu.to_matrix() * contact::from_columns({0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    const auto t = earth.fixed_to_gcrf(epoch);
    const math::Vec3 r_itrf = origin + enu_to_itrf * cg_enu_m;
    vehicle::RigidBodyState s;
    s.position_gcrf_m = t.position(r_itrf);
    s.velocity_gcrf_mps = t.velocity(r_itrf, enu_to_itrf * velocity_enu_mps);
    s.attitude = math::Quat::from_matrix(t.rotation * enu_to_itrf * body_to_enu);
    s.angular_velocity_body_radps = s.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    return s;
}

}  // namespace starsim::test
```

`core/tests/test_landing_outcome.cpp`:

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("Ground beyond the shoreline is water", "[sim][landing]") {
    const config::SeaConfig east{90.0 * constants::deg_to_rad, 500.0};
    CHECK(sim::over_water(east, 501.0, 0.0));
    CHECK_FALSE(sim::over_water(east, 499.0, 300.0));
    CHECK_FALSE(sim::over_water(east, -2000.0, 0.0));
}

TEST_CASE("Site frame positions are east, north, up from the pad", "[sim][landing]") {
    const sim::LaunchSite site{0.4, -1.7, 10.0, {}, {}};
    const double sl = std::sin(site.latitude_rad), cl = std::cos(site.latitude_rad);
    const double so = std::sin(site.longitude_rad), co = std::cos(site.longitude_rad);
    const Vec3 origin = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const Vec3 p = origin + Vec3{-so, co, 0.0} * 3.0 + Vec3{-sl * co, -sl * so, cl} * 4.0 + Vec3{cl * co, cl * so, sl} * 5.0;
    const Vec3 enu = sim::site_enu_m(site, p);
    CHECK(std::abs(enu.x - 3.0) < 1e-6);
    CHECK(std::abs(enu.y - 4.0) < 1e-6);
    CHECK(std::abs(enu.z - 5.0) < 1e-6);
}

TEST_CASE("A booster coming down beyond the shoreline splashes down, on land it impacts", "[sim][landing]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
    const auto site = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1"));
    REQUIRE(site.sea.has_value());
    const auto cfg = test::booster_config("starship_block3", 5000.0);
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const Vec3 seaward{std::sin(site.sea->direction_rad), std::cos(site.sea->direction_rad), 0.0};
    for (const double distance : {site.sea->shoreline_distance_m + 100.0, site.sea->shoreline_distance_m - 100.0}) {
        CAPTURE(distance);
        sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
        session.set_launch_site(site);
        // Engines off, 3 m above the ground: touches down at ~8 m/s (an impact, not a landing).
        session.place_body(cfg, test::state_over_site(site, earth, t0, seaward * distance + Vec3{0.0, 0.0, cg + 3.0},
                                                      {0.0, 0.0, -1.0}),
                           t0);
        session.set_control_mode(sim::ControlMode::DirectGimbal);
        for (double t = 0.1; t < 5.0 && session.landing_outcome(0) == sim::LandingOutcome::Flying; t += 0.1) {
            session.advance_to(t0 + t, 1000);
        }
        const auto outcome = session.landing_outcome(0);
        CHECK(outcome == (distance > site.sea->shoreline_distance_m ? sim::LandingOutcome::Splashdown
                                                                    : sim::LandingOutcome::Impact));
    }
}
```

In `core/tests/test_launch_site_config.cpp`, inside "Launch sites load with their catch tower" (after the existing tower checks for `starbase_olp1`, using the variable that holds the loaded site - read the test first), add:

```cpp
    REQUIRE(starbase.sea.has_value());
    CHECK(starbase.sea->direction_rad == Catch::Approx(90.0 * constants::deg_to_rad));
    CHECK(starbase.sea->shoreline_distance_m > 0.0);
```

(rename `starbase` to that test's variable name; add `#include <catch2/catch_approx.hpp>` if missing).

In `core/tests/test_catch_tower.cpp` append:

```cpp
TEST_CASE("An aborted catch keeps the arms open", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    session.tower()->abort_catch();
    place_booster(session, t0, {0.0, 0.0, 3.0}, {0.0, 0.0, -1.0});  // would be caught (see the centred arrival)
    const auto& booster = session.body(0);
    run(session, t0, 10.0, [&] { return booster.caught() || booster.impacted() || booster.resting(); });
    CHECK_FALSE(booster.caught());
    CHECK(session.tower()->phase() == sim::CatchPhase::Aborted);
    CHECK(session.tower()->arms().opening_rad() > 0.0);
    CHECK_FALSE(session.tower()->tower_struck());
}

TEST_CASE("Striking the tower is recorded", "[contact][catch]") {
    const auto t0 = start();
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    use_starbase(session);
    const auto& tower = session.tower()->geometry();
    place_booster(session, t0, tower.tower_direction() * 12.0 + Vec3{0.0, 0.0, 20.0}, tower.tower_direction() * 5.0);
    const auto& booster = session.body(0);
    run(session, t0, 10.0, [&] { return booster.impacted(); });
    CHECK(session.tower()->tower_struck());
}
```

Add `test_landing_outcome.cpp` to `core/tests/CMakeLists.txt` (after `test_descent_planner.cpp`).

- [ ] **Step 2: Run to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`
Expected: compile errors (`landing_outcome.hpp` missing, `LaunchSite` has no `sea`, `abort_catch` unknown).

- [ ] **Step 3: Config**

`config/launch_sites.toml`, after each `[site.tower]` table:

```toml
[site.sea]                    # coastline for aborted landings: ground beyond this line is water (flat until M8)
direction_deg = 90.0          # [E] towards the sea (Gulf of Mexico), clockwise from north (verify against imagery)
shoreline_distance_m = 500.0  # [E] pad centre -> shoreline along that direction (verify against imagery)
```

for `starbase_olp1`, and for `ksc_lc39a`:

```toml
[site.sea]                    # coastline for aborted landings: ground beyond this line is water (flat until M8)
direction_deg = 90.0          # [E] towards the Atlantic, clockwise from north (verify against imagery)
shoreline_distance_m = 1300.0 # [E] pad centre -> shoreline along that direction (verify against imagery)
```

`launch_site_config.hpp`, before `struct LaunchSiteConfig`:

```cpp
// Coastline of a launch site ([site.sea]): a straight shoreline perpendicular to `direction_rad`; the ground beyond
// it is water (aborted landings end there as a splashdown).
struct SeaConfig {
    double direction_rad = 0.0;          // pad centre -> sea, clockwise from north
    double shoreline_distance_m = 0.0;   // pad centre -> shoreline along that direction
};
```

and in `LaunchSiteConfig` after `tower`: `std::optional<SeaConfig> sea;`

`launch_site_config.cpp`, in the anonymous namespace:

```cpp
SeaConfig load_sea(const Document& doc, const toml::table& t) {
    SeaConfig c;
    c.direction_rad = doc.number(t, "direction_deg") * constants::deg_to_rad;
    c.shoreline_distance_m = doc.positive(t, "shoreline_distance_m");
    return c;
}
```

and after the tower line: `if (auto sea = t["sea"].as_table()) s.sea = load_sea(doc, *sea);`

`flight_session.hpp`: in `struct LaunchSite` add as the last member `std::optional<config::SeaConfig> sea;  // coastline (aborted landings)`; `launch_site_from_config` returns `{site.latitude_rad, site.longitude_rad, site.height_m, site.tower, site.sea}`.

- [ ] **Step 4: Landing outcome**

`core/include/starsim/sim/landing_outcome.hpp`:

```cpp
#pragma once

#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

// How a body's flight ended (German UI text via to_string).
enum class LandingOutcome { Flying, Caught, Landed, Splashdown, Impact };

const char* to_string(LandingOutcome outcome);

// Ground at (east, north) of the pad centre lies beyond the shoreline.
bool over_water(const config::SeaConfig& sea, double east_m, double north_m);

// Site frame (east, north, up from the pad centre on the ground) of an Earth-fixed position.
math::Vec3 site_enu_m(const LaunchSite& site, const math::Vec3& itrf_m);

// Caught on the tower; resting or impacted beyond the shoreline: splashdown (booster lost, tower undamaged);
// otherwise landed (resting) or impact.
LandingOutcome landing_outcome(const vehicle::FlightBody& body, const LaunchSite& site, const math::Vec3& position_itrf_m);

}  // namespace starsim::sim
```

`core/src/sim/landing_outcome.cpp`:

```cpp
#include "starsim/sim/landing_outcome.hpp"

#include <cmath>

#include "starsim/frames/geodetic.hpp"

namespace starsim::sim {

using math::Vec3;

const char* to_string(LandingOutcome outcome) {
    switch (outcome) {
        case LandingOutcome::Flying: return "fliegt";
        case LandingOutcome::Caught: return "gefangen";
        case LandingOutcome::Landed: return "gelandet";
        case LandingOutcome::Splashdown: return "Wasserung";
        case LandingOutcome::Impact: return "Aufprall";
    }
    return "";
}

bool over_water(const config::SeaConfig& sea, double east_m, double north_m) {
    return east_m * std::sin(sea.direction_rad) + north_m * std::cos(sea.direction_rad) >= sea.shoreline_distance_m;
}

Vec3 site_enu_m(const LaunchSite& site, const Vec3& itrf_m) {
    const double sl = std::sin(site.latitude_rad), cl = std::cos(site.latitude_rad);
    const double so = std::sin(site.longitude_rad), co = std::cos(site.longitude_rad);
    const Vec3 d = itrf_m - frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    return {dot(d, Vec3{-so, co, 0.0}), dot(d, Vec3{-sl * co, -sl * so, cl}), dot(d, Vec3{cl * co, cl * so, sl})};
}

LandingOutcome landing_outcome(const vehicle::FlightBody& body, const LaunchSite& site, const Vec3& position_itrf_m) {
    if (body.caught()) return LandingOutcome::Caught;
    if (!body.impacted() && !body.resting()) return LandingOutcome::Flying;
    const Vec3 enu = site_enu_m(site, position_itrf_m);
    if (site.sea && over_water(*site.sea, enu.x, enu.y)) return LandingOutcome::Splashdown;
    return body.resting() ? LandingOutcome::Landed : LandingOutcome::Impact;
}

}  // namespace starsim::sim
```

`flight_session.hpp`: forward-declare in `namespace starsim::sim` before `class FlightSession`: `enum class LandingOutcome;` and add the public method

```cpp
    // How the flight of body `index` ended so far (Flying while it is in the air).
    LandingOutcome landing_outcome(size_t index) const;
```

`flight_session.cpp`: `#include "starsim/sim/landing_outcome.hpp"` and

```cpp
LandingOutcome FlightSession::landing_outcome(size_t index) const {
    const auto t = earth_.fixed_to_gcrf(epoch_);
    const auto& body = *bodies_.at(index).body;
    return sim::landing_outcome(body, launch_site_, transpose(t.rotation) * body.state().position_gcrf_m);
}
```

Add `src/sim/landing_outcome.cpp` to `core/CMakeLists.txt` after `src/sim/catch_tower_sim.cpp`.

- [ ] **Step 5: Tower abort and strike flag**

`catch_tower_sim.hpp`: `enum class CatchPhase { Open, Closing, Caught, Missed, Aborted };` and public

```cpp
    // The landing guidance gave up the catch: the arms stay (or swing back) open for the rest of the flight.
    void abort_catch();
    // A body struck the tower, the mount or an arm (tower damage).
    bool tower_struck() const { return tower_struck_; }
```

private member `bool tower_struck_ = false;`.

`catch_tower_sim.cpp`: `to_string` gets `case CatchPhase::Aborted: return "Abbruch";`;

```cpp
void CatchTowerSim::abort_catch() {
    if (phase_ == CatchPhase::Caught) return;
    arms_.command_close(false);
    phase_ = CatchPhase::Aborted;
}
```

In `after_tick` set `tower_struck_ = true;` together with `body->mark_impacted()` in the `hits_tower(...)` line and in the arm `hull_penetration_m(...) > hull_crush_depth_m` line (braces around the two statements). Both `phase_ = CatchPhase::Missed;` assignments get the extra condition `&& phase_ != CatchPhase::Aborted` (the existing `phase_ != CatchPhase::Caught` stays). The arm trigger in `before_tick` runs only in `CatchPhase::Open`, so an aborted tower never closes.

- [ ] **Step 6: Run the tests**

Run: build, then `build/core/tests/starsim_core_tests.exe "[landing]"`, `build/core/tests/starsim_core_tests.exe "[catch]"`, `build/core/tests/starsim_core_tests.exe "[config]"`
Expected: all pass.

- [ ] **Step 7: Full build and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` - Expected: all pass.

```bash
git add config/launch_sites.toml core/include/starsim/config/launch_site_config.hpp core/src/config/launch_site_config.cpp core/include/starsim/sim/landing_outcome.hpp core/src/sim/landing_outcome.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/include/starsim/sim/catch_tower_sim.hpp core/src/sim/catch_tower_sim.cpp core/tests/return_scenario.hpp core/tests/test_landing_outcome.cpp core/tests/test_launch_site_config.cpp core/tests/test_catch_tower.cpp core/tests/CMakeLists.txt core/CMakeLists.txt
git commit -m "feat: shoreline per launch site, splashdown classification and an abortable catch

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE"
```

---

### Task 3: `DescentPlanner` - solver reuse across re-plans and an early stop on stalled iterations

**Files:**
- Modify: `core/include/starsim/gnc/descent_planner.hpp`, `core/src/gnc/descent_planner.cpp`
- Modify: `core/tests/test_descent_planner.cpp`

**Interfaces:**
- Consumes: `optim::SocpSolver(const SocpProblem&)`, `SocpSolver::solve(const SocpProblem&)` (throws `std::invalid_argument` on another structure).
- Produces:
  - `class DescentPlanner { public: DescentPlan plan(const DescentModel&, const DescentState& initial, int start_phase, const DescentTarget&, const ScvxSettings&, const DescentPlan* warm_start, double warm_start_elapsed_s); size_t solver_count() const; };`
  - `ScvxSettings::stall_iterations` (int, 3), `ScvxSettings::stall_ratio` (double, 0.5)
  - `plan_descent(...)` unchanged signature (wraps a fresh `DescentPlanner`).

- [ ] **Step 1: Write the failing tests** (append to `core/tests/test_descent_planner.cpp`)

```cpp
TEST_CASE("A planner reuses one solver per start phase and goal with unchanged results", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    const auto x0 = ignition_state(*b);
    const auto target = catch_target(*b, state_attitude(x0));
    DescentPlanner planner;
    const auto first = planner.plan(b->model, x0, 0, target, s, nullptr, 0.0);
    REQUIRE(first.converged);
    const double t = first.reference.duration_s[0] + 1.0;
    const auto again = planner.plan(b->model, first.state_at(t), 1, target, s, &first, t);
    const auto twice = planner.plan(b->model, first.state_at(t), 1, target, s, &first, t);
    CHECK(planner.solver_count() == 2);  // (phase 0, catch) and (phase 1, catch)
    const auto fresh = plan_descent(b->model, first.state_at(t), 1, target, s, &first, t);
    CHECK(twice.reference.x == fresh.reference.x);
    CHECK(again.reference.x == fresh.reference.x);
}

TEST_CASE("SCvx stops early when the virtual control stalls", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    // 200 m up at 250 m/s: no thrust can stop this before the catch point.
    const auto x0 = make_state(b->vehicle->mass_properties().mass_kg, {200.0, 0.0, 0.0}, {-250.0, 0.0, 0.0}, Quat{}, {});
    const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, Quat{}), s, nullptr, 0.0);
    CHECK_FALSE(plan.converged);
    CHECK(plan.iterations <= s.stall_iterations + 2);
}
```

- [ ] **Step 2: Run to verify they fail**

Run: build. Expected: compile errors (`DescentPlanner`, `stall_iterations` unknown).

- [ ] **Step 3: Implement**

`descent_planner.hpp`: add `#include <map>`, `#include <memory>`, `#include <utility>`, `#include "starsim/optim/socp_solver.hpp"`; in `ScvxSettings` after `defect_tolerance`:

```cpp
    // Not converging: stop once the virtual control has not shrunk below stall_ratio of its value stall_iterations
    // iterations earlier while still above 10x its tolerance (an unreachable target; keeps failing re-plans short).
    int stall_iterations = 3;
    double stall_ratio = 0.5;
```

Below the `plan_descent` declaration:

```cpp
// SCvx with one reusable SOCP solver per (start phase, goal): the subproblem structure depends only on these (and
// the node count), so re-plans reuse ordering, symbolic factorisation and work arrays. Not thread-safe: one planner
// per thread. A non-converged plan may still carry status Optimal (the last subproblem solved): check `converged`.
class DescentPlanner {
public:
    DescentPlan plan(const DescentModel& model, const DescentState& initial, int start_phase, const DescentTarget& target,
                     const ScvxSettings& settings, const DescentPlan* warm_start, double warm_start_elapsed_s);
    size_t solver_count() const { return solvers_.size(); }

private:
    optim::SocpResult solve(int start_phase, DescentGoal goal, const optim::SocpProblem& problem);
    std::map<std::pair<int, DescentGoal>, std::unique_ptr<optim::SocpSolver>> solvers_;
};
```

`descent_planner.cpp`: rename the body of `plan_descent` into `DescentPlan DescentPlanner::plan(...)` (same parameters), and make `plan_descent` a wrapper:

```cpp
DescentPlan plan_descent(const DescentModel& model, const DescentState& initial, int start_phase, const DescentTarget& target,
                         const ScvxSettings& settings, const DescentPlan* warm_start, double elapsed) {
    DescentPlanner planner;
    return planner.plan(model, initial, start_phase, target, settings, warm_start, elapsed);
}

optim::SocpResult DescentPlanner::solve(int start_phase, DescentGoal goal, const optim::SocpProblem& problem) {
    auto& solver = solvers_[{start_phase, goal}];
    if (solver) {
        try {
            return solver->solve(problem);
        } catch (const std::invalid_argument&) {
            // Another structure (e.g. another node count): rebuild below. Invalid data throws again there.
        }
    }
    solver = std::make_unique<optim::SocpSolver>(problem);
    return solver->solve(problem);
}
```

In `DescentPlanner::plan`, delete `std::optional<optim::SocpSolver> solver;` and the two lines `if (!solver) solver.emplace(sub.problem); const auto result = solver->solve(sub.problem);`, replacing them by `const auto result = solve(start_phase, target.goal, sub.problem);`. Add `std::vector<double> virtual_controls;` before the loop and, right after `plan.virtual_control = sub.virtual_control(result.x);`:

```cpp
        virtual_controls.push_back(plan.virtual_control);
        if (it >= settings.stall_iterations &&
            plan.virtual_control > 10.0 * settings.virtual_control_tolerance &&
            plan.virtual_control > settings.stall_ratio * virtual_controls[static_cast<size_t>(it - settings.stall_iterations)]) {
            plan.last_step = sub.max_step(result.x);
            plan.reference = sub.unpack(result.x);
            return plan;  // stalled: not converged
        }
```

Add `#include <stdexcept>` to the .cpp.

- [ ] **Step 4: Run the tests**

Run: build, `build/core/tests/starsim_core_tests.exe "[descent]"`
Expected: all pass (including Plan 3a's tests). If the stall test runs all iterations, print `virtual_controls` (temporary `std::printf`, removed before commit) and check whether the stall criterion's index arithmetic matches; the tolerances in the test stay.

- [ ] **Step 5: Full build and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` - Expected: all pass.

```bash
git add core/include/starsim/gnc/descent_planner.hpp core/src/gnc/descent_planner.cpp core/tests/test_descent_planner.cpp
git commit -m "feat: reusable DescentPlanner with one SOCP solver per start phase and goal, stall stop

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE"
```

---

### Task 4: Planner extensions for flight - spool-up, tilt taper, abort ground constraint, warm start from the snapshot

**Files:**
- Modify: `core/include/starsim/gnc/descent_problem.hpp`, `core/src/gnc/descent_problem.cpp`
- Modify: `core/include/starsim/gnc/descent_planner.hpp`, `core/src/gnc/descent_planner.cpp`
- Modify: `core/tests/test_descent_planner.cpp`

**Interfaces:**
- Consumes: Task 3's `DescentPlanner`.
- Produces:
  - `DescentLimits::terminal_tilt_rad` (4 deg), `DescentLimits::tilt_taper_nodes` (4), `DescentLimits::phase0_initial_spool` (1.0), `DescentLimits::phase0_startup_s` (0.0)
  - `DescentReference warm_start_reference(const DescentPlan& previous, double elapsed_s, int start_phase, int nodes_per_phase, const DescentState& initial);` (public; node(s) at time 0 = `initial`)
  - `double reference_node_time(const std::array<double, 2>& duration_s, int k, int nodes_per_phase);` (public, in `descent_problem.hpp`)

- [ ] **Step 1: Write the failing tests** (append to `core/tests/test_descent_planner.cpp`)

```cpp
namespace {

double tilt_of(const DescentState& x) {
    return 2.0 * std::asin(std::min(1.0, std::hypot(x[descent_index::attitude + 2], x[descent_index::attitude + 3])));
}

}  // namespace

TEST_CASE("The tilt limit tapers towards the catch", "[gnc][descent][scvx]") {
    const auto b = block3();
    auto s = settings(*b);
    const auto x0 = ignition_state(*b);
    const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, state_attitude(x0)), s, nullptr, 0.0);
    REQUIRE(plan.converged);
    const int last = static_cast<int>(plan.reference.x.size()) - 1;
    for (int from_end = 0; from_end < s.limits.tilt_taper_nodes; ++from_end) {
        const double limit = s.limits.terminal_tilt_rad +
                             (s.limits.tilt_max_rad[1] - s.limits.terminal_tilt_rad) * from_end / s.limits.tilt_taper_nodes;
        CHECK(tilt_of(plan.reference.x[static_cast<size_t>(last - from_end)]) <= limit + 1e-6);
    }
}

TEST_CASE("Thrust follows the engines' start-up at ignition", "[gnc][descent][scvx]") {
    const auto b = block3();
    auto s = settings(*b);
    s.limits.phase0_initial_spool = 0.0;
    s.limits.phase0_startup_s = 3.0;
    const auto x0 = ignition_state(*b);
    const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, state_attitude(x0)), s, nullptr, 0.0);
    REQUIRE(plan.converged);
    const double t_max = b->model.engine_sets[0].thrust_max_n;
    for (int k = 0; k < s.nodes_per_phase - 1; ++k) {
        const double t = plan.node_time_s(k);
        CHECK(norm(plan.reference.u[static_cast<size_t>(k)]) <= t_max * std::min(1.0, t / 3.0) + 1.0);
    }
}

TEST_CASE("An abort plan stays above the ground at every node", "[gnc][descent][scvx]") {
    const auto b = block3();
    auto s = settings(*b);
    s.limits.glide_slope_rad = 0.0;
    DescentTarget target;
    target.goal = DescentGoal::Abort;
    target.ground_cg_height_m = b->vehicle->mass_properties().cg_station_m;
    target.sea_target_m = {0.0, 2000.0, 0.0};
    target.keep_out_center_m = {0.0, 0.0, -30.0};
    target.keep_out_radius_m = 25.0;
    // Low and slow: the cheapest path to the sea would skim the ground.
    const auto x0 = make_state(b->vehicle->mass_properties().mass_kg, {120.0, 0.0, 0.0}, {-20.0, 15.0, 0.0}, Quat{}, {});
    const auto plan = plan_descent(b->model, x0, 1, target, s, nullptr, 0.0);
    REQUIRE(plan.converged);
    for (const auto& x : plan.reference.x) CHECK(x[descent_index::position] >= target.ground_cg_height_m - 1e-3);
}

TEST_CASE("A warm start begins at the measured snapshot", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    const auto x0 = ignition_state(*b);
    const auto target = catch_target(*b, state_attitude(x0));
    const auto first = plan_descent(b->model, x0, 0, target, s, nullptr, 0.0);
    REQUIRE(first.converged);
    const double t = 2.0;
    auto snapshot = first.state_at(t);
    snapshot[descent_index::position + 1] += 20.0;  // 20 m east of the old plan
    snapshot[descent_index::velocity + 2] -= 3.0;
    const int phase = first.phase_at(t);
    const auto ref = warm_start_reference(first, t, phase, s.nodes_per_phase, snapshot);
    CHECK(ref.x[0] == snapshot);
    const auto again = plan_descent(b->model, snapshot, phase, target, s, &first, t);
    CHECK(again.converged);
}
```

- [ ] **Step 2: Run to verify they fail**

Run: build. Expected: compile errors (`tilt_taper_nodes`, `phase0_initial_spool`, `warm_start_reference` unknown).

- [ ] **Step 3: Limits and node time**

`descent_problem.hpp`: `#include "starsim/math/constants.hpp"`; replace `DescentLimits` by

```cpp
struct DescentLimits {
    std::array<double, 2> tilt_max_rad{25.0 * constants::deg_to_rad, 10.0 * constants::deg_to_rad};  // per phase
    double terminal_tilt_rad = 4.0 * constants::deg_to_rad;   // [E] at the catch / touchdown (structural)
    int tilt_taper_nodes = 4;                                 // phase 1's last nodes taper linearly to it
    double rate_max_radps = 0.1;                              // [E] structural / controllability
    double glide_slope_rad = 30.0 * constants::deg_to_rad;    // above the horizontal, towards the catch point
    double min_mass_kg = 0.0;                                 // dry mass + reserve
    std::array<double, 2> min_phase_s{1.0, 1.0};
    double max_phase_s = 120.0;
    // Start-up of the ignition engines (thrust = spool x throttled thrust, spool rising linearly over startup_s):
    // phase 0's thrust bounds scale with the spool at each node's reference time.
    double phase0_initial_spool = 1.0;
    double phase0_startup_s = 0.0;
};
```

and below `descent_state_scales()`:

```cpp
// Time of node k from the plan start for the given phase durations (nodes equally spaced within a phase).
double reference_node_time(const std::array<double, 2>& duration_s, int k, int nodes_per_phase);
```

`descent_problem.cpp`:

```cpp
double reference_node_time(const std::array<double, 2>& duration_s, int k, int n) {
    const double step = 1.0 / (n - 1);
    return k < n ? duration_s[0] * k * step : duration_s[0] + duration_s[1] * (k - (n - 1)) * step;
}
```

(check that `guess_node_time` in `descent_planner.cpp` computes the same; if so, replace its body by a call to `reference_node_time` so there is one definition.)

- [ ] **Step 4: Constraints**

In `build_descent_subproblem`'s path-constraint loop replace

```cpp
        const double t_max = set.thrust_max_n;
        const double t_min = set.thrust_min_n;
        const double gimbal = set.gimbal_max_rad;
        const double tilt = limits.tilt_max_rad[static_cast<size_t>(p)];
```

by

```cpp
        double spool = 1.0;
        if (p == 0 && limits.phase0_startup_s > 0.0) {
            spool = std::min(1.0, limits.phase0_initial_spool +
                                      reference_node_time(ref.duration_s, k, n) / limits.phase0_startup_s);
        }
        const double t_max = set.thrust_max_n * spool;
        const double t_min = set.thrust_min_n * spool;
        const double gimbal = set.gimbal_max_rad;
        double tilt = limits.tilt_max_rad[static_cast<size_t>(p)];
        if (const int from_end = last - k; p == 1 && from_end < limits.tilt_taper_nodes) {
            tilt = limits.terminal_tilt_rad + (tilt - limits.terminal_tilt_rad) * from_end / limits.tilt_taper_nodes;
        }
```

and in the `else` branch of the goal (keep-out half-space), after `b.add_nonnegative(keep);`:

```cpp
            b.add_nonnegative(X(k, descent_index::position).add_constant(-target.ground_cg_height_m));  // above ground
```

(The planning model's mass flow keeps the full pressure-loss term during the spool-up: a few kg too much; documented
simplification, add this sentence to the `phase0_startup_s` comment.)

- [ ] **Step 5: Warm start from the snapshot**

`descent_planner.hpp`, below `initial_guess`:

```cpp
// Previous plan shifted by `elapsed_s` onto the node grid of a re-plan; nodes at time 0 are the measured `initial`
// state (the old plan's prediction there differs by the tracking error).
DescentReference warm_start_reference(const DescentPlan& previous, double elapsed_s, int start_phase, int nodes_per_phase,
                                      const DescentState& initial);
```

`descent_planner.cpp`: rename `shifted` to `warm_start_reference` (move it out of the anonymous namespace, new parameter `initial`), and in its node loop use

```cpp
        ref.x.push_back(t <= 0.0 ? initial : previous.state_at(elapsed + t));
```

In `DescentPlanner::plan`: `plan.reference = warm_start ? warm_start_reference(*warm_start, elapsed, start_phase, n, initial) : initial_guess(...);`

- [ ] **Step 6: Run the tests**

Run: build, `build/core/tests/starsim_core_tests.exe "[descent]"`
Expected: all pass. The abort-ground test may already pass before Step 4 on some builds (the optimum need not skim the ground); it stays as the guard of the constraint either way - note in the ledger which it did.

- [ ] **Step 7: Full build and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` - Expected: all pass.

```bash
git add core/include/starsim/gnc/descent_problem.hpp core/src/gnc/descent_problem.cpp core/include/starsim/gnc/descent_planner.hpp core/src/gnc/descent_planner.cpp core/tests/test_descent_planner.cpp
git commit -m "feat: SCvx spool-up bounds, tilt taper, abort ground constraint, warm start from the snapshot

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE"
```

---

### Task 5: Landing site types and the descent snapshot

**Files:**
- Create: `core/include/starsim/gnc/landing_site.hpp`, `core/src/gnc/landing_site.cpp`
- Modify: `core/include/starsim/gnc/booster_return.hpp`, `core/src/gnc/booster_return.cpp` (move `CatchTarget`, `LandingSite`, `catch_roll_reference` out)
- Create: `core/include/starsim/gnc/descent_snapshot.hpp`, `core/src/gnc/descent_snapshot.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (`return_site`, `environment`)
- Create: `core/tests/test_descent_snapshot.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `make_descent_model`, `DescentTarget`, `TowerGeometry`, `LaunchSite::sea` (Task 2).
- Produces:
  - `gnc/landing_site.hpp`: `struct SeaSide { double direction_rad; double shoreline_distance_m; };`, `struct CatchTarget { Vec3 pin_point_itrf_m; Vec3 across_itrf; double pin_station_m; double pin_azimuth_rad; Vec3 tower_center_itrf_m; double tower_keep_out_radius_m; };`, `struct LandingSite { double latitude_rad, longitude_rad, height_m; std::optional<CatchTarget> catch_target; std::optional<SeaSide> sea; };`, `Vec3 catch_roll_reference(...)` (unchanged signature)
  - `struct PlanFrame { math::Mat3 plan_to_itrf; math::Vec3 origin_itrf_m; Vec3 to_plan(const Vec3& itrf_m) const; Vec3 direction_to_plan(const Vec3&) const; Vec3 direction_to_itrf(const Vec3&) const; };`, `PlanFrame plan_frame(double latitude_rad, double longitude_rad, double height_m);`
  - `DescentState plan_state(const vehicle::FlightBody&, const frames::FixedToInertial&, const PlanFrame&);`
  - `struct SnapshotSettings { std::vector<std::string> ignition_groups, final_groups; double reserve_kg = 5000.0; double catch_clearance_m = 1.0; double catch_sink_mps = 1.0; double sea_margin_m = 150.0; };`
  - `struct DescentSnapshot { DescentModel model; DescentState initial; DescentTarget catch_target; DescentTarget abort_target; double min_mass_kg; double phase0_spool; double startup_s; time::Epoch epoch; };`
  - `DescentSnapshot take_descent_snapshot(const vehicle::FlightBody&, const vehicle::FlightEnvironment&, time::Epoch, const LandingSite&, const PlanFrame&, const SnapshotSettings&);`
  - `gnc::LandingSite FlightSession::return_site(const vehicle::Vehicle& booster) const;`, `const vehicle::FlightEnvironment& FlightSession::environment() const;`

- [ ] **Step 1: Write the failing test**

`core/tests/test_descent_snapshot.cpp`:

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/gnc/descent_snapshot.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("The descent snapshot puts the booster into the plan frame", "[gnc][descent][snapshot]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto site = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1"));
    session.set_launch_site(site);
    const auto cfg = test::booster_config("starship_block3", 70000.0);
    const vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR),
                                   test::state_over_site(site, earth, t0, {30.0, -15.0, 1200.0}, {4.0, -2.0, -250.0}));
    const auto landing = session.return_site(body.vehicle());
    REQUIRE(landing.catch_target.has_value());
    REQUIRE(landing.sea.has_value());
    const auto frame = gnc::plan_frame(landing.latitude_rad, landing.longitude_rad, landing.height_m);
    gnc::SnapshotSettings settings;
    settings.ignition_groups = {"booster/center", "booster/middle"};
    settings.final_groups = {"booster/center"};
    const auto snap = gnc::take_descent_snapshot(body, session.environment(), t0, landing, frame, settings);

    const Vec3 r = gnc::state_position(snap.initial);
    const Vec3 v = gnc::state_velocity(snap.initial);
    CHECK(std::abs(r.x - 1200.0) < 1e-6);
    CHECK(std::abs(r.y - 30.0) < 1e-6);
    CHECK(std::abs(r.z + 15.0) < 1e-6);
    CHECK(std::abs(v.x + 250.0) < 1e-6);
    CHECK(std::abs(v.y - 4.0) < 1e-6);
    CHECK(std::abs(v.z + 2.0) < 1e-6);
    const auto q = gnc::state_attitude(snap.initial);  // upright, body +y east: identity
    CHECK(std::abs(std::abs(q.w) - 1.0) < 1e-9);
    CHECK(norm(gnc::state_rate(snap.initial)) < 1e-12);  // co-rotating with the Earth
    CHECK(snap.initial[gnc::descent_index::mass] == body.vehicle().mass_properties().mass_kg);

    const double lat = site.latitude_rad;
    const double omega = norm(snap.model.earth_rate_radps);
    CHECK(std::abs(omega - 7.292115e-5) < 1e-9);
    CHECK(std::abs(snap.model.earth_rate_radps.x - omega * std::sin(lat)) < 1e-9);
    CHECK(std::abs(snap.model.earth_rate_radps.z - omega * std::cos(lat)) < 1e-9);
    CHECK(snap.model.gravity_mps2.x < -9.7);
    CHECK(snap.model.gravity_mps2.x > -9.85);
    CHECK(snap.model.earth_center_m.x < -6.3e6);

    const double cg = body.vehicle().mass_properties().cg_station_m;
    const auto& tower = *site.tower;
    CHECK(std::abs(snap.catch_target.position_m.x - (tower.catch_height_m + 1.0 - (landing.catch_target->pin_station_m - cg))) < 1e-6);
    CHECK(std::abs(snap.catch_target.position_m.y) < 1e-6);
    CHECK(std::abs(snap.catch_target.position_m.z) < 1e-6);
    CHECK(snap.catch_target.velocity_mps.x == -1.0);
    CHECK(snap.abort_target.goal == gnc::DescentGoal::Abort);
    CHECK(std::abs(snap.abort_target.sea_target_m.y - (site.sea->shoreline_distance_m + 150.0)) < 1e-6);  // due east
    CHECK(snap.abort_target.keep_out_radius_m > 0.5 * tower.width_m);
    CHECK(snap.min_mass_kg == body.vehicle().mass_properties().mass_kg - body.vehicle().propellant_kg() + 5000.0);
    CHECK(snap.phase0_spool == 0.0);  // engines off
    CHECK(snap.startup_s == 3.0);     // Raptor 3 start-up [E]
}
```

Add `test_descent_snapshot.cpp` to `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Run to verify it fails**

Run: build. Expected: compile error (`descent_snapshot.hpp` missing).

- [ ] **Step 3: Move the landing-site types**

`core/include/starsim/gnc/landing_site.hpp`:

```cpp
#pragma once

#include <optional>

#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// Coastline of the landing site: ground beyond the shoreline (perpendicular to direction_rad) is water.
struct SeaSide {
    double direction_rad = 0.0;          // pad centre -> sea, clockwise from north
    double shoreline_distance_m = 0.0;
};

// Catch on the tower arms instead of a touchdown on the pad.
struct CatchTarget {
    math::Vec3 pin_point_itrf_m;   // pins' midpoint on a centred catch: on the catch axis at the arm tops
    math::Vec3 across_itrf;        // horizontal unit; the pins must point along +-this direction
    double pin_station_m = 0.0;    // vehicle frame
    double pin_azimuth_rad = 0.0;  // first pin, from body +y towards +z
    math::Vec3 tower_center_itrf_m;      // tower axis on the ground (abort keep-out)
    double tower_keep_out_radius_m = 0.0; // around the tower axis, incl. the hull radius and a margin
};

struct LandingSite {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;  // ground (pad) height above WGS84
    std::optional<CatchTarget> catch_target;
    std::optional<SeaSide> sea;
};

// Body +y direction (same frame as the inputs) that turns the first pin along +-across, upright body (x = up);
// of the two solutions the one nearer `current_body_y` (the pins are symmetric).
math::Vec3 catch_roll_reference(const math::Vec3& across, const math::Vec3& up, double pin_azimuth_rad,
                                const math::Vec3& current_body_y);

}  // namespace starsim::gnc
```

Delete `CatchTarget`, `LandingSite` and the `catch_roll_reference` declaration from `booster_return.hpp` and include `"starsim/gnc/landing_site.hpp"` there. Move the definition of `catch_roll_reference` from `booster_return.cpp` into the new `core/src/gnc/landing_site.cpp` (with `#include "starsim/gnc/landing_site.hpp"`, `<cmath>`, the same body). The `stage()` construction `gnc::LandingSite landing{lat, lon, h, {}}` keeps compiling (the new members default).

- [ ] **Step 4: Session helpers**

`flight_session.hpp` public:

```cpp
    // Landing site of a returning `booster`: catch target when the site has a tower and the booster has pins,
    // coastline when configured.
    gnc::LandingSite return_site(const vehicle::Vehicle& booster) const;
    const vehicle::FlightEnvironment& environment() const { return env_; }
```

`flight_session.cpp`, in the anonymous namespace (create one at the top if there is none):

```cpp
constexpr double tower_keep_out_margin_m = 5.0;  // [E] aborted landings stay this far from the tower lattice
```

and

```cpp
gnc::LandingSite FlightSession::return_site(const vehicle::Vehicle& booster) const {
    gnc::LandingSite landing{launch_site_.latitude_rad, launch_site_.longitude_rad, launch_site_.height_m, {}, {}};
    const auto& stage = booster.stages().front();
    if (tower_ && !stage.config.catch_pins.azimuths_rad.empty()) {
        const auto& g = tower_->geometry();
        const auto& c = g.config();
        landing.catch_target = gnc::CatchTarget{g.to_itrf({0.0, 0.0, g.arm_top_height_m(0.0)}),
                                                g.site_to_itrf() * g.across(),
                                                stage.base_station_m + stage.config.catch_pins.station_m,
                                                stage.config.catch_pins.azimuths_rad.front(),
                                                g.to_itrf(g.tower_direction() * c.distance_m),
                                                0.5 * c.width_m * std::sqrt(2.0) + 0.5 * booster.diameter_m() +
                                                    tower_keep_out_margin_m};
    }
    if (launch_site_.sea) landing.sea = gnc::SeaSide{launch_site_.sea->direction_rad, launch_site_.sea->shoreline_distance_m};
    return landing;
}
```

In `stage()` replace the block that builds `landing` (from `gnc::LandingSite landing{...}` to the closing brace of `if (tower_ && ...)`) by `const auto landing = return_site(lower.body->vehicle());`.

- [ ] **Step 5: Snapshot**

`core/include/starsim/gnc/descent_snapshot.hpp`:

```cpp
#pragma once

#include <string>
#include <vector>

#include "starsim/frames/body_orientation.hpp"
#include "starsim/gnc/descent_problem.hpp"
#include "starsim/gnc/landing_site.hpp"
#include "starsim/time/epoch.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

// Plan frame of a landing site: origin at the pad centre on the ground, x up (geodetic vertical), y east, z north.
struct PlanFrame {
    math::Mat3 plan_to_itrf;   // columns: up, east, north
    math::Vec3 origin_itrf_m;

    math::Vec3 to_plan(const math::Vec3& itrf_m) const { return transpose(plan_to_itrf) * (itrf_m - origin_itrf_m); }
    math::Vec3 direction_to_plan(const math::Vec3& itrf) const { return transpose(plan_to_itrf) * itrf; }
    math::Vec3 direction_to_itrf(const math::Vec3& plan) const { return plan_to_itrf * plan; }
};

PlanFrame plan_frame(double latitude_rad, double longitude_rad, double height_m);

// Current state of the body in the plan frame: velocity relative to the ground, attitude body -> plan, body rate
// relative to the (rotating) plan frame. `earth`: the Earth-fixed -> GCRF transform at the current epoch.
DescentState plan_state(const vehicle::FlightBody& body, const frames::FixedToInertial& earth, const PlanFrame& frame);

struct SnapshotSettings {
    std::vector<std::string> ignition_groups;  // phase 0 engine set
    std::vector<std::string> final_groups;     // phase 1 engine set
    double reserve_kg = 5000.0;        // [E] propellant the plan must leave (residuals, unusable)
    double catch_clearance_m = 1.0;    // pins above the arm tops at the end of the plan (spec)
    double catch_sink_mps = 1.0;       // sink rate at the end of the plan (spec)
    double sea_margin_m = 150.0;       // [E] abort target beyond the shoreline
};

// Everything the planner needs, taken on the simulation thread (the worker must not touch SPICE, the environment
// or the vehicle): the model with its own tables, the start state and both targets.
struct DescentSnapshot {
    DescentModel model;
    DescentState initial;
    DescentTarget catch_target;   // goal Catch (meaningless without a catch target at the site)
    DescentTarget abort_target;   // goal Abort
    double min_mass_kg = 0.0;
    double phase0_spool = 1.0;    // mean spool of the ignition engines now
    double startup_s = 0.0;       // their start-up time
    time::Epoch epoch;
};

DescentSnapshot take_descent_snapshot(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                      time::Epoch epoch, const LandingSite& site, const PlanFrame& frame,
                                      const SnapshotSettings& settings);

}  // namespace starsim::gnc
```

`core/src/gnc/descent_snapshot.cpp`:

```cpp
#include "starsim/gnc/descent_snapshot.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/frames/geodetic.hpp"

namespace starsim::gnc {

using math::Mat3;
using math::Quat;
using math::Vec3;

namespace {

Mat3 from_columns(const Vec3& a, const Vec3& b, const Vec3& c) {
    Mat3 m;
    m(0, 0) = a.x; m(1, 0) = a.y; m(2, 0) = a.z;
    m(0, 1) = b.x; m(1, 1) = b.y; m(2, 1) = b.z;
    m(0, 2) = c.x; m(1, 2) = c.y; m(2, 2) = c.z;
    return m;
}

bool contains(const std::vector<std::string>& groups, const std::string& group) {
    return std::find(groups.begin(), groups.end(), group) != groups.end();
}

}  // namespace

PlanFrame plan_frame(double latitude_rad, double longitude_rad, double height_m) {
    const double sl = std::sin(latitude_rad), cl = std::cos(latitude_rad);
    const double so = std::sin(longitude_rad), co = std::cos(longitude_rad);
    return {from_columns({cl * co, cl * so, sl}, {-so, co, 0.0}, {-sl * co, -sl * so, cl}),
            frames::geodetic_to_fixed({latitude_rad, longitude_rad, height_m})};
}

DescentState plan_state(const vehicle::FlightBody& body, const frames::FixedToInertial& earth, const PlanFrame& frame) {
    const auto& s = body.state();
    const Mat3 to_fixed = transpose(earth.rotation);
    const Vec3 omega = vehicle::earth_rotation_vector(earth);  // GCRF
    const Vec3 r_fixed = to_fixed * s.position_gcrf_m;
    const Vec3 v_fixed = to_fixed * (s.velocity_gcrf_mps - cross(omega, s.position_gcrf_m));
    const Quat gcrf_to_plan = Quat::from_matrix(transpose(frame.plan_to_itrf) * to_fixed);
    return make_state(body.vehicle().mass_properties().mass_kg, frame.to_plan(r_fixed), frame.direction_to_plan(v_fixed),
                      (gcrf_to_plan * s.attitude).normalized(),
                      s.angular_velocity_body_radps - s.attitude.inverse_rotate(omega));
}

DescentSnapshot take_descent_snapshot(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                      time::Epoch epoch, const LandingSite& site, const PlanFrame& frame,
                                      const SnapshotSettings& settings) {
    const auto t = env.earth_orientation.fixed_to_gcrf(epoch);
    const Mat3 gcrf_to_plan = transpose(frame.plan_to_itrf) * transpose(t.rotation);
    const auto& vehicle = body.vehicle();
    const auto mass = vehicle.mass_properties();

    DescentModelInputs in;
    in.vehicle = &vehicle;
    in.ignition_groups = settings.ignition_groups;
    in.final_groups = settings.final_groups;
    in.atmosphere = &env.atmosphere;
    in.site_height_m = site.height_m;
    in.gravity_mps2 = gcrf_to_plan * env.gravity.acceleration_mps2(t.rotation * frame.origin_itrf_m, epoch);
    in.earth_rate_radps = gcrf_to_plan * vehicle::earth_rotation_vector(t);
    in.earth_center_m = frame.to_plan({0.0, 0.0, 0.0});

    DescentSnapshot snap;
    snap.model = make_descent_model(in);
    snap.initial = plan_state(body, t, frame);
    snap.epoch = epoch;
    snap.min_mass_kg = mass.mass_kg - vehicle.propellant_kg() + settings.reserve_kg;

    if (site.catch_target) {
        const auto& c = *site.catch_target;
        const Vec3 pins = frame.to_plan(c.pin_point_itrf_m) + Vec3{settings.catch_clearance_m, 0.0, 0.0};
        snap.catch_target.goal = DescentGoal::Catch;
        snap.catch_target.position_m = pins - Vec3{c.pin_station_m - mass.cg_station_m, 0.0, 0.0};
        snap.catch_target.velocity_mps = {-settings.catch_sink_mps, 0.0, 0.0};
        const Vec3 tower = frame.to_plan(c.tower_center_itrf_m);
        snap.abort_target.keep_out_center_m = {0.0, tower.y, tower.z};
        snap.abort_target.keep_out_radius_m = c.tower_keep_out_radius_m;
    }
    snap.abort_target.goal = DescentGoal::Abort;
    snap.abort_target.ground_cg_height_m = mass.cg_station_m;  // bottom on the ground (flat at the pad height)
    if (site.sea) {
        const Vec3 seaward{0.0, std::sin(site.sea->direction_rad), std::cos(site.sea->direction_rad)};
        snap.abort_target.sea_target_m = seaward * (site.sea->shoreline_distance_m + settings.sea_margin_m);
    }

    double spool = 0.0;
    int engines = 0;
    for (const auto& e : vehicle.stages().front().engines) {
        if (e.starved || !contains(settings.ignition_groups, e.group_id)) continue;
        spool += e.spool;
        snap.startup_s = std::max(snap.startup_s, e.performance->config().startup_s);
        ++engines;
    }
    snap.phase0_spool = engines > 0 ? spool / engines : 1.0;
    return snap;
}

}  // namespace starsim::gnc
```

Add `src/gnc/landing_site.cpp` and `src/gnc/descent_snapshot.cpp` to `core/CMakeLists.txt` after `src/gnc/descent_planner.cpp`.

- [ ] **Step 6: Run the tests**

Run: build, `build/core/tests/starsim_core_tests.exe "[snapshot]"`, `build/core/tests/starsim_core_tests.exe "[booster]"`
Expected: all pass. If the attitude check fails with |w| ~ 0.7, the plan-frame column order (up, east, north) or the `state_over_site` body axes (x up, y east) disagree - fix the code, not the test.

- [ ] **Step 7: Full build and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` - Expected: all pass.

```bash
git add core/include/starsim/gnc/landing_site.hpp core/src/gnc/landing_site.cpp core/include/starsim/gnc/booster_return.hpp core/src/gnc/booster_return.cpp core/include/starsim/gnc/descent_snapshot.hpp core/src/gnc/descent_snapshot.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/tests/test_descent_snapshot.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat: descent snapshot - plan frame, model, start state and targets taken on the sim thread

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE"
```

---

### Task 6: `DescentReplanner` - worker thread with a fixed sim-time latency

**Files:**
- Create: `core/include/starsim/gnc/descent_replanner.hpp`, `core/src/gnc/descent_replanner.cpp`
- Create: `core/tests/test_descent_replanner.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `DescentPlanner` (Task 3), `DescentModel`, `DescentTarget`, `ScvxSettings`.
- Produces:
  - `struct PlanRequest { DescentModel model; DescentState initial; int start_phase = 0; DescentTarget target; ScvxSettings settings; std::shared_ptr<const DescentPlan> warm_start; double warm_start_elapsed_s = 0.0; time::Epoch snapshot_epoch; };`
  - `struct PlanOutcome { std::shared_ptr<const DescentPlan> plan; time::Epoch snapshot_epoch; double solve_ms = 0.0; };`
  - `using PlanFunction = std::function<DescentPlan(DescentPlanner&, const PlanRequest&)>;`, `PlanFunction default_plan_function();`
  - `class DescentReplanner { explicit DescentReplanner(double latency_s, PlanFunction plan = default_plan_function()); ~DescentReplanner(); PlanOutcome plan_now(const PlanRequest&); void submit(PlanRequest); bool busy() const; std::optional<PlanOutcome> collect(time::Epoch now, bool wait); };`

- [ ] **Step 1: Write the failing tests**

`core/tests/test_descent_replanner.cpp`:

```cpp
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <thread>

#include "starsim/gnc/descent_replanner.hpp"

using namespace starsim;
using namespace starsim::gnc;

namespace {

time::Epoch at(double s) { return time::Epoch::from_tdb_seconds_past_j2000(s); }

// Fake planner: marks the plan with the request's start phase, optionally slow.
PlanFunction fake(std::chrono::milliseconds delay, std::atomic<int>* calls = nullptr) {
    return [delay, calls](DescentPlanner&, const PlanRequest& r) {
        std::this_thread::sleep_for(delay);
        if (calls) ++*calls;
        DescentPlan p;
        p.start_phase = r.start_phase;
        p.converged = true;
        return p;
    };
}

PlanRequest request(double snapshot_s, int phase = 0) {
    PlanRequest r;
    r.snapshot_epoch = at(snapshot_s);
    r.start_phase = phase;
    return r;
}

}  // namespace

TEST_CASE("A re-plan takes effect at its snapshot plus the latency", "[gnc][replanner]") {
    DescentReplanner replanner(0.5, fake(std::chrono::milliseconds(0)));
    replanner.submit(request(10.0, 1));
    CHECK(replanner.busy());
    CHECK_FALSE(replanner.collect(at(10.4), true).has_value());  // too early, even when waiting
    const auto outcome = replanner.collect(at(10.5), true);
    REQUIRE(outcome.has_value());
    CHECK(outcome->plan->start_phase == 1);
    CHECK(outcome->snapshot_epoch - at(10.0) == 0.0);  // Epoch has no operator==
    CHECK_FALSE(replanner.busy());
}

TEST_CASE("Waiting mode blocks until the plan is ready", "[gnc][replanner]") {
    DescentReplanner replanner(0.5, fake(std::chrono::milliseconds(200)));
    replanner.submit(request(0.0));
    CHECK(replanner.collect(at(0.5), true).has_value());
}

TEST_CASE("Real-time mode never waits for a slow plan", "[gnc][replanner]") {
    DescentReplanner replanner(0.5, fake(std::chrono::milliseconds(300)));
    replanner.submit(request(0.0));
    const auto start = std::chrono::steady_clock::now();
    CHECK_FALSE(replanner.collect(at(0.5), false).has_value());
    CHECK(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(50));
    CHECK(replanner.busy());
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    CHECK(replanner.collect(at(0.6), false).has_value());  // used once it arrives
}

TEST_CASE("The first plan is made synchronously", "[gnc][replanner]") {
    std::atomic<int> calls{0};
    DescentReplanner replanner(0.5, fake(std::chrono::milliseconds(0), &calls));
    const auto outcome = replanner.plan_now(request(3.0, 0));
    CHECK(calls == 1);
    CHECK(outcome.plan->converged);
    CHECK(outcome.solve_ms >= 0.0);
    CHECK_FALSE(replanner.busy());
}

TEST_CASE("Destroying the replanner while it works waits for the worker", "[gnc][replanner]") {
    std::atomic<int> calls{0};
    {
        DescentReplanner replanner(0.5, fake(std::chrono::milliseconds(150), &calls));
        replanner.submit(request(0.0));
    }
    CHECK(calls == 1);
}
```

Add `test_descent_replanner.cpp` to `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Run to verify they fail**

Run: build. Expected: compile error (`descent_replanner.hpp` missing).

- [ ] **Step 3: Implement**

`core/include/starsim/gnc/descent_replanner.hpp`:

```cpp
#pragma once

#include <functional>
#include <future>
#include <memory>
#include <optional>

#include "starsim/gnc/descent_planner.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::gnc {

// Everything one re-plan needs; owned by the request (the worker touches nothing else).
struct PlanRequest {
    DescentModel model;
    DescentState initial;
    int start_phase = 0;
    DescentTarget target;
    ScvxSettings settings;
    std::shared_ptr<const DescentPlan> warm_start;  // null: fresh plan
    double warm_start_elapsed_s = 0.0;
    time::Epoch snapshot_epoch;                     // the plan starts here (node 0 = initial)
};

struct PlanOutcome {
    std::shared_ptr<const DescentPlan> plan;
    time::Epoch snapshot_epoch;
    double solve_ms = 0.0;  // wall-clock time of the planner
};

using PlanFunction = std::function<DescentPlan(DescentPlanner&, const PlanRequest&)>;
PlanFunction default_plan_function();  // DescentPlanner::plan on the request

// Plans on one worker thread with a fixed latency in simulation time: a plan from a snapshot at t takes effect at
// t + latency. Waiting mode (tests, time warp) blocks at that moment until the plan is ready - deterministic.
// Real-time mode never blocks: a late plan is used at the first collect after it is ready. One request at a time;
// the planner (and its solvers) is used by one thread at a time. Not thread-safe itself: call from one thread.
class DescentReplanner {
public:
    explicit DescentReplanner(double latency_s, PlanFunction plan = default_plan_function());
    ~DescentReplanner();  // waits for a running request
    DescentReplanner(const DescentReplanner&) = delete;
    DescentReplanner& operator=(const DescentReplanner&) = delete;

    PlanOutcome plan_now(const PlanRequest& request);  // synchronous; precondition: !busy()
    void submit(PlanRequest request);                  // precondition: !busy()
    bool busy() const { return pending_.valid(); }     // submitted and not yet collected
    std::optional<PlanOutcome> collect(time::Epoch now, bool wait);

private:
    double latency_s_;
    PlanFunction plan_;
    DescentPlanner planner_;
    std::future<PlanOutcome> pending_;
    time::Epoch pending_snapshot_;
};

}  // namespace starsim::gnc
```

`core/src/gnc/descent_replanner.cpp`:

```cpp
#include "starsim/gnc/descent_replanner.hpp"

#include <chrono>
#include <stdexcept>

namespace starsim::gnc {

namespace {

PlanOutcome run(DescentPlanner& planner, const PlanFunction& plan, const PlanRequest& request) {
    const auto start = std::chrono::steady_clock::now();
    PlanOutcome outcome;
    outcome.plan = std::make_shared<const DescentPlan>(plan(planner, request));
    outcome.snapshot_epoch = request.snapshot_epoch;
    outcome.solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return outcome;
}

}  // namespace

PlanFunction default_plan_function() {
    return [](DescentPlanner& planner, const PlanRequest& r) {
        return planner.plan(r.model, r.initial, r.start_phase, r.target, r.settings, r.warm_start.get(),
                            r.warm_start_elapsed_s);
    };
}

DescentReplanner::DescentReplanner(double latency_s, PlanFunction plan) : latency_s_(latency_s), plan_(std::move(plan)) {}

DescentReplanner::~DescentReplanner() {
    if (pending_.valid()) pending_.wait();
}

PlanOutcome DescentReplanner::plan_now(const PlanRequest& request) {
    if (busy()) throw std::logic_error("DescentReplanner::plan_now while a request is pending");
    return run(planner_, plan_, request);
}

void DescentReplanner::submit(PlanRequest request) {
    if (busy()) throw std::logic_error("DescentReplanner::submit while a request is pending");
    pending_snapshot_ = request.snapshot_epoch;
    pending_ = std::async(std::launch::async, [this, r = std::move(request)] { return run(planner_, plan_, r); });
}

std::optional<PlanOutcome> DescentReplanner::collect(time::Epoch now, bool wait) {
    if (!pending_.valid() || now - pending_snapshot_ < latency_s_ - 1e-9) return std::nullopt;
    if (!wait && pending_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return std::nullopt;
    return pending_.get();
}

}  // namespace starsim::gnc
```

(`time::Epoch` difference `a - b` returns seconds, `epoch + seconds` an epoch.) Add `src/gnc/descent_replanner.cpp` to `core/CMakeLists.txt`. If the linker reports missing thread symbols, add `find_package(Threads REQUIRED)` and `Threads::Threads` to the core library's `target_link_libraries`.

- [ ] **Step 4: Run the tests**

Run: build, `build/core/tests/starsim_core_tests.exe "[replanner]"`
Expected: all 5 pass.

- [ ] **Step 5: Full build and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` - Expected: all pass.

```bash
git add core/include/starsim/gnc/descent_replanner.hpp core/src/gnc/descent_replanner.cpp core/tests/test_descent_replanner.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat: DescentReplanner - SCvx re-plans on a worker thread with a fixed sim-time latency

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE"
```

---

### Task 7: Plan tracking and attitude feed-forward

**Files:**
- Create: `core/include/starsim/gnc/descent_tracking.hpp`, `core/src/gnc/descent_tracking.cpp`
- Modify: `core/include/starsim/gnc/attitude_controller.hpp`, `core/src/gnc/attitude_controller.cpp`
- Create: `core/tests/test_descent_tracking.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `DescentPlan::{state_at, thrust_at, phase_at, duration_s}`, `DescentModel::gravity_mps2`.
- Produces:
  - `struct TrackingReference { int phase; bool finished; Vec3 position_m, velocity_mps, thrust_acceleration_mps2, axis; Vec3 rate_body_radps, angular_acceleration_body_radps2; };` (plan frame; rates relative to the plan frame, body axes)
  - `TrackingReference sample_plan(const DescentPlan&, const DescentModel&, double t_s, double terminal_sink_mps);`
  - `struct TrackingGains { double frequency_rad_s = 0.6; double damping = 1.0; double max_correction_mps2 = 2.0; double max_tilt_rad = 25 deg; };`
  - `math::Vec3 commanded_acceleration(const TrackingReference&, const math::Vec3& position_m, const math::Vec3& velocity_mps, const TrackingGains&);` (plan frame, thrust acceleration incl. correction, tilt-limited)
  - `struct AttitudeController::Feedforward { math::Vec3 rate_body_radps; math::Vec3 acceleration_body_radps2; };` and `steer(body, pressure, axis_gcrf, roll_reference_gcrf = {}, const Feedforward* feedforward = nullptr)`

- [ ] **Step 1: Write the failing tests**

`core/tests/test_descent_tracking.cpp`:

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/gnc/descent_tracking.hpp"

using namespace starsim;
using namespace starsim::gnc;
using math::Quat;
using math::Vec3;

namespace {

// Two phases of 2 s, two nodes each (3 nodes): straight down, the rate about body z rising 0 -> 0.2 rad/s.
DescentPlan synthetic_plan() {
    DescentPlan p;
    p.nodes_per_phase = 2;
    p.converged = true;
    p.reference.duration_s = {2.0, 2.0};
    p.reference.x = {make_state(1.0e5, {100.0, 0.0, 0.0}, {-20.0, 0.0, 0.0}, Quat{}, {0.0, 0.0, 0.0}),
                     make_state(0.99e5, {70.0, 0.0, 0.0}, {-10.0, 0.0, 0.0}, Quat{}, {0.0, 0.0, 0.2}),
                     make_state(0.98e5, {60.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, Quat{}, {0.0, 0.0, 0.2})};
    p.reference.u = {{2.0e6, 0.0, 0.0}, {1.5e6, 0.0, 0.0}, {1.0e6, 0.0, 0.0}};
    return p;
}

DescentModel flat_model() {
    DescentModel m;
    m.gravity_mps2 = {-9.8, 0.0, 0.0};
    return m;
}

}  // namespace

TEST_CASE("Plan samples interpolate between nodes", "[gnc][tracking]") {
    const auto ref = sample_plan(synthetic_plan(), flat_model(), 1.0, 1.0);
    CHECK(ref.phase == 0);
    CHECK_FALSE(ref.finished);
    CHECK(std::abs(ref.position_m.x - 85.0) < 1e-9);
    CHECK(std::abs(ref.velocity_mps.x + 15.0) < 1e-9);
    CHECK(std::abs(ref.thrust_acceleration_mps2.x - 1.75e6 / 0.995e5) < 1e-6);
    CHECK(std::abs(ref.axis.x - 1.0) < 1e-12);
    CHECK(std::abs(ref.rate_body_radps.z - 0.1) < 1e-9);
    CHECK(std::abs(ref.angular_acceleration_body_radps2.z - 0.1) < 1e-6);
    CHECK(sample_plan(synthetic_plan(), flat_model(), 3.0, 1.0).phase == 1);
}

TEST_CASE("After the plan's end the reference keeps sinking slowly against gravity", "[gnc][tracking]") {
    const auto ref = sample_plan(synthetic_plan(), flat_model(), 6.0, 1.0);
    CHECK(ref.finished);
    CHECK(ref.phase == 1);
    CHECK(std::abs(ref.velocity_mps.x + 1.0) < 1e-12);
    CHECK(std::abs(ref.position_m.x - 58.0) < 1e-9);  // 60 m - 1 m/s x 2 s
    CHECK(std::abs(ref.thrust_acceleration_mps2.x - 9.8) < 1e-12);
    CHECK(norm(ref.rate_body_radps) == 0.0);
}

TEST_CASE("Tracking corrects position and velocity errors within limits", "[gnc][tracking]") {
    const auto ref = sample_plan(synthetic_plan(), flat_model(), 1.0, 1.0);
    const TrackingGains gains;
    const Vec3 exact = commanded_acceleration(ref, ref.position_m, ref.velocity_mps, gains);
    CHECK(norm(exact - ref.thrust_acceleration_mps2) < 1e-12);
    const Vec3 off = commanded_acceleration(ref, ref.position_m + Vec3{0.0, 50.0, 0.0}, ref.velocity_mps, gains);
    CHECK(norm(off - ref.thrust_acceleration_mps2) <= gains.max_correction_mps2 + 1e-12);
    CHECK(off.y < 0.0);  // pushes back towards the plan
    const double tilt = std::atan2(std::hypot(off.y, off.z), off.x);
    CHECK(tilt <= gains.max_tilt_rad + 1e-12);
}

TEST_CASE("Rate feed-forward removes the damping torque for a planned rotation", "[gnc][tracking]") {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.resize(1);
    vehicle::RigidBodyState s;
    s.position_gcrf_m = {6.4e6, 0.0, 0.0};
    s.angular_velocity_body_radps = {0.0, 0.02, 0.0};  // engines off: RCS
    const vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR), s);
    const AttitudeController controller;
    const Vec3 axis = s.attitude.rotate({1.0, 0.0, 0.0});
    const auto plain = controller.steer(body, 0.0, axis);
    AttitudeController::Feedforward ff;
    ff.rate_body_radps = {0.0, 0.02, 0.0};
    const auto fed = controller.steer(body, 0.0, axis, {}, &ff);
    CHECK(plain.rcs_torque_fraction.y < -1e-3);          // damps the rate
    CHECK(std::abs(fed.rcs_torque_fraction.y) < 1e-9);   // the rate is planned: nothing to damp
}
```

Add `test_descent_tracking.cpp` to `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Run to verify they fail**

Run: build. Expected: compile error (`descent_tracking.hpp` missing).

- [ ] **Step 3: Tracking**

`core/include/starsim/gnc/descent_tracking.hpp`:

```cpp
#pragma once

#include "starsim/gnc/descent_planner.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

// What the active plan asks for at one moment (plan frame; rates body axes, relative to the plan frame).
struct TrackingReference {
    int phase = 0;
    bool finished = false;                     // past the plan's end: slow sink against gravity until the catch
    math::Vec3 position_m;
    math::Vec3 velocity_mps;
    math::Vec3 thrust_acceleration_mps2;       // planned thrust / planned mass
    math::Vec3 axis;                           // planned body x-axis
    math::Vec3 rate_body_radps;
    math::Vec3 angular_acceleration_body_radps2;
};

TrackingReference sample_plan(const DescentPlan& plan, const DescentModel& model, double t_s, double terminal_sink_mps);

struct TrackingGains {
    double frequency_rad_s = 0.6;     // [E] position loop (re-plans every second remove the rest)
    double damping = 1.0;
    double max_correction_mps2 = 2.0; // [E] keeps the correction a correction (the plan does the flying)
    double max_tilt_rad = 25.0 * constants::deg_to_rad;  // thrust from the vertical (phase 0 limit)
};

// Thrust acceleration to fly now: planned plus a PD correction of the position/velocity error (bounded), the
// direction limited to max_tilt from the vertical.
math::Vec3 commanded_acceleration(const TrackingReference& reference, const math::Vec3& position_m,
                                  const math::Vec3& velocity_mps, const TrackingGains& gains);

}  // namespace starsim::gnc
```

`core/src/gnc/descent_tracking.cpp`:

```cpp
#include "starsim/gnc/descent_tracking.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double rate_difference_step_s = 0.05;  // central difference of the planned rate

}  // namespace

TrackingReference sample_plan(const DescentPlan& plan, const DescentModel& model, double t, double terminal_sink_mps) {
    TrackingReference r;
    const double end = plan.duration_s();
    if (t >= end) {
        const auto& x = plan.reference.x.back();
        r.phase = 1;
        r.finished = true;
        r.velocity_mps = {-terminal_sink_mps, 0.0, 0.0};
        r.position_m = state_position(x) + r.velocity_mps * (t - end);
        r.thrust_acceleration_mps2 = -model.gravity_mps2;
        r.axis = {1.0, 0.0, 0.0};
        return r;
    }
    const auto x = plan.state_at(t);
    const auto q = state_attitude(x);
    r.phase = plan.phase_at(t);
    r.position_m = state_position(x);
    r.velocity_mps = state_velocity(x);
    r.thrust_acceleration_mps2 = q.rotate(plan.thrust_at(t)) / x[descent_index::mass];
    r.axis = q.rotate({1.0, 0.0, 0.0});
    r.rate_body_radps = state_rate(x);
    const double a = std::max(t - rate_difference_step_s, 0.0);
    const double b = std::min(t + rate_difference_step_s, end);
    if (b > a) r.angular_acceleration_body_radps2 = (state_rate(plan.state_at(b)) - state_rate(plan.state_at(a))) / (b - a);
    return r;
}

Vec3 commanded_acceleration(const TrackingReference& ref, const Vec3& position_m, const Vec3& velocity_mps,
                            const TrackingGains& g) {
    const double w = g.frequency_rad_s;
    Vec3 correction = (ref.position_m - position_m) * (w * w) + (ref.velocity_mps - velocity_mps) * (2.0 * g.damping * w);
    if (const double n = norm(correction); n > g.max_correction_mps2) correction = correction * (g.max_correction_mps2 / n);
    Vec3 a = ref.thrust_acceleration_mps2 + correction;
    const double up = std::max(a.x, 0.0);
    const double side = std::hypot(a.y, a.z);
    const double side_max = up * std::tan(g.max_tilt_rad);
    if (side > side_max && side > 0.0) {
        a.y *= side_max / side;
        a.z *= side_max / side;
    }
    return a;
}

}  // namespace starsim::gnc
```

(The plan's `state_at` interpolates linearly, so the planned rate is piecewise linear and its central difference is the planned angular acceleration.)

- [ ] **Step 4: Attitude feed-forward**

`attitude_controller.hpp`, inside `class AttitudeController` public, before `steer`:

```cpp
    // Planned motion the controller should not fight: body rate (relative to inertial space) and angular
    // acceleration, body axes. The roll components are ignored (roll follows the roll reference).
    struct Feedforward {
        math::Vec3 rate_body_radps;
        math::Vec3 acceleration_body_radps2;
    };
```

and change `steer` to

```cpp
    ControlOutput steer(const vehicle::FlightBody& body, double ambient_pressure_pa, const math::Vec3& axis_gcrf,
                        const math::Vec3& roll_reference_gcrf = {}, const Feedforward* feedforward = nullptr) const;
```

`attitude_controller.cpp` `steer`: same new parameter; replace the `rate` and `torque` lines by

```cpp
    const Vec3 ff_rate = feedforward ? Vec3{0.0, feedforward->rate_body_radps.y, feedforward->rate_body_radps.z} : Vec3{};
    const Vec3 ff_acceleration =
        feedforward ? Vec3{0.0, feedforward->acceleration_body_radps2.y, feedforward->acceleration_body_radps2.z} : Vec3{};
    const Vec3 rate{rate_command(roll_error, gain, max_acceleration(a.roll_moment, rcs_max.x, fin_max.x, inertia.x)),
                    rate_command(rotation.y, gain, max_acceleration(a.lever_moment, rcs_max.y, fin_max.y, inertia.y)) + ff_rate.y,
                    rate_command(rotation.z, gain, max_acceleration(a.lever_moment, rcs_max.z, fin_max.z, inertia.z)) + ff_rate.z};
    const Vec3 torque{inertia.x * kd * (rate.x - w.x), inertia.y * (kd * (rate.y - w.y) + ff_acceleration.y),
                      inertia.z * (kd * (rate.z - w.z) + ff_acceleration.z)};
```

Add `src/gnc/descent_tracking.cpp` to `core/CMakeLists.txt`.

- [ ] **Step 5: Run the tests**

Run: build, `build/core/tests/starsim_core_tests.exe "[tracking]"`, `build/core/tests/starsim_core_tests.exe "[booster]"`
Expected: all pass (existing callers of `steer` compile unchanged through the default argument).

- [ ] **Step 6: Full build and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` - Expected: all pass.

```bash
git add core/include/starsim/gnc/descent_tracking.hpp core/src/gnc/descent_tracking.cpp core/include/starsim/gnc/attitude_controller.hpp core/src/gnc/attitude_controller.cpp core/tests/test_descent_tracking.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat: descent plan tracking and attitude rate/acceleration feed-forward

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE"
```

---

### Task 8: `ScvxLandingGuidance` - first plan, re-plans, abort decision, commands

**Files:**
- Create: `core/include/starsim/gnc/scvx_landing.hpp`, `core/src/gnc/scvx_landing.cpp`
- Create: `core/tests/test_scvx_landing.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 5 (`plan_frame`, `plan_state`, `take_descent_snapshot`, `SnapshotSettings`, `LandingSite`, `catch_roll_reference`), Task 6 (`DescentReplanner`, `PlanRequest`, `PlanOutcome`, `PlanFunction`), Task 7 (`sample_plan`, `commanded_acceleration`, `TrackingGains`, `AttitudeController::Feedforward`), Task 4 (`DescentLimits::phase0_*`).
- Produces:
  - `struct ScvxLandingParameters { double replan_interval_s = 1.0; double plan_latency_s = 0.5; double min_replan_time_to_go_s = 3.0; int failures_before_abort = 2; bool realtime = false; SnapshotSettings snapshot; ScvxSettings settings; TrackingGains tracking; };`
  - `struct ScvxLandingCommand { std::vector<std::string> run_groups; double throttle; math::Vec3 axis_gcrf, roll_reference_gcrf; AttitudeController::Feedforward feedforward; bool abort; double time_to_go_s; double offset_m; };`
  - `class ScvxLandingGuidance { ScvxLandingGuidance(LandingSite, ScvxLandingParameters, PlanFunction = default_plan_function()); bool start(const vehicle::FlightBody&, const vehicle::FlightEnvironment&, time::Epoch); ScvxLandingCommand update(...same...); bool aborting() const; void set_realtime(bool); struct Statistics { int plans; int failures; double last_solve_ms; double max_solve_ms; }; const Statistics& statistics() const; };`

- [ ] **Step 1: Write the failing tests**

`core/tests/test_scvx_landing.cpp`:

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/gnc/scvx_landing.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

struct Fixture {
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    time::Epoch t0;
    std::unique_ptr<sim::FlightSession> session;
    std::unique_ptr<vehicle::FlightBody> body;

    Fixture() {
        test::spice_session();
        t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
        session = std::make_unique<sim::FlightSession>(ephemeris, earth, STARSIM_CONFIG_DIR);
        const auto site = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1"));
        session->set_launch_site(site);
        body = std::make_unique<vehicle::FlightBody>(
            std::make_unique<vehicle::Vehicle>(test::booster_config("starship_block3", 70000.0), STARSIM_CONFIG_DIR),
            test::state_over_site(site, earth, t0, {30.0, -15.0, 1300.0}, {4.0, -2.0, -250.0}));
    }

    gnc::ScvxLandingParameters parameters() const {
        gnc::ScvxLandingParameters p;
        p.snapshot.ignition_groups = {"booster/center", "booster/middle"};
        p.snapshot.final_groups = {"booster/center"};
        return p;
    }
};

}  // namespace

TEST_CASE("SCvx landing guidance starts with a plan and commands the ignition engines", "[gnc][scvx_landing]") {
    Fixture f;
    gnc::ScvxLandingGuidance guidance(f.session->return_site(f.body->vehicle()), f.parameters());
    REQUIRE(guidance.start(*f.body, f.session->environment(), f.t0));
    const auto cmd = guidance.update(*f.body, f.session->environment(), f.t0);
    CHECK(cmd.run_groups == std::vector<std::string>{"booster/center", "booster/middle"});
    CHECK(cmd.throttle > 0.0);
    CHECK(cmd.throttle <= 1.0);
    CHECK_FALSE(cmd.abort);
    CHECK(cmd.time_to_go_s > 5.0);
    CHECK(guidance.statistics().plans == 1);
    std::printf("first plan: %.0f ms\n", guidance.statistics().last_solve_ms);
}

TEST_CASE("A re-plan from a snapshot takes effect half a second later", "[gnc][scvx_landing]") {
    Fixture f;
    gnc::ScvxLandingGuidance guidance(f.session->return_site(f.body->vehicle()), f.parameters());
    REQUIRE(guidance.start(*f.body, f.session->environment(), f.t0));
    const auto& env = f.session->environment();
    guidance.update(*f.body, env, f.t0 + 1.0);  // snapshot (the body is not moved: a static re-plan)
    guidance.update(*f.body, env, f.t0 + 1.4);
    CHECK(guidance.statistics().plans == 1);
    guidance.update(*f.body, env, f.t0 + 1.5);
    CHECK(guidance.statistics().plans == 2);
}

TEST_CASE("Repeated failed re-plans switch to an abort", "[gnc][scvx_landing]") {
    Fixture f;
    int calls = 0;
    gnc::PlanFunction failing = [&calls](gnc::DescentPlanner& planner, const gnc::PlanRequest& r) {
        auto plan = gnc::default_plan_function()(planner, r);
        if (calls++ > 0 && r.target.goal == gnc::DescentGoal::Catch) plan.converged = false;  // re-plans fail
        return plan;
    };
    gnc::ScvxLandingGuidance guidance(f.session->return_site(f.body->vehicle()), f.parameters(), failing);
    REQUIRE(guidance.start(*f.body, f.session->environment(), f.t0));
    const auto& env = f.session->environment();
    for (double t = 1.0; t <= 3.5 && !guidance.aborting(); t += 0.5) guidance.update(*f.body, env, f.t0 + t);
    CHECK(guidance.aborting());
    CHECK(guidance.statistics().failures == 2);
    CHECK(guidance.update(*f.body, env, f.t0 + 4.0).abort);
}
```

Add `test_scvx_landing.cpp` to `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Run to verify they fail**

Run: build. Expected: compile error (`scvx_landing.hpp` missing).

- [ ] **Step 3: Implement**

`core/include/starsim/gnc/scvx_landing.hpp`:

```cpp
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/gnc/descent_replanner.hpp"
#include "starsim/gnc/descent_snapshot.hpp"
#include "starsim/gnc/descent_tracking.hpp"

namespace starsim::gnc {

struct ScvxLandingParameters {
    double replan_interval_s = 1.0;        // spec
    double plan_latency_s = 0.5;           // spec: a plan takes effect at snapshot + this
    double min_replan_time_to_go_s = 3.0;  // [E] no catch re-plans closer to the end (phase durations degenerate)
    int failures_before_abort = 2;         // [E] consecutive failed re-plans before the catch is given up
    bool realtime = false;                 // never wait for a re-plan (interactive play)
    SnapshotSettings snapshot;
    ScvxSettings settings;
    TrackingGains tracking;
};

struct ScvxLandingCommand {
    std::vector<std::string> run_groups;
    double throttle = 1.0;
    math::Vec3 axis_gcrf;
    math::Vec3 roll_reference_gcrf;               // zero: free roll
    AttitudeController::Feedforward feedforward;  // body axes, relative to inertial space
    bool abort = false;
    double time_to_go_s = 0.0;
    double offset_m = 0.0;  // horizontal distance of the CG from the plan's end point
};

// Landing burn on SCvx plans: the first plan synchronously at ignition, re-plans every replan_interval_s from a
// snapshot (worker thread, effective at snapshot + latency), tracking of the active plan (thrust acceleration with a
// position/velocity correction, attitude with feed-forward, pins rolled across the arms), engine switch at the
// planned time. failures_before_abort consecutive failed re-plans give up the catch: from then on the plans go to
// the sea (keep-out around the tower) and the command says abort (the tower keeps its arms open).
class ScvxLandingGuidance {
public:
    ScvxLandingGuidance(LandingSite site, ScvxLandingParameters parameters, PlanFunction plan = default_plan_function());

    // First plan; false if it failed (the caller falls back to other guidance).
    bool start(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);
    ScvxLandingCommand update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);

    bool aborting() const { return aborting_; }
    void set_realtime(bool realtime) { parameters_.realtime = realtime; }

    struct Statistics {
        int plans = 0;       // collected plans incl. the first
        int failures = 0;    // of these not converged
        double last_solve_ms = 0.0;
        double max_solve_ms = 0.0;
    };
    const Statistics& statistics() const { return stats_; }

private:
    PlanRequest request(const DescentSnapshot& snapshot, int start_phase) const;
    void accept(const PlanOutcome& outcome);

    LandingSite site_;
    ScvxLandingParameters parameters_;
    PlanFrame frame_;
    DescentReplanner replanner_;
    std::shared_ptr<const DescentPlan> plan_;
    time::Epoch plan_epoch_;
    DescentModel model_;  // of the active plan's snapshot (gravity after the plan's end)
    time::Epoch next_request_;
    bool final_engines_ = false;
    bool aborting_ = false;
    int consecutive_failures_ = 0;
    Statistics stats_;
};

}  // namespace starsim::gnc
```

`core/src/gnc/scvx_landing.cpp`:

```cpp
#include "starsim/gnc/scvx_landing.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/frames/geodetic.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double min_spool_for_throttle = 0.05;  // engines just starting: throttle as if this far spooled

bool contains(const std::vector<std::string>& groups, const std::string& group) {
    return std::find(groups.begin(), groups.end(), group) != groups.end();
}

// Thrust the running (or starting) engines of `groups` give at full throttle now.
double available_thrust_n(const vehicle::Vehicle& v, const std::vector<std::string>& groups, double pressure_pa) {
    double sum = 0.0;
    for (const auto& e : v.stages().front().engines) {
        if (e.starved || !contains(groups, e.group_id)) continue;
        sum += std::max(e.spool, min_spool_for_throttle) * e.performance->thrust_n(1.0, pressure_pa);
    }
    return sum;
}

}  // namespace

ScvxLandingGuidance::ScvxLandingGuidance(LandingSite site, ScvxLandingParameters parameters, PlanFunction plan)
    : site_(std::move(site)),
      parameters_(std::move(parameters)),
      frame_(plan_frame(site_.latitude_rad, site_.longitude_rad, site_.height_m)),
      replanner_(parameters_.plan_latency_s, std::move(plan)) {}

PlanRequest ScvxLandingGuidance::request(const DescentSnapshot& snapshot, int start_phase) const {
    PlanRequest r;
    r.model = snapshot.model;
    r.initial = snapshot.initial;
    r.start_phase = start_phase;
    r.target = aborting_ ? snapshot.abort_target : snapshot.catch_target;
    r.settings = parameters_.settings;
    r.settings.limits.min_mass_kg = snapshot.min_mass_kg;
    r.settings.limits.phase0_initial_spool = snapshot.phase0_spool;
    r.settings.limits.phase0_startup_s = snapshot.startup_s;
    if (plan_ && plan_->goal == r.target.goal) {
        r.warm_start = plan_;
        r.warm_start_elapsed_s = snapshot.epoch - plan_epoch_;
    }
    r.snapshot_epoch = snapshot.epoch;
    return r;
}

void ScvxLandingGuidance::accept(const PlanOutcome& outcome) {
    ++stats_.plans;
    stats_.last_solve_ms = outcome.solve_ms;
    stats_.max_solve_ms = std::max(stats_.max_solve_ms, outcome.solve_ms);
    const bool stale = aborting_ && outcome.plan->goal == DescentGoal::Catch;  // decided before the abort
    if (outcome.plan->converged && !stale) {
        plan_ = outcome.plan;
        plan_epoch_ = outcome.snapshot_epoch;
        consecutive_failures_ = 0;
        return;
    }
    if (outcome.plan->converged) return;
    ++stats_.failures;
    if (++consecutive_failures_ >= parameters_.failures_before_abort) aborting_ = true;
}

bool ScvxLandingGuidance::start(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch) {
    const auto snapshot = take_descent_snapshot(body, env, epoch, site_, frame_, parameters_.snapshot);
    model_ = snapshot.model;
    const auto outcome = replanner_.plan_now(request(snapshot, 0));
    ++stats_.plans;
    stats_.last_solve_ms = stats_.max_solve_ms = outcome.solve_ms;
    if (!outcome.plan->converged) {
        ++stats_.failures;
        return false;
    }
    plan_ = outcome.plan;
    plan_epoch_ = epoch;
    next_request_ = epoch + parameters_.replan_interval_s;
    return true;
}

ScvxLandingCommand ScvxLandingGuidance::update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                               time::Epoch epoch) {
    if (auto outcome = replanner_.collect(epoch, !parameters_.realtime)) accept(*outcome);

    const double t = epoch - plan_epoch_;
    const double time_to_go = plan_->duration_s() - t;
    const bool abort_pending = aborting_ && plan_->goal == DescentGoal::Catch;  // no abort plan yet
    if (epoch - next_request_ >= -1e-9 && !replanner_.busy() &&
        (time_to_go > parameters_.min_replan_time_to_go_s || abort_pending)) {
        const auto snapshot = take_descent_snapshot(body, env, epoch, site_, frame_, parameters_.snapshot);
        model_ = snapshot.model;
        const int phase = final_engines_ ? 1 : plan_->phase_at(t);
        replanner_.submit(request(snapshot, phase));
        next_request_ = epoch + parameters_.replan_interval_s;
    }

    const auto earth = env.earth_orientation.fixed_to_gcrf(epoch);
    const auto x = plan_state(body, earth, frame_);
    const auto ref = sample_plan(*plan_, model_, epoch - plan_epoch_, parameters_.snapshot.catch_sink_mps);
    if (ref.phase == 1) final_engines_ = true;
    const Vec3 a = commanded_acceleration(ref, state_position(x), state_velocity(x), parameters_.tracking);
    const Vec3 axis_plan = norm(a) > 0.0 ? a / norm(a) : Vec3{1.0, 0.0, 0.0};

    ScvxLandingCommand cmd;
    cmd.run_groups = final_engines_ ? parameters_.snapshot.final_groups : parameters_.snapshot.ignition_groups;
    const Vec3 r_fixed = transpose(earth.rotation) * body.state().position_gcrf_m;
    const double pressure = env.atmosphere.at(frames::fixed_to_geodetic(r_fixed).height_m).pressure_pa;
    const double available = available_thrust_n(body.vehicle(), cmd.run_groups, pressure);
    cmd.throttle = available > 0.0 ? std::clamp(x[descent_index::mass] * norm(a) / available, 0.0, 1.0) : 1.0;
    cmd.axis_gcrf = earth.rotation * frame_.direction_to_itrf(axis_plan);
    const Vec3 omega_body = body.state().attitude.inverse_rotate(vehicle::earth_rotation_vector(earth));
    cmd.feedforward.rate_body_radps = ref.rate_body_radps + omega_body;
    cmd.feedforward.acceleration_body_radps2 = ref.angular_acceleration_body_radps2;
    if (site_.catch_target && !aborting_) {
        const Vec3 up_itrf = frame_.direction_to_itrf({1.0, 0.0, 0.0});
        const Vec3 body_y = transpose(earth.rotation) * body.state().attitude.rotate({0.0, 1.0, 0.0});
        cmd.roll_reference_gcrf = earth.rotation * catch_roll_reference(site_.catch_target->across_itrf, up_itrf,
                                                                        site_.catch_target->pin_azimuth_rad, body_y);
    }
    cmd.abort = aborting_;
    cmd.time_to_go_s = std::max(time_to_go, 0.0);
    const Vec3 end = state_position(plan_->reference.x.back());
    cmd.offset_m = std::hypot(state_position(x).y - end.y, state_position(x).z - end.z);
    return cmd;
}

}  // namespace starsim::gnc
```

Add `src/gnc/scvx_landing.cpp` to `core/CMakeLists.txt`.

- [ ] **Step 4: Run the tests**

Run: build, `build/core/tests/starsim_core_tests.exe "[scvx_landing]"`
Expected: all 3 pass; the first plan prints its time (expect 100-400 ms in Release).

- [ ] **Step 5: Full build and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` - Expected: all pass.

```bash
git add core/include/starsim/gnc/scvx_landing.hpp core/src/gnc/scvx_landing.cpp core/tests/test_scvx_landing.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat: SCvx landing guidance - first plan, timed re-plans, abort decision, tracking commands

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE"
```

---

### Task 9: Hand-over in the return guidance, flight-session wiring and the catch scenario

**Files:**
- Modify: `core/include/starsim/gnc/booster_return.hpp`, `core/src/gnc/booster_return.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`
- Modify: `core/tests/return_scenario.hpp`
- Create: `core/tests/test_scvx_catch.cpp`
- Modify: `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 8 (`ScvxLandingGuidance`, `ScvxLandingParameters`, `ScvxLandingCommand`), Task 2 (`abort_catch`, `tower_struck`, `landing_outcome`), Task 5 (`return_site`, `environment`), Task 7 (`Feedforward`).
- Produces:
  - `BoosterReturnParameters::scvx_landing` (bool, true), `BoosterReturnParameters::scvx` (`ScvxLandingParameters`)
  - `ReturnCommand::{abort (bool), scvx (bool), feedforward (std::optional<AttitudeController::Feedforward>)}`
  - `void BoosterReturnGuidance::skip_to(ReturnPhase)`, `void BoosterReturnGuidance::set_realtime(bool)`, `const ScvxLandingGuidance* BoosterReturnGuidance::scvx() const`
  - `void FlightSession::place_returning_booster(const config::VehicleConfig&, const vehicle::RigidBodyState&, time::Epoch, gnc::BoosterReturnParameters, gnc::ReturnPhase start_phase)`, `void FlightSession::set_realtime_planning(bool)`, `const gnc::BoosterReturnGuidance* FlightSession::return_guidance(size_t index) const`
  - test helpers: `double test::ignition_height_m(const sim::FlightSession&, const config::VehicleConfig&, const frames::BodyOrientation&, time::Epoch, double sink_mps)`, `struct test::CatchRun { ... }`, `CatchRun test::fly_landing(sim::FlightSession&, const frames::BodyOrientation&, time::Epoch t0, double max_s)`

- [ ] **Step 1: Scenario helpers and the failing scenario test**

Append to `core/tests/return_scenario.hpp` (inside the namespace; add `#include <algorithm>`, `#include <memory>`, `#include "starsim/gnc/booster_return.hpp"`, `#include "starsim/sim/landing_outcome.hpp"`):

```cpp
// CG height above the pad at which the return guidance's own burn prediction ignites for a booster falling straight
// down at `sink_mps` (stop height 0), by bisection.
inline double ignition_height_m(const sim::FlightSession& session, const config::VehicleConfig& cfg,
                                const frames::BodyOrientation& earth, time::Epoch epoch, double sink_mps) {
    double low = 200.0, high = 8000.0;  // [m] bracket
    for (int i = 0; i < 40; ++i) {
        const double h = 0.5 * (low + high);
        const vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR),
                                       state_over_site(session.launch_site(), earth, epoch, {0.0, 0.0, h}, {0.0, 0.0, -sink_mps}));
        const gnc::BoosterReturnGuidance guidance(session.return_site(body.vehicle()));
        (guidance.predict_landing_burn(body, session.environment(), epoch).stop_height_m > 0.0 ? high : low) = h;
    }
    return 0.5 * (low + high);
}

// End of a landing: first pin contact (offset of the CG from the catch axis, horizontal and vertical speed, site
// frame) and how it ended.
struct CatchRun {
    bool contact = false;
    double offset_m = 0.0;
    double lateral_mps = 0.0;
    double sink_mps = 0.0;
    sim::CatchPhase tower_phase = sim::CatchPhase::Open;
    bool tower_struck = false;
    sim::LandingOutcome outcome = sim::LandingOutcome::Flying;
    bool caught_within_tolerance() const {
        return tower_phase == sim::CatchPhase::Caught && contact && offset_m < 1.0 && lateral_mps < 0.5;
    }
};

// Advances body 0 tick by tick until it is caught, rests, impacts or `max_s` pass.
inline CatchRun fly_landing(sim::FlightSession& session, const frames::BodyOrientation& earth, time::Epoch t0, double max_s) {
    CatchRun run;
    const auto& booster = session.body(0);
    const auto* tower = session.tower();
    for (double t = sim::FlightSession::tick_s; t <= max_s; t += sim::FlightSession::tick_s) {
        session.advance_to(t0 + t, 10);
        if (!run.contact && tower && tower->in_contact(booster)) {
            const auto tr = earth.fixed_to_gcrf(session.epoch());
            const auto& s = booster.state();
            const auto& g = tower->geometry();
            const math::Vec3 p = g.to_site(transpose(tr.rotation) * s.position_gcrf_m);
            const math::Vec3 v = transpose(g.site_to_itrf()) * transpose(tr.rotation) *
                                 (s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), s.position_gcrf_m));
            run.contact = true;
            run.offset_m = std::hypot(p.x, p.y);
            run.lateral_mps = std::hypot(v.x, v.y);
            run.sink_mps = -v.z;
        }
        if (booster.caught() || booster.impacted() || booster.resting()) break;
    }
    if (tower) {
        run.tower_phase = tower->phase();
        run.tower_struck = tower->tower_struck();
    }
    run.outcome = session.landing_outcome(0);
    return run;
}
```

`core/tests/test_scvx_catch.cpp`:

```cpp
// The booster placed at landing-burn ignition over Starbase flies the burn on SCvx plans and is caught.

#include <catch2/catch_test_macros.hpp>

#include <cstdio>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double nominal_propellant_kg = 70.0e3;  // below the FAA EA's ~74 t at ignition (Task 1)
constexpr double nominal_sink_mps = 260.0;        // [E] tail-first fall speed at ignition (recorded returns: 250-280)

const spice::SpiceEphemeris& ephemeris() {
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& earth() {
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

time::Epoch start() {
    test::spice_session();
    return spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
}

test::CatchRun fly(const Vec3& offset_enu_m, const Vec3& velocity_offset_mps) {
    const auto t0 = start();
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, nominal_sink_mps);
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, offset_enu_m + Vec3{0.0, 0.0, h},
                                                          velocity_offset_mps + Vec3{0.0, 0.0, -nominal_sink_mps}),
                                    t0, {}, gnc::ReturnPhase::LandingBurn);
    const auto run = test::fly_landing(session, earth(), t0, 90.0);
    if (const auto* g = session.return_guidance(0); g && g->scvx()) {
        const auto& s = g->scvx()->statistics();
        std::printf("ignition at %.0f m: %s, offset %.2f m, lateral %.2f m/s, sink %.2f m/s, %d plans (%d failed), "
                    "max %.0f ms\n", h, sim::to_string(run.tower_phase), run.offset_m, run.lateral_mps, run.sink_mps,
                    s.plans, s.failures, s.max_solve_ms);
    }
    return run;
}

}  // namespace

TEST_CASE("SCvx flies a nominal landing burn onto the arms", "[gnc][catch]") {
    const auto run = fly({20.0, -10.0, 0.0}, {});
    CHECK(run.caught_within_tolerance());
    CHECK(run.sink_mps < 2.0);
    CHECK_FALSE(run.tower_struck);
    CHECK(run.outcome == sim::LandingOutcome::Caught);
}

TEST_CASE("SCvx landings are deterministic", "[gnc][catch]") {
    const auto a = fly({20.0, -10.0, 0.0}, {3.0, 0.0, 0.0});
    const auto b = fly({20.0, -10.0, 0.0}, {3.0, 0.0, 0.0});
    CHECK(a.offset_m == b.offset_m);
    CHECK(a.lateral_mps == b.lateral_mps);
    CHECK(a.sink_mps == b.sink_mps);
}
```

Add `test_scvx_catch.cpp` to `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Run to verify it fails**

Run: build. Expected: compile errors (`place_returning_booster`, `return_guidance`, `scvx()` unknown).

- [ ] **Step 3: Return guidance hand-over**

`booster_return.hpp`: `#include <memory>`, `#include "starsim/gnc/attitude_controller.hpp"`, `#include "starsim/gnc/scvx_landing.hpp"`. Move `enum class ReturnPhase` and `to_string(ReturnPhase)` above `BoosterReturnParameters` (no change otherwise). In `BoosterReturnParameters` append:

```cpp
    // Landing burn on SCvx plans (catch sites only); the explicit laws above remain the fallback when the first
    // plan fails, and the guidance of sites without a tower.
    bool scvx_landing = true;
    ScvxLandingParameters scvx;
```

In `ReturnCommand` append:

```cpp
    bool scvx = false;                                          // landing burn flown on SCvx plans
    bool abort = false;                                         // catch given up: arms stay open, landing at sea
    std::optional<AttitudeController::Feedforward> feedforward; // planned rotation (SCvx)
```

In `BoosterReturnGuidance` public:

```cpp
    // Scenarios: start in a later phase (e.g. LandingBurn for a booster placed at ignition).
    void skip_to(ReturnPhase phase) { phase_ = phase; started_ = true; }
    void set_realtime(bool realtime);
    const ScvxLandingGuidance* scvx() const { return scvx_.get(); }
```

private: `std::unique_ptr<ScvxLandingGuidance> scvx_; bool landing_started_ = false;` and the private methods

```cpp
    void start_landing(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);
    void explicit_landing(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch,
                          ReturnCommand& cmd);
```

`booster_return.cpp`: Move the whole body of `if (phase_ == ReturnPhase::LandingBurn) { ... }` (from `// Gravity + centrifugal acceleration ...` to the `phase_ = ReturnPhase::Landed; }` of the touchdown check) verbatim into `explicit_landing`, preceded by the locals it uses, recomputed:

```cpp
void BoosterReturnGuidance::explicit_landing(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                             time::Epoch epoch, ReturnCommand& cmd) {
    const auto& s = body.state();
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const Mat3 to_fixed = transpose(transform.rotation);
    const Vec3 omega = vehicle::earth_rotation_vector(transform);
    const Vec3 r_fixed = to_fixed * s.position_gcrf_m;
    const Vec3 v_fixed = to_fixed * (s.velocity_gcrf_mps - cross(omega, s.position_gcrf_m));
    const auto mass = body.vehicle().mass_properties();
    const Vec3 site_up = up_at(site_.latitude_rad, site_.longitude_rad);
    const Vec3 site_fixed = frames::geodetic_to_fixed({site_.latitude_rad, site_.longitude_rad, site_.height_m});
    const double pressure = env.atmosphere.at(frames::fixed_to_geodetic(r_fixed).height_m).pressure_pa;
    // ... moved block, unchanged ...
}
```

(remove from the moved block any local the compiler reports as unused; keep behaviour identical.) Then

```cpp
void BoosterReturnGuidance::set_realtime(bool realtime) {
    parameters_.scvx.realtime = realtime;
    if (scvx_) scvx_->set_realtime(realtime);
}

void BoosterReturnGuidance::start_landing(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                          time::Epoch epoch) {
    landing_started_ = true;
    if (!parameters_.scvx_landing || !site_.catch_target) return;
    auto p = parameters_.scvx;
    p.snapshot.ignition_groups = parameters_.landing_ignition_groups;
    p.snapshot.final_groups = parameters_.landing_groups;
    scvx_ = std::make_unique<ScvxLandingGuidance>(site_, p);
    if (!scvx_->start(body, env, epoch)) scvx_.reset();  // fallback: the explicit laws
}
```

and the `LandingBurn` branch of `update` becomes

```cpp
    if (phase_ == ReturnPhase::LandingBurn) {
        if (!landing_started_) start_landing(body, env, epoch);
        if (scvx_) {
            const auto c = scvx_->update(body, env, epoch);
            cmd.run_groups = c.run_groups;
            cmd.throttle = c.throttle;
            cmd.axis_gcrf = c.axis_gcrf;
            cmd.roll_reference_gcrf = c.roll_reference_gcrf;
            cmd.feedforward = c.feedforward;
            cmd.scvx = true;
            cmd.abort = c.abort;
            cmd.time_to_go_s = c.time_to_go_s;
            cmd.impact_miss_m = c.offset_m;
            const Vec3 r_catch = site_.catch_target->pin_point_itrf_m -
                                 site_up * (site_.catch_target->pin_station_m - mass.cg_station_m);
            const double height = dot(r_fixed - r_catch, site_up);
            const bool catch_over = !c.abort && (catch_contact_ || height < -catch_missed_height_m);
            if (body.resting() || body.impacted() || catch_over) phase_ = ReturnPhase::Landed;
        } else {
            explicit_landing(body, env, epoch, cmd);
        }
    }
```

(`r_fixed`, `site_up`, `mass` are the locals at the top of `update`.)

- [ ] **Step 4: Flight session**

`flight_session.hpp` public:

```cpp
    // Scenarios: a returning booster placed with a free state, guided by its return guidance from `start_phase`
    // (it becomes the controlled body; pilot input is ignored while it has return guidance).
    void place_returning_booster(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                                 time::Epoch epoch, gnc::BoosterReturnParameters parameters, gnc::ReturnPhase start_phase);
    // Interactive real-time play: landing re-plans never hold up the simulation (default: off = deterministic).
    void set_realtime_planning(bool realtime);
    const gnc::BoosterReturnGuidance* return_guidance(size_t index) const { return bodies_.at(index).return_guidance.get(); }
```

private member `bool realtime_planning_ = false;`.

`flight_session.cpp`:

```cpp
void FlightSession::place_returning_booster(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                                            time::Epoch epoch, gnc::BoosterReturnParameters parameters,
                                            gnc::ReturnPhase start_phase) {
    place_body(config, state, epoch);
    auto& flying = bodies_.front();
    flying.return_guidance = std::make_unique<gnc::BoosterReturnGuidance>(return_site(flying.body->vehicle()), std::move(parameters));
    flying.return_guidance->skip_to(start_phase);
    flying.return_guidance->set_realtime(realtime_planning_);
}

void FlightSession::set_realtime_planning(bool realtime) {
    realtime_planning_ = realtime;
    for (auto& flying : bodies_) {
        if (flying.return_guidance) flying.return_guidance->set_realtime(realtime);
    }
}
```

In `stage()`, after creating `lower.return_guidance`: `lower.return_guidance->set_realtime(realtime_planning_);`.
In `run_return_guidance`, after the engine-group loop: `if (tower_ && flying.return_command.abort) tower_->abort_catch();`.
In `advance_to`, the loop condition `if (i != controlled_ && bodies_[i].return_guidance)` becomes `if (bodies_[i].return_guidance)`.
In `command_for`, `if (index != controlled_) {` becomes `if (index != controlled_ || flying.return_guidance) {` and the steer call passes the feed-forward:

```cpp
            return_controller_.steer(*flying.body, ambient_pressure_pa, r.axis_gcrf, r.roll_reference_gcrf,
                                     r.feedforward ? &*r.feedforward : nullptr).apply_to(command);
```

- [ ] **Step 5: Run the scenario**

Run: build, `$env:STARSIM_VERBOSE="1"; build/core/tests/starsim_core_tests.exe "[gnc][catch]" -s` (PowerShell) or `STARSIM_VERBOSE=1 build/core/tests/starsim_core_tests.exe "[gnc][catch]"` (bash)
Expected: both pass; the print shows "gefangen", offset < 1 m, lateral < 0.5 m/s, plans ~15, max plan time printed.
If not caught, debug in this order (systematic debugging, one change at a time, each ledgered): (1) print per tick around the first contact the plan reference vs actual position (offset) and the time since the last accepted plan; (2) check the engine start-up: throttle at 1.0 during the first 3 s and the first plan's thrust bounded by the spool (Task 4); (3) the 13 -> 3 switch: compare `ref.phase` switching time with the engines' spool-down; (4) tracking gains (`TrackingGains::frequency_rad_s` 0.4-1.0, `max_correction_mps2`); (5) the attitude loop (`return_controller_` gains 1.5/0.9 and feed-forward signs: the rate test of Task 7 must still pass); (6) `min_replan_time_to_go_s` (2-4 s). Tolerances in the test stay as written.

- [ ] **Step 6: Full build and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` - Expected: all pass (including the explicit-guidance return tests: the catch-return validation now flies SCvx).

```bash
git add core/include/starsim/gnc/booster_return.hpp core/src/gnc/booster_return.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/tests/return_scenario.hpp core/tests/test_scvx_catch.cpp core/tests/CMakeLists.txt
git commit -m "feat: returning booster flies its landing burn on SCvx plans with explicit fallback

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE"
```

---

### Task 10: Dispersion campaign with an engine failure

**Files:**
- Modify: `core/tests/test_scvx_catch.cpp`

**Interfaces:**
- Consumes: Task 9's helpers (`fly_landing`, `CatchRun`, `ignition_height_m`, `place_returning_booster`), `FlightSession::fail_engine(body, stage, engine)`.
- Produces: the campaign test `[gnc][catch][validation]`.

- [ ] **Step 1: Write the campaign**

Append to `core/tests/test_scvx_catch.cpp` (add `#include <random>`, `#include <cstdlib>`, `#include "starsim/math/constants.hpp"`):

```cpp
namespace {

struct Dispersion {
    Vec3 offset_m;         // lateral, site frame
    Vec3 velocity_mps;     // added to the nominal fall
    double mass_factor = 1.0;  // residual propellant
    math::Quat tilt;       // attitude error
    bool engine_failure = false;
};

// Fixed seeds: lateral +-50 m, velocity +-10 m/s per axis, residual mass +-5 %, attitude +-3 deg about a random
// horizontal axis (spec).
Dispersion dispersion(unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    Dispersion d;
    d.offset_m = {50.0 * unit(rng), 50.0 * unit(rng), 0.0};
    d.velocity_mps = {10.0 * unit(rng), 10.0 * unit(rng), 10.0 * unit(rng)};
    d.mass_factor = 1.0 + 0.05 * unit(rng);
    const double azimuth = constants::pi * unit(rng);
    d.tilt = math::Quat::from_axis_angle({std::cos(azimuth), std::sin(azimuth), 0.0}, 3.0 * constants::deg_to_rad * unit(rng));
    return d;
}

test::CatchRun fly_dispersed(const Dispersion& d) {
    const auto t0 = start();
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    const auto nominal = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, nominal, earth(), t0, nominal_sink_mps);  // nominal ignition point
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg * d.mass_factor);
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, d.offset_m + Vec3{0.0, 0.0, h},
                                                          d.velocity_mps + Vec3{0.0, 0.0, -nominal_sink_mps}, d.tilt),
                                    t0, {}, gnc::ReturnPhase::LandingBurn);
    if (d.engine_failure) {
        // One middle-ring engine fails 1 s into the 13-engine phase.
        session.advance_to(t0 + 1.0, 1000);
        const auto& engines = session.body(0).vehicle().stages().front().engines;
        for (size_t i = 0; i < engines.size(); ++i) {
            if (engines[i].group_id == "booster/middle") {
                session.fail_engine(0, 0, i);
                break;
            }
        }
    }
    return test::fly_landing(session, earth(), t0, 90.0);
}

}  // namespace

TEST_CASE("SCvx catch campaign: dispersions and an engine failure", "[gnc][catch][validation]") {
    std::vector<Dispersion> cases;
    for (unsigned seed = 1; seed <= 20; ++seed) cases.push_back(dispersion(seed));
    Dispersion failure;
    failure.engine_failure = true;
    cases.push_back(failure);

    int caught = 0;
    for (size_t i = 0; i < cases.size(); ++i) {
        const auto run = fly_dispersed(cases[i]);
        CAPTURE(i);
        std::printf("case %2zu: %s / %s, offset %.2f m, lateral %.2f m/s, sink %.2f m/s\n", i,
                    sim::to_string(run.tower_phase), sim::to_string(run.outcome), run.offset_m, run.lateral_mps, run.sink_mps);
        CHECK_FALSE(run.tower_struck);  // never, in any case
        if (run.caught_within_tolerance()) {
            ++caught;
        } else {
            // Not caught: it must have ended as a splashdown or a soft touchdown without tower damage.
            CHECK((run.outcome == sim::LandingOutcome::Splashdown || run.outcome == sim::LandingOutcome::Landed));
        }
    }
    std::printf("caught %d of %zu\n", caught, cases.size());
    CHECK(caught >= 19);  // >= 90 % of 21
}
```

(`std::vector` needs `#include <vector>`.)

- [ ] **Step 2: Run the campaign**

Run: build, `build/core/tests/starsim_core_tests.exe "SCvx catch campaign: dispersions and an engine failure"`
Expected: PASS, >= 19 of 21 caught, no tower strike. The 20 seeded cases are fixed: a failing run reproduces exactly.
If it fails: list the failing cases from the print; debug one at a time (start with the worst) with the order of Task 9 Step 5, plus: (a) aborts that end on land (`gelandet`) instead of `Wasserung` are allowed by the spec ("splashdown or a soft touchdown"); an abort that ends as `Aufprall` on land is a failure - check the abort plan's terminal velocity and the touchdown rest criterion; (b) an engine-failure miss: check that the next snapshot drops the starved engine (`make_descent_model` skips `starved`) and that the re-plan converges with 12 engines. Do not loosen the 19-of-21 threshold or the dispersion ranges; parameter changes (gains, latency-independent margins) are ledgered rulings.

- [ ] **Step 3: Full build and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` - Expected: all pass.

```bash
git add core/tests/test_scvx_catch.cpp
git commit -m "test: SCvx catch campaign with 20 seeded dispersions and an engine failure

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE"
```

(If Step 2 needed code changes, commit them with `fix:` before this commit, listing the cause in the message.)

---

### Task 11: Full returns of blocks 1-3 end on the arms

**Files:**
- Modify: `core/tests/test_booster_return.cpp:147-185`

**Interfaces:**
- Consumes: Task 9 (SCvx is the default landing guidance), Task 2 (`tower_struck`), Task 1 (FAA check stays).
- Produces: the strengthened integration test "Booster return to the catch tower ends on the arms".

- [ ] **Step 1: Tighten the test**

Rename the test case to "Booster return to the catch tower ends on the arms" (tags unchanged). In its loop, record the first pin contact like `test::fly_landing` does (include `return_scenario.hpp`): add before the loop

```cpp
    bool contact = false;
    double contact_offset_m = 0.0, contact_lateral_mps = 0.0;
```

and inside the loop, after `const auto& booster = session.body(0);`:

```cpp
        if (!contact && session.tower()->in_contact(booster)) {
            const auto tr = earth.fixed_to_gcrf(session.epoch());
            const auto& s = booster.state();
            const auto& g = session.tower()->geometry();
            const math::Vec3 p = g.to_site(transpose(tr.rotation) * s.position_gcrf_m);
            const math::Vec3 v = transpose(g.site_to_itrf()) * transpose(tr.rotation) *
                                 (s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), s.position_gcrf_m));
            contact = true;
            contact_offset_m = std::hypot(p.x, p.y);
            contact_lateral_mps = std::hypot(v.x, v.y);
        }
```

Replace `CHECK((phase == sim::CatchPhase::Caught || phase == sim::CatchPhase::Missed));` by

```cpp
    const auto* status = session.return_status(0);
    REQUIRE(status != nullptr);
    CHECK(status->scvx);  // flown on SCvx plans, no fallback
    CHECK(phase == sim::CatchPhase::Caught);
    CHECK(contact);
    CHECK(contact_offset_m < 1.0);
    CHECK(contact_lateral_mps < 0.5);
    CHECK_FALSE(tower->tower_struck());
```

and extend the print with `contact_offset_m` and `contact_lateral_mps`. Update the file's header comment (first two lines) to: "Return to launch site: after hot staging the booster flips, boosts back, coasts engines first and is caught on the tower arms after an SCvx-guided landing burn (or lands on the pad without a tower)."

- [ ] **Step 2: Run it**

Run: build, `build/core/tests/starsim_core_tests.exe "Booster return to the catch tower ends on the arms"`
Expected: PASS for blocks 1, 2, 3 (caught, < 1 m, < 0.5 m/s); block 3 ignition propellant still <= 74 t.
If a block fails: check first whether the SCvx plan exists from ignition (`status->scvx`; a failed first plan means the ignition state is outside what the planner accepts - print the snapshot state and `plan.virtual_control`), then whether the block has enough propellant at ignition (block 1 lands with ~0.6 t: the 5 t `reserve_kg` may make its plan infeasible - a block-specific `ScvxLandingParameters::snapshot.reserve_kg` is not available, so lower the default only with a ruling that states the reserve physically required). Debug per Task 9 Step 5 otherwise; tolerances stay.

- [ ] **Step 3: Full build and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` - Expected: all pass.

```bash
git add core/tests/test_booster_return.cpp
git commit -m "test: full booster returns of blocks 1-3 are caught on the tower arms

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE"
```

---

### Task 12: Game wiring, HUD and documentation

**Files:**
- Modify: `bindings/src/flight_sim.hpp`, `bindings/src/flight_sim.cpp`
- Modify: `game/ui/flight_hud.gd`, `game/vehicle/flight_controller.gd`, `game/world/world.gd`
- Modify: `CLAUDE.md`, `docs/superpowers/specs/2026-10-12-scvx-landing-design.md`, `docs/ansichten.md`

**Interfaces:**
- Consumes: `FlightSession::{set_booster_return, set_realtime_planning, return_status, return_guidance, landing_outcome}`.
- Produces: GDScript API `FlightSim.set_booster_return(enabled: bool)`, `FlightSim.set_realtime_planning(enabled: bool)`, `FlightSim.return_state(index: int) -> Dictionary` with keys `present`, `phase`, `guidance` ("SCvx"/"explizit"), `abort`, `plans`, `max_plan_ms`, `outcome`.

- [ ] **Step 1: Bindings**

`flight_sim.hpp` (public, next to `tower_state`):

```cpp
    void set_booster_return(bool enabled);
    void set_realtime_planning(bool enabled);
    godot::Dictionary return_state(int64_t index) const;
```

`flight_sim.cpp` (add `#include "starsim/sim/landing_outcome.hpp"`):

```cpp
void FlightSim::set_booster_return(bool enabled) {
    if (session_) session_->set_booster_return(enabled);
}

void FlightSim::set_realtime_planning(bool enabled) {
    if (session_) session_->set_realtime_planning(enabled);
}

godot::Dictionary FlightSim::return_state(int64_t index) const {
    godot::Dictionary d;
    d["present"] = false;
    if (!session_ || index < 0 || static_cast<size_t>(index) >= session_->body_count()) return d;
    const auto i = static_cast<size_t>(index);
    d["outcome"] = godot::String::utf8(sim::to_string(session_->landing_outcome(i)));
    const auto* status = session_->return_status(i);
    if (!status) return d;
    d["present"] = true;
    d["phase"] = godot::String::utf8(gnc::to_string(status->phase));
    d["guidance"] = godot::String::utf8(status->scvx ? "SCvx" : "explizit");
    d["abort"] = status->abort;
    const auto* guidance = session_->return_guidance(i);
    const auto* scvx = guidance ? guidance->scvx() : nullptr;
    d["plans"] = scvx ? scvx->statistics().plans : 0;
    d["max_plan_ms"] = scvx ? scvx->statistics().max_solve_ms : 0.0;
    return d;
}
```

and in `_bind_methods`:

```cpp
    ClassDB::bind_method(D_METHOD("set_booster_return", "enabled"), &FlightSim::set_booster_return);
    ClassDB::bind_method(D_METHOD("set_realtime_planning", "enabled"), &FlightSim::set_realtime_planning);
    ClassDB::bind_method(D_METHOD("return_state", "index"), &FlightSim::return_state);
```

- [ ] **Step 2: Game**

`game/vehicle/flight_controller.gd`, in the `KEY_P:` branch, before `sim.engage_ascent_autopilot(...)`:

```gdscript
			sim.set_booster_return(true)  # the booster flies back and is caught after staging
```

`game/world/world.gd`, before `var reached := flight.advance(clock.tdb_s, _delta)`:

```gdscript
		flight.sim.set_realtime_planning(clock.warp <= 1.0)  # landing re-plans never stall real-time play
```

`game/ui/flight_hud.gd`, after the tower line block:

```gdscript
	if sim.body_count() > 1:
		var ret: Dictionary = sim.return_state(0)
		if ret.get("present", false):
			lines.append("Booster         %s (%s)" % [ret["phase"], ret["guidance"]])
			if ret["abort"]:
				lines.append("Turm            Abbruch")
			if ret["outcome"] != "fliegt":
				lines.append("Ergebnis        %s" % ret["outcome"])
```

(The tower line already shows "Abbruch" from `CatchPhase::Aborted`; the extra "Turm Abbruch" line is only added if the tower line is absent - check by reading `flight_hud.gd` lines 55-60 and keep a single "Turm" line: if `tower` is present, skip the extra line.)

- [ ] **Step 3: Build with bindings and smoke test**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` then
`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd`
Expected: build and all tests pass; smoke test ends without errors.

- [ ] **Step 4: Documentation**

`CLAUDE.md`:
- Known issues: replace the bullet "Booster return lands on the pad for blocks 1-3 (...) Planned: powered-descent guidance by convex optimisation ..." by: "Booster landing burn: SCvx plans (`gnc/scvx_landing`) re-planned every 1 s on a worker thread (effective at snapshot + 0.5 s; tests and time warp wait, real-time play never waits - `FlightSession::set_realtime_planning`). Blocks 1-3 are caught (< 1 m, < 0.5 m/s); the dispersion campaign `[gnc][catch][validation]` catches >= 19 of 21. Planning model: constant CG/inertia, no grid fins, no wind; no re-plans in the last 3 s. The explicit laws remain the fallback if the first plan fails."
- GNC section: add `gnc/descent_snapshot` (sim-thread snapshot, the worker never calls SPICE), `gnc/descent_replanner`, `gnc/descent_tracking`, `gnc/scvx_landing`; `sim/landing_outcome` (shoreline `[site.sea]`, "Wasserung"); tower `CatchPhase::Aborted`.

Spec: append a section "## Refinements during Plan 3b" listing the ledger's rulings that changed behaviour (one line each) and the measured re-plan times (from the Task 9/10 prints).

`docs/ansichten.md`: in the flight HUD description add the lines "Booster <Phase> (SCvx/explizit)", "Turm Abbruch" and "Ergebnis gefangen/gelandet/Wasserung/Aufprall" (German, matching the HUD).

- [ ] **Step 5: Commit**

```bash
git add bindings/src/flight_sim.hpp bindings/src/flight_sim.cpp game/ui/flight_hud.gd game/vehicle/flight_controller.gd game/world/world.gd CLAUDE.md docs/superpowers/specs/2026-10-12-scvx-landing-design.md docs/ansichten.md
git commit -m "feat: booster return in the game with SCvx status, abort and landing outcome on the HUD

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE"
```
