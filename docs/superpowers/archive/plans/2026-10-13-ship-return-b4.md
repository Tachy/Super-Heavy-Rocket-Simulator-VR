# Ship Return from Orbit (b4) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A ship in low Earth orbit plans its own deorbit burn, flies it on the belly sea-level Raptor alone, enters
on the b2 guidance and is caught at the Starbase tower by b3: "Rückkehr nach Starbase" as one mission.

**Architecture:** `gnc/deorbit_planner` (onboard computer, simulation thread) searches the coming passes for one whose
ground track passes the target within the entry's crossrange reach, then shoots the burn epoch and the retrograde
delta-v so that the entry interface (120 km) is reached at the flight-path angle b2 was built for (-1 deg) and the entry
predictor, flying a nominal bank, ends at the gate. `sim::FlightSession` executes the node on the vehicle's deorbit
engine set (`[stage.recovery] deorbit_engine_groups`), steering the set's thrust axis through the CG, coasts to the
entry interface and hands the ship to the existing entry guidance (b2), which hands over to the flip and catch (b3).

**Tech Stack:** C++20 core (`gnc`, `sim`, `config`), Catch2, TOML config, GDScript (key + HUD only).

**Spec:** `docs/superpowers/specs/2026-10-13-ship-return-design.md` (section "b2-b4 outline", b4 mission).

## Global Constraints

- SI units, radians, frame and unit in the name; English code/docs, German UI strings; Conventional Commits.
- `core` only; no physics in GDScript or bindings. Planning on the simulation thread (it calls SPICE).
- User 2026-10-13: the deorbit burn runs on ONE sea-level Raptor, `sea_level_3` (270 deg, belly side, in the symmetry
  plane), gimballed so its thrust passes through the CG (`gnc::thrust_axis_body`). Super Heavy is always caught;
  the ship returns to a tower site.
- Estimates are tagged [E], derived [D], sourced [S].

## Review Focus

1. An orbit whose ground track never passes the site within the crossrange reach in the search window: the planner
   must say so (no burn), not burn anyway - Task 2 test "no pass".
2. Too little propellant for burn + landing reserve: refused with a reason - Task 2 test.
3. The deorbit burn with the single engine: no tumbling, thrust through the CG (rates stay small) - Task 1 test.
4. The minimum-impulse rule picks a configuration outside the deorbit set: the forced set must be honoured (full
   throttle, else minimum throttle of the same set) - Task 1 test.
5. Long coast before the burn (hours): the session coasts analytically; the burn is not missed - Task 3 test (phase
   with a long wait).

---

### Task 1: Deorbit engine set and the thrust-axis burn

**Files:**
- Modify: `config/vehicles/starship_block3.toml` (`[stage.recovery] deorbit_engine_groups = ["sea_level_3"]`)
- Modify: `core/include/starsim/config/vehicle_config.hpp`, `core/src/config/vehicle_config.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`
- Test: `core/tests/test_ship_deorbit.cpp` (new, add to `core/tests/CMakeLists.txt`)

**Interfaces:**
- Produces: `StageConfig::deorbit_engine_groups` (`std::vector<std::string>`, empty = free choice);
  `bool FlightSession::execute_burn(const gnc::BurnNode& node, const std::vector<std::string>& groups)` (forced set:
  full throttle if its start/shutdown impulse is <= 1/4 of the burn, else minimum throttle; false if neither);
  `BurnConfiguration::thrust_axis_body` (unit, body axes; (1,0,0) for symmetric sets). In state Burn the controller
  steers the hull so that this axis points along the remaining delta-v.

- [ ] Step 1: test "Deorbit burn on the belly Raptor alone": ship (dry + 45 t) placed in a 200 km circular orbit
  (`place_in_orbit` on the stage-erased config), `execute_burn(retrograde 60 m/s node at t0 + 300 s, {"sea_level_3"})`
  returns true, advance to the end. Expect: only `sea_level_3` ever produced thrust; delivered delta-v (velocity change
  minus gravity, integrated) within 1 % and 0.5 deg of the node; peak body rate during the burn < 0.02 rad/s; the burn
  configuration's `thrust_axis_body` tilted 1-5 deg from the hull axis.
- [ ] Step 2: run, expect FAIL (no overload / field).
- [ ] Step 3: implement config field + parser; `execute_burn` overload; thrust-axis attitude
  (q = q0 * q_off with q_off turning thrust_axis_body to body x: the commanded hull x-axis and roll reference are
  q's images of x and y; q0: x along the delta-v, body y towards the orbit normal).
- [ ] Step 4: run, PASS; full ctest. Step 5: commit `feat: deorbit burns run on the vehicle's deorbit engine set, thrust through the CG`.

### Task 2: Deorbit planner

**Files:**
- Create: `core/include/starsim/gnc/deorbit_planner.hpp`, `core/src/gnc/deorbit_planner.cpp` (add to `core/CMakeLists.txt`)
- Test: `core/tests/test_deorbit_planner.cpp`

**Interfaces:**
- Consumes: `gnc::EntryPredictorModel`, `gnc::predict_entry`, `gnc::EntryTarget`, `orbit::propagate_coast`,
  `frames::BodyOrientation`, `environment::GravityModel`.
- Produces:
  ```cpp
  struct DeorbitParameters {
      double entry_interface_height_m = 120000.0;          // b2's entry interface
      double entry_flight_path_rad = -1.0 * deg;           // b2 campaign nominal [E]
      double nominal_bank_rad = 50.0 * deg;                // [E] the plan flies this: margin both ways
      double max_crossrange_m = 60000.0;                   // b2 campaign reach [E]
      double search_window_s = 2.0 * 86400.0;
      double lead_s = 600.0;                               // earliest burn after planning (orientation)
      double gate_height_m = 800.0;                        // as EntryGuidanceParameters
  };
  struct DeorbitPlan {
      bool feasible = false;
      std::string reason;                 // German, when not feasible
      BurnNode node;                      // retrograde, impulsive at node.epoch
      double delta_v_mps = 0.0;
      time::Epoch entry_interface_epoch;  // predicted (coasting after the impulse)
      double entry_flight_path_rad = 0.0;
      double entry_speed_mps = 0.0;       // relative to the Earth
      double downrange_error_m = 0.0;     // nominal-bank prediction
      double crossrange_m = 0.0;          // zero-bank prediction (track vs target)
      double propellant_kg = 0.0;         // for the burn (rocket equation)
  };
  DeorbitPlan plan_deorbit(const math::StateVector& state_gcrf, time::Epoch epoch, const environment::GravityModel& gravity,
                           const frames::BodyOrientation& earth, const EntryPredictorModel& model, const EntryTarget& target,
                           double mass_kg, double exhaust_velocity_mps, double reserve_propellant_kg,
                           double available_propellant_kg, const DeorbitParameters& parameters = {});
  ```
- Algorithm: sample the coast every 60 s over the window; passes = local minima of the sub-satellite distance to the
  target; for each pass in time order: scan burn epochs over the orbit before the pass at a fixed delta-v guess,
  bracket the downrange-error sign change (- to +), then a 2-D Newton (finite differences) on (gamma_EI - target,
  downrange error) in (burn epoch, delta-v); accept when |crossrange| <= max; else next pass.

- [ ] Step 1: tests (SPICE environment as the session's: `PointMassGravity` default, `SpiceOrientation::earth()`):
  "Deorbit planner hits the entry interface for Starbase" (200 km, i 26.5 deg, a phase): feasible, |gamma_EI + 1 deg|
  < 0.02 deg, |downrange| < 2 km, |crossrange| < 60 km, delta-v 40-150 m/s, burn within the window and >= lead;
  re-propagating the impulsive burn independently reproduces the EI epoch within 1 s. "Deorbit planner refuses without
  propellant": available < burn + reserve -> not feasible, reason mentions Treibstoff. "Deorbit planner finds no pass
  in a short window": 1 h window on a phase far from the pass -> not feasible.
- [ ] Steps 2-4 RED/GREEN; Step 5: commit `feat: onboard deorbit planner for a return to the tower`.

### Task 3: Mission and the whole chain in the session

**Files:**
- Create: `config/missions/ship_return_starbase.toml`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`
- Create: `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp` (ship return mission)
- Test: `core/tests/test_ship_return_mission.cpp`

**Interfaces:**
- Produces: `config::ShipReturnMission load_ship_return_mission(config_dir, id)` (site id, orbit: perigee/apogee km,
  inclination, RAAN, argument of latitude at the epoch (UTC), orbit propellant t, deorbit parameters);
  `bool FlightSession::plan_ship_return()` (controlled body = ship with heat shield + flaps; plans with the return site
  target `ship_entry_target`, executes the burn on the deorbit set; false + `ship_return_status()` reason);
  `AutopilotState::Return` ("Rückkehr"): after the burn the ship coasts to `entry_interface_epoch - entry_lead_s`
  (300 s [E]), then gets its entry guidance (as `place_entering_ship`), which hands over to the landing at the gate.
- [ ] Step 1: test "Ship returns from orbit into the tower's arms" `[ship][catch][session][mission]`: mission placed
  (state from elements), `plan_ship_return()` true, advance until caught/impacted/limit. Expect caught within the
  tower's tolerance, no breakup, propellant >= 0, the gate reached.
- [ ] Steps 2-4; Step 5 commit `feat: ship return mission - deorbit, entry and catch in one session`.

### Task 4: Campaign over orbit phase and dispersions

**Files:** Test: `core/tests/test_ship_return_mission.cpp` (`[ship][mission][validation]`)
- [ ] Cases: orbit phase 0..315 deg in 45 deg steps (8) plus dispersed cases (orbit height +-10 km, orbit propellant
  +-3 t, burn execution: delta-v scale +-1 %, pointing +-0.5 deg). Expect all caught, or a refusal with a reason
  where the plan is infeasible (none expected); record the miss at the gate and the propellant left.
- [ ] Commit `test: ship return campaign over orbit phase and dispersions`.

### Task 5: Game key, HUD and docs

**Files:** `bindings/src/flight_sim.*` (method `plan_ship_return`, status text), `game/vehicle/flight_controller.gd`
(key R "Rückkehr"), `game/ui/flight_hud.gd` (deorbit plan line), `docs/ansichten.md`, `CLAUDE.md`.
- [ ] Smoke test passes; commit `feat: return to Starbase from the game (key R)`; docs commit.
