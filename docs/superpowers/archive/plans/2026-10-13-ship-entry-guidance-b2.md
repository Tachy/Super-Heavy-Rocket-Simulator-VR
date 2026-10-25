# Ship Entry Guidance (b2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The ship, from any entry state, flies a predictor-corrector entry and a subsonic belly flop to a gate
1.5 km above the Starbase tower.

**Architecture:** A pure 3-DOF predictor (Earth-fixed frame, the vehicle's own trimmed aero coefficients, online
scale factors) inside a bank-magnitude corrector with corridor constraints and heading-error bank reversals; below
Mach 0.7 a zero-effort-miss belly-flop law tilts the belly. `EntryGuidance` outputs an attitude setpoint for the
b1 `AttitudeController` (flaps + RCS, 0.5 rad/s). `FlightSession` gets a scenario entry point and setpoint
telemetry for the HUD.

**Tech Stack:** C++20 core (`gnc`, `vehicle`, `sim`), Catch2, godot-cpp bindings, GDScript HUD.

**Spec:** `docs/superpowers/specs/2026-10-13-ship-return-design.md` (section "b2 design").

## Global Constraints

- SI units, radians; frame and unit in names (`position_itrf_m`); no magic numbers (constants with source, [E]/[D] tags).
- `core` has no Godot dependency; modules depend downwards (`gnc` may use `vehicle`, `aero`, `environment`, `frames`).
- The predictor never calls SPICE (pure; worker-safe); the guidance converts GCRF <-> ITRF on the sim thread.
- Entry attitude control: `AttitudeController::Gains{0.5, 0.9, false, true}`, bank rate <= 1 deg/s (b1 ruling).
- UI strings German, code English. Commits: Conventional Commits with the session trailer.
- Build: `tools/build.ps1 -NoBindings -NoTests` (with bindings when `bindings/` changes, then the Godot smoke test).

## Review Focus

1. Target unreachable (too far / too short even at the bank limits): guidance saturates and still flies a stable
   entry, reports the predicted miss - test in Task 3 (corrector saturation).
2. A state a human produced (e.g. bank already at 120 deg, wrong sign) - predictor starts from the current bank:
   test in Task 2 (rate-limited start).
3. Near-vertical fall in the belly flop: no attitude reference from the velocity - `belly_flop_attitude` uses the
   local vertical only: test in Task 4.
4. Constraint binding (heat / load) - range given up, vehicle kept: test in Task 5.
5. Aero model mismatch (truth differs from the guidance's table): scale factors - dispersion campaign, Task 5.

---

### Task 1: Entry aero table and accelerometer

**Files:**
- Modify: `core/include/starsim/math/constants.hpp` (add `earth_gm_m3ps2 = 3.986004418e14` (IERS 2010),
  `earth_rotation_rate_radps = 7.292115e-5` (WGS84 nominal))
- Modify: `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp`
  (`aero_acceleration_body_mps2()`: aerodynamic acceleration at the end of the last tick, set in the entry block)
- Modify: `core/include/starsim/config/vehicle_config.hpp`, `core/src/config/vehicle_config.cpp`
  (`AeroConfig::entry_alpha_table`), `config/vehicles/starship_block3.toml`
- Create: `core/include/starsim/gnc/entry_aero_table.hpp`, `core/src/gnc/entry_aero_table.cpp`
- Test: `core/tests/test_entry_guidance.cpp` (new, add to `core/tests/CMakeLists.txt`)

**Interfaces (Produces):**
```cpp
namespace starsim::gnc {
struct AlphaSchedule {           // nominal angle of attack over Mach
    std::vector<double> mach, alpha_rad;
    double at(double mach) const;  // linear, clamped
};
AlphaSchedule load_alpha_schedule(const std::filesystem::path& config_dir, const std::string& table);
class EntryAeroTable {
public:
    EntryAeroTable() = default;
    // From a single-stage vehicle (the ship) at its current mass properties; flaps in pitch trim.
    EntryAeroTable(const vehicle::Vehicle& ship, const AlphaSchedule& schedule);
    double cd_area_m2(double mach) const;   // drag / dynamic pressure
    double cl_area_m2(double mach) const;   // lift / dynamic pressure (in the alpha plane)
    double alpha_rad(double mach) const;
    bool empty() const;
};
}
```
Trim: forward pair at `mid + delta`, aft pair at `mid - delta` (mid = half the max deflection, clamped to
[0, max]); `delta` by bisection on the pitch moment about the CG (hull `normal_force_station_m` lever + flap
forces at their centroids); no root -> the end with the smaller |moment|. Air: rho 1e-3 kg/m^3, a 300 m/s,
mean free path 0 (continuum), air velocity body = M a (cos alpha x + sin alpha belly). Drag = -F.u, lift =
F.l with l = normalized(x - (x.u) u). Mach grid: 0.3 0.5 0.7 0.8 0.9 1 1.2 1.5 2 3 4 6 8 10 15 20 25 30.

- [ ] Step 1: test "Entry aero table reproduces the 6-DOF ship's aerodynamic acceleration" `[ship][guidance]`:
  `ShipEntryScenario` nominal start, law = alpha schedule, bank 0, until Mach 0.6; at samples with Mach in
  {25, 15, 5, 2, 0.8} (continuum, below 80 km) compare the body's `aero_acceleration_body_mps2()` drag and lift
  (along / across the relative velocity) with `q * table / mass`: within 10 %. Also L/D at Mach 20 in 0.3-0.9.
- [ ] Step 2: run, expect compile failure (RED).
- [ ] Step 3: implement constants, accessor (store `end.aero_body_n / end.mass.mass_kg` in the entry block),
  config key, table; move the replay test's alpha loader to `load_alpha_schedule`.
- [ ] Step 4: run `[ship][guidance]` and `[entry]` -> pass; full ctest green.
- [ ] Step 5: commit `feat: entry aero table (trimmed C_D A, C_L A over Mach) and aero accelerometer`.

### Task 2: Entry predictor

**Files:** Create `core/include/starsim/gnc/entry_predictor.hpp`, `core/src/gnc/entry_predictor.cpp`;
test in `core/tests/test_entry_guidance.cpp`.

**Interfaces:**
- Consumes: `EntryAeroTable`, `stagnation_heat_rate_w_m2`, `Us76Atmosphere`, `frames::fixed_to_geodetic`.
- Produces:
```cpp
struct EntryTarget {
    math::Vec3 position_itrf_m;   // gate point projected to the ground (target ground)
    double ground_height_m = 0.0; // WGS84 height of the target ground
};
struct EntryPredictorModel {
    EntryAeroTable aero;
    double mass_kg = 0.0;
    double nose_radius_m = 0.0;
    double drag_scale = 1.0, lift_scale = 1.0;
    double bank_rate_radps = 1.0 * deg;
    double gm_m3ps2 = constants::earth_gm_m3ps2, j2 = constants::earth_j2,
           earth_radius_m = constants::wgs84_semi_major_axis_m, omega_radps = constants::earth_rotation_rate_radps;
};
struct EntryPredictorState {
    math::Vec3 position_itrf_m, velocity_itrf_mps;  // velocity relative to the rotating Earth
    double bank_rad = 0.0;                          // current (signed)
};
struct EntryPrediction {
    math::Vec3 end_position_itrf_m, end_velocity_itrf_mps;
    double time_s = 0.0;
    double downrange_error_m = 0.0;   // > 0: overshoot
    double crossrange_m = 0.0;        // target right of the predicted ground track end: > 0
    double peak_heat_rate_w_m2 = 0.0, heat_load_j_m2 = 0.0, peak_load_g = 0.0, peak_dynamic_pressure_pa = 0.0;
    bool reached_gate = false;        // false: skip-out, ground or time limit
};
EntryPrediction predict_entry(const EntryPredictorModel& model, const EntryPredictorState& start,
                              double bank_command_rad, const EntryTarget& target, double gate_height_m,
                              double heat_load_so_far_j_m2 = 0.0, double step_s = 1.0, double max_time_s = 3000.0);
// Geometry helpers: great-circle range (sphere through the points' directions, WGS84 a) and heading.
double ground_range_m(const math::Vec3& a_itrf, const math::Vec3& b_itrf);
```
Dynamics: RK4 on (r, v) in ITRF: `a = g_J2(r) - 2 w x v - w x (w x r) + (-D u + L l)/m`; `l` = up (radial) perpendicular
to u rotated about u by the bank (same convention as `entry_attitude`); bank moves toward the command at the rate
limit; alpha from the table's schedule (not needed for forces). Ends when the geodetic height <= target ground +
gate height (linear interpolation of the last step), height > 130 km after having been below 80 km, or time.
Downrange error = range(start, end) - range(start, target); crossrange = signed distance of the target from the
great circle through start and end, at the end.

- [ ] Step 1: tests: (a) "Entry predictor matches the 6-DOF ship at constant bank" - nominal start, bank 45 deg
  (start bank 45 deg), 6-DOF run to 1.5 km (stop_altitude); predicted end within 2 % of the 6-DOF ground range;
  (b) "Entry predictor range falls with bank and starts from the current bank" - ranges at commands 0 / 60 / 120 deg
  strictly decreasing; command 0 from a start bank of 120 deg flies shorter than from 0 deg.
- [ ] Step 2: RED (compile).
- [ ] Step 3: implement.
- [ ] Step 4: green; full ctest.
- [ ] Step 5: commit `feat: entry predictor (3-DOF, trimmed aero, rate-limited bank)`.

### Task 3: Entry guidance - entry phase

**Files:** Create `core/include/starsim/gnc/entry_guidance.hpp`, `core/src/gnc/entry_guidance.cpp`; modify
`core/tests/ship_entry_scenario.hpp` (`run_guided`); tests in `core/tests/test_entry_guidance.cpp`.

**Interfaces:**
```cpp
enum class EntryPhase { Entry, BellyFlop, Gate };
const char* to_string(EntryPhase);
struct EntryGuidanceParameters {
    double gate_height_m = 1500.0;            // [E] above the target ground (b3: flip start)
    double cycle_s = 2.0;
    double max_bank_rad = 150.0 * deg;
    double bank_rate_radps = 1.0 * deg;       // b1 ruling
    double constraint_fraction = 0.9;
    double max_dynamic_pressure_pa = 25000.0; // [E]
    double belly_flop_mach = 0.7;             // [E]
    double max_tilt_rad = 15.0 * deg;         // [E]
    double zem_gain = 3.0;
    // reversal corridor: |heading error| > a + b * v  [E], tuned in tests
    double reversal_heading_base_rad = 2.0 * deg, reversal_heading_per_mps = 1.0e-3 * deg;
};
struct EntryCommand {
    EntryPhase phase = EntryPhase::Entry;
    double alpha_rad = 0.0, bank_rad = 0.0;         // setpoint (Entry phase)
    math::Vec3 axis_gcrf, roll_reference_gcrf;      // for AttitudeController::steer
    double downrange_error_m = 0.0, crossrange_m = 0.0;  // latest prediction / ZEM
    bool constrained = false;                        // bank limited by a corridor constraint
    double drag_scale = 1.0, lift_scale = 1.0;
};
class EntryGuidance {
public:
    EntryGuidance(EntryTarget target, EntryAeroTable aero, double nose_radius_m, double belly_azimuth_rad,
                  const config::HeatShieldConfig& limits, EntryGuidanceParameters p = {});
    EntryCommand update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);
    EntryPhase phase() const;
    // corrector on its own (tests): bank magnitude for zero downrange error from a predictor state
    struct Solution { double bank_rad; EntryPrediction prediction; bool constrained; };
    Solution solve(const EntryPredictorModel& model, const EntryPredictorState& state, double heat_load_j_m2) const;
};
```
Corrector: f(s) = downrange error at |bank| = s with the current sign. f(0) <= 0 -> 0; f(max) >= 0 -> max;
else Illinois between the bracket (start from the last solution +-15 deg when it brackets), <= 8 predictions,
stop at |f| < 500 m. Constraints: if a peak > fraction * limit, bisection (6 steps) on s in [0, s] for the
largest s within all limits. Scale factors updated each cycle from the accelerometer when |a_aero| > 1 m/s^2:
`k += 0.3 (k_meas - k)`, clamped [0.5, 2]. Reversal: heading error = azimuth to the target - azimuth of the
relative velocity (wrapped); if |error| > corridor(v) and the bank sign turns the track away from the target,
flip the sign. Bank output rate-limited.

- [ ] Step 1: tests: (a) "Entry corrector hits the target on its own model" - predictor-only: target placed at
  the predicted end for bank 55 deg; solve from bank 0 returns 55 +- 3 deg and |error| < 1 km; targets beyond
  the full-lift range / short of the 150 deg range saturate at 0 / 150 deg; (b) "Guided 6-DOF entry reaches
  the belly-flop hand-over on course" - nominal start, target = 6-DOF bank-50 run's end + 200 km downrange
  (via predictor), hand-over (Mach 0.7) with a predicted gate miss < 3 km, no breakup.
- [ ] Step 2: RED. Step 3: implement (`run_guided(EntryGuidance&, max_time)` in the scenario: steer on the
  command's axis / roll reference, stop at phase Gate or breakup). Step 4: green + ctest.
- [ ] Step 5: commit `feat: entry guidance - bank predictor-corrector with corridor constraints and reversals`.

### Task 4: Belly flop to the gate

**Files:** Create `core/include/starsim/gnc/belly_flop.hpp`, `core/src/gnc/belly_flop.cpp`; modify
`entry_guidance.cpp`; tests in `core/tests/test_entry_guidance.cpp`.

**Interfaces:**
```cpp
// Belly-down attitude: belly = -up tilted away from `tilt_direction` (horizontal unit) by `tilt_rad`, nose along
// the horizontal `nose_azimuth` direction (made perpendicular to the belly). Needs no velocity.
EntryAttitude belly_flop_attitude(const math::Vec3& up, const math::Vec3& nose_horizontal,
                                  const math::Vec3& tilt_direction, double tilt_rad, double belly_azimuth_rad);
// ZEM steering: horizontal acceleration wanted to reach the gate point.
math::Vec3 belly_flop_acceleration(const math::Vec3& position_rel_gate_m /*from the gate point, local ENU*/,
                                   const math::Vec3& velocity_mps /*local ENU, relative*/, double gate_height_m, double gain);
```
Tilt = atan(|a| / g0) clamped to max tilt; t_go = (height above gate) / max(sink, 20 m/s).
Phase switch to Gate when the height above the target ground <= gate height.

- [ ] Step 1: tests: (a) "Belly flop attitude points the belly down, tilted away" (geometry: belly . -up =
  cos(tilt), nose horizontal); (b) "Belly flop steers to the gate" - scenario from 12 km, speed 120 m/s falling
  (gamma -85 deg), 2 km horizontal offset, guidance in phase BellyFlop: at the gate the horizontal error < 50 m,
  belly within 15 deg of down, sink < 200 m/s.
- [ ] Step 2: RED. Step 3: implement (EntryGuidance: `skip_to(EntryPhase)` for scenarios). Step 4: green + ctest.
- [ ] Step 5: commit `feat: subsonic belly flop steered to the flip gate`.

### Task 5: Whole entry, dispersion campaign, constraint

**Files:** tests in `core/tests/test_entry_guidance_campaign.cpp` (new).

- [ ] Step 1: tests: (a) campaign `[ship][guidance][validation]` - 12 cases (seeded, deterministic) as the spec:
  gamma +-0.2 deg, speed +-40 m/s, target range +-300 km, crossrange +-60 km, mass +-5 % (propellant), truth
  cross-flow and flap table x(0.9..1.1) via the scenario tweak; all reach the gate: horizontal < 100 m, belly
  within 15 deg of down, sink < 200 m/s, no breakup; verbose prints per case. Target: Starbase OLP-1 (from
  `config/launch_sites.toml`) on the scenario Earth, start state uprange (back-propagated along a great circle
  with heading 100 deg from the target by the nominal range). (b) "Entry guidance holds the load limit" -
  steep start (-2.5 deg), shield `max_load_g` set to 1.1 x the guided nominal peak... (chosen in the test from a
  first run): guided run stays below the limit and does not break up.
- [ ] Step 2: run; tune the [E] parameters (reversal corridor, ZEM gain, hand-over Mach) until green - every
  change ledgered with its effect.
- [ ] Step 3: ctest green; commit `test: entry guidance dispersion campaign to the Starbase flip gate`.

### Task 6: Session, bindings, HUD, docs

**Files:** `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`,
`bindings/src/flight_sim.cpp` (+ header), `game/ui/flight_hud.gd`, `CLAUDE.md`, `docs/ansichten.md`,
`config/README.md`; test `core/tests/test_flight_session_entry.cpp` (new).

**Interfaces:**
```cpp
// FlightSession
void place_entering_ship(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                         time::Epoch epoch, gnc::EntryGuidanceParameters parameters = {});  // target: the launch site's tower catch point
const gnc::EntryGuidance* entry_guidance(size_t index) const;
const gnc::EntryCommand* entry_status(size_t index) const;
```
`Flying` gains `entry_guidance` + `entry_command`; `command_for` steers such a body with an entry controller
(`Gains{0.5, 0.9, false, true}`) on the command's axis / roll reference; the target is the site's catch point
(tower catch pin point projected to the ground) or the pad. Bindings: telemetry dict `entry_phase`,
`entry_alpha_deg`, `entry_bank_deg`, `entry_downrange_km`, `entry_crossrange_km`. HUD: "Eintritt: Soll α / Bank,
Ist α, Abweichung".

- [ ] Step 1: test "Session flies an entering ship to the gate" (Starbase site, state from the Task 5 nominal,
  short: start at Mach 3 / 25 km uprange) - reaches phase Gate, horizontal miss < 150 m.
- [ ] Step 2: RED. Step 3: implement session + bindings + HUD. Step 4: full build with bindings, ctest, smoke test.
- [ ] Step 5: docs (CLAUDE.md "Ship entry guidance" known-issue block, ansichten HUD lines, config/README
  `entry_alpha_table`); commit `feat: ship entry guidance in the flight session, HUD setpoint`.
