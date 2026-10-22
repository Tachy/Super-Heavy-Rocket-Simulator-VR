# External View (desktop + VR) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** An external view of any simulated vehicle body with a transparent SpaceX-webcast overlay (relevance-chosen
round gauges of the viewed body, whole-mission timeline arc, UTC + T-/T+ clock), in two start modes: desktop (16:9)
and PC-VR (overlay on a curved panel 2 m ahead, fixed to the camera's base pose, headset 6DOF on top).

**Architecture:** The core gets a mission-wide event log (`sim::MissionTimeline`): recorded events from hooks and a
per-tick observer, predicted events from the active plans, T-0 rules, and a scheduled liftoff with a cheap pad hold.
Bindings expose it like the other snapshot states. The game gets an `XrManager` autoload (sibling-project pattern), a
reworked `OrbitCamera` (CG target, local-vertical up, right-drag, Tab object switch), and a webcast overlay rendered
once into a transparent `SubViewport` that is shown full-screen on the desktop or mapped onto a sphere-section mesh in VR.

**Tech Stack:** C++20 core (Catch2), godot-cpp GDExtension, Godot 4.7.2 double build (GDScript, OpenXR).

**Spec:** `docs/superpowers/specs/2026-10-22-external-view-design.md`

## Global Constraints

- Physics and event detection only in `core`; bindings thin; GDScript only reads snapshots and sends commands.
- SI internally, frame + unit in names (`epoch_tdb_s`, `position_gcrf_m`); no magic numbers: tuned values are named
  constants with a provenance tag ([E] estimate, [D] derived, [S] sourced).
- UI text German, code/comments/docs English. Core UI strings (event labels) are German UTF-8.
- Every start mode: desktop = `--xr-mode off -- --no-xr`; VR without a headset -> message and quit, never a silent
  desktop fallback; `--script` / headless runs are always desktop.
- Overlay: the viewed object's gauges only; a gauge only while its value is simulated and meaningful; the arc shows
  the whole mission with all objects and does not change with the viewed object.
- Clock: T- before T-0, then only T+ for the whole mission. T-0 = liftoff for launches; else the first action (first
  burn ignition, or undocking). A wait for a window before the first action counts T- over the whole wait.
- VR panel: sphere section centred on the eye, radius 2 m, in the camera view direction, fixed to the base pose (not
  the head). F12 = usual VR recentre only (headset yaw + position), never touches orbit angles/distance/viewed object.
- Mouse: right drag up/down = pitch about the object, left/right = rotation about the local vertical; wheel = ×1.15
  distance per notch (logarithmic), FOV fixed; left button stays free; never rolled.
- Existing scripts keep working: `smoke_test.gd`, `flight_check.gd`, `render_check.gd`, all core tests of touched tags.
- Free assets only, recorded in `tools/data_pipeline/sources.toml` (CREDITS.md regenerated).
- Commit after each task (Conventional Commits) and `git push origin main` once its tests are green. Run only the
  affected test tags (`tools/build.ps1 -NoBindings -TestFilter "<tags>"`; bindings tasks: `tools/build.ps1 -TestFilter ...`).

## Review Focus

1. A mission with no launch and no planned burn (e.g. a placed body coasting): no T-0 -> the clock shows UTC only, no
   T-/T+, the arc shows recorded events or stays empty; nothing divides by a zero span. -> Task 2 test, Task 8 test.
2. The viewed body disappears (crew body removed, station kept by `keep_only_station`, campaign relaunch reindexes
   bodies): the view falls back to the controlled body without a camera jump larger than the distance clamp. -> Task 7 test.
3. Time warp through a long countdown (hours on the pad waiting for an ISS window): the pad hold coasts, the vehicle
   stays on the pad, liftoff happens at T-0 (not early, not skipped). -> Task 4 test.
4. Events all in the past, or > 50 events (52-day flown campaign): the arc window still shows the last event and the
   next ones, labels do not overlap into illegibility (thinning by priority). -> Task 8 test.
5. Earth <-> Moon reference-body change during a lunar transfer: "up" blends over `UP_BLEND_S`, no roll appears, no
   NaN when the camera looks straight down the local vertical (pitch clamp). -> Task 6 test.

---

### Task 1: Stable body serials

Bodies are reindexed on staging, removal and campaign relaunches; the timeline and the camera need a stable identity.

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp` (`Flying`, public accessor), `core/src/sim/flight_session*.cpp`
  (every place that creates a `Flying`: `grep -n "bodies_.push_back\|bodies_.emplace_back\|bodies_.insert"`, 8 sites)
- Test: `core/tests/test_mission_timeline.cpp` (new; add to `core/tests/CMakeLists.txt`)

**Interfaces:**
- Produces: `uint32_t FlightSession::body_serial(size_t index) const;` Serials start at 1, are unique for the session
  lifetime, never reused (also not after `clear_bodies`). On `stage()` the body that keeps flying as the controlled
  upper part keeps its serial only if it is the existing `Flying`; whichever `Flying` is newly created gets a new serial.

- [ ] **Step 1: Write the failing test** `TEST_CASE("Body serials are unique and survive reindexing", "[timeline]")`:
  launch `starship_block3` from the Starbase pad (as in `test_flight_session.cpp`), record `body_serial(0)`, ignite,
  release clamps, advance 10 s, `stage()`; CHECK `body_count()==2`, the two serials differ, the pre-staging serial is
  still present at some index, and both are `>= 1`.
- [ ] **Step 2:** Run `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[timeline]"` -> FAIL (no member).
- [ ] **Step 3:** Add `uint32_t serial = 0;` to `Flying`, `uint32_t next_serial_ = 1;` to the session, assign at every creation site.
- [ ] **Step 4:** Same command -> PASS.
- [ ] **Step 5:** Commit `feat(sim): stable body serials`.

### Task 2: MissionTimeline data model and T-0 rules

**Files:**
- Create: `core/include/starsim/sim/mission_timeline.hpp`, `core/src/sim/mission_timeline.cpp` (add to core CMake)
- Test: `core/tests/test_mission_timeline.cpp`

**Interfaces:**
- Produces:
```cpp
namespace starsim::sim {
enum class MissionEventKind { EngineStart, Liftoff, MaxQ, Meco, StageSeparation, Seco, Ignition, Cutoff, Boostback,
    LandingBurn, EntryInterface, Parachutes, CrewEjection, Docking, Undocking, Caught, Landed, Splashdown, Impact,
    Launch /* predicted liftoff of a later launch, e.g. the next tanker */ };
const char* to_string(MissionEventKind kind);  // German label: "Triebwerksstart", "Liftoff", "Max Q", "MECO",
    // "Stufentrennung", "SECO", "Zündung", "Brennschluss", "Boostback", "Landezündung", "Wiedereintritt",
    // "Fallschirme", "Ausstieg Crew", "Andocken", "Abkoppeln", "Gefangen", "Gelandet", "Wasserung", "Aufprall", "Start"
struct MissionEvent {
    MissionEventKind kind;
    time::Epoch epoch;
    uint32_t body_serial = 0;   // 0: mission-wide
    std::string body_name;      // German display name of the body at the event (upper stage name)
    bool predicted = false;
};
class MissionTimeline {
public:
    void reset(bool starts_on_pad);                 // a new mission (start_mission / free launch)
    void record(MissionEvent event);                // past event (predicted forced false), kept sorted by epoch
    void set_predicted(std::vector<MissionEvent> events);  // replaces all predicted events
    void set_scheduled_liftoff(std::optional<time::Epoch> t0);
    const std::vector<MissionEvent>& recorded() const;
    const std::vector<MissionEvent>& predicted() const;
    std::optional<time::Epoch> t0() const;
    std::optional<double> mission_clock_s(time::Epoch now) const;  // now - t0; negative = T-
};
}
```
- T-0 rules, in order: (1) first recorded `Liftoff`; (2) if `starts_on_pad`: the scheduled liftoff, else nullopt;
  (3) not on pad: first recorded `Ignition` or `Undocking`; (4) earliest predicted `Ignition` or `Undocking`; (5) nullopt.

- [ ] **Step 1: Write failing tests** (`[timeline]`):
  - `"T-0 of a pad mission is the scheduled liftoff, then the real one"`: reset(true), schedule t0=E+100 -> `t0()==E+100`,
    `mission_clock_s(E)==-100`; record Liftoff at E+101.5 -> `t0()==E+101.5`.
  - `"An engine start before liftoff is no T-0"`: reset(true), no schedule, record EngineStart at E -> `t0()` nullopt.
  - `"T-0 of an orbit mission is its first action"`: reset(false), predicted Ignition E+3600 -> t0 E+3600; record
    Undocking at E+10 -> t0 E+10 (recorded wins).
  - `"No action, no T-0"`: reset(false) -> `t0()` and `mission_clock_s` nullopt (Review Focus 1).
  - `"Recorded events stay sorted; predicted are replaced"`: record out of order -> sorted; `set_predicted` twice -> only the second set.
- [ ] **Step 2:** `tools/build.ps1 -NoBindings -TestFilter "[timeline]"` -> FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Same -> PASS.
- [ ] **Step 5:** Commit `feat(sim): mission timeline with T-0 rules`.

### Task 3: Recording events in the session

**Files:**
- Create: `core/include/starsim/sim/mission_event_observer.hpp`, `core/src/sim/mission_event_observer.cpp`
- Modify: `flight_session.hpp` (members `MissionTimeline timeline_; MissionEventObserver observer_;`, accessors),
  `flight_session.cpp` (`release_clamps`, `stage`, end of each tick and coast in `advance_to`), docking/undocking and
  crew-escape sites (`flight_session_docking.cpp`, `flight_session_crew_escape.cpp`), `mission_start.cpp`
  (`start_mission` calls `reset_timeline`)
- Test: `core/tests/test_mission_timeline.cpp`

**Interfaces:**
- Produces: `const MissionTimeline& FlightSession::timeline() const;`
  `void FlightSession::reset_timeline();` (calls `timeline_.reset(any body clamped)`, clears the observer);
  `class MissionEventObserver { public: void reset(); void observe(const FlightSession& s, MissionTimeline& t); };`
- Direct hooks (exact moment known): `release_clamps` -> Liftoff; `stage()` and payload/trunk separation ->
  StageSeparation (body = the separated lower part); hard capture -> Docking; undock -> Undocking; seat firing ->
  CrewEjection.
- Observer (per tick and after each coast; per-body memory keyed by serial; `telemetry()` only for bodies whose
  rules need it, at most every `observe_interval_s = 0.1` [E] sim seconds while ticking):
  - `engines_running` 0 -> >0: EngineStart while clamped; Boostback / LandingBurn if the body has return guidance
    (first relight after separation = Boostback, the last one = LandingBurn, decided by the return guidance phase);
    else Ignition. >0 -> 0: Meco for the first cutoff after liftoff of a body whose lowest stage is the stack's first
    stage while an upper stage is attached; Seco for the first cutoff of the controlled upper stage while the ascent
    autopilot is `UpperStage`; else Cutoff.
  - Max Q: per body, once: track the peak of `dynamic_pressure_pa` while `vertical_speed_mps > 0`; record at the peak
    epoch when q falls below `max_q_drop_fraction = 0.9` [E] × peak and the peak is `>= max_q_min_pa = 10e3` [E].
  - EntryInterface: Earth reference body, altitude crosses `entry_interface_altitude_m = 120e3` [S: conventional
    entry interface] downward after having been above it; once per descent.
  - Parachutes: the body's parachute phase leaves "none/stowed" for the first time.
  - Landing outcome change from Flying: Caught / Landed (Landed, TippedOver) / Splashdown (Splashdown, SplashedDown)
    / Impact (Impact, BrokenUp).

- [ ] **Step 1: Write failing tests** (`[timeline]`, ascent ones `[slow]` if > 40 s):
  - `"A Starship ascent records liftoff, Max Q, MECO, separation and SECO"`: `start_mission(...,"starship_leo",...)`,
    advance to orbit; CHECK the recorded kinds contain, in order, EngineStart, Liftoff, MaxQ, Meco, StageSeparation,
    Seco; Max Q between T+50 s and T+80 s; q at Max Q ≥ 10 kPa; MECO and SECO bodies differ.
  - `"Booster return records boostback, landing burn and the catch"`: same flight with booster return on; CHECK
    Boostback, EntryInterface not required, LandingBurn, Caught for the booster serial.
  - `"An undocking mission starts its clock at the undocking"`: `iss_departure` mission -> `t0()` equals the recorded
    Undocking epoch after it happens.
  - `"start_mission resets the timeline; campaign launches do not"`: two `start_mission` calls -> second log has no
    events of the first.
- [ ] **Step 2:** `tools/build.ps1 -NoBindings -TestFilter "[timeline]"` -> FAIL.
- [ ] **Step 3:** Implement hooks and observer.
- [ ] **Step 4:** `-TestFilter "[timeline],[sim],[mission]"` -> PASS (the touched session paths).
- [ ] **Step 5:** Commit `feat(sim): record mission events`.

### Task 4: Scheduled liftoff, pad hold, waiting for a window; predicted events

**Files:**
- Modify: `flight_session.hpp/.cpp` (`schedule_liftoff`, `PreLaunch` in `run_autopilot`, `coast_allowance_s`),
  `mission_start.hpp/.cpp`, `refuelling_mission.cpp`, `iss_mission.cpp` (the launch-window starts), mission TOML loader
  (`countdown_s`)
- Create: `core/src/sim/predicted_events.cpp` + declaration in `mission_timeline.hpp`
- Test: `core/tests/test_mission_timeline.cpp`

**Interfaces:**
- Produces:
  - `void FlightSession::schedule_liftoff(std::optional<time::Epoch> t0);` sets the timeline's scheduled liftoff; with
    a schedule, `PreLaunch` ignites at `t0 - liftoff_ignition_lead_s()` and releases the clamps at the first tick with
    all engines running and `epoch >= t0`.
  - `double FlightSession::liftoff_ignition_lead_s() const;` = max `startup_s` of the first-stage engines (config,
    e.g. Raptor 3: 3.0 s) + `liftoff_hold_margin_s = 0.5` [E].
  - Pad hold: while `PreLaunch` waits with a schedule and every body is clamped, `coast_allowance_s()` allows coasting
    up to `ignition epoch - burn_preparation_s`; a clamped body's coast keeps it on the pad (verify `FlightBody::coast`
    handles clamped bodies; fix there if not).
  - `struct StartOptions { bool wait_on_pad = false; double countdown_s = 0.0; };` new last parameter of
    `sim::start_mission(..., const StartOptions& options = {})`. `wait_on_pad`: launch missions (ascent, iss_launch,
    depot_launch, tanker_flight, hls_refuel) place the vehicle at `requested` and `schedule_liftoff` the window's
    liftoff (ascent without window: `requested + countdown_s`, `countdown_s` from the mission TOML key
    `[mission] countdown_s`, default `options.countdown_s`); stations added for these missions are placed at the
    session epoch, not at the liftoff. Default `false` keeps today's behaviour (existing tests unchanged).
  - `std::vector<MissionEvent> predicted_mission_events(const FlightSession& s);` called at the end of every
    `advance_to`: the scheduled liftoff (Liftoff), a pending burn (`autopilot().burn.time_to_ignition_s` -> Ignition),
    the ship deorbit plan (Ignition + EntryInterface), the lunar outbound / return plan burns, the rendezvous plan
    burns, the ISS mission's planned undocking, the lunar ascent liftoff, and the supervisor's next launch
    (`wake_epoch` -> Launch). Read the same accessors the bindings' `build_*` functions use.

- [ ] **Step 1: Write failing tests** (`[timeline]`):
  - `"A scheduled liftoff waits on the pad and lifts off at T-0"`: launch at E, `schedule_liftoff(E+6 h)`, engage
    ascent autopilot, `advance_to(E+6 h+20 s)` with `max_ticks` large; CHECK Liftoff recorded in [E+6 h, E+6 h+1 s],
    EngineStart in [T0-3.5 s-0.1, T0], before T0-10 s the altitude equals the pad height ±0.1 m, and the number of
    ticks (count `advance_to` calls with `max_ticks=4000` needed) shows coasting: the run finishes in < 60 s wall time.
  - `"An ISS launch can wait on the pad for its window"`: `start_mission(iss_launch, requested, time, {true})`;
    CHECK `session.epoch()==requested`, `timeline().t0()==launch_window->liftoff`, `mission_clock_s(epoch) < 0`.
  - `"Predicted events follow the plan"`: a ship in orbit with `plan_ship_return()` -> predicted contains an Ignition
    at the plan's burn epoch and an EntryInterface; after re-planning they move.
- [ ] **Step 2:** `-TestFilter "[timeline]"` -> FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** `-TestFilter "[timeline],[mission],[iss],[refuelling]"` (their `[slow]` cases included) -> PASS.
- [ ] **Step 5:** Update `docs/subsystems.md` (ascent: scheduled liftoff + pad hold; new section "Mission timeline").
  Commit `feat(sim): scheduled liftoff, pad hold and predicted mission events`.

### Task 5: Bindings

**Files:**
- Create: `bindings/src/flight_snapshot_timeline.cpp`
- Modify: `bindings/src/flight_snapshot.hpp/.cpp` (snapshot fields, `body_stages`, `body_engines`),
  `bindings/src/flight_sim.hpp/.cpp` (getters, command params), `game/tests/smoke_test.gd`

**Interfaces:**
- Produces (GDScript):
  - `FlightSim.mission_timeline() -> Dictionary`: `{"t0_tdb_s": float or null, "clock_s": float or null,
    "events": Array[{"kind": String (enum name, snake_case), "label": String (German), "tdb_s": float,
    "body_serial": int, "body_name": String, "predicted": bool}]}` sorted by `tdb_s`.
  - `FlightSim.body_serial(index: int) -> int`.
  - `body_stages(i)[k]` gains `"oxidizer_kg", "fuel_kg", "oxidizer_capacity_kg", "fuel_capacity_kg"` (main tanks).
  - `body_engines(i)[k]` gains `"stage": int` (index into `body_stages`).
  - `launch_from_site(vehicle, site, heading_deg, tdb_s, countdown_s := 0.0)`: > 0 schedules liftoff at
    `tdb_s + countdown_s` and resets the timeline with `starts_on_pad`.
  - `start_mission(id, tdb_s, wait_on_pad := false)`.
- [ ] **Step 1:** Extend `smoke_test.gd`: after `start_mission("starship_leo", tdb)` + advance 200 s,
  `mission_timeline()["events"]` contains `kind == "liftoff"`; `body_serial(0) >= 1`; `body_engines(0)[0].has("stage")`;
  `body_stages(0)[0]["oxidizer_capacity_kg"] > 0`; `launch_from_site(..., tdb, 30.0)` -> `clock_s` ≈ -30 at start.
- [ ] **Step 2:** `tools/build.ps1 -TestFilter "[timeline]"`, then the Godot smoke test (CLAUDE.md command) -> FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Both -> PASS ("smoke test passed" line, exit 0).
- [ ] **Step 5:** Commit `feat(bindings): mission timeline, body serials, tank levels`.

### Task 6: XR manager, start modes, camera math

**Files:**
- Create: `game/xr/xr_manager.gd` (autoload `XrManager`), `game/world/camera_math.gd` (static helpers),
  `Start-VR.cmd`, `game/tests/camera_math_test.gd`
- Modify: `game/project.godot` (autoload, `[xr]` OpenXR enabled, `startup_alert=false`, `reference_space=0`,
  `d3d12` driver override for VR as in the sibling), `Start-Desktop.cmd` (`--xr-mode off -- --no-xr`),
  `game/world/orbit_camera.gd`

**Interfaces:**
- Consumes the sibling `../Stunt-Track-Racer-VR/scripts/autoload/xr_manager.gd` (port: `_ready` mode decision,
  `set_base`, `recenter`, F12, VSync off in VR); drop fade/FPS panel unless needed.
- Produces:
  - VR start without a headset: German message "Kein VR-Headset gefunden" and `get_tree().quit(1)`.
  - `XrManager.xr_active: bool`, `XrManager.origin: XROrigin3D`, `XrManager.set_base(basis: Basis) -> void` (origin
    stays at the render origin; only the basis), `XrManager.recenter()` (F12: headset yaw + position only).
  - `CameraMath.horizon_frame(up_render: Vector3, previous_north: Vector3) -> Basis` (y = up, -z = north; uses
    `previous_north` when up is parallel to the pole axis), `CameraMath.orbit_offset(frame: Basis, yaw: float,
    pitch: float, distance_m: float) -> Vector3`, `CameraMath.blend_frame(from: Basis, to: Basis, t: float) -> Basis`,
    `CameraMath.zoom(distance_m: float, notches: int, min_m: float, max_m: float) -> float` (×1.15 per notch),
    `const PITCH_LIMIT := 1.55`, `const UP_BLEND_S := 3.0` [E].
  - `OrbitCamera`: new fields `target_gcrf_m` (CG), `reference_center_gcrf_m` (Earth or Moon centre); rotation only
    on right drag; in VR it does not become `current` and feeds `XrManager.set_base(basis)` each frame.
- [ ] **Step 1: Write failing tests** `camera_math_test.gd` (pattern of `smoke_test.gd`, `_check` + exit code):
  camera x-axis ⟂ up (no roll) for 20 random yaw/pitch; `orbit_offset` length == distance; `zoom(100, 1, 1, 1e9)
  == 100/1.15`; zoom clamps at min/max; `blend_frame` at t=0/1 returns the ends and at 0.5 is orthonormal; pitch
  ±PITCH_LIMIT with up parallel to the view gives finite bases (Review Focus 5).
- [ ] **Step 2:** `godot ... --headless --path game --script res://tests/camera_math_test.gd` -> FAIL.
- [ ] **Step 3:** Implement `camera_math.gd`, the camera changes, the autoload and start scripts.
- [ ] **Step 4:** camera_math_test PASS; smoke_test PASS (headless = desktop); `flight_check.gd` PASS.
- [ ] **Step 5:** Commit `feat(game): VR start mode and external camera math`.

### Task 7: Viewed object and camera target

**Files:**
- Create: `game/world/view_target.gd`, `game/tests/view_target_test.gd`
- Modify: `game/world/world.gd` (`_update_camera` uses the view target), `game/vehicle/flight_controller.gd`
  (launch with `countdown_s`, `LAUNCH_COUNTDOWN_S := 60.0` [E] for free launches)

**Interfaces:**
- Produces: `class_name ViewTarget extends Node`: `var serial: int` (0 = controlled body);
  `func resolve(sim: FlightSim) -> int` (index of `serial`, else the controlled index, and sets `serial` to it);
  `func cycle(sim: FlightSim, step: int) -> void` (Tab = +1, Shift+Tab = -1, ordered by serial);
  `func cg_gcrf_m(sim: FlightSim, index: int) -> Vector3` (position + attitude × body axis × `body_cg_station_m`);
  `func object_length_m(sim: FlightSim, index: int) -> float` (sum of stage lengths). On switch the camera keeps
  yaw/pitch, distance clamped to `[0.6, 40] × object length` [E].
- The camera's local vertical comes from `telemetry(i)["reference_body"]` (Earth centre or `world.moon_gcrf_m`); a change
  starts the `UP_BLEND_S` blend.
- [ ] **Step 1: Write failing test** `view_target_test.gd` with a real `FlightSim`: launch Starship, cycle -> stays
  (one body); advance past staging -> cycle reaches the other serial; `resolve` after the viewed serial is gone
  (start a new mission) returns the controlled index (Review Focus 2).
- [ ] **Step 2:** Run it headless -> FAIL.
- [ ] **Step 3:** Implement; Tab handling in `world.gd` input.
- [ ] **Step 4:** view_target_test, smoke_test, flight_check -> PASS.
- [ ] **Step 5:** Commit `feat(game): viewed object with Tab switching and CG target`.

### Task 8: Overlay logic: gauge rules and timeline window

**Files:**
- Create: `game/ui/webcast/gauge_rules.gd`, `game/ui/webcast/timeline_window.gd`, `game/tests/webcast_rules_test.gd`

**Interfaces:**
- Produces:
  - `GaugeRules.select(telemetry: Dictionary, body: Dictionary, state: Dictionary, now_s: float) -> Dictionary`
    returning `{"left": Array[String], "right": Array[String]}`; `body` = `{"engines": Array, "stages": Array,
    "docking": Dictionary}`; `state` is the caller-owned hysteresis memory (last-true time per gauge id).
    Left (vehicle), priority order: `engines` (body has engines), `g_load` (not clamped), `dynamic_pressure`
    (> 100 Pa), `mach` (knudsen < 0.1, existing rule), `heat_flux` (heat_rate_w_m2 > 1e3), `heat_load` (with heat_flux).
    Right (trajectory): `speed_surface`, `speed_inertial`, `altitude` (always), `vertical_speed` (altitude < 150 km),
    `apoapsis`, `periapsis` (altitude > 100 km and apoapsis finite), `range`, `closing_rate` (docking state active for
    this body). A gauge hides only after its condition has been false for `HIDE_DELAY_S := 5.0`; at most
    `MAX_PER_SIDE := 4` per side, by priority.
  - `TimelineWindow.target_span(events_tdb: PackedFloat64Array, now_tdb: float) -> Vector2` (start, end): from the
    last past event minus 10 % of the span to the third next event plus 10 %; min span `MIN_SPAN_S := 60.0`; no past
    event -> start = now - 10 %; no future event -> end = now + max(10 % span, MIN_SPAN_S).
    `TimelineWindow.ease(current: Vector2, target: Vector2, delta_real_s: float) -> Vector2` (exponential,
    `EASE_TIME_S := 2.0` real seconds). `TimelineWindow.visible_labels(events: Array, window: Vector2, arc_px: float)
    -> Array` thins labels closer than `MIN_LABEL_PX := 70` keeping the higher-priority kind (Liftoff, MECO, SECO,
    stage separation, Docking, Undocking, landing outcomes above Ignition/Cutoff).
- [ ] **Step 1: Write failing tests** `webcast_rules_test.gd`: heat flux appears at 2e3 W/m², stays 4 s after
  dropping to 0, gone at 6 s; Mach hidden at knudsen 1; a clamped body has no g_load; never > 4 per side; window with
  events at -70 s and +30, +400, +3 d contains -70 s and +400 s and ends before +3 d; empty event list -> span
  ≥ 60 s; 60 events over 52 days -> labels thinned, no two closer than 70 px (Review Focus 1, 4).
- [ ] **Step 2:** Run headless -> FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** PASS.
- [ ] **Step 5:** Commit `feat(game): webcast gauge rules and timeline window`.

### Task 9: Webcast overlay widgets and desktop presentation

**Files:**
- Create: `game/ui/webcast/webcast_overlay.gd` (root Control, 1600×900 reference layout), `round_gauge.gd`,
  `engine_gauge.gd`, `timeline_arc.gd`, `top_bar.gd`, `overlay_host.gd`
- Modify: `game/main.gd` (overlay host; old `FlightHud`, `NavBall`, `FlightOverlay` behind toggle `F1`, default off;
  `TimePanel` unchanged), `tools/data_pipeline/sources.toml` + `fetch.py` entry for the font, `CREDITS.md`
  (regenerated), `game/tests/render_check.gd` (`--webcast` flag)

**Interfaces:**
- Consumes: Task 5 getters, Task 7 `ViewTarget`, Task 8 `GaugeRules` / `TimelineWindow`.
- Produces: `OverlayHost.viewport: SubViewport` (`transparent_bg = true`, size 1600×900, `UPDATE_ALWAYS`) containing
  `WebcastOverlay`; desktop: a full-window `TextureRect` (keep aspect, centred) in a `CanvasLayer`.
- Look: thin white strokes (2 px at 1600×900), white text with a soft dark outline, no opaque panels, values German
  units (km/h, km, m/s, g, kPa, kW/m², MJ/m²). Font: Barlow / Barlow Condensed (SIL OFL 1.1, google/fonts repo),
  fetched to `data/fonts/`, default theme font as fallback when missing.
- `engine_gauge.gd`: engines of the lowest stage that has engines, positions from `position_m` projected on the
  stage's cross-section (body x/z), lit when `firing`; two outer arcs: LOX (`oxidizer_kg / oxidizer_capacity_kg`)
  and CH4 (`fuel_*`), labelled.
- `top_bar.gd`: UTC `YYYY-MM-DD hh:mm:ss UTC`, mission clock `T- hh:mm:ss` / `T+ hh:mm:ss` (days: `T+ 3 d 04:12:09`),
  hidden when `clock_s` is null; the viewed object's name.
- `timeline_arc.gd`: an arc of 120° bottom centre, events as dots (filled = past, hollow = predicted) with labels from
  `TimelineWindow.visible_labels`, "now" marker. Labels stand alone (Liftoff, Max Q, MECO, SECO, ...): no body names
  on the arc (user 2026-10-22: it is clear which part they belong to). Both ends of the arc
  fade out smoothly as in the webcasts (user 2026-10-22): line, dots and labels lose opacity over the outer
  `ARC_FADE_FRACTION := 0.15` [E] of the arc length at each end (smoothstep), so events slide in and out softly.
- [ ] **Step 1:** Add `--webcast` to `render_check.gd` (starts `starship_leo`, advances to T+75 s, saves PNG).
- [ ] **Step 2:** Implement widgets, host, toggle, font pipeline entry.
- [ ] **Step 3:** Render checks (GPU, not headless): T+75 s ascent, a ship entry (heat gauges), a docked ISS mission
  -> read the PNGs; overlay legible, transparent, nothing overlaps.
- [ ] **Step 4:** smoke_test + flight_check + webcast_rules_test PASS; show the PNGs to the user for the look
  (spec: appearance reviewed on screenshots) and adjust.
- [ ] **Step 5:** Commit `feat(game): webcast overlay on the desktop`.

### Task 10: VR overlay panel

**Files:**
- Create: `game/xr/vr_overlay_panel.gd`
- Modify: `game/main.gd` (in VR: no desktop `TextureRect`; panel under `XrManager.origin`)

**Interfaces:**
- Produces: `class_name VrOverlayPanel extends MeshInstance3D`; `func setup(texture: ViewportTexture) -> void`;
  `const RADIUS_M := 2.0`, `const H_FOV_DEG := 64.0` [E: fits the Pimax/Quest comfort zone; tune in the VR session],
  vertical span from 16:9; mesh = sphere section centred on the origin (eye), facing -Z, 64×36 segments; material
  unshaded, transparent, `no_depth_test`, `render_priority` max, cull back; child of `XrManager.origin` (base pose),
  never of the `XRCamera3D`.
- [ ] **Step 1:** Headless test in `camera_math_test.gd`: every vertex of the generated mesh is 2.0 m ± 1 mm from
  the origin and within ±H_FOV/2 of -Z.
- [ ] **Step 2:** Run -> FAIL; implement; run -> PASS.
- [ ] **Step 3:** Manual VR session with the user (Start-VR.cmd): panel stays when the head turns; F12 brings it in
  front without changing the orbit; mouse orbit/zoom works; 90 fps in the ISS and launch scenes; no jitter at the
  Moon. Note findings in the ledger.
- [ ] **Step 4:** Commit `feat(game): webcast overlay on a curved VR panel`.

### Task 11: Docs and closing

**Files:**
- Modify: `docs/ansichten.md` (§1-2, §5 Außenansichten: Webcast-Overlay, Steuerung, VR-Panel, F12, Tab, F1; German),
  `CLAUDE.md` (Game structure: `game/xr/`, `game/ui/webcast/`; sim module list: mission timeline, scheduled liftoff),
  master plan M7 note (external view done first), memory.
- [ ] **Step 1:** Update the docs.
- [ ] **Step 2:** Final review over the plan's commits; run the affected tags `[timeline],[mission],[iss],[refuelling],[sim]`
  plus the Godot tests.
- [ ] **Step 3:** Commit `docs: external view`, push, ask the user for `/clear`.

## Ledger

| Task | State | Notes |
|---|---|---|
| 1-11 | open | |
