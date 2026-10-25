# Crew escape - Ejection seats on the crewed ship return - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** On every abort of a crewed ship return the onboard computer gives the ship up, the crew leaves on
ejection seats (each member simulated from the seat firing to touchdown) and the empty ship is flown on to its impact;
the game reads all of it through one session-wide state.

**Architecture:** Pure pieces first: a crew-escape config (seats, envelope, motor tables, timing, limits) and two
small body configs (seat + occupant, occupant under the main canopy); `gnc` gets the seat envelope, a time-based seat
sequencer and the catch-feasibility check; `EntryGuidance` gets `give_up(sea target)`. `sim` wires them into the
session (`CrewEscape` owned by `FlightSession`): arming, firing, seat bodies driven by an external wrench (catapult +
rocket), seat-man separation into a parachutist body, load/clearance/touchdown judging, ship `ReturnResult::Lost`. The
crewed missions carry `crew_escape = "<id>"`; bindings export `crew_escape_state` and the ship's return state.

**Tech Stack:** C++20 core, Catch2, toml++, godot-cpp bindings, TOML configs.

**Spec:** `docs/superpowers/specs/2026-10-22-crew-escape-design.md`

## Global Constraints

- Work and commit on `main`; push (`git push origin main`) after every task once its tests are green.
- Code, identifiers, comments, commits in English (Conventional Commits); UI strings in the core German; answers to the
  user in German.
- SI units internally; frame/unit in the name; no magic numbers (constants/config with provenance [S]/[E]/[D]).
- **The ship's data stay as they are (user 2026-10-22):** no change to `config/vehicles/starship_block3.toml` or any
  ship data (dry mass, CG, inertia, tanks, engines, aero). Seats and crew live in their own config. The ship keeps its
  mass and CG when the seats leave (documented simplification).
- Uncrewed missions, tankers, depot, test ships and every existing campaign are unchanged: everything new is inactive
  without a crew-escape config on the session.
- Tests: run only the affected tags (`-TestFilter`), `[slow]` cases of those tags included; tag new cases `[slow]` when
  > ~40 s; runs longer than 10 min go to the background with checks every 10 min.
- No heredocs: files and scripts with the Write tool.
- Build/test: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "<tags>"`; test sources
  are listed explicitly in `core/tests/CMakeLists.txt`, core sources in `core/CMakeLists.txt`.
- New test tag for everything here: `[crew_escape]`.

## Plan decisions (for the user's review)

1. **Seat = ACES II-class zero-zero seat** [S: Collins ACES II: 0-50,000 ft, 0-600 KEAS; modes by speed/height].
   Envelope: height above ground <= 15,240 m, equivalent airspeed <= 309 m/s (600 KEAS). Mode 1 (EAS < 129 m/s /
   250 KEAS and below 4,572 m / 15,000 ft): main canopy straight after the rocket; mode 2 (faster, below 15,000 ft):
   drogue first, then separation and main; mode 3 (above 15,000 ft): drogue until 15,000 ft, then as mode 2.
2. **Seat motion by an external wrench** (`FlightBody::apply_external_wrench`): catapult and rocket are time-thrust
   tables along the seat rail fixed at firing (pitch stabilisation not modelled - documented); their few kg of
   propellant are not subtracted [documented]. No new engine type in `propulsion`.
3. **Two body configs:** `ejection_seat` (seat + occupant, fixed drag, drogue) and `parachutist` (occupant, main canopy,
   `recovery = splashdown`). At seat-man separation the seat body is replaced by a parachutist body with the same
   state; the empty seat is dropped (not simulated).
4. **Hatch direction:** the ship's body +z (the leeward back in the belly flop; the belly is -z), seats in a row along
   the axis at the crew deck [E]; rails tilted 10 deg forward [E]. After the flip that points sideways: the clearance
   checks report what happens, nothing is forced.
5. **Catch feasibility (crewed only):** given up when a flip-group engine has failed or the usable landing propellant is
   below `min_catch_propellant_kg`. That number is **measured** in Task 3 (lowest nominal-gate load that is still
   caught, 1 t steps) + 2 t margin [D]; the f3/catch finding (21-23 t strikes the tower) is the expected range.
6. **Early give-up target:** pad + sea direction x (shoreline + 2,000 m) [E]; the seats wait until the ship is beyond
   the shoreline, at the latest at 1,500 m above ground [E] (mode 1 main fully open after ~3 s / ~300 m of fall).
7. **Stagger 0.4 s** between seats [E: multi-seat sequencing is ~0.3-0.5 s], order aft to forward.
8. **Judging limits** [S where sourced in Task 1, else E]: peak acceleration magnitude <= 20 g for longer than 0.1 s
   (catapult/rocket and opening shock), clearance >= 1 m to the ship hull, the plume cone (half-angle 15 deg [E],
   length 100 m [E]) while engines burn, and the tower geometry; touchdown sink <= 8 m/s [E]. Violations make the
   member "verloren" with the reason; landing on land is reported, not a loss.
9. **Ship result:** a crewed ship that is given up returns `ReturnResult::Lost` ("aufgegeben" reason) whatever its
   contact; uncrewed ships keep today's result.

## Review Focus

1. **Body index shifts:** seat bodies are added and erased (separation) while the ship is the controlled body - every
   stored index (controlled, station, tower, crew members) must survive; test: separation of seat 0 while seats 1-3
   are still on rockets keeps every member mapped to its body (Task 5).
2. **Time warp / coasting:** a session advanced in large steps must still tick seat and parachutist bodies (no
   analytic coast past a rocket burn or a deployment); test: `advance_to(t + 120 s)` in one call gives the same
   touchdown within 1 m as 0.1 s steps (Task 5).
3. **Give-up below the minimum firing height** (failure found at the gate, 800 m): fires at once - test asserts the
   seats fire within one tick and mode 1 is chosen (Task 5).
4. **Give-up while the ship tumbles** (abort beyond 35 deg): the rail direction follows the hull - test: seats fired from
   a 60 deg tilted ship report a clearance and a load, no NaN, outcome set (Task 5).
5. **Never inside the envelope:** a ship that breaks up / impacts before arming fires reports all members "verloren:
   außerhalb der Schleudersitz-Grenzen" (Task 6).

---

### Task 1: Crew-escape config and the seat / parachutist bodies

**Files:**
- Create: `config/crew_escape/starship_crew.toml`, `config/vehicles/ejection_seat.toml`, `config/vehicles/parachutist.toml`
- Create: `core/include/starsim/config/crew_escape_config.hpp`, `core/src/config/crew_escape_config.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`, `config/README.md` (new directory), `tools/data_pipeline/sources.toml` only if a new data source is used (none expected)
- Test: `core/tests/test_crew_escape_config.cpp`

**Interfaces:**
- Produces:
```cpp
namespace starsim::config {
struct ThrustSample { double time_s; double thrust_n; };          // piecewise linear, 0 outside
struct SeatPosition { math::Vec3 position_stage_m; };              // x station from the stage bottom, y/z body axes
struct CrewEscapeConfig {
    std::string id, seat_vehicle_id, parachutist_vehicle_id;
    int crew_count = 0;                                            // seats.size()
    std::vector<SeatPosition> seats;                               // firing order
    math::Vec3 rail_direction_body;                                // unit, body axes (+z tilted 10 deg to +x)
    double rail_length_m = 0.0;                                    // seat clears the hatch after this travel
    std::vector<ThrustSample> catapult, rocket;                    // along the rail
    double stagger_s = 0.0;
    double envelope_max_height_m = 0.0, envelope_max_eas_mps = 0.0;
    double mode1_max_eas_mps = 0.0, mode_low_height_m = 0.0;       // 129 m/s, 4,572 m
    double mode1_main_delay_s = 0.0, mode2_drogue_delay_s = 0.0, mode2_separation_delay_s = 0.0;
    double min_firing_height_m = 0.0, sea_target_margin_m = 0.0;
    double min_catch_propellant_kg = 0.0;                          // Task 3 measures it
    double max_load_g = 0.0, max_load_duration_s = 0.0, min_clearance_m = 0.0;
    double plume_half_angle_rad = 0.0, plume_length_m = 0.0, max_touchdown_sink_mps = 0.0;
};
CrewEscapeConfig load_crew_escape(const std::filesystem::path& config_dir, std::string_view id);
double thrust_at(const std::vector<ThrustSample>& table, double t_s);
}
```

- [ ] **Step 1: Research and write the configs.** Source the seat numbers (ACES II envelope and mode timing, seat +
  occupant mass ~190 kg, catapult ~1 m stroke / exit speed, rocket impulse and burn time, drogue (ACES II: 0.6 m
  hemisflo) and personnel main canopy (C-9 / GQ 5000: 8.5 m, sink ~6-7 m/s at sea level), opening loads) into
  `[sources]` of each file with [S]/[E]/[D] on every number. The seat body: `fixed_drag_area_m2` (seat + occupant,
  ~0.8 m^2 [E]) and one `[[stage.parachute]] role = "drogue"`; the parachutist: fixed drag (~0.5 m^2 [E]), one
  `role = "main"`, `[stage.recovery] method = "splashdown"`, `max_splashdown_speed_mps = 8.0`. Crew deck seat stations
  [E] in the ship's upper body (4 seats).
- [ ] **Step 2: Write the failing tests** (`[crew_escape][config]`):
  - `"Crew escape config loads"`: `crew_count == 4`, `seats.size() == 4`, `|rail_direction_body| == 1 +- 1e-12`,
    `envelope_max_eas_mps == Approx(308.7).margin(0.5)`, `stagger_s == 0.4`, `thrust_at(rocket, -1) == 0`,
    `thrust_at(rocket, t_mid) > 0`; both vehicle configs load with `config::load_vehicle`.
  - `"Crew escape config refuses bad values"`: non-finite or negative numbers, empty seat list, unknown vehicle ids,
    unsorted thrust tables -> `std::runtime_error` naming the key.
  - `"Seat bodies reach their canopy sink rates"` (reference): a parachutist body dropped from 1,500 m at 50 m/s with
    its main deployed sinks at 6-7 m/s (+-0.5) at 50 m height (US76, sea level site) - flown with `FlightSession::place_body`
    and `Vehicle::deploy_parachutes("main")`.
- [ ] **Step 3: Run, expect FAIL** (missing header / loader).
- [ ] **Step 4: Implement** `load_crew_escape` (toml++, as `load_ship_return_mission`; `crew_count` = number of
  `[[seat]]` tables) and `thrust_at` (linear interpolation, 0 outside the table).
- [ ] **Step 5: Run `-TestFilter "[crew_escape]"`, expect PASS.**
- [ ] **Step 6: Commit + push** `feat(config): crew escape config with ejection seat and parachutist bodies`.

### Task 2: Seat envelope and seat sequencer (gnc, pure)

**Files:**
- Create: `core/include/starsim/gnc/ejection_seat.hpp`, `core/src/gnc/ejection_seat.cpp`
- Test: `core/tests/test_ejection_seat.cpp`

**Interfaces:**
- Consumes: `config::CrewEscapeConfig`, `config::thrust_at` (Task 1).
- Produces:
```cpp
namespace starsim::gnc {
enum class SeatMode { Mode1, Mode2, Mode3 };
enum class SeatPhase { Waiting, Catapult, Rocket, Drogue, Separated, Main, Down };
const char* to_string(SeatPhase phase);   // German UI text
double equivalent_airspeed_mps(double dynamic_pressure_pa);          // sqrt(2 q / rho0), rho0 US76 sea level
bool inside_seat_envelope(const config::CrewEscapeConfig& c, double height_m, double dynamic_pressure_pa);
SeatMode seat_mode(const config::CrewEscapeConfig& c, double height_m, double dynamic_pressure_pa);
struct SeatInput { double t_since_fire_s; double height_m; double dynamic_pressure_pa; bool on_surface; };
struct SeatActions { double rail_thrust_n = 0.0; bool deploy_drogue = false; bool separate = false;
                     bool deploy_main = false; };
class SeatSequencer {                       // one per seat; mode latched at firing
public:
    SeatSequencer(const config::CrewEscapeConfig& c, SeatMode mode);
    SeatActions update(const SeatInput& in);
    SeatPhase phase() const;
    SeatMode mode() const;
};
}
```

- [ ] **Step 1: Write the failing tests** (`[crew_escape][gnc]`):
  - `"Seat envelope: belly flop inside, hypersonic and high outside"`: (3,000 m, q 5 kPa) inside; (20 km, any) outside;
    (5 km, q 60 kPa) outside; `equivalent_airspeed_mps(0.5 * 1.225 * 100 * 100) == Approx(100)`.
  - `"Seat modes by speed and height"`: (1,000 m, EAS 90) Mode1; (1,000 m, EAS 200) Mode2; (8,000 m, EAS 90) Mode3.
  - `"Mode 1 sequence"`: catapult thrust > 0 at t = 0.05 s, rocket thrust > 0 during its burn, `deploy_main` true exactly
    once at `mode1_main_delay_s`, never `deploy_drogue`; `Down` after `on_surface`.
  - `"Mode 2 sequence"`: drogue at `mode2_drogue_delay_s`, separation at `mode2_separation_delay_s`, main on the tick of
    the separation.
  - `"Mode 3 holds the drogue until the low-mode height"`: separation only once `height_m < mode_low_height_m`.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** (rail thrust = `thrust_at(catapult, t) + thrust_at(rocket, t)`; phases by time; mode 3
  waits for the height).
- [ ] **Step 4: Run `-TestFilter "[crew_escape]"`, expect PASS.**
- [ ] **Step 5: Commit + push** `feat(gnc): ejection seat envelope and sequencer`.

### Task 3: Catch feasibility check and its calibration

**Files:**
- Create: `core/include/starsim/gnc/catch_feasibility.hpp`, `core/src/gnc/catch_feasibility.cpp`
- Modify: `config/crew_escape/starship_crew.toml` (`min_catch_propellant_kg` with the measurement as [D])
- Test: `core/tests/test_catch_feasibility.cpp`; calibration probe as hidden case in `core/tests/test_ship_catch.cpp`

**Interfaces:**
- Produces:
```cpp
namespace starsim::gnc {
struct CatchFeasibility { bool feasible = true; std::string reason; };   // German reason
// Failed engine in any of the flip groups, or usable landing propellant (main + header tanks of the stage) below the
// minimum -> not feasible.
CatchFeasibility catch_feasibility(const vehicle::Vehicle& ship, const std::vector<std::string>& flip_groups,
                                   double min_catch_propellant_kg);
}
```

- [ ] **Step 1: Failing tests** (`[crew_escape][gnc]`): full 35 t ship feasible; one failed `ship/sea_level_2` engine ->
  reason contains "Triebwerk"; 15 t -> reason contains "Treibstoff".
- [ ] **Step 2: Run, expect FAIL; implement; run, expect PASS.**
- [ ] **Step 3: Calibrate:** hidden case `"Ship catch: lowest catchable gate load"` (`[.][ship-single]`) flies
  `nominal_gate()` with 35, 34, ... t until the first miss; write the lowest caught load + 2 t as
  `min_catch_propellant_kg` [D] with the probe result in the comment.
- [ ] **Step 4: Commit + push** `feat(gnc): catch feasibility check for crewed ships`.

### Task 4: Entry give-up - retarget to the sea and fly the belly flop down

**Files:**
- Modify: `core/include/starsim/gnc/entry_guidance.hpp`, `core/src/gnc/entry_guidance.cpp`
- Create: `core/include/starsim/gnc/sea_target.hpp`, `core/src/gnc/sea_target.cpp`
- Test: `core/tests/test_entry_give_up.cpp` (uses `ship_entry_scenario.hpp`)

**Interfaces:**
- Produces:
```cpp
// gnc/sea_target.hpp
EntryTarget sea_give_up_target(const LandingSite& site, double margin_m);   // pad + sea dir x (shoreline + margin), no nose
// EntryGuidance
void give_up(const EntryTarget& sea);   // new target, no gate (phase never becomes Gate), terminal steering to impact
bool given_up() const;
```

- [ ] **Step 1: Failing tests** (`[crew_escape][entry]`): an entering ship started at Mach 0.9 / 10 km, 6 km west of the
  pad heading east, `give_up(sea_give_up_target(site, 2000))`: never reaches `EntryPhase::Gate`; impacts with its
  ground point beyond the shoreline (`sim::over_water`) and > 300 m from the tower. Second case: give-up at 800 m
  (gate height) reports the impact point honestly (no assertion on water; asserts no NaN and impact).
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement:** `give_up` swaps `target_`, sets a flag that suppresses the gate trigger and forces the
  terminal phase if still in Entry; the terminal steering keeps the belly to the flow (unchanged law) towards the new
  point until the body impacts.
- [ ] **Step 4: Run `-TestFilter "[crew_escape],[entry]"`, expect PASS** (existing entry tests unchanged).
- [ ] **Step 5: Commit + push** `feat(gnc): entry guidance gives a ship up to the sea`.

### Task 5: Session crew escape - arming, firing, seat bodies, judging

**Files:**
- Create: `core/include/starsim/sim/crew_escape.hpp`, `core/src/sim/crew_escape.cpp` (member state + judging, no session),
  `core/src/sim/flight_session_crew_escape.cpp` (session wiring)
- Modify: `core/include/starsim/sim/flight_session.hpp` (members + API), `core/src/sim/flight_session.cpp` (gate hand-over:
  crewed feasibility check; `advance_to`: crew escape tick; `coast_allowance_s`: tick while seats fly;
  `erase_body`: shift crew indices), `core/src/sim/flight_session_return.cpp` (`run_ship_landing`: abort -> arm),
  `core/src/sim/landing_outcome.cpp` / `ship_return_result` (crewed give-up -> Lost)
- Test: `core/tests/test_crew_escape_session.cpp`

**Interfaces:**
- Consumes: Tasks 1-4.
- Produces:
```cpp
namespace starsim::sim {
enum class CrewOutcome { Flying, Rescued, Lost };
struct CrewMemberState {
    std::optional<size_t> body_index;      // seat or parachutist body; empty before firing / after the end
    gnc::SeatPhase phase; gnc::SeatMode mode;
    double fire_time_s = 0.0;              // mission time
    double load_g = 0.0, peak_load_g = 0.0, min_clearance_m = 1e9;
    std::optional<math::Vec3> touchdown_enu_m; bool touchdown_over_water = false; double touchdown_sink_mps = 0.0;
    CrewOutcome outcome = CrewOutcome::Flying; std::string reason;   // German
};
struct CrewEscapeState {
    bool armed = false, fired = false; std::string give_up_reason; double give_up_time_s = 0.0;
    std::vector<CrewMemberState> members;
    std::optional<math::Vec3> ship_impact_enu_m; double ship_impact_speed_mps = 0.0;
    double ship_impact_tower_distance_m = 0.0, ship_impact_shore_distance_m = 0.0; bool ship_impact_over_water = false;
};
}
// FlightSession
void set_crew_escape(std::optional<config::CrewEscapeConfig> config);   // nullopt: uncrewed (default)
const CrewEscapeState* crew_escape_state() const;                        // nullptr when uncrewed
```

- [ ] **Step 1: Failing tests** (`[crew_escape][session]`, starting with `fly_from_gate`-style placements; each prints
  per member mode, peak g, clearance, touchdown and outcome):
  - `"Uncrewed ship ignores the crew escape"`: no config -> `crew_escape_state() == nullptr`, catch unchanged.
  - `"Crewed ship with a failed engine before the gate gives up and the crew is rescued"`: entering ship (Task 4 start)
    with `ship/sea_level_2` failed -> `armed` before the gate, seats fire beyond the shoreline or at
    `min_firing_height_m`, 4 members `Rescued`, touchdown sinks <= 8 m/s, peak loads <= 20 g, clearances >= 1 m;
    `ship_return_result(0) == Lost`, ship impact over water.
  - `"A give-up at the gate fires at once"` (Review Focus 3): low propellant at `place_landing_ship` -> fired within one
    tick, mode 1, outcomes set.
  - `"A catch abort after the flip ejects the crew"`: `abort_in_flip` -> armed by the abort, fired, outcomes and
    clearances to plume/tower reported (assert set and finite, not rescued).
  - `"Tumbling ship: seats fire along the hull"` (Review Focus 4).
  - `"Separation keeps every member on its body"` (Review Focus 1).
  - `"One long advance gives the same touchdown"` (Review Focus 2).
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement the pure part** `crew_escape.cpp`: judging (load from the body's accelerometer magnitude /
  g0 with the duration window; clearance to the ship hull capsule, plume cone and `contact::hits_tower` with a 0.5 m
  seat radius; touchdown outcome).
- [ ] **Step 4: Implement the session wiring:** arm on (a) entry feasibility failing in any entry cycle (crewed, below
  the terminal hand-over) -> `EntryGuidance::give_up`, (b) the gate check (no `ShipLandingGuidance` created), (c)
  `ShipLandingGuidance` aborting / upright descent / `abort_landing`. Fire when armed and (inside envelope and
  (over water or below `min_firing_height_m` or not an early give-up)). Spawn each seat as in `add_station` with the
  ship's state + seat offset (velocity + omega x r), the stagger, the external rail wrench per tick; separation:
  erase the seat body, add a parachutist with its state; main via `Vehicle::deploy_parachutes`.
- [ ] **Step 5: Run `-TestFilter "[crew_escape]"` and `"[ship][landing]"`, expect PASS.**
- [ ] **Step 6: Commit + push** `feat(sim): crew escape on the crewed ship return`.

### Task 6: Crewed missions - orbit return and lunar direct return

**Files:**
- Modify: `core/include/starsim/config/mission_config.hpp` + loader (`crew_escape_id` in `ShipReturnMission` and
  `LunarReturnMission`, `[mission] crew_escape = "<id>"`, optional), the placement/start code of both missions
  (`sim::start_lunar_return_mission` calls `session.set_crew_escape(...)`; the ship return starts from the flown orbit
  (`plan_ship_return`), so the crewed orbit-return test places the mission and calls `set_crew_escape` itself; the game
  sets it from the mission when it starts one)
- Create: `config/missions/ship_return_starbase_crewed.toml`, `config/missions/ship_lunar_return_llo_crewed.toml`
  (copies of the uncrewed ones + `crew_escape = "starship_crew"`)
- Test: `core/tests/test_crew_escape_mission.cpp`

- [ ] **Step 1: Failing tests** (`[crew_escape][mission]`, `[slow]` where > 40 s):
  - `"Crewed missions load their crew escape"`; uncrewed ones have none.
  - `"Crewed orbit return: nominal flight is caught, the crew stays aboard"` - `fired == false`, Caught.
  - `"Crewed orbit return with an engine failed in the entry"` - rescued crew, ship in the sea.
  - `"Crewed lunar direct return with low propellant"` - give-up in the entry, rescued.
  - `"Crew outside the envelope is lost"` (Review Focus 5): a crewed ship broken up above the envelope (heat-shield
    limit forced low) -> all members Lost with "außerhalb der Schleudersitz-Grenzen".
- [ ] **Step 2: Run, expect FAIL; implement; run `-TestFilter "[crew_escape]"`, expect PASS.**
- [ ] **Step 3: Commit + push** `feat(sim): crewed ship return missions`.

### Task 7: Game interface

**Files:**
- Create: `bindings/src/flight_snapshot_crew_escape.cpp`
- Modify: `bindings/src/flight_snapshot.hpp/.cpp` (members `crew_escape`, `ship_return` in the snapshot; `absent()` in
  `empty_snapshot`), `bindings/src/flight_sim.hpp/.cpp` (getters `crew_escape_state()`, `ship_return_state(index)` +
  `bind_method`), `game/tests/smoke_test.gd`
- Test: Godot smoke test

- [ ] **Step 1:** `crew_escape_state()` dictionary: `present, armed, fired, reason, give_up_time_s, members` (array of
  `{body_index, phase, mode, fire_time_s, load_g, peak_load_g, min_clearance_m, position_gcrf_m, velocity_gcrf_mps,
  latitude_rad, longitude_rad, height_m, attitude, touchdown_over_water, touchdown_sink_mps, outcome, reason}`),
  `ship_impact` (`present, enu_m, speed_mps, tower_distance_m, shore_distance_m, over_water`).
  `ship_return_state(index)`: `present, phase (flip/landing/upright), aborting, upright_descent, result`.
- [ ] **Step 2:** smoke test reads both getters on a fresh sim (`present == false`).
- [ ] **Step 3:** full build (`tools/build.ps1 -TestFilter "[crew_escape]"`) + smoke test, expect PASS.
- [ ] **Step 4: Commit + push** `feat(bindings): crew escape and ship return state for the game`.

### Task 8: Docs, affected campaigns, final review

- [ ] **Step 1:** `docs/subsystems.md` new section "Crew escape" (decisions, numbers, sources, simplifications, debug);
  `CLAUDE.md` module summary line; `docs/ansichten.md` note (pilot view data source, rendering later).
- [ ] **Step 2:** affected campaigns unchanged: `-TestFilter "[ship][catch],[lunar][return],[crew_escape]"` incl. slow
  (background, checks every 10 min) - catch counts as before (ship catch 20/21, lunar direct 21/21, aerocapture
  21/21).
- [ ] **Step 3:** final review (fresh reviewer, most capable model) of `adbd181..HEAD`; fix pass; ledger + memory.
- [ ] **Step 4: Commit + push** `docs: crew escape`.
