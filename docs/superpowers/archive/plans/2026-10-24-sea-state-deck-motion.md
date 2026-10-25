# Sea State and Deck Motion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The drone ship's deck heaves, rolls and pitches in a configurable sea state, Falcon 9 stages land on the
moving deck, the onboard computer refuses a drone-ship recovery in rough sea, and the game draws the ship riding it.

**Architecture:** `environment::SeaState` (JONSWAP components) -> `contact::DeckMotion` (box-barge plane fit +
second-order response per DOF) -> `sim::LegLandingSim` computes the leg contact in the deck's moving frame and holds a
landed stage relative to the deck. `FlightSession` owns the sea state (from the mission's `[sea_state]`) and the
fidelity flag `deck_motion`; `plan_launch` checks the spectral deck statistics against the ship's limits. Bindings
expose the deck pose; `game/world/drone_ship_view.gd` draws the ship.

**Tech Stack:** C++20 core (Catch2 tests, toml++ config), godot-cpp bindings, Godot 4 GDScript.

**Spec:** `docs/superpowers/specs/2026-10-24-sea-state-deck-motion-design.md`

## Global Constraints

- No mission without `[sea_state]` may change behaviour: every existing test passes unchanged, no tolerance widened.
- SI units, radians internally; frame/unit in names (`heave_m`, `roll_rate_radps`); constants with source in
  `core/include/starsim/math/constants.hpp`; config values tagged [S]/[E]/[D] in TOML.
- `core` has no Godot dependency; modules depend downwards only (`contact` may use `environment` and `config`).
- UI/refusal strings German, code/comments/docs English. Commits Conventional Commits, pushed to `main` after each
  task with its affected tags green (`tools/build.ps1 -NoBindings -TestFilter "[tags]"`, CLAUDE.md test workflow).
- Fidelity flag `deck_motion` (default on): off -> level deck, no sea-state refusal.
- Deterministic: same seed -> same sea; the deck motion's time origin is a fixed epoch of the sea state (not wall clock).

## Review Focus

- A mission with `[sea_state]` followed by one without in the same session: the second must be calm (mission start
  resets the sea state). Test in Task 4.
- `[sea_state]` with Hs > 0 but Tp missing, Tp <= 0 or Hs < 0: load error naming the key, not a silent calm sea.
  Test in Task 4.
- A sea state while the landing zone is a pad (RTLS) or a tower catch: no deck motion, no refusal, no crash. Test in
  Task 3 (pad) and Task 4 (RTLS plan unaffected).
- A stage landed on the moving deck through high time warp (many ticks per frame): it stays on the deck, no drift or
  bounce, velocity = deck velocity. Test in Task 3.
- Flag `deck_motion` off with a rough `[sea_state]`: the old level deck and no refusal. Test in Task 4.

---

### Task 1: Sea state (JONSWAP)

**Files:**
- Create: `core/include/starsim/environment/sea_state.hpp`, `core/src/environment/sea_state.cpp`
- Modify: `core/CMakeLists.txt` (source), `core/include/starsim/math/constants.hpp` (JONSWAP gamma 3.3, sigma 0.07 /
  0.09 with source DNV-RP-C205 / Hasselmann 1973; reuse the existing standard gravity)
- Test: `core/tests/test_sea_state.cpp` (+ `core/tests/CMakeLists.txt`)

**Interfaces:**
- Produces:
  ```cpp
  namespace starsim::environment {
  struct SeaStateConfig {
      double significant_wave_height_m = 0.0;  // Hs = 4 sqrt(m0); 0: calm
      double peak_period_s = 0.0;
      double heading_rad = 0.0;   // direction the waves travel towards, clockwise from north
      std::uint64_t seed = 1;
  };
  struct WaveComponent { double amplitude_m, angular_frequency_radps, wavenumber_radpm, phase_rad; };
  double jonswap_spectral_density(double omega_radps, double hs_m, double tp_s);  // S(omega), m^2 s
  class SeaState {
  public:
      explicit SeaState(const SeaStateConfig& config, int component_count = 48);  // throws on hs < 0, tp <= 0 with hs > 0
      const SeaStateConfig& config() const;
      bool calm() const;                                     // hs == 0: no components
      const std::vector<WaveComponent>& components() const;
      double variance_m2() const;                            // sum a_i^2 / 2
      // elevation at (east, north) of a local tangent plane, t_s seconds after the sea's time origin
      double elevation_m(double east_m, double north_m, double t_s) const;
  };
  }
  ```

- [ ] **Step 1: Write the failing tests** (tag `[sea_state]`):
  - `"The JONSWAP spectrum integrates to Hs^2/16 and peaks at Tp"`: numeric integral of `jonswap_spectral_density`
    over (0, 10 omega_p] for Hs 2.5 m, Tp 8 s equals 2.5^2/16 within 1 %; argmax at 2 pi / 8 within 1 %.
  - `"A discretised sea keeps the spectral variance"`: `SeaState({2.5, 8.0, 0.3, 7}).variance_m2()` within 1 % of
    0.390625; every component satisfies omega^2 = g k (1e-12 rel.).
  - `"The sampled elevation has the spectral variance"`: mean of `elevation_m(0,0,t)^2` over t in [0, 3600] s step
    0.25 s within 10 % of `variance_m2()`.
  - `"The same seed gives the same sea, another seed another"`: elevation at (10, -20, 33) equal for seed 7 twice,
    different for seed 8.
  - `"A calm sea is flat and bad parameters are refused"`: Hs 0 -> `calm()`, elevation 0; Hs -1 and (Hs 2, Tp 0) throw
    `std::invalid_argument`.
  - Waves travel towards `heading_rad`: with heading 90 deg (east) the elevation at (east = c t, 0, t) for the
    single-component limit (component_count 1) is constant to 1e-9 m (c = omega / k).
- [ ] **Step 2: Run** `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[sea_state]"`
  - expected: build error (header missing).
- [ ] **Step 3: Implement.** Discretise omega in [0.5 omega_p, 3 omega_p] into equal-energy bins (or equal widths;
  builder's choice, documented) with amplitude sqrt(2 S(omega_i) d omega_i), rescaled so the discrete variance equals
  Hs^2/16 exactly; phases uniform from `std::mt19937_64(seed)` (portable: draw raw 64-bit values, map to [0, 2 pi)
  yourself, no `std::uniform_real_distribution`). Long-crested: all components along `heading_rad`.
- [ ] **Step 4: Run the filter** - all `[sea_state]` PASS.
- [ ] **Step 5: Commit + push** `feat(environment): JONSWAP sea state`.

### Task 2: Drone-ship hull and deck motion

**Files:**
- Modify: `config/drone_ships.toml` (hull + limits per ship), `core/include/starsim/config/drone_ship_config.hpp`,
  `core/src/config/drone_ship_config.cpp`
- Create: `core/include/starsim/contact/deck_motion.hpp`, `core/src/contact/deck_motion.cpp`
- Test: `core/tests/test_deck_motion.cpp`, `core/tests/test_drone_ship_config.cpp` (extend)

**Interfaces:**
- Consumes: Task 1 `environment::SeaState`, `WaveComponent`.
- Produces:
  ```cpp
  namespace starsim::config {
  struct DroneShipHull {            // DroneShipConfig::hull
      double length_m = 0.0, beam_m = 0.0, draft_m = 0.0;    // waterplane rectangle, draft = depth - freeboard [D]
      double roll_gyration_m = 0.0, pitch_gyration_m = 0.0;
      double heave_added_mass_factor = 0.0, roll_added_inertia_factor = 0.0, pitch_added_inertia_factor = 0.0;
      double heave_damping_ratio = 0.0, roll_damping_ratio = 0.0, pitch_damping_ratio = 0.0;
  };
  // DroneShipConfig gains: DroneShipHull hull; double max_deck_tilt_rad; double max_heave_rate_mps;
  }
  namespace starsim::contact {
  struct DeckPose {
      double heave_m = 0.0, roll_rad = 0.0, pitch_rad = 0.0;  // roll about the long axis (starboard down +),
      double heave_rate_mps = 0.0, roll_rate_radps = 0.0, pitch_rate_radps = 0.0;  // pitch about the beam (bow up +)
  };
  struct DeckFrame {  // the deck frame in the site frame (x east, y north, z up), origin at the deck centre
      math::Mat3 deck_to_site;          // columns: long axis (bow), port, deck normal
      math::Vec3 origin_site_m, velocity_site_mps, angular_velocity_site_radps;
  };
  struct DeckMotionStatistics { double significant_heave_m, significant_heave_rate_mps, significant_tilt_rad; };  // 2 sigma
  struct DeckNaturalPeriods { double heave_s, roll_s, pitch_s; };
  class DeckMotion {
  public:
      DeckMotion(const environment::SeaState& sea, const config::DroneShipHull& hull, double deck_heading_rad);
      DeckPose pose(double t_s) const;          // t_s: seconds after the sea's time origin
      DeckFrame frame(double t_s) const;        // rotation about the deck centre (call; spec non-goal: no surge/sway)
      DeckMotionStatistics statistics() const;  // spectral, from the components' complex responses
      DeckNaturalPeriods natural_periods() const;
  };
  // Per-unit-amplitude response of one regular wave (complex amplitudes of heave, roll, pitch), for tests and stats.
  struct DeckResponse { std::complex<double> heave_m, roll_rad, pitch_rad; };
  DeckResponse deck_response(const config::DroneShipHull& hull, double omega_radps, double wave_angle_rad);
  //   wave_angle_rad: wave travel direction relative to the long axis (0: from stern to bow)
  }
  ```

- [ ] **Step 1: Config values** in each `[[drone_ship]]` (all three ships): `hull_length_m = 91.4` [S:wiki_asds 300 ft],
  `hull_beam_m = 30.5` [S:wiki_asds 100 ft], `hull_depth_m = 6.1` [S:wiki_asds 20 ft] (loader derives draft =
  depth - freeboard = 3.1 m [D]; refuse draft <= 0), radii of gyration roll 0.35 x beam, pitch 0.25 x length [E],
  added-mass / damping values [E] from 2D strip theory of a shallow rectangle (document the basis in the TOML comment;
  heave added mass of a wide shallow barge is several times the displaced mass), `max_deck_tilt_deg = 3.0`,
  `max_heave_rate_mps = 1.0` [E] (Task 4 documents the Hs they correspond to). Verify the wiki figures when adding
  the source line; if the article differs, use its numbers.
- [ ] **Step 2: Write the failing tests** (tags `[deck_motion]`, config ones `[config][drone_ship]`):
  - Config loads hull and limits; draft 3.1 m; limits in radians.
  - `"Long waves lift and tilt the deck with the sea"`: `deck_response` at omega -> 0.05 omega_heave: |heave| within
    2 % of 1; pitch for a head wave within 2 % of k (the wave slope), roll ~0; beam wave: roll within 2 % of k.
  - `"Short waves average out over the hull"`: head wave with k L = 2 pi -> heave quasi-static factor 0 (|heave| <
    1e-3 x the long-wave value).
  - `"The analytic plane fit matches quadrature"`: for 5 (omega, angle) pairs the quasi-static heave / roll / pitch
    (exposed through `deck_response` with damping-free static path or a separate `deck_plane_fit`, builder's choice)
    equal a 200 x 200 midpoint quadrature of the least-squares plane over the waterplane within 1e-6 relative.
  - `"Natural periods follow the hydrostatics"`: `natural_periods()` equal 2 pi sqrt((1 + ca) d / g) (heave) and
    2 pi sqrt((1 + ca) r^2 / (g GM)) with GM = B^2 / (12 d) (roll), L^2 / (12 d) (pitch) (KG at the waterline:
    call, document) to 1e-9; at resonance |response| / quasi-static = 1 / (2 zeta) within 1e-6.
  - `"Deck statistics match the time series"`: Hs 2.5 m / Tp 8 s / beam-ish heading: 2 x std of `pose(t).heave_m`
    over 3600 s within 10 % of `statistics().significant_heave_m`; same for heave rate and tilt.
  - `"The deck frame is a rigid motion consistent with the pose"`: finite difference of `frame(t).origin_site_m` and
    of `deck_to_site` over 1e-4 s matches `velocity_site_mps` / `angular_velocity_site_radps` within 1e-5; calm sea ->
    identity at z = 0.
- [ ] **Step 3: Run** `-TestFilter "[deck_motion],[drone_ship]"` - expected FAIL (missing symbols).
- [ ] **Step 4: Implement.** Plane fit per component analytically (sinc-type factors of the rectangle), response
  H(omega) = 1 / (1 - (omega/omega_n)^2 + 2 i zeta omega/omega_n) per DOF, superposed over the components with their
  phases. Small-angle rotation composition for `deck_to_site` (roll then pitch about the heading-rotated axes; angles
  < ~10 deg).
- [ ] **Step 5: Run the filter** - PASS.
- [ ] **Step 6: Commit + push** `feat(contact): drone-ship deck motion from the sea state`.

### Task 3: Landing on the moving deck

**Files:**
- Modify: `core/include/starsim/sim/leg_landing_sim.hpp`, `core/src/sim/leg_landing_sim.cpp`,
  `core/include/starsim/vehicle/flight_body.hpp` + `.cpp` (moving landed hold),
  `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (sea state, flag, deck pose,
  `earth_leg_site`), `core/include/starsim/contact/landing_surface.hpp` (`Deck` gains `config::DroneShipHull hull`
  filled by `drone_ship_site`)
- Test: `core/tests/test_deck_landing.cpp`

**Interfaces:**
- Consumes: Task 2 `contact::DeckMotion`, `DeckFrame`, `DeckPose`; Task 1 `SeaState`.
- Produces:
  ```cpp
  // LegLandingSim
  void set_deck_motion(std::shared_ptr<const contact::DeckMotion> motion, time::Epoch sea_origin);  // null: fixed
  // FlightSession
  void set_sea_state(std::optional<environment::SeaStateConfig> sea);  // rebuilds the leg site's deck motion
  const std::optional<environment::SeaState>& sea_state() const;
  void set_deck_motion(bool on);   // fidelity flag, default on
  bool deck_motion() const;
  std::optional<contact::DeckPose> deck_pose() const;  // the landing zone's deck now (nullopt: no deck or no motion)
  // FlightBody: a landed hold whose pose is set each tick (rides a carrier)
  void move_landed_hold(const math::Vec3& pad_fixed_m, const math::Quat& attitude_fixed, const math::Vec3& velocity_fixed_mps);
  ```
  Sea time origin: the epoch of `set_sea_state` rounded down to a whole hour TDB [call] - deterministic per mission.

- [ ] **Step 1: Write the failing tests** (tags `[legs][deck_motion]`; a flown one over ~40 s gets `[slow]`):
  - `"A fixed deck gives exactly today's contact"`: the existing deck scenario of `test_falcon9_asds.cpp` ("A stage set
    down on the deck lands...") with `set_sea_state(SeaStateConfig{})` and with the flag off: identical touchdown
    result and final position bit for bit.
  - `"A stage set down on a heaving deck lands and rides it"`: Hs 2.0 m / Tp 8 s; stage dropped 1 m above the deck
    centre at 1 m/s relative sink, engines off -> `LegTouchdown::Landed`; afterwards for 120 s the foot points stay
    within 0.05 m of the deck plane from `deck_pose()` and the stage velocity equals the deck point velocity within
    0.01 m/s; also stepped through `advance_to` in 30 s chunks (time-warp path).
  - `"The touchdown is judged relative to the deck"`: on a deck rolling 3 deg a stage upright to the deck is not
    TippedOver; a stage at 20 deg to the deck normal is.
  - `"A pad landing ignores the sea state"`: a landing-zone (pad) site with Hs 3 m behaves as with no sea state.
- [ ] **Step 2: Run** `-TestFilter "[deck_motion],[legs]"` - expected FAIL.
- [ ] **Step 3: Implement.** In `before_tick`/`after_tick`, with a motion set: `pose_rel` = the site pose transformed
  into `DeckFrame` at the tick epoch (cg' = R^T (cg - o); v' = R^T (v - v_o - w_d x (cg - o)); w' = R^T (w - w_d);
  body_to_deck = R^T body_to_site), contact computed on the unchanged `LandingSurface::Deck`, force/torque rotated back
  with R. Touchdown judgement on `pose_rel`. On Landed: store the stage's pose in the deck frame; each tick convert it
  back through the deck frame and the site frame into Earth-fixed and call `move_landed_hold`. `earth_leg_site` builds
  the `DeckMotion` from the session's sea state and the deck's hull when the flag is on and the sea is not calm.
- [ ] **Step 4: Run** `-TestFilter "[deck_motion],[legs],[asds]"` - PASS (asds unchanged).
- [ ] **Step 5: Commit + push** `feat(sim): legs land on the moving drone-ship deck`.

### Task 4: Mission sea state and the onboard refusal

**Files:**
- Modify: `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp`
  (`std::optional<environment::SeaStateConfig> sea_state` from `[sea_state]`: `significant_wave_height_m`,
  `peak_period_s`, `heading_deg`, `seed`), `core/src/sim/mission_start.cpp` (every start calls
  `session.set_sea_state(mission.sea_state)` - nullopt resets), `core/include/starsim/sim/ascent_profile_planner.hpp`,
  `core/src/sim/ascent_profile_planner.cpp`
- Test: `core/tests/test_sea_state_refusal.cpp`

**Interfaces:**
- Consumes: Task 2 `DeckMotion::statistics`, `DroneShipConfig::max_*`; Task 3 `FlightSession::sea_state()`,
  `deck_motion()`.
- Produces: `std::optional<std::string> sea_state_refusal(const FlightSession& like, const config::DroneShipConfig& ship,
  double heading_rad)` (German reason or nullopt) in `ascent_profile_planner.hpp`.

- [ ] **Step 1: Write the failing tests** (`[asds][sea_state]`):
  - `[sea_state]` loads; Hs given without Tp, Tp 0, Hs < 0 -> load error naming the key.
  - `sea_state_refusal`: Hs 1.5 m / Tp 8 s -> nullopt; Hs 6 m / Tp 10 s -> reason starting
    "Seegang im Landegebiet zu hoch" containing the value and the limit; flag off -> nullopt.
  - `"A rough-sea drone-ship launch is refused before its drone-ship trials"`: the Starlink-class request (RTLS
    infeasible, as in `falcon9_asds.toml`) with Hs 6 m: `plan_launch` not feasible, reason from the sea state, the
    drone-ship trial count 0 (pre-trial check with the best deck heading over 0..180 deg in 5 deg steps; after the
    impact prediction the actual heading is checked again).
  - `"An RTLS launch ignores the sea state"`: CRS-35 class request with Hs 6 m -> LZ-40, feasible.
  - Mission with `[sea_state]` started, then `falcon9_asds` started in the same session -> `sea_state()` nullopt.
- [ ] **Step 2: Run** `-TestFilter "[sea_state],[asds]"` - FAIL.
- [ ] **Step 3: Implement**; document in `drone_ships.toml` and `docs/subsystems.md` which Hs (Tp 8 and 10 s, waves
  along and across the deck) the [E] limits correspond to.
- [ ] **Step 4: Run** `-TestFilter "[sea_state],[asds],[ascent_planner],[mission_config]"` (the last two only if those
  tags exist; check `core/tests`) - PASS, existing numbers unchanged.
- [ ] **Step 5: Commit + push** `feat(sim): onboard refusal of a drone-ship landing in rough sea`.

### Task 5: Demonstration mission flown

**Files:**
- Create: `config/missions/falcon9_asds_sea.toml` (copy of `falcon9_asds.toml` + `[sea_state]` Hs 2.0-2.5 m, Tp 8 s,
  heading [E] e.g. waves towards 60 deg, seed 1; display name "Medium Launcher: Drohnenschiff bei Seegang")
- Modify: `docs/subsystems.md` (new section "Sea state and deck motion": models, values, limits, flown numbers),
  `config/README.md` (the `[sea_state]` table and the hull keys), `CLAUDE.md` (environment/contact/sim lists)
- Test: `core/tests/test_falcon9_asds.cpp` (new case)

- [ ] **Step 1: Write the test** `"The drone-ship mission lands on a moving deck"` (`[slow][falcon9][asds][sea_state]`):
  start `falcon9_asds_sea` like the existing flown ASDS mission test, fly to the end: first stage `Landed`, touchdown
  speed relative to the deck <= 3 m/s, tilt to the deck <= 10 deg, print touchdown relative sink, deck heave rate at
  touchdown and the deck pose; 60 s after landing the stage still rides the deck (Task 3 criterion).
- [ ] **Step 2: Run** `-TestFilter "[asds]"`. If it fails on a hard touchdown, apply spec goal 4's smallest fix
  (terminal descent referenced to the measured deck height) and document it; otherwise no guidance change.
- [ ] **Step 3: Docs** as listed; numbers from the run.
- [ ] **Step 4: Commit + push** `feat(config): drone-ship mission in a moderate sea`.

### Task 6: Game - drone ship, waves, readout

**Files:**
- Modify: `bindings/src/flight_snapshot.cpp` (landing-zone dict: `sea_state` {`hs_m`, `tp_s`, `heading_deg`} when set,
  `deck_heave_m`, `deck_roll_deg`, `deck_pitch_deg`, `hull_length_m`, `hull_beam_m`, `hull_draft_m`),
  `bindings/src/flight_sim.cpp` if a setter is needed for the flag (`set_deck_motion`)
- Create: `game/world/drone_ship_view.gd` (procedural boxes: hull 91.4 x 30.5 m from deck - freeboard - draft to deck,
  deck slab 91 x 52 m, 2 m walls along the long sides, matte grey; placed like `tower_view.gd` at the landing zone,
  posed by heave / roll / pitch about the deck centre)
- Modify: `game/world/world.gd` (create / update the view while the landing zone has a deck),
  `game/world/terrain/water_waves.gd` (`set_sea_state(hs_m, tp_s, heading_deg)`: dominant wavelength g Tp^2 / 2 pi for
  the first octave, the others at the existing ratios, slopes scaled so the resolved slope variance follows Hs; no
  sea state -> today's calm set), `game/ui/time_panel.gd` or `ui/flight_hud.gd` (German line "Seegang Hs %.1f m, Tp
  %.0f s - Deck: Hub %+.2f m, Rollen %+.1f°, Stampfen %+.1f°"), `game/tests/smoke_test.gd` (keys present for
  `falcon9_asds_sea`, absent / calm for `falcon9_asds`), `docs/ansichten.md` (the readout)
- [ ] **Step 1: Smoke-test assertions first** (fail), then bindings + game code.
- [ ] **Step 2: Build with bindings** (`tools/build.ps1 -TestFilter "[sea_state]"`) and run the smoke test:
  `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd`
  - expected all checks OK.
- [ ] **Step 3: Visual check** with `render_check.gd` (GPU) at the `falcon9_asds_sea` deck after landing (or a direct
  mission-placed view if render_check cannot fly it: add a `--mission=` option only if cheap), read the PNG: ship on
  the water, stage standing on the deck, no gap.
- [ ] **Step 4: Commit + push** `feat(game): drone ship riding the sea state`.

### Task 7: Final review

- [ ] Whole-plan review (fresh reviewer) against the spec; fix wave; affected tags green; update memory
  (`project-milestone-order`) and ask the user for the VR check (Start-VR.cmd: `falcon9_asds_sea` landing in the
  external view) and `/clear`.
