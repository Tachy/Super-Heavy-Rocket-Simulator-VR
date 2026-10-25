# Follow-ups (F1-F8) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the approved follow-ups: parallel filtered tests, webcast gauges from TOML, planning flag only on pad
starts, one stack builder without the FlightSession <-> planner cycle, a persistent test plan cache, drone ship chosen
from a list, the Falcon 9 RCS flip from researched data, and the cross-flow normal force in the landing planner model.

**Architecture:** Each task is one follow-up with its own tests and commit on `main` (no branches), pushed when its
affected tests are green. Core changes stay in their modules (`sim`, `gnc`, `config`); the game part is GDScript only
reading config; `tools/build.ps1` changes only how tests are launched.

**Tech Stack:** C++20 core (Catch2, CTest via `catch_discover_tests ... ADD_TAGS_AS_LABELS`), GDScript (Godot 4.7
double build, headless test scripts), PowerShell 5.1 build script, TOML config.

**Spec:** `docs/superpowers/specs/2026-10-23-follow-ups-design.md`

## Global Constraints

- CLAUDE.md rules: core without Godot; bindings thin; config not code; SI units; frame/unit in names; no magic numbers
  (constants with source or `[E]/[D]/[S]` tags in config); small files, one responsibility.
- Code, comments, commits in English; UI strings German; answer the user in German.
- Conventional Commits; commit and `git push origin main` after each task with its affected tests green.
- Tests: only the tags the change affects, `[slow]` cases of those tags included. Many cases / slow ones: build with
  `-NoTests`, then `ctest --test-dir build -j 24 -L "^(tag1|tag2)$"` (after Task 1: `-TestFilter` does that). A run
  longer than ~10 min goes to the background with output in a file; check every 10 min; stop early on failures.
- Test cases must not share temp files - the only exception is Task 5's plan cache (user-approved 2026-10-23).
- Physics first: no tuning to webcast gauges that the physics cannot explain.
- `docs/subsystems.md` updated with every physics change (Tasks 5, 6, 7, 8); every changed reference number re-baselined
  there with the old and new value.
- Game tests: `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/<name>.gd`.

## Review Focus

1. A `-TestFilter` tag that no test carries: ctest finds 0 tests and exits 0 - the script must say "no test matched"
   and fail, not report success (Task 1, Step 3 check).
2. `webcast_gauges.toml` missing or with an unknown gauge/condition name: the overlay must still run (error logged, that
   gauge never shown), not crash or show nothing (Task 2 test `_test_config_errors`).
3. A stale plan cache after a config edit: a cached plan from before the edit must not be reused (Task 5 test
   "plan cache key changes with the config contents").
4. A corrupt / truncated cache file (killed process mid-write is avoided by rename, but a hand-edited or old-format
   file): ignored and re-planned (Task 5 test "corrupt cache file is ignored").
5. A drone-ship launch whose impact lies outside every ship's area: refused with a German reason, not sent to the
   default ship (Task 6 test "impact outside every service area is refused").

---

### Task 1: `-TestFilter` runs tags in parallel through ctest (F1)

**Files:**
- Modify: `tools/build.ps1` (param block comment lines 1-7, test step lines 31-40)
- Modify: `CLAUDE.md` (Build & test: the `-TestFilter` serial note)

**Interfaces:**
- Produces: `tools/build.ps1 -TestFilter "<tags>"` = `ctest -L "^(t1|t2)$" -j $Jobs` (+ `-LE slow` with `-Fast`).
  Used by every later task.

- [ ] **Step 1: Implement the tag-list translation in `tools/build.ps1`**
  - A spec is a pure tag list when it matches `^(\[[A-Za-z0-9_.-]+\])+(,(\[[A-Za-z0-9_.-]+\])+)*$`. Every tag in it
    (both `[a][b]` and `[a],[b]`) joins the union: `-L "^(a|b)$"` (regex-escape each tag). Document in the header
    comment that `[a][b]` is a union here (unlike Catch's AND).
  - The tag `plan_speed` maps to its own CTest label already (`core/tests/CMakeLists.txt:200-201`, `RUN_SERIAL`); no
    special case.
  - Anything else (names, wildcards, `~`): the serial exe as now, with `Write-Host "TestFilter is not a tag list: running the test exe serially"`.
  - The ctest step writes its output to `build/ctest_filter.log` (`--output-on-failure`, `-O <log>` or redirect), and
    the script prints the summary: the `tests passed, N tests failed` line and the failed names (`ctest` prints them
    under "The following tests FAILED"). Exit code non-zero when any test failed.
  - ctest's "No tests were found!!!" (0 tests) is a failure: `throw "TestFilter matched no test"`.

- [ ] **Step 2: Verify the parallel path**
  Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[cr3bp]"`
  Expected: ctest runs 3 tests in parallel, summary `100% tests passed, 0 tests failed out of 3`.

- [ ] **Step 3: Verify union, the miss and the fallback**
  - `-TestFilter "[cr3bp],[staging]"` -> 8 tests (3 + 5), all passed.
  - `-TestFilter "[no_such_tag]"` -> the script fails with "TestFilter matched no test".
  - `-TestFilter "Kepler*"` -> the serial-exe notice, the exe runs.

- [ ] **Step 4: Update CLAUDE.md** - replace the bullet "`-TestFilter` runs the test exe serially in one process: ..."
  with: `-TestFilter "[a],[b]"` runs the union of the tags as parallel ctest processes (`[slow]` included unless
  `-Fast`); a spec that is not a tag list runs the exe serially. Update the memory note `project-external-view.md`
  follow-up 3 as done.

- [ ] **Step 5: Commit and push**
  `git commit -m "chore(build): -TestFilter runs tag lists as parallel ctest labels"`; `git push origin main`.

---

### Task 2: Webcast gauges from `config/ui/webcast_gauges.toml` (F2)

**Files:**
- Modify: `game/util/toml_lite.gd` (single-line arrays of strings/numbers/booleans)
- Create: `game/tests/toml_lite_test.gd`
- Create: `config/ui/webcast_gauges.toml`
- Create: `game/ui/webcast/gauge_config.gd` (`class_name GaugeConfig`)
- Modify: `game/ui/webcast/gauge_rules.gd` (constants -> config; conditions by name)
- Modify: `game/ui/webcast/webcast_overlay.gd:19-38` (`GAUGES` const removed, read from config), lines 70-74, 194, 231
- Modify: `game/tests/webcast_rules_test.gd`, `game/tests/webcast_overlay_test.gd:124-128`

**Interfaces:**
- Produces:
  - `TomlLite.parse(text)` also returns `Array` for `key = ["a", 1.0, true]` (one line, no nesting).
  - `GaugeConfig.load_file(path: String) -> GaugeConfig`; `GaugeConfig.shared() -> GaugeConfig` (lazy, from
    `Paths.config_dir().path_join("ui/webcast_gauges.toml")`).
  - Fields: `hide_delay_s: float`, `max_per_side: int`, `thresholds: Dictionary` (name -> float),
    `gauges: Dictionary` (id -> {label, unit, full_scale, decimals, log_min?, signed?, condition}),
    `left: Array[String]`, `situations: Dictionary` (docking/transfer/orbit/default -> Array[String]),
    `errors: Array[String]` (load problems, each also `push_error`ed).
  - `GaugeRules.select(telemetry, body, state, now_s, config: GaugeConfig = null) -> Dictionary` (null = shared).
  - `GaugeRules.CONDITIONS: Array[String]` - the names the TOML may use: `always`, `has_engines`, `has_propellant`,
    `heating`, `powered_or_aero`, `continuum_aero`, `aero`, `has_moon`, `low_altitude`, `in_orbit`, `docking`.

- [ ] **Step 1: Write the failing TomlLite test** `game/tests/toml_lite_test.gd` (SceneTree script like
  `webcast_rules_test.gd`): `parse('a = ["x", "y"]\nb = [1.5, 2]\nc = []\n[t]\nd = "q # not a comment"')` gives
  `a == ["x","y"]`, `b == [1.5, 2.0]`, `c == []`, `t.d == "q # not a comment"`; a string containing a comma inside an
  array: `e = ["a,b", "c"]` -> `["a,b", "c"]`. Prints `TOML LITE TEST PASSED`.

- [ ] **Step 2: Run it** - expected FAIL (arrays parse as 0.0).

- [ ] **Step 3: Implement arrays in `TomlLite._value`** (split on commas outside quotes, each element through `_value`).
  Run the test: PASS.

- [ ] **Step 4: Write `config/ui/webcast_gauges.toml`** carrying today's values exactly (comments keep the `[E]/[D]`
  reasoning from `gauge_rules.gd`):
  - `[rules] hide_delay_s = 5.0`, `max_per_side = 4`.
  - `[thresholds]`: `dynamic_pressure_min_pa = 100.0`, `knudsen_continuum_max = 0.1`, `heat_rate_min_w_m2 = 1.0e3`,
    `vertical_speed_max_altitude_m = 150.0e3`, `orbit_min_altitude_m = 100.0e3`, `transfer_min_altitude_m = 5.0e6`,
    `transfer_min_apoapsis_km = 100.0e3`, `transfer_exit_factor = 0.8`.
  - `[left] gauges = ["engines", "propellant", "heat_flux", "heat_load", "g_load", "mach", "dynamic_pressure"]`.
  - `[situation.docking]`, `[situation.transfer]`, `[situation.orbit]`, `[situation.default]` each `gauges = [...]`
    with the four lists of `gauge_rules.gd:111-118`.
  - `[gauge.<id>]` for every id of `webcast_overlay.gd:21-38` plus `engines` (engine gauge: label/unit unused, only
    `condition`): label, unit, full_scale, decimals, optional log_min, optional signed, and `condition` =
    engines `has_engines`, propellant `has_propellant`, heat_flux/heat_load `heating`, g_load `powered_or_aero`,
    mach `continuum_aero`, dynamic_pressure `aero`, distance_moon/speed_moon `has_moon`, vertical_speed `low_altitude`,
    apoapsis/periapsis `in_orbit`, range/closing_rate `docking`, all others `always`.
  - Header comment: the file is edited by a later settings wizard; situations are chosen in code (docking > transfer >
    orbit > default), conditions are named rules in `gauge_rules.gd`.

- [ ] **Step 5: Write the failing config tests** in `game/tests/webcast_rules_test.gd`:
  - `_test_config_matches_defaults`: `GaugeConfig.shared()` has `errors.is_empty()`, `max_per_side == 4`,
    `hide_delay_s == 5.0`, `situations["orbit"] == ["speed_inertial", "altitude", "apoapsis", "periapsis"]`,
    `gauges["altitude"]["log_min"] == 0.1`.
  - `_test_config_errors`: `GaugeConfig` parsed from a text with `[gauge.foo] condition = "nonsense"` and
    `[left] gauges = ["engines", "bar"]` has 2 errors; `GaugeRules.select(..., cfg)` returns without `foo`/`bar` and
    still shows `engines`. A missing file gives one error and an empty selection (no crash).
  - The existing `_test_gauges` cases stay as they are and must pass on the TOML.
  Add `GaugeConfig.parse(text: String) -> GaugeConfig` for these tests (`load_file` reads and calls it).

- [ ] **Step 6: Run** `webcast_rules_test.gd` - expected FAIL (no GaugeConfig).

- [ ] **Step 7: Implement `gauge_config.gd` and switch `gauge_rules.gd` / `webcast_overlay.gd` to it.** Conditions
  stay GDScript (one function computing a name -> bool dict from telemetry/body/state/config); the situation choice
  and `in_transfer` hysteresis use the config thresholds. `webcast_overlay.gd` takes label/unit/full scale/decimals/
  log_min/signed from `GaugeConfig.shared().gauges`. Remove the old constants.

- [ ] **Step 8: Run** `webcast_rules_test.gd`, `webcast_overlay_test.gd` (its `GAUGES` loop now over
  `GaugeConfig.shared().gauges`), `toml_lite_test.gd`, `smoke_test.gd` - all PASSED.

- [ ] **Step 9: Commit and push**
  `git commit -m "feat(game): webcast gauge selection from config/ui/webcast_gauges.toml"`. Mark memory follow-up 2 done.

---

### Task 3: Planning flag only for missions that plan on the pad (F7)

**Files:**
- Modify: `core/include/starsim/config/mission_config.hpp` (next to `mission_kind`), `core/src/config/mission_config.cpp`
- Modify: `bindings/src/flight_sim.cpp:42-54` (`may_plan_ascent` removed), `:212-220`
- Test: `core/tests/test_mission_config.cpp` (or the file that tests `mission_kind`; tag `[mission_config]`)
- Modify: `game/tests/smoke_test.gd` (around line 137-147)

**Interfaces:**
- Produces: `bool config::mission_plans_on_pad(const std::filesystem::path& config_dir, std::string_view mission_id);`
  - true: kind Ascent, IssLaunch, DepotLaunch; TankerFlight / HlsRefuel with `[start] mode = "pad"`;
    RefuellingCampaignFlown with `[depot] start = "launch"`.
  - false: every other kind, `mode = "phasing_orbit"`, `start = "placed"`.

- [ ] **Step 1: Failing test** `"mission_plans_on_pad: pad starts only", "[mission_config]"`:
  true for `leo_ascent`, `falcon9_rtls` (or any ascent file), an iss_launch file, `depot_launch`, `tanker_flight`,
  `hls_refuel`, `refuelling_campaign_flown`; false for `hls_refuel_orbit`, `tanker_flight_orbit`,
  `refuelling_campaign_flown_placed`, a ship_return and a lunar mission file. (Check the actual file ids in
  `config/missions/`; use the mission loader's own parsing of `[start]` / `[depot]`, no second parser.)
- [ ] **Step 2: Run** `-TestFilter "[mission_config]"` - FAIL (undefined).
- [ ] **Step 3: Implement** in `mission_config.cpp`; in `FlightSim::start_mission` replace
  `may_plan_ascent(config::mission_kind(...))` with `config::mission_plans_on_pad(config_dir_, id)`.
- [ ] **Step 4: Smoke test** - add: `start_mission("tanker_flight_orbit", ...)` never shows
  `autopilot_status()["planning"] == true` while its result is pending (poll as the existing pad check does).
- [ ] **Step 5: Run** `-TestFilter "[mission_config]"` (full build with bindings) and `smoke_test.gd` - PASS.
- [ ] **Step 6: Commit and push** `fix(game): planning flag only for missions that plan their ascent on the pad`.

---

### Task 4: One ascent stack builder; FlightSession no longer includes the planner (F5)

**Files:**
- Modify: `core/include/starsim/sim/ascent_mission.hpp:14`, `core/src/sim/ascent_mission.cpp:34-73`
- Modify: `core/include/starsim/sim/flight_session.hpp:402-410, ~1141` (pad accessors; remove
  `std::string engage_ascent_autopilot(const gnc::AscentTarget&)`), `core/src/sim/flight_session.cpp:13, 122-147`
- Modify: `core/include/starsim/sim/ascent_profile_planner.hpp`, `core/src/sim/ascent_profile_planner.cpp:92-96`
- Modify: `bindings/src/flight_sim.cpp:175`, the 6 test files calling `engage_ascent_autopilot(target)`
  (`grep -rln engage_ascent_autopilot core/tests`)

**Interfaces:**
- Produces:
  - `struct AscentStackSpec { double payload_kg = 0.0; bool fairing = true; std::optional<config::MissionPayload> payload; std::map<std::string, double> propellant_load_kg; };`
    and `config::VehicleConfig build_ascent_stack(config::VehicleConfig vehicle, const std::filesystem::path& config_dir, const AscentStackSpec& spec);`
    in `ascent_mission.hpp` - the only place that sets payload mass, drops the fairing, fills tanks
    (`propellant_load_kg`: stage id -> kg, the existing `load_propellant` rules incl. its capacity error) and stacks the
    payload. `ascent_stack(...)` removed; its callers use it.
  - `std::string engage_pad_ascent(FlightSession& session, const gnc::AscentTarget& target);` in
    `ascent_profile_planner.hpp` - the body of today's `FlightSession::engage_ascent_autopilot(target)`, unchanged in
    behaviour; reads the pad through new const accessors on FlightSession: `const std::string& pad_vehicle_id() const`,
    `const std::optional<config::VehicleConfig>& pad_stack() const`, `double pad_heading_rad() const`,
    `const LaunchSite& launch_site() const` (only those not existing yet).
  - `flight_session.cpp` no longer includes `ascent_profile_planner.hpp`; `ascent_profile_planner.hpp` still includes
    `flight_session.hpp` (one direction).

- [ ] **Step 1: Failing test** `"build_ascent_stack: the one builder", "[ascent]"` in `core/tests/test_ascent_mission.cpp`
  (or the existing ascent-mission test file): for falcon9_block5 with payload_kg 5000, fairing false, a propellant load
  of half the first stage, and a payload vehicle (the Crew Dragon id used by the ISS launch missions): the result's
  upper-stage payload mass 5000, fairing mass 0, the first stage's propellant equals the load, the top stage is the
  payload's; a load above capacity throws `std::invalid_argument`.
- [ ] **Step 2: Run** `-TestFilter "[ascent]"` - FAIL (undefined).
- [ ] **Step 3: Implement** `build_ascent_stack`, use it in `start_ascent_mission` and `request_stack`; move
  `engage_ascent_autopilot(target)` out as `engage_pad_ascent`; update bindings and tests.
- [ ] **Step 4: Verify the cycle is gone:** `grep -n ascent_profile_planner core/src/sim/flight_session*.cpp
  core/include/starsim/sim/flight_session.hpp` prints nothing.
- [ ] **Step 5: Verify behaviour unchanged:** `-TestFilter "[ascent],[ascent_planner],[staging],[falcon9],[rtls],[asds]"`
  (with `[slow]`) - all pass with the same numbers as before (memo keys unchanged: `memo_key` is not touched).
  Smoke test passes (bindings build).
- [ ] **Step 6: Commit and push** `refactor(sim): one ascent stack builder; the pad ascent planned outside FlightSession`.
  Mark the memory follow-up done.

---

### Task 5: Persistent plan cache for test runs (F6)

**Files:**
- Create: `core/include/starsim/sim/ascent_plan_cache.hpp`, `core/src/sim/ascent_plan_cache.cpp`
- Modify: `core/src/sim/ascent_profile_planner.cpp:423-438` (`plan_ascent_profile`), `core/src/CMakeLists.txt` (new file)
- Modify: `core/tests/CMakeLists.txt:198-201` (environment + POST_BUILD cache wipe)
- Create: `core/tests/test_ascent_plan_cache.cpp` (tag `[plan_cache]`)
- Modify: planner tests that assert `trial_flights > 0` (in `test_ascent_profile_planner.cpp`, `[plan_speed]`)
- Modify: `docs/subsystems.md` (test cost section, ~line 2261), CLAUDE.md (Quality rules: the temp-file exception)

**Interfaces:**
- Consumes: `memo_key(like, request)` (`ascent_profile_planner.cpp:232`), `AscentProfile` (planner header).
- Produces (`namespace starsim::sim`):
  - `std::optional<std::filesystem::path> plan_cache_dir();` - from env `STARSIM_PLAN_CACHE_DIR`; nullopt when unset,
    empty, or disabled.
  - `void disable_plan_cache();` - process-wide off switch for tests that must fly trials.
  - `std::string config_contents_hash(const std::filesystem::path& config_dir);` - FNV-1a 64 (hex) over the sorted
    relative paths + bytes of every regular file under `config_dir`; computed once per config dir per process.
  - `std::string encode_profile(const AscentProfile&);` / `std::optional<AscentProfile> decode_profile(std::string_view);`
    - a versioned line format (`starsim-plan-cache 1` first line), doubles as hexfloat (`%a`) so a round trip is
    bit-exact; any parse error -> nullopt.
  - `std::optional<AscentProfile> load_cached_plan(const std::string& key);` and
    `void store_cached_plan(const std::string& key, const AscentProfile& profile);` - file name = FNV-1a 64 hex of
    (key + '|' + config hash); store writes `<name>.<pid>.<counter>.tmp` then `std::filesystem::rename` to `<name>.plan`;
    every filesystem error is swallowed (cache is an optimisation); the file stores the full key and load checks it
    (hash collision -> miss).
  - In `plan_ascent_profile`: in-process memo first, then the disk cache (hit -> also into the memo,
    `trial_flights = 0`), then the search (store in both). `request.stack` requests stay uncached.
- CMake: `catch_discover_tests(... PROPERTIES ENVIRONMENT "STARSIM_PLAN_CACHE_DIR=${CMAKE_BINARY_DIR}/plan_cache")`;
  the `plan_speed` test gets no such environment; a POST_BUILD command on `starsim_core_tests` removes
  `${CMAKE_BINARY_DIR}/plan_cache` (every relink invalidates all plans: code changes never reuse an old plan).

- [ ] **Step 1: Failing tests** in `test_ascent_plan_cache.cpp` (`[plan_cache]`), each with its own temp dir
  (unique name per test case):
  - `"encode/decode round trip is bit-exact"`: a profile with kick 0.2617993877991494, propellant 123456.789, a
    DroneShip recovery with lat/lon/heading and a booster_impact; decode(encode(p)) equals p field by field with `==`.
  - `"corrupt cache file is ignored"`: decode of `""`, of the header only, of a truncated encode -> nullopt;
    `load_cached_plan` on a file with garbage -> nullopt.
  - `"store then load through the directory"` (set the env var for the test via `_putenv_s` and restore it): store,
    load -> equal; a different key -> nullopt.
  - `"plan cache key changes with the config contents"`: copy a 2-file mini config into a temp dir, hash, change one
    byte, hash again -> different; renaming a file -> different.
- [ ] **Step 2: Run** `-TestFilter "[plan_cache]"` - FAIL (undefined).
- [ ] **Step 3: Implement** the cache, hook it into `plan_ascent_profile`, CMake environment + POST_BUILD wipe; call
  `disable_plan_cache()` at the start of the planner tests that assert trial flights (find them:
  `grep -n "trial_flights" core/tests`).
- [ ] **Step 4: Run** `-TestFilter "[plan_cache],[ascent_planner],[plan_speed]"` - PASS.
- [ ] **Step 5: Measure:** with a freshly built exe (empty cache), run the ascent-related labels
  `ctest --test-dir build -j 24 -L "^(ascent|ascent_planner|staging|falcon9|rtls|asds|iss_launch|depot)$"` twice
  (labels: check `ctest -N --print-labels`), record CPU/wall of both runs (`Measure-Command`) and the sum of per-test
  times from ctest's output; the second run must be clearly cheaper and the first no worse than before Task 5 minus
  the duplicate plans. Record the numbers in `docs/subsystems.md` (replace the "+1700 s CPU" paragraph's open point).
- [ ] **Step 6: Bit-identity check:** one mission test with numbers (e.g. the RTLS mission's landing-burn mass) gives
  the same printed values with `STARSIM_PLAN_CACHE_DIR` unset and set (warm cache) - run it both ways
  (`STARSIM_VERBOSE=1`) and diff.
- [ ] **Step 7: Docs** - CLAUDE.md "ctest runs test cases as parallel processes: tests must not share temp files" gets
  "... (exception: the write-once plan cache `build/plan_cache`, Task 5 of the 2026-10-23 follow-ups)".
- [ ] **Step 8: Commit and push** `perf(test): plans shared across test processes through a write-once cache`.

---

### Task 6: Drone ship chosen from a list (F8)

**Files:**
- Modify: `config/drone_ships.toml`, `core/include/starsim/config/drone_ship_config.hpp`,
  `core/src/config/drone_ship_config.cpp`
- Modify: `core/include/starsim/sim/ascent_profile_planner.hpp:29-31` (`default_drone_ship_id` removed),
  `core/src/sim/ascent_profile_planner.cpp:487-501`
- Test: `core/tests/test_drone_ship_config.cpp` (create if absent, tag `[drone_ship]`), planner test `[asds]`
- Modify: `docs/subsystems.md:2160-2175`

**Interfaces:**
- Produces:
  - `DroneShipConfig` gains `std::string basin;` (`"atlantic"` / `"pacific"`), `double home_port_latitude_rad`,
    `double home_port_longitude_rad`, and a service area `double area_min_latitude_rad, area_max_latitude_rad,
    area_min_longitude_rad, area_max_longitude_rad` (a lat/lon box, longitudes in (-pi, pi]).
  - `std::vector<DroneShipConfig> load_drone_ships(const std::filesystem::path& config_dir);` (all, file order).
  - `std::optional<DroneShipConfig> drone_ship_for(const std::vector<DroneShipConfig>& ships, double latitude_rad, double longitude_rad);`
    - the ship whose area contains the point; several: the nearest home port (great-circle); none: nullopt.
- Config (ids stable: `asds_atlantic` keeps its id - missions reference it):
  - `asds_atlantic` and `asds_atlantic_2`: basin atlantic, home port Port Canaveral 28.41 N, 80.62 W [S]; area box
    [E] 20 N .. 45 N, 82 W .. 40 W (off the US east coast to mid-Atlantic, covers Cape launches' downrange).
  - `asds_pacific`: basin pacific, home port Port of Long Beach 33.75 N, 118.22 W [S]; area [E] 0 N .. 40 N,
    150 W .. 115 W.
  - Same deck/freeboard as today; `name = "Drohnenschiff"` for all (generic display names); sources in the header.
- `plan_launch`: the provisional station's ship = `drone_ship_for(ships, provisional lat, lon)` (none -> refused
  "Kein Drohnenschiff für dieses Seegebiet."); after the trial the ship = `drone_ship_for` at the predicted impact
  (none -> same refusal). The memo key carries the provisional ship id (unchanged format).

- [ ] **Step 1: Failing tests** (`[drone_ship]`): `load_drone_ships` returns 3 ships, `asds_atlantic` first;
  `drone_ship_for` at 30.5 N / 74 W -> an atlantic ship (`asds_atlantic`: same home port, file order breaks the tie);
  at 30 N / 125 W -> `asds_pacific`; at 10 S / 20 E -> nullopt.
- [ ] **Step 2: Failing test** `"impact outside every service area is refused", "[asds]"`: `plan_launch` for falcon9
  from a site config copied to a temp config dir with the site moved to 35 S / 18 E (no landing zone) -> not
  feasible, reason contains "Kein Drohnenschiff". (Planning stops before trials when the provisional station has no
  ship: the test is fast.)
- [ ] **Step 3: Run** `-TestFilter "[drone_ship],[asds]"` - FAIL.
- [ ] **Step 4: Implement.**
- [ ] **Step 5: Run** `-TestFilter "[drone_ship],[asds],[falcon9],[ascent_planner]"` (with `[slow]`) - PASS, the drone-ship
  missions' numbers unchanged (Cape launches still pick `asds_atlantic`).
- [ ] **Step 6: Docs + commit + push** `feat(sim): the drone ship is chosen by the stage's impact from config/drone_ships.toml`.

---

### Task 7: Falcon 9 RCS flip from researched data, boostback loss re-identified (F3)

**Files:**
- Modify: `config/vehicles/falcon9_block5.toml` (`[stage.rcs]` ~lines 180-196, header masses ~line 20,
  `[stage.recovery]` `flip_duration_s` line 133, `boostback_loss_fraction` line 139)
- Modify: tests with Falcon 9 return numbers (`grep -ln "falcon9" core/tests/test_falcon9_*.cpp core/tests/test_booster_return_config.cpp`)
- Modify: `docs/subsystems.md` ("Open discrepancy - Falcon 9 flip time" ~1903-1912, "Re-baselined" ~1926)

**Interfaces:** config only; no code API changes. If a code change looks necessary, stop (Step 3).

- [ ] **Step 1: Research** (web; record every source with URL and what it states): Falcon 9 first-stage cold-gas
  thrusters - count, pod layout, thrust per thruster; nitrogen supply (COPV count/volume/pressure or mass); flip timing
  from separation to boostback ignition on at least three RTLS webcasts (CRS-35 plus two more; times from the
  webcast clock); whether the boostback lights before the turn is complete; pitch inertia of an empty stage if
  published. Write the findings into `docs/subsystems.md` under the flip discrepancy (table: quantity, value, source,
  tag).
- [ ] **Step 2: Derive the data.**
  - Sourced values -> `[S:<id>]` in the TOML with `[sources]` entries.
  - Missing thrust (or count) -> `[D]` from the mean measured flip time: the bang-bang turn over the flip angle the
    stage flies (sim: ~165 deg) at the torque that gives that time, with the existing `return_controller_` and the
    15 deg boostback pointing gate.
  - N2 supply -> covers flip + coast trims + landing of the full RTLS with a documented margin (>= 20 % [E]) at Isp
    70 s; the extra N2 mass goes into the stage's dry-mass bookkeeping so the identified total stage mass is unchanged
    (comment the split).
- [ ] **Step 3: Gate.** If the research shows a mechanism the model lacks (e.g. boostback lit mid-turn, engines
  helping the flip), stop: record it in `docs/subsystems.md` as an open point, commit the research, and report to the
  controller for the user's decision. Do not implement it.
- [ ] **Step 4: Fly the full RTLS** (`-TestFilter "[rtls]"` incl. `[slow]`, `STARSIM_VERBOSE=1`): the separation ->
  boostback time within 3 s of the measured mean; N2 left > 0 at touchdown.
- [ ] **Step 5: Re-identify `boostback_loss_fraction`** on that run: the landing-burn mass matches the plan (~12.1 t
  planned today) within 0.2 t; update `flip_duration_s` [D] to the flown flip. Iterate Steps 4-5 (each change of the
  fraction re-plans).
- [ ] **Step 6: Re-baseline** every Falcon 9 test number that moves (`[falcon9]`, `[rtls]`, `[asds]`,
  `[booster]`, `[staging]` incl. `[slow]`) with old -> new in `docs/subsystems.md`; tolerances only tightened or kept.
- [ ] **Step 7: Commit and push** `fix(config): Falcon 9 RCS from researched data, flip and boostback loss re-identified`.

---

### Task 7b: Why the Falcon 9 boostback costs too little (added 2026-10-23, user decision)

Task 7's real 13.6 s flip exposed it: the planner keeps 53.6 t at MECO for the RTLS return (CRS-35 webcast ~66 t), so
MECO comes late and fast (+7.6 % MECO speed vs the webcast; the CRS-35 test's allowance was raised to ~9.6 % as a
documented interim). Our boostback burns ~43 s, CRS-35's ~55 s, ending in a similar state.

**Files:** to be found - `core/src/gnc/booster_return*.cpp`, `booster_return_prediction.cpp` (the MECO plan's return
cost: run-down, flip, boostback by the rocket equation on the Lambert arc x (1 + boostback_loss_fraction)), the
Falcon 9 `[stage.recovery]` config, `docs/subsystems.md`, the CRS-35 ascent test.

- [ ] **Step 1: Diagnose** (no code change yet): compare our RTLS return against CRS-35 phase by phase - MECO state
  (speed, altitude, flight-path angle, downrange), flip end state, boostback engine count / throttle / duration /
  delta-v / end state, entry burn, landing burn - using the webcast tables already in the repo (tools/telemetry,
  the CRS-35 fixtures) and `STARSIM_VERBOSE` / `STARSIM_DEBUG_*` output. Name the phase(s) where our propellant use
  falls short of the flight and the physical cause (e.g. boostback engine count/throttle, gravity loss modelling,
  the Lambert arc's target, aero during the boostback, MECO state). Record the comparison table in
  `docs/subsystems.md`.
- [ ] **Step 2: Gate.** If the cause needs a guidance/model change, write the proposed change (what, why, expected
  effect on the MECO propellant) into the report and STOP for the user's decision. A pure data correction with a
  source (e.g. the boostback engine count) may proceed.
- [ ] **Step 3: Fix** (after approval or when data-only), re-identify `boostback_loss_fraction` again, tighten the
  CRS-35 `meco_speed_allowance` back towards 3 % (measured gap + 2 %), re-baseline the moved Falcon 9 numbers
  (`[falcon9]`, `[rtls]`, `[asds]`, `[booster]`, `[staging]`, `[iss_mission]` incl. `[slow]`).
- [ ] **Step 4: Commit and push** `fix(gnc): Falcon 9 boostback cost matches the flights` (or `docs:` if only the
  diagnosis lands).

---

### Task 7c: Falcon 9 late-ascent thrust against stack mass (added 2026-10-23, user decision)

Task 7b's diagnosis (docs/subsystems.md "Falcon 9 boostback against CRS-35", commit 847b3e1): the boostback is not
the cause. The CRS-35 MECO speed gap (+7.6 %) comes from the ascent: near MECO our stack accelerates ~6 % harder
(37.8 vs 35.7 m/s2) - the flight's stack is ~13 t heavier or has ~6 % less thrust; our MECO is 0.9 s late (+1.7 %).
The first stage's propellant at MECO is bounded to 54-61 t by the boostback fit, so extra mass would be in the upper
stack (payload 12.5 t [E], second-stage masses [D]) - or the first stage's late thrust is lower.

**Files:** to be found - `config/vehicles/falcon9_block5.toml`, the CRS-35 mission config (payload), Merlin engine
config, `tools/telemetry/*` (webcast tables, `boostback_fit.py`), `core/tests/test_falcon9_ascent.cpp`,
`docs/subsystems.md`.

- [ ] **Step 1: Diagnose** (no change yet): compare our CRS-35 ascent with the webcast from T+100 s to MECO
  (acceleration, speed, altitude at the same true time - mind the dial lags from the ascent milestone), and identify
  which of (a) upper-stack mass (CRS-35 Cargo Dragon launch mass + trunk, second-stage dry/propellant), (b) first-stage
  thrust / throttle near MECO (Merlin 1D sea-level/vacuum thrust, a throttle-down before MECO on the flight),
  (c) first-stage mass explains it. Look for sources for (a) and (b). Record the analysis in `docs/subsystems.md`.
- [ ] **Step 2: Gate.** A sourced data correction may proceed. Anything else (a guidance throttle-down, a model
  change, an unsourced [E] change to fit the webcast) - write the proposal into the report, commit the diagnosis,
  and STOP for the user's decision.
- [ ] **Step 3: Fix** after approval or when sourced data: re-identify what moves (MECO timing, ascent planner),
  tighten the CRS-35 `meco_speed_allowance` from 0.097 to measured gap + 2 % (aim ~0.03), re-baseline moved numbers
  (`[falcon9]`, `[rtls]`, `[asds]`, `[ascent]`, `[ascent_planner]`, `[staging]`, `[iss_mission]`, `[timeline]` incl.
  `[slow]`).
- [ ] **Step 4: Commit and push.**

---

### Task 8: Cross-flow normal force in the landing planner model (F4)

**Files:**
- Modify: `core/include/starsim/gnc/descent_model.hpp:62-80` (`DescentModel` fields), `core/src/gnc/descent_model.cpp`
  (`derivative`)
- Modify: `core/src/gnc/descent_snapshot.cpp` (fill the new fields from the vehicle's aero)
- Test: `core/tests/test_descent_model.cpp` (tag `[descent]`)
- Modify: `docs/subsystems.md` ("Mid-burn clipping" ~2089-2099, the descent-model simplifications)

**Interfaces:**
- Consumes: the flight model's `aero::SlenderBodyAero` of the stage (geometry, `CrossflowCurve` or the built-in
  fallback eta 0.68 x C_dc 1.2 - expose that fallback as a named constant/function in `slender_body_aero.hpp` instead
  of duplicating the numbers).
- Produces in `DescentModel`:
  - `std::vector<double> crossflow_cd_by_mach;` (eta * C_dc every `mach_step`, the flight model's curve or fallback),
  - `double planform_area_m2 = 0.0;` (L x D of the body), `double crossflow_station_above_cg_m = 0.0;` (mid-length
    station relative to the CG: the moment arm, positive towards the nose),
  - `bool crossflow_enabled = true;` (A/B switch for the measurement; default on).
  - `derivative` adds the cross-flow normal force `q * planform_area * cd_x(M) * sin^2(alpha)` perpendicular to the
    body axis in the plane of axis and air velocity (pointing against the cross component of the air velocity), and
    its moment about the CG from `crossflow_station_above_cg_m`; the potential term (sin 2a ...) is left out unless
    the flight model's potential term is material at landing (state the choice in the doc). Jacobians follow through
    the existing central differences.

- [ ] **Step 1: Failing tests** (`[descent]`):
  - `"cross-flow normal force matches the flight model"`: a Falcon 9 descent snapshot at 1.2 km, 250 m/s tail-first,
    10 deg angle of attack: the model's lateral acceleration (derivative minus the same state with
    `crossflow_enabled = false`) equals `SlenderBodyAero::compute`'s cross-flow normal force / mass within 5 % (the
    remaining difference: the potential term, if left out, quantified in the test comment).
  - `"no cross-flow at zero angle of attack"`: axial flight -> lateral acceleration 0 (abs < 1e-9).
  - `"cross-flow moment sign"`: at 10 deg the angular acceleration from the cross-flow has the sign that the flight
    model's moment about the CG has.
- [ ] **Step 2: Run** `-TestFilter "[descent]"` - FAIL.
- [ ] **Step 3: Implement** fields, snapshot fill, derivative.
- [ ] **Step 4: Measure before/after** (A/B via `crossflow_enabled`): drone-ship and RTLS landing burns'
  `clipped_ticks` / `terminal_clipped_ticks` (`scvx_landing.cpp:370-374`; print with `STARSIM_VERBOSE=1` or the
  existing debug variable) for the `[asds]` and `[rtls]` slow landings and the Super Heavy catches (`[catch]`), and the
  ship landings (`[ship]` catch/landing cases). Record a table in `docs/subsystems.md`.
- [ ] **Step 5: Run the affected tags** `"[descent],[scvx],[scvx_landing],[landing],[catch],[asds],[rtls],[falcon9],[ship],[booster],[replanner]"`
  incl. `[slow]` (background, log file, check every 10 min) - landings and catches pass; numbers that move are
  re-baselined (old -> new) only if not worse; a catch or landing that gets worse is a stop: report it.
- [ ] **Step 6: Success check** (goal, not a hard gate): mid-burn clipping above the terminal phase gone or the
  measured reduction documented with the rest listed as a follow-up in `docs/subsystems.md`.
- [ ] **Step 7: Commit and push** `feat(gnc): cross-flow normal force in the landing planner's descent model`.

---

### Task 9: Close the plan

**Files:** memory (`project-milestone-order.md`, `project-external-view.md`, `project-ascent-staging.md`), the ledger.

- [ ] **Step 1:** Final whole-change review (subagent-driven-development's final review) over the commits of Tasks 1-8;
  its affected-tag test run as the tasks did.
- [ ] **Step 2:** Memory: follow-ups marked done with the measured numbers (test cost, flip, clipping); the next step
  named: mission selection in the panel (selection boxes, data-driven, `config/missions/`), then M8 with tiles + DEM.
- [ ] **Step 3:** Commit, push, ask the user for `/clear`.
