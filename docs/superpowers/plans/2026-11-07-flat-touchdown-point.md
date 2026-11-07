# Flat Touchdown Point Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The lunar touchdown search prefers a point with a slope of at most ~5 deg anywhere within the search radius
and falls back to the nearest point within the hard 8 deg limit only when no flatter one exists, so the HLS stops
rocking off by 1.5-2.4 m on 6-8 deg slopes (Apollo 11 nightly 4/7, Mons Mouton seed 14).

**Architecture:** `mission::select_touchdown_point` (both overloads, `core/src/mission/lunar_touchdown.cpp`) keeps its
admissibility checks with `slope_limit_rad` (8 deg) unchanged; the ring-wise nearest search gains a preference: it
remembers the first admissible candidate (serial order) as the fallback and continues until the first admissible
candidate whose slope is `<= preferred_slope_rad`. The hazard memo, the threads and the bit-for-bit serial/parallel
equality stay. The preferred slope is mission config (`[descent] preferred_slope_deg`).

**Tech Stack:** C++20 core, Catch2, TOML mission configs.

**Spec:** none separate - the user's decision 2026-11-07 ("the descent planner shall pick a touchdown point with
<= ~5 deg slope near the target", memory `project-lunar-sites-followup`; user: "einfach anfangen"). Background:
`docs/known-issues.md` "Lunar sites plan, Task 13", `docs/subsystems/lunar.md` open points "Landing on 6-8 deg".

## Decisions taken by the controller (the user skipped the questions; to check in review)
- D1 Preference, not a new hard limit: no point <= 5 deg within 1 km -> the nearest admissible point <= 8 deg as
  before (sites are not refused because of the preference; the hard limit and the refusal texts stay).
- D2 The whole 1 km search radius counts as "near the target" (a flat point 900 m away beats a 7 deg point 30 m away).
- D3 The slope the preference is judged on is the one `TouchdownPoint::slope_rad` already reports: with the hazard
  check the steepest rest plane over all rolls, LOLA-only the footprint plane.
- D4 Value 5.0 deg, `[E]` (user "~5 deg"); not tuned in this plan. If the validation shows landings still off at <= 5
  deg, that is reported to the user, not fixed by moving the value ([[feedback-config-changes-need-decision]]).
- D5 No UI change: `TouchdownPoint::within_preferred_slope` is for diagnostics, tests and later UI.

## Global Constraints
- SI internally, radians; config in degrees as the other `[descent]` keys (`*_deg`).
- Serial and parallel search return the same point bit for bit (existing test must stay green).
- With `preferred_slope_rad >= slope_limit_rad` the result is bit for bit the pre-plan result.
- Config validation: `preferred_slope_deg` positive and `<= slope_limit_deg` (English developer message).
- Tests per change: `-Fast -TestFilter "[moon_surface][lunar],[lunar_descent_plan],[config]"`; before push the same
  with `[slow]`; the campaigns of Task 2 in the background.

## Review Focus
1. Site with no point <= 5 deg but admissible ones <= 8 deg -> nearest <= 8 deg point, flag false (Task 1 test).
2. Search time at a site whose nearest admissible point is steeper than 5 deg: the search now runs on past it -
   measure IM-2 and Apollo 11 with `STARSIM_DEBUG_TOUCHDOWN=1` (Task 1 step 6), report in the subsystem doc.
3. The window search (`lunar_windows.cpp`) and the in-flight replan (`flight_session_lunar_descent.cpp`) pick the
   same point as the mission start for the same azimuth (same function, same parameters - the setup copies the new
   field; Task 1 test on `lunar_landing_setup`).
4. Preferred point found in a later ring than the fallback in the parallel search: it must still be the first
   preferred one in serial order (Task 1 parallel test extended).
5. Old configs without the key are rejected with a clear message (all three shipped missions get it).

---

### Task 1: Preferred slope in the touchdown search

**Files:**
- Modify: `core/include/starsim/mission/lunar_descent.hpp` (`LunarDescentParameters`, `TouchdownPoint`, doc comments of
  both `select_touchdown_point`)
- Modify: `core/src/mission/lunar_touchdown.cpp` (`nearest_admissible`, `search`)
- Modify: `core/include/starsim/config/mission_config.hpp:110`, `core/src/config/mission_config.cpp:278`,
  `core/src/sim/lunar_landing_mission.cpp:222`
- Modify: `config/missions/hls_lunar_llo.toml`, `hls_lunar_llo_depot.toml`, `hls_lunar_nrho.toml` (`[descent]`)
- Test: `core/tests/test_lunar_touchdown_hazards.cpp`, `core/tests/test_lunar_descent.cpp`, the mission config test

**Interfaces:**
- Produces: `double LunarDescentParameters::preferred_slope_rad = 5.0 * constants::deg_to_rad; // [E] user 2026-11-07`;
  `bool TouchdownPoint::within_preferred_slope = false;` (true when `slope_rad <= preferred_slope_rad`);
  `double config::LunarLandingMission::preferred_slope_rad` read from `[descent] preferred_slope_deg`.

- [ ] **Step 1: Write the failing tests**
  - `"A flatter point farther out beats a steeper near one"` (`[moon_surface][lunar][mission]`, test_lunar_touchdown_hazards.cpp):
    on the file's synthetic surface fixture build terrain whose nominal point is admissible at ~7 deg and an area
    >= 200 m off at <= 3 deg; with defaults: `point->slope_rad <= p.preferred_slope_rad`, `point->within_preferred_slope`,
    `point->offset_m >= 200`; with `p.preferred_slope_rad = p.slope_limit_rad`: the near point (offset < the flat
    area's), as before.
  - `"No point within the preferred slope falls back to the nearest admissible one"`: uniform ~7 deg surface:
    point found, `slope_rad > p.preferred_slope_rad`, `!within_preferred_slope`, offset equal to the search with
    `preferred_slope_rad = slope_limit_rad`.
  - Extend `"The parallel touchdown search picks the serial point bit for bit"` with the flat-farther-out surface
    (threads 1 vs 4: lat, lon, height, slope identical with `==`).
  - Extend `"Detail off selects the same point as before"` (or test_lunar_descent.cpp
    `"The flattest admissible touchdown point near the site is chosen"`): LOLA-only, defaults ->
    `point->slope_rad <= p.preferred_slope_rad` when the old point was steeper, else the same point.
  - Config: shipped missions load `preferred_slope_rad == Approx(5 deg)`; a mission with `preferred_slope_deg` above
    `slope_limit_deg` fails validation; `lunar_landing_setup(m).descent.preferred_slope_rad == m.preferred_slope_rad`.

- [ ] **Step 2: Build and run, expect FAIL** (missing members)
  `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast -TestFilter "[moon_surface][lunar],[lunar_descent_plan],[config]"`

- [ ] **Step 3: Implement**
  - `nearest_admissible(candidates, ring_m, threads, evaluate, preferred)` with `preferred(const TouchdownPoint&) -> bool`:
    serial: first preferred returns at once, the first admissible is kept as fallback; parallel: per ring after the
    join scan results in candidate order, keep the first admissible overall as fallback, return the first preferred;
    after the last ring return the fallback. When `preferred_slope_rad >= slope_limit_rad` every admissible point is
    preferred (the old behaviour, bit for bit).
  - `search` passes `[&](const TouchdownPoint& q){ return q.slope_rad <= p.preferred_slope_rad; }` and sets
    `within_preferred_slope` on the returned point.
  - Config: `preferred_slope_deg = 5.0  # [E] user 2026-11-07: touchdown points <= ~5 deg preferred (6-8 deg rock off
    1.5-2.4 m)` in the three mission files; `doc.positive`, then `doc.fail` if it exceeds `slope_limit_deg`.
  - Doc comments: "Nearest admissible ... preferring the nearest one within preferred_slope_rad anywhere in the radius".

- [ ] **Step 4: Run the tags, expect PASS**; also `[slow]` of the same tags.

- [ ] **Step 5: Timing** - `STARSIM_DEBUG_TOUCHDOWN=1` on `"The IM-2 site with full detail has a safe point within
  the search radius"` and the `[.touchdown_worst]` probe: record wall ms, checks, chosen offset and slope before/after.

- [ ] **Step 6: Docs** - `docs/subsystems/lunar.md` touchdown point paragraph (preference, D1-D3, timing),
  `docs/subsystems/moon-surface.md` touchdown hazard check if it states the selection order; superseded wording to
  `docs/history/lunar.md`.

- [ ] **Step 7: Commit and push**
  `feat(lunar): touchdown search prefers points within 5 deg slope, nearest 8 deg point as fallback`

### Task 2: Validation on the landing campaigns

**Files:**
- Modify: `docs/subsystems/lunar.md` (campaign numbers, open point "Landing on 6-8 deg"), `docs/known-issues.md`
  (Task 13 entry and the seed-14 entry), `docs/subsystems/moon-surface.md` (U-8 open point if it changes)
- Possibly modify: `core/tests/test_lunar_landing_campaign.cpp` - only the printout (`within_preferred_slope`); gates
  stay (1 m, sink 1.1 m/s).

- [ ] **Step 1: Run in the background with output to a file** (check every 10 min):
  `-TestFilter "[lunar_campaign]"` with `[slow]` (21 dispersed + 5 site cases + aborts) and the three
  `"Site campaign nightly"` cases (`-Nightly`, filter on `[lunar_campaign][nightly]`).
- [ ] **Step 2: Table per case**: seed, chosen offset, slope, preferred yes/no, outcome, final offset, sink. Compare
  with the pre-plan numbers (Apollo 11 4/7: seeds 2/3/4 1.71/2.38/1.69 m on 6.6/7.8/5.8 deg; seed 14 1.51 m on 8.0 deg).
- [ ] **Step 3: Verdict** - all within 1 m: remove the open point and the known-issues entries (move to history).
  Any red: diagnose (which slope, rocking as U-8?) and report to the user with the numbers - no value change, no
  gate change (D4).
- [ ] **Step 4: Ledger, memory, commit and push** `test(lunar): campaigns with the preferred flat touchdown point` /
  `docs: ...`.
