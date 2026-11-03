# Test Tiers Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Short test waits: a per-change tier of seconds to a minute or two, a pre-push tier of at most ~335 s per tag, a nightly tier for the full campaigns; webcast agreement as a report, not a gate.

**Architecture:** Catch tags stay the source of truth (they become ctest labels). `tools/build.ps1` gets Catch-like filter semantics and a `nightly` label that is excluded unless asked. Campaigns are cut into a reduced `[slow]` part and a full `[nightly]` part; overlapping cases are merged; webcast comparisons move into hidden report cases. No physics, no guidance, no limit changes.

**Tech Stack:** C++20 / Catch2 v3.8.1 / CTest (`catch_discover_tests ... ADD_TAGS_AS_LABELS`), PowerShell 5.1 (`tools/build.ps1`), Godot 4 headless scripts.

**Spec:** `docs/superpowers/specs/2026-11-03-test-audit.md` (the audit list; data files in `.superpowers/sdd/test-audit/`: `classified.csv`, `flags.txt`, `tagsets.md`, `full_run.log`, `w_checks.txt`, `godot_scripts.txt`).

## Global Constraints

- User 2026-11-03: "Tests müssen sich nach der Physik richten, nicht nach Webcasts. Wenn die Schiffe anders fliegen wie im Webcast, liegt es am Parameter des Schiffs. ... die Tests effizient genutzt werden, die Wartezeiten sind viel zu hoch."
- No physics, guidance, config or limit value changes. A test keeps every physical assertion it has unless the same assertion exists in the case it is merged into (name that case and line in the commit message).
- Deletion only of what this plan lists (the user approves the plan = approves these lists); anything else found: list, do not delete.
- Planner constants (spec section "Planner audit") are out of scope: own physics plan later.
- #369/#370 (settling tanker orbit) stay red and open for the user; #392, #464, #579, #932, #935, #1147 stay as known reds (#1147 changes form in Task 4).
- Commit on `main` by path, Conventional Commits in English, push when green; never `*.uid`, `cmake/build/`; agents never `git stash`, no `pkill`, no unbounded wait loops, no leftover background processes.
- Each task runs only the tags it touches (with `[slow]`), output to a file; the full run happens once, in Task 8.

## Review Focus

1. A tag filter with a typo or an empty intersection must fail loudly ("matched no test"), not pass silently with zero tests - Task 1 step 1.
2. A case moved to `[nightly]` must not vanish: every former case name is still present in `ctest -N -L nightly` or in its reduced `[slow]` sibling - Task 3 step 4.
3. A merged overlap must keep every physical CHECK of the removed case - Task 5 reviewer checklist and step 1.
4. A webcast report case must never fail the build, and its physical gate part must still fail on a real break - Task 4 step 1.
5. `-Nightly` and default runs together cover every registered case exactly once (no case in no tier) - Task 8 step 2.

---

### Task 1: Filter semantics in `tools/build.ps1`

**Files:**
- Modify: `tools/build.ps1`
- Create: `tools/tests/test_build_filter.ps1` (Pester-free: plain asserts, exit code)
- Modify: `docs/setup.md` (test section), `CLAUDE.md` (build & test block, test workflow line)

**Interfaces:**
- Produces: `build.ps1 [-NoBindings] [-Fast] [-TestFilter "<spec>"] [-Nightly] [-ListOnly] [-Jobs N]`. Spec grammar: `,` = union of groups; inside a group `[a][b]` = intersection (Catch semantics). Label `nightly` excluded unless `-Nightly`; `-Nightly` alone runs only `nightly`; with a filter it adds the nightly cases of those tags. `-ListOnly` prints the ctest command and the matched count (`ctest -N`) without building or running. 0 matches throws "TestFilter matched no test".

- [ ] Step 1: Write `test_build_filter.ps1` against `-ListOnly`: `"[gnc][validation][ascent]"` matches the intersection count from `ctest -N` (audit: 5 cases); `"[legs],[asds]"` equals |legs ∪ asds|; `"[nosuchtag]"` throws; `"[falcon9]"` without `-Nightly` contains no `nightly` label case once Task 3 tags exist (until then: count equals all `[falcon9]`).
- [ ] Step 2: Run it, expect fail (no `-ListOnly`, union semantics).
- [ ] Step 3: Implement. Build the selected test list from `ctest -N --show-only=json-v1` (labels per test) in PowerShell and pass it as `-R "^(name1|name2|...)$"` or a `--tests-from-file` (check the installed ctest version for `--tests-from-file`, CMake >= 3.29; else `-I` list or `-R`). Keep the log in `build/ctest_filter.log`.
- [ ] Step 4: Run the test script, expect pass; run `-TestFilter "[legs]"` for real once.
- [ ] Step 5: Docs: setup.md and CLAUDE.md state the new grammar and the three tiers (Task 8 fills the final numbers). Commit `fix(tools): Catch-like tag filter in build.ps1, nightly label, ListOnly`.

### Task 2: Tags follow runtime and module

**Files:** test sources named in `.superpowers/sdd/test-audit/flags.txt`.

- [ ] Step 1: Add `[slow]` to the 32 cases over 40 s listed there; remove `[slow]` from the 18 cases under 5 s listed there.
- [ ] Step 2: Add `[ascent_planner]` to every case that runs the ascent profile planner but lacks the tag (audit: cases tagged only `[mission][sim]`, `[timeline]`, `[staging]`, `[dragon]`; find them by the planner call, list them in the commit).
- [ ] Step 3: Unique temp names in `test_entry_guidance_campaign.cpp:50` (`crossflow_<index>.csv` written by :173 and :209): include the test case name.
- [ ] Step 4: `-ListOnly` counts before/after per tag into `.superpowers/sdd/2026-11-03-test-tiers/task-2-counts.md`; run the touched tags `-Fast` (no campaigns needed: tags only). Commit `test: tags follow runtime and the planner they run`.

### Task 3: Nightly tier and reduced campaigns

**Files:** `core/tests/test_lunar_return_campaign.cpp`, `test_refuelling_campaign_flight.cpp`, `test_refuelling_missions.cpp`, `test_atmosphere_scattering.cpp`, `test_ascent_profile_planner.cpp`, `test_falcon9_rtls.cpp`.

**Interfaces:**
- Consumes: `nightly` label semantics of Task 1.

- [ ] Step 1: Lunar return: add `[nightly]` (drop `[slow]`) to the 7 direct and 3 aerocapture seed cases at :259-271; add reduced `[slow]` cases "Lunar return campaign (direct): seeds 2, 12, 15, 18" and "(aerocapture): seeds A, B, C" where A, B, C are the three aerocapture seeds with the largest gate miss in `full_run.log` (state them in the test comment). Use the existing `fly_seeds` helper or a list overload `fly_seed_list(std::initializer_list<int>, bool aerocapture) -> int`. Seed 2 direct is a known red (known-issues) and stays red.
- [ ] Step 2: `[nightly]` instead of `[slow]` for: flown campaign #466 (`test_refuelling_campaign_flight.cpp:519`) and #464 (:414); refuelling chains `test_refuelling_missions.cpp:586`, `:694`, `:1078`; atmosphere convergence (`test_atmosphere_scattering.cpp:108`) and full resolution (:182); ascent kick sweep (`test_ascent_profile_planner.cpp:152`).
- [ ] Step 3: Falcon 9 flip campaign (`test_falcon9_rtls.cpp:446`, 13 separations): split into 3 cases of 4-5 separations each, same assertions; the off-pad list check `{4}` stays in the case holding separation 4, the others assert an empty off-pad list.
- [ ] Step 4: Check: every case name removed from the default tier appears in `ctest -N -L nightly` (script output to the task folder). Run `[lunar],[return],[falcon9],[atmosphere]` (default tier, with `[slow]`): wall per tag into the report; then `-Nightly -TestFilter "[lunar]"` once in the background. Commit `test: nightly tier for full campaigns, reduced seeds in [slow]`.

### Task 4: Webcast agreement as a report

**Files:** the 11 W cases in the spec: `test_ship_entry_telemetry.cpp:225`, `test_starship_ascent.cpp:82`, `test_falcon9_ascent.cpp:218`, `test_descent_telemetry.cpp:78`, `test_falcon9_descent_telemetry.cpp:78`, `test_booster_return.cpp:239`, `test_falcon9_rtls.cpp:270`, `test_falcon9_asds.cpp:480` and `:268`, `test_staging_sequence.cpp:151`, `test_capsule_splashdown.cpp:393`; B pins `test_entry_skip.cpp:145`, `test_docking_mechanism.cpp:142`. Create `core/tests/support/webcast_report.hpp/.cpp`.

**Interfaces:**
- Produces: `void webcast_report(const std::string& case_id, const std::vector<WebcastRow>& rows)`; `struct WebcastRow { std::string quantity; double ours; double reference; double tolerance; std::string reference_source; std::string parameters; };` writes `${STARSIM_OUTPUT_DIR}/webcast_report/<case_id>.md` (unique per case) and prints one `WARN` line per row outside tolerance. Never fails.

- [ ] Step 1: Per W case: split into (a) the physical gate case, normal tier, with the "gate to keep" of the spec (e.g. ship entry: no impact, trim not saturated; F9 RTLS: orbit, landed upright on LZ-40); (b) a `[nightly][webcast]` case flying the same flight that calls `webcast_report` with every former comparison; `parameters` names the vehicle parameters of the spec's W list (e.g. "entry mass, CG, crossflow Cd, flap CdA"). Where a W case has no physical gate (descent telemetry rms) only (b) remains. KNOWN MISS pins of the F9 flip go to the report.
- [ ] Step 2: B: drop the 77.5 +-0.01 deg bank pin in `test_entry_skip.cpp:145` (gate, miss and heat stay); `test_docking_mechanism.cpp:142` KNOWN STATE becomes a report row.
- [ ] Step 3: Run the touched tags (default tier) and `-Nightly -TestFilter "[webcast]"`; read two report files. #1147's gate part is expected green or red by physics only; state which. Docs: `docs/known-issues.md` (#1147 form), the subsystem pages whose validation names a webcast band (`falcon9.md`, ship entry, booster return): "report, not gate" with the date and the user's words. Commit `test: webcast agreement as a nightly report, physical gates kept`.

### Task 5: Merge overlaps

**Files:** per the spec's C list.

- [ ] Step 1: For each pair write into `.superpowers/sdd/2026-11-03-test-tiers/task-5-map.md`: removed case, kept case, each CHECK of the removed case and where it lives in the kept case (add it there if missing). Pairs: `test_mission_start.cpp:69` -> `test_mission_timeline.cpp:760/:800`; `test_mission_timeline.cpp:482` -> `:451` (placed orbit); `:564` from the lunar-return hand-over state; `test_staging_sequence.cpp:205` from a placed MECO state; `test_ascent_profile_planner.cpp:227` memo with a Falcon 9 request; `test_lunar_return_session.cpp:171` -> direct campaign seeds; `test_proximity_session.cpp:184/:189` vs `test_docking_campaign.cpp:84/:100` vs `test_iss_mission_campaign.cpp:48/:75` (one per port); `test_crew_dragon.cpp:435` -> `test_iss_mission_campaign.cpp:178`; `test_refuelling_missions.cpp:95/:127` from a placed caught booster; `test_atmosphere_tables.cpp:79` small grid. `test_ship_return_mission.cpp:250`: only if the map proves full coverage, else keep and say so.
- [ ] Step 2: Apply. A "placed state" starts from a state written by a helper from the flown state's documented numbers, not a re-pinned number of a new run.
- [ ] Step 3: Run the touched tags with `[slow]`; table old/new wall per tag. Commit `test: merge overlapping flights, placed start states`.

### Task 6: #711 sampling and the gap tests

**Files:** `core/tests/test_mission_timeline.cpp:945-990`; create `core/tests/test_booster_return_prediction.cpp` (+ register in `core/tests/CMakeLists.txt`).

- [ ] Step 1: #711: step at <= 1 s from 20 s before the predicted ignition to the end of the burn; the assertion (state Burn inside the burn) stays. Red before? It is red now; green after with no code change.
- [ ] Step 2: `[booster_return][gnc]` unit cases for `gnc/booster_return_prediction.cpp` and `booster_return_burns.cpp`: boostback dv against the rocket equation on a hand case, flip propellant against RCS impulse / Isp, landing-burn prediction against a constant-thrust hand solution; tolerances stated with their reason.
- [ ] Step 3: Run `[timeline],[booster_return],[booster]`. Commit `test: timeline steps through the burn, booster return prediction unit cases`.

### Task 7: Godot test runner

**Files:** create `tools/run_godot_tests.ps1`; the 7 never-run scripts (`clock_hold_test`, `free_camera_run`, `local_time_test`, `trajectory_view_test`, `ui_rate_test`, `water_waves_test`, `weather_fetcher_test`); `docs/setup.md`.

- [ ] Step 1: Runner: every `game/tests/*_test.gd` headless (`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/<name>`), with a list of GPU-only and model-dependent ones skipped unless `-Gpu`; per-script time and exit code to `build/godot_tests.log`; non-zero exit if any fails.
- [ ] Step 2: Run it. Each of the 7: passes -> stays; fails because what it tests is gone -> delete (list in commit); fails otherwise -> keep, list in known-issues. `free_camera_run` is a tool, not a test: say so in its header.
- [ ] Step 3: setup.md: runner and when (game/ changes). Commit `test(game): headless Godot test runner, stale scripts`.

### Task 8: Measure and document the tiers

**Files:** `CLAUDE.md`, `docs/setup.md`, `docs/known-issues.md`; memory note by the controller.

- [ ] Step 1: One full default run (`build.ps1 -NoBindings`, background, log to `.superpowers/sdd/2026-11-03-test-tiers/full_after.log`) and one `-Nightly` run. Table: wall and CPU per common tag set now vs the audit (spec "Tag sets"), full-run wall (target <= ~500 s), nightly wall.
- [ ] Step 2: Coverage check: `ctest -N` of default ∪ nightly = all registered cases; no case in neither (script output).
- [ ] Step 3: CLAUDE.md test workflow: per change `-Fast -TestFilter` of the touched tags; before push and before a final review the same tags with `[slow]`; `-Nightly` only at a plan's end or when asked, in the background; webcast reports read, not gated. setup.md with the measured numbers. Commit `docs: test tiers with measured waits`; push.

---

Execution: subagent-driven; Sonnet implementers (Task 4 and 5 Sonnet with an Opus reviewer: physical gates must survive), final review Opus.
