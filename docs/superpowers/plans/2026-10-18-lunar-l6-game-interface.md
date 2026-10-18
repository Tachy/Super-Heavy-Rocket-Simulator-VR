# Lunar l6: game interface (simulation thread, fast return planning, lunar bindings) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The game can start and follow both lunar missions through `FlightSim` without ever freezing: the flight
session runs on its own simulation thread, the return planner's gate scan is cheap, and the lunar state (reference
body, terrain, step, plans, pilot cue, re-plan) is readable from GDScript - covered by the smoke test.

**Architecture:** A core `sim::SessionRunner` owns the `FlightSession` on a worker thread (command queue, a target
epoch, a latest-input slot, an observer called after every step). The GDExtension `FlightSim` posts commands to it and
its getters read an immutable snapshot the observer builds on the worker after every step; command results come back
by id (`result(id)`, signal `command_finished`). CSPICE is serialised by a process-wide lock inside the `spice` module,
so `StarsimCore` (rendering, main thread) and the simulation thread may both call it. The return planner's gate scan is
profiled first and made cheaper without changing the plans beyond a stated tolerance.

**Tech Stack:** C++20 (std::thread, std::mutex, std::condition_variable), Catch2, godot-cpp (precision=double), GDScript
4.7, CSPICE (via `core/src/spice/` only).

**Spec:** `docs/superpowers/specs/2026-10-14-lunar-missions-design.md` (section "l6 design: game interface", plus the
error-handling section). Earlier ledgers that feed l6: `.superpowers/sdd/2026-10-16-lunar-l5-return/progress.md`
(plan time, re-plan key), the l3/l4 plan ledgers (Moon orientation in the game's FlightSim, telemetry altitude at the
Moon, osculating elements at burn end).

## User decisions (2026-10-18, this plan)

- **Simulation thread + cheaper gate scan** (option 1 of 3): `FlightSim` runs the session on its own worker thread
  (commands via a queue, the game reads a snapshot after each step) - the CLAUDE.md architecture ("the simulation runs
  on its own thread"); CSPICE serialised by a process-wide lock; while a plan runs the simulation clock stands (the
  world clock already waits for the flight sim). Plus: the gate scan coarse first, then fine - target < 10 s.
- **Bindings + smoke test only:** no key, no HUD line, no mission menu, `docs/ansichten.md` untouched. Everything
  visible comes with the user's UI specification. (Existing game scripts are adapted to the threaded API only as far
  as they must be to keep working.)

## Global Constraints

- `core/` has no Godot dependency; `bindings/` stays thin: no physics or business logic, only conversion, the
  snapshot and the thread glue to `sim::SessionRunner`.
- CSPICE is called only from `core/src/spice/`; every adapter call holds the process-wide CSPICE lock.
- SI units internally, angles in radians; names carry frame and unit (`position_moon_me_m`, `height_m`). The GDScript
  interface keeps the existing conventions (degrees in `*_deg` keys, GCRF metres).
- UI strings / refusal reasons in German, code and comments in English. No magic numbers: tuned values are named
  constants with an `[E]` tag and a reason.
- Commit on `main` (Conventional Commits) and push after every task once its tests are green; `-Fast` before every
  push; full suite (with `[slow]`) for Task 1 (guidance planning) and before the final review.
- ctest runs test cases as parallel processes: no shared temp files.
- Never filter `-TestFilter "[mission]"` / `"[outbound]"` (pulls in slow campaigns serially, > 27 min); use narrow
  tags or `-Fast`.
- Long command output to files in `$CLAUDE_JOB_DIR/tmp` (or `tmp/` in the repo, git-ignored), read tails / grep.

## Review Focus

1. **Quitting or re-`setup()` while a plan runs:** freeing `FlightSim` during a 2-min NRHO plan must neither crash nor
   leave a thread behind; it waits for the running call (documented) and drops the queued ones. Test: Task 3 Step 1
   (`runner destroyed during a long command`), Task 4 Step 6 (smoke: `setup()` twice).
2. **Pilot input during a long plan:** inputs must not pile up behind the plan (only the newest input counts) and a
   command posted during the plan runs after it, in posting order. Test: Task 3 Step 1 (`latest input only`,
   `commands keep their order`).
3. **Rendering's SPICE calls while the simulation thread plans:** no corrupted state, no spurious SPICE error on the
   main thread when the worker hits a SPICE error (CSPICE's error state is global). Test: Task 2 Step 1.
4. **A refused lunar mission must not look like a started one:** today `FlightSim.start_mission` returns `true` for a
   lunar start the planner refused (`MissionStart::refusal` is dropped). Expected: `ok == false` with the German
   reason. Test: Task 6 Step 5 (smoke: NRHO landing mission at 120 t is refused with "Mission nicht durchführbar").
5. **Lunar telemetry without terrain or away from the Moon:** a lunar return ship (no terrain loaded) and Earth flights
   must report `altitude_terrain_m` (= the shape altitude) and `moon_terrain()` `present == false` - no exception, no
   NaN. Test: Task 5 Step 1, Task 6 Step 5.

---

## File structure

| File | Responsibility |
|---|---|
| `core/src/mission/earth_return.cpp` (modify) | gate scan: coarse-then-fine (Task 1) |
| `core/tests/test_plan_timing.cpp` (create) | hidden `[.plan_timing]` probe + `[lunar_return][plan_speed]` equality/timing test |
| `core/src/spice/spice_check.hpp`, `spice_session.cpp`, `spice_ephemeris.cpp`, `spice_orientation.cpp`, `spice_time_converter.cpp` (modify) | process-wide CSPICE lock |
| `core/tests/test_spice_threads.cpp` (create) | two threads against CSPICE |
| `core/include/starsim/sim/session_runner.hpp`, `core/src/sim/session_runner.cpp` (create) | the simulation thread |
| `core/tests/test_session_runner.cpp` (create) | runner tests |
| `bindings/src/flight_sim.hpp/.cpp` (modify) | commands (ids, results, signal), snapshot reads |
| `bindings/src/flight_snapshot.hpp/.cpp` (create) | the snapshot struct and its builders (run on the worker) |
| `bindings/src/flight_snapshot_lunar.cpp` (create) | lunar parts of the snapshot (Task 6) |
| `game/vehicle/flight_controller.gd`, `game/vehicle/trajectory_view.gd`, `game/world/world.gd`, `game/tests/*.gd` (modify) | threaded API |
| `core/include/starsim/vehicle/flight_body.hpp`, `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (modify) | `altitude_terrain_m`, stored lunar site (Task 5) |
| `docs/subsystems.md`, `CLAUDE.md`, spec (modify) | docs (Task 7) |

---

### Task 1: Cheaper gate scan (profile first)

The return planner's gate scan takes ~55 s (LLO) to ~2 min (NRHO) (`docs/subsystems.md`, l5 "Plan time"). Measure
where the time goes, make the scan coarse-then-fine, and prove the plans are unchanged within tolerance.

**Files:**
- Create: `core/tests/test_plan_timing.cpp` (add to `core/tests/CMakeLists.txt`, `add_executable(starsim_core_tests`)
- Modify: `core/src/mission/earth_return.cpp` (gate scan, lines ~47-51 constants and ~599-700 `aim_at_gate_once`
  scan loop), `core/include/starsim/mission/earth_return.hpp` only if a parameter is added
- Test: the new file; `[lunar_return]` tests; full suite (campaigns)

**Interfaces:**
- Consumes: `sim::start_mission`, `test::LunarReturnSession` (`core/tests/lunar_return_test_support.hpp`),
  `FlightSession::lunar_return_plan()` (`mission::ReturnPlan`: `tei`, `tei_delta_v_mps`, `braking_delta_v_mps`,
  `arrival.interface_epoch`, `arrival.speed_mps`, `arrival.flight_path_rad`).
- Produces: nothing new in the API; `EarthReturnPlanner` keeps its signatures.

- [ ] **Step 1: Write the timing probe (hidden) and record the baseline**

```cpp
// Plan time of the lunar mission starts (l6 Task 1): wall seconds per start_mission, printed. Hidden: run by name.
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdio>

#include "lunar_return_test_support.hpp"
#include "starsim/sim/mission_start.hpp"

using namespace starsim;

namespace {
double timed_start(const char* mission_id) {
    test::LunarReturnSession s;
    const auto t0 = std::chrono::steady_clock::now();
    const auto start = sim::start_mission(s.session, STARSIM_CONFIG_DIR, mission_id, {}, s.time);
    const double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("[plan_timing] %-26s %7.1f s  planned %d  %s\n", mission_id, wall_s, start.return_planned ? 1 : 0,
                start.refusal.c_str());
    return wall_s;
}
}  // namespace

TEST_CASE("plan time of the lunar mission starts", "[.plan_timing]") {
    for (const char* id : {"ship_lunar_return_llo", "ship_lunar_return_nrho", "ship_lunar_aircatch_llo",
                           "hls_lunar_llo"}) {
        timed_start(id);
    }
}
```

Run (one process, output to a file):
`core/build/tests/starsim_core_tests.exe "[.plan_timing]" > tmp/plan_timing_before.txt 2>&1` (path of the test
binary as `tools/build.ps1` builds it - check `core/build*/` once). Record the four times in the Ledger.

- [ ] **Step 2: Profile the scan**

Add a wall-time and call-count line to the existing `STARSIM_DEBUG_RETURN` output of `EarthReturnPlanner` (no new
flag): per `plan`/`recovery` call the seconds and counts of `evaluate()` calls, predictor runs and scan steps (counters
as `mutable int` members reset at the start of the public call; printed only when the debug flag is set). Run the LLO
start with `STARSIM_DEBUG_RETURN=1` into `tmp/plan_profile.txt`; grep the counts. Record in the Ledger: share of the
time in (a) the inbound coast propagations (DOP853, ephemeris), (b) the onboard predictor (`gnc::predict_entry`, 1 s
step, up to 3,000 s), (c) Newton iterations per scan step.

- [ ] **Step 3: Write the equality + speed test (fails on the speed bound)**

Add to `core/tests/test_plan_timing.cpp`:

```cpp
// The cheap scan finds the same plans (l6 Task 1, user 2026-10-18: target < 10 s). Reference numbers: the plans of
// the scan before Task 1 (Ledger, Step 1) - TEI within 0.5 m/s, braking within 1 m/s, interface epoch within 60 s,
// interface speed within 2 m/s and flight-path angle within 0.02 deg [E] (well inside the campaign's closed-loop
// residuals +-0.2 % / 0.1 deg).
TEST_CASE("cheap gate scan keeps the LLO return plan", "[lunar_return][plan_speed]") {
    test::LunarReturnSession s;
    const auto t0 = std::chrono::steady_clock::now();
    const auto start = sim::start_mission(s.session, STARSIM_CONFIG_DIR, "ship_lunar_return_llo", {}, s.time);
    const double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    REQUIRE(start.return_planned);
    const auto* plan = s.session.lunar_return_plan();
    REQUIRE(plan != nullptr);
    CHECK(std::abs(plan->tei_delta_v_mps - kLloTeiMps) < 0.5);
    CHECK(std::abs(plan->braking_delta_v_mps - kLloBrakingMps) < 1.0);
    CHECK(std::abs(plan->arrival.interface_epoch.tdb_seconds_past_j2000() - kLloInterfaceTdbS) < 60.0);
    CHECK(std::abs(plan->arrival.speed_mps - kLloInterfaceSpeedMps) < 2.0);
    CHECK(std::abs(plan->arrival.flight_path_rad - kLloInterfaceAngleRad) < 0.02 * constants::deg_to_rad);
    CHECK(wall_s < kPlanTimeLimitS);
}
```

with the five `kLlo*` constants filled with the baseline plan's values printed in Step 1 (extend the probe's printf
with `tei_delta_v_mps`, `braking_delta_v_mps`, interface epoch, speed and angle - `%.3f`), and
`constexpr double kPlanTimeLimitS = 10.0;  // user 2026-10-18`. The same test case for `ship_lunar_return_nrho`
(constants `kNrho*`, limit 20 s [E]: two burns, 8 departure options) and `ship_lunar_aircatch_llo` (limit 10 s).
Run `-TestFilter "[plan_speed]"`: expected FAIL on `wall_s` only (the plan checks pass on the old code).

- [ ] **Step 4: Implement the coarse-then-fine scan**

The measure follows the Step 2 profile. Default order (stop when the bound holds):
1. **Coarse predictor in the scan:** the bracketing of the downrange sign change runs the predictor at
   `gate_scan_predictor_step_s = 4.0` [E] (the predictor's RK4 at 4 s stays within ~1 km of the 1 s run on these
   ~2,000 s glides - check it in the test of Step 3 by also comparing the final plan); the refinement of the bracketed
   crossing (secant to `downrange_error_m == 0`) runs at the existing 1 s.
2. **Coarse scan grid first:** keep `gate_scan_step_s` (2 h) but evaluate the Newton solve of each step with a looser
   tolerance (`gate_scan_coarse_tolerance_m` [E]) and only the bracket's two ends at the full tolerance.
3. **Coast reuse:** consecutive scan steps differ only in the arrival epoch; warm-start Newton from the last two
   solutions already exists - if Step 2 shows Newton iterations dominate (> 4 per step), add a one-step linear
   extrapolation in the epoch.

Implementation sketch for 1 (the predictor call sites in the scan pass the step):

```cpp
// [E] The gate scan only needs the sign of the predictor's downrange error to bracket the site's pass; a 4 s
// predictor step keeps the range within ~1 km of the 1 s run on a 2,000 s glide (l6 Task 1 test). The bracketed
// crossing is refined at the full predictor step.
constexpr double gate_scan_predictor_step_s = 4.0;
```

and an `double predictor_step_s` parameter on the private `aim_at_gate_once` / evaluation helpers (default the current
1 s), passed as `gate_scan_predictor_step_s` by the scan loop (lines ~617-645) and as 1 s by the refinement.

If no measure reaches the bound without breaking a plan check, STOP: write a `FINDING` line to the Ledger with the
profile and ask the user (bound vs plan tolerance).

- [ ] **Step 5: Run the touched tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[plan_speed]"`, then
`-TestFilter "[lunar_return]"` (to `tmp/l6_t1_lunar_return.txt`). Expected: PASS. Re-run the probe of Step 1 and
record the new times.

- [ ] **Step 6: Full suite (guidance planning changed)**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings > tmp/l6_t1_full.txt 2>&1`; tail it.
Expected: all green (628/628 before Task 1 plus the new cases), lunar return campaigns direct 21/21 and aircatch
21/21 unchanged in outcome.

- [ ] **Step 7: Commit and push**

```bash
git add core/tests/test_plan_timing.cpp core/tests/CMakeLists.txt core/src/mission/earth_return.cpp
git commit -m "perf(mission): return gate scan coarse first, refined at the full predictor step"
git push origin main
```

Update the Ledger (times before/after, profile shares) and `docs/subsystems.md` l5 "Plan time" sentence in the same
commit.

---

### Task 2: Process-wide CSPICE lock

**Files:**
- Modify: `core/src/spice/spice_check.hpp`, `core/src/spice/spice_session.cpp`, `core/src/spice/spice_ephemeris.cpp`,
  `core/src/spice/spice_orientation.cpp`, `core/src/spice/spice_time_converter.cpp`,
  `core/include/starsim/spice/spice_session.hpp` (comment)
- Create: `core/tests/test_spice_threads.cpp` (add to `core/tests/CMakeLists.txt`)

**Interfaces:**
- Produces: `spice::detail::lock_spice()` (internal to `core/src/spice/`): `std::unique_lock<std::recursive_mutex>`.
  Every public adapter function (SpiceEphemeris, SpiceOrientation, SpiceTimeConverter, SpiceSession open/close and
  `default_kernel_files` users that call CSPICE) holds it for the whole call, including `throw_if_failed` (CSPICE's
  error state is global: check and reset must happen under the same lock as the failing call).

- [ ] **Step 1: Write the failing test**

```cpp
// CSPICE from two threads (l6 Task 2): the simulation thread and rendering (StarsimCore) both call the adapters.
#include <catch2/catch_test_macros.hpp>
#include <atomic>
#include <thread>
#include <vector>

#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

TEST_CASE("CSPICE adapters are safe from two threads", "[spice][threads]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    constexpr int n = 4000;
    std::vector<math::Vec3> moon_ref(n);
    for (int i = 0; i < n; ++i) {
        moon_ref[i] = ephemeris.state_gcrf(ephemeris::Body::Moon, time::Epoch::from_tdb_seconds_past_j2000(i * 60.0))
                          .position_m;
    }
    std::atomic<int> mismatches = 0, spurious_errors = 0;
    std::thread worker([&] {  // valid calls only: must never throw nor differ
        for (int i = 0; i < n; ++i) {
            try {
                const auto p = ephemeris.state_gcrf(ephemeris::Body::Moon,
                                                    time::Epoch::from_tdb_seconds_past_j2000(i * 60.0)).position_m;
                if (p.x != moon_ref[i].x || p.y != moon_ref[i].y || p.z != moon_ref[i].z) ++mismatches;
                (void)earth.fixed_to_gcrf(time::Epoch::from_tdb_seconds_past_j2000(i * 60.0));
            } catch (...) {
                ++spurious_errors;
            }
        }
    });
    int expected_errors = 0;
    for (int i = 0; i < n; ++i) {  // every 10th call fails on purpose (far outside de440s: 1849-2150)
        try {
            if (i % 10 == 0) {
                (void)ephemeris.state_gcrf(ephemeris::Body::Moon, time::Epoch::from_tdb_seconds_past_j2000(1.0e11));
            } else {
                (void)earth.fixed_to_gcrf(time::Epoch::from_tdb_seconds_past_j2000(i * 30.0));
            }
        } catch (const std::exception&) {
            ++expected_errors;
        }
    }
    worker.join();
    CHECK(mismatches == 0);
    CHECK(spurious_errors == 0);
    CHECK(expected_errors == n / 10);
}
```

Run: `tools/build.ps1 -NoBindings -TestFilter "[threads]"`. Expected: FAIL (spurious errors / mismatches or a crash -
note which in the Ledger; a crash also counts as the failing state).

- [ ] **Step 2: Implement the lock**

`core/src/spice/spice_check.hpp`:

```cpp
#include <mutex>

namespace starsim::spice::detail {
// CSPICE is process-global and not thread-safe (kernel pool, file buffers, error state): every adapter call holds this
// lock for its whole duration, including the error check and reset. Recursive: an adapter may call another.
std::unique_lock<std::recursive_mutex> lock_spice();
}  // namespace starsim::spice::detail
```

`core/src/spice/spice_session.cpp`:

```cpp
std::unique_lock<std::recursive_mutex> detail::lock_spice() {
    static std::recursive_mutex mutex;
    return std::unique_lock<std::recursive_mutex>(mutex);
}
```

First line of every public adapter function in the four `.cpp` files: `const auto lock = detail::lock_spice();`.
Update the comment in `spice_session.hpp` ("all SPICE-backed objects must be used from one thread" -> "adapters
serialise CSPICE with a process-wide lock; a SpiceSession must still be opened once, before any adapter call").

- [ ] **Step 3: Run the tests**

`-TestFilter "[threads]"` then `-Fast` (to `tmp/l6_t2_fast.txt`). Expected: PASS; no slowdown above noise in
`-Fast` wall time (record it).

- [ ] **Step 4: Docs, commit, push**

CLAUDE.md, Quality rules: replace "CSPICE is not thread-safe: only the simulation thread calls `ephemeris`." with
"CSPICE is not thread-safe: the `spice` adapters serialise every call with a process-wide lock (l6)."

```bash
git add core/src/spice core/include/starsim/spice/spice_session.hpp core/tests/test_spice_threads.cpp core/tests/CMakeLists.txt CLAUDE.md
git commit -m "feat(spice): process-wide CSPICE lock for the simulation thread"
git push origin main
```

---

### Task 3: `sim::SessionRunner` - the simulation thread

**Files:**
- Create: `core/include/starsim/sim/session_runner.hpp`, `core/src/sim/session_runner.cpp` (add to
  `core/CMakeLists.txt` next to `src/sim/flight_session_lunar_return.cpp`; link `Threads::Threads` if the build needs
  it - MSVC does not)
- Create: `core/tests/test_session_runner.cpp` (add to `core/tests/CMakeLists.txt`)

**Interfaces:**
- Consumes: `FlightSession::advance_to(time::Epoch target, int max_ticks)`, `FlightSession::active()`,
  `FlightSession::epoch()` (all exist in `flight_session.hpp`).
- Produces (used by Task 4):

```cpp
#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

// Runs a FlightSession on its own thread (l6, user 2026-10-18; CLAUDE.md: "the simulation runs on its own thread").
// From construction on only the worker touches the session: commands run in posting order, between them the session
// advances towards the target epoch in chunks; the observer runs on the worker after every command and every chunk
// (the GDExtension builds its snapshot there). A long call (a return plan: seconds) holds the simulation clock -
// the caller sees busy_s() grow and the epoch stand.
class SessionRunner {
public:
    using Command = std::function<void(FlightSession&)>;
    using Observer = std::function<void(const FlightSession&, time::Epoch epoch)>;

    // [E] One chunk advances at most this many ticks (FlightSession::advance_to's limit) before commands and the
    // observer get their turn again: 4000 ticks = 20 s of flight at 200 Hz.
    static constexpr int max_ticks_per_chunk = 4000;

    SessionRunner(std::unique_ptr<FlightSession> session, Observer observer);
    // Drops the queued commands, waits for the running call (a plan may take seconds) and joins.
    ~SessionRunner();
    SessionRunner(const SessionRunner&) = delete;
    SessionRunner& operator=(const SessionRunner&) = delete;

    // Queues a command; returns its id (> 0, increasing). An exception it throws is caught: result(id) carries it.
    std::uint64_t post(Command command);
    // Overwrites the single "latest" slot (pilot input): run once before the next chunk; older values are dropped.
    void set_latest(Command command);
    // The session advances towards `target` whenever no command waits (never backwards).
    void set_target(time::Epoch target);

    struct Result {
        bool done = false;
        bool threw = false;
        std::string error;  // what() of the exception
    };
    Result result(std::uint64_t id) const;  // done == false while queued or running (unknown ids: done, error set)
    // Finished results not yet taken, oldest first (the GDExtension emits command_finished for them).
    std::deque<std::pair<std::uint64_t, Result>> take_finished();

    bool idle() const;                     // no command queued or running, and the target reached (or inactive)
    bool wait_idle(double timeout_s) const;  // true when idle within the timeout (tests, scripts)
    double busy_s() const;                 // wall seconds the current command has been running (0 without one)
    std::string last_error() const;        // of the latest input, advance or observer exception ("" none)

private:
    void run();
    bool idle_locked() const;  // mutex_ held
    std::unique_ptr<FlightSession> session_;
    Observer observer_;
    mutable std::mutex mutex_;
    mutable std::condition_variable wake_, idle_cv_;
    std::deque<std::pair<std::uint64_t, Command>> queue_;
    std::optional<Command> latest_;
    std::optional<time::Epoch> target_;
    time::Epoch reached_;
    std::uint64_t next_id_ = 1;
    std::uint64_t running_id_ = 0;  // 0: none
    std::chrono::steady_clock::time_point running_since_;
    std::map<std::uint64_t, Result> results_;  // kept until taken by take_finished() or 1,000 newer ids [E]
    std::deque<std::pair<std::uint64_t, Result>> finished_;
    std::string last_error_;
    bool active_ = false;  // the session's active(), as of the last command / chunk (worker writes under the lock)
    bool stop_ = false;
    std::thread worker_;  // last: started after every member is initialised
};

}  // namespace starsim::sim
```

(Add `<chrono>`, `<map>` and `<utility>` includes.) Idle semantics: `idle()` is `queue_.empty() && running_id_ == 0 &&
(!session_->active() || !target_ || reached_ >= *target_)` - `active()` read on the worker only, cached in a member
`active_` the worker updates after every command / chunk (the main thread must not touch the session).

- [ ] **Step 1: Write the failing tests**

```cpp
// sim::SessionRunner (l6 Task 3): the simulation thread.
#include <catch2/catch_test_macros.hpp>
#include <atomic>
#include <chrono>
#include <thread>

#include "starsim/sim/session_runner.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using namespace std::chrono_literals;

namespace {
struct RunnerFixture {
    bool kernels = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    std::atomic<int> observed = 0;
    std::atomic<double> last_epoch_s = -1.0e300;
    std::atomic<bool> epochs_monotone = true;
    sim::SessionRunner runner{std::make_unique<sim::FlightSession>(ephemeris, earth, STARSIM_CONFIG_DIR),
                              [this](const sim::FlightSession&, time::Epoch e) {
                                  const double s = e.tdb_seconds_past_j2000();
                                  if (s < last_epoch_s) epochs_monotone = false;
                                  last_epoch_s = s;
                                  ++observed;
                              }};
};
const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);  // 2026-08
}  // namespace

TEST_CASE("runner advances a session to the target", "[session_runner]") {
    RunnerFixture f;
    const auto id = f.runner.post([](sim::FlightSession& s) {
        s.launch_from_pad("starship_block3", sim::LaunchSite{0.4537, -1.6956, 10.0}, 0.0, t0);
    });
    f.runner.set_target(t0 + 5.0);
    REQUIRE(f.runner.wait_idle(30.0));
    CHECK(f.runner.result(id).done);
    CHECK_FALSE(f.runner.result(id).threw);
    CHECK(f.last_epoch_s == (t0 + 5.0).tdb_seconds_past_j2000());
    CHECK(f.epochs_monotone);
    CHECK(f.observed >= 2);
}

TEST_CASE("runner commands keep their order and report exceptions", "[session_runner]") {
    RunnerFixture f;
    std::vector<int> order;
    const auto a = f.runner.post([&](sim::FlightSession&) { order.push_back(1); });
    const auto b = f.runner.post([](sim::FlightSession&) { throw std::runtime_error("Absicht"); });
    const auto c = f.runner.post([&](sim::FlightSession&) { order.push_back(3); });
    REQUIRE(f.runner.wait_idle(10.0));
    CHECK(order == std::vector<int>{1, 3});
    CHECK(f.runner.result(b).threw);
    CHECK(f.runner.result(b).error == "Absicht");
    const auto finished = f.runner.take_finished();
    REQUIRE(finished.size() == 3);
    CHECK(finished[0].first == a);
    CHECK(finished[2].first == c);
}

TEST_CASE("runner keeps only the latest input and reports a long command as busy", "[session_runner]") {
    RunnerFixture f;
    std::atomic<int> latest_runs = 0, latest_value = 0;
    f.runner.post([](sim::FlightSession&) { std::this_thread::sleep_for(400ms); });  // a "plan"
    std::this_thread::sleep_for(100ms);
    CHECK(f.runner.busy_s() > 0.05);
    for (int i = 1; i <= 50; ++i) {
        f.runner.set_latest([&, i](sim::FlightSession&) { ++latest_runs; latest_value = i; });
    }
    f.runner.post([](sim::FlightSession& s) {
        s.launch_from_pad("starship_block3", sim::LaunchSite{0.4537, -1.6956, 10.0}, 0.0, t0);
    });
    f.runner.set_target(t0 + 1.0);
    REQUIRE(f.runner.wait_idle(30.0));
    CHECK(latest_runs == 1);
    CHECK(latest_value == 50);
    CHECK(f.runner.busy_s() == 0.0);
}

TEST_CASE("runner destroyed during a long command", "[session_runner]") {
    std::atomic<bool> second_ran = false;
    const auto begin = std::chrono::steady_clock::now();
    {
        RunnerFixture f;
        f.runner.post([](sim::FlightSession&) { std::this_thread::sleep_for(300ms); });
        f.runner.post([&](sim::FlightSession&) { second_ran = true; });
        std::this_thread::sleep_for(50ms);
    }  // destructor: waits for the first, drops the second
    CHECK_FALSE(second_ran);
    CHECK(std::chrono::steady_clock::now() - begin < 2s);
}
```

(Check `sim::LaunchSite`'s member order in `flight_session.hpp` - latitude, longitude, height; Starbase is
25.997 deg N, 97.154 deg W = 0.4537, -1.6956 rad.) Run `-TestFilter "[session_runner]"`: expected FAIL (does not
compile: header missing).

- [ ] **Step 2: Implement the runner**

```cpp
#include "starsim/sim/session_runner.hpp"

namespace starsim::sim {

namespace {
constexpr size_t max_kept_results = 1000;  // [E] results nobody took: dropped oldest first
}

SessionRunner::SessionRunner(std::unique_ptr<FlightSession> session, Observer observer)
    : session_(std::move(session)), observer_(std::move(observer)), worker_([this] { run(); }) {}

SessionRunner::~SessionRunner() {
    {
        std::lock_guard lock(mutex_);
        stop_ = true;
        queue_.clear();
        latest_.reset();
    }
    wake_.notify_all();
    worker_.join();
}

std::uint64_t SessionRunner::post(Command command) {
    std::uint64_t id = 0;
    {
        std::lock_guard lock(mutex_);
        id = next_id_++;
        queue_.emplace_back(id, std::move(command));
        results_[id] = Result{};
    }
    wake_.notify_all();
    return id;
}

void SessionRunner::set_latest(Command command) {
    { std::lock_guard lock(mutex_); latest_ = std::move(command); }
    wake_.notify_all();
}

void SessionRunner::set_target(time::Epoch target) {
    { std::lock_guard lock(mutex_); if (!target_ || target > *target_) target_ = target; }
    wake_.notify_all();
}

void SessionRunner::run() {
    std::unique_lock lock(mutex_);
    while (true) {
        wake_.wait(lock, [this] {
            return stop_ || !queue_.empty() || latest_ || (active_ && target_ && reached_ < *target_);
        });
        if (stop_) return;
        if (!queue_.empty()) {
            auto [id, command] = std::move(queue_.front());
            queue_.pop_front();
            running_id_ = id;
            running_since_ = std::chrono::steady_clock::now();
            lock.unlock();
            Result r{true, false, {}};
            try { command(*session_); } catch (const std::exception& e) { r.threw = true; r.error = e.what(); }
            catch (...) { r.threw = true; r.error = "unknown exception"; }
            const bool active = session_->active();
            const auto now = active ? session_->epoch() : reached_;
            observer_(*session_, now);
            lock.lock();
            running_id_ = 0;
            active_ = active;
            reached_ = now;
            results_[id] = r;
            finished_.emplace_back(id, r);
            while (results_.size() > max_kept_results) results_.erase(results_.begin());
            idle_cv_.notify_all();
            continue;
        }
        std::optional<Command> latest = std::move(latest_);
        latest_.reset();
        const auto target = target_;
        lock.unlock();
        if (latest) (*latest)(*session_);
        time::Epoch now = reached_;
        if (session_->active() && target && now < *target) now = session_->advance_to(*target, max_ticks_per_chunk);
        const bool active = session_->active();
        observer_(*session_, now);
        lock.lock();
        active_ = active;
        reached_ = now;
        idle_cv_.notify_all();
    }
}
bool SessionRunner::idle_locked() const {  // mutex_ held
    return queue_.empty() && !latest_ && running_id_ == 0 && (!active_ || !target_ || reached_ >= *target_);
}

bool SessionRunner::idle() const {
    std::lock_guard lock(mutex_);
    return idle_locked();
}

bool SessionRunner::wait_idle(double timeout_s) const {
    std::unique_lock lock(mutex_);
    return idle_cv_.wait_for(lock, std::chrono::duration<double>(timeout_s), [this] { return idle_locked(); });
}

double SessionRunner::busy_s() const {
    std::lock_guard lock(mutex_);
    if (running_id_ == 0) return 0.0;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - running_since_).count();
}

SessionRunner::Result SessionRunner::result(std::uint64_t id) const {
    std::lock_guard lock(mutex_);
    const auto it = results_.find(id);
    if (it == results_.end()) return Result{true, true, "unknown command id"};
    return it->second;
}

std::deque<std::pair<std::uint64_t, SessionRunner::Result>> SessionRunner::take_finished() {
    std::lock_guard lock(mutex_);
    return std::exchange(finished_, {});
}

}  // namespace starsim::sim
```

Notes: `reached_` and `active_` are written only under the lock; the worker reads `reached_` unlocked only between
its own writes (single writer). Wrap the chunk branch (latest command, `advance_to`, observer) in the same
try/catch as a command: the exception text goes to a member `last_error_` (read by `std::string last_error() const`,
under the lock) and the worker keeps running - the GDExtension pushes it as an error on the main thread. Test: post a
`set_latest` command that throws, then `wait_idle`; `last_error() == "Absicht"` and a later command still runs (add
this as a fifth test case in Step 1).

- [ ] **Step 3: Run the tests**

`-TestFilter "[session_runner]"` 4 cases PASS; run them 20 times in a loop
(`for ($i=0;$i -lt 20;$i++){ & <test exe> "[session_runner]" | Select-String "test cases" }`) - no failure, no hang.
Then `-Fast`.

- [ ] **Step 4: Commit and push**

```bash
git add core/include/starsim/sim/session_runner.hpp core/src/sim/session_runner.cpp core/CMakeLists.txt core/tests/test_session_runner.cpp core/tests/CMakeLists.txt core/include/starsim/sim/flight_session.hpp
git commit -m "feat(sim): session runner - the flight session on its own thread"
git push origin main
```

---

### Task 4: `FlightSim` on the simulation thread

`FlightSim` keeps its GDScript method names. Changes of meaning:
- **Commands** (`launch_from_pad`, `launch_from_site`, `start_mission`, `plan_*`, `engage_ascent_autopilot`,
  `disengage_autopilot`, `abort_landing`, `set_booster_return`, `set_realtime_planning`, `set_splashdown_zone`,
  `ignite`, `cutoff`, `release_clamps`, `stage`, `set_rate_mode`): post to the runner and return the command id
  (`int`, > 0; 0 when refused before posting, e.g. no `setup()`). Former `bool` results arrive as
  `result(id) -> {done, ok, message}` and the signal `command_finished(id: int, name: String, ok: bool,
  message: String)`, emitted on the main thread from `advance_to` and `wait_idle`.
- **`set_input`**: the runner's latest slot.
- **`advance_to(tdb_s)`**: sets the target, emits finished commands, returns the snapshot's epoch (the world clock
  already holds back when it is behind).
- **Getters** read the latest snapshot (no session access on the main thread). `predict_trajectory(index, duration,
  step)` returns the snapshot's prediction made for the last requested `(index, duration, step)` (the request is
  stored and the worker recomputes it at most every `prediction_refresh_s = 1.0` [E] wall second - the trajectory view
  refreshes on its own `REFRESH_S`); `target_orbit_points(count)` is built on the main thread from the snapshot's
  target plane normal and radius.
- New: `wait_idle(timeout_s) -> bool`, `planning_s() -> float` (`busy_s()`: the clock stands while > 0),
  `result(id) -> Dictionary`.
- `site_tower(site_id)` stays synchronous (config only, no session).

**Files:**
- Create: `bindings/src/flight_snapshot.hpp`, `bindings/src/flight_snapshot.cpp` (add to `bindings/CMakeLists.txt`)
- Modify: `bindings/src/flight_sim.hpp`, `bindings/src/flight_sim.cpp`
- Modify: `game/vehicle/flight_controller.gd`, `game/vehicle/trajectory_view.gd`, `game/world/world.gd`,
  `game/tests/smoke_test.gd`, `game/tests/render_check.gd`, `game/tests/flight_check.gd`

**Interfaces:**
- Consumes: `sim::SessionRunner` (Task 3): `post`, `set_latest`, `set_target`, `result`, `take_finished`,
  `wait_idle`, `busy_s`.
- Produces (Task 6 extends it):

```cpp
// Everything the main thread reads, built on the simulation thread after every command and chunk (immutable once
// published). Godot containers are created on the worker and only read afterwards.
struct FlightSnapshot {
    bool active = false;
    double epoch_tdb_s = 0.0;
    double mission_time_s = 0.0;
    double mission_start_tdb_s = 0.0;
    godot::String site_id;
    int controlled_index = 0;
    struct Body {
        godot::Vector3 position_gcrf_m, velocity_gcrf_mps;
        godot::Quaternion attitude_gcrf;
        double cg_station_m = 0.0;
        godot::Array stages, engines, flaps, grid_fins, rcs, parachutes;
        godot::Dictionary legs, capsule_state, telemetry, return_state;
    };
    std::vector<Body> bodies;
    godot::Dictionary autopilot_status, tower_state, landing_zone, splashdown_zone;
    bool has_target_orbit = false;
    godot::Vector3 target_plane_normal_gcrf;
    double target_radius_m = 0.0;
    godot::PackedVector3Array prediction;  // for prediction_request
    PredictionRequest prediction_request;   // {index, duration_s, step_s}; index -1: none
};
// Builds the snapshot from the session (worker thread). `previous` supplies the prediction when it is not due.
std::shared_ptr<const FlightSnapshot> build_snapshot(const sim::FlightSession& session, time::Epoch epoch,
                                                     const SnapshotContext& context,
                                                     const FlightSnapshot* previous);
```

`SnapshotContext` carries what the commands set on the worker (`site_id`, `mission_start_tdb_s`, the prediction
request and its last wall time). The body builders are the current getter bodies of `flight_sim.cpp` moved verbatim
into `flight_snapshot.cpp` as free functions taking `const sim::FlightSession&` and the index.

- [ ] **Step 1: Write the failing smoke checks**

In `game/tests/smoke_test.gd` add, and adapt every existing check to the id API (each command call is followed by
`sim.wait_idle(...)` and its `sim.result(id)["ok"]` is checked where the bool was):

```gdscript
# Simulation thread (l6): commands by id, results, idle wait, re-setup while busy.
func _check_thread(config_dir: String, tdb: float) -> bool:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	var id: int = sim.start_mission("starship_leo", tdb)
	_check(id > 0, "start_mission returns a command id")
	_check(sim.wait_idle(60.0), "the simulation thread becomes idle")
	var r: Dictionary = sim.result(id)
	_check(r.get("done", false) and r.get("ok", false), "start_mission finished ok")
	var reached := sim.advance_to(tdb + 2.0)
	_check(sim.wait_idle(30.0) and absf(sim.advance_to(tdb + 2.0) - (tdb + 2.0)) < 1e-6, "advance reaches the target")
	_check(reached <= tdb + 2.0, "advance_to returns the snapshot's epoch, never ahead")
	var bad: int = sim.start_mission("no_such_mission", tdb)
	sim.wait_idle(10.0)
	_check(not sim.result(bad).get("ok", true) and str(sim.result(bad).get("message", "")) != "",
			"unknown mission: refused with a message")
	sim.setup(config_dir)  # re-setup joins the old thread
	_check(not sim.is_active(), "re-setup starts empty")
	return true
```

Call it from `_initialize` like the other check functions. Run the smoke test (command in CLAUDE.md) to
`tmp/l6_t4_smoke.txt`. Expected: FAIL (`wait_idle` / `result` not found).

- [ ] **Step 2: Snapshot struct and builders**

Create `bindings/src/flight_snapshot.hpp/.cpp` with `FlightSnapshot`, `SnapshotContext`, `PredictionRequest` and
`build_snapshot`; move the getter bodies (`body_rcs`, `body_parachutes`, `capsule_state`, `splashdown_zone`,
`landing_zone`, `body_legs`, `body_grid_fins`, `tower_state`, `return_state`, `body_flaps`, `body_stages`,
`body_engines`, `telemetry`, `autopilot_status`) from `flight_sim.cpp` into builder functions. No behaviour change in
their content.

- [ ] **Step 3: FlightSim commands on the runner**

`flight_sim.hpp`: members become

```cpp
    spice::SpiceEphemeris ephemeris_;
    spice::SpiceOrientation earth_ = spice::SpiceOrientation::earth();
    spice::SpiceTimeConverter time_;
    std::filesystem::path config_dir_;
    std::unique_ptr<sim::SessionRunner> runner_;    // owns the session (worker thread)
    mutable std::mutex snapshot_mutex_;
    std::shared_ptr<const FlightSnapshot> snapshot_;  // latest published
    SnapshotContext context_;                       // worker-owned (written by commands)
    std::mutex request_mutex_;
    PredictionRequest prediction_request_;          // main thread writes, worker copies
    std::map<std::uint64_t, godot::String> command_names_;  // main thread: for command_finished
    std::shared_ptr<const FlightSnapshot> snapshot() const;
    std::uint64_t post(const char* name, std::function<godot::Dictionary(sim::FlightSession&)> command);
```

Each command lambda returns `{ok, message}`; the runner's `Result` (exception) maps to `ok = false, message =
error`. Keep a per-id result dictionary map filled from `take_finished()` (main thread). `setup()`:
`runner_.reset()` first (joins), then a new session and runner; `~FlightSim` resets the runner before the SPICE
adapter members are destroyed (declaration order above: `runner_` after the adapters, so it is destroyed first).
Signal: `ADD_SIGNAL(MethodInfo("command_finished", PropertyInfo(Variant::INT, "id"), PropertyInfo(Variant::STRING,
"name"), PropertyInfo(Variant::BOOL, "ok"), PropertyInfo(Variant::STRING, "message")));`

`start_mission` example:

```cpp
int64_t FlightSim::start_mission(const godot::String& mission_id, double tdb_s) {
    const std::string id = mission_id.utf8().get_data();
    return static_cast<int64_t>(post("start_mission", [this, id, tdb_s](sim::FlightSession& session) {
        const auto s = sim::start_mission(session, config_dir_, id, epoch(tdb_s), time_);
        context_.site_id = godot::String::utf8(s.site_id.c_str());
        context_.mission_start_tdb_s = s.epoch.tdb_seconds_past_j2000();
        return command_result(true, "");
    }));
}
```

(`context_` is touched only by the worker: commands and the observer.)

- [ ] **Step 4: Adapt the game scripts**

- `flight_controller.gd`: `launch()` returns `sim.launch_from_site(...) > 0` and sets `last_event` from the
  `command_finished` signal (connect in `setup()`; `launch_site_id` set when ok); keys `R` / `O` / `P`: the refusal
  text from `command_finished` (name `plan_ship_return`: `autopilot_status()["return"]["reason"]` as now).
- `trajectory_view.gd`: unchanged call `sim.predict_trajectory(...)` (now the snapshot's); it may return the previous
  request's points for one refresh.
- `world.gd`: unchanged logic (`advance` returns the reached epoch; the clock holds back).
- `render_check.gd`, `flight_check.gd`: after `launch*` / `start_mission` call `sim.wait_idle(60.0)`; render_check's
  advance loop calls `sim.wait_idle(5.0)` before reading the reached time.

- [ ] **Step 5: Build, smoke, interactive check**

`tools/build.ps1` (with bindings) to `tmp/l6_t4_build.txt`; smoke test PASS (all old and new checks); run
`res://tests/flight_check.gd` (CLAUDE.md) - lift-off and throttle logged as before; record a line in the Ledger.

- [ ] **Step 6: Commit and push**

```bash
git add bindings game/vehicle game/world game/tests
git commit -m "feat(bindings): FlightSim runs its session on the simulation thread"
git push origin main
```

---

### Task 5: Core lunar accessors - terrain altitude and the stored landing site

**Files:**
- Modify: `core/include/starsim/vehicle/flight_body.hpp` (`FlightTelemetry`), `core/include/starsim/sim/flight_session.hpp`,
  `core/src/sim/flight_session.cpp` (`telemetry`, `set_lunar_landing_site`, `set_moon_terrain`)
- Test: `core/tests/test_hls_landing.cpp` (existing Moon fixtures) - new cases `[lunar_telemetry]`

**Interfaces:**
- Produces: `FlightTelemetry::altitude_terrain_m` - above the ground under the body: Moon with terrain: altitude above
  the sphere minus `LunarTerrain::height_m(latitude, longitude)` (MOON_ME, the telemetry's latitude/longitude); Moon
  without terrain and Earth: `altitude_m` (Earth DEM: M8). `const LunarSite* FlightSession::lunar_landing_site() const`
  (nullptr: none; dropped by `set_moon_terrain` as documented). `const environment::LunarTerrain*
  FlightSession::moon_terrain() const`.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST_CASE("telemetry altitude over the lunar terrain", "[lunar_telemetry][terrain][data]") {
    Fixture fx;  // this file's fixture (Moon orientation)
    const auto terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);
    fx.session.set_moon_terrain(terrain);
    const auto site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
    fx.session.set_lunar_landing_site(site);
    REQUIRE(fx.session.lunar_landing_site() != nullptr);
    CHECK(fx.session.lunar_landing_site()->config.id == "mons_mouton_im2");
    CHECK(fx.session.moon_terrain() == terrain.get());
    const auto cfg = hls(100.0e3);
    place_over_site(fx, cfg, 0.5, {0.0, 0.0, -1.0});  // over the site, 0.5 m foot clearance
    const auto t = fx.session.telemetry(0);
    REQUIRE(t.reference_body == ephemeris::Body::Moon);
    const double ground_m = terrain->height_m(t.latitude_rad, t.longitude_rad);
    CHECK(t.altitude_terrain_m == Catch::Approx(t.altitude_m - ground_m).margin(1e-6));
    CHECK(ground_m > 1000.0);              // Mons Mouton stands kilometres above the sphere (site.height_m)
    CHECK(t.altitude_terrain_m > 0.0);     // the CG above the ground below it ...
    CHECK(t.altitude_terrain_m < 60.0);    // ... by tens of metres, not by the ~5 km of the sphere altitude
    fx.session.set_moon_terrain(terrain);  // a new terrain drops the site
    CHECK(fx.session.lunar_landing_site() == nullptr);
}

TEST_CASE("telemetry altitude without terrain is the shape altitude", "[lunar_telemetry]") {
    test::LunarReturnSession s;  // no terrain
    sim::start_mission(s.session, STARSIM_CONFIG_DIR, "ship_lunar_return_llo", {}, s.time);
    const auto t = s.session.telemetry(0);
    CHECK(t.reference_body == ephemeris::Body::Moon);
    CHECK(t.altitude_terrain_m == t.altitude_m);
    CHECK(s.session.moon_terrain() == nullptr);
    CHECK(s.session.lunar_landing_site() == nullptr);
}
```

(Take the landed-HLS placement and site id from the existing `[hls]` drop tests in `test_hls_landing.cpp`; the second
case reuses Task 1's fast planning.) Run `-TestFilter "[lunar_telemetry]"`: FAIL (members missing).

- [ ] **Step 2: Implement**

`FlightTelemetry`: `double altitude_terrain_m = 0.0;  // above the ground below (Moon: terrain if loaded; Earth: = altitude_m until the Earth DEM, M8)`.
`FlightSession::telemetry`:

```cpp
vehicle::FlightTelemetry FlightSession::telemetry(size_t index) const {
    auto t = bodies_.at(index).body->telemetry(env_, epoch_);
    t.altitude_terrain_m = t.altitude_m;
    if (t.reference_body == ephemeris::Body::Moon && moon_terrain_) {
        t.altitude_terrain_m -= moon_terrain_->height_m(t.latitude_rad, t.longitude_rad);
    }
    return t;
}
```

`set_lunar_landing_site` stores `lunar_site_ = site;` (`std::optional<LunarSite> lunar_site_;`), `set_moon_terrain`
resets it; accessors as in Interfaces.

- [ ] **Step 3: Run, commit, push**

`-TestFilter "[lunar_telemetry]"`, `-TestFilter "[hls]"`, then `-Fast`.

```bash
git add core/include/starsim/vehicle/flight_body.hpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/tests/test_hls_landing.cpp
git commit -m "feat(sim): telemetry altitude over the lunar terrain, stored landing site"
git push origin main
```

---

### Task 6: Lunar bindings

**Files:**
- Create: `bindings/src/flight_snapshot_lunar.cpp` (add to `bindings/CMakeLists.txt`)
- Modify: `bindings/src/flight_snapshot.hpp/.cpp`, `bindings/src/flight_sim.hpp/.cpp`, `game/tests/smoke_test.gd`

**Interfaces:**
- Consumes: Task 4 (`FlightSnapshot`, `post`), Task 5 (`altitude_terrain_m`, `lunar_landing_site()`,
  `moon_terrain()`), `FlightSession`: `lunar_step()`, `to_string(LunarStep)`, `lunar_status()`, `lunar_burns()`,
  `lunar_mission_complete()`, `lunar_mission_ended()`, `lunar_descent_plan()`, `lunar_touchdown()`,
  `lunar_return_plan()`, `lunar_return_replans()`, `lunar_pilot_cue()`, `complete_pilot_correction()`,
  `replan_lunar_return(double)`, `reference_body(size_t)`.
- Produces (GDScript):
  - `FlightSim` builds its session with the Moon orientation (`spice::SpiceOrientation::moon_mean_earth()`, member
    `moon_` declared next to `earth_`), so `start_mission` runs `lunar_landing` and `lunar_return` missions.
  - `start_mission` result: `ok = false`, `message = MissionStart::refusal` when `!s.refusal.empty()` (Review Focus 4).
  - `telemetry(i)` adds `reference_body` (`"earth"` / `"moon"`), `altitude_terrain_m`, `periapsis_altitude_km`,
    `apoapsis_altitude_km` (`INF` on an open orbit), `inclination_deg` (osculating, about the reference body - the
    l4 note "95 x 100 km at burn end" is the honest osculating value; documented in the method comment).
  - `lunar_state() -> Dictionary`: `{present}`; present: `step` (German, `to_string`), `status` (German),
    `mission_complete`, `mission_ended`, `burns: [{step, epoch_tdb_s, delta_v_mps, mass_kg}]`, `descent:
    {present, pdi_epoch_tdb_s, touchdown_latitude_deg, touchdown_longitude_deg, touchdown_height_m, slope_deg,
    offset_m, periselene_altitude_km}`, `touchdown: {present, outcome (German), offset_m, sink_mps, lateral_mps}`,
    `return: {present, feasible, reason, tei_delta_v_mps, braking_delta_v_mps, interface_epoch_tdb_s,
    interface_speed_mps, interface_angle_deg, replans}`, `pilot_cue: {present, burn_epoch_tdb_s,
    remaining_delta_v_gcrf_mps (Vector3), remaining_mps, attitude_error_deg, window_end_s}`.
  - `lunar_site() -> Dictionary`: `{present}`; present: `id, name, latitude_deg, longitude_deg, height_m,
    position_moon_me_m (Vector3, on the terrain)`.
  - `moon_terrain(latitude_deg, longitude_deg) -> Dictionary`: `{present}`; present: `height_m` (above the
    1737.4 km sphere), `normal_moon_me` (Vector3), `in_tile` (bool). Computed on the worker? No: the terrain is
    immutable and shared (`std::shared_ptr<const LunarTerrain>`) - the snapshot carries the pointer and the query
    runs on the main thread (no SPICE, no session).
  - Commands: `complete_pilot_correction() -> int`, `replan_lunar_return() -> int` (ok = the session's bool, message
    = `lunar_status()`).

- [ ] **Step 1: Write the failing smoke checks**

```gdscript
# Lunar missions (l6): both kinds start, lunar telemetry, terrain query, refusal, re-plan command.
func _check_lunar(config_dir: String, tdb: float) -> bool:
	var ret := FlightSim.new()
	ret.setup(config_dir)
	var id: int = ret.start_mission("ship_lunar_return_llo", tdb)
	_check(ret.wait_idle(120.0), "lunar return planned within 120 s")
	_check(ret.result(id).get("ok", false), "lunar return mission starts: %s" % ret.result(id).get("message", ""))
	var t: Dictionary = ret.telemetry(ret.controlled_index())
	_check(t.get("reference_body", "") == "moon", "return ship is Moon-referenced")
	_check(absf(float(t.get("altitude_terrain_m", -1.0)) - float(t["altitude_m"])) < 1e-6,
			"no terrain: terrain altitude = shape altitude")
	_check(absf(float(t.get("periapsis_altitude_km", 0.0)) - 100.0) < 15.0, "LLO periselene ~100 km (osculating)")
	var lunar: Dictionary = ret.lunar_state()
	_check(lunar.get("present", false) and lunar["return"].get("feasible", false), "return plan in lunar_state")
	_check(float(lunar["return"].get("tei_delta_v_mps", 0.0)) > 700.0, "TEI delta-v plausible (> 700 m/s)")
	_check(not ret.moon_terrain(-84.8, 0.0).get("present", true), "no terrain loaded for the return mission")
	var rp: int = ret.replan_lunar_return()
	_check(ret.wait_idle(120.0) and ret.result(rp).get("done", false), "re-plan command finishes")

	var land := FlightSim.new()
	land.setup(config_dir)
	var lid: int = land.start_mission("hls_lunar_llo", tdb)
	_check(land.wait_idle(120.0) and land.result(lid).get("ok", false), "lunar landing mission starts")
	var site: Dictionary = land.lunar_site()
	_check(site.get("present", false), "landing site present")
	var ground: Dictionary = land.moon_terrain(site["latitude_deg"], site["longitude_deg"])
	_check(ground.get("present", false) and ground.get("in_tile", false), "terrain query at the site (tile)")
	_check(absf(float(ground["height_m"]) - float(site["height_m"])) < 1.0, "terrain height = site height")
	_check(land.telemetry(land.controlled_index()).get("reference_body", "") == "earth", "HLS starts in LEO")

	var nrho := FlightSim.new()
	nrho.setup(config_dir)
	var nid: int = nrho.start_mission("hls_lunar_nrho", tdb)
	nrho.wait_idle(180.0)
	_check(not nrho.result(nid).get("ok", true) and "nicht durchführbar" in str(nrho.result(nid).get("message", "")),
			"NRHO landing at 120 t is refused honestly")
	return true
```

(Check the refusal text against `start_lunar_landing_mission` - "Mission nicht durchführbar: " + budget - and that
the NRHO landing mission is still refused at 120 t in the l4 ledger; adjust the expected text if the reason differs,
never the mission.) Run the smoke test: FAIL (methods missing).

- [ ] **Step 2: Moon orientation and the refusal**

`FlightSim::setup`: `std::make_unique<sim::FlightSession>(ephemeris_, earth_, config_dir_, {}, &moon_)`;
`start_mission`'s command returns `command_result(s.refusal.empty(), s.refusal)`.

- [ ] **Step 3: Snapshot lunar parts and getters**

`flight_snapshot_lunar.cpp`: `godot::Dictionary build_lunar_state(const sim::FlightSession&)`,
`godot::Dictionary build_lunar_site(const sim::FlightSession&)`; `FlightSnapshot` gets `godot::Dictionary
lunar_state, lunar_site;` and `std::shared_ptr<const environment::LunarTerrain> moon_terrain;` (copied from the
session - add `std::shared_ptr<const environment::LunarTerrain> moon_terrain_shared() const` to `FlightSession` if
`moon_terrain()` returns a raw pointer). Telemetry builder adds the fields of Interfaces. `moon_terrain()` getter:

```cpp
godot::Dictionary FlightSim::moon_terrain(double latitude_deg, double longitude_deg) const {
    godot::Dictionary d;
    d["present"] = false;
    const auto snap = snapshot();
    if (!snap || !snap->moon_terrain) return d;
    const double lat = latitude_deg * constants::deg_to_rad, lon = longitude_deg * constants::deg_to_rad;
    const auto& terrain = *snap->moon_terrain;
    d["present"] = true;
    d["height_m"] = terrain.height_m(lat, lon);
    d["normal_moon_me"] = to_godot(terrain.normal_fixed(terrain.surface_point(lat, lon)));
    d["in_tile"] = terrain.in_tile(lat, lon);
    return d;
}
```

- [ ] **Step 4: Commands**

`complete_pilot_correction` and `replan_lunar_return` via `post(...)`; bind all new methods in `_bind_methods`.

- [ ] **Step 5: Build and smoke**

`tools/build.ps1` (to `tmp/l6_t6_build.txt`) and the smoke test (to `tmp/l6_t6_smoke.txt`): all checks PASS. Record
the smoke's wall time (the three lunar starts).

- [ ] **Step 6: Commit and push**

```bash
git add bindings game/tests/smoke_test.gd
git commit -m "feat(bindings): lunar missions, telemetry, terrain query and re-plan in FlightSim"
git push origin main
```

---

### Task 7: Docs, full suite, review

**Files:**
- Modify: `docs/subsystems.md` (l6 paragraph: simulation thread, snapshot, command ids, CSPICE lock, lunar bindings,
  plan times), `CLAUDE.md` (Game structure: "Flight: ... `FlightSim` runs `sim::SessionRunner` (own thread), getters
  read a snapshot, commands return ids"; drop "Moon orientation yet" remarks), spec status line (l6 implemented),
  `docs/superpowers/specs/2026-10-14-lunar-missions-design.md`.

- [ ] **Step 1: Docs** as above; grep `docs/subsystems.md` for "l6" and resolve every "with l6" note (display items
  stay "with the UI specification").
- [ ] **Step 2: Full suite + smoke**: `tools/build.ps1 > tmp/l6_full.txt 2>&1`, smoke test. All green.
- [ ] **Step 3: Final review**: one fresh reviewer (most capable model) over `git diff <l6 base>..HEAD` with this
  plan and the spec; fix Critical/Important, list minors in the Ledger.
- [ ] **Step 4: Memory, commit, push**: `project-lunar-mission.md` (l6 complete, numbers, next), commit
  `docs: lunar l6 game interface`, push.

---

## Ledger

(Task status, rulings, findings and numbers go here, one line each, prefixed with the task.)
