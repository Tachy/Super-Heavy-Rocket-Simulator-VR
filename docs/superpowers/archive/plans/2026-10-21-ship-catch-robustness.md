# Ship Catch Robustness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every ship return from the gate ends caught or in an upright, controlled abort - the flip brakes the
horizontal velocity towards the catch axis, SCvx takes over at up to 35 deg tilt, and an abort never tumbles.

**Architecture:** Two small pure guidance laws in `gnc` (flip brake, upright descent) with unit tests, wired into
`gnc::ShipLandingGuidance` (flip -> brake -> hand-over -> SCvx; abort -> abort plan or upright descent). A return
result (caught / upright abort / lost) is classified in `sim` from the landing outcome and the guidance's contact
record. Campaign thresholds tighten to "no lost, no miss".

**Tech Stack:** C++20 core, Catch2, `tools/build.ps1 -NoBindings -TestFilter`.

**Spec:** `docs/superpowers/specs/2026-10-21-ship-catch-robustness-design.md`

## Re-scope (user 2026-10-21, after Task 1 Step 8)

The seed-18 reproduction showed a landing-burn tracking oscillation, not a flip that ends too fast across (spec
section "Re-scope"). Changes, binding over the task texts below:

- **Task 1:** the "hard gate" is the replayed seed-18 gate state, not a tuned `Gate`: the hidden replay test of
  `core/tests/test_lunar_return_entry.cpp` is factored into `replay_gate(state16, aero, density, thrust)` returning
  the `CatchRun` with `result`; the seed-18 numbers (`tmp/seed18_gate.txt`, factors 1.014 / 1.105 / 1.019) become a
  constant there. Task 1 commits the classification, the contact record, the gate diagnostics and the replay helper;
  the seed-18 test is added RED in Task 3.
- **Task 2:** the flip brake law is removed again (files, CMake entries, its tests); the upright descent law stays.
- **Task 3 (replaces "Braking flip and hand-over"): landing burn tracking.**
  - Step 1: test `"Lunar return seed-18 gate is caught"` (`[ship][catch][lunar]`): `replay_gate(seed18)` ->
    `result == Caught` and `caught_within_tolerance()`. Run it, expect FAIL ("Abbruch").
  - Step 2: systematic debugging with `STARSIM_DEBUG_SCVX` (tracking lines: `axis` vs `ref`) - candidates in this
    order, one hypothesis at a time, each measured on the replay: (a) the re-plan reference jump (the new plan's
    start attitude vs the old plan's reference at the switch epoch; plan latency 0.5 s); (b) the attitude lag (actuator
    delay 0.75 s vs the LQR effort 0.1); (c) the 3 -> 2 switch (thrust-centroid shift, `switch_prerotation_s` = 0 for
    the ship); (d) plans that start from a state their attitude model cannot follow (rate / tilt at the snapshot).
    Ledger each hypothesis with its numbers.
  - Step 3: fix the cause found (smallest change, [E] values with the measured reason); the replay is caught.
  - Step 4: run `[ship][landing]`, `[ship][catch]~[slow]`, then the `[ship][catch][validation]` campaign (background)
    - not worse than before (20 of 21); commit + push `fix(gnc): ...` naming the cause.
- **Task 4:** unchanged in intent (upright abort); its unreachable-gate test uses `nominal_gate()` with too little
  propellant (not `hard_gate()`); `upright_descent` takes `min_accel_mps2` (Task 2 ruling).
- **Task 5 / 6:** unchanged; docs also correct the seed-12 / seed-18 record in `docs/subsystems.md`.

## Global Constraints

- Answer the user in German; code, comments, commits in English (Conventional Commits); UI strings in the core German.
- Work on `main`, push after every completed task once its tests are green.
- Run only the affected tags: `[ship]`, `[landing]`, `[catch]`, `[return]`, `[lunar]` return cases, the refuelling
  reference flights (tanker catch). Long runs (> 10 min) to the background, check every 10 min.
- Physics first; every new value tagged [E]/[D]/[S] with a reason; no magic numbers (constants from
  `math/constants.hpp`).
- Booster catch, Falcon 9, Dragon, entry guidance and the gate: unchanged (spec "Out of scope").
- `flip_duration_s` / `flip_end_height_m` keep their meaning (the flip proper ends at 15 deg tilt).
- Spec values: flip brake tilt limit 35 deg [E]; hand-over tilt 35 deg [E] at rate < 0.2 rad/s with a converged first
  plan; SCvx tilt limits {35, 20, 15} deg; upright descent tilt <= 15 deg [E], sink <= 3 m/s [E]; upright abort =
  contact tilt < 20 deg [E].

## Review Focus

1. A flip that hands over while the engines are still spooling (thrust ~0): the brake must command the vertical,
   never a NaN axis - unit test in Task 2.
2. The ship far beyond the catch axis (overshot, moving away): the brake must command back towards the axis, not
   further away - unit test in Task 2.
3. Abort requested by the pilot/tower during the flip brake (`request_abort()` before hand-over): the first plan goes
   to the sea, the hand-over floor does not block it - session test in Task 3.
4. Upright descent with the ship directly above the tower centre (no horizontal "away" direction): must pick a
   direction, not divide by zero - unit test in Task 4.
5. Engine failure during the brake (the belly engine fails 1 s into the flip, campaign case): thrust acceleration
   drops, the brake law re-scales, the case ends caught or upright abort - covered by the `[ship][catch]` campaign in
   Task 5 (outcome must not be Lost).

## File Structure

- Create `core/include/starsim/gnc/flip_brake.hpp`, `core/src/gnc/flip_brake.cpp`: the flip brake law (pure).
- Create `core/include/starsim/gnc/upright_descent.hpp`, `core/src/gnc/upright_descent.cpp`: the upright descent law
  (pure).
- Create `core/tests/test_ship_catch_laws.cpp`: unit tests of both laws (`[ship][landing]`).
- Modify `core/include/starsim/gnc/ship_landing.hpp`, `core/src/gnc/ship_landing.cpp`: brake sub-phase, hand-over,
  floor, abort fallback, contact record, gate diagnostics.
- Modify `core/include/starsim/gnc/scvx_landing.hpp`, `core/src/gnc/scvx_landing.cpp`: `PlanStart` mode on `start`,
  `has_abort_plan()` getter (booster behaviour unchanged by default arguments).
- Modify `core/include/starsim/sim/landing_outcome.hpp`, `core/src/sim/landing_outcome.cpp`: `ReturnResult`.
- Modify `core/include/starsim/sim/flight_session.hpp` (+ the .cpp that holds `landing_outcome(size_t)`):
  `ship_return_result(size_t)`.
- Modify `core/tests/return_scenario.hpp` (CatchRun gets `result`), `core/tests/test_ship_catch.cpp`,
  `core/tests/test_lunar_return_campaign.cpp`.
- Modify `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`, `docs/subsystems.md`.

---

### Task 1: Return result, gate diagnostics, reproduction of the miss

**Files:**
- Modify: `core/include/starsim/sim/landing_outcome.hpp`, `core/src/sim/landing_outcome.cpp`
- Modify: `core/include/starsim/gnc/ship_landing.hpp`, `core/src/gnc/ship_landing.cpp` (contact record, `STARSIM_DEBUG_GATE`)
- Modify: `core/include/starsim/sim/flight_session.hpp` + its .cpp (`ship_return_result`)
- Modify: `core/tests/return_scenario.hpp`, `core/tests/test_ship_catch.cpp`
- Test: `core/tests/test_ship_catch.cpp`, `core/tests/test_ship_landing.cpp`

**Interfaces:**
- Produces:
  - `enum class sim::ReturnResult { Flying, Caught, UprightAbort, Lost };`
  - `const char* sim::to_string(ReturnResult)` ("fliegt", "gefangen", "aufrecht abgebrochen", "verloren")
  - `ReturnResult sim::return_result(LandingOutcome outcome, double contact_tilt_rad, double contact_sink_mps)`
  - `inline constexpr double sim::upright_abort_max_tilt_rad` (20 deg [E]), `sim::upright_abort_max_sink_mps` (5 m/s [E])
  - `ShipLandingGuidance::contact_tilt_rad()`, `contact_sink_mps()`, `has_contact()`
  - `FlightSession::ship_return_result(size_t index) const`
  - `test::CatchRun::result` (`sim::ReturnResult`)
  - Test helper `hard_gate()` in test_ship_catch.cpp (the seed-18 flip end) for Tasks 3 and 4.

- [ ] **Step 1: Failing unit test of the classification** (append to `core/tests/test_ship_landing.cpp`, include
  `starsim/sim/landing_outcome.hpp`):

```cpp
TEST_CASE("Ship return result: caught, upright abort or lost", "[ship][landing]") {
    using sim::LandingOutcome;
    using sim::ReturnResult;
    const double deg = constants::deg_to_rad;
    CHECK(sim::return_result(LandingOutcome::Caught, 0.0, 0.0) == ReturnResult::Caught);
    CHECK(sim::return_result(LandingOutcome::Flying, 0.0, 0.0) == ReturnResult::Flying);
    CHECK(sim::return_result(LandingOutcome::Splashdown, 5.0 * deg, 2.5) == ReturnResult::UprightAbort);
    CHECK(sim::return_result(LandingOutcome::Landed, 8.0 * deg, 1.0) == ReturnResult::UprightAbort);
    CHECK(sim::return_result(LandingOutcome::Impact, 12.0 * deg, 4.5) == ReturnResult::UprightAbort);  // upright, slow
    CHECK(sim::return_result(LandingOutcome::Splashdown, 120.0 * deg, 2.0) == ReturnResult::Lost);   // tumbled
    CHECK(sim::return_result(LandingOutcome::Splashdown, 5.0 * deg, 30.0) == ReturnResult::Lost);    // hard
    CHECK(sim::return_result(LandingOutcome::TippedOver, 5.0 * deg, 1.0) == ReturnResult::Lost);
    CHECK(sim::return_result(LandingOutcome::BrokenUp, 0.0, 0.0) == ReturnResult::Lost);
    CHECK(std::string(sim::to_string(ReturnResult::UprightAbort)) == "aufrecht abgebrochen");
}
```

- [ ] **Step 2: Run, expect a compile failure** (`return_result` undefined).

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[ship][landing]"`
Expected: build error `'return_result': is not a member of 'starsim::sim'`.

- [ ] **Step 3: Implement** in `landing_outcome.hpp`:

```cpp
// How a ship return ended (ship catch robustness, 2026-10-21): caught; an upright, controlled abort (water or ground
// reached upright and slow - the ship may be lost, the abort did its job); lost (tumbled, hard impact, broken up).
enum class ReturnResult { Flying, Caught, UprightAbort, Lost };
const char* to_string(ReturnResult result);
inline constexpr double upright_abort_max_tilt_rad = 20.0 * constants::deg_to_rad;  // [E] spec
inline constexpr double upright_abort_max_sink_mps = 5.0;  // [E] the upright descent brakes to <= 3 m/s; margin
ReturnResult return_result(LandingOutcome outcome, double contact_tilt_rad, double contact_sink_mps);
```

and in `landing_outcome.cpp`:

```cpp
const char* to_string(ReturnResult result) {
    switch (result) {
        case ReturnResult::Flying: return "fliegt";
        case ReturnResult::Caught: return "gefangen";
        case ReturnResult::UprightAbort: return "aufrecht abgebrochen";
        case ReturnResult::Lost: return "verloren";
    }
    return "?";
}

ReturnResult return_result(LandingOutcome outcome, double contact_tilt_rad, double contact_sink_mps) {
    switch (outcome) {
        case LandingOutcome::Flying: return ReturnResult::Flying;
        case LandingOutcome::Caught: return ReturnResult::Caught;
        case LandingOutcome::BrokenUp:
        case LandingOutcome::TippedOver: return ReturnResult::Lost;
        default: break;
    }
    const bool upright = contact_tilt_rad < upright_abort_max_tilt_rad && contact_sink_mps <= upright_abort_max_sink_mps;
    return upright ? ReturnResult::UprightAbort : ReturnResult::Lost;
}
```

(include `starsim/math/constants.hpp` in the header if not yet there).

- [ ] **Step 4: Run, expect PASS.** Same command. Expected: all `[ship][landing]` cases pass.

- [ ] **Step 5: Contact record in `ShipLandingGuidance`.** Members `bool contact_ = false; double contact_tilt_rad_ = 0.0;
  double contact_sink_mps_ = 0.0;` and getters `has_contact()`, `contact_tilt_rad()`, `contact_sink_mps()` (comment:
  "first tick resting or impacted: axis tilt from the local vertical and sink speed (Earth-fixed)"). In `update()`,
  before `if (body.resting() || body.impacted()) phase_ = ReturnPhase::Landed;`:

```cpp
    if ((body.resting() || body.impacted()) && !contact_) {
        contact_ = true;
        const Vec3 axis = body.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
        contact_tilt_rad_ = std::acos(std::clamp(dot(axis, up_gcrf), -1.0, 1.0));
        const Vec3 r = body.state().position_gcrf_m;
        const Vec3 v_fixed = body.state().velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(transform), r);
        contact_sink_mps_ = -dot(v_fixed, up_gcrf);
    }
```

  (Use the session's own Earth-rotation helper as `flight_session_return.cpp` / `test_ship_catch.cpp` do:
  `vehicle::earth_rotation_vector(transform)`; check its exact signature in `vehicle/` and include its header.)
  Note: `update()` is not called once `body.impacted()` in `run_ship_landing` (`if (flying.body->impacted()) return;`).
  Move that early return below the `update()` call for ship landings so the contact tick is recorded:

```cpp
void FlightSession::run_ship_landing(Flying& flying) {
    if (flying.body->impacted()) {
        if (!flying.ship_landing->has_contact()) flying.ship_landing->update(*flying.body, env_, epoch_);  // contact record
        return;
    }
```

- [ ] **Step 6: `FlightSession::ship_return_result(size_t index) const`** next to `landing_outcome(size_t)`:

```cpp
// How the ship's return ended (sim::return_result): Flying until contact; without a ship landing guidance the
// landing outcome alone (tilt and sink 0).
ReturnResult ship_return_result(size_t index) const;
```

```cpp
ReturnResult FlightSession::ship_return_result(size_t index) const {
    const auto outcome = landing_outcome(index);
    const auto* g = ship_landing(index);
    if (!g || !g->has_contact()) return return_result(outcome, 0.0, 0.0);
    return return_result(outcome, g->contact_tilt_rad(), g->contact_sink_mps());
}
```

- [ ] **Step 7: CatchRun result + gate diagnostic.** In `return_scenario.hpp` add `sim::ReturnResult result =
  sim::ReturnResult::Flying;` to `CatchRun`. In `test_ship_catch.cpp::fly_from_gate` set
  `run.result = session.ship_return_result(0);` (make `run` non-const) and print `sim::to_string(run.result)` in the
  existing summary line. In `ShipLandingGuidance::update()`, on the first call (`!started_`), when
  `std::getenv("STARSIM_DEBUG_GATE")`, print the gate state in the site frame relative to the catch axis:
  `[gate] to-axis E %.1f N %.1f m, height %.1f m, v E %.1f N %.1f U %.1f m/s` (diagnostics comment); and when the flip
  proper ends (where `flip_duration_s_` is set) print `[flip-end] to-axis ... v ...` the same way. Site-frame east /
  north: `east = normalized(cross(Vec3{0,0,1}, site_up))`, `north = cross(site_up, east)` (fixed frame).

- [ ] **Step 8: Reproduce the miss.** Build, then run the lunar-return single case for seed 18 in the background with
  diagnostics, output to a file:

Run (Bash): `STARSIM_RETURN_SEED=18 STARSIM_DEBUG_GATE=1 STARSIM_DEBUG_FLIP=1 STARSIM_DEBUG_SCVX=1 build/core/tests/starsim_core_tests.exe "[.lunar_return_case]" > tmp/scr_seed18.txt 2>&1` (background, ~8 min)
Expected: `[gate]` and `[flip-end]` lines; flip end ~25 m short, ~29 m/s across; "open"/impacted, not caught.

  Record in the ledger: gate state, flip-end state, the abort's tilt history (`[flip]`/`[scvx]` lines after the abort,
  grep `tilt`), the first abort-plan line (`[scvx] first:` or the re-plan lines: `conv`, `status`) - the cause of the
  tumble is one of: (a) no abort plan converges and the vehicle flies a stale catch plan, (b) the abort plan converges
  but commands a tilt beyond the limits / the LQR saturates, (c) the engines are cut (run_groups empty). Ledger line:
  `Task 1: Finding: tumble cause <a|b|c> - <numbers>`.

- [ ] **Step 9: Hard gate helper.** In `test_ship_catch.cpp`, from the recorded `[gate]` line, add:

```cpp
// The lunar-return direct seed 18 gate (ship catch robustness, 2026-10-21): the flip ends ~25 m short of the catch axis
// at ~29 m/s across; before the flip brake the catch aborted and the abort tumbled.
static Gate hard_gate() {
    Gate g = nominal_gate();
    g.attitude_velocity_enu_mps = g.velocity_enu_mps;  // nose to the tower
    g.offset_enu_m = /* recorded to-axis -> offset from the pad centre: see below */;
    g.velocity_enu_mps = /* recorded gate velocity E, N, U */;
    g.propellant_kg = /* recorded propellant at the gate */;
    return g;
}
```

  The `[gate]` print gives the position relative to the catch axis; the Gate's `offset_enu_m` is relative to the pad
  centre: print the catch axis's ENU offset from the pad centre once (same diagnostic: `axis E %.1f N %.1f`) and add it.
  Replace the three comments with the recorded numbers (literal values, each with "seed 18" in the comment).
  Then a diagnostic case (hidden, it does not catch yet):

```cpp
TEST_CASE("Ship catch: the seed-18 gate", "[.][ship-hard]") {
    const auto run = fly_from_gate(hard_gate());
    std::printf("hard gate: %s\n", sim::to_string(run.result));
}
```

Run: `build/core/tests/starsim_core_tests.exe "[.][ship-hard]" -s > tmp/scr_hard.txt 2>&1` with `STARSIM_DEBUG_GATE=1`
Expected: flip-end within ~5 m / ~3 m/s of the seed-18 flip end; result not "gefangen". If it does not reproduce
(the ship starts from a placed state without entry history), tune the gate velocity along the tower line in 2 m/s
steps until the flip end matches; ledger `Task 1: Ruling: hard gate tuned <values>`.

- [ ] **Step 10: Run the affected fast tags, commit, push.**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[ship][landing]"` and `"[ship][catch]~[slow]"`
Expected: all pass (the nominal gate prints "gefangen").

```bash
git add core/include/starsim/sim/landing_outcome.hpp core/src/sim/landing_outcome.cpp core/include/starsim/gnc/ship_landing.hpp core/src/gnc/ship_landing.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/*.cpp core/tests/return_scenario.hpp core/tests/test_ship_catch.cpp core/tests/test_ship_landing.cpp
git commit -m "feat(sim): ship return result (caught / upright abort / lost) and gate diagnostics"
git push origin main
```

---

### Task 2: Flip brake and upright descent laws

**Files:**
- Create: `core/include/starsim/gnc/flip_brake.hpp`, `core/src/gnc/flip_brake.cpp`
- Create: `core/include/starsim/gnc/upright_descent.hpp`, `core/src/gnc/upright_descent.cpp`
- Create: `core/tests/test_ship_catch_laws.cpp`
- Modify: `core/CMakeLists.txt` (after `src/gnc/ship_landing.cpp`), `core/tests/CMakeLists.txt` (after `test_ship_landing.cpp`)

**Interfaces:**
- Produces:
  - `struct gnc::FlipBrakeParameters { double tilt_max_rad; double brake_fraction; double gain_per_s; double max_speed_mps; };`
  - `math::Vec3 gnc::flip_brake_axis(const math::Vec3& to_axis_m, const math::Vec3& velocity_mps, const math::Vec3& up, double thrust_accel_mps2, const FlipBrakeParameters& p)` - unit thrust axis, any frame (all inputs in the same frame).
  - `struct gnc::UprightDescentParameters { double tilt_max_rad; double gain_per_s; double away_speed_mps; double clear_radius_m; double final_sink_mps; double deceleration_fraction; double sink_gain_per_s; };`
  - `struct gnc::UprightDescentCommand { math::Vec3 axis; double acceleration_mps2; };`
  - `gnc::UprightDescentCommand gnc::upright_descent(const math::Vec3& from_tower_m, const math::Vec3& velocity_mps, const math::Vec3& up, double height_m, double max_accel_mps2, double gravity_mps2, const UprightDescentParameters& p)`

- [ ] **Step 1: Failing tests** (`core/tests/test_ship_catch_laws.cpp`):

```cpp
// Ship catch robustness: the flip brake and the upright descent (pure guidance laws).

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/flip_brake.hpp"
#include "starsim/gnc/upright_descent.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
const Vec3 up{0.0, 0.0, 1.0};
double tilt(const Vec3& axis) { return std::acos(std::clamp(axis.z, -1.0, 1.0)); }
}  // namespace

TEST_CASE("Flip brake: on the axis at rest the thrust stays vertical", "[ship][landing]") {
    const auto a = gnc::flip_brake_axis({0.0, 0.0, 0.0}, {0.0, 0.0, -60.0}, up, 40.0, {});
    CHECK(a.z == Catch::Approx(1.0).margin(1e-9));
}

TEST_CASE("Flip brake: the seed-18 flip end leans fully against the velocity", "[ship][landing]") {
    // 25 m short of the axis (axis along +x), 29 m/s towards it: stopping needs ~17 m/s^2
    const gnc::FlipBrakeParameters p;
    const auto a = gnc::flip_brake_axis({25.0, 0.0, 0.0}, {29.0, 0.0, -50.0}, up, 40.0, p);
    CHECK(tilt(a) == Catch::Approx(p.tilt_max_rad).margin(1e-6));
    CHECK(a.x < 0.0);  // against the motion
    CHECK(std::abs(a.y) < 1e-9);
}

TEST_CASE("Flip brake: far from the axis and slow it leans towards the axis", "[ship][landing]") {
    const auto a = gnc::flip_brake_axis({0.0, 300.0, 0.0}, {0.0, 5.0, -60.0}, up, 40.0, {});
    CHECK(a.y > 0.0);
}

TEST_CASE("Flip brake: overshot and moving away it pushes back", "[ship][landing]") {
    // the axis lies 30 m behind (-x), the ship moves away at 10 m/s (+x)
    const auto a = gnc::flip_brake_axis({-30.0, 0.0, 0.0}, {10.0, 0.0, -40.0}, up, 40.0, {});
    CHECK(a.x < 0.0);
}

TEST_CASE("Flip brake: no thrust yet, the axis is the vertical; tilt never beyond the limit", "[ship][landing]") {
    const auto a = gnc::flip_brake_axis({25.0, 0.0, 0.0}, {29.0, 0.0, -50.0}, up, 0.0, {});
    CHECK(a.z == Catch::Approx(1.0));
    const gnc::FlipBrakeParameters p;
    for (double vx = -60.0; vx <= 60.0; vx += 7.5) {
        for (double d = -200.0; d <= 200.0; d += 37.0) {
            const auto b = gnc::flip_brake_axis({d, 0.5 * d, 0.0}, {vx, -0.3 * vx, -40.0}, up, 35.0, p);
            CHECK(tilt(b) <= p.tilt_max_rad + 1e-9);
            CHECK(std::abs(std::hypot(b.x, b.y, b.z) - 1.0) < 1e-12);
        }
    }
}

TEST_CASE("Upright descent: brakes a fast sink and stays upright", "[ship][landing]") {
    const gnc::UprightDescentParameters p;
    const auto c = gnc::upright_descent({200.0, 0.0, 0.0}, {0.0, 0.0, -40.0}, up, 150.0, 40.0, 9.81, p);
    CHECK(c.acceleration_mps2 > 9.81);
    CHECK(tilt(c.axis) <= p.tilt_max_rad + 1e-9);
}

TEST_CASE("Upright descent: next to the tower it moves away; above its centre it still picks a way", "[ship][landing]") {
    const gnc::UprightDescentParameters p;
    const auto c = gnc::upright_descent({0.0, 20.0, 0.0}, {0.0, 0.0, -5.0}, up, 100.0, 40.0, 9.81, p);
    CHECK(c.axis.y > 0.0);  // away from the tower
    const auto o = gnc::upright_descent({0.0, 0.0, 0.0}, {0.0, 0.0, -5.0}, up, 100.0, 40.0, 9.81, p);
    CHECK(std::isfinite(o.axis.x));
    CHECK(std::hypot(o.axis.x, o.axis.y) > 0.0);
}

TEST_CASE("Upright descent: hovering low and clear of the tower holds about the weight", "[ship][landing]") {
    const gnc::UprightDescentParameters p;
    const auto c = gnc::upright_descent({300.0, 0.0, 0.0}, {0.0, 0.0, -p.final_sink_mps}, up, 1.0, 40.0, 9.81, p);
    CHECK(c.acceleration_mps2 == Catch::Approx(9.81).margin(0.5));
    CHECK(c.axis.z == Catch::Approx(1.0).margin(1e-6));
    for (double vh = -30.0; vh <= 30.0; vh += 6.0) {
        const auto d = gnc::upright_descent({50.0, 10.0, 0.0}, {vh, -vh, -20.0}, up, 80.0, 40.0, 9.81, p);
        CHECK(tilt(d.axis) <= p.tilt_max_rad + 1e-9);
    }
}
```

  Add `test_ship_catch_laws.cpp` to `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Run, expect compile failure** (headers missing).

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[ship][landing]"`
Expected: `cannot open include file 'starsim/gnc/flip_brake.hpp'`.

- [ ] **Step 3: Implement `flip_brake`.**

```cpp
// flip_brake.hpp
#pragma once

#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// The ship's flip brake (ship catch robustness, 2026-10-21): once the flip has the hull below the flaps' tilt, the thrust
// axis leans against the horizontal velocity relative to a reference that points to the catch axis with a stopping
// profile v_ref = min(max_speed, sqrt(2 * brake_fraction * a_max * d)), a_max = thrust acceleration * sin(tilt_max).
struct FlipBrakeParameters {
    double tilt_max_rad = 35.0 * constants::deg_to_rad;  // [E] spec: three Raptors give ~2-3 g, 35 deg ~15-20 m/s^2 across
    double brake_fraction = 0.7;  // [E] the profile plans with this share of a_max (margin for the slew and spool)
    double gain_per_s = 0.8;      // [E] horizontal velocity loop
    double max_speed_mps = 40.0;  // [E] cap of the reference speed towards the axis
};

// Unit thrust axis. to_axis_m: from the CG to the catch axis; velocity_mps: Earth-fixed; up: local vertical (unit); all
// in one frame, vertical parts are ignored. Zero thrust (engines still spooling): the vertical.
math::Vec3 flip_brake_axis(const math::Vec3& to_axis_m, const math::Vec3& velocity_mps, const math::Vec3& up,
                           double thrust_accel_mps2, const FlipBrakeParameters& p = {});

}  // namespace starsim::gnc
```

```cpp
// flip_brake.cpp
#include "starsim/gnc/flip_brake.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

using math::Vec3;

Vec3 flip_brake_axis(const Vec3& to_axis_m, const Vec3& velocity_mps, const Vec3& up, double thrust_accel_mps2,
                     const FlipBrakeParameters& p) {
    if (thrust_accel_mps2 <= 0.0) return up;
    const Vec3 d = to_axis_m - up * dot(to_axis_m, up);
    const Vec3 v = velocity_mps - up * dot(velocity_mps, up);
    const double a_max = thrust_accel_mps2 * std::sin(p.tilt_max_rad);
    const double distance = norm(d);
    Vec3 v_ref{0.0, 0.0, 0.0};
    if (distance > 1e-6) {
        v_ref = d * (std::min(p.max_speed_mps, std::sqrt(2.0 * p.brake_fraction * a_max * distance)) / distance);
    }
    Vec3 a = (v_ref - v) * p.gain_per_s;
    const double a_norm = norm(a);
    if (a_norm > a_max) a = a * (a_max / a_norm);
    const double vertical = std::sqrt(std::max(thrust_accel_mps2 * thrust_accel_mps2 - dot(a, a), 0.0));
    return math::normalized(a + up * vertical);
}

}  // namespace starsim::gnc
```

- [ ] **Step 4: Implement `upright_descent`.**

```cpp
// upright_descent.hpp
#pragma once

#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// The ship's last-resort abort (ship catch robustness, 2026-10-21): when no abort plan converges the ship descends
// upright on a deterministic law - no optimiser, nothing that can fail to converge. Horizontal: away from the tower
// inside clear_radius, else stop; vertical: sink profile max(final_sink, sqrt(2 * f * (a_max - g) * h)).
struct UprightDescentParameters {
    double tilt_max_rad = 15.0 * constants::deg_to_rad;  // [E] spec
    double gain_per_s = 0.5;          // [E] horizontal velocity loop
    double away_speed_mps = 5.0;      // [E] drift away from the tower inside clear_radius
    double clear_radius_m = 60.0;     // [E] tower keep-out (~25 m) + the hull length's lean margin
    double final_sink_mps = 2.0;      // [E] spec: contact at <= 3 m/s
    double deceleration_fraction = 0.5;  // [E] share of the excess acceleration the sink profile plans with
    double sink_gain_per_s = 1.0;     // [E] vertical velocity loop
};

struct UprightDescentCommand {
    math::Vec3 axis;                  // unit thrust axis
    double acceleration_mps2 = 0.0;   // thrust acceleration wanted (<= max_accel)
};

// from_tower_m: from the tower centre to the CG; velocity_mps Earth-fixed; height_m: of the engines above the ground
// or water; max_accel_mps2: of the running engines at full throttle. One frame for all vectors.
UprightDescentCommand upright_descent(const math::Vec3& from_tower_m, const math::Vec3& velocity_mps, const math::Vec3& up,
                                      double height_m, double max_accel_mps2, double gravity_mps2,
                                      const UprightDescentParameters& p = {});

}  // namespace starsim::gnc
```

```cpp
// upright_descent.cpp
#include "starsim/gnc/upright_descent.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

using math::Vec3;

UprightDescentCommand upright_descent(const Vec3& from_tower_m, const Vec3& velocity_mps, const Vec3& up, double height_m,
                                      double max_accel_mps2, double gravity_mps2, const UprightDescentParameters& p) {
    Vec3 away = from_tower_m - up * dot(from_tower_m, up);
    const double distance = norm(away);
    if (distance < 1e-3) {  // straight above the tower centre: any horizontal direction
        away = math::normalized(cross(up, std::abs(up.x) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0}));
    } else {
        away = away / distance;
    }
    const Vec3 v_h = velocity_mps - up * dot(velocity_mps, up);
    const Vec3 v_ref = distance < p.clear_radius_m ? away * p.away_speed_mps : Vec3{0.0, 0.0, 0.0};
    Vec3 a_h = (v_ref - v_h) * p.gain_per_s;

    const double sink = -dot(velocity_mps, up);
    const double excess = std::max(max_accel_mps2 - gravity_mps2, 0.0);
    const double wanted_sink =
        std::max(p.final_sink_mps, std::sqrt(2.0 * p.deceleration_fraction * excess * std::max(height_m, 0.0)));
    const double a_v = std::clamp(gravity_mps2 + p.sink_gain_per_s * (sink - wanted_sink), 0.0, max_accel_mps2);

    // tilt limit: the horizontal part never exceeds tan(tilt_max) of the vertical part (and the total stays <= max)
    const double h_max = std::min(a_v * std::tan(p.tilt_max_rad), std::sqrt(std::max(max_accel_mps2 * max_accel_mps2 - a_v * a_v, 0.0)));
    const double a_h_norm = norm(a_h);
    if (a_h_norm > h_max) a_h = a_h_norm > 0.0 ? a_h * (h_max / a_h_norm) : a_h;
    const Vec3 a = a_h + up * a_v;
    const double a_norm = norm(a);
    if (a_norm <= 0.0) return {up, 0.0};
    return {a / a_norm, a_norm};
}

}  // namespace starsim::gnc
```

  Note for the "above the centre" test: inside `clear_radius` the reference points `away`, so the axis gets a
  horizontal component. Add both .cpp files to `core/CMakeLists.txt` after `src/gnc/ship_landing.cpp`.

- [ ] **Step 5: Run, expect PASS.** Same command. Expected: all `[ship][landing]` cases pass.

- [ ] **Step 6: Commit and push.**

```bash
git add core/include/starsim/gnc/flip_brake.hpp core/src/gnc/flip_brake.cpp core/include/starsim/gnc/upright_descent.hpp core/src/gnc/upright_descent.cpp core/tests/test_ship_catch_laws.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat(gnc): flip brake and upright descent laws for the ship catch"
git push origin main
```

---

### Task 3: Braking flip and hand-over to SCvx

**Files:**
- Modify: `core/include/starsim/gnc/ship_landing.hpp`, `core/src/gnc/ship_landing.cpp`
- Modify: `core/include/starsim/gnc/scvx_landing.hpp`, `core/src/gnc/scvx_landing.cpp`
- Test: `core/tests/test_ship_catch.cpp`

**Interfaces:**
- Consumes: `gnc::flip_brake_axis`, `gnc::FlipBrakeParameters` (Task 2); `hard_gate()`, `CatchRun::result` (Task 1).
- Produces:
  - `enum class gnc::PlanStart { CatchOrAbort, CatchOnly };` and
    `bool ScvxLandingGuidance::start(const vehicle::FlightBody&, const vehicle::FlightEnvironment&, time::Epoch, PlanStart mode = PlanStart::CatchOrAbort)`:
    `CatchOnly` returns false on an unconverged catch plan without switching to the abort (no state change except
    the statistics).
  - `ShipLandingParameters`: `FlipBrakeParameters flip_brake;`, `double handover_tilt_max_rad = 35 deg;`,
    `double handover_floor_m = 250.0;` [E, refined in Step 6 from the measured flip-end heights].
  - `ShipLandingGuidance::handover_s()`, `handover_height_m()` (0 until the hand-over).

- [ ] **Step 1: Failing test** (`core/tests/test_ship_catch.cpp`): replace the hidden `[.][ship-hard]` case with

```cpp
TEST_CASE("Ship catch: the seed-18 gate is caught after the flip brake", "[ship][landing][catch]") {
    const auto run = fly_from_gate(hard_gate());
    CHECK(run.result == sim::ReturnResult::Caught);
    CHECK(run.caught_within_tolerance());
}

TEST_CASE("Ship catch: an abort requested in the flip goes to the sea upright", "[ship][landing][catch]") {
    // Review focus 3: the pilot gives the catch up before the hand-over - the floor must not hold the first plan back.
    Gate g = nominal_gate();
    g.abort_in_flip = true;
    const auto run = fly_from_gate(g);
    CHECK(run.result == sim::ReturnResult::UprightAbort);
}
```

  with a new `Gate` field `bool abort_in_flip = false;  // request_abort() 1 s into the flip` and in `fly_from_gate`
  after placing: `if (gate.abort_in_flip) { session.advance_to(t0 + 1.0, 1000); session.request_ship_abort(0); }` -
  use the session's existing pilot/tower abort entry point (grep `request_abort` in `flight_session*.hpp`; if only the
  guidance has it, call `const_cast`-free through the session's command that the bindings use; ledger the name).

- [ ] **Step 2: Run, expect FAIL** on the seed-18 case (not caught) - and record whether the abort case passes already.

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[ship][catch]~[slow]"`
Expected: "the seed-18 gate is caught" FAILS with result `Lost` or `UprightAbort`.

- [ ] **Step 3: `PlanStart` in `ScvxLandingGuidance::start`.** In `scvx_landing.hpp` add the enum and the defaulted
  parameter. In `start()`: right after the first `plan_now`, before the "out of reach from the start" branch:

```cpp
    if (!outcome.plan->converged && mode == PlanStart::CatchOnly && !aborting_) {
        ++stats_.plans;
        ++stats_.failures;
        return false;  // the flip brake flies on and tries again
    }
```

- [ ] **Step 4: Brake sub-phase and hand-over in `ShipLandingGuidance::update()`** (Flip phase):
  - Thrust acceleration: sum of `e.spool * e.performance->thrust_n(e.throttle, 0.0)` over producing engines (as the
    debug print) divided by `body.vehicle().mass_properties().mass_kg` (check the field name).
  - Catch axis and Earth-fixed velocity: `r_fixed`, `v_fixed = transpose(R) * (v_gcrf - earth_rate x r_gcrf)`, the
    axis at the CG height `axis_point = pin_point_itrf_m` (horizontal part only matters), `to_axis = axis_point - r_fixed`.
  - Below `flip_flaps_until_tilt_rad` (the flaps are released):

```cpp
            const Vec3 axis_fixed = flip_brake_axis(to_axis, v_fixed, site_up, thrust_accel, parameters_.flip_brake);
            cmd.axis_gcrf = transform.rotation * axis_fixed;
```

    above it `cmd.axis_gcrf = up_gcrf` as today.
  - Hand-over condition replaces `tilt < parameters_.flip_tolerance_rad && settled`:
    `tilt < parameters_.handover_tilt_max_rad && settled && epoch >= next_attempt_ && site_.catch_target`.
  - Start mode: `const bool above_floor = height_above_site > parameters_.handover_floor_m && !abort_requested_;`
    `guidance->start(body, env, epoch, above_floor ? PlanStart::CatchOnly : PlanStart::CatchOrAbort)`.
  - Record `handover_s_ = epoch - flip_start_` and `handover_height_m_` when the hand-over succeeds.
  - Keep the `flip_duration_s_` / `flip_end_height_m_` record at `flip_tolerance_rad` (15 deg) unchanged.
  - Extend the `[flip]` debug line with `brake %.1f deg to-axis %.1f m v_h %.1f m/s`.
  - Update the class comment (Flip / LandingBurn) with the brake and the hand-over.

- [ ] **Step 5: SCvx tilt limits** in `ship_landing_parameters()`:

```cpp
    // [E] ship catch robustness 2026-10-21: the plans start at the flip brake's hand-over (up to 35 deg tilt); the
    // three-engine phase may lean that far (three Raptors ~2-3 g: ~15-20 m/s^2 across). The lateral phase took 15 deg
    // until f3: it saturated on the divert after the flip (lunar return seed 1, ~27 m/s across: overshoot, hover, headers
    // dry), then 20 deg.
    l.tilt_max_rad = {35.0 * deg, 20.0 * deg, 15.0 * deg};
```

- [ ] **Step 6: Run the seed-18 gate and the fast catch tags; tune within the [E] ranges.**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[ship][catch]~[slow]"` and `"[ship][landing]"`
Expected: PASS, seed-18 gate "gefangen"; nominal gate still caught with its propellant within 0.5 t of the Task-1
print (ledger the before/after numbers).
  If the seed-18 gate is not caught: check with `STARSIM_DEBUG_FLIP=1` that the brake leans (brake angle > 0 below
  30 deg tilt) and the hand-over waits for a converged plan; tune in this order, one at a time, ledger each:
  `brake_fraction` 0.7 -> 0.5, `gain_per_s` 0.8 -> 1.2, `handover_floor_m` from the printed hand-over heights
  (floor = lowest successful hand-over height of the passing cases minus 50 m, not below 150 m).

- [ ] **Step 7: Commit and push.**

```bash
git add core/include/starsim/gnc/ship_landing.hpp core/src/gnc/ship_landing.cpp core/include/starsim/gnc/scvx_landing.hpp core/src/gnc/scvx_landing.cpp core/tests/test_ship_catch.cpp
git commit -m "feat(gnc): the ship's flip brakes towards the catch axis, hand-over at 35 deg on a converged plan"
git push origin main
```

---

### Task 4: Upright abort

**Files:**
- Modify: `core/include/starsim/gnc/ship_landing.hpp`, `core/src/gnc/ship_landing.cpp`
- Modify: `core/include/starsim/gnc/scvx_landing.hpp`, `core/src/gnc/scvx_landing.cpp` (only per the Task-1 finding)
- Test: `core/tests/test_ship_catch.cpp`

**Interfaces:**
- Consumes: `gnc::upright_descent` (Task 2); Task-1 ledger finding (tumble cause a/b/c); `PlanStart` (Task 3).
- Produces:
  - `bool ScvxLandingGuidance::has_abort_plan() const` - an abort plan is being flown (`plan_ && plan_->goal == DescentGoal::Abort`).
  - `ShipLandingParameters::upright_descent` (`UprightDescentParameters`), `double upright_guard_tilt_rad = 25 deg` [E].
  - `bool ShipLandingGuidance::upright_descent_active() const`.

- [ ] **Step 1: Failing test - an unreachable gate ends upright** (`test_ship_catch.cpp`):

```cpp
TEST_CASE("Ship catch: an unreachable catch ends in an upright abort", "[ship][landing][catch]") {
    // The seed-18 gate with too little propellant for any catch: the catch aborts; the ship must come down upright.
    Gate g = hard_gate();
    g.propellant_kg = 9000.0;  // [E] below the flip (~8 t) + a landing burn: chosen so the catch is out of reach
    const auto run = fly_from_gate(g);
    CHECK(run.result == sim::ReturnResult::UprightAbort);
    CHECK_FALSE(run.tower_struck);
}
```

  If 9 t still catches after Task 3, lower in 1 t steps until the catch aborts (ledger the value). Also re-run the
  pre-Task-3 behaviour is not needed; this test pins the abort.

- [ ] **Step 2: Run, expect FAIL** (result `Lost`, tumble).

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[ship][catch]~[slow]"`
Expected: "an unreachable catch ends in an upright abort" FAILS (Lost). If it passes, the Task-1 tumble needs the
seed-18 state itself: replace the gate by the Task-1 seed-18 gate *without* the flip brake
(`g.parameters.flip_brake.tilt_max_rad = 0.0`) to reproduce the tumbling abort; ledger the ruling.

- [ ] **Step 3: Fix the cause found in Task 1.**
  - (a) no abort plan converges / stale catch plan flown: nothing to fix in SCvx - Step 4's fallback covers it.
  - (b) the abort plan commands beyond the limits: in `ScvxLandingGuidance::request(...)` the abort request keeps
    `settings.limits` (check that `aborting_` only clears `min_phase_s`, and that the abort target does not lift the
    tilt limit); fix the found override, comment it with the finding.
  - (c) the engines are cut on the abort: the abort command must keep the plan's run groups; fix where `run_groups`
    is emptied on `abort`, comment it with the finding.
  Ledger `Task 4: fixed cause <a|b|c>: <what>`.

- [ ] **Step 4: Fallback upright descent in `ShipLandingGuidance::update()`** (LandingBurn phase, after reading `c`):

```cpp
        // Ship catch robustness: an abort that has no converged abort plan, or that leans the hull beyond the guard,
        // descends upright on a deterministic law instead (sticky).
        if (c.abort && !upright_descent_ && (!scvx_->has_abort_plan() || tilt > parameters_.upright_guard_tilt_rad)) {
            upright_descent_ = true;
        }
        if (upright_descent_) {
            fly_upright_descent(body, env, epoch, cmd);  // sets run_groups, throttle, axis, abort = true
        }
```

  `fly_upright_descent`: engines = `flip_groups` while sink > 20 m/s, then the two-engine group
  (`parameters_.scvx.snapshot.phase_groups[1]`); `max_accel` from those engines at full throttle (`thrust_n(1.0, 0.0)`
  per engine) over the mass; `from_tower` = CG - `catch_target->tower_center_itrf_m` (fixed frame); height = CG height
  above the site minus `mass.cg_station_m` (engines at the bottom); gravity `constants::standard_gravity_mps2`;
  `throttle = clamp(c.acceleration_mps2 / max_accel, engine minimum, 1.0)`; `cmd.axis_gcrf = R * c.axis`;
  `cmd.scvx = false; cmd.abort = true; cmd.feedforward.reset(); cmd.torque_body_nm.reset();` flaps folded. Also call
  it from the Flip phase when the hand-over at/below the floor failed (`start` returned false with `CatchOrAbort`).
  `tilt` in the LandingBurn phase: compute it as in the Flip phase (hoist the computation above both phases).

- [ ] **Step 5: Run, expect PASS** (unreachable gate upright; the seed-18 gate still caught; nominal caught).

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[ship][catch]~[slow]"` and `"[ship][landing]"`
Expected: PASS. Ledger the contact tilt and sink of the abort case.

- [ ] **Step 6: Commit and push.**

```bash
git add core/include/starsim/gnc/ship_landing.hpp core/src/gnc/ship_landing.cpp core/include/starsim/gnc/scvx_landing.hpp core/src/gnc/scvx_landing.cpp core/tests/test_ship_catch.cpp
git commit -m "fix(gnc): ship catch aborts come down upright (abort limits, upright descent fallback)"
git push origin main
```

---

### Task 5: Campaign thresholds

**Files:**
- Modify: `core/tests/test_ship_catch.cpp:149-170`, `core/tests/test_lunar_return_campaign.cpp:253-264`

**Interfaces:**
- Consumes: `CatchRun::result` (Task 1); the lunar return `fly()` result gets the session's `ship_return_result(0)`
  (add `std::string result;` or the enum to its result struct, filled where `r.caught` is set).

- [ ] **Step 1: Tighten the ship catch campaign:**

```cpp
    int caught = 0, lost = 0;
    ...
        if (run.caught_within_tolerance()) ++caught;
        if (run.result == sim::ReturnResult::Lost) ++lost;
    ...
    std::printf("caught %d of %zu, lost %d\n", caught, cases.size(), lost);
    CHECK(caught >= 20);  // ship catch robustness: the engine-failure case may abort, upright
    CHECK(lost == 0);
```

- [ ] **Step 2: Lunar return direct cases:** `CHECK(fly_seeds(a, a + 2, false) == 3)` for all seven cases; replace the
  "one miss allowed" comment by: "Since the ship catch robustness plan (2026-10-21) the flip brakes towards the catch
  axis: all 21 direct seeds are caught (seed 18 was the f3 miss)." Print the return result in `fly_seeds`' line.

- [ ] **Step 3: Run the campaigns** in the background (parallel runner from the scratchpad, binary copy; each lunar
  case ~8-9 min, the ship campaign ~5 min):

Run: `"[ship][catch][validation]"` and `"[lunar][return][validation]"` (direct + aerocapture), output to `tmp/scr_campaigns_*.txt`
Expected: ship campaign caught >= 20, lost 0; lunar direct 21/21, aerocapture 21/21. Check every 10 min; a miss ->
stop, diagnose with `[.lunar_return_case]` / `[.][ship-single]` and the debug flags, fix in Task 3/4 code (ledger).

- [ ] **Step 4: Booster campaign unaffected:** run `"[scvx][catch]"` (fast) and the booster SCvx campaign
  (`test_scvx_catch.cpp`, its `[slow]` tag) - `start()`'s default `CatchOrAbort` keeps the booster's behaviour.
Expected: unchanged (>= 19 of 21 as before).

- [ ] **Step 5: Commit and push.**

```bash
git add core/tests/test_ship_catch.cpp core/tests/test_lunar_return_campaign.cpp
git commit -m "test: ship catch campaigns allow no miss and no lost ship"
git push origin main
```

---

### Task 6: Reference flights, docs, final review

**Files:**
- Modify: `config/refuelling/reference_flights.toml` (only if the tanker catch numbers move beyond the file tolerance)
- Modify: `docs/subsystems.md` (ship flip and catch section, lines ~195-238; the summary ~1545)
- Modify: `CLAUDE.md` only if a module list changes (it does not: gnc gains laws only - leave it)

- [ ] **Step 1: Refuelling reference flights** (tanker return + catch): run the slow reference-flight cases of
  `core/tests/test_refuelling_missions.cpp` (grep its tags; background, parallel runner).
Expected: pass. If "The measured numbers belong to the current configs" or a tolerance check fails because the
catch propellant moved, update the measured value in `reference_flights.toml` (the inputs digest is unchanged: no
config change) and re-run `build/core/refuelling_report.exe`; expect 13 tankers / 49.7 d (ledger the answer).

- [ ] **Step 2: Docs** in `docs/subsystems.md` ship flip and catch:
  - the flip brake (law, 35 deg, brake_fraction / gain / max speed [E]), the hand-over (35 deg, converged plan,
    floor), SCvx tilt {35, 20, 15};
  - the upright abort (finding of Task 1, fix, upright descent law, guard tilt), the return result classification;
  - validation numbers: nominal gate, seed-18 gate (caught, propellant left), unreachable gate (contact tilt / sink),
    campaigns (ship X of 21, lost 0; lunar direct 21/21);
  - delete the "Open (own follow-up ship catch robustness)" text and the "allows one miss" rationale;
  - diagnostics: `STARSIM_DEBUG_GATE`, the extended `[flip]` line.

- [ ] **Step 3: Commit, push, final review** (fresh reviewer, most capable model, per executing-plans), fix pass,
  memory update (`project-orbital-refuelling.md`: ship catch robustness done; NEXT f2).

```bash
git add docs/subsystems.md config/refuelling/reference_flights.toml
git commit -m "docs: ship catch robustness"
git push origin main
```

## Plan decisions (for the user's approval)

1. **Booster SCvx campaign (`test_scvx_catch.cpp`) keeps >= 19 of 21.** The spec named "the SCvx catch campaign";
   that file is the *booster's* campaign, and the booster catch is out of scope. The shared SCvx code changes only
   behind a defaulted parameter, so the booster's behaviour is unchanged.
2. **Return result thresholds:** upright abort = contact tilt < 20 deg [E] (spec) and sink <= 5 m/s [E] (the upright
   descent aims at <= 3 m/s; 5 m/s leaves a margin, harder counts as lost).
3. **Brake values [E]:** tilt 35 deg, brake_fraction 0.7, gain 0.8 /s, max speed 40 m/s; hand-over floor 250 m
   (refined from the measured hand-over heights, not below 150 m).
4. **Upright descent values [E]:** tilt 15 deg, away 5 m/s inside 60 m of the tower centre, final sink 2 m/s, guard
   tilt 25 deg (an abort leaning beyond it switches to the law).
5. **Reproduction gate:** the seed-18 gate is placed from its recorded state (fast `[ship][catch]` case, no 8-min
   lunar flight in the fast tags).
