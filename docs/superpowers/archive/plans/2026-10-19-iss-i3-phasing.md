# ISS i3: Phasing from the separation orbit to Waypoint 0 - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Crew Dragon in its separation orbit plans and flies the coelliptic rendezvous with the ISS on its Dracos -
boost, coelliptic burn, approach initiation, mid-course correction and braking - and arrives at Waypoint 0 (400 m below
the ISS) with a near-zero relative velocity, re-planning from the live state when the flight leaves the plan.

**Architecture:** Three new planning blocks and one session step machine. `orbit/clohessy_wiltshire` gives the linear
relative motion (first guesses, timing). `mission/rendezvous_geometry` names the target-relative aim points and the
approach ellipsoid. `mission/phasing_schedule` picks the burn epochs analytically (phase rates with J2), and
`mission/phasing_planner` targets each burn in the full force model with orbital drag (`PositionTargeter` gets a drag
term, both bodies are coasted with their own C_D A / m). `FlightSession::plan_rendezvous` flies the burns through the
existing `execute_burn` (Draco translation along +x), re-targets every burn from the live states before it, tracks
both bodies' drag from their arcs and re-plans the remaining sequence when the predicted phase error at the boost grows
beyond a tolerance.

**Tech Stack:** C++20, Catch2, CSPICE only through the `spice` adapters (tests).

**Spec:** `docs/superpowers/specs/2026-10-19-iss-docking-design.md` (sections "Intent", "i3 design: phasing", "Error
handling", "Testing").

## Research done while planning (2026-10-19) and the spec corrections it implies

Real Crew Dragon profile, Crew-2 timeline (Spaceflight Now, 2021-04-22; times EDT): separation 06:01, "Phase burn"
06:38, "Boost burn" 21:25 (+15.4 h), "Close burn" 22:10, "Transfer burn" 00:07, "Coelliptic burn" 00:53, range 30 km
at 02:10, 15 km at 03:01, "Out of Plane burn" 02:49, 10 km at 03:18, **"Approach initiation burn" 03:40 at a range of
7.5 km**, "Approach initiation midcourse burn" 04:05, go/no-go to enter the 200 m keep-out sphere 04:15, **Waypoint 0
arrival 04:25, 400 m below the ISS**, Waypoint 1 04:49 (220 m in front), Waypoint 2 05:00 (20 m), contact 05:10.
Demo-2 burn sizes (everydayastronaut.com timeline): phase 16.11 m/s, boost 44.2 m/s, circularisation 57.89 m/s.

Derived numbers [D]: the coelliptic closing rate between 30 and 10 km is 15 km / 51 min = 4.9 m/s; the CW coelliptic
rate is 1.5 n dh, so dh = 4.9 / (1.5 x 1.128e-3) = **2.9 km** below the ISS; the approach initiation -> Waypoint 0
transfer takes **45 min** (~0.48 revolution); the mid-course correction comes **25 min** after the approach initiation;
the coelliptic coast lasts **2.8 h** (00:53 -> 03:40).

Spec corrections (to be confirmed by the user with this plan; Task 6 writes them into the spec):
1. **i3 ends at Waypoint 0** (400 m below the ISS, braked to zero LVLH relative velocity), not at an "approach
   ellipsoid entry point": the real approach initiation burn starts 7.5 km out, its transfer crosses the approach
   ellipsoid (4 x 2 x 2 km) and ends at Waypoint 0. The go for the approach initiation is the clearance to enter the
   ellipsoid. i4 starts at Waypoint 0.
2. **i3 is the same for both ports:** Crew-2 went to IDA-2 (forward) through Waypoint 0 below the station; the port
   only matters from Waypoint 1 on (i4). The phasing campaign therefore runs over phase angles, not ports.
3. **The phase burn is used only when an earliest arrival is set:** a phase burn and the boost at the same perigee add
   up to the same Hohmann transfer, so every revolution count costs the same in-plane delta-v and the "cheapest correct
   window" is the earliest one - the Dragon waits on the separation orbit (fastest catch-up) and boosts when the phase
   is right. Real flights slow the catch-up with the phase burn to fit the crew's day; `earliest_arrival_s` does that
   (and a test pins it).
4. **The orbit plane:** the J2 node regression differs by ~0.58 deg/day between 205 km and 415 km (0.46 deg of wedge,
   ~60 m/s if all corrected at the end). Real launches aim at a plane that drifts onto the ISS plane; the tests build
   the start state that way (RAAN biased by the differential regression until the boost). The i6 launch-window
   planner must do the same. The remaining plane error is removed by the 3D-targeted coelliptic and approach burns
   (no separate out-of-plane burn).

## Global Constraints

- `core/` has no Godot dependency; modules depend downwards only: `orbit` may use `frames`, `math`, `environment`;
  `mission` may use `orbit` and `gnc`; `sim` uses everything. Nothing in `orbit` or `mission` includes `sim`.
- SI units internally, angles in radians; names carry frame and unit (`position_gcrf_m`, `delta_v_gcrf_mps`).
- No magic numbers: tuned values are named constants or `PhasingParameters` fields with `[S]` / `[E]` / `[D]` tags and
  a reason (source in a comment).
- User-facing refusal reasons and status lines are German (`"Kein Rendezvous: ..."`); programming errors stay English.
- Physics first (memory `feedback-physics-over-gauges`): no phantom terms to match published burn sizes.
- Re-plan rule (memory `feedback-replan-from-flight-data`): plan the cheapest correct window, follow and correct it;
  when a correction fails, re-plan from the live state.
- Commit on `main` (Conventional Commits) and push after every task once its tests are green; `-Fast` before every
  push; full suite before the final review (this plan touches guidance and flight sessions).
- ctest runs test cases as parallel processes: no shared temp files. Tag test cases over ~40 s `[slow]`.
- Long command output to `tmp/` (git-ignored), read tails / grep. Never filter `-TestFilter "[mission]"`.
- Do not use Bash heredocs to write files: use the Write tool (memory `feedback-no-heredocs`).
- Test tags: `[rendezvous]` on every new test, plus `[cw]`, `[phasing]`, `[session]`, `[validation]` as given.

## Review Focus

1. **Drag makes the long wait drift:** a day at ~205 km moves the Dragon ~70 km along track against a drag-free
   prediction, and the onboard C_D A / m guess can be off by tens of percent. The flight must still reach Waypoint 0.
   Test: Task 5 Step 1 (`Rendezvous flight with a denser truth atmosphere re-plans and arrives`).
2. **A phase that cannot be closed within the mission limit** (ISS just ahead, or ~300 deg ahead with a 28 h limit) is
   refused with a German reason, never planned past the limit. Test: Task 4 Step 1 (`Phasing plan refuses a phase
   beyond the mission limit`), Task 3 Step 1 (schedule cases).
3. **The planned arc must not enter the approach ellipsoid before the approach initiation** (passing under the
   station early). Test: Task 4 Step 1 (campaign checks `min_ellipsoid_metric_before_ai > 1`), Task 5 Step 1 (flown
   arc sampled every 60 s).
4. **A start above the coelliptic height** (or an orbit that is not closed) cannot be phased from below: refused, no
   NaN epochs. Test: Task 3 Step 1 (`Schedule refuses a chaser above the coelliptic height`).
5. **Too little propellant:** the plan reports the budget against the Dracos' propellant and refuses when the reserve
   (deorbit ~600 kg + proximity) is not left. Test: Task 4 Step 1 (`Phasing plan refuses without the reserve`).

---

## File structure

| File | Responsibility |
|---|---|
| `core/include/starsim/orbit/clohessy_wiltshire.hpp`, `core/src/orbit/clohessy_wiltshire.cpp` | CW propagation, two-impulse transfer, LVLH relative state |
| `core/include/starsim/mission/position_targeter.hpp`, `core/src/mission/position_targeter.cpp` | Optional orbital drag in the targeter's coast |
| `core/include/starsim/mission/rendezvous_geometry.hpp`, `core/src/mission/rendezvous_geometry.cpp` | Aim points behind/below the target, approach ellipsoid metric, KOS radius |
| `core/include/starsim/mission/phasing_schedule.hpp`, `core/src/mission/phasing_schedule.cpp` | `PhasingParameters`, analytic burn epochs (wait / phase burn, extra laps) |
| `core/include/starsim/mission/phasing_planner.hpp`, `core/src/mission/phasing_planner.cpp` | Full-model targeting of each burn, plan + budget, in-flight retarget, phase-error check |
| `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`, `core/src/sim/flight_session_rendezvous.cpp` | `plan_rendezvous`, `RendezvousStep`, drag tracking, re-plan |
| `core/tests/test_clohessy_wiltshire.cpp`, `test_rendezvous_geometry.cpp`, `test_phasing_schedule.cpp`, `test_phasing_planner.cpp`, `test_rendezvous_phasing_session.cpp`, `rendezvous_test_support.hpp` | Tests |
| `core/CMakeLists.txt`, `core/tests/CMakeLists.txt` | New sources and tests |
| `docs/subsystems.md`, `CLAUDE.md`, the spec | Documentation (Task 6) |

---

### Task 1: Clohessy-Wiltshire relative motion

**Files:**
- Create: `core/include/starsim/orbit/clohessy_wiltshire.hpp`, `core/src/orbit/clohessy_wiltshire.cpp`
- Create: `core/tests/test_clohessy_wiltshire.cpp`
- Modify: `core/CMakeLists.txt` (add `src/orbit/clohessy_wiltshire.cpp` after `src/orbit/synthetic_station_orbit.cpp`),
  `core/tests/CMakeLists.txt` (append `test_clohessy_wiltshire.cpp`)

**Interfaces:**
- Consumes: `frames::lvlh_to_inertial(state)` (LVLH -> inertial quaternion; x along the velocity, y = -orbit normal,
  z = nadir), `frames::lvlh_rate_inertial_radps(state)`, `orbit::propagate_two_body(state, gm, dt)`.
- Produces: `orbit::RelativeState{position_m, velocity_mps}`; `orbit::cw_propagate(state, n_radps, t_s)`;
  `orbit::cw_transfer_velocity(from_m, to_m, n_radps, t_s) -> std::optional<Vec3>`;
  `orbit::lvlh_relative_state(target, chaser) -> RelativeState`; `orbit::chaser_from_lvlh(target, relative) -> StateVector`.

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_clohessy_wiltshire.cpp`:

```cpp
// ISS i3: Clohessy-Wiltshire relative motion in the target's LVLH frame (x V-bar, y -orbit normal, z nadir).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "starsim/orbit/kepler.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double gm = constants::earth_gm_m3ps2;
const double radius_m = constants::wgs84_semi_major_axis_m + 415e3;
const double n = std::sqrt(gm / (radius_m * radius_m * radius_m));

math::StateVector circular_target() { return {{radius_m, 0.0, 0.0}, {0.0, std::sqrt(gm / radius_m), 0.0}}; }
}  // namespace

TEST_CASE("CW: a coelliptic chaser below drifts forward at 1.5 n dh", "[rendezvous][cw]") {
    const orbit::RelativeState below{{0.0, 0.0, 2900.0}, {1.5 * n * 2900.0, 0.0, 0.0}};
    const auto later = orbit::cw_propagate(below, n, 3600.0);
    CHECK(later.position_m.z == Catch::Approx(2900.0).margin(1e-6));
    CHECK(later.position_m.x == Catch::Approx(1.5 * n * 2900.0 * 3600.0).epsilon(1e-12));
    CHECK(later.velocity_mps.z == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("CW: V-bar points are equilibria, out-of-plane motion oscillates", "[rendezvous][cw]") {
    const auto held = orbit::cw_propagate({{-400.0, 0.0, 0.0}, {}}, n, 5000.0);
    CHECK(norm(held.position_m - Vec3{-400.0, 0.0, 0.0}) < 1e-9);
    const double period = 2.0 * constants::pi / n;
    const auto lateral = orbit::cw_propagate({{0.0, 100.0, 0.0}, {}}, n, 0.5 * period);
    CHECK(lateral.position_m.y == Catch::Approx(-100.0).margin(1e-6));
}

TEST_CASE("CW: the transfer velocity reaches the aim point", "[rendezvous][cw]") {
    const Vec3 from{-6900.0, 30.0, 2900.0}, to{0.0, 0.0, 400.0};
    for (const double t : {600.0, 1500.0, 2700.0, 4000.0}) {
        CAPTURE(t);
        const auto v = orbit::cw_transfer_velocity(from, to, n, t);
        REQUIRE(v);
        const auto arrival = orbit::cw_propagate({from, *v}, n, t);
        CHECK(norm(arrival.position_m - to) < 1e-6);
    }
    const double period = 2.0 * constants::pi / n;
    CHECK_FALSE(orbit::cw_transfer_velocity(from, to, n, period));  // in-plane singular after a full revolution
}

TEST_CASE("CW agrees with two-body motion near the target", "[rendezvous][cw]") {
    // A chaser 2.9 km below and 6.9 km behind; both coasted two-body for 45 min against the CW prediction.
    const auto target = circular_target();
    const orbit::RelativeState start{{-6900.0, 50.0, 2900.0}, {1.5 * n * 2900.0, 0.02, 0.1}};
    const auto chaser = orbit::chaser_from_lvlh(target, start);
    const auto back = orbit::lvlh_relative_state(target, chaser);
    CHECK(norm(back.position_m - start.position_m) < 1e-6);
    CHECK(norm(back.velocity_mps - start.velocity_mps) < 1e-9);
    const double t = 2700.0;
    const auto truth = orbit::lvlh_relative_state(orbit::propagate_two_body(target, gm, t),
                                                  orbit::propagate_two_body(chaser, gm, t));
    const auto cw = orbit::cw_propagate(start, n, t);
    // [E] the CW linearisation error grows with (range / r)^2 x range: ~10 m at 7 km after half a revolution
    CHECK(norm(truth.position_m - cw.position_m) < 40.0);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[cw]" > tmp/i3_t1.txt 2>&1`
Expected: build error (`starsim/orbit/clohessy_wiltshire.hpp` not found).

- [ ] **Step 3: Write the header**

Create `core/include/starsim/orbit/clohessy_wiltshire.hpp`:

```cpp
#pragma once

#include <optional>

#include "starsim/math/state_vector.hpp"

namespace starsim::orbit {

// Relative motion of a chaser near a target on a circular orbit (Clohessy-Wiltshire / Hill) in the target's LVLH frame
// of frames/lvlh.hpp: x along the velocity (V-bar), y = -orbit normal (H-bar), z = nadir (R-bar, towards the Earth).
// Linear and two-body (no J2, no drag): first guesses and timing only; the burns are targeted in the full model.
struct RelativeState {
    math::Vec3 position_m;
    math::Vec3 velocity_mps;  // relative to the rotating LVLH frame
};

// Closed-form CW state after `t_s`; `n_radps`: mean motion of the target orbit.
RelativeState cw_propagate(const RelativeState& state, double n_radps, double t_s);

// Initial relative velocity that carries the chaser from `from_m` to `to_m` in `t_s` (two-impulse transfer). Empty when
// the in-plane problem is singular (t at a multiple of the period) or the out-of-plane one (t at a multiple of half
// the period with an out-of-plane change).
std::optional<math::Vec3> cw_transfer_velocity(const math::Vec3& from_m, const math::Vec3& to_m, double n_radps,
                                               double t_s);

// The chaser relative to the target in the target's LVLH frame (velocity relative to the rotating frame), and back.
RelativeState lvlh_relative_state(const math::StateVector& target, const math::StateVector& chaser);
math::StateVector chaser_from_lvlh(const math::StateVector& target, const RelativeState& relative);

}  // namespace starsim::orbit
```

- [ ] **Step 4: Write the implementation**

Create `core/src/orbit/clohessy_wiltshire.cpp`:

```cpp
#include "starsim/orbit/clohessy_wiltshire.hpp"

#include <cmath>

#include "starsim/frames/lvlh.hpp"

namespace starsim::orbit {

using math::Vec3;

namespace {
constexpr double singular_tolerance = 1e-9;  // of the dimensionless determinants below
}

// CW in this LVLH convention: x'' = 2 n z', z'' = -2 n x' + 3 n^2 z, y'' = -n^2 y (Fehse, Automated Rendezvous and
// Docking of Spacecraft, eq. 3.25 with x = V-bar, z = R-bar).
RelativeState cw_propagate(const RelativeState& state, double n, double t) {
    const double tau = n * t, s = std::sin(tau), c = std::cos(tau);
    const Vec3& r = state.position_m;
    const Vec3& v = state.velocity_mps;
    RelativeState out;
    out.position_m.x = r.x + 6.0 * (tau - s) * r.z + (4.0 * s - 3.0 * tau) / n * v.x + 2.0 * (1.0 - c) / n * v.z;
    out.position_m.y = r.y * c + v.y / n * s;
    out.position_m.z = (4.0 - 3.0 * c) * r.z + 2.0 * (c - 1.0) / n * v.x + s / n * v.z;
    out.velocity_mps.x = 6.0 * n * (1.0 - c) * r.z + (4.0 * c - 3.0) * v.x + 2.0 * s * v.z;
    out.velocity_mps.y = -r.y * n * s + v.y * c;
    out.velocity_mps.z = 3.0 * n * s * r.z - 2.0 * s * v.x + c * v.z;
    return out;
}

std::optional<Vec3> cw_transfer_velocity(const Vec3& from, const Vec3& to, double n, double t) {
    const double tau = n * t, s = std::sin(tau), c = std::cos(tau);
    // In plane: [x; z](t) - (terms of the initial position) = (1/n) [[4s - 3 tau, 2(1 - c)], [2(c - 1), s]] [vx; vz].
    const double a11 = (4.0 * s - 3.0 * tau), a12 = 2.0 * (1.0 - c), a21 = 2.0 * (c - 1.0), a22 = s;
    const double det = a11 * a22 - a12 * a21;  // 8 - 8 cos tau - 3 tau sin tau
    if (std::abs(det) < singular_tolerance) return std::nullopt;
    const double bx = n * (to.x - (from.x + 6.0 * (tau - s) * from.z));
    const double bz = n * (to.z - (4.0 - 3.0 * c) * from.z);
    Vec3 v;
    v.x = (a22 * bx - a12 * bz) / det;
    v.z = (a11 * bz - a21 * bx) / det;
    const double dy = to.y - from.y * c;
    if (std::abs(s) < singular_tolerance) {
        if (std::abs(dy) > 1e-6) return std::nullopt;
        v.y = 0.0;
    } else {
        v.y = n * dy / s;
    }
    return v;
}

RelativeState lvlh_relative_state(const math::StateVector& target, const math::StateVector& chaser) {
    const auto to_inertial = frames::lvlh_to_inertial(target);
    const Vec3 omega = frames::lvlh_rate_inertial_radps(target);
    const Vec3 dr = chaser.position_m - target.position_m;
    const Vec3 dv = chaser.velocity_mps - target.velocity_mps - cross(omega, dr);
    return {to_inertial.inverse_rotate(dr), to_inertial.inverse_rotate(dv)};
}

math::StateVector chaser_from_lvlh(const math::StateVector& target, const RelativeState& relative) {
    const auto to_inertial = frames::lvlh_to_inertial(target);
    const Vec3 omega = frames::lvlh_rate_inertial_radps(target);
    const Vec3 dr = to_inertial.rotate(relative.position_m);
    return {target.position_m + dr, target.velocity_mps + to_inertial.rotate(relative.velocity_mps) + cross(omega, dr)};
}

}  // namespace starsim::orbit
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[cw]" > tmp/i3_t1.txt 2>&1`
Expected: 4 test cases pass. If "CW agrees with two-body motion" exceeds 40 m, print the error and check the sign
convention of `frames::lvlh_to_inertial` (z must point to the Earth) before touching the tolerance.

- [ ] **Step 6: Commit**

```bash
git add core/include/starsim/orbit/clohessy_wiltshire.hpp core/src/orbit/clohessy_wiltshire.cpp core/tests/test_clohessy_wiltshire.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat(orbit): Clohessy-Wiltshire relative motion in the target LVLH frame"
```

Run `-Fast` and push (`git push origin main`).

---

### Task 2: Drag in the position targeter and the rendezvous geometry

**Files:**
- Modify: `core/include/starsim/mission/position_targeter.hpp`, `core/src/mission/position_targeter.cpp:27-30`
- Create: `core/include/starsim/mission/rendezvous_geometry.hpp`, `core/src/mission/rendezvous_geometry.cpp`
- Create: `core/tests/test_rendezvous_geometry.cpp`
- Modify: `core/CMakeLists.txt` (add `src/mission/rendezvous_geometry.cpp` after `src/mission/position_targeter.cpp`),
  `core/tests/CMakeLists.txt` (append `test_rendezvous_geometry.cpp`)

**Interfaces:**
- Consumes: `gnc::coast_with_drag(gravity, state, epoch, end, drag_area_per_mass_m2pkg)` (gnc/deorbit_planner.hpp;
  US76, air co-rotating about the GCRF z axis), `orbit::lvlh_relative_state` (Task 1).
- Produces:
  - `PositionTargeter::set_drag_area_per_mass(double m2pkg)` (0: the old drag-free coast, unchanged results);
    `coast()` and `solve()` use it.
  - `mission::approach_ellipsoid_semi_axes_m` = `{2000.0, 1000.0, 1000.0}` (LVLH x, y, z), `mission::keep_out_sphere_radius_m` = 200.0.
  - `mission::point_behind_below(const math::StateVector& target, double behind_m, double below_m) -> math::Vec3`
    (curvilinear: rotated about the target's orbit normal by -behind/r, radius r - below).
  - `mission::approach_ellipsoid_metric(const math::StateVector& target, const math::Vec3& chaser_position_m) -> double`
    (< 1 inside).
  - `mission::in_plane_angle_rad(const math::StateVector& target, const math::Vec3& position_m) -> double`: angle of the
    position's projection onto the target orbit plane, measured from the target position about the target orbit
    normal, in (-pi, pi] (> 0: ahead of the target).

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_rendezvous_geometry.cpp`:

```cpp
// ISS i3: target-relative aim points, approach ellipsoid and the drag term of the position targeter.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/position_targeter.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double gm = constants::earth_gm_m3ps2;
const double radius_m = constants::wgs84_semi_major_axis_m + 415e3;
// A circular target in an inclined plane (51.6 deg about x).
math::StateVector target_state() {
    const double v = std::sqrt(gm / radius_m), i = 51.6 * constants::deg_to_rad;
    return {{radius_m, 0.0, 0.0}, {0.0, v * std::cos(i), v * std::sin(i)}};
}
}  // namespace

TEST_CASE("Aim point behind and below the target", "[rendezvous]") {
    const auto target = target_state();
    const Vec3 p = mission::point_behind_below(target, 6900.0, 2900.0);
    CHECK(norm(p) == Catch::Approx(radius_m - 2900.0).margin(1e-6));
    const auto rel = orbit::lvlh_relative_state(target, {p, target.velocity_mps});
    CHECK(rel.position_m.x == Catch::Approx(-6900.0).margin(5.0));  // curvilinear: arc length 6.9 km behind
    CHECK(rel.position_m.z == Catch::Approx(2900.0).margin(5.0));   // nadir is +z
    CHECK(std::abs(rel.position_m.y) < 1e-6);
    CHECK(mission::in_plane_angle_rad(target, p) == Catch::Approx(-6900.0 / radius_m).epsilon(1e-9));
    const Vec3 wp0 = mission::point_behind_below(target, 0.0, 400.0);
    CHECK(norm(wp0 - target.position_m * ((radius_m - 400.0) / radius_m)) < 1e-6);
}

TEST_CASE("Approach ellipsoid metric", "[rendezvous]") {
    const auto target = target_state();
    const auto at = [&](Vec3 lvlh) { return orbit::chaser_from_lvlh(target, {lvlh, {}}).position_m; };
    CHECK(mission::approach_ellipsoid_metric(target, at({2000.0, 0.0, 0.0})) == Catch::Approx(1.0).epsilon(1e-6));
    CHECK(mission::approach_ellipsoid_metric(target, at({0.0, 0.0, 1000.0})) == Catch::Approx(1.0).epsilon(1e-6));
    CHECK(mission::approach_ellipsoid_metric(target, at({0.0, 0.0, 400.0})) < 1.0);  // Waypoint 0 lies inside
    CHECK(mission::approach_ellipsoid_metric(target, at({-6900.0, 0.0, 2900.0})) > 1.0);
}

TEST_CASE("Position targeter with orbital drag", "[rendezvous][phasing]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const environment::PointMassGravity gravity{ephemeris, earth};
    const time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 10, 16, 0, 0, 0.0});
    const double low = constants::wgs84_semi_major_axis_m + 205e3;
    const double i = 51.6 * constants::deg_to_rad, v = std::sqrt(gm / low);
    const math::StateVector chaser{{low, 0.0, 0.0}, {0.0, v * std::cos(i), v * std::sin(i)}};
    constexpr double beta = 2.2e-3;  // [E] Dragon-like C_D A / m
    mission::PositionTargeter targeter{gravity};
    // Without drag nothing changes: the coast equals the drag-free propagator.
    CHECK(norm(targeter.coast(chaser, t0, t0 + 3600.0).position_m -
               gnc::coast_with_drag(gravity, chaser, t0, t0 + 3600.0, 0.0).position_m) < 0.05);
    targeter.set_drag_area_per_mass(beta);
    const auto dragged = targeter.coast(chaser, t0, t0 + 6.0 * 3600.0);
    CHECK(norm(dragged.position_m - gnc::coast_with_drag(gravity, chaser, t0, t0 + 6.0 * 3600.0, beta).position_m) < 0.05);
    // A 3 h aim point taken from a drag-free coast with a 1 m/s prograde burn: the drag-aware solution hits it.
    const auto at_burn = targeter.coast(chaser, t0, t0 + 600.0);
    math::StateVector free = gnc::coast_with_drag(gravity, chaser, t0, t0 + 600.0, 0.0);
    free.velocity_mps += normalized(free.velocity_mps);
    const mission::PositionTarget aim{gnc::coast_with_drag(gravity, free, t0 + 600.0, t0 + 3.0 * 3600.0, 0.0).position_m,
                                      t0 + 3.0 * 3600.0};
    const auto solution = targeter.solve(chaser, t0, t0 + 600.0, aim, normalized(at_burn.velocity_mps), 1.0);
    CHECK(solution.converged);
    CHECK(solution.miss_m < 1.0);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[rendezvous]" > tmp/i3_t2.txt 2>&1`
Expected: build error (`rendezvous_geometry.hpp` not found, no `set_drag_area_per_mass`).

- [ ] **Step 3: Drag in the position targeter**

In `core/include/starsim/mission/position_targeter.hpp`, after `set_finite_burn`:

```cpp
    // Orbital drag of the coasts, C_D A / m (US76, gnc::coast_with_drag); 0 (default): drag-free. The finite burn
    // itself stays drag-free (minutes).
    void set_drag_area_per_mass(double m2pkg) { drag_area_per_mass_m2pkg_ = m2pkg; }
```

and the member `double drag_area_per_mass_m2pkg_ = 0.0;` after `finite_`.

In `core/src/mission/position_targeter.cpp` add `#include "starsim/gnc/deorbit_planner.hpp"` and replace `coast`:

```cpp
StateVector PositionTargeter::coast(const StateVector& state, time::Epoch from, time::Epoch to) const {
    if (to - from == 0.0) return state;
    if (drag_area_per_mass_m2pkg_ > 0.0) {
        return gnc::coast_with_drag(gravity_, state, from, to, drag_area_per_mass_m2pkg_);
    }
    return orbit::propagate_coast(gravity_, state, from, to).state;
}
```

- [ ] **Step 4: The geometry header and implementation**

Create `core/include/starsim/mission/rendezvous_geometry.hpp`:

```cpp
#pragma once

#include "starsim/math/state_vector.hpp"

namespace starsim::mission {

// ISS approach ellipsoid, semi-axes in the station's LVLH frame (x V-bar, y H-bar, z R-bar) [S: NASA, "approach
// ellipsoid 4 x 2 x 2 km" centred on the station's centre of mass], and the keep-out sphere [S: NASA, 200 m].
inline constexpr double approach_ellipsoid_semi_axes_m[3] = {2000.0, 1000.0, 1000.0};
inline constexpr double keep_out_sphere_radius_m = 200.0;

// A point `behind_m` along track behind the target (arc length on the target's radius: the target position turned
// about its orbit normal by -behind_m / r) and `below_m` below the target's radius.
math::Vec3 point_behind_below(const math::StateVector& target, double behind_m, double below_m);

// (x/a)^2 + (y/b)^2 + (z/c)^2 of the chaser in the target's LVLH frame; < 1: inside the approach ellipsoid.
double approach_ellipsoid_metric(const math::StateVector& target, const math::Vec3& chaser_position_m);

// Angle of the position's projection onto the target orbit plane, from the target position about the orbit normal,
// in (-pi, pi]; > 0 ahead of the target.
double in_plane_angle_rad(const math::StateVector& target, const math::Vec3& position_m);

}  // namespace starsim::mission
```

Create `core/src/mission/rendezvous_geometry.cpp`:

```cpp
#include "starsim/mission/rendezvous_geometry.hpp"

#include <cmath>

#include "starsim/orbit/clohessy_wiltshire.hpp"

namespace starsim::mission {

using math::Vec3;

Vec3 point_behind_below(const math::StateVector& target, double behind_m, double below_m) {
    const double r = norm(target.position_m);
    const Vec3 radial = target.position_m / r;
    const Vec3 normal = normalized(cross(target.position_m, target.velocity_mps));
    const Vec3 along = cross(normal, radial);
    const double angle = -behind_m / r;
    return (r - below_m) * (std::cos(angle) * radial + std::sin(angle) * along);
}

double approach_ellipsoid_metric(const math::StateVector& target, const Vec3& chaser_position_m) {
    const Vec3 p = orbit::lvlh_relative_state(target, {chaser_position_m, target.velocity_mps}).position_m;
    const auto& a = approach_ellipsoid_semi_axes_m;
    return (p.x / a[0]) * (p.x / a[0]) + (p.y / a[1]) * (p.y / a[1]) + (p.z / a[2]) * (p.z / a[2]);
}

double in_plane_angle_rad(const math::StateVector& target, const Vec3& position_m) {
    const Vec3 radial = normalized(target.position_m);
    const Vec3 normal = normalized(cross(target.position_m, target.velocity_mps));
    const Vec3 along = cross(normal, radial);
    return std::atan2(dot(position_m, along), dot(position_m, radial));
}

}  // namespace starsim::mission
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[rendezvous]" > tmp/i3_t2.txt 2>&1`
Expected: the 3 new test cases and Task 1's pass. Then run `-TestFilter "[transfer]"` and `-TestFilter "[lunar]"`
(other users of `PositionTargeter`): unchanged results (the default path is untouched).

- [ ] **Step 6: Commit**

```bash
git add core/include/starsim/mission/position_targeter.hpp core/src/mission/position_targeter.cpp core/include/starsim/mission/rendezvous_geometry.hpp core/src/mission/rendezvous_geometry.cpp core/tests/test_rendezvous_geometry.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat(mission): rendezvous geometry and orbital drag in the position targeter"
```

Run `-Fast` and push.

---

### Task 3: Analytic phasing schedule

**Files:**
- Create: `core/include/starsim/mission/phasing_schedule.hpp`, `core/src/mission/phasing_schedule.cpp`
- Create: `core/tests/test_phasing_schedule.cpp`
- Modify: `core/CMakeLists.txt` (add `src/mission/phasing_schedule.cpp` after `src/mission/rendezvous_geometry.cpp`),
  `core/tests/CMakeLists.txt` (append `test_phasing_schedule.cpp`)

**Interfaces:**
- Consumes: `orbit::elements_from_state`, `orbit::j2_secular_rates(StationMeanElements, gm, j2, radius)` and
  `orbit::StationMeanElements` (orbit/synthetic_station_orbit.hpp), `mission::in_plane_angle_rad` (Task 2),
  `constants::earth_j2`, `environment::earth_j2_reference_radius_m`.
- Produces:
  - `mission::PhasingParameters` (fields below; Task 4 and 5 use them).
  - `mission::PhasingSchedule{bool feasible; std::string reason; int phasing_revolutions; int extra_laps; double
    phase_orbit_apoapsis_radius_m; time::Epoch phase_epoch, boost_epoch, coelliptic_epoch,
    approach_initiation_epoch, waypoint0_epoch;}`
  - `mission::phasing_schedule(chaser, target, epoch, parameters, gm) -> PhasingSchedule`
  - `mission::coelliptic_behind_m(const PhasingParameters&, double target_radius_m, double gm) -> double`: along-track
    distance behind the target at the coelliptic burn.

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_phasing_schedule.cpp`:

```cpp
// ISS i3: analytic phasing schedule (wait on the separation orbit, or slow down with a phase burn).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/math/constants.hpp"
#include "starsim/mission/phasing_schedule.hpp"
#include "starsim/orbit/kepler.hpp"

using namespace starsim;

namespace {
constexpr double gm = constants::earth_gm_m3ps2;
constexpr double deg = constants::deg_to_rad;
const double re = constants::wgs84_semi_major_axis_m;
const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);

math::StateVector circular(double height_m, double argument_of_latitude_rad) {
    orbit::KeplerianElements e;
    e.semi_major_axis_m = re + height_m;
    e.inclination_rad = 51.64 * deg;
    e.raan_rad = 1.0;
    e.true_anomaly_rad = argument_of_latitude_rad;
    return orbit::state_from_elements(e, gm);
}
}  // namespace

TEST_CASE("Schedule: the Dragon waits on the separation orbit and arrives in time", "[rendezvous][phasing]") {
    const auto target = circular(415e3, 1.0);
    for (const double lead_deg : {30.0, 120.0, 240.0}) {
        CAPTURE(lead_deg);
        const auto chaser = circular(205e3, 1.0 - lead_deg * deg);  // the ISS is lead_deg ahead
        const auto s = mission::phasing_schedule(chaser, target, t0, {}, gm);
        REQUIRE(s.feasible);
        CHECK(s.phasing_revolutions == 0);  // no phase burn: the earliest window
        CHECK(s.extra_laps == 0);
        CHECK(s.boost_epoch - t0 >= mission::PhasingParameters{}.first_burn_lead_s);
        CHECK(s.waypoint0_epoch - t0 <= mission::PhasingParameters{}.mission_limit_s);
        // ~11 deg/h catch-up at 205 km against 415 km: the wait grows with the lead angle
        CHECK((s.boost_epoch - t0) / 3600.0 == Catch::Approx(lead_deg / 11.2).margin(1.0));
        CHECK(s.coelliptic_epoch - s.boost_epoch > 2700.0);  // half a transfer orbit (~46 min)
        CHECK(s.approach_initiation_epoch - s.coelliptic_epoch == Catch::Approx(10080.0));
        CHECK(s.waypoint0_epoch - s.approach_initiation_epoch == Catch::Approx(2700.0));
    }
}

TEST_CASE("Schedule: an earliest arrival slows the catch-up with a phase burn", "[rendezvous][phasing]") {
    const auto target = circular(415e3, 1.0);
    const auto chaser = circular(205e3, 1.0 - 60.0 * deg);
    mission::PhasingParameters p;
    p.earliest_arrival_s = 20.0 * 3600.0;  // a crew day: not before 20 h
    const auto s = mission::phasing_schedule(chaser, target, t0, p, gm);
    REQUIRE(s.feasible);
    CHECK(s.phasing_revolutions >= 1);
    CHECK(s.phase_epoch - t0 == Catch::Approx(p.first_burn_lead_s));
    CHECK(s.phase_orbit_apoapsis_radius_m > re + 205e3);
    CHECK(s.phase_orbit_apoapsis_radius_m < re + 415e3 - p.coelliptic_height_m);
    CHECK(s.waypoint0_epoch - t0 >= p.earliest_arrival_s);
    CHECK(s.waypoint0_epoch - t0 <= p.mission_limit_s);
}

TEST_CASE("Schedule: a phase beyond the mission limit is refused, a longer limit takes an extra lap", "[rendezvous][phasing]") {
    const auto target = circular(415e3, 1.0);
    const auto chaser = circular(205e3, 1.0 - 330.0 * deg);
    const auto refused = mission::phasing_schedule(chaser, target, t0, {}, gm);
    CHECK_FALSE(refused.feasible);
    CHECK(refused.reason.rfind("Kein Rendezvous", 0) == 0);
    // The ISS just ahead (0.05 deg): the coelliptic geometry needs more lead - one more lap.
    mission::PhasingParameters two_days;
    two_days.mission_limit_s = 48.0 * 3600.0;
    const auto lap = mission::phasing_schedule(circular(205e3, 1.0 - 0.05 * deg), target, t0, two_days, gm);
    REQUIRE(lap.feasible);
    CHECK(lap.extra_laps == 1);
}

TEST_CASE("Schedule refuses a chaser above the coelliptic height", "[rendezvous][phasing]") {
    const auto target = circular(415e3, 1.0);
    const auto s = mission::phasing_schedule(circular(414e3, 0.5), target, t0, {}, gm);
    CHECK_FALSE(s.feasible);
    CHECK(s.reason.rfind("Kein Rendezvous", 0) == 0);
    orbit::KeplerianElements hyperbolic;
    hyperbolic.semi_major_axis_m = -2.0e7;
    hyperbolic.eccentricity = 1.5;
    CHECK_FALSE(mission::phasing_schedule(orbit::state_from_elements(hyperbolic, gm), target, t0, {}, gm).feasible);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[phasing]" > tmp/i3_t3.txt 2>&1`
Expected: build error (`phasing_schedule.hpp` not found).

- [ ] **Step 3: The header**

Create `core/include/starsim/mission/phasing_schedule.hpp`:

```cpp
#pragma once

#include <string>

#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

// Coelliptic rendezvous after the Crew Dragon pattern (spec i3; plan docs/superpowers/plans/2026-10-19-iss-i3-phasing.md
// lists the Crew-2 timeline the values are taken from).
struct PhasingParameters {
    double coelliptic_height_m = 2900.0;  // [D] Crew-2: 4.9 m/s closing between 30 and 10 km = 1.5 n dh
    double approach_initiation_range_m = 7500.0;  // [S] Crew-2: approach initiation burn at 7.5 km
    double approach_transfer_s = 2700.0;   // [S] Crew-2: approach initiation 03:40 -> Waypoint 0 04:25
    double mid_course_after_s = 1500.0;    // [S] Crew-2: approach initiation midcourse burn 04:05
    double waypoint0_below_m = 400.0;      // [S] Crew-2: Waypoint 0, 400 m below the ISS
    double coelliptic_coast_s = 10080.0;   // [S] Crew-2: coelliptic burn 00:53 -> approach initiation 03:40
    double first_burn_lead_s = 1800.0;     // [E] checkout after the separation (Crew-2 phase burn 37 min, Demo-2 49 min)
    double mission_limit_s = 28.0 * 3600.0;  // [S] crew missions dock 24-28 h after the launch
    double earliest_arrival_s = 0.0;       // Waypoint 0 not before (a crew timeline); 0: the earliest window
    double mid_course_threshold_mps = 0.05;  // [E] smaller corrections are skipped
    double reserve_kg = 900.0;  // [E] deorbit 598 kg (d3: 139 m/s from 420 km) + proximity operations and abort
    // C_D A / m of the planner's coasts (gnc::coast_with_drag); the session fills them from its tracking.
    double chaser_drag_area_per_mass_m2pkg = 0.0;
    double target_drag_area_per_mass_m2pkg = 0.0;
};

// Burn epochs of the coelliptic profile from mean motions with the secular J2 rates (two-body otherwise): the Dragon
// either waits on its orbit until the phase is right and boosts (phasing_revolutions = 0, the earliest window) or, when
// that would arrive before earliest_arrival_s, slows the catch-up with a prograde phase burn at first_burn_lead_s (an
// apoapsis raise) and boosts after `phasing_revolutions` revolutions at the burn point. A phase that needs more than
// one full catch-up lap adds `extra_laps`. Boost at the start (or phasing) orbit -> half a transfer ellipse -> coelliptic
// burn coelliptic_height_m below the target -> coelliptic coast -> approach initiation -> Waypoint 0.
struct PhasingSchedule {
    bool feasible = false;
    std::string reason;  // German when refused
    int phasing_revolutions = 0;
    int extra_laps = 0;
    double phase_orbit_apoapsis_radius_m = 0.0;  // the phase burn's target apoapsis (0 without one)
    time::Epoch phase_epoch, boost_epoch, coelliptic_epoch, approach_initiation_epoch, waypoint0_epoch;
};

PhasingSchedule phasing_schedule(const math::StateVector& chaser, const math::StateVector& target, time::Epoch epoch,
                                 const PhasingParameters& parameters, double gm_m3ps2);

// Along-track distance behind the target at the coelliptic burn: the approach initiation point's horizontal distance
// plus the coelliptic closing (1.5 n dh) over the coelliptic coast.
double coelliptic_behind_m(const PhasingParameters& parameters, double target_radius_m, double gm_m3ps2);

}  // namespace starsim::mission
```

- [ ] **Step 4: The implementation**

Create `core/src/mission/phasing_schedule.cpp`:

```cpp
#include "starsim/mission/phasing_schedule.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/environment/gravity.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"

namespace starsim::mission {

namespace {
constexpr int max_extra_laps = 3;
constexpr int bisection_steps = 60;

// Mean argument-of-latitude rate of an orbit with semi-major axis a (Kepler + secular J2).
double latitude_rate(double a, double eccentricity, double inclination, double gm) {
    orbit::StationMeanElements mean;
    mean.semi_major_axis_m = a;
    mean.eccentricity = eccentricity;
    mean.inclination_rad = inclination;
    return orbit::j2_secular_rates(mean, gm, constants::earth_j2, environment::earth_j2_reference_radius_m)
        .latitude_radps;
}

double half_transfer_s(double r1, double r2, double gm) {
    const double a = 0.5 * (r1 + r2);
    return constants::pi * std::sqrt(a * a * a / gm);
}
}  // namespace

double coelliptic_behind_m(const PhasingParameters& p, double target_radius_m, double gm) {
    const double n = std::sqrt(gm / (target_radius_m * target_radius_m * target_radius_m));
    const double range = p.approach_initiation_range_m, dh = p.coelliptic_height_m;
    return std::sqrt(std::max(0.0, range * range - dh * dh)) + 1.5 * n * dh * p.coelliptic_coast_s;
}

PhasingSchedule phasing_schedule(const math::StateVector& chaser, const math::StateVector& target, time::Epoch epoch,
                                 const PhasingParameters& p, double gm) {
    PhasingSchedule s;
    const auto ec = orbit::elements_from_state(chaser, gm);
    const auto et = orbit::elements_from_state(target, gm);
    if (!(ec.semi_major_axis_m > 0.0) || ec.eccentricity >= 1.0 || !(et.semi_major_axis_m > 0.0)) {
        s.reason = "Kein Rendezvous: keine geschlossene Bahn";
        return s;
    }
    const double r_d = ec.semi_major_axis_m, r_i = et.semi_major_axis_m;
    const double r_c = r_i - p.coelliptic_height_m;
    if (r_d >= r_c) {
        s.reason = "Kein Rendezvous: Bahn nicht unter der koelliptischen Höhe";
        return s;
    }
    const double n_d = latitude_rate(r_d, ec.eccentricity, ec.inclination_rad, gm);
    const double n_i = latitude_rate(r_i, et.eccentricity, et.inclination_rad, gm);
    const double t_h = half_transfer_s(r_d, r_c, gm);
    // Phase (target ahead of the chaser) the boost needs: the chaser half a transfer behind the coelliptic aim point.
    const double behind_c = coelliptic_behind_m(p, r_i, gm);
    const double required_at_boost = constants::pi + behind_c / r_i - n_i * t_h;
    double lead = -in_plane_angle_rad(target, chaser.position_m);  // > 0: target ahead
    if (lead < 0.0) lead += constants::two_pi;
    const double tail_s = t_h + p.coelliptic_coast_s + p.approach_transfer_s;
    const double closing = n_d - n_i;  // rad/s of catch-up on the start orbit (the coelliptic closing is in behind_c)

    for (int m = 0; m <= max_extra_laps; ++m) {
        const double catch_up = lead + constants::two_pi * m - required_at_boost;
        if (catch_up < 0.0) continue;
        // 1. Wait on the start orbit: the earliest window of this lap.
        const double wait_s = catch_up / closing;
        if (wait_s >= p.first_burn_lead_s && wait_s + tail_s <= p.mission_limit_s &&
            wait_s + tail_s >= p.earliest_arrival_s) {
            s.feasible = true;
            s.extra_laps = m;
            s.boost_epoch = epoch + wait_s;
            break;
        }
        // 2. Phase burn at first_burn_lead_s raising the apoapsis to r_a, boost after k revolutions at its point:
        //    catch-up = closing * lead_s + k (2 pi - n_i P(a)) with P = 2 pi / n(a), a = (r_d + r_a) / 2.
        bool found = false;
        for (int k = 1; !found; ++k) {
            const double base = closing * p.first_burn_lead_s;
            const auto caught = [&](double r_a) {
                const double a = 0.5 * (r_d + r_a);
                const double period = constants::two_pi / latitude_rate(a, (r_a - r_d) / (r_a + r_d), ec.inclination_rad, gm);
                return base + k * (constants::two_pi - n_i * period);
            };
            const double period_low = constants::two_pi / n_d;
            if (p.first_burn_lead_s + k * period_low + tail_s > p.mission_limit_s) break;
            if (caught(r_d) < catch_up || caught(r_c) > catch_up) continue;  // this k cannot close this lap
            double lo = r_d, hi = r_c;  // caught() decreases with r_a
            for (int b = 0; b < bisection_steps; ++b) {
                const double mid = 0.5 * (lo + hi);
                (caught(mid) > catch_up ? lo : hi) = mid;
            }
            const double r_a = 0.5 * (lo + hi);
            const double a = 0.5 * (r_d + r_a);
            const double period = constants::two_pi / latitude_rate(a, (r_a - r_d) / (r_a + r_d), ec.inclination_rad, gm);
            const double boost_s = p.first_burn_lead_s + k * period;
            if (boost_s + tail_s > p.mission_limit_s || boost_s + tail_s < p.earliest_arrival_s) continue;
            s.feasible = true;
            s.extra_laps = m;
            s.phasing_revolutions = k;
            s.phase_orbit_apoapsis_radius_m = r_a;
            s.phase_epoch = epoch + p.first_burn_lead_s;
            s.boost_epoch = epoch + boost_s;
            found = true;
        }
        if (found) break;
    }
    if (!s.feasible) {
        s.reason = "Kein Rendezvous: Phasenwinkel nicht innerhalb von " +
                   std::to_string(static_cast<int>(std::lround(p.mission_limit_s / 3600.0))) + " h erreichbar";
        return s;
    }
    s.coelliptic_epoch = s.boost_epoch + t_h;
    s.approach_initiation_epoch = s.coelliptic_epoch + p.coelliptic_coast_s;
    s.waypoint0_epoch = s.approach_initiation_epoch + p.approach_transfer_s;
    return s;
}

}  // namespace starsim::mission
```

Notes for the implementer: the earliest-arrival case must prefer a phase burn in the *same* lap before an extra lap
(the loop does: option 1 fails on `earliest_arrival_s`, option 2 searches k).

- [ ] **Step 5: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[phasing]" > tmp/i3_t3.txt 2>&1`
Expected: Task 2's targeter case and the 4 schedule cases pass. If the wait in the first case is off by more than an
hour, print `closing` (expected ~5.5e-5 rad/s = 11.4 deg/h) and `required_at_boost` (expected ~4.6 deg: pi + 0.47 deg
of coelliptic lead - the target's ~175.9 deg during the 2,720 s transfer).

- [ ] **Step 6: Commit**

```bash
git add core/include/starsim/mission/phasing_schedule.hpp core/src/mission/phasing_schedule.cpp core/tests/test_phasing_schedule.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat(mission): analytic coelliptic phasing schedule"
```

Run `-Fast` and push.

---

### Task 4: Phasing planner in the full force model

**Files:**
- Create: `core/include/starsim/mission/phasing_planner.hpp`, `core/src/mission/phasing_planner.cpp`
- Create: `core/tests/rendezvous_test_support.hpp` (shared start states for Tasks 4 and 5)
- Create: `core/tests/test_phasing_planner.cpp`
- Modify: `core/CMakeLists.txt` (add `src/mission/phasing_planner.cpp` after `src/mission/phasing_schedule.cpp`),
  `core/tests/CMakeLists.txt` (append `test_phasing_planner.cpp`)

**Interfaces:**
- Consumes: Tasks 1-3; `PositionTargeter` (`set_drag_area_per_mass`, `set_finite_burn`, `solve`, `coast`),
  `mission::apply_finite_burn`, `mission::FiniteBurnModel{mass_kg, thrust_n, mass_flow_kgps}`,
  `gnc::coast_with_drag`, `frames::lvlh_rate_inertial_radps`.
- Produces:
  - `enum class mission::PhasingBurnKind { Phase, Boost, Coelliptic, ApproachInitiation, MidCourse, Braking };`
    `const char* mission::german_name(PhasingBurnKind)` ("Phasenmanöver", "Anhebung", "Koelliptisches Manöver",
    "Annäherungsbeginn", "Kurskorrektur", "Bremsmanöver").
  - `struct mission::PhasingBurn { PhasingBurnKind kind; gnc::BurnNode node; };`
  - `struct mission::PhasingPlan { bool feasible; std::string reason; PhasingSchedule schedule; std::vector<PhasingBurn>
    burns; double total_delta_v_mps; double propellant_kg; double min_ellipsoid_metric_before_ai; PhasingParameters
    parameters; };` (burns in order, without the mid-course correction, which is planned in flight).
  - `class mission::PhasingPlanner(const environment::GravityModel& gravity)` with
    `PhasingPlan plan(chaser, target, epoch, parameters, engines, propellant_kg) const`,
    `std::optional<gnc::BurnNode> retarget(PhasingBurnKind kind, chaser, target, epoch, const PhasingPlan&, engines) const`
    (empty: not converged, or a mid-course correction below the threshold),
    `double boost_phase_error_m(chaser, target, epoch, const PhasingPlan&) const` (along-track miss of the boost
    geometry, > 0 when the chaser is late).
  - Test support `rendezvous_test::Start` (ISS from the OEM fixture at 2026-10-16T00:00 UTC, a Dragon separation orbit
    200 x 210 km at a given phase behind the ISS with the plane bias) used by Task 5.

- [ ] **Step 1: Write the test support and the failing tests**

Create `core/tests/rendezvous_test_support.hpp`:

```cpp
#pragma once
// ISS i3 tests: start states of a coelliptic rendezvous - the ISS from the OEM fixture and a Crew Dragon in its
// separation orbit `lead_deg` behind the ISS, in the plane the ISS plane will have at `plane_match_s` (a real launch
// aims there: the J2 node regression is ~0.58 deg/day faster at 205 km than at 415 km).

#include <cmath>
#include <filesystem>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/oem.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/sim/station_start.hpp"
#include "starsim/spice/spice_adapters.hpp"

namespace starsim::rendezvous_test {

inline const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
inline const std::filesystem::path fixture_dir = std::filesystem::path(STARSIM_FIXTURE_DIR) / "iss_oem";

struct Start {
    sim::StationStart iss;
    math::StateVector dragon;
    time::Epoch epoch;
};

// Secular RAAN rate (J2) of a near-circular orbit.
inline double raan_rate(double a, double inclination) {
    orbit::StationMeanElements mean;
    mean.semi_major_axis_m = a;
    mean.inclination_rad = inclination;
    return orbit::j2_secular_rates(mean, constants::earth_gm_m3ps2, constants::earth_j2,
                                   environment::earth_j2_reference_radius_m).raan_radps;
}

inline Start start(const spice::SpiceTimeConverter& clock, const frames::BodyOrientation& earth, double lead_deg,
                   double plane_match_s) {
    Start s;
    s.epoch = clock.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto archive = orbit::OemArchive::scan(fixture_dir, clock);
    s.iss = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, clock, earth, s.epoch,
                           sim::IssOrbitSource::Oem);
    constexpr double gm = constants::earth_gm_m3ps2;
    const auto e_iss = orbit::elements_from_state(s.iss.state_gcrf, gm);
    orbit::KeplerianElements e;
    const double perigee = constants::wgs84_semi_major_axis_m + 200e3, apogee = constants::wgs84_semi_major_axis_m + 210e3;
    e.semi_major_axis_m = 0.5 * (perigee + apogee);  // [S] 200 x 210 km separation orbit (falcon9_crew_dragon)
    e.eccentricity = (apogee - perigee) / (apogee + perigee);
    e.inclination_rad = e_iss.inclination_rad;
    e.raan_rad = e_iss.raan_rad;
    e.argument_of_periapsis_rad = e_iss.argument_of_periapsis_rad + e_iss.true_anomaly_rad - lead_deg * constants::deg_to_rad;
    e.true_anomaly_rad = 0.0;  // at perigee
    s.dragon = orbit::state_from_elements(e, gm);
    // Plane bias: the Dragon's node regresses faster; shifted ahead so that both planes coincide after plane_match_s.
    const double bias = (raan_rate(e_iss.semi_major_axis_m, e_iss.inclination_rad) -
                         raan_rate(e.semi_major_axis_m, e.inclination_rad)) * plane_match_s;
    const auto turn = math::Quat::from_axis_angle({0.0, 0.0, 1.0}, bias);
    s.dragon = {turn.rotate(s.dragon.position_m), turn.rotate(s.dragon.velocity_mps)};
    return s;
}

}  // namespace starsim::rendezvous_test
```

(Rotating about the GCRF z axis instead of the true pole is a ~0.4 deg tilt of a ~0.3 deg bias: negligible.)

Create `core/tests/test_phasing_planner.cpp`:

```cpp
// ISS i3: the phasing planner in the full force model with drag - burn sizes, arrival at Waypoint 0, approach
// ellipsoid kept clear before the approach initiation, budget, refusals; campaign over the phase angle.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

#include "rendezvous_test_support.hpp"
#include "starsim/ephemeris/interpolated_ephemeris.hpp"
#include "starsim/frames/interpolated_orientation.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/mission/phasing_planner.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double gm = constants::earth_gm_m3ps2;
// [D] ISS: C_D A / m from the OEM header (1.5 x 1177.76 m^2 / 460,320 kg); Dragon: [E] C_D 2.2 x pi (2 m)^2 / 12.5 t
constexpr double iss_beta = 1.5 * 1177.76 / 460320.0;
constexpr double dragon_beta = 2.2 * 3.14159265358979 * 4.0 / 12500.0;
// Crew Dragon on its four side Dracos along +x (i2: 1556 N, 4.25 kg per m/s at 12.5 t -> mass flow 0.53 kg/s)
const mission::FiniteBurnModel dracos{12500.0, 1556.0, 1556.0 / (300.0 * constants::standard_gravity_mps2)};
constexpr double dragon_propellant_kg = 2562.0;

struct Fixture {
    spice::SpiceEphemeris spice_ephemeris;
    spice::SpiceOrientation spice_earth = spice::SpiceOrientation::earth();
    spice::SpiceTimeConverter clock;
    ephemeris::InterpolatedEphemeris ephemeris{spice_ephemeris};
    frames::InterpolatedOrientation earth{spice_earth};
    environment::PointMassGravity gravity{ephemeris, earth};
    mission::PhasingPlanner planner{gravity};

    mission::PhasingParameters parameters() const {
        mission::PhasingParameters p;
        p.chaser_drag_area_per_mass_m2pkg = dragon_beta;
        p.target_drag_area_per_mass_m2pkg = iss_beta;
        return p;
    }
    // A plan whose start plane meets the ISS plane at the boost (two passes, as a launch planner would).
    mission::PhasingPlan plan(double lead_deg, const mission::PhasingParameters& p, rendezvous_test::Start& start) {
        start = rendezvous_test::start(clock, spice_earth, lead_deg, 0.0);
        const auto first = planner.plan(start.dragon, start.iss.state_gcrf, start.epoch, p, dracos, dragon_propellant_kg);
        if (!first.feasible) return first;
        start = rendezvous_test::start(clock, spice_earth, lead_deg, first.schedule.boost_epoch - start.epoch);
        return planner.plan(start.dragon, start.iss.state_gcrf, start.epoch, p, dracos, dragon_propellant_kg);
    }
};

// Flies the plan impulsively-finite through the planner's own dynamics: the arrival at Waypoint 0.
orbit::RelativeState fly_plan(const Fixture& fx, const mission::PhasingPlan& plan, const rendezvous_test::Start& start) {
    const auto& p = plan.parameters;
    math::StateVector dragon = start.dragon, iss = start.iss.state_gcrf;
    time::Epoch t = start.epoch;
    for (const auto& b : plan.burns) {
        const auto node = fx.planner.retarget(b.kind, dragon, iss, t, plan, dracos);
        REQUIRE(node);
        dragon = gnc::coast_with_drag(fx.gravity, dragon, t, node->epoch, p.chaser_drag_area_per_mass_m2pkg);
        iss = gnc::coast_with_drag(fx.gravity, iss, t, node->epoch, p.target_drag_area_per_mass_m2pkg);
        dragon = mission::apply_finite_burn(fx.gravity, dragon, node->epoch, node->delta_v_gcrf_mps, dracos);
        t = node->epoch;
    }
    return orbit::lvlh_relative_state(iss, dragon);  // at the braking burn = the Waypoint 0 epoch
}
}  // namespace

TEST_CASE("Phasing plan from 120 deg: burns, Waypoint 0, budget", "[rendezvous][phasing][validation]") {
    test::spice_session();
    Fixture fx;
    rendezvous_test::Start start;
    const auto plan = fx.plan(120.0, fx.parameters(), start);
    REQUIRE(plan.feasible);
    REQUIRE(plan.burns.size() == 4);  // boost, coelliptic, approach initiation, braking (no phase burn)
    CHECK(plan.burns[0].kind == mission::PhasingBurnKind::Boost);
    CHECK(plan.burns[3].kind == mission::PhasingBurnKind::Braking);
    for (const auto& b : plan.burns) {
        std::printf("[validation] %-22s at +%.2f h: %.2f m/s\n", mission::german_name(b.kind),
                    (b.node.epoch - start.epoch) / 3600.0, norm(b.node.delta_v_gcrf_mps));
    }
    std::printf("[validation] total %.1f m/s, %.0f kg, Waypoint 0 at +%.2f h, AE metric before AI %.2f\n",
                plan.total_delta_v_mps, plan.propellant_kg, (plan.schedule.waypoint0_epoch - start.epoch) / 3600.0,
                plan.min_ellipsoid_metric_before_ai);
    // Hohmann 205 km -> 418 km (OEM ISS 420.7 km - 2.9 km) is ~123 m/s [D]; CW approach initiation ~0.8 m/s, braking
    // ~0.1 m/s [D]; Demo-2 boost + circularisation 102 m/s plus a 16 m/s phase burn [S]
    CHECK(plan.total_delta_v_mps > 115.0);
    CHECK(plan.total_delta_v_mps < 140.0);  // [E] Hohmann + approach + braking + residual plane error
    CHECK(plan.propellant_kg + plan.parameters.reserve_kg < dragon_propellant_kg);
    CHECK(plan.min_ellipsoid_metric_before_ai > 1.0);
    const auto arrival = fly_plan(fx, plan, start);
    std::printf("[validation] Waypoint 0 arrival (LVLH): (%.1f, %.1f, %.1f) m, (%.3f, %.3f, %.3f) m/s\n",
                arrival.position_m.x, arrival.position_m.y, arrival.position_m.z, arrival.velocity_mps.x,
                arrival.velocity_mps.y, arrival.velocity_mps.z);
    CHECK(norm(arrival.position_m - Vec3{0.0, 0.0, plan.parameters.waypoint0_below_m}) < 100.0);  // [E] spec
}

TEST_CASE("Phasing plan uses a phase burn for a crew-day arrival", "[rendezvous][phasing]") {
    test::spice_session();
    Fixture fx;
    auto p = fx.parameters();
    p.earliest_arrival_s = 20.0 * 3600.0;
    rendezvous_test::Start start;
    const auto plan = fx.plan(60.0, p, start);
    REQUIRE(plan.feasible);
    REQUIRE(plan.burns.front().kind == mission::PhasingBurnKind::Phase);
    CHECK(plan.schedule.waypoint0_epoch - start.epoch >= p.earliest_arrival_s);
    std::printf("[validation] crew day: phase burn %.2f m/s, %d revolutions, total %.1f m/s\n",
                norm(plan.burns.front().node.delta_v_gcrf_mps), plan.schedule.phasing_revolutions,
                plan.total_delta_v_mps);
    CHECK(plan.total_delta_v_mps < 140.0);  // the phase burn is part of the same Hohmann transfer
}

TEST_CASE("Phasing plan refuses a phase beyond the mission limit", "[rendezvous][phasing]") {
    test::spice_session();
    Fixture fx;
    const auto start = rendezvous_test::start(fx.clock, fx.spice_earth, 330.0, 0.0);
    const auto plan = fx.planner.plan(start.dragon, start.iss.state_gcrf, start.epoch, fx.parameters(), dracos,
                                      dragon_propellant_kg);
    CHECK_FALSE(plan.feasible);
    CHECK(plan.reason.find("nicht innerhalb von 28 h") != std::string::npos);
}

TEST_CASE("Phasing plan refuses without the reserve", "[rendezvous][phasing]") {
    test::spice_session();
    Fixture fx;
    const auto start = rendezvous_test::start(fx.clock, fx.spice_earth, 90.0, 0.0);
    const auto plan = fx.planner.plan(start.dragon, start.iss.state_gcrf, start.epoch, fx.parameters(), dracos, 1200.0);
    CHECK_FALSE(plan.feasible);
    CHECK(plan.reason.rfind("Kein Rendezvous: Treibstoff", 0) == 0);
}

TEST_CASE("Phasing campaign over the phase angle", "[rendezvous][phasing][validation][slow]") {
    test::spice_session();
    Fixture fx;
    auto p = fx.parameters();
    p.mission_limit_s = 48.0 * 3600.0;  // every phase closes within two days
    for (double lead = 0.0; lead < 360.0; lead += 30.0) {
        CAPTURE(lead);
        rendezvous_test::Start start;
        const auto plan = fx.plan(lead, p, start);
        REQUIRE(plan.feasible);
        const auto arrival = fly_plan(fx, plan, start);
        const double miss = norm(arrival.position_m - Vec3{0.0, 0.0, p.waypoint0_below_m});
        std::printf("[validation] lead %3.0f deg: %d lap(s), Waypoint 0 +%5.2f h, %.1f m/s, AE %.2f, miss %.1f m\n",
                    lead, plan.schedule.extra_laps, (plan.schedule.waypoint0_epoch - start.epoch) / 3600.0,
                    plan.total_delta_v_mps, plan.min_ellipsoid_metric_before_ai, miss);
        CHECK(plan.total_delta_v_mps < 140.0);
        CHECK(plan.min_ellipsoid_metric_before_ai > 1.0);
        CHECK(miss < 100.0);
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[phasing]" > tmp/i3_t4.txt 2>&1`
Expected: build error (`phasing_planner.hpp` not found).

- [ ] **Step 3: The header**

Create `core/include/starsim/mission/phasing_planner.hpp`:

```cpp
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/mission/finite_burn.hpp"
#include "starsim/mission/phasing_schedule.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

enum class PhasingBurnKind { Phase, Boost, Coelliptic, ApproachInitiation, MidCourse, Braking };
const char* german_name(PhasingBurnKind kind);

struct PhasingBurn {
    PhasingBurnKind kind = PhasingBurnKind::Boost;
    gnc::BurnNode node;
};

struct PhasingPlan {
    bool feasible = false;
    std::string reason;  // German when refused
    PhasingSchedule schedule;
    std::vector<PhasingBurn> burns;  // in order; the mid-course correction is planned in flight only
    double total_delta_v_mps = 0.0;
    double propellant_kg = 0.0;      // rocket equation at the engines' Isp
    double min_ellipsoid_metric_before_ai = 0.0;  // of the planned arc, sampled every minute; > 1: outside
    PhasingParameters parameters;
};

// Coelliptic phasing in the full force model (the flight's gravity model) with orbital drag on both bodies
// (gnc::coast_with_drag with the parameters' C_D A / m). Timing from phasing_schedule; then each burn is targeted from
// the state the previous one leaves, the burn flown as a finite burn of `engines` (apply_finite_burn):
// - Phase: prograde magnitude (secant) so that the chaser is half a transfer behind the coelliptic aim point at the boost;
// - Boost: prograde magnitude (secant) so that the radius at the coelliptic epoch is the coelliptic radius (a
//   180 deg position target would be singular out of plane); waiting plans solve the boost epoch for the phase first;
// - Coelliptic: position targeter to the approach initiation point (range 7.5 km, dh below) at its epoch (3D: removes
//   the residual plane error);
// - Approach initiation: position targeter (CW first guess) to Waypoint 0 at its epoch;
// - Mid-course (in flight only): the same target, zero first guess;
// - Braking at Waypoint 0: the velocity that leaves the chaser at rest in the target's LVLH frame.
class PhasingPlanner {
public:
    explicit PhasingPlanner(const environment::GravityModel& gravity);

    // `engines`: the burn engines at the current mass; `propellant_kg`: on board (budget against reserve_kg).
    PhasingPlan plan(const math::StateVector& chaser, const math::StateVector& target, time::Epoch epoch,
                     const PhasingParameters& parameters, const FiniteBurnModel& engines, double propellant_kg) const;

    // The burn of `kind` re-targeted from the live states at `epoch` (before the burn), on the plan's epochs. Empty when
    // the targeting does not converge, the plan has no such burn, or a mid-course correction is below its threshold.
    std::optional<gnc::BurnNode> retarget(PhasingBurnKind kind, const math::StateVector& chaser,
                                          const math::StateVector& target, time::Epoch epoch, const PhasingPlan& plan,
                                          const FiniteBurnModel& engines) const;

    // Along-track miss (m) of the boost geometry predicted from the live states: the chaser's angle at the boost epoch
    // against the coelliptic aim point minus half a transfer, times the target radius; > 0: the chaser is late.
    double boost_phase_error_m(const math::StateVector& chaser, const math::StateVector& target, time::Epoch epoch,
                               const PhasingPlan& plan) const;

private:
    math::StateVector coast(const math::StateVector& s, time::Epoch from, time::Epoch to, double beta) const;
    double phase_error_rad(const math::StateVector& chaser_at_boost, const math::StateVector& target_at_boost,
                           time::Epoch boost_epoch, const PhasingParameters& p, double boost_to_coelliptic_s) const;
    std::optional<gnc::BurnNode> solve_phase(const math::StateVector& chaser, const math::StateVector& target,
                                             time::Epoch epoch, const PhasingPlan& plan,
                                             const FiniteBurnModel& engines) const;
    std::optional<gnc::BurnNode> solve_boost(const math::StateVector& chaser, const math::StateVector& target,
                                             time::Epoch epoch, const PhasingPlan& plan,
                                             const FiniteBurnModel& engines) const;
    std::optional<gnc::BurnNode> solve_position(PhasingBurnKind kind, const math::StateVector& chaser,
                                                const math::StateVector& target, time::Epoch epoch,
                                                const PhasingPlan& plan, const FiniteBurnModel& engines) const;
    gnc::BurnNode braking(const math::StateVector& chaser, const math::StateVector& target, time::Epoch epoch,
                          const PhasingPlan& plan) const;

    const environment::GravityModel& gravity_;
};

}  // namespace starsim::mission
```

- [ ] **Step 4: The implementation**

Create `core/src/mission/phasing_planner.cpp`. Key pieces (write them out in full; helper names as below):

```cpp
#include "starsim/mission/phasing_planner.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/frames/lvlh.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/position_targeter.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"

namespace starsim::mission {

using math::StateVector;
using math::Vec3;

namespace {
constexpr int secant_iterations = 20;
constexpr double phase_tolerance_m = 50.0;      // [E] along-track at the boost (the coelliptic burn targets the rest)
constexpr double radius_tolerance_m = 20.0;     // [E] boost: coelliptic radius at the coelliptic epoch
constexpr double position_tolerance_m = 5.0;    // [E] coelliptic / approach initiation / mid-course aim points
constexpr double ellipsoid_sample_s = 60.0;
constexpr double max_boost_shift_s = 6.0 * 3600.0;  // [E] a waiting plan's boost epoch moves at most this far

double hohmann_departure_mps(double r1, double r2, double gm) {
    return std::sqrt(gm / r1) * (std::sqrt(2.0 * r2 / (r1 + r2)) - 1.0);
}

// Secant on f(x) = 0 from x0, x1; empty when it does not reach |f| <= tolerance.
template <class F>
std::optional<double> secant(F f, double x0, double x1, double tolerance) {
    double f0 = f(x0), f1 = f(x1);
    for (int i = 0; i < secant_iterations; ++i) {
        if (std::abs(f1) <= tolerance) return x1;
        if (f1 == f0) return std::nullopt;
        const double x2 = x1 - f1 * (x1 - x0) / (f1 - f0);
        x0 = x1;
        f0 = f1;
        x1 = x2;
        f1 = f(x1);
    }
    return std::abs(f1) <= tolerance ? std::optional<double>(x1) : std::nullopt;
}
}  // namespace

const char* german_name(PhasingBurnKind kind) {
    switch (kind) {
        case PhasingBurnKind::Phase: return "Phasenmanöver";
        case PhasingBurnKind::Boost: return "Anhebung";
        case PhasingBurnKind::Coelliptic: return "Koelliptisches Manöver";
        case PhasingBurnKind::ApproachInitiation: return "Annäherungsbeginn";
        case PhasingBurnKind::MidCourse: return "Kurskorrektur";
        case PhasingBurnKind::Braking: return "Bremsmanöver";
    }
    return "?";
}

PhasingPlanner::PhasingPlanner(const environment::GravityModel& gravity) : gravity_(gravity) {}

StateVector PhasingPlanner::coast(const StateVector& s, time::Epoch from, time::Epoch to, double beta) const {
    if (to - from == 0.0) return s;
    return gnc::coast_with_drag(gravity_, s, from, to, beta);
}
```

The remaining functions, specified exactly:

- `phase_error_rad(chaser_b, target_b, p, t_h)` (all angles in the target's orbit plane at the boost epoch, measured
  from `target_b` by `in_plane_angle_rad(target_b, .)`, so no epoch mixing and no wrap at 180 deg):

  ```cpp
  const StateVector target_c = coast(target_b, t_b, t_b + t_h, p.target_drag_area_per_mass_m2pkg);
  const double r = norm(target_c.position_m);
  const Vec3 c = point_behind_below(target_c, coelliptic_behind_m(p, r, gm), p.coelliptic_height_m);
  // the chaser should sit half a transfer (pi) before the aim point
  const double required = wrap_pi(in_plane_angle_rad(target_b, c) - constants::pi);
  return wrap_pi(required - in_plane_angle_rad(target_b, chaser_b.position_m));  // > 0: the chaser is late
  ```

  (`t_b` is passed in as well - add it to the signature: `phase_error_rad(chaser_b, target_b, t_b, p, t_h)`;
  `wrap_pi` maps to (-pi, pi], a local helper; `gm` is `constants::earth_gm_m3ps2`: the planner is Earth-centred, as
  `PositionTargeter`.) The schedule's `required_at_boost = pi + behind / r - n_i t_h` is the same condition in mean
  motions.
- `boost_phase_error_m`: coast both live states to `plan.schedule.boost_epoch`, return
  `phase_error_rad(...) * |r_target|`.
- `solve_phase`: `t_p = plan.schedule.phase_epoch`; chaser at `t_p` (coast); `f(dv) = phase_error_rad(chaser after a
  finite prograde burn of dv at t_p, coasted to the boost epoch; target at the boost epoch; ...)`; first guesses
  `dv0 = hohmann_departure_mps(|r|, phase_orbit_apoapsis_radius_m)` and `1.05 dv0`; tolerance
  `phase_tolerance_m / |r_target|`; node `{t_p, dv * unit(v at t_p)}`.
- `solve_boost`: `t_b`, `t_c` from the schedule; `f(dv) = |r_chaser(t_c)| - |c|` after a finite prograde burn of dv
  at `t_b` (chaser drag); first guesses `hohmann_departure_mps(|r(t_b)|, |c|)` and `1.02x`; tolerance
  `radius_tolerance_m`.
- `solve_position(kind, ...)`: Coelliptic: burn at `t_c`, aim `point_behind_below(target(t_ai),
  sqrt(range^2 - dh^2), dh)` at `t_ai`, first guess the circular velocity at the current radius along the horizontal
  minus the current velocity. ApproachInitiation: burn at `t_ai`, aim `point_behind_below(target(t_wp0), 0,
  waypoint0_below_m)` at `t_wp0`, first guess from `orbit::cw_transfer_velocity(rel.position_m, wp0_lvlh, n_target,
  approach_transfer_s)` minus `rel.velocity_mps`, rotated to GCRF with `frames::lvlh_to_inertial(target(t_ai))`.
  MidCourse: burn at `t_ai + mid_course_after_s`, same aim, zero first guess; empty when `|dv| <
  mid_course_threshold_mps`. All with a `PositionTargeter` that has `set_drag_area_per_mass(chaser beta)` and
  `set_finite_burn(engines)`, tolerance `position_tolerance_m`; empty when not converged.
- `braking(...)`: coast both to `t_wp0`; `dv = target.v + omega x (r_c - r_t) - chaser.v` with
  `omega = frames::lvlh_rate_inertial_radps(target)`; node at `t_wp0`.
- `retarget(kind, ...)`: dispatch to the solvers above with the live states (`epoch` <= the burn epoch). For a waiting
  plan (`phasing_revolutions == 0`) the boost is not re-timed here - the session re-plans when
  `boost_phase_error_m` exceeds its tolerance (Task 5).
- `plan(...)`:
  1. `schedule = phasing_schedule(chaser, target, epoch, p, gm)`; refused -> `reason` copied.
  2. Waiting plan: refine the boost epoch in the full model: `g(t_b) = phase_error_rad(chaser(t_b), target(t_b), p,
     t_h)`; secant on `t_b` from the schedule's epoch and `+60 s`, tolerance `phase_tolerance_m / r`; then shift the
     coelliptic, approach initiation and Waypoint 0 epochs by the same amount. A shift beyond `max_boost_shift_s`, a
     boost earlier than `first_burn_lead_s`, or an arrival beyond `mission_limit_s`: refused
     ("Kein Rendezvous: Zielführung konvergiert nicht" / the schedule's limit text).
  3. Chain: for each burn in order (Phase if `phasing_revolutions > 0`, Boost, Coelliptic, ApproachInitiation,
     Braking) solve from the states the previous burn left (finite burn via `apply_finite_burn`; the engines' mass
     drops by the rocket equation after each burn); a solver returning empty refuses the plan with "Kein Rendezvous:
     Zielführung konvergiert nicht (<german_name>)".
  4. While chaining, sample the chaser and the target every `ellipsoid_sample_s` from `epoch` to the approach
     initiation epoch (coast in 60 s chunks) and keep the minimum `approach_ellipsoid_metric`; <= 1 refuses: "Kein
     Rendezvous: Bahn tritt vor dem Annäherungsbeginn in das Annäherungsellipsoid ein".
  5. Budget: `total_delta_v_mps` = sum of the burn magnitudes; `isp = thrust / (mass_flow g0)`;
     `propellant_kg = mass (1 - exp(-total / (isp g0)))`; `propellant_kg + reserve_kg > propellant on board` refuses:
     "Kein Rendezvous: Treibstoff reicht nicht (<need> kg + Reserve <reserve> kg, an Bord <have> kg)" (German number
     format as `sim::german()` uses it is not available in `mission` - plain integers via `std::lround`).
  6. `feasible = true`, `parameters = p`.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[phasing]" > tmp/i3_t5.txt 2>&1`
Then `grep "\[validation\]" tmp/i3_t5.txt`. Expected: all `[phasing]` cases pass; the 120 deg plan around 120-130 m/s,
Waypoint 0 miss < 100 m (expect metres: the flown model equals the planning model); the campaign 12/12. Record the
printed numbers in the ledger for the docs. If the boost's radius secant fails for a phase burn plan, check that the
phase orbit's apoapsis is below the coelliptic radius (the boost then raises the apoapsis at the burn point).

- [ ] **Step 6: Commit**

```bash
git add core/include/starsim/mission/phasing_planner.hpp core/src/mission/phasing_planner.cpp core/tests/rendezvous_test_support.hpp core/tests/test_phasing_planner.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat(mission): coelliptic phasing planner in the full force model with drag"
```

Run `-Fast` and push.

---

### Task 5: The rendezvous flight in the session

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp` (AutopilotState, RendezvousStep, API, members)
- Create: `core/src/sim/flight_session_rendezvous.cpp`
- Modify: `core/src/sim/flight_session.cpp` (`to_string`, `run_autopilot` case, `coast_allowance_s`, the RCS burn
  completion at ~line 293 and the main-engine completion at ~line 312, body clearing next to the rendezvous target)
- Create: `core/tests/test_rendezvous_phasing_session.cpp`
- Modify: `core/CMakeLists.txt` (add `src/sim/flight_session_rendezvous.cpp` after `src/sim/flight_session_lunar_descent.cpp`),
  `core/tests/CMakeLists.txt` (append `test_rendezvous_phasing_session.cpp`)

**Interfaces:**
- Consumes: Task 4 (`PhasingPlanner`, `PhasingPlan`, `PhasingBurnKind`, `german_name`, `boost_phase_error_m`),
  `gnc::estimate_drag_area_per_mass(gravity, reference, reference_epoch, now, now_epoch, guess)`,
  `execute_burn(node)` (Draco translation along +x for the Dragon), `burn_configurations()`, `station_index()`,
  `station_impulses_` (the station's pending impulses), `german(value, decimals)` (number format helper in
  `core/src/sim/flight_session_lunar_detail.hpp`), `controlled_state()`, `math::Quat::from_axis_angle(axis, angle)`.
- Produces:
  - `AutopilotState::Rendezvous` (`to_string`: "Rendezvous").
  - `enum class RendezvousStep { None, Coast, Burn, Waypoint0 };` `const char* to_string(RendezvousStep)`.
  - `struct RendezvousBurnRecord { mission::PhasingBurnKind kind; gnc::BurnNode node; double mass_kg; };`
  - `bool FlightSession::plan_rendezvous(mission::PhasingParameters parameters)` (controlled body against the station;
    the drag fields are filled by the session when 0); `RendezvousStep rendezvous_step() const`;
    `const mission::PhasingPlan* rendezvous_plan() const`; `const std::string& rendezvous_status() const` (German);
    `const std::vector<RendezvousBurnRecord>& rendezvous_burns() const`; `int rendezvous_replans() const`;
    `const std::vector<std::string>& rendezvous_warnings() const` (German: a station impulse inside the plan).
  - Constants: `rendezvous_check_lead_s = 3.0 * 3600.0` ([E] the boost phase check ~2 orbits before the boost, as
    `deorbit_replan_lead_s`), `rendezvous_replan_tolerance_m = 2000.0` ([E]), `rendezvous_track_min_arc_s =
    3.0 * 3600.0` ([E] shorter arcs do not tell the drag), `dragon_free_molecular_cd = 2.2` ([E] blunt body in free
    molecular flow) - all `static constexpr` in `FlightSession` with these comments.

Behaviour (write it in `flight_session_rendezvous.cpp`, pattern of `flight_session_lunar.cpp`):
- `plan_rendezvous(p)`: refuse (false + German `rendezvous_status_`) when there is no station ("Kein Rendezvous: keine
  Station"), the controlled body is the station, the autopilot is not Off / Orbit, or a rendezvous or lunar flight is
  under way. Drag guesses when the fields are 0: target `C_D A / m` from the station stage's
  `aero.fixed_drag_area_m2 x aero.fixed_drag_coefficient / mass`; chaser `dragon_free_molecular_cd x pi (d/2)^2 /
  mass` with `d` the largest stage diameter. Engines: `burn_configurations()` empty -> the RCS capability along +x (as
  `start_burn` picks it: `vehicle.rcs_force_capability({1,0,0})`, else -x); `FiniteBurnModel{mass, thrust,
  mass_flow}`. `PhasingPlanner(gravity_).plan(controlled_state(), station state, epoch_, p, engines,
  vehicle.rcs_propellant_kg())` inside try/catch (a throwing planner: "Kein Rendezvous: Planung fehlgeschlagen").
  Feasible -> store plan, `deadline_ = epoch_ + p.mission_limit_s`, `earliest_ = epoch_ + p.earliest_arrival_s`,
  burn index 0, the drag tracking references (both states and `epoch_`), `autopilot_.state = Rendezvous`,
  `attitude_mode_ = {}`, step `Coast` with `step_epoch_ = command epoch of burn 0` (below); warnings for every
  station impulse between now and Waypoint 0 ("Achtung: Bahnanhebung der ISS um <x> m/s während des Rendezvous").
- Command epoch of a burn: `node.epoch - 0.5 * duration - burn_preparation_s - burn_drift_lead_s` (the Dragon turns in
  the drift lead, as the capsule deorbit), at least now. The Boost of a waiting plan gets an extra check at
  `boost_epoch - rendezvous_check_lead_s`.
- `run_rendezvous` (step Coast, when `epoch_ >= step_epoch_`): update the drag estimates (chaser arc since the last
  burn, target arc since the plan or its last impulse, each only when the arc is at least
  `rendezvous_track_min_arc_s`); before the Boost of a waiting plan: `boost_phase_error_m` with the updated drag; above
  `rendezvous_replan_tolerance_m` -> re-plan (`plan()` from the live states with `mission_limit_s = deadline_ - epoch_`,
  `earliest_arrival_s = max(0, earliest_ - epoch_)`, updated drag; feasible: replace the plan, burn index 0,
  `++replans_`, status "Neu geplant: ..."; refused: abort with its reason). Then `retarget` the burn due; empty for a
  mid-course correction: skip it; empty otherwise: re-plan once as above (only before the boost), else abort
  ("Rendezvous abgebrochen: <german_name> nicht berechenbar"). `execute_burn(node)`; false -> abort ("Rendezvous
  abgebrochen: Brennvorgang nicht möglich"). Record the burn, step `Burn`, status "<german_name>: <dv> m/s".
- After a burn completes (state returns to Rendezvous with step `Burn`): store the chaser drag reference (state now);
  next burn: after the ApproachInitiation burn insert the MidCourse (its command epoch is its node epoch minus half
  its duration minus `burn_preparation_s`: inside the drift lead of the previous burn's end); after Braking -> step
  `Waypoint0`, autopilot Off, status "Wegpunkt 0 erreicht" (i4 continues there; the Dragon drifts passively safe below
  and ahead of the station).
- Status while coasting: "Phasing: <german_name> in <h,h> h" (German decimal comma via `german`).
- `coast_allowance_s`: in Rendezvous, `min(allowance, step_epoch_ - epoch_)` (a wreck: 0, as Lunar).
- Burn completion (both branches): `autopilot_.state = lunar_step_ != None ? Lunar : rendezvous_step_ != None ?
  Rendezvous : ...` (keep the existing ship-return branch).
- Clearing: `reset_rendezvous()` wherever the rendezvous target is cleared with the bodies (place_body /
  launch_from_pad / place_in_orbit, and a change of the controlled body), and in `disengage_autopilot()`.
- `STARSIM_DEBUG_RENDEZVOUS=1` prints every status change with the elapsed hours, range to the station and mass (as
  `lunar_debug_`).

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_rendezvous_phasing_session.cpp`:

```cpp
// ISS i3: the rendezvous flight in the flight session - Draco burns, drag tracking, re-plan, arrival at Waypoint 0.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "rendezvous_test_support.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "starsim/sim/flight_session.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
struct Flight {
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    sim::FlightSession session{ephemeris, earth, rendezvous_test::config_dir};
    rendezvous_test::Start start;

    Flight(double lead_deg, double plane_match_s) {
        start = rendezvous_test::start(clock, earth, lead_deg, plane_match_s);
        session.place_in_orbit(config::load_vehicle(rendezvous_test::config_dir, "crew_dragon"), start.dragon,
                               start.epoch);
        session.add_station(start.iss.vehicle, start.iss.state_gcrf, start.iss.epoch, start.iss.impulses);
    }

    // Flies until the step machine ends; returns the smallest approach-ellipsoid metric before the approach initiation.
    double fly(double max_hours) {
        auto& s = session;
        const size_t iss = *s.station_index();
        double min_metric = 1e9;
        const auto end = start.epoch + max_hours * 3600.0;
        while (s.epoch() - end < 0.0 && s.rendezvous_step() != sim::RendezvousStep::None &&
               s.rendezvous_step() != sim::RendezvousStep::Waypoint0) {
            s.advance_to(std::min(s.epoch() + 60.0, end), 100000);
            const bool before_ai = std::none_of(s.rendezvous_burns().begin(), s.rendezvous_burns().end(), [](const auto& b) {
                return b.kind == mission::PhasingBurnKind::ApproachInitiation;
            });
            if (before_ai) {
                const auto& t = s.body(iss).state();
                min_metric = std::min(min_metric, mission::approach_ellipsoid_metric(
                                                      {t.position_gcrf_m, t.velocity_gcrf_mps},
                                                      s.body(s.controlled_index()).state().position_gcrf_m));
            }
        }
        return min_metric;
    }

    orbit::RelativeState relative() const {
        const auto& t = session.body(*session.station_index()).state();
        const auto& c = session.body(session.controlled_index()).state();
        return orbit::lvlh_relative_state({t.position_gcrf_m, t.velocity_gcrf_mps}, {c.position_gcrf_m, c.velocity_gcrf_mps});
    }

    void report(const char* name) const {
        for (const auto& b : session.rendezvous_burns()) {
            std::printf("[validation] %s: %-22s +%.2f h %.2f m/s\n", name, mission::german_name(b.kind),
                        (b.node.epoch - start.epoch) / 3600.0, norm(b.node.delta_v_gcrf_mps));
        }
        const auto r = relative();
        std::printf("[validation] %s: %s, re-plans %d, LVLH (%.1f, %.1f, %.1f) m (%.3f, %.3f, %.3f) m/s, %.0f kg used\n",
                    name, session.rendezvous_status().c_str(), session.rendezvous_replans(), r.position_m.x,
                    r.position_m.y, r.position_m.z, r.velocity_mps.x, r.velocity_mps.y, r.velocity_mps.z,
                    2562.0 - session.body(session.controlled_index()).vehicle().rcs_propellant_kg());
    }
};
}  // namespace

TEST_CASE("Rendezvous plan refusals in the session", "[rendezvous][phasing][session]") {
    test::spice_session();
    Flight f(330.0, 0.0);
    CHECK_FALSE(f.session.plan_rendezvous({}));  // beyond 28 h
    CHECK(f.session.rendezvous_status().find("nicht innerhalb") != std::string::npos);
    CHECK(f.session.autopilot().state == sim::AutopilotState::Off);
    sim::FlightSession lonely{f.ephemeris, f.earth, rendezvous_test::config_dir};
    lonely.place_in_orbit(config::load_vehicle(rendezvous_test::config_dir, "crew_dragon"), f.start.dragon, f.start.epoch);
    CHECK_FALSE(lonely.plan_rendezvous({}));
    CHECK(lonely.rendezvous_status() == "Kein Rendezvous: keine Station");
}

TEST_CASE("Rendezvous flight from 30 deg to Waypoint 0", "[rendezvous][phasing][session][validation][slow]") {
    test::spice_session();
    Flight f(30.0, 3.0 * 3600.0);
    REQUIRE(f.session.plan_rendezvous({}));
    CHECK(f.session.autopilot().state == sim::AutopilotState::Rendezvous);
    const double metric = f.fly(12.0);
    f.report("30 deg");
    REQUIRE(f.session.rendezvous_step() == sim::RendezvousStep::Waypoint0);
    const auto r = f.relative();
    CHECK(norm(r.position_m - Vec3{0.0, 0.0, 400.0}) < 100.0);  // [E] spec
    CHECK(norm(r.velocity_mps) < 0.5);                           // [E] spec
    CHECK(metric > 1.0);
    CHECK(norm(r.position_m) > mission::keep_out_sphere_radius_m);
}

TEST_CASE("Rendezvous flight with a denser truth atmosphere re-plans and arrives", "[rendezvous][phasing][session][validation][slow]") {
    test::spice_session();
    Flight f(150.0, 13.0 * 3600.0);
    f.session.set_air_density_factor(1.5);  // truth only: the planner starts from US76 and the C_D guess
    REQUIRE(f.session.plan_rendezvous({}));
    const double metric = f.fly(28.0);
    f.report("150 deg, air x1.5");
    REQUIRE(f.session.rendezvous_step() == sim::RendezvousStep::Waypoint0);
    CHECK(f.session.rendezvous_replans() >= 1);  // ~13 h at 205 km with 1.5x air: tens of km of phase error
    const auto r = f.relative();
    CHECK(norm(r.position_m - Vec3{0.0, 0.0, 400.0}) < 100.0);
    CHECK(norm(r.velocity_mps) < 0.5);
    CHECK(metric > 1.0);
}
```

Note: if the 1.5x case does not need a re-plan (the tracked drag already corrects at the 3 h check without exceeding
the 2 km tolerance), do not loosen anything: report the measured phase error at the check, and replace the
`replans >= 1` check by the measured outcome only after asking the user - the test's purpose is the closed loop on a
wrong drag model.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[session]" > tmp/i3_t6.txt 2>&1`
Expected: build error (`plan_rendezvous` / `RendezvousStep` unknown).

- [ ] **Step 3: Header changes**

In `core/include/starsim/sim/flight_session.hpp`:
- `#include "starsim/mission/phasing_planner.hpp"`.
- `enum class AutopilotState { ..., Lunar, Rendezvous };` with a comment line: "Rendezvous: a phasing flight to a
  station between its burns (FlightSession::plan_rendezvous)".
- After `LunarStep`'s `to_string`:

```cpp
// Steps of a rendezvous (plan_rendezvous, ISS i3): Coast until the next burn's command epoch, Burn while it is under way
// (the autopilot state is Burn meanwhile), Waypoint0 after the braking burn (400 m below the station; i4 continues).
enum class RendezvousStep { None, Coast, Burn, Waypoint0 };
const char* to_string(RendezvousStep step);

struct RendezvousBurnRecord {
    mission::PhasingBurnKind kind = mission::PhasingBurnKind::Boost;
    gnc::BurnNode node;
    double mass_kg = 0.0;  // at the command
};
```

- Public API block after the lunar mission block (with the comment from the Interfaces section above) and the
  constants; private: `void run_rendezvous(Flying& flying); void reset_rendezvous(); void rendezvous_abort(const
  std::string& status); bool rendezvous_replan(); time::Epoch rendezvous_command_epoch(const gnc::BurnNode& node)
  const; void rendezvous_track_drag(); mission::FiniteBurnModel rendezvous_engines() const;` and members
  `RendezvousStep rendezvous_step_ = RendezvousStep::None; std::optional<mission::PhasingPlan> rendezvous_plan_;
  size_t rendezvous_next_ = 0; bool rendezvous_mid_course_ = false; time::Epoch rendezvous_step_epoch_,
  rendezvous_deadline_, rendezvous_earliest_; std::optional<time::Epoch> rendezvous_boost_check_; math::StateVector
  rendezvous_chaser_ref_, rendezvous_target_ref_; time::Epoch rendezvous_chaser_ref_epoch_,
  rendezvous_target_ref_epoch_; std::string rendezvous_status_; std::vector<RendezvousBurnRecord> rendezvous_burns_;
  std::vector<std::string> rendezvous_warnings_; int rendezvous_replans_ = 0; bool rendezvous_debug_ = false;
  std::string rendezvous_debug_last_;`

- [ ] **Step 4: Implement `flight_session_rendezvous.cpp` and the session hooks**

Write the behaviour listed above. The station impulse reset of the target drag reference: when the station's
`station_impulses_` front epoch passes (compare the count at the reference), take a new reference after it.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[rendezvous]" > tmp/i3_t7.txt 2>&1`
Then `grep "\[validation\]\|FAILED\|passed\|failed" tmp/i3_t7.txt`. Expected: all pass; record burns, arrival state,
re-plans and propellant for the docs. Use `STARSIM_DEBUG_RENDEZVOUS=1` when a flight ends early.
Then the full suite (this task touches the flight session): `powershell -ExecutionPolicy Bypass -File tools/build.ps1
-NoBindings > tmp/i3_full.txt 2>&1`; tail for the totals.

- [ ] **Step 6: Commit**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/src/sim/flight_session_rendezvous.cpp core/tests/test_rendezvous_phasing_session.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat(sim): rendezvous phasing flight with drag tracking and re-plan"
```

Run `-Fast` and push.

---

### Task 6: Documentation, spec update, final review

**Files:**
- Modify: `docs/subsystems.md` (new section "### Phasing (i3)" after "Crew Dragon translation and relative navigation (i2)")
- Modify: `CLAUDE.md` (module list: `orbit` + "Clohessy-Wiltshire relative motion"; `mission` + "coelliptic phasing to
  the ISS (schedule + full-model planner with drag)"; `sim` + "rendezvous phasing flight")
- Modify: `docs/superpowers/specs/2026-10-19-iss-docking-design.md` (status line: i3 implemented; "i3 design" section:
  the four corrections from the top of this plan, with the Crew-2 numbers)

- [ ] **Step 1: Write the subsystems section**

Content (numbers from the ledger of Tasks 4-5): the profile and its sources (Crew-2 timeline, Demo-2 burn sizes, the
derived dh 2.9 km / 45 min / 25 min / 2.8 h), the four spec corrections, the schedule (wait vs phase burn; extra
laps; mission limit 28 h), the planner (which burn targets what, why the boost is a radius secant), drag (US76 in
the planner, C_D guesses, tracking, re-plan tolerance and check lead), the session API and statuses, validation
numbers (plan at 120 deg, campaign table, both flights incl. propellant), and `STARSIM_DEBUG_RENDEZVOUS`.

- [ ] **Step 2: CLAUDE.md and spec**

Apply the edits listed under Files.

- [ ] **Step 3: Full suite, smoke test, commit, push**

```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1 > tmp/i3_final.txt 2>&1
external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd > tmp/i3_smoke.txt 2>&1
```

```bash
git add docs/subsystems.md CLAUDE.md docs/superpowers/specs/2026-10-19-iss-docking-design.md
git commit -m "docs: ISS i3 phasing"
git push origin main
```

- [ ] **Step 4: Final review**

A fresh reviewer (most capable model) reviews the i3 commits against the spec and this plan; fix pass, full suite,
push; ledger + memory updated; ask the user for `/clear` and name the next step (writing-plans for i4).
