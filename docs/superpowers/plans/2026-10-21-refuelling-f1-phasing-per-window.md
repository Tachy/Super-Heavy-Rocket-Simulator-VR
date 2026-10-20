# Refuelling f1: Phasing Orbit per Launch Window - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Each tanker (and the HLS) picks its phasing orbit per launch window from a configured candidate set - below
the depot to catch up, above it to let the depot catch up - so more plane crossings give an arrival; the calculator
books the per-window cost and the flown tanker raises to the chosen orbit.

**Architecture:** `gnc::next_raise_node` plans the apsis burns from the 120 x 190 km insertion to a target orbit (one
burn below the depot, two above). `mission::phasing_schedule` learns the mirrored case (chaser above the target:
the target catches up, the boost is a descending transfer). `mission::plan_launch_window` estimates the raise
two-body for every candidate at each plane crossing and keeps the cheapest feasible one by a pure
`PhasingCostModel`. The session's Raise leg flies `next_raise_node` until done; the campaign setup books the cost
difference to the reference flight's own window.

**Tech Stack:** C++20 core (float64), Catch2, toml++ via `config::detail::Document`, godot-cpp bindings.

**Spec:** `docs/superpowers/specs/2026-10-21-refuelling-followups-design.md` (section "f1").

## Global Constraints

- SI units internally, frame and unit in names; German UI / refusal strings, English code and comments.
- No magic numbers: every new value carries a provenance tag [S]/[E]/[D] and a comment.
- Modules depend downwards only (`mission` may use `gnc` and `orbit`; never `sim`).
- The ISS / Crew Dragon path stays bit-identical: empty `phasing_orbits` = today's behaviour.
- `bindings/` stays thin: lever parsing only.
- Tests per CLAUDE.md: only the affected tags (`-TestFilter`), new cases > ~40 s tagged `[slow]`; no shared temp files.
- Commits: Conventional Commits on `main`, push after each task once its tests are green.
- Nothing is tuned towards a target count: the new campaign answer is measured and reported.

## Plan decisions (await the user's approval)

1. **Mirrored schedule for orbits above the depot.** Today `phasing_schedule` refuses a chaser above the coelliptic
   height. Above: the target (lower, faster) catches up; the chaser waits until the same `required_at_boost` phase
   is reached (the formula is symmetric: half a transfer takes pi either way), then a retrograde boost descends to the
   coelliptic orbit. Its periapsis must stay `coelliptic_height_m` above the target's semi-major axis [E] (no crossing
   of the depot's orbit while waiting). The phase-burn branch (slowing down) stays below-only. `PhasingPlanner`
   already solves a signed boost (`hohmann_departure_mps` < 0 for a lower target) - checked by a test.
2. **The raise is modelled in the window estimate.** Today the window places the separation directly on 170 x 190 km.
   Now the separation is on the ascent's insertion orbit (120 x 190 km) and every candidate's raise is propagated
   two-body (`gnc::next_raise_node` + `orbit::propagate_two_body`, impulsive) before `phasing_schedule` runs from the
   raised state with the remaining mission limit. This shifts the reference window slightly: the r5 reference flights
   are re-measured (Task 5).
3. **Cost model** `PhasingCostModel { mass_kg, exhaust_velocity_mps, boil_off_kgps }`: propellant for raise
   delta-v + boost transfer (+ phase burn) by the rocket equation, plus boil-off x (Waypoint 0 - separation). Mass
   and boil-off rate are mission config values [D] (`planning_mass_t` = 308: dry 209 + residue 99.2, r5;
   `planning_boil_off_t_per_day` = 12, r6), the exhaust velocity is the vehicle's deorbit Raptor set. The flown
   mission and the calculator use the same config, so both choose the same orbit (needed for f2).
4. **Candidates [E]** in `[phasing] candidates_km` (periapsis, apoapsis): (150,190) (160,190) (170,190) (180,190)
   (190,190) (220,240) (250,270). `periapsis_km = 170` stays as the reference set (`--reference-phasing`, r6 answer).
5. **Calculator baseline = the reference flight's own window** (same candidates, same start): delivery = base +
   boil-off term (as r6) - (chosen propellant cost - reference propellant cost). No fixed 170 x 190 baseline.
6. **HLS:** same candidates; its extra phasing propellant over its reference window is demanded from the depot via a
   new `CampaignFlight::extra_demand_kg` (x (1 + vent)) at the HLS check.
7. **Raise done** when both apsides are within 1 km [E] of the target (or above it); `first_burn_lead_s` (30 min
   checkout) is kept after the raise.
8. **Honest expectation:** in 48 h the lowest candidate closes up to ~65 deg of phase, the highest ~110 deg the
   other way - roughly half the ~13.8 d phase cycle. Expected: several arrivals per cycle instead of one, not one per
   day. The measured answer is reported as is.

## Review Focus

- A candidate whose raise does not fit the mission limit (raise + phasing > limit) must be infeasible, not negative
  time - test in Task 2.
- The ISS window (empty candidates) unchanged to the second - test in Task 2.
- A Raise burn that runs dry ends the mission with `burn_abort_reason`, not a phasing start - test in Task 3.
- An above-depot orbit whose periapsis crosses the depot is refused - test in Task 1.
- Calculator with `--reference-phasing` keeps the r6 tanker count within the re-measured reference - test in Task 5.

---

### Task 1: Raise burns and the mirrored phasing schedule

**Files:**
- Modify: `core/include/starsim/gnc/burn_node.hpp`, `core/src/gnc/burn_node.cpp`
- Modify: `core/include/starsim/mission/phasing_schedule.hpp`, `core/src/mission/phasing_schedule.cpp`
- Create: `core/tests/test_raise_to_orbit.cpp` (add to `core/tests/CMakeLists.txt` beside `test_phasing_schedule.cpp`)
- Modify: `core/tests/test_phasing_schedule.cpp`, `core/tests/test_phasing_planner.cpp`

**Interfaces:**
- Produces: `std::optional<gnc::BurnNode> gnc::next_raise_node(const math::StateVector& state, time::Epoch now, double gm_m3ps2, double periapsis_radius_m, double apoapsis_radius_m, double tolerance_m)`;
  `struct mission::PhasingOrbit { double periapsis_radius_m = 0.0; double apoapsis_radius_m = 0.0; };` (in
  `phasing_schedule.hpp`); `PhasingSchedule::above` (bool, the mirrored case).

- [ ] **Step 1: Failing tests for `next_raise_node`** (`[burn_node]`)

```cpp
// Refuelling f1: apsis burns from the insertion orbit to a phasing orbit.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"

using namespace starsim;

namespace {
constexpr double gm = constants::earth_gm_m3ps2;
const double re = constants::wgs84_semi_major_axis_m;
const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);

math::StateVector at_periapsis(double rp, double ra) {
    orbit::KeplerianElements e;
    e.semi_major_axis_m = 0.5 * (rp + ra);
    e.eccentricity = (ra - rp) / (ra + rp);
    e.inclination_rad = 26.5 * constants::deg_to_rad;
    return orbit::state_from_elements(e, gm);
}

// Flies the node impulsively and returns the state just after it.
math::StateVector fly(const math::StateVector& s, time::Epoch now, const gnc::BurnNode& node) {
    auto at = orbit::propagate_two_body(s, gm, node.epoch - now);
    at.velocity_mps = at.velocity_mps + node.delta_v_mps;
    return at;
}
}  // namespace

TEST_CASE("next_raise_node: below the depot one burn at apoapsis", "[burn_node]") {
    auto s = at_periapsis(re + 120e3, re + 190e3);
    const auto node = gnc::next_raise_node(s, t0, gm, re + 160e3, re + 190e3, 1e3);
    REQUIRE(node);
    s = fly(s, t0, *node);
    const auto el = orbit::elements_from_state(s, gm);
    CHECK(el.semi_major_axis_m * (1.0 - el.eccentricity) == Catch::Approx(re + 160e3).margin(100.0));
    CHECK(el.semi_major_axis_m * (1.0 + el.eccentricity) == Catch::Approx(re + 190e3).margin(100.0));
    CHECK_FALSE(gnc::next_raise_node(s, node->epoch, gm, re + 160e3, re + 190e3, 1e3));  // done
}

TEST_CASE("next_raise_node: above the depot two burns", "[burn_node]") {
    auto s = at_periapsis(re + 120e3, re + 190e3);
    time::Epoch now = t0;
    int burns = 0;
    while (const auto node = gnc::next_raise_node(s, now, gm, re + 250e3, re + 270e3, 1e3)) {
        s = fly(s, now, *node);
        now = node->epoch;
        REQUIRE(++burns <= 2);
    }
    CHECK(burns == 2);
    const auto el = orbit::elements_from_state(s, gm);
    CHECK(el.semi_major_axis_m * (1.0 - el.eccentricity) == Catch::Approx(re + 250e3).margin(1e3));
    CHECK(el.semi_major_axis_m * (1.0 + el.eccentricity) == Catch::Approx(re + 270e3).margin(1e3));
}

TEST_CASE("next_raise_node: nothing to do at or above the target; open orbit refused", "[burn_node]") {
    CHECK_FALSE(gnc::next_raise_node(at_periapsis(re + 200e3, re + 210e3), t0, gm, re + 170e3, re + 190e3, 1e3));
    orbit::KeplerianElements h;
    h.semi_major_axis_m = -2.0e7;
    h.eccentricity = 1.5;
    CHECK_FALSE(gnc::next_raise_node(orbit::state_from_elements(h, gm), t0, gm, re + 170e3, re + 190e3, 1e3));
}
```

- [ ] **Step 2: Failing tests for the mirrored schedule** (append to `test_phasing_schedule.cpp`, `[rendezvous][phasing]`)

```cpp
TEST_CASE("Schedule: a chaser above the target waits for the target to catch up, then descends",
          "[rendezvous][phasing]") {
    mission::PhasingParameters p;
    p.mission_limit_s = 48.0 * 3600.0;
    const auto target = circular(200e3, 1.0);
    for (const double behind_deg : {10.0, 40.0, 80.0}) {  // the target behind the chaser
        CAPTURE(behind_deg);
        const auto chaser = circular(260e3, 1.0 + behind_deg * deg);
        const auto s = mission::phasing_schedule(chaser, target, t0, p, gm);
        REQUIRE(s.feasible);
        CHECK(s.above);
        CHECK(s.phasing_revolutions == 0);
        CHECK(s.boost_epoch - t0 >= p.first_burn_lead_s);
        CHECK(s.waypoint0_epoch - t0 <= p.mission_limit_s);
    }
    // the further behind, the longer the wait (the target catches up at a constant rate)
    const auto near = mission::phasing_schedule(circular(260e3, 1.0 + 10.0 * deg), target, t0, p, gm);
    const auto far = mission::phasing_schedule(circular(260e3, 1.0 + 80.0 * deg), target, t0, p, gm);
    CHECK(far.boost_epoch - near.boost_epoch > 0.0);
}

TEST_CASE("Schedule: an above orbit whose periapsis crosses the target's is refused", "[rendezvous][phasing]") {
    orbit::KeplerianElements e;  // 195 x 300 km around a 200 km target: the periapsis dips below
    e.semi_major_axis_m = re + 247.5e3;
    e.eccentricity = 52.5e3 / (re + 247.5e3);
    e.inclination_rad = 51.64 * deg;
    e.raan_rad = 1.0;
    const auto s = mission::phasing_schedule(orbit::state_from_elements(e, gm), circular(200e3, 1.0), t0, {}, gm);
    CHECK_FALSE(s.feasible);
    CHECK(s.reason.rfind("Kein Rendezvous", 0) == 0);
}

TEST_CASE("Schedule: the catch-up rate matches dn = -1.5 n da / a", "[rendezvous][phasing]") {
    // waits of two leads 20 deg apart differ by 20 deg / |dn|
    mission::PhasingParameters p;
    p.mission_limit_s = 96.0 * 3600.0;
    const auto target = circular(200e3, 1.0);
    const auto a = mission::phasing_schedule(circular(170e3, 1.0 - 30.0 * deg), target, t0, p, gm);
    const auto b = mission::phasing_schedule(circular(170e3, 1.0 - 50.0 * deg), target, t0, p, gm);
    REQUIRE(a.feasible);
    REQUIRE(b.feasible);
    const double r = re + 200e3, n = std::sqrt(gm / (r * r * r));
    const double dn = 1.5 * n * 30e3 / r;  // rad/s, two-body (J2 changes it by < 1 %)
    CHECK(b.boost_epoch - a.boost_epoch == Catch::Approx(20.0 * deg / dn).epsilon(0.02));
}
```

Also change the existing `"Schedule refuses a chaser above the coelliptic height"`: the 414 km chaser around the
415 km ISS sits **between** the coelliptic height and the target - it stays refused (neither below `r_c` nor above
`r_i + coelliptic_height_m`); keep the case, rename it "a chaser between the coelliptic height and the target is
refused". In `test_phasing_planner.cpp` add a case that plans (`PhasingPlanner`) from a 260 km circular chaser 40 deg
ahead of a 200 km target with `mission_limit_s` 48 h and checks the Boost burn's delta-v points retrograde
(`dot(node.delta_v_mps, velocity) < 0`) and Waypoint 0 is reached within 100 m (copy the setup of the existing
"plans and flies" case in that file).

- [ ] **Step 3: Run** `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[burn_node],[phasing]"` - expected: compile error (`next_raise_node`, `above` missing).

- [ ] **Step 4: Implement `next_raise_node`** (`burn_node.cpp`, after `raise_periapsis_at_apoapsis`)

```cpp
std::optional<BurnNode> next_raise_node(const math::StateVector& state, time::Epoch now, double gm,
                                        double periapsis_radius_m, double apoapsis_radius_m, double tolerance_m) {
    const auto el = orbit::elements_from_state(state, gm);
    if (!(el.semi_major_axis_m > 0.0) || el.eccentricity >= 1.0) return std::nullopt;
    const double rp = el.semi_major_axis_m * (1.0 - el.eccentricity);
    const double ra = el.semi_major_axis_m * (1.0 + el.eccentricity);
    // 1. a higher apoapsis: burn at the current apoapsis so the opposite apsis becomes the target apoapsis
    if (apoapsis_radius_m > ra + tolerance_m) {
        const double dt = time_to_apoapsis_s(state, gm);  // existing helper used by set_periapsis_at_apoapsis
        const auto at = orbit::propagate_two_body(state, gm, dt);
        const double r = math::norm(at.position_m);
        const double v_new = std::sqrt(gm * (2.0 / r - 2.0 / (r + apoapsis_radius_m)));
        return BurnNode{now + dt, (v_new - math::norm(at.velocity_mps)) * math::normalized(at.velocity_mps)};
    }
    // 2. the periapsis (raised only; a higher one is left alone)
    if (rp < periapsis_radius_m - tolerance_m) return set_periapsis_at_apoapsis(state, now, gm, periapsis_radius_m);
    return std::nullopt;
}
```

If `set_periapsis_at_apoapsis` computes its time to apoapsis inline, factor that into an anonymous-namespace
`time_to_apoapsis_s(state, gm)` first and use it in both. Header comment: "Next apsis burn from `state` towards the
orbit periapsis x apoapsis (two-body): first the apoapsis (burn at the current apoapsis), then the periapsis (burn at
the new apoapsis); empty when both are within tolerance or above, or the orbit is open. Refuelling f1."

- [ ] **Step 5: Implement the mirrored schedule** (`phasing_schedule.cpp`)

Add `bool above = false;  // the chaser above the target: the target catches up, the boost descends (f1)` to
`PhasingSchedule`, and `PhasingOrbit` to the header. In `phasing_schedule`, replace the single apoapsis check by:

```cpp
    const double rp_d = r_d * (1.0 - ec.eccentricity), ra_d = r_d * (1.0 + ec.eccentricity);
    const bool below = ra_d < r_c;
    // [E] above: the periapsis keeps coelliptic_height_m above the target's orbit while waiting (f1)
    const bool above = rp_d > r_i + p.coelliptic_height_m;
    if (!below && !above) {
        s.reason = "Kein Rendezvous: Bahn kreuzt die Zielbahn";
        return s;
    }
    s.above = above;
```

and the catch-up of lap `m`:

```cpp
    // below: the chaser gains on the target (lead shrinks); above: the target gains (lead grows)
    const double closing = std::abs(n_d - n_i);
    ...
        double catch_up = above ? required_at_boost - lead : lead - required_at_boost;
        catch_up = std::fmod(catch_up + 4.0 * constants::two_pi, constants::two_pi) + constants::two_pi * m;
```

(`lead` is already in [0, 2 pi).) Keep the waiting branch as is; skip the phase-burn branch when `above` (it only
slows a chaser below). Remove the `if (catch_up < 0.0) continue;` (the wrap makes it non-negative) and verify the
existing below cases still pass - the old `lead + 2 pi m - required` with `continue` for negative values gives the
same first non-negative value.

- [ ] **Step 6: Run** the filter of Step 3 plus `[iss][launch_window]` and `[rendezvous]` (the ISS phasing must be
unchanged) - expected: all pass. Fix the planner case if the retrograde boost needs `solve_boost`'s secant start
`1.02 * dv0` to work for negative `dv0` (it does: both start points are retrograde).

- [ ] **Step 7: Commit**

```bash
git add core/include/starsim/gnc/burn_node.hpp core/src/gnc/burn_node.cpp core/include/starsim/mission/phasing_schedule.hpp core/src/mission/phasing_schedule.cpp core/tests/test_raise_to_orbit.cpp core/tests/CMakeLists.txt core/tests/test_phasing_schedule.cpp core/tests/test_phasing_planner.cpp
git commit -m "feat(mission): phasing from above the target and raise burns to a phasing orbit"
git push origin main
```

---

### Task 2: Launch window with candidate phasing orbits and cost

**Files:**
- Modify: `core/include/starsim/mission/iss_launch_window.hpp`, `core/src/mission/iss_launch_window.cpp`
- Test: `core/tests/test_iss_launch_window.cpp`

**Interfaces:**
- Consumes: `gnc::next_raise_node`, `mission::PhasingOrbit`, `PhasingSchedule::above` (Task 1).
- Produces:
  ```cpp
  struct PhasingCostModel {
      double mass_kg = 0.0;               // after the ascent
      double exhaust_velocity_mps = 0.0;  // the phasing engine set
      double boil_off_kgps = 0.0;
  };
  struct PhasingCost { double propellant_kg = 0.0, boil_off_kg = 0.0; double total_kg() const; };
  struct RaiseEstimate { bool feasible = false; math::StateVector state; double elapsed_s = 0.0, delta_v_mps = 0.0; };
  RaiseEstimate raise_estimate(const math::StateVector& separation, const PhasingOrbit& orbit, double gm_m3ps2);
  PhasingCost phasing_cost(const RaiseEstimate& raise, const PhasingSchedule& schedule, double separation_to_waypoint0_s,
                           const PhasingParameters& p, const math::StateVector& raised, const math::StateVector& target,
                           const PhasingCostModel& model, double gm_m3ps2);
  ```
  `LaunchWindowParameters::phasing_orbits` (`std::vector<PhasingOrbit>`, empty = the separation orbit),
  `LaunchWindowParameters::cost` (`PhasingCostModel`), `LaunchWindow::phasing_orbit` (`std::optional<PhasingOrbit>`),
  `LaunchWindow::cost` (`PhasingCost`), `LaunchWindow::raised_state_gcrf`, `LaunchWindow::raised` (epoch).

- [ ] **Step 1: Failing tests** (`[iss][launch_window]`; reuse the file's pad / `earth` / synthetic-station fixtures)

```cpp
TEST_CASE("Launch window: candidates - the cheapest feasible orbit, below or above the station", "[iss][launch_window]") {
    // a 200 km / 26.5 deg station, the pad at Starbase, insertion 120 x 190 km (the tanker's)
    auto p = tanker_like_parameters();  // helper in this file: inclination 26.5 deg, insertion_s 445.5, arc 12.2 deg,
                                        // separation 120 x 190 km, mission_limit 48 h - insertion, search 20 days
    p.phasing_orbits = {{re + 150e3, re + 190e3}, {re + 170e3, re + 190e3}, {re + 190e3, re + 190e3},
                        {re + 220e3, re + 240e3}, {re + 250e3, re + 270e3}};
    p.cost = {308e3, 3.4e3, 12e3 / 86400.0};
    int below = 0, above = 0;
    time::Epoch earliest = start;
    for (int i = 0; i < 12; ++i) {  // twelve consecutive windows
        const auto w = mission::plan_launch_window(pad, earth, station_200km, earliest, p, gm);
        REQUIRE(w.feasible);
        REQUIRE(w.phasing_orbit);
        (w.phasing_orbit->periapsis_radius_m > re + 200e3 ? above : below)++;
        // the choice is the cheapest of the feasible candidates at that crossing
        for (const auto& c : p.phasing_orbits) {
            auto single = p;
            single.phasing_orbits = {c};
            single.search_s = 600.0;  // only this crossing
            const auto other = mission::plan_launch_window(pad, earth, station_200km, w.liftoff - 300.0, single, gm);
            if (other.feasible && std::abs(other.liftoff - w.liftoff) < 60.0) {
                CHECK(w.cost.total_kg() <= other.cost.total_kg() + 1.0);
            }
        }
        earliest = w.liftoff + 3600.0;
    }
    CHECK(below > 0);
    CHECK(above > 0);
}

TEST_CASE("Launch window: empty candidates keep the separation orbit (ISS unchanged)", "[iss][launch_window]") {
    const auto p = iss_parameters();  // the existing ISS fixture of this file
    const auto w = mission::plan_launch_window(pad_ksc, earth, iss, start, p, gm);
    REQUIRE(w.feasible);
    CHECK_FALSE(w.phasing_orbit);
    CHECK(w.liftoff.tdb_seconds_past_j2000() == Catch::Approx(reference_liftoff_s).margin(1e-6));
}

TEST_CASE("Launch window: a raise that eats the mission limit is infeasible", "[iss][launch_window]") {
    auto p = tanker_like_parameters();
    p.phasing_orbits = {{re + 250e3, re + 270e3}};
    p.phasing.mission_limit_s = 2.0 * 3600.0;  // two burns half an orbit apart + checkout leave no phasing
    const auto w = mission::plan_launch_window(pad, earth, station_200km, start, p, gm);
    CHECK_FALSE(w.feasible);
    CHECK(w.reason.rfind("Kein Startfenster", 0) == 0);
}

TEST_CASE("Phasing cost against a hand computation", "[iss][launch_window]") {
    mission::RaiseEstimate raise;
    raise.feasible = true;
    raise.delta_v_mps = 20.0;
    // ... a schedule from a 170 km circular chaser to a 200 km target; boost dv = Hohmann 170 -> r_c (two burns)
    // expected propellant = m (1 - exp(-(20 + dv_boost) / ve)), boil-off = rate x (waypoint0 - separation)
}
```

For the "ISS unchanged" case take `reference_liftoff_s` by running the existing ISS window case once before the change
(record the value with full precision in the test, comment `// recorded before f1 (empty candidates must not move it)`).
Complete the hand computation case with concrete numbers: chaser 170 km circular 60 deg behind a 200 km target at
`t0`, `PhasingParameters` with 48 h, `PhasingCostModel{308e3, 3.4e3, 12e3 / 86400}`; dv_boost =
`hohmann_departure(r170, r_c) + hohmann_arrival(r170, r_c)` written out with vis-viva in the test.

- [ ] **Step 2: Run** `-TestFilter "[launch_window]"` - expected: compile errors.

- [ ] **Step 3: Implement**

`raise_estimate`: from the separation state loop `gnc::next_raise_node(state, t, gm, rp, ra, raise_tolerance_m)`
(`constexpr double raise_tolerance_m = 1e3;  // [E] f1 decision 7`) at most 3 times, propagating two-body to each node
and adding its delta-v; `elapsed_s` = last node epoch - separation; `delta_v_mps` = sum of |dv|. Not feasible when
a fourth node would be needed.

`phasing_cost`: boost delta-v = |departure| + |arrival| of the Hohmann transfer between the raised orbit's semi-major
axis and `r_c = r_target - coelliptic_height_m` (plus, when `phasing_revolutions > 0`, twice
`|hohmann_departure(r_d, phase_orbit_apoapsis_radius_m)|`); `propellant_kg = mass (1 - exp(-dv / ve))` with
dv = raise + boost; `boil_off_kg = boil_off_kgps * separation_to_waypoint0_s`. The later coelliptic / approach burns
are the same for every candidate and are left out (comment that).

`plan_launch_window`: factor the body of the bias iteration (from `const Vec3 pad = ...` to the re-aim) into a
local lambda `evaluate(time::Epoch lo, time::Epoch hi, std::optional<PhasingOrbit> orbit) -> LaunchWindow` that:
uses `a_chaser` = the candidate's semi-major axis (or the separation orbit's when empty) for `plane_bias_rad`;
computes the separation estimate on the separation orbit; with a candidate runs `raise_estimate`, then
`phasing_schedule(raised.state, station(separation + raised.elapsed_s), separation + raised.elapsed_s, phasing, gm)`
with `phasing.mission_limit_s -= raised.elapsed_s` (infeasible when <= 0); fills `phasing_orbit`, `cost`,
`raised_state_gcrf`, `raised`. At each crossing evaluate every candidate (or the empty one) and keep the feasible one
with the smallest `cost.total_kg()`; ties keep the earlier candidate in the list. With empty `phasing_orbits` the
code path must be the old one (same bias iterations, same epochs).

- [ ] **Step 4: Run** `-TestFilter "[launch_window],[iss_mission],[phasing]"` - expected: pass (the ISS missions use
empty candidates).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/mission/iss_launch_window.hpp core/src/mission/iss_launch_window.cpp core/tests/test_iss_launch_window.cpp
git commit -m "feat(mission): launch window picks the cheapest feasible phasing orbit per crossing"
git push origin main
```

---

### Task 3: The flown raise to the chosen orbit

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp` (`IssMissionSequence`), `core/src/sim/flight_session_iss_mission.cpp`
- Modify: `core/src/sim/refuelling_mission.cpp` (the `q.raise_*` assignment only; the window comes in Task 4)
- Test: `core/tests/test_refuelling_missions.cpp`

**Interfaces:**
- Consumes: `gnc::next_raise_node` (Task 1), `mission::PhasingOrbit`.
- Produces: `IssMissionSequence::raise_to` (`std::optional<mission::PhasingOrbit>`, replaces
  `raise_periapsis_radius_m`; comment "Ascent -> Raise (apsis burns on the deorbit set, f1) -> Rendezvous").

- [ ] **Step 1: Failing tests** (`[refuelling][iss_mission]`; set up like "Leg marks record the chaser's mass ..."
  in this file: tanker placed on 120 x 190 km at its periapsis, depot station added, `set_iss_mission` entering the
  Ascent leg's completion)

```cpp
TEST_CASE("Raise: two burns to an orbit above the depot, then the phasing plan", "[refuelling][iss_mission]") {
    auto f = raise_fixture(/*main_kg=*/100e3);  // helper: tanker 120 x 190 km, depot 200 km 40 deg behind it
    f.sequence.raise_to = mission::PhasingOrbit{re + 250e3, re + 270e3};
    f.start();
    f.session.advance_to(f.session.epoch() + 4.0 * 3600.0);
    CHECK(f.session.iss_mission_leg() == sim::IssMissionLeg::Rendezvous);
    const auto el = f.chaser_elements();
    CHECK(el.periapsis_m == Catch::Approx(re + 250e3).margin(3e3));
    CHECK(el.apoapsis_m == Catch::Approx(re + 270e3).margin(3e3));
}

TEST_CASE("Raise: a burn that runs dry ends the mission with its reason", "[refuelling][iss_mission]") {
    auto f = raise_fixture(/*main_kg=*/50.0);  // almost empty main tanks
    f.sequence.raise_to = mission::PhasingOrbit{re + 250e3, re + 270e3};
    f.start();
    f.session.advance_to(f.session.epoch() + 2.0 * 3600.0);
    CHECK(f.session.iss_mission_leg() == sim::IssMissionLeg::Ended);
    CHECK(f.session.iss_mission_status().find("kein Treibstoff") != std::string::npos);
}
```

Use the leg/status getters the existing tests in this file use (names may differ: `iss_leg()`, `iss_status()` -
take them from "Leg marks ..."). The fixture's RCS fallback must not fly the raise when the Raptor starves: if the
fallback takes over with cold gas, set the RCS supply to zero in the fixture for the dry case.

- [ ] **Step 2: Run** `-TestFilter "[refuelling][iss_mission]"` - expected: compile error (`raise_to`).

- [ ] **Step 3: Implement**

`iss_start_raise()`:

```cpp
void FlightSession::iss_start_raise() {
    const auto& s = bodies_[controlled_].body->state();
    const auto& to = *iss_mission_->raise_to;
    const auto node = gnc::next_raise_node({s.position_gcrf_m, s.velocity_gcrf_mps}, epoch_, gm_earth_,
                                           to.periapsis_radius_m, to.apoapsis_radius_m, raise_tolerance_m);
    if (!node) {  // at (or above) the phasing orbit
        ...unchanged: plan_rendezvous or iss_end
    }
    ...unchanged burn start; status "Phasing-Bahn anheben"
}
```

with `static constexpr double raise_tolerance_m = 1e3;  // [E] f1 decision 7` in `FlightSession`. In the leg switch,
`case IssMissionLeg::Raise:` becomes: lost -> end; `!burn_abort_reason_.empty()` -> `iss_end(burn_abort_reason_)`
(r5 minor: a dry Raise is no longer treated as flown); else `iss_start_raise()` (it plans the next burn or the
phasing). `case IssMissionLeg::Ascent:` tests `q.raise_to` instead of `q.raise_periapsis_radius_m`. In
`refuelling_mission.cpp` set `q.raise_to = mission::PhasingOrbit{R + phasing_periapsis_m, R + ascent.apoapsis_height_m}`
where the old assignment was (Task 4 replaces it by the window's orbit).

- [ ] **Step 4: Run** `-TestFilter "[refuelling][iss_mission],[refuelling][mission]"` (incl. `[slow]`: the r5 pad
flights still fly 170 x 190 with one burn) - expected: pass, reference numbers unchanged.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session_iss_mission.cpp core/src/sim/refuelling_mission.cpp core/tests/test_refuelling_missions.cpp
git commit -m "feat(sim): raise leg flies apsis burns to the phasing orbit; a dry raise ends the mission"
git push origin main
```

---

### Task 4: Mission config candidates and the flown window choice

**Files:**
- Modify: `core/include/starsim/config/refuelling_mission_config.hpp`, `core/src/config/mission_config.cpp`
- Modify: `config/missions/tanker_flight.toml`, `config/missions/hls_refuel.toml`
- Modify: `core/include/starsim/sim/refuelling_mission.hpp`, `core/src/sim/refuelling_mission.cpp`
- Modify: `core/src/sim/refuelling_campaign_setup.cpp` (move `exhaust_velocity_mps` out, see below)
- Test: `core/tests/test_refuelling_mission_config.cpp`, `core/tests/test_refuelling_missions.cpp`

**Interfaces:**
- Consumes: Task 2 window fields, Task 3 `raise_to`.
- Produces: `RefuellingFlightMission::phasing_candidates` (`std::vector<std::pair<double, double>>`, heights m),
  `planning_mass_kg`, `planning_boil_off_kgps`;
  `double sim::phasing_exhaust_velocity_mps(const vehicle::Vehicle&)` (moved from the setup's anonymous
  `exhaust_velocity_mps`, declared in `refuelling_mission.hpp`);
  `mission::LaunchWindowParameters sim::refuelling_window_parameters(const config::RefuellingFlightMission&, const config::AscentMission&, double exhaust_velocity_mps, bool reference_phasing = false)`.

- [ ] **Step 1: Config** (`tanker_flight.toml`, same block in `hls_refuel.toml`)

```toml
[phasing]
mission_limit_h = 48.0            # [E] uncrewed (the crew's 28 h opened the first window after 177 h, r3)
earliest_arrival_h = 0.0
periapsis_km = 170.0              # the reference phasing orbit (r5, --reference-phasing) after the raise at apoapsis
# [E] f1: phasing orbits (periapsis, apoapsis km) the window chooses from - below the depot the tanker catches up,
# above it the depot does; 150 km: lower bound for 48 h of drag
candidates_km = [[150.0, 190.0], [160.0, 190.0], [170.0, 190.0], [180.0, 190.0], [190.0, 190.0],
                 [220.0, 240.0], [250.0, 270.0]]
planning_mass_t = 308.0           # [D] r5: tanker dry 209 t + residue 99.2 t after the ascent (HLS: dry 120 + 208)
planning_boil_off_t_per_day = 12.0  # [D] r6: the bare tanker's boil-off with its residue (HLS: insulated, 1.0 [D])
```

For `hls_refuel.toml` use `planning_mass_t = 328.0` and take the boil-off from the r5 HLS numbers in
`docs/subsystems.md` (r5 section) - if none is recorded, compute it once with `vehicle::orbit_average_boil_off` in a
throwaway probe and enter it with [D] and the probe's date.

- [ ] **Step 2: Failing tests** - config parses the candidates (count 7, the first (150, 190) km in metres, planning
values in SI); a candidate with periapsis > apoapsis or below 100 km is refused with a German message;
`refuelling_window_parameters` builds 7 orbits with the insertion as the separation orbit (120 x 190 km) and
`reference_phasing = true` builds the single (170, 190) one. Then in `test_refuelling_missions.cpp`:

```cpp
TEST_CASE("tanker_flight: an above-depot window is flown to docking", "[refuelling][mission][slow]") {
    // find the first earliest-day whose window chooses an orbit above the depot (analytic, cheap)
    auto m = load_tanker_flight();
    const auto w = first_window_where(m, [](const mission::LaunchWindow& w) {
        return w.phasing_orbit->periapsis_radius_m > re + 200e3;
    });  // helper: plan_launch_window day by day from m.earliest_utc
    m.earliest_utc = utc_of(w.liftoff - 3600.0);
    auto session = make_session();
    const auto s = sim::start_refuelling_flight_mission(session, config_dir, m, time);
    REQUIRE(s.refusal.empty());
    REQUIRE(s.launch_window->phasing_orbit->periapsis_radius_m > re + 200e3);
    run_until_leg(session, sim::IssMissionLeg::Docking, /*limit_s=*/60.0 * 3600.0);
    CHECK(session.iss_mission_leg() == sim::IssMissionLeg::Docking);
}
```

(Helpers as in the r5 tanker_flight pad test of this file; stop at Docking to keep it ~2-3 min.)

- [ ] **Step 3: Run** `-TestFilter "[refuelling][config],[refuelling][mission]"` - expected: failures.

- [ ] **Step 4: Implement** - parse `candidates_km` (array of 2-element arrays, `doc` helpers as for the other
arrays in `mission_config.cpp`), `planning_mass_t`, `planning_boil_off_t_per_day` (required when `candidates_km`
is present). `refuelling_window_parameters`: separation = the ascent's insertion orbit (`ascent.periapsis_height_m`
x `ascent.apoapsis_height_m`); `phasing_orbits` from the candidates (or the single `periapsis_km` orbit when
`reference_phasing` or no candidates); `cost = {planning_mass_kg, exhaust_velocity_mps, planning_boil_off_kgps}`.
`start_refuelling_flight_mission`: build the chaser vehicle once for `phasing_exhaust_velocity_mps`, then
`q.raise_to = *w.phasing_orbit`. `MissionStart::launch_window` already carries the window.

- [ ] **Step 5: Run** the filter of Step 3 incl. `[slow]` (background, output to file) - the r5 pad flights now pick
their own orbit: record their new numbers (the reference checks against `reference_flights.toml` may fail - that
is Task 5's re-measurement; note the measured values in the ledger).

- [ ] **Step 6: Commit**

```bash
git add config/missions/tanker_flight.toml config/missions/hls_refuel.toml core/include/starsim/config/refuelling_mission_config.hpp core/src/config/mission_config.cpp core/include/starsim/sim/refuelling_mission.hpp core/src/sim/refuelling_mission.cpp core/src/sim/refuelling_campaign_setup.cpp core/tests/test_refuelling_mission_config.cpp core/tests/test_refuelling_missions.cpp
git commit -m "feat(sim): refuelling flights choose their phasing orbit per window"
git push origin main
```

---

### Task 5: Calculator, lever, re-measurement

**Files:**
- Modify: `core/include/starsim/mission/refuelling_campaign.hpp`, `core/src/mission/refuelling_campaign.cpp`
- Modify: `core/include/starsim/sim/refuelling_campaign_setup.hpp`, `core/src/sim/refuelling_campaign_setup.cpp`
- Modify: `core/src/sim/refuelling_campaign_text.cpp`, `core/apps/refuelling_report.cpp` (usage line), `bindings/src/flight_sim.cpp`, `bindings/src/flight_sim.hpp` (key list comment)
- Modify: `config/refuelling/reference_flights.toml` (re-measured)
- Test: `core/tests/test_refuelling_campaign.cpp`

**Interfaces:**
- Consumes: Tasks 2 and 4.
- Produces: `CampaignFlight::extra_demand_kg` (HLS: more than its reference window's phasing propellant),
  `CampaignLevers::reference_phasing` (bool), CLI flag `--reference-phasing`, game key `reference_phasing` (bool),
  `CampaignReport::tankers` rows carry the chosen orbit: add `std::vector<mission::PhasingOrbit> tanker_orbits` to
  `CampaignReport` (same order as `result.tankers`), printed by the report text as "Phasing 150 x 190 km".

- [ ] **Step 1: Failing tests**

```cpp
TEST_CASE("Campaign: the HLS's extra demand must be in the depot at its arrival", "[refuelling][campaign]") {
    // hand models as in "Campaign against a hand computation": demand 100 t, the HLS flight asks 5 t more
    // -> the HLS is accepted only once the depot holds 100 + 5 x (1 + vent) at its arrival
}

TEST_CASE("Reference phasing reproduces the single-orbit campaign", "[refuelling][campaign][slow]") {
    sim::CampaignLevers ref;
    ref.reference_phasing = true;
    const auto r = sim::plan_refuelling_campaign(sources(), ref);
    REQUIRE(r.result.closes);
    for (const auto& o : r.tanker_orbits) CHECK(o.periapsis_radius_m == Catch::Approx(re + 170e3).margin(1.0));
}

TEST_CASE("Per-window phasing: more arrivals, never more tankers or days", "[refuelling][campaign][slow]") {
    sim::CampaignLevers ref;
    ref.reference_phasing = true;
    const auto single = sim::plan_refuelling_campaign(sources(), ref);
    const auto chosen = sim::plan_refuelling_campaign(sources(), {});
    REQUIRE(single.result.closes);
    REQUIRE(chosen.result.closes);
    CHECK(chosen.result.duration_s < single.result.duration_s);
    CHECK(chosen.result.tanker_flights <= single.result.tanker_flights + 1);  // more, shorter-phased tankers may
                                                                            // deliver less each; days must drop
    int above = 0;
    for (const auto& o : chosen.tanker_orbits) above += o.periapsis_radius_m > re + 200e3;
    CHECK(above > 0);
}
```

Fill the first case with concrete hand numbers in the style of the existing hand-computation case. Extend the CLI
parser test with `--reference-phasing` (flag without a value) and the text test with the "Phasing" column.

- [ ] **Step 2: Run** `-TestFilter "[refuelling][campaign]"` - expected: compile errors.

- [ ] **Step 3: Implement**
- `refuelling_campaign`: at the HLS check the required fill is `demand_kg + hls->extra_demand_kg * (1 + vent)`.
- Setup: the window parameters lambda passes `phasing_exhaust_velocity_mps(tanker)` and `levers.reference_phasing`;
  the reference window (unshifted, same candidates) gives `reference_phasing_s` and the reference `cost`. In
  `next_tanker`: delivery = `base_kg + rate * (reference_phasing_s - phasing_s) - (w.cost.propellant_kg -
  reference_cost.propellant_kg)`; push `*w.phasing_orbit` to `report.tanker_orbits` (the models run in campaign
  order; clear the vector if the calculator re-asks a window - check `next_slot` in `refuelling_campaign.cpp`: if it
  can call `next_tanker` for a window it then rejects, record the orbit in `CampaignFlight` instead
  (`std::optional<std::pair<double,double>> phasing_orbit_m` field) and copy from `result.tankers` afterwards).
  `next_hls`: `extra_demand_kg = max(0, w.cost.propellant_kg - hls_reference_cost.propellant_kg)`.
- CLI flag and game key; `campaign_lever_error` unchanged.

- [ ] **Step 4: Re-measure** - run `-TestFilter "[refuelling][mission]"` incl. `[slow]` in the background (output to
a file, check every 10 min). Update `config/refuelling/reference_flights.toml` with the new measured values (keep
the [D] tags, date 2026-10-21, note "f1: window with candidate orbits, raise modelled"). Re-run until
`check_reference` passes.

- [ ] **Step 5: Run** `-TestFilter "[refuelling][campaign]"` incl. `[slow]` and record the new answer (tankers, days,
orbits chosen, deliveries) and the `--reference-phasing` answer in the ledger. Also `build/core/refuelling_report.exe`
and `--reference-phasing` once each (runtime noted: the candidates multiply the window searches).

- [ ] **Step 6: Commit**

```bash
git add core/include/starsim/mission/refuelling_campaign.hpp core/src/mission/refuelling_campaign.cpp core/include/starsim/sim/refuelling_campaign_setup.hpp core/src/sim/refuelling_campaign_setup.cpp core/src/sim/refuelling_campaign_text.cpp core/apps/refuelling_report.cpp bindings/src/flight_sim.cpp bindings/src/flight_sim.hpp config/refuelling/reference_flights.toml core/tests/test_refuelling_campaign.cpp
git commit -m "feat(sim): campaign books the per-window phasing orbit; reference flights re-measured"
git push origin main
```

---

### Task 6: Docs, smoke test, final review

**Files:**
- Modify: `docs/subsystems.md` (r3 "Phasing", r5 "Raise", r6 "Campaign calculator": cadence finding replaced by the
  f1 result; new "Phasing orbit per window (f1)" subsection), `CLAUDE.md` (module summary: "phasing from above the
  target, phasing orbit chosen per window"), `config/README.md` if it lists mission keys.

- [ ] **Step 1:** Write the docs with the measured numbers (new answer vs `--reference-phasing`, orbits chosen,
  arrivals per ~13.8 d cycle, runtime of the CLI).
- [ ] **Step 2:** Build with bindings (`tools/build.ps1 -TestFilter "[refuelling]"`) and run the Godot smoke test.
- [ ] **Step 3:** Commit `docs: refuelling f1 phasing orbit per window`, push.
- [ ] **Step 4:** Final review of the whole f1 diff (one reviewer, most capable model); fix pass; ledger
  `.superpowers/sdd/2026-10-21-refuelling-f1-phasing-per-window/progress.md` with numbers, decisions, deferred
  minors; memory update; ask the user for `/clear`, next: writing-plans f3 (settling).
