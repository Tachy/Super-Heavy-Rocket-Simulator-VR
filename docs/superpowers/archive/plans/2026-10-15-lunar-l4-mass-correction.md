# Lunar l4 Mass Correction (real HLS masses at the Moon) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-base the l4 landing and ascent on the HLS's real masses at the Moon (~570 t in the LLO, ~500-570 t at PDI,
~260-340 t landed) instead of the wrong ~1,185 t, so that mission 1 (l4 Task 9) can land and ascend.

**Architecture:** An amendment inserted into the l4 plan before its Task 9 (`docs/superpowers/plans/2026-10-15-lunar-l4-
landing-ascent.md`); executed under the l4 ledger (`.superpowers/sdd/2026-10-15-lunar-l4-landing-ascent/progress.md`,
tasks named C1-C4), reviewed with l4 in its final review. Config (landing thrusters), the descent guidance's sets and
PDI range (gnc, mission, sim), and the l4 test scenarios change; no new module.

**Tech Stack:** C++20 core (Catch2), TOML vehicle config, existing SCvx / Apollo braking / PEG guidance.

**Spec:** `docs/superpowers/specs/2026-10-14-lunar-missions-design.md` (l4 section; this plan corrects its premise).

## Why (the finding, l4 Task 9, 2026-10-15)

The l4 plan and spec state "~1,185 t in the LLO (l3: 1,177-1,194 t)". That l3 number is the campaign's **propellant
used** (`core/tests/test_lunar_outbound_session.cpp:188`: `propellant_t = (m0 - mass) / 1000`), not the mass in the
LLO. Rocket equation and the mission flight agree: 1,720 t -> TLI 3,142 m/s at 380 s -> ~732 t -> LOI 856 m/s ->
**~573 t in the LLO** (measured). NRHO variant: ~547-586 t in the LLO after the NRHO -> LLO transfer. Landing ~300-340 t
(LLO) / ~260-300 t (NRHO). Consequences measured in Task 9:

- The landing thrusters (12 x 140 kN, minimum 40 % = 0.65 MN axial) cannot hover below ~400 t (2.2 m/s^2 at 300 t
  against the lunar 1.62 m/s^2); the single sea-level Raptor (minimum 874 kN) not below ~540 t.
- Powered descent from PDI at 550 t (test `[lunar_descent_light]`, RED): the braking demand is below the vacuum set's
  minimum at once and below the single Raptor's minimum from t_go ~410 s; the law over-brakes, its demand falls to ~0
  and the attitude loop chases a vanishing thrust vector (axis error 100-150 deg) - both mission flights crashed
  (57 / 73 m/s).
- Already fixed in Task 9 (uncommitted, ledger Ruling "finding A"): trims and DOI flew on cold gas (~15 t each);
  `lunar_small_burn` now flies them on the single sea-level Raptor.

## Global Constraints

- Physics first: masses come from the rocket equation / the flight, never set (user 2026-10-14).
- Braking on the vacuum Raptors, then the gimballed sea_level_1 (user 2026-10-15) - kept; this plan adds the landing
  thrusters as the last braking set (the single Raptor cannot throttle low enough at ~350 t).
- Terminal descent on the landing thrusters as Apollo P66 with differential throttling (user 2026-10-15) - kept.
- Ascent on the vacuum Raptors, steering by the landing thrusters until aligned (l4 Task 8) - kept.
- Every [E] value with its reason in the comment; SI units; German UI strings; no magic numbers in code.
- Test workflow (CLAUDE.md): touched tags during a task, `-Fast` before every push, the full suite at the end of C4
  (guidance and vehicles change). Tag a case `[slow]` when it takes more than ~40 s.

## Reference masses (all tasks use these; `core/tests/lunar_test_support.hpp`)

| Scenario | Propellant (120 t dry) | Total | Source |
|---|---|---|---|
| LLO, after the LOI (LLO variant) | 453e3 | 573 t | [D] mission flight, Task 9 |
| PDI, LLO variant | 446e3 | 566 t | [D] - trim + DOI ~7 t (Task 9 `[burn_engines]`) |
| PDI, light (NRHO variant) | 380e3 | 500 t | [D] ~547-586 t after the transfer, more trims |
| Landed, heavy | 220e3 | 340 t | [D] 566 t after ~1.8-2.0 km/s at 350-380 s |
| Landed, light | 140e3 | 260 t | [D] NRHO variant after a 2.2 km/s descent |

## Review Focus

1. PDI mass between the reference cases (520-540 t) and a 5 % heavier dry mass: the computed braking range must still
   give a landing - C2's sweep covers 500 and 566 t; the reviewer checks the range function is monotone in mass.
2. The set drop vacuum -> sea_level_1 -> landing thrusters at a demand between two sets' ranges (a gap): the guidance must
   pick the set that can deliver it, never hold a set below its minimum for long.
3. Engine-out in the braking phase at the real mass (the engine-out set of three sea-level Raptors has a 2.6 MN minimum):
   untested since l4 Task 6 - C2 adds it at 566 t.
4. Ascent at ~260 t: the vacuum set's minimum (2.68 MN) exceeds the 1 g limit; the throttle rule must stay at the
   minimum without oscillating (C4 records the peak acceleration).
5. Legs at 260 t: the l4 x3 crush force (4.5 MN, sized for 662 t) must not bounce a light HLS (C1 drop tests).

---

### Task C1: Landing thrusters and test masses for the real HLS mass

**Files:**
- Modify: `config/engines/hls_landing_thruster.toml`, `config/vehicles/starship_hls.toml` (comments, thruster comment
  block, leg comments), `core/tests/test_hls_vehicle.cpp:45-64`, `core/tests/test_hls_landing.cpp:169-181`,
  `core/tests/lunar_test_support.hpp`
- Modify (premise note only): `docs/superpowers/specs/2026-10-14-lunar-missions-design.md`,
  `docs/superpowers/plans/2026-10-15-lunar-l4-landing-ascent.md`

**Interfaces:**
- Produces (`core/tests/lunar_test_support.hpp`, namespace `starsim::test`):

```cpp
// Real HLS masses at the Moon (l4 mass correction, [D]: 1,720 t in LEO, TLI + LOI on the vacuum Raptors).
inline constexpr double hls_llo_propellant_kg = 453e3;          // 573 t in the LLO after the LOI
inline constexpr double hls_pdi_propellant_kg = 446e3;          // 566 t at PDI (LLO variant)
inline constexpr double hls_pdi_light_propellant_kg = 380e3;    // 500 t at PDI (NRHO variant)
inline constexpr double hls_landed_propellant_kg = 220e3;       // 340 t landed (LLO variant)
inline constexpr double hls_landed_light_propellant_kg = 140e3; // 260 t landed (NRHO variant)
```

- Thrusters: 12 x 90 kN [E], throttle 0.35-1.0 [E]: axial max 12 x 90 x cos 15 deg = 1.043 MN (1.9 lunar weights of
  340 t; >= sea_level_1's minimum 874 kN / 0.9 so the braking can drop onto them), minimum 0.365 MN (hovers down to
  225 t, below the lightest landing 260 t - 5 %).

- [ ] **Step 1: Write the failing tests.** Replace the body of "HLS landing thrusters lift ~1.5 lunar weights and can
  hover" (rename it "HLS landing thrusters take over from one Raptor and hover the real landing masses"):

```cpp
    std::printf("[hls] landing thrust %.0f-%.0f kN axial: hovers %.0f-%.0f t\n", min_axial / 1e3, max_axial / 1e3,
                min_axial / lunar_g_mps2 / 1e3, max_axial / lunar_g_mps2 / 1e3);
    // l4 mass correction: landing 260-340 t (not ~660 t); the braking drops from one sea-level Raptor (minimum 874 kN)
    // onto them, so their maximum must exceed that minimum / set_drop_fraction (0.9).
    CHECK(n == 12);
    double raptor_min = 0.0;
    for (const auto& e : v.stages().front().engines) {
        if (e.group_id == "hls/sea_level_1") raptor_min = e.performance->thrust_n(e.performance->config().throttle_min, 0.0);
    }
    CHECK(max_axial > raptor_min / 0.9);
    CHECK(min_axial / lunar_g_mps2 < 0.95 * 260e3);  // hovers the lightest landing, -5 %
    CHECK(max_axial / lunar_g_mps2 > 1.5 * 340e3);   // lifts the heaviest with 1.5 lunar weights
```

  Change "HLS drop tests at the l4 landing masses" to the real masses (rename "(260 t and 340 t)"):

```cpp
    for (const double propellant_kg : {test::hls_landed_light_propellant_kg, test::hls_landed_propellant_kg}) {
```

  (include `lunar_test_support.hpp`). Add the constants above to `lunar_test_support.hpp`.

- [ ] **Step 2: Run** `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[hls]"`.
  Expected: the thruster case FAILS (`min_axial / g < 247 t` false: 0.65 MN hovers ~400 t); drop tests may pass.

- [ ] **Step 3: Implement.** `hls_landing_thruster.toml`: `thrust_ref_n = 90.0e3`, `[throttle] min = 0.35`, header and
  value comments rewritten with the derivation above (and the l4 mass correction as the reason; drop the 662/503 t
  text). `starship_hls.toml`: the thruster block comment and the leg comment ("x3 (662 / 221 t)") restated for 260-340 t
  - keep `crush_force_n = 4.5e6` unless the light drop test bounces or tips (then rule and ledger). Run
  `python tools/vehicle_report.py starship_hls` and keep its output in the ledger line.

- [ ] **Step 4: Run** the `[hls]` tag again. Expected: all pass. If a drop test at 260 t fails, systematic-debugging
  (leg stiffness vs mass), ledger the ruling.

- [ ] **Step 5: Premise notes.** In the spec's l4 section and in the l4 plan's "User decisions for l4" first bullet add
  one paragraph: "Correction 2026-10-15 (Task 9): ~1,185 t was l3's propellant used, not the LLO mass; real ~573 t in the
  LLO, 260-340 t landed - see `2026-10-15-lunar-l4-mass-correction.md`." Do not rewrite the old numbers elsewhere.

- [ ] **Step 6: Commit** (no push yet: the l4 Task 9 work in the tree is uncommitted and red - push after C4).

```bash
git add config/engines/hls_landing_thruster.toml config/vehicles/starship_hls.toml core/tests/test_hls_vehicle.cpp core/tests/test_hls_landing.cpp core/tests/lunar_test_support.hpp docs/superpowers/specs/2026-10-14-lunar-missions-design.md docs/superpowers/plans/2026-10-15-lunar-l4-landing-ascent.md
git commit -m "fix(vehicle): size the HLS landing thrusters for the real landing mass (12 x 90 kN)"
```

---

### Task C2: Powered descent at the real PDI mass

**Files:**
- Modify: `core/include/starsim/gnc/lunar_landing_guidance.hpp`, `core/src/gnc/lunar_landing_guidance.cpp`
  (`hls_landing_parameters`, the set drop to a third set), `core/include/starsim/mission/lunar_descent.hpp`,
  `core/src/mission/lunar_descent.cpp` (braking range function), `core/include/starsim/sim/flight_session.hpp`
  (`LunarLandingSetup::braking_range`), `core/src/sim/flight_session_lunar_descent.cpp` (`plan_lunar_landing` sets the
  range), `core/tests/test_lunar_powered_descent.cpp`,
  `core/tests/test_lunar_scvx.cpp`, `core/tests/test_lunar_descent.cpp`

**Interfaces:**
- Consumes: C1's constants and thrusters.
- Produces:

```cpp
// mission/lunar_descent.hpp
// Ground range of the braking phase for a lander at PDI: v^2 / (2 a) with a = thrust_fraction x thrust_n / mass_kg,
// times range_factor (the Apollo law's thrust falls over the phase; the factor is fitted in C2's sweep), clamped to
// [min_range_m, max_range_m].
struct BrakingRangeParameters {
    double thrust_fraction = 0.45;   // [E] of the first braking set's maximum (C2 sweep)
    double range_factor = 1.0;       // [E] (C2 sweep)
    double min_range_m = 150.0e3;    // [E]
    double max_range_m = 600.0e3;    // [E]
};
double braking_range_m(double pdi_speed_mps, double mass_kg, double thrust_n, const BrakingRangeParameters& p);

// gnc::LunarLandingParameters (hls_landing_parameters)
//   braking_sets {{"hls/vacuum"}, {"hls/sea_level_1"}, {"hls/landing"}}   // was two sets
//   approach.snapshot.phase_groups: {"hls/landing"} in all three phases     // was sl1, sl1, landing
```

- [ ] **Step 1: Write the failing tests.** In `test_lunar_powered_descent.cpp` replace the light test added in Task 9
  ("At the mission's PDI mass (~550 t) ...") and the 1,185 t landing test by one helper and two cases:

```cpp
namespace {
struct DescentResult { sim::LegTouchdown outcome; double offset_m, sink_mps, m0, m1, seconds; };
DescentResult fly_from_pdi(Fixture& fx, double propellant_kg) {
    const double v_pdi = 1690.0;  // [D] periselene speed of the ~20 x 100 km descent orbit (place_hls_at_pdi)
    const mission::BrakingRangeParameters params;  // the defaults Step 3 tunes
    const double m = 120e3 + propellant_kg;
    fx.place_at_pdi(propellant_kg, mission::braking_range_m(v_pdi, m, fx.vacuum_set_max_thrust_n(), params));
    const double m0 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    REQUIRE(fx.session.begin_powered_descent(fx.point, fx.azimuth));
    for (int i = 0; i < 1200 * 10 && fx.session.lunar_step() == sim::LunarStep::PoweredDescent; ++i) {
        fx.session.advance_to(fx.session.epoch() + 0.1, 100);
    }
    fx.session.advance_to(fx.session.epoch() + 5.0, 2000);
    const auto& td = fx.session.lunar_touchdown();
    REQUIRE(td);
    return {td->outcome, td->offset_m, td->sink_mps, m0, fx.session.body(0).vehicle().mass_properties().mass_kg,
            fx.session.epoch() - fx.t0};
}
}  // namespace

TEST_CASE("The HLS brakes from PDI at the real mass and lands within 1 m", "[sim][lunar_descent_flight][terrain][data][slow]") {
    for (const double propellant : {test::hls_pdi_propellant_kg, test::hls_pdi_light_propellant_kg}) {
        Fixture fx;
        const auto r = fly_from_pdi(fx, propellant);
        // equivalent delta-v at the braking set's Isp mix: report both bounds
        const double dv350 = 350.0 * constants::standard_gravity_mps2 * std::log(r.m0 / r.m1);
        std::printf("[lunar_descent] %.0f t at PDI: %s, offset %.2f m, sink %.2f m/s, %.0f s, %.0f -> %.0f t "
                    "(%.0f m/s at 350 s)\n", r.m0 / 1000.0, sim::to_string(r.outcome), r.offset_m, r.sink_mps, r.seconds,
                    r.m0 / 1000.0, r.m1 / 1000.0, dv350);
        CAPTURE(propellant);
        CHECK(r.outcome == sim::LegTouchdown::Landed);
        CHECK(r.offset_m < 1.0);
        CHECK(r.sink_mps <= 1.1);
        CHECK(dv350 < 2400.0);  // spec ~2.0 km/s incl. the approach; the crash run burnt ~3.5 km/s
    }
}

TEST_CASE("A vacuum-Raptor failure early in the braking phase at the real mass: the sea-level Raptors land the HLS",
          "[sim][lunar_descent_flight][terrain][data][slow]") {
    // Review focus 3 (untested since l4 Task 6): fail one RVac 30 s after PDI.
    Fixture fx;
    const double m = 120e3 + test::hls_pdi_propellant_kg;
    fx.place_at_pdi(test::hls_pdi_propellant_kg,
                    mission::braking_range_m(1690.0, m, fx.vacuum_set_max_thrust_n(), mission::BrakingRangeParameters{}));
    const double m0 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    REQUIRE(fx.session.begin_powered_descent(fx.point, fx.azimuth));
    fx.session.advance_to(fx.t0 + 30.0, 10'000);
    const auto& engines = fx.session.body(0).vehicle().stages().front().engines;
    size_t rvac = engines.size();
    for (size_t i = 0; i < engines.size(); ++i) {
        if (engines[i].group_id == "hls/vacuum" && engines[i].producing_thrust()) rvac = std::min(rvac, i);
    }
    REQUIRE(rvac < engines.size());
    fx.session.fail_engine(0, 0, rvac);   // as test_lunar_ascent.cpp's engine-out case
    for (int i = 0; i < 1200 * 10 && fx.session.lunar_step() == sim::LunarStep::PoweredDescent; ++i) {
        fx.session.advance_to(fx.session.epoch() + 0.1, 100);
    }
    fx.session.advance_to(fx.session.epoch() + 5.0, 2000);
    const auto& td = fx.session.lunar_touchdown();
    REQUIRE(td);
    const double m1 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    const double dv350 = 350.0 * constants::standard_gravity_mps2 * std::log(m0 / m1);
    std::printf("[lunar_descent] engine out 30 s after PDI: %s, offset %.2f m, sink %.2f m/s, %.0f m/s at 350 s\n",
                sim::to_string(td->outcome), td->offset_m, td->sink_mps, dv350);
    CHECK(td->outcome == sim::LegTouchdown::Landed);
    CHECK(td->offset_m < 1.0);
    CHECK(dv350 < 2600.0);
}
```

  In `fly_from_pdi` use `mission::BrakingRangeParameters{}` for `params` (the defaults Step 3 tunes). Add
  `Fixture::vacuum_set_max_thrust_n()`: `gnc::set_max_thrust_n(session.body(0).vehicle(), {"hls/vacuum"})` after the
  vehicle is placed - so call it after `place_at_pdi` with a provisional range, or compute it from a
  `vehicle::Vehicle(test::hls_with_propellant(...), STARSIM_CONFIG_DIR)` built in the fixture (preferred: no
  re-placement).

  In `test_lunar_scvx.cpp` set `high_gate_body`'s propellant to the gate mass (~350 t: `propellant = 230e3`) and the
  phase groups / settings of "SCvx plans a lunar approach ..." to `hls_landing_parameters().approach` (landing thrusters
  in all phases); keep its `REQUIRE(result.converged)` and touchdown checks. In `test_lunar_descent.cpp` add:

```cpp
TEST_CASE("The braking range grows with the PDI mass and stays within its limits", "[mission][lunar_descent]") {
    const mission::BrakingRangeParameters p;
    const double t = 8.94e6;  // three RVacs
    const double r500 = mission::braking_range_m(1690.0, 500e3, t, p), r566 = mission::braking_range_m(1690.0, 566e3, t, p);
    CHECK(r566 > r500);
    CHECK(r500 >= p.min_range_m);
    CHECK(mission::braking_range_m(1690.0, 5e6, t, p) == p.max_range_m);
    CHECK(mission::braking_range_m(1690.0, 1e3, t, p) == p.min_range_m);
}
```

- [ ] **Step 2: Run** `-TestFilter "[lunar_descent],[lunar_scvx]"` (compile errors expected), then the flights by name
  (`build/core/tests/starsim_core_tests.exe "The HLS brakes from PDI at the real mass*" -s`, in the background, with
  `STARSIM_DEBUG_LUNAR_DESCENT=1` to a log in the workspace). Expected: FAIL (crash, as Task 9's light run).

- [ ] **Step 3: Implement.**
  - `braking_range_m` in `lunar_descent.cpp` as specified (`std::clamp(range_factor * v*v / (2 a), min, max)`).
  - `sim::LunarLandingSetup` gains `std::optional<mission::BrakingRangeParameters> braking_range =
    mission::BrakingRangeParameters{};` (empty: `descent.braking_range_m` as configured). `plan_lunar_landing` sets
    `descent.braking_range_m` from it before planning: PDI speed = periselene speed of the descent orbit
    (vis-viva with the LLO's aposelene and the planned periselene `site height + pdi_altitude_above_site_m`), mass = the
    current mass, thrust = the first braking set's maximum (`set_max_thrust_n` - move it from the anonymous namespace
    of `lunar_landing_guidance.cpp` to a declared `gnc::set_max_thrust_n(const vehicle::Vehicle&, const
    std::vector<std::string>&)` in `lunar_landing_guidance.hpp`). Debug print the range with the descent plan line.
  - `hls_landing_parameters`: `braking_sets` three sets as above; approach phase groups `{"hls/landing"}` x 3; revisit
    `kinematic_guess_s`, `tilt_max_rad` and the switch limits for the landing thrusters (they are steered by
    differential throttling, equivalent gimbal 9.4 deg: `final_gimbal_fraction` stays 1). The drop to the third set
    uses the existing rule (`set_drop_fraction` of the NEXT set's maximum) - landing thrusters are lit ahead as any
    disjoint set.
  - `braking()` must never hold a set below its minimum while a smaller set exists: if the demand is below the running
    set's minimum thrust for `set_drop_s`, drop even when it is above `set_drop_fraction` x the next set's maximum
    (review focus 2) - add that as a second drop condition with a unit-free debug line.
  - Tune by a sweep, not by guessing: run the real-mass case for `thrust_fraction` in {0.35, 0.45, 0.55} and
    `range_factor` in {0.8, 1.0, 1.2} (9 runs, background, one log each); pick the pair that lands both masses with the
    lowest `dv350`, set it as the default, ledger the table. If none lands, the high gate / `approach_time_to_go_s`
    are next (they were tuned for 700 t on one Raptor) - one variable at a time, ledgered.

- [ ] **Step 4: Run** `-TestFilter "[lunar_descent],[lunar_scvx],[differential_throttle]"` and both slow flights by
  name. Expected: all pass; record per mass: braking s, set drops (t_go), landing offset/sink, masses, dv350.

- [ ] **Step 5: Commit** (no push).

```bash
git add core/include/starsim/gnc/lunar_landing_guidance.hpp core/src/gnc/lunar_landing_guidance.cpp core/include/starsim/mission/lunar_descent.hpp core/src/mission/lunar_descent.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session_lunar_descent.cpp core/tests/test_lunar_powered_descent.cpp core/tests/test_lunar_scvx.cpp core/tests/test_lunar_descent.cpp
git commit -m "fix(gnc): lunar powered descent at the real HLS mass (landing thrusters as the last braking set, braking range from the mass)"
```

---

### Task C3: Descent autopilot from the LLO at the real mass, budget values from the flight

**Files:**
- Modify: `core/tests/test_lunar_descent_session.cpp`, `core/tests/test_lunar_descent.cpp:55-62` (the budget unit test's
  1,185 t), `core/include/starsim/sim/flight_session.hpp` (`LunarLandingSetup` budget defaults),
  `config/missions/hls_lunar_llo.toml`, `config/missions/hls_lunar_nrho.toml` (`[budget]` descent value)

**Interfaces:**
- Consumes: C2 (`LunarLandingSetup::braking_range`), C1 constants.

- [ ] **Step 1: Change the tests to the real mass.** `test_lunar_descent_session.cpp`: every `place_in_llo(1065e3, ...)`
  becomes `place_in_llo(test::hls_llo_propellant_kg, ...)` (include `lunar_test_support.hpp`); the refusal section
  "propellant short ... (204 t dry, half tanks)" keeps its 400e3 / 204e3 (it is a refusal case). The budget unit test in
  `test_lunar_descent.cpp` uses `566e3` instead of `1185e3` (both lines). The Task 9 test `[burn_engines]` stays.

- [ ] **Step 2: Run** `-TestFilter "[lunar_descent_session]"` plus the slow landing case by name. Expected: the landing
  from the LLO passes after C2 (if not: systematic-debugging; the trim/DOI now fly on sea_level_1).

- [ ] **Step 3: Budget values from the flight.** From the C2 and C3 runs compute the flown descent delta-v as the
  rocket equation with the braking set's share: `dv = 380 g0 ln(m_pdi / m_drop) + 350 g0 ln(m_drop / m_landed)` (m_drop:
  mass at the drop to sea_level_1; print it in the C2 test) and set `LunarLandingSetup::descent_delta_v_mps` /
  `braking_delta_v_mps` and the TOMLs' `descent_delta_v_mps` to the flown values rounded up to 50 m/s, tagged [D] with
  the run in the comment. Do NOT change the 204 t expectation - l4 Task 9 re-checks it with these values.

- [ ] **Step 4: Run** `-TestFilter "[lunar_descent_session],[lunar_descent],[config][lunar]"`. Expected: pass.

- [ ] **Step 5: Commit** (no push).

```bash
git add core/tests/test_lunar_descent_session.cpp core/tests/test_lunar_descent.cpp core/include/starsim/sim/flight_session.hpp config/missions/hls_lunar_llo.toml config/missions/hls_lunar_nrho.toml
git commit -m "fix(sim): lunar descent autopilot at the real HLS mass; budget descent delta-v from the flight"
```

---

### Task C4: Ascent and aborts at the real mass, full suite, push

**Files:**
- Modify: `core/tests/test_lunar_ascent.cpp`, `core/src/sim/flight_session_lunar_ascent.cpp` /
  `core/src/gnc/lunar_ascent_guidance.cpp` only if a test fails for a cause found by systematic-debugging

**Interfaces:**
- Consumes: C1 constants, C2 descent.

- [ ] **Step 1: Change the scenarios.** In `test_lunar_ascent.cpp`: `hls(542e3)` -> `hls(test::hls_landed_propellant_kg)`
  everywhere (comment "~340 t, LLO variant"); add the light case to "After a 3-day stay ..." as a loop over
  `{hls_landed_propellant_kg, hls_landed_light_propellant_kg}`; `place_hls_at_pdi(..., 1065e3 ...)` ->
  `test::hls_pdi_propellant_kg` with the range from `mission::braking_range_m` as in C2 (the auto-abort case keeps its
  short 60 km range: it must be unreachable); the refusal-at-1-km case ("heavy dry mass leaves ~20 t at 1 km") re-derives
  its dry mass: print the propellant left at 1 km for dry 300e3, adjust by the printed difference until ~20 t are left,
  hard-code it with a comment of the derivation. Print the peak thrust acceleration of each ascent (review focus 4).

- [ ] **Step 2: Run** `-TestFilter "[lunar_ascent]~[slow]"` and the slow ascent/abort cases by name in the background.
  Expected: ascents reach the LLO (100 +- 1 km), the 1 km abort reaches orbit, the refusals hold. Failures:
  systematic-debugging, one hypothesis at a time, ledger.

- [ ] **Step 3: Full suite and smoke.** `powershell -ExecutionPolicy Bypass -File tools/build.ps1` (with bindings, full
  ctest incl. `[slow]`) and the Godot smoke test. Expected: all pass except l4 Task 9's two mission flights and its
  budget case (red until l4 Task 9 resumes) - list them in the ledger line. If anything else fails, fix it here.

- [ ] **Step 4: Commit and push** (the Task 9 work stays uncommitted in the tree: `git add` only these files).

```bash
git add core/tests/test_lunar_ascent.cpp
git commit -m "test(sim): lunar ascent and aborts at the real HLS mass"
git push origin main
```

- [ ] **Step 5: Resume l4 Task 9** from its ledger: re-run the budget case and both mission flights (the epoch tuning
  for a < 2 day wait and the 204 t check with the new budget values belong to Task 9).
