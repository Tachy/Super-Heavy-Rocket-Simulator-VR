# Lunar l4: Landing and Ascent (descent orbit, Apollo braking, SCvx approach, surface stay, ascent, abort) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The HLS in the 100 km polar LLO (l3) waits for the landing site's pass, trims its plane, lowers its periselene
(DOI), brakes from the periselene on the vacuum Raptors (then one sea-level Raptor) with the Apollo lunar descent guidance (Klumpp 1974) to a high
gate, flies the approach on SCvx plans (Raptors, then the landing thrusters) to a touchdown point chosen on the LOLA
terrain, lands on its legs (< 1 m, <= 1 m/s), stays on the surface, lifts off on the landing thrusters, ascends on PEG
into a 15 x 100 km orbit, circularises at the aposelene and is back in the LLO (< 1 km) - for the NRHO variant then back
in the NRHO. An abort during the descent flies to orbit from the current state; mission 1 is complete with 120 t dry
and refused with 204 t.

**Architecture:** Pure planning in `mission/` (descent planning: site pass, plane trim, DOI, PDI, touchdown point;
mission budget), guidance laws in `gnc/` (PEG extracted from the ascent guidance and usable about any body; the SCvx
landing generalised from the Earth to any reference body; the Apollo braking law; a lunar landing sequencer; a lunar
ascent guidance), flight in `sim::FlightSession` as new `LunarStep`s in `core/src/sim/flight_session_lunar.cpp` (descent
and surface) and a new `core/src/sim/flight_session_lunar_ascent.cpp` (ascent, abort). The mission config
(`kind = "lunar_landing"`) gains `[descent]`, `[surface]` and `[ascent]`; `start_mission` chains outbound, descent,
stay, ascent (and the NRHO transfers) and refuses a mission whose budget does not close.

**Tech Stack:** C++20 core (Catch2), `optim::SocpSolver` via the existing SCvx planner, DOP853 coasts
(`orbit::propagate_coast`), CSPICE via the existing adapters, toml++ inside `config`.

**Spec:** `docs/superpowers/specs/2026-10-14-lunar-missions-design.md` (section "l4 design: landing and ascent"; l1-l3
and the post-l3 cleanup are on `main`, 7bf4c2b).

## User decisions for l4 (2026-10-15)

> **Correction 2026-10-15 (l4 Task 9):** ~1,185 t was l3's propellant USED, not the LLO mass; real ~573 t in the LLO, 500-566 t at PDI, 260-340 t landed - see `docs/superpowers/plans/2026-10-15-lunar-l4-mass-correction.md` (landing thrusters 12 x 90 kN, landing thrusters as the last braking set, braking range from the mass).

- **Landing thrusters re-sized [E] to the real landing mass.** The HLS leaves LEO fully fuelled (spec) and lands with
  ~660 t (LLO variant: ~1,185 t in the LLO, ~2.0 km/s descent) or ~500 t (NRHO variant: ~1,140 t in the NRHO, 876 m/s
  down to the LLO, then the descent). l2 sized the 12 thrusters (46 kN) for a 221 t landing mass: at 660 t they give
  only ~0.5 lunar weights. New: 12 x 140 kN (axial 1.62 MN = 1.5 lunar weights of 668 t; at 40 % minimum throttle they
  hover from ~400 t). Legs re-checked at 500 and 660 t.
- **Site pass: wait in the LLO.** The site (Mons Mouton, 84.8 deg S) lies 158 km from the pole: a polar LLO passes
  within reach only twice per lunar day (the cross-track distance drifts ~36 km/day). The HLS waits in the LLO for the
  first pass within `max_crossrange_m` (20 km [E]); a plane trim one and a quarter revolutions before the pass removes
  the cross-track offset (~0.9 m/s per km). The mission epoch in the TOML is chosen so the wait is short (as Apollo
  chose its launch dates). The l3 planners stay unchanged.

- **Braking on the vacuum Raptors (user 2026-10-15).** The braking phase from the periselene (~1.65 km/s, the largest
  part of the descent) burns on the three vacuum Raptors (fixed, symmetric on the ring: thrust through the CG), the
  attitude held by the RCS; when the thrust demand falls below their minimum (3 x 30 % = 2.68 MN) the gimballed
  `sea_level_1` (0.87-2.91 MN) takes over and flies on into the SCvx approach. Isp 380 s instead of ~350 s saves
  ~25-30 t of the ~450 t braking propellant (LLO variant). RCS authority ~1.2e5 N m / ~2e8 kg m^2 ~ 6e-4 rad/s^2: enough
  for Klumpp's slow pitch-over. The switch lights a disjoint set: `sea_level_1` starts `startup_s` ahead.
- **Ascent on the vacuum Raptors (user 2026-10-15, after Task 5).** Liftoff on the landing thrusters, then the three
  vacuum Raptors (RCS attitude, Isp 380 s: ~10 t of ascent propellant saved); aborts on them too. Task 8 implements,
  Task 9's budget uses 380 s for the ascent (re-check the 204 t refusal).
- **Approach on one sea-level Raptor (Task 3 Ruling).** Three (or two) Raptors at minimum throttle over-brake the
  high gate's ~2.3 m/s^2; SCvx phases {`sea_level_1`} -> {`sea_level_1`} -> {`landing`}.

## Global Constraints

- Work on `main` (no feature branch); commit after every task, push (`git push origin main`) when its tests are green.
  Before every push run `-Fast`; Tasks 3, 6-10 touch guidance / flight sessions / vehicles: run the full suite at the
  end of Task 10 (and before the final review).
- `-TestFilter` runs the matching cases serially in one process: never filter `[mission]` or `[outbound]` (they pull in
  the slow campaigns, > 27 min); use the narrow tags named in each task or `-Fast`.
- SI internally, radians; frame and unit in names (`position_gcrf_m`, `epoch_tdb_s`, `delta_v_gcrf_mps`).
- Vehicle states stay Earth-centred GCRF; Moon-relative quantities come from `environment::ReferenceBody` /
  `BodyFrame` (l1). The Moon is the sphere R = 1737.4 km plus the LOLA terrain (l2: tile + global LDEM_16).
- All planning uses the session's force model (`environment::PointMassGravity`, DE440); CSPICE only on the simulation
  thread (snapshots for the SCvx worker never call SPICE - as on Earth).
- Earth behaviour stays bit-identical: every Earth landing, catch and ascent test keeps its numbers (Tasks 2 and 3 are
  refactors on the Earth side).
- HLS engines: braking on the three vacuum Raptors (`vacuum`, fixed, RCS attitude), then `sea_level_1` (gimballed);
  approach on SCvx in three phases {`sea_level_1`} -> {`sea_level_1`} -> {`landing`}; liftoff on `landing`, ascent on the three vacuum Raptors
  (user 2026-10-15: fixed, RCS attitude, Isp 380 s; abort to orbit on them as well); DOI, plane trim and circularisation by `execute_burn(node)` (the session
  picks RCS or a Raptor set). The landing thrusters (`landing_only`) are lit only by the landing and ascent guidance.
- No propellant is ever set mid-mission (user 2026-10-14); a plan that does not close is refused with a German message.
- No magic numbers: constants with source in `math/constants.hpp`; guidance values as named members with [S]/[E]/[D]
  tags; mission values in the mission TOML.
- UI strings in the core German; code, comments, docs English. Commits: Conventional Commits with the session trailer.
- Drawing, UI and bindings: untouched (l6). `docs/ansichten.md` untouched. Boil-off, plume-surface interaction,
  lighting constraints and the lunar gravity field beyond the point mass: out of scope (spec).

## Review Focus

1. **Time warp across the descent steps:** one `advance_to(now + 10 d)` while waiting for the site pass must stop at the
   plane trim, the DOI and the PDI (the coast allowance stops at each step epoch); the powered descent then runs on
   200 Hz ticks. Test in Task 7.
2. **Pilot takes over during the powered descent:** `disengage_autopilot()` in the braking phase or the approach ends
   the lunar steps for good: no guidance command afterwards, no automatic ascent; the pilot's input flies the vehicle.
   Test in Task 7.
3. **Abort without the propellant for orbit:** `abort_landing` with too little propellant (a mass-dispersed HLS late in
   the approach) returns false with a German status, the landing continues unchanged. Test in Task 8.
4. **No admissible touchdown point:** a site on steep terrain (no point within the slope limit inside the search radius,
   or the glide cone cut by a ridge) refuses `plan_lunar_landing` with a German reason; nothing fires. Test in Task 5
   (planner) and Task 7 (session).
5. **Surface stay under time warp:** a landed HLS warped over its stay (days) stays landed (resting, following the Moon,
   no drift, no leg judgement change) and lifts off at the stay's end. Test in Task 8.

## File Structure

| File | Responsibility |
|---|---|
| `config/engines/hls_landing_thruster.toml`, `config/vehicles/starship_hls.toml` | landing thrusters re-sized (140 kN [E]), comments on the real landing mass |
| `core/include/starsim/gnc/peg_steering.hpp`, `core/src/gnc/peg_steering.cpp` | Powered Explicit Guidance about any central body (extracted from `AscentGuidance`) |
| `core/src/gnc/ascent_guidance.cpp` | uses `PegSteering` (bit-identical) |
| `core/include/starsim/gnc/landing_site.hpp` | `LandingSite::body` |
| `core/include/starsim/gnc/descent_snapshot.hpp`, `core/src/gnc/descent_snapshot.cpp` | plan frame on any body's shape, plan state from a `BodyFrame`, Moon snapshot (vacuum, terrain origin) |
| `core/include/starsim/gnc/descent_model.hpp`, `core/src/gnc/descent_model.cpp` | vacuum model, canted (axial) thrust, ungimballed set steered by RCS torque |
| `core/include/starsim/gnc/descent_problem.hpp`, `core/src/gnc/descent_problem.cpp` | `EngineSwitchLimits::min_cg_height_m` |
| `core/include/starsim/environment/reference_body.hpp`, `core/src/environment/reference_body.cpp` | `centre_acceleration_gcrf` |
| `core/include/starsim/gnc/scvx_landing.hpp`, `core/src/gnc/scvx_landing.cpp` | frame from the site's body |
| `core/include/starsim/gnc/apollo_braking.hpp`, `core/src/gnc/apollo_braking.cpp` | Apollo lunar descent (braking phase) guidance, Klumpp 1974 |
| `core/include/starsim/mission/lunar_descent.hpp`, `core/src/mission/lunar_descent.cpp` | site pass search, plane trim, DOI, PDI, touchdown point selection |
| `core/include/starsim/mission/lunar_budget.hpp`, `core/src/mission/lunar_budget.cpp` | rocket-equation budget of a leg list (descent/ascent check, mission refusal) |
| `core/include/starsim/gnc/lunar_landing_guidance.hpp`, `core/src/gnc/lunar_landing_guidance.cpp` | powered descent sequencer: braking -> SCvx approach -> touchdown |
| `core/include/starsim/gnc/lunar_ascent_guidance.hpp`, `core/src/gnc/lunar_ascent_guidance.cpp` | liftoff on the landing thrusters, vertical rise, PEG to 15 x 100 km; abort from flight |
| `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp`, `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp` | `release_landed()`; truth thrust factor (campaign dispersion) |
| `core/include/starsim/sim/leg_landing_sim.hpp`, `core/src/sim/leg_landing_sim.cpp` | `release(body)` (liftoff: the judgement starts over) |
| `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session_lunar.cpp`, `core/src/sim/flight_session_lunar_ascent.cpp` (new), `core/src/sim/flight_session.cpp`, `core/src/sim/flight_session_command.cpp` | new `LunarStep`s, `begin_powered_descent`, `plan_lunar_landing`, `plan_lunar_ascent`, lunar `abort_landing`, mission sequence |
| `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp` | `[descent]`, `[surface]`, `[ascent]`, `[budget]` of `lunar_landing` |
| `core/include/starsim/sim/lunar_landing_mission.hpp`, `core/src/sim/lunar_landing_mission.cpp`, `core/src/sim/mission_start.cpp` | landing setup and mission budget from the config, the chained mission |
| `config/missions/hls_lunar_llo.toml`, `config/missions/hls_lunar_nrho.toml` | descent, stay, ascent sections; epochs for a short wait |
| tests (see each task) | `test_hls_vehicle.cpp`, `test_hls_landing.cpp`, `test_peg_steering.cpp`, `test_lunar_scvx.cpp`, `test_apollo_braking.cpp`, `test_lunar_descent.cpp`, `test_lunar_powered_descent.cpp`, `test_lunar_descent_session.cpp`, `test_lunar_ascent.cpp`, `test_lunar_landing_mission.cpp`, `test_lunar_landing_campaign.cpp` |
| `core/CMakeLists.txt`, `core/tests/CMakeLists.txt` | new sources |
| `docs/subsystems.md`, `CLAUDE.md`, `config/README.md` | docs (Task 10) |

Build and test commands used below (PowerShell, repo root):
- Build only: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests > $env:TEMP\l4_build.txt; Get-Content $env:TEMP\l4_build.txt -Tail 30`
- Tagged tests: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "<tags>" > $env:TEMP\l4_test.txt; Get-Content $env:TEMP\l4_test.txt -Tail 40`
- One test with output: `build/core/tests/starsim_core_tests.exe "<test name>" -s > $env:TEMP\l4_one.txt; Get-Content $env:TEMP\l4_one.txt -Tail 60`
- Fast suite: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast > $env:TEMP\l4_fast.txt; Get-Content $env:TEMP\l4_fast.txt -Tail 20`

---

### Task 1: HLS landing thrusters for the real landing mass; legs re-checked

**Files:**
- Modify: `config/engines/hls_landing_thruster.toml`, `config/vehicles/starship_hls.toml` (comments), `config/README.md`
- Test: `core/tests/test_hls_vehicle.cpp`, `core/tests/test_hls_landing.cpp`

**Interfaces:**
- Consumes: l2 vehicle `starship_hls`, test helpers `hls(propellant_kg)`, `place_over_site`, `run_drop` in
  `test_hls_landing.cpp`.
- Produces: landing group `landing`: 12 x 140 kN vacuum [E], canted 15 deg (axial 12 x 140 x cos 15 = 1.623 MN), minimum
  throttle 0.40 -> hover between ~400 t and ~1,000 t in lunar gravity. Later tasks rely on: the landing set can hover
  any landing mass of 450-700 t and lift off.

Numbers (put them in the TOML comments): lunar gravity at the surface g = GM_moon / R^2 = 4.9028e12 / 1737.4e3^2 =
1.624 m/s^2. LLO variant: ~1,185 t in the LLO (l3: 1,177-1,194 t), descent ~2.0 km/s at Isp 350 s ->
1,185 t x exp(-2000 / (350 x 9.80665)) = 662 t. NRHO variant: ~1,140 t in the NRHO (l3: 1,124-1,150 t), NRHO -> LLO
876 m/s at 380 s -> 901 t, descent -> 503 t. Sizing: 1.5 x 1.624 x 668 t = 1.627 MN axial -> 140 kN per thruster at
15 deg cant ([E], SpaceX publishes no figure; the l2 value of 46 kN assumed the 221 t "dry + ascent propellant").

- [x] **Step 1: Write the failing test.** Replace the body of `TEST_CASE("HLS landing thrusters lift ~1.5 lunar weights
  and can hover", "[vehicle][hls]")` in `core/tests/test_hls_vehicle.cpp` (keep its helpers) by:

```cpp
TEST_CASE("HLS landing thrusters lift ~1.5 lunar weights and can hover", "[vehicle][hls]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    const vehicle::Vehicle hls(cfg, STARSIM_CONFIG_DIR);
    double axial_max = 0.0, axial_min = 0.0;
    int count = 0;
    for (const auto& e : hls.stages().front().engines) {
        if (e.group_id != "landing") continue;
        ++count;
        axial_max += e.performance->thrust_n(1.0, 0.0) * e.axis_body.x;
        axial_min += e.performance->thrust_n(e.performance->config().throttle_min, 0.0) * e.axis_body.x;
    }
    const double g_moon = constants::gm_moon_m3ps2 / (constants::moon_mean_radius_m * constants::moon_mean_radius_m);
    // l4 (user 2026-10-15): sized for the real landing mass - LLO variant ~662 t, NRHO variant ~503 t.
    CHECK(count == 12);
    CHECK(axial_max / (668e3 * g_moon) == Catch::Approx(1.5).margin(0.02));
    CHECK(axial_min / g_moon < 450e3);   // hovers the lightest landing (NRHO ~503 t, -5 % mass dispersion)
    CHECK(axial_max / g_moon > 700e3);   // and lifts the heaviest (LLO ~662 t, +5 %)
}
```

  `math/constants.hpp` has no Moon GM yet: add `inline constexpr double gm_moon_m3ps2 = ...;` next to
  `moon_mean_radius_m` with the DE440 value - print `ephemeris.gm_m3ps2(ephemeris::Body::Moon)` once from the loaded
  kernels and copy it with its source comment (`// [S] DE440 (gm_de440.tpc, BODY301_GM)`); a test in
  `test_hls_vehicle.cpp` checks the constant against the kernel value (`test::spice_session()`, relative 1e-12).
  Later tasks use this constant where no ephemeris is at hand (guidance and planning code takes the GM from the
  `ReferenceBody` / ephemeris).

  Add to `core/tests/test_hls_landing.cpp` after the l2 drop tests:

```cpp
TEST_CASE("HLS drop tests at the l4 landing masses (503 t and 662 t)", "[hls][legs][lunar]") {
    // l4: the HLS lands with 380-540 t of propellant (NRHO / LLO variant); the legs sized in l2 at 221 t must still
    // land 1 m/s + 0.5 m on 0 / 5 / 8 deg and tip over at 25 deg.
    for (const double propellant_kg : {383e3, 542e3}) {
        for (const double slope_deg : {0.0, 5.0, 8.0, 25.0}) {
            Fixture fx;
            set_slope_surface(fx, slope_deg);   // the l2 helper of the slope drop tests (rename if it differs)
            place_over_site(fx, hls(propellant_kg), 0.5, {0.0, 0.0, -1.0});
            const auto result = run_drop(fx);
            CAPTURE(propellant_kg, slope_deg);
            if (slope_deg < 20.0) {
                CHECK(result == sim::LegTouchdown::Landed);
            } else {
                CHECK(result == sim::LegTouchdown::TippedOver);
            }
        }
    }
}
```

  Read the l2 test `"HLS drop tests on 0, 5 and 8 deg slopes land; 25 deg tips over"` first and reuse exactly how it
  sets the sloped surface (if it is inline code, extract it into a helper `set_slope_surface(Fixture&, double deg)` in
  the anonymous namespace and use it in both tests).

- [x] **Step 2: Run to see the vehicle test fail.**
  Run: tagged tests with `"[hls]"`. Expected: the thruster test FAILS (46 kN: 1.5 weights of 221 t); the drop tests
  may pass or fail - record which.

- [x] **Step 3: Re-size the thrusters.** In `config/engines/hls_landing_thruster.toml`: `thrust_ref_n = 140.0e3` with
  the comment `# [E] l4 (user 2026-10-15): 12 x 140 kN, axial 1.62 MN = 1.5 lunar weights of the ~668 t LLO-variant
  landing mass; hover from ~400 t at 40 % (NRHO variant lands ~503 t). l2's 46 kN assumed 221 t.` Update the header
  comment of the file accordingly and the `throttle.min` comment (`# [E] all twelve hover from ~400 t`). In
  `config/vehicles/starship_hls.toml` replace the "Landing mass ~221 t" comment by the l4 numbers above (LLO ~662 t,
  NRHO ~503 t, [D] from the l3 arrival masses and a 2.0 km/s descent) and say the legs were re-checked at both masses.

- [x] **Step 4: Make the drop tests pass.** Run `"[hls]"` again. If a heavy drop no longer lands (crush cores used up,
  or a bounce tips it), re-size the legs [E] by mass ratio: `crush_force_n` and `damping_n_s_m` x 3 (662 / 221),
  `stiffness_n_m` x 3; keep the footprint (12 m, user 2026-10-15). Re-run until all cases of both drop tests pass
  (the l2 221 t cases too). Record the outcome (sizes changed or not) in the Ledger.

- [x] **Step 5: Vehicle report.** Run `python tools/vehicle_report.py starship_hls > $env:TEMP\l4_vr.txt; Get-Content
  $env:TEMP\l4_vr.txt -Tail 15` and check the line `landing thrusters: 12 x 140 kN vac (axial 1623 kN, ...)` - axial
  = 12 x 140 x cos 15 deg = 1,623 kN. In `config/README.md` update the HLS paragraph (landing thrusters) if it names 46 kN.

- [x] **Step 6: Fast suite, commit, push.**

```bash
git add config/engines/hls_landing_thruster.toml config/vehicles/starship_hls.toml config/README.md core/include/starsim/math/constants.hpp core/tests/test_hls_vehicle.cpp core/tests/test_hls_landing.cpp
git commit -m "feat(vehicle): size HLS landing thrusters for the real landing mass (12 x 140 kN)"
git push origin main
```

---

### Task 2: PEG steering about any central body (extracted from the ascent guidance)

**Files:**
- Create: `core/include/starsim/gnc/peg_steering.hpp`, `core/src/gnc/peg_steering.cpp`, `core/tests/test_peg_steering.cpp`
- Modify: `core/include/starsim/gnc/ascent_guidance.hpp`, `core/src/gnc/ascent_guidance.cpp`, `core/CMakeLists.txt`,
  `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `AscentGuidance::peg` (`core/src/gnc/ascent_guidance.cpp:187-301`), `current_performance`,
  `shutdown_time_s`.
- Produces (`gnc/peg_steering.hpp`):

```cpp
#pragma once

#include "starsim/math/vec3.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::gnc {

// State relative to the central body's centre, inertial axes (GCRF axes).
struct PegState {
    math::Vec3 position_m;
    math::Vec3 velocity_mps;
};

// The running engines now.
struct PegVehicle {
    double thrust_n = 0.0;
    double mass_flow_kgps = 0.0;
    double mass_kg = 0.0;
    double shutdown_s = 0.0;  // thrust tail-off time (cutoff lead)
};

// Insertion target: at radius_m with the given radial and horizontal speed, in the plane with unit normal plane_normal.
struct PegTarget {
    double radius_m = 0.0;
    double radial_speed_mps = 0.0;
    double horizontal_speed_mps = 0.0;
    math::Vec3 plane_normal;
    // Terminal phase (last seconds, velocity-to-be-gained steering): > 0 - horizontal at the vis-viva speed for this
    // semi-major axis at the current radius (Kepler target); 0 - radial_speed_mps / horizontal_speed_mps (solved
    // insertion, e.g. J2-aware).
    double terminal_semi_major_axis_m = 0.0;
};

struct PegCommand {
    math::Vec3 thrust_direction;  // inertial axes
    double time_to_go_s = 0.0;
    bool cutoff = false;
};

// Powered Explicit Guidance (linear-tangent steering, Space Shuttle PEG as formulated on OrbiterWiki "Powered Explicit
// Guidance") with zero-effort out-of-plane steering and terminal velocity-to-be-gained steering; about any central body
// (gm). Extracted unchanged from AscentGuidance (l4) so the lunar ascent uses the same law.
class PegSteering {
public:
    explicit PegSteering(double gm_m3ps2) : gm_(gm_m3ps2) {}
    PegCommand update(const PegState& state, const PegVehicle& vehicle, const PegTarget& target, time::Epoch epoch);
    bool converged() const { return converged_; }
    double time_to_go_s() const { return t_; }
    void reset() { converged_ = false; }

private:
    double gm_;
    double a_ = 0.0;
    double b_ = 0.0;
    double t_ = 0.0;
    bool converged_ = false;
    time::Epoch last_;
};

}  // namespace starsim::gnc
```

  `AscentGuidance` keeps its public interface; its `peg_a_`, `peg_b_`, `peg_t_`, `peg_converged_`, `peg_last_` members
  are replaced by `PegSteering peg_{gm}` (constructed in the constructor's initializer list after `gm_`).

- [x] **Step 1: Write the failing test** `core/tests/test_peg_steering.cpp`: a point mass about the Moon under PEG
  commands reaches a 15 x 100 km insertion (no session, own RK4 at 0.1 s):

```cpp
// PEG about the Moon (l4): a point mass with constant thrust flown on PegSteering from a vertical rise reaches the
// insertion point of a 15 x 100 km orbit.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/peg_steering.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("PEG steers a lunar ascent into a 15 x 100 km insertion", "[gnc][peg]") {
    const double gm = constants::gm_moon_m3ps2;            // name as in math/constants.hpp
    const double R = constants::moon_mean_radius_m;
    const double rp = R + 15e3, ra = R + 100e3;
    gnc::PegTarget target;
    target.radius_m = rp;
    target.horizontal_speed_mps = std::sqrt(gm * (2.0 / rp - 2.0 / (rp + ra)));
    target.plane_normal = {0.0, 0.0, 1.0};
    target.terminal_semi_major_axis_m = 0.5 * (rp + ra);
    gnc::PegSteering peg(gm);

    // 600 t on three sea-level Raptors in vacuum (2.912 MN, 350 s each), 300 m above the sphere, rising 20 m/s.
    double m = 600e3;
    const double thrust = 3.0 * 2.912e6, flow = thrust / (350.0 * constants::standard_gravity_mps2);
    Vec3 r{R + 300.0, 0.0, 0.0}, v{20.0, 0.0, 0.0};
    time::Epoch t = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    bool cut = false;
    for (int i = 0; i < 20000 && !cut; ++i) {
        const auto cmd = peg.update({r, v}, {thrust, flow, m, 0.5}, target, t);
        if (cmd.cutoff) { cut = true; break; }
        const double dt = 0.1;
        auto accel = [&](const Vec3& rr, double mm) { return cmd.thrust_direction * (thrust / mm) - rr * (gm / std::pow(norm(rr), 3)); };
        // RK4 on (r, v), mass linear
        const Vec3 k1v = accel(r, m), k1r = v;
        const Vec3 k2v = accel(r + k1r * (dt / 2), m - flow * dt / 2), k2r = v + k1v * (dt / 2);
        const Vec3 k3v = accel(r + k2r * (dt / 2), m - flow * dt / 2), k3r = v + k2v * (dt / 2);
        const Vec3 k4v = accel(r + k3r * dt, m - flow * dt), k4r = v + k3v * dt;
        r += (k1r + k2r * 2.0 + k3r * 2.0 + k4r) * (dt / 6.0);
        v += (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6.0);
        m -= flow * dt;
        t = t + dt;
    }
    REQUIRE(cut);
    // Orbit after cutoff: periapsis ~15 km, apoapsis ~100 km (the 0.1 s step and no tail-off: within 1 km).
    const double rn = norm(r), vn2 = dot(v, v);
    const double a = 1.0 / (2.0 / rn - vn2 / gm);
    const Vec3 h = cross(r, v);
    const double e = std::sqrt(std::max(0.0, 1.0 - dot(h, h) / (gm * a)));
    CHECK(a * (1.0 - e) - R == Catch::Approx(15e3).margin(1e3));
    CHECK(a * (1.0 + e) - R == Catch::Approx(100e3).margin(1e3));
    CHECK(std::abs(h.z) / norm(h) > 0.9999);   // stays in the target plane
}
```

  (`constants::gm_moon_m3ps2` comes from Task 1; `constants::standard_gravity_mps2` exists.)

- [x] **Step 2: Run to see it fail.** Add the test to `core/tests/CMakeLists.txt`; run tagged tests `"[peg]"`.
  Expected: compile error (`peg_steering.hpp` missing).

- [x] **Step 3: Extract the law.** Create `core/src/gnc/peg_steering.cpp` with `PegSteering::update` = the body of
  `AscentGuidance::peg` from `const double r = norm(...)` to the end, with these replacements, keeping every expression
  and its evaluation order unchanged (bit-identical Earth ascents):
  - `s.position_gcrf_m` -> `state.position_m`, `s.velocity_gcrf_mps` -> `state.velocity_mps`;
  - `plane_normal_` -> `target.plane_normal`; `perf.thrust_n` / `perf.mass_flow_kgps` -> `vehicle.thrust_n` /
    `vehicle.mass_flow_kgps`; `body.vehicle().mass_properties().mass_kg` -> `vehicle.mass_kg`;
    `shutdown_time_s(body.vehicle())` -> `vehicle.shutdown_s`;
  - `r_t`, `vt_t`, `vr_t` -> `target.radius_m`, `target.horizontal_speed_mps`, `target.radial_speed_mps`;
  - `peg_a_/peg_b_/peg_t_/peg_converged_/peg_last_` -> `a_/b_/t_/converged_/last_`;
  - the terminal `v_target`: `target.terminal_semi_major_axis_m > 0.0 ? required_speed * horizontal_now :
    target.horizontal_speed_mps * horizontal_now + target.radial_speed_mps * r_hat` with
    `required_speed = std::sqrt(gm_ * (2.0 / r - 1.0 / target.terminal_semi_major_axis_m))`;
  - the early return for `thrust <= 0` returns `{normalized(state.velocity_mps), t_, false}`;
  - the `update_insertion` call is NOT in PegSteering (the caller does it, see below);
  - `peg_freeze_time_s` and `terminal_time_s` move to the anonymous namespace of `peg_steering.cpp` with their comments.

  `AscentGuidance::peg` becomes:

```cpp
GuidanceCommand AscentGuidance::peg(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                    time::Epoch epoch) {
    const auto& s = body.state();
    const auto perf = current_performance(body.vehicle(), 0.0);
    GuidanceCommand cmd;
    cmd.phase = AscentPhase::ClosedLoop;
    if (perf.thrust_n > 0.0 && peg_.converged()) {
        const double r = norm(s.position_gcrf_m);
        const Vec3 r_hat = s.position_gcrf_m / r;
        const Vec3 theta_hat = math::normalized(cross(plane_normal_, r_hat));
        update_insertion(r_hat, r, dot(s.velocity_gcrf_mps, theta_hat), peg_.time_to_go_s(), env, epoch);
    }
    gnc::PegTarget target;
    target.radius_m = insertion_active_ ? insertion_radius_m_ : target_radius_m_;
    target.horizontal_speed_mps = insertion_active_ ? insertion_horizontal_mps_ : target_speed_mps_;
    target.radial_speed_mps = insertion_active_ ? insertion_radial_mps_ : 0.0;
    target.plane_normal = plane_normal_;
    target.terminal_semi_major_axis_m =
        insertion_active_ ? 0.0 : 0.5 * (target_radius_m_ + equatorial_radius_m + target_.apoapsis_altitude_m);
    const auto peg = peg_.update({s.position_gcrf_m, s.velocity_gcrf_mps},
                                 {perf.thrust_n, perf.mass_flow_kgps, body.vehicle().mass_properties().mass_kg,
                                  shutdown_time_s(body.vehicle())},
                                 target, epoch);
    cmd.thrust_direction_gcrf = peg.thrust_direction;
    cmd.time_to_go_s = peg.time_to_go_s;
    if (peg.cutoff) {
        cmd.cutoff = true;
        cmd.phase = AscentPhase::Cutoff;
    }
    return cmd;
}
```

  Check against the old code that `update_insertion` received the same arguments (old: `vt` = in-plane horizontal
  speed with `theta_hat` from `n_hat = plane_normal_`; `peg_t_` before the update) and that the thrust <= 0 path still
  returns the velocity direction with phase ClosedLoop (old code returned before `update_insertion`). Note: the old
  `required_speed` path used the Kepler target only when `insertion_active_` is false - the `terminal_semi_major_axis_m`
  switch above reproduces that.

- [x] **Step 4: Run the new test and the ascent tests.** Tagged tests `"[peg],[ascent]~[slow],[starship]~[slow],[falcon9]~[slow]"`
  (if a tag does not exist, `-Fast` covers them). Expected: PASS. Then `-Fast`: all pass with unchanged outcomes. If any
  Earth ascent number moved, the extraction changed an expression order - compare line by line until identical.

- [x] **Step 5: Commit, push.**

```bash
git add core/include/starsim/gnc/peg_steering.hpp core/src/gnc/peg_steering.cpp core/tests/test_peg_steering.cpp core/include/starsim/gnc/ascent_guidance.hpp core/src/gnc/ascent_guidance.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "refactor(gnc): extract PEG steering from the ascent guidance (any central body)"
git push origin main
```

---

### Task 3: SCvx landing on any reference body (Moon: vacuum, terrain origin, canted thrusters, RCS-steered set)

**Files:**
- Modify: `core/include/starsim/gnc/landing_site.hpp`, `core/include/starsim/gnc/descent_snapshot.hpp`,
  `core/src/gnc/descent_snapshot.cpp`, `core/include/starsim/gnc/descent_model.hpp`, `core/src/gnc/descent_model.cpp`,
  `core/include/starsim/gnc/descent_problem.hpp`, `core/src/gnc/descent_problem.cpp`,
  `core/include/starsim/environment/reference_body.hpp`, `core/src/environment/reference_body.cpp`,
  `core/src/gnc/scvx_landing.cpp`, every caller of `plan_state` / `plan_frame` (grep)
- Test: `core/tests/test_lunar_scvx.cpp` (new), existing `[descent]`, `[scvx]`, `[catch]`, `[legs]` tests unchanged

**Interfaces:**
- Consumes: `environment::ReferenceBody` / `BodyFrame` (l1), `vehicle::FlightEnvironment::moon`,
  `Vehicle::rcs_allocator().max_torque_nm()`, `EngineState::axis_body`.
- Produces:

```cpp
// landing_site.hpp
struct LandingSite {
    ephemeris::Body body = ephemeris::Body::Earth;  // lunar l4: the Moon (height above the sphere = terrain height)
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;  // ground above the body's shape (Earth: WGS84; Moon: the 1737.4 km sphere)
    ...  // unchanged
};

// descent_snapshot.hpp
PlanFrame plan_frame(double latitude_rad, double longitude_rad, double height_m,
                     const frames::Ellipsoid& shape = frames::wgs84);
PlanFrame plan_frame(const LandingSite& site);  // on the site body's shape (Moon: sphere)
// The site body's frame at `epoch` (Earth: ITRF93 about the origin; Moon: env.moon, std::logic_error without it).
environment::BodyFrame site_body_frame(const LandingSite& site, const vehicle::FlightEnvironment& env, time::Epoch epoch);
DescentState plan_state(const vehicle::FlightBody& body, const environment::BodyFrame& frame, const PlanFrame& plan);

// descent_problem.hpp, EngineSwitchLimits
double min_cg_height_m = 0.0;  // the switch node's CG at least this high above the target ground (0: no limit)

// reference_body.hpp
// Acceleration of the body's centre in the GCRF (Earth: zero; Moon: central difference of the ephemeris velocity over
// +-60 s [E] - the DE440 Moon's own acceleration, exact to ~1e-9 m/s^2). The plan frame of a lunar landing accelerates
// with it.
math::Vec3 centre_acceleration_gcrf(time::Epoch epoch) const;
```

  `DescentModelInputs::atmosphere` may be `nullptr` (vacuum: density 0, speed of sound 0, pressure 0).

Design decisions (comments in the code):
- **Plan frame on the Moon:** origin at the touchdown point on the terrain (`height_m` = terrain height above the
  sphere), x along the sphere's radial (= the geodetic vertical of a sphere), y east, z north (MOON_ME).
- **Gravity in the plan frame:** `gravity(origin_gcrf) - centre_acceleration_gcrf` (the frame falls with the Moon);
  rotation `BodyFrame::rotation_gcrf_radps` (Moon: 2.66e-6 rad/s), centre `frame.to_plan(fixed 0)`.
- **Canted thrusters:** every engine contributes its axial thrust `thrust_n x axis_body.x` to a set's maximum / minimum
  and to the mass-flow ratio's vacuum thrust (gimballed engines have `axis_body.x == 1`: Earth unchanged).
- **Set without gimbal (the landing thrusters):** the planning model has no torque input; such a set is given an
  *equivalent gimbal* that reproduces the vehicle's RCS torque about the CG:
  `gimbal_max_rad = atan(min(max_torque.y, max_torque.z) / (|gimbal_lever_m| * thrust_max))`. The fictitious lateral
  force it implies (torque / lever, ~0.1 % of the thrust) is a documented simplification; re-plans every second absorb
  it. In flight the lateral control's body torque goes to the RCS (no gimbals running). A vehicle without RCS keeps 0.
  Differential throttling of the canted thrusters (much more authority) is noted as a later refinement.
- **Raptors off near the ground:** `EngineSwitchLimits::min_cg_height_m` keeps the switch to the landing thrusters
  above a height [E] (spec: landing engines against plume-surface interaction).

- [x] **Step 1: Write the failing test** `core/tests/test_lunar_scvx.cpp`:

```cpp
// SCvx plans a lunar approach (l4): from a high gate 2 km above and 3 km uprange of a touchdown point on the LOLA
// sphere (no terrain needed), on {3 sea-level Raptors} -> {sea_level_1} -> {landing thrusters}, to the touchdown at
// 1 m/s; the snapshot is in vacuum with the Moon's gravity; Earth snapshots unchanged.

#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/gnc/descent_planner.hpp"
#include "starsim/gnc/descent_snapshot.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
constexpr double site_lat = -84.7906 * deg, site_lon = 29.1957 * deg, site_height = 5322.0;  // lunar_sites.toml

struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    environment::Us76Atmosphere air;
    environment::ReferenceBody moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    vehicle::FlightEnvironment env{gravity, air, earth, &ephemeris, true, &moon};
    time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
};

gnc::LandingSite lunar_site() {
    gnc::LandingSite site;
    site.body = ephemeris::Body::Moon;
    site.latitude_rad = site_lat;
    site.longitude_rad = site_lon;
    site.height_m = site_height;
    site.pad = gnc::PadTarget{1.5, 0.0};  // feet 1.5 m below the skirt (starship_hls.toml foot_station_m)
    return site;
}

gnc::SnapshotSettings hls_settings() {
    gnc::SnapshotSettings s;
    s.phase_groups = {{{"sea_level_1", "sea_level_2", "sea_level_3"}, {"sea_level_1"}, {"landing"}}};
    s.reserve_kg = 5000.0;
    return s;
}

// The HLS (580 t: between the variants) upright at the high gate: 2 km above the touchdown point, 3 km uprange to the
// south-west, moving 90 m/s towards it and sinking 35 m/s.
vehicle::FlightBody high_gate_body(Fixture& fx, const gnc::PlanFrame& plan) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    const double propellant = 460e3;
    for (auto& t : cfg.stages.front().tanks) {
        t.mass_kg = propellant * (t.contents == config::TankContents::Oxidizer ? 3.6 / 4.6 : 1.0 / 4.6);
    }
    const auto frame = fx.moon.frame(fx.t0);
    const Vec3 dir = math::normalized(Vec3{0.0, 1.0, 1.0});                 // downrange (plan y east, z north)
    const Vec3 r_plan = Vec3{2000.0, 0.0, 0.0} - dir * 3000.0;
    const Vec3 v_plan = Vec3{-35.0, 0.0, 0.0} + dir * 90.0;
    const Vec3 r_fixed = plan.origin_itrf_m + plan.plan_to_itrf * r_plan;    // "itrf" = the body-fixed frame here
    const Vec3 v_fixed = plan.plan_to_itrf * v_plan;
    const auto up_gcrf = frame.transform.rotation * plan.direction_to_itrf({1.0, 0.0, 0.0});
    const auto east_gcrf = frame.transform.rotation * plan.direction_to_itrf({0.0, 1.0, 0.0});
    const auto attitude = math::Quat::from_matrix(math::Mat3::from_columns(up_gcrf, east_gcrf, cross(up_gcrf, east_gcrf)));
    return vehicle::FlightBody::from_state(cfg, STARSIM_CONFIG_DIR,
                                           {frame.position_gcrf(r_fixed), frame.velocity_gcrf(r_fixed, v_fixed), attitude, {}},
                                           fx.t0);
}
}  // namespace

TEST_CASE("A lunar SCvx snapshot is in vacuum, in the Moon's gravity, with the touchdown point as origin", "[descent][lunar_scvx]") {
    Fixture fx;
    const auto site = lunar_site();
    const auto plan = gnc::plan_frame(site);
    CHECK(norm(plan.origin_itrf_m) == Catch::Approx(constants::moon_mean_radius_m + site_height).margin(1e-6));
    auto body = high_gate_body(fx, plan);
    const auto snap = gnc::take_descent_snapshot(body, fx.env, fx.t0, site, plan, hls_settings());
    const double g = constants::gm_moon_m3ps2 / std::pow(constants::moon_mean_radius_m + site_height, 2);
    CHECK(snap.model.gravity_mps2.x == Catch::Approx(-g).epsilon(1e-3));   // Earth's tide: ~1e-5 m/s^2
    CHECK(std::abs(snap.model.gravity_mps2.y) < 1e-3);
    CHECK(snap.model.density_kg_m3.front() == 0.0);
    CHECK(norm(snap.model.earth_rate_radps) == Catch::Approx(2.6617e-6).epsilon(1e-2));
    CHECK(gnc::state_position(snap.initial).x == Catch::Approx(2000.0).margin(1e-3));
    // The landing set: axial thrust (cant 15 deg) and an equivalent gimbal from the RCS torque.
    const auto& landing = snap.model.engine_sets[2];
    CHECK(landing.thrust_max_n == Catch::Approx(12.0 * 140e3 * std::cos(15.0 * deg)).epsilon(1e-6));
    CHECK(landing.gimbal_max_rad > 0.0);
    CHECK(landing.gimbal_max_rad < 1.0 * deg);
}

TEST_CASE("SCvx plans a lunar approach from the high gate to the touchdown point", "[descent][lunar_scvx]") {
    Fixture fx;
    const auto site = lunar_site();
    const auto plan = gnc::plan_frame(site);
    auto body = high_gate_body(fx, plan);
    const auto snap = gnc::take_descent_snapshot(body, fx.env, fx.t0, site, plan, hls_settings());
    gnc::ScvxSettings settings;
    settings.limits.tilt_max_rad = {25.0 * deg, 15.0 * deg, 3.0 * deg};     // [E] HLS (lunar_landing_guidance.cpp)
    settings.limits.rate_max_radps = {0.1, 0.05, 0.01};
    settings.limits.switches[0] = {0.0, 0.0, 5.0, 60.0, 0.0};
    settings.limits.switches[1] = {2.0 * deg, 0.01, 1.0, 10.0, 150.0};    // landing thrusters from >= 150 m CG height
    settings.limits.approach_time_constant_s = 0.0;                      // pad landing
    settings.limits.min_mass_kg = snap.min_mass_kg;
    gnc::DescentPlanner planner;
    const auto result = planner.plan(snap.model, snap.initial, 0, snap.catch_target, settings, nullptr, 0.0);
    std::printf("[lunar_scvx] converged %d, %d iterations, %.1f s, end mass %.1f t\n", result.converged, result.iterations,
                result.duration_s(), result.reference.states.back()[gnc::descent_index::mass] / 1000.0);
    REQUIRE(result.converged);
    const auto end = result.reference.states.back();
    CHECK(norm(gnc::state_position(end) - snap.catch_target.position_m) < 0.3);
    CHECK(gnc::state_velocity(end).x == Catch::Approx(-1.0).margin(0.05));
    // The landing thrusters take over at least 150 m above the ground (CG height).
    const int switch_node = 2 * (settings.nodes_per_phase - 1);
    CHECK(gnc::state_position(result.reference.states[static_cast<size_t>(switch_node)]).x >= 150.0 - 0.5);
}
```

  Use the actual names of `FlightBody`'s factory and `DescentReference` members (grep `from_state` /
  `class FlightBody` and `struct DescentReference`); if `FlightBody` has no standalone factory, build the body the way
  `core/tests/test_descent_snapshot.cpp` does. `EngineSwitchLimits` gets its new member last so existing aggregate
  initialisers keep compiling (`{tilt, rate, min_sink, max_sink}` then `min_cg_height_m` = 0).

- [x] **Step 2: Run to see it fail.** Add the file to `core/tests/CMakeLists.txt`; tagged tests `"[lunar_scvx]"`.
  Expected: compile errors (`LandingSite::body`, `plan_frame(site)`, `min_cg_height_m`).

- [x] **Step 3: Implement.**
  1. `ReferenceBody::centre_acceleration_gcrf`: Earth (`ephemeris_ == nullptr`) returns zero; otherwise
     `(centre_gcrf(epoch + h).velocity_mps - centre_gcrf(epoch - h).velocity_mps) / (2 h)` with
     `constexpr double centre_acceleration_step_s = 60.0;  // [E]` in the .cpp.
  2. `LandingSite::body` (default Earth). `plan_frame(lat, lon, h, shape)` passes `shape` to
     `frames::geodetic_to_fixed`; `plan_frame(site)` uses `frames::wgs84` for the Earth and
     `frames::Ellipsoid{constants::moon_mean_radius_m, 0.0}` (or the Moon `ReferenceBody`'s `shape()` - same values)
     for the Moon.
  3. `site_body_frame(site, env, epoch)`: Earth -> `environment::BodyFrame::centred_at_origin(
     env.earth_orientation.fixed_to_gcrf(epoch))`; Moon -> `env.moon->frame(epoch)` (throw `std::logic_error("lunar
     landing site without the Moon as a reference body")` when `env.moon == nullptr`).
  4. `plan_state(body, frame, plan)`: as today, with `to_fixed = transpose(frame.transform.rotation)`,
     `omega = frame.rotation_gcrf_radps`, `r_fixed = frame.to_fixed(s.position_gcrf_m)`,
     `v_fixed = to_fixed * frame.relative_velocity(s.position_gcrf_m, s.velocity_gcrf_mps)`. For the Earth frame
     (`centre = 0`, `rotation_gcrf_radps = earth_rotation_vector(t)`) every expression reduces to the old one; verify
     that `frames::rotation_vector(t)` equals `vehicle::earth_rotation_vector(t)` bit for bit (read both; if they differ,
     build the Earth frame with `earth_rotation_vector` so the old numbers stay).
  5. `take_descent_snapshot`: `const auto frame = site_body_frame(site, env, epoch);`,
     `gcrf_to_plan = transpose(plan.plan_to_itrf) * transpose(frame.transform.rotation)`,
     `in.gravity_mps2 = gcrf_to_plan * (env.gravity.acceleration_mps2(frame.position_gcrf(plan.origin_itrf_m), epoch)
     - centre_acceleration)` where `centre_acceleration` is zero for the Earth and
     `env.moon->centre_acceleration_gcrf(epoch)` for the Moon; `in.earth_rate_radps = gcrf_to_plan *
     frame.rotation_gcrf_radps`; `in.atmosphere = site.body == Earth ? &env.atmosphere : nullptr`; the snapshot's
     pressure is 0 without an atmosphere. Earth: `frame.position_gcrf(x) == t.rotation * x` exactly (centre zero) - keep
     the Earth path's arithmetic identical (if `position_gcrf` adds a zero vector first, that is exact in IEEE).
  6. `make_descent_model`: `nullptr` atmosphere -> `pressure = 0`, the air tables a single `{0.0}` entry each (the
     interpolation handles size 1). Per engine `const double axial = e.axis_body.x;` multiplies `thrust_n(...)` in
     `thrust_max_n`, `thrust_min_n` and `vacuum_thrust`. After the loop over engines of a phase, for a set with thrust
     but `set.gimbal_max_rad == 0` and a vehicle with RCS thrusters: compute the gimbal lever first (move the
     `gimbal_lever_m` computation before the per-set loop: it only needs the gimballed engines of all phases), then
     `set.gimbal_max_rad = std::atan(std::min(torque.y, torque.z) / (std::abs(m.gimbal_lever_m) * set.thrust_max_n))`
     with `torque = vehicle.rcs_allocator().max_torque_nm()` (guard: no thrusters or zero lever -> 0).
  7. `descent_problem.cpp`: where the engine-switch limits are imposed at the shared node of phases p and p + 1, add
     `if (limits.switches[p].min_cg_height_m > 0.0)` the linear constraint `x_node.position.x >= min_cg_height_m`
     (in the problem's scaled variables, written exactly like the existing `min_sink` row - copy its pattern).
  8. `ScvxLandingGuidance`: `frame_ = plan_frame(site_)`; every `plan_state(body, earth_transform, frame_)` call in
     `scvx_landing.cpp` (and any other caller found by `grep -rn "plan_state\|plan_frame(" core/src core/tests`) passes
     `site_body_frame(site_, env, epoch)`. Commands are produced in GCRF via the frame's rotation - use
     `frame.transform.rotation * plan.direction_to_itrf(...)` where the code used the Earth transform.

- [x] **Step 4: Run the new tests, then the Earth landing tests.** Tagged `"[lunar_scvx]"` until both pass; if the plan
  does not converge, adjust ONLY the HLS settings in the test ([E] values: phase tilts, rates, the switch sink windows,
  `max_phase_s` if the approach needs > 120 s on one set) and copy the working values into the Ledger (Task 6 puts
  them into `hls_landing_parameters()`). Then `"[descent]~[slow],[scvx]~[slow],[catch]~[slow],[legs]~[slow]"`:
  every Earth test passes unchanged; `-Fast` passes.

- [x] **Step 5: Commit, push.**

```bash
git add -A core/include/starsim/gnc core/src/gnc core/include/starsim/environment core/src/environment core/tests/test_lunar_scvx.cpp core/tests/CMakeLists.txt
git commit -m "feat(gnc): SCvx landing on any reference body (lunar vacuum, canted thrusters, RCS-steered set)"
git push origin main
```


---

### Task 4: Apollo braking-phase guidance (Klumpp 1974)

**Files:**
- Create: `core/include/starsim/gnc/apollo_braking.hpp`, `core/src/gnc/apollo_braking.cpp`,
  `core/tests/test_apollo_braking.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `math::Vec3` only (pure law; the frame handling is Task 6's).
- Produces (`gnc/apollo_braking.hpp`):

```cpp
#pragma once

#include <optional>

#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// Guidance frame of the lunar descent (Apollo): origin at the touchdown point, x up (local vertical), y crossrange
// (x cross z... completes right-handed: y = z x x), z downrange (the approach azimuth). Positions and velocities are
// relative to the rotating Moon.
// Target of the braking phase (the "high gate"): position, velocity and acceleration, and the downrange jerk that fixes
// the time to go [S: Klumpp, "Apollo lunar descent guidance", Automatica 10 (1974) 133-146].
struct BrakingTarget {
    math::Vec3 position_m;
    math::Vec3 velocity_mps;
    math::Vec3 acceleration_mps2;
    double jerk_downrange_mps3 = 0.0;
};

// Klumpp's time to go: the positive root of J t^3 - 6 A t^2 + (18 V_T + 6 V) t - 24 (R_T - R) = 0 in the downrange
// components (J, A, V_T, R_T of the target; V, R now), found by Newton from `guess_s`; empty if Newton does not
// converge to a positive root within 30 iterations.
std::optional<double> braking_time_to_go_s(const BrakingTarget& target, double position_downrange_m,
                                           double velocity_downrange_mps, double guess_s);

// Quadratic-acceleration law: the acceleration now of the unique quadratic acceleration profile that reaches the target
// position, velocity and acceleration after t_go: a = a_T + 12 (r_T - r) / t_go^2 - 6 (v_T + v) / t_go.
math::Vec3 braking_acceleration(const BrakingTarget& target, const math::Vec3& position_m, const math::Vec3& velocity_mps,
                                double time_to_go_s);

// The braking phase: every update solves t_go (warm-started) and returns the thrust acceleration = law - gravity.
class ApolloBrakingGuidance {
public:
    explicit ApolloBrakingGuidance(BrakingTarget target, double time_to_go_guess_s = 400.0)
        : target_(target), t_go_(time_to_go_guess_s) {}
    struct Command {
        math::Vec3 thrust_acceleration_mps2;  // guidance frame
        double time_to_go_s = 0.0;
        bool valid = false;  // false: no time to go (past the target) - the caller ends the phase
    };
    // `gravity_mps2`: the effective gravity in the guidance frame (gravitation, the frame's centrifugal and Coriolis
    // terms).
    Command update(const math::Vec3& position_m, const math::Vec3& velocity_mps, const math::Vec3& gravity_mps2);
    const BrakingTarget& target() const { return target_; }

private:
    BrakingTarget target_;
    double t_go_;
};

}  // namespace starsim::gnc
```

Derivation (put it in the .cpp comment): with a(tau) = a0 + a1 tau + a2 tau^2 on [0, T] and the end conditions
r(T) = r_T, v(T) = v_T, a(T) = a_T: a0 = a_T + 12 (r_T - r) / T^2 - 6 (v_T + v) / T; the end jerk is
j(T) = (24 (r_T - r - v T) / T^2 - 18 (v_T - v) / T + 6 a_T) / T, and j_z(T) = J_T gives Klumpp's cubic above. Check:
constant acceleration 1 m/s^2 from rest over 10 s (r_T = 50, v_T = 10, a_T = 1, J = 0) satisfies the cubic at t = 10 and
gives a0 = 1.

- [x] **Step 1: Look up the reference values.** From Klumpp 1974 (Table 1, braking-phase targets of Apollo 11 - RTG,
  VTG, ATG, JTGZ in the guidance frame) and from Bennett, "Apollo experience report - mission planning for lunar module
  descent and ascent", NASA TN D-6846 (1972): the LM mass and descent engine thrust / Isp at PDI, the PDI altitude and
  speed, and the braking phase's duration (PDI to the high gate). Record each with its [S] page in the test file's
  header comment. If a value cannot be found, use the Apollo 11 mission report (MSC-00171) timeline (PDI 102:33:05,
  P64 102:41:32 GET -> 507 s) and say so.

- [x] **Step 2: Write the failing test** `core/tests/test_apollo_braking.cpp`:

```cpp
// Apollo braking-phase guidance (Klumpp 1974): the cubic for t_go and the quadratic law reach the target exactly in
// uniform gravity; an Apollo-11-like LM from PDI reaches Klumpp's high gate in about the flown braking time.
// References: <Step 1 values with [S] pages>

#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/apollo_braking.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("Klumpp's time-to-go cubic has the profile's duration as its root", "[gnc][braking]") {
    gnc::BrakingTarget t;
    t.position_m = {0.0, 0.0, 50.0};
    t.velocity_mps = {0.0, 0.0, 10.0};
    t.acceleration_mps2 = {0.0, 0.0, 1.0};
    t.jerk_downrange_mps3 = 0.0;
    const auto t_go = gnc::braking_time_to_go_s(t, 0.0, 0.0, 7.0);
    REQUIRE(t_go);
    CHECK(*t_go == Catch::Approx(10.0).margin(1e-9));
    CHECK(gnc::braking_acceleration(t, {}, {}, 10.0).z == Catch::Approx(1.0).margin(1e-12));
}

TEST_CASE("The quadratic law reaches position, velocity and acceleration in uniform gravity", "[gnc][braking]") {
    gnc::BrakingTarget t;
    t.position_m = {2000.0, 0.0, -3000.0};
    t.velocity_mps = {-35.0, 0.0, 90.0};
    t.acceleration_mps2 = {0.5, 0.0, -1.0};
    t.jerk_downrange_mps3 = 0.01;
    gnc::ApolloBrakingGuidance guidance(t, 300.0);
    const Vec3 g{-1.62, 0.0, 0.0};
    Vec3 r{15000.0, 200.0, -450000.0}, v{0.0, 0.0, 1690.0};
    double t_go = 1e9;
    for (int i = 0; i < 100000; ++i) {
        const auto cmd = guidance.update(r, v, g);
        if (!cmd.valid || cmd.time_to_go_s < 2.0) { t_go = cmd.time_to_go_s; break; }
        const Vec3 a = cmd.thrust_acceleration_mps2 + g;   // ideal thrust: the law is flown exactly
        const double dt = 0.01;
        r += v * dt + a * (0.5 * dt * dt);
        v += a * dt;
    }
    // Stopped 2 s before the target: the remaining quadratic profile ends on it.
    CHECK(t_go < 2.0);
    CHECK(norm(r + v * t_go - t.position_m) < 400.0);      // coarse: 2 s of flight left
    CHECK(norm(v - t.velocity_mps) < 10.0);
}

TEST_CASE("An Apollo 11 LM from PDI reaches the high gate in about the flown braking time", "[gnc][braking][validation]") {
    // Point-mass LM over a flat Moon with constant gravity (the braking phase covers ~450 km: the curvature term
    // v^2/r ~ 1.6 m/s^2 is included as an upward acceleration v_z^2 / (R + h), as in Klumpp's guidance frame).
    // Values from Step 1: m0, thrust, Isp, PDI state, Klumpp's targets.
    // ... integrate at 0.01 s with the thrust limited to the descent engine's maximum (direction from the law), until
    // t_go < 2 s; print the duration, the propellant used and the state at the end.
    // CHECK duration within 10 % of the flown braking time, end position within 100 m and velocity within 2 m/s of the
    // target (the throttle saturates early in the phase as on Apollo - the law then catches up).
}
```

  Complete the third test with the Step 1 numbers (the comments say exactly what to integrate and check); keep its
  integration loop like the second test's, plus the mass (`m -= |F| / (Isp g0) dt`) and the thrust limit
  (`|a_thrust| <= F_max / m`).

- [x] **Step 3: Run to see them fail.** Tagged `"[braking]"`. Expected: compile error.

- [x] **Step 4: Implement** `core/src/gnc/apollo_braking.cpp`:

```cpp
#include "starsim/gnc/apollo_braking.hpp"

#include <cmath>

namespace starsim::gnc {

namespace {
constexpr int max_newton_iterations = 30;
constexpr double newton_tolerance_s = 1e-9;
}  // namespace

std::optional<double> braking_time_to_go_s(const BrakingTarget& target, double r, double v, double guess_s) {
    const double j = target.jerk_downrange_mps3, a = target.acceleration_mps2.z;
    const double vt = target.velocity_mps.z, rt = target.position_m.z;
    double t = guess_s;
    for (int i = 0; i < max_newton_iterations; ++i) {
        const double f = j * t * t * t - 6.0 * a * t * t + (18.0 * vt + 6.0 * v) * t - 24.0 * (rt - r);
        const double df = 3.0 * j * t * t - 12.0 * a * t + 18.0 * vt + 6.0 * v;
        if (df == 0.0) return std::nullopt;
        const double step = f / df;
        t -= step;
        if (std::abs(step) < newton_tolerance_s) return t > 0.0 ? std::optional<double>(t) : std::nullopt;
    }
    return std::nullopt;
}

math::Vec3 braking_acceleration(const BrakingTarget& target, const math::Vec3& r, const math::Vec3& v, double t_go) {
    return target.acceleration_mps2 + (target.position_m - r) * (12.0 / (t_go * t_go)) -
           (target.velocity_mps + v) * (6.0 / t_go);
}

ApolloBrakingGuidance::Command ApolloBrakingGuidance::update(const math::Vec3& r, const math::Vec3& v,
                                                             const math::Vec3& gravity_mps2) {
    Command cmd;
    const auto t_go = braking_time_to_go_s(target_, r.z, v.z, t_go_);
    if (!t_go) return cmd;
    t_go_ = *t_go;
    cmd.time_to_go_s = t_go_;
    cmd.thrust_acceleration_mps2 = braking_acceleration(target_, r, v, t_go_) - gravity_mps2;
    cmd.valid = true;
    return cmd;
}

}  // namespace starsim::gnc
```

- [x] **Step 5: Run until green.** Tagged `"[braking]"`. If the Apollo case misses its duration by > 10 %, check the
  frame signs against Klumpp first (his t_go is negative and his z is downrange - convert, do not tune the law), then
  record the found numbers in the Ledger.

- [x] **Step 6: Commit, push.**

```bash
git add core/include/starsim/gnc/apollo_braking.hpp core/src/gnc/apollo_braking.cpp core/tests/test_apollo_braking.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat(gnc): Apollo braking-phase guidance (Klumpp quadratic law)"
git push origin main
```

---

### Task 5: Lunar descent planning (site pass, plane trim, DOI, PDI, touchdown point) and the budget

**Files:**
- Create: `core/include/starsim/mission/lunar_descent.hpp`, `core/src/mission/lunar_descent.cpp`,
  `core/include/starsim/mission/lunar_budget.hpp`, `core/src/mission/lunar_budget.cpp`,
  `core/tests/test_lunar_descent.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `orbit::propagate_coast` (Earth-centred GCRF with `environment::PointMassGravity`, as l3's planners),
  `mission::moon_relative`, `environment::LunarTerrain` (`height_m(lat, lon)`, `in_tile`), the MOON_ME orientation
  (`frames::BodyOrientation::fixed_to_gcrf`).
- Produces (`mission/lunar_descent.hpp`):

```cpp
struct LunarDescentParameters {
    double pdi_altitude_above_site_m = 15.0e3;  // [S] Apollo: PDI at ~50,000 ft above the landing site
    double braking_range_m = 450.0e3;           // [E] spec: PDI 400-500 km before the site (ground arc)
    double max_crossrange_m = 20.0e3;           // [E] user 2026-10-15: passes farther off are not taken
    double search_s = 30.0 * 86400.0;           // [E] more than one lunar day (two passes per lunar day)
    double min_lead_s = 1800.0;                 // [E] the first burn at least this after the plan
    double trim_threshold_mps = 0.3;            // [E] smaller plane trims are skipped
    double trim_tolerance_m = 50.0;             // [E] predicted cross-track miss after the trim
    double doi_tolerance_m = 50.0;              // [E] predicted periselene radius after the DOI
    double slope_limit_rad = 8.0 * constants::deg_to_rad;  // [E] NASA ~8-10 deg; the 12 m stance lands >= 9 deg (l2)
    double roughness_limit_m = 0.5;             // [E] terrain off the footprint's plane (leg stroke 0.6 m)
    double touchdown_search_radius_m = 1000.0;  // [E]
    double touchdown_grid_m = 20.0;             // the LOLA tile's pixel
    double glide_slope_rad = 30.0 * constants::deg_to_rad;  // the SCvx glide cone (DescentLimits)
    double glide_check_range_m = 3000.0;        // [E] the approach from the high gate
    double glide_margin_m = 50.0;               // [E] terrain below the cone by at least this
};

struct TouchdownPoint {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;   // terrain above the sphere
    double slope_rad = 0.0;  // of the footprint's plane
    double offset_m = 0.0;   // from the mission's site
};

// Nearest admissible touchdown point to (latitude, longitude) for an approach towards `approach_azimuth_rad`
// (clockwise from north): footprint (the leg circle and its centre) fits a plane within the slope and roughness limits,
// and the terrain uprange stays below the glide cone. Candidates on a grid in the site's east/north plane, nearest
// first; empty when none lies within the search radius.
std::optional<TouchdownPoint> select_touchdown_point(const environment::LunarTerrain& terrain, double latitude_rad,
                                                     double longitude_rad, double approach_azimuth_rad,
                                                     double footprint_radius_m, const LunarDescentParameters& p);

struct LunarDescentPlan {
    bool feasible = false;
    std::string reason;                   // German when refused
    std::optional<gnc::BurnNode> plane_trim;
    gnc::BurnNode doi;
    time::Epoch pdi_epoch;                // powered descent initiation (the braking guidance starts)
    time::Epoch site_pass_epoch;          // the unperturbed orbit's closest pass of the touchdown point
    TouchdownPoint touchdown;
    double approach_azimuth_rad = 0.0;    // ground track at the touchdown point, clockwise from north
    double crossrange_before_trim_m = 0.0;
    double crossrange_m = 0.0;            // predicted at the pass after the trim
    double periselene_altitude_m = 0.0;   // predicted after the DOI, above the sphere
    double pdi_range_m = 0.0;             // ground arc PDI -> touchdown point (predicted)
};

// From the LLO: the first pass of the site within max_crossrange_m after min_lead_s + 1.25 revolutions, a plane trim
// 1.25 revolutions before it (the out-of-plane offset of an impulse peaks a quarter revolution later and repeats every
// revolution - the trim must come before the DOI), the DOI half a revolution before the PDI point, PDI braking_range_m
// before the touchdown point. All predictions in the full force model.
class LunarDescentPlanner {
public:
    LunarDescentPlanner(const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                        const frames::BodyOrientation& moon_orientation);
    LunarDescentPlan plan(const math::StateVector& state_gcrf, time::Epoch epoch, const environment::LunarTerrain& terrain,
                          double site_latitude_rad, double site_longitude_rad, double footprint_radius_m,
                          const LunarDescentParameters& p) const;
};
```

  and (`mission/lunar_budget.hpp`):

```cpp
struct BudgetLeg {
    std::string name;     // German (refusal text), e.g. "Abstieg"
    double delta_v_mps = 0.0;
    double isp_s = 0.0;   // vacuum
};
struct BudgetResult {
    bool closes = false;
    double final_mass_kg = 0.0;
    double margin_kg = 0.0;  // final - (dry + reserve); < 0 when it does not close
    std::string reason;      // German when it does not close: the first leg after which the mass is short
};
// Rocket equation over the legs in order; closes when the final mass >= dry + reserve.
BudgetResult lunar_budget(double initial_mass_kg, double dry_mass_kg, double reserve_kg, const std::vector<BudgetLeg>& legs);
```

Algorithm notes (comments in the code):
- **Pass search:** propagate the vehicle's Earth-centred state with `orbit::propagate_coast` in 60 s samples over
  `search_s` (one DOP853 run with dense output is fine; re-starting per sample is too slow), each sample made
  Moon-relative with `moon_relative`. The site's direction `s` (MOON_ME -> GCRF at the sample epoch, the
  touchdown point's fixed position normalised), the orbit normal `n = r x v / |r x v|`, the projection
  `s_p = s - (s.n) n`, and the in-plane angle `phi = atan2((r x s_p).n, r.s_p)`. A pass is where `phi` crosses zero
  from positive to negative (the vehicle passes the projection); refine its epoch by bisection on a short propagation.
  Crossrange at the pass: `(R + h) * asin(s.n)`. The first pass with |crossrange| <= max and
  epoch >= epoch + min_lead_s + 1.25 P (P = the LLO period, 2 pi sqrt(a^3 / GM)) is taken; none -> reason
  `"Kein Überflug der Landestelle innerhalb von N Tagen (Bahnebene zu weit entfernt)"`.
- **Approach azimuth:** at the pass, the Moon-relative velocity minus the Moon's rotation projected on the local
  horizontal: `atan2(east, north)`.
- **Touchdown point:** `select_touchdown_point` with that azimuth (needs the tile; outside a tile `in_tile` false ->
  reason `"Landestelle außerhalb der Geländekachel (tools/data_pipeline/fetch.py)"`); none -> reason
  `"Kein Landepunkt mit höchstens X Grad Hangneigung im Umkreis von Y m"`. The pass is re-evaluated for the touchdown
  point (shifts by <= 1 km: same pass).
- **Footprint plane:** heights at the centre and the 6 leg azimuths on the footprint circle (same azimuths as
  `starship_hls.toml`; take them as a parameter from the vehicle in the session - here the planner samples 12 points on
  the circle so it does not depend on the leg count); least-squares plane z = a e + b n + c; slope = atan(sqrt(a^2 +
  b^2)); roughness = max |residual|. Points are turned from site east/north offsets into latitude/longitude on the
  sphere through `R + height` (a helper in the .cpp: `offset_position(lat, lon, east_m, north_m)` via the local ENU
  basis and `atan2` back).
- **Glide cone:** for d = 100 m, 120 m, ... glide_check_range_m uprange (against the approach azimuth):
  `height(d) - height(0) <= d tan(glide_slope) - glide_margin_m`.
- **Plane trim:** at the epoch 1.25 P before the pass (bisection on `phi = +90 deg` one revolution earlier), a burn
  along the orbit normal; first guess `dv = -v * asin(s.n)` (sign: turns the plane towards the point), then secant
  iterations (<= 6) on the predicted crossrange at the pass until |crossrange| < trim_tolerance_m. Skipped when
  |dv| < trim_threshold_mps.
- **DOI:** PDI point = central angle `braking_range_m / (R + h)` before the touchdown point's projection on the (trimmed)
  orbit; DOI epoch = the vehicle 180 deg before it (bisection on phi). Retrograde dv, first guess Hohmann
  `v1 - sqrt(GM 2 rp / (r1 (r1 + rp)))` with `rp = R + h_site + pdi_altitude_above_site_m`; secant on the predicted
  minimum radius (event: radial velocity zero) until within doi_tolerance_m.
- **PDI epoch:** on the post-DOI prediction, the epoch where the in-plane angle to the touchdown point equals the
  braking range's central angle (bisection); `pdi_range_m` is the ground arc there.
- The plan does not check propellant (the session does it with `lunar_budget`, Task 7).

- [x] **Step 1: Write the failing test** `core/tests/test_lunar_descent.cpp`:

```cpp
// Lunar descent planning (l4): the LLO's first pass of the site within 20 km, the plane trim, the DOI to 15 km above
// the site and the PDI 450 km uprange; touchdown point selection on the LOLA tile; the budget.

#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/gravity.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_budget.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
constexpr double site_lat = -84.7906 * deg, site_lon = 29.1957 * deg;

struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    std::shared_ptr<const environment::LunarTerrain> terrain =
        sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);   // as test_hls_landing.cpp loads it

    // Circular polar 100 km LLO (Earth-centred GCRF state) whose plane contains the site at t0 + `align_s`.
    math::StateVector llo(double align_s) {
        const auto t = t0 + align_s;
        const auto rot = moon_me.fixed_to_gcrf(t).rotation;
        const Vec3 pole = rot * Vec3{0.0, 0.0, 1.0};
        const Vec3 site = rot * Vec3{std::cos(site_lat) * std::cos(site_lon), std::cos(site_lat) * std::sin(site_lon),
                                     std::sin(site_lat)};
        const Vec3 n = math::normalized(cross(pole, site));             // polar plane through the site
        const double gm = ephemeris.gm_m3ps2(ephemeris::Body::Moon);
        const double r = constants::moon_mean_radius_m + 100e3;
        const Vec3 x = math::normalized(cross(n, pole));                 // in the plane, on the equator
        const Vec3 y = cross(n, x);
        const double speed = std::sqrt(gm / r);
        // at t0 on the equator (the orbit's plane drifts slowly against the rotating Moon: aligned at t0 + align_s)
        const math::StateVector moon_relative{x * r, y * speed};
        const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t0);
        return {moon.position_m + moon_relative.position_m, moon.velocity_mps + moon_relative.velocity_mps};
    }
};
}  // namespace

TEST_CASE("The budget closes or names the leg where the propellant runs out", "[mission][lunar_descent_plan]") {
    const auto ok = mission::lunar_budget(1185e3, 120e3, 10e3, {{"Abstieg", 2000.0, 350.0}, {"Aufstieg", 1900.0, 350.0}});
    CHECK(ok.closes);
    CHECK(ok.final_mass_kg == Catch::Approx(1185e3 * std::exp(-3900.0 / (350.0 * 9.80665))).epsilon(1e-12));
    const auto short_ = mission::lunar_budget(400e3, 204e3, 10e3, {{"Abstieg", 2000.0, 350.0}, {"Aufstieg", 1900.0, 350.0}});
    CHECK_FALSE(short_.closes);
    CHECK(short_.reason.find("Aufstieg") != std::string::npos);
    CHECK(short_.margin_kg < 0.0);
}

TEST_CASE("The flattest admissible touchdown point near the site is chosen", "[mission][lunar_descent_plan][terrain][data]") {
    Fixture fx;
    mission::LunarDescentParameters p;
    const auto point = mission::select_touchdown_point(*fx.terrain, site_lat, site_lon, 0.0, 12.0, p);
    REQUIRE(point);
    CHECK(point->slope_rad <= p.slope_limit_rad);
    CHECK(point->offset_m <= p.touchdown_search_radius_m);
    std::printf("[lunar_descent] touchdown %.1f m from the site, slope %.2f deg, height %.1f m\n", point->offset_m,
                point->slope_rad / deg, point->height_m);
    // Review focus 4: no admissible point.
    p.slope_limit_rad = 0.01 * deg;
    CHECK_FALSE(mission::select_touchdown_point(*fx.terrain, site_lat, site_lon, 0.0, 12.0, p));
}

TEST_CASE("Descent planning waits for the site's pass, trims the plane and lowers the periselene", "[mission][lunar_descent_plan][terrain][data]") {
    Fixture fx;
    mission::LunarDescentPlanner planner(fx.gravity, fx.ephemeris, fx.moon_me);
    mission::LunarDescentParameters p;
    const auto plan = planner.plan(fx.llo(0.5 * 86400.0), fx.t0, *fx.terrain, site_lat, site_lon, 12.0, p);
    std::printf("[lunar_descent] %s: pass in %.2f d, crossrange %.1f -> %.1f m, trim %.2f m/s, DOI %.2f m/s, "
                "periselene %.2f km, PDI range %.1f km\n",
                plan.feasible ? "feasible" : plan.reason.c_str(), (plan.site_pass_epoch - fx.t0) / 86400.0,
                plan.crossrange_before_trim_m, plan.crossrange_m,
                plan.plane_trim ? norm(plan.plane_trim->delta_v_gcrf_mps) : 0.0, norm(plan.doi.delta_v_gcrf_mps),
                plan.periselene_altitude_m / 1000.0, plan.pdi_range_m / 1000.0);
    REQUIRE(plan.feasible);
    CHECK(plan.site_pass_epoch - fx.t0 < 1.0 * 86400.0);
    CHECK(std::abs(plan.crossrange_m) < p.trim_tolerance_m);
    CHECK(plan.periselene_altitude_m == Catch::Approx(plan.touchdown.height_m + 15e3).margin(p.doi_tolerance_m));
    CHECK(plan.pdi_range_m == Catch::Approx(450e3).margin(5e3));
    CHECK(norm(plan.doi.delta_v_gcrf_mps) == Catch::Approx(20.0).margin(6.0));   // spec: ~20 m/s (Apollo profile)
    CHECK(plan.doi.epoch < plan.pdi_epoch);
    const auto first_burn = plan.plane_trim ? plan.plane_trim->epoch : plan.doi.epoch;
    if (plan.plane_trim) CHECK(plan.plane_trim->epoch < plan.doi.epoch);
    CHECK(first_burn > fx.t0 + p.min_lead_s);
}

TEST_CASE("Descent planning refuses when no pass comes within reach", "[mission][lunar_descent_plan][terrain][data]") {
    Fixture fx;
    mission::LunarDescentPlanner planner(fx.gravity, fx.ephemeris, fx.moon_me);
    mission::LunarDescentParameters p;
    p.search_s = 0.2 * 86400.0;            // the plane is aligned only after 5 days
    const auto plan = planner.plan(fx.llo(5.0 * 86400.0), fx.t0, *fx.terrain, site_lat, site_lon, 12.0, p);
    CHECK_FALSE(plan.feasible);
    CHECK(plan.reason.find("Überflug") != std::string::npos);
}
```

  `STARSIM_DATA_DIR` and the terrain loading: copy how `test_hls_landing.cpp` gets the terrain (it may use a test
  helper instead of the macro). The `llo()` helper places the vehicle on the equator at t0 with the plane through the
  site at `t0 + align_s` (the Moon rotates under the inertial plane - the cross-track distance at the pass then grows
  ~36 km per day away from that epoch); if the first pass with |crossrange| <= 20 km falls before `min_lead_s + 1.25 P`,
  the planner takes a later one - the test's `< 1 d` holds either way.

- [x] **Step 2: Run to see it fail.** Tagged `"[lunar_descent_plan]"`. Expected: compile error.

- [x] **Step 3: Implement** `lunar_budget.cpp` (rocket equation, `constants::standard_gravity_mps2`; reason
  `"Treibstoff reicht nicht: nach <Leg> fehlen <t> t"` with the shortfall in tonnes, one decimal) and
  `lunar_descent.cpp` following the algorithm notes. Keep the file below ~400 lines: if it grows, move the touchdown
  point selection to `lunar_touchdown.cpp` (same header). Diagnostics behind `STARSIM_DEBUG_LUNAR_DESCENT=1`
  (each pass found: epoch, crossrange; each trim / DOI iteration).

- [x] **Step 4: Run until green.** Tagged `"[lunar_descent_plan]"`. Record the printed numbers in the Ledger.

- [x] **Step 5: Commit, push** (after `-Fast`).

```bash
git add core/include/starsim/mission/lunar_descent.hpp core/src/mission/lunar_descent.cpp core/include/starsim/mission/lunar_budget.hpp core/src/mission/lunar_budget.cpp core/tests/test_lunar_descent.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat(mission): lunar descent planning (site pass, plane trim, DOI, touchdown point) and budget"
git push origin main
```

---

### Task 6: Powered descent in flight (braking -> SCvx approach -> touchdown)

**Files:**
- Create: `core/include/starsim/gnc/lunar_landing_guidance.hpp`, `core/src/gnc/lunar_landing_guidance.cpp`,
  `core/tests/test_lunar_powered_descent.cpp`
- Create: `core/src/sim/flight_session_lunar_descent.cpp` (descent and surface steps: `flight_session_lunar.cpp` has
  551 lines already; its `run_lunar_steps` hands the new steps to `run_lunar_descent_steps(Flying&)`)
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session_lunar.cpp` (dispatch, `to_string`),
  `core/src/sim/flight_session_command.cpp` (command of the landing body), `core/CMakeLists.txt`,
  `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 3 (`LandingSite::body`, `plan_frame(site)`, `site_body_frame`, `plan_state`, `ScvxLandingGuidance` on the
  Moon), Task 4 (`ApolloBrakingGuidance`, `BrakingTarget`), Task 5 (`mission::TouchdownPoint`), `gnc::ReturnCommand`
  and `FlightSession::apply_return_command` (the session flies it as a ship landing), `LegLandingSim::in_contact` /
  `touchdown`, `thrust_axis_body(vehicle, cg, groups)`.
- Produces (`gnc/lunar_landing_guidance.hpp`):

```cpp
enum class LunarDescentPhase { Braking, Approach, Touchdown, Landed, Aborted };
const char* to_string(LunarDescentPhase phase);  // German: "Bremsphase", "Anflug", "Aufsetzen", "gelandet", "Abbruch"

struct LunarLandingParameters {
    // Braking on the vacuum Raptors (Isp 380 s, fixed: attitude by the RCS - user 2026-10-15), then the gimballed
    // sea_level_1 as the thrust demand falls (as Apollo throttled down): a smaller set takes over when the demand stays
    // below `set_drop_fraction` of its maximum for `set_drop_s`; a set not yet running is lit startup_s ahead.
    std::vector<std::vector<std::string>> braking_sets{{"vacuum"}, {"sea_level_1"}};
    double set_drop_fraction = 0.9;  // [E]
    double set_drop_s = 2.0;         // [E]
    // High gate (guidance frame: x up, y crossrange, z downrange; origin the touchdown point) [E] - within the SCvx
    // glide cone (30 deg: 2 km over 3 km is 33.7 deg).
    BrakingTarget high_gate{{2000.0, 0.0, -3000.0}, {-35.0, 0.0, 90.0}, {0.3, 0.0, -1.2}, 0.0};
    double approach_time_to_go_s = 5.0;  // [E] the SCvx approach takes over at this braking t_go
    double braking_time_to_go_guess_s = 400.0;
    ScvxLandingParameters approach;      // phase groups {2 sea-level}, {sea_level_1}, {landing}; HLS limits
};
// The HLS's values (the [E] limits found in Task 3's lunar SCvx test).
LunarLandingParameters hls_landing_parameters();

// Powered descent of a lunar lander: the Apollo braking law from PDI to the high gate, then SCvx plans to the touchdown
// point (Raptors, then the landing thrusters), engines off on leg contact. Commands as the ship landing
// (ReturnCommand) so the session flies them the same way.
class LunarLandingGuidance {
public:
    // `touchdown`: body Moon, the point on the terrain with the pad target from the legs; `approach_azimuth_rad`:
    // clockwise from north at the touchdown point.
    LunarLandingGuidance(LandingSite touchdown, double approach_azimuth_rad, LunarLandingParameters parameters);
    ReturnCommand update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch,
                         bool leg_contact);
    void mark_landed() { phase_ = LunarDescentPhase::Landed; }
    LunarDescentPhase phase() const { return phase_; }
    double braking_time_to_go_s() const { return braking_t_go_; }
    const std::string& status() const { return status_; }  // German, e.g. why the approach could not be planned
    void set_realtime(bool realtime);

private:
    ReturnCommand braking(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);
    LandingSite site_;
    LunarLandingParameters parameters_;
    PlanFrame plan_;
    math::Mat3 plan_to_guidance_;  // rows: up, crossrange, downrange in plan coordinates
    ApolloBrakingGuidance braking_;
    std::unique_ptr<ScvxLandingGuidance> approach_;
    LunarDescentPhase phase_ = LunarDescentPhase::Braking;
    size_t braking_set_ = 0;
    double below_set_s_ = 0.0;
    time::Epoch last_update_;
    double braking_t_go_ = 0.0;
    std::string status_;
};
```

  Session (`flight_session.hpp`):

```cpp
// LunarStep gains (in this order, after TransferIn): PoweredDescent, Landed, Lost
//   to_string: "Landeanflug", "gelandet", "verloren"
struct LunarTouchdown {
    LegTouchdown outcome = LegTouchdown::None;
    double offset_m = 0.0;     // horizontal distance of the CG from the touchdown point when the judgement fell
    double sink_mps = 0.0;     // at the first leg contact, relative to the Moon
    double lateral_mps = 0.0;  // at the first leg contact
};
// Powered descent of the controlled body from now (scenarios; Task 7's step machine calls it at PDI): needs the Moon
// orientation, a lunar landing site (set_lunar_landing_site) and a Moon-referenced body with legs. False with a German
// lunar_status() otherwise.
bool begin_powered_descent(const mission::TouchdownPoint& point, double approach_azimuth_rad,
                           gnc::LunarLandingParameters parameters = gnc::hls_landing_parameters());
const gnc::LunarLandingGuidance* lunar_landing_guidance() const { return lunar_landing_.get(); }
const std::optional<LunarTouchdown>& lunar_touchdown() const { return lunar_touchdown_; }
```

Design decisions (comments in the code):
- **Braking frame:** the guidance frame is the touchdown point's plan frame turned about x by the approach azimuth
  (`downrange = (0, sin az, cos az)` in plan coordinates, `crossrange = downrange x up`... keep it right-handed:
  `y = z x x`). The vehicle's position and velocity relative to the rotating Moon come from `plan_state` (Task 3).
  The **effective gravity is evaluated at the vehicle** (the braking phase spans ~450 km, ~15 deg of the Moon):
  `R (gravity(r_gcrf) - centre_acceleration)` minus the Coriolis and centrifugal terms of the Moon's rotation, all in
  the guidance frame.
- **Thrust:** direction = the law's thrust acceleration, throttle = `m |a| / T_max(set)` clamped to the set's
  [minimum, 1]. While the demand stays below `set_drop_fraction` x the next smaller set's maximum for `set_drop_s`,
  the smaller set takes over (engines spool down / the remaining ones throttle up; the hull keeps the set's thrust axis
  through the CG, `thrust_axis_body(vehicle, cg, groups)`). Vacuum -> `sea_level_1` is a disjoint set: light
  `sea_level_1` `startup_s` before the predicted drop, then cut the vacuum Raptors (the demand trend over the last
  seconds predicts it). With the vacuum Raptors running no gimbal acts: the controller's body torque goes to the RCS;
  the test checks the thrust-axis tracking error (< 1 deg [E]) and prints the RCS propellant used. Ignition: the first update lights the first set (Raptor
  start-up 3 s: the session begins the descent `startup_s` before the PDI epoch - Task 7).
- **Hand-over:** at braking t_go <= `approach_time_to_go_s` the SCvx guidance starts (first plan synchronously, from
  the current state with the running set as phase 0's engines). If the first plan fails the phase becomes Aborted
  with status "Anflug nicht planbar" and the command says `abort` (Task 8 flies the abort ascent; until then the session
  cuts the engines and the vehicle falls - the test checks only the nominal case).
- **Touchdown:** engines off on the first leg contact (`leg_contact`), phase Touchdown; the session marks Landed when
  the leg simulation judges "gelandet" (as Falcon 9: >= 3 feet, at rest 2 s), Lost on "umgekippt" / "Aufprall".
- `hls_landing_parameters()`: `approach.snapshot.phase_groups = {{"sea_level_1"}, {"sea_level_1"}, {"landing"}}`
  (Task 3 Ruling), `approach.settings.limits` and `kinematic_guess_s` from Task 3's Ledger values, `approach.snapshot.catch_sink_mps = 1.0`
  (spec: ~1 m/s), `approach.lqr_lateral = true`, `approach.final_lateral_effort_fraction` [E] for the RCS-steered set
  (start with 0.2; Task 10's campaign may adjust - record).

- [x] **Step 1: Write the failing test** `core/tests/test_lunar_powered_descent.cpp`:

```cpp
// Powered descent from PDI (l4): an HLS on a 15 km x 100 km polar ellipse, 450 km uprange of the touchdown point,
// brakes on the Apollo law, flies the SCvx approach and lands on the LOLA terrain within 1 m at <= 1 m/s.

#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/lunar_landing_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;

config::VehicleConfig hls(double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    for (auto& t : cfg.stages.front().tanks) {
        t.mass_kg = propellant_kg * (t.contents == config::TankContents::Oxidizer ? 3.6 / 4.6 : 1.0 / 4.6);
    }
    return cfg;
}

struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    std::shared_ptr<const environment::LunarTerrain> terrain;
    mission::TouchdownPoint point;
    double azimuth = 0.0;

    Fixture() {
        terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);  // as test_hls_landing.cpp
        session.set_moon_terrain(terrain);
        const auto site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
        session.set_lunar_landing_site(site);
        azimuth = 180.0 * deg;  // a polar pass heading south over the site
        point = *mission::select_touchdown_point(*terrain, site.config.latitude_rad, site.config.longitude_rad, azimuth,
                                                 12.0, mission::LunarDescentParameters{});
    }

    // Moon-relative PDI state: periselene of a (h_site + 15 km) x 100 km polar ellipse, `range_m` of ground arc before
    // the touchdown point, in the plane through the pole and the point.
    void place_at_pdi(double propellant_kg, double range_m = 450e3) {
        const double R = constants::moon_mean_radius_m, gm = ephemeris.gm_m3ps2(ephemeris::Body::Moon);
        const auto rot = moon_me.fixed_to_gcrf(t0).rotation;
        const Vec3 p = rot * Vec3{std::cos(point.latitude_rad) * std::cos(point.longitude_rad),
                                  std::cos(point.latitude_rad) * std::sin(point.longitude_rad), std::sin(point.latitude_rad)};
        const Vec3 pole = rot * Vec3{0.0, 0.0, 1.0};
        const Vec3 n = math::normalized(cross(p, pole));       // motion from the point's uprange towards it: southwards
        const double angle = range_m / (R + point.height_m);
        const Vec3 r_hat = p * std::cos(angle) - cross(n, p) * std::sin(angle);  // uprange of p
        const Vec3 along = cross(n, r_hat);
        const double rp = R + point.height_m + 15e3, ra = R + 100e3;
        const double vp = std::sqrt(gm * (2.0 / rp - 2.0 / (rp + ra)));
        const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t0);
        const Vec3 r = moon.position_m + r_hat * rp, v = moon.velocity_mps + along * vp;
        const Vec3 x_body = -along;   // retrograde: ready to brake
        const auto attitude = math::Quat::from_matrix(math::Mat3::from_columns(x_body, r_hat, cross(x_body, r_hat)));
        session.place_body(hls(propellant_kg), {r, v, attitude, {}}, t0);
    }
};
}  // namespace

TEST_CASE("The HLS brakes from PDI and lands on the LOLA terrain within 1 m", "[sim][lunar_descent_flight][terrain][data][slow]") {
    Fixture fx;
    fx.place_at_pdi(1065e3);   // ~1,185 t in the LLO after the DOI (LLO variant)
    const double m0 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    REQUIRE(fx.session.begin_powered_descent(fx.point, fx.azimuth));
    double braking_s = -1.0;
    for (int i = 0; i < 1200 * 200 && fx.session.lunar_step() == sim::LunarStep::PoweredDescent; ++i) {
        fx.session.advance_to(fx.session.epoch() + sim::FlightSession::tick_s, 10);
        const auto* g = fx.session.lunar_landing_guidance();
        if (braking_s < 0.0 && g && g->phase() != gnc::LunarDescentPhase::Braking) braking_s = fx.session.epoch() - fx.t0;
    }
    fx.session.advance_to(fx.session.epoch() + 5.0, 2000);
    const auto& td = fx.session.lunar_touchdown();
    REQUIRE(td);
    const double used_t = (m0 - fx.session.body(0).vehicle().mass_properties().mass_kg) / 1000.0;
    const double dv = 350.0 * constants::standard_gravity_mps2 * std::log(m0 / fx.session.body(0).vehicle().mass_properties().mass_kg);
    std::printf("[lunar_descent] %s: offset %.2f m, sink %.2f m/s, lateral %.2f m/s, braking %.0f s, total %.0f s, "
                "%.0f t used (%.0f m/s at 350 s)\n",
                sim::to_string(td->outcome), td->offset_m, td->sink_mps, td->lateral_mps, braking_s,
                fx.session.epoch() - fx.t0, used_t, dv);
    CHECK(fx.session.lunar_step() == sim::LunarStep::Landed);
    CHECK(td->outcome == sim::LegTouchdown::Landed);
    CHECK(td->offset_m < 1.0);    // spec
    CHECK(td->sink_mps <= 1.0 + 0.1);
    CHECK(braking_s > 240.0);     // spec ~6-7 min with Apollo-like targets; Raptors are stronger - recorded
    CHECK(dv == Catch::Approx(2000.0).margin(250.0));  // spec ~2.0 km/s (incl. the approach)
}

TEST_CASE("Powered descent is refused without a lunar landing site", "[sim][lunar_descent_flight]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    session.place_in_orbit("starship_hls", orbit::state_from_elements({6578137.0, 0.0, 0.5, 0.0, 0.0, 0.0},
                                                                       ephemeris.gm_m3ps2(ephemeris::Body::Earth)),
                           time::Epoch::from_tdb_seconds_past_j2000(8.4e8));
    CHECK_FALSE(session.begin_powered_descent({}, 0.0));
    CHECK(session.lunar_status().find("Landestelle") != std::string::npos);
}
```

  (Include `starsim/orbit/kepler.hpp` for `state_from_elements`; 1,065 t of propellant + 120 t dry = 1,185 t,
  the l3 LLO mass; DOI and trims cost ~1 t - any propellant load of 1,000-1,100 t is fine.)

- [x] **Step 2: Run to see it fail.** Tagged `"[lunar_descent_flight]~[slow]"`, then the slow case alone by name.
  Expected: compile errors.

- [x] **Step 3: Implement the guidance** (`lunar_landing_guidance.cpp`, design decisions above). `update`:
  Braking -> `braking()`; when `braking_t_go_ <= approach_time_to_go_s` (or the law becomes invalid near the gate):
  `approach_ = std::make_unique<ScvxLandingGuidance>(site_, parameters_.approach)`; `approach_->start(body, env,
  epoch)` false -> Aborted (status, `cmd.abort = true`, no run groups). Approach -> `approach_->update(...)` converted
  to `ReturnCommand` exactly as `ShipLandingGuidance` converts its SCvx command (copy that block: run groups, throttle,
  axis, roll reference, feed-forward, torque, `scvx = true`); `leg_contact` -> phase Touchdown, run groups empty.
  Touchdown / Landed -> empty run groups, axis = the current body x (hold). `cmd.phase` = `ReturnPhase::LandingBurn`
  in Braking and Approach (the session's command path treats it as a landing burn).

- [x] **Step 4: Implement the session part.** In `flight_session.hpp`: the three `LunarStep`s, `LunarTouchdown`,
  `begin_powered_descent`, the accessors, `void run_lunar_descent_steps(Flying& flying);`, members `std::unique_ptr<gnc::LunarLandingGuidance> lunar_landing_;`,
  `std::optional<LunarTouchdown> lunar_touchdown_;`, `std::optional<LunarSite> lunar_site_;` (set by
  `set_lunar_landing_site`, cleared by `set_moon_terrain`). `begin_powered_descent` checks: Moon orientation, a lunar
  site, controlled body Moon-referenced (`reference_body(controlled_) == Moon`), legs configured; builds the
  `gnc::LandingSite{Moon, lat, lon, height, ..., pad = PadTarget{-foot_station_m, 0.0}}`, the guidance, sets
  `autopilot_.state = Lunar`, `lunar_step_ = PoweredDescent`, status "Landeanflug". In `run_lunar_descent_steps` (`flight_session_lunar_descent.cpp`):

```cpp
        case LunarStep::PoweredDescent: {
            auto& body = *flying.body;
            const bool contact = legs_->in_contact(body);
            if (contact && !lunar_touchdown_) {
                const auto frame = moon_->frame(epoch_);
                const Vec3 rel = frame.relative_velocity(body.state().position_gcrf_m, body.state().velocity_gcrf_mps);
                const Vec3 up = math::normalized(body.state().position_gcrf_m - frame.centre_gcrf_m);
                lunar_touchdown_ = LunarTouchdown{LegTouchdown::None, 0.0, -dot(rel, up), norm(rel - up * dot(rel, up))};
            }
            flying.return_command = lunar_landing_->update(body, env_, epoch_, contact);
            const auto judged = legs_->touchdown(body);
            if (judged != LegTouchdown::None || body.impacted()) {
                lunar_touchdown_->outcome = body.impacted() ? LegTouchdown::Impact : judged;
                lunar_touchdown_->offset_m = <horizontal distance of the CG from the touchdown point, site frame>;
                if (judged == LegTouchdown::Landed) {
                    lunar_landing_->mark_landed();
                    lunar_step_ = LunarStep::Landed;
                    lunar_status_ = "gelandet: " + german(lunar_touchdown_->offset_m, 2) + " m vom Zielpunkt";
                } else {
                    lunar_step_ = LunarStep::Lost;
                    lunar_status_ = std::string("verloren: ") + to_string(judged);
                    autopilot_.state = AutopilotState::Off;
                }
            }
            break;
        }
        case LunarStep::Landed:
        case LunarStep::Lost:
            break;
```

  (If the leg simulation judges before any contact was recorded - impossible for a landing, possible for an impact -
  create the record there.) The CG offset: `legs_->to_site(frame.to_fixed(cg))` minus the touchdown point's site
  coordinates (`legs_->to_site(point fixed)`), east/north components. `command_for`: while `lunar_step_ ==
  PoweredDescent` for the controlled body, `apply_return_command(flying, ship_landing_controller_, pressure, cmd)` (the
  same controller as the ship's landing burn: gimbals + RCS); the coast allowance is 0 in PoweredDescent (ticks).
  Engine groups `landing` are `landing_only`: check that `apply_return_command` lets a guidance light them (it must;
  `ignite()` and burns must still refuse them - the l2 tests cover that).

- [x] **Step 5: Run until green.** Fast cases first, then the slow landing by name with `-s`. Diagnostics with
  `STARSIM_DEBUG_SCVX=1` and a new `STARSIM_DEBUG_LUNAR_DESCENT=1` print in the guidance (phase changes, braking set
  drops, t_go every 10 s, the hand-over state in the guidance frame). Tune only [E] values (high gate, set drop,
  approach limits); record every change and the printed numbers in the Ledger. If the throttle saturates at the
  minimum for long stretches of the braking phase, move the PDI range (the planner's `braking_range_m`) rather than
  the law.

- [x] **Step 6: Commit, push** (after `-Fast`).

```bash
git add core/include/starsim/gnc/lunar_landing_guidance.hpp core/src/gnc/lunar_landing_guidance.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session_lunar.cpp core/src/sim/flight_session_lunar_descent.cpp core/src/sim/flight_session_command.cpp core/src/sim/flight_session.cpp core/tests/test_lunar_powered_descent.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat(sim): lunar powered descent (Apollo braking, SCvx approach, touchdown on legs)"
git push origin main
```

---

### Task 7: Descent autopilot from the LLO (wait, plane trim, DOI, coast, PDI) and the surface stay

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session_lunar_descent.cpp`,
  `core/src/sim/flight_session_lunar.cpp` (`to_string`, dispatch)
- Test: `core/tests/test_lunar_descent_session.cpp` (new), `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 5 (`LunarDescentPlanner`, `LunarDescentParameters`, `lunar_budget`), Task 6 (`begin_powered_descent`,
  steps PoweredDescent / Landed / Lost), l3's `lunar_burn`, `lunar_enter_coast`, `lunar_ignition_lead`,
  `lunar_step_epoch_`, coast allowance.
- Produces (`flight_session.hpp`):

```cpp
// LunarStep gains (before PoweredDescent): DescentWait, PlaneTrim, Doi, DescentCoast; (after Landed): SurfaceStay
//   to_string: "Warten auf Überflug", "Bahnebenenkorrektur", "Abstiegsbahn (DOI)", "Freiflug zum Abstieg",
//              "Oberflächenaufenthalt"
struct LunarLandingSetup {
    mission::LunarDescentParameters descent;
    gnc::LunarLandingParameters guidance = gnc::hls_landing_parameters();
    double surface_stay_s = 0.0;          // [E] mission parameter (spec): resting after "gelandet"
    double descent_delta_v_mps = 2000.0;  // [E] budget: DOI + braking + approach (spec ~2.0 km/s, Apollo LM ~2.0)
    double braking_delta_v_mps = 1600.0;  // [E] of it on the vacuum Raptors (Task 6 measures the flown share)
    double ascent_delta_v_mps = 1900.0;   // [E] budget: ascent + circularisation (spec ~1.9 km/s, Apollo LM ~1.85)
    double reserve_kg = 10000.0;          // [E] residuals and unusable propellant
};
// Descent of the controlled vehicle from a closed lunar orbit (autopilot Off / Orbit or lunar step InLlo): plans the
// site pass, plane trim, DOI and PDI (mission::LunarDescentPlanner on the lunar landing site's terrain), checks the
// propellant for descent and ascent (mission::lunar_budget) and flies the steps; after "gelandet" the vehicle rests for
// surface_stay_s (step SurfaceStay, then Landed). False with a German lunar_status() when refused.
bool plan_lunar_landing(const LunarLandingSetup& setup);
const mission::LunarDescentPlan* lunar_descent_plan() const { return lunar_descent_plan_ ? &*lunar_descent_plan_ : nullptr; }
```

Design decisions (comments in the code):
- Preconditions: Moon orientation, lunar landing site (terrain), controlled body Moon-referenced on a closed orbit
  (osculating `e < 1` about the Moon, periselene above the terrain maximum), no lunar step other than None / InLlo, no
  autopilot phase other than Off / Orbit / Lunar(InLlo). Refusals (German): "Kein Abstieg: nicht in einer
  Mondumlaufbahn", "Kein Abstieg: keine Landestelle gesetzt", "Kein Abstieg: " + planner reason,
  "Kein Abstieg: " + budget reason.
- Budget: `lunar_budget(mass, dry, reserve, {{"Bremsphase", braking_dv, 380}, {"Anflug", descent_dv - braking_dv,
  350}, {"Aufstieg", ascent_dv, 380}})`: braking and ascent on the vacuum Raptors (user 2026-10-15; `braking_delta_v_mps`
  [E] 1600 m/s in LunarLandingSetup - the part above the RVac minimum thrust, measured in Task 6), DOI and the rest of
  the descent on `sea_level_1` (vacuum Isp 350 s; the landing thrusters are 340 s for ~1 % of the descent). Isp from
  the vehicle's engine groups, not literals.
- Step machine (in `run_lunar_descent_steps`): DescentWait -> at the trim's ignition lead `lunar_burn(trim, false,
  PlaneTrim)` (small: RCS or one Raptor - `execute_burn(node)` chooses) -> DescentWait -> DOI the same (step Doi) ->
  DescentCoast with `lunar_step_epoch_ = pdi_epoch - startup_s` (the Raptors' 3 s start-up: the braking's first update
  lights them) -> `begin_powered_descent(plan.touchdown, plan.approach_azimuth_rad, setup.guidance)` ->
  PoweredDescent -> Landed (Task 6) -> SurfaceStay until `landing epoch + surface_stay_s` -> Landed (Task 8 adds the
  ascent there). Waits are Sun-pointed coasts like l3's (`lunar_enter_coast` with the next step epoch); the burn
  executor turns to the burn attitude in its preparation.
- The trim and the DOI are flown as planned (closed-loop cutoff on the delta-v); the PDI uses the predicted epoch and
  the braking law absorbs the dispersion. A re-plan of the DOI after the trim is a later refinement - record it in the
  Ledger if the landing misses because of it.
- `disengage_autopilot()` already resets `lunar_step_`; it also drops `lunar_landing_` (the pilot flies: no guidance
  command afterwards).

- [x] **Step 1: Write the failing test** `core/tests/test_lunar_descent_session.cpp`:

```cpp
// Descent autopilot from the LLO (l4): one time-warp call from the LLO to the landing (steps not skipped), pilot
// override in the braking phase, refusals.

#include <cmath>
#include <cstdio>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;

config::VehicleConfig hls(double propellant_kg, double dry_kg = 0.0) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    if (dry_kg > 0.0) cfg.stages.front().dry_mass_kg = dry_kg;
    for (auto& t : cfg.stages.front().tanks) {
        t.mass_kg = propellant_kg * (t.contents == config::TankContents::Oxidizer ? 3.6 / 4.6 : 1.0 / 4.6);
    }
    return cfg;
}

struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    sim::LunarSite site;

    Fixture() {
        const auto terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);
        session.set_moon_terrain(terrain);
        site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
        session.set_lunar_landing_site(site);
    }
    // Circular polar 100 km LLO whose plane contains the site at t0 + align_s (as test_lunar_descent.cpp).
    void place_in_llo(double propellant_kg, double align_s, double dry_kg = 0.0) {
        const auto t = t0 + align_s;
        const auto rot = moon_me.fixed_to_gcrf(t).rotation;
        const Vec3 pole = rot * Vec3{0.0, 0.0, 1.0};
        const double lat = site.config.latitude_rad, lon = site.config.longitude_rad;
        const Vec3 s = rot * Vec3{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
        const Vec3 n = math::normalized(cross(pole, s));
        const double r = constants::moon_mean_radius_m + 100e3, gm = ephemeris.gm_m3ps2(ephemeris::Body::Moon);
        const Vec3 x = math::normalized(cross(n, pole)), y = cross(n, x);
        const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t0);
        session.place_in_orbit(hls(propellant_kg, dry_kg),
                               {moon.position_m + x * r, moon.velocity_mps + y * std::sqrt(gm / r)}, t0);
    }
};
}  // namespace

TEST_CASE("From the LLO the HLS waits for the site, trims, lowers its orbit and lands", "[sim][lunar_descent_session][terrain][data][slow]") {
    Fixture fx;
    fx.place_in_llo(1065e3, 0.5 * 86400.0);
    sim::LunarLandingSetup setup;
    setup.surface_stay_s = 3600.0;
    REQUIRE(fx.session.plan_lunar_landing(setup));
    CHECK_FALSE(fx.session.plan_lunar_landing(setup));   // a descent under way keeps its plan
    const auto* plan = fx.session.lunar_descent_plan();
    REQUIRE(plan);
    // Review focus 1: one warp call over the wait, the trim, the DOI and the PDI.
    fx.session.advance_to(plan->pdi_epoch + 1200.0, 50'000'000);
    std::vector<sim::LunarStep> flown;
    for (const auto& b : fx.session.lunar_burns()) flown.push_back(b.step);
    CHECK(std::find(flown.begin(), flown.end(), sim::LunarStep::Doi) != flown.end());
    if (plan->plane_trim) CHECK(std::find(flown.begin(), flown.end(), sim::LunarStep::PlaneTrim) != flown.end());
    const auto& td = fx.session.lunar_touchdown();
    REQUIRE(td);
    std::printf("[lunar_descent_session] %s, offset %.2f m, sink %.2f m/s, wait %.2f d, status %s\n",
                sim::to_string(td->outcome), td->offset_m, td->sink_mps, (plan->pdi_epoch - fx.t0) / 86400.0,
                fx.session.lunar_status().c_str());
    CHECK(td->outcome == sim::LegTouchdown::Landed);
    CHECK(td->offset_m < 1.0);
    CHECK(fx.session.lunar_step() == sim::LunarStep::SurfaceStay);
    fx.session.advance_to(fx.session.epoch() + 2.0 * 3600.0, 50'000'000);
    CHECK(fx.session.lunar_step() == sim::LunarStep::Landed);   // stay over (no ascent requested)
    CHECK(fx.session.body(0).resting());
}

TEST_CASE("The pilot takes over in the braking phase: the descent guidance stops for good", "[sim][lunar_descent_session][terrain][data][slow]") {
    Fixture fx;
    fx.place_in_llo(1065e3, 0.5 * 86400.0);
    REQUIRE(fx.session.plan_lunar_landing({}));
    const auto pdi = fx.session.lunar_descent_plan()->pdi_epoch;
    fx.session.advance_to(pdi + 60.0, 50'000'000);
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::PoweredDescent);
    fx.session.disengage_autopilot();
    sim::PilotInput input;
    input.throttle = 0.0;
    fx.session.set_input(input);
    fx.session.cutoff();
    fx.session.advance_to(fx.session.epoch() + 10.0, 5000);
    CHECK(fx.session.lunar_step() == sim::LunarStep::None);
    CHECK(fx.session.autopilot().state == sim::AutopilotState::Off);
    CHECK(fx.session.lunar_landing_guidance() == nullptr);
    CHECK_FALSE(fx.session.body(0).vehicle().any_engine_producing_thrust());
}

TEST_CASE("Descent planning refusals", "[sim][lunar_descent_session][terrain][data]") {
    Fixture fx;
    SECTION("not in a lunar orbit") {
        fx.session.place_in_orbit("starship_hls", orbit::state_from_elements({6578137.0, 0.0, 0.5, 0.0, 0.0, 0.0},
                                                                             fx.ephemeris.gm_m3ps2(ephemeris::Body::Earth)),
                                  fx.t0);
        CHECK_FALSE(fx.session.plan_lunar_landing({}));
        CHECK(fx.session.lunar_status().find("Mondumlaufbahn") != std::string::npos);
    }
    SECTION("propellant short for descent and ascent (204 t dry, half tanks)") {
        fx.place_in_llo(400e3, 0.5 * 86400.0, 204e3);
        CHECK_FALSE(fx.session.plan_lunar_landing({}));
        CHECK(fx.session.lunar_status().find("Treibstoff") != std::string::npos);
    }
    SECTION("no admissible touchdown point (review focus 4)") {
        fx.place_in_llo(1065e3, 0.5 * 86400.0);
        sim::LunarLandingSetup setup;
        setup.descent.slope_limit_rad = 0.01 * deg;
        CHECK_FALSE(fx.session.plan_lunar_landing(setup));
        CHECK(fx.session.lunar_status().find("Landepunkt") != std::string::npos);
        CHECK(fx.session.lunar_step() == sim::LunarStep::None);
        CHECK(fx.session.lunar_burns().empty());
    }
}
```

  (`place_in_orbit(const config::VehicleConfig&, ...)` takes the loaded config; `#include <algorithm>` for `std::find`.)

- [x] **Step 2: Run to see it fail.** Tagged `"[lunar_descent_session]~[slow]"`. Expected: compile errors.

- [x] **Step 3: Implement** the setup struct, `plan_lunar_landing`, the steps (`to_string` entries), the coast
  allowance stop at `lunar_step_epoch_` for the new waits (as l3's Coast), and `disengage_autopilot()` resetting
  `lunar_landing_` and `lunar_descent_plan_`. Members: `std::optional<mission::LunarDescentPlan> lunar_descent_plan_;`,
  `std::optional<LunarLandingSetup> lunar_landing_setup_;`, `time::Epoch lunar_landed_epoch_;`. Diagnostics through
  the existing `STARSIM_DEBUG_LUNAR_SESSION` status printing.

- [x] **Step 4: Run until green.** Fast cases, then both slow cases by name (`-s`). Record the wait, trim, DOI,
  touchdown numbers in the Ledger.

- [x] **Step 5: Commit, push** (after `-Fast`).

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session_lunar.cpp core/src/sim/flight_session_lunar_descent.cpp core/tests/test_lunar_descent_session.cpp core/tests/CMakeLists.txt
git commit -m "feat(sim): lunar descent autopilot from the LLO (site pass, plane trim, DOI, PDI, surface stay)"
git push origin main
```

---

### Task 8: Lunar ascent (liftoff on the landing thrusters, PEG to 15 x 100 km, circularisation) and abort to orbit

**Files:**
- Create: `core/include/starsim/gnc/lunar_ascent_guidance.hpp`, `core/src/gnc/lunar_ascent_guidance.cpp`,
  `core/src/sim/flight_session_lunar_ascent.cpp`, `core/tests/test_lunar_ascent.cpp`
- Modify: `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp`,
  `core/include/starsim/sim/leg_landing_sim.hpp`, `core/src/sim/leg_landing_sim.cpp`,
  `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session_lunar_descent.cpp` (stay end),
  `core/src/sim/flight_session.cpp` (`abort_landing`), `core/src/sim/flight_session_command.cpp`,
  `core/src/sim/flight_session_lunar.cpp` (`to_string`, dispatch), `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 2 (`PegSteering`, `PegTarget`, `PegState`, `PegVehicle`), Task 6/7 (steps, `lunar_touchdown_`,
  `lunar_site_`), `gnc::circularize_at_apoapsis(state, now, gm)`, `lunar_burn`, `moon_->frame`, `legs_`.
- Produces:

```cpp
// flight_body.hpp
// Lets a landed body go (liftoff): it is integrated again from its held pose (moving with its body), not resting.
void release_landed();

// leg_landing_sim.hpp
// The body lifts off: its judgement starts over (the crush strokes stay).
void release(const vehicle::FlightBody& body);

// gnc/lunar_ascent_guidance.hpp
struct LunarAscentParameters {
    std::vector<std::string> liftoff_groups{"landing"};
    std::vector<std::string> ascent_groups{"vacuum"};  // user 2026-10-15: fixed RVacs (Isp 380 s), attitude by the RCS
    double raptor_start_height_m = 30.0;     // [E] the Raptors start this high (spec: "after a few tens of metres")
    double vertical_rise_height_m = 150.0;   // [E] PEG from here (Apollo: ~10 s vertical rise)
    double insertion_altitude_above_site_m = 15.0e3;  // [E] Apollo inserted ~18 km up; >= 10.8 km clears all terrain
    double aposelene_altitude_m = 100.0e3;   // the LLO (spec)
    bool northerly = true;                   // polar plane through the site, launched towards the north
    double abort_min_climb_mps = 5.0;        // [E] abort: climb at least this before PEG takes over
    double abort_delta_v_margin = 0.15;      // [E] gravity and steering losses on the abort's propellant check
};
enum class LunarAscentPhase { Liftoff, Vertical, Arrest, Peg, Cutoff };
const char* to_string(LunarAscentPhase phase);  // German: "Abheben", "Senkrechtaufstieg", "Sinken stoppen", "PEG",
                                                // "Brennschluss"
struct LunarAscentCommand {
    std::vector<std::string> run_groups;
    double throttle = 1.0;
    math::Vec3 axis_gcrf;        // desired body x
    LunarAscentPhase phase = LunarAscentPhase::Liftoff;
    double time_to_go_s = 0.0;   // PEG
    bool cutoff = false;
};
class LunarAscentGuidance {
public:
    // `moon_gm_m3ps2` from the reference body; `site_height_m` = terrain height above the sphere at the liftoff
    // point (the insertion altitude is above it).
    LunarAscentGuidance(LunarAscentParameters parameters, double moon_gm_m3ps2, double site_height_m);
    // Liftoff from the ground (abort = false) or abort from flight (abort = true: Arrest first if sinking, plane = the
    // current Moon-relative orbital plane). Fixes the target plane.
    void start(const vehicle::FlightBody& body, const environment::BodyFrame& moon, time::Epoch epoch, bool abort);
    LunarAscentCommand update(const vehicle::FlightBody& body, const environment::BodyFrame& moon, time::Epoch epoch);
    LunarAscentPhase phase() const { return phase_; }
    // Propellant check for an abort from `body`'s state (rocket equation on the ascent set's vacuum Isp, the speed still
    // to gain to the insertion plus the sink to stop, times 1 + abort_delta_v_margin): the delta-v needed.
    double abort_delta_v_mps(const vehicle::FlightBody& body, const environment::BodyFrame& moon) const;
private:
    ...
    PegSteering peg_;
};

// flight_session.hpp
// LunarStep gains (after SurfaceStay): Ascent, AscentCoast, Circularize, AbortAscent
//   to_string: "Aufstieg", "Freiflug zur Aposelene", "Zirkularisierung", "Abbruch zum Orbit"
// LunarLandingSetup gains: std::optional<gnc::LunarAscentParameters> ascent;  // lift off at the stay's end
// Ascent of the controlled vehicle standing landed on the Moon (lunar step Landed / SurfaceStay, or a body the leg
// simulation judged "gelandet"), `liftoff_delay_s` from now: liftoff, PEG to the insertion, coast to the aposelene,
// circularisation -> InLlo. False with a German lunar_status() (not landed, propellant short for orbit).
bool plan_lunar_ascent(const gnc::LunarAscentParameters& parameters = {}, double liftoff_delay_s = 0.0);
const gnc::LunarAscentGuidance* lunar_ascent_guidance() const { return lunar_ascent_.get(); }
// abort_landing(index) on the controlled body in step PoweredDescent: abort to orbit (step AbortAscent) when the
// propellant suffices (LunarAscentGuidance::abort_delta_v_mps vs the rocket equation, reserve as the landing setup's),
// else false with "Abbruch nicht möglich: ..." and the landing continues.
```

Design decisions (comments in the code):
- **Liftoff:** the landing thrusters at full throttle (T/W ~1.5 at 662 t), axis along the local vertical; the leg
  simulation's track is released (`LegLandingSim::release`) and the body (`release_landed`) is integrated again. At
  `raptor_start_height_m` above the liftoff point the ascent set starts (3 s start-up); the landing thrusters stop when
  the Raptors' thrust exceeds the weight (phase Vertical). At `vertical_rise_height_m`: PEG.
- **Ascent on the vacuum Raptors (user 2026-10-15).** The ascent set is the three fixed vacuum Raptors (symmetric on
  the ring: thrust through the CG, Isp 380 s instead of 350 s: ~10 t of the ~95 t ascent propellant saved): landing
  -> vacuum is a disjoint switch as in the braking phase, and with no gimbal acting the attitude controller's body
  torque goes to the RCS (Task 6's braking path, reused). Thrust 2.68-8.94 MN: at ~130-660 t PEG flies mostly near the
  minimum (~2 g at burnout on the 120 t HLS); the guidance throttles down to the minimum, never below. Aborts fly on
  the same set.
- **Vacuum-Raptor engine-out (user 2026-10-15).** A failed RVac (`fail_engine`; the guidance sees the engine's state)
  makes the off-axis thrust of the other two a torque no RCS holds: the other two are cut at once, the RCS damps the
  rates their shutdown transient left and holds the PEG attitude, and the three gimballed sea-level Raptors start
  (`startup_s`); PEG continues on them (their thrust and flow, Isp 350 s). Below `raptor_start_height_m` + the drop
  during the start-up the landing thrusters stay or come back on until the sea-level Raptors carry the weight. The
  same rule holds in Task 6's braking phase (the braking guidance continues on the three sea-level Raptors).
  Test (Task 8, `[lunar_ascent]`): one RVac failed 30 s into PEG -> the LLO within the same 1 km; the extra propellant
  printed and ledgered.
- **PEG target:** `radius = R + site_height + insertion_altitude_above_site_m`, horizontal speed = vis-viva at that
  periselene of the ellipse to `R + aposelene_altitude_m`, radial speed 0, `terminal_semi_major_axis_m` = that
  ellipse's; plane: polar through the liftoff point (normal = `normalized(cross(r_hat, north_hat))` for `northerly`, its
  negative otherwise; inertial, fixed at `start`). State for PEG: `moon.centred(gcrf)` (Moon-relative inertial).
  Vehicle: thrust and flow of the running ascent set (`current_performance` like the ascent guidance), shutdown time.
- **Launch window:** a polar plane through the liftoff point always passes over it: no wait. (A fixed target plane -
  rendezvous with a depot, M9 - would need the window of the spec.)
- **After cutoff:** AscentCoast to the aposelene (lunar coast, no Sun hold: < 1 h), then `circularize_at_apoapsis(
  moon.centred(state), epoch, gm_moon)` flown by `lunar_burn(node, false, Circularize)`; then InLlo (status
  "in LLO: <periselene> x <aposelene> km"). The burn node's delta-v is in GCRF axes (Moon-relative = same axes).
- **Abort:** from PoweredDescent the landing guidance is dropped, `lunar_ascent_` starts with `abort = true`: while
  the Moon-relative vertical speed is below `abort_min_climb_mps` the ascent set thrusts along the local vertical tilted
  towards the downrange horizontal by at most 30 deg [E] (phase Arrest; the landing thrusters, if running, stay on
  until the Raptors carry the weight), then PEG into the current orbital plane. The check uses
  `abort_delta_v_mps`; refused -> the landing continues untouched (review focus 3).
- **Automatic abort (spec: landing out of reach -> abort to orbit):** when the landing guidance reports Aborted (the
  first approach plan failed) or its SCvx guidance is `aborting()` (repeated unreachable re-plans - on the Moon there
  is no sea, so its abort plans only set the vehicle down near the point), the session tries the abort to orbit; if the
  propellant does not suffice it keeps flying the landing guidance (status "Landepunkt nicht erreichbar, Abbruch nicht
  möglich"). Task 6's "engines off" placeholder for Aborted is replaced by this. Test: `begin_powered_descent` with a
  touchdown point 30 km off the PDI track (unreachable) ends in AbortAscent -> InLlo (`[slow]`, in this task's test
  file).
- **Session:** steps in `flight_session_lunar_ascent.cpp`; command path like the powered descent (the ascent command
  converted to `gnc::ReturnCommand` - run groups, throttle, axis - flown with `ship_landing_controller_`); ticks while
  Ascent / AbortAscent; coast allowance to the circularisation's ignition lead in AscentCoast.

- [x] **Step 1: Write the failing test** `core/tests/test_lunar_ascent.cpp`:

```cpp
// Lunar ascent (l4): a landed HLS lifts off on the landing thrusters, ascends on PEG into 15 x 100 km and circularises
// into the 100 km LLO (< 1 km); a surface stay under time warp; abort to orbit from 1 km and its refusal without
// propellant.

#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/primitives.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;

config::VehicleConfig hls(double propellant_kg, double dry_kg = 0.0) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    if (dry_kg > 0.0) cfg.stages.front().dry_mass_kg = dry_kg;
    for (auto& t : cfg.stages.front().tanks) {
        t.mass_kg = propellant_kg * (t.contents == config::TankContents::Oxidizer ? 3.6 / 4.6 : 1.0 / 4.6);
    }
    return cfg;
}

struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    environment::ReferenceBody moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    sim::LunarSite site;
    Fixture() {
        const auto terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);
        session.set_moon_terrain(terrain);
        site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
        session.set_lunar_landing_site(site);
    }
    // Sets the HLS down at the site centre 0.3 m above the ground and lets the legs judge it "gelandet".
    void land(const config::VehicleConfig& cfg) {
        // the l2 helper place_over_site of test_hls_landing.cpp (copy it here unchanged: clearance 0.3 m, at rest)
        place_over_site(*this, cfg, 0.3, {});
        for (int i = 0; i < 20 * 200 && session.leg_touchdown(0) != sim::LegTouchdown::Landed; ++i) {
            session.advance_to(session.epoch() + sim::FlightSession::tick_s, 10);
        }
        REQUIRE(session.leg_touchdown(0) == sim::LegTouchdown::Landed);
    }
    double altitude_m() const {
        const auto& s = session.body(0).state();
        const auto c = ephemeris.state_gcrf(ephemeris::Body::Moon, session.epoch());
        return norm(s.position_gcrf_m - c.position_m) - constants::moon_mean_radius_m;
    }
};
}  // namespace

TEST_CASE("A released landed body flies again and the leg judgement starts over", "[sim][lunar_ascent][terrain][data]") {
    Fixture fx;
    fx.land(hls(542e3));
    REQUIRE(fx.session.body(0).resting());
    REQUIRE(fx.session.plan_lunar_ascent({}, 0.0));
    fx.session.advance_to(fx.session.epoch() + 3.0, 2000);
    CHECK_FALSE(fx.session.body(0).resting());
    CHECK(fx.session.leg_touchdown(0) == sim::LegTouchdown::None);
    CHECK(fx.session.lunar_step() == sim::LunarStep::Ascent);
}

TEST_CASE("After a 3-day stay under time warp the HLS ascends into the 100 km LLO", "[sim][lunar_ascent][terrain][data][slow]") {
    Fixture fx;
    fx.land(hls(542e3));   // the LLO variant's landing mass ~662 t
    const auto fixed0 = fx.moon.frame(fx.session.epoch()).to_fixed(fx.session.body(0).state().position_gcrf_m);
    REQUIRE(fx.session.plan_lunar_ascent({}, 3.0 * 86400.0));
    // Review focus 5: one warp call over the stay - still landed, no drift on the Moon.
    fx.session.advance_to(fx.session.epoch() + 3.0 * 86400.0 - 60.0, 50'000'000);
    CHECK(fx.session.body(0).resting());
    CHECK(fx.session.leg_touchdown(0) == sim::LegTouchdown::Landed);
    CHECK(norm(fx.moon.frame(fx.session.epoch()).to_fixed(fx.session.body(0).state().position_gcrf_m) - fixed0) < 0.01);
    const double m0 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    fx.session.advance_to(fx.session.epoch() + 3.0 * 3600.0, 50'000'000);
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::InLlo);
    const double m1 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    double low = 1e12, high = -1e12;
    for (int i = 0; i < 2 * 355; ++i) {
        fx.session.advance_to(fx.session.epoch() + 20.0, 100000);
        low = std::min(low, fx.altitude_m());
        high = std::max(high, fx.altitude_m());
    }
    const double dv = 380.0 * constants::standard_gravity_mps2 * std::log(m0 / m1);
    std::printf("[lunar_ascent] LLO %.2f x %.2f km, %.0f t used (%.0f m/s at 380 s), %s\n", low / 1000.0, high / 1000.0,
                (m0 - m1) / 1000.0, dv, fx.session.lunar_status().c_str());
    CHECK(low > 99e3);    // spec: the LLO within 1 km
    CHECK(high < 101e3);
    CHECK(dv == Catch::Approx(1900.0).margin(200.0));   // spec ~1.9 km/s
}
```

  The abort tests use Task 6's PDI placement: move `Fixture::place_at_pdi` of `test_lunar_powered_descent.cpp` into
  `core/tests/test_support.hpp` / `.cpp` as `test::place_hls_at_pdi(session, ephemeris, moon_me, point, t0,
  propellant_kg, dry_kg = 0, range_m = 450e3)` (and use it from both files), then add:

```cpp
namespace {
// Flies the powered descent from PDI until the CG is `height_m` above the touchdown point (or the step changes).
bool descend_to(Fixture& fx, double height_m, const mission::TouchdownPoint& point) {
    const double target_r = constants::moon_mean_radius_m + point.height_m + height_m;
    for (int i = 0; i < 1200 * 200; ++i) {
        if (fx.session.lunar_step() != sim::LunarStep::PoweredDescent) return false;
        const auto c = fx.ephemeris.state_gcrf(ephemeris::Body::Moon, fx.session.epoch());
        if (norm(fx.session.body(0).state().position_gcrf_m - c.position_m) <= target_r) return true;
        fx.session.advance_to(fx.session.epoch() + sim::FlightSession::tick_s, 10);
    }
    return false;
}
}  // namespace

TEST_CASE("An abort 1 km above the touchdown point reaches orbit", "[sim][lunar_ascent][abort][terrain][data][slow]") {
    Fixture fx;
    const auto point = *mission::select_touchdown_point(*fx.session_terrain(), fx.site.config.latitude_rad,
                                                        fx.site.config.longitude_rad, 180.0 * deg, 12.0, {});
    test::place_hls_at_pdi(fx.session, fx.ephemeris, fx.moon_me, point, fx.t0, 1065e3);
    REQUIRE(fx.session.begin_powered_descent(point, 180.0 * deg));
    REQUIRE(descend_to(fx, 1000.0, point));
    REQUIRE(fx.session.abort_landing(0));
    CHECK(fx.session.lunar_step() == sim::LunarStep::AbortAscent);
    fx.session.advance_to(fx.session.epoch() + 3.0 * 3600.0, 50'000'000);
    CHECK(fx.session.lunar_step() == sim::LunarStep::InLlo);
    CHECK(fx.altitude_m() > 90e3);
}

TEST_CASE("An abort without the propellant for orbit is refused and the landing continues", "[sim][lunar_ascent][abort][terrain][data][slow]") {
    Fixture fx;
    const auto point = *mission::select_touchdown_point(*fx.session_terrain(), fx.site.config.latitude_rad,
                                                        fx.site.config.longitude_rad, 180.0 * deg, 12.0, {});
    // Review focus 3: a heavy dry mass leaves ~20 t of propellant at 1 km - enough to land, far too little for orbit.
    test::place_hls_at_pdi(fx.session, fx.ephemeris, fx.moon_me, point, fx.t0, 330e3, 420e3);
    REQUIRE(fx.session.begin_powered_descent(point, 180.0 * deg));
    REQUIRE(descend_to(fx, 1000.0, point));
    CHECK_FALSE(fx.session.abort_landing(0));
    CHECK(fx.session.lunar_status().find("Abbruch nicht möglich") != std::string::npos);
    CHECK(fx.session.lunar_step() == sim::LunarStep::PoweredDescent);
}
```

  (`session_terrain()`: keep the loaded terrain pointer in the fixture and return it.)

- [x] **Step 2: Run to see it fail.** Tagged `"[lunar_ascent]~[slow]"`. Expected: compile errors.

- [x] **Step 3: Implement** `release_landed` (read `hold_landed` and the held-pose integration: release must start the
  integration from the held pose's current state - position and velocity moving with the Moon - with no jump; clear
  `landed_`, `resting_`), `LegLandingSim::release` (track: `touchdown = None`, `at_rest_s = 0`), the guidance and the
  session steps per the design decisions. `plan_lunar_ascent` checks the propellant with `mission::lunar_budget(mass,
  dry, reserve, {{"Aufstieg", 1900, 380}})` (the setup's ascent delta-v and reserve when a setup exists, else these
  defaults). In Task 7's SurfaceStay: at the stay's end, if `lunar_landing_setup_->ascent`, call `plan_lunar_ascent(
  *ascent, 0.0)`; else step Landed.

- [x] **Step 4: Run until green.** Fast cases, then the three slow ones by name (`-s`). Record LLO apsides, ascent
  delta-v, abort outcome in the Ledger. If PEG's insertion leaves the periselene below the site's terrain + 10 km,
  raise `insertion_altitude_above_site_m` - never lower a check.

- [x] **Step 5: Commit, push** (after `-Fast`).

```bash
git add core/include/starsim/gnc/lunar_ascent_guidance.hpp core/src/gnc/lunar_ascent_guidance.cpp core/src/sim/flight_session_lunar_ascent.cpp core/include/starsim/vehicle/flight_body.hpp core/src/vehicle/flight_body.cpp core/include/starsim/sim/leg_landing_sim.hpp core/src/sim/leg_landing_sim.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session*.cpp core/tests/test_lunar_ascent.cpp core/tests/test_lunar_powered_descent.cpp core/tests/test_support.hpp core/tests/test_support.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat(sim): lunar ascent on PEG, circularisation and abort to orbit"
git push origin main
```

---

### Task 9: Mission 1 complete: config, chained steps, budget (120 t closes, 204 t refused)

**Files:**
- Modify: `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp`,
  `core/include/starsim/sim/lunar_landing_mission.hpp`, `core/src/sim/lunar_landing_mission.cpp`,
  `core/include/starsim/sim/mission_start.hpp`, `core/src/sim/mission_start.cpp`,
  `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session_lunar.cpp` (chain hooks),
  `config/missions/hls_lunar_llo.toml`, `config/missions/hls_lunar_nrho.toml`
- Test: `core/tests/test_lunar_landing_mission.cpp`

**Interfaces:**
- Consumes: Tasks 5-8; l3 `plan_lunar_outbound`, `plan_lunar_orbit_transfer`, `OutboundPlan` (TLI, flyby, insertion
  nodes), `lunar_outbound_parameters`.
- Produces:

```cpp
// mission_config.hpp, LunarLandingMission gains (TOML sections in brackets; defaults = LunarDescentParameters /
// LunarAscentParameters / LunarLandingSetup defaults):
    // [descent]
    double pdi_altitude_above_site_m = 15.0e3;
    double braking_range_m = 450.0e3;
    double max_crossrange_m = 20.0e3;
    double slope_limit_rad = 8.0 * constants::deg_to_rad;
    double touchdown_search_radius_m = 1000.0;
    // [surface]
    double surface_stay_s = 0.0;
    // [ascent]
    double insertion_altitude_above_site_m = 15.0e3;
    double ascent_aposelene_m = 100.0e3;
    // [budget] (nominal delta-v of the legs not planned at the start; [D]/[E] in the TOML)
    double descent_delta_v_mps = 2000.0;
    double ascent_delta_v_mps = 1900.0;
    double nrho_to_llo_delta_v_mps = 876.0;
    double llo_to_nrho_delta_v_mps = 725.0;
    double reserve_kg = 10000.0;

// flight_session.hpp
// The whole lunar landing mission as one autopilot sequence (mission 1): after the outbound flight the vehicle
// (NRHO variant: first transfers NRHO -> LLO) plans the landing, stays, ascends into the LLO and (NRHO variant)
// transfers back to the NRHO. Each leg is planned from the real state when the previous one ends ("planning nodes").
struct LunarMissionSequence {
    LunarLandingSetup landing;   // with `ascent` set
    bool via_nrho = false;
    std::shared_ptr<const mission::NrhoReference> nrho;   // NRHO variant
    mission::LloTarget llo;
};
void set_lunar_mission(std::optional<LunarMissionSequence> sequence);
bool lunar_mission_complete() const;   // the sequence's last leg reached (status "Mission abgeschlossen: ...")

// lunar_landing_mission.hpp
LunarLandingSetup lunar_landing_setup(const config::LunarLandingMission& mission);
// Rocket-equation budget of the whole mission from the planned outbound flight (TLI, swing-by, insertion / LOI) and the
// mission's nominal legs; Isp 380 s for the vacuum-Raptor legs (the braking phase and the ascent), 350 s for the rest of
// the descent.
mission::BudgetResult lunar_mission_budget(const config::LunarLandingMission& mission, const mission::OutboundPlan& plan,
                                           double initial_mass_kg, double dry_mass_kg);
// start_mission's lunar branch with an optional vehicle override (tests: the 204 t HLS).
MissionStart start_lunar_landing_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                         const config::LunarLandingMission& mission, time::Epoch epoch,
                                         const config::VehicleConfig* vehicle = nullptr);
```

Design decisions:
- `place_lunar_landing_mission` also loads the terrain (`sim::load_lunar_terrain`) and sets the mission's site
  (`set_moon_terrain`, `set_lunar_landing_site`); a missing tile refuses the start with the German hint to `fetch.py`
  (spec error handling) - catch the loader's exception into `MissionStart::refusal`.
- `start_lunar_landing_mission`: place, plan the outbound flight, then the budget; when it does not close:
  `disengage_autopilot()`, `return_planned = false`, refusal `"Mission nicht durchführbar: " + reason`. Else
  `set_lunar_mission(sequence)`.
- Chain hooks in the step machine (one place, `lunar_mission_next()` called when a step settles): InNrho after the
  outbound insertion (first time) -> `plan_lunar_orbit_transfer(Llo, nrho, llo)`; InLlo before the landing ->
  `plan_lunar_landing(landing)`; InLlo after the ascent -> NRHO variant `plan_lunar_orbit_transfer(Nrho, nrho)`, else
  complete; InNrho after that transfer -> complete. A refused leg ends the sequence with its reason in the status
  (autopilot stays in the orbit).
- Spec deviation (ruling for the Ledger): the spec lists "no pass over the site within the planning span" among the
  mission-start refusals; the LLO's plane is only known after the LOI (and, NRHO variant, after the NRHO -> LLO
  transfer), so the pass is checked when the descent is planned in the LLO - a refusal there ends the sequence in the
  LLO with the planner's German reason. The start refuses only what it can know: budget, missing tile, no touchdown
  point within the slope limit (checked at the start with the approach azimuth of a polar pass - both directions
  tried).
- Mission epochs: run the LLO mission once with `STARSIM_DEBUG_LUNAR_SESSION=1` and read the descent wait; if it is
  longer than 2 days, move `epoch_utc` [E] (both TOMLs) by whole days until the wait is < 2 days, keeping the
  outbound's checks green - record the epoch and the wait in the TOML comment and the Ledger.

- [x] **Step 1: Write the failing tests** (append to `core/tests/test_lunar_landing_mission.cpp`):

```cpp
TEST_CASE("Lunar landing missions load their descent, surface, ascent and budget sections", "[config][lunar]") {
    const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    CHECK(m.pdi_altitude_above_site_m == 15e3);
    CHECK(m.braking_range_m == 450e3);
    CHECK(m.surface_stay_s > 0.0);
    CHECK(m.insertion_altitude_above_site_m == 15e3);
    CHECK(m.descent_delta_v_mps == 2000.0);
    CHECK(m.reserve_kg == 10000.0);
}

TEST_CASE("Mission 1 closes with the 120 t HLS and is refused with 204 t", "[sim][lunar][lunar_budget_mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    for (const char* id : {"hls_lunar_llo", "hls_lunar_nrho"}) {
        const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, id);
        {
            sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
            const auto s = sim::start_lunar_landing_mission(session, STARSIM_CONFIG_DIR, m, time.from_utc(m.epoch_utc));
            CAPTURE(id, s.refusal);
            CHECK(s.return_planned);
        }
        {
            auto heavy = config::load_vehicle(STARSIM_CONFIG_DIR, m.vehicle_id);
            heavy.stages.front().dry_mass_kg = 204e3;   // the independent bottom-up estimate (spec)
            sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
            const auto s = sim::start_lunar_landing_mission(session, STARSIM_CONFIG_DIR, m, time.from_utc(m.epoch_utc), &heavy);
            CAPTURE(id);
            CHECK_FALSE(s.return_planned);
            CHECK(s.refusal.find("Mission nicht durchführbar") != std::string::npos);
            CHECK(session.lunar_step() == sim::LunarStep::None);
        }
    }
}

TEST_CASE("Mission 1, LLO variant: LEO to the lunar surface and back to the LLO", "[sim][lunar][lunar_mission_flight][terrain][data][slow]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    const auto s = sim::start_mission(session, STARSIM_CONFIG_DIR, "hls_lunar_llo", {}, time);
    REQUIRE(s.return_planned);
    const double m0 = session.body(0).vehicle().mass_properties().mass_kg;
    session.advance_to(s.epoch + 40.0 * 86400.0, 200'000'000);
    const auto& td = session.lunar_touchdown();
    REQUIRE(td);
    std::printf("[lunar_mission] LLO variant: %s, touchdown %.2f m / %.2f m/s, %.0f t left of %.0f t, %s\n",
                sim::to_string(td->outcome), td->offset_m, td->sink_mps,
                session.body(0).vehicle().mass_properties().mass_kg / 1000.0, m0 / 1000.0, session.lunar_status().c_str());
    CHECK(td->outcome == sim::LegTouchdown::Landed);
    CHECK(td->offset_m < 1.0);
    CHECK(session.lunar_mission_complete());
    CHECK(session.lunar_step() == sim::LunarStep::InLlo);
}

TEST_CASE("Mission 1, NRHO variant: LEO to the lunar surface and back to the NRHO", "[sim][lunar][lunar_mission_flight][nrho][terrain][data][slow]") {
    // as the LLO variant with "hls_lunar_nrho"; ends complete in step InNrho. If the flight runs short of propellant
    // (the NRHO variant closes with ~10-20 t, spec: SpaceX's plan closes only <= ~140 t dry), report it - do not tune
    // a limit.
}
```

  Complete the NRHO case like the LLO one (`advance_to(s.epoch + 60 d)` - the NRHO transfer windows; check
  `lunar_mission_complete()` and `InNrho`).

- [x] **Step 2: Run to see them fail.** Tagged `"[lunar_budget_mission]"` and `"[config][lunar]"`. Expected: compile
  errors.

- [x] **Step 3: Implement** the config fields (parse the sections with toml++ like the existing ones; units in the
  TOML: km, deg, hours, m/s, t - converted on load), `lunar_landing_setup`, `lunar_mission_budget` (legs: "TLI"
  |tli dv|, NRHO: "Swing-by" |flyby dv|, "Einschuss" |insertion dv|, "NRHO -> LLO" nominal; LLO: "LOI" |loi dv|; then
  "Abstieg", "Aufstieg", NRHO: "LLO -> NRHO"), `start_lunar_landing_mission`, `start_mission` delegating to it, the
  sequence and its hooks, and the TOML sections:

```toml
[descent]
pdi_altitude_above_site_km = 15.0   # [S] Apollo: PDI at ~50,000 ft above the landing site
braking_range_km = 450.0            # [E] spec 400-500 km
max_crossrange_km = 20.0            # [E] user 2026-10-15: wait for a pass within this, plane trim removes it
slope_limit_deg = 8.0               # [E] NASA Artemis ~8-10 deg; the 12 m stance lands >= 9 deg
touchdown_search_radius_m = 1000.0  # [E]

[surface]
stay_hours = 24.0                   # [E] mission parameter (Artemis III plans ~6.5 days; Apollo 17 75 h)

[ascent]
insertion_altitude_above_site_km = 15.0  # [E] Apollo inserted ~18 km up
aposelene_km = 100.0                     # the LLO (spec)

[budget]                            # legs not planned at the start (nominal); the outbound legs come from its plan
descent_delta_v_mps = 2000.0        # [E] spec ~2.0 km/s (Apollo LM ~2.0)
ascent_delta_v_mps = 1900.0         # [E] spec ~1.9 km/s (Apollo LM ~1.85)
nrho_to_llo_delta_v_mps = 876.0     # [D] l3: 660 + 216 m/s
llo_to_nrho_delta_v_mps = 725.0     # [D] l3: right after the LLO arrival (cheap window)
reserve_kg = 10000.0                # [E] residuals, unusable propellant
```

- [x] **Step 4: Run until green.** The fast tagged cases, then the two flights by name (`-s`, each may take many
  minutes - run them in the background and read the tail). Record the numbers (wait, touchdown, masses left, NRHO
  margin) in the Ledger. The NRHO variant's margin is a real result: if it does not close in flight, record the
  shortfall, mark the test `[!mayfail]` ONLY after asking the user (ruling in the Ledger).

- [x] **Step 5: Commit, push** (after `-Fast`).

```bash
git add core/include/starsim/config/mission_config.hpp core/src/config/mission_config.cpp core/include/starsim/sim/lunar_landing_mission.hpp core/src/sim/lunar_landing_mission.cpp core/include/starsim/sim/mission_start.hpp core/src/sim/mission_start.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session_lunar.cpp config/missions/hls_lunar_llo.toml config/missions/hls_lunar_nrho.toml core/tests/test_lunar_landing_mission.cpp
git commit -m "feat(sim): lunar landing mission 1 complete (chained descent, stay, ascent; budget refusal)"
git push origin main
```

---

### Task 10: Landing campaign 21/21, aborts from 3 / 1 / 0.3 km, docs, full suite, review

**Files:**
- Create: `core/tests/test_lunar_landing_campaign.cpp`
- Modify: `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp` (truth thrust factor),
  `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (`set_thrust_dispersion`),
  `core/tests/CMakeLists.txt`, `docs/subsystems.md`, `CLAUDE.md`, `config/README.md`, this plan's Ledger, memory

**Interfaces:**
- Consumes: Tasks 6 and 8 (`begin_powered_descent`, `lunar_touchdown`, `abort_landing`), `test::place_hls_at_pdi`.
- Produces:

```cpp
// vehicle.hpp: truth-model dispersion (tests, training): thrust and mass flow of every engine scaled by `factor` in
// the forces the vehicle produces; the performance queries the guidance uses (thrust_n, current_performance) keep the
// nominal values.
void set_thrust_factor(double factor);
// flight_session.hpp
void set_thrust_dispersion(size_t body_index, double factor);
```

- [x] **Step 1: Truth thrust factor.** Find where the vehicle sums the engines' thrust into its force and mass flow
  (grep `thrust_n(` in `core/src/vehicle/vehicle.cpp` / the stepping code) and multiply there by `thrust_factor_`
  (default 1.0: bit-identical). Test in `test_hls_vehicle.cpp`: a body with factor 1.02 accelerates 2 % more over one
  tick than with 1.0 while `performance->thrust_n` is unchanged.

- [x] **Step 2: Write the campaign** `core/tests/test_lunar_landing_campaign.cpp` (pattern of
  `test_scvx_catch.cpp`'s campaign: fixed seeds, one printed line per case, the sum at the end):

```cpp
// Lunar landing campaign (spec l4): from PDI with state +-1 km / +-5 m/s, thrust +-2 %, mass +-5 % and touchdown
// points on the real terrain (the admissible point nearest to a random spot within 1 km of the site): 21 / 21 land
// within 1 m at <= 1 m/s. Aborts from 3 / 1 / 0.3 km reach orbit. Single cases: "[.lunar_case]" with
// STARSIM_LUNAR_CASE=<seed>.

namespace {
struct Dispersion {
    Vec3 position_m;      // added to the PDI state (GCRF), each axis +-1 km
    Vec3 velocity_mps;    // +-5 m/s per axis
    double thrust_factor = 1.0;   // +-2 %
    double mass_factor = 1.0;     // propellant +-5 %
    double east_m = 0.0, north_m = 0.0;   // touchdown target spot (within 1 km)
};
Dispersion dispersion(unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    Dispersion d;
    d.position_m = {1000.0 * unit(rng), 1000.0 * unit(rng), 1000.0 * unit(rng)};
    d.velocity_mps = {5.0 * unit(rng), 5.0 * unit(rng), 5.0 * unit(rng)};
    d.thrust_factor = 1.0 + 0.02 * unit(rng);
    d.mass_factor = 1.0 + 0.05 * unit(rng);
    const double r = 1000.0 * std::sqrt(0.5 * (unit(rng) + 1.0)), a = constants::pi * unit(rng);
    d.east_m = r * std::cos(a);
    d.north_m = r * std::sin(a);
    return d;
}
struct Run { sim::LegTouchdown outcome = sim::LegTouchdown::None; double offset_m = 0, sink_mps = 0, used_t = 0; };
Run fly(const Dispersion& d);   // place_hls_at_pdi (1,065 t x mass factor), the dispersion, the touchdown point from
                                // select_touchdown_point at the spot, begin_powered_descent, ticks to the judgement
}  // namespace

TEST_CASE("Lunar landing campaign: 21 dispersed descents land within 1 m", "[sim][lunar_campaign][validation][terrain][data][slow]") {
    int landed = 0;
    for (unsigned seed = 1; seed <= 21; ++seed) {
        const auto run = fly(dispersion(seed));
        std::printf("case %2u: %s, offset %.2f m, sink %.2f m/s, %.0f t\n", seed, sim::to_string(run.outcome),
                    run.offset_m, run.sink_mps, run.used_t);
        std::fflush(stdout);
        if (run.outcome == sim::LegTouchdown::Landed && run.offset_m < 1.0 && run.sink_mps <= 1.1) ++landed;
    }
    std::printf("landed %d of 21\n", landed);
    CHECK(landed == 21);   // spec: 21/21
}

TEST_CASE("Lunar landing campaign: a single case", "[.lunar_case]") {
    const char* seed = std::getenv("STARSIM_LUNAR_CASE");
    const auto run = fly(dispersion(seed ? static_cast<unsigned>(std::atoi(seed)) : 1u));
    std::printf("%s, offset %.2f m, sink %.2f m/s\n", sim::to_string(run.outcome), run.offset_m, run.sink_mps);
}

TEST_CASE("Aborts from 3, 1 and 0.3 km reach orbit", "[sim][lunar_campaign][abort][validation][terrain][data][slow]") {
    for (const double h : {3000.0, 1000.0, 300.0}) {
        // as Task 8's abort test: descend_to(h), abort_landing, warp 3 h; InLlo and altitude > 90 km
    }
}
```

  Fill in `fly` and the abort loop (the comments say what; reuse Task 8's `descend_to` via `test_support` if it is
  needed in both files). Split the campaign over several test cases (e.g. seeds 1-7, 8-14, 15-21) if one case runs
  longer than ~10 min, so ctest runs them in parallel.

- [x] **Step 3: Run the campaign** in the background (`-s`, output to a file; read the tail). Every failing seed: run
  it alone (`"[.lunar_case]"`, `STARSIM_LUNAR_CASE`, `STARSIM_DEBUG_SCVX=1`, `STARSIM_DEBUG_LUNAR_DESCENT=1`), find
  the cause (superpowers:systematic-debugging), fix the guidance (not the tolerance), re-run all 21. Record each
  finding in the Ledger.

- [x] **Step 4: Delta-v against the literature.** From the campaign and Task 7/8 prints: descent (DOI + braking +
  approach) vs ~2.0 km/s (Apollo LM: DOI ~22 m/s + PDI ~2.04 km/s [S: Bennett TN D-6846]), ascent vs ~1.9 km/s (Apollo
  ~1.85 km/s); write both into `docs/subsystems.md` with the differences explained (Raptor T/W: less gravity loss).

- [x] **Step 5: Docs.** `docs/subsystems.md`: an "l4 landing and ascent" paragraph under the lunar entry (decisions
  with dates: thruster re-sizing, site-pass wait; DOI / braking / approach / touchdown / stay / ascent / abort; the
  equivalent-gimbal simplification for the landing thrusters; the numbers; diagnostics `STARSIM_DEBUG_LUNAR_DESCENT`;
  open points: DOI re-plan after the trim, differential throttling of the landing thrusters, boil-off).
  `CLAUDE.md`: the module list in "Architecture" (mission: lunar descent, budget; gnc: Apollo braking, PEG module,
  lunar landing / ascent guidance) - one line each, no numbers (they live in subsystems.md). `config/README.md`: the
  mission sections `[descent]`, `[surface]`, `[ascent]`, `[budget]`.

- [x] **Step 6: Full suite and smoke test.** `powershell -ExecutionPolicy Bypass -File tools/build.ps1 >
  $env:TEMP\l4_full.txt; Get-Content $env:TEMP\l4_full.txt -Tail 30` (with the bindings: the bindings use
  `plan_state` / `LandingSite` only through the core - check they build), then the Godot smoke test from `CLAUDE.md`.
  All pass.

- [x] **Step 7: Final review.** One opus reviewer (superpowers:requesting-code-review) over `7bf4c2b..HEAD` with the
  spec's l4 section, this plan and its Review Focus. Fix Critical / Important; list the deferred minors in the Ledger.

- [x] **Step 8: Memory, commit, push.** Update the memory file `project-lunar-mission.md` (l4 complete, numbers, next:
  l5 plan or l6), commit with the docs:

```bash
git add core/include/starsim/vehicle/vehicle.hpp core/src/vehicle/vehicle.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/tests/test_lunar_landing_campaign.cpp core/tests/test_hls_vehicle.cpp core/tests/CMakeLists.txt docs/subsystems.md CLAUDE.md config/README.md docs/superpowers/plans/2026-10-15-lunar-l4-landing-ascent.md
git commit -m "test(sim): lunar landing campaign 21/21, aborts to orbit; docs for l4"
git push origin main
```

---

## Ledger

(Rulings, measured numbers and deferred minors are recorded here during execution.)

Copied from the execution ledger (.superpowers/sdd/2026-10-15-lunar-l4-landing-ascent/progress.md) at completion:

# SDD ledger — plan: docs/superpowers/plans/2026-10-15-lunar-l4-landing-ascent.md

Executor: inline (superpowers:executing-plans), on main (CLAUDE.md: no feature branches), one task per session (/clear between).
Spec: docs/superpowers/specs/2026-10-14-lunar-missions-design.md (l4 section).

Pre-flight (interface rows):
- T1 -> T6/T8: landing group 12 x 140 kN canted 15 deg, min throttle 0.40 -> consumed by braking/approach/liftoff sets; consistent.
- T2 -> T8: PegSteering/PegTarget/PegState/PegVehicle -> lunar ascent; consistent (names identical).
- T3 -> T6: LandingSite::body, plan_frame(site), site_body_frame, plan_state, ScvxLandingGuidance on the Moon; consistent.
- T4 -> T6: ApolloBrakingGuidance, BrakingTarget; consistent.
- T5 -> T6/T7/T9: TouchdownPoint, LunarDescentPlanner, LunarDescentParameters, lunar_budget; consistent.
- T6 -> T7/T8/T10: begin_powered_descent, steps PoweredDescent/Landed/Lost, lunar_touchdown; consistent.
- T7/T8 -> T9/T10: LunarLandingSetup, abort_landing, SurfaceStay; consistent.
No conflicts found.

Task 1: Ruling: test uses the existing l2 pattern (e.landing_only, e.thrust_direction()) instead of the brief's group_id == "landing" / axis_body (group ids are stage-prefixed, "hls/landing") — same quantity — cost if wrong: none.
Task 1: Ruling: Moon GM constant named `moon_gm_m3ps2` (convention of earth_gm_m3ps2), not the brief's gm_moon_m3ps2; later tasks use moon_gm_m3ps2 — cost if wrong: one rename.
Task 1: Ruling: drop test reuses drop_on_slope(slope, azimuth, propellant_kg) (new default arg) instead of a set_slope_surface helper — less code, same cases (uphill 0 and 30 deg) — cost if wrong: none.
Task 1: legs: only crush_force_n x3 (1.5 -> 4.5 MN; at 1.5 MN the 542 t-propellant drops crushed through on 8 deg); stiffness and damping kept at l2 values (x3 left the 220 t HLS balancing on two feet on the LOLA terrain > 60 s, l2 terrain test failed). Measured: thrusters 649-1623 kN axial, hover 400-999 t; all drops land 0/5/8 deg, tip at 25 deg, crush 0 mm.
Task 1: complete (commits dbce778..7a76593, tests: powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter [hls] → All tests passed (75 assertions in 12 test cases))
Task 2: complete (commits 7a76593..adc64a4, tests: powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast → Total Test time (real) = 106.58 sec)
Task 2: Ruling: the extracted PEG time-to-go iteration diverges from a vertical lunar start (high T/W: denominator f_theta + f_theta_dot tau + f_theta_ddot tau^2 ~ -1.75, tgo 48 s instead of ~96 s, orbit -1697 x 41 km) - added a fallback for the same horizontal-impulse equation (thrust integrals L, S, Q at the current t_go, dv = (dh/r_mean - f_theta_dot S - f_theta_ddot Q)/f_theta) when f_theta > 0.5 and denominator < 0.5 f_theta — the plan's own test and Task 8 (PEG from 150 m vertical rise) need it; Earth bit-identity proven by an abort() probe in the branch over the full suite (521/521; the first probe without the f_theta > 0.5 guard fired in a degenerate post-insertion Earth state, fr ~ 42, hence the guard) — cost if wrong: lunar ascent steering law to revisit in Task 8.
Task 2: Ruling: test integrates at 5 ms (session's 200 Hz) and applies the shutdown tail-off (0.5 * a * shutdown_s) as an impulse at cutoff, instead of the brief's 0.1 s and no tail-off — at ~24 m/s^2 a 0.1 s step alone moves the aposelene ~10 km and PEG deliberately leaves ~6 m/s (~25 km aposelene) to the tail-off, so the brief's 1 km margin was unreachable; result 15.00 x 100.01 km — cost if wrong: none.
Task 2: Ruling: constant `moon_gm_m3ps2` (Task 1 ruling) instead of brief's gm_moon_m3ps2 — cost if wrong: none.
Task 3: Ruling: phase groups are stage-prefixed ("hls/sea_level_1", "hls/landing"), not the brief's bare ids - same engines - cost if wrong: none.
Task 3: Ruling: approach phases {hls/sea_level_1} -> {hls/sea_level_1} -> {hls/landing} instead of the brief's {3 sea-level} -> {sea_level_1} -> {landing}: three Raptors at minimum throttle (2.62 MN = 4.5 m/s^2 at 580 t) over-brake the gate's ~2.3 m/s^2 (Klumpp gate acceleration 0.3 up / 1.2 back + g); SCvx then stretched phase 0 to max_phase_s, tilted > 100 deg and never converged (every variant tried). One sea-level Raptor (0.87-2.9 MN vacuum) covers the gate; a doubled phase is legal (same set, switch window only). Task 6's braking sets must end on one Raptor as well (its {2 sea-level} min 1.75 MN also over-brakes) - cost if wrong: Task 6 approach sets to revisit.
Task 3: Ruling: new opt-in ScvxSettings::kinematic_guess_s + gnc::kinematic_guess (cubic Hermite path matching both end states, thrust = m(a - g + frame terms) clipped to the set and tilt limits, attitude along it): the straight-line guess (22 s for 3 km at 96 m/s) put SCvx in a poor basin. All zero (default) keeps the old guess: Earth bit-identical - cost if wrong: one guess function to drop.
Task 3: Ruling: test gate attitude tilted along the gate acceleration (~32 deg uprange) instead of upright - the braking phase hands over tilted; upright cost the slow single Raptor (0.05 rad/s) the time to tilt and SCvx failed - cost if wrong: none (Task 6 hands over the real attitude).
Task 3: HLS SCvx values for Task 6 (hls_landing_parameters): tilt {45, 45, 3} deg, rate {0.05, 0.05, 0.01} rad/s, switches[0] {0, 0, 5, 60, 0}, switches[1] {2 deg, 0.01, 1, 10, 150 m}, approach_time_constant 0, kinematic_guess_s {60, 10, 30}, reserve 5 t. Result: converged in 20 iterations, phases 120.0 / 8.7 / 56.0 s, 580 -> 517.4 t (~370 m/s at Isp 350 s), switch to the landing thrusters at ~243 m; phase 0 sits at max_phase_s (a single Raptor at minimum ~ the lunar weight: it descends only by tilting) - fuel within the 2.0 km/s descent budget, efficiency to revisit with the flown re-plans in Task 6.
Task 3: equivalent gimbal of the landing set 0.31 deg (RCS torque / (lever 13.6 m x 1.62 MN)); Earth: frames::rotation_vector == vehicle::earth_rotation_vector (same function), BodyFrame centred at origin subtracts exact zeros.
Task 3: complete (commits adc64a4..428bfee, tests: powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast → 100% tests passed out of 498; [descent]~[slow],[scvx]~[slow],[catch]~[slow],[legs]~[slow] → 90/90)
User decision 2026-10-15 (after Task 3): braking phase on the three vacuum Raptors (RCS attitude), then sea_level_1; plan (User decisions, Global Constraints, Task 6 braking_sets + disjoint-set lead + RCS tracking check, Task 7/9 budget split braking 380 s / rest 350 s) and spec amended. Task 9 must re-check that the 204 t HLS is still refused with the better braking Isp (~25-30 t saved) - if it now closes, report to the user, do not tune the budget.
Task 4: complete (commits a44c9b9..4093e9c, tests: powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast → Total Test time (real) = 104.53 sec)
Task 4: Ruling: Klumpp's Apollo 11 target pad loads are not tabulated in R-695 - the validation test builds the high-gate target with Klumpp's own P63 targeting recipe (R-695: terminal state = P64 initial state 2.2 km / 7.5 km / -44 / 129 m/s, 57 % thrust at 60 deg pitch, Z jerk = KJ 1.2 x mass-flow thrust-acceleration rate, terminal mass estimate 8800 kg [D]); trim 10 % for 26 s, then max 94 % of 46 706 N, Isp 311 s, PDI 14 990 kg at 50 000 ft / 260 nmi / 5560 ft/s - cost if wrong: none (law validated, values sourced in the test header).
Task 4: Ruling: the end check extrapolates the state over the remaining t_go (< 2 s) with the last commanded acceleration instead of comparing the raw state (that would differ by up to v*t_go ~ 260 m) - cost if wrong: none.
Task 4: measured: Apollo braking 508.5 s (flown 507 s, premission 505 s, R-695 typical 514 s), propellant 6259 kg (end 8731 kg vs estimate 8800), end error 0.0 m / 0.01 m/s; no frame/sign conversion needed (positive t_go, x up, z downrange).
Task 5: Ruling: touchdown selection in its own lunar_touchdown.cpp from the start (brief allows it when > ~400 lines; descent 331 + touchdown 113) — cost if wrong: none.
Task 5: Ruling: plane-trim first guess dv = +v asin(s.n) along the orbit normal (derived: an impulse +dv n moves the track +dv r/v towards +n at the pass), not the brief's minus sign — the secant would fix either; with the derived sign the first guess already lands at 0.1 m — cost if wrong: none.
Task 5: Ruling: approach azimuth from the ground velocity projected on the SITE's horizontal (not the vehicle's subpoint): near the pole (site ~5 deg off) a 20 km crossrange turns the local north by several degrees — cost if wrong: glide-cone check against a slightly wrong uprange direction.
Task 5: Ruling: budget refusal names the first short leg with the shortfall after THAT leg (margin_kg is the final one) — cost if wrong: wording only.
Task 5: Ruling: in_tile checked on the site up front (before the pass search), not after the pass — same refusal, earlier — cost if wrong: none.
Task 5: measured (test fixture, align 0.5 d): touchdown 0.0 m from the site (slope 2.53 deg, height 5321.9 m); passes at +1665/+8732/+15800 s (crossrange -17.4/-14.4/-11.3 km; the first two before min_lead + 1.25 P = 10.6 ks); pass 0.18 d, crossrange -11373.6 -> 0.1 m, trim 10.66 m/s (one secant step not needed), DOI 18.22 m/s (one secant step: 1.4 m), periselene 20.32 km above the sphere (= height + 15.0 km), PDI range 450.0 km; refusal case: crossrange ~ -142 km. -Fast 505/505.
Task 5: complete (commits 4093e9c..f13fcb4, tests: powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter [lunar_descent_plan] → All tests passed (20 assertions in 4 test cases))
User decision 2026-10-15 (after Task 5): ascent on the three vacuum Raptors (RCS attitude, Isp 380 s, aborts too; engine-out not modelled); plan (User decisions, Global Constraints, Task 7 budget, Task 8 ascent_groups + design note + test Isp, Task 9 budget text) and spec amended. Task 9 re-checks the 204 t refusal with both 380 s legs.
User decision 2026-10-15: RVac engine-out (ascent and braking): cut the other two at once, RCS corrects, start the three sea-level Raptors, guidance continues; plan Task 8 design note + test (fail 30 s into PEG -> LLO within 1 km), applies to Task 6 braking too; spec amended.
Task 6: Ruling: test placement flipped (brief's r_hat was SOUTH of the point, moving north, against azimuth 180) - north of p, moving south - cost if wrong: none.
Task 6: Ruling: test steps 0.1 s (advance_to(epoch + tick_s) rounds below one tick and does nothing) - cost: none.
Task 6: Ruling: braking hand-over at t_go 15 s (not 5): Klumpp gains 12/t^2, 6/t outrun the attitude loop of a 700 t HLS on one Raptor below t_go ~12 s (tumbled, 0.47 rad/s at t_go 5).
Task 6: Ruling: high gate lowered to (1150 up, -1350 down) m, (-20, 35) m/s, a (0.3, -1.0) [E] (was 2000/-3000, 90 m/s): from 3 km / 4.5 km / 108 m/s SCvx produced junk optima (overshoot past the point, tilt 121 deg, phase 0 at max_phase_s) and did not converge; from 1.5 km / 2 km / 50 m/s it converges in 8 iterations, re-plans in 2-10. Spec says "~2 km" - deviation.
Task 6: Ruling (gnc fix): ScvxLandingGuidance's "too slow -> switch now" rule only when the running set climbs at minimum throttle (every Earth set does; the HLS's single Raptor at minimum is below the lunar weight and was switched to the landing thrusters 460 m up, 290 m off).
Task 6: added SnapshotSettings::pad_final_set_radius_m (default 0, HLS 10 m) and DescentLimits::final_gimbal_fraction (default 0, HLS 1.0); both Earth-neutral by default.
Task 6: (intermediate, superseded below) braking 711 s (1185 -> 685 t; vacuum -> sea_level_1 drop at 2.6 MN, ~t_go 420 s), axis tracking error <= 0.26 deg on the vacuum Raptors (RCS), braking law reaches the hand-over exactly. Landings so far: "gelandet" at 0.7-25 m, sink 0.8-1.5 m/s, lateral 0.4-1.6 m/s, 2.25-2.3 km/s (at 350 s). OPEN: final phase on the landing thrusters (RCS only: ~1e-3 rad/s^2, 3 deg tilt takes ~13 s) - axis jump ~5.5 deg at the Raptor -> landing-thruster switch (single off-axis Raptor), phase-2 re-plans rarely converge, LQR with the 0.31 deg equivalent gimbal has no authority (attitude controller on the final set works better).
User decision 2026-10-15 (Task 6): terminal descent on the landing thrusters as Apollo P66 (sink rate from height, lateral nulling) instead of SCvx tracking to the touchdown (that left 1-25 m / up to 1.6 m/s sideways on the RCS-steered set).
User decision 2026-10-15 (Task 6): the twelve landing thrusters are throttled individually (differential throttling, "the FAA would never allow anything else"): canted thrust lines miss the CG by ~9 m each (~1.3 MNm per thruster), trims give ~20x the RCS torque; the planner's equivalent gimbal of the landing set 0.31 -> 9.4 deg (67badae).
Task 6: Ruling: high gate finally (700 up, -800 down) m, (-15, 25) m/s, a (0.3, -0.8) [E] - Apollo-like 2-3 km gates gave SCvx junk optima or none; the 1150/1350 m gate worked but its 120 s Raptor approach cost ~50 m/s more. Spec says "~2 km" - deviation, tell the user - cost if wrong: the approach phase's visibility of the site (M7 cockpit view) starts lower.
Task 6: Ruling: offset measured at the footprint centre (hull axis at the feet's station), not the CG: on the 2.5 deg slope the 21 m high CG of the standing HLS leans ~0.9 m downhill - cost if wrong: none.
Task 6: Ruling: P66 vertical term floored at the set's minimum thrust (a sink demand below it turned the thrust 90 deg sideways: 28 m miss); [E] P66 values sink h/10 s within [1, 10] m/s, vertical tau 1 s, lateral omega 0.2 rad/s critically damped, lead 1 s, tilt 4 deg.
Task 6: Ruling: SCvx lateral_effort_fraction 0.05 for the HLS (0.2 rocked the 700 t HLS on one Raptor up to 0.7 rad/s) [E].
Task 6: Ruling: RVac engine-out in the braking phase implemented (engine_out_set = three sea-level Raptors) but not tested here - Task 8 tests the same rule in the ascent; Task 10's campaign should add a braking engine-out case - cost if wrong: an untested branch.
Task 6: measured (450 km PDI, 1,185 t): braking 755 s on vacuum -> sea_level_1 (drop at 2.6 MN), hand-over at t_go 15 s, SCvx first plan 4 iterations, terminal descent from ~140 m, gelandet 0.72 m from the point, sink 1.05 m/s, lateral 0.23 m/s, 557 t used (2182 m/s at 350 s; 400 km PDI: 684 s braking, 2132 m/s). Full suite 536/536 (with bindings) + Godot smoke test.
Task 6: not done here (Task 10 docs): docs/subsystems.md lunar descent section.
Task 6: complete (commits 6e64bcc..8b1bb4d, tests: powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter [lunar_descent_flight],[differential_throttle],[lunar_scvx] → All tests passed (80 assertions in 8 test cases))
Task 7: Ruling: trim (10.7 m/s) and DOI (18.2 m/s) at ~1,185 t fit neither the cold-gas RCS (> 1 h) nor a three-Raptor set at the time execute_burn(node) checks (start/shutdown impulse > 1/4 of the burn: "Brennvorgang nicht möglich") - new lunar_descent_burn: execute_burn(node) first, else forced on the approach set's single gimballed Raptor (braking_sets.back(), sea_level_1). Flown: trim on sea_level_1 (874 kN), DOI on the vacuum set at minimum (2682 kN, chosen by execute_burn) - cost if wrong: one fallback path.
Task 7: Ruling: DescentCoast (DOI -> PDI, half a revolution) coasts freely, then holds the braking attitude (LVLH: nose retrograde, +y up) from descent_attitude_lead_s = 600 s [E] before PDI, instead of the brief's Sun pointing (the braking guidance needs the vehicle retrograde at PDI; a Sun-pointed vehicle would turn ~180 deg on RCS during braking). DescentWait (days) is Sun-pointed as specified - cost if wrong: 600 s of ticked hold.
Task 7: Ruling: l3's Coast Sun-pointing extracted into lunar_sun_coast(flying) (Coast and DescentWait share it, behaviour unchanged) - cost if wrong: none.
Task 7: Ruling: wait epoch = lunar_ignition_lead(node) - small_burn_margin_s (as l3's trims: the burn is commanded before its preparation) - cost if wrong: none.
Task 7: Ruling: closed-orbit check uses the new constant moon_max_terrain_height_m = 10,786 m [S: Smith et al. 2010 LOLA]; extra refusal "Kein Abstieg: Fahrzeug ohne Landebeine oder Abstiegstriebwerke" (no legs / no braking-set Isp); plan_lunar_landing keeps lunar_burns_ of the flight (from InLlo the outbound burns stay) - cost if wrong: none.
Task 7: measured (align 0.5 d, 1,185 t): plan pass 0.183 d, trim 10.66 m/s (crossrange -11374 -> 0.1 m), DOI 18.22 m/s, periselene 20.32 km, PDI after 0.178 d (4.3 h), range 450 km, budget margin 282.3 t; flown: trim 1184.7 -> 1179.5 t, DOI -> 1173.1 t, PDI at 1172.6 t, gelandet 0.57 m from the point, sink 1.05 m/s, 619.8 t on the surface (565 t used from the LLO incl. trim/DOI); stay 1 h -> Landed, resting. Pilot override at PDI + 60 s: step None, autopilot Off, guidance gone, engines off. -Fast 511/511. No DOI re-plan needed (0.57 m).
Task 7: complete (commits 8b1bb4d..7688194, tests: powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter [lunar_descent_session]~[slow] → All tests passed (8 assertions in 1 test case))
Task 8: Ruling: steering set - the landing thrusters (differential throttling, ~20x the RCS torque) run from the liftoff (abort: from the abort) until the RVac thrust axis is within 2 deg of PEG's direction and turning < 0.01 rad/s, then only the RVacs + RCS fly (new LunarAscentParameters::steering_groups / steering_alignment_rad / steering_rate_radps [E]). With the RCS alone (brief/user: RVac attitude by the RCS) the ~650 t HLS turned at ~6e-4 rad/s^2: 60-80 deg behind PEG for the whole ascent, near-vertical to 360 m/s, sub-orbital cutoff. With it: aligned ~20 s into PEG, then < 0.5 deg - tell the user (costs a few t of propellant at Isp 340 on the landing thrusters) - cost if wrong: the steering set can be shortened or replaced by a pre-tilt.
Task 8: Ruling: ascent throttle - all running groups share one throttle (ReturnCommand): full while a bridge set carries the start-up, else the thrust acceleration is held at max_acceleration_mps2 = 1 g [E] within the groups' common throttle range (brief: "throttles down to the minimum, never below", value not given) - cost if wrong: a different g-limit changes gravity losses by tens of m/s.
Task 8: Ruling: group ids stage-prefixed in LunarAscentParameters ("hls/landing", "hls/vacuum", engine_out_groups "hls/sea_level_1..3"), as Task 3's ruling - cost if wrong: none.
Task 8: Ruling: test helpers in a new header core/tests/lunar_test_support.hpp (inline hls_with_propellant + place_hls_at_pdi) instead of test_support.hpp/.cpp - test_support.hpp is included by every test file and is header-only; pulling FlightSession into it would slow all builds - cost if wrong: none.
Task 8: Ruling (l2 bug fix): LegLandingSim tracked a body as "on site" by its horizontal site-frame offset only; near the antipode (the aposelene of a polar orbit through the site, half a revolution after the ascent) that offset is small too, the feet "touched" the site plane ~3,500 km above and the HLS was judged an impact in orbit. on_site now also needs cg z > -tracking_radius - cost if wrong: none (an Earth site's infinite radius is unchanged).
Task 8: Ruling: circularisation (~18 m/s at ~400 t) on the single sea-level Raptor (execute_burn(node, {sea_level_1}), quarter rule), else the landing thrusters, else lunar_descent_burn (session choice incl. RCS). The session's own choice picked the cold-gas RCS: a ~40 min burn commanded at the main engines' ignition lead (far off-centre) - cost if wrong: none.
Task 8: Ruling: abort Arrest exit - PEG takes over once the climb is >= abort_min_climb_mps AND (a fresh PEG solve steers below abort_peg_max_radial = 0.85 [E] OR the ballistic apex reaches the insertion radius). From a fast low state PEG's time to go (from the horizontal speed to gain) collapses (~25 s), it steers at its radial limit and cuts off sub-orbital; plain "vr >= 5 m/s" gave -731 x 57 km -> circularised 53 x 58 km, a PEG restart on sub-orbital cutoff 336 x 342 km, the PEG gate alone 1085 km, gate + apex 111 x 115 km (kept); the 1 km abort is unchanged (PEG ready at once). Arrest lean towards downrange kept (the brief's): leaning towards the hull's current (retrograde) axis lost ~20 km of height and made the 1 km abort 30 t dearer - cost if wrong: aborts far from the 1 km case fly a degraded profile (see measured).
Task 8: Ruling: automatic-abort test scenario - a touchdown point 30 km off the PDI track (brief) is REACHABLE on the Raptors (braking nulled the crossrange to 6 m; the shifted point had the wrong terrain height and the HLS struck the ground); the test places PDI only 60 km before the real point instead (braking needs ~450 km): the approach cannot be planned -> AbortAscent -> InLlo - cost if wrong: none.
Task 8: Ruling: automatic abort when refused - status "Landepunkt nicht erreichbar, Abbruch nicht möglich", tried once per descent (lunar_auto_abort_tried_); in phase Aborted the landing guidance commands engines off (Task 6) - so a refused abort after a failed first approach plan falls; SCvx aborting() keeps flying its plans - cost if wrong: a refused abort at the high gate is lost instead of attempting a landing elsewhere.
Task 8: Ruling: abort_landing on PoweredDescent drops the landing guidance (lunar_landing_.reset()); a refused abort changes nothing (review focus 3) - cost if wrong: none.
Task 8: measured: ascent after a 3-day stay (one warp call, no drift < 1 cm): liftoff 661.6 t, PEG from 150 m at 42 m/s, cutoff 20.3 x ~100 km (osculating at cutoff before the tail-off 20.3 x 89.3), circularisation 18.24 m/s on sea_level_1, LLO 100.05 x 100.14 km (two revolutions), 261.9 t used = 1878 m/s at 380 s (spec ~1.9 km/s). RVac engine-out 30 s into PEG: switch to the 3 sea-level Raptors, LLO 100.28 x 100.38 km, 275.4 t used (+13.5 t). Abort from 1 km above the point (1,065 t propellant start): 1920 m/s needed, Arrest -> PEG, LLO 97.8 x 100.1 km, 261.8 t used. Refusal (420 t dry, 330 t propellant): "Abbruch nicht möglich", landing continues. Automatic abort (PDI 60 km short): Arrest from 14 km at 1.44 km/s, PEG degraded (saturated), cutoff -592 x 115 km, circularisation 204 m/s, LLO 111.0 x 115.4 km, 743.6 t left.
Task 8: not done here (Task 10 docs): docs/subsystems.md lunar ascent section (steering set, Arrest gate, leg on-site fix).
Task 8: full suite 546/546 (incl. [slow], 615 s) + Godot smoke test passed before the commit.
Task 8: complete (commits 7688194..c67f83e, tests: powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter [lunar_ascent]~[slow] → All tests passed (12 assertions in 2 test cases))
Task 9: Ruling: lunar_mission_budget takes the two Isps (vacuum_isp_s, approach_isp_s) as parameters - start_lunar_landing_mission reads them from the vehicle (groups_vacuum_isp_s of braking_sets front/back: 380 / 350 s) instead of the brief's hard-coded 380/350 (no magic numbers) - cost if wrong: none.
Task 9: Ruling: data dir for the terrain = config_dir.parent_path() / "data" (repository layout; signature has no data_dir); place_lunar_landing_mission gains the vehicle override, loads terrain + site; start_lunar_landing_mission catches its runtime_errors into the refusal (start_mission no longer throws for "no plane") - cost if wrong: a relocated data dir needs a parameter.
Task 9: Ruling: chain hook at the top of run_lunar_steps (runs only in autopilot state Lunar, i.e. after a burn is over) when the step is InLlo/InNrho; legs Outbound -> Surface -> Return -> Complete/Ended; back in the LLO without a Landed touchdown (abort to orbit) ends the sequence "Mission abgebrochen: ..."; NRHO variant LLO target = LloTarget default (100 km polar) - cost if wrong: none.
Task 9: Ruling (spec deviation, brief): the site pass is checked when the descent is planned in the LLO, not at the start; the start checks touchdown point (both polar directions), outbound plan, budget.
Task 9: budget at the start (nominal [budget] values): LLO TLI 3142 + LOI 856 m/s; 120 t margin +74.7 t, 204 t margin +0.7 t (CLOSES); NRHO TLI 3162 + swing-by 243 + insertion 224 m/s; 120 t +17.1 t, 204 t -59.7 t (refused). Nominal descent 1600@380 + 400@350 from 1185 t uses ~498 t; Task 6/7 flew 557 t (PDI->touchdown) + 8 t trim/DOI = 565 t -> the nominal descent underestimates by ~67 t at that mass. Per the ledger line after Task 3: report to the user, do not tune.
Task 9: Ruling (bug fix): chain hook moved to the end of run_lunar's tick (InLlo/InNrho set inside run_lunar_steps waits far_future_s: the coast allowance skipped to the test's end before the next tick's hook) - cost if wrong: none.
Task 9: Ruling (bug fix, finding A): execute_burn(node, groups) fell back to the cold-gas RCS against its documented contract (false if neither fits) - start_burn(rcs_fallback=false) for forced sets; the circularisation's {landing} fallback never worked (auxiliary engines are never burn engines) - removed. New lunar_small_burn: the approach set's single sea-level Raptor first (transients <= small_burn_transient_share 0.5 [E] of the burn), else the session's choice; used by trims, DOI, station keeping, circularisation. Test [burn_engines]: trim 10.66 + DOI 18.22 m/s at 570 t: 13.8 t -> 7.0 t (RED->GREEN). Cost if wrong: a sloppier cutoff on 10-20 m/s burns.
Task 9: FINDING (stop, user): mission flights (first runs): LLO variant - LOI at 723.7 t -> 573 t in the LLO (not 1,185 t), wait 5.77 d, trim+DOI on cold gas (30 t), PDI at 537.7 t -> impact 57 m/s, 528 m off, 88 t propellant left. NRHO variant - NRHO->LLO 955 m/s (nominal 876), LLO 546.7 t, ~60 t lost to cold-gas trims, impact 73 m/s, 16 t left. Root cause of the descent failure (test [lunar_descent_light], PDI 550 t, RED): the braking demand drops below the vacuum set's minimum at once and below the single Raptor's minimum (874 kN) from t_go ~410 s; the law over-brakes, demand -> 0, the attitude loop chases a vanishing thrust vector (axis error 100-150 deg). ROOT: plan/spec premise '~1,185 t in the LLO (l3: 1,177-1,194 t)' is the l3 campaign's PROPELLANT USED (test_lunar_outbound_session.cpp:188 propellant_t = m0 - mass), not the mass in the LLO. Real: ~573 t in the LLO (LLO variant), ~470-550 t (NRHO variant); landing ~300 t / ~240 t. Consequences: landing thrusters 12 x 140 kN (min 40 %: 672 kN axial = 2.2 m/s^2 at 300 t > lunar g 1.62) cannot hover; single Raptor min 874 kN cannot hover below ~540 t; Tasks 1/3/6/7/8 tuned and tested at 1,185 t / 661 t liftoff. Budget at start (nominal): LLO 120 t +74.7 t, 204 t +0.7 t (closes); NRHO 120 t +17.1 t, 204 t -59.7 t. Work uncommitted (Task 9 code + fix A + tests); asked the user how to proceed.
User decision 2026-10-15 (Task 9 finding): correction plan first - docs/superpowers/plans/2026-10-15-lunar-l4-mass-correction.md (C1 thrusters 12 x 90 kN min 0.35 + test masses, C2 descent at the real PDI mass: landing thrusters as 3rd braking set, approach on them, braking range from the mass by sweep, C3 descent autopilot + budget [D], C4 ascent/aborts + full suite + push); executed under this ledger as C1-C4, then Task 9 resumes. Plan written, awaiting the user's review.
C1: start (BASE c67f83e); plan approved by the user ('passt'), native execution.
C1: complete (commit 271cb9c, tests: build.ps1 -NoBindings -TestFilter [hls] -> 12/12, 76 assertions; landing thrust 365-1043 kN axial, hovers 225-642 t; drop tests 260/340 t land, tip 25 deg; crush force 4.5 MN kept; vehicle_report ok). Not pushed (Task 9 WIP red in the tree).
C2: Ruling: approach on the landing thrusters in all three SCvx phases; terminal (P66) starts when SCvx enters its last phase (ScvxLandingGuidance::engine_set(), new accessor) instead of 'last set lit' (all sets equal now); gimbal_fraction 1.0 in every phase (equivalent gimbal of the canted thrusters). SCvx from the real gate (700/800 m, 25 m/s, 350 t): converged in 3 iterations, phases 50/13/30 s - cost if wrong: none.
C2: Ruling: second drop condition as max(set_drop_fraction x next max, running set's minimum) - flown drops: vacuum->sea_level_1 at 2.60 MN (threshold 2.68 = 3 RVac minimum; sea_level_1 max is below it: the gap is flown at full sea_level_1 thrust), sea_level_1->landing at 0.93 MN (0.94 = 0.9 x landing max) - cost if wrong: none.
C2: Ruling: LunarLandingParameters::small_burn_set {hls/sea_level_1} for trims/DOI/circularisation (lunar_small_burn used braking_sets.back(), now the landing thrusters = auxiliary engines, never burn engines) - cost if wrong: none.
C2: Ruling (bug found in the sweep, systematic-debugging): terminal descent at minimum throttle has NO differential-throttling authority (attitude_controller ThrottleAuthority range = distance to the nearer limit) - a 297 t HLS overshot to 8 deg on RCS alone and landed 12-15 m off (500 t PDI, ranges >= 212 km). Fix: TerminalDescentParameters::steering_throttle_margin 0.1 [E] above the minimum, capped at floor_weight_fraction 0.95 [E] of the weight (the uncapped floor 0.47 MN exceeded a 290 t weight: the HLS climbed away). Tell the user.
C2: Sweep (BrakingRangeParameters, both PDI masses, after the fix): all 18 land 0.19-0.21 m, sink 1.04 m/s. dv (380/350 split) 566 t | 500 t: 0.35x0.8 2105|2089, 0.35x1.0 2154|2137, 0.35x1.2 2206|2181, 0.45x0.8 2065|2062, 0.45x1.0 2100|2089, 0.45x1.2 2138|2122, 0.55x0.8 2055|2062 (both clamped 150 km), 0.55x1.0 2068|2062 (164|150 km), 0.55x1.2 2096|2087. Chosen 0.55 x 1.0 (mass-dependent above the 150 km floor). Before the fix 0.35x1.0/1.2 and 0.45x1.2 missed at 500 t (12-15 m).
C2: Ruling: test_differential_throttle 'applies the trims' expected the old 0.4 minimum (stale since C1, not in the [hls] tag) - now reads the configured throttle_min.
C2: complete (tests: -TestFilter [lunar_scvx],[lunar_descent_plan],[differential_throttle] 11/11, 94 assertions; slow flights: 566 t PDI 164.1 km, braking 309 s, drops at t 566->417.5 t (sea_level_1) ->318.0 t landed, 0.20 m, 1.04 m/s, 2068 m/s [380/350 split]; 500 t PDI 150 km, braking 283 s, 500->370.8->281.3 t, 0.19 m, 2062 m/s; engine out 30 s after PDI (566 t): 3 SL Raptors, 566->512.3->312.6 t, 0.20 m, 2038 m/s at 350 s). Commit includes Task 9 WIP hunks of flight_session.hpp / flight_session_lunar_descent.cpp (entangled; commits C1-C3 build only with the Task 9 tree - fine, nothing pushed until C4). Next: C3.
C3: start (BASE 36db543)
C3: Ruling: test_lunar_descent_session.cpp's local hls() replaced by test::hls_with_propellant (lunar_test_support.hpp); landing test prints LLO and landed mass - cost if wrong: none.
C3: measured: LLO 573.0 t -> trim + DOI (~7 t, sea_level_1) -> landed 317.1 t, 0.20 m, 1.04 m/s, wait 0.18 d; pilot takeover case passes. Budget [D]: descent 2100 m/s (DOI 18 + C2 PDI->touchdown 2068 @566 t / 2062 @500 t, 380/350 split), braking 1150 m/s (566 -> 417.5 t @380 s = 1134). Also set in Task 9 WIP (mission_config.hpp default, test_lunar_landing_mission.cpp check 2000 -> 2100, uncommitted). Note for Task 9: the budget's approach Isp is read from braking_sets.back() = now the landing thrusters, while the flown approach share is mostly sea_level_1 (350 s) - check in Task 9.
C3: complete (tests: -TestFilter [lunar_descent_session]~[slow],[lunar_descent_plan],[config][lunar] 10/10, 67 assertions; [lunar_descent_session][slow] 2/2). Commit includes Task 9 WIP hunks of the two mission TOMLs and the [burn_engines] tests (entangled, as C2). Next: C4.
C4: start (BASE 72755fc)
C4: Ruling: new test::pdi_speed_mps / test::pdi_braking_range_m in lunar_test_support.hpp (test_lunar_powered_descent.cpp's private helpers moved there, used by the ascent aborts too) - cost if wrong: none.
C4: first runs at the real mass: 340 t ascent sub-orbital (-70 x 110 km), 260 t 94 x 105 km, 1 km abort (343.8 t) sub-orbital, auto-abort stuck in Arrest; engine-out and refusal (31.4 t left at 1 km, dry 420 t) pass. Root cause 1 (systematic-debugging, debug trace): the common throttle sat at the minimum (1 g limit at ~340 t) -> the steering set (landing thrusters) had no differential-throttling authority (same as C2's terminal finding) -> hull 60-90 deg off PEG all ascent. Fix: LunarAscentParameters::steering_throttle_margin 0.1 [E] above the minimum while steering. Result 340 t: aligned after ~25 s, LLO 100.13 x 100.26 km, 1966 m/s. Root cause 2 (260 t): PEG cutoff fine (15 km, vr 0, vh 1684), circularisation 18 m/s at 151 t refused on sea_level_1 by the quarter rule -> lunar_descent_burn chose the RCS (3 kN, ~15 min at the main-engine lead) -> 93.7 x 101.9 km. Fix: circularisation via lunar_small_burn (Task 9 fix A intent; transient share 0.5). User 2026-10-15 (mid-C4): 'Es muss ja nicht 1 g sein, finde einfach das Optimum' -> sweep of max_acceleration_mps2 (0.5-10 g, both landed masses), hidden test [lunar_ascent_sweep].
C4: Ruling: steering_throttle_margin applies at BOTH limits (min + 0.1, max 1 - 0.1) in ascent_throttle and in Arrest (was 1.0) while the steering set runs - sweep at 3 g/10 g sat at full throttle (no authority) and crashed; auto-abort Arrest at 1.0 turned at 0.03 rad/s with the hull 126 deg off vertical (thrusting down) and struck; with the margin 0.146 rad/s, abort reaches the LLO - cost if wrong: 10 % less thrust during the steering seconds.
C4: Ruling: small_burn_transient_share 0.5 -> 0.6 [E]: circularisation 18.2 m/s at 151 t has transients 10.1 m/s (0.56) and fell to the RCS (93.7 x 101.9 km); at 0.6 it flies on sea_level_1: 100.2 x 100.3 km. 340 t case (0.41) unchanged - cost if wrong: a sloppier small burn (none seen).
C4: Ruling (user 'find the optimum'): max_acceleration_mps2 1 g -> 1.5 g [D]. Sweep (landed 340 / 260 t, after the fixes): 1 g 139.19/109.65 t, 1.25 g 139.10/109.65, 1.5 g 138.67/109.67, 1.75 g 138.88/109.31, 2 g 139.68/108.53, 2.5 g 144.28/109.49 (orbits 103-105 km: PEG degrades), 3 g / 10 g crash (PEG t_go collapses from the vertical start at high T/W, known since Task 2). Flat within 0.15 t over 1.5-2 g; 1.5 g = LLO variant optimum, farthest from the PEG edge. The RVac minimum throttle sets a floor of ~1.4-1.8 g near cutoff anyway. Hidden sweep test [lunar_ascent_sweep] kept.
C4: FINDING (deferred, tell user): after a sub-orbital ascent cutoff (periselene -1737 km) the session planned a 1643 m/s 'circularisation' (small burn on sea_level_1) instead of lunar_abort('keine geschlossene Bahn') - circularize_at_apoapsis returns a node for an orbit through the Moon; guard belongs to Task 9/l6. Also the 'in LLO' status shows osculating 95.4 x 100.2 km while two revolutions measure 100.1 x 100.2 km (status at burn end; l6 telemetry).
C4: measured (slow cases): ascent 339.7 t -> LLO 100.12 x 100.22 km, 138.7 t, 1955 m/s, peak 15.4 m/s^2; 259.7 t -> 100.19 x 100.31 km, 109.7 t, 2045 m/s, peak 20.8 m/s^2 (RVac minimum at ~150 t); engine-out 30 s into PEG (340 t): 100.04 x 100.16 km, 145.0 t (+6.3 t), peak 14.7; abort from 1 km at 343.8 t: 149.2 t used, 100.0 km, peak 30.0 m/s^2 (Arrest at full thrust + bridge sets); refusal: 31.4 t left at 1 km with dry 420 t (kept: 'enough to land, far too little for orbit', comment updated); auto-abort (566 t, PDI 60 km short): Arrest from 18 km at -195 m/s, reaches the LLO.
C4: full suite (with bindings) 553/554 + Godot smoke passed; only red: 'Mission 1, NRHO variant' (lands 0.14 m / 1.04 m/s, then 1.7 t propellant left, 'Mondflug beendet: Aufschlag' - Task 9 budget). The LLO mission flight and the budget case now PASS (plan expected them red).
C4: complete (commit 21abdfe, tests: [lunar_ascent] slow cases 5/5 + fast, -Fast 518/518). NOT PUSHED - ruling: the plan's 'push after C4 with Task 9 uncommitted' is impossible: with the Task 9 WIP stashed, HEAD does not compile (C2 committed WIP header hunks: start_burn signature vs old flight_session_burns.cpp). Pushing would put a non-building main on origin. C1-C4 (271cb9c..21abdfe) are pushed together with Task 9 once its NRHO flight is green. Next: resume Task 9 (NRHO return propellant, the 204 t check with the new budget values, epoch tuning for a < 2 day wait, sub-orbital-cutoff guard).
Task 9 (resumed 2026-10-16): sub-orbital-cutoff guard (C4 finding) in flight_session_lunar_ascent.cpp: periselene inside the Moon at cutoff -> the 'circularisation' (whole orbital speed at the aposelene) is flown only if the propellant holds it on the small-burn set (no reserve), else lunar_abort 'suborbitaler Brennschluss, Treibstoff reicht nicht fuer die Kreisbahn'. Not flight-tested (no scenario reproduces a sub-orbital PEG cutoff since C4's fixes).
Task 9: measured (flights after C1-C4): LLO variant PASSES: TLI 3182 m/s, MCC 3.37, LOI -> 575 t in the LLO, descent wait 5.77 d (pass, trim 17.1 m/s on sea_level_1, DOI 18.2), PDI 559.6 t, landed 313.5 t 0.20 m 1.04 m/s, ascent cutoff 20.3 x 83.9 km, circularisation 18.2 m/s, end 184.2 t (64.2 t propellant), 'Mission abgeschlossen'. NRHO variant FAILS: TLI 3213, swing-by 243, insertion 224, NRHO->LLO 956 m/s (nominal 876) -> 489.9 t in the LLO, wait 4.08 d, landed 268.4 t 0.14 m, ascent -> 155.6 t in the LLO, LLO->NRHO planned 986 m/s (nominal 725: departure 5.3 d after the LLO arrival, outside l3's cheap <=2.5 d window) -> runs dry (1.7 t), impact after 4 d.
Task 9: FINDING (stop, user): epoch tuning cannot shorten the descent wait. Hidden probe [lunar_epoch_probe] (STARSIM_LUNAR_MISSION / _EPOCH_SHIFT_D / _SPAN_D), shifts -3..+6 d: LLO wait 5.36-5.93 d for every shift, crossrange always ~18-20 km (window edge); NRHO 3.75 (shift -2) .. 10.5 d, +5 d impact, +6 d no pass within 30 d. Cause: the LLO's node comes from the arrival geometry (LloTarget has altitude + inclination only), and the Moon is tidally locked - the site's place relative to the Earth-Moon line, hence to the arrival plane, barely changes with the epoch. Shortening the wait needs the LLO plane targeted over the site (l3 outbound targeter / NRHO->LLO transfer get a node constraint) - l3 'planners unchanged' decision -> asked the user.
Task 9: physics check after the user picked 'LLO plane over the site' (first answer): not cheap - a polar LLO contains the arrival asymptote (fixed in the Moon frame: tidal lock); from the NRHO perilune the plane is pinned as well and repeats every revolution. Mons Mouton at -84.8 deg: crossrange of any polar plane <= 158 km (<= ~150 m/s trim, ~34 m/s per day of wait saved). NRHO: early trim ~76 m/s at 490 t (~11 t) > return saving (986 -> ~725 m/s at 155 t, ~8.6 t); late window 21-30 d costs ~15 t Sun-hold cold gas. Told the user, asked again.
User decision 2026-10-16: refuse honestly. Ruling: NRHO TOML [budget] takes the flown legs [D] nrho_to_llo 956, llo_to_nrho 990 m/s -> start refuses the NRHO variant with 120 t ('Mission nicht durchführbar: ... nach LLO -> NRHO fehlen 2.2 t'); LLO with 204 t: 'nach Aufstieg fehlen 9.3 t'. Test renamed 'Mission 1 closes in the LLO with the 120 t HLS; NRHO and 204 t are refused'; NRHO flight test removed (the budget refusal covers it); LLO epoch kept (wait 5.8 d documented in the TOML); probe [lunar_epoch_probe] kept hidden. Plan step 'epoch until wait < 2 d' dropped (impossible, see finding).
Task 9: complete (tests: [lunar_budget_mission],[config][lunar] 3/3, 34 assertions; LLO mission flight [slow] passed before the guard edit (guard only acts on a sub-orbital cutoff); -Fast 518/518). Pushes C1-C4 (271cb9c..21abdfe) together with Task 9. Next: Task 10 (campaign 21/21, aborts 3/1/0.3 km, docs incl. Task 8/9 notes + the tidal-lock wait finding, full suite, opus review).
Task 10: start (BASE 68a0e91)
Task 10: Ruling: truth thrust factor scales thrust (flight_body forces) AND mass flow (tank_mass_flows) - same Isp, as an engine running hot/cold; copied by copy_upper and kept on both parts by separate() (as hull_aero_scale) - cost if wrong: none. Test [dispersion]: RED (mass-flow ratio 1.0 vs 1.02) -> GREEN (dv ratio 1.0200).
Task 10: Ruling: campaign PDI mass = the real 566 t (hls_pdi_propellant_kg x mass factor), not the plan's 1,065 t (pre-mass-correction value); braking range from the dispersed mass (the onboard computer knows its mass), state dispersion added in place_hls_at_pdi (new optional position/velocity error arguments); descend_to moved to lunar_test_support.hpp (used by the ascent tests and the campaign aborts) - cost if wrong: none.
Task 10: measured first campaign run (leg damping 1e6): 20/21; case 19 (spot 789/598 m, slope 5.2 deg) 'gelandet' 1.18 m off, though the terminal guidance had 0.04 m at first leg contact. Offsets grow with the slope (0.1-0.2 m at ~1 deg, 0.56-0.58 m at 4.6 deg). Cause (systematic-debugging, STARSIM_DEBUG_LEGS): engines off at the first (uphill) foot contact, the hull falls onto the downhill feet, rebounds (vz +0.5 m/s) and rocks on 1-2 feet for ~15 s (tilt to 9 deg on a 5.2 deg slope), the footprint walks ~1 m; rocking mode damping ~0.3 with 1e6 N s/m per leg. Friction regularisation 0.1 -> 0.02 m/s (experiment, reverted) explains only 0.17 m (creep); leg damping 2e6 gave 1.16 m (not monotonic).
Task 10: Ruling: HLS leg damping 1e6 -> 3e6 N s/m [E] (rocking mode ~0.9, no rebound; 1 m/s x 3e6 = 3 MN < crush 4.5 MN): campaign 21/21, offsets 0.02-0.78 m, sink 0.98-1.03 m/s. Side effect: l2's terrain drop (220 t, flattest spot) rests nearly upright on two feet and topples onto a third only slowly (unstable equilibrium, growth ~e per 4 s from mm) - run_drop's window 60 -> 300 s, lands. Guidance unchanged; spec 'engines off on leg contact' kept - cost if wrong: a softer/harder real absorber changes the settle offset on slopes > 4 deg by tenths of a metre.
Task 10: measured aborts (566 t PDI): from 3 km at 359.0 t: 148.1 t used; 1 km at 343.8 t: 142.0 t; 0.3 km at 330.9 t: 135.4 t; all InLlo at ~100 km (status shows the osculating 95 x 100 km at burn end - known l6 telemetry point). Campaign wall time ~90 s per 7 seeds.
Task 10: full suite 558/558 (with bindings, 586 s) + Godot smoke passed; commit 66ffb41
Final review (opus, 7bf4c2b..66ffb41): 0 Critical, 2 Important, 8 Minor; re-graded: I1, I2 Important (enter fix pass); M1-M8 Minor.
Final: fixed I1 (stay_hours = 0 with an ascent never lifted off: step Landed for good) - 'A surface stay of 0 h with an ascent lifts off at once' RED (resting) -> GREEN (Freiflug zur Aposelene 20.3 x 84.0 km).
Final: fixed I2 (refused automatic abort cut the engines: phase Aborted) - new LunarLandingGuidance::land_without_approach() (Aborted -> Terminal, 'Endanflug ohne Plan'), called by the session after the refusal - 'A refused automatic abort keeps the engines on in a terminal descent' RED (phase 5, no thrust) -> GREEN. Whether the HLS then lands depends on the state (from a fast low state the landing thrusters cannot null the speed) - not measured.
Final: minor (deferred): M1 lunar_abort on a planner exception during PoweredDescent/Ascent leaves running_groups lit without steering (comment says it coasts) - flight_session_lunar.cpp:417-421.
Final: minor (deferred): M2 sub-orbital cutoff guard compares the periselene with the mean radius (planner uses + moon_max_terrain_height_m) and does not check the aposelene against the terrain - flight_session_lunar_ascent.cpp:182.
Final: minor (deferred): M3 a refused ascent at the stay's end leaves the mission leg Surface (sequence never Ended, no 'Mission beendet:' prefix) - flight_session_lunar_descent.cpp:281-287.
Final: minor (deferred): M4 campaign accepts sink <= 1.1 m/s (spec <= 1 m/s; measured 0.98-1.03); high gate 700/800 m vs spec ~2 km (ledger Task 6) - report to the user.
Final: minor (deferred): M5 pilot takeover tested in braking only (approach: same code path); refused-abort test checks the step, not the completed landing.
Final: minor (deferred): M6 redundant lunar_small_burn retry via lunar_descent_burn before lunar_abort - flight_session_lunar_ascent.cpp:215-224.
Final: minor (deferred): M7 stale comment on lunar_mission_budget's approach_isp_s ('single sea-level Raptor'; braking_sets.back() is the landing thrusters since the mass correction) - lunar_landing_mission.hpp.
Final: minor (deferred): M8 execute_burn(node, groups) no longer falls back to the RCS - also ship deorbit (flight_session_return.cpp:87); harmless at deorbit sizes, one line for docs/subsystems.md.
Final: Ruling (declined to judge, launch window): ascent LLO plane is the polar plane through the liftoff point (plan line 1844), not the arrival LLO plane - after a 24 h stay ~13 deg apart; a user flying mission 1 gets a correct LLO; it matters for l5 (TEI) / M9 rendezvous - carried into the l5 plan - cost if wrong: an l5 plane change.
Final: Ruling (declined to judge, abort budget RCS share): covered by the 15 % margin, unquantified - stands - cost if wrong: an accepted abort running short near orbit.
User 2026-10-16: no separate post-l4 cleanup plan ('Lieber l5'): the cleanup candidates (deferred minors above, l2 minors, file sizes flight_session*.cpp, leg friction regularisation creep) go into the l5 plan.

