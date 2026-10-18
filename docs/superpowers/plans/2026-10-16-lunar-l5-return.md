# Lunar l5: Ship Return from Lunar Orbit to Starbase - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Mission 2 of the lunar spec: a Block 3 ship in the 100 km polar LLO or the NRHO, tanked by the mission
planning with just the propellant its return needs, flies TEI, mid-course corrections and a braking burn, and then either
enters directly and is caught at Starbase, or aerocaptures into an Earth orbit, raises its perigee and is caught by b4.

**Architecture:** New core pieces bottom-up: radiative heating from the NASA equilibrium tables (`aero`), a predictor that
carries skip arcs and reports the exit orbit, an aerocapture mode and a skip-carrying mode of the entry guidance and the
entry corridor (`gnc`), the Earth-return targeter and planner with the braking burn modelled as an impulse (`mission`),
the mission config, start state and propellant fixed point (`config`, `sim`), and the session autopilot steps that fly
it (`sim/flight_session_lunar_return.cpp`). The session reuses the existing machinery: the lunar coast (Sun hold, burns
on the vacuum Raptors), the b2 entry guidance + b3 flip and catch, and b4's deorbit planning.

**Tech Stack:** C++20 core (float64, SI), Catch2 tests via `tools/build.ps1`, TOML/CSV config, SPICE (DE440) only on the
simulation thread.

**Spec:** `docs/superpowers/specs/2026-10-14-lunar-missions-design.md`, section "l5 design: return to Earth" with its
amendment of 2026-10-16 (braking burn). Read both.

## Global Constraints

- SI units internally, angles in radians; frame and unit in every name (`position_gcrf_m`, `velocity_itrf_mps`).
- `core` has no Godot dependency; modules depend downwards only (`aero` < `gnc` < `mission` < `sim`).
- No magic numbers: physical constants in `core/include/starsim/math/constants.hpp` with their source; every estimate is
  tagged `[E]`, derived `[D]`, sourced `[S]` in code comments and TOML.
- **Heat-shield limits are not raised** (spec): `max_heat_rate_w_m2 = 1.2e6`, `max_heat_load_j_m2 = 6.0e8`,
  `max_load_g = 6.0`, `nose_radius_m = 6.0` stay as they are in `config/vehicles/starship_block3.toml`.
- **User decision 2026-10-16:** the ship brakes propulsively before the entry interface (vacuum Raptors, retrograde, at
  ~2,000 km altitude [E]) to the highest entry speed whose corridor fits the limits [D: computed], then direct entry or
  single-pass aerocapture. No propellant is ever set mid-mission (lunar spec): the start state carries what the plan
  needs.
- Mission start refuses with a German message (`MissionStart::refusal`): no entry corridor, propellant beyond the tanks,
  TEI not plannable.
- UI strings in the core are German; code, comments, docs English (UTF-8 sources).
- LEO entries (b2, b4, d3 campaigns) must stay unchanged in outcome; radiative heating is zero below 8 km/s, so they stay
  bit-identical.
- CSPICE is not thread-safe: planners run on the simulation thread; the entry predictor stays pure (worker-safe).
- Test workflow (CLAUDE.md): during a task only the touched non-slow tags (`-TestFilter`); before every push `-Fast`; the
  full suite in Task 10 (this plan touches guidance, flight sessions and vehicles). Never filter `[mission]` or
  `[outbound]` alone (serial, pulls in slow campaigns, > 27 min). New cases > ~40 s are tagged `[slow]`.
- Commit and push on `main` after every task (CLAUDE.md, no feature branches); Conventional Commits; attribution lines
  from the session's system reminder.
- One task per session: record the state in the ledger (`.superpowers/sdd/2026-10-16-lunar-l5-return/progress.md`),
  commit, push, ask the user for `/clear`.

## Review Focus

1. **A ship whose LLO plane is not aligned with the return asymptote** (the chain from mission 1: after a 24 h stay the
   ascent LLO is ~13 deg off the arrival plane). Expected: the TEI planner either finds the next window within its search
   span and reports the cost, or refuses with a German reason - never a diverged burn. Test: Task 6
   ("TEI from a misaligned LLO waits for the window or refuses").
2. **The configured entry speed has no corridor** (a lighter / heavier ship, a mission file with a higher speed).
   Expected: the mission start refuses ("Kein Eintrittskorridor ..."), it never launches the ship into a certain
   "Hitzeschild versagt". Test: Task 7 ("Lunar return start refuses without an entry corridor").
3. **The braking burn falls short** (thrust -2 %, an aborted burn): the ship enters faster than planned. Expected: the
   entry guidance cuts the bank back on the predicted heat rate; if the corridor is left the outcome is "zerbrochen" with
   "Hitzeschild versagt" - honestly, no limit bent. Test: Task 10 campaign case with braking execution -1 % / 0.5 deg
   and the hidden single case with a short braking burn.
4. **Aerocapture exit far from the target** (density +15 %, aero -10 %): the apogee is off by hundreds of km. Expected:
   the perigee raise is planned from the actual exit orbit (not the plan), and a capture that stays in the atmosphere or
   exits hyperbolic ends with its cause in the status. Test: Task 9 ("Aerocapture outcome names a capture that stays in
   the atmosphere") and the Task 10 campaign.
5. **Time warp across the braking ignition and the entry interface**: `advance_to` with long coasts must stop for the
   braking burn's preparation and before the entry interface. Test: Task 8 mission flight advances in 1-day calls and
   still brakes at the planned height (+-50 km).

---

## File Structure

| File | Responsibility |
|---|---|
| `config/aero/radiative_heating_earth.csv` (new) | NASA TM-102652 equilibrium radiative heating tables 1-10 [S] |
| `core/include/starsim/aero/radiative_heating.hpp`, `core/src/aero/radiative_heating.cpp` (new) | Table load + interpolation |
| `core/include/starsim/aero/entry_heating.hpp`, `core/src/aero/entry_heating.cpp` | `entry_heat_rate_w_m2`: convective + radiative |
| `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp` | Truth heating incl. radiative; air density factor |
| `core/include/starsim/environment/reference_body.hpp` | `air_density_factor` (truth dispersion) |
| `core/include/starsim/gnc/entry_predictor.hpp`, `core/src/gnc/entry_predictor.cpp` | Radiative in the prediction; skip arcs; exit orbit |
| `core/include/starsim/gnc/entry_guidance.hpp`, `core/src/gnc/entry_guidance.cpp` | Lunar-return (skip) and aerocapture modes, phase `Exit` |
| `core/include/starsim/gnc/entry_corridor.hpp`, `core/src/gnc/entry_corridor.cpp` (new) | Corridor boundaries, highest admissible entry speed |
| `core/include/starsim/gnc/burn_node.hpp`, `core/src/gnc/burn_node.cpp` | `raise_periapsis_at_apoapsis` |
| `core/include/starsim/mission/earth_return.hpp`, `core/src/mission/earth_return.cpp` (new) | Return targeter + planner (TEI, MCC, braking) |
| `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp` | `MissionKind::LunarReturn`, `LunarReturnMission` |
| `config/missions/ship_lunar_return_llo.toml`, `ship_lunar_return_nrho.toml`, `ship_lunar_aircatch_llo.toml` (new) | Missions |
| `core/include/starsim/sim/lunar_return_mission.hpp`, `core/src/sim/lunar_return_mission.cpp` (new) | Start state, propellant fixed point, start |
| `core/src/sim/mission_start.cpp`, `core/include/starsim/sim/mission_start.hpp` | Dispatch `lunar_return` |
| `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session_lunar_return.cpp` (new), `flight_session.cpp` | Return autopilot steps |
| `core/src/sim/flight_session_lunar.cpp` -> + `flight_session_lunar_coast.cpp` (split, Task 1) | Lunar coast / MCC events moved out |
| Tests (new): `test_radiative_heating.cpp`, `test_entry_skip.cpp`, `test_entry_corridor.cpp`, `test_earth_return.cpp`, `test_lunar_return_mission.cpp`, `test_lunar_return_session.cpp`, `test_lunar_return_campaign.cpp`, `lunar_return_test_support.hpp` | |

---

### Task 1: Post-l4 cleanup (deferred minors, file split)

The user chose (2026-10-16, "Lieber l5") to fold the post-l4 cleanup into l5. Each item is small; items with behaviour
get a test.

**Files:**
- Modify: `core/src/sim/flight_session_lunar.cpp` (split), `core/src/sim/flight_session_lunar_ascent.cpp`,
  `core/src/sim/flight_session_lunar_descent.cpp`, `core/include/starsim/sim/lunar_landing_mission.hpp`,
  `core/src/sim/flight_session_lunar_coast.cpp` (new), `core/CMakeLists.txt` (source list),
  `docs/subsystems.md`, `core/tests/test_lunar_session.cpp`, `core/tests/test_engine_placement.cpp`,
  `core/tests/test_lunar_ascent.cpp`, `core/tests/test_lunar_descent_session.cpp`, `core/src/sim/lunar_site.cpp`
  (comment), `core/src/gnc/j2_insertion.cpp` (comment)

**Interfaces:**
- Consumes: nothing new.
- Produces: `flight_session_lunar_coast.cpp` holding `lunar_enter_coast`, `lunar_coast_event`, `lunar_sun_coast`,
  `lunar_pilot_cue`, `complete_pilot_correction`, `lunar_next_mcc_epoch` (moved unchanged). Task 8 adds
  `flight_session_lunar_return.cpp` next to them.

- [ ] **Step 1: Split `flight_session_lunar.cpp`** (650 lines). Move, unchanged, the functions listed above into the new
  `core/src/sim/flight_session_lunar_coast.cpp` with a header comment `// FlightSession: lunar coast - Sun hold,
  mid-course corrections, pilot-flown corrections.` and the anonymous-namespace helpers they use (`german`, `first_after`
  - move them into a private header `core/src/sim/flight_session_lunar_detail.hpp` as `inline` functions in
  `starsim::sim::detail`, and include it from both files). Add the new file to the `starsim_core` source list in
  `core/CMakeLists.txt` next to `flight_session_lunar.cpp`. Trim includes by direct use (post-l3 ruling: not by "build
  stays green").

- [ ] **Step 2: Build and run the lunar session tags**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[lunar]~[slow]"`
Expected: all pass (pure move).

- [ ] **Step 3: M1 - planner exception during a powered phase leaves engines lit.** In `run_lunar`'s `catch` (now in
  `flight_session_lunar.cpp`), before `lunar_abort(...)`, cut the engines when the step was `PoweredDescent`, `Ascent`,
  `AbortAscent` or `Circularize`:

```cpp
    } catch (const std::exception&) {
        // A planner that throws (integrator step underflow on a trial arc, ephemeris range) would leave the step due
        // and throw again on every tick: the flight ends. Engines of a powered phase are cut (review l4 M1: they stayed
        // lit without steering); the vehicle then coasts or falls.
        const bool powered = lunar_step_ == LunarStep::PoweredDescent || lunar_step_ == LunarStep::Ascent ||
                             lunar_step_ == LunarStep::AbortAscent || lunar_step_ == LunarStep::Circularize;
        if (powered) cutoff();
        lunar_abort("Mondflug abgebrochen: Planung fehlgeschlagen");
    }
```

  No test: a planner exception cannot be provoked from a test without a fault hook (ruling recorded in the ledger).

- [ ] **Step 4: M2 - sub-orbital cutoff guard.** In `flight_session_lunar_ascent.cpp` (the guard added in l4 Task 9,
  near line 182) compare the periselene with `constants::moon_mean_radius_m + constants::moon_max_terrain_height_m`
  (the planner's clearance) instead of the mean radius, and also refuse when the aposelene is below that radius plus
  the ascent's insertion height (`ascent_parameters.insertion_altitude_above_site_m`):

```cpp
    const double clearance_m = constants::moon_mean_radius_m + constants::moon_max_terrain_height_m;
    const bool suborbital = periselene_m < clearance_m || aposelene_m < clearance_m;
```

  Test (`test_lunar_ascent.cpp`, tag `[lunar][ascent]`): a unit test of the guard's predicate. Extract it as a free
  function in the anonymous namespace? No - make it a `static bool FlightSession::lunar_orbit_clears_terrain(double
  periselene_m, double aposelene_m)` (public, documented: "a closed lunar orbit clear of the highest terrain") and test:

```cpp
TEST_CASE("A lunar orbit must clear the highest terrain on both apsides", "[lunar][ascent]") {
    const double r = constants::moon_mean_radius_m;
    CHECK(sim::FlightSession::lunar_orbit_clears_terrain(r + 15e3, r + 100e3));
    // moon_max_terrain_height_m = 10,786 m (constants.hpp)
    CHECK_FALSE(sim::FlightSession::lunar_orbit_clears_terrain(r + 5e3, r + 100e3));   // periselene below the highest terrain
    CHECK_FALSE(sim::FlightSession::lunar_orbit_clears_terrain(r + 15e3, r + 5e3));    // aposelene below it
    CHECK_FALSE(sim::FlightSession::lunar_orbit_clears_terrain(r - 1737e3, r + 84e3));  // the C4 sub-orbital cutoff
}
```

- [ ] **Step 5: M3 - refused ascent at the stay's end.** In `flight_session_lunar_descent.cpp` (lines ~281-287, the
  stay's end calls `plan_lunar_ascent`): when it returns false and a mission sequence runs, end it:

```cpp
            if (!plan_lunar_ascent(*lunar_landing_setup_->ascent)) {
                if (lunar_mission_) {
                    lunar_mission_leg_ = LunarMissionLeg::Ended;
                    lunar_status_ = "Mission beendet: " + lunar_status_;
                }
                ...existing handling...
            }
```

  Test (`test_lunar_descent_session.cpp`, `[lunar][session]`): place the HLS landed (existing helper of the stay tests)
  with a mission sequence whose ascent cannot close (dry mass 420 t as in C4's refusal), stay 0.01 h, advance 10 s:
  `CHECK(session.lunar_mission_leg() == sim::LunarMissionLeg::Ended)` (add the accessor
  `LunarMissionLeg lunar_mission_leg() const` if missing) and `CHECK(session.lunar_status().rfind("Mission beendet: ", 0) == 0)`.

- [ ] **Step 6: M5 - tests.** Add to `test_lunar_descent_session.cpp` a pilot takeover in the approach phase (copy the
  braking-phase takeover test, start the takeover when `lunar_landing_guidance()->phase()` is the approach phase) and
  extend "A refused automatic abort keeps the engines on in a terminal descent" to fly on until the outcome and record it
  in the ledger (CHECK only that an outcome is set: landed or impact).

- [ ] **Step 7: M6 / M7 / M8 and the post-l3 minors (no behaviour change).**
  - M6: remove the redundant `lunar_descent_burn` retry before `lunar_abort` in `flight_session_lunar_ascent.cpp`
    (~215-224): `lunar_small_burn` already chooses the set.
  - M7: `lunar_landing_mission.hpp`, `lunar_mission_budget` comment: `approach_isp_s` is the last braking set's Isp (the
    landing thrusters since the mass correction), not "the single sea-level Raptor".
  - M8: `docs/subsystems.md`, ship return paragraph: "`execute_burn(node, groups)` flies on the named groups only (no
    RCS fallback; the deorbit burn is far above the RCS's reach)".
  - Post-l3: MCCs are planned `burn_preparation_s` ahead without `small_burn_margin_s` - document the exemption in
    `lunar_coast_event` ("an MCC is < 240 s of burn: the preparation covers it"); `min_departure_lead_s` comment: "margin
    for the finite-TLI re-solve; a window < 30 min away waits one parking revolution"; `j2_insertion.cpp`: comment the
    relative 1e-14 collinearity refusal and `allocate_fins`' zero deflection on refusal; `test_lunar_session.cpp`
    `upright_above_moon` uses `math::Mat3::from_columns`; format `++vacuum;    }` in `test_engine_placement.cpp`;
    `lunar_site.cpp`: comment that a site moved by < 1 px still matches its tile.
  - Leg friction creep (l4 Task 10): add one sentence to the HLS legs paragraph of `docs/subsystems.md`: "the
    regularised friction (0.1 m/s) lets the footprint creep only while the hull settles (~15 s; 0.17 m of the 1.18 m
    walk on 5.2 deg); a resting body is held fixed to the Moon, so a stay adds none".

- [ ] **Step 8: Tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[lunar]~[slow]"` then
`powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast`
Expected: all pass.

- [ ] **Step 9: Commit and push**

```bash
git add core docs/subsystems.md
git commit -m "refactor(sim): post-l4 cleanup - split the lunar coast out, deferred l4/l3 minors"
git push origin main
```

---

### Task 2: Radiative heating from the NASA equilibrium tables

The spec names Tauber-Sutton; its fit is valid for nose radii 0.3-3 m only (our ship: 6 m). Its source, Sutton &
Hartung, "Equilibrium Radiative Heating Tables for Earth Entry", NASA TM-102652, May 1990 [S], tabulates the
stagnation-point radiative rate for 0.05-10 m nose radii, 30-84 km and 8-18 km/s - used directly (public domain,
NTRS 19900014359). Tauber-Sutton stays as a cross-check in the test.

**Files:**
- Create: `config/aero/radiative_heating_earth.csv`, `core/include/starsim/aero/radiative_heating.hpp`,
  `core/src/aero/radiative_heating.cpp`, `core/tests/test_radiative_heating.cpp`
- Modify: `core/include/starsim/aero/entry_heating.hpp`, `core/src/aero/entry_heating.cpp`,
  `core/include/starsim/vehicle/flight_body.hpp` (`FlightEnvironment`), `core/src/vehicle/flight_body.cpp:357-365`,
  `core/src/sim/flight_session.cpp` (session loads the table into `env_`), `core/CMakeLists.txt`,
  `core/tests/CMakeLists.txt`, `tools/data_pipeline/sources.toml` (licence record), `CREDITS.md` (regenerated)

**Interfaces:**
- Produces:
  - `class aero::RadiativeHeatingTable { static RadiativeHeatingTable load(const std::filesystem::path& config_dir);
    double heat_rate_w_m2(double density_kg_m3, double speed_mps, double nose_radius_m) const; }`
  - `double aero::entry_heat_rate_w_m2(double density_kg_m3, double speed_mps, double nose_radius_m,
    const RadiativeHeatingTable* radiative)` - convective (Sutton-Graves) + radiative (when the table is given).
  - `FlightEnvironment::radiative_heating` (`const aero::RadiativeHeatingTable*`, nullptr: convective only).
  - `FlightBody::radiative_heat_rate_w_m2()` (last tick's radiative share, telemetry/tests).

- [ ] **Step 1: Transcribe the tables.** Download NTRS 19900014359 into `$CLAUDE_JOB_DIR/tmp/pdfs/` (it is a scan:
  extract the page images with pypdf + Pillow and read them). Write `config/aero/radiative_heating_earth.csv`:

```
# Equilibrium stagnation-point radiative heating for Earth entry [S: K. Sutton, L. C. Hartung, "Equilibrium Radiative
# Heating Tables for Earth Entry", NASA TM-102652, May 1990 (NTRS 19900014359), Tables 1-10]. Public domain (US Gov).
# Interpolate by the free-stream density (the source's advice), not the altitude. Columns: altitude_km (label only),
# density_kg_m3, velocity_kmps, nose_radius_m, qr_mw_m2.
altitude_km,density_kg_m3,velocity_kmps,nose_radius_m,qr_mw_m2
30,1.8410e-2,8.0,0.05,1.332
30,1.8410e-2,8.0,0.10,2.410
...
```

  Every row of tables 1-10 (all velocities and all eight radii 0.05/0.10/0.23/0.50/1.00/2.30/5.00/10.00 m). Spot values
  read in planning (use them as the transcription check, Step 2): table 1 (30 km, 1.8410e-2): 8.0 km/s 5.00 m 51.55
  (5.155E+01), 11.0 km/s 10.00 m 770.9 (7.709E+02); table 6 (60 km, 3.0592e-4): 9.0 km/s 5.00 m 0.1822, 9.5 km/s 5.00 m
  0.6324, 10.0 km/s 5.00 m 1.940, 11.0 km/s 5.00 m 6.784, 11.0 km/s 10.00 m 8.543, 12.0 km/s 1.00 m 8.765. Record the
  table count and row count in the ledger. Add the source to `tools/data_pipeline/sources.toml` (licence "public domain
  (US Government work)") and regenerate `CREDITS.md` with the tool the file names.

- [ ] **Step 2: Write the failing tests** (`core/tests/test_radiative_heating.cpp`, tag `[aero][heating]`):

```cpp
// Radiative heating (lunar l5): NASA TM-102652 equilibrium tables, interpolated by density, velocity and nose radius.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/aero/entry_heating.hpp"
#include "starsim/aero/radiative_heating.hpp"
#include "starsim/environment/atmosphere.hpp"

using namespace starsim;

namespace {
const aero::RadiativeHeatingTable& table() {
    static const auto t = aero::RadiativeHeatingTable::load(STARSIM_CONFIG_DIR);
    return t;
}
}  // namespace

TEST_CASE("Radiative table reproduces the tabulated values at its nodes", "[aero][heating]") {
    // transcription check: values read from the scan of NASA TM-102652 (MW/m^2)
    CHECK(table().heat_rate_w_m2(1.8410e-2, 8000.0, 5.0) == Catch::Approx(51.55e6).epsilon(1e-6));
    CHECK(table().heat_rate_w_m2(1.8410e-2, 11000.0, 10.0) == Catch::Approx(770.9e6).epsilon(1e-6));
    CHECK(table().heat_rate_w_m2(3.0592e-4, 9000.0, 5.0) == Catch::Approx(0.1822e6).epsilon(1e-6));
    CHECK(table().heat_rate_w_m2(3.0592e-4, 9500.0, 5.0) == Catch::Approx(0.6324e6).epsilon(1e-6));
    CHECK(table().heat_rate_w_m2(3.0592e-4, 10000.0, 5.0) == Catch::Approx(1.940e6).epsilon(1e-6));
    CHECK(table().heat_rate_w_m2(3.0592e-4, 11000.0, 5.0) == Catch::Approx(6.784e6).epsilon(1e-6));
    CHECK(table().heat_rate_w_m2(3.0592e-4, 12000.0, 1.0) == Catch::Approx(8.765e6).epsilon(1e-6));
}

TEST_CASE("Radiative heating interpolates monotonically and vanishes below 8 km/s", "[aero][heating]") {
    const double rho = 3.0592e-4;
    double last = 0.0;
    for (double v = 8000.0; v <= 12000.0; v += 250.0) {  // grows with speed
        const double q = table().heat_rate_w_m2(rho, v, 6.0);
        CHECK(q >= last);
        last = q;
    }
    // between the 5 m and 10 m radii (log-log): 6 m lies between them
    const double q5 = table().heat_rate_w_m2(rho, 11000.0, 5.0), q10 = table().heat_rate_w_m2(rho, 11000.0, 10.0);
    const double q6 = table().heat_rate_w_m2(rho, 11000.0, 6.0);
    CHECK(q6 > q5);
    CHECK(q6 < q10);
    // LEO entries stay untouched (b2/b4/d3 bit-identical): zero below 8 km/s
    CHECK(table().heat_rate_w_m2(rho, 7900.0, 6.0) == 0.0);
    CHECK(table().heat_rate_w_m2(0.0, 11000.0, 6.0) == 0.0);
}

TEST_CASE("Radiative table agrees with Tauber-Sutton inside the correlation's range", "[aero][heating][validation]") {
    // Tauber & Sutton, J. Spacecraft Rockets 28(1) 1991: q = C Rn^a rho^1.22 f(V) W/cm^2, C = 4.736e4,
    // a = 1.072e6 V^-1.88 rho^-0.325 (V m/s, rho kg/m^3, Rn m); valid 0.3-3 m, 10-16 km/s, 54-72 km; their stated fit
    // error against these tables is 20-30 %. f(V) at 11 km/s: 151 (Tauber & Sutton Table 2; verify against the paper
    // when available and record it in the ledger).
    const double rho = 3.0592e-4, v = 11000.0, rn = 2.3;
    const double a = 1.072e6 * std::pow(v, -1.88) * std::pow(rho, -0.325);
    const double tauber_w_m2 = 4.736e4 * std::pow(rn, a) * std::pow(rho, 1.22) * 151.0 * 1e4;
    CHECK(std::abs(table().heat_rate_w_m2(rho, v, rn) / tauber_w_m2 - 1.0) < 0.3);
}

TEST_CASE("Lunar-return glide exceeds the ship's heat limit, a 9.5 km/s glide does not", "[aero][heating][validation]") {
    // The l5 finding (spec amendment 2026-10-16): equilibrium glide with lift down holds the altitude where
    // q (C_L A) = m (v^2/r - g); ship 290 t, C_L A ~354 m^2 (L/D 0.71, C_D A 498 m^2), nose radius 6 m.
    const auto glide_rate = [&](double v) {
        const double r = 6.378137e6 + 63e3;
        const double q = 290e3 * (v * v / r - 9.6) / 354.0;
        const double rho = 2.0 * q / (v * v);
        return aero::entry_heat_rate_w_m2(rho, v, 6.0, &table());
    };
    CHECK(glide_rate(10900.0) > 1.2e6);
    CHECK(glide_rate(9500.0) < 0.9 * 1.2e6);
}
```

  Add the file to `core/tests/CMakeLists.txt`.

- [ ] **Step 3: Run to see them fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[heating]"`
Expected: compile error (`radiative_heating.hpp` missing).

- [ ] **Step 4: Implement the table** (`radiative_heating.hpp/.cpp`):

```cpp
#pragma once

#include <filesystem>
#include <vector>

namespace starsim::aero {

// Equilibrium stagnation-point radiative heating of Earth entries [S: Sutton & Hartung, NASA TM-102652, 1990]: the
// tables of config/aero/radiative_heating_earth.csv, interpolated linearly in ln(q) over ln(density) (between the
// tabulated altitudes), velocity and ln(nose radius). Outside: zero below 8 km/s (radiation < 1 % of the convective
// rate there; keeps LEO entries untouched); below a density's lowest tabulated velocity ln(q) is extrapolated with the
// slope of its two lowest velocities; below the lowest density (84 km) ln(q) is extrapolated in ln(density) with the
// slope of the two lowest; nose radius and density clamped to the table's range above. Equilibrium only: the
// non-equilibrium radiation of the thin upper air is not modelled.
class RadiativeHeatingTable {
public:
    static RadiativeHeatingTable load(const std::filesystem::path& config_dir);
    double heat_rate_w_m2(double density_kg_m3, double speed_mps, double nose_radius_m) const;
    bool empty() const { return levels_.empty(); }

private:
    struct Level {  // one altitude table
        double ln_density = 0.0;
        std::vector<double> velocity_mps;              // ascending
        std::vector<double> ln_radius;                 // ascending, same for all velocities
        std::vector<std::vector<double>> ln_rate;      // [velocity][radius], ln(W/m^2)
    };
    double ln_rate_at(const Level& level, double speed_mps, double ln_radius) const;
    std::vector<Level> levels_;  // ascending density
};

}  // namespace starsim::aero
```

```cpp
#include "starsim/aero/radiative_heating.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace starsim::aero {

namespace {

constexpr double min_speed_mps = 8000.0;  // [D] below: < 1 % of the convective rate (table at 8-9 km/s)

// Linear interpolation weight of x in [x0, x1] (clamped to [0, 1] when `clamp`).
double weight(double x, double x0, double x1, bool clamp) {
    const double w = (x - x0) / (x1 - x0);
    return clamp ? std::clamp(w, 0.0, 1.0) : w;
}

// Index i with xs[i] <= x < xs[i + 1] (first / last interval outside).
size_t interval(const std::vector<double>& xs, double x) {
    const auto it = std::upper_bound(xs.begin(), xs.end(), x);
    const size_t i = it == xs.begin() ? 0 : static_cast<size_t>(it - xs.begin()) - 1;
    return std::min(i, xs.size() - 2);
}

}  // namespace

RadiativeHeatingTable RadiativeHeatingTable::load(const std::filesystem::path& config_dir) {
    const auto path = config_dir / "aero" / "radiative_heating_earth.csv";
    std::ifstream in(path);
    if (!in) throw std::runtime_error("radiative heating table missing: " + path.string());
    // density -> velocity -> radius -> rate
    std::map<double, std::map<double, std::map<double, double>>> rows;
    std::string line;
    bool header = false;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (!header) { header = true; continue; }
        std::stringstream s(line);
        std::string cell;
        double v[5];
        for (double& x : v) {
            std::getline(s, cell, ',');
            x = std::stod(cell);
        }
        rows[v[1]][v[2] * 1000.0][v[3]] = v[4] * 1e6;
    }
    RadiativeHeatingTable t;
    for (const auto& [density, by_velocity] : rows) {
        Level level;
        level.ln_density = std::log(density);
        for (const auto& [velocity, by_radius] : by_velocity) {
            level.velocity_mps.push_back(velocity);
            std::vector<double> rates;
            if (level.ln_radius.empty()) {
                for (const auto& [radius, rate] : by_radius) level.ln_radius.push_back(std::log(radius));
            }
            for (const auto& [radius, rate] : by_radius) rates.push_back(std::log(rate));
            if (rates.size() != level.ln_radius.size()) throw std::runtime_error("radiative heating table: ragged radii");
            level.ln_rate.push_back(std::move(rates));
        }
        if (level.velocity_mps.size() < 2 || level.ln_radius.size() < 2) {
            throw std::runtime_error("radiative heating table: a level needs two velocities and two radii");
        }
        t.levels_.push_back(std::move(level));
    }
    if (t.levels_.size() < 2) throw std::runtime_error("radiative heating table: needs two density levels");
    return t;
}

double RadiativeHeatingTable::ln_rate_at(const Level& level, double speed_mps, double ln_radius) const {
    const size_t j = interval(level.ln_radius, ln_radius);
    const double wr = weight(ln_radius, level.ln_radius[j], level.ln_radius[j + 1], true);
    const auto at_velocity = [&](size_t i) {
        return level.ln_rate[i][j] + wr * (level.ln_rate[i][j + 1] - level.ln_rate[i][j]);
    };
    const size_t i = interval(level.velocity_mps, speed_mps);
    // below the lowest velocity: extrapolated with the lowest pair's slope; above the highest: clamped
    const bool above = speed_mps > level.velocity_mps.back();
    const double wv = above ? 1.0 : weight(speed_mps, level.velocity_mps[i], level.velocity_mps[i + 1], false);
    return at_velocity(i) + wv * (at_velocity(i + 1) - at_velocity(i));
}

double RadiativeHeatingTable::heat_rate_w_m2(double density_kg_m3, double speed_mps, double nose_radius_m) const {
    if (levels_.empty() || density_kg_m3 <= 0.0 || speed_mps < min_speed_mps || nose_radius_m <= 0.0) return 0.0;
    const double ln_rho = std::log(density_kg_m3);
    const double ln_rn = std::log(nose_radius_m);
    std::vector<double> ln_densities;
    for (const auto& l : levels_) ln_densities.push_back(l.ln_density);
    const size_t k = interval(ln_densities, ln_rho);
    // above the densest level clamped; below the thinnest extrapolated (radiation falls fast with density)
    const bool denser = ln_rho > ln_densities.back();
    const double w = denser ? 1.0 : weight(ln_rho, ln_densities[k], ln_densities[k + 1], false);
    const double a = ln_rate_at(levels_[k], speed_mps, ln_rn), b = ln_rate_at(levels_[k + 1], speed_mps, ln_rn);
    return std::exp(a + w * (b - a));
}

}  // namespace starsim::aero
```

  `entry_heating.hpp` gains (keep `stagnation_heat_rate_w_m2` unchanged):

```cpp
class RadiativeHeatingTable;

// Stagnation heat rate of an entry: convective (Sutton-Graves) plus, with a table, equilibrium radiative heating
// (NASA TM-102652). The radiative part matters above ~9 km/s (lunar returns): at 11 km/s, 60 km and a 5 m radius it is
// 6.8 MW/m^2 against ~2 MW/m^2 convective.
double entry_heat_rate_w_m2(double density_kg_m3, double speed_mps, double nose_radius_m,
                            const RadiativeHeatingTable* radiative);
```

  with the obvious body in `entry_heating.cpp`
  (`stagnation_heat_rate_w_m2(...) + (radiative ? radiative->heat_rate_w_m2(...) : 0.0)`), and update the header
  comment of `stagnation_heat_rate_w_m2` ("Radiative heating: entry_heat_rate_w_m2").

- [ ] **Step 5: Truth heating in `FlightBody`.** `FlightEnvironment` gains
  `const aero::RadiativeHeatingTable* radiative_heating = nullptr;  // lunar l5: Earth air radiation (nullptr: none)`.
  In `FlightBody::step` (flight_body.cpp ~361) only on the Earth (radiative tables are Earth air):

```cpp
        const double speed = norm(air_data_.air_velocity_body_mps);
        const double density = air_data_.air.density_kg_m3;
        const bool earth_air = body.id() == ephemeris::Body::Earth;
        radiative_heat_rate_w_m2_ =
            earth_air && env.radiative_heating ? env.radiative_heating->heat_rate_w_m2(density, speed, shield.nose_radius_m)
                                               : 0.0;
        heat_rate_w_m2_ = aero::stagnation_heat_rate_w_m2(density, speed, shield.nose_radius_m) + radiative_heat_rate_w_m2_;
```

  (`radiative_heat_rate_w_m2_` member + accessor, documented next to `heat_rate_w_m2()`). In `FlightSession`'s
  constructor load the table once (`radiative_heating_ = aero::RadiativeHeatingTable::load(config_dir_)` as a member,
  `env_.radiative_heating = &radiative_heating_`). Check that every `FlightEnvironment` built in tests still compiles
  (aggregate with a default member: yes).

- [ ] **Step 6: Run the tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[heating],[ship]~[slow]"`
Expected: PASS; the ship entry tests unchanged (LEO speeds: radiative zero).

- [ ] **Step 7: Commit and push** (after `-Fast` passes)

```bash
git add config/aero/radiative_heating_earth.csv core tools/data_pipeline/sources.toml CREDITS.md
git commit -m "feat(aero): equilibrium radiative heating from the NASA TM-102652 tables"
git push origin main
```

---

### Task 3: Predictor - radiative heating, skip arcs, exit orbit; truth air-density factor

**Files:**
- Modify: `core/include/starsim/gnc/entry_predictor.hpp`, `core/src/gnc/entry_predictor.cpp`,
  `core/include/starsim/environment/reference_body.hpp` (+ `.cpp` if `air` is defined there),
  `core/include/starsim/vehicle/flight_body.hpp` (`FlightEnvironment::air_density_factor`),
  `core/src/vehicle/flight_body.cpp` (`reference_body_of`), `core/include/starsim/sim/flight_session.hpp`,
  `core/src/sim/flight_session.cpp`
- Test: `core/tests/test_entry_skip.cpp` (new), `core/tests/test_lunar_flight_body.cpp` (density factor)

**Interfaces:**
- Consumes: `aero::RadiativeHeatingTable` (Task 2).
- Produces:
  - `EntryPredictorModel::radiative` (`std::shared_ptr<const aero::RadiativeHeatingTable>`, null: convective only),
    `EntryPredictorModel::carry_skips` (bool, default false), `EntryPredictorModel::max_skip_apoapsis_height_m`
    (1000e3 [E]).
  - `EntryPrediction::exited` (bool), `exit_apoapsis_height_m`, `exit_periapsis_height_m`, `exit_inclination_rad`,
    `skips` (int: arcs carried above the atmosphere).
  - `environment::ReferenceBody::set_air_density_factor(double)`; `FlightEnvironment::air_density_factor` (1.0);
    `FlightSession::set_air_density_factor(double)` (truth dispersion of the campaigns; the onboard predictor keeps
    US76).

- [ ] **Step 1: Write the failing tests** (`core/tests/test_entry_skip.cpp`, tag `[ship][guidance][skip]`), using
  `ship_entry_scenario.hpp` like `test_entry_guidance.cpp` (copy its `predictor_model` / `predictor_start` helpers):

```cpp
TEST_CASE("Predictor reports the exit orbit of a skip-out", "[ship][guidance][skip]") {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::EntryStart start;
    start.speed_relative_mps = 10900.0;
    start.flight_path_angle_rad = -5.0 * deg;   // probe 2026-10-16: lift up skips out here
    test::ShipEntryScenario scenario(start);
    auto model = predictor_model(scenario, schedule);
    const auto p = gnc::predict_entry(model, predictor_start(scenario), 0.0, gnc::EntryTarget{}, 800.0);
    CHECK_FALSE(p.reached_gate);
    CHECK(p.exited);
    CHECK(p.exit_apoapsis_height_m > 1000e3);        // still a lunar-return energy minus one dip
    CHECK(p.exit_periapsis_height_m < 120e3);
}

TEST_CASE("Predictor carries a low skip arc to the second entry", "[ship][guidance][skip]") {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::EntryStart start;
    start.speed_relative_mps = 9600.0;
    start.flight_path_angle_rad = -4.0 * deg;
    test::ShipEntryScenario scenario(start);
    auto model = predictor_model(scenario, schedule);
    // find a lift-up bank that exits low (apoapsis below the carry limit) by scanning; at least one must exist
    model.carry_skips = true;
    bool carried = false;
    for (double bank_deg = 0.0; bank_deg <= 90.0 && !carried; bank_deg += 5.0) {
        const auto p = gnc::predict_entry(model, predictor_start(scenario), bank_deg * deg, gnc::EntryTarget{}, 800.0,
                                          0.0, 1.0, 6000.0);
        if (p.skips > 0) {
            carried = true;
            CHECK(p.reached_gate);  // the carried arc came back down to the gate
        }
    }
    CHECK(carried);
}

TEST_CASE("Predictor heat rate includes the radiative part at lunar speeds", "[ship][guidance][skip]") {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::EntryStart start;
    start.speed_relative_mps = 10900.0;
    start.flight_path_angle_rad = -6.0 * deg;
    test::ShipEntryScenario scenario(start);
    auto model = predictor_model(scenario, schedule);
    const auto convective = gnc::predict_entry(model, predictor_start(scenario), 0.0, gnc::EntryTarget{}, 800.0);
    model.radiative = std::make_shared<const aero::RadiativeHeatingTable>(
        aero::RadiativeHeatingTable::load(STARSIM_CONFIG_DIR));
    const auto total = gnc::predict_entry(model, predictor_start(scenario), 0.0, gnc::EntryTarget{}, 800.0);
    CHECK(total.peak_heat_rate_w_m2 > 1.3 * convective.peak_heat_rate_w_m2);
}
```

  and in `test_lunar_flight_body.cpp` (`[lunar]`): `ReferenceBody::earth(...)` with `set_air_density_factor(1.15)` gives
  `air(60e3).density_kg_m3 == Approx(1.15 * unscaled)` and the mean free path divided by 1.15.

- [ ] **Step 2: Run to see them fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[skip]"`
Expected: compile errors (`exited`, `carry_skips`, `radiative` unknown).

- [ ] **Step 3: Implement.** In `entry_predictor.hpp` add to `EntryPredictorModel`:

```cpp
    // Lunar l5: equilibrium radiative heating (Earth air) on top of Sutton-Graves; null: convective only (LEO entries).
    std::shared_ptr<const aero::RadiativeHeatingTable> radiative;
    // Lunar l5 skip entry: an exit above the atmosphere whose osculating apoapsis stays below this is carried (Kepler +
    // J2 in the Earth-fixed frame, no air) to the next entry; false or a higher apoapsis: the prediction ends at the exit.
    bool carry_skips = false;
    double max_skip_apoapsis_height_m = 1000e3;  // [E] an arc to 1000 km takes ~20 min: still a guided skip
```

  and to `EntryPrediction`:

```cpp
    // Exit above the atmosphere (above 130 km after dipping below 80 km) that was not carried: the osculating orbit
    // there (inertial velocity = Earth-fixed + Earth rotation; central gravity, heights above the equatorial radius).
    bool exited = false;
    double exit_apoapsis_height_m = 0.0;
    double exit_periapsis_height_m = 0.0;
    double exit_inclination_rad = 0.0;  // to the equator (ITRF z)
    int skips = 0;                      // arcs carried above the atmosphere
```

  In `entry_predictor.cpp`: the heat rate becomes
  `aero::entry_heat_rate_w_m2(air.density_kg_m3, speed, m_.nose_radius_m, m_.radiative.get())`. Replace the skip-out
  break:

```cpp
        if (now.height_m < entry_height_m) entered = true;
        if (entered && now.height_m > skip_out_height_m) {
            const Exit e = exit_orbit(m, r, v);
            if (m.carry_skips && e.apoapsis_height_m < m.max_skip_apoapsis_height_m && e.apoapsis_height_m > 0.0) {
                entered = false;  // carried: the arc comes back down
                ++p.skips;
                continue;
            }
            p.exited = true;
            p.exit_apoapsis_height_m = e.apoapsis_height_m;
            p.exit_periapsis_height_m = e.periapsis_height_m;
            p.exit_inclination_rad = e.inclination_rad;
            break;
        }
```

  with, in the anonymous namespace:

```cpp
struct Exit {
    double apoapsis_height_m = 0.0;   // < 0: hyperbolic (no apoapsis)
    double periapsis_height_m = 0.0;
    double inclination_rad = 0.0;
};

// Osculating two-body orbit of an Earth-fixed state (inertial velocity v + w x r, the axes aligned at this instant).
Exit exit_orbit(const EntryPredictorModel& m, const Vec3& r, const Vec3& v_fixed) {
    const Vec3 v = v_fixed + cross(Vec3{0.0, 0.0, m.omega_radps}, r);
    const double rn = norm(r), energy = 0.5 * dot(v, v) - m.gm_m3ps2 / rn;
    const Vec3 h = cross(r, v);
    Exit e;
    e.inclination_rad = std::acos(std::clamp(h.z / norm(h), -1.0, 1.0));
    if (energy >= 0.0) {
        e.apoapsis_height_m = -1.0;
        return e;
    }
    const double a = -m.gm_m3ps2 / (2.0 * energy);
    const double ecc = std::sqrt(std::max(0.0, 1.0 - dot(h, h) / (m.gm_m3ps2 * a)));
    e.apoapsis_height_m = a * (1.0 + ecc) - m.earth_radius_m;
    e.periapsis_height_m = a * (1.0 - ecc) - m.earth_radius_m;
    return e;
}
```

  Above the atmosphere the RK4 step stays 1 s (a 20 min arc is 1,200 steps: cheap; the air model returns zero density
  there). Callers that carry skips pass a larger `max_time_s` (6000 s).

  `ReferenceBody`: add `void set_air_density_factor(double f) { air_density_factor_ = f; }` and scale in `air()`:

```cpp
    AtmosphereState air(double height_m) const {
        if (!atmosphere_) return vacuum_state();
        auto s = atmosphere_->at(height_m);
        s.density_kg_m3 *= air_density_factor_;        // truth dispersion (campaigns); 1 otherwise
        s.mean_free_path_m /= air_density_factor_;
        return s;
    }
```

  (check the field names of `AtmosphereState`; keep pressure and temperature: the engines' back-pressure is not
  dispersed). `FlightEnvironment` gains `double air_density_factor = 1.0;  // lunar l5 campaigns: truth air density`,
  `reference_body_of(env, Earth)` sets it on the Earth body, and `FlightSession::set_air_density_factor(double f)` sets
  `env_.air_density_factor` (documented: "truth only; the onboard predictor keeps US76").

- [ ] **Step 4: Run the tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[skip],[ship][guidance]~[slow],[lunar]~[slow]"`
Expected: PASS.

- [ ] **Step 5: Commit and push** (after `-Fast`)

```bash
git add core
git commit -m "feat(gnc): entry predictor with radiative heating, carried skip arcs and the exit orbit"
git push origin main
```

---

### Task 4: Entry guidance - skip-carrying (lunar return) and aerocapture modes

**Files:**
- Modify: `core/include/starsim/gnc/entry_guidance.hpp`, `core/src/gnc/entry_guidance.cpp`
- Test: `core/tests/test_entry_skip.cpp`

**Interfaces:**
- Consumes: Task 3 predictor fields.
- Produces:
  - `EntryPhase::Exit` ("Austritt": aerocapture left the atmosphere; `to_string` German).
  - `EntryGuidanceParameters`: `bool carry_skips = false; double max_time_s = 3000.0; double aerocapture_apoapsis_m =
    0.0 /* > 0: aerocapture to this apoapsis height */; double target_inclination_rad = -1.0 /* < 0: none */;
    double inclination_deadband_rad = 0.2 * deg_to_rad /* [E] */; double exit_height_m = 130e3;`
  - `static EntryGuidanceParameters lunar_return()` (carry_skips, max_time 6000 s) and
    `static EntryGuidanceParameters aerocapture(double apoapsis_m, double inclination_rad)`.
  - `EntryCommand::exit_apoapsis_height_m` (latest prediction, aerocapture).
  - The model always carries the radiative table when the guidance is built with one:
    `EntryGuidance(const vehicle::FlightBody&, const AlphaSchedule&, EntryTarget, EntryGuidanceParameters = {},
    std::shared_ptr<const aero::RadiativeHeatingTable> radiative = nullptr)`.

- [ ] **Step 1: Write the failing tests** (append to `test_entry_skip.cpp`):

```cpp
TEST_CASE("Guided lunar-return entry at 9.6 km/s reaches the gate within the heat limit", "[ship][guidance][skip]") {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::EntryStart start;
    start.speed_relative_mps = 9600.0;
    start.flight_path_angle_rad = -5.0 * deg;  // inside the corridor (Task 5 measures it; adjust to its middle then)
    test::ShipEntryScenario scenario(start);
    const auto radiative = std::make_shared<const aero::RadiativeHeatingTable>(
        aero::RadiativeHeatingTable::load(STARSIM_CONFIG_DIR));
    // target: the end point of a 60 deg bank prediction (reachable by construction)
    auto model = predictor_model(scenario, schedule);
    model.radiative = radiative;
    model.carry_skips = true;
    const auto nominal = gnc::predict_entry(model, predictor_start(scenario), 60.0 * deg, gnc::EntryTarget{}, 800.0,
                                            0.0, 1.0, 6000.0);
    REQUIRE(nominal.reached_gate);
    const math::Vec3 end = nominal.end_position_itrf_m;
    const gnc::EntryTarget target{end * ((norm(end) - 800.0) / norm(end)), 0.0};
    auto p = gnc::EntryGuidanceParameters::lunar_return();
    p.predictor_j2 = false;
    scenario.fly_guided(schedule, target, p, radiative);   // helper added to ship_entry_scenario.hpp (below)
    CHECK(scenario.reached_gate());
    CHECK(scenario.gate_miss_m(target) < 2000.0);
    CHECK(scenario.body().peak_heat_rate_w_m2() < 1.2e6);
    CHECK_FALSE(scenario.body().impacted());
}

TEST_CASE("Aerocapture guidance exits near the target apoapsis", "[ship][guidance][skip]") {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::EntryStart start;
    start.speed_relative_mps = 9600.0;
    start.flight_path_angle_rad = -4.5 * deg;  // Task 5's aerocapture corridor middle replaces this
    test::ShipEntryScenario scenario(start);
    const auto radiative = std::make_shared<const aero::RadiativeHeatingTable>(
        aero::RadiativeHeatingTable::load(STARSIM_CONFIG_DIR));
    auto p = gnc::EntryGuidanceParameters::aerocapture(400e3, -1.0);
    p.predictor_j2 = false;
    const auto exit = scenario.fly_aerocapture(schedule, p, radiative);  // flies until phase Exit, returns the orbit
    REQUIRE(exit.exited);
    CHECK(std::abs(exit.apoapsis_height_m - 400e3) < 50e3);  // [E] band; the campaign (Task 10) widens it
    CHECK(exit.periapsis_height_m < 120e3);                  // still dips: the perigee raise follows
    CHECK(scenario.body().peak_heat_rate_w_m2() < 1.2e6);
}
```

  Extend `ship_entry_scenario.hpp` with `fly_guided(schedule, target, parameters, radiative)` (an `EntryGuidance` in the
  loop as "Guided 6-DOF entry flies to the gate" does it in `test_entry_guidance.cpp` - move that loop into the scenario
  so both tests share it), `reached_gate()`, `gate_miss_m(target)` and `fly_aerocapture(...)` returning
  `struct AerocaptureExit { bool exited; double apoapsis_height_m, periapsis_height_m; }` (two-body from the 6-DOF
  state at the first tick in phase `Exit`; the scenario Earth's GM). The scenario's `FlightEnvironment` gets the
  radiative table so the truth heats as the predictor does.

- [ ] **Step 2: Run to see them fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[skip]"`
Expected: compile errors (`lunar_return`, `aerocapture`, `fly_guided`).

- [ ] **Step 3: Implement the modes** in `entry_guidance.cpp`:
  - `configure_model`: copy `carry_skips` into the model, set the radiative pointer passed to the constructor.
  - Every `predict_entry` call passes `parameters_.max_time_s`.
  - `solve()`: the miss function becomes a member `double miss(const EntryPrediction& p) const`:

```cpp
// Corrector miss (> 0: too much energy left - more bank).
// Entry: downrange error at the gate; a prediction that exits (not carried) overshoots by the whole Earth's range.
// Aerocapture: exit apoapsis minus the target; a prediction that stays in (reaches the gate or times out) undershoots.
double EntryGuidance::miss(const EntryPrediction& p) const {
    constexpr double overshoot_m = 2.0e7;  // [E] beyond any range: forces more bank
    if (parameters_.aerocapture_apoapsis_m > 0.0) {
        if (p.exited) return p.exit_apoapsis_height_m < 0.0 ? overshoot_m : p.exit_apoapsis_height_m - parameters_.aerocapture_apoapsis_m;
        return -overshoot_m;
    }
    if (p.exited) return overshoot_m;
    return p.downrange_error_m;
}
```

    and the bracket / Illinois search uses `miss(prediction)` where it used `prediction.downrange_error_m` (the existing
    search assumes the miss falls with the bank magnitude - true for both: more bank = less lift up = less range / less
    apoapsis).
  - `violates()` unchanged (heat rate now includes radiative through the model).
  - Aerocapture: no terminal phase (`terminal_phase` forced false by the factory), no gate: in `update`, once the body
    has been below 80 km and is above `exit_height_m`, `phase_ = EntryPhase::Exit` and the command holds lift up.
    Reversals: when `target_inclination_rad >= 0`, after each corrector cycle compare `prediction.exit_inclination_rad`
    for the current sign and the reversed sign (two predictions); reverse when the current error exceeds
    `inclination_deadband_rad` and the reversed one is smaller (respecting `min_reversal_interval_s`). The crossrange
    reversal logic is skipped in aerocapture.
  - Factories:

```cpp
    // Lunar return (l5): the predictor carries skip arcs (Orion-like skip entry, Brunner & Lu 2008 principle) and
    // looks further ahead.
    static EntryGuidanceParameters lunar_return() {
        EntryGuidanceParameters p;
        p.carry_skips = true;
        p.max_time_s = 6000.0;
        return p;
    }
    // Aerocapture (l5, "Aircatch"): one pass, the bank magnitude by the predicted exit apoapsis (numerical
    // predictor-corrector, the principle of FNPAG [S: Lu, Cerimele, Tigges, Matz, AIAA 2015-1520]), constant bank per
    // cycle; reversals on the exit orbit's inclination; no terminal phase.
    static EntryGuidanceParameters aerocapture(double apoapsis_m, double inclination_rad) {
        EntryGuidanceParameters p;
        p.aerocapture_apoapsis_m = apoapsis_m;
        p.target_inclination_rad = inclination_rad;
        p.terminal_phase = false;
        p.max_time_s = 3000.0;
        return p;
    }
```

  - `EntryCommand::exit_apoapsis_height_m` set from the latest prediction in aerocapture mode.

- [ ] **Step 4: Run the tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[skip],[ship][guidance]~[slow]"`
Expected: PASS; the existing b2 tests unchanged (default parameters: miss = downrange, an exit was "not reached" before
and is now an overshoot - check the b2 campaign stays 11/12 within 0.11 m in Task 10's full suite; if a LEO case now
changes, record it).

- [ ] **Step 5: Commit and push** (after `-Fast`)

```bash
git add core
git commit -m "feat(gnc): lunar-return skip entry and aerocapture modes of the entry guidance"
git push origin main
```

---

### Task 5: Entry corridor and the highest admissible entry speed

**Files:**
- Create: `core/include/starsim/gnc/entry_corridor.hpp`, `core/src/gnc/entry_corridor.cpp`,
  `core/tests/test_entry_corridor.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `predict_entry` with the Task 3 fields; `config::HeatShieldConfig`.
- Produces:

```cpp
namespace starsim::gnc {

// Entry conditions at the interface for a corridor query (relative to the rotating Earth).
struct CorridorQuery {
    double speed_mps = 0.0;
    double interface_height_m = 120e3;
    double latitude_rad = 0.0;
    double heading_rad = 0.5 * constants::pi;  // clockwise from north
    double gate_height_m = 800.0;
    config::HeatShieldConfig limits;
    double constraint_fraction = 0.9;          // of the heat-rate and load limits (as the guidance)
    double aerocapture_apoapsis_m = 0.0;       // > 0: aerocapture corridor to this apoapsis height
};

struct EntryCorridor {
    bool exists = false;
    double shallow_rad = 0.0;  // overshoot boundary: full lift down still captures (direct) / reaches the apoapsis
    double steep_rad = 0.0;    // undershoot boundary: full lift up stays within the limits (and, aerocapture, exits high enough)
    double middle_rad = 0.0;
    double width_rad = 0.0;
    std::string limiting;      // German: what bounds the steep side ("Hitzestrom", "Last", "Austritt")
};

// Corridor in flight-path angle at the interface by bisection on the predictor (constant full lift down / up), in
// [-15, -0.5] deg. Direct: shallow = the shallowest angle at which lift down does not exit; steep = the steepest at
// which lift up keeps the peaks within fraction x limits. Aerocapture: shallow = the shallowest at which lift down
// exits at or below the apoapsis; steep = the steepest at which lift up exits at or above it within the limits.
EntryCorridor entry_corridor(EntryPredictorModel model, const CorridorQuery& query);

// The highest interface speed in [low, high] whose corridor is at least min_width wide (bisection, 25 m/s).
// Returns low - 1 when even `low` has none.
double max_entry_speed(const EntryPredictorModel& model, CorridorQuery query, double min_width_rad, double low_mps,
                       double high_mps);

}  // namespace starsim::gnc
```

- [ ] **Step 1: Write the failing tests** (`core/tests/test_entry_corridor.cpp`, `[ship][guidance][corridor]`):

```cpp
TEST_CASE("The ship has no lunar-return corridor at 10.9 km/s within its heat limit", "[ship][guidance][corridor]") {
    auto [model, limits] = ship_model();  // helper: ship scenario's predictor model + radiative + heat_shield config
    gnc::CorridorQuery q;
    q.speed_mps = 10900.0;
    q.limits = limits;
    const auto c = gnc::entry_corridor(model, q);
    CHECK_FALSE(c.exists);
    CHECK(c.limiting == "Hitzestrom");
}

TEST_CASE("The ship's corridor at the highest admissible speed is at least 1 deg wide", "[ship][guidance][corridor]") {
    auto [model, limits] = ship_model();
    gnc::CorridorQuery q;
    q.limits = limits;
    const double v = gnc::max_entry_speed(model, q, 1.0 * deg, 8000.0, 11000.0);
    if (std::getenv("STARSIM_VERBOSE")) std::printf("max entry speed %.0f m/s\n", v);
    CHECK(v > 9000.0);   // the planning estimate was ~9.6-9.7 km/s
    CHECK(v < 10500.0);
    q.speed_mps = v;
    const auto c = gnc::entry_corridor(model, q);
    REQUIRE(c.exists);
    CHECK(c.width_rad >= 1.0 * deg - 1e-6);
    CHECK(c.middle_rad < -2.0 * deg);
}

TEST_CASE("A capsule's 11 km/s corridor is Apollo-like", "[capsule][guidance][corridor][validation]") {
    // Plausibility (spec): Apollo (L/D ~0.3) entered at -6.5 deg at 400,000 ft within a corridor ~2 deg wide; the
    // Crew Dragon model (L/D 0.2) with a 10 g load limit and no heat limit should give a corridor 1-2.5 deg wide with
    // its middle between -7.5 and -5.5 deg.
    auto [model, limits] = capsule_model();  // Crew Dragon EntryAeroTable::capsule, no radiative, nose radius 0
    limits.max_heat_rate_w_m2 = 0.0;          // 0: not judged (check entry_corridor treats 0 as "no limit")
    limits.max_load_g = 10.0;
    gnc::CorridorQuery q;
    q.speed_mps = 11000.0;
    q.interface_height_m = 121.9e3;  // 400,000 ft
    q.gate_height_m = 5500.0;
    q.limits = limits;
    q.constraint_fraction = 1.0;
    const auto c = gnc::entry_corridor(model, q);
    REQUIRE(c.exists);
    CHECK(c.width_rad > 1.0 * deg);
    CHECK(c.width_rad < 2.5 * deg);
    CHECK(c.middle_rad > -7.5 * deg);
    CHECK(c.middle_rad < -5.5 * deg);
}

TEST_CASE("The aerocapture corridor to 400 km exists at the admissible speed", "[ship][guidance][corridor]") {
    auto [model, limits] = ship_model();
    gnc::CorridorQuery q;
    q.limits = limits;
    q.aerocapture_apoapsis_m = 400e3;
    q.speed_mps = gnc::max_entry_speed(model, q, 0.5 * deg, 8000.0, 11000.0);
    const auto c = gnc::entry_corridor(model, q);
    REQUIRE(c.exists);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("aerocapture %.0f m/s: %.2f .. %.2f deg\n", q.speed_mps, c.steep_rad / deg, c.shallow_rad / deg);
    }
}
```

  `ship_model()` builds the ship scenario's predictor model (as `test_entry_guidance.cpp`), loads the radiative table,
  `j2 = 0`, and returns `config.heat_shield` of the ship stage; `capsule_model()` the same for `crew_dragon` with
  `EntryAeroTable::capsule` (see `test_capsule_entry.cpp`).

- [ ] **Step 2: Run to see them fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[corridor]"`
Expected: compile error.

- [ ] **Step 3: Implement** (`entry_corridor.cpp`):

```cpp
namespace {

constexpr double steepest_rad = -15.0 * constants::deg_to_rad;
constexpr double shallowest_rad = -0.5 * constants::deg_to_rad;
constexpr double tolerance_rad = 0.01 * constants::deg_to_rad;

// Earth-fixed interface state on the scenario meridian (longitude 0) from speed, angle, heading.
EntryPredictorState interface_state(const EntryPredictorModel& m, const CorridorQuery& q, double gamma_rad, double bank_rad) {
    const double r = m.earth_radius_m + q.interface_height_m;  // sphere: the corridor is a 2-D question
    const math::Vec3 up{std::cos(q.latitude_rad), 0.0, std::sin(q.latitude_rad)};
    const math::Vec3 east{0.0, 1.0, 0.0};
    const math::Vec3 north = cross(up, east);
    const math::Vec3 horizontal = north * std::cos(q.heading_rad) + east * std::sin(q.heading_rad);
    const math::Vec3 v = (horizontal * std::cos(gamma_rad) + up * std::sin(gamma_rad)) * q.speed_mps;
    return {up * r, v, bank_rad};
}

EntryPrediction fly(const EntryPredictorModel& m, const CorridorQuery& q, double gamma_rad, double bank_rad) {
    return predict_entry(m, interface_state(m, q, gamma_rad, bank_rad), bank_rad, EntryTarget{}, q.gate_height_m, 0.0,
                         1.0, 6000.0);
}

bool within_limits(const EntryPrediction& p, const CorridorQuery& q, std::string* limiting) {
    const auto& l = q.limits;
    if (l.max_heat_rate_w_m2 > 0.0 && p.peak_heat_rate_w_m2 > q.constraint_fraction * l.max_heat_rate_w_m2) {
        if (limiting) *limiting = "Hitzestrom";
        return false;
    }
    if (l.max_heat_load_j_m2 > 0.0 && p.heat_load_j_m2 > q.constraint_fraction * l.max_heat_load_j_m2) {
        if (limiting) *limiting = "Wärmelast";
        return false;
    }
    if (l.max_load_g > 0.0 && p.peak_load_g > q.constraint_fraction * l.max_load_g) {
        if (limiting) *limiting = "Last";
        return false;
    }
    return true;
}

// Bisection for the boundary of a predicate that holds at `good` and fails at `bad`.
template <class F>
double boundary(double good, double bad, F holds) {
    while (std::abs(good - bad) > tolerance_rad) {
        const double mid = 0.5 * (good + bad);
        (holds(mid) ? good : bad) = mid;
    }
    return good;
}

}  // namespace

EntryCorridor entry_corridor(EntryPredictorModel model, const CorridorQuery& query) {
    model.carry_skips = false;  // a corridor boundary is about the first pass
    const bool capture = query.aerocapture_apoapsis_m > 0.0;
    const double lift_up = 0.0, lift_down = constants::pi;
    // Shallow side: lift down still captures (direct: no exit; aerocapture: exit at or below the apoapsis).
    const auto captures = [&](double g) {
        const auto p = fly(model, query, g, lift_down);
        if (!capture) return !p.exited;
        return !p.exited || (p.exit_apoapsis_height_m >= 0.0 && p.exit_apoapsis_height_m <= query.aerocapture_apoapsis_m);
    };
    // Steep side: lift up stays within the limits (aerocapture: and still exits at or above the apoapsis).
    std::string limiting;
    const auto admissible = [&](double g) {
        const auto p = fly(model, query, g, lift_up);
        if (!within_limits(p, query, &limiting)) return false;
        if (capture && !(p.exited && (p.exit_apoapsis_height_m < 0.0 || p.exit_apoapsis_height_m >= query.aerocapture_apoapsis_m))) {
            limiting = "Austritt";
            return false;
        }
        return true;
    };
    EntryCorridor c;
    if (!captures(steepest_rad) || !admissible(shallowest_rad)) {
        if (!admissible(shallowest_rad) && limiting.empty()) limiting = "Hitzestrom";
        c.limiting = limiting;
        return c;  // no capture even steeply, or not even the shallowest entry stays within the limits
    }
    c.shallow_rad = captures(shallowest_rad) ? shallowest_rad : boundary(steepest_rad, shallowest_rad, captures);
    c.steep_rad = admissible(steepest_rad) ? steepest_rad : boundary(shallowest_rad, steepest_rad, admissible);
    // re-evaluate the limiting cause just beyond the steep boundary
    admissible(c.steep_rad - 2.0 * tolerance_rad);
    c.limiting = limiting;
    c.exists = c.steep_rad < c.shallow_rad;
    c.width_rad = std::max(0.0, c.shallow_rad - c.steep_rad);
    c.middle_rad = 0.5 * (c.shallow_rad + c.steep_rad);
    return c;
}

double max_entry_speed(const EntryPredictorModel& model, CorridorQuery query, double min_width_rad, double low_mps,
                       double high_mps) {
    const auto wide_enough = [&](double v) {
        query.speed_mps = v;
        const auto c = entry_corridor(model, query);
        return c.exists && c.width_rad >= min_width_rad;
    };
    if (!wide_enough(low_mps)) return low_mps - 1.0;
    if (wide_enough(high_mps)) return high_mps;
    double good = low_mps, bad = high_mps;
    while (bad - good > 25.0) {
        const double mid = 0.5 * (good + bad);
        (wide_enough(mid) ? good : bad) = mid;
    }
    return good;
}
```

  Note on the probe (2026-10-16): with constant full lift down to the ground the load reaches 50-100 g - irrelevant for
  the shallow boundary (only "does it exit?" is asked there). If the "10.9 km/s has no corridor" test finds the steep
  side limited by "Last" instead of "Hitzestrom", report it (the heat-rate finding of the spec amendment would be
  wrong) - do not change the test to pass.

- [ ] **Step 4: Run the tests; record** the max entry speed (direct, 1 deg; aerocapture, 0.5 deg), the corridors at
  those speeds and the capsule corridor in the ledger.

Run: `STARSIM_VERBOSE=1 powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[corridor]"`
Expected: PASS.

- [ ] **Step 5: Commit and push** (after `-Fast`)

```bash
git add core
git commit -m "feat(gnc): entry corridor and the highest admissible entry speed"
git push origin main
```

---

### Task 6: Earth-return targeter and planner (TEI, MCC, braking burn)

**Files:**
- Create: `core/include/starsim/mission/earth_return.hpp`, `core/src/mission/earth_return.cpp`,
  `core/tests/test_earth_return.cpp`
- Modify: `core/include/starsim/gnc/burn_node.hpp`, `core/src/gnc/burn_node.cpp` (`raise_periapsis_at_apoapsis`),
  `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `mission::LunarOrbitKind`, `NrhoReference::perilune_epochs()`, `FiniteBurnModel` / `apply_finite_burn`,
  `orbit::propagate_coast` with events, `gnc::predict_entry` + `EntryPredictorModel` + `EntryTarget`,
  `frames::fixed_to_geodetic`.
- Produces:

```cpp
namespace starsim::mission {

// Where and how the return should meet the Earth.
struct EarthReturnTarget {
    math::Vec3 site_itrf_m;                  // the landing site (Starbase tower) on the ground
    double interface_height_m = 120e3;       // b2's entry interface
    double flight_path_rad = -5.0 * constants::deg_to_rad;  // relative to the rotating Earth (corridor middle)
    double speed_mps = 0.0;                  // relative, after the braking burn; 0: no braking
    double braking_height_m = 2000e3;        // [E] the braking burn's centre on the inbound leg (spec amendment)
    // Direct entry: the onboard predictor flying `nominal_bank_rad` (both signs averaged) must end at `gate` (b4's
    // deorbit rule); empty: aerocapture - only the plane is aimed (the site in the arrival plane).
    std::optional<gnc::EntryPredictorModel> entry_model;
    gnc::EntryTarget gate;
    double gate_height_m = 800.0;
    double nominal_bank_rad = 50.0 * constants::deg_to_rad;  // [E] as b4
};

// The arrival of a return trajectory (after the modelled braking burn).
struct EarthArrival {
    bool reached = false;                 // crossed the interface height inbound
    bool braked = false;                  // the braking burn was applied (speed above the target at its height)
    time::Epoch braking_epoch;
    double braking_delta_v_mps = 0.0;
    time::Epoch interface_epoch;
    math::StateVector interface_gcrf;
    double flight_path_rad = 0.0;         // relative
    double speed_mps = 0.0;               // relative
    double perigee_height_m = 0.0;        // osculating, after the braking (two-body)
    double crossrange_m = 0.0;            // the site's distance from the arrival plane (> 0: right)
    double downrange_error_m = 0.0;       // direct: predictor at the nominal bank (> 0: overshoot); else 0
};

struct ReturnBurn {
    bool converged = false;
    int iterations = 0;
    gnc::BurnNode burn;
    EarthArrival arrival;
};

struct EarthReturnParameters {
    LunarOrbitKind orbit = LunarOrbitKind::Llo;
    std::shared_ptr<const NrhoReference> nrho;   // Nrho
    EarthReturnTarget target;
    double flight_time_guess_s = 3.5 * 86400.0;  // [E] spec: ~3-4 days (first guess only)
    double search_s = 86400.0;                   // [E] departure revolutions (LLO) / perilunes (NRHO) scanned
    double earliest_departure_s = 1800.0;        // [E] after the plan's epoch
    double max_tei_mps = 1500.0;                 // [E] a TEI above this is refused
    std::vector<double> mcc_after_tei_s{86400.0, 2.5 * 86400.0};  // [E] spec: two corrections
    double mcc_threshold_mps = 0.05;             // [E] as the outbound flight
    double mcc_min_before_braking_s = 6.0 * 3600.0;  // [E] no MCC closer to the braking burn
};

struct ReturnPlan {
    bool feasible = false;
    std::string reason;  // German when refused
    gnc::BurnNode tei;
    EarthArrival arrival;
    double tei_delta_v_mps = 0.0;
    double braking_delta_v_mps = 0.0;
};

class EarthReturnPlanner {
public:
    EarthReturnPlanner(const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                       const frames::BodyOrientation& earth_orientation);

    // Coasts `state` to `burn_epoch`, applies `delta_v` (finite with the model set), coasts inbound, applies the
    // braking burn as an impulse at the braking height (retrograde, sized by vis-viva for the target speed at the
    // interface), coasts to the interface and evaluates it.
    EarthArrival evaluate(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                          const math::Vec3& delta_v_mps, const EarthReturnTarget& target) const;

    // Newton (minimum-norm, finite differences, backtracking) on the 3 delta-v components: perigee height of the
    // braked conic (the target angle's), crossrange zero, and (direct) the predictor's downrange error zero.
    ReturnBurn solve(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                     const EarthReturnTarget& target, const math::Vec3& initial_delta_v_mps, int max_iterations = 30) const;

    // TEI from the LLO (departure revolutions within search_s, burn at the patched-conic periapsis of each) or from the
    // NRHO (its perilunes within search_s); the cheapest converged candidate within max_tei_mps.
    ReturnPlan plan(const math::StateVector& state, time::Epoch epoch, const EarthReturnParameters& p,
                    double mass_kg, double isp_s, double propellant_kg) const;

    // Mid-course correction at `burn_epoch` (zero first guess); empty below the threshold or not converged.
    std::optional<gnc::BurnNode> mid_course(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                            const EarthReturnParameters& p) const;

    // The braking burn re-planned from the state now as the last targeting burn: burn epoch at the predicted braking
    // height crossing; constraints: relative speed and flight-path angle at the interface, and (direct) the predictor's
    // downrange error. Empty if not converged.
    std::optional<gnc::BurnNode> braking(const math::StateVector& state, time::Epoch epoch,
                                         const EarthReturnTarget& target) const;

    // The epoch the inbound coast crosses the braking height (empty: none within 6 days).
    std::optional<time::Epoch> braking_epoch(const math::StateVector& state, time::Epoch epoch,
                                             const EarthReturnTarget& target) const;

    void set_finite_burn(std::optional<FiniteBurnModel> model) { finite_ = model; }

private:
    // patched-conic first guess at the departure: burn epoch and delta-v
    struct Departure { time::Epoch epoch; math::Vec3 delta_v_mps; };
    std::vector<Departure> departures(const math::StateVector& state, time::Epoch epoch,
                                      const EarthReturnParameters& p) const;
    const environment::GravityModel& gravity_;
    const ephemeris::Ephemeris& ephemeris_;
    const frames::BodyOrientation& earth_;
    std::optional<FiniteBurnModel> finite_;
};

}  // namespace starsim::mission
```

  and in `burn_node.hpp`:

```cpp
// Prograde burn at the next apoapsis raising the periapsis to `periapsis_radius_m` (two-body); empty if the orbit is
// not closed or the periapsis is already higher.
std::optional<BurnNode> raise_periapsis_at_apoapsis(const math::StateVector& state, time::Epoch now, double gm_m3ps2,
                                                    double periapsis_radius_m);
```

- [ ] **Step 1: Write the failing tests** (`core/tests/test_earth_return.cpp`, tags `[lunar][return]`; the planning
  cases take a few seconds each - tag `[slow]` any that exceeds ~40 s):

```cpp
// Earth return from lunar orbit (l5): TEI, MCC and braking targeting in the full force model.

namespace {
struct Fixture {
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    mission::EarthReturnPlanner planner{gravity, ephemeris, earth};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    Fixture() { test::spice_session(); }
};

// Starbase tower (config/launch_sites.toml starbase_olp1) on the ground, ITRF
math::Vec3 starbase_itrf();
// 100 km polar LLO at t whose plane contains the given Moon-relative direction (unit, GCRF)
math::StateVector llo_containing(const Fixture& f, time::Epoch t, const math::Vec3& direction);
// the return asymptote's direction at t (patched conic) - from lunar_return_test_support.hpp (Task 7 moves it to the
// production placement; the test uses the same function: sim::lunar_return_asymptote)
}  // namespace

TEST_CASE("TEI from an aligned LLO costs the literature's delta-v and arrives in the corridor", "[lunar][return]") {
    Fixture f;
    mission::EarthReturnParameters p;
    p.target.site_itrf_m = starbase_itrf();
    p.target.flight_path_rad = -5.5 * deg;
    p.target.speed_mps = 0.0;  // no braking: the geometry alone first
    const auto s = llo_containing(f, f.t0, sim::lunar_return_asymptote(f.ephemeris, f.t0 + 3600.0, p.flight_time_guess_s));
    const auto plan = f.planner.plan(s, f.t0, p, 573e3, 380.0, 318e3);
    REQUIRE(plan.feasible);
    CHECK(plan.tei_delta_v_mps > 800.0);   // [S] spec: ~0.85-0.9 km/s from the LLO
    CHECK(plan.tei_delta_v_mps < 1000.0);
    CHECK(plan.arrival.reached);
    CHECK(std::abs(plan.arrival.flight_path_rad - p.target.flight_path_rad) < 0.05 * deg);
    CHECK(std::abs(plan.arrival.crossrange_m) < 5e3);
    CHECK(plan.arrival.speed_mps > 10.6e3);  // natural lunar-return entry speed
    CHECK(plan.arrival.speed_mps < 11.2e3);
}

TEST_CASE("TEI with the braking burn arrives at the target speed", "[lunar][return]") {
    Fixture f;
    mission::EarthReturnParameters p;
    p.target.site_itrf_m = starbase_itrf();
    p.target.flight_path_rad = -5.0 * deg;
    p.target.speed_mps = 9600.0;
    const auto s = llo_containing(f, f.t0, sim::lunar_return_asymptote(f.ephemeris, f.t0 + 3600.0, p.flight_time_guess_s));
    const auto plan = f.planner.plan(s, f.t0, p, 573e3, 380.0, 318e3);
    REQUIRE(plan.feasible);
    CHECK(plan.arrival.braked);
    CHECK(std::abs(plan.arrival.speed_mps - 9600.0) < 50.0);  // vis-viva sizing; the in-flight braking re-plan is exact
    CHECK(plan.braking_delta_v_mps > 1300.0);  // [D] planning estimate ~1.5 km/s at 2,000 km
    CHECK(plan.braking_delta_v_mps < 1700.0);
}

TEST_CASE("TEI from the NRHO costs less than from the LLO", "[lunar][return][slow]") {
    // [S] spec: ~0.4-0.5 km/s from the NRHO. Reference built as lunar_outbound_parameters does (hls_lunar_nrho seed).
    ...build the NRHO reference with first perilune at t0 + 1 d, place the ship on it at t0...
    CHECK(plan.tei_delta_v_mps > 300.0);
    CHECK(plan.tei_delta_v_mps < 600.0);
}

TEST_CASE("Mid-course correction after a dispersed TEI is small", "[lunar][return]") {
    // TEI executed 1 % short and 0.5 deg off (as the outbound campaign): MCC 1 below 30 m/s, after it the arrival
    // within 0.05 deg / 5 km
}

TEST_CASE("The braking burn re-planned near the Earth hits speed, angle and the gate range", "[lunar][return]") {
    // From the TEI plan's coast to 1 h before the braking height: braking() converges; evaluate the result: speed
    // within 5 m/s, angle within 0.02 deg, predicted downrange error at the nominal bank within 5 km.
}

TEST_CASE("TEI from a misaligned LLO waits for the window or refuses", "[lunar][return]") {
    // Review Focus 1: LLO plane 13 deg off the asymptote (the mission-1 ascent after a 24 h stay). search_s 1 day:
    // either feasible with tei_delta_v_mps reported (> the aligned case) or refused with a German reason containing
    // "TEI"; never a converged burn above max_tei_mps.
}

TEST_CASE("Perigee raise at apoapsis lifts the perigee to the target", "[gnc][burn]") {
    // 120 x 400 km orbit: node at the apoapsis, prograde, ~50-60 m/s to 200 km (spec); applied impulsively the
    // two-body perigee is 200 km within 100 m.
}
```

  Write the bodies of the elided cases in the same style before running (they are fully specified by their comments).

- [ ] **Step 2: Run to see them fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[return]~[slow]"`
Expected: compile errors.

- [ ] **Step 3: Implement `evaluate`**:

```cpp
EarthArrival EarthReturnPlanner::evaluate(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                          const math::Vec3& delta_v_mps, const EarthReturnTarget& target) const {
    EarthArrival a;
    const double gm = ephemeris_.gm_m3ps2(ephemeris::Body::Earth);
    // helpers in the anonymous namespace: periapsis_radius(state, gm) - two-body p / (1 + e); to_itrf(earth, state,
    // epoch) - position R^T r and relative velocity R^T (v - dR R^T r) from fixed_to_gcrf(epoch) {rotation, rate}
    auto at_burn = orbit::propagate_coast(gravity_, state, epoch, burn_epoch).state;
    math::StateVector after = finite_ ? apply_finite_burn(gravity_, at_burn, burn_epoch, delta_v_mps, *finite_)
                                      : math::StateVector{at_burn.position_m, at_burn.velocity_mps + delta_v_mps};
    const double r_brake = constants::wgs84_semi_major_axis_m + target.braking_height_m;
    const double r_interface = constants::wgs84_semi_major_axis_m + target.interface_height_m;
    const integrators::Event<6> braking{"braking height",
        [&](double, const integrators::State<6>& y) { return std::hypot(y[0], y[1], y[2]) - r_brake; },
        integrators::Crossing::Falling};
    const time::Epoch horizon = burn_epoch + 6.0 * 86400.0;
    auto coast = orbit::propagate_coast(gravity_, after, burn_epoch, horizon, std::span(&braking, 1));
    if (coast.event_index != 0) return a;  // never came near the Earth
    math::StateVector s = coast.state;
    time::Epoch t = coast.epoch;
    if (target.speed_mps > 0.0) {
        // vis-viva: inertial speed at the interface for the target relative speed (Earth rotation along the track is
        // folded in by the in-flight re-plan; here the relative and inertial speeds differ by < 0.5 km/s and only the
        // energy matters)
        const double v_now = norm(s.velocity_mps);
        const double v_interface_now = std::sqrt(v_now * v_now + 2.0 * gm * (1.0 / r_interface - 1.0 / r_brake));
        if (v_interface_now > target.speed_mps) {
            const double v_new = std::sqrt(target.speed_mps * target.speed_mps - 2.0 * gm * (1.0 / r_interface - 1.0 / r_brake));
            a.braking_delta_v_mps = v_now - v_new;
            s.velocity_mps = s.velocity_mps * (v_new / v_now);
            a.braked = true;
            a.braking_epoch = t;
        }
    }
    // osculating perigee of the braked conic (the constraint: continuous even when the interface is missed)
    a.perigee_height_m = periapsis_radius(s, gm) - constants::wgs84_semi_major_axis_m;
    const integrators::Event<6> interface{"entry interface",
        [&](double t_s, const integrators::State<6>& y) {
            const math::Vec3 r = transpose(earth_.fixed_to_gcrf(t + t_s).rotation) * math::Vec3{y[0], y[1], y[2]};
            return frames::fixed_to_geodetic(r).height_m - target.interface_height_m;
        },
        integrators::Crossing::Falling};
    auto in = orbit::propagate_coast(gravity_, s, t, t + 7200.0, std::span(&interface, 1));
    const auto [r_itrf, v_itrf] = to_itrf(earth_, in.state, in.epoch);  // helper: rotation + relative velocity
    a.reached = in.event_index == 0;
    a.interface_epoch = in.epoch;
    a.interface_gcrf = in.state;
    a.speed_mps = norm(v_itrf);
    a.flight_path_rad = std::asin(std::clamp(dot(math::normalized(r_itrf), v_itrf) / a.speed_mps, -1.0, 1.0));
    // crossrange: the site's distance from the arrival plane (plane through the Earth's centre, the interface point and
    // its relative velocity) - Earth rotation during the entry is the predictor's business
    const math::Vec3 n = math::normalized(cross(r_itrf, v_itrf));
    a.crossrange_m = -std::asin(std::clamp(dot(math::normalized(target.site_itrf_m), n), -1.0, 1.0)) *
                     constants::wgs84_semi_major_axis_m;
    if (a.reached && target.entry_model) {
        const gnc::EntryPredictorState start{r_itrf, v_itrf, target.nominal_bank_rad};
        const auto plus = gnc::predict_entry(*target.entry_model, start, target.nominal_bank_rad, target.gate,
                                             target.gate_height_m, 0.0, 1.0, 6000.0);
        auto start_minus = start;
        start_minus.bank_rad = -target.nominal_bank_rad;
        const auto minus = gnc::predict_entry(*target.entry_model, start_minus, -target.nominal_bank_rad, target.gate,
                                              target.gate_height_m, 0.0, 1.0, 6000.0);
        a.downrange_error_m = 0.5 * (plus.downrange_error_m + minus.downrange_error_m);
        if (plus.exited || minus.exited) a.downrange_error_m = 2.0e7;  // skipped beyond the carry limit
    }
    return a;
}
```

  The target perigee for the constraint follows from the target interface conditions (two-body, relative ~ inertial):
  `h = r_i v_i cos(gamma)`, `energy = v_i^2/2 - gm/r_i`, `r_p = (-gm + sqrt(gm^2 + 2 energy h^2)) / (2 energy)` for an
  ellipse/hyperbola (use the general periapsis formula `p / (1 + e)`). Implement `perigee_for(target)` with the
  interface speed = target speed (braked) or the arrival's own speed (no braking) - the latter from the current
  iterate's arrival (fixed-point inside Newton; converges because the speed changes little).

- [ ] **Step 4: Implement `solve`** after `LunarTransferTargeter::solve` (read it first: same minimum-norm Newton with
  finite differences and backtracking): residual `c = {(perigee - perigee_for(target)) / 1e3, crossrange / 1e3,
  downrange / 1e4 (direct only)}` (scaled to comparable size), step `h = 1e-3 m/s` for the finite differences,
  minimum-norm step `dx = J^T (J J^T)^-1 (-c)` (2 x 3 for aerocapture, 3 x 3 square for direct: plain solve with
  `math::solve`), backtracking halves the step until `|c|` decreases (max 8 halvings); converged when perigee within
  200 m, crossrange within 500 m and downrange within 500 m. Two stages for the direct target: first the 2-constraint
  (perigee, crossrange) solve, then all three from its solution (the predictor's downrange is defined only once the
  interface is reached).

- [ ] **Step 5: Implement `departures` and `plan`.** LLO: per revolution `k` from `epoch + earliest_departure_s` within
  `search_s`, the patched-conic departure: Moon-relative excess velocity `v_inf` of an Earth two-body Lambert arc from
  the Moon's position at the departure to the vacuum perigee point at `departure + flight_time_guess_s` (the point:
  direction of the site at that time rotated 90 deg back along the Earth's rotation, radius `R + 50 km`; any point works
  as a guess), hyperbola about the Moon with periapsis at the LLO radius: `e = 1 + r v_inf^2 / gm_moon`, asymptote
  angle `nu = acos(-1/e)`; the departure point is the LLO point whose direction is the asymptote direction rotated by
  `-nu` within the LLO plane (projected into the plane); the epoch = the time the coasted LLO state passes that
  direction (scan the revolution in 60 s steps, refine by bisection on the in-plane angle); delta-v guess
  `(sqrt(v_inf^2 + 2 gm/r) - v_circ)` along the velocity there. NRHO: every perilune epoch in the window, the guess
  `(sqrt(v_inf^2 + 2 gm/r_p) - |v|)` along the velocity. `plan`: for each departure, `solve`; keep the converged one with
  the smallest `|delta_v| + braking` within `max_tei_mps`; check the propellant by the rocket equation (TEI + braking
  + an MCC allowance of 30 m/s [E]) against `propellant_kg` - refused "Kein Rückflug: Treibstoff reicht nicht (... t
  fehlen)"; none converged: "Kein Rückflug: TEI nicht planbar"; above max: "Kein Rückflug: TEI über ... m/s (Bahnebene
  passt nicht)".

  The `sim::lunar_return_asymptote(ephemeris, t, flight_time_s)` used by the test is the same patched-conic
  `v_inf` direction: implement it in `mission/earth_return.cpp` as
  `math::Vec3 return_asymptote(const ephemeris::Ephemeris&, time::Epoch departure, double flight_time_s, const
  math::Vec3& site_itrf_m, const frames::BodyOrientation& earth)` (unit, GCRF) - declared in `earth_return.hpp`; the test
  calls `mission::return_asymptote` (correct the test's name accordingly).

- [ ] **Step 6: Implement `mid_course`, `braking_epoch`, `braking`** - `mid_course`: `solve` from a zero guess at the
  burn epoch, returned only above `mcc_threshold_mps`. `braking_epoch`: the coast event of `evaluate`. `braking`: the
  burn epoch from `braking_epoch`; unknowns the 3 components; constraints relative speed - target (m/s / 10), angle -
  target (deg x 10), and (direct) predictor downrange / 1e4 - without the vis-viva model (`target.speed_mps` handled by
  the constraint: evaluate with `speed_mps = 0` for the coast after the burn). Initial guess: the vis-viva retrograde
  delta-v. Finite burn model when set (the session sets it: ~70 s on the vacuum Raptors).

- [ ] **Step 7: Implement `raise_periapsis_at_apoapsis`** (`burn_node.cpp`): two-body elements of the state; time to
  the next apoapsis by Kepler's equation (use the existing helpers that `circularize_at_apoapsis` uses); at the apoapsis
  `v_a = sqrt(gm (2/r_a - 1/a))`, target `a' = (r_a + r_p')/2`, `v_a' = sqrt(gm (2/r_a - 1/a'))`; node = prograde
  `(v_a' - v_a)` (empty when `<= 0` or the orbit is open).

- [ ] **Step 8: Run the tests; record** TEI delta-v (LLO, NRHO), braking delta-v, MCC sizes, planning wall times in the
  ledger.

Run: `STARSIM_VERBOSE=1 powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[return]~[slow],[burn]"`
then the slow NRHO case alone: `-TestFilter "TEI from the NRHO costs less than from the LLO"`.
Expected: PASS.

- [ ] **Step 9: Commit and push** (after `-Fast`)

```bash
git add core
git commit -m "feat(mission): Earth return targeting from lunar orbit - TEI, mid-course and braking burn"
git push origin main
```

---

### Task 7: Mission config, start state and propellant planning

**Files:**
- Create: `config/missions/ship_lunar_return_llo.toml`, `config/missions/ship_lunar_return_nrho.toml`,
  `config/missions/ship_lunar_aircatch_llo.toml`, `core/include/starsim/sim/lunar_return_mission.hpp`,
  `core/src/sim/lunar_return_mission.cpp`, `core/tests/test_lunar_return_mission.cpp`,
  `core/tests/lunar_return_test_support.hpp`
- Modify: `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp`,
  `core/include/starsim/sim/mission_start.hpp`, `core/src/sim/mission_start.cpp`, `core/include/starsim/sim/flight_session.hpp`
  (declaration of `LunarReturnSetup` only - the steps come in Task 8), `config/README.md`, `core/CMakeLists.txt`,
  `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 5 (`entry_corridor`, `max_entry_speed`), Task 6 (`EarthReturnPlanner`, `return_asymptote`).
- Produces:
  - `config::MissionKind::LunarReturn`; `struct config::LunarReturnMission` (fields as the TOML below);
    `load_lunar_return_mission(config_dir, id)`.
  - `struct sim::LunarReturnSetup { mission::EarthReturnParameters ret; bool aerocapture = false;
    gnc::EntryGuidanceParameters entry; double aerocapture_apoapsis_m = 400e3; double perigee_raise_height_m = 200e3;
    gnc::DeorbitParameters deorbit; };` (in `flight_session.hpp`).
  - `sim::LunarReturnBudget { bool closes; double propellant_kg; double tei_delta_v_mps; double braking_delta_v_mps;
    double entry_speed_mps; gnc::EntryCorridor corridor; std::string reason; }`.
  - `sim::MissionStart start_lunar_return_mission(FlightSession&, const std::filesystem::path& config_dir,
    const config::LunarReturnMission&, time::Epoch epoch, const config::VehicleConfig* vehicle = nullptr);`
  - `sim::LunarReturnSetup lunar_return_setup(const config::LunarReturnMission&, ...)` and
    `sim::place_lunar_return_ship(FlightSession&, const config::LunarReturnMission&, const config::VehicleConfig& ship,
    time::Epoch, double propellant_kg, const mission::EarthReturnParameters&)`.

- [ ] **Step 1: Mission files.** `config/missions/ship_lunar_return_llo.toml`:

```toml
# Lunar return, mission 2 of the lunar spec (l5): a Block 3 ship in the 100 km polar LLO, tanked by tankers with just the
# propellant of its return (the start state; tankers, rendezvous and transfer are M9), flies TEI, two mid-course
# corrections and the braking burn (user 2026-10-16: the vacuum Raptors brake at ~2,000 km to the highest entry speed
# within the heat-shield limits), then the direct (skip) entry to Starbase and the flip and catch (b2/b3).

[mission]
name = "Rückkehr vom Mond (LLO, direkt)"
kind = "lunar_return"            # FlightSim.start_mission dispatches on it
vehicle = "starship_block3"
stage = "ship"
site = "starbase_olp1"

[orbit]
epoch_utc = "2026-11-08T00:00:00"   # [E]
kind = "llo"
altitude_km = 100.0                 # [S] spec
inclination_deg = 90.0              # [S] spec: polar; the node is placed so the plane contains the return asymptote

[return]
variant = "direct"                  # "direct" | "aerocapture"
flight_time_days = 3.5              # [E] spec ~3-4 days (first guess)
search_days = 1.0                   # [E] departure revolutions scanned
mcc_after_tei_days = [1.0, 2.5]     # [E] spec: two corrections
entry_interface_km = 120.0          # b2
braking_height_km = 2000.0          # [E] user 2026-10-16 (burn ends ~7 min before the interface: time to turn belly first)
min_corridor_width_deg = 1.0        # [E] the planned angle sits in the middle: +-0.5 deg of margin
nominal_bank_deg = 50.0             # [E] as b4

[budget]
mcc_allowance_mps = 30.0            # [E]
coast_attitude_kg_per_day = 1000.0  # [D] l3: Sun hold ~0.9 t/day of cold gas
reserve_kg = 10000.0                # [E] as the lunar landing
```

  `ship_lunar_return_nrho.toml`: same with `[orbit] kind = "nrho"` and the NRHO seed keys copied from
  `hls_lunar_nrho.toml`; `ship_lunar_aircatch_llo.toml`: LLO with `variant = "aerocapture"`, `aerocapture_apoapsis_km =
  400.0  # [E] spec`, `perigee_raise_km = 200.0  # spec`, `min_corridor_width_deg = 0.5  # [E]` and a `[deorbit]`
  section copied from `ship_return_starbase.toml` (b4 after the capture).

- [ ] **Step 2: Write the failing tests** (`core/tests/test_lunar_return_mission.cpp`, `[lunar][return][config]`):

```cpp
TEST_CASE("Lunar return missions load", "[lunar][return][config]") {
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "ship_lunar_return_llo") == config::MissionKind::LunarReturn);
    CHECK(m.stage_id == "ship");
    CHECK(m.orbit == "llo");
    CHECK_FALSE(m.aerocapture);
    CHECK(m.braking_height_m == Catch::Approx(2000e3));
    const auto a = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_aircatch_llo");
    CHECK(a.aerocapture);
    CHECK(a.aerocapture_apoapsis_m == Catch::Approx(400e3));
    CHECK(a.deorbit.search_window_s > 0.0);
}

TEST_CASE("Lunar return start tanks the ship for its return and plans it", "[lunar][return][session]") {
    // Session with Moon orientation (as the lunar landing mission tests); start_mission("ship_lunar_return_llo"):
    // return_planned; the ship's propellant lies between 250 and 400 t ([D] planning estimate ~318 t: TEI 0.9 +
    // braking 1.5 km/s + MCC + coast + 35 t landing + 10 t reserve on 255 t dry); it is in a ~100 km polar LLO;
    // session.lunar_step() == LunarStep::ReturnDeparture (Task 8 adds the enum value; until then this CHECK is the
    // RED of Task 8 - write it now, expect it to fail to compile, and keep it).
}

TEST_CASE("Lunar return start refuses without an entry corridor", "[lunar][return][session]") {
    // Review Focus 2: a ship whose heat-rate limit is lowered to 0.3 MW/m^2 (vehicle override): no admissible entry
    // speed above the planner's floor -> return_planned false, refusal starts with "Kein Eintrittskorridor".
}

TEST_CASE("Lunar return start refuses a return the tanks cannot hold", "[lunar][return][session]") {
    // a ship with dry mass 1,500 t (vehicle override): refusal contains "Treibstoff".
}

TEST_CASE("The aligned LLO contains the return asymptote", "[lunar][return]") {
    // place_lunar_return_ship: the angle between the LLO plane normal and the asymptote is 90 deg within 0.1 deg; the
    // orbit is 100 +- 1 km circular and polar within 0.1 deg (MOON_ME).
}
```

- [ ] **Step 3: Run to see them fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[return][config]"`
Expected: compile errors.

- [ ] **Step 4: Implement the config** (`mission_config.cpp`, after `load_lunar_landing_mission`, same TOML helper
  style): `"lunar_return"` -> `MissionKind::LunarReturn` in `mission_kind`; the struct:

```cpp
// Ship return from lunar orbit (config/missions/<id>.toml, lunar l5): the start orbit (LLO or NRHO), the variant
// (direct entry / aerocapture) and the return planning settings. The propellant is computed by the start.
struct LunarReturnMission {
    std::string id, name, vehicle_id, stage_id, site_id;
    time::UtcDateTime epoch_utc;
    std::string orbit;  // "llo" | "nrho"
    double llo_altitude_m = 100e3;
    double llo_inclination_rad = 0.5 * constants::pi;
    math::Vec3 nrho_apolune_position{}, nrho_apolune_velocity{};
    int nrho_revolutions = 0, nrho_synodic_months = 0, nrho_reference_revolutions = 0;
    bool aerocapture = false;
    double flight_time_s = 3.5 * 86400.0;
    double search_s = 86400.0;
    std::vector<double> mcc_after_tei_s;
    double entry_interface_height_m = 120e3;
    double braking_height_m = 2000e3;
    double min_corridor_width_rad = 1.0 * constants::deg_to_rad;
    double nominal_bank_rad = 50.0 * constants::deg_to_rad;
    double aerocapture_apoapsis_m = 400e3;
    double perigee_raise_height_m = 200e3;
    ShipReturnDeorbit deorbit;  // [deorbit] (aerocapture): entry_flight_path, nominal bank, crossrange, window
    double mcc_allowance_mps = 30.0;
    double coast_attitude_kg_per_day = 1000.0;
    double reserve_kg = 10000.0;
};
```

  (`ShipReturnDeorbit`: factor the five deorbit fields of `ShipReturnMission` into a struct reused by both; adjust
  `load_ship_return_mission` and its users - bindings included - without behaviour change.)

- [ ] **Step 5: Implement the start** (`lunar_return_mission.cpp`):
  1. `ship_config(mission, vehicle)`: the vehicle cut to the stage (as `test_ship_return_mission.cpp::place_mission`).
  2. Onboard entry model: an `EntryGuidance` built on a probe `FlightBody` of the ship at the entry mass (dry +
     `landing_propellant_kg`) - the same way `plan_ship_return` gets `onboard.model()` - plus the radiative table.
  3. Entry speed: `gnc::max_entry_speed(model, query, min_corridor_width, 8000, 11500)` with the ship's heat shield,
     the site's latitude and an eastward heading [E] (`query.aerocapture_apoapsis_m` for the aerocapture variant);
     below 8000: refuse "Kein Eintrittskorridor innerhalb der Hitzeschild-Grenzen". Corridor at that speed -> planned
     angle `middle_rad`.
  4. Start state: LLO - `return_asymptote(...)` at `epoch + 1 h` (direction `s`), the polar plane containing it: plane
     normal `n = normalized(cross(moon_pole_gcrf(epoch), s))`, a circular state at the LLO radius in that plane
     (argument of latitude such that the departure point lies ~1/4 revolution ahead), Moon state added (Earth-centred
     GCRF). NRHO - the reference (built as `lunar_outbound_parameters` builds it, first perilune at `epoch + 1 d`) at
     `epoch`.
  5. Propellant fixed point: `m_end = dry + landing_propellant (+ aerocapture: raise ~60 m/s + deorbit ~100 m/s [E] on
     the sea-level Isp)`; iterate up to 5 times: `m0 = m_end * exp((braking + mcc_allowance) / ve_vac) * exp(tei / ve_vac)
     + coast_days * coast_kg_per_day + reserve`, re-planning TEI with the finite-burn model at `m0` each time, until
     `|m0 - m0_prev| < 100 kg`. Propellant above the tanks' capacity: refuse "Kein Rückflug: Treibstoff (... t) übersteigt
     die Tanks (... t)".
  6. Place the ship with that propellant (tank scaling as the ship return test), attitude prograde with the belly to the
     Moon, `session.plan_lunar_return(setup)` (Task 8; until then the start returns after placement with
     `return_planned = false` and refusal "Rückflug-Autopilot fehlt" - Task 8 replaces this line).
  7. `mission_start.cpp`: `case config::MissionKind::LunarReturn:` -> `start_lunar_return_mission(..., time.from_utc(
     m.epoch_utc))`; update the `start_mission` comment.

- [ ] **Step 6: Run the tests** (the `lunar_step()` CHECK stays red until Task 8: comment it out with
  `// Task 8:` and restore it there)

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[return]~[slow]"`
Expected: PASS; record the computed entry speed, corridor and propellant in the ledger.

- [ ] **Step 7: Commit and push** (after `-Fast`)

```bash
git add config core
git commit -m "feat(sim): lunar return mission config, start state and propellant planning"
git push origin main
```

---

### Task 8: Session autopilot - TEI, corrections, braking, direct entry and catch

**Files:**
- Create: `core/src/sim/flight_session_lunar_return.cpp`, `core/tests/test_lunar_return_session.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (Burn completion, Return
  entry parameters), `core/src/sim/flight_session_lunar.cpp` (`run_lunar_steps` dispatch, `reset_lunar`,
  `to_string(LunarStep)`), `core/src/sim/flight_session_return.cpp` (`start_entry_guidance` parameters + radiative),
  `core/src/sim/lunar_return_mission.cpp` (call `plan_lunar_return`), `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: Tasks 4, 6, 7.
- Produces:
  - `LunarStep` values `ReturnDeparture` ("Abflug zur Erde"), `Tei` ("TEI"), `ReturnCoast` ("Rückflug"),
    `ReturnCorrection` ("Kurskorrektur"), `Braking` ("Bremsbrennen"), `EarthEntry` ("Wiedereintritt"),
    `Aerocapture` ("Aircatch"), `CaptureCoast` ("Freiflug zum Apogäum"), `PerigeeRaise` ("Perigäumsanhebung")
    (the last three are used by Task 9).
  - `bool FlightSession::plan_lunar_return(const LunarReturnSetup& setup);` (false with `lunar_status()` German reason)
  - `const mission::ReturnPlan* FlightSession::lunar_return_plan() const;`

- [ ] **Step 1: Write the failing tests** (`core/tests/test_lunar_return_session.cpp`):

```cpp
TEST_CASE("Lunar return: TEI is flown and the coast reaches the braking height", "[lunar][return][session]") {
    // start_mission("ship_lunar_return_llo"); advance until the step is ReturnCoast: one Tei burn recorded in
    // lunar_burns() with |dv| within 2 % of the plan; the ship is Earth-bound (osculating apogee about the Earth
    // < 500,000 km after leaving the Moon's sphere).
}

TEST_CASE("Lunar return direct: the ship is caught at Starbase", "[lunar][return][session][slow]") {
    // start_mission("ship_lunar_return_llo"); advance in 1-day calls (Review Focus 5) until the body is caught,
    // impacted or 8 days passed. Expect: caught (session.body(0).caught()), braking burn recorded at the planned height
    // within 50 km (the burn centre's geodetic height), peak heat rate < 1.2 MW/m^2, propellant left >= 0 (record it),
    // no "Hitzeschild versagt". Record TEI, MCC sizes, braking dv, entry speed / angle, peak heat (conv + rad), peak load,
    // gate miss, catch offset in the ledger.
}

TEST_CASE("Lunar return refuses a second plan while flying", "[lunar][return][session]") {
    // plan_lunar_return during ReturnCoast returns false and keeps the running plan.
}
```

  Restore Task 7's `// Task 8:` CHECK.

- [ ] **Step 2: Run to see them fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[return][session]~[slow]"`
Expected: compile errors.

- [ ] **Step 3: Implement the steps** (`flight_session_lunar_return.cpp`; header comment `// FlightSession: lunar l5 -
  the ship's return from lunar orbit (TEI, corrections, braking burn, entry or aerocapture).`):

```cpp
bool FlightSession::plan_lunar_return(const LunarReturnSetup& setup) {
    if (bodies_.empty() || lunar_step_ != LunarStep::None ||
        (autopilot_.state != AutopilotState::Off && autopilot_.state != AutopilotState::Orbit)) {
        return false;  // a flight under way keeps its plan
    }
    if (!moon_orientation_) {
        lunar_status_ = "Kein Rückflug: keine Mondorientierung";
        return false;
    }
    if (reference_body(controlled_) != ephemeris::Body::Moon) {
        lunar_status_ = "Kein Rückflug: nicht beim Mond";
        return false;
    }
    const auto configurations = burn_configurations(&detail::vacuum_groups);
    if (configurations.empty()) {
        lunar_status_ = "Kein Rückflug: keine Vakuum-Triebwerke";
        return false;
    }
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    const double isp_s = configurations.front().thrust_n / configurations.front().mass_flow_kgps /
                         constants::standard_gravity_mps2;
    mission::EarthReturnPlanner planner(gravity_, *env_.ephemeris, earth_);
    planner.set_finite_burn(mission::FiniteBurnModel{vehicle.mass_properties().mass_kg, configurations.front().thrust_n,
                                                     configurations.front().mass_flow_kgps});
    auto planned = setup;
    planned.ret.earliest_departure_s = std::max(planned.ret.earliest_departure_s, min_departure_lead_s);
    try {
        lunar_return_plan_ = planner.plan(controlled_state(), epoch_, planned.ret, vehicle.mass_properties().mass_kg,
                                          isp_s, vehicle.propellant_kg());
    } catch (const std::exception&) {
        lunar_return_plan_.reset();
        lunar_status_ = "Kein Rückflug: Planung fehlgeschlagen";
        return false;
    }
    if (!lunar_return_plan_->feasible) {
        lunar_status_ = lunar_return_plan_->reason;
        return false;
    }
    lunar_return_ = planned;
    lunar_mcc_index_ = 0;
    lunar_burns_.clear();
    lunar_step_ = LunarStep::ReturnDeparture;
    lunar_step_epoch_ = lunar_ignition_lead(lunar_return_plan_->tei);
    autopilot_.state = AutopilotState::Lunar;
    attitude_mode_ = {};
    lunar_status_ = "Rückflug: TEI " + detail::german(lunar_return_plan_->tei_delta_v_mps, 0) + " m/s, Bremsen " +
                    detail::german(lunar_return_plan_->braking_delta_v_mps, 0) + " m/s geplant";
    return true;
}
```

  (`vacuum_groups` moves into `flight_session_lunar_detail.hpp` as `detail::vacuum_groups`.) `run_lunar_steps` routes
  the return steps to `run_lunar_return_steps(flying)`:

```cpp
void FlightSession::run_lunar_return_steps(Flying& flying) {
    if (flying.body->impacted()) {
        const auto& cause = flying.body->breakup_reason();
        lunar_abort("Rückflug beendet: " + (cause.empty() ? std::string("Aufschlag") : cause));
        return;
    }
    mission::EarthReturnPlanner planner(gravity_, *env_.ephemeris, earth_);
    const auto& target = lunar_return_->ret.target;
    switch (lunar_step_) {
        case LunarStep::ReturnDeparture: {
            if (epoch_ - lunar_step_epoch_ < 0.0) break;
            // re-solved for the finite burn from the state now (as the TLI), the plan's node epoch
            auto tei = lunar_return_plan_->tei;
            const auto configurations = burn_configurations(&detail::vacuum_groups);
            planner.set_finite_burn(mission::FiniteBurnModel{flying.body->vehicle().mass_properties().mass_kg,
                                                             configurations.front().thrust_n,
                                                             configurations.front().mass_flow_kgps});
            const auto solved = planner.solve(controlled_state(), epoch_, tei.epoch, target, tei.delta_v_gcrf_mps);
            if (solved.converged) tei = solved.burn;
            if (lunar_burn(tei, true, LunarStep::Tei)) {
                lunar_step_ = LunarStep::Tei;
                lunar_status_ = "TEI: " + detail::german(norm(tei.delta_v_gcrf_mps), 0) + " m/s";
            }
            break;
        }
        case LunarStep::Tei:
            lunar_tli_end_ = epoch_;  // the MCC clock (shared with the outbound flight)
            lunar_return_enter_coast();
            break;
        case LunarStep::ReturnCorrection:
            ++lunar_mcc_index_;
            lunar_return_enter_coast();
            break;
        case LunarStep::ReturnCoast:
            lunar_sun_coast(flying);
            if (epoch_ - lunar_step_epoch_ >= 0.0) lunar_return_coast_event();
            break;
        case LunarStep::Braking:
            lunar_return_after_braking(flying);
            break;
        default:
            break;  // EarthEntry: the autopilot state is Return (b2 / b3); Task 9: aerocapture steps
    }
}
```

  `lunar_return_enter_coast()`: next MCC epoch (`lunar_tli_end_ + mcc_after_tei_s[i]`, skipped when within
  `mcc_min_before_braking_s` of the braking epoch), else the braking ignition lead: `planner.braking_epoch(...)` minus
  half the braking burn (rocket equation on the vacuum set) minus `burn_preparation_s`; Sun hold as `lunar_enter_coast`.
  Status "Rückflug: Kurskorrektur n in x h" / "Rückflug: Bremsbrennen in x h".
  `lunar_return_coast_event()`: MCC -> `planner.mid_course(...)` at `epoch_ + burn_preparation_s`, flown by
  `lunar_burn(node, false, LunarStep::ReturnCorrection)`; braking -> `planner.braking(state, epoch_, target)` with the
  finite model set, flown by `lunar_burn(node, true, LunarStep::Braking)`; not converged: the plan's vis-viva impulse at
  the braking epoch (status "Bremsbrennen ungelenkt") - never skipped silently.
  `lunar_return_after_braking()`: direct ->

```cpp
    lunar_step_ = LunarStep::EarthEntry;
    deorbit_parameters_.entry_interface_height_m = target.interface_height_m;  // entry_start_epoch() uses it
    return_entry_parameters_ = lunar_return_->entry;                            // lunar_return(): skips carried
    ship_return_ = true;
    autopilot_.state = AutopilotState::Return;
    entry_start_ = entry_start_epoch();
    attitude_mode_ = {};
    lunar_status_ = "Wiedereintritt: " + detail::german(lunar_return_arrival_speed(), 0) + " m/s";
```

  In `flight_session.cpp`, AutopilotState::Return start: `start_entry_guidance(flying, capsule ? capsule() :
  return_entry_parameters_.value_or(gnc::EntryGuidanceParameters{}))`; `start_entry_guidance` passes the session's
  radiative table (as `std::shared_ptr`; hold the table in the session as `std::shared_ptr<const
  aero::RadiativeHeatingTable>` from Task 2 on so it can be shared) to the `EntryGuidance`. `reset_lunar` /
  `reset_mission_state` clear `lunar_return_`, `lunar_return_plan_`, `return_entry_parameters_`.
  `coast_allowance_s` needs no change (state Lunar: `lunar_step_epoch_`; state Return: `entry_start_`).
  `lunar_burn`'s burn-completion path returns to state Lunar (lunar step set): the step handlers above run next tick.

- [ ] **Step 4: Wire the start** (`lunar_return_mission.cpp`): replace Task 7's placeholder line with
  `s.return_planned = session.plan_lunar_return(setup); if (!s.return_planned) s.refusal = session.lunar_status();`.

- [ ] **Step 5: Run the tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[return][session]~[slow]"`
then the slow mission flight alone: `-TestFilter "Lunar return direct: the ship is caught at Starbase"`.
Expected: PASS. If the catch fails, use `STARSIM_DEBUG_ENTRY=1` / `STARSIM_DEBUG_FLIP=1` (superpowers:systematic-debugging)
- the b2 terminal phase was tuned for 7.4 km/s entries: a lunar-return entry arrives at the terminal phase with another
  energy state; record the finding, do not retune b2 blindly.

- [ ] **Step 6: Commit and push** (after `-Fast`)

```bash
git add core
git commit -m "feat(sim): lunar return autopilot - TEI, corrections, braking burn and direct entry to the catch"
git push origin main
```

---

### Task 9: Aerocapture variant - pass, coast to apogee, perigee raise, b4

**Files:**
- Modify: `core/src/sim/flight_session_lunar_return.cpp`, `core/src/sim/flight_session.cpp` (phase `Exit` hand-back in
  `advance_to`'s guidance loop), `core/include/starsim/sim/flight_session.hpp`
- Test: `core/tests/test_lunar_return_session.cpp`

**Interfaces:**
- Consumes: `EntryGuidanceParameters::aerocapture`, `EntryPhase::Exit` (Task 4), `gnc::raise_periapsis_at_apoapsis`
  (Task 6), `plan_ship_return` (b4).
- Produces: steps `Aerocapture`, `CaptureCoast`, `PerigeeRaise`; on b4's hand-over the lunar step returns to `None`
  with status "Rückkehr nach Starbase (b4)".

- [ ] **Step 1: Write the failing tests:**

```cpp
TEST_CASE("Lunar return aerocapture: captured, perigee raised, caught by b4", "[lunar][return][session][slow]") {
    // start_mission("ship_lunar_aircatch_llo"); advance in 1-day calls until caught / impacted / 10 days. Expect: the
    // exit apoapsis within 400 +- 100 km [E] (record), after the perigee raise both apsides above 150 km (perigee 200 +-
    // 20 km), then b4 plans its deorbit (deorbit_plan()->feasible) and the ship is caught. Peak heat < 1.2 MW/m^2.
}

TEST_CASE("Aerocapture outcome names a capture that stays in the atmosphere", "[lunar][return][session][slow]") {
    // Review Focus 4: truth air density x 1.6 (set_air_density_factor) and truth aero x 1.3 - beyond the guidance's
    // reach: the status names the cause ("Aircatch: in der Atmosphäre geblieben" or "... zerbrochen") and the
    // autopilot is off; no perigee raise is attempted on an orbit that dips below 100 km after the raise window.
}
```

- [ ] **Step 2: Run to see them fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "Lunar return aerocapture: captured, perigee raised, caught by b4"`
Expected: FAIL (aerocapture variant hands to the direct entry or stops after braking).

- [ ] **Step 3: Implement.** `lunar_return_after_braking()` aerocapture branch: step `Aerocapture`,
  `return_entry_parameters_ = gnc::EntryGuidanceParameters::aerocapture(apoapsis, arrival inclination)`, state
  `Return`, `entry_start_` as the direct branch; `ship_return_ = false` (no catch guidance at a gate). In `advance_to`'s
  per-body loop, after `flying.entry_command = flying.entry_guidance->update(...)`:

```cpp
                if (flying.entry_command.phase == gnc::EntryPhase::Exit) {
                    // aerocapture (l5): out of the atmosphere - the lunar return plans the perigee raise
                    flying.entry_guidance.reset();
                    if (&flying == &bodies_[controlled_]) lunar_return_captured(flying);
                }
```

  `lunar_return_captured()`: two-body orbit about the Earth from the state; open orbit -> `lunar_abort("Aircatch:
  Bahn nicht geschlossen (Abprall)")`; else `gnc::raise_periapsis_at_apoapsis(state, epoch_, gm_earth, R +
  perigee_raise_height_m)`; node empty (perigee already high) -> straight to `EarthOrbit` handling; else step
  `CaptureCoast`, autopilot `Lunar`, `lunar_step_epoch_` = ignition lead on the deorbit engine set
  (`stage.deorbit_engine_groups`, the belly sea-level Raptor: a ~55 m/s burn at ~300 t lasts ~6 s), status "Aircatch:
  Apogäum x km, Perigäumsanhebung y m/s in z min". `CaptureCoast` due -> `execute_burn(node, deorbit_engine_groups)`
  -> step `PerigeeRaise` (record in `lunar_burns_`). `PerigeeRaise` settled -> check both apsides > 150 km [E] (else
  `lunar_abort("Aircatch: Perigäum zu tief")`), then `lunar_step_ = LunarStep::None`, `autopilot_.state = Off`,
  status "Rückkehr nach Starbase (b4)", `plan_ship_return(lunar_return_->deorbit)` (false: status = its reason).
  If the aerocapture guidance reaches phase `Gate` (the ship never exited: it stays in the air) ->
  `lunar_abort("Aircatch: in der Atmosphäre geblieben")` and the entry guidance is dropped (the ship falls on;
  honest outcome).

- [ ] **Step 4: Run the tests** (both slow cases alone, then `[return]~[slow]`)
Expected: PASS; record exit apoapsis / perigee, raise delta-v, b4 deorbit delta-v, wait to the deorbit pass, catch.

- [ ] **Step 5: Commit and push** (after `-Fast`)

```bash
git add core
git commit -m "feat(sim): lunar return aerocapture - one pass, perigee raise, b4 catch"
git push origin main
```

---

### Task 10: Campaigns, NRHO flight, docs, full suite, final review

**Files:**
- Create: `core/tests/test_lunar_return_campaign.cpp`
- Modify: `docs/subsystems.md` (new "Lunar return (l5)" paragraph), `CLAUDE.md` (implemented modules list: aero
  radiative heating, gnc entry corridor / aerocapture, mission earth_return), `config/README.md` (radiative table,
  missions), spec (status line), this plan's Ledger, memory `project-lunar-mission.md`

- [x] **Step 1: Campaigns** (`[lunar][return][validation][slow]`, 7 cases per TEST_CASE as the l4 campaign so ctest runs
  them in parallel; seeds 1-21):
  - Direct (21 cases): TEI execution +-1 % / 0.5 deg (`apply_velocity_error` after the burn), braking execution +-1 % /
    0.5 deg, entry angle dispersion +-0.1 deg (applied as a velocity rotation 10 min before the interface), truth aero
    +-10 % (`set_hull_aero_scale`), truth density +-15 % (`set_air_density_factor`), thrust +-2 %
    (`Vehicle::set_thrust_factor`). Expect: all caught, or every miss explained in the ledger with its cause (spec
    validation: "direct at the gate as b2's campaign then caught"; the b2 campaign itself holds 11/12 within 0.11 km).
  - Aerocapture (21 cases): same dispersions. Expect: exit apoapsis within a band [E] (record min/max; propose the band
    from the spread), perigee after the raise above 150 km in all cases, caught by b4 in all cases.
  - Hidden single cases `STARSIM_RETURN_SEED=<n>` with `STARSIM_VERBOSE=1` (as the ship return campaign).
- [x] **Step 2: NRHO variant flight** (`[lunar][return][session][slow]`): start "ship_lunar_return_nrho", caught;
  TEI delta-v recorded against the spec's 0.4-0.5 km/s.
- [x] **Step 3: Artemis I comparison** (magnitude only, in docs): Orion entered at ~11 km/s and skipped (Artemis I,
  Dec 2022); our ship enters at the computed speed after ~1.5 km/s of braking - state both, no test.
- [x] **Step 4: Docs.** `docs/subsystems.md`: radiative heating (source, interpolation, validity), corridor numbers, max
  entry speed, braking burn (height, delta-v, propellant), TEI/MCC numbers, skip / aerocapture guidance, campaign
  results, diagnostics, known gaps (non-equilibrium radiation, ablation-free tiles, Starship heat-flux limit [E]).
- [x] **Step 5: Full suite + smoke**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` (with bindings, all tests incl. `[slow]`) and
`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd`
Expected: all pass (record counts and wall time).

- [x] **Step 6: Final review.** One opus reviewer (superpowers:requesting-code-review) over the l5 commits with the
  spec's l5 section + amendment, this plan and its Review Focus. Fix Critical / Important; list deferred minors in the
  Ledger.
- [x] **Step 7: Memory, commit, push.** Update `project-lunar-mission.md` (l5 complete, numbers, next: l6), commit:

```bash
git add core docs CLAUDE.md config/README.md
git commit -m "test(sim): lunar return campaigns (direct and aerocapture), NRHO return; docs for l5"
git push origin main
```

---

## Ledger

(Rulings, measured numbers and deferred minors are recorded here during execution.)

Planning 2026-10-16:
- Probe (onboard predictor, ship 290 t at entry, L/D 0.71, C_D A 498 m^2, nose radius 6 m): 10.9 km/s - lift up skips out
  shallower than ~-6 deg, constant lift down captures from ~-5.5 deg; convective peaks 0.75-1.06 MW/m^2 (lift-up passes
  -5.5..-6 deg), 1.4-3.7 MW/m^2 lift down; heat load <= 2.6e8 J/m^2. 7.4 km/s LEO entries: 0.19-1.4 MW/m^2 by bank.
- NASA TM-102652 table 6 (60 km): 11 km/s Rn 5 m 6.78 MW/m^2 radiative; 10 km/s 1.94; 9.5 km/s 0.63; 9.0 km/s 0.18.
- User decision 2026-10-16: braking burn + both variants (spec amendment). Research: no published Starship heat-flux
  limit; limits unchanged.
- Deferred to the user's report (l4 final minors M4): campaign sink <= 1.1 m/s accepted (spec <= 1 m/s, measured
  0.98-1.03); high gate 700/800 m vs the spec's ~2 km.

Final 2026-10-18: Task 10 complete - campaigns direct 21/21, aircatch 21/21, full suite 628/628 + smoke; opus review 0 Critical / 4 Important (fixed: gate hand-over height bounded to dense air, short-braking test, MCC slot skip, re-plan without Earth approach), deferred minors in .superpowers/sdd/2026-10-16-lunar-l5-return/progress.md. L5 COMPLETE.
