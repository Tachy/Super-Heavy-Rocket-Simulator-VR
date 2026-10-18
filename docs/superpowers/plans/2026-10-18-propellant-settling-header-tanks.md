# Propellant settling and header tanks Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (native, user 2026-10-18: "gleich
> implementieren, so lange nichts offen ist"). Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Tank liquid settles perpendicular to the apparent gravity, and Starship gets header tanks that feed the
flip and landing burn, so a lunar-return ship with main-tank residue trims with a forward CG.

**Architecture:** A pure geometry table (`vehicle/liquid_settling`) gives a tilted-surface liquid centroid per tank;
`Vehicle` keeps one lagged liquid "down" direction fed by `FlightBody` (fidelity flag
`FlightEnvironment::propellant_settling`); tanks get a role (main / header) and stages a feed source switched by the
session at the flip.

**Tech Stack:** C++20 core, Catch2, TOML config.

**Spec:** `docs/superpowers/specs/2026-10-18-header-tanks-design.md` (revision 2)

## Global Constraints

- SI units, radians; frame/unit in names (`specific_force_body_mps2`); no magic numbers (named `constexpr` with [E]).
- `core` modules depend downwards only; `vehicle` must not include `sim` / `gnc`.
- Settling off => mass properties bit-identical to today (the aft column keeps its old code path).
- Headers are part of the 1,600 t: header 35 t, main 1,565 t, split at O/F 3.6.
- Tests that change behaviour get a provenance comment; tests > ~40 s tagged `[slow]`.
- During a task run only the touched tags; `-Fast` before every push; full suite in Task 4.

## Review Focus

- A stage with a tank of zero capacity (Rcs tanks, scaled-to-zero tanks): settling must skip it, no division by 0.
- Specific force exactly opposite the liquid direction (flip through 180 deg): rotation stays finite and unit.
- A ship loaded below the header capacity (e.g. 20 t): all in headers, main tanks empty, deorbit refuses honestly.
- Booster / Falcon 9 / Crew Dragon (no headers): select_feed no-op, load_propellant equals the old scaling.
- Stage separation copies the liquid direction and the feed source.

---

### Task 1: Liquid settling geometry

**Files:**
- Create: `core/include/starsim/vehicle/liquid_settling.hpp`, `core/src/vehicle/liquid_settling.cpp`
- Create: `core/tests/test_liquid_settling.cpp` (tag `[settling]`)
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt` (source lists)

**Interfaces (Produces):**
```cpp
namespace starsim::vehicle {
// Liquid below a plane perpendicular to "down" in a cylinder of radius 1 and length `aspect` (= L / r), centred at
// the origin, axis x. Lateral u: along down's component perpendicular to x; transverse w: the third axis.
struct LiquidMoments {
    double axial = 0.0, lateral = 0.0;                  // centroid (units of r)
    double axial_var = 0.0, lateral_var = 0.0, transverse_var = 0.0;  // about the centroid (units of r^2)
};
class SettlingTable {
public:
    explicit SettlingTable(double aspect);
    // fill: liquid volume over tank volume (clamped 0..1); tilt_rad: angle between down and -x (0..pi).
    LiquidMoments query(double fill, double tilt_rad) const;
};
std::shared_ptr<const SettlingTable> settling_table(double aspect);  // cached per aspect, thread-safe
// Direct integration (reference for tests, slow).
LiquidMoments settle_direct(double aspect, double fill, double tilt_rad);
}
```

- [ ] Step 1: tests - theta 0 column (fill 0.3: axial -L/2 + 0.15 L, axial_var (0.3 L)^2 / 12, lateral 0,
  lateral_var = transverse_var = 1/4); theta 90 deg fill 0.5: lateral 4/(3 pi), axial 0; mirror theta vs pi - theta;
  fill 1 = full cylinder (centroid 0, axial_var L^2/12); table vs `settle_direct` at off-grid (fill 0.037, tilt 3.3 deg
  and 71 deg, aspect 3.6) within 2 % of r (centroid) / 5 % (variances).
- [ ] Step 2: run `-TestFilter "[settling]"`, expect build failure.
- [ ] Step 3: implement. Per slice x (N_x = 100 midpoints) the disk part u >= u0 with u0 = (x cos t - c) / sin t
  (sin t ~ 0: full or empty slice): area acos(u0) - u0 s0, int u = (2/3) s0^3, int u^2 = (1/4)(pi/2 - asin u0 +
  u0 s0 (1 - 2 u0^2)), int w^2 = (2/3) int_{u0}^{1} (1-u^2)^{3/2} du with
  int (1-u^2)^{3/2} = u (5 - 2u^2) s / 8 + 3/8 asin u. Row per tilt (5 deg, 37 rows) with N_c = 200 offsets c over
  [min h, max h], h = x cos t - u sin t; store V, Mx, Mu, Sxx, Suu, Sww (raw moments). Query: per row find c with
  V = fill V_total (binary search + linear interpolation of the raw moments), blend rows linearly in tilt, then
  centroid = M / V and variance = S / V - centroid^2.
- [ ] Step 4: tests green; commit `feat(vehicle): tilted-surface liquid settling table`.

### Task 2: Settling in Vehicle and FlightBody (flag default off)

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp` (TankConfig `capacity_kg`), `core/src/config/vehicle_config.cpp`
- Modify: `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp` (state, settle, mass_properties)
- Modify: `core/include/starsim/vehicle/flight_body.hpp` (FlightEnvironment `propellant_settling = false`),
  `core/src/vehicle/flight_body.cpp` (specific force capture, call, engine moment about the full CG point)
- Test: `core/tests/test_vehicle_settling.cpp` (`[settling]`)

**Interfaces (Produces):** `Vehicle::settle_propellant(const math::Vec3& specific_force_body_mps2, double dt_s)`,
`const math::Vec3& Vehicle::liquid_down_body() const`, `void Vehicle::set_liquid_down_body(const math::Vec3&)` (tests,
mission starts), `TankConfig::capacity_kg`, `FlightBody::specific_force_body_mps2()`.

- [ ] Step 1: tests - default down -x and mass properties equal the column formula; ship at 64 t with down set for
  alpha 60 deg (d = (sin? ) from f = (-0.05, 0, +1) normalised): CG station forward by > 1 m and cg_offset z < 0
  (belly, -z); settle with f below 0.01 holds; lag: f constant, after tau the angle shrinks to ~e^-1; reversal
  f = +x after down = +x stays unit and ends at -x; a stage copy (split) keeps the direction.
- [ ] Step 2: implement (tilt, lateral unit, table path only when d != -x; capacity 0 -> column).
- [ ] Step 3: FlightBody: TickContext gets `Vec3* first_specific_force` written at tau == 0 (nongravitational body
  force + inverse-rotated external force, over mass); after the tick `if (env.propellant_settling)
  vehicle_->settle_propellant(...)`. Engine moment arm `e.gimbal_point_m - f.mass.cg_point_m()`.
- [ ] Step 4: `[settling]` green, `-Fast` green (flag off: bit-identical); commit
  `feat(vehicle): propellant settles with the apparent gravity (fidelity flag, off)`; push.

### Task 3: Header tanks, feed source, dry CG re-tune (settling off)

**Files:**
- Modify: `config/vehicles/starship_block3.toml`, `config/README.md`
- Modify: config tank parsing (role, diameter_m), derived `landing_propellant_kg` + mismatch error
- Create: `core/include/starsim/config/propellant_loading.hpp`, `core/src/config/propellant_loading.cpp`
  (`load_propellant(StageConfig&, double total_kg)`, `load_propellant(StageConfig&, double oxidizer_kg, double fuel_kg)`)
- Modify: `vehicle.hpp/.cpp` (FeedSource, `Stage::feed`, `select_feed`, flows, starvation, RCS fallback,
  `main_propellant_kg`, tank area from `diameter_m`)
- Modify: `core/src/sim/flight_session_return.cpp` (`run_ship_landing` selects Header), `lunar_return_mission.cpp`
  (`scale_tanks` -> `load_propellant`), `flight_session_lunar_return.cpp:384` (`main_propellant_kg()`)
- Modify: every ship-stage scaling site in tests (list from
  `grep -rn "tank.mass_kg \*=\|t.mass_kg \*=" core/tests`: ship_entry_scenario.hpp, test_ship_catch.cpp,
  test_deorbit_planner.cpp, test_entry_attitude.cpp, test_entry_heating.cpp, test_flight_session_entry.cpp,
  test_lunar_return_entry.cpp (4 sites incl. the per-contents gate replay), test_ship_deorbit.cpp,
  test_ship_landing.cpp, test_ship_return_mission.cpp; booster / HLS / Dragon sites unchanged)
- Test: `core/tests/test_header_tanks.cpp` (`[header]`)

- [ ] Step 1: tests (spec section "Headers") + dry CG test: ship alone, load_propellant(35 t), CG station 25.50 m
  +- 0.01 m from the ship's base.
- [ ] Step 2: implement config + vehicle + loading + session switch; dry CG from the CG test ([D] comment).
- [ ] Step 3: run ship tags (`[ship]`, `[entry]`, `[catch]`, `[header]`, `[deorbit]`; never `[mission]`), then
  `-Fast`; `python tools/vehicle_report.py starship_block3`; commit `feat(vehicle): Starship header tanks feed the
  flip and landing burn`; push.

### Task 4: Settling on, validation, docs, review

- [ ] Step 1: `propellant_settling = true` by default (FlightEnvironment and every session constructor path).
- [ ] Step 2: un-hide `[.header_tanks]` -> `[header_tanks][aerocapture]`; run it; aircatch seeds 1-7.
- [ ] Step 3: full suite (incl. `[slow]` campaigns) + smoke test; fix or report each moved case.
- [ ] Step 4: docs - `docs/subsystems.md` (ship dry CG, entry mass in tests, aerocapture finding closed, settling
  paragraph), `config/README.md`, CLAUDE.md implemented-modules line (vehicle: settling).
- [ ] Step 5: opus review of the plan's commits; fix Critical / Important; commit + push; ledger + memory.

## Ledger

(progress, decisions, findings - appended per session)
