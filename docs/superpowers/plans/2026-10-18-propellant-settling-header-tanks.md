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

Session 2026-10-18 (one session, Tasks 1-3):
- Finding that led to revision 2: header tanks alone (dry CG re-tuned to keep the landing-mass CG) moved the 64 t CG
  by 3 cm only (23.867 -> 23.836 m) - the surplus sat aft in the main tanks. User asked whether the liquid surface
  tilts with the ship: it did not (always a column on the tank bottom). Upper bound with the liquid at the forward
  domes: +1.8 m at 35 t, +3.0 m at 64 t.
- Probe (throwaway, settling on by default, NO headers): `[.header_tanks]` passed - axis error 1.0 deg (was 166),
  apoapsis 421.1 km, peak 915 kW/m2, 60.22 t left. Settling is the root cause of the aerocapture departure.
- Task 1 DONE (1924d7f): `vehicle/liquid_settling` - strips across the axis for tilts <= 45 / >= 135 deg (exact aft
  column), axial slices with analytic circular segments otherwise; 181 tilt rows x 121 offsets x 100 points, rows 0 /
  180 analytic. Table vs direct: centroid 2 % r, variances 5 % or 0.01 r^2 (shallow residues in a header).
- Task 2 DONE (e072b61, pushed): Vehicle::settle_propellant (threshold 0.01 m/s^2, tau = sqrt(2 D / a) in
  [0.5, 60] s, rotation on the sphere), FlightBody specific force from the first RK4 stage, flag
  `FlightEnvironment::propellant_settling` default false. DEVIATION: engine moments keep the axial CG - the full CG
  point moved the Falcon 9 / Crew Dragon ascent apoapsis 210 -> 223 km (spec amended).
- Task 3 DONE (285f2d1, pushed): header tanks (lox_header 27,391.3 kg base 44 m d 3.6; ch4_header 7,608.7 kg base
  20 m d 3.0), FeedSource + select_feed (session: run_ship_landing), RCS gas fallback, main_propellant_kg,
  landing_propellant_kg derived (mismatch = error), `config::fill_tanks` (named so: vehicle_config.cpp already has a
  `load_propellant` for the propellant combination), 15 ship-stage scaling sites in tests switched, the small-tank
  test scales capacity_kg too. Dry CG 23.516 m [D] (test: landing-mass CG 25.50 m within 0.01). Column inertia of a
  header uses its own radius. With headers + settling at 64 t (entry direction (0.05, 0, -1)): CG +0.95 m forward.
  vehicle_report shows header geometry (diameter_m).
- Tests: -Fast 582/583 - the one red is `fast planning keeps the LLO return plan` on WALL TIME only (19-21 s under
  the parallel ctest load; standalone 6.7 s before and after these changes; plan values unchanged). Pre-existing
  load sensitivity - tell the user; candidate fix: measure CPU time or run plan_speed serially.
- NEXT: Task 4 - settling on by default (FlightEnvironment default true; check every session constructor path),
  un-hide `[.header_tanks]`, full suite incl. [slow] campaigns + smoke, docs (subsystems.md: ship dry CG, entry mass
  in tests, aerocapture finding closed, settling paragraph; CLAUDE.md implemented modules), opus review.

Session 2026-10-18 (Task 4, steps 1-3):
- Steps 1-2 done (uncommitted): `FlightEnvironment::propellant_settling = true`; `[header_tanks][aerocapture]` un-hidden
  and green (axis error 1.3 deg, apoapsis 420.3 km, peak 904 kW/m2, 60.87 t left).
- Full suite: 650/654. Red: (1) LLO plan_speed wall time (known load sensitivity); (2) entry campaign "Entry guidance
  brings the ship to the Starbase flip gate from dispersed entries": cases 1/2/3/5/10 depart in pitch at peak q;
  (3) lunar direct seeds 7-9: seed 8 impacted; (4) ship catch campaign: case 12 "verfehlt" (sink 5.5 m/s).
- A/B with settling off: (2) and (4) fail identically -> TASK 3 REGRESSION (not seen: -Fast skips [slow]). Cause of
  (2): the dry CG re-tune (28 -> 23.516 m) was anchored at 35 t only; below 35 t all propellant sits in the headers, so
  the CG now moves AFT as the ship gets lighter (11 t: 24.16 m, was 27.14; 16.5 t: 24.48, was 26.73). Every failing
  case carries <= 16.5 t; the pitch-trim limit is ~24.5 m. (4): case 12 was the documented marginal case (runs dry, 1 m
  drop at 4.5 m/s); now 5.5 m/s.
- (3) is settling: seed 8 caught with settling off (8.9 t left). Flip identical (on/off); in the landing burn the liquid
  swings +-10 deg with the gimbal (lateral CG +-0.1 m, lagged) and after the 3->2 engine switch the SCvx plans stop
  converging (conv 0, defect 1.7-2.5) -> pitch limit cycle to 0.53 rad/s -> abort, impact. Settling off also had a
  failed plan at the same point: marginal case pushed over. `STARSIM_DEBUG_BURN` now prints CG and liquid direction.
- OPEN: user decision on (2)/(4) (entry propellant semantics vs dry CG), then (3).
- Ruling (user 2026-10-19, "Header immer voll"): headers are filled at launch and drawn only from the flip, so a
  returning ship always carries 35 t in them; test dispersions below 35 t were unphysical. Entry campaign: 35 t + 0-28 t
  main-tank residue (the old 28 t spread); catch campaign gate: 35 t + 0-3.5 t (the old 5 % spread width). Dry CG stays
  23.516 m. Cost if wrong: partial-header scenarios (e.g. a leaking header) remain untested below ~24.5 m CG.
  Result (settling on): entry campaign 12/12 (miss <= 2 m), catch campaign 20/21 (case 12 caught; 20 = engine failure).
- Seed 8 (user: investigate + fix). Sweep of the replayed gate (thrust 1.011-1.023 x aero 0.97-0.99): settling off
  11/15 caught, on 0/15 -> systematic, not chaos. Ablation (9 variants each): liquid lateral CG zeroed 7/9, main tanks
  as columns 6/9, headers as columns 0/9, aero moment without the lateral arm 7/9, RCS allocator on the axial CG 0/9.
  Mechanism: the 4.4 t main-tank residue (a ~5 cm pool in the flat 9 m tank model) swings with the gimbal and the flip,
  lateral CG ~0.1 m; aero normal force (up to 2.8 MN in the flip) x that offset = ROLL moment up to 0.05 MNm
  (pitch/yaw parts ~0). With two engines (asymmetric set) roll goes to the RCS only, and the burn's roll rate is barely
  damped even with settling off (w.x 0.01 -> 0.06 rad/s against a commanded -0.3..-0.9 MNm): the extra roll
  disturbance tips the SCvx plans into non-convergence. Catch campaign (headers only, <= 3.5 t residue) unaffected.
- User (2026-10-19): "rollen erzeugt kein Gegenmoment, da die Tanks ja zylinderfoermig sind und die Fluessigkeit durch
  die Traegheit an ihrem Platz bleibt" - correct: the wall pressure of a round tank passes through the axis, the liquid
  neither turns with the hull (no roll inertia, already modelled) nor gives it a roll moment. FIX: `MassProperties::
  roll_offset_m` (lateral offset of the parts that turn with the hull, over the whole mass); FlightBody's aero and
  parachute moments take roll about it, pitch / yaw about the full CG (`moment_about_cg`). Settling off: bit-identical.
  Test "Liquid off the axis gives no roll moment" RED (-1.8e-5 rad/s after one 5 ms tick, broadside 1,500 m/s at
  60 km) -> GREEN. Seed 8 gate sweep 7/9 caught (settling off 7/9 on the same grid); lunar direct seeds 7-9 3/3
  (8.6 / 10.9 / 11.2 t left). The RCS allocator still uses the full CG point (~2 % of its roll arm; minor, deferred).
  User also noted real tanks are domed (flat ends here exaggerate a shallow residue's swing) - follow-up candidate.
- Full suite 653/655 + smoke test passed. Red: LLO plan_speed wall time (known) and "Falcon 9 drone-ship mission":
  apoapsis 269 km (280 +- 5). Not the roll fix itself: the PEG cutoff state was identical (|v_go| 6.314 m/s); the test
  measured the orbit on a 1 s grid right after cutoff, INSIDE the Merlin's shutdown transient (6.4 m/s tail-off) when
  the cutoff fell just before a grid point. Ruling: the test now waits until no engine produces thrust -> 200 x 281 km,
  [asds] 17/17. Cost if wrong: none (measurement only).
- Aircatch seeds 1-7 threshold back to == 7 (known-finding comment removed); docs: subsystems.md (dry CG, entry mass
  and campaign propellant, settling paragraph incl. the roll rule, aerocapture finding CLOSED, catch campaign),
  CLAUDE.md implemented modules (vehicle). `STARSIM_DEBUG_BURN` prints CG and liquid direction.
- NEXT: commit + push, then Step 5 (opus review of the plan's commits bba1792..HEAD), then l6 Task 2.
