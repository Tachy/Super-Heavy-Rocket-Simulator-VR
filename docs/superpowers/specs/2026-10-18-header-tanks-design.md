# Propellant settling and header tanks (Starship Block 3) - design

Date: 2026-10-18. Status: approved by the user ("ja, und gleich implementieren, so lange nichts offen ist").
Revision 2 (replaces the header-only design of bba1792, see "History").

## Why

The aircatch seeds 4/7 depart at peak q (`docs/subsystems.md`, "Aerocapture attitude at peak q"): with 61-64 t
aboard the CG sits aft and the pitch trim pins both aft flaps at the 90 deg stop. The model puts every tank's
liquid as a settled column on the tank's AFT bottom, always (`Vehicle::mass_properties`, "Settled liquid column").
That is only true under thrust or on the pad. In a belly-first entry (alpha 55-75 deg, nose leading) the only
non-gravitational acceleration is aerodynamic: it pushes the hull towards the leeward side and slightly aft, so the
liquid runs to the belly side and to the FORWARD end of each tank; its free surface stands perpendicular to the
apparent gravity (user 2026-10-18: "der Flüssigkeitsspiegel kippt ja mit").

Estimate (column against the forward dome = upper bound; dry CG 28 m): CG at 35 t 25.50 -> 27.33 m, at 64 t
23.87 -> 26.86 m. With the surface tilt (~3 deg: axial / normal aerodynamic force ~0.05) the shift is smaller but of
the same sign; the CG then varies by ~0.5 m between 35 and 64 t instead of 1.6 m today.

History: revision 1 (header tanks alone, dry CG re-tuned to keep the landing-mass CG) moved the 64 t CG by 3 cm
only - the surplus above 35 t still sat aft in the main tanks. Header tanks stay in the design: in an entry the
main-tank residue lies away from the outlets, so flip and landing burn can only feed from the headers (user
2026-10-18: "the belly-flip burn depends on the header tanks anyway").

## User decisions

- Feed: ascent, deorbit and lunar-return burns draw from the main tanks; from the start of the belly flip (flip and
  landing burn) only the header tanks feed the engines. Main-tank residue stays aboard as dead mass.
- Filling: the headers are part of the 1,600 t [S]: header 35 t [E], main tanks 1,565 t. Full at liftoff,
  untouched until the flip.
- Liquid position follows the apparent gravity (this design, approved 2026-10-18).

## Design

### 1. Liquid geometry (`vehicle/liquid_settling`, pure)
- A tank is a cylinder (radius r, length L = capacity / (density x area)); the liquid fills the part below a plane
  whose normal is the "down" direction d (unit, body axes; d = -specific force / |specific force|).
- `SettlingTable` (normalised: r = 1, aspect L / r) holds, per tilt theta = angle(d, -x) on a 5 deg grid
  (0..180 deg), the liquid volume and its first and second moments (axial x, lateral u along d's lateral part,
  transverse w) over a grid of plane offsets c. Each entry integrates analytic circular segments over axial slices.
  A query (fill fraction, theta) inverts volume -> c by interpolation in each of the two neighbouring rows and blends
  them: centroid (axial, lateral) and variances. Tables are cached per aspect ratio (process-wide, thread-safe).
- Exact cases (tests): theta = 0 is the aft column (centroid -L/2 + h/2, variance h^2/12); theta = 90 deg is a
  horizontal cylinder (segment centroid 4 sin^3(a/2) / (3 (a - sin a)) for the segment angle a); volume and
  mirror symmetry (theta <-> 180 deg - theta flips the axial centroid).

### 2. Settling state (`vehicle`)
- `Vehicle` holds one liquid "down" direction for all its tanks (body axes, initial -x: settled aft).
- `Vehicle::settle_propellant(specific_force_body_mps2, dt_s)`: below 0.01 m/s^2 [E] the liquid holds its last
  position (coasts, orbital drag); above it the direction turns towards -f / |f| with the time constant
  tau = clamp(sqrt(2 D / |f|), 0.5 s, 60 s) [E] (D = vehicle diameter: the liquid's travel), as a rotation on the
  sphere (no blend through zero on reversals). Quasi-static: no slosh dynamics.
- `FlightBody::step` measures the specific force at the tick start (first RK4 stage: thrust + aero + fins + flaps +
  parachutes + RCS + contact force, body axes, over mass) and calls `settle_propellant` after the tick when the
  fidelity flag `FlightEnvironment::propellant_settling` is on. Off: the liquid stays aft and mass properties are
  bit-identical to today.
- `mass_properties()`: oxidizer and fuel tanks with a known capacity use the table when d != -x (the aft column
  stays the exact old code path); the liquid's lateral offset enters `cg_offset_m`; its pitch/yaw inertia is
  m r^2 (sigma_x^2 + (sigma_u^2 + sigma_w^2) / 2), no roll inertia (as today). Rcs tanks stay columns (bladders).
- Engine moments use the full CG point (lateral offset included), as aero and RCS already do.
- `TankConfig::capacity_kg` = the configured load at config load (configured loads are full tanks); scenarios
  change `mass_kg` only.
- Moving the CG inside the body frame without an internal reaction is a documented simplification [E].
- Not modelled: engine starvation when a main-tank outlet is uncovered, slosh, rotation (centripetal) effects.

### 3. Header tanks (`config`, `vehicle`, `sim`)
- `[[stage.tank]]` gets `role = "main" | "header"` (default main; headers only for oxidizer / fuel) and an
  optional `diameter_m` (default: the stage diameter).
- `starship_block3.toml`: `lox_main` / `ch4_main` reduced by 35 t (O/F 3.6), `lox_header` 27.4 t in the nose
  (base ~44 m, diameter 3.6 m [E]) and `ch4_header` 7.6 t on the CH4 main tank's lower dome (base 20 m, diameter
  3 m [E]) (2020-2022 SN-era reports; Block 3 placement not published).
- `landing_propellant_kg` is derived from the header contents when a stage has headers; giving a differing value
  is a config error.
- Each stage has a feed source `Main | Header` (default Main). Engines draw only from tanks of that role; a dry
  source flames the engines out (no silent switch). `Vehicle::select_feed(stage, Header)` on a stage without
  headers is a no-op; switching to a source that holds propellant clears the engines' `starved` flag.
- RCS ullage gas draws from the main tanks; only when they are dry from the headers [E] (gas is pressurant).
  RCS starves only when main and header tanks are dry.
- `propellant_kg()` stays the total; new `main_propellant_kg()` (the webcast gauge, main tanks only).
- The flight session selects `Header` on the ship when the ship landing guidance starts (`run_ship_landing`: the
  belly flip of every ship return - LEO catch / splashdown, aircatch b4, lunar return).
- `config::load_propellant(stage, total_kg)` and `load_propellant(stage, oxidizer_kg, fuel_kg)` replace the
  "scale every tank" code in missions and tests: headers first (to capacity), the rest into the main tanks in their
  capacity ratio; Rcs tanks untouched. Stages without headers behave as the old scaling.

### 4. Dry CG and validation
- With headers (settling off) the dry CG is re-tuned so the landing-mass CG (dry + 35 t) stays 25.50 m from the
  ship's base (today's validated entry trim): ~23.5 m [D].
- Then settling is switched on by default. Full headers at landing mass do not move, so the LEO entries, the flight
  12-14 replays and the catches keep their trim; ships with main-tank residue (lunar return 61-64 t) get the
  forward CG. If a validated case moves, the dry CG / the provenance is revisited, not the physics.

## Testing / success

- Geometry: exact cases above, volume conservation, table vs direct integration at off-grid points (tolerance
  documented in the test).
- Vehicle: aft column bit-identical with settling off; entry-like specific force moves the CG forward and towards
  the belly; lag time constant; hold below the threshold; reversal through 180 deg stays a unit vector.
- Headers: role / diameter parsing; header-fed engines draw only headers; flame-out on a dry source; select_feed
  no-op and un-starve; RCS gas fallback; derived landing propellant and mismatch error; `load_propellant` splits.
- Mission level: `[.header_tanks]` un-hidden and green; aircatch seeds 1-7 caught; campaigns direct 21/21 and
  aircatch 21/21; LEO catch campaign; flight 12-14 replays; booster / Falcon 9 returns; full suite + smoke test;
  `python tools/vehicle_report.py starship_block3`.

## Out of scope

HLS-specific changes (HLS uses the same settling but no headers), propellant transfer, slosh dynamics, boil-off,
outlet uncovering.
