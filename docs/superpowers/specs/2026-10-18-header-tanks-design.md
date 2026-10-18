# Header tanks (Starship Block 3) - design

Date: 2026-10-18. Status: approved in chat by the user ("passt"), spec for review.

## Why

The aircatch seeds 4/7 depart at peak q (l6 Task 1 finding, `docs/subsystems.md`, "Aerocapture attitude at peak
q"): with 61-64 t aboard, all propellant sits in the main tanks, LOX aft, the CG is aft and the pitch trim pins both
aft flaps at the 90 deg stop. A controller margin held the attitude but cost 1-2 t of cold gas per pass and was
dropped. User decision 2026-10-18: model the header tanks ("the belly-flip burn depends on the header tanks anyway").
The landing propellant then sits forward as on the real ship, and the CG moves forward with propellant.

## What the user decided

- Feed: the ascent, deorbit and lunar-return burns draw from the main tanks; from the start of the belly flip
  (flip and landing burn) only the header tanks feed the engines. Main-tank residue stays aboard as dead mass.
- Filling: the headers are part of the 1,600 t [S]. Header 35 t [E], main tanks 1,565 t. The headers are full at
  liftoff and stay untouched until the flip.

## Assumptions ([E], sources 2020-2022, SN-era)

- LOX header in the nose tip, CH4 header a "bubble" on the lower dome of the CH4 main tank (Teslarati / SN4-SN8
  reports, see the source list in the brainstorm). Block 3 placement is not published.
- Split at the ship engines' O/F 3.6: LOX header 27.4 t, CH4 header 7.6 t.
- LOX header: base station ~44 m, diameter ~3.6 m (24 m^3 at 1,141 kg/m^3). CH4 header: base station ~20 m
  (the CH4 main tank's base), diameter ~3 m. Exact numbers are set in the plan from the stage geometry.

## Design

### 1. Configuration (`config/vehicle_config`)
- `[[stage.tank]]` gets `role = "main" | "header"` (default `main`) and an optional `diameter_m` (default: the
  stage diameter). The column height of a tank uses its own cross-section, so a small header has a realistic fill
  height. Rcs tanks keep their role-less behaviour.
- `starship_block3.toml`: `lox_main` / `ch4_main` reduced by 35 t (O/F 3.6), new `lox_header` / `ch4_header`.
- `[stage.recovery] landing_propellant_kg` is derived from the header contents when a stage has header tanks;
  giving both and differing is a config error. Stages without headers (Super Heavy, Falcon 9) keep the key.

### 2. Feed (`vehicle`)
- Each stage has a feed source `Main | Header` (default `Main`). Engines draw oxidizer and fuel only from tanks of
  that role (first tank of the kind that still holds some, as today).
- If the selected source runs dry (oxidizer or fuel), the stage's engines flame out (`starved`) - no silent
  switch to the other source.
- Selecting `Header` on a stage without header tanks is a no-op (the stage keeps `Main`).
- RCS ullage gas (`cold_gas` / `cold_gas_ullage` with `tank = -1`) draws from the main tanks in proportion to
  oxidizer and fuel; only when the main tanks are dry does it draw from the headers. Documented simplification [E]:
  the ullage gas is pressurant, not liquid, and the attitude control must not die with the last main-tank residue.
  RCS availability: starved only when main AND header tanks are dry.
- Mass, CG and inertia come from `mass_properties()` unchanged in structure (tanks are already parts with their own
  station); only the per-tank cross-section changes.
- `propellant_kg()` stays the total (budgets, planners, HUD). New `main_propellant_kg()`: the webcast gauge (main
  tanks only), used by the flight 12-14 telemetry replays.
- `split_off` / separation copies the feed source with the stages.

### 3. Feed switch (`sim` / `gnc`)
- The ship return selects `Header` at the start of the belly flip, for every path that ends in b3's flip:
  LEO ship return (catch and splashdown), aircatch b4, lunar return direct. The plan locates the single hand-over
  point (flip start in the ship landing controller or the session step that starts it).
- The deorbit and lunar-return planners already reserve `landing_propellant_kg`; with the derived value the
  reserve equals the header contents, and `vehicle.propellant_kg() - landing_propellant_kg` becomes
  `main_propellant_kg()` where it means "propellant available before the flip" (the plan lists each call site).

### 4. Mission starts
- `lunar_return_mission.cpp`'s `scale_tanks` becomes a `config` helper `load_propellant(stage, total_kg)`: headers
  filled first (in their O/F split), the rest into the main tanks in their ratio; a total below the header capacity
  goes into the headers only. Every scenario that sets a ship's propellant (entry tests "dry + landing propellant",
  catch tests, lunar/aircatch missions) uses it.

### 5. Dry CG re-tune
- `dry_cg_station_m` is chosen so that the CG at landing mass (dry + 35 t, now in the headers) equals today's CG at
  landing mass (dry 28 m + 35 t in the main tanks). Estimate ~23.4 m [D]. Entry trim, the flight 12-14 replays and
  the LEO catches at landing mass stay as validated; heavier ships (lunar return 61-64 t) are far less aft-heavy.
- The provenance comment records the derivation; `docs/subsystems.md` (ship dry CG, entry mass in tests,
  aerocapture finding) is updated.

## Testing / success

- Unit tests: tank role and diameter parsing; header-fed engines draw only headers; flame-out when the selected
  source is dry; RCS gas falls back to the headers; CG moves forward with header propellant; `load_propellant`
  splits; derived `landing_propellant_kg` and the mismatch error.
- `[.header_tanks]` ("An aerocapture pass with 64 t aboard damps a roll disturbance at peak q") is un-hidden and
  green.
- Aircatch seeds 1-7 caught (7/7), campaigns direct 21/21 and aircatch 21/21, LEO catch campaign, flight 12-14
  telemetry replays, ship entry/catch tests, lunar return sessions, NRHO flight: all green. Full suite + smoke test.
- `python tools/vehicle_report.py starship_block3` after the config change.

## Out of scope

HLS (lands on its main tanks), Super Heavy, propellant transfer between main and header, sloshing, header boil-off.
