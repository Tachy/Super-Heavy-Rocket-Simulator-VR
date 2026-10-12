# Configuration

All vehicles are data, not code. Three layers reference each other by `id`:

```
propellants.toml        propellant combinations (densities, nominal mixture ratio)
engines/<id>.toml       one engine type (performance, throttle, gimbal, mass, timing)
vehicles/<id>.toml      one vehicle version: stages, tanks, engine groups, aero, separation
aero/<id>.csv           aerodynamic coefficient tables referenced by vehicles
```

## Conventions
- SI units; the unit is part of the key (`thrust_ref_n`, `dry_mass_kg`, `max_angle_deg`).
  Degrees are allowed in config for readability (`_deg` suffix); the loader converts to radians.
- Stations (`*_station_m`) are measured along the vehicle axis **from the bottom of the stage**
  (engine gimbal plane = 0 unless stated otherwise).
- Every value carries a provenance tag in its comment:
  - `[S:<ref>]` sourced value, `<ref>` points to the `[sources]` table at the top of the file
  - `[E]` engineering estimate (no public data); calibrate against flight telemetry
  - `[D]` derived from other values in the file (the comment shows how)

## Engine thrust model
Each engine is defined at one reference ambient pressure (sea level or vacuum):

```
mdot        = thrust_ref_n / (isp_ref_s * g0)                 (constant at full throttle)
F(p_amb)    = thrust_ref_n - (p_amb - reference_pressure_pa) * A_exit
Isp(p_amb)  = F(p_amb) / (mdot * g0)
```
`A_exit` comes from `nozzle_exit_diameter_m`. Sea-level and vacuum Isp therefore stay
mutually consistent. `max_ambient_pressure_pa` limits where the nozzle may run without
flow separation (vacuum engines).

## Aerodynamics and control surfaces (per stage)
- `[stage.aero]`: `ascent_drag_table` (nose first, whole stack), `descent_drag_table` (tail first,
  e.g. a returning booster; defaults to the ascent table), `entry_table`, `nose_length_m`.
- `[stage.grid_fins]`: one fin per entry of `azimuths_deg` (from body +y towards +z), `area_m2`
  (frontal lattice area per fin), `station_m` (hinge), `radius_m` (fin centre of pressure),
  `max_deflection_deg`, `rate_deg_s`, `table` (normal-force slope and limit over Mach, e.g.
  `aero/grid_fin.csv`). Fin mass belongs in the stage's `dry_mass_kg`.

## Adding another rocket
1. Add missing engines under `engines/`.
2. Copy a vehicle file, adjust stages/tanks/engine groups.
3. `python tools/vehicle_report.py <vehicle-id>` prints masses, burn times, T/W and the
   Tsiolkovsky delta-v budget as a sanity check.
