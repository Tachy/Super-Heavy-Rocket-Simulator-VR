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
  e.g. a returning booster; defaults to the ascent table), `crossflow_table` (hull cross-flow coefficient over Mach for a stage flying alone, e.g. the ship's
  belly-first entry; without it a generic value), `nose_length_m`.
- `[stage.grid_fins]`: one fin per entry of `azimuths_deg` (from body +y towards +z), `area_m2`
  (frontal lattice area per fin), `station_m` (hinge), `radius_m` (fin centre of pressure),
  `max_deflection_deg`, `rate_deg_s`, `table` (normal-force slope and limit over Mach, e.g.
  `aero/grid_fin.csv`). Fin mass belongs in the stage's `dry_mass_kg`.
- `[[stage.flap]]` (ship): flat plates hinged along the hull side - `id`, `hinge_azimuth_deg` (from body +y
  towards +z), `station_fwd_m`/`station_aft_m` (from the bottom of the stage), `span_m`, `area_m2`,
  `max_deflection_deg` (0 = folded leeward, 90 = broadside to a belly-first flow), `rate_deg_s`, optional
  `initial_deflection_deg`; needs `[stage.aero] flap_table` (`aero/flap.csv`: mach, cn_max) and
  `belly_azimuth_deg` (body direction of the heat shield, default 270 = -z).
- `[stage.heat_shield]` (ship): `nose_radius_m` (Sutton-Graves), `max_heat_rate_w_m2`, `max_heat_load_j_m2`,
  `max_load_g` (aerodynamic acceleration); judged while the stage flies alone.
- `aero/ship_entry_alpha.csv`: nominal angle of attack over Mach of the belly-first entry (entry guidance profile),
  referenced by `[stage.aero] entry_alpha_table`. The ship's `ascent_drag_table` (the stack's [E]) gives its axial
  drag when it flies alone.
- Ship sea-level Raptors: three single-engine groups (`sea_level_1/2` the leeward pair at 30/150 deg, `sea_level_3`
  towards the belly at 270 deg) so the landing burn can drop the belly engine (3 -> 2); burns elsewhere run the three
  together (groups of the same engine on the same ring form one symmetric burn set).
- `[stage.recovery] landing_propellant_kg`: booster - at landing-burn ignition (MECO is planned for it); ship - entry
  mass in scenarios, the header-tank propellant for flip and landing burn (35 t [E]).
- `[stage.catch_pins]`: hardpoints the tower arms catch, one per entry of `azimuths_deg` (from body +y
  towards +z, like the grid fins), `station_m` (from the bottom of the stage), `protrusion_m` (beyond the
  hull), `max_load_n` (structural limit per pin; above it the catch is an impact).
- `[stage.recovery] method`: `"tower_catch"` (Super Heavy: always caught), `"leg_landing"` (Falcon 9: on its legs on a
  landing zone or a drone ship, chosen by the mission); empty: expendable.
- `[stage.recovery]` return keys (`gnc::booster_return_parameters`; left out: the guidance's defaults, Super Heavy's):
  `boostback_engine_groups`, `entry_engine_groups` (group ids of this stage), `return_flight_time_s` (boostback start
  to touchdown), `entry_burn_altitude_m` / `entry_burn_delta_v_mps` / `entry_burn_throttle` (entry burn lit while
  descending below that altitude, retrograde, for that delta-v), `landing_engine_phases` (3 group lists: ignition,
  divert, final set), `landing_phase_throttles` (3 design throttles that time the ignition) and
  `landing_switch_sink_mps` (2 `[min, max]` sink windows of the engine switches).
- `[stage.recovery.drone_ship]`: the return to a drone ship; its keys override the profile above (left out: taken from
  it). `boostback = false`: no flip or boostback (the stage coasts on its ascent arc). `landing_phase_max_tilt_deg`
  (3 values, either table): planned thrust tilt limit per landing engine set (default: Super Heavy's 25 / 20 / 10).
- `[stage.landing_legs]`: one leg per entry of `azimuths_deg` (at least three), deployed geometry
  (`footprint_radius_m` from the axis, `foot_station_m` below the stage bottom: negative), `deploy_time_s`, and the
  per-leg crush core (`stiffness_n_m` and `damping_n_s_m` until the force reaches `crush_force_n`, then it crushes
  plastically over at most `stroke_m`; more is an impact) and the foot's Coulomb `friction`.
- Falcon 9 octaweb: `center` (1, the landing engine), `pair` (2 opposite ring engines: with the centre the 3-engine set
  of boostback and entry burn, thrust on the axis), `ring` (the other 6).

## Launch sites (`launch_sites.toml`)
- `[[site]]`: `id`, `name`, `latitude_deg`, `longitude_deg`, `height_m` (ground above WGS84), `kind`
  (`"launch_pad"` default, `"landing_zone"`: a pad first stages land on, with `pad_radius_m`).
- `[site.sea]` (optional): straight shoreline perpendicular to `direction_deg`, `shoreline_distance_m` from the pad
  centre; beyond it the ground is water (aborted landings, leg landings: "Wasserung").
- `[site.tower]` (optional catch tower; site frame: origin at the pad centre on the ground, east/north/up):
  `azimuth_deg`/`distance_m` (pad centre -> tower centre), `width_m`, `height_m`, launch mount
  (`mount_height_m`, `mount_inner_radius_m`, `mount_outer_radius_m`), arms (`arm_length_m`, `arm_width_m`,
  `arm_thickness_m`, `arm_open_deg`, `arm_closed_gap_m`, `arm_rate_deg_s`, `arm_force_limit_n`),
  `catch_height_m` (arm tops above the ground) and the carriage shock absorbers (`carriage_mass_kg`,
  `carriage_stiffness_n_m`, `carriage_damping_n_s_m`, `carriage_stroke_m`). The hinges sit on the tower face,
  `arm_closed_gap_m / 2 + arm_width_m / 2` from the catch axis.

## Drone ships (`drone_ships.toml`)
- `[[drone_ship]]`: `id`, `name` (generic display name), `deck_length_m`, `deck_width_m`, `freeboard_m` (deck above the
  sea). Where it waits is set by the mission.

## Missions (`missions/`)
- `leo_ascent.toml`: target orbit of the game's ascent autopilot (key P).
- `ship_return_starbase.toml`: ship return from orbit (`config::load_ship_return_mission`).
- `falcon9_rtls.toml` (`config::load_ascent_mission`): `[mission]` vehicle, launch `site`, optional `landing_zone` (a
  site of kind `landing_zone`: the first stage returns to it), `heading_deg`, `payload_t`, `fairing`; `[target]` orbit;
  `[pitch_program]` `kick_deg` and `booster_throttle` (`[t_s, fraction]` pairs, increasing times).
- `falcon9_asds.toml`: the same keys; instead of a `landing_zone` a `[drone_ship]` table (`id`, `latitude_deg`,
  `longitude_deg`, `heading_deg` of the deck's long axis, `sea_height_m` above WGS84): the first stage lands on its deck.

## Adding another rocket
1. Add missing engines under `engines/`.
2. Copy a vehicle file, adjust stages/tanks/engine groups.
3. `python tools/vehicle_report.py <vehicle-id>` prints masses, burn times, T/W and the
   Tsiolkovsky delta-v budget as a sanity check.
