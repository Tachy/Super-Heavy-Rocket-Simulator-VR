# Configuration

All vehicles are data, not code. Three layers reference each other by `id`:

```
propellants.toml        propellant combinations (densities, nominal mixture ratio; optional boil-off data
                        `<oxidizer|fuel>_boiling_point_k`, `_latent_heat_j_kg`, `_surface_tension_n_m`, all or
                        none - absent: storable)
engines/<id>.toml       one engine type (performance, throttle, gimbal, mass, timing)
vehicles/<id>.toml      one vehicle version: stages, tanks, engine groups, aero, staging
aero/<id>.csv           aerodynamic coefficient tables referenced by vehicles
crew_escape/<id>.toml   ejection seats of a crewed ship return (referenced by missions; ship data untouched)
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
- Display names (shown in the game, English for now - the UI language is decided later, no trademarks; the `name` fields keep the real names and are never shown
  in the webcast overlay): `[vehicle] display_name` for the whole vehicle; `[[stage]] display_name` (optional) for a
  stage that flies as a separate body (booster, ship, capsule, trunk). A part split off at staging is shown by its
  lowest stage with main engines, else its topmost named stage (`vehicle::Vehicle::shown_name`).
  `[mission] display_name` (optional) names the mission under the overlay's mission clock.
  Start declarations (mission selection panel): `[mission] start_place` (a pad id of `launch_sites.toml` or an id of
  `start_places.toml`: `earth_orbit`, `iss_docked`, `lunar_orbit`) and `start_vehicle` (a `vehicles/` file id) on every
  selectable mission; missions flown only as a follow-on (ship returns, `leo_ascent`) have neither. `[[site]] vehicles =
  [...]` (one line) lists what may start on a pad (sites without `kind`; landing zones have none).
  `core/tests/test_mission_start_places.cpp` checks them against the start dispatch.

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
- `[stage.heat_shield]` (ship, capsule): `nose_radius_m` (Sutton-Graves), `max_heat_rate_w_m2`, `max_heat_load_j_m2`,
  `max_load_g` (aerodynamic acceleration, breakup), optional `crew_load_limit_g` (warning, and the entry guidance's load
  limit when lower); judged while the stage flies alone.
- `[stage.thermal]` (refuelling r2, boil-off): `absorptivity` (solar) and `emissivity` (infrared), both in (0, 1],
  `wall_resistance_m2k_w` > 0 (effective skin -> propellant: insulation, wall, liquid-side film),
  `ullage_resistance_m2k_w` > 0 (added on the dry wall: ullage gas conduction near free fall). Optional
  `[stage.thermal.shield]` with the same keys plus `half_width_deg`: the heat-shield tiles, a zone of
  +-`half_width_deg` around `[stage.aero] belly_azimuth_deg`. Without `[stage.thermal]` the stage's tanks do not boil
  off. A `from` stage inherits the source's block.
- Capsule (Crew Dragon, d3): `[stage.aero] capsule_table` (`aero/crew_dragon_aero.csv`: mach, alpha_deg = angle between
  the air velocity and -x, ca, cn, cm about the heat-shield apex at station 0; generated by
  `tools/aero/gen_capsule_aero.py`, never edited by hand) is flown once the stage is the lowest (the trunk jettisoned);
  `entry_profile = "capsule"` (default `"ship"`) selects the entry guidance's phases; `[[stage]] dry_cg_offset_m = [y, z]`
  (lateral dry CG offset that trims the capsule at an angle of attack: `gen_capsule_aero.py --trim` prints it);
  `[stage.recovery] method = "splashdown"` with `max_splashdown_speed_mps` (required there: sink at the water up to which
  the capsule is "gewassert"; faster is an impact).
- Parachutes (d4): `[[stage.parachute]]` sets: `id`, `role` (`"drogue"` / `"main"`, at most one set each), `count`,
  `drag_area_m2` (C_D S0 of one canopy, fully open), `attach_station_m` (riser point on the axis, from the stage's
  bottom), `fill_time_s` (each reefing stage and the final opening fill with the area growing as t^2), `reefing`
  (`[[area_fraction, hold_s], ...]`, ascending fractions in (0, 1); empty: opens to full), `max_load_n` (per canopy:
  the set tears beyond it), `deploy_height_m` (mains below drogues), optional `max_deploy_dynamic_pressure_pa` (the
  deployment waits above it). Only the lowest stage's sets are sequenced (a capsule flying alone).
- `aero/ship_entry_alpha.csv`: nominal angle of attack over Mach of the belly-first entry (entry guidance profile),
  referenced by `[stage.aero] entry_alpha_table`. The ship's `ascent_drag_table` (the stack's [E]) gives its axial
  drag when it flies alone.
- `aero/radiative_heating_earth.csv` (lunar l5): equilibrium stagnation-point radiative heating for Earth air
  [S: Sutton & Hartung, NASA TM-102652, 1990; 624 rows, 30-84 km, 8-18 km/s, nose radius 0.05-10 m], interpolated by
  free-stream density, velocity and nose radius; the heat rate of a ship / capsule = convective (Sutton-Graves) +
  radiative from 8 km/s up (the table's validity: equilibrium air only, no non-equilibrium or ablation coupling).
- Ship sea-level Raptors: three single-engine groups (`sea_level_1/2` the leeward pair at 30/150 deg, `sea_level_3`
  towards the belly at 270 deg) so the landing burn can drop the belly engine (3 -> 2); burns elsewhere run the three
  together (groups of the same engine on the same ring form one symmetric burn set).
- `[[stage.tank]]` `role = "header"` (default `"main"`; oxidizer or fuel only) and optional `diameter_m` (default: the
  stage diameter): Starship's header tanks hold the landing propellant and feed the engines from the belly flip on
  (the session selects them); the main tanks feed everything before. The configured loads are full tanks (capacities);
  scenarios load propellant with `config::fill_tanks` (headers first). The gauge (`main_propellant_kg`) shows the main
  tanks only. Tank liquid settles perpendicular to the apparent gravity (`vehicle/liquid_settling`).
- `[stage.recovery] landing_propellant_kg`: booster - at landing-burn ignition (MECO is planned for it); ship - derived
  from the header tanks' contents (35 t [E]; giving a different value is an error). `rcs_propellant_kg`: main-tank RCS
  gas a returning booster spends from MECO to landing-burn ignition (the MECO plan adds it; 0 with an own RCS supply).
- `[stage.rcs]`: reaction control as individual thrusters, on/off with a fixed thrust (constant feed pressure), controlled
  by time: `isp_s`, `propellant` (`"main"`: ullage gas from the stage's oxidizer and fuel tanks, drawn in proportion to
  what is left; or the id of a `[[stage.tank]]` with `contents = "rcs"`, which needs `density_kg_m3` and is not part of
  the propellant gauge), `minimum_on_time_s` (shortest pulse; the minimum impulse bit is thrust x this), `type`
  (documentation). Thrusters come from `[[stage.rcs.pod]]` - `id`, `station_m`, `radius_m`, `azimuths_deg` (from body
  +y towards +z), `thrust_n`, `directions` (at each azimuth one thruster per entry: `tangential_plus`,
  `tangential_minus`, `inward` (force towards the axis, plume outwards), `forward` (+x, plume aft), `aft`) - and/or
  single `[[stage.rcs.thruster]]` - `id`, `position_m` [station, y, z], `direction` (force on the vehicle), `thrust_n`.
  An RCS tank's mass is part of the stage's real dry mass: carve it out of `dry_mass_kg`. `behind_nose_cone = true` (pod
  or thruster): fires only while the stage's nose cone is open (`[[stage]] nose_cone = true`, Crew Dragon).
- `[[stage.engine_group]] abort_only = true`: launch escape engines (SuperDraco), loaded as data and never lit by the
  session (milestone d2). `landing_only = true`: landing thrusters (HLS), never in burn configurations or `ignite()`,
  flown by landing guidance only (lunar l4). `station_m`: thrust point above the stage bottom (default 0, the base);
  `cant_deg`: thrust tilted towards the axis, the exhaust away from the hull (a canted group cannot gimbal).
- `[stage.catch_pins]`: hardpoints the tower arms catch, one per entry of `azimuths_deg` (from body +y
  towards +z, like the grid fins), `station_m` (from the bottom of the stage), `protrusion_m` (beyond the
  hull), `max_load_n` (structural limit per pin; above it the catch is an impact).
- `[stage.recovery] method`: `"tower_catch"` (Super Heavy: always caught), `"leg_landing"` (Falcon 9: on its legs on a
  landing zone or a drone ship, chosen by the mission); empty: expendable.
- `[stage.recovery]` return keys (`gnc::booster_return_parameters`; left out: the guidance's defaults, Super Heavy's):
  `boostback_engine_groups`, `entry_engine_groups` (group ids of this stage), `return_flight_time_s` (boostback start
  to touchdown), `entry_burn_altitude_m` / `entry_burn_delta_v_mps` / `entry_burn_throttle` (entry burn lit while
  descending below that altitude, retrograde, for that delta-v), `landing_engine_phases` (3 group lists: ignition,
  divert, final set), `landing_phase_throttles` (3 design throttles that time the ignition),
  `landing_switch_sink_mps` (2 `[min, max]` sink windows of the engine switches) and `flip_engine_groups` (the flip
  after separation at minimum throttle; left out: the final landing set; `[]`: engines off, the RCS turns the stage).
- `[[stage]] max_dynamic_pressure_pa` (a first stage; left out: no limit): the ascent guidance throttles the stack to
  hold the dynamic pressure at or below it (max-Q limit). One number, or `[[mach, pa], ...]` (increasing Mach, linear
  between, held beyond the ends) for a limit over Mach (Falcon 9: the transonic hold). [D] from the flights' q and
  throttled phases, `tools/telemetry/peak_dynamic_pressure.py`. `max_ascent_throttle` (a first stage, (0, 1], default 1): the operational
  throttle the ascent never exceeds ([D] from the propellant gauge's flow, `tools/telemetry/mass_estimate.py`); the
  engines' rating stays their capability for every other burn. The q limit acts below it.
- `[stage.staging]` (a lower stage; left out: `stage()` separates at once): the staging sequence to the stage above.
  `kind = "cold"`: MECO cuts every engine (the tail-off acts on the joined stack), `separation_delay_s` later the
  separation with `separation_dv_mps` (pushers: relative speed of the parts), `upper_ignition_delay_s` after it the
  upper stage lights. `kind = "hot"`: MECO keeps `run_on_engine_groups` lit at minimum throttle, the stage above lights
  while joined and separates (no push) on the tick its engines are at thrust; delays and push are not allowed.
  Stages taken with `from` keep the source's sequence.
- `[stage.recovery.drone_ship]`: the return to a drone ship; its keys override the profile above (left out: taken from
  it). `boostback = false`: no flip or boostback (the stage coasts on its ascent arc). `landing_phase_max_tilt_deg`
  (3 values, either table): planned thrust tilt limit per landing engine set (default: Super Heavy's 25 / 20 / 10).
- `[stage.landing_legs]`: one leg per entry of `azimuths_deg` (at least three), deployed geometry
  (`footprint_radius_m` from the axis, `foot_station_m` below the stage bottom: negative), `deploy_time_s`, and the
  per-leg crush core (`stiffness_n_m` and `damping_n_s_m` until the force reaches `crush_force_n`, then it crushes
  plastically over at most `stroke_m`; more is an impact) and the foot's Coulomb `friction`. `fixed = true`: always
  deployed (HLS; `deploy_time_s` not needed).
- Falcon 9 octaweb: `center` (1, the landing engine), `pair` (2 opposite ring engines: with the centre the 3-engine set
  of boostback and entry burn, thrust on the axis), `ring` (the other 6).

## Space stations (ISS, milestone i)
- `[[stage]] inertia_kg_m2 = [Ixx, Iyy, Izz]`: principal moments of the dry stage about its own CG (any stage;
  without it a cylinder from length and diameter).
- `[stage.aero] fixed_drag_area_m2` + `fixed_drag_coefficient`: drag C_D A along the oncoming flow, replacing the
  slender-body aero (a station); `solar_area_m2`: fixed area for solar radiation pressure (else the cylinder model).
- `[stage.station]`: marks the stage as a station flown in ideal LVLH attitude (+XVV, x forward, y starboard,
  z nadir); `body_in_lvlh_deg = [yaw, pitch, roll]` (torque equilibrium attitude).
- `[stage.station.proximity]` (refuelling r3, all optional): the target's approach limits over the code's ISS
  defaults - `keep_out_radius_m`, `approach_ellipsoid_m = [x, y, z]` (LVLH semi-axes), `waypoint0_below_m`,
  `waypoint1_range_m`, `waypoint2_range_m` (below Waypoint 1), `flyaround_radius_m`. A station is one stage (the
  depot is added without its booster).
- `[stage.station.reference_orbit]`: the synthetic orbit without any OEM - `altitude_km` (a minus the equatorial
  radius), `inclination_deg`, `raan_deg` (both in the true equator frame of date), `reference_utc`.
- `[[stage.docking_port]]`: `id`, `name` (shown; English for now), `position_m` (stage frame: x station from the stage bottom,
  y/z body axes), `axis` (outward unit vector), `up` (clocking reference perpendicular to it), optional
  `behind_nose_cone` (default false: the port is covered while the vehicle's nose cone is closed - Crew Dragon `idss`),
  optional `soft_capture = "active" | "passive"` (default passive; IDSS: the active side extends its soft-capture ring,
  the passive one stays retracted - Crew Dragon active, the ISS IDAs passive; a docking needs one of each),
  optional `mechanism = "idss" | "probe_drogue"` (default idss) with `elements_m = [[x, y, z], ...]` (probe_drogue
  only, at least 3: the probe tips on the active port, the drogue mouths on the passive one, stage frame - the
  Starship tanker / HLS against the depot, refuelling r1). Two ports mate only with equal mechanisms; probes belong to
  the chaser.
- `[[stage.transfer_connector]]` (refuelling r1): `id`, `tank` (a main propellant tank of the stage), `position_m`
  (the coupling face's centre, stage frame). Docked, connectors of the same contents meet. r4: `line_diameter_m`,
  `discharge_coefficient` (both > 0; the coupled pair's smaller C_d A governs `mdot = C_d A sqrt(2 rho dp)`) and
  `chilldown_kg` (>= 0, vented to chill the line before each transfer) are required.
- `[stage.propellant_transfer]` (refuelling r4): `delivery_pressure_pa` (the delivering ship holds its ullage there),
  `receive_pressure_pa` (the receiving ship vents down to it; must be below delivery). Without it the ship cannot
  transfer propellant.

## Crew escape (`crew_escape/`)
- `[crew_escape]`: `seat_vehicle` (seat + occupant body until the seat-man separation) and `parachutist_vehicle`
  (occupant under the main canopy), both in `vehicles/`; `rail_direction` (ship body axes, normalised),
  `rail_length_m`, `stagger_s`; envelope `envelope_max_height_m`, `envelope_max_eas_mps`; modes `mode1_max_eas_mps`,
  `mode_low_height_m`, `mode1_main_delay_s`, `mode2_drogue_delay_s`, `mode2_separation_delay_s`; give-up
  `min_firing_height_m`, `sea_target_margin_m`, `min_catch_propellant_kg`; judging `max_load_g`, `max_load_duration_s`,
  `min_clearance_m`, `plume_half_angle_deg`, `plume_length_m`, `max_touchdown_sink_mps`; thrust tables `catapult`,
  `rocket` (`[[time_s, thrust_n]]`, ascending times, along the rail, from the seat's firing).
- `[[seat]] position_m`: one per crew member in the firing order, in the ship's stage frame (x from the stage bottom).

## Reused stages
- `[[stage]] from = "<vehicle>:<stage>"`: the stage exactly as `config/vehicles/<vehicle>.toml` configures it
  (the Starship tanker and the depot's Super Heavy are Block 3's). Only `[[stage.docking_port]]`,
  `[[stage.transfer_connector]]`, `[[stage.rcs_pod]]` (r3: same keys as `[[stage.rcs.pod]]`; the source stage needs
  `[stage.rcs]`) and `[stage.propellant_transfer]` (r4) may be added, `dry_mass_kg` and `display_name` overridden; any other key is an error (it would silently diverge from the source).
  The propellant must be the vehicle's. `tools/vehicle_report.py` follows `from`.
- `[stage] hull_top_diameter_m` (optional): the hull tapers linearly from `diameter_m` at the stage bottom to this at
  its top - the hull sample points of a visiting vehicle against the station primitives (ISS i5; Crew Dragon capsule).
- `[[stage.collision]]`: `id`, `kind = "capsule"` (segment `a_m`-`b_m`, `radius_m`) or `"box"` (`centre_m`,
  body-aligned `half_extents_m`), stage frame (the ISS modules; the depot's tank section and nose, refuelling r3).
  A visiting vehicle's hull is sampled up to a nose docking port (a dorsal port does not cut it) plus its landing
  legs along the struts.
- With an OEM, mass, drag area and C_D come from its header instead of the config.

## Lunar landing sites (`lunar_sites.toml`)
- `[[site]]`: `id`, `name`, `latitude_deg` / `longitude_deg` (MOON_ME, the frame of the LOLA maps), `tile_half_size_m`
  (half the side of the terrain tile), `product` (LOLA product the tile is cropped from: `"LDEM_80S_20M"`).
  `python tools/data_pipeline/fetch.py` writes the tile to `data/terrain/<id>.toml` / `.i16` (and the global grid
  `ldem_16`); `sim::load_lunar_terrain` reads them.

## Launch sites (`launch_sites.toml`)
- `[[site]]`: `id`, `name`, `latitude_deg`, `longitude_deg`, `height_m` (ground above the WGS84 ellipsoid, [D]:
  the median of the 1 m local terrain grid within `plateau_radius_m`, written by
  `python tools/data_pipeline/site_heights.py --write`; rerun it after a terrain or radius change and re-measure the
  reference flights - `refuelling/reference_flights.toml` digests this file), `kind`
  (`"launch_pad"` default, `"landing_zone"`: a pad first stages land on, with `pad_radius_m`), and (required) the
  site's plateau in the Earth terrain: flat at `height_m` within `plateau_radius_m`, a linear apron of
  `plateau_apron_m` into the terrain beyond it, never water (`sim::load_earth_terrain`).
- `[site.viewpoint]` (optional, game only): the time panel's ground view "<site> (Aussicht)" stands here instead of
  on the pad: `name` (German display name), `latitude_deg`, `longitude_deg`, `eye_height_m` (above the terrain at the
  viewpoint: roof or deck height + ~1.7 m eye); the view starts aimed at the pad (`World._aim_ground_camera_at`).
  Launches and physics keep the pad coordinates; the core ignores this table.
- `[site.sea]` (optional): straight shoreline perpendicular to `direction_deg`, `shoreline_distance_m` from the pad
  centre; beyond it the ground is water (aborted landings, leg landings: "Wasserung").
- `[site.tower]` (optional catch tower; site frame: origin at the pad centre on the ground, east/north/up):
  `azimuth_deg`/`distance_m` (pad centre -> tower centre), `width_m`, `height_m`, launch mount
  (`mount_height_m`, `mount_inner_radius_m`, `mount_outer_radius_m`), arms (`arm_length_m`, `arm_width_m`,
  `arm_thickness_m`, `arm_open_deg`, `arm_closed_gap_m`, `arm_rate_deg_s`, `arm_force_limit_n`),
  `catch_height_m` (arm tops above the ground) and the carriage shock absorbers (`carriage_mass_kg`,
  `carriage_stiffness_n_m`, `carriage_damping_n_s_m`, `carriage_stroke_m`). The hinges sit on the tower face,
  `arm_closed_gap_m / 2 + arm_width_m / 2` from the catch axis.

## Splashdown zones (`splashdown_zones.toml`)
- `[[zone]]`: `id`, `name` (shown; English for now), `latitude_deg`, `longitude_deg`, `radius_m`. A capsule's entry guidance aims its
  drogue gate over the centre at sea level (`FlightSession::set_splashdown_zone`).

## Drone ships (`drone_ships.toml`)
- `[[drone_ship]]`: `id`, `name` (generic display name), `deck_length_m`, `deck_width_m`, `freeboard_m` (deck above the
  sea), `basin` (`"atlantic"` / `"pacific"`), `home_port_latitude_deg` / `home_port_longitude_deg`, and the service area
  `area_min_latitude_deg`, `area_max_latitude_deg`, `area_min_longitude_deg`, `area_max_longitude_deg` (a lat / lon box;
  min < max, so an area cannot cross the antimeridian). Where it waits is set by the mission.
- Hull and landing limits (deck motion, `contact::DeckMotion`; the box barge under the deck): `hull_length_m`,
  `hull_beam_m` (waterplane, without the wing extensions), `hull_depth_m` (draft = depth - freeboard [D]),
  `hull_roll_gyration_m`, `hull_pitch_gyration_m`, `hull_heave_added_mass_factor`, `hull_roll_added_inertia_factor`,
  `hull_pitch_added_inertia_factor` (per displaced mass / inertia), `hull_heave_damping_ratio`, `hull_roll_damping_ratio`,
  `hull_pitch_damping_ratio` (of critical), `max_deck_tilt_deg`, `max_heave_rate_mps` (significant values allowed for a
  landing; above them the onboard computer refuses the drone ship, `sim::sea_state_refusal`). Derivations in the file
  header and `docs/subsystems.md` "Sea state and deck motion".
- Selection (`config::` drone-ship choice): the ship whose area contains the predicted impact; several: the one with the
  nearest home port; on a tie the first in file order. `asds_atlantic_2` is identical to `asds_atlantic`, so it is never
  chosen.

## Webcast gauges (`ui/webcast_gauges.toml`)
- Which round gauges the webcast overlay shows, their order, labels, units, full scales and look, plus `[rules]`
  (`hide_delay_s`, `max_per_side`) and `[thresholds]` for the named conditions. Situations are chosen in code
  (`game/ui/webcast/gauge_rules.gd`); read by `gauge_config.gd`. Errors in the file are collected, bad entries dropped.

## Missions (`missions/`)
- `[mission] kind = "ascent" | "ship_return" | "capsule_return" | "lunar_landing" | "lunar_return" | "iss_rendezvous" |
  "iss_launch" | "iss_departure" | "depot_launch" | "tanker_flight" | "hls_refuel"` (default "ascent"): `config::mission_kind`;
  `sim::start_mission` / `FlightSim.start_mission` dispatch on it (an ascent starts at the requested time, a capsule return
  at its `[orbit]` epoch with the deorbit planned, a ship return is refused: it is flown from orbit, key R). Each loader
  refuses a file of another kind.
- `leo_ascent.toml`: target orbit of the game's ascent autopilot (key P).
- `ship_return_starbase.toml`: ship return from orbit (`config::load_ship_return_mission`).
- `crew_dragon_return.toml`: capsule return from orbit (`config::load_capsule_return_mission`,
  `sim::place_capsule_return_mission`): `[mission]` `vehicle`, `zone` (`splashdown_zones.toml`); `[orbit]` epoch, heights,
  inclination, RAAN, argument of latitude, `rcs_propellant_kg`; `[deorbit]` as the ship's.
- `falcon9_rtls.toml` (`config::load_ascent_mission`): `[mission]` vehicle, launch `site`, `heading_deg`, `payload_t`,
  `fairing`; `[target]` orbit. No profile and no recovery: the onboard computer plans them at the start
  (`sim::plan_launch`: the pitch kick by trial flights, the tower catch, the site's landing zone (RTLS) or a drone ship at
  the stage's predicted impact; the first stage's throttle comes from its `max_ascent_throttle` and
  `max_dynamic_pressure_pa`). The loader refuses `[pitch_program]`, `kick_deg`, `booster_throttle`, `landing_zone` and
  `[drone_ship]`.
- `[mission] liftoff_utc` (optional, ascent; ISO UTC like `epoch_utc`): a historical liftoff (the validation flights
  `flight14_ascent`, `crs35_ascent`, dates from `tests/validation/telemetry/flights.toml`). The mission lifts off at that
  date whatever epoch is requested, and with `wait_on_pad` it is placed `countdown_s` before it (`MissionStart::epoch`).
  None: lifts off at the requested time (+ countdown).
- `falcon9_crew_dragon.toml`: the same keys with a `[payload]` table instead of `payload_t` (exclusive): `vehicle` (a
  vehicle of `vehicles/`, stacked on the launcher by `config::stack_payload`), `separation_delay_s` (after the upper
  stage's cutoff), `separation_dv_mps` (spring push), `nose_cone_open_s` (after the separation; < 0: stays closed).
- `falcon9_asds.toml`: the same keys; the Starlink-class payload leaves the RTLS return short of the orbit, so the
  computer sends the drone ship (`drone_ships.toml`) to the stage's predicted impact and the stage lands on its deck.
- `[sea_state]` (optional, any mission; `config::mission_sea_state`, read by every `sim::start_mission`): the sea at the
  drone ship's station - `significant_wave_height_m` (Hs, 0: calm), `peak_period_s` (Tp, JONSWAP gamma 3.3),
  `heading_deg` (direction the waves travel towards, clockwise from north), `seed` (wave phases; deterministic). None:
  calm (the level fixed deck). `falcon9_asds_sea.toml`: `falcon9_asds.toml` in Hs 2.5 m / Tp 8 s.
- `hls_lunar_llo.toml`, `hls_lunar_nrho.toml` (`kind = "lunar_landing"`, `config::load_lunar_landing_mission`,
  `sim::place_lunar_landing_mission`): `[mission]` `vehicle`, `site`
  (`lunar_sites.toml`, must exist); `[orbit]` scenario start (`epoch_utc`, `height_km`, `inclination_deg`,
  `argument_of_latitude_deg`; the RAAN is computed so the plane passes through the Moon at arrival); `[transfer]`
  `flight_time_days` (from the scenario start to the arrival), `earliest_departure_h` (0 = as early as possible; the
  session never plans the TLI centre earlier than 30 min after the start), `mcc_after_tli_days`;
  `[lunar_orbit]` `kind = "llo" | "nrho"`, for an LLO `altitude_km`, `inclination_deg`; for an NRHO an `[nrho]` table
  with the CR3BP seed at apolune (`apolune_position`, `apolune_velocity`, nondimensional), `revolutions` /
  `synodic_months` (resonance) and `reference_revolutions` (span of the full-model reference). The session needs a Moon
  orientation (the game's FlightSim has none yet - lunar l6).
  Since lunar l4 the mission flies on to the surface and back to the LLO (`sim::start_lunar_landing_mission`):
  `[descent]` `pdi_altitude_above_site_km` (periselene after the DOI), `braking_range_km` (fallback; the planner sizes
  the range from the mass), `max_crossrange_km` (the LLO waits for a site pass within this; a plane trim removes the
  rest), `slope_limit_deg`, `touchdown_search_radius_m` (admissible point nearest to the site); `[surface]`
  `stay_hours`; `[ascent]` `insertion_altitude_above_site_km` (PEG cutoff), `aposelene_km` (circularised there);
  `[budget]` delta-v of the legs not planned at the start (`descent_delta_v_mps`, `ascent_delta_v_mps`, for the NRHO
  variant `nrho_to_llo_delta_v_mps` / `llo_to_nrho_delta_v_mps`) and `reserve_kg`: the start refuses a mission whose
  budget does not close ("Mission nicht durchführbar: ... fehlen N t").
- `ship_lunar_return_llo.toml`, `ship_lunar_return_nrho.toml`, `ship_lunar_aircatch_llo.toml` (`kind = "lunar_return"`,
  `config::load_lunar_return_mission`, `sim::start_lunar_return_mission`; lunar l5): a ship (`[mission]` `vehicle`,
  `stage`, `site` = the Starbase tower) returns from lunar orbit. `[orbit]` `epoch_utc`, `kind = "llo" | "nrho"`; LLO
  `altitude_km`, `inclination_deg` (Moon equator; any value at or above the return asymptote's declination: the start
  places the plane through the asymptote, below it the start refuses); NRHO an `[nrho]` table as the lunar landing's
  (the reference's first perilune two days after the start). `[return]`
  `variant = "direct" | "aerocapture"`, `flight_time_days` (first guess), `search_days` (departures scanned),
  `mcc_after_tei_days`, `entry_interface_km`, `braking_height_km` (the vacuum Raptors brake there to the entry speed),
  `min_corridor_width_deg` (the entry speed is the highest whose corridor is this wide; the planned angle is its
  middle), `nominal_bank_deg` (gate targeting); aerocapture also `aerocapture_apoapsis_km`, `perigee_raise_km` and a
  `[deorbit]` table as `ship_return_starbase.toml`'s (b4 after the capture). `[budget]` `mcc_allowance_mps`,
  `coast_attitude_kg_per_day`, `reserve_kg`. The start computes the propellant (fixed point over the planned burns down
  to dry + landing propellant) and refuses without an entry corridor ("Kein Eintrittskorridor ...") or when it exceeds
  the tanks. A direct entry reaches Starbase only at epochs with the Moon south at the departure (l5 Task 6).

- `crew_dragon_iss.toml` (`kind = "iss_rendezvous"`, `config::load_iss_rendezvous_mission`,
  `sim::start_iss_rendezvous_mission`; ISS i6): `[mission]` `vehicle`, `station` (vehicle id), `port` (`ida2` / `ida3`),
  `iss_orbit = "oem" | "synthetic" | "auto"`; `[start]` `epoch_utc`, `launch` (an `iss_launch` mission: the start is
  the predicted separation of its first window after epoch_utc), optional `phase_deg` (instead: that far behind the
  ISS at epoch_utc) and `iss_argument_of_latitude_deg` (synthetic orbit); `[phasing]` `mission_limit_h`,
  `earliest_arrival_h`.
- `falcon9_crew_dragon_iss.toml` (`kind = "iss_launch"`, `config::load_iss_launch_mission`,
  `sim::start_iss_launch_mission`): `[mission]` `ascent` (an ascent mission with the Dragon as payload), `station`,
  `port`, `iss_orbit`; `[launch_window]` `earliest_utc`, `search_days`, `insertion_s` and `insertion_arc_deg` ([D]
  from the flown ascent: liftoff -> separation and the inertial in-plane arc from the pad); `[phasing]` as above,
  counted from the liftoff.
- `crew_dragon_iss_departure.toml` (`kind = "iss_departure"`, `config::load_iss_departure_mission`,
  `sim::start_iss_departure_mission`): `[mission]` `vehicle`, `station`, `port`, `iss_orbit`, `return` (a
  `capsule_return` mission: zone and deorbit settings); `[start]` `epoch_utc` (earliest undocking),
  `rcs_propellant_kg`, `undock_to_deorbit_h`, `search_window_h`, optional `iss_argument_of_latitude_deg`.
- Refuelling reference flights (r5, `config/refuelling_mission_config.hpp`, `sim::refuelling_mission`):
  - `depot_launch.toml` (`kind = "depot_launch"`, `config::load_depot_launch_mission`): `[mission]` `ascent` (the
    depot stack's ascent mission, `depot_ascent.toml`); `[start]` `epoch_utc`. In orbit the depot becomes the station.
  - `tanker_flight.toml`, `tanker_flight_orbit.toml` (`kind = "tanker_flight"`) and `hls_refuel*.toml`
    (`kind = "hls_refuel"`), `config::load_refuelling_flight_mission`: `[mission]` `ascent` (the chaser's ascent
    mission: vehicle, site, phasing orbit), `port` (default `transfer_dock`), `return` (tanker: a `ship_return`
    mission whose `[deorbit]` block is flown); `[depot]` `vehicle`, `port`, `height_km`, `inclination_deg`,
    `raan_deg`, `argument_of_latitude_deg` (at earliest_utc; hls_refuel: the RAAN comes from the lunar plane),
    `propellant_t`; `[start]` `mode = "pad" | "phasing_orbit"` (phasing orbit: `behind_deg`, `propellant_t` main
    tanks); `[launch_window]` as `iss_launch` (insertion = liftoff -> orbit insertion); `[phasing]` as above;
    hls_refuel: `[lunar]` `mission` (the `lunar_landing` mission continued after the refuelling).
- `refuelling/reference_flights.toml` (refuelling r6, `config::load_refuelling_reference`): the measured numbers of
  the r5 pad flights [D] that the campaign calculator (`sim::plan_refuelling_campaign`, `refuelling_report`) starts
  from - `[depot]` `mission`, `initial_propellant_t`; `[tanker]` `mission`, `dry_mass_t` (the configured tanker),
  `residue_after_ascent_t`, `rendezvous_t`, `delivered_oxidizer_t`, `delivered_fuel_t`, `chilldown_t`,
  `settling_gas_t`, `main_at_undock_t`, `main_at_catch_t`, `dry_mass_sensitivity` (residue per kg of dry mass);
  `[hls]` `mission`, `main_at_docking_t`. The slow r5 campaigns check them (1 % or 0.3 t): re-measure after any change
  to the vehicles, ascents, phasing or transfer.

- Payload vehicles: `config::stack_payload` stacks a vehicle (e.g. `crew_dragon`) on the launcher; its stages are
  `payload` stages, whose thrusters stay blocked until the payload flies alone.

## Adding another rocket
1. Add missing engines under `engines/`.
2. Copy a vehicle file, adjust stages/tanks/engine groups.
3. `python tools/vehicle_report.py <vehicle-id>` prints masses, burn times, T/W and the
   Tsiolkovsky delta-v budget as a sanity check.

## Atmosphere optics (`atmosphere/earth_optics.toml`)
Scalar optical inputs of the Earth atmosphere for the precomputed scattering (M8 atmosphere): model top and ground,
aerosol (optical depth at 550 nm, Angstrom exponent, single-scattering albedo, scale height, Mie asymmetry), ozone
column in Dobson units, ground albedo, scattering orders. Every key is tagged ([S] / [E] / [call]) in the file;
loaded by `config::load_atmosphere_optics` (missing keys throw). The spectra (solar irradiance, ozone cross sections,
CIE colour matching; Bruneton, BSD-3) are compiled in `core/include/starsim/environment/atmosphere_spectra.hpp`.

## Geoid (`geoid/egm2008_1deg.csv`)
EGM2008 undulation N (geoid above the WGS84 ellipsoid) on a 1 degree grid: a header of `#` comment lines (source,
generation command, licence), then 181 rows (90 N .. 90 S) x 361 integer values in cm (180 W .. 180 E inclusive,
the last column repeats the first). Generated by `python tools/data_pipeline/geoid.py --write-config` from the NGA 2.5'
grid fetched with `fetch.py geoid_egm2008` (public domain); loaded by `config::load_coarse_geoid`. Over ocean and coast
the bilinear error against the 2.5' grid stays below 2 m (measured maxima in `tests/validation/geoid/egm2008_points.csv`);
the fine grid in `data/geoid/` serves where metres matter.

## Earth terrain (`earth_terrain.toml`)
`[[area]]`: `id`, centre `latitude_deg` / `longitude_deg`, `local_half_size_m` (3DEP 1 m tile) and
`regional_half_size_m` (GLO-30 tile); `[global] relief_step_arcmin` (ETOPO 2022 relief). `fetch.py` cuts
`data/terrain/earth/<id>_local|regional.toml/.i16` and `global_relief.*` (WGS84 ellipsoidal heights);
`sim::load_earth_terrain` reads them with the fine geoid `data/geoid/egm2008_25.*` and the site plateaus of
`launch_sites.toml`, and refuses a tile whose area, level, centre or half size (one cell of slack) no longer matches.
Precedence: plateau > local > regional > global relief, each finer grid blended into the next over 500 m from its
edge; terrain at or below 0.05 m orthometric is water at the geoid.

## Earth imagery (`imagery.toml`)
`[[source]]`: `id`, `product` (`naip` = USDA NAIP 0.6 m via Planetary Computer, newest year covering the box;
`sentinel2_median` = own 2025 L2A clear-pixel median composite, 10 m), `area` (id in `earth_terrain.toml`), `box`
(`local` = local half size + 500 m margin, `regional` = regional half size), `min_level` / `max_level` (EPSG:4326 TMS
levels: z17 = 0.59 m/px, z13 = 9.5 m/px). `fetch.py imagery` writes `data/imagery/earth/<id>/<z>/<x>/<y>.jpg` (256 px,
quality 88) and `data/imagery/earth/index.toml` (per source: levels, exact coverage, `fade_m`, per-channel colour fit
`gain_rgb` / `offset_rgb`, already applied to the tiles). NAIP is shifted from NAD83 to ITRF.
Colour fit: Sentinel-2 -> day texture with ONE luminance gain and offset (equal in R, G, B; the texture is ~7 km per texel);
NAIP -> the fitted Sentinel-2 composite of its area, per channel at 10 m. A gain outside [0.2, 5] aborts with an error.
No data inside a box (sea beyond the NAIP scenes, Sentinel-2 composite gaps) is filled before writing, in final colours:
NAIP from the fitted Sentinel-2 composite, Sentinel-2 from the day texture; every tile of a box is written, and the lower
levels average only valid pixels.
`python tools/data_pipeline/fetch.py --retile imagery` redoes the fit and all tiles from the cached clips / composites in
`data/raw/imagery/` without network; rebuilding Sentinel-2 (also `--retile`) always re-fits the NAIP of its area.
Parallelism: tile writing and the Sentinel-2 composite blocks run on a process pool; the worker count is `--workers N`, else
the CPUs usable by the process (`os.process_cpu_count()` / `os.cpu_count()`), capped by 80 % of the free memory
(per-worker estimates in `imagery.py`); the chosen count is logged at start. The output does not depend on the count.
The renderer (`game/world/terrain/imagery_source.gd`) reads `index.toml`: per terrain node the finest source covering it at
about 256 texels across the node (levels between `min_level` and `max_level`) plus the next coarser source as base, blended
over the coverage edge with a width of max(`fade_m`, camera distance x 0.05 rad) (`docs/subsystems.md` "Terrain rendering").

## Structure detail files (`structures/`)
- `structures/starbase_olp2.toml`: the site-independent Pad B tower and chopstick detail (draft from first measurements):
  `[tower] height_m` + `[[tower.sections]]` (`height_m`, `face_width_m`, `pitch_m`, `pattern`, `leg_profile`,
  `brace_profile`; `pattern` x_brace / k_brace / chevron), `[tower.deck]` (`thickness_m`, `inset_m`), `[tower.core]`
  (`size_m`), `[tower.roof]`, `[tower.foot]`, `[profiles.<name>]` (`kind` h_beam / tube / angle / box, `size_m`, `wall_m`), `[chopstick]`
  (`length_m`, `width_m`, `depth_m`, `truss_pitch_m`, rail / chord / brace profile). Every value line carries
  `# [E] <method>` or `# [S] <url>`. Validated by `tools/structures/detail_file.py` (`python -m pytest tools/structures/tests`).
  Placement and the physics numbers stay in `launch_sites.toml`. State: `docs/subsystems/launch-structures.md`.

## Vehicle detail files (`vehicles/detail/`)
- `vehicles/detail/<vehicle id>.toml`: visual detail of one vehicle for the Blender model generators (weld pitch, hot-stage
  vents, grid-fin lattice, flap slab, catch-pin heads, chine, raceway, tile layout, engine bell contour). Placement and
  size always come from `vehicles/<id>.toml` and `engines/*.toml` (length, diameter, nose, flaps, fins, pins, engine
  rings, nozzle exit diameters, belly azimuth); the detail file repeats none of them. Every value line ends in
  `# [E] <method>` or `# [S] <url>`. `[hull]`: `segments` of the plain lathe, `nose_drawn_length_m` / `nose_tip_cap_m` (the drawn ogive; the physics nose length is the vehicle's); `[flap.aft|fwd]`: drawn planform `outline_u_m` / `outline_v_m` (hinge frame), hinge length, lug positions; `[grid_fin]` octagon `across_flats_m`, lattice pitch, serrated edge, `catch_fin_azimuths_deg` (fins with the catch hardware); `[booster.hot_stage_ring|top_truss|skirt|raceway]`, `[ship.ports]`, `[ship.engine_recess]` (exit plane above the ship's aft end per engine kind), `[ship.engine_bay]` (open aft end, plate); ring pitch, weld height / half width and
  `dent_slope_rms` are baked into the steel map (`tools/structures/hull_textures.py`). Validated by `tools/structures/vehicle_spec.py`
  (`python -m pytest tools/structures/tests/test_vehicle_models.py`). State: `docs/subsystems/vehicle-models.md`.
