# Earth terrain and geoid (M8 Plan A)

History and the reasons behind superseded decisions: `docs/history/earth-terrain.md`.

Plan `docs/superpowers/archive/plans/2026-10-23-m8a-earth-terrain-physics.md`, spec
`docs/superpowers/archive/specs/2026-10-23-m8-terrain-tiles-design.md`. The physics stands on the real ground near the sites
and on the geoid at sea; the renderer (`docs/subsystems/terrain-rendering.md`) draws the same surface.

## Model

- **Sources and levels** (`tools/data_pipeline/earth_terrain.py`, `geoid.py`, licences in `sources.toml`; all grids
  hold WGS84 ellipsoidal heights as int16 + TOML header with `[[check]]` points):

  | Level | Grid | Source | Size |
  |---|---|---|---|
  | local (1) | 1 m, 10 x 10 km Starbase / 12 x 12 km Cape (LC-39A, SLC-40, LZ-40) | USGS 3DEP 1 m bare earth (NAVD88 / GEOID18), gaps (water, ~34 %) from GLO-30 | Starbase 189 MB, Cape 272 MB |
  | regional (2) | ~1", 200 x 200 km around each area | Copernicus GLO-30 (EGM2008 heights) | 93.5 / 95.7 MB |
  | global (3) | 2' | NOAA ETOPO 2022 60" surface, 2 x 2 box average, + EGM2008 | 116.7 MB |
  | fine geoid | 2.5' | NGA EGM2008 | 74.7 MB (`data/geoid/egm2008_25.*`) |
  | coarse geoid | 1 deg | EGM2008 (`config/geoid/egm2008_1deg.csv`, committed) | bilinear error < 2 m over ocean / coast |

  `config/earth_terrain.toml` names the areas; `environment::EarthTerrain` blends local -> regional -> global over
  500 m from each finer grid's edge; `sim::load_earth_terrain(config_dir, data_dir)` loads it and refuses stale tiles
  (area, level, centre or half size off by more than one cell).
- **Datum chain.** 3DEP: NAVD88 orthometric + GEOID18 -> NAD83(2011) ellipsoidal -> ITRF2014 (EPSG 7912 Helmert,
  -1.36 m at OLP-1, -1.54 m at the Cape; WGS84(G2139) = ITRF2014 at the cm level). GLO-30 and ETOPO: + the EGM2008
  undulation. Horizontal NAD83(2011) -> ITRF2014 shift of the 3DEP grids (`earth_terrain.py`
  `nad83_horizontal_shift_deg`, pyproj; the grid origin moves by the shift at the grid centre, header
  `horizontal_shift_m`): Starbase -0.892 m east / +0.439 m north (0.995 m), Cape -0.453 / +0.641 m (0.785 m); the
  shift varies by at most 1.9 / 2.5 mm over a box. The GLO-30 fallback is WGS84 already. Seam local - regional after
  the shift (mean / median / rms): Starbase -0.205 / -0.156 / 0.537 m, Cape -0.943 / -0.674 / 1.567 m - GLO-30 is a
  surface model (canopy, buildings), 3DEP bare earth; the 500 m blend absorbs it.
- **Shared terrain cache** (`sim::shared_earth_terrain` / `shared_lunar_terrain`, `terrain_cache.hpp`): one immutable
  instance per (config_dir, data_dir), weak_ptr map, loaded under a mutex; the flight session, the lunar missions and
  the renderer's `StarsimCore.surface_samples` share it. Peak private memory of two headless `FlightSim.setup` calls:
  942 MB.
- **Leg sites on the Earth**: with the terrain set, a site's leg landing (`LegSite` on the terrain field) tracks a body
  only within `earth_leg_site_tracking_radius_m` = 5 km [E] of the site (`flight_session.cpp`; the local 1 m tile
  reaches 5-6 km): a landing that misses its zone by more is no leg landing, and the body's own ground contact on the
  terrain judges it (rests or impacts at the terrain height).
- **Plateaus.** Every site of `launch_sites.toml` is a flat plateau at its `height_m` within `plateau_radius_m`, then a
  linear apron of `plateau_apron_m` into the terrain; never water. Precedence plateau > local > regional > global.
- **Site heights** [D] (`tools/data_pipeline/site_heights.py --write`: the median of the local grid within the plateau
  radius; ellipsoidal, orthometric, spread inside the radius):

  | Site | Height | Orthometric | Spread | Radius |
  |---|---|---|---|---|
  | Starbase OLP-1 | -21.11 m | +1.41 m | 3.1 m | 150 m |
  | KSC LC-39A | -15.85 m | +13.84 m | 12.9 m | 150 -> 60 m |
  | CCSFS SLC-40 | -23.29 m | +6.41 m | 3.3 m | 100 -> 30 m |
  | CCSFS LZ-40 | -26.03 m | +3.67 m | 3.6 m | 61 m |

  LC-39A's spread gate (5 m) is waived (ruling 6): the median matches the published ~48 ft pad deck and is stable over
  radii 30-80 m; the spread is the flame trench and ramps inside the mound.
- **Water rule.** Terrain at or below 0.05 m orthometric (`EarthTerrain::water_threshold_m`; GLO-30 sets the sea to 0)
  is water at the geoid. Lakes and inland depressions are not separated from the sea (no connectivity test): inland
  ground below mean sea level would count as water. Tides and mean dynamic topography: not modelled.
- **Sea level everywhere = the geoid**: `FlightSession::sea_height_m` (the fine geoid with terrain, else the coarse
  committed grid). Drone-ship decks sit at N + freeboard (the ASDS station at 32 N 76 W: N = -43.2 m); capsule
  splashdown on N (San Diego zone -37.0 m); the crew escape's water and ditching from the terrain's water flag (else
  the site's `[site.sea]` shoreline in the flat model).
- **Flat-ground comparison mode.** `FlightSession::set_earth_terrain(nullptr)` (the default) keeps the flat model:
  ground at each body's `ground_height_m` (the site height), the sites' sea config, the coarse geoid for the sea. Tests
  (not `[data]`) run that way with the derived site heights; the game and the `[data]` tests load the terrain
  (`bindings/src/flight_sim.cpp` loads it when `data/` holds it). Guidance always aims at the baked site height, which
  the plateau reproduces exactly. Telemetry `altitude_terrain_m` is above the terrain when set, else above the flat
  ground (`altitude_m - ground_height_m`): the webcast altitude gauge and the HUD show it (0-ish on the pad; the
  ellipsoidal `altitude_m` is negative there), the gauge situation thresholds keep `altitude_m`.

## Code

- `tools/data_pipeline/earth_terrain.py`, `geoid.py`, `site_heights.py`; `config/earth_terrain.toml`,
  `config/geoid/egm2008_1deg.csv`.
- `environment::EarthTerrain`, `sim::load_earth_terrain`, `sim::shared_earth_terrain` (`terrain_cache.hpp`),
  `FlightSession::set_earth_terrain`, `FlightSession::sea_height_m`, `flight_session.cpp`
  (`earth_leg_site_tracking_radius_m`), `contact/landing_legs.cpp`.
- `EntryGuidanceParameters::scale_max_attitude_error_rad` (`EntryGuidance::gate_epoch`).

## Standing decisions

- Plateau > local > regional > global; every site flat at its derived height (user rulings of the M8 plan; ruling 6
  for LC-39A's spread).
- Sea at the EGM2008 geoid, calm, no tides (M8 terrain decisions).
- Guidance aims at the baked site height; the flat comparison mode stays the default and the test mode.
- Entry guidance: in the terminal phase the online drag / lift scales are kept when the attitude error (largest over
  the 2 s cycle) exceeds `scale_max_attitude_error_rad` 2 deg [E] (steady terminal flight <= 0.3 deg).
- Webcast comparison altitudes are above the pad (flight 14 at OLP-1, CRS-35 at SLC-40), not ellipsoidal.

## Validation

- `core/tests/test_earth_terrain_flight.cpp`, `[data][slow][earth_terrain]`, the session with
  `sim::load_earth_terrain`: "starbase catch on the real terrain" - caught, tower not struck, booster 0.03 m from the
  catch axis, 1.25 MN, the ground under it -21.110 m = the plateau; "Falcon 9 RTLS to LZ-40 on the real terrain" -
  landed 0.01 m from the centre, 6.4 t left, feet at -26.0356 m on the -26.0300 m plateau (5.6 mm leg contact
  compression).
- Flown on the derived heights (flat mode): Starbase booster catch 0.0 m from the catch axis, 1.25 MN pin load, 52.2 t
  at landing-burn ignition (planned 51.0 t); Falcon 9 RTLS MECO T+140 s with 52.1 t, landing burn T+423 s at 2.97 km /
  287 m/s / 12.1 t, landed T+452 s, 0.01 m off, 6.4 t left; ASDS landed T+529 s, 0.00 m off the deck, 5.0 t; flight 14
  kick 16.30 deg, MECO T+129.75 s with 293.7 t, 165.0 x 277.1 km; CRS-35 kick 11.80 deg, MECO T+135.49 s,
  197.9 x 202.1 km.
- Webcast comparisons (above the pad): flight 14 altitude T+30 / 60 / 90 / 120 s / MECO 1.54 / 8.26 / 21.46 / 41.45 /
  52.52 km; CRS-35 1.22 / 7.17 / 19.42 / 40.52 / 59.53 km; booster apogee 100.4 km above the pad at T+223 s (flight
  14: 98.9 km at T+240 s); allowances unchanged (`core/tests/ascent_validation.cpp`, `test_booster_return.cpp`).
- `test_mission_timeline.cpp` "A ship's flip and catch records one landing burn" (ship 28 km west / 26 km above the
  site): gate prediction -0.82 s 179 s ahead, 0.30 s within 120 s, 0.09 s within 60 s, 0.04 s within 30 s (bound 1.0 s;
  bounds 0.5 s within the last 60 s and drag scale <= 1.03); catch prediction -1.81 s 18 s ahead.
- Launch window's recorded Waypoint 0 epoch (SLC-40 pad): 845519535.204483509 s; Starbase height in "Launch sites load
  with their catch tower" -21.11 m.
- Refuelling reference flights on the derived heights (`reference_flights.toml`): tanker rendezvous 17.93 t, delivered
  LOX / CH4 58.9 / 14.7 t, main at catch 1.38 t; residue 105.6 t, settling gas 0.98 t, main at undock 12.6 t, depot
  fill 70.7 t (199.7 x 211.3 km); HLS main at docking 203.5 t. Calculator (default): 12 tankers / 50.7 d.
- Re-validation tag sets passed: `[ascent],[ascent_planner],[catch],[return],[rtls],[asds],[falcon9],[ship],
  [crew_escape],[splashdown],[iss_mission],[validation],[booster],[scvx_landing],[descent],[entry],[deorbit]`;
  `[refuelling],[campaign],[lunar]`; `[config],[sim],[session],[legs],[staging],[timeline],[dragon],[iss],
  [earth_terrain],[gnc],[contact],[capsule],[parachute]`.

## Open points and known limits

- Landing legs creep downhill at ~0.054 m/s on slopes >= 2 deg (the regularised friction in
  `contact/landing_legs.cpp`, a `PlaneField` the same) and are never judged landed there. The plateaus are flat, so no
  site landing is affected; an off-plateau touchdown on a slope stays "moving". Not fixed.
- Lakes / inland depressions below mean sea level count as water; no tides or mean dynamic topography.
- GLO-30 (surface model) vs 3DEP (bare earth) seam offsets up to ~1 m, absorbed by the 500 m blend.
