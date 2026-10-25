# Earth terrain and geoid (M8 Plan A) - history

Verbatim record of this subsystem's notes as they stood in `docs/subsystems.md` on 2026-10-25, before the split. Current state: `docs/subsystems/earth-terrain.md`. Kept for the reasons behind decisions, superseded numbers and earlier approaches; read only when that history is needed.

Plan `docs/superpowers/archive/plans/2026-10-23-m8a-earth-terrain-physics.md`, spec
`docs/superpowers/archive/specs/2026-10-23-m8-terrain-tiles-design.md`. The physics stands on the real ground near the sites
and on the geoid at sea; the renderer (Plan B, section "Terrain rendering") draws the same surface.

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
  undulation. Horizontal NAD83(2011) -> ITRF2014 shift of the 3DEP grids (Plan B Task 3, `earth_terrain.py`
  `nad83_horizontal_shift_deg`, pyproj; the grid origin moves by the shift at the grid centre, header
  `horizontal_shift_m`): Starbase -0.892 m east / +0.439 m north (0.995 m), Cape -0.453 / +0.641 m (0.785 m); the shift
  varies by at most 1.9 / 2.5 mm over a box. Site heights unchanged at 0.01 m (no re-validation needed). The GLO-30
  fallback is WGS84 already. Seam local - regional after the shift (mean / median / rms): Starbase -0.205 / -0.156 /
  0.537 m, Cape -0.943 / -0.674 / 1.567 m - GLO-30 is a surface model (canopy, buildings), 3DEP bare earth; the 500 m
  blend absorbs it.
- **Shared terrain cache** (Plan B Task 1, `sim::shared_earth_terrain` / `shared_lunar_terrain`,
  `terrain_cache.hpp`): one immutable instance per (config_dir, data_dir), weak_ptr map, loaded under a mutex; the
  flight session, the lunar missions and the renderer's `StarsimCore.surface_samples` share it. Peak private memory of
  two headless `FlightSim.setup` calls: 1746 -> 942 MB.
- **Leg sites on the Earth** (Plan B Task 2): with the terrain set, a site's leg landing (`LegSite` on the terrain
  field) tracks a body only within `earth_leg_site_tracking_radius_m` = 5 km [E] of the site (`flight_session.cpp`; the
  local 1 m tile reaches 5-6 km): a landing that misses its zone by more is no leg landing, and the body's own ground
  contact on the terrain judges it (rests or impacts at the terrain height).
- **Plateaus.** Every site of `launch_sites.toml` is a flat plateau at its `height_m` within `plateau_radius_m`, then a
  linear apron of `plateau_apron_m` into the terrain; never water. Precedence plateau > local > regional > global.
- **Site heights** [D] (Task 6, `tools/data_pipeline/site_heights.py --write`: the median of the local grid within the
  plateau radius; old [E] -> new ellipsoidal, orthometric, spread inside the radius):

  | Site | Old | New | Orthometric | Spread | Radius |
  |---|---|---|---|---|---|
  | Starbase OLP-1 | +10.00 m | -21.11 m | +1.41 m | 3.1 m | 150 m |
  | KSC LC-39A | +15.00 m | -15.85 m | +13.84 m | 12.9 m | 150 -> 60 m |
  | CCSFS SLC-40 | -20.00 m | -23.29 m | +6.41 m | 3.3 m | 100 -> 30 m |
  | CCSFS LZ-40 | -20.00 m | -26.03 m | +3.67 m | 3.6 m | 61 m |

  LC-39A's spread gate (5 m) is waived (ruling 6): the median matches the published ~48 ft pad deck and is stable over
  radii 30-80 m; the spread is the flame trench and ramps inside the mound.
- **Water rule.** Terrain at or below 0.05 m orthometric (`EarthTerrain::water_threshold_m`; GLO-30 sets the sea to 0)
  is water at the geoid. Lakes and inland depressions are not separated from the sea (no connectivity test): inland
  ground below mean sea level would count as water. Tides and mean dynamic topography: not modelled.
- **Sea level everywhere = the geoid** (Task 5): `FlightSession::sea_height_m` (the fine geoid with terrain, else the
  coarse committed grid). Drone-ship decks sit at N + freeboard (the ASDS station at 32 N 76 W: N = -43.2 m, was a
  fixed -35 m [E]); capsule splashdown on N (San Diego zone -37.0 m, was the ellipsoid); the crew escape's water and
  ditching from the terrain's water flag (else the site's `[site.sea]` shoreline in the flat model).
- **Flat-ground comparison mode.** `FlightSession::set_earth_terrain(nullptr)` (the default) keeps the flat model:
  ground at each body's `ground_height_m` (the site height), the sites' sea config, the coarse geoid for the sea. Tests
  (not `[data]`) run that way with the derived site heights; the game and the `[data]` tests load the terrain
  (`bindings/src/flight_sim.cpp` loads it when `data/` holds it). Guidance always aims at the baked site height, which
  the plateau reproduces exactly. Telemetry `altitude_terrain_m` is above the terrain when set, else above the flat
  ground (`altitude_m - ground_height_m`): the webcast altitude gauge and the HUD show it (0-ish on the pad; the
  ellipsoidal `altitude_m` is negative there), the gauge situation thresholds keep `altitude_m`.
- **Known issue (Task 4, not fixed):** landing legs creep downhill at ~0.054 m/s on slopes >= 2 deg (the regularised
  friction in `contact/landing_legs.cpp`, a `PlaneField` the same) and are never judged landed there. The plateaus are
  flat, so no site landing is affected; an off-plateau touchdown on a slope stays "moving".
- **Re-validation (Task 7, derived site heights).** Tag sets `[ascent],[ascent_planner],[catch],[return],[rtls],[asds],
  [falcon9],[ship],[crew_escape],[splashdown],[iss_mission],[validation],[booster],[scvx_landing],[descent],[entry],
  [deorbit]` (425 cases) and `[refuelling],[campaign],[lunar]` (268 cases): every catch, landing, planner and campaign
  passed on the new heights; one failure, the tanker's reference measurement (below). A wider sweep over the tags the
  site heights reach (`[config],[sim],[session],[legs],[staging],[timeline],[dragon],[iss],[earth_terrain],[gnc],
  [contact],[capsule],[parachute]`, 708 cases) found three more: the Starbase height in "Launch sites load with their
  catch tower" (10.0 -> -21.11 m; missed by Task 6) and the launch window's recorded Waypoint 0 epoch (SLC-40's pad
  3.29 m lower: 845519535.204464555 -> .204483509 s, +19 us; the liftoff unchanged), both re-baselined, and the ship
  flip-and-catch timeline's gate prediction - a defect, fixed (below).
  - **Gate prediction of the ship's landing burn: the online scales learnt attitude transients** (`test_mission_timeline.cpp`
    "A ship's flip and catch records one landing burn", a ship placed 28 km west / 26 km above the site): the
    whole-track error grew 0.83 -> 2.92 s (bound 1.0 s). A/B with only the Starbase height reverted to 10 m gives
    0.83 s again. Cause: `EntryGuidance::gate_epoch` predicts with the online drag / lift scales (updated every 2 s,
    the accelerometer against the trimmed table at the flown alpha). At Mach ~0.59 the terminal command steps from
    alpha 78 to 86 deg; the attitude overshoots and oscillates for ~12 s (5-9 deg off the command, roll / pitch rates
    up to 5 deg/s) and the flaps' control deflections raise the measured drag to 1.08-1.12 x the table (steady flight
    before and after: 1.004 / 0.996). The scales learnt it (drag 1.00 -> 1.09, lift 1.11) and the prediction ran +2.9 s
    late until the filter unlearnt it; on the old height the oscillation was better damped (1.01). The 31 m only moved
    the phase of the oscillation. Fix: in the terminal phase the scales are kept when the attitude error (flown vs
    commanded, the largest over the 2 s cycle - an oscillation passes through small errors with its flaps deflected)
    exceeds `EntryGuidanceParameters::scale_max_attitude_error_rad` 2 deg [E] (steady terminal flight <= 0.3 deg).
    Now: -0.82 s 179 s ahead, 0.30 s within 120 s, 0.09 s within 60 s, 0.04 s within 30 s (bound 1.0 s kept, new
    bounds 0.5 s within the last 60 s and the drag scale <= 1.03); catch prediction -1.81 s 18 s ahead.
  - **Altitude reference of the webcast comparisons** (category a): the dials read 0 on the pad, our samples were the
    ellipsoidal height (`core/tests/ascent_validation.cpp`), which the old estimates kept near the pad (+10 m / -20 m)
    and the derived heights would put 21-23 m low. The samples are now above the pad (flight 14 at OLP-1, CRS-35 at
    SLC-40); the same for the booster apogee print (`test_booster_return.cpp`). Flight 14 altitude T+30 / 60 / 90 /
    120 s / MECO: 1.52 / 8.24 / 21.44 / 41.43 / 52.50 -> 1.54 / 8.26 / 21.46 / 41.45 / 52.52 km (T+30 s -4.2 ->
    -2.8 %); CRS-35: 1.20 / 7.15 / 19.40 / 40.50 / 59.51 -> 1.22 / 7.17 / 19.42 / 40.52 / 59.53 km (T+30 s -8.9 ->
    -7.2 %, T+60 s -9.0 -> -8.7 %); speeds, times and path angles unchanged by the conversion. Allowances unchanged.
    Booster apogee 100.4 km above the pad at T+223 s (flight 14: 98.9 km at T+240 s).
  - Flown on the derived heights (flat mode): Starbase booster catch 0.0 m from the catch axis, 1.25 MN pin load,
    52.2 t at landing-burn ignition (planned 51.0 t); Falcon 9 RTLS MECO T+140 s with 52.1 t, landing burn T+423 s at
    2.97 km / 287 m/s / 12.1 t, landed T+452 s (was T+455 s: LZ-40 6 m lower), 0.01 m off, 6.4 t left; ASDS unchanged
    (landed T+529 s, 0.00 m off the deck, 5.0 t); flight 14 kick 16.30 deg, MECO T+129.75 s with 293.7 t, 165.0 x
    277.1 km; CRS-35 kick 11.80 deg, MECO T+135.49 s, 197.9 x 202.1 km.
  - **Refuelling reference flights re-measured** (`reference_flights.toml`, category a: the pads moved): tanker
    rendezvous 17.37 -> 17.93 t (outside its 1 % / 0.3 t check: the failing case), delivered LOX / CH4 59.4 / 14.85 ->
    58.9 / 14.7 t, main at catch 1.21 -> 1.38 t; residue 105.6 t, settling gas 0.98 t, main at undock 12.6 t, depot
    fill 70.7 t (199.7 x 211.3 km) unchanged; HLS main at docking 202.8 -> 203.5 t. Campaign offsets kept (the full
    flown campaign and the placed campaign passed against the calculator). Calculator (default): 12 tankers / 50.6 d
    -> 12 tankers / 50.7 d.
  - **On the real terrain** (`core/tests/test_earth_terrain_flight.cpp`, `[data][slow][earth_terrain]`, the session
    with `sim::load_earth_terrain`): "starbase catch on the real terrain" - caught, tower not struck, booster 0.03 m
    from the catch axis, 1.25 MN, the ground under it -21.110 m = the plateau; "Falcon 9 RTLS to LZ-40 on the real
    terrain" - landed 0.01 m from the centre, 6.4 t left, feet at -26.0356 m on the -26.0300 m plateau (5.6 mm leg
    contact compression).

