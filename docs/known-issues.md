# Known issues

Moved verbatim from `CLAUDE.md` on 2026-10-25. Per-subsystem open points also live in `docs/subsystems/<slug>.md`.

- `optim` (SOCP solver, ECOS-style HSDE): dense Nesterov-Todd blocks per cone, so cones should stay small
  (dimension <= ~20); ordering is minimum degree on the explicit elimination graph (not full AMD). Solution
  vectors are accurate to roughly sqrt(tolerance) near cone boundaries; objective and feasibility to the tolerance.
- The very first `--import` of a fresh `game/.godot` segfaults on editor shutdown when the
  extension is present (import itself completes; later runs are fine). Not yet investigated.
- Solar radiation pressure is modelled for vehicles (FlightBody); the Artemis I coast test still uses the
  pure gravity propagator and shows SRP as a t^2 residual (~1.9e-8 m/s^2: 39 m after 18 h).
- Detailed subsystem notes - vehicle data (Block 3, Falcon 9, Crew Dragon, HLS), ascent, booster return and catch,
  SCvx landing, ship entry / flip / catch / return, legs, RCS, capsule entry and parachutes, lunar missions - with
  their decisions, validation numbers and `STARSIM_DEBUG_*` diagnostics live in `docs/subsystems.md`. Read the
  relevant section before changing a subsystem and update it with the change.
- Standing decisions from there: only Block 3 / V3 is modelled (2026-10-13); Super Heavy is always caught, Falcon 9
  lands on legs; physics first - webcast propellant gauges that the engines cannot explain are not followed
  (2026-10-14); Starship throttle schedules are guidance choices, not vehicle data.
- In scaled space the Earth is a 256x128 ellipsoid mesh (facets up to ~0.6 km below the true surface); closer, the
  streamed terrain quadtree (`game/world/terrain/`, M8 Plan B) draws it: with the Earth terrain loaded it follows
  `StarsimCore.surface_samples` (relief, site plateaus, sea at the geoid) plus NAIP / Sentinel-2 imagery, without data/
  the flat rule (the pad height near launch sites). Physics ground (M8 Plan A): with `FlightSession::set_earth_terrain` (the game, `[data]`
  tests) the real Earth terrain - flat plateaus at the derived site heights under every pad / landing zone, 3DEP /
  GLO-30 / ETOPO beyond, water at the EGM2008 geoid; without it (all non-`[data]` tests, the comparison mode) flat at
  the site height (FlightBody::ground_height_m) with the coarse committed geoid as the sea. Site heights in
  `launch_sites.toml` are ellipsoidal [D] (`tools/data_pipeline/site_heights.py`; Starbase -21.11 m). Gentle contact
  (<= 3 m/s, <= 10 deg) rests, harder contact is an impact. Landing legs creep downhill (~0.05 m/s) on slopes >= 2 deg
  and are never judged landed there (off-plateau only; not fixed). Sky, haze and limb come from the precomputed
  atmosphere (M8, `docs/subsystems.md` "Atmosphere rendering"): every Earth surface, the sea's reflected sky, Moon, Sun,
  stars and vehicles are seen through it, with eye adaptation (gain <= 25) and the eclipse sky; no clouds yet; one Sun
  light, evaluated at the followed body. Unverified: NAIP suggests the real OLP-1 mount stands some tens of metres WSW
  of our [E] site coordinates. Details: `docs/subsystems.md` "Earth terrain and geoid".
- Terrain-load draw stall (open, for M7 where the VR frame budget is real): ~1 `terrain_perf.gd` run in 3 shows one
  11-17 ms frame (once 33.7 ms) inside the RenderingServer draw (outside the viewport render) while a batch of terrain
  builds runs on the worker pool. Engine-internal, pool-size dependent: `threading/worker_pool/max_threads = 8`
  removed it (9 of 9 runs) but the Cape then loads in > 20 s instead of ~8 s - a trade-off, not changed. Also open:
  the eclipse epoch's p99 ~8.7 ms (GPU, the per-vertex sky eclipse factor). Details: `docs/subsystems.md`
  "Atmosphere rendering", Performance.
- C++ sources are UTF-8 (/utf-8); UI strings in the core are German.
- The TWCS lever reads 0 until first moved; FlightController only takes it over after a movement.
- Interactive regression check (keys injected, logs clock/altitude/throttle): `res://tests/flight_check.gd`.
- Horizons serves DE441; our DE440 Moon differs by up to 3.3 m (1990-2040). Tolerances reflect this.
- Weather and clouds (M8): the snapshot's layers are scaled to the data's total cloud cover (static clouds Task 9c;
  the 37 pressure levels lose the overlap of the model levels, 0.14-0.20 below tcc before), keeping the grid-mean
  water: the in-cloud water of decks and cirrus is thinner by that factor; the boundary-layer cumulus get a
  sub-adiabatic parcel's water (grid-mean liquid water path 50 -> 111 g/m^2 on the analog hours - still above the ocean
  climatology of order 60-90 g/m^2 quoted from memory for MAC-LWP, so the mean water is deliberately high (open: compare against the MAC-LWP monthly climatology on the same months / ocean regions); f_ad falls
  from 0.50 at the base to 0.20 at 2 km); low stratiform is not too dark by its renderer (body-core tau median 97 on the
  analog set, carried by > 4 km frontal runs; the < 1 km runs median 20, Task 10 - `weather-clouds.md` Validation); the limits (light between columns, shadow darkness under broken cloud, plumes always draw
  over clouds, earthshine at the constant Bond albedo, half-resolution staircase at sharp dark edges, no rain / fog /
  wind) are listed under "Open points and known limits" in `docs/subsystems/weather-clouds.md`.
- `cloud_far_check.gd` (2026-10-27): the Moon-view case needs `data/weather/cache/era5_1089330.wthr`, which the cache
  eviction removed; it prints a SKIP line (environmental; refetch it or pick a cached epoch). Two hand-over fade cases
  fail (listed in `docs/subsystems/weather-clouds.md`, Open points).
- Planned (user 2026-10-27): milestone M11 (after M7 cockpit and M10 sound design) holds the rest of the weather - part
  2 rain / snow / fog / lightning / weather-dependent haze (small variant: climatology AOD plus humidity growth, a few
  precomputed table sets) / city light on clouds; part 3 GOES patterns; part 4 wind / density and launch rules.
