# Known issues

Moved verbatim from `CLAUDE.md` on 2026-10-25. Per-subsystem open points also live in `docs/subsystems/<slug>.md`.

- Red by decision since the Pad B physics alignment (user 2026-10-31: no campaign tuning before the vehicle plan, plan
  `docs/superpowers/archive/plans/2026-10-31-pad-b-physics-alignment.md`): the lunar direct campaign has no robust gate height, 21 seeds
  (800 m: seed 3 lost, arm in the hull with 20.7 t; 815 m: all caught, one on 0.0 t; 825-850 m: seed 11 upright abort), cause not
  found; `tanker_flight` and the refuelling inputs digest need the re-measure; ship case 20 ends upright on 0.0 t below the
  1 t reserve, unexplained. Not verified: that a ship abort with the water out of reach never strikes the tower, and that the
  booster's 400 m abort point (loss, 28 m from the pad centre) clears the drawn tower, mount, foot and ramps; the wide run did
  not finish (aerocapture at gate 800, the flown depot campaign).

- Red since the ship's entry value set (vehicle physics Task 5b, 2026-11-01, user's OK on the set; no limit changed; re-measured Task 6d):
  - `[ship][entry][validation]` flight 14: speed rms 8.33 % (limit 5, the limit stays; flights 13, 12: 4.05, 1.51 %), flying fast. The mesh's hull centre of pressure takes the aft flaps from the stop (88-90 deg) to 30-37 deg, flap CdA from 55-65 to 13 m^2, total -10 % at Mach 8-25. Nose and `ship_ascent.csv` move this by at most 0.3 points. An experiment with 11 t instead of 35 t of header propellant reaches 3.15 % there, but low rms 112.9 % and subsonic ratio 3.31 (departs, alpha 168 deg at Mach 1.46): not adopted, the header load is not established by it. The webcast flights share a CdA shortfall of the mesh hull (`aero-derivation.md`).
  - Trim margin, "The ship trims belly first with control margin" and "Flaps pinned at their stops" (`[entry][flap][trim]`): at 65 km, 35 t, alpha 60 the forward pair trims at 77.2 deg (limit 75); alpha 70 does not trim (forward at the 90 stop, aft 19.9, remainder 3.2 x, limit 0.01). The schedule flies 52-55 deg there, where the margin holds (forward 53-62 deg). The limit is not changed.
  - The flap allocation test at alpha 60 keeps its 1e5 N m: 3e5 was tried with the authority floor and still leaves RCS, the forward pair at 77 deg.
  - The 64 t aerocapture is green: apoapsis 426 km, axis error 1.6 deg, peak 924 kW/m^2. At 64 t below Mach 2.5 the aft pair is at its stop (short 42-619 m^3 per q).
- Low-Mach control margin of the belly flop (Task 6d): the attitude step when the terminal basis freezes (gamma -70, Mach ~0.68) demands ~1e6 N m of yaw, above what the flaps deliver at zero roll (+-8.4e5 N m at alpha 80.5, forward flaps fixed about -1.3e5 / +0.7e5). Roll reaches 0.05-0.07 rad/s and makes sideslip; at 13 deg of it the hull's yaw moment nearly exhausts the flaps, at 20 deg it is beyond them. The authority floor of the allocation (`flap_lowest_effective_rad`, a quarter of the best slope) cleared every departure seen, and case 8 under flap table scalings up to +-0.2 %: 0 of 10 depart (without floor 8 of 10). The window is narrow: fraction 0.20 2 of 10, 0.15 and 0.30 9 of 10. 0.25 was set before the first run, from `flap_tiny_column_fraction`, and not moved. The step itself is untouched (no guidance change).

- Ship crossflow gap: `ship_crossflow.csv` stays the flights' 1.5-1.6 [D], the drawn hull derives 1.07-1.10 at Mach 4-25 (`aero-derivation.md`). Open, not closed by the alpha the frames support.
- Forward RCS pod (for Task 6): floats 1.15 m off the drawn hull (radius 4.5 against 3.348 at station 46). Moved alone it misses the catch gate at 148 m (limit 100), so it stays.
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
- Pad B physics alignment (user 2026-10-31): the final arm geometry (one hinge axis, arm 24.0 m, standby 7.1 deg, closing
  0.71 s) has flown no catch; the booster and ship catch campaigns and the `[slow]` catch cases have not run since. They are a
  gate of the vehicle plan.
- `tools/package.ps1` packs `data/models` unchecked: a stale or missing local model would ship in the installer. Open (review
  2026-11-01 F5; fix: stop packaging when the manifest hash does not match `build_models.input_hash`).

## Pad B drawing, open after the physics alignment plan (2026-11-01)
- Slits between the cladding plates on the pad-side tower face: the sky shows through in places (`screenshots/pad-b-plan/fix6_leg.png`).
- Tower braces have no gusset plates; the close-up photo shows them clearly (`nsf_padb_chopsticks_2025-02.jpg`).
- Hoist cables: the worker photo reads 2-3 cable widths apart across the block; only 1.2 fit in the 2 m block at 4 per group, so the pitch is 0.19 m and the group spacing 1.25 m (photo reading 1.5 m). From the pad side the two groups still read as two bars.
- Hinge axis direction (outward diagonal) is an assumption; the photos give only the 2 +-1 m. Rollers, third rail corner and second drum are the user's description, not resolved on photos.
- Plan Task 6 step 2 (`[slow]` catch cases) did not run by user decision; it moves to the vehicle plan's campaign gate.
- `tower_handover_check` is red (2026-11-01): 8 failures on fa226c8, 7 on aae1e5d, about 6 % near/mid against its limit. Cause not traced; not known whether it was green before the leg colour change e67af13.
- Tower head underside 140.5 m, 1.4 m above the lattice top where the photo reads level: at 139.1 m the sheave block would enter the head at +0.7 m carriage offset, the stroke allows +4 m, and it clears to +2.1 m as built.
