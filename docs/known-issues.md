# Known issues

Moved verbatim from `CLAUDE.md` on 2026-10-25. Per-subsystem open points also live in `docs/subsystems/<slug>.md`.

- Red list on the frozen aero state (vehicle physics Task 6e, 2026-11-01; measured on 5167ce4 + the re-measure, findings not fixed):
  - Lunar direct campaign, with the arms as keep-out and the gradual gate law (Tasks 6g, 6h): 20 of 21 (limit 21). Seed 2 upright 4.9 t, gate 838.9 m at 101.9 m/s, 127 m east of the corridor, why: not determined; it was lost 19.7 t with the keep-out alone, and its safe end is not understood. Seed 15 caught 4.2 t (keep-out alone: lost 20.6 t), seed 12 caught 11.5 t, no ship lost, no strike, the others caught 7.2-12.2 t. Peak over all 21 seeds is 1171 kW/m^2 (seed 21, limit 1200), not the 1096 first reported.
  - `Docked tanker: the ship return flies to the catch`: lost. The tanker (35.7 t at the gate) departs in the terminal approach, axis error 100 deg, gate at 3,769 m with 217 m/s of sink; same phase as the low-Mach margin below, whether the same mechanism: not determined. `tanker_flight` from the pad is caught, on the header tanks (main 0.0 t).
  - `Flown campaign, placed depot`: 4 rows, the test wants 3: tanker 1 refused ("Kein Rendezvous: Phasenwinkel nicht innerhalb ..."), the campaign re-plans and completes. The full flown campaign passes its comparison but loses tankers 5, 9 ("Kein Rendezvous: Bahn tritt vor dem Annäherungsbeginn in das Annäherungsellipsoid ein", was 0 failures); its first tanker return ends upright (allowed by the test). Whether the re-measured file moved the placed case: not determined.
  - Ship abort scan (seeds 0-20, gate 950 and 800 shifted from the catch cases, not flown entry gates; abort CG 250-550 m, 273 runs per gate), struck / lost, before -> with the keep-out (Task 6g): gate 950: 85 / 94 -> 10 / 35; gate 800: 62 / 65 -> 20 / 24. Lost without a strike rose at gate 950 from 9 to 25 (several hit the ground with the headers dry, 0.0 t: going round costs propellant). Before, the tower itself was struck at dispersed gates too (-2.76 m); "never struck" held only for the nominal gate. Still: seed 19 at gate 800 strikes at every height (the abort in the flip, its last plan refused too late); some strikes have arm and tower clear (the mount, which has no hull row). Below 250 m: gate 950 no strike at 150-225 m; gate 800 1-3 of 21 at 150-225 m, so no lower edge there. Nominal gate 950, all heights: no strike; 375, 450 m lost (375 m: 0.0 t), the others upright. `tools/validation/abort_clearance.py` on the hidden `[.abort_clearance]` cases.
  - Aborts on the drawn pad, which the physics does not collide with: booster 400 m: landed 29 m from the centre, foot 12.2, block 9.1, annulus 8.45, tower box 17.4, arm 5.2 m, but 0.84 m into the 0.5 m trench wall. Ship aborts with the keep-out at 200-300 m come down at or over the drawn ramp, 1.1-4.3 m into the trench walls, except the 225 m abort (2.1 m clear, on the ramp over the drawn pit the flat physics ground does not know); 325 m: 0.3 m into the tower foot.
  - Ship case 20: upright on 0.3 t, below the 1 t reserve (the 0.0 t was the braking feedforward; fixed in ca4d506).

- Red since the ship's entry value set (vehicle physics Task 5b, 2026-11-01, user's OK on the set; no limit changed; re-measured Task 6d):
  - `[ship][entry][validation]` flight 14: speed rms 8.33 % (limit 5, the limit stays; flights 13, 12: 4.05, 1.51 %), flying fast. The mesh's hull centre of pressure takes the aft flaps from the stop (88-90 deg) to 30-37 deg, flap CdA from 55-65 to 13 m^2, total -10 % at Mach 8-25. Nose and `ship_ascent.csv` move this by at most 0.3 points. An experiment with 11 t instead of 35 t of header propellant reaches 3.15 % there, but low rms 112.9 % and subsonic ratio 3.31 (departs, alpha 168 deg at Mach 1.46): not adopted, the header load is not established by it. The webcast flights share a CdA shortfall of the mesh hull (`aero-derivation.md`).
  - Trim margin, "The ship trims belly first with control margin" and "Flaps pinned at their stops" (`[entry][flap][trim]`): at 65 km, 35 t, alpha 60 the forward pair trims at 77.2 deg (limit 75); alpha 70 does not trim (forward at the 90 stop, aft 19.9, remainder 3.2 x, limit 0.01). The schedule flies 52-55 deg there, where the margin holds (forward 53-62 deg). The limit is not changed.
  - The flap allocation test at alpha 60 keeps its 1e5 N m: 3e5 was tried with the authority floor and still leaves RCS, the forward pair at 77 deg.
  - The 64 t aerocapture is green: apoapsis 426 km, axis error 1.6 deg, peak 924 kW/m^2. At 64 t below Mach 2.5 the aft pair is at its stop (short 42-619 m^3 per q).
- Low-Mach control margin of the belly flop (Task 6d): the attitude step when the terminal basis freezes (gamma -70, Mach ~0.68) demands ~1e6 N m of yaw, above what the flaps deliver at zero roll (+-8.4e5 N m at alpha 80.5, forward flaps fixed about -1.3e5 / +0.7e5). Roll reaches 0.05-0.07 rad/s and makes sideslip; at 13 deg of it the hull's yaw moment nearly exhausts the flaps, at 20 deg it is beyond them. The authority floor of the allocation (`flap_lowest_effective_rad`, a quarter of the best slope) cleared every departure seen, and case 8 under flap table scalings up to +-0.2 %: 0 of 10 depart (without floor 8 of 10). The window is narrow: fraction 0.20 2 of 10, 0.15 and 0.30 9 of 10. 0.25 was set before the first run, from `flap_tiny_column_fraction`, and not moved. The step itself is untouched (no guidance change).

- Ship aero state frozen (2026-11-01, mass / CG / geometry audits and replays): crossflow `ship_crossflow.csv` 1.5-1.6 [D], the old entry alpha table, entry CG 25.50 m, dry mass 255 t, dry CG 23.516 m [D] stay as on main, because no tested combination passes all replays (flights 12-14 speed rms < 5 %, low-altitude rms < 10 %, saturated < 10 %, subsonic ratio 0.9-1.1; the guidance case 8). Replays, entry mass per flight 221 / 252 / 215 t:

  | Variant | f12 speed / sat % | f13 speed / low / sat % | f14 speed / low / sat % | Case 8 |
  |---|---|---|---|---|
  | main (255 t dry, crossflow 1.5, CG 25.50 m) | 1.51 / 5.5 | 4.06 / 4.7 / 0 | **8.33** / 9.0 / 0 | passes (0.11 m) |
  | measured masses, crossflow 1.5 | **5.14** / 1.0 | 2.66 / **12.5** / 0 | 3.75 / **22.2** / 0 | 0 of 50 green at 175 t dry |
  | measured masses, geometry crossflow 1.07-1.10 | 1.54 / 5.7 | **9.76** / **10.9** / 0 | **7.31** / 8.5 / 0 | passes at CG 25.50 m |
  | geometry + alpha 59.5 + masses + CG 24.85 m | 4.54 / **10.20** | 3.54 / **14.96** / **15.21** | 4.45 / **22.06** / 8.53 | departs, 10 of 10 fail (miss 5.2 km) |

  Facts behind it (audits are estimates, +-20 t unless noted):
  - Dry mass 255 t is probably too high: the one-engine phase of flight 12 allows m <= 209-218 t at 280 tf (a weak source); 255 t would need >= 117 % of the engine rating (328 tf). Judgement: about 175 +-25 t (not a statistic). The propellant gauge excludes the landing propellant, so the rocket-equation 255 t contains it.
  - Entry masses (gauge still shows main-tank residue): flight 12 221 t, flight 13 252 t, flight 14 203-227 t (only if its payload was released, one weak source), each +-20 t.
  - CG: the trim window over Mach 0.7-25 is 24.49-25.88 m at the stops (15 deg margin 24.73-25.65 m). 25.50 m (entry) and 23.516 m (dry) are tuned values: the dry CG was 28 m, scanned to make the old hull model trim, then inverted to 23.516 m through the header tanks. A bottom-up estimate gives a dry CG of about 22.9 m (range 19.4-23.5 m). The trim margin test is green at an entry CG of 24.85 m at every dry mass tried, red at 25.50 m. 24.85 m costs aft flaps of 66-71 deg at Mach 0.7-1.5; the flight 14 frames fit it, flight 12's do not.
  - A lighter dry mass moves the CG more with the main-tank residue: at 175 t dry and 24.85 m the entry CG at Mach 1.1 is 23.75 m, below the window (24.42 m), and case 8 departs. At the old alpha with the measured masses and CG 25.50 m case 8 passes only with the geometry crossflow.
  - Crossflow gap: the drawn hull derives 1.07-1.10 at Mach 4-25 against the flights' 1.5. The gap in CdA (500-570 m^2 at 290 t, 381-433 m^2 at the measured masses) is a mass artefact; the table is then within +-13 % of the estimate above Mach 6 on flights 13, 14. The geometry set with alpha 59.5 fits the CdA (within 8 % on flights 13, 14) and the speed, but flies too slow at Mach 3-6 (mean -10 to -15 %): flight 13 low 14.96 %, flight 14 low 22.06 %.
  - Allocator window (Task 6d): fraction 0.25 passes case 8; failures out of 10 perturbed runs at fractions 0 / 0.15 / 0.20 / 0.25 / 0.30: 8 / 9 / 2 / 0 / 9. The window vanishes with the geometry crossflow (0 failures at every fraction) and with the table at 175 t dry it is 10 of 10 at every fraction. Cause: the yaw demand at the terminal basis freeze (gamma -70 deg, Mach ~0.68, ~1e6 N m) exceeds the zero-roll flap authority (+-8.4e5 N m); why one tick decides the sign is not determined. Chatter of the commands at that freeze remains.
  - Next step when picked up: the low Mach part (Mach 2-6, where the geometry set flies too slow and the aft flaps rise twice as fast as the frames), not a fit of the table. Open, not closed by the alpha the frames support.
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
  0.71 s) was re-flown in the vehicle plan's Task 6: booster catch campaign 21 of 21, ship catch campaign 20 of 21
  (`docs/subsystems/ship-entry-return.md`, `ascent-staging.md`). The lunar campaigns flew it in Task 6e: aerocapture 21 of 21; direct above.
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
