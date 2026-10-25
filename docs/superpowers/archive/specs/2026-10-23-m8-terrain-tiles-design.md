# M8 part 1: terrain tiles, Earth DEM, geoid and calm sea - design

Date: 2026-10-23. Status: decisions taken with the user 2026-10-23; awaiting final approval.

## Intent

The ground around the places where vehicles launch and land must look real in the external view (desktop and VR),
and the physics must stand on the same ground. Far away and high up the existing coarse Earth is enough: the
high-resolution data are islands that the camera leaves while climbing (user: "um die Start- und Landestellen herum
muss es hochaufloesend sein; ab einer bestimmten Hoehe duerfen die Kacheln wegfallen; muss eben entsprechend
aussehen"). Much of the final look comes with the atmosphere / diffuse light (next M8 part); this part delivers the
base: geometry, ground imagery, sea surface, and a consistent physical ground.

The user ordered M8 to start with the physics-relevant parts (milestone order 2026-10-23). Target hardware: RTX 4090,
PC-VR at 90 Hz stereo. Disk space and download size are not a constraint (user: "genug Platz").

## Goals

1. **Physical Earth terrain.** The Earth gets a terrain model like the Moon's (`environment/lunar_terrain`): height
   grids that blend into each other, hooked into `ReferenceBody::surface_height_m`. Everything that already asks the
   body for its surface (legs, impact check, `altitude_terrain_m`, leg surfaces via `TerrainField`) then sees the real
   ground near the sites. `FlightBody::ground_height_at` stops returning the flat per-body height on Earth where
   terrain exists.
2. **Geoid and sea level.** The sea surface comes from a geoid model (EGM2008, free, NGA) instead of the ellipsoid
   (capsule splashdown today at 0 m ellipsoidal) or the fixed `drone_ship_sea_height_m = -35 m`. Ellipsoidal ground
   height = orthometric DEM height + geoid undulation. Tides, mean dynamic topography: not modelled, documented.
3. **Real pad heights (user: yes, 2026-10-23).** The heights in `config/launch_sites.toml` are hand estimates labelled
   "ellipsoidal, not geoid-corrected" (Starbase +10 m; true ellipsoidal ground there is ~-20 m because the geoid lies
   ~25-30 m below the ellipsoid on the Gulf and Florida coasts). Pad, tower base and landing-zone heights are derived
   from the terrain + geoid data (provenance [D]) instead of being typed in. Pad structures stay ours (launch mount
   height, tower, LZ pad) and sit on the derived ground; guidance keeps aiming at the defined site height.
4. **Re-validation.** Shifting every site by tens of metres touches the reference flights. Flight 14, CRS-35, the
   booster catches, Falcon 9 RTLS and drone-ship landings, ISS-from-pad, the capsule splashdown and the refuelling
   reference flights are re-run against their documented tolerances; where a number moves, it is re-baselined with the
   reason written down (`docs/subsystems.md`). No tolerances are widened to hide a physics change.
5. **Tile renderer.** A streamed tile system in the game draws terrain + imagery by camera height and distance, with
   smooth transitions, loaded in the background, no visible popping and no jitter (floating origin: tile vertices
   relative to the tile centre in float64, converted to float32 last). It replaces the interim `GroundPatch`.
6. **Resolution levels (my proposal, accepted as the working basis):**

   | Level | Area | Imagery | Heights |
   |---|---|---|---|
   | 1 | ~10 x 10 km around each pad / landing zone (Starbase OLP-1; LC-39A, SLC-40, LZ-40) | <= 1 m (USGS NAIP, ~0.6 m) | 1 m lidar bare-earth DEM (USGS 3DEP) |
   | 2 | ~200 x 200 km around Starbase and around the Cape (whole coast visible on approach) | 10 m (Sentinel-2 cloud-free mosaic) | Copernicus GLO-30 |
   | 3 | rest of the Earth | existing day/night textures | WGS84 ellipsoid + geoid for the sea |

   Above a few hundred km altitude all tiles are off; only level 3 is drawn.
7. **Calm sea.** The sea surface is generated in the shader wherever there is sea (coastline from the terrain data /
   a land-sea mask): gentle waves, sun glint, reflection. It works everywhere, because drone ships are placed at
   launch time anywhere in their service area (`config/drone_ships.toml`), so the sea cannot be pre-tiled. The
   drone-ship deck stays fixed (user decision 2026-10-23).
8. **Moon landing site visible (user: yes).** The same renderer draws the existing lunar site tile (Mons Mouton,
   `data/terrain/mons_mouton_im2.*`, 20 m) so the landing is seen on the terrain the physics already uses. Around it the
   whole Moon gets coarse relief from the global grid the physics already loads (`data/terrain/ldem_16.*`, LDEM_16:
   16 px/deg, ~1.9 km; downsampled further for distant views where needed) instead of the smooth sphere (user
   2026-10-23: "aussenherum den Mond mit groben Hoehendaten ... oder runterrechnen"). Physics for the Moon does not
   change.
9. **Data pipeline.** `tools/data_pipeline` fetches the sources, cuts tiles, converts to runtime formats, records
   each source with its licence in `sources.toml`; `CREDITS.md` regenerated. Physics and rendering read the same
   height files.

## Non-goals

- Atmosphere / scattering, clouds, exhaust plumes, entry plasma, vehicle models: later M8 parts.
- Moving sea and deck motion (heave/roll/pitch, sea-state parameter as a fidelity flag, onboard refusal in rough sea):
  a separate small part directly after this one (user 2026-10-23).
- Trees, 3D buildings: imagery shows them flat. The 3DEP DEM is bare earth; towers and pads are our own models.
- High-resolution terrain elsewhere on Earth (other pads, the Pacific drone-ship port, splashdown zones beyond the
  sea surface), lunar terrain finer than LDEM_16 outside the site tile: not now.
- Wind and weather: not part of this.

## Anti-goals (would be a failure even if it works technically)

- Rendered ground and physics ground disagree (vehicles float above or sink into the drawn surface).
- VR frame rate below 90 Hz on the RTX 4090, visible tile popping, or horizon / ground jitter.
- Silently changed validation numbers or widened tolerances.
- Data with a licence that does not allow free redistribution in the project (only free data, `sources.toml`).

## Constraints

- Architecture per CLAUDE.md: terrain sampling and geoid in `core/environment` (no Godot dependency, float64); the
  bindings expose heights / tiles thinly; the game only renders. External libraries (e.g. a GeoTIFF reader) wrapped in
  one module or, better, kept in the Python pipeline so the core reads simple raw grids + TOML headers like the lunar
  tiles.
- Height conventions in names: `height_ellipsoid_m`, `height_orthometric_m`, `geoid_undulation_m`.
- Every physical model gets unit tests and a validation case with a documented tolerance (e.g. geoid undulation vs
  NGA reference points; DEM heights vs published survey points / the `[[check]]` points of the tile headers).
- Test workflow: run the tags the change affects (here: environment, contact, the recovery / ascent / ISS / capsule /
  refuelling campaigns whose numbers move). Long runs to the background with logs.
- Godot `game/`: text formats only; runtime data under `data/` (git-ignored), fetched by the pipeline.
- UI text German, code/docs English.

## Decided with the user

- High resolution only around launch and landing sites; tiles fade out with altitude.
- Sea around drone ships: real-looking water; calm sea, fixed deck now; deck motion as its own next part.
- Disk / download size: no limit needed.
- Geoid + derived pad heights included, accepting the re-validation of reference flights.
- Lunar site terrain rendered by the same renderer.

## Two plans (user 2026-10-23)

- **Plan A - physics first:** Earth terrain in the core, geoid, global coarse relief + land/sea, derived site heights,
  re-validation of the reference flights, the height part of the data pipeline (DEM, geoid, ETOPO). The interim
  `GroundPatch` stays and only follows the new heights (pad plateau at the baked height, sea at the geoid) so rendered
  and physics ground agree in between.
- **Plan B - rendering:** streamed tile renderer (Earth levels 1-2, lunar site tile + LDEM_16 relief), imagery
  pipeline (NAIP, Sentinel-2), calm sea shader; `GroundPatch` removed.
- `/clear` after each plan.

## Settled after the builder check (my calls, open to change)

- **Site heights are baked, the pad is flat.** The pipeline derives each site's ground height (DEM + geoid, [D] with
  source) and writes it into `config/launch_sites.toml` (committed); the core does not sample the DEM to place a
  site. Under each pad, tower footprint and landing zone the ground is a flat plateau at that height (real pads are
  built-up concrete plateaus), blended into the DEM over a short apron; physics and renderer flatten identically.
- **Tests run without the downloaded data.** The reference-flight campaigns use the baked site heights and a coarse
  geoid committed as a test fixture (small grid over the launch / drone-ship / splashdown areas, interpolation error
  documented). Tests that read the full DEM / geoid / imagery files are tagged `[data]` like the lunar terrain tests.
- **Physics ground outside the tiles.** A global coarse relief (NOAA ETOPO 2022, 60", public domain - licence to be
  confirmed) gives land heights and the land/sea decision everywhere: impacts, crew-escape touchdowns, ditching
  inland vs at sea. It replaces the per-site `site.sea` shoreline approximation in the core (shoreline stays as config
  only where still needed by a planner). Sea = geoid where the relief is below sea level and connected to the ocean
  (lakes: not modelled, documented). Rendering of level 3 is unchanged (textures).
- **Out-of-tolerance after the shift.** A catch or landing that fails after the height change is a bug in this part's
  scope (fix guidance / physics). A comparison against webcast telemetry whose expected value moves only because the
  site moved is re-baselined with the reason. The builder checks which reference altitude each comparison uses (above
  pad / MSL / ellipsoid) and converts consistently.

## Left to the builder (my calls, open to change)

- The level sizes (10 km / 200 km) and the fade altitudes, within the VR budget.
- Tile scheme (quadtree / chunked LOD, skirts or morphing), streaming mechanics, GPU texture format (whatever Godot
  4.7 loads at runtime without the editor import step), mesh resolution per level.
- Geoid grid resolution (EGM2008 2.5' or 1'), interpolation method.
- Vertical datum of 3DEP (NAVD88 / GEOID18) vs GLO-30 (EGM2008): convert to ellipsoidal consistently; document the
  residual offset between the datums at the seams.
- Which imagery product for level 2 (EOX s2cloudless only if its licence allows redistribution - some years are
  CC BY-NC-SA; otherwise an own cloud-free composite from Sentinel-2 L2A); the licence check decides.
- Coverage fallback: if 3DEP 1 m or NAIP does not cover a site area, use the next coarser source and note it.
- Water shading details (normal maps, Fresnel, glint) and the land-sea mask source.
- Whether the old flat `ground_height_m` stays as a fidelity flag for comparison runs (recommended: yes).
- Visual checks via `tests/render_check.gd` (views at Starbase tower, LC-39A, LZ-40, a drone ship at sea, Mons Mouton)
  and a VR frame-time check by the user.
