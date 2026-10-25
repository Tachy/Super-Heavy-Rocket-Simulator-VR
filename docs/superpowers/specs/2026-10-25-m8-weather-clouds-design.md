# M8 weather and clouds - design

Date: 2026-10-25. Milestone order: the next M8 part after the atmosphere (VR check passed 2026-10-25). The user's
words: "Echtes Wetter zum Startzeitpunkt, Wolken beim Aufstieg durchfliegen"; the weather should act on the flight too,
"zuerstmal nur sehen... Physik später"; for future dates "Vorhersage, sonst typisches Wetter"; volumetric clouds
"überall" (not only at the launch sites); a runtime download of weather data is "in Ordnung"; matching the real
satellite pattern "muss nicht ... aber wenn es ginge, wäre es natürlich genial"; rain, fog, thunderstorms, haze "wenn es
ginge... klar!"; the four-part order below "passt". Every decision not quoted from the user is Claude's call, marked
**[call]**. Background: the user's M8 note of 2026-10-12 (master plan, "Notiz des Nutzers"; memory
project-atmosphere-clouds-note): GIBS / GOES / Blue Marble clouds, a cloud shell with ground shadows from orbit,
volumetric clouds for launches, the VR frame budget (LUTs or few raymarch steps), the float64 camera-to-Earth vector,
no separate cloud-shell mesh in orbit (z-fighting / alpha sorting).

## Intent

Today every flight starts under a clear sky with one fixed haze (AOD 0.1) everywhere on Earth. The user wants the real
weather of the chosen moment: replaying a flight on its real date shows the clouds of that day, starting now shows
today's weather, a date in the next days shows the forecast, anything further out a typical weather for the place and
season. The clouds are three-dimensional everywhere on Earth: a rocket climbs through the deck, a returning ship or a
Dragon under parachutes falls through it, and from orbit the same clouds form the pattern over the Earth with their
shadows on the ground. The weather is a physical field (cloud water, ice, cloud fraction, temperature, wind, pressure
levels with their heights) from numerical weather models; the clouds' look follows from it by physics (optical depth
from water content and droplet size, Mie-like phase function, multiple scattering, lit by the same Sun and sky as
everything else), not from tuned shapes and colours.

## The four parts (user-approved order)

1. **Weather data and clouds** (this document's detailed scope, one plan): weather data for any epoch (archive,
   current / forecast, typical), the weather field in the core, volumetric clouds everywhere, cloud shadows, light
   under clouds, the weather source in the UI.
2. **Weather phenomena**: rain and snow (visible shafts, wet ground / sea surface look optional), fog and low stratus at
   the ground, thunderstorms with lightning, weather-dependent haze (aerosol optical depth, e.g. from CAMS, replacing the
   fixed AOD 0.1 of the atmosphere tables; the user ruling of 2026-10-25 "keep AOD, revisit with weather" lands here).
3. **Satellite pattern**: where geostationary imagery exists (GOES-East / GOES-West, public domain, 10 min, ~2 km,
   since 2017; covers Starbase, the Cape and the Pacific / Atlantic splashdown areas), the 2D cloud pattern and cloud
   top heights are fitted to the satellite image so the clouds match a webcast or photo; elsewhere part 1 stays.
4. **Weather physics**: winds aloft and gusts acting on the flight (aero uses air velocity), pressure / temperature /
   density from the weather instead of US76 (fidelity flag), launch commit criteria and weather scrubs (wind, lightning
   rules), drone-ship sea state consistent with the weather's wind.

Parts 2-4 get their own brainstorm / spec / plan later. Part 1 must not block them: its data layer is built so more
variables can be added per source (wind for part 4) and the 2D cloud-pattern input can be replaced (part 3).

## Part 1 goals

1. **Weather sources by epoch [call, sources verified 2026-10-25].** The epoch is the scene's UTC (time panel /
   mission start / time warp). The source is picked automatically:
   - **Archive: ERA5** (ECMWF reanalysis, hourly, 0.25 deg, 37 pressure levels, 1940 to ~6 days before today) from the
     public, account-free ARCO-ERA5 Zarr store on Google Cloud
     (`gs://gcp-public-data-arco-era5/ar/full_37-1h-0p25deg-chunk-1.zarr-v3`, HTTPS
     `storage.googleapis.com/gcp-public-data-arco-era5/...`; chunks are one global level stack per variable and hour,
     float32, blosc/lz4). Its `.zattrs` give the end of the final ERA5 (`valid_time_stop`, ~3 months back) and of the
     preliminary ERA5T (`valid_time_stop_era5t`, ~6 days back) - both are used. A chunk cannot be cut by level or
     region, so the download is whole global chunks: measured by the builder check (2026-10-25), compressed per hour:
     cloud cover 28 MB, liquid 20 MB, ice 20 MB, temperature 85 MB, geopotential 80 MB, u / v wind 117 / 122 MB.
     **User decision 2026-10-25 ("alles ok"): ~150 MB per hour of weather** - variables on pressure levels
     `fraction_of_cloud_cover`, `specific_cloud_liquid_water_content`, `specific_cloud_ice_water_content`, `temperature`;
     single level `surface_pressure` (small) and the static `geopotential_at_surface` (once). **No geopotential and no
     wind in part 1**: the level heights are integrated hypsometrically from the surface (surface pressure and height,
     level temperatures; ERA5 derives its own geopotential the same way - the dry-air form without humidity is the
     builder's call if the error stays within the Validation tolerance, else `specific_humidity` is added); the wind
     comes with part 4 (its download design is part 4's problem, e.g. only when the physics needs it). Licence: Copernicus licence (CC-BY-4.0-like: reuse and derivatives allowed
     with attribution "Contains modified Copernicus Climate Change Service information", a licence link and a note of
     modifications) - the builder confirms the current text and records it in `sources.toml` / `CREDITS.md`.
   - **Recent days and forecast: NOAA GFS** (0.25 deg, analyses and forecasts to 384 h, hourly to 120 h then 3-hourly;
     public domain) from the AWS open-data bucket `noaa-gfs-bdp-pds` (`gfs.YYYYMMDD/HH/atmos/gfs.tHHz.pgrb2.0p25.fFFF`,
     GRIB2 with `.idx` inventories, so only the needed records are fetched by HTTP byte ranges). Variables on
     pressure levels: `TCDC`, `CLMR`, `ICMR`, `HGT`, `TMP` (verified in a 2026-09-20 inventory; `UGRD` / `VGRD`
     exist for part 4). GFS records can be fetched per level, so the GFS download stays well below the ERA5 one.
     Used for epochs after the end of ERA5T up to the latest run's forecast horizon; for a past epoch the analysis /
     shortest-lead forecast of the cycle covering it, for a future epoch the latest available cycle.
   - **Typical weather** beyond the forecast horizon, without network, or when a download fails [call: a real analog,
     not a climatological mean - a monthly-mean cloud fraction is a smooth haze that no real sky ever shows]: the ERA5
     field of the same calendar day and UTC hour from a fixed recent reference year, so the place, season and time of
     day are right and the clouds look like real clouds. An offline set ships with the game (generated by the data
     pipeline: e.g. one analog day per month at 00/06/12/18 UTC, reduced resolution and levels) so it works without any
     download; online, the full-resolution analog of the exact day may be fetched instead. The size of the shipped set is
     the builder's call within the package's data-part limits (stated in the plan, ~100-300 MB [E]).
   - The UI always says which weather is shown (German, e.g. "Wetter: ERA5, 2024-04-08 18:00 UTC", "Wetter:
     GFS-Vorhersage vom 25.10. 00 UTC, +54 h", "Wetter: typisch (Analogtag 2025-04-08)", "Wetter: aus") and a switch turns
     the weather off (clear sky as today) [call].
2. **Download at runtime, decoding in the core [call].** The game downloads (Godot's HTTP client on a worker thread:
   no networking code in `core/`, which stays a pure simulation library), into a cache `data/weather/` (one file per
   source / cycle / valid time, a size cap with least-recently-used eviction [E: a few GB]); the next snapshot of a
   running scene is prefetched in the background. The core decodes the raw files: a new adapter module for the external
   decoders (Zarr chunk decompression - c-blosc / lz4 -, GRIB2 decoding - e.g. NCEPLIBS-g2c or ecCodes, the builder
   picks by licence and footprint), wrapped behind an interface and included nowhere else, exactly as CSPICE is. The
   data pipeline (`tools/data_pipeline/`) can prefetch the same files offline (packaging, tests, the offline analog
   set). Download size: ~150 MB per ERA5 hour (user decision above); the builder documents the measured GFS size. Memory:
   two decoded snapshots are held for the blend; the builder keeps them compact (e.g. only the levels up to the
   highest clouds, converted to the render form on decode) and states the footprint.
   **Time warp (user decision 2026-10-25):** while the downloads keep up, the weather follows the hourly snapshots; at
   higher warp it steps to sparser snapshots (every 3 h, then 6 h); beyond that (days of a lunar mission or the
   refuelling campaign) the last weather stays and the UI says so (e.g. "Wetter: Stand 18:00 UTC"); when the warp drops
   it is brought to the current time (blended in, Anti-goals). The warp thresholds are the builder's call.
3. **The weather field in the core [call: `environment` module, e.g. `weather_field.*`].** One immutable snapshot per
   source / valid time on the source grid, shared like the terrain (`sim::shared_*` pattern), with:
   - level heights from the geopotential (geopotential height -> geometric height -> ellipsoidal via the EGM2008 geoid
     the core already has), so cloud bases and tops sit at their real heights over the WGS84 surface;
   - cloud fraction, liquid and ice water content (kg/kg -> kg/m^3 with the level's air density from pressure and
     temperature) and temperature per cell and level (wind: part 4);
   - time interpolation between the two bracketing snapshots: a smooth blend in place (no wind, so no advection in
     part 1 - user decision 2026-10-25; drifting clouds come with the wind in part 4);
   - a query interface by geodetic position and epoch that part 4 can call from the flight physics later (this part
     does not call it from `FlightSession`: no flight physics changes);
   - the render data: the fields resampled into GPU-friendly textures (e.g. a global 2D "weather map" of cloud cover /
     base / top / water path per layer plus a 3D density volume near the camera), produced in the core, uploaded by the
     game, the same way the atmosphere tables are.
4. **Cloud optics from physics [call].** Extinction from water content and effective droplet radius (beta = 3 LWC /
   (2 rho_w r_eff); liquid r_eff ~10 um, ice ~25-30 um [E, sourced in the plan]); single-scattering albedo ~1 in the
   visible; a phase function that approximates Mie for cloud droplets (e.g. Jendersie & d'Eon 2023's approximate Mie,
   or a two-lobe Henyey-Greenstein fit, with its citation); multiple scattering by a documented approximation (e.g.
   Wrenninge's octaves or Hillaire 2016's multi-scattering approximation) whose plane-parallel albedo is checked against
   a reference (Validation). A CPU reference implementation in the core (like the atmosphere's CPU evaluation) for the
   tests; the shader is its port - change both together.
5. **Sub-grid cloud shapes, coverage-preserving [call].** 0.25 deg (~28 km) cells cannot show single cumuli. The shader
   adds procedural detail (noise shapes) with the rule that the covered area fraction at a level equals the field's
   cloud fraction and the in-cloud water content equals the grid-mean content divided by that fraction, so the large
   scale (where, how high, how thick, how much) is the model's and only the small scale is invented. The cloud type's
   look (cumuliform vs stratiform vs cirrus) follows from the field (vertical extent, temperature, ice share), not from a
   per-site setting. No visible cell edges: the field is interpolated smoothly across cells.
6. **One cloud model from the pad to the Moon, no pop [call].** Near the camera (below and inside the cloud layers,
   at the pad, during ascent / entry / parachute descent, anywhere on Earth) the clouds are raymarched as volumes and can
   be flown through; far away and from orbit the same field is shaded more cheaply (e.g. column optical depth and top
   height from the weather map, with self-shadowing / top lighting), in the Earth's own shaders (no separate shell mesh,
   per the user's note). The hand-over is continuous: the same optical depth and light at the switch distance, no
   altitude where the clouds change look visibly.
7. **Clouds in the light [call].** Clouds are lit by the transmitted Sun and the sky of the atmosphere model (the
   `atmosphere.gdshaderinc` functions; sunsets colour cloud bases), seen through the atmosphere (aerial perspective),
   and at night by the Moon light that exists since the night-vision work. Their shadows fall on terrain, sea, vehicles,
   tower and drone ship (a cloud-shadow / transmittance map toward the Sun). Under a cloud deck the ambient / sky
   irradiance on the ground and on vehicles drops by the deck's transmittance, and the vehicle Sun light and the eye
   adaptation follow the Sun transmitted through the clouds at the followed body (the core's CPU evaluation gains the
   cloud transmittance). From orbit the clouds brighten the Earth disc and its limb as they do in photos. Earthshine on
   the Moon keeps its current model in part 1 (follow-up: real cloud albedo).
8. **Numerics and the floating origin**: as the atmosphere does - camera-to-Earth vector and altitude differenced in
   float64 on the CPU, cloud-layer heights relative to the WGS84 surface, no float32 differences of large radii; no
   jitter of the cloud deck at the pad or in orbit.
9. **VR budget (user decision 2026-10-25: 90 Hz headset, clouds may add up to 3 ms).** The frame at 90 Hz is 11.1 ms;
   the performance gate (terrain_perf.gd, 1440p desktop proxy) becomes p99 <= 11 ms with clouds on (was 8 ms; today
   ~7.7 ms without clouds), max <= 16 ms unchanged. Quality levels (raymarch steps, volume resolution, temporal
   accumulation over frames) with a desktop and a VR setting; the gate is measured with the VR setting. No per-eye
   differences in the cloud shapes (stereo must agree; temporal accumulation must not make the eyes disagree).

## Non-goals (part 1)

- Rain, snow, fog, lightning, weather-dependent haze (part 2); satellite-matched patterns (part 3); any change to
  flight physics, the US76 atmosphere, launch rules or the sea state (part 4).
- Clouds on other bodies; noctilucent clouds; contrails / launch exhaust clouds (plume work in M8 later).
- Earthshine from real cloud albedo (follow-up); city light glowing on low clouds (part 2).
- High-resolution regional models (HRRR etc.): the global 0.25 deg sources everywhere [call: one consistent pipeline
  worldwide; HRRR could be a later refinement for the US sites].

## Anti-goals

- Tuned looks: no hand-picked cloud colours, opacity curves or "cloud amount" sliders that bypass the field; a wrong look
  is fixed in the optics or the data, not by a fudge (as in the atmosphere spec).
- Invented weather: shapes may be procedural, but where, how high, how thick and how much cloud there is comes from the
  data of the shown source - never random.
- Visible switches: no pop between the volumetric and the far-field clouds, between snapshots, or between sources when
  the time warp crosses from ERA5 to GFS.
- A blocking game: a missing download never stalls the flight; the scene shows the typical weather (or keeps the last
  snapshot) and states it, and swaps in the real data when it arrives (blended, not popped).
- Breaking the VR budget or stereo consistency.
- Physics in GDScript; networking in `core/`; an external decoder included outside its adapter module.

## Validation (documented tolerances)

Core (`[weather]` tag [call]):
- Decoding: small committed fixtures (a few cells cut from a real ERA5 Zarr chunk and a GFS GRIB2 record) against
  reference values written by an offline Python tool (xarray / cfgrib, like `gen_socp_fixtures.py`): exact to float32.
- Level heights: the hypsometric heights vs ERA5's own `geopotential` (offline fixture, a few columns from tropics to
  high latitudes) within ~30 m [E] up to 100 hPa; geopotential height -> ellipsoidal height vs an independent
  computation (1 m); GFS `HGT` used directly, checked the same way.
- Source selection: epochs before / inside / after ERA5, ERA5T, the GFS horizon and offline map to the documented
  source; the boundary is continuous in the interpolated field (no step larger than the snapshot difference).
- Cloud optics: extinction vs the formula; the CPU multiple-scattering approximation's plane-parallel slab reflectance
  vs a two-stream / adding reference (or published DISORT values) for optical depths 1-50 within a documented tolerance
  (~10 % [E], stated with the reason); transmittance of an overcast deck (tau ~10-30) giving ground illuminance in the
  measured range for overcast skies (roughly 10-30 % of clear-sky [S, cited]).
- Coverage preservation: the procedural detail's covered fraction over a cell matches the field's cloud fraction within
  a few percent.
- Real-date plausibility: for a documented historical epoch (e.g. a flight with known weather) the field's total cloud
  cover and cloud base at the site agree with ERA5's own `total_cloud_cover` / `cloud_base_height` (consistency) and,
  where a METAR / sounding is cited, roughly with it (stated tolerance; reanalysis vs point observation).

Game (GPU, render_check.gd / a GPU check):
- Shader vs core: cloud transmittance and radiance at sample pixels vs the CPU reference within a few percent.
- Visual set read as PNGs: a historical overcast day and a clear day at Starbase from the ground; an ascent through a
  deck (below, inside, above); the Cape from 100 km and the ISS orbit over a frontal band; sunset light on cloud bases;
  cloud shadows on the Cape coast; the Earth from the Moon with clouds; the same epoch from ERA5 vs the satellite image
  of that time compared by eye (pattern position, not shapes).
- Performance gate (Goal 9); VR check by the user (pad under clouds, ascent through clouds, orbit).

## Left to the builder

The library choices for blosc / lz4 and GRIB2 (licence check, vendoring in `external/` like CSPICE), the exact cache
format and cap, the render-data layout (weather map resolution and layers, near-camera volume size and update), the
raymarching scheme and step counts, noise shapes, the multiple-scattering approximation and phase-function fit (with
citations), the reference year and the size of the offline analog set, the prefetch policy, the German UI wording and
its place (time panel / overlay), how the extension exposes snapshots and textures (`StarsimCore` methods), and how
the switch persists. Docs: a new "Weather and clouds" section in `docs/subsystems.md`, CLAUDE.md (architecture list:
the weather field and its adapter; known issues), `docs/ansichten.md` where views change, `sources.toml` / `CREDITS.md`
for ERA5, GFS and the decoder libraries.
