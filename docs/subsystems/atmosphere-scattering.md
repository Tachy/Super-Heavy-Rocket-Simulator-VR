# Atmosphere scattering and surfaces (M8)

History and the reasons behind superseded decisions: `docs/history/atmosphere-rendering.md`.

Plan `docs/superpowers/archive/plans/2026-10-24-m8-atmosphere.md`, spec
`docs/superpowers/archive/specs/2026-10-24-m8-atmosphere-design.md`.
Other atmosphere pages: `eye-adaptation.md`, `atmosphere-eclipse-refraction.md`, `atmosphere-performance.md`.
One
physical model of the Earth's atmosphere for every viewpoint - pad, ascent, orbit, the Moon - so the sky, the
limb
and the haze change continuously with the camera and the Sun; nothing is tuned. Flight physics is untouched
(the
vehicles still fly through `Us76Atmosphere`).

## Model

- **Scattering** (core, `environment`): a float64 port of Bruneton's "Precomputed Atmospheric Scattering: a New
  Implementation" (2017, BSD-3; ported files carry his notice, CREDITS.md ships it in full) in precomputed-luminance
  mode: 15 wavelengths 360-830 nm in five passes of three, folded with the CIE 1931 2-degree functions into XYZ and
  linear sRGB (D65), no white balance. Inputs `config/atmosphere/earth_optics.toml` + spectra in
  `atmosphere_spectra.hpp`: Rayleigh from the refractive index of air (Peck & Reeder 1972) with the King factor
  (Bucholtz 1995), the US76 number density as profile; ozone cross sections (Serdyuchenko 2014 via Bruneton) on the AFGL
  US standard profile, 300 DU [S]; aerosol AOD 0.1 at 550 nm, Angstrom 1.3, single-scattering albedo 0.9, scale height
  1.2 km, Cornette-Shanks g 0.8 [E]; solar spectrum ASTM G-173 ETR (scaled by 1 / d_AU^2 at run time); ground albedo
  0.1 [E]; top 100 km [call], model ground `ground_altitude_m` -150 m [E] (below the geoid minimum -106 m and every pad;
  cameras and points below are clamped onto it).
- **WGS84.** Evaluated in a space where the ellipsoid becomes the sphere of radius a (polar axis x a/b = 1.00336):
  ground matches the ellipsoid (test: < 1 mm at 0/30/60/90 deg), optical paths off by at most 0.34 %. Rays and the
  Sun map by that stretch, normals by its inverse transpose; the direct-sun cosine is taken in true ITRF geometry. CPU
  and shaders use the same mapping.
- **Multiple scattering.** Five orders plus a geometric tail of the higher orders (Hillaire 2020, Eq. 9): tail = last
  order x q / (1 - q), q = last / previous per texel and wavelength, capped at 0.75. Fidelity flag
  `multiple_scattering_tail` in `earth_optics.toml` (true; false truncates like Bruneton; part of the input hash; test:
  on in the shipped config and it changes the tables). 5 vs 7 orders with the tail: within 0.20 % (sky illuminance
  0.03 %).
- **Tables and cache.** Transmittance 1024 x 1024, scattering (Rayleigh + all multiple orders), single Mie and single
  Rayleigh (static clouds Task 9b: the first order alone, for the cloud shadows on the air, `weather-clouds.md`) 8 nu x
  32 mu_s x 128 mu x 64 r, irradiance 64 x 16; mu_s_min = cos 120 deg (twilight to -18 deg for the star threshold).
  Precompute 61-62 s on 24 threads (once per input hash; `atmosphere_tables_code_version` 4), file
  `data/atmosphere/earth_atmosphere.bin` (117.5 MB, format version 3: header with magic, version, input hash, sizes
  and parameters, five float32 arrays, FNV-1a 64 checksum verified before
  parsing; temp name then rename; a damaged, stale or missing file is recomputed, a failed write still returns the
  tables). The input hash covers the config, the table sizes, the stored spectral / optics data and
  `atmosphere_tables_code_version`. CLI `build/core/atmosphere_tables.exe [--config] [--out]`; `tools/package.ps1` runs
  it, so installed games ship the cache. The game loads the cache on a worker (`StarsimCore.load_atmosphere`; first run
  computes and shows "Atmosphäre wird berechnet (einmalig, etwa eine Minute) ..." in the time panel). GPU formats:
  transmittance RGBAF (grazing values ~1e-5 are half-float subnormals), scattering / Mie / single Rayleigh /
  irradiance RGBAH; ~66 MB VRAM. The single Rayleigh table is read only by the cloud pass (`atmosphere_single_rayleigh`
  declared in `cloud_pass.gdshaderinc`, not in this include).
- **Shader include** `game/shaders/atmosphere.gdshaderinc` (port of the CPU code, globals in project.godot
  `[shader_globals]` `atmosphere_*`): `atm_sky`, `atm_to_point*` (aerial perspective; camera-relative offsets for near
  points), `atm_sun_at` / `atm_sky_light` / `atm_sun_and_sky`, `atm_sky_to_far_object` (Moon, Sun), `atm_fog`
  (vehicles). Floating-origin numerics: the camera's Earth-local position arrives as float32 hi + lo
  (`atmosphere_camera_hi/lo`), its altitude above the model ground differenced in float64 on the CPU
  (`atmosphere_camera_altitude_m`); rho = sqrt(h (2R + h)), discriminants as (r mu)^2 - rho^2 and (T - r)(T + r), a
  point's height along a ray as h + d(d + 2 r mu) / (r_d + r). Cameras above the top enter through the ray's closest
  approach (the limb is right from the Moon).
- **Where it is used.** Sky: `sky.gdshader` (Environment BG_SKY, every pixel from the camera's true position; the
  radiance cubemap also draws the model ground, so Godot's ambient and reflections carry sky light, ground bounce and
  earthshine; PROCESS_MODE_INCREMENTAL, radiance 128; see "Radiance cubemap"). Earth: `earth.gdshader` (scaled space; with the far-field
  clouds `earth_clouds.gdshader`), `terrain_earth.gdshader`, `water.gdshaderinc` (the sea's reflected sky is the
  model's sky along the reflected ray; a ray reflected below the horizontal takes its upward mirror [E]): Sun
  transmitted to the point x cos + the sky irradiance on the normal, then transmittance + in-scattering to the camera;
  city lights x T. Moon, Sun disc: x T + the in-scattered sky. The Sun disc's colour is the model's TOA solar
  spectrum (`atmosphere_sun_illuminance_lux`), normalised to unit luminance. Its brightness is physical on the scene scale
  (`World.sun_disc_scene_value`: the centre 8.67e4, the disk's mean 6.9e4; scaled as a whole to a largest channel of
6e4 where T x the exposure gain > 0.69: `weather-clouds.md`, the Sun through cloud). Vehicles, tower, pad, drone ship:
  `lit_aerial.gdshader` (`LitAerial.material`): Godot's
  PBR plus aerial perspective through FOG (per-channel T split: t = max(T), FOG.a = 1 - t, FOG.rgb = S / (1 - t), and
  T / t on albedo and emission; only the dielectric specular takes the scalar); the tower lattice through `atm_fog`.
  Plumes (`plume.gdshader`, additive): x T only [call]. Flight trail, orbit lines and labels are UI markers, not
  attenuated (past the rod step unchanged: `vision.gdshaderinc`; map labels are `MapLabel` TextMeshes, the Moon's orbit
  line `display_colour.gdshader`).
- **Sun light.** One DirectionalLight: colour and energy per channel = TOA solar RGB x the core's
  `atmosphere_sun_transmittance` x luminance scale / pi x the Moon's visible fraction, at one point: the followed
  vehicle (vehicle focus), else the camera (`Atmosphere.sun_light_at`, `world.gd::_update_sun_light`). Near the Moon
  the visible fraction is `Atmosphere.sun_visible_near_moon` (one combination for the vehicles and the eye adaptation):
  the terrain's fraction (`MoonSunHorizon.sun_visible_fraction`, at every height within `MOON_TERRAIN_SUN_MAX_DISTANCE_M`
  = 66 000 km of the Moon [E], no height switch; the same geometry as `terrain_moon.gdshader`: the core's
  `horizon_elevation_rad` march in the two sectors k * 2 pi / 16 around the Sun's azimuth, linear between them, the exact
  circular segment of the solar disc above that horizon) x the Earth's disc in front of the Sun (lunar eclipse; the
  terrain knows only the Moon). Exact pre-check: from height h over the reference sphere no relief rises above
  -acos((R + max_height) / (R + h)) (`StarsimCore.terrain_max_height_m`, 10 686 m; none at or below it); a Sun disc
  wholly above that bound is 1 without a march (the mean sphere lies below it too). A point under the terrain (vehicle in
  contact) is marched from 0.5 m above the surface [E]. No lunar terrain data (NAN) or beyond the distance: the
  mean / ground-sphere value `Atmosphere.moon_visible_fraction` as before. The step at the 66 000 km gate is about
  4 % of the Sun's disc (the relief ~2 % plus the straight terrain horizon against the sphere's limb ~1.9 %; 0.009 deg
  is the limb part). Known limits at the gate: the vehicles' path jumps there by the Earth's penumbral share (the
  core's `solar_eclipse_fraction_gcrf`, used beyond the gate, knows only the Moon; a rare configuration); inside the
  gate the Earth's penumbra counts twice with the atmosphere transmittance, as the eye path's documented call. `world.gd::_moon_terrain_sun_fraction`; the camera's value is computed once per frame (eye adaptation and,
  without a followed vehicle, the light). Cost, main thread, one fraction (two marches) over rough highlands with the
  Sun at the horizon: 0.02 / 0.04 / 0.05 / 0.08 / 0.15 ms at 2 m / 5 km / 20 km / 100 km / 1000 km: uncached. Tests:
  `tests/moon_sun_test.gd` (headless: bound exact over 7 sites x 5 heights x 16 sectors, rim sweep, height sweep 2 m - 200
  km continuous, Earth umbra 0 / penumbra = 1 - occulted fraction, orbit, NAN, below-ground clamp) and the GPU
  `atmosphere_check.gd` (IM-2 2 m above ground, Sun 1 deg below / above / at the horizon: 0 / 1 / 0.5; 1551 m below the
  mean radius, Sun 60 deg up: light = the open sky's). Clouds (Task 9, `weather-clouds.md` "Light"):
  x the direct beam's cloud transmittance at that point (`Atmosphere.cloud_light_at`, the Moon's light likewise), and
  the sky's radiance cube x the light under the clouds over the clear sky's there
  (`Atmosphere.cloud_radiance_factors`, by channel, one for its sky, one for its ground; exactly 1 with weather off).
  Earth surfaces (`terrain_earth.gdshader`): the Sun
  through the cloud shadow maps, the Sun's light diffused through the column and the sky x its diffuse transmittance.
- **Exposure.** One exposure: scene value = luminance x `DAY_EXPOSURE` (1.5) x pi / E_v,TOA / d_AU^2 - a white
  Lambertian under the TOA Sun renders 1.5 in every material.

## Code

- Core `environment`: `atmosphere_optics.*` (coefficients), `atmosphere_tables.*` (parameterisation, input hash),
  `atmosphere_precompute.*` (parallel over table rows), `atmosphere_scattering.*` (CPU evaluation: sun transmittance,
  sky luminance, sky luminance to a point, sun + sky illuminance), `atmosphere_table_file.*` (cache),
  `atmospheric_refraction`, `eclipse.cpp`.
- Game: `game/world/atmosphere.gd` (`Atmosphere`), `world/eye_adaptation.gd`, `world/vision_effect.gd`,
  `world/refraction.gd`, `world/lit_aerial.gd`, `world.gd`; shaders `atmosphere.gdshaderinc`, `sky.gdshader`,
  `earth.gdshader` / `earth_clouds.gdshader` (both `earth_surface.gdshaderinc`), `terrain_earth.gdshader`,
  `terrain_moon.gdshader`, `moon.gdshader`, `moon_regolith.gdshaderinc`,
  `lit_aerial.gdshader`, `plume.gdshader`, `star_visibility.gdshaderinc`, `vision.gdshaderinc`,
  `eclipse.gdshaderinc` (mirrors `core/src/environment/eclipse.cpp` - change both together),
  `refraction.gdshaderinc`, `display_colour.gdshader`. `atmosphere.gdshaderinc` mirrors
  `core/src/environment/atmosphere_scattering.cpp` - change both together.


## Standing decisions

- One model for all viewpoints; physics first, nothing tuned (spec). Flight physics keeps `Us76Atmosphere`.
- Sun light evaluated at one point (the followed vehicle, else the camera); per-object lights not done.
- Plumes, the stars' own brightness and UI markers are display-referred (inverse rod step).
- Sun disc colour = the model's TOA solar spectrum, normalised to unit luminance.

## Validation

- Core `[atmosphere]` (21 ctest cases, incl. aerial perspective to a point before the entry): Rayleigh coefficient and
  vertical optical depth vs Bucholtz 1995 (1 % / 2 %); transmittance vs a direct optical-depth integral (3.24e-3) and
  single scattering vs a brute-force raymarch (1.07e-3); off-texel lookups 7.1e-3, horizon seam 9.2e-3; multiple
  scattering 5 vs 7 orders 1.96e-3; direct sun at air mass 1.5 vs ASTM G-173 (400-700 nm) -2.1 %, runtime direct beam
  vs raymarch < 0.05 %; zenith at Sun 60 deg 3819 cd/m^2 (B/R 1.97), horizontal illuminance at Sun zenith 125.4 klx,
  sky at 50 km 7.1e-4 x the ground, twilight zenith bluer than noon (B/R 3.12 at -4 deg); ellipsoid mapping < 1 mm;
  cache round trip / damage / stale hash / failed write.

## Open points and known limits

- Clouds on the true-space Earth (terrain, cloud pass), the scaled-space Earth (`earth_clouds.gdshader`, the far
  field) and the vehicles' light (`weather-clouds.md`); no weather-dependent
  aerosols, no airglow; one aerosol set for the whole Earth; one Sun light (other bodies in
  view, e.g. the booster far below the ship or the tower seen from orbit, get the followed point's light); the totality
  approximation; night vision's limits (above); the 256 x 128 Earth mesh in scaled space sits up to ~0.6 km below the
  ellipsoid (-2 % in its aerial perspective).

## Radiance cubemap (ambient and reflections of every lit object)

- **Refresh.** The sky reads global uniforms only and Godot redraws a sky's cube only after a write to its material, so
  `World._update_sky_radiance` writes `radiance_serial` when `SkyRadianceRefresh.due` says the sky changed. Thresholds
  [E, rationale in the file]: tables loaded; Sun (Earth-local) or the eye's zenith (cube axes) 0.035 deg (a 2 % step in the
  deepest twilight); eye height max(200 m, 2 %); `luminance_scale` 2 % (the cube holds scene values, i.e. times the eye's
  gain); Moon light 2 %; 1 % of a fraction (visible solar disc, the Earth's disc, both lights on the lunar ground, the
  cloud factors). Cadence: ~8 s at rest on the Earth, ~0.5 s in low orbit, a clock or site jump at once. The cube takes 8 to
  16 frames after the write (one base render, then one roughness layer per frame; a write during a pass waits for its end).
- **Why the gain needs a redraw.** Godot 4.7's `ambient_light_energy` does not reach a sky's cube: at
  `ambient_light_sky_contribution` 1 `scene_forward_clustered.glsl` mixes it out, and the cube's ambient and reflections are
  scaled by `background_energy_multiplier` only, which also scales the sky's own pass. Scaling instead of redrawing is
  therefore not available; the cloud factors sit in the cube pass.
- **Content.** Sky and model ground of the same model, so the shade side sees the real sky and the lit ground below
  the horizon. A cube of the eye's place: a hull far from the eye is lit as at the eye.
- **Only in the cube pass (`AT_CUBEMAP_PASS`).** Near the Moon (`Atmosphere.near_moon`) a lunar ground sphere (mean texture
  albedo 0.30, regolith reflectance under Sun and earthshine, `moon_ground_radius`) stands in front of the Earth: the
  ground bounce on the Moon, and no Earth through the Moon. A far Earth smaller than 0.07 rad radius is drawn at that
  radius at the same flux [E, `RadianceFarEarth`], radiance 3 (1 - x)^2 with x = (1 - cos angle) / (1 - cos radius), whose
  integral is exactly the uniform disc's: from the Moon it is ~1 texel; point sampling made the earthshine jump, and a uniform
  disc's hard edge still did (range 10.6 % over 10 h).
- **The Earth from the Moon.** One earthshine: near the Moon the disc is uniform and grey with the flux of
  `Atmosphere.earthshine_relative` (Bond albedo), the regolith's and the eye's; the cube's cloudless model Earth gave
  0.37 x that. Its flux x the Earth's true visible fraction at the eye (terrain horizon via
  `MoonSunHorizon.disc_visible_fraction`, marched anew past 50 m of the eye or 0.02 deg of the Earth; else the ground
  sphere). A wide disc that would cross the cube's horizon is drawn lifted clear of it, its centre 4 deg high (computed, not measured: for a
  true e = 1 deg a side plate reads cos 4 / cos 1, 0.2 % too little, a zenith plate 0.070 instead of 0.017 of the earthshine's
  plate), a limit; before, e = +1 deg lost 34 % and e = -1 deg kept 35 % although hidden.
- **Shadows on the lunar ground.** The ground's Sun term x the Sun's visible fraction at the eye x the Earth's disc in front
  of the Sun, its earthshine term x the Earth's fraction [E, `RadianceFarEarth.ground_fraction`]: the eye's shadows stand for
  the ground in view up to 1 km over the terrain, fading out by 10 km (an orbiter in shadow still sees the sunlit ground
  ahead). Limits: an eye in a crater's shadow gets no light from the sunlit rim; an eye on a lit plain none of the shadows
  around it; vehicles' shadows do not enter the cube; terrain does not shape the ground sphere.
- **Weather.** The cube has no clouds: over a deck the ground half shows the cloudless ground (sea ~0.06 against a cloud
  top ~0.7, estimated up to ~10 x too dark on a belly over an overcast sea). Not small, so documented, not fixed. Under the
  deck, reflections included, by channel (linear sRGB; the droplets are grey, the clear sky is blue, the light under a
  deck
  mostly the Sun's), its sky x (T_d E_sky + T_sd E_h) / E_sky and its ground, which already holds the direct light, x
  (E_h (T_s + T_sd) + E_sky T_d) / (E_h + E_sky), each a sum over the Sun and, while the cube shows it, the Moon; E_sky
  and E_h
  at the eye, so the air under the base is in the colour. At the pad by day (R G B): optical depth 3 sky 7.01 4.77 2.96,
  ground
  0.83 (grey within 1 %); depth 15 sky 4.05 2.74 1.69, ground 0.46. A scalar factor kept the clear sky's colour under a
  deck: B/G
  1.49 on a white plate against 0.90, a blue pad and hull. A white plate under a deck is warm, R/G 1.19 B/G 0.90: the
  Sun's
  colour without a white balance, like a white plate in the open Sun. The sky's half is the mean: the brighter part of a
  deck
  towards the Sun is not modelled. The cube's users are `LitAerial` (the pad's concrete, the tower, the drone ship) and
  `vehicle_hull`, through Godot's ambient and reflections; the terrain and the sea do not read it (their own cloud light
  adds the
  Sun's diffuse light in its colour). Limits: the aerial perspective (`atm_fog`) keeps the cloudless in-scattering under
  a deck;
  the sea's reflected moonlit sky (`terrain_earth.gdshader`) has no cloud term.
- **Moon light.** `World.moon_light` is off near the Moon (it lit objects from the Moon's centre through the ground); the
  one switch `near_moon` also turns on the cube's ground and flat Earth. Limit at that border (~67 000 km from the Moon): the
  Moon is ~2 texels of the cube there (undersampled, not widened) and `moon_light` before the border is still the
  Earth-distance value (~1/30 of the light really there).
- **Validation** (`game/tests/sky_radiance_check.gd`, GPU; `sky_radiance_rule_test.gd`, headless). White sphere, direct
  lights excluded, linear display, tables first, then a chain of jumps, each lit phase against a reference, as measured /
  reference: pad by day, up / (`sky_lux` / pi) 0.906 (+-15 %); moonless night `0 0 0`; full Moon, up / the Moon's sky 0.881
  (+-30 %); orbit, down / estimate of the ground in view 0.686 (factor 2); Moon by day, down / analytic regolith 0.948
  (+-30 %); lunar night under the Earth, up / (earthshine / pi) 0.742 (+-30 %, was 0.372; the side plate at e = +10 deg reads
  1.11: steady over time, so not aliasing, but direction-dependent: the coarsest level's lobe on a small source, not isolated);
  Earth below the horizon `0 0 0`;
  Earth at e = +1 deg against e = +10 deg x true fraction 0.983, e = -1 deg `0 0 0`; IM-2 sunlit, Sun 6.75 deg, display gain 2, code 60, 1.026
  (+-30 %; the reference is the same mean sphere as the cube, so it shares the sphere's simplification and says nothing about
  the terrain); IM-2 shadowed eye below the earthshine's plate; a crater floor 2758 m below the mean sphere: the ground's lights
  are the eye's (0.196 before, height over the sphere counted twice); transit (eye 200 000 km up, display gain 700) down /
  estimate 0.749; back at the pad the same pixels (+-2 code values). Decks: optical depth 3 and 15, up 1.016 and 1.016, down
  1.027 and 1.027 of clear x factor + the sky's leak (+-15 %; with the ground's half off a down plate still reads 0.075 of the
  sky's plate: the coarsest level); opaque deck in the moonlight up / the Moon's sky x factor 0.854 (+-30 %; 0.099 with
  T_d alone).
  Colour under the decks, R/G and B/G of the plate against the computed colour (+-5 %, set before measuring; the clear
  plate against
  the sky's colour, the same read, 0.795 1.478 / 0.808 1.463): depth 3 up 1.166 0.926 / 1.188 0.906, down 1.000 0.758 /
  1.008 0.767;
  depth 15 up 1.167 0.919 / 1.192 0.900, down 1.007 0.761 / 1.013 0.765; the Moon's deck, up at display gain 8, 1.170
  0.936 / 1.184
  0.904 (+-10 %). With scalar factors every deck plate fails (depth 15 up 0.793 1.491, down 0.784 1.099).
  `--timelapse`: sunset into twilight at warp 120, adapted: frame step at most 1.0 code values (limit 6), no step of the eye's gain; fixed gain: 177
  refreshes, up / sky 0.625 .. 0.709 (band 1.6) while the sky falls x 5.6. `--steady`: far Earth over 8 h in 30 min steps, range
  0.6 % of the mean (limit 5 %, set before measuring: a texel crossing at warp is a flicker). `--flight`: white hull entered
  from the opposite light, values in `vehicle-models.md`; `--flight --live`: fixed gain, the clock running through the ascent,
  43 refreshes, probe 3419 -> 1516 cd/m^2.
- **The checks can fail.** `--stale` freezes the cube (the refresh rule replaced in the test): phases 18 failures, `--timelapse`
  up / sky 0.655 .. 9.5 and 0 refreshes, `--flight --live` 3410 -> 3419 cd/m^2 and 0 refreshes, flight by day `0 0 1`, flight by
  night `251 253 255` (a day cube kept into the night), all exit 1.
- `render_check` never showed the stale cube: it blocks until everything is set, so its first cube is already right. Of
  its before / after pairs four are pixel-identical, pad differs in 254 px and pad_ground in 1308 px, local patches, not
  the cube (a cube change moves every lit pixel; `sky-fix-report.md` Fix 2).
