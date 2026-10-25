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
- **Tables and cache.** Transmittance 1024 x 1024, scattering (Rayleigh + all multiple orders) and single Mie 8 nu x 32
  mu_s x 128 mu x 64 r, irradiance 64 x 16; mu_s_min = cos 120 deg (twilight to -18 deg for the star threshold).
  Precompute 61-62 s on 24 threads (once per input hash), file `data/atmosphere/earth_atmosphere.bin` (83.9 MB: header
  with magic, version, input hash, sizes and parameters, four float32 arrays, FNV-1a 64 checksum verified before
  parsing; temp name then rename; a damaged, stale or missing file is recomputed, a failed write still returns the
  tables). The input hash covers the config, the table sizes, the stored spectral / optics data and
  `atmosphere_tables_code_version`. CLI `build/core/atmosphere_tables.exe [--config] [--out]`; `tools/package.ps1` runs
  it, so installed games ship the cache. The game loads the cache on a worker (`StarsimCore.load_atmosphere`; first run
  computes and shows "Atmosphäre wird berechnet (einmalig, etwa eine Minute) ..." in the time panel). GPU formats:
  transmittance RGBAF (grazing values ~1e-5 are half-float subnormals), scattering / Mie / irradiance RGBAH; ~50 MB
  VRAM.
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
  earthshine; PROCESS_MODE_INCREMENTAL, radiance 128). Earth: `earth.gdshader` (scaled space; with the far-field
  clouds `earth_clouds.gdshader`), `terrain_earth.gdshader`, `water.gdshaderinc` (the sea's reflected sky is the
  model's sky along the reflected ray; a ray reflected below the horizontal takes its upward mirror [E]): Sun
  transmitted to the point x cos + the sky irradiance on the normal, then transmittance + in-scattering to the camera;
  city lights x T. Moon, Sun disc: x T + the in-scattered sky. The Sun disc's colour is the model's TOA solar
  spectrum (`atmosphere_sun_illuminance_lux`), normalised to unit luminance. Vehicles, tower, pad, drone ship:
  `lit_aerial.gdshader` (`LitAerial.material`): Godot's
  PBR plus aerial perspective through FOG (per-channel T split: t = max(T), FOG.a = 1 - t, FOG.rgb = S / (1 - t), and
  T / t on albedo and emission; only the dielectric specular takes the scalar); the tower lattice through `atm_fog`.
  Plumes (`plume.gdshader`, additive): x T only [call]. Flight trail, orbit lines and labels are UI markers, not
  attenuated (past the rod step unchanged: `vision.gdshaderinc`; map labels are `MapLabel` TextMeshes, the Moon's orbit
  line `display_colour.gdshader`).
- **Sun light.** One DirectionalLight: colour and energy per channel = TOA solar RGB x the core's
  `atmosphere_sun_transmittance` x luminance scale / pi x the Moon's visible fraction, at one point: the followed
  vehicle (vehicle focus), else the camera (`Atmosphere.sun_light_at`, `world.gd::_update_sun_light`). Near the Moon
  the visible fraction comes from `Atmosphere.moon_visible_fraction` (shared with the eye adaptation): within 2 m of
  the lunar mean sphere or below it the Moon's occultation is taken on the point's own ground sphere (GPU check:
  1551 m below the mean radius, Sun 60 deg up: light = the open sky's). Clouds (Task 9, `weather-clouds.md` "Light"):
  x the direct beam's cloud transmittance at that point (`Atmosphere.cloud_light_at`, the Moon's light likewise), and
  `Environment.ambient_light_energy` = the diffuse light under the clouds over the clear sky's there
  (`Atmosphere.cloud_ambient_factor`; exactly 1 with weather off). Earth surfaces (`terrain_earth.gdshader`): the Sun
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
