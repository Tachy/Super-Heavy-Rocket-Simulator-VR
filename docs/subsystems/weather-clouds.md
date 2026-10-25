# Weather clouds (M8 weather and clouds)

Plan `docs/superpowers/plans/2026-10-25-m8-weather-clouds.md`, spec
`docs/superpowers/specs/2026-10-25-m8-weather-clouds-design.md`. The snapshots come from `weather-service.md`. State
after Task 8 (volumetric clouds near the camera); Tasks 9-11 (light on the ground, far field, gate and docs) extend it.

## Model

- **Field on the GPU**: the GPU form of a snapshot (`environment::cloud_render_texels`, returned by
  `StarsimCore.weather_load_snapshot` as `cloud_texels`): RGBA8 per cell and layer, R cloud fraction, G/B the grid-mean
  extinction of liquid + ice as a 16-bit linear number (hardware filtering interpolates it like `WeatherSnapshot::at`;
  the log water codes would not), A the log-coded in-cloud optical depth from 20 km down to the layer centre (maximum
  overlap). One `ImageTexture3D` per snapshot (columns x rows x 32, no mipmaps): 133 MB at 0.25 deg; two resident
  (measured 265.8 MB, 49 ms main-thread upload each).
- **Shapes** (`clouds.gdshaderinc`, the core's coverage rule): cloud where the shape noise's R (trilinear, no mipmaps)
  exceeds `coverage_threshold(cloud fraction)`; in-cloud extinction = mean / fraction; detail noise x (1 + a (d - mean))
  with a fading to 0 at the preset's distance (no pop). Shape tile 32 km across / 16 km high [E], detail tile 2 km [E];
  Earth-fixed coordinates, the camera's part in float64 on the CPU.
- **Light**: the core's TMS source on the delta-Eddington source function (`cloud_scattering_source`, Sun and, at night,
  Moon) with the light's cloud optical depth from `light_steps` samples over 6 km, plus the sky source
  (`cloud_sky_scattering_source`: the clear sky's horizontal irradiance as isotropic diffuse incidence on the column);
  the air's Sun / sky irradiance at the cloud from `atmosphere.gdshaderinc` (incl. the eclipse's unocculted part),
  re-evaluated every 500 m / 5 % of the distance along a ray.
- **Raymarch** (`cloud_layer.gdshader`, a full-screen quad first in the transparent pass): the shell -0.5..20 km to the
  opaque depth; coarse steps (a fraction of the distance, at most one layer high) outside the field's cloudy region,
  fine steps inside it, never finer than what still reaches the end within `max_steps`; per-pixel-and-frame hash jitter
  (both VR eyes alike, no temporal accumulation). Background through exp(-tau), scattered light through
  exp(-(1 - f) tau); composited as T_bg scene + (1 - T_bg) S_c + T_c L with the air at the transmittance-weighted cloud
  distance. Off beyond 1.5e7 m (scaled space; Task 10).
- **Cross-fade** (`CloudLayer`, ruling 2026-10-25): two slots and the weight of slot 1; a new pair is reached through
  one visible snapshot (weight moves 1 per 2 s), the missing snapshot is uploaded into the slot without weight, then
  the weight tracks `WeatherService.blend()`. Weather off fades to the empty texture. Unused zero-weight slots are freed.
- **Quality presets** [E]: desktop 128 coarse / 256 max steps, fine 1.2 %, 6 light samples, detail to 30 km; VR 64 /
  128, 2.5 %, 4, 15 km.

## Validation

- Core `[weather]`: render texels (linear extinction, in-cloud depth vs trapezoids, code round trip); sky source:
  R + T = 1, R within 1 % of the two-stream at tau >= 10 (+26 % at tau 1), T within 3 %.
- GPU `game/tests/cloud_check.gd` (grid-mean field in the debug outputs): background transmittance below / inside /
  above a tau-1 slab vs the CPU Beer-Lambert within 1.3 %; Sun radiance from above a tau-10 slab vs
  `slab_reflected_radiance` within 1.7 % (6 geometries); filtered coverage 0.1035 / 0.5057 / 0.9021 for 0.1 / 0.5 / 0.9;
  climbing out in 50 m steps <= 4.7 % of the brightest value per step; inside the slab finite and non-black.
- `game/tests/cloud_layer_test.gd` (headless): the cross-fade never jumps, never changes a weighted slot.

## Diagnostics

- `render_check.gd --weather=<abs path .wthr>` or `--weather=slab:<bottom>,<top>,<liquid kg/m^3>,<fraction>` (pinned,
  no service), `--weather[=s]` online, `--cloud_quality=desktop|vr`, `--look_at_sun [--away_from_sun]`.
- `terrain_perf.gd --weather=<abs path .wthr> --cloud_quality=vr` (weather off otherwise).

## Open points and known limits

- Performance (Task 11 gate): at the VR preset the eclipse-epoch run gave p99 17.4 ms (13.0 without clouds), GPU max
  +4-5 ms on the pad, up to +17 ms at ~8 km looking down on a full deck; the per-step cost dominates (light samples
  matter little): a reduced-resolution cloud pass is the lever.
- Plane-parallel light (independent column): cloud sides facing a low Sun get mu0 x E (too dark grey with the Sun
  behind the camera); forward radiance at low Sun up to -56 % vs Monte Carlo, TMS aureole too bright through thick
  cloud (cloud_optics.hpp). Sun below the local horizon (mu0 <= 0): no Sun source (no lit bases at sunset).
- Hash grain at thin edges and at the horizon (no temporal accumulation); the forward silver lining is speckled.
- Ice uses the droplet phase function; no ground-reflected light into the cloud bases; the sky light kept in the
  forward peak is left out.
- No cloud shadows / light under the deck yet (Task 9).
