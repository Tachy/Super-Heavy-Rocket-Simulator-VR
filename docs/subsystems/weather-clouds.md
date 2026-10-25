# Weather clouds (M8 weather and clouds)

Plan `docs/superpowers/plans/2026-10-25-m8-weather-clouds.md`, spec
`docs/superpowers/specs/2026-10-25-m8-weather-clouds-design.md`. The snapshots come from `weather-service.md`. State
after Task 8 and its amendment (volumetric clouds near the camera); Tasks 8b-11 (transfer table, light on the ground,
far field, gate and docs) extend it.

## Model

- **Field on the GPU**: the GPU form of a snapshot (`environment::cloud_render_texels`, returned by
  `StarsimCore.weather_load_snapshot` as `cloud_texels`): RGBA8 per cell and layer, R cloud fraction, G/B the grid-mean
  extinction of liquid + ice as a 16-bit linear number (hardware filtering interpolates it like `WeatherSnapshot::at`;
  the log water codes would not), A the log-coded in-cloud optical depth from 20 km down to the layer centre (maximum
  overlap); beside it the ice share of the extinction (`cloud_render_ice_texels`, R8, `cloud_ice_texels`). Two
  snapshots resident (0.25 deg: 133 + 33 MB each). Uploaded by `CloudFieldTexture` one layer per frame
  (staging volume + GPU copy; the longest frame step 5.2 ms, incl. the volume creation) instead of one 49 ms main-thread creation.
- **Vertical profile rule** (`cloud_render_texels.hpp` `cloud_profile_value`, port in `clouds.gdshaderinc`): the two
  layer centres around a point are fetched apart; where cloud starts above a clear layer the base is flat and sharp
  (50 m transition, `cloud_base_transition_m` [E]) in the middle of the gap, partly so where cloud only thickens upward;
  thinning upward keeps the linear taper (the covered area shrinks: rounded tops). The term integrates to zero over the
  gap: water path and cloud amount are the field's.
- **Shapes with soft edges** (`cloud_noise.hpp`): cloud where the shape noise's R (trilinear, no mipmaps) exceeds
  `coverage_threshold(cf)` (covered area = cf); density = mean / m(cf) x smoothstep((s - t) / w): the liquid water rises
  over the edge zone (`cloud_edge_width_m` 100 m [E], Katzwinkel et al. 2014 / Heus & Jonker 2008) instead of jumping;
  w = 100 m at the noise's median gradient (0.0225 in shape value), m = the edge mean factor table (the cell mean stays
  the field's within 0.13 %). Ice-dominated cloud (the ice share) mixes in the ramp of thin-sheet coordinates (vertical
  tile 2 km instead of 16 km [E]; no fibre direction: no wind in the snapshot). Detail noise x (1 + a (d - mean)) with
  a fading to 0 at the preset's distance (no pop). Shape tile 32 km across / 16 km high [E], detail tile 2 km [E];
  Earth-fixed, the camera's part in float64 on the CPU.
- **Light**: the Sun's (and at night the Moon's) optical depth is marched toward the light through the 3D density
  (`cloud_light_optical_depth`: near samples over 6 km in geometric intervals from 20 m through the shapes, then far
  samples to the cloud top through the grid mean; stratified, unbiased over the jitter). The source is the core's
  `cloud_scattering_source_3d`: TMS on the delta-Eddington source function of the surface the light entered through
  (entry cosine mu_e = clamp((t_v + mu0) / (tau_s + 1), mu0, 1); a slab gives mu_e = mu0 exactly, a sunlit flank or a
  base under a Sun below the local horizon gets its true incidence; tilted independent column, Varnai & Davies 1999).
  Plus the sky source (`cloud_sky_scattering_source`, vertical column). The air's Sun / sky irradiance at the cloud from
  `atmosphere.gdshaderinc` (incl. the eclipse), re-evaluated every 500 m / 5 % along a ray.
- **Cloud pass** (`CloudEffect`, a compositor effect before the transparent pass; kernels in
  `shaders/cloud_pass.gdshaderinc` with `atmosphere.gdshaderinc` + `clouds.gdshaderinc` compiled as GLSL compute by
  `CloudShaderPort`, which turns their `global uniform`s into one uniform block + sampler bindings filled from
  `ShaderGlobals` - every script sets globals through `ShaderGlobals.set_value`, since
  `global_shader_parameter_get` is editor-only):
  1. raymarch at 1/2 resolution per view (the shell -0.5..20 km to the scene depth; the block's depth in a checkerboard
     of nearest / farthest; coarse steps outside the field's cloudy region, fine steps inside; blue-noise start jitter,
     64 x 64 void-and-cluster mask offset per frame by the golden ratio - the same for both eyes); background through
     exp(-tau), scattered light through exp(-(1 - f) tau); colour = T_c L + (1 - T_bg) S_c with the air at the
     transmittance-weighted cloud distance;
  2. temporal accumulation per eye: reprojection of the cloud distance (sky: rotation only) with the camera's float64
     motion into the previous frame's clip space of the same eye, history clamped to the 3 x 3 neighbourhood, weight
     0.1 of the new frame;
  3. depth-aware upsampling (bilinear x exp(-4 |ln(D / D_i)|)) into a full-resolution array (layer = view).
  The composite quad (`cloud_layer.gdshader`, first in the transparent pass, premultiplied) blends it over the scene;
  plumes draw over it, MSAA resolves stay intact. Off beyond 1.5e7 m (scaled space; Task 10).
- **Cross-fade** (`CloudLayer`, ruling 2026-10-25): two slots and the weight of slot 1; a new pair is reached through
  one visible snapshot (weight moves 1 per 2 s), the missing snapshot is uploaded into the slot without weight, then
  the weight tracks `WeatherService.blend()`. Weather off fades to the empty texture. Unused zero-weight slots are freed.
- **Quality presets** [E]: desktop 128 coarse / 256 max steps, fine 1.2 %, light 6 near + 3 far samples, detail to
  30 km; VR 64 / 128, 2.5 %, 5 + 2, 15 km; both 1/2 resolution, temporal weight 0.1.

## Validation

- Core `[weather]`: render texels (linear extinction, in-cloud depth vs trapezoids, code round trip, ice share);
  vertical profile (flat base, water path kept exactly); soft edges (mean water / grid mean 0.9987-1.0006 for cf
  0.05-1, covered fraction unchanged); blue noise (each value equally often, 4 x 4 block-mean variance 0.18 of white
  noise); 3D Sun light (light march unbiased within 1 %, shadow-side transmittance exp(-mean tau) within 2 % of
  Beer-Lambert; sunlit flank 6.9x / 52x the shadowed one and 3.9x the column model at +10 / -3 deg Sun; a slab gives
  the column source exactly); sky source R + T = 1.
- GPU `game/tests/cloud_check.gd` (the cloud pass; grid-mean field in debug outputs 1 / 2): background transmittance
  below / inside / above a tau-1 slab vs the CPU Beer-Lambert within 1.5 %; Sun radiance from above a tau-10 slab vs
  `slab_reflected_radiance` within 1.4 % with a converged light march (24 samples, 8 frames), the preset's 6-sample
  march +10 % at Sun 60 deg (Jensen bias of exp(-tau_s), tolerance 15 %); filtered coverage 0.1036 / 0.5057 / 0.9021 at
  full resolution; climbing out in 50 m steps <= 4.9 % of the brightest value per step; inside finite and non-black.
- `game/tests/cloud_layer_test.gd` (headless): the cross-fade never jumps, never changes a weighted slot.

## Diagnostics

- `render_check.gd --weather=<abs path .wthr>` or `--weather=slab:<bottom>,<top>,<liquid kg/m^3>,<fraction>` (pinned,
  no service; the overlay shows the pinned snapshot), `--weather[=s]` online, `--cloud_quality=desktop|vr`,
  `--look_at_sun [--away_from_sun]`.
- `terrain_perf.gd --weather=<abs path .wthr> --cloud_quality=vr` (weather off otherwise).
- `cloud_debug_output` 1 / 2 / 3 / 4: background transmittance (grid mean), Sun radiance (grid mean), coverage at
  `cloud_debug_height_m`, background transmittance with the shapes.

## Open points and known limits

- Performance: see the Task 8 amendment report (`.superpowers/sdd/.../task-8b-amendment-report.md`) for the VR preset
  numbers; the Task 11 gate is p99 <= 11 ms.
- The stochastic light march is unbiased in tau_s but exp(-tau_s) per sample is brighter on average (+10 % at the
  worst checked geometry with the preset's samples); more near samples reduce it.
- Forward radiance at low Sun up to -56 % vs Monte Carlo, TMS aureole too bright through thick cloud
  (cloud_optics.hpp) - Task 8b (Bouthors 2008 transfer table). The tilted column does not move light between columns.
- Ice uses the droplet phase function and thin sheets without a fibre direction; no ground-reflected light into the
  cloud bases; the sky light kept in the forward peak is left out.
- A window resize re-creates the pass's images: one frame without clouds (and one renderer error line).
- Multiview (VR) not yet verified on a headset (the pass handles each view's matrices; per-eye history).
- No cloud shadows / light under the deck yet (Task 9).
