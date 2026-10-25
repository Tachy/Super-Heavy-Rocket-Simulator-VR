# Weather clouds (M8 weather and clouds)

Plan `docs/superpowers/plans/2026-10-25-m8-weather-clouds.md`, spec
`docs/superpowers/specs/2026-10-25-m8-weather-clouds-design.md`. The snapshots come from `weather-service.md`. State
after Task 8, its amendment (volumetric clouds near the camera), Task 8b (Monte Carlo transfer table) and Task 9 (the
clouds in the light of the ground, vehicles and the eye); Tasks 10-11 (far field, gate and docs) extend it.

## Model

- **Field on the GPU**: the GPU form of a snapshot (`environment::cloud_render_texels`, returned by
  `StarsimCore.weather_load_snapshot` as `cloud_texels`): RGBA8 per cell and layer, R cloud fraction, G/B the grid-mean
  extinction of liquid + ice as a 16-bit linear number (hardware filtering interpolates it like `WeatherSnapshot::at`;
  the log water codes would not), A the log-coded in-cloud optical depth from 20 km down to the layer centre (maximum
  overlap); beside it the ice share of the extinction (`cloud_render_ice_texels`, R8, `cloud_ice_texels`). Two
  snapshots resident (0.25 deg: 133 + 33 + 66 MB each, with the column texture below). Uploaded by `CloudFieldTexture` one layer per frame
  (staging volume + GPU copy; the longest frame step 5.2 ms, incl. the volume creation) instead of one 49 ms main-thread creation.
- **Vertical profile rule** (`cloud_render_texels.hpp` `cloud_profile_value`, port in `clouds.gdshaderinc`): the two
  layer centres around a point are fetched apart; where cloud starts above a clear layer the base is flat and sharp
  (50 m transition, `cloud_base_transition_m` [E]) in the middle of the gap, partly so where cloud only thickens upward;
  thinning upward keeps the linear taper (the covered area shrinks: rounded tops). The term integrates to zero over the
  gap: water path and cloud amount are the field's.
- **Fractal shapes** (`cloud_noise.hpp`, fix round 1): cloud perimeters are fractal with the area-perimeter dimension
  D = 1.35 (Lovejoy 1982, `cloud_perimeter_fractal_dimension`). The thresholded value is v = s + sum_k w_k a_k (d_k -
  mean): the shape noise's R (trilinear, no mipmaps; its Perlin octaves with the fBm gain 2^-H, H = 2 - D) plus three
  detail levels (isotropic tiles 1000 / 250 / 62.5 m, cells 500 m down to 8 m; single Worley octaves with gain 2^-H,
  a_k = A 4^(-H k)). A continues the shape's increment spectrum at 62.5 m, times a factor calibrated on the observable
  (`cloud_detail_amplitude_calibration` 2.5: the Worley octaves are not Gaussian, D = 2 - H alone gave D = 1.22; with
  it the box-counted D is 1.350). Each level is band-limited by the pixel footprint f (w_k = clamp(log2(l_k / f) - 1,
  0, 1), l_k the level's middle cell; the march pixel's angle comes per view): an octave below the pixel enters with
  its mean, so silhouettes stay irregular down to the pixel at every distance. Tables per detail state L = sum w_k
  (256 x 4, linear between rows): thresholds t[L](cf) (covered area = cf at every state) and edge mean factors m[L].
- **Soft edges of varying width**: density = mean / m[L](cf) x smoothstep((v - t) / w(n)); the edge zone width varies
  log-uniformly between 25 and 200 m (`cloud_edge_width_min_m` / `_max_m` [E]: Katzwinkel et al. 2014 inner shells,
  Wang, Geerts & French 2009 transition zone ~200 m) by the width noise n in the shape's A channel; w_min / w_max =
  the median increment of v over those widths (0.024 / 0.072). m is taken over the joint (v, n) distribution: the cell
  mean stays the field's (0.996-1.000 at the table states, also between them). Ice-dominated cloud (the ice share)
  mixes in the ramp of thin-sheet coordinates (vertical tile 2 km instead of 16 km [E]; no fibre direction: no wind in
  the snapshot), the same detail. Shape tile 32 km across / 16 km high [E]; Earth-fixed, the camera's part in float64
  on the CPU (level k's origin fract(4^k x level 0's)).
- **Multiple scattering: Monte Carlo transfer table** (Task 8b, after Bouthors et al. 2008; `cloud_transfer_table.hpp`,
  `config/clouds/cloud_transfer_table.bin`, 9.7 MB): the source of the multiply scattered light of a plane-parallel
  conservative slab, made offline by `tools/validation/cloud_slab_monte_carlo.py --table-runs / --table-build` with
  the project's phase function (16 mu0 x 27 slabs tau* 0.1-1024, 3.42e8 photons, fixed seeds; 1-sigma per entry from 8
  batches in `tests/validation/weather/cloud_transfer_table_sigma.bin`: median 0.48 %, p99 8.6 % over entries > 1e-3).
  Axes: depth below the top a and above the bottom b (22 nodes each, s (sqrt2^i - 1) up to 255.75 plus infinity), mu0
  (16 nodes ((i + 1) / 16)^2), scattering angle theta (48, log-linear) and the folded azimuth psi about the Sun's
  propagation (13); half floats. It holds the Draine part w_D (p_D * L) of the source: the phase function's HG part (g
  0.995, a ~0.3 deg peak) stays in the view ray, which therefore attenuates the lights' source with exp(-w_D tau) (exact
  up to the peak's width; tallying the HG part too left 25-50 % noise near the Sun's direction deep in thick slabs).
  `cloud_scattering_source_mc` = exact single scattering p exp(-tau_s) + the table at (a = mu_e tau_s, b, mu_e, theta,
  psi) on the same tilted column as the 3D source. GPU: one RGBA16F 3D texture (720 x 273 x 22; RGBA = the (mu0, b)
  corners of a cell), one trilinear fetch per light sample. Fidelity flag `CloudLayer.scattering_model` (shader
  `cloud_scattering_model`): TABLE (default once the table is loaded) or CLOSED_FORM (below, for comparison). The sky
  light keeps the closed-form sky source on the scaled depths.
- **Light**: the Sun's (and at night the Moon's) optical depth is marched toward the light through the 3D density
  (`cloud_light_optical_depth`: near samples over 6 km in geometric intervals from 20 m through the shapes, then far
  samples to the cloud top through the grid mean; stratified, unbiased over the jitter). The closed-form source (the
  fidelity flag's CLOSED_FORM) is the core's `cloud_scattering_source_3d`: TMS on the delta-Eddington source function of the surface the light entered through
  (entry cosine mu_e = clamp((t_v + mu0) / (tau_s + 1), max(mu0, 0.02), 1); a slab gives mu_e = mu0 exactly, a sunlit
  flank or a base under a Sun below the local horizon gets its true incidence; tilted independent column, Varnai &
  Davies 1999). The light march band-limits the detail at the larger of the pixel footprint and its interval. Plus the
  sky source (`cloud_sky_scattering_source`, vertical column). Column depths use one convention in core and shader:
  the covered area's in-cloud mean mean / cf (the A channel's), continuous through the layer centres. The air's Sun /
  sky irradiance at the cloud from `atmosphere.gdshaderinc`, re-evaluated every 500 m / 5 % along a ray; the eclipse
  acts on both: the Sun through `atm_sunlit`, the sky light through `atm_sky_eclipse_factor`, evaluated per 8 x 8 work
  group at 5 km and 150 km along its centre ray and interpolated in distance (per sample it cost 10 ms at the eclipse
  epoch; the penumbra changes by ~3 % per 100 km).
- **Cloud pass** (`CloudEffect`, a compositor effect before the transparent pass; kernels in
  `shaders/cloud_pass.gdshaderinc` with `atmosphere.gdshaderinc` + `clouds.gdshaderinc` compiled as GLSL compute by
  `CloudShaderPort`, which turns their `global uniform`s into one uniform block + sampler bindings filled from
  `ShaderGlobals` - every script sets globals through `ShaderGlobals.set_value`, since
  `global_shader_parameter_get` is editor-only):
  1. raymarch at 1/2 resolution per view (the shell -0.5..20 km to the scene depth; the block's depth in a checkerboard
     of nearest / farthest; coarse steps outside the field's cloudy region, fine steps inside; blue-noise start jitter,
     64 x 64 void-and-cluster mask offset per frame by the golden ratio - the same for both eyes); background through
     exp(-tau), scattered light through exp(-(1 - f) tau); colour = T_c L + (1 - T_bg) S_c with the air at the
     transmittance-weighted cloud distance; a step whose weight in the scattered light T_sc (1 - exp(-(1 - f) tau))
     exceeds 0.15 [E] is marched again with steps of that weight (one sample deciding a whole deep step was the grain
     at cloud surfaces and at bases seen at a grazing angle);
  2. temporal accumulation per eye: reprojection of the cloud distance (sky: rotation only) with the camera's float64
     motion into the previous frame's clip space of the same eye, history clamped to the 3 x 3 neighbourhood; a running
     mean of weight 1 / n (n the frames the history holds, kept per pixel) down to 1/32 (an exponential mean of 0.1
     kept 23 % of one frame's noise); a clamped history keeps at most 4 frames. `World` resets the history on view and
     clock jumps (`_view_jumped`, `SimClock.time_jumped`);
  3. depth-aware upsampling (bilinear x exp(-4 |ln(D / D_i)|)) into a full-resolution array (layer = view).
  The composite quad (`cloud_layer.gdshader`, first in the transparent pass, premultiplied) blends it over the scene;
  plumes draw over it, MSAA resolves stay intact. Off beyond 1.5e7 m (scaled space; Task 10). A new size (resize,
  divisor) creates new images; the main thread shows the new result before the next frame (`CloudEffect.swap_output`:
  replacing it inside the render callback broke the composite's material for a frame), the old images are freed three
  render callbacks later (no renderer error, no frame without clouds). `CloudLayer.shutdown` frees the resident volumes.
- **Cross-fade** (`CloudLayer`, ruling 2026-10-25): two slots and the weight of slot 1; a new pair is reached through
  one visible snapshot (weight moves 1 per 2 s), the missing snapshot is uploaded into the slot without weight, then
  the weight tracks `WeatherService.blend()`. Weather off fades to the empty texture. Unused zero-weight slots are freed.
  An upload no longer needed is cancelled, and an upload never replaces a slot that carries weight.
- **Light on the ground, vehicles and the eye** (Task 9; `cloud_optics.hpp` "CPU light", `clouds.gdshaderinc` "Light
  under the clouds", `cloud_light.gdshaderinc`, `cloud_shadow.gdshaderinc`). Three factors at a point for a light
  (Sun, Moon): T_s the direct beam's transmittance, T_sd the light's part diffused through the column per unit of its
  horizontal irradiance (isotropic from above), T_d the sky's diffuse transmittance. Surfaces: sun x T_s, sky x T_d +
  the Sun's normal-incidence irradiance x mu0 x T_sd x (1 + n.up) / 2 (the upper sky's view factor); the sea's
  reflected sky x T_d + that isotropic radiance (the deck's glow). Without the Sun's diffuse part an overcast deck would
  pass ~4 % of clear sky; with it tau 30 at Sun 45 deg gives 0.277 (Kasten & Czeplak 1980: 0.25, the spec's 10-30 %).
  - T_total of a conservative slab = 1 - R from the Monte Carlo table (`slab_total_transmittance_mc`, R as the
    hemisphere integral of `slab_reflected_radiance_mc` on 24 x 24 nodes), tabulated once on load
    (`CloudSlabTransmittanceLut`: 40 depths 0-1024, s = 0.25 (r^i - 1); 17 cosines 0.02-1; monotone in tau; the
    diffusion asymptote beyond; ~1 s on all cores in `StarsimCore.cloud_transfer_table`; GPU `cloud_slab_transmittance`
    RF 40 x 17). Delta-Eddington is up to 26 % off it at low Sun (ruling: table). T_d = the two-stream diffuse
    transmittance 1 / (1 + 3 (1 - g) tau / 4) (ruled); against the table's white-sky transmittance 2 int T_total mu dmu
    within 4.6 % for tau 1-100 (+3.5 / +4.6 / +3.5 / +0.5 / -3.4 % at 1 / 3 / 10 / 30 / 100).
  - Independent columns, maximum overlap: a column's cover C = the largest cloud fraction in it (or along a light's
    path), its cloudy part carries the grid-mean depth / C: T = 1 - C + C T(tau / C) for all three factors (T_sd =
    C x (T_total - exp(-tau / (C mu0)))). For C = 1 (decks) this is the plane-parallel column; for broken cloud the
    plane-parallel column would turn a sunny cumulus field into an overcast (cf 0.15, in-cloud tau 27: exp(-4) direct
    instead of ~0.85 - seen in the first Task 9 renders, fixed).
  - Column per pixel: `environment::cloud_render_column_texels` (RG8 per cell and layer: R the grid-mean depth from
    20 km to the layer centre, the A channel's log code; G the largest cloud fraction above; +33 MB per snapshot) - the
    centre above (four decoded texels, bilinear: linear in tau as the CPU) + the trapezoid of the grid-mean extinction
    down to the point; cover = max(its G, the point's fraction). Godot gives a Texture3DRD of R8G8 the luminance-alpha
    swizzle (R, R, R, G): the shaders read `.ra`.
  - Direct Sun on the ground: two shadow maps (`CloudShadowEffect`, a compositor effect before the opaque pass; 512^2
    RGBA32F each) orthographic along the Sun, axes perpendicular to it, anchored on the ground under the camera
    (Earth-fixed until the camera is 4 / 50 km away: no swimming). Per texel the line toward the Sun through the shell
    -0.5..20 km: 64 grid-mean samples find the cloudy part (every 312 m of height), 64 samples over it give (t_front,
    t_back, optical depth, cover); a receiver at t_r gets 1 - C + C exp(-od / C x clamp((t_back - t_r) / (t_back -
    t_front))) - exact below the clouds, linear inside them; the four texels' transmittances bilinear. The near map
    (+-16 km, 62.5 m texels) marches the visible clouds' density (the shapes, detail band-limited at the larger of the
    texel and the step, the shapes' mean for steps > 0.5-2 km; cover 1), the far map (+-200 km) the grid mean with its
    cover. Distance rule (horizontal distance of the receiver from the camera): near -> far over 9-11 km, far -> the
    column slanted to the Sun (1 - C + C exp(-tau / (C mu0))) over 120-140 km; smooth, no pop. World space: the same for
    both VR eyes. The Moon's direct light uses the slanted column.
  - CPU (`StarsimCore.weather_cloud_light` -> {sun_transmittance, sky_factor, sun_diffuse}; (1, 1, 0) without
    weather): the vehicles' DirectionalLight x T_s (`Atmosphere.sun_light_at`, the Moon's light likewise),
    `Environment.ambient_light_energy` = (T_d E_sky + T_sd E_sun,h) / E_sky at the followed body (above 1 under a
    bright thin deck; reflections keep the clear sky), the eye adaptation's down plate (E_n T_s + E_n mu0 T_sd + E_sky
    T_d) / pi, also for the Moon. All grid mean along the ray / column (independent columns).
  - Weather off (cloud_enabled false): every Earth-surface pixel as before - the shader's `CLOUD_LIGHT_BEGIN` / `_END`
    blocks only change values inside `if (cloud_enabled)`; the CPU factors are exactly (1, 1, 0).
  - The terrain material now binds the cloud globals (~15 more sampler descriptors per terrain node):
    `rendering_device/d3d12/max_resource_descriptors` = 262144 (the default 65536 ran out with ~2000 nodes).
- **Quality presets** [E]: desktop 128 coarse / 256 max steps, fine 1.2 %, light 6 near + 3 far samples; VR 64 / 128,
  2.5 %, 5 + 2; both 3 detail levels, 1/2 resolution, temporal weight floor 1/32.

## Validation

- Core `[weather]`: render texels (linear extinction, in-cloud depth vs trapezoids, code round trip, ice share);
  vertical profile (flat base, water path kept exactly); covered fraction = cf within 0.003 at every detail state
  (0.006 between rows); soft edges of varying width (mean water / grid mean 0.996-1.000 for cf 0.05-1 at states 0, 1,
  1.5, 3); perimeter fractal dimension by box counting of the cloud mask (16 km plane at 4 m, cf 0.5, boxes 8 m to
  1024 m: D = 1.350, test 1.25-1.45, Lovejoy 1982; the shape alone 1.01); blue noise (each value equally often, 4 x 4 block-mean variance 0.18 of white
  noise); 3D Sun light (light march unbiased within 1 %, shadow-side transmittance exp(-mean tau) within 2 % of
  Beer-Lambert; sunlit flank 6.9x / 52x the shadowed one and 3.9x the column model at +10 / -3 deg Sun; a slab gives
  the column source exactly); sky source R + T = 1.
- Core `[weather]` transfer table (`test_cloud_transfer_table.cpp`): the 42 bins of `cloud_slab_radiance_mc.csv` (Sun
  0 / 60 deg, tau 1-30) within 4.5 % (bound 5 % or 3 sigma; the closed form up to -56 % in the forward bins); slab albedo
  vs `cloud_slab_albedo_mc.csv` (2e6 photons) within 1.1 % for tau 1-50, mu0 1 and 0.5 (closed form up to -32 %);
  semi-infinite R = 1.003 / 0.995 / 0.998 / 0.995 at mu0 1 / 0.5 / 0.2 / 0.1 (bound 1 %); the uniform-slab limit
  (source = single + table at (t, tau* - t, mu0); its ray integral = `slab_reflected_radiance_mc` within 0.2 %); the
  table's Draine weight = the core's phase function's; loader refuses foreign / truncated files; GPU packing.
- GPU `game/tests/cloud_check.gd` (the cloud pass; grid-mean field in debug outputs 1 / 2): background transmittance
  below / inside / above a tau-1 slab vs the CPU Beer-Lambert within 1.5 %; Sun radiance from above a tau-10 slab vs
  `slab_reflected_radiance` within 1.4 % with a converged light march (24 samples, 8 frames), the preset's 6-sample
  march +10 % at Sun 60 deg (Jensen bias of exp(-tau_s), tolerance 15 %); with the table vs
  `slab_reflected_radiance_mc` within 1.9 % (tolerance 3 %) at six geometries incl. a Sun 10 deg up seen forward (view
  75 deg: 0.368 /sr), the preset's march +5.7 % there; filtered coverage 0.1036 / 0.5057 / 0.9021 at
  full resolution (state 0, detail levels off for all numbers); climbing out in 50 m steps (temporal weight 1) <= 4.81 %
  of the brightest value per step; inside finite and non-black; transmittance to an occluder 2 km ahead inside the
  slab exp(-beta 2 km) within 0.7 % (depth -> distance); a near "mountain" edge through a half-resolution block: the
  occluder column T = 1.000, the sky column the slab's within 1.5 % (upsampling).
- Core `[weather]` light under the clouds: T_total = 1 - albedo of the Monte Carlo fixture within 3.2 % (10 cases;
  the LUT likewise), LUT monotone / 1 at tau 0 / continuous at its last node; white-sky vs two-stream within 10 %
  (4.6 % measured); overcast tau 30 at Sun 45 deg 0.277 in 0.2-0.3 (Kasten & Czeplak 0.25); the Sun's diffuse part under
  a uniform slab = T_total - exp(-tau / mu0) (table and delta-Eddington), none above it or with the Sun down; a broken
  layer (cf 0.4) as independent columns (direct, diffuse, sky); column texels = `cloud_column_depths_above` at the
  centres within the code's 3 % (depth) and 0.003 (cover).
- GPU `cloud_check.gd` section 6 (terrain_earth.gdshader on a quad 300 m from a camera 2 m up; slabs tau 1 / deck tau
  3 / broken cf 0.4; Sun 45 / 20 deg; debug output 5 = the three factors): with the near map of the grid mean
  (`CloudShadowEffect.grid_mean_only`) direct vs the CPU within 1.3 %, diffuse within 0.6 %, sky within 0.3 %, the
  rendered ground's luminance ratio weather on / off vs (T_s E_dir + T_sd E_sun,h + T_d E_sky) / (E_dir + E_sky) within
  0.9 % (tolerance 5 %); with the game's map (the shapes) under the cf 1 slabs the ground's total light vs the vehicle's
  CPU light at the same spot within 5.7 % (tolerance 10 %; under a deck tau 3 0.4 % / 0.1 %; the shapes' direct differs
  from the grid mean by up to 2.3x at Sun 20 deg through tau 1 - their low octaves vary along a 3 km path); with weather
  off land and sea bit-identical to the shader without its cloud blocks. atmosphere_check 86/86.
- `game/tests/cloud_layer_test.gd` (headless): the cross-fade never jumps, never changes a weighted slot.

## Diagnostics

- `render_check.gd --weather=<abs path .wthr>` or `--weather=slab:<bottom>,<top>,<liquid kg/m^3>,<fraction>` (pinned,
  no service; the overlay shows the pinned snapshot), `--weather=analog` (the shipped analog of `--utc`, pinned),
  `--weather[=s]` / `--weather=online` online; any other value exits with an error. `--cloud_quality=desktop|vr`,
  `--cloud_detail_levels=0..3`, `--cloud_scattering=table|closed`, `--look_at_sun [--away_from_sun]`.
- `terrain_perf.gd --weather=<abs path .wthr> --cloud_quality=vr [--cloud_scattering=table|closed]` (weather off
  otherwise).
- `cloud_debug_output` 1 / 2 / 3 / 4: background transmittance (grid mean), Sun radiance (grid mean), coverage at
  `cloud_debug_height_m`, background transmittance with the shapes; 5: the terrain draws its cloud factors (direct,
  the Sun's diffuse share, the sky's) as its colour (`render_check.gd --cloud_debug=5 --fixed_exposure`).
- `CloudShadowEffect.grid_mean_only` (checks): the near shadow map with the grid mean.

## Open points and known limits

- Performance (terrain_perf 1440p, 2024-04-08 18:42 eclipse epoch, ERA5 18:00 pinned, VR preset, one session, Task
  8b): without clouds p99 13.15 ms, with clouds 15.50 ms with the table / 15.13 ms with the closed form (p50 8.92 ->
  9.65 / 9.60); the table costs +0.4 ms p99, the descent 11.97 -> 16.69 ms p99, worst GPU second 21.3 ms (closed form
  20.5) at 8.4 km above the decks. One run had a 48 ms frame in the descent at 19 m (GPU 8.6 ms: a draw / present stall,
  not the clouds' GPU work). The baseline itself misses the Task 11 gate (p99 <= 11 ms). Levers measured: the detail levels ~2 ms in the descent
  (`--cloud_detail_levels`), the step refinement ~0.3 ms, the eclipse probes ~0.3 ms (per sample they cost ~10 ms).
- Performance after Task 9 (terrain_perf 1440p, same epoch and ERA5 file, VR preset, one session): without clouds p99
  7.98 / max 13.60 ms (p50 4.83); clouds at the Task 8b state p99 9.45 / max 14.83 (p50 5.43); with Task 9's light
  (shadow maps, column, terrain factors) p99 10.19 (p50 5.84; descent 10.29 -> 11.27, Cape 9.60 -> 10.11): +0.74 ms p99
  for Task 9, +2.21 ms for all clouds over the baseline (budget +3 ms); one 39 ms frame at t = 0 of the descent is the
  terrain's main-thread start (process 37.6 ms). VRAM +76 MB (the column textures). The 11 ms gate is Task 11's.
- At sharp dark edges (a deck's base seen at 30 km) a staircase of the half-resolution blocks is still visible.
- The stochastic light march is unbiased in tau_s but exp(-tau_s) per sample is brighter on average (+10 % at the
  worst checked geometry with the preset's samples); more near samples reduce it.
- The transfer table is a plane-parallel slab result on a tilted independent column: light is not moved between
  columns (no light leaking around a cumulus' shadowed side beyond the entry-surface tilt); the HG peak is treated as
  forward (its ~0.3 deg width is not resolved in the multiply scattered light). Ice uses the droplet table.
- Ice uses the droplet phase function and thin sheets without a fibre direction; no ground-reflected light into the
  cloud bases; the sky light kept in the forward peak is left out.
- A resize shows the clouds of the old size for one frame (scaled), then the new ones.
- The ray of each eye starts at the camera, not at the eye offset (the eye offset only moves the reprojection): the
  stereo parallax of near clouds is approximate.
- Multiview (VR) not yet verified on a headset (the pass handles each view's matrices; per-eye history).
- Light (Task 9): the vehicles' light and the eye adaptation are the grid mean at one point (a vehicle in a gap
  between cumuli gets its cell's mean, not the shadow next to it; under a deck the same within 0.4 %); one light point
  as before (the booster far below the ship gets the followed body's light). Ambient on vehicles = Godot's clear-sky
  cubemap x one factor (its angular shape and the reflections stay the clear sky's). The shadow maps are plane-parallel
  inside clouds (a receiver inside one gets a linear share), the Moon's direct light the column's (no shapes). The
  sky's diffuse transmittance and the Sun's diffuse light are the local column's (no light from neighbouring columns,
  no cloud-side glow in gaps). No cloud shadows in the far scaled-space Earth (Task 10).
