# Weather clouds (M8 weather and clouds)

Plan `docs/superpowers/plans/2026-10-25-m8-weather-clouds.md`, spec
`docs/superpowers/specs/2026-10-25-m8-weather-clouds-design.md`. The snapshots come from `weather-service.md`. This file
is the current state after Tasks 1-11: the field and its optics, the cloud pass, the light on the ground, vehicles and
the eye, the far field on the scaled-space Earth, the performance gate. Superseded numbers and earlier approaches:
`docs/history/weather-clouds.md`.

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
- **Shape prefilter at large pixel footprints** (Task 10 fix round 1, anti-aliasing; `cloud_noise.hpp` "Shape
  prefilter", `clouds.gdshaderinc`): the shape texture is point-sampled per ray, right while its octaves (Worley
  8 km, Perlin 4 km to 500 m) span pixels; from the scaled-space limit a march pixel is 7-27 km and the shape aliased
  (moire, speckle, a wrong local cover). Beyond the start footprint f_s = 3.5 km of the pixel (not the light march's
  interval) the shape is read from its box-filtered mips (`CloudNoise::shape_mips`, 2^3 texel means) at the level
  lambda = clamp(log2(f / f_s), 0, 1) log2(f / 250 m), at most 2 (1 km texels), against the level's own threshold and
  edge mean factor (`prefilter_threshold` / `prefilter_edge_mean`, rows at quarter levels: covered area cf, mean water
  the grid mean - the detail band limit's rule applied to the shape). Beyond that, each ray reads the shape at a
  horizontal offset drawn from the pixel's tent filter (half-width f, faded in from f_s to 2 f_s; blue noise + the R4
  sequence per frame) and the temporal mean averages the pixel's transmittance over its footprint - the
  coverage-preserving mean in transmittance (a mean density would make broken cloud a fog); the cell (fraction,
  water) stays the true point's. Deeper mips (level 4, 4 km texels) were tried: their few features per horizontal
  slice of the tile showed as rings around the sub-camera point (cf 0.4 hand-over 10.7 % instead of 7.5 %). The
  code sits under `CLOUD_SHAPE_PREFILTER`: `CloudEffect` runs that march kernel only when a footprint on the frame's
  clouds can exceed f_s (largest pixel angle x (the horizon distances of the camera and of the 20 km cloud top on the
  largest radius of curvature + the light march's 6 km)), else the unfiltered kernel - bit for bit the code before
  (one added branch alone moved results by an ulp through the compiler). With the 1600 x 900 check window (pixel
  3.1e-3 rad at half resolution) that keeps the ground and an ascent up to ~30 km unfiltered; the limb from higher
  (an ascent near the Karman line, low orbit) is prefiltered where its footprint exceeds f_s (there the shape aliased
  as at the hand-over). Shadow maps never prefilter.
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
     of nearest / farthest, or the block's mean when its four distances lie within 5 % (`CLOUD_ONE_SURFACE_SPREAD`
     [E]: one surface - the ground seen from far, where the nearest depth cut every other ray short of the low clouds
     by the block's depth extent, several km from 8 600 km; Task 10); coarse steps outside the field's cloudy region,
     fine steps inside; blue-noise start jitter,
     64 x 64 void-and-cluster mask offset per frame by the golden ratio - the same for both eyes); background through
     exp(-tau), scattered light through exp(-(1 - f) tau); colour = T_c L + (1 - T_bg) S_c with the air at the
     transmittance-weighted cloud distance; a step whose weight in the scattered light T_sc (1 - exp(-(1 - f) tau))
     exceeds 0.15 [E] is marched again with steps of that weight (one sample deciding a whole deep step was the grain
     at cloud surfaces and at bases seen at a grazing angle; the step's intended length is compared, not the float32
     difference of the distances, whose 1 m ulp at 8 600 km retried the same step until the budget ran out - Task 10);
  2. temporal accumulation per eye: reprojection of the cloud distance (sky: rotation only) with the camera's float64
     motion into the previous frame's clip space of the same eye, history clamped to the 3 x 3 neighbourhood; a running
     mean of weight 1 / n (n the frames the history holds, kept per pixel) down to 1/32 (an exponential mean of 0.1
     kept 23 % of one frame's noise); a clamped history keeps at most 4 frames. `World` resets the history on view and
     clock jumps (`_view_jumped`, `SimClock.time_jumped`);
  3. depth-aware upsampling (bilinear x exp(-4 |ln(D / D_i)|)) into a full-resolution array (layer = view).
  The composite quad (`cloud_layer.gdshader`, first in the transparent pass, premultiplied) blends it over the scene;
  plumes draw over it, MSAA resolves stay intact. Off beyond 1.5e7 m (scaled space: the far field below). A new size
  (resize, divisor) creates new images; the main thread shows the new result before the next frame (`CloudEffect.swap_output`:
  replacing it inside the render callback broke the composite's material for a frame), the old images are freed three
  render callbacks later (no renderer error, no frame without clouds). `CloudLayer.shutdown` frees the resident volumes.
- **Per-eye geometry (VR)** (fix 73b6465, `game/tests/cloud_vr_stereo_test.gd`): Godot's `get_view_projection(v)`
  already contains the eye offset, so its inverse maps clip space to the HEAD's space. The pass therefore takes each
  eye's ray direction from two unprojected depths (`cloud_ray_earth`; the eye's own ray, not a point 2 near in front
  of the eye seen from the head - that crossed the disparity and fused the clouds at ~0.4 m), builds `clip_from_earth`
  with the head transform (the offset once), and marches from each eye's position (`eye_earth` per view). The test
  reproduces the renderer's matrices and checks the rays of both eyes meet at the cloud to 1e-16 rad; it mirrors the
  GDScript, not the shader (a typo in the shader's copy is not detectable by it). The user's headset check of the fix is
  pending.
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
  - Independent columns: a column's cover C, its cloudy part carries the grid-mean depth / C: T = 1 - C + C T(tau / C)
    for all three factors (T_sd = C x (T_total - exp(-tau / (C mu0)))). For C = 1 (decks) this is the plane-parallel
    column; for broken cloud the plane-parallel column would turn a sunny cumulus field into an overcast (cf 0.15,
    in-cloud tau 27: exp(-4) direct instead of ~0.85 - seen in the first Task 9 renders, fixed). The sky (vertical
    column) takes C = the largest cloud fraction in it (maximum overlap). The direct beam and the Sun's diffuse part take
    the beam's cover (fix round 1): `cloud_beam_cover(f, mu0)` = 1 - (1 - f)^n, n = 1 + 0.66 (1 - f) + 0.21 max(cot e -
    1.8, 0) (`constants.hpp cloud_overlap_*`, `CLOUD_OVERLAP_*` in the shader, [E]). The shapes' shadows overlap neither
    maximally nor randomly: the 3D noise displaces the layers' clouds against each other (cover above f at the zenith,
    a) and a slanted beam crosses further cells (b, k). Fitted to the shipped shapes' shadows (the near map, area means
    over eight independent sites, `cloud_check.gd` `STARSIM_SLANT_TABLE=1`, f 0.15-0.6, e 5-75 deg, the layer 1 km
    deep, in-cloud tau 3): the direct Sun's transmittance within 0.018. Measured, f 0.3: 0.629 at 75 deg, 0.627 at 45,
    0.571 at 20, 0.404 at 8, 0.278 at 5 (maximum overlap would give 0.70 at every elevation). Without it the near map
    (the shapes) and the far map / CPU light disagreed by 10 % at 45 deg and 40 % at 8 deg (a ring at the 9-11 km
    hand-over at sunset, seen by the first tests of fix round 1). Limits: the layer depth (1 km) and the in-cloud tau
    are those of the fit; the sky factor keeps the maximum-overlap cover (its hemisphere average would see ~4 % more
    cover at cf 0.3).
  - Column per pixel: `environment::cloud_render_column_texels` (RG8 per cell and layer: R the grid-mean depth from
    20 km to the layer centre, the A channel's log code; G the largest cloud fraction above; +66 MB per snapshot, two bytes per texel) - the
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
    texel and the step, the shapes' mean for steps > 0.5-2 km: its cover and depth mix toward the cell's own - cover
    1 for the shapes, the beam's cover of the cell fraction for the mean part), the far map (+-200 km) the grid mean with
    the beam's cover of the largest cloud fraction on the line. Distance rule (horizontal distance of the receiver from the camera): near -> far over 9-11 km, far -> the
    column slanted to the Sun (1 - C + C exp(-tau / (C mu0))) over 120-140 km; smooth, no pop. World space: the same for
    both VR eyes. The Moon's direct light uses the slanted column.
  - CPU (`StarsimCore.weather_cloud_light` -> {sun_transmittance, sky_factor, sun_diffuse}, i.e. (direct, sky, Sun's
    diffuse) in the dictionary; (1, 1, 0) without weather. `Atmosphere.cloud_light_at` and the Vector3s of the
    GPU-side checks use the order (direct, Sun's diffuse, sky): (1, 0, 1) without weather): the vehicles'
    DirectionalLight x T_s (`Atmosphere.sun_light_at`, the Moon's light likewise),
    `Environment.ambient_light_energy` = (T_d E_sky + T_sd E_sun,h) / E_sky at the followed body (above 1 under a
    bright thin deck; reflections keep the clear sky), the eye adaptation's down plate (E_n T_s + E_n mu0 T_sd + E_sky
    T_d) / pi, also for the Moon. All grid mean along the ray / column (independent columns).
  - The CPU light follows the cross-fade (final review I1): `CloudLayer` pushes (slot 0 file, slot 1 file, w1) to
    `weather_set_active` after every `_advance` when a slot changes or w1 moves by 0.01 (an empty slot is "", the empty
    snapshot: the core blends toward zero depths); a file out of the core's cache is retried every frame and
    `core_cache_missed` asks the service for a worker load. `cloud_check.gd` compares the GPU factors with the CPU
    light at five weights of a deck -> empty fade (within 3 %) and checks the plate light is monotonic. The empty
    slot's column texture is a zero LA8 volume (an R8 one reads cover 1 through `.ra`: the ground light dipped below
    both end states).
  - The CPU factors follow the GPU's clouds: `Atmosphere.cloud_light_at` returns (1, 0, 1) while `cloud_enabled` is false
    (noise not ready, nothing to show, beyond the scaled-space limit), so vehicles and the eye are not dimmed by clouds
    that are not drawn. Both snapshots' column depth and cover are blended first, then the transmittances computed
    (`blend_cloud_column_depths`; the renderer's order: `cloud_column_mean_depth`), not the mean of two transmittances.
  - Shadow darkness under broken cloud (documented limit): the Sun's diffuse part T_sd is the cell's mean and added
    uniformly - gaps get 1 + T_sd, shadows T_sd; physically the light scattered out of the clouds' sides concentrates
    under and beside them (shadows T_sd / C, the clear parts less), so the contrast between shadow and gap is a little
    too high in the gaps' favour: shadows too dark by (1 / C - 1) T_sd, gaps too bright by T_sd (the cell mean is
    right). A cf 0.3 cumulus layer, in-cloud tau 3, Sun 45 deg: the cell's T_sd ~0.25, the shadows' ~0.65 in reality.
  - Weather off (cloud_enabled false): every Earth-surface pixel as before - the shader's `CLOUD_LIGHT_BEGIN` / `_END`
    blocks only change values inside `if (cloud_enabled)`; the CPU factors are exactly (1, 1, 0).
  - The terrain material now binds the cloud globals (~15 more sampler descriptors per terrain node):
    `rendering_device/d3d12/max_resource_descriptors` = 262144 (the default 65536 ran out with ~2000 nodes, i.e. ~32
    per node; 262144 holds ~8000 nodes against 1875 drawn at most in `terrain_perf`: 4x headroom). Follow-up: a slim
    terrain include that declares only the ~7 samplers the terrain uses (the others are bound per node regardless).
- **Far field** (Task 10; `cloud_far.gdshaderinc`, `earth_clouds.gdshader` = `earth_surface.gdshaderinc` with
  `EARTH_CLOUDS`): beyond 1.5e7 m from the Earth's centre (`CelestialBody.SCALED_SPACE_DISTANCE_M`, the same distance
  that hands the terrain over to the ellipsoid mesh) the cloud pass is off (`cloud_enabled`) and `CloudLayer.far_enabled`
  makes `World` draw the scaled-space Earth with `earth_clouds.gdshader` (weather off: `earth.gdshader`, the shader of
  before - one shader with a run-time flag changed the code generation, ~1400 pixels by up to 4 half-float ulps). A pixel
  there covers >= 7 km; the column replaces the march, with the same field and light transport in its plane-parallel
  limit:
  - the column above the ground point G from the column texels (grid-mean depth tau, largest fraction f); the view's
    cover C_v = the beam cover of f at the view's cosine mu (`cloud_beam_cover_of`: the rule fitted to the shapes'
    shadows, for any line through the layer); independent columns: clear on 1 - C_v, the in-cloud depth tau / C_v;
  - the cloud's place: the view ray's crossing of the height where the column's depth from its top reaches the
    transmittance-weighted mean depth of the seen light (bisection over the column texels' layer centres; the cloud
    pass's weighted distance for a slab); the column is taken again there (parallax of high cloud near the limb);
  - the cloudy part's radiance: a conservative slab of that depth lit by the Sun (at night the Moon) transmitted to the
    cloud point, through the transfer table - the cloud pass's source for a slab (mu_e = mu0 exactly) integrated along
    the ray with exp(-w_D tau_v) (single scattering in closed form, the table by 8-point Gauss-Legendre in
    u = 1 - exp(-w_D tau_v / mu); the core's `slab_reflected_radiance_mc` integrates the same between the depth nodes);
    the closed form likewise under the fidelity flag; plus the sky's source (closed form, linear in depth: integrated
    exactly) x the sky irradiance there;
  - composite as the cloud pass: alpha = C_v (1 - exp(-tau / (C_v mu))), colour = T_air C_v L + alpha S_air over
    (1 - alpha) x the ground; the ground lit through the column (`cloud_ground_light`'s column tier, the terrain's
    beyond 140 km: direct, the Sun's diffuse part, the sky's; the Moon likewise).
  Approximations: no light between columns (no lit cloud sides, no shadows on neighbouring columns), the light taken
  at one point, a Sun below the cloud's local horizon at the entry floor 0.02, view cosine floor 0.02
  (`CLOUD_FAR_MU_MIN` [E]). The CPU light factors stay (1, 0, 1) beyond the limit (`cloud_enabled` false; a camera
  there is far above any cloud). The earthshine on the Moon keeps the Bond albedo 0.306 (clouds included on average).
- **Quality presets** [E]: desktop 128 coarse / 256 max steps, fine 1.2 %, light 6 near + 3 far samples; VR 48 / 96,
  2.5 %, 5 + 2 (Task 11: 64 / 128 before; see Performance); both 3 detail levels, 1/2 resolution, temporal weight floor 1/32.

## Validation

- Real-date plausibility (final review I4, `test_weather_snapshot.cpp` "Real-date columns ...", fixture
  `era5_plausibility.toml` by `tools/validation/gen_weather_fixtures.py`: ERA5 columns of 2024-04-08 18 UTC and
  2025-08-15 18 UTC at Starbase, the Cape, the North Atlantic, the warm pool and the most overcast cell near Starbase,
  with the store's own `total_cloud_cover` and `cloud_base_height`). Cloud base (lowest layer with fraction > 1 % and
  condensate, ERA5's definition) within one layer plus the spacing of the pressure levels around ERA5's cbh. Cover: the
  snapshot is systematically LOWER than ERA5's tcc, by 0.00-0.36 (mean 0.14) with the same maximum-random overlap (the
  renderer's maximum overlap is lower still, 0.00-0.37): the 37 pressure levels hold the model levels' fractions
  interpolated, so clouds that overlap in the model at different horizontal places no longer add up. The test asserts
  never more than tcc + 0.05, at most 0.40 below, mean deficit < 0.2 (a limit of the data choice, `known-issues.md`).
- Core `[weather]`: render texels (linear extinction, in-cloud depth vs trapezoids, code round trip, ice share);
  vertical profile (flat base, water path kept exactly); covered fraction = cf within 0.003 at every detail state
  (0.006 between rows); soft edges of varying width (mean water / grid mean 0.996-1.000 for cf 0.05-1 at states 0, 1,
  1.5, 3); the shape prefilter (level 0 up to f_s and continuous, the mips' box filter, level 0 = the unfiltered
  sample and state-0 tables bit for bit; covered fraction within 0.003 and mean water / grid mean 0.995-1.007 at
  levels 0.125-2 for cf 0.1 / 0.4 / 0.9, 4e5 random positions); perimeter fractal dimension by box counting of the cloud mask (16 km plane at 4 m, cf 0.5, boxes 8 m to
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
- GPU `cloud_check.gd` section 7 (fix round 1; `tests/cloud_light_probe.gdshader` evaluates the surfaces' cloud-light
  functions at receivers on annuli around the camera, means over eight sites ~55 km apart = independent realisations of
  the shape noise; 93 samples in all): the shapes' area mean of the direct Sun over the near disc (r < 8 km) vs the CPU's
  cell, cf 0.15 / 0.3 / 0.4 at Sun 45 / 8 deg: within 6.1 % (the worst, cf 0.3 at 45 deg; ground 0.795 / 0.657 vs CPU 0.777 / 0.649 at cf 0.15, 0.627 /
  0.404 vs 0.592 / 0.396 at cf 0.3, 0.517 / 0.277 vs 0.490 / 0.277 at cf 0.4; tolerance 8 %); the near map's mean vs the
  far map's over the same receivers likewise; the column terms (Sun's diffuse, sky) within 0.6 %; the vehicle's total light
  vs the ground's mean within 3.3 % (cf 0.4 at Sun 8 deg: 0.715 vs 0.714; tolerance 10 %; asserted, no longer only
  reported). Hand-overs: near -> far (9-11 km, annuli 8 / 10 / 12 km, a cf 0.3 layer, Sun 45 / 8 deg) steps 0.02 / 0.08 against
  an allowance of 0.05 + 3 standard errors over the sites (0.11-0.18 for the broken layer: one annulus holds only a few
  cells, so this check is weak; the disc check above carries the agreement); a deck: 0.003 / 0.005. Far -> the column
  slanted (110 / 130 / 150 km): steps <= 0.002, the far map within 0.003 of the column (tolerance 0.03). Before the
  beam-cover fit the same checks failed: the near mean was 10 % below the far / CPU at 45 deg and 40 % at 8 deg.
  `STARSIM_SLANT_TABLE=1` prints the fit table. Render: `tmp/weather_t9f_sunset_handover.png` (cf 0.4 layer, Sun 9 deg,
  looking away from the Sun and 4 deg down from 700 m: the ground at ~10 km): no ring.
- GPU `game/tests/cloud_far_check.gd` (Task 10; the game's scene, HDR, linear tone map, fixed exposure): the far
  field's slab radiance vs the core's `slab_reflected_radiance_mc` at 72 geometries (tau 0.5-30, Sun 0-80 deg, views
  0-75 deg forward / side / back) within 0.34 % (tolerance 3 %; `tests/cloud_far_probe.gdshader`); weather off: the
  scaled-space Earth drawn with `earth.gdshader`, bit-identical to the pre-Task-10 shader as one file (0 of 8.6e6
  bytes); the hand-over just inside / outside 1.5e7 m (2024-04-08 14:00 UTC, camera over 0 N 45 W, seven Earth points
  0-45 deg away, Sun 20-75 deg up): decks of tau 10 / 2 (cf 1, 1-2 km) with the volume on the grid mean (cloud debug
  output 6: the far field's physics without the shapes) within 2.95 % / 3.89 % (tolerance 5 %); the volume pass only
  inside, `earth_clouds.gdshader` only outside, the CPU light factors (1, 0, 1) outside; from 384 400 km with the ERA5
  hour of 2024-04-08 18:00 five cloudy cells (vertical transmittance < 0.05) 5.3-8.2x brighter with the weather on,
  five clear cells (no cloud within 1 deg) unchanged (ratio 1.0000). The game's volume (the shapes, prefiltered)
  against the far field, tolerance 10 % (ruling 2026-10-26: the shapes' 3D light is outside a column model): deck
  tau 10 within 6.4 %, tau 2 within 4.3 %, broken cf 0.4 (21 x 21 block means) within 7.3 % (Task 11: every placement
  resets the cloud history and the frame counter and draws exactly 64 frames, so the temporal mean is the same number of
  frames in every run; the earlier runs, timing-dependent, gave 7.3 / 5.2 / 7.5-8.5 %; before the prefilter 32 %).
- Image identity of the shape prefilter (`render_check.gd --fixed_exposure --deterministic=64`: the cloud history
  and frame counter reset, then exactly 64 frames; ERA5 2024-04-08 18:00; identity holds at the check window (1600 x
  900) and the views' FOV used there - a larger window or a narrower FOV shrinks the pixel angle and keeps the
  unfiltered kernel up to a higher view; a wider pixel prefilters from lower): the Starbase ground view and an ascent
  view at 25 km toward the horizon are bit-identical to the commit before (a7d52e1); two runs of a7d52e1 itself
  differ in 0 / at most 1 pixel by 1. The low-orbit oblique view (420 km) differs at the limb only, where the
  footprint exceeds f_s.
- Visual set (Task 11, `render_check.gd --weather=online --cloud_quality=vr`, ERA5 hours downloaded at run time; PNGs
  `tmp/weather_t11_*.png`, read by eye): overcast vs clear at Starbase (OLP-1, 2 m eye, 2025-08-15 18:00 UTC: a thick
  anvil deck, the Sun hidden, vertical T_s 0.05 on the 1-degree analog / 2025-03-15 18:00 UTC: clear, deep blue sky);
  ascent at Starbase on the first date, below (500 m: a high deck over the sea), inside (9 km: white-out, no black frame,
  no banding) and above (20 km: the deck's top below the dark sky, a blue limb); the Cape from 100 km (2025-07-15 18:00,
  cumulus fields over the sea, an overcast mass over the coast); ISS orbit (420 km) over the frontal band of 2024-04-08
  18:00; sunset cloud bases (Starbase 2025-07-15 01:25 UTC = 20:25 CDT, just after sunset: the high clouds lit pink-orange
  against the pale blue sky, the low cumulus' bases dark grey-blue with lit rims, no rim artefact); cloud shadows at the Cape (2025-07-15 18:00 UTC, 14:00 EDT: from 6 km
  over the inland Florida land the ground seen between the clouds lies in their shadow - dark blue-grey, only sky
  light - with a few sunlit patches; from 4.5 km over the sea (`weather_t11_cape_shadows.png`) the cumulus field's
  shadows are only faintly visible on the dark water, `..._cape_shadows_land.png` carries the contrast); the Earth from the Moon at new Moon (2025-07-24 19:00,
  376 000 km: continents with the cloud pattern, the limb); Antarctica from 3000 km over the pole (2025-01-15 12:00)
  and the Pacific date line (lat 5, lon 180, 2025-07-15 00:00): no seam or NaN at the pole or the lon 0 / 180 line.
  ERA5 against GOES-East of the same hour (`tmp/weather_t11_gulf_vs_goes.png`, left the game, right GOES-16 ABI band 2
  0.64 um reflectance of 2024-04-08 18:00:20 UTC reprojected to the same pinhole camera: 3000 km above 27 N 90 W, 45
  deg): the long frontal band from the Mexican Gulf coast over the Mississippi valley to the Great Lakes lies in the same
  place and with the same orientation (offset by less than ~100 km by eye), the dense cumulus fields over the western
  Atlantic / Bahamas and the cloud streets over the Pacific coast of Mexico are in the right places, the eclipse's
  umbra / penumbra darkens the same lower-left corner; the Gulf is clear in both. Differences: ERA5's band is broader
  and softer (a 0.25 deg reanalysis, no sub-grid structure, the real band's thin cirrus streaks missing), GOES shows
  more broken cumulus over Texas and the Atlantic coast, and the game's far field has the dot lattice described under
  the known limits. Source: NOAA GOES-R series ABI Level 2 CMIPF, `s3://noaa-goes16/ABI-L2-CMIPF/2024/099/18/`
  (`OR_ABI-L2-CMIPF-M6C02_G16_s20240991800204_...`), public domain (NOAA Open Data Dissemination; GOES-16 was the
  GOES-East satellite on that date); not shipped, not in the repo (the reprojection script lived in `tmp/`).
- `game/tests/cloud_layer_test.gd` (headless): the cross-fade never jumps, never changes a weighted slot.

## Diagnostics

- `render_check.gd --weather=<abs path .wthr>` or `--weather=slab:<bottom>,<top>,<liquid kg/m^3>,<fraction>` (pinned,
  no service; the overlay shows the pinned snapshot), `--weather=analog` (the shipped analog of `--utc`, pinned),
  `--weather[=s]` / `--weather=online` online; any other value exits with an error. `--cloud_quality=desktop|vr`,
  `--cloud_detail_levels=0..3`, `--cloud_scattering=table|closed`, `--look_at_sun [--away_from_sun]`;
  `--fixed_exposure --deterministic[=64]` for before / after image identity (cloud history and frame counter reset
  once settled, then exactly that many frames).
- `terrain_perf.gd --weather=<abs path .wthr> --cloud_quality=vr [--cloud_scattering=table|closed]` (weather off
  otherwise).
- `cloud_debug_output` 1 / 2 / 3 / 4: background transmittance (grid mean), Sun radiance (grid mean), coverage at
  `cloud_debug_height_m`, background transmittance with the shapes; 5: the terrain draws its cloud factors (direct,
  the Sun's diffuse share, the sky's) as its colour (`render_check.gd --cloud_debug=5 --fixed_exposure`); 6: the
  full image on the grid-mean field without the shapes (the far field's physics; `cloud_far_check.gd`).
- `CloudShadowEffect.grid_mean_only` (checks): the near shadow map with the grid mean. `CloudShadowEffect.ran_msec`: the
  render callback's last run; `CloudLayer` takes the maps as valid (`cloud_shadow_valid`) only within 250 ms of it.
- `cloud_check.gd` with `STARSIM_SLANT_TABLE=1`: only the beam-cover fit table (shapes' direct Sun, near disc, eight
  sites, f 0.15-0.6 x e 5-75 deg).

## Open points and known limits

- **Performance gate (Task 11)**: `terrain_perf.gd --weather=<abs path .wthr> --cloud_quality=vr` (1440p desktop proxy of
  the 90 Hz VR frame; the gate it prints: p99 <= 11 ms with clouds / 8 ms without, max <= 16 ms; 2024-04-08 18:42 eclipse
  epoch, ERA5 18:00 pinned, three runs with and three without in one session, alternating). The machine's baseline moved
  between the notes: a clean one measured without clouds p50 4.5 / p99 7.6 ms (Task 9), the session of Task 11
  p50 8.25-8.29 / p99 12.58-12.63 / max 16.3-17.1 ms (three runs; the Task 9 scripts at 21a833c on the same machine and
  the same day gave p99 12.47, so it is the machine, not the code; no VR headset runtime was running (Pimax client / runtime and DesktopXR bridge stopped), the
  Virtual Desktop service and its virtual display driver could not be stopped without administrator rights). In that
  state the absolute gate p99 <= 11 ms cannot be met even without clouds and was not met. What the clouds add, measured
  against the baseline of the same session, is the budget the gate is derived from (clouds <= +3 ms p99 over the
  baseline):
  - preset 64 / 128 steps (before Task 11): p50 +1.4 ms, p99 +3.5-3.7 ms (16.14 / 16.07 / 16.23 vs 12.51 / 12.59 /
    12.53), max 20.9-22.4 ms - over the budget.
  - tuning runs (single runs, VR preset, `terrain_perf.gd --cloud_tune=...`): steps 48 / max 96: p99 14.90 (-1.2 ms);
    light near 5 -> 4: 15.62 (-0.5); fine step 2.5 -> 3.5 %: 15.72 (-0.4); all three: 14.47 (-1.7). Not touched: the
    detail levels (they are the approved shapes' fine structure), the resolution divisor, the light march's far part.
  - **preset 48 / 96 (the VR preset now)**: with clouds p50 9.51 / 9.51 / 9.51, p99 14.96 / 15.00 / 14.86, max 18.86 /
    19.04 / 19.61 ms against 8.29 / 8.25 / 8.27, 12.63 / 12.59 / 12.58, 16.91 / 16.32 / 17.07 ms without: clouds +1.25 ms
    p50, +2.4 ms p99 (within the +3 ms budget, 3 of 3), +2-3 ms max. By phase (third run): descent p99 15.90 vs 11.91
    ms (the worst, the decks seen from 8 km: GPU 14-15 ms per frame), pan low 13.45 vs 12.62, pan high 13.27 vs 12.23,
    Cape 13.99 vs 13.29. VRAM +430 MB (2886 vs 2456 MB at the Cape: two snapshots of field, ice and column textures).
    Image A/B of 64 / 128 vs 48 / 96 (`render_check --cloud_quality=vr --fixed_exposure --deterministic=64`, 1600 x 900,
    four views: cumulus layer from 2.5 km, sunset bases, the Cape shadows from 3.5 km, 420 km orbit over a front): mean
    absolute difference 0.3-0.7 of 255, 1-2.5 % of the pixels differ by more than 8 levels (cloud edges, p99 5-13), none
    visible by eye. The checks (`cloud_check`, `cloud_far_check`) run the desktop preset and do not cover the VR one.
  - The absolute gate (p99 <= 11, max <= 16 ms with clouds) therefore needs the machine's clean state (baseline p99
    ~7.6 ms): expected ~10.0 ms p99 with the +2.4 ms of the clouds. To be re-run by the user on a quiet machine.
  - Pre-warm (Task 11): `earth_clouds.gdshader` is drawn once on a sub-pixel quad at load, and the shape-prefilter march
    kernel is dispatched once (one group) in the first cloud frame; the first switch to the scaled-space far field used
    to cost up to 5.4 ms in one frame. Sporadic draw / present stalls of 16-46 ms happen with and without clouds (one
    46 ms frame in a baseline run of Task 11), i.e. they are the platform's.
  - Levers measured earlier (superseded numbers in `docs/history/weather-clouds.md`): the detail levels ~2 ms in the
    descent, the step refinement ~0.3 ms, the eclipse probes ~0.3 ms (per sample ~10 ms).
- Scaled space (Task 10; 2560 x 1440, 2025-10-25 15:00 UTC ERA5): the far field adds 0.26 ms GPU from the Moon's direction
  at 330 000 km (0.51 -> 0.77 ms) and 0.30 ms just outside 1.5e7 m (0.43 -> 0.73 ms); frame p99 ~1.5 ms. The shape
  prefilter kernel (A/B in one session, inside the limit at 1.4985e7 / 1.2e7 m): GPU p50 6.09 -> 6.29 / 6.07 and 8.05
  -> 8.03 / 8.08 ms: not measurable.
- Visible at kilometre footprints (Task 11 review of the visual set, `tmp/weather_t11_gulf_era5.png`, a camera 3000 km
  above the Gulf, footprint ~5 km, 96 settled frames; also over Antarctica): a regular lattice of small dots in the
  cloud interiors and along fronts (~40 km period). It is the 32 km shape tile's periodicity seen obliquely
  (32 km / cos 37-45 deg of the viewing angle at those latitudes), resolvable once the pixel footprint exceeds the
  prefilter's f_s - not temporal noise and not the half-resolution blocks. The large-scale pattern (fronts, cloud
  streets, the Moon's shadow) is right (below). Not changed: shapes are frozen. Follow-up (user to judge the look):
  de-tile the shape lookup by a per-tile hashed offset / rotation, blended across the tile borders.
- At sharp dark edges (a deck's base seen at 30 km) a staircase of the half-resolution blocks is still visible.
- The stochastic light march is unbiased in tau_s but exp(-tau_s) per sample is brighter on average (+10 % at the
  worst checked geometry with the preset's samples); more near samples reduce it.
- The transfer table is a plane-parallel slab result on a tilted independent column: light is not moved between
  columns (no light leaking around a cumulus' shadowed side beyond the entry-surface tilt); the HG peak is treated as
  forward (its ~0.3 deg width is not resolved in the multiply scattered light). Ice uses the droplet table.
- Ice uses the droplet phase function and thin sheets without a fibre direction; no ground-reflected light into the
  cloud bases; the sky light kept in the forward peak is left out.
- A resize shows the clouds of the old size for one frame (scaled), then the new ones.
- Light (Task 9): the vehicles' light and the eye adaptation are the grid mean at one point (a vehicle in a gap
  between cumuli gets its cell's mean, not the shadow next to it; under a deck the same within 0.4 %); one light point
  as before (the booster far below the ship gets the followed body's light). Ambient on vehicles = Godot's clear-sky
  cubemap x one factor (its angular shape and the reflections stay the clear sky's). The shadow maps are plane-parallel
  inside clouds (a receiver inside one gets a linear share), the Moon's direct light the column's (no shapes). The
  sky's diffuse transmittance and the Sun's diffuse light are the local column's (no light from neighbouring columns,
  no cloud-side glow in gaps). The scaled-space Earth takes the column (the far field: no shapes, no shadow maps).
- Far field against the volume at the hand-over (Task 10, `cloud_far_check.gd`): with the game's volume (the shapes)
  a deck of tau 10 differs by up to 6.4 % (the volume brighter 25-45 deg from the sub-camera point, Sun ~35 deg up,
  view ~68 deg off the vertical), tau 2 by up to 4.3 %, a broken layer (cf 0.4) by up to 7.3 % (the volume brighter
  at the centre by 5-8 %); with the volume on the grid mean the decks agree within 2.95 % / 3.89 %: the step is the shapes' sub-grid structure (the 3D entry cosine
  under a varying Sun depth only brightens: mu_e is clamped at mu0 from below; the soft edges' thin fringes), which a
  column model cannot carry. Accepted with the tolerance 10 % (ruling 2026-10-26): a faint step when crossing
  1.5e7 m. A thin layer seen from afar cuts many horizontal slices of the 32 km shape tile, whose cover is not exactly
  cf: faint rings around the sub-camera point remain (the shapes' own, not aliasing). The sub-pixel offsets leave a
  fine temporal grain on broken cloud beyond f_s until the temporal mean converges (still camera: frames; a moving
  one: the floor weight's ~23 % of one frame's noise).
- Far field: plane-parallel independent columns (no 3D radiative effects: no lit cloud sides at a low Sun, no shadows
  cast on neighbouring columns or the ground beside them); the field's 0.25 deg grid shows as soft cells near the
  hand-over (~4 pixels per cell at 1.5e7 m).
