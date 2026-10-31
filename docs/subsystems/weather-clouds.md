# Weather clouds (M8 weather and clouds)

Plan `docs/superpowers/plans/2026-10-25-m8-weather-clouds.md`, spec
`docs/superpowers/specs/2026-10-25-m8-weather-clouds-design.md`. The snapshots come from `weather-service.md`. This file
is the current state after Tasks 1-11: the field and its optics, the cloud pass, the light on the ground, vehicles and
the eye, the far field on the scaled-space Earth, the performance gate. Superseded numbers and earlier approaches:
`docs/history/weather-clouds.md` (there, since 2026-10-30, also the cloud view cache, spike S, the hybrid march, the
old world volume and - removed in the cleanup, the baked volume being the only method - the old per-frame march with
its light cache, outline pass, near / far hand-over, F7 / F9 and the old shadow-map kernel; last state with them: tag
`clouds-old-march-last`).

## Model

- **Field on the GPU**: the GPU form of a snapshot (`environment::cloud_render_texels`, returned by
  `StarsimCore.weather_load_snapshot` as `cloud_texels`): RGBA8 per cell and layer, R cloud fraction, G/B the grid-mean
  extinction of liquid + ice as a 16-bit linear number (hardware filtering interpolates it like `WeatherSnapshot::at`;
  the log water codes would not), A the log-coded in-cloud optical depth from 20 km down to the layer centre (maximum
  overlap); beside it the ice share of the extinction (`cloud_render_ice_texels`, R8, `cloud_ice_texels`). Two
  snapshots resident (0.25 deg: 133 + 33 + 66 + 8 MB each, with the column and base textures below). Uploaded by `CloudFieldTexture` one layer per frame
  (staging volume + GPU copy; the longest frame step 5.2 ms, incl. the volume creation) instead of one 49 ms main-thread creation.
- **Cloud bases at the lifting condensation level** (static clouds Task 7 and its fix round 1; spec decisions 1-2;
  `weather_snapshot.hpp`, file format 5): `WeatherSnapshot::lcl_m(row, column)` is the condensation height (m above the
  GROUND) of the 2 m parcel, Romps 2017 exact (`weather-service.md`): 0 = saturated surface air (T - Td <
  `cloud_fog_spread_k` 1 K [E]), `weather_lcl_unknown_m` (-1) = unknown (no 2 m fields, invalid input), capped at the
  20 km weather top. The downsampled set and the pole rows take the LOWEST known base of the box (min of ground + LCL,
  less the box's mean ground, at least 0; a saturated column makes the box 0, unknown and NaN levels are skipped, a box
  of only unknown columns stays unknown), so the flat base never cuts a lower-based column's cloud.
  The build (`ColumnLayers::build` -> `raise_liquid_cloud_to_lcl`, on the 32 layer means of the column): a layer is
  significant liquid with at least the smallest non-zero water code (2e-7 kg/m^3) and a cloud fraction of at least
  `cloud_min_cover`. When the lowest significant liquid cloud has its base less than `cloud_boundary_layer_top_m` 3 km
  [E] above the ground, ALL liquid below the LCL (every run, traces included) is moved into the lowest significant run
  with a part above the LCL, its base (at least the LCL) in the boundary layer and at most the depth of the lowest run
  above the LCL (it is then the continuation of the same cloud; a deck further up is another cloud and keeps its
  water), spread over that part in proportion to its liquid (profile kept; cloud fraction at least the moved water's
  mass-weighted fraction, so the covered area does not shrink); without such a run the lowest run (wholly under the LCL)
  is lifted to start at the LCL with its depth, carrying all the moved water. The layer containing the LCL keeps its part above the level as a mean over the whole
  layer. Untouched: an LCL of 0 (fog / low stratus keep the ground), unknown, an LCL >= 3 km (incl. the 20 km cap), no
  liquid below the LCL, a lowest significant base above the boundary layer, and all ice (a layer's cloud fraction stays
  where it holds ice). The column's liquid water path is conserved (1e-6 relative on the layer means; the 8-bit codes
  then cost <= 1.97 % per layer). Limit: a very dense result clamps at the water code's 4e-3 kg/m^3 maximum (not reached
  by the ERA5 / GFS data tested).
  GPU: the base texture (`environment::cloud_render_base_texels`, `cloud_base_texels`, RG32F columns x rows, 8 MB at
  0.25 deg, globals `cloud_base_before` / `_after`): R the cell's base as a height above the ellipsoid (ground + LCL),
  G its layer's extinction scale, 0 = no base - fog, unknown, LCL >= 3 km, or condensate left in a layer wholly under
  the base's layer (e.g. ice under the LCL: the column stays uncut, the ice is shown), or a base closer than
  `cloud_base_snap_m` 20 m [E] (twice `CLOUD_MIN_STEP_M`, mirrored in the core, smoke_test compares) under its layer's
  top (the base is dropped: scale 1, no cut, the layer keeps a faint haze - a thinner sheet is not resolved by the march and its scale would
  diverge; the scale of a base 25 m under the top is 23). Limit: the rule works per layer, so ice in the base's own
  layer is pushed above the base by the scale (its water path kept, its part below the LCL not shown). `cloud_lcl_cut` (shader; core
  `cloud_lcl_cut_factor`, `cloud_lcl_extinction_factor`) reads the four nearest columns by `texelFetch`: the cloud
  fraction is multiplied by a ramp from 0 at the base to 1 over `cloud_lcl_ramp_m` 30 m [E] (ended at the top of the
  base's layer, so the cut stays in one layer), the extinction by that ramp times the scale; mixed bilinearly with the
  columns without a base counting 1. The scale (`cloud_lcl_extinction_scale`, >= 1) is the integral of the renderer's
  profile (the vertical profile rule, as `cloud_snapshot_cell` at the column) over the base layer's above-ground part
  divided by that of the profile times the ramp (Simpson on the pieces between the kinks: exact up to rounding), so
  what the cut removes under the base is put back above it and the rendered column keeps its water path (cloud_check
  section 12 on a built snapshot: 0.9993 of the core's optical depth; the uncut layers 0.9992). The CPU
  light and the shadow maps read the layer means without the cut: their column water path is the same. Flat sharp
  bases, nothing hangs below them, silhouettes above the base's layer untouched; nothing is fetched above 12 km
  (Everest 8.85 km + the 3 km boundary layer, rounded up).
- **Cover from the total cloud cover, cumulus water from an adiabatic parcel** (static clouds Task 9c and its fix round
  1, user 2026-10-27 "Cumulus sind nie flach", ruling "Bedeckung nach ERA5 + Wasser ergaenzen"; `weather_snapshot.hpp`,
  file format 7). Every snapshot carries the data's total cloud cover per column (ERA5 single-level
  `total_cloud_cover`, GFS `TCDC` entire atmosphere instantaneous; `WeatherSnapshot::total_cloud_cover`, -1 unknown).
  The column build (`ColumnLayers::build`) runs, after the LCL raise:
  - `scale_cloud_cover_to_total`: the fractions of the layers that count for the cover (stored condensate, a non-zero
    water code, above the ground, fraction above `cloud_trace_fraction` 0.01 = ERA5's cloud-presence threshold; traces
    keep their value so no cloud base appears under the data's) are multiplied by one factor (in proportion, at most 1,
    at least `cloud_min_cover`) until their maximum-random cover (`cloud_column_cover_max_random`, Geleyn &
    Hollingsworth 1979 as ERA5 combines its levels) is the data's tcc; the water contents (grid means) are kept, so the
    in-cloud water of every type falls by that factor. The downsampled set and the pole rows scale their box means
    again to the box's mean tcc (before that re-scaling the 1-deg cover was 0.057 below tcc). The plausibility test
    (cloud base, cover within 0.02 of tcc) runs on the shipped, scaled snapshot; the cover check is close to a tautology
    (same overlap function as the scaler) and verifies the 8-bit codes and the layer remap, not ERA5's overlap physics.
  - `bound_incloud_condensate`: where the scaling puts a layer's water on a much smaller fraction (down to
    `cloud_min_cover` where tcc ~ 0) the in-cloud liquid is cut to `cloud_liquid_water_max_kg_m3` 3e-3 and the ice to
    `cloud_ice_water_max_kg_m3` 1e-3 [E, order-of-magnitude aircraft maxima, no specific citation]; the excess is dropped
    (not concentrated into dense specks) and stored per column (`dropped_condensate_path_kg_m2`, diagnostic). Also
    applied to downsampled boxes and pole rows after their re-scaling.
  - `add_cumulus_adiabatic_water` (the path rule of the measurement round): the lowest significant liquid run with
    its base in the boundary layer and a known LCL (0 < LCL < 3 km), base z_b = max(ground + LCL, run bottom), path =
    its layers with more than `cloud_base_snap_m` above z_b. The cumulus-weighted (w_k of `cloud_type_weights`: no
    switch, a deck's layers keep their water) in-cloud liquid water path sum w_k L t / C is raised to the weighted
    sub-adiabatic parcel path sum w_k Gamma_l(T, p) * integral f_ad(h) h dh (h above z_b, parcel capped at ground + 3 km),
    each path layer's liquid multiplied by 1 + w_k (s - 1), in-cloud at most `cumulus_liquid_water_max_kg_m3` 1.5e-3
    [E] - a denser layer keeps its value; the layers that get water meet the floor exactly (a deck below no longer
    lowers the ratio). f_ad(h) falls linearly from `cumulus_adiabatic_fraction_base` 0.50 at the base to
    `cumulus_adiabatic_fraction_at_reference` 0.20 at 2 km, constant above [E]: the trend (shallow Cu 0.3-0.6, Brenguier
    et al. 2000; continental Cu 0.2-0.6 falling with height, Blyth 1993; Warner 1955) is from those papers, the end
    values are the low ends of the ruled ranges (0.5-0.6, 0.2-0.3), chosen to bring the mean water down, not read off one
    dataset. Gamma_l (`adiabatic_liquid_water_gradient_kg_m3_per_m`) = -rho_d dr_s/dz of the pseudo-adiabat with
    Romps' constants (1.6-2.4 g/m^3/km at 275-290 K, 800-950 hPa; 2.6-2.8 at a 298-300 K tropical base). The parcel ends
    at the boundary-layer top: f_ad is a shallow-cumulus value, and a congestus / cumulonimbus run keeps its data water
    above 3 km (it shares the factor). The added grid-mean liquid path is stored per column
    (`added_liquid_path_kg_m2`, diagnostic). The layer means carry T and p for it (`LevelSample` 5 fields).
  - Order: LCL raise, cover, bound, water - the cloud types follow the final fractions, the water floor holds on them.
    CPU light, far field, shadow maps, air shadow and the GPU field read the same layer means (one source).
  - Measured on four analog hours (Jan / Apr / Jul / Oct 2025 15th 00 UTC, 1 deg, `tau_inv/t9c_*.py`), fix round 1
    against the first 9c state (f_ad 0.4 constant): grid-mean liquid water path 50.2 -> 140.4 -> 111.1 g/m^2 (ice 20.4;
    the 48-hour set 117.5 g/m^2, ice 21.6); added water 96 -> 61 g/m^2; low boundary-layer cumulus (area-resolved
    vertical in-cloud tau, maximal overlap) p10 / p50 / p90 6.2 / 30.6 / 144 -> 6.0 / 27.4 / 106, thin area (tau < 7.07)
    16.6 % in the data -> 11.8 -> 12.4 %; in-cloud LWC p50 / p90 / p99 0.117 / 0.303 / 0.516 (data) -> 0.233 / 0.629 /
    1.054 -> 0.189 / 0.488 / 0.823 g/m^3; dropped condensate 0.002 g/m^2 grid mean (0.25 deg: 0.44 % of the columns,
    at most 36 g/m^2 in one). Cover deficit vs tcc 0.196-0.203 (data) -> 0.004-0.009. The mean liquid water path is
    still above the ocean climatology the reviewer quoted from memory (MAC-LWP, O'Dell et al. 2008 / Elsaesser et al.
    2017: order 60-90 g/m^2, not verified here), see `known-issues.md`.
- **Vertical profile rule** (`cloud_render_texels.hpp` `cloud_profile_value`, port in `clouds.gdshaderinc`): the two
  layer centres around a point are fetched apart; where cloud starts above a clear layer the base is flat and sharp
  (50 m transition, `cloud_base_transition_m` [E]) in the middle of the gap, partly so where cloud only thickens upward;
  thinning upward keeps the linear taper (the covered area shrinks: rounded tops). The term integrates to zero over the
  gap: water path and cloud amount are the field's.
- **Fractal shapes, one body per cumulus** (`cloud_noise.hpp`, M8 fix round 1; erosion: static clouds Task 5): cloud
  perimeters are fractal with the area-perimeter dimension D = 1.35 (Lovejoy 1982, `cloud_perimeter_fractal_dimension`).
  The thresholded value is v = s + sum_k e_k(p_k), p_k = w_k a_k (d_k - mean), e_k(p) = p below 0 and lift_k p above
  (`cloud_detail_lift` [E] 0.5 / 0.25 / 0.25, `cloud_detail_erosion`; the shader gets them as the global
  `cloud_level_lift`): the shape noise's R (trilinear, no mipmaps; its Perlin octaves with the fBm gain 2^-H,
  H = 2 - D) eroded by three detail levels (isotropic tiles 1000 / 250 / 62.5 m, cells 500 m down to 8 m; single Worley
  octaves with gain 2^-H, a_k = A 4^(-H k)). Each level lowers v fully and lifts it only by its share (user
  2026-10-26: each cumulus hangs together as ONE cloud; fix round 1: with its cauliflower lobes): level 0 (the
  100-500 m bulges) keeps half its upward part, so a body keeps its convex relief; the finer levels keep a quarter, so
  they lift few beads beside it; the shape's low octaves decide where a cloud is. The lift keeps the rims crisp
  (erosion only left them on the shape's slope: steepness p90 41 m). A continues the shape's increment spectrum at
  62.5 m, times a factor calibrated on the observables (`cloud_detail_amplitude_calibration` 2.5: box-counted
  D = 1.278, test range 1.25-1.45; x 3 broke the cloud_check base contour). Measured on a 1.5 km cumulus layer at
  cf 0.3 (16 m voxels, level 0 / all levels): fragments under (200 m)^3 0.135 / 0.122 % of the volume (signed detail
  of M8: 0.46 / 0.48 %), 0.94 / 0.90 % of the top view (1.25 / 1.89 %); punched holes (enclosed clear < (200 m)^2)
  0.08 / 0.20 %; a level-0 lift of 0.6 / 0.8 gave 1.10 % of the top view (over the 1 % bound). Each level is
  band-limited by the pixel footprint f (w_k = clamp(log2(l_k / f) - 1, 0, 1), l_k the level's middle cell; the march pixel's angle
  comes per view): an octave below the pixel enters with its mean, so silhouettes stay irregular down to the pixel at
  every distance. Tables per detail state L = sum w_k (256 x 4, linear between rows): thresholds t[L](cf) (covered area
  = cf at every state) and edge mean factors m[L].
- **Steep edges of varying width** (static clouds Task 4): density = mean / m[L](cf) x smoothstep((v - t) / w(n)); the
  edge zone width varies log-uniformly between 20 and 40 m (`cloud_edge_width_min_m` / `_max_m` [E]: Katzwinkel et al.
  2014 inner shells; 10 m: case 9 desktop 3 px vs limit 2) by the width noise n in the shape's A channel; w_min / w_max
  = the median increment of the signed s + P over those widths (0.0207 / 0.0316; the eroded v's median, 0.0076 / 0.0137,
  made the edges ~2.7 x steeper and failed the cloud_check base contour). m is taken over the joint (v, n) distribution:
  the cell mean stays the field's (0.996-1.000 at the table states, also between them).
  Clear air without the detail (static clouds Task 10, `clouds.gdshaderinc` `cloud_extinction_in_cell`): the shapes of
  the types present are read first; where their largest value plus the largest lift the detail levels can give at the
  state (`cloud_detail_lift_bound`: sum of w_k x lift_k x a_k x (1 - detail mean), the detail value at most 1) lies at
  or below the threshold, every ramp is 0 and the sample returns 0 without the detail's three lookups - the same image
  bit for bit (render_check frames identical), the VR proxy's cloud pass -4 ms at the ground (`vr-performance.md`).
- **Shapes by cloud type** (static clouds Task 5; `cloud_type_weights` in `cloud_noise.hpp` and
  `clouds.gdshaderinc`): ERA5 / GFS carry no type, so it follows per point from the layer data as continuous weights
  (fix round 1: no seam, no pop): cirrus = smoothstep of the ice share over 0.4-0.6 (`cloud_cirrus_ice_share` +-
  `cloud_cirrus_ice_band` [E]): thin sheets (vertical tile `cloud_ice_tile_vertical_m` 2 km instead of 16 km [E]; no
  fibre direction: no wind in the snapshot); stratiform = (1 - cirrus) x the stratiform weight: per layer centre the
  smoothstep of its fraction over 0.85-0.95 (`cloud_stratiform_fraction` +- `_fraction_band` [E]) times 1 - smoothstep
  of the thickening toward the centre above over 0-0.05 (`cloud_stratiform_thickening_band` [E]); between the two
  centres around a point their mean weighted by (1 - x) and x times their fractions (the type of the cloud that is
  there: the empty centre under a deck's base has no say; a plain linear mix made that interval half cumulus and moved
  cloud_far_check's deck luminance from 9.1 to 11.1 %), blended between the snapshots like every field value:
  the shape tile stretched
  horizontally by 4 (`cloud_stratiform_stretch` [E]: 128 km across at 16 km height, a flat sheet with an undulating
  top and few, wide gaps); cumulus the rest (the shape tile). The types' edge ramps are mixed by the weights; a type
  of weight 0 costs no lookup, so outside the bands one shape lookup per sample at the type's coordinates (own
  float64 origin per tile on the CPU: `cloud_noise_origin`, `cloud_ice_origin`, `cloud_stratiform_origin`); the same
  detail and tables for every type (affine maps keep the value statistics: covered fraction and water kept per type,
  test "Coverage and water are kept for every cloud type"; each ramp has the mean m, so a mix keeps the water too,
  while the covered area grows inside a band where the patterns overlap - a bounded transition artifact (ruling
  2026-10-26), checked <= their union 1 - (1 - cf)^n + 0.05 for n mixed types: cf 0.3 -> 0.48-0.51 for two types,
  0.63-0.64 for three, where the cirrus and stratiform bands overlap). smoke_test.gd checks
  the shader's type constants against the core's (`StarsimCore.cloud_noise()["type_constants"]`). Shape tile 32 km
  across / 16 km high [E];
  Earth-fixed, the camera's part in float64 on the CPU (level k's origin fract(4^k x level 0's)).
- **Shape prefilter at large pixel footprints** (Task 10 fix round 1, anti-aliasing; `cloud_noise.hpp` "Shape
  prefilter", `clouds.gdshaderinc`): the shape texture is point-sampled per ray, right while its octaves (Worley
  8 km, Perlin 4 km to 500 m) span pixels; from the scaled-space limit a march pixel is 7-27 km and the shape aliased
  (moire, speckle, a wrong local cover). Beyond the start footprint f_s = 3.5 km of the pixel (not the light march's
  interval) the shape is read from its box-filtered mips (`CloudNoise::shape_mips`, 2^3 texel means) at the level
  lambda = clamp(log2(f / f_s), 0, 1) log2(f / 250 m), at most 2 (1 km texels), against the level's own threshold and
  edge mean factor (`prefilter_threshold` / `prefilter_edge_mean`, rows at quarter levels: covered area cf, mean water
  the grid mean - the detail band limit's rule applied to the shape). The baked volume reads the prefiltered shape for the flat levels' sub-cell pattern (`cloud_baked_flat.gdshaderinc`);
  the march kernel's start-footprint switch, the sub-pixel offsets and the march's own prefiltered variant went with the
  old march (2026-10-30). Deeper mips (level 4, 4 km texels) were tried: their few features per horizontal slice of the
  tile showed as rings around the sub-camera point. Shadow maps never prefilter.
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
- **Cloud pass** (`CloudEffect`, a compositor effect before the transparent pass; kernels in
  `shaders/cloud_pass.gdshaderinc` with `atmosphere.gdshaderinc`, `clouds.gdshaderinc`, `cloud_far.gdshaderinc`,
  `cloud_light.gdshaderinc` and the baked volume's `cloud_baked*.gdshaderinc`, compiled as GLSL compute by
  `CloudShaderPort`, which turns their `global uniform`s into one uniform block + sampler bindings filled from
  `ShaderGlobals` - every script sets globals through `ShaderGlobals.set_value`, since
  `global_shader_parameter_get` is editor-only; the shared RenderingDevice helpers in `CloudRD`):
  1. the march at 1/2 resolution (desktop) or, in VR, once in a centre view covering both eyes at 1/3, uniform in angle
     per axis (`CloudCentreView`: a rectilinear image spent ~1.9 x the pixels on the Pimax union for the same centre
     resolution, cost study 2026-10-29): one ray per march pixel through the baked volume (`cloud_baked_shade`, next
     bullets but several) up to the scene depth, in VR read from the eyes' depth buffers; the block's depth in a
     checkerboard of nearest / farthest, or the block's mean when its four distances lie within 5 %
     (`CLOUD_ONE_SURFACE_SPREAD` [E]: one surface - the ground seen from far; Task 10). Fixed sample points: an
     unchanged view gives a bit-identical image;
  2. depth-aware upsampling into a full-resolution array (layer = view): a 4 x 4 Catmull-Rom filter of the march
     pixels x the depth weight exp(-4 |ln(D / D_i)|), T clamped to [0, 1], colour >= the lowest inner tap's (per T),
     T >= 0.5 from 1 - the gathered alpha, below from the gathered T (the normalised sum of T = 1 taps with cancelling
     lobes flickered one half-float step below 1 on the moving sea); where the taps' depth weights differ by more than
     a factor 2 (`CLOUD_DEPTH_MIXED_WEIGHT` [E]: two surfaces, the checkerboard along a horizon) only the kernel's
     positive part. A VR eye first gathers a radius-2 tent at its scene point to find the taps' cloud distance, then
     the Catmull-Rom at that distance (the cloud's own disparity).
  The composite quad (`cloud_layer.gdshader` x T, its next pass `cloud_layer_light.gdshader` + the colour: first in the
  transparent pass, premultiplied; the images' .a is T, see the Sun's disk) blends it over the scene;
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
- **Centre size hysteresis (VR)** (`CloudCentreView.keep_size`, `SIZE_HYSTERESIS` 1 % [E]): the centre image size is
  rebuilt from the runtime's field of view every frame; on the Pimax (VR run 2026-10-27, per-eye target 3534 x 2838,
  divisor 3) its height sat on an integer boundary and alternated 946 / 947 rows with the FOV's per-frame jitter. Every
  new size re-created the pass's images, and the composite showed the previous frame's result until `swap_output`: the
  whole cloud layer lagged the head and jumped back (the rest of the scene steady), 86 re-creations in 20 s. A size
  within 1 % per axis of the current one is kept (the frame's own projection, the pixel angle of the kept size); a
  quality preset or a resize changes it. PerfLog columns `cloud_image_creations` / `cloud_output_swaps` /
  `cloud_low_w` / `cloud_low_h` show it (1 / 1 on a steady run).
- **Cross-fade** (`CloudLayer`, ruling 2026-10-25): two slots and the weight of slot 1; a new pair is reached through
  one visible snapshot (weight moves 1 per 2 s), the missing snapshot is uploaded into the slot without weight, then
  the weight tracks `WeatherService.blend()`. Weather off fades to the empty texture. Unused zero-weight slots are freed.
  An upload no longer needed is cancelled, and an upload never replaces a slot that carries weight.
- **Light on the ground, vehicles and the eye** (Task 9; `cloud_optics.hpp` "CPU light", `clouds.gdshaderinc` "Light
    under the clouds", `cloud_light.gdshaderinc`, the maps' fill in `cloud_baked_fill.gdshaderinc`). Three factors at a point for a light
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
    deep, in-cloud tau 3 - a synthetic test layer, far thinner than the data's low cumulus, Validation "In-cloud optical
    depths"): the direct Sun's transmittance within 0.018. Measured, f 0.3: 0.629 at 75 deg, 0.627 at 45,
    0.571 at 20, 0.404 at 8, 0.278 at 5 (maximum overlap would give 0.70 at every elevation). Without it the near map
    (the shapes) and the far map / CPU light disagreed by 10 % at 45 deg and 40 % at 8 deg (a ring at the 9-11 km
    hand-over at sunset, seen by the first tests of fix round 1). Limits: the layer depth (1 km) and the in-cloud tau
    are those of the fit; the sky factor keeps the maximum-overlap cover (its hemisphere average would see ~4 % more
    cover at cf 0.3).
  - Column per pixel: `environment::cloud_render_column_texels` (RG8 per cell and layer: R the grid-mean depth from
    20 km to the layer centre, the A channel's log code; G the largest cloud fraction above; +66 MB per snapshot, two
    bytes per texel) - the centre above (four decoded texels, bilinear: linear in tau as the CPU) + the trapezoid of the
    grid-mean extinction down to the point; cover = max(its G, the point's fraction). Godot gives a Texture3DRD of R8G8
    the luminance-alpha swizzle (R, R, R, G): the shaders read `.ra`.
  - Direct Sun on the ground: two shadow maps (`CloudShadowEffect`, a compositor effect before the opaque pass; 512^2
    RGBA32F each) orthographic along the Sun, axes perpendicular to it, anchored on the ground under the camera
    (Earth-fixed until the camera is 4 / 50 km away: no swimming). Per texel the line toward the Sun through the shell
    from the baked volume's Sun transmittance S (one read per texel, "Baked volume: cloud shadows from the bake"):
    (t_front, t_back, optical depth, cover 1) per texel; a receiver at t_r gets 1 - C + C exp(-od / C x clamp((t_back -
    t_r) / (t_back - t_front))) - exact below the clouds, linear inside them; the four texels' transmittances bilinear.
    The near map is +-16 km at 62.5 m texels, the far map +-200 km; where a line's floor point lies beyond the volume
    levels (+-410 km) the field's slanted column. Distance rule (horizontal distance of the receiver from the camera):
    near -> far over 9-11 km, far -> the column slanted to the Sun (1 - C + C exp(-tau / (C mu0))) over 120-140 km;
    smooth, no pop. World space: the same for both VR eyes. The Moon's direct light uses the slanted column. Until the
    first set is shown (start, the kernels building) the maps are invalid and the surfaces take the column.
- CPU (`StarsimCore.weather_cloud_light` -> {sun_transmittance, sky_factor, sun_diffuse}, i.e. (direct, sky, Sun's
    diffuse) in the dictionary; (1, 1, 0) without weather. `Atmosphere.cloud_light_at` and the Vector3s of the
    GPU-side checks use the order (direct, Sun's diffuse, sky): (1, 0, 1) without weather): the vehicles'
    DirectionalLight x T_s (`Atmosphere.sun_light_at`, the Moon's light likewise),
    the sky's radiance cube x (T_d E_sky + T_sd E_sun,h) / E_sky at the followed body (above 1 under a
    bright thin deck; reflections with it; `atmosphere-scattering.md`), the eye adaptation's down plate (E_n T_s + E_n mu0 T_sd + E_sky
    T_d) / pi, also for the Moon. All grid mean along the ray / column (independent columns).
  - The CPU light follows the cross-fade (final review I1): `CloudLayer` pushes (slot 0 file, slot 1 file, w1) to
    `weather_set_active` after every `_advance` when a slot changes or w1 moves by 0.01 (an empty slot is "", the empty
    snapshot: the core blends toward zero depths); a file out of the core's cache is retried every frame and
    `core_cache_missed` asks the service for a worker load. `cloud_check.gd` compares the GPU factors with the CPU
    light at five weights of a deck -> empty fade (within 3 %) and checks the plate light is monotonic. The empty
    slot's column texture is a zero LA8 volume (an R8 one reads cover 1 through `.ra`), its base texture a zero RGF
    2 x 2 (no base).
  - The CPU factors follow the GPU's clouds: `Atmosphere.cloud_light_at` returns (1, 0, 1) while `cloud_enabled` is false
    (noise not ready, nothing to show, beyond the scaled-space limit), so vehicles and the eye are not dimmed by clouds
    that are not drawn. Both snapshots' column depth and cover are blended first, then the transmittances computed
    (`blend_cloud_column_depths`; the renderer's order: `cloud_column_mean_depth`), not the mean of two transmittances.
  - Shadow darkness under broken cloud (documented limit): the Sun's diffuse part T_sd is the cell's mean and added
    uniformly - gaps get 1 + T_sd, shadows T_sd; physically the light scattered out of the clouds' sides concentrates
    under and beside them (shadows T_sd / C, the clear parts less), so the contrast between shadow and gap is a little
    too high in the gaps' favour: shadows too dark by (1 / C - 1) T_sd, gaps too bright by T_sd (the cell mean is
    right). A cf 0.3 cumulus layer, in-cloud tau 3 (synthetic), Sun 45 deg: the cell's T_sd ~0.25, the shadows' ~0.65
    in reality.
  - Weather off (cloud_enabled false): every Earth-surface pixel as before - the shader's `CLOUD_LIGHT_BEGIN` / `_END`
    blocks only change values inside `if (cloud_enabled)`; the CPU factors are exactly (1, 1, 0).
  - The terrain material now binds the cloud globals (~15 more sampler descriptors per terrain node):
    `rendering_device/d3d12/max_resource_descriptors` = 262144 (the default 65536 ran out with ~2000 nodes, i.e. ~32
    per node; 262144 holds ~8000 nodes against 1875 drawn at most in `terrain_perf`: 4x headroom). Follow-up: a slim
    terrain include that declares only the ~7 samplers the terrain uses (the others are bound per node regardless).
- **Cloud shadows on the air** (static clouds Task 9b, ruling 2026-10-27 "Wolkenschatten auf die Luft";
  `cloud_pass.gdshaderinc` `cloud_air_shadow_parts` / `cloud_air_shadow_loss`, fidelity flag `CloudLayer.air_shadow` /
  global `cloud_air_shadow`, render_check `--no_air_shadow`). The tables' air light is the clear sky's; under and beside
  clouds the Sun's direct beam is cut. Of the air light only the single scattering of the direct beam is weighted by
  the beam's cloud transmittance T_c at the scattering point; the multiple scattering (the sky's diffuse light, the
  deck's glow) stays. The single Rayleigh part comes from the atmosphere table `atmosphere_single_rayleigh` (core
  `AtmosphereTables::single_rayleigh`, `atmosphere-scattering.md`), the single Mie from `atmosphere_single_mie`, faded
  as the scene fades it: `smoothstep(0, 0.01, mu_s)` at the CAMERA (`atm_to_point_model`, `atm_sky`), so the
  unweighted sum is exactly the scene's single scattering. The march kernel integrates per pixel over the ray below the
  highest cloud top of the bound snapshots (`cloud_air_shadow_top_m` from `CloudLayer._bind_slots`, core
  `environment::cloud_render_top_m`: the centre of the layer above the highest layer with cloud fraction and
  condensate, 20 km for the top layer, 0 without cloud: no work) to the scene or the shell's exit, in 32 segments at
  (i / n)^2 [E] (16 segments in the baked march), Bruneton's difference form per segment without a clamp per piece (a
  piece the tables' interpolation makes negative belongs to the telescoping sum; the front and behind parts are >= 0
  as a whole), T_c = the mean of 2 fixed points per segment [E] from the baked volume's S (`cloud_baked_air_sun`, the
  shapes it draws within +-410 km, the slanted column beyond; "Baked volume: cloud shadows from the bake") - one
  midpoint per segment aliased the shadows into regular stripes across the sky. The segment's part in front of the
  clouds' weighted distance is seen directly, the rest through the background's T (linear split: a step at the
  midpoint stair-stepped the loss along the clouds' distance contours); the two parts are returned apart (loss = front
  + T x behind). The composite adds the air in front of a cloud at weight 1 (the march's alpha x
  `atm_to_point_offset` to the clouds' distance plus T x the scene), the same weight as the loss's front part. The
  pass's colour carries -L (clamped to +-65000; the composite's additive pass adds it). The upsample's undershoot
  floor: the inner 2 x 2 taps of this pixel's scene depth (depth weight within `CLOUD_DEPTH_MIXED_WEIGHT` of the best),
  the larger of their lowest colour and this pixel's T x their lowest colour per T (`CLOUD_FLOOR_MIN_T`): a negative
  colour belongs to its T x scene, so a cloud body next to the shadowed sky cannot take the sky's -L; 0 without air
  shadow. Static (fixed points), world space (both VR eyes), the eclipse's unocculted fraction applied. Weather off or
  no cloud in the Sun's way: L = 0 exactly. The baked air shadow's numbers: "Baked volume: cloud shadows from the bake"
  (the earlier march's with the old maps: `docs/history/weather-clouds.md`).
  
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
    exactly; its slope from the source at the slab's base - a unit depth beyond a slab thinner than 1 was clamped to
    the slab and underestimated it) x the sky irradiance there; seen from below (view cosine < 0, the cloud pass's
    flat-level column model under a layer above the eye) the same sources integrated from the slab's base, attenuated by exp(-w (tau - t) /
    |mu|): the diffusely transmitted light of the base;
  - composite as the cloud pass: alpha = C_v (1 - exp(-tau / (C_v mu))), colour = T_air C_v L + alpha S_air over
    (1 - alpha) x the ground; the ground lit through the column (`cloud_ground_light`'s column tier, the terrain's
    beyond 140 km: direct, the Sun's diffuse part, the sky's; the Moon likewise).
  - the shapes' 3D Sun light (static clouds Task 8 fix rounds 1 and 2; `CLOUD_FAR_ENTRY_TURN` 0.15,
    `CLOUD_FAR_FLANK_GAIN` 0.4 [E]; fidelity flag `CloudLayer.far_3d_light` / shader global `cloud_far_3d_light`,
    default on): the volume lights a sample whose Sun path is shorter than its depth below the column's top as through
    a surface turned toward the Sun (`cloud_entry_cosine`) - a deck's domes under a low Sun were 12-14 % brighter in the
    converged volume than the slab, broken cumulus 20-46 % (their lit flanks; more toward a low Sun: 1.20x at 73 deg,
    1.46x at 24 deg from above); the slab's normal for the Sun's light is turned 0.15 of the way from mu0 to 1 and the
    Sun's light gains 1 + 0.4 (1 - f) for the column's largest fraction f (bounded: <= 1.4). The constants were fitted
    in fix round 1 to cloud_far_check sections 3 and 6 (cf 0.4, Sun 73 deg; decks) and match the game's converged
    volume renderer, not a radiative-transfer reference (for broken cumulus 3D transfer studies find the domain-mean
    reflectance above the plane-parallel one under a low Sun and near or below it under a high one, Welch & Wielicki
    1984, Benner & Evans 2001 - the renderer's own 3D gain at a high Sun is its entry-cosine model's). Held out in fix
    round 2 with the constants unchanged: cf 0.12 under the high Sun and cf 0.4 under a 24 deg Sun (section 6) within
    6 %, and the scaled-space far field against the converged volume (section 3); the plane-parallel slab alone
    (`--no_3d_light`) is 17-31 % dark in every broken case;
  - the shapes' sunlit faces under a low Sun (baked step 7, 2026-10-30; the scaled-space far field and the baked
    march's kernels above 40 km camera height, `CLOUD_BAKED_FAR_FALLBACK` - in the one kernel the code cost the ground
        views +0.17 ms even untaken):
    `cloud_far_facet_weight` mixes into the Sun's slab radiance (one copy of the light function, a loop over the two
    normals), by w = `CLOUD_FAR_FACET_SHARE` 0.75 x s / (s +
    `CLOUD_FAR_FACET_ONSET` 0.1) x (1 + cos alpha) / 2, the radiance of a slab whose normal is the bisector of the Sun
    and the view (the faces seen lit, facing both on average); s the beam cover's slant term (0.21 max(cot e - 1.8, 0):
    0 above a 29 deg Sun - the high-Sun look bit for bit), alpha the phase angle (the lit share of a convex shape's
    visible area). Cause: the slab turned 0.15 toward the Sun (mu_e 0.27 at 8 deg) against the volume's faces lit near
    normally (`cloud_entry_cosine`); from the Sun's side the volume's Sun light was 3.2 x the slab's at 8 deg (50 km
    takeover, ERA5, mean cloud radiance flat / volume 0.40 at 8 deg, 0.49 at 15 deg; high Sun 0.90). The constants
    [E] were fitted to the volume at 8 deg; numbers per Sun elevation: `baked-lowsun-report.md`;
  Approximations: no light between columns (no shadows on neighbouring columns; the lit sides only as that mean gain),
  the light taken at one point, a Sun below the cloud's local horizon at the entry floor 0.02, view cosine floor 0.02
  (`CLOUD_FAR_MU_MIN` [E]). The CPU light factors stay (1, 0, 1) beyond the limit (`cloud_enabled` false; a camera
  there is far above any cloud). The earthshine on the Moon keeps the Bond albedo 0.306 (clouds included on average).
- **The Sun's disk through cloud** (static clouds Task 9 + fix round 1; `sun.gdshader`, `World.sun_disc_material` /
  `sun_disc_scene_value`, `cloud_pass.gdshaderinc` `CLOUD_SUN_*`, `cloud_layer.gdshader` +
  `cloud_layer_light.gdshader`): the disk is drawn at its physical radiance on the scene scale (a white Lambertian in
  full Sun = `Atmosphere.DAY_EXPOSURE` 1.5 = E / pi, so the disk's mean is E / solid angle = 1.5 x (AU / R_sun)^2 =
  6.9e4, its centre that over the limb law's mean 1 - u / 3 = 0.8: 8.67e4; it was 30, under which the clouds' own
  forward peak - the phase function's HG part, g 0.995 - outshone the disk 200 x at tau 2 and the disk vanished under
  any cloud). `sun.gdshader` scales the colour as a whole down to a largest channel of 6e4 (inside the half-float scene
  buffer): the centre clips where T_atm x the exposure gain > 0.69 - a high Sun from the ground (55 deg up: 81 500 -> 60
  000), above the haze, in space (known limit; the colour is kept). The cloud pass's images hold the background's
  transmittance T = 1 - alpha in .a (fix round 1; the march computes it without the cancellation of 1 - alpha: T_bg plus
  the hand-over's dropped step shares), and the composite draws its quad in two passes: the scene x T (`blend_mul`),
  then + the colour (`blend_add`, the next pass). Every stage (the tent / Catmull-Rom gather) is linear in T. A half float resolves T relatively: normal to 6.1e-5, subnormal to 6e-8 (measured
  on the RTX 4090: subnormals survive the compute store and the multiply blend - the disk at slant tau 12.2 is 0.985 of
  the march's T). Before, alpha was stored (spacing 2^-11 below 1) and the premultiplied blend rounds its factor 1 -
  alpha to the target's half precision too (measured: the disk stuck at 2^-11 = 4.9e-4 of itself behind every deck of
  slant tau 7.6 and more). The march continues past the scaled criterion (which stops at exp(-tau) ~ 5e-4) on rays
  within `CLOUD_SUN_RAY_ANGLE` 0.014 rad of the Sun down to T_bg 1e-9 (coverage-only sub-rays too), with the
  background's transmittance alone once T_sc < 0.01 (no light samples: no brighter disc of the last percent around the
  Sun, and cheaper). The disk follows the drawn cloud: the shapes' transmittance along the Sun's ray is above exp(-tau /
  mu) of the grid mean (1.15 x at vertical tau 2, 1.41 x at 5, 1.63 / 1.75 / 1.83 / 1.99 x at 7 / 8 / 9 / 10, mu 0.82):
  the plane-parallel bias of unresolved inhomogeneity - exp(-tau) is convex, so the mean transmittance through a
  variable cloud exceeds exp(-mean tau) (Cahalan et al. 1994, J. Atmos. Sci. 51, 2434) - accepted as physics (controller
  ruling 2026-10-27; open point below for the CPU's grid-mean value).
- **Quality presets** [E] (`CloudLayer.PRESETS`): desktop divisor 2, VR divisor 3 (the centre view; one march for both
  eyes, `vr-performance.md`), 3 detail levels each.
- **Baked volume (the only method since 2026-10-30; "Weg 2", 2026-10-29)**: `CloudBaked` (`cloud_baked.gd`,
  `cloud_baked.gdshaderinc`: its header and the includes `cloud_baked_grid` (the sets' grid block, level rule),
  `_shape` (shared shape helpers), `_sun` (S read back), `_bake` / `_light_bake` / `_shadow_bake` / `_flat_bake` (the
  bake kernels), `_volume` (the march's volume reads), `_flat` (the flat levels in the march), `_march` (the march and
  the ray's shading), `_fill` (the shadow maps' fill, the Sun probe)). Shape and light are baked
  once into a world-fixed clipmap about an anchor (the camera at the bake): 6 levels of 256 x 256 columns, level 0
  100 m (+-12.8 km) doubling to 3.2 km (+-410 km), vertical 100 / 200 / 400 m over the band of the field's cloudy layers
  (a band kernel scans the region; ERA5 2025-10-25: 0-16.8 km). Per voxel RG16F (signed edge coordinate x =
  (shape + detail - threshold) / edge width, box-averaged as a ramp over taps at <= half a shape texel; extinction
  scale), RGBA8 sqrt-coded light (Sun transmittance from a march toward the Sun: 24 quadratically spaced samples
  through the bake over the first 6 km, then 8 through the field's grid mean to the band's top - the old march's and
  the core's light model, `cloud_light_optical_depth`; step 7, 2026-10-30: through the shapes all the way a grazing
  Sun path found the gaps of broken cirrus, 1.14 x the old march's brightness per alpha toward a 5 deg Sun), the Sun's
  multiple-scattering source and the sky's source for the view from the anchor, the edge-width noise), R8 occupancy per
  4^3 cells. Per frame: one march per ray through the levels - empty blocks skipped to their exit, in occupied blocks
  clear steps from the edge coordinate's distance estimate (20 m per edge width beyond the detail's lift, >= 16 m and
  the pixel footprint, <= a cell), the entry bisected 3 x, in-cloud trapezoid steps limited by the old march's step
  weight and a quarter cell (half a cell and cell steps in clear air once the scaled transmittance is < 0.3), the
  phase function per ray (J = P T_s + J_ms), one detail fetch (level 1) near edges in level 0 (`STARSIM_BAKED_DETAIL=0`
  off; level 0's bake takes the threshold and edge mean of the state with level 1, so the march's erosion keeps the
  cloud fraction), the air in front and the air shadow's loss (16 segments x 2 points in this kernel instead of 32 x
  3). No outline pass (fix 1, 2026-10-29: the near rims' holes, stairs, wood-grain contours and shrunken, torn
  outlines came from the cell-size clear steps, the half-cell in-cloud steps and the bake's threshold; march +0.3-0.5 ms
  in the mock's fast GPU state; report `baked-fix1-report.md`). Empty blocks exit through the height shell in the
  ray's direction (rising: the top's forward crossing; descending: the bottom's first, else the top's), a root just
  behind t counting as "here" (cirrus check 2026-10-30: taking the first root ahead of t let the roots' ~1 m float32
  rounding put a ray just under a top shell past both roots, the skip then ran to an x / y face kilometres away -
  pixel holes in square rings in cirrus overhead). Ice: the bake keeps no ice channel; the ice share only picks the
  shapes (cirrus sheets) and the finer vertical taps, and ice scatters with the liquid phase function in both marches
  (`cloud_optics.hpp`'s documented simplification); report `baked-cirrus-report.md`. Diagnostics:
  `STARSIM_BAKED_DEFINES=<A,B>` (extra
  defines on every kernel), `STARSIM_BAKED_MEMORY_MB`, `STARSIM_BAKED_CELL0`. Re-bake (into the second set, ~117 dispatches of <= 3.6 ms, 21-27 ms GPU in all, one per frame) when the camera
  is 3.2 km from the anchor, the Sun turns 0.5 deg or the weather changes; the new set is cross-faded in over 8 frames
  (both sets marched, the renders mixed). Memory 216 MB per set (two sets). Beyond level 5: the flat outer levels (next
  bullet). Moon light not
  baked. Stereo mock, divisor 2, equiangular, 120 frames (frame / cloud pass / march ms): old G 35.8 / 28.4 / 20.8,
  D 78.0 / 60.6 / 48.0, H 58.5 / 45.0 / 35.3; baked G 22.0 / 8.6 / 5.1, D 27.2 / 9.7 / 5.8, H 22.1 / 8.4 / 4.8 (the
  upsample's ~3.3 ms is in both). Details, images and rulings: `.superpowers/sdd/2026-10-28-cloud-world-volume/
  baked-proto-report.md`; every kernel in `tests/cloud_compile_check.gd`.
- **Baked volume: flat levels, takeover and the level rule (2026-10-29, step 3)**: 2.5D levels of up to 1536 x 1536
  columns (`CloudBaked.FLAT_COLUMNS_MAX`; from orbit a level holds ~2 / pixel angle columns, so the footprint, not the
  extent, picks the cell), each doubling the cell, azimuthal equidistant about their own anchor, over only the visible
  cap (camera horizon + the 20 km tops' horizon, baked for 1.25 x the height + 2 km, + 0.5 deg, + 2 deg drift; the far
  side never baked or fetched). Below 40 km camera height they start beyond the volume (level 6, 6.4 km; ground 1 level
  of 320 columns); above, at the footprint's level at the nearest cloud, coarser while the layout exceeds 300 MB
  (`FLAT_MEMORY_CAP_BYTES`; 100 km: 4 levels from 400 m, ISS: 4 from 800 m). Columns no camera of the geometry's height
  range (x / 1.25) and pixel angle (x 1.25) can fetch are not baked (re-bake outside those ranges). Takeover: from 40 to
  60 km camera height the flat levels replace the volume region by region (`cloud_baked_takeover_local`: a value
  noise of 20 km cells against the height's weight, each region switching over 0.08 of the noise, so a pixel shows one
  render - the volume with the hand-over behind it, or the flat levels alone; cirrus check 2026-10-30: scaling the
  volume's extinction by 1 - w kept its shapes opaque while the flat levels' half-alpha shapes filled the gaps - grey
  translucent shapes, lower-half mean alpha 0.40 at 50 km now 0.31), above 62 km the volume is not marched and its
  light is not baked; the levels the footprint keeps are baked for S only (shape, occupancy, S - the shadows' source at
  every height; a re-bake every ~2.6 s in orbit, ~20 bake frames at 400 km, cost not yet measured in the headset) (below
  40 km the volume levels are unchanged; `STARSIM_BAKED_DEFINES=CLOUD_BAKED_FLAT_NO_TAKEOVER` keeps the volume at every
  height - its coarse voxel levels close a deck's sub-voxel gaps, so their square stayed visible against the flat
  cover). Cover and light follow the scaled-space far field (user ruling 2026-10-29: the old whole-Earth far field's
  cover is the reference; `cloud_far.gdshaderinc` `cloud_far_layer`, gain 1): per column (`CLOUD_BAKED_FLAT`) the
  field's column at 1 (cells <= 14 km) or 2 x 2 points - the nadir beam cover C_col of the largest fraction, the
  grid-mean depth, the far field's heights (steps 4-5 below) - and a pattern P: the share of 3 x 3 (4 x 4) taps whose prefiltered
  shape (the layers' depth-weighted types at the nadir height, at the taps' spacing; every second tap read unaligned from
  1.5 km cells; the taps' offsets a smooth field, no per-cell jitter) exceeds the prefilter threshold of C_col (mean
  C_col: the scaled-space far field's `cloud_far_cover`), faded to C_col itself from 6.4 to 12.8 km cells (the 32 km tile's
  lattice). F (RGBA16F) = (P, depth, nadir / grazing height x depth), T (RG16F) = (C_col, optical top x C_col) (both
  shared by the sets).
  Per pixel: the crossing of the column's height (steps 4-5 below), then F and T; the pattern taken to the view's slant
  as the far field's beam cover (1 - (1 - P)^(1 + s / n_v)), the in-cloud depth the grid mean / cloud_beam_cover_of(f,
  mu), alpha = cover (1 - exp(-tau / mu)); the per-pixel pattern, reconstruction and light: steps 4-5 below. Against the
  far field at the same camera (step 3's numbers; diagnostic
  define `CLOUD_BAKED_FAR_REFERENCE`, desktop nadir / 45 deg, 420 / 1 500 / 5 000 km, Sun 56 and 8 deg): mean alpha
  0.98-1.02, brightness 1.00-1.05 (step 2: alpha 1.09-1.14, brightness 1.23-1.45). Level rule (all levels): l =
  log2(max(r / inner, footprint / c_0)) per sample, level ceil(l), blended toward the next over a band of 0 octaves
  below 20 km growing to 1 octave at 60 km (flat levels >= 0.25). Re-bake triggers as the volume's plus the flat cap,
  the height / pixel-angle ranges and the first flat level; the flat geometry bakes in ~41 dispatches of <= 0.5 ms (ISS,
  100 km: ~4.5 ms GPU in all). Diagnostics: `STARSIM_BAKED_DEFINES=CLOUD_BAKED_FLAT_ONLY |
  CLOUD_BAKED_VOLUME_ONLY | CLOUD_BAKED_NO_BLEND | CLOUD_BAKED_FAR_REFERENCE | CLOUD_BAKED_FLAT_TAP_STATS |
  CLOUD_BAKED_FLAT_NO_TAKEOVER | CLOUD_BAKED_DIAG_NO_BAKE_PATTERN | CLOUD_BAKED_DIAG_NO_PIXEL_PATTERN |
  CLOUD_BAKED_DIAG_LEVEL` (the last: flat level / blend / optical top as colour); render_check `--centre_dump=<file>` (the march image, raw RGBA16F). Numbers and
  images: `.superpowers/sdd/2026-10-28-cloud-world-volume/baked-outer2-report.md`.
- **Baked volume steps 4-5: flat levels per pixel (2026-10-30)**: (a) the flat bake takes the field's column with
  the cubic B-spline (`cloud_column_mean_depth_smooth`, as the scaled-space far field's column model); the march reconstructs F and T
  with the cubic B-spline (four bilinear fetches each, `cloud_baked_flat_bspline`) - no cell drawn as a square.
  (b) Heights (step 5): per column the far field's own placement (`cloud_far_depth_height` of `cloud_far_seen_depth`)
  for a nadir view and for mu = 0.2, depth-weighted (F.z / F.w = height km x depth, so a clear neighbour does not lower
  them and RGBA16F does not overflow), and the optical top (5 % of the depth, T.y x C_col); the pattern's types are the
  layers' depth-weighted mix. The march finds the ray's crossing of the column's height for its view (heights-only
  fetches: fixed-point steps until bracketed, then false position, <= 4) - no entry on the highest layer's top any more
  (that top jumped by kilometres where an upper layer set in: a seam); behind the scene (the field's layers under high
  terrain) the cloud is drawn on the scene's point while its optical top is in front or it lies < 3 km (vertically)
  behind. (c) Light: the far field's column model at that
  point for the pixel's own view (`cloud_far_cloudy_radiance`), with the shapes' depth; within 0.08 of the Sun's horizon
  cosine the mean of the slab's light at its optical top, its point and as far below (the Earth's shadow edge spread
  over the column's heights instead of one sharp line). (d) The sub-cell pattern (`cloud_baked_flat_pattern`) at every
  footprint up to 8-16 km and below 5 500-8 000 km camera height (faded out before the scaled-space far field, which
  has none): 2 x 2 prefiltered shape taps over the footprint (read on a 2 km sphere: no shapes dragged along steep
  height changes) against the threshold of the reconstructed cover, anti-aliased (a tap covers clamp((s - thr) / g +
  0.5), g the shape's change across a tap: edges one pixel wide); beyond the last prefilter level the shape read scaled
  up by powers of two. The depth inside the shapes: the volume's edge ramp of the shape's excess, k = (0.3 + 0.7 r) /
  (0.3 + 0.7 E[r | covered]) with E from `cloud_prefilter_edge_mean` (thick cores, translucent margins where thin; mean
  kept), toward a closed cover (0.7-0.95) the deck texture x 1 + 3 (s - s_50); per pixel the mean opacity of the taps
  and the light for their mean depth. (e) Tile breaking: the shape read twice (as is and turned) and blended by a
  large-scale weight, histogram-preserving (`cloud_baked_shape`; step 4's domain warp sheared the shapes into marbled
  streaks). Numbers, images: `.superpowers/sdd/2026-10-28-cloud-world-volume/baked-outer3-report.md` (step 4),
  `baked-outer4-report.md` (step 5).
- **Baked volume step 6: beyond the cap, the terminator (2026-10-30)**: (a) beyond the shown flat geometry's cap
  the far field's column draws the clouds (`cloud_baked_flat_far_weight` / `_layer`: `cloud_far_layer` at the ray's
  ground point, blended in over 0.5 deg up to 3 of the last level's cells inside the cap, the flat levels faded out by
  the same weight; nothing where the scene is > 3 km (vertically) in front; evaluated once per pixel after the sets, only in
  the march kernels `march_far` / `march_fade_far` (define `CLOUD_BAKED_FAR_FALLBACK`) chosen above 40 km camera
  height - its code raised the march's register peak, ground +0.14 ms even untaken; below 40 km a camera moves at
  most ~40 km/s, the 2 deg drift allowance lasts > 5 s against a re-bake's ~0.3 s) - a camera that outran the re-bake (the free camera moves up to its distance to the surface
  per second, a re-bake takes ~0.3 s) saw the clouds end at the old geometry's cap, a circle inside the disc. A settled
  view never reaches the band (the visible cap ends 2 deg inside it). Diagnostics: `STARSIM_BAKED_STALE_GEO=<height
  m>,<offset deg>` bakes the flat geometry for that height and that far from the camera and keeps it. (b) The far
  field's column light (`cloud_far_cloudy_radiance`, flat levels and scaled-space far field) turns its entry normal continuously through the cloud's
  local horizon (mu_e = max(mu0 + (1 - mu0) 0.15, 0.02) for every mu0): the switch to the bare up at mu0 = 0 dropped
  the Sun's light ~7 x there - a hard day / night edge on the clouds. The edge now ends by the Sun's grazing-path
  transmittance and its disc's visible fraction over the Earth's horizon seen from the cloud (`atm_sun_at`; higher
  tops lit longer), the twilight sky's light going on; daytime (mu0 > 0) unchanged bit for bit. The volume levels'
  light (`cloud_entry_cosine`) was continuous already. Numbers, images: `baked-outer5-report.md`.
- **Baked volume step 7: low Sun, the limb (2026-10-30)**: (a) the light bake's Sun path through the shapes over
  6 km only, then the grid mean (bullet above; cirrus toward a 5 deg Sun 1.14 -> 1.02 x the old march per alpha).
  (b) The far field's column light gets the shapes' sunlit faces below a 29 deg Sun (`cloud_far_facet_weight`, far
  field section): at the 40-60 km takeover the flat levels matched the volume at a high Sun but were 0.40 x its cloud
  radiance at 8 deg (from the Sun's side), now 0.86 / 0.92 / 1.01 / 1.20 x at 15 / 8 / 5 / 2 deg. (c) The flat
  levels' early exit "the scene in front of the 20 km shell's entry" now uses the vertical rule of the crossing
  behind the scene (`CLOUD_BAKED_FLAT_SCENE_DEPTH_M`): near the limb the scene's depth landed in front of the entry
  along rows and columns of the depth buffer - a striped staircase of holes at 3 000 km (nadir: south limb; 60 deg:
  left limb). Diagnostics: `CLOUD_DIAG_SUN_ONLY` / `CLOUD_DIAG_SKY_ONLY` (one light alone, far field and baked
  march), `CLOUD_DIAG_NO_FACETS`, `CLOUD_BAKED_DIAG_NO_SCENE` (the scene's depth ignored). Numbers, images:
  `baked-lowsun-report.md`.
- **Baked volume: VR bugs 1 (2026-10-30)**: (a) a re-bake with nothing to bake (above 62 km every volume level
  dropped and the flat geometry carried over) finishes at once (`CloudBaked._bake_step`); it indexed the empty phase
  list before, the script error left the bake "running" and no re-bake ever started again - the centre stayed where
  the camera crossed ~90 km. (b) A flat-level crossing beyond the ray's own crossing of the model sphere (sea level)
  is behind the Earth and never drawn (`cloud_baked_flat_shade`): from below a deck the far side's crossing of the
  column's height (the ray through the Earth) passed where the scene depth let it (the 3 km vertical rule at grazing
  rays, depth-less horizon blocks) - far cap parts drawn just under the horizon line. Diagnostics:
  `STARSIM_BAKED_DEFINES=CLOUD_BAKED_DIAG_BEHIND_EARTH` (the old behaviour, those hits in red); render_check
  `--climb=<eye height m>,... [--climb_lon=<deg>]` (each step must re-bake about the new eye). Report, images:
  `vr-bugs1-report.md`, `screenshots/cloud_vr_bugs1/`.
- **Baked volume: cloud shadows from the bake (2026-10-30)**: the ground's / terrain's cloud shadows and the air
  shadow come from the baked shapes. Per set S (`cloud_baked.gdshaderinc` `CLOUD_BAKED_SHADOW`, RGBA16F 2-D array, one
  layer per volume level at 2 x the columns: 50 m in level 0; +4 MB per set): per column the line toward the Sun from
  the band's floor to its top (<= 300 km) through the baked shapes (A, every level it crosses; the grid mean beyond the
  levels) - 128 intervals looked up in the occupancy blocks, 8 A samples per interval near an occupied one, stop at
  tau 30 - storing T = exp(-tau) and tau x the lowest / highest cloudy height (premultiplied). Each level is baked
  whole, through its own or coarser levels' A (a prefiltered shadow for the air's far samples). Not the light bake's B.x:
  that exists only around occupied blocks (clear voxels store 1) and takes the shapes over 6 km only (a cloud's own
  light); a shadow is the whole line through the shapes. Baked after the light (6 more phases, 4 dispatches per
  level of 65 536 columns, `CloudBaked.SHADOW_COLUMNS_PER_FRAME`). Readers, both one S read per point: (a)
  `CloudShadowEffect` fills both maps with `CLOUD_SHADOW_BAKED_FILL` while the baked volume draws
  (`CloudBaked.shadow_fill`; the line's floor crossing -> level -> S; t_front / t_back = the line's crossings of the
  cloudy heights; beyond the levels the slanted column) - the maps' texel semantics, the terrain shaders and the
  distance rule (near -> far 9-11 km, far -> column 120-140 km) unchanged; at every camera height (2026-10-30: above
  62 km the volume levels the footprint keeps are baked for S - shape and S, not the light - but not marched, plan
  `shadow_first` / grid misc.w); before the first set the surfaces take the column; (b) the baked march's air shadow (`cloud_air_sun_transmittance` ->
  `cloud_baked_air_sun`): T_c from S within the levels (+-410 km: crepuscular rays of clouds tens of km away; the old
  maps' shapes reached 9-11 km), the slanted column beyond; a point inside the cloudy heights takes exp(-tau) scaled
  linearly in the height (the maps' rule). The air's lookup is prefiltered: S's level from the footprint at the floor
  (the samples' spacing x the rate the Sun line moves at the floor along the ray, and distance x pixel angle / max(sin
  Sun elevation, 0.1)), blended between its two levels - read at the finest level, the 16 x 2 samples aliased the
  shapes' shadows into horizontal stripes between pixel rows toward a low Sun (`s1_sunbeams_nolod_stripes.png`). Cross-fade: both sets' S mixed by its weight. A line passing above the
  floor (grazing Sun) reads the column under its lowest point. Check (diagnostic define `CLOUD_SHADOW_BAKED_DIAG` +
  render_check `--shadow_dump`): the near map against a dense 2048-sample march through A from each texel's ground
  point, V1_era5 Sun 56 / 8 deg: mean |dT| 0.008 / 0.020 (8 x 8 blocks 0.0025 / 0.008), lit / shadowed agreement
  99.6 / 98.9 %, clear lines 2 texels from any shadow lit fully on 100 / 98.0 %; the cloud's offset over its height
  0.999 / 0.994 x tan(zenith); the bake's own T_s just below the line's lowest cloud equal on median. Environment
  diagnostics `CLOUD_BAKED_DIAG_AIR_NO_LOD`,
  `CLOUD_BAKED_DIAG_AIR_SHARP`. Cost (stereo mock, divisor 2, G / D / H): the shadow pass 0.59 -> 0.016 ms, the cloud
  pass -0.1 / -0.36 / -0.31 ms, S's bake ~4-5 ms GPU in 24 dispatches of <= 0.56 ms. Crepuscular-ray residual (on /
  off over its 61-column running median, p90 - p10): cf 0.4 slab, Sun ~3 deg 2.5 % (old maps 5.5 %); ERA5 V1 toward a
  5 deg Sun 0.4 % (old 0.8 %): under a broken layer a 5 deg Sun line crosses ~10 cells, so nearly all air below it is
  shadowed by the shapes (the old far map's grid mean let 1 - C through everywhere). Numbers, images:
  `baked-shadow-report.md`. (c) The vehicles' and the eye's direct Sun (2026-10-30): the Sun probe
  (`CloudShadowEffect.request_sun_probe` / `sun_probe`, kernel `CLOUD_BAKED_SUN_PROBE` in `cloud_baked_fill.gdshaderinc`:
  one work group of up to 64 named points, S read as the air shadow reads it, `buffer_get_data_async` - the value one to
  two frames late, never a stall; its callback is a method of a RefCounted state, not a lambda: a lambda's captures
  outlived the scripts at the engine's exit and crashed it) through `CloudLayer.direct_sun_transmittance(key, point)`
  into `Atmosphere.cloud_light_at`'s direct beam (`Atmosphere.direct_sun`, wired by `World`; keys "sun_light" - the
  vehicles' DirectionalLight - and "eye" - the adaptation's plate); NAN (the CPU's grid mean) beyond the levels or
  without a set. cloud_check section 3: a vehicle in a cumulus' shadow 0.119 vs 1.375 in the gap beside it (Sun light
  energy), the grid mean 0.490 at both.
- **Divisor (F8)** (2026-10-29 / 10-30): `CloudLayer.divisor_override` (F8 cycles 1, 1.5, 2, 3, 4;
  `--cloud_divisor=<x>`, fractional allowed) - the one live toggle; the HUD line (`status_line`) shows the divisor and
  "(Volumen wird gebacken)" only while a bake really runs (kernels built, no set shown yet); "(Wolken-Shader werden
  gebaut)" while the kernels build, "(keine Wolken aktiv)" when `cloud_enabled` is off (nothing bakes), "(Wolkenvolumen
  nicht verfügbar)" after a kernel failure. The baked volume is the only cloud method (user ruling
  2026-10-30; F7's method switch and F9's march cut removed with the old march).
- **Kernel build and fast start** (2026-10-30; `CloudKernels`, `game/world/weather/cloud_kernels.gd`): every cloud
  compute kernel (the cloud pass's, the baked volume's, the shadow maps') is ported (CloudShaderPort) and compiled on
  WorkerThreadPool workers, one task per kernel in parallel: glslang to SPIR-V, then the driver binary
  (`shader_compile_binary_from_spirv`; on D3D12 the SPIR-V -> DXIL translation, which was as slow as glslang: creating
  creating 9 march shaders from cached SPIR-V took 6.8 s, from the binary 0.02 s).The binary is cached under
  `user://shader_cache/clouds/<sha256>.bin`, the key the final GLSL source (defines in its header) + Godot version and
  build hash + rendering driver + device name; written to a unique temp name and renamed (parallel checks) only when the
  write succeeded and the file has the full length. A cached binary the RenderingDevice rejects (truncated, corrupt, a
  container the engine no longer reads) makes the owner call `rebuild_rejected`: every kernel of that set read from the
  cache is compiled again and its file replaced (once per set; log `CloudKernels <set>: a cached binary was rejected -
  rebuilding n kernels without the cache`), the frame meanwhile as while building. Pruning: once per process (first
  worker) binaries last used 30 days before the newest one (not before now: a long break keeps the cache) and temp files older than an hour are removed (a hit older than a day rewrites
  its file, so the file time is the last use). The build state is a job object the worker tasks hold a strong reference
  to (never a freed object), and a set freed with its group still running (quitting during a build: the scene tree goes
  before the engine waits for the workers) waits for it in its predelete - the one blocking wait; an unreleased running
  group hung the engine's exit (cold build, quit at frame 5; now a clean exit after ~4 s). Workers load nothing: the
  include files' text is read on the creating thread (a worker's `load()` can wait for the main thread's queue, the
  likely cause of the earlier deadlocks of a blocking wait). At start the upsample, the baked volume's kernels (for the stereo march when an XR
  interface is up; the first frame corrects a wrong guess; a later change of the view count re-creates the baked volume
  for it) and the shadow maps' fill and Sun probe are built. Shaders are created from the binaries on the render thread
  when first needed. No frame blocks on the workers (owners poll `done()`; only a freed set's predelete waits): until
  the kernels and the baked volume's first set are ready the frame shows no clouds (`CloudEffect._skip_frame` clears
  the result to T = 1). The loading screen
  holds "Wolken-Shader übersetzen (einmalig nach einer Änderung, einige Sekunden)" (a kernel had to be compiled) or
  "Wolken-Shader laden" while the kernels build (`CloudLayer.kernels_status`, Main's hold); Boot's step
  before instantiating Main is "Hauptszene aufbauen". Measured (desktop RTX 4090, Start-Desktop args, engine start to
  "[Boot] done" = the first world frames): before the fast start 23.70 s (19.6 s of it the serial build in Main's
  instantiation); with the old march still built in the background 4.06 s (cache empty) / 4.37 s (warm); after the
  cleanup (2026-10-30, warm cache) the world shown at 3.91 s, "[Boot] done" at 3.96 s (twice), all cloud kernels from
  the cache in 0.01-0.02 s.

## Validation

- In-cloud optical depths (static clouds Task 10, the 48 shipped analog files, format 7; the body-core vertical tau of
  each column's contiguous cloudy run, sum of beta / m(cf) dz - the renderer's plateau -, weighted by cos(lat) x the
  run's max cf; script and table: sdd `tau_inv/t10_cloud_tau.py`, `t10_analog_v7.log`). p10 / p50 / p90:
  low cumulus 25 / 102 / 257 (by run depth: < 0.5 km 2.3, 0.5-1 km 22, 1-2 km 67, 2-4 km 157); low stratiform
  22 / 97 / 309 (by run depth: < 1 km 20, 1-2 km 44, 2-4 km 108, > 4 km 178 - the deep frontal columns carry the
  median); mid stratiform 19 / 44 / 99; mid cumuliform 7 / 30 / 92; cirrus (ice > 50 %) 0.18 / 2.3 / 12 (runs < 1.5 km
  0.05). The < 1 km low stratiform bin (median 20, observed St / Sc 5-30) does not open the "overcast too dark" finding
  (threshold 40); the deep runs are thick by their water, not by the renderer.
- Real-date plausibility (final review I4, `test_weather_snapshot.cpp` "Real-date columns ...", fixture
  `era5_plausibility.toml` by `tools/validation/gen_weather_fixtures.py`: ERA5 columns of 2024-04-08 18 UTC and
  2025-08-15 18 UTC at Starbase, the Cape, the North Atlantic, the warm pool and the most overcast cell near Starbase,
  with the store's own `total_cloud_cover` and `cloud_base_height`; the build gets the tcc). Cloud base (lowest layer
  with fraction > 1 % and condensate, ERA5's definition, on the fractions as the data has them: the cover scaling lifts
  a trace layer over 1 % - the Cape 2025, 0.75 % 830 m under the base - without moving cloud) within one layer plus the
  spacing of the pressure levels around ERA5's cbh, and a boundary-layer base no more than one layer below the column's
  LCL: all ten columns pass. Cover (Task 9c): the maximum-random cover of the snapshot's layers is tcc within 0.02 per
  column (measured -0.002 .. +0.012, mean deficit 0.002; test mean <= 0.03); the renderer's maximum overlap stays
  below it. Cumulus water added: 0-1.27 kg/m^2 (the deep convective Starbase 2025 column the most).
- Core `[weather]` Task 9c (`test_weather_cover_water.cpp`): Gamma_l = a numerical parcel (0.1 m steps of c_pd dT +
  g dz + L dr_s = 0, finite differences of r_s) within 0.3 % at 268-298 K, and within 1-2.5 g/m^3/km at 275-290 K /
  800-950 hPa; a shallow cumulus reaches the sub-adiabatic path exactly with its profile kept; monotone in f_ad; a deep
  run's parcel ends at 3 km; the in-cloud bound reached and never exceeded, a denser layer kept; decks, ice,
  deep-enough cumulus, fog / unknown / high LCL, a run above the boundary layer, the second run and a deck under
  rising cumulus keep their water; the half-stratiform weight gives half the factor; the cover scaling (two separated
  clouds 0.44 -> 0.3 / 0.6 / 0.95 / 1.0 within 1e-9, ratio kept, water kept, maximum overlap of touching layers,
  unknown / clear / tcc 0, dry layers, buried layers); a built GFS column scaled from 0.4 to 0.8 within 0.01 with the
  water diagnostic; .wthr round trip of tcc and added water, box means; a checkerboard box scaled back to its 0.5.
- Core `[weather]`: render texels (linear extinction, in-cloud depth vs trapezoids, code round trip, ice share);
  vertical profile (flat base, water path kept exactly); covered fraction = cf within 0.003 at every detail state (0.006
  between rows); steep edges of varying width, 10-90 % rise median 7 m (< 10 m), 90th percentile 21 m (<= 40 m, the
  width maximum) (mean water / grid mean 0.991-1.005 for cf 0.05-1 at states 0, 1, 1.5, 3); the shape prefilter (level 0
  up to f_s and continuous, the mips' box filter, level 0 = the unfiltered sample and state-0 tables bit for bit;
  covered fraction within 0.003 and mean water / grid mean 0.995-1.007 at levels 0.125-2 for cf 0.1 / 0.4 / 0.9, 4e5
  random positions); perimeter fractal dimension by box counting of the cloud mask (16 km plane at 4 m, cf 0.5, boxes 8
  m to 1024 m: D = 1.278, test 1.25-1.45, Lovejoy 1982; the shape alone 1.01); one connected body per cumulus
  (26-connected components of a 8.2 x 8.2 x 1.5 km layer at 16 m, cf 0.3 / 0.5, states 1 / 3: fragments under (200 m)^3
  < 0.2 % of the volume [E], measured 0.07-0.14 %; top view fragments < 1 % (0.09-0.94 %), punched holes < 1 %
  (0.08-0.34 %)); cloud types (pure weights on canonical columns, sum 1, no weight change > 0.035 per 1e-3 step of
  fraction, fraction above, ice share or layer coordinate (measured 0.030); covered fraction within 0.003 and water /
  grid mean within 1.5 % for cumulus, stratiform and cirrus coordinates at states 0 / 3, cf 0.3 / 0.9, water also for
  three mixes); blue noise (each value equally often, 4 x 4
  block-mean variance 0.18 of white noise); 3D Sun light (light march unbiased within 1 %, shadow-side transmittance exp(-mean tau) within 2 % of
  Beer-Lambert; sunlit flank 6.9x / 52x the shadowed one and 3.9x the column model at +10 / -3 deg Sun; a slab gives
  the column source exactly); sky source R + T = 1.
- Core `[weather]` transfer table (`test_cloud_transfer_table.cpp`): the 42 bins of `cloud_slab_radiance_mc.csv` (Sun
  0 / 60 deg, tau 1-30) within 4.5 % (bound 5 % or 3 sigma; the closed form up to -56 % in the forward bins); slab albedo
  vs `cloud_slab_albedo_mc.csv` (2e6 photons) within 1.1 % for tau 1-50, mu0 1 and 0.5 (closed form up to -32 %);
  semi-infinite R = 1.003 / 0.995 / 0.998 / 0.995 at mu0 1 / 0.5 / 0.2 / 0.1 (bound 1 %); the uniform-slab limit
  (source = single + table at (t, tau* - t, mu0); its ray integral = `slab_reflected_radiance_mc` within 0.2 %); the
  table's Draine weight = the core's phase function's; loader refuses foreign / truncated files; GPU packing.
- Core `[weather]` light under the clouds: T_total = 1 - albedo of the Monte Carlo fixture within 3.2 % (10 cases;
  the LUT likewise), LUT monotone / 1 at tau 0 / continuous at its last node; white-sky vs two-stream within 10 %
  (4.6 % measured); overcast tau 30 at Sun 45 deg 0.277 in 0.2-0.3 (Kasten & Czeplak 0.25); the Sun's diffuse part under
  a uniform slab = T_total - exp(-tau / mu0) (table and delta-Eddington), none above it or with the Sun down; a broken
  layer (cf 0.4) as independent columns (direct, diffuse, sky); column texels = `cloud_column_depths_above` at the
  centres within the code's 3 % (depth) and 0.003 (cover).
- GPU `cloud_check.gd` (2026-10-30: the ground light and the shadow maps on the baked volume's S; 62 checks PASS,
  79 s): section 1 (terrain_earth.gdshader on a quad 300 m from a camera 2 m up; slabs tau 1 / deck tau 3 / broken cf
  0.4, Sun 45 / 20 deg): the S maps' direct Sun on the closed slabs at 0.927-0.937 x the core's slant optical depth
  (tolerance `S_DEPTH_TOLERANCE` 10 % [E]; open point), the Sun's diffuse share and the sky's within 0.6 % of the CPU,
  the weather on / off luminance ratio within 2.7 % (closed slabs), the ground's total light vs the CPU's within 2.4 %;
  the broken layer's spot reported (Sun 45 deg: in a cumulus' shadow, S 0.31 vs the cell's 0.49; Sun 20 deg: in a
  gap, 1.0 vs 0.44); a deck -> empty cross-fade: column terms within 3 % of the pushed pair's CPU light, the light with
  the grid mean's direct Sun monotonic (0.79 -> 0.99); weather off bit-identical (land and sea). Section 2 (the probe
  shader, eight sites): near -> far map hand-over 9-11 km steps <= 0.058 (allowed 0.18-0.24), far map -> column
  120-140 km on the deck <= 0.008; the broken layer's 120-140 km reported (open point: S's coarse levels); the shapes'
  disc mean vs the CPU's cell cf 0.15 / 0.3 / 0.4 at Sun 45 / 8 deg 0.793 / 0.777, 0.658 / 0.649, 0.626 / 0.592, 0.405
  / 0.396, 0.515 / 0.490, 0.278 / 0.277 (tolerance 8 %), near vs far map within 0.03, column terms within 0.6 %.
  Section 3 (the vehicles' direct Sun): along a 4 km line under the cf 0.4 layer at a 45 deg Sun the probe reads T
  0.086 in a cumulus' shadow and 1.000 in the gap 1.2 km away, the Sun light energy 0.119 / 1.375 (0.91 of the clear
  value), the CPU's grid mean 0.490 at both; probe vs near map at 41 points mean |dT| 0.0045, worst 0.055 (a shadow's
  edge, the map's 62.5 m texels).
- GPU `cloud_far_check.gd` (2026-10-30, sections 1-4, 14 checks PASS): the far field's slab radiance vs Monte Carlo
  (72 geometries), weather off bit-identical, the scaled-space hand-over against the baked volume's flat levels: deck
  tau 10 / tau 2 worst 0.26 / 0.51 %, broken cf 0.4 (21 x 21 blocks) 2.27 % (tolerance 5 %), the Moon's distance.
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
- `game/tests/cloud_layer_test.gd` (headless): the cross-fade never jumps, never changes a weighted slot. Also passes
  with a GPU (non-headless): there the slot uploads take real frames, so the world-volume case's first pair gets 12 s (it
  failed on "pair resident at its blend" at 6 s, 2026-10-30). Exit leak check: `render_check` prints no "RID leaked" warning (the effect's per-view `volume_buffer` was not freed, fixed 2026-10-30).

## Diagnostics

- Stereo harness (world volume plan Task 1): `render_check.gd --stereo=<json>` renders through the multiview path on a
  stand-in headset (`StereoMockXr`: two views from recorded per-eye projections, eye transforms and render target; the JSON
  is written by `PerfLog` at a VR start, `Start-VR.cmd --perf-log`, as `data/perf/xr_projections_<stamp>.json`, to be kept
  as `game/tests/data/pimax_dream_air_projections.json`; until then
  `game/tests/data/pimax_dream_air_projections_provisional.json`: the stereo test's FOV_LEFT_EYE mirrored, IPD 0.064 m, 3534 x
  2838 per eye, NOT the headset). `XrManager._ready` enters VR mode on that interface (the main scene builds its VR panel,
  which is in the eye images), the VR cloud preset is the default, the eye images are `<out>_eye0.png` / `_eye1.png`, the
  log shows `stereo view_count 2`. Run it WITHOUT `--xr-mode off` (that switches the multiview shader variants off: black
  images). `--cloud_view=<id>` prepends a gate view's arguments (`CloudViews`, ids `V1 V2 V3 V4_20km V4_80km V5_nadir
  V5_limb V6_backlit V6_sunset V7` and, for V1 V3 V4_* V5_*, `_analog` / `_era5`; the ERA5 snapshot is
  `data/weather/cache/era5_1102887.wthr`, 2025-10-25 15:00 UTC; V6_sunset: Sun 3.01 deg up at 2024-04-10T00:26:00 UTC; V7:
  `--view=1` is the booster); command-line arguments win. `--frames_every` also runs with `--mission` / `--at_event`.
- `render_check.gd --weather=<abs path .wthr>` or `--weather=slab:<bottom>,<top>,<liquid kg/m^3>,<fraction>` (pinned,
  no service; the overlay shows the pinned snapshot), `--weather=analog` (the shipped analog of `--utc`, pinned),
  `--weather[=s]` / `--weather=online` online; any other value exits with an error. `--cloud_quality=desktop|vr`,
  `--cloud_detail_levels=0..3`, `--cloud_scattering=table|closed`, `--cloud_divisor=<x>`, `--no_air_shadow`,
  `--climb=<eye height m>,...`, `--shadow_dump=<file>`, `--centre_dump=<file>`,
  `--look_at_sun [--away_from_sun]`;
  `--fixed_exposure --deterministic[=64]` for before / after image identity (exactly that many frames once settled,
  the HUD's frame-rate line hidden; the cleanup's proof set, 2026-10-30: V1_era5 ground, 3 km under an overcast slab
  at 5-7 km, V3_era5 at 10 km, V4_80km_era5 at 90 km, V5_nadir_era5, V6_sunset, the stereo mock - two runs of the same
  code can differ in 3 HUD-panel pixels by 1/255).
- `terrain_perf.gd --weather=<abs path .wthr> --cloud_quality=vr [--cloud_scattering=table|closed]` (weather off
  otherwise).
- Kernel build (`CloudKernels`): each set prints `CloudKernels <set>: n kernels built in x s (k compiled, m from the
  cache)`; `CloudEffect: baked volume's shaders created in y s` on first use. `STARSIM_CLOUD_SHADER_CACHE=0` neither reads nor writes the cache (a first start's
  build); delete `%APPDATA%/Godot/app_userdata/Super-Heavy-Rocket-Simulator-VR/shader_cache/clouds` for a cold start.
  `tests/cloud_compile_check.gd` (GPU) builds and creates every kernel (the upsample, desktop and stereo baked, the
  shadow maps' fill and Sun probe) by polling, exit 1 on a failure.
- Cell bake CPU reference (core `cloud_volume_bake.hpp`, `StarsimCore.weather_cloud_cell_bake[s]`, `[weather]` tests,
  smoke_test): kept from the removed world volume; its GPU side and the rest of that work in the history file.
- Real sessions (VR or desktop): `Start-VR.cmd --perf-log` (`PerfLog`, `game/util/perf_log.gd`) writes
  `data/perf/perf_<date>_<time>.csv`: one row per second (frame interval p50 / p99 / max, root viewport GPU / CPU render
  time, main-thread script time, camera altitude, cloud far field / upload state, draw calls, VRAM) plus a "spike" row for every
  frame over 22 ms.
- `cloud_check.gd` with `STARSIM_SLANT_TABLE=1`: only the beam-cover fit table (shapes' direct Sun, near disc, eight
  sites, f 0.15-0.6 x e 5-75 deg).

## Open points and known limits

- **Standing decisions (user 2026-10-27, static clouds Task 10)**: with clouds the frame rate is lower - the 2.5 ms
  cloud budget is not met (cloud pass 37 / 45 ms in the proxy at the ground / the deck, `vr-performance.md`) and the
  user accepted that until a better approach exists (e.g. precomputing the static field). March sub-samples: 6 (4 fail
  the Cape's grain bound, 5 pass with no margin). March divisor 3 (4 rejected: softer cumulus, blockier deck outlines).
  Shape de-tiling NOT done: the lattice stays (options and costs below).
- **Open: Task 10 step 3, the VR run with the user (not done)**: pad -> decks -> orbit with `--perf-log` in the headset:
  the cloud pass p99 and the frame rate at the VR pixel count, the look check against the anti-goals of
  `cloud-visual-reference.md`, the composite's second pass A/B (Task 9), the light-cache fill's worst frame under load
  (Task 2), the quiet-machine gate. Plan: `docs/superpowers/archive/plans/2026-10-26-static-clouds.md`, Task 10 step 3.
- **Planned: milestone M11 (after M7 cockpit and M10 sound design)**: the rest of the weather. Part 2: rain, snow, fog,
  lightning, weather-dependent haze (the small variant: climatology AOD plus humidity growth with a few precomputed
  table sets), city light on clouds. Part 3: GOES cloud patterns. Part 4: wind / density and launch rules. Open
  action: compare the grid-mean liquid water path with the MAC-LWP monthly climatology (same months / ocean regions).
- **Deferred minors (static clouds final review)**, none of them blocking:
  
  
  
  
  - No service-level test that an old-version weather file heals; `map::operator[]` in checks (Task 6 M6 / M7).
  - The LCL cut scale is computed per load, untimed; the RG32F base texture is created synchronously on the main
    thread (cloud_layer.gd, 8 MB at 0.25 deg) while the field textures are spread over frames: an unmeasured
    main-thread stall at every snapshot change (Task 7).
  - The checks restore a hard-coded FOV of 60 deg (cloud_check, cloud_far_check; Task 8).
  
  - Task 9b: the telescoping reference shares `cloud_air_single_at` with the loss (no independent check); the outline
    tolerance p99 2 % against 1.67 % measured; the upsample floor falls back to 0 when no inner tap passes the depth
    gate; 16-entry tap arrays with 4 used in the radius-0 path (register pressure); a per-region cloud top for the air
    shadow (the global top gives no gain on real weather).
  

- **Shape de-tiling (Task 9d, deferred by the user 2026-10-27)**: the shapes' 32 km tile shows as a chessboard lattice
  from altitude (the shapes' own period, not the weather). Measured options on 6b9149e (cloud pass ground / 400 km;
  tiled 40.9 / 6.2 ms): blend of two reads in a 0.1 band (c7fe2ca) 97.8 / 11.9 ms (+139 / +91 %), in a 0.05 band (V4)
  72.9 / 10.2 (+78 / +64 %), smooth warp A = 0.33 (V5) 47.3 / 7.3 (+16 / +18 %, shapes stretched locally: rejected as
  too slow for the look it buys). The lattice stays. The attempt is parked on the local ref `wip/t9d-detile`, the
  variants in the sdd patch `t9d2_variants.patch`; no plan task.
- **Low-Sun base over-lighting (Task 9b secondary)**: the entry-cosine clamp gives cumulus bases under a 15-deg Sun
  b ~13 from the vertical column (debug source 0.167 /sr vs 0.012-0.018 of the slab); a cap needs a 3D Monte Carlo box
  reference first - not built (the air shadow met the acceptance alone).

- **Earlier VR notes and the Task 11 gate**: the VR centre-view design (`CloudCentreView`, divisor 3, a 4 x 4 tent per
  eye) stands; the open point that the tent is fixed in screen pixels (far clouds get the near clouds' blur, user
  2026-10-26) stays. The absolute Task 11 gate (p99 <= 11 ms) was not re-run: superseded by the Task 10 profile and the
  accepted frame-rate cost (`vr-performance.md`); the old numbers (VR preset 48 / 96, divisor 4) are in
  `docs/history/weather-clouds.md`.
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
- The transfer table is a plane-parallel slab result on a tilted independent column: light is not moved between
  columns (no light leaking around a cumulus' shadowed side beyond the entry-surface tilt); the HG peak is treated as
  forward (its ~0.3 deg width is not resolved in the multiply scattered light). Ice uses the droplet table.
- Ice uses the droplet phase function and thin sheets without a fibre direction; no ground-reflected light into the
  cloud bases; the sky light kept in the forward peak is left out.
- A resize shows the clouds of the old size for one frame (scaled), then the new ones.
- The Sun's direct beam: the drawn disk follows the shapes (above exp(-tau / mu), the plane-parallel bias); the
  vehicles' and the eye's direct Sun follow them too through S (decided 2026-10-30, next bullet but one); the CPU's
  `weather_cloud_light` `sun_transmittance` (the grid mean's exp(-tau / mu); tau 5 at 55 deg: 0.00235 vs the disk's
  0.00331) remains their fallback beyond the volume levels.
- The Sun's disk is clipped at a largest channel of 6e4 (half-float scene buffer) where T_atm x the exposure gain >
  0.69 (high Sun, above the haze, in space; by up to 31 %), its colour kept; through cloud it is dimmed from that.
- Light (Task 9, decided 2026-10-30, coordinator ruling "shadows uniform"): the vehicles' and the eye's direct Sun
  follow the drawn shapes - S at the light point through `CloudShadowEffect`'s Sun probe (one work group, asynchronous
  read-back, one to two frames late), the CPU's grid mean only where S has no value (no set shown, beyond the volume
  levels' +-410 km); their diffuse parts (the Sun's diffuse share, the sky's) stay the column's (`cloud_light_at`).
- Far field: plane-parallel independent columns (the shapes' 3D Sun light only as a mean gain matched to the converged
  volume, switchable: `CloudLayer.far_3d_light`; no shadows
  cast on neighbouring columns or the ground beside them); in the scaled-space Earth the field's 0.25 deg grid shows as
  soft cells near the hand-over (~4 pixels per cell at 1.5e7 m; its column stays bilinear, without the cloud pass's
  sub-grid cover). A column with several layers is one slab (placed by its seen light's mean
  depth): from the ground, a high layer far out and a low one share one crossing.
- **S against the core on a closed slab (2026-10-30, cloud_check section 1)**: the maps' S gives 0.927-0.937 x the
  core's slant optical depth through the 1 km slabs (tau 1 and 3, Sun 45 / 20 deg; T 0.266 vs 0.242 at tau 1 / 45 deg)
  - a property of the bake through the voxelised shapes (cause not pinned: the levels' vertical cells at the slab's
  faces, the edge ramp's mean), also in the air shadow and the vehicles' direct Sun. The check holds it at 10 % in
  depth; open: whether to correct the bake.
- **S's coarse levels on broken cloud (2026-10-30, cloud_check section 2)**: at 110 km from the camera the far map's S
  comes from volume level 4 (1.6 km cells, S at 800 m), whose smoothed shapes shadow more than the shapes do - cf 0.3
  at a 45 / 8 deg Sun 0.556 / 0.226 against the slanted column's 0.591 / 0.395, so the 120-140 km hand-over to the
  column steps by 0.03-0.09 on the ground (a faint ring, most at a low Sun; visible from altitude). Reported, not
  asserted; the same S was the game's below 62 km since the shadow step, now at every height.
- **Above 62 km the volume levels are baked for S only** (shape and S, not the light): a re-bake in orbit (every
  ~19 km of flight at 400 km, ~2.4 s) runs the density / occupancy / S phases of the kept levels (3 levels from
  400 km at the desktop's pixel angle) - more bake dispatches than before (only the flat geometry), each <= ~2 ms GPU.

