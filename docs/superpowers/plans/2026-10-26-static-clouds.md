# Static Clouds Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Clouds that are computed once and only re-shown - no flicker, crisp cumulus outlines, fog inside, shapes by
cloud type, flat bases at the lifting condensation level, a seamless near / far hand-over - within 2.5 ms of VR GPU.

**Architecture:** (A) the light toward the Sun is cached in camera-anchored Earth-fixed 3D textures, filled
progressively, instead of a light march per sample per frame; (B) the view march becomes deterministic (no per-frame
jitter) with refined silhouettes, steep edges and per-type shapes; (C) the snapshot carries 2 m temperature / dew point,
the core moves boundary-layer condensate up to the LCL, and the cloud pass hands distant pixels to the far field's
column model. One path for desktop and VR.

**Tech Stack:** C++20 core (environment module, Catch2 `[weather]`), Godot 4.7 double GDScript, RenderingDevice compute
(GLSL via CloudShaderPort), D3D12.

**Spec:** `docs/superpowers/specs/2026-10-26-static-clouds-design.md` (approved by the user 2026-10-26; read its
"user's definition of a cloud" and "Decisions taken for the builder" first).

## Global Constraints

- Physics first: every cached or simplified quantity reproduces the per-frame reference within a documented tolerance;
  fidelity flags keep the reference path switchable (`cloud_light_cache_enabled`, `cloud_deterministic_march`).
- Budget: cloud cost p99 <= 2.5 ms GPU in VR in every phase, light-cache fill included (spec decision 6).
- Invariants: covered area = cloud fraction and mean water = grid mean at every detail state, per cloud type (spec
  decision 3); the column water path is conserved by the LCL redistribution (spec decision 2).
- Stereo: `cloud_vr_stereo_test.gd` stays green; both eyes read the one centre image.
- One path for desktop and VR; only `CloudLayer.PRESETS` differ (resolution, budget).
- SI units, frame / unit in names, no magic numbers (`[E]` with rationale, physical constants in `constants.hpp`).
- Data: ERA5 `2m_temperature`, `2m_dewpoint_temperature`; GFS `TMP` / `DPT` at 2 m above ground - check
  `tools/data_pipeline/sources.toml` covers them, add if not.
- Commits: Conventional Commits, push to `main` after each green task. Tests: only the affected tags / scripts
  (`[weather]`, `cloud_check.gd`, `cloud_far_check.gd`, `cloud_layer_test.gd`, `cloud_vr_stereo_test.gd`,
  `smoke_test.gd`); GPU checks need a window (no `--headless`).
- Docs: `docs/subsystems/weather-clouds.md` updated per task (current state; superseded text to
  `docs/history/weather-clouds.md`), `docs/subsystems/vr-performance.md` with the measurements.

## Review Focus

- A Sun-direction change during a cache fill (sunset in time warp): no frame may mix two Sun directions inside one
  cascade without the cross-fade; the old generation stays until the new one is complete.
- Camera jump (Tab to another site, mission start): all cascades re-anchor; until filled, the march falls back to the
  per-sample light march (no black / unlit clouds for a frame) - test in Task 2.
- Snapshot cross-fade (weather hour change) while a cache refresh runs: the cache follows the blended field it was
  filled from; a blend step larger than the refresh threshold triggers a refill (Task 2).
- Moving camera without jitter: no shimmering of far broken cumulus (the hand-over and the deterministic footprint
  average, spec decision 4) - image pair check in Task 8.
- Old snapshot files (format version 1) in the cache after the update: rejected and re-downloaded / rebuilt, never
  misread (Task 6).

---

### Task 1: Light cache - one cascade, fill kernel and lookup

**Files:**
- Create: `game/world/weather/cloud_light_cache.gd` (class `CloudLightCache`: the cascade textures, the fill dispatch)
- Create: `game/shaders/cloud_light_cache.gdshaderinc` (`CLOUD_LIGHT_FILL` kernel; `cloud_cached_light_depth()` lookup)
- Modify: `game/shaders/cloud_pass.gdshaderinc` (`cloud_light_source`: cache lookup when enabled),
  `game/world/weather/cloud_effect.gd` (bind the cache), `game/world/weather/cloud_layer.gd` (owns the cache, global
  `cloud_light_cache_enabled`)
- Test: `game/tests/cloud_check.gd` (new case 8)

**Interfaces:**
- Produces: `CloudLightCache.new(size: Vector3i, extent_m: Vector3)`, `func fill(rd: RenderingDevice, anchor_earth_m:
  Vector3, light_dir_earth: Vector3, budget_voxels: int) -> bool` (true when complete), `var texture: RID`
  (R32F: `tau_light` toward the light from the voxel centre, the same quantity `cloud_light_optical_depth` returns);
  GLSL `float cloud_cached_light_depth(vec3 offset_local, float height_m)` (trilinear; returns -1.0 outside the
  cascade so the caller falls back to the march).
- The fill kernel evaluates `cloud_light_optical_depth` at each voxel centre with a converged sample count
  (24 near + 8 far, no jitter: the stratified midpoints) - the cache is the converged reference, not a noisy one.

- [ ] **Step 1: Test** - `cloud_check.gd` case 8: a slab of tau ~10 and a broken layer cf 0.4, Sun 45 / 20 deg: the
  Sun's scattered radiance (debug output 2) with the cache vs the per-sample march with 24 + 8 converged samples
  (mean of 8 frames) within 5 % (slab) / 10 % (broken: the voxel size against the shapes) at every pixel of a 64 x 64
  view; a pixel outside the cascade equals the march exactly.
- [ ] **Step 2: Run, expect FAIL** (cache missing).
- [ ] **Step 3: Implement** one cascade (128 x 128 x 64 over 8 km x 8 km x the shell, [E] start values, measured in
  Task 10) filled at once (budget = all voxels).
- [ ] **Step 4: Run** `cloud_check.gd` (all cases PASS), `cloud_layer_test.gd`, `smoke_test.gd`.
- [ ] **Step 5: Commit** `feat(game): cloud light cache - the light toward the Sun looked up, not marched`

### Task 2: Cascades, re-anchoring, refresh and cross-fade

**Files:** `game/world/weather/cloud_light_cache.gd` (cascades, toroidal slabs, generations),
`game/world/weather/cloud_light_cache_schedule.gd` (pure logic: what to fill next), `game/shaders/cloud_light_cache.gdshaderinc`
(cascade selection, generation blend), test `game/tests/cloud_light_cache_test.gd` (headless)

**Interfaces:**
- `class_name CloudLightCacheSchedule` (RefCounted, no GPU): `func update(camera_earth_m: Vector3, light_dir: Vector3,
  blend: float, clock_jumped: bool) -> Array[Dictionary]` returning the fill jobs `{cascade, region: AABB(voxels),
  generation}` within the per-frame voxel budget; constants `REFRESH_ANGLE_RAD` [E] (start 0.25 deg: the light at a
  cloud edge moves < 1 voxel), `REFRESH_BLEND_STEP` [E], `CASCADES` (start 3: 8 / 64 / 512 km, [E] in Task 10).
- Generations: two per cascade; the lookup cross-fades from the old to the new one over `CROSSFADE_S` [E] once the new
  one is complete; a re-anchor fills only the uncovered slab (toroidal addressing).
- (Task 1 review, ledger ruling) No seam: the lookup blends neighbouring cascades over a band at every cascade edge, and
  the outermost cascade into the per-sample march over a band at its edge (weight by the distance to the edge); an
  image-pair check (the same view before / after a re-anchor, and a view across a cascade edge) shows no step above the
  image noise.
- (Task 1 measurements) Budget: a full fill of one 128 x 128 x 512 cascade at 96 + 32 samples costs 4.8-7.8 ms GPU -
  the per-frame voxel budget keeps the fill <= 0.5 ms per frame [E] (inside the 2.5 ms p99, spec decision 6); far
  cascades may use fewer vertical voxels / samples where their tolerance allows (measure against cloud_check case 8 at
  their scale); VRAM of all cascades x 2 generations <= 128 MB [E] (R16F if the tolerance holds).

- [ ] **Step 1: Tests** (`cloud_light_cache_test.gd`): a camera moving 1 km inside the central part schedules nothing;
  leaving it schedules exactly the uncovered slab; the Sun turning 0.1 deg schedules nothing, 0.3 deg a full new
  generation; a clock jump re-anchors every cascade and marks them invalid (lookup returns -1 until filled); the jobs
  of one frame never exceed the budget; a blend step above `REFRESH_BLEND_STEP` schedules a refill; `cloud_check.gd`
  case 10: the Sun's scattered light along a line of pixels crossing a cascade edge and the outer edge has no step
  larger than the case's noise, and the same view before / after a re-anchor differs by less than the image noise.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** the schedule, cascades, slabs, generations and the shader's cascade / generation lookup.
- [ ] **Step 4: Run** the new test, `cloud_check.gd` (case 8 with 3 cascades and a budget of 1/16 of the voxels per
  frame after 16 frames), `smoke_test.gd`.
- [ ] **Step 5: Commit** `feat(game): light cache cascades, re-anchoring and cross-faded refresh`

### Task 3: Deterministic march with refined silhouettes

**Files:** `game/shaders/cloud_pass.gdshaderinc` (`cloud_march`: Earth-fixed step phase, entry refinement; temporal
kernel bypass), `game/world/weather/cloud_effect.gd`, `game/world/weather/cloud_layer.gd` (global
`cloud_deterministic_march`, presets), test `game/tests/cloud_check.gd` (case 9), `game/tests/render_check.gd`
(`--repeat_identical` check)

**Interfaces:**
- Step phase: the first step boundary of a ray lies on Earth-fixed spherical shells (heights `k * step_m` above the
  ellipsoid), so the samples of a static ray never change between frames - no blue noise, no frame offset.
- Entry refinement: when a step's extinction crosses `CLOUD_SILHOUETTE_EXTINCTION_PER_M` [E] from below, bisect the
  step `CLOUD_SILHOUETTE_BISECTIONS` [E] (start 4) times and start the in-cloud steps there.
- Temporal accumulation: off when `cloud_deterministic_march` (the history kernel copies the current frame); the
  centre tent of the VR plan stays only as the upsample filter at radius 1 (bilinear) unless Task 10's images ask for
  more.

- [ ] **Step 1: Tests** - `render_check.gd --repeat_identical=2`: two frames of an unchanged scene (no history reset)
  are bit-identical; `cloud_check.gd` case 9: the silhouette of a single cumulus seen from 5 km lies within 1 pixel of
  the converged reference's 50 % transmittance contour; cases 1-8 still PASS.
- [ ] **Step 2: Run, expect FAIL** (frames differ).
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `cloud_check.gd`, `cloud_vr_stereo_test.gd`, `smoke_test.gd`, the identical-frames check.
- [ ] **Step 5: Commit** `feat(game): deterministic cloud march with refined silhouettes`

### Task 4: Steep cloud edges with the invariants

**Files:** `core/include/starsim/math/constants.hpp` (`cloud_edge_width_min_m` / `_max_m`: tens of m [E], the rim of a
cumulus), `core/src/environment/cloud_noise.cpp` (tables rebuilt), `core/tests/test_cloud_optics.cpp`,
`game/shaders/clouds.gdshaderinc` (`cloud_edge_ramp` width in metres per detail state)

**Interfaces:** unchanged signatures; the edge width moves from the M8 soft range to `[E]` 10..40 m (start), the
thresholds `t[L](cf)` and edge-mean factors `m[L](cf)` regenerated by `generate_cloud_noise()`.

- [ ] **Step 1: Tests** (`[weather]`): "Noise coverage is preserved" and "Soft cloud edges ... keep the grid-mean
  water at every detail state" pass with the new widths (rename the latter to "Cloud edges ..."); new: "A cumulus edge
  rises from 10 % to 90 % of its interior extinction within the edge width" (the steepness).
- [ ] **Step 2: Run, expect FAIL** (steepness).
- [ ] **Step 3: Implement** the widths and regenerate; update the GPU tables' upload if their layout changes.
- [ ] **Step 4: Run** `build.ps1 -NoBindings -TestFilter "[weather]"`, then the full GDExtension build and
  `cloud_check.gd`, `cloud_far_check.gd` (the near / far comparisons re-baselined only if the invariants hold).
- [ ] **Step 5: Commit** `feat(core): steep cumulus edges, coverage and water kept`

### Task 5: Shapes by cloud type

**Files:** `core/include/starsim/environment/cloud_noise.hpp` + `core/src/environment/cloud_noise.cpp`
(`CloudType`, `cloud_type_of`), `game/shaders/clouds.gdshaderinc` (the same rule; per-type shape coordinates),
`core/tests/test_cloud_optics.cpp`, render images to the user

**Interfaces:**
- `enum class CloudType { Cumulus, Stratiform, Cirrus };`
  `CloudType cloud_type_of(double fraction, double fraction_above, double ice_share, double height_above_ground_m);`
  rule: ice share > 0.5 -> Cirrus; fraction >= `stratiform_fraction` [E] (0.9) and the fraction does not thicken upward
  -> Stratiform; else Cumulus. GLSL mirror `int cloud_type_of(CloudCell cell, float height_above_ground_m)`.
- Shapes: Cumulus - the current shape tile with the low octaves' lobes (cauliflower); user 2026-10-26: each cumulus is
  ONE connected body ("hängen zusammen zu jeweils einer Wolke"), not a scatter of separate puffs - the low-frequency
  cell field decides where a cloud is, higher octaves only deform its surface (erode inward from the rim, never punch
  isolated fragments or holes into the interior); check: render shows compact clouds, a test counts connected
  components per cloud cell (fragments < [E] small share of the cloud's area); Stratiform - the horizontal tile
  stretched by `stratiform_stretch` [E] (flat sheet, undulating top); Cirrus - the existing 2 km ice tile.
- Invariants per type: coverage and water tables per type if the stretch changes the value statistics.

- [ ] **Step 1: Tests** (`[weather]`): `cloud_type_of` on the three canonical columns; coverage and water preserved for
  each type; "Cloud perimeters have the observed fractal dimension" for Cumulus.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** core + GLSL mirror.
- [ ] **Step 4: Run** `[weather]`, `cloud_check.gd`, `cloud_far_check.gd`; render the view set (ground at Starbase,
  deck at 9 km, 20 km, orbit, sunset) and send the PNGs to the user.
- [ ] **Step 5: Commit** `feat(game): cloud shapes by type - cumulus, stratiform, cirrus`

### Task 5b: Anti-aliased cloud outlines (user 2026-10-26)

User on the t3-t5 images: "die Wolken wirken pixelig, treppenartig.. sollen auf jeden Fall weicher sein". The
deterministic march has one hard sample per march pixel (divisor 2-3), so outlines show march-pixel stairs and beads;
the Catmull-Rom upsample (Task 3) only rounds the corners. Required: outlines smooth at screen resolution (no visible
stairs), static (render_check repeat_identical stays bit-identical - no temporal jitter), crisp (no wide blur).

**Files:** `game/shaders/cloud_pass.gdshaderinc` (+ a kernel in `game/world/weather/cloud_effect.gd` if needed),
`game/tests/cloud_check.gd` (an edge-aliasing check), docs `weather-clouds.md`.

- Approach, in this order (stop at the first that removes the stairs in the renders):
  1. Edge-adaptive supersampling: march pixels whose alpha differs from a neighbour by > [E] threshold re-march with 4
     rotated-grid sub-rays (fixed offsets) and average - cost only on outline pixels.
  2. A full-resolution, edge-directed alpha smoothing of the upsampled cloud layer (morphological / FXAA-like, along
     the edge, not across it).
- Check: a test views a straight-edged slab rim at the VR divisor and asserts the outline's step pattern is gone
  (e.g. along-edge alpha profile monotone, max per-pixel jump <= [E] 0.35) and repeat_identical holds.
- Measure the cloud pass in the VR proxy once (Task 10 owns the budget) and render t5b_* with the Task 5 view set.
- Commit `feat(game): anti-aliased cloud outlines`

### Task 5c: No step aliasing along the ray (user 2026-10-26, after Task 7)

User on t7_cape.png: "Warum bekommt man dieses Grieselige nicht weg?". Diagnosis (renders dbg_cape_* in
`tmp/static_clouds`): the grain scales with the march pixel (divisor 2 finer, same pattern), stays with detail levels
0, and vanishes with 8x march steps (768 / 1536): the fixed step phase of each ray hits or skips thin / broken cloud
on long oblique paths - static salt-and-pepper the jittered march's history used to average away.

**Files:** `game/shaders/cloud_pass.gdshaderinc`, `game/shaders/clouds.gdshaderinc`, `game/world/weather/cloud_layer.gd`
(presets), `game/tests/cloud_check.gd`, docs `weather-clouds.md`.

- Approach (measure each): inside cloud layers bound the step by the cloud structure scale [E] instead of a fixed
  count over the whole path (empty-space skip outside via the column field); and/or sample the shape at the mip of
  max(pixel footprint, step length) so a long step reads the averaged density, not a point.
- Check: the Cape view (`--lat=28.5729 --lon=-80.649 --elevation=20`, VR preset) against the 8x-steps reference -
  a grain metric (e.g. high-pass energy at march-pixel scale in cloud regions) within [E] of the reference;
  repeat_identical; cloud_check limits unchanged.
- Measure the cloud pass in the VR proxy (Task 10 budget); render the Task 5 view set + Cape for the user.
- Commit `fix(game): cloud march without step aliasing`

### Task 6: 2 m temperature and dew point in the snapshot

**Files:** `core/include/starsim/environment/weather_source.hpp` + `.cpp` (ERA5 `2m_temperature`,
`2m_dewpoint_temperature`; GFS `TMP` / `DPT` 2 m above ground), `core/include/starsim/environment/weather_snapshot.hpp`
(`Era5Fields`, `GfsFields`; per-column `lcl_m`), `weather_snapshot_build.cpp`, `weather_snapshot_file.hpp`
(`weather_snapshot_file_version = 2`), `tools/data_pipeline/sources.toml`, `tools/data_pipeline/fetch.py weather_analog`
(regenerate), tests `core/tests/test_weather_*.cpp`

**Interfaces:** `double lifting_condensation_level_m(double t2m_k, double td2m_k);` (Romps 2017 exact if within budget,
else Espy 125 m/K - document which; constants with source in `constants.hpp`); `WeatherSnapshot::lcl_m(column)`.

- [ ] **Step 1: Tests** (`[weather]`): LCL of T = 303.15 K, Td = 293.15 K vs the Romps reference value (tolerance 1 %);
  requests list the new files; a version-1 file is rejected; build of a synthetic snapshot stores the LCL per column.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement**; regenerate the analog files (`python tools/data_pipeline/fetch.py weather_analog`).
- [ ] **Step 4: Run** `[weather]`, `weather_service_test.gd`, `smoke_test.gd`.
- [ ] **Step 5: Commit** `feat(core): 2 m temperature and dew point, lifting condensation level per column`

### Task 7: Cloud bases at the LCL

**Files:** `core/src/environment/weather_snapshot.cpp` (`ColumnLayers::build`: boundary-layer liquid below the LCL moved
up, water conserved), `game/shaders/clouds.gdshaderinc` (flat base at the column's LCL: `cloud_profile_value` cut with
a ramp of `CLOUD_LCL_RAMP_M` [E]), column texture channel for the LCL (`cloud_field_texture.gd`), tests
`core/tests/test_weather_snapshot.cpp`, the cbh fixture

**Interfaces:** the rule of spec decisions 1-2: lowest liquid cloud with base < 3 km above ground [E]; saturated surface
(T - Td < `fog_spread_k` [E] 1 K) keeps its cloud down to the ground (fog).

- [ ] **Step 1: Tests**: a column with liquid from the ground up and LCL 800 m -> no condensate below 800 m, the column
  water path unchanged (1e-6 relative); T - Td = 0.5 K -> unchanged (fog); an ice layer at 8 km unchanged; the cbh
  fixture re-baselined with the documented tolerance.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** core redistribution + the shader's flat base.
- [ ] **Step 4: Run** `[weather]`, `cloud_check.gd`, `cloud_far_check.gd`, images from below (Starbase, Cape) to the user.
- [ ] **Step 5: Commit** `feat(core): cumulus bases at the lifting condensation level`

### Task 8: Near / far hand-over in the cloud pass

**Files:** `game/shaders/cloud_pass.gdshaderinc` (march ends at `cloud_handover_far_m`; beyond, `cloud_far_layer` per
pixel; blend over `cloud_handover_band_m`), `game/shaders/cloud_far.gdshaderinc` (callable from the compute port),
`game/world/weather/cloud_layer.gd` (presets: hand-over range; `far_enabled` no longer only beyond 1.5e7 m),
`game/tests/cloud_far_check.gd`

**Interfaces:** globals `cloud_handover_near_m`, `cloud_handover_far_m` [E] (start 30 / 50 km, measured in Task 10);
the far model's premultiplied result composited behind the march's result with the remaining transmittance.
Carried from Task 3 (ledger "Task 3 follow-up"): the deterministic march point-samples the prefiltered shape at the
footprint centre; with ~7 km march pixels (coarsest shape mip 1 km) the volume's cover beats with the 32 km tile and
`cloud_far_check` "volume block luminance" fails (38 % vs 10 %). The hand-over must end the volume before footprints
outgrow the shape mips (spec decision 4: < 3.5 km) so this case passes on the deterministic path again.

- [ ] **Step 1: Tests** (`cloud_far_check.gd`): across the band the mean radiance of a broken layer seen at 30-50 km
  differs between near and far by <= 10 % (the M8 ruling) and the blended image has no step > 3 % between neighbouring
  distance bins; at 420 km every pixel is far (march skipped); a moving camera pair (100 m apart, 20 km view) shows no
  pixel flip of cover larger than the image noise of a static pair.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `cloud_far_check.gd`, `cloud_check.gd`, `cloud_vr_stereo_test.gd`, `smoke_test.gd`.
- [ ] **Step 5: Commit** `feat(game): distant clouds from the column model in the cloud pass`

### Task 9: The Sun through clouds

**Files:** `game/tests/render_check.gd` (`--sun_through_deck` view), the cloud composite / Sun disk path if the check
fails (`game/shaders/cloud_layer.gdshader`, the Sun disk material)

- [ ] **Step 1: Test**: looking at the Sun through slabs of vertical tau 2, 5, 10, 20 (slab snapshots): the Sun disk's
  HDR radiance = unclipped disk radiance x exp(-tau / mu) within 5 % and brighter than the terrain behind the slab
  until that drops below the terrain's own radiance; the aureole (forward scattering) present.
- [ ] **Step 2: Run** - FAIL if the disk is clipped before the composite.
- [ ] **Step 3: Fix** if needed (disk in HDR before the clouds).
- [ ] **Step 4: Run** the check, `smoke_test.gd`.
- [ ] **Step 5: Commit** `test(game): the Sun through cloud decks` (or `fix(game): ...`)

### Task 9b: Backlit cumulus contrast (controller look check 2026-10-27, quality first)

The t9f_edge render (Sun 15 deg, just behind a cumulus) shows the silver rim, but the backlit body is as bright as or
brighter than the sky beside it; real backlit cumulus bodies are dark grey with a brilliant rim (rim 2-4x the body).

**Files:** `game/shaders/cloud_pass.gdshaderinc` (light / multiple scattering), `game/tests/cloud_check.gd`, docs
`docs/subsystems/weather-clouds.md`

- [ ] **Step 1: Measure** HDR body / sky / rim radiance of a backlit cumulus at known slant tau (>= 30) and of a
  backlit slab against the project's Monte Carlo transfer reference (transmitted radiance at that view angle).
- [ ] **Step 2: Test** (RED): body darker than the sky at the same angle from the Sun, rim 2-4x the body, slab
  transmitted radiance within [E] of the MC reference.
- [ ] **Step 3: Fix** the cause (light march extinction scale, multiple-scattering approximation, forward-peak
  in-scattering), without breaking the front-lit tonal zones, cloud_check limits, or the far tier match.
- [ ] **Step 4: Run** the cloud checks, renders t9b_* (backlit, front-lit, side-lit) vs t9f.
- [ ] **Step 5: Commit** `fix(game): backlit cumulus bodies dark with a bright rim`

### Task 9c: Cumulus water from an adiabatic parcel (user 2026-10-27: "Cumulus sind nie flach")

Investigation (sdd cloud-tau-investigation.md): ERA5 / GFS cloud water lacks the convective updraft cores, so
grid-mean water / cover gives cumulus about half the subadiabatic in-cloud water; the thinnest decile (and GFS
shallow cumulus, median in-cloud tau 0.4) are wide, thin, translucent sheets - not cumulus.

**Files:** `core/src/environment/` (column build / LCL bases), `core/tests/`, docs `docs/subsystems/weather-clouds.md`

- [ ] **Step 1: Test** (RED): liquid cumuliform boundary-layer runs reach in-cloud LWC >= f_ad x the adiabatic LWC
  gradient (T, p) x height above the cloud base (f_ad 0.3-0.5 [E], Blyth 1993, Brenguier et al. 2000); column water
  path conserved; cover <= the data's.
- [ ] **Step 2: Implement**: where the data's in-cloud water is below the floor, lower the cumulus cover to keep the
  water path (fewer, denser, deeper-looking cells); shapes unchanged (user-approved), only cover / extinction.
- [ ] **Step 3: Run** core tags + cloud checks; record the cover change vs ERA5 tcc; renders t9c_* (Cape, ground,
  deck) vs t9b.
- [ ] **Step 4: Commit** `feat(core): cumulus in-cloud water from an adiabatic parcel`

### Task 10: Budget, presets, desktop re-baseline, VR acceptance

**Files:** `game/world/weather/cloud_layer.gd` (presets), `docs/subsystems/weather-clouds.md`,
`docs/subsystems/vr-performance.md`, memory

- [ ] **Step 1: Measure** with `terrain_perf.gd --render_scale=1.65 --gpu_passes` (cloud pass, fill) and tune the
  presets (cascade sizes, budget, divisor, steps, hand-over) to cloud p99 <= 2.5 ms x the stereo correction.
- [ ] **Step 2: Desktop re-baseline**: the render_check view set before / after, PNGs to the user; `cloud_check.gd` /
  `cloud_far_check.gd` reference values documented.
- [ ] **Step 3: VR run with the user** (`Start-VR.cmd --perf-log`, weather on, pad -> decks -> orbit, `nvidia-smi`):
  cloud cost p99, and the user's look check against the spec's anti-goals.
- [ ] **Step 4: Docs, ledger, memory; commit** `docs: static clouds - measurements and current state`
