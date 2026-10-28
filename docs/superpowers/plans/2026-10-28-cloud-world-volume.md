# Cloud World Volume Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every frame a fresh, exact march of every visible cloud through a camera-centred, Earth-fixed world volume of
baked density, occupancy and light - the full-resolution reference look with feathery edges, no reprojection - at
45 Hz in VR with weather on.

**Architecture:** An 8-level toroidal clipmap (baked macro + meso density of both snapshots, max-extinction mip and
Chebyshev distance field, light volume tau_sun / tau_below / tau_above / tau_moon) is kept current by a budgeted upkeep
inside the frame. Per frame, a per-eye near tier marches [0, 2 km] and one shared far tier on Earth-fixed directions
marches [2 km, end]; both are composited exactly (L = L_near + T_near L_far); the 2.5D column model draws beyond the
far fade. Measurement first: instrumentation, Spike S (STOP 1), prototype behind `CloudLayer.world_volume`, gate
(STOP 2), then the switch-over that deletes the direction view cache.

**Tech Stack:** Godot 4.7.2 double, GDScript, RenderingDevice compute (GLSL compiled by `CloudShaderPort`), D3D12,
OpenXR; C++20 core (Catch2, tag `[weather]`) and the GDExtension `StarsimCore`.

**Spec:** `docs/superpowers/specs/2026-10-28-cloud-world-volume-design.md` (approved by the user 2026-10-28; read
"The design (decided)", "Decisions taken for the builder" and "Measures of success"). Inputs:
`.superpowers/sdd/2026-10-28-cloud-architecture/` (`user-decisions.md` binding; `recon-current.md` for file:line refs).

## Global Constraints

Verbatim copy for dispatches: `.superpowers/sdd/2026-10-28-cloud-world-volume/global-constraints.md`.

- Same physics, no new model: weather envelope, cover / water rules, optics, phase function, transfer table, column
  model physics unchanged; no Wrenninge octaves. Transfer table over delta-Eddington; independent columns; beam-cover
  fit; plane-parallel Sun-disk bias accepted; the 32 km shape lattice stays.
- Nothing view-dependent kept between frames: no reprojection, no temporal filtering / jitter accumulation, no
  foveated rendering / VRS (user). Reduced tiers upsampled depth-aware WITHOUT history. Deterministic march: a still
  frame is bit-identical; samples on Earth-fixed shells.
- VR weather on 45 Hz (22.2 ms), weather off 90 Hz; cloud GPU <= 11 ms median, <= 14 ms p99 per phase; one path for
  desktop and VR.
- One graphics+compute queue: upkeep inside the frame, 1.5 ms per frame (light refresh 1.0 ms of it), <= 3 ms [E] in
  re-frame (and jump) frames. Main-thread scheduling <= 1.0 ms per frame.
- `core` without Godot; physics never in GDScript; shaders in `game/shaders/`. Volume origins float64 Earth-fixed,
  differenced in double before float32. Volume contents never depend on the frame's depth buffer.
- Weather / clock / view jumps restart without stale content and never show a frame without clouds (coarsest level
  first; full volume within ~1 s [E]).
- Far tier marches the shapes; the column model draws only beyond the far fade (1.5-3 km footprint, kept); scaled space
  beyond 1.5e7 m unchanged.
- World volume: 8 levels, 256 x 256 x 128 cells, toroidal, snapped; cells 50 m x 25 m (level 0, 3.2 km vertical
  window), 100 m x 50 m (level 1, 6.4 km), levels 2-7 200 m .. 6.4 km x 164 m over the shell -0.5 .. 20.5 km;
  azimuthal-equidistant horizontal axes about a frame origin, height above WGS84; 16 B per cell (density RGBA16 both
  snapshots, light RGBA16F); 1/4-res max-extinction mip (R16F) + Chebyshev distance field (R8) dilated by the sub-cell
  lift; ~1.07 GB (+134 MB spare level), volume <= 1.1 GB; fade band 8 cells; re-frame at 1000 km, coarsest first,
  0.5 s cross-fade.
- Scroll: density and occupancy of an exposed slab baked in that frame before the march (never late); light may be
  provisional (from the next coarser level), blended in over 0.25 s, replaced within 1 s.
- Light: ~32-64 Delta-tau-limited samples per cell, then the grid-mean column; Earth's shadow -> infinite; tau_moon
  only with the Sun below -6 deg at the level centre; triggers: shadow shift H dtheta / sin^2 e < 1/4 cell, blend
  weight moved >= 0.02, provisional, re-frame; rate-limited (lag, never a front). Surface correction: T (1 -
  exp(-Delta tau)) > 1e-3 and cell coarser than the footprint -> 4 Sun samples over 2 cells (`light_entry_samples`).
- March: R_near 2 km; near tier per eye (skipped when the distance field at the eye > R_near); far tier shared from the
  cyclops point on Earth-fixed directions at 0.5 x eye density, union-of-frusta texels only, cyclopean scene depth;
  composite L = L_near + T_near L_far, far term 0 where scene depth < R_near; near full or 0.7x, far 0.5x; far never
  below 0.5x without the user; far storage <= 300 MB.
- Steps: sphere tracing on the finest level's distance field; occupied step = min(Delta tau <= 0.3, 1/2 cell, 1/4
  finest structure at the footprint), >= 2 m; Hillaire 2016 segment integration; entry bisection 4 x; Sun-ray
  exception to T 1e-9. Per sample: density 1, light 1, table 1, sub-cell 1-4 (shape residual from level 2 up + detail
  1000 / 250 / 62.5 m, w_k = clamp(log2(l_k / f) - 1, 0, 1)); `edge_curl`; level fade linear over 8 cells.
- Fidelity flags kept: `scattering_model`, `air_shadow`, `far_3d_light`, `detail_levels`, `deterministic_march`,
  `cloud_debug_output`, `far_fade_m`, `CloudShadowEffect.grid_mean_only`, `pass_enabled`. New: `world_volume` (default
  false until Task 13; false = today's per-sample march = the reference path), `light_entry_samples`, `edge_curl`,
  `near_scale`.
- No magic numbers (`[E]` + rationale); tests only the affected scripts / tags; GPU checks `--xr-mode off`, no
  `--headless`; images `screenshots/cloud-world-volume/`; long runs to files; Conventional Commits, push after each
  green task; docs per task (`weather-clouds.md` current state, history verbatim, `vr-performance.md` numbers).

## Review Focus

- Weather pair switch while the volume bakes (new snapshot into the free channel pair at weight 0): the blend never
  jumps, the shown snapshot's channels are never rewritten (Task 5 tests).
- Camera height crossing level 0 / level 1's on-off heights (~22.1 / ~23.7 km) in ascent and descent: no pop, no frame
  reading an unbaked level (Task 3 and Task 9 tests).
- Sun at / below the horizon (Earth's shadow, infinite tau in half floats) and the Moon switch at -6 deg: no NaN, no
  black frame, no front (Task 6 tests).
- Scene geometry inside R_near in front of distant cloud (tower, vehicle; in one eye only): far term exactly 0 behind
  it, no halo from the depth-aware upsample (Task 8 test).
- VR start / stop, resize or preset change mid-flight: tier images re-created without a frame without clouds and
  without rebaking the volume (Task 8 test).

## Controller notes

| Task | Content | Model | User / STOP |
|---|---|---|---|
| 0 | Counters and ablations of today's march (spec Step 0) | Sonnet | - |
| 1 | Stereo harness, recorded Pimax projections, gate views | Sonnet | headset 1 min (projection record) |
| 2 | Spike S (far tier alone, V3 + V1, (a) vs (b)) | Opus | **STOP 1** - continue only on "build" |
| 3 | Volume layout and upkeep schedule (pure logic) | Sonnet | - |
| 4 | Core reference of the cell bake | Opus | - |
| 5 | GPU bake, max-extinction mip, distance field | Opus | - |
| 6 | Light volume and its refresh | Opus | - |
| 7 | Volume march (unsplit per eye): steps, sub-cell field, curl, surface correction, level fade | Opus | - |
| 8 | Near / far tiers, Earth-fixed far grid, cyclopean depth, exact composite, upsample | Opus | - |
| 9 | Scroll / re-frame in the frame, far hand-over beyond the levels | Opus | - |
| 10 | Air shadow and shadow maps from the light volume | Sonnet | - |
| 11 | GATE (V1-V7, kill criteria, F1-F4, headset) | Opus, controller-led | **STOP 2**; headset; `edge_curl`, `near_scale`, F4 |
| 12 | Re-entry scroll / re-frame acceptance | Sonnet | - |
| 13 | Switch-over (view cache deleted in one commit), docs | Sonnet | - |
| 14 | Final acceptance | Opus final review | headset, look verdict |

Commands used below: `G=external/godot/bin/godot.windows.editor.double.x86_64.console.exe`;
`RC="$G --xr-mode off --path game --script res://tests/render_check.gd --"`; GPU checks
`$G --xr-mode off --path game --script res://tests/<check>.gd`; headless `$G --headless --path game --script
res://tests/<test>.gd`; after a new `class_name`: `$G --headless --path game --import`. Pass strings: `cloud_check:
PASS`, `CLOUD LAYER TEST PASSED`, `cloud_vr_stereo_test: PASS`, `cloud_world_volume_test: PASS`, render_check exit
code 0. Every GPU run's log goes to `.superpowers/sdd/2026-10-28-cloud-world-volume/<task>_<name>.log`; check it for
`ERROR: CloudEffect` (shader compile errors).

---

### Task 0: Counters and ablations of today's march

Where the ~10 x per-sample gap to Nubis^3 sits, measured on today's march (spec G, Step 0).

**Files:**
- Modify: `game/shaders/cloud_pass.gdshaderinc`, `game/shaders/clouds.gdshaderinc` (counters under `#ifdef
  CLOUD_COUNTERS`; ablation defines `CLOUD_ABLATE_*`)
- Modify: `game/world/weather/cloud_shader_port.gd` (defines from `CloudEffect.counters` / `CloudEffect.ablate`; the
  temporary `STARSIM_CLOUD_ABLATE` hook of the view-cache plan made permanent)
- Modify: `game/world/weather/cloud_effect.gd` (counter image RGBA32UI per march pixel, read back on request)
- Modify: `game/tests/render_check.gd` (`--counters`, `--ablate=<names>`)
- Docs: `docs/subsystems/vr-performance.md` (section "Where the march's time goes"), `weather-clouds.md` (Diagnostics)

**Interfaces:**
- Produces: `CloudEffect.counters: bool` (static), `CloudEffect.ablate: PackedStringArray` (static; names
  `field`, `shape`, `detail`, `light`, `subsamples`, `outline`, `air_shadow`, `far_tier`); render_check prints
  `render_check: counter <name> median <m> p90 <a> p99 <b> max <c>` for `density_fetches`, `light_fetches`,
  `occupied_steps`, `empty_steps` (per march pixel) and writes `<out>_counter_<name>.png` (normalised to the max).
  Occupied step = a step whose sample extinction > 0; fetch counts are added at the call sites by each function's fixed
  texture-fetch count. The same counter names and image layout are reused by Tasks 2, 7, 8.

- [ ] **Step 1: Shader clock.** Grep `external/godot/servers/rendering/rendering_device*.h` for a shader-clock
  feature; the game runs on D3D12, `VK_KHR_shader_clock` is Vulkan-only - expected absent. Record the finding in
  `vr-performance.md`; without it every time split comes from ablations.
- [ ] **Step 2: Test.** Save the march reference before any change (`STARSIM_MARCH_REFERENCE=<dir>` with
  `STARSIM_VIEW_CACHE_ONLY=1 cloud_check.gd`, section 15). Checks to pass after Step 3: section 15 against the saved
  images bit-identical with counters off (no cost, no change when `CLOUD_COUNTERS` is undefined); `$RC
  --weather=slab:1000,2000,0.0003,0.0 --lat=30 --lon=-95 --utc=2024-06-15T18:00:00 --elevation=30 --no_view_cache
  --cloud_divisor=1 --counters` prints `occupied_steps max 0`; the same with fraction 1.0 prints `occupied_steps
  median` > 0.
- [ ] **Step 3: Implement** counters and ablations.
- [ ] **Step 4: Run** Step 2's checks: PASS / the expected lines.
- [ ] **Step 5: Measure** (proxy, today's march: `--no_view_cache --cloud_quality=vr --render_scale=1.65 --gpu_time=120
  --gpu_passes --fixed_exposure`, Starbase analog 2024-06-15T21:00 views `ground` and `deck` of the view-cache plan's
  `t1_prof.sh`): base + `--counters` + each ablation alone. Table in `vr-performance.md`: ms saved per ablation,
  fetches per sample (density, light), occupied / empty steps per pixel (median, p99), and the per-occupied-step cost
  (cloud-pass ms / occupied steps summed over the image) - the number Spike S compares against.
- [ ] **Step 6: Commit** `feat(game): cloud march counters and ablation switches (world volume Step 0)`

### Task 1: Stereo harness, Pimax projections, gate views

**Files:**
- Create: `game/tests/stereo_mock_xr.gd` (class `StereoMockXr extends XRInterfaceExtension`: two views from a
  projections JSON, render target size, eye transforms relative to the head)
- Create: `game/tests/cloud_views.gd` (class `CloudViews`: `const VIEWS := {"V1": PackedStringArray([...]), ...}`)
- Modify: `game/util/perf_log.gd` (at XR start write `data/perf/xr_projections_<stamp>.json`)
- Modify: `game/tests/render_check.gd` (`--stereo=<json>`, `--cloud_view=Vn`, `--frames_every` also with `--mission`
  / `--at_event`)
- Test: `game/tests/stereo_mock_xr_test.gd` (headless)

**Interfaces:**
- Produces: JSON `{"render_target": [w, h], "views": [{"projection": [16 floats, column-major], "eye_from_head":
  [12 floats]}, {...}], "near_m": n, "far_m": f}` (from `XRInterface.get_projection_for_view(v, aspect, near, far)` and
  `get_transform_for_view(v, head)` relative to the head); `StereoMockXr.load(path: String) -> bool`;
  `CloudViews.args(name: String) -> PackedStringArray`.
- Gate views (`CloudViews.VIEWS`, base SB = `--lat=25.997 --lon=-97.157 --utc=2024-06-15T21:00:00`, each also with
  `--weather=analog` and with `--weather=<abs path>/era5_1102887.wthr` (ERA5 2025-10-25 15:00 UTC) where marked *):
  - V1* pad, broken Cu: SB `--weather=slab:1000,2500,0.0003,0.5 --azimuth=90 --elevation=10 --fov=100`
  - V2 ascent through the deck: `--vehicle=starship_block3 --site=starbase_olp1 --autopilot --fly=25
    --weather=slab:1500,2500,0.0003,0.6 --frames_every=0.5 --frames_for=120`
  - V3* 9 km above a broken deck: SB `--weather=slab:1500,2500,0.0003,0.5 --eye_height=9000 --azimuth=0
    --elevation=-10`
  - V4* `--eye_height=20000 --elevation=-15` and `--eye_height=80000 --elevation=-20` (SB, slab cf 0.5)
  - V5* 400 km: `--utc=2024-06-15T10:00:00 --lat=0 --lon=0 --eye_height=400000 --elevation=-90 --fov=40` (nadir) and
    `--elevation=-19` (limb)
  - V6 backlit: `--lat=30 --lon=-95 --weather=slab:1000,2000,1e-4,0.5 --utc=2024-04-09T23:30:00 --look_at_sun`; sunset:
    the same site and slab at the UTC where the Sun is 3.0 +- 0.3 deg up (found once, printed, fixed in the table)
  - V7 booster catch: `--mission=starship_leo --at_event=caught,-90 --view=<the booster's Tab index>
    --weather=slab:1500,2500,0.0003,0.6 --frames_every=0.5 --frames_for=95`

- [ ] **Step 1: Test** `stereo_mock_xr_test.gd`: a JSON written from two known projections (the stereo test's
  `FOV_LEFT_EYE` mirrored, IPD 0.064 m) loads; `_get_projection_for_view` returns them within 1e-12; view count 2;
  render target as given. `CloudViews.args("V3")` contains `--eye_height=9000`.
- [ ] **Step 2: Run, expect FAIL** (classes missing).
- [ ] **Step 3: Implement.** If a GDScript `XRInterfaceExtension` cannot drive the multiview render path of this build,
  stop and report BLOCKED with what fails (do not emulate stereo another way without the controller).
- [ ] **Step 4: Run** the test: `stereo_mock_xr_test: PASS`; `$RC --stereo=<test json> --cloud_view=V1
  --no_view_cache` (today's march): exit 0, both eye images written (`<out>_eye0.png`, `_eye1.png`), the log shows
  `view_count 2`.
- [ ] **Step 5: Headset with the user** (1 min): `Start-VR.cmd --perf-log` on the pad; copy the JSON to
  `game/tests/data/pimax_dream_air_projections.json` (committed; per-eye target 3534 x 2838 expected). Until it exists
  Tasks 2-10 may use the test JSON scaled to 3534 x 2838, marked provisional in their reports; the gate needs the
  recorded one.
- [ ] **Step 6: ERA5 date:** `$RC --utc=2025-10-25T15:00:00 --weather=online --lat=25.997 --lon=-97.157` once (writes
  `data/weather/cache/era5_1102887.wthr`; not cached today).
- [ ] **Step 7: Commit** `test(game): stereo offscreen harness with recorded headset projections, cloud gate views`

### Task 2: Spike S - far tier alone (STOP 1)

Throw-away code allowed (spec G). Decides whether the prototype is built.

**Files:**
- Create: `game/shaders/cloud_spike.gdshaderinc` (kernels `CLOUD_SPIKE_OCCUPANCY`, `CLOUD_SPIKE_BAKE`,
  `CLOUD_SPIKE_FAR`), `game/world/weather/cloud_spike.gd` (class `CloudSpike`: scratch volumes, dispatch)
- Modify: `game/world/weather/cloud_effect.gd` (one branch: `CloudSpike.mode != 0` runs the spike instead of the
  march), `game/tests/render_check.gd` (`--spike=a|b`, `--spike_scale=0.5|1.0`)
- Docs: `vr-performance.md` ("Spike S")

**Interfaces:**
- Consumes: Task 0 counters / ablation names, Task 1 projections JSON and `CloudViews` (V1, V3).
- Produces: nothing kept by later tasks (deleted in Task 13).

The spike marches the far tier only: rays [R_near = 2000 m, end] from the cyclops point over the union region of both
recorded eye frusta (`CloudCentreView.build` with the recorded projections; divisor 2 for 0.5x, 1 for the 1.0x scaling
run), mono, at the real union size. Exact occupancy (both variants): a scratch volume over the view built once by
dense evaluation of `cloud_extinction_at` (max over sub-taps + `cloud_detail_lift_bound`), cell <= 200 m, a Chebyshev
distance field from it; scratch memory <= 6 GB [E]. Step rule both variants: sphere tracing on it; occupied step =
min(Delta tau <= 0.3 from the occupancy's max extinction, 1/2 cell, 1/4 finest structure), >= 2 m; one sample per step,
no sub-samples, constant light.
- (a) today's per-sample procedural density (`cloud_extinction_at` at the footprint).
- (b) a single-level bake of the meso density over the view (cell = the median far footprint, >= 100 m), one density
  fetch per step plus one dummy light-volume fetch (an RGBA16F scratch texture) per step.

- [ ] **Step 1: Sanity test:** (a)'s image vs the reference march's far part (`--no_view_cache --handover=0,0`, same
  view): mean alpha within 0.05, mean luminance within 10 % (it marches the same clouds); (b) vs (a) the same bounds.
- [ ] **Step 2: Run** the matrix: V3, V1 x (a), (b) at 0.5x; V3 (b) at 1.0x; each `--gpu_time=120 --gpu_passes
  --counters` and the ablations `density` (fetch -> constant) and `light` (dummy fetch removed). Record ms (median,
  p99), occupied / empty steps per pixel, ms per occupied step, density vs light split.
- [ ] **Step 3: Apply the rules** (spec G, verbatim): (b) <= 5 ms in V3 -> build the prototype; (b) not >= 1.5 x cheaper
  per occupied step than (a) -> the density fetch is not the lever: rework the step rule / termination first and
  report; > 100 occupied steps per pixel in V3 despite perfect occupancy -> redesign the step rule before the build;
  (b) > 10 ms -> stop and report to the user. (b) between 5 and 10 ms with the other rules met: report to the user with
  the controller's recommendation (the spec does not decide this band).
- [ ] **Step 4: Docs + commit** `chore(game): world volume spike S - far tier cost of baked vs procedural density`
- [ ] **STOP 1 (controller):** report the table and the rule outcome to the user. Tasks 3-14 run only on "build"; the
  other outcomes go back to the user / a revised spec.

### Task 3: Volume layout and upkeep schedule (pure logic)

**Files:**
- Create: `game/world/weather/cloud_world_volume_layout.gd` (class `CloudWorldVolumeLayout`)
- Create: `game/world/weather/cloud_world_volume_schedule.gd` (class `CloudWorldVolumeSchedule`; patterns of
  `cloud_light_cache_schedule.gd`: tangent frame, re-anchor, budgeted jobs)
- Test: `game/tests/cloud_world_volume_test.gd` (headless; prints `cloud_world_volume_test: PASS`)

**Interfaces:**
- Produces `CloudWorldVolumeLayout`: constants `LEVELS 8`, `CELLS_H 256`, `CELLS_V 128`, `CELL_H_M [50, 100, 200,
  400, 800, 1600, 3200, 6400]`, `CELL_V_M [25, 50, 164.0625 x 6]`, `SHELL_BOTTOM_M -500`, `SHELL_TOP_M 20500`,
  `FADE_CELLS 8`, `REFRAME_M 1.0e6`, `BYTES_PER_CELL 16`; `set_frame(origin_earth_m: Vector3)`;
  `grid_of(p_earth_m: Vector3) -> Vector3` (x, y azimuthal-equidistant on the sphere of the WGS84 Gaussian radius at
  the frame origin, z = height above the ellipsoid); `earth_of(grid_m: Vector3) -> Vector3`;
  `level_active(k: int, camera_height_m: float) -> bool`; `window_origin(k: int, camera_grid_m: Vector3) -> Vector3i`
  (lower corner in cells, snapped, centred on the ground point; levels 0-1 vertically centred on the camera height,
  clamped to the shell); `texel_of(k: int, cell: Vector3i) -> Vector3i` (toroidal); `exposed_slabs(k: int,
  old_origin: Vector3i, new_origin: Vector3i) -> Array[Dictionary]` ({from: Vector3i, to: Vector3i}, half-open cells);
  `fade_weight(k: int, grid_m: Vector3) -> float`; `reframe_needed(ground_earth_m: Vector3) -> bool`;
  `static memory_bytes() -> int`. The GLSL mirror (`cloud_volume_grid`, Task 5) evaluates the same projection.
- Produces `CloudWorldVolumeSchedule.new(layout)`: `reset()`; `plan(camera_earth_m: Vector3, sun_dir_earth: Vector3,
  moon_dir_earth: Vector3, blend: float, cloud_top_m: float, jumped: bool, delta_s: float) -> Dictionary` with keys
  `bake` (Array[{level, from, to, snapshot_channel}]), `light` (Array[{level, from, to}]), `provisional`
  (Array[{level, from, to}]: light copied from the next coarser level), `reframe` ({level, spare: bool, fade: float}),
  `lookup` (per level origin, active, fade weights; adopted by the render thread only after the batch ran);
  `feedback(upkeep_ms: float, bake_cells: int, light_cells: int)` (cells-per-ms calibration from GPU timestamps);
  `static sun_threshold_rad(k: int, sun_elevation_rad: float, height_above_m: float) -> float` = 0.25 x CELL_H_M[k] x
  sin^2(e) / H, H = the snapshot's cloud top above the level's lowest cell; `complete() -> bool`. Constants
  `UPKEEP_BUDGET_MS 1.5`, `LIGHT_BUDGET_MS 1.0`, `UPKEEP_SPIKE_MS 3.0` (re-frame and jump frames until complete),
  `PROVISIONAL_FADE_S 0.25`, `PROVISIONAL_MAX_S 1.0`, `REFRAME_FADE_S 0.5`, `WEATHER_REFRESH_STEP 0.02`,
  `LIGHT_STEP_T 0.01` [E: the 1 % front bound - a refresh moves a cell's exp(-tau) by at most this],
  `MOON_SUN_ELEVATION_RAD deg_to_rad(-6.0)`.

- [ ] **Step 1: Tests** in `cloud_world_volume_test.gd`:
  - `earth_of(grid_of(p))` within 1e-6 m for 1000 random points <= 1500 km from the origin, heights -0.5 .. 400 km;
    |(x, y)| = the great-circle distance on the projection sphere within 1e-9 relative.
  - `level_active`: level 0 on at 22.0 km, off at 22.2 km; level 1 off at 23.8 km; levels 2-7 on at 400 km. Level 0
    window at camera height 100 m = [-500, 2700] m. At the off height no cloud cell lies in level 0's window (no pop).
  - `texel_of` is a bijection of every window onto the level's texels for 100 random origins.
  - `exposed_slabs`: +3 cells in x -> one slab of 3 x 256 x 128; a diagonal move -> disjoint slabs whose cell count =
    new window minus overlap; a move >= 256 cells -> the whole level.
  - `fade_weight` 1 inside, 0 at the outermost cell, linear over 8 cells; for 1000 random cameras each level's window
    lies inside the next coarser level's region with weight 1 (the fade band always has a complete coarser level).
  - `reframe_needed` true at 1000.1 km, false at 999.9 km; `memory_bytes() == 8 * 256 * 256 * 128 * 16`.
  - Schedule: after `reset`, the first plan bakes and lights the whole coarsest level (non-provisional), then finer
    levels coarse to fine; a camera at 1 km/s (45 Hz frames) gets exactly its newly exposed slabs baked in the plan
    whose `lookup` adopts the new origin (never late, 600 frames); 7.8 km/s at 120 km: bake cells per frame
    0.15 M +- 20 %; Sun moved below / above `sun_threshold_rad(0, deg_to_rad(30), 12000)` (= 2.6e-4 rad) -> no light
    jobs / level 0 stale; blend moved 0.019 -> none, 0.02 -> stale; light cells per plan <= the calibrated budget;
    order provisional, then in-view and near, then most stale; a re-frame rebuilds levels coarsest first into the
    spare and every level has a shown version in every plan; `jumped` -> next plan is a cold start that still has a
    shown coarsest level in the same batch; Moon light jobs only with the Sun below -6 deg at the level centre;
    `plan()` p99 <= 1.0 ms over 1000 frames at 7.8 km/s.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** both classes.
- [ ] **Step 4: Run** `$G --headless --path game --import`, then the test: `cloud_world_volume_test: PASS`.
- [ ] **Step 5: Commit** `feat(game): cloud world volume layout and upkeep schedule`

### Task 4: Core reference of the cell bake

**Files:**
- Create: `core/include/starsim/environment/cloud_render_field.hpp`, `core/src/environment/cloud_render_field.cpp`
  (CPU mirror of the GPU's point model: `clouds.gdshaderinc` `cloud_extinction_prefiltered` / `cloud_extinction_in_cell`
  without the detail: field texels, LCL cut, type weights, shape value, threshold / edge-mean / prefilter tables)
- Create: `core/include/starsim/environment/cloud_volume_bake.hpp`, `core/src/environment/cloud_volume_bake.cpp`
- Modify: `core/CMakeLists.txt` (or the environment module's list), `bindings/src/starsim_core.hpp/.cpp`
- Test: `core/tests/test_cloud_volume_bake.cpp` (`[weather]`)

**Interfaces:**
- Produces (C++):
  `struct CloudRenderField` (the snapshot's render texels as uploaded: field RGBA8, ice R8, base RG32F, grid);
  `CloudRenderField make_cloud_render_field(const WeatherSnapshot& snapshot);`
  `struct CloudShapeFrame { math::Vec3 shape_origin_earth_m; math::Vec3 stratiform_origin_earth_m; math::Vec3
  ice_origin_earth_m; };` (the float64 noise origins the GPU gets);
  `double cloud_point_extinction_per_m(const CloudRenderField&, const CloudNoise&, const CloudShapeFrame&, const
  math::Vec3& position_earth_m, double footprint_m);`
  `struct CloudCellBake { double mean_extinction_per_m; double in_cloud_extinction_per_m; };`
  `CloudCellBake bake_cloud_cell(const CloudRenderField&, const CloudNoise&, const CloudShapeFrame&, const math::Vec3&
  centre_earth_m, double cell_horizontal_m, double cell_vertical_m);` - the band-limited cell mean (shape box-filtered
  mips + prefilter tables at the cell size, a few taps) and the envelope's sigma_c.
- Produces (binding): `StarsimCore.weather_cloud_cell_bake(snapshot_path: String, centre_earth_m: Vector3, cell_m:
  Vector2, shape_origins_earth_m: PackedFloat64Array) -> Vector2` (rho, sigma_c).

- [ ] **Step 1: Tests** (`[weather]`, slab and built-from-fields snapshots as in `test_weather_cover_water.cpp`):
  - point extinction inside a cf 1 slab = the slab's extinction within 2 % (8-bit code), 0 above it; 0 below the LCL
    cut;
  - per weather cell (0.25 deg) the sum of level-2 cell bakes x volume / the grid-mean water path in 0.996-1.000;
  - per volume cell |rho - dense mean| <= 0.02 sigma_c [E: few taps across 20-40 m edges] against a 16^3 stratified
    mean of the point model at a 10 m footprint, 200 cells per level size;
  - level consistency: a level-k+1 cell's rho = the mean of its 8 children's within 1 % [E: no pop in the fade band];
  - 0 <= rho <= sigma_c; a clear column gives exactly 0.
- [ ] **Step 2: Run, expect FAIL:** `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter
  "[weather]"` (does not compile).
- [ ] **Step 3: Implement** the mirror and the bake; the binding.
- [ ] **Step 4: Run** `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -TestFilter "[weather]"`: all pass;
  `$G --headless --path game --script res://tests/smoke_test.gd` passes.
- [ ] **Step 5: Commit** `feat(core): CPU reference of the cloud world volume's cell bake`

### Task 5: GPU bake, max-extinction mip, distance field

**Files:**
- Create: `game/shaders/cloud_volume.gdshaderinc` (the `CloudVolume` uniform block and lookups),
  `game/shaders/cloud_volume_bake.gdshaderinc` (`CLOUD_VOLUME_BAKE`, `CLOUD_VOLUME_OCCUPANCY`)
- Create: `game/world/weather/cloud_world_volume.gd` (class `CloudWorldVolume`)
- Modify: `game/world/weather/cloud_layer.gd` (`world_volume` flag + global `cloud_world_volume`, user argument
  `--world_volume`; owns layout + schedule; plan per frame; reset on `notify_view_jump` / `notify_clock_jump` / pinned
  snapshot; `world_volume_stats() -> Dictionary` with `bake_cells`, `light_cells`, `upkeep_ms`),
  `game/world/weather/cloud_effect.gd` (upkeep before the march, timestamp `cloud_volume_upkeep`),
  `game/tests/render_check.gd` (`--world_volume` / `--no_world_volume`), `game/tests/cloud_check.gd` (section 16; env
  `STARSIM_VOLUME_ONLY=16`; `STARSIM_WORLD_VOLUME=1` sets the flag in every GPU check)
- Test: `cloud_check.gd` section 16, `game/tests/cloud_layer_test.gd`

**Interfaces:**
- Consumes: Task 3 layout / schedule, Task 4 binding.
- Produces `CloudWorldVolume`: `ensure(rd: RenderingDevice) -> bool`; `run(rd: RenderingDevice, batch: Dictionary,
  set0: RID) -> void` (bake -> occupancy -> light (Task 6)); `density_texture(k: int) -> RID`, `light_texture(k) ->
  RID`, `max_extinction_texture(k) -> RID`, `distance_texture(k) -> RID`; `globals_bytes(lookup: Dictionary,
  camera_earth_m: Vector3) -> PackedByteArray` (std140; per level origin, active, fade; the camera-to-frame-origin
  offset differenced in double); `channel_of_slot(slot: int) -> int`; `free_resources(rd)`; `static texture_bytes() ->
  int`.
- Produces GLSL (`cloud_volume.gdshaderinc`): `vec3 cloud_volume_grid(vec3 offset_from_camera_m)`;
  `int cloud_volume_level(vec3 grid_m, out float fade)` (finest active level containing the point);
  `vec2 cloud_volume_density(vec3 grid_m, int level, float fade)` (rho, sigma_c; both snapshots blended by
  `cloud_blend`, fade toward level + 1); `vec4 cloud_volume_light(vec3 grid_m, int level, float fade)`;
  `float cloud_volume_free_m(vec3 grid_m, int level)`; `float cloud_volume_max_extinction(vec3 grid_m, int level)`.
- Encoding (builder's choice per spec) must pass the water bound below after quantisation.

- [ ] **Step 1: Tests**
  - Section 16 (analog and ERA5 2025-10-25 15:00, Starbase, camera 2 m and 9 km): 512 random cells per level 0, 2, 5
    read back vs `weather_cloud_cell_bake`: rho and sigma_c within 1 % + 1e-6 /m; per weather cell water ratio of the
    GPU level-2 bake in 0.996-1.000; distance field conservative: 1e5 random points per level - no cell with rho > 0
    (either snapshot) within the claimed Chebyshev distance, and on 1e4 points no per-sample density (sub-cell lift at
    the finest footprint) > 0 there (0 violations); max-extinction mip >= every child cell's sigma_c incl. the lift
    (0 violations); a rebake of a slab bit-identical; `cloud_world_volume_test`'s GLSL mirror check: 100 grid
    coordinates from `cloud_volume_grid` (probe kernel) vs `grid_of` within 0.1 m at 1000 km.
  - Pair switch (Review Focus 1): with the pair (A, B) shown at w 0.3 a new snapshot C replaces A only after w reached
    1; C is baked into A's channel pair while its weight is 0; B's channels read back unchanged during the bake.
    `cloud_layer_test.gd`: the volume's channel mapping follows the slots; the blend never jumps.
- [ ] **Step 2: Run, expect FAIL:** `STARSIM_VOLUME_ONLY=16 $G --xr-mode off --path game --script
  res://tests/cloud_check.gd` -> section unknown.
- [ ] **Step 3: Implement.** No march reads the volume yet; `world_volume` on runs the upkeep only.
- [ ] **Step 4: Run** section 16: `cloud_check: PASS`; `cloud_layer_test.gd`: `CLOUD LAYER TEST PASSED`;
  `smoke_test.gd`; report `gpu pass cloud_volume_upkeep` at cold start and cells per ms.
- [ ] **Step 5: Commit** `feat(game): cloud world volume - GPU bake of density, max-extinction mip and distance field`

### Task 6: Light volume and its refresh

**Files:**
- Create: `game/shaders/cloud_volume_light.gdshaderinc` (`CLOUD_VOLUME_LIGHT`, `CLOUD_VOLUME_PROVISIONAL`)
- Modify: `game/world/weather/cloud_world_volume.gd` (light dispatch, bounded step, provisional copy + 0.25 s blend),
  `cloud_world_volume_schedule.gd` (feedback wiring), `cloud_layer.gd` (Sun / Moon / blend / cloud top inputs),
  `game/tests/render_check.gd` (`--front_check=<warp>,<seconds>`), `game/tests/cloud_check.gd` (section 17)

**Interfaces:**
- Consumes: Tasks 3, 5.
- Produces: light texels (tau_sun, tau_below, tau_above, tau_moon) per cell: tau_sun with 32-64 Delta-tau-limited
  samples (step from the max-extinction mip, cell's level then coarser levels, then the grid-mean column
  `cloud_column_mean_depth` beyond level 7; Earth's shadow -> +inf); tau_below / tau_above along the geodetic
  vertical; tau_moon per `MOON_SUN_ELEVATION_RAD`. A refresh writes `old + clamp(new - old)` such that exp(-tau)
  moves by <= `LIGHT_STEP_T`, except cold start, jumps and re-frame levels (written directly) and provisional cells
  (blended in over `PROVISIONAL_FADE_S`). `render_check --front_check=W,S`: the clock at W x for S s in 1/45 s frames;
  per frame the 16 x 16-block means of the frame-to-frame radiance change; front step = max |difference of
  neighbouring blocks| / mean radiance; prints `front_check: max step <x> % lag <deg>`; exit 1 above 1 %.

- [ ] **Step 1: Tests** (section 17, `STARSIM_VOLUME_ONLY=17`):
  - section 8's slab (tau 10): tau_sun of cells at 5 heights in levels 0, 2, 5 vs the analytic slab depth / mu0
    within 5 %; tau_below, tau_above within 5 %;
  - section 8's broken cf 0.4 views: radiance with the volume light only (`light_entry_samples` 0) vs the converged
    per-sample light march - printed (p99, worst), asserted in Task 7;
  - Sun 5 deg below the cell's horizon: tau_sun infinite (or >= 6e4), radiance 0; no NaN in any channel of any level
    (readback scan); Sun -10 deg: tau_moon within 5 % of the slab's analytic depth along the Moon; Sun +10 deg: no
    Moon jobs;
  - a 5 deg Sun step: one refresh moves no cell's exp(-tau_sun) by more than 0.01 (readback before / after);
  - cold start: the first frame has clouds (coverage > 0 where the reference has cloud), every level's light complete
    within 1 s at 45 Hz under `UPKEEP_SPIKE_MS` (frames counted);
  - `$RC --cloud_view=V6 (sunset) --world_volume --front_check=100,10`: max step <= 1 %; and across the Moon switch
    (`--utc` with the Sun at -5.5 deg, `--front_check=100,20`): <= 1 %, no black frame.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** section 17, the two front checks, sections 16 and `cloud_world_volume_test`: PASS; report
  `cloud_volume_upkeep` median / p99 in V1 and V3 (light <= 1.0 ms, upkeep <= 1.5 ms after the cold start).
- [ ] **Step 5: Commit** `feat(game): cloud world volume - light volume with budgeted, rate-limited refresh`

### Task 7: Volume march (unsplit per eye)

The march itself, first as one full-resolution ray per eye pixel over [0, scene] (kernel `CLOUD_VOLUME_FULL`) - the
reference the tiers of Task 8 must match.

**Files:**
- Create: `game/shaders/cloud_volume_march.gdshaderinc` (`cloud_volume_march`, the step rule, sub-cell field,
  `edge_curl`, surface correction, level fade, counters / ablations of Task 0; kernel `CLOUD_VOLUME_FULL`)
- Modify: `core/include/starsim/environment/cloud_noise.hpp`, `core/src/environment/cloud_noise.cpp` (a tileable curl
  texture `curl` 32^3 RGB8 in `CloudNoise`), `core/tests/test_cloud_optics.cpp` or a new `[weather]` case;
  `game/world/weather/cloud_layer.gd` (`light_entry_samples` 4, `edge_curl` 1.0 [E: amplitude 1 x the detail cell
  size], curl texture upload), `cloud_effect.gd` (world_volume path: upkeep -> FULL per eye -> output),
  `game/tests/render_check.gd` (`--ab_reference`, `--edge_check`), `cloud_check.gd` (section 19),
  `game/tests/cloud_grain_check.gd` (`STARSIM_WORLD_VOLUME=1`)

**Interfaces:**
- Consumes: Tasks 5-6 GLSL.
- Produces GLSL: `void cloud_volume_march(vec3 dir_earth, vec3 origin_offset, float from_m, float to_m, float
  pixel_rad, vec3 eclipse_near, vec3 eclipse_far, out vec3 color, out float transmittance, out float cloud_m)`
  (`cloud_m` = transmittance-weighted cloud distance; air loss composited as today). Sub-cell field: density =
  sigma_c m(c) smoothstep((v_sub - t(c)) / w), c = rho / sigma_c, existing threshold / mean tables; v_sub = shape
  residual above the cell scale (levels >= 2, one fetch at the footprint's mip) + the three detail levels, each
  w_k = clamp(log2(l_k / f) - 1, 0, 1); early-out where |c - t| leaves no room; weights 0 -> baked rho. Light per
  sample: volume tau (+ the 4-sample surface correction), transfer table, `cloud_sky_scattering_source` with tau_above
  x the sky irradiance re-evaluated every 500 m / 5 % (`CLOUD_LIGHT_REFRESH_M`), Sun disk handling unchanged.
- Produces render_check: `--ab_reference` (the cloud image vs the reference path `world_volume` off at divisor 1 and
  the grain check's `REFERENCE_STEP_FACTOR` 8 x steps; median <= 1 %, p99 <= the reference against itself turned
  0.5 px; replaces `--ab_view_cache`'s logic); `--edge_check` (edge pixels alpha 0.05-0.95: edge-band width and alpha
  high-pass energy vs the reference; bound = max of the reference's own change under the four 0.5 px turns, capped at
  15 %); `--world_volume_unsplit` (forces `CLOUD_VOLUME_FULL` after Task 8).
- `cloud_check` section 19 (no level seam): rows of pixels crossing each active level's fade band (as section 10
  crosses cascades): radiance step across the band <= 1 %.

- [ ] **Step 1: Tests** (all with `STARSIM_WORLD_VOLUME=1` / `--world_volume`):
  - curl texture: numerical divergence <= 1e-2 of the gradient magnitude ([weather]);
  - cloud_check sections 4 (slab vs MC 3 %), 5 (coverage 0.02), 9, 12 (LCL 0.9993), 13 (Sun disk unchanged), 14
    backlit transmitted radiance within 5 %, 17 with `light_entry_samples` 4 inside section 8's bounds (slab 5 %,
    broken p99 10 % / worst 20 %), 19 (<= 1 %);
  - `$RC --cloud_view=V1 --world_volume --ab_reference`, same for V3, V6: exit 0; `--edge_check` at V1, V3: exit 0;
    `--repeat_identical` at V1, V3: exit 0;
  - `cloud_grain_check.gd` with `STARSIM_WORLD_VOLUME=1`: grain / alpha grain within 25 %, cover / mean within 3 %,
    RMS <= 0.1, next frame bit-identical.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** Step 1's checks: PASS; report counters (occupied / empty steps per pixel, fetches per occupied
  step; goal 3-8, near ~10) and `gpu pass cloud_volume_full` at V1 / V3 in the stereo harness; images
  `edge_curl` 0 vs 1 side by side at V1 / V3 to `screenshots/cloud-world-volume/` for the gate.
- [ ] **Step 5: Commit** `feat(game): cloud world volume march - sphere tracing, sub-cell edges, surface-corrected
  light`

### Task 8: Near / far tiers, Earth-fixed far grid, exact composite

**Files:**
- Create: `game/world/weather/cloud_far_grid.gd` (class `CloudFarGrid`; union-of-frusta math from
  `cloud_centre_view.gd`)
- Create: `game/shaders/cloud_volume_tiers.gdshaderinc` (`CLOUD_VOLUME_NEAR`, `CLOUD_VOLUME_FAR`,
  `CLOUD_VOLUME_CYCLOPS_DEPTH`, `CLOUD_VOLUME_COMPOSITE`)
- Modify: `cloud_effect.gd` (tier path; timestamps `cloud_volume_near`, `_far`, `_composite`),
  `cloud_layer.gd` (`near_scale` presets: desktop 1.0, vr 1.0 [E: spec start, the gate may apply F1 0.7]),
  `game/util/perf_log.gd` (columns `cloud_near_ms`, `cloud_far_ms`, `volume_upkeep_ms`), `render_check.gd`
  (`--jump_check` with `--world_volume`)
- Test: `cloud_world_volume_test.gd`, `cloud_vr_stereo_test.gd`, `cloud_check.gd` section 18

**Interfaces:**
- Consumes: Task 7 `cloud_volume_march`.
- Produces `CloudFarGrid.new(face_texels: int)`; `static face_texels_for(projection: Projection, view_size: Vector2i,
  scale: float) -> int` (texel angle <= 1 / (scale x the eye's px/rad) everywhere on a face, multiple of `TILE` 8
  [E]); `texel_of(dir_earth: Vector3) -> Vector3`; `dir_of(face: int, uv: Vector2) -> Vector3`;
  `tiles_in(view_projections: Array[Projection], eye_to_earth: Array[Transform3D], margin_rad: float) ->
  PackedInt32Array`; `storage_bytes() -> int`. Directions Earth-fixed; the per-frame tile list drives the far
  dispatch; nothing is kept for the next frame.
- Near tier per eye over [0, min(R_near, scene)] at `near_scale`; skipped when `cloud_volume_free_m` at the eye (finest
  active level) > R_near. Composite per eye pixel: L = L_near + T_near L_far(dir), T = T_near T_far; far term 0 where
  the eye's scene depth < R_near; depth-aware upsample guided by the tiers' `cloud_m` and the scene depth, no history.

- [ ] **Step 1: Tests**
  - `cloud_world_volume_test`: `dir_of(texel_of(d))` within 1e-9 rad; texel angle at face centre <= 1 / (0.5 x
    2005) rad for the recorded projection (35 px/deg); `tiles_in` covers 1e4 sampled pixel directions of both eyes
    (+ margin) and no tile beyond union + margin + 1 tile; head yawed 5 deg at the same position: the shared tiles'
    directions bit-identical; `storage_bytes() <= 300 MB`.
  - `cloud_vr_stereo_test`: split mirror on a synthetic medium (exponential + slab, 1D) - near + T_near far = unsplit
    within 1e-6; far term 0 for scene depth < R_near; a far-tier cloud point at 2 / 5 / 20 km composited per eye vs
    its per-eye ray: <= 0.1 px at 35 px/deg.
  - Section 18 (stereo harness, V1 and V3): composite with the far tier at 1.0x vs `CLOUD_VOLUME_FULL` per eye: p99
    per-pixel luminance difference <= 0.5 %; at 0.5x the `--ab_reference` bounds against FULL.
  - Review Focus 4: V1 with the tower in view (`--site=starbase_olp1`): far-term debug output exactly 0 on pixels with
    scene depth < R_near; edge pixels within the `--ab_reference` bounds against FULL.
  - Review Focus 5: `$RC --cloud_view=V3 --world_volume --jump_check` (incl. its layout re-creation step): every frame's
    coverage within 0.02 of the reference; `world_volume_stats().bake_cells` 0 in the re-creation frame (no rebake).
  - near tier skipped (dispatch count 0) at V3 and V5, present at V1; `--shimmer_check` at V3 (head turned): ratio
    <= 1.10; `--repeat_identical` at V1, V3.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** Step 1's checks and Task 7's `--ab_reference` V1 / V3: PASS; report per-tier `gpu pass` at V1,
  V3 in the stereo harness with the recorded projections.
- [ ] **Step 5: Commit** `feat(game): cloud world volume - per-eye near tier, shared Earth-fixed far tier, exact
  composite`

### Task 9: Scroll / re-frame in the frame, far hand-over beyond the levels

**Files:**
- Modify: `cloud_world_volume.gd` (slab jobs every frame before the march, spare level, re-frame cross-fade),
  `cloud_volume_march.gdshaderinc` (beyond level 7: density from the snapshot directly - the reference model
  band-limited - and light from the grid-mean column `cloud_far_light_depth`; beyond the far fade
  `cloud_far_slab` / `cloud_far_column`, composited by the ray split; `handover_m` unused on this path),
  `cloud_layer.gd` (`world_volume_stats() -> Dictionary`), `perf_log.gd` (columns `volume_bake_cells`,
  `volume_light_cells`, `volume_late_slabs`, `volume_provisional_max_s`), `render_check.gd` (`--frames_every` with
  `--world_volume` compares against the reference per frame; `--reframe_check`), `cloud_check.gd` (section 19: the
  level-7 edge), `game/tests/cloud_far_check.gd` (`STARSIM_WORLD_VOLUME=1`)

**Interfaces:**
- Consumes: Tasks 3, 5-8.
- Produces: `volume_late_slabs` = frames in which a march read a level whose exposed slab was not yet baked (must be 0);
  `render_check --reframe_check`: a camera at 9 km moving east at 7.8 km/s for 150 s (crosses a re-frame): per frame
  coverage vs the reference <= 0.02, `volume_late_slabs` 0, upkeep <= 3 ms, radiance step at the cross-fade <= 1 %.

- [ ] **Step 1: Tests**
  - `$RC --cloud_view=V2 --world_volume --render_scale=1.65 --cloud_quality=vr` (and `--watch_from_site=2000`):
    coverage within 0.02 of the reference in EVERY sample frame (no in-deck 0.06 exception), `volume_late_slabs` 0,
    upkeep p99 <= 1.5 ms; the flight extended to 30 km (`--frames_for=200`): no step at the level 0 / 1 off heights
    (Review Focus 2);
  - `$RC --world_volume --reframe_check`: exit 0;
  - section 19 with the camera at 80 km and 400 km: the level-7 edge and the far fade <= 1 %;
  - `cloud_far_check.gd` with `STARSIM_WORLD_VOLUME=1`: limb speckle 5 %, 50 km horizon <= 10 %, far field vs volume
    10 %, scaled-space hand-over 5 %.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** Step 1's checks, sections 16-18, `cloud_world_volume_test`: PASS.
- [ ] **Step 5: Commit** `feat(game): cloud world volume - never-late scroll, re-frame, far hand-over beyond the levels`

### Task 10: Air shadow and shadow maps from the light volume

**Files:**
- Modify: `cloud_volume_march.gdshaderinc` / `cloud_volume_tiers.gdshaderinc` (Bruneton difference form kept; T_c =
  exp(-tau_sun) from the light volume, beyond level 7 the far map's grid mean; far tier 32 segments over [R_near,
  end], near tier 4 segments; composited like L), `game/shaders/cloud_shadow.gdshaderinc`,
  `game/world/weather/cloud_shadow_effect.gd` (with `world_volume` the two maps are filled from the light volume, one
  fetch per texel; `grid_mean_only` kept), `cloud_check.gd` (section 14 addition)

**Interfaces:**
- Consumes: Task 6 light texels, Task 8 tiers. Map texel semantics unchanged for the terrain shaders.

- [ ] **Step 1: Tests** (`STARSIM_WORLD_VOLUME=1`): cloud_check 14 (backlit within its bounds, crepuscular residual
  >= 2 %, segment sum 0.03 %), plus a new case: a cumulus 40 km up-Sun of the camera casts its shadow on the air
  (residual >= 2 %; the old maps reached 9-11 km); cloud_check 6 (3 % / 5 % / 10 %) and 7 (hand-overs) within their
  bounds; `gpu pass cloud_shadow` <= 0.1 ms [E: spec ~0.05].
- [ ] **Step 2: Run, expect FAIL** (the 40 km case).
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** sections 6, 7, 14, 18: PASS; report air-shadow ms per tier (ablation `air_shadow`).
- [ ] **Step 5: Commit** `feat(game): cloud air shadow and shadow maps read the world light volume`

### Task 11: GATE (STOP 2)

Controller-led; measurement and judgement, code only for fall-backs.

**Files:** Docs `vr-performance.md` (gate table), `weather-clouds.md` (current state of the prototype); presets in
`cloud_layer.gd` only through fall-backs.

- [ ] **Step 1: Performance matrix** (stereo harness, recorded projections, `--world_volume --gpu_passes
  --counters`): V1-V7 (V1 / V3 / V4 / V5 with analog and ERA5); per tier median / p99 ms, upkeep, air shadow,
  composite; occupied / empty steps per pixel; fetches per occupied step. V2 / V7 as flights (`--frames_every`).
- [ ] **Step 2: Quality matrix:** `--ab_reference`, `--edge_check`, `--shimmer_check`, `--repeat_identical` at V1-V7;
  `cloud_grain_check`; coverage <= 0.02 every frame of V2 / V7; cloud_check 4, 5, 8/17 (entry 4), 13, 14, 16-19;
  `cloud_far_check`; V6 `--front_check`; jumps (`--jump_check`): clouds in the first frame, full volume <= 1 s,
  coverage within 0.02 after it.
- [ ] **Step 3: Kill criteria** (spec G, verbatim): far tier > 10 ms p99 at 0.5x; near tier > 7.5 ms p99 at 0.7x in
  V1 / V2 / V7; total cloud GPU > 14 ms p99 after the fall-backs; light volume outside cloud_check 8's bounds with 4
  entry samples; any visible level / slab / refresh seam (radiance step across a fade band or front > 1 %); a still
  frame not bit-identical.
- [ ] **Step 4: Fall-backs in order, each reported to the user:** F1 near tier full -> 0.7x; F2 level one coarser at
  range and fewer detail octaves at range (band cut at 2 x footprint), checked by `--edge_check`; F3 R_near 2 -> 1 km
  (disparity 0.13 px); F4 altitude views only: lower the column model's fade footprint (user decides). Then stop and
  report; no further architecture switch and no far tier below 0.5x without the user.
- [ ] **Step 5: Headset run with the user:** `Start-VR.cmd --perf-log --world_volume`: pad, ascent through a deck
  (autopilot), above the decks, orbit, booster catch; cloud GPU median / p99 per phase (<= 11 / <= 14 ms), frame p99
  <= 22.2 ms; the user's look verdict (sharp, steady, feathery edges, no seams / pops / shimmer); the user decides
  `edge_curl` (side-by-side images of Task 7) and confirms `near_scale`.
- [ ] **Step 6: Docs + commit** `docs: cloud world volume gate - measurements, kill criteria, decisions`
- [ ] **STOP 2 (controller):** present the gate result; Task 12 starts only on the user's go.

### Task 12: Re-entry scroll / re-frame acceptance

**Files:** Modify `render_check.gd` only if the flight path needs it; docs `vr-performance.md`.

- [ ] **Step 1: Check:** `$RC --mission=ship_return_starbase --at_event=entry_interface,0 --world_volume
  --render_scale=1.65 --cloud_quality=vr --frames_every=0.5 --frames_for=600 --weather=analog` (log to a file,
  background if > 10 min): `volume_late_slabs` 0 (no frame reads a coarser level's DENSITY because a slab was late),
  `volume_provisional_max_s` <= 1.0, upkeep <= 3 ms in re-frame frames and <= 1.5 ms p99 otherwise, volume bytes <=
  1.1 GB + the spare level, main-thread `plan()` <= 1.0 ms per frame, coverage within 0.02 of the reference per frame.
- [ ] **Step 2: Fix** what fails (a fresh fixer per finding set), re-run.
- [ ] **Step 3: Docs + commit** `test(game): cloud world volume at re-entry speed - scroll and re-frame`

### Task 13: Switch-over

**Files:**
- Modify: `cloud_layer.gd` (`world_volume` default true; final presets from the gate; the light cache allocated only
  with `world_volume` false), `cloud_effect.gd` (production path = tiers; reference path = today's march behind
  `world_volume = false` / `STARSIM_MARCH_REFERENCE`, at divisor 1 and 8 x steps for checks)
- Delete (one `refactor:` commit): `game/world/weather/cloud_view_cache.gd`, `cloud_view_cache_layout.gd`,
  `cloud_view_cache_planner.gd`, `cloud_view_cache_schedule.gd`, `game/shaders/cloud_view_cache.gdshaderinc`,
  `game/tests/cloud_view_cache_test.gd`, cloud_check section 15's cache-fill part (its march-reference bit-identity
  via `STARSIM_MARCH_REFERENCE` stays, as the reference path's check), `render_check --ab_view_cache` /
  `--no_view_cache`,
  the `view_cache` flag and its PerfLog columns; the spike files (`cloud_spike.gd`, `cloud_spike.gdshaderinc`,
  `--spike`); their `.uid` files
- Docs: `weather-clouds.md` (world volume = current state; the view cache's record and lessons, the light cache, the
  hybrid light march, sub-samples, outline pass, 20 / 34 km band moved verbatim to `docs/history/weather-clouds.md`),
  `vr-performance.md`, `docs/known-issues.md`, `docs/architecture.md` (game structure line if it names the view
  cache); archive `docs/superpowers/specs/2026-10-27-cloud-view-cache-design.md` and
  `docs/superpowers/plans/2026-10-27-cloud-view-cache.md` to `docs/superpowers/archive/`

- [ ] **Step 1: Test:** `grep -rn "view_cache\|ViewCache" game/ docs/subsystems/` returns nothing outside history.
- [ ] **Step 2: Default flip** + presets; run `smoke_test`, `cloud_layer_test`, `cloud_vr_stereo_test`,
  `cloud_world_volume_test`, `cloud_check` (all sections), `cloud_far_check`, `cloud_grain_check`, `$RC --cloud_view=V1
  --repeat_identical`, and the reference path `$RC --cloud_view=V1 --no_world_volume --repeat_identical`: PASS.
  Commit `feat(game): clouds from the world volume by default`.
- [ ] **Step 3: Delete** the view cache in one commit; `$G --headless --path game --import`; re-run Step 2's list and
  Step 1's grep. Commit `refactor(game): remove the direction view cache (replaced by the cloud world volume)`.
- [ ] **Step 4: Docs** + commit `docs: cloud world volume is the cloud rendering architecture`

### Task 14: Final acceptance

- [ ] **Step 1: Headset run with the user** (`Start-VR.cmd --perf-log`): pad, deck passage, above the decks, orbit,
  re-entry, booster catch with weather on: frame p99 <= 22.2 ms, cloud GPU <= 11 ms median, <= 14 ms p99 per phase;
  weather off 90 Hz; the user's look verdict. Fix rounds as needed.
- [ ] **Step 2: Docs + memory** (`project-m8-weather-clouds.md`, `project-milestone-order.md`), archive this plan and
  the spec to `docs/superpowers/archive/`; final whole-plan review (Opus).
- [ ] **Step 3: Commit** `docs: cloud world volume - VR acceptance and closing`
