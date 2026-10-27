# Cloud View Cache Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Clouds rendered into a world-fixed cache around the camera and only looked up per eye per frame - sharp at
the display's resolution, no shimmer, fixed in the world - with weather on at a steady 45 Hz in VR.

**Architecture:** A cube-map cache (colour + T, cloud distance) around a float64 Earth-local anchor is filled tile by
tile with the existing march (one shared ray-shading function) under a GPU budget, coarse to fine, cross-faded. Each
eye's upsample kernel is replaced by a parallax lookup into the cache plus the per-frame depth test against the scene.
A near shell (0..R_near) is marched per frame at today's coarse density when clouds are close. The per-frame march
stays the reference path behind a flag.

**Tech Stack:** Godot 4.7 double GDScript, RenderingDevice compute (GLSL via `CloudShaderPort`), D3D12, OpenXR.

**Spec:** `docs/superpowers/specs/2026-10-27-cloud-view-cache-design.md` (approved by the user 2026-10-27; read its
"Decisions taken for the builder" first).

## Global Constraints

- The look is the existing march's: the cache is filled by the same shading code; no second density / light model.
- Frame target: weather on -> 45 Hz in VR (22.2 ms), weather off -> 90 Hz (11.1 ms) (user rule 2026-10-27).
- The cache's contents never depend on the frame's depth buffer; scene geometry hides clouds per frame by depth.
- Anchor and eye positions in float64 Earth-local; offsets differenced in double before float32 (floating origin).
- Both eyes read one cache; per-eye geometry only from the renderer's matrices (`cloud_vr_stereo_test.gd` stays green).
- One path for desktop and VR; the per-frame march stays behind the flag `CloudLayer.view_cache` (global
  `cloud_view_cache`, render_check `--no_view_cache`) as the reference.
- VRAM of the cache <= 2 GB [E].
- No magic numbers: every new constant `[E]` with its rationale in a comment; SI units, frame / unit in names.
- Tests: only the affected scripts (`cloud_view_cache_test.gd`, `cloud_vr_stereo_test.gd`, `cloud_check.gd`,
  `cloud_far_check.gd`, `cloud_layer_test.gd`, `smoke_test.gd`, `render_check.gd` cases named per task); GPU checks
  need a window (no `--headless`); images to `screenshots/cloud-view-cache/`; long runs to files, read the tail.
- Commits: Conventional Commits, push to `main` after each green task (no feature branches).
- Docs per task: `docs/subsystems/weather-clouds.md` (current state; superseded text to
  `docs/history/weather-clouds.md`), measurements in `docs/subsystems/vr-performance.md`.

## Review Focus

- Camera jump / clock jump / pinned snapshot (Tab to another site, mission start): the cache restarts coarse at once -
  never a frame of the old site's clouds (Task 4 test).
- Sun moving under time warp while tiles refill: no tile switches its light abruptly (cross-fade) and no tile mixes two
  Sun states (Task 4 test).
- Head turned fast toward a never-filled direction (start-up, after a jump): a coarse level is there, never a hole or
  black (Task 4 test: lookup of an unfilled tile falls back to the coarse level / the near-shell march).
- Camera moving along a deck at 300 m/s: no gap or double cloud at the near-shell boundary R_near (Task 5 test).
- Window resize / quality preset change / VR start-stop: the cache's resolution follows without a frame without clouds
  (Task 3 test).

---

### Task 1: Baseline and the frame-rate rule

Measurement first (spec decision): the budget the cache has, and how 45 Hz is held on the Pimax.

**Files:**
- Modify: `game/xr/xr_manager.gd` (the frame-rate target: weather on -> 45 Hz, off -> 90 Hz)
- Modify: `game/world/weather/cloud_layer.gd` (a debug switch `pass_enabled` that skips the cloud pass; render_check
  `--no_cloud_pass`, PerfLog-visible)
- Modify: `game/tests/render_check.gd` (`--no_cloud_pass`)
- Test: `game/tests/xr_frame_rate_test.gd` (headless)
- Docs: `docs/subsystems/vr-performance.md` (the measured table)

**Interfaces:**
- Produces: `XrManager.target_hz(weather_enabled: bool) -> float` (45.0 / 90.0; constants `TARGET_HZ_WEATHER`,
  `TARGET_HZ_CLEAR`), applied when `WeatherService.enabled` changes and at XR start; `CloudLayer.pass_enabled: bool`.

- [ ] **Step 1: Test** `xr_frame_rate_test.gd`: `target_hz(true) == 45.0`, `target_hz(false) == 90.0`; toggling the
  weather on a running `XrManager` (desktop mode: no change applied, no error) - PASS on exit code 0.
- [ ] **Step 2: Run, expect FAIL** (function missing).
- [ ] **Step 3: Implement** the rule. How it is applied in XR is measured, not assumed: variant A `Engine.max_fps = 45`
  (the runtime reprojects the missing display frames), variant B the OpenXR display refresh rate if the Dream Air offers
  a rate the user accepts; start with A.
- [ ] **Step 4: Desktop proxy baseline** (window, VR pixel proxy): `render_check.gd --render_scale=1.65 --gpu_time
  --gpu_passes --cloud_quality=vr --weather=analog` at the Starbase ground view and a 9 km deck view, each with and
  without `--no_cloud_pass`, and `--weather` off; write the medians / p99 per pass into `vr-performance.md`.
- [ ] **Step 5: Headset run with the user** (Start-VR.cmd `--perf-log`, weather on, pad 60 s, `--no_cloud_pass`
  variant 60 s): GPU frame without the cloud pass = the cache budget's base; variant A smooth at 45 Hz? Record the
  user's verdict and the numbers; if variant A judders, try B before Task 2.
- [ ] **Step 6: Report** the budget (22.2 ms - scene p99 - margin 2 ms [E]) to the user before Task 2.
- [ ] **Step 7: Commit** `feat(game): VR frame-rate target 45 Hz with weather, 90 Hz without; cloud pass switch`

### Task 2: Cache layout and schedule (pure logic)

**Files:**
- Create: `game/world/weather/cloud_view_cache_layout.gd` (class `CloudViewCacheLayout`: cube faces, tiles, levels,
  direction <-> texel, resolution from a projection)
- Create: `game/world/weather/cloud_view_cache_schedule.gd` (class `CloudViewCacheSchedule`: tile state, staleness,
  priority, the per-frame pick under a budget; pattern of `cloud_light_cache_schedule.gd`)
- Test: `game/tests/cloud_view_cache_test.gd` (headless)

**Interfaces:**
- Produces: `CloudViewCacheLayout.new(face_texels: int, tile_texels: int)`; `static func face_texels_for(projection:
  Projection, view_size: Vector2i) -> int` (the eyes' pixel density at the view centre over a 90 deg face, rounded up
  to a multiple of `tile_texels`); `func tile_of(dir: Vector3) -> int`; `func texel_of(dir: Vector3) -> Vector3`
  (x, y in the face, z = face); `func dir_of(face: int, uv: Vector2) -> Vector3`; `func tile_count() -> int`;
  `const LEVELS := 3` (level 0 = 1/4 density, 1 = 1/2, 2 = full; [E]: level 0 ~ today's 1/3 at the first fill).
- Produces: `CloudViewCacheSchedule.new(layout)`; `func invalidate_all()`; `func mark_light(sun_dir: Vector3,
  weather_weight: float, light_generation: int)` (tiles whose state differs by more than `SUN_REFRESH_RAD` 0.002 [E]
  (~0.1 deg: one pixel of the evening light's gradient) or `WEATHER_REFRESH` 0.02 [E] become stale);
  `func pick(camera_earth_m: Vector3, anchor_earth_m: Vector3, view_dirs: Array[Vector3], budget_tiles: int) ->
  Array[Dictionary]` ({tile, level} in priority order); `func parallax_error_px(tile: int, camera_earth_m: Vector3,
  anchor_earth_m: Vector3) -> float`; `func completed(tile: int, level: int, state: Dictionary)`.
- Priority: (1) tiles in the view (+ `VIEW_MARGIN_RAD` 0.35 [E]: a fast head turn) without level 0, (2) visible tiles
  by parallax error over `PARALLAX_REFRESH_PX` 0.5 [E] (half an eye pixel) times nearness, (3) visible tiles below the
  full level, (4) stale light / weather, (5) the rest.

- [ ] **Step 1: Tests** in `cloud_view_cache_test.gd`: `dir_of(texel_of(d)) ~= d` within 1e-9 rad for 1000 random
  directions incl. face edges and corners; every direction maps to exactly one tile; `face_texels_for` of the run's
  projection (3534 x 2838, ~100 deg) is within 10 % of 3200 and a multiple of the tile size; after `invalidate_all`
  `pick` returns level-0 tiles of the view first; a 0.5 deg Sun change marks all tiles stale, 0.05 deg none; the
  parallax error of a tile with a 2 km cloud distance at 10 m camera offset at 35 px/deg is ~10 px (+-5 %);
  `pick` never returns more than `budget_tiles`.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** both classes (the tile's cloud distance for the parallax error: its nearest cached cloud
  distance, read back in Task 4; here a setter `set_tile_distance(tile, near_m)`).
- [ ] **Step 4: Run** `cloud_view_cache_test.gd`: PASS.
- [ ] **Step 5: Commit** `feat(game): cloud view cache layout and refresh schedule`

### Task 3: Fill kernel and the one shading function

**Files:**
- Modify: `game/shaders/cloud_pass.gdshaderinc` (extract `void cloud_shade_ray(vec3 dir_earth, vec3 origin_offset,
  float scene_m, ivec2 noise_pixel, vec3 eclipse_near, vec3 eclipse_far, out vec4 color_t, out float cloud_m)` from
  the march `main` - the non-refine path, air shadow included; `main` calls it, bit-identical)
- Create: `game/shaders/cloud_view_cache.gdshaderinc` (`CLOUD_CACHE_FILL` kernel: one invocation per texel of the
  picked tiles; ray from the anchor; `scene_m` = the static world: the ellipsoid / sea crossing from the anchor, else
  1e30; writes colour+T (RGBA16F) and cloud distance (R32F) at the tile's level)
- Create: `game/world/weather/cloud_view_cache.gd` (class `CloudViewCache`: the cube textures per level, the
  tile-list buffer, `fill(rd, list_tiles, anchor_offset_earth, ...)`, `free()`; re-creates on a new face size)
- Modify: `game/world/weather/cloud_effect.gd` (compile the fill kernel; dispatch fill before the march)
- Test: `game/tests/cloud_check.gd` (new section 15)

**Interfaces:**
- Consumes: `CloudViewCacheLayout` (Task 2).
- Produces: `CloudViewCache.texture(level: int) -> RID` (cube array: colour+T), `.distance_texture(level) -> RID`;
  `CloudViewCache.fill(rd: RenderingDevice, tiles: Array[Dictionary], anchor_offset_earth: Vector3, set0: RID) -> void`.

- [ ] **Step 1: Test** section 15 (desktop, window): the march `main` before / after the extraction - `render_check
  --repeat_identical`-style identical frame (0 differing pixels) at the deck and ground views; then the fill of the
  full cube at level 2 with anchor = camera vs the per-frame march at divisor 1 over a 256 x 256 view, both debug
  output 0 and 12 (distance): colour per pixel <= 1 % median, <= 5 % at the 99th percentile (texel vs pixel
  resampling at edges), T RMS <= 0.01, cloud distance <= 1 % where alpha > 0.5.
- [ ] **Step 2: Run, expect FAIL** (section 15 unknown).
- [ ] **Step 3: Implement** the extraction (Step 1's identity first, commit-worthy alone), then the fill kernel and
  `CloudViewCache`.
- [ ] **Step 4: Run** `cloud_check.gd` (all sections), `cloud_far_check.gd`, `cloud_vr_stereo_test.gd`,
  `smoke_test.gd`: PASS.
- [ ] **Step 5: Commit** `feat(game): cloud view cache fill from the shared ray shading`

### Task 4: Lookup with parallax, schedule in the frame, cross-fade, jumps

**Files:**
- Modify: `game/shaders/cloud_view_cache.gdshaderinc` (`CLOUD_CACHE_LOOKUP` kernel per eye, replacing `CLOUD_UPSAMPLE`
  when the cache is on: parallax lookup + depth test + level choice + cross-fade weight)
- Modify: `game/world/weather/cloud_effect.gd` (the cache path: schedule pick -> fill -> lookup per eye; GPU-timestamp
  budget control; read back each filled tile's nearest cloud distance)
- Modify: `game/world/weather/cloud_layer.gd` (owns `CloudViewCache` + schedule; `view_cache` flag + global
  `cloud_view_cache`; presets `cache_budget_ms` (vr: Task 1's reported budget 22.2 ms - scene p99 - 2 ms margin [E];
  desktop 6.0 [E]);
  `reset` on `_view_jumped` / `SimClock.time_jumped` / pinned snapshot)
- Modify: `game/tests/render_check.gd` (`--no_view_cache`; `--shimmer_check`)
- Modify: `game/util/perf_log.gd` (columns `cache_tiles_filled`, `cache_full_fraction`, `cache_fill_ms`)
- Test: `game/tests/cloud_vr_stereo_test.gd` (parallax mirror), `game/tests/cloud_layer_test.gd` (reset / cross-fade),
  `render_check.gd --shimmer_check`

**Interfaces:**
- Consumes: Tasks 2-3.
- Produces: GLSL `vec4 cloud_cache_lookup(vec3 eye_offset_earth, vec3 dir_earth, out float cloud_m)` (iterative
  parallax: `CACHE_PARALLAX_ITERATIONS` 3 [E]); the lookup takes the finest level filled for the texel and blends
  toward it over `CACHE_CROSSFADE_S` 0.25 s [E] (per-tile fade start time in a buffer); unfilled at every level ->
  T = 1 with the near shell / per-frame coarse march covering (Task 5) and level 0 filled first anyway.

- [ ] **Step 1: Tests**
  - `cloud_vr_stereo_test.gd`: GDScript mirror of the parallax lookup - a point cloud at 300 m / 2 km / 20 km seen
    from each eye (IPD 0.064 m) and from a camera 50 m off the anchor lands within 0.25 eye pixel of the direct ray
    (35 px/deg).
  - `cloud_layer_test.gd`: a view jump and a clock jump call the cache reset; a Sun step of 1 deg starts cross-fades,
    none completes in the same frame.
  - `render_check.gd --shimmer_check` (deck view, cache full): 16 frames with the head yawed by +-0.3 eye pixel
    alternating; per pixel the frame-to-frame change of the cloud image must not exceed the full-resolution direct
    march's (`--no_view_cache --cloud_divisor=1`) change on the same sequence by more than 10 % (99th percentile) -
    exit code 1 otherwise.
  - `render_check.gd` A/B at the deck / ground / 20 km / dusk views, cache full vs `--no_view_cache
    --cloud_divisor=1`: the section 15 bounds; images to `screenshots/cloud-view-cache/`.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** the lookup, the frame loop and the budget control (tiles per frame adjusted from the
  previous frames' `cloud_cache_fill` timestamps toward `cache_budget_ms`, never below 1 tile).
- [ ] **Step 4: Run** the Step 1 checks, `cloud_check.gd`, `smoke_test.gd`: PASS; send the A/B images to the user.
- [ ] **Step 5: Commit** `feat(game): clouds looked up from the world-fixed view cache per eye`

### Task 5: Near shell and re-anchoring

**Files:**
- Modify: `game/shaders/cloud_pass.gdshaderinc` (the centre march limited to `[0, R_near]` when the cache is on:
  `cloud_view.params` carries R_near), `game/shaders/cloud_view_cache.gdshaderinc` (fill from `R_near` on)
- Modify: `game/world/weather/cloud_effect.gd` (near shell only when a cloud is within R_near of the camera: the field's
  nearest cloud distance from `CloudLayer`; composite near shell over the cache lookup)
- Modify: `game/world/weather/cloud_view_cache_schedule.gd` (re-anchor when `|camera - anchor| > REANCHOR_FRACTION`
  0.05 [E] x R_near; progressive: the new anchor's tiles fill while the old ones are shown, old freed per tile)
- Test: `game/tests/cloud_view_cache_test.gd`, `render_check.gd --fly` sequence

**Interfaces:**
- Produces: `CloudLayer.near_radius_m()` (R_near: 2000.0 [E], checked against the parallax error budget in Step 1);
  `CloudViewCacheSchedule.reanchor_needed(camera_earth_m) -> bool`.

- [ ] **Step 1: Tests**
  - `cloud_view_cache_test.gd`: re-anchor triggers at 5 % of R_near, not below; during a re-anchor every tile has a
    shown level (old or new).
  - `render_check.gd --vehicle=starship_block3 --site=starbase_olp1 --autopilot --fly=25
    --weather=slab:1500,2500,0.0003,0.6 --watch_from_site=2000` and the same from on board (no `--watch_from_site`):
    the ascent through the deck, frames every 0.5 s (a `--frames_every=0.5` option of render_check if missing): no ring at R_near - the mean alpha along the boundary annulus within 0.02 of the per-frame
    march at divisor 3 (`--no_view_cache`); frame GPU p99 within the Task 4 budget.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** Step 1 checks, `cloud_check.gd`, `cloud_vr_stereo_test.gd`, `smoke_test.gd`: PASS.
- [ ] **Step 5: Commit** `feat(game): near cloud shell and progressive re-anchoring of the view cache`

### Task 6: VR acceptance, budgets, docs, closing

**Files:**
- Modify: `game/world/weather/cloud_layer.gd` (final presets from the measurements)
- Docs: `docs/subsystems/weather-clouds.md`, `docs/subsystems/vr-performance.md`, `docs/history/weather-clouds.md`,
  `docs/known-issues.md`; archive `docs/superpowers/plans/2026-10-26-static-clouds.md`, its spec, this plan and spec to
  `docs/superpowers/archive/`
- Memory: `project-m8-weather-clouds.md`

- [ ] **Step 1: Desktop proxy** measurements (Task 1's views, cache on): fill ms, lookup ms, time to full resolution
  after a jump - into `vr-performance.md`.
- [ ] **Step 2: Headset run with the user** (`--perf-log`): pad, deck passage, above the decks, orbit; weather on
  45 Hz frame p99 <= 22.2 ms in normal phases, weather off 90 Hz; the user's look verdict (sharp, steady, no layer
  shift, deck passage acceptable). Fix rounds as needed.
- [ ] **Step 3: Docs + memory**, archive the plans / specs, final review.
- [ ] **Step 4: Commit** `docs: cloud view cache - VR acceptance, budgets, standing decisions`
