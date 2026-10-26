# VR Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The game holds 90 Hz on the user's VR rig in a launch from the pad through the cloud decks to orbit.

**Architecture:** First measure where the main thread's ~12-17 ms per frame go (named sections recorded by the existing
`PerfLog`), then cut the largest sections on the main thread; separately make the cloud pass cheap enough for VR
(lower march resolution, fewer steps) against a desktop proxy that renders the VR pixel count. The user's VR runs with
`--perf-log` are the acceptance measurement; the 1440p desktop gate of M8 Task 11 no longer counts for VR.

**Tech Stack:** Godot 4.7 double (GDScript, compositor compute shaders), D3D12, OpenXR (Pimax runtime).

**Spec:** this plan's "Basis" section (the measurements of 2026-10-26 with the user; no separate design doc - the
user approved the direction in the session: main thread first, then the cloud pass, VR runs as the gate; no foveated
rendering).

## Basis (measurements 2026-10-26, user's rig)

Rig: Pimax Dream Air, RTX 4090, Pimax OpenXR, 90 Hz. Pimax Play GPU Upscaling FSR 1.5 (kept): OpenXR render target
3530 x 2834 per eye (20 MP per frame; without upscaling 5294 x 4250). Logs in `data/perf/` (git-ignored).

| Phase | GPU, weather on | GPU, weather off | Main-thread script, weather off | Frame, weather off |
|---|---|---|---|---|
| Pad before launch | 26-34 ms | 7 ms | 9.5 ms | 16-22 ms |
| Pad from ignition | 26-34 ms | 7.5 ms | 15-17 ms | 22-30 ms |
| Climb 1-10 km (decks 7-12 km) | 38-66 ms | 7 ms | 12.5-14.5 ms | 23-26 ms |
| 10-24 km | 18-30 ms | 6.7 ms | 12-13 ms | 22-23 ms |
| Space (start view) | 2.6 ms | 2.5 ms | 1.2 ms | 11.1 ms |

Render-thread CPU time is 1-1.5 ms throughout. When a frame misses 11.1 ms the runtime locks to 45 Hz (22.2 ms).
Two causes: (1) without clouds the main thread misses the frame; (2) the clouds add +20-27 ms GPU on the pad and
+55 ms in the decks at the VR pixel count (desktop 1440p measured +2.4 ms p99: the proxy underestimated VR ~10x).

## Global Constraints

- Budget at 90 Hz: 11.1 ms per frame. Targets: main-thread script <= 5 ms per frame in flight (headroom for the
  render thread, XR submit and spikes); GPU <= 9 ms with clouds at the VR pixel count.
- Physics never changes for performance: no change in `core/` behaviour, sim tick, or anything the sim reads. Only
  presentation work may be throttled, cached or skipped.
- No foveated rendering (user 2026-10-26). Pimax FSR 1.5 stays as the user's setting; nothing in the game depends on it.
- The approved cloud SHAPES are frozen (user, M8): VR may lose fine detail through resolution, not change the look.
- Desktop quality (`CloudLayer.Quality.DESKTOP`) is unchanged; all cloud changes are in the VR preset.
- Code, comments, commits English; UI German; Conventional Commits; commit + push to `main` after each green task.
- Tests: only the affected game scripts (`smoke_test.gd`, the touched `*_test.gd`, `cloud_vr_stereo_test.gd` for
  cloud changes); no blanket runs. Console output during a timed run stalls the main thread: print at the end only.
- Every `[E]` value gets its rationale in the comment; docs: `docs/subsystems/weather-clouds.md` (clouds) and a new
  `docs/subsystems/vr-performance.md` (main thread, measuring, the VR gate), indexed in `docs/subsystems.md`.

## Review Focus

- Time warp and paused clock: throttled presentation updates must still follow a clock jump at once (no stale HUD,
  trajectory or labels for a throttle interval after a jump / view switch) - test in the task that throttles.
- Stereo: any per-view cloud change must keep both eyes consistent (`cloud_vr_stereo_test.gd` stays green).
- Profiler off: with `--perf-log` absent the sections cost nothing measurable (no dictionary work per frame).
- Desktop path unchanged: the desktop preset images stay identical (render_check A/B before/after Task 5).
- Window resize / render-scale change mid-run: the cloud pass re-creates its images (divisor change path).

---

### Task 1: Section profiler in the perf log

**Files:**
- Create: `game/util/frame_profiler.gd`, `game/tests/frame_profiler_test.gd`
- Modify: `game/util/perf_log.gd`, `game/world/world.gd` (`update_world`, `_process`), `_process` of
  `game/ui/flight_hud.gd`, `flight_overlay.gd`, `nav_ball.gd`, `time_panel.gd`, `webcast/webcast_overlay.gd`,
  `game/xr/xr_manager.gd`

**Interfaces:**
- Produces: `class_name FrameProfiler` with static `enabled: bool` (set by `PerfLog._ready`), static
  `begin(section: StringName) -> void`, `end(section: StringName) -> void`, `take_frame() -> Dictionary`
  (StringName -> usec of this frame, then cleared). Both calls return at once when `enabled` is false.
- `PerfLog` CSV: one extra column per section in a fixed order `FrameProfiler.SECTIONS` (per-second max in ms),
  header `sec_<name>_max_ms`.

Sections (in `update_world` order): `flight_advance`, `weather`, `towers_zone`, `ephemeris` (the core body / frame
calls incl. the apparent positions), `camera`, `atmosphere`, `clouds`, `refraction`, `earth_terrain`, `water`,
`moon`, `shader_globals`, `sun`, `flight_views`, `moon_orbit`; UI: `ui_hud`, `ui_overlay`, `ui_navball`,
`ui_time_panel`, `ui_webcast`; `xr`.

- [ ] **Step 1: Write `frame_profiler_test.gd`** (SceneTree script like `toml_lite_test.gd`): with `enabled = false`
  begin/end leave `take_frame()` empty; with `enabled = true` a section timed around a ~2 ms busy loop reports
  1500-20000 usec, two begin/end pairs of the same section in one frame add up, `take_frame()` clears.
- [ ] **Step 2: Run it, expect FAIL** (`FrameProfiler` missing):
  `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/frame_profiler_test.gd`
- [ ] **Step 3: Implement `FrameProfiler`, wire the sections, extend `PerfLog`** (columns after `script_max_ms`).
- [ ] **Step 4: Run the test (PASS) and `smoke_test.gd` (PASSED)**; a 400-frame desktop run with `--perf-log`
  (`--xr-mode off --path game --quit-after 400 -- --no-xr --perf-log`) writes the section columns.
- [ ] **Step 5: Commit** `feat(game): named main-thread sections in the perf log`

### Task 2: Desktop main-thread measurement of a launch

**Files:**
- Modify: `game/tests/flight_check.gd` (pass-through only if needed: it must run non-headless with `--perf-log`)
- Create: `docs/subsystems/vr-performance.md` (the Basis table, the measuring procedure, this task's results);
  `docs/subsystems.md` index row

**Interfaces:** consumes Task 1's CSV columns.

- [ ] **Step 1: Run** `godot --xr-mode off --path game --script res://tests/flight_check.gd -- --no-xr --perf-log --countdown=10 --seconds=150`
  (window, vsync as in play) with weather off, then on; output to a file.
- [ ] **Step 2: Summarise per phase** (pad before ignition, pad after, 0-10 km, 10-60 km): `script_max_ms` and every
  section's mean / max; check the desktop script time is in the VR range (pad 9.5, flight 12-17 ms). If it is far
  lower, the VR-only work (`xr`, overlay panels) is the suspect: note it.
- [ ] **Step 3: Write the results into `vr-performance.md`** with the ranked sections and the candidate cut for each
  of the top sections (what work, per-frame or once, presentation-only or sim-facing).
- [ ] **Step 4: Gate - report the ranking to the user** before Task 3 (the cuts may touch visible behaviour: update
  rates of HUD, trajectory, labels).
- [ ] **Step 5: Commit** `docs: VR performance - main-thread sections of a launch`

### Task 3: Cut the main thread to <= 5 ms in flight

**Files:** decided by Task 2's ranking (re-plan the steps in the ledger before starting; keep this task's tests).
Expected candidates, in the order to try:
1. Work repeated per frame with the same inputs (core calls for the same epoch, rebuilt arrays / meshes whose
   inputs did not change: trajectory line, trail, orbit line, terrain node lists) - cache by input.
2. Presentation at a lower rate where the eye cannot tell: HUD / overlay text and gauges at 15 Hz, trajectory and
   moon orbit line rebuilt at <= 4 Hz or when the clock jumps; never vehicle / camera / terrain placement.
3. GDScript hot loops moved into the existing `bindings` adapter only when they are pure data shuffling (no physics
   in the adapter).

**Interfaces:** whatever is throttled exposes `invalidate()` and is called from `World.set_focus`, the clock's
`time_jumped` and view switches (Review Focus 1).

- [ ] **Step 1: For each cut, write the test first** in the owning `*_test.gd` (or `smoke_test.gd`): the throttled
  output equals the unthrottled one after `invalidate()` / a clock jump in the same frame; cached values change when
  their input changes.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement the cut.**
- [ ] **Step 4: Run the tests (PASS), `smoke_test.gd` (PASSED); re-run Task 2's desktop launch** and record the
  section times before / after in `vr-performance.md`. Done when `script_max_ms` p50 <= 5 ms in flight phases.
- [ ] **Step 5: Commit per cut** `perf(game): ...`

### Task 4: VR pixel-count proxy in terrain_perf

**Files:**
- Modify: `game/tests/terrain_perf.gd` (option `--render_scale=<f>`: the root viewport's `scaling_3d_scale` with
  `SCALING_3D_MODE_BILINEAR`, applied before the run)
- Modify: `docs/subsystems/vr-performance.md` (proxy definition and its calibration)

**Interfaces:** produces the command the cloud task measures with:
`terrain_perf.gd -- --render_scale=1.65 --weather=<abs .wthr> --cloud_quality=vr` (2560 x 1440 x 1.65^2 = 20.1 MP,
the two 3530 x 2834 eyes).

- [ ] **Step 1: Implement the option**; the printed header names the internal size.
- [ ] **Step 2: Calibrate** - run with and without `--weather` at `--render_scale=1.65`; the cloud difference in the
  descent phase should be of the VR order (+20 ms or more at decks seen from above). Record both; if it is far below,
  note the gap (stereo per-view fixed cost) and use the ratio as the correction for Task 5.
- [ ] **Step 3: Commit** `test(game): terrain_perf renders the VR pixel count (--render_scale)`

### Task 5: VR cloud preset within budget

**Files:**
- Modify: `game/world/weather/cloud_layer.gd` (`PRESETS[Quality.VR]`), `docs/subsystems/weather-clouds.md`
  (Performance section: VR numbers replace the 1440p gate for VR)

**Interfaces:** consumes Task 4's command. `CloudEffect.set_quality(divisor, temporal_weight)` already re-creates the
images on a divisor change; the upsample kernel takes the four nearest low-resolution pixels for any divisor.

- [ ] **Step 1: Measure the levers one at a time** at the proxy (descent p50 / p99 with clouds minus without):
  `divisor` 2 -> 3 -> 4; then `steps` / `max_steps` 48/96 -> 32/64; `light_near` 5 -> 4. Record each.
- [ ] **Step 2: Pick the cheapest combination that meets GPU <= 9 ms p99 with clouds at the proxy** (scaled by Task
  4's correction); prefer divisor over steps (steps change the shapes' sampling, the divisor only their sharpness).
- [ ] **Step 3: Image check** - `render_check.gd --cloud_quality=vr --fixed_exposure --deterministic=64` at
  1600 x 900, the four views of M8 Task 11, before / after: no banding, no edge shimmer between frames, mean
  difference reported; send the PNGs to the user.
- [ ] **Step 4: Run `cloud_layer_test.gd`, `cloud_vr_stereo_test.gd`, `smoke_test.gd`** (PASS); desktop preset image
  unchanged (render_check desktop A/B: identical).
- [ ] **Step 5: Commit** `perf(game): VR cloud preset within the VR pixel budget`

### Task 6: VR acceptance run with the user

**Files:** `docs/subsystems/vr-performance.md`, `docs/subsystems/weather-clouds.md` (results), memory.

- [ ] **Step 1: The user flies** `Start-VR.cmd --perf-log` (FSR 1.5 as in the Basis), weather on, pad -> decks ->
  > 60 km; `nvidia-smi` sampled alongside.
- [ ] **Step 2: Evaluate per phase**: target 90 Hz (frame p50 11.1 ms) in every phase, GPU <= 9 ms, script <= 5 ms;
  report the table against the Basis.
- [ ] **Step 3: If a phase misses**: name the bound (GPU / script / other), add a follow-up task to this plan's ledger
  with the user's decision.
- [ ] **Step 4: Commit** `docs: VR performance - acceptance run`
