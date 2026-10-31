# VR performance

Current state of the frame budget in VR: what is measured, how, and where the time goes. Plan:
`docs/superpowers/archive/plans/2026-10-26-vr-performance.md`.
Cloud-pass numbers: `weather-clouds.md` (Performance).

## Rig and budget

User's rig: Pimax Dream Air, RTX 4090, Pimax OpenXR runtime, 90 Hz, Pimax Play GPU Upscaling FSR 1.5 (user setting,
kept): OpenXR render target 3530 x 2834 per eye, 20 MP per frame (5294 x 4250 without upscaling). Budget 11.1 ms per
frame; a missed frame makes the runtime lock to 45 Hz (22.2 ms). Targets: main thread <= 5 ms per frame in flight,
GPU <= 9 ms with clouds at the VR pixel count. The 1440p desktop proxy of M8 Task 11 underestimated VR ~10x and does
not count for VR.

Frame-rate rule (user 2026-10-27, cloud view cache plan Task 1): with weather on the VR target is 45 Hz
(`XrManager.apply_frame_rate`: `Engine.max_fps` 45, the runtime reprojects the display frames between - the user's
headset verdict "alles bestens bei 45 fps"), with weather off 90 Hz. The per-frame cloud pass costs 41-45 ms in the
proxy and ~45 ms in VR (8-18 fps in the cloud zone, VR run 2026-10-27). Since 2026-10-30 the baked volume is the clouds' only
method (`weather-clouds.md` "Baked volume": stereo mock march 4.8-5.8 ms against the old march's 21-48 ms; the old
march's measurements in the history file). The cloud view cache and the world
volume tried before, their budgets and measurements: `docs/history/vr-performance.md`.

## Measuring

- `Start-VR.cmd --perf-log` (any start: `-- --perf-log`): `PerfLog` (`game/util/perf_log.gd`) writes
  `data/perf/perf_<date>_<time>.csv`, one row per second plus a "spike" row per frame over 22 ms; the Godot log
  names the XR render target size. Columns: frame interval p50 / p99 / max, root viewport GPU / CPU render time,
  `script_max_ms` (process_frame to the last node's _process), `main_max_ms` (process_frame to
  `RenderingServer.frame_pre_draw`: also deferred calls and the UI's `_draw`), then one column per
  `FrameProfiler.SECTIONS` entry (`game/util/frame_profiler.gd`: begin / end around a block, off without the flag).
- Desktop launch with the same columns: `godot --xr-mode off --path game --script res://tests/flight_check.gd --
  --no-xr --perf-log --weather=on|off --countdown=10 --seconds=150`. The main-thread work is the same as in VR
  (the desktop runs reproduce the VR script times), so main-thread work is measured on the desktop.
- `nvidia-smi --query-gpu=timestamp,utilization.gpu,clocks.gr,memory.used,power.draw,temperature.gpu,clocks_throttle_reasons.active --format=csv -l 1`
  alongside a VR run: GPU-bound runs sit at 100 % and the 450 W cap (reason 0x4).
- Time panel: "Bildrate: N fps (x ms)" (`TimePanel.frame_rate_line`).
- Per GPU pass at a fixed camera (static clouds Task 10): `render_check.gd -- <view> --cloud_quality=vr
  --render_scale=1.65 --fixed_exposure --gpu_time=120 --gpu_passes` waits for the baked volume to settle and prints the
  median / p99 per timestamp interval: the cloud pass's kernels `cloud_pass_march` / `_upsample` (their sum
  `cloud_pass`), the bake's dispatches `cloud_baked_bake` (`CloudEffect._stamp` splits the compute
  list only while timestamps are captured), `shadow_pass` (the cloud shadow maps), `opaque_and_sky`,
  `transparent_and_post`. `--cloud_divisor=<n>` overrides the preset's march resolution. `terrain_perf --gpu_passes`
  sums the `cloud_pass_*` kernels as `cloud_pass` too. Runs repeat to about +-1.5 ms at 40 ms (GPU clocks).

## Desktop VR-pixel proxy (Task 4)

`terrain_perf.gd -- --utc=2024-04-08T18:42:00 --render_scale=1.65 [--weather=<abs>/data/weather/cache/era5_1089330.wthr
--cloud_quality=vr]`: the root viewport renders 4223 x 2344 (bilinear `scaling_3d_scale`), the pixel count of the two
eyes. Calibration 2026-10-26 (median of the per-second GPU max, ms):

| Phase | without clouds | with clouds (VR preset 48 / 96, divisor 2) | clouds add |
|---|---|---|---|
| descent > 100 km | 13.3 | 30.4 | +17 |
| descent 20-100 km | 17.3 | 34.5 | +17 |
| descent 5-20 km (decks) | 19.3 | 40.8 | +21 |
| descent 1-5 km | 20.2 | 36.0 | +16 |
| descent < 1 km | 17.4 | 25.7 | +8 |
| pan low / pan high (ground) | 13.3 / 17.4 | 27.3 / 28.1 | +14 / +11 |

VR measured the clouds at +20-27 ms on the pad and +55 ms in the decks: the proxy shows ~40 % of the VR cloud cost
(one view instead of two with their own march, history and upsample; another day's weather). Correction for the
cloud task: VR cloud cost ~ 2.6 x the proxy's.

## VR measurements 2026-10-26 (FSR 1.5)

| Phase | GPU, weather on | GPU, weather off | Main-thread script, weather off | Frame, weather off |
|---|---|---|---|---|
| Pad before launch | 26-34 ms | 7 ms | 9.5 ms | 16-22 ms |
| Pad from ignition | 26-34 ms | 7.5 ms | 15-17 ms | 22-30 ms |
| Climb 1-10 km (decks 7-12 km) | 38-66 ms | 7 ms | 12.5-14.5 ms | 23-26 ms |
| 10-24 km | 18-30 ms | 6.7 ms | 12-13 ms | 22-23 ms |
| Space (start view) | 2.6 ms | 2.5 ms | 1.2 ms | 11.1 ms |

## Main-thread sections of a launch (desktop, weather off, 2026-10-26)

Per-second maxima of the per-frame totals, mean over the phase (ms). `main` = script + the UI's `_draw`.

| Section | Pad | 0-10 km | 10-60 km |
|---|---|---|---|
| main (all) | 28.9 | 24.6 | 21.2 |
| script (`_process`) | 20.2 | 14.6 | 12.8 |
| `ui_webcast_draw` (webcast gauges' `_draw`) | 8.0 | 9.7 | 8.2 |
| - of it `EngineGauge` | 5.2 | 6.4 | |
| - of it `RoundGauge`s | 2.0 | 2.9 | |
| - of it `TimelineArc` | 0.7 | 0.9 | |
| `flight_views` | 7.9 | 8.7 | 8.1 |
| - of it `fv_trajectory` (target-orbit ribbon) | 4.9 | 4.6 | 4.5 |
| - of it `fv_vehicles` (VehicleView.update_view) | 2.5 | 4.2 | 3.7 |
| `earth_terrain` (quadtree update) | 6.2 | 4.4 | 3.1 |
| `atmosphere` | 2.5 | 0.3 | 0.3 |
| `clouds` (weather on: 4.0 on the pad) | 2.1 | 0.1 | 0.1 |
| `ui_webcast` (`_process`) | 2.1 | 0.5 | 0.5 |
| `refraction` | 0.2 | 1.1 | 1.2 |
| everything else | < 0.7 each | | |

With weather on: `weather` 2.4 ms on the pad, 0.2-0.7 in flight; `sun` 0.6; the rest as above.

## After the main-thread cuts (Task 3, desktop launch, weather off)

| Section | Pad | 0-10 km | 10-60 km | > 60 km |
|---|---|---|---|---|
| main (all) | 18.8 (28.9) | 12.8 (24.6) | 11.0 (21.2) | 8.5 |
| `fv_vehicles` (plumes: left for the plume rework, user 2026-10-26) | 3.0 | 5.1 | 4.1 | 1.7 |
| `earth_terrain` (uploads 3.3 on the pad, traversal 1.5-2.3, free 0.5-1) | 5.8 | 3.9 | 2.8 | 2.2 |
| `ui_webcast_draw` (round gauges at 15 Hz, timeline arc every frame) | 2.4 | 2.5 | 2.4 | 2.7 |
| `fv_trajectory` (rebuilds: new target points, camera > 1 km) | 2.0 | 1.3 | 2.4 | 2.5 |
| `atmosphere` | 2.5 | 0.3 | 0.3 | 0.3 |
| `refraction` | 0.2 | 1.1 | 1.2 | 1.2 |

(before the cuts in brackets.) The desktop holds 60 Hz in every phase now. Done: the overlay no longer toggles every
gauge's visibility each frame (each toggle redrew it), engine gauge redraws on a firing / layout change only, round
gauges on a text change or a half-pixel arc move, texts and round gauges at 15 Hz (`UiRate`, user 2026-10-26), the
trajectory ribbon is rebuilt with new points or after a 1 km camera move and moved with the camera in between,
`RibbonMesh` fills preallocated arrays. Open: the plumes regenerate their cone meshes every frame (left for the plume
rework, user 2026-10-26); the terrain quadtree's GDScript traversal (~1 us per node) and the free pass over all nodes.

Candidate cuts as listed before Task 3: `EngineGauge` recomputes the O(n^2) dot radius (33 engines: 528 pairs) and redraws every
frame - compute on a layout change, redraw on a firing change; `RoundGauge` / `TimelineArc` redraw every frame -
redraw when the shown value changes by a displayed step; `TrajectoryView` rebuilds its ribbon mesh every frame from
points refreshed only every `REFRESH_S` - rebuild with the points, move it with the camera by its transform;
`VehicleView` and the terrain quadtree - profile inside before choosing.

## Clouds after the cleanup (baked volume only, 2026-10-30)

RTX 4090, `render_check --cloud_view=V1_era5 --fixed_exposure --gpu_time=300 --gpu_passes`, medians of two runs each:
desktop 1600 x 900 frame 2.92 ms, cloud pass 0.76 ms (march 0.70, upsample 0.05; p99 2.2-2.8 ms with re-bakes);
stereo mock (`--stereo=game/tests/data/pimax_dream_air_projections.json`, 3534 x 2838 per eye, VR preset) frame
6.89 ms, cloud pass 2.12 ms (p99 3.9-7.2 ms). The temporal pass is gone (it only copied the frame since the
deterministic march); the shadow maps' fill 0.02 ms. The GPU's clock state moves single runs a lot (the same
stereo case measured 18.2 ms frame / 6.2 ms cloud pass in one earlier run): compare runs interleaved. The old march's
measurements (light cache, outline pass, sub-samples, near / far hand-over, profile, cost study):
`docs/history/vr-performance.md`.

## Radiance cubemap and anisotropy (2026-10-31)

RTX 4090, `render_check --render_scale=2 --gpu_time=600`, means (medians) of 2-3 runs, ms. The sky's cube marked dirty
every frame (`--radiance_every_frame`, its largest duty cycle; the game's rule: every 0.5-8 s): pad 1.86 (1.70) -> 1.86
(1.73); Dallas in the 2024 eclipse (8 sky segments) 3.02 (2.82) -> 3.08 (2.85). So at most +0.03-0.06 ms; no spike in the
per-frame series (`--gpu_series`), REALTIME not needed. The p99 of 5-7 ms is in the static runs as well, not the cube.
Project-wide anisotropy 8x (level 3) against the former default 4x (level 2): Earth terrain 2.96 (2.74) vs 2.94 (2.73),
Moon terrain 1.24 (1.14) vs 1.22 (1.12): +0.02 ms, kept.
