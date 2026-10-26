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

## Cloud light cache (static clouds Tasks 1-2, 2026-10-26)

The light toward the Sun is looked up in a light cache instead of marched per cloud sample (`weather-clouds.md`): 5
cascades x 128 x 128 x 512 voxels, R16F, 2 generations = 160 MB of VRAM (budget 192 MB [E]). Its fill is spread over
frames: at most 65 536 voxels per frame (96 + 32 light samples each); `cloud_check.gd` section 8 measures every
frame of a full refresh (640 frames, 7.1 s at 90 Hz; numbers measured at 98 304 voxels before the budget was lowered): median 0.099, worst 0.454 ms (RTX 4090, the check's slab). The
cost per voxel depends on the GPU's clock state and the weather, not the jobs: on the slab 262 144 voxels took 0.18 ms
at full clock and 0.37-0.51 ms clocked down (the earlier 0.45-0.80 ms spread at 327 680); in ERA5 weather (below)
196 608 voxels reached 0.45 ms per second's mean at full speed, 98 304 0.44 ms in the slow state. Task 1's
whole-cascade refill was 5-8 ms in one frame.

VR proxy (`terrain_perf.gd -- --utc=2025-10-26T15:00:00 --render_scale=1.65 --gpu_passes --weather=<abs>/data/
weather/cache/era5_1102911.wthr --cloud_quality=vr`, median of the per-second means, ms). The machine ran in two
states between runs: "fast" and "slow" (every pass ~2x slower in slow, e.g. transparent_and_post 0.88 vs 2.1 ms - the
system's state, not this change); compare within a state:

| Phase | cloud_pass read at the sample (round 2) fast / slow | hybrid cascades 1-4 (1 km / 12) fast / slow |
|---|---|---|
| descent | 1.15 / 4.71 | 2.51, 2.52 / 4.93, 5.04, 4.79 |
| pan low | 1.98 / 4.50 | 3.12, 3.10 / 7.44, 7.48, 7.40 |
| pan high | 1.48 / 3.60 | 2.23, 2.24 / 5.53, 5.39, 5.52 |
| cape | 0.12 / 0.27 | 0.10, 0.12 / 0.26, 0.24, 0.26 |

The hybrid near march adds +0.75-1.4 ms (fast) / +0.2-2.9 ms (slow) to the proxy's cloud pass, +45-65 % (VR ~2.6x
the proxy) - with the rest of the pass over the 2.5 ms cloud budget (open). Before the cache (Task 11, other weather):
1.4-3.2 ms at divisor 4. Not yet measured in the headset.

Fix round 1 (near lengths 1 / 3 / 2 / 2 km in cascades 1-4, 12 / 32 / 22 / 22 samples - case 8 needs them; the
clear-air early-out tried and dropped). Same command, back to back; the GPU state told by shadow_pass (~0.5 fast /
~0.9 ms slow). cloud_pass median ms, descent / pan low / pan high:

| Variant | fast | slow |
|---|---|---|
| round 3 lengths (1 km everywhere) | 2.24 / 3.62 / 2.67 | 5.06 / 7.51 / 5.50 |
| round 3 lengths + early-out | - | 5.13, 3.50 / 7.62, 7.71 / 5.68, 5.71 |
| new lengths | 2.25 / 5.04 / 3.60 | 5.14 / 10.66 / 7.68 |
| new lengths + early-out | 2.32 / 5.08 / 3.73 | 5.13 / 11.03 / 8.00 |

The longer near marches cost +35-42 % in the pans (descent unchanged); the early-out (2-4 texture fetches and two
lat / lon conversions per sample against the samples above a clear column top it skips) cost +1-4 % in both states
and was removed. Over the 2.5 ms budget in VR: Task 10's binding item (controller ruling 2026-10-26).
