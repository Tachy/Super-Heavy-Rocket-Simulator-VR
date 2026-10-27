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
frames: at most 98 304 voxels per frame (96 + 32 light samples each); `cloud_check.gd` section 8 measures every
frame of a full refresh (427 frames, 4.7 s at 90 Hz): median 0.099, worst 0.454 ms (RTX 4090, the check's slab). The
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

Static clouds Task 3 (deterministic march, fix round 1): VR preset 96 coarse / 192 max steps, fine 2.5 % (was 48 /
96 / 2.5 %), the entry bisection (4 field lookups per silhouette), the cloud-ahead lookup per clear coarse step, the
Catmull-Rom upsample; same command, one run, fast state (shadow_pass 0.40-0.48 ms). cloud_pass ms, median / p99 of the
per-second means (n = 60 / 20 / 20 / 20; p99 = the largest at these counts):

| Phase | before Task 3 (fix round 1 "new lengths", fast) median | Task 3 fix round 1 median / p99 |
|---|---|---|
| descent | 2.25 | 3.24 / 8.39 |
| pan low | 5.04 | 5.89 / 7.55 |
| pan high | 3.60 | 4.50 / 5.93 |
| cape | - | 0.32 / 0.34 |

+44 / +17 / +25 % over the medians before Task 3. Not tuned here: Task 10's binding 2.5 ms item.

## Cloud outline pass (static clouds Task 5b, 2026-10-26)

VR proxy as above, but with `era5_1102910.wthr` (`era5_1102911` is no longer in the cache), the same file before and after,
fast state (shadow_pass 0.41-0.42 ms in both runs). cloud_pass, median / max of the per-second means, ms:

| Phase | before (19bcac9) | outline pass (threshold 0.25) | threshold 0.5 (trial) |
|---|---|---|---|
| descent | 3.20 / 9.05 | 11.37 / 22.50 | 11.20 / 21.25 |
| pan low | 6.54 / 7.37 | 17.25 / 21.62 | 16.73 / 21.03 |
| pan high | 5.04 / 5.67 | 14.87 / 17.18 | 14.62 / 17.44 |
| cape | 0.31 / 0.34 | 0.32 / 0.55 | 0.32 / 0.43 |

That is +195-255 % in the cloud phases, far over the budget (open, Task 10). The edge threshold hardly matters: the cost
comes from the outline pixels of broken cumulus fields and from each one marching four rays in series in its lane.

Fix round 1 (the same run, `t5c_perf_step*.log`), each step on top of the previous one:

| Phase | before (19bcac9) | 1: coverage rays | 2: + early end, neighbour colour | 3: + centre ray | 4: + list |
|---|---|---|---|---|---|
| descent | 3.20 / 9.05 | 8.02 / 16.75 | 6.51 / 14.11 | 5.71 / 13.00 | 4.66 / 11.02 |
| pan low | 6.54 / 7.37 | 11.86 / 14.23 | 10.06 / 11.76 | 9.16 / 10.65 | 8.25 / 9.18 |
| pan high | 5.04 / 5.67 | 10.18 / 11.07 | 8.36 / 9.11 | 7.47 / 8.16 | 6.74 / 7.22 |
| cape | 0.31 / 0.34 | 0.32 / 0.46 | 0.32 / 0.44 | 0.32 / 0.43 | 0.33 / 0.37 |

Now +46 / +26 / +34 % over the medians before Task 5b: still over the 15 % mark (open, Task 10 / the controller:
approach 2, an edge-directed full-resolution alpha smoothing, costs nothing in the march). The remaining cost is the
coverage rays' own steps through broken cumulus (field and shape lookups), not their light.

## Cloud march sub-samples (static clouds Task 5c, 2026-10-27)

VR proxy as above (`terrain_perf --render_scale=1.65 --gpu_passes --cloud_quality=vr`), but the cache files predate the
snapshot format v4: the shipped analog of the run's month and slot (`data/weather/analog/era5_1102644.wthr`, October 12
UTC), the same for every column; base = 82e387d. cloud_pass, median / max of the per-second means, ms
(`.superpowers/sdd/2026-10-26-static-clouds/t5c_vrperf_*.log`):

| Phase | base | 8 x steps (768 / 1536) | sub 125 m, max 8, cell per sub | max 4, cell per sub | max 4, step's cell (shipped) |
|---|---|---|---|---|---|
| descent | 4.33 / 17.06 | 14.95 / 66.90 | 9.94 / 42.24 | 7.95 / 31.48 | 6.45 / 26.35 |
| pan low | 13.74 / 14.89 | 29.97 / 31.86 | 33.91 / 37.48 | 26.09 / 27.72 | 21.31 / 22.61 |
| pan high | 9.77 / 11.13 | 19.94 / 20.62 | 24.84 / 29.58 | 19.35 / 21.78 | 15.84 / 17.45 |
| cape | 0.33 / 0.39 | 0.34 / 0.47 | 0.33 / 0.70 | 0.33 / 0.61 | 0.34 / 0.54 |

The shipped variant costs +49 / +55 / +62 % (medians) over the base. The extra cost is the shapes' lookups at the
sub-samples (the light is per step and cached: a fine step of 5 % instead of 2.5 % with the sub-samples cost no less,
29.48 ms pan low); skipping the sub-samples behind T_sc < 0.1 saved nothing (the march ends at 0.01 and dense steps are
short anyway). Over the budget like the base (Task 10).

Fix round 1 (2026-10-27, base cb65707 - snapshot format v5, the analog regenerated; same run and weather,
`t5d_vrperf_*.log`): max steps 384, a field cell per sub-sample, the per-type quarter-texel structure step, the band
limit continuous. cloud_pass, median / max of the per-second means, ms:

| Phase | base cb65707 | at most 4 sub-samples | at most 8 (shipped) |
|---|---|---|---|
| descent | 6.41 / 26.68 | 8.88 / 44.10 | 12.64 / 67.09 |
| pan low | 21.67 / 22.88 | 27.52 / 31.09 | 38.02 / 42.82 |
| pan high | 16.25 / 17.71 | 20.28 / 23.81 | 28.03 / 33.23 |
| cape | 0.34 / 0.47 | 0.33 / 0.62 | 0.33 / 0.54 |

+97 / +75 / +72 % (medians) shipped, +39 / +27 / +25 % with 4 (user: quality first; both pass cloud_grain_check, 8
within 0.8 % of the 8 x reference's cover and alpha, 4 within 1.8 %).

## Near / far hand-over (static clouds Task 8, 2026-10-27)

Same VR proxy and weather as the Task 5c fix round (`t8_vrperf_*.log`, base 1cb02f6 = the table above, max 8
sub-samples). The march ends at 34 km; a second pass of the same steps (the far tier) runs from 20 km to the segment's
end with the far light path (no light cache there) and band-limited shapes; both run over the 20-34 km band.
cloud_pass, median / max of the per-second means, ms:

| Phase | base 1cb02f6 | separate far march (0.5 % steps, 384) | two passes (shipped) |
|---|---|---|---|
| descent | 12.64 / 67.09 | 10.89 / 78.54 | 16.55 / 85.84 |
| pan low | 38.02 / 42.82 | 42.28 / 46.21 | 51.17 / 57.99 |
| pan high | 28.03 / 33.23 | 28.48 / 30.31 | 36.12 / 40.99 |
| cape | 0.33 / 0.54 | 0.36 / 0.76 | 0.38 / 0.67 |

The shipped variant costs +31 / +35 / +29 % (medians) over the base: the band's overlap (both passes) and the far
pass's light path (5 samples per step instead of the cache's lookup). The cheaper separate far march lost far deck in
cloud_grain_check's deck view (alpha -12 % against the 8 x reference). Over the budget like the base (Task 10: knobs are
the band, the far pass's step budget and its light samples).

