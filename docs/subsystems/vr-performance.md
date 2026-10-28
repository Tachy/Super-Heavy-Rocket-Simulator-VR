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
proxy and ~45 ms in VR (8-18 fps in the cloud zone, VR run 2026-10-27): it is replaced by the world-fixed cloud view
cache (plan `docs/superpowers/plans/2026-10-27-cloud-view-cache.md`), whose fill gets the budget below.

## Cloud view cache budget (cloud view cache plan Task 1, 2026-10-27)

`--no_cloud_pass` (user argument; render_check passes it through) skips the near cloud pass (`CloudLayer.pass_enabled`;
its shadow maps and light cache with it). Desktop proxy (`render_check --render_scale=1.65 --gpu_time=120 --gpu_passes
--cloud_quality=vr --fixed_exposure`, Starbase 2024-06-15T21:00 UTC, analog weather; scripts
`.superpowers/sdd/2026-10-27-cloud-view-cache/t1_prof.sh`), GPU frame median / p99:

| View | with the cloud pass | without | weather off |
|---|---|---|---|
| ground (az 90, el 25) | 44.9 / 47.2 ms (cloud pass 41.3: march 32.6, refine 8.5; shadow pass 1.5) | 1.6 / 3.2 ms | 1.6 / 2.7 ms |
| 9 km deck (el -10) | 49.8 / 56.0 ms (cloud pass 45.0: march 35.4, refine 9.6) | 2.5 / 6.3 ms | - |

Headset (Pimax Dream Air, 3534 x 2838 per eye, pad, weather on, 45 Hz, `data/perf/perf_20261027_2234*.csv` /
`_223635.csv`): with the cloud pass GPU 45.3 ms median (15-33 fps); without it GPU 7.4 ms median, 8.2 ms p90, 8.8 ms
max, the frame interval a steady 22.22 ms. **Cache budget at 45 Hz: 22.2 - 9 (scene, max) - 2 (margin [E]) ~ 11 ms
per frame** (the preset `cache_budget_ms`, Task 4). The per-frame pass's rate (1.3 M rays of the 1377 x 947 centre in
~45 ms, refine included) puts a full-resolution fill of the visible region (~9 x the rays) at ~0.4 s of GPU, i.e.
~1 s at 11 ms per frame (measured with the cache, Task 4 fix round 1: level 0 of the view in ~12 frames, below).

## Cloud view cache in the frame (cloud view cache Task 4, 2026-10-28)

The frame shows the world-fixed cache (fill under the budget, per-eye lookup; `docs/subsystems/weather-clouds.md`).
Desktop VR-pixel proxy, the same runs as Task 1 with the cache on (`t1_prof.sh cache <view>`; the VR preset: budget
cap 11.4 ms; face 3200 texels at the proxy's density), once the cache has nothing left to fill:

| View | GPU frame median / p99 | cloud pass (the lookup) | before (per-frame pass) |
|---|---|---|---|
| ground (az 90, el 25) | 2.56 / 4.99 ms | 0.10 / 0.52 ms | 44.9 / 47.2 ms |
| 9 km deck (el -10) | 3.63 / 8.27 ms | 0.11 / 0.46 ms | 49.8 / 56.0 ms |

Fix round 1 (2026-10-28; the plan on a worker, the covering march, the latency cost model; `t1_prof.sh f1final
<view>`, logs `.superpowers/sdd/2026-10-27-cloud-view-cache/t1_prof_<view>_f1final.log`). VR proxy (budget 11.4 ms,
face 3200): level 0 over the view after 0.58-0.63 s (12 frames; the frames are slow while the march covers), the
covering march ended after 1.21-1.22 s (23-24 frames), the full level over the view after 4.5-4.7 s (217-228 frames),
the whole cube after 27.2 s (~1580 frames); fill median 2.0 / 2.8 ms, p99 7.1 / 8.2 ms, max 8.0 / 9.3 ms (ground /
deck; budget + 2 ms never exceeded - the largest excess -3.4 / -2.1 ms); converged GPU frame 2.56 / 3.64 ms median.
Level 0 is bound by the fill cap (`MAX_FILL_GROUPS` 1024 work groups = 256 level-0 tiles, ~6 ms of GPU a frame) after
four frames under the unknown-latency prior (~64 tiles each). Main thread (FrameProfiler `cloud_view_cache`, every
frame incl. the restart): median 0.04 ms, p99 0.07 ms, max 0.09 / 0.21 ms (the plan runs on a worker). Desktop preset
(1600 x 900, budget 9 ms, face 1984; `t4_rc.sh f2t <view>`): deck level 0 after 0.45 s (8 frames), covering ended 0.97
s, full level 2.3 s, cube 14.6 s; fill median 3.1 / 3.5 ms, p99 8.9 / 10.0 ms, max 9.8 / 11.4 ms (ground / deck; one
full-level deck tile alone takes up to ~8.4 ms, a fill's time is its slowest ray's - `weather-clouds.md`, Budget);
main thread p99 0.06 / 0.07 ms, max 0.08 / 0.14 ms.

## Near shell and re-anchor (cloud view cache Task 5, 2026-10-28)

`render_check --frames_every=0.5 --frames_for=80` (Starbase ascent, `--autopilot --fly=25`, slab 1.5-2.5 km cover
0.6, VR preset, render scale 1.65 - the desktop VR proxy; 3600 flight frames of 1/45 s each): from on board (the near
shell in 3529 of them: the deck within R_near of the pad already) GPU frame median 8.7 ms, p99 27.5 ms, max 32.4 ms -
**the 22.2 ms target is missed in the deck passage** (near shell march + covering after evictions + fills); watched
from the ground 2 km east median 5.3 ms, p99 18.8 ms, max 24.1 ms. The near shell's march is the frame's steady cost
(not excluded from the adaptive budget's rest of the frame; only the covering march is). Logs
`.superpowers/sdd/2026-10-27-cloud-view-cache/t5_final_fly_{board,site}.log`.

Fix round 1 (the near shell predicted in the budget, covering by the near shell's kernel, eviction by a tile's own
parallax, `--frames_for=120`, the first frame after each sample's reference not counted): on board GPU median 12.3 /
p99 24.4 / max 30.1 ms - still over 22.2 ms: the rest of the frame p95 ~14 ms (median 8; `opaque_and_sky` 6-11 ms)
plus the near shell inside the deck 8-15 ms (`cloud_pass_march`) leave the fill its 2 ms floor; watched from the
ground median 6.9 / p99 15.2 / max 19.4 ms. Logs `r1_fly_{board,site}_final2.log`.

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
  --render_scale=1.65 --fixed_exposure --gpu_time=120 --gpu_passes` waits for the light cache to settle and prints the
  median / p99 per timestamp interval: `cloud_light_fill`, the cloud pass's kernels `cloud_pass_march` /
  `_outline` / `_refine` / `_temporal` / `_upsample` (their sum `cloud_pass`; `CloudEffect._stamp` splits the compute
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
and was removed. Over the 2.5 ms budget in VR (accepted by the user 2026-10-27, see Rig and budget).

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

+44 / +17 / +25 % over the medians before Task 3. Not tuned here (the budget is accepted as exceeded, see Rig and budget).

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

## The Sun through cloud (static clouds Task 9 fix round 1, 2026-10-27)

VR pixel proxy (`render_check --sun_through_deck=5 --cloud_quality=vr --render_scale=1.65 --resolution 2560x1440
--fov=90 --gpu_time=240`, 30N 95W, Sun 67 deg up, RTX 4090): the whole frame's GPU time looking at the Sun through the
deck 14.492 ms median (p99 19.58), the same deck with the Sun behind (`--away_from_sun`) 14.490 ms (p99 19.51): the
march toward the Sun past the scaled criterion (rays within 0.014 rad, transmittance only) costs nothing measurable.
The composite's second quad pass (the scene x T, then + the colour) is in both numbers (not measured apart).

## Cloud shadows on the air (static clouds Task 9b, 2026-10-27)

`render_check --cloud_quality=vr --fixed_exposure --render_scale=1.65 --gpu_time=120` (whole frame, two rounds each),
air shadow on / off (`--no_air_shadow`): backlit cumulus (cf 0.5 slab, Sun 15 deg) median 14.85 / 15.09 vs 13.22 /
13.34 ms (+1.6-1.7 ms; p99 17.1 / 20.6 vs 18.1 / 18.2); the 9 km deck over Starbase (analog weather) 41.21 / 41.27 vs
36.98 / 36.79 ms (+4.2-4.5 ms; p99 52.6 / 54.6 vs 46.7 / 48.7). Per march pixel 33 boundary lookups (transmittance +
two single-scattering tables) and 96 shadow-map lookups. Task 10 owns the budget (candidates: fewer segments over
long horizontal rays, a quarter-resolution pass of its own, skipping rays whose shell has no cloud in the Sun's way).
Fix round 1 (the ray clipped to the highest cloud top of the loaded weather, `cloud_air_shadow_top_m`, instead of
20 km; the same runs, `t9b_r1_cost_before/after.log`): backlit cumulus (tops 2.1 km) on 14.86 / 15.17 -> 14.21 /
14.26 ms against off 13.25 / 13.23 -> 14.18 (noisy) / 13.19 ms: +1.6-1.9 -> +1.0-1.1 ms; the 9 km deck (analog
weather, cloud up to the upper layers) on 40.94 / 41.02 -> 40.77 / 40.70 ms against off ~36.8 ms: +4.0 ms, no gain.

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

Fix round 1 (the lateral far fade 1.5-3 km, the column model's sub-grid cover - 32 or 64 shape reads per column pixel,
once - and the B-spline column, the mixed-depth upsample; `t8_vrperf_t8f_fix1.log`), same run: descent 16.73 / 85.50,
pan low 51.67 / 58.45, pan high 36.15 / 40.76, cape 0.45 / 0.73 - within the run-to-run spread of the two-pass numbers
above (the proxy's phases see the column model only at their farthest pixels). Orbit views, where every pixel beyond
~1 km footprints is the column model, were not measured (Task 10).

## Cloud pass profile (static clouds Task 10, 2026-10-27)

VR pixel proxy per pass (`render_check --cloud_quality=vr --render_scale=1.65 --fixed_exposure --gpu_time=120
--gpu_passes`, light cache settled, daylight; sdd `t10_prof.sh`, logs `t10_prof_<view>_<tag>.log`). Views: ground
(Starbase, analog weather 2024-06-15 21 UTC, az 90 el 25), 3 km (el -5), 9 km deck (el -10), 20 km (el -15), 400 km
(nadir, cf 0.5 slab 1-2 km), backlit cumulus (cf 0.5 slab, Sun 15 deg, at the Sun), the Sun through a tau-5 deck.
Without weather the ground frame is 0.99 ms. ms, median of 120 frames: base 058318e / after the clear-air detail
skip (`weather-clouds.md` "Steep edges"; images bit-identical in all seven views) / after the follow-up (below):

| View | frame | cloud pass | march kernel | outline refine | shadow maps |
|---|---|---|---|---|---|
| ground | 46.2 / 42.6 / 38.8 | 44.6 / 40.9 / 36.9 | 34.0 / 30.9 / 28.4 | 10.4 / 10.0 / 8.4 | 0.59 |
| 3 km | 41.8 / 38.0 / 32.6 | 38.9 / 35.2 / 29.6 | 28.7 / 25.5 / 21.8 | 9.9 / 9.7 / 7.7 | 0.59 |
| 9 km deck | 52.9 / 51.8 / 48.2 | 49.9 / 48.9 / 45.3 | 38.0 / 37.6 / 35.8 | 11.7 / 11.1 / 9.5 | 0.59 |
| 20 km | 34.2 / 33.8 / 28.8 | 31.4 / 30.6 / 25.8 | 22.1 / 21.9 / 18.7 | 9.0 / 8.6 / 6.9 | 0.60 |
| 400 km | 8.5 / 8.5 / 8.6 | 6.1 / 6.2 / 6.1 | 4.0 / 3.9 / 3.8 | 2.0 / 2.1 / 2.1 | 0.60 |
| backlit Cu | 16.4 / 16.1 / 14.5 | 15.0 / 14.7 / 13.0 | 10.7 / 10.2 / 8.9 | 4.2 / 4.4 / 3.9 | 0.59 |
| Sun through deck | 8.4 / 8.2 / 8.2 | 7.3 / 7.1 / 7.1 | 7.2 / 7.0 / 7.0 | 0 | 0.57 |

(third value: after the Task 10 follow-up, 6 sub-samples and step-weight limit 0.3, 954a4c8 - the step weight was taken
back to 0.15 because 0.3 failed cloud_far_check's hand-over fade cases; the ground / deck rows show the 6 sub-samples
and step weight 0.15 state, sdd `t10_prof_{ground,deck}_w015.log`; the other rows' third values were measured at 0.3
and are about 4-5 ms (cloud pass) too low where the view is in dense cloud; the look is the before's within 0.1-0.4 of 255 on average, crops
`t10b_substeps_*`.)

The outline pass's list kernel, the temporal copy and the upsample are 0.01-0.12 ms; the light cache fill is 0 once
settled. Run-to-run spread about +-1.5 ms at 40-50 ms.

Where the milliseconds are (ablations on the base: one part switched off or changed, temporary defines; ground / deck
cloud pass, ms saved):
- the march's sub-samples (up to 8 per step, each a field lookup and the shapes): 1 sub-sample instead of 8 saves
  23 / 29 ms - the largest item; of it the field lookups per sub-sample (the step's cell reused) 8 / 10 ms;
- the far tier (pass 1 of the hand-over, march and outline sub-rays): 13 / 21 ms; its 20-34 km band (marched by both
  passes) 4.6 / 3.9, its light path (5 samples) 2.0 / 4.1;
- the hybrid cascades' near light march (12-32 samples per lit step): 12.6 / 11.2 ms; the whole light 13.4 / 12.7;
  the per-sample fallback march 0;
- the outline refinement (3 coverage-only sub-rays per outline pixel): 10.4 / 11.7 ms;
- the air shadow (32 segments x 3 shadow-map samples + 33 single-scattering lookups): 5.3 / 2.3 ms;
- the step-weight limit (CLOUD_MAX_STEP_WEIGHT) 0.15 -> 0.3: 4.7 / 5.2 ms (fewer light samples in dense cloud) - not
  kept, see below.

Tried and not taken (all measured on the ground / deck views):
- Clear-air skip of the detail's three lookups (shapes first; exact): -3.7 ms ground, -1.0 deck (noise level) -
  shipped.
- Empty-space skip of whole steps from conservative bounds (the field texels' largest fraction and ice share, the
  lowest threshold, the shape's widened 4- / 8-texel block maxima from the core): bit-identical but +9 / +13 ms - the
  block maxima of the shape noise lie above the threshold almost everywhere (its 500 m octaves), so the bound costs
  without skipping. Patch kept (sdd `t10_skip2_*.patch`).
- Caching the four LCL base texels per ray (exact): +8-13 ms (the state held across the march loop costs occupancy).
- Air shadow: one lookup per segment instead of 3, 16 segments, lazy single-scattering lookups only for shadowed
  segments (exact): -2 / 0, -0.5 / -0.4, -0.2 / +1.5 ms: noise level.
- Far light 3 samples instead of 5: -1.5 / -0.6 ms; the near march capped at 22 samples: ~0; the pass loop unrolled
  (far / near specialised): ~0.

Look-trade options after the user's rulings (2026-10-27; crops `screenshots/static_clouds/t10_opt_*.png`, applied ones
`t10b_substeps_{ground,deck,cape}.png`: before | after):
- at most 6 sub-samples instead of 8: APPLIED, -5.4 / -4.2 ms at the ground / the deck. 4 (-9.3 / -8.9 ms) was the
  ruling but fails the Cape's alpha grain check (1.375 x the reference's, bound 1.25; 5: 1.241); no visible loss in the
  crops (mean difference 0.1-0.4 of 255).
- step-weight limit 0.3 instead of 0.15: REVERTED. It was allowed only if every reference case passed within its
  tolerance; cloud_far_check's hand-over fade cases (scattered cf 0.12 nadir 1.1085, broken cf 0.4 low Sun 45 deg
  1.1024 against 10 %) failed with it (the shapes' radiance dropped), 1.0936 / 1.0849 with 0.15. -4.7 / -5.2 ms lost.
- march divisor 4 instead of 3 (-11.3 / -14.3 ms): REJECTED by the user (visibly softer near cumulus, blockier lower
  deck outlines); divisor 3 stays.
The cloud pass is still 37 ms at the ground and 45 ms at the deck in the proxy - the budget (cloud p99 2.5 ms x the
stereo correction) is out of reach of optimisation without a different march design.

Shape de-tiling (Task 9d, re-measured on 6b9149e with the parked `wip/t9d-detile` commit and its variants; cloud pass,
ground / 400 km; tiled 40.9 / 6.2 ms): blend in a 0.1 band (c7fe2ca) 97.8 / 11.9 (+139 / +91 %), blend in a 0.05 band
(V4) 72.9 / 10.2 (+78 / +64 %), smooth warp A = 0.33 (V5) 47.3 / 7.3 (+16 / +18 %; shapes stretched locally, Task 9d
report). The detail skip does not shorten the per-lookup cost the de-tiling adds, so the ratios stay those of Task 9d;
the user decided 2026-10-27: not done, the lattice stays.

## Where the march's time goes (cloud world volume plan Task 0, 2026-10-28)

Today's per-frame march (`--no_view_cache`, the reference path of the world-volume plan), measured with the counters and
ablation switches of `weather-clouds.md` (Diagnostics). VR pixel proxy: `--cloud_quality=vr --render_scale=1.65
--gpu_time=120 --gpu_passes --fixed_exposure`, Starbase analog 2024-06-15 21 UTC, views ground (az 90, el 25) and deck
(9 km, el -10); sdd `t0_prof.sh` / `t0_prof_all.sh`, logs `t0_prof_<view>_<tag>.log`, summary `t0_prof_summary.txt`.

Shader clock: none. Grepping `servers/rendering/rendering_device*` and the D3D12 / Vulkan drivers finds no
`VK_KHR_shader_clock` / `shaderClock` feature (only glslang's compiler knows the extension), and the game runs on
D3D12 - every time split below comes from ablations.

Time saved, cloud pass ms (median of 120 frames; base 41.4 / 46.9 ms ground / deck; run-to-run spread about +-1.5 ms).
Each ablation changes the image (a fetch class replaced by a constant also changes the steps taken), so these are
upper bounds of what the class costs in isolation:

| Ablation | What goes | Ground | Deck |
|---|---|---|---|
| `density` | view samples' shape, detail, tables -> the cell's grid mean | 25.2 | 9.8 (outline pass emptied: -9.5 of it) |
| `light` | a sample's light-source evaluation -> constant | 12.7 | 16.1 |
| `air` | atmosphere tables at the clouds -> constants | 0.0 | 0.4 |
| `field` | flat-base / ice / stratiform fetches of the field | 10.3 | 13.2 |
| `shape` | shape + detail everywhere (light march too) | 29.7 | 22.7 |
| `detail` | the three detail levels | 12.2 | 16.5 |
| `subsamples` | 1 sample per step instead of up to 6 | 17.6 | 21.9 |
| `outline` | the outline pass | 8.6 | 9.5 |
| `air_shadow` | the air shadow loss | 2.9 | 3.5 |
| `far_tier` | the far pass | 9.8 | 17.0 |

The counters themselves cost 5.6 / 6.0 ms of the cloud pass (the numbers below are from those runs).

Per march pixel (`--counters`, march image at the proxy's resolution; median / p99 / max; sum over the image):

| | ground | deck |
|---|---|---|
| density_fetches | 6 646 / 13 270 / 20 055; 2.84e9 | 8 695 / 17 209 / 26 411; 4.09e9 |
| light_fetches | 1 217 / 3 905 / 5 655; 5.93e8 | 677 / 4 644 / 6 196; 5.58e8 |
| occupied_steps | 12 / 42 / 100; 4.77e6 | 13 / 47 / 124; 5.69e6 |
| empty_steps | 184 / 278 / 285; 8.40e7 | 267 / 303 / 325; 1.02e8 |

Derived: density fetches per step (occupied + empty) 32 / 38; light fetches per occupied step 124 / 98 (the light is
evaluated on occupied steps only). Cost per occupied step, the number Spike S compares against: march kernel ms /
occupied steps summed over the image = 32.7 ms / 4.77e6 = 6.9 ns (ground), 37.2 ms / 5.69e6 = 6.5 ns (deck); the whole
cloud pass (outline refine included) 8.7 / 8.2 ns. The empty steps are 95 % / 95 % of all steps and about 18 times more
numerous; their field-cell fetches (about 13 per empty step in a cloud-free slab run) are most of the density fetches.

## Spike S: far tier, baked vs procedural density (cloud world volume plan Task 2, 2026-10-28)

The far tier alone (`CloudSpike`, `weather-clouds.md` Diagnostics): rays [2 km, end] from the cyclops point over the
union of both eye frusta (stereo harness, provisional Pimax projections, 0.5x: centre image 2196 x 1420 = 3.12 Mpx;
1.0x: 4392 x 2839), perfect occupancy (view-exact taps, 200 m cells, Chebyshev distance field), constant light, no air.
(a) per-step `cloud_extinction_at`; (b) one fetch of a 100 m bake + one dummy RGBA16F light fetch. Gate views V1 (pad,
slab 1-2.5 km cf 0.5) and V3 (9 km above slab 1.5-2.5 km), `--handover=0,0 --gpu_time=120 --gpu_passes`; sdd logs
`t2_<view>_<spike>_<tag>.log`, summary `t2_summary.txt`. March kernel ms, median / p99 (`c`: without counters):

| | V3 (a) | V3 (b) | V1 (a) | V1 (b) | V3 (b) 1.0x |
|---|---|---|---|---|---|
| march, no counters | 23.5 / 41.1 | **12.4 / 25.3** | 34.7 / 52.4 | 17.7 / 28.8 | 72.7 / 91.3 |
| march, counters | 24.9 / 44.3 | 13.1 / 26.6 | 39.8 / 61.2 | 19.0 / 34.9 | 48.7 / 102.6 |
| occupied steps / px median, p90, p99; mean | 124, 587, 1581; 219 | 121, 591, 1425; 215 | 227, 917, 2343; 358 | 228, 927, 2116; 356 | 122, 620, 1512; 227 |
| of them Delta-tau limited / with extinction > 0 | 97 % / 16 % | 97 % / 40 % | 72 % / 7 % | 72 % / 21 % | 94 % / 38 % |
| empty (sphere-traced) steps / px median | 1 | 1 | 0 | 0 | 1 |
| ps per occupied step (counter run) | 36.4 | 19.5 | 35.7 | 17.1 | 17.2 |
| density fetch ps / step (ablation `density`) | 27.6 | 9.4 | 27.7 | 7.7 | |
| light fetch ps / step (ablation `light`) | - | 2.1 | - | 2.4 | |

Today's march over the same far part (`--spike=ref`): V1 21.5 / 38.4 ms. Sanity (mean alpha of the centre image):
V3 ref 0.4655, (a) 0.4710, (b) 0.4763; V1 ref 0.3513, (a) 0.3558, (b) 0.3595; luminance (b) vs (a) +0.1 % / -0.5 %.
The 1.0x run without counters measured slower than with counters (72.7 vs 48.7 ms): run-to-run spread at that load
(both p99 > 90 ms); the per-step figure is from the counter run. Ablation `density` replaces the fetch by half the
occupancy's max extinction (no fetch): the rays end sooner (2.4e8 vs 6.7e8 steps in V3), so its split is per step.

Rules (spec G): (b) <= 5 ms in V3: no (12.4 ms). (b) >= 1.5x cheaper per occupied step than (a): yes (1.87x V3, 2.09x
V1). > 100 occupied steps per pixel in V3 despite perfect occupancy: yes (median 121, mean 215). (b) > 10 ms: yes - stop
and report. The cost is the step count: inside occupied cells the step is 0.3 / (the cell's dilated max extinction) -
real cumulus has ~0.05-0.15 /m, so 2-6 m steps also through the cells' clear parts (V3 (b): 60 % of the occupied steps
sample no extinction), down to the scaled termination (tau ~18 at k = 1 - g^2). At (b)'s 19.5 ps a step, 5 ms in V3 is
~82 occupied steps per pixel on average (2.6x fewer).

### Spike S round 2: the step rule (Task 2b, 2026-10-28)

Same spike, variant (b) only, now with the recorded Pimax projections (centre image 2065 x 1420 = 2.93 Mpx at 0.5x;
the V3 region grew to 503 x 788 km). Levers (`render_check --spike_levers=1,2,4,s,t`, `--spike_stop=<T>`), on a
second volume over the bake grid (R16F, 100 m: the max of the 3^3 bake texels a trilinear fetch in the cell reads -
the exact sup of the interpolated field - or, for a clear cell, minus its Chebyshev distance to the nearest non-clear
cell, cap 8): **1** Delta tau 0.3 from that max, step ends at the bake cell's exit, clear bake cells stepped to their
exit; **2** sphere tracing on its distance; **3** termination at scaled T 0.03 instead of 0.01; **4** one step per
bake cell (to its exit, no Delta tau term: Beer-Lambert is exact for a constant-extinction segment, and with a constant
source the in-scatter S T (1 - e^{-k tau}) / k is exact for any profile, so only tau matters), **t** with 4 the
trilinear bake at the step's middle instead of the cell's mean (midpoint rule on a field linear between texel centres),
**s** with 4 the structure term kept (production adds sub-cell detail where w_k > 0). sdd logs `t2b_*.log`, summary
`t2b_summary.txt`; one run at a time. ms = march kernel median / p99; steps per pixel median / p90 / p99 / mean.

| V3 (b) | ms (counters) | ms (none) | occupied steps | empty | mean alpha | lum vs base |
|---|---|---|---|---|---|---|
| base (round 1 rule) | 22.7 / 28.4 | 18.8 / 25.4 | 137 / 610 / 1503 / 228 | 1 / 6 / 18 | 0.4683 | - |
| 1 | 11.8 / 17.9 | | 10 / 233 / 560 / 95 | 1 / 33 / 73 | 0.4684 | +0.0 % |
| 2 | 29.0 / 35.9 | | 116 / 469 / 1287 / 180 | 1 / 10 / 23 | 0.4683 | +0.0 % |
| 1,2 | 12.2 / 18.2 | 9.6 / 16.9 | 10 / 233 / 560 / 95 | 1 / 17 / 44 | 0.4684 | +0.0 % |
| 3 (T_sc 0.03) | 22.3 / 27.4 | | 121 / 588 / 1443 / 216 | 1 / 6 / 18 | 0.4683 | -1.9 % |
| 4 | 1.17 / 3.01 | | 3 / 16 / 41 / 6.5 | 1 / 33 / 73 | 0.4624 | -0.5 % |
| 1,2,4 | 1.07 / 2.63 | 1.03 / 2.64 | 3 / 16 / 41 / 6.5 | 1 / 17 / 45 | 0.4624 | -0.5 % |
| 1,2,4,t | 1.03 / 2.72 | 0.97 / 2.20 | 3 / 16 / 38 / 6.3 | 1 / 17 / 44 | 0.4678 | +0.3 % |
| 1,2,4,s | 1.86 / 4.55 | 1.80 / 4.42 | 7 / 48 / 101 / 19 | 1 / 17 / 45 | 0.4624 | -0.8 % |
| **1,2,4,s,t** | 1.93 / 4.69 | **1.85 / 4.99** | 7 / 47 / 98 / 19 | 1 / 17 / 44 | 0.4684 | +0.2 % |

V1 (b): base 25.7 / 33.2 (counters), 26.1 / 33.1 ms, steps 329 / 976 / 2212 / 391, alpha 0.3916; 1,2: 8.7 / 15.0 ms,
105 mean; 1,2,4: 1.03 / 1.30 ms, 6.6 mean, alpha 0.3861; 1,2,4,t: 1.08 / 1.43 ms, alpha 0.3910; 1,2,4,s: 3.88 / 7.89 ms,
59 mean; **1,2,4,s,t: 4.14 / 10.06 ms**, alpha 0.3916, lum +0.1 %. 1.0x V3: 1,2,4 3.0 / 5.5 ms; 1,2,4,s 11.8 / 13.9 ms.
References (today's march clipped, `--spike=ref`): V3 alpha 0.4554 (32.0 / 36.0 ms), V1 0.3840 (25.1 / 28.7 ms); all
(b) variants are within 0.014 alpha of them. Display-level difference vs the base (eye 0 PNG, 8-bit, max over RGB):
1,2,4,s,t V3 mean 0.12, p99 1, 0.05 % of pixels > 2 steps; V1 mean 0.04, p99 1, 0.00 %. Lever 3 (0.03): mean 0.43,
max 2 steps, for 5 % fewer steps. Lever 4 with the cell's mean (no t) shows the 100 m cells as stair-stepped edges near
R_near (V3 4 % of pixels > 2 steps, max 156); t removes them (with s: 0.05 %).
Base ms is noisier than round 1 (V3 14.3 ms in a first run, 18.8 ms in the clean matrix): the user works on the machine.

Rules (spec G) with 1,2,4,s,t: (b) <= 5 ms in V3: yes (1.85 ms median, p99 4.99) - build. > 100 occupied steps per
pixel in V3: no (median 7, mean 19). (b) > 10 ms: no. Per occupied step 34.6 ps (more work per step: two grid maps and a
max fetch), but 12x fewer steps. Lever 2 alone is slower than the base (its per-step overhead on every occupied step,
only 21 % fewer steps); lever 3 is not worth its luminance bias. The lever that closes the gap is 4 (+ t for the
image, + s for production's sub-cell detail); 1 and 2 matter only for clear bake cells under it.
