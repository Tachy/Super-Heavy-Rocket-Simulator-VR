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

## Moon procedural surface (2026-11-05)

Stereo mock at the VR target (3534 x 2838 per eye), 2 m above IM-2, terrain only (no clouds on the Moon): detail off
3.24 ms, detail on 4.96 ms (+1.7 ms, ~2.2x the drawn triangles; Moon procedural surface plan Task 7). Of the 9 ms GPU
target ~4 ms remain for the plan's textures, Hapke shading, rocks and near-field shadows (Tasks 8-12); Task 13's budget
table starts from these numbers. Hapke shading (Task 9): +0.05 ms at 1600 x 900 (2 m, detail on: 1.74 against 1.69 ms
median); not yet measured in the stereo mock.

Regolith detail shading (Task 10), 2 m above IM-2, elevation -20 deg, median of 240 frames: 1600 x 900 1.82 -> 2.27 ms
(+0.45); stereo mock (3534 x 2838 per eye) 5.76 -> 7.21 ms (+1.45: statistics alone +0.34, the textures +1.11 with
half-float slope maps; float32 maps +1.47; trilinear instead of anisotropic filtering would save another 0.28 ms, not
taken: the grazing VR view keeps its texture detail). 1.8 ms of the 9 ms GPU target remain for rocks and near-field
shadows (Tasks 11-12): Task 13's budget table. Task 10 fix round (crater component in the slope maps, four-channel
half-float maps, scale s3), same views, median of 120 frames, two alternating rounds: 1600 x 900 1.80 -> 2.32 ms
(+0.52, was +0.45); stereo mock 5.69 / 5.78 -> 7.85 / 7.86 ms (+2.1, was +1.45): ~1.15 ms remain for Tasks 11-12. A
per-fragment analytic crater field (the review's preferred route) cost 13.0 against 7.6 ms (+5.4 ms) and was dropped.

Drawn rocks (moon procedural surface Task 11, `MoonRockLayer`), stereo mock, 2 m above IM-2 (Sun 6.75 deg), elevation
-20 deg, median of 120 frames, budget control held (`--rock_budget=off`): rocks off 7.77 / 7.87 ms (two rounds); first
version (every listed instance through the vertex shader) 10.48 / 10.48 (+2.6; a flat-colour fragment 10.22: the cost
is the vertex work of collapsed instances); with the per-node visible counts (diameter-sorted lists, prefix drawn)
8.57 ms with pebbles from 1 cm, 8.35 from 2 cm (shipped: +0.53 ms), 8.16 from 4 cm.

Task 11 fix round B (user decision 2026-11-05: on the Moon below 90 fps is acceptable, quality first; the rock control
is only a floor guard at 22.2 ms GPU = 45 fps, the main-thread hitch gate of 5 ms stays). Same views, median of 120
frames, two alternating rounds: IM-2 2 m rocks off 8.08 / 8.14 ms (was 7.77 / 7.87: the 1024 px regolith maps and
the earthshine statistics), rocks on 8.73 / 8.76 ms (+0.63 ms, 4.48 M drawn rock vertices: 0.14 ns per vertex; facet
normals per fragment included). Diviner boulder field (39.36 N, -0.17 E, abundance load factor 3.81 at the camera,
Sun 48.6 deg, 600 frames): rocks off 9.88 ms; rocks on with the floor guard 20.1 ms median (p99 27.8; the scale rose
from the load factor to ~7.7; 97 M drawn vertices at scale 1, 0.21 ns per vertex), pool 650 MB. Before the guard
learnt the rocks' cost (0.05 ns per vertex assumed) it held at 30.3 ms: the estimate now uses the higher measured
value (`ROCK_NS_PER_VERTEX` 0.21). The stereo mock is 3534 x 2838 per eye; the headset's 5294 x 4250 draws ~2.5 x the
rocks (M3, T13). Pool memory: ~210-220 MB of GPU buffers at IM-2 2 m once warmed (`MoonRockLayer.PREWARM`, per (LOD,
capacity) key), 650 MB in the boulder field. Main thread: the rock layer's share during a 10 km camera jump max
1.1-1.2 ms per frame (attach queue 2 ms budget incl. the pool warming, visible counts 0.5 ms); the quadtree's own
update without rocks max 13.5 ms (known-issues, T13).

Near-field Sun shadows (moon procedural surface Task 12, `MoonNearShadow`), stereo mock, 2 m above IM-2 (Sun 6.75 deg),
azimuth 150 deg from the Sun, elevation -20 deg, fov 60, median of 120 frames, `--rock_budget=off`, two alternating
rounds: shadows off 12.18 / 12.09 ms, on 16.81 / 16.07 ms (+4.0-4.6 ms). The off numbers include Task 12's oblique-eye
rock terms and fill light (11.5-11.7 ms with the first, coarser view table; Task 11's 8.7 ms were another view). Diviner
boulder field (39.36 N, -0.17 E, same instant and view): 39.4 -> 51.8 ms (+12.4 ms). Breakdown at IM-2 (first version,
every drawn rock casting, PCSS medium; off 11.6 ms): receivers only, no casters 14.1 ms (+2.5: the lit pipeline's light
loop and the PCSS lookups; hard PCF instead saves 2.0 ms), + terrain casters 16.0, + rock casters 16.3, all 17.6 ms; 2
splits instead of 4 -1.1 ms. With every drawn rock casting the boulder field took 120.7 ms (42.3 off: the shadow pass ran
every listed rock's vertices once per split), hence the casting threshold 4 x the geometry threshold (separate
shadow-only MultiMeshes with their own visible prefix) and PCSS LOW (medium and ultra give the same mean radiance).
Task 12 fix 1, same views, median of 120 frames, two rounds each: the game before Task 12 (62ee7474^ game files, the
current core) IM-2 9.30 / 9.31 ms, boulder field 32.2 / 33.6 ms; Task 12 as committed (same session) IM-2 off 11.92 /
on 16.39, boulder off 38.3 / on 51.9; after fix 1 IM-2 off 11.63 / 11.71, on 16.03 / 16.09 ms, boulder off 37.3 / 36.7,
on 51.4 / 51.7 ms. So Task 12 costs +6.8 ms at IM-2 (+2.4 the oblique-eye terms and fill, +4.4 the map) and +18.5 ms in
the boulder field (+4.1 statistics, +14.4 the map). Gating the rock casters by the map's reach (fix 1) removed the far
nodes' casters but changed the GPU time by < 0.5 ms: the map's cost is the receivers, the terrain casters and the near
rocks: boulder field without casters (receivers only) 44.4 ms, + terrain casters 45.9, + rock casters 51.5. The fill's
wall term with Smith's G1 per lit horizon sector (added last in fix 1) measured 16.75 ms at IM-2 in one round (+0.7 ms
against 16.03 / 16.09; not repeated).

## Earth near-field Sun shadows (moon procedural surface plan Task 16, 2026-11-05)
Stereo mock (3534 x 2838 per eye), Starbase Pad B with Starship on the mount, 2026-10-15 21:30 UTC (Sun 31 deg), ground
eye 150 m and 400 m east of the pad aimed at the vehicle (render_check `--vehicle=starship_block3 --site=starbase_pad2
--watch_from_site=150|400`), weather off, median of 120 frames, two interleaved rounds. Before Task 16 (unshaded
terrain, no shadow map): 4.10 / 4.02 ms (150 m), 4.39 / 4.33 ms (400 m). Task 16, shadows off (the lit terrain's off
path): 4.19 / 4.20 and 4.28 / 4.28 ms (no measurable cost). Shadows on with the committed settings (8192 px atlas,
4 splits, PCSS HIGH): 5.48 / 5.50 and 5.50 / 5.51 ms: +1.2-1.3 ms, 5.5 ms of the 11.1 ms frame. Soft-filter quality
(same views, one session): LOW 4.84 / 4.77 and 4.85 / 4.79 ms (+0.5-0.6), MEDIUM 4.88 / 4.95 and 5.02 / 5.03, HIGH
5.80 / 5.41 and 5.44 / 5.50; LOW and MEDIUM left a visible grain in the wide penumbrae on the terrain, hence HIGH.
Some runs shared the machine with another agent's Godot runs; the medians agree within 0.1 ms across
rounds. Not measured: the headset itself, the pad with weather on (the 45 Hz rule applies there).
Atlas 8192 px 16-bit (128 MB computed; was 32-bit 256 MB with the same image, Task 13b): GPU -0.56 ms in the stereo mock
(3534 x 2838, pad with Starship, eye 150 m, Sun 32.7 deg: shadows off 7.44 ms, 32-bit 10.55, 16-bit 9.99; analog
weather on: 14.37 off, 17.19 on - the clouds' cost, known-issues). Active to 30 km (full to 20 km, fading to 30 km).
4096 / 2048 px atlases fail the shadow-length tolerance (Task 13b), so 8192 stays.
Task 13 re-measure of Task 16's views (same commands, 16-bit atlas, alone on the GPU, median of 120 frames, two rounds):
see the table below. The cost depends on the view: +1.0-1.1 ms in Task 16's views (the vehicle and tower a small part of
the frame), +2.55 ms in 13b's view (its off baseline 7.44 ms is a heavier framing of the pad). Both are right for their
views; the game's settings text says "1-2,5 ms".

## Task 13 frame times (Moon procedural surface plan, 2026-11-06)
Stereo mock (`render_check --stereo=game/tests/data/pimax_dream_air_projections.json --gpu_time=120`, 3534 x 2838 per
eye), alone on the GPU (RTX 4090), median of 120 frames, two rounds (r1 / r2). Moon: 2026-11-22T01:20 UTC, azimuth 150 deg
from the Sun, elevation -20 deg, fov 60, `--rock_budget=off` (threshold scale 1); IM-2 (-84.79, 29.20; Sun 6.75 deg) and
the Diviner boulder field (39.36 N, -0.17 E; Sun 48.6 deg, abundance load factor ~3.8 at the camera); near shadows on /
`--moon_near_shadows=off`. Pad: Task 16's views (`--vehicle=starship_block3 --site=starbase_pad2 --watch_from_site=N
--utc=2026-10-15T21:30:00`, Sun 31 deg), Earth near shadows on / off. Includes Task 17's rocks, Task 13's Hapke rock
tables, pool warming for six LODs and the coarser shadow-only rock meshes.

| View | Eye | Shadows off ms | Shadows on ms | Shadow cost ms |
|---|---|---|---|---|
| IM-2 | 2 m | 13.92 / 13.98 | 17.46 / 17.62 | +3.6 |
| IM-2 | 100 m | 14.10 / 14.10 | 16.88 / 16.86 | +2.8 |
| IM-2 | 2 km | 10.06 / 10.08 | 11.66 / 11.66 | +1.6 |
| boulder field | 2 m | 38.24 / 38.35 | 44.87 / 44.89 | +6.6 |
| boulder field | 100 m | 22.24 / 22.22 | 26.34 / 26.31 | +4.1 |
| boulder field | 2 km | 12.07 / 12.04 | 13.91 / 13.93 | +1.9 |
| pad, eye 150 m | 150 m east | 4.20 / 4.20 | 5.26 / 5.20 | +1.0 |
| pad, eye 400 m | 400 m east | 4.27 / 4.29 | 5.25 / 5.31 | +1.0 |

p99 within 0.5-1.6 ms of the medians. Against Task 12 fix 1 (same views): IM-2 2 m off 11.6-11.7 / on 16.0-16.1 ms, boulder
2 m off 36.7-37.3 / on 51.4-51.7 ms: the off path is +2.3 ms at IM-2 (Task 17's rock surface shader and its sixth LOD,
Task 13's brighter Hapke rock statistics are free), the shadow cost in the boulder field fell from +14.5 to +6.6 ms (the
shadow-only rocks one LOD coarser). Ruling T11-U3 (user 2026-11-05: on the Moon quality before 90 fps): IM-2 meets the
45 fps floor (22.2 ms) at every height with shadows (57-86 fps in the mock), the boulder field at 100 m and 2 km but not
on its ground (44.9 ms, ~22 fps; 38.2 ms already without shadows - the rock budget control was held off here; in the game
the floor guard raises the threshold scale toward 22.2 ms). The headset's 5294 x 4250 per eye draws ~1.6 x the mock's
pixels (the rocks' vertex cost scales with the pixel angle: ~2.5 x, Task 11): the Moon numbers above are lower bounds for
the headset. The Earth pad stays within the 9 ms GPU target (5.2-5.3 ms with shadows).
Main thread (moon_surface_check, 10 km camera jump at 2 m): the rock layer's share max 4.82 ms (IM-2) / 4.75 ms (boulder
field) per frame, gate 5 ms - passes since the pool warms the six LODs' buckets (was 36.9 ms at Task 17). The quadtree's
own selection phase max 21 / 52 ms, p99 14.4 / 16.6 ms (known-issues). Pool: 194 MB at IM-2 2 m once warmed (6930 warm
allocations), the 512 MB bound in the boulder field (353 nodes statistical). The pool's MB are computed from instance
counts x bytes; the driver's real allocation was not read.
