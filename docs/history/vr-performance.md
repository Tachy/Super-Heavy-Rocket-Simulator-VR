# VR performance - history

Superseded text of `docs/subsystems/vr-performance.md`.

## Before the Task 10 follow-up (superseded 2026-10-27)

Cloud pass profile, "Options with a look trade" as first written (not applied, user decision open):

Options with a look trade (not applied; user decision, crops `screenshots/static_clouds/t10_opt_*.png`: base | 4
sub-samples | divisor 4):
- at most 4 sub-samples instead of 8: -9.3 / -8.9 ms (-23 / -19 %); the grain check passed with 4 in Task 5c (1.8 %
  vs the 8 x reference's alpha, 8: 0.8 %); visible: slightly more fine speckle in the far deck band.
- march divisor 4 instead of 3 (1.8 x fewer rays): -11.3 / -14.3 ms (-28 / -30 %); visible: softer near cumulus,
  blockier outlines of the lower deck.
- step-weight limit 0.3 instead of 0.15 (a dense step's light sampled for up to 30 % of the pixel): -4.7 / -5.2 ms
  (-12 / -11 %); crops `t10_opt_*_stepweight.png`: mean |difference| 0.5-0.7 of 255, the lobes' shading a little
  flatter; not checked against the reference cases (cloud_check 8 / 9, the grain check).

## Cloud view cache in the frame before Task 4 fix round 1 (superseded 2026-10-28)

> ~1 s at 11 ms per frame, the coarse level 0 (1/4 density) in ~2-3 frames.
>
> Convergence after a start / jump (the planner's restart; VR proxy): level 0 over the view after 0.32-0.37 s (20-23
> frames), the full level over the view after 3.2-3.3 s (191-199 frames), the whole cube after 12.8 / 16.8 s; the fill
> per frame median 4.0-4.4 ms (the level-0 / level-1 phases are capped by the pick's 128 tiles a call, not by the
> budget), max 31-34 ms (overshoot where the picks turn from clear-sky to deck tiles: the cost per full tile varies
> 0.1-1.6 ms). Desktop preset (1600 x 900, budget 6 ms, face 1984): view level 0 after 0.1-0.2 s, full level after
> 2.1-4.3 s, the whole cube after 6.4-13 s; fill median 4.0 ms, p90 7.5, p99 17 ms. Main thread (FrameProfiler section
> `cloud_view_cache`, desktop deck while filling): median 0.6 ms, p90 1.0, p99 1.4 ms, the first frame (cold pick) 3.0
> ms (the layout and schedule are built on a worker: 8 ms at face 1984 before); VR proxy median 0.8-0.9 ms, max 5.5-5.9
> ms (the cold pick at face 3200); 0 once converged.

## Superseded by cloud view cache Task 4 fix round 2 (2026-10-28)

> (1600 x 900, budget 6 ms, face 1984; `t4_rc.sh f1g <view>`): level 0 after 0.6 / 1.7 s (12 / 30 frames, ground /
> deck), covering ended 1.2 / 2.6 s, full level 3.2 / 5.9 s, cube 17.3 / 21.6 s; fill median 3.1 / 3.8 ms, p99 8.2 / 9.1
> ms, max 9.9 / 11.0 ms - over the fixed 6 ms because one full-level deck tile alone takes up to ~8.4 ms (a fill's time
> is its slowest ray's; `weather-clouds.md`, Budget); main thread max 0.19 / 0.20 ms.

## World volume light before Task 6 review round 1 (superseded 2026-10-28)

Superseded by the distance-stepped density march, one dispatch per level and the per-level cost model (vr-performance.md).
Text as it stood:

    ### World volume light (cloud world volume plan Task 6, 2026-10-28)

    GPU intervals now `cloud_volume_upkeep` (bake, max mip, distance field) and `cloud_volume_light` (provisional and real
    light); the upkeep is their sum. Desktop RTX 4090, `--xr-mode off`:
    - Light kernel rate (cloud_check section 17, section 8's thick slab, every ray through tau ~10: the costliest content
      measured): ~2.8e5 cells/ms on average over the cold start, ~6e4 cells/ms at level 0 (every cell within 2.6 km of
      the deck); clear-dominated refresh passes calibrate to 1-2e6 cells/ms. the spec's 1.1e6 cells/ms [E] holds only for
      mostly clear content: a full refresh of the 67 M cells takes ~50 ms of GPU at the clear rate (~1 s of 1 ms frames),
      ~0.25 s in the slab (~5 s at the 1.0 ms light budget: a refresh lags by seconds, never a front).
    - Cold start in the slab (light at the spike's share, the bake at 40 x): every level complete 16-51 frames after its
      density is shown (level 0: 51 frames = 1.13 s, above the 1 s target; levels 1-7 within it); light GPU median
      2.71 ms per frame (the 3 ms spike cap less the bake), max 14.7 ms (the rate calibration lags content changes).
    - `render_check --cloud_view=V1 --world_volume --gpu_time --gpu_passes` (fill and light awaited): cold start in 309
      frames, upkeep (bake + light) median 1.74 ms, p99 13.5 ms, max 21 ms; after it (still view, clock 1 x)
      `cloud_volume_upkeep` median 0.003 / p99 0.003 ms, `cloud_volume_light` 0.003 / 0.003 ms. V3: complete after 822
      frames (its cold-start upkeep series was not captured: one sample); after it 0.003 / 0.004 ms and 0.003 / 0.003 ms.
      The cold start's p99 / max exceed the 3 ms spike cap: per-frame cost follows content (clear vs cloudy cells, taps
      skipped by the clear test) faster than the 20 %-per-frame calibration - the gate (Task 11) measures it in VR.

## World volume light before Task 6 review round 1c (superseded 2026-10-29)

Superseded by N = 32 cells per level and the latency-aware cost model (vr-performance.md). Text as it stood:

    ### World volume light (cloud world volume plan Task 6 and its review round 1, 2026-10-28)

    GPU intervals `cloud_volume_upkeep` (bake, max mip, distance field; per level `cloud_volume_bake_<k>`) and
    `cloud_volume_light` (provisional and real light; per level `cloud_volume_light_column_<k>` /
    `cloud_volume_light_cells_<k>`); the upkeep is their sum. Light: the distance-stepped density march (16 cells per
    level), one dispatch per level and kernel. Desktop RTX 4090, `--xr-mode off`:
    - Light kernel rate, whole-level refresh on the thick slab (cloud_check section 18, every ray through tau ~10), cells
      per ms, levels 0 .. 7: 1.1e6, 3.6e6, 6.1e6, 5.0e6, 1.1e7, 3.9e6, 1.0e7, 1.1e7; the column pass 3.6-8.9e6 column
      cells per ms. A full refresh of the 67 M cells ~17 ms of GPU (~17 frames at the 1.0 ms light budget).
    - Latency floor: a dispatch costs at least its longest ray's march (~0.25-0.45 ms on cloudy content: 7-8 segments of
      32 dependent samples), so small dispatches run far below those rates (level 1's cold-start dispatches of ~170 k
      cells: ~3.5e5 cells/ms). The cost model is linear in cells (plus 0.05 ms per level); small dispatches only raise a
      rate; the refresh serves one level at a time. Before (a dispatch per job, equal shares over eight stale levels): a
      pair switch's light median 1.6 ms for a few thousand cells.
    - Cold start in the slab (section 17, light at 0.65 of what the bake leaves of the 3 ms spike cap): upkeep median
      1.26 ms, p99 1.90, max 2.27 ms (before round 1: max 14.7 ms; at 0.8 of it: max 2.71 ms, and 3.36 against 2.40 ms
      planned in the full check); level 0 complete 30 frames after its density is shown (<= 45 frames, 1 s); levels 2-7
      within 34-52 frames, level 1 48-98 frames over five runs (latency-bound, above).
    - Under load at the game's budget (section 18): the Sun at 100 x for 10 s (4.2 deg) light median 0.80 ms, upkeep max
      1.40 ms; a pair switch (analog + ERA5 -> ERA5 + ERA5, the weight 0.82 -> 0.50 over 10 s, every 0.02 relighting all
      levels) light median 0.92, p99 1.26, upkeep max 1.67 ms.
    - Bake rates per level (render_check V1 cold start, taps per ms, occupancy included): L0 3.7e7, L1 3.0e7, L2-L4
      3.7-4.3e7, L5-L7 0.5-1.0e8 (section 17's slab: L0 1.6e7 .. L7 1.8e8).
    - `render_check --cloud_view=V1 --world_volume --gpu_time --gpu_passes`: cold start complete after 1007 frames, upkeep
      median 1.22 ms, p99 1.84, max 2.23 ms (before round 1: p99 13.5, max 21 ms); after it `cloud_volume_upkeep` and
      `cloud_volume_light` median 0.003 ms. V3: complete after 823 frames (its cold-start series again one sample), after
      it 0.003 / 0.004 ms and 0.003 / 0.003 ms.
