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
