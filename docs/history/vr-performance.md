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
