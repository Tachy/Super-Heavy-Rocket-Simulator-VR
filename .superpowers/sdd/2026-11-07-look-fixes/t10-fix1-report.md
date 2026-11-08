# T10 fix round 1 - local adaptation: temporal stability, one source of truth, per-eye (2026-11-08)

Reviewer findings I1 (temporal), I2 (constants / CPU-vs-GPU), minor 3-6, 11.

## I1 temporal stability
Measured first (CPU mirror, headset numbers 31.5 px/deg, block 8, disc 1e4 cd/m^2 over 1e-3, antialiased, moved
0..8 px diagonally in half-pixel steps): the old model popped. 10 px disc: displayed flux ratio 1.0 <-> 0.004 (x250);
16 px: single edge pixels (625 cd/m^2, 1.8 decades under the disc) stayed at c = 1 = display 64 (white sparkles), peak x60.
Causes: block mean in LOG luminance (a partly covered block is dominated by the dark part), hard "like block" cut, edge
pixels too unlike the disc.
Fix (model, CPU mirror and shaders):
- block mean in LINEAR luminance (area-exact), stored as log2; pooled sum LINEAR (Vangorp; weights from log differences) [call]
  (also answers minor 5: the geometric mean underestimated L_loc);
- the like-block share blends log c in by a smoothstep 1 %..4 % instead of a hard cut (`smooth_share`);
- upsampling range sigma 1.0 decade for a pixel dimmer than the block (`UPSAMPLE_DIM_SIGMA_DECADES`) [call].
Result (flux ratio / peak pixel variation over the phase): 10 px x1.86 / 1.84, 16 px x1.54 / 1.57, 24 px x1.22 / 1.22,
32 px x1.12 / 1.17. Sub-block 6 px disc (0.19 deg) is the documented limit (x34, printed not gated).
No temporal low-pass, with reason: a screen-space filter on log c cannot follow a moving object; at the global rule's 0.5 s
light time constant a 30 deg/s head turn drags a ~15 deg ghost (darkened trailing field / bright leading edge). Fast neural
light adaptation (~0.1 s) is faster than a frame at these scales, c follows the same frame; entering lamps / Earth are compressed
at once (documented). Pop on entry is by design.

## I2 one source of truth + CPU-vs-GPU
- `EyeAdaptation`: new consts CONE_MID_SLOPE / OFFSET / EXPONENT (the formula had literals).
- Shaders in `local_adaptation_effect.gd` are templates with `@NAME@` placeholders filled from EyeAdaptation /
  LocalAdaptation (Ferwerda branches, floor, luminance weights, fp16 clamp); push constants built by shared static
  `down_push` / `pool_push` / `apply_push`.
- New `game/tests/local_adaptation_gpu_check.gd` (local RenderingDevice, runs next to T7c renders; needs GPU): 12 discs
  (10..32 px, 3 phases) GPU vs CPU: worst |c_gpu/c_cpu - 1| 0.0010 (tolerance 0.03, fp16 input).

## Per eye (minor 3, 4)
One block size per frame from view 0 (both eyes the same footprint in degrees) - documented. Stereo sweep on the GPU path
(24 px disc, disparity 0..8 px): left/right displayed peak within x1.123 (gate 1.2). Not a full stereo render of the game
(synthetic discs through the real shaders). px/deg at the image centre only: wider pooling in degrees at the periphery,
tagged [call], accepted.

## Tags (minor 5, 6)
[call] on linear pooling, dim-pixel sigma, smoothstep blend, the rods' c^2 shape (Aguilar & Stiles only justifies "rods
saturate"), centre px/deg, block-per-frame. Minor 6 Durand "sec 5.2" / "Eq. 10" are unverified offline (kept as cited).

## Tests (new cases in local_adaptation_test.gd)
translation invariance (10, 16, 24, 32 px; bound x2.0), soft 1/r^2 gradient (c rises outward, no ring), image size not a
multiple of the block (disc at the corner), generated-constants checks. All earlier 21 checks unchanged and green
(disc mean 1.339 / 1.364). adaptation_meter_test, glare_veil_test PASS, smoke_test (see commit message), GPU check PASS.
atmosphere_check not run: no shader it uses changed (only the compositor effect's own shaders).

## Open
- Not done: minor 10 (skip MAP when gain ~ 1), minor 7 known-issues note on desktop/VR difference (in eye-adaptation.md only).
- No A/B game render this round (model change is edge-pixel level; a visual re-check of the lunar-night Earth is advisable
  after T7c frees the GPU).
