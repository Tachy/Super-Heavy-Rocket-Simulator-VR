# I2 fix report (look-fixes final review)

Status: DONE_WITH_CONCERNS (the finding's mechanism does not reproduce; code simplified, behaviour unchanged).

## What
`game/shaders/cloud_baked_flat.gdshaderinc`: removed `CLOUD_BAKED_FLAT_TAKEOVER_HEIGHTS_M` (40/60 km),
`cloud_baked_flat_takeover()` and the diagnostic `CLOUD_BAKED_FLAT_NO_TAKEOVER`. `cloud_baked_flat_start()` now has no
height branch: it returns the distance where the flat weight first becomes > 0.
`game/tests/cloud_flyup_check.gd`: new option `--heights_km=a,b,c` (explicit camera heights). Docs:
`docs/subsystems/weather-clouds.md` (current state).

## Finding vs. code
The review says the start below 40 km tests the geometric radius r0 only. In the committed code the start already was
`min(radial crossing of r0, footprint term)` (line ~116: `c0 2^l0 / (LEVEL_DROP pixel_angle)`), with the same l0 =
last - max(band, seam band), the same inner0 / c0 terms and the same footprint (`pixel_angle * t`) as
`cloud_baked_flat_weight_at` and the volume's keep (`cloud_baked_lod`). So start == onset of the weight already; the
40 km branch (start 0) differs only where the weight is 0 anyway (flat hits before the onset get w = 0 at their own
distance). Removing it is a no-op by construction and by measurement.

## Measurement (old shader A = HEAD, new B), cloud_flyup_check, same camera/epoch, fixed exposure
- 26-45 km (heights 26,28,30,32,34,35,36,38,40,42,45), Gulf and cumulus, straight down and 45 deg: 44 PNGs, 42
  byte-identical, 2 differ by bake noise; all per-step region / global numbers identical in A.log / B.log.
  Mean luma / cloud share (luma>150) at 30 / 35 / 40 km: `ab_numbers.txt` (e.g. Gulf 45 deg 124.13 / 123.36 / 121.39,
  A == B to 2 decimals in all 12 cases; mean |dL| 0.000).
- 50-800 km (50,70,120,250,410,800): mean |dL| <= 0.001 code, max 3 codes on isolated pixels, share(|dL|>2) 0.0000
  (`ab_numbers_high.txt`).
- No step at 40 km in the climb: 38->40 / 40->42 km region change Gulf 0 deg 2.04 / 0.35, cumulus 0 deg 0.81 / 0.53 codes.
  The open "steps at 29-41 km" is NOT this mechanism: at the Gulf nadir the change is a smooth rise at 34-40 km
  (1.9 / 1.9 / 4.1 / 2.0 codes per 1-2 km steps, mean luma 104 -> 108, 1.7 % per km global), cumulus nadir 1.0-2.0 codes;
  oblique views stay <= 1.3 codes. Still open (cause elsewhere: probably the volume / cover itself, the 40 km geometry
  re-bake or the bright-top weight 20-40 km `CLOUD_BAKED_BRIGHT_TOP_HEIGHT_M` - to be investigated separately; the
  bright-top height fade ends exactly at 40 km and is the first suspect for the nadir luma rise 34-40 km).
- Tests: cloud_check PASS (63 samples), cloud_far_check PASS (49 samples, incl. 5b 39.5-60.5 km), smoke_test PASSED.
  A full 20-2000 km ratio-1.2 flyup was not rerun (15-25 min; the 50-800 km A/B is pixel-identical).

## Files / images
`screenshots/look_fixes/final/i2/`: `sheet_ab_gulf_45.png`, `sheet_ab_cumulus_0.png` (30 / 35 / 40 km, A over B),
`A/ B/ A2/ B2/` PNGs, logs, `ab_numbers*.txt`.

## Open
- The 29-41 km nadir luma rise at the Gulf (smooth, 4 codes / 2 km) - cause not the hand-over start; suspect
  `cloud_baked_bright_top_weight` (20-40 km fade), test by setting that fade constant off in an A/B.
- The review's claim should be annotated: I2 not reproducible, code was already complementary.
