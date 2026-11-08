# T10 report - Earth from the lunar night: the halo and the overexposure; local adaptation (2026-11-08)

## Diagnosis (before any fix)
- **The rim is Godot's glow** (world.gd: threshold 4, intensity 0.6; Godot caps the glow input at luminance 12, so the
  halo does not depend on the source's luminance or the eye). Same camera, `render_check --glow=off` (new switch):
  luma around the disc at 1.5 / 2 / 3 / 4 / 5 / 6 / 8 / 10 deg 223 / 213 / 172 / 122 / 81 / 55 / 21 / 8 with glow,
  0 everywhere without. Not the Sun glare veil (Sun-only, adaptation-only), not the Earth shader, not MSAA/mips.
- **CIE 146:2002 comparison** (full equation, age 25, p 0.5; E = 6.9 lux at the eye = the log's earthshine; displayed at
  this view's gain, scene/cd 0.103, through AgX): 238 / 223 / 186 / 156 / 126 / 103 / 74 / 57 at the same angles. So
  Godot's glow is of the physical veil's order out to ~5 deg (it falls off faster beyond), but only by accident: it is
  capped and not tied to the source.
- **The overexposure** is the global rule D: gain 7557 (L_a 0.49 cd/m^2) puts the ~1e4 cd/m^2 Earth at scene ~1000; AgX
  white is 16.3. 96 % of the disc's pixels clip.

## Fix: local adaptation (`game/world/local_adaptation.gd`, `local_adaptation_effect.gd`)
- Model: g_px = min(g, G(L_eff)), L_eff = L_loc x DAY_EXPOSURE / (PLATEAU d^2); G = the global rule's cone gain
  (Ferwerda, [1, 1e5]); only lowers the gain (fast light adaptation, slow dark adaptation, Pattanaik 2000). In the Weber
  range a locally adapted region shows its local mean at PLATEAU = ANCHOR_WHITE x DAY_EXPOSURE = 1.35 [E: Gilchrist
  1999 anchoring, as the anchored rule; sRGB ~213]; monotonic in L_loc (flat at the plateau, rising again at the gain
  floor: the Sun's disc stays white). Regions at or below 1.35 in the global image: untouched.
- Pooling [S: Vangorp, Myszkowski, Graf & Mantiuk 2015, ACM TOG 34(6) 166, Eq. 10: Gaussian sigma 0.428 deg; the
  adapting field levels off at ~0.5 deg]; edge-aware [S: Durand & Dorsey 2002, range sigma 0.4 decades] so there is no
  dark ring / bright limb (call: a display requirement). Rods' post-step part x c^2 (cones' x c) [E: rod saturation,
  Aguilar & Stiles 1954] - it was 38 % of the disc's displayed Y and turned it blue.
- GPU: MAP stage before the transparent pass (after the field meter; block means of log2 luminance incl. the cloud
  composite, bilateral pooling, log2 c per block; asks for the MSAA resolve every frame), APPLY stage after VisionEffect
  (joint bilateral upsampling against each pixel, 4 x 4 blocks). Per view (both eyes). Emitters drawn in the transparent
  pass over a dark field keep c = 1.
- Switch: `--local_adaptation=off` (game, render_check, cockpit_shots; `LocalAdaptation.enabled`).
- As a side effect Godot's glow no longer fires on a locally adapted disc (written below its threshold): the rim is gone.

## Numbers (A = `--local_adaptation=off`, B = default; same camera / time; sheets `screenshots/look_fixes/t10/`)
| scene | gain | A | B |
|---|---|---|---|
| lunar night 0N 45E, Earth ~97 % (2026-11-07) | 7557 | disc sRGB (254,254,254), 96 % clipped, luma sd 3; halo 2-3 deg luma 194, 5-6 deg 67 | disc (188,209,244), luma sd 18 (ocean / Africa / clouds visible); halo 0.7 / 0 |
| lunar night, Earth gibbous (11-04) | 8156 | (253,253,254) 95 % clipped | (184,206,243), sd 24 |
| lunar night, Earth half (12-01) | 9266 | (251,252,253) 90 % clipped, half disc lost in glow | (180,201,240), sd 27, half disc |
| lunar night 0N 80E, Earth crescent (11-26) | - | crescent white + glow | 1-2 px crescent still white (sub-block, see open points), night side and halo as before minus glow |
| Earth filling the view (fov 12, 11-07) | 6.1 | (137,179,241) | identical (consistency: B's wide disc ~ this, slightly brighter) |
| Starbase night, full Moon fov 100 | 10 647 | 3-4 px Moon white | unchanged (sub-block) |
| full Moon fov 12 | 6645 | disc (255,255,255) 100 % clipped, glow | (233,211,189) with maria, no glow |
| full Moon, stereo mock 3534 x 2838 / eye | - | white + glow | maria visible, both eyes |
| Earth from space (EPIC start view) | 1.000 | - | pixel-identical (max diff 0) |
| pad noon el 5 | 1.100 | - | mean abs diff 0.013 / 255, 0.04 % px > 2 (a horizon band, both signs: run noise) |
| lunar day Apollo 11 el 5 | 3.291 | ground (118,111,94) | identical; 0.1 % px > 2 (both signs) |
| cabin pilot_eye (T-60, cabin lamps + daylit windows) | 157.4 | windows clip (2 % px saturated) | windows show the sky (0.08 % saturated); walls / screens unchanged (172.8 -> 172.7, 1/14/35 = same) |
The cockpit_shots harness has no --utc: the cabin is the pad day case (lamps + windows); no night-cabin shot.

## Cost (RTX 4090, stereo mock at the headset's 5294 x 4250 per eye, `--gpu_passes` timestamps, median of 300 frames)
- First version: MAP 0.60 ms, APPLY 0.46 ms (both eyes). Final (block 8 at 1.5 blocks / sigma, 4 x 4 sample grid per
  block, APPLY reads the 16 map texels first and skips pixels with no compression near them): MAP 0.27 ms, APPLY
  0.33 ms (two rounds, identical). Plus the MSAA resolve before the transparent pass every frame (Task 14 measured
  0.26 ms at 3534 x 2838). Whole-frame medians on / off 10.25 / 10.13 ms - the GPU was shared with T7c, so the frame
  totals are only indicative. Desktop result unchanged by the optimisation (B3 vs final B: max diff 0).
- Headset-resolution look (stereo eye 0, full Moon): `crop_vr_full_moon_final.png` - maria visible, no glow.

## Tests
local_adaptation_test (new, headless): 21 checks pass (plateau 1e-9, monotonic 1e-3..1e10 cd/m^2, Sun at the gain
floor, fixed exposure, block sizes, rods removed (38 % of Y), image case at block 2 and 4: mean 1.349 / 1.368, detail
0.3 / 0.7 %, no halo (864 px c = 1), no rim (limb 1.908 / 2.034 vs 1.904 / 1.962), emitter c = 1, flag).
adaptation_meter_test, white_balance_test, glare_veil_test, display_tone_test PASS; atmosphere_check PASS (88 samples,
0 failures, GPU); smoke_test exit 0.

## Open points (user decisions)
1. Physical glare of the Earth / Moon: no veil is drawn now. CIE 146 predicts for the full Earth at this adaptation a
   veil of 18 / 6.4 / 1.3 / 0.46 cd/m^2 at 2 / 3 / 6 / 10 deg (13x L_a at 3 deg; the full Moon from the Earth ~40x less).
   Proposal: a CIE 146 glare spread function for Sun, Earth, Moon replacing Godot's capped glow (glow also serves plume,
   lamp lenses, Sun disc - unchanged, not CIE-bound).
2. Structures narrower than ~2 blocks (thin crescent, a 3-4 px Moon at desktop fov 100) keep c = 1 (VR has ~4.8x the
   pixels per degree).
3. The locally adapted Earth is light blue with the ocean at the gamut edge (as when it fills the view); the full Moon
   pale cream (B/R 0.81: the moonlit night's adopted white, not this step).
4. PLATEAU 1.35 is an [E] choice (anchored white); 2.0 (first try) made the disc nearly white (sRGB ~240).
5. A display-referred emitter over a locally adapted region (label on the Earth's disc) takes its factor.
6. During ~5 min of the work a GDScript parse error in local_adaptation.gd (Basis + Basis) broke world.gd's compile;
   T7c renders started in that window would have failed to load - fixed, all later renders fine.
