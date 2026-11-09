# Eye adaptation: history

Passages moved out of `docs/subsystems/eye-adaptation.md` (current state only there), verbatim, with their original line breaks. Look-fixes plan final review I3, 2026-11-09.

## Why horizontal: the first reading of the illuminant (T5b fix round 2, review I1) and the pre-T9b illuminant numbers

```text
    Why horizontal (fix round 2, review I1): CIECAM02's adopted white is the illuminant of the scene; the scene is
    mostly horizontal (ground, deck, sea) and daylight is measured as global horizontal light [S: CIE 015:2018 daylight;
    Judd et al. 1964; Hernandez-Andres et al. 2001, JOSA A 18, 1325]. The luminance plate (Sun at normal incidence +
    horizontal sky, below) is unchanged; as a colour it weighted the Sun 1 / mu0 too much (1.35 at Sun 48 deg). Units
    checked (headless print at the pad): `sky_lux` and E_sun are both lux (sky 15 834 lx, Sun 74 646 lx horizontal /
    100 446 lx normal at Sun 48 deg; sky 17.5 % of the horizontal), so the sky had its weight; the white was warm
    because the direct beam is (AOD 0.1 at 550 nm, air mass 1.35; with the T9b channel wavelengths 628 / 546 / 444 nm the
    direct beam is 5135 K, B/R 0.657, as a 1 nm spectrum; before T9b 4940 K = D50).
```

## Mesopic degree of adaptation: renders A (photopic D) / B (mesopic D), T5b fix round 3

```text
    Renders (A = photopic D / B = mesopic D, Starbase, fov 100, el 5, display-linear B/R; `screenshots/look_fixes/
    t5b_fix3/`): dusk Sun -5.5 deg (L_a 12.2) D 0.829 -> 0.751, sky 1.25 -> 1.36, ground 0.95 -> 1.05; blue hour Sun
    -7.7 deg (L_a 1.51) D 0.825 -> 0.523, sky 1.42 -> 1.91, ground 1.11 -> 1.38; full-Moon night (L_a 0.067) D 0.824
    -> 0.186, ground 2.31 -> 2.24, sky 2.79 -> 2.67 (the rods' tint dominates); noon pad, Moon day, cockpit identical
    (D 1 / m 1). At Sun -5.5 deg the blue stays weak: its L_a is the anchored adaptation (the white-plate rule ~9
    cd/m^2), photopic by CIE 191 (L_A 2.4: m 0.91), while the frame's own mean is ~2 cd/m^2.
```

## White balance: numbers A / B of T5b fix round 1

```text
  Numbers, A (HEAD 3dd8340a) / B (this round), display sRGB of named regions, `screenshots/look_fixes/t5b_fix1/`
  (numbers.txt, sheet_*_ab.png): see the T5b report's "Fix round 1" (`.superpowers/sdd/2026-11-07-look-fixes/t5b-report.md`).
```

## Display tone step: glow-bound A/B/C (pre-T5 / T5 / fix round 1)

```text
A/B/C (pre-T5 / T5 / fix round 1, `screenshots/look_fixes/t5_fix1/`): pad noon Sun view, displayed
  luma of the aureole r 40-60 px 0.760 / 0.839 / 0.760, r 60-100 0.737 / 0.805 / 0.736, r 100-200 0.652 / 0.660 /
  0.651 (the T5 blue glow gone; the colour stays the scene's, B/R 1.07 / 1.38 / 1.35); night plume halo (97-99.5 %
  band) 0.144 / 0.153 / 0.153, noon plume 0.403 / 0.399 / 0.399; cabin (pilot eye, 97-99.5 %) 0.822 / 0.826 / 0.825.
  The night plume's ~6 % wider halo is the emitters' inverse under additive blending (T5), not the Hunt boost (S < 1
  there). 
```

## Display tone step: T5 A/B renders and cost

```text
  A/B (A = AgX + S against L_day, B = this step + S against the display; `screenshots/look_fixes/t5/`, report
  `.superpowers/sdd/2026-11-07-look-fixes/t5-report.md`): pad noon el 5 (S 1.58) sky el ~29 sRGB (82, 118, 154) B/R
  3.81 -> (62, 111, 182) B/R 9.85, OkLab hue -110.3 -> -102.4 deg = the scene's -102.4 (AgX turned it 7.9 deg); zenith
  (el 85, S 1.53) (105, 133, 163) -> (87, 128, 192), hue -109.0 -> -101.1 (scene -101.1); displayed luma within 7 %.
  Start view (weather analog, S 1.61): ocean (60, 81, 110) B/R 3.50 -> (61, 82, 125) 4.36 (EPIC 4.8-5.6); 400 km nadir
  ocean (48, 72, 102) -> (39, 68, 117) (ISS photo (24, 76, 123)). Sun disc unchanged (75 px at display white in both,
  centre (247, 247, 247) -> (245, 250, 254)); plume core noon (253, 233, 219) -> (255, 234, 221), night (254, 228, 202)
  -> (255, 231, 213). Cost (the step + its bisection only where C / 0.75 leaves the gamut): pad view 1600 x 900 median
  1.858 / 1.860 -> 1.923 / 1.905 ms (+0.06 ms), stereo mock 3534 x 2838 per eye 5.910 / 5.732 -> 6.041 / 6.034 ms
  (+0.13 / +0.30 ms, two rounds).
```

## Standing decisions: side effects of the highlight cut of the lit weight (2026-11-07)

```text
  Side effects (renders `screenshots/look_fixes/t5b/`): the visibility V keeps the old cut of the whole weight
  (`field_log_average(.., cut_of_whole)`), else a small lit object only the frame has (the Earth's disc in the lunar
  sky, absent from the model) closes V (first render: gain 12.99). A lone small lit object now enters the combined field
  (lunar day el 60: the Earth disc, near the view centre, field 2617 -> 4048 cd/m^2, gain 2.98 -> 1.93); the start view
  1.401 -> 1.269 (the cut of the ~30 % lit weight leaves more bright cloud in the field). The Sun's disc no longer
  depends on this cut: it is masked out by its direction and its glare is added as a veil ("Sun glare" above).
```

## Open points: the anchored rule alone (2026-11-07 renders and numbers)

```text
- Anchored rule alone (2026-11-07, render_check / cockpit_shots, `--hide_ui`, PNGs and sheets
  `screenshots/eye_adaptation/anchored/`; field L_f / anchor L_anchor cd/m^2, gain A field / B white plate / C anchored):
  Apollo 11 day 1345 / 2710, 5.80 / 1.00 / 14.4 (ground sRGB luma mean 156 / 71 / 190); lunar night 0 N 45 E 0.045 /
  0.063, 8844 / 3012 / 10 843; Starbase noon 4526 / 8013, 1.72 / 1.15 / 4.85; dusk (Sun -3.6 deg) 19.6 / 86.0, 397 / 778
  / 452; 400 km day 2638 / 5649, 2.95 / 1.00 / 6.88; cockpit pilot_eye (A at the game key 0.18, not the shot's 0.5)
  15.9 / 242, 480 / 1.98 / 157. The anchor is only 2-4.4x the field in these centre-weighted fields, so C is brighter
  than A outdoors everywhere; at noon and from orbit the anchor is the sky / the hazy limb (a self-luminous area seen
  as a white surface), which washes the ground out. Open: percentile, ANCHOR_WHITE, a surface-only anchor.
- The rendered Apollo 11 regolith (490 cd/m^2 in the foreground probe, field 660-710) is 3-6x below the core's Hapke
  I/F x E_sun,n / pi at that geometry (I/F 0.08-0.10 at 25-30 deg down-Sun -> 3400-4300 cd/m^2 with the highland w
  0.429); the photo's f/11 1/250 ISO 160 exposure (mid grey 2363 cd/m^2) also points to ~2000+ cd/m^2. The field rule
  hides this (it normalises the field), the camera mode does not: an open point of the Moon shading, not of the
  adaptation.
```

## Limits (superseded text of the Night vision entry, before T10 / T13)

```text
  - Limits: one adaptation per view (no local / foveal adaptation: bright parts in a dim view still saturate - the
    full Earth from the lunar night, the full Moon's disc, city lights take the rods' tint); no loss of acuity; glare
    only of the Sun (the Moon's and the Earth's disability glare not modelled); rods' slower dark adaptation (~30 min) paced like the cones' (110 s); no airglow, zodiacal light or
    integrated starlight (a moonless sky is black); Jensen's k on the adaptation luminance, not per pixel; B and S one
    value per view, no Bezold-Bruecke hue shift (lunar day keeps B = 1, S 1.66).
```
