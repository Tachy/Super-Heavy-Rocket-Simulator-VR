# Bug hunt: red-shifted horizon clouds at noon (2026-11-08)

Status: DONE_WITH_CONCERNS - reproduced, every listed candidate checked by A/B or measurement, NO code defect; the
colour is the model's physics amplified by the Hunt step. A fix needs a model decision (user); nothing in config /
core / shaders changed. Suite check added (fails until decided).

## Reproduction (fresh DLL 20:25, desktop render_check, pad, 2026-11-07 18:15 UTC, Sun 47.6 deg, --weather=analog)
Images: `screenshots/look_fixes/bug_horizon/` (sheet `sheet_bug_horizon.png`). Displayed linear B/R of the far cloud
band at the horizon (same pixels in every image) vs near cumulus:

| view | horizon | near |
|---|---|---|
| ground el 2 (A_h0) | 0.565 | 0.80 |
| 2 km el 2, game (A_h2000) | 0.639 | 0.846 |
| 2 km --local_adaptation=off | 0.642 | 0.848 |
| 2 km --no_air_shadow | 0.657 | 0.848 |
| 2 km --hunt=day (S 1) | 0.752 | 0.898 |
| 2 km --white_balance=off | 0.455 | 0.586 |
| 2 km --fixed_exposure --tone=agx | 0.781 | 0.866 |
| 2 km linear camera f/22 (scene, no WB) | 0.599 | 0.708 (plate white 0.788) |
| 5 km (A_h5000) | ~0.55-0.63 | - |

Desktop shows it (not XR-specific). The user's GFS view (19:30 UTC, 6.74 km, gfs_1111980_f007) also shows the warm
horizon band on desktop (A_user.png); its weather looked different from the screenshot (near clouds white-grey here),
so item (2) "dirty grey-brown/purple near volume clouds" was NOT reproduced - open (VR mirror / other forecast step?).

## Candidates
- T13 local chromatic adaptation: inactive at noon (adopted white D = 1 -> K = identity); --local_adaptation=off 0.642.
- T7 step 3 bright-top term: gated off below 20 km camera height (weight 0 at 0-5 km).
- ATM_SHORT_PATH_M: only paths < 200 m.
- Air shadow: +3 % only.
- Display chain: computed from the scene value (cam22) with the CAT02 white balance of the logged field illuminant and
  OkLab chroma x 1.504: near 0.847 (measured 0.846), horizon 0.652 (0.639) - the post chain is exact, no defect.
- Aerial perspective: a uniform deck (slab 1000-1500 m) from 2 km follows the core's `sky_luminance_to_point`
  composite row by row (render 0.72-0.79 vs core 0.62-0.69 at 10-40 km: not redder); the core's to-point in-scatter
  is additive (16-segment sum identical) - no table / horizon artefact.
- Direct beam (T9, 680 nm red): horizontal-path check with the T9 replica (`atmosphere_spectrum_check.py` optics):
  T_B/T_R at 30 km exact 0.416, now 0.389, T9b 0.444; 60 km 0.160 / 0.152 / 0.197. The direct beam +7 % B/R is
  mostly cancelled by the bluer plate white. T9b is not the fix.

## Cause
Physics of the configured atmosphere, then the Hunt step:
1. cumulus sides lit mainly by the direct Sun: scene B/R 0.90 of the plate white;
2. 20-60 km of boundary-layer air (Rayleigh + aerosol AOD 0.1, Angstrom 1.3, scale height 1.2 km): T_B/T_R 0.39 at
   30 km, air light bluish but ~0.2 of the cloud -> a white Lambertian cloud 30 km away is 0.845 of the plate white
   (core probe), the actual cumulus 0.76;
3. white balance -> displayed 0.755; Hunt factor S 1.50 (OkLab chroma) -> 0.652 = salmon.

## Levers (measured with the core, `tmp_horizon_probe.gd`, config copies in the scratchpad; probe deleted)
- Aerosol Angstrom 1.3 -> 0.5 (maritime / coastal; Boca Chica is on the Gulf): white cloud at 30 km 0.845 -> 0.937 of
  the plate white, 60 km 1.0 -> 1.13; plate white unchanged (0.790 -> 0.783). Strongest physical lever; global config
  (all sites) - a per-site / per-airmass aerosol would be the clean form. Needs a sourced value (AERONET maritime
  0.3-0.8, continental 1.0-1.5) and the user's OK; re-measure sky colours (suite, sky_radiance_check, atmosphere_check).
- Hunt factor: S 1.5 -> 1 gives 0.752 (horizon) / 0.898 (near). The boost is user-approved (T5); a limited boost for
  near-neutral highlights would be a [call].
- Cloud sky light (suspected model defect, separate task): at 5 km, `STARSIM_BAKED_DEFINES=CLOUD_DIAG_SUN_ONLY` /
  `CLOUD_DIAG_SKY_ONLY` linear camera renders (A_h5000_cam22 / _sun / _sky): the clouds' sky-lit part (full - sun only,
  the air cancels) is 7 % (near), 5 % (mid), 6 % (far) of the cloud luminance; a thick cloud reflects the sky's and the
  Sun's irradiance alike (conservative scattering, albedo ~0.75-0.8 for both), so the expected share is the plate's
  E_sky / (E_sky + E_sun mu0) = 14 % at 1.5 km (core: 13.0 / 80.2 klx G). Half the sky light -> clouds lit in the
  direct beam's colour (near B/R 0.717 vs plate white 0.82); doubling it would add ~+6 % B/R. Check
  `cloud_sky_scattering_source` x J_MAX x light.z against the Sun's j for a thick top before changing.
- Mie g 0.8 -> 0.7: air light +6-10 %, composite B/R +0.1 % - no.
- T9b: small here (see above).

## Suite check
`tools/look_reference_suite.py` scene `horizon_clouds_noon_2km` (2 km, az 90, el 2, fov 60, analog weather):
`horizon_cloud_b_over_r` >= 0.75 and `horizon_over_near` >= 0.85 [D: core composite of a white cloud 30 km away
shown at S 1.5 -> 0.775]. Measured on A_h2000: 0.639 / 0.755 -> FAIL (open until the decision).

## Tests
Python suite compiles; measure-only run of the new scene (FAIL 0.639 / 0.755 as expected); smoke test PASSED. No
game / core / shader code changed (the temporary probe `game/tests/tmp_horizon_probe.gd` was deleted; copy in the
scratchpad). Only my hunks of `tools/look_reference_suite.py` committed (the Moon agent's m_moon_day / day_moon edits
left unstaged).
