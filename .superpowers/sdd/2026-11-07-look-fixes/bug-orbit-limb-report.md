# Bug 2026-11-09: double limb from 212 km, orange / red / brown clouds from orbit

Status: DONE (the double limb was a defect and is fixed; the orange, red and brown colours are physical, so no change).

## The user's scene
- From the game log (`%APPDATA%/Godot/.../logs/godot2026-11-09T06.53.01.log`, the CloudBaked anchors in render axes
  (y, z, x)), the camera was at **46.2 N 161.7 E, ~210 km** (NW Pacific, east of the Kurils), not over a desert.
- At 2026-11-09 05:52:33 UTC the Sun is **-1.47 deg** at the nadir (render_check confirms: "Sun elevation at the eye
  -1.47 deg"), azimuth 247 deg. The white dot in the user's image is the **Sun** (13 deg above the limb). The user was
  looking toward the setting Sun, not away from it.
- Reproduced with render_check (`--lat=46.2 --lon=161.7 --height=212000 --eye_height=0 --look_at_sun --fov=50`,
  GFS `gfs_1111992_f005.wthr` and analog). It looks the same as the game: the user's deck B/R is 0.18 linear, the render's 0.15.

## (a) Double limb: a defect, fixed
Diagnosis (all images in `screenshots/look_fixes/bugs-2026-11-09/orbit/`):
- `diag1`: with weather off there is no band; `CLOUD_BAKED_DIAG_NO_SCENE` makes no change, so the scene depth is not the cause.
- `diag2`: `CLOUD_BAKED_DIAG_LEVEL` is continuous through the band, so the level, height and cover are fine, and the hits are
  not behind the Earth.
- `diag3`: SUN_ONLY still shows the band and `--no_air_shadow` does not remove it, so the cause is the Sun's light on the cloud.
- `diag4` (temporary defines): the cloud radiance alone (no air) drops to ~0 in the band, while the air
  transmittance and bright-top term are smooth (I tried gating the bright-top at mu > 0 and it made no change, so I reverted it).
- `diag5`-`diag7`: geometric view cosine > 0.02 and the ray hits sea level. T_sun(p), mu0, cov and alpha are all continuous.
- **Cause:** `cloud_far_cloudy_radiance` turns the slab's light-entry normal toward a low Sun (3D light,
  `CLOUD_FAR_ENTRY_TURN` 0.15: about 3-8 deg). `cloud_far_light_radiance` then decided the camera's side of the slab by that
  turned normal. A deck seen from above toward the Sun, at less than the turn angle, counted as "below". It was drawn
  as the diffusely transmitted light of its base, which is near black for a thick deck. That gives a dark band under the limb whose
  lower edge is a second horizon line (stair-stepped by the half-resolution cloud pass). It only appears toward the Sun, at all
  heights 150-450 km. It is not the 205-410 km volume / far-field cross-fade. It hits the flat levels and the scaled-space far field
  (same function).
- **Fix** (`game/shaders/cloud_far.gdshaderinc`): `cloud_far_light_radiance_side(..., bool below)`. The slab's side
  now comes from the layer's geometric up, and the turned normal only sets the light's entry. The view cosine against the
  turned normal is held at CLOUD_FAR_MU_MIN through the turned plane, so it stays continuous. The facets keep their own normal's side.
  `cloud_far_light_radiance` is a wrapper with the old signature (the Moon and the probe use it).
  There is no model value change and no core mirror; the code is GPU only.
- **Gate:** new suite scenes `orbit_limb_212km` (analog) and `orbit_limb_212km_gfs` (if the GFS file is cached) in
  `tools/look_reference_suite.py`. `m_orbit_limb` takes the columns' median luma profile from the limb down, and
  `limb_band_rel` = its mean 3-5 % of the height under the limb over its maximum, gate >= 0.85.
  - A: GFS 0.626 / analog 0.706, both FAIL.
  - B: 0.970 / 0.928, both pass.
  - Weather off: ~0.97.
  - I first tried a per-column valley metric and dropped it, because it reads legitimate cloud gaps in broken analog cloud as bands.
- Sheets:
  - `sheet_orbit_limb_AB.png`: GFS and analog, A over B.
  - `ab1.png`: the GFS view.
  - `sweepAB.png`: 150 / 212 / 300 / 450 km toward the Sun, A and B. The band is gone at every height.
  - `ab_analog.png`.

## (b) Orange / red: physical
Sun elevation at the visible cloud points (looking toward the Sun from 212 km):

| Where | Sun elevation |
|---|---|
| Nadir | -1.5 deg |
| Bottom of the user's view (~450 km out) | ~+2.7 deg |
| Limb (~1650 km) | ~+13 deg |

The whole lit field is a sunset deck.
- **Direct beam (T9 table):** 2 deg ~2070 K, 5 deg ~2850 K, 15 deg ~4190 K. As linear sRGB (D65), a blackbody at
  2400 / 2850 / 3500 / 4200 K has B/R 0.05 / 0.13 / 0.26 / 0.43.
- **Eye:** adopted white is the field illuminant, B/R 0.856, D 1 (WB diagonal B/R x1.09). The Hunt factor S is 1.53-1.66.
- **Displayed:**
  - The deck at a 5-7 deg Sun is linear B/R 0.15-0.18 (user's game 0.18, render 0.15). That matches a ~3000-3300 K beam plus blue sky light.
  - The near broken cumulus at a 0-3 deg Sun is sRGB (97, 74, 72), B/R 0.55: dim red. These are the user's "red spots".
- I found no misapplied local chromatic adaptation or Moon field tint: the moonlight is off and the log shows no Moon field share.
- Not a defect, but still open: the near-field red spots look blocky. That is the known T7c open point "the volume's coarse levels from
  orbit" (blocky cumulus within +-410 km), which the user accepted 2026-11-08. Image: `B_down212.png`.

## (c) Brown / olive patches: physical
In `T_COV` / DIAG_COV the patches have lower alpha:
- patches: alpha codes 116-164
- deck: alpha codes 176-199

They are thinner or partial cloud (the sub-grid / meso cover pattern) over the dark twilit ocean (Sun 0-3 deg).
The orange-lit cloud mixes with the dark blue sea and the blue air light in front, which reads as brown. Per patch:

| Patch | sRGB | linear B/R |
|---|---|---|
| User, deck gaps | (129, 96, 69) | 0.27 |
| User, lower view | (78, 57, 52) | 0.45 |
| Render, deck partial-cover areas | (227, 181, 125) | 0.27 |
| Render, open ocean between near cumulus | (27, 42, 63) | 4.6 |

These are not dark holes from the optical-depth spread, not a far-field colour defect, and not a GFS cover/water mismatch.

## Tests
- Suite A/B, unchanged by the fix (numbers identical): horizon_clouds_noon_2km 0.751 / 0.843 ok, horizon_clouds_noon_300m 0.758 / 0.881 ok,
  dusk_m5_5 ok, terminator_400km band 0.220 -> 0.221.
- earth_from_space_epic white 0.345 FAILs in both A and B; it was already accepted at 0.35 (ledger).
- New orbit_limb scenes pass on B.
- `cloud_far_check.gd`: PASS (49 samples, 0 failures). `smoke_test.gd`: PASSED.
- `cloud_flyup_check` was not run (~20 min). Its views look down or obliquely at 30-60 deg zenith, never grazing toward the Sun at less
  than the turn angle, so they should be unaffected.
- The DLL was not touched (shader + Python only).

## Open points
- The bright-top transmittance (`cloud_far_bright_top_transmittance`) gives ~0.5 of the cloud radiance at grazing views.
  That makes the limb clouds toward the Sun bright and cream right to the edge (forward scattering). This is plausible for a bright deck,
  but there is no reference. It is a possible check against ISS limb photos.
- The limb silhouette is stair-stepped at cloud divisor 2: the half-resolution upsample at the limb, an existing behaviour.
- In the volume (`cloud_scattering_source_3d`) the view cosine is also taken against the turned normal (no side switch there, so no band).
  I did not change it.
