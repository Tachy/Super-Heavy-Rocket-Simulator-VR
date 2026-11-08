Status: DONE_WITH_CONCERNS (open points 1-2).

# T7c fix round 1 - Sun beam rule refitted to the depth-aware shapes; cloud_check green (2026-11-08)

Logs, scripts, tables: `screenshots/look_fixes/t7c_fix1/` (scripts in `scripts/`: fit_beam.py, run_diag.sh, run_nadir.sh).

## Diagnosis (verified, partly corrected)

T7c's claim "shadows cover less than the rule" is right in effect, but the mechanism is not only the deflation:

- Slant table at HEAD (`slant_table_head.log`, cf 0.15/0.3/0.4/0.6 x e 75-5 deg, 8 sites): the shapes' shadows give an
  effective exponent n = 0.60-0.65 for e >= 30 deg at every f (old rule: 1 + 0.66 (1 - f) = 1.26-1.56), rising to
  ~2.3 at 5 deg. So C_eff ~ 0.66-0.81 f at the zenith (cf 0.3, Sun 89: shadow 0.211).
- Parts, measured with a diagnostic copy of cloud_check (removed again):
  - depth cover off: zenith shadow 0.197/0.338/0.550 (f 0.15/0.3/0.6) = ~0.70 x the shapes' analytic union of
    cloud_far_check 1e - the same ratio with and without the deflation: the deflation itself is right;
  - cumulus spread off (far_subgrid false): 0.211 -> 0.229 (Jensen, physical);
  - level-0 cells 50 m instead of 100 m: 0.211 -> 0.229, 0.229 -> 0.252 without spread (voxel reconstruction loss);
  - other site groups (8 sites each): 0.211 / 0.218 / 0.243 (sampling);
  - detail levels on: 0.142 / 0.242 / 0.446 (f 0.15/0.3/0.6) - darker than with them off.
  Remainder: thin margins (1e: mean opacity 0.92-0.95 of the union) and the slab profile's tapered rims (estimate).
- In-cloud transmission is not the cause: in-cloud depth 7.5-20, T_in/f nearly constant across f.

## Change

- Core: `cloud_beam_cover` = 1 - (1 - f)^n, n = 0.62 + 0.018 cot(e)^1.85 (`constants.hpp cloud_sun_beam_zenith_exponent
  / _slant / _slant_power`, [E], least-squares fit to the table: rms 0.006, worst 0.013 in T; fit_beam.py). Old form
  (linear onset) refits to rms 0.018 / worst 0.037 - rejected.
- `cloud_overlap_*` kept, re-documented as the renderer's view projected-cover rule (far field, flat levels: only the
  slant growth since T7c) - not touched, so T7c's tuned hand-overs stay.
- Shader: `CLOUD_SUN_BEAM_*`, `cloud_sun_beam_cover_of`, `cloud_column_sun_cover`; `cloud_column_direct` and
  `cloud_sun_diffuse` (all Sun users: ground light far tier, air shadow, fill's analytic column) use it; the view keeps
  `cloud_column_beam_cover`. Bindings: the three constants in `field_constants` (smoke compares 8 now).
- cloud_check: tau-1 slab tolerance 10 -> 12 % (0.898 at Sun 20 deg; 0.927 with volume_depth_cover off: the rim
  heights drawn at their depth-aware fraction); vehicle line centred on the darkest near-map point of a 16 km scan (the
  fixed line lay in one gap; probe has 64 keys max, so the line stays 41 points); moonlight check: compares with the
  keyless light in the Moon's colour (failing since 04bd4d24, moon_light_colour tint) and takes the keyed value from
  the settled `_vehicle_light` read (the key held the bright point before).
- Core test "The beam's cover follows the fitted shadows of the drawn shapes" (4 table points, margin 0.02); broken-cloud
  test now `beam < cover`.
- Docs: weather-clouds.md (light paragraph, validation, S-depth, open point replaced), history entry.

## Origin of the 5 pre-T7c failures

- 4 x "shapes vs CPU" cf 0.3 / 0.4: T7b 01397b72 (cumulus spread lightens the shadows; T7b state 0.641 vs 0.592 at cf
  0.3 / 45 deg, the spread worth ~8 % of the shadow, measured at FSD 1; inferred, T7 bb9cb929 not bisected).
- moonlight: 04bd4d24 (eye adaptation: moon light tinted), a test defect.

## Tests

- core `[weather]` incl. slow: 116/116 pass (build.ps1 -TestFilter "[weather]", core + bindings rebuilt).
- cloud_check: 63 samples PASS (was 14 FAIL). Disc means GPU / CPU: cf 0.15 45/8 deg 0.904/0.902, 0.809/0.811; cf 0.3
  0.794/0.795, 0.632/0.627; cf 0.4 0.725/0.722, 0.518/0.515. Vehicle: T 0.000 / 1.000, contrast 1.00.
- cloud_far_check full PASS (43 samples), cloud_layer_test PASS, atmosphere_check PASS, smoke PASS.

## Open points

1. The drawn nadir cover of a broken slab is below the data's on level 0 (voxel loss ~8-10 %, rims); a level-0 cover
   gain would bring it to f - a look change (more cloud near the camera), not made; then refit (STARSIM_SLANT_TABLE=1 +
   fit_beam.py).
2. Fit with detail levels off (the check's config); with them on (game) shadows are darker at low f (0.94 f vs 0.66 f
   at cf 0.15) - the core's light is then lighter than the drawn shadows there.
3. The view rule (`cloud_overlap_*`) still carries the undeflated shapes' vertical term for its fraction inversion.
