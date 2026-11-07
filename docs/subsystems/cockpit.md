# Cockpit (M7): Crew Dragon cabin

Spec `docs/superpowers/specs/2026-11-07-m7-cockpit-crew-dragon-design.md`, plan
`docs/superpowers/plans/2026-11-07-m7-cockpit-crew-dragon.md`. State after plan Task 1 (measurement): typed tables only,
no game code or generator yet.

## Model
- `config/cockpits/crew_dragon.toml`: `[cabin]` (inner wall of revolution, floor, ceiling, forward tunnel, windows /
  hatch in the exterior table's convention, window reveals), `[[seat]]` s1..s4, `[eye]`, `[console]` (housing slab),
  `[[screen]]` left / centre / right, `[[tablet]]`, `[[light]]`, `[[button]]` (39, incl. the centre handle), `[hotas]`.
  Every value line is tagged [S] / [E] / [D] with its derivation (`tools/structures/detail_file.value_lines_without_tag`
  finds none untagged).
- Frame: capsule stage body frame of `vehicle-models.md` "Crew Dragon": X = station above the heat shield apex (nose
  +X), (Y, Z) = r (cos az, sin az), az 0 = side hatch. The crew face the hatch side and the console: Y toward the hatch,
  **Z = az +90 = the crew's left**. Seats s1..s4 from the crew's left (s1 at +Z), s2 = commander (player), s3 = pilot.
- Buttons: `panel` = the screen above (its normal / up apply), `row` 1 upper / 2 lower, `shape` round / rect,
  `kind` push / covered / handle, `function` only with evidence, else `"none"`; the 18 keys under the left screen
  (clusters A, B) are `camera` with `camera_slot` 1..18 (upper row left to right, then lower row; user decision).
  Known functions: `abort` (centre handle), `chute_mains_only`, `chute_drogues_and_mains`, `backup_g_meter`; nothing is
  wired in Task 1.
- `[hotas]`: deliberate deviation (real Dragon has none): T.16000M stick on the commander's right armrest, TWCS throttle
  on the left; grips as offsets from the eye point in capsule axes, `props = true`; calibrated in the VR run (Task 9).

### Provenance of the key numbers
| Quantity | Value | Tag | Derivation | Uncertainty |
|---|---|---|---|---|
| Side wall | 0.18 m normal | E | crew1_1 window wells 0.15-0.25 deep; window spacing in crew1_1 gives inner r 1.43 at x 2.38 (exterior 1.62) | +-0.05 |
| Inner profile | exterior `profile_m` offset by the wall | D | closest exterior point 0.094 m (>= 0.02 required) | as wall |
| Floor | x 1.15 | E | Wikimedia: cargo C5-C7 under the seat pans, 0.5 m below the hatch sill | +-0.15 |
| Ceiling / tunnel | x 3.35 / r 0.45 to x 3.66 | E / D | crew1_1, Wikimedia flat ceiling ring; IDSS 0.8 m passage + trim; hatch face 0.08 under the recess floor 3.74 | +-0.1 |
| Seat pitch | 0.70 m (Z +-0.35, +-1.05) | E | Wikimedia: pitch / seat-back width 1.2, back 0.58 m | +-0.05 |
| Seat hip (s2) | [1.80, -0.02, 0.35] | E | eye 0.70 m from the console (reach, crew2_1 / inspiration4_1); outer seats 0.08 lower (Wikimedia) | +-0.1 |
| Back angle | 35 deg from +X toward -Y | E | crew1_1 torso foreshortening from the hatch, knees above the hips | +-10 deg |
| Eye (s2) | [2.447, -0.351, 0.35] | D | hip + 0.72 up the back + 0.10 forward (seated eye 0.79 m above the pan) | +-0.10 per axis |
| Eye forward | 20 deg above the chest normal | E | crew10 / crew11 gaze to the screens | +-10 deg |
| Screens | 0.382 x 0.215 m (16:9) | E | training2 finger scale (index finger ~0.072 m on the glass) 0.35-0.40 wide; aspect 1.76-1.88 measured | +-15 % |
| Screen layout | centre on the centre line 0.70 m along the gaze, gap 0.045, side screens folded 12 deg | E / D | discovery10 (gap 90 / 780 px), training2 | +-0.1 m, +-5 deg |
| Console housing | 1.60 x 0.54 x 0.15 m | E / D | crew1_1 box 1.2-1.4 m + tablet wings (discovery10) | +-0.15 m |
| Tablets | 0.1348 x 0.2032 m | S / E | Apple iPad mini spec; model assumed from the shape | - |
| Button positions | rows 0.17 / 0.20 m below the screen centres | E | discovery10 x along the screens' lower edges (fisheye: fractions per screen) | +-0.02 along, +-0.03 across |
| HOTAS grips | 0.22 m forward, 0.44 m below the eye (seat axes), +-0.25 / 0.27 lateral | E | armrest, forearm reach | calibrated in Task 9 |

### Windows and hatch against the exterior table
- `[cabin] windows` / `hatch` are copies [D] of `config/vehicles/detail/crew_dragon.toml` (outer-surface convention);
  the cabin generator carries each aperture through the wall along the wall normal (x shifts -0.06 m on the inner face).
- Az +-33.3 windows: exterior photo `D_CCP_SpaceX_Demo_2_Dragon__3_` shows the oval glass beside the hatch, about
  0.30-0.37 x 0.48 m depending on the assumed camera distance (table 0.31 x 0.51); inside (`dragon_cockpit_crew11`,
  `11b`) a round reveal ahead of and inboard of an outer seat, below the console: same place, size not measurable.
- Az +-90 windows: `crew1_1` (camera in the hatch, helmet-scaled) puts them at az 84-90 (+-10 deg), centre at the outer
  crews' eye height (x ~2.37 vs 2.385); the dark interior opening measures about 0.46 (x) x 0.50 (tangential) m
  (+-25 %). Larger than the 0.31 m wide glass: read as a flared round reveal (`window_reveal_radius_m` 0.24), not as a
  conflict with the exterior pane.
- Hatch: interior face seen only in part under the console (discovery views), no scale; consistent with az 0, x
  1.70-2.99.
- Gate (ruling E): no measured interior number contradicts the exterior table beyond its uncertainty, so no proposal and
  the exterior file is unchanged. The photos resolve positions to about +-10 deg / +-0.1 m and sizes to +-20-25 %; the
  10 % gate (2.5 cm, 0.55 deg on the windows) is finer than the evidence: "not contradicted", not "confirmed to 10 %".

## Code
- `game/cockpit/cockpit_config.gd` (`CockpitConfig`): `for_vehicle(id)` (cached; `+` stack ids use the first part whose
  `config/vehicles/<part>.toml` `[vehicle] cockpit = "<id>"` names a cabin; the core ignores that key), properties
  `id`, `capsule_stage_id` (`[cockpit] capsule_stage`), `eye` (Transform3D, -Z = gaze), `seats`, `screens`, `buttons`,
  `cameras` (empty until the camera task), `hotas`, `hotas_props_enabled`, plus `cabin`, `console`, `tablets`, `lights`,
  `validate()` and `validate_crew(count)`. `MissionCatalogue.crew_count(mission_id)` reads `[crew] count` (4 in the five
  Crew Dragon missions; missing = the cockpit's seat count, else 0). `TomlLite` reads one-line arrays of arrays.
- Task 3: `tools/structures/blender/gen_dragon_cabin.py` (not yet).

## Validation
- Wall check (scratch script, Task 1): inner profile >= 0.094 m from the exterior meridian and the neck well everywhere.
- Tag check: `detail_file.value_lines_without_tag("config/cockpits/crew_dragon.toml")` empty; unit vectors normalised.

## Open points
- Cabin volume: our trim surface above the floor encloses 15.2 m3, SpaceX gives 9.3 m3 pressurized [S:
  spacex.com/vehicles/dragon]. The photo-measured width (four seats, windows at r ~1.43) does not leave room for a
  9.3 m3 vessel of this height; either the published figure is a free / habitable volume or the floor / ceiling sit
  higher or lower than read. Re-check against a dimensioned source if one appears.
- Screen size has no published value; 0.382 x 0.215 m rests on one finger scale (+-15 %).
- Button labels on the flight panel are unreadable at video resolution; functions beyond the four named stay `none`.
  Candidates from reports (fire suppression, cabin pressure, trunk jettison, deorbit) are not tied to a button.
- Seat poses: one launch / entry recline (35 deg) for all seats; the real seats reportedly tilt between phases (not
  measured). Outer seats may be toed in (crew1_1); not modelled.
- Console struts, lockers in the lower corners (Wikimedia "17", "18"), the small display left of the handle and the
  hatch's inner mechanism are not in the tables (surface detail or later).
