# T6 report - F1: DAY_EXPOSURE out of the field rule (2026-11-08)

## What
`Atmosphere.rule_key()` = `field_key * field_day_exposure / DAY_EXPOSURE`; the field rule (`field_adaptation_luminance`)
gets that key (0.12 at field_key 0.18), so the metered field renders at 0.18 instead of 0.27. The anchor (white-plate)
term is untouched. `field_day_exposure` defaults to 1.0; user argument `--day_exposure=1.5` restores the earlier rule
(render_check and every script that reads the user args). Test: `adaptation_meter_test.gd` (rule_key default / old).
Docs: `docs/subsystems/eye-adaptation.md` (rule D paragraph), render_check header.

## Numbers (A = `--day_exposure=1.5`, B = new default; same camera / time; `screenshots/look_fixes/t6/`)
| scene | gain A -> B | ground mean sRGB A -> B |
|---|---|---|
| pad noon el 5 | 1.650 -> 1.100 | 156 -> 136 |
| lunar day, Apollo 11 down-Sun el 5 | 4.937 -> 3.291 | 128 -> 108 (Y 0.242 -> 0.171) |
| lunar night, Earth (0 N 45 E) | 8342 -> 7557 (anchor decides) | 18 -> 13 |
| dusk (Sun -5.5 deg) | 1996 -> 1706 | 32 -> 29 |
| cockpit (anchor decides) | identical (pilot_eye 157.39, hatch 82.1) | - |
| Earth from space, EPIC 14:28 | identical, gain 1.000 both | disc stats identical |

Grey-term cases are -0.58 EV (x1/1.5) as predicted.

## Key finding: the Earth from space is NOT changed
The space view sits at the gain floor (cone_gain clamped >= 1: field 7850 cd/m^2 -> L_a 43.6k old / 65.4k new, both above
the white plate L_ref 26k). So F1 does nothing for the "Earth too white" complaint; the disc stats vs EPIC are identical
(dark/grey/white 0.078/0.478/0.444 against EPIC 0.374/0.497/0.129, corr 0.491). Lowering that would need the gain floor
(exposure below the TOA-Sun white plate) or the tone curve (T7 step 2) - user decision.

## Moon
Lunar day goes from sRGB 128 to 108 on the regolith (Y 0.24 -> 0.17), -0.58 EV; still clearly lit but visibly darker
than before. User decision needed: keep, or compensate for the Moon (e.g. `--day_exposure=1.5` default for body=moon or a
Moon key). Lunar night with Earth barely changes.

## Tests
adaptation_meter_test PASS, white_balance_test PASS, sunset_adaptation_test PASS; atmosphere_check PASS (88 samples, 0 failures; needs a GPU, headless hangs), smoke test exit 0.

## Images
`screenshots/look_fixes/t6/sheet_<epic|pad_e5|moon_e5|moon_earth|dusk>.png` (A left, B right), `cockpit_A/`, `cockpit_B/`,
logs `A_*.log`, `B_*.log`, `run.sh` (V=A|B).
