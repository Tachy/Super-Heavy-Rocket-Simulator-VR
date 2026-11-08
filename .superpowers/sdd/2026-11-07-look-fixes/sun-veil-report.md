# Sun veil like Moon and Earth (2026-11-08)

Status: DONE. User decision "Sonne wie Mond und Erde".

## Change
- `Atmosphere.sun_disc_scene(scene_per_cd)`: the Sun disc's mean luminance in scene units (beam illuminance / solid angle x scene per cd/m^2).
- The Sun's analytic veil (glare_spread_effect.gd `_add`) is shown at its disc's local factor `GlareSpread.source_factor(Y_disc, 0)` = PLATEAU / Y_disc (no floor; was 1/g ~ 1): halo = CIE 146 x Omega_Sun of the disc, like Moon / Earth. The meter keeps the full veil; the sky shader's aureole, the exact Sun counters (flicker fix) are unchanged.
- Found on the way: the spread's Sun mask used the Sun from the Earth's centre; from lunar orbit the 0.15 deg parallax left the disc rim unmasked (white ball of 1.4 deg). Now `sun_apparent - camera`.

## Numbers (8-bit sRGB luma of the ring mean at 1 / 2 / 3 / 5 / 10 deg; A before, B after)
| scene | A | B |
|---|---|---|
| Sun in space 400 km, 60 deg | 255 / 255 / 251 / 211 / 128 | 3.1 / 0.5 / 0.1 / 0.0 / 0.0 |
| Sun in space 400 km, 10 deg | 255 / 255 / 251 / 212 / 136 | 3.1 / 0.5 / 0.1 / 0.0 / 0.0 |
| ground noon (weather off: cumulus shadow), 60 deg | 255 / 255 / 253 / 237 / 203 | 225 / 224 / 221 / 214 / 191 (sky aureole) |
| ground noon, 10 deg | 255 / 255 / 253 / 237 / 207 | 225 / 224 / 221 / 214 / 194 |
| lunar orbit 100 km, 60 / 10 deg | white ball 1.4 deg | 3.1 / 0.5 / 0.0 / 0.0 |
Physical: CIE 146 x 6.8e-5 sr = 1.0e-3 / 1.7e-4 of the disc at 1 / 2 deg = luma 3.3 / 0.6 (matches). White pixels (60 deg): 5600 -> 52 (the real disc).

## Tests
glare_spread_test (new Sun factor / halo ratio cases), glare_veil_test, local_adaptation_test, adaptation_meter_test: PASS. smoke_test: see reply. `--halo_sequence=40 --halo_drift=0.013` ground 60 deg: worst step 0.03 % (limit 2 %) PASS. Lunar orbit halo_sequence reports 48 % relative step only because the rings are ~1e-5 linear (8-bit quantisation); ring 1-2.5 is constant at 0.00018.
Suite (`tools/look_reference_suite.py`): Sun scenes measure the profile (`sun_measure`); gates: space / lunar orbit luma_1deg 1.6-6.6, 2deg 0.2-1.3, 5/10deg <= 0.5 [S: CIE 146 x Omega_Sun]; ground luma_1deg <= 245 (not a clipped ball). All Sun scenes pass (sun_in_view, _vr, _d60, _f10, space f60/f10, lunar orbit f60/f10, profile f10).

## Images (screenshots/look_fixes/sun_veil/)
`sheet_sun_space_400km_f60.png`, `_f10`, `sheet_sun_in_view_d60.png`, `_f10` (A | B), crops `A_/B_crop_*`, `B_sun_in_view_vr.png`, `B_sun_lunar_orbit_f60/f10.png`.

## Open
- Ground scene uses weather off (the analog weather puts the pad in a cumulus' shadow at noon); the aureole there is the sky shader's (unchanged, ~3x KS91 at the Moon - open point from fix 1).
- Not verified on a headset.
