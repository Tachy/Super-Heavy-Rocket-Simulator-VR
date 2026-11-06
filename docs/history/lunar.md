# Lunar missions: history

Superseded states of `docs/subsystems/lunar.md` (older lunar history: `docs/history/guidance-missions.md`).

## Polar LLO only (until the lunar sites plan Task 7, 2026-11-06)
- `mission::LloTarget{altitude_m, inclination_rad = pi / 2}`: every LLO mission targeted a polar plane through the
  arrival asymptote (`[lunar_orbit] inclination_deg = 90` in `hls_lunar_llo` / `hls_lunar_llo_depot`); the TLI Newton
  held periselene altitude and inclination, the arrival time free (it drifted along a nearly flat family: the
  2026-11-13 Mons Mouton transfer arrived 19.1 h after the planned 4 d, TLI 3,240 m/s).
- Site pass: the descent waited for the first pass of the site within `max_crossrange_km` (20 km) - 5.4-5.9 d for
  Mons Mouton at any epoch (the polar plane follows the asymptote, fixed in the tidally locked Moon's frame); on
  2026-11-22 ~01:20 UTC the old plane passed the site 22.4 km off before the trim. Only near-polar sites could be
  reached in a reasonable wait; an equatorial site sits in a polar plane only twice a month.
- Ascent: `LunarAscentParameters::northerly` (true): the polar plane through the liftoff point, launched north,
  whatever plane the landing came from.
