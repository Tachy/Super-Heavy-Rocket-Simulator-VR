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

## Task 7 first state (2026-11-06, superseded by its fix round 1)

- Planned touchdown key `[descent] touchdown_utc` (a UTC date, applied as an offset from `[orbit] epoch_utc`);
  renamed `touchdown_after_epoch_h` (a duration) because the use was relative.
- The start checked the touchdown point before the outbound plan, for polar approaches (0 / 180 deg) only.
- NRHO: the budget refusal carried `lunar.nrho_site_unreachable` but its legs were fixed [D] numbers (the same for
  every site).
- A site on the arrival axis took the plane through it of least inclination even with an arrival plane given.
- Plan time noted as "whole mission start LLO ~7 s, NRHO ~8 s, aircatch ~4.5 s" (return starts, before the l6 timing
  work was re-measured).

## Task 8 first state (2026-11-07, superseded by its fix round 1)

- Window search: a candidate whose arrival drifted past its planned pass (the descent then took a pass days later,
  Apollo 11 candidate +9.96 d -> +14.0 d) was dropped and counted as `lunar.no_daylight_for_stay`; now the start is
  re-solved by the drift.
- The budget had no leg for the Moon's rotation over the stay (the listed reserve ended after the ascent).
- NRHO: every candidate ran the chain with its own NRHO reference - a full refusal took 488 s (165 candidates,
  ~2.2-3 s each); now decided once by the budget lower bound.
- The search kept references into the prototype session and read its surface flags per candidate; every complete
  result was cached, also refusals from a missing site tile.

## Site albedo low-pass 10 km (2026-11-07, superseded by ruling R21)

- The albedo tile was divided by its own 10 km box low-pass. The global albedo map the shader multiplies it with
  (`moon_color.png`, 2.67 km/px) already holds the 2.7-10 km band, so that band entered twice: at Apollo 17 the km-scale
  log contrast rose from 0.287 (global) to 0.377 (x1.09 at 1 sigma, +31 % in std). Now the low-pass has the footprint of
  one global pixel (2665 m) and the albedo cache key carries `_lp2665`.

## Touchdown point before the preferred slope (to 2026-11-07)
- `select_touchdown_point` took the nearest admissible point (slope <= 8 deg, roughness, glide cone, hazard check). On 6-8 deg the HLS rocked 1.5-2.4 m off after the engine cut (Apollo 11 nightly 4/7, Mons Mouton seed 14); the user chose the touchdown-point choice over a lower hard limit or a new terminal procedure (2026-11-07), so the flat touchdown point plan added the 5 deg preference.
