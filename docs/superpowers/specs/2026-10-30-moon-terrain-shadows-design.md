# Moon terrain shadows (design, 2026-10-30)

Approved by the user in chat 2026-10-30 ("ja, setz den Plan auf") after the diagnosis below.

## Why
VR test (user 2026-10-30): beyond the lunar terminator, spots are lit although the Sun has already set. Cause:
`terrain_moon.gdshader` lights a fragment by its relief normal only (Lommel-Seeliger, `mu0 = n . to_sun`); nothing
occludes the Sun. A slope tilted a few degrees toward a Sun just below the horizon is fully lit; crater rims and
mountains cast no shadows anywhere (known limit in docs/subsystems/terrain-rendering.md). Render:
`screenshots/moon_terminator/fq_lon-2_east.png` (2024-04-15 19:00 UTC, lat 0 lon -2, 15 km, azimuth 90).

## Goal
Physically correct terrain occlusion of the Sun (and the earthshine) on the Moon at every scale the terrain mesh
resolves: lit peaks beyond the terminator stay ("terminator stars" are real), everything the terrain or the Moon's
curvature hides goes dark; long low-Sun shadows at the poles. Soft edge from the solar disc (0.53 deg).

## Decisions (user-approved approach)
- **Horizon maps baked per terrain node on the workers**: for every grid vertex the terrain horizon elevation angle
  in N_AZ = 16 azimuth directions (local east/north tangent frame), curvature included, from the core's lunar heights
  (LDEM_16 + 20 m site tiles - the physics' surface). Computed in the core (C++, `environment`), exposed through the
  bindings; the node builder stores them in a per-node texture.
- **Shader**: horizon in the Sun's azimuth (interpolated between sectors and bilinear between vertices) against the
  Sun's elevation -> visible fraction of the solar disc (disc segment above a straight horizon). Multiplies the direct
  Sun term (and the umbra glow); the earthshine term uses the Earth's direction the same way.
- No rebuild over time: all azimuths are baked, the Sun's motion (incl. time-lapse) needs nothing.
- Vehicles and the eye on / near the Moon: the direct Sun takes the same terrain horizon (single-point full-resolution
  query in the core) instead of the mean-sphere fraction near the ground - uniform with the terrain (as the clouds'
  "shadows fit the rest" ruling).
- Earth terrain: unchanged (out of scope).
- Rejected: per-pixel GPU ray-march over a height texture (LDEM_16 only 1.9 km, misses the site tiles, per-pixel VR
  cost).

## Limits (accepted)
Shadow edges are as fine as the node grid (10 m near the camera at leaf level, coarser far away). Vehicles cast no
shadow on the terrain and the terrain no shadow on parts of a vehicle (whole-vehicle factor only) - later.

## Budgets
- VR frame time on the Moon unchanged within +-2 % (one texture fetch pair per fragment).
- Node build time on the workers: measured and reported; the horizon bake must keep the lunar loading screen and the
  descent streaming without visible holes (state numbers before / after).
- Horizon accuracy against a brute-force fine march: <= 0.1 deg below 5 deg elevation.
