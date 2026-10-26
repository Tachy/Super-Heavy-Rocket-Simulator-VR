# Atmosphere: GPU checks, render values and performance (M8)

History and the reasons behind superseded decisions: `docs/history/atmosphere-rendering.md`.

Plan `docs/superpowers/archive/plans/2026-10-24-m8-atmosphere.md`, spec
`docs/superpowers/archive/specs/2026-10-24-m8-atmosphere-design.md`.
Part of the atmosphere docs; index `docs/subsystems/atmosphere-rendering.md`.

## Validation

- GPU `game/tests/atmosphere_check.gd` (HDR viewport, float readback, 2 % per channel unless stated; RGBAH and
  `--f32`): 86 samples - sky, sunset, limb from 400 km and from the Moon distance, the pad camera's float64 altitude
  path, ground and terrain (3 %), stars / Sun / Moon, lit cubes (5 %), Sun colour, eye adaptation, eclipse, adaptation
  away from the Earth, earthshine, night vision (section 13, 18 samples); the run asserts its sample count
  (`EXPECTED_SAMPLES`: a GDScript error in an awaited sample ends it silently); worst sky deviation 0.52 % (50 km limb
  from the Moon distance, float32 rounding at 3.8e8 m), Earth mesh samples -2.1 / -1.7 % (facets up to ~0.5 km below
  the ellipsoid; 0.15 % against the point 300 m lower).
- GPU check specifics: day frames = fixed exposure x gain (0.02 %), gain = Weber gain within 1e-9 (noon, 400 km); Sun
  -8 deg zenith blue > 0.004 (0.0065); moonless starlit ground quad (0.002 lux x albedo 0.2 / pi = 1.27e-4 cd/m^2)
  renders as the cone + rod image (0.2 %), pixel luminance 0.019 (0.0068 with B 0.368); full-Moon ground cone image =
  the CPU's moonlit surface within 1 %, rods 78 % of its luminance; stars (magnitude 6 moonless, 2 / 5 under the full
  Moon); earthshine full Earth 7.46 lux (band 5-15 lux [call]), Moon dark side at the sub-Earth point = albedo x
  moon_earthshine x R x f(0) (0.6925) within 3 % at full Earth and phase 60 deg; the -3 deg zenith 0.726 x the noon
  zenith on screen, displayed / scene chroma 0.675 against noon's 1.000; order noon 0.148 > -3 deg 0.108 > -8 deg
  0.0031 > -12 deg 0.0015; 2026-11-12 crescent glow 16 % above the sky beside it; limb glow on the game's path (>0),
  adapted totality takes the glow (> 1.5 x the zenith), twilight eye off the shadow on the eclipse day keeps its glow
  (90.8 cd/m^2).
- Render values (adapted, snapped; B / S: OLP-1 Sun -3 deg 0.377 / 0.675, -8 and below 0.368 / 0.521, totality
  0.368 / 0.553, orbit sunset 0.368 / 0.603, full Moon 0.368 / 0.521): pad noon 1.10 (OLP-1 noon view 1.123); Sun -3 deg
  L_a 90.8, gain 464; Sun -8 deg L_a 1.44, gain 5324, rod scale 0.025, zenith (0.0036, 0.0083, 0.025); Sun -12 deg
  gain 11 235, rod scale 5.5; moonless night L_a at the 1e-6 floor, gain 12 368, rod scale 138, magnitude-6 star
  visible; full Moon at the Cape (2024-10-17 05:00, phase ~3 deg, moonlight 2.84e-6 of the Sun): L_a 0.12, gain 9977,
  rod scale 1.35, ground 0.019 with 78 % from the rods, magnitude 2 visible, 5 hidden; lunar night 2845; totality at
  Dallas (2 m above the terrain at 107 m): L_a 8.24 cd/m^2, gain 2438, rods 0 (GPU check eye on the ellipsoid: 7.17,
  gain 2624); 400 km in the umbra 17.7; 400 km 60 s after orbital sunset: limb glow 109 cd/m^2, gain 389, sunward limb
  peak 2.73; Cape Sun -26.6 deg: glow 23.3, gain 1325; orbit night under the full Moon, 400 km: L_a 0.168, gain 9429.
- Performance (`game/tests/terrain_perf.gd`, RTX 4090, 2560 x 1440, gate p99 <= 8 / max <= 16 ms without clouds, 11 / 16 with them - `weather-clouds.md` "Performance gate"): default epoch p99
  7.75 / 7.90 / 7.90, max 10.7 / 12.5 / 11.7 ms (GPU <= ~3 ms; Cape p99 ~9.1 while ~240 nodes load); full-Moon night
  (--utc=2024-10-17T05:00:00) p99 7.88 / max 14.3 ms PASS; eclipse epoch (2024-04-08 18:42, Starbase 80 % partial)
  p99 8.7 ms, max 12-13 ms, GPU-bound. Sky cubemap incremental at 128, half-float scattering tables.


## Diagnostics

- GPU check: `godot --path game --script res://tests/atmosphere_check.gd [-- --f32]` (needs a GPU).
- `build/core/atmosphere_tables.exe` (~62 s) builds the table cache; or the first game start.
- `res://tests/render_check.gd` (`--utc= --focus= --view_from= --look_at_sun --no_refraction`): `--focus=earth
  --view_from=moon` is the orbit camera 22 000 km from the Earth's centre on the Moon side, not a lunar eye;
  render_check
  snaps the adaptation after the view is placed. In the game every view change (`set_focus`, `set_ground_site`, Tab,
  mission start) snaps before the next `update_world`. Ground views stand on the core's surface.
- `game/tests/terrain_perf.gd` prints its per-second lines at the end of the run (the prints themselves stalled the
  main thread by up to ~22 ms).


## Open points and known limits

- Rarer draw stall: ~1 `terrain_perf.gd` run in 3 shows one 11-17 ms frame (once 33.7 ms) at a fixed instant of the
  descent (t = 3.06 s or 10.89 s), inside the RenderingServer draw outside the viewport render, while a batch of 4
  terrain builds runs on the worker pool. `threading/worker_pool/max_threads = 8` removed it in 9 of 9 runs but limits
  terrain to 2 concurrent builds (Cape loads in > 20 s instead of ~8 s); 10 threads with 7 low-priority slots kept it
  (3 of 8), 24 threads with 2 slots made it worse (6 of 8, 23-31 ms). Not fixed (engine-internal; open).
- Eclipse epoch: the per-vertex sky-light eclipse factor costs ~3.5 ms (9 directions x 4 segments on ~8 M terrain
  vertices); adaptive segment counts gave no gain; a shadow-plane texture updated per simulated second would remove it
  (open).
- Brightness term performance: the default-epoch run with the VR game beside it on the same GPU (99 % load) gave p99
  47.1 / max 56.3 ms (FAIL), not a measurement of the change; to be rerun on an idle GPU. Night-vision default-epoch
  runs gave max 24.5 ms (the known stall) and on rerun p99 7.67 / max 13.3 ms (PASS).
