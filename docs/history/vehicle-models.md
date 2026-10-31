# Vehicle models: history

## Black shade sides (open point until 2026-10-31, fixed by the radiance refresh)

- Black shade sides in a static scene (user decision, lighting unchanged): the sky's radiance cubemap (ambient and reflections) stays
  black after the atmosphere tables load: `world.gd` uses `Sky.PROCESS_MODE_INCREMENTAL` and `sky.gdshader` reads only global uniforms,
  so Godot does not re-render it. In a real flight session a white diffuse hull's shade side reads `0 1 2`, after one sky-material write
  74-92, with REALTIME ~80. Not the hull shader, the camera or its height. An earlier probe seemed to refute this because its clock jump
  dirtied the cube; whether a running clock refreshes it in the real game is unchecked. Smallest fix: write a `radiance_serial` uniform on
  the sky material after the tables load and as the Sun moves; it brightens the shade side of every lit object. Evidence:
  `game/tests/shade_probe.gd`, `vehicle_look_shots.gd --poke / --sky_realtime`.

## Steel values until 2026-10-31

Steel: metallic 0.8, roughness 0.27 [E] (a diffuse share stands in for the sky's weak reflection); albedo sRGB 0.80 / 0.81 / 0.83, warm tint (1, 0.985, 0.96), mottle +-10 %. The diffuse share compensated the then black radiance cube.

## Analytic heat shield tiles until 2026-10-31

The tiles were computed per pixel in the shader (`heat_shield_tiles.gdshaderinc`): axial hex cells of (arc, station), the distance
to the cell edge box-filtered by the pixel footprint for the gap, a bevel normal faded out at 1-3 bevel widths, a per-tile hash for
the shade and the repaired tiles, all faded to the mean by the footprint. Shimmer: tiles ratio 1.081, error <= 0.124 (40 m), temporal
0.0001-0.010; cost vs the old steel in the belly close-up +0.024 / +0.087 ms (scale 1 / 2). Replaced by the baked maps (see the
hull shader section); the file is now `hull_noise.gdshaderinc` (hash and value noise of the steel only).

## Lowered dome and bells below the ship's base (finish pass, superseded 2026-10-31)
The ship's engines hung below its base (gimbal at station 0, exit planes 2.1 / 3.6 m below it, a closed base disc), so bells stuck into the booster
and looked like bumps in the dome. The finish pass moved the dome down instead (start 4.4 m below the top, 2.25 m high, apex 70.15 m), which only
moved the symptom. Replaced by the true arrangement: "Stage interface" in the subsystem file.
