# Vehicle models: history

## Black shade sides (open point until 2026-10-31, fixed by the radiance refresh)

- Black shade sides in a static scene (user decision, lighting unchanged): the sky's radiance cubemap (ambient and reflections) stays
  black after the atmosphere tables load: `world.gd` uses `Sky.PROCESS_MODE_INCREMENTAL` and `sky.gdshader` reads only global uniforms,
  so Godot does not re-render it. In a real flight session a white diffuse hull's shade side reads `0 1 2`, after one sky-material write
  74-92, with REALTIME ~80. Not the hull shader, the camera or its height. An earlier probe seemed to refute this because its clock jump
  dirtied the cube; whether a running clock refreshes it in the real game is unchecked. Smallest fix: write a `radiance_serial` uniform on
  the sky material after the tables load and as the Sun moves; it brightens the shade side of every lit object. Evidence:
  `game/tests/shade_probe.gd`, `vehicle_look_shots.gd --poke / --sky_realtime`.
