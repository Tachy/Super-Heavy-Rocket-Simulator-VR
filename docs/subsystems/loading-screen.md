# Startup loading screen

Current state (2026-10-30). Shows which boot step runs right now, in German, light text on black, no progress bar.

## Code
- `game/ui/loading_steps.gd` (`LoadingSteps`): the data. Step list with start ticks, the running step, the last
  `HISTORY_MAX` = 4 finished steps, durations, the end-of-boot table. Pure logic, unit-tested.
- `game/ui/loading_view.gd` (`LoadingView`): the picture (title top, current step 64 px, history 32 px dimmed). Same view
  for both targets.
- `game/ui/loading_screen.gd` (`LoadingScreen`): the reporter and the lifecycle. `Main` creates it (`start`), the boot
  code calls `await LoadingScreen.step("...")` (a no-op without a screen), `Main` arms it with `finish_when_idle`.
- `game/xr/vr_loading_layer.gd` (`VrLoadingLayer`): `OpenXRCompositionLayerQuad` (2 m wide, 2 m in front of the head
  pose of the first visible frame, upright, sort order 1) fed by a 1920x1080 SubViewport with the view.
- Boot steps live in `World._ready` / `_setup_bodies` (`world.gd`) and `Main._ready` (`main.gd`).

## Behaviour (standing decisions)
- No screen (`LoadingScreen.start` returns null) for headless, `--script` runs (all tests), `--self-check`,
  `--no-loading-screen`, and the test harness' stand-in headset. Tests that load `main.tscn` stay synchronous.
- Desktop: CanvasLayer 100 on black; each step is drawn at once with `RenderingServer.force_draw()`, the boot stays
  synchronous. The root viewport's 3-D is off meanwhile (the half-built world and its compositor effects never render).
- VR: only rendered frames reach the headset, so in VR (`LoadingScreen.yielding()`) every step waits a frame after its
  text is set: `World._ready` suspends between steps, `Main._ready` awaits `World.boot_finished`, `World._process`
  waits for `booted`. The XR camera is blanked (cull mask 0, black environment, empty compositor) until the world is up.
- The screen stays while `atmosphere.loading` (its status text becomes the step; first run precomputes about a minute),
  then `READY_FRAMES` = 6 rendered world frames (covers the 4-frame cloud shader warm-up), then fades (desktop, 0.3 s)
  or is removed (VR).
- Every step is logged: `[Boot] +s  text`, and a table with durations when it ends.

## Boot steps (desktop, RTX 4090, warm cache, 2026-10-30; time from engine start)
Engine/GPU start before the first step: about 21-22 s (D3D12 and project start, not reportable from script; headless
the whole start is 5 s). Then:

| Step | Duration |
|---|---|
| SPICE-Kernel laden | 0.03 s |
| Startplätze und Fahrzeugdaten laden | 0.55 s |
| Himmel und Atmosphäre vorbereiten | 0.01 s |
| Wetterdienst und Wolken einrichten | 0.01 s |
| Erdtextur laden | 0.29 s |
| Erdgelände und Satellitenbilder einrichten | 0.71 s |
| Startplätze und Fangtürme aufbauen | 0.03 s |
| Mond und Mondgelände aufbauen | 0.62 s |
| Wolken-Shader vorwärmen, Sternkatalog laden | 0.17 s |
| Oberfläche aufbauen | 0.05 s |
| Atmosphäre wird geladen (background, until the tables are up) | 0.16 s here |

## Open points
- VR is unverified on the headset: whether the Pimax runtime keeps showing the last quad frame during a blocking
  step of 0.5-0.7 s (no frames submitted), whether `session_visible` arrives early enough, the quad's size and distance.
- Weather downloads and the cloud bake run after the boot and are not steps of this screen.
