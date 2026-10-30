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
- `game/boot.gd` + `boot.tscn` (`Boot`): the project's start scene (`run/main_scene`). Starts the screen first, loads
  `main.tscn` with `ResourceLoader.load_threaded_request` (polled per frame), reports "Hauptszene laden" and
  "Wolken-Shader übersetzen ...", instantiates Main, reparents the screen to the root and switches
  (`Boot.switch_to`, deferred add). Without a screen (headless, `--self-check`) it loads synchronously and switches.
  `LoadingScreen.start` returns the existing screen when Main calls it. Tests (`--script`, `load("res://main.tscn")`)
  never pass through it.
- `game/assets/boot_splash.png`: the engine's boot splash (black, name, "wird gestartet ..."), made by
  `tools/godot/make_boot_splash.gd` (GPU run); project settings `boot_splash/image`, `show_image`, `fullsize=false`.
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
Measured split of the former 21.5 s before the first step (ticks from engine start): driver/renderer/project load and
all autoloads 0.9 s; `main.tscn` load 0.7 s; instantiating Main 20.1 s, almost all of it `World`'s member
`CloudLayer.new()` (the `CloudEffect` / `CloudShadowEffect` compute shaders compiled at construction on the render
thread; consistent over runs); first frame right after. Now the Boot scene shows its first text at 0.77 s. The 20 s
stay a blocking call on the main thread (a static screen; moving them off-thread needs a change in the cloud code). Then:

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
- The ~20 s cloud shader build blocks the main thread: in VR no frame is submitted meanwhile (whether the runtime keeps
  showing the last quad frame is unverified; VR was not measured). Fix would be compiling the cloud compute shaders
  lazily / threaded in `CloudEffect` (weather-clouds subsystem).
- VR is unverified on the headset: whether the Pimax runtime keeps showing the last quad frame during a blocking
  step of 0.5-0.7 s (no frames submitted), whether `session_visible` arrives early enough, the quad's size and distance.
- Weather downloads and the cloud bake run after the boot and are not steps of this screen.
