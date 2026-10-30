# Startup loading screen - history

Current state: `docs/subsystems/loading-screen.md`.

## 2026-10-30: VR stuck on "Wolken-Shader übersetzen" after the boot (fix 1)
Symptom (user, Pimax OpenXR): the screen came at once and showed the steps, but after them the headset showed
"Wolken-Shader übersetzen ..." again and stayed on it; the world never appeared. The log said `[Boot] done`.

Cause: the boot scene reparented its screen to the root (`screen.reparent(root)`); the reparent left the tree for a
moment and `LoadingScreen._exit_tree` cleared `LoadingScreen.current`. Main's `LoadingScreen.start` therefore built a
second screen (log: a second `VrLoadingLayer.setup` backtrace from `main.gd:20`, the boot table starting at "SPICE",
no "Hauptszene laden"). Effects:
- The second quad blanked the already blank XR camera and saved those blank values; its `release()` "restored" cull
  mask 0, black environment, empty compositor - the world stayed invisible.
- The first quad (still showing its last text, "Wolken-Shader übersetzen") was never armed nor freed; when the second
  quad was freed after "done", the first one became visible again.
- The second screen's first step waited the full `SESSION_WAIT_MS` = 4 s for a `session_visible` signal that had come
  long before (SPICE step 4.07 s in VR, 0.03 s on the desktop).
The desktop took the same path (two screens; the first one's CanvasLayer 100 stayed in the root) - not checked on a picture.

Fix: `current` cleared only on finish / PREDELETE; the session wait also reads `get_session_state()`; `remove()` on the
layer (camera restored, SubViewport detached, hidden, freed, all logged); an armed timeout of 180 s; the headless VR
lifecycle test in `loading_steps_test.gd`. The quad's properties are now set after it is in the tree (the
`!is_inside_tree()` error in `setup`).

## The blocking cloud shader build (superseded 2026-10-30: kernels built on workers)

Measured split of the former 21.5 s before the first step (ticks from engine start): driver/renderer/project load and
all autoloads 0.9 s; `main.tscn` load 0.7 s; instantiating Main 20.1 s, almost all of it `World`'s member
`CloudLayer.new()` (the `CloudEffect` / `CloudShadowEffect` compute shaders compiled at construction on the render
thread; consistent over runs); first frame right after. Now the Boot scene shows its first text at 0.77 s. The 20 s
stay a blocking call on the main thread (a static screen; moving them off-thread needs a change in the cloud code). Then:

Open point then:
- The ~20 s cloud shader build blocks the main thread: in VR no frame is submitted meanwhile; verified on the headset
  (user, 2026-10-30, commit 614f6c4): the Pimax runtime keeps the quad stable in the room, free 6DOF head movement, no
  freeze. Fix (to shorten it) would be compiling the cloud compute shaders
  lazily / threaded in `CloudEffect` (weather-clouds subsystem).
