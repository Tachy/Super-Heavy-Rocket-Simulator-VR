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
