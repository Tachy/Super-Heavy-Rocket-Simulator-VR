# World-fixed cloud view cache - design

Date: 2026-10-27. Status: draft for the user's approval. Follows the static clouds plan
(`docs/superpowers/plans/2026-10-26-static-clouds.md`, its VR run of 2026-10-27) and changes the cloud pass of the M8
clouds (`docs/subsystems/weather-clouds.md`, `docs/subsystems/vr-performance.md`).

## Why

The VR run of 2026-10-27 (Pimax Dream Air, per-eye target 3534 x 2838, RTX 4090, `data/perf/perf_20261027_215514.csv`)
after the centre-size fix (af75ef3):

- The look convinces the user ("sehr ueberzeugend", the evening skies "irre").
- The clouds are too coarse: "wie eine 6x6-Pixel-Rasterung durchgehend". The march runs at 1/3 of the eye resolution
  (VR preset divisor 3; the centre image 1377 x 947 for both eyes), so one march pixel spans 3 x 3 rendered pixels.
- The edge pixels shimmer: the march grid is fixed to the head, every small head motion moves the sample points, an
  edge pixel flips between cloud and sky. With the deterministic march nothing is averaged over frames.
- The GPU frame is 55-135 ms in the cloud zone (8-18 fps; pad ~60 ms, between the decks up to 135 ms); the CPU stays
  near 10 ms. Without clouds (far out, the cloud pass off) the frame is 2.6-10 ms at 90 Hz.

Full resolution with the per-frame march would be ~9 x the rays (0.5-1.2 s per frame) - not reachable. The march is
recomputed every frame although the clouds are static (static clouds design: the look changes only with the weather
hour and the Sun) and most of the view changes only by head rotation.

User's question 2026-10-27: "Warum koennen wir die Wolken nicht in die Welt rendern? Ist das nicht performanter?"

## What the user wants (their words, condensed)

- Clouds sharp and steady in VR "eigentlich durchgehend" - every phase, at display resolution, no 6 x 6 raster, no
  shimmering edges, fixed in the world.
- The look stays: light, shape, the evening skies.
- Approach: the world-fixed cache, **first without foveated rendering** ("also erstmal nur Cache.. ohne foveated").
- Fast flight close to clouds (deck passage): accepted that the near clouds are briefly softer while the frame rate
  holds ("ok, probieren wirs").
- Frame rate: **with weather always 45 Hz, without weather 90 Hz** ("mit Wetter immer 45 fps, ohne 90"); 45 Hz with
  the runtime's reprojection "ist butterweich".

## Goals

1. The clouds are rendered into a world-fixed cache around the camera and each eye only reads it per frame. Head
   rotation costs nothing and does not change what is shown.
2. In a still or slowly moving view (pad, ship deck, slow flight, orbit) the cache converges to the display's angular
   resolution: as sharp as a rendered pixel of the eye, no march grid visible, no shimmering edges.
3. Fixed GPU budget per frame for filling and refreshing the cache, so the frame rate target holds: with weather on the
   whole frame fits 45 Hz (22.2 ms), with weather off 90 Hz (11.1 ms) as today.
4. Camera motion is absorbed by reprojecting the cache through its stored cloud distance (parallax); only what the
   reprojection cannot give (disoccluded regions, too large a distortion, changed light or weather) is marched again,
   in order of importance (visible, near, most out of date).
5. The look is the existing march's: the cache is filled by the same march (density, light cache, air shadow, far tier,
   colour, transmittance), so the image converges to what the per-frame march at divisor 1 would show.
6. The same cache path on the desktop and in VR (one path; the desktop's camera turns become cheap too).

## Non-goals

- Foveated rendering / eye tracking (user: "erstmal ohne foveated"; a later option).
- Changes to the cloud physics, the field, the light model or the presets' look.
- Temporal jitter accumulation (the user rejected the jittered march's look in VR, 2026-10-26).
- The scaled-space far field beyond 1.5e7 m (already cheap, stays as is) and the rest of the weather (M11).
- A frame-time governor that switches between 90 and 45 Hz by load (discussed, replaced by the user's fixed rule).

## Anti-goals (failure even if it "works")

- The look changes against today's march (light, shape, colour, the evening skies).
- The cloud layer shifts against the world, or edges shimmer.
- Ghosting, smears or trails behind cloud edges when the camera moves.
- Visible cube seams, tile borders, or regions that pop in hard when refined.
- The two eyes see different content (other than the correct disparity).
- With weather on the frame misses the 45 Hz target in normal flight phases.

## Constraints

- Godot 4 double build, `CompositorEffect` with RenderingDevice compute (the existing `CloudEffect`); rendering only,
  no physics in GDScript; shaders in `game/shaders/`.
- Floating origin at the head: the cache anchor is held in float64 Earth-local coordinates (like `_previous_camera`),
  offsets to the eyes are differenced in double before they reach float32.
- Both eyes read one shared cache; per-eye geometry from the renderer's own matrices (`get_view_projection`,
  `get_cam_transform` = Godot's combined multiview camera; `cloud_vr_stereo_test`), each eye's own position for the
  parallax.
- The cache's contents may not depend on the frame's depth buffer (vehicles, tower, plume move or are near): what the
  cache marches ends at the static world (terrain / ellipsoid / sea) seen from its anchor; nearer scene geometry hides
  clouds behind it per frame by comparing its depth with the cached cloud distance (as the composite does today).
- Light and weather changes: the Sun's direction, the snapshot cross-fade weight, the light cache's re-anchor and
  clock jumps (`SimClock.time_jumped`, `World._view_jumped`) change cached content.
- VRAM: RTX 4090 (24 GB) shared with terrain, snapshots (~240 MB each, two resident) and the light cache; the cache's
  memory stays within 2 GB [E] (see the decisions below).
- The existing checks (`cloud_check.gd`, `cloud_far_check.gd`, `render_check.gd`, `cloud_vr_stereo_test.gd`) keep
  working; the per-frame march stays available as the reference path.
- Test workflow: only the affected tags / checks; visual checks need the GPU; images to `screenshots/<topic>/`.

## The design (decided)

### A. The cache

- A cube map (or an equivalent direction-parameterised atlas) around an **anchor** point in Earth-local float64,
  holding per texel what the march produces for the ray from the anchor in that direction: premultiplied colour,
  transmittance T, the transmittance-weighted cloud distance (the parallax depth).
- Its angular resolution matches the eyes' pixel density at the view centre (computed from the renderer's projection,
  not assumed), so a cache texel is not larger than an eye pixel.
- Organised in tiles (blocks of texels) that are filled, refined and invalidated independently, each with the state it
  was rendered with (anchor, Sun direction, weather weight, light cache generation, resolution level).
- Filled coarse-to-fine: a tile first gets a coarse level (about today's 1/3 density), then finer levels up to full
  resolution; refined tiles cross-fade over a short time instead of switching.

### B. Per frame: lookup with parallax

- Each eye pixel finds its cached value along its own ray from its own position: the anchor-to-eye offset is corrected
  through the cached cloud distance (an iterative parallax lookup), so near clouds sit at the right place and depth for
  each eye; far clouds reduce to rotation only.
- Scene geometry nearer than the cached cloud distance hides the cloud (per-frame depth comparison, soft at the
  boundary as the upsample's depth weight is today).
- The composite (`cloud_layer.gdshader` + `cloud_layer_light.gdshader`) stays: it blends the per-eye result as today.

### C. Refresh with a fixed budget

- A per-frame GPU budget for marching cache tiles (fill, refine, refresh); measured with GPU timestamps and the tile
  count adapted so the frame target holds.
- Priority: visible now (and a margin around the view for head turns), then nearest, then most out of date (the
  parallax error a tile's reprojection makes at the current camera, light / weather changes).
- When the camera has moved far from the anchor the cache is re-anchored progressively (no frame without clouds, no
  hard switch).
- Camera jumps and clock jumps restart the fill from the coarse level (no stale content shown).

### D. Frame-rate target

- With weather on the VR target is 45 Hz, with weather off 90 Hz (user rule). The cache budget is what is left of
  22.2 ms (weather on) after the rest of the scene.

## Measures of success

- VR, weather on, Start-VR.cmd `--perf-log`: pad, the deck passage of an ascent, above the decks and orbit at a steady
  45 Hz (frame p99 <= 22.2 ms in normal phases); weather off 90 Hz as today.
- Still view: the cache reaches full resolution within a few seconds [E, the plan fixes the number] after a start or a
  jump; the image then matches the per-frame march at divisor 1 within the existing checks' tolerances
  (cloud_check / render_check comparisons; the plan names them).
- No shimmer: with the camera still and the head turned by small angles the cloud pixels change only by the
  resampling of the eye's pixel grid (a check with a numeric bound).
- Parallax: a near cloud seen from two eye positions lands where the direct march puts it (extend
  `cloud_vr_stereo_test`).
- The user's VR check: sharp, steady, no layer shift, the deck passage acceptable.

## Decisions taken for the builder (Claude's calls, 2026-10-27 - the user may overrule)

- **The plan starts with measurement, not code**: (1) the VR GPU frame with weather on and the cloud pass off, per
  pass (`terrain_perf --gpu_passes` / `render_check --gpu_passes` at the VR pixel count, and one headset run) - the
  cache's real budget; known so far: without clouds 2.6-10 ms at 90 Hz (far out), the desktop proxy 5-9 ms (static
  clouds design); (2) how 45 Hz is held with the Pimax runtime (app frame pacing at 45, the runtime's half-rate
  reprojection, or the OpenXR refresh rate) - one short headset run with the user. If the budget turns out too small
  the convergence time grows (the frame target wins); the plan reports the numbers to the user before the cache is
  built.
- **Near shell when the camera is in or next to a cloud**: what lies closer than a near radius R_near [E, the plan sets
  it from the parallax error; order 1-3 km] is marched per frame at today's coarse density (the centre view, divisor 3)
  and composited in front of the cache, which then holds only what lies beyond R_near. Away from clouds the near shell
  is empty and costs nothing. This is the "briefly softer near clouds" the user accepted for the deck passage; the
  cache never has to represent a volume the camera is inside.
- **Resolution and memory**: about 35 px/deg (3534 px over ~100 deg per eye; the plan computes it from the renderer's
  projection): a 90 deg cube face ~3200^2 texels, six faces ~60 M texels; colour + T (RGBA16F) + distance (R32F) ~12 B
  -> ~0.75 GB, with the coarse levels ~1 GB. The VRAM cap is therefore 2 GB [E] (24 GB card), not 1 GB.
- **Tolerances**: the plan names the existing checks' bounds that the cache must meet against the per-frame march at
  divisor 1 (cloud_check / render_check / cloud_far_check sections) and sets the no-shimmer and parallax bounds
  numerically before the code that has to meet them.
- **Desktop and VR in the same plan**; VR is tuned first, the desktop must keep its checks green at every commit.
- One cache path for desktop and VR; the per-frame march stays as the reference path behind a flag (checks, A/B).
- The cache is filled by the existing march kernels (no second light or density model).
- Time acceleration: light changes are refreshed inside the same budget; at high time warp the lighting may lag
  slightly but never jumps (cross-fade).
- Refined regions cross-fade instead of switching (no pop-in).

## Left to the builder

- The exact parameterisation (cube faces, octahedral atlas, ...), the tile size, the levels, the formats.
- The parallax lookup's method and iteration count; how disocclusion is detected.
- The priority metric's form and the re-anchor strategy.
- How the cache marches to the static world without the frame's depth buffer (terrain heights, the ellipsoid / sea
  surface) and how near scene geometry hides it.
- Whether the outline refinement (Task 5b) is still needed at full cache resolution.
- How far the march kernels have to be refactored to fill direction-parameterised tiles (today they read the frame's
  depth buffer and the head-fixed centre grid; they already write the alpha-weighted cloud distance, the `depth`
  image).

## Open (the user decides later)

- Foveated rendering on top (eye tracking of the Dream Air), if the deck passage stays too soft.
