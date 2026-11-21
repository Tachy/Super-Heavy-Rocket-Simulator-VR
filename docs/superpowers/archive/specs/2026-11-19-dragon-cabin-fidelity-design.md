# Crew Dragon cabin fidelity (M7 follow-up) - design

Date: 2026-11-19. Continues `2026-11-07-m7-cockpit-crew-dragon-design.md`; runs before that plan's VR acceptance (T9).
Subsystem doc: `docs/subsystems/cockpit.md`.

## Intent
The user: the cabin "is far from reality". Everything in the crew's field of view must be an exact rebuild of the real
Crew Dragon: shape, real dimensions and textures. Reality counts; any photo or video is a measuring reference because we
rebuild everything ourselves (nothing from a reference ships, see memory "rebuild everything real").
What bothers the user most: the console's suspension and the look of the screens; also "not just boxy" (rounded,
chamfered, sculpted parts, not boxes).

## Goals
1. Console, in this order of priority:
   - Suspension: the two thin posts from the ceiling with their cylindrical joint heads, their attachment to the ceiling
     ring, the trapezoid recess of the white wall panelling around the console (crew1_1).
   - Housing: deep hood-like body tilted toward the crew (screens about 25-30 deg [E] back), silver / brushed underside
     with the vent strip, soft edges, sculpted side faces.
   - Screens: thin dark bezels, corner radii, slightly recessed in the panel.
   - Button tray: one continuous sloped tray with a lip (not separate blocks): square end keys (one EXECUTE), the 2 x 3
     guarded block (EMERGENCY / BREAKOUT / SUPPRESS ... under a hinged clear cover with red rim), small displays, blue
     LEDs, Velcro patch, capped connector, the handle.
   - Upper panel: dark grey panels with locker numbers and latches, the iPad arms above the outer screens.
2. Seats: moulded black shells, cushions, headrests, carbon leg supports, harness.
3. Room: ceiling ring with the hatch and red handles, curved wall panels with seams, window reveals, lamps.
4. Real dimensions for every rebuilt part, measured (see Measurement), each tagged [S] / [E] / [D] with source and
   tolerance in the `cockpit.md` table and the config tables (existing convention).
5. Textures: albedo / normal / roughness maps authored by us after the photos (labels, seams, screws, fabric, carbon,
   brushed metal).

## Non-goals
- Screen content: centre and right screens stay placeholders (user 2026-11-19); the left screen keeps the existing
  external camera feed (`ScreenCamera`, Task 7). The shot script must render that feed (it shows black today).
- No physics, mission or input behaviour changes; the existing button / pointer functions keep working on the new
  geometry (same node ids or a mapped rename).
- No console swing-up for ingress: only Dragon V2 (2014) is documented swinging; all flight photos show it fixed
  (Claude's call; the user may revisit).
- Crew figure / suit: not in scope (own Task 8b).

## Anti-goals (a failure even if it "works")
- Boxy parts where the real part is rounded or sculpted.
- Dimensions guessed without a documented fit or reference.
- Any downloaded model, mesh or texture shipped or traced vertex for vertex.

## Constraints and standing decisions
- Geometry vs texture (user rule "surface detail not mesh", read for this work and confirmed by the user's "los"):
  rounds and chamfers visible as silhouette or in a highlight are real geometry; seams, screws, labels and surface
  structure are textures / normal maps.
- The cabin and console come from the Blender generator `tools/structures/blender/gen_dragon_cabin.py` (+ `cabin_layout.py`)
  and the config tables; vehicle config stays data-driven. Placement values that exist ([[seat]], [eye], console
  placement from round 4 fit) may change only through the new measurement, documented with the old value moved to
  `docs/history/cockpit.md`.
- Performance: VR target 90 Hz on RTX 4090 at the Pimax render target; the cabin is always in view, so triangle and
  texture budgets must be stated and measured (builder sets the numbers).
- Images and diagnostics go to `screenshots/<topic>/` only.

## Measurement (how real dimensions are obtained)
No public drawings exist. Known hard facts: pressurised volume ~9.3 m^3, outer diameter 4.0 m, 4 seats, 3 touchscreens.
Method: multi-view photogrammetric fit per part (as round 4, `fit4.py`, but part by part): camera pose and focal per photo,
part geometry parameters shared across photos, scale from parts of known size: IDSS docking passage 800 mm, iPad mini
(already the console scale), ANSUR II hand / helmet measures, window size from the exterior table. Optional: COLMAP on
frame sequences from cabin walkthrough videos as a point-cloud cross-check. Every fitted value gets a residual and a
tolerance.

## References
- Existing: `screenshots/m7-cockpit-refs/` (mielke/: crew1_*, discovery*, demo1_*; nasa/), notes in the M7 ledger dir.
- New 2026-11-19: `screenshots/m7-cockpit-refs/r2/` (36 NASA public-domain photos, README with per-image use; best for
  console: jsc2022e068617 (tray, bezels), jsc2026e404725 / jsc2025e064539 (side profile, tilt), jsc2022e068660 /
  068643 / 068646 (underside), jsc2024e000258 (ceiling hatch ring), KSC-20211110-PH-SPX02_0003 (seats)).
- Gaps to fill during the work: Wikimedia Commons (rate-limited, retry), patents, ICES-2020-333 paper, video stills of
  the in-cabin launch cameras and trainer tours.
- Note: the Hawthorne trainer stands on a floor frame; use it for console face details only, not for the suspension.

## Verification
- Per part a comparison image: our render beside the reference photo from the fitted camera pose (extend
  `game/tests/cockpit_shots.gd`); progress images sent to the user after each task.
- Generator tests keep / extend `tools/structures/tests/test_dragon_cabin.py` (every table value tagged, fit
  tolerances, button reachability as today).
- Existing cockpit tests stay green (`console_pointer_test.gd`, `cockpit_camera_test.gd`).

## Answered with the user's "ok, dann los" (2026-11-19; read as accepting Claude's proposals)
- Textures: mixed. Labels, numbers, seams, screws and panel edges are drawn by us (vector / procedural after the
  photos). Irregular structures (seat fabric, carbon, brushed metal grain) may be projected from the public-domain photos
  and baked, with the photo lighting divided out (de-lit), so the maps stay correct under any light.
- Done per part: the render from the fitted camera pose lies on the reference photo (edges within a few px at the
  photo's resolution, stated per comparison) AND the user accepts the comparison image. Each measured value carries its
  own tolerance from the fit; no fixed global mm tolerance.
- Scope of this run: all three goals (console, seats, room), in that order; the room until diminishing returns, then
  report and ask.

## Calls made by Claude (from the builder check; user may revisit)
- Performance: quality holds; the cabin's GPU time is measured at the VR target (`render_check --stereo --gpu_time`)
  before and after each part; above +2 ms per frame vs today the builder reports and asks instead of cutting quality.
- Interaction: button and screen node ids stay; pointer / hit geometry may be simplified (flat faces or boxes) under
  the sculpted visual meshes; the game keeps loading the cabin by node name from the generated glTF.
- Under-constrained parts (seen in only 2-3 photos): tagged [E] with the stated tolerance, no blocking; listed in the
  report so the user can supply photos. Reference gaps (Commons, patents, ICES paper, video stills) are retried as a
  task of their own, the work does not wait for them.
- Left screen black in the shots: a shot-script bug (the SubViewport feed is not driven there); fixed as the first,
  small task, GPU run.

## Left to the builder
Task split and order inside the priority above, mesh topology and triangle budget, texture resolution and atlas layout,
the fitting scripts, which photos per part, how far the room (goal 3) goes before diminishing returns (report and ask).
