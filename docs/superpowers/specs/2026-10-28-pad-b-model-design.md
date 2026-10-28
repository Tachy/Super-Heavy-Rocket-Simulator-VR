# Starbase Pad B detail model - design

Date: 2026-10-28. Status: draft for the user's approval. First of the detail models (order: Pad B tower + launch
table, then Starship + Super Heavy, before everything else). Inputs: `.superpowers/sdd/2026-10-28-vehicle-models/`
(`user-decisions.md` binding; `research-models.md` research and recommended approach). Touches the catch tower physics
(`core/src/contact/`, `core/src/sim/catch_tower_sim.cpp`, `config/launch_sites.toml` `[site.tower]`), the tower view
(`game/world/tower_view.gd`), lighting / eye adaptation (`docs/subsystems/eye-adaptation.md`) and the VR budget
(`docs/subsystems/vr-performance.md`). A subsystem file `docs/subsystems/launch-structures.md` is created with the
first build task.

Terms. **Pad B**: Orbital Launch Pad 2 at Starbase (tower 2 "Mechazilla", the square launch mount, the flame trench).
**Physics geometry**: what the core collides with (`TowerConfig`, `TowerGeometry`). **Detail file**: the new
data file of the visual model (`config/structures/starbase_olp2.toml`). **Modules**: the non-repeating parts made by
Blender scripts (chopsticks, carriage, QD arm, mount, flame trench / bucket, light fixtures).

## Why

- Today `tower_view.gd` (83 lines) draws the tower as one box with the `lattice.gdshader`, the mount as a ring of 24
  concrete boxes and each arm as one steel box, all from `FlightSim.site_tower()`, moved by `FlightSim.tower_state()`
  (arm opening, carriage offset). Correct for the physics, but nothing in it looks like the real tower.
- The only Starbase site, `starbase_olp1`, carries Pad A numbers (arms 36 m [E], annular mount). Pad A was
  decommissioned 2025-10-14 for a rebuild; flights 12-14 (2026) flew from Pad B, whose arms are ~10 m shorter and whose
  mount is square with a flame trench (`research-models.md` 2.2).
- No usable ready-made model exists: fan / print models are SDFL or CC BY-NC-SA, Pad A era and print-simplified;
  marketplace models cost money or forbid AI use; photogrammetry from public video is neither legal nor practical for a
  thin lattice (`research-models.md` 1, 3.1). The project's 2026-10-23 rule stands: rebuild from references.

## What the user wants (their words, condensed)

- Start with the current Mechazilla at Pad B. It must look real: "die Chopsticks, das Gitterwerk und die Größe".
- Seen mainly "beim Fang aus der Außenansicht und am Pad unten". "Das Ding muss möglichst 1:1 die Form haben wie in
  der Realität ... das Gitterwerk ... es kann ruhig vereinfacht aufgebaut sein, aber vom Eindruck her".
- Chopsticks: "auch die Bewegung, zufahren und greifen wie in echt" - carriage travel, arms closing, catch, driven by
  the sim.
- Scope: tower + "der Starttisch mit den beidseitigen wassergekühlten Abgastunneln"; "erstmal nur was direkt am
  Startplatz steht" (no tank farm, buildings, roads yet).
- "Beleuchtung wie in echt von Anfang an" (floodlights, tower lights).
- Deluge water and steam later, with flames and plasma (effects milestone).
- How it is made: "leg es nach der Recherche fest" (Claude decides after the research).
- Earlier (2026-10-23): no ready-made models; rebuild from references (photos, published dimensions [S]/[E], no copied
  images or logos); per-structure detail file in `config/`; meshes from the physics data; Blender scripts -> glTF only
  where clearly better.
- Any photo that shows reality may serve as a reference for building: "alles hernehmen, was taugt und nach dem Bau
  wieder verwerfen".
- Afterwards Starship + Super Heavy with the same approach ("nach dem Turm, vor allem anderen"); tiles and similar
  surfaces "am besten mit Texturen".

## Goals

1. Pad B tower, chopsticks, carriage, ship QD arm, square launch mount and the flame trench with its two exhaust
   tunnels read as the real structure from the catch outside view (50-500 m) and from the pad (eye at 2 m, 10-100 m):
   silhouette, proportions, lattice pattern, member sizes, materials.
2. One master: every visual part is placed from the physics numbers (`[site.tower]`) plus the detail file; surfaces the
   vehicle can touch coincide with the physics boxes, purely visual parts stay inside the physics envelope.
3. A Pad B physics site with its own numbers (arm length, mount shape) and the catch tests green on it.
4. Motion like the real one, driven by the sim: carriage travel, arms swinging open / to standby / closed and gripping,
   the carriage's shock stroke under the caught booster, the QD arm swinging.
5. Night and twilight lit like reality: floodlights and tower lights with physically based intensities, seen through
   the eye adaptation, from the first build.
6. VR: the pad scene keeps 45 Hz with weather on and 90 Hz with weather off on the user's rig.

## Non-goals

- Starship and Super Heavy detail models (separate spec, same pipeline; tiles and surfaces as textures).
- Deluge water, steam, flames, plasma, scorch build-up (effects milestone).
- Everything not standing directly at the pad: tank farm, buildings, roads, Pad A rebuild, other sites' towers
  (LC-39A keeps today's box view until it gets its own detail file).
- The booster QD / mount clamps as moving parts (static geometry now; motion with the vehicle spec).
- Lowering a caught booster onto the mount (handover) and ground operations.
- Independent per-arm actuation (each chopstick on its own angle): one symmetric opening as in the core today.

## Anti-goals (failure even if it "works")

- Any photo pixel in a game asset, any photo file in the repo, any logo or trademark lettering on the structure.
- A visual surface the vehicle touches that disagrees with the physics (arm tops, tower face, mount deck) by more than
  the tolerance, or a visual part outside the physics envelope (the booster hits air or passes through steel).
- Shimmer / crawling of thin members in VR, LOD popping, a visible hand-over band.
- Arms or carriage moving on their own clock (animation instead of `tower_state()`).
- "Game" lighting: lights that do not illuminate, constant-brightness emissive blobs, a night pad brighter than the
  eye-adapted reality, or a day scene changed by the night lights.
- A frame budget miss in the pad or catch scene.

## Constraints

- Architecture: `core` pure C++ / float64, no Godot; the game only reads state and sends commands; physics never in
  GDScript; vehicles and sites are configuration; `game/` holds text formats only (.tscn / .tres / .gd / shaders).
- Data rule: only free data; every source in `tools/data_pipeline/sources.toml` with its licence; numbers [S] / [E] /
  [D] with source (`config/README.md`).
- Floating origin: positions in float64 Earth-fixed, differenced in double before float32 (the tower is a child of the
  Earth body today; kept).
- Lighting: physical luminances through `LitAerial` (aerial perspective) and the eye adaptation's cone gain; Sun,
  Moon and sky as today.
- VR budget (`vr-performance.md`): 11.1 ms at 90 Hz without weather, 22.2 ms at 45 Hz with weather; scene without
  clouds 7.4 ms median today; the cloud world volume plans ~11 ms.
- Tests: only the affected tags (`[contact]`, `[catch]`, `[ship]`, `[gnc]` for the physics change); visual checks on
  the GPU, images to `screenshots/pad-b/`.
- Toolchain: Blender 3.4 at `C:\Program Files\Blender Foundation\Blender 3.4` (GPL; its output is not covered by the
  GPL).

## The design (decided)

Chosen after the research (`research-models.md` 5, Approach 1, with the reference rule of Approach 3): a data-driven
hybrid - the repeating lattice instanced by Godot, the non-repeating modules generated by headless Blender scripts,
both from the same data, the physics numbers the master. All points in this section are Claude's calls on the user's
"leg es nach der Recherche fest", except where a user decision is cited.

### A. References and measurement

- Rebuild ourselves from dimensions. Photos and videos of any source (SpaceX, NSF, LabPadre, RGV, Commons, NAIP) are
  references for dimensions, shape and appearance only (user decision 2026-10-28), kept in git-ignored
  `screenshots/mechazilla/pad-b/` (with an `INDEX.md`: source, date, view, what it shows) and discarded after the
  build. No photo pixels in assets, no photo files committed, no traced drawings. No stream downloading (no yt-dlp):
  only frames / photos viewed and saved by hand; the user may contribute frames but is not required to.
- Measurement is agent work and scripted (Claude's call): `tools/structures/measure/` scripts read the identified
  image points of each reference (a points file per image next to it, git-ignored) and derive dimensions from
  known-scale ratios (booster diameter 9 m [S], NAIP footprint) and, for the views used in the silhouette check, a
  scripted camera solve (OpenCV `solvePnP` from identified points with known positions; OpenCV is BSD-licensed and is
  recorded in `tools/data_pipeline/sources.toml` when added as a dependency). Each result is reproducible from the
  points files while they exist.
- Every measured value goes into the detail file tagged [E] with its method: "ratio to the 9 m booster diameter,
  image <id>", "camera solve (solvePnP, n points, reprojection error x px), image <id>", "NAIP 0.6 m top-down, vintage
  YYYY", "shadow length on dated NAIP + own ephemeris", "count of sections x section height". Published values [S]
  with the URL. Values needed first: tower height (144.5 m [E]),
  section count and height, face width, leg and brace profiles and pitch, platform levels, chopstick length (~26 m
  [E]) / width / depth, carriage size, QD arm length and hinge height, mount plan, height and deck, trench footprint,
  depth and tunnel exits, Pad B coordinates (25.9966 N, -97.1583 [E], verified on NAIP).
- NAIP vintage must post-date the tower (stacked 2024-08-21) and the mount (lifted 2025-05-12); if none does, the plan
  states which values stay photo-only.

### B. Physics site `starbase_olp2` (the master)

- New `[[site]]` `starbase_olp2` "Starbase Pad B" in `config/launch_sites.toml` with its own `[site.tower]`: arm length
  ~26 m [E], tower height, width, distance and azimuth from the measurement; height_m [D] from
  `tools/data_pipeline/site_heights.py`; plateau radius covering mount, tower and trench.
- Arm kinematics re-derived for the shorter arms: `arm_standby_deg` and `arm_closed_gap_m` (today defined at 24 m
  from the hinges) from the measured hinge-to-catch-axis distance. Physics first: if `distance_m` minus half the width
  plus the arm length does not span the catch axis with the pins on the arm tops, one of the [E] numbers is wrong and
  is re-measured - no fudge.
- Square mount: `TowerConfig` gets `mount_shape` ("annulus" default, "square") with `mount_half_width_m` and
  `mount_opening_half_width_m`; `TowerGeometry` builds the square frame from four boxes. Pad A and LC-39A keep the
  annulus.
- Flame trench: not a collision body. The site ground stays the flat plateau in physics (a vehicle falling into the
  trench is not modelled: documented simplification, fidelity note in the subsystem file). The trench is cut out of the
  rendered terrain by a site mask from the detail file's footprint polygon (terrain shader discard + the trench's own
  walls), so it is visible from the pad.
- Carriage travel: `catch_arms` gets a commanded carriage height (winch, rate [E] from video timing) on top of the
  shock stroke; `tower_state()` reports `carriage_height_m` and `carriage_offset_m` separately. The catch sequence
  raises the carriage to `catch_height_m` before the return; parked height [E] otherwise.
- Ship QD arm: `tower_state()` gets `qd_arm_angle_rad`, driven by the launch sequence (connected on the pad, swings
  clear at a time [E] before ignition, rate [E]). No collision box: it is retracted inside the tower envelope before
  any vehicle motion (documented).
- Physics first (project rule, user 2026-10-14): the Pad B geometry comes from measurement and is never tuned for
  catch success. The real Pad B catches boosters with these arms, so if the `[slow]` catch campaigns on
  `starbase_olp2` fall below `starbase_olp1`'s success rates, that is a guidance gap, not a geometry fact: the plan
  then runs a catch-guidance retuning stage (`[gnc][catch]`, SCvx catch, booster return, ship catch laws) until the
  rates on `starbase_olp2` are at least `starbase_olp1`'s (Claude's call).
- The Starbase missions that today name `starbase_olp1` (flight 14 ascent, depot / tanker / HLS / LEO, ship returns)
  move to `starbase_olp2` only after its catch tests are green and the retuning stage (if needed) has met the rates;
  `starbase_olp1` stays as a site with today's box view.
- Re-run on Pad B: `test_tower_geometry`, `test_catch_arms`, `test_catch_tower`, `test_catch_trigger`,
  `test_catch_feasibility`, `test_scvx_catch`, `test_ship_catch` (+ their `[slow]` campaigns), `test_booster_return`
  and the mission tests that use the moved site.

### C. Detail file and agreement check

- `config/structures/starbase_olp2.toml`: sections, face width, member profiles (H-beam, tube, angle, box: size, wall),
  bracing pattern and pitch per section, platforms, stairs / cable trays as boxes, carriage, chopsticks (truss, catch
  rails, shock pads, hydraulic actuators), QD arm, mount, trench / bucket / tunnels, light fixtures (type, position,
  aim, flux, colour temperature, shadow flag). It references the physics numbers by name (`tower.arm_length_m`,
  ...) and never duplicates them; derived positions are computed from them.
- A Python check (`tools/structures/check_envelope.py`) reads both files and the exported glTF bounds: touch surfaces
  (arm top rails, tower face toward the pad, mount deck) within **0.10 m** of the physics boxes, every other part inside
  the physics envelope (tolerance 0.05 m). It runs in the generation step and in a `[structures]` test.

### D. Lattice: Godot instancing from config

- `game/world/structures/lattice_builder.gd` builds the tower lattice at load time from the detail file: 3-4 profile
  meshes (beveled H-beam, tube, angle, box, made by a Blender script as well) instanced as MultiMesh, one
  MultiMeshInstance3D per section and face (Godot picks one LOD per MultiMesh node).
- Distance hand-over: full members near; a merged per-section mesh mid-range; beyond, a lattice shader on the tower box
  (today's `lattice.gdshader`, extended to the measured pattern so the far look matches the near one). Fades by
  visibility ranges with dithering over a band, chosen so a member is never thinner than ~1.5 px before it fades
  (anti-shimmer); starting ranges ~600 m / ~5 km [E], fixed by the prototype.
- Shadows: members do not cast individually; a per-section shadow proxy casts.
- Materials: CC0 PBR sets (ambientCG, Poly Haven: painted / galvanised steel, concrete, stainless) via `LitAerial`
  (extended for normal / roughness / AO maps), triplanar grime and rust masks, own decals (hazard stripes, stencil
  numbers); no logos. Each set recorded in `sources.toml`.

### E. Modules: headless Blender Python -> glTF

- Scripts in `tools/structures/blender/` (`gen_chopstick.py`, `gen_carriage.py`, `gen_qd_arm.py`, `gen_mount.py`,
  `gen_trench.py`, `gen_lights.py`, `gen_profiles.py`, shared `common.py`), run as
  `blender -b --factory-startup -P <script> -- --config config/structures/starbase_olp2.toml --out <dir>`; a driver
  `tools/structures/build_models.py` runs them all, deterministically (fixed seeds, no time stamps).
- Output: glTF (`.gltf` + `.bin` + textures) into git-ignored `data/models/starbase_olp2/` with a manifest (input hash
  of config + scripts + Blender version). The game loads them at run time from `data/models/` like other pipeline data
  (`Paths`), so `game/` stays text-only and no binary is committed.
- Generated when missing or stale: `tools/build.ps1` and the render / smoke test runners call the driver if the
  manifest hash differs; `tools/package.ps1` runs it and copies `data/models/` into the base data of the release (it
  is small, < 100 MB; no separate data part). Without the models the game falls back to today's boxes and logs it.
- Each moving module has its pivot at its physical hinge (arm hinge on the carriage, QD arm hinge, carriage origin at
  the arm top plane) and named nodes; bevels, UVs, baked AO / curvature for the grime masks; LODs by Godot's import of
  glTF (or Blender decimate levels where Godot's runtime-loaded glTF has none - left to the prototype).

### F. Motion from the sim

- `tower_view.gd` becomes the assembler: it loads the lattice and the modules, places them from `site_tower` + the
  detail file, and each frame applies `tower_state()` (interpolated like every other render state): carriage height +
  shock offset, the arm opening, QD arm angle; the hydraulic actuators follow the arm angle by their geometry
  (visual only). No animation player, no own clock.
- Arms (Claude's call): the core has one symmetric opening angle (`opening_rad`); the view applies it to both arms,
  mirrored about the tower's centre plane (as `TowerGeometry::arm_direction` does). The mirroring is purely visual;
  independent per-arm actuation is a non-goal for now.

### G. Lighting like reality

- Fixtures from the detail file: floodlight banks (spot lights, aimed at the mount and the tower; luminous flux and
  CCT [E] from the class of stadium / industrial LED floods), tower lights and obstruction lights (red, flashing per
  FAA AC 70/7460-1 pattern [S]), deck and stair lights (omni, small). Intensities in physical units (lm -> cd), scaled
  into the game's luminance like the Sun and Moon, so the eye adaptation treats them exactly as other physical light;
  fixture lenses emissive at the same luminance.
- Cost control: only a few floodlights cast shadows (e.g. 4 [E]), the rest unshadowed; distance-limited ranges; far
  away the lights become emissive points (the pad seen from the viewpoint or the air at night).
- Schedule: on between civil dusk and dawn at the site (Sun elevation from the ephemeris), obstruction lights always.
- Budget priority (Claude's call): if the combined pad + clouds scene misses 22.2 ms p99 in VR, the pad side gives way
  first - fewer shadowed floodlights, then earlier lattice fades (hand-over ranges shortened), then fewer unshadowed
  lights - before any cloud quality is reduced; each step reported with its frame time.
- The pad's light enters the adaptation luminance L_a only through what the eye sees (no global term).

### H. Prototype first - STOP point

1-2 days, before the full build:
- Full-height Pad B tower lattice instanced from a draft detail file (first measurements), placed at `starbase_olp1`'s
  tower box for now; one CC0 steel material + grime; hand-over to the lattice shader.
- One chopstick from a Blender script, exported glTF, swapped in for one arm box, driven by `tower_state()`.
- Renders via `res://tests/render_check.gd` to `screenshots/pad-b/`: from the catch camera, from the pad at 2 m eye
  height, from 1 km; plus a slow dolly through the hand-over range (pop check) and a headset run
  (`Start-VR.cmd --perf-log`) with the tower in view.
- **STOP**: images and frame times to the user; the full build starts only on the user's go.

### I. Full build order (after the gate)

Measurement pass -> `starbase_olp2` physics site + square mount + re-derived arms + carriage travel + QD arm state ->
catch tests -> catch-guidance retuning stage if the campaign rates fall below `starbase_olp1`'s -> detail file complete -> lattice + profiles -> chopsticks, carriage, QD arm -> mount, trench, tunnels ->
materials, decals -> lights -> envelope check, renders, VR run -> mission switch to `starbase_olp2`.

## Decisions taken for the builder (Claude's calls - the user may overrule)

1. Rebuild from dimensions; photos / videos of any source as references only, discarded after the build, no photo
   pixels or files in the repo, measured values [E] with method.
2. Hybrid pipeline: lattice instanced by Godot (MultiMesh) from config with a distance hand-over to a lattice shader;
   non-repeating modules from headless Blender Python scripts (Blender 3.4) as glTF.
3. Scripts committed; the generated glTF in git-ignored `data/models/`, generated reproducibly (manifest hash) at
   build / test time and copied into the release's base data by `tools/package.ps1`; fallback to boxes when missing.
4. CC0 materials (ambientCG, Poly Haven) and own decals, no logos.
5. Physics numbers are the master; visual parts are placed from them; envelope check 0.10 m (touch surfaces) /
   0.05 m (inside the envelope).
6. New physics site `starbase_olp2` (arms ~26 m [E], square mount); catch tests re-run; Starbase missions moved to it
   when green; `starbase_olp1` kept with the box view.
7. Square mount as four physics boxes (`mount_shape`); flame trench visual only (terrain mask), not a collision body.
8. Carriage height travel and the QD arm angle become sim state in `tower_state()`; QD arm without collision.
9. Real lighting from the start: physical units, few shadowed floodlights, obstruction lights per the FAA pattern,
   on at civil dusk.
10. VR at 45 Hz (with weather) as the binding budget; the pad structure's share below.
11. 1-2 day prototype as a STOP point before the full build.
12. Subsystem file `docs/subsystems/launch-structures.md` created with the first task.
13. Physics first: Pad B geometry from measurement only, never tuned for catch success; a lower catch rate on
    `starbase_olp2` triggers a catch-guidance retuning stage (`[gnc][catch]`, SCvx catch, booster return); the mission
    switch waits until olp2's rates are >= olp1's.
14. Reference measurement is scripted agent work: known-scale ratios (9 m booster, NAIP footprint) and an OpenCV
    `solvePnP` camera solve from identified points (BSD, recorded in `sources.toml`) on the references collected in
    `screenshots/mechazilla/pad-b/` (`INDEX.md`); no stream downloading; user frames welcome, not required.
15. One symmetric arm opening (the core's `opening_rad`) mirrored visually for both arms; per-arm actuation later.
16. Budget priority: when pad + clouds miss 22.2 ms p99, the pad gives way first (fewer shadowed floodlights,
    earlier lattice fades) before any cloud quality is reduced.

## Left to the builder

- Profile mesh triangle counts, bevel sizes, LOD levels and exact visibility ranges within the anti-shimmer rule.
- How runtime-loaded glTF gets LODs (Godot import generation vs Blender decimate levels).
- Per-section merging and shadow proxy shapes; light count per class within the budget.
- The carriage / QD arm rates and timings from video timing ([E] with method).
- The structure of the detail file beyond the items listed in C.

## Measures of success

- **Silhouette / proportions**: against the reference measurements (each [E] with its method) - tower height within
  1 %, face width, section heights, platform levels, chopstick length and depth, carriage height, mount plan and
  height within 3 % or 0.3 m (the larger); lattice pitch per section within 5 %. Checked by overlaying the render's
  silhouette (camera from the scripted `solvePnP` solve of the same reference image) on the reference's outline:
  silhouette IoU >= 0.90 in three views (catch outside view, pad, 1 km) - the overlay is computed locally and not kept.
  If fewer than three references allow a camera solve, the IoU check falls back to the proportion checks above for
  the missing views and this is reported to the user.
- **Physics agreement**: `check_envelope.py` green (0.10 m / 0.05 m); `[contact]`, `[catch]`, `[ship]`,
  `[gnc][catch]` tests incl. `[slow]` campaigns green on `starbase_olp2`; campaign catch success rates on
  `starbase_olp2` >= `starbase_olp1`'s after the retuning stage (if it was needed), with the geometry unchanged by it.
- **Motion**: the rendered arm angle, carriage height and shock offset equal `tower_state()` (interpolated) within
  0.1 deg / 0.01 m in a recorded catch (render_check frames vs the snapshot log); no motion while the sim reports none.
- **LOD hand-over**: in a dolly from 50 m to 5 km, per-frame luminance change of the tower pixels at each hand-over
  band <= 2 % beyond the dolly's own change; no frame with a member below 1 px wide before its fade; shimmer ratio
  (head turned in a still view) <= 1.10.
- **VR frame time** (headset `--perf-log`, Pimax Dream Air, RTX 4090): pad scene at night with all lights and the
  catch outside view: tower + mount + lights add <= 2.0 ms GPU median, <= 3.0 ms p99 to today's scene (7.4 ms
  median); 90 Hz without weather (frame p99 <= 11.1 ms) and 45 Hz with weather (p99 <= 22.2 ms) hold; main thread
  + 0.3 ms at most.
- **Lighting**: floodlit mount deck illuminance at night within 30 % of the fixture model's computed value (render
  probe); daytime pad luminance unchanged with the lights off vs on (< 0.5 %); obstruction light flash rate per the FAA
  pattern; the user's judgement from night and twilight images.
- **Licences**: every material and source in `sources.toml` / CREDITS; `git ls-files` shows no image other than
  generated or CC0 files; no logos.
- The user's verdict on the prototype images and the final catch scene: "sieht echt aus".

## Open (the user decides later)

- Start order against the cloud world volume work in flight (which runs first, or interleaved); the budget priority
  rule (decision 16) applies either way.
- Independent per-arm actuation of the chopsticks (needs a per-side angle in the core).

- Whether the ship QD arm and the booster QD / clamps get collision bodies later (with the vehicle spec).
- Whether `starbase_olp1` stays selectable at all (Pad A is out of service) or is replaced by its V3 rebuild later.
- Own photos from public places (Highway 4 / Boca Chica beach) as clean references or texture sources, if the user or a
  contact can take them.
- The FAA EA site plans (faa.gov 403 to the fetcher): one manual look by the user for the trench footprint.
- The order inside the vehicle spec (booster first or ship first) - after this tower.
