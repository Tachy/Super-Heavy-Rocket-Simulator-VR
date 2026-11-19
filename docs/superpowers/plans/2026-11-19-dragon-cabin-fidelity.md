# Crew Dragon Cabin Fidelity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild everything in the crew's field of view in the Crew Dragon cabin (console with suspension, screens and
button tray; seats; ceiling, walls, windows) to the real vehicle's shape, measured dimensions and textures.

**Architecture:** A committed multi-view photo-fit toolkit (`tools/structures/cabin_fit/`) measures each part from
annotated reference photos with known-size scale anchors; the values go tagged into `config/cockpits/crew_dragon.toml`.
Pure-Python shape modules (rounded / sculpted profiles, no Blender import) feed the existing Blender generator
`gen_dragon_cabin.py`; a cabin texture baker draws labels / seams and de-lights photo-projected structures. The shot
script renders each part from the fitted photo camera beside the photo for user acceptance.

**Tech Stack:** Python 3 (numpy, scipy.optimize, Pillow), Blender headless via `tools/structures/build_models.py`,
Godot 4 double (GDScript shot script), pytest.

**Spec:** `docs/superpowers/specs/2026-11-19-dragon-cabin-fidelity-design.md` (read it first).

## Global Constraints
- Rebuild only: no downloaded mesh / texture ships, nothing traced vertex for vertex; photos are measuring references.
- Rounds / chamfers visible as silhouette or highlight = geometry; seams, screws, labels, surface structure = textures / normal maps.
- Every new or changed config value carries [S] / [E] / [D] with derivation and tolerance (`test_cockpit_tables_are_tagged...`); superseded values move to `docs/history/cockpit.md`.
- Button / screen node ids stay (`test_manifest_buttons_carry_their_node_names`, `test_key_labels_only_where_a_photo_shows_them_and_ids_stay`); pointer hit geometry may be simplified under sculpted visuals.
- Centre and right screens stay placeholders; left screen = existing `ScreenCamera` feed.
- Console fixed (no swing-up).
- Performance: measure cabin GPU time at the VR target (`render_check --stereo=res://tests/data/pimax_dream_air_projections.json --gpu_time=120`, cockpit view) before T4 and after every geometry / texture task; > +2 ms vs the T1 baseline -> report and ask, do not cut quality.
- Done per part: render from the fitted camera lies on the photo (edge residual stated in px at photo resolution) AND user accepts the comparison PNG (send via SendUserFile; ask, then continue with the next task while waiting).
- All images to `screenshots/m7-cockpit/fidelity/<task>/`; long output to files.
- Commit + push on `main` after each green task (Conventional Commits, English). Tests: `python -m pytest tools/structures/tests -q -k <affected>`; GPU tests via `tools/run_godot_tests.ps1 -Gpu` only for touched cockpit tests.
- Rebuild models: `python tools/structures/build_models.py --vehicle crew_dragon` (`--force` after generator-only edits).
- Update `docs/subsystems/cockpit.md` (current state) in the task that changes the thing.

## Review Focus
1. Pointer / button use on the new tray and recessed screens: every key still reachable and hit (pointer lies on a surface, keys >= 5 mm proud or a hit face) -> T5 test `test_every_button_has_a_hit_face_on_the_tray`.
2. Fit with a wrong scale anchor or too few photos silently giving confident numbers -> T2 test `test_fit_reports_unconstrained_parameters` (Jacobian rank / covariance: tolerance inf -> tagged [E] with the prior).
3. VR stereo: sculpted parts very close to the eye (posts, tray at 0.5-0.7 m) z-fight or alias; normal maps shimmer -> T4/T5/T6 render `--stereo` close-up check, mipmaps on every new texture (`test_cabin_textures_have_mipmap_friendly_sizes`: power-of-two).
4. Lighting dependence of de-lit projected textures (cabin lamp on, Sun through the window, night) -> T6 shots in three light states.
5. The left feed in shots and in the game after the console rebuild (screen_left node / UV unchanged) -> T1 test plus T5 re-run of `cockpit_camera_test.gd`.

---

### Task 1: Shot baseline - left feed in the shots, fitted-camera comparison shots, GPU baseline

**Files:**
- Modify: `game/tests/cockpit_shots.gd` (drive the `ScreenCamera` SubViewport before capture; new `--fidelity` set)
- Create: `config/cockpits/photo_cameras.toml` (per photo: file, size_px, camera pose in the cabin frame, focal / fisheye model; tagged)
- Test: `game/tests/cockpit_shots_feed_test.gd`

**Interfaces:** Produces `photo_cameras.toml` rows `[[photo]] id, file, size_px, position_m, forward, up, fov_h_deg, model ("pinhole"|"fisheye_equidistant")`; shot names `fid_<photo_id>` and composites `fid_<photo_id>_vs`, `fid_<photo_id>_overlay` (50 % blend + edge overlay).

- [ ] Write `cockpit_shots_feed_test.gd`: after `_shoot()` of `pilot_eye`, the mean luminance of the `screen_left` region in the PNG differs from the placeholder black by > 0.02 (feed present). Run: fails (black today).
- [ ] Fix: force `ScreenCamera` update (`UPDATE_ONCE` + two frames) before capture. Test passes.
- [ ] Move the existing crew1_1 / discovery2 poses (round 4 values in `cockpit.md`) into `photo_cameras.toml`; `--fidelity` renders every row and composes vs / overlay sheets.
- [ ] GPU baseline: cockpit view `--gpu_time=120` stereo; record ms in `cockpit.md` Diagnostics.
- [ ] Send `fid_*_vs` to the user; commit `fix(cockpit): left feed in the shots, fitted photo cameras, fidelity baseline`.

### Task 2: Reference gaps

**Files:** `screenshots/m7-cockpit-refs/r2/README.md` (append), images in `r2/`; `tools/data_pipeline/sources.toml` only if something ships (nothing should).

- [ ] Retry Wikimedia Commons candidates listed in the README (1920 px thumbnails, 1 request / 5 s, generic User-Agent).
- [ ] Google Patents: SpaceX crew vehicle display / control panel mount, crew seat; save figures with dimensions.
- [ ] ICES-2020-333 PDF: read (`pip install pypdf` in a venv if needed), note cabin dimensions.
- [ ] Video stills: in-cabin launch camera views (Crew-1..11, Inspiration4, Axiom, Polaris Dawn webcasts) and trainer tours; `ffmpeg` (path in memory "Webcasts") frames into `r2/stills/`, URL + timestamp in README. Priority: console side / underside, posts and joint heads, ceiling ring, seat side.
- [ ] Append to README a per-part coverage table (part -> photos showing it -> viewpoint spread). Commit README only (images are git-ignored).

### Task 3: Fit toolkit

**Files:**
- Create: `tools/structures/cabin_fit/__init__.py`, `camera.py`, `fit.py`, `annotations.py`
- Create: `tools/structures/cabin_fit/annotations/` (one TOML per photo: `[[point]] label, px = [u, v]`; `[[edge]] label, px = [[u,v],...]`)
- Test: `tools/structures/tests/test_cabin_fit.py`

**Interfaces (produced):**
- `camera.project(points_m: np.ndarray[N,3], cam: Camera) -> np.ndarray[N,2]`; `Camera(position_m, rot (3x3), f_px, cx, cy, model, k: tuple)`.
- `fit.fit(photos: list[PhotoObs], model: PartModel, anchors: list[Anchor]) -> FitResult(params: dict[str, float], sigma: dict[str, float], residual_px: dict[photo_id, float], cameras: dict[photo_id, Camera])`.
- `PartModel`: `params: dict[str, (prior, prior_sigma)]`, `points(params) -> dict[label, xyz]`; `Anchor(label_a, label_b, length_m, sigma_m, source)`.
- `annotations.load(photo_id) -> PhotoObs`.

- [ ] Tests: (a) `test_fit_recovers_synthetic_box`: project a known 1.2 x 0.5 x 0.3 m box with 3 synthetic cameras (+0.5 px noise), fit from perturbed priors -> dims within 2 mm, residual < 1 px. (b) `test_fit_reports_unconstrained_parameters`: a parameter seen in no photo returns its prior and sigma == prior_sigma. (c) `test_fisheye_round_trip`: project / unproject within 1e-9.
- [ ] Implement with `scipy.optimize.least_squares` (soft_l1), covariance from the Jacobian; priors as residuals.
- [ ] Anchors available: IDSS docking passage 0.800 m [S], iPad mini 7.9" body 0.2032 x 0.1348 m [S], window aperture from the exterior table, ANSUR II hand / helmet medians (existing CSV).
- [ ] Commit `feat(cockpit): multi-view photo fit toolkit for the cabin`.

### Task 4: Measure and rebuild the console suspension and housing

**Files:**
- Create: `tools/structures/blender/console_shape.py` (pure: rounded-rectangle / filleted profile sweeps, post with joint heads); add to `generators.toml` `scripts`.
- Modify: `tools/structures/blender/gen_dragon_cabin.py` (`build_console` uses `console_shape`); `config/cockpits/crew_dragon.toml` `[console]` new keys; annotations for the console photos; `cabin_fit/parts/console.py` (PartModel)
- Test: `tools/structures/tests/test_console_shape.py`, extend `test_dragon_cabin.py`

**Interfaces:** new `[console]` keys (all tagged): `tilt_deg`, `hood_depth_m`, `edge_radius_m`, `side_draft_deg`, `post_radius_m`, `post_head_radius_m`, `post_head_length_m`, `post_top_m` (pair), `underside_profile` (list of [depth_m, drop_m]); `console_shape.rounded_box(w, h, d, r, segments) -> (verts, faces)`, `console_shape.post(base, top, r, head_r, head_len, sides) -> (verts, faces)`.

- [ ] Annotate the console in crew1_1, discovery2/10, jsc2026e404725, jsc2025e064539, jsc2022e068643/646/660 and T2 stills; fit; write values + sigma + residuals into `[console]` and the `cockpit.md` table; old values to history.
- [ ] Tests: rounded box has no edge sharper than `edge_radius_m` (max dihedral at sampled edges < 90 deg - tolerance), watertight; posts end on the ceiling ring surface within 5 mm; console tilt equals `tilt_deg` +-0.5 deg.
- [ ] Build models, render `--fidelity`, check edge residual, stereo close-up, GPU time; send PNGs; commit `feat(cockpit): measured console suspension and sculpted housing`.

### Task 5: Screens, bezels and the button tray

**Files:** `console_shape.py` (bezel, recessed screen, continuous tray with lip, guard cover), `gen_dragon_cabin.py` (`screen_mesh`, shelf -> tray, `build_button`, guard), `crew_dragon.toml` (`[[screen]]` bezel / radius / recess keys, `[tray]` table, guarded block, EXECUTE keys), `cabin_layout.py` (hit faces), tests.

**Interfaces:** `[[screen]]` adds `bezel_m`, `corner_radius_m`, `recess_m`; `[tray]` `depth_m`, `slope_deg`, `lip_height_m`, `radius_m`; `[[button]]` may add `guard = "<guard id>"`; `[[guard]] id, buttons, hinge_edge, rim_colour`. Node ids of screens and buttons unchanged; hit geometry node `console_hit` (flat faces, invisible, pointer collision).

- [ ] Measure screen diagonal, bezel, corner radius, recess, tray slope / depth / lip, key sizes and layout (jsc2022e068617 is the key reference) with the T3 toolkit; record.
- [ ] Tests: `test_every_button_has_a_hit_face_on_the_tray`, `test_screen_active_area_matches_the_measured_diagonal` (+-sigma), `test_guarded_block_lies_under_its_cover`, existing coplanarity / ids tests stay green (adapt `test_shelf_is_a_ten_centimetre_row_of_separate_modules` to the continuous tray; old rule to history).
- [ ] Rebuild, GPU tests `console_pointer_test.gd`, `cockpit_camera_test.gd`; `--fidelity` + close-up; GPU time; send PNGs; commit `feat(cockpit): measured screens, bezels and continuous button tray`.

### Task 6: Console textures

**Files:** Create `tools/structures/cabin_textures.py` (baker, registered in `generators.toml` `bakers` for crew_dragon), `tools/structures/photo_delight.py` (project photo onto a UV chart with the fitted camera, divide by a shading estimate from our render of the same pose, median over photos); modify `gen_dragon_cabin.py` (UVs, materials), `CockpitView` material setup if needed.

**Interfaces:** `cabin_textures.write(out_dir, cfg) -> dict[name, path]` like `suit_textures.write`; maps `console_albedo.png`, `console_normal.png`, `console_orm.png` (power-of-two, max 4096); `photo_delight.delight(photo_ids, part_uv_chart, cameras) -> np.ndarray` (linear albedo).

- [ ] Drawn layers: key labels, locker numbers, latches, screws, panel seams, vent strip, Velcro, connector cap (vector after the photos, positions from the fit).
- [ ] Projected + de-lit: brushed metal grain of the underside, dark panel surface structure.
- [ ] Tests: `test_cabin_textures_have_mipmap_friendly_sizes`, `test_labels_sit_on_their_keys` (label centre within 1 mm of key centre in UV->3D), `test_delight_recovers_flat_albedo` (synthetic: known albedo x shading -> albedo within 3 %).
- [ ] Shots in three light states (cabin lamps, Sun through a window, night); stereo shimmer check; GPU time; send; commit `feat(cockpit): console textures drawn and de-lit from the photos`.

### Task 7: Upper panels, lockers, iPad arms, wall recess around the console

**Files:** `gen_dragon_cabin.py` (`build_shell` trapezoid recess cut, lockers, tablet arms), `console_shape.py`, `crew_dragon.toml` (`[recess]`, `[[tablet]] arm_*`, `[[locker]]` latch), textures via T6 baker.

- [ ] Measure recess edges (crew1_1 + stills), lockers, latches, tablet arms; record.
- [ ] Tests: shell still closed except apertures (`test_shell_has_no_hole_except_the_apertures` extended by the recess), recess edges within sigma of the fit, tablets on their arms.
- [ ] Rebuild, `--fidelity`, GPU time, send, commit `feat(cockpit): console recess, lockers and tablet arms as measured`.

### Task 8: Seats

**Files:** Create `tools/structures/blender/seat_shape.py` (moulded shell, cushions, headrest, carbon leg supports), `gen_dragon_cabin.py` `build_seat`, `crew_dragon.toml` `[[seat]]` / `[seat_shape]`, textures (fabric / carbon projected + de-lit; seams drawn), tests `test_seat_shape.py`.

- [ ] Measure from KSC-20211110-PH-SPX02_0003, crew1_1, jsc2026e404725 + stills (shell width, back height, headrest, recline, leg support); keep knee / shin / foot photo bands (`test_knee_and_shin_follow_the_hatch_photo`, `test_foot_rest_lies_under_the_boots_of_every_seat`) green.
- [ ] Rebuild, check crew figures still seat (no interpenetration > 1 cm: `test_figure_inside_seat_shell`), `--fidelity`, GPU time, send, commit `feat(cockpit): measured seats with moulded shells and carbon leg supports`.

### Task 9: Room - ceiling ring and hatch, wall panels, window reveals, lamps

**Files:** `gen_dragon_cabin.py` (`build_shell`, `build_hatch`, `build_window`, `build_ring_light`), `crew_dragon.toml` `[cabin]`, textures (panel seams, handles red, placards), tests.

- [ ] Measure ceiling ring steps (jsc2024e000258, crew1_1), wall panel seam lines, window reveals, lamp positions; record.
- [ ] Build in order of visibility from the commander's eye; after each sub-part render and compare; stop at diminishing returns (sub-part residual already within sigma, or < 1 % of the pilot view's pixels) and report to the user what is left.
- [ ] Tests: shell / window tests stay green; ring steps within sigma; red handles present (`test_hatch_handles_exist`).
- [ ] GPU time, send, commit `feat(cockpit): measured ceiling ring, wall panels and window reveals`.

### Task 10: Docs, performance summary, final review

- [ ] `docs/subsystems/cockpit.md`: current state of geometry, measurements table, fit method, textures, GPU time before / after; history file for superseded values.
- [ ] `[slow]`-tier affected tests: `pytest tools/structures/tests -q`, `tools/run_godot_tests.ps1 -Gpu` (cockpit tests).
- [ ] Final review (Opus), fix round, commit + push; record state in ledger + memory; ask the user for the VR acceptance (M7 T9) and `/clear`.
