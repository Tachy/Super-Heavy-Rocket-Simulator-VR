# Launch structures (Pad B tower and chopstick model)

State: prototype plan `docs/superpowers/plans/2026-10-31-pad-b-prototype.md`, task 1 done (draft detail file). Spec:
`docs/superpowers/specs/2026-10-28-pad-b-model-design.md`. Nothing is rendered from the file yet.

## Model
- `config/structures/starbase_olp2.toml`: site-independent detail of THE Mechazilla (tower sections, member profiles,
  chopstick truss). Placement and physics numbers (arm length, hinge, tower height) stay in `launch_sites.toml`; the
  file never duplicates them. Keys: `config/README.md`.
- Every value is tagged `[E]` (own estimate with its method) or `[S]` (published, URL).
- Draft numbers: 12 sections x 12.04 m = 144.5 m, face width 12 m, pitch 8 m, leg box 1.6 m, brace H 0.7 m; chopstick
  length 26 m, width 3 m, depth 8.75 m, tube chords 0.65 m, diagonals 0.5 m.

## Standing decisions (from the spec)
- Rebuild from dimensions; photos only as references in git-ignored `screenshots/mechazilla/pad-b/`, no photo pixels,
  no traced drawings, no logos.
- Measurement is scripted: `tools/structures/measure/ratio.py` reads a points file (next to the image, git-ignored)
  and scales by the 9 m booster diameter [S]. Camera solve (`solvePnP`) and silhouette IoU: full build.
- Hybrid pipeline: lattice instanced by Godot from the file, non-repeating modules by headless Blender (glTF in
  git-ignored `data/models/starbase_olp2/`), boxes as fallback. Physics numbers are the master (envelope check 0.10 m /
  0.05 m, full build).
- Prototype: placed at `starbase_olp1`'s tower box; arm visual length 26 m vs. physics 36 m is a known mismatch.

## Validation
- `python -m pytest tools/structures/tests -q`: required keys, sections sum to 144.5 m within 1 %, every value line
  tagged, loader raises on missing keys / bad units.

## Open points
- Single-view ratios in oblique photos (face width, depth) carry several percent error; leg, wall, pitch and truss
  pitch are estimates (not measurable from the references). The full build re-measures with a camera solve.
