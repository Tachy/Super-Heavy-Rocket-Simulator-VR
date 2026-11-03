# Falcon 9, Crew Dragon with trunk, ISS: detail models

## Intent
Falcon 9 and Crew Dragon missions still show plain cylinders. These three vehicles get real models, built the same way as the Starship prototype, so that they hold up in the external views at the moments the user named:
- the landing legs deploying during a Falcon 9 landing;
- Crew Dragon's hinged nose opening, and its docking mechanism;
- the ISS docking rings for Crew Dragon, as exact as possible.

Docking manoeuvres become visible later, first through the external cameras that exist today, later from the pilot's seat. This plan prepares the moving docking mechanism for that, seen at close range; it does not build the cockpit view or new cameras.

## Goals
1. **Falcon 9 first stage** drawn with the interstage: 47.7 m in total [S: Wikipedia, 156.5 ft], tank section about 41.2 m, interstage about 6.5 m. Four grid fins on the interstage, octaweb with nine Merlins, four landing legs that visibly deploy from the sim's `deployment` value.
2. **Falcon 9 second stage**: 13.8 m with the MVac nozzle. No fairing (user 2026-11-01): Dragon sits directly on the second stage.
3. **Crew Dragon with trunk**: capsule, trunk with its solar and radiator surfaces, SuperDraco and Draco pods, nose opening on the hinge from the sim's `nose_cone_open` (animated by the view between its two states), the IDSS docking mechanism modelled in detail.
4. **Moving docking mechanism prepared now** (user's choice): soft-capture ring extending, capture, retraction, hooks closing, as separate driven parts. A demo mode in the test scene runs the sequence. The parts expose one small interface (ring extension, ring attitude, hook state) so the later docking visuals only connect it to the sim's docking state.
5. **ISS**: simple overall (modules, truss, radiators, solar arrays with their rotating joints as separate nodes), with Harmony, PMA-2/IDA-2 and PMA-3/IDA-3 as the detailed area: the passive IDSS rings as exact as references allow.
6. **Port positions**: measured from the NASA model and corrected in `config/vehicles/iss.toml` if they differ, with old and new stated.

## Non-goals
- Fairing, Orion, European Service Module (user 2026-11-01: "Erstmal nur Dragon mit Trunk").
- Depot, tanker, HLS: afterwards, in their own plan (orchestrator's default, the user can overrule).
- Showing docking from the sim's real state, new camera positions, pilot's seat view, cockpit.
- Dragon interior, drone ship rework, plumes beyond re-anchoring to the new nozzle planes.
- Any change to the frozen ship aero, or to the reds in `docs/known-issues.md`.

## Anti-goals
- A copied third-party mesh in the repo or the game. Every model is our own rebuild.
- Surface detail as geometry where texture or shader can do it: geometry only for the silhouette and for what moves or is seen at docking range. Exception: the docking rings and hooks, where the user asks for exactness.
- Trademarks or logos: no SpaceX, NASA or flag markings copied; display names come from config.
- The prop going stale: a part that looks right but moves differently from what the sim says.

## Standing decisions
- **Rebuild everything real** (user 2026-11-02). Every model we can get is a measuring reference, measured by script; photos and real data win when they disagree. Meshes stay reference material outside the repo; a model's terms are checked before download.
- **Same pipeline as the Starship prototype** (`docs/subsystems/vehicle-models.md`): physics config is the master, `config/vehicles/detail/<id>.toml` holds the rest and repeats no physics key, `tools/structures/` generates glTF via Blender into git-ignored `data/models/<id>/` with a manifest, `vehicle_models.gd` and `vehicle_view.gd` load and pose. The single `PROTOTYPE_VEHICLE_ID` switch becomes a list.
- **Simple hulls, detail by texture and shader.**
- **Provenance tags** [S]/[E]/[D] for all new values; sources in `tools/data_pipeline/sources.toml` only if something ships, reference URLs in the detail files.
- **Moving parts follow the sim**: the game only reads state (`body_legs`, `nose_cone_open`, later `docking_state`); no physics in GDScript.

## The first stage's physics data follow the drawing (user 2026-11-02)
`config/vehicles/falcon9_block5.toml` has `length_m = 41.2` for the whole first stage and grid fins at station 39.0. The stage becomes 47.7 m and the fins move to the interstage, near its top. Masses and tank contents stay: the dry mass already includes the interstage. Dry CG and tank stations are reviewed, changed only if the interstage demonstrably moves them, and marked. This changes the stack height and the lever arms of the grid fins. Falcon 9 ascent, RTLS, drone ship and the mission campaigns run before and after, in old value, new value and limit; a drag table keyed on reference area is not refitted. If a validated Falcon 9 result breaks and the cause is not plain, stop and report rather than tune.

Docs and config comments that state 41.2 m as the stage length are corrected. `tests/` values follow only where they are the geometry itself.

## Order
1. Search for measuring references: Sketchfab, Thingiverse, GrabCAD and similar (not yet checked; NASA has no Dragon or Falcon). Terms checked, scripts measure, findings in `screenshots/falcon9_ref/INDEX.md`.
2. Falcon 9 first and second stage, physics follow-up, legs.
3. Crew Dragon with trunk, nose, docking mechanism and its demo.
4. ISS, measured from NASA model D (IGOAL) by script; port positions; rings.

Each step ends with renders in `screenshots/<topic>/`, sent to the user to compare with photos; the user's verdict on the look closes it. Reference photos: `screenshots/falcon9_ref/`. Gaps listed there (no clean photo of Harmony's ports, Draco pod angles not public) are filled with the best estimate and marked [E].

## Success
- In a Falcon 9 landing the legs deploy from the sim's value and the feet stand where the contact physics puts them.
- With the nose open on orbit and in the demo, Dragon's mechanism and the ISS rings match the references at close range in the existing external views, and the two rings meet on the sim's port axes and positions.
- The demo runs the docking sequence, moving only through the small interface.
- Frame rate: the three models stay within the VR budget used for the Starship prototype; measured with `vehicle_cost_check.gd` or its equivalent, ISS included.
- Falcon 9 and Dragon missions are not less valid than before: every result above stated with old value, new value and limit.

## Orchestrator's calls (not asked of the user; the user can overrule)
- **Datum:** station 0 stays at the stage's base; tank and engine stations stay; the second stage and Dragon move up 6.5 m with the stack. Fin station measured from references and marked [E].
- **Limits:** "limit" means the existing test's tolerance. No new tolerances are invented; a value without a test is reported as old and new.
- **Reality wins over config:** legs, second stage, capsule (4.4 vs 4.5 m), trunk: if a reference disagrees with the config, the config changes, is marked, and goes through the same before and after runs. The same holds for port positions. A config value is not drawn wrong to avoid a re-run.
- **Legs:** the sim's `deployment` value drives the real mechanism, drawn as the real legs fold; the contact physics' feet and the drawn feet must coincide when deployed.
- **Ring meeting:** the rings meet on the ports' config data on both sides (`crew_dragon.toml`, `iss.toml`); the correction goes to whichever side the measurement shows wrong.
- **Solar arrays:** the sim has no array angle, and tracking in the view is logic the game must not hold. So the arrays stand at a fixed pose for now, their joints as separate nodes, ready for a later sim value.
- **Trunk:** a separate part, following the stage list as stage separation does today.
- **Verdicts:** the plan runs through its tasks without waiting. Renders go to the user after each step; the user's remarks are worked in as they arrive, and no step blocks the next. The plan ends with the user's overall verdict.
- **Fallbacks:** the NASA ISS model D is already downloaded (`screenshots/falcon9_ref/models/iss/d_igoal/`). The IDSS interface document is the source for the rings' dimensions. If no usable Falcon or Dragon model turns up, photos and published dimensions carry the drawing, marked [E].

## Left to the builder
Mesh construction, triangle counts within the budget, node names, the demo's form, how much of the ISS's small hardware is left out, generator file layout, the task cut.
