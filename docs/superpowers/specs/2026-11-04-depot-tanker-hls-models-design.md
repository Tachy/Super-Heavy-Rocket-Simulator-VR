# Depot, tanker, HLS: detail models

## Intent
The refuelling campaign and the lunar mission still show today's cylinders for the depot, the tanker and the HLS (and
for the HLS on its booster). These ships get real models, built in the pipeline of the Starship prototype and the
Falcon 9 / Dragon / ISS plan (2026-11-02), so that they hold up in the external views at the moments that matter:
- the tanker or the HLS docking dorsal-to-dorsal under the depot, probes in drogues, transfer connectors meeting;
- the HLS landing on and lifting off the Moon: legs on the ground, landing thrusters high on the hull firing;
- the depot alone on orbit as the long-lived, insulated, finless ship it is.

The user handed every design choice to the orchestrator (2026-11-04: "egal, leg los"). Everything below the goals is
the orchestrator's call; the user can overrule any of it.

## Goals
1. **Tanker**: the Block 3 ship model (tiles, flaps, chine as today) plus four dorsal docking probes and the two
   transfer connectors at the config positions (`starship_tanker.toml` `[[stage.docking_port]]`, `[[stage.transfer_connector]]`),
   no payload door. Its booster is the Block 3 booster model.
2. **Depot**: the Block 3 hull without heat shield tiles, flaps, chine and catch pins (the depot never re-enters), an
   insulated light skin by texture, four dorsal drogues and the two connectors at the config positions, the RCS pods at
   the config stations.
3. **HLS**: the Block 3 hull without tiles, flaps, chine and catch pins; white skin; a solar band below the nose; crew
   windows; the airlock / elevator hatch with the exterior elevator track and basket; the landing thrusters at the config
   stations; six fixed legs drawn from `[stage.landing_legs]` (azimuths, footprint radius, foot station); the dorsal
   probes and connectors at the config positions; a nose docking port. The same HLS model on its booster
   (`starship_hls_launch`).
4. **Docking hardware moves with the sim**: probes extend / retract as driven parts through one small interface like
   the Dragon mechanism of 2026-11-02 (extension, capture state). Where the sim reports a docking state it drives them;
   otherwise they rest stowed. A demo in the test scene runs the sequence on a docked tanker / depot pair.
5. **Taken-over stages pick the right model**: a stage taken over with `from = "<vehicle>:<stage>"` (tanker booster,
   HLS-launch booster, HLS-launch HLS) shows the model of the vehicle it comes from, or its own where it differs (tanker
   ship).

## Non-goals
- Interior of the HLS, elevator motion and airlock door motion driven by the sim (no sim state exists; the parts are
  separate nodes at a fixed pose, ready for a later value - as the ISS arrays were).
- New cameras, cockpit, the procedural lunar surface, plumes beyond re-anchoring to the drawn exit planes and the
  landing-thruster exits.
- Any change to the Block 3 ship / booster models beyond what the variants need, to the ship aero, or to the reds in
  `docs/known-issues.md`.
- A sunshield or solar array on the depot (no source describes one: not invented).

## Anti-goals
- A copied third-party mesh in the repo or the game. Every model is our own rebuild; reference models are only measured.
- Surface detail as geometry where texture or shader can do it. Geometry for the silhouette, legs, thrusters, the
  docking hardware (exact at docking range) and the elevator.
- Logos, trademark lettering, flag or agency markings; display names come from config.
- A prop that disagrees with the sim: drawn probe tips / drogue mouths, connector faces, leg feet and thruster exits
  must coincide with the config points the physics uses.

## Standing decisions
- **Rebuild everything real; reality wins over config** (user 2026-11-02). Photos and real data win over the config
  where they exist; a disagreeing config value is corrected, tagged, and goes through before / after runs of the
  campaigns that use it. Where no reality data exist (V3-era HLS legs, drogue positions) the config values (sized by our
  physics) are drawn and the gap is marked [E].
- **Same pipeline** (`docs/subsystems/vehicle-models.md`): physics config is the master; `config/vehicles/detail/<id>.toml`
  holds the rest and repeats no physics key; `tools/structures/blender/generators.toml` lists the generators; glTF +
  manifest into git-ignored `data/models/<id>/`; `vehicle_models.gd` / `vehicle_view.gd` load and pose.
- **Variants, not copies**: the Block 3 ship generators gain switches (tiles, flaps, chine, catch pins, payload door,
  dorsal hardware, HLS parts) read from the detail files; the Block 3 output stays byte-identical (manifest hash test).
- **Planner absorbs vehicle changes** (user 2026-11-03): if a config correction from the drawing turns a campaign red,
  the planner is fixed so the flight works - no test weakened; an unplain break: stop and report.

## Order
1. References: Flight 12-14 webcast frames (`data/webcasts/`) and the press photos listed in
   `screenshots/depot_tanker_hls_ref/INDEX.md` for the V3 docking drogues (count, placement - nose vs dorsal), NASA
   HLS renders and mockup photos (airlock, elevator, legs, solar band), measured by script or by scaling; findings back
   into `INDEX.md`. Baseline of the affected campaigns on the current tree.
2. Pipeline: variant switches, taken-over stages, stubs for the three detail files.
3. Tanker (docking probes, connectors) - render to the user.
4. Depot (finless insulated hull, drogues) and the docked pair with the demo - render to the user.
5. HLS (legs, thrusters, solar band, windows, elevator, nose port), on the Moon and on its booster - render to the user.
6. Cost check, docs, final review.

Each vehicle step ends with renders in `screenshots/depot_tanker_hls/` sent to the user; remarks are worked in as they
arrive, no step waits for a verdict. The plan ends with the user's overall verdict.

## Success
- In the refuelling campaign's docked state the probes sit in the drogues and the connectors meet on the config points;
  the demo runs the sequence through the small interface.
- On the Moon the HLS feet stand on the contact physics' foot points, the landing-thruster plumes leave the drawn exits.
- Separated / taken-over stages never carry the wrong model; a missing or stale model set gives one log line and the
  cylinders.
- Frame rate: depot + tanker docked, and the HLS on the Moon, within the VR budget of the prototype
  (`vehicle_cost_check.gd`).
- Refuelling and lunar campaigns not less valid than before: every changed result as old value, new value and limit.

## Left to the builder
Mesh construction, triangle counts within the budget, node names, the demo's form, generator layout, the task cut,
which affected test tags form the baseline.
