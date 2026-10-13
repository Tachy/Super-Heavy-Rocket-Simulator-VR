# Falcon 9 first-stage landing: pad (RTLS) and drone ship - design

Status: draft 2026-10-13. Branch `feat/scvx-flight`, milestone (c) of the SCvx flight plan's follow-ups.

## Intent

The user (2026-10-13): "Super Heavy wird immer gefangen, Falcon 9 Booster landen auf dem Pad oder auf dem
Drohnenschiff". Milestone (c) flies a Falcon 9 Block 5 first stage back after staging and lands it on its legs:
on a landing zone at the Cape (RTLS) or on a drone ship at sea (ASDS). As for Starship, the autopilots fly every
mission step; the player may steer by hand and the onboard computer re-plans the setpoint (memory
`project-autopilot-control-levels`).

Decisions accepted by the user 2026-10-13 ("und los" on the proposals):
- RTLS first, then the drone ship.
- The drone ship holds its position and its deck is level (station keeping, no sea state). Sea state
  (pitch/roll/heave) is a later, switchable option and is out of scope here.
- Only Block 5 is modelled, analogous to "only Block 3" for Starship. Its data come from flights that flew it.

Success:
- A Falcon 9 lifts off from SLC-40 with a light payload. Its first stage flips, boosts back, flies the entry burn
  and a single-engine landing burn, and stands on its legs on the landing zone ("gelandet", < 5 m from the centre).
- A heavy mission (Starlink class) lands its first stage on the drone ship deck without a boostback.
- The unguided descent of the stage reproduces the webcast telemetry of one RTLS and one ASDS flight within the
  stated tolerances. The landing campaigns land all dispersed cases.

## Decomposition

Each part has its own implementation plan and ends with a testable deliverable.

| Part | Content | Validation |
|---|---|---|
| c1 | Falcon 9 vehicle model complete: engine groups (centre / 3-engine set / ring), grid fins (4), cold-gas RCS, landing legs geometry, descent drag table; launch site SLC-40 and a landing zone; webcast telemetry of one RTLS and one ASDS flight | Ascent to orbit on the existing autopilot; unguided descent replay vs telemetry |
| c2 | Landing legs: deployment, four feet with crush-core contact, touchdown judgement (gelandet / umgekippt / Aufprall) | Drop tests: gentle, hard, tilted, sliding |
| c3 | RTLS: return guidance generic over the vehicle (engine sets from config), entry-burn phase, SCvx landing to a pad (no tower) as a hoverslam on one Merlin, mission Falcon 9 RTLS | Landing campaign from ignition states; full mission lands |
| c4 | Drone ship: deck at sea (stationary, finite deck, off the deck = water), ASDS return (no boostback, entry burn steers), mission Falcon 9 ASDS | ASDS campaign; full mission lands on the deck |
| c5 | Game: choose the vehicle and mission, draw legs, landing zone and drone ship, HUD lines, docs | Smoke test, render check |

This document fixes the design of all five parts. Each plan refines its part; decisions taken there are recorded in
the plan's ledger.

## What exists and what changes

- `gnc/booster_return` already flies the flip, a Lambert boostback, the coast with grid fins and RCS, and a landing
  burn: explicit laws, or SCvx at a catch site. Its default engine sets name Super Heavy groups
  (`"booster/center"` ...). Change: the sets come from `[stage.recovery]` of the vehicle, and the defaults stay as
  they are for Super Heavy.
- `FlightSession::returns_after_staging` requires `recovery_method == "tower_catch"` and a tower. Change: a
  `"leg_landing"` stage (replaces `"drone_ship_landing"` in the Falcon 9 config) returns to the mission's landing
  target, a landing zone or a drone ship.
- The descent planner (`gnc/descent_problem`) has two goals: catch, and sea abort. Change: it gets a third goal,
  touchdown on a pad (a point on flat ground; final state upright, at rest at leg contact height, no keep-out).
- `FlightBody` ground contact treats the bottom point as the only contact (gentle <= 3 m/s and <= 10 deg rests,
  else impact). Change: for a vehicle with legs, `contact/landing_legs` replaces it, and the ground becomes a
  surface (pad plane, or deck over sea).
- The Falcon 9 config models nine Merlins as one group and has no legs, grid fin geometry or RCS.

## c1 design: vehicle and evidence

### Engine groups
Octaweb [S]: 8 engines on a ring plus a centre engine. The groups are:
- `center` (1): the landing engine.
- `pair` (2, opposite on the ring): with `center`, the 3-engine set for boostback, entry burn and an ASDS landing
  burn that starts on three.
- `ring` (6): the rest.

Ring radius [E] from imagery. Merlin 1D [S]: 845 kN at sea level, minimum throttle 40 %. One engine at 40 % gives
~300 kN, more than the stage's weight at landing (~27-30 t, ~280 kN). **The stage cannot hover:** the landing burn
must reach zero speed at leg contact (a "hoverslam").

### Grid fins, RCS, legs
- **Grid fins:** 4 at 0/90/180/270 deg near the top (titanium, ~1.2 x 1.5 m each [E]). They reuse `aero/grid_fin`
  and its table, with the area and station from config.
- **RCS:** nitrogen cold gas, two pods at the top [E], thrust per thruster [E].
- **`[stage.landing_legs]`:** 4 legs at 45/135/225/315 deg [E]. Values (all [E] from imagery and public figures,
  verified in c1):
  - footprint radius ~9 m (span ~18 m)
  - foot height below the engine bell exit
  - deployment time ~2 s, triggered at a height
  - crush-core stroke and crush force
  - leg mass, included in the dry mass [S] 25.6 t

### Descent aerodynamics
The body is tail-first, a 3.66 m cylinder with legs stowed. Its table `aero/falcon9_descent.csv` is identified from
the telemetry as for Super Heavy: `tools/telemetry/drag_estimate.py`, ballistic coefficient over Mach. Where the
telemetry does not cover a Mach range, the value is the slender-body cross-flow estimate [E]. Deployed legs add drag
[E] for the last seconds.

### Sites
- **Launch site `ccsfs_slc40`:** no tower. Coordinates [E] from imagery.
- **Landing zone:** the current Cape landing zone for Falcon 9 RTLS. The c1 plan verifies which zone is in use in
  2026 (LZ-1/LZ-2 were retired in 2025; their successors sit at the launch pads) and records the source.
- **Config:** a landing zone is a `[[site]]` with `kind = "landing_zone"`, a pad radius and a `[site.sea]` if the
  coast is near. Missions name their launch site and landing target separately.

### Evidence
SpaceX webcasts show first-stage speed (km/h) and altitude (km) as text until touchdown on RTLS and ASDS flights.
`tools/telemetry/webcast_telemetry.py` gets a Falcon 9 layout (its text-strip reader is the starting point).
- **Flights:** one recent RTLS flight and one recent Starlink ASDS flight, chosen in the c1 plan by availability and
  documented landing.
- **Recorded data:** sources and licence notes go into `tools/data_pipeline/sources.toml`; flight facts (booster
  serial, landing zone or ship, payload class) go into `flights.toml`.

What the telemetry yields (each tagged [D]):
- MECO speed and altitude
- boostback length
- entry-burn start and end (speed and altitude)
- landing-burn ignition (height and speed) and burn duration
- the ballistic coefficient between the burns

Propellant at landing-burn ignition is not shown in the webcast. It is derived from the burns (rocket equation on
the observed delta-v) [D].

### Validation (c1 deliverable)
- **Ascent:** the Falcon 9 stack reaches a 200 km orbit on the existing ascent autopilot (PEG). Payload of the
  validation flight, staging and MECO times within 10 % of the telemetry.
- **Descent replay:** the stage, unguided from after the entry burn to landing-burn ignition, follows the telemetry
  speed over altitude within 3 % rms on both flights. It loses altitude to the ignition point within 300 m. These
  are the tolerances of the Super Heavy replay.

## c2 design: landing legs and touchdown

Approaches considered:
1. Extend the bottom-point contact with a wider tilt limit. Cheap, but it cannot tip over, slide or bounce, and
   it cannot judge a deck edge.
2. **Four feet as penalty contacts with a crush-core stroke (chosen).** This is the same technique as the catch pins
   (`contact/pin_contact`).
3. Full multibody legs. More than the evidence supports.

Model (`contact/landing_legs`):
- Each foot is a point on a deployed leg. Its normal force is k x penetration + c x penetration rate while the
  crush core is intact. Once the force exceeds the crush force, the core crushes plastically: constant force,
  stroke consumed, no rebound.
- Coulomb friction 0.6 [E] on concrete and on the deck.
- Forces go through `FlightBody::apply_external_wrench`. The surface comes from `contact/landing_surface`:
  - a plane at a height (pad)
  - or a rectangle (deck) over a lower plane (sea): a foot over water gets no support.

Judgement (`sim/landing_outcome`, extended):
- **gelandet:** engines off, all four feet in contact, at rest for 2 s. The body is then held fixed to the Earth or
  deck (`hold_fixed_to_earth`).
- **umgekippt:** the CG passes outside the support polygon, or the tilt exceeds 30 deg.
- **Aufprall:** a stroke is exhausted, or the hull or bell touches the surface first.
- **Wasserung:** contact with the sea.

Legs stowed: the old bottom-point contact applies.

Validation:
- **Drop tests (gentle):** 1, 3 and 6 m/s sink, upright, give "gelandet". The 6 m/s case uses some stroke, tagged [E].
- **Hard drop:** 10 m/s gives "Aufprall".
- **Tilted:** 2 m/s at 5 deg gives "gelandet"; 25 deg gives "umgekippt".
- **Sliding:** 2 m/s lateral gives "gelandet".
- **Deck edge:** one foot over water gives "umgekippt" or "Wasserung".
- Energy check: the crushed stroke times the crush force equals the kinetic energy lost, within 5 %.

## c3 design: return to the landing zone (RTLS)

### Guidance (`gnc/booster_return`, generic)
Phases: Flip -> Boostback -> Coast -> **EntryBurn** (new) -> Coast -> LandingBurn -> Landed.

- **Engine sets:** from `[stage.recovery]`: `boostback_engine_groups`, `entry_engine_groups`,
  `landing_engine_phases` (list of sets: Falcon 9 RTLS `[["first_stage/center"]]`, Super Heavy keeps 13 -> 5 -> 3).
- **Entry burn:**
  - *Purpose:* it limits the dynamic pressure and heating of the descent. Super Heavy has none.
  - *Ignition:* at a height, ending at a speed [D from the telemetry].
  - *Steering:* during the burn it steers along the drag-aware impact error, like the boostback's v_go. That is a
    small correction; the grid fins do the rest.
  - *MECO planning:* `meco_propellant_kg` adds the entry burn (rocket equation on its planned delta-v).
- **Landing burn:** SCvx with the pad goal. Phases from the config, minimum throttle 40 %.
  - *Plan end:* at leg-contact height with a sink rate of 1 m/s [E], upright, lateral speed 0.
  - *Cutoff:* on leg contact, or at plan end + 0.5 s.
  - *Re-plans:* every 1 s as for Super Heavy; none in the last 3 s.
- **Lateral control:** the LQR from `gnc/tvc_lateral`.
- **Ignition timing:** `predict_landing_burn` with the configured profile. A hoverslam has no slack: ignition must
  leave throttle margin both ways, so the design throttle is ~70 % [E] on one engine.
- **Leg deployment:** at 2 s + margin before the planned touchdown, by height [E].

### Failure handling
Divert to the sea is the RTLS abort:
- *Trigger:* the landing zone is unreachable after two consecutive failed re-plans. This mirrors the Super Heavy
  sea abort.
- *Target:* a sea point beyond `[site.sea]`, reached with the pad goal at sea level ("Wasserung").

### Mission (`config/missions/falcon9_rtls.toml`)
- Launch SLC-40, light payload [E] (~ 12 t class, the RTLS payload range), 200 km orbit.
- Return to the landing zone; `landing_propellant_kg` [D] from c1.

### Validation (c3 deliverable)
- **From the ignition state** (from the c1 telemetry): lands < 1 m off the centre, sink at contact < 2 m/s.
- **Campaign `[falcon9][rtls][validation]`:** +-10 m, +-3 m/s, +-5 % mass, +-3 deg, truth aero +-10 %. All
  cases land; propellant left is reported.
- **Full mission:** liftoff to "gelandet" in one session. The landing-burn ignition state lies within the
  telemetry's spread.

## c4 design: drone ship (ASDS)

### Drone ship (`config/drone_ships.toml`, `contact/landing_surface` deck)
- **Deck:** 91 x 52 m [S: published dimensions of the drone ships], height above the sea ~3 m [E], heading [E].
- **Position and motion:** stationary in the Earth-fixed frame (station keeping); no sea state.
- **Recovery point:** set per mission. It is the nominal impact point of that mission's trajectory: real recovery
  zones are placed that way. It is derived from our nominal ascent and tagged [D].
- **Display name:** generic "Drohnenschiff" (trademarks internal only).

### Return
- No flip and no boostback. The stage coasts on its ascent arc, engines first (attitude flip with RCS).
- The entry burn on three engines steers the impact onto the deck.
- The landing burn starts on three engines and ends on one (`[["first_stage/center", "first_stage/pair"], ["first_stage/center"]]`; switch window [E]).
- The goal is the deck centre at deck height.
- *Abort:* a target beside the ship (water).

### Mission (`config/missions/falcon9_asds.toml`)
- Starlink class payload [E] (~ 16-17 t), launch SLC-40, drone ship ~ 600 km downrange [D from c1 telemetry/ascent].

### Validation (c4 deliverable)
- **Campaign `[falcon9][asds][validation]`:** as for RTLS plus dispersed MECO states; all land on the deck.
- **Full mission:** lands on the deck.

## c5 design: game

- **Vehicle and mission choice:** a start menu or keys picks the mission; F9 RTLS and F9 ASDS appear next to
  Starship.
- **Drawing:** `vehicle_view.gd` draws the stage with grid fins and legs (deployment angle from telemetry), a
  landing zone marker, and the drone ship (box hull + deck).
- **HUD:** landing phase, legs, outcome.
- **Docs:** `docs/ansichten.md` and `CLAUDE.md`.

## Out of scope
- Sea state on the drone ship (later, switchable).
- Falcon Heavy side boosters.
- Second-stage recovery and fairing recovery.
- Block 4 and earlier blocks.
- Landing-leg failure modes other than overload.

## Rulings already taken (please check)
- **Landing zone:** chosen in c1 from current sources (not LZ-1/2, retired 2025). Cost if wrong: one config entry.
- **ASDS landing burn:** 3 -> 1 engines. RTLS: 1 engine. Both [E] from public descriptions; the telemetry decides the
  timing. Cost if wrong: one config list.
- **Recovery point:** the drone ship is placed at the mission's nominal impact point [D], not at a historical
  coordinate. Cost if wrong: the ASDS mission depends on our own ascent.
- **Leg contact:** crush cores modelled as plastic stroke, friction 0.6 [E]. Cost if wrong: different touchdown
  tolerances; parameters are config.
