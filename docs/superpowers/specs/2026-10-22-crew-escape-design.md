# Crew escape: ejection seats on the ship return - design

## Intent and why
A realistic simulator needs contingencies (user, idea of 2026-10-18). When a crewed Starship return cannot end in a
catch, the ship is lost - a water landing with crew is no option because the ship usually explodes (user). So every
abort ends with the crew leaving the ship on ejection seats. The simulator follows each crew member from the seat
firing through the parachute descent to touchdown - the game will later show it from the pilot's point of view - and
follows the empty ship on until it strikes the sea. This milestone is data only: physics in the core, state through
the game interface; rendering comes later (user).

## Goals
1. **Trigger (onboard computer only, user):** every abort of a crewed ship return arms the seats.
   - During the entry (new, user 2026-10-22: "early, over water"): the onboard computer checks continuously whether
     a catch is still possible - enough usable landing propellant and enough working landing engines. When it is
     not, it gives the ship up early: the entry guidance retargets beyond the coastline into the sea (the existing
     sea-target path of uncaught returns is the template), no flip follows, the ship stays in the belly flop.
   - At the gate (before the flip): the same check once more; a failure found only here gives the ship up there.
   - After the flip: every existing abort - SCvx abort to the sea, upright descent latched, the abort command -
     arms the seats.
2. **Firing (user 2026-10-22):** an early give-up fires the seats over water - once the ship is inside the seat
   envelope (height, Mach, dynamic pressure; zero-zero capable) and beyond the shoreline, at the latest at a minimum
   firing height [E, chosen so the canopy is fully open well above the ground]. A give-up at the gate or after the
   flip fires at once (inside the envelope), even if ship and crew then come down on land - reported honestly.
   A ship that never enters the envelope (breaks up, impacts first) reports the crew as lost - no fudging.
3. **Every crew member simulated (user):** crew count and seat positions from config; each seat is its own body.
   Phases per member: catapult -> seat rocket -> drogue -> seat-man separation -> main canopy -> descent ->
   touchdown. Seats fire in a staggered sequence so their trajectories diverge.
4. **The empty ship simulated on (user):**
   - Without a flip: the flaps fly the belly flop down and steer it away from the tower and the coast into the sea;
     impact point, impact speed, distance to the tower and to the shoreline.
   - After the flip: the existing abort / upright descent flies on without the crew's mass to the sea; same report.
5. **Game interface:** a session-wide state (like `lunar_state`, `iss_state`): armed, abort reason, firing time,
   per member position/velocity (GCRF and lat/lon/height), attitude, phase, current and peak load (g), canopy state,
   touchdown point (water or land), outcome; the ship's impact report. Each crew member is also an ordinary body, so
   the existing per-body getters work. The ship's flip/catch/abort state and `ReturnResult` reach the bindings too
   (today they do not).
6. **Outcome:** per member rescued / lost with a reason (outside envelope, load limit exceeded, struck the ship /
   plume / tower, canopy failed, touchdown too hard, landed on land is reported but not a loss by itself);
   `ReturnResult` of the ship becomes "lost" for every abort of a crewed ship.

## Non-goals
- Aborts during the ascent (later, user); crew escape on the Moon; Crew Dragon (has its own capsule).
- A pilot-triggered abort button for the seats (the onboard computer decides, user).
- Wind (M8), waves, buoyancy, drift after splashdown, recovery of the crew.
- Rendering, cockpit views, UI (later; UI by the user).

## Anti-goals
- A crew that "always survives" by tuned numbers; firing outside the envelope reported as survivable.
- Physics in GDScript; any change to the catch success rates of the existing campaigns.

## Constraints (from the project)
- Physics first, SI internally, `[S]/[E]/[D]` provenance on every number, constants with sources, config not code.
- Small modules: seat sequencing in `gnc`, seat/person bodies via config (`config/vehicles/`), session wiring in `sim`,
  bindings thin.
- Uncrewed ships (tankers, depot, test ships) are unchanged; only a crewed mission/vehicle arms the system.
- Existing campaigns (ship catch 20/21 + upright abort, lunar direct 21/21, aerocapture 21/21, refuelling) keep their
  numbers for the catch; they stay uncrewed, so their aborts keep the upright abort to the sea.

## What exists to build on (recon 2026-10-22)
- Ship return: `core/src/sim/flight_session.cpp` gate hand-over (~886-909, height only, no "cannot catch" check),
  `gnc::ShipLandingGuidance` (`core/src/gnc/ship_landing.cpp`: flip, SCvx catch, abort to sea, `upright_descent`),
  `FlightSession::abort_landing`, `sim::ReturnResult` (`core/include/starsim/sim/landing_outcome.hpp`), sea side from
  `config/launch_sites.toml` `[site.sea]` (`descent_snapshot.cpp` sea target). Lunar direct and aerocapture returns
  end on the same path (`flight_session_lunar_return.cpp`).
- Parachutes: `[[stage.parachute]]` (drogue/main, reefing, fill time, load limit), `aero/parachute.hpp`,
  `gnc::ParachuteSequencer`, created automatically for bodies whose lowest stage has parachutes; `[stage.recovery]
  method = "splashdown"`; fixed-area drag (`aero::FixedAreaAero`). No pendulum canopy, no wind.
- Bodies: `FlightSession::bodies_` (`Flying`), free bodies from a config as in `add_station`; bindings snapshot
  pattern (`bindings/src/flight_snapshot*.cpp`, `flight_sim.cpp`).
- Crew today: only `crew_load_limit_g` (Dragon) and `FlightBody::crew_load_warning`; no seats, hatches or crew count.
- Ground: flat at the pad height; "water" = beyond the shoreline line; impacts hold the wreck.
- Missing: a solid seat-rocket / catapult thrust model (engines are tank-fed liquids).

## Decided by the user
- Seats fire on every abort, never a water landing with crew; return only (ascent later); onboard computer triggers;
  every crew member computed; the empty ship computed to its impact; data + game interface first, visuals later;
  the builder chooses the check limits so that only sensible data comes out.

## My calls (builder may refine, must document)
- **Envelope:** an ACES II-class zero-zero seat [S: ACES II envelope 0-50,000 ft, 0-600 KEAS]; firing needs height
  below the envelope ceiling and equivalent airspeed / q below its limit; the belly flop (~90 m/s, q ~5 kPa) is well
  inside.
- **Seat model:** gas catapult on rails + short solid rocket as a time-thrust table (new small "solid motor"
  component, no tank), seat+person one body until seat-man separation, then person+canopy body; the empty seat is
  dropped from the session after separation. Drogue and main reuse `[[stage.parachute]]` + `ParachuteSequencer`;
  main canopy a round personnel chute, sink ~6-7 m/s [S to be sourced].
- **Exit direction:** through hatches on the leeward (dorsal) side of the belly flop; after the flip that is
  horizontal, so the rocket must carry the seat up and away from the hull, the engine plume and the tower.
- **Crewed missions:** a mission flag (`crewed`, crew count) arms the system; the existing missions and campaigns stay
  uncrewed and unchanged (their aborts keep the upright abort to the sea). New crewed variants of the orbit return
  and the lunar direct return carry the tests.
- **Gate/entry feasibility check:** crewed ships only; uncrewed ships keep today's flip + upright abort.
- **Crew count:** 4 by default [E], configurable per crewed mission; seats + crew are carved out of the ship's
  existing dry mass (no change to the masses of any existing flight) and leave the ship on firing.
- **Stagger:** fixed interval between seats [E ~0.3-0.5 s, sourced from multi-seat sequences].
- **Load check:** peak acceleration per axis against published seat limits (e.g. spinal ~ catapult/rocket peaks,
  opening shock) [S]; exceeding = lost/injured reason reported.
- **Collision checks:** minimum distance of each seat to the ship hull, the plume cone (engines burning) and the
  tower geometry during the first seconds; a violation = lost.
- **Belly-flop divert:** after a give-up the entry guidance aims beyond the shoreline + margin; below the gate a
  flap steering law holds the belly flop and keeps turning the ship seaward until impact. A give-up at the gate may
  not reach the sea (the ship approaches from the west over land, the coast is 500 m east of the pad) - the impact
  point is reported as it falls, not forced.

## Left to the builder
- Exact envelope numbers, seat/rocket/catapult tables and canopy data with sources; gate feasibility criterion (reuse
  `upright_stop_propellant_kg`, landing propellant need, engine availability); module/file split; test cases
  (unit + reference: canopy sink rate, seat apex at zero-zero, injected-failure return campaigns per return type);
  snapshot field names; `docs/subsystems.md` section and `docs/ansichten.md` note.

## Verification (for the plan)
- Unit tests per model (solid motor table, seat sequencer, envelope, gate feasibility, divert law).
- Reference cases with tolerances: zero-zero apex height and canopy sink rate vs published seat data; opening shock.
- Scenario tests: crewed orbit return and lunar direct return with (a) a landing engine failed during the entry ->
  early give-up, ship and crew down in the sea, (b) a failure found only at the gate, (c) a catch abort after the
  flip -> crew rescued with loads/clearances logged and touchdown points (water/land) reported; one case where the
  envelope is never reached -> crew lost.
- Unchanged: affected catch campaigns (`[ship][catch]`, lunar direct/aerocapture) keep their catch counts.
- Godot smoke test with the new getter.
