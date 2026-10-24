# Sea state and drone-ship deck motion - design

Date: 2026-10-24. Milestone order: the small part directly after M8 part 1 (user 2026-10-23: "deck motion (sea state,
fidelity flag, onboard refusal in rough sea) as its own small part"). The user asked to start without questions
("leg direkt los"): every decision below not quoted from the user is Claude's call, marked **[call]**.

## Intent

Falcon 9 first stages land on a drone ship that today holds a level, motionless deck over a flat sea. Real landings
happen on a barge that heaves, rolls and pitches in the swell; in rough sea the recovery is called off before launch.
This part makes the deck move with a physically derived response to a configurable sea state, lets the legs land on
that moving deck, has the onboard computer refuse a drone-ship recovery when the sea is too rough, and draws the
drone ship (it is not drawn at all yet) riding the waves in the external view and VR.

## Goals

1. **Sea state** (core, `environment`): a long-crested wind sea / swell described by significant wave height Hs, peak
   period Tp, the direction the waves travel towards and a seed. A JONSWAP spectrum (gamma 3.3 [S: DNV-RP-C205]; Hs = 0
   is a calm sea) discretised into N components (N ~ 30-60, [call]) with deterministic random phases from the seed;
   deep-water dispersion omega^2 = g k. Surface elevation and slope at a point and time, and the spectral moments.
2. **Deck motion** (core, `contact` or `vehicle` - builder's choice where it fits the module rules): the barge's heave,
   roll and pitch (plus their rates and accelerations) at a time, from the sea state, per component:
   - quasi-static excitation: the least-squares plane fit of the wave over the hull's waterplane rectangle (Froude-Krylov
     for a wall-sided box): mean elevation -> heave, mean slopes along the long / short axis -> pitch / roll; analytic
     per component (sinc-type factors), checked against numeric quadrature;
   - dynamic response: a second-order system per degree of freedom (natural period from hydrostatics: heave from
     waterplane area and displaced mass with an added-mass factor; roll / pitch from the metacentric heights and radii
     of gyration of a box barge; damping ratios [E]); linear superposition of all components.
   - Surge, sway and yaw: none - the ship holds station on its thrusters [call].
   Data in `config/drone_ships.toml` with provenance: hull length 91.4 m / beam 30.5 m (Marmac 300 class barge
   [S: wiki_asds], the 52 m deck includes the wing extensions), draft, radii of gyration, added-mass factor, damping
   ratios [E], derived periods documented [D].
3. **Landing on the moving deck** (core, `sim::LegLandingSim`): the leg contact runs in the deck's moving frame - the
   stage's pose, velocity and angular velocity are transformed into the deck frame (relative velocity includes the
   deck's heave rate and the rotation rates), the existing flat `LandingSurface::Deck` contact is computed there and
   the wrench transformed back. The touchdown judgement (gentle contact, tip-over tilt, at rest) uses the pose
   relative to the deck. A stage judged landed is held relative to the deck from then on (stand-in for the deck robot
   that secures the stage [call]) and rides its motion; its reported velocity includes the deck's.
   The sea beside the ship (splashdown flag) stays at the mean sea level [call].
4. **Guidance unchanged** [call]: the landing guidance aims at the deck centre at its mean height and cuts the engine
   on leg contact as now; the touchdown speed relative to the deck varies with the heave rate. If the flown sea-state
   case shows this is not enough (hard touchdowns within the refusal limits), the builder may add the smallest fix
   (e.g. terminal descent referenced to the measured deck height) and documents it.
5. **Onboard refusal** (core, `sim::plan_launch`): when the drone ship is chosen, the onboard computer computes the
   predicted deck motion statistics at its station from the sea state (spectral, no time simulation): significant deck
   tilt (2 sigma of the combined roll / pitch angle) and significant heave rate (2 sigma). Above the ship's limits
   (`max_deck_tilt_deg`, `max_heave_rate_mps` in `drone_ships.toml`, [E], chosen against the legs' gentle-contact
   envelope; the builder documents which Hs at Tp ~ 8-10 s they correspond to) the drone-ship recovery is refused with a
   German reason naming the value and the limit ("Seegang im Landegebiet zu hoch: ..."). RTLS stays preferred when
   feasible (unchanged); RTLS infeasible + drone ship refused -> the launch is refused (as other refusals).
6. **Configuration and fidelity flag**: a mission may carry a `[sea_state]` table (Hs, Tp, direction, seed); none ->
   calm. A session fidelity flag `deck_motion` (default on) [call]: off -> level deck and no sea-state refusal (the old
   behaviour) whatever the mission says. With no `[sea_state]` everything behaves exactly as today: no reference
   flight or validation number may move.
7. **A demonstration mission** `config/missions/falcon9_asds_sea.toml` [call]: the Starlink-class drone-ship flight
   with a moderate sea (around Hs 2-2.5 m, Tp ~ 8 s, within the limits) that lands on the moving deck.
8. **Game**: the snapshot reports the sea state and the deck's current heave / roll / pitch (and rates). The game draws
   a simple procedural drone ship (hull box with draft and freeboard, deck 91 x 52 m incl. wings, low walls along the
   long sides; matte grey; no logos) at the landing zone, posed by the core's deck motion, interpolated like the
   vehicles. The physics deck and the drawn deck must coincide. The sea shading's wave set follows the sea state
   (dominant wavelength from Tp, direction, slope from Hs; the calm set without a sea state); geometry stays on the
   geoid (shading only, as now). The HUD / time panel shows the sea state and the deck motion while a drone ship is the
   landing zone.

## Non-goals

- Wave geometry (displaced water mesh), foam, spray, wakes; phase coherence between the drawn wave pattern and the
  deck motion (the deck motion is the physics; the shading only matches its scale and direction).
- Short-crested (directionally spread) seas, wind, currents, swell + wind-sea mixtures (one spectrum).
- Surge / sway / yaw, station-keeping dynamics, the deck robot as a mechanism.
- Sea state for capsule splashdowns, the ship's sea landing or any other water contact.
- A UI to choose the sea state (comes with the mission selection panel: data-driven from the mission TOML).
- Weather forecasts or a time-varying sea state.

## Anti-goals

- Changed validation numbers or widened tolerances for flights without a `[sea_state]`.
- A drawn ship that does not match the physics deck (stage visibly floating above or sinking into it).
- Deck motion from a hand-tuned sinusoid with no physical derivation, or numbers without provenance tags.
- VR frame rate loss (the ship is a handful of boxes).

## Validation (each model with documented tolerances)

- Sea state: spectral variance m0 = Hs^2 / 16 (discretised, < 1 %); peak at Tp; dispersion; the time-sampled
  elevation's variance over a long record against m0 (statistical tolerance); same seed -> same sea.
- Deck motion: long-wave limit (heave RAO -> 1, pitch / roll -> the wave slope), short-wave limit (-> 0, the sinc
  zeros at kL = 2 pi), natural periods against the hydrostatic formulas, resonance amplification 1 / (2 zeta); the
  analytic plane fit against numeric quadrature.
- Contact in the moving frame: a stage resting on a deck that heaves sinusoidally stays at rest relative to it (no
  drift, no bounce); a stationary deck gives exactly today's forces (regression).
- Flown: `falcon9_asds_sea` lands (gentle touchdown, landed state, rides the deck afterwards) [slow]; a rough-sea
  request is refused by `plan_launch` before any trial; flag off -> today's flight.
- Game: smoke test reads the new snapshot fields; a render check of the ship at the deck.

## Left to the builder

Module placement of the deck motion, the component count and discretisation scheme, the deck-frame transform's
location (LegLandingSim or LandingSurface), how a held body follows a moving carrier (extend the landed hold or reuse
the docked-hold pattern), the [E] values of draft / damping / limits within plausible ranges, the exact snapshot keys,
and how the game's wave set maps Hs / Tp onto its four octaves.

## Amendments during implementation (2026-10-24)

Controller rulings that supersede the text above where they differ:

- Excitation: the heave / roll / pitch response uses strip-theory relative-motion excitation (Korvin-Kroukovsky with the
  Smith factor) instead of pure quasi-static excitation times the oscillator, so the forcing is consistent with the
  added-mass and damping basis; the long-wave limit is unchanged. The 1/(2 zeta) resonance check is replaced accordingly.
- Deck heading: with a non-calm sea the drone ship turns its deck to the heading of least predicted tilt (5 deg grid over
  0..180 deg, symmetric under a half turn at zero speed); in a calm sea it keeps the ground-track heading.
- Tip-over judgement test: 32 deg instead of 20 deg (20 deg lies inside the 30 deg tip-over angle and could not test one).
- The sea's time origin is the mission's requested epoch (floored to a whole TDB hour), passed by `start_mission`.
