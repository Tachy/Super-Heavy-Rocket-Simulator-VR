# Ascent trajectory and staging sequence - design

Date: 2026-10-22. Status: draft for the user's approval.

## Intent

A launch from the pad must fly one clean, continuous arc from liftoff through MECO and staging to SECO, computed from
physics for the chosen orbit target - the least propellant within the real constraints - not fitted to a webcast. A flight
to flight 14's target should then come out close to flight 14's webcast by itself; that is a check, not an input
(user 2026-10-22: "the webcast is no reference for the trajectory, it must be computed for the orbit target ... it must
follow physics").

### What the user saw (desktop, Starship and Falcon 9)

1. After separation the upper stage turns ~10 deg nose below the horizon while it keeps climbing.
2. The booster seems to overtake the upper stage instead of falling back.

### Measured causes (probe `tmp/staging_probe.gd`, headless flights, 2026-10-22)

1. The free launch (`TimePanel` "Auf die Rampe stellen" -> `FlightController._engage_ascent`) flies the code default
   `gnc::PitchProgram` (kick 10 deg, full thrust), not a profile fitted to the vehicle. MECO comes far too steep:
   Starship T+121 s at 2.15 km/s with an Earth-relative flight-path angle of 64 deg, Falcon 9 T+136 s at 79 km / 51 deg.
   PEG then points the upper stage 50-70 deg below its velocity (Starship -11 deg pitch, AoA 73 deg; Falcon 9 -5 deg,
   AoA 53 deg) to kill the excess vertical speed - optimal from that state, but the arc has a kink. With the tuned
   reference profiles (`starship_leo` kick 14 deg + throttle schedule, `falcon9_rtls` 12 deg) MECO is at 23 / 32 deg
   and the upper stage pitches smoothly (Starship up to 58 deg briefly, then down steadily).
2. `FlightSession::stage()` cuts the booster engines and separates in the same tick: the engines' shutdown tail-off
   (Raptor 0.5 s, Merlin 0.3 s) then acts on the light booster alone. Right after separation the booster is 17 m/s
   (Falcon 9) to 61 m/s (Starship) faster than the upper stage. In addition the Starship upper stage ignites only after
   separation (3 s start-up) while the booster's three centre engines idle through the flip.

## Goals

1. **Optimised first-stage profile at launch.** Before liftoff the onboard computer chooses the first-stage pitch
   program by optimisation: the least total propellant to SECO (equivalently: the most propellant left in the upper stage
   at insertion) for the target orbit, the payload and the booster recovery. The first stage keeps flying a zero
   angle-of-attack gravity turn after the kick (structural loads), with throttling around max-Q; the upper stage keeps
   PEG (already propellant-optimal). Applies to **every launch from a pad** - the free launch and every mission and
   mission chain (`starship_leo`, `falcon9_*`, ISS, depot, tanker, HLS, the flown campaign): the webcast-tuned
   `kick_deg` values leave the mission files (user 2026-10-22: "optimise everywhere, physics decides").
2. **Booster recovery is a constraint, decided by the computer.**
   - Super Heavy is always caught at the tower (standing decision). A target the stack cannot reach with a catch is
     refused before liftoff with a German message - never flown expendable.
   - Falcon 9: the onboard computer picks the recovery itself (user 2026-10-22): return to a landing zone (RTLS) when
     the launch site has one and the target and payload still close with the RTLS reserve, else a drone ship placed by
     the computer on the ground track at the first stage's predicted no-boostback impact point (the way
     `falcon9_asds.toml` derived its position). Neither feasible: refused with a message.
3. **Realistic staging sequence, data-driven** (new `[stage.staging]` section in the vehicle TOML, provenance tagged):
   - Falcon 9 (cold staging): MECO -> the stack coasts together through the engines' tail-off and a settling gap (~3 s
     [S: webcasts]) -> separation (pneumatic pushers, small push) -> upper-stage ignition ~7 s later [S: webcasts]
     ("SES-1"); the booster flips on its cold-gas thrusters, engines off, until the boostback.
   - Starship (hot staging): MECO (outer 30 engines off, the 3 centre engines stay lit) -> the ship's engines start while
     the stack is still joined -> separation once the ship is at thrust; the booster keeps its centre engines through the
     flip into the boostback (as today). Timing from flight 14 telemetry (`tests/validation/telemetry/flight14_*.csv`)
     where it shows; else [E] with the reason.
   - Momentum is conserved through the sequence: the tail-off acts on the joined stack; the push / the ship's thrust
     while joined acts on the stack, then the split.
4. **The timeline and webcast stay right**: MECO, stage separation and upper-stage ignition are recorded and predicted
   at their new, separate instants (the onboard-computer predictions and the webcast arc already show these events).
5. **Validation**: a free launch to flight 14's target (and to a Starlink / CRS-35 target for Falcon 9) is compared with
   the recorded telemetry (MECO time, altitude, speed, flight-path angle; SECO) with documented tolerances - as a check
   of the physics, not as tuning. The reference missions' tests are re-baselined on the optimised profile; the recorded
   telemetry stays the comparison. A clear deviation is fixed in the model, not in the profile: the ascent drag tables
   (`config/aero/*_ascent.csv`) are generic [E] trends never calibrated, and the reference missions' throttle level
   (0.944 x the gauge) was fitted to the recorded speed, which may have absorbed drag errors (user question 2026-10-22).
   Where the telemetry separates drag from thrust (coast phases, known engine counts), calibrate the drag; record the
   result with its provenance.

## Non-goals

- No change of the upper-stage guidance (PEG) itself.
- No expendable boosters, no landing choice in the menu (the computer decides).
- No new vehicles or Block versions (only Block 3 / V3 and Falcon 9 Block 5).
- No webcast-gauge matching (standing decision: physics over gauges).

## Anti-goals (would be a failure even if it "works")

- A trajectory that only looks like a webcast because numbers were fitted to it.
- A kink after staging repaired by PEG, or a booster that gains speed on the upper stage at separation.
- An optimisation that blocks the game: the free launch must still lift off on its countdown.

## Constraints

- Architecture (CLAUDE.md): physics in `core` (C++20, float64), bindings thin, the game only sends commands; vehicle
  and mission data in TOML with provenance tags; no magic numbers.
- The pre-launch optimisation runs on the simulation thread during the countdown (the free launch counts down 10 s;
  if the optimisation needs longer, the countdown holds - "Startvorbereitung" - rather than flying a worse profile).
- Fidelity: same force models as the flight (gravity, J2, atmosphere, drag, throttle, engine timing); simplifications
  only deliberate and documented.
- Tests: tag-filtered runs of the touched modules (`[ascent]`, `[staging]`, booster return, catch, RTLS / ASDS, timeline,
  and the ISS / refuelling chains that fly an ascent); `[slow]` campaigns via ctest labels in parallel.

## Decisions taken (user)

- Optimised at launch (not fixed reference profiles) - 2026-10-22.
- Realistic staging sequence as above - 2026-10-22.
- Super Heavy always returns; Falcon 9 chooses pad or drone ship by itself - 2026-10-22.
- The optimisation and the staging sequence apply to every flight, missions and chains included; reference numbers
  are re-baselined (catch, landing reserve, timeline, ISS launch window, refuelling r5 / f2 campaigns) - 2026-10-22.

## My calls (marked for the user's review)

- **The optimised quantity is the kick angle** (one parameter, as an open-loop first-stage program in practice), bounded
  to a plausible range; the kick timing (vertical rise to clear the tower) stays. A second free parameter only if the
  one-parameter optimum leaves a visible kink.
- **Throttle around max-Q**: a dynamic-pressure limit per vehicle (`[stage] max_dynamic_pressure_pa`, provenance
  tagged) the first stage throttles to, instead of a time schedule tied to one flight - in every launch, so the
  missions' `booster_throttle` schedules go too (consistent with "physics decides"). Rationale: a time schedule fitted
  to one trajectory is not physics for another target. The flights' recorded throttle shapes stay the check in the
  validation cases. A late-boost acceleration limit (the Starship schedule's drop before MECO) only if a vehicle has a
  sourced g limit.
- **Objective**: the most upper-stage propellant at SECO with the booster recovery as a hard constraint (Super Heavy:
  `meco_propellant_kg` decides MECO in every trial flight). Falcon 9: RTLS whenever feasible with a reserve, else the
  drone ship (SpaceX practice), even if the drone ship would leave more upper-stage margin.
- **Hot-staging physics**: the booster's three centre engines run at minimum throttle (Raptor 30 %: ~5 m/s^2 on ~520 t)
  through the separation and flip, the ship has ~9.5 m/s^2 at thrust - with the sequence right the booster falls back
  physically; a gain over the ship at separation is a defect the separation test asserts against.
- **Validation cases**: flight 14 flies its own recorded target (262 x 275 km, 30.5 deg, its payload and propellant
  load), engine-outs excluded and stated; Falcon 9 compares with CRS-35 (RTLS from SLC-40, ISS plane). Tolerances are
  set from the telemetry's reading accuracy; a miss is a documented discrepancy investigated in the model (drag, thrust,
  masses), never closed by tuning the profile.
- **Free-launch target**: the configured `leo_ascent.toml` orbit; where the site's latitude makes the configured
  inclination unreachable (Florida > 26.5 deg), the lowest reachable inclination (due east).
- **Falcon 9 payload in the free launch**: unchanged (whatever the free launch carries today).

## Left to the builder

- The optimiser (golden-section / bisection on the kick over full ascent simulations, or a cheaper surrogate) and its
  tolerances, as long as the result is within ~0.2 deg of the true optimum and the countdown rule holds.
- How the staging sequence is represented in `FlightSession` (a small state machine on the controlled stack) and how
  the timeline predictions follow it.
- Exact `[stage.staging]` keys and their sourced values.
- Re-baselining the affected reference tests: every changed reference number gets its reason in `docs/subsystems.md`.
