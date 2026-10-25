# Crew escape (ejection seats on the crewed ship return) - history

Verbatim record of this subsystem's notes as they stood in `docs/subsystems.md` on 2026-10-25, before the split. Current state: `docs/subsystems/crew-escape.md`. Kept for the reasons behind decisions, superseded numbers and earlier approaches; read only when that history is needed.

Spec `docs/superpowers/archive/specs/2026-10-22-crew-escape-design.md`, plan `docs/superpowers/archive/plans/2026-10-22-crew-escape.md`.
User decisions (2026-10-22): every abort of a crewed ship return ends with the crew on ejection seats (a water landing
with crew is no option - the ship usually explodes); return only (ascent aborts later); the onboard computer
triggers, no pilot button; every crew member is its own body; the empty ship is flown to its impact; "early, over
water"; data + game interface first, the pilot's view rendered later; **the ship's data stay untouched** (no change to
`starship_block3.toml`: dry mass, CG, tanks, engines, aero) - seats and crew live in their own config.

- **Config:** `config/crew_escape/starship_crew.toml` (`config::load_crew_escape`): 4 seats in the ship's stage frame
  (x 43.0-46.6 m, z 3.5 m [E]), rails along body +z (the back, leeward in the belly flop) tilted 10 deg to the nose
  [E], ACES II-class numbers with sources: rocket catapult 22.2 kN for 0.55 s [S: Whiteman AFB] split into
  `catapult` + `rocket` tables, envelope 600 KEAS [S: DTIC ADA133628] / 50,000 ft [E], mode 1 below 250 KEAS and
  15,000 ft [S, secondary], drogue 0.16 s [S: LSI PAR-1610-00], separation 1.16 s [D: mortar 1.0 s after the drogue];
  stagger 0.4 s [E]; judging 20 g for > 0.1 s, 1 m clearance, plume cone 15 deg x 100 m [E], touchdown <= 8 m/s [S:
  AP3456]. Bodies `vehicles/ejection_seat.toml` (170 kg, fixed drag 0.9 m^2, drogue 0.73 m^2) and
  `vehicles/parachutist.toml` (100 kg, main 35 m^2 [D: 6.7 m/s at sea level, S: GQ 5000 / AP3456], splashdown
  recovery 8 m/s). The ship keeps its mass and CG when the seats leave (documented simplification, ~0.8 t of ~260 t).
- **Catch feasibility** (`gnc::catch_feasibility`, crewed ships only): a failed flip-group engine or propellant below
  `min_catch_propellant_kg` = 30 t [D: nominal gate 28 t caught, 27 t missed (hidden test "Ship catch: lowest
  catchable gate load") + 2 t]. Checked every tick of the entry up to the gate (the same tick before the gate
  hand-over); after the flip every abort (SCvx abort to the sea, upright descent, `abort_landing`) arms the seats,
  and so do a missed catch (phase Landed without pins on the arms) and every commanded engine starved or failed (final
  review I1: at 27 t the ship ran dry hovering at the arms at ~93 m and dropped onto them - 3 seats out, the fourth
  0.4 s stagger slot too late: "Schiff aufgeschlagen vor dem Ausstieg"). An aerocapture pass is not judged; the final
  entry after the capture is (final review I2).
- **Give-up in the entry** (`EntryGuidance::give_up`, `gnc::sea_give_up_target`): the target moves to the pad + sea
  direction x (shoreline + 2 km) [E], the gate height to 0, no gate hand-over, no flip: the entry / terminal steering fly
  the belly flop down to the impact. Unit case: 12 km / 120 m/s, 2 km west of the pad -> impact 2,500 m east (the
  target); given up at 800 m: impact 7 m beside the pad (reported as it falls).
- **Firing** (`FlightSession::run_crew_escape`, before the guidance each tick): inside the envelope (height above the
  flat site ground, EAS); an early give-up (above `min_firing_height_m` 1.5 km) waits until the ship is over water or
  at that height; a give-up at the gate or after the flip fires at once. Seats spawn at the ship's state + offset
  (velocity + omega x r), the rail thrust is an external wrench along the rail fixed at firing (pitch stabilisation and
  the few kg of seat propellant not modelled). `gnc::SeatSequencer` (mode latched at firing): mode 1 separates and
  opens the main at max(0.2 s, rocket burnout 0.55 s) - the rocket carries seat + occupant; modes 2/3 drogue, then
  separation + main (mode 3 waits for 15,000 ft). The separation swaps the seat body in place for a parachutist body
  (same index; the empty seat is not flown). Crew bodies get no `ParachuteSequencer` (`Flying::crew_member`).
- **Judging** (`sim::crew_escape.hpp`): load = |specific force| (incl. the rail thrust); hull clearance counted once
  the seat is 1 m out of the hull (riding the rails before); plume cone while engines burn; the tower box. Lost:
  sustained load, torn canopy, collision, touchdown > 8 m/s / impact, or the ship down before the seat fired
  ("außerhalb der Schleudersitz-Grenzen"). Caught with the crew aboard: rescued, no firing. A crewed ship that was given
  up returns `ReturnResult::Lost`.
- **Numbers:** entry engine failure 12 km, 2 km west -> fired at 61.7 s over water, 4 rescued 600-670 m east (water)
  at 6.7 m/s, peak 15.8 g, ship 2,500 m east in the sea at 96 m/s. Give-up at 1.2 km (20 t): fires at once, mode 1,
  crew on land ~340 m from the pad, ship on land 469 m from the tower. Catch abort in the flip: crew on land ~160 m
  from the pad (14-15 g), ship upright in the water 20 m beyond the shore. Crewed orbit return and lunar direct return
  with an entry engine failure: the ship is over the Gulf when it enters the envelope - seats fire at ~15 km in mode 3,
  4 rescued on water at 6.7 m/s, peak 16.4 g.
- **Missions:** `[mission] crew_escape = "<id>"` (`ship_return_starbase_crewed`, `ship_lunar_return_llo_crewed`);
  `start_lunar_return_mission` sets it; the orbit return (flown from the orbit) gets it from the game
  (`FlightSim.set_crew_escape(id)`) or the test. Any new placement (`clear_bodies`, `clear_for_new_body`) makes the
  flight uncrewed. Existing missions and campaigns stay uncrewed and unchanged.
- **Game:** `FlightSim.crew_escape_state()` (members with phase, mode, loads, clearance, position / attitude, touchdown,
  outcome; the ship's impact), `return_state(i)` of the ship (phase, guidance, abort, upright_descent, result),
  `set_crew_escape(id)`; each fired member is an ordinary body.
- **Known limits:** the load check is the magnitude of the specific force against 20 g for > 0.1 s [E], not per axis
  against sourced spinal / lateral limits, and there is no opening-shock reference case (final review I3, ruled: no
  per-axis source in hand); no Mach limit in the envelope (height + EAS only); no wind (M8), no zero-zero apex reference (no published ACES II number found), the seat's
  attitude is free (no STAPAC), no seat-seat collision check, crew landing on land beside the pad after a late give-up
  is reported, not avoided.
- **Tests:** `[crew_escape]` (`test_crew_escape_config.cpp`, `test_ejection_seat.cpp`, `test_catch_feasibility.cpp`,
  `test_entry_give_up.cpp`, `test_crew_escape_session.cpp`, `test_crew_escape_mission.cpp`; `[slow]`: the entry
  engine failure, the flip abort, the crewed orbit and lunar returns).

