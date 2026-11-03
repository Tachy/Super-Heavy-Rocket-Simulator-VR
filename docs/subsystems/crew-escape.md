# Crew escape (ejection seats on the crewed ship return)

History and the reasons behind superseded decisions: `docs/history/crew-escape.md`.
Spec `docs/superpowers/archive/specs/2026-10-22-crew-escape-design.md`, plan `docs/superpowers/archive/plans/2026-10-22-crew-escape.md`.

## Model
- **Config:** `config/crew_escape/starship_crew.toml` (`config::load_crew_escape`): 4 seats in the ship's stage frame
  (x 43.0-46.6 m, z 3.5 m [E]), rails along body +z (the back, leeward in the belly flop) tilted 10 deg to the nose
  [E], ACES II-class numbers: rocket catapult 22.2 kN for 0.55 s [S: Whiteman AFB] split into `catapult` + `rocket`
  tables, envelope 600 KEAS [S: DTIC ADA133628] / 50,000 ft [E], mode 1 below 250 KEAS and 15,000 ft [S, secondary],
  drogue 0.16 s [S: LSI PAR-1610-00], separation 1.16 s [D: mortar 1.0 s after the drogue]; stagger 0.4 s [E];
  judging 20 g for > 0.1 s, 1 m clearance, plume cone 15 deg x 100 m [E], touchdown <= 8 m/s [S: AP3456].
  Bodies `vehicles/ejection_seat.toml` (170 kg, fixed drag 0.9 m^2, drogue 0.73 m^2) and `vehicles/parachutist.toml`
  (100 kg, main 35 m^2 [D: 6.7 m/s at sea level, S: GQ 5000 / AP3456], splashdown recovery 8 m/s). The ship keeps its
  mass and CG when the seats leave (documented simplification, ~0.8 t of ~260 t).
- **Catch feasibility** (`gnc::catch_feasibility`, crewed ships only): a failed flip-group engine, or usable landing
  propellant (the header tanks at the engines' mixture ratio, `gnc::landing_feed_propellant_kg`, less the unusable
  header propellant) below the ship's catch floor `gnc::ship_catch_propellant_floor_kg` [D, planner plan Task 4 fix
  round 2]: the ideal need from the nominal gate (the entry guidance's `gate_height_m` at `gate_reference_sink_mps`)
  to the catch (pins at the arm tops, the heavy landing mass's CG below them) - the ideal vertical stop at the first
  landing set's planned maximum, dv = (v_i - v_f) a / (a - g) with v_i^2 = ((v^2 + 2 g h)(a - g) + g v_f^2) / a, plus
  the flip's push across the path (`ship_flip_push`), rocket equation to the light landing mass at the landing
  engines' sea-level Isp, the gate mass as a fixed point - times (1 + `catch_propellant_margin` 0.1 [E]). Block 3
  (177 t): need 12.7 t over 679 m (stop 175 m/s, flip 49 m/s), floor 13.9 t usable; the nominal gate catches from 18 t
  (17 t usable). Computed once at the first entry tick (`CrewRuntime::catch_floor_kg`). Checked every tick of the entry
  up to the gate (the same tick before the gate hand-over); after the flip every abort (SCvx abort to the sea, upright descent, `abort_landing`) arms the seats, and
  so do a missed catch (phase Landed without pins on the arms) and every commanded engine starved or failed. An
  aerocapture pass is not judged; the final entry after the capture is.
- **Give-up in the entry** (`EntryGuidance::give_up`, `gnc::sea_give_up_target`): the target moves to the pad + sea
  direction x (shoreline + 2 km) [E], the gate height to 0, no gate hand-over, no flip: the entry / terminal steering
  fly the belly flop down to the impact.
- **Firing** (`FlightSession::run_crew_escape`, before the guidance each tick): inside the envelope (height above the
  flat site ground, EAS); an early give-up (above `min_firing_height_m` 1.5 km) waits until the ship is over water or
  at that height; a give-up at the gate or after the flip fires at once. Seats spawn at the ship's state + offset
  (velocity + omega x r), the rail thrust is an external wrench along the rail fixed at firing (pitch stabilisation and
  the few kg of seat propellant not modelled). `gnc::SeatSequencer` (mode latched at firing): mode 1 separates and
  opens the main at max(0.2 s, rocket burnout 0.55 s) - the rocket carries seat + occupant; modes 2/3 drogue, then
  separation + main (mode 3 waits for 15,000 ft). The separation swaps the seat body in place for a parachutist body
  (same index; the empty seat is not flown). Crew bodies get no `ParachuteSequencer` (`Flying::crew_member`).
- **Judging** (`sim::crew_escape.hpp`): load = |specific force| (incl. the rail thrust); hull clearance counted once the
  seat is 1 m out of the hull (riding the rails before); plume cone while engines burn; the tower box. Lost: sustained
  load, torn canopy, collision, touchdown > 8 m/s / impact, or the ship down before the seat fired
  ("außerhalb der Schleudersitz-Grenzen"). Caught with the crew aboard: rescued, no firing. A crewed ship that was
  given up returns `ReturnResult::Lost`.

## Code
- `config::load_crew_escape`; `gnc::catch_feasibility`, `gnc::sea_give_up_target`, `gnc::SeatSequencer`;
  `EntryGuidance::give_up`; `FlightSession::run_crew_escape`; `sim::crew_escape.hpp` (judging).
- **Missions:** `[mission] crew_escape = "<id>"` (`ship_return_starbase_crewed`, `ship_lunar_return_llo_crewed`);
  `start_lunar_return_mission` sets it; the orbit return (flown from the orbit) gets it from the game
  (`FlightSim.set_crew_escape(id)`) or the test. Any new placement (`clear_bodies`, `clear_for_new_body`) makes the
  flight uncrewed. Existing missions and campaigns stay uncrewed and unchanged.
- **Game:** `FlightSim.crew_escape_state()` (members with phase, mode, loads, clearance, position / attitude,
  touchdown, outcome; the ship's impact), `return_state(i)` of the ship (phase, guidance, abort, upright_descent,
  result), `set_crew_escape(id)`; each fired member is an ordinary body.

## Standing decisions
- User 2026-10-22: every abort of a crewed ship return ends with the crew on ejection seats (a water landing with crew
  is no option - the ship usually explodes); return only (ascent aborts later); the onboard computer triggers, no
  pilot button; every crew member is its own body; the empty ship is flown to its impact; "early, over water"; data +
  game interface first, the pilot's view rendered later.
- User 2026-10-22: the ship's data stay untouched (no change to `starship_block3.toml`: dry mass, CG, tanks, engines,
  aero); seats and crew live in their own config.
- Final review I3 (ruled): no per-axis load source in hand, so the magnitude check stays.

## Validation
- Unit case: give-up at 12 km / 120 m/s, 2 km west of the pad -> impact 2,500 m east (the target); given up at 800 m:
  impact 7 m beside the pad (reported as it falls).
- Entry engine failure 12 km, 2 km west -> fired at 61.7 s over water, 4 rescued 600-670 m east (water) at 6.7 m/s,
  peak 15.8 g, ship 2,500 m east in the sea at 96 m/s.
- Catch floor hand case (`test_catch_feasibility.cpp`): the stop's dv against a fall-then-brake flown in 0.1 ms steps
  (ignition by bisection, within 0.2 %); the floor below the nominal gate's lowest caught load.
- Give-up at 1.2 km (half the floor, 7.0 t usable): fires at once, mode 1, crew on land ~350 m from the pad, ship on land
  313 m from the tower.
- A catch that fails after the gate (1200 m at 100 m/s; usable midway between the floor 13.9 t and that gate's ideal
  need 14.3 t: passes the floor, cannot be caught): "Fang abgebrochen" at 5.7 s, 4 rescued (13.5 g), ship 86 m from the
  tower. The guidance's own abort at 9.1 t (950 m gate, ideal stop alone): "Fang abgebrochen" at 5.6 s, 4 rescued.
- Catch abort in the flip: crew on land ~160 m from the pad (14-15 g), ship upright in the water 20 m beyond the shore.
- Crewed orbit return and lunar direct return with an entry engine failure: the ship is over the Gulf when it enters
  the envelope - seats fire at ~15 km in mode 3, 4 rescued on water at 6.7 m/s, peak 16.4 g.
- Tests `[crew_escape]`: `test_crew_escape_config.cpp`, `test_ejection_seat.cpp`, `test_catch_feasibility.cpp`,
  `test_entry_give_up.cpp`, `test_crew_escape_session.cpp`, `test_crew_escape_mission.cpp` (`[slow]`: the entry engine
  failure, the flip abort, the crewed orbit and lunar returns).

## Open points and known limits
- The load check is the magnitude of the specific force against 20 g for > 0.1 s [E], not per axis against sourced
  spinal / lateral limits; there is no opening-shock reference case.
- No Mach limit in the envelope (height + EAS only); no wind (M8); no zero-zero apex reference (no published ACES II
  number found); the seat's attitude is free (no STAPAC); no seat-seat collision check.
- A crew landing on land beside the pad after a late give-up is reported, not avoided.
- The "Fang verfehlt" arm (phase Landed, CG past the catch point, no abort flag) has no session case: no load reaches
  it (scans 2026-11-03, 177 t ship: gates 950 m at 90 / 100 / 115 m/s and 1200 m, 15-18 t, and entering from 12 km gave
  "Fang abgebrochen" or ran dry hovering at the arms - 17-18 t at 115 m/s: "Triebwerke ohne Treibstoff", then dropped
  onto the arms and caught with the crew already out). It needs a truth-vs-plan shortfall scenario.
- The floor's ideal need leaves out the real profile's losses (nominal gate: lowest caught 18 t = 1.3 x the ideal); the
  margin only moves the give-up early, it never gives up a catchable ship.
