# Ship return: entry, belly flop, flip and tower catch - design

Status: draft 2026-10-13. Branch `feat/scvx-flight`, milestone (b) of the SCvx flight plan's follow-ups.

## Intent

The user (2026-10-13): the ship returns "mit Fang an Mechazilla" - entry drag, belly flop, bellyflip, landing burn,
caught by the tower. The autopilots fly every mission step; the player may steer by hand and the onboard computer
re-plans the setpoint from the actual state (memory `project-autopilot-control-levels`). The first target is
**Starbase**: the ship returns from orbit and is caught there, as SpaceX plans it (user decision 2026-10-13).
Flights 12-14 (Block 3, suborbital to the Indian Ocean) validate the physics only.

Success: a ship in low Earth orbit, given "return to Starbase", burns out of orbit, flies the entry, belly flop and
flip, and is caught by the tower within the same tolerances as the booster; its entry from 90 km to the
subsonic belly flop reproduces the webcast telemetry of flights 12-14 within the stated tolerances.

## Decomposition (user-approved 2026-10-13)

Each part has its own implementation plan and ends with a testable deliverable.

| Part | Content | Validation |
|---|---|---|
| b1 | Entry aerodynamics (belly-first hull, four flaps), flap actuators, attitude control on flaps + RCS, entry heating and load limits (skip-out / too hot emerge) | Entries of flights 12-14 replayed; corridor ends |
| b2 | Entry guidance to a target site (bank modulation, predictor-corrector), subsonic belly flop | Reaches the flip gate above the tower from dispersed entry states |
| b3 | Flip (engine relight, rotation to vertical), landing burn by SCvx with the ship's model, tower catch on the ship's pins | Final phase of flights 13/14; catch campaign |
| b4 | Mission "return to Starbase": deorbit burn from orbit, whole chain | Full return from orbit caught |

This document details b1 and fixes the interfaces b2-b4 build on; b2-b4 are refined when their plans are written.

## Evidence (webcast telemetry, `tests/validation/telemetry`)

Ship speed (Earth-relative) and altitude (0.1 km) from 92 km to splashdown (flight 13 complete; flight 12 to
57 km; flight 14 98 km to 0.8 km). Along the path D/m = -dv/dt - g sin(gamma); across it (vertical plane) the
lift component follows from the curvature of the altitude, with the Coriolis term of eastward flight:

| Phase | Mach | beta = m/(C_D A) [kg/m^2] | vertical L/D |
|---|---|---|---|
| Upper entry, 88-72 km | 27-25 | 450-690 | 0.0-0.3 (rising) |
| Equilibrium glide, 70-55 km | 24-15 | 500-770 | 0.45-0.65 |
| Mach 10-3 | 10-3 | 540-630 | 0.50-0.58; drag ~13-14 m/s^2 almost constant |
| Belly flop, 15-2 km | 0.7-0.3 | 500-560 | (terminal, ~110-165 m/s) |

Readings: C_D A ~ 450-550 m^2 at the entry mass (~270-285 t: dry 255 t [D] + landing propellant), i.e. a
belly-first body (planform 52 x 9 m = 470 m^2 plus flaps); the near-constant drag between Mach 10 and 3 points to a
drag-profile guidance; a vertical L/D below the body's L/D means bank. The current model has none of this: after
separation the ship flies the stack's ascent table with a generic cross-flow (eta 0.68, C_dc 1.2) and no flaps;
`config/aero/ship_entry.csv` is loaded but unused (a placeholder ~4x too low) - it is removed.

## b1 design

### Approaches considered

1. **Calibrated cross-flow body + physical flaps (chosen).** The hull stays a `SlenderBodyAero` (axial table +
   nose potential term + cross-flow), with a per-vehicle cross-flow coefficient table over Mach identified from
   the telemetry; the flaps are separate surfaces with their own forces at their own positions (drag, trim and
   control all emerge from them). Fits the existing grid-fin pattern; few parameters, each identifiable.
2. Offline Newtonian panel tables (C_N, C_A, C_m over Mach and alpha for each flap setting). More geometric
   detail, but a new offline tool, large tables, and the flap coupling hidden in the tables - control allocation
   would need table derivatives.
3. Fit a point-mass L/D and beta schedule (no attitude). Cheapest, but no flaps, no trim, nothing for the player
   to fly - rejected (physical correctness first).

### Hull

`SlenderBodyAero` gains an optional cross-flow table (`[stage.aero] crossflow_table`, CSV `mach, crossflow_cd`,
the product eta * C_dc of the Jorgensen cross-flow term). Without the table the generic constants stay (booster,
stack, Falcon 9). The ship's table is [D]: identified from flights 12-14 together with the flaps (below);
expected ~1.2-1.4 hypersonic (modified Newtonian cylinder 1.23), lower subsonic. Free-molecular bridging above
~90 km is unchanged. The body's L/D at angle of attack alpha is ~cot(alpha) (axial force small): alpha ~55-60 deg
gives the measured 0.55-0.65.

### Flaps (`core/aero/flap`)

Four flaps (two forward near the nose, two aft), each a flat plate hinged along a line parallel to the body axis
on the hull side. Geometry per flap: hinge azimuth, fore/aft stations, span, area. Deflection delta from 0
(stowed: folded against the hull towards the leeward side) to delta_max ~ 90 deg (extended: in the belly plane,
broadside to a belly-first flow). Plane: hinge direction e_x and s(delta) = cos(delta) u_stow + sin(delta) e_r; normal
n = e_x x s.

Force (windward face only; a flap with the flow on its back side or in the hull's lee carries none):
|C_N| = cn_max(M) sin^2(theta), theta = incidence of the local flow (vehicle air velocity + omega x r at the flap
centroid) on the plate, acting at the centroid, reference = flap area. cn_max(M) [E] table: 1.84 hypersonic
(modified Newtonian stagnation coefficient), ~1.2-1.3 subsonic (flat plate broadside). Rarefied flow scales it
like the hull. This Newtonian form underestimates small-incidence subsonic lift; flaps fly at large incidence in
the belly flop - a documented simplification.

Actuators: deflection limits and rate limit (`Vehicle::update_flaps`, like the grid fins), [E] ~ 20 deg/s.

Config (`[[stage.flap]]`, ship only): `azimuth_deg`, `station_fwd_m`, `station_aft_m`, `span_m`, `area_m2`,
`max_deflection_deg`, `rate_deg_s`; `[stage.aero] flap_table` (mach, cn_max) and `belly_azimuth_deg` (body
direction of the heat shield). All [E] from public dimensions (Block 3 forward flaps moved leeward/smaller, aft
flaps ~ 2 x 30 m^2), stated in the TOML with sources where available.

### Mass and centre of gravity

Entry mass = dry 255 t [D] + the ship's landing propellant (`[stage.recovery] landing_propellant_kg`, [E] ~25 t,
replaces the unused `reserve_propellant_kg` for the ship). The dry CG station (now 20 m [E]) is checked for
trimmability: at alpha 55-70 deg the flaps must trim the pitch moment within their range with margin for control
(test). If no CG in a physically plausible band allows that, the flap geometry estimate is revisited, not the CG
alone.

### Attitude control

- `gnc/entry_attitude`: from the air-relative velocity, a commanded angle of attack alpha and bank sigma it builds
  the body axis direction and the roll reference (belly towards the flow, lift vector rotated by sigma about the
  air velocity). Used by the autopilot and as the flight director's setpoint for manual flight.
- `AttitudeController`: a flag `flaps` (like `grid_fins`). Allocation: the flaps hold a nominal extension (pitch
  trim found by the controller's integral of the commanded torque - or simply: minimum-norm deviations around a
  nominal setting, clamped), the remaining torque goes to RCS. Effectiveness: numeric derivative of the flap
  torques per deflection at the last tick's air data (`FlightBody::air_data()`), as for the grid fins.
- RCS: the ship's `[stage.rcs]` (cold-gas, [E]) acts above ~ 2 kPa dynamic pressure only as a supplement; in thin
  air (above ~ 80 km) it holds the attitude alone.

### Entry corridor: skip-out, heating, load (user request 2026-10-13)

The user: "zu flach: Stein auf dem Wasser - und zu steil: zu heiss", and a skip is sometimes wanted for the first
deceleration. Both ends of the corridor must emerge from the physics, not from scripted outcomes:

- **Skip-out** needs nothing special: lift (set by angle of attack and bank) and a shallow entry flight-path angle
  carry the ship back above the atmosphere in the integrated trajectory. It is reported as an event ("Abprall") with
  the new apoapsis; a skip into an orbit that re-enters later is a valid trajectory.
- **Heating** (`environment`-level helper, `aero/entry_heating`): stagnation-point convective heat rate by
  Sutton-Graves, q = k sqrt(rho / R_n) v^3 with k = 1.7415e-4 (Earth air, SI) and an effective nose radius of the
  belly-first ship [E]; integrated heat load. Radiative heating is neglected below ~ 9 km/s (LEO entries).
  Limits per vehicle (`[stage.heat_shield]`: max heat rate, max heat load [E]) give a warning at 80 % and
  "Hitzeschild versagt" beyond the limit (the vehicle breaks up, like an impact). Telemetry: heat rate and load.
- **Load**: sensed acceleration (aero / mass) against a structural limit [E] ("Überlast" beyond it).
- Angle of attack and bank are the player's levers: a higher alpha raises drag and lowers L/D (steeper, hotter
  peak, shorter); lower alpha flies further and longer (more heat load); bank up spends lift and steepens the path.

Tests: a too shallow entry skips out (event, apoapsis above 100 km); a too steep one exceeds the heat-rate or load
limit; a nominal flight-13 replay stays inside both (peak heat rate reported for comparison with published values).

### Validation (b1 deliverable)

- `tools/telemetry/entry_estimate.py` (committed): beta and vertical L/D of the ship over time and Mach from a
  telemetry table (the analysis above).
- Unit tests: flap force (windward/leeward, theta, rarefaction), flap geometry, cross-flow table interpolation,
  `entry_attitude` geometry, allocation (a commanded pitch/roll/yaw torque is produced by the flaps within limits).
- Trim test: the ship trims at alpha 55, 60, 70 deg at Mach 20 and in the belly flop (Mach 0.4) with all flaps
  inside their range and >= 15 deg of control margin.
- Replay test (tolerance stated in the test, [D] calibration): from the flight-13 state at ~88 km (speed, path
  angle from the telemetry; position and heading on the flight's ground track to the Indian Ocean [E]) the ship,
  flown by `entry_attitude` with an alpha schedule over Mach and a bank chosen each second so that its vertical
  lift matches the telemetry's vertical L/D, reproduces speed over altitude of flights 12, 13 and 14 within
  5 % rms from 85 km to Mach 1, and the subsonic terminal speed within 10 %. The alpha schedule and the
  cross-flow table are identified on flight 13 and checked on 12 and 14.

## b2 design (entry guidance, refined 2026-10-13; decisions taken autonomously, ledgered)

### Approaches considered

1. **Numeric predictor-corrector on the bank magnitude (chosen, as outlined).** The onboard computer integrates a
   3-DOF point mass from the measured state to the gate with the vehicle's own aero coefficients, and solves for
   the bank magnitude that zeroes the range error; bank reversals by a heading-error corridor. Closed loop from the
   actual state (control levels: a human-made state is just another initial state).
2. Reference drag profile tracking (Shuttle / Apollo style). Needs an offline reference per entry and gains;
   brittle for arbitrary states the player produces - rejected.
3. SCvx entry trajectory optimisation. Heavy, and the bank-only control with slow reversals gains little from
   optimality - rejected.

### Units

- **`gnc/entry_aero_table`** (built on the simulation thread from the vehicle): C_D A and C_L A [m^2] over Mach at
  the nominal angle of attack of the alpha schedule, and over Mach and angle of attack 40-140 deg (terminal
  steering), hull plus the four flaps in pitch trim (forward and aft pairs moved differentially about mid-range
  until the pitch moment about the current CG is zero). Continuum coefficients (above ~90 km forces are tiny; the
  online scale factors absorb the difference). The alpha schedule moves into the vehicle config:
  `[stage.aero] entry_alpha_table = "aero/ship_entry_alpha.csv"`.
- **`gnc/entry_predictor`** (pure, no SPICE - can run on a worker): 3-DOF in the Earth-fixed frame (central
  gravity + J2, Coriolis and centrifugal terms, US76 air, constant mass), RK4 with 1 s steps. Lift is rotated
  about the air velocity by the bank angle, the bank moving from its current value to the commanded one at the
  bank rate limit (over Mach, as the guidance); optionally one planned reversal (the command changes sign at a
  given time) or, for the terminal phase, a fixed steering vector (lift direction x fraction, alpha from it).
  Drag and lift are multiplied by online scale factors. Ends at the gate height above the target (or skip-out
  above 130 km, 3000 s). Miss: farther than 50 km from the target the range difference from the start and the
  target's distance from the great circle start -> end (monotone in the bank); closer, the horizontal miss at
  the end along / across the track there (a range difference cannot tell a target behind the end from one ahead).
  Peaks of heat rate (Sutton-Graves, b1), heat load, aerodynamic load and dynamic pressure.
- **`gnc/entry_guidance`** (`EntryGuidance`, like `BoosterReturnGuidance`):
  - *Entry* phase, every 2 s: (a) scale factors drag/lift = measured / modelled aerodynamic acceleration
    (accelerometer: `FlightBody::aero_acceleration_body_mps2()`, filtered, clamped 0.5-2, only above 1 m/s^2);
    (b) corrector: bracketed secant (Illinois) on |sigma| in [0, 150 deg] for zero downrange error, warm-started;
    full lift up if even that is short, 150 deg if even that is long; (c) corridor constraints: if the
    prediction exceeds 90 % of a limit (heat rate, load, dynamic pressure [E] 25 kPa), |sigma| is reduced by
    bisection to the largest value inside them (range is given up, not the vehicle); (d) bank reversals timed by
    the predictor: with the sign held the end lies on one side of the target, reversing now on the other - the
    reversal is flown in the cycle in which a reversal one cycle later would already end on the "hold" side; the
    bank magnitude accounts for the reversal still to come (its time interpolated). A heading-error corridor does
    not work here: the long glide on one sign curves the track by 15-20 deg on a trajectory that ends on target.
    No reversals below 600 m/s, at least 20 s apart [E]; (e) output: alpha from the schedule at the current Mach,
    bank rate-limited: 1 deg/s above Mach 10 (faster departs in alpha, b1 - seen again with 8 % weaker flaps),
    3 deg/s below Mach 5 (late reversals must be quick; 1 deg/s left 5-15 km at the hand-over) [E].
  - *Terminal* below Mach 2 [E] (supersonic to the subsonic belly flop): like a skydiver - the belly faces the air
    flow (no sideslip), the lift is pointed: its direction about the air velocity and its size by the angle of
    attack (90 deg none, 55 deg most [E], the trimmed range). One Newton step per 0.5 s on the predicted
    horizontal miss at the gate (three predictions with the steering vector held). Starts from the lift vector
    flown at the hand-over (no jump); commanded attitude slew-limited to 3 deg/s. Rejected on the way (6-DOF
    evidence): tilting a level hull (a rolled cylinder makes no side force; nose-up beyond ~100 deg alpha tumbles),
    yawing the axis onto the gate while gliding (large sideslip: the ship spun up), a constant-velocity ZEM (the
    belly's drag brakes the hand-over speed within seconds).
  - *Gate*: reached at `gate_height_m` [E] 1500 m above the target's ground (b3 starts the flip there).
  - Command: phase, alpha / bank setpoint, body axis + roll reference for `AttitudeController` (flaps, 0.5 rad/s),
    predicted downrange / crossrange miss, constraint flag.
- **Ship axial drag** (found in b2): the ship alone had no axial drag table (a separated ship inherited the stack's
  in the session, a ship built alone in tests had none - the level hull drifted along its axis unbraked). The
  ship now carries the stack's ascent table [E] (same nose and diameter); the b1 replay stays inside its
  tolerances (flight 13 1.8 %, 12 2.5 %, 14 4.3 % above Mach 3).
- **Session**: `FlightSession::place_entering_ship(config, state, epoch, target)` (scenarios; b4 adds the deorbit
  burn), the ship then flies its entry guidance; telemetry carries the setpoint (alpha, bank, phase, predicted
  miss) for the HUD's flight director lines. Manual entry flight (the player banks, the computer re-plans) comes
  with the cockpit (M7); the setpoint shown now is what it will fly against.

### Validation (b2 deliverable)

- Predictor vs the 6-DOF ship (b1 physics) at constant bank: end point of an entry within 2 % of the range flown.
- Corrector unit tests on the predictor alone (long / short targets, saturation).
- Terminal / belly flop: from 12 km, falling at terminal speed with the gate 2 km ahead and 0.5 km to the side, the
  ship reaches the gate within 50 m; a guided 6-DOF entry from the interface reaches it within 100 m.
- Campaign `[ship][guidance][validation]`: from entry interface states uprange of Starbase with dispersions (entry
  flight-path angle +-0.2 deg, speed +-40 m/s, target range +-300 km, crossrange +-60 km, entry mass +-5 %, hull
  cross-flow +-10 % and flap table +-10 % in the truth model only) every case reaches the gate within 100 m
  horizontally, belly down within 15 deg, sink rate below 200 m/s, without breakup.
- Constraint test: a steep entry (-2.5 deg) flown again with the load limit at 85 % of its guided peak: the
  guidance holds the limit and the ship does not break up.

Out of b2: the guided skip (optional mode, last priority - deferred), wind, manual entry flight (M7).

## b2-b4 outline (refined in their plans)

- **b2 entry guidance**: numeric predictor-corrector (the onboard computer integrates the remaining entry with the
  vehicle's aero model from the actual state, every few seconds on a worker, like the descent re-planner) solving
  for the bank magnitude that hits the target; bank reversals keep the crossrange inside a corridor; constraints:
  load factor, heat rate and heat load (b1 heating model) and dynamic pressure. A guided skip (first
  deceleration pass above the atmosphere, then the final entry) is an optional guidance mode for long-range or
  high-energy returns. Ends with the transition to the subsonic
  belly flop (alpha ~ 70-90 deg) and a belly-flop glide steered by the flaps to the flip gate above the tower.
  Manual flight: the flight director shows the alpha/bank setpoint.
- **b3 flip and catch** (user 2026-10-13: engine sequence 3 -> 2 -> 1, as seen in the videos; SCvx phases like the
  booster's 13 -> 5 -> 3): relight the three sea-level Raptors at the flip gate (flights 13/14: engines lit at
  ~0.5-1.5 km, ~100 m/s), rotate to vertical (gimbal + flaps + RCS), then the SCvx landing planner with a model
  built from the ship (`make_descent_model` generalised; one engine phase), the tower's catch point for the ship
  (`[stage.catch_pins]` on the ship), the existing tower contact and judgement. Ditching fallback as for the
  booster.
- **b4 mission**: `config/missions/ship_return_starbase.toml`; deorbit burn planned by the onboard computer from the
  orbit state for the entry interface state b2 needs (burn_node), then b2 and b3. Campaign over orbit phase and
  dispersions.

## Out of scope

Thermal model inside the heat shield (tiles, temperatures - only stagnation heat rate and load are modelled), plasma blackout, flap damage, wind,
Falcon 9 (milestone c).
