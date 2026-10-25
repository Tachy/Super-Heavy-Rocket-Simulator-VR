# Ship catch robustness - design

Date: 2026-10-21. Placed by the user before refuelling f2 (the flown campaign), so a random miss does not disturb its
comparison against the calculator.

## Problem

The ship (Block 3 / tanker / depot-class Starship) returns, flips at the gate (800 m) on three sea-level Raptors and
is caught by the tower after SCvx landing burns (`gnc/ship_landing`, `gnc/scvx_landing`). One of 21 dispersed
lunar-return direct gates misses (seed 12 before the f3 settling lead, seed 18 since; which seed moves with the gate
timing):

- The flip is attitude-only: it ends upright 25 m short of the catch axis at ~29 m/s across.
- Stopping in 25 m needs ~17 m/s^2; the first SCvx phase's 20 deg tilt gives ~3.6 m/s^2. The plans do not converge
  and the catch aborts at ~180 m.
- The abort then tumbles the ship (tilt 95-145 deg) into the sea instead of holding it upright.

Today the campaigns tolerate this: lunar return direct allows one miss per case of three, `[ship][catch]` and the
SCvx catch campaign allow two of 21.

## Goal (user 2026-10-21)

A reachable flip state is caught; a catch that is physically impossible (propellant, engine failure, energy-short
gate) ends in an **upright, controlled abort** - never a tumble. No campaign tolerates a miss any more.

Approach (user-approved): **A - the flip brakes across** (the alternatives, a predictive gate in the entry guidance
or a planner-only fix, were rejected: the first does not cover flip dispersions or engine failures and touches the
delicate terminal entry phase, the second leaves the hand-over in an unreachable state).

## Re-scope after the reproduction (user 2026-10-21)

Replaying seed 18 (the current miss) showed that the premise above holds for the old seed 12, not for seed 18: its flip
ends 148 m from the axis at 32 m/s towards it, 472 m up, and the first plan converges. The miss is a growing
attitude / lateral oscillation in the landing burn: the hull axis lags the plan's reference axis by ~1 s (actuator
delay 0.75 s), the reference jumps 3-6 deg at each re-plan, the re-plans stop converging around the 3 -> 2 engine
switch (~190 m), the ship stops sinking and the unconverged abort plans swing the hull to 45-54 deg. User decision:
sections 1-2 (flip brake, hand-over) are replaced by **section 1' (landing burn tracking)**; the flip brake law is
removed (no failing case needs it); sections 3-4 stay.

## 1'. Landing burn tracking (`gnc/scvx_landing`, `gnc/ship_landing`)

- Reproduce in seconds from the printed seed-18 gate state (`[.lunar_gate_replay]` path, made a regular test).
- Find the lever that carries the oscillation by systematic debugging (re-plan reference jumps, attitude lag /
  actuator delay vs the LQR, the 3 -> 2 switch, plans started from a state the plan's attitude model cannot follow);
  fix the cause, not the symptom; every changed value [E] with its measured reason.
- Criteria: the seed-18 gate is caught; the ship catch campaign and the lunar return direct seeds are not worse.

## 1. Braking flip (`gnc/ship_landing`) - superseded by 1' (kept for the record)

The flip is unchanged until the tilt is below `flip_flaps_until_tilt_rad` (30 deg): three Raptors at 80 %, forward
flaps broadside, roll reference for the pins across the arms. From there a new sub-phase **flip brake** commands the
thrust axis instead of the vertical:

- Horizontal reference velocity towards the catch axis (at the catch point, horizontal distance `d`):
  `v_ref = min(v_max, sqrt(2 * a_brake * d))` along the horizontal direction to the axis; `a_brake` [D] from the
  available thrust at the brake tilt limit (thrust of the flip group at the flip throttle, current mass).
- Commanded horizontal acceleration `a_h = k * (v_ref - v_h)`; thrust-axis tilt `atan(|a_h| / a_thrust_vertical)`
  limited by `flip_brake_tilt_max_rad` (35 deg [E]); the commanded axis slews no faster than the flip's rate limit.
- The flip push towards the tower (the gate lies `gate_lead_m` back on the tower -> pad line) stays wanted; the
  brake profile only stops it from overshooting the axis.
- The roll reference (pins across the arms) is unchanged.
- The entry guidance and the gate are unchanged.

## 2. Hand-over to SCvx - superseded by 1' (kept for the record)

- Hand-over when tilt < `handover_tilt_max_rad` (35 deg [E], replaces the 15 deg `flip_tolerance_rad` as the plan
  start condition) and the pitch/yaw rate < `flip_settled_rate_radps` (0.2 rad/s), **and** the first plan converges.
  The existing retry (`first_plan_retry_s` = 1 s) keeps trying; the flip brake flies in between.
- SCvx phase tilt limits: phase 1 (three engines) 35 deg [E] so that a plan starts admissibly at the hand-over state;
  the two-engine phases keep 20 / 15 deg.
- If no catch plan has converged by a floor (height / time-to-ground [E], set in the plan from the measured flip
  heights), the flight goes to the abort (section 3).
- `flip_duration_s` / `flip_end_height_m` keep their meaning (the flip proper ends at the old 15 deg tilt); the hand-
  over time and height are reported separately.
- Cross-checks: lunar-return seed 1 (the 15 deg saturation / overshoot / hover case that set the 20 deg limit in f3)
  and seed 18 are both caught; seeds caught today keep their propellant at catch within a documented band.

## 3. Upright abort

First plan step: reproduce the tumbling abort of seed 18 (`STARSIM_DEBUG_FLIP/BURN/SCVX`) and find whether the abort
plan does not converge, the engines are cut, or a plan with too much tilt is flown. The fix follows the finding;
these requirements hold regardless:

- Abort plans use the same tilt / rate limits as catch plans (35 / 20 / 15 deg per phase); attitude control stays
  active in the abort.
- Fallback law **upright descent** when no abort plan converges (deterministic, no optimiser): thrust axis within
  15 deg [E] of the vertical; lateral component away from the tower keep-out circle and towards the sea; vertical
  braking with the available engines to a sink rate <= 3 m/s [E] until contact or propellant out.
- An abort ends upright: tilt < 20 deg [E] at water / ground contact, no tumble. Whether it reaches the sea is
  physics; ground contact next to the tower is "upright abort, not caught".
- Return outcome classification from the session (not from test logic): **caught / upright abort / lost** (lost =
  tumbled or hard impact).

## 4. Tests and criteria

- Unit tests: the upright-descent law (stays upright, brakes, keeps clear of the tower).
- Fixed case (fast): the seed-18 gate state replayed (`place_landing_ship` from the printed GCRF state with the
  flight's aero / density / thrust factors) -> caught.
- Abort case: an unreachable gate (energy-short or an engine failed before the hand-over) -> upright abort.
- Campaign thresholds:
  - lunar return direct: 3 of 3 caught per case (was 2 of 3);
  - `[ship][catch][validation]` and the SCvx catch campaign: >= 20 of 21 caught, the rest upright abort, none lost
    (the belly-engine-failure case may abort).
- Re-measure the refuelling reference flights (tanker catch propellant, today 1.81 t at catch; inputs digest); the
  campaign answer (13 tankers / 49.7 d) is expected unchanged.
- `docs/subsystems.md` ship flip and catch: the tracking fix, the upright abort, the corrected seed-12 / seed-18 record; the "Open" note and the
  "one miss allowed" rationale go.

## Out of scope

- Entry guidance / gate changes, booster catch, Falcon 9 and Dragon landings (they do not use the ship flip).
- A real tank-pressure / engine start-up model for the flip.
