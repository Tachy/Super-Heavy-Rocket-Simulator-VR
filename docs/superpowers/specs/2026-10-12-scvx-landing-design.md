# 6-DOF successive-convexification landing guidance - design

Status: approved in conversation 2026-10-12, sub-project 3 of 3 of the tower-catch landing guidance
(1 SOCP solver [done] -> 2 catch tower and arms [done] -> 3 this).

## Purpose

Catch the returning booster on the tower arms reliably. The arms leave 0.3 m of clearance per side; the current
explicit landing-burn guidance misses by 8.7-12.2 m because, on the three centre engines, the attitude follows
commands with seconds of lag and the explicit laws do not plan for it. A 6-DOF successive-convexification (SCvx)
planner (Szmuk & Acikmese; Malyuk et al.) plans attitude, rates and thrust together under the actual limits and
is re-solved in flight.

User decisions:
- **Takeover at landing-burn ignition** through the 13 -> 3 engine switch to the catch. Flip, boostback and the
  grid-fin-steered descent stay as they are (they deliver the booster to ignition with ~4 m impact miss); the
  ignition time still comes from the existing 1-D burn simulation.
- **Robustness target:** fixed dispersion cases plus an engine failure; >= 90 % caught, every catch within
  < 1 m offset and < 0.5 m/s lateral speed, no tower impact in any case.
- **Abort to the sea** when the catch can no longer succeed (no water model yet: a configured shoreline).

## Prerequisite: reusable SOCP solver (`optim`)

From the review of sub-project 1, needed now because SCvx solves 5-10 SOCPs per re-plan of identical structure:
- `optim::SocpSolver`: built once from the problem structure (sparsity patterns of A and G, cone layout);
  `solve(c, A values, b, G values, h)` reuses the equilibration workspace, the KKT pattern, the minimum-degree
  ordering, the symbolic LDL^T and all work arrays. `solve_socp()` stays as a convenience wrapper.
- `KktSystem` gets a values-only update path.
- Iterative refinement stops when a step no longer reduces the residual by a factor (Clarabel's stop ratio 5).
- `SocpProblem::validate` rejects NaN/Inf; a non-finite step ends with `NumericalError` at once;
  `VariableBlock::operator[]` is bounds-checked.
- `SocpResult` reports regularised pivots and refinement failures (diagnostics).

## Guidance model (`gnc/powered_descent`)

Frame: the launch site frame of the tower (ENU, origin at the pad centre on the ground). The rotating Earth enters
as known Coriolis and centrifugal accelerations evaluated on the previous iterate.

- **State** per node (14): mass, position, velocity, attitude quaternion (body -> site), body angular rate.
- **Control**: thrust vector in body axes, first-order hold between nodes.
- **Two phases**: phase 1 on the ignition engine set (13), phase 2 on the final set (3); each phase has its own
  duration (time dilation) as a decision variable, so the switch time and the total time are optimised.
  Nodes: 15 per phase (configurable).
- **Dynamics** (planning model, documented simplification): rigid body with the thrust at the gimbal point,
  mass flow proportional to thrust (engine Isp of the set), gravity, tail-first axial drag from the vehicle's
  descent table along the air velocity. Grid fins are not in the plan (their moment is reserve for the tracking
  controller); no wind (M8). Jacobians numerically (central differences on the planning model); discretisation by
  first-order hold with RK4 integration of the variational equations.
- **Constraints** (second-order cones, nonconvex parts linearised about the previous iterate):
  - thrust magnitude between the set's minimum and maximum (lower bound linearised),
  - gimbal cone about the body axis (engine gimbal limit),
  - tilt cone of the body axis from the vertical (limit tapering towards the catch as today),
  - angular rate bound,
  - glide-slope cone towards the catch point,
  - minimum mass (reserve).
- **Terminal condition** (catch): pin axis at the catch point 1 m above the arm tops, sink rate 1 m/s, lateral
  velocity 0, upright, roll so the pins point across the arms (either sign), angular rate 0.
- **Objective**: minimum propellant + weighted L1 norm of the virtual control + trust-region penalty (SOC epigraph).
- **Algorithm**: SCvx with virtual control and an adaptive trust region; converged when the virtual control and
  the state change fall below tolerances; at most 10 iterations per re-plan. Initial guess: the previous plan
  shifted in time, or (first plan) straight-line interpolation of position/velocity with a vertical attitude and
  a constant-deceleration thrust profile.
- **Size**: 2 x 15 nodes, ~1000 variables per SOCP; target 0.1-0.3 s per re-plan (printed by the tests).

## Execution in flight

- The first plan is computed when the landing burn is triggered (the simulation waits for it).
- Re-plan every 1.0 s of simulation time from a state snapshot, on a worker thread. A plan takes effect at the
  fixed simulation time snapshot + 0.5 s. In tests and at time warp the simulation waits if it is not ready
  (deterministic: identical runs give identical results). In real-time play (flag) it never waits and keeps
  flying the previous plan.
- The worker never calls SPICE (CSPICE is not thread-safe): the snapshot carries everything the planning model
  needs (Earth rotation, gravity at the site, the atmosphere table, vehicle data).

## Tracking (`gnc/descent_tracking`)

At the current time it samples the plan: reference attitude, angular rate, angular acceleration, thrust vector
and engine set. The existing attitude controller gets the reference attitude/rate with a feed-forward torque
(J * angular acceleration of the plan) and corrects the error with gimbals, grid fins and RCS. Throttle follows
the planned thrust magnitude with a small position/velocity correction; the engine set switches at the planned
time.

## Abort to the sea

- Trigger: a re-plan is infeasible, its virtual control stays above tolerance, or its terminal pin offset or
  lateral speed exceeds the catch tolerance.
- Then the tower keeps its arms open and SCvx re-plans a soft touchdown as far seaward as possible: objective
  = distance to a sea target + propellant, terminal velocity ~0 at the ground, keep-out cylinder around the tower
  and mount (linearised per node as a half-space).
- `config/launch_sites.toml`: `[site.sea]` with `direction_deg` (towards the sea) and `shoreline_distance_m`
  [E]; ground beyond the line is water. A touchdown or impact there is a splashdown ("Wasserung": booster lost,
  tower undamaged). The surface stays flat at the site height until terrain (M8).
- Honest limit: during the landing burn the divert capacity is a few hundred metres, so a late abort at
  Starbase often ends on the beach rather than in the water. An early abort (before or during the boostback)
  is a later extension.
- HUD tower line: "Turm: Abbruch"; outcome "Wasserung" / "gelandet" / "Aufprall".

## Fallback

If the first SCvx plan fails, the existing explicit landing-burn guidance takes over (catch target as today), so
a solver failure never leaves the booster without guidance.

## Testing

Unit (Catch2):
- Planning-model Jacobians vs central differences; discretisation vs RK4 propagation of the nonlinear model
  (defect < tolerance for a converged plan).
- SCvx on a nominal ignition state converges (virtual control < 1e-6 relative), satisfies all constraints at the
  nodes, and is deterministic (identical results twice).
- `SocpSolver` re-use: same results as `solve_socp`, structure built once.

Scenario (`[gnc][catch]`): booster placed in an ignition-like state above the site (from a recorded nominal
return) -> caught, offset < 1 m, lateral speed < 0.5 m/s.

Dispersion campaign (`[gnc][catch][validation]`), fixed seeds:
- 20 cases: lateral position +-50 m, velocity +-10 m/s, residual mass +-5 %, attitude error +-3 deg at ignition;
- plus one engine failure in the 13-engine phase;
- pass: >= 90 % caught within tolerance, no tower impact, every abort ends in a splashdown or a soft touchdown
  without tower damage. Re-plan times printed with `STARSIM_VERBOSE=1`.

Integration: full returns of blocks 1-3 at Starbase -> caught.

## Out of scope

Optimising the ignition time and the descent phase; grid fins in the planning model; wind (M8); early abort
before the boostback; the ship landing (same machinery later, own phases and cones); real coastlines (M8).

## Refinements during Plan 3a

- The node shared by both phases takes the final (3-engine) set's limits: 13 engines at minimum throttle give more
  thrust than 3 at full, so the intersection is empty; the thrust steps down over phase 0's last interval.
- Convergence: virtual control below tolerance AND (step below tolerance OR final mass changing < 100 kg) AND the
  new iterate's nonlinear interval defects < 5 cm. The mass criterion stops the slow creep towards the exact fuel
  optimum; the defect check keeps the plan flyable.
- Iterative refinement in the KKT solves stops when a step no longer shrinks the residual (ratio 1, not
  Clarabel's 5: with 5 the SOCP landing fixture needed 50 instead of 17 iterations).
- Review fixes: the catch ends upright with any roll (the thrust has no roll authority; the tracking controller
  rolls the pins towards the arms with gimbal roll / RCS); no state limits on the measured start node; minimum
  phase durations only for a fresh plan; mass flow affine in thrust (pressure thrust loss); interval defect
  tolerance 1 cm.

## Follow-ups (from the Plan 3a review, for Plan 3b)

- Timing: failure cases run all 10 iterations (430-600 ms); add an iteration budget / early stop on a stalled
  virtual control and keep one `SocpSolver` per (start phase, goal) across re-plans.
- Refinement keeps a residual-increasing correction step (test before accepting).
- Abort plans need a ground constraint at the intermediate nodes.
- `plan_descent` leaves `status == Optimal` and stale diagnostics when not converged: callers check `converged`.
- Tilt limit not tapered towards the catch; the trust region is a fixed penalty (not adaptive); thrust bounds at
  site pressure (~2 % off at 1.2 km); hard-coded pi in `DescentLimits`.
- Tests to add: NumericalError in the middle of a reused solve; warm start from a perturbed state;
  `shifted()` should use the snapshot for node 0.
- Engine start-up (Raptor `startup_s` 3 s) vs a 1.6 s phase 0: the tracking layer must handle the transient.

## Decision 2026-10-13: robustness target for Plan 3b

Measured during Plan 3b: the catch is precise for small offsets at ignition (centred: 0.05 m, 0.19 m/s) but large
diverts during the landing burn (22 m and more) fail: the planning model has no grid fins (their side force while
they deliver control torque is ~0.7 MN, ~2 m/s^2) and the attitude loop lags ~0.3 s, both growing steeply with the
divert. In a full return the grid-fin descent delivers the booster with ~4 m miss to ignition. User decision: the
Plan 3b campaign uses realistic dispersions at ignition - lateral +-10 m, velocity +-3 m/s, residual mass +-5 %,
attitude +-3 deg, plus one engine failure - with the same pass criteria (>= 90 % caught within < 1 m / < 0.5 m/s, no
tower impact). Large diverts (+-50 m; grid fins in the planning model, attitude control) are a follow-up.

## Refinements during Plan 3b

- Catch arms: standby at 6.5 deg from 10 s before pin contact, late force-limited closing, damage by hull force
  (> 6 MN) or overlap (> 1 m) instead of geometric contact (user decision "Realistischer machen").
- Tilt, rate and corridor limits are soft (exact L1 penalty) so re-plans from a slightly violating state stay
  feasible; the engine switch is hard-calm (tilt <= 3 deg, rate <= 0.03 rad/s) and happens at 70-95 m/s sink.
- Gentle end: terminal thrust at most minimum + 5 % of the maximum; throttle-rate limit from the engine config.
- Only a catch found unreachable (virtual control >= stall threshold) twice in a row aborts; plans stopped at the
  iteration limit with small virtual control are flown.
- Abort plans keep clear of the tower and of the launch mount.
- Lateral tracking: LQR per plan axis on (position, velocity, tilt, tilt rate) with the gimbal's wrong-way side
  force (c = I / (m L)); control effort scale 0.3 x the gimbal tilt acceleration at the current thrust, output
  clamped to it; torque straight to gimbals/RCS, grid fins neutral during the SCvx burn. Cascaded position ->
  attitude tracking swung up on 3 engines (campaign 0/21); with the LQR 11/21; with the plan limited to 30 % of
  the gimbal 20/21; with the effort scaled to the available authority 21/21.
- Burn profile after the user's catch-video data (13 engines from ~2 km at ~333 m/s for ~5 s; switch at
  ~83 m/s, ~500 m; 3 engines ~20 s): ignition timed for 13 engines at 0.5 throttle, switch at 83 m/s, final
  throttle 0.6 (separate from the explicit pad-landing profile). Scenario result: ignition ~1.5 km at 260 m/s,
  13 engines ~4.6 s, switch ~650 m, 3 engines ~15 s.
- Re-plan times (Release, 24 cores): first plan 0.2-1.1 s, re-plans typically 0.1-0.5 s, worst seen ~2 s on the
  longer catch profile (the 0.5 s latency is then exceeded; tests wait, real-time play keeps the old plan).
- Open: full returns arrive far too fast (see CLAUDE.md known issues); descent drag needs calibration.
