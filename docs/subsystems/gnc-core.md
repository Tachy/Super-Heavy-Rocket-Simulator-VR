# Shared GNC: ascent, tower catch, booster return, SCvx landing, grid fins, RCS

History and the reasons behind superseded decisions: `docs/history/guidance-missions.md`.
Index of the guidance / missions topics: `docs/subsystems/guidance-missions.md`.

## Model
- Ascent: `gnc/ascent_guidance` (pitch program + PEG + terminal v_go steering), `gnc/burn_node` (closed-loop burns),
  `gnc/peg_steering`. Burns pick the strongest engine configuration whose start/shutdown impulse is <= 1/4 of the
  delta-v; smaller burns fly on RCS translation along +x when the vehicle has thrusters along +x (ship aft ullage
  thrusters, Falcon 9 second stage), else they are rejected.
- Catch tower (`contact/*`, `sim/catch_tower_sim`): a site loaded with `config::load_launch_site` /
  `sim::launch_site_from_config` carries its tower; stacks stand on the launch mount (20 m). The tower closes its arms
  automatically (time to pin contact <= closing time + 1 s, axis offset <= 3 m), applies penalty contact forces (pins
  and a hull ring vs the arm boxes, friction 0.3) through `FlightBody::apply_external_wrench`, and judges: caught (both
  pins, engines off, at rest 2 s -> `FlightBody::hold_fixed_to_earth`), impact (tower/mount hit, pin load >
  `max_load_n`, carriage hard stop > 1 m/s, an arm > 0.2 m deep in the hull) or missed. Fidelity flag
  `FlightSession::set_towers`; `FlightSession::place_body` places a vehicle freely (scenarios).
- Booster return (`gnc/booster_return`, `FlightSession::set_booster_return(true)`): flip, Lambert boostback with
  drag-aware aim-point shooting, coast (RCS impact trim above ~2 kPa, below it grid-fin steering with the angle of
  attack found by bisection on the aero model), landing burn (explicit fallback 13 -> 3 engines; SCvx 13 -> 5 -> 3):
  1-D burn simulation for ignition, ZEM/ZEV vertical to the gate, approach profile sink = 1 m/s + height / tau,
  braking-profile divert. Parameters from the stage's config (`gnc::booster_return_parameters(vehicle, stage,
  drone_ship)`, planner plan Task 5; struct defaults are test fixtures): return profile keys (required on a returning
  first stage, `config/README.md`), grid fins' `max_return_alpha_deg` (coast steering, 8 deg [E]), legs'
  `deploy_margin_s` / `max_touchdown_tilt_deg` or catch pins' `max_catch_tilt_deg` (4 deg [E], also the SCvx plans'
  terminal tilt), and [D] the catch approach tau = the final set's throttle range time (`gnc::throttle_range_time_s`,
  Raptor (1 - 0.3) / 0.5 = 1.4 s, was 3 s [E]; explicit law and SCvx plans; a pad landing flies none). MECO planning
  derives the flip (`gnc::booster_meco_to_boostback_s`, `ascent-staging.md`). A catch-site abort cuts the engines 0.5 s
  after the abort plan's end at the water, as the pad landing does (the final set at minimum throttle lifts the light
  stage: it hovered there until dry). SCvx limits per landing phase (fix round 1): tilt `landing_phase_max_tilt_deg`
  and `landing_gimbal_fraction` [E] keys, the pitch/yaw rate [D] `gnc::landing_phase_rate_limit_radps` (as the ship's:
  min(gimbal slew, sqrt(alpha tilt)) at the landing-burn load; Super Heavy 0.349 / 0.254 / 0.139 rad/s, Falcon 9 RTLS
  0.141 / 0.218 / 0.089, were [E] 0.2 / 0.15 / 0.1). Landing-ignition forecast: the guidance's own criterion (burn
  prediction stops at the ground) found on the predicted fall (`ignition_on_fall`, bisection over the 1 s samples of
  the impact prediction), the touchdown that ignition plus that burn's duration. Own faster attitude controller with
  grid fins (`gnc::booster_return_attitude_gains`); rate command limited to what the available angular acceleration
  (gimbal or RCS, plus fins) can stop. Files: `booster_return.cpp`, `_prediction.cpp` (impact /
  landing-burn / MECO predictions, drag estimate), `_burns.cpp`, private `booster_return_detail.hpp`.
- SCvx landing planner (`gnc/descent_*`): `descent_phases` = 3 engine phases (13 -> 5 -> 3; 10 nodes per phase,
  neighbours share a node) in the plan frame (origin pad centre on the ground, x up, y east, z north; upright =
  identity attitude); `descent_model` (14-state rigid body, tabulated air/drag and hull normal force, Coriolis +
  centrifugal, numeric Jacobians; `make_descent_model` on the simulation thread), `descent_discretization` (FOH + RK4
  sensitivities), `descent_problem` (convex subproblem in scaled deviations: thrust/gimbal/tilt/rate/glide cones, catch
  or sea-abort goal with tower keep-out; shared phase node has the next set's bounds, engine switches have sink
  windows, the last one is calm), `descent_planner` (`plan_descent`: penalised trust region, warm start, converged when
  virtual control is small, step or final mass settled, nonlinear interval defects < 5 cm). A fresh plan from the
  initial guess starts twice (`gnc::SwitchGuess`, planner plan Task 5): the phase ends from the state (as late as the
  later sets can finish in their minimum phase times, not below what the set cannot remove within the height left,
  within the window) and in the windows' middle; it keeps the converged one with the most mass left (neither wins
  everywhere: the state start ~1 t heavier on booster sea aborts across a box, the middle 0.2-0.9 t on Falcon 9's
  recorded deck plans; the ship fixture `ship_return_dispersed1` needs both starts in the escalation). The HLS approach
  uses the same guess (its fixed kinematic phase times were removed in planner plan Task 6). A fresh plan that does not
  converge while its virtual control stays below `stall_virtual_control` (in reach, SCvx cycling) is solved again with
  the soft tilt / rate limits' penalty raised 10x, 100x, 1000x (`attitude_penalty_growth` 10, `attitude_penalty_raises`
  3 [E]; both starts at each weight); the converged one with the most mass left is the plan (exact penalty: too low a weight lets the subproblem
  lean beyond the limits into a cycle; ship first catch plans of lunar seed 15 / ship return case 1, fixtures
  `ship_*_first_catch_plan.txt`, Task 1c 2026-11-03). Converging fresh plans and re-plans are unchanged. Uses `optim::SocpSolver`.
  Nominal Block 3 case 6 iterations ~0.5 s fresh; re-plans 1-4 iterations < 0.5 s. In flight: `gnc/descent_snapshot`
  (the worker never calls SPICE), `gnc/descent_replanner` (worker thread, fixed sim-time latency),
  `gnc/descent_tracking`, `gnc/tvc_lateral` (LQR gains), `gnc/scvx_landing` (first plan, re-plans, abort to the sea,
  body torque); the tracking reference (position, velocity, axis, rate for the position loop and the lateral LQR) is the
  plan's own path, `gnc::plan_path`: each interval propagated through the planning model from its start node with
  the first-order-hold thrust, as the discretisation does, sampled every 0.05 s (`plan_path_step_s`; ~2 mm of chord
  at 7 m/s^2), built once per accepted plan on the plan's snapshot model. The nodes' chord (`DescentPlan::state_at`,
  still the warm start's) ran up to 3.3 m / 3 m/s / 1.6 deg off the ship's braking path between nodes ~1.9 s apart -
  during braking always above it - and the loop held the ship there: higher, slower, each re-plan later (flip and
  catch: catch prediction 4.30 -> 2.74 s, r1 fix 2026-11-06). `sim/landing_outcome` (shoreline `[site.sea]`, "Wasserung"); tower `CatchPhase::Aborted` (arms stay
  open). Re-plans may draw on half the propellant reserve at a penalty, aborts on all of it (not the ship); a converged
  plan that tilts the final set > 3 deg beyond its soft limit counts as unreachable where there is a sea.
  First plans in real-time play (planner plan Task 11b fix round 1, `gnc::FirstPlanHold`): planned on the worker from
  the state a known hold leads to (`predict_hold`, planning model, attitude held) and in effect at the hand-over
  `hold` later; a plan not there by then is waited for. Hold = 1.3 x the latest measured first-plan wall time of the
  kind (booster / ship / HLS; references 0.95 / 1.87 / 1.0 s), at least the plan latency, at most 3 s
  (`first_plan_hold_max_s`). The holds: the ship its flip engines coming down to minimum throttle at the set's
  throttle rate (`throttle_down`; the flip's 0.8 held 2.4 s turned the sink into a climb, all three attempts failed);
  the booster its unpowered fall, begun when the guidance's own ignition on the predicted fall (else the stop
  height's trend over >= 1 s: tick to tick it jitters by ~16 m, a 5 ms trend began it at 7.6 km) lies within the hold,
  lit at the hand-over (its start-up counts from there); the HLS its braking thrust, begun the hold before the
  approach t_go. Off (tests, warp): synchronous, unchanged. Re-plans in real-time play are used when they arrive, but
  one a re-plan interval (1 s) past its epoch of effect (snapshot + latency) is waited for. Rule [E]: the worst gap
  between plans is one missed synchronous cycle, 2 x `replan_interval_s` + `plan_latency_s`, however slow the machine.
  Its sim-thread hold is the solve's wall time less latency + interval (idle <= ~0.2 s; 6-12 s under the full
  parallel test load), logged per wait by `STARSIM_DEBUG_REALTIME` ("late landing re-plan"). The trials behind it:
  `docs/history/guidance-missions.md`.
- SCvx landing aborts (all vehicles; the pilot's `abort_landing` too): a first plan that finds the target unreachable
  aborts at once; `failures_before_abort` (2) re-plans with a virtual control above the stall threshold, or
  `unconverged_before_abort` (4) unconverged re-plans of any kind, give the target up and plan for the sea.
- Flap allocation (`AttitudeController`, details `ship-entry-return.md`): box-constrained active set; the lower bound comes from the true moments in the current air (`flap_lowest_effective_rad`, a quarter of the flap's best slope [E]), so a flap is not driven to where it has neither force nor slope. A flap already below it holds until the demand raises it.
- Grid fins (`aero/grid_fin`, `[stage.grid_fins]`): flat-plate normal force from the local flow at each fin (incl.
  omega x r), slope/limit over Mach from a table (Super Heavy `aero/super_heavy_grid_fin.csv`, lattice derivation: table 0.5 x the generic below Mach 0.9; Falcon 9 `aero/grid_fin.csv`); actuators with deflection and rate limits
  (`Vehicle::update_grid_fins`). The controller allocates torque to the fins first (minimum-norm least squares), the
  remainder to gimbals/RCS, using the last tick's air data (`FlightBody::air_data()`); only with
  `AttitudeController::Gains::grid_fins`. SCvx landing burns feather the grid fins into their local flow
  (`AttitudeController::feathered_grid_fins`). Each fin's axial drag stays in the hull's curve, on the axis. Its moment at the
  fin is a fidelity flag, default off (`FlightEnvironment::grid_fin_drag_couple`): Super Heavy's 0 / 90 / 180 layout would
  pitch about +y with 1.8 % of the fin authority by the derived CA (`aero-derivation.md`).
- Vehicle capabilities for planners (`gnc/vehicle_capability`, `[gnc][capability]`): stage pitch inertia about the
  stage's CG with the propellant settled aft - the flight dynamics' own parts (`vehicle::settled_stage_mass_properties`
  in `vehicle/stage_inertia`, shared with `Vehicle::mass_properties`: dry stage about its own CG, config
  `inertia_kg_m2` or the thin-walled cylinder; payload / fairing; each tank's liquid as a column on its bottom; parallel
  axis to the common CG; the main load split over the main tanks by capacity, headers / RCS at their configured mass).
  Equal to `Vehicle::mass_properties` to 1e-12 (Falcon 9 first stage alone); Falcon 9 at our RTLS separation (52.1 t)
  1.123e7 kg m^2, CG 11.46 m, matching the config comment's 1.12e7 / 30.4 m lever; at the drone-ship MECO load (46.75 t)
  1.065e7. An engine set's thrust at its common min / max throttle from the engine config (`EnginePerformance`), its
  smallest gimbal angle / rate, lever (stage-local CG station) and the gimballed engines' thrust at both ends (mixed
  engine types: Raptor SL + RVac); gimbal pitch acceleration `F_gimballed sin(gimbal) lever / I`; RCS pitch
  acceleration from the allocator's `max_torque_nm` (weaker transverse axis); rest-to-rest bang-bang turn time with a
  rate cap (throws for no authority). Planners derive their limits from these instead of carrying own numbers.
- `FlightBody::resting()`: carried by the ground after a gentle touchdown; resting bodies coast analytically (follow the
  Earth), impacted/resting bodies do not block coast segments. A freely falling body below 5 km above its ground is
  ticked.

### RCS
- Individual thrusters from `[stage.rcs]` pods (layouts [E]): Super Heavy 12 x 3 kN at the forward dome; ship 26 x 1.5
  kN fore and aft (two of each direction per pod) incl. two aft ullage thrusters along +x; Falcon 9 8 x 6.5 kN at the
  interstage [D] and 8 x 200 N on the second stage. On/off with fixed thrust (user 2026-10-14: no continuous mode).
  Per tick `Vehicle::update_rcs` allocates the commanded torque / force fractions (`propulsion/rcs_allocator`: box QP,
  exact active set on the 6 x n effectiveness about the current CG, rebuilt when the CG moves > 1 cm or a supply runs
  dry); `propulsion/rcs_modulator` turns duties into pulses >= 20 ms (4 ticks). Torque comes first; orbit manoeuvres
  set `VehicleCommand::rcs_keep_force` (balanced layouts, i.e. the ship, keep the net force as asked). One-station
  layouts (Super Heavy, Falcon 9) always turn with a push. Falcon 9 entry burn trims the impact miss (user
  2026-10-14): in the coast its ignition altitude is shot on the miss along the track (+-8 km [E]), in the burn its
  length (+-25 % [E]) and lean across (<= 10 deg [E]). A returning booster's coast slews are limited to 2 deg/s [E].
  Capability per axis (`max_rcs_torque_nm`) and per direction (`rcs_force_capability`) come from the geometry.
  Propellant: Falcon 9 N2 tanks (600 / 80 kg carved out of the dry masses; not in `propellant_kg()`),
  Starship / Super Heavy ullage gas from the main tanks (Isp 70 s [E]). The burn executor counts the realised RCS force
  while a translation is commanded. Not modelled: thrust rise / tail-off, short-pulse Isp loss, pressure blowdown.

## Validation
- Booster catch/return validation (`[gnc][catch][validation]`, booster descent drag) is in vehicle-data.md.

## Diagnostics
- Booster: `STARSIM_VERBOSE=1`, `STARSIM_DEBUG_LANDING=1`, `STARSIM_DEBUG_HOLD=1` with test `[booster]`.
- SCvx: `STARSIM_DEBUG_SCVX=1`, `STARSIM_DEBUG_SCVX_SOLVER=1`, `STARSIM_DEBUG_CATCH=1`; `STARSIM_DUMP_SCVX=<dir>` +
  hidden `[scvx-diag]` replays one plan request.
- HUD / game: NavBall (`ui/nav_ball.gd`), flight director (`ui/flight_overlay.gd`), trajectories
  (`vehicle/trajectory_view.gd`: green planned, orange predicted); keys P ascent autopilot, O circularize at apoapsis,
  R ship return, X autopilot off. Tower: `game/world/tower_view.gd` from `FlightSim.site_tower` / `tower_state`
  (render_check `--tower_arms_deg=`).
