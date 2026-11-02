# Falcon 9: vehicle, legs, RTLS, drone ship, game interface

History and the reasons behind superseded decisions: `docs/history/guidance-missions.md`.
Index of the guidance / missions topics: `docs/subsystems/guidance-missions.md`.

## Model
### Falcon 9 (milestone c)
- `config/vehicles/falcon9_block5.toml` (Block 5 only): octaweb groups `center` / `pair` / `ring`, 4 grid fins, cold-gas
  RCS, `[stage.landing_legs]`; sites `ccsfs_slc40` (launch, no tower) and `ccsfs_lz40` (`kind = "landing_zone"`, ~340 m
  east of the pad [E], 85 m pad [S]). Derived [D] (webcast telemetry of CRS-35 RTLS and Starlink 15-25 drone ship;
  `webcast_telemetry.py clean --vehicle falcon9`): first-stage propellant 425 t (decision 2026-10-14; gauge falls
  0.63 %/s at full thrust, nine Merlins at 845 kN burn 2.75 t/s; `tools/telemetry/mass_estimate.py --area-m2 10.52`),
  second-stage propellant 106.7 t, landing propellant 12.1 t (gauge 2.8-2.9 % at landing-burn ignition; touchdown 1.7 %
  = 7.2 t), MECO gauge 66 t RTLS / 47 t drone ship (not followed: the Falcon 9 gauge drops without burns), descent drag
  table `aero/falcon9_descent.csv`. Burns seen: CRS-35 entry burn on the centre engine only, landing burn 1 -> 3 -> 1;
  Starlink 15-25 entry burn on 3, landing burn on 1 (~40 % throttle). Ascent: full thrust (no `max_ascent_throttle`)
  under the Mach-dependent q limit `max_dynamic_pressure_pa` [D] (transonic hold 29.6 kPa, 35.6 kPa supersonic; see
  ascent-staging.md); `STARSIM_DEBUG_ASCENT_G=1` prints the simulated G. The upper stage's plane steering nulls
  out-of-plane speed and offset together (ZEM/ZEV), terminal v_go removes the rest; inclinations of steep planes are
  judged against the true equator of date. `FlightSession::launch_from_pad` also takes a loaded `VehicleConfig`
  (Dragon flies without a fairing); every launch starts with the autopilot off (`set_booster_return` stays).
- Landing legs (`contact/landing_legs`, `contact/landing_surface`, `sim/leg_landing_sim`): `Vehicle::deploy_legs`
  (sticky, `deploy_time_s`; `FlightSession::deploy_legs` / `set_leg_deployment`); once the legs move `LegLandingSim`
  owns the ground contact: four feet as spring-dampers in series with plastic crush cores (`stroke_m`,
  `crush_force_n`), friction capped at what stops a foot's slip within a tick. Surface: the site's ground plane with
  shoreline, or a deck over the sea (`FlightSession::set_landing_surface`). Judgement (`FlightSession::leg_touchdown`):
  "gelandet" (>= 3 feet, engines off, at rest 2 s -> `FlightBody::hold_landed`), "umgekippt" (> 30 deg on a foot),
  "Aufprall" (core used up, base on the surface, legs not fully out), "Wasserung".
- RTLS (`config/missions/falcon9_rtls.toml`, `config::load_ascent_mission`): `FlightSession::set_landing_zone` (after
  `launch_from_pad`) makes a `"leg_landing"` stage return to the zone. `gnc::booster_return_parameters(stage)` takes
  engine sets, return flight time, boostback loss fraction, entry burn and landing-burn profile from `[stage.recovery]`
  (Super Heavy has none of these keys). Phases Flip -> Boostback -> Coast -> EntryBurn ("Wiedereintritt", retrograde on
  the centre engine from 56.6 km for 125 m/s [D], lean <= 5 deg; ends after 5 s [E] without thrust - engines out - and
  the landing burn still comes) -> Coast -> LandingBurn. Landing burn: SCvx goal `DescentGoal::Pad` (terminal state at
  leg-contact height, sink 1 m/s, no corridor, no exponential approach), 1 -> 3 -> 1 engines, switch windows 230-270 /
  140-190 m/s [D]; legs deploy 5 s [E] + deploy time before the planned touchdown, engines stop on leg contact or 0.5 s
  after the plan's end; an unreachable zone aborts to the sea. `return_flight_time_s` 260 s and
  `boostback_loss_fraction` 0.12 [D].
- Drone ship (`config/drone_ships.toml`, `config/missions/falcon9_asds.toml`): `sim::drone_ship_site` (deck 91 x 52 m
  [S], 3 m freeboard [E]; `sea` marks the water beside the long edge); the mission's `[drone_ship]` places it at the
  nominal impact point of our ascent [D]. `sim::start_ascent_mission` sets a session up from an `AscentMission`.
  `[stage.recovery.drone_ship]` overrides the RTLS profile when the zone has a deck (`StageConfig::return_profile_for`,
  `FlightSession::return_parameters`): `boostback = false` (MECO plans run-down + entry burn + landing propellant only:
  24.7 t), entry burn on centre + pair from 66 km for 880 m/s [D] at 0.72, landing burn on the centre engine in all
  three phases [D] (`landing_phase_max_tilt_deg` 20). Generic supports: the impact prediction flies the pending entry
  burn as a finite burn with its lean (`PendingBurn`); the entry burn solves its lean by shooting on that prediction
  (`entry_max_lean_rad` 5 deg); leg landings aim the ballistic impact beyond the pad by the landing burn's lead
  (`landing_burn_lead_itrf`); the return guidance estimates drag online from the accelerometer
  (`FlightBody::set_accelerometer`, `drag_scale()`). A deck out of reach ditches beside the ship. Our MECO leaves 25 t
  (physics first, decision 2026-10-14; the gauge falls 47 -> 25.9 t without a burn).
- Game interface (c5): `FlightSim.start_mission(id, tdb)`, `mission_site_id`, `landing_zone()`, `body_legs(i)`,
  `body_grid_fins(i)`, `return_state(i)["legs"]`. The bindings build with C++ exceptions on
  (`GODOTCPP_DISABLE_EXCEPTIONS OFF`).

### First stage geometry (detail model plan 2026-11-02, Task 3)
- Stage length 47.7 m [S] with the interstage, from the outer Merlins' exit plane (station 0) to the interstage top; 41.2 m is the
  stage without its interstage, where tanks and engine bay end. A model's 48.4 m is that model's scale error, not averaged in.
- Physics thrust acts at station 0 (the exit plane); the Merlin's drawn gimbal point stands 1.7 m [E] above it.
- Fin hinge 43.35 m [E], RCS packs 41.9 m [E]: on the interstage, measured in `F_ORBCOMM_2`, `F_Falcon_9_first_stage_in_hangar`.
- Foot -2.1 m [E] (model M4, scale-true; the Starlink 15-25 frame at file 1124 s is weak support: strut 37 +-3 deg off the axis against
  38 deg at -2.1 and 40 deg at -1.5).

### Fin and leg azimuths
Evidence (photos, `screenshots/falcon9_ref/`; the reference models disagree, so they do not decide):
- `F_Landed_rockets_in_hangar_39A`, the stage seen from its top: the raceway runs on the centre line, the two near fins sit +-45 deg (+-5) off it,
  the leg covers at the far end also about +-50 deg: the legs are on the FINS' azimuths and the raceway between them.
- `F_ORBCOMM_2`: fins at about +-48 deg and leg covers at about +-40 deg from the same camera axis (+-10 deg); with a 45 deg offset one of the two
  would stand on the centre line and the other on the silhouette.
- Webcast frames (Starlink 15-25 file 796 s, 1104 s; Crew-13): the thruster pack stands between two neighbouring fins, as the config's pods.
So the fins stay at 45 / 135 / 225 / 315, the legs moved there from 0 / 90 / 180 / 270 and the pods stay at 0 / 180.
Ring engines: `F_Falcon_9_B1046`: the base's leg clevises lie midway between two ring engines, the ring engines at 22.5 + 45 k deg from them (model M4
agrees), so the config's ring stands at 22.5 + 45 k. The 3-engine set's `pair` is 22.5 / 202.5 [E], the opposite pair nearest the former 0 / 180; no photo shows which two
engines fly it. An opposite pair and the centre stay balanced: the thrust stays on the axis, as do all nine.

### Fresh landing plans (detail model plan 2026-11-02, Task 3)
The 47.7 m hull's normal force (area and lever: moment coefficient x 1.58) needs two settings on fresh SCvx plans, those with no
warm start: first plans and abort plans without one, on RTLS and drone ship alike. Re-plans keep the defaults (0.1 and 10).
- `landing_fresh_trust_region_weight` 0.3 [E] (`ScvxSettings::fresh_trust_region_weight`): at 0.1 the first deck plan stays in a
  limit cycle (defect 0.03-0.16 over 300 iterations, on the propellant floor).
- `landing_fresh_attitude_violation_weight` 100 [E] (`ScvxSettings::fresh_attitude_violation_weight`), penalty on the soft tilt and
  rate limits: at 10 the solver can settle on one swing to 26-28 deg against the final set's 20 deg, which `beyond_limits` refuses
  rightly, and the stage ditches. Which plan it finds flips with 0.5 m of start state; clean plans exist from the same state. At
  100: 0 of 240 perturbed starts refused. The price: first plans up to ~0.5 t less final mass than the swing; reserve unchanged.
- The guard is unchanged and there is no retry. Super Heavy and the ship hold 0 in both settings (plan requests of a whole flight
  identical to the former binary's).

## Standing decisions
- Physics first (user 2026-10-14): gauges the engines cannot explain are not followed (Falcon 9 MECO 25 t vs gauge 47).
- Falcon 9 first-stage propellant 425 t (2026-10-14); RCS on/off with fixed thrust, no
  continuous mode (user 2026-10-14).

## Validation
- Leg drop tests `[legs]`: 1/3/6 m/s land (6 m/s crushes 0.36 m), 10 m/s and 25 deg at 2 m/s are lost.
- Falcon 9 RTLS: from ignition (3.3 km, 300 m/s, 12.1 t) lands 0.01 m off at 0.88 m/s; campaign
  `[falcon9][rtls][validation]` (+-10 m, +-3 m/s, +-5 % propellant, +-3 deg, hull aero +-10 % via
  `FlightSession::set_aero_dispersion`) 21 of 21 within 0.01 m, <= 1.06 m/s, >= 5.7 t left, 0 of 497 plans failed; full mission
  MECO T+140 s, landing burn T+423 s at 2.96 km / 287 m/s / 12.1 t, touchdown T+455 s, 6.0 t left, N2 127 kg left = 27 % of
  the 473 kg used (CRS-35: 137 / 355 / 420 at ~3.2
  km, ~300 m/s / 442 s). Test "Falcon 9 descent drag reproduces ..." (unguided replay after the entry burn: 0.9 / 1.5 %
  rms). CRS-35 ascent: -9.5 % at T+30 s, +2-5 % after max-Q, MECO T+132.7 s / 58.2 km / 6596 km/h vs 134.6 / 57.4 /
  6490, SECO T+508 vs ~512 s (dials' time base 3.42 / 5.87 s, `crs35_ascent.toml`).
- Drone ship: full mission MECO T+150 s, 70.3 km, 8159 km/h, 24.8 t vs Starlink 15-25 T+149 s, 60 km,
  ~8000 km/h; 16.5 t to 200 x 281 km; entry burn T+397-418 s, landing burn T+490 s at 2.95 km / 271 m/s /
  11.8 t, landed T+529 s 0.01 m off, 5.0 t left; campaigns `[falcon9][asds][validation]` from ignition and from MECO states
  (+-200 m, +-5 m/s, +-5 %, aero +-10 %; two halves) 21 of 21 each, >= 4984 kg left, 3 of 702 and 25 of 713 plans failed
  (none of them a first plan). Fixtures of the refused first plans: `falcon9_asds_seed105_first_plan.txt`, `..seed109..`.

## Diagnostics
- Falcon 9: `STARSIM_DEBUG_LEGS=1`, `STARSIM_DEBUG_ASCENT_G=1`; hidden test "Falcon 9 ASDS single MECO case"
  (`STARSIM_ASDS_SEED=<101-120>`).

## Open points and known limits
- Falcon 9 drone-ship landing burns end at the SCvx reserve (5 t [E]) after ~36 s; the gauge shows 7.2 t at touchdown
  after a ~23 s burn, and a 7.2 t reserve is infeasible from the recorded ignition state (unmeasured subsonic and
  power-on drag [E]).
