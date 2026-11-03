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
  `launch_from_pad`) makes a `"leg_landing"` stage return to the zone. `gnc::booster_return_parameters(vehicle, stage, drone_ship)`
  takes engine sets, return flight time, boostback loss fraction, entry burn and landing-burn profile from
  `[stage.recovery]` (required keys: `config/README.md`; Super Heavy has its own since planner Task 5), the fins' steering
  limit and the legs' touchdown keys; the catch approach time is derived (Merlin (1 - 0.4) / 0.5 = 1.2 s; a pad landing
  flies none). Phases Flip -> Boostback -> Coast -> EntryBurn ("Wiedereintritt", retrograde on
  the centre engine from 56.6 km for 125 m/s [D], lean <= 5 deg; ends after 5 s [E] without thrust - engines out - and
  the landing burn still comes) -> Coast -> LandingBurn. Landing burn: SCvx goal `DescentGoal::Pad` (terminal state at
  leg-contact height, sink 1 m/s, no corridor, no exponential approach), 1 -> 3 -> 1 engines, switch windows 230-270 /
  140-190 m/s [D]; legs deploy `deploy_margin_s` 5 s [E] + deploy time before the planned touchdown, engines stop on leg contact or 0.5 s
  after the plan's end; an unreachable zone aborts to the sea. `return_flight_time_s` 260 s and
  `boostback_loss_fraction` 0.157 [D].
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
  100: 0 of 120 perturbed starts refused at the flown 0.3 / 100 (240 runs in all, half of them at other settings; not re-counted over campaigns). The price: first plans up to ~0.5 t less final mass than the swing; reserve unchanged.
- The guard is unchanged and there is no retry. Super Heavy and the ship hold 0 in both settings (plan requests of a whole flight
  identical to the former binary's).

### Stack (plan 2026-11-02, Task 10)
- Height with Dragon 65.6 m [D]: first stage 47.7 m [S] + second stage above the interstage top 9.8 m [D] + Dragon 8.1 m [S]. Was 69.6 m: the published 13.8 m second stage counts its nozzle, which hangs inside the interstage.
- 9.8 m measured from the interstage's top edge to the trunk's base, at the 3.66 m hull's own scale: `D_SpaceX_Crew_9_Preflight` 587 px / 220 px = 9.77 m, `F_NHQ202210010026` 758 / 281 px = 9.87 m, +-0.2 m (`tools/structures/measure/stack_bands.py`; both long telephoto shots from the ground). Not averaged with: 70 m - 13.1 m - 47.7 m = 9.2 m (70 m is rounded; the fairing stands on the ledge about 1.2 m below the trunk's base, 47.7 + 8.6 + 13.1 = 69.4 m), `F_KSC_20200429` 9.1 m (a SpaceX illustration, its hull 5 % too thick against its length), reference model M1 (stack total 64.6 m, split wrong). Crew-13 t2880 is too coarse (5.4 px/m, the dark band is the trunk's cells, not the joint).
- Representation: the stage's `length_m` is the hull above its base (the interstage top); nozzle and engine lie below station 0. Why not a `stack_overlap_m`: `Vehicle` adds up `length_m` for stations and `length_m()`, and so do `station_collision` and the shift on separation; `FlightBody`, `aero` and the descent model read `length_m()` as the hull. An overlap would need all of them to subtract it again, and only the case with a stage above. Here no core code changes; engine, tanks and pods can sit at any station already (`engine_group.station_m`). The stage alone is 9.8 m of hull in physics (aero length, cross-flow, sun area: it flies alone from about 62 km at a few hundred Pa [E], near-zero alpha and tiny against its mass; its dry inertia also uses 9.8 m, `vehicle.cpp`) and 13.8 m drawn, nozzle exit to top (tested).
- Stations of the second stage, from the interstage top: gimbal -0.6 m [D] (13.8 - 9.8 - 3.4 m gimbal to exit [E]; ASSUMES the published 13.8 m ends at the trunk's base at 9.8 m. If it ends at the ledge at 8.6 m, the exit is 5.2 m below the interstage top, station 42.5 m, still 1.3 m over the dome, gimbal -1.8 m. No source settles it; for the user), nozzle exit -4.0 m = stack 43.7 m, 2.5 m above the first stage's tank dome at 41.2 m; LOX tank -0.15 .. 5.79 m, RP-1 5.95 .. 9.58 m (their lengths come from mass and density; tanks 0.2 m apart, top 0.2 m under the top plane) [E]; dry CG 2.4 m [E], the former fraction of the height above the gimbal. Nothing here is measured but the 9.8 m.
- Separation: 1 m/s of pushers x 6 s to ignition = 6 m > 4.0 m of nozzle in the interstage (tested, straight line only). The first stage holds until the nozzle is clear (see "The flip"). `game/tests/separation_clearance.gd` (flown Dragon mission, 0.25 s steps, exit code is the gate): the wall never cuts the drawn bell, smallest clearance 0.035 m at 3.75 s, nozzle exit above the rim from 4.0 s. No collision between the parts is modelled.
- Dry CG of the first stage (12.0 m [E]): reviewed, unchanged: its mass already included the interstage, so the 47.7 m length does not move it. `vehicle.cpp` derives the inertia from the length, so that did change with Task 3.
- Payload and fairing stand above the stage: `payload_station_m` 14.0 m [E] (Dragon's own CG, 4.2 m over the 9.8 m plane, also taken for the Starlink stack, not measured for it), `fairing_station_m` 14.2 m [E] (thin shell on the 8.6 m ledge). Without the keys a stage keeps 0.75 of its length (Starship). Was 7.35 m, in the RP-1 tank. N2 tank's base at 1.0 m lies inside the LOX column in the mass model [E]; harmless.
- Length against photos (Task 12, `screenshots/stack_height/ratio_check.png`): `F_NHQ202210010026`, side-on, 286 +-3 px hull. Photo vs ours in hull diameters: Dragon 2.12 vs 2.21, second stage 2.65-2.70 vs 2.68, interstage 1.84 vs 1.78; stage below the interstage only to where the deck hides it, 10.21 vs 11.26. Perspective: at most about 1 % (camera about 300-1100 m off, from the lens range; not known). The difference in the whole stack, 16.8 vs 17.9 diameters, is the transporter deck hiding the last 3.8 m of the stage in every photo, with our stack seen free to the nozzles: at station 3.8 ours is 61.8 m, the photo 61.5 m. Not perspective and not a shown wrong measure; the published 41.2 m stays, and the hidden end cannot be checked in this photo. One residual: leg tips 29.0 m below the interstage base in the photo, about 30.3 m in ours (1.3 m low, > the error). Values unchanged.
- Fairing: only a mass and a `nose_length_m`. The stack without Dragon ends at 57.5 m where a real one with fairing stands 70 m tall, so its aero length is short by the fairing (was 61.5 m); not modelled, and no Falcon flight in the tests shows it above limits.

### The flip (plan 2026-11-02, Task 14)
Law (`gnc::BoosterReturnGuidance`, Flip phase with `flip_ignition_off_velocity_deg` > 0; Super Heavy has none of these keys):
hold `flip_hold_s` on the attitude controller -> full RCS torque tipping the top towards the nadir, no braking, no torque above `flip_max_rate_deg_s`, fins feathered -> ignition `flip_ignition_off_velocity_deg` off the velocity, or at `flip_ignition_timeout_s` at the latest -> no torque until the first thrust -> the centre engine, the rest `boostback_buildup_s` later and within `boostback_full_thrust_pointing_deg` -> the gimbal ends the turn. Drone ship: no turn law, only the hold (15 s), then the coast's steering.

Evidence, webcast frames (`screenshots/falcon9_flip/frames`, `tools/telemetry/flip_thrust_direction.py`). Flights with boostback: Crew-13 (day, onboard view up the interstage), CRS-35 (night, dials only). Starlink 15-25 has none. Bandwagon-3 is not in the webcast folder.
| value | measured | flights | error |
|---|---|---|---|
| hold | second stage's bell within ~10 deg of the axis to 4 s after separation; timelines 4 s | Crew-13; three timelines | whole seconds |
| drone ship hold | second stage within ~10 deg to T+2:48, separation T+2:33: **15 s is a lower bound** (the camera view ends at T+2:48 and the plume washes out T+2:42-43; the stage had not visibly turned; jets at T+2:51 would bound it to ~18 s) [D] | Starlink 15-25 | censored, not +-2 s |
| turn direction | over the ground: earth limb enters the view at the top of the interstage hole at video T+2:41 and leaves at the bottom at T+2:43.5, clouds fill the hole | Crew-13 | one flight |
| rate during the turn | limb crosses ~32 deg in 2.5 s: 13 deg/s | Crew-13 | +-4 deg/s: the hole's 32 deg comes from geometry [E] (rim 1.83 m, ~6.5 m away) |
| acceleration | 1.6-1.9 deg/s2 from the start at the hold's end through those two limb positions | Crew-13 | two points, two unknowns: not checked independently |
| ignition -> three engines | 3.5 s: speed dial steps | CRS-35, Crew-13 | +-0.5 s |
| separation -> ignition | 13.0 s (CRS-35 dials: separation T+141.5, speed leaves its ballistic decline T+154.5; own record, `docs/history/ascent-staging.md`); 12.5 s with ignition at T+154; Crew-13 dial reading ~14.5 s [D]; 13.7 s (three timelines) | CRS-35, Crew-13; three timelines | +-1 s; +-0.6 s |
| ignition -> axis at the final attitude | 6 s; then horizontal uprange (+-5 deg from the speed and altitude dials) | CRS-35, Crew-13 | +-0.5 s |
| attitude and rate at ignition | 114 +-15 deg off the velocity, 16 +-4 deg/s: **extrapolated**, the onboard view is cut before the ignition | Crew-13 | only an extrapolation |
The frames cannot show: the attitude under thrust directly (only the thrust direction through the dials), the gimbal angles, the RCS pulses during the turn (jets are visible only later, T+3:37-3:41 Crew-13, T+2:51-2:57 Starlink), the dial's lag against the video (dials lead the pictures by ~2.5 s by the separation, one reading; not used across the two). The route over the ground rests on one daylight flight, against the shorter way up; thrust directions cannot tell them apart. No public source gives the RCS thrust or the N2 load (searched: SpaceX, Wikipedia, spec sheets, AIAA guide).

Flown (`falcon9_rtls`): hold tilt 0.01 deg; ignition 14.7 s after separation, 114 deg off the velocity at 20.2 deg/s; three engines at +3.5 s; final attitude at +6 s. Misses: 14.7 s misses the timelines' 13.7 +-0.6 s by 0.4 s beyond the band (1.0 s off the mean) and CRS-35's own 13.0 s; no dial reading we can confirm contains it. 20 deg/s is not measured: the 16 +-4 deg/s extrapolation ends at 20, the flown 20.2 lies just outside it, and 16 does not follow from 13 deg/s at mid-turn plus 1.75 deg/s2 (that gives ~19.6). The MECO plan's separation -> ignition is derived since planner Task 5 (`gnc::booster_meco_to_boostback_s`: hold 4 s + free turn to 114 deg at the RCS' 2.02 deg/s2 on the stage alone at ~52 t, capped at 20 deg/s = 14.65 s; was the fitted 14.8 s); `boostback_loss_fraction` 0.157 stays identified on our own RTLS simulation, not measured from flights [D]. Not tuned. A gate ~14 deg earlier would light ~0.7 s sooner (estimate, unflown): an option for the user, not done. Report, not gate (user 2026-11-03: "report, not gate; physical gate kept"): `test_falcon9_rtls.cpp` gates only the physics (landed upright within 5 m of the LZ-40 centre, hold tilt < 0.5 deg, N2 floor, three engines after the ignition); the webcast agreement (flip time, ignition angle and rate, landing-burn state, timeline) is the nightly case `[nightly][webcast]`, which writes `webcast_report/falcon9_rtls_timeline.md` to the test output directory. The misses above are rows there (flip time 14.75 s and rate 20.2 deg/s outside their bands), no longer pins.

N2 at the flown law, RTLS (kg): ascent 83, hold 4, turn 189 (= 130 kNs, I w / lever: 1.12e7 x 0.35 / 30.4 m), boostback 0, coast and landing 32, 292 left of 600. At 13 kN: ascent 92, hold 7, turn 189; ignition at 12.25 s. The turn's impulse does not depend on the thrust, as nothing brakes. The former law used 358 kg on the turn (acceleration and braking) and 129.5 kg stayed at the touchdown.

Cold-gas doubt: 6.5 kN at Isp 70 s is 9.5 kg/s per thruster (at 100 bar a throat of ~2.3 cm, at 30 bar ~4.2 cm; CF 1.6 [E]). The measured acceleration asks 5-6 kN at the stage's inertia, 13.7 s ~7.6 kN. Not ruled out, not confirmed. If the stage's inertia or a moment we do not model (separation push, propellant) carries more, the thrust is lower. 600 kg N2 is our own value, nothing says what the real tanks hold; the measured need is ~190 kg per turn at our inertia.

Fins do not help: they stay stowed through flip and boostback on RTLS flights (Crew-13: appear T+3:45-3:47; CRS-35: T+3:47; boostback ends ~T+3:20), and out at T+2:51-3:03 on Starlink 15-25, 18-30 s after separation. In the model over the turn at q 197-16 Pa: all fins at full deflection 13 kNm falling to 0.2 kNm, hull moment up to 37 kNm, RCS 396 kNm. Physics has no fold state for fins (they always carry their table); the turn law feathers them, so they give no moment; the hold and the boostback run on `return_controller_` with fins allocated first, at the above negligible moment. The fin law of the coast (`aero_steering_min_q_pa` 2000 Pa) does not reach the flip. The hull's aerodynamic moment carries about 9 % at most.

Outcomes: dispersed separation (+-5 deg, +-1 deg/s, +-5 % propellant, aero +-10 %, +-3 m/s, 13 cases, `[flip]`): all land, ignition <= 15.2 s, >= 5189 kg left; 12 within 0.04 m, case 4 is `docs/known-issues.md`. RCS at half thrust: lights at the time limit 17.05 s, lands 0.01 m off, 5222 kg left. Dead RCS: lights at 17.05 s, the gimbal turns the stage, lands 93 m off (outside the pad; cause not examined), 5061 kg left.

## Standing decisions
- Physics first (user 2026-10-14): gauges the engines cannot explain are not followed (Falcon 9 MECO 25 t vs gauge 47).
- Falcon 9 first-stage propellant 425 t (2026-10-14); RCS on/off with fixed thrust, no
  continuous mode (user 2026-10-14).

## Validation
- Leg drop tests `[legs]`: 1/3/6 m/s land (6 m/s crushes 0.36 m), 10 m/s and 25 deg at 2 m/s are lost.
- Falcon 9 RTLS: from ignition (3.3 km, 300 m/s, 12.1 t) lands 0.01 m off at 0.88 m/s; campaign
  `[falcon9][rtls][validation]` (+-10 m, +-3 m/s, +-5 % propellant, +-3 deg, hull aero +-10 % via
  `FlightSession::set_aero_dispersion`) 21 of 21 within 0.01 m, <= 1.06 m/s, >= 5.7 t left, 0 of 497 plans failed; full mission
  MECO T+140 s, landing burn T+423 s at 2.98 km / 288 m/s / 12.1 t, touchdown T+455 s, 6.0 t left, N2 292 kg left = 95 % of
  the 308 kg used (CRS-35: 137 / 355 / 420 at ~3.2
  km, ~300 m/s / 442 s). Test "Falcon 9 descent drag reproduces ..." (unguided replay after the entry burn: 0.9 / 1.5 %
  rms). CRS-35 ascent: -9.5 % at T+30 s, +2-5 % after max-Q, MECO T+132.7 s / 58.2 km / 6596 km/h vs 134.6 / 57.4 /
  6490, SECO T+508 vs ~512 s (dials' time base 3.42 / 5.87 s, `crs35_ascent.toml`).
- Drone ship: full mission MECO T+150 s, 70.3 km, 8159 km/h, 24.8 t vs Starlink 15-25 T+149 s, 60 km,
  ~8000 km/h; 16.5 t to 200 x 281 km; entry burn T+397-418 s, landing burn T+490 s at 2.95 km / 271 m/s /
  11.8 t, landed T+529 s 0.01 m off, 5.0 t left; campaigns `[falcon9][asds][validation]` from ignition and from MECO states
  (+-200 m, +-5 m/s, +-5 %, aero +-10 %; two halves) 21 of 21 each; with the Task 14 flip (hold 15 s) >= 4983 kg left (ignition) / >= 4984 kg (MECO), 6 of 697 (ignition) and 22 of 718 (MECO) plans failed (before: 3 of 702, 25 of 713; none of them a first plan). Fixtures of the refused first plans: `falcon9_asds_seed105_first_plan.txt`, `..seed109..`.

## Diagnostics
- Falcon 9: `STARSIM_DEBUG_LEGS=1`, `STARSIM_DEBUG_ASCENT_G=1`; hidden test "Falcon 9 ASDS single MECO case"
  (`STARSIM_ASDS_SEED=<101-120>`).
- Webcast agreement of the drone-ship flight (landing-burn state, entry / landing burn times against Starlink 15-25, entry burn propellant 13.2 t) is report, not gate (user 2026-11-03: "report, not gate; physical gate kept"): the normal-tier cases gate orbit, the entry burn engine set, delta-v band, impact < 300 m, landed upright within 5 m of the deck centre; the `[nightly][webcast]` cases write `webcast_report/falcon9_asds_timeline.md`, `falcon9_asds_entry_burn.md` (also `falcon9_ascent_crs35.md`, `falcon9_descent_*.md`). RTLS: the normal-tier case never asserted an orbit (the audit spec lists "orbit"); no orbit assertion was added, the gate is landed upright within 5 m of the LZ-40 centre. The report keeps only rows with a sourced reference and tolerance (the touchdown time row was dropped: no source).

## Open points and known limits
- Falcon 9 drone-ship landing burns end at the SCvx reserve (5 t [E]) after ~36 s; the gauge shows 7.2 t at touchdown
  after a ~23 s burn, and a 7.2 t reserve is infeasible from the recorded ignition state (unmeasured subsonic and
  power-on drag [E]).
- End state of the detail-model plan (2026-11-02): the first-stage foot at -2.1 m rests on weak evidence (one reading), the leg is drawn as one beam and the pusher stretches instead of telescoping, the fins fold by a view rule with no physics state (they snap out on separation). The Raptor nozzle (drawn 1.30 m official against the physics config) is its own later topic: Raptor physics is unchanged. Fresh leg-landing plans use trust region 0.3 and tilt penalty 100 (Falcon only; see "Fresh landing plans"). Cost of the model on the pad and in landing: `vehicle-models.md` "Cost of the Falcon 9, Dragon and ISS models" (no measurable cost over cylinders).
