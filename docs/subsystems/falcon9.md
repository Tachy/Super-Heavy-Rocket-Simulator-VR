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

## Standing decisions
- Physics first (user 2026-10-14): gauges the engines cannot explain are not followed (Falcon 9 MECO 25 t vs gauge 47).
- Falcon 9 first-stage propellant 425 t (2026-10-14); RCS on/off with fixed thrust, no
  continuous mode (user 2026-10-14).

## Validation
- Leg drop tests `[legs]`: 1/3/6 m/s land (6 m/s crushes 0.36 m), 10 m/s and 25 deg at 2 m/s are lost.
- Falcon 9 RTLS: from ignition (3.3 km, 300 m/s, 12.1 t) lands 0.01 m off at 0.88 m/s; campaign
  `[falcon9][rtls][validation]` (+-10 m, +-3 m/s, +-5 % propellant, +-3 deg, hull aero +-10 % via
  `FlightSession::set_aero_dispersion`) 21 of 21 within 0.01 m, <= 0.99 m/s, >= 5.7 t left; full mission MECO T+140 s,
  landing burn T+423 s at 2.97 km / 287 m/s / 12.1 t, touchdown T+452 s, 6.4 t left (CRS-35: 137 / 355 / 420 at ~3.2
  km, ~300 m/s / 442 s). Test "Falcon 9 descent drag reproduces ..." (unguided replay after the entry burn: 0.9 / 1.5 %
  rms). CRS-35 ascent: -9.5 % at T+30 s, +2-5 % after max-Q, MECO T+132.7 s / 58.2 km / 6596 km/h vs 134.6 / 57.4 /
  6490, SECO T+508 vs ~512 s (dials' time base 3.42 / 5.87 s, `crs35_ascent.toml`).
- Drone ship: full mission (kick 13 deg) MECO T+152 s, 64.8 km, 8373 km/h, 24.7 deg vs Starlink 15-25 T+149 s, 60 km,
  ~8000 km/h, ~25 deg; 16.5 t to 200 x 281 km; entry burn T+379-399 s, landing burn T+476 s at 3.0 km / 271 m/s /
  12.0 t, landed T+515 s 0.01 m off; campaigns `[falcon9][asds][validation]` from ignition and from MECO states (+-200
  m, +-5 m/s, +-5 %, aero +-10 %; two halves) 21 of 21 each.

## Diagnostics
- Falcon 9: `STARSIM_DEBUG_LEGS=1`, `STARSIM_DEBUG_ASCENT_G=1`; hidden test "Falcon 9 ASDS single MECO case"
  (`STARSIM_ASDS_SEED=<101-120>`).

## Open points and known limits
- Falcon 9 drone-ship landing burns end at the SCvx reserve (5 t [E]) after ~36 s; the gauge shows 7.2 t at touchdown
  after a ~23 s burn, and a 7.2 t reserve is infeasible from the recorded ignition state (unmeasured subsonic and
  power-on drag [E]).
