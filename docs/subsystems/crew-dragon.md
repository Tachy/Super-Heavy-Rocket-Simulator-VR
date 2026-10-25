# Crew Dragon: vehicle, entry, parachutes, splashdown, game interface

History and the reasons behind superseded decisions: `docs/history/guidance-missions.md`.
Index of the guidance / missions topics: `docs/subsystems/guidance-missions.md`.

## Model
### Crew Dragon (spec `docs/superpowers/archive/specs/2026-10-14-rcs-crew-dragon-design.md`)
- d2 (`config/vehicles/crew_dragon.toml`, `core/tests/test_crew_dragon.cpp`): capsule + trunk, 12.5 t [S] (capsule dry
  8.4 t [D], 2,562 kg MMH/NTO [S], trunk 1.5 t [E]). 16 Dracos (400 N, Isp 300 s, 10 ms pulses) [E layout]: 4 under
  the nose cone push aft (only with the cone open, `Vehicle::set_nose_cone_open`, `FlightSession::set_nose_cone`), 4
  side pods give pure couples; no lateral translation. 8 SuperDracos are an `abort_only` group: data only. Payload
  vehicles: a mission's `[payload]` stacks via `config::stack_payload`; `FlightSession::set_payload_separation` /
  `separate_payload`: released at cutoff + 30 s with a 0.3 m/s push [E] (`AutopilotState::PayloadSeparation`
  "Nutzlasttrennung"); the capsule opens its nose cone 600 s later [E]. Orbit attitude modes (`gnc/orbital_attitude`:
  Inertial, Lvlh, Direction, Sun; `FlightSession::set_attitude_mode`) on `orbit_controller_`: slews <= 1 deg/s on every
  axis (`Gains::rcs_max_roll_rate_radps`), hold deadband 0.5 deg / 0.05 deg/s [E]; any mode but Free keeps the session
  ticking. RCS burns fly along +x when the vehicle has thrust along it, else -x.
- d3 entry (`core/tests/test_capsule_aero.cpp`, `test_capsule_entry.cpp`): `aero/capsule_aero` from
  `aero/crew_dragon_aero.csv` (modified Newtonian [D] by `tools/aero/gen_capsule_aero.py`: spherical-section shield
  R_n = 1.2 D [E], 15 deg sidewall; [E] below Mach 2, subsonic C_A ~0.97; hypersonic C_A 1.68 at 0 deg, ~10 % above
  Apollo/Orion's measured values; no rarefied bridging, no damping derivatives). Used once the capsule is the lowest
  stage (`Vehicle::capsule_aero`, `hull_aero`). Lateral CG `[[stage]] dry_cg_offset_m` 9.6 cm [D] trims the 10.5 t
  entry capsule at 12.5 deg, L/D 0.20 [E: spec], C_D 1.57. `gnc/capsule_entry_control`: Dracos roll the lift about the
  air velocity (bank rate <= 10 deg/s [E]) and damp pitch / yaw (deadband 1 deg/s); below 200 Pa the full trim attitude
  is held; ~31 kg MMH/NTO per entry. `EntryGuidance(capsule, target)` with `EntryGuidanceParameters::capsule()`
  (`EntryAeroTable::capsule`; no terminal phase; gate 5.5 km [S: NASA Crew-2]; reversals down to 500 m/s with a 0.5 km
  deadband; bank held below 500 m/s; load limit = crew 4 g [E] (`[stage.heat_shield] crew_load_limit_g`,
  `FlightBody::crew_load_warning`) x 0.9). Nominal entry -2.0 deg at 120 km with 30 deg of bank [E]. Splashdown zone
  `config/splashdown_zones.toml` (`pacific_san_diego` 32.6 N 117.8 W [E], radius 2 km - user 2026-10-14). Deorbit:
  `gnc::plan_deorbit` with `DeorbitParameters::burn_thrust_n` and `entry_mass_kg`. `FlightSession::plan_capsule_return`
  (steps `CapsuleReturnStep`): Draco burn along +x (4 side Dracos, 1.6 kN), trunk jettison 120 s after it, nose cone
  closed 120 s later [E], turn to trim, entry guidance `entry_lead_s` before the interface, gate;
  `place_entering_capsule`. A capsule that impacts or breaks up ends as "verloren" (`CapsuleReturnStep::Lost`).
- d4 parachutes and splashdown (`core/tests/test_parachute.cpp`, `test_capsule_splashdown.cpp`): `[[stage.parachute]]`
  sets (`aero/parachute`: drag area grows as t^2 through reefing stages; massless canopy on a riser point on the axis;
  no opening-shock factor, no pendulum body, no wind). Crew Dragon: 2 drogues 28 m^2 each [D: 53 m/s at 1.8 km], reefed
  35 % for 4 s, 4 mains 815 m^2 each [D: 7.0 m/s at sea level], reefed 6 / 20 / 45 % for 4 s each (10.0 t capsule).
  Main limit 120 kN per canopy [E] (2x nominal reefed peak 60 kN); loads judged every tick
  (`Vehicle::record_parachute_load`, "Fallschirm gerissen"). `gnc/parachute_sequencer`: drogues at 5.5 km [S] once
  q <= 15 kPa [E], mains at 1.8 km [S] whatever the drogues did (drogues released), mains released on the water; only
  descending flight deploys. A sequencer per body whose lowest stage has sets; the Dracos stay off once a set is out
  (user 2026-10-14); the session ticks below the highest deployment height + 1 km. Splashdown: `"splashdown"` recovery
  rests on the water at any tilt up to `max_splashdown_speed_mps` 9 m/s [E: spec] (`FlightBody::touchdown_sink_mps`),
  outcome `SplashedDown` "gewassert"; faster is "Aufprall". Steps Gate -> Drogues -> Mains -> SplashedDown
  ("Bremsschirme", "Hauptschirme", "gewassert"); `FlightSession::splashdown_miss_m`, `parachute_sequencer(i)`. The
  water is the EGM2008 geoid (San Diego zone N = -37.0 m): entry target and miss sit on the zone's geoid; sequencer
  heights are above the ground below the capsule. Return mission `config/missions/crew_dragon_return.toml`
  (`sim::place_capsule_return_mission`, 420 km / 51.6 deg, RAAN 60 deg [D]). Webcast telemetry of a Dragon return is
  not used (no free licence).
- d5 game interface: `FlightSim.start_mission` dispatches on `[mission] kind` (`sim::start_mission`: "ascent" default;
  "capsule_return" starts at `mission_start_tdb_s()`; "ship_return" is refused: key R from orbit). `body_rcs(i)`,
  `body_parachutes(i)`, `capsule_state(i)`, `splashdown_zone()`, `set_splashdown_zone(id)`, `plan_capsule_return()`,
  `autopilot_status()["capsule_step"]`. Covered by the smoke test, whose check functions must run to their end.

## Validation
- Falcon 9 Crew Dragon mission `falcon9_crew_dragon` (SLC-40, 51.6 deg, 200 x 210 km, RTLS to LZ-40): SECO T+503 s,
  release T+533 s, 200.0 x 210.8 km, 51.74 deg, first stage landed. Inertial hold after a 1.3 deg/s tumble <= 0.71
  deg, 1.9 kg/h; LVLH trunk forward <= 0.71 deg, 2.4 kg/h; Draco burns 1 / 10 m/s within 0.002 / 0.014 m/s (5.0 / 43.4
  kg), -x 2 m/s within 0.007 m/s.
- Dragon entry `[dragon][entry][validation]` (entry angle +-0.1 deg, mass +-3 %, aero +-10 %): 12 of 12 at the gate
  within 1.2 km, 160-178 m/s, <= 3.6 g. From 420 km / 51.6 deg: 139 m/s / 598 kg after 37 h, gate 0.58 km off at 166
  m/s, 3.5 g. Return mission: burn after 37.1 h, water 0.41 km from the zone centre at 6.9 m/s, deorbit ignition ->
  splashdown 50.5 min (Crew-10 2025-08-09: 54 min [S]); drogues 5.50 km, mains 1.80 km / 53 m/s, peak 2.8 g. Campaign
  `[dragon][mission][validation]` (8 cases): 8 of 8 gewassert within 1.09 km, 6.9-7.0 m/s, <= 2.97 g. Test "A capsule
  return without mains ends lost, ...".

## Diagnostics
- Dragon: hidden tests "Capsule entry single case" (`STARSIM_CAPSULE_SEED=<1-12>`), "Crew Dragon return single case"
  (`STARSIM_DRAGON_CASE=<0-7>`), "Crew Dragon return: the first feasible orbit plane".

## Open points and known limits
- Capsule: Newtonian aero over-predicts flat shields; Dragon deorbit burn 1058 s vs published 7.5-12 min (Draco layout
  is an estimate). Pilot-flown MCC / re-plan display waits for the UI specification (M7).
