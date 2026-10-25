# Lunar missions: reference bodies, HLS, outbound, landing, ascent, return, game interface

History and the reasons behind superseded decisions: `docs/history/guidance-missions.md`.
Index of the guidance / missions topics: `docs/subsystems/guidance-missions.md`.

## Model
Spec `docs/superpowers/archive/specs/2026-10-14-lunar-missions-design.md`.
- l1 reference bodies: vehicle states stay Earth-centred GCRF; `environment::ReferenceBody` (Earth: ITRF93 + WGS84 +
  US76; Moon: MOON_ME, sphere 1737.4 km, vacuum). A `FlightBody` takes the Moon within 60,000 km and the Earth again
  beyond 100,000 km [E]; bodies on the ground keep theirs. `FlightSession` takes a Moon orientation (5th constructor
  argument, `spice::SpiceOrientation::moon_mean_earth()`; `FlightSim` passes it). A Moon-centred `PointMassGravity` is a
  different model (frame accelerates; 0.33 / 4.4 m in a day vs the Earth-centred integration).
- l2 HLS and terrain: `config/vehicles/starship_hls.toml` (120 t dry [E], Block 3 tanks and Raptors, no heat shield /
  flaps / pins, ship RCS; landing thrusters re-sized in l4 to 12 x 90 kN (throttle 35-100 %), canted 15 deg at 40 m,
  `landing_only`: never in burns or `ignite()`). Engine groups take `station_m` / `cant_deg`. Legs [E] fixed, six,
  footprint radius 12 m (user 2026-10-15): slope limit ~9 deg at 12 m stance (NASA's Artemis limit ~8-10 deg); absorber
  1e7 N/m, damping 3e6 N s/m [E], crush 4.5 MN [E]. Terrain: `config/lunar_sites.toml` (default `mons_mouton_im2`,
  IM-2's landing point [S], 5,322 m above the sphere); `fetch.py` crops LOLA `LDEM_80S_20M` to 20 x 20 km per site by
  one HTTP range request and fetches the global `LDEM_16` into `data/terrain/`. `environment::LunarTerrain` (tile
  inside, global grid outside, 1 km smoothstep blend; normals by central differences over 10 m) through
  `ReferenceBody::set_terrain` / `surface_height_m` / `surface_normal_fixed`; `FlightSession::set_moon_terrain`,
  `set_lunar_landing_site` (`sim::load_lunar_terrain`, `sim::lunar_site`). Leg landings work on any reference body
  (`sim::LegSite` with a tracking radius; `set_legs_on_site`) and press the feet along the surface normal
  (`contact::SurfaceField`: `PlaneField`, `TerrainField`). The Earth takes the `environment::Terrain` interface
  (earth-terrain.md). Leg tracks start only near the body's own ground.
- l3 outbound: `mission/position_targeter` (fixed-time position in the full model; needs a patched-conic first guess on
  long arcs; finite burns via `set_finite_burn` / `mission::apply_finite_burn`), `orbit/cr3bp`, `mission/nrho_reference`
  (9:2 southern L2 NRHO [S: Zimovan 2017 seed], multiple shooting in the full model: periods 6.25-6.71 d, perilune
  2,861-3,670 km, apolune 69,804-71,706 km [Gateway: 6.26-6.76 d, rp 3,195-3,557 km, Davis et al. AAS 2020]; use
  `perilune_epochs()`, never `first + k P`), `mission/lunar_outbound` (LLO: TLI 3,142 m/s impulsive / 3,182 m/s finite,
  LOI 856 m/s; NRHO via a powered lunar swing-by (user 2026-10-15) after Kikuchi et al. ISSFD 2024: TLI 3,162 m/s,
  swing-by 243 m/s at 100 km, insertion 224 m/s ~1.3 d later; MCCs, LLO trims, propellant check),
  `mission/lunar_orbit_transfer` (NRHO -> LLO 660 + 216 = 876 m/s; LLO -> NRHO arrives anywhere on the reference (user
  2026-10-15) in one-day windows: 725 m/s right after the LLO arrival; NRHO station keeping at apolune when the
  predicted perilune miss exceeds 1 km [E]). `FlightSession::plan_lunar_outbound` / `plan_lunar_orbit_transfer`,
  `AutopilotState::Lunar` ("Mondflug", `core/src/sim/flight_session_lunar.cpp`), `LunarStep`, `lunar_burns()`. The TLI
  is re-solved for the finite burn; LOI is timed on the periselene predicted at the last coast; burns too small for the
  vacuum Raptors fall back to RCS. Coasts are Sun-pointed with a re-aim every 6 h [E] on their own controller (0.5
  deg/s slews, 0.002 deg/s deadband), ~0.9 t cold gas per day; `gnc::SunHold` ends a hold that has not settled (1 deg,
  0.005 deg/s) after 30 min [E]. Leads: trims and station keeping at least 120 s + 60 s [E] ahead; TLI centre never
  earlier than 30 min [E] after the plan (`FlightSession::min_departure_lead_s`). NRHO insertion and the LLO -> NRHO
  arrival match the reference velocity only. `plan_lunar_outbound` refuses a Moon-referenced vehicle ("Kein Mondflug:
  schon beim Mond"). Pilot-flown MCCs (user 2026-10-15, `pilot_corrections`): call 600 s before the MCC,
  `lunar_pilot_cue()`, `complete_pilot_correction()`. Missions `hls_lunar_llo` / `hls_lunar_nrho`
  (`kind = "lunar_landing"`).
- l4 landing and ascent (plans `docs/superpowers/archive/plans/2026-10-15-lunar-l4-landing-ascent.md`,
  `...-lunar-l4-mass-correction.md`):
  mission 1 (`hls_lunar_llo`) flies LEO -> LLO -> descent -> surface stay -> ascent -> LLO
  (`sim::start_lunar_landing_mission`).
  Masses: HLS ~573 t in the LLO, ~566 t at PDI (~500 t NRHO variant), lands ~315 t. Site pass: the LLO waits for a pass
  of the polar site within `max_crossrange_km` (20 km) and trims the plane ~90 deg before it (user 2026-10-15); the
  wait cannot be shortened by the mission epoch (user 2026-10-16: "refuse honestly"; 5.8 d for Mons Mouton). DOI on the
  vacuum Raptors at minimum (periselene 15 km above the site), trims / DOI / circularisation on the single sea-level
  Raptor (`lunar_small_burn`, transients <= 60 % of the burn [E]). Braking (`gnc/apollo_braking`, Klumpp R-695 quartic
  targets) on the 3 RVacs + RCS attitude (user 2026-10-15: Isp 380 vs 350), then sea_level_1, then the landing
  thrusters (the set drops at max(0.9 x next max, running minimum)); `mission::braking_range_m` (thrust fraction 0.55,
  factor 1.0, floor 150 km: 164 km at 566 t). Approach: SCvx on the landing thrusters from the high gate (700 m / 800 m
  / 25 m/s), "equivalent gimbal" simplification (differential throttling ~20x the RCS torque, equivalent gimbal 9.4
  deg; user 2026-10-15). Terminal descent (Apollo P66 style, `LunarDescentPhase::Terminal`, "Endanflug"): 1 m/s sink,
  throttle >= 0.1 above the minimum and <= 0.95 of the weight floor. Engines off on leg contact. Surface stay: resting
  on the legs for `stay_hours`; no boil-off. Ascent (`gnc/lunar_ascent_guidance`, `gnc/peg_steering`): vertical rise to
  150 m, PEG to 15 km x 100 km on the 3 RVacs (user 2026-10-15), landing thrusters steer by differential throttling
  until the RVac axis is within 2 deg of PEG, acceleration limit 1.5 g [D], circularised at the aposelene. RVac
  engine-out: the other two cut, the 3 sea-level Raptors take over. Aborts (`abort_landing`, automatic when the
  approach cannot be planned): Arrest (climb) then PEG once a fresh solve steers below 0.85 radial or the ballistic
  apex reaches the insertion radius; refused when the propellant cannot reach orbit ("Abbruch nicht möglich").
  Budget (`mission::lunar_budget`, `[budget]` [D]): 120 t LLO closes; 204 t LLO refused (9.3 t short); NRHO variant
  refused at 120 t (2.2 t short).
- l5 return to Earth (plan `docs/superpowers/archive/plans/2026-10-16-lunar-l5-return.md`): mission 2 - a Block 3 ship in the
  LLO or NRHO flies TEI -> MCCs -> braking burn -> direct entry and catch at Starbase (`ship_lunar_return_llo`,
  `_nrho`), or an aerocapture ("Aircatch") into a ~400 km orbit -> perigee raise -> circularisation -> b4
  (`ship_lunar_aircatch_llo`); `sim::start_lunar_return_mission`, `FlightSession::plan_lunar_return`, steps in
  `core/src/sim/flight_session_lunar_return.cpp`. Radiative heating: NASA TM-102652 (Sutton & Hartung 1990)
  equilibrium stagnation tables (`config/aero/radiative_heating_earth.csv`, 624 rows, 30-84 km, 8-18 km/s, nose radius
  0.05-10 m; log-linear in density, linear in speed and radius); Tauber-Sutton cross-check +3 % (60 km, 11 km/s,
  2.3 m). `FlightBody` heat rate = Sutton-Graves convective + radiative (Earth air, from 8 km/s). At 60 km / 5 m: 6.8
  MW/m^2 at 11 km/s, 0.63 at 9.5 km/s - the reason for the braking burn (user 2026-10-16); limit 1.2 MW/m^2 [E].
  Corridor (`gnc/entry_corridor`; relative speeds, east, 120 km): direct (1 deg wide) max 9,688 m/s (-5.53 .. -4.51
  deg), aerocapture to 400 km (0.5 deg) 10,086 m/s; planned entry angle = corridor angle with the widest bank window
  (`gnc::widest_bank_window`); aerocapture entry speed from a bank window >= 10 deg (9.78 km/s). Earth return planner
  (`mission/earth_return`): LLO TEI ~880 m/s (aligned plane, 3.4 d), braking burn on the vacuum Raptors centred at
  ~2,000 km, 1.03-1.15 km/s, entry ~9.7 km/s relative; aerocapture braking ~0.93 km/s. The gate is aimed by moving the
  arrival epoch with a gate window scan +-12 h by continuation (2 h steps halved to 15 min); the plan aims ONE bank
  sign's predicted track. The direct entry reaches Starbase only at epochs where the arrival brings the site within
  reach (Nov 12 2026 works, Nov 2 not); rule (user 2026-10-18): plan as mission planners do - find the departure window,
  no epoch hopping. NRHO: single perilune TEI costs 3.4 km/s -> NRHO departure + 100 km return powered flyby, up to 8
  patched-conic departure options tried cheapest first: Nov 13 2026 575.7 m/s.
  Entry guidance (`EntryGuidanceParameters::lunar_return()` / `aerocapture()`): bank linear in energy (FNPEG principle,
  Lu JGCD 2014; final 40 deg [E], user 2026-10-16); capture preferred to the predicted limits (lowest capturing bank +
  2 deg); q limit 32 kPa [E] (LEO entries keep 25 kPa; user 2026-10-17); no reversal above 8.5 km/s [E] nor while the
  range is missed by > 50 km [E]; lateral funnel (300 km (v / 9.7 km/s)^3 [E], closed at 5 km/s); the bank roll is fed
  forward (`EntryCommand::rate_gcrf_radps`, also roll) and ramped (`bank_step_smooth`, 0.5 deg/s^2 [E]); ~180 deg
  reversals keep their roll direction. Aerocapture: one bank sign, no plane steering, phase Exit; a second pass (up to
  3) when perigee raise + circularisation would cost > 150 m/s [E]. Braking is cut off closed-loop on the
  entry-interface state, then an RCS entry trim (threshold 0.5 m/s); the burn executor honours the thrust dispersion.
  Re-plan from flight data (user 2026-10-18): `FlightSession::replan_lunar_return` (gate window scan from the live coast
  via `EarthReturnPlanner::recovery`), automatic when an MCC does not converge or exceeds its 90 m/s allowance and the
  new plan is cheaper; `lunar_return_replans()`.
- l6 game interface (plan `docs/superpowers/archive/plans/2026-10-18-lunar-l6-game-interface.md`): `FlightSim` runs its session
  on a simulation thread (`sim::SessionRunner`): commands are queued and return a command id (0 when refused before
  posting); `result(id)` -> {done, ok, message} and signal
  `command_finished(id, name, ok, message)` (emitted on the main
  thread from `advance_to` / `wait_idle`). Pilot input goes to a single "latest" slot read before each chunk;
  `advance_to` sets the target epoch, the worker advances in chunks of at most 4,000 ticks and builds a snapshot after
  every command and chunk (getters read that snapshot); the target counts as reached within one tick. `planning_s()`
  gives the wall time of a running plan (the world clock stands meanwhile). CSPICE is serialised by a process-wide
  recursive lock (`detail::lock_spice`). `start_mission` kinds `lunar_landing` / `lunar_return` (all legs planned at the
  start; a refusal's reason is the command's message), `lunar_state()` (step, status, burns, descent plan, touchdown,
  return plan with re-plan count, pilot cue), `lunar_site()`, `moon_terrain(lat, lon)` (calling thread; immutable),
  commands `complete_pilot_correction()` and `replan_lunar_return()`. Telemetry carries `reference_body`,
  `altitude_terrain_m` and osculating periapsis / apoapsis / inclination about the reference body (a polar LLO reads
  ~90 deg). `idle()` also waits for a running input / chunk; an advance that throws stalls the runner until the next
  command, the snapshot is still built, and `FlightSim` reports input / advance / snapshot errors once via `push_error`
  (`SessionRunner::take_error`). `[plan_speed]` checks wall time and runs as one `RUN_SERIAL` CTest test. Plan time:
  whole mission start LLO ~7 s, NRHO ~8 s, aircatch ~4.5 s (`InterpolatedEphemeris` / `InterpolatedOrientation`;
  `solve_lambert` bracket scan 100 samples first, 4,000 only as fallback).

## Standing decisions
- Plan like mission planners: find the departure window, no epoch hopping (user 2026-10-18); re-plan from flight data.
- Lunar plane wait is refused honestly rather than re-targeting (user 2026-10-16).

## Validation
- Lunar propagation `test_lunar_propagation.cpp`: 100 km LLO and a 3,000 x 70,000 km orbit one day Earth-centred vs
  Moon-centred 0.020 / 0.025 m; 200 Hz ticks 20 min 5e-5 m. HLS drop tests (1 m/s + 0.5 m): 0 / 5 / 8 deg land, 25 deg
  tips over; static tip-over 26 deg.
- Lunar outbound: LLO 99.97-100.00 km after one trim; NRHO 0.58 km worst perilune offset over 3 revolutions, NRHO ->
  LLO 99.97-100.03 km; campaign (TLI +-1 % and 0.5 deg, 8/8): MCC 1 183-311 m/s, MCC 2 0.3-0.4 m/s, LLO 99.97-100.01
  km; NRHO worst perilune offset 0.47-2.88 km.
- Lunar landing: Apollo 11 braking validation 508.5 s vs 507 s flown; descent from PDI 566 t 2068 m/s, landed 0.20 m,
  1.04 m/s; with DOI 18 m/s ~2.09 km/s vs Apollo ~2.06 km/s [S: Bennett, NASA TN D-6846]; ascent from 340 t 1955 m/s
  into 100 x 100 km. Campaign (`test_lunar_landing_campaign.cpp`, PDI state +-1 km / +-5 m/s per axis, thrust +-2 % via
  `FlightSession::set_thrust_dispersion`, propellant +-5 %): 21 / 21 landed, 0.02-0.78 m, sink 0.98-1.03 m/s, 237-256
  t from PDI; aborts from 3 / 1 / 0.3 km reach the LLO (148 / 142 / 135 t).
- Lunar return: LLO direct (Nov 12 2026) TEI ~876-888 m/s, braking ~1,030-1,043 m/s, entry 9,695 m/s -4.8 deg, peak
  0.75-1.06 MW/m^2, 2.6-4.2 g, caught (8-13 t left); aircatch braking 926 m/s, exit ~53-55 x ~422-427 km, perigee raise
  43 m/s + circularisation 62 m/s, b4 deorbit 63 m/s ~93 h later, caught 7.6 t; NRHO direct caught 6.1 t. Campaigns
  (`test_lunar_return_campaign.cpp`, seeds 1-21: TEI and braking +-0.2 % / 0.1 deg [E], entry angle +-0.1 deg, hull
  aero +-10 %, density +-15 %, thrust +-2 %): direct 21 / 21 caught (gates <= 0.03 km, 5.9-13.2 t left, peak 0.93-1.16
  MW/m^2, 3.4-4.2 g); aircatch 21 / 21 (exit perigee 41-61 km, apoapsis 413-438 km, b4 gates <= 0.04 km, 0.7-9.6 t
  left). A braking burn short by 500 m/s or not flown ends in "Hitzeschild versagt". Botched TEI: 1-3 % MCC 22-66 m/s,
  5 % re-planned 109 m/s caught 2.2 t, 7 % re-planned 152 m/s caught 0.0 t, 10 % refused honestly. Density x3.0 breaks
  the aircatch ship up, x0.5 exits to ~10,000 km apoapsis (outside the +-15 %). Fixture "An aerocapture pass with 64 t
  aboard damps a roll disturbance at peak q" (`[header_tanks][aerocapture]`): 1.3 deg axis error, apoapsis 420 km,
  peak 904 kW/m2, 60.9 t left.

## Diagnostics
- Lunar: `STARSIM_DEBUG_NRHO`, `STARSIM_DEBUG_OUTBOUND`, `STARSIM_DEBUG_LUNAR_TRANSFER`,
  `STARSIM_DEBUG_LUNAR_SESSION`, `STARSIM_DEBUG_LUNAR_DESCENT`, `STARSIM_DEBUG_LUNAR_ASCENT`; single
  campaign case `"[.lunar_case]"` with `STARSIM_LUNAR_CASE=<seed>`; hidden `"[.lunar_return_case]"`
  (`STARSIM_RETURN_SEED`, `STARSIM_RETURN_AIRCATCH=1`), `"[.lunar_gate_replay]"` (`STARSIM_GATE_STATE`),
  `"[.lunar_handover_replay]"` (`STARSIM_HANDOVER_STATE`), `"[.lunar_return_epoch_probe]"`, `"Aerocapture dispersion
  probe"` (`STARSIM_PROBE_DENSITY`, `_AERO`, `_PROPELLANT_KG`, `_KICK` "h_km,wx,wy,wz"; a peak-q
  attitude repro in ~13 s).

## Open points and known limits
- Lunar: the plan's propellant check uses the impulsive TLI (~40 m/s gravity loss inside the 5 % reserve); no DOI
  re-plan after the trim; no boil-off in the stay; no degraded profile for automatic aborts from fast low states (PEG
  saturates: 111 x 115 km); status at burn end shows osculating elements (95 x 100 km for a 100 km LLO); the sub-orbital
  cutoff guard is untested.
- Heating: equilibrium air only (no non-equilibrium radiation, no ablation coupling); Starship's heat-flux limit and 6 m
  nose radius are estimates (tiles ~1650 C ~0.65 MW/m^2 radiative equilibrium); Tauber-Sutton is outside its range
  (Rn 0.3-3 m).
