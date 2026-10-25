# Vehicle data, ascent and booster recovery - history

Verbatim record of this subsystem's notes as they stood in `docs/subsystems.md` on 2026-10-25, before the split. Current state: `docs/subsystems/vehicle-data.md`. Kept for the reasons behind decisions, superseded numbers and earlier approaches; read only when that history is needed.

- Only the current configuration (Block 3 / V3, Raptor 3) is modelled (decision 2026-10-13); its data come from
  the flights that flew it (12-14, `tests/validation/telemetry/flights.toml`). Earlier blocks were removed.
- The booster's ascent throttle is vehicle data since ascent Task 2 (user 2026-10-23): `[stage] max_ascent_throttle`
  0.88 [D] (the flights 13/14 propellant gauge's flow before max-Q against the rated flow) and the max-Q limit
  `max_dynamic_pressure_pa` 38909 Pa [D] (flight 14's peak q); the speed-fitted time schedule of the missions is gone
  (section "Ascent trajectory and staging"). Every launch plans its kick (ascent Task 3-5). Validation (ascent Task 6,
  test "Starship ascent to flight 14's insertion orbit against its webcast telemetry"): on the dials' own time base
  (speed 3.50 s, altitude 5.45 s late) and with flight 14's booster load (96.8 %) the planned ascent follows flight 14
  within the readings to max-Q; from there flight 14 throttles below 0.88 and loses an engine: +7-9 % fast at T+90-120 s
  (open, section "Ascent trajectory and staging"). Staging mass vs the gauge (decision 2026-10-14: physics first, the gauge after staging is not
  followed): on this ascent our return plans MECO with ~310 t (301 t at the boostback start, ~2.2 km/s on 13 engines),
  flight 14's gauge shows 11.8 % (431 t) there. The gauge is not consistent with the engines after staging: it falls
  ~15 t/s on 13 engines (full mass flow ~10.5 t/s) and 3.5 -> 1.4 % (77 t) in the last ~12 s on 3 engines (~30 t);
  Falcon 9's falls 21 t in a coast without a burn. The timeline matches (test "Block 3 plans MECO ..."): boostback
  T+152-176 s (flight 14 ~152-187 s), apogee 91.8 km at T+236 s (98.9 km, T+240 s), landing-burn ignition T+390 s
  (~T+398 s) at 287 m/s / 2.2 km (flights 13/14: ~355 m/s at ~2.9 km; subsonic drag is not measured). The earlier
  "~207 t, boostback ~1.5 km/s" came from the default full-thrust ascent (MECO at ~97 km climbing at 1.7 km/s).
- **Scheduled liftoff and pad hold** (external view, Task 4): `FlightSession::schedule_liftoff(t0)` gives the stack on
  the pad a T-0. The ascent autopilot (`PreLaunch`) then holds: the engines start `liftoff_ignition_lead_s()` before
  T-0 (longest first-stage `startup_s` + `liftoff_hold_margin_s` 0.5 s [E]; Raptor 3: 3.5 s) and the clamps open at
  the first tick with all engines running at or after T-0 (test: engine start T-3.49 s, liftoff at T-0 within a tick).
  While waiting the clamped stack coasts (`FlightBody::coast` keeps a clamped body on its pad, co-rotating) until
  `burn_preparation_s` before the ignition, so hours on the pad cost a few coast segments. Without a schedule the
  liftoff follows the engine start at once (unchanged). `sim::StartOptions{wait_on_pad, countdown_s}` of
  `start_mission`: launch missions (ascent, iss_launch, depot_launch, tanker_flight / hls_refuel from the pad) stand on
  the pad from `requested` and lift off at the window's liftoff (depot_launch: its epoch; ascent: `requested` +
  `[mission] countdown_s`, else the option's `countdown_s`); their stations are placed at the session epoch. T-0 is
  never less than the ignition lead after the placement (fix round 1: a T-0 the engines cannot meet made the clock jump
  back at the real liftoff): a window's liftoff or a configured epoch is kept and the vehicle placed the lead before it,
  a countdown T-0 moves to placement + lead. iss_launch waits only where the ISS comes from the same source as at the
  liftoff (the OEM file the window used, else the synthetic orbit): the placement moves up to that file's coverage.
  Disengaging the autopilot during the hold drops the schedule (no T-0, no predicted liftoff). The default
  (`wait_on_pad = false`) places the vehicle at the liftoff as before.
- Circular ascent targets are J2-aware (`gnc/j2_insertion`): when the gravity model has J2
  (`GravityModel::central_j2`), PEG aims at the radius, radial and horizontal speed at the predicted cutoff point whose
  next revolution stays closest to the target radius (Gauss-Newton on a point-mass + J2 revolution, re-solved when the
  cutoff point moves > 2 deg). A Kepler-circular insertion swung 5-20 km in radius (Starship 26.5 deg: 190.8-200.0 km);
  now the flown revolution keeps 199.7-201.6 km - the J2 short-period swing left is 0.7 / 2.1 / 3.3 km at 26.5 / 51.6 /
  97 deg. Circular-orbit tests judge the flown revolution; elliptical targets (Starlink 200 x 280 km) are still judged
  on osculating elements at cutoff (Kepler). Burn planning helpers are two-body.
- Booster recovery (decision 2026-10-13): Super Heavy is always caught (no legs; `[stage.recovery] method =
  "tower_catch"`): return guidance only at sites with a tower. Falcon 9 boosters land on the pad or the drone ship
  ("leg_landing", milestone c: `docs/superpowers/archive/specs/2026-10-13-falcon9-landing-design.md`;
  vehicle, legs, the RTLS to LZ-40 (c3), the drone ship (c4) and the game interface (c5) done; the drawing and UI wait
  for the user's UI specification). If the first SCvx catch plan fails, an SCvx sea abort is planned at once;
  the explicit laws are the last fallback (over the arms, ~1-3 m: not accurate enough to be caught); with
  `scvx_landing = false` the whole return aims beyond the shoreline and ditches (test: 1.05 m/s, 650 m off).
- Booster return: at a catch site the landing burn flies SCvx plans (`gnc/scvx_landing`), V3 sequence 13 -> 5 (divert pair) -> 3 engines,
  re-planned every 1 s on a worker thread (effective at snapshot + 0.5 s: a re-plan starts from the snapshot
  predicted over that latency with the active plan's thrust, its start thrust within the throttle-rate reach of the
  measured thrust; tests and time warp wait, real-time play never waits - `FlightSession::set_realtime_planning`).
  The approach into the arms is slow (sink <= 1 m/s + height / 3 s, soft). From an ignition state (scenario:
  355 m/s at 2.88 km as on flights 13/14 [D]; the ignition profile flies 13 engines at 35 %) the dispersion campaign
  `[gnc][catch][validation]` (+-10 m, +-3 m/s, +-5 % mass, +-3 deg, engine failure) catches 21 of 21 (< 0.1 m, < 0.05 m/s lateral, 0.95-0.97 m/s sink at pin contact). The landing
  throttle accounts for the nozzle pressure loss (thrust = throttle x F_vac - p A_e; a throttle proportional to sea
  level thrust gave ~10 % too little thrust at low throttle). Lateral tracking is an LQR on the
  gimbaled-rocket dynamics (`gnc/tvc_lateral`); plans use <= 30 % of the gimbal limit, the LQR effort is 0.2 x the
  gimbal authority. The coast phase already rolls the pins towards the arms. Planning model: constant CG/inertia,
  the hull normal force (Task 8, below "Mid-burn clipping"), no grid fins, no wind; no re-plans in the last 3 s. The explicit laws remain the fallback.
  Switch windows [E]: 13 -> 5 at 180-220 m/s sink (5 engines seen at ~200 m/s on flights 13/14), 5 -> 3 calm at
  15-30 m/s (satnews 2025-10-05: V3 ends in a hover on 3 engines).
  Full return: caught, 51.0 t at landing-burn ignition (planned 51 t, flight 14 gauge; the MECO plan budgets 1.7 t [D] of
  main-tank RCS gas, `[stage.recovery] rcs_propellant_kg`, since d1; the catch and MECO tests fly the flight-like
  ascent `starship_leo`: 52.6 t); it arrives slower than flights 13/14 (287 m/s at ignition, ~2.2 km). Abort plans must end beyond the shoreline (`DescentTarget::sea_min_distance_m`
  = shoreline + `sea_clearance_m` 30 m [E], a hard constraint) and pull towards the sea target (soft); only when the
  water is out of reach do the plans fall back to the soft pull (before: the soft pull alone, 100 kg per metre, set the
  booster down beside the tower). `FlightSession::abort_landing` / `FlightSim.abort_landing` is the pilot's abort
  (refused - false, arms untouched - without a sea, once down or with the pins on the arms; on the explicit laws the
  descent re-aims beyond the shoreline; before the landing burn it is remembered for it); test
  "A commanded abort late in the landing burn ditches the booster at sea" (1500 / 800 / 400 m: water, 25.7 / 14.6 /
  5.7 t left).
- Descent drag (tail first): body table `aero/super_heavy_descent.csv` [D] plus the grid fins' axial drag
  (`aero/grid_fin.csv` column `ca_axial` [E], 3 x 13.5 m^2). Identified from our webcast telemetry of flights 13 and
  14 (`tests/validation/telemetry`, `tools/telemetry/webcast_telemetry.py` + `drag_estimate.py`; below ~9 km the
  speed dial gives way to the engine icon, speed then from the 0.1 km altitude of the nearly vertical fall): the
  unguided booster (210 t dry + 51 t) follows both descents from ~27 km within 0.6 / 2.6 % rms and loses altitude to
  the landing burn within 415 / 158 m (test "Booster descent drag reproduces the webcast telemetry of flights 13
  and 14"). The webcast's speed and altitude graphics are offset by ~1.5-2 s. Subsonic drag is not measured.
- Block 3 masses [D] from flights 12-14 (`tools/telemetry/mass_estimate.py`, `ship_dv.py`): ship dry 255 t
  (rocket equation staging -> cutoff with the propellant gauge, 248-260 t), booster dry 210 t +-40 t (stack mass
  balance and descent ballistic coefficient), liftoff ~5,550 t; tank capacities 3,650 / 1,600 t [S]; Raptor 3
  280 tf consistent (peaks ~270 tf sea-level equivalent; 250 tf is contradicted), minimum throttle 30 % [D] (V3
  hovers on 3 engines). MECO of a returning booster is planned by the ascent autopilot from the state
  (`BoosterReturnGuidance::meco_propellant_kg`: run-down + flip + boostback by the rocket equation on the Lambert
  delta-v after the flip, +8.6 % [D] for the drag-shifted aim point and finite burn) so that `landing_propellant_kg`
  = 51 t [D] (flight 14 gauge) remain at landing-burn ignition (test: 51.0 t with the RCS gas budget; 30 / 70 t targets within 0.4 t);
  on the flight-like ascent it stages with ~310 t. `reserve_propellant_kg` (225 t) is the MECO without return guidance.
  The flights' gauge shows ~500 t (~13.5 %) at MECO and 1.4 % (51 t) after the boostback (video T+188-195 s; the
  cleaned table drops these readings) - more than the engines can burn in between: the gauge is not followed after
  staging (see the Starship throttle note).
- Catch tower dimensions and azimuths (`[site.tower]`, Starbase 315 deg, LC-39A 0 deg) and the catch pins
  (`[stage.catch_pins]`, 5 MN per pin) are estimates [E]; verify the tower azimuths against imagery.


