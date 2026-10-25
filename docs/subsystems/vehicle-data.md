# Vehicle data, ascent and booster recovery

History and the reasons behind superseded decisions: `docs/history/vehicle-data.md`.
Ascent trajectory and staging: `docs/subsystems/ascent-staging.md`.

## Model
- Only the current configuration (Block 3 / V3, Raptor 3) is modelled; data come from flights 12-14
  (`tests/validation/telemetry/flights.toml`).
- **Block 3 masses** [D] from flights 12-14 (`tools/telemetry/mass_estimate.py`, `ship_dv.py`): ship dry 255 t (rocket
  equation staging -> cutoff with the propellant gauge, 248-260 t), booster dry 210 t +-40 t (stack mass balance and
  descent ballistic coefficient), liftoff ~5,550 t; tank capacities 3,650 / 1,600 t [S]; Raptor 3 280 tf consistent
  (peaks ~270 tf sea-level equivalent; 250 tf is contradicted), minimum throttle 30 % [D] (V3 hovers on 3 engines).
- **Booster ascent throttle** (vehicle data): `[stage] max_ascent_throttle` 0.88 [D] (flights 13/14 gauge flow before
  max-Q against the rated flow) and `max_dynamic_pressure_pa` 38909 Pa [D] (flight 14's peak q). Every launch plans its
  kick. On the dials' own time base (speed 3.50 s, altitude 5.45 s late) and with flight 14's booster load (96.8 %)
  the planned ascent follows flight 14 within the readings to max-Q; afterwards flight 14 throttles below 0.88 and
  loses an engine: +7-9 % fast at T+90-120 s (open, see ascent-staging.md).
- **MECO of a returning booster** is planned by the ascent autopilot from the state
  (`BoosterReturnGuidance::meco_propellant_kg`: run-down + flip + boostback by the rocket equation on the Lambert
  delta-v after the flip, +8.6 % [D] for the drag-shifted aim point and finite burn) so that `landing_propellant_kg`
  = 51 t [D] (flight 14 gauge) remain at landing-burn ignition (test: 51.0 t with the RCS gas budget; 30 / 70 t
  targets within 0.4 t); the MECO plan budgets 1.7 t [D] of main-tank RCS gas (`[stage.recovery] rcs_propellant_kg`).
  On the flight-like ascent it stages with ~310 t (301 t at the boostback start, ~2.2 km/s on 13 engines).
  `reserve_propellant_kg` (225 t) is the MECO without return guidance. Timeline (test "Block 3 plans MECO ..."):
  boostback T+152-176 s (flight 14 ~152-187 s), apogee 91.8 km at T+236 s (98.9 km, T+240 s), landing-burn ignition
  T+390 s (~T+398 s) at 287 m/s / 2.2 km (flights 13/14: ~355 m/s at ~2.9 km; subsonic drag is not measured).
- **Scheduled liftoff and pad hold:** `FlightSession::schedule_liftoff(t0)` gives the stack on the pad a T-0. The
  ascent autopilot (`PreLaunch`) then holds: the engines start `liftoff_ignition_lead_s()` before T-0 (longest
  first-stage `startup_s` + `liftoff_hold_margin_s` 0.5 s [E]; Raptor 3: 3.5 s) and the clamps open at the first tick
  with all engines running at or after T-0. While waiting the clamped stack coasts (`FlightBody::coast` keeps a
  clamped body on its pad, co-rotating) until `burn_preparation_s` before the ignition. Without a schedule the liftoff
  follows the engine start at once. `sim::StartOptions{wait_on_pad, countdown_s}` of `start_mission`: launch missions
  (ascent, iss_launch, depot_launch, tanker_flight / hls_refuel from the pad) stand on the pad from `requested` and
  lift off at the window's liftoff (depot_launch: its epoch; ascent: `requested` + `[mission] countdown_s`, else the
  option's `countdown_s`); their stations are placed at the session epoch. T-0 is never less than the ignition lead
  after the placement: a window's liftoff or a configured epoch is kept and the vehicle placed the lead before it, a
  countdown T-0 moves to placement + lead. iss_launch waits only where the ISS comes from the same source as at the
  liftoff (the OEM file the window used, else the synthetic orbit): the placement moves up to that file's coverage.
  Disengaging the autopilot during the hold drops the schedule. Default (`wait_on_pad = false`) places the vehicle at
  the liftoff.
- **J2-aware circular ascent targets** (`gnc/j2_insertion`): when the gravity model has J2
  (`GravityModel::central_j2`), PEG aims at the radius, radial and horizontal speed at the predicted cutoff point whose
  next revolution stays closest to the target radius (Gauss-Newton on a point-mass + J2 revolution, re-solved when the
  cutoff point moves > 2 deg). The flown revolution keeps 199.7-201.6 km; the J2 short-period swing left is
  0.7 / 2.1 / 3.3 km at 26.5 / 51.6 / 97 deg. Circular-orbit tests judge the flown revolution; elliptical targets
  (Starlink 200 x 280 km) are judged on osculating elements at cutoff (Kepler). Burn planning helpers are two-body.
- **Booster recovery:** Super Heavy is always caught (no legs; `[stage.recovery] method = "tower_catch"`): return
  guidance only at sites with a tower. Falcon 9 boosters land on the pad or the drone ship ("leg_landing", milestone
  c: `docs/superpowers/archive/specs/2026-10-13-falcon9-landing-design.md`; vehicle, legs, RTLS to LZ-40, drone ship and game
  interface done; drawing and UI wait for the user's UI specification). If the first SCvx catch plan fails, an SCvx
  sea abort is planned at once; the explicit laws are the last fallback (over the arms, ~1-3 m: not accurate enough to
  be caught); with `scvx_landing = false` the whole return aims beyond the shoreline and ditches (test: 1.05 m/s,
  650 m off).
- **Booster return landing burn:** at a catch site SCvx plans (`gnc/scvx_landing`), V3 sequence 13 -> 5 (divert pair)
  -> 3 engines, re-planned every 1 s on a worker thread (effective at snapshot + 0.5 s: a re-plan starts from the
  snapshot predicted over that latency with the active plan's thrust, its start thrust within the throttle-rate reach
  of the measured thrust; tests and time warp wait, real-time play never waits -
  `FlightSession::set_realtime_planning`). The approach into the arms is slow (sink <= 1 m/s + height / 3 s, soft).
  The landing throttle accounts for the nozzle pressure loss (thrust = throttle x F_vac - p A_e). Lateral tracking is
  an LQR on the gimbaled-rocket dynamics (`gnc/tvc_lateral`); plans use <= 30 % of the gimbal limit, the LQR effort is
  0.2 x the gimbal authority. The coast phase already rolls the pins towards the arms. Planning model: constant
  CG/inertia, the hull normal force, no grid fins, no wind; no re-plans in the last 3 s. Switch windows [E]:
  13 -> 5 at 180-220 m/s sink, 5 -> 3 calm at 15-30 m/s (satnews 2025-10-05: V3 ends in a hover on 3 engines).
  Full return: caught, 51.0 t at landing-burn ignition (planned 51 t, flight 14 gauge; the catch and MECO tests fly
  the flight-like ascent `starship_leo`: 52.6 t); it arrives slower than flights 13/14 (287 m/s at ignition,
  ~2.2 km).
- **Abort:** plans must end beyond the shoreline (`DescentTarget::sea_min_distance_m` = shoreline +
  `sea_clearance_m` 30 m [E], a hard constraint) and pull towards the sea target (soft); only when the water is out of
  reach do the plans fall back to the soft pull. `FlightSession::abort_landing` / `FlightSim.abort_landing` is the
  pilot's abort (refused - false, arms untouched - without a sea, once down or with the pins on the arms; on the
  explicit laws the descent re-aims beyond the shoreline; before the landing burn it is remembered for it).
- **Descent drag (tail first):** body table `aero/super_heavy_descent.csv` [D] plus the grid fins' axial drag
  (`aero/grid_fin.csv` column `ca_axial` [E], 3 x 13.5 m^2). Identified from our webcast telemetry of flights 13 and
  14 (`tests/validation/telemetry`, `tools/telemetry/webcast_telemetry.py` + `drag_estimate.py`; below ~9 km the speed
  dial gives way to the engine icon, speed then from the 0.1 km altitude of the nearly vertical fall). The webcast's
  speed and altitude graphics are offset by ~1.5-2 s. Subsonic drag is not measured.
- Catch tower dimensions and azimuths (`[site.tower]`, Starbase 315 deg, LC-39A 0 deg) and the catch pins
  (`[stage.catch_pins]`, 5 MN per pin) are estimates [E].

## Code
- `config/vehicles` Block 3 data (`starship_block3.toml`), `tools/telemetry/*`, `gnc/j2_insertion`,
  `gnc/scvx_landing`, `gnc/tvc_lateral`, `BoosterReturnGuidance`, `FlightSession::schedule_liftoff`,
  `abort_landing`, `set_realtime_planning`, `sim::StartOptions`.

## Standing decisions
- Only Block 3 / V3 is modelled (2026-10-13).
- Super Heavy is always caught (2026-10-13); Falcon 9 lands on legs.
- Physics first (user 2026-10-14): the propellant gauge after staging is not followed. The gauge is inconsistent with
  the engines: it shows ~500 t (~13.5 %) at MECO and 1.4 % (51 t) after the boostback, falls ~15 t/s on 13 engines
  (full mass flow ~10.5 t/s), 3.5 -> 1.4 % (77 t) in the last ~12 s on 3 engines (~30 t); Falcon 9's falls 21 t in a
  coast without a burn; flight 14's gauge shows 11.8 % (431 t) at the boostback start.
- Ascent throttle / max-Q limit are vehicle data (user 2026-10-23).

## Validation
- Test "Starship ascent to flight 14's insertion orbit against its webcast telemetry" (see Model).
- Test "Booster descent drag reproduces the webcast telemetry of flights 13 and 14": the unguided booster (210 t dry +
  51 t) follows both descents from ~27 km within 0.6 / 2.6 % rms and loses altitude to the landing burn within
  415 / 158 m.
- `[gnc][catch][validation]` dispersion campaign from an ignition state (355 m/s at 2.88 km as on flights 13/14 [D];
  the ignition profile flies 13 engines at 35 %): +-10 m, +-3 m/s, +-5 % mass, +-3 deg, engine failure: catches 21 of
  21 (< 0.1 m, < 0.05 m/s lateral, 0.95-0.97 m/s sink at pin contact).
- Test "A commanded abort late in the landing burn ditches the booster at sea" (1500 / 800 / 400 m: water, 25.7 /
  14.6 / 5.7 t left).
- Scheduled liftoff test: engine start T-3.49 s, liftoff at T-0 within a tick.

## Open points and known limits
- Flight 14 vs the planned ascent after max-Q: +7-9 % fast at T+90-120 s (see ascent-staging.md).
- Subsonic descent drag is not measured; tower dimensions and azimuths are estimates [E] (verify against imagery).
