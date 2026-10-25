# Orbital refuelling follow-ups - design

Date: 2026-10-21. Status: approved in conversation (sections 1-3), awaiting written-spec review.
Predecessor: `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md` (milestone r, parts r1-r6, complete).

## Goal and understanding

Milestone r answers "how many tanker flights does a lunar mission need": **11 tankers, 156.9 days** (200 km depot,
48 h phasing limit). Three things are left open, chosen by the user after r6:

1. The arrivals recur only every ~13.8 days: with the single fixed 170 x 190 km phasing orbit the tanker can close
   only a narrow band of phase gaps in 48 h. Real mission planners pick the phasing orbit per launch window.
2. The calculator has never been checked against a campaign flown in one continuous world time.
3. Small items: the settling burn before an in-orbit engine ignition (user idea from r2) and the deferred minors of
   the r1-r6 final reviews.

Success: the campaign uses (nearly) every daily Starbase plane crossing and becomes shorter; a fully flown campaign
reproduces the calculator's tanker count and duration; in-orbit ignitions need settled propellant; the deferred
minors are fixed or deliberately documented.

## Parts and order (user decision 2026-10-21)

| Part | Content | Plan |
|------|---------|------|
| f1 | Phasing orbit chosen per launch window (calculator + flown tanker) | own plan |
| f3 | Settling before in-orbit ignition, gas ingestion on unsettled ignition | own plan |
| f4 | Deferred minors of r1-r6 | own plan |
| f2 | Fully flown campaign as the final check of the calculator | own plan |

f2 comes last because f1, f3 and f4 change the measured reference numbers; the flown campaign then checks the final
state once.

Out of scope (user 2026-10-21): launches from Cape Canaveral / several pads (memory note: needs a depot inclination
>= ~28.6 deg; deferred completely, no pad list or port-count levers now). Depot phasing manoeuvres (the target stays
passive). The Moon mission after the HLS refuel in the flown campaign.

## f1 - Phasing orbit per launch window

Approach A (user 2026-10-21): the ascent stays at the calibrated 120 x 190 km insertion; the raise manoeuvre after it
picks the phasing orbit per window from a configured candidate set.

**mission (pure, no vehicle knowledge)**
- `PhasingOrbit { periapsis_radius_m, apoapsis_radius_m }`.
- `LaunchWindowParameters::phasing_orbits` (candidates). Empty = today's behaviour: the separation orbit is the
  phasing orbit (ISS / Crew Dragon path unchanged).
- `PhasingCostModel { mass_kg, isp_s, boil_off_kgps }`: cost of a candidate = propellant for the raise delta-v
  (vis-viva, from the separation orbit) + the phasing schedule's burns (rocket equation at `mass_kg`, `isp_s`) +
  `boil_off_kgps` x phasing time (separation -> Waypoint 0).
- `plan_launch_window` checks every candidate at each plane crossing with `phasing_schedule` (mission limit, drag,
  the plane bias from that candidate's semi-major axis) and takes the cheapest feasible one. The window is the first
  crossing with at least one feasible candidate.
- `LaunchWindow` returns `phasing_orbit` and `phasing_cost_kg`.

**Candidates (config, [E])** in `tanker_flight.toml [phasing]`; `hls_refuel.toml` gets the same set (its window is chosen the same way):
- below the depot: periapsis 150, 160, 170, 180, 190 km with apoapsis 190 km (catch up);
- above the depot: 220 x 240 km and 250 x 270 km (the depot catches up);
- 150 km is the lower bound for 48 h of drag; the planner models drag anyway.

**Flown**
- Raise leg generalised to `raise_to_orbit(periapsis, apoapsis)`: below the depot one burn at apoapsis (as today);
  above it, burn 1 at apoapsis raises the apoapsis, burn 2 at the new apoapsis raises the periapsis. Belly Raptor on
  the deorbit engine set, cold gas as fallback (as today).
- `start_refuelling_flight_mission` takes the window's `phasing_orbit` for the raise and the rendezvous.
- r5 minor folded in: a Raise burn aborted dry ends the mission with `burn_abort_reason` instead of being treated as
  flown.

**Calculator**
- `next_tanker`: delivery = reference delivery - (cost of the chosen orbit - cost of the reference orbit 170 x 190 km
  at the reference phasing 45.2 h). The rule "latest launch reaching the same Waypoint 0" stays.
- The candidate set is a lever (`CampaignLevers`, CLI `--phasing-orbits ref|default`, game parameter); `ref` gives the
  r6 behaviour for comparison.

**Tests**
- Unit: a gap that 170 x 190 km cannot close in 48 h picks a lower orbit; a negative gap picks an orbit above the
  depot; empty candidates reproduce the old window; the phase rate against dn = -1.5 n da/a; cost model against a
  hand computation.
- `[slow]`: a tanker flown from the pad in a window that needs an orbit above the depot, up to docking.
- Re-measure `reference_flights.toml` if the reference flight changes; new campaign answer documented (expected: more
  arrivals, clearly fewer than 157 days).

## f3 - Settling before an in-orbit ignition

**vehicle**
- Per-stage settling state. Propellant counts as settled once a thrust acceleration along +x (towards the engines)
  of at least 0.01 m/s^2 has acted for `settling_time_s(L, a)` = sqrt(2L/a), L = travel of the liquid in the feed
  tanks (tank length minus fill height) [D].
- It stays settled while the Bond number of the feed tanks is >= 10 (r4 `outlet_check`). Below that (coast without
  thrust) the liquid leaves the outlets again after a time of the same order, sqrt(2L/a) with the residual
  acceleration (tide, drag) [E].
- Tanks flagged `propellant_management_device = true` [E] are always covered: Dragon Draco / SuperDraco tanks
  (diaphragm tanks) and the Starship header tanks (user 2026-10-21; SpaceX details not public, deliberate [E] switch).

**sim**
- In the Burn autopilot at ignition the feed tanks' outlet coverage is checked. Not covered: a settling phase starts
  ahead of the ignition epoch - cold gas +x with the vehicle's translation capability (tanker ~0.010 m/s^2 -> ~30-60 s),
  shown as a step ("Settling"), then ignition. Its delta-v (<< 1 m/s) is absorbed by the closed-loop guidance.
- Gas is drawn from the RCS supply; the planners (phasing, raise, deorbit, TLI and the lunar burns) reserve it.
- Binding command `settle(bool)` for the expert / manual mode (UI by the user).
- Unsettled ignition (gas empty, manual ignition without settling): the lit engines ingest gas and fail
  deterministically (`fail_engine`), `burn_abort_reason` "Gasansaugung"; the mission aborts.
- Re-measure the r5 reference flights and `reference_flights.toml`; campaign answer updated.

**Tests:** settling time and loss against hand values; PMD tanks always covered; unsettled ignition fails the engines;
the autopilot settles before a phasing burn and the burn flies; gas reservation in the planners.

## f4 - Deferred minors

Every item is fixed or deliberately documented (with the reason) in `docs/subsystems.md`.

- r1: hull sampling cuts at the first port x (check r3 state); mating test nearest drogue instead of one-to-one, no
  negative layout test; `set_tank_mass` accepts NaN; no duplicate-id check for ports/connectors, two from stages with
  the same id overwrite; `vehicle_report.py` from-resolution weaker than C++; stale Block 3 comments in the depot TOML,
  depot header 35 t not transferable (document); tanker comment `z = +4.5 m`; bindings `station_geometry` /
  `iss_state` without mechanism / `elements_m`; `load_stage_from` reloads the source per from stage.
- r2: resting/docked bodies still boil in `step()`; abort reason for failed engines; eclipse time-warp test;
  `boil_off_last_` not reset after holds; `DeorbitParameters::boil_off_kgps` unused; coast evaluation cost for
  non-thermal vehicles; local AU / solar pressure duplicates in `flight_body`; mixture ratio from the last engine;
  `plan.boil_off_kg` not refreshed on re-plan.
- r3: M1 rendezvous model at full throttle vs min-throttle burns; M3 proximity warning text "Bahnanhebung der ISS";
  M4 station limits override caller parameters; M5 wall -> plate force jump.
- r4: settling impulse in full when the gas draw falls short (scale by drawn/gas); pressure difference per transfer,
  not per line; a transfer can start while the hooks open for undocking; re-chill after a pause vents nothing more.
- r5: hull collision during Transfer reports "Transfer gestoppt" instead of "Kollision"; continuation with a new
  sequence overwritten by Complete (generation counter); `set_iss_mission` with transfer && docked starts at once / an
  active docking autopilot ends the mission; `[mission] port` parsed but unused; fused comments on `iss_start_raise` /
  `iss_start_transfer`; `nose_cone_body_` not shifted in `erase_body`; CW-drift departure target as an ISS behaviour
  change in subsystems.md; ascent tail-off gate on the controlled body only (document). (The dry Raise abort is in f1.)
- r6: HLS tank check incl. the wait; HLS altitude cost; wait boil-off Sun at t = 0; vent as 1/(1-f); comment tag; no
  cheap file-drift guard for `reference_flights.toml`; FlightSim worker blocked ~1 s by the campaign command.

## f2 - Fully flown campaign

**One session, continuous world time**
- `depot_launch` -> `make_station`; the depot then flies in the full model to the end (drag, heat-balance boil-off,
  Sun / eclipse).
- `start_ascent_mission`, `launch_from_pad`, `place_*` get a `keep_station` option: all other bodies are removed, the
  station stays. A tanker is removed after its departure; only the first tanker also flies its ship return (and the
  booster catch of every tanker is flown as part of its ascent).

**Driver `sim::RefuellingCampaignFlight`** - plans live from the flown depot state, like a mission planner:
next window with the f1 logic (orbit choice, latest launch to the arrival) -> time warp to the launch -> fly the
tanker (ascent, booster catch, raise, phasing, docking, transfer, departure). When the depot fill covers the HLS demand
(the calculator's demand), the HLS window is planned; the HLS launches, docks and is refuelled; end. If the fill drops
below the demand before the HLS arrival, another tanker flies (same rule as the calculator).

**Depot station keeping, flown**
- Mean altitude below 195 km [E] -> a Raptor Hohmann pair back to 200 km, with settling (f3), only while no tanker is
  in flight and the next arrival is more than 1 day away [E]. The depot gets control for the burns
  (`execute_burn`) and is a station again afterwards.
- Attitude gas 20 kg/day [E] drawn as in the calculator (fair comparison).

**Comparison and report** - one row per tanker: window, chosen orbit, phasing time, delivery, depot fill at arrival
(calculator vs flown), plus the loss items (boil-off, station keeping, gas, vent). Pass: same tanker count, days within
+-1 day, depot fill per arrival within 2 % [E]. Deviations caused by the calculator are corrected there; the cause is
documented in subsystems.md.

**Run time and tests** - estimate ~11 x 150 s tanker flights plus ~157 days of depot coast under time warp, roughly
30-45 min: own tag `[campaign_flown][slow]`, run in the background with output to a file and a check every 10 min.
Smaller tests: `keep_station`, a 2-tanker campaign started in orbit, the station-keeping trigger and flight.

**Game** - mission config `refuelling_campaign_flown`, startable via FlightSim like the other missions; getter
`campaign_flight_state()` (tanker n of m, depot fill, next window, comparison rows). Autopilot only, UI by the user.

## Documentation

`docs/subsystems.md` refuelling sections updated per part; CLAUDE.md module summary when interfaces change;
`reference_flights.toml` re-measured where flights change.
