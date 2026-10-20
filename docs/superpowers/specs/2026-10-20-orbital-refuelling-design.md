# Orbital refuelling (tanker flights) - design

Status: approved 2026-10-20. r1-r6 implemented (milestone complete; r6: `...-r6-campaign.md`) (plans `docs/superpowers/plans/2026-10-20-refuelling-r1-vehicles.md`,
`...-r2-boil-off.md`, `...-r3-rendezvous-docking.md`, `...-r4-propellant-transfer.md`, `...-r5-reference-flights.md`;
notes in `docs/subsystems.md` "Orbital refuelling", corrections below). Milestone (r), the rest of plan milestone M9
(tanker flights), after the ISS milestone (i1-i6, main `ffbb574`).

## Intent

The user's request (plan M9, 2026-10-12; chosen as the next milestone 2026-10-20): tanker Starships launch, meet the
target ship in orbit (rendezvous, docking) and transfer propellant (ship-to-ship transfer, boil-off). The question the
simulation answers: **how many tanker flights a lunar mission needs.** Until now the lunar missions start in LEO with
the configured full 1,600 t ("after the depot", `config/missions/hls_lunar_*.toml`) - a refuelled ship is assumed,
not modelled.

Decisions taken with the user (brainstorm 2026-10-20; memory `project-orbital-refuelling`):
- **Hybrid answer:** one representative tanker flight is flown in the full model (pad -> catch) and measured; a
  campaign calculator turns its numbers into the flight count for a concrete lunar mission. A fully flown campaign
  (every tanker in one continuous world time) is a later check on the same foundation, not part of this milestone.
- **Depot + HLS:** a depot ship launches first, the tankers fill it, then the HLS launches, docks at the depot and
  takes one transfer (NASA/SpaceX concept).
- **Boil-off by heat balance:** sun with eclipse, Earth albedo and Earth IR on the hull, an effective wall heat leak
  [E], LOX and CH4 evaporated separately by their latent heat. No tank pressure / temperature / stratification model.
- **Back-to-back docking:** the ships lie parallel, dorsal side to dorsal side; four probes on the tanker (and the HLS)
  engage four drogues on the depot; LOX and CH4 transfer connectors at tank height. Flow driven by a pressure
  difference; a small settling thrust of the cold-gas RCS keeps the liquid at the outlets; masses and CGs of both
  ships change live.
- **Tanker reserve:** the header tanks stay full (35 t) for flip and landing, as validated; the main tanks are
  delivered down to a computed residue (deorbit burn + RCS gas for the transfer, departure and return + unusable
  residue). The tanker returns and is caught.
- **Own depot vehicle:** a Starship derivative without heat shield and flaps (it never returns), with better
  insulation [E] and four dorsal drogues. Tanker = Block 3 ship + four probes; the HLS gets the same probes.
- **Depot altitude is a calculator parameter:** the reference flight goes to the lunar missions' start orbit
  (200 x 200 km, 26.5 deg from Starbase); other altitudes are scaled by the Hohmann cost and pay the drag reboost.
- **Game scope like the ISS milestone:** autopilot only (manual flying with M7); missions startable through
  `FlightSim`, getters for the transfer and depot state, the campaign as a command; the UI is built by the user. Plus
  a command-line campaign report.
- **Architecture A:** generalise the ISS stack (station body, phasing planner, proximity guidance, contact framework,
  compound) instead of a separate Starship stack.

Research (2026-10-20): SpaceX's Starship V3 update (May 2026) lists "four docking drogues and dedicated propellant
transfer connections added to support in-space refueling architecture" (Teslarati). Secondary sources place the
ports on the dorsal (leeward) side; SpaceX publishes no positions. Transfer by a pressure differential between the
vehicles (Ars Technica 2024, via Wikipedia "Starship Propellant Transfer Demonstration"); flight 3 (March 2024)
demonstrated an intra-ship LOX transfer with settling. The ship-to-ship demonstration has not flown (NET late 2026).
Public tanker counts range from 4 to the high teens ("10-ish", SpaceX 2024). No boil-off figures are published.

## Decomposition

| Part | Content |
|---|---|
| r1 Vehicles | depot and tanker configs, probes / drogues / transfer connectors, runtime tank mass setter, Starship translation capability |
| r2 Boil-off | heat balance on the hull -> kg/day LOX and CH4, flight application, orbit-averaged rate, validation |
| r3 Starship rendezvous | generalised target body, Starship phasing (Raptor + RCS), proximity limits from the target, probe/drogue contact, compound with variable attached mass |
| r4 Transfer | pressure-driven flow, settling thrust, live masses and CGs, end conditions, losses |
| r5 Reference flights | `depot_launch`, `tanker_flight` (pad -> catch, plus an orbit start), `hls_refuel` -> lunar mission with the real tank state |
| r6 Campaign + game | campaign calculator, measured-numbers file, `refuelling_report` CLI, FlightSim interface, docs |

Each part gets its own plan, execution and final review, as in milestone (i).

## What exists and what changes

- **Tanks:** `TankConfig` / `vehicle::Tank` per tank (LOX / CH4, main / header, `capacity_kg`); `config::fill_tanks`
  fills before the vehicle is built (headers first). New: a runtime setter `Vehicle::set_tank_mass` (clamped to
  capacity) for transfer, boil-off and the test mode.
- **Station body:** `FlightSession::add_station` requires `[stage.station]`, one station per session, ideal LVLH
  attitude with the torque dropped. Generalised to a target block shared by ISS and depot.
- **Phasing:** `plan_rendezvous` refuses every vehicle with main-engine burn configurations ("only Crew Dragon
  docks") and uses Dragon's free-molecular drag coefficient. Generalised to Starship.
- **Proximity:** `gnc::ProximityGuidance` uses generic CW/LVLH maths with ISS parameters (keep-out sphere 200 m,
  fly-around 250 m, approach ellipsoid). The limits move into the target's configuration.
- **Contact:** the IDSS interface is tuned for a reduced mass of ~12 t. A new probe/drogue interface is added beside
  it; IDSS stays unchanged.
- **Compound:** `merge_docked` holds the docked vehicle as a fixed `Vehicle::set_attached_mass` point mass. New: the
  attached mass and its CG are updated during the transfer.
- **Boil-off:** none exists; `environment::occulted_fraction` (eclipse) exists and is used for SRP.
- **Lunar missions:** `lunar_mission_budget` closes the budget from the current mass; `hls_refuel` hands over the
  real tank state instead of the configured full load.

## r1 design: vehicles and docking geometry

Corrections approved with the r1 plan (user 2026-10-20): (1) the tanker and the depot's booster reuse Block 3's
stages with `[[stage]] from = "<vehicle>:<stage>"`, only ports and connectors may be added; (2) depot dry mass 236.5 t
[E] (255 - 10.5 heat shield - 10 flaps - 1 catch pins + 3 insulation), dry CG as Block 3; (3) hardware layout [E]:
interface plane 0.5 m off the dorsal hull at station 20 m, probes / drogues at 12 / 28 m, y +-1.5 m, LOX connector at
8 m, CH4 at 24 m - centrelines 10 m apart; (4) the HLS legs' clearance at the depot is checked in r3; (5) ports mate
only with equal mechanisms, probes on the chaser, drogues on the target.

- **Depot** `config/vehicles/starship_depot.toml`: Super Heavy as Block 3; ship stage as Block 3 without the heat
  shield (10.5 t [E], the Block 1 figure) and the flaps (mass [E]), with an insulation layer (mass and effective heat
  resistance [E], literature in r2). Engines stay: the depot flies itself to orbit and makes up its drag.
  Four drogues on the dorsal side (azimuth 90 deg; the belly is at 270 deg), stations [E] spread over the tank length;
  the docking port's reference point is their centroid, its axis the dorsal normal. LOX and CH4 transfer connectors on
  the dorsal line at the height of the respective main tank [E].
- **Tanker** `config/vehicles/starship_tanker.toml`: Block 3 ship with four probes matching the drogues and the
  matching connectors. **HLS** (`starship_hls.toml`) gets the same probes.
- **Docked geometry:** parallel axes, back to back, rolled 180 deg against each other, both noses the same way;
  centrelines ~10 m apart (9 m diameter + gap [E]).
- **Translation capability:** the ship's RCS pods sit at azimuth 0 / 180 deg. Tangential thrusters give +-dorsal
  (the approach axis), inward thrusters +-lateral, the aft "forward" thrusters +x only (no -x). r1 verifies this with
  `rcs_pure_force_capability`; docking only needs the axial axis held, not braked.
- **Runtime tank mass:** `Vehicle::set_tank_mass(tank, kg)`, clamped to `[0, capacity]`, updating mass properties.

## r2 design: boil-off by heat balance

Implemented (plan `docs/superpowers/plans/2026-10-20-refuelling-r2-boil-off.md`, notes in `docs/subsystems.md`
"Boil-off by heat balance (r2)"). Corrections with the user during execution (2026-10-20): (1) only the wetted wall
feeds the liquid - its place by the Bond number (tide, drag, rotation), the dry wall through the ullage gas; (2) the
ship return checks its main tanks per propellant at the deorbit node by flying a copy of itself ahead, a short burn is
refused / called off, a burn on starved engines is aborted; (3) the Block 3 lunar missions run without boil-off until
r5 (Moon radiation, thermal attitude); (4) validation against Centaur flight reconstructions (AIAA 2007-5845).

- Lives in `vehicle` (the tank state), radiation sources from `environment`. Configured by `[stage.thermal]`
  (absorptivity, emissivity, effective wall heat resistance per tank or per stage [E]). Fidelity flag `boil_off`.
- Heat input per tank section (cylinder pieces from the tank stations): sun 1361 W/m^2 [S] x occulted fraction x the
  projected area for the sun angle to the long axis; Earth albedo 0.30 [E] x solar flux x view factor (altitude) x
  illumination phase; Earth IR ~239 W/m^2 [S] x view factor.
- Per section a quasi-steady balance: absorbed = radiated + leak into the tank; the leak runs through the effective
  wall resistance between the skin temperature and the boiling point (LOX 90 K, CH4 112 K [S]).
- Leak power / latent heat (LOX 213 kJ/kg, CH4 510 kJ/kg [S]) = the boil-off mass flow of that tank; the gas is vented.
- Deliberate simplifications (documented): no skin heat capacity (quasi-steady; the propellant integrates the heat
  anyway), no tank pressure, stratification or LOX/CH4 common-dome exchange.
- Two uses: in flight per tick (or on a coarse grid) for every vehicle with `[stage.thermal]`; for the calculator as
  an orbit average for a given orbit and attitude.
- Constants go to `math/constants.hpp` with their source.

## r3 design: Starship rendezvous and docking

- **Target block:** `[stage.station]` becomes a general target block (LVLH attitude, proximity limits, optional
  reference orbit) used by the ISS and the depot. The depot is a full `FlightBody` like the ISS (same force model incl.
  drag) with an ideal LVLH attitude hold (documented simplification). Its attitude-hold gas over weeks is a campaign
  item, estimated [E] from the gravity-gradient torque in r6.
- **Depot attitude:** nose along +V-bar (low drag), dorsal side to nadir. Tankers approach from below along the R-bar
  (natural braking, as Dragon's IDA-3 approach) and fly rolled 180 deg, dorsal side up.
- **Phasing:** `plan_rendezvous` accepts Starship. Coelliptic pattern as for Crew-2 with Waypoint 0 below the target.
  Burns on one vacuum Raptor; below a minimum size [E] on the RCS (a Raptor cannot deliver very small impulses
  cleanly). Drag with Starship's free-molecular coefficient.
- **Launch window:** the existing window search of the ISS missions with the depot orbit as target. At 26.5 deg
  inclination from Starbase (26.0 deg latitude) the plane touches the site about once a day; this sets the shortest
  launch interval.
- **Proximity:** `ProximityGuidance` takes its limits from the target (keep-out sphere, approach ellipsoid, R-bar
  waypoints, hold and retreat points [E]); the final approach runs along the depot's dorsal normal.
- **Probe/drogue contact** (new, in `contact`): four probes into four cones, one spring-damper contact per probe,
  stiffness designed for a ~400 t tanker against a depot up to >1,000 t; soft capture with latches -> retraction ->
  rigid mate. Hull collision of both cylinders through the existing primitives. Envelope [E]: closing 0.03-0.1 m/s,
  lateral <= 0.1 m, angle <= 1 deg.
- **Compound:** the depot carries the tanker as attached mass (existing mechanism, momentum conserved); new is the
  continuous update during the transfer.
- The ISS tags `[iss]`, `[docking]`, `[proximity]`, `[rendezvous]` stay green through the generalisation.

Corrections approved with the r3 plan (user 2026-10-20): (1) the tanker and the HLS get two -x cold-gas thrusters
[E] - with +x along-track thrust only, `v_along + 2 n x_radial` is conserved and a hold or retreat below the depot
cannot stop the forward drift ("+x only" above was wrong); (2) the phasing burns run on the single belly Raptor
(`sea_level_3`, the deorbit set), not a vacuum Raptor (the vacuum group lights its three together), below ~7 m/s on
the cold-gas RCS; (3) a station is one stage (the depot without its booster); (4) a chaser already on the port axis
beyond an on-axis waypoint approaches along the axis (the depot's Waypoint 0 lies on its R-bar). Finding: from
Starbase the plane crosses the pad every ~22 h, but within the crew phasing limit of 28 h only every few days'
crossing closes the phase (first window after 7.4 days in the test) - the launch interval is set by the phasing,
not by the plane (open for r6).

## r4 design: propellant transfer

- **Drive:** pressure difference between the tanker's ullage (held pressurised, ~3-4 bar [E]) and the depot's (vented
  to a lower pressure [E]). Flow per line `mdot = C_d A sqrt(2 rho dp)`, line area and `C_d` [E]; LOX and CH4
  separately.
- **Settling:** the aft "forward" thrusters push the compound along +x at ~1e-4 g [E]; the existing settling model
  decides whether the liquid sits at the outlets. With the depot's nose along +V the push is posigrade and offsets
  part of the drag. Gas from the main tanks at Isp 70 s.
- **Live masses:** both vehicles' tank masses and CGs are updated continuously; the compound's attached mass follows.
- **End:** tanker main tanks at the computed residue (deorbit + RCS gas for departure and return + unusable residue
  [E]) or depot full.
- **Losses:** line chill-down [E]; boil-off continues during the transfer.
- **Result:** duration, delivered LOX and CH4, gas used, losses.

Decisions approved with the r4 plan (user 2026-10-20): lines 0.15 m / C_d 0.6, 3.5 / 1.5 bar, chill-down 150 / 60 kg
per connector [E]; the push as an average impulse per 10 s step, outlet covered at Bo >= 10 within 30 deg [E]; the
docked pair's boil-off by the session; the tanker reserve by the rocket equation from the previewed deorbit
(8-day pass search) + gas 1.5 t, boil-off 2 t, unusable 1 t [E]; floors honoured at the line's closing; both
directions in one model; FlightSim interface in r6.

## r5 design: reference flights

Missions in `config/missions/`, chained by the existing mission sequence:
1. `depot_launch`: the depot launches from Starbase into the depot orbit; the residue after the ascent is its initial
   fill.
2. `tanker_flight`: launch window into the depot plane -> ascent with booster catch -> phasing -> proximity ->
   docking -> transfer -> undocking and departure -> deorbit -> entry -> flip and catch on the header tanks.
   Plus an orbit start near the depot for fast tests.
3. `hls_refuel`: the HLS launches, docks, is filled and undocks; the existing lunar mission then continues with the
   **real** tank state instead of the configured 1,600 t.

Measured numbers for the calculator: residue after ascent (tanker and depot), rendezvous propellant, net delivered,
transfer duration and gas, tanker reserve at the catch, the depot's boil-off per day.

Corrections with the user (r5, 2026-10-20; numbers in `docs/subsystems.md` "Reference flights (r5)"): (1) the plan's
nine decisions approved (chain generalised in place, depot placed not flown in the tanker / HLS flights, the depot
snaps to its station attitude, the caught booster leaves the tower, 48 h uncrewed phasing, HLS launch stack and
insulation, depot in the lunar parking plane, Moon radiation; Block 3 lunar returns keep boil-off off); (2) with the
flown Block 3 cargo ship the tanker delivered 17.6 t - the user tuned the tanker to 100 t after its ascent: own dry
mass 209 t [E] (no cargo hardware; heat shield, flaps, headers stay) and insertion at 120 km raised to 170 km at
apoapsis (flight-profile probes: the booster's return reserve limits everything else); the values are placeholders
until the real tanker is known; (3) the tanker reserve items come from the flown return (gas 3.5 t, boil-off 56 %
CH4); (4) found and fixed on the way: the ascent leg changed during the engine tail-off; the departure burn fought the
Coriolis term with a heavy chaser; an autonomous mission resumes a failed capture up to twice.

## r6 design: campaign calculator, game interface

- **Calculator** `mission::refuelling_campaign` (a pure function, daily steps):
  - Demand: what `lunar_mission_budget` needs from the HLS in the depot orbit, plus the HLS rendezvous and transfer
    loss.
  - Inputs: depot altitude, launch interval (at least the window spacing), depot initial fill, net per tanker
    (measured; scaled to other altitudes by the Hohmann cost up and back).
  - Daily losses: boil-off (orbit average from r2), drag makeup (US76 at the day's depot mass), attitude-hold gas.
  - Output: **number of tanker flights**, campaign duration, depot fill over time, loss breakdown, a limit message when
    the depot volume does not suffice.
- **Measured-numbers file:** the reference flight's numbers are stored (committed), so the calculator runs without a
  new flight; a test checks they still match the flight.
- **Game:** `FlightSim.start_mission` for `depot_launch`, `tanker_flight`, `hls_refuel`; getters `transfer_state`
  (flow, masses, duration) and `depot_state` (fill, boil-off rate); command `refuelling_campaign(params)` returning a
  Dictionary. UI by the user.
- **CLI:** a small C++ program `refuelling_report` in the core build (the calculator needs core models; a Python tool
  like `vehicle_report.py` would not have them).

Corrections with the user (r6, 2026-10-20; numbers in `docs/subsystems.md` "Campaign calculator (r6)"): the plan's
nine decisions (real windows, minimum-mass HLS demand, tanker net scaling, depot losses with 20 kg/day gas [E], 1.5 %
vent [E], file checks, campaign clock, worker command, CLI); the window search spans 60 days. Answer: 12 tanker flights
in 170.7 days; the cadence is set by the phasing orbit's phase alignment (~13.8 days), not by the phasing limit.

## Error handling

- Commands outside their state (transfer while not docked, undock during transfer) are refused with a German
  `refusal()`, as in milestone (i).
- A failed capture retreats to the hold point; a hull collision ends the flight (as the ISS docking).
- A transfer whose settling fails (liquid away from the outlets) pauses the flow and reports it.
- The calculator reports when the campaign cannot close (depot volume, boil-off exceeding delivery at long intervals).

## Testing

- Unit tests per model (tank setter, heat balance closure, eclipse without sun, analytic cylinder case, flow law,
  settling, probe contact, compound mass update, calculator against a hand computation).
- Boil-off validated against published cryogenic-tank figures in LEO (sources found in the r2 plan, tolerance
  documented).
- Reference flights as `[slow]` campaigns.
- The flight count is compared with the public range (4 to high teens, "10-ish") **for plausibility only**, without
  tuning towards it (physics over gauges).
- Each part runs the tags it affects, including the ISS and lunar tags of touched code paths.

## Documentation

- `docs/subsystems.md`: a refuelling section (vehicles, boil-off, transfer, missions, campaign numbers, diagnostics).
- `CLAUDE.md`: module summary updated.
- `docs/ansichten.md`: the new missions and states, if they have views.
- `config/README.md`: the new config blocks (`[stage.thermal]`, probes / drogues, target block).

## Out of scope

- The fully flown campaign (later, as a check of the calculator).
- Tank pressure, temperature, stratification; slosh dynamics.
- Hot-gas thrusters (the config keeps the cold-gas RCS).
- Manual flying (M7).
- Refuelling in HEEO or lunar orbit.
