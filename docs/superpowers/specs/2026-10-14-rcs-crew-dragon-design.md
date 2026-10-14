# Individual RCS thrusters and Crew Dragon - design

Status: draft 2026-10-14, for the user's review. Milestone (d), after the merge of `feat/scvx-flight` (main `4e8992d`).

## Intent

The user (2026-10-14): "Außerdem ist ja offen: Steuerdüsen von Starship und Dragon-Kapsel und die Dragon-Kapsel selbst."
Decisions taken by the user (AskUserQuestion, 2026-10-14; memory `project-dragon-rcs-scope`):
- Dragon: **Crew Dragon** (SuperDracos, crew layout). Cargo Dragon later as a configuration variant.
- First Dragon milestone: **separation + orbital flight**, and **re-entry + splashdown**. Launch abort (SuperDraco) and
  docking (M9) are out of scope.
- RCS of Starship, Super Heavy, Falcon 9 and Dragon: **individual thrusters** (position, direction, thrust, minimum
  impulse bit, propellant use, allocation; per-thruster state for rendering).
- Drawing and UI wait for the user's UI specification; the game interface (bindings) is built ahead.

Success:
- Every vehicle's attitude control and RCS translation acts through its individual thrusters. The torque and force
  come from where each thruster sits and where it points, and each thruster draws on a finite propellant supply.
  The existing campaigns stay where they are: Super Heavy 21/21, ship from orbit 14/14, ship from the gate 20/21,
  Falcon 9 RTLS / ASDS 21/21.
- A Falcon 9 lifts a Crew Dragon from SLC-40 to its insertion orbit at 51.6 deg. The capsule separates, opens its
  nose cone and holds attitude on its 16 Dracos. It flies translation burns on the Dracos with the existing burn
  executor.
- From orbit, the onboard computer plans and flies a Draco deorbit burn and jettisons the trunk. It then guides a
  lifting entry by bank (offset-CG trim) to the drogue deployment point. Drogues, then mains, bring the capsule down
  in the target recovery zone at splashdown speed ("gewassert"). The event timeline matches published Crew Dragon
  returns within the stated tolerances.

## Decomposition

| Part | Content | Validation |
|---|---|---|
| d1 | Individual RCS thrusters for all existing stages: config, physics (force, moment, propellant), allocator, pulse modulation; the controllers' authority estimates come from the thrusters | Unit tests (geometry, allocator, minimum impulse, starvation); all existing campaigns re-run unchanged in outcome |
| d2 | Crew Dragon vehicle (capsule + trunk, Dracos, SuperDracos as data), Falcon 9 crew mission, separation, nose cone, orbital attitude modes, Draco translation burns | Ascent to the insertion orbit; separation clearance; a 1 m/s and a 10 m/s Draco burn within tolerance; attitude hold with propellant use |
| d3 | Capsule aerodynamics (blunt body, offset CG), heat shield corridor, deorbit planner on Dracos with trunk jettison, entry guidance to the drogue point | Trim alpha and L/D against the reference; dispersed entry campaign hits the drogue point |
| d4 | Parachutes (drogues, mains, reefing, inflation, opening loads), splashdown judgement, recovery zone; full return mission | Event timeline vs published returns; full mission from orbit splashes down in the zone |
| d5 | Game interface: per-thruster state, capsule and parachute state, Dragon missions in `FlightSim`; docs | Smoke test |

Each part has its own plan; part decisions go into that plan's ledger.

## What exists and what changes

- `[stage.rcs]` today holds `thruster_count` and `thrust_per_thruster_n`. `Vehicle::max_rcs_torque_nm()` assumes that
  half the thrusters act as pure couples: the pitch/yaw lever is half the vehicle length, the roll lever is the radius.
  `FlightBody` applies `fraction x max` with no propellant use. The callers are `AttitudeController::allocate`, its
  rate limit (what the RCS can still stop), the booster's coast trim (`rcs_force_fraction`), `burn_node` RCS burns,
  and the slew-time estimates in `FlightSession`.
- Change: `[[stage.rcs.thruster]]` entries replace the counts. The command interface stays a desired body torque and
  force (`VehicleCommand::rcs_torque_fraction` / `rcs_force_fraction` become `rcs_torque_nm` / `rcs_force_n` in body
  axes). `Vehicle` turns them into thruster duty cycles (the allocator) and on/off pulses (the modulator), and
  `FlightBody` integrates the force and moment of the thrusters that are actually firing.
  `max_rcs_torque_nm()` / `max_rcs_force_n()` stay as the allocator's per-axis capability: what all the thrusters
  give together along each axis. They are computed from the geometry, so the callers keep their meaning.
- No blunt-body aerodynamics, parachutes or capsule exist yet. `landing_outcome` knows "Wasserung" only as a failed
  landing (ditching). A planned splashdown is a new outcome: "gewassert".

## d1 design: individual thrusters

### Configuration
```toml
[stage.rcs]
type = "cold_gas"                 # documentation only
isp_s = 70.0                      # [E] per propellant: N2 cold gas ~70 s, Draco ~300 s vac
propellant_tank = "rcs_n2"        # a [[stage.tank]] of the stage (contents = "rcs"), or "main_ullage" for gas
                                  # drawn from the main tanks (Starship, Super Heavy: autogenous gas, mass from
                                  # the main LOX/CH4 tanks)
minimum_impulse_bit_ns = 20.0     # [E] smallest impulse one pulse delivers
minimum_on_time_s = 0.02          # [E] valve; the modulator never fires a shorter pulse

[[stage.rcs.thruster]]
id = "pod_a_up"
position_m = [12.0, 1.8, 0.0]     # station (x along the stage axis from its base), y, z in body axes
direction = [0.0, 0.0, 1.0]       # thrust direction (the force on the vehicle; the plume points the other way)
thrust_n = 450.0
```
A `pattern` helper (a ring of `count` thrusters at a radius and station with a direction in the local radial/
tangential/axial frame) keeps symmetric layouts short. The loader expands it into thrusters, so the core sees only
the expanded list. `separate()` re-bases the positions like the other stage geometry.

Layouts [E] (positions from imagery, thrust from public figures where they exist; the plans record the sources):
- Super Heavy: gas thrusters on the forward dome, four pods [E]. They draw on main-tank ullage gas.
- Starship (ship): forward and aft thruster clusters on both sides [E]. Main-tank ullage gas.
- Falcon 9 first stage: two N2 pods on the interstage, 4 thrusters each [E]. Falcon 9 second stage: N2 cold gas,
  it has none today [E].
- Crew Dragon: 16 Dracos, 400 N, Isp 300 s [S] (4 forward under the nose cone, 12 on the side in quads) [E positions];
  propellant MMH/NTO shared with the SuperDracos [S].

### Allocation
Given a desired body torque `tau` and force `f` (6-vector `w`), the allocator finds duties `d_i` in [0, 1] that
minimise `||B d - w||^2 + eps * sum(d_i)`, where `B` is the 6 x n effectiveness matrix (columns `[r_i x F_i; F_i]`,
with `r_i` from the current CG). This is a bounded least-squares problem, small (n <= ~40), solved by projected
active-set iterations warm-started from the previous tick. Torque has priority over force. The torque rows are
weighted 100:1, so a saturated set gives up translation first. `B` is rebuilt only when the CG moves by more than
1 cm or the set of available thrusters changes (a thruster fails, or a tank runs dry).
Per-axis capability (`max_rcs_torque_nm`, `max_rcs_force_n`) = the allocator's result for a unit demand on that axis,
scaled to saturation. It is cached the same way.

### Modulation and physics
Each thruster has a sigma-delta modulator. It accumulates `d_i x dt` and fires a whole tick once the accumulated on
time reaches `minimum_on_time_s`. A thruster that has fired keeps firing while its duty stays above 0.5. At 200 Hz
(5 ms ticks) a 20 ms minimum on-time means pulses of at least 4 ticks. Small demands therefore become fewer, larger
pulses rather than a continuous trickle: this produces the limit cycle of real attitude hold. The force and moment in
a tick are the sum over the firing thrusters (`r_i x F_i` about the current CG). Each firing thruster's mass flow
`F / (Isp g0)` is drawn from its tank. When the tank is empty the thruster is starved, as an engine would be.
Thrusters are on/off only with a fixed thrust (constant feed pressure; user 2026-10-14: "konstanten Druck annehmen
... fest annehmen und nur über die Zeit steuern. Autopilot arbeitet dann auch damit"). There is no continuous-thrust
mode: autopilots and manual flight both fly on the pulses. Not modelled (documented): thrust rise and tail-off
within a pulse, the Isp loss of short pulses, and thrust falling with the tank pressure.

### Callers
`AttitudeController` keeps its torque law and passes the remainder to `Vehicle::allocate_rcs`. Its rate limit uses
the cached capability. The booster's coast trim passes a force. `burn_node` RCS burns pass a force along the burn
direction, and the burn executor measures the achieved acceleration from the actual pulses, as it does today.
Per-thruster state for rendering: `FlightBody::rcs_thrusters()` returns position, direction, firing flag and duty.

### Risks
Real geometry changes the authority: the roll lever stays the radius, but the pitch/yaw levers become the true pod
stations, which are often shorter than half the length. The plan re-runs every campaign. If one degrades, the
thruster layout gets a source check first, and only then a controller retune. The retune is ledgered.
Super Heavy flips on its engines, not on its RCS, and is not expected to be affected. The ship's entry uses RCS to
help the flaps, and the 14/14 and 20/21 campaigns will show whether that still works.

## d2 design: Crew Dragon and orbital flight

### Vehicle
`config/vehicles/crew_dragon.toml`, two stages: `trunk` (below) and `capsule`.
- Capsule: diameter 4.0 m [S], height 4.4 m with the nose cone open [S]. Dry mass ~9.5 t with crew [E from 12.5 t
  launch mass and propellant]. Propellant 2.56 t MMH/NTO [S] in one `rcs` tank. 16 Dracos (as in d1), and 8
  SuperDracos as an engine group (71 kN each, Isp 235 s [S]; present as data, not flown in this milestone). Heat
  shield PICA-X (d3), nose cone (open/closed: a flag that changes the drag in d3 and has no mass effect).
- Trunk: 2.5 m [E] long cylinder plus fins, ~1.5 t [E]; solar arrays and radiators are not modelled. It separates
  before entry (d3) as a second body that is no longer guided.
- `falcon9_block5.toml` stays the launcher. A mission `config/missions/falcon9_crew_dragon.toml` stacks Falcon 9 with
  `crew_dragon` as its payload vehicle (no fairing). This is new: until now the payload has been a mass only. Then
  SLC-40 (Crew-9 flew from there [S]), 51.6 deg, insertion ~200 x 210 km [S], and an RTLS or ASDS first stage as
  already flown.

### Payload as a vehicle
`AscentMission.payload_vehicle` (optional): the payload is the top of the stack as real stages, not a point mass.
Separation from the second stage is a staging event at SECO + 30 s [E]: spring push 0.3 m/s [E], then the capsule
becomes the controlled body (the second stage coasts unguided; a deorbit of the second stage is a later topic).

### Orbital flight
- Attitude modes on the capsule: inertial hold, LVLH hold (nose along the velocity, or "trunk forward" as flown),
  and pointing at a direction (sun / target), through `AttitudeController` with RCS only.
- Nose cone opens 10 min [E] after separation (it is closed for ascent and re-entry). This is a state flag only.
- Translation burns: `FlightSession::execute_burn` with the Dracos (`groups` empty, RCS translation). Burns below 1/4
  start/shutdown impulse that were rejected so far (CLAUDE.md "RCS translation comes with docking") become possible
  for an RCS-only vehicle. The burn executor stops on the measured delta-v.
- Propellant accounting: attitude hold costs grams per hour in pulsed mode (validated against the order of magnitude
  only; no public figure).

## d3 design: capsule entry

### Aerodynamics
`aero/capsule_aero` (new, one responsibility): blunt-body force and moment coefficients C_A, C_N, C_m about the heat
shield's apex, over Mach and total angle of attack (axisymmetric), from a table `aero/crew_dragon_aero.csv`. Source:
NASA public-domain data for Apollo-like 15 deg sidewall capsules [S, the plan records the report], scaled to the
Dragon diameter [E]. Reference area pi D^2 / 4. The CG offset ([E] radial, set so the hypersonic trim angle of attack
gives L/D ~0.2 [E]) produces trim. Bank is steered by the Dracos only (no flaps). Damping comes from the table's
C_mq column if the source has one; otherwise the column is zero and the RCS damps.

### Heat shield and corridor
`[stage.heat_shield]` as on the ship: Sutton-Graves with the shield's effective nose radius [E] (~1 x D for a
spherical-section shield), heat-rate, heat-load and load limits [E]. G-load limit for crew 4 g sustained [E]
(warning) - the corridor judgement already used for the ship.

### Deorbit and trunk
`gnc/deorbit_planner` is generalised over the vehicle. The burn is on the Dracos (RCS translation, ~10-15 min for
~100-130 m/s [E]), with the capsule's own entry predictor. The interface point is set where the drogues deploy
(below), not at a tower gate. Sequence [S, Crew Dragon returns since 2025]: deorbit burn, then trunk jettison (the
trunk enters as debris), nose cone closes, entry. The plan checks the order against the latest published timeline.
The flown splashdown zones are off California (Pacific) [S]. One zone is configured in
`config/splashdown_zones.toml` (point, radius [E] 10 km).

### Entry guidance
`gnc/entry_guidance` and `gnc/entry_predictor` are generalised over a vehicle aero table. The ship's table depends on
alpha and flaps; the capsule has a fixed trim alpha over Mach, so its table is C_D A / C_L A over Mach only.
The guidance flies bank modulation with reversals to the drogue deployment point (the "gate": altitude ~5.5 km [S]).
The ship's terminal nose-turn phase does not apply to the capsule. A `EntryProfile` choice (ship / capsule) in the
stage config selects the phases. Validation: a dispersed campaign (entry interface +-0.1 deg, mass +-3 %, aero
+-10 %) reaches the gate within 2 km [E] (the parachutes drift further than that).

## d4 design: parachutes and splashdown

### Model
`aero/parachute` (new): a canopy is a drag area C_D S(t) that grows from zero through reefing stages to full
(inflation times and reefing ratios [E], from NASA public-domain parachute literature). It acts through a riser
attachment point on the capsule. The force goes along the air-relative velocity of that point, and the moment comes
from the attachment offset. The canopy is massless, with no pendulum body of its own (a deliberate simplification,
documented; added mass and pendulum swing are left out). Opening load = q x C_D S. It is judged against a limit
[E] ("Fallschirm gerissen" beyond it).
Crew Dragon [S]: 2 drogues at ~5.5 km (~560 km/h), released when the 4 mains deploy at ~1.8 km, splashdown at
~25 km/h. The deployment triggers are altitude plus a dynamic-pressure check (`gnc/parachute_sequencer`).
Fidelity: wind is not modelled (the atmosphere has none yet); the zone radius covers it.

### Splashdown
`landing_outcome`: a vehicle with `[stage.recovery] method = "splashdown"` that reaches sea level gives
"gewassert" (success) when the vertical speed is <= 9 m/s [E] and its mains are open. Otherwise the outcome is
"Aufprall". The capsule rests at the surface (no buoyancy model).
Full return mission `config/missions/crew_dragon_return.toml`: from an orbit (as the ship's b4), plan the deorbit,
fly it, and splash down in the zone. Campaign: orbit phase dispersion as for b4.

### Evidence
Published event timelines of Crew Dragon returns (NASA blog / SpaceX timeline: deorbit burn, trunk jettison, drogue
and main deployment altitudes and speeds, splashdown time) [S]. Webcast telemetry of a return, if its overlay shows
speed and altitude: `tools/telemetry/webcast_telemetry.py` gains a Dragon layout. The plan checks whether such a
recording exists under a free licence. Tolerances: event altitudes +-15 %, splashdown speed +-30 % [E].

## d5 design: game interface

`FlightSim`: `body_rcs(i)` (thrusters: position, direction, firing, duty; for plumes), `body_parachutes(i)` (state,
inflation, attachment, direction), capsule flags (nose cone, trunk attached), missions `falcon9_crew_dragon` and
`crew_dragon_return` via `start_mission`. Covered by the smoke test. No drawing or UI.

## Out of scope

Launch abort (SuperDraco), docking and ISS (M9), Cargo Dragon, sea state, wind, buoyancy, the second stage's own
deorbit, crew displays.

## Choices made for the review (the user may change them)

1. RCS: on/off pulses with fixed thrust (constant pressure) and a minimum on-time, sigma-delta in whole ticks
   rather than true sub-tick pulse widths; no continuous mode (user 2026-10-14). Reason: it keeps the fixed 200 Hz tick. The cost is that impulses come in 5 ms
   steps, about 2 Ns for a Draco.
2. Starship and Super Heavy RCS draw gas from the main tanks (no separate tank). Reason: the real vehicles use
   autogenous ullage gas.
3. Splashdown zone off California (current practice), not Florida.
4. The capsule flies from SLC-40 on the existing Falcon 9 mission chain. The payload becomes a real vehicle (stages)
   instead of a mass.
5. Parachutes as massless drag areas at an attachment point (no pendulum body).
