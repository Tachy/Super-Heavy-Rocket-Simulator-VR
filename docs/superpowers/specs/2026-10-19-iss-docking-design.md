# ISS rendezvous and docking with Crew Dragon - design

Status: approved in the brainstorm 2026-10-19 (sections 1-5); written spec awaiting the user's review. Milestone (i),
part of plan milestone M9 (rendezvous and ISS docking), after the lunar milestone (l1-l6, main `6da3f70`).

## Intent

The user's request (2026-10-16, German): an ISS the vehicles can dock with. Plan M9: ISS orbit from real data, phase
planning, Clohessy-Wiltshire approach, RCS fine control, a docking-port contact model; check: a docking scenario with
a contact speed < 0.1 m/s succeeds.

Decisions taken with the user (2026-10-16 and the brainstorm 2026-10-19; memory `project-iss-docking`):
- **Only Crew Dragon docks** (Starship is not designed or certified for the ISS).
- **Tanker flights** (ship-to-ship propellant transfer, also M9) are a separate later milestone that reuses this one's
  rendezvous machinery.
- **Missions:** both a core mission that starts in the separation orbit (as the lunar legs start from states) **and** a
  continuous mission from the pad with launch-window planning; plus a departure mission (undock -> existing capsule
  return).
- **ISS orbit:** NASA "ISS Trajectory Data" (CCSDS OEM, public domain) when the mission date lies in an OEM's usable
  window - including a self-collected archive (every `fetch.py` run keeps the file) - otherwise a synthetic reference
  orbit. No SGP4 / TLE (Space-Track terms forbid redistribution; OEM is more accurate than SGP4). Fidelity flag.
- **Drag:** US76 (exists) now, NRLMSISE-00 with M8. The density error at 400 km (factor ~5-10 with solar activity) is
  documented; the guidance closes the loop on it.
- **Docking mechanism:** middle fidelity - soft-capture ring and guide petals as penalty contact (real geometry and
  forces), capture latches inside the envelope, a damped joint, a timed retraction, hard capture to one compound body.
  Not a Stewart-platform controller (no public data).
- **Control levels:** autopilot only in this milestone, plus hold / retreat / abort at any time. Manual 6-DOF docking
  comes with the cockpit (M7).
- **Both ports:** IDA-2 (Harmony forward, V-bar approach) and IDA-3 (Harmony zenith, R-bar approach), chosen per
  mission.
- **Architecture A:** the ISS is a second `FlightBody` in the `FlightSession` with the **same force model** as the
  Dragon. Interpolating the OEM directly would put its prediction error (~1-3 km/day) into the relative motion as a
  phantom drift; CW relative dynamics only serve the planning (linear, no J2, no differential drag).
- Undocking and departure are in scope; the hand-over to `plan_capsule_return` closes the chain ISS -> Earth.
- Drawing and UI wait for the user's UI specification; the game interface (bindings) is built ahead.
- The full OEM file is committed as a validation fixture (public domain, ~700 kB).

## Decomposition

| Part | Content |
|---|---|
| i1 | ISS as a target body: OEM fetch + archive, OEM parser/interpolation, synthetic orbit, ISS vehicle config, ideal LVLH attitude, OEM events, drag in orbit |
| i2 | Crew Dragon 6-DOF translation (Draco layout review), translation command, relative navigation |
| i3 | Phasing: separation orbit -> approach ellipsoid entry, full-model targeting, re-plan |
| i4 | Proximity operations and final approach: waypoints, keep-out sphere corridor, V-bar / R-bar, hold / retreat / abort |
| i5 | Docking mechanism: soft capture, latches, attenuation, retraction, hard capture, compound body; undocking and departure |
| i6 | Missions (orbit start, pad start with launch window, departure), game interface, documentation, end-to-end campaign |

Each part gets its own plan, tests and push (as l1-l6).

## What exists and what changes

Exists: `FlightSession` with several bodies on one 200 Hz tick and a coast integrator (interpolated ephemeris and
orientation), US76 and the body air data, individual RCS thrusters with the box-QP allocator (`rcs_keep_force`) and the
pulse modulator, Crew Dragon (`config/vehicles/crew_dragon.toml`: 16 Dracos [E layout], nose cone, orbit attitude
modes, Draco burns), `falcon9_crew_dragon` (SLC-40, 51.6 deg, 200 x 210 km, payload separation), the capsule return
(`plan_capsule_return`, d3/d4), `orbit/lambert`, `mission/position_targeter`, finite burns, payload stacking and
separation (`config::stack_payload`), penalty contact (`contact/pin_contact`, primitives), `SessionRunner` and the
`FlightSim` command/result pattern.

New: `orbit/oem` (parser + interpolation), `orbit/synthetic_station_orbit`, an ideal-attitude mode for a body, OEM
events as impulses, a translation command in a target frame, relative navigation, `mission/rendezvous_*` (phasing and
proximity planners), `gnc/proximity_guidance` (waypoints, corridor, final approach), `contact/docking_mechanism`
(ring, latches, joint, hooks), body merge (docking) and split (undocking), missions and bindings.

## i1 design: ISS as a target body

**Data.** `tools/data_pipeline/fetch.py` downloads
`https://nasa-public-data.s3.amazonaws.com/iss-coords/current/ISS_OEM/ISS.OEM_J2K_EPH.txt` and also stores it as
`data/iss_oem/ISS.OEM_J2K_EPH_<CREATION_DATE>.txt` (skipped when a file with that creation date exists): the archive
grows with every run. Source and licence (NASA, public domain) in `tools/data_pipeline/sources.toml`. The file (format
seen 2026-10-19, created 2026-10-14): CCSDS OEM 2.0, `REF_FRAME = EME2000`, `TIME_SYSTEM = UTC`, 4 min steps over
15 days, km and km/s, header comments `MASS=460320.00`, `DRAG_AREA=1177.76`, `DRAG_COEFF=1.50`,
`SOLAR_RAD_AREA=0.00` (kg, m^2) and a "TRAJECTORY EVENT SUMMARY" table (event, TIG as day-of-year GMT, DV m/s, HA/HP km).

**`orbit/oem`.** Parses one file (metadata, header values, events, states); interpolates position and velocity by
Lagrange interpolation of order 8 (the CCSDS-recommended scheme for OEM) on the state rows, UTC -> TDB through the
existing `time::TimeScaleConverter` at load time. An `OemArchive` picks, for an epoch, the **most recent file whose
usable window contains it** (the newest prediction). EME2000 is our GCRF/J2000 axis set (the frame-bias difference,
~23 mas, is below the OEM's own accuracy; documented).

**`orbit/synthetic_station_orbit`.** Mean elements (a, e, i) from the latest available OEM (osculating at its first
ascending node), RAAN and argument of latitude advanced to the mission date by the J2 secular rates; the scenario sets
the phase (argument of latitude) at the mission epoch. Without any OEM, defaults in `iss.toml` [E: 51.64 deg,
~415 km]. Fidelity flag `iss_orbit = "oem" | "synthetic" | "auto"`: `oem` outside every window is an error (no silent
fallback); `auto` uses the OEM where it can and otherwise the synthetic orbit, and reports which it used.

**ISS vehicle.** `config/vehicles/iss.toml`, one stage, no engines: mass, drag area and C_D from the OEM header when an
OEM is used [S], else the config's values; inertia [S/E from NASA documentation]; collision primitives - pressurised
modules as cylinders, the truss as boxes, solar arrays as plates in a nominal orientation [E dimensions from NASA
documentation]; ports IDA-2 (Harmony forward) and IDA-3 (Harmony zenith) with position, docking axis and clocking in
the ISS body frame.

**Ideal attitude.** A new attitude model for a body: the body axes follow LVLH (+XVV, a TEA offset from the config
[E]) exactly; contact torques do not change the attitude (the CMGs are taken as strong enough - documented,
switchable). Translation is free: gravity, drag, contact impulses.

**Drag in orbit.** All bodies get drag from the reference body's air down to vacuum (US76 to 1,000 km): the ISS with a
fixed area x C_D, the Dragon with its capsule aerodynamics. The plan checks whether today's `FlightBody` cuts aero off
at some height and lifts that.

**OEM events.** Events with DV > 0 (reboosts, debris avoidance) are applied as an impulsive Delta-v along the velocity
at their TIG [D]; a mission warns when one falls inside its window (real operations would not dock then).

**Validation.** From an OEM state with our force model (US76, header area and C_D, no SRP as in the header) against the
OEM after 6 h and 24 h: expected a few km, mostly along track (US76 density); the tolerance is set after the first
measurement and documented. Interpolation against the rows themselves (leave-one-out, < 1 m). Synthetic orbit: RAAN
drift against the J2 formula; back-propagation to a known OEM date. Fixture: the full OEM file in
`tests/validation/iss/`.

## i2 design: Crew Dragon 6-DOF and relative navigation

- The **16 Dracos** [S] stay; their layout is [E]. First measure with the allocator the pure translation reachable
  along +-x / +-y / +-z at zero torque. Below a minimum authority (e.g. >= 0.05 m/s^2 on 12.5 t, ~600 N net [E]) the
  layout is changed [E] - e.g. side pods split over two stations, so lateral forces combine without a tipping moment -
  with the reason in the config.
- A **translation command** in a target frame (the ISS port frame): force fractions to the allocator with
  `rcs_keep_force`; attitude control keeps priority (as today).
- **Relative navigation:** Dragon port <-> ISS port position and velocity from the true states (ideal; the real
  Dragon uses GPS, LIDAR and thermal cameras). Fidelity flag `relative_nav = "ideal"`, sensor noise later. The Dragon
  port sits under the nose cone: docking needs `nose_cone_open`.
- Validation: authority per axis, coupled torque ~ 0, propellant per m/s.

## i3 design: phasing

- Profile after the real Crew Dragon pattern (coelliptic): phasing burns (NC) to set the phase rate, height adjust
  (NH) to a coelliptic orbit Delta-h below the ISS [E, a few km], coelliptic (NSR), terminal phase initiation (TPI) at a
  line-of-sight elevation [E], one or two mid-course corrections, braking to the entry point of the **approach
  ellipsoid (AE, 4 x 2 x 2 km about the ISS)** [S] with a near-zero relative velocity.
- Planning: timing and Delta-v estimates analytically (Hohmann / CW), each burn then targeted on the **full model**
  (`position_targeter`, finite Draco burns of minutes). The planner picks the cheapest number of revolutions within a
  mission limit (default ~1 day [S: typically 24-28 h on crew missions]) - the user's rule "cheapest correct window".
- After each burn the actual state is compared with the plan; beyond a tolerance the remaining sequence is re-planned
  from the live state.
- Result structure as the lunar plans: burn nodes, Delta-v per burn, total against the Dragon propellant (2,562 kg)
  with a reserve for deorbit and abort.
- Validation: a campaign over phase angles 0-360 deg x both ports: AE entry hit (position <= 100 m, relative
  velocity <= 0.5 m/s [E]), total Delta-v of the order of real missions (figures researched in the plan), no AE entry
  before it is cleared.

## i4 design: proximity operations and final approach

- Waypoints in the ISS LVLH frame, V-bar for IDA-2 and R-bar for IDA-3: **Waypoint 0** at the AE boundary / ~400 m
  [S: NASA mission blogs, verified in the plan] with a go/no-go hold; **Waypoint 1** ~220 m on the port axis, just
  outside the **keep-out sphere (KOS, 200 m)** [S], hold and alignment on the docking axis; **Waypoint 2** ~20 m on the
  axis, hold; then **contact** at the ring's target speed.
- Transfers between waypoints: CW initial guess, then multi-impulse translation targeted on the full model. Inside
  200 m an **approach corridor** (a cone about the port axis, half angle ~10 deg [E]); leaving it holds automatically,
  a large deviation retreats.
- Final approach (20 m to contact): range-scheduled closing rate (~0.1 m/s at 20 m to ~0.08 m/s at the ring [E]),
  lateral control onto the axis, attitude onto the port axis including roll - all on Draco pulses.
- **Hold** (keep the relative position), **retreat** (back to the previous waypoint) and **abort / breakout** (a
  separation burn out of the KOS onto a passively safe trajectory) at any time, automatic and as a command.
- A state machine with German phase names ("Phasing", "Annäherung", "Wegpunkt 0/1/2", "Endanflug", "Halt",
  "Rückzug", "Abbruch", "Kontakt", "angedockt", ...) in the pattern of the existing `AutopilotState`.
- Validation: arrival at the ring inside the IDSS envelope, a corridor violation is a failure.

## i5 design: docking mechanism, undocking and departure

- **Soft-capture ring** of the Dragon [E dimensions from IDSS: ring diameter, three guide petals] against the passive
  IDA ring as a **penalty contact** (spring-damper, stiffness/damping [E]); hull <-> ISS primitives for collisions.
- **Capture latches** close when the ring sits inside the envelope. Guide values (verified against the IDSS IDD
  Rev. E in the plan and then marked [S]): axial 0.05-0.10 m/s, lateral <= 0.04 m/s, offset <= 0.10 m, angle <= 4 deg,
  rate <= 0.2 deg/s.
- Then a **damped 6-DOF joint** (loads attenuated [E]), a **retraction** of the ring as a timed sequence [E: a few
  minutes], and **hard capture** (12 hooks): one **compound body** with summed mass and inertia, flying in the ISS
  attitude model.
- Failures: too fast or offset - the ring bounces, the latches do not engage, the Dragon retreats; hull contact -
  status "Kollision" with the contact impulse, the mission has failed.
- **Undocking:** hooks open, spring separation [E: ~0.1 m/s], a departure profile after the real pattern (burns out of
  the KOS and the AE) [E]; the Dragon then flies free and hands over to `plan_capsule_return` (deorbit, splashdown).
  The ISS part ends at the release.
- Validation: contact cases inside and outside the envelope, momentum conservation through capture and merge,
  undock -> separation speed.

## i6 design: missions, launch window and game interface

**Missions** (`config/missions/`, new `[mission] kind` values):
- `crew_dragon_iss`: starts in the separation orbit (200 x 210 km, 51.6 deg) with date, port (`ida2` / `ida3`),
  `iss_orbit`, and for the synthetic orbit the phase (default from the separation state of a realistic launch;
  the scenario may set it). Phasing -> proximity -> docking -> docked.
- `falcon9_crew_dragon_iss`: from SLC-40 / LC-39A with a new **launch-window planner**: the times on the date when the
  pad passes through the ISS orbit plane (northerly crossing); the first whose phase angle allows phasing within the
  mission limit (instantaneous window, as real); a small plane error is removed by an out-of-plane part during
  phasing; then the existing ascent and separation and `crew_dragon_iss`.
- `crew_dragon_iss_departure`: starts docked; undocking, departure, chain into `crew_dragon_return`.

**Game interface** (`FlightSim`, pattern of d5/l6; the user specifies UI and HOTAS): getters `iss_state` (state,
attitude, port frames), `rendezvous_plan` (burn nodes, Delta-v), `docking_state` (phase, relative position and
velocity in the port frame, range, range rate, offset, angles, latches / hooks), `station_geometry` (primitives for
drawing); commands with ids `hold`, `retreat`, `abort`, `resume` (re-plan from the live state), `undock`. Covered by
the smoke test.

**Performance.** Two bodies over 1-2 days of coast run in the existing coast integrator with interpolated ephemeris;
burns and proximity (inside ~5 km) on the fixed 200 Hz tick. Target: a mission start (planning) of a few seconds, as
the lunar missions.

## Error handling

- `iss_orbit = "oem"` without a usable OEM window, a malformed OEM file, a missing port id: a config error with a
  German message, no silent fallback.
- A phasing plan that does not close within the mission limit or the propellant reserve: the planner reports it
  (as the lunar planners); the mission does not start.
- A reboost inside the mission window: a warning in the mission result.
- In flight: corridor violation -> hold / retreat; failed capture -> retreat; hull contact -> "Kollision", failed;
  propellant below the deorbit reserve -> abort and departure.

## Testing

Tags `[iss]`, `[rendezvous]`, `[docking]`; `[slow]` for cases over ~40 s.
- Unit tests per module: OEM parser and interpolation, archive choice, synthetic orbit, translation / allocator, CW
  initial guess, corridor, contact, latches, merge / split.
- Validation: OEM propagation 24 h; phasing campaign phase angle x port; final-approach campaign with dispersed initial
  errors at Waypoint 2: every contact inside the envelope and **< 0.1 m/s** (the M9 criterion); failure cases (too
  fast -> no capture, hull contact -> collision); end-to-end from the pad and undock-to-splashdown `[slow]`.
- Propellant budget: phasing + proximity + departure + deorbit within the Dragon tank with a reserve.

## Documentation

`docs/subsystems.md` (an ISS / rendezvous / docking section with decisions and validation numbers), `CLAUDE.md`
(module list), `config/README.md` (ISS vehicle). `docs/ansichten.md` only gets pointers to the getters; the views
are the user's.

## Out of scope

Manual 6-DOF docking (M7), sensor-based relative navigation (fidelity flag later), NRLMSISE-00 (M8), Starship docking
and tanker flights (own milestone), Cargo Dragon (configuration variant later), ISS attitude dynamics with CMGs,
docking of other visiting vehicles, rendering of the ISS (after the user's UI specification).
