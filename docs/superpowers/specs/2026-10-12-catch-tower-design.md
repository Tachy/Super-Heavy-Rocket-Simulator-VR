# Catch tower and arms - design

Status: approved in conversation 2026-10-12, sub-project 2 of 3 of the tower-catch landing guidance
(1 SOCP solver [done] -> 2 launch tower and catch arms -> 3 6-DOF successive-convexification landing guidance).

## Purpose

The returning booster is caught by two arms on the launch tower instead of touching down on the pad. This
sub-project builds the tower, the arms and the physics of the catch so that sub-project 3 has a real target
and an honest test: a catch either works mechanically or it does not.

User decisions:
- **Real contact model** (not a snap-to-caught criterion): the booster's catch pins land on the arms with
  spring-damper contact forces and friction; the booster stays a free 6-DOF body until it hangs at rest.
- **Fully automatic tower**: the player usually flies the ship at the same time; the booster is an autopilot
  vehicle, but simulated, so every landing comes out differently. There is no manual arm control in the game.
- **Procedural geometry** in Godot now, built from the same dimensions as the physics; a detailed model
  follows with M8.
- Ship catch and ship catch pins are out of scope (YAGNI; they come with the ship landing).

## Geometry and configuration

All dimensions are estimates [E] from public imagery and reporting unless a source is given; they live in
config, never in code. Site frame: origin at the pad centre on the ground (`height_m` of the site), axes
east/north/up (ENU), anchored in ITRF through the site's geodetic position.

`config/launch_sites.toml`, new optional table per site:

```toml
[site.tower]
azimuth_deg = 90.0            # direction from the pad centre to the tower centre, clockwise from north
distance_m = 30.0             # pad centre -> tower centre (horizontal)
width_m = 12.0                # tower cross-section (square)
height_m = 146.0              # top of the structure above the pad ground
mount_height_m = 20.0         # top of the launch mount (OLM): a stack on the pad stands here
mount_outer_radius_m = 13.0   # OLM ring (collision: annulus between inner and outer radius, up to mount_height_m)
mount_inner_radius_m = 5.5
arm_length_m = 36.0           # from the hinge at the tower face to the arm tip
arm_width_m = 2.0             # top surface the pins rest on
arm_thickness_m = 2.5
arm_hinge_spacing_m = 10.0    # horizontal distance between the two hinges on the tower face
arm_open_deg = 40.0           # each arm swung outward from "closed" (parallel arms)
arm_closed_gap_m = 9.6        # clear gap between the closed arms: hull 9 m + 0.3 m each side; pins (4.5-5.5 m) overlap the arm tops by 0.7 m
arm_rate_deg_s = 10.0         # closing rate (actuator limit)
arm_force_limit_n = 2.0e6     # closing stalls when the arms push on the hull harder than this
catch_height_m = 95.0         # arm top surface (carriage at rest) above the pad ground
carriage_mass_kg = 1.5e5      # moving mass of carriage + arms
carriage_stiffness_n_m = 2.0e6     # shock absorbers (vertical)
carriage_damping_n_s_m = 1.2e6
carriage_stroke_m = 4.0       # travel below the rest position before the hard stop
```

Booster stage, new optional table:

```toml
[stage.catch_pins]
station_m = 65.5              # [E] above the grid fins, below the hot-staging ring
count = 2
protrusion_m = 1.0            # beyond the hull radius
azimuth_deg = [90.0, 270.0]   # body frame, measured like the grid fins
```

The "closed" arm geometry places both arm top surfaces under the pins of a booster whose axis is on the
catch axis (the vertical through the pad centre) and whose pins point at the arms; the catch height is chosen
so that the caught booster's bottom clears the OLM. A stack on the pad stands on the OLM (`mount_height_m`)
when the site has a tower.

## Core: new module `contact`

Depends on `math` and `frames` only (geometry is passed in, the config loader lives in `config`).

| Unit | Responsibility |
|------|----------------|
| `contact/tower_geometry` | `TowerGeometry` from `TowerConfig` + site: tower box, OLM annulus, arm hinge points, catch axis; ENU <-> ITRF |
| `contact/primitives` | point-in-oriented-box penetration (depth, normal), capsule-vs-box and capsule-vs-annulus overlap tests |
| `contact/catch_arms` | `CatchArms` state: arm angle (rate-limited towards the command, stalls at the force limit), carriage offset and velocity (mass-spring-damper, stroke limit); arm boxes in ITRF for the current state |
| `contact/pin_contact` | penalty contact of the pin segments (3 sample points each) and of the hull near the pins against the arm boxes: normal force `k d + c d'` (no tension), regularised Coulomb friction `mu` along the relative sliding velocity; returns force and torque on the body and the reaction on the carriage |
| `contact/catch_controller` | automatic tower: closes the arms when the pins are inside the trigger window (height above the arm tops, horizontal distance of the booster axis from the catch axis, sink rate) |
| `contact/tower_collision` | booster hull (capsule along the body axis, hull radius) against the tower box and the OLM annulus: any overlap is an impact |

Contact constants that are not site geometry (pin contact stiffness, damping, friction coefficient, regularisation
speed, "at rest" thresholds) are named constants with a comment in the contact sources, tuned for a stable 200 Hz
RK4 tick: stiffness about 4e7 N/m per pin, contact period about 0.4 s for a 200 t booster.

## Integration

- `config`: `load_launch_site(config_dir, id)` returns the site (position, height) and an optional
  `TowerConfig`; the vehicle loader reads `[stage.catch_pins]`. Godot keeps reading names and coordinates for
  display but takes all tower geometry from the core (one source of truth).
- `FlightBody`: an external force and torque (body frame or GCRF, documented) applied during the tick, and a
  `caught()` state analogous to `resting()`: the body follows the tower (co-rotating with the Earth) and coasts
  analytically.
- `FlightSession`: owns the tower of its launch site (fidelity flag `towers`, default on). Each tick it computes
  the contact forces of every body near the tower from the state at the start of the tick, applies them as the
  external wrench, integrates the arms and the carriage on the same tick, runs the catch controller and the
  tower collision. Caught: both pins in contact, engines off, relative speed < 0.1 m/s and angular rate
  < 0.01 rad/s for 2 s. Impact: tower or OLM overlap, or the carriage hitting its hard stop faster than 1 m/s.
- A stack launched from a site with a tower stands on the OLM; the ascent must clear the tower (test).
- `gnc/booster_return`: with a tower, the landing burn aims at the catch point (pins 1 m above the arm tops,
  sink rate 1 m/s, upright, pins facing the arms by roll) instead of the pad; the engines cut off on the first
  pin contact. The current explicit guidance is expected to miss often (7-12 m today) - that is the baseline
  sub-project 3 improves. The roll alignment of the pins is new: the attitude controller gets a roll
  reference (it already accepts one).
- Bindings (`FlightSim`): tower geometry (once) and per-frame arm angles, carriage offset, catch state.

## Godot

- `game/world/tower_view.gd`: procedural meshes (tower box with a lattice shader, OLM ring, carriage, two arm
  boxes) from the core's geometry, placed in the site frame under the floating origin like the ground patch.
- Arm angles and carriage offset are updated every frame from the flight sim (interpolated like the vehicles).
- HUD: booster status shows "Turm: offen / schließt / gefangen / verfehlt" (German UI).

## Testing

Unit (Catch2, tag `[contact]`):
- Primitives: point-in-box depth/normal on each face; capsule-vs-box and capsule-vs-annulus overlap and
  non-overlap cases.
- Tower geometry: closed arms put both arm tops under the pins of a booster on the catch axis; open arms leave
  a gap larger than the hull plus pins; ENU/ITRF round trip at Starbase < 1 mm.
- Arms: closing rate limit, stall at the force limit, carriage spring-damper response and stroke limit.
- Pin contact: normal force direction and magnitude for a given penetration, no tension, friction opposes
  sliding and is bounded by `mu N`, torque = r x F.
- Controller: no trigger outside the window, trigger inside it.

Scenario tests (tag `[contact][catch]`, booster placed by the test, engines off, full force model):
- (a) centred, 1 m/s sink: caught; hangs at rest on both pins; position stays fixed in ITRF for 60 s.
- (b) 3 m lateral offset towards one arm: one pin misses, the booster tips off and impacts.
- (c) 8 m/s sink: the carriage hits its hard stop: impact.
- (d) lateral drift into the tower: impact.
- (e) ascent from the OLM with the tower present clears the tower; the existing ascent validation tolerances
  still hold.
- Existing pad-landing tests run with `towers` off and keep their assertions.

Integration (tag `[gnc][booster]`): full booster return to the tower for blocks 1-3; the test asserts that the
outcome is classified (caught / impact / missed) and records the miss distance, not that the catch succeeds.

Rendering: `render_check` view of the tower at Starbase with open and closed arms; read the PNG.

## Out of scope

Ship catch; tower flexibility; arm-tower collisions of the ship; wind loads (M8); detailed tower model (M8);
manual arm control.
