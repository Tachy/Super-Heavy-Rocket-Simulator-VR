# Falcon 9 Landing Legs (c2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A stage with landing legs deploys them, stands on four feet with crush-core shock absorbers, and is judged:
"gelandet", "umgekippt", "Aufprall" or "Wasserung".

**Architecture:** Two pure contact units:
- `contact/landing_surface`: a pad plane with a shoreline, or a deck over the sea.
- `contact/landing_legs`: per-foot penalty contact with a plastic crush core, and a base ring that must not touch.

`sim/leg_landing_sim` applies their wrench to every body with deployed legs each tick and judges at the end of it.
This is the pattern of `sim/catch_tower_sim`. `Vehicle` carries the leg deployment (sticky command, deploy time), and
`FlightBody` hands its ground contact to the legs once they move. A landed stage is held to the Earth as resting.

**Tech Stack:** C++20 core (`contact`, `vehicle`, `sim`), Catch2.

**Spec:** `docs/superpowers/specs/2026-10-13-falcon9-landing-design.md` (section "c2 design").

## Global Constraints

- SI units, radians, frame and unit in names; English code/docs, German UI strings; Conventional Commits.
- No physics in bindings/GDScript. Parameters from `[stage.landing_legs]` (c1 Task 1); provenance tags on values.
- Judgement texts (German): "gelandet", "umgekippt", "Aufprall", "Wasserung".

## Review Focus

1. **Coasting through the ground.** A body with deployed legs and engines off could coast a long analytic segment
   through the surface. `LegLandingSim::coast_limit_s` must force ticks near the surface. Task 3 test: a drop with
   time warp (`advance_to` far ahead in one call) still lands.
2. **Friction at rest.** Explicit regularised friction can make a standing stage creep or jitter. After landing the
   body is held fixed. Before that, a 2 m/s sideways touchdown must come to rest (Task 3 test).
3. **Legs stowed.** A stage whose legs are not deployed keeps the old bottom-point contact (gentle rests, hard is an
   impact). Task 2 test.
4. **Partially deployed legs** count as not deployed for contact. Their feet are not on the ground yet, so the base
   ring hits first: "Aufprall". Task 3 test.
5. **Session coverage.** Bodies without legs (Starship) see no change: the full ctest stays green.

---

### Task 1: Landing surface and leg contact (pure)

**Files:**
- Create: `core/include/starsim/contact/landing_surface.hpp`, `core/src/contact/landing_surface.cpp`
- Create: `core/include/starsim/contact/landing_legs.hpp`, `core/src/contact/landing_legs.cpp`
- Modify: `core/CMakeLists.txt`
- Test: `core/tests/test_landing_legs.cpp` (new)

**Interfaces:**
```cpp
// landing_surface.hpp - site frame: origin on the support surface at the site, x east, y north, z up.
struct LandingSurface {
    struct Deck {                 // a drone ship's deck at z = 0 over the sea at z = -freeboard_m
        double half_length_m = 0.0, half_width_m = 0.0;
        double heading_rad = 0.0; // long axis, clockwise from north
        double freeboard_m = 0.0;
    };
    std::optional<Deck> deck;                 // none: a ground plane at z = 0
    std::optional<config::SeaConfig> sea;     // ground plane: water beyond the shoreline (still at z = 0)
    struct Support { double height_m = 0.0; bool water = false; };
    Support below(double east_m, double north_m) const;
};
// landing_legs.hpp - vehicle frame points (station, y, z).
struct LegGeometry {
    std::vector<math::Vec3> feet;       // deployed feet
    std::vector<math::Vec3> base_ring;  // stage bottom (engine section), must not touch
    config::LandingLegConfig config;
};
LegGeometry make_leg_geometry(const config::LandingLegConfig& legs, double stage_base_station_m, double hull_radius_m);
struct LegContact {
    math::Vec3 force_n, torque_nm;  // on the body, site frame (torque about the CG)
    int feet_in_contact = 0;
    bool water = false;             // a foot or the base below the surface over water
    bool stroke_exhausted = false;
    bool base_contact = false;
    double max_foot_force_n = 0.0;
};
// `crushed_m` (one per foot) is the plastic stroke used so far; updated.
LegContact leg_contact(const LegGeometry& legs, std::vector<double>& crushed_m, const SitePose& pose,
                       const LandingSurface& surface);
```
Foot law: penetration d = support height - foot z. While d > crushed: elastic f = k (d - crushed) - c v_z (never
pulling). If f > crush force, the core crushes: crushed = d - crush/k and f = crush. Crushed > stroke means
stroke_exhausted. Friction: regularised Coulomb mu f, regularisation 0.1 m/s as in `pin_contact`.

- [ ] Tests:
  - flat-pad static load sharing: 4 feet, upright, weight split ~equally, net torque ~0
  - crush: penetration beyond crush/k keeps the force at the crush force and stores the stroke
  - stroke beyond `stroke_m` sets the flag
  - base ring below the surface sets `base_contact`
  - deck: a foot beyond the deck edge drops to the sea (no support, `water` once it is below the sea)
  - shoreline: support beyond it reports `water`
- [ ] RED, implement, GREEN; commit `feat: landing surface and crush-core leg contact`.

### Task 2: Leg deployment and the hand-over of ground contact

**Files:** `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp`,
`core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp`; test `core/tests/test_landing_legs.cpp`.

**Interfaces:**
- `VehicleCommand::deploy_legs` (bool; sticky in the vehicle: legs cannot retract).
- `double Vehicle::leg_deployment() const` (0 stowed .. 1 deployed). `bool Vehicle::has_landing_legs() const`.
  `void Vehicle::update_landing_legs(double dt_s, const VehicleCommand&)`, called in `FlightBody::step` beside the
  grid fins.
- `FlightBody::resolve_ground_contact` skips when `leg_deployment() > 0`: the leg simulation owns the contact.
- `void FlightBody::hold_landed(const frames::BodyOrientation&, time::Epoch)`: fixed to the Earth where it stands,
  `resting()` true, `caught()` false.

- [ ] Tests:
  - deployment takes `deploy_time_s` and is sticky
  - stowed legs: a 2 m/s drop rests (old contact)
  - `hold_landed` keeps the pose and co-rotates
- [ ] RED/GREEN; commit `feat: landing legs deploy and take over the ground contact`.

### Task 3: Leg landing in the session and the drop tests

**Files:**
- Create: `core/include/starsim/sim/leg_landing_sim.hpp`, `core/src/sim/leg_landing_sim.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`
- Modify: `core/include/starsim/sim/landing_outcome.hpp`, `core/src/sim/landing_outcome.cpp`
  (`LandingOutcome::TippedOver` "umgekippt")
- Test: `core/tests/test_leg_landing.cpp` (new)

**Interfaces:**
```cpp
enum class LegTouchdown { None, Landed, TippedOver, Impact, Splashdown };
const char* to_string(LegTouchdown);  // "", "gelandet", "umgekippt", "Aufprall", "Wasserung"
class LegLandingSim {
public:
    LegLandingSim(double latitude_rad, double longitude_rad, double height_m, contact::LandingSurface surface);
    void before_tick(const std::vector<vehicle::FlightBody*>&, const frames::BodyOrientation&, time::Epoch);
    void after_tick(const std::vector<vehicle::FlightBody*>&, const frames::BodyOrientation&, time::Epoch end, double dt_s);
    LegTouchdown touchdown(const vehicle::FlightBody&) const;
    bool in_contact(const vehicle::FlightBody&) const;  // any foot, last tick
    double coast_limit_s(const std::vector<vehicle::FlightBody*>&, const frames::BodyOrientation&, time::Epoch) const;
};
// FlightSession: the sim exists for the session's site (surface = plane + its sea); set_landing_surface(...) replaces
// it (drone ship, c4). LegTouchdown FlightSession::leg_touchdown(size_t index) const.
```
Judgement at the end of a tick, for a body with deployed legs:
- stroke exhausted or base contact: Impact (`mark_impacted`)
- water: Splashdown (`mark_impacted`)
- tilt > 30 deg, or the CG's ground projection outside the support polygon of the feet in contact while at least one
  foot touches: TippedOver (`mark_impacted`)
- all feet in contact, engines off, speed < 0.05 m/s, rate < 0.01 rad/s for 2 s: Landed (`hold_landed`)

- [ ] Drop tests from 5 m above the LZ-40 pad (Falcon 9 first stage, dry + 1 t, legs deployed, engines off; initial
  sink set):
  - 1 / 3 / 6 m/s upright: gelandet; 6 m/s uses stroke
  - 10 m/s: Aufprall
  - 2 m/s at 5 deg: gelandet; 25 deg: umgekippt
  - 2 m/s sink with 2 m/s sideways: gelandet
  - legs half deployed: Aufprall
  - one advance_to over 60 s: still judged
  - energy check: crush work = kinetic + potential energy lost, within 5 %
- [ ] RED/GREEN; full ctest; commit `feat: leg landing judged in the session (gelandet / umgekippt / Aufprall)`.

### Task 4: Docs
- [ ] `CLAUDE.md` (contact module list, leg landing paragraph), `config/README.md` (`[stage.landing_legs]` meaning).
  Commit `docs: landing legs (c2)`.
