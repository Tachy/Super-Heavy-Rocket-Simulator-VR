# Ship Flip and Catch (b3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** From the b2 gate (1.5 km, belly down, ~95 m/s sink) the ship relights, flips to vertical and is caught by
the Starbase tower on SCvx plans with the engine sequence 3 -> 2 -> 1.

**Architecture:** Config gives the ship three single-engine sea-level groups and catch pins. `ShipLandingGuidance`
(new, `gnc/ship_landing`) flies the flip with the attitude controller and hands over to the existing
`ScvxLandingGuidance` with ship-specific phase groups and limits. `FlightSession` switches from the entry guidance to
it at the gate and applies its `ReturnCommand` like the booster's.

**Tech Stack:** C++20 core, Catch2, existing SCvx planner (`gnc/descent_*`), tower contact (`sim/catch_tower_sim`).

**Spec:** `docs/superpowers/specs/2026-10-13-ship-return-design.md` (section "b3 design").

## Global Constraints

- SI units, radians; [S]/[E]/[D] tags on every config value; constants with sources.
- `core` has no Godot dependency; the SCvx worker never calls SPICE (existing snapshot path).
- UI strings German, code English; Conventional Commits with the session trailer.
- Booster behaviour unchanged: its campaign `[gnc][catch][validation]` stays green.

## Review Focus

1. One engine fails during the flip or burn: the remaining set must still be planned (engine failure case in the
   campaign).
2. Thrust/weight ~1.04 on the last engine: plans must not demand more than the engine has (throttle margin) - the
   campaign's mass +5 % case.
3. The gate state arrives with lateral velocity (b2 terminal phase): the flip starts from a drifting state.
4. Pins must face the arms at the catch (roll during the burn, as the booster).
5. The first plan fails: retried, not silently dropped (test with a deliberately unreachable target -> abort or
   retry visible in the command).

---

### Task 1: Ship engines 3 x 1 and catch pins

**Files:** `config/vehicles/starship_block3.toml`, `core/tests/test_flight_body.cpp` (group names), test in
`core/tests/test_ship_landing.cpp` (new).

- [ ] Step 1: test "Ship has three single sea-level engine groups and catch pins": groups `ship/sea_level_1..3`
  (one engine each, gimbaled, ring radius 1.3 m, angles 0/120/240 deg), `ship/vacuum` unchanged; catch pins: 2,
  azimuths 90/270 deg, station between 40 and 46 m.
- [ ] Step 2: RED. Step 3: config change ([E] tags); `python tools/vehicle_report.py starship_block3` unchanged
  thrust. Step 4: ctest green (ascent, burns use all groups).
- [ ] Step 5: commit `feat: ship engines as three single groups (3 -> 2 -> 1) and catch pins`.

### Task 2: Ship SCvx landing plan

**Files:** `core/include/starsim/gnc/ship_landing.hpp`, `core/src/gnc/ship_landing.cpp` (parameters first);
test `core/tests/test_ship_landing.cpp`.

**Interfaces (Produces):**
```cpp
struct ShipLandingParameters {
    std::vector<std::string> flip_groups{"ship/sea_level_1", "ship/sea_level_2", "ship/sea_level_3"};
    double flip_throttle = 0.5;                       // [E]
    double flip_tolerance_rad = 15.0 * deg;           // [E]
    double first_plan_retry_s = 1.0;
    ScvxLandingParameters scvx;                       // ship preset (phase groups, limits)
};
ShipLandingParameters ship_landing_parameters();       // the preset
```
Preset: phase groups {1,2,3}/{1,2}/{1}; limits [E]: tilt {20, 15, 8} deg, rate {0.3, 0.2, 0.1} rad/s, switch
3->2 sink window and 2->1 calm window found by planning tests, throttle_margin 0.03, terminal_thrust_margin 1.0
(no cap).

- [ ] Step 1: test "SCvx plans a ship catch from after the flip": `DescentPlanner` on `make_descent_model` of the
  ship (25 t -> after flip ~22 t propellant) from 1.2 km, 70 m/s sink, upright, over the catch axis -> converged,
  catch within 0.5 m, phase durations > 0, final phase thrust within its limits.
- [ ] Step 2: RED; Step 3: preset; tune [E] windows until the plan converges (ledger each value); Step 4: green.
- [ ] Step 5: commit `feat: ship SCvx landing preset (3 -> 2 -> 1)`.

### Task 3: Flip and landing in flight, session

**Files:** `gnc/ship_landing` (class), `sim/flight_session` (`place_landing_ship`, switch at the gate, command
application shared with the booster), test `core/tests/test_ship_landing.cpp`.

**Interfaces:**
```cpp
class ShipLandingGuidance {
public:
    ShipLandingGuidance(LandingSite site, ShipLandingParameters parameters = ship_landing_parameters());
    ReturnCommand update(const vehicle::FlightBody&, const vehicle::FlightEnvironment&, time::Epoch);
    ReturnPhase phase() const;               // Flip, LandingBurn, Landed
    void set_catch_contact(bool contact);
    void set_realtime(bool realtime);
    const ScvxLandingGuidance* scvx() const;
};
// FlightSession
void place_landing_ship(const config::VehicleConfig&, const vehicle::RigidBodyState&, time::Epoch);
const gnc::ShipLandingGuidance* ship_landing(size_t index) const;
```
- [ ] Step 1: test "Ship flips at the gate and is caught" (session, Starbase, state from the gate: belly down,
  1.5 km over the catch axis, 95 m/s sink) -> vertical within 15 deg in < 10 s, caught (tower phase Caught, pin
  offset < 0.5 m). Test "Entering ship is caught after the entry" (b2 session test extended).
- [ ] Step 2: RED; Step 3: implement; Step 4: green + ctest.
- [ ] Step 5: commit `feat: ship flip and SCvx catch at the tower`.

### Task 4: Campaign and flight comparison

**Files:** `core/tests/test_ship_catch_campaign.cpp` (new).
- [ ] Step 1: campaign `[ship][catch][validation]`: gate dispersions (+-10 m, +-10 m/s, mass +-5 %, attitude +-5 deg,
  one engine failed in the flip) -> caught within the booster's tolerances; print relight height/speed vs flights
  13/14 (~0.5-1.5 km, ~100 m/s).
- [ ] Step 2: run, tune [E] (ledger); Step 3: commit `test: ship catch campaign`.

### Task 5: Docs
- [ ] CLAUDE.md known-issues block "Ship flip and catch (b3)", config/README (groups, pins), ansichten (HUD phase
  lines if any). Commit `docs: ship flip and catch`.
