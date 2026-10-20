# ISS i6: missions, launch window and game interface - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Three ISS missions that the autopilot flies end to end - from the separation orbit to hard capture
(`crew_dragon_iss`), from the pad with a launch-window planner (`falcon9_crew_dragon_iss`) and from the docked state
through undocking to splashdown (`crew_dragon_iss_departure`) - plus the `FlightSim` game interface for all of it.

**Architecture:** A mission sequence in `FlightSession` (`IssMissionSequence`, its own file
`flight_session_iss_mission.cpp`, the pattern of `set_lunar_mission`) chains the existing i3-i5 machinery: ascent ->
`plan_rendezvous` -> Waypoint 0 -> `start_proximity` -> docking -> docked, or docked -> `undock` -> departure ->
`plan_capsule_return`. A new `mission/iss_launch_window` finds the liftoff epoch at which the ascent guidance's
inertial target plane (fixed at liftoff through the pad) equals the ISS plane shifted by the differential J2 node
regression, and checks the phasing schedule from the predicted separation state. The mission start functions live in
`sim/iss_mission.cpp`; `start_mission` dispatches three new `[mission] kind` values. The bindings get an ISS part of
the snapshot (`flight_snapshot_iss.cpp`) and five commands.

**Tech Stack:** C++20 core (float64, SI), Catch2, toml++ config loader (`detail::Document`), godot-cpp GDExtension,
GDScript smoke test.

**Spec:** `docs/superpowers/specs/2026-10-19-iss-docking-design.md` (section "i6 design", plus "Error handling",
"Testing", "Documentation"). Read the i3 corrections (plane bias, earliest window) and the i5 corrections (departure).
Subsystem notes: `docs/subsystems.md`, sections "Phasing (i3)", "Proximity operations (i4)", "Docking mechanism,
undocking and departure (i5)". Previous ledgers: `.superpowers/sdd/2026-10-19-iss-i5-docking/progress.md`.

## Spec corrections proposed by this plan (need the user's approval)

1. **Launch window search over days, not "on the date":** a pad passes through a given orbit plane on a northerly
   pass once per (sidereal) day, so "the times on the date" give one candidate. The planner searches from the
   mission's earliest liftoff forward up to `search_days` (default 3 [E]) and takes the first crossing whose phasing
   closes within the mission limit (real crews scrub to the next day's instantaneous window).
2. **No out-of-plane part in phasing** (already i3 correction 4): the window aims the plane with the J2 bias to the
   boost epoch; the residual is removed by the 3D-targeted coelliptic and approach-initiation burns. The plane bias is
   iterated twice against the schedule's boost epoch.
3. **Separation state from two calibrated numbers [D]:** liftoff -> separation time and the in-plane arc from the pad
   to the separation point, measured from the `falcon9_crew_dragon` ascent in this plan (Task 5) - the window planner
   does not fly the ascent.
4. **Orbit-start mission default phase:** `crew_dragon_iss` starts at the predicted separation of the first launch
   window after its `epoch_utc` (from the launch mission it names); `phase_deg` overrides this with a start in the ISS
   plane that far behind (the i3 test start, moved into the core).
5. **Departure timed to a return opportunity:** the mission previews the first capsule-return pass at least
   `undock_to_deorbit_h` (16 h [S: Crew-9 2025-03-18, undock 05:05 UTC, deorbit burn 21:11 UTC - verify in Task 6])
   after its `epoch_utc`, starts docked 600 s [E] before the undock epoch (deorbit epoch - undock_to_deorbit) and,
   after the departure, plans the return with `lead_s` so that the planner takes that pass.
6. **Command names:** `proximity_hold`, `proximity_retreat`, `proximity_abort`, `proximity_resume`, `undock`
   (the spec's `hold` / `retreat` / `abort` / `resume` collide in meaning with `abort_landing`).

## Global Constraints

- SI units internally, angles in radians; frame and unit in every name (`position_gcrf_m`, `epoch_tdb_s`).
- No magic numbers: physical constants in `core/include/starsim/math/constants.hpp`; tuned values `[E]`, sourced `[S]`,
  derived `[D]` with a comment, in parameter structs or config.
- UI / status strings German (UTF-8 sources), code, comments, commit messages English, Conventional Commits.
- `core` has no Godot dependency; bindings stay thin (no physics); physics never in GDScript.
- Missions are configuration (`config/missions/*.toml`), not code.
- `iss_orbit = "oem"` without a usable OEM window, a malformed OEM, a missing port id: a config error with a German
  message, no silent fallback (spec "Error handling").
- A phasing plan / launch window that does not close: refused with the planner's German reason; the mission does not
  start (spec).
- A reboost inside the mission window: a warning in the mission result (spec).
- Target: a mission start (planning) of a few seconds, as the lunar missions (spec "Performance").
- Tests: tags `[iss]`, `[rendezvous]`, `[docking]`, new `[iss_mission]`, `[launch_window]`; `[slow]` above ~40 s;
  tests must not share temp files. Run only the affected tags (CLAUDE.md "Test workflow").
- Work on `main`, commit and `git push origin main` after every task once its tests are green.

## Review Focus

1. **The mission epoch outside every OEM window with `iss_orbit = "oem"`** - a person picks a date next month: the
   mission must be refused with the German `iss_start` message, not start on a synthetic orbit (Task 4 test).
2. **No feasible launch window within the search** (phase never closes inside a short mission limit): `start_mission`
   reports the German reason and leaves the session without a launched vehicle (Task 1 unit + Task 5 test).
3. **Proximity commands outside a proximity flight** (during the phasing coast, while docked): refused with a German
   message, the mission sequence keeps running (Task 3 test, Task 7 smoke test).
4. **A proximity abort or a collision during a sequenced mission:** the sequence must not restart the approach on
   its own; abort waits for `proximity_resume`, a collision ends the mission ("Mission beendet: Kollision")
   (Task 3 test).
5. **Coast past a sequence transition at high time warp** (Waypoint 0 reached, undock epoch due): the session must
   stop its coast there and act on the next tick (Task 3 test with a single large `advance_to`).

---

## File structure

| File | Responsibility |
|---|---|
| `core/include/starsim/gnc/ascent_guidance.hpp`, `core/src/gnc/ascent_guidance.cpp` | modify: free `ascent_plane_normal` (used by `AscentGuidance::start` and the window planner) |
| `core/include/starsim/mission/iss_launch_window.hpp`, `core/src/mission/iss_launch_window.cpp` | create: plane bias, separation-state estimate, launch-window search |
| `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp` | modify: three mission kinds and their loaders |
| `config/missions/crew_dragon_iss.toml`, `falcon9_crew_dragon_iss.toml`, `crew_dragon_iss_departure.toml` | create |
| `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session_iss_mission.cpp` | modify / create: `IssMissionSequence`, legs, tick hook, coast cap |
| `core/src/sim/flight_session_docking.cpp`, `core/include/starsim/sim/docking_sim.hpp`, `core/src/sim/docking_sim.cpp`, `core/include/starsim/contact/docking_mechanism.hpp`, `core/src/contact/docking_mechanism.cpp` | modify: `dock_in_place` (a docked scenario start) |
| `core/src/sim/flight_session_capsule.cpp` | modify: `capsule_return_preview` (plan without committing) |
| `core/include/starsim/sim/iss_mission.hpp`, `core/src/sim/iss_mission.cpp` | create: start functions of the three missions |
| `core/include/starsim/sim/mission_start.hpp`, `core/src/sim/mission_start.cpp` | modify: dispatch, warnings |
| `bindings/src/flight_snapshot_iss.cpp`, `flight_snapshot.hpp/.cpp`, `flight_sim.hpp/.cpp`, `bindings/CMakeLists.txt` | create / modify: getters + commands |
| `game/tests/smoke_test.gd` | modify: ISS checks |
| `core/tests/test_iss_launch_window.cpp`, `test_iss_mission_config.cpp`, `test_iss_mission_sequence.cpp`, `test_iss_mission_campaign.cpp` | create |
| `docs/subsystems.md`, spec, `CLAUDE.md`, `config/README.md`, `docs/ansichten.md` | modify (Task 8) |

---

### Task 1: Launch-window planner

**Files:**
- Modify: `core/include/starsim/gnc/ascent_guidance.hpp`, `core/src/gnc/ascent_guidance.cpp:91-101`
- Create: `core/include/starsim/mission/iss_launch_window.hpp`, `core/src/mission/iss_launch_window.cpp`
- Modify: `core/CMakeLists.txt` (add `src/mission/iss_launch_window.cpp` beside `src/mission/rendezvous_geometry.cpp`)
- Modify: `core/tests/rendezvous_test_support.hpp` (use `mission::plane_bias_rad`)
- Test: `core/tests/test_iss_launch_window.cpp` (add to `core/tests/CMakeLists.txt` beside `test_docking_campaign.cpp`)

**Interfaces:**
- Consumes: `orbit::j2_secular_rates` (`orbit/synthetic_station_orbit.hpp`), `mission::phasing_schedule`,
  `mission::PhasingParameters` / `PhasingSchedule` (`mission/phasing_schedule.hpp`), `frames::geodetic_to_fixed`,
  `frames::BodyOrientation::fixed_to_gcrf`.
- Produces:
  ```cpp
  namespace starsim::gnc {
  // The inertial target plane AscentGuidance fixes at liftoff: through the Earth's centre, containing the site (local
  // geodetic up) and the inertial launch direction of `inclination_rad` (northerly: towards the ascending node).
  math::Vec3 ascent_plane_normal(const math::Vec3& position_gcrf_m, const frames::BodyOrientation& earth,
                                 time::Epoch epoch, double inclination_rad, bool northerly);
  }
  namespace starsim::mission {
  double plane_bias_rad(double chaser_semi_major_axis_m, double station_semi_major_axis_m, double inclination_rad,
                        double match_s);
  struct LaunchWindowParameters { ... };   // below
  struct LaunchWindow { ... };             // below
  using StationStateAt = std::function<math::StateVector(time::Epoch)>;
  math::StateVector separation_state_estimate(const math::Vec3& pad_gcrf_m, const math::Vec3& plane_normal_gcrf,
                                              const LaunchWindowParameters& p, double gm_m3ps2);
  LaunchWindow plan_launch_window(const math::Vec3& pad_itrf_m, const frames::BodyOrientation& earth,
                                  const StationStateAt& station, time::Epoch earliest,
                                  const LaunchWindowParameters& p, double gm_m3ps2);
  }
  ```

- [ ] **Step 1: Factor `ascent_plane_normal` out of `AscentGuidance::start`**

In `ascent_guidance.hpp` after `launch_azimuth_rad`'s declaration add the declaration above (with
`#include "starsim/frames/body_orientation.hpp"` already there for `start`). In `ascent_guidance.cpp` add after
`launch_azimuth_rad`:

```cpp
math::Vec3 ascent_plane_normal(const Vec3& position_gcrf_m, const frames::BodyOrientation& earth, time::Epoch epoch,
                               double inclination_rad, bool northerly) {
    const auto frame = local_frame(position_gcrf_m, earth, epoch);
    const double s = std::clamp(std::cos(inclination_rad) / std::cos(frame.latitude_rad), -1.0, 1.0);
    const double inertial = northerly ? std::asin(s) : constants::pi - std::asin(s);
    const Vec3 heading = std::sin(inertial) * frame.east + std::cos(inertial) * frame.north;
    return math::normalized(cross(frame.up, heading));
}
```

and replace the five lines in `AscentGuidance::start` from `// Target plane:` to `plane_normal_ = ...` with

```cpp
    plane_normal_ = ascent_plane_normal(body.state().position_gcrf_m, earth, epoch, target_.inclination_rad,
                                        target_.northerly);
```

Run `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[ascent]"`. Expected: all pass
(pure refactor).

- [ ] **Step 2: Write the failing tests** (`core/tests/test_iss_launch_window.cpp`)

```cpp
// ISS i6: the launch-window planner - plane bias, separation-state estimate, the liftoff at which the ascent plane is
// the biased ISS plane, phase feasibility and the refusal.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

#include "rendezvous_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/mission/iss_launch_window.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
constexpr double gm = constants::earth_gm_m3ps2;

struct Fixture {
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    const orbit::OemArchive archive = orbit::OemArchive::scan(rendezvous_test::fixture_dir, clock);
    const config::VehicleConfig iss = config::load_vehicle(rendezvous_test::config_dir, "iss");
    const config::LaunchSiteConfig site = config::load_launch_site(rendezvous_test::config_dir, "ccsfs_slc40");
    Vec3 pad_itrf() const {
        return frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    }
    mission::StationStateAt station() const {
        return [this](time::Epoch t) {
            return sim::iss_start(iss, archive, clock, earth, t, sim::IssOrbitSource::Oem).state_gcrf;
        };
    }
    mission::LaunchWindowParameters parameters() const {
        mission::LaunchWindowParameters p;
        p.inclination_rad = 51.64 * deg;
        p.insertion_s = 720.0;
        p.insertion_arc_rad = 30.0 * deg;
        return p;
    }
};
}  // namespace

TEST_CASE("Launch window: the plane bias makes both nodes meet after the match time", "[iss][launch_window]") {
    const double a_c = constants::wgs84_semi_major_axis_m + 205e3, a_s = constants::wgs84_semi_major_axis_m + 415e3;
    const double i = 51.64 * deg;
    const double day = 86400.0;
    const double bias = mission::plane_bias_rad(a_c, a_s, i, day);
    // ~0.58 deg/day faster regression at 205 km (i3 correction 4): the chaser starts east of the station's node
    CHECK(bias / deg == Catch::Approx(0.58).margin(0.05));
    CHECK(mission::plane_bias_rad(a_c, a_s, i, 0.0) == 0.0);
}

TEST_CASE("Launch window: separation state in the plane, at the arc, on the 200 x 210 km orbit",
          "[iss][launch_window]") {
    mission::LaunchWindowParameters p;
    p.insertion_arc_rad = 30.0 * deg;
    const Vec3 pad{6.0e6, 0.0, 0.0};
    const Vec3 n = math::normalized(Vec3{0.0, -std::sin(51.6 * deg), std::cos(51.6 * deg)});
    const auto s = mission::separation_state_estimate(pad, n, p, gm);
    CHECK(std::abs(dot(s.position_m, n)) < 1e-6);
    CHECK(std::abs(dot(s.velocity_mps, n)) < 1e-9);
    CHECK(std::acos(dot(math::normalized(s.position_m), math::normalized(pad))) / deg == Catch::Approx(30.0));
    CHECK(norm(s.position_m) == Catch::Approx(p.separation_periapsis_radius_m));
    CHECK(dot(cross(s.position_m, s.velocity_mps), n) > 0.0);  // prograde about the plane normal
    const auto e = orbit::elements_from_state(s, gm);
    CHECK((e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::wgs84_semi_major_axis_m) / 1000.0 ==
          Catch::Approx(210.0).margin(0.01));
}

TEST_CASE("Launch window: at liftoff the ascent plane is the biased ISS plane", "[iss][launch_window]") {
    test::spice_session();
    const Fixture f;
    const time::Epoch earliest = f.clock.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto w = mission::plan_launch_window(f.pad_itrf(), f.earth, f.station(), earliest, f.parameters(), gm);
    INFO(w.reason);
    REQUIRE(w.feasible);
    CHECK(w.liftoff - earliest >= 0.0);
    CHECK(w.liftoff - earliest < 86400.0 * 1.01);  // one northerly crossing per sidereal day
    const Vec3 pad = f.earth.fixed_to_gcrf(w.liftoff).position(f.pad_itrf());
    const Vec3 n_ascent = gnc::ascent_plane_normal(pad, f.earth, w.liftoff, 51.64 * deg, true);
    // the target plane: the ISS plane at liftoff turned by the bias about the Earth's pole
    const auto iss = f.station()(w.liftoff);
    const Vec3 pole = f.earth.fixed_to_gcrf(w.liftoff).rotation * Vec3{0.0, 0.0, 1.0};
    const Vec3 n_iss = math::normalized(cross(iss.position_m, iss.velocity_mps));
    const Vec3 n_target = math::Quat::from_axis_angle(pole, w.plane_bias_rad).rotate(n_iss);
    CHECK(std::acos(std::clamp(dot(n_ascent, n_target), -1.0, 1.0)) / deg < 0.01);
    CHECK(w.plane_bias_rad > 0.0);
    CHECK(w.separation - w.liftoff == Catch::Approx(720.0));
    CHECK(w.schedule.feasible);
    CHECK(w.schedule.waypoint0_epoch - w.liftoff < 28.0 * 3600.0);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("[launch_window] liftoff +%.3f h, bias %.3f deg, Waypoint 0 %.1f h after liftoff, rejected %d\n",
                    (w.liftoff - earliest) / 3600.0, w.plane_bias_rad / deg,
                    (w.schedule.waypoint0_epoch - w.liftoff) / 3600.0, w.rejected);
    }
}

TEST_CASE("Launch window: a phase that never closes in a short limit is refused in German", "[iss][launch_window]") {
    test::spice_session();
    const Fixture f;
    auto p = f.parameters();
    p.phasing.mission_limit_s = 2.0 * 3600.0;  // only a phase of a few degrees would close: no window in a day
    p.search_s = 86400.0;
    const auto w = mission::plan_launch_window(f.pad_itrf(), f.earth, f.station(),
                                               f.clock.from_utc({2026, 10, 16, 0, 0, 0.0}), p, gm);
    CHECK_FALSE(w.feasible);
    CHECK(w.reason.find("Kein Startfenster") != std::string::npos);
    CHECK(w.rejected >= 1);
}
```

- [ ] **Step 3: Run to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[launch_window]"`
Expected: compile error, `starsim/mission/iss_launch_window.hpp` not found.

- [ ] **Step 4: Write the header** `core/include/starsim/mission/iss_launch_window.hpp`

```cpp
#pragma once

#include <functional>
#include <string>

#include "starsim/frames/body_orientation.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/mission/phasing_schedule.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

// RAAN shift (about the Earth's pole) of a separation orbit so that its plane lies on the station's after `match_s`:
// the J2 node regression is faster at the lower orbit (~0.58 deg/day at 205 vs 415 km, i3 correction 4). > 0: east.
double plane_bias_rad(double chaser_semi_major_axis_m, double station_semi_major_axis_m, double inclination_rad,
                      double match_s);

struct LaunchWindowParameters {
    double inclination_rad = 51.64 * constants::deg_to_rad;  // the ascent target (the ISS's)
    bool northerly = true;
    double insertion_s = 0.0;        // [D] liftoff -> payload separation (calibrated from the ascent, Task 5)
    double insertion_arc_rad = 0.0;  // [D] inertial in-plane angle from the pad at liftoff to the separation point
    // [S] the separation orbit 200 x 210 km (falcon9_crew_dragon); the separation at its periapsis [E]
    double separation_periapsis_radius_m = constants::wgs84_semi_major_axis_m + 200e3;
    double separation_apoapsis_radius_m = constants::wgs84_semi_major_axis_m + 210e3;
    double search_s = 3.0 * 86400.0;  // [E] from the earliest liftoff (spec correction 1)
    double sample_s = 600.0;          // [E] node-difference sampling (the Earth turns 2.5 deg in it)
    int bias_iterations = 2;          // [E] plane bias re-aimed at the schedule's boost epoch
    PhasingParameters phasing;        // its mission limit counts from the separation
};

struct LaunchWindow {
    bool feasible = false;
    std::string reason;  // German when refused
    time::Epoch liftoff, separation;
    math::StateVector separation_state_gcrf;  // predicted
    double plane_bias_rad = 0.0;
    PhasingSchedule schedule;  // from the predicted separation state
    int rejected = 0;          // earlier crossings whose phasing did not close
};

// The station's GCRF state at an epoch (OEM or synthetic orbit, sim::iss_start).
using StationStateAt = std::function<math::StateVector(time::Epoch)>;

// The separation state: on the plane `plane_normal_gcrf` (the ascent's, fixed at liftoff), the pad's direction
// projected into it and turned by insertion_arc_rad prograde, at the separation periapsis with the periapsis speed.
math::StateVector separation_state_estimate(const math::Vec3& pad_gcrf_m, const math::Vec3& plane_normal_gcrf,
                                            const LaunchWindowParameters& p, double gm_m3ps2);

// The first liftoff at or after `earliest` (within search_s) at which the ascent guidance's target plane
// (gnc::ascent_plane_normal through the pad) is the station's plane turned by plane_bias_rad (match at the schedule's
// boost), and whose predicted separation state gives a feasible phasing_schedule. Refused: "Kein Startfenster in
// <n> Tagen: <reason of the last schedule>".
LaunchWindow plan_launch_window(const math::Vec3& pad_itrf_m, const frames::BodyOrientation& earth,
                                const StationStateAt& station, time::Epoch earliest, const LaunchWindowParameters& p,
                                double gm_m3ps2);

}  // namespace starsim::mission
```

- [ ] **Step 5: Write the implementation** `core/src/mission/iss_launch_window.cpp`

```cpp
#include "starsim/mission/iss_launch_window.hpp"

#include <cmath>
#include <cstdio>

#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"

namespace starsim::mission {

namespace {

using math::Vec3;

double raan_rate_radps(double a_m, double inclination_rad) {
    orbit::StationMeanElements mean;
    mean.semi_major_axis_m = a_m;
    mean.inclination_rad = inclination_rad;
    return orbit::j2_secular_rates(mean, constants::earth_gm_m3ps2, constants::earth_j2,
                                   environment::earth_j2_reference_radius_m).raan_radps;
}

Vec3 pole_gcrf(const frames::BodyOrientation& earth, time::Epoch t) {
    return earth.fixed_to_gcrf(t).rotation * Vec3{0.0, 0.0, 1.0};
}

// Node of the ascent plane minus node of the biased station plane, about the pole, wrapped to (-pi, pi]; it grows with
// the Earth's rotation (~7.3e-5 rad/s), its upward zero is the window.
double node_difference_rad(const Vec3& pad_itrf_m, const frames::BodyOrientation& earth, const StationStateAt& station,
                           time::Epoch t, const LaunchWindowParameters& p, double bias_rad) {
    const Vec3 pole = pole_gcrf(earth, t);
    const Vec3 pad = earth.fixed_to_gcrf(t).position(pad_itrf_m);
    const Vec3 n_ascent = gnc::ascent_plane_normal(pad, earth, t, p.inclination_rad, p.northerly);
    const auto s = station(t);
    const Vec3 n_target = math::Quat::from_axis_angle(pole, bias_rad)
                              .rotate(math::normalized(cross(s.position_m, s.velocity_mps)));
    const Vec3 node_a = cross(pole, n_ascent), node_t = cross(pole, n_target);
    return std::atan2(dot(cross(node_t, node_a), pole), dot(node_t, node_a));
}

}  // namespace

double plane_bias_rad(double chaser_semi_major_axis_m, double station_semi_major_axis_m, double inclination_rad,
                      double match_s) {
    return (raan_rate_radps(station_semi_major_axis_m, inclination_rad) -
            raan_rate_radps(chaser_semi_major_axis_m, inclination_rad)) * match_s;
}

math::StateVector separation_state_estimate(const Vec3& pad_gcrf_m, const Vec3& plane_normal_gcrf,
                                            const LaunchWindowParameters& p, double gm_m3ps2) {
    const Vec3 n = math::normalized(plane_normal_gcrf);
    const Vec3 in_plane = math::normalized(pad_gcrf_m - dot(pad_gcrf_m, n) * n);
    const Vec3 u = math::Quat::from_axis_angle(n, p.insertion_arc_rad).rotate(in_plane);
    const double rp = p.separation_periapsis_radius_m, ra = p.separation_apoapsis_radius_m;
    const double speed = std::sqrt(gm_m3ps2 * (2.0 / rp - 2.0 / (rp + ra)));  // vis-viva at periapsis
    return {rp * u, speed * cross(n, u)};
}

LaunchWindow plan_launch_window(const Vec3& pad_itrf_m, const frames::BodyOrientation& earth,
                                const StationStateAt& station, time::Epoch earliest, const LaunchWindowParameters& p,
                                double gm_m3ps2) {
    LaunchWindow w;
    const double a_chaser = 0.5 * (p.separation_periapsis_radius_m + p.separation_apoapsis_radius_m);
    const auto s0 = station(earliest);
    const double a_station = orbit::elements_from_state(s0, gm_m3ps2).semi_major_axis_m;
    PhasingParameters phasing = p.phasing;
    std::string last_reason = "keine Ebenenquerung";
    // the first bias guess: plane match half way through the mission limit
    double match_s = 0.5 * phasing.mission_limit_s;
    time::Epoch t = earliest;
    while (t - earliest < p.search_s) {
        double bias = plane_bias_rad(a_chaser, a_station, p.inclination_rad, match_s);
        // the next upward zero of the node difference after t: sample, then bisect
        double f0 = node_difference_rad(pad_itrf_m, earth, station, t, p, bias);
        time::Epoch t1 = t;
        bool found = false;
        while (t1 - earliest < p.search_s) {
            const time::Epoch t2 = t1 + p.sample_s;
            const double f2 = node_difference_rad(pad_itrf_m, earth, station, t2, p, bias);
            if (f0 <= 0.0 && f2 > 0.0 && f2 - f0 < 1.0) {  // not the +-pi wrap
                found = true;
                break;
            }
            t1 = t2;
            f0 = f2;
        }
        if (!found) break;
        time::Epoch root{};
        for (int iteration = 0; iteration < p.bias_iterations; ++iteration) {
            time::Epoch lo = t1, hi = t1 + p.sample_s;
            for (int k = 0; k < 40; ++k) {  // 600 s / 2^40: far below a microsecond
                const time::Epoch mid = lo + 0.5 * (hi - lo);
                (node_difference_rad(pad_itrf_m, earth, station, mid, p, bias) > 0.0 ? hi : lo) = mid;
            }
            root = lo + 0.5 * (hi - lo);
            const Vec3 pad = earth.fixed_to_gcrf(root).position(pad_itrf_m);
            const Vec3 n = gnc::ascent_plane_normal(pad, earth, root, p.inclination_rad, p.northerly);
            w.liftoff = root;
            w.separation = root + p.insertion_s;
            w.separation_state_gcrf = separation_state_estimate(pad, n, p, gm_m3ps2);
            w.plane_bias_rad = bias;
            w.schedule = phasing_schedule(w.separation_state_gcrf, station(w.separation), w.separation, phasing,
                                          gm_m3ps2);
            if (!w.schedule.feasible) break;
            match_s = w.schedule.boost_epoch - w.separation;  // re-aim the bias at the boost (spec correction 2)
            bias = plane_bias_rad(a_chaser, a_station, p.inclination_rad, match_s);
        }
        if (w.schedule.feasible) {
            w.feasible = true;
            return w;
        }
        last_reason = w.schedule.reason;
        ++w.rejected;
        t = root + 3600.0;  // past this crossing: the next one is ~a day later
    }
    char text[160];
    std::snprintf(text, sizeof text, "Kein Startfenster in %.0f Tagen: ", p.search_s / 86400.0);
    w.feasible = false;
    w.reason = text + last_reason;
    return w;
}

}  // namespace starsim::mission
```

Note on the bias loop: the last iteration's bias is the one stored in `w.plane_bias_rad` and used for `root`; the
re-aimed bias after the last iteration is only used when `bias_iterations` allows another bisection. With the default 2
the second bisection uses the boost-aimed bias. The bisection bracket `[t1, t1 + sample_s]` stays valid because the
bias changes by well under 2.5 deg between iterations (check: STARSIM_VERBOSE output; if the root leaves the bracket,
widen it to `[t1 - sample_s, t1 + 2 sample_s]`).

- [ ] **Step 6: Replace the test-local bias in `rendezvous_test_support.hpp`**

Replace the body of `raan_rate` usage in `start` by `mission::plane_bias_rad(e.semi_major_axis_m,
e_iss.semi_major_axis_m, e.inclination_rad, plane_match_s)` (include `starsim/mission/iss_launch_window.hpp`) and
delete the now-unused `raan_rate` helper. The values are the same function, so `[rendezvous]` results are unchanged.

- [ ] **Step 7: Run the tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[launch_window],[ascent]"`
then, as `rendezvous_test_support.hpp` changed, `-TestFilter "[rendezvous]"` (its `[slow]` campaigns included).
Expected: PASS. With `STARSIM_VERBOSE=1` note the liftoff, bias and Waypoint-0 time in the ledger.

- [ ] **Step 8: Commit and push**

```bash
git add core/include/starsim/gnc/ascent_guidance.hpp core/src/gnc/ascent_guidance.cpp core/include/starsim/mission/iss_launch_window.hpp core/src/mission/iss_launch_window.cpp core/CMakeLists.txt core/tests/CMakeLists.txt core/tests/test_iss_launch_window.cpp core/tests/rendezvous_test_support.hpp
git commit -m "feat(mission): ISS launch window - ascent plane through the pad on the J2-biased station plane, phasing check"
git push origin main
```

---

### Task 2: Mission configuration (three kinds)

**Files:**
- Modify: `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp`
- Create: `config/missions/crew_dragon_iss.toml`, `config/missions/falcon9_crew_dragon_iss.toml`,
  `config/missions/crew_dragon_iss_departure.toml`
- Test: `core/tests/test_iss_mission_config.cpp` (register in `core/tests/CMakeLists.txt`)

**Interfaces:**
- Produces:
  ```cpp
  enum class MissionKind { Ascent, ShipReturn, CapsuleReturn, LunarLanding, LunarReturn,
                           IssRendezvous, IssLaunch, IssDeparture };
  struct IssLaunchWindowConfig { double insertion_s; double insertion_arc_rad; double search_s; };
  struct IssLaunchMission { std::string id, name, ascent_id, station_id, port, iss_orbit;
                            time::UtcDateTime earliest_utc; IssLaunchWindowConfig window;
                            double mission_limit_s; double earliest_arrival_s; };
  struct IssRendezvousMission { std::string id, name, vehicle_id, station_id, port, iss_orbit, launch_id;
                                time::UtcDateTime epoch_utc; std::optional<double> phase_rad;
                                std::optional<double> iss_argument_of_latitude_rad;
                                double mission_limit_s; double earliest_arrival_s; };
  struct IssDepartureMission { std::string id, name, vehicle_id, station_id, port, iss_orbit, return_id;
                               time::UtcDateTime epoch_utc; double rcs_propellant_kg;
                               double undock_to_deorbit_s; double search_window_s;
                               std::optional<double> iss_argument_of_latitude_rad; };
  IssLaunchMission load_iss_launch_mission(const std::filesystem::path&, std::string_view);
  IssRendezvousMission load_iss_rendezvous_mission(const std::filesystem::path&, std::string_view);
  IssDepartureMission load_iss_departure_mission(const std::filesystem::path&, std::string_view);
  ```
  `kind` strings: `"iss_rendezvous"`, `"iss_launch"`, `"iss_departure"`. `iss_orbit` stays a string here
  (`sim::parse_iss_orbit_source` validates it in Task 4; the loader checks it is one of `oem|synthetic|auto` so the
  error names the file).

- [ ] **Step 1: Write the mission files**

`config/missions/crew_dragon_iss.toml`:
```toml
# Crew Dragon from its separation orbit to the ISS (ISS i6): phasing (i3) -> Waypoint 0 -> proximity operations (i4)
# -> soft and hard capture (i5). The start is the predicted separation of the first launch window of `launch` after
# epoch_utc (spec correction 4); phase_deg instead starts in the ISS plane that far behind the station.

[mission]
name = "Crew Dragon zur ISS"
kind = "iss_rendezvous"
vehicle = "crew_dragon"
station = "iss"
port = "ida2"                       # "ida2" (Harmony vorn, V-bar) or "ida3" (Harmony oben, R-bar)
iss_orbit = "auto"                  # "oem" | "synthetic" | "auto" (fidelity flag)

[start]
epoch_utc = "2026-10-16T00:00:00"   # inside the committed OEM fixture (2026-10-14 ..)
launch = "falcon9_crew_dragon_iss"  # the launch whose separation state starts the mission
# phase_deg = 30.0                  # optional: Dragon this far behind the ISS at epoch_utc, in its biased plane
# iss_argument_of_latitude_deg = 0  # optional, synthetic orbit only: the ISS position in its orbit

[phasing]
mission_limit_h = 28.0              # [S] crew missions dock 24-28 h after the launch
earliest_arrival_h = 0.0            # 0: the earliest window (cheapest, i3 correction 3)
```

`config/missions/falcon9_crew_dragon_iss.toml`:
```toml
# Falcon 9 with Crew Dragon from SLC-40 to the ISS (ISS i6): the launch-window planner picks the liftoff at which the
# ascent's target plane is the ISS plane shifted by the J2 bias and the phasing closes in the mission limit; the
# ascent and separation of `ascent` follow, then the Dragon flies phasing, proximity operations and docking.

[mission]
name = "Falcon 9 mit Crew Dragon zur ISS"
kind = "iss_launch"
ascent = "falcon9_crew_dragon"      # vehicle, site, pitch program, payload separation, separation orbit
station = "iss"
port = "ida2"
iss_orbit = "auto"

[launch_window]
earliest_utc = "2026-10-16T00:00:00"
search_days = 3.0                   # [E] spec correction 1
insertion_s = 720.0                 # [D] liftoff -> separation, from the ascent (Task 5 calibration)
insertion_arc_deg = 30.0            # [D] pad at liftoff -> separation point, inertial in-plane angle (Task 5)

[phasing]
mission_limit_h = 28.0              # [S] from the launch
earliest_arrival_h = 0.0
```

`config/missions/crew_dragon_iss_departure.toml`:
```toml
# Crew Dragon undocks from the ISS and returns (ISS i6): the mission previews the first capsule-return pass at least
# undock_to_deorbit_h after epoch_utc, starts docked 600 s before the undocking, undocks (i5: hooks, springs, undock
# burn, departure out of the approach ellipsoid) and hands over to the capsule return of `return` (deorbit, entry,
# parachutes, splashdown).

[mission]
name = "Crew Dragon: Abkoppeln und Rückkehr"
kind = "iss_departure"
vehicle = "crew_dragon"
station = "iss"
port = "ida2"
iss_orbit = "auto"
return = "crew_dragon_return"       # splashdown zone and deorbit settings

[start]
epoch_utc = "2026-10-16T00:00:00"   # the earliest undocking
rcs_propellant_kg = 2000.0          # [E] after the ISS mission (as crew_dragon_return)
undock_to_deorbit_h = 16.1          # [S: Crew-9 2025-03-18, undock 05:05 UTC, deorbit burn 21:11 UTC]
search_window_h = 168.0             # [E] a week of passes (the 48 h of crew_dragon_return found none for some planes)
```

- [ ] **Step 2: Write the failing loader tests** (`core/tests/test_iss_mission_config.cpp`)

```cpp
// ISS i6: the three ISS mission kinds load from config/missions and refuse bad values with the file named.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>

#include "starsim/config/mission_config.hpp"

using namespace starsim;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
constexpr double deg = constants::deg_to_rad;

// A copy of config/ with one mission file replaced (unique directory per test case: ctest runs in parallel).
std::filesystem::path config_with(const std::string& tag, const std::string& id, const std::string& text) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_iss_cfg_" + tag);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "missions");
    std::ofstream(dir / "missions" / (id + ".toml")) << text;
    return dir;
}
}  // namespace

TEST_CASE("ISS mission config: the three kinds load", "[iss][iss_mission][config]") {
    CHECK(config::mission_kind(config_dir, "crew_dragon_iss") == config::MissionKind::IssRendezvous);
    CHECK(config::mission_kind(config_dir, "falcon9_crew_dragon_iss") == config::MissionKind::IssLaunch);
    CHECK(config::mission_kind(config_dir, "crew_dragon_iss_departure") == config::MissionKind::IssDeparture);

    const auto r = config::load_iss_rendezvous_mission(config_dir, "crew_dragon_iss");
    CHECK(r.vehicle_id == "crew_dragon");
    CHECK(r.station_id == "iss");
    CHECK(r.port == "ida2");
    CHECK(r.iss_orbit == "auto");
    CHECK(r.launch_id == "falcon9_crew_dragon_iss");
    CHECK_FALSE(r.phase_rad);
    CHECK(r.mission_limit_s == Catch::Approx(28.0 * 3600.0));

    const auto l = config::load_iss_launch_mission(config_dir, "falcon9_crew_dragon_iss");
    CHECK(l.ascent_id == "falcon9_crew_dragon");
    CHECK(l.window.search_s == Catch::Approx(3.0 * 86400.0));
    CHECK(l.window.insertion_s > 0.0);
    CHECK(l.window.insertion_arc_rad > 0.0);

    const auto d = config::load_iss_departure_mission(config_dir, "crew_dragon_iss_departure");
    CHECK(d.return_id == "crew_dragon_return");
    CHECK(d.undock_to_deorbit_s == Catch::Approx(16.1 * 3600.0));
    CHECK(d.search_window_s == Catch::Approx(168.0 * 3600.0));
    CHECK(d.rcs_propellant_kg == Catch::Approx(2000.0));
}

TEST_CASE("ISS mission config: optional phase and refusals", "[iss][iss_mission][config]") {
    const std::string base = R"([mission]
name = "x"
kind = "iss_rendezvous"
vehicle = "crew_dragon"
station = "iss"
port = "ida3"
iss_orbit = "synthetic"
[start]
epoch_utc = "2026-10-16T00:00:00"
launch = "falcon9_crew_dragon_iss"
phase_deg = 30.0
iss_argument_of_latitude_deg = 10.0
[phasing]
mission_limit_h = 28.0
earliest_arrival_h = 0.0
)";
    const auto ok = config::load_iss_rendezvous_mission(config_with("phase", "m", base), "m");
    REQUIRE(ok.phase_rad);
    CHECK(*ok.phase_rad == Catch::Approx(30.0 * deg));
    REQUIRE(ok.iss_argument_of_latitude_rad);
    CHECK(*ok.iss_argument_of_latitude_rad == Catch::Approx(10.0 * deg));

    std::string bad_orbit = base;
    bad_orbit.replace(bad_orbit.find("\"synthetic\""), 11, "\"sgp4\"");
    CHECK_THROWS(config::load_iss_rendezvous_mission(config_with("orbit", "m", bad_orbit), "m"));

    std::string no_port = base;
    no_port.erase(no_port.find("port = \"ida3\"\n"), 14);
    CHECK_THROWS(config::load_iss_rendezvous_mission(config_with("port", "m", no_port), "m"));

    // a rendezvous file is not a launch mission
    CHECK_THROWS(config::load_iss_launch_mission(config_with("kind", "m", base), "m"));
}
```

- [ ] **Step 3: Run to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[iss_mission]"`
Expected: compile error (`IssRendezvous` not a member of `MissionKind`).

- [ ] **Step 4: Implement the loaders**

In `mission_config.hpp` extend `MissionKind` and add the three structs and loaders from **Interfaces** (with one-line
comments in the file's style; `std::optional` is already included). In `mission_config.cpp`:

`kind_of`:
```cpp
    if (kind == "iss_rendezvous") return MissionKind::IssRendezvous;
    if (kind == "iss_launch") return MissionKind::IssLaunch;
    if (kind == "iss_departure") return MissionKind::IssDeparture;
    doc.fail("mission: unknown kind '" + kind +
             "' (ascent, ship_return, capsule_return, lunar_landing, lunar_return, iss_rendezvous, iss_launch, "
             "iss_departure)");
```

Shared helpers in the anonymous namespace:
```cpp
// [mission] station / port / iss_orbit of an ISS mission.
void iss_common(const detail::Document& doc, const toml::table& mission, std::string& station, std::string& port,
                std::string& iss_orbit) {
    station = doc.text(mission, "station");
    port = doc.text(mission, "port");
    iss_orbit = doc.text(mission, "iss_orbit");
    if (iss_orbit != "oem" && iss_orbit != "synthetic" && iss_orbit != "auto") {
        doc.fail("mission: iss_orbit '" + iss_orbit + "' (oem, synthetic, auto)");
    }
}

time::UtcDateTime utc(const detail::Document& doc, const toml::table& table, const char* key) {
    const auto t = time::UtcDateTime::parse_iso(doc.text(table, key));
    if (!t) doc.fail(std::string(key) + " is not YYYY-MM-DDTHH:MM:SS");
    return *t;
}

std::optional<double> optional_deg(const toml::table& table, const char* key) {
    if (const auto v = table[key].value<double>()) return *v * constants::deg_to_rad;
    return std::nullopt;
}
```

(Check `toml_document.hpp` for the exact table type name used by `doc.text(*mission, ...)` - the existing loaders take
`*doc.root["mission"].as_table()`; use the same type for the helper parameters. If `Document` already offers an
optional-number accessor, use it instead of `optional_deg`'s raw `value<double>()`.)

`load_iss_rendezvous_mission`:
```cpp
IssRendezvousMission load_iss_rendezvous_mission(const std::filesystem::path& config_dir, std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    require_kind(doc, MissionKind::IssRendezvous, "ISS rendezvous");
    const auto* mission = doc.root["mission"].as_table();
    const auto* start = doc.root["start"].as_table();
    const auto* phasing = doc.root["phasing"].as_table();
    if (!mission || !start || !phasing) doc.fail("missing [mission], [start] or [phasing]");
    IssRendezvousMission m;
    m.id = std::string(mission_id);
    m.name = doc.text(*mission, "name");
    m.vehicle_id = doc.text(*mission, "vehicle");
    iss_common(doc, *mission, m.station_id, m.port, m.iss_orbit);
    m.epoch_utc = utc(doc, *start, "epoch_utc");
    m.launch_id = doc.text(*start, "launch");
    m.phase_rad = optional_deg(*start, "phase_deg");
    m.iss_argument_of_latitude_rad = optional_deg(*start, "iss_argument_of_latitude_deg");
    m.mission_limit_s = doc.positive(*phasing, "mission_limit_h") * 3600.0;
    m.earliest_arrival_s = doc.number(*phasing, "earliest_arrival_h") * 3600.0;
    return m;
}
```

`load_iss_launch_mission` (tables `[mission]`, `[launch_window]`, `[phasing]`): `ascent_id = doc.text(*mission,
"ascent")`, `earliest_utc = utc(doc, *window, "earliest_utc")`, `window.search_s = positive("search_days") * 86400`,
`window.insertion_s = positive("insertion_s")`, `window.insertion_arc_rad = positive("insertion_arc_deg") * deg`,
mission limit and earliest arrival as above.

`load_iss_departure_mission` (tables `[mission]`, `[start]`): `return_id = doc.text(*mission, "return")`,
`epoch_utc`, `rcs_propellant_kg = positive`, `undock_to_deorbit_s = positive("undock_to_deorbit_h") * 3600`,
`search_window_s = positive("search_window_h") * 3600`, `iss_argument_of_latitude_rad = optional_deg(*start, ...)`.

- [ ] **Step 5: Run the tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[iss_mission],[config]"`
Expected: PASS (the existing `[config]` cases prove the other kinds still load).

- [ ] **Step 6: Commit and push**

```bash
git add core/include/starsim/config/mission_config.hpp core/src/config/mission_config.cpp config/missions/crew_dragon_iss.toml config/missions/falcon9_crew_dragon_iss.toml config/missions/crew_dragon_iss_departure.toml core/tests/test_iss_mission_config.cpp core/tests/CMakeLists.txt
git commit -m "feat(config): ISS mission kinds - rendezvous from the separation orbit, launch with window, departure"
git push origin main
```

---

### Task 3: ISS mission sequence in the flight session (docking chain)

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp` (public API + private state), `core/src/sim/flight_session.cpp`
  (tick hook at line ~678 before `if (autopilot_.state != AutopilotState::Off ...`, coast cap in `coast_allowance_s`
  at line ~606, reset with the bodies)
- Create: `core/src/sim/flight_session_iss_mission.cpp` (add to `core/CMakeLists.txt` after
  `src/sim/flight_session_docking.cpp`)
- Test: `core/tests/test_iss_mission_sequence.cpp` (register)

**Interfaces:**
- Consumes: `plan_rendezvous`, `rendezvous_step()`, `start_proximity`, `proximity_phase()`, `docked()`,
  `collision_impulse_ns()`, `rendezvous_status()`, `proximity_status()`, `docking_status()`.
- Produces (public in `FlightSession`, declared after the undocking block):
  ```cpp
  // ISS mission legs (i6): Ascent until the payload is separated, Rendezvous (phasing to Waypoint 0), Proximity
  // (to contact), Docking (capture .. hooks), Docked = complete; departure: WaitUndock, Departure, Return (capsule
  // return to the water) = complete at splashdown. Ended: a refused leg, a lost vehicle or a collision.
  enum class IssMissionLeg { None, Ascent, Rendezvous, Proximity, Docking, Docked, WaitUndock, Departure, Return,
                             Complete, Ended };
  const char* to_string(IssMissionLeg leg);  // German
  struct IssMissionSequence {
      std::string station_port;              // ida2 / ida3
      mission::PhasingParameters phasing;
      gnc::ProximityParameters proximity;
      bool from_launch = false;              // starts in Ascent (else Rendezvous / WaitUndock)
      std::optional<time::Epoch> undock_epoch;           // departure: WaitUndock until then
      std::optional<gnc::DeorbitParameters> capsule_return;  // departure: planned when the departure is complete
      std::optional<time::Epoch> deorbit_epoch;          // departure: the previewed pass (lead_s aims at it)
  };
  void set_iss_mission(std::optional<IssMissionSequence> sequence);
  IssMissionLeg iss_mission_leg() const { return iss_leg_; }
  const std::string& iss_mission_status() const { return iss_status_; }  // German
  const std::optional<IssMissionSequence>& iss_mission() const { return iss_mission_; }
  static constexpr double iss_return_lead_margin_s = 1800.0;  // [E] the return is planned this far before the pass
  ```
  Private: `std::optional<IssMissionSequence> iss_mission_; IssMissionLeg iss_leg_ = IssMissionLeg::None;
  std::string iss_status_; void run_iss_mission(); bool iss_mission_pending() const; void iss_end(std::string reason);`

Leg machine (`run_iss_mission`, every tick, cheap; `iss_mission_pending()` true when the next call would change the
leg - the coast allowance is then 0):

| Leg | Condition | Action |
|---|---|---|
| Ascent | `autopilot_.state == Orbit` and the controlled vehicle has a docking port and no stacked payload | `plan_rendezvous(phasing)`: ok -> Rendezvous, else `iss_end(rendezvous_status())` |
| Rendezvous | `rendezvous_step() == Waypoint0` | `start_proximity(port, proximity)`: ok -> Proximity, else `iss_end(proximity_status())` |
| Rendezvous | `rendezvous_step() == None` (aborted) | `iss_end(rendezvous_status())` |
| Proximity | `docking_busy()` | -> Docking |
| Proximity / Docking | `collision_impulse_ns()` | `iss_end("Kollision")` |
| Docking | `docked()` | -> Docked, status "Mission abgeschlossen: angedockt an <port name>" |
| Docking | `!docking_busy()` (failed capture -> proximity retreats) | -> Proximity |
| WaitUndock | `epoch_ >= *undock_epoch` | `undock()`: ok -> Departure, else `iss_end(docking_status())` |
| Departure | `departure_phase() == Complete` | `plan_capsule_return(p)` with `p.lead_s = max(p.lead_s, *deorbit_epoch - epoch_ - iss_return_lead_margin_s)`: ok -> Return, else `iss_end(ship_return_plan()->reason)` |
| Return | `capsule_return_step() == SplashedDown` | -> Complete "Mission abgeschlossen: gewassert" |
| Return | `capsule_return_step() == Lost` | `iss_end("Kapsel verloren")` |
| any active | the controlled body impacted | `iss_end("Fahrzeug verloren")` |

`iss_end(reason)`: leg Ended, status "Mission beendet: " + reason. A proximity abort keeps the leg Proximity (the user
resumes; Review Focus 4). `set_iss_mission` sets the first leg (`from_launch` ? Ascent : `undock_epoch` ? WaitUndock :
Rendezvous) and the status ("Aufstieg", "Phasenflug", "Angedockt: Abkoppeln um ..."). The sequence is cleared with
the bodies (every `bodies_.clear()` path: `place_body`, `launch_from_pad`, `place_in_orbit`, the same places that call
`clear_station()`). Coast cap in `coast_allowance_s`, beside the station impulses:

```cpp
    if (iss_mission_pending()) return 0.0;  // a leg change is due: act on the next tick (Review Focus 5)
    if (iss_leg_ == IssMissionLeg::WaitUndock) allowance = std::min(allowance, *iss_mission_->undock_epoch - epoch_);
```

The tick hook (in `advance_to`'s tick branch, before the autopilot block, so it also runs in Off / Orbit):

```cpp
        if (iss_mission_) run_iss_mission();
```

- [ ] **Step 1: Write the failing tests** (`core/tests/test_iss_mission_sequence.cpp`)

Use the existing fixtures: `proximity_test::DragonNearIss` (i4, `proximity_test_support.hpp`) for a Waypoint-0 start
and `rendezvous_test::start` for a phasing start; `docking_test::DragonAtPort` for docked states.

```cpp
// ISS i6: the mission sequence chains phasing -> proximity -> docking (and its endings) in the flight session.

#include <catch2/catch_test_macros.hpp>

#include "docking_test_support.hpp"
#include "rendezvous_test_support.hpp"
#include "starsim/sim/flight_session.hpp"

using namespace starsim;
using sim::IssMissionLeg;

namespace {
sim::IssMissionSequence docking_to(const std::string& port) {
    sim::IssMissionSequence q;
    q.station_port = port;
    return q;
}
}  // namespace

TEST_CASE("ISS mission sequence: Waypoint 0 hands over to the proximity flight within one tick, also at time warp",
          "[iss][iss_mission][proximity]") {
    test::spice_session();
    // the Dragon at Waypoint 0 (i4 start) with a rendezvous that has arrived there
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5.0, 0.0, 396.9}, {0.012, 0.0, -0.012}));
    auto& s = d.session;
    s.set_iss_mission(docking_to("ida2"));
    // no rendezvous flight: the Rendezvous leg sees step None and would end - place it at Waypoint 0 instead
    // (FlightSession::mark_rendezvous_at_waypoint0, a test seam declared next to rendezvous_step: sets the step and
    // the status "Wegpunkt 0 erreicht" without a plan)
    s.mark_rendezvous_at_waypoint0();
    s.advance_to(d.t0 + 3600.0, 100000);  // one large call: the coast must stop at the leg change
    CHECK(s.iss_mission_leg() == IssMissionLeg::Proximity);
    CHECK(s.proximity_active());
}

TEST_CASE("ISS mission sequence: proximity -> docking -> docked completes the mission",
          "[iss][iss_mission][docking][slow]") {
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5.0, 0.0, 396.9}, {0.012, 0.0, -0.012}));
    auto& s = d.session;
    s.set_iss_mission(docking_to("ida2"));
    s.mark_rendezvous_at_waypoint0();
    for (double t = 0.0; t < 4.0 * 3600.0 && s.iss_mission_leg() != IssMissionLeg::Docked; t += 60.0) {
        s.advance_to(d.t0 + t + 60.0, 1000000);
    }
    CHECK(s.iss_mission_leg() == IssMissionLeg::Docked);
    CHECK(s.docked());
    CHECK(s.iss_mission_status().find("Mission abgeschlossen") != std::string::npos);
}

TEST_CASE("ISS mission sequence: an abort waits for resume, a collision ends the mission", "[iss][iss_mission][proximity]") {
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5.0, 0.0, 396.9}, {0.012, 0.0, -0.012}));
    auto& s = d.session;
    s.set_iss_mission(docking_to("ida2"));
    s.mark_rendezvous_at_waypoint0();
    s.advance_to(d.t0 + 10.0, 100000);
    REQUIRE(s.iss_mission_leg() == IssMissionLeg::Proximity);
    REQUIRE(s.proximity_abort());
    s.advance_to(d.t0 + 1800.0, 1000000);
    CHECK(s.iss_mission_leg() == IssMissionLeg::Proximity);  // not restarted, not ended
    CHECK(s.proximity_phase() == gnc::ProximityPhase::Abort);

    // a collision: the Dragon drifting into the modules beside the port (i5 fixture)
    auto c = docking_test::DragonAtPort::beside_modules(0.2, 0.05);
    c.session.set_iss_mission(docking_to("ida2"));
    c.session.mark_iss_leg_for_test(IssMissionLeg::Proximity);  // test seam: the leg without a flight
    REQUIRE(docking_test::DragonAtPort::wait_for(
        c.session, c.t0, [&] { return c.session.iss_mission_leg() == IssMissionLeg::Ended; }, 120.0));
    CHECK(c.session.iss_mission_status() == "Mission beendet: Kollision");
}

TEST_CASE("ISS mission sequence: a phasing flight that is refused ends the mission with the planner's reason",
          "[iss][iss_mission][rendezvous]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    sim::FlightSession s(ephemeris, earth, rendezvous_test::config_dir);
    const auto start = rendezvous_test::start(clock, earth, 30.0, 6.0 * 3600.0);
    s.place_in_orbit(config::load_vehicle(rendezvous_test::config_dir, "crew_dragon"), start.dragon, start.epoch);
    s.add_station(start.iss.vehicle, start.iss.state_gcrf, start.iss.epoch, start.iss.impulses);
    auto q = docking_to("ida2");
    q.from_launch = true;  // Ascent: the Dragon is already in orbit with the autopilot in Orbit
    q.phasing.mission_limit_s = 1800.0;  // no phasing closes in half an hour
    s.engage_orbit_hold_for_test();  // autopilot Orbit (test seam)
    s.set_iss_mission(q);
    s.advance_to(start.epoch + 10.0, 10000);
    CHECK(s.iss_mission_leg() == IssMissionLeg::Ended);
    CHECK(s.iss_mission_status().rfind("Mission beendet: ", 0) == 0);
}

TEST_CASE("ISS mission sequence: cleared with the bodies", "[iss][iss_mission]") {
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5.0, 0.0, 396.9}, {0.0, 0.0, 0.0}));
    d.session.set_iss_mission(docking_to("ida3"));
    const auto& dragon = d.session.body(0).state();
    d.session.place_in_orbit(config::load_vehicle(proximity_test::config_dir, "crew_dragon"),
                             {dragon.position_gcrf_m, dragon.velocity_gcrf_mps}, d.session.epoch());
    CHECK(d.session.iss_mission_leg() == IssMissionLeg::None);
    CHECK_FALSE(d.session.iss_mission());
}
```

Test seams (public, commented "test seam"): `mark_rendezvous_at_waypoint0()` (sets `rendezvous_step_ = Waypoint0`,
status "Wegpunkt 0 erreicht"), `mark_iss_leg_for_test(IssMissionLeg)`, `engage_orbit_hold_for_test()` (sets
`autopilot_.state = AutopilotState::Orbit`). Before adding a seam, check whether the fixtures already reach that state
through public calls (e.g. a `place_in_orbit` that leaves the autopilot in Orbit); prefer the public path and drop the
seam when one exists. (`body(0)` is the Dragon in `DragonNearIss` - check the fixture's body order.)

- [ ] **Step 2: Run to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[iss_mission]"`
Expected: compile error (`set_iss_mission` unknown).

- [ ] **Step 3: Implement** the API, the leg machine of the table, the tick hook, the coast cap and the reset in
`flight_session_iss_mission.cpp` / `flight_session.cpp` as specified above. `to_string(IssMissionLeg)`: "keine",
"Aufstieg", "Phasenflug", "Annäherung", "Andocken", "Angedockt", "Warten auf Abkoppeln", "Abflug", "Rückkehr",
"Abgeschlossen", "Beendet". `run_iss_mission` for Rendezvous also copies `rendezvous_status()` into the status
while the leg runs ("Phasenflug: <status>"), Proximity `proximity_status()`, Docking `docking_status()`.

- [ ] **Step 4: Run the tests**

Run: `... -TestFilter "[iss_mission]"`, then (the tick loop and coast allowance changed, new branches inactive without
a sequence) `-TestFilter "[proximity],[docking],[rendezvous]"` without `[slow]` campaigns of other subsystems.
Expected: PASS.

- [ ] **Step 5: Commit and push**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/src/sim/flight_session_iss_mission.cpp core/CMakeLists.txt core/tests/test_iss_mission_sequence.cpp core/tests/CMakeLists.txt
git commit -m "feat(sim): ISS mission sequence - phasing, proximity and docking chained in the flight session"
git push origin main
```

---

### Task 4: `crew_dragon_iss` - orbit-start mission

**Files:**
- Create: `core/include/starsim/sim/iss_mission.hpp`, `core/src/sim/iss_mission.cpp` (add to `core/CMakeLists.txt`)
- Modify: `core/include/starsim/sim/mission_start.hpp`, `core/src/sim/mission_start.cpp`
- Test: `core/tests/test_iss_mission_campaign.cpp` (register)

**Interfaces:**
- Consumes: Task 1 (`plan_launch_window`, `separation_state_estimate`, `plane_bias_rad`), Task 2 loaders, Task 3
  sequence, `sim::iss_start`, `orbit::OemArchive::scan`.
- Produces:
  ```cpp
  namespace starsim::sim {
  // data/iss_oem next to config/ (a missing directory: an empty archive).
  orbit::OemArchive iss_archive(const std::filesystem::path& config_dir, const time::TimeScaleConverter& time);
  // The window of an iss_launch mission (its ascent's site and target inclination, the ISS from `archive`).
  mission::LaunchWindow iss_launch_window(const FlightSession& session, const std::filesystem::path& config_dir,
                                          const config::IssLaunchMission& mission, const orbit::OemArchive& archive,
                                          const time::TimeScaleConverter& time);
  MissionStart start_iss_rendezvous_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                            const config::IssRendezvousMission& mission,
                                            const orbit::OemArchive& archive, const time::TimeScaleConverter& time);
  }
  ```
  `MissionStart` gets `std::vector<std::string> warnings;` (German: reboosts inside the mission window, the ISS source
  note) and `std::optional<mission::LaunchWindow> launch_window;`.

`start_iss_rendezvous_mission`:
1. `iss = config::load_vehicle(config_dir, mission.station_id)`; check the port exists in
   `iss.stages.back().docking_ports` (else `std::runtime_error("Mission <id>: unbekannter Andockport '<port>'")`).
2. Without `phase_rad`: `launch = load_iss_launch_mission(config_dir, mission.launch_id)` with its `earliest_utc`
   replaced by `mission.epoch_utc`; `w = iss_launch_window(...)`; refused -> `MissionStart{refusal = w.reason}`, no
   bodies placed. Epoch = `w.separation`, Dragon state = `w.separation_state_gcrf`.
   With `phase_rad`: epoch = `time.from_utc(mission.epoch_utc)`, the state of `rendezvous_test::start` moved here: the
   200 x 210 km orbit at periapsis, ISS plane, `phase_rad` behind, the plane turned by `plane_bias_rad(..., match =
   schedule boost)` (schedule from an unbiased first guess, as Task 1's iteration).
3. `st = iss_start(iss, archive, time, session.earth(), epoch, parse_iss_orbit_source(mission.iss_orbit),
   mission.iss_argument_of_latitude_rad)` - throws the German message for `oem` without a window (Review Focus 1).
4. `session.place_in_orbit(load_vehicle(config_dir, mission.vehicle_id), state, epoch)`;
   `session.add_station(st.vehicle, st.state_gcrf, st.epoch, st.impulses)`.
5. Sequence: port, phasing (`mission_limit_s`, `earliest_arrival_s`), `from_launch = false`; `set_iss_mission`.
   The Rendezvous leg needs a plan: call `plan_rendezvous` directly here (refused -> `refusal`, `set_iss_mission({})`),
   so the planning cost is inside start_mission (measured, spec "a few seconds").
6. Warnings: `st.note` and `session.rendezvous_warnings()`.
   `return_planned = true` (field reused as "planned", as the lunar missions do).

`start_mission` dispatches `IssRendezvous` to it with `iss_archive(config_dir, time)`; `IssLaunch` / `IssDeparture`
throw `std::invalid_argument("... Task 5/6")` until those tasks (remove then).

(`session.earth()`: if `FlightSession` has no accessor for its `BodyOrientation`, add `const frames::BodyOrientation&
earth() const { return earth_; }` - check first.)

- [ ] **Step 1: Write the failing tests** (`core/tests/test_iss_mission_campaign.cpp`)

```cpp
// ISS i6 campaigns: the three missions through start_mission - orbit start to hard capture at both ports, the
// refusals, the pad mission end to end, the departure to splashdown.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "rendezvous_test_support.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/sim/iss_mission.hpp"
#include "starsim/sim/mission_start.hpp"
#include "test_support.hpp"

using namespace starsim;
using sim::IssMissionLeg;

namespace {
bool verbose() { return std::getenv("STARSIM_VERBOSE") != nullptr; }

struct Session {
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    sim::FlightSession session{ephemeris, earth, rendezvous_test::config_dir};
    // the committed fixture stands in for data/iss_oem (git-ignored)
    const orbit::OemArchive archive = orbit::OemArchive::scan(rendezvous_test::fixture_dir, clock);
};

// Flies until `done` or `limit_s`; large steps while coasting (the sequence stops the coast at its leg changes).
template <class Done>
void fly(sim::FlightSession& s, Done done, double limit_s) {
    const auto start = s.epoch();
    while (!done() && s.epoch() - start < limit_s && s.iss_mission_leg() != IssMissionLeg::Ended) {
        s.advance_to(s.epoch() + 600.0, 10000000);
    }
}
}  // namespace

TEST_CASE("ISS mission: crew_dragon_iss from the window's separation to hard capture at IDA-2",
          "[iss][iss_mission][docking][slow]") {
    test::spice_session();
    Session f;
    const auto m = config::load_iss_rendezvous_mission(rendezvous_test::config_dir, "crew_dragon_iss");
    const auto wall = std::chrono::steady_clock::now();
    const auto st = sim::start_iss_rendezvous_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    const double planning_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall).count();
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    REQUIRE(st.launch_window);
    CHECK(planning_s < 10.0);  // spec: a few seconds
    const double kg0 = f.session.body(f.session.controlled_index()).vehicle().rcs_propellant_kg();
    fly(f.session, [&] { return f.session.iss_mission_leg() == IssMissionLeg::Docked; }, 30.0 * 3600.0);
    INFO(f.session.iss_mission_status());
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Docked);
    const double used_kg = kg0 - f.session.body(f.session.controlled_index()).vehicle().rcs_propellant_kg();
    CHECK(f.session.epoch() - st.launch_window->liftoff < 28.0 * 3600.0);
    CHECK(used_kg < 2562.0 - 900.0);  // the deorbit + abort reserve stays (PhasingParameters::reserve_kg)
    if (verbose()) {
        std::printf("[iss_mission] crew_dragon_iss: planning %.1f s, docked %.1f h after liftoff, %.0f kg\n",
                    planning_s, (f.session.epoch() - st.launch_window->liftoff) / 3600.0, used_kg);
    }
}

TEST_CASE("ISS mission: crew_dragon_iss at IDA-3 with a set phase on the synthetic orbit",
          "[iss][iss_mission][docking][slow]") {
    test::spice_session();
    Session f;
    auto m = config::load_iss_rendezvous_mission(rendezvous_test::config_dir, "crew_dragon_iss");
    m.port = "ida3";
    m.iss_orbit = "synthetic";
    m.phase_rad = 40.0 * constants::deg_to_rad;
    const auto st = sim::start_iss_rendezvous_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    CHECK_FALSE(st.launch_window);
    fly(f.session, [&] { return f.session.iss_mission_leg() == IssMissionLeg::Docked; }, 30.0 * 3600.0);
    INFO(f.session.iss_mission_status());
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Docked);
}

TEST_CASE("ISS mission: refusals - OEM date outside the archive, unknown port", "[iss][iss_mission]") {
    test::spice_session();
    Session f;
    auto m = config::load_iss_rendezvous_mission(rendezvous_test::config_dir, "crew_dragon_iss");
    m.iss_orbit = "oem";
    m.epoch_utc = {2027, 3, 1, 0, 0, 0.0};  // beyond the fixture's window
    m.phase_rad = 30.0 * constants::deg_to_rad;
    CHECK_THROWS_AS(sim::start_iss_rendezvous_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock),
                    std::runtime_error);
    CHECK(f.session.body_count() == 0);  // nothing placed (Review Focus 1)
    auto p = config::load_iss_rendezvous_mission(rendezvous_test::config_dir, "crew_dragon_iss");
    p.port = "ida9";
    CHECK_THROWS_AS(sim::start_iss_rendezvous_mission(f.session, rendezvous_test::config_dir, p, f.archive, f.clock),
                    std::runtime_error);
}

TEST_CASE("ISS mission: start_mission dispatches crew_dragon_iss", "[iss][iss_mission]") {
    test::spice_session();
    Session f;
    const auto st = sim::start_mission(f.session, rendezvous_test::config_dir, "crew_dragon_iss", {}, f.clock);
    CHECK(st.kind == config::MissionKind::IssRendezvous);
    // data/iss_oem may be absent on a fresh checkout: "auto" then flies the synthetic orbit - either way planned
    INFO(st.refusal);
    CHECK(st.return_planned);
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Rendezvous);
    CHECK_FALSE(st.warnings.empty());  // at least the ISS source note
}
```

(`body_count()`: use the session's existing count accessor - `bodies().size()` or whatever `FlightSession` exposes.)

- [ ] **Step 2: Run to verify they fail** - `-TestFilter "[iss_mission]"`: compile error (`iss_mission.hpp`).

- [ ] **Step 3: Implement** `iss_mission.hpp/.cpp` and the dispatch as specified. The order matters for the refusals:
validate the port and compute `iss_start` **before** `place_in_orbit`, so a refused mission places nothing.

- [ ] **Step 4: Run** `-TestFilter "[iss_mission]"` (includes the `[slow]` campaigns; they take minutes - note the
times and the docked numbers in the ledger). Expected: PASS. If the window's separation phase gives a Waypoint-0
arrival later than the mission limit, the window planner refused it - the test then shows `st.refusal`; fix the cause,
do not widen the limit.

- [ ] **Step 5: Commit and push**

```bash
git add core/include/starsim/sim/iss_mission.hpp core/src/sim/iss_mission.cpp core/include/starsim/sim/mission_start.hpp core/src/sim/mission_start.cpp core/CMakeLists.txt core/tests/test_iss_mission_campaign.cpp core/tests/CMakeLists.txt
git commit -m "feat(sim): crew_dragon_iss mission - separation of the launch window to hard capture"
git push origin main
```

---

### Task 5: `falcon9_crew_dragon_iss` - pad mission with launch window

**Files:**
- Modify: `core/include/starsim/sim/iss_mission.hpp`, `core/src/sim/iss_mission.cpp`, `core/src/sim/mission_start.cpp`
- Modify: `config/missions/falcon9_crew_dragon_iss.toml` (calibrated `insertion_s`, `insertion_arc_deg`)
- Test: `core/tests/test_iss_mission_campaign.cpp`

**Interfaces:**
- Produces: `MissionStart start_iss_launch_mission(FlightSession&, const std::filesystem::path& config_dir,
  const config::IssLaunchMission&, const orbit::OemArchive&, const time::TimeScaleConverter&);`

Flow: `w = iss_launch_window(...)` (refused -> `refusal`, nothing launched - Review Focus 2); `ascent =
load_ascent_mission(config_dir, mission.ascent_id)`; `start_ascent_mission(session, ascent, config_dir, w.liftoff)`;
`st = iss_start(..., w.liftoff, ...)`; `session.add_station(st...)` (after the launch: `launch_from_pad` clears the
station); sequence with `from_launch = true`, port, phasing with `mission_limit_s = mission.mission_limit_s -
window.insertion_s` and `earliest_arrival_s`. `MissionStart.epoch = w.liftoff`, `site_id = ascent.site_id`,
`launch_window = w`, warnings: the source note and every station impulse between liftoff and liftoff + mission limit
("Reboost der ISS um <UTC>: <dv> m/s" - use `time.to_utc` for the text if the converter offers it, else TDB hours after
liftoff). The ascent's inclination must equal the window's: `LaunchWindowParameters::inclination_rad =
ascent.inclination_rad`, `northerly = ascent.northerly`, the separation radii from `ascent.periapsis_height_m` /
`apoapsis_height_m` + `wgs84_semi_major_axis_m`.

- [ ] **Step 1: Write the calibration test (fails until the config is calibrated)**

```cpp
TEST_CASE("ISS mission: the window's separation estimate matches the flown ascent", "[iss][iss_mission][ascent][slow]") {
    test::spice_session();
    Session f;
    const auto m = config::load_iss_launch_mission(rendezvous_test::config_dir, "falcon9_crew_dragon_iss");
    const auto st = sim::start_iss_launch_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    INFO(st.refusal);
    REQUIRE(st.launch_window);
    const auto& w = *st.launch_window;
    auto& s = f.session;
    // fly to the separation (the sequence then plans the rendezvous: stop as soon as the leg changes)
    while (s.iss_mission_leg() == IssMissionLeg::Ascent && s.epoch() - w.liftoff < 1800.0) {
        s.advance_to(s.epoch() + 1.0, 100000);
    }
    REQUIRE(s.iss_mission_leg() == IssMissionLeg::Rendezvous);
    const auto& dragon = s.body(s.controlled_index()).state();
    const double dt = s.epoch() - w.separation;
    const auto predicted = orbit::propagate_kepler(w.separation_state_gcrf, constants::earth_gm_m3ps2, dt);
    const math::Vec3 n_pred = math::normalized(cross(predicted.position_m, predicted.velocity_mps));
    const math::Vec3 n_flown = math::normalized(cross(dragon.position_gcrf_m, dragon.velocity_gcrf_mps));
    const double plane_deg = std::acos(std::clamp(dot(n_pred, n_flown), -1.0, 1.0)) / constants::deg_to_rad;
    const double phase_deg = std::acos(std::clamp(dot(math::normalized(predicted.position_m),
                                                      math::normalized(dragon.position_gcrf_m)), -1.0, 1.0)) /
                             constants::deg_to_rad;
    if (verbose()) {
        std::printf("[iss_mission] separation %.1f s after liftoff (planned %.1f), plane %.4f deg, phase %.3f deg\n",
                    s.epoch() - w.liftoff, w.separation - w.liftoff, plane_deg, phase_deg);
    }
    CHECK(std::abs(s.epoch() - w.separation) < 30.0);
    CHECK(plane_deg < 0.05);   // [E] the PEG plane: 0.05 deg ~ 7 m/s of plane change, removed by the coelliptic burn
    CHECK(phase_deg < 0.5);    // [E] 0.5 deg ~ 2 min of catch-up: well inside the schedule's slack
}
```

(`orbit::propagate_kepler` - use the Kepler propagator `orbit/kepler.hpp` offers under its actual name.)

- [ ] **Step 2: Run, then calibrate**

Run: `$env:STARSIM_VERBOSE=1; ... -TestFilter "ISS mission: the window's separation estimate*"`. Read the flown
separation time and compute the flown in-plane arc (print it too: the angle between the pad's direction at liftoff
projected into the plane and the flown separation position). Write both into `falcon9_crew_dragon_iss.toml` as
`insertion_s` / `insertion_arc_deg` with `[D]` and the date, rerun until PASS. If the plane error exceeds 0.05 deg the
cause is the guidance plane vs `ascent_plane_normal` (pad CG vs site, geodetic up) - investigate before loosening.

- [ ] **Step 3: Add the end-to-end and refusal tests**

```cpp
TEST_CASE("ISS mission: falcon9_crew_dragon_iss from the pad to hard capture", "[iss][iss_mission][docking][slow]") {
    test::spice_session();
    Session f;
    const auto m = config::load_iss_launch_mission(rendezvous_test::config_dir, "falcon9_crew_dragon_iss");
    const auto st = sim::start_iss_launch_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    INFO(st.refusal);
    REQUIRE(st.launch_window);
    fly(f.session, [&] { return f.session.iss_mission_leg() == IssMissionLeg::Docked; }, 30.0 * 3600.0);
    INFO(f.session.iss_mission_status());
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Docked);
    CHECK(f.session.epoch() - st.launch_window->liftoff < m.mission_limit_s);
    if (verbose()) {
        std::printf("[iss_mission] pad -> docked %.1f h after liftoff, rendezvous burns %zu, re-plans %d\n",
                    (f.session.epoch() - st.launch_window->liftoff) / 3600.0, f.session.rendezvous_burns().size(),
                    f.session.rendezvous_replans());
    }
}

TEST_CASE("ISS mission: no launch window - refused, nothing launched", "[iss][iss_mission][launch_window]") {
    test::spice_session();
    Session f;
    auto m = config::load_iss_launch_mission(rendezvous_test::config_dir, "falcon9_crew_dragon_iss");
    m.mission_limit_s = m.window.insertion_s + 2.0 * 3600.0;
    m.window.search_s = 86400.0;
    const auto st = sim::start_iss_launch_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    CHECK_FALSE(st.return_planned);
    CHECK(st.refusal.find("Kein Startfenster") != std::string::npos);
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::None);
}
```

The window is checked on the separation estimate, not the flown separation; the 0.5 deg phase tolerance must leave the
schedule feasible - the end-to-end test proves it.

- [ ] **Step 4: Dispatch** `IssLaunch` in `start_mission` (remove the Task-4 placeholder throw).

- [ ] **Step 5: Run** `-TestFilter "[iss_mission]"` (+ `[ascent]` because the ascent is now flown under a sequence -
its branches are inactive without one, so only `[ascent]` fast cases). Expected: PASS. Ledger: separation numbers, the
end-to-end hours, burns and propellant.

- [ ] **Step 6: Commit and push**

```bash
git add core/include/starsim/sim/iss_mission.hpp core/src/sim/iss_mission.cpp core/src/sim/mission_start.cpp config/missions/falcon9_crew_dragon_iss.toml core/tests/test_iss_mission_campaign.cpp
git commit -m "feat(sim): falcon9_crew_dragon_iss - launch window, ascent and separation into the ISS sequence"
git push origin main
```

---

### Task 6: `crew_dragon_iss_departure` - docked start, undocking timed to the return

**Files:**
- Modify: `core/include/starsim/contact/docking_mechanism.hpp`, `core/src/contact/docking_mechanism.cpp`
  (`start_docked(double t_s)`), `core/include/starsim/sim/docking_sim.hpp`, `core/src/sim/docking_sim.cpp`
  (`start_docked(time::Epoch)`: sets the mechanism clock origin `t0_ = epoch`), `core/include/starsim/sim/flight_session.hpp`,
  `core/src/sim/flight_session_docking.cpp` (`dock_in_place`, factor `mated_pose` out of `merge_docked`),
  `core/src/sim/flight_session_capsule.cpp` (`capsule_return_preview`)
- Modify: `iss_mission.hpp/.cpp`, `mission_start.cpp`
- Test: `core/tests/test_iss_mission_sequence.cpp`, `core/tests/test_iss_mission_campaign.cpp`, a `[docking]` unit case
  in the existing `core/tests/test_docking_mechanism.cpp` (check the file name: the i5 mechanism tests)

**Interfaces:**
- Produces:
  ```cpp
  // contact::DockingMechanism: Ready -> Docked directly (a scenario that starts docked): hooks closed, event
  // "fest angedockt" at t_s, status as after a hard capture. False + refusal() outside Ready.
  bool start_docked(double t_s);
  // sim::DockingSim
  bool start_docked(time::Epoch epoch);
  // FlightSession: the armed pair (set_rendezvous_target with an active chaser port on a passive station port) mated
  // at once - the chaser moved to the mated pose moving with the station, the mechanism Docked, the compound merged.
  // Throws std::logic_error without an armed mechanism or outside Ready.
  void dock_in_place();
  // FlightSession: the capsule return plan_capsule_return would make now (same refusals except docked), not stored;
  // for a docked capsule its held state is used.
  gnc::DeorbitPlan capsule_return_preview(gnc::DeorbitParameters parameters) const;
  // sim
  MissionStart start_iss_departure_mission(FlightSession&, const std::filesystem::path& config_dir,
                                           const config::IssDepartureMission&, const orbit::OemArchive&,
                                           const time::TimeScaleConverter&);
  static constexpr double iss_departure_start_lead_s = 600.0;  // [E] the scenario starts this long before undocking
  ```

`dock_in_place` (in `flight_session_docking.cpp`; `mated_pose` returns `{math::Quat attitude; math::Vec3 cg_gcrf_m;}`
computed exactly as the first half of `merge_docked`, which then calls it):

```cpp
void FlightSession::dock_in_place() {
    if (!docking_ || !station_index_) throw std::logic_error("dock_in_place: no armed docking mechanism");
    auto& chaser = *bodies_[controlled_].body;
    const auto& station = *bodies_[*station_index_].body;
    const auto pose = mated_pose(chaser, station);
    const auto& ss = station.state();
    const math::Vec3 w = ss.attitude.rotate(ss.angular_velocity_body_radps);
    auto cs = chaser.state();
    cs.attitude = pose.attitude;
    cs.position_gcrf_m = pose.cg_gcrf_m;
    cs.velocity_gcrf_mps = ss.velocity_gcrf_mps + cross(w, pose.cg_gcrf_m - ss.position_gcrf_m);
    cs.angular_velocity_body_radps = pose.attitude.conjugate().rotate(w);
    chaser.set_state(cs);
    if (!docking_->start_docked(epoch_)) throw std::logic_error("dock_in_place: mechanism not ready");
    docking_seen_phase_ = contact::DockingPhase::Docked;
    docking_status_ = docking_->mechanism().status();
    merge_docked();
}
```

`capsule_return_preview`: move the body of `plan_capsule_return` from `// The capsule after the trunk jettison` to the
second `plan(...)` into a private const helper `gnc::DeorbitPlan capsule_deorbit_plan(gnc::DeorbitParameters) const`
that returns the plan (with the same refusal texts as a `DeorbitPlan{feasible=false, reason}`); `plan_capsule_return`
keeps its docking / step guards and stores the helper's result; the preview calls the helper without the docking guard.

`start_iss_departure_mission`:
1. Load ISS (port check as Task 4), Dragon (`rcs_propellant_kg` set as `place_capsule_return_mission` does - look at
   it for the tank call), the return mission `ret = load_capsule_return_mission(config_dir, mission.return_id)`.
2. Probe at `e0 = time.from_utc(mission.epoch_utc)`: `place_docked(e0)` = iss_start at e0, `place_in_orbit(dragon,
   iss state + 50 m along the port axis (any free state, moved by dock_in_place), e0)`, `add_station`, open the nose
   cone (the Dragon port sits behind it - use the call `plan_rendezvous` / `start_proximity` use), `set_rendezvous_target
   (station, dragon port, mission.port)`, `dock_in_place()`, `set_splashdown_zone(load_splashdown_zone(..., ret.zone_id))`.
3. `p = capsule_return_parameters(ret)`, `p.search_window_s = mission.search_window_s`, `p.lead_s =
   mission.undock_to_deorbit_s`; `plan = session.capsule_return_preview(p)`; refused -> `refusal = "Keine
   Rückkehrgelegenheit: " + plan.reason`, `session` left with the probe placed but no sequence (document).
4. `undock = plan.node.epoch - mission.undock_to_deorbit_s`; place again (step 2) at `undock -
   iss_departure_start_lead_s` (a fresh `iss_start` at that epoch).
5. Sequence: port, `undock_epoch = undock`, `capsule_return = p` (with the return's search window), `deorbit_epoch =
   plan.node.epoch`. `MissionStart.epoch` = the start epoch; warnings: source note + reboosts until the deorbit.

- [ ] **Step 1: Write the failing tests**

In `test_docking_mechanism.cpp` (i5):
```cpp
TEST_CASE("Docking mechanism: start_docked enters Docked from Ready only", "[docking]") {
    contact::DockingMechanism m;
    REQUIRE(m.start_docked(5.0));
    CHECK(m.phase() == contact::DockingPhase::Docked);
    CHECK(m.hard_captured());
    CHECK_FALSE(m.start_docked(6.0));
    CHECK(m.undock(10.0));  // the docked state undocks as after a capture
}
```

In `test_iss_mission_sequence.cpp`:
```cpp
TEST_CASE("ISS mission sequence: dock_in_place mates the pair rigidly with momentum conserved",
          "[iss][iss_mission][docking]") {
    test::spice_session();
    auto d = docking_test::DragonAtPort::place("ida2", 30.0, 0.0, 0.0, 0.0);  // 30 m out, at rest
    auto& s = d.session;
    s.dock_in_place();
    CHECK(s.docked());
    CHECK(norm(s.relative_port_state()->position_m) < 1e-6);
    const auto& r = s.docking_merge_record();
    CHECK(norm(r.momentum_after - r.momentum_before) / norm(r.momentum_before) < 1e-12);
    s.advance_to(d.t0 + 3600.0, 1000000);
    CHECK(norm(s.relative_port_state()->position_m) < 0.001);
    REQUIRE(s.undock());
}

TEST_CASE("ISS mission sequence: WaitUndock undocks at its epoch, also through a long coast",
          "[iss][iss_mission][docking]") {
    test::spice_session();
    auto d = docking_test::DragonAtPort::place("ida2", 30.0, 0.0, 0.0, 0.0);
    auto& s = d.session;
    s.dock_in_place();
    sim::IssMissionSequence q;
    q.station_port = "ida2";
    q.undock_epoch = d.t0 + 1234.0;
    s.set_iss_mission(q);
    CHECK(s.iss_mission_leg() == IssMissionLeg::WaitUndock);
    s.advance_to(d.t0 + 1300.0, 1000000);  // one call across the undock epoch
    CHECK(s.iss_mission_leg() == IssMissionLeg::Departure);
    CHECK(s.docking_phase() == contact::DockingPhase::HooksOpening);
    const auto& ev = *s.docking_events();
    CHECK(ev.back().t_s >= 0.0);  // the hooks opened on the mechanism clock after the start
}
```
(Adjust the event field name to `DockingEvent`'s; the check is that the undock happened at 1234 s +- one tick - assert
it with whatever epoch the event or `departure_events()` records.)

In `test_iss_mission_campaign.cpp`:
```cpp
TEST_CASE("ISS mission: crew_dragon_iss_departure - undock timed to the pass, splashdown",
          "[iss][iss_mission][docking][slow]") {
    test::spice_session();
    Session f;
    const auto m = config::load_iss_departure_mission(rendezvous_test::config_dir, "crew_dragon_iss_departure");
    const auto st = sim::start_iss_departure_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    auto& s = f.session;
    REQUIRE(s.iss_mission()->deorbit_epoch);
    const auto planned_deorbit = *s.iss_mission()->deorbit_epoch;
    CHECK(*s.iss_mission()->undock_epoch - st.epoch == Catch::Approx(600.0));
    const double rcs0 = s.body(s.controlled_index()).vehicle().rcs_propellant_kg();
    fly(s, [&] { return s.iss_mission_leg() == IssMissionLeg::Complete; }, 4.0 * 86400.0);
    INFO(s.iss_mission_status());
    CHECK(s.iss_mission_leg() == IssMissionLeg::Complete);
    CHECK(s.capsule_return_step() == sim::CapsuleReturnStep::SplashedDown);
    REQUIRE(s.ship_return_plan());
    CHECK(std::abs(s.ship_return_plan()->node.epoch - planned_deorbit) < 1800.0);  // the previewed pass was taken
    if (verbose()) {
        std::printf("[iss_mission] departure: undock -> deorbit %.1f h, RCS used %.0f kg\n",
                    (s.ship_return_plan()->node.epoch - *s.iss_mission()->undock_epoch) / 3600.0,
                    rcs0 - s.body(s.controlled_index()).vehicle().rcs_propellant_kg());
    }
}
```

The `fly` helper's 600 s steps: the deorbit burn's own coast stops are inside `advance_to` (the d4 campaign used
smaller steps near the burn only for speed - if the run is slow, copy its step rule from `test_docking_campaign.cpp`).

- [ ] **Step 2: Run to verify they fail** - `-TestFilter "[iss_mission],[docking]"`: compile errors.

- [ ] **Step 3: Implement** mechanism, DockingSim, `dock_in_place` (+ `mated_pose` refactor), preview helper, the
departure start and the sequence legs WaitUndock / Departure / Return (already specified in Task 3's table: implement
now if Task 3 left them as stubs), dispatch `IssDeparture` in `start_mission`.

- [ ] **Step 4: Verify the Crew-9 timing source** (web search "Crew-9 undocking March 18 2025 deorbit burn time"); if
the times differ, correct `undock_to_deorbit_h` and its `[S]` comment.

- [ ] **Step 5: Run** `-TestFilter "[iss_mission],[docking],[capsule]"` (the capsule return code was refactored: its
`[capsule]` / d4 tags with their `[slow]` cases). Expected: PASS.

- [ ] **Step 6: Commit and push**

```bash
git add core/include/starsim/contact/docking_mechanism.hpp core/src/contact/docking_mechanism.cpp core/include/starsim/sim/docking_sim.hpp core/src/sim/docking_sim.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session_docking.cpp core/src/sim/flight_session_capsule.cpp core/include/starsim/sim/iss_mission.hpp core/src/sim/iss_mission.cpp core/src/sim/mission_start.cpp core/tests/
git commit -m "feat(sim): crew_dragon_iss_departure - docked start, undocking timed to the return pass, splashdown"
git push origin main
```

---

### Task 7: Game interface (`FlightSim`) and smoke test

**Files:**
- Create: `bindings/src/flight_snapshot_iss.cpp` (add to `bindings/CMakeLists.txt` after `src/flight_snapshot_lunar.cpp`)
- Modify: `bindings/src/flight_snapshot.hpp` (fields + declarations), `bindings/src/flight_snapshot.cpp` (build, absent
  defaults at line ~398 / ~413 like `lunar_state`), `bindings/src/flight_sim.hpp`, `bindings/src/flight_sim.cpp`
  (getters, commands, `_bind_methods`, `start_mission`'s result message carries the warnings)
- Modify: `game/tests/smoke_test.gd`

**Interfaces:**
- Getters (Dictionaries, `present: false` without the thing; keys documented in `flight_sim.hpp` as for
  `lunar_state`):
  - `iss_state()`: `index`, `position_gcrf_m`, `velocity_gcrf_mps`, `attitude_gcrf` (Quaternion), `mass_kg`,
    `docked_mass_kg`, `ports`: Array of `{id, name, position_gcrf_m, axis_gcrf, up_gcrf, soft_capture}` from
    `vehicle::port_kinematics(station, id)` (axis = column x of `axes_gcrf`, up = column z - check the column order in
    `vehicle/docking_port.hpp` against the config's axis/up), `warnings` (rendezvous + proximity warnings).
  - `rendezvous_plan()`: `feasible`, `reason`, `step` (`to_string(rendezvous_step())`), `status`, `replans`,
    `total_delta_v_mps`, `propellant_kg`, `waypoint0_tdb_s`, `burns`: Array of `{kind (german_name), epoch_tdb_s,
    delta_v_mps, flown (bool)}` - planned burns from `rendezvous_plan()->burns`, flown from `rendezvous_burns()`.
  - `docking_state()`: `proximity_phase`, `docking_phase` (German `to_string`, "" when absent), `status`,
    `position_port_m`, `velocity_port_mps`, `range_m`, `range_rate_mps` (= -closing_speed), `offset_m`
    (hypot(y, z)), `misalignment_deg`, `clocking_deg`, `latched` (phase SoftCapture .. Docked), `hooks_closed` (Docked),
    `docked`, `collision_impulse_ns` (0 when none), `departure_phase`, `loads` `{compression_n, shear_n, bending_nm}`.
  - `station_geometry()`: `boxes`: Array of `{center_m, axis_x, axis_y, axis_z, half_extents_m}`, `capsules`: Array of
    `{a_m, b_m, radius_m}`, all in the station body frame (`contact::station_shape(config)`), plus `ports` in the body
    frame from the config.
  - `iss_mission()`: `leg` (German), `status`, `port`, `liftoff_tdb_s`, `separation_tdb_s`, `plane_bias_deg`,
    `undock_tdb_s`, `deorbit_tdb_s` (0 when not set), `warnings` (MissionStart warnings, kept in `SnapshotContext`).
- Commands (ids, `result(id)`): `proximity_hold()`, `proximity_retreat()`, `proximity_abort()`,
  `proximity_resume()`, `undock()` - each `post_active(name, [](FlightSession& s) { const bool ok = s.proximity_hold();
  return Outcome{ok, godot::String::utf8(s.proximity_status().c_str())}; })`; `undock` returns `docking_status()`.
  Outside a proximity flight the core already refuses with a German `proximity_status()` (i4 review) - check, else
  return `Outcome{false, "Kein Anflug aktiv"}` (Review Focus 3).

- [ ] **Step 1: Write the failing smoke checks** (`game/tests/smoke_test.gd`, a new `_check_iss(config_dir, tdb)` called
after `_check_lunar` with `_check(_check_iss(config_dir, tdb) == true, "ISS mission checks ran to the end")`):

```gdscript
# ISS missions (i6): the three kinds start, ISS / rendezvous / docking telemetry, station geometry, commands.
func _check_iss(config_dir: String, tdb: float) -> bool:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	var started := Time.get_ticks_msec()
	var id: int = sim.start_mission("crew_dragon_iss", tdb)
	_check(sim.wait_idle(120.0), "ISS rendezvous planned within 120 s")
	_check(sim.result(id).get("ok", false), "crew_dragon_iss starts: %s" % sim.result(id).get("message", ""))
	print("  ISS rendezvous start: %.1f s" % ((Time.get_ticks_msec() - started) / 1000.0))
	var iss: Dictionary = sim.iss_state()
	_check(iss.get("present", false) and iss["ports"].size() == 2, "iss_state with both ports")
	var plan: Dictionary = sim.rendezvous_plan()
	_check(plan.get("present", false) and plan.get("feasible", false), "rendezvous_plan feasible")
	_check(plan["burns"].size() >= 3, "rendezvous_plan has its burns")
	_check(sim.iss_mission().get("leg", "") == "Phasenflug", "iss_mission in the phasing leg")
	var geo: Dictionary = sim.station_geometry()
	_check(geo.get("present", false) and geo["boxes"].size() + geo["capsules"].size() >= 9, "station_geometry primitives")
	var hold: int = sim.proximity_hold()
	_check(sim.wait_idle(10.0) and not sim.result(hold).get("ok", true), "hold refused outside the approach")
	var und: int = sim.undock()
	_check(sim.wait_idle(10.0) and not sim.result(und).get("ok", true), "undock refused while not docked")
	_check(sim.docking_state().get("present", false) == false or sim.docking_state().get("docked", true) == false,
			"not docked during phasing")

	var pad := FlightSim.new()
	pad.setup(config_dir)
	var pid: int = pad.start_mission("falcon9_crew_dragon_iss", tdb)
	_check(pad.wait_idle(120.0) and pad.result(pid).get("ok", false), "falcon9_crew_dragon_iss starts")
	_check(pad.iss_mission().get("liftoff_tdb_s", 0.0) > 0.0, "launch window in iss_mission")
	_check(absf(pad.mission_start_tdb_s() - pad.iss_mission()["liftoff_tdb_s"]) < 1e-6, "the mission starts at the window")

	var dep := FlightSim.new()
	dep.setup(config_dir)
	var did: int = dep.start_mission("crew_dragon_iss_departure", tdb)
	_check(dep.wait_idle(120.0) and dep.result(did).get("ok", false), "crew_dragon_iss_departure starts")
	_check(dep.docking_state().get("docked", false), "departure mission starts docked")
	_check(dep.iss_mission().get("undock_tdb_s", 0.0) > dep.mission_start_tdb_s(), "undocking ahead")
	var ab: int = dep.proximity_abort()
	_check(dep.wait_idle(10.0) and not dep.result(ab).get("ok", true), "abort refused while docked")
	return true
```

- [ ] **Step 2: Build with bindings and run the smoke test - expect failures** (`Invalid call. Nonexistent function
'iss_state'`):
```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -TestFilter "[iss_mission]"
external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd
```

- [ ] **Step 3: Implement** `flight_snapshot_iss.cpp` (`build_iss_state`, `build_rendezvous_plan`,
`build_docking_state`, `build_station_geometry`, `build_iss_mission(session, context)` - same helpers `to_godot` /
`german` as `flight_snapshot_lunar.cpp`), the snapshot fields, getters, commands, bindings, the warnings in
`SnapshotContext` (set by `start_mission`'s command) and the start message: `ok` with the warnings joined by "; " as
`message` when not refused.

- [ ] **Step 4: Run** the build (`-TestFilter "[iss_mission]"` fast cases) and the smoke test. Expected: all smoke checks
pass, the printed ISS start time a few seconds.

- [ ] **Step 5: Commit and push**

```bash
git add bindings/ game/tests/smoke_test.gd
git commit -m "feat(bindings): ISS game interface - iss_state, rendezvous_plan, docking_state, station_geometry, iss_mission, proximity and undock commands"
git push origin main
```

---

### Task 8: Documentation, final review

**Files:**
- Modify: `docs/subsystems.md` (new section "ISS missions and launch window (i6)": the legs, the window method and
  its numbers from the ledger - liftoff, bias, separation calibration, end-to-end hours / burns / propellant, the
  departure timing, planning times, the game interface keys; the deferred i5 minors that i6 settled or not)
- Modify: `docs/superpowers/specs/2026-10-19-iss-docking-design.md` (status line: i6 implemented; "Corrections" block
  under "i6 design" with this plan's six corrections as approved)
- Modify: `CLAUDE.md` (module list: `mission` + launch window, `sim` + ISS missions; `FlightSim` ISS getters in "Game
  (Godot) structure")
- Modify: `config/README.md` (the three mission kinds and their keys)
- Modify: `docs/ansichten.md` (pointers only: which getters carry ISS, rendezvous, docking and the station geometry -
  the views are the user's)

- [ ] **Step 1: Write the docs** (numbers from the ledger, no new claims).
- [ ] **Step 2: Run the affected tags once more**: `-TestFilter "[iss],[iss_mission],[launch_window]"` (with `[slow]`)
  and the smoke test. Expected: PASS; record the counts.
- [ ] **Step 3: Commit and push** - `git commit -m "docs: ISS i6 missions, launch window and game interface"`.
- [ ] **Step 4: Final review** of the i6 range (`git diff <i6 start>..HEAD`) by a fresh reviewer on the most capable
  model (superpowers:requesting-code-review); fix Critical / Important findings in one pass (`fix(sim): i6 review
  fixes - ...`), run the affected tags, push. Record minors in the ledger.
- [ ] **Step 5: Ledger and memory** - `.superpowers/sdd/2026-10-20-iss-i6-missions/progress.md` and memory
  `project-iss-docking` (i6 COMPLETE = milestone i complete); ask the user for `/clear`.
