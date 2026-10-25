# ISS i1: the ISS as a target body - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The ISS exists in the simulation as a second, uncontrolled `FlightBody` in ideal LVLH attitude, started
from NASA's OEM trajectory (or a synthetic reference orbit) and propagated with the same force model as every vehicle
(Earth + Moon + Sun, J2, US76 drag, SRP); validated against the OEM over 6 h and 24 h.

**Architecture:** A new `orbit/oem` module parses CCSDS OEM files and interpolates them (Lagrange degree 8 inside
uniform-step segments); `OemArchive` picks the newest file covering an epoch. `orbit/synthetic_station_orbit`
builds a state from mean elements advanced by the J2 secular rates. The ISS is configuration
(`config/vehicles/iss.toml`): a one-stage vehicle with a fixed drag area (`aero/fixed_area_aero`), an explicit
inertia, docking ports and collision primitives (data for i4/i5). `FlightBody` gets an ideal LVLH attitude law
(frames-level LVLH helper, so `vehicle` does not depend on `gnc`); `FlightSession::add_station` adds the body and
applies OEM events as impulses; `sim/station_start` chooses OEM / synthetic / auto.

**Tech Stack:** C++20, Catch2, toml++, CSPICE (only through `spice::SpiceTimeConverter`), Python 3.11 (data pipeline,
`unittest`).

**Spec:** `docs/superpowers/specs/2026-10-19-iss-docking-design.md` (sections "Intent", "i1 design: ISS as a target
body", "Error handling", "Testing").

## Global Constraints

- `core/` has no Godot dependency; modules depend downwards only: `vehicle` must not include `gnc` (the LVLH helper
  therefore lives in `frames`).
- SI units internally, angles in radians; names carry frame and unit (`position_gcrf_m`, `epoch_tdb_s`). OEM km and
  km/s are converted on load.
- Time: `Epoch` is TDB; OEM UTC strings are converted with `time::TimeScaleConverter` at load time only.
- EME2000 (OEM `REF_FRAME`) is taken as GCRF/J2000 axes; the frame bias (~23 mas, < 1 m at the ISS) is documented, not
  modelled.
- No magic numbers: tuned values are named constants or config values with `[S]` / `[E]` / `[D]` tags and a reason.
- Errors that reach the mission user (no OEM window, unknown port) carry German text; technical parse errors stay
  English like the existing `ConfigError`s.
- Fidelity flag `iss_orbit = "oem" | "synthetic" | "auto"`: `oem` without coverage is an error, never a silent fallback.
- Only free data: the OEM is NASA public domain; recorded in `tools/data_pipeline/sources.toml`.
- Commit on `main` (Conventional Commits) and push after every task once its tests are green; `-Fast` before every
  push; full suite before the final review (Task 6 touches flight sessions and vehicles).
- ctest runs test cases as parallel processes: no shared temp files (unique names per test case).
- Long command output to `tmp/` (git-ignored), read tails / grep. Never filter `-TestFilter "[mission]"`.
- Do not use Bash heredocs to write files: use the Write tool (memory `feedback-no-heredocs`).

## Review Focus

1. **The OEM grid is not uniform:** the committed file steps 4 min but shifts by 1.5 min at three rows
   (2026-10-17T17:57:30, 2026-10-21T04:35:00, 2026-10-28T08:42:30). An interpolation window straddling such a seam
   must not be used (segment boundaries may hide a state discontinuity). Test: Task 2 Step 1 (`interpolation does not
   straddle a step change`).
2. **CRLF files and trailing blanks:** the NASA file has CRLF line ends (git may convert them); the parser must accept
   both, tolerate blank `COMMENT` lines and the imperial "(...)" second lines of the event table. Test: Task 2 Step 1
   (`parses LF and CRLF alike`, `event table`).
3. **An epoch outside every OEM window with `iss_orbit = "oem"`:** a clear German error, no fallback; with `auto` the
   synthetic orbit and a note saying so. Test: Task 5 Step 1.
4. **The station alone in a session must still coast:** its orbital rate (1.1e-3 rad/s) exceeds the session's
   "rotation settled" threshold (1e-3 rad/s) for the controlled body. Test: Task 5 Step 1 (`a station alone coasts`).
5. **place_body / launch_from_pad after add_station drops the station** (they clear `bodies_`): documented order, and
   `station_index()` must not dangle. Test: Task 5 Step 1 (`placing a vehicle clears the station`).

---

## File structure

| File | Responsibility |
|---|---|
| `core/include/starsim/aero/fixed_area_aero.hpp`, `core/src/aero/fixed_area_aero.cpp` | Drag of a body with a fixed reference area (C_D A), along the oncoming flow |
| `core/include/starsim/config/vehicle_config.hpp`, `core/src/config/vehicle_config.cpp` | New stage fields: fixed drag, solar area, inertia; `[[stage.docking_port]]`, `[[stage.collision]]` |
| `core/src/vehicle/vehicle.cpp`, `core/include/starsim/vehicle/vehicle.hpp` | Fixed-area hull aero, explicit stage inertia (Izz separate) |
| `core/src/vehicle/flight_body.cpp`, `core/include/starsim/vehicle/flight_body.hpp` | SRP with a fixed area; ideal LVLH attitude law |
| `core/include/starsim/frames/lvlh.hpp`, `core/src/frames/lvlh.cpp` | LVLH axes and rate of an orbit state (moved from gnc) |
| `core/src/gnc/orbital_attitude.cpp` | delegates to `frames::lvlh_to_inertial` |
| `config/vehicles/iss.toml` | ISS data (mass, drag, inertia, ports, primitives, TEA, reference orbit) |
| `core/include/starsim/orbit/oem.hpp`, `core/src/orbit/oem.cpp` | OEM parser, events, interpolation, archive |
| `core/include/starsim/orbit/synthetic_station_orbit.hpp`, `core/src/orbit/synthetic_station_orbit.cpp` | Mean elements, J2 secular rates, synthetic state |
| `core/include/starsim/sim/station_start.hpp`, `core/src/sim/station_start.cpp` | OEM / synthetic / auto choice, OEM header overrides, impulses |
| `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` | `add_station`, `station_index`, impulses, coast allowance |
| `tools/data_pipeline/iss_oem.py`, `fetch.py`, `sources.toml`, `test_iss_oem.py` | Download + archive |
| `tests/validation/fixtures/iss_oem/ISS.OEM_J2K_EPH_2026-10-14T18-30-01.txt` | Committed fixture (already in the repo with this plan) |
| `core/tests/test_iss_vehicle.cpp`, `test_oem.cpp`, `test_synthetic_station_orbit.cpp`, `test_iss_station.cpp` | Tests |

---

### Task 1: ISS vehicle - fixed drag area, inertia, ports, primitives, ideal LVLH attitude

**Files:**
- Create: `core/include/starsim/aero/fixed_area_aero.hpp`, `core/src/aero/fixed_area_aero.cpp`
- Create: `core/include/starsim/frames/lvlh.hpp`, `core/src/frames/lvlh.cpp`
- Create: `config/vehicles/iss.toml`, `core/tests/test_iss_vehicle.cpp`
- Modify: `core/include/starsim/config/vehicle_config.hpp` (`AeroConfig`, `StageConfig`), `core/src/config/vehicle_config.cpp` (`load_stage`)
- Modify: `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp` (`rebuild_aero`, `hull_aero`, `mass_properties`, `copy`)
- Modify: `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp` (`solar_radiation_acceleration`, `step`, `coast`)
- Modify: `core/src/gnc/orbital_attitude.cpp` (`lvlh_attitude_gcrf` delegates)
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Produces:
  - `aero::FixedAreaAero{double drag_area_m2; double drag_coefficient;}` with
    `AeroResult compute(const math::Vec3& air_velocity_body_mps, double density_kg_m3, double speed_of_sound_mps) const`.
  - `config::AeroConfig::fixed_drag_area_m2` (0 = none), `fixed_drag_coefficient`, `solar_area_m2` (0 = cylinder
    model).
  - `config::StageConfig::inertia_kg_m2` (`std::optional<math::Vec3>`, principal (Ixx, Iyy, Izz) of the dry stage
    about its own CG), `docking_ports` (`std::vector<DockingPortConfig>`), `collision` (`std::vector<CollisionPrimitiveConfig>`),
    `body_in_lvlh_rad` (`std::optional<math::Vec3>`: yaw, pitch, roll of the ideal LVLH attitude; present = station).
  - `config::DockingPortConfig{std::string id; std::string name; math::Vec3 position_m; math::Vec3 axis; math::Vec3 up;}`
    (stage frame: x = station from the stage bottom, y/z body axes; `axis` outward unit vector, `up` the clocking
    reference perpendicular to it).
  - `config::CollisionPrimitiveConfig{std::string id; std::string kind; math::Vec3 a_m; math::Vec3 b_m; double radius_m; math::Vec3 half_extents_m;}`
    (`kind` "capsule": segment a-b with radius; "box": centre `a_m`, body-aligned, `half_extents_m`).
  - `frames::lvlh_to_inertial(const math::StateVector& state) -> math::Quat` (LVLH -> inertial; x completes the
    triad, y = -orbit normal, z = nadir) and `frames::lvlh_rate_inertial_radps(const math::StateVector&) -> math::Vec3`
    (`(r x v) / r^2`).
  - `vehicle::FlightBody::set_ideal_lvlh_attitude(std::optional<math::Quat> body_in_lvlh)`,
    `bool ideal_attitude() const`.
  - `vehicle::Vehicle::fixed_area_aero() const -> const aero::FixedAreaAero*`.

- [ ] **Step 1: Write the failing tests** - `core/tests/test_iss_vehicle.cpp`:

```cpp
#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/aero/fixed_area_aero.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/lvlh.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/flight_body.hpp"

using namespace starsim;
using math::Vec3;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

math::StateVector iss_like_state() {
    const double r = constants::wgs84_semi_major_axis_m + 415.0e3;
    const double v = std::sqrt(constants::earth_gm_m3ps2 / r);
    const double i = 51.64 * constants::deg_to_rad;
    return {{r, 0.0, 0.0}, {0.0, v * std::cos(i), v * std::sin(i)}};
}
}  // namespace

TEST_CASE("Fixed-area drag opposes the motion with C_D A q", "[iss][aero]") {
    const aero::FixedAreaAero aero{1177.76, 1.5};
    // the hull-aero convention (SlenderBodyAero, FlightBody): the body's velocity relative to the air, body axes
    const Vec3 motion{7660.0, 30.0, -12.0};
    const double rho = 3.0e-12;
    const auto result = aero.compute(motion, rho, 900.0);
    const double q = 0.5 * rho * dot(motion, motion);
    CHECK(norm(result.force_body_n) == Catch::Approx(q * 1.5 * 1177.76).epsilon(1e-12));
    CHECK(norm(cross(result.force_body_n, motion)) < 1e-12 * norm(result.force_body_n) * norm(motion));
    CHECK(dot(result.force_body_n, motion) < 0.0);  // drag
    CHECK(norm(result.moment_body_nm) == 0.0);
    CHECK(result.dynamic_pressure_pa == Catch::Approx(q));
}

TEST_CASE("ISS config: mass, drag, inertia, ports and primitives", "[iss][config]") {
    const auto iss = config::load_vehicle(config_dir, "iss");
    REQUIRE(iss.stages.size() == 1);
    const auto& s = iss.stages.front();
    CHECK(s.dry_mass_kg == Catch::Approx(460320.0));
    CHECK(s.aero.fixed_drag_area_m2 == Catch::Approx(1177.76));
    CHECK(s.aero.fixed_drag_coefficient == Catch::Approx(1.5));
    REQUIRE(s.inertia_kg_m2.has_value());
    REQUIRE(s.body_in_lvlh_rad.has_value());
    REQUIRE(s.docking_ports.size() == 2);
    CHECK(s.docking_ports[0].id == "ida2");
    CHECK(s.docking_ports[1].id == "ida3");
    for (const auto& p : s.docking_ports) {
        CHECK(norm(p.axis) == Catch::Approx(1.0));
        CHECK(std::abs(dot(p.axis, p.up)) < 1e-12);
    }
    CHECK(s.docking_ports[0].axis.x == Catch::Approx(1.0));   // forward: V-bar
    CHECK(s.docking_ports[1].axis.z == Catch::Approx(-1.0));  // zenith (body +z = nadir): R-bar
    CHECK(s.collision.size() >= 6);
}

TEST_CASE("Vehicle uses the fixed area and the explicit inertia", "[iss][vehicle]") {
    const auto cfg = config::load_vehicle(config_dir, "iss");
    const vehicle::Vehicle iss(cfg, config_dir);
    REQUIRE(iss.fixed_area_aero() != nullptr);
    const auto mp = iss.mass_properties();
    CHECK(mp.mass_kg == Catch::Approx(460320.0));
    const auto& inertia = *cfg.stages.front().inertia_kg_m2;
    CHECK(mp.inertia_kg_m2.x == Catch::Approx(inertia.x));
    CHECK(mp.inertia_kg_m2.y == Catch::Approx(inertia.y));
    CHECK(mp.inertia_kg_m2.z == Catch::Approx(inertia.z));
    const auto air = environment::AtmosphereState{3.0e-12, 0.0, 1000.0, 900.0};
    const auto drag = iss.hull_aero({7660.0, 0.0, 0.0}, air, false);  // flying +x through the air
    CHECK(drag.force_body_n.x == Catch::Approx(-0.5 * 3.0e-12 * 7660.0 * 7660.0 * 1.5 * 1177.76));
}

TEST_CASE("Rocket inertia is unchanged by the explicit-inertia path", "[iss][vehicle]") {
    const auto cfg = config::load_vehicle(config_dir, "crew_dragon");
    const vehicle::Vehicle dragon(cfg, config_dir);
    const auto mp = dragon.mass_properties();
    CHECK(mp.inertia_kg_m2.y == mp.inertia_kg_m2.z);  // axisymmetric as before
}

TEST_CASE("LVLH axes: x along the velocity, z to nadir, rate = orbit rate", "[iss][frames]") {
    const auto s = iss_like_state();
    const auto q = frames::lvlh_to_inertial(s);
    CHECK(dot(q.rotate({1.0, 0.0, 0.0}), math::normalized(s.velocity_mps)) == Catch::Approx(1.0).margin(1e-12));
    CHECK(dot(q.rotate({0.0, 0.0, 1.0}), -math::normalized(s.position_m)) == Catch::Approx(1.0).margin(1e-12));
    const Vec3 w = frames::lvlh_rate_inertial_radps(s);
    CHECK(norm(w) == Catch::Approx(norm(s.velocity_mps) / norm(s.position_m)).epsilon(1e-12));
}

TEST_CASE("Ideal LVLH attitude follows the orbit through ticks and coasts", "[iss][vehicle]") {
    const auto cfg = config::load_vehicle(config_dir, "iss");
    const auto s = iss_like_state();
    vehicle::RigidBodyState state{s.position_m, s.velocity_mps, frames::lvlh_to_inertial(s), {}};
    vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(cfg, config_dir), state);
    const math::Quat tea = math::Quat::from_axis_angle({0.0, 0.0, 1.0}, -3.0 * constants::deg_to_rad);
    body.set_ideal_lvlh_attitude(tea);
    CHECK(body.ideal_attitude());

    environment::Us76Atmosphere atmosphere;
    EarthPointMass gravity;
    FixedEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
    const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.5e8);
    for (int k = 0; k < 200; ++k) body.step(env, t0 + k * 0.005, 0.005, {});
    body.coast(env, t0 + 1.0, 1500.0);
    const auto& end = body.state();
    const math::StateVector end_sv{end.position_gcrf_m, end.velocity_gcrf_mps};
    const math::Quat expected = frames::lvlh_to_inertial(end_sv) * tea;
    CHECK(std::abs(std::abs(dot4(end.attitude, expected)) - 1.0) < 1e-12);
    const Vec3 w_body = end.attitude.inverse_rotate(frames::lvlh_rate_inertial_radps(end_sv));
    CHECK(norm(end.angular_velocity_body_radps - w_body) < 1e-15);
}
```

Put these stand-ins (as in `core/tests/test_flight_body.cpp`) and the quaternion helper into the anonymous namespace
at the top of the file:

```cpp
class EarthPointMass final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    math::Vec3 acceleration_mps2(const math::Vec3& r, time::Epoch) const override {
        const double d = norm(r);
        return r * (-constants::earth_gm_m3ps2 / (d * d * d));
    }
};
class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};
// Same rotation <=> |q1 . q2| = 1 (q and -q are the same rotation).
double dot4(const math::Quat& a, const math::Quat& b) { return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z; }
```

(Add `#include "starsim/environment/gravity.hpp"`, `"starsim/environment/atmosphere.hpp"`,
`"starsim/frames/body_orientation.hpp"`.)

- [ ] **Step 2: Run to verify it fails**

Add `test_iss_vehicle.cpp` to `core/tests/CMakeLists.txt`. Run:
`powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[iss]" > tmp/i1t1.txt 2>&1`; tail it.
Expected: compile errors (missing headers `fixed_area_aero.hpp`, `lvlh.hpp`, config fields).

- [ ] **Step 3: Implement**

`core/include/starsim/frames/lvlh.hpp`:

```cpp
#pragma once

#include "starsim/math/quat.hpp"
#include "starsim/math/state_vector.hpp"

namespace starsim::frames {

// Local vertical / local horizontal axes of an orbit state relative to its central body (inertial axes): x completes
// the triad (along the velocity on a circular orbit), y = -orbit normal, z = nadir. Returns the rotation LVLH -> inertial.
math::Quat lvlh_to_inertial(const math::StateVector& state);

// Angular velocity of the LVLH frame in inertial axes: (r x v) / r^2 (the orbit rate on a circular orbit).
math::Vec3 lvlh_rate_inertial_radps(const math::StateVector& state);

}  // namespace starsim::frames
```

`core/src/frames/lvlh.cpp`: move the body of `gnc::lvlh_attitude_gcrf` and its `from_columns` helper there;
`gnc::lvlh_attitude_gcrf(state)` becomes `return frames::lvlh_to_inertial(state);` (keep the gnc name: its callers stay
unchanged). Add `src/frames/lvlh.cpp` and `src/aero/fixed_area_aero.cpp` to `core/CMakeLists.txt`.

`core/include/starsim/aero/fixed_area_aero.hpp`:

```cpp
#pragma once

#include "starsim/aero/slender_body_aero.hpp"

namespace starsim::aero {

// A body whose drag is a fixed C_D A along the oncoming flow, independent of its attitude: a space station flown in a
// fixed attitude, its drag area and coefficient from its operator (ISS: the OEM header). No lift, no moment.
// Free-molecular flow is implied by the coefficient (orbit operators fit C_D ~ 2 for the free-molecular regime).
struct FixedAreaAero {
    double drag_area_m2 = 0.0;
    double drag_coefficient = 0.0;

    AeroResult compute(const math::Vec3& air_velocity_body_mps, double density_kg_m3, double speed_of_sound_mps) const;
};

}  // namespace starsim::aero
```

`core/src/aero/fixed_area_aero.cpp`:

```cpp
#include "starsim/aero/fixed_area_aero.hpp"

#include <cmath>

namespace starsim::aero {

AeroResult FixedAreaAero::compute(const math::Vec3& air_velocity_body_mps, double density_kg_m3,
                                  double speed_of_sound_mps) const {
    AeroResult r;
    const double speed = norm(air_velocity_body_mps);
    r.dynamic_pressure_pa = 0.5 * density_kg_m3 * speed * speed;
    r.mach = speed_of_sound_mps > 0.0 ? speed / speed_of_sound_mps : 0.0;
    r.rarefaction = 1.0;
    if (speed <= 0.0) return r;
    // As SlenderBodyAero: air_velocity_body is the body's motion relative to the air (FlightBody passes
    // relative_velocity); the drag opposes it.
    r.force_body_n = air_velocity_body_mps * (-r.dynamic_pressure_pa * drag_coefficient * drag_area_m2 / speed);
    return r;
}

}  // namespace starsim::aero
```

Config (`vehicle_config.hpp`): add to `AeroConfig`

```cpp
    double fixed_drag_area_m2 = 0.0;      // > 0: fixed C_D A drag (a station), replaces the slender-body aero
    double fixed_drag_coefficient = 0.0;
    double solar_area_m2 = 0.0;           // > 0: fixed area for solar radiation pressure (else the cylinder model)
```

add the two structs `DockingPortConfig`, `CollisionPrimitiveConfig` (fields as in Interfaces) and to `StageConfig`

```cpp
    std::optional<math::Vec3> inertia_kg_m2;      // principal moments of the dry stage about its own CG (else a cylinder)
    std::optional<math::Vec3> body_in_lvlh_rad;   // a station: ideal LVLH attitude, yaw / pitch / roll (TEA)
    std::vector<DockingPortConfig> docking_ports;
    std::vector<CollisionPrimitiveConfig> collision;
```

`load_stage` (vehicle_config.cpp): a local helper `vec3(doc, table, key)` that reads a 3-number array (fail
"<key> must be [x, y, z]"); then

```cpp
    if (auto aero = t["aero"].as_table()) { /* existing reads ... */
        s.aero.fixed_drag_area_m2 = doc.number_or(*aero, "fixed_drag_area_m2", 0.0);
        s.aero.fixed_drag_coefficient = doc.number_or(*aero, "fixed_drag_coefficient", 0.0);
        s.aero.solar_area_m2 = doc.number_or(*aero, "solar_area_m2", 0.0);
        if ((s.aero.fixed_drag_area_m2 > 0.0) != (s.aero.fixed_drag_coefficient > 0.0)) {
            doc.fail("stage.aero: fixed_drag_area_m2 and fixed_drag_coefficient go together");
        }
    }
    if (t.contains("inertia_kg_m2")) {
        const Vec3 i = vec3(doc, t, "inertia_kg_m2");
        if (!(i.x > 0.0 && i.y > 0.0 && i.z > 0.0)) doc.fail("inertia_kg_m2 must be positive");
        s.inertia_kg_m2 = i;
    }
    if (auto station = t["station"].as_table()) {
        const Vec3 deg = vec3(doc, *station, "body_in_lvlh_deg");  // yaw, pitch, roll [E] (torque equilibrium attitude)
        s.body_in_lvlh_rad = deg * constants::deg_to_rad;
    }
    if (auto ports = t["docking_port"].as_array()) {
        for (const auto& node : *ports) {
            const auto& p = *node.as_table();
            DockingPortConfig port{doc.text(p, "id"), doc.text(p, "name"), vec3(doc, p, "position_m"),
                                   vec3(doc, p, "axis"), vec3(doc, p, "up")};
            if (std::abs(norm(port.axis) - 1.0) > 1e-9 || std::abs(norm(port.up) - 1.0) > 1e-9 ||
                std::abs(dot(port.axis, port.up)) > 1e-9) {
                doc.fail("stage.docking_port '" + port.id + "': axis and up must be perpendicular unit vectors");
            }
            s.docking_ports.push_back(port);
        }
    }
    if (auto prims = t["collision"].as_array()) {
        for (const auto& node : *prims) {
            const auto& p = *node.as_table();
            CollisionPrimitiveConfig c;
            c.id = doc.text(p, "id");
            c.kind = doc.text(p, "kind");
            if (c.kind == "capsule") {
                c.a_m = vec3(doc, p, "a_m");
                c.b_m = vec3(doc, p, "b_m");
                c.radius_m = doc.positive(p, "radius_m");
            } else if (c.kind == "box") {
                c.a_m = vec3(doc, p, "centre_m");
                c.half_extents_m = vec3(doc, p, "half_extents_m");
            } else {
                doc.fail("stage.collision '" + c.id + "': kind must be \"capsule\" or \"box\"");
            }
            s.collision.push_back(c);
        }
    }
```

(Insert the aero lines into the existing `[stage.aero]` block of `load_stage`; read it first.)

Vehicle (`vehicle.hpp/.cpp`): member `std::unique_ptr<aero::FixedAreaAero> fixed_area_aero_;` and accessor; in
`rebuild_aero()` after the capsule block:

```cpp
    fixed_area_aero_.reset();
    if (const auto& low = stages_.front().config.aero; low.fixed_drag_area_m2 > 0.0) {
        fixed_area_aero_ = std::make_unique<aero::FixedAreaAero>(aero::FixedAreaAero{low.fixed_drag_area_m2,
                                                                                    low.fixed_drag_coefficient});
    }
```

`hull_aero`: first line `if (fixed_area_aero_) return fixed_area_aero_->compute(air_velocity_body_mps, air.density_kg_m3, air.speed_of_sound_mps);`.
`mass_properties`: give `Part` a trailing member `double izz = -1.0;  // < 0: = iyy (axisymmetric)`; for the dry part
of a stage with `config.inertia_kg_m2`, push `{dry, station, I.x, I.y, y, z, false, I.z}`; in the sum use
`izz += (p.izz < 0.0 ? p.iyy : p.izz) + p.mass * d * d;` and `mp.inertia_kg_m2 = {ixx, iyy, izz};`. Check
`Vehicle::copy` (`vehicle.cpp:~737`) copies `fixed_area_aero_` via `rebuild_aero()` (it calls it: nothing else needed).

FlightBody: `solar_radiation_acceleration` uses `stage.config.aero.solar_area_m2` of the lowest stage when > 0
instead of the cylinder area (`area = solar_area`, attitude-independent [E]). Ideal attitude:

```cpp
    // A station in ideal attitude (i1): the body axes follow LVLH (times body_in_lvlh) exactly after every step and
    // coast; contact torques do not turn it (its CMGs are taken as strong enough - documented simplification). The
    // translation stays free. The attitude enters no force of such a body (fixed-area drag and SRP).
    void set_ideal_lvlh_attitude(std::optional<math::Quat> body_in_lvlh) { ideal_lvlh_ = body_in_lvlh; snap_ideal_attitude(); }
    bool ideal_attitude() const { return ideal_lvlh_.has_value(); }
  private:
    void snap_ideal_attitude();
    std::optional<math::Quat> ideal_lvlh_;
```

```cpp
void FlightBody::snap_ideal_attitude() {
    if (!ideal_lvlh_) return;
    // LVLH about the GCRF origin: a station is Earth-referenced (the Moon's LVLH is not needed before a lunar station)
    const math::StateVector sv{state_.position_gcrf_m, state_.velocity_gcrf_mps};
    state_.attitude = frames::lvlh_to_inertial(sv) * *ideal_lvlh_;
    state_.angular_velocity_body_radps = state_.attitude.inverse_rotate(frames::lvlh_rate_inertial_radps(sv));
}
```

Call `snap_ideal_attitude()` at the end of `step()` and of `coast()` (after `state_ = unpack(result.y)`).

`config/vehicles/iss.toml`:

```toml
# International Space Station - the docking target of milestone i (spec docs/superpowers/specs/2026-10-19-iss-docking-design.md).
# Flown in ideal LVLH attitude (+XVV, body x forward, y starboard, z nadir); stations along body x from the aft end of
# the Russian segment. Mass, drag area and C_D are replaced by the OEM header when the mission uses an OEM.

[sources]
oem = "NASA JSC FOD/TOPO, ISS Trajectory Data (OEM header, file created 2026-10-14): MASS, DRAG_AREA, DRAG_COEFF"
wiki_iss = "https://en.wikipedia.org/wiki/International_Space_Station (commonly cited dimensions: 109 m x 73 m)"
nasa_ida = "NASA, International Docking Adapter (IDA-2 on Harmony forward 2016, IDA-3 on Harmony zenith 2019)"

[vehicle]
id = "iss"
name = "ISS"
display_name = "Raumstation"
propellant = "mmh_nto"                   # required by the loader; the station carries no modelled propellant
stages_bottom_to_top = ["station"]

[[stage]]
id = "station"
name = "International Space Station"
length_m = 73.0                          # [S:wiki_iss] pressurised length along body x
diameter_m = 4.2                         # [E] module diameter (knudsen length only; drag and SRP use fixed areas)
dry_mass_kg = 460320.0                   # [S:oem]
dry_cg_station_m = 38.0                  # [E] near the S0 truss over Destiny
# [E] principal moments: truss + arrays (~150 t over 109 m along y) and the module stack (~300 t over ~70 m along x);
# not used while the attitude is ideal (i1), needed by the compound body (i5).
inertia_kg_m2 = [1.6e8, 6.5e7, 2.1e8]
radiation_pressure_coefficient = 1.3     # [E]

[stage.aero]
reference_diameter_m = 4.2
fixed_drag_area_m2 = 1177.76             # [S:oem]
fixed_drag_coefficient = 1.5             # [S:oem]
solar_area_m2 = 2500.0                   # [E] sun-tracking arrays (8 wings ~ 35 x 12 m) seen on average; the OEM uses 0

[stage.station]
body_in_lvlh_deg = [-3.0, -6.0, 0.5]     # [E] yaw, pitch, roll: a typical torque equilibrium attitude (TEA)

# Ports: position in the stage frame (x station, y starboard, z nadir), outward docking axis, clocking reference.
[[stage.docking_port]]
id = "ida2"
name = "IDA-2 (Harmony vorn)"
position_m = [73.0, 0.0, 0.0]            # [E] forward end of PMA-2/IDA-2 on Harmony
axis = [1.0, 0.0, 0.0]                   # +XVV: V-bar approach
up = [0.0, 0.0, -1.0]                    # zenith

[[stage.docking_port]]
id = "ida3"
name = "IDA-3 (Harmony oben)"
position_m = [66.0, 0.0, -5.0]           # [E] Harmony zenith, PMA-3 + IDA-3
axis = [0.0, 0.0, -1.0]                  # zenith: R-bar approach
up = [1.0, 0.0, 0.0]                     # forward

# Collision primitives [E dimensions after wiki_iss / NASA module data], stage frame.
[[stage.collision]]
id = "russian_segment"                   # Zvezda + Zarya
kind = "capsule"
a_m = [0.0, 0.0, 0.0]
b_m = [26.0, 0.0, 0.0]
radius_m = 2.1

[[stage.collision]]
id = "us_modules"                        # Unity + Destiny + Harmony
kind = "capsule"
a_m = [26.0, 0.0, 0.0]
b_m = [69.0, 0.0, 0.0]
radius_m = 2.2

[[stage.collision]]
id = "columbus"
kind = "capsule"
a_m = [66.0, 2.2, 0.0]
b_m = [66.0, 9.0, 0.0]
radius_m = 2.25

[[stage.collision]]
id = "kibo"
kind = "capsule"
a_m = [66.0, -2.2, 0.0]
b_m = [66.0, -13.5, 0.0]
radius_m = 2.2

[[stage.collision]]
id = "truss"
kind = "box"
centre_m = [38.0, 0.0, -4.5]
half_extents_m = [2.5, 54.5, 2.5]

[[stage.collision]]
id = "arrays_port"                        # four wings, nominal (edge-on to the flow) [E]
kind = "box"
centre_m = [38.0, -38.0, -4.5]
half_extents_m = [17.0, 14.0, 0.2]

[[stage.collision]]
id = "arrays_starboard"
kind = "box"
centre_m = [38.0, 38.0, -4.5]
half_extents_m = [17.0, 14.0, 0.2]
```

- [ ] **Step 4: Run to verify it passes**

`tools/build.ps1 -NoBindings -TestFilter "[iss]"` -> all `[iss]` pass. Then `-TestFilter "[vehicle]"` and
`"[attitude]"` (the gnc LVLH move must not change anything) -> pass.

- [ ] **Step 5: Commit and push**

`-Fast` green, then `git add` the new/changed files and
`git commit -m "feat(vehicle): ISS vehicle with fixed drag area, inertia, ports and ideal LVLH attitude"`; push.

---

### Task 2: OEM parser, events and interpolation

**Files:**
- Create: `core/include/starsim/orbit/oem.hpp`, `core/src/orbit/oem.cpp`, `core/tests/test_oem.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`
- Fixture (exists): `tests/validation/fixtures/iss_oem/ISS.OEM_J2K_EPH_2026-10-14T18-30-01.txt`

**Interfaces:**
- Consumes: `time::TimeScaleConverter`, `time::UtcDateTime::parse_iso`.
- Produces (namespace `starsim::orbit`):

```cpp
struct OemError : std::runtime_error { using std::runtime_error::runtime_error; };

struct OemEvent {
    std::string name;            // e.g. "SPX-35 Docking", "Reboost"
    time::Epoch tig;             // time of ignition (TDB)
    double delta_v_mps = 0.0;    // magnitude as listed
    double apoapsis_height_m = 0.0;   // HA after the event
    double periapsis_height_m = 0.0;  // HP after the event
};

struct OemHeader {
    std::string object_name;
    std::string creation_date;   // as written (UTC)
    time::Epoch usable_start;
    time::Epoch usable_stop;
    std::optional<double> mass_kg, drag_area_m2, drag_coefficient, solar_radiation_area_m2, solar_radiation_coefficient;
};

class Oem {
public:
    static constexpr int interpolation_degree = 8;  // CCSDS OEM recommendation for Lagrange interpolation
    static Oem parse(std::istream& in, const time::TimeScaleConverter& time, const std::string& name = "OEM");
    static Oem load(const std::filesystem::path& file, const time::TimeScaleConverter& time);
    // For tests: a subset of rows (keep_row(i) true).
    Oem filtered(const std::function<bool(size_t)>& keep_row) const;

    const OemHeader& header() const { return header_; }
    const std::vector<OemEvent>& events() const { return events_; }
    size_t row_count() const { return epochs_.size(); }
    time::Epoch row_epoch(size_t i) const { return epochs_[i]; }
    const math::StateVector& row_state(size_t i) const { return states_[i]; }
    bool covers(time::Epoch epoch) const;  // inside the usable window
    // GCRF (EME2000) state; throws OemError outside the usable window.
    math::StateVector state_gcrf(time::Epoch epoch) const;
private:
    OemHeader header_;
    std::vector<OemEvent> events_;
    std::vector<time::Epoch> epochs_;
    std::vector<math::StateVector> states_;
    std::vector<size_t> segment_start_;  // first row of each run of equal steps
};
```

- [ ] **Step 1: Write the failing tests** - `core/tests/test_oem.cpp`:

```cpp
#include <cmath>
#include <fstream>
#include <sstream>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/orbit/oem.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
const std::string fixture = std::string(STARSIM_FIXTURE_DIR) + "/iss_oem/ISS.OEM_J2K_EPH_2026-10-14T18-30-01.txt";

std::string read_all(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream s;
    s << in.rdbuf();
    return s.str();
}
}  // namespace

TEST_CASE("OEM header, window and rows", "[iss][oem]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto oem = orbit::Oem::load(fixture, time);
    CHECK(oem.header().object_name == "ISS");
    CHECK(oem.header().creation_date == "2026-10-14T18:30:01.420");
    REQUIRE(oem.header().mass_kg);
    CHECK(*oem.header().mass_kg == Catch::Approx(460320.0));
    CHECK(*oem.header().drag_area_m2 == Catch::Approx(1177.76));
    CHECK(*oem.header().drag_coefficient == Catch::Approx(1.5));
    CHECK(*oem.header().solar_radiation_area_m2 == 0.0);
    CHECK(oem.row_count() == 5403);
    const auto start = time.from_utc({2026, 10, 14, 12, 0, 0.0});
    CHECK(oem.header().usable_start == start);
    CHECK(oem.row_epoch(0) == start);
    // first row in metres: -2107.105516188220 km
    CHECK(oem.row_state(0).position_m.x == Catch::Approx(-2107105.516188220).margin(1e-6));
    CHECK(oem.row_state(0).velocity_mps.z == Catch::Approx(-6003.96973354998).margin(1e-9));
    CHECK(oem.covers(start + 86400.0));
    CHECK_FALSE(oem.covers(start - 1.0));
    CHECK_THROWS_AS(oem.state_gcrf(start - 1.0), orbit::OemError);
}

TEST_CASE("OEM event table", "[iss][oem]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto oem = orbit::Oem::load(fixture, time);
    REQUIRE(oem.events().size() == 1);
    const auto& e = oem.events().front();
    CHECK(e.name == "SPX-35 Docking");
    CHECK(e.tig == time.from_utc({2026, 10, 15, 15, 0, 0.0}));  // day 288 of 2026
    CHECK(e.delta_v_mps == 0.0);
    CHECK(e.apoapsis_height_m == Catch::Approx(421.1e3));
    CHECK(e.periapsis_height_m == Catch::Approx(418.6e3));
}

TEST_CASE("OEM parses LF and CRLF alike", "[iss][oem]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    std::string text = read_all(fixture);
    std::string lf;
    for (char c : text) if (c != '\r') lf.push_back(c);
    std::string crlf;
    for (char c : lf) { if (c == '\n') crlf += "\r\n"; else crlf.push_back(c); }
    std::istringstream a(lf), b(crlf);
    const auto oem_lf = orbit::Oem::parse(a, time);
    const auto oem_crlf = orbit::Oem::parse(b, time);
    CHECK(oem_lf.row_count() == oem_crlf.row_count());
    CHECK(oem_lf.events().size() == oem_crlf.events().size());
    CHECK(oem_lf.header().creation_date == oem_crlf.header().creation_date);
}

TEST_CASE("OEM interpolation reproduces left-out rows", "[iss][oem][validation]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto oem = orbit::Oem::load(fixture, time);
    // every other row removed: 8 min steps; the removed rows are the truth
    const auto thin = oem.filtered([](size_t i) { return i % 2 == 0; });
    double worst_m = 0.0, worst_mps = 0.0;
    for (size_t i = 1; i + 1 < oem.row_count(); i += 2) {
        const auto s = thin.state_gcrf(oem.row_epoch(i));
        worst_m = std::max(worst_m, norm(s.position_m - oem.row_state(i).position_m));
        worst_mps = std::max(worst_mps, norm(s.velocity_mps - oem.row_state(i).velocity_mps));
    }
    if (std::getenv("STARSIM_VERBOSE")) std::printf("OEM leave-one-out: %.4f m, %.6f m/s\n", worst_m, worst_mps);
    CHECK(worst_m < 1.0);       // at 8 min; the 4 min grid itself is far better
    CHECK(worst_mps < 1e-3);
    // on a row the interpolation returns the row
    CHECK(norm(oem.state_gcrf(oem.row_epoch(100)).position_m - oem.row_state(100).position_m) < 1e-6);
}

TEST_CASE("OEM interpolation does not straddle a step change", "[iss][oem]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto oem = orbit::Oem::load(fixture, time);
    // the grid shifts by 1.5 min at 2026-10-17T17:57:30 (row index 1170): a window must lie on one side
    const auto seam = time.from_utc({2026, 10, 17, 17, 57, 30.0});
    size_t k = 0;
    while (oem.row_epoch(k) < seam) ++k;
    REQUIRE(oem.row_epoch(k) == seam);
    // just before the seam: equal to an interpolation over the rows before it only
    const auto before_only = oem.filtered([k](size_t i) { return i < k; });
    const auto t = oem.row_epoch(k - 1) + 30.0;
    CHECK(norm(oem.state_gcrf(t).position_m - before_only.state_gcrf(t).position_m) < 1e-6);
}
```

(`before_only` ends at row k-1, so its usable window must be clipped to its last row in `filtered`; see Step 3.)

- [ ] **Step 2: Run to verify it fails** - add `test_oem.cpp`; `-TestFilter "[oem]"`: compile error (no `oem.hpp`).

- [ ] **Step 3: Implement** `core/src/orbit/oem.cpp`:

- Read line by line, strip a trailing `'\r'`, trim. Keywords `KEY = VALUE` before `META_START` / inside
  `META_START...META_STOP` (`OBJECT_NAME`, `CREATION_DATE`, `USEABLE_START_TIME`, `USEABLE_STOP_TIME`, `REF_FRAME`
  must be `EME2000` (else `OemError("unsupported REF_FRAME ...")`), `TIME_SYSTEM` must be `UTC`, `CENTER_NAME`
  must be `Earth`).
- `COMMENT KEY=VALUE` (no spaces around '=': `MASS=460320.00`) for `MASS`, `DRAG_AREA`, `DRAG_COEFF`,
  `SOLAR_RAD_AREA`, `SOLAR_RAD_COEFF`.
- Event table: between `COMMENT TRAJECTORY EVENT SUMMARY:` and `COMMENT End sequence of events`, a row matches
  `std::regex R"(^COMMENT\s+(.*?)\s+(\d{3}):(\d{2}):(\d{2}):(\d{2}(?:\.\d+)?)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*$)"`;
  lines with "(" (imperial units), "|", "=" are skipped. TIG: day-of-year in the year of `USEABLE_START_TIME`; a day
  number smaller than the start's day-of-year is in the next year. Convert to a `UtcDateTime` (month/day from
  day-of-year, leap years) and then to `Epoch`.
- Data rows: 7 whitespace-separated fields, the first parses with `UtcDateTime::parse_iso`; km -> m, km/s -> m/s.
  Rows must be strictly increasing in time (`OemError` otherwise).
- Segments: a new segment starts at row i when `|dt(i-1,i) - dt(i-2,i-1)| > 1e-3 s` (the step changes). The row
  at the seam belongs to the new segment.
- `state_gcrf(t)`: throw outside `[usable_start, usable_stop]`; find the segment containing t (the last segment
  start <= t); inside it choose the `interpolation_degree + 1` rows centred on t, shifted to stay inside the segment
  (a segment shorter than 9 rows uses all its rows); Lagrange-interpolate position and velocity component-wise in
  `double` with time measured from the window's first row (seconds, avoids cancellation).
- `filtered(keep)`: copy header and events, keep rows, recompute segments, clip `usable_start/stop` to the kept
  rows' first/last epoch.
- `load(file)`: open in binary, `OemError("OEM file not found: ...")` if missing.

- [ ] **Step 4: Run to verify it passes** - `-TestFilter "[oem]"` green; run once with `STARSIM_VERBOSE=1` and note
  the leave-one-out numbers in the ledger.

- [ ] **Step 5: Commit and push** - `-Fast`, `git commit -m "feat(orbit): CCSDS OEM parser with events and segment-wise Lagrange interpolation"`; push.

---

### Task 3: OEM archive and data pipeline

**Files:**
- Create: `tools/data_pipeline/iss_oem.py`, `tools/data_pipeline/test_iss_oem.py`
- Modify: `tools/data_pipeline/fetch.py`, `tools/data_pipeline/sources.toml`
- Modify: `core/include/starsim/orbit/oem.hpp`, `core/src/orbit/oem.cpp` (`OemArchive`), `core/tests/test_oem.cpp`

**Interfaces:**
- Produces:

```cpp
// All OEM files of a directory (the archive data/iss_oem/: one file per CREATION_DATE).
class OemArchive {
public:
    // Files that fail to parse are skipped and listed in skipped(); a missing directory gives an empty archive.
    static OemArchive scan(const std::filesystem::path& dir, const time::TimeScaleConverter& time);
    // The newest file (by CREATION_DATE) whose usable window contains `epoch`, or nullptr.
    const Oem* for_epoch(time::Epoch epoch) const;
    // The newest file of all (for the synthetic orbit's mean elements), or nullptr.
    const Oem* latest() const;
    size_t size() const { return files_.size(); }
    const std::vector<std::string>& skipped() const { return skipped_; }
private:
    std::vector<Oem> files_;  // sorted by creation date, oldest first
    std::vector<std::string> skipped_;
};
```

- Python: `iss_oem.archive_name(text: str) -> str`, `iss_oem.store(data: bytes, archive_dir: Path) -> Path | None`,
  `iss_oem.fetch(source: dict, repo_root: Path, download) -> None`.

- [ ] **Step 1: Write the failing tests**

`tools/data_pipeline/test_iss_oem.py`:

```python
"""Unit tests of the ISS OEM archive (naming, de-duplication). No network.

Run: python tools/data_pipeline/test_iss_oem.py
"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import iss_oem  # noqa: E402

HEADER = b"CCSDS_OEM_VERS = 2.0\r\nCREATION_DATE  = 2026-10-14T18:30:01.420\r\nORIGINATOR     = NASA/JSC/FOD/TOPO\r\n"


class ArchiveName(unittest.TestCase):
    def test_creation_date_without_colons(self):
        self.assertEqual(iss_oem.archive_name(HEADER.decode()), "ISS.OEM_J2K_EPH_2026-10-14T18-30-01.txt")

    def test_missing_creation_date_is_an_error(self):
        with self.assertRaises(ValueError):
            iss_oem.archive_name("CCSDS_OEM_VERS = 2.0\n")


class Store(unittest.TestCase):
    def test_new_version_is_stored_once_byte_exact(self):
        with tempfile.TemporaryDirectory() as d:
            first = iss_oem.store(HEADER, Path(d))
            self.assertIsNotNone(first)
            self.assertEqual(first.read_bytes(), HEADER)
            self.assertIsNone(iss_oem.store(HEADER, Path(d)))
            newer = HEADER.replace(b"18:30:01", b"19:00:00")
            self.assertIsNotNone(iss_oem.store(newer, Path(d)))
            self.assertEqual(len(list(Path(d).glob("ISS.OEM_J2K_EPH_*.txt"))), 2)


if __name__ == "__main__":
    unittest.main()
```

Append to `core/tests/test_oem.cpp` (unique temp directory per test case: ctest runs cases in parallel):

```cpp
TEST_CASE("OEM archive picks the newest file covering the epoch", "[iss][oem]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto dir = std::filesystem::path(STARSIM_OUTPUT_DIR) / "oem_archive_case";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const std::string text = read_all(fixture);
    // an older version: same rows, earlier creation date
    std::string older = text;
    older.replace(older.find("2026-10-14T18:30:01.420"), 23, "2026-10-13T18:30:01.420");
    std::ofstream(dir / "ISS.OEM_J2K_EPH_2026-10-14T18-30-01.txt", std::ios::binary) << text;
    std::ofstream(dir / "ISS.OEM_J2K_EPH_2026-10-13T18-30-01.txt", std::ios::binary) << older;
    std::ofstream(dir / "ISS.OEM_J2K_EPH_broken.txt", std::ios::binary) << "not an OEM";
    const auto archive = orbit::OemArchive::scan(dir, time);
    CHECK(archive.size() == 2);
    CHECK(archive.skipped().size() == 1);
    const auto t = time.from_utc({2026, 10, 20, 0, 0, 0.0});
    REQUIRE(archive.for_epoch(t) != nullptr);
    CHECK(archive.for_epoch(t)->header().creation_date == "2026-10-14T18:30:01.420");
    CHECK(archive.for_epoch(time.from_utc({2026, 12, 1, 0, 0, 0.0})) == nullptr);
    CHECK(archive.latest()->header().creation_date == "2026-10-14T18:30:01.420");
    CHECK(orbit::OemArchive::scan(dir / "missing", time).size() == 0);
    std::filesystem::remove_all(dir);
}
```

- [ ] **Step 2: Run to verify both fail** - `python tools/data_pipeline/test_iss_oem.py` (ImportError) and
  `-TestFilter "[oem]"` (no `OemArchive`).

- [ ] **Step 3: Implement**

`tools/data_pipeline/iss_oem.py`:

```python
"""NASA ISS trajectory data (CCSDS OEM): fetch the current file and keep every version.

The file at the source changes with every NASA update (~3x a week, 15 days ahead). Each fetched version is stored as
data/iss_oem/ISS.OEM_J2K_EPH_<CREATION_DATE>.txt (colons replaced), so past dates stay covered by the newest
prediction made before them (core: orbit::OemArchive).
"""

from __future__ import annotations

import re
from pathlib import Path

_CREATION = re.compile(r"^CREATION_DATE\s*=\s*(\S+)", re.M)


def archive_name(text: str) -> str:
    match = _CREATION.search(text)
    if not match:
        raise ValueError("OEM without CREATION_DATE")
    stamp = match.group(1).split(".")[0].replace(":", "-")
    return f"ISS.OEM_J2K_EPH_{stamp}.txt"


def store(data: bytes, archive_dir: Path) -> Path | None:
    """Stores a version byte-exact; returns its path, or None if that version is already archived."""
    archive_dir.mkdir(parents=True, exist_ok=True)
    target = archive_dir / archive_name(data.decode("ascii", errors="replace"))
    if target.exists():
        return None
    target.write_bytes(data)
    return target


def fetch(source: dict, repo_root: Path, download) -> None:
    """Always downloads (the current file is a moving target) and archives a new version."""
    archive_dir = repo_root / source["target"]
    partial = archive_dir / "current.download"
    download(source["url"], partial)
    try:
        stored = store(partial.read_bytes(), archive_dir)
    finally:
        partial.unlink(missing_ok=True)
    print(f"  archived {stored.name}" if stored else "  this version is already archived")
```

`fetch.py`: `import iss_oem` next to `import lunar_tile`, and in the loop before the `target.exists()` check:

```python
        if source.get("kind") == "iss_oem":  # ISS OEM: always fetched, every version archived (iss_oem.py)
            iss_oem.fetch(source, REPO_ROOT, download)
            continue
```

`sources.toml` (append):

```toml
[[source]]
id = "iss_oem"
name = "NASA ISS Trajectory Data (CCSDS OEM, EME2000, ~15 days incl. planned manoeuvres) - every fetched version archived"
url = "https://nasa-public-data.s3.amazonaws.com/iss-coords/current/ISS_OEM/ISS.OEM_J2K_EPH.txt"
target = "data/iss_oem"
kind = "iss_oem"
license = "NASA data: public domain (NASA data and images are not copyrighted)"
credit = "NASA Johnson Space Center, Flight Operations Directorate (TOPO): ISS Trajectory Data"
```

Then check `tools/data_pipeline/credits.py` still runs (`python tools/data_pipeline/credits.py`) and regenerate
`CREDITS.md` if that is how the repo keeps it (read the script's docstring).

`OemArchive::scan`: iterate `std::filesystem::directory_iterator` over `*.txt`; `Oem::load` each in a try/catch
(`OemError`, `std::exception`) -> `skipped_` with the file name and reason; sort by `header().creation_date`
(ISO strings sort chronologically). `for_epoch`: search from the newest down for `covers(epoch)`.

- [ ] **Step 4: Run to verify they pass** - both test runs green. Then a real fetch:
  `python tools/data_pipeline/fetch.py iss_oem` -> prints "archived ..." (or "already archived"); a second run
  prints "already archived". Note the archived file name in the ledger.

- [ ] **Step 5: Commit and push** - `-Fast`; `git commit -m "feat(data): ISS OEM download with a version archive"`; push.

---

### Task 4: Synthetic reference orbit

**Files:**
- Create: `core/include/starsim/orbit/synthetic_station_orbit.hpp`, `core/src/orbit/synthetic_station_orbit.cpp`,
  `core/tests/test_synthetic_station_orbit.cpp`
- Modify: `config/vehicles/iss.toml` (`[stage.station.reference_orbit]`), `core/include/starsim/config/vehicle_config.hpp`,
  `core/src/config/vehicle_config.cpp`, `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `orbit::Oem` (Task 2), `orbit::elements_from_state`, `orbit::state_from_elements`
  (`orbit/kepler.hpp`), `constants::earth_gm_m3ps2`, `constants::earth_j2`, `constants::wgs84_semi_major_axis_m`.
- Produces:

```cpp
namespace starsim::orbit {
// Mean elements of a station orbit at `epoch` (J2 short-period terms averaged out over one revolution).
struct StationMeanElements {
    time::Epoch epoch;
    double semi_major_axis_m = 0.0;
    double eccentricity = 0.0;
    double inclination_rad = 0.0;
    double raan_rad = 0.0;
    double argument_of_perigee_rad = 0.0;
    double argument_of_latitude_rad = 0.0;
};
// Secular J2 rates (rad/s) of the RAAN, the argument of perigee and the mean argument of latitude.
struct J2Rates { double raan_radps, perigee_radps, latitude_radps; };
J2Rates j2_secular_rates(const StationMeanElements& mean, double gm_m3ps2, double j2, double radius_m);
// Averages the osculating elements of the OEM rows over one revolution from its first row (RAAN and argument of
// latitude unwrapped and taken at the revolution's middle row).
StationMeanElements mean_elements_from_oem(const Oem& oem, double gm_m3ps2);
// Mean elements advanced to `epoch` by the J2 rates; the argument of latitude is set by the scenario when given.
StationMeanElements advance_mean_elements(const StationMeanElements& mean, time::Epoch epoch, double gm_m3ps2, double j2,
                                          double radius_m, std::optional<double> argument_of_latitude_rad = {});
// The state of mean elements used as osculating ones [E: the J2 short-period terms, ~+-10 km in radius, are not
// restored; documented]. GCRF.
math::StateVector synthetic_station_state(const StationMeanElements& mean, double gm_m3ps2);
}
```

- `config::StationReferenceOrbitConfig{double altitude_m; double inclination_rad; double raan_rad; std::string reference_utc;}`
  as `StageConfig::reference_orbit` (`std::optional`), read from `[stage.station.reference_orbit]`
  (`altitude_km`, `inclination_deg`, `raan_deg`, `reference_utc`).

- [ ] **Step 1: Write the failing tests** - `core/tests/test_synthetic_station_orbit.cpp`:

```cpp
#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/oem.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
const std::string fixture = std::string(STARSIM_FIXTURE_DIR) + "/iss_oem/ISS.OEM_J2K_EPH_2026-10-14T18-30-01.txt";
constexpr double gm = constants::earth_gm_m3ps2;
constexpr double re = constants::wgs84_semi_major_axis_m;
constexpr double j2 = constants::earth_j2;
constexpr double deg = constants::deg_to_rad;

double unwrap_delta(double a, double b) {  // a - b in (-pi, pi]
    return std::remainder(a - b, constants::two_pi);
}
}  // namespace

TEST_CASE("J2 RAAN rate of an ISS orbit is about -5 deg/day", "[iss][synthetic]") {
    orbit::StationMeanElements m;
    m.semi_major_axis_m = re + 415.0e3;
    m.inclination_rad = 51.64 * deg;
    const auto rates = orbit::j2_secular_rates(m, gm, j2, re);
    CHECK(rates.raan_radps * 86400.0 / deg == Catch::Approx(-5.0).margin(0.1));
    const double n = std::sqrt(gm / std::pow(m.semi_major_axis_m, 3));
    CHECK(rates.latitude_radps == Catch::Approx(n).epsilon(2e-3));  // J2 changes it by ~0.1 %
}

TEST_CASE("Mean elements of the OEM: ISS altitude and inclination", "[iss][synthetic]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto oem = orbit::Oem::load(fixture, time);
    const auto mean = orbit::mean_elements_from_oem(oem, gm);
    CHECK((mean.semi_major_axis_m - re) / 1e3 == Catch::Approx(413.0).margin(8.0));  // OEM event: 418.6 x 421.1 km heights
    CHECK(mean.inclination_rad / deg == Catch::Approx(51.64).margin(0.02));
    CHECK(mean.eccentricity < 0.003);
}

TEST_CASE("J2-advanced RAAN matches the OEM's own node drift over 14 days", "[iss][synthetic][validation]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto oem = orbit::Oem::load(fixture, time);
    const auto mean = orbit::mean_elements_from_oem(oem, gm);
    // the OEM's last revolution: mean elements of a filtered copy starting 14 days later
    const auto late_start = oem.row_epoch(0) + 14.0 * 86400.0;
    const auto late = orbit::mean_elements_from_oem(oem.filtered([&](size_t i) { return oem.row_epoch(i) >= late_start; }), gm);
    const auto predicted = orbit::advance_mean_elements(mean, late.epoch, gm, j2, re);
    const double error_deg = unwrap_delta(predicted.raan_rad, late.raan_rad) / deg;
    if (std::getenv("STARSIM_VERBOSE")) std::printf("RAAN after 14 d: J2 %.4f deg off the OEM\n", error_deg);
    CHECK(std::abs(error_deg) < 0.2);  // [E] J2 only vs the operator's full field: ~70 deg of drift, 0.3 %
}

TEST_CASE("Synthetic state: the scenario's argument of latitude, the mean orbit", "[iss][synthetic]") {
    orbit::StationMeanElements m;
    m.semi_major_axis_m = re + 415.0e3;
    m.eccentricity = 0.0005;
    m.inclination_rad = 51.64 * deg;
    m.raan_rad = 120.0 * deg;
    m.argument_of_perigee_rad = 80.0 * deg;
    m.argument_of_latitude_rad = 10.0 * deg;
    const auto later = orbit::advance_mean_elements(m, m.epoch + 3600.0, gm, j2, re, 200.0 * deg);
    CHECK(later.argument_of_latitude_rad == Catch::Approx(200.0 * deg));
    const auto s = orbit::synthetic_station_state(later, gm);
    const auto el = orbit::elements_from_state(s, gm);
    CHECK(el.inclination_rad == Catch::Approx(m.inclination_rad).margin(1e-12));
    CHECK(unwrap_delta(el.raan_rad, later.raan_rad) == Catch::Approx(0.0).margin(1e-12));
    CHECK(unwrap_delta(el.argument_of_periapsis_rad + el.true_anomaly_rad, 200.0 * deg) == Catch::Approx(0.0).margin(1e-9));
}
```

- [ ] **Step 2: Run to verify it fails** - `-TestFilter "[synthetic]"`: compile error.

- [ ] **Step 3: Implement** `synthetic_station_orbit.cpp`:
  - `j2_secular_rates`: `n = sqrt(gm/a^3)`, `p = a(1-e^2)`, `k = 1.5 J2 (R/p)^2 n`;
    `raan = -k cos i`, `perigee = k (2 - 2.5 sin^2 i)`, `latitude = n + k [sqrt(1-e^2)(1 - 1.5 sin^2 i) + (2 - 2.5 sin^2 i)]`
    (mean anomaly rate plus perigee rate: the mean argument of latitude). Source: Vallado, Fundamentals of
    Astrodynamics, 4th ed., eq. 9-41 [S] - note it in a comment.
  - `mean_elements_from_oem`: rows from the first one while `t - t0 < period` (period from the first row's osculating
    `a`); average `a`, `e`, `i`; RAAN, argument of perigee and argument of latitude unwrapped along the rows; take
    RAAN and perigee as the average, the argument of latitude at the middle row (its epoch = `mean.epoch`).
    Fewer than 10 rows: `OemError("OEM too short for mean elements")`.
  - `advance_mean_elements`: add rates x dt to RAAN, perigee and latitude (wrapped to [0, 2 pi)), replace latitude when
    given; `epoch` updated.
  - `synthetic_state`: `KeplerianElements{a, e, i, raan, perigee, true anomaly = latitude - perigee}` (mean anomaly ~
    true anomaly at e < 0.003 [E]; document) -> `state_from_elements`.

  Config: `[stage.station.reference_orbit]` in `iss.toml`:

```toml
[stage.station.reference_orbit]            # synthetic orbit without any OEM (fidelity flag iss_orbit = "synthetic")
altitude_km = 415.0                        # [E] typical ISS mean altitude 2024-2026
inclination_deg = 51.64                    # [S:wiki_iss]
raan_deg = 0.0                             # [D] replace in Step 4 by the fixture's mean RAAN at reference_utc
reference_utc = "2026-10-14T12:00:00"      # the fixture's first row
```

  Loader: `StationReferenceOrbitConfig` with the four fields (strings kept as text; converted by `sim/station_start`).

- [ ] **Step 4: Run to verify it passes**, then set `raan_deg`: run the mean-elements test once with a temporary
  `printf` of `mean.raan_rad / deg` advanced back to `reference_utc` (or print it from the RAAN test with
  `STARSIM_VERBOSE`), write the value (3 decimals) into `iss.toml` with the tag `[D] mean RAAN of the 2026-10-14 OEM`,
  and remove the temporary print. Ledger: the printed RAAN error after 14 days.

- [ ] **Step 5: Commit and push** - `-Fast`; `git commit -m "feat(orbit): synthetic station orbit from OEM mean elements and J2 rates"`; push.

---

### Task 5: The ISS in the flight session - start state, impulses, OEM validation

**Files:**
- Create: `core/include/starsim/sim/station_start.hpp`, `core/src/sim/station_start.cpp`, `core/tests/test_iss_station.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`, `core/CMakeLists.txt`,
  `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `orbit::Oem`, `orbit::OemArchive`, `orbit::OemEvent` (Tasks 2-3), `orbit::advance_mean_elements`,
  `orbit::synthetic_station_state`, `orbit::mean_elements_from_oem` (Task 4), `FlightBody::set_ideal_lvlh_attitude`,
  `StageConfig::body_in_lvlh_rad`, `StageConfig::reference_orbit` (Task 1, 4).
- Produces:

```cpp
namespace starsim::sim {
enum class IssOrbitSource { Oem, Synthetic, Auto };  // fidelity flag iss_orbit
IssOrbitSource parse_iss_orbit_source(std::string_view text);  // "oem" | "synthetic" | "auto"; std::invalid_argument

// An impulsive velocity change of a station (an OEM reboost / debris avoidance) along its velocity.
struct StationImpulse {
    time::Epoch epoch;
    double delta_v_mps = 0.0;  // signed: + along the velocity
    std::string name;
};

struct StationStart {
    config::VehicleConfig vehicle;          // OEM header values applied (mass, drag area, C_D) when from the OEM
    math::StateVector state_gcrf;
    std::vector<StationImpulse> impulses;   // events with DV > 0 after the start epoch
    bool from_oem = false;
    std::string note;                       // German: which source was used (and why), shown by the mission
};

// The ISS at `epoch`. Oem: the archive file covering the epoch, else std::runtime_error (German message). Synthetic:
// mean elements of archive.latest() (or the config's reference orbit without any file), advanced by J2; the argument
// of latitude from the scenario (default: the advanced one). Auto: Oem where covered, else Synthetic (note says so).
StationStart iss_start(const config::VehicleConfig& iss, const orbit::OemArchive& archive,
                       const time::TimeScaleConverter& time, time::Epoch epoch, IssOrbitSource source,
                       std::optional<double> argument_of_latitude_rad = {});
}
```

FlightSession:

```cpp
    // A space station (a vehicle with [stage.station]) as an uncontrolled body in ideal LVLH attitude; `impulses`
    // are applied at their epochs (a coast stops there). Call after place_body / launch_from_pad / place_in_orbit -
    // they clear all bodies, the station included. Returns its body index. Throws std::logic_error for a vehicle
    // without [stage.station] or when a station exists already.
    size_t add_station(const config::VehicleConfig& config, const math::StateVector& state_gcrf,
                       std::vector<StationImpulse> impulses = {});
    std::optional<size_t> station_index() const;
    // A session that starts with a station only (no body placed yet); throws std::logic_error while bodies exist.
    void set_epoch(time::Epoch epoch);
    // Fidelity switch: solar radiation pressure on all bodies (default on; the OEM validation turns it off).
    void set_solar_radiation_pressure(bool on) { env_.solar_radiation_pressure = on; }
```

(`StationImpulse` lives in `sim/station_start.hpp`; `flight_session.hpp` includes it.)

- [ ] **Step 1: Write the failing tests** - `core/tests/test_iss_station.cpp`:

```cpp
#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/orbit/oem.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/station_start.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
const std::filesystem::path fixture_dir = std::filesystem::path(STARSIM_FIXTURE_DIR) / "iss_oem";

// Radial / along-track / cross-track components of `d` in the frame of state `s`.
math::Vec3 rtn(const math::StateVector& s, const math::Vec3& d) {
    const auto r = math::normalized(s.position_m);
    const auto n = math::normalized(cross(s.position_m, s.velocity_mps));
    const auto t = cross(n, r);
    return {dot(d, r), dot(d, t), dot(d, n)};
}
}  // namespace

TEST_CASE("ISS start: OEM, synthetic and auto", "[iss][station]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto archive = orbit::OemArchive::scan(fixture_dir, time);
    REQUIRE(archive.size() == 1);
    const auto iss = config::load_vehicle(config_dir, "iss");
    const auto inside = time.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto outside = time.from_utc({2027, 3, 1, 0, 0, 0.0});

    const auto a = sim::iss_start(iss, archive, time, inside, sim::IssOrbitSource::Oem);
    CHECK(a.from_oem);
    CHECK(norm(a.state_gcrf.position_m - archive.for_epoch(inside)->state_gcrf(inside).position_m) < 1e-6);
    CHECK(a.vehicle.stages.front().dry_mass_kg == Catch::Approx(460320.0));

    CHECK_THROWS_AS(sim::iss_start(iss, archive, time, outside, sim::IssOrbitSource::Oem), std::runtime_error);

    const auto b = sim::iss_start(iss, archive, time, outside, sim::IssOrbitSource::Auto);
    CHECK_FALSE(b.from_oem);
    CHECK_FALSE(b.note.empty());
    const double h = norm(b.state_gcrf.position_m) - constants::wgs84_semi_major_axis_m;
    CHECK(h / 1e3 == Catch::Approx(413.0).margin(15.0));

    const auto c = sim::iss_start(iss, archive, time, inside, sim::IssOrbitSource::Synthetic, 1.0);
    CHECK_FALSE(c.from_oem);
    const auto d = sim::iss_start(iss, orbit::OemArchive{}, time, outside, sim::IssOrbitSource::Synthetic);
    CHECK_FALSE(d.from_oem);  // the config's reference orbit
    CHECK(sim::parse_iss_orbit_source("auto") == sim::IssOrbitSource::Auto);
    CHECK_THROWS_AS(sim::parse_iss_orbit_source("tle"), std::invalid_argument);
}

TEST_CASE("A station alone in a session coasts in ideal attitude", "[iss][station]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    const auto archive = orbit::OemArchive::scan(fixture_dir, time);
    const auto t0 = time.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto start = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, time, t0, sim::IssOrbitSource::Oem);
    sim::FlightSession session(ephemeris, earth, config_dir);
    session.set_epoch(t0);  // a session that starts with a station only (Step 3)
    const size_t index = session.add_station(start.vehicle, start.state_gcrf, start.impulses);
    CHECK(session.station_index() == index);
    session.advance_to(t0 + 3600.0, 50);  // 50 calls at most: coasting, not 720,000 ticks
    CHECK(session.epoch() - t0 == Catch::Approx(3600.0).margin(1e-6));
    CHECK(session.body(index).ideal_attitude());
}

TEST_CASE("Placing a vehicle clears the station", "[iss][station]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    const auto archive = orbit::OemArchive::scan(fixture_dir, time);
    const auto t0 = time.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto start = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, time, t0, sim::IssOrbitSource::Oem);
    sim::FlightSession session(ephemeris, earth, config_dir);
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    session.place_in_orbit(dragon, {start.state_gcrf.position_m * 0.99, start.state_gcrf.velocity_mps}, t0);
    session.add_station(start.vehicle, start.state_gcrf, start.impulses);
    REQUIRE(session.station_index());
    CHECK(session.body_count() == 2);
    session.place_in_orbit(dragon, {start.state_gcrf.position_m * 0.99, start.state_gcrf.velocity_mps}, t0);
    CHECK_FALSE(session.station_index());
    CHECK(session.body_count() == 1);
    CHECK_THROWS_AS(session.add_station(dragon, start.state_gcrf), std::logic_error);  // no [stage.station]
}

TEST_CASE("A station impulse changes the velocity at its epoch", "[iss][station]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    const auto archive = orbit::OemArchive::scan(fixture_dir, time);
    const auto t0 = time.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto start = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, time, t0, sim::IssOrbitSource::Oem);
    sim::FlightSession with(ephemeris, earth, config_dir), without(ephemeris, earth, config_dir);
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    for (auto* s : {&with, &without}) s->place_in_orbit(dragon, {start.state_gcrf.position_m * 0.98, start.state_gcrf.velocity_mps}, t0);
    const auto i = with.add_station(start.vehicle, start.state_gcrf, {{t0 + 1000.0, 1.0, "Reboost"}});
    const auto j = without.add_station(start.vehicle, start.state_gcrf);
    with.advance_to(t0 + 2000.0, 100000);
    without.advance_to(t0 + 2000.0, 100000);
    const auto& a = with.body(i).state();
    const auto& b = without.body(j).state();
    // +1 m/s along the velocity raises the orbit: after 1000 s the bodies are ~kilometres apart, mostly along track
    CHECK(norm(a.velocity_gcrf_mps - b.velocity_gcrf_mps) > 0.5);
    CHECK(norm(a.position_gcrf_m - b.position_gcrf_m) > 500.0);
}

TEST_CASE("ISS propagated with the session's force model against the OEM", "[iss][station][validation]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    const auto archive = orbit::OemArchive::scan(fixture_dir, time);
    const auto& oem = *archive.latest();
    // after the SPX-35 docking (2026-10-15T15:00), no event inside the arc
    const auto t0 = time.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto start = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, time, t0, sim::IssOrbitSource::Oem);
    sim::FlightSession session(ephemeris, earth, config_dir);
    session.set_epoch(t0);
    session.set_solar_radiation_pressure(false);  // the OEM's SOLAR_RAD_AREA = 0 (see Step 3)
    const size_t index = session.add_station(start.vehicle, start.state_gcrf, start.impulses);
    for (const double hours : {6.0, 24.0}) {
        session.advance_to(t0 + hours * 3600.0, 100000);
        const auto& s = session.body(index).state();
        const auto truth = oem.state_gcrf(t0 + hours * 3600.0);
        const auto e = rtn(truth, s.position_gcrf_m - truth.position_m);
        if (std::getenv("STARSIM_VERBOSE")) {
            std::printf("ISS vs OEM after %4.1f h: R %8.1f T %9.1f N %7.1f m\n", hours, e.x, e.y, e.z);
        }
        // tolerances: set in Step 4 from the measurement (1.5 x measured, rounded up)
        CHECK(norm(e) < (hours < 12.0 ? 2000.0 : 20000.0));  // sanity bounds; tightened in Step 4
    }
}
```

`FlightSession` has neither an epoch setter without a body nor an SRP switch (checked 2026-10-19): Step 3 adds
`void set_epoch(time::Epoch epoch)` ("for a session that starts with a station only"; throws `std::logic_error`
while bodies exist) and `void set_solar_radiation_pressure(bool on) { env_.solar_radiation_pressure = on; }` (a
fidelity switch the OEM validation needs: the OEM is computed without SRP).

- [ ] **Step 2: Run to verify it fails** - `-TestFilter "[station]"`: compile error.

- [ ] **Step 3: Implement**

`station_start.cpp`:
- `parse_iss_orbit_source`.
- OEM branch: `const Oem* f = archive.for_epoch(epoch)`; none -> `throw std::runtime_error("Für dieses Datum liegen keine ISS-Bahndaten (OEM) vor; iss_orbit = \"auto\" oder \"synthetic\" wählen oder tools/data_pipeline/fetch.py iss_oem ausführen.")`.
  Copy the config; set `dry_mass_kg` = `MASS`, `fixed_drag_area_m2` = `DRAG_AREA`, `fixed_drag_coefficient` =
  `DRAG_COEFF` when present. State = `f->state_gcrf(epoch)`. Impulses: events with `delta_v_mps > 0` and
  `tig > epoch`; sign: `+1` when the event's mean height `(HA + HP) / 2` is above the OEM state's height
  `norm(r) - R_e` one revolution (5,550 s [E]) before the TIG (clamped to the window start), else `-1` (a
  lowering burn). `note = "ISS-Bahn: NASA-OEM vom " + creation_date`.
- Synthetic branch: `archive.latest()` -> `mean_elements_from_oem`; without files: mean elements from
  `reference_orbit` (`a = R_e + altitude`, `e = 0`, `i`, `raan`, perigee 0, latitude 0 at `reference_utc` via
  `time.from_utc(UtcDateTime::parse_iso(...))`). `advance_mean_elements(..., argument_of_latitude_rad)` ->
  `synthetic_station_state`. `note = "ISS-Bahn: synthetische Referenzbahn (Bahnebene aus dem OEM vom ...)"` or
  "(... aus der Konfiguration)". No impulses.
- Auto: OEM if `archive.for_epoch(epoch)`, else synthetic with note
  `"Kein OEM für dieses Datum: synthetische Referenzbahn"`.

FlightSession:
- `add_station`: require `config.stages.front().body_in_lvlh_rad` (else `std::logic_error("add_station: vehicle has no [stage.station]")`)
  and no station yet. Body state: position/velocity given, attitude/rate from the ideal law (construct with identity,
  then `set_ideal_lvlh_attitude(Quat from yaw-pitch-roll)`); use the existing yaw/pitch/roll helper in `math/quat.hpp`
  if there is one (else compose `Quat::from_axis_angle(z, yaw) * from_axis_angle(y, pitch) * from_axis_angle(x, roll)`,
  aerospace 3-2-1). Push into `bodies_` without touching `controlled_` when bodies exist; with no bodies it becomes
  index 0 and `controlled_ = 0` (nothing commands it: the autopilot stays Off). Store `station_index_`, the impulses
  sorted by epoch.
- Every path that clears `bodies_` (`place_body`, `launch_from_pad`, `place_in_orbit`; grep `bodies_.clear()`) resets
  `station_index_` and the impulses.
- Separation pushes new bodies at the end: the station index stays valid. If any code erases bodies from the middle
  (grep `bodies_.erase`), shift `station_index_` there too.
- `coast_allowance_s`: (a) the "rotation settled" check skips bodies with `ideal_attitude()`; (b)
  `allowance = min(allowance, next_impulse.epoch - epoch_)`.
- `advance_to`, at the top of the tick loop (where the nose cone epoch is handled): while the next impulse's epoch
  `<= epoch_`, add `delta_v * normalized(velocity)` to the station's velocity (`set_state`) and pop it.

- [ ] **Step 4: Run, measure and set the tolerances**

`STARSIM_VERBOSE=1` run of `-TestFilter "[station]"`; record the R/T/N errors after 6 h and 24 h in the ledger.
Expected a few km, mostly T (US76 density at ~415 km vs NASA's model; J2-only gravity vs the operator's field).
- Within the sanity bounds: set the tolerance per epoch to 1.5 x the measured norm, rounded up to the next 100 m,
  and write the measured numbers in a comment next to them.
- Beyond the sanity bounds (6 h > 2 km or 24 h > 20 km): STOP - do not loosen. Investigate with
  superpowers:systematic-debugging (frame, UTC/TDB, km units, drag sign, mass override, the docking event's mass
  change) and record the root cause in the ledger before continuing.

- [ ] **Step 5: Commit and push** - `-Fast` and the full suite (flight sessions changed:
  `tools/build.ps1 -NoBindings > tmp/i1full.txt 2>&1`, tail); `git commit -m "feat(sim): ISS as a station body in the flight session, OEM/synthetic start, OEM events as impulses"`; push.

---

### Task 6: Documentation, final review

**Files:**
- Modify: `docs/subsystems.md` (new section "ISS target body (i1)"), `CLAUDE.md` (module list: `orbit` OEM +
  synthetic station orbit; `aero` fixed-area drag; data pipeline OEM archive), `config/README.md` (ISS vehicle,
  `[stage.station]`, `[[stage.docking_port]]`, `[[stage.collision]]`, `fixed_drag_*`, `solar_area_m2`,
  `inertia_kg_m2`), `docs/superpowers/specs/2026-10-19-iss-docking-design.md` (status line: i1 implemented; fixture
  path `tests/validation/fixtures/iss_oem/`).

- [ ] **Step 1: Write the docs** - subsystems.md section with: data source and archive, OEM format facts (UTC, km,
  4 min grid with 1.5 min seams, header values), interpolation (degree 8, segment-wise) and the leave-one-out numbers,
  synthetic orbit (mean elements, J2 rates, RAAN error after 14 d), the ISS config provenance, ideal attitude
  (documented simplification), impulses, the OEM validation numbers (6 h / 24 h, R/T/N) and their causes, the
  fidelity flag `iss_orbit`, and the call order rule for `add_station`.
- [ ] **Step 2: Full suite** - `tools/build.ps1 > tmp/i1final.txt 2>&1` (with bindings), tail: all green; the Godot
  smoke test (`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd`) still passes.
- [ ] **Step 3: Final review** - superpowers:requesting-code-review over the i1 commits (most capable model); fix
  Critical/Important findings, record minors in the ledger.
- [ ] **Step 4: Commit and push** - `git commit -m "docs: ISS i1 target body"`; push. Update the memory
  (`project-iss-docking`: i1 done, next i2 plan) and ask the user for `/clear`.
