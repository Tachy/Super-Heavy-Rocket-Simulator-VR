# ISS i2: Crew Dragon 6-DOF translation and relative navigation - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Crew Dragon can translate along every axis of a target docking-port frame on its 16 Dracos while its
attitude hold keeps steering, and the session reports the ideal relative state between the Dragon's docking port
and an ISS port (position, velocity, misalignment, clocking).

**Architecture:** The existing Draco layout stays (user decision 2026-10-19: measured lateral authority ~0.046 m/s^2
with the nose cone open is enough; threshold 0.04 m/s^2 [E]); a validation test pins the authority, the zero-torque
coupling and the propellant per m/s. The Dragon gets a docking port (`[[stage.docking_port]]`, new flag
`behind_nose_cone`). `vehicle/docking_port` computes a port's GCRF kinematics from a `FlightBody`;
`gnc/relative_navigation` turns two port kinematics into the relative state in the target port frame (fidelity flag
`relative_nav = "ideal"`). `FlightSession` gets a rendezvous target, a `relative_port_state()` getter and a
translation command in the target port frame, applied with `rcs_keep_force` under the held attitude mode.

**Tech Stack:** C++20, Catch2, toml++, CSPICE only through the `spice` adapters (tests), Python 3.11 only for the
one-off LP probe already run during planning.

**Spec:** `docs/superpowers/specs/2026-10-19-iss-docking-design.md` (sections "Intent", "i2 design: Crew Dragon 6-DOF
and relative navigation", "Error handling", "Testing").

## Global Constraints

- `core/` has no Godot dependency; modules depend downwards only: `vehicle` must not include `gnc`; `gnc` may use
  `vehicle`; `sim` uses both.
- SI units internally, angles in radians; names carry frame and unit (`position_gcrf_m`, `velocity_mps`).
- No magic numbers: tuned values are named constants or config values with `[S]` / `[E]` / `[D]` tags and a reason.
- Errors that reach the mission user (unknown port id) carry German text; programming errors (`std::logic_error`,
  `std::out_of_range`) stay English like the existing ones.
- Fidelity flag `relative_nav = "ideal"`: the true states (the real Dragon uses GPS, LIDAR and thermal cameras);
  sensor noise later. Any other value is an error, never a silent fallback.
- The Dragon port sits under the nose cone: a port behind a closed cone is reported as covered.
- Only the Crew Dragon docks; the Draco layout is unchanged (user 2026-10-19, "Layout behalten").
- Commit on `main` (Conventional Commits) and push after every task once its tests are green; `-Fast` before every
  push; full suite before the final review (this plan touches vehicles and flight sessions).
- ctest runs test cases as parallel processes: no shared temp files (unique names per test case).
- Long command output to `tmp/` (git-ignored), read tails / grep. Never filter `-TestFilter "[mission]"`.
- Do not use Bash heredocs to write files: use the Write tool (memory `feedback-no-heredocs`).

## Review Focus

1. **A translation command must stop the session from coasting:** the coast integrator skips the RCS, so a
   commanded translation would silently deliver no delta-v. Test: Task 4 Step 1 (`Dragon translates along each
   target port axis under an LVLH hold`: the measured delta-v per axis).
2. **Full-fraction lateral translation uses nearly all Dracos:** the attitude hold must still keep the Dragon within
   1 deg of its LVLH attitude. Test: Task 4 Step 1 (worst attitude error per axis).
3. **A closed nose cone:** lateral authority is zero and the Dragon port is covered; with the cone open both are
   available. Tests: Task 1 Step 1 (`Draco lateral translation needs the open nose cone`), Task 2 Step 1
   (`A port behind a closed nose cone is covered`).
4. **The body list is cleared after a rendezvous target was set** (`place_in_orbit` / `place_body` /
   `launch_from_pad`): the target and the translation command are dropped, `relative_port_state()` is empty, no
   dangling index. Test: Task 4 Step 1 (`Placing a vehicle clears the rendezvous target`).
5. **The target port frame rotates with the station (LVLH rate ~1.1e-3 rad/s):** a chaser co-moving with the port
   must read ~0 relative velocity, not the omega x r of the rotating frame. Tests: Task 3 Step 1
   (`A co-rotating chaser reads zero relative velocity`), Task 4 Step 1 (initial relative state check).

---

## File structure

| File | Responsibility |
|---|---|
| `config/vehicles/crew_dragon.toml` | Dragon docking port `idss` (behind the nose cone); RCS comment updated |
| `core/include/starsim/config/vehicle_config.hpp`, `core/src/config/vehicle_config.cpp` | `DockingPortConfig::behind_nose_cone` |
| `core/include/starsim/vehicle/docking_port.hpp`, `core/src/vehicle/docking_port.cpp` | GCRF kinematics of a port of a `FlightBody` |
| `core/include/starsim/gnc/relative_navigation.hpp`, `core/src/gnc/relative_navigation.cpp` | Relative port state in the target port frame; `relative_nav` flag |
| `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`, `core/src/sim/flight_session_command.cpp` | Rendezvous target, `relative_port_state()`, translation command, coast stop |
| `core/tests/test_dragon_translation.cpp` | Authority, coupling, propellant per m/s; config port |
| `core/tests/test_docking_port.cpp` | Port kinematics |
| `core/tests/test_relative_navigation.cpp` | Relative state math |
| `core/tests/test_rendezvous_session.cpp` | Session: target, getter, translation campaign |
| `core/CMakeLists.txt`, `core/tests/CMakeLists.txt` | New sources and tests |
| `docs/subsystems.md`, `config/README.md`, `CLAUDE.md`, the spec | Documentation (Task 5) |

Planning probe (2026-10-19, LP bound, not committed): with the cone open the zero-torque authority is +-x 1534 N,
+-y / +-z 576 N on 12.5 t (0.123 / 0.046 m/s^2; 0.040-0.058 m/s^2 for CG stations 4.2-4.8 m); with the cone closed
-x and all lateral directions are 0. Lateral force needs the nose Dracos: the side pods' tangential thrusters sit
~1.6 m forward of the CG and their couple is cancelled by differential axial thrust.

---

### Task 1: Draco translation authority and the Dragon docking port

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp` (struct `DockingPortConfig`)
- Modify: `core/src/config/vehicle_config.cpp:271-282` (docking port loader)
- Modify: `config/vehicles/crew_dragon.toml` (capsule stage: new `[[stage.docking_port]]`; RCS layout comment)
- Create: `core/tests/test_dragon_translation.cpp`
- Modify: `core/tests/CMakeLists.txt` (add `test_dragon_translation.cpp` after `test_iss_station.cpp`)

**Interfaces:**
- Consumes: `vehicle::Vehicle::rcs_allocator()`, `propulsion::RcsAllocator::force_capability_n(dir, &duty)`,
  `torque_nm(duty)`, `force_n(duty)`, `max_torque_nm()`, `Vehicle::rcs_force_capability(dir)` (`force_n`,
  `mass_flow_kgps`), `Vehicle::set_nose_cone_open(bool)`, `Vehicle::mass_properties()`.
- Produces: `config::DockingPortConfig::behind_nose_cone` (bool, default false; TOML key `behind_nose_cone`);
  Dragon port id `"idss"` on stage `"capsule"`: position `[4.0, 0.0, 0.0]`, axis `[1, 0, 0]`, up `[0, 0, -1]`,
  `behind_nose_cone = true`.

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_dragon_translation.cpp`:

```cpp
// ISS i2 (docs/superpowers/specs/2026-10-19-iss-docking-design.md): Crew Dragon translation on its 16 Dracos -
// authority per axis at zero torque, coupling, propellant per m/s, and the Dragon docking port.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using math::Vec3;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
// [E] user decision 2026-10-19 ("Layout behalten"): the [E] Draco layout reaches ~0.046 m/s^2 laterally at 12.5 t
// with the cone open (LP bound); 0.04 m/s^2 is 1 m/s in 25 s - ample for cm/s approach corrections.
constexpr double min_translation_accel_mps2 = 0.04;
const Vec3 axes[] = {{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}};
}  // namespace

TEST_CASE("Draco translation authority with the nose cone open", "[iss][rendezvous][rcs][validation]") {
    vehicle::Vehicle v(config::load_vehicle(config_dir, "crew_dragon"), config_dir);
    v.set_nose_cone_open(true);
    const double mass = v.mass_properties().mass_kg;
    const auto& a = v.rcs_allocator();
    const Vec3 couple = a.max_torque_nm();
    const double smallest_couple = std::min({couple.x, couple.y, couple.z});
    const double ideal_kg_per_mps = mass / (v.stages().back().config.rcs.isp_s * constants::standard_gravity_mps2);
    for (const Vec3& u : axes) {
        CAPTURE(u.x, u.y, u.z);
        std::vector<double> duty;
        const double force = a.force_capability_n(u, &duty);
        const Vec3 f = a.force_n(duty), torque = a.torque_nm(duty);
        const auto capability = v.rcs_force_capability(u);
        const double kg_per_mps = capability.mass_flow_kgps / (capability.force_n / mass);
        std::printf("[validation] Draco translation (%+.0f,%+.0f,%+.0f): %.1f N = %.4f m/s^2, torque %.2f N m, "
                    "%.2f kg per m/s (ideal %.2f)\n",
                    u.x, u.y, u.z, force, force / mass, norm(torque), kg_per_mps, ideal_kg_per_mps);
        CHECK(force / mass >= min_translation_accel_mps2);
        CHECK(norm(torque) < 0.02 * smallest_couple);   // translation without a turn: < 2 % of the weakest couple
        CHECK(norm(f - u * force) < 0.02 * force);      // no cross-axis push
        CHECK(kg_per_mps >= ideal_kg_per_mps * 0.999);  // physics bound: m / (Isp g0)
        CHECK(kg_per_mps < 3.0 * ideal_kg_per_mps);     // [E] opposing thrusters cost, but bounded
    }
}

TEST_CASE("Draco lateral translation needs the open nose cone", "[iss][rendezvous][rcs]") {
    vehicle::Vehicle v(config::load_vehicle(config_dir, "crew_dragon"), config_dir);
    REQUIRE_FALSE(v.nose_cone_open());
    const auto& a = v.rcs_allocator();
    for (const Vec3& u : {Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, -1.0}, Vec3{-1.0, 0.0, 0.0}}) {
        CAPTURE(u.x, u.y, u.z);
        CHECK(a.force_capability_n(u) < 1.0);  // LP bound 0 N: the nose Dracos cancel the side pods' couple
    }
}

TEST_CASE("Crew Dragon has its docking port under the nose cone", "[iss][rendezvous][config]") {
    const auto cfg = config::load_vehicle(config_dir, "crew_dragon");
    const auto& capsule = cfg.stages.back();
    REQUIRE(capsule.id == "capsule");
    REQUIRE(capsule.docking_ports.size() == 1);
    const auto& port = capsule.docking_ports.front();
    CHECK(port.id == "idss");
    CHECK(port.behind_nose_cone);
    CHECK(port.axis.x == Catch::Approx(1.0));
    CHECK(port.position_m.x == Catch::Approx(4.0));
    for (const auto& p : config::load_vehicle(config_dir, "iss").stages.front().docking_ports) {
        CHECK_FALSE(p.behind_nose_cone);
    }
}
```

Add `test_dragon_translation.cpp` to the test list in `core/tests/CMakeLists.txt` right after `test_iss_station.cpp`.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[rendezvous]" > tmp/i2t1_red.txt 2>&1`
then `Select-String -Path tmp/i2t1_red.txt -Pattern "error C|FAILED|test cases"`.
Expected: compile error `'behind_nose_cone': is not a member of 'starsim::config::DockingPortConfig'`.
(The two authority test cases are measurements of existing code: if the build is made to compile without the field
they PASS - that is their purpose; their numbers go into the docs in Task 5.)

- [ ] **Step 3: Implement**

In `core/include/starsim/config/vehicle_config.hpp`, struct `DockingPortConfig`, add after `math::Vec3 up;`:

```cpp
    bool behind_nose_cone = false;  // covered while the stage's nose cone is closed (Crew Dragon)
```

In `core/src/config/vehicle_config.cpp` (docking port loader, after the perpendicularity check, before
`s.docking_ports.push_back(port);`):

```cpp
            port.behind_nose_cone = p["behind_nose_cone"].value_or(false);
```

In `config/vehicles/crew_dragon.toml`, replace the RCS layout comment's last sentence
`Lateral translation (docking) is left to M9.` by
`Lateral translation (docking, i2): ~0.046 m/s^2 at 12.5 t with the cone open - the side pods' tangential couple is`
`# cancelled by differential axial thrust of the nose and forward Dracos; zero with the cone closed (kept, user 2026-10-19).`
(two comment lines), and add at the end of the capsule stage (after the last `[[stage.parachute]]` block of the
capsule, i.e. at the end of the file):

```toml
# Docking port (milestone i): the IDSS-compatible docking system on the forward bulkhead, under the hinged nose cone.
# Stage frame (x station from the capsule bottom); clocking reference: body -z [E].
[[stage.docking_port]]
id = "idss"
name = "Dragon-Andocksystem"
position_m = [4.0, 0.0, 0.0]             # [E] top of the pressure vessel, below the cone hinge (length 4.4 m with the cone)
axis = [1.0, 0.0, 0.0]                   # forward, along the capsule axis
up = [0.0, 0.0, -1.0]                    # [E] clocking reference
behind_nose_cone = true                  # [S] the cone covers it during ascent and entry
```

(Check with `config::load_vehicle` that the block belongs to the capsule stage: TOML attaches `[[stage.docking_port]]`
to the last `[[stage]]`, which is the capsule.)

- [ ] **Step 4: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[rendezvous]" > tmp/i2t1.txt 2>&1`
then `Select-String -Path tmp/i2t1.txt -Pattern "\[validation\]|FAILED|All tests passed|test cases"`.
Expected: `All tests passed`; six `[validation] Draco translation` lines, lateral ~0.045 m/s^2, axial ~0.12 m/s^2.
Record the six lines in the ledger (Task 5 documents them). If a lateral value is below 0.04 m/s^2 or the torque
check fails, stop and report the numbers (the user decided on the layout with the LP bound 0.046 m/s^2).

Also run `python tools/vehicle_report.py crew_dragon > tmp/i2t1_report.txt` and check it still loads (config change).

- [ ] **Step 5: Commit**

```bash
git add config/vehicles/crew_dragon.toml core/include/starsim/config/vehicle_config.hpp core/src/config/vehicle_config.cpp core/tests/test_dragon_translation.cpp core/tests/CMakeLists.txt
git commit -m "feat(vehicle): Crew Dragon docking port and Draco translation authority validation"
```

Then `-Fast` and push (see Global Constraints).

---

### Task 2: Port kinematics of a flying body

**Files:**
- Create: `core/include/starsim/vehicle/docking_port.hpp`, `core/src/vehicle/docking_port.cpp`
- Modify: `core/CMakeLists.txt` (add `src/vehicle/docking_port.cpp` after `src/vehicle/flight_body.cpp`)
- Create: `core/tests/test_docking_port.cpp`; add it to `core/tests/CMakeLists.txt` after `test_dragon_translation.cpp`

**Interfaces:**
- Consumes: `config::DockingPortConfig` (`id`, `position_m`, `axis`, `up`, `behind_nose_cone` from Task 1),
  `vehicle::Stage::base_station_m`, `Vehicle::mass_properties().cg_point_m()`, `Vehicle::has_nose_cone()`,
  `Vehicle::nose_cone_open()`, `FlightBody::state()` (`RigidBodyState`: `position_gcrf_m` = the CG,
  `velocity_gcrf_mps`, `attitude` body -> GCRF, `angular_velocity_body_radps`).
- Produces:

```cpp
namespace starsim::vehicle {
struct PortKinematics {
    math::Vec3 position_gcrf_m;
    math::Vec3 velocity_gcrf_mps;            // of the port point: CG velocity + omega x lever
    math::Mat3 axes_gcrf;                    // columns: x = outward docking axis, y = up x axis, z = up (clocking)
    math::Vec3 angular_velocity_gcrf_radps;  // of the body
    bool covered = false;                    // behind a closed nose cone
};
PortKinematics port_kinematics(const FlightBody& body, std::string_view port_id);  // std::invalid_argument (German)
}
```

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_docking_port.cpp`:

```cpp
// ISS i2: GCRF kinematics of a docking port on a flying body (vehicle/docking_port).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <stdexcept>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/docking_port.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using math::Vec3;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

std::unique_ptr<vehicle::FlightBody> body_of(const std::string& id, const vehicle::RigidBodyState& state) {
    return std::make_unique<vehicle::FlightBody>(
        std::make_unique<vehicle::Vehicle>(config::load_vehicle(config_dir, id), config_dir), state);
}
}  // namespace

TEST_CASE("Port position, velocity and axes of a turning Dragon", "[iss][rendezvous][port]") {
    // a quarter turn about GCRF z maps body (x, y, z) to GCRF (-y, x, z); spinning at 0.01 rad/s about body z
    const Vec3 position{7.0e6, 1.0e3, -2.0e3}, velocity{10.0, 7600.0, 5.0}, omega_body{0.0, 0.0, 0.01};
    auto dragon = body_of("crew_dragon", {position, velocity,
                                          math::Quat::from_axis_angle({0.0, 0.0, 1.0}, 0.5 * constants::pi),
                                          omega_body});
    const auto& v = dragon->vehicle();
    const Vec3 cg = v.mass_properties().cg_point_m();
    const double capsule_base = v.stages().back().base_station_m;  // 3.7 m: the trunk below
    CHECK(capsule_base == Catch::Approx(3.7));
    const Vec3 lever_body = Vec3{capsule_base + 4.0, 0.0, 0.0} - cg;
    const Vec3 lever_gcrf{-lever_body.y, lever_body.x, lever_body.z};

    const auto k = vehicle::port_kinematics(*dragon, "idss");
    CHECK(norm(k.position_gcrf_m - (position + lever_gcrf)) < 1e-9);
    const Vec3 omega_gcrf{0.0, 0.0, 0.01};
    CHECK(norm(k.velocity_gcrf_mps - (velocity + cross(omega_gcrf, lever_gcrf))) < 1e-12);
    CHECK(norm(k.angular_velocity_gcrf_radps - omega_gcrf) < 1e-15);
    CHECK(norm(math::column(k.axes_gcrf, 0) - Vec3{0.0, 1.0, 0.0}) < 1e-12);   // body +x
    CHECK(norm(math::column(k.axes_gcrf, 2) - Vec3{0.0, 0.0, -1.0}) < 1e-12);  // up = body -z
    CHECK(norm(math::column(k.axes_gcrf, 1) - Vec3{1.0, 0.0, 0.0}) < 1e-12);   // up x axis
    CHECK(math::determinant(k.axes_gcrf) == Catch::Approx(1.0));
}

TEST_CASE("A port behind a closed nose cone is covered", "[iss][rendezvous][port]") {
    auto dragon = body_of("crew_dragon", {{7.0e6, 0.0, 0.0}, {0.0, 7600.0, 0.0}, math::Quat{}, {}});
    CHECK(vehicle::port_kinematics(*dragon, "idss").covered);
    dragon->vehicle().set_nose_cone_open(true);
    CHECK_FALSE(vehicle::port_kinematics(*dragon, "idss").covered);
    auto iss = body_of("iss", {{6.8e6, 0.0, 0.0}, {0.0, 7660.0, 0.0}, math::Quat{}, {}});
    CHECK_FALSE(vehicle::port_kinematics(*iss, "ida2").covered);
    // IDA-3 sits on the station stage (base 0) at its configured position relative to the CG
    const auto k = vehicle::port_kinematics(*iss, "ida3");
    const Vec3 expected = Vec3{66.0, 0.0, -5.0} - iss->vehicle().mass_properties().cg_point_m();
    CHECK(norm(k.position_gcrf_m - Vec3{6.8e6, 0.0, 0.0} - expected) < 1e-9);
}

TEST_CASE("An unknown port id is a German error", "[iss][rendezvous][port]") {
    auto iss = body_of("iss", {{6.8e6, 0.0, 0.0}, {0.0, 7660.0, 0.0}, math::Quat{}, {}});
    CHECK_THROWS_AS(vehicle::port_kinematics(*iss, "ida1"), std::invalid_argument);
}
```

Check `FlightBody::vehicle()` has a non-const overload (used by `test_crew_dragon.cpp`: `body->vehicle().set_nose_cone_open(true)`).

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[port]" > tmp/i2t2_red.txt 2>&1`
Expected: compile error `Cannot open include file: 'starsim/vehicle/docking_port.hpp'`.

- [ ] **Step 3: Implement**

`core/include/starsim/vehicle/docking_port.hpp`:

```cpp
#pragma once

#include <string_view>

#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::vehicle {

// Where a docking port of a flying body is and how it moves (GCRF). The port frame: x = the outward docking axis,
// z = the clocking reference "up", y = z x x (right-handed). Two ports mate with the axes opposed and the ups equal.
struct PortKinematics {
    math::Vec3 position_gcrf_m;
    math::Vec3 velocity_gcrf_mps;            // of the port point: CG velocity + omega x lever
    math::Mat3 axes_gcrf;                    // columns x, y, z of the port frame
    math::Vec3 angular_velocity_gcrf_radps;  // of the body
    bool covered = false;                    // behind a closed nose cone
};

// The port `port_id` on any stage of the body's vehicle (config [[stage.docking_port]]); the lever runs from the CG,
// so it follows the CG as propellant is used. Throws std::invalid_argument (German) for an unknown id.
PortKinematics port_kinematics(const FlightBody& body, std::string_view port_id);

}  // namespace starsim::vehicle
```

`core/src/vehicle/docking_port.cpp`:

```cpp
#include "starsim/vehicle/docking_port.hpp"

#include <stdexcept>
#include <string>

namespace starsim::vehicle {

PortKinematics port_kinematics(const FlightBody& body, std::string_view port_id) {
    const auto& vehicle = body.vehicle();
    for (const auto& stage : vehicle.stages()) {
        for (const auto& port : stage.config.docking_ports) {
            if (port.id != port_id) continue;
            const auto& s = body.state();
            const math::Vec3 lever_body =
                math::Vec3{stage.base_station_m, 0.0, 0.0} + port.position_m - vehicle.mass_properties().cg_point_m();
            const math::Vec3 lever = s.attitude.rotate(lever_body);
            const math::Vec3 omega = s.attitude.rotate(s.angular_velocity_body_radps);
            const math::Vec3 x = s.attitude.rotate(port.axis), z = s.attitude.rotate(port.up);
            PortKinematics k;
            k.position_gcrf_m = s.position_gcrf_m + lever;
            k.velocity_gcrf_mps = s.velocity_gcrf_mps + cross(omega, lever);
            k.axes_gcrf = math::Mat3::from_columns(x, cross(z, x), z);
            k.angular_velocity_gcrf_radps = omega;
            k.covered = port.behind_nose_cone && vehicle.has_nose_cone() && !vehicle.nose_cone_open();
            return k;
        }
    }
    // Vehicle keeps no vehicle id: name the stages searched
    std::string stages;
    for (const auto& stage : vehicle.stages()) stages += (stages.empty() ? "" : ", ") + stage.config.id;
    throw std::invalid_argument("Unbekannter Andockport '" + std::string(port_id) + "' (Stufen: " + stages + ")");
}

}  // namespace starsim::vehicle
```

Add `src/vehicle/docking_port.cpp` to `core/CMakeLists.txt` after `src/vehicle/flight_body.cpp`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[port]" > tmp/i2t2.txt 2>&1`
Expected: `All tests passed` (3 test cases).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/vehicle/docking_port.hpp core/src/vehicle/docking_port.cpp core/CMakeLists.txt core/tests/test_docking_port.cpp core/tests/CMakeLists.txt
git commit -m "feat(vehicle): docking port kinematics of a flying body"
```

Then `-Fast` and push.

---

### Task 3: Relative navigation between two ports

**Files:**
- Create: `core/include/starsim/gnc/relative_navigation.hpp`, `core/src/gnc/relative_navigation.cpp`
- Modify: `core/CMakeLists.txt` (add `src/gnc/relative_navigation.cpp` after `src/gnc/orbital_attitude.cpp`)
- Create: `core/tests/test_relative_navigation.cpp`; add it to `core/tests/CMakeLists.txt` after `test_docking_port.cpp`

**Interfaces:**
- Consumes: `vehicle::PortKinematics` (Task 2), `math::transpose(Mat3)`, `math::column(Mat3, k)`.
- Produces:

```cpp
namespace starsim::gnc {
enum class RelativeNavMode { Ideal };                       // fidelity flag relative_nav
RelativeNavMode parse_relative_nav(std::string_view text);  // "ideal"; std::invalid_argument otherwise
struct RelativePortState {
    math::Vec3 position_m;          // chaser port relative to the target port, target port frame
    math::Vec3 velocity_mps;        // its rate of change in the rotating target port frame
    math::Vec3 angular_rate_radps;  // chaser body rate relative to the target's, target port frame
    double range_m = 0.0;
    double closing_speed_mps = 0.0; // -d(range)/dt: > 0 approaching
    double misalignment_rad = 0.0;  // between the chaser's docking axis and the reversed target axis
    double clocking_rad = 0.0;      // chaser up against target up about the target axis, (-pi, pi]
    bool chaser_port_covered = false;
};
RelativePortState relative_port_state(const vehicle::PortKinematics& chaser, const vehicle::PortKinematics& target,
                                      RelativeNavMode mode = RelativeNavMode::Ideal);
}
```

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_relative_navigation.cpp`:

```cpp
// ISS i2: relative state of a chaser port against a target port (gnc/relative_navigation).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <stdexcept>

#include "starsim/gnc/relative_navigation.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;

math::Mat3 axes_of(const math::Quat& q) {  // port axes rotated from the identity
    return math::Mat3::from_columns(q.rotate({1.0, 0.0, 0.0}), q.rotate({0.0, 1.0, 0.0}), q.rotate({0.0, 0.0, 1.0}));
}

// Target at the origin, axes identity, turning at the ISS orbital rate about z.
vehicle::PortKinematics target() {
    vehicle::PortKinematics t;
    t.axes_gcrf = axes_of({});
    t.angular_velocity_gcrf_radps = {0.0, 0.0, 1.1e-3};
    return t;
}

// A docked-attitude chaser (axis opposed, up equal) at `position` co-moving with the target's rotation.
vehicle::PortKinematics chaser(const vehicle::PortKinematics& t, const Vec3& position) {
    vehicle::PortKinematics c;
    c.position_gcrf_m = position;
    c.velocity_gcrf_mps = cross(t.angular_velocity_gcrf_radps, position);
    c.axes_gcrf = axes_of(math::Quat::from_axis_angle({0.0, 0.0, 1.0}, constants::pi));  // x -> -x, z stays
    c.angular_velocity_gcrf_radps = t.angular_velocity_gcrf_radps;
    return c;
}
}  // namespace

TEST_CASE("A co-rotating chaser reads zero relative velocity", "[iss][rendezvous][relnav]") {
    const auto t = target();
    const auto r = gnc::relative_port_state(chaser(t, {10.0, 0.0, 0.0}), t);
    CHECK(norm(r.position_m - Vec3{10.0, 0.0, 0.0}) < 1e-12);
    CHECK(norm(r.velocity_mps) < 1e-12);  // omega x r of the rotating frame is no relative motion
    CHECK(norm(r.angular_rate_radps) < 1e-15);
    CHECK(r.range_m == Catch::Approx(10.0));
    CHECK(r.closing_speed_mps == Catch::Approx(0.0).margin(1e-12));
    CHECK(r.misalignment_rad == Catch::Approx(0.0).margin(1e-9));
    CHECK(r.clocking_rad == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("An approaching chaser closes along the port axis", "[iss][rendezvous][relnav]") {
    const auto t = target();
    auto c = chaser(t, {10.0, 0.0, 0.0});
    c.velocity_gcrf_mps += Vec3{-0.1, 0.0, 0.0};
    const auto r = gnc::relative_port_state(c, t);
    CHECK(norm(r.velocity_mps - Vec3{-0.1, 0.0, 0.0}) < 1e-12);
    CHECK(r.closing_speed_mps == Catch::Approx(0.1));
}

TEST_CASE("Relative position is expressed in the target port frame", "[iss][rendezvous][relnav]") {
    auto t = target();
    t.position_gcrf_m = {100.0, -50.0, 20.0};
    t.axes_gcrf = axes_of(math::Quat::from_axis_angle({0.0, 0.0, 1.0}, 0.5 * constants::pi));  // x_T = GCRF +y
    t.angular_velocity_gcrf_radps = {};
    auto c = chaser(t, t.position_gcrf_m + Vec3{0.0, 10.0, 0.0});
    c.axes_gcrf = axes_of(math::Quat::from_axis_angle({0.0, 0.0, 1.0}, -0.5 * constants::pi));  // x_C = -y
    const auto r = gnc::relative_port_state(c, t);
    CHECK(norm(r.position_m - Vec3{10.0, 0.0, 0.0}) < 1e-12);
    CHECK(r.misalignment_rad == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("Misalignment and clocking of the chaser port", "[iss][rendezvous][relnav]") {
    const auto t = target();
    const math::Quat docked = math::Quat::from_axis_angle({0.0, 0.0, 1.0}, constants::pi);
    auto tilted = chaser(t, {5.0, 0.0, 0.0});
    tilted.axes_gcrf = axes_of(math::Quat::from_axis_angle({0.0, 1.0, 0.0}, 3.0 * deg) * docked);
    CHECK(gnc::relative_port_state(tilted, t).misalignment_rad == Catch::Approx(3.0 * deg).epsilon(1e-9));
    auto clocked = chaser(t, {5.0, 0.0, 0.0});
    clocked.axes_gcrf = axes_of(math::Quat::from_axis_angle({1.0, 0.0, 0.0}, 10.0 * deg) * docked);
    const auto r = gnc::relative_port_state(clocked, t);
    CHECK(r.misalignment_rad == Catch::Approx(0.0).margin(1e-9));
    CHECK(r.clocking_rad == Catch::Approx(10.0 * deg).epsilon(1e-9));  // + about the target axis
}

TEST_CASE("A covered chaser port and the relative_nav flag", "[iss][rendezvous][relnav]") {
    const auto t = target();
    auto c = chaser(t, {10.0, 0.0, 0.0});
    c.covered = true;
    CHECK(gnc::relative_port_state(c, t).chaser_port_covered);
    CHECK(gnc::parse_relative_nav("ideal") == gnc::RelativeNavMode::Ideal);
    CHECK_THROWS_AS(gnc::parse_relative_nav("lidar"), std::invalid_argument);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[relnav]" > tmp/i2t3_red.txt 2>&1`
Expected: compile error `Cannot open include file: 'starsim/gnc/relative_navigation.hpp'`.

- [ ] **Step 3: Implement**

`core/include/starsim/gnc/relative_navigation.hpp`:

```cpp
#pragma once

#include <string_view>

#include "starsim/math/vec3.hpp"
#include "starsim/vehicle/docking_port.hpp"

namespace starsim::gnc {

// Fidelity flag relative_nav. Ideal: the true states of both bodies (the real Dragon navigates on GPS, LIDAR and
// thermal cameras; sensor models later).
enum class RelativeNavMode { Ideal };
RelativeNavMode parse_relative_nav(std::string_view text);  // "ideal"; std::invalid_argument otherwise

// The chaser's docking port against the target's, in the target port frame (x out of the target port, z its up). The
// frame turns with the target body: a chaser at rest relative to the target reads zero velocity. Docked: position 0,
// misalignment 0 (axes opposed), clocking 0 (ups equal).
struct RelativePortState {
    math::Vec3 position_m;          // chaser port relative to the target port
    math::Vec3 velocity_mps;        // its rate of change in the rotating target port frame
    math::Vec3 angular_rate_radps;  // chaser body rate relative to the target's
    double range_m = 0.0;
    double closing_speed_mps = 0.0;  // -d(range)/dt: > 0 approaching
    double misalignment_rad = 0.0;   // between the chaser's docking axis and the reversed target axis
    double clocking_rad = 0.0;       // chaser up against target up about the target axis, (-pi, pi]
    bool chaser_port_covered = false;
};

RelativePortState relative_port_state(const vehicle::PortKinematics& chaser, const vehicle::PortKinematics& target,
                                      RelativeNavMode mode = RelativeNavMode::Ideal);

}  // namespace starsim::gnc
```

`core/src/gnc/relative_navigation.cpp`:

```cpp
#include "starsim/gnc/relative_navigation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace starsim::gnc {

RelativeNavMode parse_relative_nav(std::string_view text) {
    if (text == "ideal") return RelativeNavMode::Ideal;
    throw std::invalid_argument("relative_nav must be \"ideal\", not \"" + std::string(text) + "\"");
}

RelativePortState relative_port_state(const vehicle::PortKinematics& chaser, const vehicle::PortKinematics& target,
                                      RelativeNavMode /*mode: only Ideal*/) {
    const math::Mat3 to_port = transpose(target.axes_gcrf);
    const math::Vec3 dr = chaser.position_gcrf_m - target.position_gcrf_m;
    // seen from the rotating target frame: remove the frame's omega x r
    const math::Vec3 dv =
        chaser.velocity_gcrf_mps - target.velocity_gcrf_mps - cross(target.angular_velocity_gcrf_radps, dr);
    RelativePortState s;
    s.position_m = to_port * dr;
    s.velocity_mps = to_port * dv;
    s.angular_rate_radps = to_port * (chaser.angular_velocity_gcrf_radps - target.angular_velocity_gcrf_radps);
    s.range_m = norm(dr);
    s.closing_speed_mps = s.range_m > 0.0 ? -dot(dr, dv) / s.range_m : 0.0;
    const math::Vec3 tx = math::column(target.axes_gcrf, 0), tz = math::column(target.axes_gcrf, 2);
    const math::Vec3 cx = math::column(chaser.axes_gcrf, 0), cz = math::column(chaser.axes_gcrf, 2);
    s.misalignment_rad = std::acos(std::clamp(-dot(cx, tx), -1.0, 1.0));
    const math::Vec3 cz_plane = cz - dot(cz, tx) * tx;  // chaser up projected onto the docking plane
    s.clocking_rad = std::atan2(dot(cross(tz, cz_plane), tx), dot(tz, cz_plane));
    s.chaser_port_covered = chaser.covered;
    return s;
}

}  // namespace starsim::gnc
```

Add `src/gnc/relative_navigation.cpp` to `core/CMakeLists.txt` after `src/gnc/orbital_attitude.cpp`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[relnav]" > tmp/i2t3.txt 2>&1`
Expected: `All tests passed` (5 test cases).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/gnc/relative_navigation.hpp core/src/gnc/relative_navigation.cpp core/CMakeLists.txt core/tests/test_relative_navigation.cpp core/tests/CMakeLists.txt
git commit -m "feat(gnc): relative navigation between two docking ports (ideal)"
```

Then `-Fast` and push.

---

### Task 4: Rendezvous target and translation command in the flight session

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp` (public API next to `add_station`; private members next to
  `station_index_`; private helper next to `apply_return_command`)
- Modify: `core/src/sim/flight_session.cpp` (setters / getter after `FlightSession::clear_station`; the three
  `bodies_.clear()` sites; `coast_allowance_s`)
- Modify: `core/src/sim/flight_session_command.cpp` (`command_for`: attitude-mode branch and RateCommand branch;
  new `apply_translation`)
- Create: `core/tests/test_rendezvous_session.cpp`; add it to `core/tests/CMakeLists.txt` after
  `test_relative_navigation.cpp`

**Interfaces:**
- Consumes: `vehicle::port_kinematics` (Task 2), `gnc::relative_port_state`, `gnc::RelativePortState` (Task 3),
  `sim::iss_start`, `StationStart::epoch`, `FlightSession::add_station(config, state, epoch, impulses)` (i1),
  `gnc::lvlh_hold()`, `frames::lvlh_to_inertial(state)`.
- Produces (public, `sim::FlightSession`):

```cpp
    void set_rendezvous_target(size_t target_index, const std::string& chaser_port, const std::string& target_port);
    void clear_rendezvous_target();
    std::optional<gnc::RelativePortState> relative_port_state() const;
    void set_translation_command(const math::Vec3& fraction_port);
    math::Vec3 translation_command() const { return translation_port_; }
```

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_rendezvous_session.cpp`:

```cpp
// ISS i2: the flight session's rendezvous target, relative navigation and translation command in the target port frame.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/lvlh.hpp"
#include "starsim/gnc/orbital_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/oem.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/station_start.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/docking_port.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
const std::filesystem::path fixture_dir = std::filesystem::path(STARSIM_FIXTURE_DIR) / "iss_oem";
constexpr double deg = constants::deg_to_rad;

// A Dragon (cone open, LVLH hold) `range_m` in front of the ISS port IDA-2, co-moving with it; the ISS as body 1.
struct DragonAtIss {
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;  // not "time": that would hide the namespace for time::Epoch below
    sim::FlightSession session{ephemeris, earth, config_dir};
    time::Epoch t0;
    explicit DragonAtIss(double range_m) {
        const auto archive = orbit::OemArchive::scan(fixture_dir, clock);
        t0 = clock.from_utc({2026, 10, 16, 0, 0, 0.0});
        const auto iss = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, clock, earth, t0,
                                        sim::IssOrbitSource::Oem);
        sim::FlightSession probe(ephemeris, earth, config_dir);  // reads the port of the ideal-attitude station
        probe.add_station(iss.vehicle, iss.state_gcrf, iss.epoch);
        const auto port = vehicle::port_kinematics(probe.body(0), "ida2");
        const Vec3 position = port.position_gcrf_m + port.axes_gcrf * Vec3{range_m, 0.0, 0.0};
        const Vec3 velocity = port.velocity_gcrf_mps + cross(port.angular_velocity_gcrf_radps, position - port.position_gcrf_m);
        session.place_in_orbit(config::load_vehicle(config_dir, "crew_dragon"), {position, velocity}, t0);
        session.set_nose_cone(0, true);
        session.add_station(iss.vehicle, iss.state_gcrf, iss.epoch, iss.impulses);
        session.set_attitude_mode(gnc::lvlh_hold());
    }
};

double lvlh_attitude_error_rad(const vehicle::FlightBody& b) {
    const auto& s = b.state();
    const Vec3 lvlh_x = frames::lvlh_to_inertial({s.position_gcrf_m, s.velocity_gcrf_mps}).rotate({1.0, 0.0, 0.0});
    return std::acos(std::clamp(dot(s.attitude.rotate({1.0, 0.0, 0.0}), lvlh_x), -1.0, 1.0));
}
}  // namespace

TEST_CASE("Rendezvous target: relative state of the Dragon port against IDA-2", "[iss][rendezvous][session]") {
    test::spice_session();
    DragonAtIss d(50.0);
    auto& s = d.session;
    CHECK_FALSE(s.relative_port_state());
    CHECK_THROWS_AS(s.set_translation_command({0.0, 1.0, 0.0}), std::logic_error);  // no target yet
    CHECK_THROWS_AS(s.set_rendezvous_target(1, "idss", "ida1"), std::invalid_argument);  // unknown port
    CHECK_THROWS_AS(s.set_rendezvous_target(0, "idss", "idss"), std::out_of_range);     // the chaser itself
    CHECK_THROWS_AS(s.set_rendezvous_target(5, "idss", "ida2"), std::out_of_range);
    s.set_rendezvous_target(1, "idss", "ida2");
    const auto r = s.relative_port_state();
    REQUIRE(r);
    // the Dragon's CG was placed 50 m out; its port sits on its nose (body +x along the LVLH velocity, like IDA-2)
    CHECK(r->range_m == Catch::Approx(50.0).margin(6.0));
    CHECK(norm(r->velocity_mps) < 0.01);  // co-moving: no omega x r phantom from the rotating port frame
    CHECK_FALSE(r->chaser_port_covered);
}

TEST_CASE("Placing a vehicle clears the rendezvous target", "[iss][rendezvous][session]") {
    test::spice_session();
    DragonAtIss d(50.0);
    auto& s = d.session;
    s.set_rendezvous_target(1, "idss", "ida2");
    s.set_translation_command({0.0, 0.5, 0.0});
    const auto dragon_state = s.body(0).state();
    s.place_in_orbit(config::load_vehicle(config_dir, "crew_dragon"),
                     {dragon_state.position_gcrf_m, dragon_state.velocity_gcrf_mps}, s.epoch());
    CHECK_FALSE(s.relative_port_state());
    CHECK(norm(s.translation_command()) == 0.0);
    CHECK_THROWS_AS(s.set_translation_command({0.0, 1.0, 0.0}), std::logic_error);
}

TEST_CASE("Dragon translates along each target port axis under an LVLH hold", "[iss][rendezvous][session][validation]") {
    test::spice_session();
    DragonAtIss d(50.0);
    auto& s = d.session;
    s.set_rendezvous_target(1, "idss", "ida2");
    constexpr double burn_s = 20.0, settle_s = 5.0;
    double t = settle_s;
    s.advance_to(d.t0 + t, 100000);
    for (const Vec3 u : {Vec3{1.0, 0.0, 0.0}, Vec3{-1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, -1.0, 0.0},
                         Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, -1.0}}) {
        CAPTURE(u.x, u.y, u.z);
        const auto& dragon = s.body(0);
        const auto before = *s.relative_port_state();
        const auto target = vehicle::port_kinematics(s.body(1), "ida2");
        const auto capability = dragon.vehicle().rcs_force_capability(
            dragon.state().attitude.inverse_rotate(target.axes_gcrf * u));
        const double mass = dragon.vehicle().mass_properties().mass_kg;
        const double kg0 = dragon.vehicle().rcs_propellant_kg();
        s.set_translation_command(u);
        double worst_attitude = 0.0;
        for (double k = 1.0; k <= burn_s; k += 1.0) {
            s.advance_to(d.t0 + t + k, 100000);
            worst_attitude = std::max(worst_attitude, lvlh_attitude_error_rad(s.body(0)));
        }
        s.set_translation_command({});
        t += burn_s;
        const auto after = *s.relative_port_state();
        const Vec3 dv = after.velocity_mps - before.velocity_mps;
        const double expected_mps = capability.force_n / mass * burn_s;
        const double used_kg = kg0 - s.body(0).vehicle().rcs_propellant_kg();
        std::printf("[validation] port-frame translation (%+.0f,%+.0f,%+.0f) for %.0f s: dv %.3f m/s (expected %.3f), "
                    "cross %.3f m/s, %.2f kg (expected %.2f), worst attitude %.2f deg\n",
                    u.x, u.y, u.z, burn_s, dot(dv, u), expected_mps, norm(dv - dot(dv, u) * u), used_kg,
                    capability.mass_flow_kgps * burn_s, worst_attitude / deg);
        CHECK(dot(dv, u) == Catch::Approx(expected_mps).epsilon(0.15));  // [E] pulses, CW drift over 20 s
        CHECK(norm(dv - dot(dv, u) * u) < 0.1 * expected_mps + 0.01);
        CHECK(used_kg == Catch::Approx(capability.mass_flow_kgps * burn_s).epsilon(0.25));  // + the hold's pulses
        CHECK(worst_attitude < 1.0 * deg);  // the hold keeps steering while the Dracos translate
        t += settle_s;
        s.advance_to(d.t0 + t, 100000);
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[session]" > tmp/i2t4_red.txt 2>&1`
Expected: compile error `'set_rendezvous_target': is not a member of 'starsim::sim::FlightSession'`.

- [ ] **Step 3: Implement the API**

In `core/include/starsim/sim/flight_session.hpp`, add the includes `#include "starsim/gnc/relative_navigation.hpp"`
(next to the other `gnc` includes) and, after `void set_epoch(time::Epoch epoch);`:

```cpp
    // Rendezvous (milestone i): relative navigation of the controlled body's port `chaser_port` against port
    // `target_port` of body `target_index` (fidelity flag relative_nav = "ideal": the true states). Throws
    // std::out_of_range for a bad index or the controlled body itself, std::invalid_argument (German) for an unknown
    // port. Cleared with the bodies (place_body / launch_from_pad / place_in_orbit), like the station.
    void set_rendezvous_target(size_t target_index, const std::string& chaser_port, const std::string& target_port);
    void clear_rendezvous_target();  // also stops the translation command
    std::optional<gnc::RelativePortState> relative_port_state() const;  // nullopt without a target
    // RCS translation of the controlled body in the target port frame: force fractions of the capability along that
    // direction (|f| > 1 is scaled to 1), held until changed; zero stops it. Applied with rcs_keep_force next to the
    // attitude hold's torque, so the hold keeps steering and its turns do not add to the translation; the session does
    // not coast while it is set. Throws std::logic_error without a rendezvous target.
    void set_translation_command(const math::Vec3& fraction_port);
    math::Vec3 translation_command() const { return translation_port_; }
```

Private members, after `void apply_station_impulses();`:

```cpp
    struct RendezvousTarget {
        size_t target_index = 0;
        std::string chaser_port;
        std::string target_port;
    };
    std::optional<RendezvousTarget> rendezvous_;  // set_rendezvous_target
    math::Vec3 translation_port_;                 // set_translation_command, target port frame
```

Private helper, after the `apply_return_command` declaration:

```cpp
    // The translation command as body-axis force fractions of the controlled body (nothing without a command).
    void apply_translation(const Flying& flying, vehicle::VehicleCommand& command) const;
```

In `core/src/sim/flight_session.cpp` add `#include "starsim/vehicle/docking_port.hpp"`; after
`FlightSession::clear_station()`:

```cpp
void FlightSession::set_rendezvous_target(size_t target_index, const std::string& chaser_port,
                                          const std::string& target_port) {
    if (target_index >= bodies_.size() || target_index == controlled_) {
        throw std::out_of_range("set_rendezvous_target: no such target body");
    }
    vehicle::port_kinematics(*bodies_[controlled_].body, chaser_port);  // validates both ports (German error)
    vehicle::port_kinematics(*bodies_[target_index].body, target_port);
    rendezvous_ = RendezvousTarget{target_index, chaser_port, target_port};
}

void FlightSession::clear_rendezvous_target() {
    rendezvous_.reset();
    translation_port_ = {};
}

std::optional<gnc::RelativePortState> FlightSession::relative_port_state() const {
    if (!rendezvous_ || controlled_ >= bodies_.size() || rendezvous_->target_index >= bodies_.size()) {
        return std::nullopt;
    }
    return gnc::relative_port_state(vehicle::port_kinematics(*bodies_[controlled_].body, rendezvous_->chaser_port),
                                    vehicle::port_kinematics(*bodies_[rendezvous_->target_index].body,
                                                             rendezvous_->target_port));
}

void FlightSession::set_translation_command(const math::Vec3& fraction_port) {
    if (!rendezvous_) throw std::logic_error("set_translation_command: no rendezvous target");
    const double magnitude = norm(fraction_port);
    translation_port_ = magnitude > 1.0 ? fraction_port / magnitude : fraction_port;
}
```

At each of the three `bodies_.clear()` sites (lines ~116, ~392, ~474, each already followed by or next to
`clear_station();`) add `clear_rendezvous_target();` right after `clear_station();`.

In `coast_allowance_s()`, extend the pilot-input early return:

```cpp
    if (input_.pitch != 0.0 || input_.yaw != 0.0 || input_.roll != 0.0 || norm(input_.translation) > 0.0 ||
        norm(translation_port_) > 0.0) {
        return 0.0;  // the coast skips the RCS: a commanded translation is flown tick by tick
    }
```

In `core/src/sim/flight_session_command.cpp` add `#include "starsim/vehicle/docking_port.hpp"`, and the helper at the
end of the namespace:

```cpp
void FlightSession::apply_translation(const Flying& flying, vehicle::VehicleCommand& command) const {
    if (!rendezvous_ || norm(translation_port_) <= 0.0) return;
    const auto target = vehicle::port_kinematics(*bodies_.at(rendezvous_->target_index).body, rendezvous_->target_port);
    command.rcs_force_fraction = flying.body->state().attitude.inverse_rotate(target.axes_gcrf * translation_port_);
    command.rcs_keep_force = true;  // the hold's turns must not add to the translation
}
```

In `command_for`, attitude-mode branch: replace

```cpp
        command.rcs_keep_force = true;  // an orbit: turns must not push the vehicle
        return command;
```

by

```cpp
        command.rcs_keep_force = true;  // an orbit: turns must not push the vehicle
        if (index == controlled_) apply_translation(flying, command);
        return command;
```

and in the RateCommand branch replace
`if (norm(input_.translation) > 0.0) command.rcs_force_fraction = input_.translation;  // RCS translation` by

```cpp
        if (norm(input_.translation) > 0.0) {
            command.rcs_force_fraction = input_.translation;  // RCS translation (the pilot's, body axes)
        } else if (index == controlled_) {
            apply_translation(flying, command);
        }
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[rendezvous]" > tmp/i2t4.txt 2>&1`
then `Select-String -Path tmp/i2t4.txt -Pattern "\[validation\]|FAILED|All tests passed|test cases"`.
Expected: `All tests passed`; six `[validation] port-frame translation` lines with dv ~0.9 m/s along the Dragon's
body x direction and ~0.35 m/s laterally (port +-x is the Dragon's body +-x here: both noses point along the LVLH
velocity), worst attitude < 1 deg. Record the six lines in the ledger. If the attitude bound fails on the lateral
axes (the Dracos saturate), report the numbers before changing anything - do not loosen the 1 deg bound silently.
If the range check fails, print `r->range_m` and compare with the Dragon's port lever (port station - CG).

Timing: read the test's duration from the ctest output; if over ~40 s tag it `[slow]` (CLAUDE.md).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/src/sim/flight_session_command.cpp core/tests/test_rendezvous_session.cpp core/tests/CMakeLists.txt
git commit -m "feat(sim): rendezvous target, relative port state and port-frame translation command"
```

Then `-Fast` and push.

---

### Task 5: Documentation, final review

**Files:**
- Modify: `docs/subsystems.md` (new subsection "Crew Dragon translation and relative navigation (i2)" under
  "ISS rendezvous and docking (milestone i)"), `config/README.md` ("Space stations" section: `behind_nose_cone`),
  `CLAUDE.md` (module list: `vehicle` docking-port kinematics, `gnc` relative navigation; `sim` rendezvous target),
  `docs/superpowers/specs/2026-10-19-iss-docking-design.md` (status line: i2 implemented, layout kept 2026-10-19).

- [ ] **Step 1: Write the docs** - subsystems.md subsection with: the layout decision (user 2026-10-19: kept; LP bound
  and the allocator numbers from Task 1 per axis, kg per m/s against the ideal m/(Isp g0); zero lateral with the cone
  closed and why - the side pods' couple), the Dragon port (`idss`, position [E], up [E], covered while the cone is
  closed), the port frame convention (x out, z up, y = z x x; mated: axes opposed, ups equal), the relative state
  definitions (rotating target frame, closing speed, misalignment, clocking sign), the fidelity flag `relative_nav`,
  the session API (`set_rendezvous_target`, `relative_port_state`, `set_translation_command`; cleared with the bodies;
  no coasting while translating; `rcs_keep_force`), and the Task 4 campaign numbers per axis.
- [ ] **Step 2: Full suite** - `powershell -ExecutionPolicy Bypass -File tools/build.ps1 > tmp/i2final.txt 2>&1`
  (with bindings), tail: all green; the Godot smoke test
  (`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd`)
  still passes.
- [ ] **Step 3: Final review** - superpowers:requesting-code-review over the i2 commits (most capable model); fix
  Critical/Important findings, record minors in the ledger.
- [ ] **Step 4: Commit and push** - `git commit -m "docs: ISS i2 Dragon translation and relative navigation"`; push.
  Update the memory (`project-iss-docking`: i2 done, next i3 plan) and ask the user for `/clear`.
