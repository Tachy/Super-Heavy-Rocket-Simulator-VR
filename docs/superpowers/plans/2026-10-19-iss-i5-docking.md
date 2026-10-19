# ISS i5: Docking mechanism, undocking and departure - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Crew Dragon that i4 brings to "Kontakt" at an ISS port really docks: its soft-capture ring meets the
passive IDA ring through the guide petals (penalty contact), the capture latches engage inside the IDSS envelope, a
damped joint attenuates the relative motion, the ring retracts and the 12 hooks close; from then on Dragon and ISS fly
as one compound. Outside the envelope the ring bounces or overloads, the latches do not engage and the Dragon retreats;
hull contact with the station is a "Kollision". Undocking opens the hooks, the separation springs push the Dragon off,
a departure profile leaves the keep-out sphere and the approach ellipsoid on a passively safe path, and the existing
`plan_capsule_return` brings the capsule home.

**Architecture:** Three `contact` building blocks without any session dependency, one `vehicle` and one `gnc` step,
then the session. `contact/docking_interface` computes the pre-capture contact wrench (ring faces, petal cone) in the
passive port frame and the latch condition; `contact/docking_mechanism` is the IDSS state machine (contact, soft
capture with a damped 6-DOF joint, retraction, hooks, docked, hooks opening, separated) with its loads;
`contact/station_collision` samples the chaser hull against the station's collision primitives. `vehicle` gets a
docked hold of a body (not integrated, follows its host) and the compound mass. `gnc/departure_guidance` flies the
departure. `sim/docking_sim` applies the wrenches to both bodies every tick (pattern of `CatchTowerSim`);
`flight_session_docking.cpp` wires it to the proximity flight, merges at hard capture, undocks and departs.

**Tech Stack:** C++20, Catch2, CSPICE only through the `spice` adapters (tests).

**Spec:** `docs/superpowers/specs/2026-10-19-iss-docking-design.md` (sections "Intent", "i4 design" with its
corrections, "i5 design: docking mechanism, undocking and departure", "Error handling", "Testing").

## Research done while planning (2026-10-19) and the spec corrections it implies

**IDSS Interface Definition Document, Revision E (October 2016)**, internationaldockingstandard.com (downloaded and
read for this plan; cite as [S: IDSS IDD E, <section>]):
- **Table 3.3.1.1-2, Initial Contact Conditions** (recommended capture envelope of the passive interface): closing
  (axial) rate **0.05 to 0.10 m/s**; lateral (radial) rate **0.04 m/s**; pitch/yaw rate **0.20 deg/s (vector sum)**;
  roll rate **0.20 deg/s**; lateral (radial) misalignment **0.10 m**; pitch/yaw misalignment **4.0 deg (vector sum)**;
  roll misalignment **4.0 deg**. Note 4: the lateral misalignment is the distance between the centre of the active
  soft-capture ring and the axis of the passive ring at the first contact between the guide petals. Note 1: the
  lateral rate at the vehicle CG from the lateral rate plus pitch/yaw rate should not exceed the lateral limit.
  Note 3: post-contact thrust may be used.
  -> The i4 guide values are confirmed and become [S]; **roll misalignment and roll rate were missing** in
  `CaptureEnvelope` (Task 1 adds them).
- §3.2: the **SCS mating plane** is normal to the ring axis where it cuts the conic outline of the guide petals at a
  **diameter of 1200 mm**; it is the top surface of the capture ring (active and passive).
- §3.2.2.1: **three inward-pointing guide petals**, equally spaced on the soft-capture ring.
- §3.1.1.1: sequence - petals make the first contact, the SCS corrects lateral and angular misalignment, soft capture
  is complete when **both rings are in full contact and the active latches are engaged with the strikers**; the SCS
  stabilises the vehicles, then **retracts** into hard-capture range; fine alignment by retraction plus **guide pins**;
  the **12 active/passive hook pairs** (§3.2.3.4) make the structural connection.
- §3.2.2.2: the passive ring is retracted and locked; the active ring is extended above the HCS mating plane.
- Table 3.3.1.2-1: design vehicle masses, among them **IDSS-350T** (3.5e5 kg; Ixx 1.15e8, Iyy 6.2e7, Izz 1.65e8
  kg m^2) and IDSS-10T/-15T (Dragon class: 10-15 t, Ixx 17,000-41,000, Iyy/Izz 42,000-71,000 kg m^2).
- Table 3.3.1.4-1, SCS maximum interface loads (design limit loads at the SCS mating plane centre): tension
  **3,900 N**, compression static **3,500 N**, compression dynamic (up to 0.1 s) **6,500 N**, shear **3,200 N**,
  bending **2,800 N m**, torsion **1,500 N m**.
- §3.2.3.6: separation system, total force < 2,670 N mated; **total energy 39.2-47.5 N m** (fully mated).
- App. F.2.2: the force to actuate the extended SCS latches and sensors <= 100 N (berthing case).
- The petal height, the ring stroke and all stiffnesses are not in the IDD (figures only / "TBD"): [E].

Timelines:
- Demo-2 (NASA Commercial Crew blog 2020-05-31; Wikipedia "Crew Dragon Demo-2"): soft capture 10:16 EDT, "12 hooks
  were closed to complete a hard capture at 10:27" -> **soft to hard capture 11 min** [S].
- Departure (NASASpaceflight Crew-4 return 2022-10; Demo-1 end of mission 2019-03): "undock burn 1, a 1.5 s pulse ...
  **0.15 m/s**"; "departure burn zero" begins seconds after undocking (~16 s); **Demo-1 left the keep-out sphere 6 min
  48 s and the approach ellipsoid 17 min after undocking**; later departure burns 1-3 over the following hour.

Spec corrections (to be confirmed by the user with this plan; Task 8 writes them into the spec):
1. **Envelope [S]** from IDSS IDD E Table 3.3.1.1-2, **plus roll misalignment 4.0 deg and roll rate 0.2 deg/s**;
   pitch/yaw rate and angle as vector sums (the i4 check's "angle" and "rate" already are).
2. **Petals as a reduced-order conic guide**: the three petal pairs act as one axisymmetric cone (IDSS outline
   1200 mm at the SCS mating plane, petal height and lean [E]) between the ring centres; the angular gaps between the
   three petals and roll guidance by the petal flanks are not resolved. The active ring conforms in tilt (its
   six-actuator platform) within a tilt range [E] instead of a modelled Stewart platform (spec: "not a Stewart-platform
   controller").
3. **Too fast = overload, not a separate latch rule**: the latches engage on full ring contact inside the latch
   tolerances; a too fast contact runs the joint out of its attenuation stroke [E] into the end stop, the load exceeds
   the IDSS dynamic compression limit 6,500 N, the mechanism releases the latches ("Fangen fehlgeschlagen:
   Überlast") and the Dragon rebounds. A contact slower than 0.05 m/s still captures in the model (the 100 N latch
   force is reached at any closing rate above ~3 mm/s with the ring stiffness [E]); documented.
4. **Compound body = the Dragon held on the station** (FlightBody docked hold): the station body carries the summed
   mass (its stage's `extra_mass_kg`), the momentum-weighted velocity at the merge, and its ideal attitude; the Dragon
   is no longer integrated and follows the station rigidly. The compound inertia (parallel-axis sum) is computed and
   reported but unused while the attitude is ideal. The Dragon's drag area is not added to the station's (~0.6 % of
   its C_D A) - documented. No merged `Vehicle` object.
5. **Separation speed 0.085 m/s [D]** from the IDSS separation energy (39.2-47.5 N m, middle 43.35 N m [E]) over the
   reduced mass (spec: ~0.1 m/s [E]); then the **undock burn to 0.15 m/s** [S: Crew-4 / Demo-1]; then **departure
   burn 0** = the smallest i4 breakout magnitude whose CW arc leaves the keep-out sphere, is passively safe for 24 h
   **and leaves the approach ellipsoid within 3600 s** [E; Demo-1 17 min]; the later departure burns 1-3 are not
   modelled (the deorbit planned by `plan_capsule_return` from the drift orbit replaces them).
6. **Docking timeline 11 min [S: Demo-2]** from attenuation (>= 60 s and settled) + retraction (0.30 m at 1 mm/s,
   300 s) + hooks closing 180 s [E]; undocking: hooks opening 180 s [E].
7. **A failed capture retreats to Waypoint 2 and holds there** (the crew/controller decides: `proximity_resume`
   tries again); a hull contact ends the flight with "Kollision" and the contact impulse (mission failed, the Dragon is
   not destroyed - no structural model).

## Global Constraints

- `core/` has no Godot dependency; modules depend downwards only: the new `contact` files use only `config` and
  `math` (no `vehicle`, `gnc`); the new `vehicle` files include no `contact`; `gnc` may use `orbit`, `vehicle`,
  `frames`, `math`, `time`, not `mission` / `sim`; `sim` may use all of them.
- SI units internally, angles in radians; names carry frame and unit (`position_port_m`, `force_port_n`).
- No magic numbers: every tuned value is a field of a parameters struct (`DockingInterfaceParameters`,
  `DockingMechanismParameters`, `StationCollisionParameters`, `DepartureParameters`) with an `[S]` / `[E]` / `[D]`
  tag and its source in the comment (IDSS values cite "IDSS IDD E" and the table/section).
- Passive port frame = the target port frame of `vehicle/docking_port.hpp` (x out of the port, z up, y = z x x); the
  chaser approaches from x > 0. Mated: chaser port at the origin, chaser axes x_c = -x_t, y_c = -y_t, z_c = z_t.
- LVLH convention of `frames/lvlh.hpp`: x V-bar, y H-bar, z R-bar (nadir).
- UI / status strings German (UTF-8 sources), code and comments English.
- Physics never in GDScript; no bindings in this part (i6 builds the game interface).
- Tests tagged `[iss]`, `[docking]` (plus `[proximity]` where the i4 flight is involved); `[slow]` above ~40 s;
  ctest runs test cases in parallel (no shared temp files).
- During a task only the touched tags (`-TestFilter "[docking]"`); before every push `-Fast`; Tasks 5-7 touch flight
  sessions and guidance: the full suite before their push and before the final review.
- Commit on `main`, Conventional Commits, push after every task (`git push origin main`), attribution lines from the
  session. Ledger `.superpowers/sdd/2026-10-19-iss-i5-docking/progress.md` (done, rulings, numbers, next step).
- **Run the whole plan through** (user 2026-10-19): no `/clear` request between tasks; only after Task 8.

## Review Focus

1. **The station port moves**: the ISS turns at the orbital rate (1.1e-3 rad/s) in its ideal attitude; a Dragon at
   rest in the port frame must read zero relative velocity and zero contact force. Expect: the contact uses the
   rotating port frame (relative velocities from `gnc`-equivalent port kinematics), Task 5 test "resting in the ring".
2. **Time warp while docked**: `advance_to(+6 h)` with the Dragon docked must coast (the station alone integrated) and
   keep the Dragon on the port within millimetres. Expect: coast allowed when docked, Dragon pose synced after the
   coast; Task 5 test "docked coast".
3. **Commands while docked or mid-capture**: `start_proximity`, `plan_rendezvous`, `ignite`, a translation, `undock`
   during soft capture, a second `undock`, `proximity_abort` during retraction. Expect German refusals, nothing
   changes; Task 5 / Task 6 refusal tests.
4. **Clearing paths**: `place_body` / `launch_from_pad` / `place_in_orbit` / staging / trunk jettison while docked or
   capturing must not leave a dangling docked hold, an extra station mass or a docking sim. Expect: everything reset
   with the bodies; Task 5 test "clearing while docked".
5. **Failed capture then resume**: a bounce at 0.25 m offset -> retreat to Waypoint 2 -> `proximity_resume` -> a
   second approach that captures. Expect: the docking sim re-arms for the second contact; Task 7 campaign.

---

## File map

| File | Responsibility |
|---|---|
| `core/include/starsim/contact/docking_interface.hpp`, `core/src/contact/docking_interface.cpp` | IDSS geometry + pre-capture contact wrench (ring faces, petal cone) and the latch condition, in the passive port frame |
| `core/include/starsim/contact/docking_mechanism.hpp`, `core/src/contact/docking_mechanism.cpp` | IDSS state machine: contact, soft capture joint, retraction, hooks, docked, undocking; loads and failure reasons |
| `core/include/starsim/contact/station_collision.hpp`, `core/src/contact/station_collision.cpp` | chaser hull sample points, penetration against station capsules / boxes, penalty wrench, impulse |
| `core/include/starsim/contact/primitives.hpp`, `core/src/contact/primitives.cpp` | + `point_in_capsule` |
| `core/include/starsim/vehicle/compound_mass.hpp`, `core/src/vehicle/compound_mass.cpp` | summed mass, CG and inertia tensor of two bodies |
| `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp` | + docked hold (`hold_docked`, `release_docked`, `docked()`) |
| `core/include/starsim/gnc/proximity_geometry.hpp`, `.cpp` | envelope [S] + roll terms; `contact_plane_m` comment |
| `core/include/starsim/gnc/proximity_guidance.hpp`, `.cpp` | + `capture_failed(reason)` -> retreat to Waypoint 2 |
| `core/include/starsim/gnc/proximity_transfer.hpp`, `.cpp` | breakout with an optional approach-ellipsoid exit requirement |
| `core/include/starsim/gnc/departure_guidance.hpp`, `core/src/gnc/departure_guidance.cpp` | departure state machine (undock burn, back-out, departure burn 0, drift to the AE exit) |
| `core/include/starsim/sim/docking_sim.hpp`, `core/src/sim/docking_sim.cpp` | per-tick wrench application to chaser and station, collision record |
| `core/src/sim/flight_session_docking.cpp`, `core/include/starsim/sim/flight_session.hpp` | session glue: arm at start_proximity, merge, docked sync, undock, departure, getters, refusals |
| `config/vehicles/crew_dragon.toml`, `config/vehicles/iss.toml`, `core/.../config/vehicle_config.*` | `soft_capture` mode per port, hull taper of the capsule, ISS inertia re-derived |
| `core/tests/test_docking_interface.cpp`, `test_docking_mechanism.cpp`, `test_station_collision.cpp`, `test_compound_mass.cpp`, `test_departure_guidance.cpp`, `test_docking_session.cpp`, `docking_test_support.hpp` | tests |

---

### Task 1: IDSS envelope and the pre-capture docking interface

**Files:**
- Create: `core/include/starsim/contact/docking_interface.hpp`, `core/src/contact/docking_interface.cpp`
- Modify: `core/include/starsim/gnc/proximity_geometry.hpp:102-113`, `core/src/gnc/proximity_geometry.cpp`
  (`capture_envelope_check`), `core/include/starsim/config/vehicle_config.hpp:123-130`,
  `core/src/config/vehicle_config.cpp:271-283`, `config/vehicles/crew_dragon.toml:134-140`, `config/vehicles/iss.toml:47-59`,
  `config/README.md`, `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`
- Test: `core/tests/test_docking_interface.cpp`, `core/tests/test_proximity_geometry.cpp`

**Interfaces:**
- Consumes: `gnc::RelativePortState` (i2), `config::DockingPortConfig`.
- Produces:
  - `config::DockingPortConfig::soft_capture` (`config::SoftCaptureMode { Passive, Active }`, TOML `soft_capture =
    "active"`, default passive).
  - `contact::DockingPose { Vec3 position_m; Vec3 velocity_mps; Mat3 chaser_axes; Vec3 angular_rate_radps; }` - the
    chaser port (its HCS mating plane centre) in the passive port frame; `chaser_axes` columns = chaser port axes in
    that frame; velocities in the rotating passive frame.
  - `contact::DockingInterfaceParameters` (fields below), `contact::InterfaceContact docking_interface_contact(const
    DockingPose&, double ring_extension_m, const DockingInterfaceParameters&)`.
  - `gnc::CaptureEnvelope` gains `roll_max_rad`, `roll_rate_max_radps`; `capture_envelope_check` checks them.

- [ ] **Step 1: Write the failing tests**

`core/tests/test_docking_interface.cpp`:

```cpp
// ISS i5: pre-capture contact of the IDSS soft-capture interface (ring faces, petal cone, latch condition).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

#include "starsim/contact/docking_interface.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Mat3;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
// Chaser port axes mated with the passive port: x_c = -x_t, y_c = -y_t, z_c = z_t.
const Mat3 mated_axes = Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1});

contact::DockingPose pose(Vec3 position, Vec3 velocity = {}, Mat3 axes = mated_axes) {
    return {position, velocity, axes, {}};
}
}  // namespace

TEST_CASE("Docking interface: no contact while the petals do not overlap", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    const double e = p.ring_extension_m;
    const auto c = contact::docking_interface_contact(pose({e + 2.0 * p.petal_height_m + 0.01, 0.05, 0.0}), e, p);
    CHECK_FALSE(c.any_contact);
    CHECK(norm(c.force_port_n) == 0.0);
}

TEST_CASE("Docking interface: aligned ring faces touch at the ring extension", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    const double e = p.ring_extension_m;
    // 1 cm into the passive ring face, closing at 0.075 m/s
    const auto c = contact::docking_interface_contact(pose({e - 0.01, 0, 0}, {-0.075, 0, 0}), e, p);
    CHECK(c.ring_contact);
    CHECK_FALSE(c.petal_contact);
    // pushes the chaser out of the port: k d + c v
    CHECK(c.force_port_n.x == Catch::Approx(p.ring_stiffness_n_per_m * 0.01 + p.ring_damping_ns_per_m * 0.075));
    CHECK(std::abs(c.force_port_n.y) < 1e-9);
    CHECK(c.latch_ready);  // full ring contact, centred, aligned
}

TEST_CASE("Docking interface: the penalty contact never pulls", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    const double e = p.ring_extension_m;
    const auto c = contact::docking_interface_contact(pose({e - 0.001, 0, 0}, {0.5, 0, 0}), e, p);  // separating fast
    CHECK(c.force_port_n.x >= 0.0);
}

TEST_CASE("Docking interface: the petal cone centres a lateral offset inside the capture radius", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    const double e = p.ring_extension_m;
    // halfway into the petal overlap with 0.20 m offset along +y (clearance there 0.01 + 0.20 tan 30 deg = 0.125 m,
    // capture radius 0.241 m): the cone pushes towards -y and out (+x)
    const double gap = p.petal_height_m;  // ring faces apart by one petal height
    const auto c = contact::docking_interface_contact(pose({e + gap, 0.20, 0}), e, p);
    REQUIRE(c.petal_contact);
    CHECK(c.force_port_n.y < 0.0);
    CHECK(c.force_port_n.x > 0.0);
    CHECK_FALSE(c.ring_contact);
    CHECK_FALSE(c.latch_ready);
    CHECK(c.capture_radius_m == Catch::Approx(p.petal_clearance_m + 2.0 * p.petal_height_m * std::tan(p.petal_lean_rad)));
}

TEST_CASE("Docking interface: beyond the capture radius the petal tips strike the opposing petals", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    const double e = p.ring_extension_m;
    const double tips = e + 2.0 * p.petal_height_m;
    const auto c = contact::docking_interface_contact(pose({tips - 0.01, 0.40, 0}, {-0.075, 0, 0}), e, p);
    REQUIRE(c.tip_strike);
    CHECK(c.force_port_n.x > 0.0);
    CHECK(std::abs(c.force_port_n.y) < 1e-9);  // no centring on the tips
    CHECK_FALSE(c.latch_ready);
}

TEST_CASE("Docking interface: latch tolerances - offset, tilt beyond the ring's range, roll", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    const double e = p.ring_extension_m;
    const auto touching = Vec3{e - 0.002, 0, 0};
    CHECK(contact::docking_interface_contact(pose(touching), e, p).latch_ready);
    CHECK_FALSE(contact::docking_interface_contact(pose(touching + Vec3{0, p.latch_offset_max_m + 0.005, 0}), e, p)
                    .latch_ready);
    const auto tilt = [&](double angle) {  // chaser pitched about the passive y axis
        const double c = std::cos(angle), s = std::sin(angle);
        const Mat3 r = Mat3::from_columns({c, 0, -s}, {0, 1, 0}, {s, 0, c});
        return r * mated_axes;
    };
    CHECK(contact::docking_interface_contact(pose(touching, {}, tilt(p.ring_tilt_max_rad - 0.5 * deg)), e, p).latch_ready);
    CHECK_FALSE(
        contact::docking_interface_contact(pose(touching, {}, tilt(p.ring_tilt_max_rad + 0.5 * deg)), e, p).latch_ready);
    const auto roll = [&](double angle) {  // about the passive x axis
        const double c = std::cos(angle), s = std::sin(angle);
        return Mat3::from_columns({1, 0, 0}, {0, c, s}, {0, -s, c}) * mated_axes;
    };
    CHECK_FALSE(
        contact::docking_interface_contact(pose(touching, {}, roll(p.latch_roll_max_rad + 0.5 * deg)), e, p).latch_ready);
}

TEST_CASE("Docking interface: the IDSS outline and the dynamic load limit are the IDD values", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    CHECK(p.petal_outline_radius_m == 0.600);           // 1200 mm diameter at the SCS mating plane
    CHECK(p.petal_count == 3);
    // a 0.10 m/s ring-face impact (Dragon on the ISS, reduced mass ~12 t) peaks below the dynamic compression limit
    // (planning estimate with these values: 3,981 N; 0.25 m/s: 9,954 N)
    const double mu = 12000.0, dt = 1e-4;
    double x = p.ring_extension_m + 0.001, v = -0.10, peak = 0.0;
    for (int i = 0; i < 200000 && !(x > p.ring_extension_m + 0.001 && v > 0.0); ++i) {
        const auto c = contact::docking_interface_contact(pose({x, 0, 0}, {v, 0, 0}), p.ring_extension_m, p);
        peak = std::max(peak, c.compression_n);
        v += c.force_port_n.x / mu * dt;
        x += v * dt;
    }
    CHECK(peak > 3000.0);
    CHECK(peak < 6500.0);
    CHECK(v > 0.0);  // rebounds without a latch (restitution ~0.46)
}
```

Append to `core/tests/test_proximity_geometry.cpp`:

```cpp
TEST_CASE("Capture envelope: IDSS roll misalignment and roll rate", "[iss][proximity][docking]") {
    gnc::RelativePortState s;
    s.position_m = {0.0, 0.02, 0.0};
    s.velocity_mps = {-0.075, 0.0, 0.0};
    s.closing_speed_mps = 0.075;
    CHECK(gnc::capture_envelope_check(s).inside);
    s.clocking_rad = 4.5 * constants::deg_to_rad;
    CHECK_FALSE(gnc::capture_envelope_check(s).inside);
    CHECK(gnc::capture_envelope_check(s).reason == "Rollwinkel zu groß");
    s.clocking_rad = 0.0;
    s.angular_rate_radps = {0.25 * constants::deg_to_rad, 0.0, 0.0};  // roll rate about the docking axis
    CHECK(gnc::capture_envelope_check(s).reason == "Rollrate zu groß");
}
```

Config test - append to the existing config test file that checks `crew_dragon` (find it with
`grep -ln "idss" core/tests/*.cpp`):

```cpp
TEST_CASE("Docking ports carry the soft-capture mode", "[iss][docking][config]") {
    const auto dragon = config::load_vehicle(test::config_dir() / "vehicles" / "crew_dragon.toml");
    const auto iss = config::load_vehicle(test::config_dir() / "vehicles" / "iss.toml");
    CHECK(dragon.stages.back().docking_ports.front().soft_capture == config::SoftCaptureMode::Active);
    for (const auto& port : iss.stages.front().docking_ports) CHECK(port.soft_capture == config::SoftCaptureMode::Passive);
}
```

(Use the loader and config-directory helper names that file already uses.)

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[docking]"`
Expected: compile error (`docking_interface.hpp` missing, `soft_capture` unknown).

- [ ] **Step 3: Implement**

`config/vehicle_config.hpp` - in `DockingPortConfig`:

```cpp
enum class SoftCaptureMode { Passive, Active };  // IDSS: one side extends its soft-capture ring, the other stays retracted
...
    SoftCaptureMode soft_capture = SoftCaptureMode::Passive;  // TOML soft_capture = "active" | "passive"
```

Parsing in `vehicle_config.cpp` after `behind_nose_cone`:

```cpp
            const std::string mode = p["soft_capture"].value_or(std::string("passive"));
            if (mode == "active") {
                port.soft_capture = SoftCaptureMode::Active;
            } else if (mode != "passive") {
                doc.fail("stage.docking_port '" + port.id + "': soft_capture must be \"active\" or \"passive\"");
            }
```

`crew_dragon.toml` port: `soft_capture = "active"   # [S: IDSS IDD E 3.1.1.1] Dragon's SCS extends its ring`; the ISS
ports get `soft_capture = "passive"` explicitly with the same source. `config/README.md`: one line on the field.

`docking_interface.hpp`:

```cpp
#pragma once

#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::contact {

// IDSS soft-capture interface before capture (ISS i5), in the passive port frame (x out of the passive port, z its up;
// vehicle/docking_port.hpp). Reduced order (spec correction 2): the three petal pairs act as one axisymmetric cone
// between the ring centres; the active ring conforms in tilt within ring_tilt_max_rad (its actuator platform).

// The chaser port (its HCS mating plane centre) relative to the passive port, passive port frame (rotating with the
// passive body): mated = position 0, chaser_axes columns (-x, -y, +z).
struct DockingPose {
    math::Vec3 position_m;
    math::Vec3 velocity_mps;
    math::Mat3 chaser_axes = math::Mat3::identity();  // columns: chaser port x, y, z in the passive port frame
    math::Vec3 angular_rate_radps;                     // chaser relative to the passive body, passive port frame
};

struct DockingInterfaceParameters {
    int petal_count = 3;                    // [S: IDSS IDD E 3.2.2.1] three inward-pointing guide petals
    double petal_outline_radius_m = 0.600;  // [S: IDSS IDD E 3.2] conic outline 1200 mm at the SCS mating plane
    double petal_height_m = 0.20;           // [E] above the ring face (figures only in the IDD)
    double petal_lean_rad = 30.0 * constants::deg_to_rad;  // [E] inward lean of the petal outline from the axis
    double petal_clearance_m = 0.01;        // [E] lateral play of the meshed petals at full ring contact
    double ring_extension_m = 0.30;         // [E] active ring face above the chaser HCS plane (passive ring at 0)
    double ring_inner_radius_m = 0.40;      // [E] ring face (striker band) inner / outer radius
    double ring_outer_radius_m = 0.60;      // [S: = outline radius]
    // [E] ring-face stiffness and damping: a 0.10 m/s contact on the Dragon-ISS pair (reduced mass ~12 t) peaks
    // below the IDSS dynamic compression limit 6,500 N (test); damping ratio ~0.3 at that mass.
    double ring_stiffness_n_per_m = 2.0e5;
    double ring_damping_ns_per_m = 2.9e4;
    double petal_stiffness_n_per_m = 2.0e5;  // [E] normal to the petal cone
    double petal_damping_ns_per_m = 2.9e4;   // [E]
    double friction_coefficient = 0.1;       // [E] metal on metal, lubricated (regularised Coulomb)
    double friction_slip_mps = 1e-3;         // [E] regularisation speed
    // Latch condition (soft capture, IDSS IDD E 3.1.1.1: rings in full contact, latches on the strikers) [E]:
    double latch_gap_m = 0.005;                                // ring faces within this
    double latch_offset_max_m = 0.02;                          // ring centres after the petal centring
    double ring_tilt_max_rad = 6.0 * constants::deg_to_rad;    // the active ring conforms up to this tilt
    double latch_roll_max_rad = 6.0 * constants::deg_to_rad;   // petal flanks / latch strikers in roll
};

struct InterfaceContact {
    math::Vec3 force_port_n;    // on the chaser at the active ring centre, passive port frame (the passive body: -force)
    math::Vec3 ring_centre_m;   // where the force acts: chaser port + ring_extension along -x_c ... passive frame
    bool any_contact = false;
    bool ring_contact = false;  // ring faces touching
    bool petal_contact = false; // on the petal cone (centring)
    bool tip_strike = false;    // petal tips on the opposing petals (offset beyond the capture radius)
    bool latch_ready = false;   // latch condition met
    double compression_n = 0.0; // axial load on the interface (+: compression)
    double capture_radius_m = 0.0;
};

// `ring_extension_m`: current extension of the active ring (the mechanism retracts it later; contact before capture
// uses the full extension).
InterfaceContact docking_interface_contact(const DockingPose& pose, double ring_extension_m,
                                           const DockingInterfaceParameters& p);

}  // namespace starsim::contact
```

`docking_interface.cpp` - the model:

```cpp
#include "starsim/contact/docking_interface.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::contact {

namespace {
// Regularised Coulomb friction opposing the tangential velocity, bounded by mu * normal.
math::Vec3 friction(const math::Vec3& v_tangential, double normal_n, const DockingInterfaceParameters& p) {
    const double speed = norm(v_tangential);
    if (speed <= 0.0 || normal_n <= 0.0) return {};
    const double scale = std::min(1.0, speed / p.friction_slip_mps);
    return -(p.friction_coefficient * normal_n * scale / speed) * v_tangential;
}
}  // namespace

InterfaceContact docking_interface_contact(const DockingPose& pose, double ring_extension_m,
                                           const DockingInterfaceParameters& p) {
    InterfaceContact c;
    const math::Vec3 xc = column(pose.chaser_axes, 0);  // chaser docking axis (points towards the passive port)
    // The active ring face centre: ring_extension_m along the chaser docking axis from its port.
    c.ring_centre_m = pose.position_m + ring_extension_m * xc;
    const math::Vec3 ring_velocity = pose.velocity_mps + cross(pose.angular_rate_radps, ring_extension_m * xc);
    c.capture_radius_m = p.petal_clearance_m + 2.0 * p.petal_height_m * std::tan(p.petal_lean_rad);
    const double gap = c.ring_centre_m.x;  // ring face to ring face along the passive axis (the ring conforms in tilt)
    const math::Vec3 lateral{0.0, c.ring_centre_m.y, c.ring_centre_m.z};
    const double offset = norm(lateral);
    const double overlap = 2.0 * p.petal_height_m;
    if (gap >= overlap) return c;  // petals not yet meshing

    const math::Vec3 x{1, 0, 0};
    if (offset > c.capture_radius_m) {
        // tips on the opposing petals: an axial contact at the tip plane, no centring
        const double depth = overlap - gap;
        const double normal = std::max(0.0, p.ring_stiffness_n_per_m * depth - p.ring_damping_ns_per_m * ring_velocity.x);
        c.tip_strike = c.any_contact = true;
        c.force_port_n = normal * x + friction({0.0, ring_velocity.y, ring_velocity.z}, normal, p);
        c.compression_n = normal;
        return c;
    }
    // petal cone: allowed offset grows with the gap
    const double clearance = p.petal_clearance_m + std::max(gap, 0.0) * std::tan(p.petal_lean_rad);
    if (offset > clearance) {
        const double depth = (offset - clearance) * std::cos(p.petal_lean_rad);  // normal to the cone face
        const math::Vec3 radial = lateral / offset;
        const math::Vec3 n = -std::cos(p.petal_lean_rad) * radial + std::sin(p.petal_lean_rad) * x;  // into the chaser
        const double vn = dot(ring_velocity, n);
        const double normal = std::max(0.0, p.petal_stiffness_n_per_m * depth - p.petal_damping_ns_per_m * vn);
        c.petal_contact = c.any_contact = normal > 0.0;
        c.force_port_n += normal * n + friction(ring_velocity - vn * n, normal, p);
        c.compression_n += normal * std::sin(p.petal_lean_rad);
    }
    if (gap < 0.0) {
        const double normal = std::max(0.0, -p.ring_stiffness_n_per_m * gap - p.ring_damping_ns_per_m * ring_velocity.x);
        c.ring_contact = c.any_contact = c.any_contact || normal > 0.0;
        c.force_port_n += normal * x + friction({0.0, ring_velocity.y, ring_velocity.z}, normal, p);
        c.compression_n += normal;
    }
    // latch condition: faces within the gap, centred, tilt within the ring's range, roll within the strikers'
    const double tilt = std::acos(std::clamp(-xc.x, -1.0, 1.0));  // mated: x_c = -x
    const math::Vec3 zc = column(pose.chaser_axes, 2);
    const double roll = std::atan2(-zc.y, zc.z);  // up of the chaser against the passive up about x
    c.latch_ready = gap <= p.latch_gap_m && offset <= p.latch_offset_max_m && tilt <= p.ring_tilt_max_rad &&
                    std::abs(roll) <= p.latch_roll_max_rad;
    return c;
}

}  // namespace starsim::contact
```

(The roll sign: with `roll(angle)` of the test about +x the chaser up becomes (0, -s, c) -> `atan2(s, c)` = angle;
check against the test and fix the sign there, not by abs-ing it away.)

`proximity_geometry.hpp` `CaptureEnvelope` - re-tag and extend:

```cpp
struct CaptureEnvelope {  // [S: IDSS IDD E Table 3.3.1.1-2, Initial Contact Conditions]
    double axial_min_mps = 0.05, axial_max_mps = 0.10;
    double lateral_max_mps = 0.04;
    double offset_max_m = 0.10;
    double angle_max_rad = 4.0 * constants::deg_to_rad;       // pitch/yaw, vector sum
    double rate_max_radps = 0.2 * constants::deg_to_rad;      // pitch/yaw rate, vector sum
    double roll_max_rad = 4.0 * constants::deg_to_rad;
    double roll_rate_max_radps = 0.2 * constants::deg_to_rad;
};
```

In `capture_envelope_check` (keep the existing order, append the roll checks after the angle and rate checks): the
pitch/yaw rate is the rate perpendicular to the target axis, the roll rate its x component:

```cpp
    const math::Vec3 w = state.angular_rate_radps;
    if (std::hypot(w.y, w.z) > envelope.rate_max_radps) return {false, "Drehrate zu groß"};  // existing wording kept
    if (std::abs(state.clocking_rad) > envelope.roll_max_rad) return {false, "Rollwinkel zu groß"};
    if (std::abs(w.x) > envelope.roll_rate_max_radps) return {false, "Rollrate zu groß"};
```

(If the existing rate check uses `norm(w)`, change it to the pitch/yaw part as above and keep its German reason; the
i4 campaigns' rates were <= 0.039 deg/s, so they stay inside.) `contact_plane_m` comment: "i5: the session sets the
petal mesh plane of the docking interface".

CMake: add `src/contact/docking_interface.cpp` to the core sources, `test_docking_interface.cpp` to the tests.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[docking]"` then
`-TestFilter "[proximity]"` (the envelope change must keep every i4 campaign inside).
Expected: PASS.

- [ ] **Step 5: Commit and push**

`-Fast` first. Then:

```bash
git add core/include/starsim/contact/docking_interface.hpp core/src/contact/docking_interface.cpp core/include/starsim/gnc/proximity_geometry.hpp core/src/gnc/proximity_geometry.cpp core/include/starsim/config/vehicle_config.hpp core/src/config/vehicle_config.cpp config/vehicles/crew_dragon.toml config/vehicles/iss.toml config/README.md core/CMakeLists.txt core/tests/CMakeLists.txt core/tests/test_docking_interface.cpp core/tests/test_proximity_geometry.cpp <config test file>
git commit -m "feat(contact): IDSS soft-capture interface - ring faces, petal cone, latch condition; envelope [S] with roll"
git push origin main
```

---

### Task 2: Docking mechanism state machine (soft capture, joint, retraction, hooks, undocking)

**Files:**
- Create: `core/include/starsim/contact/docking_mechanism.hpp`, `core/src/contact/docking_mechanism.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`
- Test: `core/tests/test_docking_mechanism.cpp`

**Interfaces:**
- Consumes: `contact::DockingPose`, `docking_interface_contact`, `DockingInterfaceParameters` (Task 1).
- Produces:
  - `enum class DockingPhase { Ready, Contact, SoftCapture, Retraction, HooksClosing, Docked, HooksOpening, Separated,
    Failed }`, `const char* to_string(DockingPhase)` (German: "bereit", "Kontakt", "Weichfang", "Einzug",
    "Haken schließen", "angedockt", "Haken öffnen", "getrennt", "Fangen fehlgeschlagen").
  - `struct DockingMechanismParameters` (interface + joint + timeline + limits).
  - `struct DockingWrench { Vec3 force_port_n; Vec3 point_port_m; Vec3 torque_port_nm; }` - on the chaser at the
    point (the passive body gets -force through its CG).
  - `class DockingMechanism { DockingWrench update(double t_s, const DockingPose&, double chaser_mass_kg, double
    chaser_inertia_kg_m2, double dt_s); bool undock(double t_s); bool hard_captured() const; bool separation_due();
    DockingPhase phase(); const std::string& status(); const std::string& failure(); double peak_compression_n();
    LoadPeaks peaks(); std::vector<DockingEvent> events(); void rearm(); }`.

Design:
- `Ready`: `docking_interface_contact` every update; first contact -> `Contact` (event "Erstkontakt").
- `Contact`: interface wrench; `latch_ready` -> `SoftCapture` (event "Weichfang: Verriegelung eingerastet"); the
  joint gains are set from the chaser mass and inertia at that moment. Contact lost (no contact for
  `bounce_confirm_s` [E 2 s] and the ring gap grows beyond `bounce_gap_m` [E 0.05 m]) -> `Failed` with the reason:
  "abgeprallt" if a ring contact happened, "Versatz zu groß" after tip strikes only, "Winkel zu groß" if the ring
  touched but the tilt/roll prevented the latch.
- `SoftCapture` / `Retraction`: the **joint** - a 6-DOF spring-damper from the chaser port to its target pose (axial
  rest `x_rest`, centred, mated axes): `F = -K (p - p_rest) - C v`, `T = -K_r theta - C_r w` (theta: rotation vector
  from the mated axes to the chaser axes), with `K = m w_n^2`, `C = 2 zeta m w_n`, `K_r = I w_r^2`,
  `C_r = 2 zeta I w_r` (`m`, `I` the chaser's at capture); axial compression beyond `stroke_m` [E] adds an end stop
  (`end_stop_stiffness`) - the load then shows. `x_rest` = `ring_extension_m` in SoftCapture; SoftCapture ->
  Retraction after `attenuation_min_s` and relative speed < `settled_speed_mps`, rate < `settled_rate_radps`;
  Retraction lowers `x_rest` at `retraction_speed_mps` to 0; then guide pins: |p| <= `guide_pin_tolerance_m`,
  angle <= `guide_pin_angle_rad` -> `HooksClosing` (`hooks_close_s`) -> `Docked` (`hard_captured()` true; the session
  merges). Retraction that cannot reach the tolerances within `retraction_timeout_s` -> Failed "Einzug fehlgeschlagen".
- **Loads** at the SCS mating plane (force components along / across the axis, moment about the plane centre) each
  update: peaks of compression, tension, shear, bending, torsion. A compression > `dynamic_compression_limit_n`
  (6,500 N [S]) at any time before Docked, or a static compression above 3,500 N held > 0.1 s -> `Failed`
  "Überlast" (the latches release: the joint ends, the interface contact resumes, the chaser rebounds). Tension /
  shear / bending / torsion above the IDSS limits -> a warning event (German), not a failure.
- `undock(t)`: only in Docked -> `HooksOpening` (`hooks_open_s`) -> `Separated`: `separation_due()` returns true once
  (the session then applies the spring Delta-v and releases the hold).
- `rearm()`: from Failed back to Ready (a new approach); refused (no-op) in other phases.

- [ ] **Step 1: Write the failing tests**

`core/tests/test_docking_mechanism.cpp` - a 1-D/6-DOF point model of the chaser against a fixed passive port (the
station is ~38x heavier; the reduced-mass effect is checked in the session tests):

```cpp
// ISS i5: IDSS docking mechanism - capture inside the envelope, failures outside, timeline, loads, undocking.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/contact/docking_mechanism.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Mat3;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
const Mat3 mated = Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1});

// Chaser rigid body reduced to its port point (lever folded into the rotational inertia): translational state of the
// port and a small-angle attitude about the passive y / z / x axes. Fixed passive port. dt 5 ms as the session.
struct PortModel {
    double mass_kg = 12000.0;
    double inertia_kg_m2 = 60000.0;  // about the port point [IDSS-15T class + m l^2 with the lever folded in]
    Vec3 p, v, angle, rate;          // angle: small rotation vector of the chaser axes from mated
    contact::DockingPose pose() const {
        // small-angle rotation applied to the mated axes
        const Mat3 r = Mat3::from_columns({1, angle.z, -angle.y}, {-angle.z, 1, angle.x}, {angle.y, -angle.x, 1});
        return {p, v, r * mated, rate};
    }
    void step(const contact::DockingWrench& w, double dt) {
        v += w.force_port_n / mass_kg * dt;
        const Vec3 lever = w.point_port_m - p;
        rate += (w.torque_port_nm + cross(lever, w.force_port_n)) / inertia_kg_m2 * dt;
        p += v * dt;
        angle += rate * dt;
    }
};

struct Run {
    contact::DockingMechanism m;
    PortModel c;
    double t = 0.0;
    void fly(double seconds) {
        const double dt = 0.005;
        for (double end = t + seconds; t < end; t += dt) c.step(m.update(t, c.pose(), c.mass_kg, c.inertia_kg_m2, dt), dt);
    }
};

Run approach(double axial_mps, double offset_m, double angle_rad = 0.0, double lateral_mps = 0.0) {
    Run r;
    const contact::DockingMechanismParameters p;
    r.c.p = {p.interface.ring_extension_m + 2.0 * p.interface.petal_height_m + 0.05, offset_m, 0.0};
    r.c.v = {-axial_mps, lateral_mps, 0.0};
    r.c.angle = {0.0, angle_rad, 0.0};
    return r;
}
}  // namespace

TEST_CASE("Docking mechanism: nominal contact captures, retracts and hard-captures in ~11 min", "[iss][docking]") {
    auto r = approach(0.075, 0.03);
    r.fly(20.0);
    REQUIRE(r.m.phase() == contact::DockingPhase::SoftCapture);
    r.fly(900.0);
    CHECK(r.m.phase() == contact::DockingPhase::Docked);
    CHECK(r.m.hard_captured());
    const auto& ev = r.m.events();
    const double soft = ev.front().t_s;  // first contact ~ soft capture
    const double hard = ev.back().t_s;
    CHECK(hard - soft > 9.0 * 60.0);
    CHECK(hard - soft < 13.0 * 60.0);  // Demo-2: 11 min
    CHECK(norm(r.c.p) < 0.005);
    CHECK(r.m.peaks().compression_n < 6500.0);
}

TEST_CASE("Docking mechanism: every corner of the IDSS envelope captures within the load limits", "[iss][docking]") {
    for (double axial : {0.05, 0.10}) {
        for (double offset : {0.0, 0.10}) {
            for (double angle : {0.0, 4.0 * deg}) {
                for (double lateral : {0.0, -0.04}) {
                    auto r = approach(axial, offset, angle, lateral);
                    r.fly(60.0);
                    INFO("axial " << axial << " offset " << offset << " angle " << angle << " lateral " << lateral);
                    CHECK(r.m.phase() == contact::DockingPhase::SoftCapture);
                    CHECK(r.m.peaks().compression_n < 6500.0);
                    CHECK(r.m.peaks().shear_n < 3200.0);
                }
            }
        }
    }
}

TEST_CASE("Docking mechanism: too fast overloads, the latches release, the chaser rebounds", "[iss][docking]") {
    auto r = approach(0.25, 0.0);
    r.fly(60.0);
    CHECK(r.m.phase() == contact::DockingPhase::Failed);
    CHECK(r.m.failure() == "Überlast");
    CHECK(r.c.v.x > 0.0);  // moving out again
}

TEST_CASE("Docking mechanism: an offset beyond the petals' capture radius bounces off the tips", "[iss][docking]") {
    auto r = approach(0.075, 0.30);
    r.fly(60.0);
    CHECK(r.m.phase() == contact::DockingPhase::Failed);
    CHECK(r.m.failure() == "Versatz zu groß");
    CHECK(r.c.v.x > 0.0);
}

TEST_CASE("Docking mechanism: rearm after a failure, then a good approach captures", "[iss][docking]") {
    auto r = approach(0.075, 0.30);
    r.fly(60.0);
    REQUIRE(r.m.phase() == contact::DockingPhase::Failed);
    r.m.rearm();
    CHECK(r.m.phase() == contact::DockingPhase::Ready);
    const contact::DockingMechanismParameters p;
    r.c.p = {p.interface.ring_extension_m + 2.0 * p.interface.petal_height_m + 0.05, 0.02, 0.0};
    r.c.v = {-0.075, 0.0, 0.0};
    r.c.rate = {};
    r.fly(30.0);
    CHECK(r.m.phase() == contact::DockingPhase::SoftCapture);
}

TEST_CASE("Docking mechanism: undock opens the hooks, then the separation is due once", "[iss][docking]") {
    auto r = approach(0.075, 0.0);
    r.fly(900.0);
    REQUIRE(r.m.phase() == contact::DockingPhase::Docked);
    CHECK_FALSE(r.m.separation_due());
    REQUIRE(r.m.undock(r.t));
    CHECK_FALSE(r.m.undock(r.t));  // second command refused
    r.fly(contact::DockingMechanismParameters{}.hooks_open_s + 1.0);
    CHECK(r.m.phase() == contact::DockingPhase::Separated);
    CHECK(r.m.separation_due());
    CHECK_FALSE(r.m.separation_due());
}

TEST_CASE("Docking mechanism: undock refused before hard capture", "[iss][docking]") {
    auto r = approach(0.075, 0.0);
    r.fly(20.0);
    REQUIRE(r.m.phase() == contact::DockingPhase::SoftCapture);
    CHECK_FALSE(r.m.undock(r.t));
    CHECK(r.m.refusal() == "Abkoppeln nicht möglich: nicht fest angedockt");
}

TEST_CASE("Docking mechanism: separation speed from the IDSS spring energy", "[iss][docking]") {
    const contact::DockingMechanismParameters p;
    // Dragon 12 t on the 460 t ISS: v_rel = sqrt(2 E / mu)
    const double mu = 12000.0 * 460320.0 / (12000.0 + 460320.0);
    CHECK(contact::separation_relative_speed_mps(p, 12000.0, 460320.0) == Catch::Approx(std::sqrt(2.0 * 43.35 / mu)));
    CHECK(contact::separation_relative_speed_mps(p, 12000.0, 460320.0) == Catch::Approx(0.0862).margin(0.002));
}
```

- [ ] **Step 2: Run to verify failure**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[docking]"`
Expected: compile error (`docking_mechanism.hpp` missing).

- [ ] **Step 3: Implement**

`docking_mechanism.hpp`:

```cpp
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "starsim/contact/docking_interface.hpp"

namespace starsim::contact {

// IDSS docking sequence of an active soft-capture system against a passive port (ISS i5; IDSS IDD E 3.1.1.1):
// petals -> latches on full ring contact (soft capture) -> attenuation by a damped 6-DOF joint -> retraction ->
// guide pins -> 12 hooks (hard capture). Undocking: hooks open -> separation springs. Passive port frame throughout.

enum class DockingPhase { Ready, Contact, SoftCapture, Retraction, HooksClosing, Docked, HooksOpening, Separated, Failed };
const char* to_string(DockingPhase phase);  // German

struct DockingMechanismParameters {
    DockingInterfaceParameters interface;
    double joint_frequency_radps = 0.5;   // [E] attenuation: critically damped, time constant 2 s
    double joint_rotation_frequency_radps = 0.5;  // [E]
    double joint_damping_ratio = 1.0;     // [E]
    double stroke_m = 0.10;               // [E] axial attenuation stroke beyond the rest extension
    double end_stop_stiffness_n_per_m = 2.0e6;  // [E] beyond the stroke
    double attenuation_min_s = 60.0;      // [E] then retract once settled
    double settled_speed_mps = 0.005;     // [E]
    double settled_rate_radps = 0.05 * constants::deg_to_rad;  // [E]
    double retraction_speed_mps = 0.001;  // [E] 0.30 m in 300 s
    double retraction_timeout_s = 900.0;  // [E]
    double guide_pin_tolerance_m = 0.005; // [E] fine alignment before the hooks
    double guide_pin_angle_rad = 0.5 * constants::deg_to_rad;  // [E]
    double hooks_close_s = 180.0;         // [E] with attenuation + retraction: 11 min soft -> hard [S: Demo-2]
    double hooks_open_s = 180.0;          // [E]
    int hook_count = 12;                  // [S: IDSS IDD E 3.2.3.4]
    double separation_energy_j = 43.35;   // [E] middle of 39.2-47.5 N m [S: IDSS IDD E 3.2.3.6.2]
    double bounce_confirm_s = 2.0;        // [E] no contact this long ...
    double bounce_gap_m = 0.05;           // [E] ... and the ring this far out: capture failed
    // IDSS IDD E Table 3.3.1.4-1, SCS maximum interface loads (design limit loads) [S]:
    double dynamic_compression_limit_n = 6500.0;  // up to 0.1 s: above it the mechanism releases (Überlast)
    double static_compression_limit_n = 3500.0;   // held longer than static_after_s: Überlast
    double static_after_s = 0.1;
    double tension_limit_n = 3900.0, shear_limit_n = 3200.0, bending_limit_nm = 2800.0, torsion_limit_nm = 1500.0;
};

struct DockingWrench {
    math::Vec3 force_port_n;   // on the chaser, acting at point_port_m (the passive body: -force through its CG)
    math::Vec3 point_port_m;
    math::Vec3 torque_port_nm; // pure couple on the chaser (joint rotation)
};
struct LoadPeaks {
    double compression_n = 0.0, tension_n = 0.0, shear_n = 0.0, bending_nm = 0.0, torsion_nm = 0.0;
};
struct DockingEvent {
    double t_s = 0.0;
    DockingPhase phase;
    std::string status;  // German
};

// Relative speed of the separation springs: v = sqrt(2 E / mu), mu the reduced mass.
double separation_relative_speed_mps(const DockingMechanismParameters& p, double chaser_mass_kg, double passive_mass_kg);

class DockingMechanism {
public:
    explicit DockingMechanism(DockingMechanismParameters p = {});
    // One tick: the wrench on the chaser for the next step. `chaser_inertia_kg_m2`: its largest principal moment plus
    // m l^2 about the port (the joint's rotational gain).
    DockingWrench update(double t_s, const DockingPose& pose, double chaser_mass_kg, double chaser_inertia_kg_m2,
                         double dt_s);
    bool undock(double t_s);       // Docked -> HooksOpening; false + refusal() otherwise
    bool separation_due();         // true once, when Separated is reached
    void rearm();                  // Failed -> Ready
    bool hard_captured() const { return phase_ == DockingPhase::Docked; }
    DockingPhase phase() const { return phase_; }
    const std::string& status() const { return status_; }
    const std::string& failure() const { return failure_; }
    const std::string& refusal() const { return refusal_; }
    const LoadPeaks& peaks() const { return peaks_; }
    const std::vector<DockingEvent>& events() const { return events_; }
    const std::vector<std::string>& warnings() const { return warnings_; }
    double ring_extension_m() const { return extension_m_; }
    const DockingMechanismParameters& parameters() const { return p_; }

private:
    void enter(DockingPhase phase, std::string status, double t_s);
    void fail(std::string reason, double t_s);
    DockingWrench joint(const DockingPose& pose) const;
    void record_loads(const DockingWrench& w, const DockingPose& pose, double t_s, double dt_s);

    DockingMechanismParameters p_;
    DockingPhase phase_ = DockingPhase::Ready;
    double extension_m_ = 0.0;
    double phase_since_s_ = 0.0, last_contact_s_ = 0.0, static_over_s_ = 0.0;
    bool ring_touched_ = false, ring_blocked_ = false, tips_only_ = true;
    double k_ = 0.0, c_ = 0.0, kr_ = 0.0, cr_ = 0.0;
    bool separation_pending_ = false;
    LoadPeaks peaks_;
    std::string status_ = "bereit", failure_, refusal_;
    std::vector<DockingEvent> events_;
    std::vector<std::string> warnings_;
};

}  // namespace starsim::contact
```

`docking_mechanism.cpp` - key parts (write the rest in the same style):

```cpp
namespace {
// Rotation vector of R (axis * angle), R = chaser axes * mated^T.
math::Vec3 rotation_vector(const math::Mat3& r) {
    const double c = std::clamp(0.5 * (r(0, 0) + r(1, 1) + r(2, 2) - 1.0), -1.0, 1.0);
    const double angle = std::acos(c);
    const math::Vec3 v{r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1)};
    if (angle < 1e-9) return 0.5 * v;
    return (angle / (2.0 * std::sin(angle))) * v;
}
const math::Mat3 mated_axes = math::Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1});
}  // namespace

double separation_relative_speed_mps(const DockingMechanismParameters& p, double m1, double m2) {
    const double mu = m1 * m2 / (m1 + m2);
    return std::sqrt(2.0 * p.separation_energy_j / mu);
}

DockingWrench DockingMechanism::joint(const DockingPose& pose) const {
    const math::Vec3 rest{extension_m_, 0.0, 0.0};  // chaser port target: ring extension out of the passive port
    // (the chaser's ring face then sits on the passive ring face; the port itself is extension_m_ out)
    math::Vec3 force = -k_ * (pose.position_m - rest) - c_ * pose.velocity_mps;
    const double compression = extension_m_ - pose.position_m.x - p_.stroke_m;
    if (compression > 0.0) force.x += p_.end_stop_stiffness_n_per_m * compression;
    const math::Vec3 theta = rotation_vector(pose.chaser_axes * transpose(mated_axes));
    return {force, pose.position_m, -kr_ * theta - cr_ * pose.angular_rate_radps};
}
```

The joint rest: the chaser port sits `extension_m_` out of the passive port while the ring is extended (the ring face
on the passive ring face); retraction drives `extension_m_` to 0 (the HCS planes meet). `update`:

```cpp
DockingWrench DockingMechanism::update(double t, const DockingPose& pose, double m, double inertia, double dt) {
    DockingWrench w{{}, pose.position_m, {}};
    switch (phase_) {
        case DockingPhase::Ready:
        case DockingPhase::Contact:
        case DockingPhase::Failed: {
            extension_m_ = p_.interface.ring_extension_m;  // before capture the ring is fully extended
            const auto c = docking_interface_contact(pose, extension_m_, p_.interface);
            w = {c.force_port_n, c.ring_centre_m, {}};
            if (c.any_contact) {
                last_contact_s_ = t;
                if (phase_ == DockingPhase::Ready) enter(DockingPhase::Contact, "Erstkontakt", t);
                ring_touched_ = ring_touched_ || c.ring_contact;
                tips_only_ = tips_only_ && c.tip_strike;
                ring_blocked_ = ring_blocked_ || (c.ring_contact && !c.latch_ready);
            }
            if (phase_ == DockingPhase::Contact && c.latch_ready) {
                k_ = m * p_.joint_frequency_radps * p_.joint_frequency_radps;
                c_ = 2.0 * p_.joint_damping_ratio * m * p_.joint_frequency_radps;
                kr_ = inertia * p_.joint_rotation_frequency_radps * p_.joint_rotation_frequency_radps;
                cr_ = 2.0 * p_.joint_damping_ratio * inertia * p_.joint_rotation_frequency_radps;
                enter(DockingPhase::SoftCapture, "Weichfang: Verriegelung eingerastet", t);
                w = joint(pose);
            } else if (phase_ == DockingPhase::Contact && t - last_contact_s_ > p_.bounce_confirm_s &&
                       c.ring_centre_m.x > 2.0 * p_.interface.petal_height_m + p_.bounce_gap_m) {
                fail(tips_only_ ? "Versatz zu groß" : (ring_blocked_ ? "Winkel zu groß" : "abgeprallt"), t);
            }
            break;
        }
        case DockingPhase::SoftCapture: ... // joint; settled after attenuation_min_s -> Retraction
        case DockingPhase::Retraction: ...  // extension_m_ -= retraction_speed * dt (>= 0); guide pins -> HooksClosing
        case DockingPhase::HooksClosing: ... // joint holds; after hooks_close_s -> Docked ("angedockt: 12 Haken geschlossen")
        case DockingPhase::Docked: return {};  // the session holds the chaser on the station: no wrench
        case DockingPhase::HooksOpening: ... // no wrench (still held by the session); after hooks_open_s -> Separated
        case DockingPhase::Separated: return {};
    }
    record_loads(w, pose, t, dt);  // Überlast -> fail("Überlast", t): the latches release, w = interface contact
    return w;
}
```

The elided cases follow the design bullets above literally (each `...` is one bullet). `fail()` resets `ring_touched_`, `tips_only_ = true`, `ring_blocked_` in `rearm()`. In `record_loads` the
interface frame is the passive SCS mating plane at the passive port: compression = `force.x` (positive pushes the
chaser out), tension = `-force.x`, shear = `hypot(force.y, force.z)`, bending = the moment about the plane centre
perpendicular to x, torsion = its x part. In SoftCapture / Retraction / HooksClosing an `Überlast` failure ends the
joint (latches released; the next update uses the interface contact). Settled: `norm(pose.velocity_mps) <
settled_speed_mps && norm(pose.angular_rate_radps) < settled_rate_radps`. Warnings (German, once each): "Zuglast über
IDSS-Grenze", "Querlast über IDSS-Grenze", "Biegemoment über IDSS-Grenze", "Torsionsmoment über IDSS-Grenze".

CMake: add the source and the test.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[docking]"`
Expected: PASS. If an envelope corner fails to capture, tune only [E] values (petal lean / height, ring stiffness,
latch tolerances, joint frequency) - never the IDSS [S] limits - and record the ruling and the numbers (peak loads of
the corners, soft->hard time) in the ledger.

- [ ] **Step 5: Commit and push**

`-Fast`, then:

```bash
git add core/include/starsim/contact/docking_mechanism.hpp core/src/contact/docking_mechanism.cpp core/CMakeLists.txt core/tests/CMakeLists.txt core/tests/test_docking_mechanism.cpp
git commit -m "feat(contact): IDSS docking mechanism - soft capture joint, retraction, hooks, overload, undocking"
git push origin main
```

---

### Task 3: Hull collision against the station

**Files:**
- Create: `core/include/starsim/contact/station_collision.hpp`, `core/src/contact/station_collision.cpp`
- Modify: `core/include/starsim/contact/primitives.hpp`, `core/src/contact/primitives.cpp`,
  `core/include/starsim/config/vehicle_config.hpp` (`StageConfig::hull_top_diameter_m`), `core/src/config/vehicle_config.cpp`,
  `config/vehicles/crew_dragon.toml` (capsule `hull_top_diameter_m`), `config/README.md`, CMake lists
- Test: `core/tests/test_station_collision.cpp`

**Interfaces:**
- Consumes: `config::VehicleConfig` (stages: `length_m`, `diameter_m`, collision primitives), `contact::OrientedBox`,
  `contact::point_in_box`.
- Produces:
  - `contact::Penetration point_in_capsule(const Vec3& a, const Vec3& b, double radius_m, const Vec3& point)`.
  - `std::vector<Vec3> hull_sample_points(const config::VehicleConfig&, const StationCollisionParameters&)` - chaser
    body frame (vehicle x from the bottom of stage 0); rings every `ring_spacing_m` with `points_per_ring`; a stage
    with `hull_top_diameter_m` tapers linearly; the top `port_clearance_m` below a docking port of the top stage is
    left out (the docking system region).
  - `struct StationShape { std::vector<OrientedBox> boxes; struct Capsule { Vec3 a, b; double r; }; std::vector<Capsule>
    capsules; }` and `StationShape station_shape(const config::VehicleConfig&)` (station body frame).
  - `struct HullContact { Vec3 force_station_n; Vec3 torque_station_nm; bool any = false; double max_depth_m = 0.0; }`
    and `HullContact hull_contact(const StationShape&, const std::vector<Vec3>& points_station_m, const
    std::vector<Vec3>& velocities_station_mps, const Vec3& chaser_cg_station_m, const StationCollisionParameters&)` -
    force and torque about the chaser CG on the chaser, station body frame (the station gets -force).

- [ ] **Step 1: Write the failing tests**

```cpp
// ISS i5: hull contact of a visiting vehicle with the station's collision primitives.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/station_collision.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("point_in_capsule: depth and outward normal", "[iss][docking]") {
    const auto in = contact::point_in_capsule({0, 0, 0}, {10, 0, 0}, 2.0, {5, 1.5, 0});
    CHECK(in.inside);
    CHECK(in.depth_m == Catch::Approx(0.5));
    CHECK(in.normal.y == Catch::Approx(1.0));
    CHECK_FALSE(contact::point_in_capsule({0, 0, 0}, {10, 0, 0}, 2.0, {12.5, 0, 0}).inside);  // beyond the cap
    CHECK(contact::point_in_capsule({0, 0, 0}, {10, 0, 0}, 2.0, {11.5, 0, 0}).inside);       // in the end cap
}

TEST_CASE("Hull sample points of the Crew Dragon: trunk, tapered capsule, docking system left out", "[iss][docking]") {
    const auto dragon = config::load_vehicle(test::config_dir() / "vehicles" / "crew_dragon.toml");
    const contact::StationCollisionParameters p;
    const auto pts = contact::hull_sample_points(dragon, p);
    REQUIRE(pts.size() > 50);
    double max_x = 0.0, max_r_top = 0.0;
    for (const auto& q : pts) {
        max_x = std::max(max_x, q.x);
        if (q.x > 6.5) max_r_top = std::max(max_r_top, std::hypot(q.y, q.z));
    }
    CHECK(max_x < 3.7 + 4.0 - p.port_clearance_m + 1e-9);  // below the docking port region (port at capsule 4.0 m)
    CHECK(max_r_top < 1.5);                                // the capsule narrows towards the top
}

TEST_CASE("Hull contact: a Dragon point inside the Harmony capsule is pushed out; outside nothing", "[iss][docking]") {
    const auto iss = config::load_vehicle(test::config_dir() / "vehicles" / "iss.toml");
    const auto shape = contact::station_shape(iss);
    const contact::StationCollisionParameters p;
    // a point 0.1 m inside us_modules (axis x 26..69, radius 2.2) from +y, moving inward at 0.05 m/s
    const auto c = contact::hull_contact(shape, {{50.0, 2.1, 0.0}}, {{0.0, -0.05, 0.0}}, {50.0, 4.0, 0.0}, p);
    REQUIRE(c.any);
    CHECK(c.force_station_n.y > 0.0);
    CHECK(c.max_depth_m == Catch::Approx(0.1));
    CHECK_FALSE(contact::hull_contact(shape, {{50.0, 2.5, 0.0}}, {{}}, {50.0, 4.0, 0.0}, p).any);
}

TEST_CASE("Hull contact: a Dragon docked at either IDA touches nothing", "[iss][docking]") {
    // Docked pose: chaser port at the station port, chaser x axis opposed to the port axis.
    const auto iss = config::load_vehicle(test::config_dir() / "vehicles" / "iss.toml");
    const auto dragon = config::load_vehicle(test::config_dir() / "vehicles" / "crew_dragon.toml");
    const auto shape = contact::station_shape(iss);
    const contact::StationCollisionParameters p;
    const auto pts = contact::hull_sample_points(dragon, p);
    const double port_station = 3.7 + 4.0;  // Dragon port in its vehicle frame
    for (const auto& port : iss.stages.front().docking_ports) {
        std::vector<Vec3> in_station;
        for (const auto& q : pts) {
            // chaser x runs against the port axis: the port point maps onto the station port, lateral q.y / q.z onto two
            // perpendicular unit vectors of the port (the hull is axisymmetric, the clocking does not matter here)
            const double along = port_station - q.x;  // outward from the station port
            in_station.push_back(port.position_m + along * port.axis + q.y * port.up + q.z * cross(port.axis, port.up));
        }
        const auto c = contact::hull_contact(shape, in_station, std::vector<Vec3>(in_station.size()), port.position_m, p);
        INFO(port.id);
        CHECK_FALSE(c.any);
    }
}
```

- [ ] **Step 2: Run to verify failure** - `-TestFilter "[docking]"`, expected compile error.

- [ ] **Step 3: Implement**

`point_in_capsule`: closest point on the segment (`t = clamp(dot(p - a, b - a) / |b - a|^2, 0, 1)`), distance `d`;
inside when `d < r`, depth `r - d`, normal `(p - closest) / d` (any perpendicular when `d == 0`).

`StationCollisionParameters`:

```cpp
struct StationCollisionParameters {
    double ring_spacing_m = 0.5;      // [E] hull sample rings along the vehicle
    int points_per_ring = 16;         // [E]
    double port_clearance_m = 1.0;    // [E] the top metre below a docking port (docking system, nose cone hinge) is not
                                      // sampled - the docking interface owns that contact
    double stiffness_n_per_m = 1.0e6; // [E] hull penalty: omega ~9 rad/s on 12 t (stable at 200 Hz)
    double damping_ns_per_m = 1.1e5;  // [E] damping ratio ~0.5
    double check_range_m = 120.0;     // [E] CG distance below which the hull is checked (ISS half span ~55 m + margin)
};
```

`hull_sample_points`: for each stage `s` with base `b` (sum of the lengths below) rings at
`b + k * ring_spacing_m` up to `b + length_m` (top stage: up to `port.position_m.x + b - port_clearance_m` when it has a
docking port), radius `0.5 * diameter_m` linearly to `0.5 * hull_top_diameter_m` (default: `diameter_m`).
`crew_dragon.toml` capsule: `hull_top_diameter_m = 1.8   # [E] sidewall ~15 deg from 4.0 m at the base (wiki_dragon2 images)`.

`station_shape`: capsules from `kind = "capsule"` (`a_m`, `b_m`, `radius_m`), boxes from `kind = "box"` (centre
`a_m`, body-aligned axes, `half_extents_m`), offset by the stage's base station (0 for the one-stage ISS).

`hull_contact`: for each point the deepest penetration over all primitives, normal force `k d - c dot(v, n)`
(>= 0) along `n`, summed as force; torque `cross(point - chaser_cg, f)`.

- [ ] **Step 4: Run** - `-TestFilter "[docking]"`: PASS.

- [ ] **Step 5: Commit and push** (`-Fast` first):

```bash
git add core/include/starsim/contact/station_collision.hpp core/src/contact/station_collision.cpp core/include/starsim/contact/primitives.hpp core/src/contact/primitives.cpp core/include/starsim/config/vehicle_config.hpp core/src/config/vehicle_config.cpp config/vehicles/crew_dragon.toml config/README.md core/CMakeLists.txt core/tests/CMakeLists.txt core/tests/test_station_collision.cpp
git commit -m "feat(contact): hull collision of a visiting vehicle with the station primitives"
git push origin main
```

---

### Task 4: Docked hold and compound mass

**Files:**
- Create: `core/include/starsim/vehicle/compound_mass.hpp`, `core/src/vehicle/compound_mass.cpp`
- Modify: `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp`,
  `config/vehicles/iss.toml:26` (inertia re-derived), CMake lists
- Test: `core/tests/test_compound_mass.cpp`

**Interfaces:**
- Consumes: `vehicle::MassProperties`, `FlightBody`.
- Produces:
  - `vehicle::CompoundMass { double mass_kg; Vec3 cg_m; Mat3 inertia_kg_m2; }` and
    `CompoundMass combine_mass(double m1, const Vec3& cg1, const Mat3& inertia1, double m2, const Vec3& cg2, const
    Mat3& inertia2)` (all in one frame; inertia about each body's CG; result about the compound CG, parallel axis).
  - `FlightBody::hold_docked()` (not integrated in `step` / `coast`, no RCS / engines, `can_coast()` true),
    `FlightBody::release_docked()`, `bool FlightBody::docked() const`. The session sets its state.

- [ ] **Step 1: Write the failing tests**

```cpp
// ISS i5: compound mass of a docked pair and the docked hold of a body.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/vehicle/compound_mass.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Mat3;
using math::Vec3;

namespace {
Mat3 diag(double a, double b, double c) { return Mat3::from_columns({a, 0, 0}, {0, b, 0}, {0, 0, c}); }
}

TEST_CASE("Compound mass: sum, CG and parallel-axis inertia", "[iss][docking]") {
    const auto c = vehicle::combine_mass(4.0, {0, 0, 0}, diag(1, 1, 1), 1.0, {5, 0, 0}, diag(2, 2, 2));
    CHECK(c.mass_kg == 5.0);
    CHECK(c.cg_m.x == Catch::Approx(1.0));
    // about x: 1 + 2; about y and z: 1 + 4*1^2 + 2 + 1*4^2 = 23
    CHECK(c.inertia_kg_m2(0, 0) == Catch::Approx(3.0));
    CHECK(c.inertia_kg_m2(1, 1) == Catch::Approx(23.0));
    CHECK(c.inertia_kg_m2(2, 2) == Catch::Approx(23.0));
    CHECK(c.inertia_kg_m2(0, 1) == Catch::Approx(0.0));
}

TEST_CASE("Compound mass: an off-axis part gives products of inertia", "[iss][docking]") {
    const auto c = vehicle::combine_mass(1.0, {0, 0, 0}, diag(0, 0, 0), 1.0, {2, 2, 0}, diag(0, 0, 0));
    CHECK(c.inertia_kg_m2(0, 1) == Catch::Approx(-2.0));  // -sum m x y about the compound CG (1,1,0)
    CHECK(c.inertia_kg_m2(0, 1) == c.inertia_kg_m2(1, 0));
}

TEST_CASE("Docked hold: the body is not integrated and burns nothing", "[iss][docking]") {
    auto body = test::make_orbiting_body("crew_dragon");  // helper name: see test_support.hpp; any orbiting Dragon
    const auto before = body->state();
    const double prop = body->vehicle().rcs_propellant_kg();
    body->hold_docked();
    REQUIRE(body->docked());
    vehicle::VehicleCommand cmd;
    cmd.rcs_force_fraction = {1.0, 0.0, 0.0};
    body->step(test::environment(), test::epoch(), 0.005, cmd);
    CHECK(body->state().position_gcrf_m == before.position_gcrf_m);
    CHECK(body->vehicle().rcs_propellant_kg() == prop);
    CHECK(body->can_coast());
    body->release_docked();
    CHECK_FALSE(body->docked());
}
```

(Use the test-support helpers that already build an orbiting body and an environment - `grep -n "inline" core/tests/test_support.hpp`; do not add a second set.)

- [ ] **Step 2: Run to verify failure** (`-TestFilter "[docking]"`).

- [ ] **Step 3: Implement**

`combine_mass`: `m = m1 + m2`, `cg = (m1 cg1 + m2 cg2) / m`, `I = I1 + I2 + sum_i m_i (|d_i|^2 E - d_i d_i^T)` with
`d_i = cg_i - cg`.

`FlightBody`: `bool docked_ = false;` In `step()`: `if (docked_) { rcs_force_body_n_ = {}; return; }` before any
integration or propellant use (put it next to the existing `clamped_ || caught_ || landed_` branch, before
`update_rcs`); `coast()`: return immediately when docked; `can_coast()`: true when docked. Comment: "Docked (ISS i5):
held on a station by the session (sim/docking_sim) - not integrated, no thrust, no propellant use; the session sets
its state from the station every tick and after every coast."

`iss.toml` inertia: `inertia_kg_m2 = [1.51e8, 8.15e7, 2.17e8]   # [D] IDSS IDD E Table 3.3.1.2-1 IDSS-350T (1.15e8,
6.2e7, 1.65e8) scaled by the mass ratio 460.3 / 350 (i1 deferred minor)` - check the axis order of the existing line
(roll, pitch, yaw = x, y, z of the ISS body frame; the IDSS table uses X along the vehicle's longitudinal axis toward
the docking interface - for the ISS-class article that is the station's +X).

- [ ] **Step 4: Run** `-TestFilter "[docking]"` and `-TestFilter "[iss]"`: PASS.

- [ ] **Step 5: Commit and push** (`-Fast` first):

```bash
git add core/include/starsim/vehicle/compound_mass.hpp core/src/vehicle/compound_mass.cpp core/include/starsim/vehicle/flight_body.hpp core/src/vehicle/flight_body.cpp config/vehicles/iss.toml core/CMakeLists.txt core/tests/CMakeLists.txt core/tests/test_compound_mass.cpp
git commit -m "feat(vehicle): docked hold of a body and the compound mass of a docked pair"
git push origin main
```

---

### Task 5: Docking in the flight session

**Files:**
- Create: `core/include/starsim/sim/docking_sim.hpp`, `core/src/sim/docking_sim.cpp`, `core/src/sim/flight_session_docking.cpp`,
  `core/tests/docking_test_support.hpp`, `core/tests/test_docking_session.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (`advance_to`,
  `coast_allowance_s`, the body-clearing paths), `core/src/sim/flight_session_proximity.cpp` (contact plane, failed
  capture, collision), `core/include/starsim/gnc/proximity_guidance.hpp`, `core/src/gnc/proximity_guidance.cpp`
  (`capture_failed`), `core/tests/test_proximity_guidance.cpp`, CMake lists

**Interfaces:**
- Consumes: Tasks 1-4; `vehicle::port_kinematics`, `FlightBody::apply_external_wrench`, `FlightSession`'s
  `rendezvous_`, `proximity_`, `station_index_`, `controlled_`.
- Produces:
  - `gnc::ProximityGuidance::capture_failed(time::Epoch, const std::string& reason)`: from Contact -> Retreat to the
    last on-axis waypoint (Waypoint 2), status "Rückzug: Fangen fehlgeschlagen (<reason>)"; arriving there it waits
    (as after a commanded retreat; `resume()` flies the final approach again). `ProximityGuidance::collided(epoch,
    status)`: -> phase Contact stays, status "Kollision".
  - `sim::DockingSim` (`before_tick(chaser, station, epoch)`, `after_tick`, `phase()`, `status()`, `collision()`,
    `collision_impulse_ns()`, `wrench_balance_n()`): owns `contact::DockingMechanism`, the hull points, the station
    shape; applies the wrenches.
  - `FlightSession` getters: `std::optional<contact::DockingPhase> docking_phase() const`,
    `const std::string& docking_status() const`, `const std::vector<contact::DockingEvent>* docking_events() const`,
    `std::optional<contact::LoadPeaks> docking_loads() const`, `bool docked() const`,
    `std::optional<vehicle::CompoundMass> compound_mass() const`, `std::optional<double> collision_impulse_ns() const`.

Design:
- `start_proximity` creates the `DockingSim` when the chaser port is `Active` and the station port `Passive` (else
  German refusal "Andocksystem nicht kompatibel"); it sets the guidance's `contact_plane_m` to the petal mesh plane
  `ring_extension_m + 2 petal_height_m` (the plane where the chaser port is when the petals start to overlap).
- `advance_to` per tick: before the body steps, `docking_->before_tick(...)` computes the `DockingPose` (from
  `port_kinematics` of both ports, relative velocity and rate in the rotating passive port frame - the same
  arithmetic as `gnc::relative_port_state`; reuse it: `relative_port_state` gives position, velocity, rate; the chaser
  axes come from the two `axes_gcrf`), the mechanism wrench and (within `check_range_m`) the hull contact, and applies:
  chaser `apply_external_wrench(force_gcrf, torque_body)` with the torque `cross(point - cg, F) + couple`;
  station `apply_external_wrench(-force_gcrf, {})` (ideal attitude: torques dropped - documented). The applied
  forces sum to zero every tick (`wrench_balance_n()` for the test).
- After the tick: phase changes. `Failed` -> `proximity_->capture_failed(reason)`, re-engage
  `AutopilotState::Proximity`, `docking_->rearm()` when the chaser leaves the petal plane again outward. Hull contact
  (`any` with depth > 0) -> collision recorded once with the impulse integral (`sum |F| dt` while in contact),
  `proximity_->collided`, autopilot Off, status "Kollision" (the hull contact force still acts until free).
- Hard capture (`hard_captured()` first seen): **merge** - station velocity = `(m_s v_s + m_c v_c) / (m_s + m_c)`;
  station stage `extra_mass_kg += m_c`; chaser `hold_docked()` and its relative pose fixed at the **mated pose**
  (port on port, axes opposed: snap of < guide-pin tolerance - documented); `compound_mass_` from `combine_mass`
  (chaser inertia rotated into the station body frame); autopilot Off; proximity flight ends (status "angedockt");
  `attitude_mode_` cleared; translation stopped. Every tick and after every coast `sync_docked()` sets the chaser
  state from the station state, its ideal attitude and the fixed relative pose (velocity `v_s + omega x r`, angular
  velocity the station's).
- `coast_allowance_s`: 0 while the mechanism is in Contact / SoftCapture / Retraction / HooksClosing / HooksOpening or
  a hull contact exists; Docked: the chaser counts as coastable (`docked()` skips the angular-rate and running-group
  checks); the station coasts; `sync_docked()` after the coast.
- Clearing: `place_body` / `launch_from_pad` / `place_in_orbit` (all clear the bodies) reset `docking_`,
  `compound_mass_` and the station extra mass with the station; staging / separation / trunk jettison of the docked
  chaser: refused while docked ("angedockt") - `stage()` returns without action and sets `docking_status_`.
- Refusals while docked or capturing (German, the existing status strings of each command): `start_proximity`,
  `plan_rendezvous`, `plan_capsule_return`, `ignite`, `engage_*_autopilot`, `set_translation_command` -> "nicht
  möglich: angedockt" / "nicht möglich: Andockvorgang läuft".

- [ ] **Step 1: Write the failing tests**

`core/tests/docking_test_support.hpp` - extends `proximity_test::DragonNearIss` with a placement helper that puts the
Dragon on the port axis of `ida2` / `ida3` at a given axial distance, offset, axial / lateral speed and misalignment
in the passive port frame (state from `port_kinematics` of the station port; Dragon in the docking attitude rotated by
the misalignment; rate added), and a `fly_until(phase, limit_s)` loop in 1 s calls.

`test_proximity_guidance.cpp` (append):

```cpp
TEST_CASE("Proximity guidance: a failed capture retreats to Waypoint 2 and waits for resume", "[iss][proximity][docking]") {
    auto s = proximity_test::CwSim::ida2();
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    // fly from Waypoint 2 to contact (existing helper of this file), then report the failure
    REQUIRE(fly_to_contact(g, s, p));
    g.capture_failed(s.epoch, "abgeprallt");
    CHECK(g.phase() == gnc::ProximityPhase::Retreat);
    CHECK(g.status() == "Rückzug: Fangen fehlgeschlagen (abgeprallt)");
    run(g, s, 900.0);  // existing stepping helper
    CHECK(g.phase() == gnc::ProximityPhase::Waypoint);
    CHECK(g.path()[g.waypoint()].name == "Wegpunkt 2");
    CHECK(g.resume());
    run(g, s, 900.0);
    CHECK(g.phase() == gnc::ProximityPhase::Contact);
}
```

(Use the file's existing CW stepping helpers; if they are named differently, use those names.)

`test_docking_session.cpp`:

```cpp
// ISS i5: docking in the flight session - capture, merge, docked coast, failed capture, collision, refusals, clearing.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "docking_test_support.hpp"
#include "starsim/sim/flight_session.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("Docking session: a Dragon at rest in the rotating port frame stays at rest, no contact", "[iss][docking]") {
    // 0.5 m outside the petal mesh plane (0.30 + 2 x 0.20), zero velocity relative to the turning station port
    auto d = docking_test::DragonAtPort::place("ida2", 0.30 + 0.40 + 0.5, 0.0, 0.0, 0.0);
    d.session.advance_to(d.t0 + 10.0, 100000);
    CHECK(d.session.docking_phase() == contact::DockingPhase::Ready);
    const auto rel = d.session.relative_port_state();
    REQUIRE(rel);
    CHECK(norm(rel->velocity_mps) < 1e-4);  // the frame turns with the station (1.1e-3 rad/s)
    CHECK(std::abs(rel->position_m.x - 1.2) < 0.002);
}

TEST_CASE("Docking session: IDA-2 from 2 m out at 0.075 m/s captures, merges and docks", "[iss][docking]") {
    auto d = docking_test::DragonAtPort::place("ida2", 0.30 + 0.40 + 2.0, 0.03, -0.075, 0.0);
    const double ms = d.station().vehicle().mass_properties().mass_kg;
    const double mc = d.dragon().vehicle().mass_properties().mass_kg;
    REQUIRE(d.fly_until(contact::DockingPhase::SoftCapture, 60.0));
    // forces balance every tick
    CHECK(norm(d.session.docking_wrench_balance_n()) < 1e-6);
    REQUIRE(d.fly_until(contact::DockingPhase::Docked, 1200.0));
    CHECK(d.session.docked());
    CHECK(d.dragon().docked());
    // momentum through the merge tick: the pair's velocity is the momentum-weighted mean (merge record)
    const auto merge = d.session.docking_merge_record();
    CHECK(norm(merge.momentum_after - merge.momentum_before) < 1e-9 * norm(merge.momentum_before));
    CHECK(d.station().vehicle().mass_properties().mass_kg == Catch::Approx(ms + mc));
    REQUIRE(d.session.compound_mass());
    CHECK(d.session.compound_mass()->mass_kg == Catch::Approx(ms + mc));
}

TEST_CASE("Docking session: docked time warp keeps the Dragon on the port", "[iss][docking]") {
    auto d = docking_test::DragonAtPort::docked("ida3");
    d.session.advance_to(d.session.epoch() + 6.0 * 3600.0, 1000000);
    const auto rel = d.session.relative_port_state();
    REQUIRE(rel);
    CHECK(norm(rel->position_m) < 0.001);
    CHECK(norm(rel->velocity_mps) < 1e-4);
}

TEST_CASE("Docking session: commands refused while docked", "[iss][docking]") {
    auto d = docking_test::DragonAtPort::docked("ida2");
    auto& s = d.session;
    CHECK_FALSE(s.start_proximity("ida2"));
    CHECK(s.proximity_status() == "Anflug nicht möglich: angedockt");
    CHECK_FALSE(s.plan_rendezvous({}));
    CHECK_FALSE(s.plan_capsule_return());
    s.set_translation_command({1, 0, 0});
    s.advance_to(s.epoch() + 5.0, 10000);
    CHECK(norm(d.session.relative_port_state()->position_m) < 0.001);
}

TEST_CASE("Docking session: placing new bodies clears the docked pair", "[iss][docking]") {
    auto d = docking_test::DragonAtPort::docked("ida2");
    d.session.place_in_orbit("crew_dragon", d.some_orbit_state(), d.session.epoch());
    CHECK_FALSE(d.session.docked());
    CHECK_FALSE(d.session.docking_phase());
    CHECK_FALSE(d.session.station_index());
}

TEST_CASE("Docking session: a hull collision ends the flight with Kollision and the impulse", "[iss][docking]") {
    // beside Harmony, 1 m off its surface, drifting into it at 0.1 m/s (no proximity flight needed)
    auto d = docking_test::DragonAtPort::beside_module("us_modules", 1.0, 0.1);
    d.session.advance_to(d.t0 + 30.0, 100000);
    REQUIRE(d.session.collision_impulse_ns());
    CHECK(*d.session.collision_impulse_ns() > 0.0);
    CHECK(d.session.docking_status() == "Kollision");
}
```

(`docking_wrench_balance_n()` and `docking_merge_record()` are test-facing getters on the session: the last tick's
sum of the applied forces, and `{momentum_before, momentum_after}` of the merge tick. `DragonAtPort::docked(port)`
places at contact and flies to Docked in the fixture; `beside_module` places the Dragon with a session rendezvous
target so the docking sim runs - the collision check runs whenever a station exists and the chaser CG is within
`check_range_m`, with or without a proximity flight.)

- [ ] **Step 2: Run to verify failure** (`-TestFilter "[docking]"`): compile errors.

- [ ] **Step 3: Implement** the interfaces and design above. `DockingSim` in `docking_sim.hpp`:

```cpp
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "starsim/contact/docking_mechanism.hpp"
#include "starsim/contact/station_collision.hpp"
#include "starsim/time/epoch.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::sim {

// Docking of a visiting vehicle at a station port (ISS i5) on the session tick, in the pattern of CatchTowerSim:
// before_tick applies the mechanism wrench and the hull contact to both bodies (equal and opposite forces; the
// station's torque is dropped - it flies an ideal attitude), after_tick advances the mechanism's clock-driven steps.
class DockingSim {
public:
    DockingSim(std::string chaser_port, std::string station_port, const config::VehicleConfig& chaser_config,
               const config::VehicleConfig& station_config, contact::DockingMechanismParameters p = {});
    void before_tick(vehicle::FlightBody& chaser, vehicle::FlightBody& station, time::Epoch epoch, double dt_s);
    contact::DockingMechanism& mechanism() { return mechanism_; }
    const contact::DockingMechanism& mechanism() const { return mechanism_; }
    bool in_contact() const { return in_contact_; }  // interface or hull this tick
    bool collision() const { return collision_impulse_ns_.has_value(); }
    std::optional<double> collision_impulse_ns() const { return collision_impulse_ns_; }
    math::Vec3 wrench_balance_n() const { return balance_n_; }  // sum of the forces applied this tick (GCRF)
    const std::string& chaser_port() const { return chaser_port_; }
    const std::string& station_port() const { return station_port_; }

private:
    std::string chaser_port_, station_port_;
    contact::DockingMechanism mechanism_;
    contact::StationShape shape_;
    std::vector<math::Vec3> hull_body_m_;
    contact::StationCollisionParameters collision_p_;
    std::optional<double> collision_impulse_ns_;
    bool in_contact_ = false;
    math::Vec3 balance_n_;
    double t0_s_ = 0.0;
    bool started_ = false;
};

}  // namespace starsim::sim
```

The `DockingPose` from the session state:

```cpp
// in DockingSim::before_tick
const auto ck = vehicle::port_kinematics(chaser, chaser_port_);
const auto sk = vehicle::port_kinematics(station, station_port_);
const auto rel = gnc::relative_port_state(ck, sk);  // NB: sim may use gnc
contact::DockingPose pose{rel.position_m, rel.velocity_mps, transpose(sk.axes_gcrf) * ck.axes_gcrf,
                          rel.angular_rate_radps};
```

(`relative_port_state`'s frame and sign conventions are those of `DockingPose`; Task 5's resting test pins it.)
Mechanism wrench -> GCRF: `F = sk.axes_gcrf * w.force_port_n`, application point `sk.position_gcrf_m +
sk.axes_gcrf * w.point_port_m`, lever from the chaser CG; chaser torque in body axes:
`transpose(R_c) * (cross(point - cg, F) + sk.axes_gcrf * w.torque_port_nm)`. The chaser's mass and its joint inertia
(`max principal + m |port lever|^2`) come from its `mass_properties()`.

Hull contact: transform the hull points (body frame -> GCRF -> station body frame), their velocities relative to the
station (in the station body frame, including both rotations), call `hull_contact`, transform back.

- [ ] **Step 4: Run** `-TestFilter "[docking]"`, `-TestFilter "[proximity]"`: PASS. Record the soft->hard time, the
  merge momentum residual and the peak loads of the session capture in the ledger.

- [ ] **Step 5: Commit and push** - this task touches a flight session: the **full suite** first
  (`tools/build.ps1 -NoBindings`), then:

```bash
git add core/include/starsim/sim/docking_sim.hpp core/src/sim/docking_sim.cpp core/src/sim/flight_session_docking.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/src/sim/flight_session_proximity.cpp core/include/starsim/gnc/proximity_guidance.hpp core/src/gnc/proximity_guidance.cpp core/CMakeLists.txt core/tests/CMakeLists.txt core/tests/docking_test_support.hpp core/tests/test_docking_session.cpp core/tests/test_proximity_guidance.cpp
git commit -m "feat(sim): docking in the flight session - capture, merge into the compound, collision, failed capture retreat"
git push origin main
```

---

### Task 6: Undocking and departure

**Files:**
- Create: `core/include/starsim/gnc/departure_guidance.hpp`, `core/src/gnc/departure_guidance.cpp`,
  `core/tests/test_departure_guidance.cpp`
- Modify: `core/include/starsim/gnc/proximity_geometry.hpp` (`BreakoutParameters`), `core/src/gnc/proximity_transfer.cpp`
  (ellipsoid exit), `core/include/starsim/gnc/proximity_transfer.hpp`, `core/include/starsim/sim/flight_session.hpp`,
  `core/src/sim/flight_session_docking.cpp`, `core/src/sim/flight_session.cpp` (autopilot dispatch, coast),
  `core/tests/test_proximity_transfer.cpp`, `core/tests/test_docking_session.cpp`, CMake lists

**Interfaces:**
- Consumes: `gnc::ProximityInput` (same input as the proximity guidance), `gnc::breakout_delta_v`,
  `gnc::TranslationLoop`, `contact::DockingMechanism::undock / separation_due`, `separation_relative_speed_mps`.
- Produces:
  - `BreakoutParameters` gains `std::optional<math::Vec3> ellipsoid_semi_axes_m` and `double ellipsoid_exit_limit_s
    = 3600.0` [E: Demo-1 left the AE 17 min after undocking]; `Breakout` gains `double ellipsoid_exit_s`.
    `breakout_delta_v` additionally requires the AE exit within the limit when the semi-axes are set (CW,
    `(x/a)^2 + (y/b)^2 + (z/c)^2 > 1`).
  - `gnc::DeparturePhase { UndockBurn, BackOut, DepartureBurn, Drift, Complete }` (German "Abkoppelimpuls",
    "Rückzug vom Port", "Abflugmanöver", "Abflug", "Abflug abgeschlossen"), `gnc::DepartureParameters`,
    `gnc::DepartureGuidance(DepartureParameters, ProximityParameters)` with `ProximityOutput update(const
    ProximityInput&)`, `phase()`, `status()`, `events()`, `departure_burn()` (the planned `Breakout`).
  - `FlightSession::undock()` (false + German `docking_status()` when not docked / already undocking),
    `std::optional<gnc::DeparturePhase> departure_phase() const`, `AutopilotState::Departure` (German "Abflug" in
    `to_string`).

Design:
- `undock()`: only when docked and the autopilot is Off / Orbit -> `mechanism.undock` (hooks open, 180 s). On
  `separation_due()`: un-merge - chaser `release_docked()`, station `extra_mass_kg -= m_c`, both velocities from the
  compound velocity plus the spring split `dv_c = +v_rel m_s / (m_s + m_c)` along the station port axis (outward),
  `dv_s = -v_rel m_c / (m_s + m_c)` (momentum conserved); the chaser keeps the station's angular velocity;
  `compound_mass_` cleared; autopilot `Departure` with the docking attitude hold (rendezvous target still set).
- `DepartureGuidance`: UndockBurn - axial translation outward until the opening speed reaches
  `undock_speed_mps` 0.15 [S: Crew-4 / Demo-1 undock burn 1]; BackOut - axial law outward at that speed, lateral
  convergence onto the axis, until the port range exceeds `burn_range_m` [E 10 m: clear of the petals and the PMA];
  DepartureBurn - `breakout_delta_v` with `ellipsoid_semi_axes_m` = (2000, 1000, 1000) [S: NASA, the AE 4 x 2 x 2 km,
  = `mission::approach_ellipsoid_semi_axes_m`] flown with the LVLH translation loop until the velocity error < 0.01
  m/s; Drift - attitude hold off (as after a breakout: the session may coast), until the CG leaves the ellipsoid ->
  Complete ("Abflug abgeschlossen"; autopilot Off, rendezvous target cleared, nose cone left open -
  `plan_capsule_return` closes it). An unsafe burn (none qualifies) flies the best effort and says
  "Abflug: nicht passiv sicher" in the status.
- Propellant: the departure reports its RCS use in the event text.

- [ ] **Step 1: Write the failing tests**

`test_proximity_transfer.cpp` (append):

```cpp
TEST_CASE("Breakout with the approach-ellipsoid exit: departure from IDA-2", "[iss][proximity][docking]") {
    gnc::ProximityParameters p;
    p.breakout.ellipsoid_semi_axes_m = Vec3{2000.0, 1000.0, 1000.0};
    // 10 m in front of IDA-2 (station CG to port 35 m on V-bar), opening at 0.15 m/s along +x
    const orbit::RelativeState from{{45.0, 0.0, 0.0}, {0.15, 0.0, 0.0}};
    const auto b = gnc::breakout_delta_v(from, 1.128e-3, p);
    REQUIRE(b.safe);
    CHECK(b.ellipsoid_exit_s > 0.0);
    CHECK(b.ellipsoid_exit_s <= p.breakout.ellipsoid_exit_limit_s);
    CHECK(b.min_range_after_exit_m >= p.keep_out_radius_m + p.kos_margin_m);
}

TEST_CASE("The departure ellipsoid equals the mission's approach ellipsoid", "[iss][docking]") {
    const gnc::DepartureParameters d;
    CHECK(d.ellipsoid_semi_axes_m.x == mission::approach_ellipsoid_semi_axes_m[0]);
    CHECK(d.ellipsoid_semi_axes_m.y == mission::approach_ellipsoid_semi_axes_m[1]);
    CHECK(d.ellipsoid_semi_axes_m.z == mission::approach_ellipsoid_semi_axes_m[2]);
}
```

`test_departure_guidance.cpp` (CW point model of `proximity_test_support.hpp`, start at the port with the separation
speed):

```cpp
TEST_CASE("Departure guidance: IDA-2 - undock burn, back-out, departure burn, AE exit within the hour",
          "[iss][docking]") {
    auto s = proximity_test::CwSim::ida2();
    s.cg.position_m = s.port_lever_lvlh + Vec3{4.0, 0, 0};  // chaser CG 4 m out (port at the station port)
    s.cg.velocity_mps = {0.085, 0.0, 0.0};                    // separation springs
    gnc::DepartureGuidance g({}, {});
    double t = 0.0, ks = -1.0, ae = -1.0;
    for (; t < 7200.0; t += 0.05) {
        const auto in = s.input();
        const auto out = g.update(in);
        s.step(out.translation_fraction_port, 0.05);
        const double r = norm(s.cg.position_m);
        if (ks < 0.0 && r > 200.0) ks = t;
        if (g.phase() == gnc::DeparturePhase::Complete) { ae = t; break; }
    }
    REQUIRE(ae > 0.0);
    CHECK(ks < 900.0);    // Demo-1: 6 min 48 s
    CHECK(ae < 3600.0);   // Demo-1: 17 min
    CHECK(s.propellant_kg > 2000.0 - 60.0);
    CHECK(g.events().front().status == "Abkoppelimpuls");
}

TEST_CASE("Departure guidance: IDA-3 (R-bar) leaves passively safe", "[iss][docking]") {
    auto s = proximity_test::CwSim::ida3();
    s.cg.position_m = s.port_lever_lvlh + Vec3{0, 0, -4.0};
    s.cg.velocity_mps = {0.0, 0.0, -0.085};
    gnc::DepartureGuidance g({}, {});
    for (double t = 0.0; t < 7200.0 && g.phase() != gnc::DeparturePhase::Complete; t += 0.05) {
        s.step(g.update(s.input()).translation_fraction_port, 0.05);
    }
    CHECK(g.phase() == gnc::DeparturePhase::Complete);
    REQUIRE(g.departure_burn());
    CHECK(g.departure_burn()->safe);
}
```

`test_docking_session.cpp` (append):

```cpp
TEST_CASE("Undocking: hooks open, springs separate at 0.085 m/s, momentum conserved, departure completes",
          "[iss][docking]") {
    auto d = docking_test::DragonAtPort::docked("ida2");
    auto& s = d.session;
    REQUIRE(s.docked());
    REQUIRE(s.undock());
    CHECK_FALSE(s.undock());
    CHECK(s.docking_status() == "Abkoppeln nicht möglich: Abkoppeln läuft");
    REQUIRE(d.fly_until(contact::DockingPhase::Separated, 400.0));
    const auto split = s.docking_separation_record();
    CHECK(split.relative_speed_mps == Catch::Approx(0.085).margin(0.003));
    CHECK(norm(split.momentum_after - split.momentum_before) < 1e-6 * norm(split.momentum_before));
    CHECK_FALSE(d.dragon().docked());
    double t = s.epoch() - d.t0;
    while (s.departure_phase() != gnc::DeparturePhase::Complete && t < 3.0 * 3600.0) {
        s.advance_to(d.t0 + (t += 10.0), 1000000);
    }
    CHECK(s.departure_phase() == gnc::DeparturePhase::Complete);
    CHECK(s.autopilot_state() == sim::AutopilotState::Off);
}
```

(`docking_separation_record()`: `{relative_speed_mps, momentum_before, momentum_after}` of the separation tick.)

- [ ] **Step 2: Run to verify failure** - `-TestFilter "[docking]"`.

- [ ] **Step 3: Implement.** `DepartureParameters`:

```cpp
struct DepartureParameters {
    double undock_speed_mps = 0.15;   // [S: NASASpaceflight Crew-4 / Demo-1 "undock burn 1 ... 0.15 m/s"]
    double burn_range_m = 10.0;       // [E] departure burn 0 once the port is this far out (clear of petals and PMA)
    math::Vec3 ellipsoid_semi_axes_m{2000.0, 1000.0, 1000.0};  // [S: NASA, approach ellipsoid 4 x 2 x 2 km]
    double ellipsoid_exit_limit_s = 3600.0;  // [E] Demo-1 left the AE 17 min after undocking
    double burn_tolerance_mps = 0.01;  // [E] departure burn flown to this velocity error
};
```

`breakout_delta_v`: inside `check_breakout_arc`, when `p.breakout.ellipsoid_semi_axes_m` is set, record the first
sample with metric > 1 and require it <= `ellipsoid_exit_limit_s`. The departure guidance copies its semi-axes and
limit into its `ProximityParameters::breakout`.

- [ ] **Step 4: Run** `-TestFilter "[docking]"`, `-TestFilter "[proximity]"`: PASS (i4 breakouts unchanged without the
  semi-axes).

- [ ] **Step 5: Commit and push** - full suite first, then:

```bash
git add core/include/starsim/gnc/departure_guidance.hpp core/src/gnc/departure_guidance.cpp core/include/starsim/gnc/proximity_geometry.hpp core/include/starsim/gnc/proximity_transfer.hpp core/src/gnc/proximity_transfer.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session_docking.cpp core/src/sim/flight_session.cpp core/CMakeLists.txt core/tests/CMakeLists.txt core/tests/test_departure_guidance.cpp core/tests/test_proximity_transfer.cpp core/tests/test_docking_session.cpp
git commit -m "feat(sim): undocking with the IDSS separation springs and the departure out of the approach ellipsoid"
git push origin main
```

---

### Task 7: Validation campaigns

**Files:**
- Modify: `core/tests/test_docking_session.cpp`, `core/tests/docking_test_support.hpp`
- Possibly: `core/src/sim/flight_session_docking.cpp` (`STARSIM_DEBUG_DOCKING` diagnostics)

**Interfaces:** consumes everything above; produces the numbers for `docs/subsystems.md`.

- [ ] **Step 1: Write the campaigns** (all `[iss][docking]`; `[slow]` where a case runs > ~40 s - measure):

1. **Full chain IDA-2 and IDA-3 from Waypoint 0** (i4 fixture start, `start_proximity`) to Docked: contact inside
   the envelope (the i4 record), soft capture within 30 s of the first contact, Docked 9-13 min after soft capture,
   peak compression < 6,500 N, shear < 3,200 N, the station mass = sum, the Dragon 0 mm / 0 mm/s on the port after
   `advance_to(+1 h)`.
2. **IDSS envelope grid in the session** (both ports): placed 1 m outside the petal mesh plane with every combination
   of axial {0.05, 0.10}, offset {0, 0.10 m}, misalignment {0, 4 deg}, roll {0, 4 deg}, lateral {0, 0.04}, pitch rate
   {0, 0.2 deg/s} - a fractional factorial of at least 16 cases per port (corner points first): all reach SoftCapture
   and Docked; loads within the IDSS limits (warnings allowed only for tension/shear/bending/torsion - record any).
3. **Failure cases:** 0.25 m/s -> Failed "Überlast", the proximity guidance retreats to Waypoint 2 and waits;
   offset 0.30 m -> "Versatz zu groß" -> retreat; misalignment 10 deg -> "Winkel zu groß" -> retreat; after the 0.30 m
   failure `proximity_resume()` -> a second final approach -> Docked (Review Focus 5).
4. **Collision:** a proximity flight with a commanded lateral drift into Harmony from Waypoint 2 (translation
   command overriding the guidance after `proximity_hold`, or a placed drift) -> "Kollision", impulse > 0, the
   autopilot off.
5. **Momentum:** over the soft-capture phase the sum of the applied forces is zero every tick (`docking_wrench_balance_n`
   sampled every second); merge and separation records conserve momentum to 1e-9 relative.
6. **Undock to splashdown `[slow]`:** docked at IDA-2 -> `undock()` -> departure Complete -> `plan_capsule_return()`
   -> splashdown ("gewassert" of the d3/d4 return); RCS propellant left > 0 after the deorbit; the departure's RCS
   use < 30 kg [E check].
7. **Docked coast:** docked 24 h with the OEM station impulses (a reboost inside the window): the pair stays rigid
   (port error < 1 mm), the reboost changes the compound velocity by the impulse (the station's OEM Delta-v is the
   compound's - documented: the OEM's reboost Delta-v is applied to the pair unchanged).

- [ ] **Step 2: Run** `-TestFilter "[docking]"` (then the `[slow]` case alone). Expected: PASS; record every number
  (time, propellant, contact state, peak loads, separation speed, departure times KOS/AE, splashdown point) in the
  ledger. A case outside the envelope that captures, or an envelope case that fails, is a finding: fix the model's
  [E] values (Tasks 1-2) and re-run; do not loosen [S] bounds.

- [ ] **Step 3: Diagnostics.** `STARSIM_DEBUG_DOCKING=1`: every mechanism event, and every second while in contact or
  capture the pose (axial, offset, angles), the wrench and the load peaks (pattern of `STARSIM_DEBUG_PROXIMITY`).

- [ ] **Step 4: Commit and push** - full suite (with `[slow]`) first, then:

```bash
git add core/tests/test_docking_session.cpp core/tests/docking_test_support.hpp core/src/sim/flight_session_docking.cpp
git commit -m "test(sim): docking campaigns - full chains, IDSS envelope grid, failures, collision, undock to splashdown"
git push origin main
```

---

### Task 8: Documentation, spec corrections, final review

**Files:**
- Modify: `docs/subsystems.md` (new section "Docking mechanism, undocking and departure (i5)" after "Proximity
  operations (i4)", and the i4 section's "Open / for i5-i6" line), `docs/superpowers/specs/2026-10-19-iss-docking-design.md`
  (status line: i5 implemented; the seven corrections under "i5 design"), `CLAUDE.md` (module list: contact - docking
  interface and mechanism, station hull collision; vehicle - docked hold, compound mass; gnc - departure guidance;
  sim - docking in the flight session), `config/README.md` (port `soft_capture`, `hull_top_diameter_m`).

- [ ] **Step 1: Write the docs** - sources (IDSS IDD E tables and sections, Demo-2, Crew-4 / Demo-1), the model
  (reduced-order petal cone, ring tilt conformance, joint, stroke and overload, latch condition, timeline), every
  [E] value with its reason, the campaign table (case / soft->hard time / peak compression / shear / separation speed /
  departure KOS and AE times / propellant), the simplifications (station torque dropped, Dragon drag area not added,
  snap at the merge < guide-pin tolerance, below 0.05 m/s still captures, departure burns 1-3 not modelled, OEM reboost
  applied to the pair), `STARSIM_DEBUG_DOCKING`, open points for i6 (missions `crew_dragon_iss`,
  `crew_dragon_iss_departure`, the getters `docking_state` / `station_geometry`, the `undock` command id).

- [ ] **Step 2: Full suite + smoke test**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` (with bindings) and
`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd`
Expected: all green, smoke ok.

- [ ] **Step 3: Commit and push**

```bash
git add docs/subsystems.md docs/superpowers/specs/2026-10-19-iss-docking-design.md CLAUDE.md config/README.md
git commit -m "docs: ISS i5 docking mechanism, undocking and departure"
git push origin main
```

- [ ] **Step 4: Final review** - one fresh reviewer on the most capable model over the whole i5 range
  (`git diff <i5 plan commit>..HEAD`), with the spec, this plan and the ledger; fix pass for its findings (full suite,
  push); deferred minors into the ledger and the memory. Then record the state and ask the user for `/clear`
  (next: writing-plans i6).
