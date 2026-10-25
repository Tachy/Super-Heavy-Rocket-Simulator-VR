# ISS i4: Proximity operations and final approach - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** From Waypoint 0 (400 m below the ISS, where i3 ends) the Crew Dragon flies the real waypoint pattern on its
Draco translation - Waypoint 1 on the port axis outside the keep-out sphere, Waypoint 2 20 m out, the final approach
to contact - for IDA-2 (V-bar) and IDA-3 (R-bar, with a fly-around), inside an approach corridor, with automatic and
commanded hold / retreat / abort (breakout onto a passively safe trajectory), and arrives at the port plane inside
the capture envelope at < 0.1 m/s.

**Architecture:** Four `gnc` building blocks without any session dependency, then one session step. 
`gnc/proximity_geometry` holds the parameters, the waypoint path per port, the corridor test, the docking attitude
and the capture envelope. `gnc/proximity_transfer` plans Clohessy-Wiltshire transfers between waypoints with a
keep-out-sphere clearance and picks the breakout delta-v. `gnc/translation_loop` turns a desired relative velocity
into translation fractions with a hysteresis deadband (Draco pulse economy) and holds the approach laws (axial speed
schedule, lateral convergence, position hold). `gnc/proximity_guidance` is the state machine (phases with German
names, waypoint holds, monitors, commands). `FlightSession::start_proximity` (`flight_session_proximity.cpp`) feeds
it the relative states every tick, holds the docking attitude (an LVLH hold refreshed each tick) and flies its output
through the i2 translation command.

**Tech Stack:** C++20, Catch2, CSPICE only through the `spice` adapters (tests).

**Spec:** `docs/superpowers/specs/2026-10-19-iss-docking-design.md` (sections "Intent", "i4 design: proximity
operations and final approach", "i5 design" for the envelope values, "Error handling", "Testing").

## Research done while planning (2026-10-19) and the spec corrections it implies

Real Crew Dragon pattern:
- Crew-2 / Demo-1 timeline (Spaceflight Now, Reddit launch-thread mirror): Waypoint 0 04:25 (400 m below), go to
  enter the keep-out sphere, **Waypoint 1 04:49, 220 m in front of the ISS**, Waypoint 2 05:00 (**20 m**), contact and
  capture at IDA-2 05:08-05:10. Crew-5 (Spaceflight Now, 2022-10-05): Waypoint 1 16:36, Waypoint 2 16:47, a hold and a
  docking go/no-go at Waypoint 2. Other accounts give 150-200 m for Waypoint 1; 220 m is the one in the timelines.
- Demo-2 (NASA blog / press, 2020-05-31): the final approach "at 0.98 ft/s" (**0.30 m/s**), slowed to "a little less
  than 0.33 ft/s" (**0.10 m/s**) before soft capture, nose cone open, V-bar approach.
- Zenith port: Crew-4 (NASASpaceflight 2022-04): Waypoint 0 400 m below, Waypoint 1, Waypoint 2 at 20 m, then the
  Dragon "oriented itself towards its docking port on the zenith side"; the Crew-2 port relocation (Spaceflight Now
  2021-07-21) "moved to an approach corridor above the complex" in a **fly-around** in autopilot, then an autonomous
  final approach once aligned with the zenith port. The fly-around geometry is not published: [E].

Derived [D]: Waypoint 0 -> Waypoint 1 takes 24 min including a hold; with a 5 min hold [E] the transfer is **19 min
(1140 s)**. Waypoint 1 -> Waypoint 2 (200 m in 11 min incl. a hold) fits a range-scheduled axial approach of
0.3 m/s far out and 0.1 m/s near Waypoint 2 (range / 600 s between the limits). Waypoint 2 -> contact: 20 m at
0.10 -> 0.075 m/s ~ 230 s, plus the 5 min hold = the 8-10 min of the timelines.

Spec corrections (to be confirmed by the user with this plan; Task 7 writes them into the spec):
1. **Transfers outside the keep-out sphere are CW two-impulse transfers re-solved every tick from the live relative
   state** (closed loop: the solved velocity is flown with a deadband, so the "multi-impulse" corrections fall out of
   the loop), **not targeted on the full model.** Below ~1 km the full model (J2, differential drag 1e-7 m/s^2) and CW
   differ by centimetres over a 19 min transfer (Task 2 pins the number), well inside the loop's deadband; a
   full-model targeter there would cost a coast per tick for nothing.
2. **IDA-3 (R-bar) gets a fly-around:** Waypoint 0 -> a point 250 m [E] in front of the station on V-bar -> an arc at
   250 m about the station's CG in 30 deg [E] legs up to Waypoint 1, 220 m above IDA-3 on its axis -> Waypoint 2 ->
   contact. IDA-2: Waypoint 0 -> Waypoint 1 (220 m in front on its axis) directly.
3. **Inside the keep-out sphere the motion is a range-scheduled approach along the port axis** (0.30 m/s far,
   0.10 m/s at Waypoint 2, 0.10 -> 0.075 m/s in the final approach) with lateral convergence onto the axis - also
   between Waypoint 1 and Waypoint 2 (the spec's "multi-impulse" applies outside).
4. **i4 ends at "Kontakt"**: the crossing of the contact plane (the target port plane, `contact_plane_m` = 0 [E]; i5
   sets the soft-capture ring's reach) is recorded with the relative port state; there is no contact physics until i5
   (the Dragon would drift on). The capture envelope (axial 0.05-0.10 m/s, lateral <= 0.04 m/s, offset <= 0.10 m,
   angle <= 4 deg, rate <= 0.2 deg/s) is defined here as [E] guide values; i5 verifies them against the IDSS IDD.
5. **Contact speed target 0.075 m/s** [E] (middle of the axial 0.05-0.10 band), below the M9 criterion 0.1 m/s.

## Global Constraints

- `core/` has no Godot dependency; modules depend downwards only: `gnc` may use `orbit`, `vehicle`, `frames`, `math`,
  `time`; nothing in `gnc` includes `mission` or `sim`.
- SI units internally, angles in radians; names carry frame and unit (`position_port_m`, `velocity_lvlh_mps`).
- No magic numbers: every tuned value is a `ProximityParameters` (or nested parameters) field with an `[S]` / `[E]` /
  `[D]` tag and its source in the comment.
- LVLH convention of `frames/lvlh.hpp` and `orbit/clohessy_wiltshire.hpp`: x V-bar (along the velocity), y H-bar
  (-orbit normal), z R-bar (nadir). Waypoint 0 is LVLH (0, 0, +400) relative to the station's CG.
- Port frame of `vehicle/docking_port.hpp`: x out of the port (docking axis), z the clocking reference `up`,
  y = z x x. The chaser sits at x > 0 in the target port frame while approaching.
- UI / status strings German (UTF-8 sources), code and comments English.
- Physics never in GDScript; no bindings in this part (i6 builds the game interface).
- Tests tagged `[iss]`, `[rendezvous]`, plus `[proximity]`; `[slow]` above ~40 s; ctest runs test cases in parallel
  (no shared temp files).
- During a task run only the touched tags (`-TestFilter "[proximity]"`); before every push `-Fast`; Task 6 touches a
  flight session: the full suite before its push and before the final review.
- Commit on `main`, Conventional Commits, push after every task (`git push origin main`), attribution lines from the
  session.

## Review Focus

1. **Chaining from i3's Waypoint 0:** `start_proximity` right after `plan_rendezvous` reached `Waypoint0` (autopilot
   Off, the Dragon in whatever attitude the braking burn left, up to 180 deg off the docking attitude, drifting
   ~0.02 m/s). Expected: the Dragon turns into the docking attitude during the Waypoint 0 hold before it translates
   anywhere, and the flight completes. Test in Task 6 (`From the i3 Waypoint 0 state with the braking attitude`).
2. **A station reboost during proximity** (an OEM impulse inside the span): expected a German warning at the start
   (as `rendezvous_warnings`), and a transfer that re-solves after the jump and still arrives at its waypoint.
   Test in Task 6 (synthetic impulse of 0.3 m/s during the Waypoint 0 -> 1 transfer).
3. **Commands in the wrong state:** `proximity_hold()` without a flight, `proximity_resume()` while nothing is held,
   `proximity_abort()` twice, `proximity_retreat()` at Waypoint 0 (nothing behind it). Expected: false with a German
   status, no state change. Tests in Task 4 (guidance) and Task 6 (session without a flight).
4. **One big `advance_to` during proximity** (time warp from the game): the session must keep ticking (no coast
   across a waypoint hold or the final approach) and still hold the waypoint; after a completed breakout it may coast.
   Test in Task 6 (`advance_to(+900 s)` in one call at the Waypoint 1 hold).
5. **Bodies placed or the controlled body changed mid-flight** (`place_in_orbit`, `stage`): the proximity flight is
   cleared with the rendezvous target, no dangling guidance writes a translation command. Test in Task 6.

---

## File structure

| File | Responsibility |
|---|---|
| `core/include/starsim/gnc/proximity_geometry.hpp`, `core/src/gnc/proximity_geometry.cpp` | `ProximityParameters`, `ProximityWaypoint` path per approach, `corridor_status`, `docking_attitude_gcrf`, `CaptureEnvelope` + check |
| `core/include/starsim/gnc/proximity_transfer.hpp`, `core/src/gnc/proximity_transfer.cpp` | CW transfer with keep-out clearance (`plan_proximity_transfer`), `breakout_delta_v` |
| `core/include/starsim/gnc/translation_loop.hpp`, `core/src/gnc/translation_loop.cpp` | `TranslationLoop` (hysteresis deadband), `axial_closing_speed_mps`, `approach_velocity_port`, `hold_velocity` |
| `core/include/starsim/gnc/proximity_guidance.hpp`, `core/src/gnc/proximity_guidance.cpp` | `ProximityGuidance` state machine, `ProximityPhase`, events, contact record, commands |
| `core/include/starsim/sim/flight_session.hpp` (modify), `core/src/sim/flight_session_proximity.cpp` (new), `core/src/sim/flight_session.cpp` (modify) | session API, per-tick wiring, docking attitude, clearing |
| `core/tests/test_proximity_geometry.cpp`, `test_proximity_transfer.cpp`, `test_translation_loop.cpp`, `test_proximity_guidance.cpp`, `test_proximity_session.cpp`, `proximity_test_support.hpp` | tests |
| `core/CMakeLists.txt`, `core/tests/CMakeLists.txt` (modify) | new sources |
| `docs/subsystems.md`, `CLAUDE.md`, spec (modify) | Task 7 |

Ledger for execution: `.superpowers/sdd/2026-10-19-iss-i4-proximity/progress.md` (create in Task 1, append rulings and
numbers per task, as in i3).

---

### Task 1: Proximity geometry - parameters, waypoints, corridor, docking attitude, envelope

**Files:**
- Create: `core/include/starsim/gnc/proximity_geometry.hpp`, `core/src/gnc/proximity_geometry.cpp`
- Create: `core/tests/test_proximity_geometry.cpp`
- Modify: `core/CMakeLists.txt` (add `src/gnc/proximity_geometry.cpp` next to the other `src/gnc/` entries),
  `core/tests/CMakeLists.txt` (add `test_proximity_geometry.cpp`)
- Create: `.superpowers/sdd/2026-10-19-iss-i4-proximity/progress.md`

**Interfaces:**
- Consumes: `gnc::RelativePortState` (`gnc/relative_navigation.hpp`), `math::Mat3`, `math::Quat::from_matrix`,
  `mission::keep_out_sphere_radius_m` is in `mission` (gnc must not include it): define the KOS radius again as a
  `ProximityParameters` field with the same [S] tag and a test that keeps both equal (the test may include `mission`).
- Produces (used by Tasks 2-6, exact names):

```cpp
namespace starsim::gnc {

enum class DockingApproach { VBar, RBar };  // IDA-2 forward / IDA-3 zenith
// From the target port axis in the station LVLH frame: dominant +x -> VBar, dominant -z -> RBar; otherwise
// std::invalid_argument (German: "Kein Anflugweg für diesen Port").
DockingApproach docking_approach(const math::Vec3& port_axis_lvlh);

enum class WaypointFrame { LvlhFromCg, TargetPort };  // CG-relative in station LVLH / chaser port in target port frame
struct ProximityWaypoint {
    std::string name;          // German: "Wegpunkt 0", "Umflug 1", "Wegpunkt 1", "Wegpunkt 2"
    WaypointFrame frame;
    math::Vec3 position_m;
    double hold_s = 0.0;       // go/no-go hold after arrival (0: fly on)
    bool on_axis = false;      // TargetPort waypoints on the docking axis: reached by the axial approach
};

struct TranslationLoopParameters {
    double gain_time_s = 1.0;        // [E] fraction = error / (acceleration * gain_time)
    double start_error_mps = 0.02;   // [E] a pulse train starts above this velocity error
    double stop_error_mps = 0.005;   // [E] ... and stops below this
};

struct BreakoutParameters {
    std::vector<double> magnitudes_mps{0.3, 0.5, 1.0, 1.5};  // [E] tried in order, the smallest safe one wins
    double horizon_s = 86400.0;      // [E] passively safe for 24 h (ISS visiting-vehicle practice)
    double sample_s = 30.0;          // [E] CW sampling of the safety check
    double exit_limit_s = 1800.0;    // [E] the keep-out sphere must be left within this
};

struct ProximityParameters {
    double keep_out_radius_m = 200.0;      // [S: NASA] keep-out sphere about the station CG (= mission::keep_out_sphere_radius_m)
    double kos_margin_m = 10.0;            // [E] transfers outside the sphere keep this clearance
    double waypoint0_below_m = 400.0;      // [S: Crew-2 timeline] (i3 ends here)
    double waypoint1_range_m = 220.0;      // [S: Crew-2 / Demo-1 / Crew-5 timelines] on the port axis
    double waypoint2_range_m = 20.0;       // [S: same]
    double waypoint0_hold_s = 300.0;       // [E] go/no-go holds
    double waypoint1_hold_s = 300.0;       // [E]
    double waypoint2_hold_s = 300.0;       // [E] Crew-5: hold + docking go/no-go at Waypoint 2
    double transfer_s = 1140.0;            // [D] Crew-2 Waypoint 0 04:25 -> Waypoint 1 04:49 minus the 5 min hold
    double flyaround_radius_m = 250.0;     // [E] IDA-3 fly-around about the station CG
    double flyaround_step_rad = 30.0 * constants::deg_to_rad;  // [E] legs of the arc
    double flyaround_leg_s = 300.0;        // [E]
    double transfer_final_s = 60.0;        // [E] the CW re-solve stops this long before arrival; the hold law brakes
    double arrival_tolerance_m = 2.0;      // [E] LVLH waypoints
    double axis_arrival_tolerance_m = 0.3; // [E] on-axis waypoints
    double arrival_speed_mps = 0.01;       // [E]
    double corridor_half_angle_rad = 10.0 * constants::deg_to_rad;  // [E] spec: cone about the port axis inside the sphere
    double corridor_min_half_width_m = 0.3;  // [E] the cone's lateral bound never drops below this near the port
    double corridor_retreat_factor = 2.0;  // [E] beyond this multiple of the bound: retreat instead of hold
    double approach_speed_far_mps = 0.30;  // [S: Demo-2 0.98 ft/s]
    double approach_time_constant_s = 600.0;  // [E] axial speed = range / this between the limits
    double final_speed_start_mps = 0.10;   // [S: Demo-2 0.33 ft/s before soft capture] at Waypoint 2
    double contact_speed_mps = 0.075;      // [E] middle of the axial capture band 0.05-0.10
    double contact_plane_m = 0.0;          // [E] contact when the chaser port reaches this axial distance (i5: ring reach)
    double lateral_gain_per_s = 0.02;      // [E] lateral velocity = -offset * gain
    double lateral_speed_max_mps = 0.03;   // [E] below the lateral capture bound 0.04
    double hold_time_constant_s = 60.0;    // [E] position hold: velocity = -error / this
    double hold_speed_max_mps = 0.05;      // [E]
    double retreat_speed_mps = 0.10;       // [E] outward along the axis
    double overspeed_factor = 1.5;         // [E] closing faster than factor * schedule + margin: hold
    double overspeed_margin_mps = 0.02;    // [E]
    double deorbit_reserve_kg = 600.0;     // [E] d3: 598 kg deorbit from 420 km; below it: abort
    double max_start_range_m = 5000.0;     // [E] start within the approach ellipsoid's scale (4 x 2 x 2 km)
    double resume_max_range_m = 5000.0;    // [E] a resume after an abort flies back to Waypoint 0 from within this
    size_t start_waypoint = 0;             // first waypoint of the path to fly to (tests: 2 = start at Waypoint 2)
    TranslationLoopParameters loop_far;    // outside the keep-out sphere
    TranslationLoopParameters loop_near{1.0, 0.01, 0.002};  // [E] inside: finer
    BreakoutParameters breakout;
};

// Waypoint 0, then for VBar Waypoint 1 / 2; for RBar the fly-around (front point on V-bar at flyaround_radius_m,
// arc points every flyaround_step_rad towards -z, last one short of the zenith), then Waypoint 1 / 2.
std::vector<ProximityWaypoint> proximity_path(DockingApproach approach, const ProximityParameters& p);

enum class CorridorStatus { OutsideKos, Inside, Hold, Retreat };
// `position_port_m`: chaser port in the target port frame; `cg_range_m`: CG to CG. Inside the sphere: the lateral
// offset against max(x tan(half angle), min half width) - Inside / Hold (beyond) / Retreat (beyond factor x bound, or
// x <= 0: behind the port plane without contact).
CorridorStatus corridor_status(const math::Vec3& position_port_m, double cg_range_m, const ProximityParameters& p);

// The chaser attitude that mates its port with the target port: chaser port axes x = -x_t, y = -y_t, z = z_t
// (axes opposed, ups equal). `chaser_port_axes_body`: columns of the chaser port frame in its body frame.
math::Quat docking_attitude_gcrf(const math::Mat3& chaser_port_axes_body, const math::Mat3& target_port_axes_gcrf);

struct CaptureEnvelope {  // [E] spec guide values; i5 verifies them against the IDSS IDD Rev. E
    double axial_min_mps = 0.05, axial_max_mps = 0.10;
    double lateral_max_mps = 0.04;
    double offset_max_m = 0.10;
    double angle_max_rad = 4.0 * constants::deg_to_rad;
    double rate_max_radps = 0.2 * constants::deg_to_rad;
};
struct EnvelopeCheck {
    bool inside = false;
    std::string reason;  // German, the first violated bound ("" when inside)
};
EnvelopeCheck capture_envelope_check(const RelativePortState& state, const CaptureEnvelope& envelope = {});

}  // namespace starsim::gnc
```

- [ ] **Step 1: Write the failing tests** (`core/tests/test_proximity_geometry.cpp`)

```cpp
// ISS i4: proximity geometry - waypoint paths, corridor, docking attitude, capture envelope.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/gnc/proximity_geometry.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"

using namespace starsim;
using math::Vec3;
constexpr double deg = constants::deg_to_rad;

TEST_CASE("Keep-out radius matches the mission geometry", "[iss][proximity]") {
    CHECK(gnc::ProximityParameters{}.keep_out_radius_m == mission::keep_out_sphere_radius_m);
}

TEST_CASE("Approach from the port axis", "[iss][proximity]") {
    CHECK(gnc::docking_approach({0.998, 0.0, -0.05}) == gnc::DockingApproach::VBar);
    CHECK(gnc::docking_approach({0.1, 0.0, -0.99}) == gnc::DockingApproach::RBar);
    CHECK_THROWS_AS(gnc::docking_approach({0.0, 1.0, 0.0}), std::invalid_argument);
}

TEST_CASE("V-bar path: Waypoint 0, 1, 2", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    const auto path = gnc::proximity_path(gnc::DockingApproach::VBar, p);
    REQUIRE(path.size() == 3);
    CHECK(path[0].name == "Wegpunkt 0");
    CHECK(path[0].frame == gnc::WaypointFrame::LvlhFromCg);
    CHECK(path[0].position_m.z == 400.0);
    CHECK(path[1].name == "Wegpunkt 1");
    CHECK(path[1].frame == gnc::WaypointFrame::TargetPort);
    CHECK(path[1].position_m.x == 220.0);
    CHECK(path[2].position_m.x == 20.0);
    CHECK(path[2].on_axis);
    CHECK(path[1].hold_s == 300.0);
}

TEST_CASE("R-bar path flies around the front at 250 m", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    const auto path = gnc::proximity_path(gnc::DockingApproach::RBar, p);
    // Waypoint 0, front (0 deg), 30 deg, 60 deg, Waypoint 1 (zenith axis), Waypoint 2
    REQUIRE(path.size() == 6);
    CHECK(path[1].name == "Umflug 1");
    CHECK(path[1].position_m.x == Catch::Approx(250.0));
    CHECK(path[2].position_m.x == Catch::Approx(250.0 * std::cos(30 * deg)));
    CHECK(path[2].position_m.z == Catch::Approx(-250.0 * std::sin(30 * deg)));  // up = -z
    for (size_t k = 1; k <= 3; ++k) CHECK(norm(path[k].position_m) == Catch::Approx(250.0));
    CHECK(path[4].name == "Wegpunkt 1");
    CHECK(path[4].frame == gnc::WaypointFrame::TargetPort);
}

TEST_CASE("Corridor: inside, hold, retreat, outside the sphere", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    const double t10 = std::tan(10 * deg);
    CHECK(gnc::corridor_status({50.0, 0.9 * 50 * t10, 0.0}, 80.0, p) == gnc::CorridorStatus::Inside);
    CHECK(gnc::corridor_status({50.0, 1.5 * 50 * t10, 0.0}, 80.0, p) == gnc::CorridorStatus::Hold);
    CHECK(gnc::corridor_status({50.0, 0.0, -2.5 * 50 * t10}, 80.0, p) == gnc::CorridorStatus::Retreat);
    CHECK(gnc::corridor_status({0.5, 0.25, 0.0}, 40.0, p) == gnc::CorridorStatus::Inside);  // min half width 0.3 m
    CHECK(gnc::corridor_status({-0.2, 0.0, 0.0}, 40.0, p) == gnc::CorridorStatus::Retreat);
    CHECK(gnc::corridor_status({50.0, 40.0, 0.0}, 250.0, p) == gnc::CorridorStatus::OutsideKos);
}

TEST_CASE("Docking attitude opposes the axes and matches the ups", "[iss][proximity]") {
    // Dragon port: axis body +x, up body -z -> port frame columns (x, -y, -z) in the body
    const auto chaser = math::Mat3::from_columns({1, 0, 0}, {0, -1, 0}, {0, 0, -1});
    // an arbitrary target port frame
    const auto turn = math::Quat::from_axis_angle(normalized(Vec3{0.3, -0.5, 0.8}), 1.1);
    const auto target = turn.to_matrix();
    const auto q = gnc::docking_attitude_gcrf(chaser, target);
    const Vec3 x_c = q.rotate(chaser * Vec3{1, 0, 0}), z_c = q.rotate(chaser * Vec3{0, 0, 1});
    CHECK(dot(x_c, target * Vec3{1, 0, 0}) == Catch::Approx(-1.0).margin(1e-12));
    CHECK(dot(z_c, target * Vec3{0, 0, 1}) == Catch::Approx(1.0).margin(1e-12));
}

TEST_CASE("Capture envelope", "[iss][proximity]") {
    gnc::RelativePortState s;
    s.position_m = {0.0, 0.05, -0.03};
    s.velocity_mps = {-0.075, 0.01, 0.0};
    CHECK(gnc::capture_envelope_check(s).inside);
    auto fast = s;
    fast.velocity_mps.x = -0.12;
    CHECK_FALSE(gnc::capture_envelope_check(fast).inside);
    CHECK(gnc::capture_envelope_check(fast).reason.find("axial") != std::string::npos);
    auto slow = s;
    slow.velocity_mps.x = -0.03;
    CHECK_FALSE(gnc::capture_envelope_check(slow).inside);
    auto off = s;
    off.position_m.y = 0.12;
    CHECK_FALSE(gnc::capture_envelope_check(off).inside);
    auto tilted = s;
    tilted.misalignment_rad = 5 * deg;
    CHECK_FALSE(gnc::capture_envelope_check(tilted).inside);
    auto spinning = s;
    spinning.angular_rate_radps = {0.0, 0.3 * deg, 0.0};
    CHECK_FALSE(gnc::capture_envelope_check(spinning).inside);
}
```

- [ ] **Step 2: Run them to see them fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[proximity]"`
Expected: compile error (`proximity_geometry.hpp` missing).

- [ ] **Step 3: Implement** `core/src/gnc/proximity_geometry.cpp`

Header comment: the research numbers and the spec corrections 2 and 4 (one line each). Implementation:

```cpp
DockingApproach docking_approach(const math::Vec3& axis) {
    const Vec3 u = normalized(axis);
    if (u.x > std::cos(45.0 * constants::deg_to_rad)) return DockingApproach::VBar;
    if (-u.z > std::cos(45.0 * constants::deg_to_rad)) return DockingApproach::RBar;
    throw std::invalid_argument("Kein Anflugweg für diesen Port");
}

std::vector<ProximityWaypoint> proximity_path(DockingApproach approach, const ProximityParameters& p) {
    std::vector<ProximityWaypoint> path;
    path.push_back({"Wegpunkt 0", WaypointFrame::LvlhFromCg, {0.0, 0.0, p.waypoint0_below_m}, p.waypoint0_hold_s, false});
    if (approach == DockingApproach::RBar) {
        int k = 1;  // front point, then the arc towards the zenith; the zenith leg ends at Waypoint 1
        for (double a = 0.0; a < 0.5 * constants::pi - 0.5 * p.flyaround_step_rad; a += p.flyaround_step_rad, ++k) {
            path.push_back({"Umflug " + std::to_string(k), WaypointFrame::LvlhFromCg,
                            {p.flyaround_radius_m * std::cos(a), 0.0, -p.flyaround_radius_m * std::sin(a)},
                            0.0, false});
        }
    }
    path.push_back({"Wegpunkt 1", WaypointFrame::TargetPort, {p.waypoint1_range_m, 0.0, 0.0}, p.waypoint1_hold_s, true});
    path.push_back({"Wegpunkt 2", WaypointFrame::TargetPort, {p.waypoint2_range_m, 0.0, 0.0}, p.waypoint2_hold_s, true});
    return path;
}
```
(`on_axis` of Waypoint 1 is true - it lies on the axis - but Waypoint 1 is *reached* by a CW transfer because its
CG-relative target lies outside the sphere; Task 4 picks the motion by that, see "Segment kind".)

`corridor_status`: `if (cg_range_m >= p.keep_out_radius_m) return OutsideKos; if (x <= 0) return Retreat;
bound = max(x * tan(half), min_half_width); lateral = hypot(y, z); lateral <= bound ? Inside : lateral <= factor *
bound ? Hold : Retreat`.

`docking_attitude_gcrf`: `desired = from_columns(-t.col(0), -t.col(1), t.col(2))` (use `target * Vec3{1,0,0}` etc. for
the columns), `R = desired * transpose(chaser)`, `return Quat::from_matrix(R)`.

`capture_envelope_check`: in this order, German reasons with the value (`detail`-free: `std::to_string` is fine,
or format with `snprintf` "%.3f"): axial closing `-velocity.x` outside [min, max] -> "axial zu schnell" / "axial zu
langsam"; `hypot(vy, vz)` -> "lateral zu schnell"; `hypot(y, z)` -> "Versatz zu groß"; misalignment -> "Winkel zu
groß"; `norm(angular_rate)` -> "Drehrate zu groß". The test searches "axial": keep that word in both axial reasons.

- [ ] **Step 4: Run the tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[proximity]"`
Expected: all pass.

- [ ] **Step 5: Commit and push**

Ledger: create `.superpowers/sdd/2026-10-19-iss-i4-proximity/progress.md` with the task list, "Task 1 DONE <sha>".
Run `-Fast` first.

```bash
git add core/include/starsim/gnc/proximity_geometry.hpp core/src/gnc/proximity_geometry.cpp core/tests/test_proximity_geometry.cpp core/CMakeLists.txt core/tests/CMakeLists.txt .superpowers/sdd/2026-10-19-iss-i4-proximity/progress.md
git commit -m "feat(gnc): proximity geometry - waypoint paths, corridor, docking attitude, capture envelope"
git push origin main
```

---

### Task 2: CW transfers with keep-out clearance and the breakout burn

**Files:**
- Create: `core/include/starsim/gnc/proximity_transfer.hpp`, `core/src/gnc/proximity_transfer.cpp`
- Create: `core/tests/test_proximity_transfer.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `orbit::RelativeState`, `orbit::cw_propagate`, `orbit::cw_transfer_velocity`, `orbit::lvlh_relative_state`,
  `orbit::chaser_from_lvlh` (`orbit/clohessy_wiltshire.hpp`); `gnc::ProximityParameters`, `BreakoutParameters`
  (Task 1); test only: `gnc::coast_with_drag` (`gnc/deorbit_planner.hpp`), `rendezvous_test::start` for an OEM ISS
  state.
- Produces:

```cpp
namespace starsim::gnc {

struct ProximityTransfer {
    bool feasible = false;
    double duration_s = 0.0;
    math::Vec3 departure_velocity_lvlh_mps;  // relative velocity needed now (CW)
    double min_cg_range_m = 0.0;             // closest approach to the station CG on the CW arc
};
// CW transfer of the chaser CG from `from` (current relative state, LVLH about the station CG) to `to_cg_m` (at rest
// there). Tries nominal_s, then 0.75, 1.25, 0.5, 1.5 x nominal_s; the first whose sampled arc (every 5 s) keeps
// >= keep_out_radius_m + kos_margin_m from the CG wins. Infeasible (all inside, or CW singular) otherwise.
ProximityTransfer plan_proximity_transfer(const orbit::RelativeState& from, const math::Vec3& to_cg_m,
                                          double n_radps, double nominal_s, const ProximityParameters& p);

struct Breakout {
    bool safe = false;
    math::Vec3 delta_v_lvlh_mps;
    double exit_s = 0.0;               // when the CW arc leaves the sphere (+ margin)
    double min_range_after_exit_m = 0.0;
};
// Smallest magnitude of p.breakout.magnitudes_mps with a direction (26 cube directions + the radial-away direction)
// whose CW arc leaves the sphere within exit_limit_s without first closing in (range never below 0.9 x the start
// range before the exit) and stays outside keep_out + margin until horizon_s; among those the largest minimum range
// after the exit. Unsafe (best effort: the largest magnitude, away direction) when none qualifies.
Breakout breakout_delta_v(const orbit::RelativeState& from, double n_radps, const ProximityParameters& p);

}  // namespace starsim::gnc
```

- [ ] **Step 1: Write the failing tests** (`core/tests/test_proximity_transfer.cpp`)

```cpp
// ISS i4: CW proximity transfers with keep-out clearance, the breakout burn, and CW against the full model.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/gnc/proximity_transfer.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "rendezvous_test_support.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;
constexpr double n_iss = 1.128e-3;  // [D] mean motion at ~420 km

TEST_CASE("Waypoint 0 to Waypoint 1 (IDA-2) clears the keep-out sphere", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    const orbit::RelativeState wp0{{0.0, 0.0, 400.0}, {}};
    const Vec3 wp1_cg{35.0 + 220.0, 0.0, 0.0};  // IDA-2 35 m ahead of the CG [E config], 220 m out
    const auto t = gnc::plan_proximity_transfer(wp0, wp1_cg, n_iss, p.transfer_s, p);
    REQUIRE(t.feasible);
    CHECK(t.min_cg_range_m >= 210.0);
    const auto end = orbit::cw_propagate({wp0.position_m, t.departure_velocity_lvlh_mps}, n_iss, t.duration_s);
    CHECK(norm(end.position_m - wp1_cg) < 1e-6);
    std::printf("[validation] WP0->WP1 CW transfer %.0f s, dv %.3f m/s, closest %.1f m\n", t.duration_s,
                norm(t.departure_velocity_lvlh_mps), t.min_cg_range_m);
}

TEST_CASE("A transfer through the sphere is refused", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    // straight through the station: from 400 m below to 400 m above in a short time
    const auto t = gnc::plan_proximity_transfer({{0.0, 0.0, 400.0}, {}}, {0.0, 0.0, -400.0}, n_iss, 300.0, p);
    CHECK_FALSE(t.feasible);
}

TEST_CASE("Breakout from the final approach is passively safe for 24 h (CW)", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    for (const Vec3 at : {Vec3{45.0, 0.0, 0.0}, Vec3{28.0, 0.0, -15.0}}) {  // 10 m out on IDA-2 / IDA-3 (CG-relative)
        CAPTURE(at.x, at.z);
        const orbit::RelativeState from{at, {-0.08 * at.x / norm(at), 0.0, -0.08 * at.z / norm(at)}};
        const auto b = gnc::breakout_delta_v(from, n_iss, p);
        REQUIRE(b.safe);
        CHECK(norm(b.delta_v_lvlh_mps) <= 1.5);
        CHECK(b.exit_s <= p.breakout.exit_limit_s);
        std::printf("[validation] breakout from (%.0f,%.0f,%.0f): dv %.2f m/s (%.2f,%.2f,%.2f), exit %.0f s, "
                    "closest after exit %.0f m\n", at.x, at.y, at.z, norm(b.delta_v_lvlh_mps), b.delta_v_lvlh_mps.x,
                    b.delta_v_lvlh_mps.y, b.delta_v_lvlh_mps.z, b.exit_s, b.min_range_after_exit_m);
    }
}

TEST_CASE("CW against the full model over a proximity transfer and a breakout", "[iss][proximity][validation]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    const auto s = rendezvous_test::start(clock, earth, 0.0, 0.0);  // ISS from the OEM fixture
    const environment::PointMassGravity gravity{ephemeris, earth};  // as test_phasing_planner (Earth J2 + Moon + Sun)
    const auto& iss = s.iss.state_gcrf;
    const double n = std::sqrt(constants::earth_gm_m3ps2 / std::pow(norm(iss.position_m), 3));
    const double beta_iss = 1177.76 * 1.5 / 460320.0, beta_dragon = 2.2 * constants::pi * 2.0 * 2.0 / 12500.0;  // [D]
    const gnc::ProximityParameters p;
    // 1) the WP0 -> WP1 transfer: CW arrival against the full model
    const orbit::RelativeState wp0{{0.0, 0.0, 400.0}, {}};
    const auto t = gnc::plan_proximity_transfer(wp0, {255.0, 0.0, 0.0}, n, p.transfer_s, p);
    REQUIRE(t.feasible);
    const auto chaser0 = orbit::chaser_from_lvlh(iss, {wp0.position_m, t.departure_velocity_lvlh_mps});
    const auto t1 = s.epoch + t.duration_s;
    const auto iss1 = gnc::coast_with_drag(gravity, iss, s.epoch, t1, beta_iss);
    const auto chaser1 = gnc::coast_with_drag(gravity, chaser0, s.epoch, t1, beta_dragon);
    const auto rel = orbit::lvlh_relative_state(iss1, chaser1);
    const double miss_m = norm(rel.position_m - Vec3{255.0, 0.0, 0.0});
    std::printf("[validation] CW vs full model, WP0->WP1 %.0f s: miss %.3f m\n", t.duration_s, miss_m);
    CHECK(miss_m < 2.0);  // spec correction 1: the loop's arrival tolerance; expected centimetres to decimetres
    // 2) a breakout from 10 m out on IDA-2: never back inside the sphere for 24 h in the full model
    const orbit::RelativeState from{{45.0, 0.0, 0.0}, {-0.08, 0.0, 0.0}};
    const auto b = gnc::breakout_delta_v(from, n, p);
    REQUIRE(b.safe);
    auto c = orbit::chaser_from_lvlh(iss, {from.position_m, from.velocity_mps + b.delta_v_lvlh_mps});
    auto st = iss;
    double closest_after_exit = 1e9;
    for (double k = 60.0; k <= 86400.0; k += 60.0) {
        st = gnc::coast_with_drag(gravity, st, s.epoch + (k - 60.0), s.epoch + k, beta_iss);
        c = gnc::coast_with_drag(gravity, c, s.epoch + (k - 60.0), s.epoch + k, beta_dragon);
        if (k > b.exit_s) closest_after_exit = std::min(closest_after_exit, norm(c.position_m - st.position_m));
    }
    std::printf("[validation] breakout full model 24 h: closest after the exit %.0f m\n", closest_after_exit);
    CHECK(closest_after_exit > p.keep_out_radius_m);
}
```

Includes for that case: `starsim/environment/gravity.hpp` (`PointMassGravity`, as `test_phasing_planner.cpp:37`).
1440 coast steps of 60 s each should run in a few seconds; if the case exceeds ~40 s, tag it `[slow]`.

- [ ] **Step 2: Run to see them fail** - `-TestFilter "[proximity]"`: compile error.

- [ ] **Step 3: Implement** `core/src/gnc/proximity_transfer.cpp`

```cpp
namespace {
double min_range_on_arc(const orbit::RelativeState& s, double n, double duration_s, double step_s) {
    double closest = norm(s.position_m);
    for (double t = step_s; t <= duration_s + 0.5 * step_s; t += step_s) {
        closest = std::min(closest, norm(orbit::cw_propagate(s, n, std::min(t, duration_s)).position_m));
    }
    return closest;
}
}  // namespace

ProximityTransfer plan_proximity_transfer(const orbit::RelativeState& from, const math::Vec3& to_cg_m, double n,
                                          double nominal_s, const ProximityParameters& p) {
    const double clearance = p.keep_out_radius_m + p.kos_margin_m;
    for (const double f : {1.0, 0.75, 1.25, 0.5, 1.5}) {
        const double T = f * nominal_s;
        const auto v = orbit::cw_transfer_velocity(from.position_m, to_cg_m, n, T);
        if (!v) continue;
        const double closest = min_range_on_arc({from.position_m, *v}, n, T, 5.0);
        if (closest >= clearance) return {true, T, *v, closest};
    }
    return {};
}
```

`breakout_delta_v`: build the 26 directions `(i, j, k)` in {-1, 0, 1}^3 without (0,0,0), normalised, plus
`normalized(from.position_m)`. For each magnitude in order, for each direction: `s = {from.position_m,
from.velocity_mps + m * d}`; sample `cw_propagate(s, n, t)` every `sample_s` up to `horizon_s`; `exit_s` = first t
with range >= keep_out + margin (0 when already outside); reject if no exit by `exit_limit_s`, or a range < 0.9 x
start range before the exit, or a range < keep_out + margin after the exit; keep the best (largest minimum after
exit). Return the first magnitude's best. 27 directions x 2880 samples x 4 magnitudes is ~3e5 CW evaluations: fine
once per abort.

- [ ] **Step 4: Run** `-TestFilter "[proximity]"`; all pass. Record the printed numbers (transfer dv, closest
approach, breakout dv/exit, CW-vs-full-model miss, 24 h closest) in the ledger. If the CW-vs-full-model miss is not
below 2 m, stop and report (spec correction 1 depends on it).

- [ ] **Step 5: Commit and push** (`-Fast` first)

```bash
git add core/include/starsim/gnc/proximity_transfer.hpp core/src/gnc/proximity_transfer.cpp core/tests/test_proximity_transfer.cpp core/CMakeLists.txt core/tests/CMakeLists.txt .superpowers/sdd/2026-10-19-iss-i4-proximity/progress.md
git commit -m "feat(gnc): CW proximity transfers with keep-out clearance and breakout burn"
git push origin main
```

---

### Task 3: Translation loop and approach laws

**Files:**
- Create: `core/include/starsim/gnc/translation_loop.hpp`, `core/src/gnc/translation_loop.cpp`
- Create: `core/tests/test_translation_loop.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `TranslationLoopParameters`, `ProximityParameters` (Task 1).
- Produces:

```cpp
namespace starsim::gnc {

// Pure translation authority along the six axes of the frame the loop works in (m/s^2): +x, -x, +y, -y, +z, -z.
struct AxisAcceleration {
    std::array<double, 6> mps2{};
    double along(int axis, double sign) const { return mps2[2 * axis + (sign < 0.0 ? 1 : 0)]; }
};

// Desired relative velocity -> translation fractions per axis with a hysteresis deadband per axis (a pulse train
// starts above start_error_mps and runs until the error is below stop_error_mps), fraction = error / (a * gain_time)
// clamped to [-1, 1]; the vector is scaled to norm <= 1.
class TranslationLoop {
public:
    math::Vec3 command(const math::Vec3& desired_mps, const math::Vec3& velocity_mps, const AxisAcceleration& authority,
                       const TranslationLoopParameters& p);
    void reset() { firing_ = {false, false, false}; }
    bool firing() const { return firing_[0] || firing_[1] || firing_[2]; }
private:
    std::array<bool, 3> firing_{false, false, false};
};

// Closing speed (> 0 towards the port) on the axis at axial distance x: between Waypoint 2 and the port linear from
// final_speed_start_mps to contact_speed_mps; farther out clamp(x / approach_time_constant_s, final_speed_start_mps,
// approach_speed_far_mps).
double axial_closing_speed_mps(double x_m, const ProximityParameters& p);

// Desired velocity in the target port frame on the axial approach: x = -closing speed, lateral = -offset *
// lateral_gain_per_s, the lateral vector limited to lateral_speed_max_mps.
math::Vec3 approach_velocity_port(const math::Vec3& position_port_m, const ProximityParameters& p);

// Position hold: -(position - hold) / hold_time_constant_s, limited to hold_speed_max_mps (any frame).
math::Vec3 hold_velocity(const math::Vec3& position_m, const math::Vec3& hold_m, const ProximityParameters& p);

}  // namespace starsim::gnc
```

- [ ] **Step 1: Write the failing tests** (`core/tests/test_translation_loop.cpp`)

```cpp
// ISS i4: translation loop (deadband) and the approach laws.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/translation_loop.hpp"

using namespace starsim;
using math::Vec3;

namespace {
gnc::AxisAcceleration dragon() { return {{0.1245, 0.1245, 0.0453, 0.0453, 0.0453, 0.0453}}; }  // i2 measured
}

TEST_CASE("Deadband: no pulse below start, fire until below stop", "[iss][proximity]") {
    gnc::TranslationLoop loop;
    const gnc::TranslationLoopParameters p;  // start 0.02, stop 0.005
    CHECK(norm(loop.command({0.015, 0.0, 0.0}, {}, dragon(), p)) == 0.0);
    const Vec3 f = loop.command({0.03, 0.0, 0.0}, {}, dragon(), p);
    CHECK(f.x == Catch::Approx(0.03 / 0.1245));
    CHECK(loop.firing());
    CHECK(loop.command({0.03, 0.0, 0.0}, {0.02, 0.0, 0.0}, dragon(), p).x > 0.0);  // 0.01 > stop: still firing
    CHECK(loop.command({0.03, 0.0, 0.0}, {0.027, 0.0, 0.0}, dragon(), p).x == 0.0);  // 0.003 < stop
    CHECK_FALSE(loop.firing());
}

TEST_CASE("Loop uses the authority of the commanded sign and saturates", "[iss][proximity]") {
    gnc::TranslationLoop loop;
    const gnc::TranslationLoopParameters p;
    const Vec3 f = loop.command({0.0, -1.0, 0.5}, {}, dragon(), p);
    CHECK(norm(f) == Catch::Approx(1.0));
    CHECK(f.y < 0.0);
    CHECK(f.z > 0.0);
}

TEST_CASE("Simulated 1D closing converges to the desired speed", "[iss][proximity]") {
    gnc::TranslationLoop loop;
    const gnc::TranslationLoopParameters p{1.0, 0.01, 0.002};
    double v = 0.0;
    for (int k = 0; k < 2000; ++k) {  // 10 s at 200 Hz
        const Vec3 f = loop.command({-0.1, 0.0, 0.0}, {v, 0.0, 0.0}, dragon(), p);
        v += f.x * dragon().along(0, f.x) * 0.005;
    }
    CHECK(v == Catch::Approx(-0.1).margin(0.003));
}

TEST_CASE("Axial speed schedule", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    CHECK(gnc::axial_closing_speed_mps(220.0, p) == Catch::Approx(0.30));
    CHECK(gnc::axial_closing_speed_mps(120.0, p) == Catch::Approx(0.20));
    CHECK(gnc::axial_closing_speed_mps(40.0, p) == Catch::Approx(0.10));
    CHECK(gnc::axial_closing_speed_mps(20.0, p) == Catch::Approx(0.10));
    CHECK(gnc::axial_closing_speed_mps(10.0, p) == Catch::Approx(0.0875));
    CHECK(gnc::axial_closing_speed_mps(0.0, p) == Catch::Approx(0.075));
}

TEST_CASE("Approach and hold velocities", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    const Vec3 v = gnc::approach_velocity_port({20.0, 0.5, 0.0}, p);
    CHECK(v.x == Catch::Approx(-0.10));
    CHECK(v.y == Catch::Approx(-0.01));
    CHECK(norm(Vec3{0, gnc::approach_velocity_port({20.0, 5.0, 5.0}, p).y,
                    gnc::approach_velocity_port({20.0, 5.0, 5.0}, p).z}) == Catch::Approx(0.03));
    CHECK(norm(gnc::hold_velocity({10.0, 0, 0}, {0, 0, 0}, p)) == Catch::Approx(0.05));
    CHECK(gnc::hold_velocity({1.2, 0, 0}, {0, 0, 0}, p).x == Catch::Approx(-0.02));
}
```

- [ ] **Step 2: Run to see them fail** (compile error).

- [ ] **Step 3: Implement** `core/src/gnc/translation_loop.cpp`

```cpp
math::Vec3 TranslationLoop::command(const math::Vec3& desired, const math::Vec3& velocity,
                                    const AxisAcceleration& authority, const TranslationLoopParameters& p) {
    const math::Vec3 e = desired - velocity;
    const double err[3] = {e.x, e.y, e.z};
    double f[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; ++i) {
        const double a = std::abs(err[i]);
        if (!firing_[i] && a > p.start_error_mps) firing_[i] = true;
        if (firing_[i] && a < p.stop_error_mps) firing_[i] = false;
        const double acc = authority.along(i, err[i]);
        if (firing_[i] && acc > 0.0) f[i] = std::clamp(err[i] / (acc * p.gain_time_s), -1.0, 1.0);
    }
    math::Vec3 out{f[0], f[1], f[2]};
    const double m = norm(out);
    return m > 1.0 ? out / m : out;
}

double axial_closing_speed_mps(double x, const ProximityParameters& p) {
    if (x <= p.waypoint2_range_m) {
        const double s = std::clamp(x / p.waypoint2_range_m, 0.0, 1.0);
        return p.contact_speed_mps + (p.final_speed_start_mps - p.contact_speed_mps) * s;
    }
    return std::clamp(x / p.approach_time_constant_s, p.final_speed_start_mps, p.approach_speed_far_mps);
}
```
`approach_velocity_port` and `hold_velocity` as specified (limit the lateral vector / the whole vector by norm).
Check the 120 m case: 120 / 600 = 0.20.

- [ ] **Step 4: Run** `-TestFilter "[proximity]"`; all pass.

- [ ] **Step 5: Commit and push** (`-Fast` first)

```bash
git add core/include/starsim/gnc/translation_loop.hpp core/src/gnc/translation_loop.cpp core/tests/test_translation_loop.cpp core/CMakeLists.txt core/tests/CMakeLists.txt .superpowers/sdd/2026-10-19-iss-i4-proximity/progress.md
git commit -m "feat(gnc): translation deadband loop and proximity approach laws"
git push origin main
```

---

### Task 4: Proximity guidance state machine

**Files:**
- Create: `core/include/starsim/gnc/proximity_guidance.hpp`, `core/src/gnc/proximity_guidance.cpp`
- Create: `core/tests/test_proximity_guidance.cpp`, `core/tests/proximity_test_support.hpp` (the CW point-mass
  simulator used by this test)
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: everything from Tasks 1-3; `orbit::RelativeState`, `orbit::cw_transfer_velocity`;
  `gnc::RelativePortState`.
- Produces (used by Task 5):

```cpp
namespace starsim::gnc {

// German names (to_string): "Wegpunkt" (holding at path waypoint `waypoint()` - its name is the status), "Annäherung"
// (CW transfer outside the sphere), "Umflug" (CW transfer to a fly-around point), "Anflug" (axial approach Waypoint 1
// -> 2), "Endanflug" (Waypoint 2 -> contact), "Halt", "Rückzug", "Abbruch", "Kontakt".
enum class ProximityPhase { Waypoint, Transfer, Flyaround, Approach, FinalApproach, Hold, Retreat, Abort, Contact };
const char* to_string(ProximityPhase phase);

struct ProximityInput {
    time::Epoch epoch;
    orbit::RelativeState cg_lvlh;       // chaser CG relative to the station CG, station LVLH (velocity in the rotating frame)
    RelativePortState port;             // chaser port vs target port, target port frame
    math::Mat3 port_to_lvlh;            // target port frame axes in the station LVLH frame (columns)
    double mean_motion_radps = 0.0;     // station orbit
    AxisAcceleration authority_port;    // pure translation authority along the target port frame axes
    double propellant_kg = 0.0;
};
struct ProximityOutput {
    math::Vec3 translation_fraction_port;  // to FlightSession::set_translation_command semantics
    bool attitude_hold = true;             // false after a completed breakout: the vehicle drifts (and may coast)
};
struct ProximityEvent {
    time::Epoch epoch;
    ProximityPhase phase;
    std::string status;  // German
};
struct ProximityContact {
    time::Epoch epoch;
    RelativePortState port;
    EnvelopeCheck envelope;
};

class ProximityGuidance {
public:
    ProximityGuidance(DockingApproach approach, ProximityParameters parameters);
    ProximityOutput update(const ProximityInput& in);

    // Commands (German status; false and no change when refused):
    bool hold();     // keep the current relative position; refused in Abort / Contact / Hold
    bool retreat();  // back to the last waypoint reached; refused at/before Waypoint 0, in Abort / Contact / Retreat
    bool abort();    // breakout now; refused in Abort / Contact
    bool resume();   // from a commanded Hold, a waiting waypoint after a retreat, or Abort (back to Waypoint 0 when the
                     // CG range <= resume_max_range_m); refused otherwise

    ProximityPhase phase() const { return phase_; }
    size_t waypoint() const { return target_; }  // index into path() of the waypoint flown to / held at
    const std::vector<ProximityWaypoint>& path() const { return path_; }
    const std::string& status() const { return status_; }
    const std::vector<ProximityEvent>& events() const { return events_; }
    const std::optional<ProximityContact>& contact() const { return contact_; }
private:
    // see the implementation step
};

}  // namespace starsim::gnc
```

**Behaviour (the implementation must follow this exactly; each rule has a test below):**
- Positions: `p_port_lvlh = port_to_lvlh * in.port.position_m`; `offset = p_port_lvlh - in.cg_lvlh.position_m`
  (constant: station port lever and chaser port lever in the held attitude). A waypoint's CG-relative LVLH target is
  `position_m` (LvlhFromCg) or `port_to_lvlh * position_m - offset` (TargetPort).
- **Segment kind** from the target waypoint: if it is `on_axis` and its CG-relative target lies inside the sphere
  (`norm < keep_out_radius_m`) -> axial approach (`Approach` towards Waypoint 2... generally any on-axis waypoint
  inside, `FinalApproach` after the last waypoint); otherwise a CW transfer (`Transfer`, or `Flyaround` when the
  target's name starts with "Umflug" or the previous waypoint did).
- **Start** (first update): target = `start_waypoint`, `reached_ = start_waypoint` (the retreat fallback). If the chaser is within the waypoint's arrival tolerance ->
  phase Waypoint (hold timer starts). Else a segment towards it (the transfer duration: `transfer_s` for Waypoint 0
  -> next and for a start, `flyaround_leg_s` for fly-around legs).
- **Transfer / Flyaround:** on entering, `plan_proximity_transfer(cg_lvlh, target_cg, n, nominal, p)`; infeasible ->
  Abort with "Abbruch: kein sicherer Transferweg". Arrival epoch fixed at entry. Each update with time to go
  `T > transfer_final_s`: `v_req = cw_transfer_velocity(cg.position, target_cg, n, T)` (keep the last one when empty);
  later `v_req = hold_velocity(cg.position, target_cg)`. Loop `loop_far` on `v_req` vs `cg_lvlh.velocity` in LVLH; the
  output is converted to the port frame: `transpose(port_to_lvlh) * f_lvlh` with the authority taken in the port frame
  (convert the LVLH desired and actual velocities to the port frame first, then run the loop there - one loop frame).
  Arrived when the CG-relative error < `arrival_tolerance_m` and the relative speed < `arrival_speed_mps` -> Waypoint.
- **Approach / FinalApproach** (in the port frame): `v_req = approach_velocity_port(port.position_m)`; `loop_near`.
  Approach ends at the waypoint's axial distance (`x <= x_wp + axis_arrival_tolerance_m`) -> Waypoint (the hold law
  then stops it on the axis). FinalApproach ends at `x <= contact_plane_m` -> Contact: record `ProximityContact` with
  `capture_envelope_check(in.port)`, output zero from then on, status "Kontakt" (+ " außerhalb der Fangtoleranz:
  <reason>" when outside).
- **Waypoint:** hold law towards the waypoint (`hold_velocity`, LVLH for LvlhFromCg, port frame for TargetPort),
  `loop_near` inside the sphere else `loop_far`; after `hold_s` (counted from arrival) the next segment starts,
  unless the waypoint is "waiting for go" (after a retreat): then it holds until `resume()`. After the last waypoint
  the next segment is FinalApproach.
- **Monitors** every update, in Approach / FinalApproach / Waypoint inside the sphere / Hold (automatic):
  1. `propellant_kg < deorbit_reserve_kg` -> Abort ("Abbruch: Treibstoffreserve") in any phase except Abort/Contact.
  2. `corridor_status(port.position_m, norm(cg_lvlh.position_m), p)`: Hold -> automatic Hold (hold point = on the
     axis at the current x: `{x, 0, 0}` port frame; status "Halt: Korridor verlassen"); Retreat -> Retreat.
  3. Closing speed `port.closing_speed_mps > overspeed_factor * axial_closing_speed_mps(x) + overspeed_margin_mps` ->
     automatic Hold ("Halt: zu schnell").
  4. In Transfer / Flyaround: the CG range < keep_out_radius_m while the corridor status is not Inside -> Abort
     ("Abbruch: Sicherheitszone verletzt").
- **Automatic Hold** resumes the interrupted phase by itself once the lateral offset is below half the corridor bound
  and the relative speed below `arrival_speed_mps`; a **commanded Hold** holds the position at the command (in the port
  frame inside the sphere, else LVLH CG-relative) until `resume()`.
- **Retreat:** target = the last waypoint reached (index `reached_`, the waypoint whose hold was last entered); inside
  the sphere move out along the axis: `v_req = {+retreat_speed_mps, lateral convergence}` until `x >= x_wp`, then
  Waypoint with "waiting for go"; outside: a CW transfer back with `transfer_s`. Status "Rückzug zu <name>".
- **Abort:** `breakout_delta_v(cg_lvlh, n, p)` once at entry -> `v_target_lvlh = cg.velocity + delta_v`; loop
  (`loop_far`, start error 0 - fire immediately: use a copy of the parameters with `start_error_mps = 0`) until the
  error < `stop_error_mps`; then `attitude_hold = false`, output zero, status "Abbruch: Ausweichmanöver ausgeführt"
  (or "... nicht passiv sicher" when `!safe`). Resume from Abort: target = Waypoint 0, a Transfer with `transfer_s`
  (refused, "Wiederaufnahme nicht möglich: zu weit entfernt", beyond `resume_max_range_m`).
- **Events:** every phase change pushes a `ProximityEvent` (epoch, phase, status); arriving at a waypoint pushes one
  with the waypoint's name.

- [ ] **Step 1: Write the CW point-mass simulator** (`core/tests/proximity_test_support.hpp`)

```cpp
#pragma once
// ISS i4 tests: a CW point-mass chaser with an ideal port-aligned attitude around a station in ideal LVLH attitude.
// The chaser CG is `chaser_lever` behind its port; the target port sits at `port_lever` from the station CG with the
// axes `port_to_lvlh`. Translation fractions act with the Dragon's i2 pure authority.
#include "starsim/gnc/proximity_guidance.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"

namespace starsim::proximity_test {

struct CwSim {
    double n = 1.128e-3;
    math::Mat3 port_to_lvlh;   // target port frame in LVLH
    math::Vec3 port_lever_lvlh;  // station CG -> target port
    double chaser_lever_m = 4.0;  // chaser port ahead of its CG along its docking axis (= -x_t)
    orbit::RelativeState cg;      // chaser CG, LVLH
    gnc::AxisAcceleration authority{{0.1245, 0.1245, 0.0453, 0.0453, 0.0453, 0.0453}};  // port axes: x is the Dragon's -x...
    double propellant_kg = 2000.0;
    double kg_per_mps = 16.2;  // [D] i2 lateral cost (conservative for every axis)
    time::Epoch epoch;

    static CwSim ida2() {
        CwSim s;
        s.port_to_lvlh = math::Mat3::from_columns({1, 0, 0}, {0, -1, 0}, {0, 0, -1});  // axis +x, up -z (zenith)
        s.port_lever_lvlh = {35.0, 0.0, 0.0};
        return s;
    }
    static CwSim ida3() {
        CwSim s;
        s.port_to_lvlh = math::Mat3::from_columns({0, 0, -1}, {0, -1, 0}, {1, 0, 0});  // axis -z, up +x; y = z x x
        s.port_lever_lvlh = {28.0, 0.0, -5.0};
        return s;
    }
    math::Vec3 chaser_port_lvlh() const {  // the chaser port leads its CG along -x_t (towards the target port)
        return cg.position_m - chaser_lever_m * (port_to_lvlh * math::Vec3{1, 0, 0});
    }
    gnc::ProximityInput input() const {
        gnc::ProximityInput in;
        in.epoch = epoch;
        in.cg_lvlh = cg;
        const math::Mat3 lvlh_to_port = transpose(port_to_lvlh);
        in.port.position_m = lvlh_to_port * (chaser_port_lvlh() - port_lever_lvlh);
        in.port.velocity_mps = lvlh_to_port * cg.velocity_mps;
        in.port.range_m = norm(in.port.position_m);
        in.port.closing_speed_mps = -dot(in.port.position_m, in.port.velocity_mps) / std::max(in.port.range_m, 1e-9);
        in.port_to_lvlh = port_to_lvlh;
        in.mean_motion_radps = n;
        in.authority_port = authority;
        in.propellant_kg = propellant_kg;
        return in;
    }
    // One step: CW drift plus the commanded acceleration (applied as a velocity change at the start of the step).
    void step(const math::Vec3& fraction_port, double dt) {
        const math::Vec3 f = port_to_lvlh * fraction_port;
        const math::Vec3 fp = fraction_port;
        const math::Vec3 a_port{fp.x * authority.along(0, fp.x), fp.y * authority.along(1, fp.y),
                                fp.z * authority.along(2, fp.z)};
        const math::Vec3 dv = port_to_lvlh * a_port * dt;
        (void)f;
        propellant_kg -= norm(dv) * kg_per_mps;
        cg.velocity_mps += dv;
        cg = orbit::cw_propagate(cg, n, dt);
        epoch = epoch + dt;
    }
};

// Runs the guidance until contact, abort completion or `limit_s`; returns the simulated seconds.
inline double run(gnc::ProximityGuidance& g, CwSim& s, double limit_s, double dt = 0.05) {
    double t = 0.0;
    for (; t < limit_s; t += dt) {
        const auto out = g.update(s.input());
        if (g.phase() == gnc::ProximityPhase::Contact) break;
        if (g.phase() == gnc::ProximityPhase::Abort && !out.attitude_hold) break;
        s.step(out.translation_fraction_port, dt);
    }
    return t;
}

}  // namespace starsim::proximity_test
```
(Remove the unused `f` lines while implementing; they are a reminder that the fraction is in the port frame.)

- [ ] **Step 2: Write the failing tests** (`core/tests/test_proximity_guidance.cpp`)

```cpp
// ISS i4: proximity guidance state machine on a CW point mass (fast; the session campaigns are in Task 6).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdio>

#include "proximity_test_support.hpp"

using namespace starsim;
using math::Vec3;
using gnc::ProximityPhase;
using proximity_test::CwSim;

namespace {
bool saw(const gnc::ProximityGuidance& g, ProximityPhase phase) {
    for (const auto& e : g.events()) if (e.phase == phase) return true;
    return false;
}
CwSim at_waypoint0(CwSim s) {
    s.cg = {{-5.0, 0.0, 397.0}, {0.01, 0.0, -0.01}};  // i3's Waypoint 0 arrival: a few metres, ~0.02 m/s
    return s;
}
}  // namespace

TEST_CASE("IDA-2: Waypoint 0 to contact inside the envelope", "[iss][proximity]") {
    auto s = at_waypoint0(CwSim::ida2());
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, {});
    const double kg0 = s.propellant_kg;
    const double t = proximity_test::run(g, s, 3.0 * 3600.0);
    REQUIRE(g.contact());
    std::printf("[validation] CW IDA-2: contact after %.1f min, %.1f kg, axial %.3f m/s, offset %.3f m, %s\n", t / 60.0,
                kg0 - s.propellant_kg, -g.contact()->port.velocity_mps.x,
                std::hypot(g.contact()->port.position_m.y, g.contact()->port.position_m.z),
                g.contact()->envelope.reason.c_str());
    CHECK(g.contact()->envelope.inside);
    CHECK(-g.contact()->port.velocity_mps.x < 0.1);
    CHECK_FALSE(saw(g, ProximityPhase::Hold));
    CHECK_FALSE(saw(g, ProximityPhase::Retreat));
    CHECK_FALSE(saw(g, ProximityPhase::Abort));
    CHECK(saw(g, ProximityPhase::Approach));
    CHECK(saw(g, ProximityPhase::FinalApproach));
    CHECK(t < 90.0 * 60.0);
}

TEST_CASE("IDA-3: fly-around to the zenith axis, contact inside the envelope", "[iss][proximity]") {
    auto s = at_waypoint0(CwSim::ida3());
    gnc::ProximityGuidance g(gnc::DockingApproach::RBar, {});
    proximity_test::run(g, s, 4.0 * 3600.0);
    REQUIRE(g.contact());
    CHECK(g.contact()->envelope.inside);
    CHECK(saw(g, ProximityPhase::Flyaround));
    CHECK_FALSE(saw(g, ProximityPhase::Abort));
}

TEST_CASE("Never inside the keep-out sphere outside the corridor", "[iss][proximity]") {
    for (auto s0 : {CwSim::ida2(), CwSim::ida3()}) {
        auto s = at_waypoint0(s0);
        gnc::ProximityGuidance g(s0.port_lever_lvlh.z < 0 ? gnc::DockingApproach::RBar : gnc::DockingApproach::VBar, {});
        for (double t = 0.0; t < 4.0 * 3600.0 && !g.contact(); t += 0.05) {
            const auto in = s.input();
            const auto out = g.update(in);
            if (norm(in.cg_lvlh.position_m) < 200.0) {
                CHECK(gnc::corridor_status(in.port.position_m, norm(in.cg_lvlh.position_m), {}) ==
                      gnc::CorridorStatus::Inside);
            }
            s.step(out.translation_fraction_port, 0.05);
        }
        CHECK(g.contact());
    }
}

TEST_CASE("Corridor violation holds and resumes by itself", "[iss][proximity]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;  // fly from Waypoint 2
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 20.0, 5.0, 0.0}, {}};  // 20 m out, 5 m lateral: 14 deg > 10 deg, < 20 deg
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    proximity_test::run(g, s, 3600.0);
    CHECK(saw(g, ProximityPhase::Hold));
    REQUIRE(g.contact());
    CHECK(g.contact()->envelope.inside);
}

TEST_CASE("A large corridor deviation retreats and waits for the go", "[iss][proximity]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 15.0, 7.0, 0.0}, {}};  // 25 deg off the axis
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    proximity_test::run(g, s, 1800.0);
    CHECK(saw(g, ProximityPhase::Retreat));
    CHECK(g.phase() == ProximityPhase::Waypoint);
    CHECK_FALSE(g.contact());   // waiting at Waypoint 2
    REQUIRE(g.resume());
    proximity_test::run(g, s, 1800.0);
    REQUIRE(g.contact());
    CHECK(g.contact()->envelope.inside);
}

TEST_CASE("Commanded hold, resume, abort and refusals", "[iss][proximity]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 20.0, 0.0, 0.0}, {}};
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    CHECK_FALSE(g.resume());  // nothing held
    proximity_test::run(g, s, 400.0);  // Waypoint 2 hold (300 s), then into the final approach
    REQUIRE(g.phase() == ProximityPhase::FinalApproach);
    REQUIRE(g.hold());
    CHECK_FALSE(g.hold());
    const Vec3 held = s.input().port.position_m;
    proximity_test::run(g, s, 300.0);
    CHECK(norm(s.input().port.position_m - held) < 0.5);
    CHECK(norm(s.input().port.velocity_mps) < 0.01);
    REQUIRE(g.resume());
    proximity_test::run(g, s, 30.0);
    REQUIRE(g.abort());
    CHECK_FALSE(g.abort());
    CHECK_FALSE(g.retreat());
    proximity_test::run(g, s, 600.0);
    CHECK(g.phase() == ProximityPhase::Abort);
    CHECK(g.status().find("Ausweichmanöver ausgeführt") != std::string::npos);
    // the breakout leaves the sphere and stays out for a day (CW)
    double closest_after_exit = 1e9;
    bool out = false;
    for (double t = 0.0; t < 86400.0; t += 10.0) {
        s.step({}, 10.0);
        const double r = norm(s.cg.position_m);
        out = out || r > 210.0;
        if (out) closest_after_exit = std::min(closest_after_exit, r);
    }
    CHECK(out);
    CHECK(closest_after_exit > 200.0);
}

TEST_CASE("Retreat is refused at Waypoint 0", "[iss][proximity]") {
    auto s = at_waypoint0(CwSim::ida2());
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, {});
    g.update(s.input());
    CHECK_FALSE(g.retreat());
    CHECK(g.phase() == ProximityPhase::Waypoint);
}

TEST_CASE("Propellant below the deorbit reserve aborts", "[iss][proximity]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 20.0, 0.0, 0.0}, {}};
    s.propellant_kg = p.deorbit_reserve_kg + 0.5;
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    proximity_test::run(g, s, 1200.0);
    CHECK(saw(g, ProximityPhase::Abort));
    CHECK_FALSE(g.contact());
}
```

- [ ] **Step 3: Run to see them fail** (compile error).

- [ ] **Step 4: Implement** `core/src/gnc/proximity_guidance.cpp` following the behaviour list. Private members:
  `DockingApproach approach_; ProximityParameters p_; std::vector<ProximityWaypoint> path_; ProximityPhase phase_ =
  Waypoint; size_t target_ = 0, reached_ = 0; bool started_ = false, waiting_for_go_ = false, auto_hold_ = false;
  ProximityPhase interrupted_; time::Epoch phase_epoch_, arrival_epoch_; std::optional<time::Epoch> waypoint_since_;
  math::Vec3 hold_point_; bool hold_in_port_frame_; math::Vec3 abort_target_lvlh_; bool breakout_done_, breakout_safe_;
  TranslationLoop loop_; std::string status_; std::vector<ProximityEvent> events_; std::optional<ProximityContact>
  contact_;` and helpers `enter(ProximityPhase, std::string status, time::Epoch)`, `target_cg(const ProximityInput&,
  size_t)`, `start_segment(const ProximityInput&)`, `monitor(const ProximityInput&)`, `fly(...)` per phase. Reset
  the loop's hysteresis (`loop_.reset()`) on every phase change. Commands that need the live state (hold point,
  breakout) latch a request flag and are resolved in the next `update` (the command returns true when accepted).

- [ ] **Step 5: Run** `-TestFilter "[proximity]"`; iterate until green. Tuning is allowed only through
  `ProximityParameters` defaults with their tags updated and a ledger ruling (number before / after). Record the
  printed contact numbers, durations and propellant.

- [ ] **Step 6: Commit and push** (`-Fast` first)

```bash
git add core/include/starsim/gnc/proximity_guidance.hpp core/src/gnc/proximity_guidance.cpp core/tests/test_proximity_guidance.cpp core/tests/proximity_test_support.hpp core/CMakeLists.txt core/tests/CMakeLists.txt .superpowers/sdd/2026-10-19-iss-i4-proximity/progress.md
git commit -m "feat(gnc): proximity guidance state machine - waypoints, corridor, hold, retreat, breakout"
git push origin main
```

---

### Task 5: Session integration - `start_proximity` and the per-tick wiring

**Files:**
- Create: `core/src/sim/flight_session_proximity.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (dispatch, `to_string`,
  clearing), `core/CMakeLists.txt`
- Create: `core/tests/test_proximity_session.cpp` (API and refusal cases here; the campaigns in Task 6)
- Modify: `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `gnc::ProximityGuidance` & co. (Tasks 1-4); session internals `set_rendezvous_target`,
  `translation_port_`, `attitude_mode_`, `station_state()`, `controlled_state()`, `vehicle::port_kinematics`,
  `gnc::relative_port_state`, `orbit::lvlh_relative_state`, `frames::lvlh_to_inertial`, `gnc::lvlh_hold`,
  `Vehicle::rcs_pure_force_capability`.
- Produces (public API, used by Task 6 and i6):

```cpp
// flight_session.hpp
enum class AutopilotState { ..., Rendezvous, Proximity };   // to_string: "Annäherung"... keep the style of the
                                                            // existing to_string (English "Proximity" if they are English)
// Proximity operations (ISS i4, flight_session_proximity.cpp): from Waypoint 0 (or any point outside the keep-out
// sphere within ProximityParameters::max_start_range_m) the controlled body flies the waypoint pattern to the station
// port `station_port` on its RCS translation (gnc::ProximityGuidance), in the docking attitude, to contact. Opens the
// nose cone; sets the rendezvous target (chaser port: the controlled vehicle's first docking port). False with a
// German proximity_status() when refused: no station, the station controlled, the autopilot busy (not Off / Orbit),
// a rendezvous other than at Waypoint 0 or a lunar flight under way, no docking port on the vehicle, an unknown or
// unsuitable station port, too far, or inside the sphere without start_waypoint on the axis.
bool start_proximity(const std::string& station_port, gnc::ProximityParameters parameters = {});
bool proximity_active() const { return proximity_.has_value(); }
std::optional<gnc::ProximityPhase> proximity_phase() const;
const std::string& proximity_status() const { return proximity_status_; }
const std::vector<gnc::ProximityEvent>* proximity_events() const;   // nullptr without a flight
std::optional<gnc::ProximityContact> proximity_contact() const;
const std::vector<std::string>& proximity_warnings() const { return proximity_warnings_; }  // station reboosts in the span
bool proximity_hold();     // false (German status) without a flight or when the guidance refuses
bool proximity_retreat();
bool proximity_abort();
bool proximity_resume();
```

- [ ] **Step 1: Write the failing tests** (`core/tests/test_proximity_session.cpp`)

Reuse the i2 fixture pattern (`DragonAtIss` from `test_rendezvous_session.cpp`): move it into
`core/tests/proximity_test_support.hpp` as `proximity_test::DragonNearIss` with a constructor taking a placement
function `(PortKinematics ida2, PortKinematics ida3, StateVector iss) -> StateVector dragon`, and make
`test_rendezvous_session.cpp` use it (no copy). Then:

```cpp
TEST_CASE("start_proximity refusals", "[iss][proximity][session]") {
    test::spice_session();
    // no station
    { proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0, 0, 400}), /*with_station=*/false);
      CHECK_FALSE(d.session.start_proximity("ida2"));
      CHECK(d.session.proximity_status().find("keine Station") != std::string::npos); }
    // too far: 8 km behind
    { proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-8000, 0, 0}));
      CHECK_FALSE(d.session.start_proximity("ida2")); }
    // unknown port
    { proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0, 0, 400}));
      CHECK_FALSE(d.session.start_proximity("pma1"));
      CHECK_FALSE(d.session.proximity_active()); }
    // inside the sphere without an on-axis start
    { proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0, 0, 150}));
      CHECK_FALSE(d.session.start_proximity("ida2")); }
    // commands without a flight
    { proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0, 0, 400}));
      CHECK_FALSE(d.session.proximity_hold());
      CHECK_FALSE(d.session.proximity_resume());
      CHECK_FALSE(d.session.proximity_abort());
      CHECK_FALSE(d.session.proximity_retreat()); }
}

TEST_CASE("start_proximity at Waypoint 0: docking attitude, cone open, ticking", "[iss][proximity][session]") {
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5, 0, 397}), true, /*cone_open=*/false);
    auto& s = d.session;
    REQUIRE(s.start_proximity("ida2"));
    CHECK(s.body(0).vehicle().nose_cone_open());
    CHECK(s.autopilot().state == sim::AutopilotState::Proximity);
    CHECK(*s.proximity_phase() == gnc::ProximityPhase::Waypoint);
    s.advance_to(d.t0 + 240.0, 1000000);  // inside the 300 s hold
    const auto rel = *s.relative_port_state();
    CHECK(rel.misalignment_rad < 1.0 * constants::deg_to_rad);  // turned into the docking attitude
    CHECK(std::abs(rel.clocking_rad) < 1.0 * constants::deg_to_rad);
    CHECK(*s.proximity_phase() == gnc::ProximityPhase::Waypoint);
}

TEST_CASE("Placing a body clears the proximity flight", "[iss][proximity][session]") {
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5, 0, 397}));
    REQUIRE(d.session.start_proximity("ida2"));
    d.session.place_in_orbit(config::load_vehicle(proximity_test::config_dir, "crew_dragon"),
                             {d.session.body(0).state().position_gcrf_m, d.session.body(0).state().velocity_gcrf_mps},
                             d.session.epoch());
    CHECK_FALSE(d.session.proximity_active());
    CHECK(norm(d.session.translation_command()) == 0.0);
}
```
(`autopilot()` and `epoch()` exist on `FlightSession`. `lvlh_from_cg(p, v = {})` returns a placement function: the Dragon CG at LVLH
`p` from the ISS CG with the LVLH relative velocity `v`, i.e. `orbit::chaser_from_lvlh(iss, {p, v})`; the
constructor's further arguments: `with_station = true`, `cone_open = true`.)

- [ ] **Step 2: Run to see them fail** (`-TestFilter "[proximity]"`: compile error).

- [ ] **Step 3: Implement**

`flight_session.hpp` private members:
```cpp
std::optional<gnc::ProximityGuidance> proximity_;
std::string proximity_station_port_, proximity_chaser_port_;
gnc::AxisAcceleration proximity_authority_;  // pure authority along the target port axes in the docking attitude
std::string proximity_status_;
std::vector<std::string> proximity_warnings_;
void run_proximity(Flying& flying);
void reset_proximity();          // with reset_rendezvous / clear_rendezvous_target / reset_mission_state
gnc::ProximityInput proximity_input() const;
math::Quat proximity_docking_attitude() const;
```

`flight_session_proximity.cpp` (header comment: design decisions - docking attitude as an LVLH hold re-derived every
tick; authority measured once at the start in the docking attitude; the guidance runs in the autopilot dispatch,
before the vehicle command of the same tick is built - verify the order in `FlightSession::step`/the tick in
`flight_session.cpp` and note it):

```cpp
bool FlightSession::start_proximity(const std::string& station_port, gnc::ProximityParameters p) {
    const auto refuse = [&](const std::string& reason) { proximity_status_ = reason; return false; };
    if (bodies_.empty() || !station_index_) return refuse("Kein Anflug: keine Station");
    if (controlled_ == *station_index_) return refuse("Kein Anflug: die Station selbst wird gesteuert");
    if (autopilot_.state != AutopilotState::Off && autopilot_.state != AutopilotState::Orbit)
        return refuse("Kein Anflug: Autopilot aktiv");
    if ((rendezvous_step_ != RendezvousStep::None && rendezvous_step_ != RendezvousStep::Waypoint0) ||
        lunar_step_ != LunarStep::None) return refuse("Kein Anflug: ein Flug läuft bereits");
    auto& vehicle = bodies_[controlled_].body->vehicle();
    // chaser port: the vehicle's first docking port (Crew Dragon: idss)
    ... find it in vehicle.stages()[*].config.docking_ports; none -> refuse("Kein Anflug: kein Andockstutzen")
    const auto& station = *bodies_[*station_index_].body;
    vehicle::PortKinematics target;
    try { target = vehicle::port_kinematics(station, station_port); }
    catch (const std::invalid_argument&) { return refuse("Kein Anflug: unbekannter Port " + station_port); }
    const auto st = station_state();
    const math::Mat3 lvlh = frames::lvlh_to_inertial(st).to_matrix();  // LVLH axes in GCRF (columns)
    const math::Mat3 port_to_lvlh = transpose(lvlh) * target.axes_gcrf;
    gnc::DockingApproach approach;
    try { approach = gnc::docking_approach(port_to_lvlh * math::Vec3{1, 0, 0}); }
    catch (const std::invalid_argument& e) { return refuse(std::string("Kein Anflug: ") + e.what()); }
    const auto cg = orbit::lvlh_relative_state(st, controlled_state());
    const double range = norm(cg.position_m);
    if (range > p.max_start_range_m) return refuse("Kein Anflug: zu weit von der Station");
    if (range < p.keep_out_radius_m) {
        // only an on-axis start inside the corridor (tests: Waypoint 2)
        ... compute the chaser port position in the target port frame from port kinematics;
        const auto path = gnc::proximity_path(approach, p);
        if (p.start_waypoint >= path.size() || !path[p.start_waypoint].on_axis ||
            gnc::corridor_status(pos_port, range, p) != gnc::CorridorStatus::Inside)
            return refuse("Kein Anflug: innerhalb der Sicherheitszone");
    }
    if (vehicle.has_nose_cone()) vehicle.set_nose_cone_open(true);
    set_rendezvous_target(*station_index_, chaser_port, station_port);
    proximity_station_port_ = station_port; proximity_chaser_port_ = chaser_port;
    // authority along the target port axes in the docking attitude (constant: the attitude is held in the port frame)
    const math::Quat docking = proximity_docking_attitude();
    const double mass = vehicle.mass_properties().mass_kg;
    for (int axis = 0; axis < 3; ++axis) for (const double sign : {1.0, -1.0}) {
        math::Vec3 u{}; (axis == 0 ? u.x : axis == 1 ? u.y : u.z) = sign;
        const auto c = vehicle.rcs_pure_force_capability(docking.inverse_rotate(target.axes_gcrf * u));
        proximity_authority_.mps2[2 * axis + (sign < 0 ? 1 : 0)] = c.force_n / mass;
    }
    // warnings: station impulses within the next 3 h [E: the proximity flight's span]
    proximity_warnings_.clear();
    for (const auto& impulse : station_impulses_) if (impulse.epoch >= epoch_ && impulse.epoch <= epoch_ + 3.0 * 3600.0)
        proximity_warnings_.push_back("Achtung: Bahnanhebung der ISS um " + german(impulse.delta_v_mps, 2) +
                                      " m/s während des Anflugs");
    rendezvous_step_ = RendezvousStep::None;  // i3 is complete
    proximity_.emplace(approach, p);
    autopilot_.state = AutopilotState::Proximity;
    proximity_status_ = "Anflug gestartet";
    return true;
}
```
`proximity_docking_attitude()`: chaser port axes in the body = `transpose(attitude.to_matrix()) * chaser_kin.axes_gcrf`
(constant), target from `port_kinematics(station, port)`; `gnc::docking_attitude_gcrf(...)`.

`proximity_input()`: `cg_lvlh = orbit::lvlh_relative_state(station_state(), controlled_state())`, `port =
*relative_port_state()`, `port_to_lvlh` as above, `n = sqrt(GM / r^3)` of the station (`constants::earth_gm_m3ps2`),
`authority_port = proximity_authority_`, `propellant_kg = vehicle.rcs_propellant_kg()`.

`run_proximity(Flying&)`: guard (station gone / vehicle impacted -> `reset_proximity()`, status "Anflug abgebrochen:
..."); `const auto out = proximity_->update(proximity_input())`; `translation_port_ = out.translation_fraction_port`;
`attitude_mode_ = out.attitude_hold ? gnc::lvlh_hold(lvlh_q(controlled).conjugate() * proximity_docking_attitude())
: gnc::AttitudeMode{}`; `proximity_status_ = proximity_->status()`; in Contact: `attitude_mode_ = {}`,
`translation_port_ = {}` and `autopilot_.state = AutopilotState::Off` (the flight stays readable:
`proximity_` is kept until cleared). `lvlh_q` = `gnc::lvlh_attitude_gcrf({s.position_gcrf_m, s.velocity_gcrf_mps})`
of the controlled body (the attitude hold's LVLH is about the reference body; near Earth it is the same frame).

Dispatch in `flight_session.cpp`: `case AutopilotState::Proximity: run_proximity(flying); break;`. The coast: the
attitude hold already returns 0; after a completed breakout (`attitude_hold == false`) the session may coast - the
guidance needs no update then (Abort is passive until `resume`, which re-enables the hold on the next tick: make sure
`proximity_resume()` sets `attitude_mode_` to the docking hold immediately so the next `advance_to` ticks).

Clearing: `reset_proximity()` from `clear_rendezvous_target()` and `reset_mission_state()` (check that every path
that clears the target - place_body, launch_from_pad, place_in_orbit, stage, separation, trunk jettison - reaches
one of them); sets `proximity_.reset()`, `translation_port_ = {}`, autopilot Off if it was Proximity, keeps
`proximity_status_` text.

Commands: `proximity_hold()` etc.: `if (!proximity_) { proximity_status_ = "Kein Anflug aktiv"; return false; }`
then forward; on false copy the guidance's status. `proximity_resume()` from Abort or a contact-less waiting state
sets `autopilot_.state = AutopilotState::Proximity` again when it was Off.

- [ ] **Step 4: Run** `-TestFilter "[proximity]"` and `-TestFilter "[rendezvous]"` (the moved fixture); all pass.

- [ ] **Step 5: Commit and push** (`-Fast` first)

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session_proximity.cpp core/src/sim/flight_session.cpp core/CMakeLists.txt core/tests/test_proximity_session.cpp core/tests/proximity_test_support.hpp core/tests/test_rendezvous_session.cpp core/tests/CMakeLists.txt .superpowers/sdd/2026-10-19-iss-i4-proximity/progress.md
git commit -m "feat(sim): proximity operations in the flight session"
git push origin main
```

---

### Task 6: Session validation campaigns

**Files:**
- Modify: `core/tests/test_proximity_session.cpp`

**Interfaces:**
- Consumes: the Task 5 API; `proximity_test::DragonNearIss`; `gnc::capture_envelope_check`.
- Produces: validation numbers for Task 7.

All cases print `[validation]` lines (phase timeline with minutes, propellant, contact state) and record the numbers in
the ledger. Cases above ~40 s get `[slow]`.

- [ ] **Step 1: Write the campaign tests**

```cpp
namespace {
// Runs the session in 10 s calls until contact, abort-coast or `limit_s`; returns the elapsed seconds and checks the
// corridor every call: inside the keep-out sphere the chaser port must be inside the corridor.
double fly(proximity_test::DragonNearIss& d, double limit_s, bool check_corridor = true);
}

TEST_CASE("Proximity to IDA-2 from Waypoint 0", "[iss][proximity][session][validation][slow]") {
    // Dragon at i3's measured Waypoint 0 arrival (LVLH (-5.0, 0.0, 396.9) m, 0.017 m/s)
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5.0, 0.0, 396.9}, {0.012, 0.0, -0.012}));
    REQUIRE(d.session.start_proximity("ida2"));
    const double kg0 = d.session.body(0).vehicle().rcs_propellant_kg();
    const double t = fly(d, 3.0 * 3600.0);
    const auto c = d.session.proximity_contact();
    REQUIRE(c);
    // print: every event (minutes, phase, status), contact axial / lateral / offset / angle / rate, propellant
    CHECK(c->envelope.inside);
    CHECK(-c->port.velocity_mps.x < 0.1);  // M9 criterion
    CHECK(kg0 - d.session.body(0).vehicle().rcs_propellant_kg() < 150.0);  // [E] bound, i3 reserve 900 kg incl. proximity
}

TEST_CASE("Proximity to IDA-3 with the fly-around", "[iss][proximity][session][validation][slow]") {
    // same, "ida3"; also CHECK that a Flyaround event exists
}

TEST_CASE("Final approach campaign with dispersed Waypoint 2 states", "[iss][proximity][session][validation][slow]") {
    // both ports x 4 dispersions [E]: lateral offset (+0.5, 0) / (0, -0.5) m, lateral velocity +-0.01 m/s, axial
    // velocity +-0.02 m/s, initial attitude 2 deg off the docking attitude (rotate the placed attitude about body y).
    // start_waypoint = 2, waypoint2_hold_s = 60 (shortened [E] for the campaign: the dispersion settles in it).
    // Every contact inside the envelope and < 0.1 m/s; print a table: case, axial, lateral, offset, angle, rate, kg.
}

TEST_CASE("From the i3 Waypoint 0 state with the braking attitude", "[iss][proximity][session][validation][slow]") {
    // Review focus 1: the Dragon placed with its nose 180 deg from the docking attitude (as the braking burn leaves
    // it) and drifting 0.02 m/s; start_proximity. CHECK: misalignment < 2 deg before the Waypoint 0 hold ends (sample
    // every 10 s; the hold law may translate meanwhile - that is allowed), the phase stays Waypoint for the whole
    // 300 s hold, the chaser CG stays within 10 m of Waypoint 0 during it, and the contact is inside the envelope.
}

TEST_CASE("A station reboost during the transfer is absorbed", "[iss][proximity][session][validation][slow]") {
    // Review focus 2: add_station with an extra synthetic impulse of 0.3 m/s along the velocity 600 s into the
    // Waypoint 0 -> 1 transfer (epoch = t0 + waypoint0_hold_s + 600). CHECK proximity_warnings() has one entry, the
    // flight reaches Waypoint 1 (event) and contact inside the envelope; no Abort event.
}

TEST_CASE("Time warp at a waypoint hold keeps ticking", "[iss][proximity][session]") {
    // Review focus 4: start_waypoint = 1 placed at IDA-2 Waypoint 1 (220 m on the axis); one advance_to(t0 + 290 s)
    // call; CHECK the relative position stays within 2 m of Waypoint 1 and the phase is Waypoint.
}

TEST_CASE("Abort from the final approach: breakout, passive drift outside the sphere", "[iss][proximity][session][validation]") {
    // start_waypoint = 2 at IDA-2, waypoint2_hold_s = 30; fly until x < 10 m; proximity_abort(); fly until the
    // breakout is done (status "Ausweichmanöver ausgeführt"); then advance_to(+24 h) in 10 min calls (coasting) and
    // CHECK the CG range after leaving the sphere stays > 200 m; print the breakout dv and the closest range.
}

TEST_CASE("Hold and resume in the final approach", "[iss][proximity][session]") {
    // start_waypoint = 2, hold at x ~ 12 m for 120 s: |velocity| < 0.01 m/s, drift < 0.5 m; resume -> contact inside.
}
```

Write the bodies in full (the comments state the setup and checks exactly). `fly()`:

```cpp
double fly(proximity_test::DragonNearIss& d, double limit_s, bool check_corridor) {
    auto& s = d.session;
    const gnc::ProximityParameters p;
    double t = 0.0;
    for (; t < limit_s; t += 10.0) {
        s.advance_to(d.t0 + t + 10.0, 1000000);  // NOTE: t0 + elapsed, not epoch()+10: keeps calls deterministic
        if (s.proximity_contact()) break;
        const auto phase = s.proximity_phase();
        if (!phase || (*phase == gnc::ProximityPhase::Abort &&
                       s.proximity_status().find("ausgeführt") != std::string::npos)) break;
        if (check_corridor) {
            const auto rel = *s.relative_port_state();
            const double cg_range = norm(s.body(0).state().position_gcrf_m - s.body(1).state().position_gcrf_m);
            if (cg_range < p.keep_out_radius_m) {
                CHECK(gnc::corridor_status(rel.position_m, cg_range, p) != gnc::CorridorStatus::Retreat);
            }
        }
    }
    return t;
}
```
(The 10 s granularity only samples; the guidance itself runs every 5 ms tick. The contact record is the exact
crossing tick.)

- [ ] **Step 2: Run** `-TestFilter "[proximity]"` (includes `[slow]` here). Debug failures with
  `superpowers:systematic-debugging`; add `STARSIM_DEBUG_PROXIMITY=1` diagnostics to `flight_session_proximity.cpp`
  (read once at construction like `rendezvous_debug_`; print every event and, every 60 s, the port-frame position,
  velocity and propellant) - keep them.
  Tuning only through `ProximityParameters` defaults (tags + ledger ruling). If the session results differ from the CW
  test of Task 4 by more than the pulse effects (e.g. coupled torque, attitude errors), record why in the ledger.

- [ ] **Step 3: Full suite and smoke test** (this task touches a flight session):

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1` (with bindings, full ctest) and the Godot smoke test
`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd`.
Expected: all pass (`AutopilotState` gained a value: check bindings / GDScript `match` on it -
`grep -rn "Rendezvous" bindings game` - and add "Proximity" where states are listed).

- [ ] **Step 4: Commit and push**

```bash
git add core/tests/test_proximity_session.cpp core/src/sim/flight_session_proximity.cpp core/include/starsim/sim/flight_session.hpp .superpowers/sdd/2026-10-19-iss-i4-proximity/progress.md
git commit -m "test(sim): proximity campaigns - both ports, dispersed final approach, abort, hold, reboost"
git push origin main
```

---

### Task 7: Documentation and final review

**Files:**
- Modify: `docs/subsystems.md` (new subsection "### Proximity operations (i4)" after "Phasing (i3)"), `CLAUDE.md`
  (module list: `gnc` gains "proximity guidance: waypoints, corridor, hold / retreat / breakout, final approach";
  `sim` gains "proximity flight to a station port"), `docs/superpowers/specs/2026-10-19-iss-docking-design.md` (status
  line: i4 implemented; the five corrections of this plan as a "Corrections" block at the top of the i4 section, as
  for i3), `docs/ansichten.md` (one pointer line: proximity getters `proximity_phase` / `proximity_status` /
  `relative_port_state` for the later docking view - only if the file has a getter list; else skip).

- [ ] **Step 1: Write `docs/subsystems.md` "Proximity operations (i4)"** in the style of the i3 section: plan and
  ledger paths; profile and sources (the research list of this plan); the five corrections; geometry (paths, KOS,
  corridor, fly-around); the transfer (CW re-solve, the CW-vs-full-model number from Task 2); the translation loop
  (deadbands) and approach laws; the state machine and monitors (German names); the session (`start_proximity`
  refusals, docking attitude, authority measured once, warnings, clearing, coasting after a breakout); validation
  numbers from Tasks 2, 4, 6 (timelines, propellant, contact tables, breakout); diagnostics
  `STARSIM_DEBUG_PROXIMITY`; open points and deferred minors.

- [ ] **Step 2: Fast suite** `-Fast`, then commit and push

```bash
git add docs/subsystems.md CLAUDE.md docs/superpowers/specs/2026-10-19-iss-docking-design.md docs/ansichten.md .superpowers/sdd/2026-10-19-iss-i4-proximity/progress.md
git commit -m "docs: ISS i4 proximity operations"
git push origin main
```

- [ ] **Step 3: Final review** with `superpowers:requesting-code-review` (a fresh reviewer on the most capable model)
  over the i4 commits (`git log --oneline ede6c54..HEAD`), with the spec, this plan and the ledger. Apply the fix pass
  (`fix(gnc|sim): proximity review fixes - ...`), full suite + smoke test, push. Update memory
  `project-iss-docking` (i4 COMPLETE, sha, open points, NEXT: writing-plans i5), then ask the user for `/clear`.
