# Refuelling r5: reference flights - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Three missions fly the refuelling chain end to end in the full model and measure the numbers the r6
campaign calculator needs: `depot_launch` (the depot from the Starbase pad into its orbit, booster caught, the ship
becomes the station), `tanker_flight` (launch window -> ascent with booster catch -> phasing -> proximity -> docking ->
transfer down to the computed reserve -> undocking and departure -> deorbit -> entry -> flip and catch; plus an orbit
start near the depot for fast tests) and `hls_refuel` (the HLS launches into the depot plane, docks, is filled,
undocks, and the existing lunar landing mission continues from its **real** state and tank load).

**Architecture:** The ISS mission sequence (`IssMissionSequence`, `flight_session_iss_mission.cpp`) is the chain and
gets the missing legs: Transfer (Docked -> start_transfer -> undock), the ship return (Departure -> `plan_ship_return`
-> caught), WaitContinue (Departure -> a continuation at a set epoch: the lunar mission), and the "ascent to station"
end (the inserted depot becomes the station). Three session building blocks make that possible:
`FlightSession::make_station` (a flown body becomes the station), `FlightSession::release_tower` (the caught booster
is lifted off the tower before the ship's catch) and the leg marks (mass and propellant at every leg change: the
measured numbers). Mission configs and loaders live in a new `config/refuelling_mission_config`, the start functions
in a new `sim/refuelling_mission`. The lunar landing start is split into placement + `continue_lunar_landing_mission`
(from the live vehicle). The HLS gets a Super Heavy launch stack and thermal data; the radiation environment gains the
Moon's albedo and infrared (promised to r5 by r2's correction 3).

**Tech Stack:** C++20 core (float64, SI), Catch2, toml++ loader (`detail::Document`), CSPICE through the `spice`
adapters (tests).

**Spec:** `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md` (sections "r5 design", "Error handling",
"Testing", "Documentation"; r1-r4 corrections). Notes: `docs/subsystems.md` "Orbital refuelling (milestone r)" (r1-r4),
"ISS missions, launch window and game interface (i6)". Config conventions: `config/README.md`.

## Plan decisions that need the user's approval

1. **One chain, generalised in place:** `IssMissionSequence` keeps its name (renaming touches the bindings and the
   ISS tests for no behaviour) and gets the new legs `Transfer` and `WaitContinue` plus four optional fields
   (`ascent_to_station`, `transfer`, `ship_return`, `continue_epoch` + `continuation`). Without them every ISS path is
   unchanged (Docked stays terminal for the Dragon).
2. **The depot of `tanker_flight` / `hls_refuel` is placed, not flown in the same world time:** a synthetic orbit from
   the mission's `[depot]` block (200 km, 26.5 deg, RAAN / argument of latitude at `earliest_utc`, fill in t),
   advanced by the J2 rates (`orbit::advance_mean_elements`, as the synthetic ISS). The flown chain depot -> N tankers
   -> HLS in one continuous time is the spec's "fully flown campaign" (out of scope). `depot_launch` measures the
   depot's initial fill; the tests compare it with the configured `[test]` fills only for the record.
3. **The flown depot becomes the station** (`make_station`): when the depot ship reaches orbit (autopilot Orbit, one
   stage left) it is switched to the ideal LVLH attitude (nose along +V-bar, drogues to nadir) at once - the turn is
   not flown (documented simplification, as the station attitude itself). The booster flies on to the catch.
4. **The caught booster leaves the tower** (`release_tower`): before the tanker's deorbit burn the caught booster
   body is removed from the session and the tower re-armed (real boosters are lifted onto the mount / transported
   within hours; the tanker returns days later). Without it the arms stay closed on the booster and the ship cannot be
   caught. Refused while the booster is the controlled body, the station or a rendezvous target.
5. **Uncrewed phasing limit 48 h [E]** for tanker and HLS (the crew's 28 h opened the first Starbase window only after
   177 h in r3); `search_days` 12. The window found is reported (finding for the r6 cadence), not tuned.
6. **Tanker and HLS ascents to 170 x 190 km** (r3's phasing orbit below the 200 km depot), Starship's pitch program of
   `starship_leo`. The launch window's separation numbers (`insertion_s`, `insertion_arc_deg`) are calibrated [D]
   from the flown tanker ascent (Task 5), as i6 calibrated the Falcon 9 (535 s / 15.0 deg); the HLS uses its own
   calibration from its flown ascent (Task 8).
7. **HLS launch stack** `starship_hls_launch` = Block 3 booster + the HLS stage (`from` reuse, r1 rule). **HLS
   thermal [E]:** the depot's insulated hull values (absorptivity 0.30, emissivity 0.80, wall resistance 63.7 m^2K/W
   [D], ullage resistance 100) - a long-duration ship like the depot; bare Block 3 steel would lose ~30 t/day full
   and no lunar mission would close. HLS fill: the depot fills the HLS's main tanks up to full (or down to the depot's
   floor 0 [E]).
8. **The HLS lunar continuation:** the depot orbit lies in the lunar mission's parking plane (the RAAN that
   `parking_orbit_raan` gives at the lunar mission's epoch for 26.5 deg, converted to the true-equator frame and run
   back by J2 to the HLS launch). The HLS launches into that plane before the lunar epoch, refuels and departs; at the
   lunar epoch `continue_lunar_landing_mission` plans the outbound flight from its live state and closes the budget
   with its real mass. New config `hls_lunar_llo_depot` = `hls_lunar_llo` with 26.5 deg and an epoch whose Moon
   declination at arrival is below 26.5 deg (found in Task 8). Refused (German) when the HLS is not ready by then.
9. **Moon radiation [E] in the thermal environment:** albedo 0.11 [S: Vasavada et al. 2012, Diviner] x solar flux x
   view factor x illumination phase at the sub-vehicle point (the same "cap like its centre" approximation as Earth's);
   infrared from the sub-vehicle surface in radiative equilibrium, `max((1 - A) S cos z, sigma T_night^4)` with
   T_night 95 K [S: Diviner]. Below `boil_off_min_altitude_m` over the Earth only; on the lunar surface the HLS vents
   with the same environment (documented). The Block 3 lunar returns (`ship_lunar_return_*`) keep `boil_off = false`:
   Task 8 reports what the flag would cost them, the user decides.

## Global Constraints

- SI units internally, angles in radians; frame and unit in names (`propellant_kg`, `continue_epoch`).
- No magic numbers in code: physical constants in `core/include/starsim/math/constants.hpp` with source; vehicle numbers
  in `config/` with provenance tags [S] / [E] / [D]; model estimates as commented `[E]` defaults in their parameter
  struct.
- C++ sources UTF-8; user-facing strings German, code and comments English.
- Modules depend downwards only: `config` never includes `sim`; `mission` never includes `sim`; `sim` may use all.
  `FlightSession` must not include the mission start files (`refuelling_mission.hpp`, `lunar_landing_mission.hpp`):
  the lunar continuation reaches it as a `std::function`.
- The ISS / Crew Dragon behaviour must not change: tags `[iss]`, `[iss_mission]`, `[docking]`, `[proximity]` stay green
  including their `[slow]` cases (Task 1 and 2 touch the sequence and the body list).
- Physics over gauges: measured numbers are reported, never tuned towards public figures (spec "Testing").
- ctest runs cases in parallel: tests must not share temp files or directories (unique `starsim_r5_<name>` prefixes).
- Test workflow: run only the tags the change affects, `[slow]` cases of those tags included. Build with
  `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests > tmp/r5_build.txt 2>&1`, then run the
  labels with `ctest --test-dir build -L "<regex>" -j 24 --output-on-failure > tmp/r5_tN.txt 2>&1` and read the tail
  (CTest labels = Catch tags). A single new case for RED: `build/core/tests/starsim_core_tests.exe "<name>"`. Runs that
  may exceed 10 min go to the background, checked every 10 min, stopped early on failures. New cases above ~40 s get
  `[slow]`.
- Commit on `main`, Conventional Commits, push after each task once its tests are green. Trailer:
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_012biTJd57gGHxfvAHbX6f2T`.

## Review Focus

1. **The transfer ends in a pause or a stop (settling gas gone, `max_pause_s` exceeded):** the tanker must not stay
   docked forever nor undock with a running transfer - the sequence ends with the transfer's German status ("Mission
   beendet: Transfer abgebrochen: ...") - Task 2 test "a stopped transfer ends the mission".
2. **The deorbit plan is refused after the transfer** (the reserve does not cover the burn plus boil-off at the
   r2 commit margin, or no pass in 8 days): German end with the planner's reason, no burn - Task 2 test "ship return
   refused ends the mission".
3. **The HLS is not ready at the lunar epoch** (window late, docking retried): the continuation refuses ("zu spät"),
   the mission ends, nothing is re-placed - Task 8 test "continuation after the lunar epoch is refused".
4. **`release_tower` with indices behind the erased booster:** the controlled tanker, the station index and the
   rendezvous target must still point at the same bodies - Task 1 test "release_tower keeps the indices".
5. **Time warp through the new legs:** WaitContinue and the ship-return wait must coast (no 200 Hz ticking for days)
   and still stop exactly at their epochs - Task 2 test "WaitContinue coasts to its epoch".

---

### Task 1: Session building blocks - make_station, release_tower, leg marks

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp` (public: `make_station`, `release_tower`, `MissionMark`,
  `iss_mission_marks`; private: `iss_marks_`, `erase_body`)
- Modify: `core/src/sim/flight_session.cpp` (`add_station` attitude helper shared with `make_station`; `make_station`,
  `release_tower`, `erase_body`)
- Modify: `core/src/sim/flight_session_iss_mission.cpp` (`iss_enter` appends a mark; `set_iss_mission` clears them)
- Create: `core/tests/test_refuelling_missions.cpp`; Modify: `core/tests/CMakeLists.txt` (after
  `test_refuelling_transfer.cpp`)

**Interfaces:**
- Produces:
  ```cpp
  // A flown body becomes the station (refuelling r5: the depot after its ascent): one stage with [stage.station], no
  // station yet, the body not docked; its attitude switches to the ideal LVLH hold at once (the turn is not flown).
  // The autopilot is disengaged when it is the controlled body. Throws std::logic_error otherwise.
  size_t make_station(size_t body_index);
  // The caught booster is lifted off the tower (refuelling r5): every caught body that is neither the controlled one,
  // the station nor the rendezvous target is removed, the tower re-armed (arms open, no load). False (nothing changed)
  // without a tower in phase Caught.
  bool release_tower();
  struct MissionMark {
      IssMissionLeg leg;                          // the leg entered
      time::Epoch epoch;
      double chaser_mass_kg = 0.0;                // the controlled body
      double chaser_main_propellant_kg = 0.0;
      double chaser_rcs_supply_kg = 0.0;          // Vehicle::rcs_supply_kg
      double station_main_propellant_kg = 0.0;    // 0 without a station
  };
  const std::vector<MissionMark>& iss_mission_marks() const;
  ```

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_refuelling_missions.cpp`:

```cpp
// Refuelling r5: the reference flights - session building blocks, the mission chain, the three missions.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "refuelling_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;
using sim::IssMissionLeg;

namespace {

constexpr double deg = constants::deg_to_rad;
bool verbose() { return std::getenv("STARSIM_VERBOSE") != nullptr; }

// The depot as an ordinary ship body in its orbit (not yet a station), nose along the velocity.
void place_depot_ship(refuelling_test::DepotSession& f) {
    f.session.place_in_orbit(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
}

}  // namespace

TEST_CASE("make_station turns a flown depot into the station with the ideal attitude", "[refuelling][station]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    place_depot_ship(f);
    REQUIRE_FALSE(f.session.station_index());
    REQUIRE(f.session.make_station(0) == 0);
    REQUIRE(f.session.station_index() == 0u);
    CHECK(f.session.autopilot().state == sim::AutopilotState::Off);
    // the same attitude add_station gives the same config at the same state
    refuelling_test::DepotSession g;
    g.session.add_station(refuelling_test::depot_config(g.depot_propellant_kg), g.depot, g.t0);
    f.session.advance_to(f.t0 + 60.0);
    g.session.advance_to(g.t0 + 60.0);
    const auto qa = f.session.body(0).state().attitude, qb = g.session.body(0).state().attitude;
    CHECK(std::abs(std::abs(qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z) - 1.0) < 1e-12);
    CHECK(norm(f.session.body(0).state().position_gcrf_m - g.session.body(0).state().position_gcrf_m) < 1e-3);
}

TEST_CASE("make_station refuses a stack, a second station and a vehicle without [stage.station]",
          "[refuelling][station]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(config::load_vehicle(refuelling_test::config_dir, "starship_depot"), f.depot, f.t0);
    CHECK_THROWS_AS(f.session.make_station(0), std::logic_error);  // booster + depot
    refuelling_test::DepotSession g;
    g.session.place_in_orbit(refuelling_test::tanker_config(200.0e3), g.depot, g.t0);
    CHECK_THROWS_AS(g.session.make_station(0), std::logic_error);  // no [stage.station]
    refuelling_test::DepotSession h;
    place_depot_ship(h);
    h.session.make_station(0);
    CHECK_THROWS_AS(h.session.make_station(0), std::logic_error);  // already the station
}

TEST_CASE("release_tower refuses without a caught body", "[refuelling][catch]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    place_depot_ship(f);
    CHECK_FALSE(f.session.release_tower());  // no tower
    f.session.set_launch_site(sim::launch_site_from_config(
        config::load_launch_site(refuelling_test::config_dir, "starbase_olp1")));
    CHECK_FALSE(f.session.release_tower());  // tower Open
    CHECK(f.session.body_count() == 1u);
}

TEST_CASE("release_tower keeps the indices: booster caught, then lifted off; the ship flies on",
          "[refuelling][catch][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto m = config::load_ascent_mission(refuelling_test::config_dir, "starship_leo");
    sim::start_ascent_mission(f.session, m, refuelling_test::config_dir, f.t0);
    while (f.session.epoch() - f.t0 < 1200.0 &&
           !(f.session.tower() && f.session.tower()->phase() == sim::CatchPhase::Caught)) {
        f.session.advance_to(f.session.epoch() + 1.0);
    }
    REQUIRE(f.session.tower()->phase() == sim::CatchPhase::Caught);
    REQUIRE(f.session.body_count() == 2u);
    const size_t ship = f.session.controlled_index();
    const double ship_mass = f.session.body(ship).vehicle().mass_properties().mass_kg;
    // a station behind the booster in the list
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg),
                          refuelling_test::depot_state(f.earth, f.session.epoch()), f.session.epoch());
    REQUIRE(f.session.release_tower());
    CHECK(f.session.body_count() == 2u);
    CHECK(f.session.tower()->phase() == sim::CatchPhase::Open);
    CHECK(f.session.body(f.session.controlled_index()).vehicle().mass_properties().mass_kg == Catch::Approx(ship_mass));
    REQUIRE(f.session.station_index());
    CHECK(f.session.body(*f.session.station_index()).vehicle().config().id == "starship_depot");
    CHECK_FALSE(f.session.release_tower());
    f.session.advance_to(f.session.epoch() + 60.0);  // runs on without the booster
}

TEST_CASE("Leg marks record the chaser's mass and propellant at every leg change", "[refuelling][iss_mission]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0});
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    sim::IssMissionSequence q;
    q.station_port = "transfer_dock";
    q.undock_epoch = f.session.epoch() + 30.0;
    f.session.set_iss_mission(q);
    const auto& marks = f.session.iss_mission_marks();
    REQUIRE(marks.size() == 1u);
    CHECK(marks[0].leg == IssMissionLeg::WaitUndock);
    CHECK(marks[0].chaser_main_propellant_kg == Catch::Approx(200.0e3).epsilon(1e-6));
    CHECK(marks[0].station_main_propellant_kg == Catch::Approx(600.0e3).epsilon(1e-6));
    f.fly([&] { return f.session.iss_mission_leg() == IssMissionLeg::Departure; }, 120.0);
    REQUIRE(marks.size() == 2u);
    CHECK(marks[1].leg == IssMissionLeg::Departure);
    CHECK(marks[1].epoch - marks[0].epoch == Catch::Approx(30.0).margin(0.01));
}
```

Notes for the implementer: `vehicle().config().id` - if `Vehicle` has no `config()` accessor, compare
`stages().front().config.id == "depot"` instead (the depot stage id). Check the `CatchPhase` namespace
(`sim::CatchPhase` in `catch_tower_sim.hpp`).

Add `test_refuelling_missions.cpp` to `core/tests/CMakeLists.txt` after `test_refuelling_transfer.cpp`.

- [ ] **Step 2: Run to verify they fail**

Build; expected: compile errors (`make_station`, `release_tower`, `iss_mission_marks` unknown).

- [ ] **Step 3: Implement**

In `flight_session.hpp`, next to `add_station` (public), add the `make_station` and `release_tower` declarations
above; next to `iss_mission()` the `MissionMark` struct and `iss_mission_marks()`. Private: `std::vector<MissionMark>
iss_marks_;` beside `iss_status_`, and `void erase_body(size_t index);`.

In `flight_session.cpp`, factor the attitude out of `add_station`:

```cpp
namespace {
// The station's attitude in LVLH from [stage.station] body_in_lvlh_deg (aerospace 3-2-1: yaw, pitch, roll).
math::Quat station_body_in_lvlh(const config::StageConfig& stage) {
    const math::Vec3 ypr = *stage.body_in_lvlh_rad;
    return math::Quat::from_axis_angle({0.0, 0.0, 1.0}, ypr.x) * math::Quat::from_axis_angle({0.0, 1.0, 0.0}, ypr.y) *
           math::Quat::from_axis_angle({1.0, 0.0, 0.0}, ypr.z);
}
}  // namespace
```

(`add_station` calls `body->set_ideal_lvlh_attitude(station_body_in_lvlh(config.stages.back()))`.)

```cpp
size_t FlightSession::make_station(size_t body_index) {
    if (body_index >= bodies_.size()) throw std::logic_error("make_station: no such body");
    if (station_index_) throw std::logic_error("make_station: the session has a station already");
    auto& body = *bodies_[body_index].body;
    const auto& stages = body.vehicle().stages();
    if (stages.size() != 1) throw std::logic_error("make_station: a station is one stage (stage the booster first)");
    if (!stages.back().config.body_in_lvlh_rad) throw std::logic_error("make_station: vehicle has no [stage.station]");
    if (body.docked()) throw std::logic_error("make_station: the body is docked");
    if (body_index == controlled_) disengage_autopilot();
    body.set_ideal_lvlh_attitude(station_body_in_lvlh(stages.back().config));
    station_index_ = body_index;
    return body_index;
}
```

Check while implementing: `set_ideal_lvlh_attitude` must set the attitude from the current state (the station path
starts from `Quat{}`; if it only takes effect on the next step, the first `advance_to` applies it - the test's 60 s
comparison covers both). If `disengage_autopilot` resets more than the autopilot (e.g. guidance of other bodies),
set `autopilot_.state = AutopilotState::Off` instead and keep the booster's `return_guidance` untouched.

`release_tower` and the index fix-up:

```cpp
bool FlightSession::release_tower() {
    if (!tower_ || tower_->phase() != CatchPhase::Caught) return false;
    for (size_t i = bodies_.size(); i-- > 0;) {
        const auto& body = *bodies_[i].body;
        const bool kept = i == controlled_ || (station_index_ && i == *station_index_) ||
                          (rendezvous_ && i == rendezvous_->target_index);
        if (body.caught() && !kept) erase_body(i);
    }
    // re-armed as set_launch_site builds it: arms open, no tracks, no load
    tower_ = std::make_unique<CatchTowerSim>(*launch_site_.tower, launch_site_.latitude_rad, launch_site_.longitude_rad,
                                             launch_site_.height_m);
    return true;
}

// Removes body `index` and shifts every stored body index behind it (refuelling r5: the lifted booster).
void FlightSession::erase_body(size_t index) {
    const auto shift = [index](size_t& i) {
        if (i > index) --i;
    };
    bodies_.erase(bodies_.begin() + static_cast<std::ptrdiff_t>(index));
    shift(controlled_);
    if (station_index_) shift(*station_index_);
    if (rendezvous_) shift(rendezvous_->target_index);
}
```

While implementing, grep `flight_session*.cpp` and `flight_session.hpp` for other stored body indices
(`size_t .*index`, `docked_`, transfer `delivering` / `receiving`, `legs_` tracks keyed by `FlightBody*`) and shift
or clear them in `erase_body`: pointer-keyed tracks of the leg sim must drop the erased body (add a
`LegLandingSim::forget(const vehicle::FlightBody*)` if it keeps per-body state). A transfer's indices: `release_tower`
is only called after the undocking, but shift them anyway if the transfer state stores indices.

In `flight_session_iss_mission.cpp`: `set_iss_mission` clears `iss_marks_` first; `iss_enter` appends

```cpp
void FlightSession::iss_enter(IssMissionLeg leg, std::string status) {
    iss_leg_ = leg;
    iss_status_ = std::move(status);
    if (bodies_.empty() || controlled_ >= bodies_.size()) return;
    const auto& v = bodies_[controlled_].body->vehicle();
    MissionMark mark{leg, epoch_, v.mass_properties().mass_kg, v.main_propellant_kg(), v.rcs_supply_kg(), 0.0};
    if (station_index_) mark.station_main_propellant_kg = bodies_[*station_index_].body->vehicle().main_propellant_kg();
    iss_marks_.push_back(mark);
}
```

and `iss_mission_marks()` returns `iss_marks_`.

- [ ] **Step 4: Run the tests**

`ctest --test-dir build -L "refuelling|iss_mission|station|catch" -j 24 --output-on-failure > tmp/r5_t1.txt 2>&1`.
Expected: all pass (the `[slow]` release case included).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/src/sim/flight_session_iss_mission.cpp core/tests/test_refuelling_missions.cpp core/tests/CMakeLists.txt
git commit -m "feat(sim): a flown body becomes the station, the caught booster leaves the tower, leg marks"
git push origin main
```

---

### Task 2: The chain's new legs - transfer, ship return, continuation, ascent to station

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp:116-132` (`IssMissionLeg`, `IssMissionSequence`)
- Modify: `core/src/sim/flight_session_iss_mission.cpp` (legs, pending conditions, actions)
- Modify: `core/src/sim/flight_session.cpp` (`coast_allowance_s`: WaitContinue limited to its epoch)
- Test: `core/tests/test_refuelling_missions.cpp`

**Interfaces:**
- Consumes: Task 1 (`make_station`, `release_tower`, marks); r4 `tanker_transfer_request`, `start_transfer`,
  `transfer_state`; `plan_ship_return`.
- Produces:
  ```cpp
  enum class IssMissionLeg { None, Ascent, Rendezvous, Proximity, Docking, Docked, Transfer, WaitUndock, Departure,
                             Return, WaitContinue, Complete, Ended };
  enum class MissionTransfer { TankerReserve, FillChaser };  // tanker -> station down to the reserve; station -> chaser up to full
  struct IssMissionSequence {
      // ... existing fields unchanged ...
      bool ascent_to_station = false;                    // depot_launch: Ascent -> make_station -> Complete
      std::optional<MissionTransfer> transfer;           // Docked -> Transfer -> undock -> Departure
      std::optional<gnc::DeorbitParameters> ship_return; // Departure -> release_tower + plan_ship_return -> Return
      std::optional<time::Epoch> continue_epoch;         // Departure -> WaitContinue until then -> continuation
      std::function<std::string(FlightSession&)> continuation;  // German refusal, empty: it took over -> Complete
  };
  ```
  German leg names: Transfer "Treibstofftransfer", WaitContinue "Warten auf Fortsetzung".

- [ ] **Step 1: Write the failing tests**

Append to `core/tests/test_refuelling_missions.cpp` (anonymous namespace helper first):

```cpp
namespace {

// The docked pair of r4 (tanker 200 t at the depot 600 t), Starbase as the return site.
struct DockedAtDepot : refuelling_test::TankerAtPort {
    explicit DockedAtDepot(const config::VehicleConfig& chaser = refuelling_test::tanker_config(200.0e3))
        : TankerAtPort({0.0, 0.0, 0.0}, {}, chaser) {
        session.set_launch_site(sim::launch_site_from_config(
            config::load_launch_site(refuelling_test::config_dir, "starbase_olp1")));
        session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
        session.dock_in_place();
    }
    sim::IssMissionSequence tanker_sequence() const {
        sim::IssMissionSequence q;
        q.station_port = "transfer_dock";
        q.transfer = sim::MissionTransfer::TankerReserve;
        gnc::DeorbitParameters p;
        p.search_window_s = sim::FlightSession::tanker_reserve_search_s;
        q.ship_return = p;
        return q;
    }
    IssMissionLeg leg() const { return session.iss_mission_leg(); }
};

}  // namespace

TEST_CASE("Docked tanker: transfer down to the reserve, undock, depart, deorbit planned", "[refuelling][iss_mission]") {
    test::spice_session();
    DockedAtDepot f;
    f.session.set_iss_mission(f.tanker_sequence());
    REQUIRE(f.leg() == IssMissionLeg::Transfer);  // a docked start with a transfer begins with it
    f.fly([&] { return f.leg() == IssMissionLeg::Departure || f.leg() == IssMissionLeg::Ended; }, 3600.0, 10.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.leg() == IssMissionLeg::Departure);
    CHECK(f.session.transfer_state().phase == sim::TransferPhase::Complete);
    CHECK(f.session.transfer_state().delivered_kg[0] > 140.0e3);  // r4: 144.6 t LOX
    f.fly([&] { return f.leg() != IssMissionLeg::Departure; }, 4.0 * 3600.0, 10.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.leg() == IssMissionLeg::Return);
    REQUIRE(f.session.ship_return_plan());
    CHECK(f.session.ship_return_plan()->feasible);
    const auto& marks = f.session.iss_mission_marks();
    REQUIRE(marks.size() >= 3u);
    CHECK(marks.front().leg == IssMissionLeg::Transfer);
    if (verbose()) {
        for (const auto& m : marks) {
            std::printf("[r5] %-24s +%8.0f s  main %.1f t  station %.1f t\n", sim::to_string(m.leg),
                        m.epoch - marks.front().epoch, m.chaser_main_propellant_kg / 1e3,
                        m.station_main_propellant_kg / 1e3);
        }
    }
}

TEST_CASE("Docked tanker: the ship return flies to the catch", "[refuelling][iss_mission][ship][slow]") {
    test::spice_session();
    DockedAtDepot f;
    f.session.set_iss_mission(f.tanker_sequence());
    f.fly([&] { return f.leg() == IssMissionLeg::Complete || f.leg() == IssMissionLeg::Ended; },
          10.0 * 86400.0, 600.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.leg() == IssMissionLeg::Complete);
    CHECK(f.session.body(f.session.controlled_index()).caught());
    const auto& last = f.session.iss_mission_marks().back();
    std::printf("[r5] tanker caught: main tanks %.2f t, headers %.2f t\n", last.chaser_main_propellant_kg / 1e3,
                (f.session.body(f.session.controlled_index()).vehicle().propellant_kg() -
                 last.chaser_main_propellant_kg) / 1e3);
}

TEST_CASE("A stopped transfer ends the mission", "[refuelling][iss_mission]") {
    test::spice_session();
    DockedAtDepot f;
    f.session.set_iss_mission(f.tanker_sequence());
    f.fly([&] { return f.session.transfer_state().phase == sim::TransferPhase::Flowing; }, 600.0, 10.0);
    f.session.stop_transfer();
    f.fly([&] { return f.leg() != IssMissionLeg::Transfer; }, 60.0, 1.0);
    CHECK(f.leg() == IssMissionLeg::Ended);
    CHECK_THAT(f.session.iss_mission_status(), Catch::Matchers::StartsWith("Mission beendet"));
    CHECK(f.session.docked());  // no undocking after a failed transfer
}

TEST_CASE("Ship return refused ends the mission", "[refuelling][iss_mission]") {
    test::spice_session();
    DockedAtDepot f;
    auto q = f.tanker_sequence();
    q.ship_return->search_window_s = 600.0;  // [test] no pass within reach in 10 min
    f.session.set_iss_mission(q);
    f.fly([&] { return f.leg() == IssMissionLeg::Return || f.leg() == IssMissionLeg::Ended; }, 6.0 * 3600.0, 10.0);
    CHECK(f.leg() == IssMissionLeg::Ended);
    CHECK_THAT(f.session.iss_mission_status(), Catch::Matchers::StartsWith("Mission beendet"));
}

TEST_CASE("WaitContinue coasts to its epoch and hands over to the continuation", "[refuelling][iss_mission]") {
    test::spice_session();
    DockedAtDepot f(refuelling_test::ship_with("starship_hls", 1000.0e3));
    sim::IssMissionSequence q;
    q.station_port = "transfer_dock";
    q.transfer = sim::MissionTransfer::FillChaser;
    const time::Epoch at = f.session.epoch() + 2.0 * 86400.0;
    q.continue_epoch = at;
    time::Epoch called{};
    q.continuation = [&](sim::FlightSession& s) {
        called = s.epoch();
        return std::string();
    };
    f.session.set_iss_mission(q);
    const auto wall = std::chrono::steady_clock::now();
    f.fly([&] { return f.leg() == IssMissionLeg::Complete || f.leg() == IssMissionLeg::Ended; }, 3.0 * 86400.0, 3600.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.leg() == IssMissionLeg::Complete);
    CHECK(std::abs(called - at) <= 0.5 * sim::FlightSession::tick_s + 1e-6);
    // the HLS was filled up to full (or the depot down to empty)
    const auto& hls = f.session.body(f.session.controlled_index()).vehicle();
    CHECK(hls.main_propellant_kg() > 1000.0e3 + 0.95 * 600.0e3 - 30.0e3);  // [test] 600 t minus losses / headroom
    CHECK(std::chrono::duration<double>(std::chrono::steady_clock::now() - wall).count() < 60.0);  // coasted
}

TEST_CASE("Continuation refusal ends the mission", "[refuelling][iss_mission]") {
    test::spice_session();
    DockedAtDepot f(refuelling_test::ship_with("starship_hls", 1000.0e3));
    sim::IssMissionSequence q;
    q.station_port = "transfer_dock";
    q.transfer = sim::MissionTransfer::FillChaser;
    q.continue_epoch = f.session.epoch() + 3.0 * 3600.0;
    q.continuation = [](sim::FlightSession&) { return std::string("Mondmission: zu spät"); };
    f.session.set_iss_mission(q);
    f.fly([&] { return f.leg() == IssMissionLeg::Complete || f.leg() == IssMissionLeg::Ended; }, 6.0 * 3600.0, 60.0);
    CHECK(f.leg() == IssMissionLeg::Ended);
    CHECK(f.session.iss_mission_status() == "Mission beendet: Mondmission: zu spät");
}
```

Add `#include <chrono>` to the includes. `FlightSession::tick_s`: use the session's public tick constant (check its
name in `flight_session.hpp`; 1/200 s).

- [ ] **Step 2: Run to verify they fail**

Expected: compile errors (`MissionTransfer`, the new sequence fields, `IssMissionLeg::Transfer`).

- [ ] **Step 3: Implement**

`flight_session.hpp`: the enum and the struct fields as in Interfaces (`#include <functional>`); `to_string` gets the
two names.

`flight_session_iss_mission.cpp`:

1. `set_iss_mission`: after `from_launch` add `else if (iss_mission_->transfer && docked()) enter_transfer();`
   before the `undock_epoch` branch. Give the start a private helper (declared in the header next to `iss_end`):

```cpp
// Docked with a transfer: the request of the mission's kind is started, else the mission ends (German reason).
void FlightSession::iss_start_transfer() {
    const auto& q = *iss_mission_;
    std::optional<TransferRequest> request;
    std::string reason;
    if (*q.transfer == MissionTransfer::TankerReserve) {
        request = tanker_transfer_request({}, &reason);
    } else {  // the chaser's main tanks up to full; the station down to empty
        TransferRequest r;
        r.direction = TransferDirection::StationToChaser;
        const auto& v = bodies_[controlled_].body->vehicle();
        r.limit_kg = {main_free_kg(v, config::TankContents::Oxidizer), main_free_kg(v, config::TankContents::Fuel)};
        request = r;
    }
    if (!request) return iss_end(reason);
    if (!start_transfer(*request)) return iss_end(docking_status_);
    iss_enter(IssMissionLeg::Transfer, "Treibstofftransfer");
}
```

with a file-local helper

```cpp
// The free volume of the main tanks of one propellant (headers excluded: the landing reserve), in kg.
double main_free_kg(const vehicle::Vehicle& v, config::TankContents contents) {
    double free = 0.0;
    for (const auto& stage : v.stages()) {
        for (const auto& tank : stage.tanks) {
            if (tank.config.role == config::TankRole::Main && tank.config.contents == contents) {
                free += tank.config.capacity_kg - tank.mass_kg;
            }
        }
    }
    return free;
}
```

(check the `vehicle::Tank` member names - `config`, `mass_kg` - against `vehicle.hpp`; `start_transfer`'s refusal
text: use the member r4 writes it to - `transfer_state().status` if not `docking_status_`.)

2. `iss_mission_pending`:

```cpp
        case IssMissionLeg::Ascent:
            return (autopilot_.state == AutopilotState::Off || autopilot_.state == AutopilotState::Orbit) &&
                   has_docking_port(body.vehicle()) && !body.vehicle().payload_stacked() &&
                   (!iss_mission_->ascent_to_station || body.vehicle().stages().size() == 1);
        case IssMissionLeg::Docked:
            return iss_mission_->transfer.has_value() && docked() && autopilot_.state == AutopilotState::Off;
        case IssMissionLeg::Transfer:
            return transfer_state().phase == TransferPhase::Complete || transfer_state().phase == TransferPhase::Stopped;
        case IssMissionLeg::Return:
            if (iss_mission_->ship_return) return body.caught() || body.impacted() || body.resting();
            return capsule_return_step_ == CapsuleReturnStep::SplashedDown ||
                   capsule_return_step_ == CapsuleReturnStep::Lost;
        case IssMissionLeg::WaitContinue:
            return epoch_ >= *iss_mission_->continue_epoch - 0.5 * tick_s;
```

(`Docked` moves out of the `return false` group; the r4 refusal "autopilot active" is why the leg waits for Off.)

3. `run_iss_mission` status line: `case IssMissionLeg::Transfer: iss_status_ = "Treibstofftransfer: " +
   transfer_state().status; break;`. Actions:

```cpp
        case IssMissionLeg::Ascent:
            if (q.ascent_to_station) {
                make_station(controlled_);
                iss_enter(IssMissionLeg::Complete, "Mission abgeschlossen: Depot im Orbit");
            } else if (plan_rendezvous(q.phasing)) {
                iss_enter(IssMissionLeg::Rendezvous, "Phasenflug: " + rendezvous_status_);
            } else {
                iss_end(rendezvous_status_);
            }
            return;
        // Proximity / Docking: when docked() and q.transfer: iss_enter(Docked, ...) as now; the Docked leg's pending
        // check starts the transfer on a following tick (the docking autopilot hands over first).
        case IssMissionLeg::Docked:
            iss_start_transfer();
            return;
        case IssMissionLeg::Transfer:
            if (transfer_state().phase == TransferPhase::Stopped) {
                iss_end(transfer_state().status);
            } else if (undock()) {
                iss_enter(IssMissionLeg::Departure, "Abflug: " + docking_status_);
            } else {
                iss_end(docking_status_);
            }
            return;
```

   The Departure branch: after the collision / lost checks,

```cpp
            if (q.ship_return) {
                release_tower();  // the booster has been lifted off since the launch (no-op without one)
                if (plan_ship_return(*q.ship_return)) {
                    iss_enter(IssMissionLeg::Return, "Rückkehr geplant");
                } else {
                    iss_end(ship_return_plan() ? ship_return_plan()->reason : std::string("Rückkehr nicht möglich"));
                }
                return;
            }
            if (q.continuation) {
                iss_enter(IssMissionLeg::WaitContinue, "Warten auf Fortsetzung");
                return;
            }
            if (!q.capsule_return) { ... unchanged ... }
```

   (Check `plan_ship_return`'s failure reason member: `ship_return_plan()` may be null when it refused before
   planning; `burn_abort_reason()` / the deorbit status string is the fallback - use the one the ship-return tests
   read.) Return for a ship:

```cpp
        case IssMissionLeg::Return:
            if (q.ship_return) {
                if (bodies_[controlled_].body->caught()) {
                    iss_enter(IssMissionLeg::Complete, "Mission abgeschlossen: gefangen");
                } else {
                    iss_end(lost ? "Schiff verloren" : "Schiff nicht gefangen");
                }
                return;
            }
            ... capsule as now ...
        case IssMissionLeg::WaitContinue: {
            const auto continuation = q.continuation;  // the continuation may replace the sequence
            const std::string refusal = continuation(*this);
            if (!iss_mission_) return;
            if (refusal.empty()) {
                iss_enter(IssMissionLeg::Complete, "Mission fortgesetzt");
            } else {
                iss_end(refusal);
            }
            return;
        }
```

   `q` is a reference into `iss_mission_`: copy what the continuation branch needs before calling it (done above).

4. `coast_allowance_s` (`flight_session.cpp`, next to the WaitUndock line):
   `if (iss_leg_ == IssMissionLeg::WaitContinue) allowance = std::min(allowance, *iss_mission_->continue_epoch - epoch_);`

- [ ] **Step 4: Run the tests**

`ctest --test-dir build -L "refuelling|iss_mission|iss|docking|proximity" -j 24 --output-on-failure >
tmp/r5_t2.txt 2>&1` (the `[slow]` ship-return case runs in the background if the whole run nears 10 min). Expected:
all pass; the ISS cases unchanged. Record the transfer -> deorbit numbers of the verbose run in the ledger.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session_iss_mission.cpp core/src/sim/flight_session.cpp core/tests/test_refuelling_missions.cpp
git commit -m "feat(sim): mission chain legs - transfer, ship return to the catch, continuation, ascent to station"
git push origin main
```

---

### Task 3: Mission configuration - depot_launch, tanker_flight, hls_refuel

**Files:**
- Modify: `core/include/starsim/config/mission_config.hpp:17` (`MissionKind`: `DepotLaunch, TankerFlight, HlsRefuel`)
- Modify: `core/src/config/mission_config.cpp:13-27` (`kind_of` strings and the error list)
- Create: `core/include/starsim/config/refuelling_mission_config.hpp`, `core/src/config/refuelling_mission_config.cpp`;
  Modify: `core/CMakeLists.txt` (the new source next to `mission_config.cpp`)
- Create: `config/missions/depot_ascent.toml`, `config/missions/depot_launch.toml`, `config/missions/tanker_ascent.toml`,
  `config/missions/tanker_flight.toml`, `config/missions/tanker_flight_orbit.toml`
- Modify: `config/README.md` (the new mission kinds)
- Create: `core/tests/test_refuelling_mission_config.cpp`; Modify: `core/tests/CMakeLists.txt`

**Interfaces:**
- Produces (namespace `starsim::config`):
  ```cpp
  struct DepotLaunchMission {
      std::string id, name, ascent_id, epoch_utc;  // ascent of the starship_depot stack from its site
  };
  struct DepotOrbitConfig {
      std::string vehicle_id = "starship_depot";
      std::string port = "transfer_dock";
      double height_m = 0.0;                    // circular
      double inclination_rad = 0.0;             // true equator frame (as the synthetic ISS)
      double raan_rad = 0.0;                    // at earliest_utc; hls_refuel: computed from the lunar plane
      double argument_of_latitude_rad = 0.0;    // at earliest_utc
      double propellant_kg = 0.0;               // main tanks at O/F of the stage's main tanks
  };
  enum class RefuellingStart { Pad, PhasingOrbit };
  struct RefuellingFlightMission {
      std::string id, name;
      MissionKind kind = MissionKind::TankerFlight;   // or HlsRefuel
      std::string ascent_id;                          // the chaser's ascent mission (vehicle, site, target orbit)
      std::string port = "transfer_dock";             // the chaser's port
      DepotOrbitConfig depot;
      std::string earliest_utc;
      RefuellingStart start = RefuellingStart::Pad;
      double phasing_behind_rad = 0.0;                // PhasingOrbit: behind the depot, in its plane
      double start_propellant_kg = 0.0;               // PhasingOrbit: main tanks (O/F of the stage)
      IssLaunchWindowConfig window;                   // Pad
      double mission_limit_s = 0.0;
      double earliest_arrival_s = 0.0;
      std::string return_id;                          // tanker_flight: the ship_return mission's [deorbit]
      std::string lunar_mission_id;                   // hls_refuel: the lunar landing mission continued
  };
  DepotLaunchMission load_depot_launch_mission(const std::filesystem::path& config_dir, std::string_view mission_id);
  RefuellingFlightMission load_refuelling_flight_mission(const std::filesystem::path& config_dir,
                                                         std::string_view mission_id);  // tanker_flight | hls_refuel
  ```
  The phasing orbit's apsides come from the ascent mission's target (170 x 190 km), so both starts share one orbit.

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_refuelling_mission_config.cpp`:

```cpp
// Refuelling r5: the mission configurations of the reference flights.

#include <filesystem>
#include <fstream>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/refuelling_mission_config.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
constexpr double deg = constants::deg_to_rad;

// A copy of config/ with one extra mission file (unique directory per test).
std::filesystem::path with_mission(const std::string& name, const std::string& id, const std::string& text) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_r5_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "missions");
    std::ofstream(dir / "missions" / (id + ".toml")) << text;
    return dir;
}
}  // namespace

TEST_CASE("depot_launch and tanker_flight configurations load", "[refuelling][config][mission]") {
    CHECK(config::mission_kind(config_dir, "depot_launch") == config::MissionKind::DepotLaunch);
    const auto d = config::load_depot_launch_mission(config_dir, "depot_launch");
    CHECK(d.ascent_id == "depot_ascent");
    const auto a = config::load_ascent_mission(config_dir, "depot_ascent");
    CHECK(a.vehicle_id == "starship_depot");
    CHECK(a.periapsis_height_m == Catch::Approx(200.0e3));
    CHECK(a.inclination_rad == Catch::Approx(26.5 * deg));

    const auto t = config::load_refuelling_flight_mission(config_dir, "tanker_flight");
    CHECK(t.kind == config::MissionKind::TankerFlight);
    CHECK(t.ascent_id == "tanker_ascent");
    CHECK(t.start == config::RefuellingStart::Pad);
    CHECK(t.depot.height_m == Catch::Approx(200.0e3));
    CHECK(t.depot.inclination_rad == Catch::Approx(26.5 * deg));
    CHECK(t.depot.propellant_kg == Catch::Approx(600.0e3));
    CHECK(t.mission_limit_s == Catch::Approx(48.0 * 3600.0));
    CHECK(t.window.search_s == Catch::Approx(12.0 * 86400.0));
    CHECK(t.return_id == "ship_return_starbase");
    const auto ta = config::load_ascent_mission(config_dir, "tanker_ascent");
    CHECK(ta.vehicle_id == "starship_tanker");
    CHECK(ta.periapsis_height_m == Catch::Approx(170.0e3));
    CHECK(ta.apoapsis_height_m == Catch::Approx(190.0e3));

    const auto o = config::load_refuelling_flight_mission(config_dir, "tanker_flight_orbit");
    CHECK(o.start == config::RefuellingStart::PhasingOrbit);
    CHECK(o.phasing_behind_rad == Catch::Approx(8.0 * deg));
    CHECK(o.start_propellant_kg > 0.0);
}

TEST_CASE("Refuelling mission configs refuse bad input", "[refuelling][config][mission]") {
    const std::string base = R"([mission]
name = "x"
kind = "tanker_flight"
ascent = "tanker_ascent"
return = "ship_return_starbase"
[depot]
height_km = 200.0
inclination_deg = 26.5
propellant_t = 600.0
[start]
mode = "%MODE%"
behind_deg = 8.0
propellant_t = 150.0
[launch_window]
earliest_utc = "2026-10-20T00:00:00"
search_days = 12.0
insertion_s = 520.0
insertion_arc_deg = 20.0
[phasing]
mission_limit_h = 48.0
earliest_arrival_h = 0.0
)";
    const auto mode = [&](const std::string& m) {
        std::string text = base;
        text.replace(text.find("%MODE%"), 6, m);
        return text;
    };
    const auto ok = with_mission("cfg_ok", "t", mode("pad"));
    CHECK_NOTHROW(config::load_refuelling_flight_mission(ok, "t"));
    const auto bad = with_mission("cfg_mode", "t", mode("balloon"));
    CHECK_THROWS_WITH(config::load_refuelling_flight_mission(bad, "t"), Catch::Matchers::ContainsSubstring("mode"));
    std::string no_return = mode("pad");
    no_return.erase(no_return.find("return ="), std::string("return = \"ship_return_starbase\"\n").size());
    const auto nr = with_mission("cfg_return", "t", no_return);
    CHECK_THROWS_WITH(config::load_refuelling_flight_mission(nr, "t"), Catch::Matchers::ContainsSubstring("return"));
    std::string ascent = mode("pad");
    ascent.replace(ascent.find("kind = \"tanker_flight\""), 22, "kind = \"ascent\"      ");
    const auto wrong = with_mission("cfg_kind", "t", ascent);
    CHECK_THROWS(config::load_refuelling_flight_mission(wrong, "t"));
}
```

Add the file to `core/tests/CMakeLists.txt` after `test_refuelling_missions.cpp`.

- [ ] **Step 2: Run to verify they fail**

Expected: compile errors (header missing).

- [ ] **Step 3: Implement**

`mission_config.hpp:17`: append `DepotLaunch, TankerFlight, HlsRefuel` to `MissionKind`. `mission_config.cpp`
`kind_of`: `"depot_launch"`, `"tanker_flight"`, `"hls_refuel"` and the three names in the error list. `mission_start.cpp`
gets `case`s in Task 4/5/8; until then add them to the switch as
`throw std::invalid_argument("mission '...': kind not yet startable")` only if the compiler warns about an unhandled
enum (the switch may have a default).

`refuelling_mission_config.hpp` with the structs above (includes `mission_config.hpp` for `MissionKind` and
`IssLaunchWindowConfig`). `refuelling_mission_config.cpp` follows `load_iss_launch_mission` (`detail::Document`,
`doc.text`, `doc.positive`, `doc.number`, `utc`, `optional_deg`). Those helpers are file-local in `mission_config.cpp`:
move `utc`, `optional_deg` and `require_kind` into `core/src/config/mission_config_detail.hpp` (inline, namespace
`starsim::config::detail_mission`) and include it from both files.

```cpp
RefuellingFlightMission load_refuelling_flight_mission(const std::filesystem::path& config_dir,
                                                       std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    const double deg = constants::deg_to_rad;
    RefuellingFlightMission m;
    m.id = std::string(mission_id);
    m.kind = kind_of(doc);
    if (m.kind != MissionKind::TankerFlight && m.kind != MissionKind::HlsRefuel) {
        doc.fail("not a tanker_flight / hls_refuel mission ([mission] kind)");
    }
    const auto* mission = doc.root["mission"].as_table();
    const auto* depot = doc.root["depot"].as_table();
    const auto* start = doc.root["start"].as_table();
    const auto* window = doc.root["launch_window"].as_table();
    const auto* phasing = doc.root["phasing"].as_table();
    if (!depot || !start || !window || !phasing) doc.fail("missing [depot], [start], [launch_window] or [phasing]");
    m.name = doc.text(*mission, "name");
    m.ascent_id = doc.text(*mission, "ascent");
    m.port = doc.text_or(*mission, "port", "transfer_dock");
    m.depot.vehicle_id = doc.text_or(*depot, "vehicle", "starship_depot");
    m.depot.port = doc.text_or(*depot, "port", "transfer_dock");
    m.depot.height_m = doc.positive(*depot, "height_km") * 1e3;
    m.depot.inclination_rad = doc.positive(*depot, "inclination_deg") * deg;
    m.depot.raan_rad = doc.number_or(*depot, "raan_deg", 0.0) * deg;
    m.depot.argument_of_latitude_rad = doc.number_or(*depot, "argument_of_latitude_deg", 0.0) * deg;
    m.depot.propellant_kg = doc.positive(*depot, "propellant_t") * 1e3;
    const auto mode = doc.text(*start, "mode");
    if (mode == "pad") {
        m.start = RefuellingStart::Pad;
    } else if (mode == "phasing_orbit") {
        m.start = RefuellingStart::PhasingOrbit;
        m.phasing_behind_rad = doc.positive(*start, "behind_deg") * deg;
        m.start_propellant_kg = doc.positive(*start, "propellant_t") * 1e3;
    } else {
        doc.fail("[start] mode must be \"pad\" or \"phasing_orbit\"");
    }
    m.earliest_utc = utc(doc, *window, "earliest_utc");
    m.window.search_s = doc.positive(*window, "search_days") * 86400.0;
    m.window.insertion_s = doc.positive(*window, "insertion_s");
    m.window.insertion_arc_rad = doc.positive(*window, "insertion_arc_deg") * deg;
    m.mission_limit_s = doc.positive(*phasing, "mission_limit_h") * 3600.0;
    m.earliest_arrival_s = doc.number(*phasing, "earliest_arrival_h") * 3600.0;
    if (m.kind == MissionKind::TankerFlight) {
        m.return_id = doc.text(*mission, "return");  // "missing ... 'return'" from detail::Document
    } else {
        const auto* lunar = doc.root["lunar"].as_table();
        if (!lunar) doc.fail("missing [lunar]");
        m.lunar_mission_id = doc.text(*lunar, "mission");
    }
    return m;
}
```

(Use the `Document` helpers that exist - `text_or` exists (`kind_of`); if `number_or` does not, read
`(*depot)["raan_deg"].value<double>().value_or(0.0)` after checking the key is a number.) `load_depot_launch_mission`:
`require_kind(DepotLaunch)`, `[mission] name, ascent`, `[start] epoch_utc`.

Config files:

`config/missions/depot_ascent.toml`:
```toml
# Refuelling r5: the depot's ascent from Starbase into the depot orbit (spec r5 "depot_launch"). The pitch program is
# starship_leo's (same booster, a ship stage ~19 t lighter dry); the booster is caught.
[mission]
name = "Depot in den Erdorbit"
vehicle = "starship_depot"
site = "starbase_olp1"
heading_deg = 90.0
payload_t = 0.0
fairing = false

[target]
periapsis_km = 200.0          # the lunar missions' start orbit (spec r3)
apoapsis_km = 200.0
inclination_deg = 26.5
northerly = true

[pitch_program]
kick_deg = 14.0
booster_throttle = [[60.0, 0.85], [64.0, 0.76], [76.0, 0.76], [82.0, 0.83], [112.0, 0.83], [126.0, 0.76]]
```

`config/missions/depot_launch.toml`:
```toml
# Refuelling r5: the depot launches first; the ship becomes the station in its orbit, the residue after the ascent is
# its initial fill (measured, for the r6 campaign).
[mission]
name = "Depotstart"
kind = "depot_launch"
ascent = "depot_ascent"

[start]
epoch_utc = "2026-10-20T00:00:00"
```

`config/missions/tanker_ascent.toml`: as `depot_ascent.toml` with `name = "Tanker in den Phasenorbit"`,
`vehicle = "starship_tanker"`, `periapsis_km = 170.0`, `apoapsis_km = 190.0` (comment: r3's phasing orbit below the
200 km depot).

`config/missions/tanker_flight.toml`:
```toml
# Refuelling r5: one tanker flight in the full model (spec r5 "tanker_flight"): launch window into the depot plane ->
# ascent with booster catch -> phasing -> proximity -> docking -> transfer down to the computed reserve -> undocking
# and departure -> deorbit -> entry -> flip and catch on the header tanks.
[mission]
name = "Tankerflug zum Depot"
kind = "tanker_flight"
ascent = "tanker_ascent"
port = "transfer_dock"
return = "ship_return_starbase"   # its [deorbit] block (entry interface, bank, crossrange)

# The depot, placed (not flown in this mission): synthetic orbit advanced by J2 (plan decision 2).
[depot]
vehicle = "starship_depot"
port = "transfer_dock"
height_km = 200.0
inclination_deg = 26.5            # true equator frame
raan_deg = 0.0                    # at earliest_utc
argument_of_latitude_deg = 0.0    # at earliest_utc
propellant_t = 600.0              # [test] a depot after a few tankers (r3/r4 tests)

[start]
mode = "pad"                      # "pad" | "phasing_orbit"

[launch_window]
earliest_utc = "2026-10-20T00:00:00"
search_days = 12.0                # [E] r3: the crossings close the phasing on few days
insertion_s = 520.0               # [test] liftoff -> SECO; calibrated from the flown ascent in Task 5
insertion_arc_deg = 20.0          # [test] inertial in-plane arc from the pad to SECO; calibrated in Task 5

[phasing]
mission_limit_h = 48.0            # [E] uncrewed (the crew's 28 h opened the first window after 177 h, r3)
earliest_arrival_h = 0.0
```

`config/missions/tanker_flight_orbit.toml`: the same file with `name = "Tankerflug ab Phasenorbit"` and

```toml
[start]
mode = "phasing_orbit"            # fast tests: the tanker 8 deg behind the depot in its 170 x 190 km orbit
behind_deg = 8.0
propellant_t = 150.0              # [test] main tanks; replaced by the flown ascent's residue in Task 5 [D]
```

`config/README.md`: a "Refuelling missions (r5)" paragraph with the three kinds, `[depot]`, `[start] mode`, `[lunar]`.

- [ ] **Step 4: Run the tests**

`ctest --test-dir build -L "config|mission" -j 24 --output-on-failure > tmp/r5_t3.txt 2>&1`. Expected: all pass
(the existing mission configs unchanged).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/config core/src/config core/CMakeLists.txt config/missions config/README.md core/tests/test_refuelling_mission_config.cpp core/tests/CMakeLists.txt
git commit -m "feat(config): refuelling reference missions - depot_launch, tanker_flight, hls_refuel kinds"
git push origin main
```

---

### Task 4: depot_launch - the depot flies from the pad and becomes the station

**Files:**
- Create: `core/include/starsim/sim/refuelling_mission.hpp`, `core/src/sim/refuelling_mission.cpp`; Modify:
  `core/CMakeLists.txt`
- Modify: `core/src/sim/mission_start.cpp` (case `DepotLaunch`)
- Test: `core/tests/test_refuelling_missions.cpp`

**Interfaces:**
- Consumes: Task 2 (`ascent_to_station`), Task 3 (`load_depot_launch_mission`), `start_ascent_mission`.
- Produces:
  ```cpp
  // depot_launch (refuelling r5): the depot stack from its pad; at orbit insertion the ship becomes the station
  // (sequence ascent_to_station), the booster flies on to the catch.
  MissionStart start_depot_launch_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                          const config::DepotLaunchMission& mission, const time::TimeScaleConverter& time);
  ```

- [ ] **Step 1: Write the failing tests**

Append:

```cpp
TEST_CASE("depot_launch starts the depot ascent through start_mission", "[refuelling][mission]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto s = sim::start_mission(f.session, refuelling_test::config_dir, "depot_launch", f.t0, f.clock);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    CHECK(s.kind == config::MissionKind::DepotLaunch);
    CHECK(s.site_id == "starbase_olp1");
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Ascent);
    CHECK_FALSE(f.session.station_index());
}

TEST_CASE("depot_launch: pad -> orbit, the depot becomes the station, the booster is caught",
          "[refuelling][mission][ascent][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    REQUIRE(sim::start_mission(f.session, refuelling_test::config_dir, "depot_launch", f.t0, f.clock).refusal.empty());
    while (f.session.epoch() - f.t0 < 1800.0 && f.session.iss_mission_leg() == IssMissionLeg::Ascent) {
        f.session.advance_to(f.session.epoch() + 1.0);
    }
    INFO(f.session.iss_mission_status());
    REQUIRE(f.session.iss_mission_leg() == IssMissionLeg::Complete);
    REQUIRE(f.session.station_index());
    const auto& depot = f.session.body(*f.session.station_index());
    const double fill = depot.vehicle().main_propellant_kg();
    const auto e = orbit::elements_from_state({depot.state().position_gcrf_m, depot.state().velocity_gcrf_mps},
                                              constants::earth_gm_m3ps2);
    const double rp = e.semi_major_axis_m * (1.0 - e.eccentricity) - constants::wgs84_semi_major_axis_m;
    const double ra = e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::wgs84_semi_major_axis_m;
    while (f.session.epoch() - f.t0 < 1800.0 && f.session.tower()->phase() != sim::CatchPhase::Caught) {
        f.session.advance_to(f.session.epoch() + 1.0);
    }
    CHECK(f.session.tower()->phase() == sim::CatchPhase::Caught);
    CHECK(rp > 185.0e3);  // [E] insertion within ~15 km of the 200 km target (PEG; the depot's drag is not made up yet)
    CHECK(ra < 215.0e3);
    CHECK(fill > 0.0);
    // one orbit as the station: ideal attitude, boil-off on
    const double before = depot.vehicle().main_propellant_kg();
    f.session.advance_to(f.session.epoch() + 5400.0);
    const double boil_kg = before - f.session.body(*f.session.station_index()).vehicle().main_propellant_kg();
    CHECK(boil_kg > 0.0);
    std::printf("[r5] depot_launch: %.1f x %.1f km, initial fill %.1f t (main), boil-off %.1f kg in 90 min\n",
                rp / 1e3, ra / 1e3, fill / 1e3, boil_kg);
}
```

(includes: `starsim/sim/mission_start.hpp`, `starsim/orbit/kepler.hpp`.)

- [ ] **Step 2: Run to verify they fail**

Expected: `start_mission` throws "unknown kind" / not startable, or the compile fails on the missing header.

- [ ] **Step 3: Implement**

`refuelling_mission.hpp` (namespace `starsim::sim`, includes `mission_start.hpp`, `refuelling_mission_config.hpp`):
the declaration above. `refuelling_mission.cpp`:

```cpp
MissionStart start_depot_launch_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                        const config::DepotLaunchMission& mission, const time::TimeScaleConverter& time) {
    MissionStart s;
    s.kind = config::MissionKind::DepotLaunch;
    const auto ascent = config::load_ascent_mission(config_dir, mission.ascent_id);
    s.epoch = time.from_utc(mission.epoch_utc);
    start_ascent_mission(session, ascent, config_dir, s.epoch);
    IssMissionSequence q;
    q.from_launch = true;
    q.ascent_to_station = true;
    session.set_iss_mission(q);
    s.site_id = ascent.site_id;
    s.return_planned = true;
    return s;
}
```

`mission_start.cpp`: `case config::MissionKind::DepotLaunch: s = start_depot_launch_mission(session, config_dir,
config::load_depot_launch_mission(config_dir, mission_id), time); break;` - note `start_mission` takes `requested`:
the ascent kinds start at `requested`; keep that convention: pass `requested` if it is not the default epoch, else
the config's `epoch_utc` (read how `start_ascent_mission` is dispatched and do the same; document the choice in the
header comment).

If the Ascent leg fires before staging (the stack has the depot's `transfer_dock` too): the pending condition of
Task 2 requires one stage for `ascent_to_station` - verify with `STARSIM_VERBOSE` that `make_station` runs after SECO
(the leg-mark epoch is ~500 s after liftoff), not at liftoff.

- [ ] **Step 4: Run the tests**

`ctest --test-dir build -L "refuelling|mission" -j 24 --output-on-failure > tmp/r5_t4.txt 2>&1`. If the ascent does not
reach orbit (PEG fails with the lighter ship, or the booster catch misses with the depot's staging mass), stop and
report the numbers - do not retune the pitch program silently (it is shared knowledge from `starship_leo`).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/sim/refuelling_mission.hpp core/src/sim/refuelling_mission.cpp core/src/sim/mission_start.cpp core/CMakeLists.txt core/tests/test_refuelling_missions.cpp
git commit -m "feat(sim): depot_launch - the depot ascends from Starbase and becomes the station, booster caught"
git push origin main
```

---

### Task 5: tanker_flight - start from the pad or the phasing orbit, window calibration

**Files:**
- Modify: `core/include/starsim/sim/refuelling_mission.hpp`, `core/src/sim/refuelling_mission.cpp`
- Modify: `core/src/sim/mission_start.cpp` (case `TankerFlight`)
- Modify: `config/missions/tanker_flight.toml`, `config/missions/tanker_flight_orbit.toml` (calibrated [D] numbers)
- Test: `core/tests/test_refuelling_missions.cpp`

**Interfaces:**
- Consumes: Task 2 (`transfer`, `ship_return`), Task 3 (`load_refuelling_flight_mission`), `plan_launch_window`,
  `orbit::advance_mean_elements`, `synthetic_station_state`.
- Produces:
  ```cpp
  // The depot of a refuelling flight at any epoch: its [depot] mean elements at `reference` advanced by the J2 rates.
  mission::StationStateAt depot_orbit(const config::DepotOrbitConfig& depot, time::Epoch reference,
                                      const frames::BodyOrientation& earth);
  // `behind_rad` of argument of latitude behind `station`, in its plane, on a periapsis_height_m x apoapsis_height_m
  // orbit with the periapsis at the start (r3's tanker_behind).
  math::StateVector phasing_start_state(const math::StateVector& station, double behind_rad, double periapsis_height_m,
                                        double apoapsis_height_m);
  gnc::DeorbitParameters ship_deorbit_parameters(const config::ShipReturnDeorbit& deorbit);
  // tanker_flight / hls_refuel (r5): the depot placed from [depot]; the chaser launched in its window (Pad) or placed
  // behind it (PhasingOrbit); the chain to the transfer and on (tanker: ship return, hls: continuation).
  MissionStart start_refuelling_flight_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                               const config::RefuellingFlightMission& mission,
                                               const time::TimeScaleConverter& time);
  ```

- [ ] **Step 1: Write the failing tests**

Append:

```cpp
TEST_CASE("tanker_flight_orbit starts behind the depot and plans the phasing", "[refuelling][mission][rendezvous]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto s = sim::start_mission(f.session, refuelling_test::config_dir, "tanker_flight_orbit", f.t0, f.clock);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    CHECK(s.kind == config::MissionKind::TankerFlight);
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Rendezvous);
    REQUIRE(f.session.station_index());
    CHECK(f.session.body(*f.session.station_index()).vehicle().main_propellant_kg() == Catch::Approx(600.0e3));
    CHECK(f.session.launch_site().latitude_rad == Catch::Approx(
        config::load_launch_site(refuelling_test::config_dir, "starbase_olp1").latitude_rad));  // the return site
}

TEST_CASE("tanker_flight finds a launch window into the depot plane within the uncrewed limit",
          "[refuelling][mission][launch_window]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto s = sim::start_mission(f.session, refuelling_test::config_dir, "tanker_flight", f.t0, f.clock);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    REQUIRE(s.launch_window);
    CHECK(s.launch_window->feasible);
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Ascent);
    std::printf("[r5] tanker window: liftoff %.1f h after earliest, %d crossings rejected\n",
                (s.launch_window->liftoff - f.t0) / 3600.0, s.launch_window->rejected);
}

TEST_CASE("tanker_flight: the window's separation estimate matches the flown ascent",
          "[refuelling][mission][ascent][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto m = config::load_refuelling_flight_mission(refuelling_test::config_dir, "tanker_flight");
    const auto s = sim::start_mission(f.session, refuelling_test::config_dir, "tanker_flight", f.t0, f.clock);
    REQUIRE(s.refusal.empty());
    const auto& w = *s.launch_window;
    while (f.session.iss_mission_leg() == IssMissionLeg::Ascent && f.session.epoch() - w.liftoff < 1200.0) {
        f.session.advance_to(f.session.epoch() + 1.0);
    }
    REQUIRE(f.session.iss_mission_leg() == IssMissionLeg::Rendezvous);
    const double seco_s = f.session.iss_mission_marks()[1].epoch - w.liftoff;  // marks: Ascent, Rendezvous
    const auto& ship = f.session.body(f.session.controlled_index());
    const Vec3 r = ship.state().position_gcrf_m;
    const Vec3 r_est = w.separation_state_gcrf.position_m;
    const double angle = std::acos(std::clamp(dot(math::normalized(r), math::normalized(r_est)), -1.0, 1.0));
    std::printf("[r5] tanker SECO %.1f s after liftoff (config %.1f), %.3f deg from the estimate, main %.1f t\n",
                seco_s, m.window.insertion_s, angle / deg, ship.vehicle().main_propellant_kg() / 1e3);
    CHECK(std::abs(seco_s - m.window.insertion_s) < 5.0);  // [E] calibrated (i6: Falcon 9 within 1 s)
    CHECK(angle < 0.1 * deg);                               // [E] i6: 0.004 deg in phase
}
```

- [ ] **Step 2: Run to verify they fail**

Expected: `start_mission` refuses / throws for the tanker kinds.

- [ ] **Step 3: Implement**

`refuelling_mission.cpp`:

```cpp
mission::StationStateAt depot_orbit(const config::DepotOrbitConfig& depot, time::Epoch reference,
                                    const frames::BodyOrientation& earth) {
    orbit::StationMeanElements m;
    m.epoch = reference;
    m.semi_major_axis_m = constants::wgs84_semi_major_axis_m + depot.height_m;
    m.inclination_rad = depot.inclination_rad;
    m.raan_rad = depot.raan_rad;
    m.argument_of_latitude_rad = depot.argument_of_latitude_rad;
    return [m, &earth](time::Epoch t) {
        const auto at = orbit::advance_mean_elements(m, t, constants::earth_gm_m3ps2, constants::earth_j2,
                                                     constants::wgs84_semi_major_axis_m);
        return orbit::synthetic_station_state(at, constants::earth_gm_m3ps2, earth);
    };
}
```

(Check the J2 / radius constant names in `constants.hpp` and how `iss_start` calls `advance_mean_elements` - use the
same arguments.)

```cpp
math::StateVector phasing_start_state(const math::StateVector& station, double behind_rad, double periapsis_height_m,
                                      double apoapsis_height_m) {
    const Vec3 n = math::normalized(cross(station.position_m, station.velocity_mps));
    const auto back = math::Quat::from_axis_angle(n, -behind_rad);
    const Vec3 r_hat = math::normalized(back.rotate(station.position_m));
    const Vec3 v_hat = cross(n, r_hat);
    const double rp = constants::wgs84_semi_major_axis_m + periapsis_height_m;
    const double ra = constants::wgs84_semi_major_axis_m + apoapsis_height_m;
    const double a = 0.5 * (rp + ra);
    const double vp = std::sqrt(constants::earth_gm_m3ps2 * (2.0 / rp - 1.0 / a));
    return {rp * r_hat, vp * v_hat};
}

gnc::DeorbitParameters ship_deorbit_parameters(const config::ShipReturnDeorbit& d) {
    gnc::DeorbitParameters p;
    p.entry_interface_height_m = d.entry_interface_height_m;
    p.entry_flight_path_rad = d.entry_flight_path_rad;
    p.nominal_bank_rad = d.nominal_bank_rad;
    p.max_crossrange_m = d.max_crossrange_m;
    p.search_window_s = FlightSession::tanker_reserve_search_s;  // as the reserve's preview (r4: 48 h find no pass)
    return p;
}
```

(`test_ship_return_mission.cpp:69` has the same mapping locally - make it call `sim::ship_deorbit_parameters` and set
`search_window_s` from the mission afterwards, so there is one mapping.)

`start_refuelling_flight_mission`:

```cpp
MissionStart start_refuelling_flight_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                             const config::RefuellingFlightMission& m,
                                             const time::TimeScaleConverter& time) {
    MissionStart s;
    s.kind = m.kind;
    const auto& earth = session.earth_orientation();
    const time::Epoch earliest = time.from_utc(m.earliest_utc);
    const auto depot_at = depot_orbit(m.depot, earliest, earth);
    const auto ascent = config::load_ascent_mission(config_dir, m.ascent_id);
    const auto site = config::load_launch_site(config_dir, ascent.site_id);
    auto depot_vehicle = config::top_stage_config(config::load_vehicle(config_dir, m.depot.vehicle_id));
    fill_main_tanks(depot_vehicle.stages.back(), m.depot.propellant_kg);
    mission::PhasingParameters phasing;
    phasing.mission_limit_s = m.mission_limit_s;
    phasing.earliest_arrival_s = m.earliest_arrival_s;
    IssMissionSequence q;
    q.station_port = m.depot.port;
    if (m.start == config::RefuellingStart::Pad) {
        const Vec3 pad = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
        mission::LaunchWindowParameters p;
        p.inclination_rad = ascent.inclination_rad;
        p.northerly = ascent.northerly;
        p.insertion_s = m.window.insertion_s;
        p.insertion_arc_rad = m.window.insertion_arc_rad;
        p.separation_periapsis_radius_m = constants::wgs84_semi_major_axis_m + ascent.periapsis_height_m;
        p.separation_apoapsis_radius_m = constants::wgs84_semi_major_axis_m + ascent.apoapsis_height_m;
        p.search_s = m.window.search_s;
        p.phasing.mission_limit_s = m.mission_limit_s - m.window.insertion_s;  // from SECO
        p.phasing.earliest_arrival_s = std::max(0.0, m.earliest_arrival_s - m.window.insertion_s);
        const auto w = mission::plan_launch_window(pad, earth, depot_at, earliest, p, constants::earth_gm_m3ps2);
        s.launch_window = w;
        if (!w.feasible) {
            s.refusal = w.reason;
            return s;
        }
        start_ascent_mission(session, ascent, config_dir, w.liftoff);
        session.add_station(depot_vehicle, depot_at(w.liftoff), w.liftoff);  // after the launch (it clears the bodies)
        q.from_launch = true;
        q.phasing = p.phasing;
        s.epoch = w.liftoff;
        s.site_id = ascent.site_id;
    } else {
        auto chaser = config::top_stage_config(config::load_vehicle(config_dir, ascent.vehicle_id));
        fill_main_tanks(chaser.stages.back(), m.start_propellant_kg);
        const auto depot = depot_at(earliest);
        session.place_in_orbit(chaser, phasing_start_state(depot, m.phasing_behind_rad, ascent.periapsis_height_m,
                                                           ascent.apoapsis_height_m), earliest);
        session.set_launch_site(launch_site_from_config(site));  // the return site
        session.add_station(depot_vehicle, depot, earliest);
        if (!session.plan_rendezvous(phasing)) {
            s.refusal = session.rendezvous_status();
            session.clear_bodies();
            return s;
        }
        q.phasing = phasing;
        s.epoch = earliest;
    }
    q.transfer = m.kind == config::MissionKind::TankerFlight ? MissionTransfer::TankerReserve : MissionTransfer::FillChaser;
    if (m.kind == config::MissionKind::TankerFlight) {
        q.ship_return = ship_deorbit_parameters(config::load_ship_return_mission(config_dir, m.return_id).deorbit);
    }
    session.set_iss_mission(q);  // hls_refuel adds its continuation (Task 8) before this call
    s.return_planned = true;
    return s;
}
```

with a file-local `fill_main_tanks(config::StageConfig&, double kg)`: the main oxidizer / fuel tanks filled at the
ratio of their capacities, capped at capacity (as `refuelling_test::ship_with`, but from the capacities so it holds
for any stage). The chaser's port: `set_rendezvous_target` is done by `plan_rendezvous` / `start_proximity` from the
session's station - check how the r3 campaign test gets the port pair (`transfer_dock` on both) and pass `m.port` the
same way if the sequence needs it.

`mission_start.cpp`: `case TankerFlight: case HlsRefuel:` -> `start_refuelling_flight_mission(session, config_dir,
config::load_refuelling_flight_mission(config_dir, mission_id), time)`; `hls_refuel` is completed in Task 8.

- [ ] **Step 4: Calibrate and run**

1. Run the slow case once with the `[test]` numbers:
   `build/core/tests/starsim_core_tests.exe "tanker_flight: the window's separation estimate matches the flown ascent" > tmp/r5_t5_cal.txt 2>&1`.
2. Write the printed SECO time into `tanker_flight.toml` / `tanker_flight_orbit.toml` `insertion_s` ([D] r5, flown
   tanker ascent). Compute the inertial in-plane arc pad -> SECO position (print it in the test as well: the angle
   between the pad's GCRF position at liftoff and the SECO position, projected on the plane) and write it into
   `insertion_arc_deg` ([D]). Write the printed main-tank residue into `tanker_flight_orbit.toml` `propellant_t` ([D]).
3. Re-run: the case passes within its tolerances (a second iteration if the window moved).
4. `ctest --test-dir build -L "refuelling|mission|launch_window|ship" -j 24 --output-on-failure > tmp/r5_t5.txt 2>&1`
   (the ship-return mission tests use the shared deorbit mapping now).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/sim/refuelling_mission.hpp core/src/sim/refuelling_mission.cpp core/src/sim/mission_start.cpp config/missions/tanker_flight.toml config/missions/tanker_flight_orbit.toml core/tests/test_refuelling_missions.cpp core/tests/test_ship_return_mission.cpp
git commit -m "feat(sim): tanker_flight - window into the depot plane or phasing-orbit start, calibrated SECO"
git push origin main
```

---

### Task 6: tanker_flight campaigns - phasing orbit and pad to the catch, measured numbers

**Files:**
- Test: `core/tests/test_refuelling_missions.cpp`
- Create: `core/tests/refuelling_measurements.hpp` (the numbers from the leg marks; r6 stores them)

**Interfaces:**
- Consumes: Tasks 1-5.
- Produces (test support, namespace `starsim::refuelling_test`):
  ```cpp
  struct TankerFlightNumbers {
      double residue_after_ascent_kg = 0.0;  // main tanks at the Rendezvous mark (pad start only)
      double rendezvous_kg = 0.0;            // main tanks Rendezvous mark -> Docked mark (burns, RCS gas, boil-off)
      double delivered_oxidizer_kg = 0.0, delivered_fuel_kg = 0.0;
      double transfer_s = 0.0, settling_gas_kg = 0.0, chilldown_kg = 0.0;
      double main_at_undock_kg = 0.0;        // Departure mark
      double main_at_catch_kg = 0.0;         // Complete mark: what the reserve left over
      double flight_s = 0.0;                 // first mark -> Complete
  };
  TankerFlightNumbers tanker_numbers(const sim::FlightSession& session);  // from marks + transfer_state
  ```

- [ ] **Step 1: Write the campaign cases**

`core/tests/refuelling_measurements.hpp`:

```cpp
#pragma once
// Refuelling r5: the measured numbers of a tanker flight, read from the mission's leg marks.
#include <algorithm>
#include "starsim/sim/flight_session.hpp"

namespace starsim::refuelling_test {

struct TankerFlightNumbers { /* fields as in Interfaces */ };

inline const sim::MissionMark* mark(const sim::FlightSession& s, sim::IssMissionLeg leg) {
    const auto& marks = s.iss_mission_marks();
    const auto it = std::find_if(marks.begin(), marks.end(), [leg](const auto& m) { return m.leg == leg; });
    return it == marks.end() ? nullptr : &*it;
}

inline TankerFlightNumbers tanker_numbers(const sim::FlightSession& s) {
    TankerFlightNumbers n;
    const auto* rv = mark(s, sim::IssMissionLeg::Rendezvous);
    const auto* docked = mark(s, sim::IssMissionLeg::Docked);
    const auto* transfer = mark(s, sim::IssMissionLeg::Transfer);
    const auto* departure = mark(s, sim::IssMissionLeg::Departure);
    const auto* complete = mark(s, sim::IssMissionLeg::Complete);
    if (rv) n.residue_after_ascent_kg = rv->chaser_main_propellant_kg;
    if (rv && docked) n.rendezvous_kg = rv->chaser_main_propellant_kg - docked->chaser_main_propellant_kg;
    const auto& t = s.transfer_state();
    n.delivered_oxidizer_kg = t.delivered_kg[0];
    n.delivered_fuel_kg = t.delivered_kg[1];
    n.settling_gas_kg = t.settling_gas_kg;
    n.chilldown_kg = t.chilldown_kg[0] + t.chilldown_kg[1];
    if (transfer && departure) n.transfer_s = departure->epoch - transfer->epoch;
    if (departure) n.main_at_undock_kg = departure->chaser_main_propellant_kg;
    if (complete) n.main_at_catch_kg = complete->chaser_main_propellant_kg;
    if (complete) n.flight_s = complete->epoch - s.iss_mission_marks().front().epoch;
    return n;
}

inline void print(const char* tag, const TankerFlightNumbers& n) {
    std::printf("[r5] %s: residue %.1f t, rendezvous %.2f t, delivered LOX %.1f t / CH4 %.1f t in %.0f s, gas %.2f t, "
                "chill %.2f t, main at undock %.2f t, at catch %.2f t, flight %.1f h\n",
                tag, n.residue_after_ascent_kg / 1e3, n.rendezvous_kg / 1e3, n.delivered_oxidizer_kg / 1e3,
                n.delivered_fuel_kg / 1e3, n.transfer_s, n.settling_gas_kg / 1e3, n.chilldown_kg / 1e3,
                n.main_at_undock_kg / 1e3, n.main_at_catch_kg / 1e3, n.flight_s / 3600.0);
}

}  // namespace starsim::refuelling_test
```

(add `#include <cstdio>`). Append to `test_refuelling_missions.cpp` (include `refuelling_measurements.hpp`):

```cpp
namespace {
// Flies until the mission ends: 1 s steps while something is ticked anyway, 600 s steps otherwise (advance_to coasts).
void fly_mission(refuelling_test::DepotSession& f, double limit_s) {
    const time::Epoch start = f.session.epoch();
    while (f.session.epoch() - start < limit_s && f.session.iss_mission_leg() != IssMissionLeg::Complete &&
           f.session.iss_mission_leg() != IssMissionLeg::Ended) {
        f.session.advance_to(f.session.epoch() + 600.0);
    }
}
}  // namespace

TEST_CASE("tanker_flight_orbit: phasing orbit -> docked -> transfer -> caught at Starbase",
          "[refuelling][mission][docking][ship][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    REQUIRE(sim::start_mission(f.session, refuelling_test::config_dir, "tanker_flight_orbit", f.t0, f.clock)
                .refusal.empty());
    fly_mission(f, 12.0 * 86400.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.session.iss_mission_leg() == IssMissionLeg::Complete);
    const auto n = refuelling_test::tanker_numbers(f.session);
    refuelling_test::print("tanker_flight_orbit", n);
    CHECK(n.delivered_oxidizer_kg + n.delivered_fuel_kg > 0.0);
    CHECK(n.main_at_catch_kg >= 0.0);  // the reserve held: caught, not starved
    CHECK(f.session.body(f.session.controlled_index()).caught());
}

TEST_CASE("tanker_flight: pad -> booster caught -> docked -> transfer -> ship caught",
          "[refuelling][mission][docking][ship][ascent][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto s = sim::start_mission(f.session, refuelling_test::config_dir, "tanker_flight", f.t0, f.clock);
    REQUIRE(s.refusal.empty());
    // the ascent and the booster return are ticked: 1 s calls until the booster is caught
    while (f.session.epoch() - s.epoch < 1200.0 && f.session.tower()->phase() != sim::CatchPhase::Caught) {
        f.session.advance_to(f.session.epoch() + 1.0);
    }
    REQUIRE(f.session.tower()->phase() == sim::CatchPhase::Caught);
    fly_mission(f, 12.0 * 86400.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.session.iss_mission_leg() == IssMissionLeg::Complete);
    const auto n = refuelling_test::tanker_numbers(f.session);
    refuelling_test::print("tanker_flight", n);
    CHECK(n.residue_after_ascent_kg > 0.0);
    CHECK(n.delivered_oxidizer_kg + n.delivered_fuel_kg > 0.0);
    CHECK(f.session.body(f.session.controlled_index()).caught());
    CHECK(f.session.body_count() == 2u);  // tanker + depot: the booster was lifted off the tower
}
```

- [ ] **Step 2: Run in the background**

`build/core/tests/starsim_core_tests.exe "tanker_flight_orbit*" > tmp/r5_t6_orbit.txt 2>&1` and
`build/core/tests/starsim_core_tests.exe "tanker_flight: pad*" > tmp/r5_t6_pad.txt 2>&1` as background runs (each
may take several minutes: ascent, docking and entry are ticked; check every 10 min). If a leg ends ("Mission beendet:
..."), read the status, run the leg's existing diagnostics (`STARSIM_DEBUG_RENDEZVOUS`, `_PROXIMITY`, `_DOCKING`,
`_DEORBIT`, `_FLIP`, `_CATCH`) and fix the cause. A **physical** shortfall (the reserve does not cover the return, the
phasing does not close in 48 h) is a finding: stop and report the numbers to the user, do not tune.

- [ ] **Step 3: Record the numbers**

Copy both printed lines into the ledger and into `docs/subsystems.md` (Task 9).

- [ ] **Step 4: Commit**

```bash
git add core/tests/test_refuelling_missions.cpp core/tests/refuelling_measurements.hpp
git commit -m "test(sim): tanker_flight campaigns from the phasing orbit and the pad to the catch"
git push origin main
```

---

### Task 7: Moon albedo and infrared, HLS thermal data

**Files:**
- Modify: `core/include/starsim/math/constants.hpp` (`moon_bond_albedo`, `moon_night_temperature_k`)
- Modify: `core/include/starsim/environment/thermal_radiation.hpp`, `core/src/environment/thermal_radiation.cpp`
- Modify: `config/vehicles/starship_hls.toml` (`[stage.thermal]`)
- Test: `core/tests/test_boil_off.cpp` (the r2 heat-balance tests: find the file with `[boil_off]` radiation cases)

**Interfaces:**
- Produces: `RadiationEnvironment` gains
  ```cpp
  math::Vec3 moon_direction;               // towards the Moon's centre
  double moon_distance_over_radius = 0.0;  // 0: no Moon in the environment
  double moon_albedo_phase = 0.0;          // as albedo_phase, at the sub-vehicle point on the Moon
  double moon_infrared_w_m2 = 0.0;         // emitted at the sub-vehicle point: max((1-A) S cos z, sigma T_night^4)
  ```
  `earth_orbit_radiation` fills them when `moon_from_vehicle_m` is given; `absorbed_flux_w_m2` adds the Moon's albedo
  and infrared with the plate view factor. The function keeps its name (comment: Earth and Moon).

- [ ] **Step 1: Write the failing tests**

Add to the r2 radiation test file:

```cpp
TEST_CASE("Moon radiation: dayside low orbit sees albedo and infrared, the night side a little infrared",
          "[boil_off][environment]") {
    using namespace starsim;
    const Vec3 moon_centre{3.8e8, 0.0, 0.0};                     // GCRF, from the Earth
    const Vec3 sun_from_earth{-1.496e11, 0.0, 0.0};              // Sun behind the Earth: the Moon's far side lit
    const double h = constants::moon_mean_radius_m + 100.0e3;
    // over the Moon's subsolar point (between Moon and Sun... on the Sun side: -x from the Moon centre)
    const Vec3 day = moon_centre + Vec3{-h, 0.0, 0.0};
    const Vec3 moon_from_day = moon_centre - day;
    const auto env = environment::earth_orbit_radiation(day, sun_from_earth - day, &moon_from_day);
    CHECK(env.moon_distance_over_radius == Catch::Approx(h / constants::moon_mean_radius_m));
    CHECK(env.moon_albedo_phase == Catch::Approx(1.0).margin(1e-6));
    const double s = env.solar_flux_w_m2;
    CHECK(env.moon_infrared_w_m2 == Catch::Approx((1.0 - constants::moon_bond_albedo) * s).epsilon(1e-9));
    // a plate facing the Moon: albedo + infrared with the view factor
    const double view = environment::plate_view_factor_to_sphere(0.0, env.moon_distance_over_radius);
    const double moon_part = 0.5 * constants::moon_bond_albedo * s * view + 0.8 * env.moon_infrared_w_m2 * view;
    const Vec3 to_moon = math::normalized(moon_from_day);
    CHECK(environment::absorbed_flux_w_m2(to_moon, env, 0.5, 0.8) ==
          Catch::Approx(moon_part).epsilon(1e-6));  // the Sun is behind the plate; the Earth's terms ~1e-4 W/m^2
    // night side: no albedo, infrared from the night temperature
    const Vec3 night = moon_centre + Vec3{h, 0.0, 0.0};
    const Vec3 moon_from_night = moon_centre - night;
    const auto dark = environment::earth_orbit_radiation(night, sun_from_earth - night, &moon_from_night);
    CHECK(dark.moon_albedo_phase == 0.0);
    CHECK(dark.moon_infrared_w_m2 == Catch::Approx(constants::stefan_boltzmann_w_m2k4 *
                                                   std::pow(constants::moon_night_temperature_k, 4)));
}

TEST_CASE("Moon radiation is negligible in low Earth orbit", "[boil_off][environment]") {
    using namespace starsim;
    const Vec3 r{constants::earth_mean_radius_m + 200.0e3, 0.0, 0.0};
    const Vec3 sun{0.0, 1.496e11, 0.0}, moon{0.0, -3.84e8, 0.0};
    const auto with = environment::earth_orbit_radiation(r, sun - r, &moon);  // moon_from_vehicle as passed in r2
    const Vec3 n{0.0, -1.0, 0.0};                                            // towards the Moon
    const double moon_view = environment::plate_view_factor_to_sphere(0.0, with.moon_distance_over_radius);
    CHECK(moon_view < 3e-5);
}
```

(The day-side case: the plate faces the Moon, the Sun lies behind the plate (`dot(normal, sun_direction) < 0`), the
Earth is ~3.8e8 m away - check its albedo/IR terms are below 1e-3 W/m^2 or add them to `moon_part` from the same
formulas.)

- [ ] **Step 2: Run to verify they fail**

Expected: compile errors (`moon_bond_albedo`, the new fields).

- [ ] **Step 3: Implement**

`constants.hpp` next to `earth_outgoing_longwave_w_m2`:

```cpp
// [S] Vasavada et al. 2012, JGR 117 E00H18 (LRO Diviner): bolometric Bond albedo ~0.11 (0.07-0.16 maria/highlands).
inline constexpr double moon_bond_albedo = 0.11;
// [S] Diviner equatorial night-time surface temperature before dawn ~95 K (Vasavada et al. 2012).
inline constexpr double moon_night_temperature_k = 95.0;
```

`thermal_radiation.cpp`, in `earth_orbit_radiation` after the Earth terms:

```cpp
    if (moon_from_vehicle_m != nullptr) {
        const double d = norm(*moon_from_vehicle_m);
        env.moon_direction = *moon_from_vehicle_m / d;
        env.moon_distance_over_radius = d / constants::moon_mean_radius_m;
        // the sub-vehicle point's solar zenith: its outward normal is -moon_direction [E: cap like its centre]
        const double cos_z = std::max(0.0, -dot(env.moon_direction, env.sun_direction));
        env.moon_albedo_phase = cos_z;
        // radiative equilibrium of the regolith (no conduction: [E], dayside within a few K of Diviner), the night floor
        const double night = constants::stefan_boltzmann_w_m2k4 * std::pow(constants::moon_night_temperature_k, 4);
        env.moon_infrared_w_m2 = std::max((1.0 - constants::moon_bond_albedo) * env.solar_flux_w_m2 * cos_z, night);
    }
```

`absorbed_flux_w_m2` adds, when `env.moon_distance_over_radius > 0`:

```cpp
    double moon = 0.0, moon_ir = 0.0;
    if (env.moon_distance_over_radius > 0.0) {
        const double view_m = plate_view_factor_to_sphere(
            std::acos(std::clamp(dot(normal, env.moon_direction), -1.0, 1.0)), env.moon_distance_over_radius);
        moon = constants::moon_bond_albedo * env.solar_flux_w_m2 * env.moon_albedo_phase * view_m;
        moon_ir = env.moon_infrared_w_m2 * view_m;
    }
    return absorptivity * (direct + albedo + moon) + emissivity * (constants::earth_outgoing_longwave_w_m2 * view + moon_ir);
```

Update the header comment ("The Moon only shadows" -> its albedo and infrared [E] as above). The orbit-average path
(`boil_off.cpp:173`) passes the Moon too if it has it; leave it otherwise (LEO: negligible, test 2).

`starship_hls.toml`, after `[stage.rcs]` pods (plan decision 7):

```toml
# Thermal [E] (refuelling r5, plan decision 7): a long-duration ship insulated like the depot - SpaceX publishes
# nothing; bare Block 3 steel (absorptivity 0.40, emissivity 0.15, wall 0.01) would boil ~30 t/day full.
[stage.thermal]
absorptivity = 0.30                      # [E] as starship_depot
emissivity = 0.80                        # [E]
wall_resistance_m2k_w = 63.7             # [D] as starship_depot (Centaur-derived MLI, r2)
ullage_resistance_m2k_w = 100.0          # [E] dry wall: ullage gas conduction, no convection near free fall
```

- [ ] **Step 4: Run the affected tags**

`ctest --test-dir build -L "boil_off|environment|hls|refuelling|lunar_mission_flight|lunar_budget_mission|lunar_campaign" -j 24 --output-on-failure > tmp/r5_t7.txt 2>&1`
in the background (the lunar `[slow]` campaigns now boil the HLS: their budgets change). If a configured lunar
landing mission no longer closes or a lunar campaign fails on propellant, **stop and report** the numbers (boil-off
per leg, margin) to the user - physics, not a test to adjust. Record the HLS boil-off rates in LEO (full) and in LLO.
Then run the Block 3 lunar return once with boil-off forced on (a hidden probe case `[.r5_probe]` that loads
`ship_lunar_return_llo`, sets `boil_off = true` through the mission struct and flies it; print the shortfall) - report,
keep the flags.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/math/constants.hpp core/include/starsim/environment/thermal_radiation.hpp core/src/environment/thermal_radiation.cpp config/vehicles/starship_hls.toml core/tests
git commit -m "feat(environment): lunar albedo and infrared in the thermal environment; HLS insulated like the depot"
git push origin main
```

---

### Task 8: hls_refuel - HLS launch stack, lunar continuation from the live state, campaign

**Files:**
- Create: `config/vehicles/starship_hls_launch.toml`, `config/missions/hls_ascent.toml`,
  `config/missions/hls_lunar_llo_depot.toml`, `config/missions/hls_refuel.toml`, `config/missions/hls_refuel_orbit.toml`
- Modify: `core/include/starsim/sim/lunar_landing_mission.hpp`, `core/src/sim/lunar_landing_mission.cpp`
  (`continue_lunar_landing_mission`, `lunar_parking_state`)
- Modify: `core/src/sim/refuelling_mission.cpp` (hls_refuel: depot in the lunar plane, continuation)
- Test: `core/tests/test_refuelling_missions.cpp`, `core/tests/test_lunar_landing_mission.cpp` (the refactor keeps it)

**Interfaces:**
- Produces:
  ```cpp
  // The lunar mission's circular parking state at `epoch` (the plane through the Moon at arrival, as the start places
  // it); std::nullopt when the Moon's declination exceeds the inclination.
  std::optional<math::StateVector> lunar_parking_state(const FlightSession& session, const config::LunarLandingMission& m,
                                                       time::Epoch epoch, double inclination_rad);
  // The lunar landing mission from the controlled body as it flies (refuelling r5: the HLS after the depot): terrain
  // and site set, the outbound flight planned from its live state, the budget closed with its real mass. Nothing is
  // placed. Refusal (German) in MissionStart::refusal.
  MissionStart continue_lunar_landing_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                              const config::LunarLandingMission& mission, time::Epoch epoch);
  ```
  `start_lunar_landing_mission` = placement + `continue_lunar_landing_mission` (same results as before).

- [ ] **Step 1: Configs**

`config/vehicles/starship_hls_launch.toml`:
```toml
# Refuelling r5: the HLS on Super Heavy for its launch to the depot (plan decision 7). The stages are taken over
# unchanged ([[stage]] from = "<vehicle>:<stage>", r1); the HLS stage flies its lunar mission after the refuelling.
[vehicle]
id = "starship_hls_launch"
name = "Super Heavy Block 3 / Starship HLS"
display_name = "Lunar Lander Mk3 (Start)"
propellant = "methalox"
stages_bottom_to_top = ["booster", "hls"]

[[stage]]
from = "starship_block3:booster"

[[stage]]
from = "starship_hls:hls"
```

`config/missions/hls_ascent.toml`: as `tanker_ascent.toml` with `name = "HLS in den Phasenorbit"`,
`vehicle = "starship_hls_launch"`.

`config/missions/hls_lunar_llo_depot.toml`: a copy of `hls_lunar_llo.toml` with `name = "HLS-Mondlandung (LLO) ab
Depot"`, `inclination_deg = 26.5` (the depot plane) and the comment "[orbit] is the depot orbit; the start is
continued from the refuelled HLS (hls_refuel)". Its `epoch_utc`: found in Step 3.

`config/missions/hls_refuel.toml`:
```toml
# Refuelling r5: the HLS launches into the depot plane, docks, is filled and undocks; at the lunar mission's epoch the
# lunar landing continues from the HLS's real state and tank load (spec r5 "hls_refuel").
[mission]
name = "HLS: Betankung am Depot und Mondmission"
kind = "hls_refuel"
ascent = "hls_ascent"
port = "transfer_dock"

[depot]
vehicle = "starship_depot"
port = "transfer_dock"
height_km = 200.0
inclination_deg = 26.5            # must equal the lunar mission's [orbit] inclination
argument_of_latitude_deg = 0.0    # at earliest_utc; the RAAN comes from the lunar plane
propellant_t = 1500.0             # [test] a filled depot after the campaign (r6 computes how many tankers)

[start]
mode = "pad"

[launch_window]
earliest_utc = "%EARLIEST%"       # the lunar epoch minus 6 days (Step 3)
search_days = 4.0                 # [E] ends before the lunar epoch with the 48 h phasing
insertion_s = 520.0               # [test] calibrated from the flown HLS ascent in Step 5
insertion_arc_deg = 20.0          # [test]

[phasing]
mission_limit_h = 48.0            # [E] uncrewed
earliest_arrival_h = 0.0

[lunar]
mission = "hls_lunar_llo_depot"
```

`hls_refuel_orbit.toml`: the same with `[start] mode = "phasing_orbit"`, `behind_deg = 8.0`, `propellant_t` = the HLS
residue after its ascent (Step 5, [D]; 100.0 [test] until then), `earliest_utc` = the lunar epoch minus 2 days.

- [ ] **Step 2: Write the failing tests**

```cpp
namespace {
struct LunarDepotSession : refuelling_test::DepotSession {
    const spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession lunar{ephemeris, earth, refuelling_test::config_dir, {}, &moon};
};
}  // namespace

TEST_CASE("continue_lunar_landing_mission plans like the start from the same placed vehicle", "[sim][lunar][refuelling]") {
    test::spice_session();
    LunarDepotSession f;
    const auto m = config::load_lunar_landing_mission(refuelling_test::config_dir, "hls_lunar_llo");
    const auto epoch = f.clock.from_utc(m.epoch_utc);
    const auto a = sim::start_lunar_landing_mission(f.lunar, refuelling_test::config_dir, m, epoch);
    REQUIRE(a.refusal.empty());
    const double tli_a = norm(f.lunar.lunar_outbound_plan()->tli.delta_v_gcrf_mps);
    sim::FlightSession b{f.ephemeris, f.earth, refuelling_test::config_dir, {}, &f.moon};
    sim::place_lunar_landing_mission(b, m, refuelling_test::config_dir, epoch);
    const auto c = sim::continue_lunar_landing_mission(b, refuelling_test::config_dir, m, epoch);
    REQUIRE(c.refusal.empty());
    CHECK(norm(b.lunar_outbound_plan()->tli.delta_v_gcrf_mps) == Catch::Approx(tli_a).epsilon(1e-9));
}

TEST_CASE("Continuation after the lunar epoch is refused", "[sim][lunar][refuelling]") {
    test::spice_session();
    LunarDepotSession f;
    const auto m = config::load_lunar_landing_mission(refuelling_test::config_dir, "hls_lunar_llo_depot");
    const auto epoch = f.clock.from_utc(m.epoch_utc);
    sim::place_lunar_landing_mission(f.lunar, m, refuelling_test::config_dir, epoch + 3600.0);
    const auto c = sim::continue_lunar_landing_mission(f.lunar, refuelling_test::config_dir, m, epoch);
    CHECK_THAT(c.refusal, Catch::Matchers::ContainsSubstring("zu spät"));
}

TEST_CASE("hls_refuel_orbit: docked, filled, departed, the lunar mission continues with the real tank load",
          "[refuelling][mission][lunar][slow]") {
    test::spice_session();
    LunarDepotSession f;
    const auto s = sim::start_mission(f.lunar, refuelling_test::config_dir, "hls_refuel_orbit", f.t0, f.clock);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    const time::Epoch start = f.lunar.epoch();
    while (f.lunar.epoch() - start < 4.0 * 86400.0 && f.lunar.iss_mission_leg() != IssMissionLeg::Complete &&
           f.lunar.iss_mission_leg() != IssMissionLeg::Ended) {
        f.lunar.advance_to(f.lunar.epoch() + 600.0);
    }
    INFO(f.lunar.iss_mission_status());
    REQUIRE(f.lunar.iss_mission_leg() == IssMissionLeg::Complete);
    REQUIRE(f.lunar.lunar_outbound_plan());
    const auto& hls = f.lunar.body(f.lunar.controlled_index()).vehicle();
    const auto* transfer = refuelling_test::mark(f.lunar, IssMissionLeg::Transfer);
    REQUIRE(transfer);
    std::printf("[r5] hls_refuel_orbit: HLS main at docking %.1f t, after the fill %.1f t, depot left %.1f t, "
                "TLI %.0f m/s\n",
                transfer->chaser_main_propellant_kg / 1e3, hls.main_propellant_kg() / 1e3,
                f.lunar.body(*f.lunar.station_index()).vehicle().main_propellant_kg() / 1e3,
                norm(f.lunar.lunar_outbound_plan()->tli.delta_v_gcrf_mps));
    CHECK(hls.main_propellant_kg() > transfer->chaser_main_propellant_kg);
}

TEST_CASE("hls_refuel: pad -> docked -> filled -> lunar mission planned", "[refuelling][mission][lunar][ascent][slow]") {
    test::spice_session();
    LunarDepotSession f;
    const auto s = sim::start_mission(f.lunar, refuelling_test::config_dir, "hls_refuel", f.t0, f.clock);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    while (f.lunar.epoch() - s.epoch < 1200.0 && f.lunar.tower()->phase() != sim::CatchPhase::Caught) {
        f.lunar.advance_to(f.lunar.epoch() + 1.0);
    }
    const time::Epoch start = f.lunar.epoch();
    while (f.lunar.epoch() - start < 8.0 * 86400.0 && f.lunar.iss_mission_leg() != IssMissionLeg::Complete &&
           f.lunar.iss_mission_leg() != IssMissionLeg::Ended) {
        f.lunar.advance_to(f.lunar.epoch() + 600.0);
    }
    INFO(f.lunar.iss_mission_status());
    REQUIRE(f.lunar.iss_mission_leg() == IssMissionLeg::Complete);
    REQUIRE(f.lunar.lunar_outbound_plan());
    const auto* rv = refuelling_test::mark(f.lunar, IssMissionLeg::Rendezvous);
    REQUIRE(rv);
    std::printf("[r5] hls_refuel: HLS residue after ascent %.1f t, main at the lunar start %.1f t\n",
                rv->chaser_main_propellant_kg / 1e3,
                f.lunar.body(f.lunar.controlled_index()).vehicle().main_propellant_kg() / 1e3);
}
```

(`start_mission`'s `requested` is `f.t0`; the refuelling kinds use their config epochs - see Task 4's convention.)

- [ ] **Step 3: Implement the lunar split and find the lunar epoch**

`lunar_landing_mission.cpp`: `place()` computes the parking state through a new public

```cpp
std::optional<math::StateVector> lunar_parking_state(const FlightSession& session, const config::LunarLandingMission& m,
                                                     time::Epoch epoch, double inclination_rad) {
    const mission::LunarTransferTargeter targeter(session.gravity(), session.ephemeris(), *session.moon_orientation());
    const auto raan = targeter.parking_orbit_raan(epoch + m.flight_time_s, inclination_rad);
    if (!raan) return std::nullopt;
    const orbit::KeplerianElements elements{constants::wgs84_semi_major_axis_m + m.parking_height_m, 0.0,
                                            inclination_rad, *raan, 0.0, m.argument_of_latitude_rad};
    return orbit::state_from_elements(elements, session.gm_earth_m3ps2());
}
```

(`place()` uses it with `m.parking_inclination_rad`; the terrain / site loading moves to a file-local
`set_lunar_site(session, m, config_dir)` used by both `place()` and the continuation.)
`continue_lunar_landing_mission` = the body of `start_lunar_landing_mission` after `place()`: first
`if (session.epoch() > epoch + 0.5 * FlightSession::tick_s) return refuse("Mondmission: zu spät - die Sitzung steht nach dem Start der Mondmission");`,
then `set_lunar_site` (`std::runtime_error` -> refusal), the touchdown check, `plan_lunar_outbound`, the budget, the
sequence. `start_lunar_landing_mission` = `place()` (catching its `runtime_error`) + `continue_lunar_landing_mission`.
`refuse` must not clear bodies (the HLS is the user's refuelled ship); it disengages the autopilot as now.

Find the epoch: a hidden probe `[.r5_probe]` that prints `lunar_parking_state(...)` existence for
`hls_lunar_llo_depot` at 2026-11-02 + k days, k = 0..30, inclination 26.5 deg; take the first k with a state at least
6 days after 2026-10-20 (the HLS needs launch + phasing + fill) and write it as `epoch_utc` ([D] r5: Moon declination
at arrival < 26.5 deg). `hls_refuel.toml` `earliest_utc` = that epoch - 6 days, `hls_refuel_orbit.toml` = - 2 days.

- [ ] **Step 4: Implement hls_refuel in the start**

In `start_refuelling_flight_mission`, for `HlsRefuel` before the depot is built:

```cpp
    config::LunarLandingMission lunar;
    time::Epoch lunar_epoch{};
    auto depot_cfg = m.depot;
    if (m.kind == config::MissionKind::HlsRefuel) {
        if (!session.moon_orientation()) {
            s.refusal = "HLS-Betankung: die Sitzung hat keine Mondorientierung";
            return s;
        }
        lunar = config::load_lunar_landing_mission(config_dir, m.lunar_mission_id);
        if (std::abs(lunar.parking_inclination_rad - m.depot.inclination_rad) > 1e-9) {
            s.refusal = "HLS-Betankung: Depotbahn und Parkbahn der Mondmission haben verschiedene Neigungen";
            return s;
        }
        lunar_epoch = time.from_utc(lunar.epoch_utc);
        const auto parking = lunar_parking_state(session, lunar, lunar_epoch, lunar.parking_inclination_rad);
        if (!parking) {
            s.refusal = "Kein Parkorbit durch die Mondbahnebene: Bahnneigung unter der Deklination des Mondes";
            return s;
        }
        depot_cfg = depot_in_plane(m.depot, *parking, lunar_epoch, time.from_utc(m.earliest_utc), session.earth_orientation());
    }
```

with a file-local `depot_in_plane`: rotate the GCRF parking state into the true equator frame of `lunar_epoch`
(`transpose(orbit::true_equator_to_gcrf(earth, lunar_epoch))`), take its elements (`orbit::elements_from_state`), build
`StationMeanElements` at `lunar_epoch` (a, i, RAAN from them; argument of latitude from the config), advance them by
J2 to `earliest` (`orbit::advance_mean_elements`) and return a `DepotOrbitConfig` with that inclination, RAAN and the
config's argument of latitude at `earliest` (so `depot_orbit(depot_cfg, earliest, earth)` is the depot in the lunar
plane at the lunar epoch). Use `depot_cfg` instead of `m.depot` below. Before `session.set_iss_mission(q)`:

```cpp
    if (m.kind == config::MissionKind::HlsRefuel) {
        q.continue_epoch = lunar_epoch;
        q.continuation = [config_dir, lunar, lunar_epoch](FlightSession& session) {
            return continue_lunar_landing_mission(session, config_dir, lunar, lunar_epoch).refusal;
        };
    }
```

and a refusal at the start when `s.epoch >= lunar_epoch` ("HLS-Betankung: Startfenster nach dem Start der
Mondmission").

- [ ] **Step 5: Run, calibrate the HLS ascent, run the campaigns**

1. `ctest --test-dir build -L "lunar_budget_mission|refuelling|sim" -j 24 --output-on-failure > tmp/r5_t8.txt 2>&1`
   (the fast cases; `start_lunar_landing_mission` unchanged in results).
2. Fly the HLS ascent once (the pad case up to the Rendezvous mark, `STARSIM_VERBOSE` printing SECO time, arc and
   residue as in Task 5's calibration case - add an `hls_refuel` variant of that case) and write `insertion_s`,
   `insertion_arc_deg` into `hls_refuel*.toml` and the residue into `hls_refuel_orbit.toml` ([D]).
3. The two `[slow]` campaigns in the background (`tmp/r5_t8_orbit.txt`, `tmp/r5_t8_pad.txt`), plus the lunar tags
   `lunar_mission_flight|lunar_campaign` (the refactor). If the HLS ascent cannot reach orbit with 1,600 t on Super
   Heavy, or the lunar budget does not close with the real tank load: stop and report the numbers.

- [ ] **Step 6: Commit**

```bash
git add config/vehicles/starship_hls_launch.toml config/missions core/include/starsim/sim/lunar_landing_mission.hpp core/src/sim/lunar_landing_mission.cpp core/src/sim/refuelling_mission.cpp core/tests
git commit -m "feat(sim): hls_refuel - HLS launch stack, depot in the lunar plane, lunar mission continued from the real tank load"
git push origin main
```

---

### Task 9: Documentation, measured numbers, final review

**Files:**
- Modify: `docs/subsystems.md` ("Reference flights (r5)" under "Orbital refuelling (milestone r)")
- Modify: `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md` (status line: r5 implemented; corrections:
  the plan decisions as approved)
- Modify: `CLAUDE.md` (module summary: the mission chain's new legs, the three missions; Moon radiation in
  `environment`)
- Modify: `config/README.md` (already in Task 3: check), `docs/ansichten.md` (the three missions and the legs
  Transfer / Warten auf Fortsetzung if the mission view lists legs)

- [ ] **Step 1: Write the subsystem section**

`docs/subsystems.md`, "### Reference flights (r5)": plan path; the chain (legs, marks, `make_station`,
`release_tower`); the missions and their configs; the window calibration [D] numbers (tanker, HLS); the measured
numbers from Task 4, 6, 8 (depot fill after ascent, depot boil-off/day, tanker residue, rendezvous propellant,
delivered LOX/CH4, transfer duration and gas, main tanks at undock and at the catch, flight time; HLS residue, HLS
after the fill, TLI); the window finding with the 48 h limit; HLS boil-off LEO / LLO, the lunar campaigns' new budget
margins; the Block 3 return probe result; tests and tags; deferred minors.

- [ ] **Step 2: Bindings and smoke test**

`powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoTests > tmp/r5_bindings.txt 2>&1` (the bindings build
with the new enum values: the `iss_mission` getter's leg names come from `to_string`) and
`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd > tmp/r5_smoke.txt 2>&1`.
`FlightSim.start_mission` for the new kinds is r6 (spec "Game"); r5 only keeps the build green.

- [ ] **Step 3: Commit**

```bash
git add docs CLAUDE.md config/README.md
git commit -m "docs: refuelling r5 reference flights"
git push origin main
```

- [ ] **Step 4: Final review**

Whole-plan review (most capable model) of `git diff <r5 start>..HEAD`; fix Critical / Important, re-run the affected
tags, push, update the ledger and memory, ask the user for `/clear` naming r6 (campaign calculator, game interface).
