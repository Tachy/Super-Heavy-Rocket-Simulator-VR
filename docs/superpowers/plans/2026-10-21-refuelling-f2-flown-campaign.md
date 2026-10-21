# Refuelling f2 - Fully Flown Campaign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fly the whole tanker campaign of a lunar mission in one `FlightSession` with continuous world time - depot
launch, every tanker, the depot's station keeping, the HLS refuel - planned live like a mission planner, and compare
it with the campaign calculator (same tanker count, duration within +-1 day, depot fill per arrival within 2 %).

**Architecture:** A new `sim::RefuellingCampaignFlight` is installed on the session as a `SessionSupervisor` (a new,
small hook in `FlightSession::advance_to`: called before every tick / coast segment, the coast stops at its wake
epoch). It re-plans after every flight with the existing calculator (`plan_refuelling_campaign`, extended by a live
start `CampaignLive`: the flown depot's state and fill, the last flight), warps to the launch, flies the flight with
the existing r5 mission chain (`IssMissionSequence`) beside the kept station (`keep_station`), removes it after its
departure, flies the depot's reboosts (`FlightSession::start_station_reboost`) and books the calculator's [E] items
(attitude gas, transfer vent). The first plan, made when the depot becomes the station, is the calculator's answer.

**Tech Stack:** C++20 core (`core/`), Catch2 tests, TOML configs, godot-cpp bindings (`bindings/`).

**Spec:** `docs/superpowers/specs/2026-10-21-refuelling-followups-design.md`, section "f2 - Fully flown campaign"
(predecessor spec `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md`). Subsystem notes:
`docs/subsystems.md` refuelling sections (read "Campaign calculator (r6)", "Phasing orbit per window (f1)",
"Settling (f3)" before starting).

## Global Constraints

- SI units internally, angles in radians; frame and unit in every name (`position_gcrf_m`, `epoch`, `height_m`).
- Code, identifiers, comments, commits English; UI / status / refusal strings German (UTF-8 sources).
- `core` has no Godot dependency; bindings stay thin (no logic beyond dictionary building).
- No magic numbers: every estimate is tagged `[E]`, derived values `[D]`, in config or a named `constexpr` with a comment.
- Physics never in GDScript; the game only reads state and sends commands (autopilot only, UI by the user).
- Modules depend downwards only: `orbit` < `mission` < `sim`; `mission::refuelling_campaign` stays pure.
- Commit on `main` (Conventional Commits, attribution lines from the session), push after every task with its tests
  green.
- Tests: run only the affected tags (`-TestFilter`), `[slow]` included; a case over ~40 s is tagged `[slow]`; runs
  over 10 minutes go to the background with output to a file, checked every 10 minutes.
- Tests run as parallel processes: no shared temp files.
- `config/refuelling/reference_flights.toml` has an `inputs_digest` over the configs the reference flights read: do
  NOT change those configs (vehicles, the reference missions, ascents) in this plan - new settings go into the new
  campaign mission files.
- Pass criteria of the flown campaign (spec): same tanker count as the calculator, duration (campaign start -> HLS
  at Waypoint 0) within +-1 day [E], depot fill after each tanker's transfer within 2 % [E] of the calculator's.
  Deviations caused by the calculator are corrected there and documented in `docs/subsystems.md`; tolerances are
  never widened to make the check pass (ask the user).

## Plan decisions (await the user's approval)

1. **Supervisor hook.** The campaign driver runs on the simulation thread inside `FlightSession::advance_to`
   (`SessionSupervisor::update` before every tick / coast, `wake_epoch` bounds the coast) - so the game's
   `SessionRunner`, the tests and the CLI drive it the same way. `clear_bodies()` and every `start_mission` remove it.
2. **Flights one after another.** The session flies one chaser at a time: the next flight is planned when the
   previous one has departed (or failed). The calculator's rows today never overlap (smallest gap: arrival 18.38 d ->
   next launch 18.94 d); if a live plan's window lies before the previous flight's end, the next window from now is
   flown and the row notes it.
3. **First tanker's return in a side session.** Its return (deorbit pass search up to 8 days) would overlap the next
   tanker's flight; it is flown synchronously in its own `FlightSession` from the tanker's state at departure (a copy
   of the vehicle with its tank load), to the catch at Starbase. The worker thread is blocked for that flight
   (documented, as r6's ~1 s campaign command).
4. **Calculator with a live start and the flown depot.** The launched depot's plane and phase are not tanker_flight's
   `[depot]` (RAAN 0 at 2026-10-20). The calculator gets `CampaignLive` (epoch, depot state, fill, the last flight's
   launch and arrival, the cached HLS demand); its depot orbit then comes from the flown state. The comparison's
   calculator answer is the live plan made at the moment the depot becomes the station. Without `CampaignLive`
   everything stays as today (13 tankers / 49.7 d unchanged).
5. **Energy-mean semi-major axis [D].** The J2 field is axisymmetric, so E = v^2/2 - mu/r + U_J2 is conserved; the
   mean a follows E - <U_J2> (the osculating one swings +-2 km at 200 km / 26.5 deg). It drives the station-keeping
   trigger and the live depot elements (e, i, RAAN, argument of latitude stay osculating [E], ~0.01 deg).
6. **Station keeping.** Mean height below 195 km [E] -> two burns to 200 km circular [E] with the raise's rules
   (`gnc::next_raise_node`, then the periapsis at the new apoapsis; the belly Raptor `sea_level_3` when its transients
   are at most 60 % of the burn, else cold gas from the main tanks) and the f3 settling push; only while no flight is
   under way and the next arrival is more than 24 h [E] away. The depot is the controlled body for the burns and the
   station again afterwards (also after a refused or aborted burn). Engine groups come from the campaign mission file
   (not the depot vehicle file: its digest belongs to the reference flights).
7. **Booked [E] items as in the calculator.** Attitude gas 20 kg/day from the depot's main tanks (the flown depot
   holds an ideal attitude; booked outside running transfers), transfer vent 1.5 % of each transfer's received amount
   from the receiver. Values from `mission::CampaignInputs{}` (one source).
8. **A failed flight** (its mission leg `Ended`) is recorded with its reason, removed, and the campaign re-plans from
   the live state (it then needs another tanker - the comparison reports the difference honestly).
9. **The HLS** is filled as in `hls_refuel` (station -> chaser up to full), departs, and the campaign is complete (no
   lunar mission, spec "out of scope"). Its check: propellant after the vent booking >= the calculator's required
   propellant.
10. **Placed-depot variant** for the shorter test: `refuelling_campaign_flown_placed` places the depot on
    tanker_flight's `[depot]` orbit at its `earliest_utc` with a fill of "demand - 1.5 net tanker deliveries" [test]
    -> 2 tankers + HLS.
11. **CLI**: `refuelling_report --fly` (launched depot) / `--fly-placed` runs the flown campaign and prints the
    comparison table.

## Review Focus

- A single time-warp call across a planned liftoff (`advance_to(now + 10 days)`) must launch at the liftoff epoch
  (within one tick), not at the end of the call - Task 2 (hook) and Task 7 (driver) tests.
- A flight that ends early (capture retries exhausted, a refused leg) must not stall the campaign: recorded as failed,
  removed, re-planned - Task 7 test "a failed tanker flight is recorded and the campaign re-plans".
- A reboost whose burn is refused or aborted (an almost empty depot) must leave the depot a station in its ideal
  attitude, never an uncontrolled free body - Task 3 test.
- Starting any other mission (or clearing the session) during a campaign must remove the supervisor: no ghost driver
  launching tankers into a new mission - Task 2 / Task 7 tests.
- `keep_station` at an epoch other than the session's, or without a station, must refuse loudly (std::logic_error),
  never silently drop the depot - Task 2 test.

---

## File Structure

| File | Responsibility |
|------|----------------|
| `core/include/starsim/orbit/synthetic_station_orbit.hpp`, `core/src/orbit/synthetic_station_orbit.cpp` | + `energy_mean_semi_major_axis_m`, `station_mean_elements_from_state` |
| `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` | + `SessionSupervisor`, `set_supervisor`, `keep_station` on launch / place, `keep_only_station`, `place_vehicle` |
| `core/src/sim/flight_session_reboost.cpp` (new) | station reboost state machine (`start_station_reboost`, `run_reboost`) |
| `core/include/starsim/sim/ascent_mission.hpp`, `core/src/sim/ascent_mission.cpp` | `start_ascent_mission(..., keep_station)` |
| `core/include/starsim/mission/refuelling_campaign.hpp`, `core/src/mission/refuelling_campaign.cpp` | previous launch / arrival, earliest launch; `CampaignFlight::window` |
| `core/include/starsim/sim/refuelling_campaign_setup.hpp`, `core/src/sim/refuelling_campaign_setup.cpp` | `CampaignLive`, `CampaignHlsDemand`, window parameters in the report |
| `core/include/starsim/sim/refuelling_mission.hpp`, `core/src/sim/refuelling_mission.cpp` | `station_orbit`, `pad_flight_sequence`, `launch_campaign_flight` |
| `core/include/starsim/config/refuelling_mission_config.hpp`, `core/src/config/mission_config.cpp`, `core/include/starsim/config/mission_config.hpp` | `RefuellingCampaignFlownMission`, `MissionKind::RefuellingCampaignFlown` |
| `config/missions/refuelling_campaign_flown.toml`, `config/missions/refuelling_campaign_flown_placed.toml` (new) | the two campaign missions |
| `core/include/starsim/sim/refuelling_campaign_flight.hpp`, `core/src/sim/refuelling_campaign_flight.cpp` (new) | the driver |
| `core/src/sim/mission_start.cpp` | dispatch + supervisor reset |
| `core/include/starsim/sim/refuelling_campaign_text.hpp`, `core/src/sim/refuelling_campaign_text.cpp`, `core/apps/refuelling_report.cpp` | comparison text, `--fly` |
| `bindings/src/flight_snapshot_refuelling.cpp`, `bindings/src/flight_snapshot.hpp`, `bindings/src/flight_sim.{hpp,cpp}` | `campaign_flight_state()` |
| `core/tests/test_refuelling_campaign_flight.cpp` (new), `core/tests/test_synthetic_station_orbit.cpp`, `core/tests/test_refuelling_campaign.cpp`, `core/tests/test_refuelling_missions.cpp`, `core/tests/test_refuelling_mission_config.cpp` | tests |
| `core/CMakeLists.txt`, `core/tests/CMakeLists.txt` | new sources / test file |
| `docs/subsystems.md`, `CLAUDE.md` | docs |

Test tags: `[campaign_flown]` for everything new in `sim`, plus the module tags (`[orbit]`/`[synthetic_station]` as
the file uses, `[refuelling][campaign]`, `[refuelling][reboost]`, `[config]`).

Build / test command (PowerShell): `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "<tags>"`
(output to a file in the scratchpad, read the tail).

---

### Task 1: Energy-mean semi-major axis and mean elements from a flown state

**Files:**
- Modify: `core/include/starsim/orbit/synthetic_station_orbit.hpp`
- Modify: `core/src/orbit/synthetic_station_orbit.cpp`
- Test: `core/tests/test_synthetic_station_orbit.cpp`

**Interfaces:**
- Produces:
  - `double orbit::energy_mean_semi_major_axis_m(const math::StateVector& state_gcrf, const math::Vec3& pole_gcrf, double gm_m3ps2, double j2, double radius_m);`
  - `orbit::StationMeanElements orbit::station_mean_elements_from_state(const math::StateVector& state_gcrf, time::Epoch epoch, double gm_m3ps2, double j2, double radius_m, const frames::BodyOrientation& earth);`

- [ ] **Step 1: Write the failing tests** (append to `core/tests/test_synthetic_station_orbit.cpp`; reuse its includes,
  add `<algorithm>` if missing). A local RK4 with point mass + J2 about +z keeps the test inside `orbit`:

```cpp
namespace {

// Point mass + J2 about +z (the test's true pole), RK4 [test].
math::Vec3 j2_acceleration(const math::Vec3& r) {
    const double mu = constants::earth_gm_m3ps2, re = constants::wgs84_semi_major_axis_m, j2 = constants::earth_j2;
    const double rn = norm(r), z2 = (r.z / rn) * (r.z / rn);
    const double k = 1.5 * j2 * mu * re * re / std::pow(rn, 5);
    return -mu / (rn * rn * rn) * r +
           math::Vec3{k * r.x * (5.0 * z2 - 1.0), k * r.y * (5.0 * z2 - 1.0), k * r.z * (5.0 * z2 - 3.0)};
}

math::StateVector rk4_step(const math::StateVector& s, double h) {
    const auto f = [](const math::StateVector& x) { return math::StateVector{x.velocity_mps, j2_acceleration(x.position_m)}; };
    const auto add = [](const math::StateVector& a, const math::StateVector& d, double k) {
        return math::StateVector{a.position_m + k * d.position_m, a.velocity_mps + k * d.velocity_mps};
    };
    const auto k1 = f(s), k2 = f(add(s, k1, h / 2)), k3 = f(add(s, k2, h / 2)), k4 = f(add(s, k3, h));
    return {s.position_m + h / 6 * (k1.position_m + 2 * k2.position_m + 2 * k3.position_m + k4.position_m),
            s.velocity_mps + h / 6 * (k1.velocity_mps + 2 * k2.velocity_mps + 2 * k3.velocity_mps + k4.velocity_mps)};
}

// 200 km circular, 26.5 deg, node on +x.
math::StateVector leo_state() {
    const double r = constants::wgs84_semi_major_axis_m + 200.0e3, i = 26.5 * constants::deg_to_rad;
    const double v = std::sqrt(constants::earth_gm_m3ps2 / r);
    return {{r, 0.0, 0.0}, {0.0, v * std::cos(i), v * std::sin(i)}};
}

}  // namespace

TEST_CASE("Energy-mean semi-major axis holds over a J2 revolution where the osculating one swings",
          "[orbit][synthetic_station]") {
    const math::Vec3 pole{0.0, 0.0, 1.0};
    const double mu = constants::earth_gm_m3ps2, re = constants::wgs84_semi_major_axis_m, j2 = constants::earth_j2;
    auto s = leo_state();
    double osc_min = 1e30, osc_max = -1e30, mean_min = 1e30, mean_max = -1e30, osc_sum = 0.0, mean_sum = 0.0;
    int n = 0;
    for (double t = 0.0; t < 5400.0; t += 5.0, s = rk4_step(s, 5.0), ++n) {
        const double osc = orbit::elements_from_state(s, mu).semi_major_axis_m;
        const double mean = orbit::energy_mean_semi_major_axis_m(s, pole, mu, j2, re);
        osc_min = std::min(osc_min, osc), osc_max = std::max(osc_max, osc), osc_sum += osc;
        mean_min = std::min(mean_min, mean), mean_max = std::max(mean_max, mean), mean_sum += mean;
    }
    CHECK(osc_max - osc_min > 2000.0);             // the J2 short-period swing (+-2 km at 26.5 deg)
    CHECK(mean_max - mean_min < 50.0);             // [E] the energy mean is flat
    CHECK(std::abs(osc_sum / n - mean_sum / n) < 100.0);  // [E] and it is the osculating one's average
}

TEST_CASE("Mean elements from a flown state predict the orbit a day later", "[orbit][synthetic_station]") {
    // the true pole of the test frame is +z; the session's frame comes from `earth` - here a fixed orientation whose
    // true equator frame is GCRF (test_support's identity orientation; add one to this file if none exists)
    const test::IdentityOrientation earth;
    const double mu = constants::earth_gm_m3ps2, re = constants::wgs84_semi_major_axis_m, j2 = constants::earth_j2;
    const time::Epoch t0 = time::Epoch::from_tdb_seconds(0.0);
    auto s = leo_state();
    const auto mean = orbit::station_mean_elements_from_state(s, t0, mu, j2, re, earth);
    CHECK(mean.inclination_rad == Catch::Approx(26.5 * constants::deg_to_rad).margin(1e-4));
    for (double t = 0.0; t < 86400.0; t += 5.0) s = rk4_step(s, 5.0);
    const auto later = orbit::advance_mean_elements(mean, t0 + 86400.0, mu, j2, re);
    const auto predicted = orbit::synthetic_station_state(later, mu, earth);
    const double angle = std::acos(std::clamp(dot(math::normalized(predicted.position_m), math::normalized(s.position_m)), -1.0, 1.0));
    CHECK(angle < 0.01);  // [E] ~65 km along the track after 16 revolutions (osculating phase kept)
}
```

  If `test_support.hpp` has no identity `BodyOrientation`, add a minimal one in the test file:
  `struct IdentityOrientation : frames::BodyOrientation { ... }` returning the identity rotation and zero rate for
  every epoch (look at `frames::BodyOrientation`'s pure virtuals and implement exactly those).

- [ ] **Step 2: Run to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[synthetic_station]"`
Expected: compile error - `energy_mean_semi_major_axis_m` / `station_mean_elements_from_state` not declared.

- [ ] **Step 3: Implement** - header declarations below `synthetic_station_state`:

```cpp
// The semi-major axis of the mean motion from the J2 energy [D] (refuelling f2): the axisymmetric J2 field conserves
// E = v^2/2 - mu/r + U, U = mu J2 R^2 (3 sin^2(lat) - 1) / (2 r^3); the osculating a follows E - U and swings with the
// latitude (+-2 km at 200 km / 26.5 deg), the mean one follows E - <U> with <3 sin^2(lat) - 1> = 3/2 sin^2(i) - 1 and
// <1/r^3> = 1/a^3 (near-circular). Drag lowers it secularly. `pole_gcrf`: the J2 axis (the true pole) in GCRF.
double energy_mean_semi_major_axis_m(const math::StateVector& state_gcrf, const math::Vec3& pole_gcrf,
                                     double gm_m3ps2, double j2, double radius_m);

// Mean elements of a flown station at `epoch` (refuelling f2: the flown depot for the campaign calculator): the
// semi-major axis from the J2 energy, eccentricity, inclination, RAAN and argument of latitude osculating in the true
// equator frame of `epoch` [E: their short-period terms, ~0.01 deg, kept]; the argument of perigee 0.
StationMeanElements station_mean_elements_from_state(const math::StateVector& state_gcrf, time::Epoch epoch,
                                                     double gm_m3ps2, double j2, double radius_m,
                                                     const frames::BodyOrientation& earth);
```

  Implementation in `synthetic_station_orbit.cpp` (include `starsim/orbit/kepler.hpp` if not yet):

```cpp
double energy_mean_semi_major_axis_m(const math::StateVector& s, const math::Vec3& pole, double gm, double j2,
                                     double radius_m) {
    const double r = norm(s.position_m);
    const double sin_lat = dot(s.position_m, pole) / r;
    const double k = 0.5 * gm * j2 * radius_m * radius_m;
    const double energy =
        0.5 * dot(s.velocity_mps, s.velocity_mps) - gm / r + k * (3.0 * sin_lat * sin_lat - 1.0) / (r * r * r);
    const math::Vec3 h = cross(s.position_m, s.velocity_mps);
    const double cos_i = dot(h, pole) / norm(h);
    const double mean_p = 1.5 * (1.0 - cos_i * cos_i) - 1.0;  // <3 sin^2(lat) - 1>
    double a = -gm / (2.0 * energy);
    for (int i = 0; i < 3; ++i) a = -gm / (2.0 * (energy - k * mean_p / (a * a * a)));  // <U> at the mean a
    return a;
}

StationMeanElements station_mean_elements_from_state(const math::StateVector& s, time::Epoch epoch, double gm,
                                                     double j2, double radius_m, const frames::BodyOrientation& earth) {
    const math::Mat3 to_gcrf = true_equator_to_gcrf(earth, epoch);
    const math::Mat3 to_tod = transpose(to_gcrf);
    const math::Vec3 pole_gcrf = to_gcrf * math::Vec3{0.0, 0.0, 1.0};
    const auto e = elements_from_state({to_tod * s.position_m, to_tod * s.velocity_mps}, gm);
    StationMeanElements m;
    m.epoch = epoch;
    m.semi_major_axis_m = energy_mean_semi_major_axis_m(s, pole_gcrf, gm, j2, radius_m);
    m.eccentricity = e.eccentricity;
    m.inclination_rad = e.inclination_rad;
    m.raan_rad = e.raan_rad;
    m.argument_of_latitude_rad = e.argument_of_periapsis_rad + e.true_anomaly_rad;
    return m;
}
```

  (Check the exact field names of `orbit::elements_from_state`'s result in `kepler.hpp` - `depot_in_plane` in
  `refuelling_mission.cpp` uses `inclination_rad`, `raan_rad`, `argument_of_periapsis_rad`, `true_anomaly_rad`.)

- [ ] **Step 4: Run to verify they pass** - same command. Expected: PASS. If the flatness check fails, print the
  ranges with `STARSIM_VERBOSE` and fix the formula, never the tolerance without asking.

- [ ] **Step 5: Commit and push**

```bash
git add core/include/starsim/orbit/synthetic_station_orbit.hpp core/src/orbit/synthetic_station_orbit.cpp core/tests/test_synthetic_station_orbit.cpp
git commit -m "feat(orbit): energy-mean semi-major axis and mean elements from a flown state"
git push origin main
```

---

### Task 2: Session building blocks - keep_station, keep_only_station, place_vehicle, supervisor hook

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp`
- Modify: `core/src/sim/flight_session.cpp`
- Modify: `core/include/starsim/sim/ascent_mission.hpp`, `core/src/sim/ascent_mission.cpp`
- Modify: `core/src/sim/mission_start.cpp` (supervisor reset)
- Test: `core/tests/test_refuelling_campaign_flight.cpp` (new; add to `core/tests/CMakeLists.txt` next to
  `test_refuelling_campaign.cpp`)

**Interfaces:**
- Produces (in `flight_session.hpp`, namespace `starsim::sim`):

```cpp
// Refuelling f2: a driver above the mission sequences (sim::RefuellingCampaignFlight). FlightSession::advance_to calls
// update() before every tick or coast segment (it may launch, place or remove bodies there, like a command) and ends
// every coast at wake_epoch().
class SessionSupervisor {
public:
    virtual ~SessionSupervisor() = default;
    virtual void update(FlightSession& session) = 0;
    virtual std::optional<time::Epoch> wake_epoch() const = 0;
};
```

  FlightSession members:

```cpp
void launch_from_pad(const std::string& vehicle_id, const LaunchSite& site, double heading_rad, time::Epoch epoch,
                     bool keep_station = false);
void launch_from_pad(const config::VehicleConfig& config, const LaunchSite& site, double heading_rad,
                     time::Epoch epoch, bool keep_station = false);
void place_in_orbit(const config::VehicleConfig& config, const math::StateVector& state_gcrf, time::Epoch epoch,
                    bool keep_station = false);
// Refuelling f2: an existing vehicle (e.g. Vehicle::copy_upper of another session's body, tank load included) as the
// only, controlled body - as place_body.
void place_vehicle(std::unique_ptr<vehicle::Vehicle> vehicle, const vehicle::RigidBodyState& state, time::Epoch epoch);
// Refuelling f2: every body but the station is removed (a departed tanker, a caught booster; a docked pair is released
// first); the station alone is the controlled index with the autopilot off (as add_station alone), the mission
// sequence, the rendezvous target and a running transfer cleared. Throws std::logic_error without a station.
void keep_only_station();
void set_supervisor(std::shared_ptr<SessionSupervisor> supervisor) { supervisor_ = std::move(supervisor); }
const SessionSupervisor* supervisor() const { return supervisor_.get(); }
```

  `start_ascent_mission(FlightSession&, const config::AscentMission&, const std::filesystem::path&, time::Epoch, const config::VehicleConfig* vehicle = nullptr, bool keep_station = false);`

  `keep_station` semantics (document at `launch_from_pad`): every body but the station is removed, the new vehicle
  becomes body 0 (controlled), the station body 1 with its state, tanks, ideal attitude and pending impulses; the
  mission sequence and the rendezvous target are cleared. Throws std::logic_error without a station or when `epoch`
  is not the session epoch (within one tick).

- [ ] **Step 1: Write the failing tests** (`core/tests/test_refuelling_campaign_flight.cpp`):

```cpp
// Refuelling f2: the flown campaign - session building blocks, the station reboost, the driver.

#include <cmath>
#include <memory>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "refuelling_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/mission_start.hpp"
#include "test_support.hpp"

using namespace starsim;
using sim::IssMissionLeg;

namespace {

sim::LaunchSite starbase() {
    return sim::launch_site_from_config(config::load_launch_site(refuelling_test::config_dir, "starbase"));
}

struct CountingSupervisor : sim::SessionSupervisor {
    std::optional<time::Epoch> wake;
    std::vector<time::Epoch> updates;
    void update(sim::FlightSession& s) override { updates.push_back(s.epoch()); }
    std::optional<time::Epoch> wake_epoch() const override { return wake; }
};

}  // namespace

TEST_CASE("keep_station: a launch keeps the station and removes every other body", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.add_station(refuelling_test::depot_config(300.0e3), f.depot, f.t0);
    f.session.advance_to(f.t0 + 600.0);
    const auto before = f.session.body(0).state();
    const double fill = f.session.body(0).vehicle().main_propellant_kg();
    f.session.launch_from_pad("starship_tanker_stack", starbase(), 0.0, f.session.epoch(), true);
    REQUIRE(f.session.body_count() == 2u);
    CHECK(f.session.controlled_index() == 0u);
    REQUIRE(f.session.station_index() == std::optional<size_t>{1});
    CHECK(norm(f.session.body(1).state().position_gcrf_m - before.position_gcrf_m) < 1e-6);
    CHECK(f.session.body(1).vehicle().main_propellant_kg() == Catch::Approx(fill));
    CHECK(f.session.body(1).ideal_attitude());
    // a second launch with the station kept replaces the stack, not the station
    f.session.launch_from_pad("starship_tanker_stack", starbase(), 0.0, f.session.epoch(), true);
    CHECK(f.session.body_count() == 2u);
}

TEST_CASE("keep_station refuses without a station or off the session epoch", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    CHECK_THROWS_AS(f.session.launch_from_pad("starship_tanker_stack", starbase(), 0.0, f.t0, true), std::logic_error);
    f.session.add_station(refuelling_test::depot_config(300.0e3), f.depot, f.t0);
    CHECK_THROWS_AS(f.session.launch_from_pad("starship_tanker_stack", starbase(), 0.0, f.t0 + 60.0, true),
                    std::logic_error);
    CHECK(f.session.body_count() == 1u);  // nothing dropped
}

TEST_CASE("keep_only_station removes the chaser and the sequence", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, -30.0});  // body 0 tanker, body 1 depot
    sim::IssMissionSequence q;
    q.station_port = "transfer_dock";
    f.session.set_iss_mission(q);
    f.session.keep_only_station();
    REQUIRE(f.session.body_count() == 1u);
    CHECK(f.session.station_index() == std::optional<size_t>{0});
    CHECK(f.session.controlled_index() == 0u);
    CHECK_FALSE(f.session.iss_mission().has_value());
    sim::FlightSession empty(f.ephemeris, f.earth, refuelling_test::config_dir);
    CHECK_THROWS_AS(empty.keep_only_station(), std::logic_error);
}

TEST_CASE("place_vehicle places a copy with its tank load", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::tanker_config(42.0e3), f.depot, f.t0);
    const auto& body = f.session.body(0);
    sim::FlightSession side(f.ephemeris, f.earth, refuelling_test::config_dir);
    side.place_vehicle(body.vehicle().copy_upper(0), body.state(), f.t0);
    REQUIRE(side.body_count() == 1u);
    CHECK(side.body(0).vehicle().main_propellant_kg() == Catch::Approx(42.0e3).epsilon(1e-6));
    CHECK(side.epoch() == f.t0);
}

TEST_CASE("Supervisor: a time-warp call stops at its wake epoch", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.add_station(refuelling_test::depot_config(300.0e3), f.depot, f.t0);
    auto sup = std::make_shared<CountingSupervisor>();
    sup->wake = f.t0 + 1000.3;
    f.session.set_supervisor(sup);
    f.session.advance_to(f.t0 + 5.0 * 86400.0);
    const auto hit = std::find_if(sup->updates.begin(), sup->updates.end(),
                                  [&](time::Epoch e) { return e >= *sup->wake - sim::FlightSession::tick_s; });
    REQUIRE(hit != sup->updates.end());
    CHECK(std::abs(*hit - *sup->wake) <= sim::FlightSession::tick_s);
    CHECK(f.session.epoch() - f.t0 == Catch::Approx(5.0 * 86400.0).margin(1.0));
}

TEST_CASE("Supervisor: removed by clear_bodies and by any mission start", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.add_station(refuelling_test::depot_config(300.0e3), f.depot, f.t0);
    f.session.set_supervisor(std::make_shared<CountingSupervisor>());
    f.session.clear_bodies();
    CHECK(f.session.supervisor() == nullptr);
    f.session.set_supervisor(std::make_shared<CountingSupervisor>());
    sim::start_mission(f.session, refuelling_test::config_dir, "tanker_flight_orbit", f.t0, f.clock);
    CHECK(f.session.supervisor() == nullptr);
}
```

  (Use the tanker stack's real vehicle id: check `config/missions/tanker_ascent.toml`'s vehicle and use that id in
  place of `"starship_tanker_stack"`. The site id: `tanker_ascent.toml`'s site.)

- [ ] **Step 2: Run to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[campaign_flown]"`
Expected: compile errors (no `SessionSupervisor`, `keep_only_station`, `place_vehicle`, 5-argument `launch_from_pad`).

- [ ] **Step 3: Implement**

  a) `flight_session.hpp`: the class above before `class FlightSession`, the members above (public, next to
  `launch_from_pad` / `place_in_orbit` / `clear_bodies`), private `std::shared_ptr<SessionSupervisor> supervisor_;`
  and a private helper `void clear_for_new_body(bool keep_station, time::Epoch epoch);`. Update the comments of
  `add_station` ("they clear all bodies, the station included - unless keep_station") and `set_iss_mission`.

  b) `flight_session.cpp`: one helper used by `launch_from_pad`, `place_in_orbit(config, ...)`, `place_vehicle`:

```cpp
// Before a new body: every body removed - or (keep_station, refuelling f2) every body but the station, which moves to
// index 0 (the caller inserts the new body in front of it).
void FlightSession::clear_for_new_body(bool keep_station, time::Epoch epoch) {
    if (!keep_station) {
        bodies_.clear();
        clear_station();
        clear_rendezvous_target();
        reset_rendezvous();
        reset_leg_landing();
        return;
    }
    if (!station_index_) throw std::logic_error("keep_station: the session has no station");
    if (std::abs(epoch - epoch_) > tick_s) throw std::logic_error("keep_station: the epoch is not the session epoch");
    clear_rendezvous_target();  // releases a docked pair, ends proximity and docking
    reset_rendezvous();
    set_iss_mission({});
    for (size_t i = bodies_.size(); i-- > 0;) {
        if (i != *station_index_) erase_body(i);  // shifts station_index_ down to 0
    }
}
```

  In `launch_from_pad(config, ...)` replace the four clear lines with `clear_for_new_body(keep_station, epoch);` and the
  `push_back` by `bodies_.insert(bodies_.begin(), {...})` when `keep_station` (then `station_index_ = 1`), else as now.
  Same in `place_in_orbit(config, state, epoch, keep_station)` (its `reset_leg_landing()` stays for both). The string
  overload of `launch_from_pad` forwards `keep_station`. `place_vehicle` is `place_body` with the given vehicle
  (`clear_for_new_body(false, epoch)` + the body built from `std::move(vehicle)`; ground height as `place_body`).
  Check `transfer_` (a running transfer) is reset by `clear_rendezvous_target()`; if not, call
  `reset_docked_update()` and end the transfer in `clear_for_new_body` and in `keep_only_station`.

```cpp
void FlightSession::keep_only_station() {
    if (!station_index_) throw std::logic_error("keep_only_station: the session has no station");
    clear_rendezvous_target();
    reset_rendezvous();
    set_iss_mission({});
    for (size_t i = bodies_.size(); i-- > 0;) {
        if (i != *station_index_) erase_body(i);
    }
    controlled_ = 0;
    reset_mission_state();  // autopilot off: nothing commands the station
}
```

  `clear_bodies()`: add `supervisor_.reset();`.

  `advance_to`: after `apply_station_impulses();`

```cpp
        if (supervisor_) {
            const auto keep = supervisor_;  // it may remove itself (a mission start)
            keep->update(*this);
            if (bodies_.empty()) break;
            if (target - epoch_ < tick_s) break;
        }
```

  and bound the coast:

```cpp
        double coast = std::min(coast_allowance_s(), target - epoch_);
        if (supervisor_) {
            if (const auto wake = supervisor_->wake_epoch(); wake && *wake > epoch_) coast = std::min(coast, *wake - epoch_);
        }
        if (coast > 1.0) { ... as now ... }
```

  (Keep the existing `if (const double coast = ...; coast > 1.0)` body unchanged apart from the variable.)

  c) `ascent_mission.{hpp,cpp}`: the new `keep_station` parameter passed to `session.launch_from_pad(...)`; update
  the header comment ("keep_station: beside the session's station, refuelling f2").

  d) `mission_start.cpp`: first line of `start_mission`: `session.set_supervisor(nullptr);  // f2: a new mission ends a campaign`.

- [ ] **Step 4: Run to verify they pass** - `-TestFilter "[campaign_flown]"`, then the touched subsystems:
  `-TestFilter "[refuelling]"` (includes `[slow]` - background, output to a file, check every 10 min) and
  `-TestFilter "[ascent]"`. Expected: all PASS (the existing behaviour without `keep_station` is unchanged).

- [ ] **Step 5: Commit and push**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/include/starsim/sim/ascent_mission.hpp core/src/sim/ascent_mission.cpp core/src/sim/mission_start.cpp core/tests/test_refuelling_campaign_flight.cpp core/tests/CMakeLists.txt
git commit -m "feat(sim): session supervisor hook, keep_station launches, keep_only_station, place_vehicle"
git push origin main
```

---

### Task 3: Station reboost (the depot's station keeping)

**Files:**
- Create: `core/src/sim/flight_session_reboost.cpp` (add to `core/CMakeLists.txt` after `flight_session_transfer.cpp`)
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (tick hook, reset)
- Modify: `core/tests/refuelling_test_support.hpp` (`depot_state` height parameter)
- Test: `core/tests/test_refuelling_campaign_flight.cpp`

**Interfaces:**
- Consumes: Task 1 `orbit::energy_mean_semi_major_axis_m`; existing `gnc::next_raise_node`,
  `gnc::raise_periapsis_at_apoapsis`, `start_burn`, `burn_configurations`, `small_burn_transient_share`,
  `make_station`, `mission::raise_tolerance_m`.
- Produces:

```cpp
struct ReboostRecord {  // refuelling f2: one station reboost
    time::Epoch start, end;
    double propellant_kg = 0.0;  // main tanks (Raptor + cold gas + settling)
    int burns = 0;
    bool completed = false;
    std::string reason;          // German when not completed
};
// Refuelling f2: the station alone (the depot) raises its orbit to `height_m` circular on `engine_groups` with the
// raise's rules (gnc::next_raise_node, then the periapsis at the new apoapsis; transients over
// small_burn_transient_share: the cold gas) and the settling push before each ignition (f3). The station is the
// controlled body during the burns and the station again after the last one - or when a burn is refused or aborted.
// False with a German reboost_status() when refused: no station, other bodies present, no engine groups, already
// within raise_tolerance_m of the height or above, a reboost running.
bool start_station_reboost(double height_m, std::vector<std::string> engine_groups);
bool station_reboost_active() const { return reboost_.has_value(); }
const std::string& reboost_status() const { return reboost_status_; }
const std::optional<ReboostRecord>& last_reboost() const { return last_reboost_; }
// The station's mean height above the equatorial radius from its J2 energy (f2); throws std::logic_error without one.
double station_mean_height_m() const;
```

  Private: `struct Reboost { double height_m; std::vector<std::string> groups; bool periapsis_pending = false; bool burning = false; size_t station_body; double start_main_kg; ReboostRecord record; std::vector<StationImpulse> impulses; }; std::optional<Reboost> reboost_; std::string reboost_status_; std::optional<ReboostRecord> last_reboost_; void run_reboost(); void end_reboost(bool completed, std::string reason);`

- [ ] **Step 1: Write the failing tests.** First give `refuelling_test::depot_state` a height parameter
  (`double height_m = 200.0e3` as the last parameter, used for `semi_major_axis_m`; existing callers unchanged). Then:

```cpp
namespace {

double mean_height_km(const sim::FlightSession& s) { return s.station_mean_height_m() / 1000.0; }

// Advances in 60 s calls until the reboost ends or `limit_s`.
void fly_reboost(sim::FlightSession& s, double limit_s = 4.0 * 3600.0) {
    const auto t0 = s.epoch();
    while (s.station_reboost_active() && s.epoch() - t0 < limit_s) s.advance_to(s.epoch() + 60.0);
}

const std::vector<std::string> belly = {"sea_level_3"};

}  // namespace

TEST_CASE("Reboost: a low depot raises to 200 km in two burns and is the station again", "[campaign_flown][refuelling][reboost]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.add_station(refuelling_test::depot_config(400.0e3), refuelling_test::depot_state(f.earth, f.t0, 0.0, 193.0e3), f.t0);
    const double fill0 = f.session.body(0).vehicle().main_propellant_kg();
    REQUIRE(f.session.start_station_reboost(200.0e3, belly));
    CHECK_FALSE(f.session.station_index().has_value());  // controlled for the burns
    fly_reboost(f.session);
    REQUIRE(f.session.last_reboost());
    const auto& r = *f.session.last_reboost();
    INFO(r.reason);
    CHECK(r.completed);
    CHECK(r.burns == 2);
    REQUIRE(f.session.station_index() == std::optional<size_t>{0});
    CHECK(f.session.body(0).ideal_attitude());
    CHECK(mean_height_km(f.session) == Catch::Approx(200.0).margin(1.0));  // [E]
    // propellant against the ideal Hohmann pair (vis-viva) at the Raptor's exhaust velocity: within 1.5x [E]
    const double mu = constants::earth_gm_m3ps2, re = constants::wgs84_semi_major_axis_m;
    const double r1 = re + 193.0e3, r2 = re + 200.0e3, at = 0.5 * (r1 + r2);
    const double dv = std::sqrt(mu * (2 / r1 - 1 / at)) - std::sqrt(mu / r1) + std::sqrt(mu / r2) - std::sqrt(mu * (2 / r2 - 1 / at));
    const double ve = sim::phasing_exhaust_velocity_mps(f.session.body(0).vehicle());
    const double ideal_kg = f.session.body(0).vehicle().mass_properties().mass_kg * (std::exp(dv / ve) - 1.0);
    CHECK(r.propellant_kg == Catch::Approx(fill0 - f.session.body(0).vehicle().main_propellant_kg()).margin(50.0));
    CHECK(r.propellant_kg > ideal_kg);
    CHECK(r.propellant_kg < 1.5 * ideal_kg + 200.0);  // + the settling gas [E]
}

TEST_CASE("Reboost refusals", "[campaign_flown][refuelling][reboost]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    CHECK_FALSE(f.session.start_station_reboost(200.0e3, belly));  // no station
    CHECK_FALSE(f.session.reboost_status().empty());
    f.session.add_station(refuelling_test::depot_config(400.0e3), refuelling_test::depot_state(f.earth, f.t0, 0.0, 201.0e3), f.t0);
    CHECK_FALSE(f.session.start_station_reboost(200.0e3, belly));  // already above
    CHECK_FALSE(f.session.start_station_reboost(210.0e3, {}));     // no engine groups
    refuelling_test::TankerAtPort g({0.0, 0.0, -30.0});
    CHECK_FALSE(g.session.start_station_reboost(260.0e3, belly));  // a chaser is present
}

TEST_CASE("Reboost: an aborted burn leaves the depot a station", "[campaign_flown][refuelling][reboost]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.add_station(refuelling_test::depot_config(5.0), refuelling_test::depot_state(f.earth, f.t0, 0.0, 193.0e3), f.t0);
    f.session.start_station_reboost(200.0e3, belly);  // may refuse at once or abort in flight
    fly_reboost(f.session);
    REQUIRE(f.session.last_reboost());
    CHECK_FALSE(f.session.last_reboost()->completed);
    CHECK_FALSE(f.session.last_reboost()->reason.empty());
    REQUIRE(f.session.station_index().has_value());
    CHECK(f.session.body(*f.session.station_index()).ideal_attitude());
}
```

  (`start_station_reboost` sets `last_reboost_` with `completed = false` and the reason also for a refusal that
  happens after the station was released - e.g. a first burn that cannot start; a refusal before the release only sets
  `reboost_status_`. For the third test both paths must end with a station: if the refusal comes first,
  `last_reboost()` is empty - then the test's `REQUIRE(last_reboost())` would fail; make the refusal of a burn that
  cannot start happen after the release, so it is recorded.)

- [ ] **Step 2: Run to verify they fail** - `-TestFilter "[reboost]"`. Expected: compile errors.

- [ ] **Step 3: Implement** `flight_session_reboost.cpp`:

```cpp
// Refuelling f2: the station's reboost - the depot keeps its altitude between the tanker flights (spec f2 "Depot
// station keeping, flown"): released from its ideal attitude, two apsis burns with the raise's rules, a station again.

#include <cmath>

#include "starsim/gnc/raise_nodes.hpp"  // the header that declares gnc::next_raise_node (as flight_session_iss_mission.cpp)
#include "starsim/math/constants.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

double FlightSession::station_mean_height_m() const {
    if (!station_index_) throw std::logic_error("station_mean_height_m: no station");
    const auto& s = bodies_[*station_index_].body->state();
    const math::Vec3 pole = orbit::true_equator_to_gcrf(earth_, epoch_) * math::Vec3{0.0, 0.0, 1.0};
    return orbit::energy_mean_semi_major_axis_m({s.position_gcrf_m, s.velocity_gcrf_mps}, pole, gm_earth_,
                                                constants::earth_j2, constants::wgs84_semi_major_axis_m) -
           constants::wgs84_semi_major_axis_m;
}

bool FlightSession::start_station_reboost(double height_m, std::vector<std::string> groups) {
    const auto refuse = [&](const char* why) { reboost_status_ = why; return false; };
    if (reboost_) return refuse("Reboost läuft bereits");
    if (!station_index_) return refuse("Reboost: keine Station");
    if (bodies_.size() != 1) return refuse("Reboost nur ohne andere Fahrzeuge");
    if (groups.empty()) return refuse("Reboost: keine Triebwerksgruppen");
    if (station_mean_height_m() > height_m - mission::raise_tolerance_m) return refuse("Reboost: Station hoch genug");
    const size_t index = *station_index_;
    auto& body = *bodies_[index].body;
    Reboost r;
    r.height_m = height_m;
    r.groups = std::move(groups);
    r.station_body = index;
    r.start_main_kg = body.vehicle().main_propellant_kg();
    r.record.start = epoch_;
    r.impulses = std::move(station_impulses_);
    // released: the depot flies (its ideal attitude ends, the burn executor turns it)
    station_index_.reset();
    station_impulses_.clear();
    body.set_ideal_lvlh_attitude(std::nullopt);
    controlled_ = index;
    reset_mission_state();
    reboost_ = std::move(r);
    reboost_status_ = "Reboost";
    run_reboost();  // the first burn now (or the end)
    return true;
}

void FlightSession::run_reboost() {
    auto& r = *reboost_;
    if (r.burning) {
        if (autopilot_.state == AutopilotState::Burn) return;  // flying
        r.burning = false;
        if (!burn_abort_reason_.empty()) return end_reboost(false, "Reboost abgebrochen: " + burn_abort_reason_);
    }
    const auto& s = bodies_[controlled_].body->state();
    const math::StateVector state{s.position_gcrf_m, s.velocity_gcrf_mps};
    const double radius_m = constants::wgs84_semi_major_axis_m + r.height_m;
    std::optional<gnc::BurnNode> node;
    if (r.record.burns == 0) {
        node = gnc::next_raise_node(state, epoch_, gm_earth_, radius_m, radius_m, mission::raise_tolerance_m);
        r.periapsis_pending = node.has_value();
    } else if (r.periapsis_pending) {
        r.periapsis_pending = false;
        node = gnc::raise_periapsis_at_apoapsis(state, epoch_, gm_earth_, radius_m);
    }
    if (!node) return end_reboost(true, {});
    if (!start_burn(*node, burn_configurations(&r.groups), true, small_burn_transient_share)) {
        return end_reboost(false, "Reboost: Brennvorgang nicht möglich");
    }
    r.burning = true;
    ++r.record.burns;
}

void FlightSession::end_reboost(bool completed, std::string reason) {
    auto r = std::move(*reboost_);
    reboost_.reset();
    disengage_autopilot();
    r.record.end = epoch_;
    r.record.completed = completed;
    r.record.reason = std::move(reason);
    r.record.propellant_kg = r.start_main_kg - bodies_[r.station_body].body->vehicle().main_propellant_kg();
    make_station(r.station_body);  // the ideal attitude at once
    station_impulses_ = std::move(r.impulses);
    reboost_status_ = completed ? "Reboost abgeschlossen" : r.record.reason;
    last_reboost_ = std::move(r.record);
}

}  // namespace starsim::sim
```

  Notes for the implementer:
  - Use the exact include / function names the Raise leg uses (`flight_session_iss_mission.cpp`: `gnc::next_raise_node`,
    `gnc::raise_periapsis_at_apoapsis`, `mission::raise_tolerance_m`); `burn_abort_reason_` is cleared at a burn start
    - check that, else snapshot it at the start.
  - The first burn's `periapsis_pending` must mirror `iss_start_raise`: only pending when the first node raised the
    apoapsis (copy its condition `to.apoapsis_radius_m > a(1+e) + tolerance`).
  - Tick hook in `advance_to` next to `if (iss_mission_) run_iss_mission();`: `if (reboost_) run_reboost();`.
  - `clear_for_new_body`, `clear_bodies`, `keep_only_station`: `reboost_.reset();` (a new flight ends a reboost; a
    released station body would otherwise stay free) - for `keep_only_station` refuse instead (`std::logic_error`
    "a reboost is running") since there is no station.
  - `reset_mission_state()` must not touch `reboost_` (it is called inside `start_station_reboost`).

- [ ] **Step 4: Run to verify they pass** - `-TestFilter "[reboost]"`, then `-TestFilter "[campaign_flown]"`. Measure
  the run time of the first case; tag it `[slow]` if over ~40 s. If the Raptor is refused for a 1.5 m/s burn on 636 t
  (transients > 60 %), the burn flies on cold gas - record which one flew (`BurnConfiguration::rcs` of the started
  burn, add it to `ReboostRecord` as `bool cold_gas`) and report the numbers in the task's summary.

- [ ] **Step 5: Commit and push**

```bash
git add core/src/sim/flight_session_reboost.cpp core/CMakeLists.txt core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/tests/refuelling_test_support.hpp core/tests/test_refuelling_campaign_flight.cpp
git commit -m "feat(sim): station reboost - the depot raises its orbit and is the station again"
git push origin main
```

---

### Task 4: Calculator live start

**Files:**
- Modify: `core/include/starsim/mission/refuelling_campaign.hpp`, `core/src/mission/refuelling_campaign.cpp`
- Modify: `core/include/starsim/sim/refuelling_campaign_setup.hpp`, `core/src/sim/refuelling_campaign_setup.cpp`
- Modify: `core/include/starsim/sim/refuelling_mission.hpp`, `core/src/sim/refuelling_mission.cpp` (`station_orbit`)
- Test: `core/tests/test_refuelling_campaign.cpp`

**Interfaces:**
- Consumes: Task 1 `orbit::station_mean_elements_from_state`.
- Produces:

```cpp
// mission/refuelling_campaign.hpp
struct CampaignFlight {
    ...                                        // as now
    std::optional<LaunchWindow> window;        // f2: the planned window (the flown campaign launches it)
};
struct CampaignInputs {
    ...                                        // as now
    // f2 live re-plan: the last flight before t = 0 (negative times) and the earliest launch
    double previous_launch_s = -std::numeric_limits<double>::infinity();
    double previous_arrival_s = -std::numeric_limits<double>::infinity();
    double earliest_launch_s = 0.0;
};

// sim/refuelling_mission.hpp
// A station orbit from mean elements, advanced by the J2 rates (true equator frame); `earth` must outlive it.
mission::StationStateAt station_orbit(const orbit::StationMeanElements& mean, const frames::BodyOrientation& earth);

// sim/refuelling_campaign_setup.hpp
struct CampaignHlsDemand {  // the smallest HLS mass closing the lunar mission (r6 plan decision 2)
    double required_mass_kg = 0.0, dry_mass_kg = 0.0, capacity_kg = 0.0, headers_kg = 0.0;
};
// Refuelling f2: a re-plan from the flown campaign - t = 0 at `epoch`, the depot from its flown state
// (station_mean_elements_from_state: its mean height for the drag and the shifted phasing orbits), its fill, the last
// flight (launch interval and port spacing count from it), no launch earlier than launch_lead_s, and the HLS demand of
// the first plan (empty: planned, as without a live start).
struct CampaignLive {
    time::Epoch epoch;
    math::StateVector depot_gcrf;
    double depot_propellant_kg = 0.0;
    std::optional<time::Epoch> previous_launch, previous_arrival;
    double launch_lead_s = 3600.0;  // [E] pad readiness after a decision
    std::optional<CampaignHlsDemand> demand;
};
struct CampaignReport {
    ...                                        // as now
    CampaignHlsDemand hls_demand;
    mission::LaunchWindowParameters tanker_window, hls_window;  // as the windows were planned (f2: flown with them)
};
CampaignReport plan_refuelling_campaign(const CampaignSources& sources, const CampaignLevers& levers,
                                        const CampaignLive* live = nullptr);
```

- [ ] **Step 1: Write the failing tests** (append to `core/tests/test_refuelling_campaign.cpp`; it already has fake
  models for the pure loop - reuse its helper that builds daily windows, or add this one):

```cpp
TEST_CASE("Campaign: the last flight before the start constrains the first launch and arrival", "[refuelling][campaign]") {
    mission::CampaignInputs in;
    in.depot_capacity_kg = 1000.0e3;
    in.depot_initial_kg = 0.0;
    in.demand_kg = 50.0e3;
    in.previous_launch_s = -0.5 * 86400.0;   // a launch 12 h ago: the next one from +12 h
    in.previous_arrival_s = 2.0 * 3600.0;     // arriving in 2 h: no arrival before 8 h (6 h port spacing)
    in.earliest_launch_s = 3600.0;
    mission::CampaignModels m;
    m.boil_off_kgps = [](double, double) { return 0.0; };
    m.drag_makeup_kgps = [](double) { return 0.0; };
    std::vector<double> asked;
    m.next_tanker = [&](double earliest) {  // a window every 3 h, arriving 1 h after launch
        asked.push_back(earliest);
        const double launch = std::ceil(earliest / 10800.0) * 10800.0;
        return std::optional<mission::CampaignFlight>{{launch, launch + 3600.0, 60.0e3}};
    };
    m.next_hls = [](double earliest) {
        const double launch = std::ceil(earliest / 10800.0) * 10800.0 + 86400.0;
        return std::optional<mission::CampaignFlight>{{launch, launch + 3600.0, 0.0}};
    };
    const auto r = mission::refuelling_campaign(in, m);
    REQUIRE(r.closes);
    REQUIRE_FALSE(r.tankers.empty());
    CHECK(r.tankers.front().flight.launch_s >= 0.5 * 86400.0 - 1.0);
    CHECK(r.tankers.front().flight.arrival_s >= 8.0 * 3600.0 - 1.0);
    CHECK(asked.front() >= 3600.0);
}

TEST_CASE("Campaign live start: the flown depot's orbit, fill and windows", "[refuelling][campaign][slow]") {
    test::spice_session();
    const auto& src = campaign_sources();  // the file's existing sources fixture (ephemeris, earth, moon, time)
    const auto first = sim::plan_refuelling_campaign(src, {});
    REQUIRE(first.result.closes);
    // a depot 10 days in, on tanker_flight's [depot] orbit advanced by J2, with 300 t
    const auto tanker = config::load_refuelling_flight_mission(refuelling_test::config_dir, "tanker_flight");
    const time::Epoch epoch = first.start + 10.0 * 86400.0;
    sim::CampaignLive live;
    live.epoch = epoch;
    live.depot_gcrf = sim::depot_orbit(tanker.depot, first.start, src.earth)(epoch);
    live.depot_propellant_kg = 300.0e3;
    live.previous_launch = epoch - 6.0 * 3600.0;
    live.demand = first.hls_demand;
    const auto r = sim::plan_refuelling_campaign(src, {}, &live);
    INFO(r.result.reason);
    REQUIRE(r.result.closes);
    CHECK(r.start == epoch);
    CHECK(r.demand_kg == Catch::Approx(first.demand_kg));  // the cached demand
    CHECK(r.result.tanker_flights < first.result.tanker_flights);  // 300 t instead of 55.5 t
    REQUIRE_FALSE(r.result.tankers.empty());
    const auto& f = r.result.tankers.front().flight;
    REQUIRE(f.window);
    CHECK(f.window->liftoff - r.start == Catch::Approx(f.launch_s).margin(1e-3));
    CHECK(f.launch_s >= 18.0 * 3600.0 - 1.0);  // previous launch 6 h ago + 24 h turnaround
    CHECK(r.depot_height_m == Catch::Approx(200.0e3).margin(3.0e3));  // energy mean of the synthetic state [E]
}

TEST_CASE("Campaign without a live start is unchanged", "[refuelling][campaign]") {
    test::spice_session();
    const auto r = sim::plan_refuelling_campaign(campaign_sources(), {});
    REQUIRE(r.result.closes);
    CHECK(r.result.tanker_flights == 13);
    CHECK(r.result.duration_s / 86400.0 == Catch::Approx(49.7).margin(0.05));
    for (const auto& t : r.result.tankers) CHECK(t.flight.window.has_value());
}
```

  (Name the sources fixture as the file does - read the top of `test_refuelling_campaign.cpp`. If the existing fast
  case already asserts 13 / 49.7 d, add only the `window` loop to it instead of the third case.)

- [ ] **Step 2: Run to verify they fail** - `-TestFilter "[campaign]"`. Expected: compile errors.

- [ ] **Step 3: Implement**
  - `refuelling_campaign.hpp`: include `starsim/mission/iss_launch_window.hpp`; the new fields. In
    `refuelling_campaign.cpp` replace the two initialisations:

```cpp
    // f2: a live re-plan carries its last flight; never earlier than earliest_launch_s
    double last_launch_s = std::max(in.previous_launch_s, in.earliest_launch_s - in.launch_interval_s);
    double last_arrival_s = std::max(in.previous_arrival_s, -in.port_spacing_s);
```

  - `refuelling_mission.cpp`: `station_orbit(mean, earth)` = the lambda of `depot_orbit` (which now builds the mean
    elements and calls `station_orbit`).
  - `refuelling_campaign_setup.cpp`:
    - `hls_demand` returns a struct that contains `CampaignHlsDemand` plus the refusal; skip it when
      `live && live->demand` (use the cached values).
    - `const time::Epoch config_start = src.time.from_utc(tanker_mission.earliest_utc);` - the reference windows
      (`reference_depot_at`, both `plan_launch_window(... start ...)` calls of the baselines) keep `config_start`;
      `start = live ? live->epoch : config_start`.
    - With `live`: `const auto mean = orbit::station_mean_elements_from_state(live->depot_gcrf, live->epoch, gm,
      constants::earth_j2, equatorial_radius_m, src.earth);` `depot_cfg.height_m = mean.semi_major_axis_m -
      equatorial_radius_m` (a `levers.depot_height_m` together with `live` is refused: "Kampagne: im geflogenen Lauf
      gilt die Höhe des Depots"); `depot_at = station_orbit(mean, src.earth)` (make `depot_at` a
      `mission::StationStateAt` built by either branch). `inputs.depot_initial_kg = live->depot_propellant_kg`;
      `inputs.previous_launch_s = live->previous_launch ? *live->previous_launch - start : -inf` (same for arrival);
      `inputs.earliest_launch_s = live->launch_lead_s`.
    - `models.next_tanker` / `next_hls`: set `flight.window = w;`.
    - `report.hls_demand`, `report.tanker_window = tanker_window; report.hls_window = hls_window;`.
  - Header comments updated (`plan_refuelling_campaign`: "live: f2 re-plan").

- [ ] **Step 4: Run to verify they pass** - `-TestFilter "[campaign]"` (includes `[slow]`), and the CLI still prints
  13 / 49.7 d: `build/core/refuelling_report.exe > <scratchpad>/report.txt` and read its last line.

- [ ] **Step 5: Commit and push**

```bash
git add core/include/starsim/mission/refuelling_campaign.hpp core/src/mission/refuelling_campaign.cpp core/include/starsim/sim/refuelling_campaign_setup.hpp core/src/sim/refuelling_campaign_setup.cpp core/include/starsim/sim/refuelling_mission.hpp core/src/sim/refuelling_mission.cpp core/tests/test_refuelling_campaign.cpp
git commit -m "feat(sim): campaign calculator re-plans from a live depot state"
git push origin main
```

---

### Task 5: A campaign flight launched beside the kept depot

**Files:**
- Modify: `core/include/starsim/sim/refuelling_mission.hpp`, `core/src/sim/refuelling_mission.cpp`
- Test: `core/tests/test_refuelling_campaign_flight.cpp`

**Interfaces:**
- Consumes: Task 2 `start_ascent_mission(..., keep_station)`; Task 4 `CampaignReport::tanker_window`.
- Produces:

```cpp
// The chain of a pad-start refuelling flight (r5) in `window` with `parameters` (its phasing limits): Ascent -> Raise
// (the window's phasing orbit) -> Rendezvous -> Proximity -> Docking -> Transfer (tanker: down to its reserve; HLS:
// filled from the station) -> Departure. No ship return, no continuation (the callers add them).
IssMissionSequence pad_flight_sequence(const config::RefuellingFlightMission& mission,
                                       const mission::LaunchWindowParameters& parameters,
                                       const mission::LaunchWindow& window);
// Refuelling f2: a flight of the flown campaign - `mission`'s ascent launched at window.liftoff beside the session's
// station (keep_station; the session must be at that epoch), the chain of pad_flight_sequence. MissionStart.refusal
// (German) when the window is not feasible or the session is not at its liftoff.
MissionStart launch_campaign_flight(FlightSession& session, const std::filesystem::path& config_dir,
                                    const config::RefuellingFlightMission& mission,
                                    const mission::LaunchWindow& window,
                                    const mission::LaunchWindowParameters& parameters);
```

- [ ] **Step 1: Write the failing test:**

```cpp
TEST_CASE("A campaign tanker launches beside the kept depot and flies its chain to the raise",
          "[campaign_flown][refuelling][ascent]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto dir = refuelling_test::config_dir;
    const auto m = config::load_refuelling_flight_mission(dir, "tanker_flight");
    const auto ascent = config::load_ascent_mission(dir, m.ascent_id);
    const auto site = config::load_launch_site(dir, ascent.site_id);
    f.session.add_station(refuelling_test::depot_config(300.0e3), f.depot, f.t0);
    const vehicle::Vehicle chaser(config::top_stage_config(config::load_vehicle(dir, ascent.vehicle_id)), dir);
    const auto p = sim::refuelling_window_parameters(m, ascent, sim::phasing_exhaust_velocity_mps(chaser),
                                                     sim::phasing_settling_gas_kg(chaser, f.session.ignition_settling()));
    const auto mean = orbit::station_mean_elements_from_state(f.depot, f.t0, constants::earth_gm_m3ps2,
                                                              constants::earth_j2, constants::wgs84_semi_major_axis_m, f.earth);
    const auto pad = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const auto w = mission::plan_launch_window(pad, f.earth, sim::station_orbit(mean, f.earth), f.t0, p,
                                               constants::earth_gm_m3ps2);
    REQUIRE(w.feasible);
    // off the liftoff: refused, nothing changed
    CHECK_FALSE(sim::launch_campaign_flight(f.session, dir, m, w, p).refusal.empty());
    CHECK(f.session.body_count() == 1u);
    f.session.advance_to(w.liftoff);
    const auto s = sim::launch_campaign_flight(f.session, dir, m, w, p);
    REQUIRE(s.refusal.empty());
    REQUIRE(f.session.iss_mission());
    CHECK_FALSE(f.session.iss_mission()->ship_return.has_value());
    CHECK(f.session.iss_mission()->transfer == sim::MissionTransfer::TankerReserve);
    CHECK(f.session.station_index().has_value());
    while (f.session.iss_mission_leg() == IssMissionLeg::Ascent && f.session.epoch() - w.liftoff < 1200.0) {
        f.session.advance_to(f.session.epoch() + 1.0);
    }
    CHECK((f.session.iss_mission_leg() == IssMissionLeg::Raise || f.session.iss_mission_leg() == IssMissionLeg::Rendezvous));
}
```

- [ ] **Step 2: Run to verify it fails** - `-TestFilter "[campaign_flown]"`. Expected: compile errors.

- [ ] **Step 3: Implement** - extract from `start_refuelling_flight_mission`'s Pad branch:

```cpp
IssMissionSequence pad_flight_sequence(const config::RefuellingFlightMission& m,
                                       const mission::LaunchWindowParameters& p, const mission::LaunchWindow& w) {
    IssMissionSequence q;
    q.station_port = m.depot.port;
    q.max_capture_retries = capture_retries;
    q.from_launch = true;
    q.phasing = p.phasing;
    q.phasing.mission_limit_s += m.window_margin_s;  // the flown phasing keeps the full limit
    q.raise_to = w.phasing_orbit;
    q.transfer = m.kind == config::MissionKind::TankerFlight ? MissionTransfer::TankerReserve
                                                             : MissionTransfer::FillChaser;
    return q;
}

MissionStart launch_campaign_flight(FlightSession& session, const std::filesystem::path& config_dir,
                                    const config::RefuellingFlightMission& m, const mission::LaunchWindow& w,
                                    const mission::LaunchWindowParameters& p) {
    MissionStart s;
    s.kind = m.kind;
    s.launch_window = w;
    if (!w.feasible) {
        s.refusal = w.reason;
        return s;
    }
    if (std::abs(session.epoch() - w.liftoff) > FlightSession::tick_s) {
        s.refusal = "Kampagnenflug: die Sitzung steht nicht am Startzeitpunkt";
        return s;
    }
    const auto ascent = config::load_ascent_mission(config_dir, m.ascent_id);
    start_ascent_mission(session, ascent, config_dir, w.liftoff, nullptr, true);
    session.set_iss_mission(pad_flight_sequence(m, p, w));
    s.epoch = w.liftoff;
    s.site_id = ascent.site_id;
    s.return_planned = true;
    return s;
}
```

  `start_refuelling_flight_mission`'s Pad branch uses `pad_flight_sequence` (its `q` from it, then adds the ship
  return / continuation as now). The orbit start branch keeps its own sequence.

- [ ] **Step 4: Run to verify it passes** - `-TestFilter "[campaign_flown]"`, then `-TestFilter "[refuelling][mission]"`
  (background; the r5 flights must be unchanged - they check the reference numbers). Tag the new case `[slow]` if
  over ~40 s.

- [ ] **Step 5: Commit and push**

```bash
git add core/include/starsim/sim/refuelling_mission.hpp core/src/sim/refuelling_mission.cpp core/tests/test_refuelling_campaign_flight.cpp
git commit -m "feat(sim): campaign flights launch beside the kept depot"
git push origin main
```

---

### Task 6: Campaign mission config

**Files:**
- Modify: `core/include/starsim/config/mission_config.hpp` (`MissionKind::RefuellingCampaignFlown`)
- Modify: `core/include/starsim/config/refuelling_mission_config.hpp`, `core/src/config/mission_config.cpp`
- Create: `config/missions/refuelling_campaign_flown.toml`, `config/missions/refuelling_campaign_flown_placed.toml`
- Test: `core/tests/test_refuelling_mission_config.cpp`

**Interfaces:**
- Produces:

```cpp
// refuelling_campaign_flown (f2): the campaign flown in one session with the reference flights' missions
// (config/refuelling/reference_flights.toml) - the depot launched (depot_launch) or placed (tanker_flight's [depot] at
// its earliest_utc), its station keeping, the first tanker's return, the comparison's tolerances.
struct RefuellingCampaignFlownMission {
    std::string id, name;
    bool depot_placed = false;                 // [depot] start = "launch" | "placed"
    double placed_shortfall_tankers = 0.0;     // placed: fill = demand - this x the tanker's net delivery [test]
    double keeping_min_height_m = 0.0;         // [station_keeping] below this mean height: reboost
    double keeping_target_height_m = 0.0;      // ... to this circular height
    double keeping_min_lead_s = 0.0;           // ... only when the next arrival is further away
    std::vector<std::string> keeping_engine_groups;
    bool return_first_tanker = true;           // [flights]
    double fill_tolerance = 0.0;               // [comparison] share of the calculator's depot fill
    double duration_tolerance_s = 0.0;
};
RefuellingCampaignFlownMission load_refuelling_campaign_flown_mission(const std::filesystem::path& config_dir,
                                                                      std::string_view mission_id);
```

- [ ] **Step 1: Write the configs.** `config/missions/refuelling_campaign_flown.toml`:

```toml
# Refuelling f2: the whole campaign flown in one session (spec f2) - the depot launched, every tanker launched in a
# window planned live from the flown depot, the HLS refuelled at the end; compared with the campaign calculator.
[mission]
name = "Betankungskampagne (geflogen)"
kind = "refuelling_campaign_flown"

[depot]
start = "launch"                   # depot_launch of reference_flights.toml

[station_keeping]
min_height_km = 195.0              # [E] spec f2: mean height (J2 energy) below this -> reboost
target_height_km = 200.0           # [E] circular
min_lead_h = 24.0                  # [E] only when the next arrival is further away
engine_groups = ["sea_level_3"]    # the belly Raptor (cold gas when its transients exceed 60 % of a burn)

[flights]
return_first_tanker = true         # spec f2: only the first tanker flies its return (side session, plan decision 3)

[comparison]
fill_tolerance = 0.02              # [E] spec f2: depot fill per arrival
duration_days = 1.0                # [E] spec f2
```

  `config/missions/refuelling_campaign_flown_placed.toml`: the same, with `name = "Betankungskampagne (geflogen,
  Depot platziert)"`, `[depot] start = "placed"` and `shortfall_tankers = 1.5   # [test] fill = demand - 1.5 net
  tanker deliveries -> two tankers and the HLS`.

- [ ] **Step 2: Write the failing tests:**

```cpp
TEST_CASE("refuelling_campaign_flown configs load", "[refuelling][config]") {
    const auto dir = std::filesystem::path(STARSIM_CONFIG_DIR);
    CHECK(config::mission_kind(dir, "refuelling_campaign_flown") == config::MissionKind::RefuellingCampaignFlown);
    const auto m = config::load_refuelling_campaign_flown_mission(dir, "refuelling_campaign_flown");
    CHECK_FALSE(m.depot_placed);
    CHECK(m.keeping_min_height_m == Catch::Approx(195.0e3));
    CHECK(m.keeping_target_height_m == Catch::Approx(200.0e3));
    CHECK(m.keeping_min_lead_s == Catch::Approx(86400.0));
    CHECK(m.keeping_engine_groups == std::vector<std::string>{"sea_level_3"});
    CHECK(m.return_first_tanker);
    CHECK(m.fill_tolerance == Catch::Approx(0.02));
    CHECK(m.duration_tolerance_s == Catch::Approx(86400.0));
    const auto p = config::load_refuelling_campaign_flown_mission(dir, "refuelling_campaign_flown_placed");
    CHECK(p.depot_placed);
    CHECK(p.placed_shortfall_tankers == Catch::Approx(1.5));
    CHECK_THROWS(config::load_refuelling_campaign_flown_mission(dir, "tanker_flight"));  // wrong kind
}

TEST_CASE("refuelling_campaign_flown refuses bad values", "[refuelling][config]") {
    // write variants into a unique temp dir (missions/ subfolder), as the file's other refusal tests do:
    // target <= min height, negative tolerance, start = "orbit", placed without shortfall_tankers -> each throws
}
```

  (Write the second case's four variants concretely with the file's existing temp-config helper - read how
  `test_refuelling_mission_config.cpp` writes a broken mission file; use a temp dir name unique per test case.)

- [ ] **Step 3: Run to verify they fail** - `-TestFilter "[refuelling][config]"`.

- [ ] **Step 4: Implement** in `mission_config.cpp` (`"refuelling_campaign_flown"` in the kind parser, enum value
  after `HlsRefuel`) and the loader next to `load_depot_launch_mission`:

```cpp
RefuellingCampaignFlownMission load_refuelling_campaign_flown_mission(const std::filesystem::path& config_dir,
                                                                      std::string_view mission_id) {
    const detail::Document doc(config_dir / "missions" / (std::string(mission_id) + ".toml"));
    require_kind(doc, MissionKind::RefuellingCampaignFlown, "refuelling campaign flown");
    const auto& mission = doc.table(doc.root, "mission");
    const auto& depot = doc.table(doc.root, "depot");
    const auto& keeping = doc.table(doc.root, "station_keeping");
    const auto& flights = doc.table(doc.root, "flights");
    const auto& comparison = doc.table(doc.root, "comparison");
    RefuellingCampaignFlownMission m;
    m.id = std::string(mission_id);
    m.name = doc.text(mission, "name");
    const auto start = doc.text(depot, "start");
    if (start != "launch" && start != "placed") doc.fail("[depot] start must be \"launch\" or \"placed\"");
    m.depot_placed = start == "placed";
    if (m.depot_placed) m.placed_shortfall_tankers = doc.positive(depot, "shortfall_tankers");
    m.keeping_min_height_m = doc.positive(keeping, "min_height_km") * 1000.0;
    m.keeping_target_height_m = doc.positive(keeping, "target_height_km") * 1000.0;
    if (m.keeping_target_height_m <= m.keeping_min_height_m) doc.fail("[station_keeping] target_height_km <= min_height_km");
    m.keeping_min_lead_s = doc.positive(keeping, "min_lead_h") * 3600.0;
    m.keeping_engine_groups = /* the string array "engine_groups", non-empty - use the helper the vehicle loader uses for deorbit_engine_groups */;
    m.return_first_tanker = doc.boolean(flights, "return_first_tanker");
    m.fill_tolerance = doc.positive(comparison, "fill_tolerance");
    m.duration_tolerance_s = doc.positive(comparison, "duration_days") * 86400.0;
    return m;
}
```

  (Replace the `engine_groups` comment line with the string-array reader that `vehicle_config.cpp` uses for
  `deorbit_engine_groups`; if it is a local function there, move it into `toml_document.hpp` as a `Document` member
  `std::vector<std::string> strings(const toml::table&, std::string_view key) const` that fails on an empty array.)
  The `start_mission` dispatch for the new kind comes in Task 7 (a compiler warning about the unhandled enum value
  in between is fine; the build does not use warnings-as-errors).

- [ ] **Step 5: Run to verify they pass** - `-TestFilter "[refuelling][config]"`; also `-TestFilter "[config]"`.

- [ ] **Step 6: Commit and push**

```bash
git add config/missions/refuelling_campaign_flown.toml config/missions/refuelling_campaign_flown_placed.toml core/include/starsim/config/mission_config.hpp core/include/starsim/config/refuelling_mission_config.hpp core/src/config/mission_config.cpp core/src/config/toml_document.hpp core/tests/test_refuelling_mission_config.cpp
git commit -m "feat(config): refuelling_campaign_flown missions"
git push origin main
```

---

### Task 7: The campaign driver

**Files:**
- Create: `core/include/starsim/sim/refuelling_campaign_flight.hpp`, `core/src/sim/refuelling_campaign_flight.cpp`
  (add to `core/CMakeLists.txt` after `refuelling_campaign_setup.cpp`)
- Modify: `core/src/sim/mission_start.cpp` (dispatch), `core/include/starsim/sim/mission_start.hpp` (comment)
- Test: `core/tests/test_refuelling_campaign_flight.cpp`

**Interfaces:**
- Consumes: Task 2 (`SessionSupervisor`, `keep_only_station`, `place_vehicle`), Task 3 (`start_station_reboost`,
  `station_mean_height_m`, `ReboostRecord`), Task 4 (`CampaignLive`, `CampaignReport::{hls_demand, tanker_window,
  hls_window}`, `CampaignFlight::window`), Task 5 (`launch_campaign_flight`), Task 6 (the config).
- Produces:

```cpp
// Refuelling f2: the campaign flown in one session (spec f2 "Fully flown campaign") - installed as the session's
// supervisor. After the depot became the station it plans every flight live (plan_refuelling_campaign with
// CampaignLive), warps to the launch, flies it (launch_campaign_flight), removes it after its departure, reboosts the
// depot (start_station_reboost) and books the calculator's [E] attitude gas and transfer vent. The first plan is the
// calculator's answer the flight is compared with.
enum class CampaignFlightPhase { DepotAscent, Planning, WaitLaunch, Flying, StationKeeping, Complete, Ended };
const char* to_string(CampaignFlightPhase phase);  // German

struct CampaignFlownRow {
    int number = 0;      // tankers 1..n in launch order (failed ones included); the HLS: 0
    bool hls = false;
    bool failed = false;
    std::string note;    // German: why it failed, a window moved past the plan
    time::Epoch launch, arrival;  // liftoff; Waypoint 0 (the Proximity mark; failed before it: the end)
    std::optional<mission::PhasingOrbit> phasing_orbit;
    double delivered_kg = 0.0;    // tanker: into the depot (before the vent); HLS: received (before the vent)
    double depot_after_kg = 0.0;  // the depot's main tanks after the transfer and the vent booking
    std::optional<mission::CampaignTanker> calculator;  // the first plan's tanker of the same number
};

struct CampaignFlightLosses {  // flown and booked, from the start to the end
    double boil_off_kg = 0.0;  // [D] the remainder of the depot's balance (boil-off and the docked pairs' effects)
    double station_keeping_kg = 0.0, attitude_gas_kg = 0.0, transfer_vent_kg = 0.0;
};

struct CampaignFlightState {
    CampaignFlightPhase phase = CampaignFlightPhase::DepotAscent;
    std::string status;  // German
    std::optional<time::Epoch> start;  // the depot became the station
    int tankers_flown = 0;             // delivered (not failed)
    int tankers_planned = 0;           // the latest live plan: flown + still planned
    double depot_propellant_kg = 0.0;
    std::optional<time::Epoch> next_launch;
    bool next_is_hls = false;
    std::vector<CampaignFlownRow> rows;
    CampaignFlightLosses losses;
    std::vector<ReboostRecord> reboosts;
    std::optional<CampaignReport> calculator;  // the first plan
    std::optional<time::Epoch> hls_arrival;
    double hls_propellant_kg = 0.0, hls_required_propellant_kg = 0.0;
    std::optional<ReturnResult> first_return;
    double first_return_main_kg = 0.0;
    std::string first_return_note;  // German: why it did not fly / its status
};

class RefuellingCampaignFlight : public SessionSupervisor {
public:
    RefuellingCampaignFlight(CampaignSources sources, config::RefuellingCampaignFlownMission mission);
    void update(FlightSession& session) override;
    std::optional<time::Epoch> wake_epoch() const override;
    const CampaignFlightState& state() const { return state_; }
    // The spec's pass check against the first plan: German lines of every miss (empty: passed) - tanker count,
    // duration, depot fill per arrival, the HLS's propellant.
    std::vector<std::string> comparison_misses() const;
};

// start_mission's refuelling_campaign_flown: the depot launched (depot_launch) or placed, the driver installed. Needs
// a session with the Moon orientation (the HLS demand); MissionStart.refusal (German) otherwise.
MissionStart start_refuelling_campaign_flight(FlightSession& session, const std::filesystem::path& config_dir,
                                              const config::RefuellingCampaignFlownMission& mission,
                                              const time::TimeScaleConverter& time);
```

- [ ] **Step 1: Write the failing tests** (`test_refuelling_campaign_flight.cpp`; the session needs the Moon
  orientation - build it as the lunar tests do, e.g. `test::spice_session()` + `spice::SpiceOrientation::moon()`; read
  `lunar_test_support.hpp` for the exact helper):

```cpp
namespace {

struct CampaignSession {
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceOrientation moon = spice::SpiceOrientation::moon();
    const spice::SpiceTimeConverter clock;
    sim::FlightSession session{ephemeris, earth, refuelling_test::config_dir, environment::PointMassGravityConfig{}, &moon};
    const sim::RefuellingCampaignFlight* driver() const {
        return dynamic_cast<const sim::RefuellingCampaignFlight*>(session.supervisor());
    }
    // Advances in `step_s` calls until `done` or `limit_s` of world time.
    template <class Done>
    void fly(Done done, double limit_s, double step_s = 3600.0) {
        const auto t0 = session.epoch();
        while (!done() && session.epoch() - t0 < limit_s && session.active()) session.advance_to(session.epoch() + step_s);
    }
};

config::RefuellingCampaignFlownMission placed_mission() {
    return config::load_refuelling_campaign_flown_mission(refuelling_test::config_dir, "refuelling_campaign_flown_placed");
}

bool phase_is(const CampaignSession& c, sim::CampaignFlightPhase p) { return c.driver() && c.driver()->state().phase == p; }

}  // namespace

TEST_CASE("Flown campaign start: the placed depot is the station and the first flight is planned",
          "[campaign_flown][refuelling]") {
    test::spice_session();
    CampaignSession c;
    const auto s = sim::start_mission(c.session, refuelling_test::config_dir, "refuelling_campaign_flown_placed", {}, c.clock);
    REQUIRE(s.refusal.empty());
    REQUIRE(c.driver());
    c.fly([&] { return !phase_is(c, sim::CampaignFlightPhase::Planning) && !phase_is(c, sim::CampaignFlightPhase::DepotAscent); }, 600.0, 60.0);
    const auto& st = c.driver()->state();
    INFO(st.status);
    REQUIRE(st.phase == sim::CampaignFlightPhase::WaitLaunch);
    REQUIRE(st.calculator);
    CHECK(st.calculator->result.tanker_flights == 2);  // shortfall 1.5 net deliveries
    REQUIRE(st.next_launch);
    CHECK(*st.next_launch > c.session.epoch());
    CHECK_FALSE(st.next_is_hls);
    CHECK(c.session.body_count() == 1u);
}

TEST_CASE("Flown campaign: one time-warp call launches at the planned liftoff", "[campaign_flown][refuelling]") {
    test::spice_session();
    CampaignSession c;
    sim::start_mission(c.session, refuelling_test::config_dir, "refuelling_campaign_flown_placed", {}, c.clock);
    c.fly([&] { return phase_is(c, sim::CampaignFlightPhase::WaitLaunch); }, 600.0, 60.0);
    const auto liftoff = *c.driver()->state().next_launch;
    c.session.advance_to(liftoff + 10.0 * 86400.0);  // returns early: the ascent's ticks use the call's max_ticks
    REQUIRE(c.driver()->state().phase == sim::CampaignFlightPhase::Flying);
    REQUIRE_FALSE(c.driver()->state().rows.empty());
    CHECK(std::abs(c.driver()->state().rows.back().launch - liftoff) <= sim::FlightSession::tick_s);
}

TEST_CASE("Flown campaign: a failed tanker flight is recorded and the campaign re-plans", "[campaign_flown][refuelling]") {
    test::spice_session();
    CampaignSession c;
    sim::start_mission(c.session, refuelling_test::config_dir, "refuelling_campaign_flown_placed", {}, c.clock);
    c.fly([&] { return phase_is(c, sim::CampaignFlightPhase::Flying); }, 10.0 * 86400.0);
    REQUIRE(phase_is(c, sim::CampaignFlightPhase::Flying));
    c.session.mark_iss_leg_for_test(IssMissionLeg::Ended);
    c.fly([&] { return phase_is(c, sim::CampaignFlightPhase::WaitLaunch); }, 600.0, 1.0);
    const auto& st = c.driver()->state();
    REQUIRE(st.phase == sim::CampaignFlightPhase::WaitLaunch);
    REQUIRE(st.rows.size() == 1u);
    CHECK(st.rows[0].failed);
    CHECK(st.tankers_flown == 0);
    CHECK(c.session.body_count() == 1u);  // only the depot
    CHECK(c.session.station_index().has_value());
}

TEST_CASE("Flown campaign: station keeping when the depot is low and the next arrival far", "[campaign_flown][refuelling][reboost]") {
    test::spice_session();
    CampaignSession c;
    auto m = placed_mission();
    m.keeping_min_height_m = 205.0e3;     // the placed 200 km depot is "low"
    m.keeping_target_height_m = 210.0e3;
    m.keeping_min_lead_s = 3600.0;        // the first arrival is days away
    const auto s = sim::start_refuelling_campaign_flight(c.session, refuelling_test::config_dir, m, c.clock);
    REQUIRE(s.refusal.empty());
    c.fly([&] { return phase_is(c, sim::CampaignFlightPhase::WaitLaunch); }, 86400.0, 60.0);
    const auto& st = c.driver()->state();
    INFO(st.status);
    REQUIRE(st.reboosts.size() == 1u);
    CHECK(st.reboosts[0].completed);
    CHECK(c.session.station_mean_height_m() == Catch::Approx(210.0e3).margin(1.0e3));
    CHECK(st.losses.station_keeping_kg == Catch::Approx(st.reboosts[0].propellant_kg));
}

TEST_CASE("Flown campaign, placed depot: two tankers and the HLS", "[campaign_flown][refuelling][slow]") {
    test::spice_session();
    CampaignSession c;
    sim::start_mission(c.session, refuelling_test::config_dir, "refuelling_campaign_flown_placed", {}, c.clock);
    c.fly([&] { return phase_is(c, sim::CampaignFlightPhase::Complete) || phase_is(c, sim::CampaignFlightPhase::Ended); },
          40.0 * 86400.0);
    const auto& st = c.driver()->state();
    INFO(st.status);
    REQUIRE(st.phase == sim::CampaignFlightPhase::Complete);
    REQUIRE(st.calculator);
    CHECK(st.tankers_flown == st.calculator->result.tanker_flights);
    REQUIRE(st.rows.size() == 3u);
    CHECK(st.rows.back().hls);
    for (const auto& r : st.rows) {
        if (!r.hls) CHECK(r.delivered_kg > 50.0e3);
    }
    CHECK(st.hls_propellant_kg >= st.hls_required_propellant_kg);
    REQUIRE(st.first_return);
    CHECK((*st.first_return == sim::ReturnResult::Caught || *st.first_return == sim::ReturnResult::UprightAbort));
    CHECK(st.losses.attitude_gas_kg > 0.0);
    CHECK(st.losses.transfer_vent_kg > 0.0);
    const auto misses = c.driver()->comparison_misses();
    for (const auto& line : misses) UNSCOPED_INFO(line);
    CHECK(misses.empty());
    CHECK(c.session.body_count() == 1u);
}
```

- [ ] **Step 2: Run to verify they fail** - `-TestFilter "[campaign_flown]"`. Expected: compile errors.

- [ ] **Step 3: Implement** `refuelling_campaign_flight.cpp`. Private members of the class:

```cpp
    CampaignSources sources_;
    config::RefuellingCampaignFlownMission mission_;
    config::RefuellingReference reference_;         // reference_flights.toml (the missions)
    config::RefuellingFlightMission tanker_, hls_;
    CampaignFlightState state_;
    std::optional<CampaignHlsDemand> demand_;
    std::optional<mission::CampaignFlight> next_;
    mission::LaunchWindowParameters next_parameters_;
    std::optional<time::Epoch> last_launch_, last_arrival_, last_update_;
    std::optional<time::Epoch> planned_second_launch_;  // the previous plan's next-but-one launch (moved-window note)
    double pending_gas_kg_ = 0.0;
    double delivered_total_kg_ = 0.0, hls_received_kg_ = 0.0, initial_fill_kg_ = 0.0;
    double depot_vent_kg_ = 0.0;  // the vent booked on the depot (the HLS's is drawn from the HLS)
    std::string moved_note_;      // decision 2: the next row's note
    bool placed_fill_set_ = false;
    // private steps: begin, start_placed(time::Epoch), plan_next, launch, watch_flight, fly_first_return, finish, end,
    // book_attitude_gas (as below)
```

  Key functions (write them as below; helpers in an anonymous namespace):

```cpp
namespace {

const mission::CampaignInputs booked{};  // the calculator's [E] items (attitude gas, vent): one source

double main_kg(const FlightSession& s, size_t body) { return s.body(body).vehicle().main_propellant_kg(); }

// Draws `kg` from the body's main tanks in proportion to their contents (attitude gas, vent bookings).
void draw_main(FlightSession& s, size_t body, double kg) {
    const auto& v = s.body(body).vehicle();
    const size_t stage = v.stages().size() - 1;
    const double total = v.main_propellant_kg();
    if (total <= 0.0) return;
    std::vector<std::pair<std::string, double>> masses;
    for (const auto& t : v.stages().back().tanks) {
        if (t.config.role == config::TankRole::Main) masses.emplace_back(t.config.id, t.mass_kg * std::max(0.0, 1.0 - kg / total));
    }
    for (const auto& [id, mass] : masses) s.set_tank_mass(body, stage, id, mass);
}

bool transfer_running(const FlightSession& s) {
    const auto p = s.transfer_state().phase;
    return p == TransferPhase::Settling || p == TransferPhase::Chilldown || p == TransferPhase::Flowing ||
           p == TransferPhase::Paused;
}

const FlightSession::MissionMark* mark(const FlightSession& s, IssMissionLeg leg) {
    for (const auto& m : s.iss_mission_marks()) {
        if (m.leg == leg) return &m;
    }
    return nullptr;
}

}  // namespace

void RefuellingCampaignFlight::update(FlightSession& s) {
    const double dt = last_update_ ? s.epoch() - *last_update_ : 0.0;
    last_update_ = s.epoch();
    if (state_.phase == CampaignFlightPhase::Complete || state_.phase == CampaignFlightPhase::Ended) return;
    if (state_.start && s.station_index()) book_attitude_gas(s, dt);
    switch (state_.phase) {
        case CampaignFlightPhase::DepotAscent:
            if (s.iss_mission_leg() == IssMissionLeg::Ended) return end(s.iss_mission_status());
            if (s.iss_mission_leg() != IssMissionLeg::Complete) return;
            s.keep_only_station();  // the caught booster goes
            begin(s);
            return;
        case CampaignFlightPhase::Planning: return plan_next(s);
        case CampaignFlightPhase::StationKeeping:
            if (s.station_reboost_active()) return;
            state_.reboosts.push_back(*s.last_reboost());
            state_.losses.station_keeping_kg += s.last_reboost()->propellant_kg;
            state_.phase = CampaignFlightPhase::Planning;  // the orbit changed: plan again
            return;
        case CampaignFlightPhase::WaitLaunch:
            if (s.epoch() + FlightSession::tick_s < next_->window->liftoff) return;
            return launch(s);
        case CampaignFlightPhase::Flying: return watch_flight(s);
        default: return;
    }
}

std::optional<time::Epoch> RefuellingCampaignFlight::wake_epoch() const {
    if (state_.phase == CampaignFlightPhase::WaitLaunch && next_ && next_->window) return next_->window->liftoff;
    return std::nullopt;
}

void RefuellingCampaignFlight::book_attitude_gas(FlightSession& s, double dt_s) {
    pending_gas_kg_ += booked.attitude_gas_kgps * dt_s;
    if (pending_gas_kg_ < 1.0 || transfer_running(s) || s.station_reboost_active()) return;  // [E] 1 kg batches
    draw_main(s, *s.station_index(), pending_gas_kg_);
    state_.losses.attitude_gas_kg += pending_gas_kg_;
    pending_gas_kg_ = 0.0;
}

void RefuellingCampaignFlight::begin(FlightSession& s) {
    state_.start = s.epoch();
    plan_next(s);  // the first plan becomes the calculator's answer (placed: after the fill is set, see plan_next)
}

void RefuellingCampaignFlight::plan_next(FlightSession& s) {
    const size_t depot = *s.station_index();
    const auto& b = s.body(depot).state();
    CampaignLive live;
    live.epoch = s.epoch();
    live.depot_gcrf = {b.position_gcrf_m, b.velocity_gcrf_mps};
    live.depot_propellant_kg = main_kg(s, depot);
    live.previous_launch = last_launch_;
    live.previous_arrival = last_arrival_;
    live.demand = demand_;
    auto report = plan_refuelling_campaign(sources_, {}, &live);
    if (!demand_ && report.result.closes) demand_ = report.hls_demand;
    if (mission_.depot_placed && !placed_fill_set_) {  // decision 10: demand - k net deliveries, then the real plan
        if (!demand_) return end(report.result.reason);
        placed_fill_set_ = true;
        const double fill = report.demand_kg - mission_.placed_shortfall_tankers * report.tanker_base_delivered_kg *
                                                   (1.0 - booked.transfer_vent_fraction);
        draw_main(s, depot, std::max(0.0, live.depot_propellant_kg - std::max(0.0, fill)));
        return plan_next(s);
    }
    if (!state_.calculator) {
        state_.calculator = report;
        initial_fill_kg_ = live.depot_propellant_kg;
    }
    state_.depot_propellant_kg = live.depot_propellant_kg;
    if (!report.result.closes) return end(report.result.reason);
    const bool tanker = !report.result.tankers.empty();
    next_ = tanker ? report.result.tankers.front().flight : *report.result.hls;
    next_parameters_ = tanker ? report.tanker_window : report.hls_window;
    state_.tankers_planned = state_.tankers_flown + report.result.tanker_flights;
    state_.next_launch = next_->window->liftoff;
    state_.next_is_hls = !tanker;
    if (planned_second_launch_ && std::abs(*state_.next_launch - *planned_second_launch_) > 3600.0) {
        moved_note_ = "Fenster verschoben gegenüber der vorigen Planung";  // decision 2 (std::string member)
    }
    planned_second_launch_.reset();
    if (tanker && report.result.tankers.size() > 1) {
        planned_second_launch_ = report.start + report.result.tankers[1].flight.launch_s;
    } else if (tanker && report.result.hls) {
        planned_second_launch_ = report.start + report.result.hls->launch_s;
    }
    // decision 6: station keeping when low, nothing in flight and the next arrival far enough away
    const double arrival_in_s = report.start + next_->arrival_s - s.epoch();
    if (s.station_mean_height_m() < mission_.keeping_min_height_m && arrival_in_s > mission_.keeping_min_lead_s) {
        if (s.start_station_reboost(mission_.keeping_target_height_m, mission_.keeping_engine_groups)) {
            state_.phase = CampaignFlightPhase::StationKeeping;
            state_.status = "Bahnhaltung des Depots";
            return;
        }
        state_.status = "Bahnhaltung abgelehnt: " + s.reboost_status();  // fly on
    }
    state_.phase = CampaignFlightPhase::WaitLaunch;
    state_.status = std::string(tanker ? "Tanker " + std::to_string(state_.rows.size() + 1) : std::string("HLS")) +
                    ": warten auf das Startfenster";
}

void RefuellingCampaignFlight::launch(FlightSession& s) {
    const auto& m = state_.next_is_hls ? hls_ : tanker_;
    const auto start = launch_campaign_flight(s, sources_.config_dir, m, *next_->window, next_parameters_);
    if (!start.refusal.empty()) {
        state_.status = "Start abgelehnt: " + start.refusal;
        state_.phase = CampaignFlightPhase::Planning;  // the next window
        return;
    }
    CampaignFlownRow row;
    row.hls = state_.next_is_hls;
    row.number = row.hls ? 0 : static_cast<int>(std::count_if(state_.rows.begin(), state_.rows.end(),
                                                              [](const CampaignFlownRow& r) { return !r.hls; })) + 1;
    row.launch = s.epoch();
    row.phasing_orbit = next_->window->phasing_orbit;
    row.note = std::exchange(moved_note_, {});
    state_.rows.push_back(row);
    last_launch_ = s.epoch();
    state_.phase = CampaignFlightPhase::Flying;
    state_.status = (row.hls ? std::string("HLS") : "Tanker " + std::to_string(row.number)) + ": im Flug";
}

void RefuellingCampaignFlight::watch_flight(FlightSession& s) {
    const auto leg = s.iss_mission_leg();
    if (leg != IssMissionLeg::Complete && leg != IssMissionLeg::Ended) return;
    auto& row = state_.rows.back();
    row.failed = leg == IssMissionLeg::Ended;
    if (row.failed) row.note = s.iss_mission_status();
    const auto* arrive = mark(s, IssMissionLeg::Proximity);
    row.arrival = arrive ? arrive->epoch : s.epoch();
    if (arrive) last_arrival_ = arrive->epoch;
    const auto* before = mark(s, IssMissionLeg::Transfer);
    const auto* after = mark(s, IssMissionLeg::Departure);
    const size_t depot = *s.station_index();
    if (before && after) {
        const double vent = booked.transfer_vent_fraction;
        if (row.hls) {
            row.delivered_kg = after->chaser_main_propellant_kg - before->chaser_main_propellant_kg;
            draw_main(s, s.controlled_index(), vent * row.delivered_kg);
            hls_received_kg_ += row.delivered_kg;
            state_.hls_propellant_kg = s.body(s.controlled_index()).vehicle().propellant_kg();
            state_.hls_arrival = row.arrival;
        } else {
            row.delivered_kg = after->station_main_propellant_kg - before->station_main_propellant_kg;
            draw_main(s, depot, vent * row.delivered_kg);
            depot_vent_kg_ += vent * row.delivered_kg;
            delivered_total_kg_ += row.delivered_kg;
        }
        state_.losses.transfer_vent_kg += vent * row.delivered_kg;
    }
    row.depot_after_kg = main_kg(s, depot);
    if (!row.hls && state_.calculator) {
        const auto& calc = state_.calculator->result.tankers;
        if (row.number >= 1 && static_cast<size_t>(row.number) <= calc.size()) row.calculator = calc[row.number - 1];
    }
    if (!row.hls && !row.failed) ++state_.tankers_flown;
    if (!row.hls && !row.failed && row.number == 1 && mission_.return_first_tanker) fly_first_return(s);
    s.keep_only_station();
    if (row.hls && !row.failed) return finish(s);
    state_.phase = CampaignFlightPhase::Planning;
}
```

  `fly_first_return(s)` (decision 3): a side `FlightSession side(sources_.ephemeris, sources_.earth,
  sources_.config_dir, environment::PointMassGravityConfig{}, &sources_.moon)` with
  `side.set_ignition_settling(s.ignition_settling())`, `side.place_vehicle(tanker.vehicle().copy_upper(0),
  tanker.state(), s.epoch())`, `side.set_launch_site(launch_site_from_config(site of the tanker ascent))`, then
  `side.plan_ship_return(ship_deorbit_parameters(config::load_ship_return_mission(dir, tanker_.return_id).deorbit))`;
  refused -> `state_.first_return = ReturnResult::Lost`, `first_return_note` = the plan's reason. Else loop
  `side.advance_to(side.epoch() + 3600.0)` while `side.ship_return_result(side.controlled_index()) ==
  ReturnResult::Flying` and less than `FlightSession::tanker_reserve_search_s + 86400.0` elapsed (then
  `first_return_note = "Rückkehr nicht beendet"`); record the result and `main_propellant_kg()` at the end.

  `finish(s)`: phase `Complete`, status "Kampagne abgeschlossen"; the losses' remainder
  `boil_off_kg = initial_fill_kg_ + delivered_total_kg_ - hls_received_kg_ - station_keeping_kg - attitude_gas_kg -
  depot_vent_kg_ - main_kg(depot)`; `hls_required_propellant_kg = calculator.hls_required_mass_kg -
  calculator.hls_dry_mass_kg`.

  `end(reason)`: phase `Ended`, status "Kampagne beendet: " + reason.

  `comparison_misses()`:

```cpp
std::vector<std::string> RefuellingCampaignFlight::comparison_misses() const {
    std::vector<std::string> misses;
    if (!state_.calculator) return {"keine Rechnerplanung"};
    const auto& calc = state_.calculator->result;
    char line[200];
    if (state_.tankers_flown != calc.tanker_flights) {
        std::snprintf(line, sizeof line, "Tankerzahl: geflogen %d, Rechner %d", state_.tankers_flown, calc.tanker_flights);
        misses.emplace_back(line);
    }
    if (!state_.hls_arrival) {
        misses.emplace_back("HLS nicht angekommen");
    } else if (const double flown_s = *state_.hls_arrival - *state_.start;
               std::abs(flown_s - calc.duration_s) > mission_.duration_tolerance_s) {
        std::snprintf(line, sizeof line, "Dauer: geflogen %.2f d, Rechner %.2f d", flown_s / 86400.0, calc.duration_s / 86400.0);
        misses.emplace_back(line);
    }
    for (const auto& r : state_.rows) {
        if (r.hls || r.failed || !r.calculator) continue;
        const double c = r.calculator->depot_after_kg;
        if (std::abs(r.depot_after_kg - c) > mission_.fill_tolerance * c) {
            std::snprintf(line, sizeof line, "Tanker %d: Depot %.1f t geflogen, %.1f t Rechner", r.number,
                          r.depot_after_kg / 1000.0, c / 1000.0);
            misses.emplace_back(line);
        }
    }
    if (state_.hls_propellant_kg < state_.hls_required_propellant_kg) misses.emplace_back("HLS: zu wenig Treibstoff");
    return misses;
}
```

  `start_refuelling_campaign_flight` (constructor loads `reference_ = config::load_refuelling_reference(dir)`,
  `tanker_ = load_refuelling_flight_mission(dir, reference_.tanker.mission)`, `hls_ = ...(reference_.hls.mission)`):
  - no Moon orientation -> refusal "Kampagne: die Sitzung hat keine Mondorientierung".
  - launched: `start_depot_launch_mission(session, dir, load_depot_launch_mission(dir, reference_.depot.mission), time)`.
  - placed: `session.clear_bodies()`; the depot `config::top_stage_config(load_vehicle(dir, tanker_.depot.vehicle_id))`
    filled with `fill_main_tanks(stage, depot capacity)` (full; plan_next draws it down to the placed fill), state
    `depot_orbit(tanker_.depot, earliest, earth)(earliest)` with `earliest = time.from_utc(tanker_.earliest_utc)`,
    `session.add_station(...)`; driver phase `Planning` with `state_.start = earliest` (set in the constructor's
    caller: add a `void start_placed(time::Epoch)` member or let `update` call `begin` on its first call when the
    session has only a station and no sequence - pick the member, it is explicit).
  - `session.set_supervisor(std::make_shared<RefuellingCampaignFlight>(...))` after the start (the depot launch's
    `start_ascent_mission` does not clear it; `start_mission` cleared it before).
  - `MissionStart.kind = config::MissionKind::RefuellingCampaignFlown`, `epoch` = the start, `return_planned = true`.
  `mission_start.cpp`: the new `case` calling it with
  `config::load_refuelling_campaign_flown_mission(config_dir, mission_id)`; update the dispatch comment.

- [ ] **Step 4: Run to verify they pass** - `-TestFilter "[campaign_flown]"` in the background (the placed campaign is
  `[slow]`, ~3 flights + ~10 days of coast: expect 10-20 min) with output to a file; check every 10 min. On a
  failure: read the driver's status and the rows, fix the cause; the comparison `misses` of the placed campaign must
  be empty - a miss caused by the calculator is fixed there (and its tests updated), a miss caused by the flight is
  fixed in the flight; report any that cannot be resolved to the user.

- [ ] **Step 5: Commit and push**

```bash
git add core/include/starsim/sim/refuelling_campaign_flight.hpp core/src/sim/refuelling_campaign_flight.cpp core/CMakeLists.txt core/src/sim/mission_start.cpp core/include/starsim/sim/mission_start.hpp core/tests/test_refuelling_campaign_flight.cpp
git commit -m "feat(sim): refuelling campaign flown in one session, planned live from the flown depot"
git push origin main
```

---

### Task 8: Game interface and CLI

**Files:**
- Modify: `core/include/starsim/sim/refuelling_campaign_text.hpp`, `core/src/sim/refuelling_campaign_text.cpp`
- Modify: `core/apps/refuelling_report.cpp`
- Modify: `bindings/src/flight_snapshot.hpp`, `bindings/src/flight_snapshot.cpp`, `bindings/src/flight_snapshot_refuelling.cpp`, `bindings/src/flight_sim.hpp`, `bindings/src/flight_sim.cpp`
- Modify: `game/tests/smoke_test.gd` (the getter exists and returns `active == false` without a campaign)
- Test: `core/tests/test_refuelling_campaign.cpp` (text)

**Interfaces:**
- Consumes: Task 7 `CampaignFlightState`, `RefuellingCampaignFlight::comparison_misses`.
- Produces:
  - `std::string sim::campaign_flight_text(const CampaignFlightState& state, const std::vector<std::string>& misses, const time::TimeScaleConverter& time);`
  - `godot::Dictionary FlightSim::campaign_flight_state() const;` keys: `active` (bool), `phase`, `status` (German
    strings), `tankers_flown`, `tankers_planned` (int), `depot_propellant_kg`, `next_launch_tdb_s` (or absent),
    `next_is_hls`, `rows` (Array of Dictionaries: `number`, `hls`, `failed`, `note`, `launch_tdb_s`,
    `arrival_tdb_s`, `delivered_kg`, `depot_after_kg`, `calc_launch_tdb_s`, `calc_arrival_tdb_s`,
    `calc_depot_after_kg` - the calc keys absent without a calculator row), `losses` (Dictionary: `boil_off_kg`,
    `station_keeping_kg`, `attitude_gas_kg`, `transfer_vent_kg`), `misses` (Array of German strings),
    `calc_tanker_flights`, `calc_duration_s`.
  - CLI: `refuelling_report --fly` / `--fly-placed` (no other flags with them).

- [ ] **Step 1: Write the failing text test:**

```cpp
TEST_CASE("Flown campaign text: rows, losses, verdict", "[refuelling][campaign]") {
    test::spice_session();
    const spice::SpiceTimeConverter clock;
    sim::CampaignFlightState st;
    st.phase = sim::CampaignFlightPhase::Complete;
    st.start = clock.from_utc({2026, 10, 20, 0, 0, 0.0});
    sim::CampaignFlownRow row;
    row.number = 1;
    row.launch = *st.start + 3.0 * 86400.0;
    row.arrival = *st.start + 4.5 * 86400.0;
    row.delivered_kg = 70.4e3;
    row.depot_after_kg = 122.6e3;
    st.rows.push_back(row);
    st.tankers_flown = 1;
    const auto text = sim::campaign_flight_text(st, {}, clock);
    CHECK_THAT(text, Catch::Matchers::ContainsSubstring("70.4"));
    CHECK_THAT(text, Catch::Matchers::ContainsSubstring("122.6"));
    CHECK_THAT(text, Catch::Matchers::ContainsSubstring("Comparison: passed"));
    const auto failed = sim::campaign_flight_text(st, {"Tankerzahl: geflogen 1, Rechner 2"}, clock);
    CHECK_THAT(failed, Catch::Matchers::ContainsSubstring("Tankerzahl"));
    CHECK_THAT(failed, Catch::Matchers::ContainsSubstring("Comparison: FAILED"));
}
```

- [ ] **Step 2: Run to verify it fails** - `-TestFilter "[campaign]"`.

- [ ] **Step 3: Implement**
  - `campaign_flight_text`: English report like `campaign_report_text` (CLI output is English there - keep that
    file's convention): header "Flown refuelling campaign from <UTC start>", one line per row
    `#  launch (d)  arrival (d)  orbit  delivered (t)  depot after (t)  calc depot after (t)  note`, the losses line,
    reboosts ("station keeping: N reboosts, X t"), the first return ("first tanker return: <to_string(result)>, main
    X t"), then "Comparison: passed" or "Comparison: FAILED" followed by the misses, one per line.
  - `refuelling_report.cpp`: `--fly` / `--fly-placed` build a `FlightSession` with the Moon orientation, call
    `sim::start_mission(session, dir, "refuelling_campaign_flown[_placed]", {}, time)`, loop
    `session.advance_to(session.epoch() + 3600.0)` until the phase is Complete / Ended (print one progress line to
    stderr per new row), then print `campaign_flight_text(...)`; exit code 0 when passed, 2 when the comparison failed,
    1 on Ended. Document the flags in the usage text and in the header comment of `refuelling_campaign_text.hpp`.
  - bindings: in `flight_snapshot_refuelling.cpp` a `godot::Dictionary build_campaign_flight_state(const
    sim::FlightSession&)` that `dynamic_cast`s `session.supervisor()` to `const sim::RefuellingCampaignFlight*`
    (null -> `{active: false}`), filled with the keys above; `FlightSnapshot::campaign_flight_state` filled where
    `iss_state` is built (and `absent()` where those are reset); `FlightSim::campaign_flight_state()` getter +
    `ClassDB::bind_method(D_METHOD("campaign_flight_state"), ...)` next to `iss_state`.
  - `smoke_test.gd`: after the existing FlightSim checks, `assert(not sim.campaign_flight_state().get("active", true))`
    in the file's style (read it first).

- [ ] **Step 4: Verify** - `-TestFilter "[campaign]"`; full build with bindings:
  `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -TestFilter "[campaign_flown]"` (background) and the smoke
  test `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd`.
  Run `build/core/refuelling_report.exe --fly-placed > <scratchpad>/fly_placed.txt` in the background and read it.

- [ ] **Step 5: Commit and push**

```bash
git add core/include/starsim/sim/refuelling_campaign_text.hpp core/src/sim/refuelling_campaign_text.cpp core/apps/refuelling_report.cpp bindings/src game/tests/smoke_test.gd core/tests/test_refuelling_campaign.cpp
git commit -m "feat(bindings): campaign_flight_state; refuelling_report --fly"
git push origin main
```

---

### Task 9: The full flown campaign, comparison, docs, review

**Files:**
- Test: `core/tests/test_refuelling_campaign_flight.cpp`
- Modify (if the comparison finds calculator causes): `core/src/sim/refuelling_campaign_setup.cpp` /
  `core/src/mission/refuelling_campaign.cpp` and their tests
- Modify: `docs/subsystems.md`, `CLAUDE.md`

- [ ] **Step 1: Write the full campaign test:**

```cpp
TEST_CASE("Flown campaign: the depot launched, every tanker, the HLS - compared with the calculator",
          "[campaign_flown][refuelling][slow]") {
    test::spice_session();
    CampaignSession c;
    const auto s = sim::start_mission(c.session, refuelling_test::config_dir, "refuelling_campaign_flown", {}, c.clock);
    REQUIRE(s.refusal.empty());
    c.fly([&] { return phase_is(c, sim::CampaignFlightPhase::Complete) || phase_is(c, sim::CampaignFlightPhase::Ended); },
          120.0 * 86400.0);
    const auto& st = c.driver()->state();
    std::printf("%s\n", sim::campaign_flight_text(st, c.driver()->comparison_misses(), c.clock).c_str());
    INFO(st.status);
    REQUIRE(st.phase == sim::CampaignFlightPhase::Complete);
    CHECK(c.driver()->comparison_misses().empty());
    REQUIRE(st.first_return);
    CHECK((*st.first_return == sim::ReturnResult::Caught || *st.first_return == sim::ReturnResult::UprightAbort));
    CHECK(st.hls_propellant_kg >= st.hls_required_propellant_kg);
}
```

- [ ] **Step 2: Run it in the background** - `-TestFilter "Flown campaign: the depot launched*"` (ctest name filter
  as `build.ps1` supports; else the tag plus `--` the case name), output to
  `<scratchpad>/campaign_flown.txt`; check every 10 minutes (rows so far, a flight hanging far beyond ~3 min of compute,
  the status). Expected compute: 30-45 min (spec).

- [ ] **Step 3: Compare and correct.** Read the printed table. For every miss: find whether the calculator or the
  flight causes it (e.g. the depot's phase drift between reboosts moves the windows; the booked gas; the transfer's
  delivery at a different phasing). Calculator causes are fixed in the calculator with a test in
  `test_refuelling_campaign.cpp` and the default answer re-checked with the CLI (13 tankers / 49.7 d, or the new
  answer reported); flight causes in the flight. If a miss stays (e.g. the depot's drag decay shifts the phase cycle
  by more than a day), STOP and report the numbers and the cause to the user - do not widen the tolerances.

- [ ] **Step 4: Docs.** `docs/subsystems.md`: a section "### Flown campaign (f2)" under the refuelling sections -
  the driver's phases, the supervisor hook, decisions 1-11, the energy-mean semi-major axis, the reboost (burn
  allocation found, propellant per reboost, how many reboosts), the booked items, the comparison table of the full
  run (calculator vs flown: count, days, fill per arrival, losses), run times, known limits (sequential flights,
  blocking side return). `CLAUDE.md`: in the module summary's `sim` list add "the refuelling campaign flown in one
  session (supervisor: live planning from the flown depot, station-keeping reboosts, comparison with the calculator)"
  and in `orbit` "energy-mean semi-major axis"; in "Vehicle configuration" mention `refuelling_report --fly`.

- [ ] **Step 5: Commit and push**

```bash
git add core/tests/test_refuelling_campaign_flight.cpp docs/subsystems.md CLAUDE.md
git commit -m "test: the fully flown refuelling campaign against the calculator; docs"
git push origin main
```

- [ ] **Step 6: Final review** - one reviewer (most capable model) over `git diff <first f2 commit>^..HEAD`, with the
  spec section, this plan and the Review Focus list; fix Critical / Important findings (tests of the affected tags
  green, push), record deferred minors in the ledger and the memory, then ask the user for `/clear`.
