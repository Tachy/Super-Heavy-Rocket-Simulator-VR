# Lunar l3: Outbound Flight (TLI, MCC, LOI / NRHO, NRHO <-> LLO) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The HLS, fully fuelled in a 200 x 200 km Earth orbit, flies on its autopilot to the Moon - best departure,
finite TLI on the vacuum Raptors, Sun-pointed coast, two mid-course corrections, insertion - into either a 100 km polar
LLO (< 1 km off after the trims) or the Artemis 9:2 southern L2 NRHO (a reference built by multiple shooting in the full
force model, kept for three revolutions), and transfers NRHO -> LLO and LLO -> NRHO with two burns.

**Architecture:** Pure planning lives in `mission/` (no session, no threads): a fixed-time position targeter in the full
force model, a CR3BP module, the NRHO reference (CR3BP periodic orbit -> ephemeris-model multiple shooting), the
outbound planner (TLI / MCC / insertion for both orbits) and the lunar orbit transfers. `sim::FlightSession` flies them
as steps of a new `AutopilotState::Lunar` (pattern of `CapsuleReturnStep`) in its own source file
`core/src/sim/flight_session_lunar.cpp`. A mission config (`kind = "lunar_landing"`) places the HLS and starts the
outbound flight through `sim::start_mission`; l4 extends the same mission with descent and ascent.

**Tech Stack:** C++20 core (Catch2), DOP853 coasts (`orbit::propagate_coast`), CSPICE via the existing adapters, toml++
inside `config`.

**Spec:** `docs/superpowers/specs/2026-10-14-lunar-missions-design.md` (section "l3 design: outbound flight"; l1 and l2
are on `main`, 84e4fcf).

## Global Constraints

- Work on `main` (no feature branch); commit after every task, push (`git push origin main`) when its tests are green.
- SI internally, radians; frame and unit in names (`position_gcrf_m`, `epoch_tdb_s`, `delta_v_gcrf_mps`).
- Vehicle states stay Earth-centred GCRF; nothing switches frames at a sphere of influence. Moon-relative quantities
  are differences against the DE440 Moon (`ephemeris.state_gcrf(Body::Moon, t)`).
- All planning uses the session's force model (`environment::PointMassGravity`: Earth + J2, Moon, Sun, DE440) so plans
  and flight agree; CSPICE only on the simulation thread (all l3 planning runs there).
- Moon: sphere R = 1737.4 km (`constants::moon_mean_radius_m`); lunar inclinations relative to the MOON_ME equator.
- LLO: 100 km circular, polar (90 deg). NRHO: 9:2 synodic-resonant southern L2 NRHO, perilune over the north pole,
  apolune ~70,000 km over the south pole [S].
- HLS: `starship_hls` (120 t dry [E], 1,600 t propellant [S]); TLI, LOI, NRHO insertion and the transfer burns on the
  engine group `vacuum` (3 x Raptor Vacuum 3, 2.98 MN, 380 s [S]); small burns on RCS by the session's own choice
  (`execute_burn(node)`); the landing thrusters are never used (l2: `landing_only`).
- No propellant is ever set mid-mission (user 2026-10-14); a plan that does not close is refused with a German message.
- No magic numbers: constants with source in `math/constants.hpp`; mission values in the mission TOML with
  [S]/[E]/[D] tags.
- UI strings in the core German; code, comments, docs English. Commits: Conventional Commits with the session trailer.
- Drawing, UI and bindings: untouched (l6). `docs/ansichten.md` untouched.
- Existing tests keep their outcomes (`test_lunar_transfer.cpp` keeps passing unchanged).

## Review Focus

1. **Time warp across a step:** one `advance_to(t0 + 6 d)` call (as the game's high warp) must not coast past an
   ignition, an MCC or the insertion: the coast allowance stops at every step epoch. Test in Task 6.
2. **Pilot takes over mid-transit:** `disengage_autopilot()` during the coast ends the lunar steps for good; a later
   `advance_to` must not resume an MCC or fire the insertion. Test in Task 6.
3. **A plan that does not close:** too little propellant on board (an HLS with half tanks), or no departure (Moon's
   declination above the parking inclination): `plan_lunar_outbound` returns false with a German reason, the autopilot
   stays off, nothing fires. Tests in Tasks 6 and 7.
4. **Planning twice:** a second `plan_lunar_outbound` while a flight is under way returns false and leaves the running
   plan untouched. Test in Task 6.
5. **NRHO kept beyond the reference's span:** a flight that stays in the NRHO past the last patch point must not read
   the reference outside its span (station keeping stops with a German status, no exception, no extrapolated target).
   Test in Task 3 (`NrhoReference::covers`) and Task 6.

## File Structure

| File | Responsibility |
|---|---|
| `core/include/starsim/math/constants.hpp` | `synodic_month_s` [S] |
| `core/include/starsim/mission/position_targeter.hpp`, `core/src/mission/position_targeter.cpp` | delta-v at a burn epoch so the coast reaches a GCRF position at a fixed epoch (full model, Newton) |
| `core/include/starsim/orbit/cr3bp.hpp`, `core/src/orbit/cr3bp.cpp` | CR3BP dynamics, periodic-orbit correction (fixed period), rotating <-> GCRF conversion at an epoch |
| `core/include/starsim/mission/nrho_reference.hpp`, `core/src/mission/nrho_reference.cpp` | NRHO reference: CR3BP seed -> multiple shooting in the full model; state at an epoch, perilune / apolune epochs |
| `core/include/starsim/mission/lunar_outbound.hpp`, `core/src/mission/lunar_outbound.cpp` | outbound planning: TLI, MCC re-targeting, LOI / NRHO insertion, LLO apsis trims, propellant check |
| `core/include/starsim/mission/lunar_orbit_transfer.hpp`, `core/src/mission/lunar_orbit_transfer.cpp` | NRHO -> LLO and LLO -> NRHO two-burn transfers, NRHO station keeping at apolune |
| `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session_lunar.cpp`, `core/src/sim/flight_session.cpp` | `AutopilotState::Lunar`, `LunarStep`, `plan_lunar_outbound`, `plan_lunar_orbit_transfer`, step machine, coast allowance |
| `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp` | `MissionKind::LunarLanding`, `LunarLandingMission`, `load_lunar_landing_mission` |
| `core/include/starsim/sim/lunar_landing_mission.hpp`, `core/src/sim/lunar_landing_mission.cpp` | places the HLS in its parking orbit (RAAN through the Moon at arrival) and builds the outbound parameters |
| `core/src/sim/mission_start.cpp` | dispatch of `lunar_landing` |
| `config/missions/hls_lunar_llo.toml`, `config/missions/hls_lunar_nrho.toml` | the two outbound variants |
| `core/tests/test_position_targeter.cpp`, `test_cr3bp.cpp`, `test_nrho_reference.cpp`, `test_lunar_outbound.cpp`, `test_lunar_orbit_transfer.cpp`, `test_lunar_outbound_session.cpp`, `test_lunar_landing_mission.cpp` | tests |
| `core/CMakeLists.txt`, `core/tests/CMakeLists.txt` | new sources |
| `CLAUDE.md`, `config/README.md` | docs (Task 8) |

Build and test commands used below (PowerShell, repo root):
- Build: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests > $env:TEMP\l3_build.txt; Get-Content $env:TEMP\l3_build.txt -Tail 30`
- One test: `build/core/tests/starsim_core_tests.exe "<test name or [tag]>" > $env:TEMP\l3_test.txt; Get-Content $env:TEMP\l3_test.txt -Tail 40`

---

### Task 1: Fixed-time position targeter

**Files:**
- Create: `core/include/starsim/mission/position_targeter.hpp`, `core/src/mission/position_targeter.cpp`
- Modify: `core/CMakeLists.txt` (add `src/mission/position_targeter.cpp` after `src/mission/lunar_transfer.cpp`),
  `core/tests/CMakeLists.txt` (add `test_position_targeter.cpp`)
- Test: `core/tests/test_position_targeter.cpp`

**Interfaces:**
- Consumes: `orbit::propagate_coast(gravity, state0, epoch0, epoch_end)`, `orbit::solve_lambert(r1, r2, tof, gm, normal)`.
- Produces:
  ```cpp
  namespace starsim::mission {
  struct PositionTarget { math::Vec3 position_gcrf_m; time::Epoch epoch; };
  struct PositionSolution {
      bool converged = false;
      int iterations = 0;
      gnc::BurnNode burn;                   // impulsive, at the burn epoch
      math::StateVector arrival_gcrf;       // coasted state at target.epoch (Earth-centred GCRF)
      double miss_m = 0.0;
  };
  class PositionTargeter {
  public:
      explicit PositionTargeter(const environment::GravityModel& earth_centred_gravity);
      math::StateVector coast(const math::StateVector& state, time::Epoch from, time::Epoch to) const;
      PositionSolution solve(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                             const PositionTarget& target, const math::Vec3& initial_delta_v,
                             double tolerance_m = 10.0, int max_iterations = 30) const;
  };
  }
  ```

- [ ] **Step 1: Write the failing test** `core/tests/test_position_targeter.cpp`:

```cpp
// Fixed-time position targeting in the full force model: a known manoeuvre is recovered from the point it reaches.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/gravity.hpp"
#include "starsim/mission/position_targeter.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

TEST_CASE("Position targeter recovers a known manoeuvre in the full force model", "[mission][targeter]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    const mission::PositionTargeter targeter{gravity};
    const auto t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    const double gm = ephemeris.gm_m3ps2(ephemeris::Body::Earth);
    const auto parking = orbit::state_from_elements({6578137.0, 0.0, 0.497, 1.2, 0.0, 0.0}, gm);

    // Truth: a 1 km/s prograde-ish burn at t0 + 600 s, the point reached 20 h later.
    const auto burn_epoch = t0 + 600.0;
    const auto at_burn = targeter.coast(parking, t0, burn_epoch);
    const math::Vec3 truth_dv = 1000.0 * normalize(at_burn.velocity_mps) + math::Vec3{0.0, 0.0, 50.0};
    math::StateVector kicked = at_burn;
    kicked.velocity_mps += truth_dv;
    const auto arrival_epoch = burn_epoch + 20.0 * 3600.0;
    const auto arrival = targeter.coast(kicked, burn_epoch, arrival_epoch);

    const auto solution = targeter.solve(parking, t0, burn_epoch, {arrival.position_m, arrival_epoch}, {}, 0.01);
    REQUIRE(solution.converged);
    CHECK(solution.miss_m < 0.01);
    CHECK(norm(solution.burn.delta_v_gcrf_mps - truth_dv) < 1e-4);
    CHECK(solution.burn.epoch == burn_epoch);
}

TEST_CASE("Position targeter reports an unreachable point instead of diverging", "[mission][targeter]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    const mission::PositionTargeter targeter{gravity};
    const auto t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    const auto parking = orbit::state_from_elements({6578137.0, 0.0, 0.497, 1.2, 0.0, 0.0},
                                                    ephemeris.gm_m3ps2(ephemeris::Body::Earth));
    // 1e10 m away in 60 s: no finite solution within the iteration budget.
    const auto solution = targeter.solve(parking, t0, t0, {{1e10, 0.0, 0.0}, t0 + 60.0}, {}, 10.0, 8);
    CHECK_FALSE(solution.converged);
    CHECK(std::isfinite(solution.miss_m));
}
```

- [ ] **Step 2: Run** the build → FAIL (`position_targeter.hpp` not found).

- [ ] **Step 3: Implement.** Header as in Interfaces (includes `environment/gravity.hpp`, `gnc/burn_node.hpp`,
`math/state_vector.hpp`, `time/epoch.hpp`). Source:

```cpp
#include "starsim/mission/position_targeter.hpp"

#include <array>
#include <cmath>

#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/orbit/lambert.hpp"

namespace starsim::mission {

using math::StateVector;
using math::Vec3;

namespace {
constexpr double jacobian_step_mps = 1e-3;   // finite-difference step of the delta-v columns
constexpr double max_step_mps = 500.0;       // Newton step limit (keeps a far first guess on the right arc)
}  // namespace

PositionTargeter::PositionTargeter(const environment::GravityModel& earth_centred_gravity)
    : gravity_(earth_centred_gravity) {}

StateVector PositionTargeter::coast(const StateVector& state, time::Epoch from, time::Epoch to) const {
    if (to - from == 0.0) return state;
    return orbit::propagate_coast(gravity_, state, from, to).state;
}

PositionSolution PositionTargeter::solve(const StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                         const PositionTarget& target, const Vec3& initial_delta_v,
                                         double tolerance_m, int max_iterations) const {
    const StateVector at_burn = coast(state, epoch, burn_epoch);
    auto arrive = [&](const Vec3& dv) {
        StateVector s = at_burn;
        s.velocity_mps += dv;
        return coast(s, burn_epoch, target.epoch);
    };
    PositionSolution solution;
    Vec3 dv = initial_delta_v;
    StateVector arrival = arrive(dv);
    Vec3 miss = arrival.position_m - target.position_gcrf_m;
    for (int it = 0; it < max_iterations && norm(miss) > tolerance_m; ++it) {
        solution.iterations = it + 1;
        // Jacobian d(arrival position)/d(dv), central differences.
        std::array<Vec3, 3> column;
        for (int k = 0; k < 3; ++k) {
            Vec3 e{};
            e[k] = jacobian_step_mps;
            column[k] = (arrive(dv + e).position_m - arrive(dv - e).position_m) / (2.0 * jacobian_step_mps);
        }
        const math::Mat3 jacobian = math::Mat3::from_columns(column[0], column[1], column[2]);
        Vec3 step = -(jacobian.inverse() * miss);
        if (norm(step) > max_step_mps) step = step * (max_step_mps / norm(step));
        // Backtracking line search on the miss distance.
        double scale = 1.0;
        for (int ls = 0; ls < 6; ++ls, scale *= 0.5) {
            const StateVector trial = arrive(dv + scale * step);
            const Vec3 trial_miss = trial.position_m - target.position_gcrf_m;
            if (norm(trial_miss) < norm(miss) || ls == 5) {
                dv += scale * step;
                arrival = trial;
                miss = trial_miss;
                break;
            }
        }
    }
    solution.converged = norm(miss) <= tolerance_m;
    solution.burn = {burn_epoch, dv};
    solution.arrival_gcrf = arrival;
    solution.miss_m = norm(miss);
    return solution;
}

}  // namespace starsim::mission
```

Check `math::Mat3` for `from_columns` and `inverse()` (`core/include/starsim/math/`); if they are named differently
use the existing names (e.g. build the matrix element-wise and solve with the existing 3x3 solver used by
`lunar_transfer.cpp`). Do not add a second linear-algebra helper. The private member is
`const environment::GravityModel& gravity_;`.

- [ ] **Step 4: Run** `"[targeter]"` → PASS (both cases).
- [ ] **Step 5: Commit** `feat(mission): fixed-time position targeter in the full force model`; push.

---

### Task 2: CR3BP: dynamics, fixed-period periodic orbit, conversion to GCRF

**Files:**
- Create: `core/include/starsim/orbit/cr3bp.hpp`, `core/src/orbit/cr3bp.cpp`
- Modify: `core/include/starsim/math/constants.hpp`, `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`
- Test: `core/tests/test_cr3bp.cpp`

**Interfaces:**
- Consumes: `integrators::integrate<6>` / DOP853 as used by `orbit/ballistic_propagator.cpp`; `ephemeris::Ephemeris`.
- Produces:
  ```cpp
  namespace starsim::orbit {
  // Circular restricted three-body problem, Earth-Moon, nondimensional (length: Earth-Moon distance, time: 1/n).
  // Rotating frame: origin at the barycentre, x from Earth to Moon, z along the orbital angular momentum.
  struct Cr3bpState { math::Vec3 position; math::Vec3 velocity; };
  integrators::State<6> cr3bp_derivative(double mu, const integrators::State<6>& y);
  Cr3bpState cr3bp_propagate(double mu, const Cr3bpState& s, double duration);
  double jacobi_constant(double mu, const Cr3bpState& s);
  // Symmetric periodic orbit through the x-z plane with y'(0) only: corrects (x0, z0, vy0) so that after half the
  // given period the orbit crosses y = 0 perpendicularly (vx = vz = 0). Empty when Newton does not converge.
  std::optional<Cr3bpState> correct_periodic_orbit(double mu, const Cr3bpState& guess, double period);
  // Instantaneous Earth-Moon rotating frame at an epoch (DE440): converts a nondimensional rotating state to an
  // Earth-centred GCRF state (instantaneous distance and angular rate as units).
  class EarthMoonRotatingFrame {
  public:
      EarthMoonRotatingFrame(const ephemeris::Ephemeris& ephemeris, time::Epoch epoch);
      double mu() const;
      math::StateVector to_gcrf(const Cr3bpState& rotating) const;
      Cr3bpState from_gcrf(const math::StateVector& gcrf) const;
      double time_unit_s() const;   // 1 / instantaneous angular rate
      double length_unit_m() const; // instantaneous Earth-Moon distance
  };
  }
  ```

- [ ] **Step 1: Constant.** Add to `constants.hpp` (next to the lunar constants):
```cpp
// Mean synodic month (new moon to new moon) [S: Meeus, Astronomical Algorithms, 2nd ed., ch. 49: 29.530588861 d].
inline constexpr double synodic_month_s = 29.530588861 * 86400.0;
```

- [ ] **Step 2: Write the failing test** `core/tests/test_cr3bp.cpp`:

```cpp
// CR3BP: Jacobi constant conserved, the 9:2 NRHO seed corrected to a periodic orbit of the resonant period, and the
// rotating <-> GCRF conversion round trip.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/cr3bp.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
// 9:2 southern L2 NRHO, state at apolune [S: Zimovan, Howell & Davis 2017, "Near rectilinear halo orbits and their
// application in cis-lunar space", Table 1 (Earth-Moon mu = 0.01215)].
const orbit::Cr3bpState nrho_seed{{1.0221, 0.0, -0.1821}, {0.0, -0.1033, 0.0}};
}  // namespace

TEST_CASE("CR3BP conserves the Jacobi constant", "[orbit][cr3bp]") {
    const double mu = 0.01215;
    const double c0 = orbit::jacobi_constant(mu, nrho_seed);
    const auto later = orbit::cr3bp_propagate(mu, nrho_seed, 1.5);
    CHECK(orbit::jacobi_constant(mu, later) == Catch::Approx(c0).epsilon(1e-10));
}

TEST_CASE("CR3BP corrects the 9:2 NRHO seed to the resonant period", "[orbit][cr3bp][validation]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    const orbit::EarthMoonRotatingFrame frame{ephemeris, t0};
    const double mu = frame.mu();
    // 9 revolutions in 2 synodic months [S: Zimovan et al. 2017].
    const double period_s = 2.0 * constants::synodic_month_s / 9.0;
    // Mean time unit: sidereal month / 2 pi (the CR3BP's rotating rate), not the instantaneous one.
    const double period = period_s / (constants::sidereal_month_s / (2.0 * constants::pi));
    const auto orbit = orbit::correct_periodic_orbit(mu, nrho_seed, period);
    REQUIRE(orbit);
    const auto half = orbit::cr3bp_propagate(mu, *orbit, period / 2.0);
    CHECK(std::abs(half.position[1]) < 1e-9);
    CHECK(std::abs(half.velocity[0]) < 1e-9);
    CHECK(std::abs(half.velocity[2]) < 1e-9);
    // Perilune / apolune radii about the Moon (nondimensional -> km with the mean distance 384,400 km).
    double rp = 1e9, ra = 0.0;
    for (int i = 0; i <= 2000; ++i) {
        const auto s = orbit::cr3bp_propagate(mu, *orbit, period * i / 2000.0);
        const double r = norm(s.position - math::Vec3{1.0 - mu, 0.0, 0.0}) * 384400.0;
        rp = std::min(rp, r);
        ra = std::max(ra, r);
    }
    std::printf("[validation] CR3BP 9:2 NRHO: perilune %.0f km, apolune %.0f km, z0 %.5f\n", rp, ra, orbit->position[2]);
    // [S: Zimovan et al. 2017; Lee 2019 (NASA Gateway NRHO): perilune radius ~3,200-3,400 km, apolune ~70,000 km]
    CHECK(rp > 3000.0);
    CHECK(rp < 3700.0);
    CHECK(ra > 68000.0);
    CHECK(ra < 72500.0);
    CHECK(orbit->position[2] < 0.0);  // southern: apolune below the Earth-Moon plane
}

TEST_CASE("Earth-Moon rotating frame round trip", "[orbit][cr3bp]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    const orbit::EarthMoonRotatingFrame frame{ephemeris, t0};
    // The Moon itself maps to (1 - mu, 0, 0) at rest in the rotating frame (up to the eccentric radial rate).
    const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t0);
    const auto moon_rot = frame.from_gcrf({moon.position_m, moon.velocity_mps});
    CHECK(moon_rot.position[0] == Catch::Approx(1.0 - frame.mu()).margin(1e-12));
    CHECK(std::abs(moon_rot.position[1]) < 1e-12);
    CHECK(std::abs(moon_rot.position[2]) < 1e-12);
    const auto back = frame.to_gcrf(frame.from_gcrf({moon.position_m, moon.velocity_mps}));
    CHECK(norm(back.position_m - moon.position_m) < 1e-3);
    CHECK(norm(back.velocity_mps - moon.velocity_mps) < 1e-6);
}
```

If `constants::sidereal_month_s` or `constants::pi` do not exist, add `sidereal_month_s = 27.321661 * 86400.0`
[S: Meeus ch. 49] next to `synodic_month_s` and use the existing pi constant's name.

- [ ] **Step 3: Run** the build → FAIL (`cr3bp.hpp` not found).

- [ ] **Step 4: Implement** `core/src/orbit/cr3bp.cpp`:

```cpp
#include "starsim/orbit/cr3bp.hpp"

#include <cmath>

#include "starsim/integrators/dop853.hpp"

namespace starsim::orbit {

using math::Vec3;

integrators::State<6> cr3bp_derivative(double mu, const integrators::State<6>& y) {
    const double x = y[0], yy = y[1], z = y[2], vx = y[3], vy = y[4], vz = y[5];
    const double r1 = std::sqrt((x + mu) * (x + mu) + yy * yy + z * z);       // from the Earth
    const double r2 = std::sqrt((x - 1.0 + mu) * (x - 1.0 + mu) + yy * yy + z * z);  // from the Moon
    const double r1c = r1 * r1 * r1, r2c = r2 * r2 * r2;
    integrators::State<6> d;
    d[0] = vx; d[1] = vy; d[2] = vz;
    d[3] = 2.0 * vy + x - (1.0 - mu) * (x + mu) / r1c - mu * (x - 1.0 + mu) / r2c;
    d[4] = -2.0 * vx + yy - (1.0 - mu) * yy / r1c - mu * yy / r2c;
    d[5] = -(1.0 - mu) * z / r1c - mu * z / r2c;
    return d;
}

Cr3bpState cr3bp_propagate(double mu, const Cr3bpState& s, double duration) {
    integrators::State<6> y{s.position[0], s.position[1], s.position[2], s.velocity[0], s.velocity[1], s.velocity[2]};
    if (duration != 0.0) {
        integrators::Dop853Options<6> options;
        options.absolute_tolerance.fill(1e-13);
        options.relative_tolerance = 1e-13;
        y = integrators::integrate<6>([mu](double, const integrators::State<6>& v) { return cr3bp_derivative(mu, v); },
                                      0.0, y, duration, options).state;
    }
    return {{y[0], y[1], y[2]}, {y[3], y[4], y[5]}};
}

double jacobi_constant(double mu, const Cr3bpState& s) {
    const auto& p = s.position;
    const double r1 = norm(p - Vec3{-mu, 0.0, 0.0});
    const double r2 = norm(p - Vec3{1.0 - mu, 0.0, 0.0});
    return p[0] * p[0] + p[1] * p[1] + 2.0 * (1.0 - mu) / r1 + 2.0 * mu / r2 - dot(s.velocity, s.velocity);
}

std::optional<Cr3bpState> correct_periodic_orbit(double mu, const Cr3bpState& guess, double period) {
    // Unknowns u = (x0, z0, vy0); residual g = (y, vx, vz) at half the period. Newton, finite-difference Jacobian.
    auto residual = [&](const Vec3& u) {
        const auto h = cr3bp_propagate(mu, {{u[0], 0.0, u[1]}, {0.0, u[2], 0.0}}, period / 2.0);
        return Vec3{h.position[1], h.velocity[0], h.velocity[2]};
    };
    Vec3 u{guess.position[0], guess.position[2], guess.velocity[1]};
    constexpr double step = 1e-7;
    for (int it = 0; it < 30; ++it) {
        const Vec3 g = residual(u);
        if (norm(g) < 1e-11) return Cr3bpState{{u[0], 0.0, u[1]}, {0.0, u[2], 0.0}};
        std::array<Vec3, 3> column;
        for (int k = 0; k < 3; ++k) {
            Vec3 e{};
            e[k] = step;
            column[k] = (residual(u + e) - residual(u - e)) / (2.0 * step);
        }
        u -= math::Mat3::from_columns(column[0], column[1], column[2]).inverse() * g;
    }
    return std::nullopt;
}

EarthMoonRotatingFrame::EarthMoonRotatingFrame(const ephemeris::Ephemeris& ephemeris, time::Epoch epoch) {
    const double gm_earth = ephemeris.gm_m3ps2(ephemeris::Body::Earth);
    const double gm_moon = ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    mu_ = gm_moon / (gm_earth + gm_moon);
    const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, epoch);
    const Vec3 r = moon.position_m, v = moon.velocity_mps;
    length_m_ = norm(r);
    const Vec3 h = cross(r, v);
    omega_radps_ = norm(h) / (length_m_ * length_m_);
    radial_rate_ = dot(r, v) / length_m_;                // d(length)/dt (eccentric lunar orbit)
    x_ = r / length_m_;
    z_ = h / norm(h);
    y_ = cross(z_, x_);
    barycentre_m_ = mu_ * r;                              // barycentre relative to the Earth
}

double EarthMoonRotatingFrame::mu() const { return mu_; }
double EarthMoonRotatingFrame::time_unit_s() const { return 1.0 / omega_radps_; }
double EarthMoonRotatingFrame::length_unit_m() const { return length_m_; }

math::StateVector EarthMoonRotatingFrame::to_gcrf(const Cr3bpState& s) const {
    // Position: scaled pulsating frame. Velocity: d/dt of (L * R * p) = L R (p' * omega) + L omega x (R p) + L' R p.
    const Vec3 p = s.position[0] * x_ + s.position[1] * y_ + s.position[2] * z_;
    const Vec3 pdot = s.velocity[0] * x_ + s.velocity[1] * y_ + s.velocity[2] * z_;
    const Vec3 omega = omega_radps_ * z_;
    const Vec3 position = barycentre_m_ + length_m_ * p;
    const Vec3 velocity = mu_ * (radial_rate_ * x_ + length_m_ * cross(omega, x_)) +
                          length_m_ * omega_radps_ * pdot + length_m_ * cross(omega, p) + radial_rate_ * p;
    return {position, velocity};
}

Cr3bpState EarthMoonRotatingFrame::from_gcrf(const math::StateVector& gcrf) const {
    const Vec3 omega = omega_radps_ * z_;
    const Vec3 p = (gcrf.position_m - barycentre_m_) / length_m_;
    const Vec3 barycentre_velocity = mu_ * (radial_rate_ * x_ + length_m_ * cross(omega, x_));
    const Vec3 pdot = (gcrf.velocity_mps - barycentre_velocity - length_m_ * cross(omega, p) - radial_rate_ * p) /
                      (length_m_ * omega_radps_);
    return {{dot(p, x_), dot(p, y_), dot(p, z_)}, {dot(pdot, x_), dot(pdot, y_), dot(pdot, z_)}};
}

}  // namespace starsim::orbit
```

Header: the declarations of Interfaces plus the private members `double mu_, length_m_, omega_radps_, radial_rate_;
Vec3 x_, y_, z_, barycentre_m_;`. Use the integrator call exactly as `orbit/ballistic_propagator.cpp` does (copy its
`integrate<6>` call form and options type; the names above are the expected ones - adapt to the real signature).
`from_gcrf` uses the same velocity decomposition as `to_gcrf` (the round-trip test checks both). Note in a comment:
the barycentre moves with the Moon (`mu * r`); the frame's `x` is along the Earth -> Moon line.

- [ ] **Step 5: Run** `"[cr3bp]"` → PASS. Record the printed perilune / apolune in the ledger (end of this plan).
If the seed does not converge, the sign convention of z or vy differs from the source: check the source table
(southern family: apolune z < 0) - do not loosen the residual.
- [ ] **Step 6: Commit** `feat(orbit): CR3BP dynamics, fixed-period periodic orbits, Earth-Moon rotating frame`; push.

---

### Task 3: NRHO reference by multiple shooting in the full model

**Files:**
- Create: `core/include/starsim/mission/nrho_reference.hpp`, `core/src/mission/nrho_reference.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`
- Test: `core/tests/test_nrho_reference.cpp`

**Interfaces:**
- Consumes: Task 2 (`correct_periodic_orbit`, `cr3bp_propagate`, `EarthMoonRotatingFrame`), `orbit::propagate_coast`.
- Produces:
  ```cpp
  namespace starsim::mission {
  struct NrhoSeed {
      orbit::Cr3bpState apolune;   // CR3BP state at apolune (nondimensional) [S]
      int revolutions = 9;         // resonance: revolutions ...
      int synodic_months = 2;      // ... per synodic months [S]
  };
  struct NrhoReferenceOptions {
      int revolutions = 6;               // span of the reference
      double continuity_tolerance_m = 1.0;  // position jump allowed at a patch point
      double continuity_tolerance_mps = 1e-4;
      int max_iterations = 20;
  };
  class NrhoReference {
  public:
      // Builds the reference whose first perilune falls at `first_perilune`: CR3BP periodic orbit (fixed resonant
      // period), patch points at every apolune and perilune converted to GCRF at their epochs, then minimum-norm
      // Newton on the patch states (epochs fixed) until the full-model coast is continuous. Throws
      // std::runtime_error (German) when the CR3BP correction or the shooting does not converge.
      NrhoReference(const environment::GravityModel& earth_centred_gravity, const ephemeris::Ephemeris& ephemeris,
                    const NrhoSeed& seed, time::Epoch first_perilune, NrhoReferenceOptions options = {});
      bool covers(time::Epoch epoch) const;                // within [first patch, last patch]
      math::StateVector state_gcrf(time::Epoch epoch) const;   // Earth-centred; throws std::out_of_range outside
      const std::vector<time::Epoch>& perilune_epochs() const;
      const std::vector<time::Epoch>& apolune_epochs() const;
      double max_continuity_error_m() const;
      int iterations() const;
  };
  }
  ```

- [ ] **Step 1: Write the failing test** `core/tests/test_nrho_reference.cpp`:

```cpp
// NRHO reference in the full force model (DE440 Earth, Moon, Sun + Earth J2) by multiple shooting from the CR3BP 9:2
// southern L2 NRHO: continuity, period, perilune / apolune radii and their positions over the poles vs the literature.

#include <cstdio>

#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/gravity.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
mission::NrhoSeed seed() {
    // [S: Zimovan, Howell & Davis 2017, Table 1] 9:2 southern L2 NRHO at apolune
    return {{{1.0221, 0.0, -0.1821}, {0.0, -0.1033, 0.0}}, 9, 2};
}
}  // namespace

TEST_CASE("NRHO reference by multiple shooting in the full force model", "[mission][nrho][validation]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    const auto first_perilune = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    const mission::NrhoReference nrho{gravity, ephemeris, seed(), first_perilune};

    std::printf("[validation] NRHO shooting: %d iterations, continuity %.3f m\n", nrho.iterations(),
                nrho.max_continuity_error_m());
    CHECK(nrho.max_continuity_error_m() < 1.0);
    REQUIRE(nrho.perilune_epochs().size() >= 6);
    CHECK(nrho.perilune_epochs().front() == first_perilune);

    auto moon_relative = [&](time::Epoch t) {
        const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t);
        return nrho.state_gcrf(t).position_m - moon.position_m;
    };
    for (size_t i = 0; i + 1 < nrho.perilune_epochs().size(); ++i) {
        const auto tp = nrho.perilune_epochs()[i];
        const auto ta = nrho.apolune_epochs()[i];
        const double period_d = (nrho.perilune_epochs()[i + 1] - tp) / 86400.0;
        const double rp_km = norm(moon_relative(tp)) / 1000.0;
        const double ra_km = norm(moon_relative(ta)) / 1000.0;
        // pole: MOON_ME z in GCRF
        const auto pole_m = moon_me.fixed_to_gcrf(tp).rotation;
        const math::Vec3 pole{pole_m(0, 2), pole_m(1, 2), pole_m(2, 2)};
        const double peri_lat_deg = std::asin(dot(normalize(moon_relative(tp)), pole)) / constants::deg_to_rad;
        const double apo_lat_deg = std::asin(dot(normalize(moon_relative(ta)), pole)) / constants::deg_to_rad;
        std::printf("[validation] NRHO rev %zu: period %.3f d, perilune %.0f km (lat %+.1f), apolune %.0f km (lat %+.1f)\n",
                    i, period_d, rp_km, peri_lat_deg, ra_km, apo_lat_deg);
        // [S: Lee 2019 / Zimovan et al. 2017: ~6.56 d, rp ~3,200-3,400 km, ra ~70,000 km; perilune north, apolune south]
        CHECK(period_d > 6.3);
        CHECK(period_d < 6.8);
        CHECK(rp_km > 3000.0);
        CHECK(rp_km < 3800.0);
        CHECK(ra_km > 66000.0);
        CHECK(ra_km < 73000.0);
        CHECK(peri_lat_deg > 60.0);
        CHECK(apo_lat_deg < -60.0);
    }
}

TEST_CASE("NRHO reference refuses epochs outside its span", "[mission][nrho]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    const auto first_perilune = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    mission::NrhoReferenceOptions options;
    options.revolutions = 2;
    const mission::NrhoReference nrho{gravity, ephemeris, seed(), first_perilune, options};
    CHECK(nrho.covers(first_perilune));
    CHECK_FALSE(nrho.covers(first_perilune + 30.0 * 86400.0));
    CHECK_THROWS_AS(nrho.state_gcrf(first_perilune + 30.0 * 86400.0), std::out_of_range);
}
```

- [ ] **Step 2: Run** the build → FAIL (header missing).

- [ ] **Step 3: Implement.** Algorithm (write it in `nrho_reference.cpp`, one function per step):
  1. `mu` and units from `EarthMoonRotatingFrame(ephemeris, first_perilune)`; resonant period
     `P_s = seed.synodic_months * constants::synodic_month_s / seed.revolutions`; nondimensional
     `P = P_s / (constants::sidereal_month_s / (2 pi))`; `correct_periodic_orbit(mu, seed.apolune, P)` (empty ->
     throw `std::runtime_error("NRHO: keine periodische Bahn im Dreikörperproblem gefunden")`).
  2. Patch points: the corrected orbit starts at apolune; its perilune is at `P/2`. For revolution `k = 0..revolutions`
     the perilune epoch is `first_perilune + k * P_s` and the apolune epochs lie half a period after each perilune;
     also one apolune half a period *before* the first perilune (so the reference covers the arrival). Patch states:
     the CR3BP state at the matching phase (`cr3bp_propagate(mu, orbit, P/2)` for perilune, the orbit itself for
     apolune) converted with an `EarthMoonRotatingFrame` built **at that patch epoch** (instantaneous units).
  3. Multiple shooting: unknown vector X = all patch states (6 each, epochs fixed). Constraint F = for each segment j:
     `coast(X_j, t_j, t_{j+1}) - X_{j+1}` (6 each). Jacobian blocks: `dF_j/dX_j` = state transition matrix by central
     differences (step 1 m in position, 1 mm/s in velocity), `dF_j/dX_{j+1}` = -I. Minimum-norm Newton step
     `dX = -J^T (J J^T)^-1 F` (J is 6(N-1) x 6N, banded; form `J J^T` densely - N <= 15 patch points gives <= 84 x 84;
     solve with the dense Cholesky / LU helper already in `core/include/starsim/math/` - if none exists, write a small
     LU with partial pivoting in an anonymous namespace of this file). Stop when every segment's position jump
     < `continuity_tolerance_m` and velocity jump < `continuity_tolerance_mps`; throw
     `std::runtime_error("NRHO: Mehrfachschießverfahren konvergiert nicht")` after `max_iterations`.
  4. Storage: patch epochs and states. `state_gcrf(t)`: throw `std::out_of_range` if `!covers(t)`; else coast from the
     last patch at or before `t`. `perilune_epochs()` / `apolune_epochs()`: the patch epochs of each kind, in order,
     apolunes only those after the first perilune (index i is the apolune following perilune i).
  5. Coasts with `orbit::propagate_coast(gravity, ...)` (default tolerances).

  Note in the header comment: the reference's phase is chosen by `first_perilune` (no Gateway phasing; a real Gateway
  ephemeris would replace the seed when rendezvous comes with M9).

- [ ] **Step 4: Run** `"[nrho]"` → PASS. Record the printed lines (iterations, continuity, per-revolution period /
radii / latitudes) in the ledger. If the perilune latitude comes out south, the seed's z sign or the frame's z axis is
flipped - check against the source, do not change the test.
- [ ] **Step 5: Commit** `feat(mission): NRHO reference by multiple shooting in the full force model`; push.

---

### Task 4: Outbound planning (TLI, MCC, LOI / NRHO insertion, LLO trims, propellant check)

**Files:**
- Create: `core/include/starsim/mission/lunar_outbound.hpp`, `core/src/mission/lunar_outbound.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`
- Test: `core/tests/test_lunar_outbound.cpp`

**Interfaces:**
- Consumes: `LunarTransferTargeter` (`best_departure`, `solve`, `evaluate`, `orbit_insertion`), Task 1
  `PositionTargeter`, Task 3 `NrhoReference`.
- Produces:
  ```cpp
  namespace starsim::mission {
  enum class LunarOrbitKind { Llo, Nrho };
  struct LloTarget { double altitude_m = 100e3; double inclination_rad = 0.5 * constants::pi; };
  struct LunarOutboundParameters {
      LunarOrbitKind orbit = LunarOrbitKind::Llo;
      LloTarget llo;
      std::shared_ptr<const NrhoReference> nrho;   // required for Nrho
      time::Epoch arrival_epoch;                   // planned periselene (LLO) / the NRHO perilune to meet
      double earliest_departure_s = 1800.0;        // after the plan's epoch
      std::vector<double> mcc_after_tli_s{86400.0, 2.5 * 86400.0};   // [E] spec: ~1 and ~2.5 days
      double mcc_threshold_mps = 0.05;             // [E] smaller corrections are skipped
      double llo_trim_tolerance_m = 200.0;         // [E] apsis errors below are not trimmed
      double reserve_fraction = 0.05;              // [E] propellant margin on top of the planned burns
  };
  struct OutboundPlan {
      bool feasible = false;
      std::string reason;                          // German when refused
      gnc::BurnNode tli;
      gnc::BurnNode insertion;                     // predicted (re-planned at arrival)
      double total_delta_v_mps = 0.0;
      double propellant_kg = 0.0;                  // rocket equation on the vacuum Raptors
  };
  class LunarOutboundPlanner {
  public:
      LunarOutboundPlanner(const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                           const frames::BodyOrientation& moon_orientation);
      // TLI from the parking-orbit state; checks the propellant (mass, Isp, propellant on board).
      OutboundPlan plan(const math::StateVector& state, time::Epoch epoch, const LunarOutboundParameters& p,
                        double mass_kg, double isp_s, double propellant_kg) const;
      // Mid-course correction at `burn_epoch` (zero first guess); empty if below the threshold or not converged.
      std::optional<gnc::BurnNode> mid_course(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                              const LunarOutboundParameters& p) const;
      // Insertion: LLO - circular at the predicted periselene; NRHO - reference velocity minus the arrival velocity at
      // the arrival epoch.
      std::optional<gnc::BurnNode> insertion(const math::StateVector& state, time::Epoch epoch,
                                             const LunarOutboundParameters& p) const;
      // LLO trim at the next apsis: sets the opposite apsis to the target radius (Moon two-body); empty when both
      // apsides are within the tolerance.
      std::optional<gnc::BurnNode> llo_trim(const math::StateVector& state, time::Epoch epoch,
                                            const LunarOutboundParameters& p) const;
  };
  // Moon-relative state (GCRF axes) of an Earth-centred state.
  math::StateVector moon_relative(const ephemeris::Ephemeris& ephemeris, const math::StateVector& state, time::Epoch t);
  }
  ```

- [ ] **Step 1: Write the failing test** `core/tests/test_lunar_outbound.cpp` (impulsive planning only; the session
flies it in Task 6):

```cpp
// Outbound planning: TLI and insertion delta-v for the LLO and the NRHO vs the literature, MCC re-targeting after an
// execution error, LLO trims, and the propellant check.

#include <cstdio>

#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/gravity.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/mission/lunar_transfer.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
constexpr double deg = constants::deg_to_rad;
struct Fixture {
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    mission::LunarTransferTargeter targeter{gravity, ephemeris, moon};
    mission::LunarOutboundPlanner planner{gravity, ephemeris, moon};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    math::StateVector parking(time::Epoch arrival) const {
        return orbit::state_from_elements({6378137.0 + 200e3, 0.0, 28.5 * deg,
                                           targeter.parking_orbit_raan(arrival, 28.5 * deg).value(), 0.0, 0.0},
                                          ephemeris.gm_m3ps2(ephemeris::Body::Earth));
    }
};
constexpr double hls_mass_kg = 1720e3;      // 120 t dry + 1,600 t
constexpr double rvac_isp_s = 380.0;        // [S] raptor_vacuum3.toml
}  // namespace

TEST_CASE("Outbound plan to the 100 km polar LLO", "[mission][lunar][outbound][validation]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    const auto plan = fx.planner.plan(fx.parking(p.arrival_epoch), fx.t0, p, hls_mass_kg, rvac_isp_s, 1600e3);
    REQUIRE(plan.feasible);
    const double tli = norm(plan.tli.delta_v_gcrf_mps), loi = norm(plan.insertion.delta_v_gcrf_mps);
    std::printf("[validation] outbound LLO: TLI %.0f m/s, LOI %.0f m/s, %.0f t propellant\n", tli, loi,
                plan.propellant_kg / 1000.0);
    CHECK(tli > 3050.0);  // [S] spec: TLI ~3.1-3.2 km/s
    CHECK(tli < 3250.0);
    CHECK(loi > 800.0);   // [S] spec: LOI ~0.82-0.9 km/s
    CHECK(loi < 920.0);
}

TEST_CASE("Outbound plan to the NRHO", "[mission][lunar][outbound][nrho][validation]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.orbit = mission::LunarOrbitKind::Nrho;
    p.arrival_epoch = fx.t0 + 4.5 * 86400.0;
    p.nrho = std::make_shared<mission::NrhoReference>(
        fx.gravity, fx.ephemeris, mission::NrhoSeed{{{1.0221, 0.0, -0.1821}, {0.0, -0.1033, 0.0}}, 9, 2},
        p.arrival_epoch);
    const auto plan = fx.planner.plan(fx.parking(p.arrival_epoch), fx.t0, p, hls_mass_kg, rvac_isp_s, 1600e3);
    REQUIRE(plan.feasible);
    const double tli = norm(plan.tli.delta_v_gcrf_mps), nri = norm(plan.insertion.delta_v_gcrf_mps);
    std::printf("[validation] outbound NRHO: TLI %.0f m/s, insertion %.0f m/s\n", tli, nri);
    CHECK(tli > 3050.0);
    CHECK(tli < 3250.0);
    CHECK(nri > 300.0);   // [S] spec: direct transfer, insertion at perilune ~0.4-0.45 km/s (band widened: record)
    CHECK(nri < 550.0);
}

TEST_CASE("A mid-course correction removes a TLI execution error", "[mission][lunar][outbound]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    const auto parking = fx.parking(p.arrival_epoch);
    const auto plan = fx.planner.plan(parking, fx.t0, p, hls_mass_kg, rvac_isp_s, 1600e3);
    REQUIRE(plan.feasible);
    const mission::PositionTargeter coast{fx.gravity};
    auto after = coast.coast(parking, fx.t0, plan.tli.epoch);
    after.velocity_mps += 1.002 * plan.tli.delta_v_gcrf_mps;   // 0.2 % overburn (~6 m/s)
    const auto t_mcc = plan.tli.epoch + 86400.0;
    const auto mcc = fx.planner.mid_course(after, plan.tli.epoch, t_mcc, p);
    REQUIRE(mcc);
    auto corrected = coast.coast(after, plan.tli.epoch, t_mcc);
    corrected.velocity_mps += mcc->delta_v_gcrf_mps;
    const auto arrival = fx.targeter.evaluate(corrected, t_mcc, t_mcc, {});
    std::printf("MCC %.2f m/s -> periselene %.3f km, i %.3f deg\n", norm(mcc->delta_v_gcrf_mps),
                arrival.periselene_altitude_m / 1000.0, arrival.inclination_rad / deg);
    CHECK(std::abs(arrival.periselene_altitude_m - 100e3) < 500.0);
    CHECK(std::abs(arrival.inclination_rad - 90.0 * deg) < 0.05 * deg);
    // On the planned trajectory itself no correction is needed.
    auto nominal = coast.coast(parking, fx.t0, plan.tli.epoch);
    nominal.velocity_mps += plan.tli.delta_v_gcrf_mps;
    CHECK_FALSE(fx.planner.mid_course(nominal, plan.tli.epoch, t_mcc, p));
}

TEST_CASE("Outbound plan is refused without the propellant", "[mission][lunar][outbound]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    // Half tanks: 800 t cannot fly 3.1 + 0.85 km/s with 120 t dry.
    const auto plan = fx.planner.plan(fx.parking(p.arrival_epoch), fx.t0, p, 920e3, rvac_isp_s, 800e3);
    CHECK_FALSE(plan.feasible);
    CHECK(plan.reason.find("Treibstoff") != std::string::npos);
}

TEST_CASE("LLO trim sets the opposite apsis", "[mission][lunar][outbound]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    const double gm_moon = fx.ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    const double r_moon = constants::moon_mean_radius_m;
    // 95 x 104 km, at periselene: the trim at periselene raises/lowers the aposelene to 100 km.
    const double rp = r_moon + 95e3, ra = r_moon + 104e3;
    const auto rel = orbit::state_from_elements({0.5 * (rp + ra), (ra - rp) / (ra + rp), 90.0 * deg, 0.3, 0.0, 0.0}, gm_moon);
    const auto moon = fx.ephemeris.state_gcrf(ephemeris::Body::Moon, fx.t0);
    const math::StateVector state{rel.position_m + moon.position_m, rel.velocity_mps + moon.velocity_mps};
    const auto trim = fx.planner.llo_trim(state, fx.t0, p);
    REQUIRE(trim);
    CHECK(trim->epoch - fx.t0 < 60.0);   // at this periselene (or within one minute of it)
    CHECK(norm(trim->delta_v_gcrf_mps) < 5.0);
}
```

- [ ] **Step 2: Run** the build → FAIL (header missing).

- [ ] **Step 3: Implement** `lunar_outbound.cpp`:
  - `moon_relative`: subtract `ephemeris.state_gcrf(Moon, t)`.
  - `plan`:
    - LLO: `LunarArrivalTarget{p.llo.altitude_m, p.llo.inclination_rad, p.arrival_epoch}`;
      `burn_epoch = targeter.best_departure(state, epoch, epoch + p.earliest_departure_s, p.arrival_epoch)`;
      `solve(...)`; not converged -> refuse `"Kein Transfer zum Mond gefunden"`. Insertion =
      `LunarTransferTargeter::orbit_insertion(solution.arrival, gm_moon)`.
    - NRHO: `rp = nrho->state_gcrf(arrival)` Moon-relative; first a `LunarTransferTargeter::solve` with
      `periselene_altitude_m = |rp| - R_moon`, `inclination_rad` = the inclination of the reference's Moon-relative
      angular momentum at the arrival to the MOON_ME pole, `periselene_epoch = arrival`; then
      `PositionTargeter::solve(state, epoch, burn_epoch, {nrho->state_gcrf(arrival).position_m, arrival},
      first.burn.delta_v_gcrf_mps, 1000.0)` (1 km [E]: the MCCs refine it). Insertion =
      `{arrival, nrho->state_gcrf(arrival).velocity_mps - solution.arrival_gcrf.velocity_mps}`.
      A reference that does not cover the arrival -> refuse `"NRHO-Referenz deckt die Ankunft nicht ab"`.
    - Propellant: `total = |tli| + |insertion|`; needed = `mass * (1 - exp(-total / (isp * g0)))` (g0 from constants);
      if `needed * (1 + reserve_fraction) > propellant_kg` refuse with
      `"Zu wenig Treibstoff für den Hinflug: %.0f t nötig, %.0f t an Bord"` (use `std::snprintf` into a buffer, t =
      tonnes). Fill `total_delta_v_mps`, `propellant_kg`.
  - `mid_course`: LLO -> `targeter.solve(state, epoch, burn_epoch, target, Vec3{})`; NRHO -> `PositionTargeter::solve(...,
    {reference perilune position, arrival}, Vec3{}, 100.0)` [E 100 m]. Not converged or `|dv| < mcc_threshold_mps`
    -> empty.
  - `insertion`: LLO -> `evaluate(state, epoch, epoch, {})` then `orbit_insertion`; NRHO -> coast to `arrival_epoch`,
    `{arrival_epoch, v_ref - v}`.
  - `llo_trim`: Moon-relative elements (`orbit::elements_from_state`, GM Moon); target radius `R + altitude`; if both
    `|rp - target| < tol` and `|ra - target| < tol` -> empty. Else the next apsis (time from the mean anomaly; use
    `orbit::propagate_two_body` to get the apsis state); at that apsis with radius `r`, the speed for an orbit with apsides
    `r` and `target` is `sqrt(gm * 2 * target / (r * (r + target)))`; delta-v along the Moon-relative velocity
    direction = that speed - the current speed; epoch = the apsis epoch. Comment: two-body about the Moon; the
    closed-loop burn absorbs the Earth's and Sun's pull.

- [ ] **Step 4: Run** `"[outbound]"` → PASS. Record the printed delta-v lines in the ledger. If the NRHO insertion falls
outside 300-550 m/s, record the value and the literature band and **ask the user** before changing anything (do not
widen the band unasked).
- [ ] **Step 5: Commit** `feat(mission): outbound planning to the LLO and the NRHO`; push.

---

### Task 5: Lunar orbit transfers (NRHO <-> LLO) and NRHO station keeping

**Files:**
- Create: `core/include/starsim/mission/lunar_orbit_transfer.hpp`, `core/src/mission/lunar_orbit_transfer.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`
- Test: `core/tests/test_lunar_orbit_transfer.cpp`

**Interfaces:**
- Consumes: Tasks 1, 3, 4 (`moon_relative`, `LloTarget`), `LunarTransferTargeter`, `orbit::solve_lambert`.
- Produces:
  ```cpp
  namespace starsim::mission {
  struct TwoBurnTransfer {
      bool feasible = false;
      std::string reason;          // German when refused
      gnc::BurnNode first;         // departure
      gnc::BurnNode second;        // predicted arrival burn (re-planned on arrival)
      double total_delta_v_mps = 0.0;
  };
  class LunarOrbitTransfer {
  public:
      LunarOrbitTransfer(const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                         const frames::BodyOrientation& moon_orientation);
      // NRHO -> LLO: first burn at the next perilune of the reference after `epoch` (retrograde: transfer ellipse down to
      // the LLO altitude on the far side, targeted on periselene altitude and inclination), second = circularisation.
      TwoBurnTransfer nrho_to_llo(const math::StateVector& state, time::Epoch epoch, const NrhoReference& nrho,
                                  const LloTarget& llo) const;
      // LLO -> NRHO: for each reference perilune within `search_s`, the departure on the LLO (time of flight scanned on
      // two-body Lambert about the Moon) with the least total delta-v, refined by the position targeter; the first
      // perilune whose total is <= max_total_mps is taken, else the cheapest one found.
      TwoBurnTransfer llo_to_nrho(const math::StateVector& state, time::Epoch epoch, const NrhoReference& nrho,
                                  double search_s = 14.0 * 86400.0, double max_total_mps = 1000.0) const;
      // Station keeping in the NRHO at apolune: aims the next perilune of the reference (position targeter); empty when
      // the predicted perilune miss is below `deadband_m` or the reference does not cover the next perilune.
      std::optional<gnc::BurnNode> station_keeping(const math::StateVector& state, time::Epoch epoch,
                                                   const NrhoReference& nrho, double deadband_m = 1000.0) const;
      // Second burn of a transfer flown so far: LLO circularisation at the predicted periselene, or the NRHO velocity
      // match at the targeted perilune.
      std::optional<gnc::BurnNode> arrival_burn(const math::StateVector& state, time::Epoch epoch,
                                                const TwoBurnTransfer& transfer, LunarOrbitKind target,
                                                const NrhoReference* nrho, const LloTarget& llo) const;
  };
  }
  ```

- [ ] **Step 1: Write the failing test** `core/tests/test_lunar_orbit_transfer.cpp`:

```cpp
// NRHO <-> 100 km polar LLO two-burn transfers vs the literature (~0.75-0.9 km/s each way) and NRHO station keeping.

#include <cstdio>

#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/gravity.hpp"
#include "starsim/mission/lunar_orbit_transfer.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/mission/position_targeter.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
struct Fixture {
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    time::Epoch first_perilune = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    mission::NrhoReference nrho{gravity, ephemeris,
                                mission::NrhoSeed{{{1.0221, 0.0, -0.1821}, {0.0, -0.1033, 0.0}}, 9, 2},
                                first_perilune, {.revolutions = 6}};
    mission::LunarOrbitTransfer transfer{gravity, ephemeris, moon};
    mission::PositionTargeter coast{gravity};
};
}  // namespace

TEST_CASE("NRHO to LLO and back: delta-v vs the literature", "[mission][lunar][transfer][validation]") {
    test::spice_session();
    Fixture fx;
    const auto start = fx.first_perilune + 2.0 * 86400.0;
    const auto down = fx.transfer.nrho_to_llo(fx.nrho.state_gcrf(start), start, fx.nrho, {});
    REQUIRE(down.feasible);
    std::printf("[validation] NRHO -> LLO: %.0f + %.0f = %.0f m/s\n", norm(down.first.delta_v_gcrf_mps),
                norm(down.second.delta_v_gcrf_mps), down.total_delta_v_mps);
    CHECK(down.total_delta_v_mps > 700.0);   // [S] spec: ~0.75-0.9 km/s each way
    CHECK(down.total_delta_v_mps < 950.0);

    // Fly it impulsively and go back up from the LLO.
    auto s = fx.coast.coast(fx.nrho.state_gcrf(start), start, down.first.epoch);
    s.velocity_mps += down.first.delta_v_gcrf_mps;
    s = fx.coast.coast(s, down.first.epoch, down.second.epoch);
    s.velocity_mps += down.second.delta_v_gcrf_mps;
    const auto up = fx.transfer.llo_to_nrho(s, down.second.epoch, fx.nrho);
    REQUIRE(up.feasible);
    std::printf("[validation] LLO -> NRHO: %.0f + %.0f = %.0f m/s, departure %.1f d after arrival\n",
                norm(up.first.delta_v_gcrf_mps), norm(up.second.delta_v_gcrf_mps), up.total_delta_v_mps,
                (up.first.epoch - down.second.epoch) / 86400.0);
    CHECK(up.total_delta_v_mps > 700.0);
    CHECK(up.total_delta_v_mps < 1000.0);
}

TEST_CASE("NRHO station keeping holds the reference for three revolutions", "[mission][lunar][nrho][validation]") {
    test::spice_session();
    Fixture fx;
    // Insertion error: 0.5 m/s at the first perilune.
    auto s = fx.nrho.state_gcrf(fx.first_perilune);
    s.velocity_mps += math::Vec3{0.3, -0.3, 0.2};
    time::Epoch t = fx.first_perilune;
    double spent = 0.0, worst_km = 0.0;
    for (int rev = 0; rev < 3; ++rev) {
        const auto apolune = fx.nrho.apolune_epochs()[rev];
        s = fx.coast.coast(s, t, apolune);
        t = apolune;
        if (const auto burn = fx.transfer.station_keeping(s, t, fx.nrho)) {
            s = fx.coast.coast(s, t, burn->epoch);
            t = burn->epoch;
            s.velocity_mps += burn->delta_v_gcrf_mps;
            spent += norm(burn->delta_v_gcrf_mps);
        }
        const auto perilune = fx.nrho.perilune_epochs()[rev + 1];
        s = fx.coast.coast(s, t, perilune);
        t = perilune;
        worst_km = std::max(worst_km, norm(s.position_m - fx.nrho.state_gcrf(t).position_m) / 1000.0);
    }
    std::printf("[validation] NRHO station keeping: %.2f m/s over 3 revolutions, worst perilune offset %.2f km\n",
                spent, worst_km);
    CHECK(worst_km < 5.0);   // [E]
    CHECK(spent < 10.0);     // [E]
}
```

- [ ] **Step 2: Run** the build → FAIL.

- [ ] **Step 3: Implement.**
  - `nrho_to_llo`: next perilune `tp` of `nrho.perilune_epochs()` after `epoch + 600 s`; guess = retrograde along the
    Moon-relative velocity with magnitude `v - sqrt(gm * 2 * r_llo / (r * (r + r_llo)))` (Hohmann-like to the LLO
    radius on the far side); `LunarTransferTargeter::solve(state, epoch, tp, {llo.altitude_m, llo.inclination_rad, tp +
    half transfer period}, guess)`; not converged -> refuse `"Kein Abstieg von der NRHO in die LLO gefunden"`. Second =
    `orbit_insertion(solution.arrival, gm_moon)`. Comment that `evaluate`'s "periselene" event is the first closest
    approach after the burn (the burn sits at the transfer's aposelene, radial velocity rises through zero at the far
    side).
  - `llo_to_nrho`: for each perilune `tp` of the reference with `epoch + 1800 s < tp < epoch + search_s`: scan
    departure epochs `td = tp - tof`, `tof` from 1.0 h to 4.0 h in 60 s steps, `td >= epoch + 600 s`; two-body Moon
    state at `td` = `orbit::propagate_two_body(moon_relative(state at epoch), gm_moon, td - epoch)` (adequate for the
    scan), target = Moon-relative reference position at `tp` with the Moon's position at `tp`... use
    `nrho.state_gcrf(tp) - moon(tp)` as r2 and the Moon-relative scan state's position as r1 (Moon-relative Lambert is
    consistent: both relative to the Moon's centre at their own epochs); `solve_lambert(r1, r2, tof, gm_moon, h_llo)`;
    total = |v1 - v_llo| + |v_ref_rel(tp) - v2|. Keep the minimum per perilune; refine the best with
    `PositionTargeter::solve(state, epoch, td, {nrho.state_gcrf(tp).position_m, tp}, lambert dv1, 100.0)`; second =
    `nrho velocity - arrival velocity` at `tp`. Take the first perilune whose refined total `<= max_total_mps`, else the
    cheapest refined one; none converged -> refuse `"Kein Aufstieg von der LLO in die NRHO gefunden"`.
  - `station_keeping`: the next perilune after `epoch` must be covered (else empty); predicted miss =
    `|coast(state, epoch, tp).position - nrho.state_gcrf(tp).position|`; below `deadband_m` -> empty; else
    `PositionTargeter::solve(state, epoch, epoch + 60 s, {ref position, tp}, {}, 100.0)` (converged -> burn, else empty).
  - `arrival_burn`: LLO -> `evaluate` + `orbit_insertion`; NRHO -> coast to the transfer's `second.epoch`, velocity
    match.

- [ ] **Step 4: Run** `"[transfer]"` and `"[nrho]"` → PASS; record the printed lines in the ledger. LLO -> NRHO above
1000 m/s means the LLO plane does not reach the perilune cheaply within 14 days: record the numbers and ask the user
(the spec's band assumes an aligned plane).
- [ ] **Step 5: Commit** `feat(mission): NRHO <-> LLO transfers and NRHO station keeping`; push.

---

### Task 6: Outbound autopilot in the session

**Files:**
- Create: `core/src/sim/flight_session_lunar.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`, `core/CMakeLists.txt`,
  `core/tests/CMakeLists.txt`
- Test: `core/tests/test_lunar_outbound_session.cpp`

**Interfaces:**
- Consumes: Tasks 3-5; `execute_burn(node, groups)`, `execute_burn(node)`, `set_attitude_mode`.
- Produces (in `flight_session.hpp`):
  ```cpp
  enum class AutopilotState { Off, PreLaunch, Booster, UpperStage, PayloadSeparation, Orbit, Burn, Return, Lunar };
  // Steps of a lunar flight (plan_lunar_outbound / plan_lunar_orbit_transfer).
  enum class LunarStep {
      None, Departure, Tli, Coast, MidCourse, PilotCorrection, Insertion, Trim, InLlo, InNrho, StationKeeping,
    TransferOut, TransferIn   // PilotCorrection: Step 3b
  };
  const char* to_string(LunarStep step);   // German: "aus", "Abflug", "TLI", "Freiflug", "Kurskorrektur",
                                           // "Einschuss", "Bahnkorrektur", "in LLO", "in NRHO", "Bahnhaltung",
                                           // "Transfer", "Ankunft"
  // FlightSession:
  bool plan_lunar_outbound(const mission::LunarOutboundParameters& parameters);
  bool plan_lunar_orbit_transfer(mission::LunarOrbitKind target, std::shared_ptr<const mission::NrhoReference> nrho,
                                 mission::LloTarget llo = {});
  LunarStep lunar_step() const;
  const mission::OutboundPlan* lunar_outbound_plan() const;   // also when refused (reason)
  std::string lunar_status() const;   // German status line, e.g. reason of the last refusal or "NRHO-Referenz endet"
  static constexpr double sun_repoint_interval_s = 6.0 * 3600.0;  // [E] coast: Sun pointing re-aimed this often
  ```

Design decisions (put them as comments in the code):
- The HLS coasts Sun-pointed: at the start of each coast it turns to `AttitudeModeKind::Sun` (ticked until the error
  < 1 deg and the rate < 0.05 deg/s), then coasts freely; every `sun_repoint_interval_s` it re-aims (the Sun moves
  ~1 deg/day; no disturbance torques are modelled, so a held attitude stays put). Ticking a 4-day coast at 200 Hz would
  cost ~70 M ticks.
- Burns: TLI, insertion and transfer burns on `{"vacuum"}`; MCC, trims and station keeping by `execute_burn(node)`
  (the session picks RCS for small ones).
- After each burn the state machine returns to `AutopilotState::Lunar` (as `ship_return_` returns to `Return`).
- The coast allowance stops at the next step epoch (`lunar_step_epoch_` - ignition preparation comes from `Burn`).
- `disengage_autopilot()` resets `lunar_step_` to `None`.

- [ ] **Step 1: Write the failing test** `core/tests/test_lunar_outbound_session.cpp`:

```cpp
// The HLS flies the outbound mission on its autopilot: LLO (< 1 km after the trims), NRHO (three revolutions near the
// reference), NRHO -> LLO; time warp, pilot override, refusal and double planning.

#include <cstdio>

#include <catch2/catch_test_macros.hpp>

#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/mission/lunar_transfer.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
constexpr double deg = constants::deg_to_rad;
struct Fixture {
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    mission::LunarTransferTargeter targeter{gravity, ephemeris, moon};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    void place(time::Epoch arrival) {
        const auto parking = orbit::state_from_elements(
            {6378137.0 + 200e3, 0.0, 28.5 * deg, targeter.parking_orbit_raan(arrival, 28.5 * deg).value(), 0.0, 0.0},
            ephemeris.gm_m3ps2(ephemeris::Body::Earth));
        session.place_in_orbit("starship_hls", parking, t0);
    }
    math::StateVector moon_relative_state() {
        const auto& s = session.body(session.controlled_index()).state();
        return mission::moon_relative(ephemeris, {s.position_gcrf_m, s.velocity_gcrf_mps}, session.epoch());
    }
    // Apsis altitudes over the next `revolutions` LLO revolutions (sampled every 20 s).
    std::pair<double, double> llo_range(int revolutions) {
        double low = 1e12, high = -1e12;
        for (int i = 0; i < revolutions * 355; ++i) {
            session.advance_to(session.epoch() + 20.0, 100000);
            const double h = norm(moon_relative_state().position_m) - constants::moon_mean_radius_m;
            low = std::min(low, h);
            high = std::max(high, h);
        }
        return {low, high};
    }
};
}  // namespace

TEST_CASE("HLS outbound autopilot to the 100 km polar LLO", "[sim][lunar][outbound][validation]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    fx.place(p.arrival_epoch);
    const double m0 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    REQUIRE(fx.session.plan_lunar_outbound(p));
    CHECK_FALSE(fx.session.plan_lunar_outbound(p));   // a flight under way keeps its plan
    // One warp call over the whole transfer (review focus 1): the steps must not be skipped.
    fx.session.advance_to(p.arrival_epoch + 12.0 * 3600.0, 50'000'000);
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::InLlo);
    const auto [low, high] = fx.llo_range(3);
    const double used_t = (m0 - fx.session.body(0).vehicle().mass_properties().mass_kg) / 1000.0;
    std::printf("[validation] HLS outbound LLO: %.2f x %.2f km, %.0f t used, %s\n", low / 1000.0, high / 1000.0, used_t,
                fx.session.lunar_status().c_str());
    CHECK(low > 99e3);    // spec: 100 km circular within 1 km
    CHECK(high < 101e3);
}

TEST_CASE("HLS outbound autopilot to the NRHO and down to the LLO", "[sim][lunar][outbound][nrho][validation]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.orbit = mission::LunarOrbitKind::Nrho;
    p.arrival_epoch = fx.t0 + 4.5 * 86400.0;
    p.nrho = std::make_shared<mission::NrhoReference>(
        fx.gravity, fx.ephemeris, mission::NrhoSeed{{{1.0221, 0.0, -0.1821}, {0.0, -0.1033, 0.0}}, 9, 2},
        p.arrival_epoch, mission::NrhoReferenceOptions{.revolutions = 6});
    fx.place(p.arrival_epoch);
    REQUIRE(fx.session.plan_lunar_outbound(p));
    fx.session.advance_to(p.arrival_epoch + 3600.0, 50'000'000);
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::InNrho);
    double worst_km = 0.0;
    for (int rev = 1; rev <= 3; ++rev) {
        const auto tp = p.nrho->perilune_epochs()[rev];
        fx.session.advance_to(tp, 50'000'000);
        const auto& s = fx.session.body(0).state();
        worst_km = std::max(worst_km, norm(s.position_gcrf_m - p.nrho->state_gcrf(tp).position_m) / 1000.0);
    }
    std::printf("[validation] HLS in the NRHO: worst perilune offset %.2f km over 3 revolutions\n", worst_km);
    CHECK(worst_km < 5.0);   // [E] spec: stays near the reference for three revolutions

    REQUIRE(fx.session.plan_lunar_orbit_transfer(mission::LunarOrbitKind::Llo, p.nrho));
    fx.session.advance_to(fx.session.epoch() + 9.0 * 86400.0, 50'000'000);
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::InLlo);
    const auto [low, high] = fx.llo_range(3);
    std::printf("[validation] NRHO -> LLO flown: %.2f x %.2f km\n", low / 1000.0, high / 1000.0);
    CHECK(low > 99e3);
    CHECK(high < 101e3);
}

TEST_CASE("Pilot override ends the lunar flight; a short plan is refused", "[sim][lunar][outbound]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    fx.place(p.arrival_epoch);
    REQUIRE(fx.session.plan_lunar_outbound(p));
    fx.session.advance_to(fx.session.lunar_outbound_plan()->tli.epoch + 6.0 * 3600.0, 50'000'000);
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::Coast);
    fx.session.disengage_autopilot();
    const double m = fx.session.body(0).vehicle().mass_properties().mass_kg;
    fx.session.advance_to(p.arrival_epoch + 3600.0, 50'000'000);
    CHECK(fx.session.lunar_step() == sim::LunarStep::None);
    CHECK(fx.session.autopilot().state == sim::AutopilotState::Off);
    CHECK(fx.session.body(0).vehicle().mass_properties().mass_kg == m);   // no MCC, no LOI fired

    Fixture short_fx;
    short_fx.place(p.arrival_epoch);
    // Half the propellant: drain it the realistic way is not possible here; build the HLS with half tanks instead.
    auto config = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    for (auto& tank : config.stages[0].tanks) tank.propellant_kg *= 0.5;
    short_fx.session.place_in_orbit(config, short_fx.session.body(0).state_vector(), short_fx.t0);
    CHECK_FALSE(short_fx.session.plan_lunar_outbound(p));
    CHECK(short_fx.session.lunar_outbound_plan()->reason.find("Treibstoff") != std::string::npos);
    CHECK(short_fx.session.autopilot().state == sim::AutopilotState::Off);
}
```

Adapt names that differ in the codebase while keeping the checks: the controlled-body accessor
(`controlled_index()`), the vehicle loader (`config::load_vehicle`), the tank field holding the initial load, and a
state accessor of the body (use `{state().position_gcrf_m, state().velocity_gcrf_mps}` if there is no
`state_vector()`). If `place_in_orbit` with a config replaces the body list, the second placement is fine; otherwise
create the half-tank session from scratch.

- [ ] **Step 2: Run** the build → FAIL (`LunarStep` unknown).

- [ ] **Step 3: Implement.**
  - Header: the enum, `to_string`, the public functions, and private members:
    ```cpp
    LunarStep lunar_step_ = LunarStep::None;
    time::Epoch lunar_step_epoch_;                          // next timed event of the step machine
    std::optional<mission::LunarOutboundParameters> lunar_parameters_;
    std::optional<mission::OutboundPlan> lunar_plan_;
    std::optional<mission::TwoBurnTransfer> lunar_transfer_;
    mission::LunarOrbitKind lunar_transfer_target_ = mission::LunarOrbitKind::Llo;
    std::shared_ptr<const mission::NrhoReference> lunar_nrho_;
    mission::LloTarget lunar_llo_;
    size_t lunar_mcc_index_ = 0;
    time::Epoch lunar_tli_end_;
    time::Epoch lunar_next_repoint_;
    std::string lunar_status_;
    const frames::BodyOrientation* moon_orientation_ = nullptr;   // kept from the constructor (if not stored yet)
    void run_lunar(Flying& flying);
    bool lunar_burn(const gnc::BurnNode& node, bool main_engines);  // execute + remember to come back to Lunar
    ```
  - `flight_session.cpp`: `to_string(AutopilotState::Lunar)` = `"Mondflug"`; in `run_autopilot` add
    `case AutopilotState::Lunar: run_lunar(flying); break;`; in the Burn completion branch, after `cutoff()`:
    `autopilot_.state = lunar_step_ != LunarStep::None ? AutopilotState::Lunar : (ship_return_ ? ... as now)`.
    In `coast_allowance_s`: `if (autopilot_.state == AutopilotState::Lunar) allowance = std::min(allowance,
    lunar_step_epoch_ - epoch_);` and while a Sun re-point is in progress (`attitude_mode_` Sun and not settled)
    return 0. `disengage_autopilot()` also sets `lunar_step_ = LunarStep::None`.
  - `flight_session_lunar.cpp`:
    - `plan_lunar_outbound`: false if `lunar_step_ != None` or `autopilot_.state != Off`, no Moon orientation, or no
      `vacuum` group on the controlled body (`lunar_status_ = "Kein Mondflug: keine Vakuum-Triebwerke"`). Build a
      `mission::LunarOutboundPlanner(gravity_, *env_.ephemeris, *moon_orientation_)`; `plan(state, epoch_, p, mass,
      isp of the vacuum group, propellant_kg())`; store `lunar_plan_`; refused -> `lunar_status_ = reason`, false.
      Else `lunar_parameters_ = p`, `lunar_step_ = Departure`, `lunar_step_epoch_ = tli.epoch - burn duration / 2 -
      burn_preparation_s`, `autopilot_.state = Lunar`.
    - `run_lunar` steps:
      - `Departure`: at `lunar_step_epoch_` re-solve the TLI from the current state (same burn epoch; the parking orbit
        drifted under J2 since planning) and `lunar_burn(tli, true)`; step `Tli`.
      - `Tli` (entered back in Lunar after the burn): `lunar_tli_end_ = epoch_`; step `Coast`; set Sun mode;
        `lunar_next_repoint_ = epoch_ + sun_repoint_interval_s`; `lunar_step_epoch_` = next MCC epoch
        (`tli end + mcc_after_tli_s[lunar_mcc_index_]`) or, after the last MCC, the insertion's ignition - preparation.
      - `Coast`: hold Sun mode until settled (error < 1 deg, rate < 0.05 deg/s), then `attitude_mode_ = {}` (Free) and
        coast; when `epoch_ >= lunar_next_repoint_` set Sun mode again and advance `lunar_next_repoint_`. The coast
        allowance stops at `min(lunar_step_epoch_, lunar_next_repoint_)`. At `lunar_step_epoch_`: if MCCs remain,
        `planner.mid_course(state, epoch_, epoch_ + burn_preparation_s, p)`; a burn -> `lunar_burn(mcc, false)`, step
        `MidCourse`; none -> `++lunar_mcc_index_` and the next epoch. After the last MCC: `planner.insertion(...)` ->
        `lunar_burn(insertion, true)`, step `Insertion`.
      - `MidCourse` (back after the burn): `++lunar_mcc_index_`, step `Coast` as above.
      - `Insertion` (back): LLO -> step `Trim`, `lunar_step_epoch_ = epoch_ + 600 s`; NRHO -> step `InNrho`,
        `lunar_step_epoch_` = the next apolune.
      - `Trim`: `planner.llo_trim(...)` -> a burn (`lunar_burn(trim, false)`, stays `Trim`) or none -> `InLlo`
        (status `"in LLO"`). At most 4 trims [E]; after that `InLlo` with the status naming the residual apsides.
      - `InNrho`: at each apolune `transfer.station_keeping(...)`; a burn -> `lunar_burn(sk, false)`, step
        `StationKeeping` (returns to `InNrho`); if the reference does not cover the next perilune:
        `lunar_status_ = "NRHO-Referenz endet: keine Bahnhaltung mehr"`, no further burns (review focus 5),
        `lunar_step_epoch_` far ahead (the coast allowance is then `max_coast_s`).
      - `InLlo`: nothing timed (`lunar_step_epoch_ = epoch_ + max_coast_s` each tick).
      - `TransferOut` / `TransferIn`: see `plan_lunar_orbit_transfer`.
    - `plan_lunar_orbit_transfer(target, nrho, llo)`: allowed only in `InLlo` / `InNrho` (else false). NRHO -> LLO:
      `nrho_to_llo`; LLO -> NRHO: `llo_to_nrho`. Store `lunar_transfer_`, `lunar_nrho_ = nrho`, `lunar_llo_ = llo`; step
      `TransferOut` with the first burn's ignition - preparation; at it `lunar_burn(first, true)`; back in Lunar: step
      `TransferIn`, epoch = the second burn's ignition - preparation; at it `arrival_burn(...)` -> `lunar_burn(second,
      true)`; back: LLO target -> `Trim` (as after LOI), NRHO -> `InNrho`. Refused -> `lunar_status_ = reason`, false.
    - `lunar_burn(node, main)`: `main ? execute_burn(node, {"vacuum"}) : execute_burn(node)`; false ->
      `lunar_status_ = "Brennvorgang nicht möglich"`, `lunar_step_ = None`, autopilot Off.
  - Burn duration for the Departure ignition: `gnc::burn_duration_s(|dv|, mass, thrust, mass_flow)` of the vacuum
    group (from `burn_configuration()` after a dry `execute_burn`? No - compute from the group's engines: thrust and
    mass flow of the three Raptor Vacuums; reuse the helper the session uses to fill `BurnConfiguration`).

- [ ] **Step 3b: Pilot-flown mid-course corrections** (user 2026-10-15; Apollo: the computer planned, the crew flew
  the burn and trimmed the residuals). Core only; display and controls come with l6 / M7 (`docs/ansichten.md` then).
  - `LunarOutboundParameters::pilot_corrections = false` and `pilot_call_lead_s = 600.0` [E], `pilot_window_s =
    1800.0` [E] (Task 7: optional mission keys `pilot_corrections`, default false).
  - `PilotInput::translation` (`math::Vec3`, body-axis fractions -1..1): RCS translation in manual flight (today only
    pitch / yaw / roll / throttle exist). Zero keeps every existing flight bit-identical.
  - New `LunarStep::PilotCorrection` ("Kurskorrektur (Pilot)"). With `pilot_corrections`, an MCC above the threshold
    does not call `lunar_burn`: at its epoch - `pilot_call_lead_s` the step becomes `PilotCorrection`, the session
    ticks (no coast allowance past it), the vehicle is commanded from `PilotInput` (as with the autopilot off) and
    `lunar_status_` announces it ("Kurskorrektur in 10 min: 3,2 m/s").
  - `std::optional<PilotBurnCue> lunar_pilot_cue() const` - `{gnc::BurnNode planned; math::Vec3
    remaining_delta_v_gcrf_mps; double attitude_error_rad; double window_end_s;}`: realised delta-v = current velocity
    minus the gravity-only coast of the state snapshotted at the call (no accelerometer bookkeeping); attitude error
    between the body +x axis and the remaining delta-v. The flight director's setpoint later (l6).
  - The step ends on `complete_pilot_correction()` or at the window's end (also untouched): the next MCC - or the
    insertion if none is left - is re-planned from the real state (as after an autopilot MCC); a skipped correction
    simply grows the next one. `disengage_autopilot()` ends the lunar steps as before.
  - Test (added to `test_lunar_outbound_session.cpp`, tag `[sim][lunar][outbound][pilot]`): a scripted test pilot
    (rate commands nulling `attitude_error_rad`, then +x translation until the remaining delta-v is 10 % of the planned
    one, then `complete_pilot_correction()`) flies MCC 1 of the LLO case; the flight still reaches the LLO within 1 km
    after the trims, and the total propellant is higher than in the autopilot case (print both). Second case: the
    pilot ignores the call - the window closes, MCC 2 is larger than in the autopilot case, LLO still reached.

- [ ] **Step 4: Run** `"[sim][lunar][outbound]"` (all three cases) → PASS. Runtimes go into the ledger; a case over 5 min
needs a look at the tick count (Sun re-points should be a few minutes of ticks each). Record the printed lines.
- [ ] **Step 5: Run** the existing `"[lunar]"` and `"[mission]"` tags → unchanged outcomes.
- [ ] **Step 6: Commit** `feat(sim): HLS outbound autopilot (TLI, MCC, LOI / NRHO, NRHO <-> LLO)`; push.

---

### Task 7: Mission config `lunar_landing` (outbound part) and start_mission

**Files:**
- Create: `config/missions/hls_lunar_llo.toml`, `config/missions/hls_lunar_nrho.toml`,
  `core/include/starsim/sim/lunar_landing_mission.hpp`, `core/src/sim/lunar_landing_mission.cpp`
- Modify: `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp`,
  `core/src/sim/mission_start.cpp`, `core/include/starsim/sim/mission_start.hpp` (doc comment), `core/CMakeLists.txt`,
  `core/tests/CMakeLists.txt`
- Test: `core/tests/test_lunar_landing_mission.cpp`

**Interfaces:**
- Consumes: Task 6 (`plan_lunar_outbound`), `LunarTransferTargeter::parking_orbit_raan`, `sim::lunar_site` (l2).
- Produces:
  ```cpp
  namespace starsim::config {
  enum class MissionKind { Ascent, ShipReturn, CapsuleReturn, LunarLanding };
  struct LunarLandingMission {
      std::string id, name, vehicle_id, site_id;      // site: config/lunar_sites.toml (l4 lands there)
      time::UtcDateTime epoch_utc;                    // start in the parking orbit
      double parking_height_m = 0.0;                  // circular
      double parking_inclination_rad = 0.0;
      double argument_of_latitude_rad = 0.0;
      double flight_time_s = 0.0;                     // TLI -> arrival (planned)
      double earliest_departure_s = 0.0;
      std::vector<double> mcc_after_tli_s;
      std::string lunar_orbit;                        // "llo" | "nrho"
      double llo_altitude_m = 0.0;
      double llo_inclination_rad = 0.0;
      math::Vec3 nrho_apolune_position{}, nrho_apolune_velocity{};   // CR3BP seed (nondimensional)
      int nrho_revolutions = 0, nrho_synodic_months = 0, nrho_reference_revolutions = 0;
  };
  LunarLandingMission load_lunar_landing_mission(const std::filesystem::path& config_dir, std::string_view mission_id);
  }
  namespace starsim::sim {
  // Places the HLS in its circular parking orbit at the mission's epoch, the plane through the Moon at arrival
  // (parking_orbit_raan); throws std::runtime_error (German) when no such plane exists.
  void place_lunar_landing_mission(FlightSession& session, const config::LunarLandingMission& mission,
                                   const std::filesystem::path& config_dir, time::Epoch epoch);
  // Outbound parameters (builds the NRHO reference for the NRHO variant).
  mission::LunarOutboundParameters lunar_outbound_parameters(const config::LunarLandingMission& mission,
                                                             const environment::GravityModel& gravity,
                                                             const ephemeris::Ephemeris& ephemeris, time::Epoch epoch);
  }
  ```
  `FlightSession` needs a read accessor for its gravity model and ephemeris if none exists: add
  `const environment::PointMassGravity& gravity() const { return gravity_; }` (the ephemeris is the one the caller
  passed in; `start_mission` gets it from the session if an accessor exists, else add `const ephemeris::Ephemeris&
  ephemeris() const`).
  `MissionStart` gains `std::string refusal;` (German reason when the outbound plan was refused); for `lunar_landing`
  `return_planned` means "outbound planned".

- [ ] **Step 1: Mission files.** `config/missions/hls_lunar_llo.toml`:
```toml
# Lunar landing, mission 1 of the lunar spec (l3: outbound part; l4 adds descent, surface stay and ascent). The HLS,
# refuelled in the LEO depot's orbit, departs from 200 x 200 km on its autopilot: best departure in the parking orbit,
# TLI on the vacuum Raptors, two mid-course corrections, LOI into a 100 km polar LLO. [orbit] is the scenario start.

[mission]
name = "HLS-Mondlandung (LLO)"
kind = "lunar_landing"
vehicle = "starship_hls"
site = "mons_mouton_im2"         # config/lunar_sites.toml (l4)

[orbit]
epoch_utc = "2026-11-02T00:00:00"
height_km = 200.0                # [S] spec: 200 x 200 km after the depot (NASA OIG IG-26-004)
inclination_deg = 28.5           # [E] Florida launch azimuth east; the RAAN is computed (plane through the Moon at arrival)
argument_of_latitude_deg = 0.0

[transfer]
flight_time_days = 4.0           # [E] Apollo-like direct transfer (3-4.5 days)
earliest_departure_h = 0.5       # [E]
mcc_after_tli_days = [1.0, 2.5]  # [E] spec

[lunar_orbit]
kind = "llo"
altitude_km = 100.0              # [S] spec / Artemis LLO
inclination_deg = 90.0           # [S] spec: polar
```
`config/missions/hls_lunar_nrho.toml`: same header text with "NRHO", `name = "HLS-Mondlandung (NRHO)"`,
`flight_time_days = 4.5` [E], and
```toml
[lunar_orbit]
kind = "nrho"

[nrho]
# 9:2 synodic-resonant southern L2 NRHO, CR3BP state at apolune (Earth-Moon, nondimensional)
# [S: Zimovan, Howell & Davis 2017, Table 1]; the reference is rebuilt by multiple shooting in the full model.
apolune_position = [1.0221, 0.0, -0.1821]
apolune_velocity = [0.0, -0.1033, 0.0]
revolutions = 9                  # [S] per ...
synodic_months = 2               # [S] ... synodic months
reference_revolutions = 6        # [E] span of the reference (~39 d)
```

- [ ] **Step 2: Write the failing test** `core/tests/test_lunar_landing_mission.cpp`:

```cpp
// Mission config lunar_landing: loading, placement in the parking orbit through the Moon's plane, start_mission
// dispatch, and refusal without a plane.

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/mission_config.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

TEST_CASE("Lunar landing missions load", "[config][lunar]") {
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "hls_lunar_llo") == config::MissionKind::LunarLanding);
    const auto llo = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    CHECK(llo.vehicle_id == "starship_hls");
    CHECK(llo.lunar_orbit == "llo");
    CHECK(llo.mcc_after_tli_s.size() == 2);
    const auto nrho = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_nrho");
    CHECK(nrho.lunar_orbit == "nrho");
    CHECK(nrho.nrho_revolutions == 9);
}

TEST_CASE("start_mission plans the HLS outbound flight", "[sim][lunar][mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const auto moon = spice::SpiceOrientation::moon_mean_earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    const auto start = sim::start_mission(session, STARSIM_CONFIG_DIR, "hls_lunar_llo", {}, time);
    CHECK(start.kind == config::MissionKind::LunarLanding);
    CHECK(start.return_planned);
    CHECK(session.lunar_step() == sim::LunarStep::Departure);
    CHECK(session.epoch() == time.from_utc({2026, 11, 2, 0, 0, 0.0}));
}

TEST_CASE("A parking orbit below the Moon's declination is refused", "[sim][lunar][mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const auto moon = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    auto mission = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    mission.parking_inclination_rad = 2.0 * constants::deg_to_rad;   // the Moon is at ~20 deg declination then
    const auto epoch = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    CHECK_THROWS_AS(sim::place_lunar_landing_mission(session, mission, STARSIM_CONFIG_DIR, epoch), std::runtime_error);
}
```
(Check the Moon's declination at the arrival epoch in the run and pick an inclination below it if 2 deg is not; the
test's point is the refusal path.) If `start_mission`'s `requested` epoch is a `time::Epoch`, `{}` is J2000 - fine,
the mission uses its own epoch.

- [ ] **Step 3: Run** → FAIL (no `LunarLanding`).
- [ ] **Step 4: Implement** the loader (pattern of `load_capsule_return_mission`: same `doc.text` / `doc.number` helpers,
German `doc.fail` messages; arrays via the helper that reads number arrays - if none exists, add one beside the
others), `mission_kind` mapping `"lunar_landing"`, the placement (`targeter.parking_orbit_raan(epoch +
flight_time_s, inclination)`; empty -> `throw std::runtime_error("Kein Parkorbit durch die Mondbahnebene: Bahnneigung
unter der Deklination des Mondes")`; `orbit::state_from_elements({R_e + h, 0, i, raan, 0, u}, gm_earth)`,
`session.place_in_orbit(vehicle_id, state, epoch)`), `lunar_outbound_parameters` (arrival = epoch + flight time;
NRHO: build the reference with `first_perilune = arrival`), and the `start_mission` branch (place, parameters,
`plan_lunar_outbound`, `return_planned` = its result, `refusal = session.lunar_status()`).
- [ ] **Step 5: Run** `"[lunar]"` with `[config]` / `[mission]` → PASS; smoke-check `"[mission]"` (existing start_mission
tests unchanged).
- [ ] **Step 6: Commit** `feat(config): lunar_landing missions (outbound) and their start`; push.

---

### Task 8: Execution-error campaign, docs, memory, full suite, review

**Files:**
- Modify: `core/tests/test_lunar_outbound_session.cpp` (campaign), `CLAUDE.md`, `config/README.md`
- Memory: `project-lunar-mission.md`

- [ ] **Step 1: Campaign** in `test_lunar_outbound_session.cpp`:
```cpp
TEST_CASE("HLS outbound campaign with execution errors", "[sim][lunar][outbound][campaign][validation]") {
    // TLI execution +-1 % / 0.5 deg (FlightSession::apply_velocity_error after the TLI cutoff), both orbits, 4 cases
    // each: LLO within 1 km after the trims, NRHO within 5 km of the reference at the third perilune.
}
```
Write it in full: for `i` in 0..7, orbit = `i < 4 ? Llo : Nrho`, error magnitude `(i % 2 ? +1 : -1) * 0.01` and a
0.5 deg tilt about an axis rotated by `i * 90 deg`; apply with the existing `FlightSession::apply_velocity_error` (b4)
right after the TLI step ends (advance until `lunar_step() == Coast`); then the checks of Task 6's two validation cases.
Print one `[validation]` line per case (MCC sizes, final apsides / offset, propellant used). All 8 must pass.
- [ ] **Step 2:** CLAUDE.md, a new paragraph after "l2 HLS and terrain" in the lunar bullet:
```
  l3 outbound (plan `docs/superpowers/plans/2026-10-15-lunar-l3-outbound.md`): `mission/position_targeter` (fixed-time
  position in the full model), `orbit/cr3bp` (CR3BP, fixed-period periodic orbits, instantaneous Earth-Moon rotating
  frame), `mission/nrho_reference` (9:2 southern L2 NRHO [S: Zimovan 2017 seed] by multiple shooting in the full model:
  <period> d, perilune <rp> km over the north pole, apolune <ra> km; phase set by the first perilune - no Gateway
  phasing), `mission/lunar_outbound` (TLI <n> m/s, LOI <n> m/s, NRHO insertion <n> m/s, MCCs, LLO trims, propellant
  check), `mission/lunar_orbit_transfer` (NRHO -> LLO <n> m/s, LLO -> NRHO <n> m/s, NRHO station keeping at apolune
  <n> m/s per 3 revolutions). `FlightSession::plan_lunar_outbound` / `plan_lunar_orbit_transfer`, `AutopilotState::Lunar`
  ("Mondflug"), `LunarStep`; coasts Sun-pointed with a re-aim every 6 h [E] (free coast between: no disturbance torques).
  Missions `hls_lunar_llo` / `hls_lunar_nrho` (`kind = "lunar_landing"`, outbound only until l4). Results: LLO
  <low>-<high> km; NRHO <worst> km at the perilunes over 3 revolutions; campaign 8 / 8.
```
(fill the `<...>` from the ledger). `config/README.md`: the `lunar_landing` mission keys.
- [ ] **Step 3:** Full suite in the background: `ctest --test-dir build -j 12 > $env:TEMP\l3_ctest.txt`; tail it. Bindings
build (`tools/build.ps1` without `-NoBindings`, tail for success) and the smoke test - all green.
- [ ] **Step 4:** Final review by an opus subagent over `git diff 84e4fcf..HEAD -- core config` (spec section l3 and this
plan as context); one fix pass; re-run the affected tags.
- [ ] **Step 5:** Memory `project-lunar-mission.md`: l3 done (commit), numbers, next step l4 plan. Commit
`docs: lunar l3 outbound flight`; push; ask the user for `/clear` and name the next step (write the l4 plan).

---

## Ledger

Record here per task: commit, test results (printed `[validation]` lines), runtimes, decisions and open points.

- Decisions taken while writing this plan (2026-10-15):
  - Waiting in the LLO for the site's ground track (spec l3) moves to l4: the descent planning knows the site and its
    reach; l3 ends in the LLO / NRHO.
  - Coast attitude: Sun pointing re-aimed every 6 h [E], free coast between (no disturbance torques modelled); the
    alternative - ticking the whole coast - costs ~70 M ticks per transfer.
  - NRHO phase: set by the planned arrival (first perilune = arrival); no Gateway ephemeris (M9).
  - NRHO station keeping: one targeted burn per apolune when the predicted perilune miss exceeds 1 km [E].
  - User 2026-10-15: plan approved, execution native (executing-plans; one opus review at the end).
- Task 1 (5897cc9): position targeter, `[targeter]` 2/2. Rulings: `normalized()` (plan said `normalize`); local
  Cramer's-rule 3x3 solve (no Mat3 inverse in `math`); the truth case is a 3.1 km/s TLI-like burn with a first guess
  3,000 m/s prograde - the plan's 1 km/s / 20 h case is ~8 revolutions and Newton from zero diverged (3,700 km miss),
  so callers must pass a patched-conic first guess. Executor ledger: `.superpowers/sdd/2026-10-15-lunar-l3-outbound/`.
- User 2026-10-15: mid-course corrections can be flown by the pilot (Task 6 Step 3b: switch, call before the burn,
  cue with remaining delta-v and attitude error, re-plan from the real state); display with l6 / M7.
