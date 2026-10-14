# Individual RCS Thrusters (d1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The RCS of every existing stage (Super Heavy, Starship, Falcon 9 first and second stage) acts through
individual thrusters. Each thruster has a position, a direction, a thrust and a propellant supply. Allocation turns
torque and force demands into duty cycles, and a modulator turns those into minimum-on-time pulses. The existing
campaigns keep their outcomes.

**Architecture:** Two pure units go into `propulsion/`:
- `rcs_allocator`: box-constrained least squares from the effectiveness matrix, plus the capability per axis and per
  direction.
- `rcs_modulator`: sigma-delta pulses with a minimum on-time.

`Vehicle` owns the thrusters (from `[stage.rcs]`). It runs allocator and modulator once per tick in `update_rcs`,
like `update_engines`, and draws their propellant. `FlightBody` integrates the wrench of the thrusters that actually
fire. The command interface stays fractions of the capability (`rcs_torque_fraction` / `rcs_force_fraction`), so
the controllers keep their meaning. Only the capability now comes from geometry.

**Tech Stack:** C++20 core, toml++, Catch2.

**Spec:** `docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md` (section "d1 design").

## Global Constraints

- SI units, body axes: x along the stack from the vehicle bottom, radial direction at azimuth a = (0, cos a, sin a),
  tangential = (0, -sin a, cos a). This is the grid-fin convention (`aero::grid_fin_radial`).
- A thruster's `direction` is the force on the vehicle. The plume points the other way.
- Provenance tags on every config value ([S], [E], [D]). Layout positions are [E] from imagery.
- No magic numbers in code. Weights, tolerances and iteration limits are named constants with a comment.
- Pulsed is the default fidelity. `RcsFidelity::Continuous` is the switch (session setting).
- Liftoff masses must not change: a new RCS tank is carved out of the stage's dry mass.
- Existing campaigns must keep their outcome: Super Heavy catch 21/21, ship from orbit 14/14, ship from the gate
  20/21, Falcon 9 RTLS 21/21 and ASDS 21/21 + 21/21. If one degrades, check the layout first; a controller retune is
  a ledgered ruling.
- Refinements of the spec, decided here (the spec text said otherwise):
  - The command stays in fractions of the capability, not newtons. This keeps the callers unchanged, and the spec
    asks that they "keep their meaning".
  - There is no separate `minimum_impulse_bit_ns`: it equals thrust x `minimum_on_time_s`.
  - The force capability is per direction (`rcs_force_capability(direction)`). A min over all axes is 0 for stages
    without aft-pointing thrusters. RCS burns point +x along the burn, as they do today.

## Review Focus

1. **Main-tank RCS on Super Heavy / Starship eats landing propellant.** Gas at Isp 70 s costs ~4 kg/s per 3 kN
   thruster, and a coast or entry of minutes can cost tonnes. Expected: the campaigns report the RCS propellant per
   phase, and the catch still has its reserve. Task 4 prints it, and Task 3 tests the mass flow per thruster.
2. **A stage without thrusters on an axis.** For example, Super Heavy alone has no +x thrusters. Expected: capability 0
   on that axis, no NaN, an RCS burn refused cleanly (as today without RCS). Tested in Task 1 (zero column / zero
   capability) and Task 4 (burn refused).
3. **An empty RCS tank mid-flight.** Expected: the thrusters starve, the allocator is rebuilt without them, the
   capability drops to what is left, and nothing divides by zero. Tested in Task 3.
4. **Separation while thrusters fire.** Expected: both parts keep their own thrusters with re-based stations, the
   modulator state is not carried into the wrong body, and the cache is rebuilt. Tested in Task 3.
5. **CG shifts during a burn (propellant drain).** Expected: the allocator is rebuilt when the CG moves > 1 cm, so a
   pure roll demand gives no parasitic pitch. Tested in Task 3 (CG change rebuilds the cache).

---

### Task 1: RCS allocator (pure)

**Files:**
- Create: `core/include/starsim/propulsion/rcs_allocator.hpp`, `core/src/propulsion/rcs_allocator.cpp`
- Modify: `core/CMakeLists.txt` (add the source after `src/propulsion/engine_performance.cpp`),
  `core/tests/CMakeLists.txt` (add `test_rcs.cpp`)
- Test: `core/tests/test_rcs.cpp`

**Interfaces:**
- Produces:
```cpp
namespace starsim::propulsion {
struct RcsThrusterGeometry {
    math::Vec3 position_m;  // body axes (station along x from the vehicle bottom)
    math::Vec3 direction;   // unit; force on the vehicle
    double thrust_n = 0.0;  // 0: unavailable (starved, failed)
};
class RcsAllocator {
public:
    RcsAllocator() = default;
    RcsAllocator(std::vector<RcsThrusterGeometry> thrusters, double cg_station_m);
    size_t size() const;
    double cg_station_m() const;
    // Duties in [0, 1]: torque about the CG first (weighted), then force; least propellant among equals.
    std::vector<double> allocate(const math::Vec3& torque_nm, const math::Vec3& force_n,
                                 const std::vector<double>* warm_start = nullptr) const;
    math::Vec3 torque_nm(const std::vector<double>& duty) const;
    math::Vec3 force_n(const std::vector<double>& duty) const;
    // Per body axis: the smaller of the + and - pure-torque capability (what a fraction of +-1 can rely on).
    math::Vec3 max_torque_nm() const;
    // Force along `direction_body` (unit) with no torque wanted; the duties that give it.
    double force_capability_n(const math::Vec3& direction_body, std::vector<double>* duty = nullptr) const;
};
}
```

- [ ] **Step 1: Write the failing tests** in `core/tests/test_rcs.cpp`:

```cpp
// Individual RCS thrusters: allocation, modulation, vehicle integration.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/propulsion/rcs_allocator.hpp"

using namespace starsim;
using math::Vec3;
using propulsion::RcsAllocator;
using propulsion::RcsThrusterGeometry;

namespace {

// A pod ring: at each azimuth one thruster each tangential +, tangential - and inward.
std::vector<RcsThrusterGeometry> ring(double station, double radius, double thrust, std::initializer_list<double> az_deg) {
    std::vector<RcsThrusterGeometry> out;
    for (double a_deg : az_deg) {
        const double a = a_deg * M_PI / 180.0;
        const Vec3 r{0.0, std::cos(a), std::sin(a)}, t{0.0, -std::sin(a), std::cos(a)};
        const Vec3 p = Vec3{station, 0.0, 0.0} + r * radius;
        out.push_back({p, t, thrust});
        out.push_back({p, t * -1.0, thrust});
        out.push_back({p, r * -1.0, thrust});
    }
    return out;
}

}  // namespace

TEST_CASE("RCS allocator: roll capability of a four-pod ring is four tangential thrusters", "[rcs]") {
    const RcsAllocator a(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0);
    CHECK(a.max_torque_nm().x == Catch::Approx(4 * 3000.0 * 4.5).epsilon(0.02));
    // Pitch at 38 m: the two inward thrusters on one side plus one tangential per pod whose rolls cancel - six
    // thrusters at cos 45 deg each.
    CHECK(a.max_torque_nm().y == Catch::Approx(6 * 3000.0 * std::cos(M_PI / 4) * 38.0).epsilon(0.05));
}

TEST_CASE("RCS allocator: a pure torque demand gives that torque and little force", "[rcs]") {
    const RcsAllocator a(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0);
    const Vec3 want{20000.0, -50000.0, 30000.0};
    const auto d = a.allocate(want, {});
    for (double x : d) CHECK((x >= 0.0 && x <= 1.0));
    const Vec3 got = a.torque_nm(d);
    CHECK(norm(got - want) < 0.01 * norm(want));
}

TEST_CASE("RCS allocator: torque has priority over force when saturated", "[rcs]") {
    const RcsAllocator a(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0);
    const Vec3 roll{0.9 * a.max_torque_nm().x, 0.0, 0.0};
    const auto d = a.allocate(roll, Vec3{0.0, 1e5, 0.0});  // force far beyond reach
    CHECK(a.torque_nm(d).x == Catch::Approx(roll.x).epsilon(0.03));
}

TEST_CASE("RCS allocator: no thruster along an axis gives zero capability, not NaN", "[rcs]") {
    const RcsAllocator a(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0);
    const double fx = a.force_capability_n({1.0, 0.0, 0.0});
    CHECK(std::isfinite(fx));
    CHECK(fx < 1.0);
    const RcsAllocator empty({}, 0.0);
    CHECK(norm(empty.max_torque_nm()) == 0.0);
    CHECK(empty.allocate({1.0, 0.0, 0.0}, {}).empty());
    std::vector<RcsThrusterGeometry> starved = ring(68.0, 4.5, 0.0, {45, 135, 225, 315});
    CHECK(norm(RcsAllocator(starved, 30.0).max_torque_nm()) == 0.0);
}

TEST_CASE("RCS allocator: lateral force without torque from a symmetric pair of rings", "[rcs]") {
    auto t = ring(68.0, 4.5, 3000.0, {45, 135, 225, 315});
    const auto aft = ring(8.0, 4.5, 3000.0, {45, 135, 225, 315});
    t.insert(t.end(), aft.begin(), aft.end());
    const RcsAllocator a(t, 38.0);
    std::vector<double> d;
    const double f = a.force_capability_n({0.0, 1.0, 0.0}, &d);
    CHECK(f > 3000.0);
    CHECK(norm(a.torque_nm(d)) < 0.02 * f * 30.0);
}
```

- [ ] **Step 2: Run them to verify they fail**

Run: build core (`powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`)
Expected: compile error, `starsim/propulsion/rcs_allocator.hpp` not found.

- [ ] **Step 3: Implement** `rcs_allocator.hpp` (interface above, with comments) and `rcs_allocator.cpp`:

```cpp
#include "starsim/propulsion/rcs_allocator.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::propulsion {

using math::Vec3;

namespace {

// Torque rows are scaled by 1/reference lever (comparable to force rows), then weighted: a saturated set gives up
// translation first.
constexpr double torque_priority = 100.0;
// Propellant tie-break per unit duty, relative to the mean diagonal of the normal matrix.
constexpr double propellant_weight = 1e-6;
constexpr int max_sweeps = 300;
constexpr double duty_tolerance = 1e-7;
// Capability demands: far beyond what the thrusters give, so the solution saturates.
constexpr double saturating_factor = 10.0;

}  // namespace

RcsAllocator::RcsAllocator(std::vector<RcsThrusterGeometry> thrusters, double cg_station_m)
    : thrusters_(std::move(thrusters)), cg_station_m_(cg_station_m) {
    const size_t n = thrusters_.size();
    double lever = 1.0;
    for (const auto& t : thrusters_) lever = std::max(lever, norm(t.position_m - Vec3{cg_station_m_, 0.0, 0.0}));
    torque_scale_ = torque_priority / lever;
    rows_.assign(n, {});
    for (size_t i = 0; i < n; ++i) {
        const auto& t = thrusters_[i];
        const Vec3 f = t.direction * t.thrust_n;
        const Vec3 tau = cross(t.position_m - Vec3{cg_station_m_, 0.0, 0.0}, f);
        rows_[i] = {tau, f};
    }
    gram_.assign(n * n, 0.0);
    double diagonal = 0.0;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            gram_[i * n + j] = torque_scale_ * torque_scale_ * dot(rows_[i].torque, rows_[j].torque) +
                               dot(rows_[i].force, rows_[j].force);
        }
        diagonal += gram_[i * n + i];
    }
    tie_break_ = n > 0 ? propellant_weight * diagonal / static_cast<double>(n) : 0.0;
    double scale = 0.0;  // largest single-thruster torque / force, for the saturating demands
    for (const auto& r : rows_) scale = std::max({scale, norm(r.torque), norm(r.force)});
    saturating_demand_ = saturating_factor * scale * static_cast<double>(std::max<size_t>(n, 1));
    for (int axis = 0; axis < 3; ++axis) {
        Vec3 e;
        e[axis] = 1.0;
        const double plus = dot(torque_nm(allocate(e * saturating_demand_, {})), e);
        const double minus = -dot(torque_nm(allocate(e * -saturating_demand_, {})), e);
        max_torque_[axis] = std::max(0.0, std::min(plus, minus));
    }
}

std::vector<double> RcsAllocator::allocate(const Vec3& torque_nm, const Vec3& force_n,
                                           const std::vector<double>* warm_start) const {
    const size_t n = thrusters_.size();
    std::vector<double> d(n, 0.0);
    if (warm_start && warm_start->size() == n) d = *warm_start;
    std::vector<double> c(n);  // B^T W^2 w
    for (size_t i = 0; i < n; ++i) {
        c[i] = torque_scale_ * torque_scale_ * dot(rows_[i].torque, torque_nm) + dot(rows_[i].force, force_n);
    }
    // Projected coordinate descent on 0.5 d^T G d - c^T d + tie_break * sum(d), d in [0, 1].
    for (int sweep = 0; sweep < max_sweeps; ++sweep) {
        double change = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const double gii = gram_[i * n + i];
            if (gii <= 0.0) {
                d[i] = 0.0;
                continue;
            }
            double gradient = tie_break_ - c[i];
            for (size_t j = 0; j < n; ++j) gradient += gram_[i * n + j] * d[j];
            const double next = std::clamp(d[i] - gradient / gii, 0.0, 1.0);
            change = std::max(change, std::abs(next - d[i]));
            d[i] = next;
        }
        if (change < duty_tolerance) break;
    }
    return d;
}

Vec3 RcsAllocator::torque_nm(const std::vector<double>& duty) const {
    Vec3 sum;
    for (size_t i = 0; i < duty.size() && i < rows_.size(); ++i) sum += rows_[i].torque * duty[i];
    return sum;
}

Vec3 RcsAllocator::force_n(const std::vector<double>& duty) const {
    Vec3 sum;
    for (size_t i = 0; i < duty.size() && i < rows_.size(); ++i) sum += rows_[i].force * duty[i];
    return sum;
}

double RcsAllocator::force_capability_n(const Vec3& direction_body, std::vector<double>* duty) const {
    const Vec3 u = math::normalized_or_zero(direction_body);
    auto d = allocate({}, u * saturating_demand_);
    const double f = std::max(0.0, dot(force_n(d), u));
    if (duty) *duty = std::move(d);
    return f;
}

}  // namespace starsim::propulsion
```
Private members: `thrusters_`, `cg_station_m_`, `torque_scale_ = 0`, `tie_break_ = 0`, `saturating_demand_ = 0`,
`struct Row { Vec3 torque, force; }; std::vector<Row> rows_; std::vector<double> gram_; Vec3 max_torque_;`.
`max_torque_nm()` returns `max_torque_`. If `math::Vec3` has no `operator[]`, use a small `axis(i)` helper (check
`core/include/starsim/math/vec3.hpp`).

- [ ] **Step 4: Run the tests to verify they pass**

Run: build, then `build/core/tests/starsim_core_tests.exe "[rcs]"`
Expected: all `[rcs]` tests pass. If the 1 % torque tolerance fails, the cause is an unconverged descent (raise
`max_sweeps`, ledger it), never a looser test.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/propulsion/rcs_allocator.hpp core/src/propulsion/rcs_allocator.cpp core/tests/test_rcs.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "feat(propulsion): RCS allocator over individual thrusters"
```

### Task 2: Pulse modulator (pure)

**Files:**
- Create: `core/include/starsim/propulsion/rcs_modulator.hpp`, `core/src/propulsion/rcs_modulator.cpp`
- Modify: `core/CMakeLists.txt`
- Test: `core/tests/test_rcs.cpp` (append)

**Interfaces:**
- Produces:
```cpp
namespace starsim::propulsion {
// Sigma-delta pulse modulation per thruster: the duty is accumulated as on-time; a pulse starts once a whole
// minimum on-time is owed and lasts at least that long; it continues while at least half a tick is owed.
class RcsModulator {
public:
    RcsModulator() = default;
    RcsModulator(size_t thrusters, double minimum_on_time_s);
    const std::vector<bool>& update(const std::vector<double>& duty, double dt_s);  // firing flags this tick
    const std::vector<bool>& firing() const;
};
}
```

- [ ] **Step 1: Write the failing tests** (append to `test_rcs.cpp`, add `#include "starsim/propulsion/rcs_modulator.hpp"`):

```cpp
TEST_CASE("RCS modulator: the mean on-time follows the duty, pulses are never shorter than the minimum", "[rcs]") {
    constexpr double dt = 0.005, min_on = 0.02;
    for (const double duty : {0.0, 0.03, 0.1, 0.37, 0.8, 1.0}) {
        CAPTURE(duty);
        propulsion::RcsModulator m(1, min_on);
        int on = 0, run = 0, shortest = 1 << 30;
        bool previous = false;
        const int ticks = 2000;  // 10 s
        for (int k = 0; k < ticks; ++k) {
            const bool f = m.update({duty}, dt)[0];
            on += f ? 1 : 0;
            if (f) ++run;
            if (!f && previous) shortest = std::min(shortest, run), run = 0;
            previous = f;
        }
        CHECK(on * dt == Catch::Approx(duty * ticks * dt).margin(min_on + dt));
        if (duty > 0.0 && duty < 1.0) CHECK(shortest >= static_cast<int>(std::lround(min_on / dt)));
        if (duty == 1.0) CHECK(on == ticks);
    }
}

TEST_CASE("RCS modulator: a duty that drops to zero leaves nothing owed", "[rcs]") {
    propulsion::RcsModulator m(1, 0.02);
    for (int k = 0; k < 3; ++k) m.update({0.5}, 0.005);  // owes 7.5 ms, not yet a pulse
    m.update({0.0}, 0.005);
    for (int k = 0; k < 2; ++k) CHECK_FALSE(m.update({0.3}, 0.005)[0]);  // starts from zero again
}
```

- [ ] **Step 2: Run them to verify they fail**

Run: build. Expected: compile error, `rcs_modulator.hpp` not found.

- [ ] **Step 3: Implement**

```cpp
RcsModulator::RcsModulator(size_t thrusters, double minimum_on_time_s)
    : minimum_on_time_s_(minimum_on_time_s), owed_s_(thrusters, 0.0), pulse_ticks_left_(thrusters, 0),
      firing_(thrusters, false) {}

const std::vector<bool>& RcsModulator::update(const std::vector<double>& duty, double dt_s) {
    const int pulse_ticks = std::max(1, static_cast<int>(std::lround(minimum_on_time_s_ / dt_s)));
    for (size_t i = 0; i < firing_.size(); ++i) {
        const double d = i < duty.size() ? std::clamp(duty[i], 0.0, 1.0) : 0.0;
        if (d <= 0.0) {
            owed_s_[i] = 0.0;
            pulse_ticks_left_[i] = 0;
            firing_[i] = false;
            continue;
        }
        owed_s_[i] = std::min(owed_s_[i] + d * dt_s, minimum_on_time_s_ + dt_s);  // anti-windup
        bool fire = false;
        if (pulse_ticks_left_[i] > 0) {
            fire = true;
            --pulse_ticks_left_[i];
        } else if (owed_s_[i] >= minimum_on_time_s_ - 0.5 * dt_s) {
            fire = true;
            pulse_ticks_left_[i] = pulse_ticks - 1;
        } else if (firing_[i] && owed_s_[i] >= 0.5 * dt_s) {
            fire = true;
        }
        if (fire) owed_s_[i] -= dt_s;
        firing_[i] = fire;
    }
    return firing_;
}
```
The anti-windup cap lets `owed` reach `min_on + dt`, so a duty of 1.0 fires every tick. `firing()` returns
`firing_`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `build/core/tests/starsim_core_tests.exe "[rcs]"`. Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/propulsion/rcs_modulator.hpp core/src/propulsion/rcs_modulator.cpp core/tests/test_rcs.cpp core/CMakeLists.txt
git commit -m "feat(propulsion): RCS pulse modulator with a minimum on-time"
```

### Task 3: Thrusters in the configuration, the vehicle and the physics

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp` (`RcsConfig`, `TankContents::Rcs`, `TankConfig::density_kg_m3`),
  `core/src/config/vehicle_config.cpp` (loader), `core/include/starsim/vehicle/vehicle.hpp` + `core/src/vehicle/vehicle.cpp`,
  `core/include/starsim/vehicle/flight_body.hpp` + `core/src/vehicle/flight_body.cpp`,
  `config/vehicles/starship_block3.toml`, `config/vehicles/falcon9_block5.toml`, `config/README.md`,
  `core/tests/test_vehicle_models.cpp:69` (`rcs_thruster_count` -> `rcs.thrusters`)
- Test: `core/tests/test_rcs.cpp` (append)

**Interfaces:**
- Consumes: `RcsAllocator`, `RcsModulator` (Tasks 1-2).
- Produces:
```cpp
// config
enum class TankContents { Oxidizer, Fuel, Rcs };
struct TankConfig { std::string id; TankContents contents; double mass_kg; double base_station_m;
                    double density_kg_m3 = 0.0; };  // required for "rcs" tanks
struct RcsThrusterConfig { std::string id; math::Vec3 position_m; math::Vec3 direction; double thrust_n = 0.0; };
struct RcsConfig {
    double isp_s = 0.0;
    std::string propellant;            // "main" (the stage's oxidizer + fuel tanks) or an "rcs" tank id
    double minimum_on_time_s = 0.0;
    std::vector<RcsThrusterConfig> thrusters;  // pods expanded; positions relative to the stage base
};
// StageConfig: `RcsConfig rcs;` replaces rcs_thruster_count / rcs_thrust_n.

// vehicle
enum class RcsFidelity { Pulsed, Continuous };
struct RcsThruster { std::string id; math::Vec3 position_m; math::Vec3 direction; double thrust_n = 0.0;
                     double isp_s = 0.0; int tank = -1;  /* -1: main tanks */ double output = 0.0;  /* 0..1 */
                     bool starved = false; };
// Stage: std::vector<RcsThruster> rcs;
// Vehicle:
void set_rcs_fidelity(RcsFidelity fidelity);
RcsFidelity rcs_fidelity() const;
void update_rcs(double dt_s, const VehicleCommand& command);   // allocation + modulation for this tick
math::Vec3 rcs_torque_nm() const;   // of the outputs set by update_rcs, about the CG at that time
math::Vec3 rcs_force_n() const;     // body axes
double rcs_propellant_kg() const;   // the "rcs" tanks
double propellant_kg() const;       // now oxidizer + fuel only (the gauge); RCS tanks excluded
math::Vec3 max_rcs_torque_nm() const;                       // from the allocator (cached)
struct RcsCapability { double force_n = 0.0; double mass_flow_kgps = 0.0; };
RcsCapability rcs_force_capability(const math::Vec3& direction_body) const;
// max_rcs_force_n() is removed (callers use rcs_force_capability).
// FlightBody:
math::Vec3 rcs_force_body_n() const;  // applied in the last tick
```

- [ ] **Step 1: Write the failing tests** (append to `test_rcs.cpp`; include `vehicle_config.hpp`, `vehicle.hpp`,
  `flight_body.hpp`; reuse the zero-gravity / fixed-Earth helpers from `test_flight_body.cpp` by copying them into an
  anonymous namespace):

```cpp
TEST_CASE("RCS thrusters are loaded for every stage", "[rcs][config]") {
    const auto ss = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto f9 = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    CHECK(ss.stages[0].rcs.thrusters.size() == 12);
    CHECK(ss.stages[1].rcs.thrusters.size() == 14);
    CHECK(f9.stages[0].rcs.thrusters.size() == 8);
    CHECK(f9.stages[1].rcs.thrusters.size() == 8);
    // Pod expansion: Super Heavy pod 1 at 45 deg, radius 4.5, station 68 m; first thruster tangential +.
    const auto& t = ss.stages[0].rcs.thrusters[0];
    CHECK(t.position_m.x == Catch::Approx(68.0));
    CHECK(t.position_m.y == Catch::Approx(4.5 * std::cos(M_PI / 4)));
    CHECK(t.direction.y == Catch::Approx(-std::sin(M_PI / 4)));
    CHECK(f9.stages[0].rcs.propellant == "rcs_n2");
}

TEST_CASE("RCS tanks do not change the liftoff mass and stay out of the propellant gauge", "[rcs][config]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const vehicle::Vehicle v(cfg, STARSIM_CONFIG_DIR);
    CHECK(v.rcs_propellant_kg() == Catch::Approx(200.0 + 80.0));
    double main = 0.0;
    for (const auto& s : cfg.stages) for (const auto& t : s.tanks) if (t.contents != config::TankContents::Rcs) main += t.mass_kg;
    CHECK(v.propellant_kg() == Catch::Approx(main));
    // 25.6 t + 3.9 t dry before d1: the N2 was carved out of them.
    CHECK(cfg.stages[0].dry_mass_kg + 200.0 == Catch::Approx(25600.0));
    CHECK(cfg.stages[1].dry_mass_kg + 80.0 == Catch::Approx(3900.0));
}

TEST_CASE("A roll command spins the Falcon 9 second stage and draws its nitrogen", "[rcs][vehicle]") {
    // Second stage alone in zero gravity, continuous fidelity: roll acceleration = capability / Ixx, N2 drawn at
    // sum(output * F) / (Isp g0).
    // (build the stack, separate(1), wrap the upper part in a FlightBody at rest, step 2 s at 200 Hz with
    //  rcs_torque_fraction = {1, 0, 0})
    // CHECK: roll rate == Approx(2 s * max_rcs_torque_nm().x / Ixx).epsilon(0.03); |pitch|,|yaw rate| < 1 % of it;
    //        linear speed < 0.01 m/s; N2 used == Approx(expected).epsilon(0.02).
}

TEST_CASE("Pulsed and continuous RCS give the same mean torque", "[rcs][vehicle]") {
    // Same setup, rcs_torque_fraction = {0.13, 0, 0}, 4 s, both fidelities: roll rates agree within 5 %.
}

TEST_CASE("An empty RCS tank starves its thrusters", "[rcs][vehicle]") {
    // Second stage, empty its rcs tank (stages()[0].tanks[i].mass_kg = 0 for the rcs tank), step once:
    // max_rcs_torque_nm() == 0 on all axes, a roll command gives no rotation, no NaN in the state.
}

TEST_CASE("Separation re-bases the thrusters and gives each part its own", "[rcs][vehicle]") {
    // Falcon 9 stack: second-stage thruster x before = 41.2 + local station; after separate(1) the upper part's
    // thruster x = local station; the lower part keeps 8 thrusters, the upper part 8; both report max_rcs_torque > 0.
}

TEST_CASE("The RCS allocation follows the CG", "[rcs][vehicle]") {
    // Super Heavy alone: max_rcs_torque_nm().y with full and with empty tanks differs (lever = station - CG);
    // after draining, a pure roll command produces |pitch torque| < 1 % of the roll torque.
}

TEST_CASE("RCS capability of every stage (report)", "[rcs][vehicle]") {
    // For each stage alone (Super Heavy, ship, F9 S1, F9 S2): print the new max torque per axis and +x force next to
    // the pre-d1 abstraction (force = 0.5 * count * thrust; torque = (force * D/2, force * L/2, force * L/2), with the
    // old counts SH 8 x 3000 N, ship 6 x 1500 N, F9 S1 8 x 450 N, F9 S2 none). CHECK all new torques > 0 (Task 4 reads
    // the print against the campaigns).
}
```
Write the bodies of the commented tests in full in the implementation, using the setup each comment states. The
`CHECK` lines in the comments are the assertions.

- [ ] **Step 2: Run them to verify they fail**

Run: build. Expected: compile errors (`rcs` member, `TankContents::Rcs`, `rcs_propellant_kg` missing).

- [ ] **Step 3: Configuration and loader**

In `vehicle_config.cpp`, tank parsing accepts `"rcs"` (and then requires `density_kg_m3` via `doc.positive`).
`[stage.rcs]` parsing:
```cpp
if (auto rcs = t["rcs"].as_table()) {
    s.rcs.isp_s = doc.positive(*rcs, "isp_s");
    s.rcs.propellant = doc.text(*rcs, "propellant");
    s.rcs.minimum_on_time_s = doc.positive(*rcs, "minimum_on_time_s");
    if (s.rcs.propellant != "main" &&
        std::none_of(s.tanks.begin(), s.tanks.end(), [&](const TankConfig& k) {
            return k.id == s.rcs.propellant && k.contents == TankContents::Rcs; })) {
        doc.fail("stage.rcs.propellant must be \"main\" or the id of a tank with contents = \"rcs\"");
    }
    if (auto pods = (*rcs)["pod"].as_array()) {
        for (const auto& node : *pods) {
            const auto& p = *node.as_table();
            const std::string id = doc.text(p, "id");
            const double station = doc.number(p, "station_m"), radius = doc.positive(p, "radius_m");
            const double thrust = doc.positive(p, "thrust_n");
            const auto azimuths = number_list(doc, p, "azimuths_deg");     // helper below
            const auto directions = text_list(doc, p, "directions");
            for (size_t k = 0; k < azimuths.size(); ++k) {
                const double a = azimuths[k] * constants::deg_to_rad;
                const math::Vec3 r{0.0, std::cos(a), std::sin(a)}, tg{0.0, -std::sin(a), std::cos(a)};
                for (const auto& d : directions) {
                    math::Vec3 dir;
                    if (d == "tangential_plus") dir = tg;
                    else if (d == "tangential_minus") dir = tg * -1.0;
                    else if (d == "inward") dir = r * -1.0;
                    else if (d == "forward") dir = {1.0, 0.0, 0.0};
                    else if (d == "aft") dir = {-1.0, 0.0, 0.0};
                    else doc.fail("stage.rcs.pod.directions: unknown '" + d + "'");
                    s.rcs.thrusters.push_back({id + "_" + std::to_string(k + 1) + "_" + d,
                                               math::Vec3{station, 0.0, 0.0} + r * radius, dir, thrust});
                }
            }
        }
    }
    if (auto list = (*rcs)["thruster"].as_array()) {
        for (const auto& node : *list) {
            const auto& q = *node.as_table();
            const auto pos = number_list(doc, q, "position_m"), dir = number_list(doc, q, "direction");
            if (pos.size() != 3 || dir.size() != 3) doc.fail("stage.rcs.thruster position_m / direction need 3 numbers");
            const math::Vec3 d{dir[0], dir[1], dir[2]};
            if (norm(d) <= 0.0) doc.fail("stage.rcs.thruster direction must not be zero");
            s.rcs.thrusters.push_back({doc.text(q, "id"), {pos[0], pos[1], pos[2]}, math::normalized(d),
                                       doc.positive(q, "thrust_n")});
        }
    }
    if (s.rcs.thrusters.empty()) doc.fail("stage.rcs needs [[stage.rcs.pod]] or [[stage.rcs.thruster]] entries");
}
```
Add `number_list` / `text_list` helpers in the anonymous namespace. They follow the pattern of the existing
`azimuths_deg` parsing: fail on a missing key or a non-number entry. Refactor the existing azimuth loops to use
`number_list` only if the code is touched anyway; otherwise leave them.

- [ ] **Step 4: Vehicle configs** (values [E], comments as below)

`starship_block3.toml`, booster (replace `[stage.rcs]`):
```toml
[stage.rcs]
type = "cold_gas"                         # autogenous ullage gas from the main tanks [E]
isp_s = 70.0                              # [E] warm gas thrusters (cold N2 ~70 s; GCH4/GOX similar order)
propellant = "main"
minimum_on_time_s = 0.02                  # [E] valve

[[stage.rcs.pod]]
id = "forward"
station_m = 68.0                          # [E] below the hot-stage ring (imagery)
radius_m = 4.5
azimuths_deg = [45.0, 135.0, 225.0, 315.0]  # [E] between the grid fins' quadrants
thrust_n = 3000.0                         # [E] (unchanged from the pre-d1 estimate)
directions = ["tangential_plus", "tangential_minus", "inward"]
```
Ship (replace `[stage.rcs]`; belly at 270 deg, so the clusters sit on the sides at 0 / 180 deg):
```toml
[stage.rcs]
type = "cold_gas_ullage"
isp_s = 70.0                              # [E]
propellant = "main"
minimum_on_time_s = 0.02                  # [E]

[[stage.rcs.pod]]
id = "forward"
station_m = 46.0                          # [E] forward of the forward flaps
radius_m = 4.5
azimuths_deg = [0.0, 180.0]
thrust_n = 1500.0                         # [E] (unchanged)
directions = ["tangential_plus", "tangential_minus", "inward"]

[[stage.rcs.pod]]
id = "aft"
station_m = 6.0                           # [E] above the aft flaps
radius_m = 4.5
azimuths_deg = [0.0, 180.0]
thrust_n = 1500.0
directions = ["tangential_plus", "tangential_minus", "inward", "forward"]   # forward: ullage settling / translation
```
(Ship: 2 x 3 + 2 x 4 = 14 thrusters.)

Falcon 9 first stage: `dry_mass_kg = 25400.0` (comment: `# [S:wiki_f9] 25.6 t incl. the 0.2 t N2 below [E]`), and:
```toml
[[stage.tank]]
id = "rcs_n2"
contents = "rcs"
mass_kg = 200.0                           # [E] nitrogen, carved out of the 25.6 t dry mass
density_kg_m3 = 250.0                     # [E] COPV gas at ~300 bar
base_station_m = 38.0                     # [E] interstage

[stage.rcs]
type = "cold_gas"                         # nitrogen, two pods at the top
isp_s = 70.0                              # [S] typical N2 cold gas
propellant = "rcs_n2"
minimum_on_time_s = 0.02                  # [E]

[[stage.rcs.pod]]
id = "pod"
station_m = 40.0                          # [E] interstage, below the grid fins
radius_m = 1.83
azimuths_deg = [0.0, 180.0]               # [E] between the fins (45 / 135 / 225 / 315 deg)
thrust_n = 450.0                          # [E] (unchanged)
directions = ["tangential_plus", "tangential_minus", "inward", "inward"]
```
Falcon 9 second stage: `dry_mass_kg = 3820.0` (`# [S:wiki_f9] 3.9 t incl. the 80 kg N2 below [E]`), tank `rcs_n2`
80 kg at `base_station_m = 1.0`, density 250, and:
```toml
[stage.rcs]
type = "cold_gas"
isp_s = 70.0                              # [S] typical N2 cold gas
propellant = "rcs_n2"
minimum_on_time_s = 0.02                  # [E]

[[stage.rcs.pod]]
id = "pod"
station_m = 1.5                           # [E] aft skirt
radius_m = 1.83
azimuths_deg = [90.0, 270.0]
thrust_n = 200.0                          # [E]
directions = ["tangential_plus", "tangential_minus", "inward", "forward"]
```
Then run `python tools/vehicle_report.py falcon9_block5` and `starship_block3`. Expected: liftoff masses unchanged.

- [ ] **Step 5: Vehicle**

- `Vehicle` constructor: tank density for `Rcs` from `tank_config.density_kg_m3`. Thrusters
  `position_m = cfg.position_m + {station, 0, 0}`, `tank` = the index of the named rcs tank, or -1 for "main".
- `propellant_kg()` sums oxidizer + fuel only, and `rcs_propellant_kg()` sums the rcs tanks. Grep the callers of
  `propellant_kg()` (telemetry, guidance, landing propellant): all of them mean the gauge, and need no change.
- Allocator cache, private: `mutable std::optional<propulsion::RcsAllocator> rcs_allocator_;`. Rebuilt when absent,
  when `|cg - allocator.cg_station_m()| > rcs_cg_tolerance_m` (0.01 m, named constant), or after any starvation
  change (reset to `nullopt`). Geometry: all stages' thrusters in stage order, with `thrust_n = 0` when starved.
  `max_rcs_torque_nm()` returns `allocator().max_torque_nm()`. `rcs_force_capability(dir)`: force and the mass flow of
  the capability duties (`sum duty * F / (Isp g0)`).
- `update_rcs(dt, command)`:
  ```cpp
  const auto& a = allocator();                       // rebuilt for the current CG if needed
  const Vec3 cap = a.max_torque_nm();
  const Vec3 f = command.rcs_torque_fraction;
  const Vec3 torque{std::clamp(f.x, -1.0, 1.0) * cap.x, std::clamp(f.y, -1.0, 1.0) * cap.y,
                    std::clamp(f.z, -1.0, 1.0) * cap.z};
  Vec3 force;
  const double m = norm(command.rcs_force_fraction);
  if (m > 0.0) {
      const Vec3 u = command.rcs_force_fraction / m;
      force = u * (std::min(m, 1.0) * a.force_capability_n(u));
  }
  rcs_duty_ = a.allocate(torque, force, &rcs_duty_);
  if (fidelity_ == RcsFidelity::Pulsed) {
      if (rcs_modulator_.firing().size() != rcs_duty_.size())
          rcs_modulator_ = propulsion::RcsModulator(rcs_duty_.size(), minimum_on_time());
      const auto& firing = rcs_modulator_.update(rcs_duty_, dt_s);
      // output_i = firing_i ? 1 : 0
  } else {
      // output_i = duty_i
  }
  rcs_torque_ = a.torque_nm(outputs); rcs_force_ = a.force_n(outputs);
  ```
  `minimum_on_time()` is the smallest `minimum_on_time_s` among the stages that have thrusters. Store `output` on each
  `RcsThruster` for rendering.
- `tank_mass_flows_kgps()`: add each thruster's `output * F / (Isp g0)`, either to its rcs tank, or (main) split over
  the stage's first non-empty oxidizer and fuel tanks in proportion to the stage's oxidizer and fuel mass.
- `consume_propellant()`: an empty rcs tank starves its thrusters. A dry stage (as for the engines) starves its main
  thrusters. A change resets the allocator cache.
- `separate()`: re-base thruster `position_m.x -= shift`, and reset the allocator cache and modulator on both parts.
  Copy the fidelity to the upper part.
- Remove `max_rcs_force_n()`. Callers move to `rcs_force_capability(...)` in Task 4; until then FlightSession uses
  `rcs_force_capability({1,0,0}).force_n` at line 300 and the realised force at line 820 (minimal edits, completed
  in Task 4).

- [ ] **Step 6: FlightBody**

`step()`: call `vehicle_->update_rcs(dt_s, command)` after `update_flaps`. In the clamped / caught / landed early-return
path, call it with a default `VehicleCommand{}` so the outputs drop to zero. The tick context takes
`vehicle_->rcs_torque_nm()` and `vehicle_->rcs_force_n()`. Remove the private `rcs_torque(command)` /
`rcs_force(command)`. Store `rcs_force_body_n_ = vehicle_->rcs_force_n()` for `rcs_force_body_n()`.

- [ ] **Step 7: Run the tests to verify they pass**

Run: build, `build/core/tests/starsim_core_tests.exe "[rcs]"`, then `"[vehicle]"` and `"[config]"`.
Expected: all pass. `test_vehicle_models.cpp:69` now checks `!s.rcs.thrusters.empty()`.

- [ ] **Step 8: Commit**

```bash
git add -A core config
git commit -m "feat(vehicle): RCS as individual thrusters with propellant and pulses"
```

### Task 4: Session, burns, campaigns and docs

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp` + `core/src/sim/flight_session.cpp` (RCS burn configuration,
  realised acceleration, `set_rcs_fidelity`), `CLAUDE.md`, `config/README.md`
- Test: `core/tests/test_rcs.cpp` (append), existing campaigns

**Interfaces:**
- Consumes: `Vehicle::rcs_force_capability`, `FlightBody::rcs_force_body_n`, `RcsFidelity` (Task 3).
- Produces: `void FlightSession::set_rcs_fidelity(vehicle::RcsFidelity fidelity);`. This applies to every body now
  and later; bodies created by separation inherit it through `Vehicle::separate`.

- [ ] **Step 1: Write the failing tests** (append):

```cpp
TEST_CASE("An RCS burn uses the thrusters' force and mass flow along +x", "[rcs][burn]") {
    // Session with the Starship stack in a 300 km orbit (as in "Burns below the main-engine minimum impulse use RCS
    // translation"): execute a 0.5 m/s burn; burn_configuration().rcs, thrust_n ==
    // Approx(vehicle.rcs_force_capability({1,0,0}).force_n), mass_flow_kgps > 0.1 (the ship's two aft forward
    // thrusters at Isp 70 s: 2 x 1500 / (70 x 9.81) = 4.4 kg/s at full output).
}

TEST_CASE("A stage without forward thrusters refuses an RCS burn", "[rcs][burn]") {
    // Session with Super Heavy alone placed in orbit (place_in_orbit with the booster stage only, or separate and keep
    // index 0): a 0.5 m/s burn below the Raptor minimum -> execute_burn returns false (no +x thrusters), nothing NaN.
}
```

- [ ] **Step 2: Run them to verify they fail**

Run: `build/core/tests/starsim_core_tests.exe "[rcs][burn]"`. Expected: the mass flow check fails (1e-9 today).

- [ ] **Step 3: Implement**

- RCS burn configuration (`flight_session.cpp:294-305`):
  ```cpp
  const auto cap = vehicle.rcs_force_capability({1.0, 0.0, 0.0});
  rcs.thrust_n = cap.force_n;
  rcs.mass_flow_kgps = cap.mass_flow_kgps;
  ```
- Realised acceleration (`flight_session.cpp:819-822`): move it after `body.step(...)`:
  `last_rcs_acceleration_ = body.state().attitude.rotate(body.rcs_force_body_n()) / body.vehicle().mass_properties().mass_kg;`
- `set_rcs_fidelity`: store it, apply it to all bodies, and apply it in the places where bodies are created
  (`launch_from_pad`, `place_*`).

- [ ] **Step 4: Run the new tests and the RCS burn test**

Run: `"[rcs]"` and `"Burns below the main-engine minimum impulse use RCS translation"`.
Expected: PASS; the 0.5 m/s burn stays within 30 m of the expected semi-major axis.

- [ ] **Step 5: Full suite and campaigns**

Run: the full ctest (`ctest --test-dir build -j 12`, ~4 min) with output to a log in the workspace. Then the campaigns
with `STARSIM_VERBOSE=1` to logs: `"[gnc][catch][validation]"`, `"[ship][catch][validation]"`,
`"[ship][mission][validation]"`, `"[falcon9][rtls][validation]"`, `"[falcon9][asds][validation]"`.
Expected: 326 + the new tests pass; outcomes 21/21, 20/21, 14/14, 21/21, 21/21 + 21/21.
Record in the ledger:
- the RCS propellant per mission (booster return: MECO -> landing-burn ignition; ship: deorbit -> gate). Print this
  in the full-return tests, and in `FlightSession` diagnostics via `STARSIM_VERBOSE`.
- the capability report next to the pre-d1 values.

If a campaign degrades: check the layout against imagery first (station, pods), then the RCS propellant draw (Review
Focus 1). A changed value, or a controller retune, is a ledgered `Ruling:` with the before/after numbers.

- [ ] **Step 6: Docs**

`CLAUDE.md`: in "Architecture", add `propulsion` (RCS allocator + modulator) to the implemented list. Add a bullet
under "Known issues"/Game structure: RCS as individual thrusters (`[stage.rcs]` pods/thrusters, allocator
torque-priority box least squares, sigma-delta pulses with minimum on-time, `RcsFidelity`, main-tank gas on
Starship/Super Heavy, N2 tanks on Falcon 9), with the propellant numbers from Step 5. Remove "RCS translation comes with
docking, M9" only where it is no longer true (burns below the minimum still pick RCS). `config/README.md`: the
`[stage.rcs]` keys, `[[stage.rcs.pod]]` directions vocabulary, `[[stage.rcs.thruster]]`, and tank `contents = "rcs"`
with `density_kg_m3`.

- [ ] **Step 7: Commit**

```bash
git add -A core config CLAUDE.md
git commit -m "feat(sim): RCS burns on the thrusters' force and mass flow; docs"
```
