# Lunar l1: Reference Body Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Vehicles fly, rest and report relative to the nearest celestial body (Earth or Moon) while their integrated state stays Earth-centred GCRF.

**Architecture:** A body-neutral `environment::ReferenceBody` (Earth, Moon) supplies GM, shape, body-fixed frame (`BodyFrame`: centre, rotation, rotation rate), and atmosphere or vacuum. `FlightBody` picks its reference body with hysteresis (Moon within 60,000 km, Earth again beyond 100,000 km) and routes everything that was ITRF/WGS84-specific through it: air, ground contact, resting, telemetry, ground clearance. `FlightSession` owns the Moon reference body when given a Moon orientation, uses the clearance for its coast/tick decision and flies LVLH about the reference body. The Earth case is numerically identical to today (centre at the GCRF origin: `x - 0 == x`).

**Tech Stack:** C++20 core (`core/`), Catch2 tests, CSPICE through `spice::` adapters (tests only), CMake/Ninja via `tools/build.ps1`.

**Spec:** `docs/superpowers/specs/2026-10-14-lunar-missions-design.md` (section "l1 design: reference body").

## Global Constraints

- SI units internally, angles in radians; frame and unit in names (`position_gcrf_m`, `*_fixed_m` for a reference body's body-fixed frame).
- Vehicle states stay Earth-centred GCRF ("Architecture: vehicle states stay Earth-centred GCRF ... Integration is not switched at a sphere of influence").
- Reference-body hysteresis: "Moon when within 60,000 km of the Moon, Earth again beyond 100,000 km [E] (the NRHO stays with the Moon)".
- Moon: sphere R = 1737.4 km (`constants::moon_mean_radius_m`), frame `MOON_ME` (`spice::SpiceOrientation::moon_mean_earth()`), no atmosphere.
- `core` modules depend downwards only: `environment` must not include `vehicle` or `sim`.
- CSPICE only behind `spice::` (the core takes `frames::BodyOrientation` / `ephemeris::Ephemeris`); only the simulation thread calls them.
- Existing behaviour unchanged: "the Earth case keeps its numbers bit for bit where possible (all existing campaigns re-run unchanged in outcome)".
- Validation: "a 100 km LLO and the NRHO propagated one day Earth-centred agree with the Moon-centred propagation of the same force model within 1 m [E]; a body resting on the Moon stays within 1 mm of its body-fixed point over a day".
- Code, comments, commits in English; UI strings in the core German. Conventional Commits with the session trailer.
- Physical constants only in `core/include/starsim/math/constants.hpp`; estimates tagged [E] in comments.

## Review Focus

1. A body placed near the Moon before its first step (telemetry right after `place_body`) must already report Moon-relative values - the selection must not depend on a step having run (Task 2: test "telemetry before the first step").
2. A body held on a pad / caught / landed / resting must never switch reference body whatever the Moon's distance (Task 2: the held branch of `reference_body`; the resting test checks the Moon is kept after a day on the ground).
3. The Earth-centred atmosphere flags (`entered_atmosphere_`, skip-out "Abprall") must not fire near the Moon (heights 80 / 100 km above the Moon): Task 2 test "no skip-out at the Moon".
4. Separation near the Moon: both parts keep the Moon as reference body (Task 2 test "separated parts keep the reference body").
5. A session without a Moon orientation (all existing callers, bindings) behaves exactly as before (Task 3: determinism diff of validation output before/after; Task 5 full suite).

---

### Task 1: Reference body and body frame

**Files:**
- Create: `core/include/starsim/environment/reference_body.hpp`
- Create: `core/src/environment/reference_body.cpp`
- Modify: `core/include/starsim/frames/body_orientation.hpp` (add `rotation_vector`)
- Modify: `core/include/starsim/environment/atmosphere.hpp`, `core/src/environment/us76_atmosphere.cpp` (add `vacuum_state()`)
- Modify: `core/src/vehicle/flight_body.cpp` (`earth_rotation_vector` delegates to `frames::rotation_vector`)
- Modify: `core/CMakeLists.txt` (add `src/environment/reference_body.cpp` after `src/environment/eclipse.cpp`)
- Test: `core/tests/test_reference_body.cpp` (new), `core/tests/CMakeLists.txt` (add it after `test_lunar_transfer.cpp`)

**Interfaces:**
- Consumes: `frames::BodyOrientation::fixed_to_gcrf`, `frames::Ellipsoid`, `frames::fixed_to_geodetic/geodetic_to_fixed`, `ephemeris::Ephemeris::state_gcrf/gm_m3ps2`, `environment::Atmosphere`.
- Produces:
  - `math::Vec3 frames::rotation_vector(const frames::FixedToInertial&)`
  - `environment::AtmosphereState environment::vacuum_state()`
  - `struct environment::BodyFrame { frames::FixedToInertial transform; math::Vec3 centre_gcrf_m; math::Vec3 centre_velocity_gcrf_mps; math::Vec3 rotation_gcrf_radps; to_fixed(pos) ; position_gcrf(fixed) ; velocity_gcrf(fixed, fixed_vel) ; relative_velocity(pos, vel) ; centred(StateVector) ; static centred_at_origin(transform) }`
  - `class environment::ReferenceBody { static earth(const frames::BodyOrientation&, const Atmosphere&, double gm); static moon(const ephemeris::Ephemeris&, const frames::BodyOrientation&); id(); gm_m3ps2(); shape(); has_atmosphere(); air(height_m); centre_gcrf(epoch); frame(epoch); }`

- [ ] **Step 1: Write the failing tests**

`core/tests/test_reference_body.cpp`:
```cpp
// Reference bodies (lunar l1): body-neutral frames of the Earth and the Moon.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
}

TEST_CASE("Spherical shapes convert geodetic coordinates exactly", "[environment][reference_body]") {
    const frames::Ellipsoid sphere{constants::moon_mean_radius_m, 0.0};
    for (const double lat : {-89.5, -45.0, 0.0, 30.0, 89.9}) {
        for (const double lon : {-170.0, 0.0, 95.0}) {
            const frames::GeodeticPosition g{lat * deg, lon * deg, 1234.5};
            const Vec3 fixed = frames::geodetic_to_fixed(g, sphere);
            CHECK(norm(fixed) == Catch::Approx(constants::moon_mean_radius_m + 1234.5).epsilon(1e-14));
            const auto back = frames::fixed_to_geodetic(fixed, sphere);
            CHECK(back.latitude_rad == Catch::Approx(g.latitude_rad).margin(1e-12));
            CHECK(back.longitude_rad == Catch::Approx(g.longitude_rad).margin(1e-12));
            CHECK(back.height_m == Catch::Approx(1234.5).margin(1e-6));
        }
    }
}

TEST_CASE("The Earth reference body is centred at the GCRF origin and reproduces ITRF", "[environment][reference_body]") {
    test::spice_session();
    const spice::SpiceOrientation itrf = spice::SpiceOrientation::earth();
    const environment::Us76Atmosphere atmosphere;
    const auto earth = environment::ReferenceBody::earth(itrf, atmosphere, constants::earth_gm_m3ps2);
    const auto t = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const auto frame = earth.frame(t);
    const auto direct = itrf.fixed_to_gcrf(t);
    CHECK(earth.id() == ephemeris::Body::Earth);
    CHECK(earth.has_atmosphere());
    CHECK(frame.centre_gcrf_m == Vec3{});
    CHECK(frame.centre_velocity_gcrf_mps == Vec3{});
    const Vec3 r{6.5e6, 1.0e5, 2.0e5}, v{100.0, 7500.0, 30.0};
    // Bit-identical to the ITRF arithmetic used so far.
    CHECK(frame.to_fixed(r) == transpose(direct.rotation) * r);
    CHECK(frame.relative_velocity(r, v) == v - cross(frames::rotation_vector(direct), r));
    CHECK(norm(frame.rotation_gcrf_radps) == Catch::Approx(7.2921e-5).epsilon(1e-4));
    CHECK(earth.shape().semi_major_axis_m == constants::wgs84_semi_major_axis_m);
    CHECK(earth.air(0.0).density_kg_m3 == Catch::Approx(1.225).epsilon(1e-3));
}

TEST_CASE("The Moon reference body follows DE440 and MOON_ME", "[environment][reference_body]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    const auto moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    const auto t = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const auto frame = moon.frame(t);
    const auto centre = ephemeris.state_gcrf(ephemeris::Body::Moon, t);
    CHECK(moon.id() == ephemeris::Body::Moon);
    CHECK_FALSE(moon.has_atmosphere());
    CHECK(moon.gm_m3ps2() == ephemeris.gm_m3ps2(ephemeris::Body::Moon));
    CHECK(moon.shape().semi_major_axis_m == constants::moon_mean_radius_m);
    CHECK(moon.shape().flattening == 0.0);
    CHECK(frame.centre_gcrf_m == centre.position_m);
    CHECK(frame.centre_velocity_gcrf_mps == centre.velocity_mps);
    CHECK(moon.centre_gcrf(t).position_m == centre.position_m);
    // Sidereal rotation 27.32 d.
    CHECK(norm(frame.rotation_gcrf_radps) == Catch::Approx(2.0 * constants::pi / (27.321661 * 86400.0)).epsilon(0.02));
    // A point fixed on the surface moves with the Moon: its GCRF velocity is the centre's plus omega x r.
    const Vec3 fixed = frames::geodetic_to_fixed({-89.0 * deg, 0.0, 0.0}, moon.shape());
    const Vec3 r = frame.position_gcrf(fixed);
    CHECK(norm(r - centre.position_m) == Catch::Approx(constants::moon_mean_radius_m).epsilon(1e-12));
    const Vec3 v = frame.velocity_gcrf(fixed, {});
    CHECK(norm(frame.relative_velocity(r, v)) < 1e-9);
    CHECK(norm(frame.to_fixed(r) - fixed) < 1e-6);
    // Vacuum: no air, engines see no back-pressure, the free path is infinite.
    const auto air = moon.air(100.0);
    CHECK(air.density_kg_m3 == 0.0);
    CHECK(air.pressure_pa == 0.0);
    CHECK(std::isinf(air.mean_free_path_m));
    CHECK(air.speed_of_sound_mps > 0.0);  // Mach stays finite (zero dynamic pressure)
}
```
Add `test_reference_body.cpp` to the test list in `core/tests/CMakeLists.txt` (after `test_lunar_transfer.cpp`).

- [ ] **Step 2: Run the build to verify it fails**

Run: `L=C:/Users/johan/.claude/jobs/78816a1a/tmp/b.log; powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests > "$L" 2>&1; grep -iE "error C|LNK" $L | head`
Expected: compile errors "cannot open include file 'starsim/environment/reference_body.hpp'".

- [ ] **Step 3: Implement**

`core/include/starsim/frames/body_orientation.hpp`, after `class BodyOrientation`:
```cpp
// Rotation vector of a body-fixed frame in GCRF axes from the transform's rate (omega_x = dR/dt * R^T).
inline math::Vec3 rotation_vector(const FixedToInertial& transform) {
    const math::Mat3 w = transform.rotation_rate_per_s * transpose(transform.rotation);
    return {0.5 * (w(2, 1) - w(1, 2)), 0.5 * (w(0, 2) - w(2, 0)), 0.5 * (w(1, 0) - w(0, 1))};
}
```
`core/src/vehicle/flight_body.cpp`: replace the body of `earth_rotation_vector` with `return frames::rotation_vector(transform);` (same arithmetic).

`core/include/starsim/environment/atmosphere.hpp`, after `AtmosphereState`:
```cpp
// No atmosphere: zero density and pressure, infinite free path; the US76 exosphere temperature (1000 K) keeps the
// speed of sound and with it the Mach number finite (the state US76 reports above 1000 km).
AtmosphereState vacuum_state();
```
`core/src/environment/us76_atmosphere.cpp`: implement `vacuum_state()` returning exactly what `upper_atmosphere` returns at and above the last table altitude (`{0.0, 0.0, upper.back().temperature_k, speed_of_sound(upper.back().temperature_k), infinity}`), and make that branch of `upper_atmosphere` `return vacuum_state();`.

`core/include/starsim/environment/reference_body.hpp`:
```cpp
#pragma once

#include "starsim/environment/atmosphere.hpp"
#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::environment {

// A reference body's body-fixed frame at one epoch: GCRF axes, centre relative to the Earth's centre.
struct BodyFrame {
    frames::FixedToInertial transform;      // body-fixed axes -> GCRF axes
    math::Vec3 centre_gcrf_m;               // the body's centre (Earth: zero)
    math::Vec3 centre_velocity_gcrf_mps;
    math::Vec3 rotation_gcrf_radps;         // the body's rotation vector

    static BodyFrame centred_at_origin(const frames::FixedToInertial& transform) {
        return {transform, {}, {}, frames::rotation_vector(transform)};
    }
    math::Vec3 to_fixed(const math::Vec3& position_gcrf_m) const {
        return transpose(transform.rotation) * (position_gcrf_m - centre_gcrf_m);
    }
    math::Vec3 position_gcrf(const math::Vec3& fixed_m) const { return centre_gcrf_m + transform.position(fixed_m); }
    math::Vec3 velocity_gcrf(const math::Vec3& fixed_m, const math::Vec3& fixed_velocity_mps) const {
        return centre_velocity_gcrf_mps + transform.velocity(fixed_m, fixed_velocity_mps);
    }
    // Velocity relative to the rotating body (for the Earth: relative to the co-rotating air).
    math::Vec3 relative_velocity(const math::Vec3& position_gcrf_m, const math::Vec3& velocity_gcrf_mps) const {
        return velocity_gcrf_mps - centre_velocity_gcrf_mps - cross(rotation_gcrf_radps, position_gcrf_m - centre_gcrf_m);
    }
    // Inertial state relative to the centre (GCRF axes): orbit elements, LVLH.
    math::StateVector centred(const math::StateVector& gcrf) const {
        return {gcrf.position_m - centre_gcrf_m, gcrf.velocity_mps - centre_velocity_gcrf_mps};
    }
};

// Everything that is relative to a celestial body (lunar spec l1). Earth and Moon are instances; nothing in it is
// specific to either. A small value type: it refers to the orientation, atmosphere and ephemeris it was made from.
class ReferenceBody {
public:
    // The Earth: ITRF93, WGS84, the given atmosphere; centre at the GCRF origin (no ephemeris call).
    static ReferenceBody earth(const frames::BodyOrientation& itrf, const Atmosphere& atmosphere, double gm_m3ps2);
    // The Moon: MOON_ME (the frame of lunar maps), sphere R = 1737.4 km, no atmosphere; centre from the ephemeris.
    static ReferenceBody moon(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& moon_me);

    ephemeris::Body id() const { return id_; }
    double gm_m3ps2() const { return gm_m3ps2_; }
    const frames::Ellipsoid& shape() const { return shape_; }
    bool has_atmosphere() const { return atmosphere_ != nullptr; }
    // Air at a height above the shape; vacuum_state() without an atmosphere.
    AtmosphereState air(double height_m) const { return atmosphere_ ? atmosphere_->at(height_m) : vacuum_state(); }
    math::StateVector centre_gcrf(time::Epoch epoch) const;
    BodyFrame frame(time::Epoch epoch) const;

private:
    ReferenceBody(ephemeris::Body id, double gm_m3ps2, frames::Ellipsoid shape, const frames::BodyOrientation& orientation,
                  const Atmosphere* atmosphere, const ephemeris::Ephemeris* ephemeris)
        : id_(id), gm_m3ps2_(gm_m3ps2), shape_(shape), orientation_(&orientation), atmosphere_(atmosphere),
          ephemeris_(ephemeris) {}

    ephemeris::Body id_;
    double gm_m3ps2_;
    frames::Ellipsoid shape_;
    const frames::BodyOrientation* orientation_;
    const Atmosphere* atmosphere_;           // nullptr: vacuum
    const ephemeris::Ephemeris* ephemeris_;  // nullptr: centred at the GCRF origin (the Earth)
};

}  // namespace starsim::environment
```
`core/src/environment/reference_body.cpp`:
```cpp
#include "starsim/environment/reference_body.hpp"

#include "starsim/math/constants.hpp"

namespace starsim::environment {

ReferenceBody ReferenceBody::earth(const frames::BodyOrientation& itrf, const Atmosphere& atmosphere, double gm_m3ps2) {
    return {ephemeris::Body::Earth, gm_m3ps2, frames::wgs84, itrf, &atmosphere, nullptr};
}

ReferenceBody ReferenceBody::moon(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& moon_me) {
    return {ephemeris::Body::Moon, ephemeris.gm_m3ps2(ephemeris::Body::Moon),
            frames::Ellipsoid{constants::moon_mean_radius_m, 0.0}, moon_me, nullptr, &ephemeris};
}

math::StateVector ReferenceBody::centre_gcrf(time::Epoch epoch) const {
    return ephemeris_ ? ephemeris_->state_gcrf(id_, epoch) : math::StateVector{};
}

BodyFrame ReferenceBody::frame(time::Epoch epoch) const {
    const auto transform = orientation_->fixed_to_gcrf(epoch);
    const auto centre = centre_gcrf(epoch);
    return {transform, centre.position_m, centre.velocity_mps, frames::rotation_vector(transform)};
}

}  // namespace starsim::environment
```
Add `src/environment/reference_body.cpp` to `core/CMakeLists.txt` after `src/environment/eclipse.cpp`.

- [ ] **Step 4: Build and run the tests**

Run: `L=C:/Users/johan/.claude/jobs/78816a1a/tmp/b.log; powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests > "$L" 2>&1; grep -iE "error C|LNK" $L; build/core/tests/starsim_core_tests.exe "[reference_body]"`
Expected: no compile errors; `All tests passed (… assertions in 3 test cases)`. If the sphere case of `fixed_to_geodetic` fails (Heikkinen with e^2 = 0 divides by g = p^2 + z^2: fine away from the centre), fix it there with a closed-form sphere branch (`flattening == 0`: latitude `atan2(z, p)`, height `|r| - a`) and ledger the ruling.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/environment/reference_body.hpp core/src/environment/reference_body.cpp core/include/starsim/frames/body_orientation.hpp core/include/starsim/environment/atmosphere.hpp core/src/environment/us76_atmosphere.cpp core/src/vehicle/flight_body.cpp core/CMakeLists.txt core/tests/test_reference_body.cpp core/tests/CMakeLists.txt
git commit -m "feat(environment): reference bodies Earth and Moon with body-fixed frames"
```
(with the session trailer lines)

---

### Task 2: FlightBody relative to its reference body

**Files:**
- Modify: `core/include/starsim/vehicle/flight_body.hpp`
- Modify: `core/src/vehicle/flight_body.cpp`
- Test: `core/tests/test_lunar_flight_body.cpp` (new; add to `core/tests/CMakeLists.txt`)

**Interfaces:**
- Consumes (Task 1): `environment::ReferenceBody`, `environment::BodyFrame`, `frames::rotation_vector`.
- Produces:
  - `FlightEnvironment::moon` — `const environment::ReferenceBody* moon = nullptr;` (last member; existing aggregate initialisers stay valid)
  - `environment::ReferenceBody vehicle::reference_body_of(const FlightEnvironment&, ephemeris::Body)` (free function)
  - `ephemeris::Body FlightBody::reference_body(const FlightEnvironment&, time::Epoch) const`
  - `static constexpr double FlightBody::moon_capture_distance_m = 60.0e6, moon_release_distance_m = 100.0e6`
  - `struct vehicle::GroundClearance { double height_m; double sink_mps; }` and `GroundClearance FlightBody::ground_clearance(const FlightEnvironment&, time::Epoch) const`
  - `FlightTelemetry` new fields `reference_body`, `periapsis_altitude_m`, `apoapsis_altitude_m`, `inclination_rad`
  - `FlightBody::hold_fixed(const environment::BodyFrame&, ephemeris::Body)` (generic; `hold_fixed_to_earth` keeps its signature and calls it)

- [ ] **Step 0: Record a determinism baseline (before any change of this task)**

Run (from the repo root, build from Task 1 current):
```bash
B=C:/Users/johan/.claude/jobs/78816a1a/tmp; build/core/tests/starsim_core_tests.exe "Vacuum burn reproduces the rocket equation,Ship entry reproduces the webcast telemetry of flights 12-14,Booster descent drag reproduces the webcast telemetry of flights 13 and 14,Falcon 9 descent drag reproduces*" > $B/l1_baseline.txt 2>&1; tail -3 $B/l1_baseline.txt
```
Expected: `All tests passed`. (These print their validation numbers; Step 6 diffs them.)

- [ ] **Step 1: Write the failing tests**

`core/tests/test_lunar_flight_body.cpp`:
```cpp
// FlightBody near the Moon (lunar l1): the reference body decides ground, air and telemetry; the state stays GCRF.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;
using vehicle::FlightBody;
using vehicle::RigidBodyState;

namespace {

constexpr double deg = constants::deg_to_rad;
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

struct LunarFixture {
    LunarFixture() { test::spice_session(); }
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    environment::Us76Atmosphere atmosphere;
    environment::ReferenceBody moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, &ephemeris, false, &moon};
    time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
};

std::unique_ptr<FlightBody> ship(const RigidBodyState& state) {
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    return std::make_unique<FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, config_dir), state);
}

// Upright (body +x along the local vertical) at a selenographic point, `sink_mps` downwards relative to the surface.
RigidBodyState above_moon(const environment::ReferenceBody& moon, time::Epoch t, double lat, double lon,
                          double height_m, double sink_mps) {
    const auto frame = moon.frame(t);
    const Vec3 fixed = frames::geodetic_to_fixed({lat, lon, height_m}, moon.shape());
    const Vec3 up_fixed = math::normalized(fixed);
    const Vec3 up = frame.transform.rotation * up_fixed;
    const Vec3 y = math::normalized(cross(up, Vec3{0.0, 0.0, 1.0}));
    const Vec3 z = cross(up, y);
    math::Mat3 axes;
    for (int i = 0; i < 3; ++i) {
        axes(i, 0) = i == 0 ? up.x : i == 1 ? up.y : up.z;
        axes(i, 1) = i == 0 ? y.x : i == 1 ? y.y : y.z;
        axes(i, 2) = i == 0 ? z.x : i == 1 ? z.y : z.z;
    }
    RigidBodyState s;
    s.position_gcrf_m = frame.position_gcrf(fixed);
    s.velocity_gcrf_mps = frame.velocity_gcrf(fixed, -sink_mps * up_fixed);
    s.attitude = math::Quat::from_matrix(axes);
    s.angular_velocity_body_radps = s.attitude.inverse_rotate(frame.rotation_gcrf_radps);
    return s;
}

// 100 km circular polar orbit about the Moon (Moon-centred elements in GCRF axes; inclination to GCRF z).
RigidBodyState lunar_orbit(const environment::ReferenceBody& moon, time::Epoch t) {
    const auto rel = orbit::state_from_elements(
        {constants::moon_mean_radius_m + 100e3, 0.0, 90.0 * deg, 0.0, 0.0, 0.0}, moon.gm_m3ps2());
    const auto centre = moon.centre_gcrf(t);
    return {rel.position_m + centre.position_m, rel.velocity_mps + centre.velocity_mps, {}, {}};
}

}  // namespace

TEST_CASE("A ship touching down gently on the Moon rests and follows the Moon for a day", "[vehicle][lunar]") {
    LunarFixture fx;
    // The state is the CG's: put the bottom 0.5 m above the ground.
    const double cg = ship(lunar_orbit(fx.moon, fx.t0))->vehicle().mass_properties().cg_station_m;
    auto body = ship(above_moon(fx.moon, fx.t0, -85.0 * deg, 30.0 * deg, cg + 0.5, 1.0));
    time::Epoch t = fx.t0;
    for (int i = 0; i < 2000 && !body->resting(); ++i, t = t + 0.005) {
        body->step(fx.env, t, 0.005, {});
    }
    REQUIRE(body->resting());
    CHECK_FALSE(body->impacted());
    REQUIRE(body->touchdown_sink_mps().has_value());
    CHECK(*body->touchdown_sink_mps() < 2.0);
    const auto tm0 = body->telemetry(fx.env, t);
    CHECK(tm0.reference_body == ephemeris::Body::Moon);
    CHECK(tm0.latitude_rad == Catch::Approx(-85.0 * deg).margin(1e-6));
    CHECK(tm0.longitude_rad == Catch::Approx(30.0 * deg).margin(1e-6));
    const Vec3 fixed0 = fx.moon.frame(t).to_fixed(body->state().position_gcrf_m);
    body->coast(fx.env, t, 86400.0);
    t = t + 86400.0;
    CHECK(body->resting());
    CHECK(body->reference_body(fx.env, t) == ephemeris::Body::Moon);
    CHECK(norm(fx.moon.frame(t).to_fixed(body->state().position_gcrf_m) - fixed0) < 1e-3);
    CHECK(body->telemetry(fx.env, t).speed_relative_mps < 1e-6);
}

TEST_CASE("A ship falling hard onto the Moon impacts", "[vehicle][lunar]") {
    LunarFixture fx;
    const double cg = ship(lunar_orbit(fx.moon, fx.t0))->vehicle().mass_properties().cg_station_m;
    auto body = ship(above_moon(fx.moon, fx.t0, 10.0 * deg, -20.0 * deg, cg + 2.0, 20.0));
    time::Epoch t = fx.t0;
    for (int i = 0; i < 1000 && !body->impacted(); ++i, t = t + 0.005) body->step(fx.env, t, 0.005, {});
    CHECK(body->impacted());
}

TEST_CASE("Telemetry near the Moon is relative to the Moon, also before the first step", "[vehicle][lunar]") {
    LunarFixture fx;
    auto body = ship(lunar_orbit(fx.moon, fx.t0));
    const auto tm = body->telemetry(fx.env, fx.t0);  // no step yet
    CHECK(tm.reference_body == ephemeris::Body::Moon);
    CHECK(tm.altitude_m == Catch::Approx(100e3).margin(1.0));
    CHECK(tm.periapsis_altitude_m == Catch::Approx(100e3).margin(1.0));
    CHECK(tm.apoapsis_altitude_m == Catch::Approx(100e3).margin(1.0));
    CHECK(tm.speed_inertial_mps == Catch::Approx(norm(body->state().velocity_gcrf_mps)));  // GCRF, unchanged meaning
    CHECK(tm.speed_relative_mps == Catch::Approx(1633.0).margin(10.0));  // relative to the rotating Moon
    CHECK(tm.dynamic_pressure_pa == 0.0);  // vacuum
    CHECK(std::abs(tm.vertical_speed_mps) < 0.5);
}

TEST_CASE("Telemetry in Earth orbit reports orbit elements about the Earth", "[vehicle][lunar]") {
    LunarFixture fx;
    const auto rel = orbit::state_from_elements({6378137.0 + 200e3, 0.0, 51.6 * deg, 0.0, 0.0, 0.0},
                                                fx.ephemeris.gm_m3ps2(ephemeris::Body::Earth));
    auto body = ship({rel.position_m, rel.velocity_mps, {}, {}});
    const auto tm = body->telemetry(fx.env, fx.t0);
    CHECK(tm.reference_body == ephemeris::Body::Earth);
    CHECK(tm.periapsis_altitude_m == Catch::Approx(200e3).margin(1.0));
    CHECK(tm.apoapsis_altitude_m == Catch::Approx(200e3).margin(1.0));
    // Inclination to the true equator of date (ITRF z): within the ~0.2 deg of precession-nutation of GCRF's.
    CHECK(tm.inclination_rad == Catch::Approx(51.6 * deg).margin(0.3 * deg));
}

TEST_CASE("The reference body switches with hysteresis at 60,000 / 100,000 km from the Moon", "[vehicle][lunar]") {
    LunarFixture fx;
    const auto centre = fx.moon.centre_gcrf(fx.t0);
    const Vec3 out = math::normalized(-centre.position_m);  // from the Moon towards the Earth
    auto at = [&](double distance_m) {
        return RigidBodyState{centre.position_m + distance_m * out, centre.velocity_mps, {}, {}};
    };
    auto body = ship(at(80.0e6));
    CHECK(body->reference_body(fx.env, fx.t0) == ephemeris::Body::Earth);  // never was at the Moon
    body->set_state(at(59.0e6));
    body->coast(fx.env, fx.t0, 1.5);  // a coast (or step) records the choice
    CHECK(body->reference_body(fx.env, fx.t0 + 1.5) == ephemeris::Body::Moon);
    body->set_state(at(90.0e6));
    CHECK(body->reference_body(fx.env, fx.t0 + 1.5) == ephemeris::Body::Moon);  // inside the hysteresis band
    body->set_state(at(101.0e6));
    CHECK(body->reference_body(fx.env, fx.t0 + 1.5) == ephemeris::Body::Earth);
    // Without a Moon in the environment everything is relative to the Earth (all callers before l1).
    vehicle::FlightEnvironment earth_only{fx.gravity, fx.atmosphere, fx.earth, &fx.ephemeris, false};
    body->set_state(at(10.0e6));
    CHECK(body->reference_body(earth_only, fx.t0) == ephemeris::Body::Earth);
}

TEST_CASE("Separated parts keep the reference body", "[vehicle][lunar]") {
    LunarFixture fx;
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    auto stack = std::make_unique<FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, config_dir),
                                              lunar_orbit(fx.moon, fx.t0));
    stack->coast(fx.env, fx.t0, 2.0);
    auto upper = stack->separate(1, 0.5);
    CHECK(stack->reference_body(fx.env, fx.t0 + 2.0) == ephemeris::Body::Moon);
    CHECK(upper->reference_body(fx.env, fx.t0 + 2.0) == ephemeris::Body::Moon);
}

TEST_CASE("No skip-out is judged at the Moon", "[vehicle][lunar]") {
    LunarFixture fx;
    // Through 80 km and back above 100 km above the Moon: Earth's atmosphere events must not fire.
    auto body = ship(above_moon(fx.moon, fx.t0, 0.0, 0.0, 70e3, -400.0));  // climbing 400 m/s from 70 km
    time::Epoch t = fx.t0;
    for (int i = 0; i < 30000; ++i, t = t + 0.005) body->step(fx.env, t, 0.005, {});  // 150 s: ~112 km
    CHECK(body->telemetry(fx.env, t).altitude_m > 100e3);
    CHECK_FALSE(body->skipped_out());
}

TEST_CASE("Ground clearance is measured over the reference body", "[vehicle][lunar]") {
    LunarFixture fx;
    auto body = ship(above_moon(fx.moon, fx.t0, 45.0 * deg, 45.0 * deg, 3000.0, 12.0));
    const auto c = body->ground_clearance(fx.env, fx.t0);
    const double cg = body->vehicle().mass_properties().cg_station_m;
    CHECK(c.height_m == Catch::Approx(3000.0 + cg).margin(0.01));  // of the CG; the ground height at the Moon is 0
    CHECK(c.sink_mps == Catch::Approx(12.0).margin(0.01));
}
```
`above_moon` places the CG (the state is the CG's); `resolve_ground_contact` checks the bottom point, hence `cg + 0.5` / `cg + 2.0` in the contact tests. The clearance test measures the CG and keeps the CG height.

Add `test_lunar_flight_body.cpp` to `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Run the build to verify it fails**

Run the core build command of Task 1 Step 2.
Expected: compile errors (`FlightEnvironment` has no member `moon`; `reference_body`, `ground_clearance`, telemetry fields unknown).

- [ ] **Step 3: Implement the header changes**

In `flight_body.hpp`:
- `#include "starsim/environment/reference_body.hpp"`.
- `FlightEnvironment`: comment "Everything outside the vehicle that acts on it. Gravity must be Earth-centred; the Earth is the reference body unless `moon` is given and the vehicle is near it." Add last member:
  ```cpp
  const environment::ReferenceBody* moon = nullptr;  // lunar l1: the Moon as a reference body (nullptr: Earth only)
  ```
- `FlightTelemetry`: comments of `altitude_m`, `latitude_rad`, `speed_relative_mps`, `vertical_speed_mps` say "of the reference body" (Earth: WGS84 as before; Moon: sphere, MOON_ME); add
  ```cpp
  ephemeris::Body reference_body = ephemeris::Body::Earth;
  double periapsis_altitude_m = 0.0;  // osculating orbit about the reference body, above its equatorial radius
  double apoapsis_altitude_m = 0.0;   // infinity on an open orbit
  double inclination_rad = 0.0;       // to the reference body's equator of date (body-fixed z)
  ```
- Free function and struct before `class FlightBody`:
  ```cpp
  // The reference body `id` of an environment (the Earth from its orientation and atmosphere; the Moon if given).
  environment::ReferenceBody reference_body_of(const FlightEnvironment& env, ephemeris::Body id);

  struct GroundClearance {
      double height_m = 0.0;  // CG above the ground of the reference body
      double sink_mps = 0.0;  // inertial velocity relative to the body's centre, towards it
  };
  ```
- In `FlightBody` public:
  ```cpp
  // Reference body (lunar l1): the Moon within moon_capture_distance_m of it, the Earth again beyond
  // moon_release_distance_m (hysteresis on the choice of the last step / coast); bodies on the ground keep theirs.
  static constexpr double moon_capture_distance_m = 60.0e6;   // [E]
  static constexpr double moon_release_distance_m = 100.0e6;  // [E] the NRHO's apolune (~70,000 km) stays inside
  ephemeris::Body reference_body(const FlightEnvironment& env, time::Epoch epoch) const;
  GroundClearance ground_clearance(const FlightEnvironment& env, time::Epoch epoch) const;
  // Fixes the body where it is to the body-fixed frame `frame` of `body` (generic form of hold_fixed_to_earth).
  void hold_fixed(const environment::BodyFrame& frame, ephemeris::Body body);
  ```
  `ground_height_m()` comment: "Flat ground at this height above WGS84 when the reference body is the Earth (the pad elevation); at the Moon the ground is the sphere (terrain: l2)."
- Private: rename `pad_itrf_m_` -> `pad_fixed_m_`, `attitude_itrf_` -> `attitude_fixed_` (body -> body-fixed axes of `held_on_` while clamped/caught/landed/resting); add
  ```cpp
  ephemeris::Body reference_ = ephemeris::Body::Earth;  // chosen at the last step / coast
  ephemeris::Body held_on_ = ephemeris::Body::Earth;    // the body pad_fixed_m_ / attitude_fixed_ refer to
  double ground_height_on(ephemeris::Body body) const { return body == ephemeris::Body::Earth ? ground_height_m_ : 0.0; }
  ```
  and change the private signatures to
  ```cpp
  void follow_pad(const environment::BodyFrame& frame);
  void resolve_ground_contact(const environment::ReferenceBody& body, const environment::BodyFrame& frame);
  void update_air_data(const environment::ReferenceBody& body, const environment::BodyFrame& frame);
  ```

- [ ] **Step 4: Implement the source changes**

In `flight_body.cpp`:
1. `TickContext`: replace `math::Mat3 earth_fixed_to_gcrf; Vec3 earth_rotation_gcrf;` by
   ```cpp
   const environment::ReferenceBody& body;  // reference body of the tick
   environment::BodyFrame frame;            // its frame at the start of the tick (air only: the Moon is airless)
   ```
   and update every aggregate initialiser (`step`, `coast`, `telemetry`) to pass `body, frame` in that position.
2. `evaluate`: 
   ```cpp
   const Vec3 r_fixed = ctx.frame.to_fixed(s.position_gcrf_m);
   const auto geodetic = frames::fixed_to_geodetic(r_fixed, ctx.body.shape());
   const auto air = ctx.body.air(geodetic.height_m);
   // Aerodynamics relative to the co-rotating atmosphere (no wind model yet).
   const Vec3 air_velocity = ctx.frame.relative_velocity(s.position_gcrf_m, s.velocity_gcrf_mps);
   ```
3. Add in the anonymous namespace nothing new; add at namespace scope:
   ```cpp
   environment::ReferenceBody reference_body_of(const FlightEnvironment& env, ephemeris::Body id) {
       if (id == ephemeris::Body::Moon && env.moon) return *env.moon;
       return environment::ReferenceBody::earth(
           env.earth_orientation, env.atmosphere,
           env.ephemeris ? env.ephemeris->gm_m3ps2(ephemeris::Body::Earth) : constants::earth_gm_m3ps2);
   }

   ephemeris::Body FlightBody::reference_body(const FlightEnvironment& env, time::Epoch epoch) const {
       if (clamped_ || caught_ || landed_ || resting_) return held_on_;
       if (env.moon == nullptr) return ephemeris::Body::Earth;
       const double distance = norm(state_.position_gcrf_m - env.moon->centre_gcrf(epoch).position_m);
       const double limit = reference_ == ephemeris::Body::Moon ? moon_release_distance_m : moon_capture_distance_m;
       return distance < limit ? ephemeris::Body::Moon : ephemeris::Body::Earth;
   }
   ```
4. `on_pad`: `body->held_on_ = ephemeris::Body::Earth;` and `body->follow_pad(environment::BodyFrame::centred_at_origin(earth.fixed_to_gcrf(epoch)));` (field renames).
5. `follow_pad(const environment::BodyFrame& frame)`:
   ```cpp
   const double cg = vehicle_->mass_properties().cg_station_m;
   const Vec3 cg_fixed = pad_fixed_m_ + attitude_fixed_.rotate(Vec3{cg, 0.0, 0.0});
   state_.position_gcrf_m = frame.position_gcrf(cg_fixed);
   state_.velocity_gcrf_mps = frame.velocity_gcrf(cg_fixed, {});
   state_.attitude = (Quat::from_matrix(frame.transform.rotation) * attitude_fixed_).normalized();
   state_.angular_velocity_body_radps = state_.attitude.inverse_rotate(frame.rotation_gcrf_radps);
   ```
6. `hold_fixed(frame, body)`:
   ```cpp
   const double cg = vehicle_->mass_properties().cg_station_m;
   pad_fixed_m_ = frame.to_fixed(state_.position_gcrf_m - state_.attitude.rotate(Vec3{cg, 0.0, 0.0}));
   attitude_fixed_ = (Quat::from_matrix(transpose(frame.transform.rotation)) * state_.attitude).normalized();
   held_on_ = body;
   reference_ = body;
   caught_ = true;
   resting_ = false;
   follow_pad(frame);
   ```
   `hold_fixed_to_earth(earth, epoch)` becomes `hold_fixed(environment::BodyFrame::centred_at_origin(earth.fixed_to_gcrf(epoch)), ephemeris::Body::Earth);` (`hold_landed` unchanged otherwise).
   Note for the Earth: `frame.to_fixed(x)` is `transpose(R) * (x - 0)` - the same value as `to_fixed * x` before.
7. `step`: at the top (after the `impacted_` early return)
   ```cpp
   reference_ = reference_body(env, epoch);
   const auto body = reference_body_of(env, reference_);
   const auto frame = body.frame(epoch);
   const auto geodetic = frames::fixed_to_geodetic(frame.to_fixed(state_.position_gcrf_m), body.shape());
   vehicle_->update_engines(dt_s, command, body.air(geodetic.height_m).pressure_pa);
   ```
   `update_air_data(body, frame);` ; held branch: `follow_pad(reference_body_of(env, held_on_).frame(epoch + dt_s));` ; `TickContext ctx{env, *vehicle_, epoch, body, frame, ...}`; the entry-event lines become
   ```cpp
   if (body.has_atmosphere()) {
       if (geodetic.height_m < entry_height_m) entered_atmosphere_ = true;
       if (entered_atmosphere_ && geodetic.height_m > skip_height_m) skipped_out_ = true;
   }
   ```
   and the end: `resolve_ground_contact(body, body.frame(epoch + dt_s));`
8. `resolve_ground_contact(body, frame)`: replace `to_fixed *` by `frame.to_fixed(...)`, geodetic calls get `body.shape()`, `ground_height_m_` by `ground_height_on(body.id())`, `up = frame.transform.rotation * local_up_fixed(bottom)`, `relative = frame.relative_velocity(state_.position_gcrf_m, state_.velocity_gcrf_mps)`, the rest pose:
   ```cpp
   pad_fixed_m_ = frames::geodetic_to_fixed({bottom.latitude_rad, bottom.longitude_rad, ground_height_on(body.id())},
                                            body.shape());
   attitude_fixed_ = (Quat::from_matrix(transpose(frame.transform.rotation)) * state_.attitude).normalized();
   held_on_ = body.id();
   follow_pad(frame);
   resting_ = true;
   ```
   (`local_up_fixed` of a spherical point is the radial direction: correct for the Moon.)
9. `update_air_data(body, frame)`: geodetic from `frame.to_fixed` with `body.shape()`, `air = body.air(...)`, `air_velocity = frame.relative_velocity(...)`.
10. `coast`: 
    ```cpp
    if (resting_ || caught_ || landed_) {
        follow_pad(reference_body_of(env, held_on_).frame(epoch + duration_s));
        return;
    }
    reference_ = reference_body(env, epoch);
    const auto body = reference_body_of(env, reference_);
    const TickContext ctx{env, *vehicle_, epoch, body, body.frame(epoch), ...};
    ...
    state_ = unpack(result.y);
    reference_ = reference_body(env, epoch + duration_s);
    const auto end_body = reference_body_of(env, reference_);
    const auto end_frame = end_body.frame(epoch + duration_s);
    resolve_ground_contact(end_body, end_frame);
    update_air_data(end_body, end_frame);
    ```
11. `separate`: `upper_body->reference_ = reference_;` (next to `ground_height_m_`).
12. `telemetry`:
    ```cpp
    const auto id = reference_body(env, epoch);
    const auto body = reference_body_of(env, id);
    const auto frame = body.frame(epoch);
    const auto geodetic = frames::fixed_to_geodetic(frame.to_fixed(state_.position_gcrf_m), body.shape());
    t.reference_body = id;
    ... (altitude/lat/lon as before)
    const Vec3 up = frame.transform.rotation * local_up_fixed(geodetic);
    const Vec3 relative = frame.relative_velocity(state_.position_gcrf_m, state_.velocity_gcrf_mps);
    ... (speeds, flight path as before)
    const auto centred = frame.centred({state_.position_gcrf_m, state_.velocity_gcrf_mps});
    const auto el = orbit::elements_from_state(centred, body.gm_m3ps2());
    const double radius = body.shape().semi_major_axis_m;
    t.periapsis_altitude_m = el.semi_major_axis_m * (1.0 - el.eccentricity) - radius;
    t.apoapsis_altitude_m = el.eccentricity < 1.0 ? el.semi_major_axis_m * (1.0 + el.eccentricity) - radius
                                                  : std::numeric_limits<double>::infinity();
    const Vec3 h = cross(centred.position_m, centred.velocity_mps);
    const Vec3 pole = frame.transform.rotation * Vec3{0.0, 0.0, 1.0};
    t.inclination_rad = std::acos(std::clamp(dot(h, pole) / norm(h), -1.0, 1.0));
    const TickContext ctx{env, *vehicle_, epoch, body, frame, vehicle_->tank_mass_flows_kgps(), {}, {}, {}};
    ```
    (`#include "starsim/orbit/kepler.hpp"` and `<limits>`.)
13. `ground_clearance`:
    ```cpp
    GroundClearance FlightBody::ground_clearance(const FlightEnvironment& env, time::Epoch epoch) const {
        const auto id = reference_body(env, epoch);
        const auto body = reference_body_of(env, id);
        const auto frame = body.frame(epoch);
        const auto centred = frame.centred({state_.position_gcrf_m, state_.velocity_gcrf_mps});
        const double height =
            frames::fixed_to_geodetic(frame.to_fixed(state_.position_gcrf_m), body.shape()).height_m - ground_height_on(id);
        return {height, -dot(centred.velocity_mps, math::normalized(centred.position_m))};
    }
    ```
    For the Earth this is exactly the session's former arithmetic (`height - ground_height_m()`, `-dot(v, normalized(r))`).

- [ ] **Step 5: Build and run the new tests**

Run: core build command; then `build/core/tests/starsim_core_tests.exe "[vehicle][lunar]"`
Expected: all 8 test cases pass. If the gentle-touchdown test impacts, print `touchdown_sink_mps` and the tilt: the helper's attitude must put body +x along the local vertical (fix the helper, not the limits).

- [ ] **Step 6: Determinism check and the affected suites**

Run:
```bash
B=C:/Users/johan/.claude/jobs/78816a1a/tmp; build/core/tests/starsim_core_tests.exe "Vacuum burn reproduces the rocket equation,Ship entry reproduces the webcast telemetry of flights 12-14,Booster descent drag reproduces the webcast telemetry of flights 13 and 14,Falcon 9 descent drag reproduces*" > $B/l1_after.txt 2>&1; diff <(grep -v "^Filters\|seconds\|^Randomness" $B/l1_baseline.txt) <(grep -v "^Filters\|seconds\|^Randomness" $B/l1_after.txt) && echo IDENTICAL
build/core/tests/starsim_core_tests.exe "[vehicle],[legs],[tower],[reference_body]" > $B/l1_t2.txt 2>&1; tail -3 $B/l1_t2.txt
```
Expected: `IDENTICAL`; `All tests passed`. A difference in the printed numbers means the Earth path changed arithmetic: find it (usually an operation order in `to_fixed` / `relative_velocity`), do not loosen anything.

- [ ] **Step 7: Commit**

```bash
git add core/include/starsim/vehicle/flight_body.hpp core/src/vehicle/flight_body.cpp core/tests/test_lunar_flight_body.cpp core/tests/CMakeLists.txt
git commit -m "feat(vehicle): flight bodies rest, sense air and report relative to their reference body"
```

---

### Task 3: FlightSession with the Moon as a reference body

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp` (constructor, `moon_` member before `env_`, `reference_body(i)`)
- Modify: `core/src/sim/flight_session.cpp` (constructor; `coast_allowance_s` clearance block; attitude-mode LVLH about the reference body)
- Test: `core/tests/test_lunar_session.cpp` (new; add to `core/tests/CMakeLists.txt`)

**Interfaces:**
- Consumes (Task 2): `FlightEnvironment::moon`, `FlightBody::reference_body`, `FlightBody::ground_clearance`, `vehicle::reference_body_of`.
- Produces:
  - `FlightSession(const ephemeris::Ephemeris&, const frames::BodyOrientation& earth, std::filesystem::path config_dir, environment::PointMassGravityConfig gravity_config = {}, const frames::BodyOrientation* moon_orientation = nullptr)` (with a Moon orientation the Moon is a reference body)
  - `ephemeris::Body FlightSession::reference_body(size_t index) const`

- [ ] **Step 1: Write the failing tests**

`core/tests/test_lunar_session.cpp`:
```cpp
// FlightSession near the Moon (lunar l1): ticks near the lunar ground, LVLH about the Moon.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/orbital_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

struct Fixture {
    Fixture() { test::spice_session(); }
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    environment::ReferenceBody moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
};

config::VehicleConfig ship_config() {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    return cfg;
}

}  // namespace

TEST_CASE("A session ticks a body falling towards the lunar ground instead of coasting through it", "[sim][lunar]") {
    Fixture fx;
    // At rest relative to the surface 3 km up (below the 5 km tick height): free fall, ~98 m/s at the ground.
    const auto frame = fx.moon.frame(fx.t0);
    const Vec3 fixed = frames::geodetic_to_fixed({20.0 * deg, 40.0 * deg, 3000.0}, fx.moon.shape());
    vehicle::RigidBodyState s{frame.position_gcrf(fixed), frame.velocity_gcrf(fixed, {}), {}, {}};
    fx.session.place_body(ship_config(), s, fx.t0);
    CHECK(fx.session.reference_body(0) == ephemeris::Body::Moon);
    fx.session.advance_to(fx.t0 + 120.0, 100000);
    CHECK(fx.session.body(0).impacted());
    CHECK(fx.session.telemetry(0).altitude_m > -2.0);  // stopped at the ground, not 100 km under it
}

TEST_CASE("LVLH hold in lunar orbit is about the Moon", "[sim][lunar]") {
    Fixture fx;
    const auto rel = orbit::state_from_elements({constants::moon_mean_radius_m + 100e3, 0.0, 90.0 * deg, 0.0, 0.0, 0.0},
                                                fx.moon.gm_m3ps2());
    const auto centre = fx.moon.centre_gcrf(fx.t0);
    const math::StateVector state{rel.position_m + centre.position_m, rel.velocity_mps + centre.velocity_mps};
    vehicle::RigidBodyState s{state.position_m, state.velocity_mps, gnc::lvlh_attitude_gcrf(rel), {}};
    fx.session.place_body(ship_config(), s, fx.t0);
    fx.session.set_attitude_mode(gnc::lvlh_hold());
    fx.session.advance_to(fx.t0 + 1200.0, 1000000);
    const auto& b = fx.session.body(0).state();
    const auto now = fx.moon.centre_gcrf(fx.session.epoch());
    const Vec3 v_moon = math::normalized(b.velocity_gcrf_mps - now.velocity_mps);
    const Vec3 nose = b.attitude.rotate(Vec3{1.0, 0.0, 0.0});
    // LVLH identity: nose along the Moon-relative velocity (an Earth-relative LVLH would be tens of degrees off).
    CHECK(std::acos(std::clamp(dot(nose, v_moon), -1.0, 1.0)) < 2.0 * deg);
    CHECK(fx.session.telemetry(0).reference_body == ephemeris::Body::Moon);
}

TEST_CASE("A session without a Moon orientation keeps the Earth as the only reference body", "[sim][lunar]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t0);
    vehicle::RigidBodyState s{moon.position_m + Vec3{2.0e6, 0.0, 0.0}, moon.velocity_mps, {}, {}};
    session.place_body(ship_config(), s, t0);
    CHECK(session.reference_body(0) == ephemeris::Body::Earth);
    CHECK(session.environment().moon == nullptr);
}
```
Add the file to `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Build to verify it fails**

Expected: compile errors (constructor has no 5th parameter; no `reference_body`).

- [ ] **Step 3: Implement**

`flight_session.hpp`:
- Constructor: `FlightSession(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& earth, std::filesystem::path config_dir, environment::PointMassGravityConfig gravity_config = {}, const frames::BodyOrientation* moon_orientation = nullptr);` with the comment "`moon_orientation` (MOON_ME): the Moon becomes a reference body (lunar l1: ground, air, telemetry and LVLH relative to the nearest body); without it everything is relative to the Earth."
- Public: `// Reference body of body `index` now (lunar l1).` `ephemeris::Body reference_body(size_t index) const;`
- Private, directly before `vehicle::FlightEnvironment env_;`: `std::optional<environment::ReferenceBody> moon_;` and `#include "starsim/environment/reference_body.hpp"`.

`flight_session.cpp`:
- Constructor init list: after `atmosphere_`, `moon_(moon_orientation ? std::optional<environment::ReferenceBody>(environment::ReferenceBody::moon(ephemeris, *moon_orientation)) : std::nullopt)`, then `env_{gravity_, atmosphere_, earth, &ephemeris, true, moon_ ? &*moon_ : nullptr}` (keep the member order of the header: `moon_` is declared before `env_`).
- `reference_body(index)`: `return bodies_.at(index).body->reference_body(env_, epoch_);`
- `coast_allowance_s`: replace the block
  ```cpp
  {
      const auto t = earth_.fixed_to_gcrf(epoch_);
      const double height = ... - body.ground_height_m();
      ...
      const math::Vec3 up = math::normalized(body.state().position_gcrf_m);
      const double sink = -dot(body.state().velocity_gcrf_mps, up);
      if (sink > 0.0) allowance = std::min(allowance, (height - tick_height) / sink);
  }
  ```
  by
  ```cpp
  {
      const auto clearance = body.ground_clearance(env_, epoch_);  // over the body's reference body (lunar l1)
      // a capsule ticks from above its parachute deployment heights (no coast past them)
      const double tick_height = ... (unchanged);
      if (clearance.height_m < tick_height) return 0.0;
      if (clearance.sink_mps > 0.0) allowance = std::min(allowance, (clearance.height_m - tick_height) / clearance.sink_mps);
  }
  ```
- Attitude modes (`if (attitude_mode_active())` block): build the target from the state about the reference body:
  ```cpp
  const auto ref = vehicle::reference_body_of(env_, flying.body->reference_body(env_, epoch_));
  const auto centred = ref.frame(epoch_).centred({s.position_gcrf_m, s.velocity_gcrf_mps});
  const auto target = gnc::attitude_target(mode, centred);
  ```
  (`Sun` still uses the GCRF position difference above; `Inertial` / `Direction` do not depend on the centre.) For the Earth `centred` equals the GCRF state exactly.

- [ ] **Step 4: Build and run**

Run: core build; `build/core/tests/starsim_core_tests.exe "[sim][lunar]"` then the determinism command of Task 2 Step 6 (diff against `l1_baseline.txt`) and `build/core/tests/starsim_core_tests.exe "[dragon]"` (LVLH and inertial holds of d2 run through the changed block).
Expected: 3 new test cases pass; `IDENTICAL`; `[dragon]` all pass (its printed hold figures - e.g. "LVLH trunk forward: <= 0.71 deg, 3.1 kg/h" - unchanged).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/tests/test_lunar_session.cpp core/tests/CMakeLists.txt
git commit -m "feat(sim): sessions take the Moon as a reference body for ground, ticks and LVLH"
```

---

### Task 4: Earth-centred propagation near the Moon against a Moon-centred reference

**Files:**
- Test: `core/tests/test_lunar_propagation.cpp` (new; add to `core/tests/CMakeLists.txt`)
- Modify (only if the 1 m criterion fails, see Step 3): `core/src/vehicle/flight_body.cpp` (`coast` tolerances)

**Interfaces:**
- Consumes: `FlightBody::coast/step` (Task 2), `environment::PointMassGravity` with `PointMassGravityConfig{.central_body = Moon}`, `integrators::integrate<6>`.
- Produces: the validation evidence for the spec's l1 criterion (no new API).

- [ ] **Step 1: Write the tests**

`core/tests/test_lunar_propagation.cpp`:
```cpp
// Lunar l1 validation: vehicle states stay Earth-centred GCRF near the Moon. A 100 km LLO and an NRHO-like orbit
// (3,000 x 70,000 km about the Moon) propagated one day by FlightBody::coast (Earth-centred, full model) agree with
// a Moon-centred propagation of the same forces (point masses Earth, Moon, Sun; no Earth J2, no SRP) within 1 m [E];
// 200 Hz ticks (RK4, as in a landing) over 20 min within 5 cm [E].

#include <cmath>
#include <cstdio>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/integrators/integrate.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

struct Fixture {
    Fixture() { test::spice_session(); }
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity earth_centred{ephemeris, earth, {ephemeris::Body::Earth, true, true, true, false}};
    environment::PointMassGravity moon_centred{ephemeris, earth, {ephemeris::Body::Moon, true, true, true, false}};
    environment::Us76Atmosphere atmosphere;
    environment::ReferenceBody moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    vehicle::FlightEnvironment env{earth_centred, atmosphere, earth, &ephemeris, false, &moon};
    time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
};

std::unique_ptr<vehicle::FlightBody> ship(const vehicle::RigidBodyState& state) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    return std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR), state);
}

// Moon-centred reference: DOP853 at tight tolerances on the Moon-centred point-mass model.
math::StateVector moon_centred_reference(const Fixture& fx, const math::StateVector& rel, double duration_s) {
    const integrators::OdeFunction<6> f = [&](double t, const math::OdeState6& y) {
        const Vec3 a = fx.moon_centred.acceleration_mps2({y[0], y[1], y[2]}, fx.t0 + t);
        return math::OdeState6{{y[3], y[4], y[5], a.x, a.y, a.z}};
    };
    integrators::Dop853Options<6> options;
    options.relative_tolerance = 1e-13;
    options.absolute_tolerance = math::OdeState6{{1e-6, 1e-6, 1e-6, 1e-9, 1e-9, 1e-9}};
    return math::unpack(integrators::integrate<6>(f, 0.0, math::pack(rel), duration_s, options).y);
}

double coast_error_m(Fixture& fx, const orbit::KeplerianElements& elements, double duration_s) {
    const auto rel = orbit::state_from_elements(elements, fx.moon.gm_m3ps2());
    const auto centre = fx.moon.centre_gcrf(fx.t0);
    auto body = ship({rel.position_m + centre.position_m, rel.velocity_mps + centre.velocity_mps, {}, {}});
    body->coast(fx.env, fx.t0, duration_s);
    const auto end_centre = fx.moon.centre_gcrf(fx.t0 + duration_s);
    const auto reference = moon_centred_reference(fx, rel, duration_s);
    return norm(body->state().position_gcrf_m - end_centre.position_m - reference.position_m);
}

}  // namespace

TEST_CASE("A low lunar orbit propagated Earth-centred matches the Moon-centred reference", "[lunar][validation]") {
    Fixture fx;
    const double error = coast_error_m(fx, {constants::moon_mean_radius_m + 100e3, 0.0, 90.0 * deg, 0.0, 0.0, 0.0},
                                       86400.0);
    std::printf("[validation] LLO 100 km, 1 day: Earth-centred vs Moon-centred %.3f m\n", error);
    CHECK(error < 1.0);
}

TEST_CASE("An NRHO-like orbit propagated Earth-centred matches the Moon-centred reference", "[lunar][validation]") {
    Fixture fx;
    const double rp = constants::moon_mean_radius_m + 3000e3, ra = 70000e3;
    const double error =
        coast_error_m(fx, {0.5 * (rp + ra), (ra - rp) / (ra + rp), 90.0 * deg, 0.0, 90.0 * deg, 0.0}, 86400.0);
    std::printf("[validation] NRHO-like 3,000 x 70,000 km, 1 day: Earth-centred vs Moon-centred %.3f m\n", error);
    CHECK(error < 1.0);
}

TEST_CASE("200 Hz ticks in a low lunar orbit match the Moon-centred reference", "[lunar][validation]") {
    Fixture fx;
    const auto rel = orbit::state_from_elements({constants::moon_mean_radius_m + 100e3, 0.0, 90.0 * deg, 0.0, 0.0, 0.0},
                                                fx.moon.gm_m3ps2());
    const auto centre = fx.moon.centre_gcrf(fx.t0);
    auto body = ship({rel.position_m + centre.position_m, rel.velocity_mps + centre.velocity_mps, {}, {}});
    const double duration = 1200.0;
    const int ticks = static_cast<int>(std::lround(duration / 0.005));
    for (int i = 0; i < ticks; ++i) body->step(fx.env, fx.t0 + i * 0.005, 0.005, {});
    const auto reference = moon_centred_reference(fx, rel, duration);
    const double error = norm(body->state().position_gcrf_m - fx.moon.centre_gcrf(fx.t0 + duration).position_m -
                              reference.position_m);
    std::printf("[validation] LLO 100 km, 20 min of 200 Hz ticks: Earth-centred vs Moon-centred %.4f m\n", error);
    CHECK(error < 0.05);
}
```
(The NRHO-like ellipse is not the Artemis NRHO - l3 builds that; here only the frame of integration is compared on an orbit reaching the same distances.)

- [ ] **Step 2: Build and run**

Run: core build; `build/core/tests/starsim_core_tests.exe "[lunar][validation]"`
Expected: 3 pass; printed errors < 1 m / < 1 m / < 5 cm.

- [ ] **Step 3: If a criterion fails**

If the coast errors exceed 1 m: tighten `FlightBody::coast` only when the body is near the Moon - set `options.absolute_tolerance` position entries to `1e-5` m (velocity `1e-8`) when `reference_ == ephemeris::Body::Moon`, keep the Earth values (bit-identical Earth path); re-run Step 2 and the determinism diff of Task 2 Step 6. If still > 1 m, do not loosen the criterion: ledger a ruling and stop to ask the user (the spec's fallback - Moon-relative integration inside the integrator - is an architecture decision). If the 200 Hz tick error exceeds 5 cm, record the figure, and check the timestep logic of the test loop before anything else.

- [ ] **Step 4: Commit**

```bash
git add core/tests/test_lunar_propagation.cpp core/tests/CMakeLists.txt  # plus flight_body.cpp if Step 3 changed it
git commit -m "test(vehicle): Earth-centred propagation near the Moon matches a Moon-centred reference"
```

---

### Task 5: Documentation and the full suite

**Files:**
- Modify: `CLAUDE.md` (Architecture module list: `environment` now with reference bodies; a "Lunar missions (l1)" note under Game/structure notes; Known issues: ground height at the Moon = sphere until l2)
- Modify: `docs/superpowers/plans/2026-10-14-lunar-l1-reference-body.md` (nothing beyond checkboxes)

**Interfaces:**
- Consumes: everything above.
- Produces: docs only.

- [ ] **Step 1: Update CLAUDE.md**

In "Architecture (non-negotiable)", the implemented-modules sentence: after "environment (gravity)" insert ", reference bodies (Earth, Moon: body-fixed frame, shape, atmosphere or vacuum)". Add a bullet at the end of "Game (Godot) structure" (where the milestone notes live):
```markdown
- Lunar missions (spec `docs/superpowers/specs/2026-10-14-lunar-missions-design.md`), l1 reference bodies: vehicle states
  stay Earth-centred GCRF everywhere; `environment::ReferenceBody` (Earth: ITRF93 + WGS84 + US76; Moon: MOON_ME, sphere
  1737.4 km, vacuum) gives what is relative to a body. A `FlightBody` takes the Moon within 60,000 km of it and the Earth
  again beyond 100,000 km [E] (the NRHO's apolune stays inside); bodies on the ground keep theirs. Air, ground contact,
  resting (it follows the Moon), telemetry (`reference_body`, altitude, osculating periapsis / apoapsis and inclination to
  the body's equator) and the session's tick-near-ground decision and LVLH holds use it. `FlightSession` takes a Moon
  orientation (5th constructor argument, `spice::SpiceOrientation::moon_mean_earth()`); without it (bindings until l6)
  everything is relative to the Earth as before. Ground at the Moon: the sphere (LOLA tile: l2). Validation: 100 km LLO
  and a 3,000 x 70,000 km orbit one day Earth-centred vs Moon-centred <figures from Task 4> m; 200 Hz ticks 20 min <figure> m.
```
Replace `<figures from Task 4>` and `<figure>` with the printed values of Task 4.

- [ ] **Step 2: Full suite**

Run (background, ~6 min):
```bash
"/c/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/ctest.exe" --test-dir build -j 12 > C:/Users/johan/.claude/jobs/78816a1a/tmp/l1_ctest.txt 2>&1; tail -5 C:/Users/johan/.claude/jobs/78816a1a/tmp/l1_ctest.txt
```
Expected: `100% tests passed` (430 + the new cases).

- [ ] **Step 3: Build with bindings and smoke test**

Run: `cmd //c "C:\\Users\\johan\\.claude\\jobs\\78816a1a\\tmp\\build_noscan.cmd"` then `sh C:/Users/johan/.claude/jobs/78816a1a/tmp/smoke.sh`
Expected: `BUILD_OK`; smoke test passes (bindings unchanged, `FlightEnvironment` aggregate and session constructor stay compatible).

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: lunar l1 reference bodies"
```
