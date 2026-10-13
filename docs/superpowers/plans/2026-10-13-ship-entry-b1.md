# Ship Entry Aerodynamics, Flaps and Corridor (b1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The ship flies a physically modelled belly-first entry (calibrated hull cross-flow, four flaps, flap+RCS
attitude control, heating and load limits) that reproduces the entries of flights 12-14.

**Architecture:** The hull stays `aero::SlenderBodyAero` with an optional per-vehicle cross-flow table. Flaps are a
new `aero/flap` module (Newtonian flat plates at their own positions), carried by `vehicle::Stage` like the grid
fins and actuated by `Vehicle::update_flaps`. `gnc/entry_attitude` turns (alpha, bank) into an attitude setpoint;
`AttitudeController` trims and steers with the flaps (Newton step on the modelled aero moment) and RCS.
`aero/entry_heating` gives Sutton-Graves heat rate; `FlightBody` integrates heat load and judges the limits.

**Tech Stack:** C++20 core (Catch2 tests), TOML/CSV config, Python telemetry tools.

**Spec:** `docs/superpowers/specs/2026-10-13-ship-return-design.md` (section "b1 design").

## Global Constraints

- SI units internally, angles in radians; frame and unit in names (`air_velocity_body_mps`).
- No magic numbers: physical constants in `core/include/starsim/math/constants.hpp` with source; vehicle values in
  TOML with provenance tags [S]/[E]/[D].
- `core` has no Godot dependency; modules depend downwards only (`aero` must not include `gnc`/`sim`).
- UI strings in the core are German; code, comments, commits English; Conventional Commits with the session
  attribution lines.
- Every physical model: unit tests + at least one validation case with a documented tolerance.
- Build: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`; single test:
  `build/core/tests/starsim_core_tests.exe "<name>"`; full suite: ctest (`--test-dir build -j 12`).
- Existing behaviour must not change for vehicles without flaps / cross-flow table (booster return, campaign
  21/21, ascent tests): the full suite stays green after every task.

## Review Focus

1. Ship nose-first during ascent/coast with flaps stowed: flaps must give (almost) no force and the controller must
   not use them unless `flaps` is enabled - test in Task 4 ("stowed flaps in nose-first flow give no force").
2. Flow from the leeward side (ship flying back-first, e.g. tumbling): flaps in the hull's lee carry no force, no
   NaN - test in Task 3.
3. Vacuum / rarefied flow (above 100 km): flap and heating code with density 0 return zeros, no division by zero -
   tests in Tasks 3 and 5.
4. Flap allocation at saturation (all flaps at a limit): output stays inside [0, max], remainder goes to RCS - test
   in Task 6.
5. Skip-out: a ship that leaves the atmosphere again must coast on (no breakup, event reported once) - test in
   Task 7.

---

### Task 1: Entry telemetry estimate tool

**Files:**
- Create: `tools/telemetry/entry_estimate.py`
- Modify: `tests/validation/telemetry/README.md` (usage line)

**Interfaces:**
- Produces: CLI `python tools/telemetry/entry_estimate.py <table.csv> --from T --to T [--step 40]
  [--latitude-deg 10]` printing t, h, v, Mach, D/m, vertical L/m, vertical L/D, beta.

- [ ] **Step 1: Write the tool** (from the analysis done in the brainstorming, `drag_estimate` helpers)

```python
"""Drag and lift of the entering ship from webcast telemetry (tables written by webcast_telemetry.py).

Along the path D/m = -dv/dt - g sin(gamma). Across it, in the vertical plane, the lift component follows from the
curvature of the altitude: L_v/m = v dgamma/dt + (g - v^2/r) cos(gamma) - 2 Omega v cos(lat) (Coriolis of
eastward flight; the displayed speed is relative to the rotating Earth). Bank is unknown, so L_v/D is a lower
bound of the vehicle's L/D.

  python tools/telemetry/entry_estimate.py <table.csv> --from <t_s> --to <t_s> [--step 40] [--latitude-deg 10]

Speed slope over +-3 s, altitude parabola over +-20 s (0.1 km dials). Air data: US76 (drag_estimate.atmosphere).
"""
import argparse
import math

from drag_estimate import atmosphere, load, quadratic, slope

EARTH_ROTATION_RADPS = 7.2921159e-5  # IERS
EARTH_RADIUS_M = 6371.0e3            # mean radius (local curvature of the path)
G0 = 9.80665


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("table")
    p.add_argument("--from", dest="t_from", type=float, required=True)
    p.add_argument("--to", dest="t_to", type=float, required=True)
    p.add_argument("--step", type=float, default=40.0)
    p.add_argument("--latitude-deg", type=float, default=10.0, help="mean latitude of the entry ground track")
    a = p.parse_args()
    speed, alt = load(a.table, "ship")
    st, sv = zip(*speed)
    at, ah = zip(*alt)
    print("  t[s]  h[km]  v[m/s]  Mach   D/m   Lv/m  Lv/D  beta[kg/m2]")
    t = a.t_from
    while t <= a.t_to:
        dv, v = slope(st, sv, t, 3.0)
        fit = quadratic(at, ah, t, 20.0)
        if dv is not None and fit is not None and v > 50.0:
            h, hd, hdd = fit
            g = G0 * (EARTH_RADIUS_M / (EARTH_RADIUS_M + h)) ** 2
            sin_g = max(-1.0, min(1.0, hd / v))
            cos_g = math.sqrt(1.0 - sin_g * sin_g)
            drag = -dv - g * sin_g
            gamma_rate = (hdd - dv * sin_g) / (v * cos_g)
            lift_v = (v * gamma_rate + (g - v * v / (EARTH_RADIUS_M + h)) * cos_g
                      - 2.0 * EARTH_ROTATION_RADPS * v * math.cos(math.radians(a.latitude_deg)))
            rho, sound = atmosphere(max(h, 0.0))
            ok = drag > 0.3
            print(f"{t:7.1f} {h / 1e3:5.1f} {v:7.0f} {v / sound:5.1f} {drag:6.2f} {lift_v:6.2f} "
                  f"{lift_v / drag if ok else float('nan'):5.2f} {0.5 * rho * v * v / drag if ok else float('nan'):8.0f}")
        t += a.step


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run it on flight 13**

Run: `cd tools/telemetry && python entry_estimate.py ../../tests/validation/telemetry/flight13_2026-07-24.csv --from 2800 --to 3760`
Expected: rows with beta 500-730 and Lv/D 0.27-0.62 between 2920 and 3720 s (as in the spec table).

- [ ] **Step 3: README usage line and commit**

Add under the drag usage in `tests/validation/telemetry/README.md`:
"Ship entry (beta and vertical L/D): `python tools/telemetry/entry_estimate.py <table.csv> --from 2800 --to 3760`."

```bash
git add tools/telemetry/entry_estimate.py tests/validation/telemetry/README.md
git commit -m "feat: ship entry drag and lift estimate from webcast telemetry"
```

### Task 2: Hull cross-flow table

**Files:**
- Modify: `core/include/starsim/aero/slender_body_aero.hpp`, `core/src/aero/slender_body_aero.cpp`
- Modify: `core/include/starsim/config/vehicle_config.hpp` (`AeroConfig`: replace `entry_table` by
  `crossflow_table`), `core/src/config/vehicle_config.cpp:112`
- Modify: `core/src/vehicle/vehicle.cpp` (`rebuild_aero`)
- Create: `config/aero/ship_crossflow.csv`; Delete: `config/aero/ship_entry.csv`
- Modify: `config/vehicles/starship_block3.toml` (ship `[stage.aero]`)
- Test: `core/tests/test_slender_body_aero.cpp` (existing file of the aero tests - find with
  `grep -l SlenderBodyAero core/tests/*.cpp`)

**Interfaces:**
- Produces: `aero::CrossflowCurve` (`explicit CrossflowCurve(const config::CsvTable&)`, columns `mach,
  crossflow_cd`; `double cd(double mach) const`; `bool empty() const`); `SlenderBodyAero(geometry, drag,
  tail_first_drag = {}, CrossflowCurve crossflow = {})`; `AeroConfig::crossflow_table` (path, empty = generic).

- [ ] **Step 1: Failing test**

```cpp
TEST_CASE("A cross-flow table replaces the generic cross-flow coefficient", "[aero]") {
    config::CsvTable t;
    t.columns = {"mach", "crossflow_cd"};
    t.rows = {{0.0, 1.0}, {10.0, 1.5}};
    const aero::SlenderBodyGeometry g{50.0, 9.0, 0.0};  // flat top: no potential term
    const aero::SlenderBodyAero generic(g, {});
    const aero::SlenderBodyAero tabled(g, {}, {}, aero::CrossflowCurve(t));
    const double rho = 1e-3, sound = 300.0;
    const math::Vec3 broadside{0.0, 0.0, -3000.0};  // Mach 10, alpha 90 deg
    const auto a = generic.compute(broadside, rho, sound, false);
    const auto b = tabled.compute(broadside, rho, sound, false);
    const double q = 0.5 * rho * 3000.0 * 3000.0;
    CHECK(b.force_body_n.z == Approx(1.5 * 50.0 * 9.0 * q).epsilon(1e-9));
    CHECK(a.force_body_n.z == Approx(0.68 * 1.2 * 50.0 * 9.0 * q).epsilon(1e-9));
}
```

- [ ] **Step 2: Run** `build/core/tests/starsim_core_tests.exe "A cross-flow table replaces the generic cross-flow coefficient"` - Expected: compile error (`CrossflowCurve` unknown).

- [ ] **Step 3: Implement**

Header (next to `AxialDragCurve`):
```cpp
// Cross-flow drag coefficient eta * C_dc versus Mach (Jorgensen viscous term), linear, clamped at the ends.
class CrossflowCurve {
public:
    CrossflowCurve() = default;
    explicit CrossflowCurve(const config::CsvTable& table);  // columns: mach, crossflow_cd
    double cd(double mach) const;
    bool empty() const { return mach_.empty(); }
private:
    std::vector<double> mach_;
    std::vector<double> cd_;
};
```
`SlenderBodyAero` stores `CrossflowCurve crossflow_;` (constructor parameter last, default `{}`). In `compute`:
```cpp
const double continuum_crossflow =
    crossflow_.empty() ? crossflow_efficiency * crossflow_drag_coefficient : crossflow_.cd(result.mach);
const double crossflow = (1.0 - w) * continuum_crossflow + w * free_molecular_drag_coefficient;
```
`CrossflowCurve` constructor validates ascending Mach with >= 2 rows (same error text style as `AxialDragCurve`),
`cd` uses the file's existing `interpolate`.
`rebuild_aero`: cross-flow table of the topmost stage that has one **only when that stage is alone** (the stack
keeps the generic value: `stages_.size() == 1`).
`vehicle_config.cpp`: `s.aero.crossflow_table = doc.text_or(*aero, "crossflow_table");` (remove `entry_table`).

`config/aero/ship_crossflow.csv` (initial, calibrated in Task 8):
```
# Ship hull cross-flow coefficient eta * C_dc (Jorgensen viscous term, planform = length x diameter).
# [E] initial: modified Newtonian cylinder 1.23 hypersonic, ~1.2 subsonic (Re-supercritical cylinder ~0.7 x eta 1);
# calibrated [D] against flights 12-14 entries (tests/validation/telemetry, Task 8 of the b1 plan).
mach,crossflow_cd
0.0,1.0
0.8,1.1
1.2,1.4
3.0,1.3
10.0,1.25
30.0,1.23
```
Ship TOML `[stage.aero]`: `crossflow_table = "aero/ship_crossflow.csv"`, remove `entry_table`.

- [ ] **Step 4: Run test + aero tests** `starsim_core_tests.exe "[aero]"` - Expected: all pass.
- [ ] **Step 5: Full suite** (ctest) - Expected: all pass (ship alone now uses the table; no test depends on the
  ship's broadside drag yet - if one does, ledger a ruling).
- [ ] **Step 6: Commit** `feat: per-vehicle hull cross-flow table; ship placeholder entry table removed`

### Task 3: Flap aerodynamics module

**Files:**
- Create: `core/include/starsim/aero/flap.hpp`, `core/src/aero/flap.cpp`, `config/aero/flap.csv`
- Modify: `core/CMakeLists.txt` (add `src/aero/flap.cpp`), `core/tests/CMakeLists.txt` (add `test_flaps.cpp`)
- Test: `core/tests/test_flaps.cpp`

**Interfaces:**
- Produces:
```cpp
namespace starsim::aero {
class FlapTable { public: FlapTable() = default; explicit FlapTable(const config::CsvTable&);  // mach, cn_max
                  double cn_max(double mach) const; bool empty() const; };
// Flat plate hinged along a line parallel to the body axis at the hull (radius r_hull), hinge azimuth phi
// (from body +y towards +z). u_stow: hull tangent at the hinge pointing away from the belly (leeward);
// e_r: radial. Plane spanned by e_x and s(d) = cos(d) u_stow + sin(d) e_r; normal n = e_x x s.
struct FlapGeometry {
    double station_fwd_m = 0.0, station_aft_m = 0.0;  // body frame
    double hinge_azimuth_rad = 0.0;
    double hull_radius_m = 0.0;
    double span_m = 0.0;
    double area_m2 = 0.0;
    double belly_azimuth_rad = 0.0;  // body direction of the heat shield (windward side)
};
math::Vec3 flap_span_direction(const FlapGeometry& f, double deflection_rad);  // s(d)
math::Vec3 flap_normal(const FlapGeometry& f, double deflection_rad);          // e_x x s
math::Vec3 flap_centroid(const FlapGeometry& f, double deflection_rad);        // body frame (station, y, z)
struct FlapForce { math::Vec3 force_body_n; math::Vec3 application_point_m; };
// velocity: flap centroid relative to the air (body axes, incl. omega x r). Force only on the windward side:
// the centroid moves towards the belly (v . b > 0); |C_N| = cn_max(M) sin^2(theta), sin(theta) = |v . n| / |v|,
// along -sign(v . n) n, times (1 - rarefaction) plus rarefaction x 2.2 sin^2 (free-molecular, diffuse).
FlapForce flap_force(const FlapGeometry& f, double deflection_rad, const math::Vec3& velocity_body_mps,
                     const FinAirData& air, const FlapTable& table);
}
```

- [ ] **Step 1: Failing tests** (`test_flaps.cpp`)

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include "starsim/aero/flap.hpp"
using namespace starsim;
using Catch::Approx;

namespace {
aero::FlapTable table() {
    config::CsvTable t; t.columns = {"mach", "cn_max"}; t.rows = {{0.0, 1.2}, {5.0, 1.84}};
    return aero::FlapTable(t);
}
// Belly towards body -z (azimuth 270 deg), flap hinged on the +y side.
aero::FlapGeometry flap() {
    aero::FlapGeometry f;
    f.station_fwd_m = 14.0; f.station_aft_m = 2.0; f.hinge_azimuth_rad = 0.0; f.hull_radius_m = 4.5;
    f.span_m = 4.0; f.area_m2 = 48.0; f.belly_azimuth_rad = 1.5 * M_PI;
    return f;
}
const aero::FinAirData air{1e-3, 300.0, 0.0};
}  // namespace

TEST_CASE("Flap geometry: stowed edge-on to a belly-first flow, extended broadside", "[aero][flap]") {
    const auto f = flap();
    // stowed: span direction leeward (+z, away from the belly at -z)
    const auto s0 = aero::flap_span_direction(f, 0.0);
    CHECK(s0.z == Approx(1.0).margin(1e-12));
    const auto s90 = aero::flap_span_direction(f, M_PI / 2);
    CHECK(s90.y == Approx(1.0).margin(1e-12));  // radial at the +y hinge
    CHECK(std::abs(aero::flap_normal(f, M_PI / 2).z) == Approx(1.0).margin(1e-12));
    const auto c = aero::flap_centroid(f, M_PI / 2);
    CHECK(c.x == Approx(8.0)); CHECK(c.y == Approx(6.5)); CHECK(c.z == Approx(0.0).margin(1e-12));
}

TEST_CASE("Extended flap in a belly-first flow carries Newtonian drag", "[aero][flap]") {
    const math::Vec3 v{0.0, 0.0, -1500.0};  // moving towards the belly (-z), Mach 5
    const auto r = aero::flap_force(flap(), M_PI / 2, v, air, table());
    const double q = 0.5 * 1e-3 * 1500.0 * 1500.0;
    CHECK(r.force_body_n.z == Approx(1.84 * 48.0 * q));  // opposes the motion
    CHECK(std::abs(r.force_body_n.x) < 1e-6);
    const auto stowed = aero::flap_force(flap(), 0.0, v, air, table());
    CHECK(norm(stowed.force_body_n) < 1e-6 * q);
    const auto half = aero::flap_force(flap(), M_PI / 4, v, air, table());
    CHECK(norm(half.force_body_n) == Approx(1.84 * 48.0 * q * 0.5));
}

TEST_CASE("Flaps in the hull's lee or in vacuum carry no force", "[aero][flap]") {
    const math::Vec3 back_first{0.0, 0.0, 1500.0};  // moving leeward
    CHECK(norm(aero::flap_force(flap(), M_PI / 2, back_first, air, table()).force_body_n) == 0.0);
    const aero::FinAirData vacuum{0.0, 0.0, 1.0};
    const auto r = aero::flap_force(flap(), M_PI / 2, math::Vec3{0, 0, -7000.0}, vacuum, table());
    CHECK(norm(r.force_body_n) == 0.0);
    CHECK(std::isfinite(r.application_point_m.x));
}

TEST_CASE("Stowed flaps in nose-first flow give no force", "[aero][flap]") {
    const math::Vec3 nose_first{-2000.0, 0.0, 0.0};  // flow along the axis: v . b = 0
    CHECK(norm(aero::flap_force(flap(), 0.0, nose_first, air, table()).force_body_n) == 0.0);
}
```

- [ ] **Step 2: Run** `starsim_core_tests.exe "[flap]"` - Expected: compile error (header missing).
- [ ] **Step 3: Implement** `flap.cpp`:

```cpp
#include "starsim/aero/flap.hpp"
#include <cmath>
#include "starsim/config/config_error.hpp"   // use the include that AxialDragCurve's ConfigError comes from

namespace starsim::aero {
using math::Vec3;
namespace {
constexpr double free_molecular_normal_coefficient = 2.2;  // [E] diffuse reflection, as SlenderBodyAero
Vec3 radial(double azimuth) { return {0.0, std::cos(azimuth), std::sin(azimuth)}; }
double interpolate(const std::vector<double>& x, const std::vector<double>& y, double v) {
    if (v <= x.front()) return y.front();
    if (v >= x.back()) return y.back();
    size_t i = 1;
    while (x[i] < v) ++i;
    const double t = (v - x[i - 1]) / (x[i] - x[i - 1]);
    return y[i - 1] + t * (y[i] - y[i - 1]);
}
}  // namespace

FlapTable::FlapTable(const config::CsvTable& table) : mach_(table.column("mach")), cn_max_(table.column("cn_max")) {
    if (mach_.size() < 2) throw config::ConfigError("flap table: needs >= 2 rows");
    for (size_t i = 1; i < mach_.size(); ++i)
        if (mach_[i] <= mach_[i - 1]) throw config::ConfigError("flap table: mach column must be ascending");
}
double FlapTable::cn_max(double mach) const { return empty() ? 0.0 : interpolate(mach_, cn_max_, mach); }

Vec3 flap_span_direction(const FlapGeometry& f, double d) {
    const Vec3 e_r = radial(f.hinge_azimuth_rad);
    const Vec3 belly = radial(f.belly_azimuth_rad);
    // tangent at the hinge, pointing away from the belly
    Vec3 tangent = cross(Vec3{1.0, 0.0, 0.0}, e_r);
    if (dot(tangent, belly) > 0.0) tangent = -tangent;
    return tangent * std::cos(d) + e_r * std::sin(d);
}
Vec3 flap_normal(const FlapGeometry& f, double d) { return cross(Vec3{1.0, 0.0, 0.0}, flap_span_direction(f, d)); }
Vec3 flap_centroid(const FlapGeometry& f, double d) {
    return Vec3{0.5 * (f.station_fwd_m + f.station_aft_m), 0.0, 0.0} + radial(f.hinge_azimuth_rad) * f.hull_radius_m +
           flap_span_direction(f, d) * (0.5 * f.span_m);
}

FlapForce flap_force(const FlapGeometry& f, double d, const Vec3& v, const FinAirData& air, const FlapTable& table) {
    FlapForce out;
    out.application_point_m = flap_centroid(f, d);
    const double speed = norm(v);
    if (speed < 1e-6 || air.density_kg_m3 <= 0.0) return out;
    if (dot(v, radial(f.belly_azimuth_rad)) <= 0.0) return out;  // leeward: shielded by the hull
    const Vec3 n = flap_normal(f, d);
    const double vn = dot(v, n);
    const double sin_theta = std::abs(vn) / speed;
    const double mach = air.speed_of_sound_mps > 0.0 ? speed / air.speed_of_sound_mps : 0.0;
    const double cn = ((1.0 - air.rarefaction) * table.cn_max(mach) + air.rarefaction * free_molecular_normal_coefficient) *
                      sin_theta * sin_theta;
    const double q = 0.5 * air.density_kg_m3 * speed * speed;
    out.force_body_n = n * (-std::copysign(cn * q * f.area_m2, vn));
    return out;
}
}  // namespace starsim::aero
```
(Private members of `FlapTable`: `std::vector<double> mach_, cn_max_;`. Check the ConfigError header name used by
`slender_body_aero.cpp` and use the same include.)

`config/aero/flap.csv`:
```
# Ship flap normal force, reference = flap area: |C_N| = cn_max sin^2(incidence) (Newtonian flat plate).
# [E] 1.84 hypersonic (modified Newtonian stagnation pressure coefficient, gamma 1.2 equilibrium air ~1.9);
# ~1.2 subsonic (flat plate broadside, Hoerner); ~1.6 low supersonic.
mach,cn_max
0.0,1.20
0.8,1.30
1.2,1.60
3.0,1.75
6.0,1.84
30.0,1.84
```

- [ ] **Step 4: Run** `starsim_core_tests.exe "[flap]"` - Expected: 4 test cases pass.
- [ ] **Step 5: Commit** `feat: Newtonian flap aerodynamics (aero/flap)`

### Task 4: Flaps on the vehicle (config, actuators, forces) and ship geometry

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp` (`FlapConfig`, `StageConfig::flaps`,
  `AeroConfig::flap_table`, `AeroConfig::belly_azimuth_rad`), `core/src/config/vehicle_config.cpp`
- Modify: `core/include/starsim/vehicle/vehicle.hpp` (`Flap`, `Stage::flaps`, `Stage::flap_table`,
  `VehicleCommand::flap_rad`, `Vehicle::update_flaps`, `Vehicle::flap_count`), `core/src/vehicle/vehicle.cpp`
  (build in the constructor like grid fins at `vehicle.cpp:77`, station shift at `:348`)
- Modify: `core/src/vehicle/flight_body.cpp` (`evaluate`: flap forces after the grid fins; `step`: call
  `update_flaps` next to `update_grid_fins` at `:238`)
- Modify: `config/vehicles/starship_block3.toml` (ship flaps, belly azimuth, landing propellant)
- Test: `core/tests/test_flaps.cpp` (append), `core/tests/test_vehicle_config.cpp` (find the config test file
  with `grep -l load_vehicle core/tests/*.cpp`)

**Interfaces:**
- Consumes: Task 3 (`FlapGeometry`, `FlapTable`, `flap_force`).
- Produces: `config::FlapConfig { std::string id; double hinge_azimuth_rad, station_fwd_m, station_aft_m, span_m,
  area_m2, max_deflection_rad, rate_rad_s, initial_deflection_rad; }`; `StageConfig::flaps`
  (`std::vector<FlapConfig>`); `vehicle::Flap { aero::FlapGeometry geometry; double max_deflection_rad,
  rate_rad_s, deflection_rad; }`; `VehicleCommand::flap_rad` (`std::vector<double>`, missing entries = hold the
  current deflection); `Vehicle::update_flaps(double dt_s, const VehicleCommand&)`; `Vehicle::flap_count()`.

- [ ] **Step 1: Failing tests** (append to `test_flaps.cpp`; uses the real config)

```cpp
#include "starsim/config/vehicle_config.hpp"
#include "starsim/vehicle/vehicle.hpp"
// config dir helper: use the one the other vehicle tests use (grep "config_dir" core/tests/test_vehicle*.cpp)

TEST_CASE("The block 3 ship has four flaps with rate-limited actuators", "[flap][vehicle]") {
    const auto cfg = config::load_vehicle(test_config_dir(), "starship_block3");
    const auto& ship = cfg.stages.back();
    REQUIRE(ship.flaps.size() == 4);
    vehicle::Vehicle v(cfg, test_config_dir());
    REQUIRE(v.flap_count() == 4);
    vehicle::VehicleCommand c;
    c.flap_rad = {1.5, 1.5, 1.5, 1.5};
    const double before = v.stages().back().flaps[0].deflection_rad;
    v.update_flaps(0.1, c);
    const auto& f = v.stages().back().flaps[0];
    CHECK(f.deflection_rad - before == Approx(f.rate_rad_s * 0.1));
    for (int i = 0; i < 200; ++i) v.update_flaps(0.1, c);
    CHECK(f.deflection_rad == Approx(std::min(1.5, f.max_deflection_rad)));
}
```
And a `FlightBody` test: a ship alone (stack separated or vehicle built from the ship stage only - use the same
helper the booster-alone tests use: `grep -n "separate\|ship_only\|upper_stage" core/tests/test_flight_body*.cpp`)
at 60 km, belly-first (body -z along the velocity), flaps fully extended vs stowed: the sensed deceleration
with extended flaps is larger by sum(cn_max * area) * q / m within 5 %.

- [ ] **Step 2: Run** - Expected: compile errors (`flaps` member missing).
- [ ] **Step 3: Implement**

Config parsing (TOML array of tables `[[stage.flap]]`):
```cpp
if (auto flaps = t["flap"].as_array()) {
    for (auto& node : *flaps) {
        auto* ft = node.as_table();
        if (!ft) doc.fail("stage.flap entries must be tables");
        config::FlapConfig f;
        f.id = doc.text(*ft, "id");
        f.hinge_azimuth_rad = doc.number(*ft, "hinge_azimuth_deg") * constants::deg_to_rad;
        f.station_fwd_m = doc.number(*ft, "station_fwd_m");
        f.station_aft_m = doc.number(*ft, "station_aft_m");
        f.span_m = doc.number(*ft, "span_m");
        f.area_m2 = doc.number(*ft, "area_m2");
        f.max_deflection_rad = doc.number(*ft, "max_deflection_deg") * constants::deg_to_rad;
        f.rate_rad_s = doc.number(*ft, "rate_deg_s") * constants::deg_to_rad;
        f.initial_deflection_rad = doc.number_or(*ft, "initial_deflection_deg", 0.0) * constants::deg_to_rad;
        s.flaps.push_back(f);
    }
}
```
(`[stage.aero]`: `flap_table` text_or, `belly_azimuth_deg` number_or 270.) Use the doc helpers that exist
(`doc.number_or` - if absent, follow the pattern of other optional numbers in the file).

Vehicle constructor: like grid fins - `stage.flap_table = make_shared<aero::FlapTable>(load_csv_table(...))`;
`Flap{ {station + fwd, station + aft, hinge_az, 0.5 * diameter, span, area, belly_az}, max, rate, initial }`.
Station shift in the separation code (`vehicle.cpp:348`) shifts `station_fwd_m` and `station_aft_m`.
`update_flaps`: per flap, target = command entry or current; move by at most rate*dt; clamp [0, max].
FlightBody `evaluate` (after the grid fin loop):
```cpp
for (const auto& stage : ctx.vehicle.stages()) {
    if (!stage.flap_table) continue;
    for (const auto& flap : stage.flaps) {
        const Vec3 arm = aero::flap_centroid(flap.geometry, flap.deflection_rad) - Vec3{f.mass.cg_station_m, 0.0, 0.0};
        const Vec3 v = air_velocity_body + cross(s.angular_velocity_body_radps, arm);
        const auto force = aero::flap_force(flap.geometry, flap.deflection_rad, v, fin_air, *stage.flap_table);
        f.nongravitational_body_n += force.force_body_n;
        f.moment_body_nm += cross(arm, force.force_body_n);
    }
}
```

Ship TOML (after `[stage.aero]`; `belly_azimuth_deg = 270.0` and `flap_table = "aero/flap.csv"` go into
`[stage.aero]`):
```toml
# Flaps [E]: Block 3 aft flaps ~ 12 m x 4 m, forward flaps smaller and moved leeward (Block 2 on); hinges on
# the hull sides (belly at body -z). 0 deg = folded leeward (edge-on to a belly-first flow), 90 deg = in the
# belly plane (broadside). Rates [E] electric actuators (Tesla motors) ~ 20 deg/s.
[[stage.flap]]
id = "aft_right"
hinge_azimuth_deg = 0.0
station_fwd_m = 14.0
station_aft_m = 2.0
span_m = 4.0
area_m2 = 48.0
max_deflection_deg = 90.0
rate_deg_s = 20.0
initial_deflection_deg = 0.0

[[stage.flap]]
id = "aft_left"
hinge_azimuth_deg = 180.0
station_fwd_m = 14.0
station_aft_m = 2.0
span_m = 4.0
area_m2 = 48.0
max_deflection_deg = 90.0
rate_deg_s = 20.0
initial_deflection_deg = 0.0

[[stage.flap]]
id = "forward_right"
hinge_azimuth_deg = 20.0                 # moved leeward (Block 2 on)
station_fwd_m = 46.0
station_aft_m = 38.0
span_m = 3.0
area_m2 = 24.0
max_deflection_deg = 90.0
rate_deg_s = 20.0
initial_deflection_deg = 0.0

[[stage.flap]]
id = "forward_left"
hinge_azimuth_deg = 160.0
station_fwd_m = 46.0
station_aft_m = 38.0
span_m = 3.0
area_m2 = 24.0
max_deflection_deg = 90.0
rate_deg_s = 20.0
initial_deflection_deg = 0.0
```
Ship `[stage.recovery]`: `landing_propellant_kg = 25000.0  # [E] header tanks for flip and landing burn`.

- [ ] **Step 4: Run** `"[flap]"` and the config tests - Expected: pass.
- [ ] **Step 5: Full suite** - Expected: green (flaps stowed, so the ascent is unchanged).
- [ ] **Step 6: Commit** `feat: ship flaps - config, actuators and forces in flight`

### Task 5: Entry heating and load limits

**Files:**
- Create: `core/include/starsim/aero/entry_heating.hpp`, `core/src/aero/entry_heating.cpp`
- Modify: `core/include/starsim/math/constants.hpp` (`sutton_graves_earth = 1.7415e-4` with source)
- Modify: config (`HeatShieldConfig` in `StageConfig`: `nose_radius_m`, `max_heat_rate_w_m2`,
  `max_heat_load_j_m2`, `max_load_g`; `[stage.heat_shield]` in the ship TOML), `vehicle_config.cpp`
- Modify: `FlightBody` (heat rate, heat load, peak; `FlightTelemetry::heat_rate_w_m2`, `heat_load_j_m2`;
  breakup when a limit is exceeded: `breakup_reason()` (German text) and `impacted() == true`)
- Modify: `core/src/sim/flight_session.cpp` - forward the breakup message where impacts are reported
  (find with `grep -n "impacted()" core/src/sim/flight_session.cpp`)
- Test: `core/tests/test_entry_heating.cpp` (+ CMake)

**Interfaces:**
- Produces: `double aero::stagnation_heat_rate_w_m2(double density_kg_m3, double speed_mps, double nose_radius_m)`;
  `config::HeatShieldConfig`; `FlightBody::heat_load_j_m2()`, `peak_heat_rate_w_m2()`, `breakup_reason()`
  (`std::string`, empty while intact); `FlightBody::heat_warning()` (>= 80 % of a limit).

- [ ] **Step 1: Failing tests**

```cpp
TEST_CASE("Sutton-Graves heat rate", "[heating]") {
    // q = k sqrt(rho / R_n) v^3: rho 1e-4, R_n 1 m, v 7000 m/s -> 1.7415e-4 * 0.01 * 3.43e11 = 597.3 kW/m^2
    CHECK(aero::stagnation_heat_rate_w_m2(1e-4, 7000.0, 1.0) == Approx(5.973e5).epsilon(1e-3));
    CHECK(aero::stagnation_heat_rate_w_m2(0.0, 7000.0, 1.0) == 0.0);
    CHECK(aero::stagnation_heat_rate_w_m2(1e-4, 7000.0, 0.0) == 0.0);  // undefined nose: no heating model
}
```
A FlightBody test: a ship (helper as in Task 4) flown 30 s through dense air at 7 km/s (place it at 50 km,
horizontal) breaks up with reason "Hitzeschild versagt"; with `max_heat_rate` raised it survives and
`heat_load_j_m2()` equals the integral of the rate within 1 %.

- [ ] **Step 2: Run** - Expected: compile error.
- [ ] **Step 3: Implement** (`entry_heating.cpp`):
```cpp
double stagnation_heat_rate_w_m2(double rho, double v, double r_n) {
    if (rho <= 0.0 || r_n <= 0.0) return 0.0;
    return constants::sutton_graves_earth * std::sqrt(rho / r_n) * v * v * v;
}
```
FlightBody: after each tick, rate from the tick's air data (density, air speed), `heat_load_ += rate * dt`,
peak; then limits: rate > max_heat_rate or load > max_heat_load -> breakup "Hitzeschild versagt"; sensed
acceleration (non-gravitational / g0, already in telemetry) > max_load_g -> "Strukturüberlast". Only stages with
`nose_radius_m > 0` count (the ship); vehicles without heat shield never break up from heating.
Ship TOML:
```toml
[stage.heat_shield]
nose_radius_m = 6.0                      # [E] effective radius of the belly-first windward side
max_heat_rate_w_m2 = 1.2e6               # [E] PICA-class tiles (~1.2 MW/m^2); LEO entries peak ~0.5-0.7
max_heat_load_j_m2 = 6.0e8               # [E]
max_load_g = 6.0                         # [E] structure
```
- [ ] **Step 4: Run** `"[heating]"` - pass. **Step 5:** full suite green.
- [ ] **Step 6: Commit** `feat: entry heating (Sutton-Graves) and load limits with breakup`

### Task 6: Entry attitude setpoint and flap control

**Files:**
- Create: `core/include/starsim/gnc/entry_attitude.hpp`, `core/src/gnc/entry_attitude.cpp`
- Modify: `core/include/starsim/gnc/attitude_controller.hpp` (`ControlOutput::flap_rad`, `Gains::flaps`),
  `core/src/gnc/attitude_controller.cpp` (flap allocation in `allocate`, flap authority in `steer`)
- Test: `core/tests/test_entry_attitude.cpp` (+ CMake)

**Interfaces:**
- Produces:
```cpp
namespace starsim::gnc {
struct EntryAttitude { math::Vec3 axis_gcrf; math::Vec3 roll_reference_gcrf; };
// air_velocity_gcrf: vehicle velocity relative to the air; up_gcrf: local vertical. Angle of attack alpha between
// the body axis and the velocity, belly leading; bank rotates the lift vector about the velocity.
EntryAttitude entry_attitude(const math::Vec3& air_velocity_gcrf, const math::Vec3& up_gcrf, double alpha_rad,
                             double bank_rad, double belly_azimuth_rad);
}
```
Definition: unit flight direction u = v/|v|; lift direction at zero bank l0 = component of up perpendicular to u
(normalised); banked lift l = rotate l0 about u by bank (positive = right wing down, i.e. towards u x up... fix
with the test). The body axis points forward-up: x = cos(alpha) u + sin(alpha) l (the nose leads by alpha from the
velocity towards the lift side; alpha 90 deg = nose straight "up" along l, belly into the flow). The belly
direction b = sin(alpha) u - cos(alpha) l, i.e. perpendicular to x, on the windward side; roll reference: body +y
must point to rotate(b, by -(belly_azimuth - 90 deg)) - implemented as: the body vector at azimuth
belly_azimuth goes to b, so `roll_reference_gcrf` (target of body +y) = b rotated about x by -(belly_azimuth).
`AttitudeController::steer` takes it unchanged.

- `ControlOutput::flap_rad` and `apply_to` copies it into `VehicleCommand::flap_rad`.
- `Gains::flaps` (default false). Allocation with flaps (before gimbals/RCS):
  1. Body-only aero moment M_b from `body.vehicle().aero().compute(...)` with `air_data()` and the CG; flap moments
     M_f(d) = sum over flaps of arm x flap_force at the current deflections; J (3 x n) = numeric derivative
     dM_f/dd_i (step 1 deg, clamped inside [0, max]).
  2. Required flap moment change: dM = tau_cmd - (M_b + M_f(d)) (the controller's tau_cmd is the net torque it
     wants; aero moments are cancelled explicitly - the trim).
  3. dd = J^T (J J^T + eps I)^-1 dM (reuse `allocate_fins`' solver: extract a shared helper
     `minimum_norm(const std::vector<Vec3>& columns, const Vec3& b)`), clamp each d_i + dd_i into [0, max]
     (per-flap clamp, not uniform scale - flaps are one-sided).
  4. Remainder torque = dM - J dd (after clamping) goes to gimbals/RCS as today.
  5. With density 0 (vacuum) or no flaps: d unchanged, everything to RCS.
- `steer`'s rate limiter adds the flap authority (|J| * remaining range) per axis like `fin_max`.

- [ ] **Step 1: Failing tests**

```cpp
TEST_CASE("Entry attitude: alpha 90 points the nose up with the belly into the flow", "[entry]") {
    const math::Vec3 v{7000.0, 0.0, 0.0}, up{0.0, 0.0, 1.0};
    const auto a = gnc::entry_attitude(v, up, M_PI / 2, 0.0, 1.5 * M_PI);
    CHECK(a.axis_gcrf.z == Approx(1.0).margin(1e-12));
    // belly (body -z) must face the flow: body -z maps to +x (the direction of motion)
}
TEST_CASE("Entry attitude: alpha 60, bank 30 tilts the lift vector", "[entry]") {
    const math::Vec3 v{7000.0, 0.0, 0.0}, up{0.0, 0.0, 1.0};
    const auto a = gnc::entry_attitude(v, up, M_PI / 3, M_PI / 6, 1.5 * M_PI);
    CHECK(dot(a.axis_gcrf, math::Vec3{1, 0, 0}) == Approx(0.5).margin(1e-12));          // cos(alpha)
    CHECK(std::abs(a.axis_gcrf.y) == Approx(std::sin(M_PI / 3) * 0.5).margin(1e-12));  // sin(alpha) sin(bank)
}
```
(The executor completes the belly-orientation check by building the quaternion the controller would reach:
rotate body -z with an attitude whose x = axis and whose +y = roll reference.)

Allocation test (ship alone, belly-first at Mach 20 / 65 km, alpha 60): `AttitudeController::allocate` with
`flaps=true` and tau_cmd = 0 returns flap deflections that trim the modelled moment to < 1 % of a single aft
flap's maximum moment, all deflections inside [0, max]; a commanded pitch torque of +/- 20 % of the flap
authority changes the trimmed deflections in the right direction (forward up = nose up); a commanded roll torque
is produced with left/right asymmetry. Saturation test: tau_cmd beyond the flap authority -> deflections at the
limits, `rcs_torque_fraction` non-zero.

Trim test (spec): for alpha 55, 60, 70 deg at Mach 20 (65 km, 6.5 km/s) and alpha 75 deg at Mach 0.4 (5 km),
the trimmed deflections are all inside [15 deg, max - 15 deg] (control margin). If the trim fails, adjust per a
ledgered ruling in this order: the ship's dry CG station [E] (within 18-24 m), then flap areas/stations [E].

- [ ] **Step 2: Run** - fail (missing symbols). **Step 3: Implement.** **Step 4:** `"[entry]"`, `"[attitude]"`
  pass. **Step 5:** full suite green (booster/ascent unaffected: `flaps` false).
- [ ] **Step 6: Commit** `feat: entry attitude setpoint (alpha, bank) and flap trim/control allocation`

### Task 7: Entry corridor - skip-out and too steep

**Files:**
- Create: `core/tests/test_ship_entry.cpp` (+ CMake) with a scenario helper
  `ShipEntryScenario` (vehicle = ship stage alone with `landing_propellant_kg` in its tanks, FlightBody at an
  entry interface state 120 km, given speed and flight-path angle, eastward; each 0.1 s the autopilot stand-in
  commands `entry_attitude(alpha, bank)` through `AttitudeController{flaps=true}` with RCS)
- Modify: `FlightBody` or `FlightSession` event "Abprall": when a ship that was below 80 km climbs back above
  100 km with an apoapsis above 100 km (report once, with the apoapsis) - place it where other flight events are
  reported (`grep -n "events\|message" core/include/starsim/sim/flight_session.hpp`).

**Interfaces:**
- Consumes: Tasks 2-6.
- Produces: `ShipEntryScenario` (test helper, reused by Task 8): `run(double alpha_rad, std::function<double(const
  FlightTelemetry&)> bank, double until_mach)` returning a time series of `FlightTelemetry` and the final body.

- [ ] **Step 1: Failing tests**
```cpp
TEST_CASE("Too shallow an entry skips out of the atmosphere", "[ship][entry][corridor]") {
    // 7.8 km/s inertial at 120 km, flight-path angle -0.3 deg, alpha 50 deg, lift up
    // expect: minimum altitude above 60 km, then climbs above 100 km; event "Abprall"; no breakup
}
TEST_CASE("Too steep an entry exceeds the heat or load limit", "[ship][entry][corridor]") {
    // -6 deg flight-path angle, alpha 60 deg, lift down: breakup "Hitzeschild versagt" or "Strukturüberlast"
}
TEST_CASE("A nominal entry stays inside the corridor", "[ship][entry][corridor]") {
    // -1.0 deg (flight-13-like), alpha 60 deg, lift up then bank to hold ~13 m/s^2 drag: reaches Mach 2 intact,
    // peak heat rate < 80 % of the limit; prints the peak heat rate (STARSIM_VERBOSE)
}
```
(Write each test body fully: build the scenario, run to the end condition, `CHECK` the stated quantities.)
- [ ] **Step 2: Run** - fail. **Step 3: Implement** scenario helper + skip event. **Step 4:** pass. **Step 5:** full
  suite. **Step 6: Commit** `feat: entry corridor - skip-out event, heating and load breakup verified`

### Task 8: Calibration and replay of flights 12-14

**Files:**
- Modify: `config/aero/ship_crossflow.csv` ([D] values), possibly `config/aero/flap.csv`
- Create: `core/tests/test_ship_entry_telemetry.cpp` (+ CMake) - loads `tests/validation/telemetry/flight1*.csv`
  like `test_descent_telemetry.cpp` does (reuse its CSV reader)
- Create: `config/aero/ship_entry_alpha.csv` (`mach, alpha_deg` [D]) and read it in the test (the autopilot of b2
  will use it as its nominal alpha profile)

**Interfaces:**
- Consumes: `ShipEntryScenario` (Task 7).
- Produces: `config/aero/ship_entry_alpha.csv` (nominal alpha over Mach), calibrated cross-flow table.

- [ ] **Step 1: Replay test (RED with the [E] tables)**
```cpp
TEST_CASE("Ship entry reproduces the webcast telemetry of flights 12-14", "[ship][entry][validation]") {
    // For each flight: initial state at the first telemetry row below 88 km (speed, altitude, path angle from a
    // +-20 s altitude parabola; eastward on the flight's ground track, latitude 10 deg [E]); mass dry + landing
    // propellant. Each second the bank is chosen so that the ship's vertical lift fraction cos(bank) equals the
    // telemetry's Lv/D divided by the model's current L/D (clamped to [-1, 1]; bank 0 when the telemetry asks for
    // more than the model has). alpha from ship_entry_alpha.csv.
    // Compare the speed at the telemetry's altitudes from 85 km down to Mach 1: rms relative error < 5 %.
    // Subsonic: mean speed between 10 and 3 km within 10 % of the telemetry's.
}
```
- [ ] **Step 2: Run** - expected FAIL (placeholder alpha 60 / [E] cross-flow): record the rms per flight.
- [ ] **Step 3: Calibrate on flight 13**: choose alpha(M) so the model's L/D at trim equals the upper envelope of
  the telemetry's Lv/D (~0.6 hypersonic -> alpha ~ 58 deg; the body L/D ~ cot alpha), then scale the cross-flow
  table (one factor hypersonic, one subsonic) until beta matches flight 13. Ledger each choice as a ruling with the
  numbers. Do NOT tune on flights 12/14.
- [ ] **Step 4: Run** - flights 12, 13, 14 within tolerance. If 12 or 14 fail while 13 passes, ledger the finding
  (mass uncertainty: each flight's entry mass is unknown within ~ +-10 t) and widen nothing silently.
- [ ] **Step 5: Full suite green; commit** `feat: ship entry calibrated on flight 13, validated on flights 12 and 14`

### Task 9: Documentation

**Files:** `CLAUDE.md` (Known issues: ship entry model, flaps, heating; Architecture list of implemented
modules), `config/README.md` (new tables), `docs/ansichten.md` (HUD: heat rate / load / Abprall warnings in the
entry view, if the document has an entry section - otherwise a [Vorschlag] line), ledger.

- [ ] **Step 1:** write the entries (numbers from Task 8's results).
- [ ] **Step 2:** `godot smoke test` (bindings unchanged, so only if bindings were touched; otherwise skip and
  ledger).
- [ ] **Step 3: Commit** `docs: ship entry model, flaps and corridor`
