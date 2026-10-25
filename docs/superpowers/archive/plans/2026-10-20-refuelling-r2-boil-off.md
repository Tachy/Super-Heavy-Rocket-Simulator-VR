# Refuelling r2: boil-off by heat balance - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cryogenic propellant in the hull tanks of a ship in Earth orbit boils off at the rate a quasi-steady heat
balance on the hull gives (Sun with eclipse, Earth albedo, Earth infrared, an effective wall resistance), in flight and
as an orbit average for the r6 campaign calculator, validated against Centaur flight reconstructions.

**Architecture:** `environment/thermal_radiation` describes what a surface element sees (Sun, albedo, Earth IR with
the exact plate-to-sphere view factor). `vehicle/boil_off` splits every hull tank into strips around the
circumference, solves a skin balance per strip and turns the heat into a vented mass flow by the latent heat; it also
gives the orbit average for an LVLH-held vehicle. `FlightBody` evaluates it every 10 s of free flight above the air
and after every coast and removes the mass with `Vehicle::set_tank_mass` (fidelity flag `boil_off`). Data: a
`[stage.thermal]` block (hull and optional heat-shield zone) and boiling point / latent heat per propellant.

**Tech Stack:** C++20 core (float64, SI), Catch2, toml++ loader (`detail::Document`), Python 3.11 for the committed
calibration script.

**Spec:** `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md` (sections "r2 design", "Testing",
"Documentation"). r1 notes: `docs/subsystems.md` "Orbital refuelling". Config conventions: `config/README.md`.

## Plan decisions that need the user's approval

1. **Data placement:** radiation constants (solar constant, Stefan-Boltzmann, Earth albedo and infrared, AU, Earth
   mean radius) go to `math/constants.hpp` as the spec says. Boiling point and latent heat are **per propellant** data
   and go to `config/propellants.toml` (`oxidizer_boiling_point_k`, `oxidizer_latent_heat_j_kg`, `fuel_...`; absent:
   the propellant does not boil off - RP-1, MMH/NTO), not to code.
2. **Validation and derived insulation values** - source: De Kruif & Kutter, "Centaur Upperstage Applicability for
   Several-Day Mission Durations with Minor Insulation Modifications", AIAA 2007-5845, Table 2: LO2 tank sidewall heat
   flux reconstructed from flight data in sunlit coasts, broadside to the Sun, rotisserie roll. Centaur flights:
   AC-206 (white-painted foam, 167 x 1,132 km, 58 min) 23 Btu/hr-ft^2 = 72.6 W/m^2; AC-167 (185 x 267 km) and AV-003
   (167 x 4,165 km) 25 Btu/hr-ft^2 = 78.9 W/m^2; TC-11 / TC-15 (3-layer MLI, high elliptical) 2 / 4 Btu/hr-ft^2,
   average 3 = 9.46 W/m^2.
   - **Calibrated [D]:** white-foam wall resistance 1.586 m^2K/W from AC-206 (white paint alpha 0.25 / eps 0.88 [E]);
     3-layer MLI 8.22 m^2K/W from the TC average (sun only, cover alpha 0.30 / eps 0.80 [E]).
   - **Validated:** with AC-206's resistance the model predicts AC-167 at 76.3 W/m^2 (-3 %) and AV-003 at 63.1 W/m^2
     (-20 %); tolerance **+-25 %**, set by the data's own scatter (sister flights TC-11 / TC-15 differ by a factor 2)
     and the unknown coast geometry (axis taken normal to the orbit plane, Sun in the plane at noon mid-coast [E]).
   - **Depot [D]:** 30 effective MLI layers [E] (r1's 3 t insulation, ~2.3 kg/m^2) with the paper's leak ~ 1/(n+1)
     law: 8.22 x 31/4 = **63.7 m^2K/W**, cover alpha 0.30 / eps 0.80 [E]. Pre-computed: ~1.1 W/m^2 broadside in sun,
     ~2.6 W/m^2 sunlit in a 200 km orbit (literature: 1-4 W/m^2 residual MLI flux).
   - **Block 3 ship (and so the tanker) [E]:** bare stainless hull alpha 0.40 / eps 0.15 (handbook polished stainless
     ~0.4 / ~0.1-0.15), wall resistance 0.01 m^2K/W (wetted steel wall + boiling film); heat-shield tiles alpha = eps =
     0.85, 1.5 m^2K/W (~4 cm silica tile + felt in vacuum), +-90 deg around the belly. Pre-computed bare hull in a
     200 km orbit: ~228 W/m^2 sunlit - **order 30 t/day for a bare ship**, i.e. ~1-2 t of main-tank propellant in a
     one-hour orbital coast. This is the physics of an uninsulated tank; it is not tuned away.
3. **Deliberate simplifications** (documented, switchable as a whole by the flag): quasi-steady strips (no skin heat
   capacity, no conduction around the circumference); no tank pressure / temperature / subcooling / stratification;
   all wall heat of a tank's section goes into its liquid whatever the fill level (conservative); domes, ends, header
   tanks (inside the hull) and RCS tanks take no heat; no shadow from other vehicles or flaps; the Moon only shadows
   (its albedo and infrared are not modelled - relevant only for lunar boil-off, see 4).
4. **Scope of `[stage.thermal]` in r2:** the Block 3 ship (the tanker inherits it through `from`) and the depot. The
   HLS gets none yet: its boil-off during a lunar mission needs the Moon's radiation, decided in r5 with `hls_refuel`;
   so the lunar results do not change in r2. The flag `boil_off` defaults **on**; Block 3 ship flights now lose
   main-tank propellant in orbit. The ship tags are re-run; changed numbers are reported, not tuned.
5. **Cadence:** evaluated every 10 s [E] of free flight above 100 km [E] (below: air convection and entry heating,
   not modelled) and after every coast, mass by the trapezoid of the last two evaluations. Clamped, caught, landed,
   resting and docked bodies do not boil off in r2 (a docked tanker's boil-off belongs to r4's transfer).

## Global Constraints

- SI units internally, angles in radians; frame and unit in names (`position_gcrf_m`, `wall_resistance_m2k_w`).
- No magic numbers in code: physical constants in `core/include/starsim/math/constants.hpp` with their source; vehicle
  and propellant numbers in `config/` with provenance tags [S] sourced, [E] estimate, [D] derived.
- C++ sources UTF-8; UI / error strings shown to the user are German, code and comments English.
- Small files with one responsibility; modules depend downwards only (`environment` never includes `vehicle`;
  `vehicle` may use `environment`, `orbit`, `frames`, `config`, `math`).
- Simplifications only deliberate, documented and switchable (fidelity flag `boil_off`).
- ctest runs cases in parallel: tests must not share temp files or directories.
- Test workflow: run only the tags the change affects (`-TestFilter`), incl. `[slow]` cases of those tags.
- Commit on `main`, Conventional Commits, push after each task once its tests are green.
- Commit trailer:
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_012biTJd57gGHxfvAHbX6f2T`.

## Review Focus

1. A tank emptied by boil-off must stop at 0 kg (no negative mass, no NaN) and the engines starve as usual - Task 5
   test "boil-off empties a tank to zero".
2. A stack that separates between two evaluations must not apply the stored per-tank rates of the old stack to the
   wrong (or missing) tanks - Task 5 test "separation clears the stored rates".
3. Vehicles without `[stage.thermal]` (Crew Dragon, Falcon 9, Super Heavy) and environments without an ephemeris must
   behave exactly as before - Task 5 tests.
4. Sun in the Earth's umbra or along the ship's axis: the net tank heat may be negative; the vented flow must clamp at
   0 (no condensation) - Task 3 test.
5. Time warp: one 600 s coast and sixty 10 s coasts in steady sunlight must vent the same mass (cadence independence)
   - Task 5 test.

---

### Task 1: Thermal radiation environment

**Files:**
- Modify: `core/include/starsim/math/constants.hpp` (append a "Thermal radiation" block after `obliquity_j2000_rad`)
- Create: `core/include/starsim/environment/thermal_radiation.hpp`
- Create: `core/src/environment/thermal_radiation.cpp`
- Modify: `core/CMakeLists.txt` (add `src/environment/thermal_radiation.cpp` after `src/environment/eclipse.cpp`)
- Modify: `core/src/vehicle/flight_body.cpp:27` (drop the local `earth_mean_radius_m`, use `constants::earth_mean_radius_m`)
- Create: `core/tests/test_boil_off.cpp`
- Modify: `core/tests/CMakeLists.txt` (add `test_boil_off.cpp` after `test_refuelling_vehicles.cpp`)

**Interfaces:**
- Produces (`starsim::constants`): `solar_constant_w_m2`, `astronomical_unit_m`, `stefan_boltzmann_w_m2k4`,
  `earth_bond_albedo`, `earth_outgoing_longwave_w_m2`, `earth_mean_radius_m`.
- Produces (`starsim::environment`):
  - `struct RadiationEnvironment { math::Vec3 sun_direction; double solar_flux_w_m2; double sunlit_fraction; math::Vec3 nadir_direction; double distance_over_radius; double albedo_phase; };`
  - `double plate_view_factor_to_sphere(double angle_from_nadir_rad, double distance_over_radius);`
  - `RadiationEnvironment earth_orbit_radiation(const math::Vec3& position_gcrf_m, const math::Vec3& sun_from_vehicle_m, const math::Vec3* moon_from_vehicle_m);`
  - `double absorbed_flux_w_m2(const math::Vec3& normal, const RadiationEnvironment& env, double absorptivity, double emissivity);`

- [ ] **Step 1: Write the failing tests**

Create `core/tests/test_boil_off.cpp`:

```cpp
// Refuelling r2: propellant boil-off by a heat balance on the hull.

#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/thermal_radiation.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("plate view factor to a sphere: nadir, horizon and the orientation average", "[boil_off][environment]") {
    const double h = (constants::earth_mean_radius_m + 200.0e3) / constants::earth_mean_radius_m;
    CHECK(environment::plate_view_factor_to_sphere(0.0, h) == Catch::Approx(1.0 / (h * h)).epsilon(1e-12));

    // Continuous where the sphere starts to sink below the plate's horizon, and zero once it is gone.
    const double full = std::acos(1.0 / h);
    CHECK(environment::plate_view_factor_to_sphere(full - 1e-9, h) ==
          Catch::Approx(environment::plate_view_factor_to_sphere(full + 1e-9, h)).margin(1e-7));
    const double gone = constants::pi - full;
    CHECK(environment::plate_view_factor_to_sphere(gone - 1e-9, h) == Catch::Approx(0.0).margin(1e-7));
    CHECK(environment::plate_view_factor_to_sphere(gone + 0.1, h) == 0.0);

    // Over all orientations a plate sees the sphere as a small sphere does: (1 - cos rho) / 2, sin rho = 1 / h.
    constexpr int n = 20000;
    double sum = 0.0;
    for (int k = 0; k < n; ++k) {
        sum += environment::plate_view_factor_to_sphere(std::acos(1.0 - 2.0 * (k + 0.5) / n), h);
    }
    CHECK(sum / n == Catch::Approx(0.5 * (1.0 - std::sqrt(1.0 - 1.0 / (h * h)))).epsilon(1e-6));
}

TEST_CASE("earth orbit radiation: sunlit noon side and umbra", "[boil_off][environment]") {
    const Vec3 position{constants::earth_mean_radius_m + 400.0e3, 0.0, 0.0};
    const Vec3 sun_gcrf{constants::astronomical_unit_m, 0.0, 0.0};

    const auto noon = environment::earth_orbit_radiation(position, sun_gcrf - position, nullptr);
    CHECK(noon.sunlit_fraction == 1.0);
    CHECK(noon.albedo_phase == Catch::Approx(1.0));
    CHECK(noon.solar_flux_w_m2 == Catch::Approx(constants::solar_constant_w_m2).epsilon(1e-4));
    CHECK(noon.nadir_direction.x == Catch::Approx(-1.0));
    CHECK(noon.distance_over_radius == Catch::Approx(position.x / constants::earth_mean_radius_m));

    const auto midnight = environment::earth_orbit_radiation(-position, sun_gcrf + position, nullptr);
    CHECK(midnight.sunlit_fraction == 0.0);
    CHECK(midnight.albedo_phase == 0.0);
}

TEST_CASE("absorbed flux of a flat element: sun, albedo and Earth infrared", "[boil_off][environment]") {
    environment::RadiationEnvironment env;
    env.sun_direction = {0.0, 1.0, 0.0};
    env.solar_flux_w_m2 = constants::solar_constant_w_m2;
    env.sunlit_fraction = 1.0;
    env.nadir_direction = {-1.0, 0.0, 0.0};
    env.distance_over_radius = 1.1;
    env.albedo_phase = 0.5;

    // Facing the Sun, edge-on to the Earth (the sphere half below its horizon).
    const double side = environment::plate_view_factor_to_sphere(constants::pi / 2.0, 1.1);
    CHECK(environment::absorbed_flux_w_m2({0.0, 1.0, 0.0}, env, 0.4, 0.8) ==
          Catch::Approx(0.4 * (constants::solar_constant_w_m2 +
                               constants::earth_bond_albedo * constants::solar_constant_w_m2 * 0.5 * side) +
                        0.8 * constants::earth_outgoing_longwave_w_m2 * side));
    // Facing the Earth: no direct sun, the full view factor 1 / h^2.
    const double down = 1.0 / (1.1 * 1.1);
    CHECK(environment::absorbed_flux_w_m2({-1.0, 0.0, 0.0}, env, 0.4, 0.8) ==
          Catch::Approx(0.4 * constants::earth_bond_albedo * constants::solar_constant_w_m2 * 0.5 * down +
                        0.8 * constants::earth_outgoing_longwave_w_m2 * down));
    // In the umbra the direct sun is gone, albedo and infrared stay (the Earth below is still lit at phase 0.5).
    env.sunlit_fraction = 0.0;
    CHECK(environment::absorbed_flux_w_m2({0.0, 1.0, 0.0}, env, 0.4, 0.8) ==
          Catch::Approx(0.4 * constants::earth_bond_albedo * constants::solar_constant_w_m2 * 0.5 * side +
                        0.8 * constants::earth_outgoing_longwave_w_m2 * side));
}
```

Add `test_boil_off.cpp` to `core/tests/CMakeLists.txt` after `test_refuelling_vehicles.cpp`.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[boil_off]" > tmp/r2_t1_red.txt 2>&1`, then read the tail.
Expected: build error - `starsim/environment/thermal_radiation.hpp` not found.

- [ ] **Step 3: Implement**

Append to `core/include/starsim/math/constants.hpp` (inside the namespace, after `obliquity_j2000_rad`):

```cpp
// Thermal radiation (refuelling r2, boil-off).
// Total solar irradiance at 1 AU [S: Kopp & Lean 2011, GRL 38, L01706: 1360.8 +- 0.5 W/m^2].
inline constexpr double solar_constant_w_m2 = 1361.0;
// [S: IAU 2012 Resolution B2]
inline constexpr double astronomical_unit_m = 1.495978707e11;
// [S: CODATA 2018]
inline constexpr double stefan_boltzmann_w_m2k4 = 5.670374419e-8;
// Earth's Bond albedo [S: 0.29-0.30, CERES; Stephens et al. 2015, Rev. Geophys. 53] - 0.30 as in the r2 spec.
inline constexpr double earth_bond_albedo = 0.30;
// Global mean outgoing longwave radiation [S: 239 W/m^2, Trenberth, Fasullo & Kiehl 2009, BAMS 90].
inline constexpr double earth_outgoing_longwave_w_m2 = 239.0;
// Mean radius [S: IUGG 6371.0 km]: the spherical Earth of shadows and thermal radiation.
inline constexpr double earth_mean_radius_m = 6371000.0;
```

In `core/src/vehicle/flight_body.cpp` delete line 27 (`constexpr double earth_mean_radius_m = 6371000.0;`) and write
`constants::earth_mean_radius_m` at its use in `solar_radiation_acceleration`.

Create `core/include/starsim/environment/thermal_radiation.hpp`:

```cpp
#pragma once

#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// What a spacecraft surface element sees in Earth orbit (refuelling r2): the Sun, sunlight reflected by the Earth
// (albedo) and the Earth's own infrared. Unit directions in the frame the caller works in.
struct RadiationEnvironment {
    math::Vec3 sun_direction;           // towards the Sun
    double solar_flux_w_m2 = 0.0;       // at the vehicle's distance from the Sun, unshadowed
    double sunlit_fraction = 0.0;       // 0 (umbra) .. 1: the part of the Sun's disk Earth and Moon leave visible
    math::Vec3 nadir_direction;         // towards the Earth's centre
    double distance_over_radius = 0.0;  // distance from the Earth's centre over its mean radius (> 1)
    // Albedo illumination [E]: cosine of the Sun's zenith angle at the point below the vehicle, >= 0 (the usual
    // engineering approximation: the whole visible cap lit like its centre).
    double albedo_phase = 0.0;
};

// View factor from a flat element to a sphere, the element's normal at `angle_from_nadir_rad` from the direction to
// the sphere's centre, `distance_over_radius` (> 1) radii from it [S: Howell, Menguc & Siegel, catalog of
// configuration factors, "planar element to sphere, tilted"]: cos(angle) / H^2 while the whole sphere is above the
// element's horizon, the partial-view expression across the horizon, 0 beyond.
double plate_view_factor_to_sphere(double angle_from_nadir_rad, double distance_over_radius);

// The radiation environment of a vehicle at `position_gcrf_m` with the Sun at `sun_from_vehicle_m` and the Moon at
// `*moon_from_vehicle_m` (from the vehicle, geometric as for solar radiation pressure; nullptr: no lunar shadow).
// Spherical Earth of mean radius. The Moon only shadows: its albedo and infrared are not modelled (r2, documented).
RadiationEnvironment earth_orbit_radiation(const math::Vec3& position_gcrf_m, const math::Vec3& sun_from_vehicle_m,
                                           const math::Vec3* moon_from_vehicle_m);

// Flux absorbed by a flat element with outward unit normal `normal` (the environment's frame): absorptivity x (direct
// sun + albedo) + emissivity x Earth infrared (Kirchhoff: the infrared absorptivity is the emissivity).
double absorbed_flux_w_m2(const math::Vec3& normal, const RadiationEnvironment& env, double absorptivity,
                          double emissivity);

}  // namespace starsim::environment
```

Create `core/src/environment/thermal_radiation.cpp`:

```cpp
#include "starsim/environment/thermal_radiation.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/environment/eclipse.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

using math::Vec3;

double plate_view_factor_to_sphere(double angle_from_nadir_rad, double distance_over_radius) {
    const double h = std::max(distance_over_radius, 1.0 + 1e-12);
    const double full_view = std::acos(1.0 / h);  // up to here the whole sphere is above the horizon
    const double cos_a = std::cos(angle_from_nadir_rad);
    if (angle_from_nadir_rad <= full_view) return cos_a / (h * h);
    if (angle_from_nadir_rad >= constants::pi - full_view) return 0.0;
    const double s = std::sqrt(h * h - 1.0);
    const double sin_a = std::sin(angle_from_nadir_rad);
    const double rim = std::sqrt(std::max(0.0, 1.0 - h * h * cos_a * cos_a));
    return 0.5 - std::asin(std::min(1.0, s / (h * sin_a))) / constants::pi +
           (cos_a * std::acos(std::clamp(-s * cos_a / sin_a, -1.0, 1.0)) - s * rim) / (constants::pi * h * h);
}

RadiationEnvironment earth_orbit_radiation(const Vec3& position_gcrf_m, const Vec3& sun_from_vehicle_m,
                                           const Vec3* moon_from_vehicle_m) {
    RadiationEnvironment env;
    const double sun_distance = norm(sun_from_vehicle_m);
    env.sun_direction = sun_from_vehicle_m / sun_distance;
    const double au_ratio = constants::astronomical_unit_m / sun_distance;
    env.solar_flux_w_m2 = constants::solar_constant_w_m2 * au_ratio * au_ratio;
    const double earth_shadow = occulted_fraction(sun_from_vehicle_m, constants::sun_radius_m, -position_gcrf_m,
                                                  constants::earth_mean_radius_m);
    const double moon_shadow = moon_from_vehicle_m == nullptr
                                   ? 0.0
                                   : occulted_fraction(sun_from_vehicle_m, constants::sun_radius_m,
                                                       *moon_from_vehicle_m, constants::moon_mean_radius_m);
    env.sunlit_fraction = (1.0 - earth_shadow) * (1.0 - moon_shadow);
    const double r = norm(position_gcrf_m);
    env.nadir_direction = position_gcrf_m * (-1.0 / r);
    env.distance_over_radius = r / constants::earth_mean_radius_m;
    env.albedo_phase = std::max(0.0, -dot(env.nadir_direction, env.sun_direction));
    return env;
}

double absorbed_flux_w_m2(const Vec3& normal, const RadiationEnvironment& env, double absorptivity,
                          double emissivity) {
    const double direct = env.solar_flux_w_m2 * env.sunlit_fraction * std::max(0.0, dot(normal, env.sun_direction));
    const double view = plate_view_factor_to_sphere(std::acos(std::clamp(dot(normal, env.nadir_direction), -1.0, 1.0)),
                                                    env.distance_over_radius);
    const double albedo = constants::earth_bond_albedo * env.solar_flux_w_m2 * env.albedo_phase * view;
    return absorptivity * (direct + albedo) + emissivity * constants::earth_outgoing_longwave_w_m2 * view;
}

}  // namespace starsim::environment
```

(If `Vec3` has no `operator*(double)` in that order or no unary minus, use the operators `vec3.hpp` provides -
`flight_body.cpp` uses `-s.position_gcrf_m` and `to_sun / distance`.)

Add `src/environment/thermal_radiation.cpp` to `core/CMakeLists.txt` after `src/environment/eclipse.cpp`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[boil_off]" > tmp/r2_t1.txt 2>&1`, read the tail.
Expected: 3 test cases pass. Then the SRP users (radius constant moved): `-TestFilter "[srp]"`; if no such tag exists,
`grep -l "solar_radiation" core/tests/*.cpp` and run those files' tags. Expected: unchanged results.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/math/constants.hpp core/include/starsim/environment/thermal_radiation.hpp core/src/environment/thermal_radiation.cpp core/CMakeLists.txt core/src/vehicle/flight_body.cpp core/tests/test_boil_off.cpp core/tests/CMakeLists.txt
git commit -m "feat(environment): thermal radiation environment for boil-off - sun, albedo, Earth infrared"
git push origin main
```

---

### Task 2: Thermal configuration and propellant thermodynamics

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp` (`PropellantConfig` ~line 23; new `ThermalSurfaceConfig` /
  `ThermalConfig` before `StageConfig`; `StageConfig::thermal` after `heat_shield`)
- Modify: `core/src/config/vehicle_config.cpp` (`load_propellant` ~line 113; `load_stage` after the `heat_shield` block ~line 395)
- Modify: `core/include/starsim/vehicle/vehicle.hpp` (`struct Tank` ~line 103)
- Modify: `core/src/vehicle/vehicle.cpp` (tank construction ~line 78-86)
- Modify: `config/propellants.toml`, `config/vehicles/starship_block3.toml` (ship stage, after `[stage.heat_shield]`),
  `config/vehicles/starship_depot.toml` (ship stage, after the last `[[stage.rcs.pod]]`, before `[[stage.docking_port]]`)
- Test: `core/tests/test_boil_off.cpp`

**Interfaces:**
- Consumes: nothing from Task 1.
- Produces (`starsim::config`):
  - `PropellantConfig::{oxidizer_boiling_point_k, oxidizer_latent_heat_j_kg, fuel_boiling_point_k, fuel_latent_heat_j_kg}` (0: does not boil)
  - `struct ThermalSurfaceConfig { double absorptivity; double emissivity; double wall_resistance_m2k_w; };`
  - `struct ThermalConfig { ThermalSurfaceConfig hull; std::optional<ThermalSurfaceConfig> shield; double shield_half_width_rad; };`
  - `std::optional<ThermalConfig> StageConfig::thermal;`
- Produces (`starsim::vehicle`): `Tank::boiling_point_k`, `Tank::latent_heat_j_kg` (0: does not boil).

- [ ] **Step 1: Write the failing tests**

Append to `core/tests/test_boil_off.cpp` (add `#include <fstream>`, `#include "starsim/config/vehicle_config.hpp"`,
`#include "starsim/vehicle/vehicle.hpp"` at the top, and this helper block after the `using` lines):

```cpp
namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

// The top stage of a vehicle flying alone (the ship after staging).
std::unique_ptr<vehicle::Vehicle> top_stage_alone(const std::string& vehicle_id) {
    auto cfg = config::load_vehicle(config_dir, vehicle_id);
    cfg.stages.erase(cfg.stages.begin(), cfg.stages.end() - 1);
    return std::make_unique<vehicle::Vehicle>(cfg, config_dir);
}

const vehicle::Tank& tank_named(const vehicle::Vehicle& v, const std::string& id) {
    for (const auto& t : v.stages().back().tanks) {
        if (t.config.id == id) return t;
    }
    throw std::runtime_error("no tank " + id);
}

}  // namespace

TEST_CASE("methalox boils at its normal boiling points with its latent heats", "[boil_off][config]") {
    const auto ship = top_stage_alone("starship_block3");
    const auto& lox = tank_named(*ship, "lox_main");
    const auto& ch4 = tank_named(*ship, "ch4_main");
    CHECK(lox.boiling_point_k == Catch::Approx(90.19));
    CHECK(lox.latent_heat_j_kg == Catch::Approx(213.1e3));
    CHECK(ch4.boiling_point_k == Catch::Approx(111.67));
    CHECK(ch4.latent_heat_j_kg == Catch::Approx(510.8e3));
    // Storable propellants do not boil off.
    const auto dragon = top_stage_alone("crew_dragon");
    for (const auto& t : dragon->stages().back().tanks) CHECK(t.latent_heat_j_kg == 0.0);
}

TEST_CASE("stage.thermal: Block 3 ship with tiles, the tanker inherits it, the depot is insulated", "[boil_off][config]") {
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");
    REQUIRE(block3.stages.back().thermal.has_value());
    const auto& ship = *block3.stages.back().thermal;
    CHECK(ship.hull.absorptivity == Catch::Approx(0.40));
    CHECK(ship.hull.emissivity == Catch::Approx(0.15));
    CHECK(ship.hull.wall_resistance_m2k_w == Catch::Approx(0.01));
    REQUIRE(ship.shield.has_value());
    CHECK(ship.shield->wall_resistance_m2k_w == Catch::Approx(1.5));
    CHECK(ship.shield_half_width_rad == Catch::Approx(constants::pi / 2.0));
    CHECK_FALSE(block3.stages.front().thermal.has_value());  // the booster lives minutes: no boil-off model

    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    REQUIRE(tanker.stages.back().thermal.has_value());
    CHECK(tanker.stages.back().thermal->hull.absorptivity == Catch::Approx(0.40));

    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    REQUIRE(depot.stages.back().thermal.has_value());
    CHECK(depot.stages.back().thermal->hull.wall_resistance_m2k_w == Catch::Approx(63.7));
    CHECK_FALSE(depot.stages.back().thermal->shield.has_value());
}

TEST_CASE("stage.thermal rejects optics outside (0, 1] and a missing resistance", "[boil_off][config]") {
    const auto dir = std::filesystem::temp_directory_path() / "starsim_boil_off_bad_thermal";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir, dir, std::filesystem::copy_options::recursive |
                                               std::filesystem::copy_options::overwrite_existing);
    const auto path = dir / "vehicles" / "starship_depot.toml";
    std::ifstream in(path);
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    const std::string good = "absorptivity = 0.30";
    REQUIRE(text.find(good) != std::string::npos);
    text.replace(text.find(good), good.size(), "absorptivity = 1.30");
    std::ofstream(path) << text;
    CHECK_THROWS_AS(config::load_vehicle(dir, "starship_depot"), config::ConfigError);
    std::filesystem::remove_all(dir);
}
```

(`absorptivity = 0.30` must occur in `starship_depot.toml` exactly in its `[stage.thermal]` line - the depot file has
no other `absorptivity` key. `config::ConfigError` is the loader's exception; if its name differs, use the one
`toml_document.hpp` throws.)

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[boil_off]" > tmp/r2_t2_red.txt 2>&1`, read the tail.
Expected: build errors (`boiling_point_k`, `thermal` not members).

- [ ] **Step 3: Implement**

`vehicle_config.hpp`, `PropellantConfig`:

```cpp
struct PropellantConfig {
    std::string id;
    double mixture_ratio = 0.0;  // oxidizer / fuel by mass
    double oxidizer_density_kg_m3 = 0.0;
    double fuel_density_kg_m3 = 0.0;
    // Boil-off (refuelling r2): normal boiling point and latent heat of vaporisation; 0: storable, does not boil off.
    double oxidizer_boiling_point_k = 0.0;
    double oxidizer_latent_heat_j_kg = 0.0;
    double fuel_boiling_point_k = 0.0;
    double fuel_latent_heat_j_kg = 0.0;
};
```

Before `struct StageConfig`:

```cpp
// Surface optics and wall heat path of a hull zone (refuelling r2 boil-off).
struct ThermalSurfaceConfig {
    double absorptivity = 0.0;           // solar
    double emissivity = 0.0;             // infrared (= the infrared absorptivity)
    double wall_resistance_m2k_w = 0.0;  // effective skin -> propellant: insulation, wall, liquid-side film
};

// [stage.thermal]: the hull; [stage.thermal.shield]: the windward heat-shield tiles, a zone of +-half_width_deg around
// [stage.aero] belly_azimuth_deg.
struct ThermalConfig {
    ThermalSurfaceConfig hull;
    std::optional<ThermalSurfaceConfig> shield;
    double shield_half_width_rad = 0.0;
};
```

In `StageConfig`, after `HeatShieldConfig heat_shield;`:

```cpp
    std::optional<ThermalConfig> thermal;  // [stage.thermal] (refuelling r2): none -> its tanks do not boil off
```

`vehicle_config.cpp`, `load_propellant` - replace the `return {...};` with:

```cpp
                PropellantConfig p{id, doc.number(t, "mixture_ratio"), doc.number(t, "oxidizer_density_kg_m3"),
                                   doc.number(t, "fuel_density_kg_m3")};
                // Both or neither of boiling point and latent heat, positive.
                auto boiling = [&](const char* side, double& point_k, double& latent_j_kg) {
                    const std::string point_key = std::string(side) + "_boiling_point_k";
                    const std::string latent_key = std::string(side) + "_latent_heat_j_kg";
                    if (!t.contains(point_key) && !t.contains(latent_key)) return;
                    point_k = doc.positive(t, point_key);
                    latent_j_kg = doc.positive(t, latent_key);
                };
                boiling("oxidizer", p.oxidizer_boiling_point_k, p.oxidizer_latent_heat_j_kg);
                boiling("fuel", p.fuel_boiling_point_k, p.fuel_latent_heat_j_kg);
                return p;
```

`load_stage`, after the `if (auto shield = t["heat_shield"].as_table()) { ... }` block:

```cpp
    if (auto thermal = t["thermal"].as_table()) {
        auto surface = [&](const toml::table& zone, const std::string& where) {
            const ThermalSurfaceConfig c{doc.number(zone, "absorptivity"), doc.number(zone, "emissivity"),
                                         doc.positive(zone, "wall_resistance_m2k_w")};
            if (!(c.absorptivity > 0.0 && c.absorptivity <= 1.0) || !(c.emissivity > 0.0 && c.emissivity <= 1.0)) {
                doc.fail(where + ": absorptivity and emissivity must lie in (0, 1]");
            }
            return c;
        };
        ThermalConfig c;
        c.hull = surface(*thermal, "stage.thermal");
        if (auto shield = (*thermal)["shield"].as_table()) {
            c.shield = surface(*shield, "stage.thermal.shield");
            c.shield_half_width_rad = doc.positive(*shield, "half_width_deg") * constants::deg_to_rad;
            if (c.shield_half_width_rad > constants::pi) doc.fail("stage.thermal.shield: half_width_deg <= 180");
        }
        s.thermal = c;
    }
```

`vehicle.hpp`, `struct Tank` - append after `settling`:

```cpp
    double boiling_point_k = 0.0;   // boil-off (refuelling r2): of its propellant; 0: does not boil
    double latent_heat_j_kg = 0.0;
```

`vehicle.cpp`, after `stage.tanks.push_back({...});` in the tank loop:

```cpp
            auto& tank = stage.tanks.back();
            if (tank_config.contents == config::TankContents::Oxidizer) {
                tank.boiling_point_k = config.propellant.oxidizer_boiling_point_k;
                tank.latent_heat_j_kg = config.propellant.oxidizer_latent_heat_j_kg;
            } else if (tank_config.contents == config::TankContents::Fuel) {
                tank.boiling_point_k = config.propellant.fuel_boiling_point_k;
                tank.latent_heat_j_kg = config.propellant.fuel_latent_heat_j_kg;
            }
```

`config/propellants.toml` - `methalox`, after `fuel_density_kg_m3`:

```toml
# Boil-off (refuelling r2): normal boiling point and latent heat of vaporisation [S: NIST Chemistry WebBook -
# O2 90.19 K, 6.82 kJ/mol / 32.00 g/mol; CH4 111.67 K, 8.19 kJ/mol / 16.04 g/mol]. Loaded subcooled; the sensible
# heat up to the boiling point is not credited (no tank temperature model).
oxidizer_boiling_point_k = 90.19
oxidizer_latent_heat_j_kg = 213100.0
fuel_boiling_point_k = 111.67
fuel_latent_heat_j_kg = 510800.0
```

`kerolox`, after `fuel_density_kg_m3` (LOX boils; RP-1 does not):

```toml
oxidizer_boiling_point_k = 90.19          # [S: NIST] as methalox
oxidizer_latent_heat_j_kg = 213100.0
```

`config/vehicles/starship_block3.toml`, ship stage, directly after the `[stage.heat_shield]` block:

```toml
# Boil-off heat balance (refuelling r2). Bare stainless hull: alpha / eps [E] from handbook values for polished
# stainless steel (~0.4 / 0.1-0.15); effective wall resistance [E] of a wetted 4 mm steel wall with its liquid-side
# film - an uninsulated tank. Tiles on the windward half [E]: black coating alpha = eps ~0.85, ~4 cm silica tile + felt
# in vacuum ~1.5 m^2K/W.
[stage.thermal]
absorptivity = 0.40                      # [E]
emissivity = 0.15                        # [E]
wall_resistance_m2k_w = 0.01             # [E]

[stage.thermal.shield]
absorptivity = 0.85                      # [E]
emissivity = 0.85                        # [E]
wall_resistance_m2k_w = 1.5              # [E]
half_width_deg = 90.0                    # [E] the belly half, around [stage.aero] belly_azimuth_deg
```

`config/vehicles/starship_depot.toml`, ship stage, after the last `[[stage.rcs.pod]]` block:

```toml
# Boil-off heat balance (refuelling r2): the 3 t insulation of the dry mass as ~30 effective MLI layers [E]. Wall
# resistance [D] = 8.22 m^2K/W (3-layer MLI, calibrated on Centaur TC-11/TC-15, AIAA 2007-5845 Table 2; see
# tools/validation/centaur_boil_off_calibration.py) x (30 + 1) / (3 + 1) (leak ~ 1/(n+1), same paper, section III).
# Cover optics [E]: white beta-cloth-like outer layer.
[stage.thermal]
absorptivity = 0.30                      # [E]
emissivity = 0.80                        # [E]
wall_resistance_m2k_w = 63.7             # [D]
```

Confirm `starship_tanker.toml`'s ship stage is `from = "starship_block3:ship"` (r1): it inherits `[stage.thermal]`
without a change.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[boil_off],[config],[refuelling]" > tmp/r2_t2.txt 2>&1`, read the tail.
Expected: all pass. Then `python tools/vehicle_report.py starship_depot` and `starship_block3` - unchanged masses.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/config/vehicle_config.hpp core/src/config/vehicle_config.cpp core/include/starsim/vehicle/vehicle.hpp core/src/vehicle/vehicle.cpp config/propellants.toml config/vehicles/starship_block3.toml config/vehicles/starship_depot.toml core/tests/test_boil_off.cpp
git commit -m "feat(config): stage.thermal surfaces and propellant boiling data for boil-off"
git push origin main
```

---

### Task 3: Heat balance and Centaur validation

**Files:**
- Create: `core/include/starsim/vehicle/boil_off.hpp`
- Create: `core/src/vehicle/boil_off.cpp`
- Modify: `core/CMakeLists.txt` (add `src/vehicle/boil_off.cpp` after `src/vehicle/compound_mass.cpp`)
- Create: `tools/validation/centaur_boil_off_calibration.py`
- Test: `core/tests/test_boil_off.cpp`

**Interfaces:**
- Consumes: Task 1 `RadiationEnvironment`, `absorbed_flux_w_m2`; Task 2 `ThermalConfig`, `Tank::boiling_point_k`,
  `Tank::latent_heat_j_kg`.
- Produces (`starsim::vehicle`):
  - `inline constexpr int boil_off_strips = 36;`
  - `struct BoilOffRate { double oxidizer_kgps; double fuel_kgps; };`
  - `struct TankBoilOff { size_t stage_index; size_t tank_index; double heat_w; double mass_flow_kgps; };`
  - `struct HullSection { double radius_m; double length_m; };`
  - `double strip_heat_leak_w_m2(double absorbed_w_m2, const config::ThermalSurfaceConfig& surface, double boiling_point_k);`
  - `double section_heat_w(const HullSection& section, const config::ThermalConfig& thermal, double belly_azimuth_rad, double boiling_point_k, const environment::RadiationEnvironment& env, const math::Quat& body_to_frame);`
  - `std::vector<TankBoilOff> boil_off_rates(const Vehicle& vehicle, const environment::RadiationEnvironment& env, const math::Quat& body_to_frame);`
  - `BoilOffRate total_boil_off(const Vehicle& vehicle, const std::vector<TankBoilOff>& tanks);`

- [ ] **Step 1: Write the failing tests**

Append to `core/tests/test_boil_off.cpp` (add `#include "starsim/vehicle/boil_off.hpp"`, `#include "starsim/math/quat.hpp"`,
`#include <cstdlib>`, `#include <iostream>` at the top):

```cpp
namespace {

// Far from the Earth (no albedo, no infrared), the Sun along `sun` (unit, body frame = environment frame).
environment::RadiationEnvironment deep_space_sun(const Vec3& sun) {
    environment::RadiationEnvironment env;
    env.sun_direction = sun;
    env.solar_flux_w_m2 = constants::solar_constant_w_m2;
    env.sunlit_fraction = 1.0;
    env.nadir_direction = {-1.0, 0.0, 0.0};
    env.distance_over_radius = 1.0e6;
    env.albedo_phase = 0.0;
    return env;
}

bool verbose() { return std::getenv("STARSIM_VERBOSE") != nullptr; }

}  // namespace

TEST_CASE("strip balance: conducting wall passes the absorbed flux, an insulating one none", "[boil_off][vehicle]") {
    const double tb = 90.19, absorbed = 500.0, eps = 0.15;
    const double sigma_tb4 = eps * constants::stefan_boltzmann_w_m2k4 * std::pow(tb, 4);
    // Nearly no resistance: the skin sits at the boiling point, everything but its own tiny emission goes in.
    CHECK(vehicle::strip_heat_leak_w_m2(absorbed, {0.4, eps, 1e-6}, tb) ==
          Catch::Approx(absorbed - sigma_tb4).epsilon(1e-4));
    // Nearly perfect insulation: radiative equilibrium skin, leak (T - Tb) / R.
    const double r = 1e9;
    const double t_eq = std::pow(absorbed / (eps * constants::stefan_boltzmann_w_m2k4), 0.25);
    CHECK(vehicle::strip_heat_leak_w_m2(absorbed, {0.4, eps, r}, tb) * r + tb == Catch::Approx(t_eq).epsilon(1e-6));
    // Nothing absorbed: the wall loses heat (negative leak).
    CHECK(vehicle::strip_heat_leak_w_m2(0.0, {0.4, eps, 0.01}, tb) < 0.0);
}

TEST_CASE("cylinder broadside to the Sun absorbs alpha S D L", "[boil_off][vehicle]") {
    const config::ThermalConfig bare{{0.4, 0.15, 1e-6}, std::nullopt, 0.0};
    const vehicle::HullSection section{4.5, 10.0};
    const double tb = 90.19;
    const auto env = deep_space_sun({0.0, 1.0, 0.0});
    const double emitted = 0.15 * constants::stefan_boltzmann_w_m2k4 * std::pow(tb, 4) * constants::two_pi * 4.5 * 10.0;
    CHECK(vehicle::section_heat_w(section, bare, 1.5 * constants::pi, tb, env, {}) ==
          Catch::Approx(0.4 * constants::solar_constant_w_m2 * 9.0 * 10.0 - emitted).epsilon(5e-3));
    // End-on: the walls see no sun and lose their emission - negative.
    CHECK(vehicle::section_heat_w(section, bare, 1.5 * constants::pi, tb, deep_space_sun({1.0, 0.0, 0.0}), {}) ==
          Catch::Approx(-emitted).epsilon(1e-3));
}

TEST_CASE("boil_off_rates: hull main tanks only, heat over latent heat, clamped at zero", "[boil_off][vehicle]") {
    const auto ship = top_stage_alone("starship_block3");
    const auto sunlit = vehicle::boil_off_rates(*ship, deep_space_sun({0.0, 0.0, 1.0}), {});  // sun on the dorsal side
    REQUIRE(sunlit.size() == 2);  // lox_main, ch4_main - no headers
    for (const auto& t : sunlit) {
        const auto& tank = ship->stages()[t.stage_index].tanks[t.tank_index];
        CHECK(tank.config.role == config::TankRole::Main);
        CHECK(t.heat_w > 0.0);
        CHECK(t.mass_flow_kgps == Catch::Approx(t.heat_w / tank.latent_heat_j_kg));
    }
    // Sun along the axis: the walls lose heat - no condensation, flows clamp at 0.
    for (const auto& t : vehicle::boil_off_rates(*ship, deep_space_sun({1.0, 0.0, 0.0}), {})) {
        CHECK(t.heat_w == 0.0);
        CHECK(t.mass_flow_kgps == 0.0);
    }
    const auto total = vehicle::total_boil_off(*ship, sunlit);
    CHECK(total.oxidizer_kgps > 0.0);
    CHECK(total.fuel_kgps > 0.0);
}

TEST_CASE("boil_off_rates: tiles on the belly, MLI on the depot, nothing without stage.thermal", "[boil_off][vehicle]") {
    const auto ship = top_stage_alone("starship_block3");
    auto heat = [](const vehicle::Vehicle& v, const Vec3& sun) {
        double sum = 0.0;
        for (const auto& t : vehicle::boil_off_rates(v, deep_space_sun(sun), {})) sum += t.heat_w;
        return sum;
    };
    CHECK(heat(*ship, {0.0, 0.0, -1.0}) < 0.5 * heat(*ship, {0.0, 0.0, 1.0}));  // belly (tiles) vs dorsal (bare)
    const auto depot = top_stage_alone("starship_depot");
    CHECK(heat(*depot, {0.0, 0.0, 1.0}) < 0.02 * heat(*ship, {0.0, 0.0, 1.0}));
    const auto dragon = top_stage_alone("crew_dragon");
    CHECK(vehicle::boil_off_rates(*dragon, deep_space_sun({0.0, 0.0, 1.0}), {}).empty());
}

namespace {

// Centaur LO2 sidewall in a sunlit park-orbit coast (AIAA 2007-5845): axis along the orbit normal, rolling (the strip
// sum is roll-independent), the Sun in the orbit plane at noon in mid-coast [E]; mean wall flux over the coast from
// perigee. Mirrors tools/validation/centaur_boil_off_calibration.py.
double centaur_wall_flux_w_m2(const config::ThermalSurfaceConfig& surface, double perigee_km, double apogee_km,
                              double coast_min, bool earth) {
    const double re = constants::earth_mean_radius_m, mu = constants::earth_gm_m3ps2;
    const double rp = re + perigee_km * 1e3, ra = re + apogee_km * 1e3;
    const double a = 0.5 * (rp + ra), e = (ra - rp) / (ra + rp), n = std::sqrt(mu / (a * a * a));
    auto at = [&](double t, double& r, double& nu) {
        double ecc_anomaly = n * t;
        for (int k = 0; k < 50; ++k) {
            ecc_anomaly -= (ecc_anomaly - e * std::sin(ecc_anomaly) - n * t) / (1.0 - e * std::cos(ecc_anomaly));
        }
        r = a * (1.0 - e * std::cos(ecc_anomaly));
        nu = 2.0 * std::atan2(std::sqrt(1.0 + e) * std::sin(0.5 * ecc_anomaly),
                              std::sqrt(1.0 - e) * std::cos(0.5 * ecc_anomaly));
    };
    double r_mid = 0.0, nu_mid = 0.0;
    at(0.5 * coast_min * 60.0, r_mid, nu_mid);
    const config::ThermalConfig thermal{surface, std::nullopt, 0.0};
    const vehicle::HullSection section{1.524, 1.0};  // 10 ft tank [S]; the length cancels
    // Body x (the tank axis) -> frame z (the orbit normal); body y/z -> the orbit plane.
    const math::Quat body_to_frame = math::Quat::from_axis_angle({0.0, 1.0, 0.0}, -0.5 * constants::pi);
    constexpr int samples = 60;
    double sum = 0.0;
    for (int k = 0; k < samples; ++k) {
        double r = 0.0, nu = 0.0;
        at((k + 0.5) / samples * coast_min * 60.0, r, nu);
        environment::RadiationEnvironment env;
        env.sun_direction = {std::cos(nu_mid), std::sin(nu_mid), 0.0};
        env.solar_flux_w_m2 = constants::solar_constant_w_m2;
        env.sunlit_fraction = 1.0;  // the reconstructions cover the sunlit part
        env.nadir_direction = {-std::cos(nu), -std::sin(nu), 0.0};
        env.distance_over_radius = earth ? r / re : 1.0e6;
        env.albedo_phase = earth ? std::max(0.0, std::cos(nu - nu_mid)) : 0.0;
        sum += vehicle::section_heat_w(section, thermal, 0.0, 90.19, env, body_to_frame) /
               (constants::two_pi * section.radius_m * section.length_m);
    }
    return sum / samples;
}

constexpr double btu_hr_ft2 = 3.15459;  // W/m^2 [S: NIST SP 811]

}  // namespace

TEST_CASE("Centaur LO2 sidewall flux: calibration and prediction (AIAA 2007-5845 Table 2)", "[boil_off][validation]") {
    // White-painted foam [E: paint alpha 0.25, eps 0.88]; resistance calibrated [D] on AC-206 (23 Btu/hr-ft^2).
    const config::ThermalSurfaceConfig foam{0.25, 0.88, 1.586};
    const double ac206 = centaur_wall_flux_w_m2(foam, 167.0, 1132.0, 58.0, true);
    const double ac167 = centaur_wall_flux_w_m2(foam, 185.0, 267.0, 59.0, true);
    const double av003 = centaur_wall_flux_w_m2(foam, 167.0, 4165.0, 78.0, true);
    // 3-layer MLI [E: cover alpha 0.30, eps 0.80]; calibrated [D] on the TC-11/TC-15 average (3 Btu/hr-ft^2), high
    // elliptical transfer coasts: sun only.
    const double tc = centaur_wall_flux_w_m2({0.30, 0.80, 8.22}, 0.0, 0.0, 1.0, false);
    if (verbose()) {
        std::cout << "Centaur flux W/m^2: AC-206 " << ac206 << " AC-167 " << ac167 << " AV-003 " << av003 << " TC "
                  << tc << "\n";
    }
    CHECK(ac206 == Catch::Approx(23.0 * btu_hr_ft2).epsilon(0.03));  // calibration consistency
    CHECK(tc == Catch::Approx(3.0 * btu_hr_ft2).epsilon(0.03));
    // Validation: other orbits with AC-206's wall. Tolerance 25 % [documented: TC-11 vs TC-15 differ by 2x, coast
    // geometry unknown].
    CHECK(ac167 == Catch::Approx(25.0 * btu_hr_ft2).epsilon(0.25));
    CHECK(av003 == Catch::Approx(25.0 * btu_hr_ft2).epsilon(0.25));
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[boil_off]" > tmp/r2_t3_red.txt 2>&1`, read the tail.
Expected: build error - `starsim/vehicle/boil_off.hpp` not found.

- [ ] **Step 3: Implement**

Create `core/include/starsim/vehicle/boil_off.hpp`:

```cpp
#pragma once

#include <cstddef>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/thermal_radiation.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::vehicle {

// Refuelling r2: propellant boil-off from a quasi-steady heat balance on the hull. Every hull tank of a stage with
// [stage.thermal] (a main tank of the stage's diameter) is a cylinder of its full-load length, split into
// boil_off_strips strips around the circumference. Per strip: absorbed (Sun, Earth albedo, Earth infrared) =
// emissivity sigma T^4 + (T - T_boil) / R_wall. The leaks (T - T_boil) / R_wall over the tank's strips are summed;
// that heat (clamped at 0: a tank losing heat through its wall does not condense) over the latent heat is the vented
// mass flow.
// Deliberate simplifications [docs/subsystems.md]: no skin heat capacity and no conduction around the circumference;
// no tank pressure, temperature, subcooling or stratification; a tank's whole wall heat reaches its liquid whatever
// the fill level; domes, ends, header and RCS tanks take no heat; no shadows of other vehicles or of the flaps.
inline constexpr int boil_off_strips = 36;

struct BoilOffRate {
    double oxidizer_kgps = 0.0;
    double fuel_kgps = 0.0;
};

struct TankBoilOff {
    size_t stage_index = 0;
    size_t tank_index = 0;
    double heat_w = 0.0;          // into the propellant (>= 0)
    double mass_flow_kgps = 0.0;  // vented
};

// A cylinder section of the hull along the body x axis.
struct HullSection {
    double radius_m = 0.0;
    double length_m = 0.0;
};

// Wall heat flux into the propellant (W/m^2; negative when the wall loses heat) of a strip absorbing
// `absorbed_w_m2`: solves emissivity sigma T^4 + (T - boiling_point) / resistance = absorbed for the skin temperature.
double strip_heat_leak_w_m2(double absorbed_w_m2, const config::ThermalSurfaceConfig& surface, double boiling_point_k);

// Net heat (W, may be negative) through the wall of a section. Strips within the shield's half width of
// `belly_azimuth_rad` (azimuth from body +y towards +z) use the shield surface. `body_to_frame` turns body axes into
// the environment's frame.
double section_heat_w(const HullSection& section, const config::ThermalConfig& thermal, double belly_azimuth_rad,
                      double boiling_point_k, const environment::RadiationEnvironment& env,
                      const math::Quat& body_to_frame);

// Heat and vented flow of every boiling hull tank (holding propellant) of the vehicle's stages with [stage.thermal].
std::vector<TankBoilOff> boil_off_rates(const Vehicle& vehicle, const environment::RadiationEnvironment& env,
                                        const math::Quat& body_to_frame);

// The flows summed by contents.
BoilOffRate total_boil_off(const Vehicle& vehicle, const std::vector<TankBoilOff>& tanks);

}  // namespace starsim::vehicle
```

Create `core/src/vehicle/boil_off.cpp`:

```cpp
#include "starsim/vehicle/boil_off.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::vehicle {

using math::Vec3;

double strip_heat_leak_w_m2(double absorbed_w_m2, const config::ThermalSurfaceConfig& surface,
                            double boiling_point_k) {
    const double es = surface.emissivity * constants::stefan_boltzmann_w_m2k4;
    const double r = surface.wall_resistance_m2k_w;
    const double tb = boiling_point_k;
    const double a = std::max(0.0, absorbed_w_m2);
    // f(T) = es T^4 + (T - tb) / r - a is convex and increasing: Newton from a point with f >= 0 converges
    // monotonically from the right. Both candidates have f >= 0; take the nearer one.
    double t = std::min(tb + a * r, std::pow(a / es, 0.25) + tb);
    for (int k = 0; k < 100; ++k) {
        const double f = es * t * t * t * t + (t - tb) / r - a;
        const double step = f / (4.0 * es * t * t * t + 1.0 / r);
        t -= step;
        if (std::abs(step) <= 1e-12 * t) break;
    }
    return (t - tb) / r;
}

double section_heat_w(const HullSection& section, const config::ThermalConfig& thermal, double belly_azimuth_rad,
                      double boiling_point_k, const environment::RadiationEnvironment& env,
                      const math::Quat& body_to_frame) {
    const double strip_area_m2 = constants::two_pi * section.radius_m * section.length_m / boil_off_strips;
    double heat_w = 0.0;
    for (int k = 0; k < boil_off_strips; ++k) {
        const double azimuth = constants::two_pi * (k + 0.5) / boil_off_strips;
        const Vec3 normal = body_to_frame.rotate(Vec3{0.0, std::cos(azimuth), std::sin(azimuth)});
        const bool on_shield = thermal.shield.has_value() &&
                               std::abs(std::remainder(azimuth - belly_azimuth_rad, constants::two_pi)) <=
                                   thermal.shield_half_width_rad;
        const auto& surface = on_shield ? *thermal.shield : thermal.hull;
        const double absorbed = environment::absorbed_flux_w_m2(normal, env, surface.absorptivity, surface.emissivity);
        heat_w += strip_area_m2 * strip_heat_leak_w_m2(absorbed, surface, boiling_point_k);
    }
    return heat_w;
}

std::vector<TankBoilOff> boil_off_rates(const Vehicle& vehicle, const environment::RadiationEnvironment& env,
                                        const math::Quat& body_to_frame) {
    std::vector<TankBoilOff> out;
    const auto& stages = vehicle.stages();
    for (size_t s = 0; s < stages.size(); ++s) {
        const auto& stage = stages[s];
        if (!stage.config.thermal) continue;
        for (size_t k = 0; k < stage.tanks.size(); ++k) {
            const auto& tank = stage.tanks[k];
            const bool hull_tank = tank.config.role == config::TankRole::Main && tank.config.diameter_m == 0.0 &&
                                   tank.config.contents != config::TankContents::Rcs;
            if (!hull_tank || tank.latent_heat_j_kg <= 0.0 || tank.mass_kg <= 0.0 || tank.length_m <= 0.0) continue;
            const double heat_w = std::max(0.0, section_heat_w({tank.radius_m, tank.length_m}, *stage.config.thermal,
                                                               stage.config.aero.belly_azimuth_rad,
                                                               tank.boiling_point_k, env, body_to_frame));
            out.push_back({s, k, heat_w, heat_w / tank.latent_heat_j_kg});
        }
    }
    return out;
}

BoilOffRate total_boil_off(const Vehicle& vehicle, const std::vector<TankBoilOff>& tanks) {
    BoilOffRate rate;
    for (const auto& t : tanks) {
        const auto contents = vehicle.stages()[t.stage_index].tanks[t.tank_index].config.contents;
        (contents == config::TankContents::Oxidizer ? rate.oxidizer_kgps : rate.fuel_kgps) += t.mass_flow_kgps;
    }
    return rate;
}

}  // namespace starsim::vehicle
```

Add `src/vehicle/boil_off.cpp` to `core/CMakeLists.txt` after `src/vehicle/compound_mass.cpp`.

Create `tools/validation/centaur_boil_off_calibration.py` (offline, reproduces the [D] values in the configs and the
test; no dependencies):

```python
"""Calibrates the boil-off wall resistances on Centaur flight reconstructions (refuelling r2).

Source: De Kruif & Kutter, AIAA 2007-5845, Table 2 - LO2 sidewall flux in sunlit coasts, broadside to the Sun, rolling.
Mirrors core/tests/test_boil_off.cpp (centaur_wall_flux_w_m2): Earth of mean radius, 36 strips, 60 time samples.
Prints the white-foam and 3-layer MLI resistances [D], the predictions for AC-167 / AV-003, and the depot value
(30 effective layers, leak ~ 1/(n+1)).
"""
import math

SIGMA = 5.670374419e-8
S = 1361.0
ALBEDO = 0.30
IR = 239.0
RE = 6371000.0
MU = 3.986004418e14
TB = 90.19
BTU = 3.15459  # W/m^2 per Btu/hr-ft^2
STRIPS = 36
SAMPLES = 60


def plate_view_factor(angle, h):
    full = math.acos(1.0 / h)
    if angle <= full:
        return math.cos(angle) / h**2
    if angle >= math.pi - full:
        return 0.0
    s = math.sqrt(h * h - 1.0)
    rim = math.sqrt(max(0.0, 1.0 - h * h * math.cos(angle) ** 2))
    return (0.5 - math.asin(min(1.0, s / (h * math.sin(angle)))) / math.pi
            + (math.cos(angle) * math.acos(max(-1.0, min(1.0, -s / math.tan(angle)))) - s * rim) / (math.pi * h * h))


def strip_leak(absorbed, eps, r):
    lo, hi = 1.0, 3000.0
    for _ in range(200):
        t = 0.5 * (lo + hi)
        if eps * SIGMA * t**4 + (t - TB) / r - absorbed > 0.0:
            hi = t
        else:
            lo = t
    return (t - TB) / r


def orbit_at(rp, ra, t):
    a = 0.5 * (rp + ra)
    e = (ra - rp) / (ra + rp)
    n = math.sqrt(MU / a**3)
    ecc = n * t
    for _ in range(50):
        ecc -= (ecc - e * math.sin(ecc) - n * t) / (1.0 - e * math.cos(ecc))
    nu = 2.0 * math.atan2(math.sqrt(1 + e) * math.sin(ecc / 2), math.sqrt(1 - e) * math.cos(ecc / 2))
    return a * (1.0 - e * math.cos(ecc)), nu


def wall_flux(alpha, eps, r, perigee_km, apogee_km, coast_min, earth=True):
    rp, ra = RE + perigee_km * 1e3, RE + apogee_km * 1e3
    _, nu_mid = orbit_at(rp, ra, 0.5 * coast_min * 60.0)
    total = 0.0
    for k in range(SAMPLES):
        radius, nu = orbit_at(rp, ra, (k + 0.5) / SAMPLES * coast_min * 60.0)
        h = radius / RE if earth else 1.0e6
        phase = max(0.0, math.cos(nu - nu_mid)) if earth else 0.0
        acc = 0.0
        for i in range(STRIPS):
            phi = (i + 0.5) / STRIPS * 2.0 * math.pi  # strip normal in the orbit plane
            to_nadir = math.atan2(math.sin(phi - nu - math.pi), math.cos(phi - nu - math.pi))
            view = plate_view_factor(abs(to_nadir), h)
            absorbed = alpha * (S * max(0.0, math.cos(phi - nu_mid)) + ALBEDO * S * view * phase) + eps * IR * view
            acc += strip_leak(absorbed, eps, r)
        total += acc / STRIPS
    return total / SAMPLES


def solve_resistance(target, flux):
    lo, hi = 1e-3, 1e4
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        if flux(mid) > target:
            lo = mid
        else:
            hi = mid
    return mid


if __name__ == "__main__":
    paint = (0.25, 0.88)
    foam = solve_resistance(23 * BTU, lambda r: wall_flux(*paint, r, 167, 1132, 58))
    print(f"white foam resistance (AC-206) = {foam:.3f} m^2K/W")
    for name, hp, ha, minutes, measured in [("AC-167", 185, 267, 59, 25), ("AV-003", 167, 4165, 78, 25)]:
        q = wall_flux(*paint, foam, hp, ha, minutes)
        print(f"{name}: model {q:.1f} W/m^2, flight {measured * BTU:.1f} ({q / (measured * BTU) - 1:+.1%})")
    mli3 = solve_resistance(3 * BTU, lambda r: wall_flux(0.30, 0.80, r, 0, 0, 1, earth=False))
    print(f"3-layer MLI resistance (TC average) = {mli3:.2f} m^2K/W")
    print(f"depot, 30 effective layers = {mli3 * 31 / 4:.1f} m^2K/W")
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[boil_off]" > tmp/r2_t3.txt 2>&1`, read the tail.
Expected: all pass. Run `python tools/validation/centaur_boil_off_calibration.py`: foam ~1.586, MLI ~8.22, depot ~63.7,
AC-167 ~-3 %, AV-003 ~-20 %. If the C++ AV-003 check misses the 25 % band, report the number - do not change the
tolerance or the [E] optics to make it pass.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/vehicle/boil_off.hpp core/src/vehicle/boil_off.cpp core/CMakeLists.txt tools/validation/centaur_boil_off_calibration.py core/tests/test_boil_off.cpp
git commit -m "feat(vehicle): boil-off heat balance on the hull, validated on Centaur sidewall fluxes"
git push origin main
```

---

### Task 4: Orbit-averaged boil-off

**Files:**
- Modify: `core/include/starsim/vehicle/boil_off.hpp`, `core/src/vehicle/boil_off.cpp`
- Test: `core/tests/test_boil_off.cpp`

**Interfaces:**
- Consumes: Task 3 `boil_off_rates`, `total_boil_off`; `orbit::elements_from_state`, `orbit::orbital_period_s`,
  `orbit::propagate_two_body` (`orbit/kepler.hpp`); `frames::lvlh_to_inertial` (`frames/lvlh.hpp`).
- Produces: `BoilOffRate orbit_average_boil_off(const Vehicle& vehicle, const math::StateVector& orbit_gcrf, const math::Quat& body_in_lvlh, const math::Vec3& sun_position_gcrf_m, int samples = 360);`
  - mean vented flow (kg/s) over one two-body period about the Earth, the vehicle held at `body_in_lvlh` (body ->
    LVLH of `frames/lvlh.hpp`: x along the velocity, z nadir), Sun fixed over the orbit, Earth shadow only, tank
    masses as they are (no depletion inside the orbit).

- [ ] **Step 1: Write the failing tests**

Append to `core/tests/test_boil_off.cpp` (add `#include "starsim/orbit/kepler.hpp"`, `#include "starsim/frames/lvlh.hpp"`):

```cpp
namespace {

// Circular orbit of `altitude_m`, inclination `inclination_rad`, node on +x.
math::StateVector circular_orbit(double altitude_m, double inclination_rad) {
    const double r = constants::earth_mean_radius_m + altitude_m;
    const double v = std::sqrt(constants::earth_gm_m3ps2 / r);
    return {{r, 0.0, 0.0}, {0.0, v * std::cos(inclination_rad), v * std::sin(inclination_rad)}};
}

double full_propellant_kg(const vehicle::Vehicle& v) {
    double sum = 0.0;
    for (const auto& t : v.stages().back().tanks) {
        if (t.config.contents != config::TankContents::Rcs) sum += t.config.capacity_kg;
    }
    return sum;
}

}  // namespace

TEST_CASE("orbit average: a Sun along the orbit normal gives the constant instantaneous rate", "[boil_off][vehicle]") {
    // Equatorial orbit, Sun at the north pole: no eclipse, and in LVLH the Sun and the Earth stand still.
    const auto ship = top_stage_alone("starship_tanker");
    const auto orbit = circular_orbit(300.0e3, 0.0);
    const Vec3 sun{0.0, 0.0, constants::astronomical_unit_m};
    const auto average = vehicle::orbit_average_boil_off(*ship, orbit, {}, sun);
    const auto attitude = frames::lvlh_to_inertial(orbit);
    const auto now = vehicle::total_boil_off(
        *ship, vehicle::boil_off_rates(*ship,
                                       environment::earth_orbit_radiation(orbit.position_m, sun - orbit.position_m,
                                                                          nullptr),
                                       attitude));
    CHECK(average.oxidizer_kgps == Catch::Approx(now.oxidizer_kgps).epsilon(1e-6));
    CHECK(average.fuel_kgps == Catch::Approx(now.fuel_kgps).epsilon(1e-6));
}

TEST_CASE("orbit average: eclipse lowers the rate", "[boil_off][vehicle]") {
    const auto ship = top_stage_alone("starship_tanker");
    const auto orbit = circular_orbit(300.0e3, 0.0);
    const auto in_plane = vehicle::orbit_average_boil_off(*ship, orbit, {}, {constants::astronomical_unit_m, 0.0, 0.0});
    const auto polar_sun = vehicle::orbit_average_boil_off(*ship, orbit, {}, {0.0, 0.0, constants::astronomical_unit_m});
    // In the plane the Sun is eclipsed part of the orbit and passes along the axis (end-on); from the pole it stays
    // broadside and never sets.
    CHECK(in_plane.oxidizer_kgps > 0.0);
    CHECK(in_plane.oxidizer_kgps + in_plane.fuel_kgps < polar_sun.oxidizer_kgps + polar_sun.fuel_kgps);
}

TEST_CASE("orbit average in the depot orbit: plausibility of depot and bare tanker", "[boil_off][vehicle]") {
    // 200 km, 26.5 deg (the lunar missions' start orbit), Sun in the orbit plane (the longest eclipse) and at 60 deg
    // beta; depot and tanker nose along +V, dorsal to nadir (identity in LVLH, r3's depot attitude).
    const auto orbit = circular_orbit(200.0e3, 26.5 * constants::deg_to_rad);
    const auto depot = top_stage_alone("starship_depot");
    const auto tanker = top_stage_alone("starship_tanker");
    for (const Vec3 sun_dir : {Vec3{1.0, 0.0, 0.0}, Vec3{0.5, -0.866 * std::sin(26.5 * constants::deg_to_rad),
                                                         0.866 * std::cos(26.5 * constants::deg_to_rad)}}) {
        const Vec3 sun = sun_dir * constants::astronomical_unit_m;
        const auto d = vehicle::orbit_average_boil_off(*depot, orbit, {}, sun);
        const auto t = vehicle::orbit_average_boil_off(*tanker, orbit, {}, sun);
        const double depot_kg_day = (d.oxidizer_kgps + d.fuel_kgps) * constants::seconds_per_day;
        const double tanker_kg_day = (t.oxidizer_kgps + t.fuel_kgps) * constants::seconds_per_day;
        const double depot_percent_day = 100.0 * depot_kg_day / full_propellant_kg(*depot);
        if (verbose()) {
            std::cout << "depot " << depot_kg_day << " kg/day (" << depot_percent_day << " %/day), tanker "
                      << tanker_kg_day << " kg/day\n";
        }
        // Plausibility only [documented]: conventionally insulated cryogenic tanks in orbit lose ~0.1 %/day; a large
        // depot with MLI less. A bare tanker loses orders of magnitude more.
        CHECK(depot_percent_day > 0.005);
        CHECK(depot_percent_day < 0.2);
        CHECK(tanker_kg_day > 10.0 * depot_kg_day);
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[boil_off]" > tmp/r2_t4_red.txt 2>&1`, read the tail.
Expected: build error - `orbit_average_boil_off` not declared.

- [ ] **Step 3: Implement**

`boil_off.hpp`, add `#include "starsim/math/state_vector.hpp"` and after `total_boil_off`:

```cpp
// Mean vented flow over one two-body orbit about the Earth (the r6 campaign calculator's daily losses): the vehicle
// held at `body_in_lvlh` (body -> LVLH of frames/lvlh.hpp: x along the velocity, z nadir), the Sun fixed at
// `sun_position_gcrf_m` over the orbit, Earth shadow only, tank masses as they are.
BoilOffRate orbit_average_boil_off(const Vehicle& vehicle, const math::StateVector& orbit_gcrf,
                                   const math::Quat& body_in_lvlh, const math::Vec3& sun_position_gcrf_m,
                                   int samples = 360);
```

`boil_off.cpp`, add `#include "starsim/frames/lvlh.hpp"` and `#include "starsim/orbit/kepler.hpp"`, then:

```cpp
BoilOffRate orbit_average_boil_off(const Vehicle& vehicle, const math::StateVector& orbit_gcrf,
                                   const math::Quat& body_in_lvlh, const Vec3& sun_position_gcrf_m, int samples) {
    const double gm = constants::earth_gm_m3ps2;
    const double period_s = orbit::orbital_period_s(orbit::elements_from_state(orbit_gcrf, gm).semi_major_axis_m, gm);
    BoilOffRate mean;
    for (int k = 0; k < samples; ++k) {
        const auto state = orbit::propagate_two_body(orbit_gcrf, gm, period_s * k / samples);
        const auto env = environment::earth_orbit_radiation(state.position_m, sun_position_gcrf_m - state.position_m,
                                                            nullptr);
        const auto rate =
            total_boil_off(vehicle, boil_off_rates(vehicle, env, frames::lvlh_to_inertial(state) * body_in_lvlh));
        mean.oxidizer_kgps += rate.oxidizer_kgps / samples;
        mean.fuel_kgps += rate.fuel_kgps / samples;
    }
    return mean;
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[boil_off]" > tmp/r2_t4.txt 2>&1`; then
once with `$env:STARSIM_VERBOSE=1` and `-TestFilter "[boil_off]"` into `tmp/r2_t4_verbose.txt`, grep `depot`/`Centaur`.
Expected: all pass; record the depot (kg/day, %/day) and tanker (kg/day) numbers for both Sun directions in the
progress ledger - they go into `docs/subsystems.md` in Task 6. If the depot plausibility band fails, report the
number to the user (it is a plausibility check, not a tuning target).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/vehicle/boil_off.hpp core/src/vehicle/boil_off.cpp core/tests/test_boil_off.cpp
git commit -m "feat(vehicle): orbit-averaged boil-off for an LVLH-held vehicle"
git push origin main
```

---

### Task 5: Boil-off in flight

**Files:**
- Modify: `core/include/starsim/vehicle/flight_body.hpp` (`FlightEnvironment` after `actuator_moments_about_cg_point`;
  `FlightBody` public block after `rcs_force_body_n()`; private members)
- Modify: `core/src/vehicle/flight_body.cpp` (`step` free-flight end ~line 442, `coast` end ~line 557, `separate` ~line 560,
  new `FlightBody::boil_off`)
- Modify: `core/include/starsim/sim/flight_session.hpp` (next to `set_solar_radiation_pressure`, ~line 581)
- Test: `core/tests/test_boil_off.cpp`

**Interfaces:**
- Consumes: Task 1 `earth_orbit_radiation`; Task 3 `boil_off_rates`, `total_boil_off`, `TankBoilOff`, `BoilOffRate`;
  Task 4 `orbit_average_boil_off` (test reference); r1 `Vehicle::set_tank_mass`.
- Produces:
  - `bool FlightEnvironment::boil_off = true;` (fidelity flag)
  - `const BoilOffRate& FlightBody::boil_off_rate() const;` (flows at the last evaluation; zero when not boiling)
  - `static constexpr double FlightBody::boil_off_interval_s = 10.0;`, `FlightBody::boil_off_min_altitude_m = 100.0e3;`
  - `void FlightSession::set_boil_off(bool on);`

- [ ] **Step 1: Write the failing tests**

Append to `core/tests/test_boil_off.cpp` (add `#include "starsim/vehicle/flight_body.hpp"`,
`#include "starsim/environment/atmosphere.hpp"`, `#include "starsim/ephemeris/ephemeris.hpp"`):

```cpp
namespace {

class CentralGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3& r, time::Epoch) const override {
        const double d = norm(r);
        return r * (-constants::earth_gm_m3ps2 / (d * d * d));
    }
};

class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};

// The Sun fixed in GCRF; the Moon opposite it (never in front of the Sun for a vehicle near the Earth).
class FixedSun final : public ephemeris::Ephemeris {
public:
    explicit FixedSun(const Vec3& sun_gcrf_m) : sun_(sun_gcrf_m) {}
    math::StateVector state_gcrf(ephemeris::Body target, time::Epoch) const override {
        if (target == ephemeris::Body::Sun) return {sun_, {}};
        if (target == ephemeris::Body::Moon) return {sun_ * (-3.844e8 / norm(sun_)), {}};
        return {};
    }
    double gm_m3ps2(ephemeris::Body) const override { return 0.0; }
    math::Vec3 apparent_position_gcrf(ephemeris::Body target, time::Epoch epoch,
                                      const math::StateVector&) const override {
        return state_gcrf(target, epoch).position_m;
    }

private:
    Vec3 sun_;
};

const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);

struct OrbitWorld {
    explicit OrbitWorld(const Vec3& sun) : ephemeris(sun) {}
    CentralGravity gravity;
    environment::Us76Atmosphere atmosphere;
    FixedEarth earth;
    FixedSun ephemeris;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, &ephemeris, false};
};

// A vehicle's top stage alone in a circular orbit, held in LVLH (identity: nose along +V, dorsal to nadir).
std::unique_ptr<vehicle::FlightBody> in_orbit(const std::string& vehicle_id, const math::StateVector& orbit) {
    auto body = std::make_unique<vehicle::FlightBody>(
        top_stage_alone(vehicle_id),
        vehicle::RigidBodyState{orbit.position_m, orbit.velocity_mps, frames::lvlh_to_inertial(orbit), {}});
    body->set_ideal_lvlh_attitude(math::Quat{});
    return body;
}

double main_propellant(const vehicle::FlightBody& body) { return body.vehicle().main_propellant_kg(); }

}  // namespace

TEST_CASE("boil-off in flight: one orbit of coasting vents the orbit average", "[boil_off][flight]") {
    const auto orbit = circular_orbit(400.0e3, 0.0);
    const Vec3 sun{constants::astronomical_unit_m, 0.0, 0.0};  // in the plane: an eclipse every orbit
    OrbitWorld world(sun);
    auto ship = in_orbit("starship_tanker", orbit);
    const auto average = vehicle::orbit_average_boil_off(ship->vehicle(), orbit, {}, sun);
    const double period_s = orbit::orbital_period_s(norm(orbit.position_m), constants::earth_gm_m3ps2);
    const double before = main_propellant(*ship);
    constexpr int segments = 60;
    for (int k = 0; k < segments; ++k) ship->coast(world.env, t0 + period_s * k / segments, period_s / segments);
    const double vented = before - main_propellant(*ship);
    CHECK(vented == Catch::Approx((average.oxidizer_kgps + average.fuel_kgps) * period_s).epsilon(0.03));
    CHECK(ship->boil_off_rate().oxidizer_kgps > 0.0);
}

TEST_CASE("boil-off in flight: the flag, vehicles without stage.thermal and no ephemeris change nothing",
          "[boil_off][flight]") {
    const auto orbit = circular_orbit(400.0e3, 0.0);
    OrbitWorld world({0.0, constants::astronomical_unit_m, 0.0});
    world.env.boil_off = false;
    auto ship = in_orbit("starship_tanker", orbit);
    const double ship_before = main_propellant(*ship);
    ship->coast(world.env, t0, 600.0);
    CHECK(main_propellant(*ship) == ship_before);

    world.env.boil_off = true;
    auto dragon = in_orbit("crew_dragon", orbit);
    double dragon_before = 0.0;
    for (const auto& t : dragon->vehicle().stages().back().tanks) dragon_before += t.mass_kg;
    dragon->coast(world.env, t0, 600.0);
    double dragon_after = 0.0;
    for (const auto& t : dragon->vehicle().stages().back().tanks) dragon_after += t.mass_kg;
    CHECK(dragon_after == dragon_before);

    vehicle::FlightEnvironment no_ephemeris{world.gravity, world.atmosphere, world.earth, nullptr, false};
    ship->coast(no_ephemeris, t0, 600.0);
    CHECK(main_propellant(*ship) == ship_before);
}

TEST_CASE("boil-off in flight: ticks and one long coast vent the same mass", "[boil_off][flight]") {
    // Sun along the orbit normal: steady sunlight, constant rate - the cadence must not matter.
    const auto orbit = circular_orbit(400.0e3, 0.0);
    OrbitWorld world({0.0, 0.0, constants::astronomical_unit_m});
    auto long_coast = in_orbit("starship_tanker", orbit);
    auto short_coasts = in_orbit("starship_tanker", orbit);
    auto ticks = in_orbit("starship_tanker", orbit);
    const double before = main_propellant(*long_coast);
    long_coast->coast(world.env, t0, 600.0);
    for (int k = 0; k < 60; ++k) short_coasts->coast(world.env, t0 + 10.0 * k, 10.0);
    for (int k = 0; k < 600; ++k) ticks->step(world.env, t0 + 0.1 * k, 0.1, {});  // 60 s of 200 Hz-like ticks
    const double rate = long_coast->boil_off_rate().oxidizer_kgps + long_coast->boil_off_rate().fuel_kgps;
    CHECK(before - main_propellant(*long_coast) == Catch::Approx(rate * 600.0).epsilon(1e-3));
    CHECK(before - main_propellant(*short_coasts) == Catch::Approx(before - main_propellant(*long_coast)).epsilon(5e-3));
    CHECK(before - main_propellant(*ticks) == Catch::Approx(rate * 60.0).epsilon(5e-3));
}

TEST_CASE("boil-off empties a tank to zero, never below", "[boil_off][flight]") {
    const auto orbit = circular_orbit(400.0e3, 0.0);
    OrbitWorld world({0.0, 0.0, constants::astronomical_unit_m});
    auto ship = in_orbit("starship_tanker", orbit);
    ship->vehicle().set_tank_mass(0, "lox_main", 1.0);
    ship->coast(world.env, t0, 600.0);
    const auto& lox = tank_named(ship->vehicle(), "lox_main");
    CHECK(lox.mass_kg == 0.0);
    CHECK(std::isfinite(ship->vehicle().mass_properties().mass_kg));
}

TEST_CASE("boil-off: separation clears the stored rates, the booster does not boil", "[boil_off][flight]") {
    const auto orbit = circular_orbit(400.0e3, 0.0);
    OrbitWorld world({0.0, 0.0, constants::astronomical_unit_m});
    auto cfg = config::load_vehicle(config_dir, "starship_tanker");
    auto stack = std::make_unique<vehicle::FlightBody>(
        std::make_unique<vehicle::Vehicle>(cfg, config_dir),
        vehicle::RigidBodyState{orbit.position_m, orbit.velocity_mps, frames::lvlh_to_inertial(orbit), {}});
    stack->coast(world.env, t0, 20.0);  // rates stored for stage 1 (the ship)
    auto ship = stack->separate(1);
    double booster_before = 0.0;
    for (const auto& t : stack->vehicle().stages().front().tanks) booster_before += t.mass_kg;
    const double ship_before = main_propellant(*ship);
    stack->coast(world.env, t0 + 20.0, 20.0);
    ship->coast(world.env, t0 + 20.0, 20.0);
    double booster_after = 0.0;
    for (const auto& t : stack->vehicle().stages().front().tanks) booster_after += t.mass_kg;
    CHECK(booster_after == booster_before);
    CHECK(main_propellant(*ship) < ship_before);
}

TEST_CASE("boil-off: not on the pad and not inside the air", "[boil_off][flight]") {
    OrbitWorld world({constants::astronomical_unit_m, 0.0, 0.0});
    // 50 km up, coasting: below boil_off_min_altitude_m.
    const auto low = circular_orbit(50.0e3, 0.0);
    auto ship = in_orbit("starship_tanker", low);
    const double before = main_propellant(*ship);
    ship->coast(world.env, t0, 30.0);
    CHECK(main_propellant(*ship) == before);
    CHECK(ship->boil_off_rate().oxidizer_kgps == 0.0);
}
```

(`Vehicle::main_propellant_kg()` exists (r1 notes). `circular_orbit`, `tank_named`, `top_stage_alone`, `verbose`
come from the earlier tasks of this file. If `FlightBody`'s state struct or `step`'s command type differ, follow
`core/tests/test_flight_body.cpp`. The pad case of the spec ("not clamped") is covered by the code path: the clamped
branch of `step` returns before `boil_off`.)

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[boil_off]" > tmp/r2_t5_red.txt 2>&1`, read the tail.
Expected: build error - `boil_off` / `boil_off_rate` not members.

- [ ] **Step 3: Implement**

`flight_body.hpp`: add `#include <vector>` and `#include "starsim/vehicle/boil_off.hpp"`. In `FlightEnvironment` after
`actuator_moments_about_cg_point`:

```cpp
    // Fidelity flag (refuelling r2): cryogenic propellant in the hull tanks of stages with [stage.thermal] boils off
    // by the hull heat balance (vehicle/boil_off.hpp) in free flight above boil_off_min_altitude_m.
    bool boil_off = true;
```

In `FlightBody`'s public section after `rcs_force_body_n()`:

```cpp
    // Boil-off (refuelling r2): the heat balance is evaluated every boil_off_interval_s of free flight and after every
    // coast; the vented mass is the trapezoid of the last two evaluations' flows per tank. Below
    // boil_off_min_altitude_m in air (convection and entry heating are not modelled), clamped, caught, landed, resting
    // or docked nothing boils off (a docked tanker's boil-off belongs to the r4 transfer).
    static constexpr double boil_off_interval_s = 10.0;        // [E]
    static constexpr double boil_off_min_altitude_m = 100.0e3;  // [E]
    const BoilOffRate& boil_off_rate() const { return boil_off_rate_; }  // at the last evaluation
```

Private section:

```cpp
    void boil_off(const FlightEnvironment& env, time::Epoch epoch, double dt_s);
    void reset_boil_off() {
        boil_off_elapsed_s_ = 0.0;
        boil_off_last_.clear();
        boil_off_rate_ = {};
    }
    double boil_off_elapsed_s_ = 0.0;
    std::vector<TankBoilOff> boil_off_last_;
    BoilOffRate boil_off_rate_;
```

`flight_body.cpp`: add `#include "starsim/environment/thermal_radiation.hpp"`. New function (next to
`snap_ideal_attitude`):

```cpp
void FlightBody::boil_off(const FlightEnvironment& env, time::Epoch epoch, double dt_s) {
    if (!env.boil_off || env.ephemeris == nullptr) return;
    boil_off_elapsed_s_ += dt_s;
    // The margin keeps a sum of ticks (100 x 0.1 s = 9.99...98 s) on its interval.
    if (boil_off_elapsed_s_ < boil_off_interval_s - 1e-9) return;
    const auto body = reference_body_of(env, reference_body(env, epoch));
    if (body.has_atmosphere()) {
        const auto geodetic = frames::fixed_to_geodetic(body.frame(epoch).to_fixed(state_.position_gcrf_m), body.shape());
        if (geodetic.height_m < boil_off_min_altitude_m) {
            reset_boil_off();
            return;
        }
    }
    const Vec3 sun = env.ephemeris->state_gcrf(ephemeris::Body::Sun, epoch).position_m - state_.position_gcrf_m;
    const Vec3 moon = env.ephemeris->state_gcrf(ephemeris::Body::Moon, epoch).position_m - state_.position_gcrf_m;
    auto now = boil_off_rates(*vehicle_, environment::earth_orbit_radiation(state_.position_gcrf_m, sun, &moon),
                              state_.attitude);
    for (const auto& tank : now) {
        double flow = tank.mass_flow_kgps;
        for (const auto& before : boil_off_last_) {
            if (before.stage_index == tank.stage_index && before.tank_index == tank.tank_index) {
                flow = 0.5 * (flow + before.mass_flow_kgps);
            }
        }
        const auto& t = vehicle_->stages()[tank.stage_index].tanks[tank.tank_index];
        vehicle_->set_tank_mass(tank.stage_index, t.config.id, t.mass_kg - flow * boil_off_elapsed_s_);
    }
    boil_off_rate_ = total_boil_off(*vehicle_, now);
    boil_off_last_ = std::move(now);
    boil_off_elapsed_s_ = 0.0;
}
```

(`has_atmosphere()` is the `ReferenceBody` query `step` already uses; `frames::fixed_to_geodetic` is used the same way
at the top of `step`.)

Calls:
- `step`, free-flight path: after `vehicle_->consume_propellant(dt_s);` and before `resolve_ground_contact(...)`:
  `boil_off(env, epoch + dt_s, dt_s);`
- `coast`, after `update_air_data(end_body, end_frame);`: `boil_off(env, epoch + duration_s, duration_s);`
- `separate`: before `return upper_body;` add `reset_boil_off();` (the lower part's stages changed; the new upper body
  starts empty by construction).

`flight_session.hpp`, next to `set_solar_radiation_pressure`:

```cpp
    void set_boil_off(bool on) { env_.boil_off = on; }  // fidelity flag (refuelling r2)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[boil_off]" > tmp/r2_t5.txt 2>&1`, read the tail.
Expected: all pass.

Then the affected tags - Block 3 ship / tanker flights now boil off in orbit (incl. their `[slow]` cases):
`-TestFilter "[ship],[deorbit],[entry],[catch],[header],[session],[sim],[vehicle],[refuelling]"` into
`tmp/r2_t5_tags.txt`. Lunar (HLS, no `[stage.thermal]`), ISS and Falcon 9 / Dragon paths do not enter the new branch
(no thermal stage) and are not run. Expected: green. A ship campaign that changes (less main-tank propellant at the
deorbit burn after an orbital coast) is reported with its numbers: what changed, by how much, why (boil-off of a bare
hull). Do not tune `[stage.thermal]` values or tolerances to restore old numbers; if a ship mission now fails, stop and
ask the user (option: the session flag off for that campaign, documented).

Build with bindings once: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -TestFilter "[boil_off]"` and the
Godot smoke test
(`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd`).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/vehicle/flight_body.hpp core/src/vehicle/flight_body.cpp core/include/starsim/sim/flight_session.hpp core/tests/test_boil_off.cpp
git commit -m "feat(vehicle): propellant boil-off in flight with the boil_off fidelity flag"
git push origin main
```

---

### Task 6: Documentation and final review

**Files:**
- Modify: `docs/subsystems.md` ("Orbital refuelling" section: new subsection "Boil-off (r2)")
- Modify: `config/README.md` (`[stage.thermal]`, `[stage.thermal.shield]`, propellant boiling keys)
- Modify: `CLAUDE.md` (module summary: `environment` thermal radiation, `vehicle` boil-off)
- Modify: `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md` (status line: r2 implemented, plan path)

- [ ] **Step 1: Write the docs**

`docs/subsystems.md`, "Boil-off (r2)": the model (strips, balance, latent heat, clamp), the sources (Sun, albedo,
Earth IR, view factor, eclipse), all simplifications from the plan decisions, the cadence and where it does not run,
the fidelity flag, the Centaur calibration / validation table (AC-206, AC-167, AV-003, TC; model vs flight, the 25 %
tolerance and why), the derived resistances (foam 1.586, MLI-3 8.22, depot 63.7) with
`tools/validation/centaur_boil_off_calibration.py`, the depot and tanker orbit-average numbers recorded in Task 4,
the changed ship-campaign numbers from Task 5 (if any), and the open points: Moon albedo / IR (r5 with the HLS),
docked boil-off (r4), the r6 calculator's use of `orbit_average_boil_off`.

`config/README.md`: the `[stage.thermal]` keys (`absorptivity`, `emissivity` in (0, 1], `wall_resistance_m2k_w` > 0),
`[stage.thermal.shield]` (+ `half_width_deg`, around `[stage.aero] belly_azimuth_deg`), the propellant keys
(`*_boiling_point_k`, `*_latent_heat_j_kg`, both or neither).

`CLAUDE.md`: in the module list add "thermal radiation environment (Sun, albedo, Earth IR)" to `environment` and
"propellant boil-off by a hull heat balance (in flight and orbit-averaged)" to `vehicle`.

- [ ] **Step 2: Commit and push**

```bash
git add docs/subsystems.md config/README.md CLAUDE.md docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md
git commit -m "docs: refuelling r2 boil-off heat balance"
git push origin main
```

- [ ] **Step 3: Final review**

Dispatch one reviewer (most capable model) on the r2 commit range against the spec section "r2 design", this plan's
decisions and the Review Focus. Fix Critical / Important findings in one pass (tests first), re-run `[boil_off]` and
the tags of Task 5 that the fixes touch, commit `fix(vehicle): r2 review fixes - ...`, push. Record deferred minors
in the ledger.
