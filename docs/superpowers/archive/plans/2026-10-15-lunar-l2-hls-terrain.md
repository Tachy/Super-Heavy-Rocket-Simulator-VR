# Lunar l2: HLS Vehicle and Lunar Terrain Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** An HLS (`starship_hls`, 120 t dry [E]) with landing thrusters high on the hull and fixed legs, standing on real
LOLA terrain at a lunar landing site: drop tests on 0/5/10/15 deg slopes and on the tile, heights and normals checked
against the raw PDS grids.

**Architecture:** Engine groups gain a station, a cant and a `landing_only` role (data only in l2; l4 flies them). The
data pipeline crops the LOLA polar DEM `LDEM_80S_20M` (20 m/px) around each site in `config/lunar_sites.toml` by an HTTP
range request and downloads the global `LDEM_16` (16 px/deg) once; `environment::LunarTerrain` serves height and normal
anywhere on the Moon (tile inside, global grid outside, blended over 1 km) through `ReferenceBody`. The leg landing
simulation works on any reference body (site frame from the body's fixed frame) and presses the feet along the terrain
normal; flat ground keeps today's numbers bit for bit.

**Tech Stack:** C++20 core (Catch2), Python 3.11 data pipeline (stdlib only: `urllib`, `struct`, `tomllib`), toml++.

**Spec:** `docs/superpowers/specs/2026-10-14-lunar-missions-design.md` (section "l2 design: HLS vehicle and lunar terrain";
l1 is done on branch `feat/lunar-missions`).

## Global Constraints

- SI internally, radians; frame and unit in names (`position_gcrf_m`, `latitude_rad`).
- Vehicle states stay Earth-centred GCRF; everything body-relative goes through `environment::ReferenceBody` (l1).
- Moon: sphere R = 1737.4 km (`constants::moon_mean_radius_m`), body-fixed frame MOON_ME (`spice::SpiceOrientation::moon_mean_earth()`).
- HLS dry mass 120 t [E] (user 2026-10-14); tanks 1,600 t [S]; 3 sea-level + 3 vacuum Raptor 3; no flaps, no heat shield, no catch pins; ship RCS layout.
- Landing engines: own engine group, high on the hull, lift ~1.5 lunar weights of the landing mass and can hover, never used in Earth's atmosphere.
- Legs fixed (always deployed), spring-damper + crush core as Falcon 9's (`contact/landing_legs`).
- Only free data; every source in `tools/data_pipeline/sources.toml` with its licence; `CREDITS.md` regenerated.
- `core` modules depend downwards only; external libraries (toml++) stay inside `config`.
- No magic numbers: provenance tags [S]/[E]/[D] on every config value; constants with source.
- Earth path unchanged: Falcon 9 leg and campaign outputs identical before/after (compare printed numbers).
- Drawing and UI wait for the user's UI specification (no game changes in l2).
- Commits: Conventional Commits, English, trailer from the session's system reminder.

## Review Focus

1. **An HLS touching the Moon away from any landing site** (no lunar site set, or 50 km off it): it must rest or be judged
   an impact by `FlightBody`'s own ground contact, never fall through the ground because the legs "own" the contact.
   Test in Task 6.
2. **Burn planning picking the landing thrusters** for an orbit burn or `ignite()` lighting them with the Raptors: they
   must never be chosen outside l4's landing guidance; the ship's burn configurations stay what they were. Test in Task 1.
3. **Earth leg landings after the generalisation** (Falcon 9 RTLS / drone ship): numbers must stay bit-identical (flat
   ground: normal (0,0,1)). Before/after comparison of the printed `[legs]` and `[falcon9]` lines in Task 6.
4. **Missing terrain data** (`fetch.py` not run): loading a lunar site must fail with a German message naming
   `python tools/data_pipeline/fetch.py`, not crash or silently use the sphere. Test in Task 4.
5. **Crossing the tile border** (an approach from 10 km out, or a foot near the edge): the ground must not jump (the site
   lies ~5 km above the reference sphere); the global grid carries the terrain outside, blended over 1 km. Test in Task 4.

## File Structure

| File | Responsibility |
|---|---|
| `core/include/starsim/config/vehicle_config.hpp`, `core/src/config/vehicle_config.cpp` | `EngineGroupConfig::station_m`, `cant_rad`, `landing_only`; `LandingLegConfig::fixed` |
| `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp` | engine placement at the station, canted thrust axis, `Engine::auxiliary()`, fixed legs deployed from the start |
| `core/src/sim/flight_session.cpp` | `auxiliary()` instead of `abort_only` where engines are chosen; leg sites on any body; lunar terrain |
| `tools/vehicle_report.py` | landing-only groups reported apart from the delta-v budget; axial thrust of canted groups |
| `config/engines/hls_landing_thruster.toml` | landing thruster [E] |
| `config/vehicles/starship_hls.toml` | the HLS |
| `config/lunar_sites.toml` | landing sites (id, name, latitude, longitude [S], tile half size) |
| `tools/data_pipeline/lunar_tile.py` | PDS3 label parsing, polar stereographic projection, range download + crop, global grid, orientation check |
| `tools/data_pipeline/test_lunar_tile.py` | unit tests of the projection and the crop (no network) |
| `tools/data_pipeline/fetch.py`, `sources.toml`, `CREDITS.md`, `data/README.md` | the two LOLA sources |
| `core/include/starsim/environment/lunar_terrain.hpp`, `core/src/environment/lunar_terrain.cpp` | `HeightGrid`, `PolarStereographicTile`, `CylindricalGrid`, `LunarTerrain` (height + normal in body-fixed coordinates) |
| `core/include/starsim/environment/reference_body.hpp`, `.cpp` | `ReferenceBody::set_terrain`, `surface_height_m`, `surface_normal_fixed` |
| `core/include/starsim/config/lunar_site_config.hpp`, `core/src/config/lunar_site_config.cpp` | `load_lunar_site`, `load_terrain_grid_file` (TOML header + int16 heights) |
| `core/include/starsim/sim/lunar_site.hpp`, `core/src/sim/lunar_site.cpp` | `sim::LunarSite`, `sim::load_lunar_terrain`, `sim::load_lunar_site` |
| `core/include/starsim/contact/landing_surface.hpp`, `.cpp` | `Support::normal`, `SurfaceField`, `PlaneField`, `TerrainField` |
| `core/src/contact/landing_legs.cpp` | foot force along the surface normal |
| `core/include/starsim/sim/leg_landing_sim.hpp`, `.cpp` | `LegSite` (body, latitude, longitude, height, tracking radius); env-based ticks |
| `core/include/starsim/vehicle/flight_body.hpp`, `.cpp` | ground over terrain at the Moon; `hold_landed(frame, body)`; `set_legs_on_site` |
| `core/tests/test_engine_placement.cpp` (new), `test_hls_vehicle.cpp` (new), `test_lunar_terrain.cpp` (new), `test_terrain_contact.cpp` (new), `test_hls_landing.cpp` (new) | tests |

---

### Task 1: Engine groups with station, cant and a landing-only role

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp:76-85`, `core/src/config/vehicle_config.cpp:181-198`
- Modify: `core/include/starsim/vehicle/vehicle.hpp:26-45`, `core/src/vehicle/vehicle.cpp:34-36,64-79`
- Modify: `core/src/sim/flight_session.cpp:67,430,456,637,1226`
- Modify: `tools/vehicle_report.py:99-100`
- Create: `core/tests/test_engine_placement.cpp` (add to `core/tests/CMakeLists.txt`)

**Interfaces:**
- Produces: `config::EngineGroupConfig::{double station_m = 0.0; double cant_rad = 0.0; bool landing_only = false;}`;
  `vehicle::Engine::{math::Vec3 axis_body{1,0,0}; bool landing_only = false; bool auxiliary() const;}`;
  `Engine::thrust_direction()` returns `axis_body` for a canted engine (gimbal not allowed with a cant).

Semantics: `station_m` is the gimbal / thrust point above the bottom of the stage (default 0: today's engines). `cant_deg`
tilts the thrust towards the axis (the exhaust away from the hull) in the plane of the engine's ring angle: axis
`(cos c, -sin c cos a, -sin c sin a)` for ring angle `a`. A symmetric ring then has a pure axial net force `n F cos c`
and no moment. `landing_only = true`: never in burn configurations, never lit by `ignite()`, not counted as a powered
stage; only l4's landing guidance commands it by group.

- [ ] **Step 1: Write the failing tests**

```cpp
// core/tests/test_engine_placement.cpp
// Engine groups placed up the hull and canted towards the axis (lunar l2: the HLS landing thrusters).

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

// The ship of the Block 3 stack with one extra canted ring of four Raptors at 40 m (a test vehicle, not the HLS).
config::VehicleConfig canted_ship(bool landing_only) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    config::EngineGroupConfig g;
    g.id = "canted";
    g.engine_id = "raptor3";
    g.count = 4;
    g.ring_radius_m = 4.6;
    g.station_m = 40.0;
    g.cant_rad = 15.0 * deg;
    g.relight = true;
    g.landing_only = landing_only;
    cfg.stages.front().engine_groups.push_back(g);
    return cfg;
}

}  // namespace

TEST_CASE("A canted ring up the hull thrusts along the axis without a moment", "[vehicle][engines]") {
    const vehicle::Vehicle v(canted_ship(false), STARSIM_CONFIG_DIR);
    Vec3 force, moment;
    const double cg = v.mass_properties().cg_station_m;
    int n = 0;
    for (const auto& e : v.stages().front().engines) {
        if (e.group_id != "ship/canted") continue;
        ++n;
        CHECK(e.gimbal_point_m.x == Catch::Approx(40.0));
        CHECK(std::hypot(e.gimbal_point_m.y, e.gimbal_point_m.z) == Catch::Approx(4.6));
        const Vec3 d = e.thrust_direction();
        CHECK(d.x == Catch::Approx(std::cos(15.0 * deg)));
        // towards the axis: the radial component points inwards
        CHECK(d.y * e.gimbal_point_m.y + d.z * e.gimbal_point_m.z < 0.0);
        force += d;
        moment += cross(e.gimbal_point_m - Vec3{cg, 0.0, 0.0}, d);
    }
    CHECK(n == 4);
    CHECK(force.x == Catch::Approx(4.0 * std::cos(15.0 * deg)));
    CHECK(std::hypot(force.y, force.z) < 1e-12);
    CHECK(norm(moment) < 1e-9);
}

TEST_CASE("Engines without a station or cant are placed as before", "[vehicle][engines]") {
    const vehicle::Vehicle v(config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3"), STARSIM_CONFIG_DIR);
    for (const auto& e : v.stages().back().engines) {
        CHECK(e.gimbal_point_m.x == Catch::Approx(v.stages().back().base_station_m));
        CHECK(e.thrust_direction().x == 1.0);
        CHECK(!e.auxiliary());
    }
}

TEST_CASE("A canted engine group cannot gimbal", "[vehicle][engines][config]") {
    auto cfg = canted_ship(false);
    cfg.stages.front().engine_groups.back().gimbal = true;
    CHECK_THROWS(vehicle::Vehicle(cfg, STARSIM_CONFIG_DIR));
}
```

Also append to `core/tests/test_flight_session.cpp` (it already builds sessions with a placed ship; reuse its fixture
`Env`/`env()` and `ship_in_orbit` helpers if present, else build the state with `test::circular_orbit_state` as the
file's other burn tests do):

```cpp
TEST_CASE("Landing-only engines are never chosen for burns or lit by ignite", "[sim][engines]") {
    // The ship with a landing-only canted ring: its burn configurations must equal the plain ship's.
    auto with_ring = test::ship_config();                 // the ship stage alone (helper of this file)
    config::EngineGroupConfig g;
    g.id = "landing";
    g.engine_id = "raptor3";
    g.count = 4;
    g.ring_radius_m = 4.6;
    g.station_m = 40.0;
    g.cant_rad = 15.0 * constants::deg_to_rad;
    g.relight = true;
    g.landing_only = true;
    with_ring.stages.front().engine_groups.push_back(g);
    auto plain = make_orbit_session(test::ship_config());  // helper of this file: session + ship in a 200 km orbit
    auto ringed = make_orbit_session(with_ring);
    const auto a = plain->burn_configurations();
    const auto b = ringed->burn_configurations();
    REQUIRE(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i].groups == b[i].groups);
        CHECK(a[i].thrust_n == b[i].thrust_n);
    }
    ringed->ignite();
    ringed->advance_to(ringed->epoch() + 5.0);
    for (const auto& e : ringed->body(0).vehicle().stages().front().engines) {
        if (e.landing_only) CHECK(!e.producing_thrust());
    }
}
```

(If `burn_configurations()` is private, make it public with a comment "inspection: tests and the planner UI". If the
file has no `make_orbit_session`/`test::ship_config` helpers, add them at the top of the anonymous namespace: a
`FlightSession` built as in the file's other tests and `place_body(cfg, circular state 200 km / 26.5 deg, t0)`.)

- [ ] **Step 2: Run tests to verify they fail**

Run: core build (see the ledger), expect compile errors `no member named 'station_m'`.

- [ ] **Step 3: Implement**

`vehicle_config.hpp`, `EngineGroupConfig`:
```cpp
    bool abort_only = false;         // launch escape (SuperDraco): data only, never lit by the session
    double station_m = 0.0;          // thrust point above the bottom of the stage (0: at the base)
    double cant_rad = 0.0;           // thrust tilted towards the axis (exhaust away from the hull); no gimbal then
    bool landing_only = false;       // landing thrusters (HLS): never a burn or ascent engine, flown by landing guidance
```
`vehicle_config.cpp`, after `abort_only`:
```cpp
            s.engine_groups.back().station_m = doc.number_or(g, "station_m", 0.0);
            s.engine_groups.back().cant_rad = doc.number_or(g, "cant_deg", 0.0) * constants::deg_to_rad;
            s.engine_groups.back().landing_only = g["landing_only"].value_or(false);
            if (s.engine_groups.back().cant_rad != 0.0 && s.engine_groups.back().gimbal) {
                doc.fail("engine_group: a canted group cannot gimbal");
            }
```
`vehicle.hpp`, `Engine`:
```cpp
    math::Vec3 axis_body{1.0, 0.0, 0.0};  // thrust axis without gimbal (canted engines: towards the hull axis)
    bool landing_only = false;           // landing thrusters: flown by landing guidance only
    // Launch escape or landing thrusters: never chosen for burns, never lit by ignite().
    bool auxiliary() const { return abort_only || landing_only; }
```
`vehicle.cpp`:
```cpp
Vec3 Engine::thrust_direction() const {
    if (axis_body.x != 1.0) return axis_body;  // canted: fixed (no gimbal)
    return normalized(Vec3{1.0, std::tan(gimbal_yaw_rad), std::tan(gimbal_pitch_rad)});
}
```
and in the constructor loop:
```cpp
                if (group.cant_rad != 0.0 && group.gimbal) {
                    throw std::runtime_error("engine group " + group.id + ": a canted group cannot gimbal");
                }
                engine.gimbal_point_m = {station + group.station_m, group.ring_radius_m * std::cos(angle),
                                         group.ring_radius_m * std::sin(angle)};
                engine.axis_body = {std::cos(group.cant_rad), -std::sin(group.cant_rad) * std::cos(angle),
                                    -std::sin(group.cant_rad) * std::sin(angle)};
                ...
                engine.landing_only = group.landing_only;
```
(`cos(0) == 1.0` exactly, so uncanted engines keep the old formula bit for bit.)

`flight_session.cpp`: replace `e.abort_only` by `e.auxiliary()` at lines 67, 430, 456, 637, 1226 and adjust the comments
("launch escape or landing thrusters").

`tools/vehicle_report.py`: next to `abort_only`, exclude `landing_only` groups from the budget and print them in a line
of their own: `landing thrusters: <n> x <thrust> kN (axial <n F cos c>)`.

- [ ] **Step 4: Run tests**

Run: `build/core/tests/starsim_core_tests.exe "[engines]"` → PASS. Then `"[sim]"` → PASS.

- [ ] **Step 5: Commit**

`feat(vehicle): engine groups with a station, a cant and a landing-only role`

---

### Task 2: Fixed legs and the HLS vehicle

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp:134-144`, `core/src/config/vehicle_config.cpp:319-335`, `core/src/vehicle/vehicle.cpp` (constructor: fixed legs deployed)
- Create: `config/engines/hls_landing_thruster.toml`, `config/vehicles/starship_hls.toml`
- Create: `core/tests/test_hls_vehicle.cpp`

**Interfaces:**
- Consumes: Task 1's `station_m`, `cant_deg`, `landing_only`.
- Produces: `config::LandingLegConfig::fixed` (`deploy_time_s` optional then); vehicle id `starship_hls`, single stage
  `hls`, engine groups `sea_level_1/2/3`, `vacuum`, `landing`.

Landing mass [E] used for sizing (written into the config comment and recomputed in the test): dry 120 t + the ascent
propellant to a 100 km LLO, 1.9 km/s [S: spec l4] x 1.1 margin [E] at Raptor 3 vacuum Isp 350 s [S]:
`120 t * (exp(2090 / (350 * 9.80665)) - 1) = 101 t` -> **221 t**. Lunar weight `221 t * 1.624 m/s^2 = 359 kN`.
Landing thrusters [E]: 12 x 46 kN vacuum (552 kN, axial `552 cos 15 deg = 533 kN` = 1.49 lunar weights), minimum throttle
40 % (213 kN axial: the HLS hovers from 131 t to 328 t), Isp 340 s, station 40 m, cant 15 deg, 1 s start [E].

Legs [E]: six legs (a four-legged base tips at ~14 deg between two legs), footprint radius 8.0 m, feet 1.5 m below the
skirt, crush force 5e5 N, stroke 0.6 m (1.8 MJ: ~4 m/s at 221 t), stiffness 4e7 N/m (2 mm static), damping 8e5 N s/m
(~0.27 of critical per leg), friction 0.6. Static tip-over angle across an edge `atan(8 cos 30 / h_cg)` with
`h_cg ~ 21 m` -> ~18 deg (computed in Task 7).

- [ ] **Step 1: Write the failing test**

```cpp
// core/tests/test_hls_vehicle.cpp
// The HLS (lunar l2): masses, landing thrusters sized to the lunar weight, fixed legs.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
constexpr double lunar_g_mps2 = 4.9028e12 / (1737.4e3 * 1737.4e3);  // GM [S: DE440] / R^2: 1.624 m/s^2
double landing_mass() {  // dry + ascent to LLO (1.9 km/s x 1.1 [E]) at Raptor vacuum Isp 350 s
    return 120.0e3 * std::exp(1.1 * 1900.0 / (350.0 * constants::standard_gravity_mps2));
}
}  // namespace

TEST_CASE("The HLS has the Block 3 tanks, 120 t dry and no entry hardware", "[vehicle][hls]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    REQUIRE(cfg.stages.size() == 1);
    const auto& s = cfg.stages.front();
    CHECK(s.dry_mass_kg == 120000.0);
    double propellant = 0.0;
    for (const auto& t : s.tanks) propellant += t.mass_kg;
    CHECK(propellant == Catch::Approx(1.6e6));
    CHECK(s.flaps.empty());
    CHECK(s.heat_shield.nose_radius_m == 0.0);
    CHECK(s.catch_pins.azimuths_rad.empty());
    CHECK(s.landing_legs.fixed);
}

TEST_CASE("HLS landing thrusters lift ~1.5 lunar weights and can hover", "[vehicle][hls]") {
    const vehicle::Vehicle v(config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls"), STARSIM_CONFIG_DIR);
    double max_axial = 0.0, min_axial = 0.0;
    int n = 0;
    for (const auto& e : v.stages().front().engines) {
        if (!e.landing_only) continue;
        ++n;
        const auto& c = e.performance->config();
        max_axial += e.performance->thrust_n(1.0, 0.0) * e.thrust_direction().x;
        min_axial += e.performance->thrust_n(c.throttle_min, 0.0) * e.thrust_direction().x;
        CHECK(e.gimbal_point_m.x > 30.0);  // high on the hull
    }
    CHECK(n == 12);
    const double weight = landing_mass() * lunar_g_mps2;
    std::printf("[hls] landing mass %.1f t, lunar weight %.0f kN, landing thrust %.0f-%.0f kN axial\n",
                landing_mass() / 1e3, weight / 1e3, min_axial / 1e3, max_axial / 1e3);
    CHECK(max_axial / weight == Catch::Approx(1.5).margin(0.1));
    CHECK(min_axial < weight);  // can hover
    // A single Raptor at minimum throttle exceeds the landing weight: the reason for the landing thrusters
    for (const auto& e : v.stages().front().engines) {
        if (e.group_id == "hls/sea_level_1") CHECK(e.performance->thrust_n(e.performance->config().throttle_min, 0.0) > weight);
    }
}

TEST_CASE("Fixed legs are deployed from the start and stay deployed", "[vehicle][hls][legs]") {
    vehicle::Vehicle v(config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls"), STARSIM_CONFIG_DIR);
    CHECK(v.has_landing_legs());
    CHECK(v.leg_deployment() == 1.0);
    v.update_landing_legs(1.0);
    CHECK(v.leg_deployment() == 1.0);
}
```

- [ ] **Step 2: Run** `"[hls]"` → FAIL (`starship_hls` missing).

- [ ] **Step 3: Implement**

`LandingLegConfig`: `bool fixed = false;  // always deployed (HLS): no deployment, deploy_time_s unused`.
Parsing: `s.landing_legs.fixed = (*legs)["fixed"].value_or(false);` and
`s.landing_legs.deploy_time_s = s.landing_legs.fixed ? doc.number_or(*legs, "deploy_time_s", 0.0) : doc.positive(*legs, "deploy_time_s");`.
`Vehicle` constructor, after the stages are built: `if (has_landing_legs() && stages_.front().config.landing_legs.fixed) set_leg_deployment(1.0);`
(Check `update_landing_legs` does not divide by `deploy_time_s` once deployed; guard with `if (deploy_time_s <= 0) return`.)
`has_landing_legs()` reads the lowest stage: the HLS is single-stage.

`config/engines/hls_landing_thruster.toml`:
```toml
# HLS landing thruster - methalox, high on the hull (SpaceX HLS concept: landing engines against plume-surface
# interaction). SpaceX publishes no figures: every value is an estimate [E] sized by the lunar l2 plan (12 thrusters lift
# ~1.5 lunar weights of the ~221 t landing mass and can hover).
[sources]
lunar_spec = "docs/superpowers/specs/2026-10-14-lunar-missions-design.md (l2)"

[engine]
id = "hls_landing_thruster"
name = "HLS landing thruster"
cycle = "pressure-fed"                # [E]
propellant = "methalox"
mixture_ratio = 3.6                   # [E] as Raptor (shared tanks)
chamber_pressure_pa = 20e5            # [E]

[performance]
reference_pressure_pa = 0.0           # vacuum rating
thrust_ref_n = 46.0e3                 # [E] 12 x 46 kN = 552 kN
isp_ref_s = 340.0                     # [E]
nozzle_exit_diameter_m = 0.45         # [E]
max_ambient_pressure_pa = 0.0         # vacuum only: never used in Earth's atmosphere

[throttle]
min = 0.40                            # [E] hover from ~131 t
max = 1.00
rate_per_s = 2.0                      # [E]

[gimbal]
max_angle_deg = 0.0                   # fixed, canted
rate_deg_s = 0.0

[mass]
dry_kg = 0.0                          # in the HLS dry mass
vehicle_side_hardware_kg = 0.0

[timing]
startup_s = 1.0                       # [E]
shutdown_s = 0.3                      # [E]
```
(Check `propulsion::EnginePerformance` accepts `reference_pressure_pa = 0` and `max_ambient_pressure_pa = 0`; look at
`raptor_vacuum3.toml` and use its convention if it differs.)

`config/vehicles/starship_hls.toml`: copy the `ship` stage of `starship_block3.toml` (tanks, the four Raptor groups,
`[stage.aero]` without `flap_table`, `entry_alpha_table` and `flaps`, the RCS pods) with:
```toml
# Starship HLS (Human Landing System) - lunar l2. The Block 3 ship's hull, tanks and engines without heat shield, flaps
# and catch pins, with landing thrusters high on the hull and fixed legs.
[sources]
lunar_spec = "docs/superpowers/specs/2026-10-14-lunar-missions-design.md (HLS dry mass, propulsion concept)"
oig_2026 = "NASA OIG IG-26-004, March 2026 (LEO depot, >10 tankers, HLS fills up in LEO)"
wiki_starship = "https://en.wikipedia.org/wiki/SpaceX_Starship (retrieved 2026-10-12)"

[vehicle]
id = "starship_hls"
name = "Starship HLS"
display_name = "Lunar Lander Mk3"
propellant = "methalox"
stages_bottom_to_top = ["hls"]

[[stage]]
id = "hls"
name = "Starship HLS"
length_m = 52.1                          # [D] as the Block 3 ship
diameter_m = 9.0
dry_mass_kg = 120000.0                   # [E] user 2026-10-14: SpaceX publishes none; ~100-120 t Musk's older target,
                                         # LEO departure closes <= ~140 t, bottom-up estimate ~204 t (spec l2)
dry_cg_station_m = 30.0                  # [E] crew cabin, airlock and landing thrusters high on the hull
payload_mass_kg = 0.0
... (tanks lox_main / ch4_main exactly as the ship)
... (engine groups sea_level_1/2/3 and vacuum exactly as the ship)

[[stage.engine_group]]
id = "landing"
engine = "hls_landing_thruster"
count = 12                               # [E] evenly spaced
ring_radius_m = 4.6                      # [E] on the hull (radius 4.5 m)
station_m = 40.0                         # [E] high on the hull (SpaceX HLS renderings)
cant_deg = 15.0                          # [E] exhaust away from the hull
gimbal = false
relight = true
landing_only = true                      # l4 flies them: last part of the landing, liftoff

[stage.landing_legs]                     # all [E]; fixed (always deployed)
fixed = true
azimuths_deg = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]   # six: four would tip at ~14 deg between two legs
footprint_radius_m = 8.0
foot_station_m = -1.5                    # feet below the skirt
stroke_m = 0.6
crush_force_n = 5.0e5                    # per leg; 6 legs x 0.6 m: 1.8 MJ (~4 m/s at 221 t)
stiffness_n_m = 4.0e7
damping_n_s_m = 8.0e5                    # ~0.27 of critical per leg at 221 t
friction = 0.6                           # [E] foot pad on regolith
```
Note in the landing-mass comment: `# landing mass ~221 t [E]: dry + ascent to LLO (1.9 km/s x 1.1, Isp 350 s)`.

Run `python tools/vehicle_report.py starship_hls`: expect ~9.9 km/s vacuum delta-v from full tanks (spec: LEO departure
needs ~9.3 km/s via the NRHO); record the line in the ledger.

- [ ] **Step 4: Run** `"[hls]"` and `"[legs]"` → PASS.

- [ ] **Step 5: Commit** `feat(config): Starship HLS with landing thrusters and fixed legs`

---

### Task 3: LOLA data pipeline (site tiles + global grid)

**Files:**
- Create: `config/lunar_sites.toml`
- Create: `tools/data_pipeline/lunar_tile.py`, `tools/data_pipeline/test_lunar_tile.py`
- Modify: `tools/data_pipeline/fetch.py` (dispatch `kind = "lunar_tiles"`), `tools/data_pipeline/sources.toml`, `data/README.md`
- Regenerate: `CREDITS.md` (`python tools/data_pipeline/credits.py`)

**Interfaces:**
- Produces files (git-ignored):
  - `data/terrain/<site_id>.toml` + `data/terrain/<site_id>.i16` (tile)
  - `data/terrain/ldem_16.toml` + `data/terrain/ldem_16.i16` (global)
- Header format (both), read by Task 4:
```toml
[grid]
product = "LDEM_80S_20M"          # or "LDEM_16"
projection = "polar_stereographic_south"   # or "simple_cylindrical"
sphere_radius_m = 1737400.0
dn_scale_m = 0.5                  # height above the sphere = DN x scale
heights = "mons_mouton_im2.i16"   # int16 little endian, row-major, `lines` x `samples`
lines = 1001
samples = 1001
# polar stereographic: 0-based line / sample of the pole in the full product, metres per pixel, first line / sample of
# the crop in the full product
pole_line = 15199.5
pole_sample = 15199.5
scale_m = 20.0
first_line = 0
first_sample = 0
# simple cylindrical (LDEM_16): pixels per degree, 0-based line / sample of (lat 0, lon 0 east)
# pixels_per_degree = 16.0
# zero_line = 1439.5          # = LINE_PROJECTION_OFFSET (latitude = (zero_line - line) / ppd)
# zero_sample = -0.5          # longitude = (sample - zero_sample) / ppd, east, 0..360

[[check]]                          # tile only: bilinear heights by Python from the raw DN (Task 4 compares)
latitude_deg = -84.79
longitude_deg = 29.19
height_m = 5123.25
```

Projections (LOLA `dsmap_polar.cat` [S], spherical R = 1737.4 km; 0-based indices):
- South polar stereographic: `rho = 2 R tan(pi/4 + lat/2)`, `X = rho sin(lon)`, `Y = -rho cos(lon)`,
  `sample = pole_sample + X / scale`, `line = pole_line + Y / scale` (longitude 0 points up from the south pole, east
  clockwise as seen from below). Inverse: `rho = hypot(X, Y)`, `lon = atan2(X, -Y)`, `lat = -pi/2 + 2 atan(rho / 2R)`.
- LDEM_16 simple cylindrical (`LINE_PROJECTION_OFFSET 1439.5`, `SAMPLE_PROJECTION_OFFSET 2879.5`, `CENTER_LONGITUDE 180`,
  16 px/deg [S label]): `line = 1439.5 - 16 lat_deg`, `sample = 16 lon_deg - 0.5` with lon in [0, 360)
  (sample 2879.5 at lon 180 deg).

- [ ] **Step 1: Sites config**

```toml
# config/lunar_sites.toml - lunar landing sites (MOON_ME latitude / longitude, the frame of the LOLA maps).
# The data pipeline crops a LOLA tile around each site (tools/data_pipeline/lunar_tile.py).
[sources]
artemis_2024 = "NASA, Artemis III candidate landing regions (2024-10-28): nine regions incl. Mons Mouton Plateau and Mons Mouton"
im2 = "https://en.wikipedia.org/wiki/Mons_Mouton (retrieved 2026-10-15): IM-2 Athena landed 2025-03-06 at 84.7906 S, 29.1957 E"

[[site]]
id = "mons_mouton_im2"
name = "Mons Mouton (IM-2)"
latitude_deg = -84.7906               # [S:im2] in the Mons Mouton region (Artemis III candidates [S:artemis_2024])
longitude_deg = 29.1957               # [S:im2] east
tile_half_size_m = 10000.0            # [E] 20 x 20 km tile
product = "LDEM_80S_20M"
```

- [ ] **Step 2: Write the failing Python tests** (`tools/data_pipeline/test_lunar_tile.py`, `unittest`, no network)

```python
"""Unit tests of the LOLA tile pipeline (projection, label parsing, crop). Run: python tools/data_pipeline/test_lunar_tile.py"""
import math
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import lunar_tile as lt  # noqa: E402

R = 1737400.0


class Projection(unittest.TestCase):
    def test_pole_is_at_the_pole_pixel(self):
        line, sample = lt.polar_south_line_sample(-math.pi / 2, 0.0, 15199.5, 15199.5, 20.0)
        self.assertAlmostEqual(line, 15199.5)
        self.assertAlmostEqual(sample, 15199.5)

    def test_longitude_zero_points_up_and_east_is_to_the_right(self):
        lat = math.radians(-85.0)
        l0, s0 = lt.polar_south_line_sample(lat, 0.0, 15199.5, 15199.5, 20.0)
        l90, s90 = lt.polar_south_line_sample(lat, math.radians(90.0), 15199.5, 15199.5, 20.0)
        self.assertLess(l0, 15199.5)          # up
        self.assertAlmostEqual(s0, 15199.5)
        self.assertGreater(s90, 15199.5)      # right
        self.assertAlmostEqual(l90, 15199.5)

    def test_round_trip(self):
        for lat_deg, lon_deg in [(-84.79, 29.19), (-89.0, 200.0), (-80.5, 359.0)]:
            line, sample = lt.polar_south_line_sample(math.radians(lat_deg), math.radians(lon_deg), 15199.5, 15199.5, 20.0)
            lat, lon = lt.polar_south_lat_lon(line, sample, 15199.5, 15199.5, 20.0)
            self.assertAlmostEqual(math.degrees(lat), lat_deg, places=9)
            self.assertAlmostEqual(math.degrees(lon) % 360.0, lon_deg % 360.0, places=9)

    def test_pixel_distance_from_the_pole_matches_the_label_note(self):
        # LOLA GDR help: colatitude of a pixel r metres from the pole = 2 atan(0.5 r / R)
        line, _ = lt.polar_south_line_sample(math.radians(-85.0), 0.0, 15199.5, 15199.5, 20.0)
        r = (15199.5 - line) * 20.0
        self.assertAlmostEqual(math.degrees(2.0 * math.atan(0.5 * r / R)), 5.0, places=9)


class Label(unittest.TestCase):
    def test_parses_keywords_with_units(self):
        text = 'LINES = 30400\nLINE_SAMPLES = 30400\nSAMPLE_BITS = 16\nSCALING_FACTOR = 0.5\n' \
               'MAP_SCALE = 20 <METERS/PIXEL>\nLINE_PROJECTION_OFFSET = 15199.5 <PIXEL>\n' \
               'SAMPLE_TYPE = LSB_INTEGER\nMAP_PROJECTION_TYPE = "POLAR STEREOGRAPHIC"\n'
        label = lt.parse_label(text)
        self.assertEqual(label["LINES"], 30400.0)
        self.assertEqual(label["MAP_SCALE"], 20.0)
        self.assertEqual(label["LINE_PROJECTION_OFFSET"], 15199.5)
        self.assertEqual(label["SAMPLE_TYPE"], "LSB_INTEGER")
        self.assertEqual(label["MAP_PROJECTION_TYPE"], "POLAR STEREOGRAPHIC")


class Crop(unittest.TestCase):
    def test_crop_rows_and_bilinear(self):
        samples, lines = 8, 6
        rows = b"".join(struct.pack("<8h", *[10 * l + s for s in range(samples)]) for l in range(lines))
        tile = lt.crop_rows(rows, samples, first_sample=2, count=3)       # 6 x 3
        self.assertEqual(tile[1], [12, 13, 14])
        self.assertAlmostEqual(lt.bilinear(tile, 0.5, 0.5), 0.25 * (2 + 3 + 12 + 13))


if __name__ == "__main__":
    unittest.main()
```
Run: `python tools/data_pipeline/test_lunar_tile.py` → FAIL (no module).

- [ ] **Step 3: Implement `lunar_tile.py`**

Functions (all pure except the two `fetch_*`):
```python
"""LOLA terrain for the lunar landing sites (lunar l2).

Site tiles: crop of the polar stereographic LDEM_80S_20M (20 m/px, 80-90 S) around each site of config/lunar_sites.toml,
fetched with one HTTP range request over the rows it needs (the full product is 1.8 GB). Global grid: LDEM_16
(16 px/deg, 33 MB) for the terrain outside the tiles. Output in data/terrain/: a TOML header + int16 heights (see
core/src/config/lunar_site_config.cpp). Projections: LOLA dsmap_polar.cat [S], spherical R = 1737.4 km.
"""
def parse_label(text: str) -> dict: ...                      # KEY = value [<unit>]; numbers -> float, "..." -> str
def polar_south_line_sample(lat, lon, pole_line, pole_sample, scale_m) -> tuple[float, float]: ...
def polar_south_lat_lon(line, sample, pole_line, pole_sample, scale_m) -> tuple[float, float]: ...
def crop_rows(rows: bytes, samples: int, first_sample: int, count: int) -> list[list[int]]: ...
def bilinear(tile: list[list[int]], line: float, sample: float) -> float: ...   # in DN
def fetch_site_tile(site: dict, source: dict, out_dir: Path) -> None: ...
def fetch_global(source: dict, out_dir: Path) -> None: ...
def check_orientation(site_id: str, out_dir: Path) -> float: ...   # rms vs LDEM_16; raises if the tile is mis-oriented
```
`fetch_site_tile`: download `<url>.lbl` (label url = img url with `.lbl`), parse `LINES`, `LINE_SAMPLES`, `SAMPLE_TYPE`
(must be `LSB_INTEGER`, 16 bits), `SCALING_FACTOR`, `OFFSET` (must be 1737400), `MAP_SCALE`, `LINE_PROJECTION_OFFSET`,
`SAMPLE_PROJECTION_OFFSET`, `A_AXIS_RADIUS` (km, must be 1737.4), `CENTER_LATITUDE` (must be -90); centre pixel from the
site; `half = round(tile_half_size_m / MAP_SCALE)`; `first_line = round(line) - half`, `lines = 2 half + 1` (same for
samples); `Range: bytes=first_line*row_bytes-(first_line+lines)*row_bytes-1` (row_bytes = 2 x LINE_SAMPLES); require
HTTP 206 (refuse a 200: it would be the whole 1.8 GB); crop; write `<id>.i16` (`struct.pack("<{n}h")`) and `<id>.toml`
with 9 `[[check]]` points (the site and the 8 points at +-0.4 half sizes on the grid diagonals and axes, bilinear
height `DN x 0.5`). Then `check_orientation`.
`fetch_global`: download `ldem_16.lbl` + `.img` to `data/terrain/ldem_16.i16` (the IMG is already int16 LE rows) and write
`ldem_16.toml` from the label (`zero_line = LINE_PROJECTION_OFFSET`, `zero_sample = SAMPLE_PROJECTION_OFFSET - 180 x 16`).
`check_orientation`: compare the tile with LDEM_16 at a 21 x 21 lattice over the tile (tile bilinear vs LDEM_16
bilinear); also for the mirrored alternatives (longitude sign flipped; line axis flipped). Require
`rms(correct) < 150 m` and `rms(correct) < 0.5 x min(rms(alternatives))`, else raise `RuntimeError` naming the numbers.
Print all three rms.

`fetch.py`: before the generic download, `if source.get("kind") == "lunar_tiles": lunar_tile.fetch_all(source, REPO_ROOT); continue`
(`fetch_all` reads `config/lunar_sites.toml`, fetches the global grid first, then every site whose tile is missing or
`--force`).

`sources.toml`:
```toml
[[source]]
id = "lola_ldem_16"
name = "LRO LOLA gridded elevation LDEM_16 (global, 16 px/deg) - terrain outside the site tiles"
url = "https://pds-geosciences.wustl.edu/lro/lro-l-lola-3-rdr-v1/lrolol_1xxx/data/lola_gdr/cylindrical/img/ldem_16.img"
target = "data/terrain/ldem_16.i16"
kind = "lunar_tiles"
license = "NASA PDS data: public domain (NASA data and images are not copyrighted)"
credit = "LRO LOLA team (NASA GSFC), Smith et al.; NASA Planetary Data System Geosciences Node"

[[source]]
id = "lola_ldem_80s_20m"
name = "LRO LOLA polar gridded elevation LDEM_80S_20M (20 m/px, 80-90 S) - cropped to the landing-site tiles"
url = "https://pds-geosciences.wustl.edu/lro/lro-l-lola-3-rdr-v1/lrolol_1xxx/data/lola_gdr/polar/img/ldem_80s_20m.img"
target = "data/terrain/mons_mouton_im2.i16"
kind = "lunar_tiles"
license = "NASA PDS data: public domain (NASA data and images are not copyrighted)"
credit = "LRO LOLA team (NASA GSFC), Smith et al.; NASA Planetary Data System Geosciences Node"
```
(`fetch_all` is called once per run even though two sources carry the kind: guard with a module-level flag.)

`data/README.md`: `- terrain/ - LOLA elevation: LDEM_16 global grid and 20 m tiles around config/lunar_sites.toml (lunar l2).`

- [ ] **Step 4: Run** the Python tests → PASS; then `curl -sI -r 0-99 <ldem_80s_20m url>` shows `206`; then
`python tools/data_pipeline/fetch.py lola_ldem_16 lola_ldem_80s_20m` → tile + global written, orientation rms printed
(record in the ledger). If the check fails, the projection orientation is wrong: fix the formula (not the tolerance)
and re-run the unit tests.

- [ ] **Step 5:** `python tools/data_pipeline/credits.py`; commit
`feat(data): LOLA terrain tiles for the lunar landing sites`

---

### Task 4: Lunar terrain model and reference-body surface

**Files:**
- Create: `core/include/starsim/environment/lunar_terrain.hpp`, `core/src/environment/lunar_terrain.cpp`
- Create: `core/include/starsim/config/lunar_site_config.hpp`, `core/src/config/lunar_site_config.cpp`
- Create: `core/include/starsim/sim/lunar_site.hpp`, `core/src/sim/lunar_site.cpp`
- Modify: `core/include/starsim/environment/reference_body.hpp`, `core/src/environment/reference_body.cpp`, `core/CMakeLists.txt`
- Create: `core/tests/test_lunar_terrain.cpp`

**Interfaces:**
```cpp
namespace starsim::environment {
// Heights above a reference sphere on a regular grid (int16 x scale, as the LOLA products store them).
struct HeightGrid {
    int lines = 0, samples = 0;
    double dn_scale_m = 0.0;
    std::vector<std::int16_t> dn;  // row-major
    double at(int line, int sample) const { return dn_scale_m * dn[static_cast<size_t>(line) * samples + sample]; }
    double bilinear(double line, double sample) const;  // inside [0, lines-1] x [0, samples-1]
};
// A crop of a south polar stereographic product (LOLA LDEM_80S_*).
struct PolarStereographicTile {
    HeightGrid grid;
    double sphere_radius_m = 0.0, scale_m = 0.0, pole_line = 0.0, pole_sample = 0.0;
    int first_line = 0, first_sample = 0;
    // Grid position (line, sample in the crop) of a latitude / longitude; and the distance (pixels) to the crop's edge.
    std::pair<double, double> position(double latitude_rad, double longitude_rad) const;
    double edge_distance_px(double line, double sample) const;
};
// A global simple-cylindrical grid (LOLA LDEM_16): longitude east 0..360.
struct CylindricalGrid {
    HeightGrid grid;
    double pixels_per_degree = 0.0, zero_line = 0.0, zero_sample = 0.0;
    double height_m(double latitude_rad, double longitude_rad) const;  // bilinear, longitude wraps, clamped at the poles
};
// Terrain of the Moon: the global grid everywhere, site tiles where present, blended over `blend_px` at a tile's edge.
class LunarTerrain {
public:
    static constexpr double blend_m = 1000.0;          // [E] tile -> global grid
    static constexpr double normal_step_m = 10.0;      // central differences for the normal (half a 20 m pixel)
    LunarTerrain(CylindricalGrid global, std::vector<PolarStereographicTile> tiles, double sphere_radius_m);
    double height_m(double latitude_rad, double longitude_rad) const;       // above the sphere
    double height_at_fixed_m(const math::Vec3& fixed_m) const;              // of the point's direction
    math::Vec3 normal_fixed(const math::Vec3& fixed_m) const;               // outward unit normal, body-fixed axes
    bool in_tile(double latitude_rad, double longitude_rad) const;
};
}
// ReferenceBody additions:
    void set_terrain(const LunarTerrain* terrain) { terrain_ = terrain; }   // nullptr: the bare shape
    const LunarTerrain* terrain() const { return terrain_; }
    double surface_height_m(const math::Vec3& fixed_m) const;   // above the shape: terrain or 0
    math::Vec3 surface_normal_fixed(const math::Vec3& fixed_m) const;  // terrain normal or the shape's vertical
namespace starsim::config {
struct TerrainGridFile { /* the header fields of Task 3 + std::vector<std::int16_t> dn + checks */ };
TerrainGridFile load_terrain_grid_file(const std::filesystem::path& toml_path);   // throws std::runtime_error (German)
struct LunarSiteConfig { std::string id, name, product; double latitude_rad, longitude_rad, tile_half_size_m; };
LunarSiteConfig load_lunar_site(const std::filesystem::path& config_dir, std::string_view id);
std::vector<LunarSiteConfig> load_lunar_sites(const std::filesystem::path& config_dir);
}
namespace starsim::sim {
// Terrain of all sites with a tile in data/terrain plus the global grid; throws (German, names fetch.py) when
// ldem_16 or a site's tile is missing.
std::shared_ptr<const environment::LunarTerrain> load_lunar_terrain(const std::filesystem::path& config_dir,
                                                                     const std::filesystem::path& data_dir);
struct LunarSite { config::LunarSiteConfig config; double height_m = 0.0; };  // terrain height at the site
LunarSite lunar_site(const config::LunarSiteConfig& site, const environment::LunarTerrain& terrain);
}
```
Normal: `P(lat, lon) = (R + h) u(lat, lon)`; step `d = normal_step_m / R` in latitude and `d / cos(lat)` in longitude;
`n = normalize(cross(P(lat, lon+dl) - P(lat, lon-dl), P(lat+d, lon) - P(lat-d, lon)))` oriented outward
(`dot(n, u) > 0`). Blend weight in a tile: `w = smoothstep(0, blend_m / scale_m, edge_distance_px)`;
`h = w h_tile + (1 - w) h_global`; outside every tile `h_global`.

- [ ] **Step 1: Write the failing tests**

```cpp
// core/tests/test_lunar_terrain.cpp
// Lunar terrain (lunar l2): polar stereographic tile, global grid, blend, normals; the real LOLA data against the
// pipeline's check points (raw DN, bilinear).

#include <cmath>
#include <filesystem>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/lunar_site.hpp"

using namespace starsim;
using environment::LunarTerrain;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
constexpr double R = 1737400.0;

// A flat global grid at `h` m (DN = 2h) of LDEM_16's layout.
environment::CylindricalGrid flat_global(double h) {
    environment::CylindricalGrid g;
    g.grid = {2880, 5760, 0.5, std::vector<std::int16_t>(2880 * 5760, static_cast<std::int16_t>(2.0 * h))};
    g.pixels_per_degree = 16.0;
    g.zero_line = 1439.5;
    g.zero_sample = -0.5;
    return g;
}

// A 201 x 201 tile (20 m) centred on (lat, lon) whose heights rise by `slope` along +sample (DN per pixel = 2 x 20 tan).
environment::PolarStereographicTile sloped_tile(double lat, double lon, double base_m, double slope_rad) {
    environment::PolarStereographicTile t;
    t.sphere_radius_m = R;
    t.scale_m = 20.0;
    t.pole_line = t.pole_sample = 15199.5;
    t.first_line = t.first_sample = 0;
    const auto [line, sample] = t.position(lat, lon);  // in the full product (first_* = 0)
    t.first_line = static_cast<int>(std::round(line)) - 100;
    t.first_sample = static_cast<int>(std::round(sample)) - 100;
    t.grid = {201, 201, 0.5, std::vector<std::int16_t>(201 * 201)};
    for (int l = 0; l < 201; ++l)
        for (int s = 0; s < 201; ++s)
            t.grid.dn[static_cast<size_t>(l) * 201 + s] = static_cast<std::int16_t>(std::lround(2.0 * (base_m + (s - 100) * 20.0 * std::tan(slope_rad))));
    return t;
}
}  // namespace

TEST_CASE("Polar stereographic positions put the pole at the pole pixel and longitude 0 up", "[terrain]") {
    environment::PolarStereographicTile t;
    t.sphere_radius_m = R;
    t.scale_m = 20.0;
    t.pole_line = t.pole_sample = 15199.5;
    const auto [l0, s0] = t.position(-85.0 * deg, 0.0);
    CHECK(s0 == Catch::Approx(15199.5));
    CHECK(15199.5 - l0 == Catch::Approx(2.0 * R * std::tan(2.5 * deg) / 20.0));
    const auto [l90, s90] = t.position(-85.0 * deg, 90.0 * deg);
    CHECK(s90 > 15199.5);
    CHECK(l90 == Catch::Approx(15199.5));
}

TEST_CASE("Tile heights are the raw grid at nodes and bilinear between", "[terrain]") {
    const auto tile = sloped_tile(-84.79 * deg, 29.2 * deg, 5000.0, 0.0);
    CHECK(tile.grid.bilinear(10.0, 20.0) == tile.grid.at(10, 20));
    auto g = tile.grid;
    g.dn[static_cast<size_t>(10) * 201 + 20] = 100;
    g.dn[static_cast<size_t>(10) * 201 + 21] = 200;
    g.dn[static_cast<size_t>(11) * 201 + 20] = 300;
    g.dn[static_cast<size_t>(11) * 201 + 21] = 400;
    CHECK(g.bilinear(10.5, 20.5) == Catch::Approx(0.5 * 250.0));
}

TEST_CASE("Terrain normals follow the slope of the grid", "[terrain]") {
    for (double slope : {0.0, 5.0, 10.0, 15.0}) {
        const LunarTerrain terrain(flat_global(5000.0), {sloped_tile(-84.79 * deg, 29.2 * deg, 5000.0, slope * deg)}, R);
        const Vec3 p = frames::geodetic_to_fixed({-84.79 * deg, 29.2 * deg, 5000.0}, {R, 0.0});
        const Vec3 n = terrain.normal_fixed(p);
        const double tilt = std::acos(dot(n, math::normalized(p))) / deg;
        CHECK(tilt == Catch::Approx(slope).margin(0.05));  // projection scale ~1.002 here, DN rounding 0.25 m
    }
}

TEST_CASE("The terrain blends from a tile into the global grid without a step", "[terrain]") {
    // Tile 300 m above the global grid: inside its core the tile, beyond its edge the grid, in between no jump > 1 m
    // per 20 m.
    const LunarTerrain terrain(flat_global(5000.0), {sloped_tile(-84.79 * deg, 29.2 * deg, 5300.0, 0.0)}, R);
    CHECK(terrain.height_m(-84.79 * deg, 29.2 * deg) == Catch::Approx(5300.0));
    double previous = terrain.height_m(-84.79 * deg, 29.2 * deg);
    for (int k = 1; k <= 200; ++k) {  // walk 4 km along the meridian (the tile reaches 2 km)
        const double lat = -84.79 * deg + k * 20.0 / R;
        const double h = terrain.height_m(lat, 29.2 * deg);
        CHECK(std::abs(h - previous) < 1.0);
        previous = h;
    }
    CHECK(previous == Catch::Approx(5000.0));
}

TEST_CASE("LOLA data: the site tile matches the pipeline's raw-grid check points", "[terrain][data]") {
    const std::filesystem::path data = STARSIM_DATA_DIR;
    const auto file = config::load_terrain_grid_file(data / "terrain" / "mons_mouton_im2.toml");
    const auto terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, data);
    REQUIRE(!file.checks.empty());
    for (const auto& c : file.checks) {
        CHECK(terrain->height_m(c.latitude_rad, c.longitude_rad) == Catch::Approx(c.height_m).margin(1e-6));
    }
    const auto site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
    std::printf("[terrain] %s: %.1f m above the 1737.4 km sphere\n", site.config.name.c_str(), site.height_m);
    CHECK(site.height_m > 4000.0);  // Mons Mouton region: 5,000-6,000 m [S: NASA 2024]
    CHECK(site.height_m < 7000.0);
}

TEST_CASE("Missing terrain data names the fetch script", "[terrain]") {
    try {
        sim::load_lunar_terrain(STARSIM_CONFIG_DIR, std::filesystem::temp_directory_path() / "starsim_no_data");
        FAIL("expected an error");
    } catch (const std::runtime_error& e) {
        CHECK(std::string(e.what()).find("tools/data_pipeline/fetch.py") != std::string::npos);
    }
}
```
Add a reference-body test to `core/tests/test_reference_body.cpp` (l1's file):
```cpp
TEST_CASE("The Moon's surface height and normal come from its terrain", "[environment][lunar][terrain]") {
    // with the flat-global + sloped-tile terrain of test_lunar_terrain (copy the two helpers), 10 deg:
    // surface_height_m at the tile centre == 5000 (+-0.25), surface_normal_fixed tilted 10 deg; without terrain 0 / radial.
}
```
(write it out fully with the helpers copied; the Earth `ReferenceBody` returns `0.0` and the WGS84 vertical.)

- [ ] **Step 2: Run** `"[terrain]"` → FAIL (headers missing).

- [ ] **Step 3: Implement** the structs and functions above.
  - `config::load_terrain_grid_file`: `detail::Document` on the TOML; read `[grid]`; read `heights` relative to the TOML's
    directory as int16 LE (`std::ifstream`, binary, size must be `2 lines samples`); `[[check]]` into
    `std::vector<TerrainCheck>{latitude_rad, longitude_rad, height_m}`. Missing file:
    `throw std::runtime_error("Mondgelände fehlt: " + path.string() + " - bitte python tools/data_pipeline/fetch.py ausführen")`.
  - `sim::load_lunar_terrain`: global `data_dir/terrain/ldem_16.toml`, then for each site of `load_lunar_sites` its
    `data_dir/terrain/<id>.toml`; convert `TerrainGridFile` into `environment::{CylindricalGrid, PolarStereographicTile}`
    by projection name (unknown projection: German error).
  - `ReferenceBody`: `const environment::LunarTerrain* terrain_ = nullptr;` member; `surface_height_m` returns
    `terrain_ ? terrain_->height_at_fixed_m(fixed_m) : 0.0`; `surface_normal_fixed` returns
    `terrain_ ? terrain_->normal_fixed(fixed_m) : frames::local_up_fixed(frames::fixed_to_geodetic(fixed_m, shape_))`
    (use the existing WGS84 vertical helper; check its name in `frames/geodetic.hpp`).
  - `CMakeLists.txt`: `src/environment/lunar_terrain.cpp`, `src/config/lunar_site_config.cpp`, `src/sim/lunar_site.cpp`;
    tests `test_lunar_terrain.cpp`.

- [ ] **Step 4: Run** `"[terrain]"`, `"[environment]"` → PASS; record the site height in the ledger.

- [ ] **Step 5: Commit** `feat(environment): lunar terrain from LOLA tiles and the global grid`

---

### Task 5: Landing surfaces with a normal (slopes and terrain)

**Files:**
- Modify: `core/include/starsim/contact/landing_surface.hpp`, `core/src/contact/landing_surface.cpp`
- Modify: `core/src/contact/landing_legs.cpp:47-97`
- Create: `core/tests/test_terrain_contact.cpp`

**Interfaces:**
```cpp
struct LandingSurface {
    ... (deck, sea as now)
    struct Support {
        double height_m = 0.0;
        bool water = false;
        math::Vec3 normal{0.0, 0.0, 1.0};  // site frame, unit
    };
    // Ground other than the flat plane (slopes in tests, lunar terrain): replaces the plane where present.
    std::shared_ptr<const SurfaceField> field;
    Support below(double east_m, double north_m) const;  // field first, then deck / sea / plane
};
class SurfaceField {  // ground height and normal in a site frame (x east, y north, z up)
public:
    virtual ~SurfaceField() = default;
    virtual LandingSurface::Support below(double east_m, double north_m) const = 0;
};
// A plane through the site origin rising towards `uphill_azimuth_rad` (clockwise from north) at `slope_rad`.
class PlaneField final : public SurfaceField { public: PlaneField(double slope_rad, double uphill_azimuth_rad); ... };
// A reference body's terrain seen from a site frame (origin `origin_fixed_m` on the surface, columns of
// `site_to_fixed` east, north, up). Height of the surface on the vertical through (east, north, 0) of the site frame
// (exact at the vertical; the site plane's curvature drop is included).
class TerrainField final : public SurfaceField {
public:
    TerrainField(const environment::ReferenceBody& body, const math::Vec3& origin_fixed_m, const math::Mat3& site_to_fixed);
    ...
};
```
`TerrainField::below`: `q = origin + site_to_fixed * (e, n, 0)`; `u = normalize(q)`; `r = R + body.surface_height_m(q)`;
`S = r u`; `height = dot(S - origin, up)`; `normal = transpose(site_to_fixed) * body.surface_normal_fixed(S)`.
(The surface point's east/north differ from (e, n) by `|(e, n)| height / R`: < 1 mm within 100 m of the site; documented.)

`leg_contact` along the normal (identical for `normal == (0, 0, 1)`):
```cpp
        const auto support = surface.below(p.x, p.y);
        const Vec3 n = support.normal;
        const double depth = (support.height_m - p.z) * n.z;   // penetration along the normal
        if (depth <= 0.0) continue;
        ...
        const Vec3 v = pose.velocity_mps + cross(pose.angular_velocity_radps, p - pose.cg_m);
        const double v_n = dot(v, n);
        double f_n = c.stiffness_n_m * (depth - crushed_m[i]) - c.damping_n_s_m * v_n;
        ...
        const Vec3 slip = v - n * v_n;
        ...
        const Vec3 f = n * f_n + friction;
```
(For `n = (0,0,1)`: `n.z == 1.0`, `dot` = `v.z` exactly — `0*v.x + 0*v.y + 1*v.z` adds exact zeros, `v - n*v_n` gives
`(v.x, v.y, 0)` exactly: bit-identical. The `touching` pre-count and the base ring use the same `depth` test.)

- [ ] **Step 1: Write the failing tests**

```cpp
// core/tests/test_terrain_contact.cpp
// Landing surfaces with a normal (lunar l2): sloped planes and terrain under the feet.

#include <cmath>
#include <memory>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/contact/landing_legs.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;

config::LandingLegConfig legs() {
    config::LandingLegConfig c;
    c.azimuths_rad = {0.0, 60 * deg, 120 * deg, 180 * deg, 240 * deg, 300 * deg};
    c.footprint_radius_m = 8.0;
    c.foot_station_m = -1.5;
    c.stroke_m = 0.6;
    c.crush_force_n = 5e5;
    c.stiffness_n_m = 4e7;
    c.damping_n_s_m = 8e5;
    c.friction = 0.6;
    c.fixed = true;
    return c;
}
}  // namespace

TEST_CASE("A plane field rises towards its uphill azimuth with a tilted normal", "[contact][terrain]") {
    const contact::PlaneField plane(10.0 * deg, 90.0 * deg);  // rising towards the east
    const auto s = plane.below(100.0, 50.0);
    CHECK(s.height_m == Catch::Approx(100.0 * std::tan(10.0 * deg)));
    CHECK(s.normal.x == Catch::Approx(-std::sin(10.0 * deg)));
    CHECK(s.normal.z == Catch::Approx(std::cos(10.0 * deg)));
    CHECK(s.normal.y == Catch::Approx(0.0).margin(1e-15));
}

TEST_CASE("Flat ground keeps the vertical contact force", "[contact][terrain]") {
    const auto geometry = contact::make_leg_geometry(legs(), 0.0, 4.5);
    contact::LandingSurface flat;
    contact::SitePose pose;
    pose.cg_station_m = 20.0;
    pose.body_to_site = contact::from_columns({0, 0, 1}, {1, 0, 0}, {0, 1, 0});  // body x up
    pose.cg_m = {0.0, 0.0, 21.5 - 0.01};  // feet 1 cm into the ground
    std::vector<double> crushed;
    const auto c = contact::leg_contact(geometry, crushed, pose, flat);
    CHECK(c.feet_in_contact == 6);
    CHECK(c.force_n.z == Catch::Approx(6 * 4e7 * 0.01));
    CHECK(c.force_n.x == 0.0);
    CHECK(c.force_n.y == 0.0);
}

TEST_CASE("On a slope the feet push along the normal and friction holds below its angle", "[contact][terrain]") {
    const auto geometry = contact::make_leg_geometry(legs(), 0.0, 4.5);
    contact::LandingSurface slope;
    slope.field = std::make_shared<contact::PlaneField>(10.0 * deg, 90.0 * deg);
    contact::SitePose pose;
    pose.cg_station_m = 20.0;
    pose.body_to_site = contact::from_columns({0, 0, 1}, {1, 0, 0}, {0, 1, 0});
    pose.cg_m = {0.0, 0.0, 21.5 - 0.02};  // the downhill feet clear, the uphill feet in
    std::vector<double> crushed;
    const auto c = contact::leg_contact(geometry, crushed, pose, slope);
    REQUIRE(c.feet_in_contact > 0);
    // At rest no friction: the force is a sum of normals - it leans downhill (towards -east)
    CHECK(c.force_n.x < 0.0);
    CHECK(c.force_n.x / c.force_n.z == Catch::Approx(-std::tan(10.0 * deg)));
}
```
(`contact::from_columns` is in `contact/primitives.hpp`; include it.)

- [ ] **Step 2: Run** `"[contact]"` → FAIL. **Step 3: Implement.** `PlaneField::below`: uphill unit
`u = (sin a, cos a)`; `height = tan(s) (e u_x + n u_y)`; `normal = (-sin s u_x, -sin s u_y, cos s)`.
`LandingSurface::below`: `if (field) return field->below(east_m, north_m);` first. `TerrainField` as above (contact may
include `environment/reference_body.hpp`: `environment` sits below `contact`).

- [ ] **Step 4: Run** `"[contact]"`, `"[legs]"` → PASS. Before Step 3, run
`build/core/tests/starsim_core_tests.exe "[legs]" > before_legs.txt` and after it `> after_legs.txt`; `diff` must be empty
(Review Focus 3). Keep the files in the scratchpad.

- [ ] **Step 5: Commit** `feat(contact): landing surfaces with a normal - slopes and terrain under the feet`

---

### Task 6: Leg landing on any reference body; ground over the terrain

**Files:**
- Modify: `core/include/starsim/sim/leg_landing_sim.hpp`, `core/src/sim/leg_landing_sim.cpp`
- Modify: `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`
- Test: `core/tests/test_hls_landing.cpp` (new; Task 7 extends it)

**Interfaces:**
```cpp
namespace starsim::sim {
// Where a leg landing simulation sits: a point on a reference body (Earth: WGS84 geodetic height; Moon: the height above
// the sphere, i.e. the terrain height there) with the site frame east / north / up.
struct LegSite {
    ephemeris::Body body = ephemeris::Body::Earth;
    double latitude_rad = 0.0, longitude_rad = 0.0, height_m = 0.0;
    double tracking_radius_m = std::numeric_limits<double>::infinity();  // bodies farther away keep their own contact
};
class LegLandingSim {
public:
    LegLandingSim(const LegSite& site, const frames::Ellipsoid& shape, contact::LandingSurface surface, double tick_s);
    // Earth (unchanged call sites): LegSite{Earth, lat, lon, h} on WGS84.
    LegLandingSim(double latitude_rad, double longitude_rad, double height_m, contact::LandingSurface surface, double tick_s);
    void before_tick(const std::vector<vehicle::FlightBody*>& bodies, const vehicle::FlightEnvironment& env, time::Epoch epoch);
    void after_tick(const std::vector<vehicle::FlightBody*>& bodies, const vehicle::FlightEnvironment& env,
                    time::Epoch epoch_end, double dt_s);
    const LegSite& site() const;
    math::Vec3 origin_fixed_m() const;
    math::Mat3 site_to_fixed() const;
    contact::SitePose site_pose(const vehicle::FlightBody& body, const environment::BodyFrame& frame) const;
    ... (touchdown, in_contact, crushed_m, surface, clear as now; to_site(fixed))
};
}
// FlightBody:
    void hold_landed(const environment::BodyFrame& frame, ephemeris::Body body);   // generic; the Earth overload calls it
    // Leg landing simulations set this every tick for bodies with legs: true while one owns the body's ground contact
    // (default true: an Earth site's plane is unbounded, as before l2).
    void set_legs_on_site(bool on_site) { legs_on_site_ = on_site; }
// FlightSession:
    void set_landing_surface(const LegSite& site, contact::LandingSurface surface);  // any body; the Earth overload stays
    // Lunar terrain for ground, contact and telemetry at the Moon (needs a Moon orientation); nullptr: the sphere.
    void set_moon_terrain(std::shared_ptr<const environment::LunarTerrain> terrain);
    // Leg landings at a lunar site on its terrain (TerrainField), tracking within the site's tile.
    void set_lunar_landing_site(const LunarSite& site);
```

Semantics:
- `before_tick`: `frame = vehicle::reference_body_of(env, site.body).frame(epoch)`; for each body with legs out:
  `on_site = body->reference_body(env, epoch) == site.body && horizontal distance of the CG from the origin <= tracking_radius_m`;
  `body->set_legs_on_site(on_site)`; skip when not on site. Pose via `site_pose(body, frame)`:
  `cg_m = to_site(frame.to_fixed(r))`, `velocity = to_site_rot * transpose(frame.transform.rotation) * frame.relative_velocity(r, v)`,
  angular velocity `to_site_rot * transpose(R) * (attitude.rotate(omega_body) - frame.rotation_gcrf_radps)`.
  (Earth frame: centre 0, `relative_velocity = v - cross(omega, r)`: the same operations as today. Confirm with the
  `[legs]` before/after diff.)
- `after_tick`: `Landed` -> `body->hold_landed(frame_end, site.body)`.
- `FlightBody::resolve_ground_contact`: `if (vehicle_->leg_deployment() > 0.0 && legs_on_site_) return;`
- `FlightBody` ground at the Moon: `ground_height_on(id)` becomes
  `ground_height_at(const ReferenceBody& body, const Vec3& fixed_m)`: Earth -> `ground_height_m_` (unchanged); else
  `body.surface_height_m(fixed_m)`; used in `ground_clearance` (at the CG's direction) and `resolve_ground_contact`
  (at the bottom / top points). The up vector for sink and tilt stays the sphere's vertical (a terrain normal is the
  legs' business; documented).
- Session: `moon_` gets `set_terrain(terrain.get())` and the session keeps the `shared_ptr`; `set_lunar_landing_site`
  builds `LegSite{Moon, lat, lon, site.height_m, tile_half_size_m}`, the origin on the Moon sphere + height, and
  `LandingSurface{.field = std::make_shared<contact::TerrainField>(*moon_, origin, site_to_fixed)}`; refused
  (`std::runtime_error`, German "Mondlandeplatz braucht die Mondorientierung") without a Moon orientation.
  `before_tick` / `after_tick` calls pass `env_`.

- [ ] **Step 1: Write the failing tests** (`core/tests/test_hls_landing.cpp`)

```cpp
// The HLS on the Moon (lunar l2): standing on its legs at a lunar site, judged by the leg simulation; away from the
// site by its own ground contact.

#include <cmath>
#include <memory>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/leg_landing_sim.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
constexpr double site_lat = -84.7906 * deg, site_lon = 29.1957 * deg;  // config/lunar_sites.toml

struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    environment::ReferenceBody moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
};

// The HLS with `propellant_kg` in its tanks (split at O/F 3.6), engines off.
config::VehicleConfig hls(double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    for (auto& t : cfg.stages.front().tanks) {
        t.mass_kg = propellant_kg * (t.contents == config::TankContents::Oxidizer ? 3.6 / 4.6 : 1.0 / 4.6);
    }
    return cfg;
}

// Places the HLS upright over the site frame point (east, north) with its lowest foot `clearance_m` above the surface,
// at `velocity_enu_mps` relative to the Moon. Site frame from the session's leg simulation.
void place_over_site(Fixture& fx, const config::VehicleConfig& cfg, double clearance_m, const Vec3& velocity_enu_mps,
                     double east_m = 0.0, double north_m = 0.0) {
    const auto& legs = fx.session.leg_landing();
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const auto& lc = cfg.stages.front().landing_legs;
    double cg_height = 0.0;  // CG height that puts the highest-standing foot on the surface
    for (double a : lc.azimuths_rad) {
        // feet at body (y, z) = r (cos a, sin a) (make_leg_geometry); body y = north, body z = west
        const double fe = east_m - lc.footprint_radius_m * std::sin(a), fn = north_m + lc.footprint_radius_m * std::cos(a);
        const double need = legs.surface().below(fe, fn).height_m + (cg - lc.foot_station_m);
        cg_height = std::max(cg_height, need);
    }
    const Vec3 cg_site{east_m, north_m, cg_height + clearance_m};
    const auto frame = fx.moon.frame(fx.t0);
    const math::Mat3 site_to_gcrf = frame.transform.rotation * legs.site_to_fixed();
    const Vec3 r = frame.position_gcrf(legs.origin_fixed_m() + legs.site_to_fixed() * cg_site);
    const Vec3 v = frame.velocity_gcrf(legs.origin_fixed_m() + legs.site_to_fixed() * cg_site,
                                       legs.site_to_fixed() * velocity_enu_mps);
    // body x up, body y north, body z west (right-handed; as on_pad with heading 0)
    const auto attitude = math::Quat::from_matrix(site_to_gcrf * contact::from_columns({0, 0, 1}, {0, 1, 0}, {-1, 0, 0}));
    fx.session.place_body(cfg, {r, v, attitude, {}}, fx.t0);
}
}  // namespace

TEST_CASE("An HLS set down gently on the sphere at a lunar site lands and follows the Moon", "[hls][legs][lunar]") {
    Fixture fx;
    fx.session.set_landing_surface(sim::LegSite{ephemeris::Body::Moon, site_lat, site_lon, 0.0, 10000.0},
                                   contact::LandingSurface{});
    place_over_site(fx, hls(100.0e3), 0.3, {0.0, 0.0, -0.5});
    fx.session.advance_to(fx.t0 + 30.0, 100000);
    REQUIRE(fx.session.leg_touchdown(0) == sim::LegTouchdown::Landed);
    const auto fixed0 = fx.moon.frame(fx.session.epoch()).to_fixed(fx.session.body(0).state().position_gcrf_m);
    fx.session.advance_to(fx.session.epoch() + 3600.0, 100000);
    const auto fixed1 = fx.moon.frame(fx.session.epoch()).to_fixed(fx.session.body(0).state().position_gcrf_m);
    CHECK(norm(fixed1 - fixed0) < 1e-3);
    CHECK(fx.session.telemetry(0).reference_body == ephemeris::Body::Moon);
}

TEST_CASE("An HLS touching the Moon far from the landing site is judged by its own ground contact", "[hls][legs][lunar]") {
    Fixture fx;
    fx.session.set_landing_surface(sim::LegSite{ephemeris::Body::Moon, site_lat, site_lon, 0.0, 10000.0},
                                   contact::LandingSurface{});
    place_over_site(fx, hls(100.0e3), 50.0, {0.0, 0.0, -20.0}, 50000.0, 0.0);  // 50 km east, falling
    fx.session.advance_to(fx.t0 + 30.0, 100000);
    CHECK(fx.session.body(0).impacted());  // not through the ground
    CHECK(std::abs(fx.session.telemetry(0).altitude_m) < 10.0);
}
```
(Task 6 also adds `LegLandingSim::site_to_fixed()` / `origin_fixed_m()` accessors used here. The second test relies on
`FlightBody` rejecting a 20 m/s touchdown as an impact - `max_touchdown_sink_rate_mps` 3 m/s.)

- [ ] **Step 2: Run** `"[hls]"` → FAIL (no `LegSite`).

- [ ] **Step 3: Implement** as specified above. Save `"[legs]"`, `"[falcon9]"` (the fast ones: `"[legs]","[falcon9][rtls]"`
  without `[validation]`) outputs before and after; the diff must be empty.

- [ ] **Step 4: Run** `"[hls]"`, `"[legs]"`, `"[lunar]"`, `"[falcon9]"` → PASS; diff empty.

- [ ] **Step 5: Commit** `feat(sim): leg landings on any reference body; ground over the lunar terrain`

---

### Task 7: HLS drop tests on slopes and on the LOLA tile

**Files:**
- Modify: `core/tests/test_hls_landing.cpp`

- [ ] **Step 1: Write the tests**

```cpp
namespace {
// Drop on a plane of `slope` rising towards `uphill_azimuth`: lowest foot 0.5 m up, 1 m/s down (spec l4: <= 1 m/s).
sim::LegTouchdown drop_on_slope(double slope_deg, double uphill_azimuth_deg) {
    Fixture fx;
    contact::LandingSurface surface;
    surface.field = std::make_shared<contact::PlaneField>(slope_deg * deg, uphill_azimuth_deg * deg);
    fx.session.set_landing_surface(sim::LegSite{ephemeris::Body::Moon, site_lat, site_lon, 0.0, 10000.0}, surface);
    place_over_site(fx, hls(100.0e3), 0.5, {0.0, 0.0, -1.0});
    const auto t0 = fx.session.epoch();
    while (fx.session.leg_touchdown(0) == sim::LegTouchdown::None && fx.session.epoch() - t0 < 60.0) {
        fx.session.advance_to(fx.session.epoch() + 0.1, 100000);
    }
    std::printf("[hls] slope %4.1f deg uphill %5.1f deg: %s\n", slope_deg, uphill_azimuth_deg,
                sim::to_string(fx.session.leg_touchdown(0)));
    return fx.session.leg_touchdown(0);
}

// Static tip-over slope across the weakest edge (between two legs): atan(r cos(30 deg) / CG height above the feet).
double tip_over_slope_deg(const config::VehicleConfig& cfg) {
    const vehicle::Vehicle v(cfg, STARSIM_CONFIG_DIR);
    const auto& lc = cfg.stages.front().landing_legs;
    const double h = v.mass_properties().cg_station_m - lc.foot_station_m;
    return std::atan(lc.footprint_radius_m * std::cos(30.0 * deg) / h) / deg;
}
}  // namespace

TEST_CASE("HLS drop tests on 0, 5, 10 and 15 deg slopes land; 25 deg tips over", "[hls][legs][lunar]") {
    const double tip = tip_over_slope_deg(hls(100.0e3));
    std::printf("[hls] static tip-over slope across an edge: %.1f deg\n", tip);
    REQUIRE(tip > 15.0);  // the plan's leg sizing (Task 2); if not, ask the user before changing the legs
    for (double slope : {0.0, 5.0, 10.0, 15.0}) {
        CHECK(drop_on_slope(slope, 0.0) == sim::LegTouchdown::Landed);   // uphill towards a leg
        CHECK(drop_on_slope(slope, 30.0) == sim::LegTouchdown::Landed);  // uphill between two legs (weakest)
    }
    CHECK(drop_on_slope(25.0, 30.0) == sim::LegTouchdown::TippedOver);
}

TEST_CASE("An HLS lands on the LOLA terrain at the flattest spot near the site", "[hls][legs][lunar][terrain][data]") {
    Fixture fx;
    const auto terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);
    fx.session.set_moon_terrain(terrain);
    const auto site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
    fx.session.set_lunar_landing_site(site);
    // Flattest point of a 50 m lattice within 500 m: smallest tilt of the terrain normal (vs the site frame's vertical
    // at that point) - what l4's touchdown-point choice will do properly.
    const auto& surface = fx.session.leg_landing().surface();
    double best = 1e9, be = 0.0, bn = 0.0;
    for (int i = -10; i <= 10; ++i) {
        for (int j = -10; j <= 10; ++j) {
            double worst = 0.0;  // largest tilt under the footprint (centre and the six feet)
            for (int k = -1; k < 6; ++k) {
                const double a = k * 60.0 * deg;
                const double e = 50.0 * i + (k < 0 ? 0.0 : 8.0 * std::sin(a)), n = 50.0 * j + (k < 0 ? 0.0 : 8.0 * std::cos(a));
                worst = std::max(worst, std::acos(surface.below(e, n).normal.z));
            }
            if (worst < best) { best = worst; be = 50.0 * i; bn = 50.0 * j; }
        }
    }
    std::printf("[hls] site %.1f m; flattest spot (%.0f, %.0f) m, slope %.2f deg\n", site.height_m, be, bn, best / deg);
    place_over_site(fx, hls(100.0e3), 0.5, {0.0, 0.0, -1.0}, be, bn);
    const auto t0 = fx.session.epoch();
    while (fx.session.leg_touchdown(0) == sim::LegTouchdown::None && fx.session.epoch() - t0 < 60.0) {
        fx.session.advance_to(fx.session.epoch() + 0.1, 100000);
    }
    CHECK(fx.session.leg_touchdown(0) == sim::LegTouchdown::Landed);
    // The feet stand on the terrain (spring compression of a few mm, no crush at 1 m/s)
    for (double c : fx.session.leg_crushed_m(0)) CHECK(c < 0.01);
}
```
(`place_over_site` uses the session's leg simulation surface, so it places on the plane / terrain correctly.)

- [ ] **Step 2: Run** `"[hls]"`. Expected: PASS. If the 15 deg between-legs drop tips although the static angle is
> 15 deg, that is the dynamics of a 1 m/s touchdown: record the outcome and the margin in the ledger and **ask the user**
(leg sizing is an [E] decision) instead of tuning the legs silently.

- [ ] **Step 3: Commit** `test(hls): drop tests on slopes and on the LOLA terrain`

---

### Task 8: Docs, ledger, memory; full suite

**Files:**
- Modify: `CLAUDE.md` (lunar bullet: l2), `config/README.md` (engine group `station_m` / `cant_deg` / `landing_only`;
  legs `fixed`; `lunar_sites.toml`), `data/README.md` (done in Task 3)
- Memory: `project-lunar-mission.md` (l2 done, next l3)

- [ ] **Step 1:** CLAUDE.md, after the l1 paragraph of the lunar bullet:
```
  l2 HLS and terrain: `config/vehicles/starship_hls.toml` (120 t dry [E], Block 3 tanks and Raptors, no heat shield /
  flaps / pins, ship RCS; 12 landing thrusters [E] 46 kN at 40 m, canted 15 deg, `landing_only` - never in burns or
  `ignite()`; six fixed legs [E], static tip-over ~<n> deg across an edge). Engine groups take `station_m` / `cant_deg`.
  Terrain: `config/lunar_sites.toml` (default `mons_mouton_im2`, IM-2's landing point [S]); `fetch.py` crops LOLA
  `LDEM_80S_20M` (20 m/px) to 20 x 20 km per site by an HTTP range request and fetches `LDEM_16` (global) -
  `data/terrain/`; orientation checked against LDEM_16 (rms <n> m). `environment::LunarTerrain` (tile, global grid,
  1 km blend; normals by central differences) through `ReferenceBody::surface_height_m` / `surface_normal_fixed`;
  `FlightSession::set_moon_terrain`, `set_lunar_landing_site` (`sim::load_lunar_terrain`, `sim::lunar_site`). Leg
  landings work on any reference body (`sim::LegSite`, tracking radius; outside it `FlightBody`'s own contact) and press
  the feet along the surface normal (`contact::SurfaceField`: `PlaneField`, `TerrainField`). Drop tests (1 m/s): 0-15 deg
  land, 25 deg tips; on the tile's flattest spot near the site lands. Site height <n> m above the sphere.
```
- [ ] **Step 2:** Full suite in the background (`ctest -j 12`), bindings build (`BUILD_OK`), smoke test. All green.
- [ ] **Step 3:** Final review by an opus subagent over `git diff <l2 base>..HEAD`; one fix pass.
- [ ] **Step 4:** Commit `docs: lunar l2 HLS and terrain`; ledger closed; memory updated.
