# Falcon 9 Vehicle Model and Evidence (c1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Falcon 9 Block 5 becomes a flyable vehicle. It has landing-relevant engine groups, grid fins, RCS, leg geometry
and a descent drag table, plus the SLC-40 launch site and the LZ-40 landing zone. It reaches orbit on the existing
autopilot, and its unguided descent reproduces webcast telemetry of one RTLS and one ASDS flight.

**Architecture:** Configuration first (`config/vehicles/falcon9_block5.toml`, `config/launch_sites.toml`) with small
parser extensions (`[stage.landing_legs]`, site `kind` / `pad_radius_m`). `FlightSession::launch_from_pad` gets an
overload that takes a `VehicleConfig` (payload per mission). Evidence comes from our OCR tool with a new `f9` layout,
and `tools/telemetry/drag_estimate.py` identifies the descent ballistic coefficient. Validation tests mirror
`test_ascent_guidance.cpp` and `test_descent_telemetry.cpp`.

**Tech Stack:** C++20 core (`config`, `sim`), Catch2, TOML/CSV config, Python tools (Pillow, ffmpeg).

**Spec:** `docs/superpowers/specs/2026-10-13-falcon9-landing-design.md` (sections "c1 design", "Decomposition").

## Global Constraints

- SI units, radians, frame and unit in names; English code/docs, German UI strings; Conventional Commits.
- Every config value gets a provenance tag: [S] sourced, [E] estimate, [D] derived.
- Only Block 5. Its data come from flights that flew it (2026 flights).
- Only free data. Webcast values are transcribed by our own tool, the videos stay local (`data/webcasts/`, git-ignored),
  and every source goes into `tools/data_pipeline/sources.toml`.
- Display names generic: "Drohnenschiff", "Landezone"; trademarks internal only.
- `python tools/vehicle_report.py falcon9_block5` after every vehicle config change.

## Evidence chosen (web search 2026-10-13)

- **LZ-40** is the Falcon 9 landing zone inside the SLC-40 complex. First landing: Crew-12, 2026-02-13. Its latest
  landings are Crew-13 (2026-10-01) and CRS-35 (2026-10-13). LZ-1 was decommissioned in August 2025; LZ-2 serves
  Falcon Heavy. Sources: Wikipedia "Cape Canaveral Space Launch Complex 40", Teslarati "Crew-12 ... first landing
  LZ-40".
- **RTLS flight:** CRS-35 (2026-10-13) or Crew-13 (2026-10-01), whichever webcast the user provides.
- **ASDS flight:** a recent Starlink from Florida (user choice).

## Review Focus

1. **Grouping of the nine Merlins.** `center` + `pair` must sum, with `ring`, to 9 engines with one centre on the
   axis. A 3-engine set (`center`+`pair`) must have its thrust centroid on the axis, else boostback and entry burns
   tumble. Task 1 tests the centroid.
2. **Stack with legs stowed.** The old bottom-point ground contact must still apply to the Falcon 9 stack on the pad
   (no mount at SLC-40: it stands on the ground). Task 3 tests liftoff from the pad without an impact.
3. **Payload override.** Changing the payload in a loaded `VehicleConfig` must change the liftoff mass, not just a
   field. Task 3 test.
4. **Telemetry gaps.** The webcast hides the first-stage strip during camera cuts. The replay must skip gaps and
   judge only samples with a reading. Task 5 test uses `speed_mps < 0` as "no reading", like the Super Heavy test.
5. **Ascent drag table units.** `aero/falcon9_ascent.csv` was never flown. A wrong column would show up as an ascent
   that fails or reaches orbit far too easily. Task 3 checks propellant left in a plausible band.

---

### Task 1: Falcon 9 vehicle configuration (engine groups, grid fins, RCS, legs, recovery)

**Files:**
- Modify: `config/vehicles/falcon9_block5.toml`
- Modify: `core/include/starsim/config/vehicle_config.hpp`, `core/src/config/vehicle_config.cpp` (`[stage.landing_legs]`)
- Create: `config/aero/falcon9_descent.csv` (initial [E] table: tail-first cylinder, replaced by [D] in Task 5)
- Test: `core/tests/test_vehicle_models.cpp`

**Interfaces:**
- Produces:
  ```cpp
  struct LandingLegConfig {
      std::vector<double> azimuths_rad;  // one leg per entry, from body +y towards +z
      double footprint_radius_m = 0.0;   // foot from the axis, deployed
      double foot_station_m = 0.0;       // foot below the stage bottom: negative (deployed)
      double deploy_time_s = 0.0;
      double stroke_m = 0.0;             // crush-core stroke per leg
      double crush_force_n = 0.0;        // per leg
      double stiffness_n_m = 0.0;        // before the core crushes
      double damping_n_s_m = 0.0;
      double friction = 0.0;
  };
  // StageConfig::landing_legs (LandingLegConfig; azimuths empty = no legs)
  ```
- Engine group ids: `first_stage/center` (1, radius 0), `first_stage/pair` (2, angles 0 / 180 deg on the ring),
  `first_stage/ring` (6, angles 45, 90, 135, 225, 270, 315 deg). Recovery method `"leg_landing"`.

- [ ] **Step 1: Write the failing tests** in `core/tests/test_vehicle_models.cpp`:

```cpp
TEST_CASE("Falcon 9 engine groups: 1 centre + 2 pair + 6 ring, 3-engine set on the axis", "[config][falcon9]") {
    const auto cfg = config::load_vehicle(config_dir, "falcon9_block5");
    const auto& s = cfg.stages.front();
    int total = 0;
    for (const auto& g : s.engine_groups) total += g.count;
    CHECK(total == 9);
    vehicle::Vehicle v(cfg, config_dir);
    // Thrust centroid of centre + pair lies on the axis (lateral offset < 1 mm).
    const auto c = v.thrust_centroid_body_m({"first_stage/center", "first_stage/pair"});
    CHECK(std::hypot(c.y, c.z) < 1e-3);
    CHECK(s.recovery_method == "leg_landing");
    CHECK(s.grid_fins.azimuths_rad.size() == 4);
    CHECK(s.rcs_thruster_count > 0);
    REQUIRE(s.landing_legs.azimuths_rad.size() == 4);
    CHECK(s.landing_legs.footprint_radius_m > 8.0);
    CHECK(s.landing_legs.footprint_radius_m < 10.0);
    CHECK_FALSE(s.aero.descent_drag_table.empty());
}
```
  If `Vehicle::thrust_centroid_body_m` does not exist, use the existing helper `gnc::thrust_axis_body` (b4) /
  the engine positions of the vehicle; find it with `grep -rn "thrust_axis_body\|centroid" core/include`.

- [ ] **Step 2: Run, expect FAIL.**
  Run: `build/core/tests/starsim_core_tests.exe "[falcon9]"`. Expected: FAIL (`landing_legs` is not a member, then
  the group count).

- [ ] **Step 3: Implement.**
  Parser, after `catch_pins`, in the same style:
  ```cpp
  if (auto legs = t["landing_legs"].as_table()) {
      // azimuths_deg like catch_pins; the rest positive except foot_station_m
      s.landing_legs.footprint_radius_m = doc.positive(*legs, "footprint_radius_m");
      s.landing_legs.foot_station_m = doc.number(*legs, "foot_station_m");
      s.landing_legs.deploy_time_s = doc.positive(*legs, "deploy_time_s");
      s.landing_legs.stroke_m = doc.positive(*legs, "stroke_m");
      s.landing_legs.crush_force_n = doc.positive(*legs, "crush_force_n");
      s.landing_legs.stiffness_n_m = doc.positive(*legs, "stiffness_n_m");
      s.landing_legs.damping_n_s_m = doc.positive(*legs, "damping_n_s_m");
      s.landing_legs.friction = doc.positive(*legs, "friction");
  }
  ```
  Config (first stage), replacing the single `octaweb` group and `grid_fins = 4`:
  ```toml
  [[stage.engine_group]]
  id = "center"                            # landing engine
  engine = "merlin1d"
  count = 1
  ring_radius_m = 0.0
  gimbal = true
  relight = true

  [[stage.engine_group]]
  id = "pair"                              # with the centre: the 3-engine set (boostback, entry burn)
  engine = "merlin1d"
  count = 2
  ring_radius_m = 1.25                     # [E] octaweb ring from imagery (3.66 m core)
  angles_deg = [0.0, 180.0]
  gimbal = true
  relight = true

  [[stage.engine_group]]
  id = "ring"
  engine = "merlin1d"
  count = 6
  ring_radius_m = 1.25                     # [E]
  angles_deg = [45.0, 90.0, 135.0, 225.0, 270.0, 315.0]
  gimbal = true
  relight = false

  [stage.aero]
  reference_diameter_m = 3.66
  ascent_drag_table = "aero/falcon9_ascent.csv"
  descent_drag_table = "aero/falcon9_descent.csv"   # tail first; [E] until Task 5 identifies it [D]

  [stage.grid_fins]
  azimuths_deg = [45.0, 135.0, 225.0, 315.0]   # [E] four fins between the legs' azimuths
  area_m2 = 1.8                                # [E] titanium fin ~1.2 x 1.5 m frontal lattice
  station_m = 39.0                             # [E] hinge near the interstage top (stage 41.2 m)
  radius_m = 2.6                               # [E] 1.83 m core radius + half the fin span
  max_deflection_deg = 20.0                    # [E]
  rate_deg_s = 30.0                            # [E] hydraulic (open loop) actuators
  table = "aero/grid_fin.csv"

  [stage.landing_legs]                         # all [E] from imagery and public figures; physics in c2
  azimuths_deg = [0.0, 90.0, 180.0, 270.0]
  footprint_radius_m = 9.0                     # span ~18 m
  foot_station_m = -1.5                        # feet below the engine bell exit when deployed
  deploy_time_s = 2.0
  stroke_m = 0.5                               # crush core
  crush_force_n = 4.0e5                        # per leg: 4 legs ~ 6 g on a 27 t stage
  stiffness_n_m = 2.0e7
  damping_n_s_m = 2.0e5
  friction = 0.6

  [stage.recovery]
  method = "leg_landing"                       # pad (RTLS) or drone ship, chosen by the mission (user 2026-10-13)
  reserve_propellant_kg = 30000.0              # [E] MECO without return guidance
  landing_propellant_kg = 6000.0               # [E] at landing-burn ignition; Task 5 derives it [D]
  nominal_meco_s = 150.0                       # [E] replaced from the telemetry in Task 5
  nominal_meco_altitude_m = 68000.0            # [E]

  [stage.rcs]
  type = "cold_gas"                            # nitrogen, two pods at the top
  thruster_count = 8                           # [E]
  thrust_per_thruster_n = 450.0                # [E]
  ```
  `config/aero/falcon9_descent.csv`: copy the column layout of `aero/super_heavy_descent.csv` (`head -3` it) with a
  [E] header comment "tail-first cylinder estimate, replaced in Task 5". Values: the Super Heavy table's coefficients
  (same shape: blunt base first), since both are tail-first cylinders.

- [ ] **Step 4: Run, PASS.**
  `build/core/tests/starsim_core_tests.exe "[falcon9]"` and `"[config]"`.
  `python tools/vehicle_report.py falcon9_block5`: expect liftoff ~549 t, first-stage thrust ~7.6 MN.

- [ ] **Step 5: Commit** `feat: Falcon 9 first stage with landing engine groups, grid fins, RCS and legs`.

### Task 2: Launch site SLC-40 and landing zone LZ-40

**Files:**
- Modify: `config/launch_sites.toml`
- Modify: `core/include/starsim/config/launch_site_config.hpp`, `core/src/config/launch_site_config.cpp`
- Test: `core/tests/test_launch_site_config.cpp`

**Interfaces:**
- Produces: `LaunchSiteConfig::kind` (`"launch_pad"` default, `"landing_zone"`), `LaunchSiteConfig::pad_radius_m`
  (landing zones; 0 otherwise). Site ids `ccsfs_slc40`, `ccsfs_lz40`.

- [ ] **Step 1: Write the failing test:**

```cpp
TEST_CASE("SLC-40 and its landing zone LZ-40 load", "[config][falcon9]") {
    const auto pad = config::load_launch_site(config_dir, "ccsfs_slc40");
    CHECK(pad.kind == "launch_pad");
    CHECK_FALSE(pad.tower.has_value());
    const auto lz = config::load_launch_site(config_dir, "ccsfs_lz40");
    CHECK(lz.kind == "landing_zone");
    CHECK(lz.pad_radius_m > 20.0);
    REQUIRE(lz.sea.has_value());
    // LZ-40 lies inside the SLC-40 complex: less than 1.5 km from the pad.
    const double d = frames::surface_distance_m(pad.latitude_rad, pad.longitude_rad, lz.latitude_rad, lz.longitude_rad);
    CHECK(d < 1500.0);
}
```
  Use the existing great-circle/geodetic helper (`grep -rn "distance" core/include/starsim/frames`). If there is
  none, compute the distance from `geodetic_to_fixed` of both points.

- [ ] **Step 2: Run, expect FAIL** (unknown site / no member `kind`).

- [ ] **Step 3: Implement.**
  Parser: `kind = text_or(site, "kind", "launch_pad")`, validated to the two values; `pad_radius_m =
  number_or(site, "pad_radius_m", 0.0)`, required > 0 for a landing zone.
  Config:
  ```toml
  [[site]]
  id = "ccsfs_slc40"
  name = "CCSFS SLC-40"
  latitude_deg = 28.56194     # [S: Wikipedia SLC-40 infobox 28°33'43"N]
  longitude_deg = -80.57722   # [S: 80°34'38"W]
  height_m = -20.0            # [E] ~5 m above sea level, geoid ~ -25 m

  [site.sea]
  direction_deg = 90.0        # [E] Atlantic to the east
  shoreline_distance_m = 450.0  # [E] verify against imagery

  [[site]]
  id = "ccsfs_lz40"
  name = "Landezone LZ-40"
  kind = "landing_zone"
  latitude_deg = 28.5640      # [E] inside the SLC-40 complex, north of the pad (verify against imagery)
  longitude_deg = -80.5760    # [E]
  height_m = -20.0            # [E]
  pad_radius_m = 40.0         # [E] concrete landing pad (LZ-1: 86 m diameter [S: Wikipedia LZ-1/2])

  [site.sea]
  direction_deg = 90.0        # [E]
  shoreline_distance_m = 350.0  # [E]
  ```

- [ ] **Step 4: Run, PASS;** run `"[config]"` too.
- [ ] **Step 5: Commit** `feat: launch site SLC-40 and landing zone LZ-40`.

### Task 3: Falcon 9 ascent to orbit on the autopilot

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`
  (overload `launch_from_pad(const config::VehicleConfig&, const LaunchSite&, double heading_rad, time::Epoch)`;
  the id version calls it)
- Test: `core/tests/test_falcon9_ascent.cpp` (new; add to `core/tests/CMakeLists.txt`)

**Interfaces:**
- Produces: `void FlightSession::launch_from_pad(const config::VehicleConfig& config, const LaunchSite& site,
  double heading_rad, time::Epoch epoch)`.

- [ ] **Step 1: Write the failing tests:**

```cpp
TEST_CASE("Falcon 9 payload override changes the liftoff mass", "[sim][falcon9]") {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    // ... session as in test_ascent_guidance.cpp: spice_session(), SpiceEphemeris, SpiceOrientation::earth()
    session.launch_from_pad(cfg, slc40, 45.0 * deg, t0);
    const double m0 = session.body(0).vehicle().mass_properties().mass_kg;
    cfg.stages.back().payload_mass_kg = 10000.0;
    session.launch_from_pad(cfg, slc40, 45.0 * deg, t0);
    CHECK(session.body(0).vehicle().mass_properties().mass_kg - m0 == Approx(10000.0).margin(1.0));
}

TEST_CASE("Autopilot flies Falcon 9 from SLC-40 into a 200 km orbit", "[gnc][validation][ascent][falcon9]") {
    // Stack with a 10 t payload [E] (CRS class), launch site ccsfs_slc40, target 200 x 200 km, i = 51.6 deg northerly.
    // Fly as fly_to_orbit in test_ascent_guidance.cpp (1 s steps, 900 s limit, no impact allowed).
    REQUIRE(state == sim::AutopilotState::Orbit);
    CHECK(std::abs(periapsis_km - 200.0) < 5.0);
    CHECK(std::abs(apoapsis_km - 200.0) < 5.0);
    CHECK(std::abs(inclination_deg - 51.6) < 0.1);
    CHECK(propellant_t > 0.5);   // upper stage left with something
    CHECK(propellant_t < 30.0);  // and not absurdly much (Review Focus 5)
    CHECK_FALSE(any_impact_on_pad_before_liftoff);  // Review Focus 2
}
```
  Build the site with `sim::launch_site_from_config(config::load_launch_site(cfg_dir, "ccsfs_slc40"))`.
  Record MECO time and altitude (`[validation]` printf) for Task 5's comparison.

- [ ] **Step 2: Run, expect FAIL** (no overload).
- [ ] **Step 3: Implement** the overload (move the body of the id version, which then loads the config and forwards).
  If the ascent fails, diagnose with `STARSIM_VERBOSE=1` (systematic debugging). Likely suspects: the pitch program
  defaults are tuned for Starship (`gnc::PitchProgram`); a Falcon 9 program goes into the test (and later the mission
  config) rather than changing the default. Ledger any such ruling.
- [ ] **Step 4: Run, PASS.** Full ctest.
- [ ] **Step 5: Commit** `feat: Falcon 9 reaches orbit from SLC-40 on the ascent autopilot`.

### Task 4: Webcast telemetry of the Falcon 9 flights (needs the videos)

**Files:**
- Modify: `tools/telemetry/webcast_telemetry.py` (layout `f9`: first-stage and second-stage speed/altitude boxes)
- Create: `tests/validation/telemetry/f9_<mission>_<date>.csv` (RTLS and ASDS)
- Modify: `tests/validation/telemetry/README.md`, `tests/validation/telemetry/flights.toml`,
  `tools/data_pipeline/sources.toml`

- [ ] **Step 1:** The user puts both webcasts into `data/webcasts/` (`f9_crs35_2026-10-13.mp4` or Crew-13, and the
  Starlink ASDS flight). Until then Tasks 1-3 run; Task 4 waits (ledger note).
- [ ] **Step 2:** `locate`, then extract frames at a few times with ffmpeg and measure the field boxes on them
  (Read the PNGs). The Falcon 9 strip shows "STAGE 1" speed and altitude on the left and "STAGE 2" on the right.
  Add the boxes as `BOXES_F9` and the `--layout f9` choice. Reuse `read_number` with the strip templates; rebuild
  the templates (`templates` command) if the font differs (outlier test passes < 95 % of readings).
- [ ] **Step 3:** Scan liftoff -15 s to touchdown at 2 fps; `clean` to the table. Check by eye on 5 frames:
  the readings must match the frame exactly.
- [ ] **Step 4:** Record facts in `flights.toml` (mission, date, booster serial, landing site, payload class,
  sources) and the sources in `sources.toml` (same licence text as the Starship tables).
- [ ] **Step 5: Commit** `test: Falcon 9 webcast telemetry of an RTLS and an ASDS flight`.

### Task 5: Descent drag, burn data and replay validation

**Files:**
- Modify: `tools/telemetry/drag_estimate.py` (vehicle parameters as arguments instead of Super Heavy constants)
- Modify: `config/aero/falcon9_descent.csv` ([D] over the Mach range the telemetry covers, [E] elsewhere)
- Modify: `config/vehicles/falcon9_block5.toml` (`landing_propellant_kg`, `nominal_meco_*` [D])
- Create: `tools/telemetry/f9_events.py` (burn events from the table: MECO, boostback, entry burn, landing-burn ignition)
- Test: `core/tests/test_falcon9_descent_telemetry.cpp` (new)

- [ ] **Step 1:** `f9_events.py` finds the burns as intervals where |dv/dt + g sin(gamma)| exceeds the drag
  prediction by > 5 m/s^2. It prints each burn's start/end speed and altitude, and the delta-v. The landing
  propellant [D] is the rocket equation on the landing burn's delta-v plus the gravity loss over its duration, at the
  Merlin's sea-level Isp, ending at the dry mass.
- [ ] **Step 2: Write the failing replay test**, modelled on `test_descent_telemetry.cpp`. The stage
  (dry + landing propellant) is placed unguided, tail first, at the first sample after the entry burn. Its state comes
  from the speed and from the altitude fit (`altitude_fit`, flight-path angle from the slope), on the launch azimuth
  plane. It is advanced to the telemetry's landing-burn ignition altitude.
  Expect:
  - speed over altitude within 3 % rms of the readings (only samples with a reading)
  - the altitude where it reaches the ignition speed within 300 m of the telemetry's
- [ ] **Step 3: Run, expect FAIL** with the [E] table.
- [ ] **Step 4:** Identify the table with `drag_estimate.py` on both flights (ballistic coefficient over Mach -> C_D
  at the reference area). Write it into `falcon9_descent.csv` with [D] rows and [E] rows outside the covered range.
  Re-run until PASS.
- [ ] **Step 5:** Ascent comparison. The Task 3 test's MECO time and altitude must lie within 10 % of the RTLS
  flight's MECO (payload as flown, [E] mass from the mission). Add those checks to the Task 3 test. Full ctest.
- [ ] **Step 6: Commit** `feat: Falcon 9 descent drag from webcast telemetry, landing propellant derived`.

### Task 6: Docs

- [ ] Update `CLAUDE.md` (Falcon 9 vehicle and sites, recovery method `leg_landing`, the telemetry tables, and the
  remark "Falcon 9 ... not modelled yet" narrowed to "landing in c2-c4"), `config/README.md` (`[stage.landing_legs]`,
  site `kind`).
- [ ] Commit `docs: Falcon 9 vehicle model and evidence (c1)`.
