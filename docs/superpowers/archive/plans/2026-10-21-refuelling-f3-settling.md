# Refuelling f3 - Settling before an in-orbit ignition - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Main-tank propellant must be settled before an engine lights in orbit: the autopilot settles it with cold gas
(+x), and an ignition on unsettled propellant ingests gas and fails the engines ("Gasansaugung").

**Architecture:** `vehicle` gets a per-stage settling progress (0..1) driven by the existing `outlet_check` (Bond
number + tilt), a PMD tank flag, and the gas-ingestion check at the one place every engine lights
(`Vehicle::update_engines`, Off -> Starting). `FlightBody` feeds the progress each tick and coast and tells the vehicle
when it is in orbit (the check applies only there). `sim` adds a settling push (an RCS +x overlay on the controlled
body's command) that the Burn autopilot, the lunar PDI and a manual `settle(bool)` command switch on; its delta-v is
credited to the burn executor; the planners reserve its gas.

**Tech Stack:** C++20 core, Catch2, godot-cpp bindings, TOML configs.

**Spec:** `docs/superpowers/specs/2026-10-21-refuelling-followups-design.md`, section "f3 - Settling before an in-orbit
ignition".

## Global Constraints

- Work and commit on `main`; push (`git push origin main`) after every task once its tests are green.
- Code, identifiers, comments, commits in English (Conventional Commits); UI strings in the core German; answers to the
  user in German.
- SI units internally; frame/unit in the name; no magic numbers (constants with provenance [S]/[E]/[D] beside them).
- Simplifications deliberate, documented and switchable: new fidelity flag `FlightEnvironment::ignition_settling`
  (default true).
- Tests: run only the affected tags (`-TestFilter`), `[slow]` cases of those tags included; tag new cases `[slow]` when
  > ~40 s; runs longer than 10 min go to the background with checks every 10 min.
- No heredocs: files and scripts with the Write tool.
- Build/test: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "<tags>"`; test exe
  `build/core/tests/starsim_core_tests.exe`.
- Spec values: settled after `settling_time_s(L, a) = sqrt(2L/a)` under +x acceleration, L = tank length minus fill
  height [D]; stays settled while Bo >= 10 (`outlet_check`); unsettles after sqrt(2L/a) with the residual acceleration
  [E]; tanks with `propellant_management_device = true` [E] always covered (Starship header tanks, Dragon `mmh_nto`);
  abort reason "Gasansaugung".

## Plan decisions (for the user's review)

1. **No hard 0.01 m/s^2 threshold.** Covered outlets settle at the rate 1/sqrt(2L/a) for any a that passes
   `outlet_check` (Bo >= 10, tilt <= 30 deg). The spec's 0.01 m/s^2 is the tanker's push, not a gate: the full HLS
   (~1,600 t) would never reach it and could never light. Cost if wrong: weak drag tail-first settles slowly (realistic).
2. **Travel floor 1 m [E].** L = max(tank length x (1 - fill), 1 m), over the stage's non-PMD main tanks with liquid
   (the longest governs). A near-full tank otherwise settles and unsettles instantly.
3. **Scope "in orbit":** the ingestion check applies when the vehicle's osculating periapsis lies above its reference
   body's equatorial radius (Earth or Moon). Ascent/hot staging, the booster's boostback and landing, the ship's
   suborbital landing flip, a lunar ascent from the surface are not checked (the spec says "in-orbit ignition"; the
   booster's simplified hot staging cuts all its engines, real ones keep three lit). The progress itself runs always.
4. **Push at the full pure +x capability** (spec: tanker ~0.010 m/s^2 -> ~77 s at 30 m travel). Finding for the docs:
   gas grows with sqrt(thrust) (gas = F/(Isp g0) x sqrt(2Lm/F)); a gentler push would save gas.
5. **Lead and hold.** The push starts `settling_lead_s = 1.25 [E] x settling_time_s(L, a_push)` before the planned
   ignition. If the stage is not settled when the executor wants to ignite, the ignition is held while the push still
   produces force, at most `settling_hold_factor = 3` [E] x the lead after the planned ignition; then it lights (and
   ingests). The coast stops `max(burn_preparation_s, lead + 30 s)` before ignition.
6. **Push delta-v credited** to the burn executor (`BurnExecutor::credit`, does not start the burn): the push points
   along the burn, its ~0.8 m/s is part of the manoeuvre instead of an error.
7. **Gas reservation:** `FlightSession::settling_gas_kg()` = push mass flow x lead for the controlled vehicle (0 when no
   settling is needed). Phasing: per planned burn (`PhasingParameters::reserve_per_burn_kg`, all burns, conservative);
   window cost (`PhasingCostModel::gas_per_burn_kg`, per raise / boost / phase burn); deorbit: once in the shortfall check;
   lunar budgets: once per leg in the reserve.
8. **Mission abort:** the ISS/refuelling mission sequence ends with the reason on any burn abort that starts with
   "Gasansaugung" (generic check at the top of the sequence update); the lunar step machines end with
   `disengage_autopilot` as for a dry burn.
9. **Initial state settled** (progress 1) for every vehicle, including those placed in orbit: the liquid was settled by
   the insertion burn; it unsettles during the coast.
10. **Docked bodies** keep their progress unchanged (the r4 transfer has its own settling); after undocking it runs on.

## Review Focus

1. Gas empty (Starship RCS draws main-tank ullage gas: main tanks dry but headers full) -> no push force -> the
   ignition lights at once and ingests; the abort reason is "Gasansaugung", not "kein Treibstoff". Test in Task 2/3.
2. A burn started by the lunar step machine only `burn_preparation_s` (120 s) before ignition while the lead is longer
   (heavy HLS): the hold must delay the ignition instead of lighting unsettled. Test in Task 3 (lead > time left).
3. Ship on header feed (after the flip / landing) or a vehicle with PMD tanks only: never ingests, never pushes. Test in
   Tasks 1-2.
4. Time warp / coast across the settling window: the coast must stop before the lead; results independent of the step
   size. Test in Task 3 (advance in 10 s and 600 s steps, same outcome).
5. The fidelity flag off reproduces the old behaviour (no push, no ingestion, no reserve). Test in Task 4.

---

### Task 1: Settling state and PMD tanks (vehicle)

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp` (TankConfig), `core/src/config/vehicle_config.cpp:284-305`
- Modify: `core/include/starsim/vehicle/vehicle.hpp` (Stage, Vehicle), `core/src/vehicle/vehicle.cpp`
- Modify: `config/vehicles/starship_block3.toml` (lox_header, ch4_header), `config/vehicles/starship_depot.toml`
  (headers), `config/vehicles/crew_dragon.toml` (mmh_nto)
- Create: `core/tests/test_ignition_settling.cpp` (register in `core/tests/CMakeLists.txt`)

**Interfaces:**
- Produces:
  - `bool config::TankConfig::propellant_management_device = false;` (TOML key `propellant_management_device`)
  - `double vehicle::Stage::settled = 1.0;` (progress 0..1)
  - `bool vehicle::Vehicle::needs_settling(size_t stage) const;` - main feed and at least one non-PMD main tank with liquid
  - `double vehicle::Vehicle::settling_travel_m(size_t stage) const;` - decision 2
  - `void vehicle::Vehicle::update_settling(const LiquidEnvironment& liquid, double dt_s);`
  - `bool vehicle::Vehicle::stage_settled(size_t stage) const;` - `!needs_settling || settled >= 1`
  - `constexpr double settling_min_travel_m = 1.0; // [E]` and `constexpr double settling_max_tilt_rad = 30 deg; // r4`
    in `vehicle/propellant_transfer.hpp`

- [ ] **Step 1: Write the failing tests** (`[settling][vehicle]`) in `core/tests/test_ignition_settling.cpp`, using the
  `orbit_liquid(...)` pattern of `test_refuelling_transfer.cpp:139-164` (copy the helper into an anonymous namespace):

```cpp
TEST_CASE("Settling: a +x push settles the main tanks in sqrt(2L/a)", "[settling][vehicle]") {
    auto v = refuelling_test::tanker_vehicle(100e3);  // tanker with 100 t main propellant (helper in refuelling_test)
    const size_t s = v.stages().size() - 1;
    REQUIRE(v.needs_settling(s));
    v.stages_mutable()[s].settled = 0.0;
    const double a = 0.010;
    const auto liquid = orbit_liquid({a, 0.0, 0.0});
    const double t = vehicle::settling_time_s(v.settling_travel_m(s), a);
    CHECK(t == Catch::Approx(std::sqrt(2.0 * v.settling_travel_m(s) / a)));
    v.update_settling(liquid, 0.5 * t);
    CHECK(v.stages()[s].settled == Catch::Approx(0.5).margin(0.02));  // the tide adds < 2 %
    CHECK_FALSE(v.stage_settled(s));
    v.update_settling(liquid, 0.6 * t);
    CHECK(v.stage_settled(s));
}

TEST_CASE("Settling: without thrust the liquid leaves the outlets", "[settling][vehicle]") {
    auto v = refuelling_test::tanker_vehicle(100e3);
    const size_t s = v.stages().size() - 1;
    const auto drag_nose_first = orbit_liquid({-2e-5, 0.0, 0.0});  // drag pushes the liquid forward
    v.update_settling(drag_nose_first, 60.0);
    CHECK(v.stages()[s].settled < 1.0);
    v.update_settling(drag_nose_first, 1e5);
    CHECK(v.stages()[s].settled == 0.0);
    const auto none = orbit_liquid({0.0, 0.0, 0.0}, /*tide=*/false);
    v.stages_mutable()[s].settled = 0.4;
    v.update_settling(none, 1e5);  // no acceleration: the liquid stays where it is
    CHECK(v.stages()[s].settled == Catch::Approx(0.4));
}

TEST_CASE("Settling: PMD tanks are always covered, header feed needs no settling", "[settling][vehicle]") {
    auto v = refuelling_test::tanker_vehicle(100e3);
    const size_t s = v.stages().size() - 1;
    for (const auto& t : v.stages()[s].tanks) {
        CHECK(t.config.propellant_management_device == (t.config.role == config::TankRole::Header));
    }
    v.select_feed(s, vehicle::FeedSource::Header);
    CHECK_FALSE(v.needs_settling(s));
    CHECK(v.stage_settled(s));
    v.select_feed(s, vehicle::FeedSource::Main);
    v.stages_mutable()[s].settled = 0.0;
    CHECK_FALSE(v.stage_settled(s));
}

TEST_CASE("Settling: the travel is the ullage of the longest main tank, at least 1 m", "[settling][vehicle]") {
    auto v = refuelling_test::tanker_vehicle(100e3);
    const size_t s = v.stages().size() - 1;
    double longest = 0.0;
    for (const auto& t : v.stages()[s].tanks) {
        if (t.config.role != config::TankRole::Main || !(t.mass_kg > 0.0)) continue;
        longest = std::max(longest, t.length_m * (1.0 - t.mass_kg / t.config.capacity_kg));
    }
    CHECK(v.settling_travel_m(s) == Catch::Approx(longest));
    for (size_t i = 0; i < v.stages()[s].tanks.size(); ++i) {
        const auto& t = v.stages()[s].tanks[i];
        if (t.config.role == config::TankRole::Main) v.set_tank_mass(s, i, t.config.capacity_kg);
    }
    CHECK(v.settling_travel_m(s) == Catch::Approx(vehicle::settling_min_travel_m));
}

TEST_CASE("Vehicle configs flag the PMD tanks", "[settling][config]") {
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    CHECK(dragon.stages.front().tanks.front().propellant_management_device);
    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    for (const auto& t : depot.stages.front().tanks) {
        CHECK(t.propellant_management_device == (t.role == config::TankRole::Header));
    }
}
```

  If a helper named here does not exist (`tanker_vehicle`, `stages_mutable`, `load_vehicle`), use the existing
  equivalent from `core/tests/refuelling_test_helpers.hpp` / `test_refuelling_vehicles.cpp` (e.g. build the Vehicle
  from `refuelling_test::tanker_config(main_kg)`) and note the name in the ledger; no production accessor is added for a
  test unless one already exists for the same purpose.

- [ ] **Step 2: Run to verify RED**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[settling]"`
Expected: compile errors on `propellant_management_device`, `settled`, `update_settling`.

- [ ] **Step 3: Implement**

  - `TankConfig`: `bool propellant_management_device = false;  // [E] always covers its outlet (diaphragm / PMD)`;
    parser: `t.propellant_management_device = tank["propellant_management_device"].value_or(false);` next to `role`.
  - TOML: `propellant_management_device = true  # [E] user 2026-10-21: header tanks / diaphragm tank, always covered`
    on the header tanks of `starship_block3` and `starship_depot` and on Dragon's `mmh_nto`.
  - `vehicle.cpp`:

```cpp
bool Vehicle::needs_settling(size_t stage_index) const {
    const auto& stage = stages_.at(stage_index);
    if (stage.feed != FeedSource::Main) return false;
    return std::any_of(stage.tanks.begin(), stage.tanks.end(), [](const Tank& t) {
        return t.config.role == config::TankRole::Main && t.config.contents != config::TankContents::Rcs &&
               !t.config.propellant_management_device && t.mass_kg > 0.0 && t.length_m > 0.0;
    });
}

double Vehicle::settling_travel_m(size_t stage_index) const {
    double travel = 0.0;
    for (const auto& t : stages_.at(stage_index).tanks) {
        if (t.config.role != config::TankRole::Main || t.config.contents == config::TankContents::Rcs ||
            t.config.propellant_management_device || !(t.mass_kg > 0.0) || !(t.config.capacity_kg > 0.0)) {
            continue;
        }
        travel = std::max(travel, t.length_m * (1.0 - t.mass_kg / t.config.capacity_kg));
    }
    return std::max(travel, settling_min_travel_m);
}

// Spec f3: covered outlets (every non-PMD main tank with liquid passes outlet_check) settle at 1/sqrt(2L/a); otherwise
// the liquid leaves them in the same time under the residual acceleration (tide, drag) [E]. a: the weakest tank's.
void Vehicle::update_settling(const LiquidEnvironment& liquid, double dt_s) {
    for (size_t s = 0; s < stages_.size(); ++s) {
        auto& stage = stages_[s];
        if (!needs_settling(s)) continue;
        bool covered = true;
        double a = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < stage.tanks.size(); ++i) {
            const auto& t = stage.tanks[i];
            if (t.config.role != config::TankRole::Main || t.config.contents == config::TankContents::Rcs ||
                t.config.propellant_management_device || !(t.mass_kg > 0.0)) {
                continue;
            }
            const auto check = outlet_check(*this, s, i, liquid, settling_max_tilt_rad);
            covered = covered && check.covered;
            const double centre_m = stage.base_station_m + t.config.base_station_m + 0.5 * t.length_m;
            a = std::min(a, norm(apparent_gravity_body_mps2(
                                liquid, math::Vec3{centre_m, 0.0, 0.0} - mass_properties().cg_point_m())));
        }
        const double t_settle = settling_time_s(settling_travel_m(s), a);
        if (!std::isfinite(t_settle)) continue;  // no acceleration: the liquid stays where it is
        stage.settled = std::clamp(stage.settled + (covered ? 1.0 : -1.0) * dt_s / t_settle, 0.0, 1.0);
    }
}

bool Vehicle::stage_settled(size_t stage_index) const {
    return !needs_settling(stage_index) || stages_.at(stage_index).settled >= 1.0;
}
```

  (`vehicle.cpp` includes `propellant_transfer.hpp` and `boil_off.hpp`; `outlet_check` already takes a `Vehicle` - no
  new dependency direction.)

- [ ] **Step 4: Run to verify GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[settling],[refuelling][vehicle],[refuelling][config],[prop_transfer]"`
Expected: all pass.

- [ ] **Step 5: Commit + push**

```bash
git add core/include/starsim/config/vehicle_config.hpp core/src/config/vehicle_config.cpp core/include/starsim/vehicle/vehicle.hpp core/src/vehicle/vehicle.cpp core/include/starsim/vehicle/propellant_transfer.hpp config/vehicles/starship_block3.toml config/vehicles/starship_depot.toml config/vehicles/crew_dragon.toml core/tests/test_ignition_settling.cpp core/tests/CMakeLists.txt
git commit -m "feat(vehicle): per-stage propellant settling state and PMD tanks"
git push origin main
```

### Task 2: Gas ingestion at an unsettled in-orbit ignition

**Files:**
- Modify: `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp:400-420` (update_engines)
- Modify: `core/include/starsim/vehicle/flight_body.hpp` (FlightEnvironment flag), `core/src/vehicle/flight_body.cpp`
  (step ~:449 and the clamped/landed branch ~:425, coast ~:600)
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (autopilot tick),
  `core/src/sim/flight_session_iss_mission.cpp` (sequence update)
- Test: `core/tests/test_ignition_settling.cpp`

**Interfaces:**
- Consumes: Task 1 `stage_settled`, `needs_settling`, `update_settling`.
- Produces:
  - `bool vehicle::FlightEnvironment::ignition_settling = true;` (fidelity flag)
  - `void vehicle::Vehicle::set_ignition_check(bool active);` and `bool vehicle::Vehicle::gas_ingestion() const;` (sticky,
    cleared by `clear_gas_ingestion()`)
  - `bool vehicle::FlightBody::in_orbit() const;` (osculating periapsis above the reference body's equatorial radius)
  - `void sim::FlightSession::set_ignition_settling(bool enabled);`
  - `constexpr const char* gas_ingestion_reason = "Gasansaugung: Treibstoff nicht gesetzt, Triebwerke ausgefallen";`
    in `flight_session.hpp` (starts with "Gasansaugung", the spec's reason)

- [ ] **Step 1: Write the failing tests** (`[settling][sim]`), on the RaiseFixture pattern (tanker placed in orbit
  through `refuelling_test::DepotSession`; copy the minimal placement, no mission):

```cpp
// A tanker coasting on 120 km (or the given periapsis) x 190 km, no depot and no mission.
struct TankerInOrbit : refuelling_test::DepotSession {
    explicit TankerInOrbit(double main_kg, double periapsis_height_m = 120e3) {
        const double re = constants::wgs84_semi_major_axis_m;
        const auto along = refuelling_test::depot_state(earth, t0, 0.0);
        const double rp = re + periapsis_height_m, ra = re + 190e3;
        const double va = std::sqrt(constants::earth_gm_m3ps2 * (2.0 / ra - 2.0 / (rp + ra)));
        // at apoapsis: the arc is above the atmosphere for the first coast either way
        const math::StateVector s{ra * math::normalized(along.position_m), va * math::normalized(along.velocity_mps)};
        session.place_in_orbit(refuelling_test::tanker_config(main_kg), s, t0);
    }
};

TEST_CASE("Unsettled ignition in orbit: the engines ingest gas and fail", "[settling][sim]") {
    test::spice_session();
    TankerInOrbit f(100e3);                      // 120 x 190 km, coasting
    f.session.advance_to(f.session.epoch() + 3600.0);  // the liquid leaves the outlets
    auto& v = f.session.body(0).vehicle();
    const size_t s = v.stages().size() - 1;
    REQUIRE_FALSE(v.stage_settled(s));
    f.session.ignite();
    f.session.advance_to(f.session.epoch() + 1.0);
    CHECK(v.gas_ingestion());
    CHECK_FALSE(v.any_engine_producing_thrust());
    for (const auto& e : v.stages()[s].engines) {
        if (!e.auxiliary() && e.has_run) CHECK(e.failed);
    }
    CHECK_THAT(f.session.burn_abort_reason(), Catch::Matchers::StartsWith("Gasansaugung"));
}

TEST_CASE("Settled ignition in orbit lights normally; the flag off never ingests", "[settling][sim]") {
    test::spice_session();
    TankerInOrbit f(100e3);
    auto& v = f.session.body(0).vehicle();
    f.session.ignite();  // just placed: settled (decision 9)
    f.session.advance_to(f.session.epoch() + 5.0);
    CHECK_FALSE(v.gas_ingestion());
    CHECK(v.any_engine_producing_thrust());

    TankerInOrbit g(100e3);
    g.session.set_ignition_settling(false);
    g.session.advance_to(g.session.epoch() + 3600.0);
    g.session.ignite();
    g.session.advance_to(g.session.epoch() + 5.0);
    CHECK_FALSE(g.session.body(0).vehicle().gas_ingestion());
    CHECK(g.session.body(0).vehicle().any_engine_producing_thrust());
}

TEST_CASE("Header feed and suborbital ignitions are not checked", "[settling][sim]") {
    test::spice_session();
    TankerInOrbit f(100e3);
    auto& v = f.session.body(0).vehicle();
    f.session.advance_to(f.session.epoch() + 3600.0);
    v.select_feed(v.stages().size() - 1, vehicle::FeedSource::Header);
    f.session.ignite();
    f.session.advance_to(f.session.epoch() + 5.0);
    CHECK_FALSE(v.gas_ingestion());
    // suborbital: same tanker on a 190 km apoapsis / -100 km periapsis arc, unsettled
    TankerInOrbit sub(100e3, /*periapsis_height_m=*/-100e3);
    sub.session.body(0).vehicle().stages_mutable().back().settled = 0.0;
    CHECK_FALSE(sub.session.body(0).in_orbit());
    sub.session.ignite();
    sub.session.advance_to(sub.session.epoch() + 5.0);
    CHECK_FALSE(sub.session.body(0).vehicle().gas_ingestion());
}

TEST_CASE("Gas ingestion ends the refuelling mission with its reason", "[settling][refuelling][iss_mission]") {
    test::spice_session();
    RaiseFixture f(100e3);           // copied from test_refuelling_missions.cpp
    f.start();
    // no push exists yet (Task 3): the ~45 min coast to the raise burn unsettles the liquid, the burn ingests
    f.fly_until(IssMissionLeg::Ended, 4.0 * 3600.0);
    CHECK_THAT(f.session.iss_mission_status(), Catch::Matchers::ContainsSubstring("Gasansaugung"));
}
```

  Task 3 deletes this last case: its first test ("The autopilot settles before a raise burn") replaces it, and the
  "no gas" case keeps the mission-abort path covered through `burn_abort_reason`.

- [ ] **Step 2: Run to verify RED**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[settling]"`
Expected: compile errors (`gas_ingestion`, `set_ignition_settling`, `in_orbit`).

- [ ] **Step 3: Implement**

  - `Vehicle::update_engines`, at the start transition:

```cpp
if (want_run && e.mode == EngineMode::Off && can_start) {
    if (ignition_check_ && stage.feed == FeedSource::Main && !stage_settled(stage_index)) {
        // Spec f3: gas at the outlets - the turbopumps ingest it and the engine is lost (deterministic) [E].
        e.mode = EngineMode::Off;
        e.spool = 0.0;
        e.starved = true;
        e.failed = true;
        e.has_run = true;
        gas_ingestion_ = true;
        continue;  // after ++index
    }
    e.mode = EngineMode::Starting;
    ...
```

    (introduce `size_t stage_index` in the stage loop; keep `++index` before `continue`.)
  - `FlightBody::step`: at its start (before the vehicle's engine update)
    `vehicle_->set_ignition_check(env.ignition_settling && in_orbit());`; after `settle_propellant`:
    `if (env.ignition_settling && !docked_) vehicle_->update_settling(liquid_now(), dt_s);` with
    `LiquidEnvironment liquid_now() const { return {specific_force_body_mps2_, state_.angular_velocity_body_radps, state_.position_gcrf_m, state_.attitude, {}}; }`.
    Clamped / caught / landed / resting branch: set every stage's `settled = 1` (the ground holds the liquid down).
  - `FlightBody::coast`: compute the end-of-coast non-gravitational specific force whenever `env.ignition_settling` (not
    only with boil-off), store it in `boil_off_specific_force_body_mps2_` as today, then
    `vehicle_->update_settling({that force, angular velocity, position, attitude, {}}, duration_s);`.
  - `FlightBody::in_orbit()`: `orbit::elements_from_state` about the reference body (`reference_body_of(env, ...)` GM
    and equatorial radius; store the GM/radius of the last step's reference in members so the getter needs no env);
    periapsis radius `a(1-e)` > equatorial radius and `e < 1`.
  - `FlightSession`: `set_ignition_settling(bool)` sets `env_.ignition_settling`. In the per-tick autopilot update, before
    the state switch: if the controlled vehicle reports `gas_ingestion()` and `burn_abort_reason_` is empty: `cutoff();
    ship_return_ = false; disengage_autopilot(); burn_abort_reason_ = gas_ingestion_reason;
    vehicle.clear_gas_ingestion();`.
  - `flight_session_iss_mission.cpp`: at the top of the sequence update (after the "lost" check), `if
    (burn_abort_reason_.rfind("Gasansaugung", 0) == 0) { iss_end(burn_abort_reason_); return; }`.

- [ ] **Step 4: Run to verify GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[settling]"`
Expected: all pass.

- [ ] **Step 5: Commit + push** (no other suite yet: Task 3 restores the flown missions; record in the ledger that
  `[refuelling][iss_mission]` flights with in-orbit main-engine burns now fail by design until Task 3)

```bash
git add core/include/starsim/vehicle/vehicle.hpp core/src/vehicle/vehicle.cpp core/include/starsim/vehicle/flight_body.hpp core/src/vehicle/flight_body.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/src/sim/flight_session_iss_mission.cpp core/tests/test_ignition_settling.cpp
git commit -m "feat(vehicle): unsettled in-orbit ignitions ingest gas and fail the engines"
git push origin main
```

### Task 3: The autopilot settles before ignition

**Files:**
- Modify: `core/include/starsim/gnc/burn_node.hpp`, `core/src/gnc/burn_node.cpp` (BurnExecutor::credit)
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (Burn case ~:290-345, tick
  loop ~:825-842, coast allowance ~:706), `core/src/sim/flight_session_burns.cpp` (start_burn),
  `core/src/sim/flight_session_lunar_descent.cpp` (DescentCoast -> PDI)
- Test: `core/tests/test_ignition_settling.cpp`, `core/tests/test_raise_to_orbit.cpp` (`[burn_node]`)

**Interfaces:**
- Consumes: Task 1/2 `stage_settled`, `settling_travel_m`, `gas_ingestion`.
- Produces:
  - `void gnc::BurnExecutor::credit(const math::Vec3& delta_v_gcrf_mps);` - subtracts from the remaining delta-v, does
    not start the burn
  - `double sim::FlightSession::settling_lead_s() const;` - decision 5 lead for the controlled vehicle's burn stage
    (0 when no settling is needed or the push has no force)
  - `double sim::FlightSession::settling_gas_kg() const;` - push mass flow x lead
  - `bool sim::FlightSession::settle(bool on);` (manual; refused while docked) and `bool sim::FlightSession::settling() const;`
  - constants `settling_lead_factor = 1.25; // [E]`, `settling_hold_factor = 3.0; // [E]`,
    `settling_coast_margin_s = 30.0; // [E]` in `flight_session.hpp`
  - autopilot status text "Settling" while the push runs before a burn

- [ ] **Step 1: Write the failing tests**

```cpp
// test_raise_to_orbit.cpp
TEST_CASE("BurnExecutor credit lowers the remaining delta-v without starting the burn", "[burn_node]") {
    // a node 100 s ahead, 10 m/s along x; credit 0.8 m/s along x
    ... // build a FlightBody in orbit as the existing [burn_node] cases do
    gnc::BurnExecutor ex(node);
    ex.credit({0.8, 0.0, 0.0});
    const auto st = ex.update(body, node.epoch - 100.0, 0.005, 2.0e6, 650.0, 1.0);
    CHECK(st.remaining_dv_mps == Catch::Approx(9.2));
    CHECK_FALSE(st.ignite);
}
```

```cpp
// test_ignition_settling.cpp
TEST_CASE("The autopilot settles before a raise burn and the burn flies", "[settling][refuelling][iss_mission]") {
    test::spice_session();
    RaiseFixture f(100e3);
    f.sequence.raise_to = mission::PhasingOrbit{re + 170e3, re + 190e3};
    f.start();
    bool saw_settling = false;
    const double gas0 = f.session.body(0).vehicle().rcs_supply_kg();
    const time::Epoch end = f.session.epoch() + 4.0 * 3600.0;
    while (f.session.epoch() < end && f.session.iss_mission_leg() != IssMissionLeg::Rendezvous &&
           f.session.iss_mission_leg() != IssMissionLeg::Ended) {
        f.session.advance_to(f.session.epoch() + 10.0);
        saw_settling = saw_settling || f.session.settling();
    }
    INFO(f.session.iss_mission_status());
    CHECK(saw_settling);
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Rendezvous);
    CHECK_FALSE(f.session.body(0).vehicle().gas_ingestion());
    // one settle: ~push mass flow x lead (hand value from settling_gas_kg at the raise), within 30 %
    CHECK(gas0 - f.session.body(0).vehicle().rcs_supply_kg() > 0.0);
}

TEST_CASE("Settling: the coast stops before the lead; the outcome does not depend on the step", "[settling][sim]") {
    // TankerInOrbit, 1 h coast, then execute_burn(node 2 h ahead, 20 m/s prograde); advance in 10 s and in 600 s steps
    // -> both: settling() seen, no ingestion, burn complete, final velocity equal within 0.05 m/s
}

TEST_CASE("Settling: a lead longer than the time left holds the ignition", "[settling][sim]") {
    // TankerInOrbit, 1 h coast, execute_burn(node 20 s ahead): the push starts at once, the ignition is held until
    // settled (no ingestion), and the burn completes; the ignition epoch is later than planned by < settling_hold_factor x lead
}

TEST_CASE("Settling: no gas -> the ignition lights at once and ingests", "[settling][sim]") {
    // TankerInOrbit with main tanks emptied to 0 except 2 t LOX (rcs supply from main ullage ~0) ... see note
}

TEST_CASE("Manual settle: the push runs until switched off and settles the tanks", "[settling][sim]") {
    // TankerInOrbit, 1 h coast; settle(true); advance lead; stage_settled; settle(false); settling() false;
    // docked: settle(true) refused
}
```

  Write the four `[settling][sim]` bodies out fully in the code (pattern of the first Task 2 case; nodes from
  `gnc::BurnNode{epoch, delta_v_gcrf}` with `session.execute_burn(node)`), with these exact checks. For "no gas": the
  Starship RCS draws main-tank ullage gas, so empty both main tanks with `set_tank_mass` (the burn's engines then cannot
  start for lack of propellant - `can_start` false, no ingestion) is **not** this case; instead use the depot vehicle
  config with its `[stage.rcs]` switched to an empty dedicated `rcs` tank in a test-local TOML directory (pattern
  `with_mission` in `test_refuelling_mission_config.cpp`) - expected: `settling()` turns on, the push gives no force,
  the hold ends at once, the ignition ingests, reason starts with "Gasansaugung" (Review Focus 1).
  Delete Task 2's last case ("Gas ingestion ends the refuelling mission"), superseded by the first case here.

- [ ] **Step 2: Run to verify RED**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[settling],[burn_node]"`
Expected: compile errors (`credit`, `settling`, `settle`).

- [ ] **Step 3: Implement**

  - `BurnExecutor::credit(dv)`: `remaining_ -= dv;` (no `started_`).
  - Session members: `bool settling_push_ = false; bool settle_manual_ = false; double settling_lead_s_ = 0.0;
    std::optional<time::Epoch> settling_hold_start_; math::Vec3 settling_acceleration_gcrf_;`.
  - `settling_lead_s()`: the burn stage = lowest stage with burn engines (as `burn_configurations`); 0 unless
    `env_.ignition_settling && body.in_orbit() && vehicle.needs_settling(stage)`; `a_push =
    rcs_pure_force_capability({1,0,0}).force_n / mass` (0 -> lead 0); lead = `settling_lead_factor *
    settling_time_s(settling_travel_m(stage), a_push)`. `settling_gas_kg()` = capability mass flow x lead.
  - `start_burn`: `settling_lead_s_ = chosen->rcs ? 0.0 : settling_lead_s(); settling_hold_start_.reset();`.
  - Burn case, before `burn_->update`: `const bool lit = any burn engine producing thrust;`
    `settling_push_ = !c.rcs && settling_lead_s_ > 0.0 && !lit && !autopilot_.burn.complete &&
    autopilot_.burn.time_to_ignition_s <= settling_lead_s_ && (settling_push_ || !vehicle.stage_settled(stage));`
    then `burn_->credit(settling_acceleration_gcrf_ * tick_s)` and pass `last_rcs_acceleration_` as today (the tick loop
    keeps the push out of it, below). Ignition gate: when `autopilot_.burn.ignite && !running && settling_push_ &&
    !vehicle.stage_settled(stage)`: start `settling_hold_start_` if unset; hold (do not set running_groups) while the push
    produced force last tick (`norm(settling_acceleration_gcrf_) > 0`) and `epoch_ - *settling_hold_start_ <
    settling_hold_factor * settling_lead_s_`; status "Settling".
  - Tick loop (`flight_session.cpp:~830`): after `command_for(i, ...)` for `i == controlled_`, if `settling_push_ ||
    settle_manual_`: `command.rcs_force_fraction = {1,0,0}; command.rcs_force_pure = true; command.rcs_keep_force =
    true;`. After the step: `settling_acceleration_gcrf_ = push ? attitude.rotate(rcs_force_body_n()) / mass : {}` and
    `last_rcs_acceleration_ = {}` when the push was the only translation (the executor gets it through `credit`).
  - Coast allowance (~:706): replace `burn_preparation_s` by `std::max(burn_preparation_s, settling_lead_s_ +
    settling_coast_margin_s)`; `settle_manual_` or `settling_push_` -> allowance 0.
  - Lunar PDI (`DescentCoast`): the coast step's epoch becomes `min(pdi - descent_attitude_lead_s, pdi_ignition -
    settling_lead_s() - settling_coast_margin_s)`; from `pdi_ignition - settling_lead_s()` until the braking guidance
    lights the engines, `settling_push_ = true` (cleared when any engine produces thrust). The braking law absorbs the
    push.
  - `settle(on)`: refused (`false`) while `docking_busy()`; sets `settle_manual_`. `settling()` = `settling_push_ ||
    settle_manual_`.
  - Clear `settling_push_` and `settling_hold_start_` in `disengage_autopilot()` and when a burn ends.

- [ ] **Step 4: Run to verify GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[settling],[burn_node]"`
Expected: all pass. Then the flown in-orbit flights, in the background (> 10 min), sharded with
`C:\Users\johan\AppData\Local\Temp\claude\C--Users-johan-Documents-KI-Projekte-Super-Heavy-Rocket-Simulator-VR\50049f1b-15e9-42ca-94c3-06bdc44e6f75\scratchpad\run_shards.sh`:
`[refuelling][iss_mission],[refuelling][rendezvous],[refuelling][mission],[iss_mission],[rendezvous],[deorbit],[lunar]`
Expected: all pass (numbers may move; tolerance failures in measured-value checks are Task 5's re-measure, recorded in
the ledger, not "fixed" here).

- [ ] **Step 5: Commit + push**

```bash
git add core/include/starsim/gnc/burn_node.hpp core/src/gnc/burn_node.cpp core/include/starsim/sim/flight_session.hpp core/src/sim/flight_session.cpp core/src/sim/flight_session_burns.cpp core/src/sim/flight_session_lunar_descent.cpp core/tests/test_ignition_settling.cpp core/tests/test_raise_to_orbit.cpp
git commit -m "feat(sim): cold-gas settling push before in-orbit ignitions"
git push origin main
```

### Task 4: Gas reservation in the planners

**Files:**
- Modify: `core/include/starsim/mission/phasing_schedule.hpp` (PhasingParameters), `core/src/mission/phasing_planner.cpp:297-301`
- Modify: `core/include/starsim/mission/iss_launch_window.hpp`, `core/src/mission/iss_launch_window.cpp` (phasing_cost)
- Modify: `core/src/sim/flight_session_rendezvous.cpp` (plan params), `core/src/sim/flight_session_return.cpp:94-114`
  (boil_off_shortfall), `core/src/sim/flight_session_lunar_descent.cpp:89`, `core/src/sim/flight_session_lunar_ascent.cpp:57-59,108-116`,
  `core/src/sim/refuelling_mission.cpp` (window cost model), `core/src/sim/refuelling_campaign_setup.cpp:112` (HLS demand)
- Test: `core/tests/test_phasing_planner.cpp`, `core/tests/test_iss_launch_window.cpp`, `core/tests/test_ignition_settling.cpp`

**Interfaces:**
- Consumes: Task 3 `settling_gas_kg()`.
- Produces: `double mission::PhasingParameters::reserve_per_burn_kg = 0.0;`, `double mission::PhasingCostModel::gas_per_burn_kg = 0.0;`

- [ ] **Step 1: Write the failing tests**

```cpp
// test_phasing_planner.cpp - a feasible plan becomes infeasible when the per-burn reserve eats the margin
TEST_CASE("Phasing budget: the settling gas per burn is reserved", "[phasing]") {
    // reuse the existing feasible Starship plan case's inputs; plan once with reserve_per_burn_kg = 0 -> feasible;
    // set propellant on board = plan.propellant_kg + reserve_kg + burns.size() * 400 - 1 and reserve_per_burn_kg = 400
    // -> infeasible, reason contains "Treibstoff reicht nicht"; with + 1 instead of - 1 -> feasible
}

// test_iss_launch_window.cpp
TEST_CASE("Window cost: the settling gas per burn", "[launch_window]") {
    // the existing "cost by hand" inputs with gas_per_burn_kg = 350: propellant grows by 350 x (raise burns + boost
    // burns + phase burn) exactly
}

// test_ignition_settling.cpp
TEST_CASE("Settling gas: the session's estimate and the reservation", "[settling][sim]") {
    // TankerInOrbit: settling_gas_kg() == capability mass flow x lead (hand value from rcs_pure_force_capability and
    // settling_travel_m); set_ignition_settling(false) -> 0; header feed -> 0
}
```

  Write the bodies fully, copying the inputs of the named existing cases.

- [ ] **Step 2: Run to verify RED**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[settling],[phasing],[launch_window]"`
Expected: compile errors (`reserve_per_burn_kg`, `gas_per_burn_kg`).

- [ ] **Step 3: Implement**

  - Planner budget: `const double reserve = p.reserve_kg + p.reserve_per_burn_kg * out.burns.size();` in the check and
    the message.
  - `phasing_cost`: `propellant_kg += model.gas_per_burn_kg * burns` (raise burns of the estimate + the schedule's burns).
  - Session: Starship phasing plans set `reserve_per_burn_kg = settling_gas_kg()`; `boil_off_shortfall` requires
    `margin * plan.propellant_kg + settling_gas_kg()`; lunar budgets add `legs.size() * settling_gas_kg()` to the
    reserve (descent, ascent and `lunar_landing_mission.cpp:149`); refuelling window cost model
    `gas_per_burn_kg` from a vehicle built for the window (the same helper as `phasing_exhaust_velocity_mps`: a function
    `double settling_gas_estimate_kg(const vehicle::Vehicle&)` in `refuelling_mission.cpp`, using 1.25 x sqrt(2L/a) with
    the vehicle's own travel and +x capability - the session's `settling_gas_kg()` calls the same function);
    `refuelling_campaign_setup.cpp:112` adds `legs.size() * settling_gas_estimate_kg(hls)` to the HLS reserve.
  - All of it is 0 with `ignition_settling` off (Review Focus 5).

- [ ] **Step 4: Run to verify GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[settling],[phasing],[launch_window],[refuelling][campaign]"`
Expected: all pass except recorded measured-value drifts (Task 5).

- [ ] **Step 5: Commit + push**

```bash
git add -A core config
git commit -m "feat(mission): planners reserve the settling gas of each in-orbit ignition"
git push origin main
```

### Task 5: Interface, re-measure, docs

**Files:**
- Modify: `bindings/src/flight_sim.hpp`, `bindings/src/flight_sim.cpp` (command `settle(bool)` via `post_active`,
  snapshot key `settling` and `settled` per controlled stage in the refuelling/flight snapshot)
- Modify: `config/refuelling/reference_flights.toml` (re-measured), tests that pin its values
  (`test_refuelling_campaign.cpp` file literals)
- Modify: `docs/subsystems.md` (new section "### Settling before an in-orbit ignition (f3)"), `CLAUDE.md` (vehicle and
  sim module summary: settling state, PMD, gas ingestion, settling push)
- Memory: `project-orbital-refuelling.md` (f3 COMPLETE, NEXT writing-plans f4)

- [ ] **Step 1: Bindings.** `settle(enabled)`:
  `post_active("settle", [enabled](sim::FlightSession& s) { const bool ok = s.settle(enabled); return Outcome{ok, godot::String::utf8(ok ? "" : "Settling nicht möglich: angedockt")}; });`
  plus `ClassDB::bind_method(D_METHOD("settle", "enabled"), &FlightSim::settle);` and the snapshot keys. Build with
  bindings: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -TestFilter "[settling]"`; smoke test
  `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd`.
  Expected: pass.
- [ ] **Step 2: Re-measure.** Run the r5 reference flights (`[refuelling][mission]` slow cases, background, sharded),
  read the measured values (rendezvous propellant, delivered LOX/CH4, settling gas, main at catch, HLS main at docking),
  write them into `reference_flights.toml` with the date 2026-10-21 (f3) and update the pinned literals; run
  `build/core/refuelling_report.exe` for the campaign answer.
  Expected: the file check of the slow flights passes; the answer recorded.
- [ ] **Step 3: Docs.** subsystems.md section with: model (progress, travel, PMD, scope), push and lead numbers (tanker
  lead, gas per settle, HLS), the sqrt(thrust) finding, decisions 1-10, the new campaign answer; CLAUDE.md summary.
- [ ] **Step 4: Affected tags once more** (background, sharded): `[settling],[burn_node],[phasing],[launch_window],[refuelling],[iss_mission],[rendezvous],[deorbit],[lunar]`.
  Expected: all pass.
- [ ] **Step 5: Commit + push**

```bash
git add bindings config/refuelling/reference_flights.toml core/tests docs/subsystems.md CLAUDE.md
git commit -m "docs: refuelling f3 settling before in-orbit ignition"
git push origin main
```
