# Refuelling r6: Campaign Calculator and Game Interface - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Answer "how many tanker flights does a lunar mission need" from the measured r5 reference flights: a pure
campaign calculator, its physics setup from config (real launch windows, heat-balance boil-off, drag, lunar budget),
a committed measured-numbers file checked by the flights, a `refuelling_report` CLI and the FlightSim interface.

**Architecture:** `mission::refuelling_campaign` is a pure event loop (daily loss steps, tanker arrivals, HLS
arrival) over injected models. `sim::plan_refuelling_campaign` builds those models from config and the r5 numbers:
tanker and HLS windows by the existing `mission::plan_launch_window`, depot boil-off by
`vehicle::orbit_average_boil_off` with the day's Sun, drag makeup by US76, the HLS demand by the lunar mission's
planned outbound flight and `lunar_budget_required_mass`. The CLI and `FlightSim.refuelling_campaign` call the setup.

**Tech Stack:** C++20 core (float64), Catch2, toml++ via `config::detail::Document`, CSPICE adapters, godot-cpp.

**Spec:** `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md` (section "r6 design"; r5 corrections).

## Global Constraints

- SI units internally, frame and unit in names; German UI / refusal strings, English code and comments.
- No magic numbers: every new value carries a provenance tag [S]/[E]/[D] and a comment, constants in `constants.hpp`.
- Calculator is a pure function (no SPICE, no session); external libraries only behind their module.
- `bindings/` stays thin: no physics, only conversion to Dictionaries and posting to the worker.
- Tests per CLAUDE.md: run only the affected tags (`-TestFilter`), new cases > ~40 s tagged `[slow]`; ctest runs
  cases in parallel (no shared temp files).
- Commits: Conventional Commits, on `main`, push after each task once its tests are green.
- The tanker count is compared with the public range (4 to high teens, "10-ish") for plausibility only - nothing is
  tuned towards it (physics over gauges).

## Plan decisions (await the user's approval)

1. **Real launch windows instead of a fixed cadence:** every tanker and the HLS get their window from
   `mission::plan_launch_window` against the depot's J2-advanced orbit (as the r5 flights; 48 h uncrewed phasing
   from the mission config). `launch_interval_s` is only the pad turnaround [E] 1 day; one depot port: an arrival
   waits `port_spacing_s` [E] 6 h after the previous one (docking + 550 s transfer + departure ~3 h measured). This
   answers r3's open cadence question with the flown window logic (first window 79.8 h after the earliest).
2. **HLS demand from physics:** the lunar mission of `hls_refuel` (`hls_lunar_llo_depot`) is placed in a scratch
   session and its outbound flight planned; the budget legs (`lunar_mission_legs`, refactored out of
   `lunar_mission_budget`) give the **smallest** mass that closes the budget (`lunar_budget_required_mass`), not
   full tanks. Demand from the depot = (required propellant + HLS boil-off while waiting - HLS propellant at docking
   (r5: 184.5 t main)) x (1 + vent) + chill.
3. **Tanker net per flight** from the r5 numbers (delivered 67.5 t at 200 km): + its own boil-off over the phasing
   time difference to the reference window (orbit average of the tanker with its residue, the day's Sun); tanker dry
   mass lever x 0.99 [D] (r5 probes: -10 / -20 t dry -> +9.9 / +19.8 t residue); reserve lever 1:1; depot altitude
   by the Hohmann raise from the reference and the deorbit difference (single Raptor `sea_level_3`, margin 1.10).
   The phasing burns themselves are taken as the reference flight's (the coelliptic schedule fixes their size).
4. **Depot daily losses:** boil-off by `orbit_average_boil_off` at the day's fill and Sun (depot attitude as flown:
   identity in LVLH); drag makeup end-on (Cd 2.2 = `FlightSession::dragon_free_molecular_cd`, area pi (d/2)^2, US76
   mean density, no co-rotating atmosphere) burnt by one Raptor `sea_level_3`; attitude-hold gas **20 kg/day [E]** (the
   flights fly the depot in ideal LVLH: nothing measured).
5. **Transfer losses not in the flights:** pressurant + vented displaced gas **1.5 % [E]** of every transfer (r5:
   1-2 %), plus the measured chill 0.42 t [D] for the HLS transfer.
6. **Measured-numbers file** `config/refuelling/reference_flights.toml` ([D] from the r5 pad flights); the slow r5
   campaigns check it (1 % or 0.3 t, whichever is larger).
7. **Campaign clock:** t = 0 at the tanker mission's `earliest_utc` with the depot in orbit holding its measured
   initial fill (55.5 t); the depot orbit is `tanker_flight`'s `[depot]` at the altitude lever. No coupling to the
   lunar epoch: the report names the HLS docking date and the lunar mission's epoch; a late campaign is a statement,
   not a failure.
8. **Game:** `FlightSim.refuelling_campaign(params)` posted to the worker (window searches take seconds), the result
   Dictionary in `result(id)["data"]`; getters `transfer_state()` and `depot_state()`. UI by the user.
9. **CLI** `refuelling_report` (core build, `core/apps/`), flags for every lever; English labels like
   `vehicle_report.py`, German reasons as the core writes them.

## Review Focus

- A lever that makes the tanker deliver <= 0 (very high depot, huge reserve) must end with a German reason, not loop
  until `max_tankers` -> Task 4 test "a tanker that delivers nothing ends the campaign".
- Demand above the depot's tank volume or the HLS needing more than its tanks hold -> immediate German reason, no
  window search -> Tasks 4 and 5 tests.
- Arrivals out of order (a later launch with a shorter phasing) on the single depot port -> Task 4 test "port
  spacing pushes an arrival to the next window".
- `refuelling_campaign` called from Godot with unknown / wrongly typed keys -> refused with a message, defaults used
  for absent keys -> Task 8 smoke test.
- The measured-numbers file drifting from the flights after a later model change -> Task 6 checks in the slow
  campaigns.

---

### Task 1: Lunar budget inverse and its legs

**Files:**
- Modify: `core/include/starsim/mission/lunar_budget.hpp`, `core/src/mission/lunar_budget.cpp`
- Modify: `core/include/starsim/sim/lunar_landing_mission.hpp`, `core/src/sim/lunar_landing_mission.cpp:126-144`
- Test: `core/tests/test_lunar_descent.cpp` (budget cases live there), `core/tests/test_refuelling_campaign.cpp` (new;
  add to `core/tests/CMakeLists.txt`)

**Interfaces:**
- Produces: `double mission::lunar_budget_required_mass(double dry_mass_kg, double reserve_kg, const std::vector<BudgetLeg>& legs);`
- Produces: `std::vector<mission::BudgetLeg> sim::lunar_mission_legs(const config::LunarLandingMission&, const mission::OutboundPlan&, double vacuum_isp_s, double approach_isp_s);`

- [ ] **Step 1: Write the failing tests**

In `test_lunar_descent.cpp` after "The budget closes or names the leg ...":

```cpp
TEST_CASE("The required mass closes the budget with zero margin", "[mission][lunar_descent_plan]") {
    const std::vector<mission::BudgetLeg> legs{{"Abstieg", 1000.0, 350.0}, {"Aufstieg", 500.0, 380.0}};
    const double g0 = constants::standard_gravity_mps2;
    const double expected = 110e3 * std::exp(500.0 / (380.0 * g0)) * std::exp(1000.0 / (350.0 * g0));
    const double m = mission::lunar_budget_required_mass(100e3, 10e3, legs);
    CHECK(m == Catch::Approx(expected).epsilon(1e-12));
    const auto b = mission::lunar_budget(m, 100e3, 10e3, legs);
    CHECK(b.closes);
    CHECK(b.margin_kg == Catch::Approx(0.0).margin(1e-6));
    CHECK(mission::lunar_budget_required_mass(100e3, 10e3, {}) == Catch::Approx(110e3));
}
```

New file `core/tests/test_refuelling_campaign.cpp`:

```cpp
// Refuelling r6: campaign calculator, its setup from config, the measured-numbers file.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/mission_config.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"

using namespace starsim;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
}

TEST_CASE("lunar_mission_legs: the LLO mission's legs in flight order", "[refuelling][campaign][lunar]") {
    const auto m = config::load_lunar_landing_mission(config_dir, "hls_lunar_llo_depot");
    mission::OutboundPlan plan;
    plan.tli.delta_v_gcrf_mps = {3100.0, 0.0, 0.0};
    plan.insertion.delta_v_gcrf_mps = {0.0, 900.0, 0.0};
    const auto legs = sim::lunar_mission_legs(m, plan, 375.0, 330.0);
    REQUIRE(legs.size() == 5);
    CHECK(legs[0].name == "TLI");
    CHECK(legs[0].delta_v_mps == Catch::Approx(3100.0));
    CHECK(legs[1].name == "LOI");
    CHECK(legs[1].delta_v_mps == Catch::Approx(900.0));
    CHECK(legs[3].isp_s == Catch::Approx(330.0));  // the approach on the landing set
    CHECK(legs[4].name == "Aufstieg");
    CHECK(legs[2].delta_v_mps + legs[3].delta_v_mps == Catch::Approx(m.descent_delta_v_mps));
}
```

(`OutboundPlan`'s fields `tli` / `insertion` with `delta_v_gcrf_mps` are what `lunar_mission_budget` reads today;
if the struct needs more initialisation, build it as the existing lunar tests do.)

- [ ] **Step 2: Run, verify RED**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[lunar_descent_plan],[campaign]"`
Expected: compile error - `lunar_budget_required_mass` / `lunar_mission_legs` not declared.

- [ ] **Step 3: Implement**

`lunar_budget.hpp`:

```cpp
// The smallest initial mass that closes the budget (refuelling r6): dry + reserve carried back through every leg's
// rocket equation, last leg first.
double lunar_budget_required_mass(double dry_mass_kg, double reserve_kg, const std::vector<BudgetLeg>& legs);
```

`lunar_budget.cpp`:

```cpp
double lunar_budget_required_mass(double dry_mass_kg, double reserve_kg, const std::vector<BudgetLeg>& legs) {
    double m = dry_mass_kg + reserve_kg;
    for (auto it = legs.rbegin(); it != legs.rend(); ++it) {
        m *= std::exp(it->delta_v_mps / (it->isp_s * constants::standard_gravity_mps2));
    }
    return m;
}
```

`lunar_landing_mission.hpp` (above `lunar_mission_budget`):

```cpp
// The budget legs of the mission (refuelling r6: the campaign's HLS demand), in flight order.
std::vector<mission::BudgetLeg> lunar_mission_legs(const config::LunarLandingMission& mission,
                                                   const mission::OutboundPlan& plan, double vacuum_isp_s,
                                                   double approach_isp_s);
```

`lunar_landing_mission.cpp`: move the leg building of `lunar_mission_budget` into `lunar_mission_legs` unchanged;
`lunar_mission_budget` becomes `return mission::lunar_budget(initial_mass_kg, dry_mass_kg, m.reserve_kg, lunar_mission_legs(m, plan, vacuum_isp_s, approach_isp_s));`.

- [ ] **Step 4: Run, verify GREEN** - same command plus `[lunar]` (the refactor touches every lunar start):
`-TestFilter "[lunar_descent_plan],[campaign],[lunar]"`. Expected: all pass.

- [ ] **Step 5: Commit + push**

```bash
git add core/include/starsim/mission/lunar_budget.hpp core/src/mission/lunar_budget.cpp core/include/starsim/sim/lunar_landing_mission.hpp core/src/sim/lunar_landing_mission.cpp core/tests/test_lunar_descent.cpp core/tests/test_refuelling_campaign.cpp core/tests/CMakeLists.txt
git commit -m "feat(mission): lunar budget required mass and the mission's budget legs"
git push origin main
```

### Task 2: Measured-numbers file

**Files:**
- Create: `config/refuelling/reference_flights.toml`
- Create: `core/include/starsim/config/refuelling_reference.hpp`, `core/src/config/refuelling_reference.cpp` (add to
  `core/CMakeLists.txt` next to `src/config/mission_config.cpp`)
- Modify: `config/README.md` (new file described)
- Test: `core/tests/test_refuelling_campaign.cpp`

**Interfaces:**
- Produces:

```cpp
namespace starsim::config {
struct RefuellingReference {
    struct Depot { std::string mission; double initial_propellant_kg = 0.0; } depot;
    struct Tanker {
        std::string mission;
        double dry_mass_kg = 0.0, residue_after_ascent_kg = 0.0, rendezvous_kg = 0.0;
        double delivered_oxidizer_kg = 0.0, delivered_fuel_kg = 0.0, chilldown_kg = 0.0, settling_gas_kg = 0.0;
        double main_at_undock_kg = 0.0, main_at_catch_kg = 0.0;
        double dry_mass_sensitivity = 0.0;  // residue gained per kg of dry mass removed
    } tanker;
    struct Hls { std::string mission; double main_at_docking_kg = 0.0; } hls;
};
RefuellingReference load_refuelling_reference(const std::filesystem::path& config_dir);
}
```

- [ ] **Step 1: Write the failing test**

```cpp
#include "starsim/config/refuelling_reference.hpp"
#include "starsim/config/vehicle_config.hpp"

TEST_CASE("The measured-numbers file loads and matches its vehicles", "[refuelling][campaign][config]") {
    const auto r = config::load_refuelling_reference(config_dir);
    CHECK(r.depot.mission == "depot_launch");
    CHECK(r.depot.initial_propellant_kg == Catch::Approx(55.5e3));
    CHECK(r.tanker.mission == "tanker_flight");
    CHECK(r.tanker.delivered_oxidizer_kg + r.tanker.delivered_fuel_kg == Catch::Approx(67.5e3));
    CHECK(r.tanker.dry_mass_sensitivity == Catch::Approx(0.99));
    CHECK(r.hls.mission == "hls_refuel");
    CHECK(r.hls.main_at_docking_kg == Catch::Approx(184.5e3));
    // the reference tanker is the configured one
    const auto tanker = config::top_stage_config(config::load_vehicle(config_dir, "starship_tanker"));
    CHECK(tanker.stages.back().dry_mass_kg == Catch::Approx(r.tanker.dry_mass_kg));
}
```

- [ ] **Step 2: Run, verify RED** - `-TestFilter "[campaign]"`: compile error (header missing).

- [ ] **Step 3: Implement**

`config/refuelling/reference_flights.toml`:

```toml
# Refuelling r6: measured numbers of the r5 reference flights (pad starts, 2026-10-20 epochs) - the campaign
# calculator's inputs. [D] all: read from the flights' leg marks (core/tests/refuelling_measurements.hpp) and checked by
# the slow r5 campaigns (1 % or 0.3 t). Re-measure after any change to the vehicles, ascents, phasing or transfer.

[depot]
mission = "depot_launch"
initial_propellant_t = 55.5        # [D] main tanks after the ascent (199.6 x 211.7 km)

[tanker]
mission = "tanker_flight"
dry_mass_t = 209.0                 # [D] the configured tanker (starship_tanker) the numbers belong to
residue_after_ascent_t = 99.2      # [D] main tanks at the Rendezvous mark (170 x 190 km after the raise)
rendezvous_t = 17.8                # [D] Rendezvous -> Docked: burns, RCS gas, boil-off (phasing 39.7 h)
delivered_oxidizer_t = 54.6        # [D] into the depot
delivered_fuel_t = 12.9            # [D]
chilldown_t = 0.42                 # [D] vented
settling_gas_t = 0.94              # [D]
main_at_undock_t = 12.6            # [D] the reserve kept for the return
main_at_catch_t = 2.74             # [D] what the reserve left over
dry_mass_sensitivity = 0.99        # [D] r5 probes: ship dry -10 / -20 t -> residue +9.9 / +19.8 t

[hls]
mission = "hls_refuel"
main_at_docking_t = 184.5          # [D] HLS main tanks at the Docked mark
```

`refuelling_reference.hpp` declares the struct above with a header comment ("Refuelling r6: the r5 reference
flights' measured numbers, `config/refuelling/reference_flights.toml`"). `refuelling_reference.cpp`:

```cpp
#include "starsim/config/refuelling_reference.hpp"

#include "toml_document.hpp"

namespace starsim::config {

namespace {
constexpr double kg_per_t = 1000.0;
}

RefuellingReference load_refuelling_reference(const std::filesystem::path& config_dir) {
    const detail::Document doc(config_dir / "refuelling" / "reference_flights.toml");
    const auto* depot = doc.root["depot"].as_table();
    const auto* tanker = doc.root["tanker"].as_table();
    const auto* hls = doc.root["hls"].as_table();
    if (!depot || !tanker || !hls) doc.fail("missing [depot], [tanker] or [hls]");
    const auto tonnes = [&](const toml::table& t, std::string_view key) {
        const double v = doc.number(t, key);
        if (!(v >= 0.0)) doc.fail("'" + std::string(key) + "' must be >= 0");
        return v * kg_per_t;
    };
    RefuellingReference r;
    r.depot.mission = doc.text(*depot, "mission");
    r.depot.initial_propellant_kg = tonnes(*depot, "initial_propellant_t");
    r.tanker.mission = doc.text(*tanker, "mission");
    r.tanker.dry_mass_kg = tonnes(*tanker, "dry_mass_t");
    r.tanker.residue_after_ascent_kg = tonnes(*tanker, "residue_after_ascent_t");
    r.tanker.rendezvous_kg = tonnes(*tanker, "rendezvous_t");
    r.tanker.delivered_oxidizer_kg = tonnes(*tanker, "delivered_oxidizer_t");
    r.tanker.delivered_fuel_kg = tonnes(*tanker, "delivered_fuel_t");
    r.tanker.chilldown_kg = tonnes(*tanker, "chilldown_t");
    r.tanker.settling_gas_kg = tonnes(*tanker, "settling_gas_t");
    r.tanker.main_at_undock_kg = tonnes(*tanker, "main_at_undock_t");
    r.tanker.main_at_catch_kg = tonnes(*tanker, "main_at_catch_t");
    r.tanker.dry_mass_sensitivity = doc.number(*tanker, "dry_mass_sensitivity");
    r.hls.mission = doc.text(*hls, "mission");
    r.hls.main_at_docking_kg = tonnes(*hls, "main_at_docking_t");
    return r;
}

}  // namespace starsim::config
```

(`doc.text` is the Document helper the mission loaders use; if its name differs, use that helper.)

- [ ] **Step 4: Run, verify GREEN** - `-TestFilter "[campaign],[config]"`.

- [ ] **Step 5: Commit + push** - `feat(config): r5 reference flights' measured numbers for the campaign calculator`.

### Task 3: Altitude and drag costs

**Files:**
- Create: `core/include/starsim/mission/refuelling_costs.hpp`, `core/src/mission/refuelling_costs.cpp` (CMake)
- Test: `core/tests/test_refuelling_campaign.cpp`

**Interfaces:**
- Produces:

```cpp
namespace starsim::mission {
// Total of both burns of a Hohmann transfer between circular radii (>= 0, either direction).
double hohmann_delta_v_mps(double from_radius_m, double to_radius_m, double gm_m3ps2);
// Lowering the perigee of a circular orbit to `perigee_radius_m` (the deorbit burn, one impulse at apoapsis).
double deorbit_delta_v_mps(double radius_m, double perigee_radius_m, double gm_m3ps2);
// Propellant flow that makes up the drag of a body in a circular orbit: 0.5 rho v^2 Cd A / exhaust velocity
// (impulsive makeup, small delta-v: the rocket equation linearised).
double drag_makeup_kgps(double density_kg_m3, double speed_mps, double drag_coefficient, double area_m2,
                        double exhaust_velocity_mps);
}
```

- [ ] **Step 1: Write the failing tests**

```cpp
#include "starsim/math/constants.hpp"
#include "starsim/mission/refuelling_costs.hpp"

TEST_CASE("Hohmann, deorbit and drag makeup against hand values", "[refuelling][campaign]") {
    const double mu = constants::earth_gm_m3ps2;
    const double r1 = constants::wgs84_semi_major_axis_m + 200e3;
    const double r2 = constants::wgs84_semi_major_axis_m + 300e3;
    const double a = 0.5 * (r1 + r2);
    const double expected = (std::sqrt(mu * (2 / r1 - 1 / a)) - std::sqrt(mu / r1)) +
                            (std::sqrt(mu / r2) - std::sqrt(mu * (2 / r2 - 1 / a)));
    CHECK(mission::hohmann_delta_v_mps(r1, r2, mu) == Catch::Approx(expected).epsilon(1e-12));
    CHECK(mission::hohmann_delta_v_mps(r2, r1, mu) == Catch::Approx(expected).epsilon(1e-12));
    CHECK(expected == Catch::Approx(57.0).margin(1.0));  // ~57 m/s from 200 to 300 km
    const double rp = constants::wgs84_semi_major_axis_m + 50e3;
    const double d = std::sqrt(mu / r1) - std::sqrt(mu * (2 / r1 - 2 / (r1 + rp)));
    CHECK(mission::deorbit_delta_v_mps(r1, rp, mu) == Catch::Approx(d).epsilon(1e-12));
    CHECK(mission::deorbit_delta_v_mps(r2, rp, mu) > d);
    CHECK(mission::drag_makeup_kgps(2.5e-10, 7800.0, 2.2, 63.6, 3600.0) ==
          Catch::Approx(0.5 * 2.5e-10 * 7800.0 * 7800.0 * 2.2 * 63.6 / 3600.0));
}
```

- [ ] **Step 2: RED** - `-TestFilter "[campaign]"`: header missing.

- [ ] **Step 3: Implement** (`refuelling_costs.cpp`):

```cpp
#include "starsim/mission/refuelling_costs.hpp"

#include <cmath>

namespace starsim::mission {

double hohmann_delta_v_mps(double from_radius_m, double to_radius_m, double gm_m3ps2) {
    const double a = 0.5 * (from_radius_m + to_radius_m);
    const auto speed = [&](double r, double sma) { return std::sqrt(gm_m3ps2 * (2.0 / r - 1.0 / sma)); };
    return std::abs(speed(from_radius_m, a) - speed(from_radius_m, from_radius_m)) +
           std::abs(speed(to_radius_m, to_radius_m) - speed(to_radius_m, a));
}

double deorbit_delta_v_mps(double radius_m, double perigee_radius_m, double gm_m3ps2) {
    const double a = 0.5 * (radius_m + perigee_radius_m);
    return std::sqrt(gm_m3ps2 / radius_m) - std::sqrt(gm_m3ps2 * (2.0 / radius_m - 1.0 / a));
}

double drag_makeup_kgps(double density_kg_m3, double speed_mps, double drag_coefficient, double area_m2,
                        double exhaust_velocity_mps) {
    return 0.5 * density_kg_m3 * speed_mps * speed_mps * drag_coefficient * area_m2 / exhaust_velocity_mps;
}

}  // namespace starsim::mission
```

- [ ] **Step 4: GREEN** - `-TestFilter "[campaign]"`.
- [ ] **Step 5: Commit + push** - `feat(mission): Hohmann, deorbit and drag makeup costs for the refuelling campaign`.

### Task 4: Pure campaign calculator

**Files:**
- Create: `core/include/starsim/mission/refuelling_campaign.hpp`, `core/src/mission/refuelling_campaign.cpp` (CMake)
- Test: `core/tests/test_refuelling_campaign.cpp`

**Interfaces:**
- Produces:

```cpp
namespace starsim::mission {

struct CampaignFlight {
    double launch_s = 0.0;   // from the campaign start
    double arrival_s = 0.0;  // at the depot (Waypoint 0)
    double delivered_kg = 0.0;  // tankers: what it hands over (before the transfer vent)
};

struct CampaignModels {
    std::function<double(double t_s, double depot_propellant_kg)> boil_off_kgps;
    std::function<double(double t_s)> drag_makeup_kgps;
    // The first window at or after `earliest_launch_s`; std::nullopt: none within the search.
    std::function<std::optional<CampaignFlight>(double earliest_launch_s)> next_tanker;
    std::function<std::optional<CampaignFlight>(double earliest_launch_s)> next_hls;
};

struct CampaignInputs {
    double depot_capacity_kg = 0.0;  // main tanks
    double depot_initial_kg = 0.0;
    double demand_kg = 0.0;          // what the depot must hold when the HLS arrives
    double attitude_gas_kgps = 20.0 / 86400.0;  // [E] attitude hold (the flights fly an ideal LVLH depot)
    double transfer_vent_fraction = 0.015;      // [E] pressurant + vented displaced gas (r5: 1-2 %, not modelled)
    double launch_interval_s = 86400.0;         // [E] pad turnaround between launches
    double port_spacing_s = 6.0 * 3600.0;       // [E] one depot port: docking, transfer, departure (~3 h flown)
    double window_retry_s = 3600.0;             // [E] a window too close to the last arrival: search on from here
    double step_s = 86400.0;                    // loss integration step (daily)
    int max_tankers = 60;
};

struct CampaignLosses {
    double boil_off_kg = 0.0, drag_makeup_kg = 0.0, attitude_gas_kg = 0.0, transfer_vent_kg = 0.0;
};
struct CampaignSample { double t_s = 0.0; double depot_propellant_kg = 0.0; };
struct CampaignTanker { CampaignFlight flight; double received_kg = 0.0; double depot_after_kg = 0.0; };

struct CampaignResult {
    bool closes = false;
    std::string reason;  // German when it does not close
    int tanker_flights = 0;
    double duration_s = 0.0;  // campaign start -> HLS arrival
    std::optional<CampaignFlight> hls;
    double depot_at_hls_kg = 0.0;
    std::vector<CampaignTanker> tankers;
    std::vector<CampaignSample> fill;  // after every step and every arrival
    CampaignLosses losses;             // until the HLS arrival (or the end)
};

// The tanker campaign (refuelling r6, spec "r6 design"): from t = 0 the depot loses boil-off, drag makeup and
// attitude gas in steps of at most step_s; tankers launch at their windows (launch_interval_s apart, arrivals
// port_spacing_s apart) and add delivered x (1 - vent), clamped to the tank volume; once the fill reaches the demand,
// the HLS's window is checked - the campaign closes when the depot still holds the demand at its arrival. Ends with a
// German reason when the demand exceeds the volume, a window is missing, a tanker adds nothing, or max_tankers.
CampaignResult refuelling_campaign(const CampaignInputs& inputs, const CampaignModels& models);

}  // namespace starsim::mission
```

- [ ] **Step 1: Write the failing tests**

```cpp
#include "starsim/mission/refuelling_campaign.hpp"

namespace {
constexpr double day = 86400.0;

mission::CampaignModels constant_models(double boil_t_per_day, double tanker_t, double phasing_days) {
    mission::CampaignModels m;
    m.boil_off_kgps = [=](double, double) { return boil_t_per_day * 1e3 / day; };
    m.drag_makeup_kgps = [](double) { return 0.0; };
    m.next_tanker = [=](double e) { return mission::CampaignFlight{e, e + phasing_days * day, tanker_t * 1e3}; };
    m.next_hls = [](double e) { return mission::CampaignFlight{e, e + day, 0.0}; };
    return m;
}

mission::CampaignInputs simple_inputs() {
    mission::CampaignInputs in;
    in.depot_capacity_kg = 1000e3;
    in.depot_initial_kg = 50e3;
    in.demand_kg = 300e3;
    in.attitude_gas_kgps = 0.0;
    in.transfer_vent_fraction = 0.0;
    in.port_spacing_s = 0.0;
    return in;
}
}  // namespace

TEST_CASE("Campaign against a hand computation", "[refuelling][campaign]") {
    // Tankers launch daily, arrive after 2 days with 100 t; the depot loses 1 t/day.
    // T1 arrives day 2: 50 - 2 + 100 = 148 t; T2 day 3: 247 t; T3 day 4: 346 t >= 300 t;
    // the HLS launches day 3 (one interval after T3), arrives day 4 (port spacing 0): 346 t.
    const auto r = mission::refuelling_campaign(simple_inputs(), constant_models(1.0, 100.0, 2.0));
    REQUIRE(r.closes);
    CHECK(r.tanker_flights == 3);
    CHECK(r.duration_s == Catch::Approx(4 * day));
    CHECK(r.depot_at_hls_kg == Catch::Approx(346e3));
    CHECK(r.losses.boil_off_kg == Catch::Approx(4e3));
    REQUIRE(r.tankers.size() == 3);
    CHECK(r.tankers[0].depot_after_kg == Catch::Approx(148e3));
    CHECK(r.tankers[1].flight.launch_s == Catch::Approx(day));
    CHECK(r.fill.front().depot_propellant_kg == Catch::Approx(50e3));
}

TEST_CASE("Campaign: vent and attitude gas are booked as losses", "[refuelling][campaign]") {
    auto in = simple_inputs();
    in.transfer_vent_fraction = 0.1;
    in.attitude_gas_kgps = 1e3 / day;
    const auto r = mission::refuelling_campaign(in, constant_models(0.0, 100.0, 2.0));
    REQUIRE(r.closes);
    // T1 day 2: 50 - 2 + 90 = 138; T2 day 3: 227; T3 day 4: 316 >= 300; HLS day 4.
    CHECK(r.tanker_flights == 3);
    CHECK(r.depot_at_hls_kg == Catch::Approx(316e3));
    CHECK(r.losses.transfer_vent_kg == Catch::Approx(30e3));
    CHECK(r.losses.attitude_gas_kg == Catch::Approx(4e3));
}

TEST_CASE("Campaign refusals", "[refuelling][campaign]") {
    SECTION("demand above the tank volume") {
        auto in = simple_inputs();
        in.demand_kg = 1200e3;
        const auto r = mission::refuelling_campaign(in, constant_models(1.0, 100.0, 2.0));
        CHECK_FALSE(r.closes);
        CHECK(r.reason.find("Depotvolumen") != std::string::npos);
        CHECK(r.tankers.empty());
    }
    SECTION("losses eat the delivery") {
        auto in = simple_inputs();
        in.launch_interval_s = 2 * day;
        const auto r = mission::refuelling_campaign(in, constant_models(60.0, 100.0, 2.0));
        CHECK_FALSE(r.closes);
        CHECK(r.reason.find("Verluste") != std::string::npos);
        CHECK(r.tanker_flights < 5);
    }
    SECTION("a tanker that delivers nothing ends the campaign") {
        const auto r = mission::refuelling_campaign(simple_inputs(), constant_models(0.0, 0.0, 2.0));
        CHECK_FALSE(r.closes);
        CHECK(r.reason.find("liefert nichts") != std::string::npos);
        CHECK(r.tanker_flights == 1);
    }
    SECTION("no window") {
        auto m = constant_models(1.0, 100.0, 2.0);
        m.next_tanker = [](double) { return std::optional<mission::CampaignFlight>{}; };
        const auto r = mission::refuelling_campaign(simple_inputs(), m);
        CHECK_FALSE(r.closes);
        CHECK(r.reason == "Kein Startfenster für Tanker 1");
    }
    SECTION("max_tankers") {
        auto in = simple_inputs();
        in.max_tankers = 2;
        const auto r = mission::refuelling_campaign(in, constant_models(1.0, 100.0, 2.0));
        CHECK_FALSE(r.closes);
        CHECK(r.reason.find("2 Tankerflüge") != std::string::npos);
    }
}

TEST_CASE("Campaign: port spacing pushes an arrival to the next window", "[refuelling][campaign]") {
    auto in = simple_inputs();
    in.port_spacing_s = 6.0 * 3600.0;
    auto m = constant_models(0.0, 100.0, 2.0);
    // the second tanker's first window would arrive 1 h after the first one: it must take a later window
    m.next_tanker = [](double e) {
        const double arrival = e < 1.0 ? 2 * day : (e < day + 3600.0 ? 2 * day + 3600.0 : e + day);
        return mission::CampaignFlight{e, arrival, 100e3};
    };
    const auto r = mission::refuelling_campaign(in, m);
    REQUIRE(r.tankers.size() >= 2);
    CHECK(r.tankers[1].flight.arrival_s >= r.tankers[0].flight.arrival_s + in.port_spacing_s);
    CHECK(r.tankers[1].flight.launch_s > day);
}

TEST_CASE("Campaign: a full depot clamps the delivery", "[refuelling][campaign]") {
    auto in = simple_inputs();
    in.depot_capacity_kg = 320e3;
    in.demand_kg = 310e3;
    const auto r = mission::refuelling_campaign(in, constant_models(0.0, 100.0, 2.0));
    REQUIRE(r.closes);
    CHECK(r.tankers.back().depot_after_kg == Catch::Approx(320e3));
    CHECK(r.tankers.back().received_kg == Catch::Approx(70e3));  // 250 t + 70 t fills 320 t
}
```

- [ ] **Step 2: RED** - `-TestFilter "[campaign]"`: header missing.

- [ ] **Step 3: Implement** (`refuelling_campaign.cpp`):

```cpp
#include "starsim/mission/refuelling_campaign.hpp"

#include <algorithm>
#include <cstdio>

namespace starsim::mission {

namespace {

std::string tonnes(double kg) {
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%.1f t", kg / 1000.0);
    return buffer;
}

struct Depot {
    double t_s = 0.0;
    double propellant_kg = 0.0;
    CampaignLosses losses;
};

// Losses from depot.t_s to `to_s` in steps of at most step_s (rates at each step's start).
void advance(Depot& d, double to_s, const CampaignInputs& in, const CampaignModels& m,
             std::vector<CampaignSample>* samples) {
    while (d.t_s < to_s) {
        const double dt = std::min(in.step_s, to_s - d.t_s);
        const double boil = m.boil_off_kgps(d.t_s, d.propellant_kg) * dt;
        const double drag = m.drag_makeup_kgps(d.t_s) * dt;
        const double gas = in.attitude_gas_kgps * dt;
        // an emptied depot loses only what it holds: book the losses in proportion
        const double total = boil + drag + gas;
        const double share = total > d.propellant_kg && total > 0.0 ? d.propellant_kg / total : 1.0;
        d.losses.boil_off_kg += boil * share;
        d.losses.drag_makeup_kg += drag * share;
        d.losses.attitude_gas_kg += gas * share;
        d.propellant_kg -= total * share;
        d.t_s += dt;
        if (samples) samples->push_back({d.t_s, d.propellant_kg});
    }
}

// The first window at or after `earliest` that arrives port_spacing_s after `last_arrival_s`.
std::optional<CampaignFlight> next_slot(const std::function<std::optional<CampaignFlight>(double)>& next,
                                        double earliest, double last_arrival_s, const CampaignInputs& in) {
    auto slot = next(earliest);
    while (slot && slot->arrival_s < last_arrival_s + in.port_spacing_s) {
        slot = next(slot->launch_s + in.window_retry_s);
    }
    return slot;
}

}  // namespace

CampaignResult refuelling_campaign(const CampaignInputs& in, const CampaignModels& m) {
    CampaignResult r;
    if (in.demand_kg > in.depot_capacity_kg) {
        r.reason = "Depotvolumen reicht nicht: " + tonnes(in.demand_kg) + " gefordert, " +
                   tonnes(in.depot_capacity_kg) + " Tankvolumen";
        return r;
    }
    Depot depot;
    depot.propellant_kg = in.depot_initial_kg;
    r.fill.push_back({0.0, depot.propellant_kg});
    double last_launch_s = -in.launch_interval_s;  // the first tanker may launch at t = 0
    double last_arrival_s = -in.port_spacing_s;
    double previous_after_kg = -1.0;
    const auto finish = [&](const std::string& reason) {
        r.reason = reason;
        r.losses = depot.losses;
        r.tanker_flights = static_cast<int>(r.tankers.size());
        return r;
    };
    while (true) {
        if (depot.propellant_kg >= in.demand_kg) {
            // enough now: does it last until the HLS arrives?
            const auto hls = next_slot(m.next_hls, last_launch_s + in.launch_interval_s, last_arrival_s, in);
            if (!hls) return finish("Kein Startfenster für das HLS");
            Depot projected = depot;
            advance(projected, hls->arrival_s, in, m, nullptr);
            if (projected.propellant_kg >= in.demand_kg) {
                advance(depot, hls->arrival_s, in, m, &r.fill);
                r.closes = true;
                r.hls = hls;
                r.duration_s = hls->arrival_s;
                r.depot_at_hls_kg = depot.propellant_kg;
                return finish("");
            }
        }
        if (static_cast<int>(r.tankers.size()) >= in.max_tankers) {
            return finish("Kampagne schließt nicht nach " + std::to_string(in.max_tankers) + " Tankerflügen");
        }
        const auto slot = next_slot(m.next_tanker, last_launch_s + in.launch_interval_s, last_arrival_s, in);
        if (!slot) return finish("Kein Startfenster für Tanker " + std::to_string(r.tankers.size() + 1));
        advance(depot, slot->arrival_s, in, m, &r.fill);
        CampaignTanker t;
        t.flight = *slot;
        const double vent = std::max(0.0, slot->delivered_kg) * in.transfer_vent_fraction;
        const double offered = std::max(0.0, slot->delivered_kg) - vent;
        t.received_kg = std::min(offered, in.depot_capacity_kg - depot.propellant_kg);
        depot.losses.transfer_vent_kg += vent;
        depot.propellant_kg += t.received_kg;
        t.depot_after_kg = depot.propellant_kg;
        r.tankers.push_back(t);
        r.fill.push_back({depot.t_s, depot.propellant_kg});
        last_launch_s = slot->launch_s;
        last_arrival_s = slot->arrival_s;
        if (slot->delivered_kg <= 0.0) {
            return finish("Tanker liefert nichts: " + tonnes(slot->delivered_kg) + " nach Aufstieg, Phasenflug und Reserve");
        }
        if (previous_after_kg >= 0.0 && depot.propellant_kg <= previous_after_kg &&
            depot.propellant_kg < in.demand_kg) {
            return finish("Verluste übersteigen die Lieferung: Depot " + tonnes(depot.propellant_kg) + " nach Tanker " +
                          std::to_string(r.tankers.size()));
        }
        previous_after_kg = depot.propellant_kg;
    }
}

}  // namespace starsim::mission
```

Check the hand computation of the clamp test against this code before running (250 t after T3? T1 148... with
boil 0: T1 150, T2 250, T3 320 clamped, received 70 t) and of the "losses" case (interval 2 d, 60 t/day: T1 day 2:
50 - 120 -> 0 + 100 = 100; T2 day 4: 100 - 120 -> 0 + 100 = 100 <= 100 -> "Verluste").

- [ ] **Step 4: GREEN** - `-TestFilter "[campaign]"`.
- [ ] **Step 5: Commit + push** - `feat(mission): refuelling campaign calculator`.

### Task 5: Campaign setup from config

**Files:**
- Modify: `core/include/starsim/sim/refuelling_mission.hpp`, `core/src/sim/refuelling_mission.cpp` (expose
  `fill_main_tanks`; extract the window parameters)
- Create: `core/include/starsim/sim/refuelling_campaign_setup.hpp`, `core/src/sim/refuelling_campaign_setup.cpp`
  (CMake)
- Test: `core/tests/test_refuelling_campaign.cpp`

**Interfaces:**
- Consumes: Tasks 1-4; `mission::plan_launch_window`, `sim::depot_orbit`, `vehicle::orbit_average_boil_off`,
  `sim::place_lunar_landing_mission`, `sim::lunar_outbound_parameters`, `sim::lunar_landing_setup`,
  `gnc::groups_vacuum_isp_s`, `config::load_refuelling_flight_mission`, `config::load_ascent_mission`,
  `config::load_launch_site`.
- Produces (refuelling_mission.hpp):

```cpp
// Main tanks of `stage` at `kg`, split by their capacities (r5).
void fill_main_tanks(config::StageConfig& stage, double kg);
// The launch window parameters of a pad-start refuelling flight (as start_refuelling_flight_mission flies them).
mission::LaunchWindowParameters refuelling_window_parameters(const config::RefuellingFlightMission& mission,
                                                             const config::AscentMission& ascent);
```

- Produces (refuelling_campaign_setup.hpp):

```cpp
namespace starsim::sim {

// The calculator's levers (spec r6; std::nullopt: the reference flight's value).
struct CampaignLevers {
    std::optional<double> depot_height_m;      // circular; reference: tanker_flight's [depot]
    std::optional<double> tanker_dry_mass_kg;  // reference: the measured tanker
    std::optional<double> tanker_reserve_kg;   // main tanks kept for the return; reference: main at undock
    std::optional<double> phasing_limit_s;     // tanker and HLS; reference: the missions' limits (48 h)
    std::optional<double> depot_initial_kg;    // reference: depot_launch's residue
    double depot_insulation_scale = 1.0;       // x the depot's wall resistance
    double launch_interval_s = 86400.0;        // [E] pad turnaround
    double hls_wait_s = 0.0;                   // HLS docked -> TLI (its boil-off is demanded)
};

struct CampaignSources {
    const ephemeris::Ephemeris& ephemeris;
    const frames::BodyOrientation& earth;
    const frames::BodyOrientation& moon;
    const time::TimeScaleConverter& time;
    std::filesystem::path config_dir;
};

struct CampaignReport {
    mission::CampaignResult result;
    time::Epoch start;  // t = 0: the tanker mission's earliest_utc
    std::string lunar_mission_id;
    time::Epoch lunar_epoch;
    double depot_height_m = 0.0, depot_capacity_kg = 0.0;
    double hls_required_mass_kg = 0.0, hls_dry_mass_kg = 0.0;
    double hls_propellant_at_docking_kg = 0.0, hls_capacity_kg = 0.0;
    double demand_kg = 0.0;
    double tanker_base_delivered_kg = 0.0;  // reference phasing time, levers applied
    double altitude_cost_kg = 0.0;          // per tanker (signed: < 0 below the reference)
    double reference_phasing_s = 0.0;
    double drag_makeup_kgps = 0.0;
};

// The campaign of the r5 reference flights with the levers applied (plan decisions 1-7). Refusals (lunar outbound
// plan, HLS tanks too small) come back as result.closes == false with a German reason; config errors throw.
CampaignReport plan_refuelling_campaign(const CampaignSources& sources, const CampaignLevers& levers);

}  // namespace starsim::sim
```

- [ ] **Step 1: Write the failing tests**

```cpp
#include <iostream>

#include "starsim/sim/refuelling_campaign_setup.hpp"
#include "starsim/spice/spice_ephemeris.hpp"     // the headers DepotSession includes (refuelling_test_support.hpp)
#include "starsim/spice/spice_orientation.hpp"
#include "starsim/spice/spice_time_converter.hpp"

namespace {
struct Sources {
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    const spice::SpiceTimeConverter clock;
    sim::CampaignSources get() const { return {ephemeris, earth, moon, clock, config_dir}; }
};

void print(const char* tag, const sim::CampaignReport& r) {
    std::printf("[r6] %s: %s, %d tankers, %.1f d, demand %.1f t (HLS needs %.1f t, docks with %.1f t), base %.1f t, "
                "altitude %.2f t, losses boil %.1f / drag %.2f / gas %.2f / vent %.1f t%s%s\n",
                tag, r.result.closes ? "closes" : "open", r.result.tanker_flights, r.result.duration_s / 86400.0,
                r.demand_kg / 1e3, r.hls_required_mass_kg / 1e3, r.hls_propellant_at_docking_kg / 1e3,
                r.tanker_base_delivered_kg / 1e3, r.altitude_cost_kg / 1e3, r.result.losses.boil_off_kg / 1e3,
                r.result.losses.drag_makeup_kg / 1e3, r.result.losses.attitude_gas_kg / 1e3,
                r.result.losses.transfer_vent_kg / 1e3, r.result.reason.empty() ? "" : " - ",
                r.result.reason.c_str());
}
}  // namespace

TEST_CASE("Reference campaign: the flown windows, a plausible tanker count", "[refuelling][campaign][slow]") {
    const Sources s;
    const auto r = sim::plan_refuelling_campaign(s.get(), {});
    print("reference", r);
    REQUIRE(r.result.closes);
    // the first tanker flies the r5 window (79.8 h after the earliest, phasing ~39.7 h)
    REQUIRE_FALSE(r.result.tankers.empty());
    CHECK(r.result.tankers.front().flight.launch_s / 3600.0 == Catch::Approx(79.8).margin(0.2));
    CHECK(r.reference_phasing_s / 3600.0 == Catch::Approx(39.7).margin(1.5));
    CHECK(r.altitude_cost_kg == Catch::Approx(0.0).margin(1e-6));
    CHECK(r.tanker_base_delivered_kg == Catch::Approx(67.5e3).margin(1.0));
    // the HLS needs less than full tanks, more than it docks with
    CHECK(r.hls_required_mass_kg - r.hls_dry_mass_kg < r.hls_capacity_kg);
    CHECK(r.demand_kg > 0.0);
    // plausibility only (public 4 to high teens; not tuned): a two-digit count is expected from ~1,400 t / ~66 t
    CHECK(r.result.tanker_flights >= 4);
    CHECK(r.result.tanker_flights <= 40);
    for (size_t k = 1; k < r.result.tankers.size(); ++k) {
        CHECK(r.result.tankers[k].flight.arrival_s >= r.result.tankers[k - 1].flight.arrival_s + 6.0 * 3600.0);
    }
}

TEST_CASE("Campaign levers move the count the physical way", "[refuelling][campaign][slow]") {
    const Sources s;
    const auto base = sim::plan_refuelling_campaign(s.get(), {});
    sim::CampaignLevers higher;
    higher.depot_height_m = 300.0e3;
    const auto h = sim::plan_refuelling_campaign(s.get(), higher);
    print("300 km", h);
    CHECK(h.altitude_cost_kg > 0.0);
    CHECK(h.drag_makeup_kgps < base.drag_makeup_kgps);
    sim::CampaignLevers lighter;
    lighter.tanker_dry_mass_kg = 199.0e3;
    const auto l = sim::plan_refuelling_campaign(s.get(), lighter);
    print("dry -10 t", l);
    CHECK(l.tanker_base_delivered_kg == Catch::Approx(base.tanker_base_delivered_kg + 9.9e3).margin(1.0));
    CHECK(l.result.tanker_flights <= base.result.tanker_flights);
    sim::CampaignLevers poor;
    poor.depot_insulation_scale = 0.1;
    const auto p = sim::plan_refuelling_campaign(s.get(), poor);
    print("insulation x0.1", p);
    CHECK(p.result.losses.boil_off_kg > base.result.losses.boil_off_kg);
}

TEST_CASE("Campaign setup refusals", "[refuelling][campaign]") {
    const Sources s;
    sim::CampaignLevers greedy;
    greedy.tanker_reserve_kg = 200.0e3;  // keeps more than it carries
    const auto r = sim::plan_refuelling_campaign(s.get(), greedy);
    CHECK_FALSE(r.result.closes);
    CHECK(r.result.reason.find("liefert nichts") != std::string::npos);
}
```

(The spice header names: copy the includes of `core/tests/refuelling_test_support.hpp`. If the refusal case takes
> 40 s - it searches the HLS demand and one tanker window - tag it `[slow]`.)

- [ ] **Step 2: RED** - `-TestFilter "[campaign]"`: header missing.

- [ ] **Step 3: Implement**

`refuelling_mission.cpp`: move `fill_main_tanks` out of the anonymous namespace (declared in the header), and
extract the Pad branch's parameter block into:

```cpp
mission::LaunchWindowParameters refuelling_window_parameters(const config::RefuellingFlightMission& m,
                                                             const config::AscentMission& ascent) {
    const double phasing_periapsis_m = m.phasing_periapsis_height_m.value_or(ascent.periapsis_height_m);
    mission::LaunchWindowParameters p;
    p.inclination_rad = ascent.inclination_rad;
    p.northerly = ascent.northerly;
    p.insertion_s = m.window.insertion_s;
    p.insertion_arc_rad = m.window.insertion_arc_rad;
    p.separation_periapsis_radius_m = equatorial_radius_m + phasing_periapsis_m;  // the phasing after the raise
    p.separation_apoapsis_radius_m = equatorial_radius_m + ascent.apoapsis_height_m;
    p.search_s = m.window.search_s;
    p.phasing.mission_limit_s = m.mission_limit_s - m.window.insertion_s;  // from the orbit insertion
    p.phasing.earliest_arrival_s = std::max(0.0, m.earliest_arrival_s - m.window.insertion_s);
    return p;
}
```

and use it in `start_refuelling_flight_mission` (behaviour unchanged - the r5 mission tests guard it).

`refuelling_campaign_setup.cpp` - the pieces (write them as small file-local functions):

```cpp
namespace {

constexpr double deorbit_perigee_height_m = 50.0e3;  // [E] the return's entry perigee; only differences count
constexpr const char* raptor_group = "sea_level_3";  // the single Raptor of phasing, deorbit and reboost (r3, r5)

double exhaust_velocity_mps(const vehicle::Vehicle& v) {
    return gnc::groups_vacuum_isp_s(v, {raptor_group}) * constants::standard_gravity_mps2;
}

// The vehicle's top stage alone with its main tanks at `main_kg`.
config::VehicleConfig top_stage(const std::filesystem::path& dir, const std::string& id, double main_kg) {
    auto cfg = config::top_stage_config(config::load_vehicle(dir, id));
    fill_main_tanks(cfg.stages.back(), main_kg);
    return cfg;
}

void set_main(vehicle::Vehicle& v, double kg) {  // split by the main tanks' capacities (as fill_main_tanks)
    double capacity = 0.0;
    for (const auto& t : v.stages().back().tanks) {
        if (t.config.role == config::TankRole::Main) capacity += t.config.capacity_kg;
    }
    const size_t stage = v.stages().size() - 1;
    for (const auto& t : std::vector(v.stages().back().tanks)) {
        if (t.config.role == config::TankRole::Main) v.set_tank_mass(stage, t.config.id, kg * t.config.capacity_kg / capacity);
    }
}

double orbit_boil_off_kgps(vehicle::Vehicle& v, double main_kg, const math::StateVector& orbit,
                           const math::Vec3& sun_gcrf_m) {
    set_main(v, main_kg);
    const auto rate = vehicle::orbit_average_boil_off(v, orbit, {}, sun_gcrf_m);
    return rate.oxidizer_kgps + rate.fuel_kgps;
}

}  // namespace
```

`plan_refuelling_campaign` in order:
1. Load `config::load_refuelling_reference`, the tanker mission (`reference.tanker.mission`), the HLS mission
   (`reference.hls.mission`), their ascents and launch site; `start = time.from_utc(tanker_mission.earliest_utc)`.
2. Depot: `depot_cfg = tanker_mission.depot` with `height_m = levers.depot_height_m.value_or(...)`;
   `depot_at = depot_orbit(depot_cfg, start, earth)`; `dh = depot_cfg.height_m - tanker_mission.depot.height_m`.
   Depot vehicle `top_stage(dir, depot_cfg.vehicle_id, 0)` with `thermal->hull.wall_resistance_m2k_w *=
   levers.depot_insulation_scale` before building; capacity = sum of its main tanks' `capacity_kg`.
3. Drag: `r = equatorial_radius_m + depot height`, `rho = environment::Us76Atmosphere{}.at(height).density_kg_m3`,
   `v = sqrt(gm / r)`, area `pi (diameter_m / 2)^2` of the depot stage, `cd = FlightSession::dragon_free_molecular_cd`,
   `drag_makeup_kgps(rho, v, cd, area, exhaust_velocity_mps(depot))`.
4. Tanker: vehicle from `ascent.vehicle_id` top stage with `dry_mass_kg = levers.tanker_dry_mass_kg.value_or(ref)`;
   `reserve = levers.tanker_reserve_kg.value_or(ref.main_at_undock_kg)`;
   `base = delivered_ox + delivered_fuel + (ref.dry - dry) * ref.dry_mass_sensitivity + (ref.main_at_undock - reserve)`.
   Altitude: `m_docked` = tanker mass with main = residue - rendezvous; `m_return` = mass with main = reserve;
   `sign = dh >= 0 ? 1 : -1`; `dv_up = sign * hohmann_delta_v_mps(r_ref, r_new, gm)`;
   `dv_back = deorbit_delta_v_mps(r_new, rp) - deorbit_delta_v_mps(r_ref, rp)`;
   `altitude_cost = m_docked * (exp(dv_up / ve) - 1) + m_return * (exp(dv_back / ve) - 1) * TankerReserveParameters{}.burn_margin`;
   `base -= altitude_cost`.
5. Windows: `p = refuelling_window_parameters(tanker_mission, tanker_ascent)`, separation radii `+= dh`,
   `p.phasing.mission_limit_s = levers.phasing_limit_s.value_or(tanker_mission.mission_limit_s) - insertion_s`.
   Reference phasing: the window from `start` with the unshifted reference parameters and depot →
   `reference_phasing_s = waypoint0_epoch - separation`. `next_tanker(e)`: `plan_launch_window(pad, earth, depot_at,
   start + e, p, gm)`; `nullopt` if not feasible; `phasing = waypoint0 - separation`; tanker boil-off rate
   `orbit_boil_off_kgps(tanker, ref.residue_after_ascent_kg, depot_at(separation), sun(separation))`;
   `delivered = base + rate * (reference_phasing_s - phasing)`; launch/arrival relative to `start`.
   `next_hls(e)`: the same with the HLS mission's parameters (shifted by `dh`, same phasing limit lever).
6. HLS demand: `lunar = load_lunar_landing_mission(dir, hls_mission.lunar_mission_id)`,
   `lunar_epoch = time.from_utc(lunar.epoch_utc)`; scratch `FlightSession session(ephemeris, earth, dir,
   environment::PointMassGravityConfig{}, &moon)`; HLS config = top stage of the HLS ascent's vehicle, full (as
   configured); `place_lunar_landing_mission(session, lunar, dir, lunar_epoch, &hls_cfg)` (catch
   `std::runtime_error` -> reason); `session.plan_lunar_outbound(lunar_outbound_parameters(lunar, session.gravity(),
   session.ephemeris(), lunar_epoch))` (false -> reason `session.lunar_status()`); legs by `lunar_mission_legs` with
   the Isp's `plan_from_live_state` uses (`lunar_landing_setup(lunar).guidance.braking_sets` front / back);
   `dry = mass - propellant_kg()`; `required = lunar_budget_required_mass(dry, lunar.reserve_kg, legs)`;
   `capacity = propellant_kg()` of the full HLS. `required - dry > capacity` -> reason "HLS-Tanks zu klein: <x> t
   gefordert, <y> t Tankvolumen". At docking: HLS vehicle with main = `ref.hls.main_at_docking_kg`, propellant
   `p_dock = propellant_kg()` (headers included). Wait: `orbit_boil_off_kgps(hls_full, required - dry - headers,
   depot_at(start), sun(start)) * levers.hls_wait_s`. `demand = max(0, (required - dry + wait - p_dock)) * (1 + vent) +
   ref.tanker.chilldown_kg`.
7. `mission::CampaignInputs`: capacity, `depot_initial_kg = levers.depot_initial_kg.value_or(ref.depot.initial)`,
   demand, `launch_interval_s`; models: `boil_off_kgps(t, fill)` = `orbit_boil_off_kgps(depot, fill,
   depot_at(start + t), sun(start + t))`, drag constant. Sun: `ephemeris.state_gcrf(ephemeris::Body::Sun,
   epoch).position_m` (check the StateVector member name).
8. Fill the report; return.

- [ ] **Step 4: GREEN** - `-TestFilter "[campaign],[refuelling]"` (the window refactor touches the r5 starts; their
  `[slow]` campaigns included). Record the printed `[r6]` lines in the ledger: the reference tanker count is the
  milestone's answer.

- [ ] **Step 5: Commit + push** - `feat(sim): refuelling campaign from the reference flights - real windows, boil-off, drag, lunar demand`.

### Task 6: The flights check the measured-numbers file

**Files:**
- Modify: `core/tests/test_refuelling_missions.cpp` (cases "depot_launch: pad -> orbit ...", "tanker_flight: pad ->
  booster caught -> docked -> transfer -> ship caught", "hls_refuel: pad -> docked -> filled -> lunar mission planned")
- Modify: `core/tests/refuelling_measurements.hpp` (a check helper)

**Interfaces:**
- Consumes: `config::load_refuelling_reference` (Task 2), `refuelling_test::tanker_numbers` (r5).
- Produces: `inline void check_reference(double flown_kg, double file_kg, const char* what);` in
  `refuelling_measurements.hpp`.

- [ ] **Step 1: Write the checks**

`refuelling_measurements.hpp`:

```cpp
#include <catch2/catch_test_macros.hpp>

// Refuelling r6: a flown number against config/refuelling/reference_flights.toml - 1 % or 0.3 t, the larger.
inline void check_reference(double flown_kg, double file_kg, const char* what) {
    INFO(what << ": flown " << flown_kg / 1e3 << " t, file " << file_kg / 1e3 << " t (re-measure the file)");
    CHECK(std::abs(flown_kg - file_kg) <= std::max(0.01 * std::abs(file_kg), 300.0));
}
```

In the tanker pad campaign, after `print("pad", n)`:

```cpp
const auto ref = config::load_refuelling_reference(config_dir).tanker;
check_reference(n.residue_after_ascent_kg, ref.residue_after_ascent_kg, "residue after ascent");
check_reference(n.rendezvous_kg, ref.rendezvous_kg, "rendezvous");
check_reference(n.delivered_oxidizer_kg, ref.delivered_oxidizer_kg, "delivered LOX");
check_reference(n.delivered_fuel_kg, ref.delivered_fuel_kg, "delivered CH4");
check_reference(n.chilldown_kg, ref.chilldown_kg, "chill");
check_reference(n.settling_gas_kg, ref.settling_gas_kg, "settling gas");
check_reference(n.main_at_undock_kg, ref.main_at_undock_kg, "main at undock");
check_reference(n.main_at_catch_kg, ref.main_at_catch_kg, "main at catch");
```

depot_launch: `check_reference(<the initial fill the case already reads>, ref.depot.initial_propellant_kg, "depot
initial fill")`. hls_refuel pad: `check_reference(<main at the Docked mark>, ref.hls.main_at_docking_kg, "HLS main at
docking")` (`mark(s, IssMissionLeg::Docked)->chaser_main_propellant_kg`).

- [ ] **Step 2: Run** - `-TestFilter "[refuelling][mission]"` with the `[slow]` campaigns (background, output to a
  file, checked every 10 min per CLAUDE.md). Expected: PASS. A failing check means the r5 numbers in the docs and the
  file differ from the flight: write the flown values into the file (and `docs/subsystems.md`), never widen the
  tolerance.

- [ ] **Step 3: Commit + push** - `test(refuelling): the reference flights check the measured-numbers file`.

### Task 7: refuelling_report CLI

**Files:**
- Create: `core/include/starsim/sim/refuelling_campaign_text.hpp`, `core/src/sim/refuelling_campaign_text.cpp`
- Create: `core/apps/refuelling_report.cpp`; Modify: `core/CMakeLists.txt` (executable, compile definitions as the
  tests' `STARSIM_KERNEL_DIR` / `STARSIM_DATA_DIR` / `STARSIM_CONFIG_DIR`, whichever the SPICE adapters need - read
  how `bindings/src/flight_sim.cpp` `setup()` loads the kernels)
- Modify: `tools/build.ps1` only if the new target is not built by `cmake --build` (it is: same project)
- Test: `core/tests/test_refuelling_campaign.cpp`

**Interfaces:**
- Produces:

```cpp
namespace starsim::sim {
// Command-line levers: --height-km, --interval-h, --phasing-limit-h, --tanker-dry-t, --reserve-t, --initial-t,
// --insulation, --hls-wait-d. Throws std::invalid_argument naming the flag on unknown flags or bad numbers.
CampaignLevers campaign_levers_from_args(const std::vector<std::string>& args);
// The report as text: inputs, per-tanker table (launch / arrival in days and UTC, delivered, depot after), losses,
// the answer line "Tanker flights: N" or the reason.
std::string campaign_report_text(const CampaignReport& report, const time::TimeScaleConverter& time);
}
```

- [ ] **Step 1: Failing tests**

```cpp
#include "starsim/sim/refuelling_campaign_text.hpp"

TEST_CASE("refuelling_report: levers from the command line", "[refuelling][campaign]") {
    const auto l = sim::campaign_levers_from_args({"--height-km", "300", "--tanker-dry-t", "199", "--interval-h", "12",
                                                   "--insulation", "0.5", "--hls-wait-d", "2"});
    CHECK(*l.depot_height_m == Catch::Approx(300e3));
    CHECK(*l.tanker_dry_mass_kg == Catch::Approx(199e3));
    CHECK(l.launch_interval_s == Catch::Approx(12 * 3600.0));
    CHECK(l.depot_insulation_scale == Catch::Approx(0.5));
    CHECK(l.hls_wait_s == Catch::Approx(2 * 86400.0));
    CHECK_FALSE(l.tanker_reserve_kg.has_value());
    CHECK_THROWS_AS(sim::campaign_levers_from_args({"--height"}), std::invalid_argument);
    CHECK_THROWS_AS(sim::campaign_levers_from_args({"--height-km", "abc"}), std::invalid_argument);
    CHECK_THROWS_AS(sim::campaign_levers_from_args({"--insulation", "-1"}), std::invalid_argument);
}

TEST_CASE("refuelling_report: the text names the answer and the reason", "[refuelling][campaign]") {
    const spice::SpiceTimeConverter clock;
    sim::CampaignReport r;
    r.result.closes = true;
    r.result.tanker_flights = 12;
    r.result.tankers.resize(12);
    const auto text = sim::campaign_report_text(r, clock);
    CHECK(text.find("Tanker flights: 12") != std::string::npos);
    r.result.closes = false;
    r.result.reason = "Depotvolumen reicht nicht";
    CHECK(sim::campaign_report_text(r, clock).find("Depotvolumen reicht nicht") != std::string::npos);
}
```

- [ ] **Step 2: RED** - header missing.
- [ ] **Step 3: Implement** - parser: pairs of flag + value, `std::stod` with full-consumption check, values > 0
  except `--hls-wait-d` / `--reserve-t` >= 0; unit conversion (km -> m, h -> s, t -> kg, d -> s). Text: fixed-width
  `std::ostringstream`. `core/apps/refuelling_report.cpp`:

```cpp
// refuelling_report: how many tanker flights the lunar mission needs (refuelling r6). Usage:
//   refuelling_report [--height-km 200] [--interval-h 24] [--phasing-limit-h 48] [--tanker-dry-t 209]
//                     [--reserve-t 12.6] [--initial-t 55.5] [--insulation 1] [--hls-wait-d 0]
#include <iostream>

#include "starsim/sim/refuelling_campaign_setup.hpp"
#include "starsim/sim/refuelling_campaign_text.hpp"
// + the SPICE adapter headers, kernel loading as FlightSim::setup

int main(int argc, char** argv) {
    try {
        const auto levers = starsim::sim::campaign_levers_from_args({argv + 1, argv + argc});
        // construct ephemeris, Earth, Moon (moon_mean_earth), time converter as FlightSim does
        const auto report = starsim::sim::plan_refuelling_campaign({ephemeris, earth, moon, clock, STARSIM_CONFIG_DIR}, levers);
        std::cout << starsim::sim::campaign_report_text(report, clock);
        return report.result.closes ? 0 : 2;
    } catch (const std::exception& e) {
        std::cerr << "refuelling_report: " << e.what() << "\n";
        return 1;
    }
}
```

- [ ] **Step 4: GREEN** - `-TestFilter "[campaign]"` (the slow ones excluded only if unchanged since Task 5:
  `"[campaign]~[slow]"`); then run `build/.../refuelling_report.exe` once (path from `tools/build.ps1`'s build dir)
  and once with `--height-km 300`; paste both answers into the ledger.
- [ ] **Step 5: Commit + push** - `feat(tools): refuelling_report CLI - the campaign from the command line`.

### Task 8: FlightSim interface

**Files:**
- Create: `bindings/src/flight_snapshot_refuelling.cpp` (CMake of `bindings/`)
- Modify: `bindings/src/flight_snapshot.hpp/.cpp` (two Dictionaries), `bindings/src/flight_sim.hpp/.cpp`
- Modify: `game/tests/smoke_test.gd`
- Modify: `docs/ansichten.md` (the new getters / command for the user's UI)

**Interfaces:**
- Produces (GDScript):
  - `transfer_state() -> Dictionary`: `present` (false without a transfer), `phase` (German), `status`,
    `elapsed_s`, `delivered_oxidizer_kg`, `delivered_fuel_kg`, `flow_oxidizer_kgps`, `flow_fuel_kgps`,
    `settling_gas_kg`, `settling_thrust_n`, `chilldown_kg`, `boil_off_kg`.
  - `depot_state() -> Dictionary`: `present` (false without a station with transfer connectors), `main_propellant_kg`,
    `capacity_kg`, `fill_fraction`, `boil_off_kg_per_day` (the station body's `boil_off_rate()`), `docked`.
  - `refuelling_campaign(params: Dictionary) -> int` (command id); keys `height_km`, `interval_h`, `phasing_limit_h`,
    `tanker_dry_t`, `reserve_t`, `initial_t`, `insulation`, `hls_wait_d` (all optional floats); `result(id)["data"]`:
    `closes`, `reason`, `tanker_flights`, `duration_d`, `demand_t`, `hls_required_t`, `depot_at_hls_t`,
    `tankers` (Array of {`launch_utc`, `arrival_utc`, `delivered_t`, `depot_after_t`}), `fill` (Array of [day, t]),
    `losses` ({`boil_off_t`, `drag_t`, `gas_t`, `vent_t`}), `lunar_mission`, `lunar_epoch_utc`.
- `Outcome` gains `godot::Dictionary data;`; `result()` adds `d["data"]`.

- [ ] **Step 1: Smoke test first** - in `smoke_test.gd` add `_check_refuelling(config_dir, tdb)` (called from
  `_initialize` like `_check_iss`):

```gdscript
# Refuelling r6: the campaign command, transfer and depot getters.
func _check_refuelling(config_dir: String, tdb: float) -> bool:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	_check(not sim.transfer_state().get("present", true), "no transfer without a docked pair")
	_check(not sim.depot_state().get("present", true), "no depot before a refuelling mission")
	var id: int = sim.start_mission("tanker_flight_orbit", tdb)
	_check(sim.wait_idle(120.0) and sim.result(id).get("ok", false), "tanker_flight_orbit starts")
	var depot: Dictionary = sim.depot_state()
	_check(depot.get("present", false) and depot.get("main_propellant_kg", 0.0) > 0.0, "depot_state with its fill")
	var bad: int = sim.refuelling_campaign({"height_km": "x"})
	_check(sim.wait_idle(10.0) and not sim.result(bad).get("ok", true), "a wrongly typed lever is refused")
	var cid: int = sim.refuelling_campaign({"height_km": 200.0})
	_check(sim.wait_idle(600.0) and sim.result(cid).get("ok", false), "refuelling_campaign finishes")
	var data: Dictionary = sim.result(cid).get("data", {})
	_check(data.get("tanker_flights", 0) >= 4 and data.get("tankers", []).size() == data.get("tanker_flights", -1),
			"campaign result with its tankers")
	print("  refuelling campaign: %d tankers, %.1f d" % [data.get("tanker_flights", 0), data.get("duration_d", 0.0)])
	return true
```

- [ ] **Step 2: RED** - build with bindings (`tools/build.ps1 -TestFilter "[campaign]~[slow]"`), run the smoke test
  (CLAUDE.md command). Expected: GDScript errors - methods not found.

- [ ] **Step 3: Implement** - `build_transfer_state(session)` from `session.transfer_state()` (present when phase
  != Idle; phase names German via the existing `to_string` of `TransferPhase` if there is one, else a local switch);
  `build_depot_state(session)` from `station_index()`, the station body's main tanks (`main_propellant_kg()`, sum of
  main `capacity_kg`) and `boil_off_rate()` x 86400; both added to `FlightSnapshot` and filled in `build_snapshot`.
  `FlightSim::refuelling_campaign(const godot::Dictionary& params)`: convert the keys on the calling thread (unknown
  key or non-number -> post a command that returns `{false, "<German message>"}`), then `post("refuelling_campaign",
  ...)` whose lambda calls `sim::plan_refuelling_campaign({ephemeris_, earth_, moon_, time_, config_dir_}, levers)`
  (the FlightSim members, not the session: the session stays untouched) and stores the Dictionary in
  `Outcome::data`, `ok = report.result.closes`, `message = reason`. Document every key in `flight_sim.hpp` as the
  ISS getters do. Bind the three methods.

- [ ] **Step 4: GREEN** - smoke test PASSED; `-TestFilter "[campaign]~[slow]"` green.
- [ ] **Step 5: Docs + commit + push** - `docs/ansichten.md`: a short "Betankung (r6)" entry (the getters and the
  campaign command for the user's UI, no view built). `feat(bindings): refuelling campaign command, transfer and depot state`.

### Task 9: Documentation, final review, milestone close

**Files:**
- Modify: `docs/subsystems.md` ("Orbital refuelling": new "Campaign calculator (r6)" section - model, decisions,
  the reference answer and the lever results from the ledger, diagnostics, tests)
- Modify: `CLAUDE.md` (module summary: mission - refuelling campaign; config - reference flights; tools - CLI;
  Build & test: the `refuelling_report` usage line)
- Modify: `config/README.md` (Task 2 already), spec status line (r6 implemented, corrections)
- Modify: memory `project-orbital-refuelling.md` (milestone r COMPLETE, the answer)

- [ ] **Step 1:** write the docs with the numbers from the ledger (reference tanker count, duration, demand, loss
  breakdown, lever results); compare the count with the public range in one sentence (plausibility only).
- [ ] **Step 2:** run the affected tags once more: `-TestFilter "[campaign],[refuelling],[lunar]"` (background, file
  output, 10-minute checks) + smoke test.
- [ ] **Step 3:** commit + push `docs: refuelling r6 campaign calculator`.
- [ ] **Step 4:** final review (most capable model) over `ad55408..HEAD`; fix pass with RED -> GREEN per finding;
  ledger; push; tell the user milestone r is complete and ask for `/clear`.
