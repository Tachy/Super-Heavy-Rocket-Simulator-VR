# Refuelling f4 - Deferred Minors Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every deferred minor of the r1-r6, f1 and f3 final reviews is fixed or deliberately documented (with the
reason) in `docs/subsystems.md`.

**Architecture:** No new subsystem. Items are grouped by module and test tag so each task runs one focused test set:
vehicle/config, the report tool, `FlightBody` (boil-off, coast), burns/deorbit/settling, propellant transfer, the
mission chain, the rendezvous throttle model, lunar PDI, the campaign calculator, bindings, the step-dependence
investigation, docs + re-measure. Line numbers below are from the read-only audit of 2026-10-21 (HEAD ec3e6ab); verify
before editing.

**Tech Stack:** C++20 core (Catch2), Python tools (unittest), GDExtension bindings, Godot smoke test.

**Spec:** `docs/superpowers/specs/2026-10-21-refuelling-followups-design.md` (section f4).

## Global Constraints

- Every item is fixed or deliberately documented (with the reason) in `docs/subsystems.md` (spec f4).
- SI units, frame/unit in names, no magic numbers (new constants with provenance [S]/[E]/[D]).
- Code/comments English, UI/status strings German.
- Test workflow: only the tags a change affects (`-TestFilter`, `[slow]` cases of those tags included); no blanket
  full-suite runs. Runs > 10 min in the background with output in a file, checked every 10 min.
- Work on `main`, commit per task (Conventional Commits), push after each green task.
- Re-measure `config/refuelling/reference_flights.toml` and the campaign answer when a flown reference number moves.

## Review Focus

1. A NaN or duplicate id reaching config/vehicle code: must be a clear error, not a silent overwrite (Task 1 tests).
2. A body undocked after days of docking: its first coast must vent at the new rate, not an average with the
   pre-docking rate (Task 3 test).
3. A continuation that installs a new mission sequence: the new sequence must run, not end as Complete (Task 6 test).
4. A flown tanker whose phasing deadline now counts from the separation: the reference flights must still dock
   (Task 6/Task 12 slow re-runs; a refusal there is a finding for the user, not a tolerance bump).
5. `--reference-phasing` as the first, last or repeated CLI argument (Task 9 test).

---

## Item map (spec item -> task -> action)

| Item | Task | Action |
|------|------|--------|
| r1 hull sampling cut | 1 | FIX hardening (smallest x of all nose ports) |
| r1 mating nearest drogue | 1 | FIX test (one-to-one + negative layouts) |
| r1 set_tank_mass NaN | 1 | FIX throw |
| r1 duplicate ids | 1 | FIX config errors |
| r1 vehicle_report.py | 2 | FIX + unittest |
| r1 depot stale comments / header 35 t | 1, 12 | FIX comments, DOCUMENT header |
| r1 tanker z comment | 1 | FIX comment |
| r1 bindings mechanism/elements | 10 | FIX + smoke |
| r1 load_stage_from reload | 1 | DOCUMENT (ms once per load) |
| r2 resting boils | 3 | FIX |
| r2 abort reason failed engines | 4 | FIX |
| r2 eclipse time-warp test | 3 | ADD test (fix only if it fails) |
| r2 boil_off_last_ after holds | 3 | FIX |
| r2 DeorbitParameters::boil_off_kgps | 4 | DOCUMENT (offline knob) |
| r2 coast cost non-thermal | 3 | FIX (`has_thermal`) |
| r2 AU/solar pressure duplicates | 1 | FIX (constants) |
| r2 mixture ratio last engine | 4 | FIX |
| r2 plan.boil_off_kg on re-plan | 4 | FIX |
| r3 M1 rendezvous full throttle | 7 | FIX |
| r3 M3 'Bahnanhebung der ISS' | 6 | FIX text |
| r3 M4 station limits | 12 | DOCUMENT (safety limits) |
| r3 M5 wall->plate jump | 12 | DOCUMENT (mm at the rim) |
| r4 settling impulse scale | 5 | FIX |
| r4 dp per line | 5 | FIX |
| r4 start while hooks open | 5 | FIX |
| r4 re-chill vents nothing | 5 | FIX (vent again) |
| r5 collision in Transfer | 6 | FIX |
| r5 generation counter | 6 | FIX |
| r5 set_iss_mission docked | 6 | FIX |
| r5 [mission] port unused | 6 | FIX remove key |
| r5 fused comments | - | already fixed in f1 (18a0c63) |
| r5 nose_cone_body_ shift | 6 | FIX |
| r5 CW-drift ISS note | 12 | DOCUMENT |
| r5 tail-off controlled body | 12 | DOCUMENT |
| r6 HLS tank check + wait | 9 | FIX |
| r6 HLS altitude cost | 9 | FIX |
| r6 wait Sun at t=0 | 9 | FIX |
| r6 vent 1/(1-f) | 9 | FIX |
| r6 comment tag | 9 | FIX |
| r6 file-drift guard | 9 | FIX (inputs digest) |
| r6 worker blocked ~1 s | 12 | DOCUMENT (runner accepts long calls) |
| f1 M1 raise lowers periapsis | 6 | FIX |
| f1 M2 deadline from raise end | 6 | FIX |
| f1 M3 HLS extra one-sided | 9 | FIX |
| f1 M4 refusal reason | 9 | FIX |
| f1 M5 config edges | 9 | FIX |
| f1 M6 CLI wrap-around | 9 | FIX |
| f1 M7 tail-off timeout | 6 | FIX |
| f3 M7 settling flag source | 9 | FIX |
| f3 M8 coast orbit reference | 3 | FIX |
| f3 M9 PDI without hold | 8 | FIX |
| f3 M10 PDI push when settled | 8 | FIX |
| f3 M11 manual settle vs RCS burn | 4 | FIX |
| f3 M12 lunar ignitions count | 8 | FIX (`BudgetLeg::ignitions`) |
| f3 M13 untested hold/docked | 4 | ADD tests |
| step dependence ~8 m/s | 11 | INVESTIGATE time-boxed, fix or DOCUMENT |

---

### Task 1: Vehicle and config hygiene

**Files:**
- Modify: `core/src/vehicle/vehicle.cpp` (~408 `set_tank_mass`)
- Modify: `core/src/config/vehicle_config.cpp` (~220 `append_docking_ports`, ~245 `append_transfer_connectors`, ~731 `stages_by_id`)
- Modify: `core/src/contact/station_collision.cpp` (~20-22)
- Modify: `core/include/starsim/math/constants.hpp`, `core/src/vehicle/flight_body.cpp` (~27-28)
- Modify: `config/vehicles/starship_depot.toml` (comments ~44-47, 54, 66-69, 120, 125-126, 130), `config/vehicles/starship_tanker.toml` (~28)
- Test: `core/tests/test_refuelling_vehicles.cpp`, `core/tests/test_station_collision.cpp`

**Interfaces:**
- Produces: `constants::speed_of_light_mps`; `set_tank_mass` throws `std::invalid_argument` on non-finite mass.

- [ ] **Step 1: Failing tests** in `test_refuelling_vehicles.cpp`:

```cpp
// in "set_tank_mass sets one tank ... clamped" [refuelling][vehicle]
const double before = v.tank_mass_kg(0, "lox_main");
CHECK_THROWS_AS(v.set_tank_mass(0, "lox_main", std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
CHECK_THROWS_AS(v.set_tank_mass(0, "lox_main", std::numeric_limits<double>::infinity()), std::invalid_argument);
CHECK(v.tank_mass_kg(0, "lox_main") == before);
```

Add rows `dup_port`, `dup_connector`, `dup_stage` to "Bad probe/drogue ports and transfer connectors are config
errors" `[refuelling][config]` (same TOML-snippet pattern as the existing rows: a second `[[stage.docking_port]]` with
the same id; a second `[[stage.transfer_connector]]` with the same id; two `[[stage]]` with the same id), each
`CHECK_THROWS_WITH(..., ContainsSubstring("twice"))`.

Replace the "nearest drogue" checks (`test_refuelling_vehicles.cpp:271-277`, `test_refuelling_docking.cpp:288-295`)
with a one-to-one helper in a shared test header (`core/tests/refuelling_test_support.hpp` if it exists, else local):

```cpp
// Every probe meets a distinct drogue within tol_m; returns the problem or nullopt.
inline std::optional<std::string> probe_pairing_problem(const std::vector<math::Vec3>& probes_m,
                                                        const std::vector<math::Vec3>& drogues_m, double tol_m) {
    std::set<std::size_t> used;
    for (const auto& p : probes_m) {
        std::optional<std::size_t> hit;
        for (std::size_t k = 0; k < drogues_m.size(); ++k)
            if (norm(p - drogues_m[k]) < tol_m) hit = k;
        if (!hit) return "probe without drogue";
        if (!used.insert(*hit).second) return "two probes in one drogue";
    }
    return std::nullopt;
}
```

Negative cases: drogue 0 moved by 1 m -> "probe without drogue"; drogue 1 set equal to drogue 0 -> problem.

In `test_station_collision.cpp` `[iss][docking]`: a synthetic stage with a dorsal port first and a nose port
(axis +x) second; the maximum hull sample x stays below the nose port x.

- [ ] **Step 2: Run, expect FAIL** - `tools/build.ps1 -NoBindings -TestFilter "[refuelling][vehicle],[refuelling][config],[iss][docking]"` (output to `tmp/f4_t1_red.txt`).

- [ ] **Step 3: Implement**

```cpp
// vehicle.cpp, set_tank_mass, before the clamp
if (!std::isfinite(mass_kg))
    throw std::invalid_argument("Tankmasse '" + std::string(tank_id) + "' ist keine Zahl");
```

```cpp
// vehicle_config.cpp
const std::string id = stage.id;
if (!stages_by_id.emplace(id, std::move(stage)).second) doc.fail("stage '" + id + "' defined twice");
// append_docking_ports / append_transfer_connectors, before push_back:
if (std::ranges::any_of(s.docking_ports, [&](const auto& q) { return q.id == port.id; }))
    doc.fail("docking port id '" + port.id + "' twice");
```

```cpp
// station_collision.cpp: cut at the smallest x of all nose ports (axis.x > 0.5), not front()
std::optional<double> cut_x_m;
for (const auto& p : stage.docking_ports)
    if (p.axis.x > 0.5) cut_x_m = cut_x_m ? std::min(*cut_x_m, p.position_m.x) : p.position_m.x;
```

```cpp
// constants.hpp
inline constexpr double speed_of_light_mps = 299792458.0;  // [S] SI definition
// flight_body.cpp: drop the locals, use
// constants::solar_constant_w_m2 / constants::speed_of_light_mps and constants::astronomical_unit_m
```

Comments: depot TOML Block 3 remarks -> "kept from Block 3 for the hull geometry; the depot never enters or lands";
tanker `:28` -> "the interface plane z = +5.0 m lies 0.5 m off the dorsal hull (radius 4.5 m; belly at -z)".

- [ ] **Step 4: Run, expect PASS** - same filter plus `[boil_off],[lunar]` for the SRP constant (bit-identical) -> `tmp/f4_t1.txt`.

- [ ] **Step 5: Commit + push** - `fix(vehicle): f4 config and vehicle hygiene (NaN, duplicate ids, nose-port cut, constants)`.

---

### Task 2: vehicle_report.py from-resolution like the C++ loader

**Files:**
- Modify: `tools/vehicle_report.py` (~29-39)
- Create: `tools/test_vehicle_report.py` (unittest, pattern of `tools/data_pipeline/test_iss_oem.py`)

- [ ] **Step 1: Failing tests** with a temp config dir (pass `config_dir` into `resolve_stages`):

```python
class FromResolution(unittest.TestCase):
    def test_cycle_is_refused(self): ...        # a -> b:s, b -> a:s  => SystemExit("from chain too long")
    def test_three_part_reference_refused(self): ...  # from = "a:b:c" => SystemExit
    def test_forbidden_key_refused(self): ...   # from-stage with "isp" => SystemExit
    def test_lists_are_appended(self): ...      # source 1 docking_port + added 1 => 2
    def test_depot_dry_mass_matches_cpp(self):  # real config: starship_tanker stage dry mass == 209000 kg [from TOML]
        ...
```

- [ ] **Step 2: Run, expect FAIL** - `python -m unittest tools/test_vehicle_report.py`.

- [ ] **Step 3: Implement**

```python
FROM_KEYS = {"docking_port", "transfer_connector", "rcs_pod", "propellant_transfer", "dry_mass_kg", "id", "from"}
LIST_KEYS = {"docking_port", "transfer_connector", "rcs_pod"}
MAX_FROM_DEPTH = 4  # same bound as the C++ loader

def resolve_stages(vehicle, config_dir=CONFIG_DIR, depth=0):
    if depth > MAX_FROM_DEPTH:
        raise SystemExit("from chain too long (cycle?)")
    stages = []
    for stage in vehicle["stage"]:
        if "from" not in stage:
            stages.append(stage); continue
        source_id, sep, stage_id = stage["from"].partition(":")
        if not sep or not source_id or not stage_id or ":" in stage_id:
            raise SystemExit(f"bad from '{stage['from']}'")
        bad = set(stage) - FROM_KEYS
        if bad:
            raise SystemExit(f"from-stage may not override {sorted(bad)}")
        source = load_vehicle(source_id, config_dir)
        base = next(s for s in resolve_stages(source, config_dir, depth + 1) if s["id"] == stage_id)
        merged = dict(base)
        for k, v in stage.items():
            if k == "from": continue
            merged[k] = base.get(k, []) + v if k in LIST_KEYS else v
        stages.append(merged)
    return stages
```

(Check the C++ `vehicle_config.cpp:674-679` for the exact allowed keys and depth bound and mirror them.)

- [ ] **Step 4: Run, expect PASS**; `python tools/vehicle_report.py starship_tanker` output unchanged (diff against a run before the change).

- [ ] **Step 5: Commit + push** - `fix(tools): vehicle_report resolves from-stages like the C++ loader`.

---

### Task 3: FlightBody boil-off and coast

**Files:**
- Modify: `core/src/vehicle/flight_body.cpp` (~363-368 `boil_off`, ~398-435 `step`, ~495, ~606-648 `coast`), `core/include/starsim/vehicle/flight_body.hpp`
- Modify: `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp` (`has_thermal`)
- Test: `core/tests/test_boil_off.cpp`, `core/tests/test_ignition_settling.cpp`

**Interfaces:**
- Produces: `bool Vehicle::has_thermal() const`; `void FlightBody::forget_boil_off()` (clears `boil_off_elapsed_s_` and `boil_off_last_`, keeps `boil_off_rate_`).

- [ ] **Step 1: Failing tests** (`[boil_off][flight]` unless noted):
  1. Resting on the Moon: tanker resting (lunar env as in `test_lunar_flight_body.cpp`), `step` 20 s -> main propellant unchanged.
  2. Holds forget: coast 600 s, `hold_docked()`, `set_tank_mass` halves LOX, `release_docked()`, coast 600 s -> vented mass equals `boil_off_rate().total` after release x 600 within 1e-3 relative.
  3. Eclipse at time warp: Sun in the orbit plane, 5 orbits in `FlightSession::max_coast_s` segments vs `orbit_average_boil_off` x time, relative 0.03.
  4. Non-thermal: Dragon with the flag on -> `boil_off_rate().total == 0`.
  5. `[ignition_settling][vehicle]`: one coast crossing the Moon's sphere of influence -> `liquid_now().gm_m3ps2 == constants::gm_moon...` and `in_orbit()` w.r.t. the Moon.

- [ ] **Step 2: Run, expect FAIL** (3 may pass - then it stays as a guard) -> `tmp/f4_t3_red.txt`, filter `[boil_off],[ignition_settling]`.

- [ ] **Step 3: Implement**

```cpp
// step(): remember the resting state before it is cleared
const bool was_resting = resting_;
resting_ = false;
...
if (was_resting) forget_boil_off(); else boil_off(env, epoch);
```

```cpp
// hold_docked / release_docked / hold_fixed / release_landed: call forget_boil_off()
// boil_off(): if (!vehicle_->has_thermal()) return;
// coast(): if ((env.boil_off && env.ephemeris && vehicle_->has_thermal()) || env.ignition_settling) evaluate ...
// coast(): determine the end reference first
reference_ = reference_body(env, epoch + duration_s);
const auto& end_body = ...;  // as computed today after track_reference
track_reference(end_body, end_body.frame(epoch + duration_s));
// then the settling update, ground contact, rest unchanged
```

- [ ] **Step 4: Run, expect PASS** - `[boil_off],[ignition_settling],[lunar],[hls],[refuelling][docking],[prop_transfer]` (slow included; > 10 min -> background, `tmp/f4_t3.txt`). Moved r5 numbers go to Task 12's re-measure.

- [ ] **Step 5: Commit + push** - `fix(vehicle): boil-off forgets holds, resting bodies keep their propellant, coast ends on the end body`.

---

### Task 4: Burns, deorbit and manual settling

**Files:**
- Modify: `core/src/sim/flight_session.cpp` (~352-361 dry/failed reason, ~862-867), `core/src/sim/flight_session_burns.cpp` (~182-186 `settle`, `execute_burn`), `core/src/sim/flight_session_return.cpp` (~101-102 mixture ratio, ~225-239 re-plan), `core/include/starsim/gnc/deorbit_planner.hpp` (~35-37 comment)
- Test: `core/tests/test_ship_return_mission.cpp`, `core/tests/test_ignition_settling.cpp`

- [ ] **Step 1: Failing tests**
  - `[ship][session][deorbit][boil_off]`: `fail_engine` on all `sea_level_3` with full tanks during the deorbit burn -> `burn_abort_reason()` contains "ausgefallen".
  - Same tags: plan a return, advance past the re-plan -> `ship_return_plan()->boil_off_kg > 0` and below the commit value.
  - `[ignition_settling][sim]` "Manual settle: an RCS burn refuses it": RCS-chosen small burn running -> `settle(true)` false, burn reaches its dv.
  - `[ignition_settling][sim]` "Manual settle: refused while docked" (`refuelling_test::TankerAtPort` docked) -> `settle(true)` false.
  - `[ignition_settling][sim]` "Settling: a hold that never settles lights after the factor and ingests": clear `settled` each step via the test access -> `burn_abort_reason()` "Gasansaugung", ignition >= `settling_hold_factor` x lead after the push start.

- [ ] **Step 2: Run, expect FAIL** -> `tmp/f4_t4_red.txt`, filter `[ship][session][deorbit],[ignition_settling]`.

- [ ] **Step 3: Implement**

```cpp
// flight_session.cpp, dry check: failed engines get their own reason (gas ingestion keeps its reason, checked first)
const bool failed = std::ranges::any_of(engines_in_groups, [](const auto& e) { return e.failed; });
burn_abort_reason_ = failed ? "Brennvorgang abgebrochen: Triebwerke ausgefallen"
                            : "Brennvorgang abgebrochen: kein Treibstoff für die Triebwerke";
```

```cpp
// settle(): refuse while an autopilot RCS burn or the rendezvous runs
if (on && ((autopilot_.state == AutopilotState::Burn && burn_config_.rcs) ||
           autopilot_.state == AutopilotState::Rendezvous)) return false;
// execute_burn with chosen->rcs: settle_manual_ = false;
```

```cpp
// flight_session_return.cpp: mixture ratio of the deorbit engine group, not the last engine
const auto& groups = burn_configurations(&stage.deorbit_engine_groups).front().groups;
for (const auto& e : stage.engines)
    if (std::ranges::find(groups, e.group_id) != groups.end()) { mixture_ratio = e.performance->config().mixture_ratio; break; }
// re-plan: keep the flown copy's boil-off
deorbit_plan_ = flown;                       // success path
// keep-old-plan path: deorbit_plan_->boil_off_kg = flown.boil_off_kg;
```

`deorbit_planner.hpp`: comment `boil_off_kgps` "offline planning knob; FlightSession leaves it 0 and checks boil-off by
flying a copy ahead (boil_off_shortfall)".

- [ ] **Step 4: Run, expect PASS** - `[ship],[deorbit],[ignition_settling],[rendezvous]` (slow included, background if long) -> `tmp/f4_t4.txt`.

- [ ] **Step 5: Commit + push** - `fix(sim): failed-engine abort reason, deorbit re-plan boil-off, manual settle vs RCS burns`.

---

### Task 5: Propellant transfer minors

**Files:**
- Modify: `core/include/starsim/vehicle/propellant_transfer.hpp` (~33 `TransferLine`), `core/src/vehicle/propellant_transfer.cpp` (~104-136), `core/src/sim/flight_session_transfer.cpp` (~118-125, ~144, ~186-213, ~248)
- Test: `core/tests/test_refuelling_transfer.cpp`

**Interfaces:**
- Produces: `TransferLine::pressure_difference_pa` (per line); `TransferStepInput` loses `pressure_difference_pa`.

- [ ] **Step 1: Failing tests** (`[refuelling][prop_transfer]`, `[docking]` where a session is needed):
  - Two lines with different dp: each flow follows its own orifice law `Cd A sqrt(2 rho dp)`.
  - Pause in Chilldown, resume: the second chill vents `chilldown_kg` again (total 2x).
  - `REQUIRE(undock())`, then `start_transfer` refused with "Transfer nicht möglich: Abkoppeln läuft".
  - Nearly empty tanker, floors 0: the compound's momentum gain from the settling push <= drawn x Isp x g0.

- [ ] **Step 2: Run, expect FAIL** -> `tmp/f4_t5_red.txt`, filter `[prop_transfer]`.

- [ ] **Step 3: Implement**

```cpp
// start_transfer, after the docked() check
if (docking_->mechanism().phase() != contact::DockingPhase::Docked)
    return refuse_transfer("Transfer nicht möglich: Abkoppeln läuft");
// line setup: lines[k].pressure_difference_pa = donor_pressure_pa(k) - receiver_pressure_pa(k);
// PropellantTransfer::step: use lines_[k].pressure_difference_pa
// resume into Chilldown: state_.chilldown_kg = {};  // the lines warmed during the pause: chill (and vent) again
// settling push: draw first, then apply the share actually drawn
const double share = gas_kg > 0.0 ? drawn_kg / gas_kg : 0.0;
apply_settling_impulse(impulse_ns * share);
add_settling(drawn_kg, transfer_thrust_n_ * share);
```

- [ ] **Step 4: Run, expect PASS** - `[prop_transfer],[refuelling][docking]` -> `tmp/f4_t5.txt`.

- [ ] **Step 5: Commit + push** - `fix(sim): transfer per-line pressure, re-chill vents, no start while undocking`.

---

### Task 6: Mission chain minors

**Files:**
- Modify: `core/src/sim/flight_session_iss_mission.cpp` (~66-67, ~128, ~152-153, ~227-229, ~275-283, ~345-355), `core/src/sim/flight_session.cpp` (~513-526 `erase_body`), `core/src/sim/flight_session_proximity.cpp` (~112), `core/include/starsim/sim/flight_session.hpp`
- Modify: `core/src/config/mission_config.cpp` (~454), `core/include/starsim/config/refuelling_mission_config.hpp` (~42), `config/missions/hls_refuel*.toml` (remove `[mission] port`)
- Test: `core/tests/test_refuelling_missions.cpp`, `core/tests/test_proximity_session.cpp`, `core/tests/test_refuelling_mission_config.cpp`

**Interfaces:**
- Produces: `std::optional<time::Epoch> FlightSession::rendezvous_deadline() const`; constant `raise_tail_off_timeout_s = 30.0` [E].

- [ ] **Step 1: Failing tests** (`[refuelling][iss_mission]` unless noted):
  - Continuation installs a new sequence (WaitContinue fixture ~:336, the callback calls `s.set_iss_mission(next)` and returns `""`) -> `leg() != Complete`, new sequence's first leg marked.
  - `set_iss_mission` right after a mission-free hard capture with the docking autopilot still active -> legs Docked -> Transfer, not Ended.
  - Raise with a candidate periapsis below the insertion periapsis -> flown periapsis >= insertion periapsis - 1 km.
  - After the raise: `rendezvous_deadline()` = separation epoch + limit (within 1 s).
  - Pilot throttle keeps an engine running after the raise burn -> mission ends after `raise_tail_off_timeout_s` with "Phasing-Bahn anheben: Triebwerk läuft weiter".
  - `[iss][proximity][session][validation]` (`test_proximity_session.cpp:265`): warning starts with "Achtung: Bahnanhebung der Station".
  - `[refuelling][config][mission]`: a `[mission] port` key is now an unknown-key config error.

- [ ] **Step 2: Run, expect FAIL** -> `tmp/f4_t6_red.txt`, filter `[refuelling][iss_mission],[iss][proximity],[refuelling][config]`.

- [ ] **Step 3: Implement**

```cpp
// set_iss_mission: generation counter, docked start through the Docked gate
++iss_generation_;
if (sequence.transfer && docked()) { iss_enter(IssMissionLeg::Docked, "Angedockt"); return; }
// WaitContinue
const auto generation = iss_generation_;
const std::string reason = continuation();
if (!iss_mission_ || iss_generation_ != generation) return;  // ended or replaced by the continuation
// Transfer leg: collision first
if (collision_impulse_ns()) return iss_end("Kollision");
// raise burn 2: raise only
node = gnc::raise_periapsis_at_apoapsis(state, epoch_, gm_earth_, to.periapsis_radius_m);
// raise start: raise_start_epoch_ = epoch_; raise_tail_off_since_.reset();
// before plan_rendezvous: the limit counts from the separation
auto phasing = iss_mission_->phasing;
const double used_s = epoch_ - *raise_start_epoch_;
phasing.mission_limit_s -= used_s;
phasing.earliest_arrival_s = std::max(0.0, phasing.earliest_arrival_s - used_s);
// tail-off wait
if (any_engine_producing_thrust()) {
    if (!raise_tail_off_since_) raise_tail_off_since_ = epoch_;
    if (epoch_ - *raise_tail_off_since_ > raise_tail_off_timeout_s)
        return iss_end("Phasing-Bahn anheben: Triebwerk läuft weiter");
    break;
}
// erase_body
if (nose_cone_epoch_) { if (nose_cone_body_ == index) nose_cone_epoch_.reset(); else shift(nose_cone_body_); }
```

`raise_start_epoch_` must be set where the separation-based limit starts (the separation epoch the window plan used;
check `refuelling_mission.cpp:220-221` and use the same epoch). Text: "Bahnanhebung der Station". Remove
`RefuellingFlightMission::port`, its parser line and the TOML keys.

- [ ] **Step 4: Run, expect PASS** - `[refuelling][iss_mission],[iss][proximity],[refuelling][config],[iss_mission]` incl. `[slow]` (background, `tmp/f4_t6.txt`, check every 10 min). A reference flight that is now refused (deadline shorter by the raise time) is reported to the user, not worked around.

- [ ] **Step 5: Commit + push** - `fix(sim): mission chain - generation counter, docked start, raise limits, deadline from separation`.

---

### Task 7: Rendezvous burn model at the throttle the burn flies

**Files:**
- Modify: `core/src/sim/flight_session_rendezvous.cpp` (~70-78 `rendezvous_engines`, ~282-288 `rendezvous_fly_due_burn`), `core/include/starsim/sim/flight_session.hpp`
- Test: `core/tests/test_refuelling_rendezvous.cpp`

**Interfaces:**
- Produces: `RendezvousEngineModel FlightSession::rendezvous_burn_model(double dv_mps) const` (public getter of the existing model; `dv = inf` -> full throttle as today).

- [ ] **Step 1: Failing test** `[refuelling][rendezvous]`: for a dv between the min-throttle and full-throttle thresholds of the belly Raptor, `rendezvous_burn_model(dv).thrust_n` equals the min-throttle configuration's thrust.

- [ ] **Step 2: Run, expect FAIL**.

- [ ] **Step 3: Implement** - mirror `start_burn`'s choice (`flight_session_burns.cpp:101-107`):

```cpp
for (const auto& c : set)
    if (c.minimum_dv_mps <= small_burn_transient_share * dv_mps)
        return {mass_kg, c.thrust_n, c.mass_flow_kgps};
return {mass_kg, set.back().thrust_n, set.back().mass_flow_kgps};
```

In `rendezvous_fly_due_burn`: when `rendezvous_burn_model(|dv|).thrust_n != rendezvous_burn_model(inf).thrust_n`,
retarget once more with that model (generalise the existing cold-gas second pass).

- [ ] **Step 4: Run, expect PASS** - `[refuelling][rendezvous],[iss_mission]` incl. slow (background) -> `tmp/f4_t7.txt`.

- [ ] **Step 5: Commit + push** - `fix(sim): rendezvous burns modelled at the throttle they fly`.

---

### Task 8: Lunar PDI settling and the in-orbit ignition count

**Files:**
- Modify: `core/src/sim/flight_session_lunar_descent.cpp` (~265-276), `core/include/starsim/mission/lunar_landing_mission.hpp` (`BudgetLeg`), `core/src/mission/lunar_landing_mission.cpp` (~146-149)
- Test: `core/tests/test_lunar_descent_session.cpp`, `core/tests/test_refuelling_campaign.cpp`

**Interfaces:**
- Produces: `int BudgetLeg::ignitions` (default 1; approach/ascent legs 0; NRHO transfers = the burns the session flies).

- [ ] **Step 1: Failing tests**
  - `[sim][lunar_descent_session][ignition_settling][slow]`: settled at PDI -> no push gas drawn, PDI on time (+-1 s).
  - Same tags: weakened RCS (translation x 0.2) -> PDI waits for settling (bounded by `settling_hold_factor` x lead), no "Gasansaugung".
  - `[refuelling][campaign][lunar]`: the NRHO budget's `lunar_in_orbit_ignitions` equals the sum of `ignitions` over its legs, and that sum equals the number of in-orbit burns counted from the NRHO session plan (verify in `flight_session_lunar*.cpp` how many burns "NRHO -> LLO" and "LLO -> NRHO" fly and set the leg values from there).

- [ ] **Step 2: Run, expect FAIL**.

- [ ] **Step 3: Implement**

```cpp
// PDI: no push when settled; when pushing, wait for settling (bounded) before lighting
if (lead > 0.0 && !settling_push_) {
    if (vehicle.stage_settled(*stage)) { lunar_step_epoch_ = pdi - startup; break; }
    start_settling_push(); push_start_ = epoch_; ...
}
if (settling_push_ && !vehicle.stage_settled(*stage) && epoch_ - push_start_ < settling_hold_factor * lead) break;
begin_powered_descent(...);
```

```cpp
// lunar_landing_mission.cpp
int ignitions = 0;
for (const auto& leg : budget.legs) ignitions += leg.ignitions;
```

- [ ] **Step 4: Run, expect PASS** - `[lunar_descent_session],[lunar],[refuelling][campaign]` incl. slow (background) -> `tmp/f4_t8.txt`.

- [ ] **Step 5: Commit + push** - `fix(sim): PDI settles only when needed and waits for it; lunar ignitions counted per leg`.

---

### Task 9: Campaign calculator, setup and CLI

**Files:**
- Modify: `core/src/sim/refuelling_campaign_setup.cpp` (~31, ~39, ~113, ~215, ~280-331), `core/src/mission/refuelling_campaign.cpp` (~81, ~106-107), `core/include/starsim/mission/refuelling_campaign.hpp`, `core/src/mission/iss_launch_window.cpp` (~213), `core/src/config/mission_config.cpp` (~485-494), `core/src/mission/refuelling_mission.cpp` (~130), `core/src/mission/refuelling_campaign_text.cpp` (~58-62), `config/refuelling/reference_flights.toml`
- Test: `core/tests/test_refuelling_campaign.cpp`, `core/tests/test_iss_launch_window.cpp`, `core/tests/test_refuelling_mission_config.cpp`

**Interfaces:**
- Produces: `inline double gross_for_net_kg(double net_kg, double vent_fraction)`; `CampaignModels::hls_wait_kg` (`std::function<double(double arrival_s)>`); `CampaignInputs::hls_tank_room_kg`; `CampaignReport::hls_wait_kg`; `reference_flights.toml inputs_digest` (FNV-1a 64 over the listed input TOMLs, LF-normalised).

- [ ] **Step 1: Failing tests** (`[refuelling][campaign]` fast unless noted):
  - Vent: f = 0.5, extra 10 t -> the depot must hold 20 t.
  - Wait: a model with `hls_wait_kg(arrival) = k x arrival` shifts the required fill by `gross(k x arrival)`; `report.hls_wait_kg` reported.
  - Tank room 1 t, wait 2 t -> refusal containing "HLS-Tanks zu klein für die Wartezeit".
  - Negative HLS extra lowers the required fill, never below the chill-down floor.
  - CLI: `--reference-phasing` first, middle, last, twice; `--height-km` without a value -> error text.
  - `[refuelling][campaign][config]` "The measured numbers belong to the current configs": recomputed digest == stored digest, else message "re-measure via the slow r5 flights, then update inputs_digest".
  - `[iss][launch_window]`: two candidates failing for different reasons -> the refusal contains both texts.
  - `[refuelling][config][mission]`: `candidates_km = []` -> error; neither periapsis nor candidates -> error; `window_margin_h` without candidates is read; `reference_phasing` without `periapsis_km` -> refusal "Referenz-Phasing braucht periapsis_km".
  - `[refuelling][campaign][slow]` (extend "Campaign levers move the count the physical way"): `hls_required_mass_kg` at 300 km < at 200 km.

- [ ] **Step 2: Run, expect FAIL** -> `tmp/f4_t9_red.txt`.

- [ ] **Step 3: Implement**

```cpp
// refuelling_campaign.hpp
// Gross depot draw that delivers net_kg when the share vent_fraction of the gross is vented [D].
inline double gross_for_net_kg(double net_kg, double vent_fraction) { return net_kg / (1.0 - vent_fraction); }
// calculator: required = max(chilldown_floor, demand + gross_for_net_kg(extra + wait(hls->arrival_s), f))
// tanker side unchanged (vent = offered * f); inputs reject f >= 1
```

Setup: `lunar_at_depot.parking_height_m += dh` into `hls_demand`; `hls_wait_kg = [=](double arrival_s) { return
wait_s * orbit_boil_off_kgps(hls, ..., depot_at(start + arrival_s), sun_at(start + arrival_s)); }`;
`hls_tank_room_kg = capacity - required_propellant`; HLS extra = (cost - reference cost) + hls_rate x (phasing_s -
reference phasing_s), signed; settling flag from one source `vehicle::FlightEnvironment{}.ignition_settling`, passed
to `hls_demand` (`session.set_ignition_settling(settling)`); comments: tag `[E] the return's entry perigee` moves to
`deorbit_perigee_height_m`, `later_launch_step_s` gets `// [E] probe step of later launches to the same Waypoint 0`.

```cpp
// iss_launch_window.cpp: all distinct reasons
if (std::ranges::find(reasons, c->schedule.reason) == reasons.end()) reasons.push_back(c->schedule.reason);
last_reason = join(reasons, "; ");
```

```cpp
// refuelling_campaign_text.cpp
for (std::size_t k = 0; k < args.size();) {
    if (args[k] == "--reference-phasing") { levers.reference_phasing = true; ++k; continue; }
    if (k + 1 >= args.size()) return error("Wert fehlt für " + args[k]);
    ...; k += 2;
}
```

Digest: `inputs_digest` written by a small helper `fnv1a64_files(paths)` in the test support; the stored value set
once after Task 12's re-measure (until then the test computes and prints it).

- [ ] **Step 4: Run, expect PASS** - `[refuelling][campaign],[iss][launch_window],[refuelling][config]` incl. slow (background) -> `tmp/f4_t9.txt`.

- [ ] **Step 5: Commit + push** - `fix(mission): campaign vent, HLS wait and altitude, window reasons, config edges, CLI`.

---

### Task 10: Bindings export port mechanism and elements

**Files:**
- Modify: `bindings/src/flight_snapshot_iss.cpp` (~47-56 `iss_state` ports, ~169-176 `station_geometry` ports)
- Test: `game/tests/smoke_test.gd` (~237-246)

- [ ] **Step 1: Failing smoke assertion**: `geo.ports[0].has("mechanism")`; with the depot as station `geo.ports[0].elements_m.size() == 4`.
- [ ] **Step 2: Run smoke** (expect FAIL): `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd > tmp/f4_t10_red.txt`.
- [ ] **Step 3: Implement**

```cpp
port["mechanism"] = p.mechanism == config::DockingMechanism::ProbeDrogue ? "probe_drogue" : "idss";
godot::Array elements;
for (const auto& e : p.elements_m) elements.push_back(to_godot(e + offset_m));  // station_geometry: base-shifted
port["elements_m"] = elements;                                                 // iss_state: stage frame
```

- [ ] **Step 4: Build with bindings (`tools/build.ps1 -TestFilter "[iss][docking]"`), smoke PASS** -> `tmp/f4_t10.txt`.
- [ ] **Step 5: Commit + push** - `feat(bindings): station ports export mechanism and probe/drogue elements`.

---

### Task 11: Step dependence of burn end states (time-boxed investigation)

**Files:**
- Test: `core/tests/test_ignition_settling.cpp` (~358), hidden diagnostic case `[.step_dependence]`
- Possibly modify: `core/src/sim/flight_session.cpp` (~776-797 coast/tick boundaries, `time_to_ignition_s` refresh)

Time box: ~2 h. Use superpowers:systematic-debugging.

- [ ] **Step 1: Diagnostic case** printing, for 10 s and 600 s advance steps of the same phasing burn: ignition epoch, cutoff epoch, delivered |dv|, attitude error at ignition, end state.
- [ ] **Step 2: Classify** - equal |dv| and shifted timing -> refresh `autopilot_.burn.time_to_ignition_s` right after the coast (not on the next tick); different |dv| -> look at the per-segment `stop_rcs` / `snap_ideal_attitude` and the burn executor tail-off.
- [ ] **Step 3: Fix if the cause is found**; the existing case at ~358 then compares end states (|dv| difference < 0.5 m/s [E]). Run `[ignition_settling],[burn_node],[refuelling][iss_mission],[iss_mission],[ship][deorbit]` incl. slow in the background.
- [ ] **Step 4: Otherwise DOCUMENT** the measured numbers and suspects in `docs/subsystems.md` (settling section) and keep the case hidden.
- [ ] **Step 5: Commit + push** - `fix(sim): ...` or `docs: step dependence of burn end states measured`.

---

### Task 12: Documentation, re-measure, final review

**Files:**
- Modify: `docs/subsystems.md` (refuelling sections ~1140, ~1239-1240, ~1347-1356, ~1471, ~1507, ~1544; ISS departure ~1066-1072), `config/refuelling/reference_flights.toml`, CLAUDE.md only if a module interface summary changed
- Ledger: `.superpowers/sdd/2026-10-21-refuelling-f4-deferred-minors/progress.md`

- [ ] **Step 1: DOCUMENT items** in `docs/subsystems.md`, one paragraph "f4 decisions" plus edits in place:
  - depot header 35 t stays aboard (transfer connectors take Main tanks only); usable depot fill 1,600 - 35 t;
  - station limits override caller values too (safety limits a caller must not loosen);
  - probe/drogue wall -> plate force step at the mouth rim (mm penetration, 2 kN/cm) kept: a nearest-face fix would move capture outcomes of the slow campaigns for no physical gain;
  - departure burn targets the CW drift for the ISS too (cross-reference; re-measure the IDA-2 numbers with `STARSIM_VERBOSE=1 -TestFilter "[iss][docking]"`);
  - ascent tail-off gate on the controlled body only (the separated booster's tail-off does not move the ship's apsides);
  - phasing deadline now counts from the separation ("full limit" corrected);
  - `load_stage_from` reloads the source per from-stage (ms once per load, not worth a cache);
  - FlightSim campaign command holds the simulation clock ~0.5-1 s (`SessionRunner` accepts long calls, like a return plan);
  - `DeorbitParameters::boil_off_kgps` offline knob.
- [ ] **Step 2: Re-measure** the r5 reference flights (`[refuelling][mission][slow]`, background, check every 10 min) if Tasks 3/6/7 moved them; update `reference_flights.toml` and its `inputs_digest`; run `build/core/refuelling_report.exe` and record the campaign answer (f3: 13 tankers / 49.7 d).
- [ ] **Step 3: Final review** - superpowers:requesting-code-review on the f4 range (opus), fix pass, affected tags re-run.
- [ ] **Step 4: Ledger + memory** (done, numbers, deferred), commit `docs: refuelling f4 deferred minors`, push; ask the user for `/clear`, next step: ship catch robustness plan (user to place), then f2.
