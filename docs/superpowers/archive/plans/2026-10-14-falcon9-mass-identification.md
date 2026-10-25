# Falcon 9 Mass Identification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Falcon 9 first stage carries the propellant our webcast telemetry implies (gauge capacity ~425 t instead of
the commonly cited 395.7 t), with the throttle schedule and every gauge-derived mass re-derived, so that ascent, MECO
propellant and payload follow CRS-35 and Starlink 15-25.

**Architecture:** Configuration and identification only: `tools/telemetry/mass_estimate.py` (Falcon 9 reference area)
fixes the gauge capacity from the engines' rated mass flow; config values tagged [D] are re-derived from it; the
descent drag table is re-identified with the new descent masses; pitch kicks and the drone-ship position are re-tuned
with the existing sweep / diagnostic tests. No guidance code changes expected.

**Tech Stack:** Python telemetry tools, TOML config, Catch2 validation tests.

**Spec:** `docs/superpowers/specs/2026-10-13-falcon9-landing-design.md` (vehicle data [D] from our telemetry, c1), user
request 2026-10-14 ("setz die Punkte alle erst mal um, die Simulation betreffen": MECO propellant and payload gaps).

## Evidence (2026-10-14)

`mass_estimate.py` on both flights (T+10-136 s, Isp 282/311 s, Cd 0.4 on 10.52 m^2): with C = 395.7 t the engines
would deliver 92-94 % of their 845 kN rating in the full-thrust window T+20-48 s; with C = 425 t they deliver
0.96-1.04 throughout, except the max-Q bucket (0.84-0.88 at T+54-66 s), motion vs flow thrust within a few %, liftoff
568-576 t. Our sim with 395.7 t runs 18 % fast at T+60 (1421 vs 1188 km/h); with 435 t it runs 4 % slow.

## Global Constraints

- Provenance tags on every changed value ([S], [E], [D] with the derivation in the comment).
- Run `python tools/vehicle_report.py falcon9_block5` after the config change.
- No tuning to hit a number: kicks are swept against the recorded MECO state as before; the drone ship stays at our
  ascent's nominal impact point.

## Review Focus

1. Super Heavy / Starship use the same telemetry tools: `mass_estimate.py` defaults must stay the Starship values.
2. The RTLS mission's identified `boostback_loss_fraction` / `return_flight_time_s` depend on the MECO state: re-check
   that the full RTLS mission still lands near T+442 s with ~12 t at ignition.
3. Gauge-derived landing propellant changes (11.3 -> ~12.1 t): MECO planning targets it; campaigns must still land.
4. Second stage unchanged (M_rest scatter between flights +-10 t does not resolve it): payload to orbit must stay feasible.

---

### Task 1: Mass, throttle and gauge-derived values

**Files:** `tools/telemetry/mass_estimate.py` (`--area-m2`), `config/vehicles/falcon9_block5.toml`,
`config/missions/falcon9_rtls.toml`, `config/missions/falcon9_asds.toml`, `tests/validation/telemetry/flights.toml` (notes).

- [ ] Tanks: 425 t at O/F 2.36 [D]; `landing_propellant_kg` 2.85 % -> 12,100 kg; `reserve_propellant_kg` 11 % -> 46,750 kg;
  comments for MECO 15.5 % = 66 t (RTLS), touchdown 1.7 % = 7.2 t.
- [ ] Throttle schedule both missions [D]: `[[51, 1.0], [54, 0.85], [63, 0.85], [69, 1.0]]`.
- [ ] Entry-burn throttle / delta-v comments re-derived (values kept where kinematic).
- [ ] `vehicle_report.py`; config tests `[config][falcon9]` pass.
- [ ] Commit `fix(config): Falcon 9 first stage carries ~425 t (identified from the webcast telemetry)`.

### Task 2: Descent drag re-identified

**Files:** `config/aero/falcon9_descent.csv`; test "Falcon 9 descent drag reproduces the webcast telemetry".

- [ ] `drag_estimate.py` with the new descent masses (dry 25.6 t + gauge x 425 t); update the table and its header.
- [ ] The replay test passes (rms within its tolerance). Commit.

### Task 3: RTLS re-validated

- [ ] Kick sweep (`[.tuning]`) against CRS-35 MECO (T+137 s, 56.6 km, 6441 km/h); set `kick_deg`.
- [ ] Full RTLS mission and campaign: land; MECO / entry / landing-burn times vs CRS-35; re-identify the [D] return
  values only if the mission no longer lands near T+442 s. Commit.

### Task 4: Drone ship re-validated

- [ ] Kick against Starlink 15-25 MECO (T+149 s, 60 km, ~8000 km/h); payload: largest of 15.5 / 16.5 / 17.5 t that
  reaches 200 x 280 km with margin; drone ship moved to the new nominal impact point (hidden test).
- [ ] Full mission + both campaigns land. Commit.

### Task 5: Docs

- [ ] CLAUDE.md Falcon 9 bullets (masses, gaps closed or remaining), config/README if keys changed. Full ctest. Commit.
