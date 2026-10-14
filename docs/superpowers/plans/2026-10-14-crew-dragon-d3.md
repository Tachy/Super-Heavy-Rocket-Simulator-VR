# Crew Dragon capsule entry (d3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** From orbit, the Crew Dragon's onboard computer plans and flies a Draco deorbit burn, jettisons the trunk,
closes the nose cone and guides a lifting entry by bank (offset-CG trim) to the drogue deployment point ("gate",
5.5 km above the splashdown zone off California).

**Architecture:**
- New blunt-body aerodynamics `aero/capsule_aero` with a table generated offline by modified Newtonian theory
  (`tools/aero/gen_capsule_aero.py`). A radial dry CG offset of the capsule stage trims it at an angle of attack with
  L/D ~0.2.
- `Vehicle` uses the capsule aero when its lowest stage carries a capsule table, i.e. once the trunk is gone.
  `MassProperties` gains the lateral CG offset. The aerodynamic and RCS moments are taken about the true 3-D CG.
- New `gnc/capsule_entry_control`: the Dracos roll the lift vector about the air velocity to the commanded bank and
  damp pitch and yaw. The aerodynamic trim holds the angle of attack, and the RCS never fights it. In thin air the
  controller holds the full trim attitude instead.
- The existing entry guidance and predictor take a capsule profile: a trim aero table over Mach, no terminal phase,
  a 5.5 km gate, faster bank rates and a crew g-limit. `gnc/deorbit_planner` gains a finite-burn model, because the
  Draco burn lasts ~13 min.
- `FlightSession::plan_capsule_return` runs the sequence: deorbit burn on RCS, trunk jettison, nose cone closed, coast
  in trim attitude, entry guidance, gate. d4 adds parachutes and splashdown from the gate.

**Tech Stack:** C++20 core (`core/`), Catch2 tests, TOML/CSV config, Python (numpy) for the offline table generator,
CMake/Ninja via `tools/build.ps1`.

**Spec:** `docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md`, section "d3 design: capsule entry". d1 and d2
are done; see CLAUDE.md "RCS (d1 ...)" and "Crew Dragon (d2 ...)".

## Global Constraints

- Answer the user in German. Code, identifiers, comments, commit messages and docs are in English. UI strings in the
  core are German.
- SI units internally, angles in radians, and the frame or unit in each name. Degrees appear only in config (`_deg`)
  and CSV columns (`alpha_deg`).
- Every config value carries a provenance tag: `[S:<ref>]`, `[E]` or `[D]`.
- RCS thrusters are on/off only, with fixed thrust. The autopilots fly on the pulses (user 2026-10-14).
- `core` has no Godot dependency. Bindings, the game and the UI are not touched (d5 builds the interface; the user
  specifies drawing and UI).
- Wind is not modelled. Parachutes and splashdown are d4: the d3 flight ends at the gate.
- Commits follow Conventional Commits and end with:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01EceFMdgntBrWKfVojmXJKE
  ```
- Work on the branch `feat/crew-dragon-entry`, created from `main` (`fe38b9e`). Do not merge or push without the
  user's explicit request.
- Write multi-line edits with the Edit or Write tool, or with a python script in
  `C:\Users\johan\.claude\jobs\78816a1a\tmp\`. Bash heredocs mangle `\n` and quotes.
- Build (core): `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests > <log> 2>&1`, then
  grep the log for `" error "`. Bindings build: `cmd //c "C:\\Users\\johan\\.claude\\jobs\\78816a1a\\tmp\\build_noscan.cmd"`
  prints `BUILD_OK`. The bindings must still compile, because `AeroResult` and `MassProperties` gain fields.
- Single test: `build/core/tests/starsim_core_tests.exe "<name or [tag]>"`.
- Full suite (~4 min, 362/362 before d3):
  `"/c/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/ctest.exe" --test-dir build -j 12`.
- The existing campaigns must keep their results: Super Heavy catch 21/21, ship from the gate 20/21, ship from orbit
  14/14, Falcon 9 RTLS / ASDS 21/21. d3 changes shared code (the moment arm about the CG, the entry guidance and
  predictor, the deorbit planner), so the full suite at the end of every task covers them. A lateral CG offset of zero
  must give bit-identical moments.

## Decisions taken in this plan (rulings for the user's review)

1. **Aero source.** The spec asks for NASA public-domain data for an Apollo-like capsule. I could not find a free,
   tabulated full-Mach data set that we may commit. The table is therefore computed by **modified Newtonian theory**
   (Lees), the standard method for blunt bodies, which NASA used for Apollo and Orion:
   - The stagnation pressure coefficient C_p,max over Mach comes from the Rayleigh pitot formula (perfect gas,
     gamma 1.4) [D].
   - The geometry is a spherical-section heat shield, R_n = 1.2 D [E], the ratio of Apollo and Orion. A 15 deg sidewall
     cone follows [S: spec].
   - Below ~Mach 2 Newtonian theory is outside its range. It still gives a subsonic C_D ~1.0, the usual capsule value
     [E]. The table is tagged [D] hypersonic and [E] below Mach 2.
   - The check against Apollo's published hypersonic values (trim alpha ~20-25 deg at L/D ~0.3, C_A ~1.3-1.5) is a
     plausibility check in the test, not a fit.

   **Cost if wrong:** the subsonic drag sets the speed at the gate (checked against the published ~560 km/h, +-15 %).
2. **Lateral CG offset.** `[[stage]] dry_cg_offset_m = [y, z]` [D]. The generator solves for the offset that trims
   the entry mass at L/D 0.20 [E: spec "~0.2"]. Only the aerodynamic and RCS moments use the lateral offset.
   Engines, grid fins, flaps, legs and pins keep the axis: no other vehicle has an offset, and the capsule fires no
   engine. The inertia stays the principal moments on the axis; the parallel-axis term of the offset is
   < 0.1 % [D]. Gravity-gradient torque is not modelled.
3. **No rarefied bridging for the capsule.** Above ~90 km the capsule uses the table's highest Mach row.
   - Free-molecular drag would be ~1.5x higher, but the dynamic pressure there is < 1 Pa.
   - The guidance's online drag and lift scales absorb the difference.
   - The capsule's aero has no damping derivatives; the Dracos damp.
4. **The nose cone has no aerodynamic effect.** The spec (d2) says the cone's flag "changes the drag in d3". At entry
   the cone is always closed, and in orbit the drag is negligible. An open cone in the entry is judged as a failure
   instead: the heat-shield corridor reports "Bugklappe offen", warning only.
5. **Order of events [S].** Deorbit burn, then trunk jettison, then nose cone closed. This is SpaceX's order since
   the return of West Coast recovery (2024/25): the burn comes before the trunk jettison so that the trunk falls
   uprange of the capsule off California. Timings [E]: trunk jettison 120 s after the burn ends, cone closed 120 s
   after that.
6. **Deorbit thrusters.** The burn flies the existing RCS translation along +x, on the four side "forward" Dracos
   (1.6 kN). It does not use the four nose Dracos (−x): those need the cone open, and the executor already prefers +x.
   - Burn time for ~100 m/s on 12.5 t is ~13 min. Published burns last 7.5-12 min [S: NASA/SpaceX return timelines,
     Crew-2 "~12 min"].
   - Our longer burn follows from the layout estimate (d2 [E]).
   - The test reports the burn time, compares it with the published range, and does not fail on it.
7. **Gate 5.5 km [S]** above the splashdown zone's sea level (drogues at ~18,000 ft, ~350 mph: NASA Crew-2). The
   speed at the gate is checked against 156 m/s +-15 %.
8. **Splashdown zone [E]** at 32.60 N, 117.80 W, ~50 km off San Diego (Crew-10 and Ax-4 splashed down off San Diego in
   2025 [S: NASA]), radius 10 km [E: spec]. One zone in `config/splashdown_zones.toml`.
9. **Entry interface** at 120 km and −1.2 deg [E]. A capsule with L/D 0.2 has little range authority, so the nominal
   is made a bit steeper than the ship's −1.0 deg. The planned bank is 60 deg [E]: the guidance keeps range to spare
   both ways. Crossrange is planned within 20 km [E]; the capsule's lateral reach at L/D 0.2 is small.
10. **Crew g-limit 4 g [E: spec].** The guidance treats it like the ship's load limit: it cuts the bank back when the
    prediction exceeds 90 % of it. The body reports `crew_load_warning()` above it, with no breakup. Structural
    breakup stays at `max_load_g` 15 g [E].
11. **Heat shield** (PICA-X): Sutton-Graves nose radius 4.8 m [E] (= R_n), heat-rate limit 2.0 MW/m^2 [E], heat-load
    limit 400 MJ/m^2 [E]. A LEO entry peaks well below these values. They are corridor bounds for a failed guidance,
    not the material's limits.
12. **Bank control.** 10 deg/s bank rate limit [E] (Apollo rolled at up to 20 deg/s). No reversals below 600 m/s,
    as for the ship.
13. **Deferred d2 minors fixed here** (they bite d3's tests):
    - `place_in_orbit` resets the payload separation and the nose-cone timer.
    - The attitude mode is cleared by `disengage_autopilot` and when a burn starts. Before this, a mode set before a
      burn turned the capsule back after it.

    The other d2 minors stay deferred.

## Review Focus

1. **A capsule placed far off trim at the entry interface** (tumbling 2 deg/s, alpha 90 deg). The thin-air hold turns
   it to the trim attitude before the dynamic pressure builds up; the entry still reaches the gate. Test in Task 3.
2. **Bank reversal through 0 deg vs through 180 deg.** The commanded bank changes sign. The controller must roll the
   short way through lift-up (|bank| < 90 deg), and never through lift-down, which would steepen the entry.
   Test in Task 3.
3. **Deorbit planned with the trunk attached but flown after the trunk is gone** (mass 12.5 -> ~10.5 t). The
   predictor's mass must be the capsule's entry mass, not the planning-time stack mass. Test in Task 6 (the planned
   entry mass equals the flown one within 1 %).
4. **No pass within reach** (orbit plane far from the zone). The plan is refused with a German reason, and the
   capsule stays in its attitude mode. Test in Task 6.
5. **Vehicles without a lateral offset** must not change. Super Heavy, ship and Falcon 9 moments are bit-identical:
   cg offset (0,0) adds exactly zero. Test in Task 2, and the campaigns in the full suite.

---

## File structure

| File | Responsibility |
|---|---|
| `tools/aero/gen_capsule_aero.py` (new) | Offline: modified-Newtonian table and trim offset solve; prints the offset |
| `config/aero/crew_dragon_aero.csv` (new, generated) | mach, alpha_deg, ca, cn, cm (about the shield apex) |
| `core/include/starsim/aero/capsule_aero.hpp`, `core/src/aero/capsule_aero.cpp` (new) | Table interpolation, force and moment, trim solve |
| `core/include/starsim/aero/slender_body_aero.hpp` | `AeroResult::moment_body_nm` (pure moment about the force station) |
| `core/include/starsim/config/vehicle_config.hpp`, `core/src/config/vehicle_config.cpp` | `dry_cg_offset_m`, `aero.capsule_table`, `aero.entry_profile`, `heat_shield.crew_load_limit_g`, `recovery.method = "splashdown"` |
| `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp` | `MassProperties::cg_offset_m`, capsule aero dispatch `hull_aero(...)`, RCS allocator about the 3-D CG |
| `core/include/starsim/propulsion/rcs_allocator.hpp`, `.cpp` | CG as a point (station, y, z) |
| `core/src/vehicle/flight_body.cpp`, `flight_body.hpp` | moment arm with the offset, crew load warning |
| `core/include/starsim/gnc/capsule_entry_control.hpp`, `core/src/gnc/capsule_entry_control.cpp` (new) | bank measurement, roll about the air velocity, damping, thin-air trim hold |
| `core/include/starsim/gnc/entry_aero_table.hpp`, `.cpp` | `EntryAeroTable::capsule(const Vehicle&)` |
| `core/include/starsim/gnc/entry_guidance.hpp`, `.cpp` | capsule constructor, `EntryGuidanceParameters::capsule()`, no terminal phase |
| `config/splashdown_zones.toml` (new), `core/include/starsim/config/splashdown_zone_config.hpp`, `core/src/config/splashdown_zone_config.cpp` (new) | the recovery zone |
| `core/include/starsim/gnc/deorbit_planner.hpp`, `.cpp` | finite burn, entry mass override |
| `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` | splashdown zone, `place_entering_capsule`, `plan_capsule_return`, capsule sequence, the minors |
| `config/vehicles/crew_dragon.toml` | offset, aero table, heat shield, recovery |
| `core/tests/test_capsule_aero.cpp`, `core/tests/test_capsule_entry.cpp` (new) | Tasks 1-7 tests |
| `CLAUDE.md`, `config/README.md` | docs (Task 7) |

---

### Task 1: Capsule aerodynamics (generator, table, `aero/capsule_aero`)

**Files:**
- Create: `tools/aero/gen_capsule_aero.py`, `config/aero/crew_dragon_aero.csv`,
  `core/include/starsim/aero/capsule_aero.hpp`, `core/src/aero/capsule_aero.cpp`, `core/tests/test_capsule_aero.cpp`
- Modify: `core/CMakeLists.txt` (source), `core/tests/CMakeLists.txt` (test file),
  `core/include/starsim/aero/slender_body_aero.hpp` (`AeroResult::moment_body_nm`)

**Interfaces:**
- Produces:
  - `aero::CapsuleAeroTable load_capsule_aero_table(const std::filesystem::path& csv)`
  - `class aero::CapsuleAero { CapsuleAero(CapsuleAeroTable, double reference_diameter_m, double apex_station_m);
    AeroResult compute(const Vec3& air_velocity_body_mps, double density_kg_m3, double speed_of_sound_mps) const;
    Coefficients coefficients(double mach, double theta_rad) const; CapsuleTrim trim(double mach, const Vec3& cg_body_m) const; }`
  - `struct aero::CapsuleTrim { double theta_rad; double cd; double cl; double lift_to_drag; Vec3 air_velocity_body_unit; bool found; }`
  - `AeroResult::moment_body_nm` (default zero): a pure moment that adds to `cross(station - cg, force)`.

**Conventions** (written into the header comment):
- Body x runs from the heat-shield apex towards the nose. The apex is at station `apex_station_m`, which is 0 for the
  capsule stage.
- `u` is the vehicle's velocity relative to the air, in body axes. `theta` is the angle between `u` and −x (0: shield
  straight into the flow). `rho` is the unit radial direction of `u`: `normalize(u − (u·x)x)`, or +y when `u` is axial.
  The windward side is +rho.
- Force `F = q S (C_A x − C_N rho)` with `S = pi D^2 / 4`. The moment about the apex is `M = q S D C_m (x × rho)`.
- Pure spherical cap (no sidewall in the flow): the force passes through the sphere centre at `x = R_n`, so
  `C_m = −C_N R_n / D`. This is the generator's self-check.

- [ ] **Step 1: Write the generator**

`tools/aero/gen_capsule_aero.py`:

```python
"""Crew Dragon blunt-body aerodynamics by modified Newtonian theory (offline; output committed).

Geometry [E]: spherical-section heat shield of radius R_n = 1.2 D (Apollo / Orion ratio), diameter D = 4.0 m [S],
15 deg sidewall cone [S] up to the top at 4.0 m. Pressure: C_p = C_p,max (n . u)^2 on surfaces facing the motion, 0 in
the shadow; C_p,max from the Rayleigh pitot formula (gamma 1.4) above Mach 1, isentropic stagnation below
(Newtonian theory is outside its range there: [E]). Columns: mach, alpha_deg (theta: u against -x), ca, cn, cm (about
the shield apex, reference D). Conventions: core/include/starsim/aero/capsule_aero.hpp.

Usage: python tools/aero/gen_capsule_aero.py [--out config/aero/crew_dragon_aero.csv]
       python tools/aero/gen_capsule_aero.py --trim --lift-to-drag 0.20 --cg-station-m 1.25 \
           --entry-mass-kg 10500 --dry-mass-kg 8438
"""
import argparse
import math

import numpy as np

GAMMA = 1.4
D = 4.0
R_B = 0.5 * D
R_N = 1.2 * D
SIDEWALL_RAD = math.radians(15.0)
HEIGHT = 4.0
PHI_MAX = math.asin(R_B / R_N)
X_SHOULDER = R_N * (1.0 - math.cos(PHI_MAX))
S_REF = math.pi * D * D / 4.0
MACHS = [0.3, 0.5, 0.7, 0.9, 1.0, 1.1, 1.2, 1.5, 2.0, 3.0, 5.0, 10.0, 20.0, 30.0]
THETAS_DEG = list(range(0, 62, 2))


def cp_max(mach):
    g = GAMMA
    if mach <= 1.0:
        return 2.0 / (g * mach * mach) * ((1.0 + 0.5 * (g - 1.0) * mach * mach) ** (g / (g - 1.0)) - 1.0)
    a = ((g + 1.0) ** 2 * mach * mach / (4.0 * g * mach * mach - 2.0 * (g - 1.0))) ** (g / (g - 1.0))
    b = (1.0 - g + 2.0 * g * mach * mach) / (g + 1.0)
    return 2.0 / (g * mach * mach) * (a * b - 1.0)


def panels(n_phi=120, n_x=120, n_psi=144):
    """Points, outward normals and areas of the heat shield cap and the sidewall cone."""
    pts, nrm, area = [], [], []
    psi = (np.arange(n_psi) + 0.5) * 2.0 * math.pi / n_psi
    dpsi = 2.0 * math.pi / n_psi
    dphi = PHI_MAX / n_phi
    for i in range(n_phi):
        phi = (i + 0.5) * dphi
        for p in psi:
            n = np.array([-math.cos(phi), math.sin(phi) * math.cos(p), math.sin(phi) * math.sin(p)])
            pts.append(np.array([R_N, 0.0, 0.0]) + R_N * n)
            nrm.append(n)
            area.append(R_N * R_N * math.sin(phi) * dphi * dpsi)
    length = HEIGHT - X_SHOULDER
    dx = length / n_x
    for i in range(n_x):
        x = X_SHOULDER + (i + 0.5) * dx
        r = R_B - (x - X_SHOULDER) * math.tan(SIDEWALL_RAD)
        for p in psi:
            n = np.array([math.sin(SIDEWALL_RAD), math.cos(SIDEWALL_RAD) * math.cos(p),
                          math.cos(SIDEWALL_RAD) * math.sin(p)])
            pts.append(np.array([x, r * math.cos(p), r * math.sin(p)]))
            nrm.append(n)
            area.append(r * dpsi * dx / math.cos(SIDEWALL_RAD))
    return np.array(pts), np.array(nrm), np.array(area)


def coefficients(mach, theta_rad, geo):
    pts, nrm, area = geo
    u = np.array([-math.cos(theta_rad), math.sin(theta_rad), 0.0])  # rho = +y
    cosang = nrm @ u
    cp = np.where(cosang > 0.0, cp_max(mach) * cosang * cosang, 0.0)
    df = -(cp * area)[:, None] * nrm / S_REF          # force / (q S) per panel
    f = df.sum(axis=0)
    m = np.cross(pts, df).sum(axis=0) / D             # moment about the apex / (q S D)
    ca, cn = f[0], -f[1]
    cm = m[2]                                         # x cross rho = x cross y = +z
    return ca, cn, cm


def trim(mach, cg_station, lift_to_drag, geo):
    """theta where L/D equals the target, and the radial CG offset (along rho) that trims there."""
    def ld(th):
        ca, cn, _ = coefficients(mach, th, geo)
        u = np.array([-math.cos(th), math.sin(th), 0.0])
        f = np.array([ca, -cn, 0.0])
        drag = -f @ u
        lift = np.linalg.norm(f + drag * u)
        return lift / drag
    lo, hi = math.radians(1.0), math.radians(40.0)
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        (lo, hi) = (mid, hi) if ld(mid) < lift_to_drag else (lo, mid)
    th = 0.5 * (lo + hi)
    ca, cn, cm = coefficients(mach, th, geo)
    # moment about the CG (cg_station along x, offset o along rho): cm*D*z - (c x F)_z = 0 with
    # c = (cg_station, o, 0), F = (ca, -cn, 0): (c x F)_z = cg_station*(-cn) - o*ca
    o = (cm * D + cg_station * cn) / (-ca)
    return th, o, ld(th)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="config/aero/crew_dragon_aero.csv")
    ap.add_argument("--trim", action="store_true")
    ap.add_argument("--lift-to-drag", type=float, default=0.20)
    ap.add_argument("--cg-station-m", type=float, default=1.25)
    ap.add_argument("--entry-mass-kg", type=float, default=10500.0)
    ap.add_argument("--dry-mass-kg", type=float, default=8438.0)
    args = ap.parse_args()
    geo = panels()
    # self-check: a pure cap (theta below the sidewall angle keeps the cone in the shadow) acts through the centre
    ca, cn, cm = coefficients(20.0, math.radians(10.0), geo)
    assert abs(cm + cn * R_N / D) < 1e-3, (cm, -cn * R_N / D)
    if args.trim:
        th, o, ld = trim(20.0, args.cg_station_m, args.lift_to_drag, geo)
        dry_offset = o * args.entry_mass_kg / args.dry_mass_kg
        print(f"trim theta {math.degrees(th):.2f} deg, L/D {ld:.3f}, total offset {o:+.4f} m along rho "
              f"-> dry_cg_offset_m y = {dry_offset:+.4f} (rho = +y)")
        return
    with open(args.out, "w", newline="\n") as f:
        f.write("# Crew Dragon capsule aerodynamics, modified Newtonian (tools/aero/gen_capsule_aero.py) - do not edit.\n")
        f.write(f"# [D] above Mach 2, [E] below. D = {D} m, R_n = {R_N} m, sidewall 15 deg, apex reference.\n")
        f.write("mach,alpha_deg,ca,cn,cm\n")
        for mach in MACHS:
            for t in THETAS_DEG:
                ca, cn, cm = coefficients(mach, math.radians(t), geo)
                f.write(f"{mach},{t},{ca:.5f},{cn:.5f},{cm:.5f}\n")


if __name__ == "__main__":
    main()
```

Run: `python tools/aero/gen_capsule_aero.py`, then
`python tools/aero/gen_capsule_aero.py --trim --cg-station-m <capsule CG station at entry>`.
Expected: the self-check passes; hypersonic C_A at theta 0 is ~1.4-1.6 (Apollo ~1.3-1.5); the trim theta is
~15-25 deg. The printed offset goes into Task 2's config. Record the figures in the ledger.

The CG station at entry: read it from `python tools/vehicle_report.py crew_dragon` (capsule alone, ~2.1 t of
propellant) or estimate it from the config (dry 1.3 m, tank base 0.4 m). Task 2's test verifies the result in the
simulation anyway.

- [ ] **Step 2: Write the failing tests** (`core/tests/test_capsule_aero.cpp`)

```cpp
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "starsim/aero/capsule_aero.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

namespace {
aero::CapsuleAero dragon_aero() {
    return aero::CapsuleAero(aero::load_capsule_aero_table(std::filesystem::path(STARSIM_CONFIG_DIR) /
                                                           "aero/crew_dragon_aero.csv"),
                             4.0, 0.0);
}
Vec3 air_velocity(double speed, double theta_rad) {  // rho = +y
    return {-speed * std::cos(theta_rad), speed * std::sin(theta_rad), 0.0};
}
}  // namespace

TEST_CASE("Capsule aero: shield-first flow pushes along +x with the tabulated axial force", "[aero][capsule]") {
    const auto a = dragon_aero();
    const double rho = 1e-4, sos = 300.0, v = 7000.0;
    const auto r = a.compute(air_velocity(v, 0.0), rho, sos);
    const double q = 0.5 * rho * v * v, s = constants::pi * 4.0;
    CHECK(r.force_body_n.x / (q * s) > 1.3);
    CHECK(r.force_body_n.x / (q * s) < 1.7);
    CHECK(std::abs(r.force_body_n.y) < 1e-6 * r.force_body_n.x);
    CHECK(norm(r.moment_body_nm) < 1e-6 * q * s * 4.0);
}

TEST_CASE("Capsule aero: force and moment follow the radial direction of the flow", "[aero][capsule]") {
    const auto a = dragon_aero();
    const double th = 20.0 * constants::deg_to_rad;
    const auto ry = a.compute(air_velocity(7000.0, th), 1e-4, 300.0);
    const Vec3 uz{-7000.0 * std::cos(th), 0.0, 7000.0 * std::sin(th)};  // rho = +z
    const auto rz = a.compute(uz, 1e-4, 300.0);
    CHECK(ry.force_body_n.x == Catch::Approx(rz.force_body_n.x));
    CHECK(ry.force_body_n.y == Catch::Approx(rz.force_body_n.z));
    CHECK(ry.moment_body_nm.z == Catch::Approx(-rz.moment_body_nm.y));  // x cross y = z, x cross z = -y
}

TEST_CASE("Capsule aero: offset CG trims at L/D ~0.2 and is statically stable", "[aero][capsule]") {
    const auto a = dragon_aero();
    const Vec3 cg{1.25, 0.0, 0.0};
    // offset from the generator for L/D 0.2 (Task 1 step 1; the value in the ledger)
    const double offset = /* generator output (total) */ 0.0;
    const auto t = a.trim(20.0, cg + Vec3{0.0, offset, 0.0});
    REQUIRE(t.found);
    CHECK(t.lift_to_drag == Catch::Approx(0.20).margin(0.01));
    CHECK(t.theta_rad > 10.0 * constants::deg_to_rad);
    CHECK(t.theta_rad < 30.0 * constants::deg_to_rad);
    // restoring: a larger theta gives a moment that turns theta back (about the CG)
    const auto c_lo = a.pitch_moment_about(20.0, t.theta_rad - 0.02, cg + Vec3{0.0, offset, 0.0});
    const auto c_hi = a.pitch_moment_about(20.0, t.theta_rad + 0.02, cg + Vec3{0.0, offset, 0.0});
    CHECK(c_hi < c_lo);
}

TEST_CASE("Capsule aero: subsonic drag of a blunt capsule", "[aero][capsule]") {
    const auto c = dragon_aero().coefficients(0.5, 0.0);
    CHECK(c.ca > 0.85);  // [E] capsules ~0.9-1.2 subsonic
    CHECK(c.ca < 1.3);
}
```

Replace the `0.0` placeholder for `offset` with the generator's printed *total* offset (y component) before running.
The test is the check that the C++ interpolation reproduces the generator's trim.

Run: build, then `build/core/tests/starsim_core_tests.exe "[capsule]"`. Expected: it does not compile
(`capsule_aero.hpp` is missing).

- [ ] **Step 3: Implement `aero/capsule_aero`**

Header `core/include/starsim/aero/capsule_aero.hpp`:

```cpp
#pragma once

#include <filesystem>
#include <vector>

#include "starsim/aero/slender_body_aero.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::aero {

// Blunt capsule (Crew Dragon), axisymmetric, heat shield first. Body x runs from the shield apex (station
// apex_station_m) towards the nose. u: velocity relative to the air (body axes); theta: angle between u and -x;
// rho = unit radial part of u (+y when u is axial), the windward side. Coefficients from a (mach, theta) table
// (tools/aero/gen_capsule_aero.py, modified Newtonian [D] / [E] below Mach 2), reference area pi D^2 / 4:
//   F = q S (C_A x - C_N rho),   M_apex = q S D C_m (x cross rho).
// No rarefied bridging (the highest Mach row is used in thin air) and no damping derivatives [documented, d3 plan].
struct CapsuleAeroTable {
    std::vector<double> mach;       // ascending
    std::vector<double> theta_rad;  // ascending
    std::vector<std::vector<double>> ca, cn, cm;  // [mach][theta]
};

CapsuleAeroTable load_capsule_aero_table(const std::filesystem::path& csv);  // columns mach, alpha_deg, ca, cn, cm

struct CapsuleTrim {
    bool found = false;
    double theta_rad = 0.0;
    double cd = 0.0;  // along -u
    double cl = 0.0;  // perpendicular to u
    double lift_to_drag = 0.0;
    math::Vec3 air_velocity_body_unit;  // u at trim (rho = the radial direction of the CG offset's trim plane)
    math::Vec3 lift_body_unit;          // lift direction at trim, body axes
};

class CapsuleAero {
public:
    struct Coefficients {
        double ca = 0.0, cn = 0.0, cm = 0.0;
    };
    CapsuleAero(CapsuleAeroTable table, double reference_diameter_m, double apex_station_m);

    // AeroResult::normal_force_station_m = apex station; moment_body_nm = M_apex.
    AeroResult compute(const math::Vec3& air_velocity_body_mps, double density_kg_m3, double speed_of_sound_mps) const;
    Coefficients coefficients(double mach, double theta_rad) const;  // bilinear, clamped
    // Pitching moment coefficient about a CG (body point, stations from the vehicle bottom) in the plane of the CG's
    // radial offset: > 0 turns theta up. The trim: theta in [0, 60 deg] where it crosses zero with a negative slope.
    double pitch_moment_about(double mach, double theta_rad, const math::Vec3& cg_body_m) const;
    CapsuleTrim trim(double mach, const math::Vec3& cg_body_m) const;
    double reference_area_m2() const { return reference_area_m2_; }
    double reference_diameter_m() const { return diameter_m_; }

private:
    CapsuleAeroTable table_;
    double diameter_m_;
    double apex_station_m_;
    double reference_area_m2_;
};

}  // namespace starsim::aero
```

Implementation notes for `capsule_aero.cpp`:
- **CSV:** `config::load_csv_table`. Group rows by `mach` (ascending), with the same `alpha_deg` grid in each block.
  Throw `config::ConfigError` on a ragged grid.
- **`coefficients`:** bilinear in (mach, theta), clamped at the grid ends.
- **`compute`:**
  - `speed = |u|`; return a zero result for speed < 1e-6.
  - `x = (1,0,0)`. `radial = u − u.x·x`. `rho = |radial| > 1e-9·speed ? radial/|radial| : (0,1,0)`.
  - `theta = acos(clamp(−u.x/speed, −1, 1))`. `mach = speed / sos`. `q = 0.5 ρ v²`.
  - `F = q S (ca·x − cn·rho)`. `moment_body_nm = q S D cm · cross(x, rho)`.
  - `normal_force_station_m = apex_station_m_`. Also fill `mach`, `dynamic_pressure_pa`, `angle_of_attack_rad = theta`.
  - `knudsen` and `rarefaction` are 0.
- **`pitch_moment_about(mach, theta, cg)`:**
  - Let `o = (0, cg.y, cg.z)`. If `|o| < 1e-12`, `rho = +y`, else `rho = o/|o|` (the trim plane holds the offset).
  - Build `u = −cos(theta) x + sin(theta) rho`, then F and M per unit `q S` as in `compute`.
  - `c = cg − apex·x`. `M_cg = M − cross(c, F)`. Return `dot(M_cg, cross(x, rho)) / D`.
  - The sign convention gives the restoring slope dCm/dtheta < 0.
- **`trim`:** scan theta at 0.25 deg steps from 0 to 60 deg for a sign change of `pitch_moment_about` from + to −,
  then bisect (40 iterations).
  - At trim, `F = ca x − cn rho`: `cd = −dot(F, u)` and `cl = |F + cd u|`, with `lift_body_unit = normalize(F + cd u)`.
  - `found` is false without a sign change.

Add `src/aero/capsule_aero.cpp` to `core/CMakeLists.txt` after `entry_heating.cpp`, and `test_capsule_aero.cpp` to
`core/tests/CMakeLists.txt`.

Add the field `math::Vec3 moment_body_nm;  // pure moment besides the force at normal_force_station_m (capsule)` to
`AeroResult` in `slender_body_aero.hpp`. Slender bodies leave it zero.

- [ ] **Step 4: Run the tests**

Run: build, then `build/core/tests/starsim_core_tests.exe "[capsule]"`. Expected: 4 passed. If the trim test's L/D
misses, the C++ trim plane or sign differs from the generator's: debug it with the generator's numbers at the same
(mach, theta), and do not retune the table.

- [ ] **Step 5: Commit**

```bash
git add tools/aero/gen_capsule_aero.py config/aero/crew_dragon_aero.csv core/include/starsim/aero/capsule_aero.hpp \
  core/src/aero/capsule_aero.cpp core/include/starsim/aero/slender_body_aero.hpp core/CMakeLists.txt \
  core/tests/CMakeLists.txt core/tests/test_capsule_aero.cpp
git commit -m "feat(aero): blunt capsule aerodynamics from a modified-Newtonian table with offset-CG trim"
```

---

### Task 2: The capsule flies its aerodynamics (lateral CG, aero dispatch, heat shield config)

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp`, `core/src/config/vehicle_config.cpp`,
  `core/include/starsim/vehicle/vehicle.hpp`, `core/src/vehicle/vehicle.cpp`,
  `core/include/starsim/propulsion/rcs_allocator.hpp`, `core/src/propulsion/rcs_allocator.cpp`,
  `core/src/vehicle/flight_body.cpp`, `core/include/starsim/vehicle/flight_body.hpp`, `config/vehicles/crew_dragon.toml`
- Test: `core/tests/test_capsule_entry.cpp` (new), registered in `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `aero::CapsuleAero`, `aero::CapsuleTrim`, `AeroResult::moment_body_nm` (Task 1).
- Produces:
  - `StageConfig::dry_cg_offset_m` (`math::Vec3`, x unused = 0; config `dry_cg_offset_m = [y, z]`)
  - `AeroConfig::capsule_table` (string, path relative to config/), `AeroConfig::entry_profile` (`"ship"` default,
    `"capsule"`)
  - `HeatShieldConfig::crew_load_limit_g` (0 = none)
  - `MassProperties::cg_offset_m` (`math::Vec3`, x = 0): lateral CG offset from the axis
  - `const aero::CapsuleAero* Vehicle::capsule_aero() const` (non-null when the lowest stage has a capsule table)
  - `aero::AeroResult Vehicle::hull_aero(const Vec3& air_velocity_body_mps, const environment::AtmosphereState&,
    bool powered) const` (capsule or slender body). `FlightBody` calls this instead of `aero().compute`.
  - `math::Vec3 MassProperties::cg_point_m() const { return {cg_station_m, cg_offset_m.y, cg_offset_m.z}; }`
  - `RcsAllocator(std::vector<RcsThrusterGeometry>, const math::Vec3& cg_m)`, `const math::Vec3& cg_m() const`
  - `bool FlightBody::crew_load_warning() const` (set once the aero load exceeded `crew_load_limit_g`)

- [ ] **Step 1: Write the failing tests** (`core/tests/test_capsule_entry.cpp`)

```cpp
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/us76_atmosphere.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "test_support.hpp"  // whatever the d2 tests use for spice_session / environments; match test_crew_dragon.cpp

using namespace starsim;
using math::Vec3;

namespace {
// The capsule alone (trunk jettisoned) as the entry vehicle.
config::VehicleConfig capsule_config() {
    auto c = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
    c.stages.erase(c.stages.begin());
    return c;
}
}  // namespace

TEST_CASE("Crew Dragon capsule alone flies the capsule aerodynamics with its CG offset", "[dragon][entry]") {
    vehicle::Vehicle capsule(capsule_config(), STARSIM_CONFIG_DIR);
    REQUIRE(capsule.capsule_aero() != nullptr);
    const auto mp = capsule.mass_properties();
    CHECK(std::abs(mp.cg_offset_m.y) + std::abs(mp.cg_offset_m.z) > 0.01);
    const auto t = capsule.capsule_aero()->trim(20.0, mp.cg_point_m());
    REQUIRE(t.found);
    CHECK(t.lift_to_drag == Catch::Approx(0.20).margin(0.03));  // entry mass, not the generator's assumed one
    // with the trunk the stack still flies the slender body (orbit, deorbit burn)
    vehicle::Vehicle stack(config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon"), STARSIM_CONFIG_DIR);
    CHECK(stack.capsule_aero() == nullptr);
}

TEST_CASE("An uncontrolled capsule in hypersonic flow settles about its trim angle", "[dragon][entry]") {
    // 60 km, 7 km/s horizontal, started 10 deg off trim; no RCS: aero restoring moment only (no damping derivatives,
    // so it oscillates); the mean theta over the last 20 s is the trim within 2 deg.
    // Uses FlightBody::step with an empty VehicleCommand, the US76 atmosphere and central gravity (as the ship's b1
    // entry tests do: copy their environment setup from core/tests/test_ship_entry.cpp).
    // ... body built at the trim attitude from t.air_velocity_body_unit rotated by +10 deg about cross(x, rho) ...
    // CHECK(std::abs(mean_theta - t.theta_rad) < 2.0 * constants::deg_to_rad);
}

TEST_CASE("Vehicles without a lateral CG offset keep their moments", "[vehicle][rcs]") {
    vehicle::Vehicle ship(config::load_vehicle(STARSIM_CONFIG_DIR, "starship_v3"), STARSIM_CONFIG_DIR);  // ship id as in config/vehicles
    CHECK(ship.mass_properties().cg_offset_m.y == 0.0);
    CHECK(ship.mass_properties().cg_offset_m.z == 0.0);
    CHECK(ship.capsule_aero() == nullptr);
}
```

The second test's body is written in full in this step; the comments above give its contents. Copy the
environment and stepping setup from the existing ship entry tests (`grep -l "place_entering_ship\|FlightBody(" core/tests`).
Use the same pattern: an `env` with `PointMassGravity` (Earth only), `Us76Atmosphere`, `SpiceOrientation::earth()`,
and steps of `FlightSession::tick_s` for 60 s. The mean is taken of
`acos(−u_body.x/|u|)` over the last 20 s. Use the vehicle id the existing tests use for the ship.

Run: build. Expected: compile errors (`capsule_aero()`, `cg_offset_m`, `cg_point_m` are missing).

- [ ] **Step 2: Config**

- **`vehicle_config.hpp`:**
  - `StageConfig`: `math::Vec3 dry_cg_offset_m;  // lateral dry CG offset (y, z) from the axis (capsule trim); x unused`
  - `AeroConfig`: `std::string capsule_table;  // blunt capsule table (mach, alpha_deg, ca, cn, cm), path relative to config/`
    and `std::string entry_profile = "ship";  // entry guidance phases: "ship" (terminal nose turn) or "capsule"`
  - `HeatShieldConfig`: `double crew_load_limit_g = 0.0;  // crew: sustained aero load warning and guidance constraint; 0 = none`
- **`vehicle_config.cpp`:**
  - Parse `dry_cg_offset_m` as a 2-array `[y, z]` with `number_or` defaults 0. Copy the array-parsing style of
    `angles_deg` in the same file.
  - Parse `aero.capsule_table`, `aero.entry_profile` (validate: "ship" or "capsule", else `ConfigError`) and
    `heat_shield.crew_load_limit_g` (`number_or` 0).
  - Accept `recovery.method = "splashdown"`. Find the method validation with `grep -n "leg_landing" core/src/config/vehicle_config.cpp`.
- **`config/vehicles/crew_dragon.toml`**, capsule stage:

```toml
dry_cg_offset_m = [<generator dry offset y>, 0.0]   # [D] tools/aero/gen_capsule_aero.py --trim: L/D 0.20 [E: spec]
                                                     # at the entry mass; lift in the body x-y plane

[stage.recovery]
method = "splashdown"                    # d4: parachutes; d3 guides to the drogue gate

[stage.aero]
reference_diameter_m = 4.0
nose_length_m = 4.0                      # [E] slender-body nose while stacked (orbit, ascent)
capsule_table = "aero/crew_dragon_aero.csv"   # [D/E] modified Newtonian, alone after the trunk jettison
entry_profile = "capsule"

[stage.heat_shield]                      # PICA-X [S:wiki_dragon2]; limits [E]: corridor bounds, not material limits
nose_radius_m = 4.8                      # [E] R_n = 1.2 D (Apollo / Orion ratio)
max_heat_rate_w_m2 = 2.0e6               # [E]
max_heat_load_j_m2 = 4.0e8               # [E]
max_load_g = 15.0                        # [E] structure
crew_load_limit_g = 4.0                  # [E: spec] crew, sustained (warning; guidance constraint)
```

Keep the existing `[stage.aero]` keys; there is only one `[stage.aero]` table per stage. Add `[sources]`
`nasa_crew2 = "NASA Commercial Crew blog, Crew-2 return timeline (2021-11-08): drogues ~18,000 ft at ~350 mph, mains ~6,000 ft"`.

- [ ] **Step 3: Mass properties, aero dispatch and the RCS allocator**

- **`vehicle.hpp`:**
  - `MassProperties`: add `math::Vec3 cg_offset_m;  // lateral CG offset from the axis (y, z), x = 0`, and
    `math::Vec3 cg_point_m() const { return {cg_station_m, cg_offset_m.y, cg_offset_m.z}; }`.
  - `Vehicle`: add `std::unique_ptr<aero::CapsuleAero> capsule_aero_;`, the accessor
    `const aero::CapsuleAero* capsule_aero() const { return capsule_aero_.get(); }` and `hull_aero(...)`.
- **`vehicle.cpp` `mass_properties`:** the dry part of each stage carries `dry_cg_offset_m`. The composite offset is
  `Σ m_i o_i / M`; tanks and engines sit on the axis. The inertia stays as it is (ruling 2).
- **`vehicle.cpp` `rebuild_aero`:**
  - When `stages_.front().config.aero.capsule_table` is non-empty, also build
    `capsule_aero_ = make_unique<CapsuleAero>(load_capsule_aero_table(config_dir_ / table), stage.aero.reference_diameter_m, 0.0)`.
    The apex is the lowest stage's bottom, station 0. Otherwise reset it.
  - Keep `aero_` (the slender body) always: `aero()` users stay valid.
  - `separate()` calls `rebuild_aero` on both parts, so the capsule gets its aero at the trunk jettison. Verify this
    with `grep -n rebuild_aero core/src/vehicle/vehicle.cpp`.
- **`hull_aero`:** capsule if present (`compute(u, density, sos)`), else
  `aero_->compute(u, density, sos, powered, mean_free_path)`.
- **`rcs_allocator`:** the constructor takes `const math::Vec3& cg_m`. Lever arms are `position − cg_m` instead of
  `position − (cg_station, 0, 0)`. `cg_station_m()` becomes `cg_m()`. Grep all callers:
  `grep -rn "RcsAllocator(\|cg_station_m()" core/src core/tests`.
- **`vehicle.cpp` `update_rcs` / `rcs_allocator()`:** rebuild when `|cg_point − allocator.cg_m()| > rcs_cg_tolerance_m`.
  The torque `rcs_torque_` is taken about `cg_point_m()`.
- **`flight_body.cpp` `evaluate`:**

```cpp
    f.aero = ctx.vehicle.hull_aero(s.attitude.inverse_rotate(air_velocity), air, powered);
    f.aero.force_body_n = f.aero.force_body_n * ctx.vehicle.hull_aero_scale();
    f.aero.moment_body_nm = f.aero.moment_body_nm * ctx.vehicle.hull_aero_scale();
    ...
    f.moment_body_nm = cross(Vec3{f.aero.normal_force_station_m - f.mass.cg_station_m, -f.mass.cg_offset_m.y,
                                  -f.mass.cg_offset_m.z}, f.aero.force_body_n) + f.aero.moment_body_nm;
```

  With a zero offset and a zero `moment_body_nm` this is bit-identical: adding `+0.0` and `−0.0` components does not
  change finite values. Check `air`'s type name in the atmosphere header and use it in `hull_aero`.
- **`FlightBody`:** add `bool crew_load_warning_ = false;` and its accessor. Where `max_load_g` is judged
  (`grep -n max_load_g core/src/vehicle/flight_body.cpp`), set the warning when
  `crew_load_limit_g > 0 && load_g > crew_load_limit_g`, with no breakup. Heating is judged while "the stage flies
  alone". The capsule alone is a single stage, so the existing condition applies unchanged; verify it at the same
  grep.

- [ ] **Step 4: Run the tests**

Run: build (core and bindings), then `build/core/tests/starsim_core_tests.exe "[dragon]"` and `"[capsule]"`.
Expected: all pass, the d2 Dragon tests included (the stack with the trunk still flies the slender body).

- [ ] **Step 5: Full suite and commit**

Run the full suite. Expected: 362 + 7 = 369/369, and the campaigns unchanged.

```bash
git add -A core config/vehicles/crew_dragon.toml
git commit -m "feat(vehicle): lateral CG offset and capsule aerodynamics once the Dragon flies alone"
```

---

### Task 3: Capsule entry attitude control (`gnc/capsule_entry_control`)

**Files:**
- Create: `core/include/starsim/gnc/capsule_entry_control.hpp`, `core/src/gnc/capsule_entry_control.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/test_capsule_entry.cpp`

**Interfaces:**
- Consumes: `aero::CapsuleAero::trim`, `Vehicle::hull_aero`, `MassProperties::cg_point_m`, `Vehicle::max_rcs_torque_nm`,
  `gnc::steering_basis` (entry_predictor.hpp).
- Produces:

```cpp
namespace starsim::gnc {
struct CapsuleControlGains {
    double bank_rate_max_radps = 10.0 * constants::deg_to_rad;  // [E] (Apollo rolled up to ~20 deg/s)
    double bank_gain_per_s = 0.5;          // [E] rate command per bank error
    double rate_time_constant_s = 1.0;     // [E] torque = I (w_cmd - w) / tau
    double damping_deadband_radps = 1.0 * constants::deg_to_rad;  // [E] pitch / yaw rates below this are left alone
    double roll_rate_deadband_radps = 0.5 * constants::deg_to_rad;  // [E]
    double bank_deadband_rad = 2.0 * constants::deg_to_rad;         // [E]
    double aero_hold_dynamic_pressure_pa = 200.0;  // [E] below: hold the full trim attitude (no aero restoring)
};
// Bank of the capsule now: the lift direction of its aerodynamic force (perpendicular to the air velocity) in the
// steering basis of the local vertical (entry_attitude convention: lift = e1 cos(bank) - e2 sin(bank)).
double capsule_bank_rad(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);
// Body torque fractions for this tick (VehicleCommand::rcs_torque_fraction) that fly `bank_command_rad`.
math::Vec3 capsule_entry_torque_fraction(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                         time::Epoch epoch, double bank_command_rad, const CapsuleControlGains& gains = {});
// Attitude of the capsule at trim with the given bank (body -> GCRF): thin-air hold, initial states, tests.
math::Quat capsule_trim_attitude(const vehicle::Vehicle& capsule, const math::Vec3& air_velocity_gcrf,
                                 const math::Vec3& up_gcrf, double bank_rad, double mach = 20.0);
}
```

- [ ] **Step 1: Write the failing tests** (append to `core/tests/test_capsule_entry.cpp`)
  1. "Capsule trim attitude has the commanded bank". For bank in {−120, −30, 0, 45, 150} deg, a capsule set to
     `capsule_trim_attitude(...)` returns `capsule_bank_rad` within 0.5 deg of the command.
  2. "The Dracos roll the capsule to a commanded bank and hold the trim". 60 km, 7 km/s, start at trim with bank 0,
     command 60 deg. After 15 s: |bank − 60 deg| < 5 deg, theta within 3 deg of trim, and no pitch/yaw rate above
     2 deg/s. Then command −60 deg (a reversal): it rolls through 0 deg (Review Focus 2: |bank| stays < 90 deg the
     whole way) and settles within 5 deg after 20 s.
  3. "A capsule tumbling off trim in thin air turns to its trim before the air bites" (Review Focus 1). 110 km,
     theta 90 deg, 2 deg/s tumble. Fly 120 s of the entry from −1.2 deg at 7.6 km/s. At q = 1 kPa, theta is within
     5 deg of trim and |bank error| < 10 deg.
  4. "Holding bank through an entry costs tens of kilograms". The test-2 run continued 300 s with commands that
     switch every 60 s. Propellant used is < 60 kg [E: Apollo spent ~30-40 kg of RCS propellant per entry; the
     ledger records the figure].

Write each case in full. The steering loop is: every tick, `command.rcs_torque_fraction =
capsule_entry_torque_fraction(body, env, epoch, bank_cmd)`, then `body.vehicle().update_rcs(tick, command)`, then
`body.step(env, epoch, tick, command)`. Check the exact order against how `FlightSession` ticks a body:
`grep -n "update_rcs" core/src/sim/flight_session.cpp`.

Run: build. Expected: compile errors (the header is missing).

- [ ] **Step 2: Implement**

`capsule_entry_control.cpp`:
- **Air velocity** (GCRF): `v − ω_E × r`, with `earth_rotation_vector(env.earth_orientation.fixed_to_gcrf(epoch))`
  as in `flight_body.cpp`. `u_body = attitude.inverse_rotate(air)`.
- **Up:** the geodetic vertical. Copy `local_up_fixed` and rotate it to GCRF, or use the normalised radius: the
  difference is < 0.2 deg, so the radius is enough here.
- **`capsule_bank_rad`:**
  - `F = vehicle.hull_aero(u_body, atmosphere at height, false).force_body_n`, rotated to GCRF. Use the trim's
    `lift_body_unit` when `|F|` is ~0 (vacuum).
  - `lift = F − (F·û)û`. Call `steering_basis(up, û, e1, e2)`. `bank = atan2(−dot(lift, e2), dot(lift, e1))`.
- **`capsule_entry_torque_fraction`:**
  - `bank_err = wrap(bank_cmd − bank_now)` into (−π, π].
  - **Review Focus 2:** if the short way crosses |bank| = 180 deg (lift down) while the other way crosses 0, take the
    way through 0. The rule: if `bank_now` and `bank_cmd` have opposite signs and `|bank_err| ≤ π` would pass 180,
    then `bank_err = −sign(bank_now)·(|bank_now| + |bank_cmd|)`.
  - `û_body = u_body/|u|`. `w = angular velocity (body)`. `w_roll = dot(w, û_body)`. `w_perp = w − w_roll û_body`.
  - The roll is about the air velocity, and the bank is a right-handed rotation of the lift about û. Check the sign
    with test 1. `w_cmd_roll = clamp(gain·bank_err, ±rate_max)`, zero inside the bank deadband when |w_roll| is below
    the roll-rate deadband.
  - `torque = I ⊙ ((w_cmd_roll − w_roll) û_body) / tau` (component-wise inertia, principal axes).
  - Pitch/yaw: if `|w_perp| > damping_deadband`, add `−I ⊙ w_perp / tau`.
  - **Thin air** (q < `aero_hold_dynamic_pressure_pa`): instead, the full attitude error to
    `capsule_trim_attitude(...)`. `rate_cmd = clamp(0.5 · rotation-vector error, ±2 deg/s)`, and
    `torque = I ⊙ (rate_cmd − w) / tau` with the same deadbands.
  - Fraction = `torque / max_rcs_torque_nm()` per axis, clamped to ±1.
- **`capsule_trim_attitude`:**
  - `t = capsule_aero()->trim(mach, mass_properties().cg_point_m())`.
  - The desired lift direction in GCRF is `l = e1 cos(bank) − e2 sin(bank)`, from `steering_basis(up, û)`.
  - Find the rotation that maps the body pair (`t.air_velocity_body_unit`, `t.lift_body_unit`) to the GCRF pair
    (û, l). Build an orthonormal triad from each pair (a, b, a×b) and take `R = T_gcrf T_body^T`, then
    `Quat::from_matrix`.

- [ ] **Step 3: Run the tests**

Run: `build/core/tests/starsim_core_tests.exe "[dragon][entry]"`. Expected: all pass. Tune only the [E] gains above,
and ledger the final values.

- [ ] **Step 4: Full suite, commit**

```bash
git add -A core
git commit -m "feat(gnc): capsule entry control - Dracos roll the lift vector and damp, aero trim holds alpha"
```

---

### Task 4: Entry guidance and predictor for the capsule; splashdown zone

**Files:**
- Modify: `core/include/starsim/gnc/entry_aero_table.hpp`, `core/src/gnc/entry_aero_table.cpp`,
  `core/include/starsim/gnc/entry_guidance.hpp`, `core/src/gnc/entry_guidance.cpp`,
  `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`
- Create: `config/splashdown_zones.toml`, `core/include/starsim/config/splashdown_zone_config.hpp`,
  `core/src/config/splashdown_zone_config.cpp`
- Test: `core/tests/test_capsule_entry.cpp`

**Interfaces:**
- Consumes: `capsule_entry_torque_fraction`, `capsule_trim_attitude`, `capsule_bank_rad` (Task 3);
  `Vehicle::capsule_aero`, `HeatShieldConfig::crew_load_limit_g` (Task 2).
- Produces:
  - `static EntryAeroTable EntryAeroTable::capsule(const vehicle::Vehicle& capsule)`: Mach grid = the capsule
    table's, `cd_area = cd·S`, `cl_area = cl·S` at trim about the current CG. The alpha schedule is `π − theta_trim`
    (entry_attitude convention: angle between the nose and the air velocity). The alpha-grid accessors return the
    trim values at any alpha: the terminal phase never runs.
  - `EntryGuidance(const vehicle::FlightBody& capsule, EntryTarget target, EntryGuidanceParameters parameters)`:
    capsule constructor (no alpha schedule; it uses `EntryAeroTable::capsule`).
  - `static EntryGuidanceParameters EntryGuidanceParameters::capsule()`:
    - `gate_height_m` 5500
    - `bank_rate_radps` = `fast_bank_rate_radps` = 10 deg/s
    - `terminal_mach` = `terminal_mach_latest` = 0 (never)
    - `max_bank_rad` 165 deg
    - other values as the ship's
  - `bool EntryGuidanceParameters::terminal_phase = true` (capsule false), so that the phase switch is explicit and
    not tied to Mach 0.
  - Load constraint: `EntryGuidance` uses `min(max_load_g, crew_load_limit_g)` of the heat-shield config (when > 0)
    as its load limit. Find where `limits_.max_load_g` is read: `grep -n max_load_g core/src/gnc/entry_guidance.cpp`.
  - `config::SplashdownZoneConfig { std::string id, name; double latitude_rad, longitude_rad, radius_m; }`,
    `SplashdownZoneConfig load_splashdown_zone(const std::filesystem::path& config_dir, std::string_view id)`
  - `FlightSession::set_splashdown_zone(const config::SplashdownZoneConfig&)`,
    `const std::optional<config::SplashdownZoneConfig>& splashdown_zone() const`
  - `gnc::EntryTarget FlightSession::capsule_entry_target() const`: the zone centre on the WGS84 ellipsoid (height 0;
    the geoid is ignored), with a zero nose direction.
  - `FlightSession::place_entering_capsule(const config::VehicleConfig&, const vehicle::RigidBodyState&, time::Epoch,
    gnc::EntryGuidanceParameters = gnc::EntryGuidanceParameters::capsule())`

`config/splashdown_zones.toml`:

```toml
# Splashdown zones of capsule returns (Crew Dragon, d3/d4). Point and radius of the recovery area.
[sources]
nasa_2025 = "NASA: Crew-10 (2025-08-09) and Ax-4 (2025-07-15) splashed down off San Diego, California"

[[zone]]
id = "pacific_san_diego"
name = "Pazifik vor San Diego"
latitude_deg = 32.60                     # [E] ~50 km off the coast, in the area of the 2025 returns [S:nasa_2025]
longitude_deg = -117.80                  # [E]
radius_m = 10000.0                       # [E: spec]
```

- [ ] **Step 1: Write the failing tests**
  1. "Splashdown zone loads". The loader returns the zone; its latitude is 32.6 deg ± 1e-9.
  2. "Capsule entry table follows the trim". `EntryAeroTable::capsule(capsule)` at Mach 20 gives
     `cl/cd == trim L/D` (1e-9) and `alpha_rad == π − theta_trim`.
  3. "The capsule predictor matches the flown entry at constant bank". Fly the capsule (Task 3 control holding a
     constant 60 deg) from 120 km, −1.2 deg, 7.6 km/s, to 5.5 km. Predict from the same start at 60 deg with
     `predictor_j2 = false` and the scenario's central gravity. Within 15 km of downrange and 20 s of time
     [E: rarefied drag and the off-trim oscillation; the online scales close the rest in flight]. The ledger records
     the miss.
  4. "Capsule entry guidance reaches the drogue gate". `place_entering_capsule` at the entry interface: 120 km,
     −1.2 deg, ~7.6 km/s, aimed so that the nominal 60 deg bank ends near the zone. Build the state by
     back-propagating as the ship's b2 test does; find it with `grep -n "place_entering_ship" core/tests/*.cpp`.
     Expect:
     - the phase reaches `Gate`
     - the miss at 5.5 km is < 2 km (horizontal distance to the zone centre)
     - the speed at the gate is 156 m/s ± 15 % [S: ~350 mph, NASA Crew-2]
     - peak load < 4.5 g
     - no breakup
     - the heat rate and load are printed as `[validation]`

  In the session the capsule is ticked like the entering ship. `FlightSession`'s command for a body with entry
  guidance and `entry_profile == "capsule"`:
  - RCS torque fraction from `capsule_entry_torque_fraction(body, env_, epoch_, entry_command.bank_rad)`.
  - No `AttitudeController::steer`, no flaps, no engines.
  - At the gate (`EntryPhase::Gate`) hold bank 0 and keep damping (d4 adds the chutes).

  Find where the ship's entry command becomes a `VehicleCommand`: `grep -n "entry_command" core/src/sim/flight_session.cpp`.

Run: build. Expected: compile errors.

- [ ] **Step 2: Implement**
- **`entry_aero_table.cpp`:** the static `capsule(...)`.
  - Iterate the capsule table's Mach values and call `trim(mach, cg)`. A Mach row without a trim takes the nearest
    trimmed row's values.
  - The alpha grid: a single alpha (the trim) per Mach, so the bilinear accessors return the trim values. Implement
    by filling `alpha_grid_rad_` with {0, π} and identical columns. Keep the existing ship constructor unchanged.
- **`entry_guidance.cpp`:**
  - The capsule constructor fills `model_` like the ship one: mass, nose radius, bank rates, gm/j2/omega,
    `aero = EntryAeroTable::capsule(...)`. `belly_azimuth_rad_` is unused.
  - In `update`, the hand-over to `Terminal` requires `parameters_.terminal_phase`.
  - `EntryCommand.axis_gcrf` / `roll_reference_gcrf` for the capsule come from
    `capsule_trim_attitude(...).rotate(x / y)`, used for the flight director only.
- **`flight_session.cpp`:**
  - `start_entry_guidance` branches on `stage.aero.entry_profile`: the capsule target comes from
    `capsule_entry_target()`, which throws `std::logic_error("no splashdown zone")` if none is set.
  - Add `place_entering_capsule` and the per-tick command branch above. `set_accelerometer(true)` is already done for
    entering bodies; verify with grep.
- **Config loader:** follow `drone_ship_config.cpp` (TOML, `[[zone]]`, `_deg` → rad, `ConfigError` for an unknown
  id). Add the source to `core/CMakeLists.txt`.

- [ ] **Step 3: Run the tests**

Run: `build/core/tests/starsim_core_tests.exe "[dragon][entry]"`. Expected: all pass. A gate miss > 2 km is a
finding: debug the predictor-vs-truth gap of test 3 first (drag scale, trim alpha) before touching guidance gains.

- [ ] **Step 4: Full suite** (the ship's entry and return campaigns must keep 12/12, 20/21, 14/14), **commit**

```bash
git add -A core config/splashdown_zones.toml
git commit -m "feat(gnc): capsule entry guidance - bank-modulated lifting entry to the drogue gate over the splashdown zone"
```

---

### Task 5: Deorbit planner with a finite burn and an entry mass

**Files:**
- Modify: `core/include/starsim/gnc/deorbit_planner.hpp`, `core/src/gnc/deorbit_planner.cpp`
- Test: `core/tests/test_capsule_entry.cpp`

**Interfaces:**
- Produces new `DeorbitParameters` fields:
  - `double burn_thrust_n = 0.0;  // > 0: the burn is flown as a finite burn of this thrust, centred on the node (0: impulsive)`
  - `double entry_mass_kg = 0.0;  // > 0: the predictor's mass at entry (the capsule drops its trunk after the burn); 0: mass after the burn`
- Unchanged signature of `plan_deorbit`. With both new fields at their defaults the ship's plans are bit-identical.

- [ ] **Step 1: Write the failing tests**
  1. "A finite deorbit burn ends where the planner predicts". Capsule stack (trunk + capsule) in a 420 km circular
     orbit at 51.6 deg. Plan with `burn_thrust_n = 1600` and the zone target. Fly it with
     `FlightSession::execute_burn(plan.node)` (RCS +x), then coast to 120 km. Expect:
     - the flight-path angle within 0.03 deg of the plan
     - the entry epoch within 5 s
     - the same plan with `burn_thrust_n = 0` (impulsive) flown the same way misses the flight-path angle by more
       (this shows why the finite model is needed; it prints both)
  2. "Deorbit plan uses the entry mass" (Review Focus 3). The plan with `entry_mass_kg` = the capsule's mass after
     the trunk is gone and the burn differs in downrange from the plan without it, and its prediction with that mass
     ends within 1 km of the gate. Pin by comparing `plan.downrange_error_m` against a direct `predict_entry` with
     that mass.

Run: build and the tests. Expected: FAIL (fields are missing).

- [ ] **Step 2: Implement**

In `Shooter::shoot(burn, dv)`:
- **`burn_thrust_n == 0`:** the existing impulse.
- **Otherwise:**
  - `m0 = mass at the burn`, which the shooter needs: pass `mass_kg` and `exhaust_velocity_mps` into the shooter.
  - `mdot = F / ve`. `T = burn_duration_s(dv, m0, F, mdot)` (gnc/burn_node).
  - `s = state_at(burn − T/2)`. The direction is `d = −normalize(state_at(burn).velocity)`: fixed inertial, as the
    executor holds it.
  - Integrate `ballistic_dynamics(gravity, burn − T/2)` plus `F/(m0 − mdot t) d` over [0, T] with DOP853 and the
    planning options. Write a small lambda wrapping the `OdeFunction<6>`; check its signature in
    `core/include/starsim/integrators/`.
  - Then coast from `burn + T/2` to the interface as now.
- **Entry mass:** `set_mass(parameters.entry_mass_kg > 0 ? parameters.entry_mass_kg : mass − burn_propellant(dv))`.
- **Plan:** `plan.node` stays `{t, −v̂·dv}` at the centre epoch. The executor ignites half the duration before it.
- The Newton derivative step `dt = 2 s` stays.

- [ ] **Step 3: Run the tests, full suite** (ship return campaign 14/14 unchanged), **commit**

```bash
git add -A core
git commit -m "feat(gnc): deorbit planner flies long burns as finite burns and predicts the entry with the entry mass"
```

---

### Task 6: Capsule return from orbit (session sequence) and the d2 minors

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`
- Test: `core/tests/test_capsule_entry.cpp`

**Interfaces:**
- Consumes: Tasks 2-5.
- Produces:
  - `bool FlightSession::plan_capsule_return(gnc::DeorbitParameters parameters = capsule_deorbit_parameters())`:
    false with `ship_return_plan()->reason` (German) when refused.
  - `static gnc::DeorbitParameters FlightSession::capsule_deorbit_parameters()`:
    - entry interface 120 km at −1.2 deg
    - nominal bank 60 deg
    - max crossrange 20 km
    - gate 5.5 km
    - `burn_thrust_n` = the +x RCS capability (filled in by `plan_capsule_return`)
  - `enum class CapsuleReturnStep { None, DeorbitBurn, TrunkJettison, NoseConeClose, Coast, Entry, Gate }`
    and `CapsuleReturnStep FlightSession::capsule_return_step() const`.
  - Constants: `trunk_jettison_delay_s = 120.0`, `nose_cone_close_delay_s = 120.0` [E] (ruling 5).

**Sequence** (`AutopilotState::Return`, "Rückkehr"):
1. **Plan.**
   - Refuse when the controlled vehicle's top stage has no `entry_profile == "capsule"`, no splashdown zone is set,
     or there is no RCS thrust along +x. Reason: "Keine Rückkehr möglich: keine Kapsel im Orbit" or
     "Keine Wasserungszone".
   - `entry_mass_kg` = current mass − trunk stage mass (dry and its propellant, 0) − the burn's propellant. Two passes:
     plan with the mass estimate, then set `entry_mass_kg` from that plan's propellant, and plan again.
   - `exhaust_velocity` and `burn_thrust_n` come from `rcs_force_capability(+x)`.
   - The available propellant is `rcs_propellant_kg()`. The reserve is 50 kg [E] (entry control, Task 3's
     measurement plus margin).
   - Then `execute_burn(plan.node)`. The RCS path picks +x.
2. **Burn complete.** The RCS completion branch now goes to `Return` (not `Off`) when a capsule return is active.
   `capsule_return_step_ = TrunkJettison` at burn end + 120 s.
3. **Trunk jettison.** Separate the lowest stage (the trunk) with the spring push (0.3 m/s [E], as the payload
   separation). Keep the capsule controlled. Use the separation helper of d2's `separate_payload`, generalised over
   the stage index if needed. The trunk body coasts unguided.
4. **Nose cone.** 120 s later: `set_nose_cone(false)`.
5. **Coast.** The attitude mode is `Direction` with the trim attitude for entry: the orbit controller turns the
   capsule to `capsule_trim_attitude(..., bank = 0)` (the axis and roll reference of that quaternion), updated each
   tick from the current velocity. Entry guidance starts at `entry_start_epoch()` (`entry_lead_s` before the
   interface). From then the capsule control of Task 4 flies.
6. **Gate.** `capsule_return_step_ = Gate` (d4 continues).

**d2 minors** (ruling 13):
- `place_in_orbit` resets `payload_separation_` and `nose_cone_epoch_` (the payload release and cone timers).
- `disengage_autopilot()` and the start of any burn (`execute_burn` both overloads) set the attitude mode to `Free`.
- The capsule return sets its own mode after the burn.

- [ ] **Step 1: Write the failing tests**
  1. "Crew Dragon returns from orbit to the drogue gate" [dragon][mission]. `place_in_orbit("crew_dragon", 420 km
     circular, 51.6 deg, phase 0)` with the nose cone opened by the test. Set the splashdown zone and call
     `plan_capsule_return()`. Advance until `capsule_return_step() == Gate` or 48 h. Expect:
     - feasible
     - the trunk separated after the burn, and the cone closed before the entry interface
     - the gate miss is < 2 km
     - the gate speed is 156 m/s ± 15 %
     - the burn's delta-v and duration are printed as `[validation]` and compared with the published 7.5-12 min
       (printed, not checked; ruling 6)
     - RCS propellant left > 50 kg
     - no breakup, crew load warning false
  2. "No pass within reach refuses the capsule return" (Review Focus 4). An orbit at 10 deg inclination (the zone at
     32.6 N is never overflown). `plan_capsule_return()` is false, the reason is non-empty German, and the attitude
     mode is unchanged.
  3. "Attitude mode ends with the autopilot" (minor). Set mode Inertial, then `disengage_autopilot()`: Free. Set it
     again, then `execute_burn(node)`: Free once the burn starts.
  4. "place_in_orbit forgets an earlier payload release" (minor). Run the crew mission until its separation is set,
     then `place_in_orbit("crew_dragon", ...)` and advance 700 s: the nose cone is still closed (no timer from the
     earlier flight).

Run: expected FAIL (API missing).

- [ ] **Step 2: Implement** as specified above. Keep the ship's `plan_ship_return` untouched apart from shared helpers.

- [ ] **Step 3: Run the tests, full suite, commit**

```bash
git add -A core
git commit -m "feat(sim): Crew Dragon return - Draco deorbit burn, trunk jettison, nose cone closed, guided entry to the drogue gate"
```

---

### Task 7: Dispersed entry campaign, docs

**Files:**
- Test: `core/tests/test_capsule_entry.cpp`
- Modify: `CLAUDE.md`, `config/README.md`, `tools/vehicle_report.py` (only if it breaks on the new keys)

- [ ] **Step 1: Campaign test** "Capsule entry campaign reaches the drogue gate" [dragon][entry][validation]. Twelve
  entries from the interface: seed 1-12, uniform dispersion of
  - flight-path angle ±0.1 deg [S: spec]
  - mass ±3 % (scale the capsule's RCS tank)
  - truth aero ±10 % (`Vehicle::set_hull_aero_scale`)
  - entry longitude ±0.3 deg along the track (range error)

  Expect for all 12:
  - the gate within 2 km
  - the speed at the gate within 156 m/s ± 15 %
  - no breakup
  - peak load printed; the crew load warning is allowed but counted and printed

  A hidden single case `"[.][dragon]"` reads `STARSIM_CAPSULE_SEED`.

- [ ] **Step 2: Run it.** Expected: 12/12. A failure is debugged as a finding (systematic-debugging), not by widening
  the tolerance. The tolerance comes from the spec.

- [ ] **Step 3: Docs.**
  - **CLAUDE.md:** a "Crew Dragon entry (d3 ...)" bullet with the figures:
    - trim theta and L/D
    - CG offset
    - entry interface, gate and zone
    - burn time and delta-v, and the propellant spent on the burn and in the entry
    - campaign results and the full return
  - **CLAUDE.md, core module list:** `aero` gains "capsule (modified Newtonian table, offset-CG trim)".
  - **CLAUDE.md, known issues:** the Draco deorbit lasts ~13 min against the published 7.5-12 min (layout estimate);
    the subsonic capsule drag is [E].
  - **`config/README.md`:** the new keys (`dry_cg_offset_m`, `capsule_table`, `entry_profile`, `crew_load_limit_g`,
    `recovery.method = "splashdown"`) and `splashdown_zones.toml`.
  - Run `python tools/vehicle_report.py crew_dragon`.

- [ ] **Step 4: Full suite, smoke test, commit**

```bash
git add -A core CLAUDE.md config/README.md tools
git commit -m "test(dragon): dispersed capsule entry campaign; docs for the d3 capsule entry"
```

Then the final whole-branch review (fresh opus reviewer) and one fix pass, per executing-plans.
