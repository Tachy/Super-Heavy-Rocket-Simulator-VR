"""Ship alone in the belly-first entry, from the drawn mesh: hull and flaps separately, against the core's tables and flights.

Above Mach 3 modified Newtonian on the drawn mesh with first-hit occlusion (hull, chine, covers, flaps at the core's pitch
trim pair). Below Mach 3 Hoerner's crossflow values (`crossflow_book`), recalled and marked [E]; flaps by the core's law.
The trim pairs come from the core:

    build/core/tests/starsim_core_tests.exe "[trim_dump]" with STARSIM_TRIM_CSV=screenshots/vehicle-physics/ship_trim.csv
    python tools/aero_derive/derive_ship_entry.py [--trim <csv>]

writes screenshots/vehicle-physics/ship_entry.md. The effective crossflow coefficient is the one the core's normal force law
(`SlenderBodyAero::compute`: potential sin(2a) cos(a/2) S_ref + eta C_dc L D sin^2 a) needs to carry the mesh's hull normal
force at that alpha. Errors [E]: +-10 % Newtonian on a blunt section above Mach 4 (known cylinder and sphere cases within
that), +-30 % book values transonic, see `crossflow_book`.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import geometry  # noqa: E402
import newtonian  # noqa: E402

ROOT = geometry.ROOT
sys.path.insert(0, str(ROOT / "tools" / "telemetry"))
from drag_estimate import atmosphere, load, quadratic, slope  # noqa: E402

S_REF_M2 = math.pi / 4.0 * 9.0 ** 2
CELL_M = 0.05
SHIP_ENTRY_MASS_KG = 290000.0  # dry 255 t + header tanks 35 t: the replay's mass; the flown residue is not known
NEWTONIAN_MACHS = (4.0, 8.0, 15.0, 25.0)
SHUTTLE_G0 = 9.80665
FLIGHTS = (("flight12_2026-05-22.csv", 2820.5, 3400.0), ("flight13_2026-07-24.csv", 2800.5, 3700.0),
           ("flight14_2026-09-28.csv", 9990.5, 11100.0))


HULL_CP_ALPHAS = (5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 175)
HULL_CP_HEADER = """# Ship hull alone: station of the normal force over the angle of attack, from the base.
# [E] Written by tools/aero_derive/derive_ship_entry.py --write-hull-cp: modified Newtonian with first hit on the drawn mesh
# (hull, chine, covers, no flaps; independent of Mach within its range). The core fades it in over Mach 1-4; below Mach 4 not derived.
alpha_deg,station_m
"""


def crossflow_book(mach: float) -> tuple[float, float, str]:
    """(eta C_dc, error, source) of the drawn section below Mach 3. Fixed separation at the chines: no supercritical drop.

    Hoerner, Fluid-Dynamic Drag, recalled [E]: half-cylinder with sharp edges facing the flow about 1.2; finite-length
    factor eta 0.7 at L/D 5.8 subsonic (ch. 3), rising to 1 at Mach 1 (the wake's end effect is cut off); circular cylinder
    in crossflow with Mach (ch. 16): rising to about 1.8 near Mach 1.2, about 1.5 at Mach 2, towards the Newtonian value."""
    table = {0.3: (1.2 * 0.7, 0.25), 0.8: (1.5 * 0.8, 0.3), 1.2: (1.8, 0.4), 2.0: (1.5, 0.3)}
    v, e = table[mach]
    return v, e, "Hoerner [E]"


def flap_entries(v, kind):
    return [e for e in v.manifest["flaps"]["ship"] if e["kind"] == kind]


def mesh(v, alpha_deg, aft_deg, fwd_deg, flaps=True):
    names, chunks, ids = [], [], []

    def add(name, tris):
        names.append(name)
        chunks.append(tris)
        ids.append(np.full(len(tris), len(names) - 1, dtype=np.int64))

    for part, tris in v.hulls["ship"].items():
        add(part, tris)
    for e in v.manifest["flaps"]["ship"]:
        d = aft_deg if e["kind"] == "aft" else fwd_deg
        for part, tris in v.flap_triangles(e, d).items():
            add(f"flap_{e['kind']}", tris) if flaps else None
    return np.concatenate(chunks), np.concatenate(ids), names


def loads(tris, ids, names, alpha_deg):
    """Per unit cp_max, per group (hull, flap_aft, flap_fwd): force vector [m^2] and its moment's station [m]."""
    a = math.radians(alpha_deg)
    flow = np.array([-math.cos(a), 0.0, math.sin(a)])  # belly at -z, the ship moves along cos a x - sin a z
    hit = newtonian.first_hit(tris, flow, CELL_M, ids, names)
    f, u, v = newtonian._basis(flow)
    pu, pv = tris @ u, tris @ v
    u0, v0 = pu.min(), pv.min()
    i, j = np.nonzero(hit.tri_index >= 0)
    t = hit.tri_index[i, j]
    us, vs = u0 + (i + 0.5) * hit.cell_m, v0 + (j + 0.5) * hit.cell_m
    # Hit point: the triangles' planes along the flow.
    a0, a1, a2 = tris[t, 0], tris[t, 1], tris[t, 2]
    n = np.cross(a1 - a0, a2 - a0)
    base = us[:, None] * u + vs[:, None] * v
    denom = np.sum(n * f, axis=1)
    denom[denom == 0.0] = 1e-12
    z = np.sum(n * (a0 - base), axis=1) / denom
    station = (base + z[:, None] * f)[:, 0]
    dF = (hit.cos2[t][:, None] * f + hit.force_perp[t]) * hit.cell_m ** 2
    out = {}
    groups = {"hull": [k for k, name in enumerate(names) if not name.startswith("flap")],
              "flap_aft": [k for k, name in enumerate(names) if name == "flap_aft"],
              "flap_fwd": [k for k, name in enumerate(names) if name == "flap_fwd"]}
    part = ids[t]
    for g, members in groups.items():
        mask = np.isin(part, members)
        force = dF[mask].sum(axis=0)
        normal_share = dF[mask][:, 2]
        cp = float((station[mask] * normal_share).sum() / normal_share.sum()) if abs(normal_share.sum()) > 1e-9 else float("nan")
        out[g] = (force, cp)
    return out


def planar_core(row):
    return {k: float(row[k]) for k in row}


def read_trim(path):
    with open(path, encoding="utf-8") as fh:
        return [planar_core(r) for r in csv.DictReader(fh)]


def read_old_crossflow():
    with open(ROOT / "config" / "aero" / "ship_crossflow.csv", encoding="utf-8") as fh:
        rows = [ln.strip().split(",") for ln in fh if ln.strip() and not ln.startswith("#")][1:]
    return [float(r[0]) for r in rows], [float(r[1]) for r in rows]


def potential_term(alpha_deg):
    a = math.radians(alpha_deg)
    return abs(math.sin(2.0 * a) * math.cos(0.5 * a))


def effective_crossflow(normal_m2, alpha_deg, length_m, diameter_m):
    s2 = math.sin(math.radians(alpha_deg)) ** 2
    return (abs(normal_m2) / S_REF_M2 - potential_term(alpha_deg)) * S_REF_M2 / (length_m * diameter_m * s2)


def drag_lift(force, alpha_deg):
    a = math.radians(alpha_deg)
    u = np.array([math.cos(a), 0.0, -math.sin(a)])
    lift = np.array([math.sin(a), 0.0, math.cos(a)])
    return -float(force @ u), float(force @ lift)


def flight_rows():
    out = []
    for name, t0, t1 in FLIGHTS:
        speed, alt = load(str(ROOT / "tests" / "validation" / "telemetry" / name), "ship")
        st, sv = zip(*speed)
        at, ah = zip(*alt)
        t = t0
        while t <= t1:
            dv, vel = slope(st, sv, t, 3.0)
            fit = quadratic(at, ah, t, 20.0)
            t += 20.0
            if dv is None or fit is None or vel <= 50.0:
                continue
            h, hd, hdd = fit
            g = SHUTTLE_G0 * (6371e3 / (6371e3 + h)) ** 2
            sg = max(-1.0, min(1.0, hd / vel))
            cg = math.sqrt(1.0 - sg * sg)
            drag = -dv - g * sg
            if drag < 0.3:
                continue
            rate = (hdd - dv * sg) / (vel * cg)
            lift = vel * rate + (g - vel * vel / (6371e3 + h)) * cg - 2.0 * 7.2921159e-5 * vel * math.cos(math.radians(10.0))
            rho, sound = atmosphere(max(h, 0.0))
            q = 0.5 * rho * vel * vel
            out.append({"flight": name[:8], "t": t - 20.0, "h": h, "mach": vel / sound, "v": vel, "drag": drag, "lift_v": lift, "q": q,
                        "rho": rho})
    return out


def sutton_graves(rho, v, rn):
    return 1.7415e-4 * math.sqrt(rho / rn) * v ** 3  # as core constants::sutton_graves_earth_si


def nose_curvature_radius(alpha_deg, v):
    """Sutton-Graves radius at the windward stagnation point of the drawn ogive: the station where the meridian's slope
    equals 90 - alpha (surface normal against the flow); principal radii rho (meridian) and r / cos(theta) (hoop); the
    axisymmetric equivalent is their harmonic mean, 2 / (1 / R1 + 1 / R2) (Reshotko, cross-flow velocity gradient)."""
    hull = v.detail["hull"]
    R, Ln = 4.5, hull["nose_drawn_length_m"]
    rho = (R * R + Ln * Ln) / (2.0 * R)
    theta = math.radians(90.0 - alpha_deg)
    x = rho * math.sin(theta)  # from the barrel's end
    if x >= Ln:
        return None
    r = math.sqrt(rho * rho - x * x) + R - rho
    r = max(r, hull["nose_tip_radius_m"])
    r2 = r / math.cos(theta)
    return 2.0 / (1.0 / rho + 1.0 / r2), x, r, r2, rho


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--trim", default=str(geometry.OUT_DIR / "ship_trim.csv"))
    p.add_argument("--out", default=str(geometry.OUT_DIR / "ship_entry.md"))
    p.add_argument("--write-hull-cp", action="store_true", help="write config/aero/ship_hull_cp.csv and stop")
    a = p.parse_args()
    v = geometry.load_vehicle("starship_block3")
    if a.write_hull_cp:
        rows = []
        for alpha in HULL_CP_ALPHAS:
            tris, ids, names = mesh(v, alpha, 90.0, 90.0, flaps=False)
            rows.append(f"{alpha},{loads(tris, ids, names, alpha)['hull'][1]:.2f}")
        (ROOT / "config" / "aero" / "ship_hull_cp.csv").write_text(HULL_CP_HEADER + "\n".join(rows) + "\n", encoding="utf-8")
        return
    stage = v.stage("ship")
    L, D = stage["length_m"], stage["diameter_m"]
    trim = read_trim(a.trim)
    old_m, old_c = read_old_crossflow()
    lines = ["# Ship entry derivation", ""]

    lines += ["## Hull alone: mesh, first hit (no flaps in the mesh; force per q, reference S 63.617 m^2)", "",
              "| Mach | cp_max | alpha | CN m^2 | CA m^2 | CP station m | effective eta C_dc | old table | +- |", "|---|---|---|---|---|---|---|---|---|"]
    summary = {}
    for mach in NEWTONIAN_MACHS:
        cpm = newtonian.cp_max(mach)
        eff = []
        for alpha in (40, 50, 60, 70, 80, 90, 100, 120, 140):
            tris, ids, names = mesh(v, alpha, 90.0, 90.0, flaps=False)
            r = loads(tris, ids, names, alpha)["hull"]
            f = r[0] * cpm
            old = float(np.interp(mach, old_m, old_c))
            e = effective_crossflow(f[2], alpha, L, D)
            if 50 <= alpha <= 90:
                eff.append(e)
            lines.append(f"| {mach:g} | {cpm:.3f} | {alpha} | {abs(f[2]):.1f} | {f[0]:.1f} | {r[1]:.2f} | {e:.3f} | {old:.2f} | +-10 % |")
        summary[mach] = float(np.mean(eff))

    lines += ["", "## Hull below Mach 3: book values", "", "| Mach | eta C_dc | +- | source | old table |", "|---|---|---|---|---|"]
    for mach in (0.3, 0.8, 1.2, 2.0):
        c, e, src = crossflow_book(mach)
        summary[mach] = c
        lines.append(f"| {mach:g} | {c:.2f} | +-{e:.2f} | {src} | {float(np.interp(mach, old_m, old_c)):.2f} |")
    lines += ["", "Mean effective eta C_dc alpha 50-90 per Mach: " + ", ".join(f"{m:g}: {summary[m]:.3f}" for m in sorted(summary)), ""]

    lines += ["## Flaps at the core's trim pair (35 t, scheduled alpha): mesh against the core's flap law, per q", "",
              "| Mach | alpha | aft d | fwd d | core flaps normal m^2 | mesh flaps normal m^2 (cp_max) | core / mesh | mesh aft | mesh fwd | CP aft | CP fwd |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    totals = {}
    for r in trim:
        if r["propellant_kg"] != 35000.0 or r["scheduled"] != 1.0:
            continue
        mach, alpha, aft, fwd = r["mach"], r["alpha_deg"], r["aft_deg"], r["fwd_deg"]
        tris, ids, names = mesh(v, alpha, aft, fwd)
        m = loads(tris, ids, names, alpha)
        cpm = newtonian.cp_max(max(mach, 0.3))
        fa, fw = m["flap_aft"][0] * cpm, m["flap_fwd"][0] * cpm
        mesh_flap = abs((fa + fw)[2])
        core_flap = abs(r["flaps_normal_m2"])
        lines.append(f"| {mach:g} | {alpha:.1f} | {aft:.1f} | {fwd:.1f} | {core_flap:.1f} | {mesh_flap:.1f} | {core_flap / max(mesh_flap, 1e-3):.2f} | {abs(fa[2]):.1f} | {abs(fw[2]):.1f} | {m['flap_aft'][1]:.2f} | {m['flap_fwd'][1]:.2f} |")
        hull = m["hull"][0] * cpm
        totals[mach] = (alpha, hull + fa + fw, r, m["hull"][1])

    lines += ["", "## Total at the scheduled alpha, 35 t: core against mesh above Mach 3", "",
              "| Mach | alpha | core CdA | core ClA | core L/D | core beta 290 t | mesh CdA | mesh ClA | mesh L/D | mesh hull CP | mesh+hull-only effective | ", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for mach in sorted(totals):
        alpha, f, r, cp = totals[mach]
        core_force = np.array([r["hull_axial_m2"] + r["flaps_axial_m2"], 0.0, r["hull_normal_m2"] + r["flaps_normal_m2"]])
        cd, cl = drag_lift(core_force, alpha)
        cdm, clm = drag_lift(f, alpha)
        mesh_text = f"{cdm:.0f} | {clm:.0f} | {clm / cdm:.2f} | {cp:.2f}" if mach >= 3.0 else "- | - | - | -"
        lines.append(f"| {mach:g} | {alpha:.1f} | {cd:.0f} | {cl:.0f} | {cl / cd:.2f} | {SHIP_ENTRY_MASS_KG / cd:.0f} | {mesh_text} | {effective_crossflow(f[2], alpha, L, D):.3f} |")

    flights = flight_rows()
    lines += ["", f"## Flights: entry_estimate, per Mach band (mass {SHIP_ENTRY_MASS_KG / 1e3:.0f} t for C_D A)", "",
              "| Flight | Mach band | n | D/m m/s^2 | Lv/D | beta kg/m^2 | CdA m^2 (290 t) |", "|---|---|---|---|---|---|---|"]
    bands = [(1.5, 3), (3, 6), (6, 10), (10, 16), (16, 22), (22, 30)]
    for flight in sorted({r["flight"] for r in flights}):
        for lo, hi in bands:
            rs = [r for r in flights if r["flight"] == flight and lo <= r["mach"] < hi and r["q"] > 100.0]
            if not rs:
                continue
            drag = float(np.mean([r["drag"] for r in rs]))
            ld = float(np.mean([r["lift_v"] / r["drag"] for r in rs]))
            beta = float(np.mean([r["q"] / r["drag"] for r in rs]))
            lines.append(f"| {flight} | {lo}-{hi} | {len(rs)} | {drag:.2f} | {ld:.2f} | {beta:.0f} | {SHIP_ENTRY_MASS_KG / beta:.0f} |")

    lines += ["", "## Nose radius: Sutton-Graves at the drawn ogive's windward stagnation point", "",
              "| alpha | station from barrel end m | hoop radius m | R2 m | rho m | R_eff m |", "|---|---|---|---|---|---|"]
    for alpha in (50, 52, 54, 55, 56, 58, 60, 65, 70, 80):
        r = nose_curvature_radius(alpha, v)
        if r is None:
            lines.append(f"| {alpha} | past the tip | | | | |")
        else:
            lines.append(f"| {alpha} | {r[1]:.2f} | {r[2]:.2f} | {r[3]:.2f} | {r[4]:.1f} | {r[0]:.2f} |")
    hot = max(flights, key=lambda x: sutton_graves(x["rho"], x["v"], 6.0))
    lines += ["", "Hottest reading on the flights (Sutton-Graves against radius): "
              f"{hot['flight']} t {hot['t']:.0f} h {hot['h'] / 1e3:.1f} km v {hot['v']:.0f} m/s: "
              + ", ".join(f"R {r:g} m: {sutton_graves(hot['rho'], hot['v'], r) / 1e3:.0f} kW/m^2" for r in (0.5, 1.2, 2.0, 3.0, 4.5, 6.0))]

    lines += ["", "## Flap exposure: mesh first hit against `flap_exposed_fraction`", "",
              "Share of a flat plate's c^2 A that the drawn hull lets through, per flap kind.", "",
              "| kind | alpha | " + " | ".join(f"d={d}" for d in (20, 30, 40, 60, 75, 90)) + " |", "|---|---|---|---|---|---|---|---|"]
    hull_tris = list(v.hulls["ship"].values())
    n_hull = sum(len(h) for h in hull_tris)
    for kind in ("aft", "fwd"):
        e = flap_entries(v, kind)[0]
        for alpha in (50, 60, 70, 90):
            fl = (-math.cos(math.radians(alpha)), 0.0, math.sin(math.radians(alpha)))
            row = []
            for d in (20, 30, 40, 60, 75, 90):
                panel = v.flap_triangles(e, d)["panel"]
                alone = newtonian.first_hit(panel, fl, CELL_M).k_total(1.0)
                tris = np.concatenate(hull_tris + [panel])
                ids = np.concatenate([np.zeros(n_hull, dtype=np.int64), np.ones(len(panel), dtype=np.int64)])
                exposed = newtonian.first_hit(tris, fl, CELL_M, ids, ["hull", "panel"]).k_by_part(1.0).get("panel", 0.0)
                row.append(f"{exposed / max(alone, 1e-9):.2f}")
            lines.append(f"| {kind} | {alpha} | " + " | ".join(row) + " |")

    Path(a.out).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
