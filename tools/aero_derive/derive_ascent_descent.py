"""Ascent and booster descent drag derived from the built models, compared with the tables and the flights.

    python tools/aero_derive/derive_ascent_descent.py

Writes to screenshots/vehicle-physics/: drag_compare.csv, telemetry_bins.csv, mesh_factors.csv, the plots
drag_ascent.png / drag_descent.png / beta_descent.png, and tables.md (the tables of docs/subsystems/aero-derivation.md).
Nothing here is fitted. The stack's table is this tool's output (Task 2b, user's OK 2026-11-01), written on request with
--write-stack-table; the descent tables are only compared.

Flaps stand at d = 90 in every ascent and stack case (user fact 2026-11-01, flight 14 frame at T+49 s: radial, their
rest pose, edge-on to the axial flow). The booster alone in descent has no flaps.
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import buildup  # noqa: E402
import exposure  # noqa: E402
import geometry  # noqa: E402
import newtonian  # noqa: E402

ROOT = geometry.ROOT
OUT = geometry.OUT_DIR
TELEMETRY = ROOT / "tests" / "validation" / "telemetry"
MACHS = [0.3, 0.6, 0.8, 0.95, 1.05, 1.2, 1.5, 2.0, 3.0, 5.0, 8.0, 15.0, 25.0]
ASCENT_MACHS = sorted(MACHS + [4.0])  # the stack table's grid needs this one
TABLE_MAX_MACH = 5.0  # last row flown in continuum by the stack (see aero-derivation.md); the curve holds it beyond
BAND_PARTS = ("booster/top_truss", "booster/hot_stage")  # stand in the hot-stage band: the cavity term is the band's one term
A_REF_M2 = math.pi / 4.0 * 9.0 ** 2  # the tables' reference area
DESCENT_MASS_KG = 261000.0  # 210 t dry + 51 t propellant from the gauge (aero/super_heavy_descent.csv)
KNUDSEN_CONTINUUM = 0.01  # the core's rarefaction blend starts here (slender_body_aero.cpp)
STACK_LENGTH_M = 124.4
CELL_M = 0.02  # ray spacing of the whole-vehicle first hit: 2 samples across the 0.04 m features
ASCENT_TABLE_ERROR = 0.15  # [E] the ascent table's own band: generic trend, none published
DESCENT_IDENTIFIED_ERROR = 0.125  # the table header: bins scatter +-10-15 %
DESCENT_UNMEASURED_ERROR = 0.20  # [E] subsonic trend, and the held value above Mach 3.9
DESCENT_IDENTIFIED_MACH = (1.1, 3.9)
TELEMETRY_MIN_DECEL = 3.0  # m/s^2: below it the 1 km altitude rounding dominates D/m
SUPERSONIC_MESH_FROM_MACH = 1.5  # plan Task 2: subsonic and transonic from Hoerner, above that mesh + base
FLIGHTS = {13: ("flight13_2026-07-24.csv", 340.0, 386.0), 14: ("flight14_2026-09-28.csv", 340.0, 396.0)}  # to engine relight
FLAP_REF_CHORD_M = {"aft": 12.0, "fwd": 5.784}  # hinge lengths, detail file
ASCENT_END_S = 520.0  # flight 14: ship burn still running, before any return data
STAGING_T_S = 160.0  # flight 14 engine icons: 31 -> 11 engines at T+163 s; the stack's part ends about here


def read_table(path: Path) -> dict:
    with open(path) as f:
        rows = list(csv.DictReader(line for line in f if not line.startswith("#")))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def table_at(table: dict, column: str, mach: float) -> float:
    """Linear in Mach, flat beyond the ends: as `interpolate` in slender_body_aero.cpp."""
    return float(np.interp(mach, table["mach"], table[column]))


def continuum_altitude_m(length_m: float) -> float:
    """Highest altitude with Kn <= 0.01 on this length (bisection): the core's continuum limit."""
    low, high = 0.0, 200000.0
    for _ in range(50):
        mid = 0.5 * (low + high)
        low, high = (mid, high) if buildup.air(mid)["mfp"] / length_m <= KNUDSEN_CONTINUUM else (low, mid)
    return low


def quad(*pairs) -> tuple[float, float]:
    """Sum of (value, relative error) with independent errors: (sum, absolute error)."""
    return sum(v for v, _ in pairs), math.sqrt(sum((v * e) ** 2 for v, e in pairs))


# ---------------------------------------------------------------------- flight history
def ascent_history(mach_targets) -> dict:
    """{Mach: altitude at first reaching it} on flight 14: speed and altitude dials through local least squares."""
    sys.path.insert(0, str(ROOT / "tools" / "telemetry"))
    from drag_estimate import load, slope
    speed, alt = load(TELEMETRY / FLIGHTS[14][0], "ship")
    st, sv = zip(*speed)
    at, ah = zip(*alt)
    times = np.arange(5.0, ASCENT_END_S, 1.0)
    out = {}
    last = None
    for t in times:
        _, v = slope(st, sv, t, 2.0)
        _, h = slope(at, ah, t, 3.0)
        if v is None or h is None or v <= 0.0:
            continue
        a = buildup.air(h)["a"]
        last = (h, v / a, float(t))
        for m in mach_targets:
            if m not in out and v / a >= m:
                out[m] = (h, float(t), True)
    for m in mach_targets:
        if m not in out:
            out[m] = (last[0], last[2], False)  # the flight reaches its orbit below this Mach number
    return out


def telemetry_bins() -> list:
    """Raw drag_estimate output of both flights' descents: (flight, t, h, Mach, D/m, beta, C_D at 261 t)."""
    rows = []
    for flight, (name, t0, t1) in FLIGHTS.items():
        out = subprocess.run([sys.executable, str(ROOT / "tools" / "telemetry" / "drag_estimate.py"), str(TELEMETRY / name),
                              "--from", str(t0), "--to", str(t1), "--mass-t", str(DESCENT_MASS_KG / 1e3), "--step", "2",
                              "--vertical-fallback"], capture_output=True, text=True, check=True).stdout
        for line in out.splitlines()[1:]:
            p = line.split()
            if len(p) == 8 and p[6] != "nan" and float(p[5]) >= TELEMETRY_MIN_DECEL:
                rows.append({"flight": flight, "t_s": float(p[0]), "h_km": float(p[1]), "mach": float(p[3]),
                             "decel_mps2": float(p[5]), "beta": float(p[6]), "cd": float(p[7])})
    return rows


def bins_at_machs(rows) -> dict:
    """Bins per listed Mach number: edges at the geometric means of the neighbours."""
    edges = [math.sqrt(a * b) for a, b in zip(MACHS, MACHS[1:])]
    lows, highs = [0.0] + edges, edges + [1e9]
    out = {}
    for m, lo, hi in zip(MACHS, lows, highs):
        sel = [r for r in rows if lo <= r["mach"] < hi]
        if sel:
            betas = [r["beta"] for r in sel]
            cds = [r["cd"] for r in sel]
            out[m] = {"n": len(sel), "beta": statistics.mean(betas), "beta_sd": statistics.stdev(betas) if len(sel) > 1 else float("nan"),
                      "cd": statistics.mean(cds), "cd_sd": statistics.stdev(cds) if len(sel) > 1 else float("nan"),
                      "h_km": statistics.mean(r["h_km"] for r in sel)}
    return out


def ascent_drag_loss(rows) -> str:
    """What the flight could see: drag's velocity loss to staging with each curve, along flight 14's q(t) (power on).

    Mass [D]: vehicle report's ignition mass 5715 t, falling linearly to the stack at MECO (3425 t burnt: 3650 t less the
    225 t reserve). Thrust: 33 x 280 tf (32 lit from ~2/3 of the ascent is neglected)."""
    sys.path.insert(0, str(ROOT / "tools" / "telemetry"))
    from drag_estimate import load, slope
    m0, burnt, thrust = 5715e3, 3425e3, 33 * 280e3 * 9.80665
    speed, alt = load(TELEMETRY / FLIGHTS[14][0], "ship")
    st, sv = zip(*speed)
    at, ah = zip(*alt)
    sel = [r for r in rows if r["case"] == "ascent_stack" and r["column"] == "on"]
    mx, cd_d, cd_k = [r["mach"] for r in sel], [r["derived"] for r in sel], [r["known"] for r in sel]
    loss_d = loss_k = peak = 0.0
    share = None
    for t in np.arange(5.0, STAGING_T_S, 1.0):
        _, v = slope(st, sv, t, 2.0)
        _, h = slope(at, ah, t, 3.0)
        if v is None or h is None:
            continue
        state = buildup.air(h)
        q, mach = 0.5 * state["rho"] * v * v, v / state["a"]
        m = m0 - burnt * t / STAGING_T_S
        derived, known = float(np.interp(mach, mx, cd_d)), float(np.interp(mach, mx, cd_k))
        loss_d += derived * q * A_REF_M2 / m
        loss_k += known * q * A_REF_M2 / m
        if q > peak:
            peak = q
            share = (t, mach, h, derived * q * A_REF_M2 / thrust, known * q * A_REF_M2 / thrust, (derived - known) * q * A_REF_M2 / m)
    return (f"- Flight 14, stack to T+{STAGING_T_S:.0f} s: peak q {peak / 1e3:.1f} kPa at T+{share[0]:.0f} s (Mach {share[1]:.2f}, {share[2] / 1e3:.1f} km). "
            f"Drag / thrust there: derived {share[3] * 100:.1f} %, table {share[4] * 100:.1f} %; their difference is {share[5]:.2f} m/s^2 of acceleration. "
            f"Velocity lost to drag: derived {loss_d:.0f} m/s, table {loss_k:.0f} m/s.")


# ---------------------------------------------------------------------- mesh factors
class Mesh:
    """k factors per case (per unit cp_max, on the reference area)."""

    def __init__(self, vehicle):
        self.v = vehicle
        self.cache = {}

    def exposed(self, stages, flap_deg=90.0):
        """Nose first, with the wake range (`exposure`): the samples that load."""
        key = ("exposed", stages, flap_deg)
        if key not in self.cache:
            tris, (names, ids) = self.v.triangles(stages, flap_deg, 0.0, False, True, True)
            self.cache[key] = exposure.exposed_hits(tris, ids, names, self.v.outer_radius_m, CELL_M)
        return self.cache[key]

    def hit(self, stages, flow_x, flap_deg=90.0, engines=False, fins=True, flaps=True):
        key = (stages, flow_x, flap_deg, engines, fins, flaps)
        if key not in self.cache:
            tris, (names, ids) = self.v.triangles(stages, flap_deg, 0.0, engines, flaps, fins)
            self.cache[key] = newtonian.first_hit(tris, (flow_x, 0.0, 0.0), CELL_M, ids, names).k_by_part(A_REF_M2)
        return self.cache[key]

    def fin_k_per_gross(self, k_parts: dict, face: str) -> float:
        """Mesh solid parts plus the walls the mesh cannot show, per gross area of one fin (the mean of the three)."""
        count = len(self.v.stage("booster")["grid_fins"]["azimuths_deg"])
        gross = self.v.grid_fin_gross_area_m2()
        solid_k = sum(v for k, v in k_parts.items() if k.startswith("fin/")) * A_REF_M2 / (count * gross)
        solid_area = sum(self.v.grid_fin_blocked_area_m2().values())
        walls = (gross - solid_area) / gross * self.v.grid_fin_wall_share() * self.v.grid_fin_wall_edge_cos2(face)
        return solid_k + walls


def split(k_parts: dict) -> tuple[dict, dict, dict]:
    hull = {k: v for k, v in k_parts.items() if k.endswith("/hull")}
    fins = {k: v for k, v in k_parts.items() if k.startswith("fin/")}
    flaps = {k: v for k, v in k_parts.items() if k.startswith("flap/")}
    return hull, fins, flaps


# ---------------------------------------------------------------------- ascent
def ascent_rows(v, mesh, history) -> list:
    rows = []
    fin = v.detail["grid_fin"]
    open_fraction = v.grid_fin_open_fraction()
    gross = v.grid_fin_gross_area_m2()
    nose = v.detail["hull"]["nose_drawn_length_m"] / 9.0
    nose_config = v.stage("ship")["aero"]["nose_length_m"] / 9.0
    old_table = read_table(ROOT / "config" / "aero" / "ship_ascent.csv")  # the stack's table before Task 2b, same values
    exit_booster = 33 * math.pi / 4.0 * v.manifest["engines"]["raptor3"]["exit_diameter_m"] ** 2
    exit_ship = 3 * math.pi / 4.0 * (v.manifest["engines"]["raptor3"]["exit_diameter_m"] ** 2 + v.manifest["engines"]["raptor_vacuum3"]["exit_diameter_m"] ** 2)
    stack_load = mesh.exposed(("booster", "ship"))
    stack_flat_load = mesh.exposed(("booster", "ship"), flap_deg=0.0)
    ship_load = mesh.exposed(("ship",))
    k_stack = stack_load.k_by_part(A_REF_M2)
    k_ship = ship_load.k_by_part(A_REF_M2)
    fin_count = len(v.stage("booster")["grid_fins"]["azimuths_deg"])
    wetted_b, wetted_s = v.hull_wetted_area_m2("booster"), v.hull_wetted_area_m2("ship")
    fin_k = mesh.fin_k_per_gross(k_stack, "fwd")
    gap_area = 2.0 * math.pi * 4.5 * v.hot_stage_open_height_m()

    def credited(load, mach, h):
        """Per part: loaded area / A_ref with each sample at its share of q in the layer at its station."""
        state = buildup.air(h)
        unit_reynolds = state["rho"] * mach * state["a"] / state["mu"]

        def share(x_m, height_m):
            run = np.maximum(STACK_LENGTH_M - x_m, 0.1)
            return buildup.boundary_layer_q_share(height_m, buildup.turbulent_layer_thickness_m(run, unit_reynolds * run))
        return load.credited_k_by_part(A_REF_M2, share)

    def is_other(name):
        return not (name.endswith("/hull") or name.startswith(("fin/", "flap/")) or name in BAND_PARTS)

    def flap_terms(mach, h, k_parts):
        area = {"aft": v.flap_area_m2("aft"), "fwd": v.flap_area_m2("fwd")}
        friction = [buildup.skin_friction_cd(2.0 * 2.0 * area[kind], FLAP_REF_CHORD_M[kind], mach, h, A_REF_M2) for kind in area]
        loaded = sum(x for k, x in k_parts.items() if k.startswith("flap/")) * A_REF_M2
        return friction, buildup.protuberance_cd(mach, loaded, A_REF_M2)

    for case, machs, k_parts, load, length, wet, exit_area, hull_key in (
            ("ascent_stack", ASCENT_MACHS, k_stack, stack_load, STACK_LENGTH_M, wetted_b + wetted_s, exit_booster, "ship/hull"),
            ("ascent_ship", [m for m in ASCENT_MACHS if m >= 5.0], k_ship, ship_load, v.stage("ship")["length_m"], wetted_s, exit_ship, "ship/hull")):
        for mach in machs:
            h_flown, t_s, reached = history[mach]
            h = min(h_flown, continuum_altitude_m(length))  # beyond it the flown altitude is not a continuum state
            state = buildup.air(h)
            knudsen = state["mfp"] / length
            extrapolated = h < h_flown
            friction = buildup.skin_friction_cd(wet, length, mach, h, A_REF_M2)
            nose_cd = buildup.nose_pressure_cd(mach, nose)
            flap_fric, flap_edge = flap_terms(mach, h, k_parts)
            credit = credited(load, mach, h)
            other_parts = {k: x for k, x in credit.items() if is_other(k)}
            other_plain = sum(x for k, x in k_parts.items() if is_other(k))
            other = sum(other_parts.values())
            others = buildup.protuberance_cd(mach, other * A_REF_M2, A_REF_M2)
            credit_flat = credited(stack_flat_load, mach, h) if case == "ascent_stack" else {}
            terms = [friction, nose_cd, *flap_fric, flap_edge, others]
            fin_cd = (0.0, 0.0)
            if case == "ascent_stack":
                ca, err = buildup.grid_fin_ca(mach, open_fraction, h, fin_k, fin["thickness_m"], fin["cell_pitch_m"], fin["wall_m"],
                                              v.grid_fin_mean_depth_m("fwd"))
                fin_cd = (ca * fin_count * gross / A_REF_M2, err)
                terms += [fin_cd, buildup.cavity_cd(gap_area, A_REF_M2)]
            for powered, column in ((True, "cd_axial_power_on"), (False, "cd_axial_power_off")):
                base = buildup.base_cd(mach, A_REF_M2, powered, A_REF_M2, exit_area)
                total, err = quad(*terms, base)
                newton = None
                if mach >= 3.0:
                    n_nose = newtonian.cp_max(mach) * k_parts.get(hull_key, 0.0)
                    n_rest = sum(t[0] for t in terms if t is not nose_cd) + base[0]
                    newton = n_nose + n_rest
                known = table_at(old_table, column, mach)
                rows.append(row(case, mach, "on" if powered else "off", total, err, known, ASCENT_TABLE_ERROR * known, h, t_s, reached, extrapolated,
                                clean=friction[0] + nose_cd[0] + base[0],
                                friction=friction[0], nose=nose_cd[0], base=base[0], flaps=sum(t[0] for t in flap_fric) + flap_edge[0],
                                fins=fin_cd[0], fin_ca_nose=fin_cd[0] * A_REF_M2 / (fin_count * gross), other=others[0],
                                other_uncredited=buildup.protuberance_cd(mach, other_plain * A_REF_M2, A_REF_M2)[0],
                                other_parts={k: round(x, 5) for k, x in other_parts.items()},
                                gap=terms[-1][0] if case == "ascent_stack" else 0.0,
                                newtonian=newton,
                                nose_17m=total - nose_cd[0] + buildup.nose_pressure_cd(mach, nose_config)[0],
                                flaps_d0=(total - flap_edge[0] - others[0] + buildup.protuberance_cd(mach, sum(
                                    x if k.startswith("flap/") else credit_flat[k]
                                    for k, x in stack_flat_load.k_by_part(A_REF_M2).items() if k.startswith("flap/") or is_other(k)) * A_REF_M2, A_REF_M2)[0]) if case == "ascent_stack" else None))
    return rows


def row(case, mach, column, derived, err, known, known_err, h, t_s, reached, extrapolated, **extra) -> dict:
    combined = math.hypot(err, known_err)
    verdict = "within" if abs(derived - known) <= combined else "outside"
    if extrapolated:
        verdict += " (not flown in continuum)"
    return {"case": case, "mach": mach, "column": column, "derived": derived, "derived_err": err, "known": known, "known_err": known_err,
            "ratio": derived / known if known else None, "diff": derived - known, "combined_err": combined, "verdict": verdict,
            "altitude_km": h / 1e3, "t_s": t_s, "reached_in_flight": reached, "continuum_extrapolated": extrapolated, **extra}


# ---------------------------------------------------------------------- descent
def descent_rows(v, mesh, bins) -> list:
    """Tail first, alone, fins at 0. Body against the body table; body + fins against body + fin table and the flights."""
    fin = v.detail["grid_fin"]
    open_fraction = v.grid_fin_open_fraction()
    gross = v.grid_fin_gross_area_m2()
    config_fin_area = v.stage("booster")["grid_fins"]["area_m2"]
    body_table = read_table(ROOT / "config" / "aero" / "super_heavy_descent.csv")
    fin_table = read_table(ROOT / "config" / "aero" / "grid_fin.csv")
    k_parts = mesh.hit(("booster",), 1.0)
    k_no_fins = mesh.hit(("booster",), 1.0, fins=False)
    k_bells = mesh.hit(("booster",), 1.0, engines=True)
    hull_k = k_parts["booster/hull"]
    prot_k = sum(x for k, x in k_parts.items() if not (k.startswith("fin/") or k == "booster/hull"))
    fin_k = mesh.fin_k_per_gross(k_parts, "aft")
    fin_k_nose = mesh.fin_k_per_gross(mesh.exposed(("booster", "ship")).k_by_part(A_REF_M2), "fwd")
    fin_count = len(v.stage("booster")["grid_fins"]["azimuths_deg"])
    k_bell_body = sum(x for k, x in k_bells.items() if not k.startswith("fin/"))
    wet = v.hull_wetted_area_m2("booster")
    length = v.wetted_end_station_m("booster")
    rows = []
    for mach in MACHS:
        # altitude where the flights were at this Mach number; beyond their range: the table's held / unflown ends
        h = bins[mach]["h_km"] * 1e3 if mach in bins else (1000.0 if mach < 1.0 else 30000.0)
        friction = buildup.skin_friction_cd(wet, length, mach, h, A_REF_M2)
        base = buildup.base_cd(mach, A_REF_M2, False, A_REF_M2)
        cp = newtonian.cp_max(mach)
        if mach >= SUPERSONIC_MESH_FROM_MACH:
            method = "Newtonian mesh + base + friction"
            face_cd = (cp * (hull_k + prot_k), 0.20 if mach < 5.0 else 0.15)
            band = "supersonic"
        else:
            method = "Hoerner blunt face + base + friction"
            face_cd = buildup.blunt_face_cd(mach)  # the rim's bubble covers the skirt and strake ends: no protrusion term
            band = "subsonic" if mach < 0.8 else "transonic"
        body, body_err = quad(face_cd, friction, base)
        hull_only = cp * hull_k + friction[0] + base[0]
        hoerner_face = buildup.blunt_face_cd(mach)[0] + friction[0] + base[0]
        with_bells = cp * k_bell_body + friction[0] + base[0]
        ca, ca_err = buildup.grid_fin_ca(mach, open_fraction, h, fin_k, fin["thickness_m"], fin["cell_pitch_m"], fin["wall_m"],
                                         v.grid_fin_mean_depth_m("aft"))
        ca_nose = buildup.grid_fin_ca(mach, open_fraction, h, fin_k_nose, fin["thickness_m"], fin["cell_pitch_m"], fin["wall_m"],
                                      v.grid_fin_mean_depth_m("fwd"))[0]
        table_ca = table_at(fin_table, "ca_axial", mach)
        fins_drawn = ca * fin_count * gross / A_REF_M2
        fins_config = ca * fin_count * config_fin_area / A_REF_M2
        known_body = table_at(body_table, "cd_axial_power_off", mach)
        known_fins = table_ca * fin_count * config_fin_area / A_REF_M2
        identified = DESCENT_IDENTIFIED_MACH[0] <= mach <= DESCENT_IDENTIFIED_MACH[1]
        known_rel = DESCENT_IDENTIFIED_ERROR if identified else DESCENT_UNMEASURED_ERROR
        common = dict(method=method, band=band, friction=friction[0], base=base[0], face=face_cd[0], hull_only_body=hull_only,
                      hoerner_face_body=hoerner_face, with_bells_body=with_bells, k_hull=hull_k, k_protrusions=prot_k, k_fin_gross=fin_k)
        rows.append(row("descent_body", mach, "off", body, body_err, known_body, known_rel * known_body, h, None, True, False, **common))
        total_err = math.hypot(body_err, ca_err * fins_drawn)
        for area_name, fins_value in (("drawn_12.63", fins_drawn), ("config_13.5", fins_config)):
            total = body + fins_value
            known_total = known_body + known_fins
            flight = bins.get(mach)
            flight_cd = flight["cd"] if flight else None
            r = row(f"descent_total_{area_name}", mach, "off", total, total_err, known_total, known_rel * known_total, h, None, True, False,
                    fin_cd=fins_value, fin_ca_gross=ca, fin_ca_nose=ca_nose, fin_ca_err=ca_err, table_ca=table_ca, table_fin_cd=known_fins,
                    beta_derived=DESCENT_MASS_KG / (total * A_REF_M2), beta_table=DESCENT_MASS_KG / (known_total * A_REF_M2),
                    beta_flight=flight["beta"] if flight else None, beta_flight_sd=flight["beta_sd"] if flight else None,
                    cd_flight=flight_cd, cd_flight_sd=flight["cd_sd"] if flight else None, n_flight=flight["n"] if flight else 0,
                    **common)
            if flight:
                band_cd = math.hypot(total_err, flight["cd_sd"] if not math.isnan(flight["cd_sd"]) else DESCENT_IDENTIFIED_ERROR * flight_cd)
                r["flight_verdict"] = "within" if abs(total - flight_cd) <= band_cd else "outside"
            else:
                r["flight_verdict"] = "no flight data"
            rows.append(r)
    return rows, {"k_parts": k_parts, "k_no_fins": k_no_fins, "k_bells": k_bells}


# ---------------------------------------------------------------------- output
def plot_case(rows, case, column, path, title, ylabel="axial drag coefficient per q x 63.6 m^2"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    selected = [r for r in rows if r["case"] == case and r["column"] == column]
    x = [r["mach"] for r in selected]
    fig, ax = plt.subplots(figsize=(8, 4.5), facecolor="#fcfcfb")
    ax.set_facecolor("#fcfcfb")
    d = np.array([r["derived"] for r in selected])
    e = np.array([r["derived_err"] for r in selected])
    ax.fill_between(x, d - e, d + e, color="#2a78d6", alpha=0.15, lw=0)
    ax.plot(x, d, color="#2a78d6", lw=2, marker="o", ms=4, label="derived (band: error)")
    ax.plot(x, [r["known"] for r in selected], color="#eb6834", lw=2, marker="s", ms=4, label="table")
    flight = [(r["mach"], r["cd_flight"], r["cd_flight_sd"]) for r in selected if r.get("cd_flight")]
    if flight:
        fx, fy, fs = zip(*flight)
        ax.errorbar(fx, fy, yerr=[0.0 if math.isnan(s) else s for s in fs], color="#0b0b0b", ls="none", marker="D", ms=5, capsize=3,
                    label="flights 13, 14 at 261 t (mean, sd)")
    ax.set_xscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels([f"{m:g}" for m in x], fontsize=8, rotation=60)
    ax.minorticks_off()
    ax.set_ylim(bottom=0)
    ax.set_xlabel("Mach")
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontsize=11)
    ax.grid(axis="y", color="#e4e3df", lw=0.8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.legend(frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_beta(rows, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    series = {name: [r for r in rows if r["case"] == name] for name in ("descent_total_drawn_12.63", "descent_total_config_13.5")}
    fig, ax = plt.subplots(figsize=(8, 4.5), facecolor="#fcfcfb")
    ax.set_facecolor("#fcfcfb")
    sel = series["descent_total_drawn_12.63"]
    x = [r["mach"] for r in sel]
    ax.plot(x, [r["beta_derived"] for r in sel], color="#2a78d6", lw=2, marker="o", ms=4, label="derived, fins 12.63 m^2")
    ax.plot(x, [r["beta_derived"] for r in series["descent_total_config_13.5"]], color="#1baf7a", lw=2, marker="^", ms=4, label="derived, fins 13.5 m^2")
    ax.plot(x, [r["beta_table"] for r in sel], color="#eb6834", lw=2, marker="s", ms=4, label="tables (body + fins 13.5 m^2)")
    fl = [(r["mach"], r["beta_flight"], r["beta_flight_sd"]) for r in sel if r["beta_flight"]]
    fx, fy, fs = zip(*fl)
    ax.errorbar(fx, fy, yerr=[0.0 if math.isnan(s) else s for s in fs], color="#0b0b0b", ls="none", marker="D", ms=5, capsize=3, label="flights 13, 14 (mean, sd)")
    ax.set_xscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels([f"{m:g}" for m in x], fontsize=8, rotation=60)
    ax.minorticks_off()
    ax.set_ylim(bottom=0)
    ax.set_xlabel("Mach")
    ax.set_ylabel("ballistic coefficient [kg/m^2] at 261 t")
    ax.set_title("Booster descent: ballistic coefficient", loc="left", fontsize=11)
    ax.grid(axis="y", color="#e4e3df", lw=0.8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.legend(frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fmt(value, digits=3):
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "-"
    return f"{value:.{digits}f}"


def markdown(rows, bins, factors, v, mesh) -> str:
    lines = []
    for case, head in (("ascent_stack", "Ascent, stack"), ("ascent_ship", "Ascent, ship alone")):
        lines += [f"### {head}", "",
                  "| Mach | h km | col | friction | nose | base | clean (sum) | flaps | fins | other | other, no credit | gap | derived | +- | old table | derived/old | verdict | Newtonian | nose 17 m | flaps d=0 |",
                  "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in rows:
            if r["case"] == case:
                lines.append(f"| {r['mach']:g} | {r['altitude_km']:.1f}{'*' if r['continuum_extrapolated'] else ''} | {r['column']} | {fmt(r['friction'])} | {fmt(r['nose'])} | {fmt(r['base'])} | {fmt(r['clean'])} | {fmt(r['flaps'])} | {fmt(r['fins'])} | {fmt(r['other'])} | {fmt(r['other_uncredited'])} | {fmt(r['gap'])} | {fmt(r['derived'])} | {fmt(r['derived_err'])} | {fmt(r['known'])} | {fmt(r['ratio'], 2)} | {r['verdict']} | {fmt(r['newtonian'])} | {fmt(r['nose_17m'])} | {fmt(r['flaps_d0'])} |")
        lines.append("")
    lines += ["### Booster descent, body without fins", "",
              "| Mach | band | face | friction | base | derived | +- | table | derived/table | verdict | hull face only | Hoerner face | with bells |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if r["case"] == "descent_body":
            lines.append(f"| {r['mach']:g} | {r['band']} | {fmt(r['face'])} | {fmt(r['friction'])} | {fmt(r['base'])} | {fmt(r['derived'])} | {fmt(r['derived_err'])} | {fmt(r['known'])} | {fmt(r['ratio'], 2)} | {r['verdict']} | {fmt(r['hull_only_body'])} | {fmt(r['hoerner_face_body'])} | {fmt(r['with_bells_body'])} |")
    lines += ["", "### Booster descent, body + fins", "",
              "| Mach | fin CA tail first (gross) | fin CA nose first (gross) | +- | fin CA table | total, 12.63 | total, 13.5 | +- | table total | verdict (12.63 / 13.5) | flights C_D at 261 t (sd, n) | verdict against flights (12.63 / 13.5) | beta 12.63 | beta 13.5 | beta tables | beta flights (sd) |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    drawn = [r for r in rows if r["case"] == "descent_total_drawn_12.63"]
    config = [r for r in rows if r["case"] == "descent_total_config_13.5"]
    for a, b in zip(drawn, config):
        flights = f"{a['cd_flight']:.2f} ({fmt(a['cd_flight_sd'], 2)}, {a['n_flight']})" if a["cd_flight"] else "-"
        beta_f = f"{a['beta_flight']:.0f} ({fmt(a['beta_flight_sd'], 0)})" if a["beta_flight"] else "-"
        lines.append(f"| {a['mach']:g} | {fmt(a['fin_ca_gross'])} | {fmt(a['fin_ca_nose'])} | {a['fin_ca_err'] * 100:.0f} % | {fmt(a['table_ca'])} | {fmt(a['derived'])} | {fmt(b['derived'])} | {fmt(a['derived_err'])} | {fmt(a['known'])} | {a['verdict']} / {b['verdict']} | {flights} | {a['flight_verdict']} / {b['flight_verdict']} | {a['beta_derived']:.0f} | {b['beta_derived']:.0f} | {a['beta_table']:.0f} | {beta_f} |")
    lines += ["", "### Mesh factors k (per unit cp_max, on 63.6 m^2)", ""]
    def grouped(k):
        out = {}
        for name, value in k.items():
            parts = name.split("/")
            key = parts[0] if parts[0] in ("fin", "flap", "engine") else name
            out[key] = out.get(key, 0.0) + value
        return ", ".join(f"{a} {b:.4f}" for a, b in sorted(out.items(), key=lambda x: -x[1]))
    lines.append(f"- Tail first, no bells: {grouped(factors['k_parts'])}")
    lines.append(f"- Tail first, drawn bells: {grouped(factors['k_bells'])}")
    stack = mesh.exposed(("booster", "ship")).k_by_part(A_REF_M2)
    lines.append(f"- Stack nose first, flaps 90, exposed (wake range): {grouped(stack)}")
    lines.append(f"- Same, plain first hit (not used): {grouped(mesh.hit(('booster', 'ship'), -1.0))}")
    lines.append(f"- Stack nose first, flaps 0, exposed: {grouped(mesh.exposed(('booster', 'ship'), flap_deg=0.0).k_by_part(A_REF_M2))}")
    lines.append(f"- Ship nose first, flaps 90, exposed: {grouped(mesh.exposed(('ship',)).k_by_part(A_REF_M2))}")
    per_fin = {}
    for name, value in stack.items():
        if name.startswith("fin/"):
            per_fin[name.split("/")[1]] = per_fin.get(name.split("/")[1], 0.0) + value * A_REF_M2
    lines.append("- Solid load per fin, nose first, m^2: " + ", ".join(f"{a} deg {b:.3f}" for a, b in per_fin.items()))
    lines.append(f"- Fin, per gross area: tail first {mesh.fin_k_per_gross(factors['k_parts'], 'aft'):.3f}, nose first {mesh.fin_k_per_gross(stack, 'fwd'):.3f}; aft wall edge mean cos^2 {v.grid_fin_wall_edge_cos2('aft'):.3f}; mean cell depth, flat forward face {v.grid_fin_mean_depth_m('fwd'):.3f} m, scooped aft face {v.grid_fin_mean_depth_m('aft'):.3f} m")
    lines.append(f"- Hot-stage open band {v.hot_stage_open_height_m():.2f} m")
    lines += ["", "### Parts term at Mach 1.2, per part: k with the layer credit, of k without", ""]
    peak = next(r for r in rows if r["case"] == "ascent_stack" and r["mach"] == 1.2)
    for name, value in sorted(peak["other_parts"].items(), key=lambda x: -x[1]):
        lines.append(f"- {name}: {value:.4f} of {stack.get(name, 0.0):.4f}")
    lines.append(f"- Open fraction {v.grid_fin_open_fraction():.3f}")
    return "\n".join(lines) + "\n"


def write_csv(path, rows):
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


STACK_TABLE_HEADER = """# Axial drag coefficient of the full stack at zero angle of attack, reference area pi*(d/2)^2 (grid fins at 0, flaps at d = 90).
# [E] Derived from the built models by tools/aero_derive/derive_ascent_descent.py --write-stack-table, not fitted to anything:
# friction + nose + base + flaps + nose-first grid fins (3 x 12.63 m^2) + drawn parts (wake range, 1/7 profile layer credit) +
# hot-stage band. Method, errors and the old table: docs/subsystems/aero-derivation.md. Mach 0 holds the Mach 0.3 value;
# held beyond the last row. The ship alone flies config/aero/ship_ascent.csv.
"""


def write_stack_table(rows, path: Path) -> None:
    on = {r["mach"]: r["derived"] for r in rows if r["case"] == "ascent_stack" and r["column"] == "on"}
    off = {r["mach"]: r["derived"] for r in rows if r["case"] == "ascent_stack" and r["column"] == "off"}
    grid = [m for m in ASCENT_MACHS if m <= TABLE_MAX_MACH]
    lines = [STACK_TABLE_HEADER, "mach,cd_axial_power_on,cd_axial_power_off\n", f"0.0,{on[grid[0]]:.3f},{off[grid[0]]:.3f}\n"]
    lines += [f"{m:g},{on[m]:.3f},{off[m]:.3f}\n" for m in grid]
    path.write_text("".join(lines), encoding="utf-8")


def run(write_table: bool = False) -> list:
    OUT.mkdir(parents=True, exist_ok=True)
    v = geometry.load_vehicle("starship_block3")
    geometry.write_secured_areas(v)
    mesh = Mesh(v)
    raw = telemetry_bins()
    bins = bins_at_machs(raw)
    history = ascent_history(ASCENT_MACHS)
    rows = ascent_rows(v, mesh, history)
    down, factors = descent_rows(v, mesh, bins)
    rows += down
    if write_table:
        write_stack_table(rows, ROOT / "config" / "aero" / "starship_stack_ascent.csv")
    write_csv(OUT / "drag_compare.csv", rows)
    write_csv(OUT / "telemetry_bins.csv", raw)
    (OUT / "tables.md").write_text(markdown(rows, bins, factors, v, mesh) + ascent_drag_loss(rows) + chr(10), encoding="utf-8")
    plot_case(rows, "ascent_stack", "on", OUT / "drag_ascent.png", "Ascent, stack, power on: derived against the old table")
    plot_case(rows, "ascent_stack", "off", OUT / "drag_ascent_off.png", "Ascent, stack, power off: derived against the old table")
    plot_case(rows, "descent_body", "off", OUT / "drag_descent_body.png", "Booster descent, body without fins")
    plot_case(rows, "descent_total_drawn_12.63", "off", OUT / "drag_descent.png", "Booster descent, body + fins (12.63 m^2)")
    plot_beta(rows, OUT / "beta_descent.png")
    return rows


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write-stack-table", action="store_true", help="write config/aero/starship_stack_ascent.csv from the derived total")
    print(f"{len(run(parser.parse_args().write_stack_table))} rows written to {OUT / 'drag_compare.csv'}")
