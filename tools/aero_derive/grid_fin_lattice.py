"""Normal force slope and limit of a square-cell grid fin from its depth, pitch and wall (vehicle physics Task 4b).

Comparison tool only: writes screenshots/vehicle-physics/grid_fin_lattice.{csv,md}; nothing here feeds config or core.
Reference area = gross frontal area, the core's `F = q A cn_alpha sin(alpha)` convention. Deflection turns the plate about the
radial axis, which is its mirror axis, so both wall families lie at +-45 deg to the turning axis.

Geometry of the load [derived here]: a family has c / p of wall per gross area and sees alpha_w = atan(tan(alpha) / sqrt 2); its
normal force counts cos 45 deg along the fin normal and the radial parts of the families cancel. For small alpha two families
give exactly the slope of ONE family at alpha. In linear theory the two families superpose exactly in a square cell (the wall
condition of one involves only its own normal), so a 2-D treatment per family is not an extra assumption there.

Theory, all RECALLED FROM MEMORY unless marked; none was re-read for this tool (a web search on 2026-11-01 found only secondary
statements, listed in docs/subsystems/aero-derivation.md):
- Subsonic wall loading: Weinig's unstaggered flat-plate cascade (1935), lift on the mean incidence with
  k = tanh(pi sigma / 2) / (pi sigma / 2). Closed with the momentum balance here, which gives F / (q s) = 4 T alpha / (1 + T),
  T = tanh(pi sigma / 2). Checked by its limits: isolated plate 2 pi sigma, dense guide vanes 2 alpha (both tested).
  Compressibility by Prandtl-Glauert on the spacing (sigma / beta); keeps both limits.
- Finite array: induced incidence as a lifting system of span b, AR = b^2 / A, raised by Prandtl's box-wing factor
  (1 + 0.45 h/b) / (1.04 + 2.81 h/b) (Prandtl 1924), h = stack height. Crude for a lattice, part of the +-25 %.
- Choking: one-dimensional sonic throat and the Kantrowitz-Donaldson (1945) starting limit, `buildup.grid_fin_capture`. Spilled
  mass carries no turning. Trend: Washington & Miller, AIAA 93-0035 (secondary statement only).
- Supersonic started: Ackeret on the walls, or the exact oblique shock + Prandtl-Meyer (NACA 1135) at finite incidence, with the
  cell's Mach wave reflection as in Busemann's biplane (derived here in linear theory): the pressure difference over a wall flips
  each time x passes a multiple of p beta, so the wall carries the chord `wave_chord`. No net force at c = 2 p beta. That a too
  deep cell loses force by this reflection is stated for grid fins in an IISc thesis (etd.iisc.ac.in/handle/2005/563, abstract).
- Limit, subsonic and choked: leading edge separation of a thin plate at a mean incidence of about 11 deg (Hoerner, Fluid-Dynamic
  Lift, thin plates; from memory), falling to about 6.5 deg at Mach 1 (assumed, thin-section CL max trend). Started: the wall
  shock's sonic limit or the unstart of the cell with the throat narrowed by cos(alpha_w). Low confidence.
No fin-specific measurement exists in this repo. Errors are [E] judgments, not statistics.
"""
from __future__ import annotations

import csv
import math
import sys
from dataclasses import dataclass, replace
from itertools import product
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import buildup  # noqa: E402
from geometry import ROOT, _horn  # noqa: E402
from newtonian import GAMMA_AIR  # noqa: E402

# Secured geometry, docs/subsystems/aero-derivation.md "Grid fin authority".
PLAN_SPAN_M = 4.54  # radial extent, detail outline
PLAN_WIDTH_M = 3.53  # tangential extent, detail outline
GROSS_AREA_M2 = 12.63
RADIUS_M = 7.19  # config
LEVER_M = 44.1  # station 64 less the CG at peak q: Task 4's 94.3 MN m = 2 q A cn sin 30 L
MAX_DEFLECTION_DEG = 30.0  # config
CUSP_SOFT = 0.3  # detail file
A_REF_M2 = 63.617
FIN_COUNT = 3

SOLID_ACTIVE = 0.90  # share of the gross area with lattice: 1 - (band 0.06 + frame 0.89 + pylon 0.31) / 12.63, +-0.05 (the rib stays)
SOLID_BLOCKED = 0.116  # solid share along the cells for the axial force: 1.46 m^2 of 12.63, aero-derivation.md
K_SOLID_FWD, K_SOLID_AFT = 0.105, 0.0665  # Newtonian solid parts per gross area: tool values 0.296 - 0.191 and 0.128 - 0.0615

# Thin plate leading edge separation (see the docstring).
STALL_LOW_MACH_DEG = 11.0
STALL_MACH_1_DEG = 6.5
STALL_FADE_START_MACH = 0.5
LIMIT_CAP_DEG = 45.0  # beyond the core's reach (30 deg + flow) and of the small-alpha load geometry
ALPHA_REF_DEG = 10.0  # secant of the nonlinear supersonic load; the core multiplies by sin(alpha)

MACH_GRID = [0.0, 0.3, 0.5, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 2.0, 2.5, 3.0, 4.0, 5.0, 7.0, 10.0, 15.0, 25.0]


@dataclass(frozen=True)
class Lattice:
    tip_m: float  # depth from the flat face to the cusp tips
    scoop_m: float  # depth of the saddle scoop between two crossings
    pitch_m: float
    wall_m: float
    active: float = SOLID_ACTIVE
    span_m: float = PLAN_SPAN_M
    width_m: float = PLAN_WIDTH_M

    def chords(self) -> np.ndarray:
        u = np.linspace(0.0, 1.0, 201)
        return self.tip_m - self.scoop_m * np.array([_horn(v, CUSP_SOFT) for v in u])

    @property
    def mean_chord_m(self) -> float:
        return float(np.mean(self.chords()))

    @property
    def cell_open(self) -> float:
        return (1.0 - self.wall_m / self.pitch_m) ** 2


NOMINAL = Lattice(tip_m=0.45, scoop_m=0.22, pitch_m=0.35, wall_m=0.012)
# Secured ranges; the catch and plain fin are one fin here.
TIP_RANGE, SCOOP_RANGE, PITCH_RANGE, WALL_RANGE = (0.40, 0.50), (0.09, 0.30), (0.33, 0.37), (0.006, 0.044)


def beta_sub(mach: float) -> float:
    return math.sqrt(max(1.0 - mach * mach, 0.0))


def cascade_slope(sigma: float, mach: float) -> float:
    """dF/dalpha per pitch and q of an unstaggered flat-plate cascade, solidity sigma = c / p."""
    beta = beta_sub(mach)
    t = math.tanh(0.5 * math.pi * sigma / beta) if beta > 1e-6 else 1.0
    return 4.0 * t / (1.0 + t)


def box_aspect_ratio(lat: Lattice) -> float:
    aspect = lat.span_m * lat.span_m / GROSS_AREA_M2
    h_over_b = lat.width_m / lat.span_m
    return aspect * (1.04 + 2.81 * h_over_b) / (1.0 + 0.45 * h_over_b)


def oblique_shock_pressure_ratio(mach: float, theta_rad: float) -> float:
    """p2 / p1 of the attached weak shock (NACA 1135); at the detachment angle if theta passes it."""
    if theta_rad <= 0.0:
        return 1.0
    return _shock(mach, min(theta_rad, _theta_max(mach)))[0]


def _shock(mach: float, theta: float) -> tuple[float, float]:
    """(p2 / p1, M2) of the weak solution."""
    g = GAMMA_AIR
    lo, hi = math.asin(1.0 / mach), _beta_at_theta_max(mach)
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if _theta(mach, mid) < theta:
            lo = mid
        else:
            hi = mid
    b = 0.5 * (lo + hi)
    mn = mach * math.sin(b)
    p = 1.0 + 2.0 * g / (g + 1.0) * (mn * mn - 1.0)
    mn2 = math.sqrt((1.0 + 0.5 * (g - 1.0) * mn * mn) / (g * mn * mn - 0.5 * (g - 1.0)))
    return p, mn2 / math.sin(b - theta)


def _theta(mach: float, b: float) -> float:
    g = GAMMA_AIR
    m2 = (mach * math.sin(b)) ** 2
    return math.atan(2.0 / math.tan(b) * (m2 - 1.0) / (mach * mach * (g + math.cos(2.0 * b)) + 2.0))


def _beta_at_theta_max(mach: float) -> float:
    grid = np.linspace(math.asin(1.0 / mach) + 1e-6, 0.5 * math.pi - 1e-6, 4001)
    return float(grid[int(np.argmax([_theta(mach, b) for b in grid]))])


def _theta_max(mach: float) -> float:
    return _theta(mach, _beta_at_theta_max(mach))


def sonic_wedge_rad(mach: float) -> float:
    """Wall incidence at which the flow behind the wall's shock turns sonic (just under detachment)."""
    lo, hi = 0.0, _theta_max(mach) * 0.99999
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if _shock(mach, mid)[1] > 1.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def prandtl_meyer(mach: float) -> float:
    g = GAMMA_AIR
    k = math.sqrt((g + 1.0) / (g - 1.0))
    b = math.sqrt(mach * mach - 1.0)
    return k * math.atan(b / k) - math.atan(b)


def _expansion_pressure_ratio(mach: float, theta: float) -> float:
    """p2 / p1 through the expansion; 0 past the vacuum limit."""
    g = GAMMA_AIR
    target = prandtl_meyer(mach) + theta
    if target >= 0.5 * math.pi * (math.sqrt((g + 1.0) / (g - 1.0)) - 1.0) * 0.9999:
        return 0.0
    lo, hi = mach, 1e4
    for _ in range(80):
        mid = math.sqrt(lo * hi)
        if prandtl_meyer(mid) < target:
            lo = mid
        else:
            hi = mid
    m2 = 0.5 * (lo + hi)
    f = lambda m: 1.0 + 0.5 * (g - 1.0) * m * m  # noqa: E731
    return (f(mach) / f(m2)) ** (g / (g - 1.0))


def wall_pressure_difference(mach: float, alpha_w: float) -> float:
    """cp windward - cp leeward of a wall before any reflection."""
    k = 2.0 / (GAMMA_AIR * mach * mach)
    return k * (oblique_shock_pressure_ratio(mach, alpha_w) - _expansion_pressure_ratio(mach, alpha_w))


def wave_chord(chord_m: float, p_beta_m: float) -> float:
    """Effective loaded chord: the difference across a wall flips at each multiple of p beta (triangle wave of the chord)."""
    x = chord_m % (2.0 * p_beta_m)
    return x if x <= p_beta_m else 2.0 * p_beta_m - x


def start_mach(lat: Lattice) -> float:
    """Free-stream Mach above which the cells are started."""
    lo, hi = 1.0, 10.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if _capture_raw(mid, lat) < 1.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _subsonic_slope(mach: float, lat: Lattice) -> tuple[float, float]:
    """(cascade slope averaged over the chords, the slope with the induced incidence)."""
    cf = float(np.mean([cascade_slope(c / lat.pitch_m, mach) for c in lat.chords()]))
    return cf, cf / _induced_divisor(cf, mach, lat)


def _induced_divisor(cf: float, mach: float, lat: Lattice) -> float:
    return 1.0 + cf * beta_sub(mach) / (math.pi * box_aspect_ratio(lat))


def regime(mach: float, lat: Lattice) -> str:
    if mach >= start_mach(lat):
        return "started"
    return "choked" if buildup.grid_fin_capture(mach, lat.cell_open) < 1.0 or mach >= 1.0 else "subsonic"


def cn_alpha(mach: float, lat: Lattice = NOMINAL, alpha_rad: float | None = None) -> float:
    """Secant normal force slope per gross area, /rad, in the core's law (coefficient = slope x sin alpha)."""
    alpha = math.radians(ALPHA_REF_DEG) if alpha_rad is None else alpha_rad
    if mach < 1.0:
        return lat.active * buildup.grid_fin_capture(mach, lat.cell_open) * _subsonic_slope(mach, lat)[1]
    if mach < start_mach(lat):
        return lat.active * buildup.grid_fin_capture(mach, lat.cell_open) * 2.0  # choked cell: the dense limit
    beta = math.sqrt(mach * mach - 1.0)
    alpha_w = math.atan(math.tan(alpha) / math.sqrt(2.0))
    loaded = float(np.mean([wave_chord(c, lat.pitch_m * beta) for c in lat.chords()])) / lat.pitch_m
    return lat.active * math.sqrt(2.0) * wall_pressure_difference(mach, alpha_w) * math.cos(alpha_w) * loaded / math.sin(alpha) \
        if False else lat.active * math.sqrt(2.0) * wall_pressure_difference(mach, alpha_w) * loaded / math.sin(alpha)


def _stall_mean_incidence_deg(mach: float) -> float:
    if mach <= STALL_FADE_START_MACH:
        return STALL_LOW_MACH_DEG
    if mach >= 1.0:
        return STALL_MACH_1_DEG
    f = (mach - STALL_FADE_START_MACH) / (1.0 - STALL_FADE_START_MACH)
    return STALL_LOW_MACH_DEG + f * (STALL_MACH_1_DEG - STALL_LOW_MACH_DEG)


def _fin_angle_of_wall_deg(alpha_w_deg: float) -> float:
    return math.degrees(math.atan(math.sqrt(2.0) * math.tan(math.radians(alpha_w_deg))))


def limit_deg(mach: float, lat: Lattice = NOMINAL) -> float:
    """Fin incidence beyond which the force stops growing."""
    cf, _ = _subsonic_slope(min(mach, 0.999), lat)
    if mach < 1.0:
        wall = _stall_mean_incidence_deg(mach) / (1.0 - cf / 4.0) * _induced_divisor(cf, mach, lat)
    else:
        wall = STALL_MACH_1_DEG / (1.0 - 2.0 / 4.0)
    transonic = min(_fin_angle_of_wall_deg(wall), LIMIT_CAP_DEG)
    if mach < start_mach(lat):
        return transonic
    # started: wall shock sonic, or the cell unstarts on its narrowed throat
    sonic = math.degrees(sonic_wedge_rad(mach))
    unstart = math.degrees(math.acos(min(1.0 / _capture_raw(mach, lat), 1.0)))
    return min(max(transonic, _fin_angle_of_wall_deg(min(sonic, unstart))), LIMIT_CAP_DEG)


def _capture_raw(mach: float, lat: Lattice) -> float:
    """Open share x A/A* x pitot ratio, not clipped: >= 1 = the cell swallows its shock (Kantrowitz)."""
    g = GAMMA_AIR
    m2 = mach * mach
    area = ((2.0 / (g + 1.0)) * (1.0 + 0.5 * (g - 1.0) * m2)) ** ((g + 1.0) / (2.0 * (g - 1.0))) / mach
    pitot = (((g + 1.0) * m2) / ((g - 1.0) * m2 + 2.0)) ** (g / (g - 1.0)) * ((g + 1.0) / (2.0 * g * m2 - (g - 1.0))) ** (1.0 / (g - 1.0))
    return lat.cell_open * area * pitot


def model_error(mach: float, lat: Lattice) -> float:
    """[E] judgment of the theory's own relative error at the nominal geometry."""
    kind = regime(mach, lat)
    if kind == "subsonic":
        return 0.25
    if kind == "choked":
        return 0.40
    if mach < 1.8:
        return 0.35  # reflection, shock and boundary layer in the same cell: least certain started band
    return 0.20 if mach < 5.0 else 0.30  # real gas and viscous interaction not modelled above


def _corners(lat: Lattice):
    for tip, scoop, pitch, wall in product(TIP_RANGE, SCOOP_RANGE, PITCH_RANGE, WALL_RANGE):
        yield replace(lat, tip_m=tip, scoop_m=scoop, pitch_m=pitch, wall_m=wall)


def table(lat: Lattice = NOMINAL) -> list[dict]:
    rows = []
    for mach in MACH_GRID:
        nominal = cn_alpha(mach, lat)
        band = [cn_alpha(mach, c) for c in _corners(lat)]
        limit = limit_deg(mach, lat)
        limits = [limit_deg(mach, c) for c in _corners(lat)]
        rows.append(dict(mach=mach, regime=regime(mach, lat), cn_alpha=nominal, cn_err_rel=model_error(mach, lat),
                         cn_low=min(band + [nominal]), cn_high=max(band + [nominal]), limit_deg=limit,
                         limit_err_deg=max(0.3 * limit, max(limits) - limit, limit - min(limits))))
    return rows


def authority(q_pa, slope, limit_deg, area_m2, radius_m, lever_m, deflection_deg) -> dict:
    """Largest moments with the 90 / 90 / 180 layout: pitch by the opposite pair, yaw by the single fin, roll by all."""
    force = q_pa * area_m2 * slope * math.sin(math.radians(min(deflection_deg, limit_deg)))
    return dict(fin_force_n=force, roll_nm=FIN_COUNT * force * radius_m, pitch_nm=2.0 * force * lever_m, yaw_nm=force * lever_m)


def config_table() -> dict:
    path = ROOT / "config" / "aero" / "grid_fin.csv"
    with open(path, encoding="utf-8") as handle:
        rows = list(csv.DictReader(line for line in handle if not line.startswith("#")))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def config_value(table_: dict, column: str, mach: float) -> float:
    return float(np.interp(mach, table_["mach"], table_[column]))


def axial_ca(mach: float, lat: Lattice, face: str, altitude_m: float) -> float:
    """`buildup.grid_fin_ca` at this lattice: the walls' Newtonian share, open fraction and friction follow wall, pitch and depth."""
    share = 1.0 - (1.0 - lat.wall_m / lat.pitch_m) ** 2
    open_fraction = (1.0 - SOLID_BLOCKED) * (1.0 - share)
    if face == "fwd":
        k = K_SOLID_FWD + (1.0 - SOLID_BLOCKED) * share
        entry = lat.tip_m
    else:
        s = np.linspace(0.0, lat.pitch_m, 2001)
        x = lat.scoop_m * np.array([_horn(v, CUSP_SOFT) for v in s / lat.pitch_m])
        cos2 = float(np.mean(1.0 / (1.0 + np.gradient(x, s) ** 2)))
        k = K_SOLID_AFT + (1.0 - SOLID_BLOCKED) * share * cos2
        entry = lat.mean_chord_m
    return buildup.grid_fin_ca(mach, open_fraction, altitude_m, k, lat.tip_m, lat.pitch_m, lat.wall_m, entry)[0]


def _scaled_depth(tip_m: float) -> Lattice:
    return replace(NOMINAL, tip_m=tip_m, scoop_m=NOMINAL.scoop_m * tip_m / NOMINAL.tip_m)


CASES = {
    "nominal tip 0.45": NOMINAL,
    "depth tip 0.40 (STL)": _scaled_depth(0.40),
    "depth tip 0.50": _scaled_depth(0.50),
    "depth tip 0.70 (excluded)": _scaled_depth(0.70),
    "no scoop, chord 0.45": replace(NOMINAL, scoop_m=0.0),
    "STL scoop 0.09": replace(NOMINAL, scoop_m=0.09),
    "pitch 0.33": replace(NOMINAL, pitch_m=0.33),
    "pitch 0.37": replace(NOMINAL, pitch_m=0.37),
    "pitch 0.47 (excluded)": replace(NOMINAL, pitch_m=0.47),
    "wall 0.006": replace(NOMINAL, wall_m=0.006),
    "wall 0.02": replace(NOMINAL, wall_m=0.02),
    "wall 0.044 (STL)": replace(NOMINAL, wall_m=0.044),
}
FLIGHT_POINTS = [  # task-4-report.md
    ("f13 peak q, tail first", 106.7e3, 2.63),
    ("f14 peak q, tail first", 101.0e3, 2.44),
    ("landing burn ignition", 39.0e3, 0.77),
    ("ascent max q (fins not commanded)", 38.9e3, 1.36),
]
SENS_MACH = [0.5, 1.1, 1.5, 2.0, 2.63, 4.0, 8.0]
DRAG_MACH = [0.3, 0.8, 1.05, 1.5, 2.0, 3.0, 5.0, 8.0]
DRAG_ALTITUDE_M = 20e3  # friction only, a small part; the tool's own columns use the flight's altitudes
OLD_DRAWN = Lattice(tip_m=0.50, scoop_m=0.30, pitch_m=0.35, wall_m=0.04)  # today's detail file: the tool's fin CA


def report() -> tuple[list[dict], str]:
    rows = table()
    t = config_table()
    md = ["# Grid fin lattice derivation", "",
          f"Nominal: tip {NOMINAL.tip_m} m, scoop {NOMINAL.scoop_m} m, mean chord {NOMINAL.mean_chord_m:.3f} m, c/p {NOMINAL.mean_chord_m / NOMINAL.pitch_m:.2f}, "
          f"pitch {NOMINAL.pitch_m} m, wall {NOMINAL.wall_m} m, cell open {NOMINAL.cell_open:.3f}, active {NOMINAL.active}, box AR {box_aspect_ratio(NOMINAL):.2f}, "
          f"cells started from Mach {start_mach(NOMINAL):.2f}. Secant at {ALPHA_REF_DEG:.0f} deg.", "",
          "| Mach | regime | cn_alpha /rad | model +- | geometry band | table | ratio | limit deg | +- | table limit |", "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        tv = config_value(t, "cn_alpha_per_rad", r["mach"])
        r["table_cn"], r["table_limit"] = tv, config_value(t, "limit_deg", r["mach"])
        md.append(f"| {r['mach']:g} | {r['regime']} | {r['cn_alpha']:.2f} | {r['cn_err_rel'] * 100:.0f} % | {r['cn_low']:.2f}-{r['cn_high']:.2f} | {tv:.2f} | "
                  f"{r['cn_alpha'] / tv:.2f} | {r['limit_deg']:.0f} | {r['limit_err_deg']:.0f} | {r['table_limit']:.0f} |")
    md += ["", "## Sensitivity: cn_alpha /rad and limit deg", "", "| Case | mean chord | started from | " + " | ".join(f"M {m:g}" for m in SENS_MACH) + " |",
           "|---|---|---|" + "---|" * len(SENS_MACH)]
    for name, lat in CASES.items():
        cells = " | ".join(f"{cn_alpha(m, lat):.2f} / {limit_deg(m, lat):.0f}" for m in SENS_MACH)
        md.append(f"| {name} | {lat.mean_chord_m:.3f} | {start_mach(lat):.2f} | {cells} |")
    md += ["", f"## Authority: 3 fins, r {RADIUS_M} m, lever {LEVER_M} m, deflection {MAX_DEFLECTION_DEG:.0f} deg; MN m", "",
           "| Point | q kPa | Mach | | angle deg | fin force MN | roll | pitch (pair) | yaw (single) |", "|---|---|---|---|---|---|---|---|---|"]
    ratios = {}
    for name, q, mach in FLIGHT_POINTS:
        limit_table = config_value(t, "limit_deg", mach)
        table_a = authority(q, config_value(t, "cn_alpha_per_rad", mach), limit_table, GROSS_AREA_M2, RADIUS_M, LEVER_M, MAX_DEFLECTION_DEG)
        limit = limit_deg(mach)
        angle = min(limit, MAX_DEFLECTION_DEG)
        slope = cn_alpha(mach, NOMINAL, math.radians(angle))
        derived_a = authority(q, slope, limit, GROSS_AREA_M2, RADIUS_M, LEVER_M, MAX_DEFLECTION_DEG)
        err = model_error(mach, NOMINAL)
        ratios[name] = derived_a["fin_force_n"] / table_a["fin_force_n"]
        for label, a, ang in (("table", table_a, min(limit_table, MAX_DEFLECTION_DEG)), (f"derived +-{err * 100:.0f} %", derived_a, angle)):
            md.append(f"| {name} | {q / 1e3:.1f} | {mach} | {label} | {ang:.1f} | {a['fin_force_n'] / 1e6:.2f} | {a['roll_nm'] / 1e6:.1f} | {a['pitch_nm'] / 1e6:.1f} | {a['yaw_nm'] / 1e6:.1f} |")
        md.append(f"| {name} | | | derived / table | | {ratios[name]:.2f} | | | |")
    md += ["", "Force at peak q for a depth case (f13, Mach 2.63, derived, MN per fin):"]
    for name, lat in CASES.items():
        limit = limit_deg(2.63, lat)
        angle = min(limit, MAX_DEFLECTION_DEG)
        force = authority(106.7e3, cn_alpha(2.63, lat, math.radians(angle)), limit, GROSS_AREA_M2, RADIUS_M, LEVER_M, MAX_DEFLECTION_DEG)["fin_force_n"]
        md.append(f"- {name}: {force / 1e6:.2f}")
    md += ["", "## Axial force: fin CA per gross, and the stack / descent term 3 x 12.63 / 63.617 x CA", "",
           "| Mach | face | today's drawn (tip 0.5, wall 0.04) | tip 0.40 | nominal 0.45 | tip 0.50 | tip 0.70 | wall 0.02 | wall 0.006 | table ca_axial |", "|---|---|---|---|---|---|---|---|---|---|"]
    variants = [OLD_DRAWN, _scaled_depth(0.40), NOMINAL, _scaled_depth(0.50), _scaled_depth(0.70), replace(NOMINAL, wall_m=0.02), replace(NOMINAL, wall_m=0.006)]
    factor = FIN_COUNT * GROSS_AREA_M2 / A_REF_M2
    md_term = []
    for face, label in (("fwd", "nose first (ascent)"), ("aft", "tail first (descent)")):
        for mach in DRAG_MACH:
            vals = [axial_ca(mach, v, face, DRAG_ALTITUDE_M) for v in variants]
            md.append(f"| {mach:g} | {label} | " + " | ".join(f"{v:.3f}" for v in vals) + f" | {config_value(t, 'ca_axial', mach):.2f} |")
            md_term.append(f"| {mach:g} | {label} | " + f"{vals[0] * factor:.3f} | {vals[2] * factor:.3f} | {(vals[2] - vals[0]) * factor:+.3f} | {(vals[3] - vals[1]) * factor:+.3f} | {(vals[4] - vals[2]) * factor:+.3f} |")
    md += ["", "Term in CD (per 63.6 m^2):", "", "| Mach | face | today's drawn | nominal | change | tip 0.50 less tip 0.40 | tip 0.70 less nominal |", "|---|---|---|---|---|---|---|"] + md_term
    return rows, "\n".join(md) + "\n"


def main() -> None:
    rows, text = report()
    out = ROOT / "screenshots" / "vehicle-physics"
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "grid_fin_lattice.csv", "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (out / "grid_fin_lattice.md").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
