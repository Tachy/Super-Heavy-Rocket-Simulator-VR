"""Engineering drag build-up. Every function returns (value, relative error).

Constants from the books below were recalled, not re-read from a printed copy (no copy in this project): their error
bands are deliberately wide and their [E] tag is the tool's own judgement. Nothing here is fitted to the vehicle's
tables. Air data: US76 via tools/telemetry/drag_estimate.atmosphere, which is valid below 86 km; above it the
isothermal extension is an out-of-range estimate.

Sources: Hoerner, Fluid-Dynamic Drag (1965); Schlichting, Boundary-Layer Theory; Sommer & Short, NACA TN 3391 (1955);
Fleeman, Tactical Missile Design (AIAA, 2006); Idel'chik, Handbook of Hydraulic Resistance; NACA 1135.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "telemetry"))
from drag_estimate import GAMMA_AIR, R_AIR, atmosphere  # noqa: E402

from newtonian import cp_max  # noqa: E402

SUTHERLAND_MU_REF = 1.458e-6  # kg/(m s K^0.5), US76
SUTHERLAND_T_K = 110.4  # K, US76
TURBULENT_RECOVERY = 0.89  # turbulent recovery factor, Pr^(1/3)
TRANSITION_RE = 5.0e5  # [E] flat-plate transition; below it Blasius
FLAT_FACE_MEAN_CP_RATIO_SUBSONIC = 0.76  # Hoerner ch. III: mean face pressure over stagnation on a flat cylinder face [E]
FLAT_FACE_MEAN_CP_RATIO_SUPERSONIC = 0.90  # Hoerner ch. XVI trend: towards Newtonian at Mach >= 3 [E]
NOSE_CRITICAL_MACH = 0.8  # Hoerner ch. XVI: drag rise of a fineness ~1.5 ogive begins here [E]
CAVITY_CD_ON_OPENING = 0.03  # Hoerner ch. V: open circumferential cavity, on its opening area [E], +-100 %


def viscosity(t_k: float) -> float:
    return SUTHERLAND_MU_REF * t_k ** 1.5 / (t_k + SUTHERLAND_T_K)


def air(altitude_m: float) -> dict:
    """rho, sound speed, temperature, viscosity and mean free path (Chapman: mu / rho sqrt(pi / (2 R T)))."""
    rho, a = atmosphere(max(altitude_m, 0.0))
    t = a * a / (GAMMA_AIR * R_AIR)
    mu = viscosity(t)
    return {"rho": rho, "a": a, "t": t, "mu": mu, "mfp": mu / rho * math.sqrt(math.pi / (2.0 * R_AIR * t))}


def flat_plate_cf(reynolds: float, mach: float, t_e_k: float) -> tuple[float, float]:
    """Mean flat-plate friction, adiabatic wall, with Sommer-Short reference temperature (NACA TN 3391):
    T'/Te = 1 + 0.035 M^2 + 0.45 (Tw/Te - 1), evaluated with the Prandtl-Schlichting law 0.455 / log10(Re)^2.58
    (Schlichting). Below the transition Reynolds number, Blasius 1.328 / sqrt(Re).
    Error [E]: +-5 % for the law, +-10 % at Mach > 3 for the reference temperature."""
    ratio_wall = 1.0 + TURBULENT_RECOVERY * 0.5 * (GAMMA_AIR - 1.0) * mach * mach
    ratio_t = 1.0 + 0.035 * mach * mach + 0.45 * (ratio_wall - 1.0)
    t_ref = ratio_t * t_e_k
    re_ref = reynolds / ratio_t / (viscosity(t_ref) / viscosity(t_e_k))
    if re_ref < TRANSITION_RE:
        return 1.328 / math.sqrt(max(re_ref, 1.0)) / ratio_t, 0.3
    return 0.455 / math.log10(re_ref) ** 2.58 / ratio_t, 0.10 if mach > 3.0 else 0.05


def skin_friction_cd(wetted_m2: float, length_m: float, mach: float, altitude_m: float, a_ref_m2: float) -> tuple[float, float]:
    """Friction drag per q A_ref. Hydraulically smooth; form factor neglected (slender hull: Hoerner ch. VI gives under
    3 % at L/D 14). Error [E]: the law's, widened to +-20 % for welds, tile gaps and fittings that are not resolved."""
    state = air(altitude_m)
    speed = mach * state["a"]
    reynolds = max(state["rho"] * speed * length_m / state["mu"], 1.0)
    cf, err = flat_plate_cf(reynolds, mach, state["t"])
    return cf * wetted_m2 / a_ref_m2, math.hypot(err, 0.2)


def nose_pressure_cd(mach: float, fineness: float) -> tuple[float, float]:
    """Nose wave drag, per the body's frontal area. Supersonic: Bonney's formula as given by Fleeman (ch. 2):
    (1.586 + 1.834 / M^2) atan(0.5 / (l/d))^1.69, for sharp noses, kept for the drawn blunt tip (tip r 0.5 of 4.5 m).
    Below Mach 1: zero at the critical Mach, a cubic rise to the Mach 1 value [E]. Hoerner: subsonic pressure drag of a
    faired nose is negligible. Error [E]: +-25 % supersonic, +-50 % transonic."""
    shape = math.atan(0.5 / fineness) ** 1.69
    if mach >= 1.0:
        return (1.586 + 1.834 / (mach * mach)) * shape, 0.5 if mach < 1.2 else 0.25
    if mach <= NOSE_CRITICAL_MACH:
        return 0.0, 0.5
    fraction = (mach - NOSE_CRITICAL_MACH) / (1.0 - NOSE_CRITICAL_MACH)
    return (1.586 + 1.834) * shape * fraction ** 3, 0.5


def base_cd(mach: float, base_area_m2: float, powered: bool, a_ref_m2: float, exit_area_m2: float = 0.0) -> tuple[float, float]:
    """Base drag: Fleeman ch. 2: 0.12 + 0.13 M^2 below Mach 1, 0.25 / M above, on the base area; never beyond the vacuum
    limit 2 / (gamma M^2). Powered: only the base outside the nozzle exits counts (Fleeman's (1 - Ae / Aref)), which
    leaves out plume pumping and recirculation. Error [E]: +-30 % off, +-60 % on (plume interaction)."""
    if mach < 1.0:
        c = 0.12 + 0.13 * mach * mach
    else:
        c = min(0.25 / mach, 2.0 / (GAMMA_AIR * mach * mach))
    area = base_area_m2
    if powered:
        area = max(base_area_m2 - exit_area_m2, 0.0)
    return c * area / a_ref_m2, 0.6 if powered else 0.3


def blunt_face_cd(mach: float) -> tuple[float, float]:
    """Mean pressure on a flat face per q: Hoerner's trend, a fixed share of the stagnation value (face + bubble edge).
    The share goes from 0.76 at Mach <= 1 to 0.90 at Mach >= 3 [E]. Error [E]: +-10 % subsonic, +-20 % transonic and
    low supersonic, where the book's own scatter is largest."""
    if mach <= 1.0:
        share = FLAT_FACE_MEAN_CP_RATIO_SUBSONIC
    else:
        t = min((mach - 1.0) / 2.0, 1.0)
        share = FLAT_FACE_MEAN_CP_RATIO_SUBSONIC + t * (FLAT_FACE_MEAN_CP_RATIO_SUPERSONIC - FLAT_FACE_MEAN_CP_RATIO_SUBSONIC)
    return share * cp_max(mach), 0.10 if mach < 0.8 else 0.20


def protuberance_cd(mach: float, loaded_area_m2: float, a_ref_m2: float) -> tuple[float, float]:
    """Drag of a blunt part standing in the flow: `loaded_area_m2` = sum(cos^2 phi dA) of its first-hit samples (from the
    mesh, so ramps and edges count less than a face), times face plus own base (Hoerner ch. VIII). No credit for the
    boundary layer it stands in. Error [E]: +-40 %."""
    face, _ = blunt_face_cd(mach)
    back, _ = base_cd(mach, 1.0, False, 1.0)
    return (face + back) * loaded_area_m2 / a_ref_m2, 0.4


def cavity_cd(opening_area_m2: float, a_ref_m2: float) -> tuple[float, float]:
    """Open band in the hull (the hot-stage truss): drag on the opening's area, Hoerner ch. V [E]; error +-100 %."""
    return CAVITY_CD_ON_OPENING * opening_area_m2 / a_ref_m2, 1.0


def grid_fin_capture(mach: float, open_fraction: float) -> float:
    """Share of the fin's stream tube the cells can swallow, one-dimensional gas dynamics [E]: sonic throat of area
    `open_fraction`: A/A*(M) open_fraction, with the normal-shock total pressure loss above Mach 1. 1 = started."""
    g = GAMMA_AIR
    if mach <= 0.0:
        return 1.0
    area_ratio = ((2.0 / (g + 1.0)) * (1.0 + 0.5 * (g - 1.0) * mach * mach)) ** ((g + 1.0) / (2.0 * (g - 1.0))) / mach
    pitot = 1.0
    if mach > 1.0:
        m2 = mach * mach
        pitot = (((g + 1.0) * m2) / ((g - 1.0) * m2 + 2.0)) ** (g / (g - 1.0)) * ((g + 1.0) / (2.0 * g * m2 - (g - 1.0))) ** (1.0 / (g - 1.0))
    return min(1.0, open_fraction * area_ratio * pitot) if mach > 0.3 else 1.0


def grid_fin_ca(mach: float, open_fraction: float, altitude_m: float, k_gross: float, depth_m: float, pitch_m: float) -> tuple[float, float]:
    """Axial force of a grid fin at zero deflection, per its gross frontal area.

    Started flow: wall faces and friction in the cells. Subsonic: Idel'chik's thick grid
    (0.5 (1 - f) + (1 - f)^2) / f^2 on the approach q; supersonic: the first-hit Newtonian `k_gross` (per gross area, from
    the mesh, per unit cp_max) times cp_max plus the walls' base. Choked (capture < 1): the spilled share of the stream
    acts on a solid plate (blunt face plus base), the swallowed share as started.
    Friction: 2 wall families x 2 faces x depth / pitch of wetted area per gross area. Error [E]: +-30 % subsonic and
    started, +-50 % choked (no fin-specific data; Washington & Miller, AIAA 93-0035 is the only comparison named in
    aero/grid_fin.csv, not re-read here)."""
    capture = grid_fin_capture(mach, open_fraction)
    solidity = 1.0 - open_fraction
    state = air(altitude_m)
    speed = max(mach * state["a"], 1.0)
    velocity_gain = 1.0 / open_fraction ** 2 if mach < 1.0 else 1.0
    cf, _ = flat_plate_cf(state["rho"] * speed * depth_m / state["mu"], mach, state["t"])
    friction = cf * 4.0 * depth_m / pitch_m * velocity_gain
    if mach < 1.0:
        started = (0.5 * solidity + solidity * solidity) / open_fraction ** 2 + friction
    else:
        started = cp_max(mach) * k_gross + base_cd(mach, solidity, False, 1.0)[0] + friction
    if capture >= 1.0:
        return started, 0.3
    plate = blunt_face_cd(mach)[0] + base_cd(mach, 1.0, False, 1.0)[0]
    return capture * started + (1.0 - capture) * plate, 0.5
