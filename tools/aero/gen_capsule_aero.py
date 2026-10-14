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
