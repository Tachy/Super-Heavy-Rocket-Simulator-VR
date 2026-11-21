"""The room (fidelity plan Task 9): window reveals and the ceiling ring's steps, read on crew1_1 (hatch view, fitted camera).

Window reveals (az +-90, image right / left; the az +-33 pair is hidden behind the console and the crew in every fitted photo):
crew1_1 shows each side window as a thick white rounded surround (outer edge a thin grey line), inside it a black opening (the shade cover
fills the reveal; in flight the glass with its black gasket ring, nasatour_0420). The generator's rings (gen_dragon_cabin.window_rings) are
projected through the fitted crew1_1 camera and their left / right extremes per image row compared with the reads:
- "outer": the surround's outer edge = the rim's foot rho0 + bezel on the wall,
- "near" / "far": the dark opening's edge on the side toward / away from the camera = the reveal's edge rho0 (the cover sits in the reveal).
Fitted (fit_windows): rho0, bezel (shared by both windows), and nuisance centre shifts d_x (both), d_az (mirrored), az_common (both, same
sign), with wide priors. The centre shifts come out at d_x +0.064 +- 0.007 m, az_common -1.3 +- 0.3 deg, d_az +0.7 +- 0.3 deg: the photo's
windows sit higher than the exterior table's (a vehicle-config value, [E] from the M2 model, not changed here: reported) or the crew1_1
camera's prior-dominated pitch (+-0.7 deg = 2.3 cm at the windows) and lens (no distortion term) carry it. Reads: 2.5x crops with a 10 px grid
(screenshots/m7-cockpit/fidelity/t9/work/c11_win{L,R}_grid.png).

Ceiling ring: in crew1_1 the forward tunnel's far rim (the dark step line, v 91.5 at u 600) and the closed docking hatch's far edge (the dark
door's lower edge, v 52) are the built circles (ceiling_x_m, tunnel_radius_m) and (tunnel_top_x_m, tunnel_radius_m) (ceiling_step_residuals,
the middle columns). Away from the middle the photo's arcs are flatter than any circle about the axis through this camera (both fitted hatch
views look along the same rays, so an edge's x and r are not separable, and the pinhole camera has no distortion term): the panel band's lower
edge (the bright cove line above the console) fits neither a circle nor a straight chord - it is the recess top and not used for geometry.
The band's radial seams back-projected onto the ceiling plane lie at |az| 157-165 and 128-136 -> pitch 30 +- 3 deg, phase 15 +- 3 ([cabin]
seam_pitch_deg / seam_phase_deg).

Run: python -m cabin_fit.parts.room (from tools/structures) prints the fit, sigmas and per-read residuals (px of 1200).
"""
from __future__ import annotations

import math
import sys
import tomllib
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from .. import annotations
from ..camera import Camera, project

ROOT = Path(__file__).resolve().parents[4]
COCKPIT = ROOT / "config" / "cockpits" / "crew_dragon.toml"
DETAIL = ROOT / "config" / "vehicles" / "detail" / "crew_dragon.toml"
PHOTO = "crew1_1"
READ_SIGMA_PX = 3.0  # [E] edge read on the 1200 px JPEG (2.5x crops, soft rounded rim edges)
RING_N = 240
ASPECT_SIGMA = 1e-4  # the reveal is built round (aspect 1): a free aspect fits 1.17 +- 0.12 (1.4 sigma from round, rms 13.8 vs 14.8 px): not significant
# (v row, u px) per window index (cabin_layout.window_apertures: 2 = az +90 image right, 3 = az -90 image left), per ring and image side
WINDOW_READS = {
    2: {"outer": {"l": {230: 1035, 260: 1025, 290: 1022, 320: 1022},
                  "r": {230: 1180, 260: 1190, 300: 1196, 340: 1195, 370: 1190}},
        "near": {"l": {260: 1056, 300: 1052, 340: 1054}},
        "far": {"r": {260: 1165, 300: 1168, 340: 1166}}},
    3: {"outer": {"l": {230: 55, 260: 40, 300: 30, 330: 24, 360: 22}},
        "near": {"r": {260: 148, 300: 150, 340: 148}},
        "far": {"l": {260: 48, 300: 40, 340: 42}}},  # v 370-390: the dark merges with the s1 headrest's shadow (not read)
}
PRIORS = {"rho0": (0.24, 0.06), "bezel": (0.06, 0.03), "d_x": (0.0, 0.15), "d_az": (0.0, 10.0), "az_common": (0.0, 10.0), "aspect": (1.0, ASPECT_SIGMA)}
# crew1_1 (u column: v row) on the far arcs, the middle columns (c11_ceiling_grid.png): the tunnel's far rim (dark step line) and the
# closed hatch's far edge (lower edge of the dark door)
CEILING_STEP_READS = {
    "tunnel_rim": {550: 89.0, 600: 91.5, 635: 90.0},
    "hatch_edge": {550: 50.0, 600: 52.0, 650: 50.0},
}


def _layout():
    sys.path.insert(0, str(ROOT / "tools" / "structures" / "blender"))
    import cabin_layout as cl
    return cl


def load():
    with open(COCKPIT, "rb") as fh:
        cockpit = tomllib.load(fh)
    with open(DETAIL, "rb") as fh:
        profile = tomllib.load(fh)["capsule"]["profile_m"]
    return cockpit, profile


def camera() -> Camera:
    return Camera.from_row(annotations.camera_row(PHOTO))


def window_rings(cabin, profile, ap, rho0, bezel, n=RING_N, aspect=1.0):
    """(outer = the rim's foot rho0 + bezel, reveal edge rho0) on the wall as (n, 3) arrays: gen_dragon_cabin.window_rings' construction."""
    cl = _layout()
    wall = cabin["wall_m"]
    x_c, az_c = ap["x_m"], ap["az_deg"]
    nr = cl.ext_normal(profile, x_c)[1]
    r_in = cl.lerp(profile, x_c) - wall * nr

    def reveal(rho):
        return np.array([cl.at_depth(profile, x_c + aspect * rho * math.cos(p) * nr, az_c + math.degrees(rho * math.sin(p) / r_in), wall)
                         for p in np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)])

    return reveal(rho0 + bezel), reveal(rho0)


def row_extreme(px, v, side):
    """Left- or right-most u where the closed projected polyline px (n, 2) crosses row v (None if it does not)."""
    us = []
    for a, b in zip(px, np.roll(px, -1, axis=0)):
        if (a[1] - v) * (b[1] - v) <= 0.0 and a[1] != b[1]:
            us.append(a[0] + (b[0] - a[0]) * (v - a[1]) / (b[1] - a[1]))
    if not us:
        return None
    return min(us) if side == "l" else max(us)


def window_residuals(params, cockpit, profile, cam):
    """(read - projected) / READ_SIGMA_PX per read; a row the ring does not reach counts 50 px."""
    rho0, bezel, d_x, d_az, az_common, aspect = params
    cl = _layout()
    out = []
    for ap in cl.window_apertures(cockpit["cabin"]):
        if ap["index"] not in WINDOW_READS:
            continue
        ap = dict(ap, x_m=ap["x_m"] + d_x, az_deg=ap["az_deg"] + math.copysign(d_az, ap["az_deg"]) + az_common)
        outer, inner = window_rings(cockpit["cabin"], profile, ap, rho0, bezel, aspect=aspect)
        rings = {"outer": project(outer, cam), "near": project(inner, cam)}
        rings["far"] = rings["near"]
        for ring, sides in WINDOW_READS[ap["index"]].items():
            for side, reads in sides.items():
                for v, u in reads.items():
                    up = row_extreme(rings[ring], v, side)
                    out.append(((u - up) if up is not None else 50.0) / READ_SIGMA_PX)
    return np.array(out)


def fit_windows(cockpit=None, profile=None, cam=None):
    """({name: (value, sigma)}, per-read residual px, rms px); sigma inflated by sqrt(chi2 / dof) when > 1."""
    if cockpit is None:
        cockpit, profile = load()
    cam = cam or camera()
    names = list(PRIORS)
    p0 = np.array([PRIORS[k][0] for k in names])
    ps = np.array([PRIORS[k][1] for k in names])
    sol = least_squares(lambda p: np.concatenate([window_residuals(p, cockpit, profile, cam), (p - p0) / ps]), p0, x_scale=ps, diff_step=1e-4)
    cov = np.linalg.pinv(sol.jac.T @ sol.jac)
    res_px = window_residuals(sol.x, cockpit, profile, cam) * READ_SIGMA_PX
    dof = max(1, len(res_px) - (len(names) - 1))
    scale = max(1.0, float(np.sqrt(np.sum((res_px / READ_SIGMA_PX) ** 2) / dof)))
    sig = np.sqrt(np.diag(cov)) * scale
    return {k: (float(v), float(s)) for k, v, s in zip(names, sol.x, sig)}, res_px, float(np.sqrt(np.mean(res_px ** 2)))


def built_window_residuals(cockpit=None, profile=None, cam=None, shifts=(0.0, 0.0, 0.0)):
    """Per-read residual px of the windows as configured ([cabin] window_reveal_radius_m, window_bezel_m); `shifts` = (d_x, d_az, az_common)."""
    if cockpit is None:
        cockpit, profile = load()
    c = cockpit["cabin"]
    return window_residuals((c["window_reveal_radius_m"], c["window_bezel_m"], *shifts, 1.0), cockpit, profile, cam or camera()) * READ_SIGMA_PX


def circle_px(x, r, cam, n=720):
    a = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return project(np.stack([np.full(n, x), r * np.cos(a), r * np.sin(a)], axis=1), cam)


def column_far(px, u):
    """Largest v where the projected circle crosses column u (the far arc of a ceiling circle seen from below)."""
    vs = []
    for a, b in zip(px, np.roll(px, -1, axis=0)):
        if (a[0] - u) * (b[0] - u) <= 0.0 and a[0] != b[0]:
            vs.append(a[1] + (b[1] - a[1]) * (u - a[0]) / (b[0] - a[0]))
    return max(vs) if vs else None


def ceiling_step_residuals(cockpit=None, cam=None):
    """{edge: [read - projected v px]} of the built tunnel rim and hatch edge at CEILING_STEP_READS' columns."""
    if cockpit is None:
        cockpit, _ = load()
    cam = cam or camera()
    c = cockpit["cabin"]
    circles = {"tunnel_rim": (c["ceiling_x_m"], c["tunnel_radius_m"]), "hatch_edge": (c["tunnel_top_x_m"], c["tunnel_radius_m"])}
    out = {}
    for name, reads in CEILING_STEP_READS.items():
        px = circle_px(*circles[name], cam)
        out[name] = [v - column_far(px, u) for u, v in reads.items()]
    return out


def main():
    cockpit, profile = load()
    cam = camera()
    vals, res, rms = fit_windows(cockpit, profile, cam)
    print("windows:", {k: f"{v:.4f} +- {s:.4f}" for k, (v, s) in vals.items()}, f"rms {rms:.2f} px", np.round(res, 1))
    print("built windows residual px (table centres):", np.round(built_window_residuals(cockpit, profile, cam), 1))
    shifts = tuple(vals[k][0] for k in ("d_x", "d_az", "az_common"))
    built = built_window_residuals(cockpit, profile, cam, shifts)
    print("built windows residual px (fitted centres):", np.round(built, 1), f"rms {np.sqrt(np.mean(built ** 2)):.2f}")
    for name, r in ceiling_step_residuals(cockpit, cam).items():
        print(f"ceiling {name}: residual px {np.round(r, 1)}")


if __name__ == "__main__":
    main()
