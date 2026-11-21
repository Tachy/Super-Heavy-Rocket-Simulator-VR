"""The wall recess around the console and the console's end contour (fidelity plan Task 7), from crew1_1 (hatch view).

crew1_1 shows the white wall panelling on both sides of the console ending in a light grey bevel (the "reveal") that runs as a straight
diagonal from the ceiling dish down past the console ends to the headrests: a trapezoid recess (wide at the ceiling, narrow below) in which
the console hangs. Two lines per side are read on the photo (column profiles, 1 px Gaussian, every 10 rows):
- REVEAL_OUT_PX: the panel's edge, white -> grey (the bevel's top, on the panel surface),
- REVEAL_IN_PX: the dark line where the bevel meets the recess floor (v 160-290; above v 160 the bevel fades into the lit dish without a line).
Single view: each pixel is back-projected through the fitted crew1_1 camera (config/cockpits/photo_cameras.toml) onto the cabin's inner wall
surface of revolution (crew_dragon.toml [cabin] profile_m; the floor lies PANEL_M further out), giving (x, az) on the wall. The two lines are
fitted per side as az = az_c + k (x - X_REF_M) and averaged over both sides (mirror symmetry about the camera's plane Z = 0); the sigma
combines the read sigma (READ_SIGMA_PX through the ray), the fit scatter and the left / right difference. The camera's pose sigma (x and
pitch prior-dominated, ~3.5 cm / 0.7 deg) shifts both lines together and is quoted separately (CAMERA_SHIFT).

The console end: the same dark line passes along the console's back-face end from the back top edge (v ~172) to the lip (v ~221): the end
cap's highlight touches it at mid height (lat 0.70-0.72) and the silver lip band ends exactly on it on both sides (lat 0.51). Below the lip
(v 221-290) the underside is dark on the dark recess: the line is an upper bound for the underside's ends there. END_BACK: the ray through
the contour meets the back-face plane (o = -hood_depth_m) at (lat, u); END_UNDER: it meets the underside plane at (lat, depth).

Run: python -m cabin_fit.parts.recess (from tools/structures) prints the fit and the table values.
"""
from __future__ import annotations

import math
import tomllib
from pathlib import Path

import numpy as np

from .. import annotations
from ..camera import Camera, project, unproject

ROOT = Path(__file__).resolve().parents[4]
COCKPIT = ROOT / "config" / "cockpits" / "crew_dragon.toml"
PHOTO = "crew1_1"
READ_SIGMA_PX = 2.0  # [E] line position on the 1200 px JPEG (1 px Gaussian; the bevel's edges are 2-4 px soft)
X_REF_M = 3.0  # reference station of the edge lines
PANEL_M = 0.03  # [E] the recess floor lies this far outside the panel surface (the bevel's depth; single view: not measurable, +-0.02)
# crew1_1 px (u at row v), image left = -Z (the crew's right, az < 0), image right = +Z
REVEAL_OUT_PX = {
    "l": {100: 223.5, 110: 232.5, 120: 240.5, 130: 248.5, 140: 255.5, 150: 264.5, 160: 272.5, 170: 280.5, 180: 288.5, 190: 296.5, 200: 304.5,
          210: 313.5, 220: 321.5, 230: 329.5, 240: 337.5},
    "r": {100: 967.5, 110: 957.5, 120: 949.5, 130: 942.5, 140: 934.5, 150: 926.5, 160: 918.5, 170: 909.5, 180: 901.5, 190: 893.5, 200: 885.5,
          210: 877.5, 220: 869.5, 230: 860.5, 240: 852.5},
}
REVEAL_IN_PX = {
    "l": {160: 316, 170: 327, 180: 328, 190: 333, 200: 342, 210: 349, 220: 355, 230: 365, 240: 373, 250: 380, 260: 388, 270: 397, 280: 405, 290: 413},
    "r": {170: 858, 180: 863, 190: 858, 200: 848, 210: 841, 220: 834, 270: 793, 280: 784, 290: 776},  # v 230-260: the wall camera's mount covers it
}
END_BACK_V = (186, 221)  # rows where the contour is the console's back-face end (end cap at mid height .. the lip)
END_UNDER_V = (226, 290)  # rows where it bounds the underside (dark on dark: an upper bound)
UNDER_CLEAN_V = (230, 260)  # underside rows read on the left only: the bevel's smooth light-to-dark gradient shows no console silhouette outside the line
CAMERA_SHIFT = {"position_m": 0.035, "pitch_deg": 0.7}  # [E] crew1_1 prior-dominated pose components (photo_cameras.toml)


def load_cockpit(path=COCKPIT):
    with open(path, "rb") as fh:
        return tomllib.load(fh)


def camera() -> Camera:
    return Camera.from_row(annotations.camera_row(PHOTO))


def side_wall(cabin):
    """The inner wall's (x, r) run from the floor rim to the ceiling ring (profile_m)."""
    prof = cabin["profile_m"]
    out = [prof[1]]
    for p in prof[2:]:
        out.append(p)
        if abs(p[0] - cabin["ceiling_x_m"]) < 1e-9:
            break
    return np.array(out, float)


def wall_r(run, x, off=0.0):
    return float(np.interp(x, run[:, 0], run[:, 1])) + off


def wall_hit(cam, px, run, off=0.0):
    """(x, az_deg) where the ray through px first leaves the cabin through the wall surface r = r_wall(x) + off (bisection)."""
    d = unproject(np.asarray([px], float), cam)[0]
    o = cam.position_m

    def f(t):
        p = o + d * t
        return math.hypot(p[1], p[2]) - wall_r(run, p[0], off)

    t0, t1 = 0.05, 0.05
    while f(t1) < 0.0:
        t0, t1 = t1, t1 + 0.05
        if t1 > 8.0:
            raise ValueError(f"ray {px} does not reach the wall")
    for _ in range(60):
        tm = 0.5 * (t0 + t1)
        if f(tm) < 0.0:
            t0 = tm
        else:
            t1 = tm
    p = o + d * t1
    return float(p[0]), math.degrees(math.atan2(p[2], p[1]))


def wall_point(run, x, az_deg, off=0.0):
    r = wall_r(run, x, off)
    a = math.radians(az_deg)
    return np.array([x, r * math.cos(a), r * math.sin(a)])


EDGE_DEGREE = 2  # the straight image lines map to curves on the cone: |az| quadratic in x (linear left 1-2 deg rms, 5-19 px in the photo)


def fit_line(samples):
    """Least squares |az| = c0 + c1 (x - X_REF_M) + c2 (x - X_REF_M)^2 over [(x, az, sigma_az)]: (coefficients, sigmas, rms)."""
    x = np.array([s[0] for s in samples]) - X_REF_M
    y = np.array([abs(s[1]) for s in samples])
    w = 1.0 / np.array([s[2] for s in samples]) ** 2
    A = np.stack([x ** p for p in range(EDGE_DEGREE + 1)], axis=1)
    cov = np.linalg.inv(A.T @ (A * w[:, None]))
    sol = cov @ (A.T @ (w * y))
    res = y - A @ sol
    rms = float(np.sqrt(np.mean(res ** 2)))
    scale = max(1.0, float(np.sqrt(np.sum(w * res ** 2) / max(1, len(y) - len(sol)))))  # inflate by the scatter when it exceeds the read sigma
    return np.array(sol), np.sqrt(np.diag(cov)) * scale, rms


def edge_samples(cam, run, reads, off):
    out = []
    for v, u in reads.items():
        x, az = wall_hit(cam, (u, v), run, off)
        _, az2 = wall_hit(cam, (u + READ_SIGMA_PX, v), run, off)
        out.append((x, az, max(abs(az2 - az), 1e-3)))
    return out


def fit_recess(cockpit=None, cam=None):
    """{"out"|"in": {"coef": [c0, c1, c2] (deg, deg/m, deg/m2), "sigma": [...], "lr_deg": left / right difference at x_ref,
    "rms_deg", "x": (min, max)}}: both sides fitted separately, then averaged (mirror symmetry)."""
    cockpit = cockpit or load_cockpit()
    cam = cam or camera()
    run = side_wall(cockpit["cabin"])
    result = {}
    for name, reads, off in (("out", REVEAL_OUT_PX, 0.0), ("in", REVEAL_IN_PX, PANEL_M)):
        per_side = {s: fit_line(edge_samples(cam, run, reads[s], off)) for s in ("l", "r")}
        xs = [wall_hit(cam, (u, v), run, off)[0] for s in ("l", "r") for v, u in reads[s].items()]
        (cl_, sl, rl), (cr, sr, rr) = per_side["l"], per_side["r"]
        coef = 0.5 * (cl_ + cr)
        sigma = np.hypot(0.5 * np.hypot(sl, sr), 0.5 * np.abs(cl_ - cr))
        result[name] = {"coef": [float(v) for v in coef], "sigma": [float(v) for v in sigma], "lr_deg": float(abs(cl_[0] - cr[0])),
                        "rms_deg": max(rl, rr), "x": (min(xs), max(xs))}
    return result


def console_frame(cockpit):
    con = cockpit["console"]
    t = math.radians(con["tilt_deg"])
    up = np.array([math.cos(t), -math.sin(t), 0.0])
    n = np.array([-math.sin(t), -math.cos(t), 0.0])
    right = np.array([0.0, 0.0, -1.0])
    centre = np.array([s for s in cockpit["screen"] if s["id"] == "centre"][0]["centre_m"], float)
    return centre, up, n, right


def plane_hit(cam, px, p0, normal):
    d = unproject(np.asarray([px], float), cam)[0]
    t = float(np.dot(p0 - cam.position_m, normal) / np.dot(d, normal))
    return cam.position_m + d * t


def _contour_rows(lo, hi):
    rows = {}
    for v, u in REVEAL_IN_PX["l"].items():
        if lo <= v <= hi and v in REVEAL_IN_PX["r"]:
            rows[v] = (u, REVEAL_IN_PX["r"][v])
    return rows


def body_planes(cockpit):
    """(u_top, u_bot, u_lip, hood): the console body's back-face span along up and the hood depth (frame of console_frame)."""
    con = cockpit["console"]
    centre, up, _, _ = console_frame(cockpit)
    body_c = np.array(con["centre_m"], float)
    uc = float(np.dot(body_c - centre, up))
    u_top, u_bot = uc + 0.5 * con["height_m"], uc - 0.5 * con["height_m"]
    drop = con["underside_profile"][-1][1]
    return u_top, u_bot, u_bot - drop, con["hood_depth_m"]


def measure_ends(cockpit=None, cam=None):
    """{"back": [(u, half_lat, sigma)], "under": [(depth, half_lat, sigma)]}: the console's end contour on the back face and the underside."""
    cockpit = cockpit or load_cockpit()
    cam = cam or camera()
    centre, up, n, right = console_frame(cockpit)
    u_top, u_bot, u_lip, hood = body_planes(cockpit)
    back_p0 = centre - n * hood
    out = {"back": [], "under": []}
    for v, (ul, ur) in sorted(_contour_rows(*END_BACK_V).items()):
        lats, us = [], []
        for u in (ul, ur):
            p = plane_hit(cam, (u, v), back_p0, n)
            lats.append(abs(float(np.dot(p - centre, right))))
            us.append(float(np.dot(p - centre, up)))
        q = plane_hit(cam, (ul + READ_SIGMA_PX, v), back_p0, n)
        s = abs(abs(float(np.dot(q - centre, right))) - lats[0])
        out["back"].append((float(np.mean(us)), float(np.mean(lats)), math.hypot(s, 0.5 * abs(lats[0] - lats[1]))))
    # the underside: the plane through the front bottom edge (o 0, u_bot) and the back lip (o -hood, u_lip)
    a = centre + up * u_bot
    b = centre + up * u_lip - n * hood
    under_n = np.cross(b - a, right)
    under_n /= np.linalg.norm(under_n)
    for v, (ul, ur) in sorted(_contour_rows(*END_UNDER_V).items()):
        lats, deps = [], []
        for u in (ul, ur):
            p = plane_hit(cam, (u, v), a, under_n)
            lats.append(abs(float(np.dot(p - centre, right))))
            deps.append(-float(np.dot(p - centre, n)))
        q = plane_hit(cam, (ul + READ_SIGMA_PX, v), a, under_n)
        s = abs(abs(float(np.dot(q - centre, right))) - lats[0])
        out["under"].append((float(np.mean(deps)), float(np.mean(lats)), math.hypot(s, 0.5 * abs(lats[0] - lats[1]))))
    return out


def end_samples(cockpit=None, cam=None):
    """[(u, depth, half_lat, sigma)] of the end contour: back-face rows and the clean underside rows (UNDER_CLEAN_V, left side: the right
    side's rows 230-260 are covered by the wall camera's mount, rows > 260 by the headrests)."""
    cockpit = cockpit or load_cockpit()
    cam = cam or camera()
    centre, up, n, right = console_frame(cockpit)
    u_top, u_bot, u_lip, hood = body_planes(cockpit)
    out = [(u, hood, w, s) for u, w, s in measure_ends(cockpit, cam)["back"]]
    a = centre + up * u_bot
    b = centre + up * u_lip - n * hood
    under_n = np.cross(b - a, right)
    under_n /= np.linalg.norm(under_n)
    for v, u in REVEAL_IN_PX["l"].items():
        if UNDER_CLEAN_V[0] <= v <= UNDER_CLEAN_V[1]:
            p = plane_hit(cam, (u, v), a, under_n)
            q = plane_hit(cam, (u + READ_SIGMA_PX, v), a, under_n)
            w = abs(float(np.dot(p - centre, right)))
            out.append((float(np.dot(p - centre, up)), -float(np.dot(p - centre, n)), w, abs(abs(float(np.dot(q - centre, right))) - w)))
    return out


def fit_taper(samples):
    """Plane half_lat = w0 + dw_du u + dw_ddepth depth: ((w0, dw_du, dw_ddepth), (sigmas), rms_m)."""
    A = np.array([[1.0, s[0], s[1]] for s in samples])
    y = np.array([s[2] for s in samples])
    w = 1.0 / np.array([s[3] for s in samples]) ** 2
    cov = np.linalg.inv(A.T @ (A * w[:, None]))
    sol = cov @ (A.T @ (w * y))
    res = y - A @ sol
    scale = max(1.0, float(np.sqrt(np.sum(w * res ** 2) / max(1, len(y) - 3))))
    return tuple(float(v) for v in sol), tuple(float(math.sqrt(cov[i, i]) * scale) for i in range(3)), float(np.sqrt(np.mean(res ** 2)))


def edge_az(rec, name, x):
    """|az| (deg) of the built recess edge `name` ("out" | "in") at station x: the fitted curve inside its station range, constant outside
    (= cabin_layout.recess_edge_az)."""
    lo, hi = rec[f"{name}_x_m"]
    t = min(max(x, lo), hi) - rec["x_ref_m"]
    return sum(c * t ** p for p, c in enumerate(rec[f"edge_{name}_deg"]))


def recess_outline_px(cockpit, cam=None, n=16):
    """The built recess edges (crew_dragon.toml [recess]) projected into crew1_1: {"out_l": array (n, 2), ...} over their read station range."""
    cam = cam or camera()
    rec = cockpit["recess"]
    run = side_wall(cockpit["cabin"])
    out = {}
    for name, off in (("out", 0.0), ("in", rec["depth_m"])):
        lo, hi = rec[f"{name}_x_m"]
        for s, sign in (("l", -1.0), ("r", 1.0)):
            pts = [project(wall_point(run, x, sign * edge_az(rec, name, x), off)[None], cam)[0] for x in np.linspace(lo, hi, n)]
            out[f"{name}_{s}"] = np.array(pts)
    return out


def line_residual_px(built_px, reads):
    """rms distance (px) of the read points {v: u} from the built polyline (u, v)."""
    d = []
    for v, u in reads.items():
        p = np.array([u, v], float)
        best = 1e9
        for a, b in zip(built_px[:-1], built_px[1:]):
            ab = b - a
            t = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0))
            best = min(best, float(np.linalg.norm(p - a - t * ab)))
        d.append(best)
    return float(np.sqrt(np.mean(np.square(d)))), float(np.max(d))


def main():
    cockpit = load_cockpit()
    res = fit_recess(cockpit)
    for name, r in res.items():
        print(f"{name}: coef {np.round(r['coef'], 2)} +- {np.round(r['sigma'], 2)} (deg, deg/m, deg/m2 at x {X_REF_M}), "
              f"l/r {r['lr_deg']:.2f} deg, rms {r['rms_deg']:.2f} deg, x {r['x'][0]:.3f}..{r['x'][1]:.3f}")
    ends = measure_ends(cockpit)
    for k, rows in ends.items():
        for row in rows:
            print(k, " ".join(f"{v:.4f}" for v in row))
    taper = fit_taper(end_samples(cockpit))
    print("end taper (w0, dw/du, dw/ddepth)", np.round(taper[0], 4), "+-", np.round(taper[1], 4), f"rms {taper[2]:.4f} m")
    if "recess" in cockpit:
        built = recess_outline_px(cockpit)
        for key, reads in (("out_l", REVEAL_OUT_PX["l"]), ("out_r", REVEAL_OUT_PX["r"]), ("in_l", REVEAL_IN_PX["l"]), ("in_r", REVEAL_IN_PX["r"])):
            rms, worst = line_residual_px(built[key], reads)
            print(f"built {key}: {rms:.1f} px rms, max {worst:.1f} px")
    return res, ends, taper


if __name__ == "__main__":
    main()
