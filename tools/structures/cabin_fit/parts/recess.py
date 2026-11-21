"""The wall recess around the console and the console's end contour (fidelity plan Task 7), from crew1_1 (hatch view).

crew1_1 shows the white wall panelling on both sides of the console ending in a light grey bevel (the "reveal") that runs as a straight
diagonal from the ceiling dish down past the console ends to the headrests: a trapezoid recess (wide at the ceiling, narrow below) in which
the console hangs. Two lines per side are read on the photo (column profiles, 1 px Gaussian, every 10 rows):
- REVEAL_OUT_PX: the panel's edge, white -> grey (the bevel's top, on the panel surface),
- REVEAL_IN_PX: the dark line where the bevel meets the recess floor (v 160-290; above v 160 the bevel fades into the lit dish without a line).
Single view: each pixel is back-projected through the fitted crew1_1 camera (config/cockpits/photo_cameras.toml) onto the cabin's inner wall
surface of revolution (crew_dragon.toml [cabin] profile_m; the floor lies PANEL_M further out), giving (x, az) on the wall. Each line is
fitted as |az| = c0 + c1 t + c2 t^2 (t = x - X_REF_M) with c1, c2 shared by both sides (mirror symmetry about the camera's plane Z = 0) and
one c0 per side; c0 is their mean, its sigma combines the fit sigma (read sigma READ_SIGMA_PX through the ray, inflated by the scatter) and
half the left / right difference. The camera's pose sigma (x and pitch prior-dominated, ~3.5 cm / 0.7 deg) shifts both lines together and
is quoted separately (CAMERA_SHIFT).

One image line, two features: in CONSOLE_BAND_V (v 170-220) the console's back-face end stands ~0.8 m in front of the wall along the same
rays, so the dark line there is either the recess floor edge or the console's silhouette, never both. Those rows are left out of the in-edge
fit (in_edge_reads); the clean rows (left 160 + 230-290, right 270-290) fit the mirrored wall line to 0.1 deg rms. Against that line the
band rows read: row 170 inside on both sides (the back top edge), left 180-220 0.6-2.6 px out (at the line: the console end lies at or
inside it, an upper bound), right 180-220 2-5 px out (the console's silhouette covers the wall line; row 180 maps to |lat| 0.77 on the back
plane, beyond the 0.70 end cap: not the back face). The console end taper is therefore fitted only from the right rows SILHOUETTE_V
(190-220) on the back-face plane (o = -hood_depth_m): half width = a + du u (fit_taper_back); the depth slope below the lip is not seen
(dark underside on the dark recess) and is only bounded by the left underside rows UNDER_CLEAN_V. bound_samples: every other contour read
(left back rows, left underside rows) as an upper bound for the built taper. Equality is also where the silver lip band ends on the line on
both sides (|lat| 0.53 at the lip). No second fitted view sees a console end (only crew1_1 / discovery2 / discovery10 have fitted cameras;
the end-on stills nasatour_0310 / 0320 have none).

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
CONSOLE_BAND_V = (170, 220)  # rows where the console's back-face end is seen against the wall line: not in the in-edge fit
SILHOUETTE_SIDE, SILHOUETTE_V = "r", (190, 220)  # rows where the read lies outside the wall line and on the back face: the console's silhouette
BOUND_BACK_V = (190, 220)  # left rows at the wall line: the console end at or inside it (upper bound)
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


def fit_mirrored(per_side):
    """Least squares |az| = c0_side + c1 t + c2 t^2 (t = x - X_REF_M) over {"l"|"r": [(x, az, sigma_az)]}, c1 / c2 shared (mirror):
    (c0_l, c0_r, c1, c2), their sigmas, rms (deg)."""
    rows = [(i, s[0] - X_REF_M, abs(s[1]), s[2]) for i, side in enumerate(("l", "r")) for s in per_side[side]]
    A = np.array([[1.0 - i, float(i)] + [t ** p for p in range(1, EDGE_DEGREE + 1)] for i, t, _, _ in rows])
    y = np.array([r[2] for r in rows])
    w = 1.0 / np.array([r[3] for r in rows]) ** 2
    cov = np.linalg.inv(A.T @ (A * w[:, None]))
    sol = cov @ (A.T @ (w * y))
    res = y - A @ sol
    scale = max(1.0, float(np.sqrt(np.sum(w * res ** 2) / max(1, len(y) - len(sol)))))  # inflate by the scatter when it exceeds the read sigma
    return np.array(sol), np.sqrt(np.diag(cov)) * scale, float(np.sqrt(np.mean(res ** 2)))


def in_edge_reads():
    """REVEAL_IN_PX without the console band rows (CONSOLE_BAND_V): the rows where the dark line is the recess floor edge."""
    lo, hi = CONSOLE_BAND_V
    return {s: {v: u for v, u in REVEAL_IN_PX[s].items() if not lo <= v <= hi} for s in ("l", "r")}


def edge_samples(cam, run, reads, off):
    out = []
    for v, u in reads.items():
        x, az = wall_hit(cam, (u, v), run, off)
        _, az2 = wall_hit(cam, (u + READ_SIGMA_PX, v), run, off)
        out.append((x, az, max(abs(az2 - az), 1e-3)))
    return out


def fit_recess(cockpit=None, cam=None):
    """{"out"|"in": {"coef": [c0, c1, c2] (deg, deg/m, deg/m2), "sigma": [...], "lr_deg": left / right difference at x_ref,
    "rms_deg", "x": (min, max)}}: mirrored fit (fit_mirrored), the in edge from the clean rows only (in_edge_reads)."""
    cockpit = cockpit or load_cockpit()
    cam = cam or camera()
    run = side_wall(cockpit["cabin"])
    result = {}
    for name, reads, off in (("out", REVEAL_OUT_PX, 0.0), ("in", in_edge_reads(), PANEL_M)):
        sol, sig, rms = fit_mirrored({s: edge_samples(cam, run, reads[s], off) for s in ("l", "r")})
        xs = [wall_hit(cam, (u, v), run, off)[0] for s in ("l", "r") for v, u in reads[s].items()]
        lr = float(abs(sol[0] - sol[1]))
        coef = [0.5 * (sol[0] + sol[1])] + list(sol[2:])
        sigma = [math.hypot(0.5 * math.hypot(sig[0], sig[1]), 0.5 * lr)] + list(sig[2:])
        result[name] = {"coef": [float(v) for v in coef], "sigma": [float(v) for v in sigma], "lr_deg": lr, "rms_deg": rms,
                        "x": (min(xs), max(xs))}
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


def body_planes(cockpit):
    """(u_top, u_bot, u_lip, hood): the console body's back-face span along up and the hood depth (frame of console_frame)."""
    con = cockpit["console"]
    centre, up, _, _ = console_frame(cockpit)
    body_c = np.array(con["centre_m"], float)
    uc = float(np.dot(body_c - centre, up))
    u_top, u_bot = uc + 0.5 * con["height_m"], uc - 0.5 * con["height_m"]
    drop = con["underside_profile"][-1][1]
    return u_top, u_bot, u_bot - drop, con["hood_depth_m"]


def _back_read(cam, centre, up, n, right, hood, u, v):
    """(u_up, half_lat, sigma) where the ray through (u, v) meets the back-face plane (o = -hood)."""
    p0 = centre - n * hood
    p = plane_hit(cam, (u, v), p0, n)
    q = plane_hit(cam, (u + READ_SIGMA_PX, v), p0, n)
    w = abs(float(np.dot(p - centre, right)))
    return float(np.dot(p - centre, up)), w, abs(abs(float(np.dot(q - centre, right))) - w)


def _under_plane(cockpit):
    """(point, normal) of the underside plane through the front bottom edge (o 0, u_bot) and the back lip (o -hood, u_lip)."""
    centre, up, n, right = console_frame(cockpit)
    _, u_bot, u_lip, hood = body_planes(cockpit)
    a = centre + up * u_bot
    b = centre + up * u_lip - n * hood
    nn = np.cross(b - a, right)
    return a, nn / np.linalg.norm(nn)


def silhouette_samples(cockpit=None, cam=None):
    """[(v, u, half_lat, sigma)]: the console silhouette rows (SILHOUETTE_SIDE, SILHOUETTE_V) on the back-face plane: equalities."""
    cockpit = cockpit or load_cockpit()
    cam = cam or camera()
    centre, up, n, right = console_frame(cockpit)
    hood = body_planes(cockpit)[3]
    lo, hi = SILHOUETTE_V
    return [(v, *_back_read(cam, centre, up, n, right, hood, u, v)) for v, u in sorted(REVEAL_IN_PX[SILHOUETTE_SIDE].items()) if lo <= v <= hi]


def bound_samples(cockpit=None, cam=None):
    """[(u, depth, half_lat, sigma)]: contour reads that only bound the console end from outside (the end lies at or inside them): the left
    back-face rows BOUND_BACK_V (at the wall line) and the left underside rows UNDER_CLEAN_V (the right side's rows 230-260 are covered by
    the wall camera's mount, rows > 260 by the headrests)."""
    cockpit = cockpit or load_cockpit()
    cam = cam or camera()
    centre, up, n, right = console_frame(cockpit)
    hood = body_planes(cockpit)[3]
    out = []
    for v, u in sorted(REVEAL_IN_PX["l"].items()):
        if BOUND_BACK_V[0] <= v <= BOUND_BACK_V[1]:
            uu, w, s = _back_read(cam, centre, up, n, right, hood, u, v)
            out.append((uu, hood, w, s))
    a, under_n = _under_plane(cockpit)
    for v, u in sorted(REVEAL_IN_PX["l"].items()):
        if UNDER_CLEAN_V[0] <= v <= UNDER_CLEAN_V[1]:
            p = plane_hit(cam, (u, v), a, under_n)
            q = plane_hit(cam, (u + READ_SIGMA_PX, v), a, under_n)
            w = abs(float(np.dot(p - centre, right)))
            out.append((float(np.dot(p - centre, up)), -float(np.dot(p - centre, n)), w, abs(abs(float(np.dot(q - centre, right))) - w)))
    return out


def fit_taper_back(samples):
    """Line half_lat = a + du u on the back-face plane over silhouette_samples: ((a, du), (sigmas), rms_m). a = w0 + dw_ddepth hood_depth_m
    of [console] end_taper; the depth slope is not constrained here (bound_samples only)."""
    A = np.array([[1.0, s[1]] for s in samples])
    y = np.array([s[2] for s in samples])
    w = 1.0 / np.array([s[3] for s in samples]) ** 2
    cov = np.linalg.inv(A.T @ (A * w[:, None]))
    sol = cov @ (A.T @ (w * y))
    res = y - A @ sol
    scale = max(1.0, float(np.sqrt(np.sum(w * res ** 2) / max(1, len(y) - 2))))
    return tuple(float(v) for v in sol), tuple(float(math.sqrt(cov[i, i]) * scale) for i in range(2)), float(np.sqrt(np.mean(res ** 2)))


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
    sil = silhouette_samples(cockpit)
    for row in sil:
        print("silhouette v %d: u %.4f half lat %.4f +- %.4f" % row)
    taper = fit_taper_back(sil)
    print("end taper on the back face (a = w0 + dw_ddepth hood, dw/du)", np.round(taper[0], 4), "+-", np.round(taper[1], 4),
          f"rms {taper[2]:.4f} m")
    con = cockpit["console"]
    for row in bound_samples(cockpit):
        u, depth, w, s = row
        w0, du, dd = con["end_taper"]
        print("bound u %.4f depth %.4f half lat %.4f +- %.4f; built taper %.4f" % (u, depth, w, s, w0 + du * u + dd * depth))
    if "recess" in cockpit:
        built = recess_outline_px(cockpit)
        clean = in_edge_reads()
        for key, reads in (("out_l", REVEAL_OUT_PX["l"]), ("out_r", REVEAL_OUT_PX["r"]), ("in_l", clean["l"]), ("in_r", clean["r"])):
            rms, worst = line_residual_px(built[key], reads)
            print(f"built {key}: {rms:.1f} px rms, max {worst:.1f} px")
    return res, sil, taper


if __name__ == "__main__":
    main()
