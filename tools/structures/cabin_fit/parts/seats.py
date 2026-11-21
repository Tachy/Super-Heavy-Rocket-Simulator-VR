"""The crew seats (fidelity plan Task 8): the KSC-20211110-0003 camera and the seat shape read on it and on crew1_1.

Camera: KSC-20211110-PH-SPX02_0003 (NASA, public domain) is a second hatch view (Crew-3 on the pad). Its camera is fitted on the console as built
(`ConsoleFixed`: the two ceiling posts and the back face's top edge from cabin_layout.console_posts / console_frame, no free part parameter), the
pose free around the first solve (CAMERA_SIGMA); the row goes to config/cockpits/photo_cameras.toml as `ksc_crew3` (Ruling A).

Seat shape: both hatch photos show the seats in the pad / ingress position (cockpit.md "Discrepancy with crew1_1": the seats sit ~0.3 m higher
toward the camera than the launch posture), so a seat point cannot be placed by the table's hip_m. Their lateral positions are those of the table
(the seats move in the X-Y plane only), so every seat measure here is a RATIO to the seat pitch read in the same image row (the centre distance of
two neighbouring seats, both at the same depth): the ratio is [D] from the photo, the metres follow the table's pitch PITCH_M ([E] +-0.05).
- SHELL_ROWS: the black back's outer edges of the two inner seats (row profiles over 7 rows, 1.5 px Gaussian, strongest step) on the rows where
  it is widest (KSC v 1000 / 1050 at the crew's chest, below that the arms hide it). Width = mean edge distance / centre distance x pitch.
- SHELL_GAP: the gap between the inner seats' backs, KSC (rows 900-1050) and crew1_1 (rows 340-370, its edges at 587-588 / 597; helmet pitch
  HELMET_PITCH_C11 px): two views.
- FRAME_ROWS: the carbon leg-support frame's outer edges at its foot end (crew1_1 v 690, the bar under the soles) and at the knee end (v 550),
  read on the 2x crop; one clean view (KSC agrees within ~10 % but its outer rails are cut / blurred): [E].

Run: python -m cabin_fit.parts.seats (from tools/structures) prints the camera fit and the ratios / table values.
"""
from __future__ import annotations

import math
import sys
import tomllib
from pathlib import Path

import numpy as np

from ..fit import PartModel

ROOT = Path(__file__).resolve().parents[4]
COCKPIT = ROOT / "config" / "cockpits" / "crew_dragon.toml"
PHOTO = "ksc_crew3"
CAMERA_SIGMA = (0.3, 0.2, 0.3)  # [E] around the posts-only first solve (position m, angle rad, f relative): wide, the data fix the pose
PITCH_M = 0.70  # [E] = crew_dragon.toml [[seat]] Z spacing (+-0.05)
PITCH_SIGMA_M = 0.05
# KSC px: (left outer edge, right inner edge) of the image-left inner seat (s3, -Z), (left inner edge, right outer edge) of s2, per row v
SHELL_ROWS = {1000: (1146.0, 1811.0, 1844.0, 2499.0), 1050: (1139.0, 1810.0, 1843.0, 2505.0)}
SHELL_GAP_KSC = (32.0, 696.0)  # gap px (1812 -> 1844, rows 900-1050), pitch px at those rows
SHELL_GAP_C11 = (9.5, 175.0)  # crew1_1: gap px (587.5 -> 597, rows 340-370), helmet pitch px of the inner seats (round 4 read)
READ_SIGMA_PX = 2.0  # [E] one edge read (KSC, 7-row profile); crew1_1 1.5 px edges at 175 px pitch weigh in through their own ratio
# crew1_1 px (2x crop c11_rails): the frame's outer edges (s3 left, s3 right, s2 left, s2 right) at the foot bar (v 690) and the knee end (v 550)
FRAME_ROWS = {"foot": (305.0, 542.0, 645.0, 892.0), "knee": (350.0, 520.0, 655.0, 845.0)}
FRAME_READ_SIGMA_PX = 5.0  # [E] the rails' outer edge on the 1200 px JPEG (dark carbon against the boots / floor)
RAIL_WIDTH_PX = (12.0, 15.0)  # crew1_1: the rail band at the foot end (px), at the foot-bar pitch
# Fix round 1 (closed frame, armrests). Both hatch views look along the seats' symmetry plane: every direction in the X-Y plane (up the back,
# chest normal, along the shin / foot) projects to the image vertical, so a vertical image extent is converted with the camera's px per metre
# along the direction the part is built on, and lateral extents with the px per metre along Z; both scaled to the photo by the ratio of the
# read seat pitch to the projected one (the seats' depth in the photo is not the table's). The seats sit INGRESS_SHIFT_M higher (+X) in both
# photos (cockpit.md "Discrepancy with crew1_1": heads 0.33, foot rests 0.26); at that shift the projected armrests of s2 land on the KSC blocks
# (centre rows 1223 vs read 1225).
INGRESS_SHIFT_M = 0.30  # [E] +-0.10 (the sigma of the projection ratios is taken from 0.20 / 0.40)
# crew1_1 px columns over the foot ends of s3 (380, 420, 460) and s2 (720, 760, 800): (the soles' lower edge = the plate's top face, seen nearly
# edge-on: the first row of the dark sole band; the frame's closing bar's outer edge: its last dark row before the bright floor), luminance
# column profiles (3 columns); the bar between them is 15 px (= the rail width: a round tube section)
FRAME_END_COLS = {380: (677.5, 709.5), 420: (681.5, 710.5), 460: (679.5, 711.0), 720: (681.5, 714.5), 760: (686.0, 715.0), 800: (679.5, 715.5)}
# KSC px, s2 (the image-right inner seat): its armrest blocks (short black padded blocks beside the pan under the forearms): the -Z block's outer
# edges at row 1180 (the +Z block 2605 / 2685 at row 1200 is widened by its own depth along the chest normal, which projects sideways there),
# top / bottom edge at column 1912 (the highlight on its rounded top, the dark contact line above the white pan side)
ARMREST_KSC = {"left": 1885.0, "right": 1967.0, "top": 1165.0, "bottom": 1290.0, "plus_left": 2605.0, "plus_right": 2685.0}
ARMREST_READ_SIGMA_PX = 4.0  # [E] soft dark-on-dark edges on the 3603 px photo
ARMREST_THICK_M = 0.045  # [E] the block's thickness up the back (not seen apart from its length: both cameras see the X-Y plane edge-on); +-0.015


def cockpit():
    return tomllib.loads(COCKPIT.read_text(encoding="utf-8"))


class ConsoleFixed(PartModel):
    """The console as the generator builds it (no free parameter): post tops / bases and the back face's top edge."""

    def __init__(self, ck=None):
        sys.path.insert(0, str(ROOT / "tools" / "structures" / "blender"))
        import cabin_layout as cl
        ck = ck or cockpit()
        self.params = {}
        posts = sorted(cl.console_posts(ck), key=lambda bt: bt[0][2])  # -Z first = image left (image right = +Z in both hatch views)
        f = cl.console_frame(ck)
        o, up, n, right = (np.array(f[k]) for k in ("origin", "up", "normal", "right"))
        d, r = ck["console"]["hood_depth_m"], ck["console"]["edge_radius_m"]
        self._pts = {"post_l_base": np.array(posts[0][0]), "post_l_top": np.array(posts[0][1]),
                     "post_r_base": np.array(posts[1][0]), "post_r_top": np.array(posts[1][1]),
                     # the back face's top edge as seen (lat +-0.5: inside the end caps; right = the crew's right = -Z): the light top fillet meets the
                     # dark flat back face on its tangent line, edge_radius_m below the body's top (the posts-only solve puts the sharp corner u_top
                     # 20-25 px above the read step: the fillet's 4 cm)
                     "back_top_a": o + right * 0.5 + up * (f["u_top"] - r) - n * d, "back_top_b": o - right * 0.5 + up * (f["u_top"] - r) - n * d}

    def points(self, p):
        return dict(self._pts)

    def edges(self):
        return {"post_l": ("post_l_top", "post_l_base"), "post_r": ("post_r_top", "post_r_base"), "back_top": ("back_top_a", "back_top_b")}


def fit_camera():
    from .. import annotations
    from ..fit import fit
    ob = annotations.load(PHOTO)
    if ob.camera_sigma is None:
        ob.camera_sigma = CAMERA_SIGMA
    return fit([ob], ConsoleFixed(), [])


def ratio_width(rows=SHELL_ROWS):
    """Shell width / seat pitch from the inner seats' outer edges per row: (ratio, sigma) (sigma: read sigma through the ratio, scatter)."""
    r = []
    for left_o, left_i, right_i, right_o in rows.values():
        w = 0.5 * ((left_i - left_o) + (right_o - right_i))
        pitch = 0.5 * (right_i + right_o) - 0.5 * (left_o + left_i)
        r.append(w / pitch)
    pitch_px = np.mean([0.5 * (a[2] + a[3]) - 0.5 * (a[0] + a[1]) for a in rows.values()])
    read = READ_SIGMA_PX / pitch_px / math.sqrt(2 * len(rows))
    return float(np.mean(r)), float(math.hypot(read, np.std(r)))


def gap_ratio():
    """Gap / pitch of the inner seats' backs, the two views averaged: (ratio, sigma, per view)."""
    views = [SHELL_GAP_KSC[0] / SHELL_GAP_KSC[1], SHELL_GAP_C11[0] / SHELL_GAP_C11[1]]
    sig = [READ_SIGMA_PX * math.sqrt(2) / SHELL_GAP_KSC[1], 1.5 * math.sqrt(2) / SHELL_GAP_C11[1]]
    w = [1 / s ** 2 for s in sig]
    mean = sum(v * k for v, k in zip(views, w)) / sum(w)
    return mean, math.hypot(1 / math.sqrt(sum(w)), 0.5 * abs(views[0] - views[1])), views


def frame_widths():
    """Leg-support frame outer width / pitch at the foot and knee ends (crew1_1): {end: (ratio, sigma)}."""
    out = {}
    for end, (lo_a, lo_b, hi_a, hi_b) in FRAME_ROWS.items():
        pitch = 0.5 * (hi_a + hi_b) - 0.5 * (lo_a + lo_b)
        ws = [(lo_b - lo_a) / pitch, (hi_b - hi_a) / pitch]
        out[end] = (float(np.mean(ws)), float(math.hypot(FRAME_READ_SIGMA_PX / pitch, 0.5 * abs(ws[0] - ws[1]))))
    pitch = 0.5 * (FRAME_ROWS["foot"][2] + FRAME_ROWS["foot"][3]) - 0.5 * (FRAME_ROWS["foot"][0] + FRAME_ROWS["foot"][1])
    out["rail"] = (float(np.mean(RAIL_WIDTH_PX)) / pitch, 3.0 / pitch)
    return out


def _photo_camera(photo_id):
    from ..camera import Camera
    rows = tomllib.loads((ROOT / "config" / "cockpits" / "photo_cameras.toml").read_text(encoding="utf-8"))["photo"]
    return Camera.from_row(next(r for r in rows if r["id"] == photo_id))


def _seat(ck, sid):
    s = next(s for s in ck["seat"] if s["id"] == sid)
    a = math.radians(s["back_angle_deg"])
    axes = {"b": np.array([math.cos(a), -math.sin(a), 0.0]), "n": np.array([math.sin(a), math.cos(a), 0.0]), "z": np.array([0.0, 0.0, 1.0])}
    return s, np.asarray(s["hip_m"], float), axes


def _jac(cam, p, d):
    """Pixel motion per metre along d at p (2-vector)."""
    from ..camera import project
    return (project((p + 1e-3 * d)[None], cam)[0] - project(p[None], cam)[0]) / 1e-3


def _proj_x(cam, p):
    from ..camera import project
    return float(project(p[None], cam)[0][0])


def frame_end(shift=INGRESS_SHIFT_M, ck=None):
    """The leg-support frame's foot end below the plate's top face along -foot_rest_normal (outer edge of its closing bar), from crew1_1:
    (metres, sigma of the read). The image-vertical px are converted with the camera's vertical px per metre along -normal at s2's plate, scaled
    by the read foot-bar pitch / the projected s3-s2 plate pitch."""
    ck = ck or cockpit()
    cam = _photo_camera("crew1_1")
    up = np.array([shift, 0.0, 0.0])
    s2, _, _ = _seat(ck, "s2")
    s3, _, _ = _seat(ck, "s3")
    p2, p3 = np.asarray(s2["foot_rest_m"], float) + up, np.asarray(s3["foot_rest_m"], float) + up
    read_pitch = 0.5 * (FRAME_ROWS["foot"][2] + FRAME_ROWS["foot"][3]) - 0.5 * (FRAME_ROWS["foot"][0] + FRAME_ROWS["foot"][1])
    corr = read_pitch / abs(_proj_x(cam, p2) - _proj_x(cam, p3))
    jv = abs(_jac(cam, p2, -np.asarray(s2["foot_rest_normal"], float))[1]) * corr
    ext = [b - a for a, b in FRAME_END_COLS.values()]
    return float(np.mean(ext) / jv), float(np.std(ext) / math.sqrt(len(ext)) / jv)


def armrest(shift=INGRESS_SHIFT_M, ck=None):
    """s2's armrest blocks in KSC: {"wide", "length", "out"} metres (width along Z; length along the chest normal with ARMREST_THICK_M up the
    back taken out of the vertical extent; lateral centre beyond the seat's half width), each (value, read sigma). Scale: camera px per metre at
    the block, x the read pitch (SHELL_GAP_KSC) / the projected s3-s2 pitch at the chest row."""
    ck = ck or cockpit()
    cam = _photo_camera(PHOTO)
    up = np.array([shift, 0.0, 0.0])
    s2, h2, ax = _seat(ck, "s2")
    _, h3, _ = _seat(ck, "s3")
    chest = ax["b"] * 0.40  # the shell rows 1000 / 1050 lie at the crew's chest
    corr = SHELL_GAP_KSC[1] / abs(_proj_x(cam, h2 + chest + up) - _proj_x(cam, h3 + chest + up))
    c_up, c_along, c_out = ck["seat_shape"]["armrest_centre_m"]
    hw = 0.5 * s2["width_m"]
    p = h2 + up + ax["b"] * c_up + ax["n"] * c_along - ax["z"] * (hw + c_out)  # the -Z block (image left)
    jz = abs(_jac(cam, p, ax["z"])[0]) * corr
    jn = abs(_jac(cam, p, ax["n"])[1]) * corr
    jb = abs(_jac(cam, p, ax["b"])[1]) * corr
    k = ARMREST_KSC
    sig = ARMREST_READ_SIGMA_PX * math.sqrt(2)
    wide = (k["right"] - k["left"]) / jz
    length = ((k["bottom"] - k["top"]) - ARMREST_THICK_M * jb) / jn
    half_sep = 0.25 * ((k["plus_left"] + k["plus_right"]) - (k["left"] + k["right"])) / jz
    return {"wide": (float(wide), sig / jz), "length": (float(length), float(math.hypot(sig, 0.015 * jb) / jn)),
            "out": (float(half_sep - hw), sig / jz)}


def _with_shift(fn, key=None):
    """(value at INGRESS_SHIFT_M, sigma: read sigma, the shift's +-0.1 spread, the pitch's relative tolerance) of a shift-dependent read."""
    def pick(r):
        return r if key is None else r[key]
    v, s = pick(fn(INGRESS_SHIFT_M))
    lo, hi = pick(fn(INGRESS_SHIFT_M - 0.1))[0], pick(fn(INGRESS_SHIFT_M + 0.1))[0]
    return v, math.sqrt(s * s + (0.5 * abs(hi - lo)) ** 2 + (v * PITCH_SIGMA_M / PITCH_M) ** 2)


def table():
    """The [seat_shape] values in metres with sigma (ratio sigma and the pitch's own tolerance combined)."""
    out = {}
    w, sw = ratio_width()
    g, sg, _ = gap_ratio()
    out["shoulder_width_m (edges)"] = (w * PITCH_M, math.hypot(sw * PITCH_M, w * PITCH_SIGMA_M))
    out["shoulder_width_m (pitch - gap)"] = ((1 - g) * PITCH_M, math.hypot(sg * PITCH_M, (1 - g) * PITCH_SIGMA_M))
    out["seat_gap_m"] = (g * PITCH_M, math.hypot(sg * PITCH_M, g * PITCH_SIGMA_M))
    for end, (r, s) in frame_widths().items():
        out[f"frame_{end}_m"] = (r * PITCH_M, math.hypot(s * PITCH_M, r * PITCH_SIGMA_M))
    out["frame_end_m"] = _with_shift(lambda s: frame_end(s))
    for key in ("wide", "length", "out"):
        out[f"armrest_{key}_m"] = _with_shift(lambda s: armrest(s), key)
    return out


def main():
    res = fit_camera()
    for k, v in res.residual_px.items():
        print(f"residual {k}: {v:.2f} px rms")
    print(f"chi2 {res.chi2:.1f} dof {res.dof}")
    for k, s in res.sigma.items():
        print(f"{k:22s} sigma {s:.4f} data={res.constrained_by_data[k]}")
    cam = res.cameras[PHOTO]
    print(cam.to_row())
    from ..camera import project
    pts = ConsoleFixed().points({})
    from .. import annotations
    ob = annotations.load(PHOTO)
    for lab, uv in ob.points.items():
        p = project(pts[lab][None, :], cam)[0]
        print(f"  {lab:12s} {p[0] - uv[0]:+6.1f} {p[1] - uv[1]:+6.1f}")
    print("shell width / pitch", ratio_width(), "gap / pitch", gap_ratio())
    for k, (v, s) in table().items():
        print(f"{k:32s} {v:.4f} +- {s:.4f}")
    return res


if __name__ == "__main__":
    main()
