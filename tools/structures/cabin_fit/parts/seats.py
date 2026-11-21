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
