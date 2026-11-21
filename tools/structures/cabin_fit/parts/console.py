"""Console PartModel (fidelity plan Task 4): the housing block, its back face and the two ceiling posts.

Frame: the cabin frame of config/cockpits/crew_dragon.toml. The console block is placed by the centre screen centre (cx, cy, 0) and
the tilt t of its up axis from +X toward -Y: up = (cos t, -sin t, 0), normal n (toward the crew) = (-sin t, -cos t, 0), right as
the crew sees it = (0, 0, -1). A console point is P(lat, u, o) = c + right lat + up u + n o (o < 0 behind the screen plane).

Parameters (prior, prior sigma): the block (cx, cy, tilt_deg: round 4), the housing (width_m at the screen plane, side_draft_deg:
the side faces lean in toward the back, hood_depth_m: the back face behind the screen plane, u_top: the body's top edge along up,
u_bot: the front bottom edge (shelf bottom), back_drop_m: the back face's bottom edge lies this far below u_bot along up), the posts
(post_z_m: base |Z|, post_base_o_m: base position behind the screen plane on the body top, post_top_y_m / post_top_z_m: the top
on the ceiling plane x = CEILING_X_M).

Run: python -m cabin_fit.parts.console (from tools/structures) prints the fit; the photos used are PHOTOS.
"""
from __future__ import annotations

import math
import sys

import numpy as np

from ..fit import PartModel

CEILING_X_M = 3.48  # [E] = crew_dragon.toml [cabin] ceiling_x_m (round 4): the posts end on this plane
SCREEN_HALF_M = (0.198, 0.1145)  # [D] prior of the glass outline (the round-4 active area 0.382 x 0.215 + 0.007 bezel); Task 5 fits both halves (0.2000 x 0.1136)
TABLET_M = (0.1348, 0.2032)  # [S] Apple iPad mini 7.9in body (cabin_fit/anchors.py KNOWN ipad_mini_short_m / ipad_mini_long_m, sigma 2 mm)
PHOTOS = ("crew1_1", "discovery2", "discovery10")
# [E] round 4 pose uncertainty (photo_cameras.toml rows); discovery10 = another frame of discovery2's fixed GoPro mount, same prior
CAMERA_SIGMA = {"crew1_1": (0.05, 0.02, 0.02), "discovery2": (0.05, 0.035, 0.03), "discovery10": (0.05, 0.035, 0.03)}


class Console(PartModel):
    def __init__(self):
        self.params = {
            "cx": (2.512, 0.06), "cy": (-0.043, 0.06), "tilt_deg": (45.0, 4.0),
            "width_m": (1.32, 0.05), "side_draft_deg": (10.0, 15.0), "hood_depth_m": (0.15, 0.10),
            "u_top": (0.265, 0.05), "u_bot": (-0.225, 0.03), "back_drop_m": (0.0, 0.15),
            "screen_half_w_m": (SCREEN_HALF_M[0], 0.03), "screen_pitch_m": (0.402, 0.04), "screen_half_h_m": (SCREEN_HALF_M[1], 0.03),
            # Task 5: the iPad minis on their mounts above the outer screens (scale anchor): body size [S] (cabin_fit.anchors ipad_mini_*), pose free
            # in the console frame (lat, u, o = proud of the screen plane, roll in the plane); priors = crew_dragon.toml [[tablet]] (round 4) +- wide
            "tab_w_m": (TABLET_M[0], 0.002), "tab_h_m": (TABLET_M[1], 0.002),
            "tab_l_lat": (-0.367, 0.08), "tab_l_u": (0.228, 0.06), "tab_l_o": (0.06, 0.03), "tab_l_roll_deg": (0.0, 10.0),
            "tab_r_lat": (0.367, 0.08), "tab_r_u": (0.228, 0.06), "tab_r_o": (0.06, 0.03), "tab_r_roll_deg": (0.0, 10.0),
            "post_z_m": (0.55, 0.08), "post_base_o_m": (0.10, 0.05),             # the top on the ceiling: radius prior = inside the ring with the collar clear of the wall (crew1_1: the collars sit ~15-20 px
            # above the dish's lower boundary at the wall; ceiling outer radius 0.997 minus a ~0.04 collar), angle prior wide
            "post_top_r_m": (0.95, 0.03), "post_top_a_deg": (35.0, 20.0),
        }

    @staticmethod
    def frame(p):
        t = math.radians(p["tilt_deg"])
        c = np.array([p["cx"], p["cy"], 0.0])
        up = np.array([math.cos(t), -math.sin(t), 0.0])
        n = np.array([-math.sin(t), -math.cos(t), 0.0])
        right = np.array([0.0, 0.0, -1.0])
        return c, up, n, right

    def points(self, p):
        c, up, n, right = self.frame(p)

        def P(lat, u, o):
            return c + right * lat + up * u + n * o

        out = {}
        hw, hh = p["screen_half_w_m"], p["screen_half_h_m"]
        for side in ("l", "r"):  # tablet body corners (tl = the crew's left, top)
            rl = math.radians(p[f"tab_{side}_roll_deg"])
            c_r, s_r = math.cos(rl), math.sin(rl)
            for name, (a, b) in (("tl", (-1, 1)), ("tr", (1, 1)), ("br", (1, -1)), ("bl", (-1, -1))):
                x, y = 0.5 * a * p["tab_w_m"], 0.5 * b * p["tab_h_m"]
                out[f"tab_{side}_{name}"] = P(p[f"tab_{side}_lat"] + c_r * x - s_r * y, p[f"tab_{side}_u"] + s_r * x + c_r * y, p[f"tab_{side}_o"])
        for sid, k in (("left", -1.0), ("centre", 0.0), ("right", 1.0)):
            lat = k * p["screen_pitch_m"]  # image tl = crew's left (-right), top (+up)
            out[f"scr_{sid}_tl"] = P(lat - hw, hh, 0.0)
            out[f"scr_{sid}_tr"] = P(lat + hw, hh, 0.0)
            out[f"scr_{sid}_br"] = P(lat + hw, -hh, 0.0)
            out[f"scr_{sid}_bl"] = P(lat - hw, -hh, 0.0)
        w = p["width_m"] / 2.0
        d = p["hood_depth_m"]
        wb = w - d * math.tan(math.radians(p["side_draft_deg"]))
        u_bb = p["u_bot"] - p["back_drop_m"]
        out["front_bot_l"], out["front_bot_r"] = P(-w, p["u_bot"], 0.0), P(w, p["u_bot"], 0.0)
        out["back_top_l"], out["back_top_r"] = P(-wb, p["u_top"], -d), P(wb, p["u_top"], -d)
        out["back_bot_l"], out["back_bot_r"] = P(-wb, u_bb, -d), P(wb, u_bb, -d)
        u_mid = 0.5 * (p["u_top"] + u_bb)
        out["back_end_l"], out["back_end_r"] = P(-wb, u_mid, -d), P(wb, u_mid, -d)
        # posts: image left in crew1_1 (camera looks -Y, image right = +Z) is the -Z post; "l" / "r" follow crew1_1's image
        for side, sz in (("l", -1.0), ("r", 1.0)):
            out[f"post_{side}_base"] = P(-sz * p["post_z_m"], p["u_top"], -p["post_base_o_m"])
            out[f"post_{side}_top"] = self.post_top(p, sz)
        return out

    @staticmethod
    def post_top(p, sz=1.0):
        """Post top on the ceiling plane: radius post_top_r_m, post_top_a_deg off the -Y axis toward sz * Z."""
        a = math.radians(p["post_top_a_deg"])
        return np.array([CEILING_X_M, -p["post_top_r_m"] * math.cos(a), sz * p["post_top_r_m"] * math.sin(a)])

    def edges(self):
        return {"front_bottom": ("front_bot_l", "front_bot_r"), "back_top": ("back_top_l", "back_top_r"),
                "back_bottom": ("back_bot_l", "back_bot_r"), "post_l": ("post_l_top", "post_l_base"), "post_r": ("post_r_top", "post_r_base")}


# crew1_1 widths of the posts (px, full width across the axis, read on the 8x crops; +-1.5 px incl. the JPEG blur), at the fraction t of
# the post's image length from the top (v 112 top, v 172 base): the collar (joint head) at the ceiling, the rod, the sleeve above the
# funnel, the funnel's foot on the housing; lengths along the axis from the image length (t) times the fitted 3D length.
WIDTHS_PX = {"collar": (0.02, (15.0, 16.0)), "rod": (0.30, (7.5, 7.5)), "sleeve": (0.58, (10.0, 11.0)), "foot": (1.0, (29.0, 28.0))}
SPANS_T = {"sleeve": (0.55, 0.63), "funnel": (0.63, 1.0)}  # crew1_1 v 145-150 sleeve, 150-172 funnel (fractions of 112..172)


def post_sizes(res):
    """Radii (m) and lengths (m) of the post parts from WIDTHS_PX / SPANS_T at the fitted camera and post (left / right mean)."""
    cam = res.cameras["crew1_1"]
    pts = Console().points(res.params)
    out = {}
    for name, (t, widths) in WIDTHS_PX.items():
        r = []
        for side, wpx in zip(("l", "r"), widths):
            p = pts[f"post_{side}_top"] * (1 - t) + pts[f"post_{side}_base"] * t
            r.append(0.5 * wpx * np.linalg.norm(p - cam.position_m) / cam.f_px)
        out[f"{name}_radius_m"] = float(np.mean(r))
    length = float(np.linalg.norm(pts["post_l_top"] - pts["post_l_base"]))
    out["post_length_m"] = length
    for name, (t0, t1) in SPANS_T.items():
        out[f"{name}_length_m"] = (t1 - t0) * length
    return out


# As built (crew_dragon.toml [console]): parameters the data do not dominate keep the table's value (round 4 block placement and width,
# the top edge; [E] call for the draft: the free fit reads -6 +- 10 deg); the "as built" fit holds them there (FIXED_SIGMA) so the
# cameras written back to photo_cameras.toml and the residuals belong to the geometry the generator builds.
# post_top_r_m (prior-dominated, crew1_1 sees the top along its ray): 0.945 keeps the collar's flush cut (an ellipse on the ceiling plane, its
# outer point ~0.049 m past the top) inside the ring's outer radius 0.997 (fix round 1, review: the collar poked ~2 cm through the wall at 0.981).
AS_BUILT = {"cx": 2.512, "cy": -0.043, "tilt_deg": 45.0, "width_m": 1.40, "u_top": 0.265, "side_draft_deg": 0.0, "post_top_r_m": 0.945}
FIXED_SIGMA = 1e-6


def run(photos=PHOTOS, fixed=None):
    from .. import annotations
    from ..fit import fit
    obs = []
    for pid in photos:
        ob = annotations.load(pid)
        if ob.camera_sigma is None:
            ob.camera_sigma = CAMERA_SIGMA[pid]
        obs.append(ob)
    model = Console()
    for k, v in (fixed or {}).items():
        model.params[k] = (v, FIXED_SIGMA)
    return fit(obs, model, [])


def main():
    args = sys.argv[1:]
    built = "--as-built" in args
    photos = tuple(a for a in args if not a.startswith("--")) or PHOTOS
    res = run(photos, AS_BUILT if built else None)
    for k, v in res.params.items():
        print(f"{k:16s} {v:9.4f} +- {res.sigma[k]:.4f}  data={res.constrained_by_data[k]}")
    for k, v in res.residual_px.items():
        print(f"residual {k}: {v:.2f} px rms")
    print(f"chi2 {res.chi2:.1f} dof {res.dof}")
    for pid, cam in res.cameras.items():
        print(pid, cam.to_row())
    if "crew1_1" in res.cameras:
        p = Console().points(res.params)
        print("post top", np.round(p["post_r_top"], 4), "base", np.round(p["post_r_base"], 4))
        print({k: round(v, 4) for k, v in post_sizes(res).items()})
    return res


if __name__ == "__main__":
    main()
