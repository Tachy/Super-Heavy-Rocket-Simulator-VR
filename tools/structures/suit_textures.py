"""Suit surface detail of the crew figure (Task 8b), baked on the suit hull's UV atlas (figure_body.hull layout), no photo pixels:

suit_albedo.png (sRGB RGBA8, crew_figure.toml [suit_look] texel_m): the white suit with the dark grey side panels (hip to axilla, up the
front of the shoulder to the collar), the stripe down the back of the upper arm, the grey forearm panel, the chest plate's grey chevron
band, seam piping (princess lines, hip seam), the front closure, thigh pockets, ribbed knee bellows (shading), grey joint inserts (inner
elbow, back of the knee), the flag patch on the upper left arm. References: eva_suit_newatlas, user/anzug/spacex.jpg (crew_figure.toml).

suit_wrinkle.png (linear RGBA8, [suit_look] wrinkle_texel_m): the compression folds of every joint zone at FULL bend (the shader scales
them by the joint's wrinkle weight): R, G = the fold height's slopes dh/du, dh/dv along the atlas axes (m per m of atlas) stored as
0.5 + slope / figure_layout.WRINKLE_SLOPE_RANGE; B = the fold cavity (1 in a fold's valley: the shader darkens there); A = 1.
Folds are ridges across the limb (constant v), arcing around it, strongest on the compressed side of the bend (the zone's u_in), with
a cosine envelope over the zone's reach; pitch and depth from [joints] fold_pitch_m / fold_depth_m. Deterministic (no random state).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "blender"))
import figure_body  # noqa: E402
import figure_layout  # noqa: E402
from hull_textures import png_bytes  # noqa: E402

FLAG_STRIPES = 13  # [S] the U.S. flag: 13 stripes, the canton over the top 7
FLAG_RED = (0.45, 0.04, 0.05)  # [E] linear albedo of the flag's red (spacex.jpg patch)
FLAG_BLUE = (0.03, 0.05, 0.18)  # [E] linear albedo of the canton
FOLD_BOW_M = 0.012  # [E] the folds arc this far along the limb around it (cloth folds are not straight rings)
FOLD_U_SIGMA = 0.16  # [E] the folds' spread around the limb from the compressed side (share of the circumference)
FOLD_FLOOR = 0.30  # [E] fold amplitude on the stretched side relative to the compressed side
BELLOWS_SHADE = 0.80  # [E] albedo factor in a knee-bellows valley (eva_suit_newatlas: ribs read as soft grey lines)
POCKET_FILL = 0.93  # [E] the pocket flap reads a touch darker than the suit


def _wrap(du):
    return (du + 0.5) % 1.0 - 0.5


def _band(x, centre, half, aa):
    """1 inside |x - centre| < half, smooth over aa."""
    return np.clip((half - np.abs(x - centre)) / aa + 0.5, 0.0, 1.0)


def _box(u_m, v, u0, u1, v0, v1, aa):
    return (np.clip((u_m - u0) / aa + 0.5, 0, 1) * np.clip((u1 - u_m) / aa + 0.5, 0, 1)
            * np.clip((v - v0) / aa + 0.5, 0, 1) * np.clip((v1 - v) / aa + 0.5, 0, 1))


def _paint(img, mask, colour):
    c = np.asarray(colour, dtype=float)
    img[...] = img * (1.0 - mask[..., None]) + c * mask[..., None]


def albedo(cfg) -> np.ndarray:
    """(H, W, 3) linear albedo of the atlas."""
    look = cfg["suit_look"]
    _, layout = figure_body.hull(cfg, ease=True)
    aw, ah = look["atlas_m"]
    texel = look["texel_m"]
    w, h = int(round(aw / texel)), int(round(ah / texel))
    xs = (np.arange(w) + 0.5) * texel
    ys = (np.arange(h) + 0.5) * texel
    base = np.array(cfg["material"]["suit"]["albedo"], dtype=float)
    img = np.ones((h, w, 3)) * base
    stripe, seam, insert = look["stripe_albedo"], look["seam_albedo"], look["insert_albedo"]
    aa = texel
    for name, part in layout["parts"].items():
        ox, oy = part["origin"]
        uw = part["u_width"]
        length = part.get("v_length", part["levels"][-1] - part["v0_s"] if "levels" in part else 0.0)
        x0, x1 = int(ox / texel), int(math.ceil((ox + uw) / texel))
        y0, y1 = int(oy / texel), min(h, int(math.ceil((oy + length) / texel)) + 2)
        u = (xs[x0:x1] - ox) / uw
        v = ys[y0:y1] - oy
        U, V = np.meshgrid(u, v)
        Um = U * uw  # metres around (atlas)
        sub = img[y0:y1, x0:x1]
        if name == "torso":
            s = V + part["v0_s"]
            for side in (1.0, -1.0):
                u_side = 0.5 + side * 0.25  # left 0.75, right 0.25
                u_c = u_side - side * 0.10  # the panel runs along the front edge of the side
                # above the shoulder top the panel turns toward the front of the neck
                u_c = np.where(s > 0.545, u_c - side * (s - 0.545) * 1.2, u_c)
                half = look["side_stripe_width_m"] / 2.0 / uw
                m = _band(U, u_c, half, aa / uw) * np.clip((s + 0.03) / 0.01, 0, 1) * np.clip((0.66 - s) / 0.01, 0, 1)
                _paint(sub, m, stripe)
            # chevron band: a V below the collar, tip at the sternum
            tip, rise = 0.470, 0.75
            sv = tip + rise * np.abs(_wrap(U - 0.5)) * uw
            m = _band(s, sv, look["chevron_width_m"] / 2.0, aa) * (np.abs(_wrap(U - 0.5)) * uw < 0.12)
            _paint(sub, m, insert)
            # princess seams down the chest, the hip seam all around, the front closure
            for side in (1.0, -1.0):
                m = _band(U, 0.5 + side * 0.085, look["seam_width_m"] / 2.0 / uw, aa / uw) * (s < sv - 0.01) * (s > 0.10)
                _paint(sub, m, seam)
            _paint(sub, _band(s, 0.10, look["seam_width_m"] / 2.0, aa), seam)
            m = _band(U, 0.5, look["zip_width_m"] / 2.0 / uw, aa / uw) * (s < tip) * (s > -0.06)
            _paint(sub, m, np.array(seam) * 0.7)
        elif name.startswith("arm"):
            port, elbow, wrist = part["v_port_end"], part["v_elbow"], part["v_wrist"]
            u_bl = part["u_out"] + 0.5 * _wrap(part["u_back"] - part["u_out"])
            m = _band(_wrap(U - u_bl), 0.0, look["arm_stripe_width_m"] / 2.0 / uw, aa / uw) * (V > 0.5 * port) * (V < elbow - 0.03)
            _paint(sub, m, stripe)
            m = _band(_wrap(U - part["u_out"]), 0.0, 0.05 / 2.0 / uw, aa / uw) * (V > elbow + 0.04) * (V < wrist - 0.02)
            _paint(sub, m, stripe)
            m = _band(_wrap(U - part["u_elbow_inner"]), 0.0, 0.17, aa / uw) * _band(V, elbow, 0.04, aa)
            _paint(sub, m, insert)
            _paint(sub, _band(V, wrist - 0.008, 0.008, aa), stripe)
            if part["side"] == "l":
                fw, fh = look["flag_m"]
                vc = port + 0.07
                du = _wrap(U - part["u_out"]) * uw
                inside = _box(du, V, -fw / 2, fw / 2, vc - fh / 2, vc + fh / 2, aa)
                fv = (V - (vc - fh / 2)) / fh  # 0 at the top (toward the shoulder)
                stripes = (np.floor(np.clip(fv, 0, 0.9999) * FLAG_STRIPES) % 2 == 0)
                flag = np.where(stripes[..., None], np.array(FLAG_RED), np.array([0.80, 0.80, 0.80]))
                front = np.sign(_wrap(part["u_front"] - part["u_out"]))
                canton = (fv < 7.0 / FLAG_STRIPES) * ((du * front) > 0.0)
                flag = np.where(canton[..., None], np.array(FLAG_BLUE), flag)
                sub[...] = sub * (1 - inside[..., None]) + flag * inside[..., None]
        else:
            port, knee, boot = part["v_port_end"], part["v_knee"], part["v_boot"]
            m = _band(_wrap(U - part["u_out"]), 0.0, look["leg_stripe_width_m"] / 2.0 / uw, aa / uw) * (V < knee - 0.07)
            _paint(sub, m, stripe)
            pw, ph = look["pocket_m"]
            u_p = part["u_out"] + 0.40 * _wrap(part["u_front"] - part["u_out"])
            du = _wrap(U - u_p) * uw
            vc = port + 0.14
            box = _box(du, V, -pw / 2, pw / 2, vc - ph / 2, vc + ph / 2, aa)
            inner = _box(du, V, -pw / 2 + look["seam_width_m"], pw / 2 - look["seam_width_m"], vc - ph / 2 + look["seam_width_m"],
                         vc + ph / 2 - look["seam_width_m"], aa)
            sub[...] = sub * (1 - box[..., None] * (1 - POCKET_FILL))
            _paint(sub, np.clip(box - inner, 0.0, 1.0), seam)
            ribs = 0.5 - 0.5 * np.cos(2 * math.pi * (V - knee) / look["bellows_pitch_m"])
            half_span = look["bellows_pitch_m"] * look["bellows_count"] / 2.0
            front = _band(_wrap(U - part["u_front"]), 0.0, 0.20, 0.03) * _band(V, knee, half_span, aa)
            sub[...] = sub * (1 - front * ribs * (1 - BELLOWS_SHADE))[..., None]
            m = _band(_wrap(U - part["u_back"]), 0.0, 0.14, aa / uw) * _band(V, knee, 0.05, aa)
            _paint(sub, m, insert)
    return np.clip(img, 0.0, 1.0)


def wrinkle(cfg) -> np.ndarray:
    """(H, W, 4) floats: dh/du, dh/dv (m/m), fold cavity, 1."""
    look, joints = cfg["suit_look"], cfg["joints"]
    _, layout = figure_body.hull(cfg, ease=True)
    aw, ah = look["atlas_m"]
    texel = look["wrinkle_texel_m"]
    w, h = int(round(aw / texel)), int(round(ah / texel))
    slope_u = np.zeros((h, w))
    slope_v = np.zeros((h, w))
    cav = np.zeros((h, w))
    pitch, depth = joints["fold_pitch_m"], joints["fold_depth_m"]
    for k, zone in enumerate(layout["zones"]):
        part = layout["parts"][zone["part"]]
        ox, oy = part["origin"]
        uw = part["u_width"]
        x0, x1 = int(ox / texel), int(math.ceil((ox + uw) / texel))
        y0 = max(0, int((oy + zone["v"] - zone["reach"]) / texel))
        y1 = min(h, int(math.ceil((oy + zone["v"] + zone["reach"]) / texel)) + 1)
        # one texel of margin on every side: the height is periodic in u, so the slopes at the part's u seam are the surface's own
        xs = (np.arange(x0 - 1, x1 + 1) + 0.5) * texel
        ys = (np.arange(y0 - 1, y1 + 1) + 0.5) * texel
        U, V = np.meshgrid((xs - ox) / uw, ys - oy)
        dv = V - zone["v"]
        env_v = np.where(np.abs(dv) < zone["reach"], np.cos(0.5 * math.pi * dv / zone["reach"]) ** 2, 0.0)
        du = _wrap(U - zone["u_in"])
        env_u = FOLD_FLOOR + (1.0 - FOLD_FLOOR) * np.exp(-(du / FOLD_U_SIGMA) ** 2)
        bent = dv + FOLD_BOW_M * np.cos(2.0 * math.pi * du)
        phase = bent / pitch + 0.22 * np.sin(2.0 * math.pi * 3.0 * U + 1.7 * k) + 0.12 * np.sin(2.0 * math.pi * 7.0 * U + 0.9 * k)
        ridge = 0.5 - 0.5 * np.cos(2.0 * math.pi * phase)
        env = env_v * env_u
        height = depth * env * ridge
        g_v, g_u = np.gradient(height, texel, texel)
        slope_u[y0:y1, x0:x1] += g_u[1:-1, 1:-1]
        slope_v[y0:y1, x0:x1] += g_v[1:-1, 1:-1]
        cav[y0:y1, x0:x1] = np.maximum(cav[y0:y1, x0:x1], (env * (1.0 - ridge))[1:-1, 1:-1])
    out = np.zeros((h, w, 4))
    out[..., 0], out[..., 1], out[..., 2], out[..., 3] = slope_u, slope_v, cav, 1.0
    return out


def encode_srgb(lin: np.ndarray) -> np.ndarray:
    c = np.clip(lin, 0.0, 1.0)
    s = np.where(c <= 0.0031308, 12.92 * c, 1.055 * np.power(c, 1.0 / 2.4) - 0.055)
    rgba = np.ones(lin.shape[:2] + (4,))
    rgba[..., :3] = s
    return np.round(rgba * 255.0).astype(np.uint8)


def encode_wrinkle(wr: np.ndarray) -> np.ndarray:
    r = figure_layout.WRINKLE_SLOPE_RANGE
    out = np.empty_like(wr)
    out[..., 0] = 0.5 + wr[..., 0] / r
    out[..., 1] = 0.5 + wr[..., 1] / r
    out[..., 2] = wr[..., 2]
    out[..., 3] = 1.0
    return np.round(np.clip(out, 0.0, 1.0) * 255.0).astype(np.uint8)


def write(out_dir, cfg) -> dict:
    """Bakes both maps into out_dir; returns {"albedo": file, "wrinkle": file}."""
    out = Path(out_dir)
    (out / figure_layout.SUIT_ALBEDO_FILE).write_bytes(png_bytes(encode_srgb(albedo(cfg))))
    (out / figure_layout.SUIT_WRINKLE_FILE).write_bytes(png_bytes(encode_wrinkle(wrinkle(cfg))))
    return {"albedo": figure_layout.SUIT_ALBEDO_FILE, "wrinkle": figure_layout.SUIT_WRINKLE_FILE}
