# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""Shared flat-shaded rasteriser for the same-camera comparison (reference cache and our glTFs).
Camera: eye-free orthographic: centre, view direction (station frame), up hint, half width. Sources: list of (verts Nx3, tris Mx3, rgb).
"""
import numpy as np
from PIL import Image

def camera(fwd, up_hint):
    f = np.asarray(fwd, float); f /= np.linalg.norm(f)
    r = np.cross(f, up_hint); r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return f, r, u

def render(sources, centre, fwd, up_hint, half, W=1000, H=None, persp=None, bg=(12, 12, 16)):
    H = H or W
    f, r, u = camera(fwd, up_hint)
    img = np.empty((H, W, 3), np.uint8); img[:] = bg
    zb = np.full((H, W), np.inf)
    light = (-f * 0.45 + u * 0.6 + r * 0.35); light /= np.linalg.norm(light)
    aspect = H / W
    for V, T, col in sources:
        if len(T) == 0:
            continue
        v = V - centre
        x, y, d = v @ r, v @ u, v @ f
        if persp is not None:
            depth = d + persp
            good = depth > 0.5
            s = persp / np.maximum(depth, 0.5)
            x = x * s; y = y * s
        else:
            good = np.ones(len(v), bool)
        px = (x / half * 0.5 + 0.5) * W
        py = (0.5 - y / (half * aspect) * 0.5) * H
        vis = good[T].all(axis=1)
        vis &= (px[T].max(axis=1) >= 0) & (px[T].min(axis=1) < W) & (py[T].max(axis=1) >= 0) & (py[T].min(axis=1) < H)
        tri = T[vis]
        if not len(tri):
            continue
        n = np.cross(v[tri[:, 1]] - v[tri[:, 0]], v[tri[:, 2]] - v[tri[:, 0]])
        n /= (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)
        shade = np.abs(n @ light) * 0.78 + 0.22
        col = np.asarray(col, float)
        for t, sh in zip(tri, shade):
            xs, ys, ds = px[t], py[t], d[t]
            x0, x1 = int(max(np.floor(xs.min()), 0)), int(min(np.ceil(xs.max()) + 1, W))
            y0, y1 = int(max(np.floor(ys.min()), 0)), int(min(np.ceil(ys.max()) + 1, H))
            if x1 <= x0 or y1 <= y0:
                continue
            den = (ys[1] - ys[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (ys[0] - ys[2])
            if abs(den) < 1e-9:
                continue
            gx, gy = np.meshgrid(np.arange(x0, x1) + 0.5, np.arange(y0, y1) + 0.5)
            l0 = ((ys[1] - ys[2]) * (gx - xs[2]) + (xs[2] - xs[1]) * (gy - ys[2])) / den
            l1 = ((ys[2] - ys[0]) * (gx - xs[2]) + (xs[0] - xs[2]) * (gy - ys[2])) / den
            l2 = 1 - l0 - l1
            m = (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
            if not m.any():
                continue
            dd = l0 * ds[0] + l1 * ds[1] + l2 * ds[2]
            sub = zb[y0:y1, x0:x1]
            upd = m & (dd < sub)
            sub[upd] = dd[upd]
            img[y0:y1, x0:x1][upd] = (col * sh * 255).clip(0, 255).astype(np.uint8)
    return Image.fromarray(img)
