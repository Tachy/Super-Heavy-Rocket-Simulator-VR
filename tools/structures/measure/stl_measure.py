"""Measure the 1:100 reference model (binary STL, mm; full scale metres = mm x 0.1). Data only: nothing is imported.

Usage (run with python -I):
  stl_measure.py bbox <stl>...                     bounding box (m) and triangle count per file
  stl_measure.py profile <stl> <axis> [bins]       radius profile along an axis (axis 0|1|2): r_max / r_min per bin
  stl_measure.py section <stl> <axis> <pos_m> [w]  points within +-w of the plane: bbox of the slab and its polar extent
  stl_measure.py view <out.png> <stl>... [--u 0 --v 1 --px 40 --max 200000]   shaded orthographic view (painter's
                                                   algorithm; u, v = model axes to the right / up; px per metre)
  stl_measure.py azimuth <stl> <axis> <lo_m> <hi_m> [bins]   angular coverage of the vertices in a station window
The numbers only guide our own redraw (the model is partly adapted for printing: thick walls, pegs, stands).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

MM_TO_M = 0.1  # 1:100 scale: 1 mm of the print = 0.1 m of the vehicle


def load_stl(path: str | Path) -> np.ndarray:
    """Return an (n, 3, 3) float64 array of triangle vertices in full-scale metres."""
    raw = np.fromfile(path, dtype=np.uint8)
    n = int(np.frombuffer(raw[80:84], dtype="<u4")[0])
    rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    tri = np.frombuffer(raw[84:84 + 50 * n].tobytes(), dtype=rec)
    return tri["v"].astype(np.float64) * MM_TO_M


def vertices(tris: np.ndarray) -> np.ndarray:
    return np.unique(tris.reshape(-1, 3).round(5), axis=0)


def bbox(tris: np.ndarray):
    v = tris.reshape(-1, 3)
    return v.min(axis=0), v.max(axis=0)


def radial(v: np.ndarray, axis: int, centre=None):
    """Coordinates across the axis, centred on the axis found from the bbox (or given), as (a, b, r, azimuth_deg)."""
    others = [i for i in range(3) if i != axis]
    if centre is None:
        lo, hi = v.min(axis=0), v.max(axis=0)
        centre = (lo + hi) / 2
    a, b = v[:, others[0]] - centre[others[0]], v[:, others[1]] - centre[others[1]]
    return a, b, np.hypot(a, b), np.degrees(np.arctan2(b, a))


def cmd_bbox(paths):
    for p in paths:
        t = load_stl(p)
        lo, hi = bbox(t)
        print(f"{Path(p).name:42s} tris {len(t):7d}  size {hi - lo}  lo {lo}  hi {hi}")


def cmd_profile(path, axis, bins):
    v = vertices(load_stl(path))
    _, _, r, _ = radial(v, axis)
    s = v[:, axis]
    edges = np.linspace(s.min(), s.max(), bins + 1)
    for i in range(bins):
        m = (s >= edges[i]) & (s <= edges[i + 1])
        if m.any():
            print(f"{edges[i]:8.2f}..{edges[i + 1]:8.2f}  r_max {r[m].max():6.3f}  r_min {r[m].min():6.3f}")


def cmd_section(path, axis, pos, w):
    v = vertices(load_stl(path))
    m = np.abs(v[:, axis] - pos) <= w
    if not m.any():
        print("empty")
        return
    lo, hi = v[m].min(axis=0), v[m].max(axis=0)
    print(f"{m.sum()} vertices; slab bbox lo {lo} hi {hi}")


def cmd_azimuth(path, axis, lo, hi, bins):
    v = vertices(load_stl(path))
    s = v[:, axis]
    m = (s >= lo) & (s <= hi)
    _, _, r, az = radial(v[m], axis)
    h, e = np.histogram(az[r > 0.5 * r.max()], bins=bins, range=(-180, 180))
    print(" ".join(f"{int(e[i]):4d}:{h[i]}" for i in range(bins)))


def cmd_view(out, paths, opts):
    from PIL import Image, ImageDraw
    u, v, px = int(opts.get("--u", 0)), int(opts.get("--v", 1)), float(opts.get("--px", 40))
    cap = int(opts.get("--max", 200000))
    w = 3 - u - v
    tris = np.concatenate([load_stl(p) for p in paths])
    if len(tris) > cap:
        tris = tris[np.random.default_rng(1).choice(len(tris), cap, replace=False)]
    lo, hi = tris.reshape(-1, 3).min(axis=0), tris.reshape(-1, 3).max(axis=0)
    pad = 10
    W, H = int((hi[u] - lo[u]) * px) + 2 * pad, int((hi[v] - lo[v]) * px) + 2 * pad
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    depth = tris[:, :, w].mean(axis=1)
    order = np.argsort(depth)  # far (small w) first; the viewer looks toward -w
    img = Image.new("RGB", (W, H), (20, 20, 30))
    d = ImageDraw.Draw(img)
    for i in order:
        shade = int(60 + 190 * abs(nrm[i, w]))
        pts = [((t[u] - lo[u]) * px + pad, H - ((t[v] - lo[v]) * px + pad)) for t in tris[i]]
        d.polygon(pts, fill=(shade, shade, shade))
    img.save(out)
    print(f"{out}: {W}x{H} px, u axis {u} from {lo[u]:.2f}, v axis {v} from {lo[v]:.2f}")


def main(argv):
    cmd = argv[1]
    if cmd == "bbox":
        cmd_bbox(argv[2:])
    elif cmd == "profile":
        cmd_profile(argv[2], int(argv[3]), int(argv[4]) if len(argv) > 4 else 40)
    elif cmd == "section":
        cmd_section(argv[2], int(argv[3]), float(argv[4]), float(argv[5]) if len(argv) > 5 else 0.05)
    elif cmd == "view":
        rest = argv[3:]
        paths = [a for a in rest if not a.startswith("--") and not rest[max(0, rest.index(a) - 1)].startswith("--")]
        opts = {rest[i]: rest[i + 1] for i, a in enumerate(rest) if a.startswith("--")}
        cmd_view(argv[2], paths, opts)
    elif cmd == "azimuth":
        cmd_azimuth(argv[2], int(argv[3]), float(argv[4]), float(argv[5]), int(argv[6]) if len(argv) > 6 else 36)
    else:
        raise SystemExit(__doc__)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
