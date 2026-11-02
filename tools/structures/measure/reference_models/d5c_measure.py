# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""python -I d5c_measure.py <m2 dir>: part extents and r(station, az) of the KUBAHA model in the body frame (x10, X up, az 0 = hatch)."""
import sys
import numpy as np
import d5c_gl as gl

g, ps = gl.parts(sys.argv[1])
APEX = None
for p in ps:
    v = p["verts"] * 10.0
    p["b"] = np.c_[v[:, 1], v[:, 2], v[:, 0]]  # body X, Y, Z
apex = min(p["b"][:, 0].min() for p in ps if p["node"] == 11)
print("apex (METAL min) ", apex)
for p in ps:
    p["b"][:, 0] -= apex
    b = p["b"]
    print(p["node"], p["material"], "X", b[:, 0].min().round(3), b[:, 0].max().round(3), "Y", b[:, 1].min().round(3), b[:, 1].max().round(3), "Z", b[:, 2].min().round(3), b[:, 2].max().round(3))
for n in (3, 14, 17):
    print("node", n, {k: g["nodes"][n].get(k) for k in ("name", "translation", "rotation", "scale", "matrix")})
a = g["animations"][0]
for c in a["channels"]:
    s = a["samplers"][c["sampler"]]
    from d5c_gl import acc, load
    _, bin_ = load(sys.argv[1])
    t = acc(g, bin_, s["input"]).ravel()
    o = acc(g, bin_, s["output"])
    print("chan", c["target"], "n", len(t), "t", t[0], t[-1])
    idx = np.linspace(0, len(t) - 1, 9).astype(int)
    for i in idx:
        print("   ", round(t[i], 3), np.round(o[i], 5))
# Capsule surface table over all parts of the capsule node; the nose in its rest pose reported apart.
caps = np.vstack([p["b"] for p in ps if p["node"] in (9, 10, 11, 12, 13)])
az = np.degrees(np.arctan2(caps[:, 2], caps[:, 1])) % 360.0
r = np.hypot(caps[:, 1], caps[:, 2])
stations = np.arange(0, 4.5, 0.25)
print("station  rmax over az bins of 30 deg (capsule node, all materials)")
print("        " + " ".join(f"{b:5d}" for b in range(0, 360, 30)))
for x in stations:
    m = np.abs(caps[:, 0] - x) < 0.08
    row = []
    for b in range(0, 360, 30):
        k = m & (np.abs(((az - b + 180) % 360) - 180) < 15)
        row.append(f"{r[k].max():5.2f}" if k.any() else "  -  ")
    print(f"{x:6.2f}  " + " ".join(row))
