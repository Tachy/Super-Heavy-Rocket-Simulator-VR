# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""python -I d5c_rays.py <m2 dir> <out.npz>: ray-cast the closed KUBAHA model: outermost radius r(station, az); printed tables, and seam/nose/trunk data.
Measurement only; output goes to the given path."""
import sys
import numpy as np
import d5c_gl as gl

g, ps = gl.parts(sys.argv[1])
APEX = 0.03309038
for p in ps:
    v = p["verts"] * 10.0
    p["b"] = np.c_[v[:, 1] - APEX, v[:, 2], v[:, 0]]


def tris(nodes):
    return np.vstack([p["b"][p["tris"]] for p in ps if p["node"] in nodes])


def outer(T, xs, azs):
    """largest r where the half-plane ray (x, az) hits; NaN none. Vectorised per station."""
    out = np.full((len(xs), len(azs)), np.nan)
    for i, x in enumerate(xs):
        k = (T[:, :, 0].min(1) <= x) & (T[:, :, 0].max(1) >= x)
        t = T[k]
        if len(t) == 0:
            continue
        # intersect each triangle with the plane X=x -> segments in (y, z)
        segs = []
        for tri in t:
            d = tri[:, 0] - x
            pts = []
            for a, b in ((0, 1), (1, 2), (2, 0)):
                if (d[a] <= 0) != (d[b] <= 0):
                    s = d[a] / (d[a] - d[b])
                    pts.append(tri[a, 1:] + s * (tri[b, 1:] - tri[a, 1:]))
            if len(pts) == 2:
                segs.append(pts)
        if not segs:
            continue
        s = np.array(segs)  # (n, 2, 2)
        for j, az in enumerate(azs):
            e = np.array([np.cos(az), np.sin(az)])
            n = np.array([-e[1], e[0]])
            d0, d1 = s[:, 0] @ n, s[:, 1] @ n
            m = (d0 <= 0) != (d1 <= 0)
            if m.any():
                f = d0[m] / (d0[m] - d1[m])
                hit = s[m, 0] + (s[m, 1] - s[m, 0]) * f[:, None]
                r = hit @ e
                r = r[r > 0]
                if len(r):
                    out[i, j] = r.max()
    return out


XS = np.array([float(v) for v in "0.05 0.1 0.2 0.3 0.35 0.4 0.45 0.5 0.6 0.68 0.72 0.8 0.9 1.0 1.2 1.4 1.6 1.7 1.8 1.9 2.0 2.2 2.4 2.6 2.8 3.0 3.2 3.4 3.6 3.8 4.0 4.2 4.3 4.4 4.5 4.6 4.7 4.8 4.84".split()])
AZ_DEG = np.arange(0, 361, 5.0)
AZ = np.radians(AZ_DEG)
cap = outer(tris((9, 10, 11, 12, 13, 15, 16)), XS, AZ)   # capsule + closed nose (trunk-door excluded)
np.savez(sys.argv[2], xs=XS, az=AZ_DEG, cap=cap)
np.set_printoptions(linewidth=250, precision=2, suppress=True)
print("r(x, az) every 15 deg 0..180; then asym check |r(az)-r(360-az)| max")
print("   x  " + " ".join(f"{a:5.0f}" for a in AZ_DEG[:37:3]))
for i, x in enumerate(XS):
    print(f"{x:5.2f} " + " ".join(f"{v:5.2f}" for v in cap[i, :37:3]))
print("mirror asym max", np.nanmax(np.abs(cap - cap[:, ::-1])))
# seam: nose node's lowest station by az
nose = np.vstack([p["b"] for p in ps if p["node"] == 15])
naz = np.degrees(np.arctan2(nose[:, 2], nose[:, 1])) % 360
r = np.hypot(nose[:, 1], nose[:, 2])
print("nose white: az, min X, r at min X, max r")
for a in range(0, 360, 15):
    m = np.abs(((naz - a + 180) % 360) - 180) < 4
    if m.any():
        i = np.argmin(nose[m, 0])
        print(a, round(nose[m, 0].min(), 3), round(r[m][i], 3), round(r[m].max(), 3))
# hinge: node 14 local origin in body frame
print("hinge node14 origin body:", 0.37232518 * 10 - APEX, -0.11080825 * 10, 0.0, " axis model X = body Z")
print("rotation -0.866,0.5 => q angle deg", 2 * np.degrees(np.arctan2(0.86603, 0.5)))
# seam plane fit of nose lip
lip = []
for a in range(0, 360, 10):
    m = np.abs(((naz - a + 180) % 360) - 180) < 4
    if m.any():
        i = np.argmin(nose[m, 0]); lip.append(nose[m][i])
lip = np.array(lip)
A = np.c_[lip[:, 1], lip[:, 2], np.ones(len(lip))]
c, res, *_ = np.linalg.lstsq(A, lip[:, 0], rcond=None)
print("seam plane X = a Y + b Z + c:", c, "tilt deg", np.degrees(np.arctan(np.hypot(c[0], c[1]))), "resid max", np.abs(A @ c - lip[:, 0]).max())
# the nose inside
print("capsule top opening rim etc, r at x (node 9 only inside rim)")
for nm, nodes in (("capsule black", (10,)), ("metal", (11,)), ("window", (12,)), ("port", (13,)), ("nose black", (16,)), ("door white", (18,)), ("door dark", (19,))):
    b = np.vstack([p["b"] for p in ps if p["node"] in nodes])
    print(nm, "X", b[:, 0].min().round(3), b[:, 0].max().round(3), "r", np.hypot(b[:, 1], b[:, 2]).min().round(3), np.hypot(b[:, 1], b[:, 2]).max().round(3))
# windows clusters
b = np.vstack([p["b"] for p in ps if p["node"] == 12])
az = np.degrees(np.arctan2(b[:, 2], b[:, 1])) % 360
for c0 in (40, 90, 270, 320):
    m = np.abs(((az - c0 + 180) % 360) - 180) < 25
    if m.any():
        print("window near", c0, "az", az[m].min().round(1), az[m].max().round(1), "X", b[m, 0].min().round(3), b[m, 0].max().round(3), "r", np.hypot(b[m, 1], b[m, 2]).mean().round(3))
# trunk: r(x, az)
tr = outer(tris((4, 5, 6, 7)), np.array([-3.2, -3.0, -2.6, -2.4, -2.0, -1.6, -1.2, -0.8, -0.4, -0.2, 0.0, 0.1, 0.2, 0.3]), np.radians(np.arange(0, 360, 15.0)))
print("trunk r(x, az 0..345 step 15)")
for row, x in zip(tr, [-3.2, -3.0, -2.6, -2.4, -2.0, -1.6, -1.2, -0.8, -0.4, -0.2, 0.0, 0.1, 0.2, 0.3]):
    print(f"{x:5.2f} " + " ".join(f"{v:4.2f}" for v in row))
for p in ps:
    if p["node"] in (6, 7):
        b = p["b"]; a = np.degrees(np.arctan2(b[:, 2], b[:, 1])) % 360
        print(p["material"], "X", b[:, 0].min().round(3), b[:, 0].max().round(3), "az hist(30deg)", np.histogram(a, bins=12, range=(0, 360))[0])
