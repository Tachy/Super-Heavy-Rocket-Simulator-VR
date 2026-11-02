# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""python -I d5c_rays2.py <m2 dir>: finer measurements of the KUBAHA model (bays, hood, top, nose interior, details)."""
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


def meridian(T, az_deg):
    """All intersection segments of the half plane az with triangles -> (x, r) polylines points."""
    a = np.radians(az_deg)
    e = np.array([0, np.cos(a), np.sin(a)])
    n = np.array([0, -np.sin(a), np.cos(a)])
    d = T @ n
    out = []
    for tri, dd in zip(T, d):
        pts = []
        for i, j in ((0, 1), (1, 2), (2, 0)):
            if (dd[i] <= 0) != (dd[j] <= 0):
                s = dd[i] / (dd[i] - dd[j])
                pts.append(tri[i] + s * (tri[j] - tri[i]))
        for q in pts:
            if q @ e > 0:
                out.append((q[0], q @ e))
    return np.array(out) if out else np.zeros((0, 2))


def fold(T, az):
    m = meridian(T, az)
    m = m[np.argsort(m[:, 0])]
    return m


def outer_grid(T, xs, azs):
    res = np.full((len(xs), len(azs)), np.nan)
    for j, az in enumerate(azs):
        m = meridian(T, az)
        if len(m) == 0:
            continue
        for i, x in enumerate(xs):
            k = np.abs(m[:, 0] - x) < 0.012
            if k.any():
                res[i, j] = m[k, 1].max()
    return res


cap = tris((9, 10, 11, 12))
xs = np.array([0.72, .8, .9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.55, 1.6, 1.65, 1.7, 1.75, 1.8, 1.85, 1.9, 1.95, 2.0, 2.1, 2.2, 2.4, 2.8, 3.2, 3.6, 3.8, 3.9])
azs = np.arange(0, 95, 5.0)
print("outer r, az 0..90 step 5")
print("   x  " + " ".join(f"{a:4.0f}" for a in azs))
gr = outer_grid(cap, xs, azs)
for x, row in zip(xs, gr):
    print(f"{x:5.2f} " + " ".join(f"{v:4.2f}" for v in row))
print("bay meridian az 67.5 all points (x,r) sorted, thinned")
for az in (67.5, 60, 75):
    m = fold(cap, az)
    print(az, " ".join(f"({x:.2f},{r:.2f})" for x, r in m[::max(1, len(m) // 60)]))
print("TOP capsule meridians above 3.5 (nodes 9,10,13)")
top = tris((9, 10, 13))
for az in (0, 45, 90, 135, 180):
    m = fold(top, az)
    m = m[m[:, 0] > 3.45]
    u = np.unique(np.round(m, 2), axis=0)
    print(az, " ".join(f"({x:.2f},{r:.2f})" for x, r in u))
print("NOSE meridians closed (15 white, 16 black)")
for name, nd in (("white", (15,)), ("black", (16,))):
    T = tris(nd)
    for az in (0, 90, 180):
        m = fold(T, az)
        u = np.unique(np.round(m, 2), axis=0)
        print(name, az, " ".join(f"({x:.2f},{r:.2f})" for x, r in u[::max(1, len(u) // 45)]))
# Clusters of dark features on the capsule wall
b = np.vstack([p["b"] for p in ps if p["node"] == 10])
az = np.degrees(np.arctan2(b[:, 2], b[:, 1])) % 360
r = np.hypot(b[:, 1], b[:, 2])
wall = r > 1.2
print("dark wall verts", wall.sum())
H, xe, ye = np.histogram2d(b[wall, 0], az[wall], bins=[np.arange(0.7, 4.3, 0.1), np.arange(0, 365, 5)])
for i in range(H.shape[0]):
    nz = np.nonzero(H[i])[0]
    if len(nz):
        print(f"x {xe[i]:.1f}: az", (ye[nz]).astype(int).tolist())
# Faces per connected component of the dark wall mesh: holes
for p in ps:
    if p["node"] != 10:
        continue
    t = p["tris"]
    parent = np.arange(len(p["b"]))
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for a, c, d in t:
        ra, rc, rd = find(a), find(c), find(d)
        parent[rc] = ra
        parent[find(d)] = ra
    roots = np.array([find(i) for i in range(len(p["b"]))])
    print("dark components")
    for rr in np.unique(roots):
        v = p["b"][roots == rr]
        if len(v) < 12:
            continue
        a = np.degrees(np.arctan2(v[:, 2].mean(), v[:, 1].mean())) % 360
        rv = np.hypot(v[:, 1], v[:, 2])
        lo = (np.degrees(np.arctan2(v[:, 2], v[:, 1])) - a + 180) % 360 - 180
        print(f"  n={len(v):4d} az={a:6.1f} ({lo.min():+.1f},{lo.max():+.1f}) X {v[:,0].min():.2f}..{v[:,0].max():.2f} r {rv.min():.2f}..{rv.max():.2f}")
# Windows components
for p in ps:
    if p["node"] != 12:
        continue
    b = p["b"]
    a = np.degrees(np.arctan2(b[:, 2], b[:, 1])) % 360
    # by connected clusters of azimuth
    for c0 in (0, 40, 90, 270, 320):
        m = np.abs(((a - c0 + 180) % 360) - 180) < 22
        if m.any():
            print("okno", c0, "az", np.round(((a[m] - c0 + 180) % 360 - 180).min() + c0, 1), np.round(((a[m] - c0 + 180) % 360 - 180).max() + c0, 1), "X", b[m, 0].min().round(2), b[m, 0].max().round(2), "r", np.hypot(b[m, 1], b[m, 2]).min().round(2), np.hypot(b[m, 1], b[m, 2]).max().round(2), "n", m.sum())
# Door (umbilical fairing), fin section at x=-2
door = tris((18, 19))
print("door Y", door[:, :, 1].min(), door[:, :, 1].max(), "Z", door[:, :, 2].min(), door[:, :, 2].max(), "X", door[:, :, 0].min(), door[:, :, 0].max())
dc = door.reshape(-1, 3)
print("door az", np.degrees(np.arctan2(dc[:, 2], dc[:, 1])).min(), np.degrees(np.arctan2(dc[:, 2], dc[:, 1])).max())
for xv in (0.1, 0.4, 0.7, 1.0, 1.15):
    m = np.abs(dc[:, 0] - xv) < 0.08
    if m.any():
        print("  x", xv, "r", np.hypot(dc[m, 1], dc[m, 2]).min().round(2), np.hypot(dc[m, 1], dc[m, 2]).max().round(2), "az", np.degrees(np.arctan2(dc[m, 2], dc[m, 1])).min().round(1), np.degrees(np.arctan2(dc[m, 2], dc[m, 1])).max().round(1))
fin = np.vstack([p["b"] for p in ps if p["node"] == 4])
mk = (np.abs(fin[:, 2]) < 0.2) & (fin[:, 1] > 1.9)
print("fin az0 thick Z", fin[mk, 2].min().round(3), fin[mk, 2].max().round(3))
m = fold(tris((4,)), 0.0)
u = np.unique(np.round(m[m[:, 1] > 1.87], 2), axis=0)
print("fin outline", " ".join(f"({x:.2f},{r:.2f})" for x, r in u[::max(1, len(u) // 40)]))
# Trunk top / shield seat
for nm, nd in (("metal", (11,)), ("trunk white", (4,)), ("trunk dark", (5,)), ("pasek", (7,))):
    m = fold(tris(nd), 22.0)
    u = np.unique(np.round(m, 2), axis=0)
    print(nm, "az22", " ".join(f"({x:.2f},{r:.2f})" for x, r in u[::max(1, len(u) // 40)]))
