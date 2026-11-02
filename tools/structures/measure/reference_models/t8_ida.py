# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""IDA structure of model D from the cache: children, axial profile, relative clocking of IDA-2 and IDA-3, pin / hook candidates."""
import pickle, sys
import numpy as np
c = pickle.load(open(sys.argv[1], "rb"))
names, ch, V = c["names"], c["children"], c["verts"]
def sub(i):
    out, st = [], [i]
    while st:
        k = st.pop(); out.append(k); st.extend(ch[k])
    return out
idx = {n: i for i, n in enumerate(names)}
ports = {
    "IDA2": dict(p=np.array([51.87, 0.02, 0.72]), x=np.array([1., 0, 0]), z=np.array([0, 0, -1.])),
    "IDA3": dict(p=np.array([47.45, 0.02, -5.03]), x=np.array([0, 0, -1.]), z=np.array([1., 0, 0])),
}
loc = {}
for nm, f in ports.items():
    y = np.cross(f["z"], f["x"])
    print("==", nm, "y =", y)
    for k in sub(idx[nm]):
        if k not in V:
            print(" ", names[k], "no mesh"); continue
        v = V[k] - f["p"]
        a, b, d = v @ f["x"], v @ y, v @ f["z"]
        r = np.hypot(b, d)
        print(f"  {names[k]:34s} n {len(v):6d} ax {a.min():+.3f}..{a.max():+.3f} r {r.min():.3f}..{r.max():.3f}")
    pts = []
    for k in sub(idx[nm]):
        if k in V:
            v = V[k] - f["p"]
            pts.append(np.c_[v @ f["x"], v @ y, v @ f["z"]])
    loc[nm] = np.vstack(pts)
    a, b, d = loc[nm].T
    r = np.hypot(b, d)
    print(" axial profile (ax slice: r p5 p50 max, n)")
    for t in np.arange(-1.2, 0.35, 0.05):
        m = (a >= t) & (a < t + 0.05)
        if m.sum() > 5:
            print(f"  {t:+.2f} {np.percentile(r[m],5):.3f} {np.median(r[m]):.3f} {r[m].max():.3f} {m.sum()}")
# relative clocking: histogram correlation of the azimuth pattern
def hist(p):
    az = np.degrees(np.arctan2(p[:, 2], p[:, 1])) % 360
    return np.histogram(az, bins=360, range=(0, 360))[0].astype(float)
h2, h3 = hist(loc["IDA2"]), hist(loc["IDA3"])
scores = [(float(np.dot(h2, np.roll(h3, s))), s) for s in range(360)]
scores.sort(reverse=True)
print("rotation of IDA3 pattern onto IDA2 (deg, score):", [(s, round(v / scores[0][0], 3)) for v, s in scores[:6]])
# self similarity: which rotations map the part on itself
self_s = sorted([(float(np.dot(h2, np.roll(h2, s))), s) for s in range(1, 360)], reverse=True)
print("self-similarity IDA2:", [(s, round(v / np.dot(h2, h2), 3)) for v, s in self_s[:6]])
# azimuth histogram near the HCS plane, per 10 deg, in three radius bands
for nm in loc:
    a, b, d = loc[nm].T
    r = np.hypot(b, d); az = np.degrees(np.arctan2(d, b)) % 360
    for lo, hi in ((0.58, 0.66), (0.66, 0.72), (0.72, 0.80)):
        m = (r >= lo) & (r < hi) & (a > -0.25)
        print(nm, f"r {lo}-{hi}", np.histogram(az[m], bins=36, range=(0, 360))[0].tolist())
