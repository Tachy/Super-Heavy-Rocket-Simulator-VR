# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""Module profiles of model D as typed tables: slices along the module axis -> (station, radius, centre offsets), simplified.
Output is rounded table text (1 cm), reviewed and pasted into the detail file; no vertices leave the cache."""
import pickle, sys
import numpy as np
c = pickle.load(open(sys.argv[1], "rb"))
names, V = c["names"], c["verts"]
idx = {n: i for i, n in enumerate(names)}
# node, axis index, sign (profile runs along +axis), cap radius, slice length, paint
M = [("Zvezda_SM", 0, 2.25, 0.3, "ru"), ("Zarya_FGB", 0, 2.15, 0.3, "ru"), ("Node1", 0, 2.3, 0.3, "us"), ("USLab", 0, 2.3, 0.3, "us"),
     ("Node2", 0, 2.3, 0.3, "us"), ("PMA1", 0, 1.5, 0.15, "dark"), ("PMA2", 0, 1.5, 0.12, "dark"), ("PMA3", 2, 1.5, 0.12, "dark"),
     ("Node3", 1, 2.3, 0.3, "us"), ("Cupola", 2, 1.5, 0.12, "us"), ("Columbus", 1, 2.3, 0.3, "us"), ("JEM_PM", 1, 2.3, 0.3, "us"),
     ("JEM_PS", 2, 2.5, 0.2, "us"), ("Airlock", 1, 1.8, 0.2, "us"), ("Bishop_Airlock", 1, 1.4, 0.12, "us"), ("BEAM", 1, 1.8, 0.15, "beam"),
     ("MLM", 2, 2.2, 0.3, "ru"), ("MRM1", 2, 1.2, 0.2, "ru"), ("MRM2", 2, 1.4, 0.15, "ru"), ("Russian_RSNode_DockingModule", 2, 1.7, 0.12, "ru"),
     ("PMM", 1, 2.3, 0.2, "us")]
def dp(pts, tol):
    pts = np.asarray(pts)
    if len(pts) < 3:
        return list(range(len(pts)))
    keep = [0, len(pts) - 1]
    def rec(a, b):
        if b <= a + 1:
            return
        t = (pts[a + 1:b, 0] - pts[a, 0]) / max(pts[b, 0] - pts[a, 0], 1e-6)
        est = pts[a, 1:] + np.outer(t, pts[b, 1:] - pts[a, 1:])
        e = np.abs(pts[a + 1:b, 1:] - est).max(axis=1)
        k = int(np.argmax(e))
        if e[k] > tol:
            keep.append(a + 1 + k); rec(a, a + 1 + k); rec(a + 1 + k, b)
    rec(0, len(pts) - 1)
    return sorted(set(keep))
for nm, ax, cap, step, paint in M:
    v = V[idx[nm]]
    ns = [i for i in range(3) if i != ax]
    # a coarse centre first: median of all points within the cap of a robust centre
    centre = np.median(v[:, ns], axis=0)
    for _ in range(3):
        rr = np.hypot(*(v[:, ns] - centre).T)
        centre = 0.5 * (np.percentile(v[rr < cap, ns[0]], [1, 99]).sum() / 1 * np.array([1, 0]) + np.percentile(v[rr < cap, ns[1]], [1, 99]).sum() * np.array([0, 1]))
    rr = np.hypot(*(v[:, ns] - centre).T)
    body = v[rr < cap * 1.03]
    t = body[:, ax]
    rows = []
    for a in np.arange(t.min(), t.max() + step * 0.5, step):
        m = (t >= a) & (t < a + step)
        if m.sum() < 6:
            continue
        b = body[m]
        c0 = np.array([0.5 * (b[:, ns[0]].min() + b[:, ns[0]].max()), 0.5 * (b[:, ns[1]].min() + b[:, ns[1]].max())])
        # a slice may hold only one side: fall back to the module centre unless both sides are seen
        spread = (b[:, ns] - centre).max(0) - (b[:, ns] - centre).min(0)
        c1 = c0 if spread.min() > 1.0 else centre
        r = np.hypot(*(b[:, ns] - c1).T)
        rows.append((min(a + step / 2, t.max()), float(np.percentile(r, 92)), c1[0] - centre[0], c1[1] - centre[1]))
    rows = np.array(rows)
    keep = dp(rows, 0.05)
    k = rows[keep]
    outer = v[rr >= cap * 1.03]
    print(f'# {nm}: axis {"xyz"[ax]}, {t.min():.2f}..{t.max():.2f}; excess verts {len(outer)}' + (f' box {outer.min(0).round(2).tolist()} {outer.max(0).round(2).tolist()}' if len(outer) else ''))
    print('[[lathe]]\nid = "%s"\naxis = "%s"\npaint = "%s"\ncentre_m = [%.2f, %.2f]\nspan_m = [%.2f, %.2f]' % (nm, "xyz"[ax], paint, centre[0], centre[1], t.min(), t.max()))
    print("profile_m = [" + ", ".join("[%.2f, %.2f, %.2f, %.2f]" % tuple(r) for r in k) + "]")
