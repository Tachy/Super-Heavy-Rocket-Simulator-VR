# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import os, sys
import numpy as np
import iss_measure as im

st = im.Station(sys.argv[1])
s = 0.0254
for tag, axis_vec in (("IDA2", [1, 0, 0]), ("IDA3", [0, 0, -1])):
    root = st.index(tag)
    print("=====", tag, "axis", axis_vec)
    a = np.array(axis_vec, float)
    for k in st.subtree(root):
        if k in st.vertices:
            p = st.vertices[k] * s
            c = (p.min(0) + p.max(0)) / 2
            t = p @ a
            print(f"{st.names[k]:32s} n={len(p):6d} min={p.min(0).round(3)} max={p.max(0).round(3)} axial {t.min():.3f}..{t.max():.3f}")
    # axis centre from the IDA mesh (ring) in the plane normal to the axis
    pm = st.vertices[root] * s
    ns = [i for i in range(3) if abs(a[i]) < 0.5]
    mid = (pm.min(0) + pm.max(0)) / 2
    print("IDA mesh normal-plane centre", mid[ns].round(3), "full-extent", (pm.max(0) - pm.min(0)).round(3))
    # radial profile of the IDA mesh about its centre
    t = pm @ a
    r = np.linalg.norm(pm[:, ns] - mid[ns], axis=1)
    for lo in np.arange(np.floor(t.min() * 20) / 20, t.max(), 0.05):
        m = (t >= lo) & (t < lo + 0.05)
        if m.sum():
            print(f"   ax {lo:.2f}: n={m.sum():4d} rmax={r[m].max():.3f}")
    # petals: angle clusters of the petal tips
    for k in st.subtree(root):
        if "Petals" in st.names[k] and k in st.vertices:
            pp = st.vertices[k] * s
            tt = pp @ a
            cen = mid[ns]
            ang = np.degrees(np.arctan2(pp[:, ns[1]] - cen[1], pp[:, ns[0]] - cen[0]))
            print("petal axial", tt.min().round(3), tt.max().round(3), "radius max", np.linalg.norm(pp[:, ns] - cen, axis=1).max().round(3))
            h, e = np.histogram(ang, bins=36, range=(-180, 180))
            print("petal angle hist (10deg bins, in-plane angle between", [int(x) for x in ns], "axes):", h.tolist())
