# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import os, sys
import numpy as np
import iss_measure as im
import glb_measure as gm

st = im.Station(sys.argv[1])
s = 0.0254
doc = st.doc
world = gm._world_matrices(doc)
rot = im._station_rotation()
inv = np.linalg.inv(st.root_matrix)
for nm in ("IDA2", "IDA3", "PMA2", "PMA3", "Node2", "PMA1"):
    i = st.index(nm)
    m = (inv @ world[i])[:3, :3]
    m = m / np.linalg.norm(m[:, 0])
    R = rot @ m @ rot.T  # local axes of the node expressed in the station frame
    print(nm, "origin", (st.node_origin[i] * s).round(3), "\n local x,y,z axes in station frame (columns):\n", R.round(3))

for tag, a, ns in (("IDA2", np.array([1, 0, 0.]), (1, 2)), ("IDA3", np.array([0, 0, -1.]), (0, 1))):
    root = st.index(tag)
    for k in st.subtree(root):
        if k in st.vertices and ("Petals" in st.names[k] or "Targets" in st.names[k] or st.names[k] == tag):
            p = st.vertices[k] * s
            t = p @ a
            sel = p[t > t.max() - 0.03] if "Petals" in st.names[k] else p
            print(tag, st.names[k], "tip points", len(sel))
    # fit circle to ring points of IDA mesh at the largest radius slices
    pm = st.vertices[root] * s
    t = pm @ a
    sl = pm[(t > t.min() + 0.0) & (t < t.min() + 0.06)]
    A = np.c_[2 * sl[:, ns[0]], 2 * sl[:, ns[1]], np.ones(len(sl))]
    b = sl[:, ns[0]] ** 2 + sl[:, ns[1]] ** 2
    sol = np.linalg.lstsq(A, b, rcond=None)[0]
    r = np.sqrt(sol[2] + sol[0] ** 2 + sol[1] ** 2)
    print(tag, "base ring circle fit centre", sol[:2].round(3), "r", r.round(3), "slice axial", (t.min()).round(3))
    pet = [k for k in st.subtree(root) if "Petals" in st.names[k]][0]
    pp = st.vertices[pet] * s
    c = sol[:2]
    tt = pp @ a
    for lo in np.arange(tt.min(), tt.max(), 0.05):
        m = (tt >= lo) & (tt < lo + 0.05)
        ang = np.degrees(np.arctan2(pp[m][:, ns[1]] - c[1], pp[m][:, ns[0]] - c[0]))
        rr = np.hypot(pp[m][:, ns[0]] - c[0], pp[m][:, ns[1]] - c[1])
        h, _ = np.histogram(ang, bins=12, range=(-180, 180))
        print(f"  petals ax {lo:.2f}: n={m.sum()} rmin {rr.min():.3f} rmax {rr.max():.3f} hist30deg {h.tolist()}")
