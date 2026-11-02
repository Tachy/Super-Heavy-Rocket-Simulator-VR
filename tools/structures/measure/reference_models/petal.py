# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import os, sys
import numpy as np
import iss_measure as im

st = im.Station(sys.argv[1]); st.set_scale_from_span()
for nm, ax, up in (("IDA2", (1, 0, 0), (0, 0, -1)), ("IDA3", (0, 0, -1), (1, 0, 0))):
    d = st.ida_port(nm, ax)
    a = np.array(ax, float); up = np.array(up, float)
    yp = np.cross(up, a)  # port frame y = z x x
    ns = d["normal_axes"]
    c = np.zeros(3)
    c[ns[0]], c[ns[1]] = d["axis_centre_m"]
    root = st.index(nm)
    pet = [k for k in st.subtree(root) if "Petals" in st.names[k]][0]
    p = st.vertices[pet] * st.metres_per_unit
    t = p @ a
    for lo, hi in ((0.0, 0.06), (0.1, 0.2)):
        sel = p[(t - t.min() >= lo) & (t - t.min() < hi)] - c
        sel = sel - np.outer(sel @ a, a)
        ang = np.degrees(np.arctan2(sel @ up, sel @ yp)) % 360  # angle in the port frame from y_p towards up (z_p)
        r = np.linalg.norm(sel, axis=1)
        m = r > 0.3
        h, e = np.histogram(ang[m], bins=360, range=(0, 360))
        # cluster: angles whose count is above a threshold, grouped when closer than 25 deg
        idx = np.where(h > 0)[0]
        groups, cur = [], [idx[0]]
        for k in idx[1:]:
            if k - cur[-1] > 12:
                groups.append(cur); cur = [k]
            else:
                cur.append(k)
        groups.append(cur)
        if len(groups) > 1 and (groups[0][0] + 360 - groups[-1][-1]) <= 12:
            groups[0] = groups[-1] + [g + 360 for g in groups[0]]; groups.pop()
        print(nm, f"slice {lo}-{hi}", "groups (deg, port frame angle from y_p toward up):",
              [(round(np.mean(g) % 360, 1), round(g[-1] - g[0], 0)) for g in groups])
    print(nm, "port frame y_p in station axes", yp.tolist(), "up", up.tolist())
