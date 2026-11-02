# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import os, sys, time
import numpy as np
import iss_measure as im

t0 = time.time()
st = im.Station(sys.argv[1])
print("loaded", time.time() - t0, "s", len(st.vertices), "meshes", sum(len(v) for v in st.vertices.values()), "verts", file=sys.stderr)
print("truss units", st.truss_extent_units(), file=sys.stderr)
s = st.set_scale_from_truss()
print("m/unit", s, file=sys.stderr)
rows = []
for i, v in st.vertices.items():
    lo, hi = v.min(0) * s, v.max(0) * s
    rows.append((st.path(i), lo, hi, len(v)))
for name, lo, hi, n in rows:
    e = hi - lo
    c = (lo + hi) / 2
    print(f"{name} | ext {e[0]:.2f} {e[1]:.2f} {e[2]:.2f} | ctr {c[0]:.2f} {c[1]:.2f} {c[2]:.2f} | n {n}")
