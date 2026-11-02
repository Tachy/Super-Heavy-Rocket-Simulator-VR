# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""High-resolution looks at the model's IDAs (flat colour per node; screen up = station direction named)."""
import pickle, sys, zlib, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import t8_ras as R
c = pickle.load(open(sys.argv[1], "rb")); out = sys.argv[2]
names, ch, V, F = c["names"], c["children"], c["verts"], c["faces"]
def sub(i):
    o, st = [], [i]
    while st:
        k = st.pop(); o.append(k); st.extend(ch[k])
    return o
idx = {n: i for i, n in enumerate(names)}
def src(nodes):
    res = []
    for k in nodes:
        if k in V and k in F:
            h = zlib.crc32(names[k].encode()) / 2**32
            res.append((V[k], F[k], (0.5 + 0.45 * h, 0.75 - 0.3 * ((h * 5) % 1), 0.95 - 0.5 * h)))
    return res
for nm, p, x, up in (("IDA2", (51.87, 0.02, 0.72), (1, 0, 0), (0, 0, -1)), ("IDA3", (47.45, 0.02, -5.03), (0, 0, -1), (1, 0, 0))):
    s = src(sub(idx[nm]))
    p, x = np.array(p), np.array(x, float)
    R.render(s, p, -x, up, 2.6, 1100).save(f"{out}/{nm}_head.png")
    side = np.cross(x, up)
    R.render(s, p - x * 0.4, -side, up, 2.6, 900).save(f"{out}/{nm}_side.png")
    R.render(s, p - x * 0.4, -x * 0.8 - side * 0.6 + np.array(up) * -0.4, up, 2.6, 1100).save(f"{out}/{nm}_q.png")
print("done")
