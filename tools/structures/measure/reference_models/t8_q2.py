# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import sys
import pickle, numpy as np
c = pickle.load(open(sys.argv[1], "rb")); names, V = c["names"], c["verts"]; idx = {n: i for i, n in enumerate(names)}
v = V[idx["P1_Radiator"]]
for y0, y1 in ((-20.48, -17.58), (-17.10, -12.28), (-11.79, -9.38)):
    w = v[(v[:, 1] >= y0) & (v[:, 1] <= y1)]
    print("wing", y0, y1, "z", np.percentile(w[:, 2], [0, 50, 100]).round(2))
    for a in np.arange(12.4, 36, 2.9):
        m = (w[:, 0] >= a) & (w[:, 0] < a + 0.25)
        if m.sum():
            print("   x %.2f n %d z %.2f..%.2f" % (a, m.sum(), w[m, 2].min(), w[m, 2].max()))
w = V[idx["MLM"]]
m = (w[:, 0] > 16.5) & (np.abs(w[:, 1]) > 2.2)
print("MLM x>16.5 far", w[m].min(0).round(2), w[m].max(0).round(2), m.sum())
for s in (-1, 1):
    k = m & (s * w[:, 1] > 0)
    if k.sum():
        print(" side", s, w[k].min(0).round(2), w[k].max(0).round(2), "x clusters", np.histogram(w[k, 0], bins=10)[0].tolist())
d = V[idx["MLM_Details_Misc"]]
m = (d[:, 0] > 13.7) & (d[:, 2] > 12.8)
print("MLM misc end cylinder", d[m].min(0).round(2), d[m].max(0).round(2), m.sum())
z = V[idx["Z1"]]
print("Z1 z<-6", z[z[:, 2] < -6].min(0).round(2), z[z[:, 2] < -6].max(0).round(2))
for n in names:
    if "Dish" in n or "SGANT" in n or "SASA" in n:
        i = idx[n]
        if i in V:
            print(n, V[i].min(0).round(2), V[i].max(0).round(2))
