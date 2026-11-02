# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""Node tree of ISS model D with vertex counts and station-frame boxes (config frame). python -I t8_tree.py glb out.txt"""
import os, sys
import numpy as np, pickle
import iss_measure as im
st = im.Station(sys.argv[1], keep_faces=True)
st.set_scale_from_span()
s = st.metres_per_unit
aft = float(s * st.vertices[st.index("Zvezda_SM")][:, 0].min())
z0 = float(s * st.node_origin[st.index("Node2")][2])
off = np.array([-aft, 0.0, -z0])
# cache for later scripts (station metres, config frame)
cache = {"names": st.names, "parents": st.parents, "children": st.children,
         "verts": {i: v * s + off for i, v in st.vertices.items()}, "faces": st.faces,
         "origin": {i: st.node_origin[i] * s + off for i in st.node_origin}}
pickle.dump(cache, open(sys.argv[2] + ".pkl", "wb"), protocol=4)
lines = []
def walk(i, d):
    p = st.points(i) * s + off if st.subtree(i) else np.zeros((0, 3))
    own = len(st.faces.get(i, []))
    if len(p):
        lo, hi = p.min(0), p.max(0)
        lines.append("  " * d + f"{st.names[i]} own_tris {own} sub_v {len(p)} [{lo[0]:.2f},{lo[1]:.2f},{lo[2]:.2f}]..[{hi[0]:.2f},{hi[1]:.2f},{hi[2]:.2f}]")
    if d < 3:
        for c in st.children[i]:
            walk(c, d + 1)
scene = st.doc["scenes"][st.doc.get("scene", 0)]["nodes"]
for r in scene:
    walk(r, 0)
open(sys.argv[2], "w", encoding="utf-8").write("\n".join(lines))
print(len(lines))
