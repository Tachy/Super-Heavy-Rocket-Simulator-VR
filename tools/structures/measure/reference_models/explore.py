# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import sys
import numpy as np
import glb_measure as g

doc, binary = g._load(sys.argv[1])
world = g._world_matrices(doc)
nodes = doc["nodes"]
cache = {}


def node_pts(i):
    n = nodes[i]
    if "mesh" not in n:
        return None
    out = []
    for prim in doc["meshes"][n["mesh"]]["primitives"]:
        if "POSITION" in prim["attributes"]:
            p = g._positions(doc, binary, prim["attributes"]["POSITION"])
            out.append((world[i][:3, :3] @ p.T).T + world[i][:3, 3])
    return np.vstack(out) if out else None


def sub_box(i):
    lo = np.full(3, np.inf); hi = np.full(3, -np.inf); nm = 0
    st = [i]
    while st:
        k = st.pop()
        p = node_pts(k)
        if p is not None:
            lo = np.minimum(lo, p.min(0)); hi = np.maximum(hi, p.max(0)); nm += 1
        st.extend(nodes[k].get("children", []))
    return lo, hi, nm


scene = doc["scenes"][doc.get("scene", 0)]
maxd = int(sys.argv[2]) if len(sys.argv) > 2 else 3


def show(i, d):
    n = nodes[i]
    lo, hi, nm = sub_box(i)
    t = world[i][:3, 3]
    if nm:
        ext = hi - lo
        s = f"ext=({ext[0]:.1f},{ext[1]:.1f},{ext[2]:.1f}) ctr=({(hi[0]+lo[0])/2:.1f},{(hi[1]+lo[1])/2:.1f},{(hi[2]+lo[2])/2:.1f}) m={nm}"
    else:
        s = "-"
    print("  " * d + f"[{i}] {n.get('name')} o=({t[0]:.1f},{t[1]:.1f},{t[2]:.1f}) {s} ch={len(n.get('children', []))}")
    if d < maxd:
        for c in n.get("children", []):
            show(c, d + 1)


for r in scene["nodes"]:
    show(r, 0)
