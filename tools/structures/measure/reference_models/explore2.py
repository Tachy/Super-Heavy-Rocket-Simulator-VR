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
root = world[587]
k = -root[0, 0]  # root scale magnitude (negative uniform)
# local (root) coordinates: world / root; map to station frame: X=x', Y=-z', Z=y'
rows = []


def path(i, parents):
    p = []
    while i is not None:
        p.append(nodes[i].get("name", str(i)))
        i = parents.get(i)
    return "/".join(reversed(p))


parents = {}
for i, n in enumerate(nodes):
    for c in n.get("children", []):
        parents[c] = i
parents[587] = None
inv = np.linalg.inv(root)
for i, n in enumerate(nodes):
    if "mesh" not in n:
        continue
    pts = []
    for prim in doc["meshes"][n["mesh"]]["primitives"]:
        if "POSITION" in prim["attributes"]:
            p = g._positions(doc, binary, prim["attributes"]["POSITION"])
            pts.append((world[i][:3, :3] @ p.T).T + world[i][:3, 3])
    if not pts:
        continue
    pts = np.vstack(pts)
    loc = (inv[:3, :3] @ pts.T).T + inv[:3, 3]
    st = np.stack([loc[:, 0], -loc[:, 2], loc[:, 1]], 1)
    lo, hi = st.min(0), st.max(0)
    rows.append((path(i, parents), lo, hi, len(pts)))
for name, lo, hi, nv in rows:
    e = hi - lo
    c = (hi + lo) / 2
    print(f"{name} | ext {e[0]:.1f} {e[1]:.1f} {e[2]:.1f} | ctr {c[0]:.1f} {c[1]:.1f} {c[2]:.1f} | n {nv}")
