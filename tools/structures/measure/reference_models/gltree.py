# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""glTF (.gltf + .bin) tree dump with world-space vertex bboxes. Reads data only."""
import json, sys
from pathlib import Path
import numpy as np

def load(folder):
    folder = Path(folder)
    doc = json.loads((folder / "scene.gltf").read_text(encoding="utf-8"))
    bins = [(folder / b["uri"]).read_bytes() for b in doc["buffers"]]
    return doc, bins

def local(n):
    if "matrix" in n:
        return np.array(n["matrix"], float).reshape(4, 4).T
    x, y, z, w = n.get("rotation", [0, 0, 0, 1])
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    m = np.eye(4); m[:3, :3] = R * np.array(n.get("scale", [1, 1, 1]))[None, :]
    m[:3, 3] = n.get("translation", [0, 0, 0]); return m

def acc_data(doc, bins, ai):
    a = doc["accessors"][ai]; v = doc["bufferViews"][a["bufferView"]]
    ncomp = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[a["type"]]
    dt = {5126: "<f4", 5125: "<u4", 5123: "<u2", 5121: "u1", 5122: "<i2", 5120: "i1"}[a["componentType"]]
    isz = np.dtype(dt).itemsize
    stride = v.get("byteStride", ncomp * isz)
    off = v.get("byteOffset", 0) + a.get("byteOffset", 0)
    buf = bins[v["buffer"]]
    raw = np.frombuffer(buf, np.uint8, offset=off, count=(a["count"] - 1) * stride + ncomp * isz)
    idx = np.arange(a["count"])[:, None] * stride + np.arange(ncomp * isz)[None, :]
    return raw[idx].copy().view(dt).reshape(a["count"], ncomp)

def build(folder):
    doc, bins = load(folder)
    nodes = doc["nodes"]; parent = {}
    for i, n in enumerate(nodes):
        for c in n.get("children", []): parent[c] = i
    world = [None] * len(nodes)
    def walk(i, P):
        world[i] = P @ local(nodes[i])
        for c in nodes[i].get("children", []): walk(c, world[i])
    roots = doc["scenes"][doc.get("scene", 0)]["nodes"]
    for r in roots: walk(r, np.eye(4))
    return doc, bins, world, parent, roots

def node_verts(doc, bins, world, i):
    """World-space vertices of node i (own mesh only)."""
    n = doc["nodes"][i]
    if "mesh" not in n: return None
    out = []
    for p in doc["meshes"][n["mesh"]]["primitives"]:
        P = acc_data(doc, bins, p["attributes"]["POSITION"]).astype(float)
        out.append((world[i][:3, :3] @ P.T).T + world[i][:3, 3])
    return np.vstack(out)

def dump(folder, maxdepth=99):
    doc, bins, world, parent, roots = build(folder)
    nodes = doc["nodes"]
    def rec(i, d):
        n = nodes[i]
        V = node_verts(doc, bins, world, i)
        s = "  " * d + f"[{i}] {n.get('name')}"
        t = world[i][:3, 3]
        s += f" pos=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})"
        if V is not None:
            lo, hi = V.min(0), V.max(0)
            s += f" nv={len(V)} min=({lo[0]:.3f},{lo[1]:.3f},{lo[2]:.3f}) max=({hi[0]:.3f},{hi[1]:.3f},{hi[2]:.3f})"
        print(s)
        if d < maxdepth:
            for c in n.get("children", []): rec(c, d + 1)
    for r in roots: rec(r, 0)
    print("animations:", [(a.get("name"), [(c["target"].get("node"), c["target"]["path"]) for c in a["channels"]][:12]) for a in doc.get("animations", [])])

if __name__ == "__main__":
    dump(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 99)
