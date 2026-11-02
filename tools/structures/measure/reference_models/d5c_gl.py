# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""Read-only glTF loader for measuring (run with python -I). World-space vertices per mesh node."""
import json, struct, sys
from pathlib import Path
import numpy as np

CT = {5126: ("f", 4), 5123: ("H", 2), 5125: ("I", 4), 5121: ("B", 1)}
NC = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def load(folder):
    folder = Path(folder)
    g = json.loads((folder / "scene.gltf").read_text(encoding="utf-8"))
    bin_ = (folder / g["buffers"][0]["uri"]).read_bytes()
    return g, bin_


def acc(g, b, i):
    a = g["accessors"][i]
    v = g["bufferViews"][a["bufferView"]]
    fmt, sz = CT[a["componentType"]]
    n = NC[a["type"]]
    off = v.get("byteOffset", 0) + a.get("byteOffset", 0)
    stride = v.get("byteStride", sz * n)
    dt = np.dtype("<" + fmt)
    out = np.zeros((a["count"], n), dtype=np.float64)
    for k in range(a["count"]):
        out[k] = np.frombuffer(b, dt, n, off + k * stride)
    return out


def local(node):
    if "matrix" in node:
        return np.array(node["matrix"], dtype=float).reshape(4, 4).T
    t = node.get("translation", [0, 0, 0])
    q = node.get("rotation", [0, 0, 0, 1])
    s = node.get("scale", [1, 1, 1])
    x, y, z, w = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    M = np.eye(4)
    M[:3, :3] = R @ np.diag(s)
    M[:3, 3] = t
    return M


def walk(g):
    """yield (node index, path, world matrix)"""
    def rec(i, M, path):
        n = g["nodes"][i]
        W = M @ local(n)
        p = path + "/" + n.get("name", str(i))
        yield i, p, W
        for c in n.get("children", []):
            yield from rec(c, W, p)
    for r in g["scenes"][g.get("scene", 0)]["nodes"]:
        yield from rec(r, np.eye(4), "")


def parts(folder):
    """list of dict(path, node, material, verts(world), tris(indices))"""
    g, b = load(folder)
    out = []
    for i, p, W in walk(g):
        n = g["nodes"][i]
        if "mesh" not in n:
            continue
        for prim in g["meshes"][n["mesh"]]["primitives"]:
            v = acc(g, b, prim["attributes"]["POSITION"])
            v = (W[:3, :3] @ v.T).T + W[:3, 3]
            idx = acc(g, b, prim["indices"]).astype(int).ravel() if "indices" in prim else np.arange(len(v))
            mat = g["materials"][prim["material"]].get("name", "?") if "material" in prim else "?"
            out.append({"path": p, "node": i, "material": mat, "verts": v, "tris": idx.reshape(-1, 3)})
    return g, out


if __name__ == "__main__":
    g, ps = parts(sys.argv[1])
    print("animations", [(a.get("name"), [ (c["target"].get("node"), c["target"]["path"]) for c in a["channels"]][:40]) for a in g.get("animations", [])])
    for p in ps:
        lo, hi = p["verts"].min(0), p["verts"].max(0)
        print(f'{p["path"][-50:]:50s} n{p["node"]:3d} {p["material"][:16]:16s} v={len(p["verts"]):6d} lo={np.round(lo,3)} hi={np.round(hi,3)}')
