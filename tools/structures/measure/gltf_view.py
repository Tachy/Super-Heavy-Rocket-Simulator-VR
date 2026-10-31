"""Shaded orthographic view of a generated glTF (painter's algorithm and conventions as in stl_measure.py `view`), to put
our redraw next to the 1:100 reference model's view. Positions are in metres already.

  gltf_view.py <out.png> <gltf> [--u 0 --v 1 --px 40 --node <name>] [--x0 .. --x1 ..]   (axes 0|1|2 as in stl_measure)
`--x0/--x1` keep only triangles whose axis-0 centre lies in that range (a station window of a stage).
"""
from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def load_triangles(path: str, node: str | None = None) -> np.ndarray:
    g = json.loads(Path(path).read_text(encoding="utf-8"))
    blob = (Path(path).parent / g["buffers"][0]["uri"]).read_bytes()

    def acc(i, ncomp, fmt, size):
        a, v = g["accessors"][i], None
        v = g["bufferViews"][a["bufferView"]]
        start = v.get("byteOffset", 0) + a.get("byteOffset", 0)
        stride = v.get("byteStride", ncomp * size)
        return np.array([struct.unpack_from("<" + fmt * ncomp, blob, start + k * stride) for k in range(a["count"])])

    tris = []
    for n in g["nodes"]:
        if "mesh" not in n or (node and n["name"] != node):
            continue
        for p in g["meshes"][n["mesh"]]["primitives"]:
            pos = acc(p["attributes"]["POSITION"], 3, "f", 4)
            ct = g["accessors"][p["indices"]]["componentType"]
            idx = acc(p["indices"], 1, "H" if ct == 5123 else "I", 2 if ct == 5123 else 4).astype(int).ravel()
            tris.append(pos[idx].reshape(-1, 3, 3))
    return np.concatenate(tris)


def render(tris: np.ndarray, out: str, u: int, v: int, px: float) -> None:
    w = 3 - u - v
    lo, hi = tris.reshape(-1, 3).min(axis=0), tris.reshape(-1, 3).max(axis=0)
    pad = 10
    width, height = int((hi[u] - lo[u]) * px) + 2 * pad, int((hi[v] - lo[v]) * px) + 2 * pad
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    img = Image.new("RGB", (width, height), (20, 20, 30))
    d = ImageDraw.Draw(img)
    for i in np.argsort(tris[:, :, w].mean(axis=1)):
        shade = int(60 + 190 * abs(nrm[i, w]))
        d.polygon([((t[u] - lo[u]) * px + pad, height - ((t[v] - lo[v]) * px + pad)) for t in tris[i]],
                  fill=(shade, shade, shade))
    img.save(out)
    print(f"{out}: {width}x{height}")


def main(argv):
    opts = {argv[i]: argv[i + 1] for i in range(3, len(argv) - 1) if argv[i].startswith("--")}
    tris = load_triangles(argv[2], opts.get("--node"))
    if "--x0" in opts or "--x1" in opts:
        c = tris[:, :, 0].mean(axis=1)
        tris = tris[(c >= float(opts.get("--x0", -1e9))) & (c <= float(opts.get("--x1", 1e9)))]
    render(tris, argv[1], int(opts.get("--u", 0)), int(opts.get("--v", 1)), float(opts.get("--px", 40)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
