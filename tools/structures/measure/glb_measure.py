"""Measure a binary glTF (.glb) without loading or executing it: bounding boxes and node positions.

Only the JSON header and the POSITION accessors of the BIN chunk are read (struct + json + numpy).
Used for reference models (git-ignored, untrusted files), never for shipped data.
Units are those of the file (not necessarily metres); scale by a known dimension.

    python -I tools/structures/measure/glb_measure.py <file.glb> [node_name_substring]
"""
import json
import struct
import sys
from pathlib import Path

import numpy as np

_GLB_MAGIC = b"glTF"
_CHUNK_JSON = 0x4E4F534A  # "JSON", glTF 2.0 spec section 4.4.3
_CHUNK_BIN = 0x004E4942  # "BIN\0"
_FLOAT = 5126  # accessor componentType FLOAT, glTF 2.0 spec table


def _load(path):
    data = Path(path).read_bytes()
    magic, version, _length = struct.unpack_from("<4sII", data, 0)
    if magic != _GLB_MAGIC or version != 2:
        raise ValueError(f"{path}: not a glTF 2.0 binary")
    off, doc, binary = 12, None, b""
    while off + 8 <= len(data):
        clen, ctype = struct.unpack_from("<II", data, off)
        body = data[off + 8: off + 8 + clen]
        if ctype == _CHUNK_JSON:
            doc = json.loads(body.decode("utf-8"))
        elif ctype == _CHUNK_BIN:
            binary = body
        off += 8 + clen
    if doc is None:
        raise ValueError(f"{path}: no JSON chunk")
    return doc, binary


def _local_matrix(node):
    if "matrix" in node:
        return np.array(node["matrix"], dtype=float).reshape(4, 4).T  # column-major in the file
    t = node.get("translation", [0.0, 0.0, 0.0])
    q = node.get("rotation", [0.0, 0.0, 0.0, 1.0])
    s = node.get("scale", [1.0, 1.0, 1.0])
    x, y, z, w = q
    rot = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    m = np.eye(4)
    m[:3, :3] = rot * np.array(s)[None, :]
    m[:3, 3] = t
    return m


def _world_matrices(doc):
    nodes = doc.get("nodes", [])
    world = [None] * len(nodes)

    def walk(i, parent):
        world[i] = parent @ _local_matrix(nodes[i])
        for c in nodes[i].get("children", []):
            walk(c, world[i])

    scene = doc.get("scenes", [{"nodes": range(len(nodes))}])[doc.get("scene", 0)]
    for r in scene.get("nodes", []):
        walk(r, np.eye(4))
    for i in range(len(nodes)):  # nodes outside the scene: treat as roots
        if world[i] is None:
            walk(i, np.eye(4))
    return world


def _positions(doc, binary, accessor_index):
    """Vertices of a POSITION accessor; falls back to the 8 min/max box corners when no float data."""
    acc = doc["accessors"][accessor_index]
    view = doc.get("bufferViews", [None])[acc["bufferView"]] if "bufferView" in acc else None
    if view is not None and acc["componentType"] == _FLOAT and "sparse" not in acc:
        stride = view.get("byteStride", 12)
        start = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
        count = acc["count"]
        raw = np.frombuffer(binary, dtype=np.uint8, count=(count - 1) * stride + 12, offset=start)
        idx = np.arange(count)[:, None] * stride + np.arange(12)[None, :]
        return raw[idx].copy().view("<f4").reshape(count, 3).astype(float)
    lo, hi = acc["min"], acc["max"]
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def bbox(path, node_filter=None):
    """World-space axis-aligned bounding box of all mesh nodes whose name contains node_filter.

    Returns {"min", "max", "extent", "center"} as 3-lists; all zeros when nothing matches.
    """
    doc, binary = _load(path)
    world = _world_matrices(doc)
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for i, node in enumerate(doc.get("nodes", [])):
        if "mesh" not in node:
            continue
        if node_filter and node_filter not in node.get("name", ""):
            continue
        for prim in doc["meshes"][node["mesh"]]["primitives"]:
            if "POSITION" not in prim["attributes"]:
                continue
            p = _positions(doc, binary, prim["attributes"]["POSITION"])
            w = (world[i][:3, :3] @ p.T).T + world[i][:3, 3]
            lo = np.minimum(lo, w.min(axis=0))
            hi = np.maximum(hi, w.max(axis=0))
    if not np.all(np.isfinite(lo)):
        lo = hi = np.zeros(3)
    return {"min": lo.tolist(), "max": hi.tolist(),
            "extent": (hi - lo).tolist(), "center": ((hi + lo) / 2).tolist()}


def node_positions(path):
    """World translation of every named node: {name: (x, y, z)} (the last one wins for duplicate names)."""
    doc, _ = _load(path)
    world = _world_matrices(doc)
    return {n.get("name", f"node{i}"): tuple(float(v) for v in world[i][:3, 3])
            for i, n in enumerate(doc.get("nodes", []))}


if __name__ == "__main__":
    f = sys.argv[1]
    flt = sys.argv[2] if len(sys.argv) > 2 else None
    print(json.dumps(bbox(f, flt), indent=1))
