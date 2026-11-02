"""The Dragon's meshes have no hole you can look through: open edges per file, welded across its parts.

Edges that stay open must be on a list with a reason. Run after build_models; skipped without generated models.
"""
import base64
import collections
import json
import math
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_models  # noqa: E402

MODELS = build_models.ROOT / "data" / "models" / "crew_dragon"
WELD = 1e4  # 0.1 mm


def _buffers(path, doc):
    out = []
    for b in doc["buffers"]:
        uri = b["uri"]
        out.append(base64.b64decode(uri.split(",", 1)[1]) if uri.startswith("data:") else (path.parent / uri).read_bytes())
    return out


def _accessor(doc, buffers, i):
    a = doc["accessors"][i]
    v = doc["bufferViews"][a["bufferView"]]
    off = v.get("byteOffset", 0) + a.get("byteOffset", 0)
    data = buffers[v["buffer"]]
    if a["type"] == "VEC3":
        stride = v.get("byteStride", 12)
        return [struct.unpack_from("<3f", data, off + k * stride) for k in range(a["count"])]
    fmt, size = {5123: ("<H", 2), 5125: ("<I", 4), 5121: ("<B", 1)}[a["componentType"]]
    return [struct.unpack_from(fmt, data, off + k * size)[0] for k in range(a["count"])]


def open_edges(file_name, nodes=None):
    """{node: [(x, r), ...] midpoints of edges used by one triangle in the whole file (or among `nodes`)}."""
    path = MODELS / file_name
    doc = json.loads(path.read_text(encoding="utf-8"))
    buffers = _buffers(path, doc)
    edges = collections.Counter()
    owner = {}
    for node in doc["nodes"]:
        if "mesh" not in node or (nodes is not None and node["name"] not in nodes):
            continue
        for prim in doc["meshes"][node["mesh"]]["primitives"]:
            pos = _accessor(doc, buffers, prim["attributes"]["POSITION"])
            idx = _accessor(doc, buffers, prim["indices"])
            key = [tuple(round(c * WELD) for c in p) for p in pos]
            for t in range(0, len(idx), 3):
                a, b, c = key[idx[t]], key[idx[t + 1]], key[idx[t + 2]]
                for e in ((a, b), (b, c), (c, a)):
                    k = tuple(sorted(e))
                    if k[0] == k[1]:
                        continue  # a triangle collapsed by the weld
                    edges[k] += 1
                    owner[k] = node["name"]
    result = collections.defaultdict(list)
    for k, n in edges.items():
        if n == 1:
            mid = [(k[0][i] + k[1][i]) / 2.0 / WELD for i in range(3)]
            result[owner[k]].append((mid[0], math.hypot(mid[1], mid[2])))
    return result


def test_trunk_is_closed():
    if not (MODELS / "dragon_trunk.gltf").is_file():
        pytest.skip("no generated Dragon models")
    open_ = open_edges("dragon_trunk.gltf", {"trunk", "umbilical"})
    # The one seam: the shell ends at r base_inner_r_m on station 0, where the base disc in the array's paint takes over (other material).
    assert all(x == pytest.approx(0.0, abs=1e-3) and r == pytest.approx(1.78, abs=0.01) for x, r in open_["trunk"])
    assert len(open_["trunk"]) == 64
    assert open_["umbilical"] == []


def test_trunk_panels_lie_on_the_hull():
    """Panels are single sheets 12 mm proud: their edges are open, but behind them is the closed hull, so no hole shows."""
    if not (MODELS / "dragon_trunk.gltf").is_file():
        pytest.skip("no generated Dragon models")
    open_ = open_edges("dragon_trunk.gltf", {"trunk_radiator"})
    assert all(1.85 < r < 1.87 for _, r in open_["trunk_radiator"])
