"""Tests for tools/structures/measure/glb_measure.py (header + accessor reading, no execution)."""
import json
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "measure"))

import glb_measure  # noqa: E402

# 8 corners of a unit cube centred on the origin (glTF accessor min/max are what the tool reads;
# the buffer holds the same data so a reader that ignores min/max still agrees).
_CORNERS = [(x, y, z) for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)]


def _write_glb(path, nodes, meshes, accessors, binary):
    doc = {
        "asset": {"version": "2.0"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": nodes,
        "meshes": meshes,
        "accessors": accessors,
        "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": len(binary)}],
        "buffers": [{"byteLength": len(binary)}],
    }
    js = json.dumps(doc).encode()
    js += b" " * (-len(js) % 4)
    binary += b"\0" * (-len(binary) % 4)
    total = 12 + 8 + len(js) + 8 + len(binary)
    with open(path, "wb") as f:
        f.write(struct.pack("<4sII", b"glTF", 2, total))
        f.write(struct.pack("<I4s", len(js), b"JSON") + js)
        f.write(struct.pack("<I4s", len(binary), b"BIN\0") + binary)


def _cube_glb(path, translation=(0.0, 0.0, 0.0), scale=1.0):
    binary = b"".join(struct.pack("<3f", *c) for c in _CORNERS)
    acc = [{"bufferView": 0, "componentType": 5126, "count": 8, "type": "VEC3",
            "min": [-0.5] * 3, "max": [0.5] * 3}]
    node = {"name": "cube", "mesh": 0, "translation": list(translation), "scale": [scale] * 3}
    meshes = [{"primitives": [{"attributes": {"POSITION": 0}}]}]
    _write_glb(path, [node], meshes, acc, binary)


def test_bbox_of_a_unit_cube_glb(tmp_path):
    p = tmp_path / "cube.glb"
    _cube_glb(p)
    box = glb_measure.bbox(p)
    for e in box["extent"]:
        assert abs(e - 1.0) < 1e-6
    assert all(abs(c) < 1e-6 for c in box["center"])


def test_bbox_applies_node_transform_and_filter(tmp_path):
    p = tmp_path / "cube2.glb"
    _cube_glb(p, translation=(10.0, 0.0, 0.0), scale=2.0)
    box = glb_measure.bbox(p)
    assert abs(box["extent"][0] - 2.0) < 1e-6
    assert abs(box["min"][0] - 9.0) < 1e-6
    assert glb_measure.bbox(p, node_filter="nomatch")["extent"] == [0.0, 0.0, 0.0]
    assert abs(glb_measure.bbox(p, node_filter="cub")["extent"][1] - 2.0) < 1e-6


def test_node_positions_world_translation(tmp_path):
    p = tmp_path / "cube3.glb"
    _cube_glb(p, translation=(1.0, 2.0, 3.0))
    pos = glb_measure.node_positions(p)
    assert pos["cube"] == (1.0, 2.0, 3.0)
