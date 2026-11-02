"""Tests for tools/structures/measure/iss_measure.py (frame mapping, scale, port fit; real-model checks skip without the file)."""
import json
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

_REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO / "tools" / "structures" / "measure"))

import iss_measure as im  # noqa: E402

_MODEL = _REPO / "screenshots" / "falcon9_ref" / "models" / "iss" / "d_igoal" / "ISS_D_IGOAL.glb"

def _draco_ok():
    import os
    pylibs = os.environ.get("STARSIM_PYLIBS")
    if pylibs and pylibs not in sys.path:
        sys.path.append(pylibs)
    try:
        import DracoPy  # noqa: F401
        return True
    except ImportError:
        return False


_needs_model = pytest.mark.skipif(not _MODEL.exists() or not _draco_ok(),
                                  reason="NASA ISS model D or the DracoPy decoder (STARSIM_PYLIBS) not available")


def _box(lo, hi):
    return [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]


def _write_glb(path, boxes):
    """Root with a negative uniform scale (as the NASA file) and one mesh child per (name, lo, hi) in root-local axes."""
    binary, accessors, meshes, nodes = b"", [], [], []
    for n, (name, lo, hi) in enumerate(boxes):
        binary += b"".join(struct.pack("<3f", *c) for c in _box(lo, hi))
        accessors.append({"bufferView": 0, "byteOffset": 96 * n // 8 * 1, "componentType": 5126, "count": 8, "type": "VEC3",
                          "min": list(lo), "max": list(hi)})
        meshes.append({"primitives": [{"attributes": {"POSITION": n}}]})
        nodes.append({"name": name, "mesh": n})
    for a in accessors:
        a["byteOffset"] = accessors.index(a) * 96
    root = {"name": "root", "scale": [-0.5, -0.5, -0.5], "children": list(range(len(boxes)))}
    nodes.append(root)
    doc = {"asset": {"version": "2.0"}, "scene": 0, "scenes": [{"nodes": [len(nodes) - 1]}], "nodes": nodes,
           "meshes": meshes, "accessors": accessors,
           "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": len(binary)}], "buffers": [{"byteLength": len(binary)}]}
    js = json.dumps(doc).encode()
    js += b" " * (-len(js) % 4)
    total = 12 + 8 + len(js) + 8 + len(binary)
    with open(path, "wb") as f:
        f.write(struct.pack("<4sII", b"glTF", 2, total))
        f.write(struct.pack("<I4s", len(js), b"JSON") + js)
        f.write(struct.pack("<I4s", len(binary), b"BIN\0") + binary)


def test_station_frame_is_a_proper_rotation_of_the_root_local_axes(tmp_path):
    # root-local: +x forward, +y nadir, +z port; the file's root scale -0.5 must cancel (world = -0.5 * local)
    p = tmp_path / "frame.glb"
    _write_glb(p, [("fwd", (9, -1, -1), (11, 1, 1)), ("port", (-1, -1, 9), (1, 1, 11)), ("nadir", (-1, 9, -1), (1, 11, 1))])
    st = im.Station(p)
    c = {n: st.points(st.index(n)).mean(axis=0) for n in ("fwd", "port", "nadir")}
    assert np.allclose(c["fwd"], [10, 0, 0], atol=1e-4)
    assert np.allclose(c["port"], [0, -10, 0], atol=1e-4)  # port = -y (y starboard)
    assert np.allclose(c["nadir"], [0, 0, 10], atol=1e-4)  # nadir = +z
    assert np.linalg.det(im._station_rotation()) == pytest.approx(1.0)


def test_scale_from_span_pure():
    assert im.scale_from_extent(4288.0, 109.0) == pytest.approx(109.0 / 4288.0)


@_needs_model
def test_scale_from_truss():
    """The 109 m span of the station (truss ends plus the array blanket width turned broadside) after scaling."""
    st = im.Station(_MODEL)
    st.set_scale_from_span()
    span = st.metres(st.array_span_units())
    assert abs(span - im.TRUSS_SPAN_M) <= im.TRUSS_SPAN_TOLERANCE_M
    # independent cross-checks that do not use the span: published segment / array sizes (tolerance 3 %)
    s0 = st.metres(st.extent(st.index("Truss_S0"), axis=1, include_children=False))
    assert abs(s0 - 13.4) <= 0.03 * 13.4  # S0 truss length 13.4 m [S: NASA ITS segment data, recalled, see report]
    assert abs(st.metres_per_unit - 0.0254) <= 0.005 * 0.0254  # the file's vertex unit is the inch


@_needs_model
def test_ida_ports_are_offset_and_in_front_of_harmony():
    st = im.Station(_MODEL)
    st.set_scale_from_span()
    ida2 = st.ida_port("IDA2", axis=(1.0, 0.0, 0.0))
    ida3 = st.ida_port("IDA3", axis=(0.0, 0.0, -1.0))
    assert abs(ida2["plane_m"] - 16.09) < 0.15
    assert abs(ida2["axis_centre_m"][1] - 5.575) < 0.1  # nadir offset of the ring axis from the truss axis
    assert abs(ida3["axis_centre_m"][0] - 11.68) < 0.1
    assert abs(ida3["plane_m"] - (-0.175)) < 0.15  # z of the ring face in the truss-axis frame (zenith port)
