"""Run inside Blender (test_build_models.py): builds non-convex members through MeshBuilder and prints the end cap areas."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
import common  # noqa: E402

LENGTH_M = 1.0
result = {}
for kind, size, wall in (("h_beam", 0.5, 0.04), ("angle", 0.5, 0.04)):
    common.clear_scene()
    builder = common.MeshBuilder()
    builder.add_member(kind, size, wall, (0.0, 0.0, 0.0), (0.0, 0.0, LENGTH_M))
    ob, _ = builder.to_object(kind)
    caps = {0.0: 0.0, LENGTH_M: 0.0}
    for poly in ob.data.polygons:
        zs = {round(ob.data.vertices[i].co.z, 6) for i in poly.vertices}
        if len(zs) == 1 and next(iter(zs)) in caps:
            caps[next(iter(zs))] += poly.area
    result[kind] = [caps[0.0], caps[LENGTH_M]]
print("CAPS " + json.dumps(result))
