"""Corner rails of the carriage and the tower head, both static (mechazilla_layout.py).

Run: blender -b --factory-startup -P gen_rails.py -- --config <toml> --out <dir>
Root node `rails`; node `rail` (black), the head's nodes `head_roof`, `head_floor`, `head_steel`, `head_wall_a`, `head_wall_b`, `head_band`.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402
import common  # noqa: E402
import mechazilla_layout  # noqa: E402


def main() -> None:
    args = common.script_args()
    detail = common.load_detail(args["config"])
    out = Path(args["out"])
    out.mkdir(parents=True, exist_ok=True)
    common.clear_scene()
    root = bpy.data.objects.new("rails", None)
    bpy.context.scene.collection.objects.link(root)
    triangles = common.add_layout(mechazilla_layout.rails(detail) + mechazilla_layout.head(detail), root)
    plates = {node: [(verts, [(0.0, 0.0)] * len(verts)) for verts in polys] for node, polys in mechazilla_layout.head_faces(detail).items()}
    triangles += common.add_surfaces(SimpleNamespace(nodes=plates), root)
    print(f"TRIANGLES rails {triangles}")
    common.export_gltf(out / "rails.gltf")


main()
