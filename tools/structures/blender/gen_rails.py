"""Corner rails of the carriage (static) (mechazilla_layout.py).

Run: blender -b --factory-startup -P gen_rails.py -- --config <toml> --out <dir>
Root node `rails`; node `rail` (black).
"""
import sys
from pathlib import Path

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
    print(f"TRIANGLES rails {common.add_layout(mechazilla_layout.rails(detail), root)}")
    common.export_gltf(out / "rails.gltf")


main()
