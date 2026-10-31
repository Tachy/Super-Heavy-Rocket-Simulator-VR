"""Carriage of the chopsticks: frame across the tower, hinge frames, platforms, sheave block (mechazilla_layout.py).

Run: blender -b --factory-startup -P gen_carriage.py -- --config <toml> --out <dir>
Root node `carriage`; nodes `frame` (black) and `hardware` (light grey).
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
    root = bpy.data.objects.new("carriage", None)
    bpy.context.scene.collection.objects.link(root)
    print(f"TRIANGLES carriage {common.add_layout(mechazilla_layout.carriage(detail), root)}")
    common.export_gltf(out / "carriage.gltf")


main()
