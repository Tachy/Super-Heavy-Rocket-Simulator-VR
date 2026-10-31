"""Pad B exit ramps, hardstand and the tower's foot block: static, see mount_layout.py.

Run: blender -b --factory-startup -P gen_trench.py -- --config <toml> --out <dir>
Frame: as gen_mount.py (the ramps lie on the +-Y sides).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402
import common  # noqa: E402
import mount_layout  # noqa: E402


def main() -> None:
    args = common.script_args()
    detail = common.load_detail(args["config"])
    out = Path(args["out"])
    out.mkdir(parents=True, exist_ok=True)
    common.clear_scene()
    root = bpy.data.objects.new("trench", None)
    bpy.context.scene.collection.objects.link(root)
    print(f"TRIANGLES trench {common.add_surfaces(mount_layout.trench(detail), root)}")
    common.export_gltf(out / "trench.gltf")


main()
