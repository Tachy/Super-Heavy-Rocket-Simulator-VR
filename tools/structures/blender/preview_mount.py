"""Diagnostic Workbench render of the mount and trench (not part of the build).

Run: blender -b --factory-startup -P preview_mount.py -- --config <toml> --preview <abs png> [--az 215] [--el 30] [--dist 110]
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
    common.clear_scene()
    root = bpy.data.objects.new("mount", None)
    bpy.context.scene.collection.objects.link(root)
    common.add_surfaces(mount_layout.mount(detail), root)
    common.add_surfaces(mount_layout.trench(detail), root)
    common.render_preview(args["preview"], (0.0, 0.0, 8.0), float(args.get("dist", 110.0)), float(args.get("az", 215.0)),
                          float(args.get("el", 30.0)), size=(1400, 900))


main()
