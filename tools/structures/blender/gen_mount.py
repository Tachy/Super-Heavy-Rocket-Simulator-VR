"""Pad B launch mount (pyramid, ring, hoods, QDs, tunnels, chamber): static, see mount_layout.py.

Run: blender -b --factory-startup -P gen_mount.py -- --config <toml> --out <dir>
Frame: X toward the tower, Y across, Z up, origin on the mount axis at the ground. Nodes: shell, plinth, interior,
deflector, hood, hardware, stair, door.
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
    root = bpy.data.objects.new("mount", None)
    bpy.context.scene.collection.objects.link(root)
    print(f"TRIANGLES mount {common.add_surfaces(mount_layout.mount(detail), root)}")
    common.export_gltf(out / "mount.gltf")


main()
