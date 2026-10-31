"""Ship quick-disconnect arm (the tower's lower arm at the ship's aft): static, see mechazilla_layout.py.

Run: blender -b --factory-startup -P gen_qd_arm.py -- --config <toml> --out <dir>
Root node `qd_arm`; nodes `truss` (black) and `hardware` (light grey cabinets, deck, connector head).
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
    root = bpy.data.objects.new("qd_arm", None)
    bpy.context.scene.collection.objects.link(root)
    print(f"TRIANGLES qd_arm {common.add_layout(mechazilla_layout.qd_arm(detail), root)}")
    common.export_gltf(out / "qd_arm.gltf")


main()
