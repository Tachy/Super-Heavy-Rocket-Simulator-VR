"""Chopstick arm of Pad 2: U of big round black tubes, flat bearing plate (layout and frame: mechazilla_layout.py, references there).

Run: blender -b --factory-startup -P gen_chopstick.py -- --config <toml> --out <dir> [--preview <png>]
Nodes: `truss` (black tubes), `bearing` (flat bearing plate), `strip` (tan strip), `pad` (grey contact pad), `hardware`
(light grey: platform, pushers, probe head, thin supply lines), `fittings` (black: ladders, probe brace).
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
    root = bpy.data.objects.new("chopstick", None)
    bpy.context.scene.collection.objects.link(root)
    tris = common.add_layout(mechazilla_layout.chopstick(detail), root)
    print(f"TRIANGLES chopstick {tris}")
    common.export_gltf(out / "chopstick.gltf")
    if "preview" in args:
        c = detail["chopstick"]
        Path(args["preview"]).parent.mkdir(parents=True, exist_ok=True)
        common.render_preview(str(Path(args["preview"]).resolve()), (c["length_m"] * 0.5, 0.0, -c["depth_m"] * 0.45),
                              c["length_m"] * 2.1, -62.0, 14.0)


main()
