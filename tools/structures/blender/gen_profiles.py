"""Profile meshes of the tower members: `profile_<name>`, unit length along +Z, origin at one end, bevelled.

Run: blender -b --factory-startup -P gen_profiles.py -- --config <toml> --out <dir>
The game instances these (MultiMesh) and scales Z to the member length; the cross-section is in metres.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402


def main() -> None:
    args = common.script_args()
    detail = common.load_detail(args["config"])
    out = Path(args["out"])
    out.mkdir(parents=True, exist_ok=True)
    common.clear_scene()
    for name in sorted(detail["profiles"]):
        p = detail["profiles"][name]
        mb = common.MeshBuilder()
        mb.add_member(p["kind"], p["size_m"], p["wall_m"], (0, 0, 0), (0, 0, 1.0), up=(1, 0, 0))
        _, tris = mb.to_object(f"profile_{name}")
        print(f"TRIANGLES profile_{name} {tris}")
    common.export_gltf(out / "profiles.gltf")


main()
