"""Diagnostic Workbench renders of the grid fin alone (not part of the model build).

Run: blender -b --factory-startup -P preview_fin.py -- --spec <json> --out <png dir> [--size 900]
Writes fin_face.png (looking along the cells from the forward side, root up, like the factory photo's orientation) and
fin_sky.png (from below / aft against a sky, the heart shapes of the installed-fin photo) into <png dir>.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402
import gen_grid_fin  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

COLORS = {"frame": (0.12, 0.12, 0.13, 1), "lattice": (0.03, 0.03, 0.035, 1), "plate": (0.03, 0.03, 0.035, 1),
          "band": (0.85, 0.85, 0.87, 1), "pylon": (0.15, 0.15, 0.16, 1), "shaft": (0.3, 0.3, 0.3, 1), "bearing": (0.85, 0.85, 0.87, 1)}


def _shoot(name, pos, target, up, out, size, sky, ortho=0.0, hide=()):
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.render.resolution_x = scene.render.resolution_y = size
    scene.display.shading.light = "STUDIO"
    scene.display.shading.color_type = "OBJECT"
    scene.display.shading.show_cavity = False
    scene.world = bpy.data.worlds.new("w") if scene.world is None else scene.world
    scene.world.color = sky
    cam = bpy.data.objects.get("cam") or bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    if cam.name not in scene.collection.objects:
        scene.collection.objects.link(cam)
    scene.camera = cam
    cam.data.lens = 50
    if ortho:
        cam.data.type = "ORTHO"
        cam.data.ortho_scale = ortho
    cam.location = Vector(pos)
    direction = Vector(target) - Vector(pos)
    cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    # roll so that `up` points up in the image
    z = (-direction).normalized()
    x = Vector(up).cross(z).normalized()
    y = z.cross(x)
    from mathutils import Matrix
    m = Matrix(((x.x, y.x, z.x, pos[0]), (x.y, y.y, z.y, pos[1]), (x.z, y.z, z.z, pos[2]), (0, 0, 0, 1)))
    cam.matrix_world = m
    for ob in bpy.data.objects:
        if ob.name in COLORS:
            ob.hide_render = ob.name in hide
    scene.render.filepath = str(Path(out) / name)
    bpy.ops.render.render(write_still=True)


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    size = int(args.get("size", 900))
    vc.common.clear_scene()
    root, tris = gen_grid_fin.build_fin(spec)
    for ob in root.children:
        ob.color = COLORS.get(ob.name, (0.2, 0.2, 0.2, 1))
    d = spec["detail"]["grid_fin"]
    cz = sum(d["outline_z_m"]) / len(d["outline_z_m"])
    cy = sum(d["outline_y_m"]) / len(d["outline_y_m"]) + d["root_gap_m"]
    centre = (0.0, cy, cz)
    out = args["out"]
    _shoot("fin_face.png", (10.0, cy - 2.5, cz + 2.0), centre, (0, -1, 0), out, size, (0.75, 0.75, 0.78), ortho=5.0, hide=("plate",))
    # from below / aft, 24 deg off the cell axis towards the root side and sideways
    a = math.radians(24.0)
    pos = (-9.0 * math.cos(a), cy - 9.0 * math.sin(a) * 0.5, cz + 9.0 * math.sin(a) * 0.8)
    _shoot("fin_bar.png", (-2.4, 1.6, 2.2), (0.0, 0.3, 0.1), (0, 1, 0), out, size, (0.45, 0.65, 0.95), hide=("plate",))
    _shoot("fin_sky.png", pos, centre, (0, 1, 0), out, size, (0.45, 0.65, 0.95), hide=("plate",))


if __name__ == "__main__":
    main()
