"""Diagnostic Workbench render of the grid fin in the view of the installed-fin template image (hearts against the sky).

Run: blender -b --factory-startup -P preview_fin_heart.py -- --spec <json> --out <png> [--tilt 38] [--size 900]
         [--set key=value,key=value]  (overrides of the detail file's grid_fin numbers)
The camera looks at the fin from the aft side (below), `tilt` degrees off the cell axis (+X) towards the chord axis (Z), with
the vehicle axis up in the image and the root on the left: the geometry of `real/cmp_fin_template.png` (tilt from
tools/structures/measure/fin_tilt.py: 37.7 deg on the rectified lattice of the template, see the fin-outline report).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402
import gen_grid_fin  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402

COLORS = {"frame": (0.03, 0.03, 0.035, 1), "lattice": (0.03, 0.03, 0.035, 1), "band": (0.85, 0.85, 0.87, 1),
          "pylon": (0.15, 0.15, 0.16, 1), "shaft": (0.3, 0.3, 0.3, 1), "bearing": (0.85, 0.85, 0.87, 1)}
SKY = (0.45, 0.65, 0.95)


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    for item in [s for s in args.get("set", "").split(",") if s]:
        key, value = item.split("=")
        spec["detail"]["grid_fin"][key] = float(value)
    size = int(args.get("size", 900))
    tilt = math.radians(float(args.get("tilt", 38.0)))
    vc.common.clear_scene()
    root, _ = gen_grid_fin.build_fin(spec)
    for ob in root.children:
        ob.color = COLORS.get(ob.name, (0.2, 0.2, 0.2, 1))
        ob.hide_render = ob.name in ("plate", "pylon", "shaft", "bearing")
    d = spec["detail"]["grid_fin"]
    cz = sum(d["outline_z_m"]) / len(d["outline_z_m"])
    cy = sum(d["outline_y_m"]) / len(d["outline_y_m"]) + d["root_gap_m"]
    centre = Vector((0.0, cy, cz))
    toward = Vector((-math.cos(tilt), 0.0, -math.sin(tilt)))  # camera on the aft side, displaced towards -Z
    pos = centre + 12.0 * toward
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.render.resolution_x = scene.render.resolution_y = size
    scene.display.shading.light = "FLAT"
    scene.display.shading.color_type = "OBJECT"
    scene.world = bpy.data.worlds.new("w") if scene.world is None else scene.world
    scene.world.color = SKY
    scene.render.film_transparent = False
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam)
    scene.camera = cam
    cam.data.type = "ORTHO"
    cam.data.ortho_scale = 6.2
    zax = (-toward).normalized() * -1.0  # camera +Z points away from the target
    up = Vector((1.0, 0.0, 0.0))  # the vehicle axis (forward) is up in the image
    xax = up.cross(zax).normalized()
    yax = zax.cross(xax)
    cam.matrix_world = Matrix(((xax.x, yax.x, zax.x, pos.x), (xax.y, yax.y, zax.y, pos.y), (xax.z, yax.z, zax.z, pos.z),
                               (0, 0, 0, 1)))
    scene.render.filepath = args["out"]
    bpy.ops.render.render(write_still=True)


if __name__ == "__main__":
    main()
