"""Diagnostic Workbench renders of the crew figure seated on a Dragon seat (not part of the model build, not hashed).

Run: blender -b --factory-startup -P preview_crew_figure.py -- --spec <json> --out <png dir> [--seat s2]
Builds the cabin (gen_dragon_cabin.main into a scratch dir under <png dir>), places the figure at the seat's hip point with the seat's
orientation and renders: the view from the hatch into the cabin, the figure from the crew's left (shell hidden) and from the console.
"""
import math
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402
import figure_layout as fl  # noqa: E402
import gen_crew_figure  # noqa: E402
import gen_dragon_cabin  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402


def render(path, location, target, lens_mm, size=(1400, 1000)):
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.render.resolution_x, scene.render.resolution_y = size
    scene.render.filepath = str(path)
    scene.render.image_settings.file_format = "PNG"
    shading = scene.display.shading
    shading.light = "STUDIO"
    shading.color_type = "MATERIAL"
    shading.show_cavity = True
    if scene.world is None:
        scene.world = bpy.data.worlds.new("w")
    scene.world.color = (0.18, 0.19, 0.21)
    cam_data = bpy.data.cameras.new("cam")
    cam_data.lens = lens_mm
    cam_data.clip_start, cam_data.clip_end = 0.02, 50.0
    cam = bpy.data.objects.new("cam", cam_data)
    scene.collection.objects.link(cam)
    cam.location = Vector(location)
    cam.rotation_euler = (Vector(target) - cam.location).to_track_quat("-Z", "Y").to_euler()
    scene.camera = cam
    bpy.ops.render.render(write_still=True)
    bpy.data.objects.remove(cam)


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    out = Path(args["out"])
    out.mkdir(parents=True, exist_ok=True)
    sys.argv = [sys.argv[0], "--", "--spec", args["spec"], "--out", tempfile.mkdtemp()]
    gen_dragon_cabin.main()  # builds the cabin objects into the scene (the glTF goes to a scratch directory)
    seat = next(s for s in spec["cockpit"]["seat"] if s["id"] == args.get("seat", "s2"))
    rig, fig, tris = gen_crew_figure.assemble(spec["crew_figure"])
    bx, by, bz = (Vector(v) for v in fl.seat_basis(seat["back_angle_deg"]))
    m = Matrix([[bx[i], by[i], bz[i], seat["hip_m"][i]] for i in range(3)] + [[0, 0, 0, 1]])
    rig.matrix_world = m
    hip = Vector(seat["hip_m"])
    # 1: from the hatch, inside the cabin, looking across at the commander and the console
    render(out / "figure_s2_hatch_view.png", (2.2, 1.15, 0.9), hip + Vector((0.35, 0.0, 0.05)), 32)
    for o in bpy.data.objects:  # the shell, windows and hatch frame hide a view from outside the wall
        if o.name in ("cabin_shell", "floor", "hatch_frame") or o.name.startswith("window_"):
            o.hide_render = True
    # 2: from the crew's left (+Z) at the seated profile
    render(out / "figure_s2_side_left.png", hip + Vector((0.3, -0.1, 3.2)), hip + Vector((0.4, 0.0, 0.0)), 85)
    # 3: three-quarter front view from ahead and the crew's left (other seats and the console hidden)
    for o in bpy.data.objects:
        if o.name in ("console_housing", "seat_s3", "seat_s1", "seat_s4"):
            o.hide_render = True
    mid = hip + bx * 0.40 + by * 0.30
    render(out / "figure_s2_front_left.png", mid + by * 1.7 + bz * 0.9 + bx * 0.2, mid, 40)
    # 4: helmet, visor and shoulders from the front
    head = hip + bx * 0.80 + by * 0.16
    render(out / "figure_s2_helmet_close.png", head + by * 0.95 + bz * 0.30 + bx * 0.05, head - bx * 0.05, 50)
    print("triangles", tris)


if __name__ == "__main__":
    main()
