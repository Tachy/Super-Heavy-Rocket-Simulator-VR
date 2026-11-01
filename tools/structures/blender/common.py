"""Shared helpers of the headless Blender generators (run inside Blender 3.4: `blender -b --factory-startup -P ...`).

Members are built by hand with bmesh (no operators, no random numbers, no time stamps) so two runs give byte-identical
glTF buffers. Axes: a member is made along local +Z (origin at one end), then placed between two points.
The detail file is read with the stdlib `tomllib`-free reader below (Blender 3.4 ships Python 3.10): the generators
call `load_detail`, which parses the TOML subset of config/structures/*.toml.
"""
from __future__ import annotations

import math
import re
import os
import sys
from pathlib import Path

import bmesh
import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from member_geometry import member_faces  # noqa: E402



def script_args() -> dict:
    """Arguments after the `--` of the Blender command line: --config <toml> --out <dir> [--preview <png>]."""
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = {}
    for i, a in enumerate(argv):
        if a.startswith("--") and i + 1 < len(argv):
            out[a[2:]] = argv[i + 1]
    for key in ("out", "preview"):  # a relative path lands in Blender's cwd (once created C:\screenshots)
        if key in out and not os.path.isabs(out[key]):
            raise SystemExit(f"--{key} must be an absolute path, got {out[key]!r}")
    return out


def load_detail(path) -> dict:
    """Minimal TOML reader for the detail file (tables, arrays of tables, floats, strings, comments)."""
    root: dict = {}
    cur = root
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        line = re.sub(r"\s#.*$", "", raw).strip() if "#" in raw else raw.strip()
        if not line or line.startswith("#"):
            continue
        m = re.match(r"^\[\[([\w.]+)\]\]$", line)
        if m:
            parts = m.group(1).split(".")
            d = root
            for p in parts[:-1]:
                d = d.setdefault(p, {})
            lst = d.setdefault(parts[-1], [])
            cur = {}
            lst.append(cur)
            continue
        m = re.match(r"^\[([\w.]+)\]$", line)
        if m:
            cur = root
            for p in m.group(1).split("."):
                cur = cur.setdefault(p, {})
            continue
        k, _, v = line.partition("=")
        v = v.strip()
        cur[k.strip()] = v[1:-1] if v.startswith('"') else float(v)
    root.setdefault("mount", {})["plan_yaw_deg"] = site_plan_yaw_deg(SITES_FILE)
    root["site_tower"] = site_tower(SITES_FILE)
    return root


SITE_ID = "starbase_pad2"  # the site whose [site.tower] the structures are drawn on (same as detail_file.SITE_ID)
SITES_FILE = Path(__file__).resolve().parents[3] / "config" / "launch_sites.toml"


def site_plan_yaw_deg(sites_path, site_id=SITE_ID) -> float:
    """`plan_yaw_deg` of `[site.tower]` of the site in config/launch_sites.toml (the physics value; not in the detail file)."""
    current, table = None, None
    for raw in Path(sites_path).read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if line == "[[site]]":
            current, table = None, "site"
        elif line.startswith("["):
            table = line.strip("[]")
        elif table == "site" and line.startswith("id") and "=" in line:
            current = line.partition("=")[2].strip().strip('"')
        elif table == "site.tower" and current == site_id and line.startswith("plan_yaw_deg"):
            return float(line.partition("=")[2])
    raise KeyError(f"{sites_path}: no plan_yaw_deg for site {site_id}")


SITE_TOWER_KEYS = ("azimuth_deg", "distance_m", "width_m", "plan_yaw_deg", "hinge_outside_corner_m", "arm_length_m", "arm_closed_gap_m",
                   "arm_width_m", "catch_height_m", "carriage_stroke_m", "arm_open_deg")


def site_tower(sites_path, site_id=SITE_ID) -> dict:
    """The physics numbers of `[site.tower]` the drawn corner, collars and QD arm are placed with (same keys as detail_file.site_tower)."""
    current, table, out = None, None, {}
    for raw in Path(sites_path).read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if line == "[[site]]":
            current, table = None, "site"
        elif line.startswith("["):
            table = line.strip("[]")
        elif table == "site" and line.startswith("id") and "=" in line:
            current = line.partition("=")[2].strip().strip('"')
        elif table == "site.tower" and current == site_id and "=" in line:
            k, _, v = line.partition("=")
            if k.strip() in SITE_TOWER_KEYS:
                out[k.strip()] = float(v)
    missing = [k for k in SITE_TOWER_KEYS if k not in out]
    if missing:
        raise KeyError(f"{sites_path}: [site.tower] of {site_id} lacks {missing}")
    return out


def clear_scene() -> None:
    """Empty scene (the factory startup has a cube, a light and a camera)."""
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob, do_unlink=True)
    for coll in (bpy.data.meshes, bpy.data.cameras, bpy.data.lights):
        for blk in list(coll):
            coll.remove(blk)


# ---------------------------------------------------------------- mesh building

class MeshBuilder:
    """Collects members into one bmesh; `to_object` turns it into a triangulated mesh object."""

    def __init__(self):
        self.bm = bmesh.new()
        self.uv = self.bm.loops.layers.uv.new("UVMap")

    def add_faces(self, faces, to_world):
        """Faces wound outward by their maker: the vertices are not shared, so recalculating normals would be arbitrary."""
        bm = self.bm
        for verts, uvs, smooth in faces:
            bv = [bm.verts.new(to_world(Vector(p))) for p in verts]
            f = bm.faces.new(bv)
            f.smooth = smooth
            for loop, uv in zip(f.loops, uvs):
                loop[self.uv].uv = uv

    def add_member(self, kind, size, wall, p0, p1, up=(0.0, 0.0, 1.0), size_y=None):
        """Member of the profile between the world points p0 and p1; `up` fixes the roll of non-round sections."""
        p0, p1, up = Vector(p0), Vector(p1), Vector(up)
        axis = p1 - p0
        length = axis.length
        z = axis / length
        x = up.cross(z)
        if x.length < 1e-6:
            x = Vector((1.0, 0.0, 0.0)).cross(z)
        x.normalize()
        y = z.cross(x)
        self.add_faces(member_faces(kind, size, wall, length, size_y), lambda p: p0 + x * p.x + y * p.y + z * p.z)

    def to_object(self, name, parent=None):
        bmesh.ops.triangulate(self.bm, faces=self.bm.faces[:])
        mesh = bpy.data.meshes.new(name)
        self.bm.to_mesh(mesh)
        tris = len(mesh.polygons)
        self.bm.free()
        ob = bpy.data.objects.new(name, mesh)
        bpy.context.scene.collection.objects.link(ob)
        if parent is not None:
            ob.parent = parent
        return ob, tris


def add_layout(members, root) -> int:
    """Members of mechazilla_layout (node, kind, size, p0, p1, size_y) as one mesh object per node; returns the triangles."""
    builders = {}
    for node, kind, size, p0, p1, size_y in members:
        builders.setdefault(node, MeshBuilder()).add_member(kind, size, 0.04, p0, p1, size_y=size_y)
    return sum(builder.to_object(node, root)[1] for node, builder in builders.items())


def add_surfaces(surfaces, root) -> int:
    """Polygons of mount_layout.Surfaces (verts, uvs per node, winding already outward) as one mesh object per node."""
    total = 0
    for node, polys in sorted(surfaces.nodes.items()):
        builder = MeshBuilder()
        for verts, uvs in polys:
            bv = [builder.bm.verts.new(Vector(p)) for p in verts]
            face = builder.bm.faces.new(bv)
            face.smooth = False
            for loop, uv in zip(face.loops, uvs):
                loop[builder.uv].uv = uv
        total += builder.to_object(node, root)[1]
    return total


def export_gltf(path) -> None:
    """glTF (.gltf + .bin), Blender axes kept (export_yup off): arm along +X, top plane at Z = 0."""
    bpy.ops.export_scene.gltf(filepath=str(path), export_format="GLTF_SEPARATE", export_yup=False,
                              export_cameras=False, export_lights=False, export_extras=False)


def render_preview(path, centre, distance, azimuth_deg, elevation_deg, size=(1400, 800), ortho_scale=None):
    """Workbench render of the scene to a PNG (diagnostic only)."""
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.render.resolution_x, scene.render.resolution_y = size
    scene.render.filepath = str(path)
    scene.render.image_settings.file_format = "PNG"
    shading = scene.display.shading
    shading.light = "STUDIO"
    shading.color_type = "SINGLE"
    shading.single_color = (0.12, 0.12, 0.13)
    shading.show_cavity = True
    scene.world = bpy.data.worlds.new("w") if scene.world is None else scene.world
    scene.world.color = (0.45, 0.62, 0.85)
    cam_data = bpy.data.cameras.new("cam")
    cam = bpy.data.objects.new("cam", cam_data)
    scene.collection.objects.link(cam)
    az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
    cen = Vector(centre)
    cam.location = cen + Vector((distance * math.cos(el) * math.cos(az), distance * math.cos(el) * math.sin(az),
                                 distance * math.sin(el)))
    cam.rotation_euler = (cen - cam.location).to_track_quat("-Z", "Y").to_euler()
    if ortho_scale:
        cam_data.type = "ORTHO"
        cam_data.ortho_scale = ortho_scale
    cam_data.clip_end = 1000.0
    scene.camera = cam
    bpy.ops.render.render(write_still=True)
