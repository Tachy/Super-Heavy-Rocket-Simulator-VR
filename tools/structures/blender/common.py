"""Shared helpers of the headless Blender generators (run inside Blender 3.4: `blender -b --factory-startup -P ...`).

Members are built by hand with bmesh (no operators, no random numbers, no time stamps) so two runs give byte-identical
glTF buffers. Axes: a member is made along local +Z (origin at one end), then placed between two points.
The detail file is read with the stdlib `tomllib`-free reader below (Blender 3.4 ships Python 3.10): the generators
call `load_detail`, which parses the TOML subset of config/structures/*.toml.
"""
from __future__ import annotations

import math
import re
import sys
from pathlib import Path

import bmesh
import bpy
from mathutils import Vector

TUBE_SIDES = 12  # [E] 12-gon reads round at the 0.5-0.65 m tubes from >= 5 m; a member is instanced thousands of times
TUBE_CHAMFER_FRAC = 0.2  # [E] end chamfer as a fraction of the tube radius (bevel, catches a highlight)


def script_args() -> dict:
    """Arguments after the `--` of the Blender command line: --config <toml> --out <dir> [--preview <png>]."""
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = {}
    for i, a in enumerate(argv):
        if a.startswith("--") and i + 1 < len(argv):
            out[a[2:]] = argv[i + 1]
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
    return root


def clear_scene() -> None:
    """Empty scene (the factory startup has a cube, a light and a camera)."""
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob, do_unlink=True)
    for coll in (bpy.data.meshes, bpy.data.cameras, bpy.data.lights):
        for blk in list(coll):
            coll.remove(blk)


# ---------------------------------------------------------------- member geometry (local frame, +Z along the member)

def _prism(outline, length):
    """Extruded polygon: list of (verts, uvs, smooth) faces. Flat sides, polygon end caps."""
    faces = []
    n = len(outline)
    run = [0.0]
    for i in range(n):
        a, b = outline[i], outline[(i + 1) % n]
        run.append(run[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    for i in range(n):
        a, b = outline[i], outline[(i + 1) % n]
        v = [(a[0], a[1], 0.0), (b[0], b[1], 0.0), (b[0], b[1], length), (a[0], a[1], length)]
        uv = [(run[i], 0.0), (run[i + 1], 0.0), (run[i + 1], length), (run[i], length)]
        faces.append((v, uv, False))
    for z in (0.0, length):
        faces.append(([(p[0], p[1], z) for p in outline], [(p[0], p[1]) for p in outline], False))
    return faces


def _octagon(sx, sy, bevel):
    hx, hy, b = sx / 2.0, sy / 2.0, min(bevel, sx / 4.0, sy / 4.0)
    return [(hx - b, -hy), (hx, -hy + b), (hx, hy - b), (hx - b, hy),
            (-hx + b, hy), (-hx, hy - b), (-hx, -hy + b), (-hx + b, -hy)]


def _h_outline(s, t):
    h = s / 2.0
    w = t / 2.0
    return [(-h, -h), (h, -h), (h, -h + t), (w, -h + t), (w, h - t), (h, h - t),
            (h, h), (-h, h), (-h, h - t), (-w, h - t), (-w, -h + t), (-h, -h + t)]


def _angle_outline(s, t):
    h = s / 2.0
    return [(-h, -h), (h, -h), (h, -h + t), (-h + t, -h + t), (-h + t, h), (-h, h)]


def _tube(size, length, n=TUBE_SIDES):
    """Round bar with a flat chamfer at both ends: smooth side, flat chamfer and cap faces (separate vertices)."""
    r = size / 2.0
    c = min(r * TUBE_CHAMFER_FRAC, length / 4.0)
    faces = []
    ang = [2.0 * math.pi * i / n for i in range(n)]

    def ring(rad, z):
        return [(rad * math.cos(a), rad * math.sin(a), z) for a in ang]

    def u(i):
        return 2.0 * math.pi * r * i / n

    z0, z1 = c, length - c
    for i in range(n):
        j = (i + 1) % n
        a0, a1, b0, b1 = ring(r, z0)[i], ring(r, z0)[j], ring(r, z1)[i], ring(r, z1)[j]
        faces.append(([a0, a1, b1, b0], [(u(i), z0), (u(i + 1), z0), (u(i + 1), z1), (u(i), z1)], True))
    for start, zc, zs in ((True, 0.0, z0), (False, length, z1)):
        inner, outer = ring(r - c, zc), ring(r, zs)
        for i in range(n):
            j = (i + 1) % n
            faces.append(([inner[i], inner[j], outer[j], outer[i]],
                          [(u(i), zc), (u(i + 1), zc), (u(i + 1), zs), (u(i), zs)], False))
        faces.append((inner, [(p[0], p[1]) for p in inner], False))
    return faces


def member_faces(kind, size, wall, length, size_y=None):
    """Faces of one member of `kind` ('h_beam' | 'tube' | 'angle' | 'box' | 'rect'), cross-section in metres,
    `length` along +Z. 'rect' is a bevelled slab of size x size_y (walkway deck)."""
    if kind == "tube":
        return _tube(size, length)
    if kind == "box":
        return _prism(_octagon(size, size, wall), length)
    if kind == "rect":
        return _prism(_octagon(size, size_y, wall), length)
    if kind == "h_beam":
        return _prism(_h_outline(size, wall), length)
    if kind == "angle":
        return _prism(_angle_outline(size, wall), length)
    raise ValueError(f"unknown member kind '{kind}'")


# ---------------------------------------------------------------- mesh building

class MeshBuilder:
    """Collects members into one bmesh; `to_object` turns it into a triangulated mesh object."""

    def __init__(self):
        self.bm = bmesh.new()
        self.uv = self.bm.loops.layers.uv.new("UVMap")

    def add_faces(self, faces, to_world):
        bm = self.bm
        new_faces = []
        for verts, uvs, smooth in faces:
            bv = [bm.verts.new(to_world(Vector(p))) for p in verts]
            f = bm.faces.new(bv)
            f.smooth = smooth
            for loop, uv in zip(f.loops, uvs):
                loop[self.uv].uv = uv
            new_faces.append(f)
        # per member (vertices are not shared between members): outward normals
        bmesh.ops.recalc_face_normals(bm, faces=new_faces)

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
