"""Shared helpers of the vehicle generators (run inside Blender 3.4: `blender -b --factory-startup -P ...`).

Frame of every vehicle model (Blender axes kept in the glTF, export_yup off): the BODY frame of the config - X is the
station axis (the stage's station 0 at X = 0, the nose toward +X), azimuth a puts a point at (Y, Z) = (r cos a, r sin a),
the belly of the ship is azimuth 270 (-Z). The game maps body (x, y, z) to render (y, z, x) like `Frames.to_render`.

Meshes are built with shared vertices per smooth group and per-loop UVs; every face is given an outward hint so its
winding is right by construction (no recalc on open shells). No random numbers, no time stamps: two runs give
byte-identical glTF buffers. The generators read the spec JSON the driver wrote (`vehicle_spec.py`).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import bmesh
import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402,F401  (clear_scene, export_gltf, render_preview, script_args)

KEY_DIGITS = 6


def load_spec(path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def pt(x: float, azimuth_rad: float, r: float) -> Vector:
    """Body-frame point from station, azimuth and radius."""
    return Vector((x, r * math.cos(azimuth_rad), r * math.sin(azimuth_rad)))


def radial_dir(azimuth_rad: float) -> Vector:
    return Vector((0.0, math.cos(azimuth_rad), math.sin(azimuth_rad)))


class VMesh:
    """bmesh with shared vertices (per smooth group) and per-loop UVs; faces wound by an outward hint."""

    def __init__(self):
        self.bm = bmesh.new()
        self.uv = self.bm.loops.layers.uv.new("UVMap")
        self._verts: dict = {}

    def _vert(self, p: Vector, group):
        if group is None:
            return self.bm.verts.new(p)
        key = (round(p.x, KEY_DIGITS), round(p.y, KEY_DIGITS), round(p.z, KEY_DIGITS), group)
        v = self._verts.get(key)
        if v is None:
            v = self.bm.verts.new(p)
            self._verts[key] = v
        return v

    def face(self, pts, uvs, outward, group=None) -> None:
        """One face. `uvs` are (u, v) in Blender convention (glTF v = 1 - v); `outward` a vector the normal must
        point at; `group` an id: faces of one group share vertices (smooth shading), None = flat."""
        pts = [Vector(p) for p in pts]
        n = Vector((0.0, 0.0, 0.0))
        for i in range(len(pts)):  # Newell normal
            a, b = pts[i], pts[(i + 1) % len(pts)]
            n += Vector(((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y)))
        if n.dot(Vector(outward)) < 0.0:
            pts.reverse()
            uvs = list(reversed(uvs))
        verts, loops_uv = [], []
        for p, uv in zip(pts, uvs):
            v = self._vert(p, group)
            if verts and v is verts[-1]:
                continue
            verts.append(v)
            loops_uv.append(uv)
        if len(verts) > 1 and verts[0] is verts[-1]:
            verts.pop()
            loops_uv.pop()
        if len(set(verts)) < 3:
            return
        f = self.bm.faces.new(verts)
        f.smooth = group is not None
        for loop, uv in zip(f.loops, loops_uv):
            loop[self.uv].uv = uv

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


def empty(name):
    ob = bpy.data.objects.new(name, None)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def revolve(mesh: VMesh, runs, segments, uv_radius_m, group_base):
    """Surface of revolution about the X axis. `runs`: list of (profile [(x, r)...], hint) where hint is "out" / "in"
    (radial) or (+-1, 0, 0) (axial); each run is its own smooth group. UV = (arc metres at `uv_radius_m`, x)."""
    for ri, (profile, hint) in enumerate(runs):
        group = (group_base, ri)
        for k in range(len(profile) - 1):
            (x0, r0), (x1, r1) = profile[k], profile[k + 1]
            for i in range(segments):
                a0, a1 = 2.0 * math.pi * i / segments, 2.0 * math.pi * (i + 1) / segments
                pts = [pt(x0, a0, r0), pt(x0, a1, r0), pt(x1, a1, r1), pt(x1, a0, r1)]
                uvs = [(a0 * uv_radius_m, 1.0 - x0), (a1 * uv_radius_m, 1.0 - x0),
                       (a1 * uv_radius_m, 1.0 - x1), (a0 * uv_radius_m, 1.0 - x1)]
                if hint in ("out", "in"):
                    am = 0.5 * (a0 + a1)
                    out = radial_dir(am) * (1.0 if hint == "out" else -1.0)
                else:
                    out = Vector(hint)
                mesh.face(pts, uvs, out, group)


def box(mesh: VMesh, centre, ex, ey, ez, half, uv_scale=1.0) -> None:
    """Flat-shaded box: `centre`, orthonormal axes ex/ey/ez (Vectors), half sizes (hx, hy, hz)."""
    c = Vector(centre)
    axes = (Vector(ex), Vector(ey), Vector(ez))
    for ai in range(3):
        for sign in (-1.0, 1.0):
            a, b = axes[(ai + 1) % 3], axes[(ai + 2) % 3]
            ha, hb, hn = half[(ai + 1) % 3], half[(ai + 2) % 3], half[ai]
            base = c + axes[ai] * (sign * hn)
            corners = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
            pts = [base + a * (sa * ha) + b * (sb * hb) for sa, sb in corners]
            uvs = [((sa * ha) * uv_scale, 1.0 - (sb * hb) * uv_scale) for sa, sb in corners]
            mesh.face(pts, uvs, axes[ai] * sign)


def tube(mesh: VMesh, p0, p1, radius, sides, group, cap0=True, cap1=True, radius1=None) -> None:
    """Round bar between two points, smooth sides, flat caps; `radius1` (default = radius) tapers the far end."""
    r1 = radius if radius1 is None else radius1
    p0, p1 = Vector(p0), Vector(p1)
    z = (p1 - p0).normalized()
    x = Vector((1.0, 0.0, 0.0)).cross(z)
    if x.length < 1e-6:
        x = Vector((0.0, 1.0, 0.0)).cross(z)
    x.normalize()
    y = z.cross(x)
    length = (p1 - p0).length
    for i in range(sides):
        a0, a1 = 2.0 * math.pi * i / sides, 2.0 * math.pi * (i + 1) / sides
        d0 = x * math.cos(a0) + y * math.sin(a0)
        d1 = x * math.cos(a1) + y * math.sin(a1)
        u0, u1 = a0 * radius, a1 * radius
        mid = (d0 + d1).normalized()
        mesh.face([p0 + d0 * radius, p0 + d1 * radius, p1 + d1 * r1, p1 + d0 * r1],
                  [(u0, 1.0), (u1, 1.0), (u1, 1.0 - length), (u0, 1.0 - length)], mid, group)
        if cap0:
            mesh.face([p0, p0 + d0 * radius, p0 + d1 * radius], [(0, 0), (u0, 0), (u1, 0)], -z)
        if cap1:
            mesh.face([p1, p1 + d0 * r1, p1 + d1 * r1], [(0, 0), (u0, 0), (u1, 0)], z)


def export_selected(path, objects) -> None:
    """glTF of the given objects (and their children) only, Blender axes kept."""
    bpy.ops.object.select_all(action="DESELECT")
    for ob in objects:
        ob.select_set(True)
        for ch in ob.children_recursive:
            ch.select_set(True)
    bpy.ops.export_scene.gltf(filepath=str(path), export_format="GLTF_SEPARATE", export_yup=False, export_cameras=False,
                              export_lights=False, export_extras=False, use_selection=True, export_apply=False)


def report_triangles(name, tris) -> None:
    print(f"TRIANGLES {name} {tris}")
