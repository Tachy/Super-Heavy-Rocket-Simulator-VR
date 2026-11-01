"""Faces of one tower / arm member in its local frame (+Z along the member), without Blender.

Every face is wound counter-clockwise seen from outside, by construction: the vertices of a face are not shared with its
neighbours, so nothing later can derive the orientation from connectivity.
"""
from __future__ import annotations

import math

TUBE_SIDES = 16  # [E] 16-gon reads round at the 0.8-1.1 m arm tubes from close; tubes are only the few arm members
TUBE_CHAMFER_FRAC = 0.2  # [E] end chamfer as a fraction of the tube radius (bevel, catches a highlight)


def _prism(outline, length):
    """Extruded polygon (outline counter-clockwise seen from +Z): list of (verts, uvs, smooth) faces. Flat sides,
    polygon end caps."""
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
    back = outline[::-1]
    faces.append(([(p[0], p[1], 0.0) for p in back], [(p[0], p[1]) for p in back], False))
    faces.append(([(p[0], p[1], length) for p in outline], [(p[0], p[1]) for p in outline], False))
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
            verts = [inner[i], inner[j], outer[j], outer[i]]
            uvs = [(u(i), zc), (u(i + 1), zc), (u(i + 1), zs), (u(i), zs)]
            faces.append((verts, uvs, False) if start else (verts[::-1], uvs[::-1], False))
        cap = inner[::-1] if start else inner
        faces.append((cap, [(p[0], p[1]) for p in cap], False))
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
