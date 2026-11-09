"""Tape measures on the crew body hull (figure_body.surface, build pose): girths, breadths and depths of plane sections, the way the
ANSUR II measures are taken (a horizontal section through the torso, a section across a limb). Pure numpy; used by the tests
(test_crew_figure.py: the body within 1 cm of the ANSUR medians) and for the report."""
import numpy as np

import figure_body as fb


def section(h, points, part, origin, normal):
    """Segments (k, 2, 3) where the plane (origin, normal) cuts the part's triangles."""
    tris = h.triangles({part})
    p = np.asarray(points)
    n = np.asarray(normal, dtype=float) / np.linalg.norm(normal)
    d = (p - np.asarray(origin)) @ n
    segs = []
    for t in tris:
        dd = d[t]
        if dd.min() > 0.0 or dd.max() < 0.0:
            continue
        cut = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            if (dd[a] > 0.0) != (dd[b] > 0.0):
                f = dd[a] / (dd[a] - dd[b])
                cut.append(p[t[a]] + (p[t[b]] - p[t[a]]) * f)
        if len(cut) == 2:
            segs.append(cut)
    return np.array(segs)


def girth(segs):
    """Perimeter of the convex hull of the section (a tape spans hollows, as in an anthropometric girth)."""
    pts = segs.reshape(-1, 3)
    c = pts.mean(axis=0)
    u, s, vt = np.linalg.svd(pts - c)
    q = (pts - c) @ vt[:2].T
    hull = _hull2(q)
    return float(np.sum(np.linalg.norm(np.roll(hull, -1, axis=0) - hull, axis=1)))


def _hull2(q):
    q = q[np.lexsort((q[:, 1], q[:, 0]))]

    def half(points):
        out = []
        for p in points:
            while len(out) >= 2 and _cross2(out[-1] - out[-2], p - out[-2]) <= 0.0:
                out.pop()
            out.append(p)
        return out
    lower, upper = half(q), half(q[::-1])
    return np.array(lower[:-1] + upper[:-1])


def torso(h, points, cfg, s):
    """(girth, breadth z, depth y) of the torso section s above the hip joint."""
    x = cfg["figure"]["hip_joint_up_m"] + s
    segs = section(h, points, "torso", (x, 0.0, 0.0), (1.0, 0.0, 0.0))
    pts = segs.reshape(-1, 3)
    return girth(segs), float(np.ptp(pts[:, 2])), float(np.ptp(pts[:, 1]))


def limb(cfg, h, points, part, t):
    """Girth of the limb section at t from its proximal joint, across the build-pose limb axis."""
    _, built = fb.build_pose(cfg)
    side = part[-1]
    bone = ("upper_arm_" if part.startswith("arm") else "thigh_") + side
    head, tail = built[bone]
    axis = (tail - head) / np.linalg.norm(tail - head)
    return girth(section(h, points, part, head + axis * t, axis))


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]
