"""Console shapes for gen_dragon_cabin.py (fidelity Task 4): pure Python, no Blender import, plain tuples.

`rounded_box` is the housing body: a box with every edge and corner filleted (radius r; the fillets are cube-sphere patches, so
neighbouring faces share their seam vertices and the surface is closed). `drop_underside` / `draft_sides` bend it into the measured
shape (the underside falls toward the back, the side faces lean in). `post` is a ceiling post: a round rod with the cylindrical
joint head at the ceiling and, optionally, the sleeve and funnel foot on the console. Each returns (verts, faces): vertices as
(x, y, z), faces as index tuples wound counter-clockwise seen from outside.
"""
import math

WELD_DIGITS = 9  # vertices closer than 1e-9 m are one vertex


def _weld(points, quads):
    index, verts, faces = {}, [], []
    for q in quads:
        f = []
        for p in q:
            key = tuple(round(c, WELD_DIGITS) for c in p)
            i = index.get(key)
            if i is None:
                i = index[key] = len(verts)
                verts.append(tuple(p))
            if not f or f[-1] != i:
                f.append(i)
        if len(f) > 1 and f[0] == f[-1]:
            f.pop()
        if len(set(f)) >= 3:
            faces.append(tuple(f))
    return verts, faces


def rounded_box(w, h, d, r, segments):
    """Box of size w x h x d (along x, y, z, centred on the origin) with all edges rounded to radius r, `segments` steps per 90 deg fillet."""
    half = (w / 2.0, h / 2.0, d / 2.0)
    if r <= 0.0 or r >= min(half):
        raise ValueError(f"rounded_box: radius {r} must lie in (0, {min(half)})")
    inner = tuple(a - r for a in half)
    m = max(1, math.ceil(segments / 2))  # steps per 45 deg: each face carries its half of the fillets up to the bisector

    def samples(b):
        left = [(-b, -math.tan(math.radians(45.0 * (m - k) / m))) for k in range(m + 1)]
        right = [(b, math.tan(math.radians(45.0 * k / m))) for k in range(m + 1)]
        return left + right

    quads = []
    for k in range(3):
        t1, t2 = (k + 1) % 3, (k + 2) % 3
        l1, l2 = samples(inner[t1]), samples(inner[t2])
        for s in (-1.0, 1.0):
            grid = []
            for c1, g1 in l1:
                row = []
                for c2, g2 in l2:
                    g = [0.0, 0.0, 0.0]
                    g[k], g[t1], g[t2] = s, g1, g2
                    n = math.sqrt(g[0] ** 2 + g[1] ** 2 + g[2] ** 2)
                    p = [0.0, 0.0, 0.0]
                    p[k], p[t1], p[t2] = s * inner[k], c1, c2
                    row.append(tuple(p[i] + r * g[i] / n for i in range(3)))
                grid.append(row)
            for i in range(len(l1) - 1):
                for j in range(len(l2) - 1):
                    q = [grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]]  # normal +e_k (t1 x t2 = k)
                    quads.append(q if s > 0 else q[::-1])
    return _weld([], quads)


def interp(table, x):
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0) if x1 > x0 else y1
    return table[-1][1]


def drop_underside(verts, profile, h, d):
    """Lower the underside of a body made by rounded_box (y up, front face at z = +d / 2): `profile` = [[depth_m, drop_m], ...] with
    depth from the front face; a point at the bottom (y = -h / 2) drops by the full amount, the drop fades linearly to zero at y = 0."""
    out = []
    for x, y, z in verts:
        if y < 0.0:
            y -= interp(profile, d / 2.0 - z) * (-y) / (h / 2.0)
        out.append((x, y, z))
    return out


def draft_sides(verts, draft_deg, w, d):
    """Lean the side faces in toward the back: the half width shrinks by depth * tan(draft) (depth from the front face at z = +d / 2)."""
    t = math.tan(math.radians(draft_deg))
    return [(x * (1.0 - (d / 2.0 - z) * t / (w / 2.0)), y, z) for x, y, z in verts]


def post(base, top, r, head_r, head_len, sides, foot=None, top_plane_normal=None):
    """Ceiling post from `base` (on the console) to `top` (on the ceiling): rod radius r, the cylindrical joint head (head_r, head_len)
    at the top; foot = (sleeve_r, sleeve_len, flare_r, flare_len): a funnel from flare_r at the base to the sleeve over flare_len, then
    the sleeve (a second joint head) over sleeve_len. top_plane_normal: the head is cut parallel to this plane through `top` (the ceiling),
    so the head sits flush on it; default: square to the axis. Closed: flat caps at both ends."""
    axis = [t - b for t, b in zip(top, base)]
    length = math.sqrt(sum(a * a for a in axis))
    ez = [a / length for a in axis]
    ref = (1.0, 0.0, 0.0) if abs(ez[0]) < 0.9 else (0.0, 1.0, 0.0)
    ex = [ref[1] * ez[2] - ref[2] * ez[1], ref[2] * ez[0] - ref[0] * ez[2], ref[0] * ez[1] - ref[1] * ez[0]]
    k = math.sqrt(sum(a * a for a in ex))
    ex = [a / k for a in ex]
    ey = [ez[1] * ex[2] - ez[2] * ex[1], ez[2] * ex[0] - ez[0] * ex[2], ez[0] * ex[1] - ez[1] * ex[0]]
    prof = []  # (distance from the base, radius)
    s0 = 0.0
    if foot is not None:
        sleeve_r, sleeve_len, flare_r, flare_len = foot
        prof += [(0.0, flare_r), (flare_len, sleeve_r), (flare_len + sleeve_len, sleeve_r)]
        s0 = flare_len + sleeve_len
    if head_len + s0 >= length:
        raise ValueError("post: foot and head longer than the post")
    prof += [(s0, r), (length - head_len, r), (length - head_len, head_r), (length, head_r)]

    tilt = None
    if top_plane_normal is not None:  # the head's sections run parallel to the top plane: offset along the axis per point
        pn = top_plane_normal
        en = sum(a * b for a, b in zip(ez, pn))
        if abs(en) < 0.2:
            raise ValueError("post: the axis lies too flat to the top plane")
        tilt = (pn, en)
    head_from = length - head_len

    def at(s, rad, a):
        c, sn = math.cos(a), math.sin(a)
        d = [ex[i] * c + ey[i] * sn for i in range(3)]
        if tilt is not None and s >= head_from - 1e-12:
            s -= rad * sum(x * y for x, y in zip(d, tilt[0])) / tilt[1]
        return tuple(base[i] + ez[i] * s + d[i] * rad for i in range(3))

    angles = [2.0 * math.pi * i / sides for i in range(sides)]
    rings = [[at(s, rad, a) for a in angles] for s, rad in prof]
    quads = []
    for ra, rb in zip(rings, rings[1:]):
        for i in range(sides):
            j = (i + 1) % sides
            quads.append([ra[i], ra[j], rb[j], rb[i]])  # ex, ey, ez right-handed: counter-clockwise from outside
    c0, c1 = tuple(base), tuple(top)
    for i in range(sides):
        j = (i + 1) % sides
        quads.append([c0, rings[0][j], rings[0][i]])
        quads.append([c1, rings[-1][i], rings[-1][j]])
    return _weld([], quads)
