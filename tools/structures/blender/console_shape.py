"""Console shapes for gen_dragon_cabin.py (fidelity Task 4): pure Python, no Blender import, plain tuples.

`rounded_box` is the housing body: a box with every edge and corner filleted (radius r; the fillets are cube-sphere patches, so
neighbouring faces share their seam vertices and the surface is closed). `drop_underside` / `draft_sides` bend it into the measured
shape (the underside falls toward the back, the side faces lean in). `post` is a ceiling post: a round rod with the cylindrical
joint head at the ceiling and, optionally, the sleeve and funnel foot on the console. Each returns (verts, faces): vertices as
(x, y, z), faces as index tuples wound counter-clockwise seen from outside.
"""
import math

WELD_DIGITS = 9  # vertices closer than 1e-9 m are one vertex


def _weld(quads):
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


def rounded_box(w, h, d, r, segments, cuts=None):
    """Box of size w x h x d (along x, y, z, centred on the origin) with all edges rounded to radius r, `segments` steps per 90 deg fillet.
    cuts = (xs, ys, zs): extra vertex rows across the flat faces at these local coordinates (so a later bend, taper_ends, has vertices there)."""
    half = (w / 2.0, h / 2.0, d / 2.0)
    if r <= 0.0 or r >= min(half):
        raise ValueError(f"rounded_box: radius {r} must lie in (0, {min(half)})")
    inner = tuple(a - r for a in half)
    m = max(1, math.ceil(segments / 2))  # steps per 45 deg: each face carries its half of the fillets up to the bisector

    cuts = cuts or ((), (), ())

    def samples(b, axis):
        left = [(-b, -math.tan(math.radians(45.0 * (m - k) / m))) for k in range(m + 1)]
        right = [(b, math.tan(math.radians(45.0 * k / m))) for k in range(m + 1)]
        mid = [(c, 0.0) for c in sorted(cuts[axis]) if -b + 1e-6 < c < b - 1e-6]
        return left + mid + right

    quads = []
    for k in range(3):
        t1, t2 = (k + 1) % 3, (k + 2) % 3
        l1, l2 = samples(inner[t1], t1), samples(inner[t2], t2)
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
    return _weld(quads)


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


def taper_ends(verts, half_width_at, w):
    """Pull the ends in: every vertex's x scales by half_width_at(y, z) / (w / 2) where that is < 1 (the end cap keeps its fillet, scaled)."""
    out = []
    for x, y, z in verts:
        s = min(1.0, half_width_at(y, z) / (w / 2.0))
        out.append((x * s, y, z))
    return out


def _area2(poly):
    return sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, poly[1:] + poly[:1])) / 2.0


def ccw(poly):
    """The 2D polygon counter-clockwise (positive area)."""
    poly = [tuple(p) for p in poly]
    return poly if _area2(poly) > 0.0 else poly[::-1]


def round_corners(poly, r, segments, only=None):
    """Fillet the corners of a convex 2D polygon with radius r (`segments` steps per 90 deg, at least 1 per corner); `only`: the input indices
    of the corners to round (default all). r <= 0: unchanged. The fillet is limited to the shorter half of the two edges at a corner.
    Counter-clockwise out."""
    flagged = [(tuple(p), only is None or i in only) for i, p in enumerate(poly)]
    if _area2([p for p, _ in flagged]) < 0.0:
        flagged = flagged[::-1]
    poly = [p for p, _ in flagged]
    if r <= 0.0:
        return poly
    out = []
    n = len(poly)
    for i in range(n):
        p0, p, p1 = poly[i - 1], poly[i], poly[(i + 1) % n]
        if not flagged[i][1]:
            out.append(p)
            continue
        a = (p0[0] - p[0], p0[1] - p[1])
        b = (p1[0] - p[0], p1[1] - p[1])
        la, lb = math.hypot(*a), math.hypot(*b)
        a, b = (a[0] / la, a[1] / la), (b[0] / lb, b[1] / lb)
        cosang = max(-1.0, min(1.0, a[0] * b[0] + a[1] * b[1]))
        ang = math.acos(cosang)  # interior angle
        if ang > math.pi - 1e-6:
            out.append(p)
            continue
        t = r / math.tan(ang / 2.0)
        t = min(t, 0.5 * la, 0.5 * lb)
        rr = t * math.tan(ang / 2.0)
        s = (p[0] + a[0] * t, p[1] + a[1] * t)
        e = (p[0] + b[0] * t, p[1] + b[1] * t)
        bis = (a[0] + b[0], a[1] + b[1])
        k = math.hypot(*bis)
        c = (p[0] + bis[0] / k * rr / math.sin(ang / 2.0), p[1] + bis[1] / k * rr / math.sin(ang / 2.0))
        a0, a1 = math.atan2(s[1] - c[1], s[0] - c[0]), math.atan2(e[1] - c[1], e[0] - c[0])
        sweep = (a1 - a0 + math.pi) % (2.0 * math.pi) - math.pi
        steps = max(1, math.ceil(abs(sweep) / (math.pi / 2.0) * segments))
        out += [(c[0] + rr * math.cos(a0 + sweep * j / steps), c[1] + rr * math.sin(a0 + sweep * j / steps)) for j in range(steps + 1)]
    return out


def extrude(poly, x0, x1):
    """Closed prism along x from x0 to x1 over the convex 2D profile `poly` in the (y, z) plane: flat caps, outward winding."""
    poly = ccw(poly)
    n = len(poly)
    quads = []
    for i in range(n):
        (ya, za), (yb, zb) = poly[i], poly[(i + 1) % n]
        quads.append([(x0, ya, za), (x0, yb, zb), (x1, yb, zb), (x1, ya, za)])
    quads.append([(x1, y, z) for y, z in poly])
    quads.append([(x0, y, z) for y, z in poly[::-1]])
    return _weld(quads)


def rounded_rect(cx, cy, hw, hh, r, segments):
    """Rounded rectangle as four corner groups (bottom left, bottom right, top right, top left; counter-clockwise), each a list of
    points along its arc (one point when r <= 0)."""
    corners = ((-1, -1, math.pi), (1, -1, 1.5 * math.pi), (1, 1, 0.0), (-1, 1, 0.5 * math.pi))
    groups = []
    for sx, sy, a0 in corners:
        if r <= 0.0:
            groups.append([(cx + sx * hw, cy + sy * hh)])
            continue
        ox, oy = cx + sx * (hw - r), cy + sy * (hh - r)
        groups.append([(ox + r * math.cos(a0 + 0.5 * math.pi * j / segments), oy + r * math.sin(a0 + 0.5 * math.pi * j / segments))
                       for j in range(segments + 1)])
    return groups


def ring(outer, inner):
    """2D faces (counter-clockwise) filling the region between two corner-group loops (rounded_rect), outer around inner."""
    faces = []
    for k in range(4):
        go, gi = outer[k], inner[k]
        if len(go) == len(gi) and len(go) > 1:
            for j in range(len(go) - 1):
                faces.append([go[j], go[j + 1], gi[j + 1], gi[j]])
        elif len(go) == 1:
            for j in range(len(gi) - 1):
                faces.append([go[0], gi[j + 1], gi[j]])
        else:  # inner sharp, outer arc
            for j in range(len(go) - 1):
                faces.append([go[j], go[j + 1], gi[0]])
        nk = (k + 1) % 4
        faces.append([go[-1], outer[nk][0], inner[nk][0], gi[-1]])
    return [ccw(f) for f in faces if abs(_area2(f)) > 1e-14]


def loop(groups):
    out = []
    for g in groups:
        for p in g:
            if not out or math.dist(out[-1], p) > 1e-12:
                out.append(p)
    return out[:-1] if math.dist(out[0], out[-1]) < 1e-12 else out


def plate_with_holes(x0, x1, y0, y1, holes, margin, segments):
    """2D faces (counter-clockwise) of the rectangle [x0, x1] x [y0, y1] with rounded rectangular holes (cx, cy, hw, hh, r) that share one
    vertical band: a grid of cells whose hole cells (each hole's box grown by `margin`) are rings. Neighbouring cells share their vertices
    (no T-junctions inside; the outer border carries the grid's break points)."""
    holes = sorted(holes)
    xs = [x0] + [v for cx, cy, hw, hh, r in holes for v in (cx - hw - margin, cx + hw + margin)] + [x1]
    ylo = min(cy - hh for cx, cy, hw, hh, r in holes) - margin
    yhi = max(cy + hh for cx, cy, hw, hh, r in holes) + margin
    ys = [y0, ylo, yhi, y1]
    if any(b <= a for a, b in zip(xs, xs[1:])) or any(b <= a for a, b in zip(ys, ys[1:])):
        raise ValueError("plate_with_holes: holes overlap or reach past the plate")
    faces = []
    for i in range(len(xs) - 1):
        for j in range(3):
            hole = holes[(i - 1) // 2] if (i % 2 == 1 and j == 1) else None
            a, b, c, d = (xs[i], ys[j]), (xs[i + 1], ys[j]), (xs[i + 1], ys[j + 1]), (xs[i], ys[j + 1])
            if hole is None:
                faces.append([a, b, c, d])
            else:
                cx, cy, hw, hh, r = hole
                faces += ring([[a], [b], [c], [d]], rounded_rect(cx, cy, hw, hh, r, segments))
    return faces


def wall(loop2d, z0, z1, inward=False):
    """Quads of the wall over a closed 2D loop (counter-clockwise) from z0 to z1: outward (away from the loop's inside) or inward."""
    pts = ccw(loop2d)
    quads = []
    for i in range(len(pts)):
        (xa, ya), (xb, yb) = pts[i], pts[(i + 1) % len(pts)]
        q = [(xa, ya, z0), (xb, yb, z0), (xb, yb, z1), (xa, ya, z1)]  # normal = tangent x z: outward when z1 > z0
        quads.append(q if (z1 > z0) != inward else q[::-1])
    return quads


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
    return _weld(quads)
