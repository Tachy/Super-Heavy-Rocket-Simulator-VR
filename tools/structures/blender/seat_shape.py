"""Crew Dragon seat shapes (fidelity Task 8): pure geometry (no bpy), in the SEAT frame of the crew figure (docs/subsystems/cockpit.md
"Crew figure"): origin = the seat's hip reference point ([[seat]] hip_m), x = up the seat back, y = the chest normal, z = the crew's left.

Parts (config/cockpits/crew_dragon.toml [seat_shape], measured in cabin_fit/parts/seats.py):
- the BACK: one moulded body lofted up the back from cross sections (back_section): the black cushion's front (`pad`) is flat in the middle
  (the cushion surface BACK_SURFACE_Y, the crew figure's seat envelope) and rises into wings toward the sides (wing_m at |z| = half_width -
  rim); the wing's front edge rolls over a rim (rim_radius_m, still black) into the white outer shell (`shell`: side faces, rounded back corners,
  the slightly curved back). Rows (back_rows) give half width, flat half width and wing height up the back: low lumbar wings, shoulder wings, the
  big headrest wings beside the helmet. The top edge is rounded over top_radius_m.
- the PAN: the same loft along the pan axis (the pan follows the thighs: pan_raise_deg), black cushion on top with low side bolsters, white side
  panels down to pan_depth_m, the front edge rounded over pan_front_radius_m.
- the LEG SUPPORT (`rest`, carbon): one closed U-frame, a tube of rounded-rectangle section (rail_section_m) swept from the pan's front end (knee
  end, under the pan's side panels) along the shins to its foot end frame_end_m below the soles and back: two rails whose outer edges lie
  frame_width_m apart (knee end, foot end), joined under the foot plate by the closing bar over rounded corners (frame_corner_radius_m); the foot
  plate under the soles between the rails.
- the ARMRESTS (`pad`, black padded blocks): short rounded blocks beside the pan (armrest_centre_m, armrest_size_m).

Faces: dicts {"pts": [(x, y, z)...] seat frame, "uv": [(u, v)...] atlas metres (u right, v down the image), "part": "pad" | "shell" | "rest",
"group": smooth group (hashable) or None, "out": outward hint (x, y, z)}. Atlas layout: SEAT_CHARTS (metres on a SEAT_ATLAS_M square).
"""
from __future__ import annotations

import math

BACK_SURFACE_Y = -0.01  # [D] the cushion's front at the seat's middle: 0.01 behind the hip reference point (the crew figure's seat envelope)
PAN_SURFACE_U = -0.01  # [D] the pan cushion's top: 0.01 under the hip reference point (crew figure envelope)
SEAT_ATLAS_M = 3.0  # [E] the seat atlas covers 3.0 x 3.0 m (2048 px: 1.46 mm per texel; fabric grain / carbon weave come from the detail maps)
# chart origins (u0, v0) in atlas metres; a chart's local (s, t) adds to them (s across the part, t along its loft axis)
SEAT_CHARTS = {
    "back_pad": (0.60, 1.10),   # s = arc from the front's middle (-0.55..0.55), t = -x (top of the back at the top of the image)
    "back_shell": (1.95, 1.10),  # s = arc from the back's middle (-0.62..0.62), t = -x
    "pan_pad": (0.50, 2.00),     # s = arc from the top's middle, t = -a (along the pan, -0.1..0.48)
    "pan_shell": (1.65, 2.00),
    "rest": (0.40, 2.55),        # carbon / black parts and caps: the uniform black band v > 2.2 (metric UVs for the detail weave, charts may overlap)
}
LOOP_FRONT, LOOP_RIM, LOOP_SIDE, LOOP_CORNER, LOOP_BACK = 24, 4, 2, 3, 12  # [E] samples of a cross section (front across, rim arc, ...)
ROW_STEP_M = 0.03  # [E] loft rows along the back / pan (3 cm: the wing rise reads smooth at 0.5 m)
END_STEPS = 3  # [E] rows of a rounded end (top of the back, front of the pan)
RAIL_CORNER_STEPS = 2  # [E] steps per 90 deg corner of the rail section
FRAME_BEND_STEPS = 6  # [E] path steps per 90 deg bend of the leg-support frame (its foot-end corners)
FRAME_SEGMENT_M = 0.1  # [E] the frame's straight rails are split into segments of at most this length (keeps each quad's UV inside the rest band)
TUBE_V_FOLD_M = 0.3  # [E] a swept tube's along-path UV restarts every 0.3 m (15 carbon detail tiles of 0.02 m: seamless) to stay on the rest band


def smooth(t):
    t = min(1.0, max(0.0, t))
    return t * t * (3.0 - 2.0 * t)


def interp_rows(rows, x):
    """Linear interpolation of the columns 1.. of `rows` ([[x, c1, c2, ...], ...], ascending x) at x (clamped)."""
    if x <= rows[0][0]:
        return list(rows[0][1:])
    for a, b in zip(rows, rows[1:]):
        if x <= b[0]:
            f = (x - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 1.0
            return [ca + (cb - ca) * f for ca, cb in zip(a[1:], b[1:])]
    return list(rows[-1][1:])


# ---------- cross sections ----------

def section(half_width, flat_half, wing, rim, depth, corner, bulge=0.0, surface=0.0):
    """Closed cross section [(y, z, tag)] (y forward = the cushion side, z lateral), counter-clockwise seen from +x, starting at the front's
    middle: the cushion front y = surface + wing * smooth((|z| - flat_half) / (half_width - rim - flat_half)), the rim (radius rim) round onto the
    side face z = +-half_width, down to the back corner (radius corner), the back y = surface - depth + bulge (z / half_width)^2.
    Tags per sample: "pad" (front and rim), "shell" (side, corner, back)."""
    rim = max(1e-4, min(rim, wing, half_width - flat_half - 1e-4))
    zf = half_width - rim
    span = max(1e-6, zf - flat_half)

    def front_y(z):
        return surface + wing * smooth((abs(z) - flat_half) / span)

    y_back_side = surface - depth + bulge
    half = []  # from the front's middle to the back's middle on the +z side
    for k in range(LOOP_FRONT // 2 + 1):
        z = zf * k / (LOOP_FRONT // 2)
        half.append((front_y(z), z, "pad"))
    cy, cz = surface + wing - rim, zf
    for k in range(1, LOOP_RIM + 1):
        a = math.radians(90.0 * k / LOOP_RIM)
        half.append((cy + rim * math.cos(a), cz + rim * math.sin(a), "pad"))
    top_side, low_side = surface + wing - rim, y_back_side + corner
    for k in range(1, LOOP_SIDE + 1):
        half.append((top_side + (low_side - top_side) * k / (LOOP_SIDE + 1), half_width, "shell"))
    for k in range(LOOP_CORNER + 1):
        a = math.radians(90.0 * k / LOOP_CORNER)
        half.append((low_side - corner * math.sin(a), half_width - corner + corner * math.cos(a), "shell"))
    zb = half_width - corner
    for k in range(1, LOOP_BACK // 2 + 1):
        z = zb * (1.0 - k / (LOOP_BACK // 2))
        half.append((surface - depth + bulge * (z / half_width) ** 2, z, "shell"))
    # mirror: front middle -> +z side -> back middle -> -z side -> (closing to the start)
    loop = half + [(y, -z, t) for y, z, t in reversed(half[1:-1])]
    return loop


def back_section(shape, x):
    hw, flat, wing = interp_rows(shape["back_rows"], x)
    return section(hw, flat, wing, shape["rim_radius_m"], shape["back_depth_m"], shape["corner_radius_m"], shape["back_bulge_m"], BACK_SURFACE_Y)


def pan_section(shape, half_width, a):
    return section(half_width, shape["pan_flat_half_m"], shape["pan_wing_m"], shape["rim_radius_m"], shape["pan_depth_m"],
                   shape["corner_radius_m"], 0.0, PAN_SURFACE_U)


def back_front_y(shape, x, z):
    """The back cushion's front surface y at (x, z) (inside the half width), None outside the back."""
    hw, flat, wing = interp_rows(shape["back_rows"], x)
    rim = max(1e-4, min(shape["rim_radius_m"], wing, hw - flat - 1e-4))
    if abs(z) > hw or x < shape["back_rows"][0][0] or x > shape["back_rows"][-1][0]:
        return None
    if abs(z) > hw - rim:
        dz = abs(z) - (hw - rim)
        return BACK_SURFACE_Y + wing - rim + math.sqrt(max(0.0, rim * rim - dz * dz))
    return BACK_SURFACE_Y + wing * smooth((abs(z) - flat) / max(1e-6, hw - rim - flat))


def back_depth_at(shape):
    return BACK_SURFACE_Y - shape["back_depth_m"]


# ---------- loft ----------

def _arc(loop):
    """Arc length along a closed loop from its first sample (the front's middle), per sample; and the total."""
    s = [0.0]
    for a, b in zip(loop, loop[1:]):
        s.append(s[-1] + math.hypot(a[0] - b[0], a[1] - b[1]))
    total = s[-1] + math.hypot(loop[-1][0] - loop[0][0], loop[-1][1] - loop[0][1])
    return s, total


def _rounded_rows(sections, xs, radius, steps, mid_of):
    """End rows rounding a loft's last section over `radius`: each sample moves toward its section's mid surface (mid_of(z)) by at most
    radius (1 - cos a) and the row advances radius sin a."""
    last, x_end = sections[-1], xs[-1]
    rows = []
    for k in range(1, steps + 1):
        a = math.radians(90.0 * k / (steps + 1))
        pull = radius * (1.0 - math.cos(a))
        row = []
        for y, z, t in last:
            m = mid_of(z)
            d = y - m
            row.append((y - math.copysign(min(abs(d), pull), d), z * (1.0 - pull / max(0.05, abs(z) + radius)) if abs(z) > 1e-9 else z, t))
        rows.append((x_end + radius * math.sin(a), row))
    return rows


def loft(xs, sections, to_seat, chart_pad, chart_shell, end_radius, name, cap_start=True):
    """Faces of a lofted body `name` (its smooth groups: (name, "pad") / (name, "shell")): `sections[i]` at axial coordinate xs[i] (each the
    same sample count), to_seat(x, y, z) -> seat frame point.
    Quads between rows take the tag of their segment's first sample ("pad" / "shell"); UV s = arc length from the pad's middle (pad faces)
    or from the back's middle (shell faces), t = -x (chart v = v0 - x). The far end is rounded over end_radius and capped; the start is capped (both caps
    black, on the rest chart)."""
    n = len(sections[0])
    mid = n // 2  # the back's middle (sections are symmetric: front middle at 0, back middle at n // 2)

    def mid_of_factory(sec):
        fronts = [(abs(z), y) for y, z, t in sec[:LOOP_FRONT // 2 + 1]]
        backs = [(abs(z), y) for y, z, t in sec[mid - LOOP_BACK // 2: mid + 1]]

        def mid_of(z):
            def at(tab, z0):
                tab = sorted(tab)
                if z0 <= tab[0][0]:
                    return tab[0][1]
                for (z1, y1), (z2, y2) in zip(tab, tab[1:]):
                    if z0 <= z2:
                        return y1 + (y2 - y1) * (z0 - z1) / max(1e-9, z2 - z1)
                return tab[-1][1]
            return 0.5 * (at(fronts, abs(z)) + at(backs, abs(z)))
        return mid_of

    rows = list(zip(xs, sections)) + _rounded_rows(sections, xs, end_radius, END_STEPS, mid_of_factory(sections[-1]))
    faces = []
    for (x0, s0), (x1, s1) in zip(rows, rows[1:]):
        a0, _ = _arc(s0)
        a1, _ = _arc(s1)
        b0 = [a - a0[mid] for a in a0]
        b1 = [a - a1[mid] for a in a1]
        for i in range(n):
            j = (i + 1) % n
            tag = "pad" if s0[i][2] == "pad" and s0[j][2] == "pad" else "shell"  # a segment from the rim to the side face is side face
            pts = [to_seat(x0, s0[i][0], s0[i][1]), to_seat(x0, s0[j][0], s0[j][1]), to_seat(x1, s1[j][0], s1[j][1]), to_seat(x1, s1[i][0], s1[i][1])]
            if tag == "pad":
                def sarc(a, k, tot):
                    v = a[k] if k <= n // 2 else a[k] - tot  # signed arc from the front's middle
                    return v
                _, t0 = _arc(s0)
                _, t1 = _arc(s1)
                u = [sarc(a0, i, t0), sarc(a0, j, t0), sarc(a1, j, t1), sarc(a1, i, t1)]
                c = chart_pad
            else:
                u = [b0[i], b0[j], b1[j], b1[i]]
                c = chart_shell
            uv = [(c[0] + uu, c[1] - xx) for uu, xx in zip(u, (x0, x0, x1, x1))]
            sx = 0.5 * (s0[i][1] + s0[j][1])
            sy = 0.5 * (s0[i][0] + s0[j][0])
            out_local = _section_normal(s0, i, j)
            faces.append({"pts": pts, "uv": uv, "part": tag, "group": (name, tag), "out": _out(to_seat, x0, out_local, sy, sx)})
    # caps: the rounded end's last row and the start row (fans to the centroid), on the rest chart (uniform black)
    for (xc, sec), sign in ((rows[-1], 1.0), (rows[0], -1.0)):
        if sign < 0 and not cap_start:
            continue
        cy = sum(p[0] for p in sec) / n
        cz = sum(p[1] for p in sec) / n
        centre = to_seat(xc, cy, cz)
        for i in range(n):
            j = (i + 1) % n
            pts = [centre, to_seat(xc, sec[i][0], sec[i][1]), to_seat(xc, sec[j][0], sec[j][1])]
            r0 = SEAT_CHARTS["rest"]
            uv = [(r0[0] + cz, r0[1] + cy), (r0[0] + sec[i][1], r0[1] + sec[i][0]), (r0[0] + sec[j][1], r0[1] + sec[j][0])]
            out = _sub(to_seat(xc + sign, cy, cz), to_seat(xc, cy, cz))
            faces.append({"pts": pts, "uv": uv, "part": "pad", "group": None, "out": out})  # caps: black (the top of the back, the pan front; the start caps are hidden)
    return faces


def _section_normal(sec, i, j):
    """Outward normal (dy, dz) of the section segment i -> j (counter-clockwise loop seen from +x: outward = rotate the tangent by -90 deg)."""
    dy, dz = sec[j][0] - sec[i][0], sec[j][1] - sec[i][1]
    return (dz, -dy)


def _out(to_seat, x, n_local, y, z):
    p0 = to_seat(x, y, z)
    p1 = to_seat(x, y + n_local[0], z + n_local[1])
    return _sub(p1, p0)


def _sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def _add(a, b, s=1.0):
    return tuple(a[i] + b[i] * s for i in range(3))


def _norm(a):
    length = math.sqrt(sum(c * c for c in a))
    return tuple(c / length for c in a)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


# ---------- parts ----------

def back_faces(shape):
    rows = shape["back_rows"]
    x0, x1 = rows[0][0], rows[-1][0] - shape["top_radius_m"]
    k = max(2, int(math.ceil((x1 - x0) / ROW_STEP_M)))
    xs = [x0 + (x1 - x0) * i / k for i in range(k + 1)]
    secs = [back_section(shape, x) for x in xs]
    return loft(xs, secs, lambda x, y, z: (x, y, z), SEAT_CHARTS["back_pad"], SEAT_CHARTS["back_shell"], shape["top_radius_m"], "back")


def pan_axes(pan_raise_deg):
    """(along, up) of the pan in the seat frame (x, y): the pan follows the thighs (crew_figure.toml thigh_raise_deg)."""
    r = math.radians(pan_raise_deg)
    return (math.sin(r), math.cos(r), 0.0), (math.cos(r), -math.sin(r), 0.0)


def pan_faces(shape, half_width, pan_raise_deg):
    pa, pu = pan_axes(pan_raise_deg)
    a0, a1 = shape["pan_start_m"], shape["pan_length_m"] - shape["pan_front_radius_m"]
    k = max(2, int(math.ceil((a1 - a0) / ROW_STEP_M)))
    xs = [a0 + (a1 - a0) * i / k for i in range(k + 1)]
    secs = [pan_section(shape, half_width, a) for a in xs]

    def to_seat(a, u, z):
        return (pa[0] * a + pu[0] * u, pa[1] * a + pu[1] * u, z)
    return loft(xs, secs, to_seat, SEAT_CHARTS["pan_pad"], SEAT_CHARTS["pan_shell"], shape["pan_front_radius_m"], "pan")


def _ring(w, d, r, steps):
    """Rounded-rectangle section samples (s, u): w along s, d along u, corner radius r."""
    r = min(r, 0.49 * min(w, d))
    ring = []
    for cs, cu in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        a0 = math.degrees(math.atan2(cu, cs)) - 45.0
        for k in range(steps + 1):
            a = math.radians(a0 + 90.0 * k / steps)
            ring.append((cs * (w / 2 - r) + r * math.cos(a), cu * (d / 2 - r) + r * math.sin(a)))
    return ring


def swept_tube(path, sides, w, d, r, name, part="rest", steps=RAIL_CORNER_STEPS):
    """Faces of a tube of rounded-rectangle section (w along sides[i], d along tangent x side) swept along the polyline `path` (the section at
    each path point is perpendicular to the mean of its neighbouring segments), both ends capped; smooth group (name,), chart `rest` (metric UVs:
    u = arc around the section, v = arc along the path)."""
    ring = _ring(w, d, r, steps)
    n = len(path)
    frames = []
    for i in range(n):
        a, b = path[max(0, i - 1)], path[min(n - 1, i + 1)]
        t = _norm(_sub(b, a))
        side = _norm(_sub(sides[i], tuple(c * _dot(sides[i], t) for c in t)))
        frames.append((t, side, _cross(t, side)))
    loops = [[_add(_add(path[i], frames[i][1], s), frames[i][2], u) for s, u in ring] for i in range(n)]
    r0 = SEAT_CHARTS["rest"]
    s_ring = [0.0]
    for a, b in zip(ring, ring[1:] + ring[:1]):
        s_ring.append(s_ring[-1] + math.hypot(a[0] - b[0], a[1] - b[1]))
    v_path = [0.0]
    for a, b in zip(path, path[1:]):
        v_path.append(v_path[-1] + math.sqrt(sum(c * c for c in _sub(b, a))))
    faces = []
    m = len(ring)
    for i in range(n - 1):
        v0 = math.fmod(v_path[i], TUBE_V_FOLD_M)  # folded per segment (each quad keeps its own continuous v)
        v1 = v0 + v_path[i + 1] - v_path[i]
        for k in range(m):
            j = (k + 1) % m
            mid = _norm(_add(_add((0.0, 0.0, 0.0), frames[i][1], ring[k][0] + ring[j][0]), frames[i][2], ring[k][1] + ring[j][1]))
            faces.append({"pts": [loops[i][k], loops[i][j], loops[i + 1][j], loops[i + 1][k]],
                          "uv": [(r0[0] + s_ring[k], r0[1] + v0), (r0[0] + s_ring[k + 1], r0[1] + v0),
                                 (r0[0] + s_ring[k + 1], r0[1] + v1), (r0[0] + s_ring[k], r0[1] + v1)],
                          "part": part, "group": (name,), "out": mid})
    for i, sign in ((0, -1.0), (n - 1, 1.0)):
        faces.append({"pts": list(loops[i]), "uv": [(r0[0] + s, r0[1] + u) for s, u in ring], "part": part, "group": None,
                      "out": tuple(c * sign for c in frames[i][0])})
    return faces


def rounded_rect_bar(p0, p1, lateral, w, d, r, name, part="rest", steps=RAIL_CORNER_STEPS):
    """Faces of a straight bar of rounded-rectangle section (w along `lateral`, d across) from p0 to p1, capped (swept_tube)."""
    return swept_tube([p0, p1], [lateral, lateral], w, d, r, name, part, steps)


def rounded_slab(centre, ex, ey, ez, half, r, name, steps=RAIL_CORNER_STEPS):
    """A slab (half sizes along ex, ey (thickness), ez) whose four edges around ey are rounded (radius r in the ex-ez plane), the faces along
    ey flat: the foot plate. Rest chart."""
    hx, hy, hz = half
    r = min(r, 0.49 * min(hx, hz) * 2)
    ring = []
    for cs, cz in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        a0 = math.degrees(math.atan2(cz, cs)) - 45.0
        for k in range(steps + 1):
            a = math.radians(a0 + 90.0 * k / steps)
            ring.append((cs * (hx - r) + r * math.cos(a), cz * (hz - r) + r * math.sin(a)))
    r0 = SEAT_CHARTS["rest"]
    faces = []
    top = [_add(_add(_add(centre, ey, hy), ex, a), ez, b) for a, b in ring]
    bot = [_add(_add(_add(centre, ey, -hy), ex, a), ez, b) for a, b in ring]
    uv = [(r0[0] + a, r0[1] + b) for a, b in ring]
    faces.append({"pts": top, "uv": uv, "part": "rest", "group": None, "out": ey})
    faces.append({"pts": bot, "uv": uv, "part": "rest", "group": None, "out": tuple(-c for c in ey)})
    s = [0.0]
    for a, b in zip(ring, ring[1:] + ring[:1]):
        s.append(s[-1] + math.hypot(a[0] - b[0], a[1] - b[1]))
    for i in range(len(ring)):
        j = (i + 1) % len(ring)
        mid = _norm(_add(_add((0.0, 0.0, 0.0), ex, ring[i][0] + ring[j][0]), ez, ring[i][1] + ring[j][1]))
        faces.append({"pts": [bot[i], bot[j], top[j], top[i]], "uv": [(r0[0] + s[i], r0[1]), (r0[0] + s[i + 1], r0[1]), (r0[0] + s[i + 1], r0[1] + 2 * hy),
                                                                   (r0[0] + s[i], r0[1] + 2 * hy)], "part": "rest", "group": (name,), "out": mid})
    return faces


def _fillet(p_in, corner, p_out, radius, steps):
    """Points of a circular arc rounding the polyline corner p_in -> corner -> p_out over `radius` (both legs straight up to the arc)."""
    d1 = _norm(_sub(corner, p_in))
    d2 = _norm(_sub(p_out, corner))
    cos_t = max(-1.0, min(1.0, -_dot(d1, d2)))
    theta = math.acos(cos_t)  # interior angle between the legs
    tdist = radius / math.tan(0.5 * theta)
    a = _add(corner, d1, -tdist)
    b = _add(corner, d2, tdist)
    bis = _norm(_add(tuple(-c for c in d1), d2))
    centre = _add(corner, bis, radius / math.sin(0.5 * theta))
    va, vb = _sub(a, centre), _sub(b, centre)
    sweep = math.pi - theta
    k = max(2, int(math.ceil(steps * sweep / (0.5 * math.pi))))
    pts = []
    for i in range(k + 1):  # slerp va -> vb
        f = i / k
        w0 = math.sin((1.0 - f) * sweep) / math.sin(sweep)
        w1 = math.sin(f * sweep) / math.sin(sweep)
        pts.append(_add(centre, _add(tuple(c * w0 for c in va), vb, w1)))
    return pts


def _subdivide(path, step):
    """The polyline with every segment longer than `step` split evenly."""
    out = [path[0]]
    for a, b in zip(path, path[1:]):
        k = max(1, int(math.ceil(math.sqrt(sum(c * c for c in _sub(b, a))) / step)))
        out += [_add(a, _sub(b, a), i / k) for i in range(1, k + 1)]
    return out


def leg_support_path(shape, knee_end, plate_centre, plate_normal, rail_z_centre):
    """The leg-support frame's centre line in the seat frame: [points], [in-plane lateral per point], the frame plane's normal. A closed U:
    the -z rail from the knee end (knee_end = the pan's front end, mid-way down its side panels; rail outer edges frame_width_m[0] apart) along
    the shin to the foot end, the closing bar under the foot plate (its outer edge frame_end_m below the plate's top face along -plate_normal,
    at the plate centre along the foot; rail outer edges frame_width_m[1] apart), the +z rail back to the knee end; the two foot-end corners
    rounded over frame_corner_radius_m (outer edge)."""
    w, _ = shape["rail_section_m"]
    knee_w, foot_w = shape["frame_width_m"]
    foot = _add(plate_centre, plate_normal, -(shape["frame_end_m"] - 0.5 * w))
    kl = (knee_end[0], knee_end[1], rail_z_centre - (0.5 * knee_w - 0.5 * w))
    kr = (knee_end[0], knee_end[1], rail_z_centre + (0.5 * knee_w - 0.5 * w))
    cl_ = (foot[0], foot[1], rail_z_centre - (0.5 * foot_w - 0.5 * w))
    cr_ = (foot[0], foot[1], rail_z_centre + (0.5 * foot_w - 0.5 * w))
    rc = max(0.5 * w + 1e-3, shape["frame_corner_radius_m"] - 0.5 * w)  # the centre line's radius
    path = _subdivide([kl] + _fillet(kl, cl_, cr_, rc, FRAME_BEND_STEPS) + _fillet(cl_, cr_, kr, rc, FRAME_BEND_STEPS) + [kr], FRAME_SEGMENT_M)
    plane_n = _norm(_cross(_sub(cr_, cl_), _sub(kl, cl_)))
    sides = []
    for i in range(len(path)):
        a, b = path[max(0, i - 1)], path[min(len(path) - 1, i + 1)]
        sides.append(_norm(_cross(plane_n, _norm(_sub(b, a)))))
    return path, sides, plane_n


def leg_support_parts(shape, knee_end, plate_centre, plate_normal, plate_along, rail_z_centre):
    """The carbon leg support in the seat frame: ([segments of the frame as bars (p0, p1, lateral, w, d)], slab (centre, ex, ey, ez, half)).
    The frame (leg_support_path) and the foot plate (plate_m: length along the foot, thickness) between the rails at the foot end, its top face in
    the plane through plate_centre (normal plate_normal = up the shin)."""
    w, d = shape["rail_section_m"]
    _, foot_w = shape["frame_width_m"]
    plate_len, plate_t = shape["plate_m"]
    path, sides, _ = leg_support_path(shape, knee_end, plate_centre, plate_normal, rail_z_centre)
    bars = [(a, b, sides[i], w, d) for i, (a, b) in enumerate(zip(path, path[1:]))]
    pc = _add(plate_centre, plate_normal, -0.5 * plate_t)
    slab = ((pc[0], pc[1], rail_z_centre), plate_along, plate_normal, (0.0, 0.0, 1.0), (0.5 * plate_len, 0.5 * plate_t, 0.5 * foot_w - w - 0.002))
    return bars, slab


def armrest_bars(shape, half_width):
    """The two armrests beside the pan: [(p0, p1, lateral, wide, thick)] short blocks along the chest normal (armrest_centre_m: up the back,
    along the chest normal, lateral centre beyond the seat's half width; armrest_size_m: length along the chest normal, thickness up the back,
    width)."""
    up, along, out = shape["armrest_centre_m"]
    length, thick, wide = shape["armrest_size_m"]
    bars = []
    for side in (-1.0, 1.0):
        c = (up, along, side * (half_width + out))
        bars.append(((c[0], c[1] - 0.5 * length, c[2]), (c[0], c[1] + 0.5 * length, c[2]), (0.0, 0.0, 1.0), wide, thick))
    return bars


def _knee_end(shape, pan_raise_deg):
    pa, pu = pan_axes(pan_raise_deg)
    return (pa[0] * shape["pan_length_m"] - pu[0] * shape["pan_depth_m"] * 0.5, pa[1] * shape["pan_length_m"] - pu[1] * shape["pan_depth_m"] * 0.5, 0.0)


def _plate_frame(seat, to_seat_frame):
    plate_c = to_seat_frame(seat["foot_rest_m"])
    o = to_seat_frame((0.0, 0.0, 0.0))
    plate_n = _norm(_sub(to_seat_frame(seat["foot_rest_normal"]), o))
    plate_f = _norm(_cross((0.0, 0.0, 1.0), plate_n))  # along the foot (toward the toes)
    return plate_c, plate_n, plate_f


def seat_parts(shape, seat, pan_raise_deg, to_seat_frame):
    """(bars, slab) of a seat's rigid parts in the seat frame: the leg-support frame's segments and the two armrests (last), the foot plate."""
    plate_c, plate_n, plate_f = _plate_frame(seat, to_seat_frame)
    bars, slab = leg_support_parts(shape, _knee_end(shape, pan_raise_deg), plate_c, plate_n, plate_f, 0.0)
    return bars + armrest_bars(shape, 0.5 * seat["width_m"]), slab


def bar_inside_depth(bar, p):
    """How far p lies inside a bar (p0, p1, lateral, w, d) (its box section; 0 outside)."""
    p0, p1, lateral, w, d = bar
    t = _norm(_sub(p1, p0))
    side = _norm(_sub(lateral, tuple(c * _dot(lateral, t) for c in t)))
    up = _cross(t, side)
    q = _sub(p, p0)
    a, s, u = _dot(q, t), _dot(q, side), _dot(q, up)
    length = math.sqrt(sum(c * c for c in _sub(p1, p0)))
    m = min(a, length - a, 0.5 * w - abs(s), 0.5 * d - abs(u))
    return max(0.0, m)


def slab_inside_depth(slab, p):
    centre, ex, ey, ez, half = slab
    q = _sub(p, centre)
    m = min(half[0] - abs(_dot(q, ex)), half[1] - abs(_dot(q, ey)), half[2] - abs(_dot(q, ez)))
    return max(0.0, m)


def seat_faces(shape, seat, pan_raise_deg, to_seat_frame):
    """All faces of one seat in the seat frame. `seat` = the [[seat]] row; to_seat_frame(cabin point) -> seat frame (for the foot plate)."""
    hw = 0.5 * seat["width_m"]
    faces = back_faces(shape) + pan_faces(shape, hw, pan_raise_deg)
    plate_c, plate_n, plate_f = _plate_frame(seat, to_seat_frame)
    w, d = shape["rail_section_m"]
    path, sides, _ = leg_support_path(shape, _knee_end(shape, pan_raise_deg), plate_c, plate_n, 0.0)
    faces += swept_tube(path, sides, w, d, shape["rail_radius_m"], "frame")
    _, slab = leg_support_parts(shape, _knee_end(shape, pan_raise_deg), plate_c, plate_n, plate_f, 0.0)
    faces += rounded_slab(*slab, shape["plate_radius_m"], "plate")
    for k, (p0, p1, lateral, wide, thick) in enumerate(armrest_bars(shape, hw)):
        faces += rounded_rect_bar(p0, p1, lateral, wide, thick, shape["armrest_radius_m"], f"armrest{k}", part="pad")
    return faces


def inside_depth(shape, seat_width_m, pan_raise_deg, p):
    """How far (m) a seat-frame point p lies inside the back's or the pan's moulded body (behind the cushion surface), 0 when in front of it /
    outside the body: the figure-in-seat test (no interpenetration > 1 cm)."""
    x, y, z = p
    depth = 0.0
    rows = shape["back_rows"]
    if rows[0][0] <= x <= rows[-1][0]:
        f = back_front_y(shape, x, z)
        if f is not None and back_depth_at(shape) <= y < f:
            depth = max(depth, f - y)
    pa, pu = pan_axes(pan_raise_deg)
    a = p[0] * pa[0] + p[1] * pa[1]
    u = p[0] * pu[0] + p[1] * pu[1]
    hw = 0.5 * seat_width_m
    if shape["pan_start_m"] <= a <= shape["pan_length_m"] and abs(z) <= hw:
        rim = max(1e-4, min(shape["rim_radius_m"], shape["pan_wing_m"], hw - shape["pan_flat_half_m"] - 1e-4))  # as `section` clamps it
        top = PAN_SURFACE_U + shape["pan_wing_m"] * smooth((abs(z) - shape["pan_flat_half_m"]) / max(1e-6, hw - rim - shape["pan_flat_half_m"]))
        if PAN_SURFACE_U - shape["pan_depth_m"] <= u < top:
            depth = max(depth, top - u)
    return depth
