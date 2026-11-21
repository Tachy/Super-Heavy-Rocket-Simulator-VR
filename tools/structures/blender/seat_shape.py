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
- the LEG SUPPORT (`rest`, carbon): two rails of rounded-rectangle section (rail_section_m) from the pan's front end to the foot plate, their outer
  edges frame_width_m apart (knee end, foot end), and the foot plate under the soles between them (the frame's bottom bar).
- the ARMRESTS (`rest`): rounded bars beside the pan.

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
ARMREST_M = ((0.20, 0.20), (0.40, 0.05, 0.06), 0.012)  # [E] armrest centre (along the chest normal, up the back), size (along, thick, wide), edge radius; was the round-3 box (0.2 forward, 0.2 up, 0.4 x 0.05 x 0.06)


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


def loft(xs, sections, to_seat, chart_pad, chart_shell, end_radius, cap_start=True):
    """Faces of a lofted body: `sections[i]` at axial coordinate xs[i] (each the same sample count), to_seat(x, y, z) -> seat frame point.
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
            faces.append({"pts": pts, "uv": uv, "part": tag, "group": (id(sections), tag), "out": _out(to_seat, x0, out_local, sy, sx)})
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
    return loft(xs, secs, lambda x, y, z: (x, y, z), SEAT_CHARTS["back_pad"], SEAT_CHARTS["back_shell"], shape["top_radius_m"])


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
    return loft(xs, secs, to_seat, SEAT_CHARTS["pan_pad"], SEAT_CHARTS["pan_shell"], shape["pan_front_radius_m"])


def rounded_rect_bar(p0, p1, lateral, w, d, r, steps=RAIL_CORNER_STEPS):
    """Faces of a straight bar of rounded-rectangle section (w along `lateral`, d across) from p0 to p1, capped; rest chart, metric UVs."""
    t = _norm(_sub(p1, p0))
    side = _norm(_sub(lateral, tuple(c * _dot(lateral, t) for c in t)))
    up = _cross(t, side)
    r = min(r, 0.49 * min(w, d))
    ring = []
    for cs, cu in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        a0 = math.degrees(math.atan2(cu, cs)) - 45.0
        for k in range(steps + 1):
            a = math.radians(a0 + 90.0 * k / steps)
            ring.append((cs * (w / 2 - r) + r * math.cos(a), cu * (d / 2 - r) + r * math.sin(a)))
    length = math.sqrt(sum(c * c for c in _sub(p1, p0)))
    r0 = SEAT_CHARTS["rest"]
    faces, s = [], [0.0]
    for a, b in zip(ring, ring[1:] + ring[:1]):
        s.append(s[-1] + math.hypot(a[0] - b[0], a[1] - b[1]))
    group = ("bar", round(p0[0], 5), round(p0[1], 5), round(p0[2], 5))
    for i in range(len(ring)):
        a, b = ring[i], ring[(i + 1) % len(ring)]
        pa_ = _add(_add(p0, side, a[0]), up, a[1])
        pb_ = _add(_add(p0, side, b[0]), up, b[1])
        qa, qb = _add(pa_, t, length), _add(pb_, t, length)
        mid = _norm(_add(_add((0.0, 0.0, 0.0), side, a[0] + b[0]), up, a[1] + b[1]))
        faces.append({"pts": [pa_, pb_, qb, qa], "uv": [(r0[0] + s[i], r0[1]), (r0[0] + s[i + 1], r0[1]), (r0[0] + s[i + 1], r0[1] + length),
                                                       (r0[0] + s[i], r0[1] + length)], "part": "rest", "group": group, "out": mid})
    for end, sign in ((p0, -1.0), (p1, 1.0)):
        base = end
        pts = [_add(_add(base, side, a[0]), up, a[1]) for a in ring]
        uv = [(r0[0] + a[0], r0[1] + a[1]) for a in ring]
        faces.append({"pts": pts, "uv": uv, "part": "rest", "group": None, "out": tuple(c * sign for c in t)})
    return faces


def rounded_slab(centre, ex, ey, ez, half, r, steps=RAIL_CORNER_STEPS):
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
                                                                   (r0[0] + s[i], r0[1] + 2 * hy)], "part": "rest", "group": ("slab", id(ring)), "out": mid})
    return faces


def leg_support_parts(shape, knee_end, plate_centre, plate_normal, plate_along, rail_z_centre):
    """The carbon leg support in the seat frame: ([bars (p0, p1, lateral, w, d)], slab (centre, ex, ey, ez, half)). Two rails (rail_section_m)
    from the knee end (the pan's front end, mid-way down its side panels) to the foot plate's heel end, outer edges frame_width_m apart (knee
    end, foot end) about rail_z_centre; the foot plate (plate_m: length along the foot, thickness) between them, its top face in the plane through
    plate_centre (normal plate_normal = up the shin)."""
    w, d = shape["rail_section_m"]
    knee_w, foot_w = shape["frame_width_m"]
    plate_len, plate_t = shape["plate_m"]
    heel = _add(_add(plate_centre, plate_along, -(0.5 * plate_len - 0.5 * w)), plate_normal, -0.5 * d)
    bars = []
    for side in (-1.0, 1.0):
        a = (knee_end[0], knee_end[1], rail_z_centre + side * (0.5 * knee_w - 0.5 * w))
        b = (heel[0], heel[1], rail_z_centre + side * (0.5 * foot_w - 0.5 * w))
        bars.append((a, b, (0.0, 0.0, 1.0), w, d))
    pc = _add(plate_centre, plate_normal, -0.5 * plate_t)
    slab = ((pc[0], pc[1], rail_z_centre), plate_along, plate_normal, (0.0, 0.0, 1.0), (0.5 * plate_len, 0.5 * plate_t, 0.5 * foot_w - 0.002))
    return bars, slab


def leg_support_faces(shape, knee_end, plate_centre, plate_normal, plate_along, rail_z_centre):
    bars, slab = leg_support_parts(shape, knee_end, plate_centre, plate_normal, plate_along, rail_z_centre)
    faces = []
    for p0, p1, lateral, w, d in bars:
        faces += rounded_rect_bar(p0, p1, lateral, w, d, shape["rail_radius_m"])
    return faces + rounded_slab(*slab, shape["plate_radius_m"])


def armrest_bars(half_width):
    """The two armrests beside the pan (ARMREST_M): [(p0, p1, lateral, wide, thick)] bars along the chest normal."""
    (along, up), (length, thick, wide), r = ARMREST_M
    bars = []
    for side in (-1.0, 1.0):
        c = (up, along, side * (half_width + 0.5 * wide))
        bars.append(((c[0], c[1] - 0.5 * length, c[2]), (c[0], c[1] + 0.5 * length, c[2]), (0.0, 0.0, 1.0), wide, thick))
    return bars


def seat_parts(shape, seat, pan_raise_deg, to_seat_frame):
    """(bars, slab) of a seat's rigid parts (leg-support rails, armrests; the foot plate) in the seat frame."""
    pa, pu = pan_axes(pan_raise_deg)
    knee = (pa[0] * shape["pan_length_m"] - pu[0] * shape["pan_depth_m"] * 0.5, pa[1] * shape["pan_length_m"] - pu[1] * shape["pan_depth_m"] * 0.5)
    plate_c = to_seat_frame(seat["foot_rest_m"])
    o = to_seat_frame((0.0, 0.0, 0.0))
    plate_n = _norm(_sub(to_seat_frame(seat["foot_rest_normal"]), o))
    plate_f = _norm(_cross((0.0, 0.0, 1.0), plate_n))  # along the foot (toward the toes)
    bars, slab = leg_support_parts(shape, (knee[0], knee[1], 0.0), plate_c, plate_n, plate_f, 0.0)
    return bars + armrest_bars(0.5 * seat["width_m"]), slab


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
    bars, slab = seat_parts(shape, seat, pan_raise_deg, to_seat_frame)
    for k, (p0, p1, lateral, w, d) in enumerate(bars):
        faces += rounded_rect_bar(p0, p1, lateral, w, d, shape["rail_radius_m"] if k < 2 else ARMREST_M[2])
    return faces + rounded_slab(*slab, shape["plate_radius_m"])


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
        top = PAN_SURFACE_U + shape["pan_wing_m"] * smooth((abs(z) - shape["pan_flat_half_m"]) / max(1e-6, hw - shape["rim_radius_m"] - shape["pan_flat_half_m"]))
        if PAN_SURFACE_U - shape["pan_depth_m"] <= u < top:
            depth = max(depth, top - u)
    return depth
