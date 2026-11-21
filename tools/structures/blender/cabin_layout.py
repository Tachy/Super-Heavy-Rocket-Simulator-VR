"""Crew Dragon cabin: the pure-Python layout shared by gen_dragon_cabin.py (Blender), build_models.py (manifest) and the tests.

No Blender import: plain tuples. Frame = the capsule stage body frame of gen_dragon.py (X station from the heat shield apex, nose +X,
(Y, Z) = r (cos az, sin az)). The cabin's windows / hatch are the exterior table's apertures in its outer-surface chart (x, az); the
inner surface is the exterior meridian offset by `wall_m` along the outward normal (config/cockpits/crew_dragon.toml [cabin]).
"""
import math

SCREEN_IDS = ("left", "centre", "right")
SEAT_IDS = ("s1", "s2", "s3", "s4")
SEAT_PARTS = ("shell", "pad", "rest")  # white shell, black pads, black carbon leg-rest frame with the foot plate
HATCH_POWER = 4.0  # [E] the hatch outline's super-ellipse exponent, same as gen_dragon.py (ellipse(..., power=4.0))
GLASS_DEPTH_M = 0.02  # [E] window glass sits this far under the exterior surface (the exterior window is lifted 8 mm on it, gen_dragon.py lift_detail)
LIFT_M = 0.006  # [E] trim, bezel and frame stand this far proud of the inner wall
TILT_MATCH_DEG = 0.5  # [S] fidelity plan Task 4: the console's axes (from [console] tilt_deg) match the centre screen's within 0.5 deg
HATCH_RECESS_M = 0.03  # [E] the hatch's inner face lies this much behind the inner wall (the door is thick): discovery views of the recess under the console


def lerp(table, x):
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0) if x1 > x0 else y1
    return table[-1][1]


def ext_normal(profile, x):
    """Outward unit normal (n_x, n_r) of the exterior meridian at station x (central difference over 2 cm)."""
    h = 0.01
    m = (lerp(profile, x + h) - lerp(profile, x - h)) / (2.0 * h)
    k = math.sqrt(1.0 + m * m)
    return -m / k, 1.0 / k


def at_depth(profile, x_o, az_deg, depth_m):
    """Point `depth_m` under the exterior surface at the outer chart position (x_o, az), along the outward normal. Returns (x, y, z)."""
    nx, nr = ext_normal(profile, x_o)
    r = lerp(profile, x_o)
    x, rr = x_o - depth_m * nx, r - depth_m * nr
    a = math.radians(az_deg)
    return x, rr * math.cos(a), rr * math.sin(a)


def to_outer_chart(profile, x_in, r_in, wall_m):
    """Outer chart station of an inner-wall point: the point `wall_m` out along the normal (fixed-point on the exterior normal)."""
    x_o = x_in
    for _ in range(6):
        nx, nr = ext_normal(profile, x_o)
        x_o = x_in + wall_m * nx
    return x_o


def window_apertures(cabin):
    """Window i: the table rows in order, each +az then -az. [{index, az_deg, x_m, half_x_m, half_az_deg}]."""
    out = []
    for az, x, hx, haz in cabin["windows"]:
        for sign in (1.0, -1.0):
            out.append({"index": len(out), "az_deg": sign * az, "x_m": x, "half_x_m": hx, "half_az_deg": haz})
    return out


def screen_right(screen):
    """Right of the screen as the crew sees it: (-normal) x up."""
    n, u = screen["normal"], screen["up"]
    f = (-n[0], -n[1], -n[2])
    return (f[1] * u[2] - f[2] * u[1], f[2] * u[0] - f[0] * u[2], f[0] * u[1] - f[1] * u[0])


def dot3(a, b):
    return sum(x * y for x, y in zip(a, b))


def lateral(screen, point):
    """Position of `point` along the screen's row (right as the crew sees it), from the screen centre."""
    right = screen_right(screen)
    return dot3([p - c for p, c in zip(point, screen["centre_m"])], right)


KEY_HEIGHT_M = 0.008  # [E] key faces stand 8 mm proud of the tray face (jsc2022e068617: thick keys; >= 5 mm everywhere on the 15 deg face)
COVER_HEIGHT_M = 0.012  # [E] the covered panel (r_cover) stands 12 mm proud
GUARD_CLEAR_M = 0.004  # [E] the clear guard's top lies this far over the key faces it covers
GUARD_MARGIN_M = 0.004  # [E] the guard reaches this far past its outermost keys
SCREEN_PROUD_M = 0.002  # [E] the housing's front lies this far behind the screens (the screen quads sit exactly on the TOML plane)
TRAY_BACK_M = 0.002  # [E] the tray's back face lies in the housing front (SCREEN_PROUD_M behind the screens)


def tray_rows(cockpit):
    """The tray's heights along the centre screen's up (from its centre): top at the housing, key face top / bottom / middle, lip bottom."""
    con, tr = cockpit["console"], cockpit["tray"]
    cen = next(s for s in cockpit["screen"] if s["id"] == "centre")
    top = -cen["size_m"][1] / 2.0 - con["shelf_top_gap_m"]
    bottom = top - con["shelf_height_m"]
    face_top, face_bot = top - tr["deck_drop_m"], bottom + tr["lip_width_m"]
    return {"top": top, "face_top": face_top, "face_bot": face_bot, "mid": 0.5 * (face_top + face_bot), "bottom": bottom}


def tray_face_o(cockpit, u):
    """The key face's distance in front of the screen plane at height u: depth_m at the face's middle, leaning out toward its bottom by slope_deg."""
    tr = cockpit["tray"]
    return tr["depth_m"] - (u - tray_rows(cockpit)["mid"]) * math.tan(math.radians(tr["slope_deg"]))


def tray_axes(cockpit):
    """(normal, up) of the key face in the centre screen's (up, o) plane: the face normal is turned up by slope_deg (toward the crew's eyes)."""
    s = math.radians(cockpit["tray"]["slope_deg"])
    return (math.sin(s), math.cos(s)), (math.cos(s), -math.sin(s))


def tray_profiles(cockpit):
    """(body, lip): convex (u, o) profiles of the tray (extruded along the row): the body = deck from the housing face to the key face, the key face
    down to the bottom, back to the housing; the lip = a strip on the face's bottom lip_width_m, lip_height_m proud (it overlaps the body by 1 mm)."""
    r, tr = tray_rows(cockpit), cockpit["tray"]
    o = lambda u: tray_face_o(cockpit, u)  # noqa: E731
    n = tray_axes(cockpit)[0]
    body = [(r["top"], -TRAY_BACK_M), (r["face_top"], o(r["face_top"])), (r["bottom"], o(r["bottom"])), (r["bottom"], -TRAY_BACK_M)]
    h = tr["lip_height_m"]
    lip = [(r["face_bot"] - n[0] * 0.001, o(r["face_bot"]) - n[1] * 0.001), (r["face_bot"] + n[0] * h, o(r["face_bot"]) + n[1] * h),
           (r["bottom"] + n[0] * h, o(r["bottom"]) + n[1] * h), (r["bottom"] - n[0] * 0.001, o(r["bottom"]) - n[1] * 0.001)]
    return body, lip


def key_height(b):
    return COVER_HEIGHT_M if b["kind"] == "covered" else KEY_HEIGHT_M


def hit_faces(cockpit):
    """{button id: [4 corners]}: the pointer's flat hit face of every key (and the handle): its face rectangle (size_m) centred on centre_m,
    square to the tray face normal (the handle: to its panel's normal), counter-clockwise seen from the crew."""
    cen = next(s for s in cockpit["screen"] if s["id"] == "centre")
    up0, n0, right = cen["up"], cen["normal"], screen_right(cen)
    (nu, no), (tu, to) = tray_axes(cockpit)
    out = {}
    for b in cockpit["button"]:
        if b["kind"] == "handle":
            n, up = n0, up0
        else:
            n = [up0[i] * nu + n0[i] * no for i in range(3)]
            up = [up0[i] * tu + n0[i] * to for i in range(3)]
        w, h = b["size_m"]
        c = b["centre_m"]
        out[b["id"]] = [tuple(c[i] + right[i] * sa * w / 2.0 + up[i] * sb * h / 2.0 for i in range(3)) for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return out


def guards(cockpit):
    """{guard id: {lat: (lo, hi), u: (lo, hi), height, rim_colour, hinge_edge, buttons}}: the clear cover box standing on the tray face: lat
    along the row and u along the centre screen's up of the face points under its keys (grown by GUARD_MARGIN_M, the lower edge clamped to the face bottom so the cover clears the lip), height over the face
    (the tallest key + GUARD_CLEAR_M)."""
    cen = next(s for s in cockpit["screen"] if s["id"] == "centre")
    sl = math.radians(cockpit["tray"]["slope_deg"])
    face_bot = tray_rows(cockpit)["face_bot"]  # the cover stands on the face, above the lip (the lip is proud of the face)
    out = {}
    for g in cockpit.get("guard", []):
        keys = [b for b in cockpit["button"] if b["id"] in g["buttons"]]
        lat = [lateral(cen, b["centre_m"]) for b in keys]
        ups = [dot3([p - c for p, c in zip(b["centre_m"], cen["centre_m"])], cen["up"]) - math.sin(sl) * key_height(b) for b in keys]  # the key's foot on the face
        out[g["id"]] = {"lat": (min(x - b["size_m"][0] / 2.0 for x, b in zip(lat, keys)) - GUARD_MARGIN_M,
                                max(x + b["size_m"][0] / 2.0 for x, b in zip(lat, keys)) + GUARD_MARGIN_M),
                        "u": (max(face_bot, min(u - b["size_m"][1] / 2.0 * math.cos(sl) for u, b in zip(ups, keys)) - GUARD_MARGIN_M),
                              max(u + b["size_m"][1] / 2.0 * math.cos(sl) for u, b in zip(ups, keys)) + GUARD_MARGIN_M),
                        "height": max(key_height(b) for b in keys) + GUARD_CLEAR_M, "rim_colour": list(g["rim_colour"]), "hinge_edge": g["hinge_edge"],
                        "buttons": list(g["buttons"])}
    return out


def console_frame(cockpit):
    """The console body's frame: origin = the centre screen centre, normal (toward the crew), up, right (crew's right); u_top / u_bot = the body's
    top and front bottom edge along up from the origin ([console] centre_m and height_m)."""
    con = cockpit["console"]
    cen = next(s for s in cockpit["screen"] if s["id"] == "centre")
    t = math.radians(con["tilt_deg"])  # the axes follow tilt_deg: up from +X toward -Y, normal toward the crew
    up, normal = (math.cos(t), -math.sin(t), 0.0), (-math.sin(t), -math.cos(t), 0.0)
    if dot3(up, cen["up"]) < math.cos(math.radians(TILT_MATCH_DEG)) or dot3(normal, cen["normal"]) < math.cos(math.radians(TILT_MATCH_DEG)):
        raise ValueError(f"console: tilt_deg {con['tilt_deg']} does not match the centre screen's up / normal (the screens sit in the housing)")
    shift = dot3([a - b for a, b in zip(cen["centre_m"], con["centre_m"])], up)
    frame = {"origin": list(cen["centre_m"]), "normal": list(normal), "up": list(up)}
    return {**frame, "right": list(screen_right({"normal": normal, "up": up})),
            "u_top": -shift + con["height_m"] / 2.0, "u_bot": -shift - con["height_m"] / 2.0}


def console_posts(cockpit):
    """[(base, top)] of the two ceiling posts (fidelity Task 4): the base on the body's top, its sleeve (post_foot_m[0]) in front of the back
    face, lowered onto the back edge's fillet; the top on the ceiling plane at post_top_m (Y, |Z|), on the base's side."""
    con, cab = cockpit["console"], cockpit["cabin"]
    f = console_frame(cockpit)
    r, inset = con["edge_radius_m"], con["post_foot_m"][0]
    sag = r - math.sqrt(r * r - (r - inset) ** 2) if inset < r else 0.0  # the fillet's surface this far under the flat top
    o = -(con["hood_depth_m"] - inset)
    out = []
    for z in con["strut_z_m"]:
        lat = (z - f["origin"][2]) / f["right"][2]
        base = tuple(f["origin"][i] + f["right"][i] * lat + f["up"][i] * (f["u_top"] - sag) + f["normal"][i] * o for i in range(3))
        top = (cab["ceiling_x_m"], con["post_top_m"][0], math.copysign(con["post_top_m"][1], z))
        out.append((base, top))
    return out


def recess_edge_az(rec, name, x):
    """|az| (deg) of the recess edge `name` ("out" = the panel edge, "in" = the floor edge) at station x: the fitted line inside its read
    station range (|az| = c0 + c1 t + c2 t^2, t = x - x_ref_m; cabin_fit/parts/recess.py), constant outside it."""
    lo, hi = rec[f"{name}_x_m"]
    t = min(max(x, lo), hi) - rec["x_ref_m"]
    return sum(c * t ** p for p, c in enumerate(rec[f"edge_{name}_deg"]))


def recess_offset(cockpit, x, az_deg):
    """How far (m) the wall lies outside the panel surface at (x, az) (fidelity Task 7, [recess]): 0 on the panels, depth_m on the recess floor,
    a linear ramp across the bevel from the out edge to the in edge (|az| grows toward the recess at az 180) and at the bottom / top."""
    rec, ceiling = cockpit["recess"], cockpit["cabin"]["ceiling_x_m"]
    if x <= rec["bottom_x_m"] or x >= ceiling:
        return 0.0
    a = abs((az_deg + 180.0) % 360.0 - 180.0)
    lo, hi = recess_edge_az(rec, "out", x), recess_edge_az(rec, "in", x)
    if not hi > lo:
        raise ValueError(f"[recess] the in edge (|az| {hi:.2f} deg) must lie beyond the out edge (|az| {lo:.2f} deg) at x {x:.3f} m")
    side = min(max((a - lo) / (hi - lo), 0.0), 1.0)
    bottom = min((x - rec["bottom_x_m"]) / rec["bottom_bevel_m"], 1.0)
    top = min((ceiling - x) / rec["top_bevel_m"], 1.0)
    return rec["depth_m"] * side * bottom * top


def end_half_width(cockpit, u, depth):
    """Half width (m) of the console body at height u (up from the centre screen centre) and depth behind the screen plane (fidelity Task 7):
    the full width_m / 2 in the front zone (depth <= end_taper_from_m); behind it the end_taper plane, reached from the full width by a
    chamfer that loses end_ramp metres of half width per metre of depth (the sloped end face behind the front plate)."""
    con = cockpit["console"]
    w = con["width_m"] / 2.0
    d0 = con["end_taper_from_m"]
    if depth <= d0:
        return w
    w0, du, dd = con["end_taper"]
    return min(w, max(w0 + du * u + dd * depth, w - con["end_ramp"] * (depth - d0)))


def tablet_axes(t):
    """(normal, up, right) of a tablet with its in-plane roll (roll_deg: right turned toward up, as the crew sees it)."""
    n, u = t["normal"], t["up"]
    r = screen_right(t)
    a = math.radians(t["roll_deg"])
    c, s = math.cos(a), math.sin(a)
    right = tuple(c * r[i] + s * u[i] for i in range(3))
    up = tuple(c * u[i] - s * r[i] for i in range(3))
    return tuple(n), up, right


def tablet_corners(t):
    """{"tl"|"tr"|"br"|"bl": point} of the tablet body's front face (tl = the crew's left, top)."""
    n, up, right = tablet_axes(t)
    w, h = t["size_m"]
    c = t["centre_m"]
    out = {}
    for name, (a, b) in (("tl", (-1, 1)), ("tr", (1, 1)), ("br", (1, -1)), ("bl", (-1, -1))):
        out[name] = tuple(c[i] + right[i] * a * w / 2.0 + up[i] * b * h / 2.0 for i in range(3))
    return out


TABLET_MAPS = {"albedo": "tablet_albedo.png", "normal": "tablet_normal.png", "orm": "tablet_orm.png"}  # cabin_textures.py bakes them (fidelity Task 7)
TABLET_FACE_U = 512.0 / 576.0  # the tablet maps: the front face spans U 0..TABLET_FACE_U, the strip right of it holds the back (top half) and arm (bottom half) swatches
TABLET_BACK_UV = (0.95, 0.75)  # Blender UV (v up) of the aluminium back / sides swatch
TABLET_ARM_UV = (0.95, 0.25)  # Blender UV of the dark arm swatch
CONSOLE_MAPS = {"albedo": "console_albedo.png", "normal": "console_normal.png", "orm": "console_orm.png"}  # cabin_textures.py bakes them
# the seat atlas (fidelity Task 8, cabin_textures.bake_seat; layout seat_shape.SEAT_CHARTS) and the two tiling detail maps (fabric grain, carbon twill)
SEAT_MAPS = {"albedo": "seat_albedo.png", "normal": "seat_normal.png", "orm": "seat_orm.png", "fabric": "seat_fabric_detail.png", "carbon": "seat_carbon_detail.png"}
SEAT_DETAIL_TILE_M = {"fabric": 0.02, "carbon": 0.02}  # [E] the detail tiles' size (m): CockpitView repeats them seat_shape.SEAT_ATLAS_M / tile times over the atlas UV
CONSOLE_ATLAS_SIZE = (4096, 2048)  # [E] fidelity Task 6: power of two (mipmaps), one atlas for the console body, tray and posts
CHART_MARGIN_M = 0.01  # [E] every chart reaches this far past the surface it covers (fillets, the frame plate's walls)
CHART_GUTTER_PX = 8  # [E] free texels between charts (the baker paints into them: no bleeding down to the 8th mip)
CHART_TEXEL_M = {"front": 0.0005, "tray_face": 0.0005, "tray_top": 0.0005, "tray_under": 0.001, "back": 0.0013, "top": 0.0013,
                 "under": 0.0013, "end_l": 0.001, "end_r": 0.001, "post": 0.002}  # [E] 0.5 mm where the crew looks from 0.5-0.8 m (~3.6 px per mm at the Pimax target), coarser elsewhere
POST_UV_WRAP = 1.1  # the post charts run 1.1 turns around: a face across the angle seam keeps continuous UVs
NODE_CHARTS = ("console_housing", "console_back", "console_under", "console_shelf", "console_lockers")  # the nodes on the console atlas


def console_local(cockpit, p):
    """(lat, u, o) of a cabin point in console_frame (origin the centre screen centre)."""
    f = console_frame(cockpit)
    d = [p[i] - f["origin"][i] for i in range(3)]
    return dot3(d, f["right"]), dot3(d, f["up"]), dot3(d, f["normal"])


def _pack(sizes, atlas):
    """{name: (x, y)}: first fit in decreasing area on an 8 px grid (deterministic); raises if a chart does not fit."""
    g = CHART_GUTTER_PX
    cols, rows = atlas[0] // g, atlas[1] // g
    used = [[False] * cols for _ in range(rows)]
    out = {}
    for name, (w, h) in sorted(sizes.items(), key=lambda kv: (-kv[1][0] * kv[1][1], kv[0])):
        cw, chh = -(-(w + g) // g), -(-(h + g) // g)
        spot = None
        for r in range(rows - chh + 1):
            for c in range(cols - cw + 1):
                if any(used[r + i][c + j] for i in (0, chh - 1) for j in (0, cw - 1)):
                    continue
                if not any(any(used[r + i][c:c + cw]) for i in range(chh)):
                    spot = (c, r)
                    break
            if spot:
                break
        if spot is None:
            raise ValueError(f"console atlas: chart {name} ({w} x {h} px) does not fit {atlas}")
        for i in range(chh):
            for j in range(cw):
                used[spot[1] + i][spot[0] + j] = True
        out[name] = (spot[0] * g, spot[1] * g)
    return out


def console_charts(cockpit):
    """{chart: {a, b, plane, facing, a_range, b_range, texel, rect}}: the console atlas (fidelity Task 6). Planar charts project a face onto two
    console_frame axes: a / b = (axis, sign) of the chart's horizontal (right in the image) and vertical (up in the image) coordinate, the face
    lies at `plane` on the third axis and faces `facing` along it; a_range / b_range in the signed coordinates (m), texel (m), rect (x, y, w, h) px
    in the atlas (y down). Post charts (post_0 / post_1, kind "cyl"): a = POST_UV_WRAP turns around the post axis (a_range in turns),
    b = the distance from the post's base along its axis."""
    con, f, rows = cockpit["console"], console_frame(cockpit), tray_rows(cockpit)
    m, hw = CHART_MARGIN_M, con["width_m"] / 2.0 + CHART_MARGIN_M
    drop = max(d for _, d in con["underside_profile"])
    u_panel = (con["upper_top_x_m"] - f["origin"][0]) / f["up"][0]
    o_lo, o_hi = -con["hood_depth_m"] - m, 0.06
    u_lo, u_hi = f["u_bot"] - drop - m, u_panel + m
    tl = cockpit["tray"]["half_length_m"] + m
    planar = {
        "front": (("lat", 1), ("u", 1), "o", -SCREEN_PROUD_M, 1, (-hw, hw), (f["u_bot"] - m, u_hi)),
        "back": (("lat", -1), ("u", 1), "o", -con["hood_depth_m"], -1, (-hw, hw), (u_lo, u_hi)),  # up to the panel: its back and the lockers' backs
        "top": (("lat", 1), ("o", 1), "u", f["u_top"], 1, (-hw, hw), (o_lo, o_hi)),
        "under": (("lat", 1), ("o", 1), "u", f["u_bot"], -1, (-hw, hw), (o_lo, o_hi)),
        "end_r": (("o", 1), ("u", 1), "lat", con["width_m"] / 2.0, 1, (o_lo, o_hi), (u_lo, u_hi)),
        "end_l": (("o", -1), ("u", 1), "lat", -con["width_m"] / 2.0, -1, (-o_hi, -o_lo), (u_lo, u_hi)),
        "tray_face": (("lat", 1), ("u", 1), "o", cockpit["tray"]["depth_m"], 1, (-tl, tl), (rows["bottom"] - m, rows["top"] + m)),
        "tray_top": (("lat", 1), ("o", 1), "u", rows["top"], 1, (-tl, tl), (-0.012, 0.05)),
        "tray_under": (("lat", 1), ("o", 1), "u", rows["bottom"], -1, (-tl, tl), (-0.012, 0.05)),
    }
    charts = {}
    for name, (a, b, _, plane, facing, ar, br) in planar.items():
        charts[name] = {"kind": "plane", "a": a, "b": b, "plane": plane, "facing": facing, "a_range": list(ar), "b_range": list(br),
                        "texel": CHART_TEXEL_M[name]}
    foot = con["post_foot_m"]
    for k, (base, top) in enumerate(console_posts(cockpit)):
        length = math.sqrt(sum((t - b) ** 2 for t, b in zip(top, base)))
        circ = 2.0 * math.pi * con["post_head_radius_m"]  # texels around = the collar's circumference (the widest part but the funnel's lip)
        charts[f"post_{k}"] = {"kind": "cyl", "a_range": [0.0, POST_UV_WRAP], "b_range": [-m, length + 2.0 * con["post_head_radius_m"] + m], "texel": CHART_TEXEL_M["post"],  # the head's flush cut reaches past the axis' end
                               "circumference_m": circ, "base": list(base), "top": list(top), "foot_m": list(foot)}
    sizes = {}
    for name, c in charts.items():
        span_a = (c["a_range"][1] - c["a_range"][0]) * (c["circumference_m"] if c["kind"] == "cyl" else 1.0)
        sizes[name] = (int(math.ceil(span_a / c["texel"])), int(math.ceil((c["b_range"][1] - c["b_range"][0]) / c["texel"])))
    for name, (x, y) in _pack(sizes, CONSOLE_ATLAS_SIZE).items():
        charts[name]["rect"] = [x, y, sizes[name][0], sizes[name][1]]
    return charts


def chart_of(node, n_local):
    """The chart of a face of a console node by its console-frame normal (lat, u, o): the dominant axis picks the face's chart."""
    a = [abs(c) for c in n_local]
    k = a.index(max(a))
    if node == "console_shelf":
        if k == 1:
            return "tray_top" if n_local[1] > 0 else "tray_under"
        if k == 0:
            return "end_r" if n_local[0] > 0 else "end_l"
        return "tray_face"
    if k == 0:
        return "end_r" if n_local[0] > 0 else "end_l"
    if k == 1:
        return "top" if n_local[1] > 0 else "under"
    return "front" if n_local[2] > 0 else "back"


def chart_uv(chart, a, b):
    """Atlas (x, y) in 0..1 (y down from the top, the glTF / Godot convention) of chart coordinates (a, b)."""
    x, y, w, h = chart["rect"]
    if chart["kind"] == "cyl":
        a = a * chart["circumference_m"]
        a0 = chart["a_range"][0] * chart["circumference_m"]
    else:
        a0 = chart["a_range"][0]
    W, H = CONSOLE_ATLAS_SIZE
    return (x + (a - a0) / chart["texel"]) / W, (y + (chart["b_range"][1] - b) / chart["texel"]) / H


def plane_ab(chart, lat_u_o):
    """Chart coordinates (a, b) of a console-local point on a planar chart."""
    idx = {"lat": 0, "u": 1, "o": 2}
    (an, asg), (bn, bsg) = chart["a"], chart["b"]
    return asg * lat_u_o[idx[an]], bsg * lat_u_o[idx[bn]]


def plane_point(chart, a, b, depth=None):
    """Console-local (lat, u, o) of chart coordinates (a, b) on the chart's plane (or at `depth` on its normal axis)."""
    idx = {"lat": 0, "u": 1, "o": 2}
    (an, asg), (bn, bsg) = chart["a"], chart["b"]
    p = [0.0, 0.0, 0.0]
    p[idx[an]], p[idx[bn]] = a * asg, b * bsg
    k = ({0, 1, 2} - {idx[an], idx[bn]}).pop()
    p[k] = chart["plane"] if depth is None else depth
    return tuple(p)


def post_ab(chart, p):
    """(turns around the axis in [0, 1), distance from the base, distance from the axis) of a cabin point on a post (the axis frame of console_shape.post)."""
    base, top = chart["base"], chart["top"]
    ez = [t - b for t, b in zip(top, base)]
    length = math.sqrt(dot3(ez, ez))
    ez = [c / length for c in ez]
    ref = (1.0, 0.0, 0.0) if abs(ez[0]) < 0.9 else (0.0, 1.0, 0.0)
    ex = [ref[1] * ez[2] - ref[2] * ez[1], ref[2] * ez[0] - ref[0] * ez[2], ref[0] * ez[1] - ref[1] * ez[0]]
    k = math.sqrt(dot3(ex, ex))
    ex = [c / k for c in ex]
    ey = [ez[1] * ex[2] - ez[2] * ex[1], ez[2] * ex[0] - ez[0] * ex[2], ez[0] * ex[1] - ez[1] * ex[0]]
    d = [p[i] - base[i] for i in range(3)]
    s = dot3(d, ez)
    x, y = dot3(d, ex), dot3(d, ey)
    return (math.atan2(y, x) / (2.0 * math.pi)) % 1.0, s, math.hypot(x, y)


def face_uvs(cockpit, charts, node, pts, normal):
    """Atlas UVs (Blender convention: v up = 1 - y) of one face of a console node: pts cabin points, normal the face's cabin normal."""
    f = console_frame(cockpit)
    n_local = (dot3(normal, f["right"]), dot3(normal, f["up"]), dot3(normal, f["normal"]))
    ch = charts[chart_of(node, n_local)]
    out = []
    for p in pts:
        x, y = chart_uv(ch, *plane_ab(ch, console_local(cockpit, p)))
        out.append((x, 1.0 - y))
    return out


def post_face_uvs(charts, k, pts):
    """Atlas UVs (Blender convention) of one face of post k: the turns unwrapped across the seam within the face."""
    ch = charts[f"post_{k}"]
    ab = [post_ab(ch, p) for p in pts]
    turns = [a for a, _, _ in ab]
    if max(turns) - min(turns) > 0.5:
        turns = [t + 1.0 if t < 0.5 else t for t in turns]
    if max(turns) - min(turns) > 0.5 or min(r for _, _, r in ab) < 1e-6:  # an end cap (round the axis): one colour, no wrap
        turns = [0.5] * len(turns)
    out = []
    for t, (_, s, _) in zip(turns, ab):
        x, y = chart_uv(ch, t, s)
        out.append((x, 1.0 - y))
    return out


def button_node(button):
    return "handle" if button["kind"] == "handle" else f"button_{button['id']}"


def node_names(cockpit):
    """Every mesh node of dragon_cabin.gltf, sorted."""
    names = ["cabin_shell", "cabin_recess", "floor", "hatch_frame", "forward_hatch", "tunnel_handles", "ceiling_vents", "console_housing", "console_back", "console_under", "console_lamps", "console_shelf", "console_lockers", "ceiling_ring_light",
             "console_bezel", "console_hit", "handle", "hotas_stick", "hotas_throttle", "tablet_l", "tablet_r"]
    names += [n for g in cockpit.get("guard", []) for n in (f"guard_{g['id']}", f"guard_rim_{g['id']}")]
    names += [f"seat_{part}_{s['id']}" for s in cockpit["seat"] for part in SEAT_PARTS]
    names += [f"screen_{s['id']}" for s in cockpit["screen"]]
    for w in window_apertures(cockpit["cabin"]):
        names += [f"window_{w['index']}", f"window_glass_{w['index']}"]
    names += [f"light_{i}" for i in range(len(cockpit["light"]))]
    names += [button_node(b) for b in cockpit["button"] if b["kind"] != "handle"]
    return sorted(names)


LABEL_FILE = "cockpit_labels.png"  # the key-label atlas (surface_textures.write_dragon bakes it)
LABEL_PX_PER_M = 8000.0  # [E] 8 px per mm of key face: 2-3 mm letters stay sharp at the eye's 0.6-0.8 m (about 1.3 px per arcminute)
LABEL_ATLAS_WIDTH = 1024


def label_cells(cockpit):
    """({button id: [u0, v0, u1, v1]} in the atlas (v down from the top, 0..1), (width, height) in px): one cell per labelled key, the cell
    the key face's size at LABEL_PX_PER_M, packed in rows in file order."""
    cells, x, y, row_h = {}, 0, 0, 0
    sized = [(b["id"], round(b["size_m"][0] * LABEL_PX_PER_M), round(b["size_m"][1] * LABEL_PX_PER_M))
             for b in cockpit["button"] if b.get("label") and b["kind"] != "handle"]
    for bid, w, h in sized:
        if x + w > LABEL_ATLAS_WIDTH:
            x, y, row_h = 0, y + row_h, 0
        cells[bid] = (x, y, w, h)
        x, row_h = x + w, max(row_h, h)
    height = max(1, 1 << max(0, (y + row_h - 1).bit_length())) if cells else 1
    return ({bid: [x0 / LABEL_ATLAS_WIDTH, y0 / height, (x0 + w) / LABEL_ATLAS_WIDTH, (y0 + h) / height] for bid, (x0, y0, w, h) in cells.items()},
            (LABEL_ATLAS_WIDTH, height))


def button_frame(cockpit, b):
    """The key face's axes (toward the crew, up, right): the tray face's frame (tray_axes, the one hit_faces uses) for every key; the handle keeps
    its panel's frame."""
    s = next(s for s in cockpit["screen"] if s["id"] == b["panel"])
    right = [round(v, 6) for v in screen_right(s)]
    if b["kind"] == "handle":
        return {"normal": list(s["normal"]), "up": list(s["up"]), "right": right}
    (nu, no), (tu, to) = tray_axes(cockpit)
    return {"normal": [round(s["up"][i] * nu + s["normal"][i] * no, 6) for i in range(3)],
            "up": [round(s["up"][i] * tu + s["normal"][i] * to, 6) for i in range(3)], "right": right}


def cockpit_manifest(cockpit):
    """The manifest's `cockpit` block: node names, screen frames and button rectangles in the capsule frame, the key-label atlas."""
    cells, size = label_cells(cockpit)
    return {
        "nodes": node_names(cockpit),
        "screens": {s["id"]: {"centre": s["centre_m"], "normal": s["normal"], "up": s["up"], "size": s["size_m"]} for s in cockpit["screen"]},
        "buttons": {b["id"]: {"centre": b["centre_m"], "size": b["size_m"], "node": button_node(b), "shape": b["shape"], **button_frame(cockpit, b),
                              **({"label": b["label"], "label_cell": cells[b["id"]]} if b["id"] in cells else {})} for b in cockpit["button"]},
        "labels": {"file": LABEL_FILE, "size": list(size)},
        "console_maps": dict(CONSOLE_MAPS),
        "tablet_maps": dict(TABLET_MAPS),
        "seat_maps": dict(SEAT_MAPS),
        "seat_detail_tiles": {k: 3.0 / v for k, v in SEAT_DETAIL_TILE_M.items()},  # atlas UV -> detail UV (seat_shape.SEAT_ATLAS_M 3.0 m)
        "button_ids": [b["id"] for b in cockpit["button"]],
        "tablets": {t["id"]: {"node": f"tablet_{t['id']}", "centre": t["centre_m"], "normal": t["normal"], "up": t["up"], "size": t["size_m"]} for t in cockpit["tablet"]},
        "guards": {g["id"]: {"node": f"guard_{g['id']}", "rim_node": f"guard_rim_{g['id']}", "rim_colour": list(g["rim_colour"]), "hinge_edge": g["hinge_edge"],
                             "buttons": list(g["buttons"])} for g in cockpit.get("guard", [])},
        "hit_node": "console_hit",
    }
