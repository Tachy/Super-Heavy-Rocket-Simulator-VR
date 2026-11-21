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
    along the row and u along the centre screen's up of the face points under its keys (grown by GUARD_MARGIN_M), height over the face
    (the tallest key + GUARD_CLEAR_M)."""
    cen = next(s for s in cockpit["screen"] if s["id"] == "centre")
    sl = math.radians(cockpit["tray"]["slope_deg"])
    out = {}
    for g in cockpit.get("guard", []):
        keys = [b for b in cockpit["button"] if b["id"] in g["buttons"]]
        lat = [lateral(cen, b["centre_m"]) for b in keys]
        ups = [dot3([p - c for p, c in zip(b["centre_m"], cen["centre_m"])], cen["up"]) - math.sin(sl) * key_height(b) for b in keys]  # the key's foot on the face
        out[g["id"]] = {"lat": (min(x - b["size_m"][0] / 2.0 for x, b in zip(lat, keys)) - GUARD_MARGIN_M,
                                max(x + b["size_m"][0] / 2.0 for x, b in zip(lat, keys)) + GUARD_MARGIN_M),
                        "u": (min(u - b["size_m"][1] / 2.0 * math.cos(sl) for u, b in zip(ups, keys)) - GUARD_MARGIN_M,
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


def button_node(button):
    return "handle" if button["kind"] == "handle" else f"button_{button['id']}"


def node_names(cockpit):
    """Every mesh node of dragon_cabin.gltf, sorted."""
    names = ["cabin_shell", "floor", "hatch_frame", "console_housing", "console_back", "console_under", "console_lamps", "console_shelf", "console_lockers", "ceiling_ring_light",
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
    """The key face's axes: its panel's normal (toward the crew), up and right."""
    s = next(s for s in cockpit["screen"] if s["id"] == b["panel"])
    return {"normal": list(s["normal"]), "up": list(s["up"]), "right": [round(v, 6) for v in screen_right(s)]}


def cockpit_manifest(cockpit):
    """The manifest's `cockpit` block: node names, screen frames and button rectangles in the capsule frame, the key-label atlas."""
    cells, size = label_cells(cockpit)
    return {
        "nodes": node_names(cockpit),
        "screens": {s["id"]: {"centre": s["centre_m"], "normal": s["normal"], "up": s["up"], "size": s["size_m"]} for s in cockpit["screen"]},
        "buttons": {b["id"]: {"centre": b["centre_m"], "size": b["size_m"], "node": button_node(b), "shape": b["shape"], **button_frame(cockpit, b),
                              **({"label": b["label"], "label_cell": cells[b["id"]]} if b["id"] in cells else {})} for b in cockpit["button"]},
        "labels": {"file": LABEL_FILE, "size": list(size)},
        "button_ids": [b["id"] for b in cockpit["button"]],
        "tablets": {t["id"]: {"node": f"tablet_{t['id']}", "centre": t["centre_m"], "normal": t["normal"], "up": t["up"], "size": t["size_m"]} for t in cockpit["tablet"]},
        "guards": {g["id"]: {"node": f"guard_{g['id']}", "rim_node": f"guard_rim_{g['id']}", "rim_colour": list(g["rim_colour"]), "hinge_edge": g["hinge_edge"],
                             "buttons": list(g["buttons"])} for g in cockpit.get("guard", [])},
        "hit_node": "console_hit",
    }
