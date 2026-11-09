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


def shelf_modules(cockpit):
    """The button shelf's light-grey modules as (lat0, lat1) in the centre screen's row: buttons (and the handle) whose edges lie closer than
    console.module_split_m share a module; each module reaches module_margin_m past its outermost edges. Sorted by lat."""
    con = cockpit["console"]
    centre = next(s for s in cockpit["screen"] if s["id"] == "centre")
    spans = sorted((lateral(centre, b["centre_m"]) - b["size_m"][0] / 2.0, lateral(centre, b["centre_m"]) + b["size_m"][0] / 2.0) for b in cockpit["button"])
    groups = [list(spans[0])]
    for lo, hi in spans[1:]:
        if lo - groups[-1][1] > con["module_split_m"]:
            groups.append([lo, hi])
        else:
            groups[-1][1] = max(groups[-1][1], hi)
    m = con["module_margin_m"]
    return [(lo - m, hi + m) for lo, hi in groups]


def button_node(button):
    return "handle" if button["kind"] == "handle" else f"button_{button['id']}"


def node_names(cockpit):
    """Every mesh node of dragon_cabin.gltf, sorted."""
    names = ["cabin_shell", "floor", "hatch_frame", "console_housing", "console_back", "console_under", "console_lamps", "console_shelf", "console_lockers", "ceiling_ring_light",
             "handle", "hotas_stick", "hotas_throttle", "tablet_l", "tablet_r"]
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
    }
