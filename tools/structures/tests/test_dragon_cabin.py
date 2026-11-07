"""Crew Dragon cabin and console (gen_dragon_cabin.py -> dragon_cabin.gltf): the typed cockpit tables are tagged, the generated node set is
the list of the brief, the screen quads equal the TOML, the shell lies inside the exterior profile with a wall, the window wells line up
with the exterior windows, and the shell has no hole except the apertures. Generated-model cases skip without generated Dragon models
(python tools/structures/build_models.py --vehicle crew_dragon)."""
import base64
import collections
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
import build_models  # noqa: E402
import cabin_layout as cl  # noqa: E402
import vehicle_spec  # noqa: E402

ID = "crew_dragon"
MODELS = build_models.ROOT / "data" / "models" / ID
COCKPIT = vehicle_spec.COCKPITS / "crew_dragon.toml"
GENERATED = (MODELS / "dragon_capsule.gltf").is_file()
WALL_MIN_M = 0.02  # [S] brief: shell inside the exterior with >= 2 cm wall
WINDOW_TOL_M, WINDOW_TOL_DEG = 0.025, 0.55  # [E] the Task 1 gate (10 % of the glass half sizes), cockpit.md "Windows and hatch"
SCREEN_TOL_M = 1e-3  # [S] brief: 1 mm
WELD = 1e4  # 0.1 mm


def _doc():
    path = MODELS / "dragon_cabin.gltf"
    doc = json.loads(path.read_text(encoding="utf-8"))
    buffers = []
    for b in doc["buffers"]:
        uri = b["uri"]
        buffers.append(base64.b64decode(uri.split(",", 1)[1]) if uri.startswith("data:") else (path.parent / uri).read_bytes())
    return doc, buffers


def _read(doc, buffers, i, comps):
    import struct
    a = doc["accessors"][i]
    v = doc["bufferViews"][a["bufferView"]]
    off = v.get("byteOffset", 0) + a.get("byteOffset", 0)
    data = buffers[v["buffer"]]
    if comps == 3:
        stride = v.get("byteStride", 12)
        return [struct.unpack_from("<3f", data, off + k * stride) for k in range(a["count"])]
    if comps == 2:
        stride = v.get("byteStride", 8)
        return [struct.unpack_from("<2f", data, off + k * stride) for k in range(a["count"])]
    fmt, size = {5123: ("<H", 2), 5125: ("<I", 4), 5121: ("<B", 1)}[a["componentType"]]
    return [struct.unpack_from(fmt, data, off + k * size)[0] for k in range(a["count"])]


def _nodes():
    """{name: {"pos": [(x, y, z)], "idx": [int], "uv": [(u, v)]}} in the file's frame (identity node transforms)."""
    doc, buffers = _doc()
    out = {}
    for node in doc["nodes"]:
        if "mesh" not in node:
            continue
        assert not ({"translation", "rotation", "scale"} & set(node)), node["name"]
        pos, idx, uv = [], [], []
        for prim in doc["meshes"][node["mesh"]]["primitives"]:
            base = len(pos)
            pos += _read(doc, buffers, prim["attributes"]["POSITION"], 3)
            uv += _read(doc, buffers, prim["attributes"]["TEXCOORD_0"], 2) if "TEXCOORD_0" in prim["attributes"] else []
            idx += [base + i for i in _read(doc, buffers, prim["indices"], 1)]
        out[node["name"]] = {"pos": pos, "idx": idx, "uv": uv}
    return out


@pytest.fixture(scope="module")
def spec():
    return vehicle_spec.load(ID)


@pytest.fixture(scope="module")
def nodes():
    if not GENERATED:
        pytest.skip("no generated Dragon models")
    return _nodes()


def test_cockpit_tables_are_tagged_and_load_into_the_spec(spec):
    assert vehicle_spec.value_lines_without_tag(COCKPIT) == []
    c = spec["cockpit"]
    assert [s["id"] for s in c["seat"]] == list(cl.SEAT_IDS)
    assert sorted(s["id"] for s in c["screen"]) == sorted(cl.SCREEN_IDS)
    assert len(c["button"]) == 39


def test_validation_names_a_missing_screen_and_a_bad_panel(spec):
    broken = json.loads(json.dumps(spec["cockpit"]))
    broken["screen"] = broken["screen"][:2]
    with pytest.raises(vehicle_spec.VehicleSpecError):
        vehicle_spec.validate_cockpit(broken)
    broken = json.loads(json.dumps(spec["cockpit"]))
    broken["button"][0]["panel"] = "nowhere"
    with pytest.raises(vehicle_spec.VehicleSpecError):
        vehicle_spec.validate_cockpit(broken)


def test_manifest_block_carries_screens_buttons_and_nodes(spec):
    block = cl.cockpit_manifest(spec["cockpit"])
    assert sorted(block["screens"]) == sorted(cl.SCREEN_IDS)
    assert set(block["screens"]["centre"]) == {"centre", "normal", "up", "size"}
    assert len(block["buttons"]) == 39 and "handle" in block["buttons"]
    assert block["nodes"] == sorted(block["nodes"])


def test_generated_node_set_is_the_brief_list(spec, nodes):
    assert sorted(nodes) == cl.node_names(spec["cockpit"])
    names = set(nodes)
    for required in ("cabin_shell", "floor", "hatch_frame", "console_housing", "handle", "hotas_stick", "hotas_throttle",
                     "tablet_l", "tablet_r", "screen_left", "screen_centre", "screen_right", "seat_shell_s1", "seat_pad_s2", "seat_rest_s4", "console_back", "console_shelf", "console_lockers", "ceiling_ring_light", "light_0", "light_1",
                     "window_0", "window_3", "window_glass_0", "window_glass_3", "button_cam1", "button_r_cover"):
        assert required in names, required


@pytest.mark.parametrize("sid", cl.SCREEN_IDS)
def test_screen_quad_matches_the_toml(spec, nodes, sid):
    s = next(s for s in spec["cockpit"]["screen"] if s["id"] == sid)
    n = nodes[f"screen_{sid}"]
    pos = n["pos"]
    assert len(n["idx"]) == 6 and len({tuple(round(c, 5) for c in p) for p in pos}) == 4
    centre = [sum(p[i] for p in pos) / len(pos) for i in range(3)]
    right, up = cl.screen_right(s), s["up"]
    dot = lambda a, b: sum(x * y for x, y in zip(a, b))
    along_r = [dot([p[i] - centre[i] for i in range(3)], right) for p in pos]
    along_u = [dot([p[i] - centre[i] for i in range(3)], up) for p in pos]
    assert max(along_r) - min(along_r) == pytest.approx(s["size_m"][0], abs=SCREEN_TOL_M)
    assert max(along_u) - min(along_u) == pytest.approx(s["size_m"][1], abs=SCREEN_TOL_M)
    # the quad lies on the screen's plane and is centred on the TOML centre
    d = [centre[i] - s["centre_m"][i] for i in range(3)]
    assert dot(d, right) == pytest.approx(0.0, abs=SCREEN_TOL_M) and dot(d, up) == pytest.approx(0.0, abs=SCREEN_TOL_M)
    assert dot(d, s["normal"]) == pytest.approx(0.0, abs=SCREEN_TOL_M)
    # +normal toward the crew: the triangles wind counter-clockwise seen from there
    i0, i1, i2 = n["idx"][:3]
    a, b, c = pos[i0], pos[i1], pos[i2]
    e1, e2 = [b[i] - a[i] for i in range(3)], [c[i] - a[i] for i in range(3)]
    cross = (e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0])
    assert dot(cross, s["normal"]) > 0.0
    # UV 0..1 over the active area, (0, 0) top left
    uv = n["uv"]
    assert sorted({round(u, 4) for u, _ in uv}) == [0.0, 1.0] and sorted({round(v, 4) for _, v in uv}) == [0.0, 1.0]
    top_left = min(range(len(pos)), key=lambda k: (dot([pos[k][i] - centre[i] for i in range(3)], right) - dot([pos[k][i] - centre[i] for i in range(3)], up)))
    assert uv[top_left] == pytest.approx((0.0, 0.0), abs=1e-4)


def _segment_distance(p, a, b):
    abx, aby = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / (abx * abx + aby * aby)))
    return math.hypot(p[0] - a[0] - t * abx, p[1] - a[1] - t * aby)


def test_shell_lies_inside_the_exterior_profile_with_a_wall(spec, nodes):
    profile = spec["detail"]["capsule"]["profile_m"]
    segments = list(zip(profile, profile[1:]))
    seen = set()
    worst = 1e9
    for name in ("cabin_shell", "floor"):
        for x, y, z in nodes[name]["pos"]:
            key = (round(x, 3), round(math.hypot(y, z), 3))
            if key in seen:
                continue
            seen.add(key)
            if x > segments[-1][1][0] or x < 0.0:
                continue
            p = (x, math.hypot(y, z))
            worst = min(worst, min(_segment_distance(p, a, b) for a, b in segments if a[0] - 0.5 <= x <= b[0] + 0.5))
            assert p[1] < cl.lerp(profile, x) or x < 1.0, (name, p)
    assert worst >= WALL_MIN_M, worst


@pytest.mark.parametrize("w", range(4))
def test_window_wells_line_up_with_the_exterior_windows(spec, nodes, w):
    ap = cl.window_apertures(spec["cockpit"]["cabin"])[w]
    pos = nodes[f"window_glass_{w}"]["pos"]
    cx, cy, cz = (sum(p[i] for p in pos) / len(pos) for i in range(3))
    az = math.degrees(math.atan2(cz, cy))
    x_outer = cl.to_outer_chart(spec["detail"]["capsule"]["profile_m"], cx, math.hypot(cy, cz), cl.GLASS_DEPTH_M)
    assert x_outer == pytest.approx(ap["x_m"], abs=WINDOW_TOL_M)
    assert az == pytest.approx(ap["az_deg"], abs=WINDOW_TOL_DEG)
    # the glass fills the exterior pane: its azimuth and station spans are the table's half sizes (within the same gate)
    azs = [math.degrees(math.atan2(p[2], p[1])) for p in pos]
    assert (max(azs) - min(azs)) / 2.0 == pytest.approx(ap["half_az_deg"], abs=WINDOW_TOL_DEG)
    xs = [p[0] for p in pos]
    assert (max(xs) - min(xs)) / 2.0 == pytest.approx(ap["half_x_m"], abs=WINDOW_TOL_M * 1.5)  # the glass plane is tilted by the wall slope


def test_shell_has_no_hole_except_the_apertures(spec, nodes):
    cockpit, profile = spec["cockpit"], spec["detail"]["capsule"]["profile_m"]
    cabin = cockpit["cabin"]
    edges, owner = collections.Counter(), {}
    for name in ("cabin_shell", "floor", "hatch_frame", *(n for n in nodes if n.startswith("window_"))):
        pos, idx = nodes[name]["pos"], nodes[name]["idx"]
        key = [tuple(round(c * WELD) for c in p) for p in pos]
        for t in range(0, len(idx), 3):
            a, b, c = key[idx[t]], key[idx[t + 1]], key[idx[t + 2]]
            for e in ((a, b), (b, c), (c, a)):
                k = tuple(sorted(e))
                if k[0] == k[1]:
                    continue
                edges[k] += 1
                owner[k] = name
    reach = cabin["window_reveal_radius_m"] + cabin["window_bezel_m"] + 0.04
    windows = []
    for w in cl.window_apertures(cabin):
        windows.append(cl.at_depth(profile, w["x_m"], w["az_deg"], cabin["wall_m"]))
    h_az, h_x, h_hx, h_haz = cabin["hatch"]
    trim = cabin["hatch_trim_m"] + 0.04
    stray = []
    for k, n in edges.items():
        if n != 1:
            continue
        mid = [(k[0][i] + k[1][i]) / 2.0 / WELD for i in range(3)]
        near_window = any(math.dist(mid, c) <= reach for c in windows)
        r = math.hypot(mid[1], mid[2])
        az = math.degrees(math.atan2(mid[2], mid[1]))
        x_o = cl.to_outer_chart(profile, mid[0], r, cabin["wall_m"])
        near_hatch = abs(x_o - h_x) <= h_hx + trim and abs(az - h_az) <= h_haz + math.degrees(trim / r)
        if not (near_window or near_hatch):
            stray.append((owner[k], tuple(round(c, 3) for c in mid)))
    assert stray == [], stray[:8]


BUTTON_PROUD_MIN_M = 0.005  # [E] every button's top stands at least 5 mm in front of the housing's surface beneath it (the buttons are 6 mm / 1 cm high)


def _ray_triangle(o, d, a, b, c):
    """Moller-Trumbore: distance along d to the triangle or None."""
    sub = lambda u, v: [u[i] - v[i] for i in range(3)]
    cross = lambda u, v: [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
    dot = lambda u, v: sum(x * y for x, y in zip(u, v))
    e1, e2 = sub(b, a), sub(c, a)
    p = cross(d, e2)
    det = dot(e1, p)
    if abs(det) < 1e-12:
        return None
    t_vec = sub(o, a)
    u = dot(t_vec, p) / det
    q = cross(t_vec, e1)
    v = dot(d, q) / det
    if u < 0.0 or v < 0.0 or u + v > 1.0:
        return None
    t = dot(e2, q) / det
    return t if t > 0.0 else None


def test_every_button_stands_proud_of_the_housing(spec, nodes):
    """Shelves are built per button module: no button is buried in a neighbouring panel's housing (cluster C under the left wing, r_led_3)."""
    pos, idx = [], []
    for name in ("console_housing", "console_shelf"):  # the shelf modules carry the buttons, the housing shows between them
        idx += [len(pos) + i for i in nodes[name]["idx"]]
        pos += nodes[name]["pos"]
    screens = {s["id"]: s for s in spec["cockpit"]["screen"]}
    worst = (1e9, None)
    for b in spec["cockpit"]["button"]:
        if b["kind"] == "handle":
            continue
        n = screens[b["panel"]]["normal"]
        top = b["centre_m"]
        w, h = b["size_m"]
        right, up = cl.screen_right(screens[b["panel"]]), screens[b["panel"]]["up"]
        for sa, sb in ((0, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):  # centre and corners of the top face
            origin = [top[i] + 0.5 * n[i] + 0.4 * w * sa * right[i] + 0.4 * h * sb * up[i] for i in range(3)]
            hits = [t for k in range(0, len(idx), 3) if (t := _ray_triangle(origin, [-c for c in n], pos[idx[k]], pos[idx[k + 1]], pos[idx[k + 2]])) is not None]
            assert hits, b["id"]
            proud = min(hits) - 0.5  # the housing surface lies this far behind the button's top face
            worst = min(worst, (proud, b["id"]))
    assert worst[0] >= BUTTON_PROUD_MIN_M, f"button {worst[1]} stands only {worst[0] * 1000:.1f} mm proud of the housing"


def test_three_screens_are_coplanar_when_the_console_is_straight(spec):
    """fold_deg 0 (user decision 2026-11-07): same normal within 0.1 deg, centres on one plane within 1 mm."""
    cockpit = spec["cockpit"]
    assert cockpit["console"]["fold_deg"] == 0.0
    screens = {s["id"]: s for s in cockpit["screen"]}
    n0 = screens["centre"]["normal"]
    dot = lambda u, v: sum(x * y for x, y in zip(u, v))
    for sid in ("left", "right"):
        s = screens[sid]
        assert math.degrees(math.acos(min(1.0, dot(n0, s["normal"])))) < 0.1, sid
        off = dot(n0, [a - b for a, b in zip(s["centre_m"], screens["centre"]["centre_m"])])
        assert abs(off) < 1e-3, f"{sid} centre {off * 1000:.2f} mm off the centre screen's plane"


def test_manifest_buttons_carry_their_node_names(spec):
    block = cl.cockpit_manifest(spec["cockpit"])
    assert block["buttons"]["handle"]["node"] == "handle" and block["buttons"]["cam1"]["node"] == "button_cam1"


def test_cockpit_toml_is_part_of_the_input_hash(spec):
    """An edit of config/cockpits/crew_dragon.toml changes the crew_dragon set's hash (the spec text carries spec["cockpit"])."""
    import copy
    table = vehicle_spec.generator_table(ID)
    base = build_models.vehicle_hash(vehicle_spec.dumps(spec), "x", table)
    edited = copy.deepcopy(spec)
    edited["cockpit"]["console"]["width_m"] += 0.01
    assert build_models.vehicle_hash(vehicle_spec.dumps(edited), "x", table) != base


# ---------- fix round 2: console proportions, ring light, seats, foot rests ----------

def _extent(node, axis):
    vals = [sum(p[i] * axis[i] for i in range(3)) for p in node["pos"]]
    return min(vals), max(vals)


def test_screen_gap_is_one_to_two_centimetres(spec):
    """Bezels nearly touch (discovery2): the gap between neighbouring active areas follows console.screen_gap_m = 1-2 cm."""
    c = spec["cockpit"]
    gap = c["console"]["screen_gap_m"]
    assert 0.01 <= gap <= 0.02 + 1e-9
    screens = {s["id"]: s for s in c["screen"]}
    for sid in ("left", "right"):
        lat = abs(cl.lateral(screens["centre"], screens[sid]["centre_m"]))
        assert lat - screens["centre"]["size_m"][0] == pytest.approx(gap, abs=1e-4), sid


def test_shelf_is_a_ten_centimetre_row_of_separate_modules(spec, nodes):
    c = spec["cockpit"]
    modules = cl.shelf_modules(c)
    assert len(modules) >= 6  # clusters A, B + C (they touch), handle, right of the handle, covered panel, right cluster
    for (a0, a1), (b0, b1) in zip(modules, modules[1:]):
        assert b0 - a1 >= 0.003, "modules need a dark gap between them"
    centre = next(s for s in c["screen"] if s["id"] == "centre")
    for b in c["button"]:
        lat = cl.lateral(centre, b["centre_m"])
        assert any(m0 <= lat - b["size_m"][0] / 2.0 and lat + b["size_m"][0] / 2.0 <= m1 for m0, m1 in modules), b["id"]
    lo, hi = _extent(nodes["console_shelf"], centre["up"])
    assert hi - lo == pytest.approx(c["console"]["shelf_height_m"], abs=0.004) and 0.09 <= hi - lo <= 0.11


def test_shelf_is_proud_of_the_screens_and_the_dark_panel_runs_up_to_the_ceiling(spec, nodes):
    c = spec["cockpit"]
    cen = next(s for s in c["screen"] if s["id"] == "centre")
    n = cen["normal"]
    front = max(sum(p[i] * n[i] for i in range(3)) for p in nodes["console_shelf"]["pos"])
    plane = sum(cen["centre_m"][i] * n[i] for i in range(3))
    assert front - plane > 0.015, "the ledge stands at least 1.5 cm proud of the screens"
    up = cen["up"]
    top = max(sum(p[i] * up[i] for i in range(3)) for p in nodes["console_housing"]["pos"])
    screen_top = sum(cen["centre_m"][i] * up[i] for i in range(3)) + cen["size_m"][1] / 2.0
    assert max(p[0] for p in nodes["console_housing"]["pos"]) < c["cabin"]["ceiling_x_m"]
    assert top - screen_top > 0.3, "dark area above the screens up toward the ceiling"


def test_lockers_5_and_6_flank_the_tablets(spec, nodes):
    c = spec["cockpit"]
    lk = {l["id"]: l for l in c["locker"]}
    assert lk["5"]["size_m"] == pytest.approx([0.15, 0.165], abs=0.02)
    tab = {t["id"]: t for t in c["tablet"]}
    assert lk["5"]["centre_m"][2] > tab["l"]["centre_m"][2] + 0.1 and lk["6"]["centre_m"][2] < tab["r"]["centre_m"][2] - 0.1
    assert "console_lockers" in nodes


def test_ring_light_is_an_annulus_at_the_ceiling(spec, nodes):
    c = spec["cockpit"]
    r0, r1 = c["cabin"]["ring_light_radii_m"]
    pos = nodes["ceiling_ring_light"]["pos"]
    radii = [math.hypot(p[1], p[2]) for p in pos]
    assert min(radii) == pytest.approx(r0, abs=1e-3) and max(radii) == pytest.approx(r1, abs=1e-3)
    assert {round(p[0], 3) for p in pos} == {round(c["cabin"]["ceiling_x_m"] - 0.004, 3)}


def test_seats_are_split_into_white_shell_and_black_pads(spec, nodes):
    for s in cl.SEAT_IDS:
        for part in cl.SEAT_PARTS:
            assert nodes[f"seat_{part}_{s}"]["idx"], (part, s)
    seat = next(s for s in spec["cockpit"]["seat"] if s["id"] == "s2")
    a = math.radians(seat["back_angle_deg"])
    n = (math.sin(a), math.cos(a), 0.0)
    assert _extent(nodes["seat_pad_s2"], n)[1] > _extent(nodes["seat_shell_s2"], n)[0]  # the pad is in front of the shell's back


def test_foot_rest_lies_under_the_boots_of_every_seat(spec, nodes):
    import figure_layout as fl
    bones = fl.bones(spec["crew_figure"])
    for seat in spec["cockpit"]["seat"]:
        n, c = seat["foot_rest_normal"], seat["foot_rest_m"]
        for side in "lr":
            ankle = fl.to_capsule(bones[f"foot_{side}"]["head"], seat["hip_m"], seat["back_angle_deg"])
            d = [ankle[i] - c[i] for i in range(3)]
            along = sum(d[i] * n[i] for i in range(3))
            assert 0.02 <= along <= 0.12, (seat["id"], side, along)  # the ankle stands about the sole height over the plate
            assert math.sqrt(max(0.0, sum(x * x for x in d) - along * along)) < 0.20, (seat["id"], side)
        pos = nodes[f"seat_rest_{seat['id']}"]["pos"]
        on_plate = [p for p in pos if abs(sum((p[i] - c[i]) * n[i] for i in range(3))) < 0.02 and math.dist(p, c) < 0.3]
        assert len(on_plate) >= 8  # the plate's front face lies in the plane through foot_rest_m


def test_knee_and_shin_follow_the_review(spec):
    """Interior knee 110-120 deg, shins -15..-25 deg from the hatch axis (crew1_1, review item 5)."""
    import figure_layout as fl
    cfg = spec["crew_figure"]
    assert 110.0 <= cfg["figure"]["knee_angle_deg"] <= 120.0
    seat = next(s for s in spec["cockpit"]["seat"] if s["id"] == "s2")
    b = fl.bones(cfg)
    knee = fl.to_capsule(b["shin_l"]["head"], seat["hip_m"], seat["back_angle_deg"])
    ankle = fl.to_capsule(b["shin_l"]["tail"], seat["hip_m"], seat["back_angle_deg"])
    angle = math.degrees(math.atan2(ankle[0] - knee[0], ankle[1] - knee[1]))  # from +Y (the hatch side) toward +X
    assert -25.0 <= angle <= -15.0, angle
