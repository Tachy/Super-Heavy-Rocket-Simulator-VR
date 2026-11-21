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
    assert nodes["cabin_recess"]["idx"], "the wall recess around the console (fidelity Task 7) has faces"
    for name in ("cabin_shell", "cabin_recess", "forward_hatch", "floor", "hatch_frame", *(n for n in nodes if n.startswith("window_"))):
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


# [S] fidelity Task 5 measurement (cabin_fit/parts/console.py --as-built with the iPad mini corners, docs/subsystems/cockpit.md "Screens and tray"):
# glass 0.400 +- 0.013 x 0.227 +- 0.008 m, pitch 0.449 +- 0.015 m; active area = glass - 2 bezel (0.004 +- 0.003): diagonal 0.449 +- 0.014 m
MEASURED_ACTIVE_DIAGONAL_M, ACTIVE_DIAGONAL_SIGMA_M = 0.449, 0.014
MEASURED_SCREEN_PITCH_M, SCREEN_PITCH_SIGMA_M = 0.449, 0.015
MEASURED_GLASS_M = (0.400, 0.227)


def test_screen_active_area_matches_the_measured_diagonal(spec):
    """Each screen's active area has the measured diagonal (+-sigma), the glass = active + 2 bezel_m is the fitted glass outline, the outer
    screens sit one measured pitch from the centre one (screen_gap_m = pitch - active width); corner radius and recess are set."""
    c = spec["cockpit"]
    screens = {s["id"]: s for s in c["screen"]}
    for sid, s in screens.items():
        w, h = s["size_m"]
        assert math.hypot(w, h) == pytest.approx(MEASURED_ACTIVE_DIAGONAL_M, abs=ACTIVE_DIAGONAL_SIGMA_M), sid
        assert w + 2 * s["bezel_m"] == pytest.approx(MEASURED_GLASS_M[0], abs=0.002) and h + 2 * s["bezel_m"] == pytest.approx(MEASURED_GLASS_M[1], abs=0.002), sid
        assert 0.0 < s["corner_radius_m"] < min(w, h) / 4 and 0.0 < s["recess_m"] < 0.02, sid
    for sid in ("left", "right"):
        lat = abs(cl.lateral(screens["centre"], screens[sid]["centre_m"]))
        assert lat == pytest.approx(MEASURED_SCREEN_PITCH_M, abs=SCREEN_PITCH_SIGMA_M), sid
        assert lat - screens["centre"]["size_m"][0] == pytest.approx(c["console"]["screen_gap_m"], abs=1e-4), sid


def test_tray_is_one_continuous_sloped_row_with_a_lip(spec, nodes):
    """Task 5 (replaces the Task 2 rule "a 10 cm row of separate modules"): one tray under all keys, shelf_height_m tall, its key face leaning out
    by slope_deg, the lip proud at the bottom."""
    c = spec["cockpit"]
    centre = next(s for s in c["screen"] if s["id"] == "centre")
    right = cl.screen_right(centre)
    lo, hi = _extent(nodes["console_shelf"], right)
    keys = [cl.lateral(centre, b["centre_m"]) for b in c["button"]]
    assert lo < min(keys) - 0.01 and hi > max(keys) + 0.01
    assert hi - lo == pytest.approx(2 * c["tray"]["half_length_m"], abs=1e-3)
    up_lo, up_hi = _extent(nodes["console_shelf"], centre["up"])
    assert up_hi - up_lo == pytest.approx(c["console"]["shelf_height_m"], abs=0.003)
    # one connected piece: the body is one closed prism (+ the lip strip): two vertex islands at most
    pos, idx = nodes["console_shelf"]["pos"], nodes["console_shelf"]["idx"]
    parent = list(range(len(pos)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    key = {}
    for i, p in enumerate(pos):
        k = tuple(round(x * WELD) for x in p)
        if k in key:
            parent[find(i)] = find(key[k])
        key[k] = i
    for k in range(0, len(idx), 3):
        parent[find(idx[k + 1])] = find(idx[k])
        parent[find(idx[k + 2])] = find(idx[k])
    assert len({find(i) for i in range(len(pos))}) <= 2
    rows = cl.tray_rows(c)
    slope = (cl.tray_face_o(c, rows["face_bot"]) - cl.tray_face_o(c, rows["face_top"])) / (rows["face_top"] - rows["face_bot"])
    assert math.degrees(math.atan(slope)) == pytest.approx(c["tray"]["slope_deg"], abs=1e-6)
    n = centre["normal"]
    plane = sum(centre["centre_m"][i] * n[i] for i in range(3))
    front = max(sum(p[i] * n[i] for i in range(3)) for p in pos) - plane
    assert front == pytest.approx(cl.tray_face_o(c, rows["bottom"]) + c["tray"]["lip_height_m"] * math.cos(math.radians(c["tray"]["slope_deg"])), abs=0.002)


HIT_TOL_M = 1e-3  # [E] the hit face's centre / size match the key's centre_m / size_m within 1 mm


def test_every_button_has_a_hit_face_on_the_tray(spec, nodes):
    """console_hit carries one flat quad per key (and the handle) at its face (centre_m, size_m), and behind every key's face the tray (or the
    handle's housing) lies within the key height: the pointer hits a face that sits on the tray."""
    c = spec["cockpit"]
    hit = nodes["console_hit"]
    pos, idx = hit["pos"], hit["idx"]
    faces = cl.hit_faces(c)
    assert len(idx) == 6 * len(faces)  # two triangles per key
    tray_pos, tray_idx = [], []
    for name in ("console_shelf", "console_housing"):
        tray_idx += [len(tray_pos) + i for i in nodes[name]["idx"]]
        tray_pos += nodes[name]["pos"]
    for b in c["button"]:
        corners = faces[b["id"]]
        cc = [sum(p[i] for p in corners) / 4.0 for i in range(3)]
        assert math.dist(cc, b["centre_m"]) < HIT_TOL_M, b["id"]
        assert all(min(math.dist(corner, p) for p in pos) < HIT_TOL_M for corner in corners), f"no hit quad at {b['id']}"
        assert math.dist(corners[0], corners[1]) == pytest.approx(b["size_m"][0], abs=HIT_TOL_M)
        assert math.dist(corners[1], corners[2]) == pytest.approx(b["size_m"][1], abs=HIT_TOL_M)
        if b["kind"] == "handle":
            continue
        e1 = [corners[1][i] - corners[0][i] for i in range(3)]
        e2 = [corners[3][i] - corners[0][i] for i in range(3)]
        nrm = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]]
        k = math.sqrt(sum(x * x for x in nrm))
        nrm = [x / k for x in nrm]
        assert sum(nrm[i] * c["screen"][0]["normal"][i] for i in range(3)) > 0.9, b["id"]  # faces the crew
        origin = [cc[i] + 0.5 * nrm[i] for i in range(3)]
        hits = [t for q in range(0, len(tray_idx), 3) if (t := _ray_triangle(origin, [-x for x in nrm], tray_pos[tray_idx[q]], tray_pos[tray_idx[q + 1]], tray_pos[tray_idx[q + 2]])) is not None]
        assert hits, b["id"]
        assert min(hits) - 0.5 == pytest.approx(cl.key_height(b), abs=1e-3), f"{b['id']}: the tray lies {(min(hits) - 0.5) * 1000:.1f} mm behind the face"


def test_manifest_button_frames_are_the_hit_face_frames(spec):
    """One key frame: for every key the manifest's button_frame (the pointer's face frame) equals the frame hit_faces builds console_hit in."""
    c = spec["cockpit"]
    faces = cl.hit_faces(c)
    for b in c["button"]:
        f = cl.button_frame(c, b)
        q = faces[b["id"]]
        e1, e2 = [q[1][i] - q[0][i] for i in range(3)], [q[3][i] - q[0][i] for i in range(3)]
        right = [x / math.hypot(*e1) for x in e1]
        up = [x / math.hypot(*e2) for x in e2]
        assert all(abs(right[i] - f["right"][i]) < 1e-5 and abs(up[i] - f["up"][i]) < 1e-5 for i in range(3)), b["id"]
        nrm = [right[1] * up[2] - right[2] * up[1], right[2] * up[0] - right[0] * up[2], right[0] * up[1] - right[1] * up[0]]
        assert all(abs(nrm[i] - f["normal"][i]) < 1e-5 for i in range(3)), b["id"]


def test_guarded_block_lies_under_its_cover(spec, nodes):
    """Each [[guard]]'s clear cover spans its 2 x 3 keys (and only those) and stands over their faces; its red rim follows it."""
    c = spec["cockpit"]
    centre = next(s for s in c["screen"] if s["id"] == "centre")
    right, up, n = cl.screen_right(centre), centre["up"], centre["normal"]
    rel = lambda p: [sum((p[i] - centre["centre_m"][i]) * a[i] for i in range(3)) for a in (right, up, n)]  # noqa: E731
    assert {g["id"] for g in c["guard"]} == {"l", "r"}
    faces = cl.hit_faces(c)
    for g in c["guard"]:
        cpos, cidx = nodes[f"guard_{g['id']}"]["pos"], nodes[f"guard_{g['id']}"]["idx"]
        assert len(g["buttons"]) == 6
        cover = [rel(p) for p in nodes[f"guard_{g['id']}"]["pos"]]
        assert nodes[f"guard_rim_{g['id']}"]["idx"]
        lat = (min(p[0] for p in cover), max(p[0] for p in cover))
        ups = (min(p[1] for p in cover), max(p[1] for p in cover))
        for b in c["button"]:
            r = rel(b["centre_m"])
            inside = lat[0] < r[0] < lat[1] and ups[0] < r[1] < ups[1]
            assert inside == (b["id"] in g["buttons"]) == (b.get("guard") == g["id"]), (g["id"], b["id"])
            if inside:
                assert lat[0] < r[0] - b["size_m"][0] / 2 and r[0] + b["size_m"][0] / 2 < lat[1]
                q = faces[b["id"]]  # the cover's top lies over the key face along the face normal
                e1, e2 = [q[1][i] - q[0][i] for i in range(3)], [q[3][i] - q[0][i] for i in range(3)]
                nrm = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]]
                k = math.sqrt(sum(x * x for x in nrm))
                hits = [t for j in range(0, len(cidx), 3) if (t := _ray_triangle(b["centre_m"], [x / k for x in nrm], cpos[cidx[j]], cpos[cidx[j + 1]], cpos[cidx[j + 2]])) is not None]
                assert hits and max(hits) > 0.002, f"cover not over {b['id']}"
    # the cover stands on the key face above the red lip: its lowest point (along the console up) is not below the face bottom
    rows = cl.tray_rows(c)
    for g in c["guard"]:
        assert min(rel(p)[1] for p in nodes[f"guard_{g['id']}"]["pos"]) >= rows["face_bot"] - 1e-4, g["id"]


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
    assert lk["5"]["size_m"] == pytest.approx([0.198, 0.139], abs=0.01)  # fidelity Task 7: the measured door (cabin_fit/parts/lockers.py)
    tab = {t["id"]: t for t in c["tablet"]}
    assert lk["5"]["centre_m"][2] > tab["l"]["centre_m"][2] + 0.1 and lk["6"]["centre_m"][2] < tab["r"]["centre_m"][2] - 0.1
    assert "console_lockers" in nodes


def test_cove_light_is_a_band_under_the_ceiling_corner(spec, nodes):
    """Fidelity Task 9: the emissive band runs all round the side wall over [cabin] cove_light_x_m, 15 mm in front of the wall."""
    cab = spec["cockpit"]["cabin"]
    x0, x1 = cab["cove_light_x_m"]
    run = [p for p in cab["profile_m"][1:] if p[0] <= cab["ceiling_x_m"] + 1e-9]
    pos = nodes["ceiling_ring_light"]["pos"]
    xs = [p[0] for p in pos]
    assert min(xs) == pytest.approx(x0, abs=1e-3) and max(xs) == pytest.approx(x1, abs=1e-3)
    for p in pos:
        assert cl.lerp(run, p[0]) - math.hypot(p[1], p[2]) == pytest.approx(0.015, abs=1e-3)
    azs = [math.degrees(math.atan2(p[2], p[1])) for p in pos]
    assert max(azs) - min(azs) > 350.0


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


def test_knee_and_shin_follow_the_hatch_photo(spec):
    """Recliner posture of crew1_1 (fix round 3): interior knee 95-105 deg, thigh 10-20 deg above the hatch axis, shin 60-65 deg below it."""
    import figure_layout as fl
    cfg = spec["crew_figure"]
    assert 95.0 <= cfg["figure"]["knee_angle_deg"] <= 105.0
    seat = next(s for s in spec["cockpit"]["seat"] if s["id"] == "s2")
    b = fl.bones(cfg)
    hip = fl.to_capsule(b["thigh_l"]["head"], seat["hip_m"], seat["back_angle_deg"])
    knee = fl.to_capsule(b["shin_l"]["head"], seat["hip_m"], seat["back_angle_deg"])
    ankle = fl.to_capsule(b["shin_l"]["tail"], seat["hip_m"], seat["back_angle_deg"])
    thigh = math.degrees(math.atan2(knee[0] - hip[0], knee[1] - hip[1]))  # from +Y (the hatch side) toward +X
    shin = math.degrees(math.atan2(ankle[0] - knee[0], ankle[1] - knee[1]))
    # fix 8/8b round 1: the posture keeps >= 2 deg margin to the photo bands (it sat exactly on the 10 deg edge before)
    assert 10.0 + 2.0 <= thigh <= 20.0 - 2.0, thigh
    assert -65.5 + 1.0 <= shin <= -60.0 - 2.0, shin
    # crew1_1: the knees apart (about shoulder width), the boots side by side on the tray at hip width
    knee_r = fl.to_capsule(b["shin_r"]["head"], seat["hip_m"], seat["back_angle_deg"])
    ankle_r = fl.to_capsule(b["shin_r"]["tail"], seat["hip_m"], seat["back_angle_deg"])
    assert 0.30 <= knee[2] - knee_r[2] <= 0.40 and knee[2] - knee_r[2] > ankle[2] - ankle_r[2] + 0.10
    # shins steep toward the floor; round 4: the seat sits ~0.3 m lower than in crew1_1 (the launch posture under the reach-fitted eye, see
    # test_layout_is_one_consistent_fit), so the ankle lies above the floor instead of in the photo's 1.4-1.8 band
    assert ankle[0] >= spec["cockpit"]["cabin"]["floor_x_m"] + 0.10 and knee[0] > ankle[0] + 0.3


def test_foot_rest_is_a_tray_facing_up_the_shin(spec):
    """The plate normal is up the shin (mostly +X), not an upright wall facing the hatch camera; the tray moves with its seat (hip + the figure's
    leg offset) and its low edge stays above the floor (round 4)."""
    floor = spec["cockpit"]["cabin"]["floor_x_m"]
    for seat in spec["cockpit"]["seat"]:
        n, c, hip = seat["foot_rest_normal"], seat["foot_rest_m"], seat["hip_m"]
        assert n[0] > 0.8 and -0.6 < n[1] < -0.3 and n[2] == 0.0, n
        assert c[0] - hip[0] == pytest.approx(-0.152, abs=0.002) and c[1] - hip[1] == pytest.approx(0.754, abs=0.002), (seat["id"], c)
        low_edge = c[0] - 0.16 * abs(n[1]) - 0.02 * n[0]  # tray half length 0.16 along the foot (z x n), thickness 0.02
        assert low_edge >= floor + 0.01, (seat["id"], low_edge)


def test_layout_is_one_consistent_fit(spec):
    """Round 4 joint fit (docs/subsystems/cockpit.md "Fix round 4"): the commander reaches the screens with near-straight arms (crew2_1, inspiration4_1,
    demo1_3): eye to the centre screen centre 0.6-0.75 m, the near shoulder to its screen 0.62-0.75 m of the 0.78 m reach; the eye looks at the screens
    within 30 deg of their normal; the player figure's own eye is the config eye; the console hangs under the ceiling and the posts reach it."""
    import figure_layout as fl
    c = spec["cockpit"]
    cfg = spec["crew_figure"]
    scr = {s["id"]: s for s in c["screen"]}
    eye = c["eye"]["position_m"]
    s2 = next(s for s in c["seat"] if s["id"] == "s2")
    assert 0.60 <= math.dist(eye, scr["centre"]["centre_m"]) <= 0.75
    b = fl.bones(cfg)
    for side, sid in (("r", "centre"), ("l", "left")):
        shoulder = fl.to_capsule(b[f"upper_arm_{side}"]["head"], s2["hip_m"], s2["back_angle_deg"])
        d = math.dist(shoulder, scr[sid]["centre_m"])
        assert 0.62 <= d <= 0.75 and d < fl.arm_reach_m(cfg, side), (side, sid, d)
    fwd, n = c["eye"]["forward"], scr["centre"]["normal"]
    assert math.degrees(math.acos(-sum(fwd[i] * n[i] for i in range(3)))) < 30.0
    fig_eye = fl.to_capsule(fl.eye_point(cfg), s2["hip_m"], s2["back_angle_deg"])
    assert math.dist(fig_eye, eye) < 0.01
    assert c["console"]["upper_top_x_m"] < c["cabin"]["ceiling_x_m"] - 0.3


def test_key_labels_only_where_a_photo_shows_them_and_ids_stay(spec):
    """Task 5b: labels come from conv_42860a (the 2020 module row, same key layout as discovery10) / training2; every labelled key gets one atlas cell
    of its face's aspect, unlabelled keys none (blank faces); the 39 button ids and node names are unchanged (Task 8 uses button_node(id))."""
    import surface_textures
    cockpit = spec["cockpit"]
    labelled = {b["id"]: b["label"] for b in cockpit["button"] if b.get("label")}
    assert labelled["cam3"] == "DEORBIT NOW" and labelled["cam1"] == "CANCEL" and labelled["c_mains_only"] == "MAINS ONLY"
    assert "handle" not in labelled and "r_led_1" not in labelled and "c_upper_1" not in labelled  # unreadable: blank
    cells, (w, h) = cl.label_cells(cockpit)
    assert set(cells) == set(labelled) and w == cl.LABEL_ATLAS_WIDTH and h & (h - 1) == 0
    size = {b["id"]: b["size_m"] for b in cockpit["button"]}
    for bid, (u0, v0, u1, v1) in cells.items():
        assert 0.0 <= u0 < u1 <= 1.0 and 0.0 <= v0 < v1 <= 1.0
        assert abs((u1 - u0) * w / ((v1 - v0) * h) - size[bid][0] / size[bid][1]) < 0.02  # the cell has the key face's aspect
    m = cl.cockpit_manifest(cockpit)
    assert len(m["button_ids"]) == 39 and m["buttons"]["cam3"]["node"] == "button_cam3" and m["buttons"]["handle"]["node"] == "handle"
    assert m["buttons"]["cam3"]["label_cell"] == cells["cam3"] and "label_cell" not in m["buttons"]["r_led_1"]
    atlas = surface_textures.cockpit_labels(cockpit)
    assert atlas.shape == (h, w, 4)
    for bid, (u0, v0, u1, v1) in cells.items():  # every labelled cell has letters in it
        assert atlas[round(v0 * h):round(v1 * h), round(u0 * w):round(u1 * w), 0].max() > 200, bid


POST_END_TOL_M = 0.005  # [S] fidelity plan Task 4: the posts end on the ceiling ring surface within 5 mm
TILT_TOL_DEG = 0.5  # [S] fidelity plan Task 4: console tilt = tilt_deg +- 0.5 deg


def test_console_tilt_is_the_screens_tilt(spec):
    """tilt_deg = the angle of the console's (and every screen's) up axis from +X toward -Y; the housing's normal is the screens' normal."""
    c = spec["cockpit"]
    con = c["console"]
    for s in c["screen"] + [con]:
        up = s["up"]
        tilt = math.degrees(math.atan2(-up[1], up[0]))
        assert tilt == pytest.approx(con["tilt_deg"], abs=TILT_TOL_DEG), s.get("id", "console")
    frame = cl.console_frame(c)
    assert sum(a * b for a, b in zip(frame["normal"], con["normal"])) > math.cos(math.radians(TILT_TOL_DEG))


def test_console_posts_end_on_the_ceiling_ring(spec):
    """Both posts run from the housing's top (behind its back edge's fillet, inside its width) up to the flat ceiling ring around the tunnel."""
    c = spec["cockpit"]
    cab, con = c["cabin"], c["console"]
    ring_outer = cab["profile_m"][next(i for i, (x, r) in enumerate(cab["profile_m"]) if x == cab["ceiling_x_m"])][1]
    posts = cl.console_posts(c)
    assert len(posts) == 2
    frame = cl.console_frame(c)
    for base, top in posts:
        assert abs(top[0] - cab["ceiling_x_m"]) <= POST_END_TOL_M
        assert cab["tunnel_radius_m"] < math.hypot(top[1], top[2]) <= ring_outer
        rel = [b - o for b, o in zip(base, frame["origin"])]
        u = sum(a * b for a, b in zip(rel, frame["up"]))
        o = sum(a * b for a, b in zip(rel, frame["normal"]))
        lat = sum(a * b for a, b in zip(rel, frame["right"]))
        assert frame["u_top"] - con["edge_radius_m"] <= u <= frame["u_top"] + 1e-9, "the base sits on the housing top"
        assert -con["hood_depth_m"] < o < 0.0 and abs(lat) < con["width_m"] / 2.0
    assert posts[0][1][2] == pytest.approx(-posts[1][1][2]) and posts[0][1][1] == pytest.approx(posts[1][1][1]), "mirror pair"


def test_generated_posts_reach_the_ceiling(spec, nodes):
    c = spec["cockpit"]
    top_x = max(p[0] for p in nodes["console_back"]["pos"])
    assert abs(top_x - c["cabin"]["ceiling_x_m"]) <= POST_END_TOL_M


def _ring_outer(cab):
    return cab["profile_m"][next(i for i, (x, r) in enumerate(cab["profile_m"]) if x == cab["ceiling_x_m"])][1]


def test_post_collars_stay_inside_the_ceiling_ring(spec, nodes):
    """The joint heads (collars) where the posts enter the ceiling stay on the flat ring: their outer radius, not just the post's centre,
    lies inside the ring's outer radius (fix round 1: at post top r 0.981 + head 0.039 the collar poked ~2 cm through the wall)."""
    c = spec["cockpit"]
    cab, con = c["cabin"], c["console"]
    ring_outer = _ring_outer(cab)
    for _, top in cl.console_posts(c):
        assert math.hypot(top[1], top[2]) + con["post_head_radius_m"] <= ring_outer
    head = [p for p in nodes["console_back"]["pos"] if p[0] >= cab["ceiling_x_m"] - 2.0 * con["post_head_length_m"]]
    assert head, "the collars are in console_back"
    assert max(math.hypot(p[1], p[2]) for p in head) <= ring_outer + 1e-4  # float32 file


def test_console_frame_follows_tilt_deg(spec):
    """The generator's console axes come from tilt_deg; a tilt_deg that disagrees with the centre screen is refused."""
    c = spec["cockpit"]
    t = math.radians(c["console"]["tilt_deg"])
    frame = cl.console_frame(c)
    assert frame["up"] == pytest.approx([math.cos(t), -math.sin(t), 0.0], abs=1e-12)
    assert frame["normal"] == pytest.approx([-math.sin(t), -math.cos(t), 0.0], abs=1e-12)
    broken = json.loads(json.dumps(c))
    broken["console"]["tilt_deg"] += 2.0 * TILT_TOL_DEG
    with pytest.raises(ValueError):
        cl.console_frame(broken)


def test_generated_console_front_is_square_to_tilt_deg(spec, nodes):
    """The built housing's front face is one plane square to the tilt_deg normal: its vertices within 1 mm of the front plane span the
    body's width and more than 0.3 m up the console (a body tilted 1 deg off would fan 5 mm over that height)."""
    c = spec["cockpit"]
    frame = cl.console_frame(c)
    n, up, right = frame["normal"], frame["up"], frame["right"]
    pts = nodes["console_housing"]["pos"]
    # the body's front face lies SCREEN_PROUD (2 mm) behind the screen plane (Task 5: the screens' face frame stands recess_m in front of it)
    front = sum(a * b for a, b in zip(frame["origin"], n)) - cl.SCREEN_PROUD_M
    on = [p for p in pts if abs(sum(a * b for a, b in zip(p, n)) - front) < 1e-3]
    span_up = [sum(a * b for a, b in zip(p, up)) for p in on]
    span_lat = [sum(a * b for a, b in zip(p, right)) for p in on]
    assert max(span_up) - min(span_up) > 0.3
    assert max(span_lat) - min(span_lat) > c["console"]["width_m"] - 2.0 * c["console"]["edge_radius_m"] - 1e-3


# ---- fidelity Task 7: recess, console end taper, lockers, tablets ----
from cabin_fit.parts import lockers as lockers_fit  # noqa: E402
from cabin_fit.parts import recess as recess_fit  # noqa: E402


def test_recess_edges_lie_within_sigma_of_the_fit(spec):
    rec = spec["cockpit"]["recess"]
    assert rec["depth_m"] == recess_fit.PANEL_M and rec["x_ref_m"] == recess_fit.X_REF_M  # the fit back-projects onto the built floor
    fit = recess_fit.fit_recess(spec["cockpit"])
    for name in ("out", "in"):
        f = fit[name]
        for v, m, s in zip(rec[f"edge_{name}_deg"], f["coef"], f["sigma"]):
            assert abs(v - m) <= s, (name, v, f)
        assert rec[f"{name}_x_m"] == pytest.approx(list(f["x"]), abs=0.005), name
    # the built lines, projected into crew1_1, lie on the reads within a few px (read sigma 2 px, line rms 1-2 deg)
    built = recess_fit.recess_outline_px(spec["cockpit"])
    clean = recess_fit.in_edge_reads()  # the console band rows are the console's end, not the floor edge
    for key, reads in (("out_l", recess_fit.REVEAL_OUT_PX["l"]), ("out_r", recess_fit.REVEAL_OUT_PX["r"]),
                       ("in_l", clean["l"]), ("in_r", clean["r"])):
        rms, worst = recess_fit.line_residual_px(built[key], reads)
        assert rms <= 4.0 and worst <= 8.0, (key, rms, worst)


def test_recess_in_edge_fit_leaves_out_the_console_rows():
    """The crew1_1 dark line is the recess floor edge only where the console does not stand in front of it: the console band rows
    (CONSOLE_BAND_V) feed the end taper, never the in-edge fit, and the clean rows fit the mirrored wall line within the read sigma."""
    lo, hi = recess_fit.CONSOLE_BAND_V
    for s in ("l", "r"):
        assert all(not lo <= v <= hi for v in recess_fit.in_edge_reads()[s]), s
    fit = recess_fit.fit_recess()
    assert fit["in"]["rms_deg"] < 0.5
    for v, *_ in recess_fit.silhouette_samples():  # the taper's equality points are console-band rows of the silhouette side
        assert lo <= v <= hi


def test_recess_in_edge_must_lie_beyond_the_out_edge(spec):
    import copy
    broken = copy.deepcopy(spec["cockpit"])
    broken["recess"]["edge_in_deg"] = [100.0, 0.0, 0.0]  # in edge inside the panel edge (out ~107 deg)
    with pytest.raises(vehicle_spec.VehicleSpecError, match="in edge"):
        vehicle_spec.validate_cockpit(broken)
    with pytest.raises(ValueError, match="in edge"):
        cl.recess_offset(broken, 3.0, 110.0)


def test_generated_recess_follows_its_edges(spec, nodes):
    """cabin_recess: every vertex lies on the wall pushed out by cabin_layout.recess_offset (within 1 mm), the floor reaches depth_m, and the
    seam with the panels (vertices the recess shares with cabin_shell) lies at the out edge, within one fine column."""
    c = spec["cockpit"]
    rec = c["recess"]
    run = recess_fit.side_wall(c["cabin"])
    deepest = 0.0
    for x, y, z in nodes["cabin_recess"]["pos"]:
        az = math.degrees(math.atan2(z, y))
        off = math.hypot(y, z) - recess_fit.wall_r(run, x)
        assert off == pytest.approx(cl.recess_offset(c, x, az), abs=1e-3), (x, az)
        deepest = max(deepest, off)
    assert deepest == pytest.approx(rec["depth_m"], abs=1e-3)
    shell = {tuple(round(v * WELD) for v in p) for p in nodes["cabin_shell"]["pos"]}
    seam = [p for p in nodes["cabin_recess"]["pos"] if tuple(round(v * WELD) for v in p) in shell]
    assert seam
    for x, y, z in seam:  # within one fine column, plus what the edge moves over one fine row (a face spans two rows)
        a = abs(math.degrees(math.atan2(z, y)))
        edge = cl.recess_edge_az(rec, "out", x)
        slack = 0.8 + max(abs(cl.recess_edge_az(rec, "out", x + d) - edge) for d in (-0.021, 0.021))
        if rec["bottom_x_m"] + rec["bottom_bevel_m"] < x < c["cabin"]["ceiling_x_m"] - rec["top_bevel_m"]:
            assert edge - slack <= a <= edge + 1e-6, (x, a)


def test_console_end_taper_is_the_measured_contour(spec):
    """[console] end_taper: on the back face within sigma of the line fit of the console silhouette rows (right side, CONSOLE_BAND_V), an
    upper bound at every other contour read (left back-face rows, left underside rows: the console end lies at or inside the wall line), equal
    at the silver lip band end (|lat| 0.53); the width stays whole where the photos need it: the front zone under the outer glass edges
    (+-0.649) and the back face at mid height (end cap at 0.70-0.72)."""
    c = spec["cockpit"]
    (a, du), (sa, sdu), rms = recess_fit.fit_taper_back(recess_fit.silhouette_samples(c))
    w0, cdu, dd = c["console"]["end_taper"]
    hood = c["console"]["hood_depth_m"]
    assert abs(w0 + dd * hood - a) <= sa and abs(cdu - du) <= sdu, (a, du, sa, sdu)
    assert rms < 0.005
    for u, depth, w, s in recess_fit.bound_samples(c):  # the plane within 1 sigma; the built end (with the [E] front chamfer) within 2
        assert w0 + cdu * u + dd * depth <= w + s, (u, depth, w)
        assert cl.end_half_width(c, u, depth) <= w + 2.0 * s, (u, depth, w)
    assert cl.end_half_width(c, -0.11, 0.0) == pytest.approx(c["console"]["width_m"] / 2.0)
    assert cl.end_half_width(c, 0.0, c["console"]["hood_depth_m"]) == pytest.approx(c["console"]["width_m"] / 2.0)
    assert cl.end_half_width(c, -0.336, c["console"]["hood_depth_m"]) == pytest.approx(0.53, abs=0.01)


def test_generated_console_body_lies_inside_the_end_taper(spec, nodes):
    c = spec["cockpit"]
    worst = -1.0
    for node in ("console_back", "console_under"):
        for p in nodes[node]["pos"]:
            lat, u, o = cl.console_local(c, p)
            if o > -0.03 or abs(lat) < 0.3 or u > cl.console_frame(c)["u_top"] + 1e-3:  # the front zone / the posts
                continue
            worst = max(worst, abs(lat) - cl.end_half_width(c, u, -o))
    assert worst <= 0.002, worst


def test_lockers_are_the_measured_doors_with_their_latch(spec):
    c = spec["cockpit"]
    m = lockers_fit.measure()
    lat0, lat1, u0, u1 = m["door"]
    lk = next(item for item in c["locker"] if item["id"] == "5")
    lat, u, _ = cl.console_local(c, lk["centre_m"])
    assert lat == pytest.approx(0.5 * (lat0 + lat1), abs=m["sigma"]) and u == pytest.approx(0.5 * (u0 + u1), abs=m["sigma"])
    assert lk["size_m"] == pytest.approx([lat1 - lat0, u1 - u0], abs=m["sigma"])
    llat, lu, _ = cl.console_local(c, lk["latch_m"])
    assert (llat, lu) == pytest.approx(m["latch"], abs=m["sigma"])
    for item in c["locker"]:  # the latch and the placard (cabin_textures) lie on the door
        clat, cu, _ = cl.console_local(c, item["centre_m"])
        w, h = item["size_m"]
        llat, lu, _ = cl.console_local(c, item["latch_m"])
        assert abs(llat - clat) + item["latch_radius_m"] <= w / 2.0 + 1e-3 and abs(lu - cu) + item["latch_radius_m"] <= h / 2.0
    import cabin_textures
    for text, (x0, x1, y0, y1), _, _ in cabin_textures.PLACARDS:
        item = next(i for i in c["locker"] if i["id"] == text)
        clat, cu, _ = cl.console_local(c, item["centre_m"])
        w, h = item["size_m"]
        assert clat - w / 2.0 <= x0 and x1 <= clat + w / 2.0 and cu - h / 2.0 <= y0 and y1 <= cu + h / 2.0, text


def test_tablets_hang_on_their_arms(spec, nodes):
    """Each iPad's front face is its rolled body (cabin_layout.tablet_corners within 1 mm), and its arm runs from the mount behind it back
    into the upper panel's front (screen plane - SCREEN_PROUD_M), under the iPad's outline (hidden from the crew)."""
    c = spec["cockpit"]
    for t in c["tablet"]:
        pos = nodes[f"tablet_{t['id']}"]["pos"]
        n, t_up, t_right = cl.tablet_axes(t)
        rel = lambda p, a: cl.dot3([p[i] - t["centre_m"][i] for i in range(3)], a)
        front = max(cl.dot3(p, n) for p in pos)
        face = [p for p in pos if cl.dot3(p, n) > front - 1e-4]
        assert max(rel(p, t_right) for p in face) == pytest.approx(t["size_m"][0] / 2.0, abs=1e-3)
        assert max(rel(p, t_up) for p in face) == pytest.approx(t["size_m"][1] / 2.0, abs=1e-3)
        assert min(rel(p, t_up) for p in face) == pytest.approx(-t["size_m"][1] / 2.0, abs=1e-3)
        _, _, o_t = cl.console_local(c, t["centre_m"])
        back = min(cl.console_local(c, p)[2] for p in pos)
        assert back == pytest.approx(-cl.SCREEN_PROUD_M - 0.001, abs=0.002), back
        behind = [p for p in pos if cl.console_local(c, p)[2] < o_t - 0.01]
        assert behind
        for p in behind:
            assert abs(rel(p, t_right)) < t["size_m"][0] / 2.0 and abs(rel(p, t_up)) < t["size_m"][1] / 2.0, p
