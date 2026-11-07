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
                     "tablet_l", "tablet_r", "screen_left", "screen_centre", "screen_right", "seat_s1", "seat_s4", "light_0", "light_1",
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
