"""Crew figure (gen_crew_figure.py -> crew_figure.gltf): the typed tables are tagged, the skeleton is the brief's bone set, the arm length
matches the [E] table, the figure seated on seat s2's hip point stays inside the seat envelope, the visor is its own closed surface, and
the glTF skeleton equals the layout (= the manifest block). Generated-model cases skip without generated Dragon models
(python tools/structures/build_models.py --vehicle crew_dragon)."""
import base64
import collections
import json
import math
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
import build_models  # noqa: E402
import figure_layout as fl  # noqa: E402
import vehicle_spec  # noqa: E402

ID = "crew_dragon"
MODELS = build_models.ROOT / "data" / "models" / ID
FIGURE = vehicle_spec.COCKPITS / "crew_figure.toml"
GLTF = MODELS / "crew_figure.gltf"
GENERATED = GLTF.is_file()
BRIEF_BONES = (["hips", "spine", "chest", "neck", "head"] + [f"{n}_{s}" for s in "lr" for n in (
    "shoulder", "upper_arm", "forearm", "hand", "index_1", "index_2", "index_3", "thumb_1", "thumb_2", "fingers", "thigh", "shin", "foot")])
ARM_TOL_M = 0.05  # [E] the Drillis and Contini fraction is a population mean; our bone chain is within this of it
ENVELOPE_TOL_M = 0.01  # [S] brief: nothing below the pan or behind the back by more than 1 cm
SEAT_PAN_SURFACE_M, SEAT_BACK_SURFACE_M = -0.01, -0.01  # seat boxes of gen_dragon_cabin.build_seat: pan top 0.01 under the hip point, back front face 0.01 behind it
SEAT_PAN_LENGTH_M, SEAT_BACK_HEIGHT_M = 0.44, 0.80  # gen_dragon_cabin SEAT_THIGH_M, 2 * SEAT_BACK_HALF_M[1]
EYE_TOL_M = 0.20  # [E] the config's eye is hip + 0.72 up + 0.10 forward (+-0.10 per axis); the figure's own eye point lies within this


@pytest.fixture(scope="module")
def spec():
    return vehicle_spec.load(ID)


@pytest.fixture(scope="module")
def cfg(spec):
    return spec["crew_figure"]


def _doc():
    doc = json.loads(GLTF.read_text(encoding="utf-8"))
    bufs = []
    for b in doc["buffers"]:
        uri = b["uri"]
        bufs.append(base64.b64decode(uri.split(",", 1)[1]) if uri.startswith("data:") else (GLTF.parent / uri).read_bytes())
    return doc, bufs


def _read(doc, bufs, i, comps):
    a = doc["accessors"][i]
    v = doc["bufferViews"][a["bufferView"]]
    off = v.get("byteOffset", 0) + a.get("byteOffset", 0)
    data = bufs[v["buffer"]]
    if comps == 1:
        fmt, size = {5123: ("<H", 2), 5125: ("<I", 4), 5121: ("<B", 1)}[a["componentType"]]
        return [struct.unpack_from(fmt, data, off + k * size)[0] for k in range(a["count"])]
    stride = v.get("byteStride", 4 * comps)
    return [struct.unpack_from(f"<{comps}f", data, off + k * stride) for k in range(a["count"])]


def _quat_matrix(q):
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def _node_world(doc):
    """{node index: (3x3 rotation, translation)} with the scale ignored (bones carry none)."""
    parent = {}
    for i, n in enumerate(doc["nodes"]):
        for c in n.get("children", []):
            parent[c] = i
    cache = {}

    def world(i):
        if i in cache:
            return cache[i]
        n = doc["nodes"][i]
        r = _quat_matrix(n.get("rotation", [0, 0, 0, 1]))
        t = n.get("translation", [0, 0, 0])
        if i in parent:
            pr, pt = world(parent[i])
            t = [pt[k] + sum(pr[k][j] * t[j] for j in range(3)) for k in range(3)]
            r = [[sum(pr[a][k] * r[k][b] for k in range(3)) for b in range(3)] for a in range(3)]
        cache[i] = (r, t)
        return cache[i]
    return {i: world(i) for i in range(len(doc["nodes"]))}


def _mesh():
    """(doc, [(material name, positions, triangle index list)]) of the skinned mesh node `crew_figure`."""
    doc, bufs = _doc()
    node = next(n for n in doc["nodes"] if n.get("name") == "crew_figure")
    prims = []
    for p in doc["meshes"][node["mesh"]]["primitives"]:
        pos = _read(doc, bufs, p["attributes"]["POSITION"], 3)
        idx = _read(doc, bufs, p["indices"], 1)
        prims.append((doc["materials"][p["material"]]["name"], pos, idx))
    return doc, node, prims


def test_figure_tables_are_tagged_and_load_into_the_spec(spec, cfg):
    assert vehicle_spec.value_lines_without_tag(FIGURE) == []
    assert set(cfg["material"]) == set(fl.MATERIALS)
    assert cfg["figure"]["stature_m"] == pytest.approx(1.78)


def test_validation_names_a_missing_key(cfg):
    broken = json.loads(json.dumps(cfg))
    del broken["hand"]["thumb_m"]
    with pytest.raises(vehicle_spec.VehicleSpecError):
        vehicle_spec.validate_crew_figure(broken)


def test_bone_set_is_the_brief_list_and_parents_close(cfg):
    b = fl.bones(cfg)
    assert sorted(b) == sorted(BRIEF_BONES)
    assert [n for n in b if b[n]["parent"] is None] == ["hips"]
    for name, bone in b.items():
        assert bone["parent"] is None or bone["parent"] in b
        assert fl.length(fl.sub(bone["tail"], bone["head"])) > 0.01, name


@pytest.mark.parametrize("side", ["l", "r"])
def test_arm_length_shoulder_to_index_tip_matches_the_table(cfg, side):
    table = cfg["figure"]["arm_reach_fraction"] * cfg["figure"]["stature_m"]
    assert fl.arm_reach_m(cfg, side) == pytest.approx(table, abs=ARM_TOL_M)


def test_left_is_plus_z_and_the_sides_mirror(cfg):
    b = fl.bones(cfg)
    for name in ("upper_arm", "thigh", "shin", "forearm"):
        assert b[f"{name}_l"]["head"][2] > 0.0 > b[f"{name}_r"]["head"][2]
        assert b[f"{name}_l"]["head"][2] == pytest.approx(-b[f"{name}_r"]["head"][2])
    assert b["thumb_1_l"]["head"][2] < b["fingers_l"]["head"][2] + 0.05  # the thumb root lies on the inner side (toward the body centre)


def _pan_coords(cfg, p):
    """(along the pan, above the pan surface) of a figure-frame point: the seat pan follows the thighs (thigh_raise_deg about the hip point)."""
    r = math.radians(cfg["figure"]["thigh_raise_deg"])
    return p[0] * math.sin(r) + p[1] * math.cos(r), p[0] * math.cos(r) - p[1] * math.sin(r)


def test_bones_stay_inside_the_seat_envelope(cfg):
    for name, bone in fl.bones(cfg).items():
        for p in (bone["head"], bone["tail"]):
            along, up = _pan_coords(cfg, p)
            on_pan = 0.0 <= along <= SEAT_PAN_LENGTH_M and abs(p[2]) <= 0.29
            assert not on_pan or up >= SEAT_PAN_SURFACE_M, (name, p)
            assert p[1] >= SEAT_BACK_SURFACE_M or p[0] > SEAT_BACK_HEIGHT_M, (name, p)


def test_eye_is_near_the_config_eye_of_seat_s2(spec, cfg):
    s2 = next(s for s in spec["cockpit"]["seat"] if s["id"] == "s2")
    eye = fl.to_capsule(fl.eye_point(cfg), s2["hip_m"], s2["back_angle_deg"])
    assert fl.length(fl.sub(eye, spec["cockpit"]["eye"]["position_m"])) < EYE_TOL_M


def test_manifest_block_has_the_frame_bones_and_visor(cfg):
    m = fl.figure_manifest(cfg)
    assert sorted(m["bones"]) == sorted(BRIEF_BONES) and m["visor_material"] == "visor"
    assert "x = up the seat back" in m["frame"] and "crew's left" in m["frame"]
    assert m["bones"]["hips"]["parent"] == "" and m["bones"]["upper_arm_l"]["parent"] == "shoulder_l"
    assert m["bones"]["hips"]["head"] == [round(c, 5) for c in fl.bones(cfg)["hips"]["head"]]


def test_seat_basis_is_right_handed_and_places_the_head_toward_the_nose(spec):
    s2 = next(s for s in spec["cockpit"]["seat"] if s["id"] == "s2")
    x, y, z = fl.seat_basis(s2["back_angle_deg"])
    assert fl.cross(x, y) == pytest.approx(z)
    assert x[0] > 0.0 > x[1]  # up the back = toward the nose and away from the hatch side (config: 35 deg from +X toward -Y)


# ---------- generated model ----------

@pytest.fixture(scope="module")
def mesh():
    if not GENERATED:
        pytest.skip("no generated Dragon models")
    return _mesh()


def test_gltf_has_one_skinned_mesh_and_the_bones(cfg, mesh):
    doc, node, prims = mesh
    assert "skin" in node and len([n for n in doc["nodes"] if "mesh" in n]) == 1
    joints = {doc["nodes"][j]["name"] for j in doc["skins"][node["skin"]]["joints"]}
    assert joints == set(BRIEF_BONES)


def test_gltf_skeleton_equals_the_layout(cfg, mesh):
    doc, node, _ = mesh
    world = _node_world(doc)
    b = fl.bones(cfg)
    for j in doc["skins"][node["skin"]]["joints"]:
        name = doc["nodes"][j]["name"]
        got = world[j][1]
        assert fl.length(fl.sub(tuple(got), b[name]["head"])) < 1e-3, (name, got, b[name]["head"])


def test_materials_and_visor_is_its_own_closed_surface(mesh):
    _, _, prims = mesh
    names = [p[0] for p in prims]
    assert sorted(names) == sorted(fl.MATERIALS) and names.count("visor") == 1
    _, pos, idx = next(p for p in prims if p[0] == "visor")
    weld = {}
    tris = [[weld.setdefault(tuple(round(c * 1e4) for c in pos[i]), len(weld)) for i in idx[k:k + 3]] for k in range(0, len(idx), 3)]
    edges = collections.Counter()
    for t in tris:
        for a, b in ((0, 1), (1, 2), (2, 0)):
            edges[(min(t[a], t[b]), max(t[a], t[b]))] += 1
    assert tris and set(edges.values()) == {2}, collections.Counter(edges.values())


def test_visor_is_in_front_of_the_head_and_copper(cfg, mesh):
    _, _, prims = mesh
    _, pos, _ = next(p for p in prims if p[0] == "visor")
    cy = sum(p[1] for p in pos) / len(pos)
    assert cy > cfg["pose"]["head_centre_forward_m"] + 0.05  # forward of the helmet centre
    assert cfg["material"]["visor"]["albedo"][0] > cfg["material"]["visor"]["albedo"][2] * 2.0  # copper: red over blue


def test_mesh_fits_the_seat_envelope_on_the_hip_point(cfg, mesh):
    _, _, prims = mesh
    worst_pan, worst_back = 0.0, 0.0
    for _, pos, _ in prims:
        for x, y, z in pos:
            along, up = _pan_coords(cfg, (x, y, z))
            if 0.0 <= along <= SEAT_PAN_LENGTH_M and abs(z) <= 0.29:
                worst_pan = min(worst_pan, up - SEAT_PAN_SURFACE_M)
            if 0.0 <= x <= SEAT_BACK_HEIGHT_M and abs(z) <= 0.29:
                worst_back = min(worst_back, y - SEAT_BACK_SURFACE_M)
    assert worst_pan >= -ENVELOPE_TOL_M, worst_pan
    assert worst_back >= -ENVELOPE_TOL_M, worst_back


def test_stature_and_triangle_budget(cfg, mesh):
    _, _, prims = mesh
    tris = sum(len(p[2]) // 3 for p in prims)
    assert 2000 < tris < 20000
    top = max(p[0] for _, pos, _ in prims for p in pos)
    assert 0.85 < top < 1.1  # crown of the helmet above the seat's hip point


def _shell_extent(h):
    """(rear y, front y, crown x, chin bottom x, rear lower-edge x) of the helmet shell in the helmet-centre frame (figure_layout.helmet_*)."""
    sag = [fl.helmet_point(h, el, az)[0] for el in range(-89, 90) for az in (0.0, 180.0)]
    sag = [p for p in sag if p[0] >= fl.helmet_cut_x(h, p[1]) - 1e-9]
    rear_edge = fl.helmet_point(h, fl.helmet_bottom_el(h, 180.0), 180.0)[0][0]
    return min(p[1] for p in sag), max(p[1] for p in sag), max(p[0] for p in sag), min(p[0] for p in sag), rear_edge


def test_helmet_shell_is_compact_and_faceted_after_the_spacex_profile(cfg):
    """Task 5b (replaces round 4's egg / crown-ridge bounds): the side profile of spacex-anzug.jpg at the opening-height ruler (1142 px/m) is
    0.28 m deep (rear face to the visor front) and 0.33 m tall (crown to the chin), so the shell is TALLER than deep (round 4: 0.375 deep,
    'bulbous'); the crown slopes back over a flat upper rear facet to a flat rear face; the lower edge rises from the chin to the rear by
    ~0.16 m (the grey hood shows under it); the opening (the ruler: round-4 height 0.232 m) is wide at the temple and narrows to the chin."""
    h = cfg["helmet"]
    rear, front, crown, chin, rear_edge = _shell_extent(h)
    depth = front + h["visor_proud_m"] - rear
    height = crown - chin
    assert 0.25 <= depth <= 0.31, depth
    assert 0.30 <= height <= 0.36, height
    assert depth < 0.95 * height  # compact: taller than deep
    assert abs(rear + h["rear_flat_m"]) < 1e-6  # the flat rear face is the rearmost part
    assert rear_edge - chin >= 0.12  # the sloped lower edge: high at the rear, low at the chin
    facets = {fl.helmet_facet(h, fl.helmet_direction(el, az)) for el in range(-20, 80, 5) for az in range(0, 360, 10)}
    assert {1, 2, 3, 4} <= facets  # rear face, both sides and the upper rear facet all exist
    e0, e1 = fl.visor_el_range(h)
    p0 = fl.helmet_point(h, e0, 0.0)[0]
    p1 = fl.helmet_point(h, e1, 0.0)[0]
    assert 0.20 <= math.dist(p0, p1) <= 0.26  # opening height at the ruler (round 4: 0.232 m)
    widest = max(row[1] for row in h["visor_outline"])
    assert 80.0 <= widest <= 95.0 and h["visor_outline"][0][1] <= 20.0 and h["visor_outline"][-1][1] <= 20.0
    assert [row[0] for row in h["visor_outline"]] == sorted(row[0] for row in h["visor_outline"])
    assert e0 > fl.helmet_bottom_el(h, 0.0)  # the chin tip of the opening stays on the shell above the lower edge
    assert len(h["grooves"]) >= 1 and all(len(row) == 8 for row in h["grooves"])
    block = fl.figure_manifest(cfg)["helmet_grooves"]
    assert len(block["planes"]) == len(h["grooves"]) and block["radius_m"] > 0.15


def test_visor_sits_in_a_face_opening_with_a_white_shell_around_it(cfg, mesh):
    """Helmet details (round 5 / Task 5b): a thin white frame bead (spacex-anzug, jsc2024e066389: 15-22 mm; round 4's 30-45 mm band was the
    bulbous shell's) with a black seal inside it, the closed visor domed in the opening, round pivot buttons behind the opening at jaw height,
    the neck hood a distinct grey part wider than the neck."""
    h = cfg["helmet"]
    assert 0.015 <= 2.0 * h["rim_radius_m"] <= 0.022 and 0.0 < h["liner_radius_m"] < h["rim_radius_m"]
    assert 0.007 <= h["pivot_button_radius_m"] <= 0.013 and 0.0 < h["pivot_button_height_m"] <= 0.006
    assert -40.0 <= h["pivot_el_deg"] <= -20.0 and h["pivot_az_offset_deg"] > 0.0
    assert 0.0 < h["egg_taper"] < 0.2 and 0.3 <= h["visor_tint_split"] <= 0.7
    assert cfg["material"]["collar"]["albedo"][0] < 0.5 * cfg["material"]["helmet"]["albedo"][0]  # grey hood against the white shell
    assert h["visor_proud_m"] > h["visor_edge_m"]
    assert h["collar_radius_m"] > cfg["suit"]["neck_radius_m"] * 1.25
    _, _, prims = mesh
    _, pos, _ = next(p for p in prims if p[0] == "visor")
    front = max(p[1] for p in pos) - cfg["pose"]["head_centre_forward_m"]
    assert front >= h["half_depth_front_m"] + 0.5 * h["visor_proud_m"]  # the dome leaves the shell's front
    zs = [p[2] for p in pos]
    assert max(zs) - min(zs) <= 2.0 * (h["side_flat_m"] + h["visor_proud_m"])  # the visor stays on the shell (it wraps to the temple, not past the sides)


def test_visor_is_its_own_surface_and_the_helmet_is_neutral_white(cfg, mesh):
    _, _, prims = mesh
    names = [p[0] for p in prims]
    assert names.count("visor") == 1 and names.count("helmet") == 1
    assert max(cfg["material"]["helmet"]["albedo"]) - min(cfg["material"]["helmet"]["albedo"]) < 0.02  # no blue tint
