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

import numpy as np
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
    "shoulder", "upper_arm", "forearm", "hand", "index_1", "index_2", "index_3", "thumb_1", "thumb_2", "fingers", "fingers_2", "fingers_3", "thigh", "shin", "foot")])
ARM_TOL_M = 0.05  # [E] the Drillis and Contini fraction is a population mean; our bone chain is within this of it
ENVELOPE_TOL_M = 0.01  # [S] brief: nothing below the pan or behind the back by more than 1 cm
SEAT_PAN_SURFACE_M, SEAT_BACK_SURFACE_M = -0.01, -0.01  # seat boxes of gen_dragon_cabin.build_seat: pan top 0.01 under the hip point, back front face 0.01 behind it
SEAT_PAN_LENGTH_M, SEAT_BACK_HEIGHT_M = 0.44, 0.80  # gen_dragon_cabin SEAT_THIGH_M, 2 * SEAT_BACK_HALF_M[1]
EYE_TOL_M = 0.01  # [S] Task 8b eye contract: the figure's eye on the seat s2 hip point IS the config eye


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
    """(doc, suit node, [(material name, positions, triangle index list)]) of the skinned mesh nodes `crew_figure` (the suit) and
    `crew_figure_parts` (the rigid parts)."""
    doc, bufs = _doc()
    node = next(n for n in doc["nodes"] if n.get("name") == "crew_figure")
    parts = next(n for n in doc["nodes"] if n.get("name") == "crew_figure_parts")
    prims = []
    for p in doc["meshes"][node["mesh"]]["primitives"] + doc["meshes"][parts["mesh"]]["primitives"]:
        pos = _read(doc, bufs, p["attributes"]["POSITION"], 3)
        idx = _read(doc, bufs, p["indices"], 1)
        prims.append((doc["materials"][p["material"]]["name"], pos, idx))
    return doc, node, prims


def test_figure_tables_are_tagged_and_load_into_the_spec(spec, cfg):
    assert vehicle_spec.value_lines_without_tag(FIGURE) == []
    assert set(cfg["material"]) == set(fl.MATERIALS)
    assert cfg["figure"]["stature_m"] == pytest.approx(1.755)  # ANSUR II male median


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


def test_gltf_has_the_suit_and_the_parts_mesh_on_one_skeleton(cfg, mesh):
    doc, node, prims = mesh
    meshed = [n for n in doc["nodes"] if "mesh" in n]
    assert sorted(n["name"] for n in meshed) == ["crew_figure", "crew_figure_parts"] and all("skin" in n for n in meshed)
    joints = {doc["nodes"][j]["name"] for j in doc["skins"][node["skin"]]["joints"]}
    assert joints == set(BRIEF_BONES)
    for n in meshed:
        assert {doc["nodes"][j]["name"] for j in doc["skins"][n["skin"]]["joints"]} <= set(BRIEF_BONES)


def test_morph_targets_only_on_the_suit(mesh):
    """Fix round 1: the 16 corrective shapes move only the suit; the rigid parts' mesh carries none (asset size, GPU morph work)."""
    doc, node, _ = mesh
    parts = next(n for n in doc["nodes"] if n.get("name") == "crew_figure_parts")
    assert all("targets" not in p for p in doc["meshes"][parts["mesh"]]["primitives"])
    suit_prims = doc["meshes"][node["mesh"]]["primitives"]
    assert [doc["materials"][p["material"]]["name"] for p in suit_prims] == ["suit"]
    assert (MODELS / "crew_figure.bin").stat().st_size < 6_000_000


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
    assert 10000 < tris < 32000  # Task 8b budget: ~27.5k per figure (suit 14.2k, helmet and visor ~9k, gloves, boots), 4 figures ~110k
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


# ---------- Task 8b: anatomical body (ANSUR II), suit hull, joint folds ----------
ANSUR_TOL_M = 0.01  # [S] brief: the body's measures within 1 cm of the ANSUR II medians
VOLUME_TOL = 0.01  # [E] a bent joint keeps the limb's volume within 1 % of the rest pose (corrective shapes)
ANSUR_CSV = build_models.ROOT / "data" / "anthropometry" / "ANSUR_II_MALE_Public.csv"
ANSUR_COLUMNS = {"acromial_height": "acromialheight", "axilla_height": "axillaheight", "chest_height": "chestheight", "waist_height": "waistheightomphalion",
                 "trochanter_height": "trochanterionheight", "crotch_height": "crotchheight", "knee_height": "lateralfemoralepicondyleheight",
                 "ankle_height": "lateralmalleolusheight", "cervicale_height": "cervicaleheight", "biacromial_breadth": "biacromialbreadth",
                 "bideltoid_breadth": "bideltoidbreadth", "chest_breadth": "chestbreadth", "chest_depth": "chestdepth", "chest_circumference": "chestcircumference",
                 "waist_breadth": "waistbreadth", "waist_depth": "waistdepth", "waist_circumference": "waistcircumference", "hip_breadth": "hipbreadth",
                 "buttock_depth": "buttockdepth", "buttock_circumference": "buttockcircumference", "thigh_circumference": "thighcircumference",
                 "lower_thigh_circumference": "lowerthighcircumference", "calf_circumference": "calfcircumference", "ankle_circumference": "anklecircumference",
                 "wrist_circumference": "wristcircumference", "neck_circumference_base": "neckcircumferencebase", "upper_arm_length": "acromionradialelength",
                 "forearm_length": "radialestylionlength", "hand_length": "handlength", "hand_breadth": "handbreadth", "palm_length": "palmlength",
                 "foot_length": "footlength", "thigh_clearance": "thighclearance"}


@pytest.fixture(scope="module")
def body(cfg):
    import figure_body as fb
    return fb.surface(cfg, ease=False)[0]


@pytest.fixture(scope="module")
def suit(cfg):
    import figure_body as fb
    return fb.suit_mesh(cfg)


def test_anthropometry_rows_are_the_ansur_medians(cfg):
    """[anthropometry] = the male medians of the ANSUR II public database (skips without data/anthropometry: fetch.py ansur2_male)."""
    if not ANSUR_CSV.is_file():
        pytest.skip("ANSUR II CSV not fetched (python tools/data_pipeline/fetch.py ansur2_male)")
    import csv
    import statistics
    rows = list(csv.DictReader(ANSUR_CSV.open(encoding="latin-1")))
    assert len(rows) == 4082
    for key, column in ANSUR_COLUMNS.items():
        median = statistics.median(float(r[column]) for r in rows) / 1000.0
        assert cfg["anthropometry"][key] == pytest.approx(median, abs=5e-4), key
    assert cfg["figure"]["stature_m"] == pytest.approx(statistics.median(float(r["stature"]) for r in rows) / 1000.0, abs=5e-4)


def test_body_meets_the_ansur_measures_within_1_cm(cfg, body):
    """Tape measures on the subdivided anatomical body (no suit): torso girths, breadths, depths at the ANSUR levels, limb girths,
    segment lengths of the skeleton."""
    import figure_measure as fm
    a = cfg["anthropometry"]
    p = body.positions()
    t0 = a["trochanter_height"]
    for level, girth, breadth, depth in (("buttock", "buttock_circumference", "hip_breadth", "buttock_depth"),
                                         ("waist", "waist_circumference", "waist_breadth", "waist_depth"),
                                         ("chest", "chest_circumference", "chest_breadth", "chest_depth")):
        s = 0.0 if level == "buttock" else a[f"{level}_height"] - t0
        g, b, d = fm.torso(body, p, cfg, s)
        assert g == pytest.approx(a[girth], abs=ANSUR_TOL_M), (level, g)
        assert b == pytest.approx(a[breadth], abs=ANSUR_TOL_M), (level, b)
        assert d == pytest.approx(a[depth], abs=ANSUR_TOL_M), (level, d)
    assert fm.torso(body, p, cfg, a["cervicale_height"] - t0)[0] == pytest.approx(a["neck_circumference_base"], abs=ANSUR_TOL_M)
    ln = fl.chain_lengths(cfg)
    for part, t, key in (("leg_l", 0.10, "thigh_circumference"), ("leg_r", ln["thigh"] - 0.03, "lower_thigh_circumference"),
                         ("leg_l", 0.53, "calf_circumference"), ("leg_r", ln["thigh"] + ln["shin"] - 0.026, "ankle_circumference"),
                         ("arm_l", ln["upper_arm"] + ln["forearm"], "wrist_circumference"), ("arm_r", ln["upper_arm"] + ln["forearm"], "wrist_circumference")):
        assert fm.limb(cfg, body, p, part, t) == pytest.approx(a[key], abs=ANSUR_TOL_M), (part, key)
    b = fl.bones(cfg)
    seg = lambda n: fl.length(fl.sub(b[n]["tail"], b[n]["head"]))
    for bone, key in (("upper_arm_l", "upper_arm_length"), ("forearm_r", "forearm_length")):
        assert seg(bone) == pytest.approx(a[key], abs=1e-6)
    assert seg("thigh_l") == pytest.approx(a["trochanter_height"] - a["knee_height"], abs=1e-6)
    assert seg("shin_r") == pytest.approx(a["knee_height"] - a["ankle_height"], abs=1e-6)
    assert 2.0 * abs(b["upper_arm_l"]["head"][2]) == pytest.approx(a["biacromial_breadth"], abs=ANSUR_TOL_M)
    assert seg("chest") + (b["chest"]["head"][0] - b["hips"]["head"][0]) == pytest.approx(a["acromial_height"] - t0, abs=0.01)


def test_bideltoid_breadth_with_the_arms_hanging(cfg, suit):
    """ANSUR bideltoid breadth across the shoulder caps (arms hanging parallel to the torso), from the acromion down to the axilla level."""
    import figure_body as fb
    m = fb.suit_mesh(cfg, ease=False)
    p = fb.posed(m, cfg, {"shoulder_l": 0.0, "shoulder_r": 0.0})
    a = cfg["anthropometry"]
    top = cfg["figure"]["hip_joint_up_m"] + fl.chain_lengths(cfg)["torso"]
    band = (p[:, 0] > top - (a["acromial_height"] - a["axilla_height"])) & (p[:, 0] < top + 0.02)
    assert 2.0 * abs(p[band, 2]).max() == pytest.approx(a["bideltoid_breadth"], abs=ANSUR_TOL_M)


def test_eye_contract_the_commander_sees_from_the_config_eye(spec, cfg):
    """The player's figure puts its eye at CockpitConfig.eye: the seat s2 hip point plus the figure's eye point = [eye] position_m."""
    assert fl.eye_point(cfg) == pytest.approx((0.83, 0.23, 0.0))
    s2 = next(s for s in spec["cockpit"]["seat"] if s["id"] == "s2")
    eye = fl.to_capsule(fl.eye_point(cfg), s2["hip_m"], s2["back_angle_deg"])
    assert math.dist(eye, spec["cockpit"]["eye"]["position_m"]) < 0.01


def test_suit_is_its_own_hull_outside_the_body(cfg, body):
    """The suit = the body sections plus the ease: loose at the thigh and upper arm, tight at the forearm and shank."""
    import figure_body as fb
    import figure_measure as fm
    s_hull = fb.surface(cfg, ease=True)[0]
    ps, pb = s_hull.positions(), body.positions()
    e = cfg["suit"]
    assert e["ease_thigh_m"] > e["ease_shank_m"] and e["ease_upper_arm_m"] > e["ease_forearm_m"]
    for part, t, ease in (("leg_l", 0.22, e["ease_thigh_m"]), ("leg_l", 0.53, e["ease_shank_m"]), ("arm_l", 0.20, e["ease_upper_arm_m"]),
                          ("arm_l", 0.50, e["ease_forearm_m"])):
        extra = fm.limb(cfg, s_hull, ps, part, t) - fm.limb(cfg, body, pb, part, t)
        assert extra == pytest.approx(2.0 * math.pi * ease, rel=0.35), (part, t, extra)
    g_suit = fm.torso(s_hull, ps, cfg, 0.156)[0]
    assert g_suit > fm.torso(body, pb, cfg, 0.156)[0] + 2.0 * math.pi * e["ease_torso_m"] * 0.6


def test_topology_ports_and_closed_hull(suit):
    """Base-mesh layout: the arm loop cut out of the torso side (axilla to shoulder top), the legs on one crotch split; the hull is closed."""
    h = suit["hull"]
    edges = collections.Counter()
    for f in h.faces:
        for k in range(len(f)):
            a, b = f[k], f[(k + 1) % len(f)]
            edges[(min(a, b), max(a, b))] += 1
    assert set(edges.values()) == {2}
    assert set(h.ports) == {"arm_l", "arm_r", "leg_l", "leg_r"}
    assert len(h.ports["arm_l"]) == len(h.ports["arm_r"]) and len(h.ports["leg_l"]) == len(h.ports["leg_r"])


def test_weights_are_smooth_normalised_and_at_most_four(suit):
    w = suit["w"]
    assert np.allclose(w.sum(axis=1), 1.0, atol=1e-6)
    assert ((w > 1e-4).sum(axis=1) <= 4).all()
    names = suit["names"]
    blended = ((w[:, names.index("forearm_l")] > 0.05) & (w[:, names.index("upper_arm_l")] > 0.05)).sum()
    assert blended >= 30  # the elbow blends two bones over its loops (no rigid crease)


@pytest.mark.parametrize("part,joint,theta", [("arm_l", "elbow_l", 90.0), ("arm_r", "elbow_r", 0.0), ("arm_l", "elbow_l", 140.0),
                                              ("leg_l", "knee_l", 90.0), ("leg_r", "knee_r", 135.0), ("arm_r", "shoulder_r", 120.0)])
def test_joint_volume_is_kept_by_the_corrective_shapes(cfg, suit, part, joint, theta):
    """A 90 deg elbow / knee and a 120 deg shoulder raise keep the limb's volume (LBS of rest + the corrective shapes, as the game skins it)."""
    import figure_body as fb
    h, ref = suit["hull"], suit["refs"][part]
    v_rest = fb.part_volume(h, suit["rest"], part, ref)
    v = fb.part_volume(h, fb.posed(suit, cfg, {joint: theta}), part, ref)
    assert abs(v / v_rest - 1.0) < VOLUME_TOL, (joint, theta, v / v_rest)


def test_plain_linear_skinning_would_lose_the_shoulder(cfg, suit):
    """Why the shapes exist: plain LBS changes the arm's volume by more than 2 % at the 120 deg raise (the corrected pose keeps it within 1 %)."""
    import figure_body as fb
    h, ref = suit["hull"], suit["refs"]["arm_l"]
    v_rest = fb.part_volume(h, suit["rest"], "arm_l", ref)
    assert abs(fb.part_volume(h, fb.posed(suit, cfg, {"shoulder_l": 120.0}, corrected=False), "arm_l", ref) / v_rest - 1.0) > 0.02


def test_wrinkle_weight_is_0_straight_and_1_at_full_bend(cfg):
    for name, joint in fl.joint_table(cfg).items():
        assert fl.bend_weights(joint, joint["neutral_deg"])[2] == 0.0, name
        assert fl.bend_weights(joint, joint["full_deg"])[2] == 1.0, name
        flex, ext, wr = fl.bend_weights(joint, joint["rest_deg"])
        assert flex == 0.0 and ext == 0.0 and 0.0 <= wr < 1.0, name
        assert fl.bend_weights(joint, joint["full_deg"])[0] == 1.0
    assert sorted(j["slot"] for j in fl.joint_table(cfg).values()) == list(range(8))


def test_swing_shapes_act_only_near_their_baked_direction(cfg):
    """Fix round 1 (pinched shoulder at the HOTAS grip): a swing joint's (shoulder, hip) flex shape is baked for one swing direction
    (`plane`, perpendicular to the reference); its gate is 1 there, 0 for a sideways (90 deg) or backward swing; hinges are not gated."""
    for name, joint in fl.joint_table(cfg).items():
        ref = fl.mul(fl.norm(fl.sub(fl.bones(cfg)[joint["ref"]]["tail"], fl.bones(cfg)[joint["ref"]]["head"])), joint["ref_sign"])
        assert abs(fl.dot(joint["plane"], ref)) < 1e-9 and fl.length(joint["plane"]) == pytest.approx(1.0), name
        if name.startswith(("shoulder", "hip")):
            assert joint["gated"], name
            assert fl.swing_gate(joint, 1.0) == 1.0 and fl.swing_gate(joint, math.cos(math.radians(20.0))) == 1.0
            assert fl.swing_gate(joint, 0.0) == 0.0 and fl.swing_gate(joint, -1.0) == 0.0
            assert 0.0 < fl.swing_gate(joint, math.cos(math.radians(45.0))) < 1.0
        else:
            assert not joint["gated"] and fl.swing_gate(joint, -1.0) == 1.0, name


def test_torso_uv_runs_along_the_arc(cfg):
    """Fix round 1: u around the torso is the arc-length share of each ring (the painted panels keep their width in metres on the
    near-box sections); along one ring du / edge length is constant."""
    import figure_body as fb
    h, layout = fb.hull(cfg, ease=True)
    tu = layout["parts"]["torso"]["u_width"]
    rows = collections.defaultdict(list)
    for f, uv, part in zip(h.faces, h.uv, h.part):
        if part != "torso" or len(f) != 4:
            continue
        for a, b in ((0, 1), (3, 2)):
            if abs(uv[a][1] - uv[b][1]) < 1e-12 and abs(uv[a][0] - uv[b][0]) < 0.5 * tu:
                rows[round(uv[a][1], 6)].append(abs(uv[b][0] - uv[a][0]) / np.linalg.norm(h.v[f[b]] - h.v[f[a]]))
    assert len(rows) > 10
    for v, ratios in rows.items():
        assert max(ratios) / min(ratios) < 1.0 + 1e-6, (v, min(ratios), max(ratios))


def test_no_nan_in_the_mesh_shapes_and_poses(cfg, suit):
    import figure_body as fb
    assert np.isfinite(suit["rest"]).all()
    assert all(np.isfinite(d).all() for d in suit["shapes"].values()) and len(suit["shapes"]) == 16
    p = fb.posed(suit, cfg, {"elbow_l": 140.0, "knee_r": 135.0, "shoulder_r": 130.0, "hip_l": 110.0, "elbow_r": 0.0})
    assert np.isfinite(p).all()


def test_seated_rest_pose_has_the_hands_on_the_thighs(cfg):
    """KSC-20190731 / crew1_1: the hands rest palm down on the thighs, the elbows beside the torso."""
    b = fl.bones(cfg)
    td, n = fl.thigh_frame(cfg)
    for side in "lr":
        palm = fl.add(b[f"hand_{side}"]["head"], fl.mul(fl.sub(b[f"hand_{side}"]["tail"], b[f"hand_{side}"]["head"]), 0.5))
        hip = b[f"thigh_{side}"]["head"]
        rel = fl.sub(palm, hip)
        along, up = fl.dot(rel, td), fl.dot(rel, n)
        assert 0.15 < along < fl.chain_lengths(cfg)["thigh"], (side, along)
        assert 0.08 < up < 0.16, (side, up)  # on the thigh's top surface (half depth ~0.1 with the suit) plus the glove
        assert abs(b[f"forearm_{side}"]["head"][2]) > abs(b[f"upper_arm_{side}"]["head"][2])  # the elbow beside the torso


def test_suit_maps_are_baked_on_the_atlas(cfg):
    import suit_textures
    import figure_layout
    alb = suit_textures.albedo(cfg)
    wr = suit_textures.wrinkle(cfg)
    look = cfg["suit_look"]
    assert alb.shape[:2] == (round(look["atlas_m"][1] / look["texel_m"]), round(look["atlas_m"][0] / look["texel_m"]))
    assert np.isfinite(alb).all() and alb.min() >= 0.0 and alb.max() <= 1.0
    assert np.isclose(alb, cfg["material"]["suit"]["albedo"]).all(axis=2).mean() > 0.6  # mostly white suit
    assert (np.abs(alb - np.array(look["stripe_albedo"])).max(axis=2) < 1e-3).mean() > 0.01  # the grey panels exist
    assert np.isfinite(wr).all() and np.abs(wr[..., :2]).max() < figure_layout.WRINKLE_SLOPE_RANGE / 2.0  # slopes fit the encoding
    assert wr[..., 2].max() > 0.5 and (wr[..., 2] > 0.0).mean() < 0.4  # folds only in the joint zones
    assert (suit_textures.encode_wrinkle(wr) == suit_textures.encode_wrinkle(suit_textures.wrinkle(cfg))).all()  # deterministic


def test_gltf_suit_has_shapes_mask_uv_and_smooth_skin(mesh):
    doc, node, _ = mesh
    m = doc["meshes"][node["mesh"]]
    names = m.get("extras", {}).get("targetNames", [])
    assert sorted(names) == sorted(f"{j}_{k}" for j in fl.joint_names() for k in ("flex", "ext"))
    suit_prim = next(p for p in m["primitives"] if doc["materials"][p["material"]]["name"] == "suit")
    assert "TEXCOORD_1" in suit_prim["attributes"] and len(suit_prim["targets"]) == 16
