"""Dorsal hardware of the tanker (plan 2026-11-04, Task 3): spec, layout and, when generated, the glTF. The glTF checks skip without data/models."""
import base64
import json
import struct
import sys
import tomllib
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
import build_models  # noqa: E402
import dorsal_layout as dl  # noqa: E402
import vehicle_spec  # noqa: E402

TANKER = vehicle_spec.ROOT / "config" / "vehicles" / "starship_tanker.toml"
MODELS = build_models.ROOT / "data" / "models" / "starship_tanker"
TOLERANCE_M = 0.01


def _config_ship():
    with open(TANKER, "rb") as f:
        stages = tomllib.load(f)["stage"]
    return next(s for s in stages if s["from"].endswith(":ship"))


def _mesh_points(doc, buffers, name):
    node = next(n for n in doc["nodes"] if n.get("name") == name)
    prim = doc["meshes"][node["mesh"]]["primitives"][0]
    acc = doc["accessors"][prim["attributes"]["POSITION"]]
    view = doc["bufferViews"][acc["bufferView"]]
    off = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = view.get("byteStride", 12)
    data = buffers[view["buffer"]]
    return [struct.unpack_from("<3f", data, off + k * stride) for k in range(acc["count"])]


def _load_gltf():
    path = MODELS / "dorsal_hardware.gltf"
    if not path.is_file():
        pytest.skip("data/models/starship_tanker/dorsal_hardware.gltf not generated")
    doc = json.loads(path.read_text(encoding="utf-8"))
    buffers = [base64.b64decode(b["uri"].split(",", 1)[1]) if b["uri"].startswith("data:") else (path.parent / b["uri"]).read_bytes()
               for b in doc["buffers"]]
    return doc, buffers


def test_spec_carries_the_config_ports_and_connectors():
    spec = vehicle_spec.load_starship("starship_tanker")
    ship = _config_ship()
    dorsal = spec["dorsal"]
    assert dorsal["stage_id"] == "ship" or dorsal["stage_id"] == spec["stages"]["ship"]["stage_id"]
    assert dorsal["ports"][0]["elements_m"] == ship["docking_port"][0]["elements_m"]
    assert [c["position_m"] for c in dorsal["connectors"]] == [c["position_m"] for c in ship["transfer_connector"]]
    assert dorsal["ports"][0]["soft_capture"] == "active"
    assert "dorsal" not in vehicle_spec.load_starship("starship_block3")  # Block 3 spec and hash are untouched


def test_dorsal_drawing_values_are_tagged_and_validated():
    path = vehicle_spec.VEHICLES / "detail" / "starship_tanker.toml"
    assert vehicle_spec.value_lines_without_tag(path) == []
    with pytest.raises(vehicle_spec.VehicleSpecError, match="probe_stroke_m"):
        vehicle_spec.dorsal_block("x", "ship", _config_ship(), {k: 1.0 for k in vehicle_spec.DORSAL_KEYS if k != "probe_stroke_m"})


def test_probe_tip_is_the_local_origin_and_stowed_tip_sits_inside_the_fairing():
    spec = vehicle_spec.load_starship("starship_tanker")
    d = spec["dorsal"]["drawing"]
    prof = dl.probe_profile(d)
    assert max(x for x, _ in prof) == 0.0 and prof[0] == (0.0, d["probe_tip_flat_radius_m"])
    hull_r = 0.5 * spec["stages"]["ship"]["diameter_m"]
    for n in dl.element_nodes(spec["dorsal"]["ports"]):
        stowed_axial = dl.axial_m(n["element_m"], n["axis"]) - d["probe_stroke_m"]
        assert stowed_axial <= hull_r + d["fairing_height_m"]
        g = dl.fairing_geometry(d, hull_r, n["element_m"], n["axis"])
        assert g["floor_x"] < -d["probe_stroke_m"] - dl.probe_head_total_m(d)  # the stowed head fits in its bore
        assert g["foot_x"] < g["floor_x"]


def test_manifest_block_names_the_elements_and_the_stroke():
    spec = vehicle_spec.load_starship("starship_tanker")
    table = vehicle_spec.generator_table("starship_tanker")
    manifest = build_models.vehicle_manifest(spec, "h", "v", {}, table)
    assert manifest["dorsal_elements_m"] == _config_ship()["docking_port"][0]["elements_m"]
    assert manifest["dorsal_nodes"] == ["probe_0", "probe_1", "probe_2", "probe_3"]
    assert manifest["probe_stroke_m"] == spec["dorsal"]["drawing"]["probe_stroke_m"]
    assert set(manifest["dorsal_connectors"]) == {"connector_lox", "connector_ch4"}
    assert "dorsal_hardware.gltf" in manifest["files"] and "ship.gltf" in manifest["files"]


def test_probe_tips_on_config_elements():
    """The extended tip (the glTF's highest point along the axis, on the axis line) is the node's origin, which the game puts on `elements_m`."""
    doc, buffers = _load_gltf()
    ship = _config_ship()
    axis = ship["docking_port"][0]["axis"]
    assert axis == [0.0, 0.0, 1.0]
    for i, element in enumerate(ship["docking_port"][0]["elements_m"]):
        pts = _mesh_points(doc, buffers, f"probe_{i}")
        tip = max(pts, key=lambda p: p[2])
        assert abs(tip[2]) < TOLERANCE_M and abs(tip[0]) < TOLERANCE_M + 0.03 and abs(tip[1]) < TOLERANCE_M + 0.03, (i, tip)
        assert max(p[2] for p in pts) < TOLERANCE_M  # nothing above the element
        # origin of the node is applied by the game from the manifest; it must equal the config element
    manifest = json.loads((MODELS / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["dorsal_elements_m"] == ship["docking_port"][0]["elements_m"]


def test_connector_faces_on_config_positions():
    doc, buffers = _load_gltf()
    manifest = json.loads((MODELS / "manifest.json").read_text(encoding="utf-8"))
    for c in _config_ship()["transfer_connector"]:
        name = "connector_" + c["id"].removesuffix("_transfer")
        pts = _mesh_points(doc, buffers, name)
        assert abs(max(p[2] for p in pts)) < TOLERANCE_M  # the face is the top of the part: local 0
        assert manifest["dorsal_connectors"][name]["position_m"] == c["position_m"]


def test_from_entry_appends_ports_and_connectors_to_the_source_stage(tmp_path):
    """As core/src/config/vehicle_config.cpp load_stage_from: the entry's [[docking_port]] / [[transfer_connector]] are added to the source stage's, not replacing them."""
    (tmp_path / "src.toml").write_text(
        '[vehicle]\nid = "src"\n[[stage]]\nid = "ship"\nlength_m = 1.0\n'
        '[[stage.docking_port]]\nid = "own"\n[[stage.transfer_connector]]\nid = "own_c"\n', encoding="utf-8")
    (tmp_path / "var.toml").write_text(
        '[vehicle]\nid = "var"\n[[stage]]\nfrom = "src:ship"\n'
        '[[stage.docking_port]]\nid = "added"\n[[stage.transfer_connector]]\nid = "added_c"\n', encoding="utf-8")
    (tmp_path / "only_source.toml").write_text('[vehicle]\nid = "o"\n[[stage]]\nfrom = "src:ship"\n', encoding="utf-8")
    ship = vehicle_spec.resolve_stages("var", tmp_path)[0]
    assert [p["id"] for p in ship["docking_port"]] == ["own", "added"]
    assert [c["id"] for c in ship["transfer_connector"]] == ["own_c", "added_c"]
    kept = vehicle_spec.resolve_stages("only_source", tmp_path)[0]
    assert [p["id"] for p in kept["docking_port"]] == ["own"]  # no entry tables: the source's stay
