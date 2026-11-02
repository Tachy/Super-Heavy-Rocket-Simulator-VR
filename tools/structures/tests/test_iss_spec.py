"""ISS drawing spec and generated model: the ports follow the physics config, the joint tables are complete, the generated parts stand where the tables say.
Blender cases skip without generated models (python tools/structures/build_models.py --vehicle iss)."""
import json
import sys
import tomllib
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_models  # noqa: E402
import vehicle_spec  # noqa: E402

MODELS = build_models.ROOT / "data" / "models" / "iss"
VEHICLES = vehicle_spec.VEHICLES


def spec():
    return vehicle_spec.load("iss")


def test_ports_are_the_physics_ports():
    with open(VEHICLES / "iss.toml", "rb") as f:
        station = next(st for st in tomllib.load(f)["stage"] if st["id"] == "station")
    ports = spec()["ports"]
    assert [p["id"] for p in ports] == [p["id"] for p in station["docking_port"]]
    for ours, cfg in zip(ports, station["docking_port"]):
        assert ours["position_m"] == cfg["position_m"] and ours["axis"] == cfg["axis"] and ours["up"] == cfg["up"]


def test_up_keeps_model_d_petal_clocking():
    """The config's up must put the IDD's petals (phi 0 / 120 / 240 about up) on the angles measured on model D: IDA-2 petal bodies at 58 / 178 / 302 in the frame (x out, z zenith, y = z x x),
    IDA-3 at 358 / 122 / 238 in (x out, z forward). +-3 deg: the measure's error (docs/subsystems/iss.md)."""
    import math
    frames = {"ida2": ((0.0, 0.0, -1.0), (0.0, -1.0, 0.0), (58.0, 178.0, 302.0)), "ida3": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (358.0, 122.0, 238.0))}
    for port in spec()["ports"]:
        z0, y0, measured = frames[port["id"]]
        up = port["up"]
        phi_up = math.degrees(math.atan2(sum(a * b for a, b in zip(up, z0)), sum(a * b for a, b in zip(up, y0))))
        drawn = sorted((phi_up - 90.0 + k) % 360.0 for k in (0.0, 120.0, 240.0))
        for a in drawn:
            nearest = min(min(abs(a - b) % 360.0, 360.0 - abs(a - b) % 360.0) for b in measured)
            assert nearest <= 3.0, (port["id"], drawn, measured)


def test_tables_are_tagged_and_complete():
    d = spec()["detail"]
    assert len(d["wing"]) == 8 and len(d["pose"]["beta_deg"]) == 8 and len(d["gamma"]) == 2
    assert {w["irosa"] for w in d["wing"]} - {""} == {r["id"] for r in d["irosa_plane"]}


def test_missing_table_is_refused(tmp_path):
    (tmp_path / "detail").mkdir()
    for name in ("iss.toml", "crew_dragon.toml"):
        (tmp_path / name).write_bytes((VEHICLES / name).read_bytes())
    (tmp_path / "detail" / "crew_dragon.toml").write_bytes((VEHICLES / "detail" / "crew_dragon.toml").read_bytes())
    text = (VEHICLES / "detail" / "iss.toml").read_text(encoding="utf-8").replace("[pose]", "[pose_x]")
    (tmp_path / "detail" / "iss.toml").write_text(text, encoding="utf-8")
    with pytest.raises(vehicle_spec.VehicleSpecError):
        vehicle_spec.load_iss("iss", tmp_path)


@pytest.mark.skipif(not (MODELS / "manifest.json").is_file(), reason="models not generated")
def test_manifest_joints_and_triangles():
    m = json.loads((MODELS / "manifest.json").read_text(encoding="utf-8"))
    names = {j["name"] for j in m["joints"]}
    assert {f"alpha_joint_{k}" for k in range(2)} | {f"beta_joint_{i}" for i in range(8)} | {f"radiator_joint_{j}" for j in range(2)} == names
    assert all(j["parent"] == "" or j["parent"] in names for j in m["joints"])
    # [E] plan Task 9 measures the cost; this pins the triangle count the report states
    assert sum(m["triangles"].values()) < 60000
    for part in ("pressurised", "truss", "pma_2", "ida_2", "pma_3", "ida_3", "ida_2_petals", "ida_3_hcs"):
        assert part in m["look"]["part_materials"]
