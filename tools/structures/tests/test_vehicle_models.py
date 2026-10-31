"""Tests of the vehicle model spec and the headless Blender vehicle generators (Blender tests skip without Blender)."""
import json
import math
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_models  # noqa: E402
import vehicle_spec  # noqa: E402

VEHICLE = "starship_block3"
DETAIL = vehicle_spec.VEHICLES / "detail" / f"{VEHICLE}.toml"
BOUND_TOL_M = 0.05
needs_blender = pytest.mark.skipif(build_models.find_blender() is None, reason="Blender not installed")


# ------------------------------------------------------------------------------------------ spec (no Blender)

def test_detail_file_values_are_tagged_and_valid():
    assert vehicle_spec.value_lines_without_tag(DETAIL) == []
    spec = vehicle_spec.load(VEHICLE)
    assert set(spec["stages"]) >= {"booster", "ship"}
    assert spec["stages"]["ship"]["nose_length_m"] == 17.0 and spec["stages"]["booster"]["nose_length_m"] == 0.0
    assert sum(len(g["angles_deg"]) for g in spec["stages"]["booster"]["engines"]) == 33
    assert sum(len(g["angles_deg"]) for g in spec["stages"]["ship"]["engines"]) == 6


def test_detail_file_does_not_repeat_physics_keys():
    import tomllib

    def keys(d):
        for k, v in d.items():
            yield k
            if isinstance(v, dict):
                yield from keys(v)

    physics = {"length_m", "diameter_m", "nose_length_m", "area_m2", "span_m", "hinge_azimuth_deg", "belly_azimuth_deg",
               "ring_radius_m", "nozzle_exit_diameter_m", "station_m", "station_fwd_m", "station_aft_m", "azimuths_deg",
               "protrusion_m", "count"}
    with open(DETAIL, "rb") as f:
        assert physics.isdisjoint(set(keys(tomllib.load(f))))  # a physics number lives in the vehicle config only


def test_missing_key_and_bad_value_raise(tmp_path):
    data = DETAIL.read_text(encoding="utf-8")
    bad = tmp_path / "vehicles"
    (bad / "detail").mkdir(parents=True)
    (bad / f"{VEHICLE}.toml").write_text((vehicle_spec.VEHICLES / f"{VEHICLE}.toml").read_text(encoding="utf-8"),
                                         encoding="utf-8")
    (bad / "detail" / f"{VEHICLE}.toml").write_text(data.replace("ring_pitch_m = 1.8", ""), encoding="utf-8")
    with pytest.raises(vehicle_spec.VehicleSpecError, match="ring_pitch_m"):
        vehicle_spec.load(VEHICLE, bad)
    (bad / "detail" / f"{VEHICLE}.toml").write_text(data.replace("ring_pitch_m = 1.8", "ring_pitch_m = -1.0"),
                                                    encoding="utf-8")
    with pytest.raises(vehicle_spec.VehicleSpecError, match="ring_pitch_m"):
        vehicle_spec.load(VEHICLE, bad)


def _flap_span_direction(hinge_deg, belly_deg, deflection_rad):
    """core/src/aero/flap.cpp flap_span_direction, in body (y, z) components."""
    h, b = math.radians(hinge_deg), math.radians(belly_deg)
    e_r = (math.cos(h), math.sin(h))
    stow = (-e_r[1], e_r[0])  # X x e_r
    if stow[0] * math.cos(b) + stow[1] * math.sin(b) > 0.0:
        stow = (-stow[0], -stow[1])
    return tuple(stow[i] * math.cos(deflection_rad) + e_r[i] * math.sin(deflection_rad) for i in range(2))


def test_flap_rotation_sign_reproduces_the_physics_span_direction():
    # model: the stowed panel direction rotated about +X by sign * d must equal the core's span direction
    for flap in vehicle_spec.load(VEHICLE)["stages"]["ship"]["flaps"]:
        h, b = flap["hinge_azimuth_deg"], 270.0
        stow = _flap_span_direction(h, b, 0.0)
        for d in (0.0, 0.3, math.pi / 2):
            t = flap["rotation_sign_about_plus_x"] * d  # rotation about +X: (y, z) -> (y cos - z sin, y sin + z cos)
            rotated = (stow[0] * math.cos(t) - stow[1] * math.sin(t), stow[0] * math.sin(t) + stow[1] * math.cos(t))
            expected = _flap_span_direction(h, b, d)
            assert rotated == pytest.approx(expected, abs=1e-9), (flap["id"], d)
        # at 90 deg the flap lies in the belly plane pointing outward; at 0 it points away from the belly (leeward)
        assert stow[0] * math.cos(math.radians(b)) + stow[1] * math.sin(math.radians(b)) <= 1e-9


def test_engine_ring_overlap_is_reported():
    spec = vehicle_spec.load(VEHICLE)
    lines = []
    for stage_id, st in spec["stages"].items():
        pos = [(g["engine"], g["ring_radius_m"] * math.cos(math.radians(a)), g["ring_radius_m"] * math.sin(math.radians(a)))
               for g in st["engines"] for a in g["angles_deg"]]
        worst = 1e9
        for i, (ei, xi, yi) in enumerate(pos):
            for ej, xj, yj in pos[i + 1:]:
                gap = math.hypot(xi - xj, yi - yj) - 0.5 * (spec["engines"][ei]["nozzle_exit_diameter_m"] +
                                                              spec["engines"][ej]["nozzle_exit_diameter_m"])
                worst = min(worst, gap)
        lines.append(f"{stage_id}: smallest gap between two exit planes {worst:+.3f} m")
        assert worst > -0.25  # config rings leave the 20 outer Raptors slightly overlapping; larger would be a typo
        outer = max(g["ring_radius_m"] for g in st["engines"])
        lines.append(f"{stage_id}: outermost bell edge {outer + 0.723:.2f} m vs hull radius {st['diameter_m'] / 2:.2f} m")
    print("\n".join(lines))


# ------------------------------------------------------------------------------------------ Blender

def _gltf(path):
    g = json.loads(Path(path).read_text(encoding="utf-8"))
    return g, (Path(path).parent / g["buffers"][0]["uri"]).read_bytes()


def _accessor(g, blob, index):
    a = g["accessors"][index]
    view = g["bufferViews"][a["bufferView"]]
    start = view.get("byteOffset", 0) + a.get("byteOffset", 0)
    ncomp = {"SCALAR": 1, "VEC2": 2, "VEC3": 3}[a["type"]]
    if a["componentType"] == 5126:
        fmt, size = "f", 4
    elif a["componentType"] == 5123:
        fmt, size = "H", 2
    else:
        fmt, size = "I", 4
    stride = view.get("byteStride", ncomp * size)
    out = []
    for i in range(a["count"]):
        out.append(struct.unpack_from("<" + fmt * ncomp, blob, start + i * stride))
    return out


def _meshes(path):
    """{mesh name: (positions, uvs, triangles)}; node transforms are identity by construction."""
    g, blob = _gltf(path)
    out = {}
    for node in g["nodes"]:
        assert "translation" not in node or list(node["translation"]) == [0.0, 0.0, 0.0]
        assert "rotation" not in node or list(node["rotation"]) == [0.0, 0.0, 0.0, 1.0]
        if "mesh" not in node:
            continue
        pos, uv, tri = [], [], []
        for prim in g["meshes"][node["mesh"]]["primitives"]:
            base = len(pos)
            pos += _accessor(g, blob, prim["attributes"]["POSITION"])
            uv += _accessor(g, blob, prim["attributes"]["TEXCOORD_0"])
            idx = [i[0] for i in _accessor(g, blob, prim["indices"])]
            tri += [(base + idx[k], base + idx[k + 1], base + idx[k + 2]) for k in range(0, len(idx), 3)]
        out[node["name"]] = (pos, uv, tri)
    return out


def _bounds(pos):
    return [min(p[i] for p in pos) for i in range(3)], [max(p[i] for p in pos) for i in range(3)]


def _signed_volume(pos, tri):
    v = 0.0
    for a, b, c in tri:
        pa, pb, pc = pos[a], pos[b], pos[c]
        v += (pa[0] * (pb[1] * pc[2] - pb[2] * pc[1]) - pa[1] * (pb[0] * pc[2] - pb[2] * pc[0]) +
              pa[2] * (pb[0] * pc[1] - pb[1] * pc[0])) / 6.0
    return v


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    out = tmp_path_factory.mktemp("vehicle_a") / "models"
    assert build_models.build_vehicle(VEHICLE, out) == 0
    return out


@pytest.fixture(scope="module")
def spec():
    return vehicle_spec.load(VEHICLE)


@needs_blender
@pytest.mark.parametrize("stage_id", ["booster", "ship"])
def test_hull_bounds_nose_and_uv(built, spec, stage_id):
    st = spec["stages"][stage_id]
    pos, uv, tri = _meshes(built / f"{stage_id}.gltf")["hull"]
    lo, hi = _bounds(pos)
    assert lo[0] == pytest.approx(0.0, abs=BOUND_TOL_M)
    assert hi[0] - lo[0] == pytest.approx(st["length_m"], abs=BOUND_TOL_M)  # length, station axis = X
    radius = st["diameter_m"] / 2.0
    assert hi[1] - lo[1] == pytest.approx(st["diameter_m"], abs=BOUND_TOL_M)  # diameter incl. the weld beads
    assert hi[2] - lo[2] == pytest.approx(st["diameter_m"], abs=BOUND_TOL_M)

    def ring(x0):  # radius of the profile station nearest to x0
        xn = min((p[0] for p in pos), key=lambda x: abs(x - x0))
        return max(math.hypot(p[1], p[2]) for p in pos if abs(p[0] - xn) < 1e-4)

    nose = st["nose_length_m"]
    if nose > 0.0:
        assert ring(st["length_m"] - nose - 0.01) == pytest.approx(radius, abs=BOUND_TOL_M)  # nose starts here
        assert ring(st["length_m"] - nose / 2.0) < radius - 0.3  # and tapers
        tip = max(p[0] for p in pos)
        assert max(math.hypot(p[1], p[2]) for p in pos if p[0] > tip - 0.05) < 1.0  # blunt tip, not a flat top
    else:
        assert ring(st["length_m"] - 0.01) == pytest.approx(radius, abs=BOUND_TOL_M)
    # UV = (arc metres at the barrel radius, station metres): glTF v is the station
    assert min(u for u, _ in uv) == pytest.approx(0.0, abs=1e-3)
    assert max(u for u, _ in uv) == pytest.approx(2.0 * math.pi * radius, abs=0.05)
    assert min(v for _, v in uv) == pytest.approx(0.0, abs=1e-3)
    assert max(v for _, v in uv) == pytest.approx(st["length_m"], abs=BOUND_TOL_M)
    # outward winding: the closed shell has a positive volume (~ the cylinder's)
    assert _signed_volume(pos, tri) > 0.8 * math.pi * radius ** 2 * (st["length_m"] - 0.5 * nose)


@needs_blender
def test_pins_fins_and_ship_features_sit_at_the_config_numbers(built, spec):
    b, s = spec["stages"]["booster"], spec["stages"]["ship"]
    parts = _meshes(built / "booster.gltf")
    pos, _, _ = parts["catch_pins"]
    lo, hi = _bounds(pos)
    assert (lo[0] + hi[0]) / 2.0 == pytest.approx(b["catch_pins"]["station_m"], abs=0.01)
    reach = max(p[1] * math.cos(math.radians(a)) + p[2] * math.sin(math.radians(a))
                for p in pos for a in b["catch_pins"]["azimuths_deg"])  # radial reach of the pin heads
    assert reach == pytest.approx(b["diameter_m"] / 2 + b["catch_pins"]["protrusion_m"], abs=0.01)
    pos, _, _ = parts["fin_fairings"]
    lo, hi = _bounds(pos)
    assert (lo[0] + hi[0]) / 2.0 == pytest.approx(b["grid_fins"]["station_m"], abs=0.01)
    ship = _meshes(built / "ship.gltf")
    pos, _, _ = ship["catch_pins"]
    assert _bounds(pos)[0][0] == pytest.approx(s["catch_pins"]["station_m"], abs=0.5)
    # the ship's chine follows the tile boundary: its azimuth is belly +- the coverage half angle (sides symmetric)
    pos, _, _ = ship["chine"]
    az = [math.degrees(math.atan2(p[2], p[1])) % 360.0 for p in pos if 10.0 < p[0] < 11.0]
    assert min(az) < 270.0 - 60.0 and max(az) > 270.0 + 60.0
    for name in ("hot_stage",):
        assert name in parts and len(parts[name][2]) > 1000  # the vents are geometry
    assert "raceway" in ship


@needs_blender
def test_flaps_hinge_at_origin_and_sizes_from_config(built, spec):
    for kind in ("fwd", "aft"):
        f = next(f for st in spec["stages"].values() for f in st["flaps"] if f["kind"] == kind)
        length = f["station_fwd_m"] - f["station_aft_m"]
        parts = _meshes(built / f"flap_{kind}.gltf")
        plo, phi = _bounds(parts["panel"][0])
        assert plo[0] == pytest.approx(0.0, abs=BOUND_TOL_M) and phi[0] == pytest.approx(length, abs=BOUND_TOL_M)
        assert plo[2] == pytest.approx(0.0, abs=BOUND_TOL_M) and phi[2] == pytest.approx(f["span_m"], abs=BOUND_TOL_M)
        hlo, hhi = _bounds(parts["hinge"][0])  # the hinge tube is centred on the X axis through the origin
        assert (hlo[1] + hhi[1]) / 2.0 == pytest.approx(0.0, abs=1e-4) and (hlo[2] + hhi[2]) / 2.0 == pytest.approx(0.0, abs=1e-4)
        assert hlo[0] == pytest.approx(0.0, abs=1e-4) and hhi[0] == pytest.approx(length, abs=BOUND_TOL_M)
        planform = (phi[0] - plo[0]) * (phi[2] - plo[2])
        assert planform == pytest.approx(f["area_m2"], rel=0.02)  # the model keeps the physics area (within chamfers)
        assert _signed_volume(*[parts["panel"][0], parts["panel"][2]]) > 0.0


@needs_blender
def test_grid_fin_pivot_on_the_hinge_and_area_from_config(built, spec):
    b = spec["stages"]["booster"]
    span = 2.0 * (b["grid_fins"]["radius_m"] - b["diameter_m"] / 2.0)
    parts = _meshes(built / "grid_fin.gltf")
    lo, hi = _bounds(parts["lattice"][0])
    assert (lo[0] + hi[0]) / 2.0 == pytest.approx(0.0, abs=1e-4) and (lo[2] + hi[2]) / 2.0 == pytest.approx(0.0, abs=1e-4)
    assert lo[1] == pytest.approx(0.0, abs=1e-4) and hi[1] == pytest.approx(span, abs=BOUND_TOL_M)
    assert (hi[0] - lo[0]) * (hi[1] - lo[1]) == pytest.approx(b["grid_fins"]["area_m2"], rel=0.01)
    slo, shi = _bounds(parts["shaft"][0])  # hinge shaft on the radial axis through the origin
    assert (slo[0] + shi[0]) / 2.0 == pytest.approx(0.0, abs=1e-4) and (slo[2] + shi[2]) / 2.0 == pytest.approx(0.0, abs=1e-4)
    assert slo[1] < 0.0


@needs_blender
@pytest.mark.parametrize("eid", ["raptor3", "raptor_vacuum3"])
def test_engine_exit_diameter_and_gimbal_origin(built, spec, eid):
    e = spec["engines"][eid]
    parts = _meshes(built / e["file"])
    pos = parts["bell"][0]
    lo, hi = _bounds(pos)
    assert lo[0] == pytest.approx(-e["bell_length_m"], abs=1e-4)  # exit plane
    rim = max(math.hypot(p[1], p[2]) for p in pos if abs(p[0] - lo[0]) < 1e-4)
    assert 2.0 * rim == pytest.approx(e["nozzle_exit_diameter_m"], abs=0.01)  # = config/engines value
    assert hi[0] == pytest.approx(0.0, abs=1e-4)  # the bell ends at the gimbal point
    assert _bounds(parts["housing"][0])[0][0] == pytest.approx(0.0, abs=1e-4)


@needs_blender
def test_manifest_and_triangle_counts(built, spec):
    manifest = json.loads((built / "manifest.json").read_text(encoding="utf-8"))
    assert set(manifest["files"]) == {"booster.gltf", "ship.gltf", "flap_fwd.gltf", "flap_aft.gltf", "grid_fin.gltf",
                                      "raptor.gltf", "rvac.gltf"}
    assert manifest["engines"]["raptor3"]["exit_plane_x_m"] < 0.0
    look = manifest["look"]  # what the game's hull shader reads (task 3)
    assert look["ship_belly_azimuth_deg"] == spec["stages"]["ship"]["belly_azimuth_deg"]
    assert look["ring_pitch_m"] == spec["detail"]["hull"]["ring_pitch_m"]
    assert look["tiles"]["coverage_station_m"] == spec["detail"]["ship"]["tiles"]["coverage_station_m"]
    assert {f["rotation_sign_about_plus_x"] for f in manifest["flaps"]["ship"]} == {-1, 1}
    print("triangle counts:", manifest["triangles"])
    for name, n in manifest["triangles"].items():
        assert 0 < n < 60000, name


@needs_blender
def test_generation_is_deterministic_and_up_to_date_check(built, tmp_path):
    second = tmp_path / "models"
    assert build_models.build_vehicle(VEHICLE, second) == 0

    # positions, UVs and indices are byte-identical; Blender's smooth normals differ in the 7th digit between runs
    # (multi-threaded accumulation), so the .bin files as a whole are compared only through these accessors
    for name in sorted(p.name for p in built.glob("*.gltf")):
        a, b = _meshes(built / name), _meshes(second / name)
        assert a.keys() == b.keys()
        for node in a:
            assert a[node] == b[node], (name, node)
    assert {p.name for p in built.iterdir()} == {p.name for p in second.iterdir()}
    manifest_time = (second / "manifest.json").stat().st_mtime
    assert build_models.build_vehicle(VEHICLE, second) == 0  # up to date: nothing regenerated
    assert (second / "manifest.json").stat().st_mtime >= manifest_time
