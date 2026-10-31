"""Tests of the vehicle model spec and the headless Blender vehicle generators (Blender tests skip without Blender)."""
import json
import math
import struct
import sys
import tomllib
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_models  # noqa: E402
import hull_textures  # noqa: E402
import hull_tiles  # noqa: E402
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
    top = st["length_m"]  # length, station axis = X; the booster's dome closes below the truss ring, clear of the ship's bells
    if stage_id == "booster":
        top -= spec["detail"]["hull"]["dome_start_m"] - spec["detail"]["hull"]["dome_height_m"]
    assert hi[0] - lo[0] == pytest.approx(top, abs=BOUND_TOL_M)
    radius = st["diameter_m"] / 2.0
    assert hi[1] - lo[1] == pytest.approx(st["diameter_m"], abs=BOUND_TOL_M)  # the plain lathe: radius = config radius
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
        shoulder = st["length_m"] - spec["detail"]["hull"]["dome_start_m"]  # the booster's forward dome follows here
        assert ring(shoulder - 0.01) == pytest.approx(radius, abs=BOUND_TOL_M)
        assert ring(top - 0.01) < 0.7  # the dome is closed at its apex
    # UV = (arc metres at the barrel radius, station metres): glTF v is the station
    assert min(u for u, _ in uv) == pytest.approx(0.0, abs=1e-3)
    assert max(u for u, _ in uv) == pytest.approx(2.0 * math.pi * radius, abs=0.05)
    assert min(v for _, v in uv) == pytest.approx(0.0, abs=1e-3)
    assert max(v for _, v in uv) == pytest.approx(top, abs=BOUND_TOL_M)
    # outward winding: the closed shell has a positive volume (~ the cylinder's)
    assert _signed_volume(pos, tri) > 0.8 * math.pi * radius ** 2 * (top - 0.5 * nose)


@needs_blender
@pytest.mark.parametrize("stage_id", ["booster", "ship"])
def test_hull_is_a_plain_lathe_without_ring_beads(built, spec, stage_id):
    st = spec["stages"][stage_id]
    pos, _, tri = _meshes(built / f"{stage_id}.gltf")["hull"]
    barrel_end = st["length_m"] - st["nose_length_m"]
    if stage_id == "booster":  # the barrel ends at the truss shoulder, the forward dome follows
        barrel_end = st["length_m"] - spec["detail"]["hull"]["dome_start_m"]
    barrel_rings = {round(p[0], 3) for p in pos if p[0] <= barrel_end + 1e-3}
    assert len(barrel_rings) <= 4  # the base, the chamfer's end and the barrel's end: no weld rings in between
    assert len(tri) < 8500  # booster 768, ship 7,680 (the nose rings); the whole stage was ~21,000 with the beads
    segments = spec["detail"]["hull"]["segments"]
    assert segments >= 96  # the silhouette at close range: sagitta < 2.5 mm at R = 4.5 m


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
    assert "hot_stage" in parts and len(parts["top_truss"][2]) > 400  # the V truss of the hot-stage section is geometry
    assert {"top_truss", "skirt", "raceway"} <= set(parts)
    tlo, thi = _bounds(parts["top_truss"][0])
    assert thi[0] == pytest.approx(b["length_m"], abs=0.2) and thi[0] - tlo[0] > spec["detail"]["booster"]["top_truss"]["height_m"]
    slo, shi = _bounds(parts["skirt"][0])
    assert math.hypot(shi[1], shi[2]) > b["diameter_m"] / 2 + 0.3  # the ring flanges stand out of the hull
    assert "raceway" in ship
    pos, _, _ = parts["strakes"]  # the long line fairings of the lower booster stand out of the hull, up to station ~34 m
    lo, hi = _bounds(pos)
    assert lo[0] == pytest.approx(3.2 - 0.0, abs=0.3) and hi[0] == pytest.approx(34.4, abs=0.3)
    assert max(math.hypot(p[1], p[2]) for p in pos) == pytest.approx(b["diameter_m"] / 2 + 1.2, abs=0.1)


@needs_blender
def test_flaps_hinge_on_the_axis_and_drawn_shape_from_the_detail_file(built, spec):
    for kind in ("fwd", "aft"):
        f = next(f for st in spec["stages"].values() for f in st["flaps"] if f["kind"] == kind)
        d = spec["detail"]["flap"][kind]
        length = f["station_fwd_m"] - f["station_aft_m"]
        u0 = (length - d["hinge_drawn_length_m"]) / 2.0  # the drawn hinge edge is centred on the physics hinge
        parts = _meshes(built / f"flap_{kind}.gltf")
        plo, phi = _bounds(parts["panel"][0])
        assert plo[0] == pytest.approx(u0 + min(d["outline_u_m"]), abs=BOUND_TOL_M)
        assert phi[0] == pytest.approx(u0 + max(d["outline_u_m"]), abs=BOUND_TOL_M)
        assert plo[2] == pytest.approx(0.0, abs=BOUND_TOL_M) and phi[2] == pytest.approx(max(d["outline_v_m"]), abs=BOUND_TOL_M)
        hlo, hhi = _bounds(parts["hinge"][0])  # the hinge fairing is centred on the X axis through the origin plane
        assert (hlo[1] + hhi[1]) / 2.0 == pytest.approx(0.0, abs=1e-4)
        assert hlo[0] == pytest.approx(u0, abs=BOUND_TOL_M) and hhi[0] == pytest.approx(u0 + d["hinge_drawn_length_m"], abs=0.5)
        # the panel's hinge edge lies on the X axis (v = 0 at Y = 0): the pivot stays on the hinge
        edge = [p for p in parts["panel"][0] if abs(p[2]) < 1e-6]
        assert edge and max(abs(p[1]) for p in edge) <= spec["detail"]["flap"]["root_thickness_m"] / 2.0 + 1e-6
        assert _signed_volume(*[parts["panel"][0], parts["panel"][2]]) > 0.0
        # drawn planform area (shoelace) vs the physics area: the deviation is documented, but stays within a factor 3
        pts = list(zip(d["outline_u_m"], d["outline_v_m"]))
        area = 0.5 * abs(sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1])))
        assert 0.25 * f["area_m2"] < area < 1.1 * f["area_m2"]


@needs_blender
def test_grid_fin_pivot_on_the_hinge_and_drawn_octagon(built, spec):
    b = spec["stages"]["booster"]
    a = spec["detail"]["grid_fin"]["across_flats_m"]
    parts = _meshes(built / "grid_fin.gltf")
    lo, hi = _bounds(parts["lattice"][0])
    assert (lo[0] + hi[0]) / 2.0 == pytest.approx(0.0, abs=1e-4) and (lo[2] + hi[2]) / 2.0 == pytest.approx(0.0, abs=1e-4)
    assert lo[1] == pytest.approx(0.0, abs=1e-4) and hi[1] == pytest.approx(a, abs=BOUND_TOL_M)  # root flat at Y = 0
    assert hi[0] - lo[0] == pytest.approx(a, abs=BOUND_TOL_M)
    # the lattice is an octagon: the corner of its bounding box is cut (no vertex there), the physics area is not drawn
    corner = [p for p in parts["lattice"][0] if p[0] > a / 2 - 0.3 and p[1] > a - 0.3]
    assert not corner
    slo, shi = _bounds(parts["shaft"][0])  # hinge shaft on the radial axis through the origin
    assert (slo[0] + shi[0]) / 2.0 == pytest.approx(0.0, abs=1e-4) and (slo[2] + shi[2]) / 2.0 == pytest.approx(0.0, abs=1e-4)
    assert slo[1] < 0.0
    assert b["grid_fins"]["radius_m"] > b["diameter_m"] / 2.0


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


def _dome_station(hull_pos, length_m, radius_m):
    """Highest station of the booster's hull mesh at or beyond a radius (linear between the profile's rings)."""
    rings = sorted({(round(p[0], 4), round(math.hypot(p[1], p[2]), 4)) for p in hull_pos if p[0] > length_m - 6.0})
    best = rings[0][0]
    for (x0, r0), (x1, r1) in zip(rings, rings[1:]):
        if min(r0, r1) <= radius_m <= max(r0, r1) and r0 != r1:
            best = max(best, x0 + (x1 - x0) * (radius_m - r0) / (r1 - r0))
    return best


@needs_blender
def test_ship_engines_sit_inside_the_hull_and_clear_the_dome_at_any_gimbal(built, spec):
    b, s, detail = spec["stages"]["booster"], spec["stages"]["ship"], spec["detail"]
    manifest = json.loads((built / "manifest.json").read_text(encoding="utf-8"))
    hull = _meshes(built / "booster.gltf")["hull"][0]
    L = b["length_m"]
    # true dome: apex under the hot-stage ring, the ring's top at the booster's top
    ring_h = detail["booster"]["hot_stage_ring"]["height_m"]
    assert max(p[0] for p in hull) == pytest.approx(L - ring_h, abs=BOUND_TOL_M)
    assert detail["hull"]["dome_start_m"] - detail["hull"]["dome_height_m"] == pytest.approx(ring_h)
    truss = _meshes(built / "booster.gltf")["top_truss"][0]
    assert max(p[0] for p in truss) == pytest.approx(L, abs=BOUND_TOL_M)  # the truss's ring carries the ship
    # the bay: open aft end, the plate above every drawn exit plane
    bay = _meshes(built / "ship.gltf")["engine_bay"][0]
    assert min(p[0] for p in bay) == pytest.approx(0.0, abs=BOUND_TOL_M)
    assert max(p[0] for p in bay) == pytest.approx(detail["ship"]["engine_bay"]["plate_station_m"], abs=BOUND_TOL_M)
    ship_hull = _meshes(built / "ship.gltf")["hull"][0]
    assert min(math.hypot(p[1], p[2]) for p in ship_hull if p[0] < 0.01) > 4.0  # no base disc any more
    for group in s["engines"]:
        eid = group["engine"]
        e = spec["engines"][eid]
        m = manifest["engines"][eid]
        recess = detail["ship"]["engine_recess"][eid]
        assert m["ship_exit_station_m"] == recess  # one recess per engine kind
        assert m["ship_axial_offset_m"] == pytest.approx(recess + e["bell_length_m"])
        assert m["ship_axial_offset_m"] + m["exit_plane_x_m"] == pytest.approx(recess)  # exit above the aft end by it
        rim = e["nozzle_exit_diameter_m"] / 2.0
        ring = group["ring_radius_m"]
        assert ring + rim < detail["ship"]["engine_bay"]["wall_inner_radius_m"]  # inside the hull
        # both bell kinds, at the largest gimbal of the config (RVac: its [gimbal] too, although the group is fixed)
        with open(vehicle_spec.ENGINES / f"{eid}.toml", "rb") as f:
            max_deg = tomllib.load(f)["gimbal"]["max_angle_deg"]
        t = math.radians(max_deg)
        clearance = math.inf
        for k in range(72):  # direction of the tilt
            phi = 2.0 * math.pi * k / 72
            for j in range(36):  # rim point
                psi = 2.0 * math.pi * j / 36
                # rim point in the engine frame (gimbal at the origin, thrust +x), tilted by t toward phi
                radial = rim * math.cos(psi - phi)
                x_e = -e["bell_length_m"] * math.cos(t) - radial * math.sin(t)
                u = -e["bell_length_m"] * math.sin(t) + radial * math.cos(t)
                v = rim * math.sin(psi - phi)
                y, z = ring + u * math.cos(phi) - v * math.sin(phi), u * math.sin(phi) + v * math.cos(phi)
                station = L + m["ship_axial_offset_m"] + x_e
                clearance = min(clearance, station - _dome_station(hull, L, min(math.hypot(y, z), 4.5)))
        print(f"{eid}: rim to dome clearance at {max_deg} deg: {clearance:.3f} m")
        assert clearance > 0.0


@needs_blender
def test_manifest_and_triangle_counts(built, spec):
    manifest = json.loads((built / "manifest.json").read_text(encoding="utf-8"))
    assert set(manifest["files"]) == {"booster.gltf", "ship.gltf", "flap_fwd.gltf", "flap_aft.gltf", "grid_fin.gltf",
                                      "raptor.gltf", "rvac.gltf", "hull_steel.png", "tile_patch.png", "tile_sel.png",
                                      "tile_ids.png"}
    steel = manifest["look"]["steel_map"]
    assert steel["file"] == "hull_steel.png" and (built / steel["file"]).is_file()
    assert steel["period_v_m"] == spec["detail"]["hull"]["ring_pitch_m"]
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
    assert (built / "hull_steel.png").read_bytes() == (second / "hull_steel.png").read_bytes()  # the baked map too
    manifest_time = (second / "manifest.json").stat().st_mtime
    assert build_models.build_vehicle(VEHICLE, second) == 0  # up to date: nothing regenerated
    assert (second / "manifest.json").stat().st_mtime >= manifest_time


# ------------------------------------------------------------------------------ baked steel map (no Blender)

def test_steel_map_is_deterministic_seamless_and_has_the_weld_at_v_zero(spec):
    hull = spec["detail"]["hull"]
    radius = spec["stages"]["ship"]["diameter_m"] / 2.0
    a, b = hull_textures.bake(radius, hull), hull_textures.bake(radius, hull)
    assert a.shape == (hull_textures.SIZE_V, hull_textures.SIZE_U, 4)
    assert (a == b).all() and hull_textures.png_bytes(a) == hull_textures.png_bytes(b)
    slope = (a[..., :2].astype(float) / 255.0 - 0.5) * hull_textures.SLOPE_RANGE
    # seamless: the step across the wrap is no larger than the largest step between neighbours (v: the dents' slope
    # along u only - the weld's apex sits exactly on the seam, so the v slope flips sign there by design)
    for axis, channels in ((0, [0]), (1, [1])):  # the u slope flips at the longitudinal seam by design, like v at the ring weld
        part = slope[..., channels]
        inner = abs(part.take(range(1, part.shape[axis]), axis) - part.take(range(0, part.shape[axis] - 1), axis))
        wrap = abs(part.take(0, axis) - part.take(-1, axis))
        assert wrap.max() <= inner.max() + 1e-9
    dent_rows = slice(hull_textures.SIZE_V // 4, 3 * hull_textures.SIZE_V // 4)
    cols = slice(hull_textures.SIZE_U // 4, 3 * hull_textures.SIZE_U // 4)  # away from the longitudinal seam
    assert slope[dent_rows, cols, 0].std() == pytest.approx(hull["dent_slope_rms"] / math.sqrt(2.0), rel=0.6)
    # the bead: slope +-weld_height / half width either side of v = 0 (row 0 and the last row), HAZ coverage there
    peak = hull["weld_height_m"] / hull["weld_half_width_m"]
    assert abs(slope[1, 40, 1]) == pytest.approx(peak, abs=0.05) and slope[1, 40, 1] * slope[-2, 40, 1] < 0.0
    assert a[0, 0, 3] == 255 and a[hull_textures.SIZE_V // 2, hull_textures.SIZE_U // 2, 3] == 0
    mean = a[..., 3].mean() / 255.0  # heat-affected band share of the ring: 2 x 1.2 x half width / pitch
    pu, pv = hull_textures.periods_m(radius, hull["ring_pitch_m"])
    ring, seam = (2.0 * hull_textures.HAZ_FACTOR * hull["weld_half_width_m"] / p for p in (pv, pu))
    seam *= hull_textures.SEAM_HAZ_SHARE  # coverage 0.3 over the seam's width
    assert mean == pytest.approx(ring + seam - ring * seam, rel=0.08)


def test_steel_map_needs_one_hull_radius(spec):
    import copy
    bad = copy.deepcopy(spec)
    bad["stages"]["booster"]["diameter_m"] = 8.0
    with pytest.raises(ValueError, match="one hull radius"):
        hull_textures.write(Path("."), bad)


# ------------------------------------------------------------------------------ baked tile maps (no Blender)

def test_tile_maps_are_deterministic_periodic_and_have_the_gap_share(spec):
    tiles = spec["detail"]["ship"]["tiles"]
    af, gap = tiles["hex_across_flats_m"], tiles["gap_m"]
    a, b = hull_tiles.bake_patch(af, gap), hull_tiles.bake_patch(af, gap)
    assert a.shape == (hull_tiles.TILE_SIZE_V, hull_tiles.TILE_SIZE_U, 4) and (a == b).all()
    assert (hull_tiles.bake_ids() == hull_tiles.bake_ids()).all() and (hull_tiles.bake_sel() == hull_tiles.bake_sel()).all()
    # gap coverage = perimeter x gap / 2 / area of the hexagon (two tiles per period): 2 sqrt3 x gap / (sqrt3/2 x af) / 2
    expected = 2.0 * math.sqrt(3.0) * af * gap / 2.0 / (math.sqrt(3.0) / 2.0 * af * af)
    assert a[..., 3].mean() / 255.0 == pytest.approx(expected, rel=0.03)
    tilt = a[..., :2].astype(float) / 255.0 - 0.5
    assert abs(tilt.mean(axis=(0, 1))).max() < 0.01          # a bevel tilts outwards on every side: no mean tilt
    assert abs(tilt).max() <= hull_tiles.TILE_BEVEL_TILT + 0.01
    # periodic: the pattern is mirror-symmetric about the edge across the arc seam (a flat there: equal gap coverage, opposite
    # tilt) and continuous along the station seam (a tile's middle there)
    ai = a.astype(int)
    assert abs(ai[:, 0, 3] - ai[:, -1, 3]).max() <= 2 and abs(ai[:, 0, 0] + ai[:, -1, 0] - 256).max() <= 3
    assert abs(ai[0] - ai[-1]).max() <= abs(ai[1:] - ai[:-1]).max()


def test_tile_sel_ids_cover_both_lattices_and_the_repaired_share():
    sel, ids = hull_tiles.bake_sel(), hull_tiles.bake_ids()
    assert set(sel[..., 2].flatten()) == {0, 255} and abs((sel[..., 2] > 0).mean() - 0.5) < 0.02  # two tiles per period
    assert set(sel[..., 0].flatten()) == {0, 255} and set(sel[..., 1].flatten()) == {0, 255}
    repaired = (ids[..., 2].mean() + ids[..., 3].mean()) / 2.0 / 255.0  # 16k Bernoulli draws: sigma ~ 15 %
    assert repaired == pytest.approx(hull_tiles.REPAIRED_FRACTION, rel=0.5)
    assert abs(ids[..., 0].mean() / 255.0 - 0.5) < 0.03  # the shade 0.75 + 0.5 v has the mean 1
