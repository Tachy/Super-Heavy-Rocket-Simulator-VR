"""Console textures (fidelity plan Task 6): the console atlas (cabin_layout.console_charts, cabin_textures.py) and the de-lighting tool
(photo_delight.py). No photo is read: the de-lighting case is synthetic, the baker uses only config and its own constants."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
import build_models  # noqa: E402
import cabin_layout as cl  # noqa: E402
import cabin_textures  # noqa: E402
import photo_delight  # noqa: E402
import surface_textures  # noqa: E402
import vehicle_spec  # noqa: E402

LABEL_TOL_M = 0.001  # [S] fidelity plan Task 6: label centre within 1 mm of its key centre (UV -> 3D)
DELIGHT_TOL = 0.03  # [S] fidelity plan Task 6: de-lit albedo within 3 %
MAX_SIZE = 4096  # [S] fidelity plan Task 6: power of two, at most 4096


@pytest.fixture(scope="module")
def cockpit():
    return vehicle_spec.load("crew_dragon")["cockpit"]


@pytest.fixture(scope="module")
def baked(cockpit, tmp_path_factory):
    out = tmp_path_factory.mktemp("cabin_textures")
    files = cabin_textures.write(out, cockpit)
    return out, files


def _png(path):
    from PIL import Image
    return np.asarray(Image.open(path))


def _pow2(n):
    return n > 0 and n & (n - 1) == 0


def test_cabin_textures_have_mipmap_friendly_sizes(cockpit, baked):
    out, files = baked
    assert set(files) == {"albedo", "normal", "orm"} and files == cl.CONSOLE_MAPS
    for role, name in files.items():
        img = _png(out / name)
        h, w = img.shape[:2]
        assert _pow2(w) and _pow2(h) and max(w, h) <= MAX_SIZE, (role, w, h)
        assert (w, h) == cl.CONSOLE_ATLAS_SIZE
    labels = surface_textures.cockpit_labels(cockpit)  # the key labels' own atlas (mipmapped too)
    assert _pow2(labels.shape[0]) and _pow2(labels.shape[1]) and max(labels.shape[:2]) <= MAX_SIZE
    assert cl.cockpit_manifest(cockpit)["console_maps"] == cl.CONSOLE_MAPS


def test_console_charts_are_disjoint_and_inside_the_atlas(cockpit):
    charts = cl.console_charts(cockpit)
    W, H = cl.CONSOLE_ATLAS_SIZE
    rects = [c["rect"] for c in charts.values()]
    for x, y, w, h in rects:
        assert 0 <= x and 0 <= y and x + w <= W and y + h <= H
    for i, (x0, y0, w0, h0) in enumerate(rects):
        for x1, y1, w1, h1 in rects[i + 1:]:
            g = cl.CHART_GUTTER_PX
            assert x0 + w0 + g <= x1 or x1 + w1 + g <= x0 or y0 + h0 + g <= y1 or y1 + h1 + g <= y0
    # every console node face lands in its chart: the corners of the body's faces map inside the chart rect
    f = cl.console_frame(cockpit)
    for node, n_local, p_local in (("console_housing", (0, 0, 1), (0.6, 0.4, -0.002)), ("console_back", (0, 0, -1), (-0.69, -0.33, -0.221)),
                                   ("console_under", (0, -1, 0), (0.69, -0.277, -0.22)), ("console_shelf", (0, 0.26, 0.97), (0.62, -0.236, 0.03))):
        ch = charts[cl.chart_of(node, n_local)]
        x, y = cl.chart_uv(ch, *cl.plane_ab(ch, p_local))
        rx, ry, rw, rh = ch["rect"]
        assert rx <= x * W <= rx + rw and ry <= y * H <= ry + rh, node
        p = [f["origin"][i] + f["right"][i] * p_local[0] + f["up"][i] * p_local[1] + f["normal"][i] * p_local[2] for i in range(3)]
        assert cl.console_local(cockpit, p) == pytest.approx(p_local, abs=1e-9)


def _ink_centre(cov, threshold=0.5):
    ys, xs = np.nonzero(cov > threshold)
    return 0.5 * (xs.min() + xs.max() + 1), 0.5 * (ys.min() + ys.max() + 1)


def test_labels_sit_on_their_keys(cockpit, baked):
    """Key labels (cockpit_labels.png, one cell per key face): the ink's centre, carried UV -> 3D through the manifest's key frame, lies within 1 mm of
    the key centre. Locker numerals (console atlas): the numeral's ink centre, UV -> chart -> 3D, lies within 1 mm of its placard position, and on its
    locker's placard."""
    manifest = cl.cockpit_manifest(cockpit)
    atlas = surface_textures.cockpit_labels(cockpit)[..., 0].astype(float) / 255.0
    h, w = atlas.shape
    checked = 0
    for bid, b in manifest["buttons"].items():
        if "label_cell" not in b:
            continue
        u0, v0, u1, v1 = b["label_cell"]
        x0, y0, x1, y1 = round(u0 * w), round(v0 * h), round(u1 * w), round(v1 * h)
        cx, cy = _ink_centre(atlas[y0:y1, x0:x1])
        fx, fy = cx / (x1 - x0), cy / (y1 - y0)  # the shader: face point = centre + right (f.x - 0.5) size.x + up (0.5 - f.y) size.y
        p = [b["centre"][i] + b["right"][i] * (fx - 0.5) * b["size"][0] + b["up"][i] * (0.5 - fy) * b["size"][1] for i in range(3)]
        assert math.dist(p, b["centre"]) <= LABEL_TOL_M, (bid, math.dist(p, b["centre"]))
        checked += 1
    assert checked >= 20
    out, files = baked
    albedo = _png(out / files["albedo"]).astype(float) / 255.0
    W, H = cl.CONSOLE_ATLAS_SIZE
    front = cl.console_charts(cockpit)["front"]
    for text, (lx0, lx1, uy0, uy1), (n_lat, n_u), _ in cabin_textures.PLACARDS:
        ax0, ay0 = cl.chart_uv(front, *cl.plane_ab(front, (lx0, uy1, 0.0)))
        ax1, ay1 = cl.chart_uv(front, *cl.plane_ab(front, (lx1, uy0, 0.0)))
        X0, Y0, X1, Y1 = int(ax0 * W), int(ay0 * H), int(math.ceil(ax1 * W)), int(math.ceil(ay1 * H))
        patch = albedo[Y0:Y1, X0:X1, :3]
        white = (patch.min(axis=-1) > 0.6).astype(float)  # the white numeral on the black placard (the red symbol is not white)
        cx, cy = _ink_centre(white)
        uvx, uvy = (X0 + cx) / W, (Y0 + cy) / H
        a = front["a_range"][0] + (uvx * W - front["rect"][0]) * front["texel"]
        b_ = front["b_range"][1] - (uvy * H - front["rect"][1]) * front["texel"]
        lat, u, _ = cl.plane_point(front, a, b_)
        assert math.hypot(lat - n_lat, u - n_u) <= LABEL_TOL_M, (text, lat, u)
        assert lx0 < lat < lx1 and uy0 < u < uy1


def test_delight_recovers_flat_albedo():
    """Synthetic: a pinhole camera 0.6 m in front of a plane chart; photo = known albedo x a shading image; delight with that shading image gives the
    albedo back within 3 % (and with a callable shading, too)."""
    chart = photo_delight.Chart(np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]),
                                (-0.2, 0.2), (-0.1, 0.1), 0.002)

    def albedo_at(a, b):
        return np.stack([0.3 + 0.2 * np.sin(12 * a) * np.cos(9 * b), 0.25 + 0.1 * a, 0.2 + 0.4 * (b + 0.1)], axis=-1)

    row = {"id": "synthetic", "file": "", "size_px": [800, 600], "position_m": [0.02, -0.01, 0.6], "forward": [0.0, 0.0, -1.0], "up": [0.0, 1.0, 0.0],
           "fov_h_deg": 60.0, "model": "pinhole"}
    cam = photo_delight.Camera.from_row(row)
    w, h = row["size_px"]
    px = np.stack(np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5), axis=-1).reshape(-1, 2)
    d = photo_delight.unproject(px, cam)
    t = -cam.position_m[2] / d[:, 2]
    hit = cam.position_m + d * t[:, None]
    alb = albedo_at(hit[:, 0], hit[:, 1]).reshape(h, w, 3)
    yy, xx = np.mgrid[0:h, 0:w]
    shade = (0.4 + 0.6 * np.exp(-((xx - 300) ** 2 + (yy - 250) ** 2) / (2 * 250.0 ** 2)))[..., None] * np.array([1.0, 0.95, 0.9])
    photo = alb * shade
    got = photo_delight.delight(["synthetic"], chart, {"synthetic": row}, shading={"synthetic": shade}, photos={"synthetic": photo})
    A, B = chart.ab()
    want = albedo_at(A, B)
    ok = np.isfinite(got[..., 0])
    assert ok.mean() > 0.95
    assert np.max(np.abs(got[ok] / want[ok] - 1.0)) < DELIGHT_TOL
    # callable shading (the analytic form): the same scale over the chart's points
    flat = photo_delight.delight(["synthetic"], chart, {"synthetic": row}, shading=lambda p, n: np.full(p.shape[:-1], 2.0),
                                 photos={"synthetic": alb * 2.0})
    assert np.nanmax(np.abs(flat[ok] / want[ok] - 1.0)) < DELIGHT_TOL


def test_generated_console_faces_use_the_atlas():
    """The generated cabin's console nodes carry atlas UVs inside [0, 1] (skips without generated models)."""
    path = build_models.ROOT / "data" / "models" / "crew_dragon" / "dragon_cabin.gltf"
    manifest = build_models.ROOT / "data" / "models" / "crew_dragon" / "manifest.json"
    if not path.is_file() or "console_maps" not in json.loads(manifest.read_text(encoding="utf-8")).get("cockpit", {}):
        pytest.skip("generated crew_dragon models with the console atlas needed (build_models.py --vehicle crew_dragon)")
    import base64
    doc = json.loads(path.read_text(encoding="utf-8"))
    buffers = [base64.b64decode(b["uri"].split(",", 1)[1]) if b["uri"].startswith("data:") else (path.parent / b["uri"]).read_bytes() for b in doc["buffers"]]
    meshes = {n["name"]: n["mesh"] for n in doc["nodes"] if "mesh" in n}
    for node in cl.NODE_CHARTS:
        prim = doc["meshes"][meshes[node]]["primitives"][0]
        acc = doc["accessors"][prim["attributes"]["TEXCOORD_0"]]
        view = doc["bufferViews"][acc["bufferView"]]
        raw = buffers[view["buffer"]][view.get("byteOffset", 0) + acc.get("byteOffset", 0):][:acc["count"] * 8]
        uv = np.frombuffer(raw, dtype="<f4").reshape(-1, 2)
        assert uv.min() >= -1e-4 and uv.max() <= 1.0 + 1e-4, node


def _cabin_point(cockpit, lat, u, o):
    f = cl.console_frame(cockpit)
    return tuple(f["origin"][i] + f["right"][i] * lat + f["up"][i] * u + f["normal"][i] * o for i in range(3))


def _pixel_of(cockpit, charts, node, local_p, local_n):
    """The atlas pixel (x, y from the top) the generator's UVs give a cabin point: gen_dragon_cabin.chart_faces calls cl.face_uvs (Blender V up),
    the glTF exporter flips V back (v_gltf = 1 - v_blender); Godot samples y down from the top."""
    f = cl.console_frame(cockpit)
    n = tuple(f["right"][i] * local_n[0] + f["up"][i] * local_n[1] + f["normal"][i] * local_n[2] for i in range(3))
    (u, v), = cl.face_uvs(cockpit, charts, node, [_cabin_point(cockpit, *local_p)], n)
    W, H = cl.CONSOLE_ATLAS_SIZE
    return u * W, (1.0 - v) * H


def test_generator_uvs_land_on_the_painted_features(cockpit, baked):
    """Ties the mesh UVs (cl.face_uvs, incl. the V flip) to the painted atlas: known 3D features -> UV -> atlas pixel -> the painted feature is
    there (placard numerals 5 / 6 within 2 mm, a screw head lighter than its surround, the black vent vs the grey band above it)."""
    out, files = baked
    albedo = _png(out / files["albedo"]).astype(float) / 255.0
    charts = cl.console_charts(cockpit)
    front, back = charts["front"], charts["back"]
    tol_m = 0.002  # stated tolerance: 2 mm (4 texels of the 0.5 mm front chart)
    # placard numerals: the white ink's centre in a +-12 mm window around the numeral, reached through the generator's UV path
    for text, _, (n_lat, n_u), _ in cabin_textures.PLACARDS:
        px, py = _pixel_of(cockpit, charts, "console_housing", (n_lat, n_u, 0.0), (0, 0, 1))
        r = int(0.012 / front["texel"])
        x0, y0 = int(px) - r, int(py) - r
        patch = albedo[y0:y0 + 2 * r, x0:x0 + 2 * r, :3]
        cx, cy = _ink_centre((patch.min(axis=-1) > 0.6).astype(float))
        off_m = math.hypot(x0 + cx - px, y0 + cy - py) * front["texel"]
        assert off_m <= tol_m, (text, off_m)
    # a screw head: lighter (sRGB) than the console face 8 mm away
    lat, u = cabin_textures.SCREWS_FRONT[0]
    px, py = _pixel_of(cockpit, charts, "console_housing", (lat, u, 0.0), (0, 0, 1))
    qx, qy = _pixel_of(cockpit, charts, "console_housing", (lat, u - 0.008, 0.0), (0, 0, 1))
    assert albedo[round(py), round(px), :3].mean() > 1.25 * albedo[round(qy), round(qx), :3].mean()
    # the back: the black vent (u -0.275..-0.125) is darker than the grey band above it (u -0.125..-0.015), both on the back chart (normal -o)
    vx, vy = _pixel_of(cockpit, charts, "console_back", (0.0, -0.2, -0.2), (0, 0, -1))
    bx, by = _pixel_of(cockpit, charts, "console_back", (0.0, -0.07, -0.2), (0, 0, -1))
    assert albedo[round(by), round(bx), :3].mean() > 3.0 * albedo[round(vy), round(vx), :3].mean()
