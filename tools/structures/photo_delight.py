"""De-lit photo projection onto a UV chart (fidelity plan Task 6): a measuring tool, nothing it reads ships.

`delight(photo_ids, part_uv_chart, cameras)` projects every texel of a chart (its 3D point and normal in the capsule cabin frame) through each
photo's fitted camera (config/cockpits/photo_cameras.toml, cabin_fit.camera: pinhole or equidistant fisheye), samples the photo (sRGB decoded to
linear), divides by a shading estimate of the same pose and takes the median over the photos that see the texel. Result: (H, W, 3) linear albedo
(NaN where no photo sees the texel), up to the photo's exposure (one scale per photo; `measure` calibrates it on the white wall).

Shading estimate: either our render of the same pose (`shading={photo id: (h, w) or (h, w, 3) linear image at the photo's resolution}`, sampled
at the same pixel as the photo) or, by default, `cabin_shading(cockpit)`: the irradiance our cabin lights give a white Lambert surface at the
texel (CockpitView's lamp set: the two console lamps, the eight ring omnis, the two fill omnis; no shadows), which is what our renderer computes
for that point before the albedo.

Licence rule (spec "Answered", CLAUDE.md "only free data"): the fitted cameras exist for crew1_1 / discovery2 / discovery10 (Shane Mielke /
Discovery stills, NOT public domain). So no pixel of a de-lit projection is written into a shipped map: `measure` (run by hand,
`python -I tools/structures/photo_delight.py --out <dir>`) prints the derived NUMBERS (mean albedo ratios against the white wall, the fine
structure's contrast and correlation length, the vent / panel positions read on the rectified views) that cabin_textures.py bakes its own
maps from, and writes the rectified / de-lit views as work images under screenshots/ (git-ignored).
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "blender"))
from cabin_fit.camera import Camera, project, unproject  # noqa: E402,F401

ROOT = HERE.parents[1]
LAMP_LUMENS = 3000.0  # [D] = CockpitView.LAMP_LUMENS (game/cockpit/cockpit_view.gd)
LAMP_DROP_M = 0.08  # [D] = CockpitView.LAMP_DROP_M
RING_LUMENS = 6000.0  # [D] = CockpitView.RING_LUMENS, over RING_LAMPS omnis
RING_LAMPS = 8  # [D] = CockpitView.RING_LAMPS
RING_DROP_M = 0.30  # [D] = CockpitView.RING_DROP_M (Task 9 fix 1: under the cove light)
RING_IN_M = 0.25  # [D] = CockpitView.RING_IN_M (in from the side wall)
FILL_LUMENS = 7000.0  # [D] = CockpitView.FILL_LUMENS (fill_ceiling, fill_seats)
FILL_LUX = 450.0  # [D] = CockpitView.FILL_LUX: the walls' bounced light as a flat illuminance


@dataclass
class Chart:
    """A planar UV chart: texel (i, j) (row i down from the top, column j) lies at origin + a_axis * a + b_axis * b + normal * depth(a, b) with
    a = a0 + (j + 0.5) * texel, b = b1 - (i + 0.5) * texel (b up the image). Vectors in the cabin frame."""
    origin: np.ndarray
    a_axis: np.ndarray
    b_axis: np.ndarray
    normal: np.ndarray
    a_range: tuple
    b_range: tuple
    texel: float
    depth: object = None  # callable(a, b) -> offset along normal (m), or None (flat)

    @property
    def shape(self):
        return (int(round((self.b_range[1] - self.b_range[0]) / self.texel)), int(round((self.a_range[1] - self.a_range[0]) / self.texel)))

    def ab(self):
        h, w = self.shape
        a = self.a_range[0] + (np.arange(w) + 0.5) * self.texel
        b = self.b_range[1] - (np.arange(h) + 0.5) * self.texel
        return np.meshgrid(a, b)

    def points(self):
        """(points (H, W, 3), normals (H, W, 3))."""
        A, B = self.ab()
        d = np.zeros_like(A) if self.depth is None else np.asarray(self.depth(A, B), float) * np.ones_like(A)
        p = (np.asarray(self.origin, float) + A[..., None] * np.asarray(self.a_axis, float) + B[..., None] * np.asarray(self.b_axis, float)
             + d[..., None] * np.asarray(self.normal, float))
        n = np.broadcast_to(np.asarray(self.normal, float), p.shape).copy()
        return p, n


def srgb_to_linear(c):
    c = np.asarray(c, float)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def load_photo(row) -> np.ndarray:
    """The photo of a camera row as (h, w, 3) linear floats (reads the git-ignored reference file)."""
    from PIL import Image
    im = Image.open(ROOT / row["file"]).convert("RGB")
    return srgb_to_linear(np.asarray(im, float) / 255.0)


def sample(img: np.ndarray, px: np.ndarray) -> np.ndarray:
    """Bilinear sample of img (h, w[, c]) at pixel coordinates px (..., 2) (pixel centres at +0.5); NaN outside."""
    h, w = img.shape[:2]
    x, y = px[..., 0] - 0.5, px[..., 1] - 0.5
    ok = np.isfinite(x) & np.isfinite(y) & (x >= 0) & (y >= 0) & (x <= w - 1) & (y <= h - 1)
    xs, ys = np.where(ok, x, 0.0), np.where(ok, y, 0.0)
    x0, y0 = np.clip(np.floor(xs).astype(int), 0, w - 2), np.clip(np.floor(ys).astype(int), 0, h - 2)
    fx, fy = xs - x0, ys - y0
    if img.ndim == 3:
        fx, fy = fx[..., None], fy[..., None]
    v = (img[y0, x0] * (1 - fx) * (1 - fy) + img[y0, x0 + 1] * fx * (1 - fy) + img[y0 + 1, x0] * (1 - fx) * fy + img[y0 + 1, x0 + 1] * fx * fy)
    nan = np.nan * np.ones_like(v)
    return np.where(ok[..., None] if img.ndim == 3 else ok, v, nan)


def cove_ring(cab) -> tuple:
    """(x, r) of the ring omnis = CockpitView.cove_ring: RING_DROP_M below the middle of [cabin] cove_light_x_m and RING_IN_M in from the side
    wall there (fidelity Task 9)."""
    x = 0.5 * sum(cab["cove_light_x_m"])
    run = [p for p in cab["profile_m"][1:] if p[0] <= cab["ceiling_x_m"] + 1e-9]
    return x - RING_DROP_M, float(np.interp(x, [p[0] for p in run], [p[1] for p in run])) - RING_IN_M


def cabin_lights(cockpit) -> list:
    """[(position (3,), luminous intensity cd)] of CockpitView's omni set (isotropic: I = lumens / 4 pi)."""
    out = []
    n = np.asarray(cockpit["console"]["normal"], float)
    for row in cockpit["light"]:
        out.append((np.asarray(row["position_m"], float) + n * LAMP_DROP_M, LAMP_LUMENS / (4 * math.pi)))
    cab = cockpit["cabin"]
    ring_x, ring_r = cove_ring(cab)
    for k in range(RING_LAMPS):
        az = 2 * math.pi * k / RING_LAMPS
        out.append((np.array([ring_x, ring_r * math.cos(az), ring_r * math.sin(az)]), RING_LUMENS / RING_LAMPS / (4 * math.pi)))
    out.append((np.array([cab["ceiling_x_m"] - 0.25, 0.0, 0.0]), FILL_LUMENS / (4 * math.pi)))
    out.append((np.array([2.0, 0.55, 0.0]), FILL_LUMENS / (4 * math.pi)))
    return out


def cabin_shading(cockpit):
    """shading(points, normals) -> (...,) irradiance (lux) on a white Lambert surface from CockpitView's lights (no shadows) + the fill."""
    lights = cabin_lights(cockpit)

    def shading(points, normals):
        e = np.full(points.shape[:-1], FILL_LUX)
        for pos, cd in lights:
            d = pos - points
            r2 = np.maximum(np.sum(d * d, axis=-1), 1e-4)
            cos = np.sum(d * normals, axis=-1) / np.sqrt(r2)
            e = e + cd * np.clip(cos, 0.0, None) / r2
        return e
    return shading


def project_photo(chart: Chart, cam: Camera, photo: np.ndarray, shading) -> np.ndarray:
    """One photo's de-lit view of the chart: photo / shading at every texel the camera sees (in frame, facing it), NaN elsewhere."""
    p, n = chart.points()
    px = project(p.reshape(-1, 3), cam).reshape(p.shape[:-1] + (2,))
    seen = np.sum((cam.position_m - p) * n, axis=-1) > 0.0
    rgb = sample(photo, px)
    if callable(shading):
        s = shading(p, n)
    else:
        s = sample(np.asarray(shading, float), px)
    if s.ndim == rgb.ndim - 1:
        s = s[..., None]
    with np.errstate(divide="ignore", invalid="ignore"):
        out = rgb / np.where(s > 0, s, np.nan)
    out[~seen] = np.nan
    return out


def delight(photo_ids, part_uv_chart: Chart, cameras: dict, shading=None, photos: dict | None = None, cockpit=None) -> np.ndarray:
    """(H, W, 3) linear albedo of the chart: per photo (camera rows in `cameras`, id -> photo_cameras.toml row) photo / shading at the texel's
    projection, median over the photos that see it (NaN where none does). shading: None = cabin_shading(cockpit); a callable(points, normals);
    or {photo id: linear image at the photo's resolution} (our render of the same pose). photos: {id: linear image} (default: load the row's file)."""
    views = []
    for pid in photo_ids:
        row = cameras[pid]
        cam = Camera.from_row(row)
        img = photos[pid] if photos and pid in photos else load_photo(row)
        if shading is None:
            sh = cabin_shading(cockpit)
        elif isinstance(shading, dict):
            sh = shading[pid]
        else:
            sh = shading
        views.append(project_photo(part_uv_chart, cam, img, sh))
    stack = np.stack(views)
    with np.errstate(all="ignore"):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            return np.nanmedian(stack, axis=0)


# ---------- measurement (run by hand; prints numbers, writes work images to screenshots/) ----------

def console_chart(cockpit, name, texel, o=None):
    """A Chart over one of cabin_layout.console_charts' planar charts (name), at console offset o (default: the chart's own plane)."""
    import cabin_layout as cl
    f = cl.console_frame(cockpit)
    ch = cl.console_charts(cockpit)[name]
    axis = {"lat": np.asarray(f["right"]), "u": np.asarray(f["up"]), "o": np.asarray(f["normal"])}
    (an, asg), (bn, bsg) = ch["a"], ch["b"]
    normal_name = ({"lat", "u", "o"} - {an, bn}).pop()
    plane = ch["plane"] if o is None else o
    origin = np.asarray(f["origin"]) + axis[normal_name] * plane
    return Chart(origin, axis[an] * asg, axis[bn] * bsg, axis[normal_name] * ch["facing"], tuple(ch["a_range"]), tuple(ch["b_range"]), texel)


def _rows(cams_file=None):
    import tomllib
    with open(cams_file or ROOT / "config" / "cockpits" / "photo_cameras.toml", "rb") as fh:
        return {r["id"]: r for r in tomllib.load(fh)["photo"]}


def _save(img, path, gain=1.0):
    from PIL import Image
    c = np.nan_to_num(np.clip(img * gain, 0, 1), nan=0.0)
    s = np.where(c <= 0.0031308, 12.92 * c, 1.055 * np.power(c, 1 / 2.4) - 0.055)
    Image.fromarray(np.round(s * 255).astype(np.uint8)).save(path)


def structure_stats(albedo: np.ndarray, texel: float):
    """(relative rms contrast of the luminance after removing scales > 20 mm, correlation length m) of a de-lit patch."""
    from scipy.ndimage import gaussian_filter
    y = np.nanmean(albedo, axis=-1)
    y = np.where(np.isfinite(y), y, np.nanmean(y))
    low = gaussian_filter(y, 0.02 / texel)
    rel = y / np.maximum(low, 1e-6) - 1.0
    rms = float(np.sqrt(np.mean(rel ** 2)))
    ac = np.fft.ifft2(np.abs(np.fft.fft2(rel - rel.mean())) ** 2).real
    ac /= ac[0, 0]
    row = ac[0, : ac.shape[1] // 2]
    k = int(np.argmax(row < 1 / math.e)) if np.any(row < 1 / math.e) else len(row)
    return rms, k * texel


def measure(out_dir: Path):
    import vehicle_spec
    cockpit = vehicle_spec.load("crew_dragon")["cockpit"]
    rows = _rows()
    out_dir.mkdir(parents=True, exist_ok=True)
    sh = cabin_shading(cockpit)
    res = {}
    for name, photos, texel in (("front", ["discovery2", "discovery10"], 0.0005), ("tray_face", ["discovery2", "discovery10"], 0.0005),
                                ("back", ["crew1_1"], 0.002), ("under", ["crew1_1"], 0.002)):
        ch = console_chart(cockpit, name, texel)
        alb = delight(photos, ch, rows, shading=sh)
        res[name] = (ch, alb)
        _save(alb / np.nanpercentile(alb, 99), out_dir / f"delit_{name}.png")
        print(f"{name}: {alb.shape}, seen {np.mean(np.isfinite(alb[..., 0])):.2f}")
    return res


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "screenshots" / "m7-cockpit" / "fidelity" / "t6" / "work"))
    measure(Path(ap.parse_args().out))
