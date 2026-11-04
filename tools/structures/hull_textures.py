"""Procedural steel map of the vehicle hulls (weld seams + oil-canning dents), baked from the detail file's numbers.

One RGBA8 PNG, `hull_steel.png`, tiling (seamless in both axes), no photo pixels, deterministic (fixed lattice seed,
fixed zlib level). The shader maps it with the hull's UV (arc metres at the barrel radius, station metres):

    u period = circumference / U_TILES (an integer count: the map closes around the hull), v period = ring pitch with a
    weld bead at v = 0 (so a weld at every multiple of the pitch). Texel = period / size.

Channels (linear data, no sRGB; mipmaps are plain box averages made by the engine, which is why the data are chosen
so that averaging stays correct):
    R, G  slope of the surface along u and v (dh/du, dh/dv), stored as 0.5 + slope / SLOPE_RANGE / 2
    B     slope energy sx^2 + sy^2 over SLOPE_ENERGY_RANGE (0.1). The shader's variance term is B - (mean slope)^2: zero on
          mip 0, the slope variance lost to filtering on the coarser mips (feeds the roughness: Toksvig-like, so
          glints do not shimmer)
    A     heat-affected band coverage of the weld (shader: darker, rougher)
"""
from __future__ import annotations

import math
import struct
import zlib

import numpy as np

U_TILES = 6               # [E] the map repeats this often around the hull
SIZE_U = 1024             # [E] texels along u: 4.6 mm at R = 4.5 m
SIZE_V = 512              # [E] texels along v: 3.5 mm at the 1.8 m ring pitch
SLOPE_RANGE = 0.6         # slopes are stored over +-SLOPE_RANGE / 2 (the bead's 0.24 plus dents fit)
SLOPE_ENERGY_RANGE = 0.1
SEAM_FRACTION = 0.2       # [E] longitudinal seam bead height as a fraction of the ring weld's (photos: fainter than the rings)
SEAM_HAZ_SHARE = 0.3      # [E] heat-affected darkening of the longitudinal seam relative to the ring weld's
HAZ_FACTOR = 1.2          # [E] heat-affected band half width / bead half width
DENT_OCTAVES = ((4, 2, 1.0), (12, 5, 1.085))  # [E] (cells along u, cells along v, weight): ~1.2 m and ~0.38 m waves
SEED = 20241031
FILE_NAME = "hull_steel.png"
STRINGERS_PER_TILE = 12   # [E] 0.39 m pitch: NSF photos: dotted vertical lines ~0.35 +-0.1 m apart on booster and ship
DOT_PITCH_M = 0.09        # [E] +-0.02 m along the line, same photo (3.7 px at 41 px/m)
DOT_RADIUS_M = 0.018      # [E] ~4 texels: not resolved in the frames, reads as a faint line beyond ~30 m
DOT_LINE_SHARE = 0.6     # [E] share of the stringer lines that carry dots: frames (booster and ship, f12 / NSF photos): bands, not a grid
DOT_DROPOUT = 0.2        # [E] share of dots missing along a line (same frames: broken rows)
DOT_COVERAGE = 0.8        # [E] darkening share of a dot in the A channel
INSULATED_FILE_NAME = "hull_insulated.png"
INSULATED_SEED = SEED + 31
WHITE_FILE_NAME = "hull_white.png"
WHITE_SEED = SEED + 53
OVERLAY_FILE_NAME = "hull_overlay.png"
OVERLAY_SIZE = (4096, 1024)  # [E] texels around the hull (6.9 mm at R = 4.5 m, a window slot 58 texels) x along the 10 m band (9.8 mm)
BAY_RIM_M = 0.025            # [E] light rim round a thruster's port bay
BAY_RIM_ALBEDO = (0.30, 0.30, 0.32)  # [E] sRGB of the rim: bare metal edge of the cutout
FLAP_FILE_NAME = "flap_lee.png"
FLAP_SIZE = (1024, 342)   # texels along the hinge, span: the aft flap's 12 x 4 m
FLAP_PERIOD_M = (12.0, 4.0)
FLAP_COLUMN_M = 0.95      # [E] panel width: f14 file 5099 s: 115 px against the 7.2 m outer edge of 875 px
FLAP_ROW_M = 0.74         # [E] five rows over the 3.69 m span, same frame
FLAP_DIAGONALS_M = (((4.7, 0.0), (1.2, 3.69)), ((4.7, 0.0), (6.3, 3.69)))  # [E] +-0.5 m: the two stiffeners, same frame
FLAP_LINE_HALF_M = 0.025  # [E]
FLAP_SLOPE = 0.08         # [E] rise of the panel lines


def _lattice(rng, nu, nv):
    return rng.random((nv, nu))


def _periodic_noise(rng, cells_u, cells_v, size_u, size_v):
    """Smooth periodic value noise (smoothstep interpolation) on a size_v x size_u grid."""
    lat = _lattice(rng, cells_u, cells_v)
    u = np.arange(size_u) * cells_u / size_u
    v = np.arange(size_v) * cells_v / size_v
    iu, iv = np.floor(u).astype(int), np.floor(v).astype(int)
    fu, fv = u - iu, v - iv
    fu, fv = fu * fu * (3.0 - 2.0 * fu), fv * fv * (3.0 - 2.0 * fv)
    iu1, iv1 = (iu + 1) % cells_u, (iv + 1) % cells_v
    a = lat[np.ix_(iv, iu)] * (1 - fu)[None, :] + lat[np.ix_(iv, iu1)] * fu[None, :]
    b = lat[np.ix_(iv1, iu)] * (1 - fu)[None, :] + lat[np.ix_(iv1, iu1)] * fu[None, :]
    return a * (1 - fv)[:, None] + b * fv[:, None]


def periods_m(radius_m: float, ring_pitch_m: float):
    return 2.0 * math.pi * radius_m / U_TILES, ring_pitch_m


def bake(radius_m: float, hull: dict) -> np.ndarray:
    """(SIZE_V, SIZE_U, 4) uint8 array; row index = v, column = u."""
    pu, pv = periods_m(radius_m, hull["ring_pitch_m"])
    du, dv = pu / SIZE_U, pv / SIZE_V
    rng = np.random.default_rng(SEED)
    h = np.zeros((SIZE_V, SIZE_U))
    for cu, cv, w in DENT_OCTAVES:
        h += w * (_periodic_noise(rng, cu, cv, SIZE_U, SIZE_V) - 0.5) * (pu / cu + pv / cv) * 0.5
    sx = (np.roll(h, -1, axis=1) - np.roll(h, 1, axis=1)) / (2.0 * du)
    sy = (np.roll(h, -1, axis=0) - np.roll(h, 1, axis=0)) / (2.0 * dv)
    rms = math.sqrt(float(np.mean(sx * sx + sy * sy)))
    gain = hull["dent_slope_rms"] / rms
    sx, sy = sx * gain, sy * gain
    # weld bead: a triangle at v = 0 (wrapping), height weld_height_m, half width weld_half_width_m
    v = ((np.arange(SIZE_V) + 0.5) * dv + 0.5 * pv) % pv - 0.5 * pv  # signed distance to the nearest weld (m), texel centres
    half, height = hull["weld_half_width_m"], hull["weld_height_m"]
    bead_slope = -np.sign(v) * (np.abs(v) < half) * (height / half)
    sy = sy + bead_slope[:, None]
    # vertical (longitudinal) seam at u = 0: the stainless sheets of a ring meet there (one per period = 4.71 m of arc);
    # the shader shifts u per ring, so the seams stagger from ring to ring like the real panels (photos: faint vertical lines)
    u = ((np.arange(SIZE_U) + 0.5) * du + 0.5 * pu) % pu - 0.5 * pu
    seam_half, seam_height = half, SEAM_FRACTION * height
    sx = sx - (np.sign(u) * (np.abs(u) < seam_half) * (seam_height / seam_half))[None, :]
    haz_half = HAZ_FACTOR * half
    haz_v = np.clip((haz_half - np.abs(v)) / dv + 0.5, 0.0, 1.0)  # box-filtered at the texel
    haz_u = np.clip((HAZ_FACTOR * seam_half - np.abs(u)) / du + 0.5, 0.0, 1.0)
    haz = np.maximum(haz_v[:, None], SEAM_HAZ_SHARE * haz_u[None, :])
    # dotted stringer lines (spot-weld rows): at every multiple of period / STRINGERS_PER_TILE; the shader shifts rings by
    # whole stringers, so the lines run on across the rings as in the photos
    s_pitch = pu / STRINGERS_PER_TILE
    rng = np.random.default_rng(SEED + 7)
    dots = np.zeros((SIZE_V, SIZE_U))
    u_all = (np.arange(SIZE_U) + 0.5) * du
    v_all = (np.arange(SIZE_V) + 0.5) * dv
    # frames: dotted rows come in irregular groups with bare sheet between them
    for line in rng.choice(STRINGERS_PER_TILE, round(DOT_LINE_SHARE * STRINGERS_PER_TILE), replace=False):
        n_dots = max(1, round(pv / (DOT_PITCH_M * rng.uniform(0.8, 1.25))))
        d_pitch = pv / n_dots
        phase = rng.uniform(0.0, d_pitch)
        keep = rng.random(n_dots) > DOT_DROPOUT
        du_line = (u_all - (line + 0.5) * s_pitch + 0.5 * pu) % pu - 0.5 * pu
        columns = np.abs(du_line) < 2.0 * DOT_RADIUS_M
        k = np.floor((v_all - phase) / d_pitch + 0.5).astype(int)
        vs = v_all - phase - k * d_pitch
        field = np.clip((DOT_RADIUS_M - np.hypot(du_line[columns][None, :], vs[:, None])) / du + 0.5, 0.0, 1.0)
        dots[:, columns] = np.maximum(dots[:, columns], field * keep[k % n_dots][:, None])
    haz = np.maximum(haz, DOT_COVERAGE * dots)
    energy = sx * sx + sy * sy
    out = np.empty((SIZE_V, SIZE_U, 4))
    out[..., 0] = 0.5 + sx / SLOPE_RANGE
    out[..., 1] = 0.5 + sy / SLOPE_RANGE
    out[..., 2] = energy / SLOPE_ENERGY_RANGE
    out[..., 3] = haz
    if out[..., :3].min() < 0.0 or out[..., :3].max() > 1.0:
        raise ValueError("steel map: slope exceeds the stored range")
    return np.rint(out * 255.0).astype(np.uint8)


def bake_insulated(radius_m: float, hull: dict, ins: dict) -> np.ndarray:
    """(SIZE_V, SIZE_U, 4) uint8, same channels and periods as `bake`, for the depot's MLI blanket (spec["insulation"]): a quilted outer layer.

    Pillows of `pillow_width_m` x `pillow_height_m` (rounded to whole cells per period) bulge `pillow_bulge_m` (a sine hump, creased at the
    stitched seams), plus soft wrinkles (the steel map's dent octaves, scaled to `wrinkle_share` of the pillow slope rms); the seam tape
    darkens the A channel by `seam_darkening` over `seam_half_width_m` either side of every seam. No rings, no welds: a blanket."""
    pu, pv = periods_m(radius_m, hull["ring_pitch_m"])
    du, dv = pu / SIZE_U, pv / SIZE_V
    nu, nv = max(1, round(pu / ins["pillow_width_m"])), max(1, round(pv / ins["pillow_height_m"]))
    u = (np.arange(SIZE_U) + 0.5) * du
    v = (np.arange(SIZE_V) + 0.5) * dv
    fu, fv = (u * nu / pu) % 1.0, (v * nv / pv) % 1.0
    h = ins["pillow_bulge_m"] * (np.sin(math.pi * fv)[:, None] * np.sin(math.pi * fu)[None, :]) ** 0.5
    sx = (np.roll(h, -1, axis=1) - np.roll(h, 1, axis=1)) / (2.0 * du)
    sy = (np.roll(h, -1, axis=0) - np.roll(h, 1, axis=0)) / (2.0 * dv)
    # the sqrt hump has an infinite slope at the seam; the finite difference clips it at one texel, which is the crease
    pillow_rms = math.sqrt(float(np.mean(sx * sx + sy * sy)))
    rng = np.random.default_rng(INSULATED_SEED)
    w = np.zeros((SIZE_V, SIZE_U))
    for cu, cv, weight in DENT_OCTAVES:
        w += weight * (_periodic_noise(rng, cu, cv, SIZE_U, SIZE_V) - 0.5) * (pu / cu + pv / cv) * 0.5
    wx = (np.roll(w, -1, axis=1) - np.roll(w, 1, axis=1)) / (2.0 * du)
    wy = (np.roll(w, -1, axis=0) - np.roll(w, 1, axis=0)) / (2.0 * dv)
    gain = ins["wrinkle_share"] * pillow_rms / math.sqrt(float(np.mean(wx * wx + wy * wy)))
    sx, sy = sx + gain * wx, sy + gain * wy
    seam_u = np.minimum(fu, 1.0 - fu) * pu / nu  # distance to the nearest seam, metres
    seam_v = np.minimum(fv, 1.0 - fv) * pv / nv
    half = ins["seam_half_width_m"]
    tape_u = np.clip((half - seam_u) / du + 0.5, 0.0, 1.0)
    tape_v = np.clip((half - seam_v) / dv + 0.5, 0.0, 1.0)
    tape = np.maximum(tape_v[:, None], tape_u[None, :])
    lim = SLOPE_RANGE / 2.0
    sx, sy = np.clip(sx, -lim, lim), np.clip(sy, -lim, lim)
    out = np.empty((SIZE_V, SIZE_U, 4))
    out[..., 0] = 0.5 + sx / SLOPE_RANGE
    out[..., 1] = 0.5 + sy / SLOPE_RANGE
    out[..., 2] = np.clip((sx * sx + sy * sy) / SLOPE_ENERGY_RANGE, 0.0, 1.0)
    out[..., 3] = ins["seam_darkening"] * tape
    return np.rint(out * 255.0).astype(np.uint8)


def bake_white(radius_m: float, hull: dict, paint: dict) -> np.ndarray:
    """(SIZE_V, SIZE_U, 4) uint8, same channels and periods as `bake`, for the HLS's white paint (spec["white_paint"]): painted stainless.

    Faint panel lines: a V-shaped paint-filled groove (`groove_depth_m` deep, `groove_half_width_m` half wide) at every sheet edge, `panel_width_m` x
    `panel_height_m` (rounded to whole cells per period), darkening the A channel by `groove_darkening`; the steel map's dent octaves scaled to
    `wave_share` of the hull's `dent_slope_rms` for the sheets' oil-canning under the paint. No quilting, no weld beads."""
    pu, pv = periods_m(radius_m, hull["ring_pitch_m"])
    du, dv = pu / SIZE_U, pv / SIZE_V
    nu, nv = max(1, round(pu / paint["panel_width_m"])), max(1, round(pv / paint["panel_height_m"]))
    u = (np.arange(SIZE_U) + 0.5) * du
    v = (np.arange(SIZE_V) + 0.5) * dv
    cu, cv = pu / nu, pv / nv
    edge_u = ((u + 0.5 * cu) % cu) - 0.5 * cu  # signed distance to the nearest sheet edge, metres
    edge_v = ((v + 0.5 * cv) % cv) - 0.5 * cv
    half, depth = paint["groove_half_width_m"], paint["groove_depth_m"]
    sx = np.sign(edge_u) * (np.abs(edge_u) < half) * (depth / half)  # a V dip: falling towards the edge, rising after it
    sy = np.sign(edge_v) * (np.abs(edge_v) < half) * (depth / half)
    rng = np.random.default_rng(WHITE_SEED)
    w = np.zeros((SIZE_V, SIZE_U))
    for cells_u, cells_v, weight in DENT_OCTAVES:
        w += weight * (_periodic_noise(rng, cells_u, cells_v, SIZE_U, SIZE_V) - 0.5) * (pu / cells_u + pv / cells_v) * 0.5
    wx = (np.roll(w, -1, axis=1) - np.roll(w, 1, axis=1)) / (2.0 * du)
    wy = (np.roll(w, -1, axis=0) - np.roll(w, 1, axis=0)) / (2.0 * dv)
    gain = paint["wave_share"] * hull["dent_slope_rms"] / math.sqrt(float(np.mean(wx * wx + wy * wy)))
    sx, sy = sx[None, :] + gain * wx, sy[:, None] + gain * wy
    cover_u = np.clip((half - np.abs(edge_u)) / du + 0.5, 0.0, 1.0)  # box-filtered at the texel
    cover_v = np.clip((half - np.abs(edge_v)) / dv + 0.5, 0.0, 1.0)
    groove = np.maximum(cover_v[:, None], cover_u[None, :])
    lim = SLOPE_RANGE / 2.0
    sx, sy = np.clip(sx, -lim, lim), np.clip(sy, -lim, lim)
    out = np.empty((SIZE_V, SIZE_U, 4))
    out[..., 0] = 0.5 + sx / SLOPE_RANGE
    out[..., 1] = 0.5 + sy / SLOPE_RANGE
    out[..., 2] = np.clip((sx * sx + sy * sy) / SLOPE_ENERGY_RANGE, 0.0, 1.0)
    out[..., 3] = paint["groove_darkening"] * groove
    return np.rint(out * 255.0).astype(np.uint8)


def _over(rgb: np.ndarray, alpha: np.ndarray, colour, cover: np.ndarray) -> None:
    """Paints `colour` (sRGB triple) with coverage `cover` (H x W, 0..1) over the (rgb, alpha) layer in place."""
    c = np.asarray(colour, dtype=float)
    a_new = cover + alpha * (1.0 - cover)
    safe = np.where(a_new > 1e-9, a_new, 1.0)
    rgb[:] = (c[None, None, :] * cover[..., None] + rgb * (alpha * (1.0 - cover))[..., None]) / safe[..., None]
    alpha[:] = a_new


def _box(distance_m: np.ndarray, texel_m: float) -> np.ndarray:
    """Coverage of a shape from its signed inside distance (> 0 inside), box-filtered over one texel."""
    return np.clip(distance_m / texel_m + 0.5, 0.0, 1.0)


def bake_overlay(radius_m: float, hls: dict) -> np.ndarray:
    """(H, W, 4) uint8 sRGB + straight alpha of the HLS's flat hull detail, one map over the band (spec["hls_parts"]):
    u = arc metres at the barrel radius (the map spans the circumference, repeats), v = station metres from `solar_band.station_from_m` to `station_to_m`.

    Layers, bottom to top: the solar cells with their grid and the two trim lines; the dark bay round every landing thruster (the config's
    stations and azimuths) with a thin metal rim; the crew windows (light frame, dark glass)."""
    d = hls["drawing"]
    band, win, thr = d["solar_band"], d["windows"], d["landing_thruster"]
    w_px, h_px = OVERLAY_SIZE
    circ = 2.0 * math.pi * radius_m
    v0, v1 = band["station_from_m"], band["station_to_m"]
    du, dv = circ / w_px, (v1 - v0) / h_px
    u = (np.arange(w_px) + 0.5) * du
    v = v0 + (np.arange(h_px) + 0.5) * dv
    uu, vv = np.meshgrid(u, v)  # (h, w)
    rgb = np.empty((h_px, w_px, 3))
    rgb[:] = np.asarray(band["albedo"])
    alpha = np.zeros((h_px, w_px))
    _over(rgb, alpha, band["albedo"], np.ones((h_px, w_px)))
    half = band["line_half_width_m"]
    col_pitch = circ / band["columns"]
    cu = ((uu + 0.5 * col_pitch) % col_pitch) - 0.5 * col_pitch
    rv = ((vv - v0 + 0.5 * band["row_pitch_m"]) % band["row_pitch_m"]) - 0.5 * band["row_pitch_m"]
    _over(rgb, alpha, band["line_albedo"], np.maximum(_box(half - np.abs(cu), du), _box(half - np.abs(rv), dv)))
    trim = band["trim_height_m"]
    _over(rgb, alpha, band["trim_albedo"], np.maximum(_box(v0 + trim - vv, dv), _box(vv - (v1 - trim), dv)))

    def arc(a_rad):  # arc position of an azimuth (the hull's u)
        return (a_rad * radius_m) % circ

    for t in hls["thrusters"]:  # the dark bay (and its rim) round the pod, on the hull at the thruster's station and azimuth
        a = math.atan2(t["position_m"][2], t["position_m"][1])
        du_t = ((uu - arc(a)) + 0.5 * circ) % circ - 0.5 * circ
        dist = np.hypot(du_t, vv - t["position_m"][0])
        _over(rgb, alpha, BAY_RIM_ALBEDO, _box(thr["bay_radius_m"] + BAY_RIM_M - dist, du))
        _over(rgb, alpha, thr["bay_albedo"], _box(thr["bay_radius_m"] - dist, du))
    centre_u = arc(math.radians(win["azimuth_deg"]))
    for k in range(int(win["count"])):  # a row of slots centred on the azimuth: rounded rectangles (signed-distance box)
        x0 = centre_u + (k - 0.5 * (win["count"] - 1)) * win["pitch_m"]
        qx = np.abs(((uu - x0) + 0.5 * circ) % circ - 0.5 * circ) - (0.5 * win["width_m"] + win["frame_m"] - win["corner_m"])
        qy = np.abs(vv - win["station_m"]) - (0.5 * win["height_m"] + win["frame_m"] - win["corner_m"])
        outer = win["corner_m"] - (np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0)) + np.minimum(np.maximum(qx, qy), 0.0))
        _over(rgb, alpha, win["frame_albedo"], _box(outer, du))
        _over(rgb, alpha, win["glass_albedo"], _box(outer - win["frame_m"], du))
    out = np.empty((h_px, w_px, 4))
    out[..., :3] = rgb
    out[..., 3] = alpha
    return np.rint(np.clip(out, 0.0, 1.0) * 255.0).astype(np.uint8)


def bake_flap() -> np.ndarray:
    """Panel lines of the flaps' bare lee face, same channels as the hull map, not tiling: columns, rows, two stiffeners."""
    w, h = FLAP_SIZE
    du, dv = FLAP_PERIOD_M[0] / w, FLAP_PERIOD_M[1] / h
    u = (np.arange(w) + 0.5) * du
    v = (np.arange(h) + 0.5) * dv
    uu, vv = np.meshgrid(u, v)
    col = (uu + FLAP_COLUMN_M / 2.0) % FLAP_COLUMN_M - FLAP_COLUMN_M / 2.0
    row = (vv + FLAP_ROW_M / 2.0) % FLAP_ROW_M - FLAP_ROW_M / 2.0
    on_col, on_row = np.abs(col) < FLAP_LINE_HALF_M, np.abs(row) < FLAP_LINE_HALF_M
    sx = -np.sign(col) * on_col * FLAP_SLOPE
    sy = -np.sign(row) * on_row * FLAP_SLOPE
    line = np.maximum(on_col, on_row).astype(float) * 0.6
    for (u0, v0), (u1, v1) in FLAP_DIAGONALS_M:
        eu, ev = u1 - u0, v1 - v0
        length = math.hypot(eu, ev)
        dist = ((uu - u0) * ev - (vv - v0) * eu) / length
        along = ((uu - u0) * eu + (vv - v0) * ev) / length
        on = (np.abs(dist) < 2.0 * FLAP_LINE_HALF_M) & (along > 0.0) & (along < length)
        sx = sx + np.sign(dist) * on * FLAP_SLOPE * ev / length
        sy = sy - np.sign(dist) * on * FLAP_SLOPE * eu / length
        line = np.maximum(line, on * 0.8)
    out = np.empty((h, w, 4))
    out[..., 0] = 0.5 + sx / SLOPE_RANGE
    out[..., 1] = 0.5 + sy / SLOPE_RANGE
    out[..., 2] = (sx * sx + sy * sy) / SLOPE_ENERGY_RANGE
    out[..., 3] = line
    return np.rint(np.clip(out, 0.0, 1.0) * 255.0).astype(np.uint8)


def png_bytes(rgba: np.ndarray) -> bytes:
    """8-bit RGBA PNG (filter 0, zlib level 9): byte-identical for identical pixels."""
    h, w, _ = rgba.shape
    raw = b"".join(b"\x00" + rgba[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def write(out_dir, spec: dict) -> dict:
    """Bakes the map for the vehicle's spec into `out_dir`; returns the manifest's texture entry."""
    radii = {st["diameter_m"] / 2.0 for st in spec["stages"].values()}
    if len(radii) != 1:
        raise ValueError(f"steel map: one hull radius expected (the map closes around it), got {sorted(radii)}")
    radius = radii.pop()
    hull = spec["detail"]["hull"]
    (out_dir / FILE_NAME).write_bytes(png_bytes(bake(radius, hull)))
    pu, pv = periods_m(radius, hull["ring_pitch_m"])
    (out_dir / FLAP_FILE_NAME).write_bytes(png_bytes(bake_flap()))
    entry = {}
    if spec.get("insulation"):  # a variant with the insulated skin (depot): its own map, the shader numbers travel with it
        ins = spec["insulation"]
        (out_dir / INSULATED_FILE_NAME).write_bytes(png_bytes(bake_insulated(radius, hull, ins)))
        entry["insulated"] = {"file": INSULATED_FILE_NAME, "albedo": ins["albedo"], "roughness": ins["roughness"], "metallic": ins["metallic"]}
    if spec.get("white_paint"):  # the HLS: white paint map and the band / windows / bays overlay
        paint = spec["white_paint"]
        (out_dir / WHITE_FILE_NAME).write_bytes(png_bytes(bake_white(radius, hull, paint)))
        entry["white"] = {"file": WHITE_FILE_NAME, "albedo": paint["albedo"], "roughness": paint["roughness"], "metallic": paint["metallic"]}
        hls = spec["hls_parts"]
        (out_dir / OVERLAY_FILE_NAME).write_bytes(png_bytes(bake_overlay(radius, hls)))
        band = hls["drawing"]["solar_band"]
        entry["overlay"] = {"file": OVERLAY_FILE_NAME, "station_from_m": band["station_from_m"], "station_to_m": band["station_to_m"],
                            "period_u_m": 2.0 * math.pi * radius, "roughness": band["roughness"], "metallic": band["metallic"]}
    return {**entry, "file": FILE_NAME, "flap_file": FLAP_FILE_NAME, "flap_period_m": list(FLAP_PERIOD_M),
            "shift_steps": STRINGERS_PER_TILE, "period_u_m": pu, "period_v_m": pv, "size": [SIZE_U, SIZE_V], "slope_range": SLOPE_RANGE,
            "slope_energy_range": SLOPE_ENERGY_RANGE, "texel_m": [pu / SIZE_U, pv / SIZE_V]}
