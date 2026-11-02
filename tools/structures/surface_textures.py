"""Baked surfaces of the Falcon 9 and the Crew Dragon (numpy, deterministic, no photo pixels, no logos or lettering): the silhouette is the mesh, the fine structure is here.

Written by the driver into data/models/<id>/. UVs are metres (arc at the part's radius, station); a map repeats over its period. Mips are made by the game with box averages, which is why every value is
LINEAR data: the shader reads
    RGB   multiplier of the paint's albedo, stored / `gain` (a dark part with bright fasteners needs a gain above 1)
    A     factor of the paint's roughness (the array's cover glass against its bus bars)
and `f9_soot.png` (one channel, whole stage): the amount of the soot's albedo and roughness mixed in. No normal maps, so nothing here can glint from pixel to pixel.

Numbers come from the `[surface]` table of the vehicle's detail file; every line there names its photo.
"""
from __future__ import annotations

import math
import struct
import zlib
from pathlib import Path

import numpy as np

import hull_textures
import vehicle_spec

png_bytes = hull_textures.png_bytes
SEED = 20261102

FALCON9_SURFACE_KEYS = vehicle_spec.FALCON9_SURFACE_KEYS
DRAGON_SURFACE_KEYS = vehicle_spec.DRAGON_SURFACE_KEYS

# Files and sizes (width, height): texel = period / size.
HULL_FILE, JOINT_FILE, INTERSTAGE_FILE, OCTAWEB_FILE, STAGE2_FILE, LEG_FILE, SOOT_FILE = "f9_hull.png", "f9_joint.png", "f9_interstage.png", "f9_octaweb.png", "f9_stage2.png", "f9_leg.png", "f9_soot.png"
FALCON9_SIZES = {HULL_FILE: (1024, 1024), JOINT_FILE: (128, 256), INTERSTAGE_FILE: (1024, 1024), OCTAWEB_FILE: (512, 512), STAGE2_FILE: (512, 512), LEG_FILE: (256, 1024), SOOT_FILE: (256, 1024)}
SKIN_FILE, SHIELD_FILE, BAND_FILE, ARRAY_FILE, RADIATOR_FILE = "dragon_skin.png", "dragon_shield.png", "dragon_band.png", "dragon_array.png", "dragon_radiator.png"
DRAGON_SIZES = {SKIN_FILE: (2048, 1024), SHIELD_FILE: (512, 512), BAND_FILE: (256, 256), ARRAY_FILE: (512, 512), RADIATOR_FILE: (128, 1024)}
SKIN_LENGTH_M = 5.0  # the capsule's 4.85 m and a margin
SKIN_ARC_M = 4.0 * math.pi  # the skin's UV: 2 m per radian (gen_dragon.py)
SHIELD_SPAN_M = 4.2  # planar UV across the shield, the 4.0 m diameter and a margin
BAND_PERIOD_M = (math.pi / 6.0, 1.0)
OCTAWEB_TILES_AROUND = 8  # the outer engines' spacing
NOISE_AMPLITUDE = 0.006  # [E] paint's orange peel, below what the photos resolve: a surface that is not a flat colour at arm's length


def _encode(rgb: np.ndarray, rough: np.ndarray, gain: float = 1.0) -> np.ndarray:
    out = np.empty((*rough.shape, 4))
    out[..., :3] = rgb / gain
    out[..., 3] = rough
    return np.rint(np.clip(out, 0.0, 1.0) * 255.0).astype(np.uint8)


def _coverage(distance_m: np.ndarray, half_m: float, texel_m: float) -> np.ndarray:
    """Box-filtered coverage of a line of half width half_m at this distance: anti-aliased at the texel."""
    return np.clip((half_m - np.abs(distance_m)) / texel_m + 0.5, 0.0, 1.0)


def _wrapped(coordinate: np.ndarray, period: float) -> np.ndarray:
    """Signed distance to the nearest multiple of the period."""
    return (coordinate + 0.5 * period) % period - 0.5 * period


def _centres(n: int, period: float) -> np.ndarray:
    return (np.arange(n) + 0.5) * period / n


def _periodic_noise(rng, cells_u: int, cells_v: int, w: int, h: int) -> np.ndarray:
    """Smooth periodic noise in 0..1 with this many cells (hull_textures' value noise)."""
    return hull_textures._periodic_noise(rng, cells_u, cells_v, w, h)


def _dots(shape, period, positions, radius_m: float, wrap=(True, True)) -> np.ndarray:
    """Coverage of round dots at (u, v) metres on a map of `shape` (h, w) over `period` (u, v). Each dot touches a small window only."""
    h, w = shape
    du, dv = period[0] / w, period[1] / h
    out = np.zeros(shape)
    reach_u, reach_v = int(radius_m / du) + 2, int(radius_m / dv) + 2
    for pu, pv in positions:
        cu, cv = int(pu / du), int(pv / dv)
        cols = np.arange(cu - reach_u, cu + reach_u + 1)
        rows = np.arange(cv - reach_v, cv + reach_v + 1)
        if wrap[0]:
            cols = cols % w
        else:
            cols = cols[(cols >= 0) & (cols < w)]
        if wrap[1]:
            rows = rows % h
        else:
            rows = rows[(rows >= 0) & (rows < h)]
        if cols.size == 0 or rows.size == 0:
            continue
        x = _wrapped((cols + 0.5) * du - pu, period[0])
        y = _wrapped((rows + 0.5) * dv - pv, period[1])
        field = np.clip((radius_m - np.hypot(x[None, :], y[:, None])) / min(du, dv) + 0.5, 0.0, 1.0)
        out[np.ix_(rows, cols)] = np.maximum(out[np.ix_(rows, cols)], field)
    return out


def _lerp(table, x):
    xs = [p[0] for p in table]
    ys = [p[1] for p in table]
    return np.interp(x, xs, ys)


# ---- Falcon 9 ----

def _radius_f9(spec):
    return spec["stages"]["first_stage"]["diameter_m"] / 2.0


def hull_period_m(spec):
    s = spec["detail"]["surface"]
    circ = 2.0 * math.pi * _radius_f9(spec)
    return circ / s["hull_tiles_around"], s["hull_ring_pitch_m"]


def hull_tile(spec) -> np.ndarray:
    """White paint over the tank's panels: faint ring and long weld lines, a shade per sheet."""
    s = spec["detail"]["surface"]
    w, h = FALCON9_SIZES[HULL_FILE]
    pu, pv = hull_period_m(spec)
    rng = np.random.default_rng(SEED)
    u, v = _centres(w, pu), _centres(h, pv)
    line = np.maximum(_coverage(_wrapped(v, pv), s["hull_line_half_m"], pv / h)[:, None],
                      0.6 * _coverage(_wrapped(u, pu), s["hull_line_half_m"], pu / w)[None, :])
    tone = 1.0 + NOISE_AMPLITUDE * (2.0 * _periodic_noise(rng, 64, 64, w, h) - 1.0) - 0.012 * _periodic_noise(rng, 3, 3, w, h)
    value = tone * (1.0 - s["hull_line_depth"] * line)
    return _encode(np.dstack([value] * 3), np.ones_like(value))


def joint_period_m(spec):
    s = spec["detail"]["surface"]
    circ = 2.0 * math.pi * _radius_f9(spec)
    count = round(circ / s["joint_rivet_pitch_m"] / 2.0)  # two rivets per tile
    return circ / count, s["joint_height_m"]


def joint_tile(spec) -> np.ndarray:
    """The rivet joint under the interstage (F_ORBCOMM_2, F_Falcon_9_first_stage_in_hangar: one dark row around the hull)."""
    s = spec["detail"]["surface"]
    w, h = FALCON9_SIZES[JOINT_FILE]
    pu, pv = joint_period_m(spec)
    row_v = (spec["detail"]["hull"]["interstage_base_m"] - 0.5 * pv) % pv
    dots = _dots((h, w), (pu, pv), [(0.25 * pu, row_v), (0.75 * pu, row_v)], s["joint_rivet_radius_m"])
    value = 1.0 - s["joint_rivet_depth"] * dots
    return _encode(np.dstack([value] * 3), np.ones_like(value))


def interstage_period_m(spec):
    s = spec["detail"]["surface"]
    stage, hull = spec["stages"]["first_stage"], spec["detail"]["hull"]
    circ = 2.0 * math.pi * _radius_f9(spec)
    return circ / s["interstage_tiles_around"], (stage["length_m"] - hull["interstage_base_m"]) / s["interstage_rows"]


def interstage_tile(spec) -> np.ndarray:
    """The black interstage: composite plates with dotted edges (F_Falcon_9_B1046), a fine lengthwise grain; dark, with `gain` for the lighter fasteners."""
    s = spec["detail"]["surface"]
    w, h = FALCON9_SIZES[INTERSTAGE_FILE]
    pu, pv = interstage_period_m(spec)
    gain = s["interstage_gain"]
    rng = np.random.default_rng(SEED + 1)
    u, v = _centres(w, pu), _centres(h, pv)
    grain = 1.0 + 0.10 * (2.0 * _periodic_noise(rng, 128, 8, w, h) - 1.0) + 0.05 * (2.0 * _periodic_noise(rng, 32, 32, w, h) - 1.0)
    line = np.maximum(_coverage(_wrapped(v, pv), 0.004, pv / h)[:, None], _coverage(_wrapped(u, pu), 0.004, pu / w)[None, :])
    pitch = s["interstage_rivet_pitch_m"]
    inset = 0.04  # [E] the row beside the plate's edge
    nu, nv = max(1, round(pu / pitch)), max(1, round(pv / pitch))
    positions = [(pu * i / nu, inset) for i in range(nu)] + [(pu * i / nu, pv - inset) for i in range(nu)]
    positions += [(inset, pv * i / nv) for i in range(nv)] + [(pu - inset, pv * i / nv) for i in range(nv)]
    dots = _dots((h, w), (pu, pv), positions, s["interstage_rivet_radius_m"])
    value = (1.0 / gain) * grain * (1.0 - 0.5 * line)
    value = value * (1.0 - dots) + dots * (2.0 / gain)
    return _encode(np.dstack([value] * 3), 0.9 + 0.1 * grain.clip(0.0, 1.0))


def octaweb_period_m(spec):
    d = spec["detail"]
    return 2.0 * math.pi * _radius_f9(spec) / OCTAWEB_TILES_AROUND, d["octaweb"]["cover_top_m"] - d["hull"]["base_station_m"]


def octaweb_tile(spec) -> np.ndarray:
    """The black aft cover: plates and bolt rows per engine bay (F_Falcon_9_B1046)."""
    s = spec["detail"]["surface"]
    w, h = FALCON9_SIZES[OCTAWEB_FILE]
    pu, pv = octaweb_period_m(spec)
    gain = s["octaweb_gain"]
    base = spec["detail"]["hull"]["base_station_m"] % pv
    rng = np.random.default_rng(SEED + 2)
    u, v = _centres(w, pu), _centres(h, pv)
    tone = 1.0 + 0.08 * (2.0 * _periodic_noise(rng, 16, 16, w, h) - 1.0)
    lines = np.maximum(_coverage(_wrapped(u - 0.5 * pu, pu), 0.006, pu / w)[None, :], _coverage(_wrapped(v - base - 0.5 * pv, pv), 0.006, pv / h)[:, None])
    bolts = [(pu * (i + 0.5) / 8.0, (base + pv * f) % pv) for i in range(8) for f in (0.12, 0.45, 0.55, 0.88)]
    dots = _dots((h, w), (pu, pv), bolts, 0.018)
    value = (1.0 / gain) * tone * (1.0 - 0.6 * lines)
    value = value * (1.0 - dots) + dots * (2.5 / gain)
    return _encode(np.dstack([value] * 3), np.ones_like(value))


def stage2_period_m(spec):
    s = spec["detail"]["surface"]
    stage = spec["stages"]["second_stage"]
    return math.pi * stage["diameter_m"] / s["hull_tiles_around"], stage["length_m"] / s["stage2_rows"]


def stage2_tile(spec) -> np.ndarray:
    """White second stage: as the first stage's tank, on its own ring pitch."""
    s = spec["detail"]["surface"]
    w, h = FALCON9_SIZES[STAGE2_FILE]
    pu, pv = stage2_period_m(spec)
    rng = np.random.default_rng(SEED + 3)
    u, v = _centres(w, pu), _centres(h, pv)
    line = np.maximum(_coverage(_wrapped(v, pv), s["hull_line_half_m"], pv / h)[:, None],
                      0.6 * _coverage(_wrapped(u, pu), s["hull_line_half_m"], pu / w)[None, :])
    tone = 1.0 + NOISE_AMPLITUDE * (2.0 * _periodic_noise(rng, 32, 32, w, h) - 1.0)
    value = tone * (1.0 - s["hull_line_depth"] * line)
    return _encode(np.dstack([value] * 3), np.ones_like(value))


def leg_tile(spec) -> np.ndarray:
    """The carbon leg (UV 0..1 per face: u across, v along): edge lines, a centre line, a light grain (c_nhq_base: near black, soft sheen)."""
    s = spec["detail"]["surface"]
    w, h = FALCON9_SIZES[LEG_FILE]
    gain = s["leg_gain"]
    rng = np.random.default_rng(SEED + 4)
    u = _centres(w, 1.0)
    grain = 1.0 + 0.10 * (2.0 * _periodic_noise(rng, 64, 8, w, h) - 1.0)
    lines = np.maximum(_coverage(u - 0.5, 0.004, 1.0 / w), np.maximum(_coverage(u - 0.04, 0.004, 1.0 / w), _coverage(u - 0.96, 0.004, 1.0 / w)))[None, :]
    value = (1.0 / gain) * grain * (1.0 - 0.5 * lines)
    return _encode(np.dstack([value] * 3), 0.85 + 0.15 * grain.clip(0.0, 1.0))


def soot_length_m(spec) -> float:
    return spec["stages"]["first_stage"]["length_m"]


def soot_map(spec) -> np.ndarray:
    """Amount of soot over the whole stage, one channel: `soot_level` x the profile along the stage x the windward side x lengthwise streaks. 0 everywhere at level 0."""
    s = spec["detail"]["surface"]
    w, h = FALCON9_SIZES[SOOT_FILE]
    circ = 2.0 * math.pi * _radius_f9(spec)
    rng = np.random.default_rng(SEED + 5)
    profile = _lerp(s["soot_profile_m"], _centres(h, soot_length_m(spec)))
    az = _centres(w, 2.0 * math.pi)
    windward = 1.0 - s["soot_windward_share"] * 0.5 * (1.0 - np.cos(az - math.radians(s["soot_windward_azimuth_deg"])))
    streak = 0.45 * _periodic_noise(rng, 32, 2, w, h) + 0.35 * _periodic_noise(rng, 96, 4, w, h) + 0.2 * _periodic_noise(rng, 8, 8, w, h)
    field = 0.7 * _periodic_noise(rng, 5, 12, w, h) + 0.3 * _periodic_noise(rng, 24, 48, w, h)
    blotch = np.clip((field - 0.42) / 0.14, 0.0, 1.0)  # [E] landing frame cr_3475: dark areas with hard-ish ends, white between them
    amount = s["soot_level"] * profile[:, None] * windward[None, :] * (0.3 + 1.4 * streak) * (0.25 + 0.95 * blotch)
    return np.rint(np.clip(amount, 0.0, 1.0) * 255.0).astype(np.uint8)


def write_falcon9(out, spec: dict) -> dict:
    """Bakes the maps into `out`; returns the manifest's {paint key: entry}."""
    out = Path(out)
    s = spec["detail"]["surface"]
    circ = 2.0 * math.pi * _radius_f9(spec)
    maps = {HULL_FILE: hull_tile(spec), JOINT_FILE: joint_tile(spec), INTERSTAGE_FILE: interstage_tile(spec), OCTAWEB_FILE: octaweb_tile(spec), STAGE2_FILE: stage2_tile(spec), LEG_FILE: leg_tile(spec)}
    for name, image in maps.items():
        (out / name).write_bytes(png_bytes(image))
    (out / SOOT_FILE).write_bytes(gray_png_bytes(soot_map(spec)))
    wear = {"wear_file": SOOT_FILE, "wear_period_m": [circ, soot_length_m(spec)], "wear_albedo": s["soot_albedo"], "wear_roughness": s["soot_roughness"]}
    return {"paint_hull": {"file": HULL_FILE, "period_m": list(hull_period_m(spec)), "gain": 1.0, **wear},
            "paint_joint": {"file": JOINT_FILE, "period_m": list(joint_period_m(spec)), "gain": 1.0, **wear},
            "paint_interstage": {"file": INTERSTAGE_FILE, "period_m": list(interstage_period_m(spec)), "gain": s["interstage_gain"], **wear},
            "paint_octaweb": {"file": OCTAWEB_FILE, "period_m": list(octaweb_period_m(spec)), "gain": s["octaweb_gain"]},
            "paint_leg": {"file": LEG_FILE, "period_m": [1.0, 1.0], "gain": s["leg_gain"]},
            "paint_hull2": {"file": STAGE2_FILE, "period_m": list(stage2_period_m(spec)), "gain": 1.0}}


def gray_png_bytes(image: np.ndarray) -> bytes:
    """8-bit grey PNG (filter 0, zlib level 9), same writer rules as the colour maps."""
    h, w = image.shape
    raw = b"".join(b"\x00" + image[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


# ---- Crew Dragon ----

def shield_albedo(detail: dict) -> list:
    """New (the photos' pale cover) to charred (paint.shield_albedo) by flown_soot."""
    s = detail["surface"]
    f = s["flown_soot"]
    return [a * (1.0 - f) + b * f for a, b in zip(s["shield_new_albedo"], detail["paint"]["shield_albedo"])]


def _skin_grid(w, h):
    """(azimuth deg, station m) of the skin map's texels; 2 m of arc per radian."""
    az = np.degrees(_centres(w, SKIN_ARC_M) / 2.0)
    x = _centres(h, SKIN_LENGTH_M)
    return az, x


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def _superellipse_distance(az, x, az0, x0, hx, haz, power, radius_m):
    """About the distance in metres from the outline of a patch: (azimuth, x) with half sizes hx (m), haz (deg)."""
    da = _wrap_deg(az - az0)[None, :] / haz
    dx = (x - x0)[:, None] / hx
    f = (np.abs(da) ** power + np.abs(dx) ** power) ** (1.0 / power)
    return (f - 1.0) * min(hx, math.radians(haz) * radius_m)


def _outline_points(az0, x0, hx, haz, power, pitch_m, radius_m, scale=1.0):
    """Points along a superellipse's outline about pitch_m apart, as (u, v) metres on the skin map."""
    samples = 256
    t = 2.0 * np.pi * np.arange(samples) / samples
    e = 2.0 / power
    da = scale * haz * np.sign(np.sin(t)) * np.abs(np.sin(t)) ** e
    dx = scale * hx * np.sign(np.cos(t)) * np.abs(np.cos(t)) ** e
    points = []
    run = pitch_m
    for k in range(samples):
        step = math.hypot(math.radians(da[k] - da[k - 1]) * radius_m, dx[k] - dx[k - 1])
        run += step
        if run >= pitch_m:
            run = 0.0
            points.append((math.radians((az0 + da[k]) % 360.0) * 2.0, x0 + dx[k]))
    return points


def capsule_skin(spec) -> np.ndarray:
    """The whole capsule (nose included, same UV): panel lines, fastener rows, cowl edges, window frames, the seam's paint. New white unless flown_soot > 0.
    Positions: the capsule tables of the detail file; panels: `skin_panels`, D_CCP_SpaceX_Demo_2_Dragon__3_, D_Crew_2_..._mate."""
    d = spec["detail"]
    s, c = d["surface"], d["capsule"]
    w, h = DRAGON_SIZES[SKIN_FILE]
    du, dv = SKIN_ARC_M / w, SKIN_LENGTH_M / h
    az, x = _skin_grid(w, h)
    rng = np.random.default_rng(SEED + 10)
    base_r = _lerp(c["profile_m"], x)
    tone = 1.0 + NOISE_AMPLITUDE * (2.0 * _periodic_noise(rng, 128, 64, w, h) - 1.0)
    value = np.ones((h, w, 3)) * tone[..., None]
    half = s["skin_line_half_m"]
    line_depth = 0.28  # [E] D_Crew_2_..._mate: fine grey lines, far from the dark seam
    lines = np.zeros((h, w))
    positions = []
    pitch, radius = s["skin_rivet_pitch_m"], s["skin_rivet_radius_m"]
    # access panels and big hatch-side panels
    for az0, x0, hx, haz, power, mirror, rivets in s["skin_panels"]:
        for sign in ((1.0, -1.0) if mirror else (1.0,)):
            r_mid = float(np.interp(x0, [p[0] for p in c["profile_m"]], [p[1] for p in c["profile_m"]]))
            dist = _superellipse_distance(az, x, sign * az0, x0, hx, haz, power, r_mid)
            lines = np.maximum(lines, _coverage(dist, half, min(du, dv)))
            if rivets:
                positions += _outline_points(sign * az0, x0, hx, haz, power, pitch, r_mid, 0.93)
    # window frames: a line round the flange and its fasteners
    flange = c["window_flange_scale"]
    for az0, x0, hx, haz in c["windows"]:
        for sign in (1.0, -1.0):
            r_mid = float(np.interp(x0, [p[0] for p in c["profile_m"]], [p[1] for p in c["profile_m"]]))
            dist = _superellipse_distance(az, x, sign * az0, x0, hx * flange, haz * flange, 2.0, r_mid)
            lines = np.maximum(lines, _coverage(dist, half, min(du, dv)))
            positions += _outline_points(sign * az0, x0, hx, haz, 2.0, pitch, r_mid, 0.5 * (1.0 + flange))
    # cowl edges: a line along the bay's sides and top, the valley line at the lobe's edge
    pods = spec["stages"]["capsule"]["superdraco_lobes_deg"]
    bay = np.interp(x, [p[0] for p in c["bay_half_width_deg_m"]], [p[1] for p in c["bay_half_width_deg_m"]])
    margin_deg = 1.5  # [E] D_CCP (3): rim beside the bay's edge
    top = c["bay_top_x_m"]
    low = c["band_top_x_m"]
    for pod in pods:
        d_az = np.abs(_wrap_deg(az - pod))
        side = np.radians(d_az[None, :] - (bay + margin_deg)[:, None]) * base_r[:, None]
        within = ((x >= low) & (x <= top + 0.03))[:, None]
        lines = np.maximum(lines, _coverage(side, half, min(du, dv)) * within)
        cap = (x - (top + 0.03))[:, None] * np.ones((h, w))
        lines = np.maximum(lines, _coverage(cap, half, min(du, dv)) * (d_az[None, :] < bay[np.searchsorted(x, top) - 1] + margin_deg))
        for xk in np.arange(low + 0.08, top, pitch):
            b = float(np.interp(xk, [p[0] for p in c["bay_half_width_deg_m"]], [p[1] for p in c["bay_half_width_deg_m"]])) + 2.0 * margin_deg
            positions += [(math.radians((pod + sgn * b) % 360.0) * 2.0, float(xk)) for sgn in (-1.0, 1.0)]
    # rows around the body: above the band; below the seam
    rim = s["skin_rim_row_x_m"]
    n_rim = int(2.0 * math.pi * 2.0 / pitch)
    positions += [(SKIN_ARC_M * i / n_rim, rim) for i in range(n_rim)]
    n_seam = int(2.0 * math.pi * 1.3 / pitch)
    below = s["skin_seam_rivet_below_m"]
    profile_x, profile_r = [p[0] for p in c["profile_m"]], [p[1] for p in c["profile_m"]]
    for i in range(n_seam):
        a = 2.0 * math.pi * i / n_seam
        xs = c["seam_axis_x_m"]
        for _ in range(20):
            xs = c["seam_axis_x_m"] + c["seam_slope"] * float(np.interp(xs, profile_x, profile_r)) * math.cos(a)
        positions.append((a * 2.0, xs - below))
    value *= (1.0 - line_depth * lines)[..., None]
    dots = _dots((h, w), (SKIN_ARC_M, SKIN_LENGTH_M), positions, radius, wrap=(True, False))
    value *= (1.0 - 0.45 * dots)[..., None]
    # the seam's paint: red-brown line (D_Crew_2_..._mate, c_crew9_dragon)
    seam_x = c["seam_axis_x_m"] + c["seam_slope"] * base_r[:, None] * np.cos(np.radians(az))[None, :]
    seam = _coverage(x[:, None] - seam_x, s["skin_seam_paint_half_m"], dv)
    paint = np.array(s["skin_seam_paint"])
    value = value * (1.0 - seam[..., None]) + paint * seam[..., None]
    # a flown capsule: soot streaks downstream of the SuperDraco bays and the Draco slots (below them, towards the band)
    f = s["flown_soot"]
    if f > 0.0:
        soot = np.zeros((h, w))
        streak = 0.6 + 0.8 * _periodic_noise(rng, 64, 2, w, h)
        for pod in pods:
            d_az = np.abs(_wrap_deg(az - pod))[None, :]
            soot = np.maximum(soot, np.exp(-(d_az / 22.0) ** 2) * ((x > low) & (x < 3.0))[:, None] * (1.0 - (x[:, None] - low) / (3.0 - low)))
        amount = np.clip(f * 0.6 * soot * streak, 0.0, 0.85)
        value = value * (1.0 - amount[..., None])
    return _encode(value, np.ones((h, w)))


def shield_tile(spec) -> np.ndarray:
    """Heat shield seen from below (planar UV): ablator segments, rings, a soft mottle (D_CCP (3): faint lines on the underside)."""
    s = spec["detail"]["surface"]
    w, h = DRAGON_SIZES[SHIELD_FILE]
    texel = SHIELD_SPAN_M / w
    rng = np.random.default_rng(SEED + 11)
    c = _centres(w, SHIELD_SPAN_M) - 0.5 * SHIELD_SPAN_M
    r = np.hypot(c[None, :], c[:, None])
    a = np.arctan2(c[:, None], c[None, :])
    mottle = 1.0 + 0.08 * (2.0 * _periodic_noise(rng, 32, 32, w, h) - 1.0) + 0.04 * (2.0 * _periodic_noise(rng, 128, 128, w, h) - 1.0)
    lines = np.zeros((h, w))
    for ring in s["shield_rings_r_m"]:
        lines = np.maximum(lines, _coverage(r - ring, 0.006, texel))
    n = s["shield_segments"]
    meridian = _wrapped(a, 2.0 * math.pi / n) * r
    lines = np.maximum(lines, _coverage(meridian, 0.006, texel) * (r > s["shield_rings_r_m"][0]))
    value = np.clip(mottle, 0.0, 1.2) * (1.0 - 0.3 * lines) * 0.8
    return _encode(np.dstack([value] * 3), np.ones_like(value))


def band_tile(spec) -> np.ndarray:
    """Bare metal band above the shield: brushing around the body; roughness follows it. Its seams: every 30 deg of the map (D_CCP (3))."""
    w, h = DRAGON_SIZES[BAND_FILE]
    rng = np.random.default_rng(SEED + 12)
    brush = _periodic_noise(rng, 4, 128, w, h)
    u = _centres(w, BAND_PERIOD_M[0])
    seam = _coverage(_wrapped(u, BAND_PERIOD_M[0]), 0.004, BAND_PERIOD_M[0] / w)[None, :]
    value = (0.92 + 0.08 * brush) * (1.0 - 0.3 * seam)
    return _encode(np.dstack([value] * 3), 0.8 + 0.2 * brush)


def array_tile(spec) -> np.ndarray:
    """Solar cells under cover glass: blue-black cells with a bright fine grid, darker string gaps and a silver frame per module; the glass's factor in A for the sheen.
    D_Iss072e096331 (cell grid and mirror-like reflections), c_crew9_dragon (near black from afar)."""
    s = spec["detail"]["surface"]
    w, h = DRAGON_SIZES[ARRAY_FILE]
    period = s["array_period_m"]
    gain = s["array_gain"]
    cells = int(s["array_cells"])
    texel = period / w
    rng = np.random.default_rng(SEED + 13)
    u = _centres(w, period)
    pitch = period / cells
    grid = np.maximum(_coverage(_wrapped(u, pitch), 0.0014, texel)[None, :], _coverage(_wrapped(u, pitch), 0.0014, texel)[:, None])
    module = np.maximum(_coverage(_wrapped(u, period / 2.0), 0.004, texel)[None, :], _coverage(_wrapped(u, period / 2.0), 0.004, texel)[:, None])
    tint = np.kron(rng.uniform(0.86, 1.14, (cells, cells)), np.ones((h // cells + 1, w // cells + 1)))[:h, :w]
    value = np.dstack([tint / gain] * 3)
    bright = np.maximum(0.55 * grid, 0.9 * module)[..., None]
    value = value * (1.0 - bright) + bright * (np.array([1.0, 0.95, 0.85]) * 0.9)
    rough = s["array_glass_factor"] + (1.0 - s["array_glass_factor"]) * np.maximum(grid, module)
    return _encode(value, rough)


def radiator_period_m(spec):
    s = spec["detail"]["surface"]
    trunk = spec["stages"]["trunk"]
    circumference = math.pi * trunk["diameter_m"]
    return circumference / round(circumference / s["radiator_panel_m"]), SKIN_LENGTH_M


def radiator_tile(spec) -> np.ndarray:
    """Radiator: lengthwise panels across the map (D__iss063e026772: strips about 0.4 m wide along the axis; c_crew9_dragon: fine lengthwise lines), each with a soft shade across for the shingled rib, seams dark,
    a row of rivets at both ends."""
    d = spec["detail"]
    s, t = d["surface"], d["trunk"]
    w, h = DRAGON_SIZES[RADIATOR_FILE]
    pu, pv = radiator_period_m(spec)
    u = _centres(w, pu)
    du = pu / w
    rib = 1.0 - 0.07 * (u / pu) ** 2  # [E] the panel's shingle towards its next seam
    seam = _coverage(_wrapped(u, pu), s["radiator_line_half_m"], du)
    value = (rib * (1.0 - 0.55 * seam))[None, :] * np.ones((h, 1))
    # rows of rivets at the panels' ends (stations of the patch, D__iss063e026772 shows them on the capsule side)
    length = spec["stages"]["trunk"]["length_m"]
    rows = (t["panel_base_m"] + 0.04, length + t["array_top_x_m"] - 0.04)
    pitch = s["radiator_rivet_pitch_m"]
    n = max(1, round(pu / pitch))
    dots = _dots((h, w), (pu, pv), [(pu * (i + 0.5) / n, row) for i in range(n) for row in rows], s["radiator_rivet_radius_m"], wrap=(True, False))
    value = value * (1.0 - 0.5 * dots)
    return _encode(np.dstack([value] * 3), np.ones_like(value))


def write_dragon(out, spec: dict) -> dict:
    """Bakes the maps into `out`; returns the manifest's {paint key: entry}."""
    out = Path(out)
    s = spec["detail"]["surface"]
    maps = {SKIN_FILE: capsule_skin(spec), SHIELD_FILE: shield_tile(spec), BAND_FILE: band_tile(spec), ARRAY_FILE: array_tile(spec), RADIATOR_FILE: radiator_tile(spec)}
    for name, image in maps.items():
        (out / name).write_bytes(png_bytes(image))
    skin = {"file": SKIN_FILE, "period_m": [SKIN_ARC_M, SKIN_LENGTH_M], "gain": 1.0}
    return {"paint_capsule": dict(skin), "paint_cowl": dict(skin),
            "paint_shield": {"file": SHIELD_FILE, "period_m": [SHIELD_SPAN_M, SHIELD_SPAN_M], "gain": 1.0},
            "paint_band": {"file": BAND_FILE, "period_m": list(BAND_PERIOD_M), "gain": 1.0},
            "paint_array": {"file": ARRAY_FILE, "period_m": [s["array_period_m"], s["array_period_m"]], "gain": s["array_gain"]},
            "paint_radiator": {"file": RADIATOR_FILE, "period_m": list(radiator_period_m(spec)), "gain": 1.0}}
