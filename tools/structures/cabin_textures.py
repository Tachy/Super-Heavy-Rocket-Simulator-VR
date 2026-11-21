"""Console surface maps of the Crew Dragon cabin (fidelity plan Task 6), baked on the console atlas (cabin_layout.console_charts):

console_albedo.png (sRGB RGBA8), console_normal.png (linear RGB8: tangent-space normal, x = the chart's image right, y = the chart's image up,
z = out of the surface; 0.5 + 0.5 n), console_orm.png (linear RGBA8: R ambient occlusion, G roughness, B metallic, A emission mask: the shader
emits albedo x mask x the console's emitter luminance). 4096 x 2048 (power of two, mipmapped in the game).

Two kinds of layer (spec "Answered": drawn vs projected + de-lit):
- Drawn (vector, after the photos, positions from the fitted cameras): panel seams, screws, the locker placards with their numbers and the
  red symbol, the locker latches, the latch on the upper panel, the slots, the tray's key module plates, the small display, the connector cap,
  the red light strip on the lip, the back's dark panel, its grip, the vent strip with its separators and slats, the Velcro patch, the post
  colours. Positions (console frame of cabin_layout.console_frame: lat to the crew's right, u up the screens from the centre screen centre, m)
  were read on the de-lit views photo_delight.measure rectifies through the fitted cameras of photo_cameras.toml: the front and the tray from
  discovery2 / discovery10 (0.5 mm per texel), the back and underside from crew1_1 (2 mm per texel); [E] where a photo shows them only in the
  Hawthorne trainer (jsc2022e068617: Velcro) or a side is mirrored.
- Projected + de-lit, as NUMBERS (licence: the fitted photos are not public domain, so no pixel of them ships): the albedo ratios between the
  surfaces (deck / key plate, silver / dark panel) from the de-lit views, and the fine structure (brushed grain of the silver parts, the dark
  panel's surface) as our own deterministic noise with the contrast and correlation length the de-lit patches give as an upper bound (the
  dark patches sit near the photos' black level, so sensor and JPEG noise dominate there; the amplitudes below are set under that bound).
The key labels stay in their own atlas (cockpit_labels.png, surface_textures.cockpit_labels, per key face); the blue LEDs live on the r_led keys
(CockpitView marks), not on this atlas.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "blender"))
import cabin_layout as cl  # noqa: E402
from hull_textures import png_bytes  # noqa: E402

# ---- albedos (linear), roughness, metallic ----
DARK_PANEL = (0.030, 0.030, 0.034)  # [E] the housing's dark face and upper panel (round 5 photo measure 0.028; de-lit ratio frame / panel ~1)
FRAME_BLACK = (0.018, 0.018, 0.020)  # [E] the screens' face frame below the panel seam (discovery10: blacker than the panel)
LIGHT_BAND = (0.075, 0.075, 0.080)  # [E] the upper panel's top band above the seam at u 0.300 (discovery10 de-lit: lighter than the panel)
KEY_PLATE = (0.033, 0.033, 0.037)  # [E] the key module plates on the tray (= the keys' 0.03, conv_42860a)
DECK_GREY = (0.19, 0.19, 0.20)  # [D] tray deck: 5.75 x the key plate (discovery2 / 10 de-lit luminance ratio deck / plate_B 0.00023 / 0.00004)
SILVER = (0.35, 0.36, 0.37)  # [E] brushed silver (back ends, top, lip, post rods): crew1_1 light grey under the ring light, darker than the walls
SILVER_MID = (0.12, 0.12, 0.13)  # [E] the back's mid band between the dark panel and the vent (crew1_1 de-lit: ~4 x the dark panel)
VENT_BLACK = (0.012, 0.012, 0.013)  # [E] the vent grille (crew1_1: the blackest part of the back)
UNDER_GREY = (0.10, 0.10, 0.11)  # [E] the underside (round 4 value, crew1_1)
POST_FOOT = (0.07, 0.07, 0.075)  # [E] sleeve, funnel and collar of the ceiling posts (crew1_1: dark grey; the rod is SILVER)
RED_STRIP = (0.35, 0.02, 0.02)  # [E] the red light strip on the tray lip (= the Task 5b mark)
RED_STRIP_LAT = (-0.638, -0.347)  # [D] discovery2 / 10 de-lit front: red only under the commander's two outer modules; the rest of the lip is the deck grey
PLACARD = (0.015, 0.015, 0.017)  # [E] the locker placards (glossy black)
NUMERAL = (0.75, 0.75, 0.75)  # [E] white locker numerals
RED_SYMBOL = (0.45, 0.03, 0.03)  # [E] the red symbol left of each numeral (discovery10)
SCREW = (0.28, 0.28, 0.29)  # [E] screw heads (discovery10: light dots)
VELCRO = (0.28, 0.28, 0.28)  # [E] jsc2022e068617: a light grey fuzzy patch on the pilot-side end
DISPLAY_GLASS = (0.06, 0.10, 0.13)  # [E] the small display's lit face (discovery10: a dim grey-blue square)
CAP_BLACK = (0.02, 0.02, 0.022)  # [E] the connector's dust cap
ROUGH = {"panel": 0.85, "deck": 0.5, "silver": 0.42, "vent": 0.9, "placard": 0.3, "screw": 0.35, "velcro": 1.0, "glass": 0.15, "foot": 0.6}  # [E]
METAL = {"silver": 0.15, "deck": 0.1, "screw": 0.7}  # [E] brushed silver mostly diffuse (crew1_1: it reads light grey under the diffuse cabin light; 0.6 rendered it dark); paints 0
EMIT_MASK = {"strip": 1.0, "display": 0.25}  # [E] emission mask (x CockpitView.CONSOLE_EMIT_CD_M2 x albedo): the strip ~40 cd/m2, the display ~10

# ---- fine structure (our own noise, amplitudes under the de-lit bound) ----
PANEL_NOISE = 0.04  # [E] relative albedo rms of the dark panel's surface (de-lit bound 0.31-0.34 at 4.5-28 mm: noise-dominated, see module doc)
PANEL_NOISE_M = 0.004  # [E] its correlation length
GRAIN_NOISE = 0.06  # [E] relative albedo rms of the brushed grain (de-lit silver_mid 0.10 at 10 mm, crew1_1 at 2 mm per texel)
GRAIN_LONG_M = 0.03  # [E] grain correlation along the brushing (the console's long axis / the post axis)
GRAIN_ROUGH = 0.06  # [E] roughness rms of the grain
GRAIN_HEIGHT_M = 2.0e-6  # [E] grain relief (a whisper in the normal map: no shimmer at 0.5 m)

# ---- drawn features (console frame, m) ----
PANEL_SEAM_U = 0.300  # [D] discovery10 de-lit: the light band's lower edge (u 0.300-0.321)
SEAM_WIDTH_M = 0.0012  # [E]
SEAM_DEPTH_M = 0.0004  # [E]
SCREWS_FRONT = ((-0.4365, 0.163), (-0.002, 0.163), (0.4365, 0.163))  # [D] discovery10 (left, centre); right [E] mirrored (behind the iPad)
SCREW_RADIUS_M = 0.0032  # [D] discovery10: 13 px of 0.5 mm
TOP_LATCH = (0.0, 0.302, 0.030, 0.011)  # [D] discovery10: (lat, u, width, height) the latch at the top centre of the upper panel
SLOTS = ((0.052, 0.129, 0.217, 0.239), (-0.218, -0.157, 0.217, 0.239))  # [D] discovery10: dark slots in the upper panel (lat0, lat1, u0, u1); left faint
SLOT_DEPTH_M = 0.0015  # [E]
PLACARDS = (("5", (-0.5505, -0.468, 0.148, 0.195), (-0.4925, 0.173), (-0.534, 0.173)),
            ("6", (0.468, 0.5505, 0.148, 0.195), (0.5285, 0.173), (0.489, 0.173)))  # [D] "5" discovery10 (rect, numeral centre, symbol centre); "6" [E] mirrored
NUMERAL_HEIGHT_M = 0.022  # [D] discovery10: 44 px of 0.5 mm
SYMBOL_RADIUS_M = 0.006  # [D] discovery10
LATCHES = ((-0.4845, 0.217), (0.4845, 0.217))  # [D] left discovery10 (the round latch at the locker's inner edge); right [E] mirrored
LATCH_RADIUS_M = 0.015  # [D] discovery10: 60 px of 0.5 mm
KEY_PLATES = ((-0.6125, -0.4625), (-0.4575, -0.3025), (-0.299, -0.140), (-0.134, -0.085), (0.1425, 0.300), (0.3025, 0.455), (0.4625, 0.6125))  # [D] discovery10 tray (lat); the last [E] = mirror of the first
KEY_PLATE_U = (-0.2265, -0.170)  # [D] discovery10: the plates' bottom / top (u), clipped to the key face
PLATE_RADIUS_M = 0.004  # [E]
SMALL_DISPLAY = (-0.129, -0.0875, -0.204, -0.175)  # [D] discovery10: the lit face of the small display left of the handle (lat0, lat1, u0, u1)
CONNECTOR_CAP = (0.3175, -0.203, 0.010)  # [D] discovery10: the round dust cap on the tray (lat, u, radius)
VELCRO_PATCH = (-0.10, -0.17, 0.032, 0.030)  # [E] jsc2022e068617 (trainer, ratios to the 26 mm EXECUTE key): on the pilot-side end (o, u, width, height)
BACK_PANEL = (-0.415, 0.42, -0.015, 0.205)  # [D] crew1_1 de-lit: the back's dark panel (lat0, lat1, u0, u1)
BACK_PANEL_RIM_U = 0.155  # [D] crew1_1: the darker top rim of the panel above this u
BACK_PANEL_DIVIDERS = (-0.135, 0.14)  # [D] crew1_1: dividers in the rim (lat)
BACK_GRIP = (-0.005, -0.015, 0.022, 0.055)  # [D] crew1_1: the light grip in the middle (lat, u centre, width, height)
MID_BAND_U = (-0.125, -0.015)  # [D] crew1_1: the mid grey band
VENT = (-0.505, 0.51, -0.275, -0.125)  # [D] crew1_1: the vent strip (lat0, lat1, u0, u1)
VENT_DIVIDERS = (-0.32, -0.155, 0.005, 0.165, 0.33)  # [D] crew1_1: the separators (lat), 0.1625 apart
VENT_SLAT_PITCH_M = 0.004  # [E] slats of the grille (not resolved in crew1_1)
UNDER_SEAMS_O = (-0.07, -0.15)  # [E] crew1_1: light lines across the underside
END_SILVER_O = -0.12  # [E] the ends are silver behind this o (crew1_1 end caps), dark toward the crew (discovery: dark housing)
TEXT_FONT_SHARE = 1.38  # font size / cap height of Pillow's Aileron


def _sdf_box(x, y, x0, x1, y0, y1, r=0.0):
    """Signed distance (m, negative inside) to a rounded rectangle."""
    cx, cy, hx, hy = 0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (x1 - x0) - r, 0.5 * (y1 - y0) - r
    qx, qy = np.abs(x - cx) - hx, np.abs(y - cy) - hy
    return np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0)) + np.minimum(np.maximum(qx, qy), 0.0) - r


def _cover(d, texel):
    return np.clip(0.5 - d / texel, 0.0, 1.0)


def _noise(shape, sigma_px, seed):
    """Deterministic gaussian-filtered noise, unit rms; sigma_px = (rows, cols)."""
    rng = np.random.default_rng(seed)
    w = rng.standard_normal(shape)
    fy = np.fft.fftfreq(shape[0])[:, None]
    fx = np.fft.fftfreq(shape[1])[None, :]
    g = np.exp(-2.0 * math.pi ** 2 * ((fy * sigma_px[0]) ** 2 + (fx * sigma_px[1]) ** 2))
    n = np.fft.ifft2(np.fft.fft2(w) * g).real
    return (n / max(float(n.std()), 1e-12)).astype(np.float32)


class Layer:
    """One chart's arrays (rect + gutter): albedo (h, w, 3), rough, metal, emit, height (m), coordinates."""

    def __init__(self, chart, gutter):
        x, y, w, h = chart["rect"]
        self.x0, self.y0 = x - gutter, y - gutter
        self.w, self.h = w + 2 * gutter, h + 2 * gutter
        t = chart["texel"]
        a0 = chart["a_range"][0] * (chart["circumference_m"] if chart["kind"] == "cyl" else 1.0)
        a = a0 + (np.arange(self.w) - gutter + 0.5) * t
        b = chart["b_range"][1] - (np.arange(self.h) - gutter + 0.5) * t
        self.A, self.B = np.meshgrid(a.astype(np.float32), b.astype(np.float32))
        self.texel = t
        self.chart = chart
        self.albedo = np.zeros((self.h, self.w, 3), np.float32)
        self.rough = np.full((self.h, self.w), ROUGH["panel"], np.float32)
        self.metal = np.zeros((self.h, self.w), np.float32)
        self.emit = np.zeros((self.h, self.w), np.float32)
        self.height = np.zeros((self.h, self.w), np.float32)

    def local(self):
        """(lat, u, o) of every texel (planar charts)."""
        return cl.plane_point(self.chart, self.A, self.B)

    def paint(self, mask, albedo=None, rough=None, metal=None, emit=None):
        m = mask.astype(np.float32)
        if albedo is not None:
            self.albedo = self.albedo * (1 - m[..., None]) + np.asarray(albedo, np.float32) * m[..., None]
        if rough is not None:
            self.rough = self.rough * (1 - m) + rough * m
        if metal is not None:
            self.metal = self.metal * (1 - m) + metal * m
        if emit is not None:
            self.emit = self.emit * (1 - m) + emit * m

    def fill(self, albedo, rough, metal=0.0):
        self.albedo[...] = albedo
        self.rough[...] = rough
        self.metal[...] = metal


def _brushed(L, mask, seed, along_a=True):
    """Brushed grain on mask: long correlation along the chart's a axis (or b), texel-scale across; albedo, roughness, a whisper of relief."""
    t = L.texel
    long_px, short_px = GRAIN_LONG_M / t, 0.6
    n = _noise((L.h, L.w), (short_px, long_px) if along_a else (long_px, short_px), seed)
    L.albedo *= (1.0 + GRAIN_NOISE * n * mask)[..., None]
    L.rough += GRAIN_ROUGH * n * mask
    L.height += GRAIN_HEIGHT_M * n * mask


def _panel_noise(L, mask, seed):
    n = _noise((L.h, L.w), (PANEL_NOISE_M / L.texel,) * 2, seed)
    L.albedo *= (1.0 + PANEL_NOISE * n * mask)[..., None]
    L.rough += 0.04 * n * mask


def _groove(L, d, width, depth, albedo=(0.01, 0.01, 0.011)):
    """A seam along the zero line of d: darker and depth deep, width wide (smooth profile)."""
    s = np.clip(1.0 - np.abs(d) / width, 0.0, 1.0)
    prof = s * s * (3 - 2 * s)
    L.height -= depth * prof
    L.paint(prof * 0.8, albedo=albedo)


def _screw(L, x, y, cx, cy, r):
    d = np.hypot(x - cx, y - cy)
    head = _cover(d - r, L.texel)
    L.paint(head, albedo=SCREW, rough=ROUGH["screw"], metal=METAL["screw"])
    L.height += 0.0006 * np.clip(1.0 - (d / r) ** 2, 0.0, 1.0)
    cross = (np.minimum(np.abs(x - cx), np.abs(y - cy)) < 0.25 * r) & (d < 0.7 * r)
    L.paint(cross.astype(np.float32) * 0.9, albedo=(0.03, 0.03, 0.03))
    L.height -= 0.0003 * cross


def text_mask(text, height_m, texel, shape, centre_px):
    """Coverage (shape) of `text` in capitals whose ink box is height_m tall, its ink centre at centre_px (x, y). Pillow's Aileron (CC0)."""
    from PIL import Image, ImageDraw, ImageFont
    size = max(6, round(height_m / texel * TEXT_FONT_SHARE))
    font = ImageFont.load_default(size=size)
    pad = size
    tmp = Image.new("L", (int(font.getlength(text)) + 2 * pad, 3 * size), 0)
    ImageDraw.Draw(tmp).text((pad, size), text, fill=255, font=font)
    arr = np.asarray(tmp, np.float32) / 255.0
    ys, xs = np.nonzero(arr > 0.5)
    ink_cx, ink_cy = 0.5 * (xs.min() + xs.max() + 1), 0.5 * (ys.min() + ys.max() + 1)
    out = np.zeros(shape, np.float32)
    ox, oy = int(round(centre_px[0] - ink_cx)), int(round(centre_px[1] - ink_cy))
    h, w = arr.shape
    y0, x0 = max(0, oy), max(0, ox)
    y1, x1 = min(shape[0], oy + h), min(shape[1], ox + w)
    if y1 > y0 and x1 > x0:
        out[y0:y1, x0:x1] = arr[y0 - oy:y1 - oy, x0 - ox:x1 - ox]
    return out


def chart_px(L, a, b):
    """Pixel (x, y) in the layer's arrays of chart coordinates (a, b) (planar)."""
    t = L.texel
    return (a - L.A[0, 0]) / t, (L.B[0, 0] - b) / t


# ---------- the charts ----------

def paint_front(L, cockpit):
    lat, u, _ = L.local()
    t = L.texel
    L.fill(FRAME_BLACK, ROUGH["panel"])
    frame_top = max(s["size_m"][1] / 2.0 + s["bezel_m"] for s in cockpit["screen"]) + 0.03  # = gen_dragon_cabin FRAME_ABOVE_GLASS_M
    L.paint(_cover(frame_top - u, t), albedo=DARK_PANEL)
    L.paint(_cover(PANEL_SEAM_U - u, t), albedo=LIGHT_BAND)
    _panel_noise(L, np.ones_like(lat), 11)
    _groove(L, u - PANEL_SEAM_U, SEAM_WIDTH_M, SEAM_DEPTH_M)
    _groove(L, u - frame_top, SEAM_WIDTH_M, SEAM_DEPTH_M)
    for x0, x1, y0, y1 in SLOTS:
        d = _sdf_box(lat, u, x0, x1, y0, y1, 0.003)
        L.paint(_cover(d, t), albedo=(0.008, 0.008, 0.009), rough=0.95)
        L.height -= SLOT_DEPTH_M * _cover(d + 0.001, 2 * t)
    x, y, w, h = TOP_LATCH
    d = _sdf_box(lat, u, x - w / 2, x + w / 2, y - h / 2, y + h / 2, 0.003)
    L.paint(_cover(d, t), albedo=(0.10, 0.10, 0.11), rough=0.5, metal=0.3)
    L.height += 0.001 * _cover(d + 0.0015, 3 * t)
    for cx, cy in SCREWS_FRONT:
        _screw(L, lat, u, cx, cy, SCREW_RADIUS_M)
    for lk in cockpit["locker"]:  # the locker face's bevel (its box is geometry, the edge a texture)
        c = cl.console_local(cockpit, lk["centre_m"])
        w, h = lk["size_m"]
        d = _sdf_box(lat, u, c[0] - w / 2, c[0] + w / 2, c[1] - h / 2, c[1] + h / 2, 0.004)
        _groove(L, d + 0.002, SEAM_WIDTH_M, SEAM_DEPTH_M)
    for text, (x0, x1, y0, y1), numeral, symbol in PLACARDS:
        d = _sdf_box(lat, u, x0, x1, y0, y1, 0.004)
        L.paint(_cover(d, t), albedo=PLACARD, rough=ROUGH["placard"])
        L.height += 0.0005 * _cover(d + 0.001, 2 * t)
        ds = np.hypot(lat - symbol[0], u - symbol[1]) - SYMBOL_RADIUS_M
        L.paint(_cover(ds, t), albedo=RED_SYMBOL)
        px = chart_px(L, *cl.plane_ab(L.chart, (numeral[0], numeral[1], 0.0)))
        L.paint(text_mask(text, NUMERAL_HEIGHT_M, t, lat.shape, px), albedo=NUMERAL)
    for cx, cy in LATCHES:
        d = np.hypot(lat - cx, u - cy)
        ring = _cover(np.abs(d - LATCH_RADIUS_M * 0.85) - LATCH_RADIUS_M * 0.15, t)
        L.paint(_cover(d - LATCH_RADIUS_M, t), albedo=(0.012, 0.012, 0.013), rough=0.6)
        L.paint(ring, albedo=(0.09, 0.09, 0.095), metal=0.4, rough=0.4)
        hook = _sdf_box(lat, u, cx - 0.0025, cx + 0.0025, cy - 0.008, cy + 0.006, 0.002)
        L.paint(_cover(hook, t), albedo=(0.14, 0.14, 0.15), metal=0.5, rough=0.35)
        L.height += -0.002 * _cover(d - LATCH_RADIUS_M * 0.7, 2 * t) + 0.0015 * _cover(hook, 2 * t)


def paint_tray_face(L, cockpit):
    lat, u, _ = L.local()
    t = L.texel
    rows = cl.tray_rows(cockpit)
    L.fill(DECK_GREY, ROUGH["deck"], METAL["deck"])
    _brushed(L, np.ones_like(lat), 21)
    for x0, x1 in KEY_PLATES:
        d = _sdf_box(lat, u, x0, x1, KEY_PLATE_U[0], KEY_PLATE_U[1], PLATE_RADIUS_M)
        L.paint(_cover(d, t), albedo=KEY_PLATE, rough=ROUGH["panel"], metal=0.0)
        _groove(L, d, SEAM_WIDTH_M, SEAM_DEPTH_M)
    _panel_noise(L, _cover(np.maximum(u - KEY_PLATE_U[1], KEY_PLATE_U[0] - u), t), 22)
    x0, x1, y0, y1 = SMALL_DISPLAY
    d = _sdf_box(lat, u, x0, x1, y0, y1, 0.001)
    L.paint(_cover(d, t), albedo=DISPLAY_GLASS, rough=ROUGH["glass"], metal=0.0, emit=EMIT_MASK["display"])
    cx, cy, r = CONNECTOR_CAP
    dc = np.hypot(lat - cx, u - cy)
    L.paint(_cover(dc - r, t), albedo=CAP_BLACK, rough=0.5)
    L.paint(_cover(np.abs(dc - 0.8 * r) - 0.1 * r, t), albedo=(0.10, 0.10, 0.11), rough=0.4, metal=0.3)
    L.height += 0.002 * np.clip(1 - (dc / r) ** 2, 0, 1)
    strip = _cover(u - rows["face_bot"], t) * _cover(np.abs(lat - 0.5 * sum(RED_STRIP_LAT)) - 0.5 * (RED_STRIP_LAT[1] - RED_STRIP_LAT[0]), t)  # the lip's front
    L.paint(strip, albedo=RED_STRIP, rough=0.4, metal=0.0, emit=EMIT_MASK["strip"])


def paint_tray_top(L, cockpit):
    L.fill(DECK_GREY, ROUGH["deck"], METAL["deck"])
    _brushed(L, np.ones_like(L.A), 31)


def paint_tray_under(L, cockpit):
    L.fill(UNDER_GREY, ROUGH["panel"])


def paint_back(L, cockpit):
    lat, u, _ = L.local()
    t = L.texel
    L.fill(SILVER, ROUGH["silver"], METAL["silver"])
    _brushed(L, np.ones_like(lat), 41)
    band = (_cover(np.abs(u - 0.5 * sum(MID_BAND_U)) - 0.5 * (MID_BAND_U[1] - MID_BAND_U[0]), t)
            * _cover(np.abs(lat - 0.5 * (VENT[0] + VENT[1])) - 0.5 * (VENT[1] - VENT[0]), t))
    L.paint(band, albedo=SILVER_MID, metal=0.4)
    x0, x1, y0, y1 = BACK_PANEL
    d = _sdf_box(lat, u, x0, x1, y0, y1, 0.01)
    L.paint(_cover(d, t), albedo=DARK_PANEL, rough=ROUGH["panel"], metal=0.0)
    _groove(L, d, SEAM_WIDTH_M * 1.5, SEAM_DEPTH_M)
    rim = _cover(d, t) * _cover(BACK_PANEL_RIM_U - u, t)
    L.paint(rim, albedo=(0.018, 0.018, 0.02))
    _groove(L, (u - BACK_PANEL_RIM_U) + 10 * np.maximum(d, 0), SEAM_WIDTH_M, SEAM_DEPTH_M)
    for xd in BACK_PANEL_DIVIDERS:
        m = _cover(np.abs(lat - xd) - 0.0015, t) * _cover(BACK_PANEL_RIM_U - u, t) * _cover(d, t)
        L.paint(m, albedo=(0.06, 0.06, 0.065))
    x, y, w, h = BACK_GRIP
    dg = _sdf_box(lat, u, x - w / 2, x + w / 2, y - h / 2, y + h / 2, 0.004)
    L.paint(_cover(dg, t), albedo=SILVER, rough=0.35, metal=0.6)
    di = _sdf_box(lat, u, x - w / 4, x + w / 4, y - h / 2 + 0.008, y + h / 2 - 0.008, 0.002)
    L.paint(_cover(di, t), albedo=(0.02, 0.02, 0.02), metal=0.0, rough=0.8)
    L.height += 0.002 * _cover(dg + 0.002, 2 * t) - 0.003 * _cover(di, 2 * t)
    x0, x1, y0, y1 = VENT
    dv = _sdf_box(lat, u, x0, x1, y0, y1, 0.004)
    vent = _cover(dv, t)
    L.paint(vent, albedo=VENT_BLACK, rough=ROUGH["vent"], metal=0.0)
    slat = 0.5 + 0.5 * np.cos(2 * math.pi * u / VENT_SLAT_PITCH_M)
    L.height -= 0.0008 * vent * (1 - slat)
    for xd in VENT_DIVIDERS:
        L.paint(_cover(np.abs(lat - xd) - 0.0015, t) * vent, albedo=(0.07, 0.07, 0.075), metal=0.3)
    _groove(L, dv, SEAM_WIDTH_M, SEAM_DEPTH_M)


def paint_top(L, cockpit):
    L.fill(SILVER, ROUGH["silver"], METAL["silver"])
    _brushed(L, np.ones_like(L.A), 51)


def paint_under(L, cockpit):
    lat, _, o = L.local()
    L.fill(UNDER_GREY, ROUGH["panel"], 0.2)
    _panel_noise(L, np.ones_like(lat), 61)
    for so in UNDER_SEAMS_O:
        _groove(L, o - so, SEAM_WIDTH_M * 1.5, SEAM_DEPTH_M, albedo=(0.2, 0.2, 0.21))


def paint_end(L, cockpit, side):
    lat, u, o = L.local()
    t = L.texel
    L.fill(DARK_PANEL, ROUGH["panel"])
    silver = _cover(o - END_SILVER_O, t)
    L.paint(silver, albedo=SILVER, rough=ROUGH["silver"], metal=METAL["silver"])
    _brushed(L, silver, 71 if side > 0 else 72, along_a=False)
    _groove(L, o - END_SILVER_O, SEAM_WIDTH_M, SEAM_DEPTH_M)
    if side > 0:
        x, y, w, h = VELCRO_PATCH
        d = _sdf_box(o, u, x - w / 2, x + w / 2, y - h / 2, y + h / 2, 0.003)
        m = _cover(d, t)
        L.paint(m, albedo=VELCRO, rough=ROUGH["velcro"], metal=0.0)
        fuzz = _noise(m.shape, (1.0, 1.0), 73)
        L.albedo *= (1.0 + 0.15 * fuzz * m)[..., None]
        L.height += m * (0.0008 + 0.0002 * fuzz)


def paint_post(L, cockpit):
    ch = L.chart
    length = math.dist(ch["base"], ch["top"])
    s = L.B
    t = L.texel
    flare, sleeve = ch["foot_m"][3], ch["foot_m"][1]
    head = cockpit["console"]["post_head_length_m"]
    L.fill(POST_FOOT, ROUGH["foot"])
    rod = np.clip((s - (flare + sleeve)) / t + 0.5, 0, 1) * np.clip(((length - head) - s) / t + 0.5, 0, 1)
    L.paint(rod, albedo=SILVER, rough=ROUGH["silver"], metal=METAL["silver"])
    _brushed(L, rod, 81, along_a=False)


PAINTERS = {"front": paint_front, "tray_face": paint_tray_face, "tray_top": paint_tray_top, "tray_under": paint_tray_under, "back": paint_back,
            "top": paint_top, "under": paint_under, "end_l": lambda L, c: paint_end(L, c, -1), "end_r": lambda L, c: paint_end(L, c, 1),
            "post_0": paint_post, "post_1": paint_post}


def bake(cockpit):
    """(albedo (H, W, 3) linear, normal (H, W, 3) unit tangent-space, orm (H, W, 4)) float32 over the atlas."""
    charts = cl.console_charts(cockpit)
    W, H = cl.CONSOLE_ATLAS_SIZE
    albedo = np.zeros((H, W, 3), np.float32)
    normal = np.zeros((H, W, 3), np.float32)
    normal[..., 2] = 1.0
    orm = np.zeros((H, W, 4), np.float32)
    orm[..., 0] = 1.0
    orm[..., 1] = ROUGH["panel"]
    g = cl.CHART_GUTTER_PX // 2
    for name, chart in charts.items():
        L = Layer(chart, g)
        PAINTERS[name](L, cockpit)
        dh_db, dh_da = np.gradient(L.height, L.texel)
        dh_db = -dh_db  # rows run down the image, b up
        n = np.stack([-dh_da, -dh_db, np.ones_like(dh_da)], axis=-1)
        n /= np.linalg.norm(n, axis=-1, keepdims=True)
        cav = np.clip(1.0 + L.height / 0.002, 0.6, 1.0)  # recesses get a little occlusion
        ys, xs = slice(max(0, L.y0), min(H, L.y0 + L.h)), slice(max(0, L.x0), min(W, L.x0 + L.w))
        ly, lx = slice(ys.start - L.y0, ys.stop - L.y0), slice(xs.start - L.x0, xs.stop - L.x0)
        albedo[ys, xs] = np.clip(L.albedo[ly, lx], 0.0, 1.0)
        normal[ys, xs] = n[ly, lx]
        orm[ys, xs, 0] = cav[ly, lx]
        orm[ys, xs, 1] = np.clip(L.rough[ly, lx], 0.04, 1.0)
        orm[ys, xs, 2] = np.clip(L.metal[ly, lx], 0.0, 1.0)
        orm[ys, xs, 3] = np.clip(L.emit[ly, lx], 0.0, 1.0)
    return albedo, normal, orm


def encode_srgb(lin):
    c = np.clip(lin, 0.0, 1.0)
    s = np.where(c <= 0.0031308, 12.92 * c, 1.055 * np.power(c, 1.0 / 2.4) - 0.055)
    out = np.full(lin.shape[:2] + (4,), 255, np.uint8)
    out[..., :3] = np.round(s * 255.0)
    return out


def encode_linear(x):
    out = np.full(x.shape[:2] + (4,), 255, np.uint8)
    out[..., :x.shape[2]] = np.round(np.clip(x, 0.0, 1.0) * 255.0)
    return out


def write(out_dir, cfg) -> dict:
    """Bakes the three console maps into out_dir (cfg = the cockpit table); returns {"albedo": file, "normal": file, "orm": file}."""
    out = Path(out_dir)
    albedo, normal, orm = bake(cfg)
    files = cl.CONSOLE_MAPS
    (out / files["albedo"]).write_bytes(png_bytes(encode_srgb(albedo)))
    (out / files["normal"]).write_bytes(png_bytes(encode_linear(0.5 + 0.5 * normal)))
    (out / files["orm"]).write_bytes(png_bytes(encode_linear(orm)))
    return dict(files)
