"""Look reference scene suite (look fixes T13, unified appearance model): the definition of done for every change to
the eye adaptation, local adaptation, glare, white balance or display mapping. Renders each reference scene with
game/tests/render_check.gd (cockpit: cockpit_shots.gd), measures named regions of the PNG and gates them against
targets with their sources. Every later appearance change must pass all scenes (docs/subsystems/eye-adaptation.md).

Usage (needs a GPU, non-headless Godot; ~1-3 min per scene):
  python -I tools/look_reference_suite.py [--label B] [--only a,b] [--extra "--local_adaptation=off"] [--measure_only]
      [--compare A]   (writes an A|B sheet per scene)
Output: screenshots/look_fixes/t13/suite/<label>_<scene>.png / .log, <label>_results.json; exit code 1 on a failed gate.
Every scene with the Earth or its sky in view runs with --weather=analog (the game's weather is on by default;
render_check's default is off); the EPIC scene uses the ERA5 snapshot of its hour.
Reference data (git-ignored, local): the DSCOVR EPIC image screenshots/look_investigation/refs/epic_1b_20250320143251.png
and the ERA5 weather of that hour screenshots/look_fixes/t7/capC/era5_1097630.wthr; scenes whose data is missing are
skipped (reported).
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GODOT = os.path.join(ROOT, "external", "godot", "bin", "godot.windows.editor.double.x86_64.console.exe")
OUT = os.path.join(ROOT, "screenshots", "look_fixes", "t13", "suite")
EPIC_REF = os.path.join(ROOT, "screenshots", "look_investigation", "refs", "epic_1b_20250320143251.png")
EPIC_WTHR = os.path.join(ROOT, "screenshots", "look_fixes", "t7", "capC", "era5_1097630.wthr")
# Weather as in the game (on by default): every scene with the Earth or its sky in view gets the shipped analog weather
# of its --utc (render_check defaults to --weather=off; user 2026-11-08: the lunar-night Earth had no clouds).
WEATHER = "--weather=analog"
PAD = ["--lat=25.9853", "--lon=-97.1873", "--hide_ui", WEATHER]
NOON = ["--utc=2026-11-07T18:15:00"] + PAD
MOON = ["--body=moon", "--lat=0.674", "--lon=23.473", "--utc=2026-11-17T08:00:00", "--hide_ui", WEATHER]
LUNAR_NIGHT = ["--body=moon", "--lat=0", "--hide_ui", WEATHER]


def lin(c):
    c = np.asarray(c, float) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def luma(a):
    return a @ np.array([0.2126, 0.7152, 0.0722])


def rows(a, top, bottom):
    h = a.shape[0]
    return a[int(top * h):int(bottom * h)].reshape(-1, 3)


def blob(a):
    """The brightest compact object (Earth / Moon disc): pixels above half of the 99.9th percentile luma, inside the
    bounding box around the brightest pixel (radius from the mask's extent)."""
    y = luma(a)
    threshold = max(40.0, 0.5 * np.percentile(y, 99.9))
    mask = y > threshold
    if mask.sum() < 4:
        return a[mask].reshape(-1, 3)
    iy, ix = np.unravel_index(np.argmax(np.where(mask, y, -1)), y.shape)
    ys, xs = np.nonzero(mask)
    near = (np.abs(ys - iy) < 200) & (np.abs(xs - ix) < 200)
    return a[ys[near], xs[near]]


def br(pixels):
    l = lin(pixels)
    return float(l[:, 2].mean() / max(l[:, 0].mean(), 1e-9))


def sky_br(pixels):
    """B/R of the clear sky between clouds: the 75th percentile of the per-pixel linear B/R (weather is on; cumulus is
    near neutral and would pull a region mean toward 1)."""
    l = lin(pixels)
    return float(np.percentile(l[:, 2] / np.maximum(l[:, 0], 1e-6), 75))


def clipped(pixels):
    return float((pixels.max(axis=1) >= 254).mean())


def epic_stats(path):
    """DSCOVR-EPIC disc classes (screenshots/look_fixes/t7/scripts/disc_stats.py): linear Y over the disc's p99."""
    im = Image.open(path).convert("RGB")
    a = np.asarray(im).astype(float).mean(axis=2)
    m = a > 6
    ys, xs = np.nonzero(m.sum(axis=1) > 40)[0], np.nonzero(m.sum(axis=0) > 40)[0]
    cy, cx = (ys.min() + ys.max()) / 2, (xs.min() + xs.max()) / 2
    r = max(ys.max() - ys.min(), xs.max() - xs.min()) / 2
    s = 512
    im = np.asarray(im.crop((int(cx - r), int(cy - r), int(cx + r), int(cy + r))).resize((s, s), Image.LANCZOS)).astype(float)
    yy, xx = np.mgrid[0:s, 0:s]
    disc = ((xx - s / 2 + 0.5) ** 2 + (yy - s / 2 + 0.5) ** 2) < (0.95 * s / 2) ** 2
    y = luma(lin(im))[disc]
    y = y / np.percentile(y, 99)
    return {"dark": float((y < 0.15).mean()), "grey": float(((y >= 0.15) & (y < 0.5)).mean()),
            "white": float((y >= 0.5).mean()), "_y": y}


# Each scene: render arguments and a measure function returning {metric: value}; gates: metric -> (low, high, source).
def m_ground_sky(a):
    ground, sky = rows(a, 0.80, 0.98), rows(a, 0.05, 0.30)
    return {"ground_luma": float(luma(ground).mean()), "sky_b_over_r": sky_br(sky), "clipped": clipped(a.reshape(-1, 3)),
            "frame_luma": float(luma(a.reshape(-1, 3)).mean())}


def m_sky(a):
    sky = rows(a, 0.05, 0.40)
    return {"sky_b_over_r": sky_br(sky), "ground_luma": float(luma(rows(a, 0.80, 0.98)).mean()),
            "frame_luma": float(luma(a.reshape(-1, 3)).mean())}


def m_moon_day(a):
    ground = rows(a, 0.65, 0.95)
    return {"ground_luma": float(luma(ground).mean()), "clipped": clipped(ground), "ground_b_over_r": br(ground)}


def halo_bump(a):
    """Rings / bands around the brightest object: the radial mean luma profile (1 px annuli) beyond the disc's limb
    (+3 px) must fall monotonically; returns the largest rise above the running minimum (8-bit luma)."""
    y = luma(a)
    threshold = max(40.0, 0.5 * np.percentile(y, 99.9))
    ys, xs = np.nonzero(y > threshold)
    if len(ys) == 0:
        return 0.0
    iy, ix = np.unravel_index(np.argmax(y), y.shape)
    near = (np.abs(ys - iy) < 200) & (np.abs(xs - ix) < 200)
    cy, cx = ys[near].mean(), xs[near].mean()
    r_disc = np.sqrt(near.sum() / np.pi)
    yy, xx = np.mgrid[0:y.shape[0], 0:y.shape[1]]
    r = np.sqrt((yy - cy) ** 2 + (xx - cx) ** 2).astype(int)
    r_max = int(min(cy, cx, y.shape[0] - cy, y.shape[1] - cx)) - 1
    prof = np.bincount(r.ravel(), y.ravel())[:r_max] / np.maximum(np.bincount(r.ravel())[:r_max], 1)
    start = int(r_disc + 3)
    bump, low = 0.0, float("inf")
    for v in prof[start:]:
        low = min(low, v)
        bump = max(bump, v - low)
    inner = y[r < 0.6 * r_disc].mean() if (r < 0.6 * r_disc).any() else 1.0
    rim = y[(r >= 0.85 * r_disc) & (r <= 1.15 * r_disc)].mean()
    return float(bump), float(rim / max(inner, 1e-6))


def m_disc(a):
    d = blob(a)
    y = luma(d)
    return {"disc_px": int(len(d)), "disc_clipped": clipped(d), "disc_luma_sd": float(y.std()),
            "disc_max_channel": float(d.max(axis=1).mean()), "disc_b_over_r": br(d), "halo_bump": halo_bump(a)[0],
            "limb_ratio": halo_bump(a)[1]}


def halo_profile(a, fov_deg, radius_deg):
    """The halo around the brightest compact object (look fixes T13 fix 1): mean linear display luminance (sRGB decoded,
    BT.709 luma) of rings at 1 / 2 / 5 / 10 deg from the disc's centre (+-10 %, render_check's vertical fov, rectilinear
    centre scale) over the disc's own mean (inside 60 % of its angular radius radius_deg). The centre: the centroid of the
    pixels above half the brightest near the frame centre (the scenes aim at the object)."""
    y = luma(lin(a))
    h, w = y.shape
    px_per_deg = (h / 2) / np.tan(np.radians(fov_deg / 2)) * np.pi / 180
    y8 = luma(a)
    yy, xx = np.mgrid[0:h, 0:w]
    # the object is aimed at the frame centre (--look_at_*): the centroid of the bright pixels within 1.5 radii of it
    near = np.hypot(yy - h / 2, xx - w / 2) < max(1.5 * radius_deg * px_per_deg, 4.0)
    bright = near & (y8 > 0.5 * y8[near].max())
    cy, cx = yy[bright].mean(), xx[bright].mean()
    r = np.sqrt((yy - cy) ** 2 + (xx - cx) ** 2)
    r_max = int(min(cy, cx, h - cy, w - cx))
    r_disc = radius_deg * px_per_deg
    disc = float(y[r < 0.6 * r_disc].mean()) if (r < 0.6 * r_disc).any() else float("nan")
    out = {"disc_lin": disc}
    for d in (1, 2, 5, 10):
        ring = (r > 0.9 * d * px_per_deg) & (r < 1.1 * d * px_per_deg)
        out[f"halo_{d}deg_rel"] = float(y[ring].mean() / max(disc, 1e-9)) if ring.any() and d * px_per_deg * 1.1 < r_max \
            else float("nan")
    # rings: the 8-bit radial luma profile (1 px annuli) from 1.15 disc radii outward must fall; the largest rise above
    # its running minimum (a dark ring at the limb followed by a brighter ring shows up here)
    y8r = np.bincount(r.astype(int).ravel(), y8.ravel())[:r_max] / np.maximum(np.bincount(r.astype(int).ravel())[:r_max], 1)
    rise, low = 0.0, float("inf")
    for v in y8r[int(1.15 * r_disc) + 1:]:
        low = min(low, v)
        rise = max(rise, v - low)
    out["halo_rise"] = float(rise)
    return out


def with_halo(measure, fov_deg, radius_deg):
    def m(a):
        out = measure(a)
        out.update(halo_profile(a, fov_deg, radius_deg))
        return out
    return m


def m_epic(a, path):
    if not os.path.exists(EPIC_REF):
        return {"skipped": 1.0}
    ref, own = epic_stats(EPIC_REF), epic_stats(path)
    corr = float(np.corrcoef(ref["_y"], own["_y"])[0, 1]) if len(ref["_y"]) == len(own["_y"]) else float("nan")
    return {"dark": own["dark"], "grey": own["grey"], "white": own["white"], "corr": corr}


def m_sun(a):
    return {"frame_luma": float(luma(a.reshape(-1, 3)).mean()), "white_px": int((a.min(axis=2) >= 250).sum()),
            "ground_luma": float(luma(rows(a, 0.80, 0.98)).mean())}


def m_terminator(a):
    """Day/night boundary (measured, no gate: T9b changes the direct beam's wavelengths, user-approved 2026-11-08): the
    lit Earth pixels (luma above 50 % of the 95th percentile of the bright object) are the day side; the band pixels
    between 5 % and 40 % of it the terminator. Its luminance relative to the day side, its B/R (linear) against the day's,
    its share of the object and its warm share (R > B). Physically a narrow (~500 km for Sun < 5 deg) dark orange band
    fading into night, not a bright red rim."""
    y = luma(lin(a))
    lit = y > 1e-4
    ref = np.percentile(y[lit], 95) if lit.any() else 1.0
    day = y > 0.5 * ref
    band = (y > 0.05 * ref) & (y < 0.4 * ref)
    l = lin(a)
    def ratio(m):
        return float(l[m][:, 2].mean() / max(l[m][:, 0].mean(), 1e-9)) if m.any() else float("nan")
    return {"band_rel_luminance": float(y[band].mean() / max(y[day].mean(), 1e-9)) if band.any() else float("nan"),
            "band_peak_rel": float(np.percentile(y[band], 95) / max(y[day].mean(), 1e-9)) if band.any() else float("nan"),
            "band_b_over_r": ratio(band), "day_b_over_r": ratio(day),
            "band_share": float(band.sum() / max((band | day).sum(), 1)),
            "band_warm_share": float((l[band][:, 0] > l[band][:, 2]).mean()) if band.any() else float("nan")}


def m_sun_profile(fov_deg):
    """The Sun's veil: radial mean luma (8-bit display) at 1 / 2 / 3 / 5 / 10 deg from the disc's centre (render_check's
    vertical fov), the disc white, the halo's monotonic fall (halo_bump). In space the veil is the eye's alone (CIE 146:
    L_veil = E (10 / theta^3 + 5 / theta^2 + ...), no aureole); from the ground the atmosphere's aureole adds to it."""
    def measure(a):
        y = luma(a)
        h = a.shape[0]
        px_per_deg = (h / 2) / np.tan(np.radians(fov_deg / 2)) * np.pi / 180
        white = a.min(axis=2) >= 250
        if white.any():
            ys, xs = np.nonzero(white)
            cy, cx = ys.mean(), xs.mean()
        else:
            cy, cx = np.unravel_index(np.argmax(y), y.shape)
        yy, xx = np.mgrid[0:h, 0:a.shape[1]]
        r_deg = np.sqrt((yy - cy) ** 2 + (xx - cx) ** 2) / px_per_deg
        out = {"white_px": int(white.sum()), "halo_bump": halo_bump(a)[0]}
        for d in (1, 2, 3, 5, 10):
            ring = (r_deg > d * 0.9) & (r_deg < d * 1.1)
            out[f"luma_{d}deg"] = float(y[ring].mean()) if ring.any() else float("nan")
        return out
    return measure


def sun_measure(fov_deg):
    """m_sun + the halo profile (luma at 1 / 2 / 3 / 5 / 10 deg, the rings outside the frame are nan)."""
    profile = m_sun_profile(fov_deg)

    def measure(a):
        out = m_sun(a)
        out.update(profile(a))
        return out
    return measure


def _cloud_br(a, x0, y0, x1, y1):
    """Linear B/R of the cloud pixels of a region (image fractions): per-pixel B/R < 1.2 (not the blue sky / sea),
    unclipped, luma above the cloud pixels' median (the lit cloud, not its shaded base)."""
    h, w, _ = a.shape
    r = a[int(y0 * h):int(y1 * h), int(x0 * w):int(x1 * w)].reshape(-1, 3)
    l = lin(r)
    y = luma(l)
    m = (l[:, 2] / np.maximum(l[:, 0], 1e-6) < 1.2) & (r.max(axis=1) < 250)
    if m.sum() < 50:
        return float("nan")
    m &= y > np.median(y[m])
    return float(l[m, 2].mean() / max(l[m, 0].mean(), 1e-9))


def m_horizon_clouds(a):
    """Bug hunt 2026-11-08 (red horizon clouds): the far cloud band at the horizon (from 2 km, el 2, fov 60: rows
    0.50-0.58 between the near cumulus) against the near cumulus (upper left), displayed linear B/R."""
    near, far = _cloud_br(a, 0.00, 0.33, 0.30, 0.50), _cloud_br(a, 0.28, 0.50, 0.75, 0.58)
    return {"horizon_cloud_b_over_r": far, "near_cloud_b_over_r": near, "horizon_over_near": far / near}


# Earth's mean radius for the horizon dip of the altitude sweep [S: IUGG mean radius R1 6371.0088 km].
EARTH_MEAN_RADIUS_M = 6371008.8
# The sweep's view: elevation 2 deg, vertical fov 60 deg (Godot keeps the height).
SWEEP_ELEVATION_DEG, SWEEP_FOV_DEG = 2.0, 60.0


def m_horizon_band(height_m):
    """User 2026-11-08 (pink horizon clouds, worse below 2 km?): the far cloud band at the visible horizon from an eye at
    height_m - the rows from 1 deg below to 6 deg above the horizon dip (acos(R / (R + h))): 1.5 km tops 15-90 km
    away from the ground; whole width, the cloud pixels' linear B/R (_cloud_br) - and the near cloud above it (8-30 deg
    above the horizon)."""
    dip = np.degrees(np.arccos(EARTH_MEAN_RADIUS_M / (EARTH_MEAN_RADIUS_M + height_m)))
    t = np.tan(np.radians(SWEEP_FOV_DEG / 2.0))

    def row(elev_deg):  # image fraction of an elevation (0 top)
        return 0.5 - np.tan(np.radians(elev_deg - SWEEP_ELEVATION_DEG)) / t * 0.5

    def measure(a):
        far = _cloud_br(a, 0.0, row(-dip + 6.0), 1.0, row(-dip - 1.0))
        near = _cloud_br(a, 0.0, row(-dip + 30.0), 1.0, row(-dip + 8.0))
        return {"horizon_cloud_b_over_r": far, "near_cloud_b_over_r": near, "horizon_over_near": far / near}
    return measure


def m_cockpit(a):
    return {"frame_luma": float(luma(a.reshape(-1, 3)).mean()), "clipped": clipped(a.reshape(-1, 3))}


SCENES = {
    "pad_noon_e5": (NOON + ["--azimuth=90", "--elevation=5", "--fov=100"], m_ground_sky, {
        "clipped": (0.0, 0.02, "[call] no washed-out sky: < 2 % of the frame at 254+"),
        "sky_b_over_r": (1.6, 99, "[S: clear noon sky is blue, CIE 015 daylight sky chromaticity; T5 noon sky 10.3 near zenith]"),
        "ground_luma": (60, 170, "[D: NAIP-albedo ground ~0.05-0.1 shown below the field; call: not black, not white]")}),
    "pad_noon_e60": (NOON + ["--azimuth=90", "--elevation=60", "--fov=100"], m_sky, {
        "sky_b_over_r": (1.6, 99, "[S: clear sky blue looking up]"),
        "frame_luma": (70, 200, "[call] the sky looking up is neither grey-dark nor white")}),
    # bug hunt 2026-11-08 (user: noon clouds near the horizon red-shifted, esp. from ~2000 m): .superpowers/sdd/
    # 2026-11-07-look-fixes/bug-horizon-report.md. Expected value: the core's own composite of a WHITE Lambertian cloud
    # lit by Sun + sky 30 km away from a 2 km eye is 0.845 of the plate white's B/R (60 km 1.0), through the white
    # balance and the Hunt step (S 1.50) 0.775 displayed; the near cumulus' sunlit sides show 0.85.
    "horizon_clouds_noon_2km": (NOON + ["--height=2000", "--azimuth=90", "--elevation=2", "--fov=60"], m_horizon_clouds, {
        "horizon_cloud_b_over_r": (0.75, 99, "[D: core composite of a white cloud 30 km from 2 km (0.845 of the plate "
                                             "white) shown at S 1.5 -> 0.775; call: real distant noon cumulus white to "
                                             "slightly yellowish, not salmon; 2026-11-08 0.64]"),
        "horizon_over_near": (0.84, 99, "[D: the same white cloud 0.775 over the near cumulus 0.85-0.90: the haze "
                                        "warms the far band by at most ~10-15 %; 2026-11-08 0.755; T9b 0.843, gate 0.84 by user 2026-11-09]")}),
    "dusk_m5_5": (["--utc=2026-11-08T00:05:00"] + PAD + ["--azimuth=250", "--elevation=5", "--fov=100"], m_sky, {
        "sky_b_over_r": (1.2, 99, "[T5b fix 3: dusk B/R 1.36; user: twilight bluish]")}),
    "blue_hour_m7_7": (["--utc=2026-11-08T00:15:00"] + PAD + ["--azimuth=250", "--elevation=5", "--fov=100"], m_sky, {
        "sky_b_over_r": (1.6, 99, "[user 2026-11-08: the blue hour stays blue; T5b fix 3 B/R 1.91]")}),
    "lunar_day_apollo11": (MOON + ["--azimuth_from_sun=180", "--elevation=5", "--fov=100"], m_moon_day, {
        "ground_luma": (115, 175, "[user 2026-11-08: the Moon day stays bright (T6 old 128 accepted, 108 too dark); "
                                  "S: AS11-40-5903 regolith 117-147 8-bit luma at metered exposure]"),
        "clipped": (0.0, 0.01, "[call] regolith not clipped"),
        "ground_b_over_r": (0.75, 0.95, "[user 2026-11-08: the close-up Moon grey, not cream; S: Apollo crews - regolith "
                                        "grey, brownish-grey at low Sun; model: adopted white 0.6 toward the regolith field "
                                        "(Atmosphere.moon_field_tint): measured 0.922; before 0.638]")}),
    "lunar_night_full_earth": (LUNAR_NIGHT + ["--lon=45", "--utc=2026-11-07T14:00:00", "--azimuth=270",
                                              "--elevation=20", "--fov=100"], m_disc, {
        "disc_clipped": (0.0, 0.25, "[user 2026-11-08: the Earth from the lunar night shows detail, not overexposed]"),
        "disc_luma_sd": (10, 999, "[T10: ocean / land / cloud detail, luma sd 18]")}),
    "lunar_night_half_earth": (LUNAR_NIGHT + ["--lon=45", "--utc=2026-12-01T14:00:00", "--azimuth=270",
                                              "--elevation=20", "--fov=100"], m_disc, {
        "disc_clipped": (0.0, 0.25, "[user 2026-11-08: detail visible]"),
        "disc_luma_sd": (10, 999, "[T10 half Earth luma sd 27]")}),
    "lunar_night_crescent_earth": (LUNAR_NIGHT + ["--lon=80", "--utc=2026-11-26T14:00:00", "--azimuth=270",
                                                  "--elevation=10", "--fov=100"], m_disc, {
        "disc_max_channel": (0, 245, "[user 2026-11-08: the crescent not white]")}),
    "full_moon_from_earth": (["--utc=2026-11-24T02:00:00"] + PAD + ["--look_at_moon", "--fov=12"], m_disc, {
        "disc_clipped": (0.0, 0.25, "[user 2026-11-08: maria visible]"),
        "disc_luma_sd": (8, 999, "[maria / highlands contrast: geometric albedo 0.07 vs 0.12-0.16]"),
        "disc_b_over_r": (0.70, 99, "[user 2026-11-08: not cream - T10 (233,211,189) = linear B/R 0.63; "
                                    "call: the disc (maria + highlands + limb veil) at least 0.70, a warm off-white; the "
                                    "regolith is reddish, B/R ~0.8 under sunlight, Lane & Irvine 1973]")}),
    # the first-quarter Moon in the afternoon sky over the pad (~45 deg up in the south-east; bug hunt 2026-11-08)
    "day_moon_from_earth": (["--utc=2026-11-17T22:00:00"] + PAD + ["--look_at_moon", "--fov=12"], m_disc, {}),
    "earth_from_space_epic": (["--utc=2025-03-20T14:28:03", "--focus=earth",
                               "--view_from=gcrf:1548878.149868,49983.227147,173343.826562", "--distance=4e8",
                               "--fov=2.4", "--hide_ui", "--weather=" + EPIC_WTHR], "epic", {
        "white": (0.0, 0.30, "[S: DSCOVR EPIC 2025-03-20 14:28 white share 0.128 (linear Y >= 0.5 p99); T7 linear "
                             "camera 0.213; display before T13 0.444]"),
        "dark": (0.15, 0.6, "[S: EPIC dark share 0.375; T7 linear camera 0.338]"),
        "corr": (0.40, 1.0, "[S: EPIC Y correlation; T7 display 0.49]")}),
    # the Sun in view needs a clear line to it: the analog weather puts the pad in a cumulus' shadow at this hour
    "sun_in_view": ([a if a != WEATHER else "--weather=off" for a in NOON] + ["--look_at_sun", "--fov=100"], m_sun, {
        "white_px": (1, 10 ** 9, "[call] the Sun's disc is display white"),
        "frame_luma": (20, 230, "[call] the frame is not black / not blown out around the Sun")}),
    "cockpit_day": ("cockpit", m_cockpit, {
        "clipped": (0.0, 0.03, "[T10: windows show the sky, 0.08 % saturated]"),
        "frame_luma": (90, 210, "[D: crew photos crew1_1 / discovery: white cabin ~0.85-0.9 sRGB walls, dark console]")}),
}
# Small bright objects also at the headset's angular resolution and at a normal desktop field of view (user 2026-11-08):
# the local adaptation and glare kernels are in degrees, so the gates must hold at all three. render_check's 1600 x 900
# frame, vertical fov: 22.2 deg = 40 px/deg at the centre [E: Pimax Dream Air 5294 px per eye over ~100-130 deg],
# 60 deg = 13.6 px/deg, 10 deg (central vision: the object as attended, user 2026-11-08) = 90 px/deg; the base scenes' 100 deg = 6.6 px/deg (the Earth from the Moon, 1.9 deg, ~12 px).
# The halo profile (look fixes T13 fix 1, user 2026-11-08: "a real clear full-Moon night shows at most a slight glow, no
# big ball"): linear display luminance of rings at 1 / 2 / 5 / 10 deg over the disc's own (halo_profile), all fovs. Each
# source's eye veil is shown at its source's local factor, so the halo keeps its physical ratio to the disc: CIE 146:2002
# (young eye) veil / disc = f(theta) x Omega_disc - the full Moon 9.7e-4 / 1.6e-4 / 1.9e-5 / 4.3e-6, the full Earth from
# the Moon (0.95 deg) 1.3e-2 (in its limb's antialiasing, not gated) / 2.2e-3 / 2.6e-4 / 5.8e-5. From the Earth the night
# sky and the atmosphere's lunar aureole add a floor (sky shader, KS91-type, measured 0.004-0.02 of the disc).
# T13 values (A): Moon 1 deg 0.10-0.47, 2 deg 0.02-0.11; Earth 2 deg 0.17-0.50.
HALO_GATES = {
    "full_moon_from_earth": {
        "halo_1deg_rel": (0.0, 0.06, "[user 2026-11-08: a slight glow; CIE 146 veil 9.7e-4 of the disc + sky / aureole floor; user 2026-11-09 re-baseline to the Mie-fitted aerosol phase (T9b): 0.048, forward peak ~1.3x the old Cornette-Shanks]"),
        "halo_2deg_rel": (0.0, 0.03, "[user 2026-11-08; CIE 146 veil 1.6e-4 of the disc + sky / aureole floor]"),
        "halo_rise": (0.0, 3.0, "[orchestrator 2026-11-08: no dark ring at the limb, no bright ring beyond: the 8-bit "
                                "radial profile from 1.15 disc radii falls within 3 luma]")},
    "lunar_night_full_earth": {
        "halo_2deg_rel": (0.001, 0.006, "[S: CIE 146:2002 veil / disc 2.2e-3 (x0.5 .. 2.7: the disc's mean vs its "
                                        "brightest parts, the kernel fit); no sky on the Moon]"),
        "halo_5deg_rel": (0.0, 0.001, "[S: CIE 146 veil / disc 2.6e-4, x4]"),
        "halo_10deg_rel": (0.0, 0.0003, "[S: CIE 146 veil / disc 5.8e-5, x5]"),
        "halo_rise": (0.0, 3.0, "[orchestrator 2026-11-08: no rings at the limb]")},
}
# [S: mean distances] the Moon from the Earth 1737.4 km / 384400 km, the Earth from the Moon 6371 km / 384400 km.
ANGULAR_RADIUS_DEG = {"--look_at_moon": 0.259, "--look_at_earth": 0.950, "--look_at_sun": 0.267}
FOV_VARIANTS = {"vr": "22.2", "d60": "60", "f10": "10"}


def _variants():
    out = {}
    for name, (args, measure, gates) in SCENES.items():
        aim = None
        if name.startswith("lunar_night_"):
            aim = "--look_at_earth"
        elif name in ("full_moon_from_earth", "sun_in_view"):
            aim = "--look_at_moon" if name == "full_moon_from_earth" else "--look_at_sun"
        if aim is None:
            continue
        base = [a for a in args if not a.startswith(("--azimuth", "--elevation", "--fov"))]
        if aim not in base:
            base.append(aim)
        for tag, fov in FOV_VARIANTS.items():
            g = dict(gates)
            if name == "sun_in_view":
                g = {"white_px": gates["white_px"],
                     "luma_1deg": (0.0, 254.0, "[user 2026-11-08 'Sonne wie Mond und Erde'; user 2026-11-09 re-baseline after the Mie-fitted aerosol phase (T9b, 252): the eye's veil is shown at the "
                                               "disc's local factor, the halo is the sky's aureole, not a clipped white ball]")}
            else:
                g["halo_bump"] = (0.0, 3.0, "[orchestrator / user 2026-11-08: a smooth CIE 146 veil, no ring, no bands: "
                                            "radial profile beyond the limb falls monotonically within 3 luma]")
                if not name.startswith("lunar_night_crescent"):
                    g["limb_ratio"] = (0.0, 1.2, "[T10 test: limb no brighter than the interior x1.25; no bright ring]")
                g.update(HALO_GATES.get(name, {}))
                if tag == "f10" and "halo_2deg_rel" in g:
                    # at 10 deg the eye adapts to the Earth's disc (gain ~1.2, T13): nothing above display white, no veil
                    lo, hi, src = g["halo_2deg_rel"]
                    g["halo_2deg_rel"] = (0.0, hi, src + " [f10: adapted to the disc, no source]")
                if tag == "f10":  # 5 and 10 deg lie outside the 10 deg frame
                    g.pop("halo_5deg_rel", None)
                    g.pop("halo_10deg_rel", None)
            if name == "sun_in_view":
                measure = sun_measure(float(fov))
            out[f"{name}_{tag}"] = (base + ["--fov=" + fov], measure if name == "sun_in_view" else
                                    with_halo(measure, float(fov), ANGULAR_RADIUS_DEG[aim]), g)
    return out


SCENES.update(_variants())
# The terminator (orchestrator 2026-11-08, measured values only, no gates until T9b): the half Earth from the lunar night
# at 10 deg, and the evening terminator from 400 km (lat 0, lon 0 at 18:15 UTC: the Sun ~4 deg up behind the camera's
# west, the terminator ~500 km east; view east, 30 deg down).
SCENES["terminator_half_earth_f10"] = (LUNAR_NIGHT + ["--lon=45", "--utc=2026-12-01T14:00:00", "--look_at_earth",
                                                       "--fov=10"], m_terminator, {})
# The Sun in space (orchestrator 2026-11-08): a white disc, black sky to the limb, only the eye's CIE 146 veil (no
# aureole, no Godot glow); from 400 km above the pad at noon and from 100 km above the Moon; 60 and 10 deg. Contrast:
# sun_in_view* (ground, the atmosphere's aureole + the same veil).
for _fov in ("60", "10"):
    _sun_gates = {"white_px": (1, 10 ** 9, "[call] the Sun's disc is display white"),
                  "halo_bump": (0.0, 3.0, "[CIE 146: the veil falls monotonically with the angle]"),
                  # [S: CIE 146:2002 veil x the Sun's solid angle 6.8e-5 sr = 1.0e-3 / 1.7e-4 of the disc at 1 / 2 deg, linear;
                  # the sRGB display 12.92 x v x 255 = 3.3 / 0.6 luma; the veil is shown at the disc's local factor (user
                  # 2026-11-08 'Sonne wie Mond und Erde'); x0.5 .. 2 for the kernel fit and the disc's mean]
                  "luma_1deg": (1.6, 6.6, "[S: CIE 146 x Omega_Sun, sRGB 3.3 luma]"),
                  "luma_2deg": (0.2, 1.3, "[S: CIE 146 x Omega_Sun, sRGB 0.6 luma]"),
                  "luma_5deg": (0.0, 0.5, "[S: CIE 146 x Omega_Sun, <= 0.1 luma]"),
                  "luma_10deg": (0.0, 0.5, "[S: CIE 146 x Omega_Sun, <= 0.03 luma]")}
    SCENES[f"sun_space_400km_f{_fov}"] = (["--lat=25.9853", "--lon=-97.1873", "--height=400000",
                                           "--utc=2026-11-07T18:15:00", "--hide_ui", WEATHER, "--look_at_sun",
                                           f"--fov={_fov}"], m_sun_profile(float(_fov)), _sun_gates)
    SCENES[f"sun_lunar_orbit_f{_fov}"] = (["--body=moon", "--lat=0.674", "--lon=23.473", "--height=100000",
                                           "--utc=2026-11-17T08:00:00", "--hide_ui", WEATHER, "--look_at_sun",
                                           f"--fov={_fov}"], m_sun_profile(float(_fov)), _sun_gates)
# The altitude sweep of the horizon clouds (user 2026-11-08: the pink band at the horizon from below 2 km too): measured
# only (no gate yet - the 2 km scene's gates stand for it), analog weather at noon and the user's GFS snapshot if cached.
GFS_USER_WTHR = os.path.join(ROOT, "data", "weather", "cache", "gfs_1111980_f007.wthr")
# The primary gate of the horizon clouds (user 2026-11-08: the low-altitude renders match the game, the pink band clearly
# visible): 300 m, the sweep's worst pink (far band B/R 0.594 before the cloud sky-light fix; 100-600 m all ~0.595).
SCENES["horizon_clouds_noon_300m"] = (NOON + ["--height=300", "--azimuth=90", f"--elevation={SWEEP_ELEVATION_DEG:g}",
                                              f"--fov={SWEEP_FOV_DEG:g}"], m_horizon_band(300.0), {
    "horizon_cloud_b_over_r": (0.75, 99, "[D: as horizon_clouds_noon_2km - a white cloud 30 km away shown at S 1.5 "
                                         "-> 0.775; call: distant noon cumulus white to slightly yellowish, not salmon; "
                                         "2026-11-08 0.594]"),
    "horizon_over_near": (0.85, 99, "[D: as horizon_clouds_noon_2km: the haze warms the far band by at most ~10-15 %; "
                                    "2026-11-08 0.762; after the cloud sky-light fix 0.616 / 0.769]")})
for _h in (0, 100, 300, 600, 1000, 2000):
    _view = [f"--height={_h}", "--azimuth=90", f"--elevation={SWEEP_ELEVATION_DEG:g}", f"--fov={SWEEP_FOV_DEG:g}"]
    SCENES[f"horizon_sweep_noon_{_h}m"] = (NOON + _view, m_horizon_band(float(_h)), {})
    if os.path.exists(GFS_USER_WTHR):
        SCENES[f"horizon_sweep_gfs_{_h}m"] = (["--utc=2026-11-08T19:30:00", "--lat=25.9853", "--lon=-97.1873",
                                               "--hide_ui", "--weather=" + GFS_USER_WTHR] + _view,
                                              m_horizon_band(float(_h)), {})
SCENES["sun_in_view_profile_f10"] = ([a if a != WEATHER else "--weather=off" for a in NOON] + ["--look_at_sun", "--fov=10"],
                                     m_sun_profile(10.0), {})
SCENES["terminator_400km"] = (["--lat=0", "--lon=0", "--height=400000", "--utc=2026-11-07T18:15:00", "--hide_ui", WEATHER,
                               "--azimuth=90", "--elevation=-30", "--fov=60"], m_terminator, {})
CROP_SCENES = [n for n in SCENES if n.endswith(("_vr", "_f10"))]
GAPS = ["cockpit_night_lamps: cockpit_shots.gd has no --utc / night case (T10 note) - not automated yet"]


def run_watched(cmd, log, timeout_s=900):
    """Runs a Godot render; stops it early when the project does not compile (a parse error makes render_check loop)."""
    import time
    with open(log, "w") as f:
        proc = subprocess.Popen(cmd, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT)
        start = time.time()
        while proc.poll() is None:
            time.sleep(5)
            try:
                with open(log, errors="ignore") as r:
                    head = r.read(200000)
            except OSError:
                head = ""
            if "Parse Error" in head or time.time() - start > timeout_s:
                proc.kill()
                break


def render(name, args, label, extra):
    png = os.path.join(OUT, f"{label}_{name}.png")
    log = os.path.join(OUT, f"{label}_{name}.log")
    if args == "cockpit":
        out_dir = os.path.join(OUT, f"{label}_cockpit")
        cmd = [GODOT, "--path", "game", "--script", "res://tests/cockpit_shots.gd", "--",
               "--mission=falcon9_crew_dragon", "--t_plus=-60", "--out_dir=" + out_dir] + extra
        run_watched(cmd, log, 1800)
        src = os.path.join(out_dir, "pilot_eye.png")
        if os.path.exists(src):
            Image.open(src).save(png)
        return png
    cmd = [GODOT, "--path", "game", "--script", "res://tests/render_check.gd", "--"] + args + extra + \
        ["--adaptation_log", "--out=" + png]
    if os.path.exists(png):
        os.remove(png)
    run_watched(cmd, log)
    return png


def gain_of(log):
    try:
        for line in open(log, errors="ignore"):
            if "gain" in line and "adaptation" in line.lower():
                last = line.strip()
        return last[:200]
    except (OSError, UnboundLocalError):
        return ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="B")
    ap.add_argument("--only", default="")
    ap.add_argument("--extra", default="")
    ap.add_argument("--measure_only", action="store_true")
    ap.add_argument("--compare", default="")
    ap.add_argument("--out", default="", help="output folder (default screenshots/look_fixes/t13/suite)")
    o = ap.parse_args()
    global OUT
    if o.out:
        OUT = os.path.abspath(o.out)
    os.makedirs(OUT, exist_ok=True)
    names = [n for n in SCENES if not o.only or n in o.only.split(",")]
    results, failed = {}, []
    for name in names:
        args, measure, gates = SCENES[name]
        if name == "earth_from_space_epic" and not (os.path.exists(EPIC_REF) and os.path.exists(EPIC_WTHR)):
            results[name] = {"skipped": "reference data missing"}
            continue
        png = os.path.join(OUT, f"{o.label}_{name}.png")
        if not o.measure_only:
            png = render(name, args, o.label, o.extra.split())
        if not os.path.exists(png):
            results[name] = {"error": "no image"}
            failed.append(name)
            continue
        a = np.asarray(Image.open(png).convert("RGB")).astype(float)
        m = m_epic(a, png) if measure == "epic" else measure(a)
        checks = {}
        for k, (lo, hi, src) in gates.items():
            v = m.get(k, float("nan"))
            ok = bool(lo <= v <= hi)
            checks[k] = {"value": v, "low": lo, "high": hi, "ok": ok, "source": src}
            if not ok:
                failed.append(f"{name}.{k}")
        results[name] = {"metrics": m, "gates": checks, "log": gain_of(png[:-4] + ".log")}
        print(f"{name}: " + ", ".join(f"{k} {v['value']:.3f} {'ok' if v['ok'] else 'FAIL'}" for k, v in checks.items())
              + ("" if checks else " measured " + ", ".join(f"{k} {v:.3f}" for k, v in m.items())), flush=True)
        if name in CROP_SCENES:
            crop(png, os.path.join(OUT, f"{o.label}_crop_{name}.png"))
        if o.compare:
            sheet(name, o.compare, o.label)
    results["_gaps"] = GAPS
    results["_failed"] = failed
    with open(os.path.join(OUT, f"{o.label}_results.json"), "w") as f:
        json.dump(results, f, indent=1)
    print("FAILED: " + ", ".join(failed) if failed else "ALL GATES PASS", "| gaps:", "; ".join(GAPS))
    return 1 if failed else 0


def crop(png, out, half=160):
    """A 1:1 crop (320 x 320) around the brightest object and a x3 zoom of its centre, side by side."""
    im = Image.open(png).convert("RGB")
    a = np.asarray(im).astype(float)
    y = luma(a)
    iy, ix = np.unravel_index(np.argmax(y), y.shape)
    box = (max(0, ix - half), max(0, iy - half), min(im.width, ix + half), min(im.height, iy + half))
    c = im.crop(box)
    z = im.crop((max(0, ix - half // 3), max(0, iy - half // 3), min(im.width, ix + half // 3),
                 min(im.height, iy + half // 3))).resize((c.width, c.height), Image.NEAREST)
    s = Image.new("RGB", (2 * c.width, c.height))
    s.paste(c, (0, 0))
    s.paste(z, (c.width, 0))
    s.save(out)


def sheet(name, a_label, b_label):
    pa, pb = os.path.join(OUT, f"{a_label}_{name}.png"), os.path.join(OUT, f"{b_label}_{name}.png")
    if not (os.path.exists(pa) and os.path.exists(pb)):
        return
    ia, ib = Image.open(pa).convert("RGB"), Image.open(pb).convert("RGB")
    w = 960
    ia, ib = ia.resize((w, int(ia.height * w / ia.width))), ib.resize((w, int(ib.height * w / ib.width)))
    s = Image.new("RGB", (2 * w, max(ia.height, ib.height) + 28), "black")
    s.paste(ia, (0, 28))
    s.paste(ib, (w, 28))
    d = ImageDraw.Draw(s)
    d.text((8, 6), f"A = {a_label}: {name}", fill="white")
    d.text((w + 8, 6), f"B = {b_label}", fill="white")
    s.save(os.path.join(OUT, f"sheet_{name}.png"))


if __name__ == "__main__":
    sys.exit(main())
