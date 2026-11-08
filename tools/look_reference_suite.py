"""Look reference scene suite (look fixes T13, unified appearance model): the definition of done for every change to
the eye adaptation, local adaptation, glare, white balance or display mapping. Renders each reference scene with
game/tests/render_check.gd (cockpit: cockpit_shots.gd), measures named regions of the PNG and gates them against
targets with their sources. Every later appearance change must pass all scenes (docs/subsystems/eye-adaptation.md).

Usage (needs a GPU, non-headless Godot; ~1-3 min per scene):
  python -I tools/look_reference_suite.py [--label B] [--only a,b] [--extra "--local_adaptation=off"] [--measure_only]
      [--compare A]   (writes an A|B sheet per scene)
Output: screenshots/look_fixes/t13/suite/<label>_<scene>.png / .log, <label>_results.json; exit code 1 on a failed gate.
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
PAD = ["--lat=25.9853", "--lon=-97.1873", "--hide_ui", "--weather=off"]
NOON = ["--utc=2026-11-07T18:15:00"] + PAD
MOON = ["--body=moon", "--lat=0.674", "--lon=23.473", "--utc=2026-11-17T08:00:00", "--hide_ui"]
LUNAR_NIGHT = ["--body=moon", "--lat=0", "--hide_ui"]


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
    return {"ground_luma": float(luma(ground).mean()), "sky_b_over_r": br(sky), "clipped": clipped(a.reshape(-1, 3)),
            "frame_luma": float(luma(a.reshape(-1, 3)).mean())}


def m_sky(a):
    sky = rows(a, 0.05, 0.40)
    return {"sky_b_over_r": br(sky), "ground_luma": float(luma(rows(a, 0.80, 0.98)).mean()),
            "frame_luma": float(luma(a.reshape(-1, 3)).mean())}


def m_moon_day(a):
    ground = rows(a, 0.65, 0.95)
    return {"ground_luma": float(luma(ground).mean()), "clipped": clipped(ground)}


def m_disc(a):
    d = blob(a)
    y = luma(d)
    return {"disc_px": int(len(d)), "disc_clipped": clipped(d), "disc_luma_sd": float(y.std()),
            "disc_max_channel": float(d.max(axis=1).mean()), "disc_b_over_r": br(d)}


def m_epic(a, path):
    if not os.path.exists(EPIC_REF):
        return {"skipped": 1.0}
    ref, own = epic_stats(EPIC_REF), epic_stats(path)
    corr = float(np.corrcoef(ref["_y"], own["_y"])[0, 1]) if len(ref["_y"]) == len(own["_y"]) else float("nan")
    return {"dark": own["dark"], "grey": own["grey"], "white": own["white"], "corr": corr}


def m_sun(a):
    return {"frame_luma": float(luma(a.reshape(-1, 3)).mean()), "white_px": int((a.min(axis=2) >= 250).sum()),
            "ground_luma": float(luma(rows(a, 0.80, 0.98)).mean())}


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
    "dusk_m5_5": (["--utc=2026-11-08T00:05:00"] + PAD + ["--azimuth=250", "--elevation=5", "--fov=100"], m_sky, {
        "sky_b_over_r": (1.2, 99, "[T5b fix 3: dusk B/R 1.36; user: twilight bluish]")}),
    "blue_hour_m7_7": (["--utc=2026-11-08T00:15:00"] + PAD + ["--azimuth=250", "--elevation=5", "--fov=100"], m_sky, {
        "sky_b_over_r": (1.6, 99, "[user 2026-11-08: the blue hour stays blue; T5b fix 3 B/R 1.91]")}),
    "lunar_day_apollo11": (MOON + ["--azimuth_from_sun=180", "--elevation=5", "--fov=100"], m_moon_day, {
        "ground_luma": (115, 175, "[user 2026-11-08: the Moon day stays bright (T6 old 128 accepted, 108 too dark); "
                                  "S: AS11-40-5903 regolith 117-147 8-bit luma at metered exposure]"),
        "clipped": (0.0, 0.01, "[call] regolith not clipped")}),
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
        "disc_b_over_r": (0.85, 99, "[user 2026-11-08: not cream (T10 0.81); E: the full Moon is seen near neutral "
                                    "grey-white at mesopic adaptation]")}),
    "earth_from_space_epic": (["--utc=2025-03-20T14:28:03", "--focus=earth",
                               "--view_from=gcrf:1548878.149868,49983.227147,173343.826562", "--distance=4e8",
                               "--fov=2.4", "--hide_ui", "--weather=" + EPIC_WTHR], "epic", {
        "white": (0.0, 0.30, "[S: DSCOVR EPIC 2025-03-20 14:28 white share 0.128 (linear Y >= 0.5 p99); T7 linear "
                             "camera 0.213; display before T13 0.444]"),
        "dark": (0.15, 0.6, "[S: EPIC dark share 0.375; T7 linear camera 0.338]"),
        "corr": (0.40, 1.0, "[S: EPIC Y correlation; T7 display 0.49]")}),
    "sun_in_view": (NOON + ["--look_at_sun", "--fov=100"], m_sun, {
        "white_px": (1, 10 ** 9, "[call] the Sun's disc is display white"),
        "frame_luma": (20, 230, "[call] the frame is not black / not blown out around the Sun")}),
    "cockpit_day": ("cockpit", m_cockpit, {
        "clipped": (0.0, 0.03, "[T10: windows show the sky, 0.08 % saturated]"),
        "frame_luma": (90, 210, "[D: crew photos crew1_1 / discovery: white cabin ~0.85-0.9 sRGB walls, dark console]")}),
}
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
    o = ap.parse_args()
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
        print(f"{name}: " + ", ".join(f"{k} {v['value']:.3f} {'ok' if v['ok'] else 'FAIL'}" for k, v in checks.items()),
              flush=True)
        if o.compare:
            sheet(name, o.compare, o.label)
    results["_gaps"] = GAPS
    results["_failed"] = failed
    with open(os.path.join(OUT, f"{o.label}_results.json"), "w") as f:
        json.dump(results, f, indent=1)
    print("FAILED: " + ", ".join(failed) if failed else "ALL GATES PASS", "| gaps:", "; ".join(GAPS))
    return 1 if failed else 0


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
