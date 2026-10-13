"""Telemetry tables from SpaceX Starship webcasts (OCR of the on-screen telemetry strip).

The webcast shows booster (left) and ship (right) speed [km/h] and altitude [km] plus the mission clock.
This tool reads them from a locally downloaded webcast (the video is not part of the repository):

  python tools/telemetry/webcast_telemetry.py locate <video>          (video time of liftoff, +-2 s)
  python tools/telemetry/webcast_telemetry.py scan  <video> <start_video_s> <duration_s> <raw.csv> [--fps 2] [--layout dials]
  python tools/telemetry/webcast_telemetry.py templates <video> <start_video_s> <duration_s> <fps> <raw.csv>
  python tools/telemetry/webcast_telemetry.py clean <raw.csv> <out.csv> --flight "Flight 5 (2024-10-13)" --url <webcast>

Download (any tool; e.g. yt-dlp, 1280x720 format):  yt-dlp -f replay-2750 <x.com broadcast url>
Requires ffmpeg, Tesseract (mission clock only) and Pillow. The numbers are read by template matching against
digit_templates.json: Tesseract confuses 5 and 9 in the webcast font. `templates` rebuilds that file from strips
whose readings pass the outlier test (a flight on a dark background, e.g. Flight 5's ascent).

`clean` derives the time from the mission clock (a frame showing T+00:00:30 lies in [30 s, 31 s)), keeps only
readings that parse, and rejects OCR errors with a robust neighbourhood test (median of +-6 samples). Output
columns: t_s (time after liftoff), booster_speed_kmh, booster_altitude_km, ship_speed_kmh, ship_altitude_km,
booster_propellant_pct, booster_engines_lit, ship_propellant_pct, ship_engines_lit (the last four from the round-dial
layout of flight 10 onwards); empty where nothing valid was read. Strip layout: altitude in whole km; round
dials: 0.1 km.
"""
import argparse
import collections
import csv
import json
import os
import re
import shutil
import statistics
import subprocess
from pathlib import Path

from PIL import Image, ImageOps

import dials

FFMPEG = shutil.which("ffmpeg") or r"C:\Users\johan\Documents\bin\ffmpeg.exe"
TESSERACT = shutil.which("tesseract") or r"C:\Program Files\Tesseract-OCR\tesseract.exe"

FRAME_W, FRAME_H = 1280, 720
STRIP_Y = 600  # the telemetry strip occupies y >= 600 of a 1280x720 frame
# Fields as fractions of the full frame (x0, y0, x1, y1), measured on the Flight 5 webcast layout.
BOXES = {
    "booster_speed": (0.1875, 0.842, 0.2320, 0.872),  # digits are right-aligned; the unit starts at ~0.236
    "booster_altitude": (0.1875, 0.874, 0.2320, 0.904),
    "ship_speed": (0.795, 0.842, 0.8530, 0.872),
    "ship_altitude": (0.795, 0.874, 0.8530, 0.904),
    "clock": (0.447, 0.878, 0.5516, 0.917),
}
# Before liftoff, while the telemetry strip is hidden, the clock sits in the top left corner.
CLOCK_TOP = (0.012, 0.028, 0.122, 0.070)
FIELDS = ["booster_speed", "booster_altitude", "ship_speed", "ship_altitude"]
# Outlier test: deviation from the neighbourhood median above max(relative * value, absolute) is rejected.
OUTLIER = {"speed": (0.04, 40.0), "altitude": (0.06, 2.0), "percent": (0.04, 1.0), "g": (0.1, 0.3)}  # km/h, km, %, g
# Output columns (first five as in the earliest tables) and their outlier test.
OUTPUT = [("booster_speed_kmh", "speed"), ("booster_altitude_km", "altitude"), ("ship_speed_kmh", "speed"),
          ("ship_altitude_km", "altitude"), ("booster_propellant_pct", "percent"), ("booster_engines_lit", "count"),
          ("ship_propellant_pct", "percent"), ("ship_engines_lit", "count")]
WINDOW = 6
THRESHOLD = 185  # grey level of text pixels
CLOCK_WINDOW = 10.0  # s


def ocr(img: Image.Image, whitelist: str) -> str:
    w, h = img.size
    img = img.convert("L").resize((w * 4, h * 4), Image.LANCZOS)
    # White text -> black on white; only near-white pixels count as text (bright hull texture behind the strip).
    img = img.point(lambda p: 0 if p > THRESHOLD else 255)
    img = ImageOps.expand(img, border=40, fill=255)
    path = Path(__file__).with_name(f"_ocr_{os.getpid()}.png")  # parallel scans must not share the file
    img.save(path)
    try:
        for psm in ("7", "8", "10"):  # a line, a word, a single character
            r = subprocess.run([TESSERACT, str(path), "stdout", "--psm", psm, "-c", f"tessedit_char_whitelist={whitelist}"],
                               capture_output=True, text=True)
            if r.stdout.strip():
                return r.stdout.strip()
        return ""
    finally:
        path.unlink(missing_ok=True)


GLYPH_W, GLYPH_H = 14, 20  # normalised digit raster
TEMPLATES_PATH = Path(__file__).with_name("digit_templates.json")  # strip layout font
DIALS_TEMPLATES_PATH = Path(__file__).with_name("digit_templates_dials.json")  # round-dial layout font
MAX_GLYPH_DISTANCE = 0.12  # mean squared difference above which a glyph is not a digit
_templates = {}


def glyphs(img: Image.Image, scale: int = 4, decimal: bool = False) -> list:
    """Digits of a field, left to right: each a GLYPH_W x GLYPH_H list of 0..1 values, or "." for a decimal point
    (decimal=True: a small blob on the baseline between digits)."""
    w, h = img.size
    g = img.convert("L").resize((w * scale, h * scale), Image.LANCZOS)
    W, H = g.size
    px = g.load()
    on = [[px[x, y] > THRESHOLD for x in range(W)] for y in range(H)]
    cols = [any(on[y][x] for y in range(H)) for x in range(W)]
    runs, x = [], 0
    while x < W:
        if cols[x]:
            x0 = x
            while x < W and cols[x]:
                x += 1
            runs.append((x0, x))
        else:
            x += 1
    boxes = []
    for x0, x1 in runs:
        rows = [y for y in range(H) if any(on[y][x] for x in range(x0, x1))]
        if rows:
            boxes.append((x0, x1, rows[0], rows[-1] + 1))
    digit_h = max((y1 - y0 for _, _, y0, y1 in boxes), default=0)
    bottom = max((y1 for _, _, y0, y1 in boxes if y1 - y0 >= 0.6 * digit_h), default=0)
    out = []
    for x0, x1, y0, y1 in boxes:
        if y1 - y0 >= 0.6 * digit_h and (x1 - x0) * (y1 - y0) >= 30:
            box_w = max(x1 - x0, int(round((y1 - y0) * GLYPH_W / GLYPH_H)))  # keep the aspect ratio: '1' stays narrow
            glyph = Image.new("L", (box_w, y1 - y0), 0)
            for yy in range(y0, y1):
                for xx in range(x0, x1):
                    if on[yy][xx]:
                        glyph.putpixel((xx - x0 + (box_w - (x1 - x0)) // 2, yy - y0), 255)
            small = glyph.resize((GLYPH_W, GLYPH_H), Image.BILINEAR)
            out.append([small.getpixel((i, j)) / 255.0 for j in range(GLYPH_H) for i in range(GLYPH_W)])
        elif decimal and out and y1 - y0 <= 0.3 * digit_h and abs(y1 - bottom) <= 0.12 * digit_h \
                and x1 - x0 <= 0.4 * digit_h:
            out.append(".")
    while out and out[-1] == ".":
        out.pop()
    return out


def load_templates(path: Path = TEMPLATES_PATH) -> dict:
    if path not in _templates:
        _templates[path] = json.loads(path.read_text()) if path.exists() else {}
    return _templates[path]


def read_number(img: Image.Image, path: Path = TEMPLATES_PATH, scale: int = 4, decimal: bool = False) -> str:
    templates = load_templates(path)
    if not templates:
        return ocr(img, "0123456789." if decimal else "0123456789")
    text = ""
    for glyph in glyphs(img, scale, decimal):
        if glyph == ".":
            text += "."
            continue
        best, best_d = None, float("inf")
        for digit, t in templates.items():
            d = sum((a - b) ** 2 for a, b in zip(glyph, t)) / len(t)
            if d < best_d:
                best, best_d = digit, d
        if best_d > MAX_GLYPH_DISTANCE:
            return ""  # something that is not a digit: no reading
        text += best
    return text


def read_dial_number(img: Image.Image) -> str:
    return read_number(img, DIALS_TEMPLATES_PATH, scale=2, decimal=True)


def read_strip(frame: Image.Image) -> dict:
    row = {name: read_number(crop(frame, name)) for name in FIELDS}
    row["clock"] = ocr(crop(frame, "clock"), "0123456789:+T-")
    return row


def crop(frame: Image.Image, name: str) -> Image.Image:
    x0, y0, x1, y1 = BOXES[name]
    return frame.crop((int(x0 * FRAME_W), int(y0 * FRAME_H), int(x1 * FRAME_W), int(y1 * FRAME_H)))


def frames(video: str, start: float, duration: float, fps: float, size: tuple):
    """Full frames at a fixed rate, scaled to `size`, straight from ffmpeg (no files). Cropping is done here, not in
    ffmpeg: its crop filter shifts 1080p streams coded with 1088 lines by several pixels."""
    w, h = size
    proc = subprocess.Popen([FFMPEG, "-v", "error", "-ss", f"{start}", "-i", video, "-t", f"{duration}", "-vf",
                             f"fps={fps},scale={w}:{h}", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                            stdout=subprocess.PIPE)
    n = w * h * 3
    try:
        while True:
            buf = proc.stdout.read(n)
            if len(buf) < n:
                break
            yield Image.frombytes("RGB", (w, h), buf)
    finally:
        proc.stdout.close()
        proc.wait()


def raw_fields(layout: str) -> list:
    return ["video_s", "clock", *FIELDS] if layout == "strip" else ["video_s", *dials.RAW_FIELDS]


def scan(args) -> None:
    size = (FRAME_W, FRAME_H) if args.layout == "strip" else (dials.W, dials.H)
    with open(args.raw, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=raw_fields(args.layout))
        w.writeheader()
        for i, frame in enumerate(frames(args.video, args.start, args.duration, args.fps, size)):
            row = read_strip(frame) if args.layout == "strip" else dials.read_frame(frame, ocr, read_dial_number)
            row["video_s"] = f"{args.start + i / args.fps:.2f}"
            w.writerow(row)
            if i % 100 == 0:
                f.flush()
                print(row, flush=True)


def accumulate(sums: dict, counts: dict, value: str, found: list) -> None:
    digits = [g for g in found if g != "."]
    text = value.replace(".", "")
    if len(digits) != len(text):
        return
    for digit, glyph in zip(text, digits):
        acc = sums.setdefault(digit, [0.0] * len(glyph))
        for k, v in enumerate(glyph):
            acc[k] += v
        counts[digit] = counts.get(digit, 0) + 1


def templates(args) -> None:
    """Mean digit bitmaps from a scan's raw readings and the same video window: only readings that pass the
    neighbourhood outlier test label glyphs (strip layout: a flight on a dark background, e.g. Flight 5's ascent)."""
    rows = list(csv.DictReader(open(args.raw)))
    t = [float(r["video_s"]) for r in rows]
    sums, counts = {}, {}
    if args.layout == "strip":
        labels = {}
        for name in ("booster_speed", "booster_altitude"):
            raw = [int(r[name]) if re.fullmatch(r"\d+", r[name]) else None for r in rows]
            labels[name] = clean_series(t, raw, "speed" if name.endswith("speed") else "altitude")
        for i, frame in enumerate(frames(args.video, args.start, args.duration, args.fps, (FRAME_W, FRAME_H))):
            if i >= len(rows):
                break
            for name, values in labels.items():
                if values[i] is not None:
                    accumulate(sums, counts, str(values[i]), glyphs(crop(frame, name)))
        path = TEMPLATES_PATH
    else:
        assigned = [dials.assign(r) for r in rows]
        cleaned = {col: clean_series(t, [a.get(col) for a in assigned], kind)
                   for col, kind in OUTPUT if kind in ("speed", "altitude")}
        for i, frame in enumerate(frames(args.video, args.start, args.duration, args.fps, (dials.W, dials.H))):
            if i >= len(rows):
                break
            for slot, col in dials.slot_columns(rows[i]).items():
                v = cleaned.get(col, [None] * len(rows))[i]
                if v is None:
                    continue
                text = f"{v:.1f}" if col.endswith("_km") else f"{v:.0f}"
                accumulate(sums, counts, text, glyphs(frame.crop(dials.number_box(slot)), 2, decimal=True))
        path = DIALS_TEMPLATES_PATH
    result = {d: [v / counts[d] for v in sums[d]] for d in sorted(sums)}
    path.write_text(json.dumps(result))
    print(f"templates for digits {sorted(result)} from {dict(sorted(counts.items()))} samples -> {path}")


def locate(args) -> None:
    """Video time of liftoff, approximately (+-2 s: seeking lands on key frames); refine with `clean`."""
    path = Path(__file__).with_name(f"_locate_{os.getpid()}.png")
    t = args.step
    try:
        while t < args.max:
            r = subprocess.run([FFMPEG, "-v", "error", "-y", "-ss", f"{t}", "-i", args.video, "-frames:v", "1", "-vf",
                                f"scale={FRAME_W}:{FRAME_H}", str(path)], capture_output=True)
            if r.returncode != 0 or not path.exists():
                break
            frame = Image.open(path)
            c = clock_s(ocr(crop(frame, "clock"), "0123456789:+T-"))
            if c is None:  # round-dial layout: clock lower in the frame
                x0, y0, x1, y1 = dials.CLOCK_BOX
                k = FRAME_W / dials.W
                c = clock_s(ocr(frame.crop((int(x0 * k), int(y0 * k), int(x1 * k), int(y1 * k))), "0123456789:+T-"))
            if c is None:
                x0, y0, x1, y1 = CLOCK_TOP
                c = clock_s(ocr(frame.crop((int(x0 * FRAME_W), int(y0 * FRAME_H), int(x1 * FRAME_W), int(y1 * FRAME_H))),
                                "0123456789:+T-"))
            if c is not None:
                print(f"video {t:7.0f} s: T{c:+d} s -> liftoff at video ~{t - c:.0f} s", flush=True)
                if c >= 0:
                    return
            t += args.step
    finally:
        path.unlink(missing_ok=True)


def clock_s(text: str):
    """Mission time of a clock reading 'T+hh:mm:ss' / 'T-hh:mm:ss' (None: not fully read)."""
    m = re.fullmatch(r"T([+-])(\d\d?):(\d\d):(\d\d)", text.replace(" ", ""))
    if not m:
        return None
    s = int(m.group(2)) * 3600 + int(m.group(3)) * 60 + int(m.group(4))
    return s if m.group(1) == "+" else -s


def clock_magnitude(text: str):
    """Seconds of a clock reading whose 'T' or sign may be misread ('00:05:50', '-00:01:31')."""
    m = re.search(r"(\d\d?):(\d\d):(\d\d)$", text.replace(" ", ""))
    return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + int(m.group(3)) if m else None


def clean_series(t: list, values: list, kind: str) -> list:
    rel, absolute = OUTLIER[kind]
    out = []
    for i, v in enumerate(values):
        if v is None:
            out.append(None)
            continue
        near = [values[j] for j in range(max(0, i - WINDOW), min(len(values), i + WINDOW + 1))
                if j != i and values[j] is not None and abs(t[j] - t[i]) <= WINDOW]
        if len(near) < 3:
            out.append(None)
            continue
        m = statistics.median(near)
        out.append(v if abs(v - m) <= max(rel * abs(m), absolute) else None)
    return out


def clean(args) -> None:
    rows = list(csv.DictReader(open(args.raw)))
    # Mission time per frame: a frame at video time v showing T+n puts the offset (video - mission time) in
    # (v - n - 1, v - n]. Intersecting these intervals over +-CLOCK_WINDOW s pins the offset to a fraction of a
    # second and follows cuts or pauses in the webcast; readings that contradict the rest (OCR errors) are dropped
    # by taking the intersection of the central half of the bounds.
    video = [float(r["video_s"]) for r in rows]
    # The sign ('T+'/'T-') is often misread, the digits rarely. Each reading allows two offsets (video - mission
    # time): v - n and v + n. The true one is the same for all readings (until a cut in the webcast), the other one
    # drifts, so the most common offset within +-60 s of video (1 s bins) picks the sign.
    candidates = []
    for v, r in zip(video, rows):
        n = clock_magnitude(r["clock"])
        if n is not None:
            candidates.append((v, round(v - n), round(v + n)))
    bounds = []
    for v, r in zip(video, rows):
        n = clock_magnitude(r["clock"])
        if n is None:
            continue
        votes = collections.Counter()
        for vc, a, b in candidates:
            if abs(vc - v) <= 60.0:
                votes[a] += 1
                if b != a:
                    votes[b] += 1
        ref = votes.most_common(1)[0][0]
        c = min((n, -n), key=lambda x: abs(v - x - ref))
        if abs(v - c - ref) > 1.5:
            continue
        bounds.append((v, v - c - 1.0, v - c))
    t = []
    for v in video:
        near = [(lo, hi) for vb, lo, hi in bounds if abs(vb - v) <= CLOCK_WINDOW]
        if len(near) < 4:
            t.append(None)
            continue
        lows = sorted(lo for lo, _ in near)
        highs = sorted(hi for _, hi in near)
        lo = lows[(3 * len(lows)) // 4]  # robust max of the lower bounds
        hi = highs[len(highs) // 4]      # robust min of the upper bounds
        t.append(v - 0.5 * (lo + hi))
    t0 = statistics.median([v - x for v, x in zip(video, t) if x is not None])
    keep = [i for i, x in enumerate(t) if x is not None]
    rows = [rows[i] for i in keep]
    t = [t[i] for i in keep]
    # Values per output column (raw readings of the layout -> vehicle values), then the outlier test.
    output = OUTPUT + [("g_force", "g")] if args.vehicle == "falcon9" else OUTPUT
    if args.vehicle == "falcon9":
        assigned = [dials.assign_falcon9(r) for r in rows]
        values = {col: [a.get(col) for a in assigned] for col, _ in output}
    elif "label_left" in rows[0]:
        assigned = [dials.assign(r) for r in rows]
        values = {col: [a.get(col) for a in assigned] for col, _ in output}
    else:
        values = {col: [None] * len(rows) for col, _ in output}
        for name in FIELDS:
            col = f"{name}_kmh" if name.endswith("speed") else f"{name}_km"
            values[col] = [int(r[name]) if re.fullmatch(r"\d+", r[name]) else None for r in rows]
    series = {}
    for col, kind in output:
        series[col] = majority_series(t, values[col]) if kind == "count" else clean_series(t, values[col], kind)
    with open(args.out, "w", newline="") as f:
        f.write(f"# {args.flight} - SpaceX webcast telemetry, read by tools/telemetry/webcast_telemetry.py (OCR)\n")
        f.write(f"# source: {args.url}\n")
        f.write("# t_s: time after liftoff (mission clock); speeds km/h, altitudes km as displayed (rounded); propellant:\n"
                "# the webcast's gauge in % (round-dial layout only); engines_lit: engine icons lit\n")
        w = csv.writer(f)
        w.writerow(["t_s"] + [col for col, _ in output])
        kept = 0
        for k in range(len(rows)):
            vals = [series[col][k] for col, _ in output]
            if all(v is None for v in vals):
                continue
            kept += 1
            w.writerow([f"{t[k]:.2f}"] + ["" if v is None else (f"{v:g}" if isinstance(v, float) else v) for v in vals])
    valid = {col: sum(v is not None for v in series[col]) for col, _ in output}
    print(f"liftoff at video ~{t0:.2f} s (median offset); {kept} rows; valid readings {valid}")


def majority_series(t: list, values: list) -> list:
    """Integer readings (engines lit): kept where they equal the most common value of the +-WINDOW s neighbourhood."""
    out = []
    for k, v in enumerate(values):
        if v is None:
            out.append(None)
            continue
        near = [values[j] for j in range(max(0, k - WINDOW), min(len(values), k + WINDOW + 1))
                if values[j] is not None and abs(t[j] - t[k]) <= WINDOW]
        out.append(v if near.count(v) * 2 > len(near) else None)
    return out


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("scan")
    s.add_argument("video")
    s.add_argument("start", type=float)
    s.add_argument("duration", type=float)
    s.add_argument("raw")
    s.add_argument("--fps", type=float, default=2.0)
    s.add_argument("--layout", choices=["strip", "dials"], default="strip",
                   help="strip: flights up to 9 (text strip); dials: round dials (flight 10 onwards)")
    tp = sub.add_parser("templates")
    tp.add_argument("video")
    tp.add_argument("start", type=float)
    tp.add_argument("duration", type=float)
    tp.add_argument("fps", type=float)
    tp.add_argument("raw")
    tp.add_argument("--layout", choices=["strip", "dials"], default="strip")
    lo = sub.add_parser("locate")
    lo.add_argument("video")
    lo.add_argument("--step", type=float, default=60.0)
    lo.add_argument("--max", type=float, default=4 * 3600.0)
    c = sub.add_parser("clean")
    c.add_argument("raw")
    c.add_argument("out")
    c.add_argument("--flight", required=True)
    c.add_argument("--url", required=True)
    c.add_argument("--vehicle", choices=["starship", "falcon9"], default="starship",
                   help="falcon9: STAGE 1 / STAGE 2 sides, G-FORCE dial before separation (dials.assign_falcon9)")
    args = p.parse_args()
    {"scan": scan, "templates": templates, "locate": locate, "clean": clean}[args.cmd](args)


if __name__ == "__main__":
    main()
