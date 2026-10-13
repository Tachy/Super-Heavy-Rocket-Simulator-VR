"""Telemetry tables from SpaceX Starship webcasts (OCR of the on-screen telemetry strip).

The webcast shows booster (left) and ship (right) speed [km/h] and altitude [km] plus the mission clock.
This tool reads them from a locally downloaded webcast (the video is not part of the repository):

  python tools/telemetry/webcast_telemetry.py locate <video>          (video time of liftoff, +-2 s)
  python tools/telemetry/webcast_telemetry.py scan  <video> <start_video_s> <duration_s> <raw.csv> [--fps 2]
  python tools/telemetry/webcast_telemetry.py read  <frames_dir> <start_video_s> <fps> <raw.csv>   (re-read kept strips)
  python tools/telemetry/webcast_telemetry.py templates <frames_dir> <raw.csv>   (digit templates from a clean scan)
  python tools/telemetry/webcast_telemetry.py clean <raw.csv> <out.csv> --flight "Flight 5 (2024-10-13)" --url <webcast>

Download (any tool; e.g. yt-dlp, 1280x720 format):  yt-dlp -f replay-2750 <x.com broadcast url>
Requires ffmpeg, Tesseract (mission clock only) and Pillow. The numbers are read by template matching against
digit_templates.json: Tesseract confuses 5 and 9 in the webcast font. `templates` rebuilds that file from strips
whose readings pass the outlier test (a flight on a dark background, e.g. Flight 5's ascent).

`clean` derives the time from the mission clock (a frame showing T+00:00:30 lies in [30 s, 31 s)), keeps only
readings that parse, and rejects OCR errors with a robust neighbourhood test (median of +-6 samples). Output
columns: t_s (time after liftoff), booster_speed_kmh, booster_altitude_km, ship_speed_kmh, ship_altitude_km;
empty where nothing valid was read. The webcast rounds altitude to whole km and speed to whole km/h.
"""
import argparse
import csv
import json
import os
import re
import shutil
import statistics
import subprocess
from pathlib import Path

from PIL import Image, ImageOps

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
OUTLIER = {"speed": (0.04, 40.0), "altitude": (0.06, 2.0)}  # km/h, km
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
TEMPLATES_PATH = Path(__file__).with_name("digit_templates.json")
MAX_GLYPH_DISTANCE = 0.12  # mean squared difference above which a glyph is not a digit
_templates = None


def glyphs(img: Image.Image) -> list:
    """Digit bitmaps of a field (left to right), each a GLYPH_W x GLYPH_H list of 0..1 values."""
    w, h = img.size
    g = img.convert("L").resize((w * 4, h * 4), Image.LANCZOS)
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
    out = []
    for x0, x1 in runs:
        rows = [y for y in range(H) if any(on[y][x] for x in range(x0, x1))]
        if not rows:
            continue
        y0, y1 = rows[0], rows[-1] + 1
        if y1 - y0 < 0.4 * H or (x1 - x0) * (y1 - y0) < 30:  # specks and partial strokes are not digits
            continue
        box_w = max(x1 - x0, int(round((y1 - y0) * GLYPH_W / GLYPH_H)))  # keep the aspect ratio: '1' stays narrow
        glyph = Image.new("L", (box_w, y1 - y0), 0)
        for yy in range(y0, y1):
            for xx in range(x0, x1):
                if on[yy][xx]:
                    glyph.putpixel((xx - x0 + (box_w - (x1 - x0)) // 2, yy - y0), 255)
        small = glyph.resize((GLYPH_W, GLYPH_H), Image.BILINEAR)
        out.append([small.getpixel((i, j)) / 255.0 for j in range(GLYPH_H) for i in range(GLYPH_W)])
    return out


def load_templates() -> dict:
    global _templates
    if _templates is None:
        _templates = json.loads(TEMPLATES_PATH.read_text()) if TEMPLATES_PATH.exists() else {}
    return _templates


def read_number(img: Image.Image) -> str:
    templates = load_templates()
    if not templates:
        return ocr(img, "0123456789")
    text = ""
    for glyph in glyphs(img):
        best, best_d = None, float("inf")
        for digit, t in templates.items():
            d = sum((a - b) ** 2 for a, b in zip(glyph, t)) / len(t)
            if d < best_d:
                best, best_d = digit, d
        if best_d > MAX_GLYPH_DISTANCE:
            return ""  # something that is not a digit: no reading
        text += best
    return text


def read_strip(frame: Image.Image) -> dict:
    row = {name: read_number(crop(frame, name)) for name in FIELDS}
    row["clock"] = ocr(crop(frame, "clock"), "0123456789:+T-")
    return row


def crop(frame: Image.Image, name: str) -> Image.Image:
    x0, y0, x1, y1 = BOXES[name]
    return frame.crop((int(x0 * FRAME_W), int(y0 * FRAME_H), int(x1 * FRAME_W), int(y1 * FRAME_H)))


def scan(args) -> None:
    frames = Path(args.raw).with_suffix("")
    frames.mkdir(parents=True, exist_ok=True)
    for old in frames.glob("f_*.png"):
        old.unlink()
    # One decoding pass at a fixed rate: exact frame times (seeking per frame lands on key frames).
    subprocess.run([FFMPEG, "-v", "error", "-ss", f"{args.start}", "-i", args.video, "-t", f"{args.duration}", "-vf",
                    f"fps={args.fps},scale={FRAME_W}:{FRAME_H},crop={FRAME_W}:{FRAME_H - STRIP_Y}:0:{STRIP_Y}",
                    str(frames / "f_%05d.png")], check=True)
    read_frames(frames, args.start, args.fps, args.raw)


def read_frames(frames: Path, start: float, fps: float, raw: str) -> None:
    rows = []
    for i, path in enumerate(sorted(frames.glob("f_*.png"))):
        frame = Image.new("RGB", (FRAME_W, FRAME_H))
        frame.paste(Image.open(path), (0, STRIP_Y))
        row = {"video_s": f"{start + i / fps:.2f}", **read_strip(frame)}
        rows.append(row)
        if i % 100 == 0:
            print(row, flush=True)
    with open(raw, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["video_s", "clock", *FIELDS])
        w.writeheader()
        w.writerows(rows)


def read(args) -> None:
    read_frames(Path(args.frames), args.start, args.fps, args.raw)


def templates(args) -> None:
    """Mean digit bitmaps from kept strips and the raw readings of the same scan: only readings that pass the
    neighbourhood outlier test label glyphs (use a flight read on a dark background, e.g. Flight 5's ascent)."""
    rows = list(csv.DictReader(open(args.raw)))
    frames = sorted(Path(args.frames).glob("f_*.png"))
    if len(rows) != len(frames):
        raise SystemExit(f"{len(rows)} readings but {len(frames)} strips: not the same scan")
    t = [float(r["video_s"]) for r in rows]
    labels = {}
    for name in ("booster_speed", "booster_altitude"):
        raw = [int(r[name]) if re.fullmatch(r"\d+", r[name]) else None for r in rows]
        labels[name] = clean_series(t, raw, "speed" if name.endswith("speed") else "altitude")
    sums, counts = {}, {}
    for i, path in enumerate(frames):
        frame = Image.new("RGB", (FRAME_W, FRAME_H))
        frame.paste(Image.open(path), (0, STRIP_Y))
        for name, values in labels.items():
            if values[i] is None:
                continue
            value = str(values[i])
            found = glyphs(crop(frame, name))
            if len(found) != len(value):
                continue
            for digit, glyph in zip(value, found):
                acc = sums.setdefault(digit, [0.0] * len(glyph))
                for k, v in enumerate(glyph):
                    acc[k] += v
                counts[digit] = counts.get(digit, 0) + 1
    result = {d: [v / counts[d] for v in sums[d]] for d in sorted(sums)}
    TEMPLATES_PATH.write_text(json.dumps(result))
    print(f"templates for digits {sorted(result)} from {dict(sorted(counts.items()))} samples -> {TEMPLATES_PATH}")


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
    m = re.fullmatch(r"T([+-])(\d\d):(\d\d):(\d\d)", text.replace(" ", ""))
    if not m:
        return None
    s = int(m.group(2)) * 3600 + int(m.group(3)) * 60 + int(m.group(4))
    return s if m.group(1) == "+" else -s


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
    bounds = [(v, v - c - 1.0, v - c) for v, r in zip(video, rows) if (c := clock_s(r["clock"])) is not None]
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
    series = {}
    for name in FIELDS:
        raw = [int(r[name]) if re.fullmatch(r"\d+", r[name]) else None for r in rows]
        series[name] = clean_series(t, raw, "speed" if name.endswith("speed") else "altitude")
    with open(args.out, "w", newline="") as f:
        f.write(f"# {args.flight} - SpaceX webcast telemetry, read by tools/telemetry/webcast_telemetry.py (OCR)\n")
        f.write(f"# source: {args.url}\n")
        f.write("# t_s: time after liftoff (mission clock); speeds km/h, altitudes km as displayed (rounded)\n")
        w = csv.writer(f)
        w.writerow(["t_s", "booster_speed_kmh", "booster_altitude_km", "ship_speed_kmh", "ship_altitude_km"])
        kept = 0
        for i in range(len(rows)):
            vals = [series[n][i] for n in FIELDS]
            if all(v is None for v in vals):
                continue
            kept += 1
            w.writerow([f"{t[i]:.2f}"] + ["" if v is None else v for v in vals])
    valid = {n: sum(v is not None for v in series[n]) for n in FIELDS}
    print(f"liftoff at video ~{t0:.2f} s (median offset); {kept} rows; valid readings {valid}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("scan")
    s.add_argument("video")
    s.add_argument("start", type=float)
    s.add_argument("duration", type=float)
    s.add_argument("raw")
    s.add_argument("--fps", type=float, default=2.0)
    rd = sub.add_parser("read")
    rd.add_argument("frames")
    rd.add_argument("start", type=float)
    rd.add_argument("fps", type=float)
    rd.add_argument("raw")
    tp = sub.add_parser("templates")
    tp.add_argument("frames")
    tp.add_argument("raw")
    lo = sub.add_parser("locate")
    lo.add_argument("video")
    lo.add_argument("--step", type=float, default=60.0)
    lo.add_argument("--max", type=float, default=4 * 3600.0)
    c = sub.add_parser("clean")
    c.add_argument("raw")
    c.add_argument("out")
    c.add_argument("--flight", required=True)
    c.add_argument("--url", required=True)
    args = p.parse_args()
    {"scan": scan, "read": read, "templates": templates, "locate": locate, "clean": clean}[args.cmd](args)


if __name__ == "__main__":
    main()
