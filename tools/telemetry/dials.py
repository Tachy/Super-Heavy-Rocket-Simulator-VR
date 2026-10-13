"""Reader for the webcast layout with round dials (flight 10 onwards), 1920x1080 frames.

Bottom row, four slots on a line at y ~985: L1, L2 (left), R1, R2 (right), the mission clock in the centre and a
group label under each side ("SUPER HEAVY" / "STARSHIP"). A slot shows a number dial (speed km/h, altitude km
with one decimal), an engine icon (one dot per engine, lit = running) inside a propellant gauge arc, or an
attitude sphere. Which vehicle a slot belongs to follows from the labels (see `assign`).
"""
import math
import re

from PIL import Image

W, H = 1920, 1080
SLOT_Y = 985.0
SLOTS = {"L1": 155.0, "L2": 322.0, "R1": 1598.0, "R2": 1765.0}
NUMBER_HALF_W, NUMBER_TOP, NUMBER_BOTTOM = 55, 968, 1008
CLOCK_BOX = (825, 960, 1090, 1012)
LABEL_BOXES = {"left": (40, 1030, 360, 1060), "right": (1520, 1030, 1830, 1060)}
# Propellant gauge: arc around the engine icon (grid search on full flight 14 frames: booster at L1, ship at R2);
# centre offset from the slot centre and radius. The track runs clockwise from ~211 deg to ~-31 deg (0 = right,
# 90 = up); white = remaining propellant.
ARC = {"L1": (0.0, 4.0, 67.0), "R2": (-2.0, 3.5, 66.0)}
ARC_DEFAULT = (-1.0, 3.75, 66.5)
ARC_FROM_DEG, ARC_TO_DEG = 240.0, -64.0
TRACK_LEVEL, FILL_LEVEL = 55, 175  # grey track (ship ~75, booster ~110) / white fill
DOT_RADIUS = 52  # engine dots lie within this radius of the slot centre
LIT_LEVEL, DOT_LEVEL = 200, 95


def number_box(slot: str):
    x = SLOTS[slot]
    return (int(x - NUMBER_HALF_W), NUMBER_TOP, int(x + NUMBER_HALF_W), NUMBER_BOTTOM)


def gauge_fraction(gray: Image.Image, slot: str):
    """Filled fraction of the propellant arc around `slot` (None: no arc)."""
    px = gray.load()
    dx, dy, radius = ARC.get(slot, ARC_DEFAULT)
    xc, yc = SLOTS[slot] + dx, SLOT_Y + dy

    def sample(a):
        return max(px[int(round(xc + r * math.cos(math.radians(a)))), int(round(yc - r * math.sin(math.radians(a))))]
                   for r in (radius - 1.5, radius - 0.5, radius + 0.5, radius + 1.5))

    angles = [a / 4 for a in range(int(ARC_FROM_DEG * 4), int(ARC_TO_DEG * 4), -1)]
    vals = [sample(a) for a in angles]
    on = [v > TRACK_LEVEL for v in vals]
    if True not in on:
        return None
    i0 = on.index(True)
    i1 = len(on) - 1 - on[::-1].index(True)
    span = angles[i0] - angles[i1]
    if not 230.0 <= span <= 250.0:  # the track spans ~242 deg; anything else is background
        return None
    last, gap = i0, 0
    for k in range(i0 + 8, i1 + 1):  # skip the end-cap tick (2 deg); tolerate gaps up to 1 deg
        if vals[k] > FILL_LEVEL:
            last, gap = k, 0
        else:
            gap += 1
            if gap > 4:
                break
    return (last - i0) / (i1 - i0)


def engine_dots(gray: Image.Image, slot: str):
    """(lit, total) engine dots inside the icon of `slot` (connected components)."""
    px = gray.load()
    xc, yc = SLOTS[slot], SLOT_Y
    r2 = DOT_RADIUS * DOT_RADIUS
    seen = set()

    def components(level):
        found = []
        for y in range(int(yc - DOT_RADIUS), int(yc + DOT_RADIUS) + 1):
            for x in range(int(xc - DOT_RADIUS), int(xc + DOT_RADIUS) + 1):
                if (x, y) in seen or (x - xc) ** 2 + (y - yc) ** 2 > r2 or px[x, y] <= level:
                    continue
                stack, size = [(x, y)], 0
                seen.add((x, y))
                while stack:
                    cx, cy = stack.pop()
                    size += 1
                    for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                        if (nx, ny) not in seen and (nx - xc) ** 2 + (ny - yc) ** 2 <= r2 and px[nx, ny] > level:
                            seen.add((nx, ny))
                            stack.append((nx, ny))
                if size >= 12:
                    found.append(size)
        return found

    lit = components(LIT_LEVEL)
    seen.clear()
    total = components(DOT_LEVEL)
    return len(lit), len(total)


def read_frame(frame: Image.Image, ocr, number) -> dict:
    """Raw readings of one frame. `ocr(img, whitelist)` reads a text field, `number(img)` a dial's number."""
    gray = frame.convert("L")
    row = {"clock": ocr(frame.crop(CLOCK_BOX), "0123456789:+T-")}
    for side, box in LABEL_BOXES.items():
        row[f"label_{side}"] = ocr(frame.crop(box), "ABCDEFGHIJKLMNOPQRSTUVWXYZ ")
    for slot in SLOTS:
        row[slot] = number(frame.crop(number_box(slot)))
        # Raw icon readings everywhere; `assign` uses them only where the layout puts an engine icon.
        lit, total = engine_dots(gray, slot)
        row[f"{slot}_engines"] = f"{lit}/{total}"
        g = gauge_fraction(gray, slot)
        row[f"{slot}_gauge"] = "" if g is None else f"{100.0 * g:.1f}"
    return row


RAW_FIELDS = ["clock", "label_left", "label_right"] + [f"{s}{k}" for s in SLOTS for k in ("", "_gauge", "_engines")]


def _number(text: str):
    return float(text) if re.fullmatch(r"\d+(\.\d)?", text) else None


def _dots(row: dict, slot: str):
    m = re.fullmatch(r"(\d+)/(\d+)", row.get(f"{slot}_engines", ""))
    return (int(m.group(1)), int(m.group(2))) if m else (0, 0)


def _phase(row: dict) -> tuple:
    """(left, right) contents: left 'booster' | 'ship' | 'booster_icon' | '', right 'stack' | 'ship' | ''.

    Stack: no labels, right dials speed/altitude, left the booster's engine icon. At hot staging the right
    altitude dial becomes the ship's engine icon (6 dots, no number) - from then on the right side is the ship,
    the left icon still the booster (boostback) until the labels appear ('SUPER HEAVY' left, 'STARSHIP' right).
    After the booster's flight the ship is alone: its speed/altitude dials on the left without a label, its engine
    icon on the right."""
    left_label, right_label = row["label_left"].upper(), row["label_right"].upper()
    # Ship engine icon: 6 dots (fewer drawn when an engine is out), no number.
    right_ship_icon = _number(row["R2"]) is None and 4 <= _dots(row, "R2")[1] <= 7
    left_numbers = _number(row["L1"]) is not None and _number(row["L2"]) is not None
    if "STARSHIP" in right_label or (right_ship_icon and _number(row["R1"]) is not None):
        right = "ship"
    elif "HEAVY" not in left_label and "STARSHIP" not in left_label and _number(row["R1"]) is not None             and _number(row["R2"]) is not None:  # other label text is OCR noise from the background
        right = "stack"
    else:
        right = ""
    if "HEAVY" in left_label and _number(row["L1"]) is None and _dots(row, "L1")[1] >= 1:
        left = "booster_icon_alt"  # landing approach: engine icon (+ propellant arc) instead of the speed dial
    elif "HEAVY" in left_label:
        left = "booster"
    elif "STARSHIP" in left_label or (left_numbers and _number(row["R1"]) is None and _number(row["R2"]) is None):
        # labelled, or the ship alone after the booster's flight: unlabelled numbers on the left and none on the
        # right (its engine icon there is not always read - flight 14's entry shows it dimmed)
        left = "ship"
    elif _number(row["L1"]) is None and _dots(row, "L1")[1] >= 1 and right in ("stack", "ship"):
        left = "booster_icon"  # dark (shut-down) engines are not drawn: during the boostback only a few dots
    else:
        left = ""
    return left, right


def slot_columns(row: dict) -> dict:
    """Which output column each number slot shows in this frame (see `assign`)."""
    left, right = _phase(row)
    out = {}
    if right == "stack":
        out.update({"R1": "booster_speed_kmh", "R2": "booster_altitude_km"})
    elif right == "ship":
        out["R1"] = "ship_speed_kmh"
    if left == "booster":
        out.update({"L1": "booster_speed_kmh", "L2": "booster_altitude_km"})
    elif left == "booster_icon_alt":
        out["L2"] = "booster_altitude_km"
    elif left == "ship":
        out.update({"L1": "ship_speed_kmh", "L2": "ship_altitude_km"})
    return out


def assign(row: dict) -> dict:
    """Vehicle values from one raw row: speed_kmh, altitude_km, propellant_pct, engines_lit for booster and ship
    (see `_phase` for which slot shows what)."""
    left, right = _phase(row)

    def gauge(slot):
        return float(row[f"{slot}_gauge"]) if row[f"{slot}_gauge"] else None

    out = {}
    for slot, col in slot_columns(row).items():
        out[col] = _number(row[slot])
    if right == "stack":
        out["ship_speed_kmh"] = out.get("booster_speed_kmh")
        out["ship_altitude_km"] = out.get("booster_altitude_km")
    if right == "ship" or left == "ship":
        if _number(row["R2"]) is None and 4 <= _dots(row, "R2")[1] <= 7:
            out["ship_propellant_pct"] = gauge("R2")
            out["ship_engines_lit"] = _dots(row, "R2")[0]
    if left in ("booster_icon", "booster_icon_alt") or right == "stack":
        out["booster_propellant_pct"] = gauge("L1")
        out["booster_engines_lit"] = _dots(row, "L1")[0]
    return out


def assign_falcon9(row: dict) -> dict:
    """Falcon 9 webcasts (same round-dial layout): before stage separation the left dials show the stack's speed and
    altitude, the right ones the acceleration ("G-FORCE", one decimal) and the first stage's engine icon (9 dots,
    lit = running) inside its propellant gauge. After separation the labels read "STAGE 1" (left) and "STAGE 2"
    (right) and each side shows its stage's speed and altitude; for the first stage's burns (entry, landing) the
    speed dial gives way to its engine icon and gauge. After the first stage's flight the second stage's numbers move
    to the left without a label. Columns: booster_* = first stage, ship_* = second stage, g_force."""
    l1, l2, r1, r2 = (_number(row[k]) for k in ("L1", "L2", "R1", "R2"))

    def gauge(slot):
        return float(row[f"{slot}_gauge"]) if row[f"{slot}_gauge"] else None

    staged = "STAGE" in row["label_left"].upper() or "STAGE" in row["label_right"].upper() or (r1 is not None and r1 > 100)
    out = {}
    if staged:
        if l1 is not None:
            out["booster_speed_kmh"] = l1
        elif _dots(row, "L1")[1] >= 1:  # engine icon: dark engines are not counted, lit ones are
            out["booster_engines_lit"] = _dots(row, "L1")[0]
            out["booster_propellant_pct"] = gauge("L1")
        out["booster_altitude_km"] = l2
        out["ship_speed_kmh"] = r1
        out["ship_altitude_km"] = r2
    elif l1 is not None and l1 > 10000.0 and r1 is None:  # second stage alone (the stack never flies this fast)
        out["ship_speed_kmh"] = l1
        out["ship_altitude_km"] = l2
    else:
        for col in ("booster_speed_kmh", "ship_speed_kmh"):
            out[col] = l1
        for col in ("booster_altitude_km", "ship_altitude_km"):
            out[col] = l2
        if r1 is not None and r1 < 15.0:
            out["g_force"] = r1
        if r2 is None and _dots(row, "R2")[1] >= 1:
            out["booster_engines_lit"] = _dots(row, "R2")[0]
            out["booster_propellant_pct"] = gauge("R2")
    return out
