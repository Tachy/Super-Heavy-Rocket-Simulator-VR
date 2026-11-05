"""Contact sheet of the baked Moon detail textures: python tools/moon_textures/contact_sheet.py [textures_dir] [out.png]

Row 1 regolith heights s0..s2 (stretched to +-3 sigma), row 2 regolith normals, row 3 rock albedo 0..3,
row 4 rock normals 0..3. Default output: screenshots/moon_surface/task8/moon_detail_contact_sheet.png.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

REPO_ROOT = Path(__file__).resolve().parents[2]
CELL = 320


def tile(path: Path, height: bool = False) -> Image.Image:
    img = Image.open(path)
    if height:
        a = np.array(img).astype(np.float64)
        a = np.clip(0.5 + (a - a.mean()) / (6.0 * a.std()), 0.0, 1.0)
        img = Image.fromarray(np.rint(a * 255).astype(np.uint8))
    return img.convert("RGB").resize((CELL, CELL), Image.LANCZOS)


def main() -> int:
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else REPO_ROOT / "data" / "textures" / "moon_detail"
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else REPO_ROOT / "screenshots" / "moon_surface" / "task8" / "moon_detail_contact_sheet.png"
    rows = [[(src / f"regolith_height_s{i}.png", True, f"height s{i}") for i in range(3)],
            [(src / f"regolith_normal_s{i}.png", False, f"normal s{i}") for i in range(3)],
            [(src / f"rock_albedo_{i}.png", False, f"rock albedo {i}") for i in range(4)],
            [(src / f"rock_normal_{i}.png", False, f"rock normal {i}") for i in range(4)]]
    sheet = Image.new("RGB", (4 * CELL, 4 * CELL), (20, 20, 20))
    draw = ImageDraw.Draw(sheet)
    for r, row in enumerate(rows):
        for c, (path, is_height, label) in enumerate(row):
            sheet.paste(tile(path, is_height), (c * CELL, r * CELL))
            draw.text((c * CELL + 6, r * CELL + 4), label, fill=(255, 255, 0))
    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
