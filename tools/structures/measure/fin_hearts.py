"""Opening shapes of the grid-fin lattice seen at an angle (numpy ray test, no Blender): where does a line of sight at
`tilt` degrees off the cell axis, towards the chord axis, pass through the lattice of cusped saddle walls?

Usage: python fin_hearts.py <out.png> <tilt_deg> <depth> <dip_fwd> <dip_aft> <soft> [wall_m] [pitch_m] [rot_deg]
Several settings: pass `--grid` as the first argument followed by groups "tilt,depth,dip_fwd,dip_aft,soft" (one panel each).
Shows the 5 x 5 cells around the origin; white = sky seen through, dark = wall. `rot_deg` turns the picture (the view of the
installed-fin template has the vehicle axis vertical: the tilt direction is vertical in the panel).
"""
import math
import sys

import numpy as np
from PIL import Image


def soft_profile(u, soft):
    return (1.0 - soft) * np.sin(np.pi * u) + soft * 0.5 * (1.0 - np.cos(2.0 * np.pi * u))


def panel(tilt_deg, depth, dip_fwd, dip_aft, soft, az_deg=0.0, wall=0.04, pitch=0.35, size=360, extent=1.3):
    """Image (size x size): horizontal = span axis y, vertical = chord axis z (the tilt direction) of the sky seen through."""
    ys = np.linspace(-extent, extent, size)
    zs = np.linspace(extent, -extent, size)
    y, z = np.meshgrid(ys, zs)
    blocked = np.zeros(y.shape, bool)
    t = math.tan(math.radians(tilt_deg))
    s2 = math.sqrt(0.5)
    for x in np.linspace(0.0, depth, 60):  # x from the aft face (0) to the forward face (depth)
        yy, zz = y + x * t * math.sin(math.radians(az_deg)), z + x * t * math.cos(math.radians(az_deg))  # drift of the line of sight
        for sign in (1.0, -1.0):
            across = (yy * s2 + sign * zz * s2) if sign > 0 else (yy * s2 - zz * s2)
            along = (yy * s2 - zz * s2) if sign > 0 else (yy * s2 + zz * s2)
            near = np.abs(across - np.round(across / pitch) * pitch) < wall / 2.0
            u = along / pitch - np.floor(along / pitch)
            h = soft_profile(u, soft)
            inside = (x > dip_aft * h) & (x < depth - dip_fwd * h)
            blocked |= near & inside
    img = np.where(blocked, 20, 235).astype(np.uint8)
    return Image.fromarray(img)


def main():
    if sys.argv[1] == "--grid":
        out = sys.argv[2]
        panels = []
        for group in sys.argv[3:]:
            tilt, depth, df, da, soft, az = (float(c) for c in group.split(","))
            panels.append(panel(tilt, depth, df, da, soft, az))
        sheet = Image.new("L", (len(panels) * 364, 364), 128)
        for i, p in enumerate(panels):
            sheet.paste(p, (i * 364 + 2, 2))
        sheet.save(out)
        return
    out, tilt, depth, df, da, soft = sys.argv[1], *(float(c) for c in sys.argv[2:7])
    panel(tilt, depth, df, da, soft).save(out)


if __name__ == "__main__":
    main()
