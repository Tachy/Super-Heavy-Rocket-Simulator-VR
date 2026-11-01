"""Clearance of aborted landings from the drawn pad structures.

Reads the CSVs of the hidden `[.abort_clearance]` cases (STARSIM_ABORT_TRACE_DIR) and measures the hull (axis + radius)
against what the physics does not collide with: the tower foot, the mount block and annex, the trench walls and ramps.
Numbers: config/launch_sites.toml (site frame, squares' bearing), config/structures/starbase_olp2.toml and
docs/subsystems/launch-structures.md (drawing). Usage: abort_clearance.py <trace dir>
"""

import csv
import glob
import math
import os
import sys

AZIMUTH_DEG = 349.0  # launch_sites.toml, starbase_pad2: pad centre -> tower centre
PLAN_YAW_DEG = 37.4  # squares' X axis against that line
SQUARES_BEARING_RAD = math.radians(AZIMUTH_DEG + PLAN_YAW_DEG)

# Boxes in the squares' axes: (name, x0, x1, y0, y1, top z). Heights above the pad ground.
BOXES = [
    ("tower foot", 17.6, 37.6, -9.0, 9.0, 7.0),  # launch-structures.md: X 17.6-37.6, 18 m across, 7 m walls
    ("mount block", -10.9, 10.3, -9.4, 9.4, 14.5),  # block walls, ring top
    ("trench wall +Y", -10.9, 10.3, 9.4, 41.5, 0.5),  # the 0.5 m small wall; higher near the block, see wall_top_z
    ("trench wall -Y", -10.9, 10.3, -41.5, -9.4, 0.5),
]
ANNEX = (-10.9, 22.3, -20.0, 9.2, 13.5)  # trapezoid: half width 22.3 at X -10.9, 9.2 at X -20; slopes taken as a prism
RAMP = (-7.6, 7.6, 9.4, 41.5)  # the open ramps (below ground) on both sides: being over them is reported
WALL_TOP_BLOCK_M = 13.5  # wall top at the block
WALL_END_Y_M = 22.3  # falls to the small wall here


def to_squares(east, north):
    x = east * math.sin(SQUARES_BEARING_RAD) + north * math.cos(SQUARES_BEARING_RAD)
    y = -east * math.cos(SQUARES_BEARING_RAD) + north * math.sin(SQUARES_BEARING_RAD)
    return x, y


def box_distance(px, py, pz, box):
    _, x0, x1, y0, y1, top = box
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    dz = max(pz - top, 0.0)  # the box stands on the ground
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def wall_distance(px, py, pz):
    """Trench side wall strips (X 7.6 to the deck edge on either side), top falling along |Y| from the block."""
    ay = abs(py)
    best = 1e12
    for x0, x1 in ((-10.9, -7.6), (7.6, 10.3)):
        for i in range(0, 33):
            y = 9.4 + (41.5 - 9.4) * i / 32.0
            f = min(max((y - 9.4) / (WALL_END_Y_M - 9.4), 0.0), 1.0)
            top = WALL_TOP_BLOCK_M + (0.5 - WALL_TOP_BLOCK_M) * f
            dx = max(x0 - px, 0.0, px - x1)
            dz = max(pz - top, 0.0)
            best = min(best, math.sqrt(dx * dx + (ay - y) ** 2 + dz * dz))
    return best


def annex_distance(px, py, pz):
    x_near, half_near, x_far, half_far, top = ANNEX
    best = 1e12
    for i in range(0, 21):
        x = x_near + (x_far - x_near) * i / 20.0
        half = half_near + (half_far - half_near) * i / 20.0
        dy = max(abs(py) - half, 0.0)
        dz = max(pz - top, 0.0)
        best = min(best, math.sqrt((px - x) ** 2 + dy * dy + dz * dz))
    return best


def hull_points(row):
    bottom = [float(row[k]) for k in ("bx", "by", "bz")]
    top = [float(row[k]) for k in ("tx", "ty", "tz")]
    length = math.dist(bottom, top)
    n = max(int(length), 1)
    for i in range(n + 1):
        f = i / n
        yield [bottom[j] + (top[j] - bottom[j]) * f for j in range(3)]


def measure(path):
    with open(path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    least = {}
    def note(name, value, t):
        if name not in least or value < least[name][0]:
            least[name] = (value, t)
    for row in rows:
        t, radius = float(row["t_s"]), float(row["radius_m"])
        note("tower box (physics)", float(row["tower_m"]), t)
        note("arm +", float(row["arm1_m"]), t)
        note("arm -", float(row["arm2_m"]), t)
        for e, n, u in hull_points(row):
            x, y = to_squares(e, n)
            note("tower foot", box_distance(x, y, u, BOXES[0]) - radius, t)
            note("mount block", box_distance(x, y, u, BOXES[1]) - radius, t)
            note("mount annex", annex_distance(x, y, u) - radius, t)
            note("trench walls", wall_distance(x, y, u) - radius, t)
            r = math.hypot(e, n)
            mount_r = 13.0  # physics annulus, launch_sites.toml mount_outer_radius_m, top 14.5 m
            note("mount annulus (physics)", math.hypot(max(r - mount_r, 0.0), max(u - 14.5, 0.0)) - radius, t)
    last = rows[-1]
    be, bn, bu = float(last["bx"]), float(last["by"]), float(last["bz"])
    x, y = to_squares(be, bn)
    over_ramp = RAMP[0] - 4.5 <= x <= RAMP[1] + 4.5 and RAMP[2] - 4.5 <= abs(y) <= RAMP[3] + 4.5
    print(f"{os.path.basename(path)}: end bottom E {be:.1f} N {bn:.1f} up {bu:.1f} (squares X {x:.1f} Y {y:.1f}), "
          f"{math.hypot(be, bn):.1f} m from the pad centre, footprint over a ramp: {over_ramp}, phase {last['phase']}")
    for name, (value, t) in sorted(least.items(), key=lambda kv: kv[1][0]):
        print(f"    {name:26s} {value:9.2f} m at t {t:.2f} s")


if __name__ == "__main__":
    for path in sorted(glob.glob(os.path.join(sys.argv[1], "*.csv"))):
        measure(path)
