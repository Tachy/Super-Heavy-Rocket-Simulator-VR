"""Prints render_check.gd ground-view arguments for a camera D metres from the Pad B mount (the site origin).

    python tools/structures/measure/mount_view_args.py <distance_m> [eye_height_m] [aim_height_m] [camera_bearing_deg]

The camera lies on the bearing (default 135 = the front, away from the tower at 315) from the mount axis and looks at
the axis, aimed at `aim_height_m` above the ground. Small-offset conversion: fine for a few km.
"""
import math
import sys

SITE_LAT_DEG = 25.99686
SITE_LON_DEG = -97.15394
EARTH_RADIUS_M = 6378137.0


def main() -> None:
    dist = float(sys.argv[1])
    eye = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
    aim = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
    bearing = math.radians(float(sys.argv[4]) if len(sys.argv) > 4 else 135.0)
    east, north = dist * math.sin(bearing), dist * math.cos(bearing)
    lat = SITE_LAT_DEG + math.degrees(north / EARTH_RADIUS_M)
    lon = SITE_LON_DEG + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(SITE_LAT_DEG))))
    azimuth = (math.degrees(bearing) + 180.0) % 360.0
    elevation = math.degrees(math.atan2(aim - eye, dist))
    print(f"--lat={lat:.7f} --lon={lon:.7f} --eye_height={eye} --azimuth={azimuth:.2f} --elevation={elevation:.2f}")


if __name__ == "__main__":
    main()
