"""Prints render_check.gd ground-view arguments for a camera D metres from the Pad B tower (placed at starbase_olp1).

    python tools/structures/measure/view_args.py <distance_m> [eye_height_m] [aim_height_m] [camera_azimuth_deg]

The camera lies on the bearing `camera_azimuth_deg` (default 135, clockwise from north) from the tower base and looks back
at the tower, aimed at `aim_height_m` above the base. Spherical small-offset conversion: fine for a few km.
"""
import math
import sys

SITE_LAT_DEG = 25.99686
SITE_LON_DEG = -97.15394
TOWER_OFFSET_M = 30.0        # tower centre from the site origin along the tower azimuth (config/launch_sites.toml)
TOWER_AZIMUTH_DEG = 315.0
EARTH_RADIUS_M = 6378137.0


def main() -> None:
    dist = float(sys.argv[1])
    eye = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
    aim = float(sys.argv[3]) if len(sys.argv) > 3 else 73.0
    bearing = math.radians(float(sys.argv[4]) if len(sys.argv) > 4 else 135.0)
    tz = math.radians(TOWER_AZIMUTH_DEG)
    east = TOWER_OFFSET_M * math.sin(tz) + dist * math.sin(bearing)
    north = TOWER_OFFSET_M * math.cos(tz) + dist * math.cos(bearing)
    lat = SITE_LAT_DEG + math.degrees(north / EARTH_RADIUS_M)
    lon = SITE_LON_DEG + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(SITE_LAT_DEG))))
    azimuth = (math.degrees(bearing) + 180.0) % 360.0
    elevation = math.degrees(math.atan2(aim - eye, dist))
    print(f"--lat={lat:.7f} --lon={lon:.7f} --eye_height={eye} --azimuth={azimuth:.2f} --elevation={elevation:.2f}")


if __name__ == "__main__":
    main()
