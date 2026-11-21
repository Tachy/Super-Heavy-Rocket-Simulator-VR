"""Key layout on the button tray (fidelity plan Task 5): key centres read in discovery10 back-projected through its fitted camera
(config/cockpits/photo_cameras.toml) onto the key-face plane (o = KEY_FACE_O_M in front of the screen plane), in the console frame of
cabin_fit/parts/console.py (lat to the crew's right, u up from the centre screen centre).

Single view: the depth along the ray comes from the plane, so lat / u carry the camera sigma (discovery10 fit: position ~1-2 cm, angles
~0.3-0.8 deg -> ~5 mm at 0.65 m) plus the read sigma (KEY_READ_SIGMA_PX; the labels on 1.15-1.3x tiles, the key centre taken at the
label / symbol centre) plus the plane's o uncertainty (+-5 mm -> lat shifts by o * tan(ray angle)).

Run: python -m cabin_fit.parts.keys (from tools/structures) prints lat / u per key and the config's values.
"""
from __future__ import annotations

import math
import tomllib
from pathlib import Path

import numpy as np

from .. import annotations
from ..camera import Camera, unproject

KEY_FACE_O_M = 0.030  # [E] key faces 3.0 cm in front of the screen plane (crew_dragon.toml [[button]] centre_m: 0.029-0.030 along the normal)
KEY_READ_SIGMA_PX = 8.0  # [E] key centre read on the blurred GoPro frame (label / symbol centre)
CENTRE = np.array([2.512, -0.043, 0.0])  # the centre screen centre (crew_dragon.toml [[screen]] centre)
TILT_DEG = 45.0

# discovery10 px of the key centres (u, v), read on screenshots/m7-cockpit/fidelity/t5/work/k_*.png / d10_tray_*.png
KEYS_PX = {
    "cam1": (642, 1500), "cam2": (692, 1490), "cam3": (739, 1490), "cam4": (794, 1490), "cam5": (849, 1500),
    "cam11": (709, 1545), "cam12": (764, 1545), "cam13": (819, 1545),
    "cam6": (927, 1495), "cam7": (989, 1495), "cam8": (1052, 1495), "cam9": (1114, 1495), "cam10": (1179, 1495),
    "cam14": (947, 1555), "cam15": (1009, 1555), "cam16": (1069, 1555), "cam17": (1132, 1555), "cam18": (1202, 1555),
    "c_upper_1": (1276, 1495), "c_upper_2": (1344, 1497), "c_mains_only": (1412, 1500), "c_drogues_mains": (1499, 1500),
    "c_lower_1": (1300, 1565), "c_lower_2": (1364, 1568), "c_lower_round": (1594, 1600),
    "r_backup_g_meter": (2460, 1540), "r_led_1": (2556, 1516), "r_led_2": (2632, 1512), "r_led_3": (2708, 1512),
    "r_cover": (3005, 1490),
    "r_upper_1": (3218, 1412), "r_upper_2": (3273, 1410), "r_upper_3": (3330, 1408), "r_upper_4": (3388, 1408), "r_upper_5": (3442, 1381),
    "r_lower_1": (3261, 1465), "r_lower_2": (3315, 1465), "r_lower_3": (3372, 1462),
}


def frame():
    t = math.radians(TILT_DEG)
    up = np.array([math.cos(t), -math.sin(t), 0.0])
    n = np.array([-math.sin(t), -math.cos(t), 0.0])
    right = np.array([0.0, 0.0, -1.0])
    return up, n, right


def back_project(cam: Camera, px, o_m=KEY_FACE_O_M):
    """(lat, u) of the point where the ray through px meets the plane o = o_m."""
    up, n, right = frame()
    d = unproject(np.asarray([px], float), cam)[0]
    p0 = CENTRE + n * o_m
    t = np.dot(p0 - cam.position_m, n) / np.dot(d, n)
    p = cam.position_m + d * t
    return float(np.dot(p - CENTRE, right)), float(np.dot(p - CENTRE, up))


def measure(photo="discovery10"):
    cam = Camera.from_row(annotations.camera_row(photo))
    out = {}
    for key, px in KEYS_PX.items():
        lat, u = back_project(cam, px)
        # read sigma: shift by the read sigma in u / v; plane sigma: o +- 5 mm
        du = [back_project(cam, (px[0] + s, px[1]))[0] - lat for s in (KEY_READ_SIGMA_PX,)]
        do = back_project(cam, px, KEY_FACE_O_M + 0.005)[0] - lat
        out[key] = (lat, u, math.hypot(du[0], do))
    return out


def main():
    cfg = tomllib.loads((Path(annotations.CAMERA_FILE).parent / "crew_dragon.toml").read_text(encoding="utf-8"))
    up, n, right = frame()
    for key, (lat, u, s) in measure().items():
        b = next(b for b in cfg["button"] if b["id"] == key)
        c = np.asarray(b["centre_m"]) - CENTRE
        print(f"{key:18s} lat {lat:+.4f} +- {s:.4f} (cfg {np.dot(c, right):+.4f})  u {u:+.4f} (cfg {np.dot(c, up):+.4f})")


if __name__ == "__main__":
    main()
