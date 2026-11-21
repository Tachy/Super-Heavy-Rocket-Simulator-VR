"""Locker 5 door, its latch and the iPads' roll on the upper panel (fidelity plan Task 7): px read in discovery2 and discovery10 (2x crops with a
20 px grid, screenshots/m7-cockpit/fidelity/t7/work/d2_locker.png, d2_ul.png, d10_ul.png) back-projected through the fitted cameras
(config/cockpits/photo_cameras.toml) onto the door's plane (o = DOOR_O_M in front of the screen plane), console frame of cabin_fit/parts/keys.py.

Locker 5 is the crew's left one (lat < 0); the right one ("6") is behind the pilot's helmet in every frame: mirrored [E].
The door: a black plate with rounded corners; its lower-left corner is hidden behind a helmet rim in both frames, so the box is fixed by the
top-left, top-right and bottom-right corners (the bottom edge read along the placard's foot). The latch: the round recessed finger latch with
its ring, at the door's right (inner) edge above the placard.

Run: python -m cabin_fit.parts.lockers (from tools/structures) prints the door box, the latch and the config's values.
"""
from __future__ import annotations

import math

import numpy as np

from .. import annotations
from ..camera import Camera
from .keys import back_project

DOOR_O_M = 0.012  # [E] = crew_dragon.toml [console] locker_proud_m: the door face this far in front of the screen plane
READ_SIGMA_PX = 6.0  # [E] corner read on the blurred GoPro frames (rounded corners, ~4 px soft)
PLANE_SIGMA_M = 0.01  # [E] the door plane's o
# (u, v) px of the door corners, the latch centre and the placard box (tl, br) per photo
DOOR_PX = {
    "discovery2": {"tl": (275, 380), "tr": (632, 284), "br": (680, 668), "latch": (621, 470), "plac_tl": (508, 538), "plac_br": (682, 672)},
    "discovery10": {"tl": (280, 378), "tr": (638, 283), "br": (686, 668), "latch": (626, 468), "plac_tl": (512, 536), "plac_br": (688, 670)},
}


def measure():
    """{"door": (lat0, lat1, u0, u1), "latch": (lat, u), "placard": (lat0, lat1, u0, u1), "sigma": m} averaged over the photos."""
    rows = []
    sig = []
    for photo, px in DOOR_PX.items():
        cam = Camera.from_row(annotations.camera_row(photo))
        p = {k: back_project(cam, v, DOOR_O_M) for k, v in px.items()}
        lat0 = p["tl"][0]
        lat1 = 0.5 * (p["tr"][0] + p["br"][0])
        u1 = p["tr"][1]
        u0 = p["br"][1]
        rows.append((lat0, lat1, u0, u1, p["latch"][0], p["latch"][1], p["plac_tl"][0], p["plac_br"][0], p["plac_br"][1], p["plac_tl"][1]))
        s_read = abs(back_project(cam, (px["tr"][0] + READ_SIGMA_PX, px["tr"][1]), DOOR_O_M)[0] - p["tr"][0])
        s_plane = abs(back_project(cam, px["tr"], DOOR_O_M + PLANE_SIGMA_M)[0] - p["tr"][0])
        sig.append(math.hypot(s_read, s_plane))
    a = np.array(rows)
    m = a.mean(axis=0)
    spread = 0.5 * np.abs(a[0] - a[1])
    sigma = float(math.hypot(max(sig), float(spread[:4].max())))
    return {"door": tuple(float(v) for v in m[:4]), "latch": (float(m[4]), float(m[5])), "placard": tuple(float(v) for v in m[6:10]),
            "sigma": sigma, "photos": [tuple(float(v) for v in r) for r in rows]}


def main():
    res = measure()
    lat0, lat1, u0, u1 = res["door"]
    print(f"door lat {lat0:+.4f} .. {lat1:+.4f} (width {lat1 - lat0:.4f}), u {u0:+.4f} .. {u1:+.4f} (height {u1 - u0:.4f}), "
          f"centre ({0.5 * (lat0 + lat1):+.4f}, {0.5 * (u0 + u1):+.4f}) +- {res['sigma']:.4f}")
    print(f"latch ({res['latch'][0]:+.4f}, {res['latch'][1]:+.4f})  placard lat {res['placard'][0]:+.4f}..{res['placard'][1]:+.4f}, "
          f"u {res['placard'][2]:+.4f}..{res['placard'][3]:+.4f}")
    for r in res["photos"]:
        print("  per photo", " ".join(f"{v:+.4f}" for v in r))
    return res


if __name__ == "__main__":
    main()
