"""View tilt of a grid-fin image from its lattice homography (fin_fit.py output `<prefix>_h.npy`, lattice steps -> image).

The lattice is square, so the local Jacobian of lattice -> image at a point is s * (2x2 part of a rotation seen at the tilt):
its singular values are s and s cos(tilt) (affine / weak-perspective reading; the perspective of the homography enters
through the Jacobian at the chosen point). Prints the tilt of the face-normal from the view axis at the given points.
Usage: python fin_tilt.py <h.npy> <x,y> [<x,y> ...]
"""
import sys

import numpy as np


def jacobian_lattice_to_image(h, x, y):
    """h = lattice -> image (fin_lattice.fit); returns the 2x2 Jacobian at the lattice point under image pixel (x, y)."""
    q = np.linalg.inv(h) @ np.array([x, y, 1.0])
    lat = q[:2] / q[2]

    def img(u, v):
        r = h @ np.array([u, v, 1.0])
        return r[:2] / r[2]

    e = 1e-4
    ju = (img(lat[0] + e, lat[1]) - img(lat[0] - e, lat[1])) / (2 * e)
    jv = (img(lat[0], lat[1] + e) - img(lat[0], lat[1] - e)) / (2 * e)
    return np.stack([ju, jv], axis=1), lat, None


def main():
    h = np.load(sys.argv[1])
    for arg in sys.argv[2:]:
        x, y = (float(c) for c in arg.split(","))
        j, lat, _ = jacobian_lattice_to_image(h, x, y)
        s = np.linalg.svd(j, compute_uv=False)
        tilt = np.degrees(np.arccos(min(1.0, s[1] / s[0])))
        print(f"point {x:.0f},{y:.0f} lattice {lat.round(2)} step px {s[0]:.1f} / {s[1]:.1f} tilt {tilt:.1f} deg")


if __name__ == "__main__":
    main()
