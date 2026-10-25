"""Calibration of the Moon shaders' regolith scale against Krisciunas & Schaefer 1991 (night-vision fix round, I3/I4).

The Moon shaders (game/shaders/moon_regolith.gdshaderinc) draw the regolith as
    L = albedo_texel x R x E / pi x LS(mu0, mu) x f(g),
LS = 2 mu0 / (mu0 + mu) (Lommel-Seeliger, 1 at mu0 = mu), E the illuminance of the light (the Sun or the earthshine),
g the local phase angle (light - surface - eye) and f(g) the phase correction that makes the disc-integrated
brightness follow K&S's phase law:
    f(g) = O(g) x 10^(-0.4 (0.026 g + 4e-9 g^4)) / Phi_LS(g),   O(g) = 1.35 - 0.05 g for g < 7 deg, else 1,
Phi_LS the disc-integrated phase function of a Lommel-Seeliger sphere (1 at g = 0). R is fixed so that the disc of the
texture's near side at full phase (g -> 0 without the opposition term) gives K&S's 0.313 lux at 384 400 km:
    R = E_KS(0) / (E_toa / pi x Omega_disc x A_near),
A_near the texture's near-side mean weighted by projected area (cos^2 lat cos lon), sRGB-decoded, BT.709.

This script checks the closed form of Phi_LS against a numerical disc integral, prints f(g) and computes A_near and R.
Usage: python tools/validation/moon_disc_calibration.py [--texture data/textures/moon_color.png] [--toa-lux N]
"""
import argparse
import math

import numpy as np
from PIL import Image

KS_FULL_MAG = -12.73
FC_OFFSET = 16.57
LUX_PER_FC = 10.763910
MOON_RADIUS_M = 1737400.0
MOON_MEAN_DISTANCE_M = 384400000.0


def phi_ls_closed(g):
    if g < 1e-9:
        return 1.0
    return 1.0 - math.sin(g / 2) * math.tan(g / 2) * math.log(1.0 / math.tan(g / 4))


def phi_ls_numeric(g, n=1500):
    # Observer along +z, light at phase g in the x-z plane; sphere points on the visible hemisphere.
    x = np.linspace(-1, 1, n)
    xx, yy = np.meshgrid(x, x)
    rr = xx * xx + yy * yy
    inside = rr < 1
    zz = np.sqrt(np.clip(1 - rr, 0, None))
    mu = zz
    mu0 = xx * math.sin(g) + zz * math.cos(g)
    ls = np.where(inside & (mu0 > 0), 2 * np.clip(mu0, 0, None) / (mu0 + mu + 1e-30), 0.0)
    return ls[inside].mean()  # disc mean of the luminance over the full-phase mean (1)


def ks_relative(g_deg):
    return 10 ** (-0.4 * (0.026 * g_deg + 4e-9 * g_deg ** 4))


def opposition(g_deg):
    return 1.35 - 0.05 * g_deg if g_deg < 7 else 1.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--texture", default="data/textures/moon_color.png")
    parser.add_argument("--toa-lux", type=float, default=None,
                        help="the tables' TOA illuminance at 1 AU (lux); default 1.3327e5 (pi x 4.242e4)")
    args = parser.parse_args()

    print("g deg   Phi_LS closed   numeric   K&S rel    f(g)")
    for g_deg in [0, 2, 5, 7, 10, 30, 60, 90, 120, 150, 170]:
        g = math.radians(g_deg)
        closed = phi_ls_closed(g)
        numeric = phi_ls_numeric(g)
        f = opposition(g_deg) * ks_relative(g_deg) / max(closed, 1e-30)
        print(f"{g_deg:5.0f}   {closed:12.5f}   {numeric:8.5f}   {ks_relative(g_deg):8.5f}   {f:8.4f}")

    image = np.asarray(Image.open(args.texture).convert("RGB"), dtype=np.float64) / 255.0
    linear = np.where(image <= 0.04045, image / 12.92, ((image + 0.055) / 1.055) ** 2.4)
    luminance = linear @ np.array([0.2126, 0.7152, 0.0722])
    h, w = luminance.shape
    v = (np.arange(h) + 0.5) / h
    u = (np.arange(w) + 0.5) / w
    lat = math.pi * (0.5 - v)[:, None]
    lon = (2 * math.pi * u - math.pi)[None, :]
    cos_view = np.cos(lat) * np.cos(lon)
    weight_near = np.where(cos_view > 0, cos_view * np.cos(lat), 0.0)
    weight_sphere = np.cos(lat) * np.ones_like(lon)
    a_near = (luminance * weight_near).sum() / weight_near.sum()
    a_sphere = (luminance * weight_sphere).sum() / weight_sphere.sum()
    print(f"texture mean: sphere {a_sphere:.4f}, near-side disc (projected) {a_near:.4f}")

    e_ks = 10 ** (-0.4 * (KS_FULL_MAG + FC_OFFSET)) * LUX_PER_FC
    omega = math.pi * (MOON_RADIUS_M / MOON_MEAN_DISTANCE_M) ** 2
    toa = args.toa_lux if args.toa_lux else math.pi * 4.242e4
    r = e_ks / (toa / math.pi * omega * a_near)
    print(f"K&S full Moon {e_ks:.4f} lux, disc {omega:.4e} sr -> mean disc luminance {e_ks / omega:.0f} cd/m^2")
    print(f"regolith scale R = {r:.4f} (the former shaders: 0.8 / 1.5 = {0.8 / 1.5:.4f}, x {0.8 / 1.5 / r:.3f})")
    print(f"equivalent geometric albedo R x A_near = {r * a_near:.4f}")


if __name__ == "__main__":
    main()
