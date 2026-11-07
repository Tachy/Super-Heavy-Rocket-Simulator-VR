"""Crew Dragon cabin: gen_dragon_cabin.py -- --spec <json> --out <dir> writes dragon_cabin.gltf.

Own rebuild from config/cockpits/crew_dragon.toml (spec["cockpit"], validated by vehicle_spec.validate_cockpit); no third-party mesh.
Frame = the capsule stage body frame of gen_dragon.py / dragon_capsule.gltf (X station from the heat shield apex, nose +X, (Y, Z) = r (cos az, sin az),
Z = az +90 = the crew's left). Silhouette only: panel lines, labels and placards come later by texture.

Nodes: `cabin_shell` (inner wall of revolution with the ceiling ring, the forward tunnel and the top plate; normals into the cabin; cut at the window
reveals and the hatch), `floor`, `hatch_frame` (white trim, the recessed hatch inner face and its reveal), `window_<i>` (funnel from the round interior
reveal to the exterior pane, bezel ring; i = exterior table row, +az then -az) with `window_glass_<i>`, `seat_shell|pad|rest_<s1..s4>` (white shell, black pads, black leg-rest rails
+ foot plate at the seat's foot_rest_m), `console_housing` (dark: one straight box across the three screens + the dark upper panel up to the ceiling), `console_back`
(light grey back, underside, two hand-hold posts), `console_shelf` (the button shelf as separate light grey modules, cabin_layout.shelf_modules), `console_lockers` (5 / 6),
`ceiling_ring_light` (emissive annulus), `screen_left|centre|right` (one quad, UV 0..1 over the active area,
(0, 0) top left, +normal toward the crew), `tablet_l|r`, `button_<id>` (one node per button; the centre handle is the node `handle`), `light_<i>`
([[light]] rows in file order), `hotas_stick`, `hotas_throttle`.
The cabin layout helpers (apertures, node names, manifest block) live in cabin_layout.py; the wall is the exterior meridian offset along its normal.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import cabin_layout as cl  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

FINE_ROWS_X_M = (1.6, 3.1)  # [E] the stations the hatch (1.70-2.99) and the windows' reveals (about 1.8-2.9) cover: fine rows there
FINE_ROW_M, COARSE_ROW_M = 0.02, 0.06  # [E] row pitch of the wall: under the bezel width (0.06) so the cut edge hides under it
FINE_COL_DEG, COARSE_COL_DEG = 0.75, 3.0  # [E] column pitch around the apertures (about 0.02 m at r 1.45) and elsewhere
FINE_MARGIN_DEG = 15.0  # [E] fine columns this far past a window's centre (reveal 0.24 + bezel 0.06 at r 1.43 is 12 deg)
HATCH_FINE_MARGIN_DEG = 8.0  # [E] fine columns this far past the hatch's half angle
CUT_MARGIN_M = 0.012  # [E] cells whose centre lies within the aperture + this are removed: the remainder never pokes into the opening
RING_POINTS = 32  # [E] points around a window ring
HATCH_POINTS = 64  # [E] points around the hatch outline
HATCH_RINGS = 6  # [E] rings of the hatch's inner face (it follows the wall's curvature)
SCREEN_PROUD_M = 0.002  # [E] the housing's front lies this far behind the screens (the screen quads sit exactly on the TOML plane)
BUTTON_PROUD_M = 0.006  # [E] push button height over the shelf
COVER_PROUD_M = 0.012  # [E] hinged cover height
SHELF_TOP_GAP_M = 0.0175  # [E] the shelf's top edge lies this far under a screen's lower edge
TABLET_THICKNESS_M = 0.008  # [E] iPad mini 6.1 mm [S] plus the case
TABLET_POST_M = 0.012  # [E] mount post radius
STRUT_RADIUS_M = 0.02  # [E] console strut
SEAT_BACK_HALF_M = (0.05, 0.40)  # [E] seat back half thickness / half height (a 0.8 m back, crew1_1 / Wikimedia)
SEAT_THIGH_M, SEAT_SHIN_M = 0.44, 0.40  # [E] pan length / lower-leg support length (seated adult, 5th percentile hip to knee 0.5)
SIDES = 16  # [E] sides of round parts
FOOT_PLATE_M = (0.32, 0.34, 0.02)  # [E] foot frame: length along the foot, width across, (thickness unused) (crew1_1: a plate-wide carbon frame under both boots)
FOOT_FRAME_TUBE_M = 0.012  # [E] carbon tube radius of the foot frame
FOOT_RAIL_Z_M = 0.17  # [E] leg-rest rails stand this far either side of the seat centre (outside the shins: hip joint 0.088 + boot half width 0.05)
FOOT_RAIL_RADIUS_M = 0.014  # [E] carbon rail radius (crew1_1 frame tubes ~30 mm)
LIGHT_PROUD_M = 0.006  # [E] lamp panel height over the console front
BACK_PLATE_M = 0.003  # [E] the light grey skin of the console's back / underside, outside the dark housing
RING_LIFT_M = 0.004  # [E] the ring light stands this far under the ceiling plane
RING_STEP_DEG = 3.0  # [E] segments of the ring light

V = Vector


def smoothstep_sign(c, power):
    return math.copysign(abs(c) ** (2.0 / power), c)


def seat_axes(back_deg):
    """(n, b, z): n = chest normal, b = up the seat back (35 deg from +X toward -Y in the config), z = capsule Z."""
    a = math.radians(back_deg)
    return V((math.sin(a), math.cos(a), 0.0)), V((math.cos(a), -math.sin(a), 0.0)), V((0.0, 0.0, 1.0))


# ---------- the shell ----------

def refine(p0, p1, step):
    n = max(1, int(math.ceil(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / step)))
    return [(p0[0] + (p1[0] - p0[0]) * k / n, p0[1] + (p1[1] - p0[1]) * k / n) for k in range(n)]


def wall_step(x):
    return FINE_ROW_M if FINE_ROWS_X_M[0] <= x <= FINE_ROWS_X_M[1] else COARSE_ROW_M


def column_angles(cabin):
    """Azimuths (deg, 0..360, ascending, without the closing 360): fine around the windows and the hatch, coarse elsewhere."""
    spans = []
    h_az, _, _, h_haz = cabin["hatch"]
    spans.append((h_az - h_haz - HATCH_FINE_MARGIN_DEG, h_az + h_haz + HATCH_FINE_MARGIN_DEG))
    for w in cl.window_apertures(cabin):
        spans.append((w["az_deg"] - FINE_MARGIN_DEG, w["az_deg"] + FINE_MARGIN_DEG))
    angles = {round(a * COARSE_COL_DEG, 6) for a in range(int(360.0 / COARSE_COL_DEG))}
    for lo, hi in spans:
        k = math.ceil(lo / FINE_COL_DEG)
        while k * FINE_COL_DEG <= hi:
            angles.add(round((k * FINE_COL_DEG) % 360.0, 6))
            k += 1
    return sorted(a for a in angles if a < 360.0)


class Cut:
    """Which wall cells the apertures remove (cell centre inside the opening plus a margin)."""

    def __init__(self, cabin, profile):
        self.cabin, self.profile = cabin, profile
        self.windows = [(cl.at_depth(profile, w["x_m"], w["az_deg"], cabin["wall_m"])) for w in cl.window_apertures(cabin)]
        self.reveal = cabin["window_reveal_radius_m"] + CUT_MARGIN_M

    def __call__(self, p):
        for c in self.windows:
            if (p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2 + (p[2] - c[2]) ** 2 < self.reveal ** 2:
                return True
        r = math.hypot(p[1], p[2])
        az = math.degrees(math.atan2(p[2], p[1]))
        h_az, h_x, hx, h_haz = self.cabin["hatch"]
        x_o = cl.to_outer_chart(self.profile, p[0], r, self.cabin["wall_m"])
        daz = (az - h_az + 180.0) % 360.0 - 180.0
        return (abs(x_o - h_x) / (hx + CUT_MARGIN_M)) ** cl.HATCH_POWER + (abs(daz) / (h_haz + math.degrees(CUT_MARGIN_M / r))) ** cl.HATCH_POWER < 1.0


def lathe(mesh, run, hint, angles, group, cut=None):
    """Surface of revolution of the polyline `run` [(x, r)...] over the column angles; `hint`: "in" (toward the axis) or an axial vector."""
    m = len(angles)
    for k in range(len(run) - 1):
        (x0, r0), (x1, r1) = run[k], run[k + 1]
        for j in range(m):
            a0, a1 = math.radians(angles[j]), math.radians(angles[(j + 1) % m] + (360.0 if j == m - 1 else 0.0))
            pts = [vc.pt(x0, a0, r0), vc.pt(x0, a1, r0), vc.pt(x1, a1, r1), vc.pt(x1, a0, r1)]
            if cut is not None and cut(sum(pts, V()) / 4.0):
                continue
            uvs = [(a0 * 1.5, x0), (a1 * 1.5, x0), (a1 * 1.5, x1), (a0 * 1.5, x1)]
            out = -vc.radial_dir(0.5 * (a0 + a1)) if hint == "in" else V(hint)
            mesh.face(pts, uvs, out, group)


def build_shell(cabin, profile, angles):
    prof = cabin["profile_m"]
    cut = Cut(cabin, profile)
    # profile_m: floor centre -> floor rim -> side wall up to the ceiling ring -> ceiling -> tunnel wall -> top plate
    i_rim = 1
    i_ceiling = next(i for i, (x, _) in enumerate(prof) if i > 1 and abs(x - cabin["ceiling_x_m"]) < 1e-9 and abs(prof[i + 1][0] - x) < 1e-9)
    rings = int(math.ceil(prof[1][1] / 0.2))
    floor_run = [(prof[0][0], prof[1][1] * k / rings) for k in range(rings + 1)]
    wall = []
    for a, b in zip(prof[i_rim:i_ceiling], prof[i_rim + 1:i_ceiling + 1]):
        wall += refine(a, b, wall_step(0.5 * (a[0] + b[0])))
    wall.append(prof[i_ceiling])
    ceiling = refine(prof[i_ceiling], prof[i_ceiling + 1], 0.1) + [prof[i_ceiling + 1]]
    tunnel = refine(prof[i_ceiling + 1], prof[i_ceiling + 2], 0.1) + [prof[i_ceiling + 2]]
    top = refine(prof[i_ceiling + 2], prof[i_ceiling + 3], 0.1) + [prof[i_ceiling + 3]]
    shell, floor = vc.VMesh(), vc.VMesh()
    lathe(floor, floor_run, (1.0, 0.0, 0.0), angles, ("floor", 0))
    lathe(shell, wall, "in", angles, ("shell", 0), cut)
    lathe(shell, ceiling, (-1.0, 0.0, 0.0), angles, ("shell", 1))
    lathe(shell, tunnel, "in", angles, ("shell", 2))
    lathe(shell, top, (-1.0, 0.0, 0.0), angles, ("shell", 3))
    return shell, floor


# ---------- windows and hatch ----------

def inward(profile, x_o, az_deg):
    """Unit vector into the cabin at the outer chart position (the reverse of the exterior normal)."""
    nx, nr = cl.ext_normal(profile, x_o)
    a = math.radians(az_deg)
    return V((-nx, -nr * math.cos(a), -nr * math.sin(a)))


def strip(mesh, ring_a, ring_b, hint_fn, group, closed=True):
    n = len(ring_a)
    for i in range(n if closed else n - 1):
        j = (i + 1) % n
        pts = [ring_a[i], ring_a[j], ring_b[j], ring_b[i]]
        centre = sum(pts, V()) / 4.0
        mesh.face(pts, [(0, 0), (1, 0), (1, 1), (0, 1)], hint_fn(centre), group)


def window_rings(cabin, profile, ap):
    """(reveal ring at the wall, glass ring under the exterior surface, centre point on the wall) for window `ap`."""
    wall = cabin["wall_m"]
    x_c, az_c = ap["x_m"], ap["az_deg"]
    nx, nr = cl.ext_normal(profile, x_c)
    r_in = cl.lerp(profile, x_c) - wall * nr

    def reveal(rho, depth):
        pts = []
        for i in range(RING_POINTS):
            phi = 2.0 * math.pi * i / RING_POINTS
            x_o = x_c + rho * math.cos(phi) * nr  # arc along the meridian -> station
            az = az_c + math.degrees(rho * math.sin(phi) / r_in)
            pts.append(V(cl.at_depth(profile, x_o, az, depth)))
        return pts

    glass = [V(cl.at_depth(profile, x_c + ap["half_x_m"] * math.cos(2.0 * math.pi * i / RING_POINTS), az_c + ap["half_az_deg"] * math.sin(2.0 * math.pi * i / RING_POINTS), cl.GLASS_DEPTH_M))
             for i in range(RING_POINTS)]
    return reveal, glass, V(cl.at_depth(profile, x_c, az_c, wall))


def build_window(cabin, profile, ap):
    wall, rho0 = cabin["wall_m"], cabin["window_reveal_radius_m"]
    reveal, glass, centre = window_rings(cabin, profile, ap)
    into = inward(profile, ap["x_m"], ap["az_deg"])
    well, pane = vc.VMesh(), vc.VMesh()
    ring_wall = reveal(rho0, wall)
    ring_a_top, ring_b_top = reveal(rho0, wall + cl.LIFT_M), reveal(rho0 + cabin["window_bezel_m"], wall + cl.LIFT_M)
    ring_b_base = reveal(rho0 + cabin["window_bezel_m"], wall)
    axis_end = sum(glass, V()) / len(glass)
    strip(well, ring_wall, glass, lambda c: axis_end - c, ("well", ap["index"]))  # the funnel: lit side faces the window axis
    strip(well, ring_a_top, ring_b_top, lambda c: into, ("bezel", ap["index"], 0))
    strip(well, ring_b_top, ring_b_base, lambda c: c - centre, ("bezel", ap["index"], 1))
    strip(well, ring_a_top, ring_wall, lambda c: centre - c, ("bezel", ap["index"], 2))
    for i in range(RING_POINTS):  # the glass: a fan on its ring, facing the cabin
        j = (i + 1) % RING_POINTS
        pane.face([axis_end, glass[i], glass[j]], [(0.5, 0.5), (0, 0), (1, 0)], into, ("glass", ap["index"]))
    return well, pane


def hatch_outline(cabin, profile, add_m, depth, scale=1.0):
    h_az, h_x, hx, h_haz = cabin["hatch"]
    nr = cl.ext_normal(profile, h_x)[1]
    r_in = cl.lerp(profile, h_x) - cabin["wall_m"] * nr
    pts = []
    for i in range(HATCH_POINTS):
        phi = 2.0 * math.pi * i / HATCH_POINTS
        sx, sy = smoothstep_sign(math.cos(phi), cl.HATCH_POWER), smoothstep_sign(math.sin(phi), cl.HATCH_POWER)
        pts.append(V(cl.at_depth(profile, h_x + scale * (hx + add_m) * sx, h_az + scale * (h_haz + math.degrees(add_m / r_in)) * sy, depth)))
    return pts


def build_hatch(cabin, profile):
    wall, trim = cabin["wall_m"], cabin["hatch_trim_m"]
    h_az, h_x, hx, h_haz = cabin["hatch"]
    mesh = vc.VMesh()
    into = inward(profile, h_x, h_az)
    centre_pt = V(cl.at_depth(profile, h_x, h_az, wall))
    opening_wall = hatch_outline(cabin, profile, 0.0, wall)
    opening_door = hatch_outline(cabin, profile, 0.0, wall - cl.HATCH_RECESS_M)
    door_centre = V(cl.at_depth(profile, h_x, h_az, wall - cl.HATCH_RECESS_M))
    strip(mesh, opening_wall, opening_door, lambda c: door_centre - c, "hatch_reveal")
    rings = [[door_centre] * HATCH_POINTS] + [hatch_outline(cabin, profile, 0.0, wall - cl.HATCH_RECESS_M, k / HATCH_RINGS) for k in range(1, HATCH_RINGS + 1)]
    for a, b in zip(rings, rings[1:]):
        strip(mesh, a, b, lambda c: into, "hatch_door")
    top_a, top_b = hatch_outline(cabin, profile, 0.0, wall + cl.LIFT_M), hatch_outline(cabin, profile, trim, wall + cl.LIFT_M)
    base_b = hatch_outline(cabin, profile, trim, wall)
    strip(mesh, top_a, top_b, lambda c: into, ("hatch_trim", 0))
    strip(mesh, top_b, base_b, lambda c: c - centre_pt, ("hatch_trim", 1))
    strip(mesh, top_a, opening_wall, lambda c: centre_pt - c, ("hatch_trim", 2))
    return mesh


# ---------- seats, console, props ----------

def screens_by_id(cockpit):
    return {s["id"]: s for s in cockpit["screen"]}


class Panel:
    """A screen's frame: origin at the screen centre, axes n (toward the crew), up, right (as the crew sees it)."""

    def __init__(self, s):
        self.n, self.up, self.right = V(s["normal"]), V(s["up"]), V(cl.screen_right(s))
        self.o = V(s["centre_m"])
        self.w, self.h = s["size_m"]

    def at(self, lat, up, off):
        return self.o + self.right * lat + self.up * up + self.n * off

    def box(self, mesh, lat0, lat1, up0, up1, off_front, off_back):
        centre = self.at(0.5 * (lat0 + lat1), 0.5 * (up0 + up1), 0.5 * (off_front + off_back))
        vc.box(mesh, centre, self.n, self.up, self.right, (0.5 * abs(off_front - off_back), 0.5 * (up1 - up0), 0.5 * (lat1 - lat0)))


def build_seat(seat, floor_x):
    """(shell, pad, rest): white shell, black pads, black carbon leg-rest frame + foot plate (crew1_1: white shells, black pads, carbon frames)."""
    n, b, z = seat_axes(seat["back_angle_deg"])
    hip, w = V(seat["hip_m"]), seat["width_m"]
    shell, pad, rest = vc.VMesh(), vc.VMesh(), vc.VMesh()
    half_back = SEAT_BACK_HALF_M[1]
    # back: the shell behind (front face 0.035 behind the hip plane), the pad on its front (front face 0.01 behind the hip point, as before)
    vc.box(shell, hip + b * half_back - n * 0.0725, n, b, z, (0.0375, half_back, 0.5 * w))
    for side in (-1.0, 1.0):  # side bolsters of the shell
        vc.box(shell, hip + b * 0.30 - n * 0.0075 + z * (side * (0.5 * w - 0.02)), n, b, z, (0.0275, 0.26, 0.02))
    vc.box(pad, hip + b * 0.36 - n * 0.0225, n, b, z, (0.0125, 0.30, 0.5 * w * 0.78))
    vc.box(shell, hip + b * (2.0 * half_back + 0.06) - n * 0.06, n, b, z, (0.03, 0.09, 0.15))  # headrest shell
    vc.box(pad, hip + b * (2.0 * half_back + 0.06) - n * 0.02, n, b, z, (0.01, 0.08, 0.13))  # headrest pad
    # pan: shell below (b -0.09 .. -0.04), pad on top (-0.04 .. -0.01)
    vc.box(shell, hip + n * 0.5 * SEAT_THIGH_M - b * 0.065, n, b, z, (0.5 * SEAT_THIGH_M, 0.025, 0.5 * w * 0.95))
    vc.box(pad, hip + n * 0.5 * SEAT_THIGH_M - b * 0.025, n, b, z, (0.5 * SEAT_THIGH_M * 0.95, 0.015, 0.5 * w * 0.78))
    for side in (-1.0, 1.0):  # armrests (black)
        vc.box(rest, hip + n * 0.2 + b * 0.2 + z * (side * (0.5 * w + 0.03)), n, b, z, (0.2, 0.025, 0.03))
    # leg rest: two carbon rails from the pan's front end to the foot plate, the plate under the boots (foot_rest_m / foot_rest_normal)
    plate_n, plate_c = V(seat["foot_rest_normal"]), V(seat["foot_rest_m"])
    plate_f = z.cross(plate_n)  # along the foot
    # the foot plate is a carbon tube frame (outline + one cross bar) whose front face lies in the plane through foot_rest_m: the boots stay visible through it
    half_l, half_w, tube_r = 0.5 * FOOT_PLATE_M[0], 0.5 * FOOT_PLATE_M[1], FOOT_FRAME_TUBE_M
    corner = lambda a, c: plate_c - plate_n * tube_r + plate_f * (a * half_l) + z * (c * half_w)
    for (a0, c0), (a1, c1) in (((-1, -1), (1, -1)), ((1, -1), (1, 1)), ((1, 1), (-1, 1)), ((-1, 1), (-1, -1)), ((0, -1), (0, 1))):
        vc.tube(rest, corner(a0, c0), corner(a1, c1), tube_r, 10, ("foot_frame", seat["id"], a0, c0, a1, c1))
    pan_end = hip + n * SEAT_THIGH_M - b * 0.065
    for side in (-1.0, 1.0):
        start = pan_end + z * (side * FOOT_RAIL_Z_M)
        end = plate_c - plate_f * half_l - plate_n * tube_r
        end.z = hip.z + side * FOOT_RAIL_Z_M
        vc.tube(rest, start, end, FOOT_RAIL_RADIUS_M, SIDES, ("rail", seat["id"], side))
    return shell, pad, rest


def shelf_front_off(cockpit, cen):
    """The shelf modules' front plane along the console normal from the centre screen: the buttons' faces minus their height."""
    return min((V(b["centre_m"]) - cen.o).dot(cen.n) for b in cockpit["button"] if b["kind"] != "handle") - BUTTON_PROUD_M


def build_console(cockpit):
    """(housing, back, shelf, lockers): one straight dark housing and dark upper panel across all three screens (fold_deg 0, user decision 2026-11-07),
    light grey back / underside / posts, the shelf as separate light grey modules, the two lockers 5 / 6."""
    con = cockpit["console"]
    if con["fold_deg"] != 0.0:
        raise ValueError("console: only the straight console (fold_deg 0) is built")
    cen = Panel(screens_by_id(cockpit)["centre"])
    shift = (cen.o - V(con["centre_m"])).dot(cen.up)  # the screens' centre stands this far above the housing's front-plane centre
    half_w, depth = con["width_m"] / 2.0, con["depth_m"]
    u0, u1 = -shift - con["height_m"] / 2.0, -shift + con["height_m"] / 2.0
    u_top = (con["upper_top_x_m"] - cen.o.x) / cen.up.x  # the dark upper panel's front top edge reaches this station
    up_depth = con["upper_depth_m"]
    housing, back, shelf, lockers = vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh()
    cen.box(housing, -half_w, half_w, u0, u1, -SCREEN_PROUD_M, -depth)
    cen.box(housing, -half_w, half_w, u1, u_top, -SCREEN_PROUD_M, -up_depth)
    # light grey (silver) back, underside and the housing's top behind the upper panel
    cen.box(back, -half_w, half_w, u0, u1, -depth, -depth - BACK_PLATE_M)
    cen.box(back, -half_w, half_w, u1, u_top, -up_depth, -up_depth - BACK_PLATE_M)
    cen.box(back, -half_w, half_w, u0 - BACK_PLATE_M, u0, -SCREEN_PROUD_M, -depth - BACK_PLATE_M)
    cen.box(back, -half_w, half_w, u1, u1 + BACK_PLATE_M, -up_depth, -depth)
    for z_strut in con["strut_z_m"]:  # the hand-hold posts: from the upper panel's top edge to the ceiling
        lat = (z_strut - cen.o.z) / cen.right.z
        top = cen.at(lat, u_top, -up_depth / 2.0)
        vc.tube(back, top - V((0.01, 0.0, 0.0)), V((cockpit["cabin"]["ceiling_x_m"], top.y, top.z)), STRUT_RADIUS_M, SIDES, ("strut", z_strut))
    # the shelf: separate modules (clusters of buttons), dark housing showing in the gaps; the ledge stands proud of the screens
    front = shelf_front_off(cockpit, cen)
    s1 = -cen.h / 2.0 - con["shelf_top_gap_m"]
    for lat0, lat1 in cl.shelf_modules(cockpit):
        cen.box(shelf, lat0, lat1, s1 - con["shelf_height_m"], s1, front, -SCREEN_PROUD_M)
    for lk in cockpit["locker"]:  # dark lockers with the placards 5 / 6 (the placard: texture)
        c = V(lk["centre_m"]) - cen.o
        cen.box(lockers, c.dot(cen.right) - lk["size_m"][0] / 2.0, c.dot(cen.right) + lk["size_m"][0] / 2.0,
                c.dot(cen.up) - lk["size_m"][1] / 2.0, c.dot(cen.up) + lk["size_m"][1] / 2.0, con["locker_proud_m"], -SCREEN_PROUD_M)
    return housing, back, shelf, lockers


def build_ring_light(cabin):
    """The ceiling ring light: an emissive annulus around the forward tunnel, facing into the cabin (crew1_1)."""
    r0, r1 = cabin["ring_light_radii_m"]
    mesh = vc.VMesh()
    x = cabin["ceiling_x_m"] - RING_LIFT_M
    lathe(mesh, [(x, r0), (x, r1)], (-1.0, 0.0, 0.0), [k * RING_STEP_DEG for k in range(int(360.0 / RING_STEP_DEG))], ("ring", 0))
    return mesh


def screen_mesh(s):
    p = Panel(s)
    w, h = s["size_m"]
    corners = [p.at(-w / 2, -h / 2, 0.0), p.at(w / 2, -h / 2, 0.0), p.at(w / 2, h / 2, 0.0), p.at(-w / 2, h / 2, 0.0)]
    mesh = vc.VMesh()
    mesh.face(corners, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)], p.n, None)
    return mesh


def slab(mesh, centre, n, up, right, w, h, t):
    """Thin plate: front face (toward +n) has UV 0..1 over w x h, centred on `centre`."""
    c = V(centre)
    corners = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    front = [c + n * (t / 2) + right * (sa * w / 2) + up * (sb * h / 2) for sa, sb in corners]
    back = [c - n * (t / 2) + right * (sa * w / 2) + up * (sb * h / 2) for sa, sb in corners]
    mesh.face(front, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)], n)
    mesh.face(back, [(0, 0), (1, 0), (1, 1), (0, 1)], -n)
    for i in range(4):
        j = (i + 1) % 4
        mesh.face([front[i], front[j], back[j], back[i]], [(0, 0), (1, 0), (1, 1), (0, 1)], (front[i] + front[j]) / 2.0 - c)


def build_tablet(t):
    n, up = V(t["normal"]), V(t["up"])
    right = V(cl.screen_right(t))
    mesh = vc.VMesh()
    slab(mesh, t["centre_m"], n, up, right, t["size_m"][0], t["size_m"][1], TABLET_THICKNESS_M)
    c = V(t["centre_m"])
    vc.tube(mesh, c - n * (TABLET_THICKNESS_M / 2), c - n * 0.06, TABLET_POST_M, SIDES, ("tablet_post", t["id"]), cap0=False)
    return mesh


def build_light(light, cen):
    """A lamp panel on the console's front plane (size = along the row x up)."""
    mesh = vc.VMesh()
    vc.box(mesh, V(light["position_m"]) + cen.n * (LIGHT_PROUD_M / 2.0), cen.n, cen.up, cen.right, (LIGHT_PROUD_M / 2.0, light["size_m"][1] / 2.0, light["size_m"][0] / 2.0))
    return mesh


def build_button(b, panel):
    c = V(b["centre_m"])
    w, h = b["size_m"]
    mesh = vc.VMesh()
    height = COVER_PROUD_M if b["kind"] == "covered" else BUTTON_PROUD_M
    if b["shape"] == "round":
        vc.tube(mesh, c - panel.n * height, c, 0.5 * w, SIDES, ("btn", b["id"]))
    else:
        vc.box(mesh, c - panel.n * (height / 2.0), panel.n, panel.up, panel.right, (height / 2.0, h / 2.0, w / 2.0))
    return mesh


def build_handle(b, panel, shelf_off):
    """The centre handle: a raised pad up to centre_m with the T grip (the real recess and rim: texture)."""
    c = V(b["centre_m"])
    w, h = b["size_m"]
    mesh = vc.VMesh()
    depth = max((c - panel.o).dot(panel.n) - shelf_off, 0.004)
    vc.box(mesh, c - panel.n * (depth / 2.0), panel.n, panel.up, panel.right, (depth / 2.0, h / 2.0, w / 2.0))
    stem = c + panel.n * 0.03
    vc.tube(mesh, c - panel.n * 0.002, stem, 0.008, SIDES, "handle_stem")
    vc.tube(mesh, stem - panel.right * (w * 0.31), stem + panel.right * (w * 0.31), 0.012, SIDES, "handle_bar")
    return mesh


def build_stick(eye, grip):
    n, b, z = seat_axes(35.0)
    g = V(eye) + V(grip)
    mesh = vc.VMesh()
    vc.box(mesh, g - b * 0.17, n, b, z, (0.04, 0.015, 0.04))
    vc.tube(mesh, g - b * 0.16, g - b * 0.05, 0.011, SIDES, "stick_shaft")
    vc.tube(mesh, g - b * 0.07, g + b * 0.07, 0.019, SIDES, "stick_grip", radius1=0.016)
    return mesh


def build_throttle(eye, grip):
    n, b, z = seat_axes(35.0)
    g = V(eye) + V(grip)
    mesh = vc.VMesh()
    vc.box(mesh, g - b * 0.15, n, b, z, (0.05, 0.015, 0.03))
    vc.tube(mesh, g - b * 0.14, g - b * 0.03, 0.011, SIDES, "throttle_lever")
    vc.box(mesh, g, n, b, z, (0.025, 0.06, 0.035))
    return mesh


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    cockpit = spec["cockpit"]
    cabin, profile = cockpit["cabin"], spec["detail"]["capsule"]["profile_m"]
    vc.common.clear_scene()
    root = vc.empty("dragon_cabin")
    parts = {}
    angles = column_angles(cabin)
    parts["cabin_shell"], parts["floor"] = build_shell(cabin, profile, angles)
    parts["hatch_frame"] = build_hatch(cabin, profile)
    for ap in cl.window_apertures(cabin):
        parts[f"window_{ap['index']}"], parts[f"window_glass_{ap['index']}"] = build_window(cabin, profile, ap)
    for seat in cockpit["seat"]:
        for part, mesh in zip(cl.SEAT_PARTS, build_seat(seat, cabin["floor_x_m"])):
            parts[f"seat_{part}_{seat['id']}"] = mesh
    parts["console_housing"], parts["console_back"], parts["console_shelf"], parts["console_lockers"] = build_console(cockpit)
    parts["ceiling_ring_light"] = build_ring_light(cabin)
    scr = screens_by_id(cockpit)
    for s in cockpit["screen"]:
        parts[f"screen_{s['id']}"] = screen_mesh(s)
    for t in cockpit["tablet"]:
        parts[f"tablet_{t['id']}"] = build_tablet(t)
    for i, light in enumerate(cockpit["light"]):
        parts[f"light_{i}"] = build_light(light, Panel(scr["centre"]))
    for b in cockpit["button"]:
        panel = Panel(scr[b["panel"]])
        if b["kind"] == "handle":
            parts["handle"] = build_handle(b, panel, shelf_front_off(cockpit, panel))
        else:
            parts[cl.button_node(b)] = build_button(b, panel)
    eye = cockpit["eye"]["position_m"]
    parts["hotas_stick"] = build_stick(eye, cockpit["hotas"]["stick_grip_m"])
    parts["hotas_throttle"] = build_throttle(eye, cockpit["hotas"]["throttle_grip_m"])
    assert sorted(parts) == cl.node_names(cockpit), "node set differs from cabin_layout.node_names"
    tris = 0
    for name in sorted(parts):
        tris += parts[name].to_object(name, root)[1]
    vc.export_selected(Path(args["out"]) / "dragon_cabin.gltf", [root])
    vc.report_triangles("dragon_cabin", tris)


if __name__ == "__main__":
    main()
