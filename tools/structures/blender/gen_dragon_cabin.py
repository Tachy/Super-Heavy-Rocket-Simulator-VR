"""Crew Dragon cabin: gen_dragon_cabin.py -- --spec <json> --out <dir> writes dragon_cabin.gltf.

Own rebuild from config/cockpits/crew_dragon.toml (spec["cockpit"], validated by vehicle_spec.validate_cockpit); no third-party mesh.
Frame = the capsule stage body frame of gen_dragon.py / dragon_capsule.gltf (X station from the heat shield apex, nose +X, (Y, Z) = r (cos az, sin az),
Z = az +90 = the crew's left). Silhouette only: panel lines, labels and placards come later by texture.

Nodes: `cabin_shell` (inner wall of revolution with the ceiling ring, the forward tunnel and the top plate; normals into the cabin; cut at the window
reveals and the hatch), `floor`, `hatch_frame` (white trim, the recessed hatch inner face and its reveal), `window_<i>` (funnel from the round interior
reveal to the exterior pane, bezel ring; i = exterior table row, +az then -az) with `window_glass_<i>`, `seat_shell|pad|rest_<s1..s4>` (white shell, black pads, black leg-rest rails
+ foot plate at the seat's foot_rest_m), `console_housing` (dark: front and ends of the filleted body across the three screens + the dark upper panel above it, console_shape), `console_back`
(silver back face, the two ceiling posts with joint heads and funnel feet), `console_under` (dark grey underside and top), `console_lamps` (lamp strips under the console), `console_shelf` (the continuous sloped button tray with its lip, cabin_layout.tray_profiles), `console_lockers` (5 / 6),
`console_bezel` (the dark glass border around each active area, rounded outer corners; the housing carries the face frame recess_m in front of the glass),
`guard_<id>` / `guard_rim_<id>` (clear cover over a guarded key block / its red rim and hinge), `console_hit` (one flat quad per key face: the pointer's hit
geometry, hidden in the game),
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
import console_shape as cs  # noqa: E402
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
SCREEN_PROUD_M = cl.SCREEN_PROUD_M  # [E] the housing's front lies this far behind the screens (the screen quads sit exactly on the TOML plane)
KEY_EMBED_M = 0.001  # [E] key bodies (and the handle pad, the guard) reach this far into the tray: no gap at their foot
TRAY_FILLET_SEGMENTS = 3  # [E] steps per 90 deg of the tray's 4 mm fillets
CORNER_SEGMENTS = 4  # [E] steps per screen-glass corner arc (6 mm: 2.4 mm facets)
FRAME_ABOVE_GLASS_M = 0.03  # [E] the screens' face frame reaches this far above the glass tops (its bottom meets the tray's deck)
FRAME_HOLE_MARGIN_M = 0.003  # [E] the frame's grid cell around each glass outline (plate_with_holes margin; under the 11 mm frame outside the outer glass)
GUARD_RADIUS_M = 0.002  # [E] the clear cover's edge fillet
GUARD_RIM_M = (0.003, 0.002)  # [E] red rim bar width / height on the cover's top perimeter (jsc2022e068617: a thin red outline)
GUARD_HINGE_RADIUS_M = 0.003  # [E] hinge barrel along the hinge edge
TABLET_THICKNESS_M = 0.008  # [E] iPad mini 6.1 mm [S] plus the case
TABLET_POST_M = 0.012  # [E] mount post radius
FILLET_SEGMENTS = 4  # [E] steps per 90 deg fillet of the console body (22.5 deg: smooth highlight at 0.6 m; 4 cm radius -> 1.6 cm facets)
POST_SIDES = 20  # [E] sides of the ceiling posts (the 3.9 cm collar: 1.2 cm facets)
SEAT_BACK_HALF_M = (0.05, 0.40)  # [E] seat back half thickness / half height (a 0.8 m back, crew1_1 / Wikimedia)
SEAT_THIGH_M, SEAT_SHIN_M = 0.44, 0.40  # [E] pan length / lower-leg support length (seated adult, 5th percentile hip to knee 0.5)
SIDES = 16  # [E] sides of round parts
FOOT_PLATE_M = (0.32, 0.34, 0.02)  # [E] foot tray: length along the foot, width across, thickness (crew1_1: a plate-wide carbon tray under both boots)
FOOT_RAIL_Z_M = 0.17  # [E] leg-rest rails stand this far either side of the seat centre (outside the shins: hip joint 0.088 + boot half width 0.05)
FOOT_RAIL_RADIUS_M = 0.014  # [E] carbon rail radius (crew1_1 frame tubes ~30 mm)
LIGHT_PROUD_M = 0.006  # [E] lamp panel height over the console front
RING_LIFT_M = 0.015  # [E] the ring light stands this far under the ceiling plane (0.004 z-fought with the ceiling: concentric banding in pilot_eye)
RING_STEP_DEG = 3.0  # [E] segments of the ring light
# [E] crew1_1 (round 4): two groups of light rectangles on the console's underside along its back lip, at u 400-500 and 640-790 px of the 350-855 px back
# (1.32 m): lateral ranges from the centre (mirrored), and one strip's (width along the console normal, height under the underside, inset from the start of
# the back lip's fillet: Task 4 crew1_1 shows the strips right under the lip, v 228-232 against the lip edge at 226.5 px)
# Fix round 1: re-read on the as-built crew1_1 camera (photo light segments at v 228-232: u 406-434 / 466-546 | 642-734 / 742-786 -> 0.35-0.41 /
# 0.11-0.28 | 0.10-0.30 / 0.32-0.41 m, mean of both sides); round 4's (0.10, 0.36), (0.42, 0.56) rendered the outer strips ~60 px too far out
UNDER_LAMPS_LAT_M = ((0.10, 0.29), (0.335, 0.41))
UNDER_LAMP_M = (0.015, 0.003)  # [E] strip width across the lip, height over the surface
UNDER_LAMP_FILLET_DEG = 45.0  # [E] the strips sit on the back-bottom fillet this far round from the underside (Task 4 fix round 1: in front of the
# fillet they rendered 15-18 px of 800 under the photo's strips; on the fillet they lie right under the lip)

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


def build_seat(seat, floor_x, pan_raise_deg=0.0):
    """(shell, pad, rest): white shell, black pads, black carbon leg-rest frame + foot plate (crew1_1: white shells, black pads, carbon frames)."""
    n, b, z = seat_axes(seat["back_angle_deg"])
    r = math.radians(pan_raise_deg)  # the pan follows the thighs (crew_figure.toml thigh_raise_deg, negative = the knees lower than the seat-back normal)
    pn, pb = n * math.cos(r) + b * math.sin(r), b * math.cos(r) - n * math.sin(r)
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
    vc.box(shell, hip + pn * 0.5 * SEAT_THIGH_M - pb * 0.065, pn, pb, z, (0.5 * SEAT_THIGH_M, 0.025, 0.5 * w * 0.95))
    vc.box(pad, hip + pn * 0.5 * SEAT_THIGH_M - pb * 0.025, pn, pb, z, (0.5 * SEAT_THIGH_M * 0.95, 0.015, 0.5 * w * 0.78))
    for side in (-1.0, 1.0):  # armrests (black)
        vc.box(rest, hip + n * 0.2 + b * 0.2 + z * (side * (0.5 * w + 0.03)), n, b, z, (0.2, 0.025, 0.03))
    # leg rest: two carbon rails from the pan's front end to the foot plate, the plate under the boots (foot_rest_m / foot_rest_normal)
    plate_n, plate_c = V(seat["foot_rest_normal"]), V(seat["foot_rest_m"])
    plate_f = z.cross(plate_n)  # along the foot
    # the foot rest is a TRAY under the soles (round 3, crew1_1: seen nearly edge-on from the hatch): a flat carbon plate whose top face lies in the
    # plane through foot_rest_m (normal = up the shin), with a low lip at the toe end and a heel stop
    half_l, half_w = 0.5 * FOOT_PLATE_M[0], 0.5 * FOOT_PLATE_M[1]
    thick = FOOT_PLATE_M[2]
    vc.box(rest, plate_c - plate_n * (0.5 * thick), plate_f, plate_n, z, (half_l, 0.5 * thick, half_w))
    for end_sign in (-1.0, 1.0):  # heel stop and toe lip
        vc.box(rest, plate_c + plate_f * (end_sign * (half_l - 0.01)) + plate_n * 0.015, plate_f, plate_n, z, (0.01, 0.015, half_w))
    pan_end = hip + pn * SEAT_THIGH_M - pb * 0.065
    for side in (-1.0, 1.0):
        start = pan_end + z * (side * FOOT_RAIL_Z_M)
        end = plate_c - plate_f * half_l - plate_n * thick
        end.z = hip.z + side * FOOT_RAIL_Z_M
        vc.tube(rest, start, end, FOOT_RAIL_RADIUS_M, SIDES, ("rail", seat["id"], side))
    return shell, pad, rest


def add_shape(mesh, verts, faces, to_cabin, part_of, group_of):
    """Faces of a console_shape surface (local verts) into the VMeshes: to_cabin(local) -> cabin point, part_of(local normal) -> VMesh,
    group_of(local normal) -> smooth group (None = flat). UV = the local (x, y) (planar, metres)."""
    for f in faces:
        loc = [verts[i] for i in f]
        n = V((0.0, 0.0, 0.0))
        for i in range(len(loc)):  # Newell normal of the local face (console_shape winds counter-clockwise from outside)
            a, b = loc[i], loc[(i + 1) % len(loc)]
            n += V(((a[1] - b[1]) * (a[2] + b[2]), (a[2] - b[2]) * (a[0] + b[0]), (a[0] - b[0]) * (a[1] + b[1])))
        n.normalize()
        pts = [to_cabin(p) for p in loc]
        out = to_cabin((n.x, n.y, n.z)) - to_cabin((0.0, 0.0, 0.0))
        (part_of(n) if mesh is None else mesh).face(pts, [(p[0], p[1]) for p in loc], out, group_of(n))


def build_console(cockpit):
    """(housing, back, under, shelf, lockers, lamps): the console body (fidelity Task 4): one straight filleted body across all three screens
    (console_shape.rounded_box, every edge rounded to edge_radius_m, the underside lowered along underside_profile; fold_deg 0, user decision
    2026-11-07), split by face direction into the dark front / ends (`housing`), the silver back face (`back`, crew1_1; the vent band by shader,
    CockpitView) and the dark grey underside and top (`under`); the dark upper panel above it (`housing`); the two ceiling posts with their
    joint heads and funnel feet (`back`, console_shape.post at cabin_layout.console_posts); the shelf as separate modules, the two lockers 5 / 6,
    the lamp strips under the body along its back lip (`lamps`, crew1_1)."""
    con = cockpit["console"]
    if con["fold_deg"] != 0.0:
        raise ValueError("console: only the straight console (fold_deg 0) is built")
    frame = cl.console_frame(cockpit)  # axes from [console] tilt_deg (checked against the centre screen)
    cen = Panel({**screens_by_id(cockpit)["centre"], "normal": frame["normal"], "up": frame["up"]})
    u0, u1 = frame["u_bot"], frame["u_top"]
    half_w, depth, r = con["width_m"] / 2.0, con["hood_depth_m"] - SCREEN_PROUD_M, con["edge_radius_m"]
    u_panel = (con["upper_top_x_m"] - cen.o.x) / cen.up.x  # the dark upper panel's front top edge reaches this station
    housing, back, under, shelf, lockers, lamps, bezel = vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh()

    def body_part(n):  # local normal: x right, y up, z toward the crew
        if n.z < -0.7071:
            return back
        return under if abs(n.y) > 0.7071 else housing

    def flat_or_fillet(tag):
        return lambda n: None if max(abs(n.x), abs(n.y), abs(n.z)) > 0.99999 else tag

    # the body: front face SCREEN_PROUD_M behind the screens, back face at hood_depth_m
    h = u1 - u0
    verts, faces = cs.rounded_box(2.0 * half_w, h, depth, r, FILLET_SEGMENTS)
    verts = cs.draft_sides(cs.drop_underside(verts, con["underside_profile"], h, depth), con["side_draft_deg"], 2.0 * half_w, depth)
    uc, oc = 0.5 * (u0 + u1), -SCREEN_PROUD_M - 0.5 * depth
    add_shape(None, verts, faces, lambda p: cen.at(p[0], uc + p[1], oc + p[2]), body_part, flat_or_fillet("console_body"))
    # the dark upper panel (it closes the view up toward the ceiling; its lower fillet hides in the body)
    t = con["upper_depth_m"]
    pr = min(r, 0.4 * t)
    pv, pf = cs.rounded_box(2.0 * half_w, u_panel - (u1 - pr), t, pr, FILLET_SEGMENTS)
    pc = 0.5 * (u_panel + u1 - pr)
    add_shape(housing, pv, pf, lambda p: cen.at(p[0], pc + p[1], -SCREEN_PROUD_M - 0.5 * t + p[2]), None, flat_or_fillet("console_panel"))
    # lamp strips on the back lip's fillet, centred UNDER_LAMP_FILLET_DEG round from the underside (crew1_1: right under the lip, v 228-232 px
    # against the lip edge at 226.5): the strip lies on the chord of the (lowered) fillet it covers, sunk 1 mm into it
    def fillet_point(ang):  # local body point on the back-bottom fillet, then lowered like the body (cs.drop_underside)
        y = -0.5 * h + r - r * math.cos(ang)
        z = -0.5 * depth + r - r * math.sin(ang)
        y -= cs.interp(con["underside_profile"], 0.5 * depth - z) * (-y) / (0.5 * h)
        return cen.at(0.0, uc + y, oc + z)

    a, da = math.radians(UNDER_LAMP_FILLET_DEG), 0.5 * UNDER_LAMP_M[0] / r
    p0, p1 = fillet_point(a - da), fillet_point(a + da)
    tangent = (p1 - p0).normalized()
    out_n = tangent.cross(cen.right)
    if out_n.dot(cen.up + cen.n) > 0.0:  # outward = away from the body (down and back)
        out_n = -out_n
    mid = 0.5 * (p0 + p1)
    for lat0, lat1 in UNDER_LAMPS_LAT_M:
        for sign in (-1.0, 1.0):
            lo, hi = sorted((sign * lat0, sign * lat1))
            centre = mid + cen.right * (0.5 * (lo + hi)) + out_n * (0.5 * UNDER_LAMP_M[1] - 0.001)
            vc.box(lamps, centre, out_n, tangent, cen.right, (0.5 * UNDER_LAMP_M[1], 0.5 * (p1 - p0).length, 0.5 * (hi - lo)))
    # the two ceiling posts: rod, joint head (collar) at the ceiling, sleeve and funnel foot on the body (crew1_1, Task 4 fit)
    for k, (base, top) in enumerate(cl.console_posts(cockpit)):
        verts, faces = cs.post(base, top, con["post_radius_m"], con["post_head_radius_m"], con["post_head_length_m"], POST_SIDES,
                               foot=tuple(con["post_foot_m"]), top_plane_normal=(1.0, 0.0, 0.0))  # flush on the flat ceiling ring
        axis = (V(top) - V(base)).normalized()
        # smooth within one lathe segment (rod, head, sleeve, funnel: same axial normal component), flat caps and steps
        add_shape(back, verts, faces, lambda p: V(p), None,
                  lambda n, k=k, axis=axis: None if abs(n.dot(axis)) > 0.99 else ("post", k, round(n.dot(axis), 2)))
    # the tray (fidelity Task 5): one continuous sloped tray under all keys, the deck from the housing, the key face, the lip at its bottom
    tr = cockpit["tray"]
    body, lip = cl.tray_profiles(cockpit)
    for prof in (body, lip):
        tv, tf = cs.extrude(cs.round_corners(prof, tr["radius_m"], TRAY_FILLET_SEGMENTS, only={1, 2}), -tr["half_length_m"], tr["half_length_m"])
        add_shape(shelf, tv, tf, lambda p: cen.at(p[0], p[1], p[2]), None, lambda n, k=len(prof): None if abs(n.x) > 0.99 else ("tray", k))
    # the screens' face frame: a plate recess_m in front of the glass with rounded holes (the glass outlines = active area + bezel), its walls
    # down to the glass and back to the housing front; the bezel (dark glass border) around each active area at the screen plane
    screens = cockpit["screen"]
    rec = screens[0]["recess_m"]  # vehicle_spec requires one recess_m for the three screens (one face-frame plate)
    holes = []
    for s in screens:
        lat = (V(s["centre_m"]) - cen.o).dot(cen.right)
        hw, hh = s["size_m"][0] / 2.0 + s["bezel_m"], s["size_m"][1] / 2.0 + s["bezel_m"]
        holes.append((lat, 0.0, hw, hh, s["corner_radius_m"]))
        outer = cs.rounded_rect(lat, 0.0, hw, hh, s["corner_radius_m"], CORNER_SEGMENTS)
        inner = cs.rounded_rect(lat, 0.0, s["size_m"][0] / 2.0, s["size_m"][1] / 2.0, 0.0, CORNER_SEGMENTS)
        for f in cs.ring(outer, inner):
            bezel.face([cen.at(x, y, 0.0) for x, y in f], [(x, y) for x, y in f], cen.n, None)
        for q in cs.wall(cs.loop(outer), 0.0, rec, inward=True):
            add_shape(housing, q, [tuple(range(4))], lambda p: cen.at(p[0], p[1], p[2]), None, lambda n: "screen_well")
    x0, x1 = -half_w + r, half_w - r
    y0, y1 = cl.tray_rows(cockpit)["top"], max(h[3] for h in holes) + FRAME_ABOVE_GLASS_M
    for f in cs.plate_with_holes(x0, x1, y0, y1, holes, FRAME_HOLE_MARGIN_M, CORNER_SEGMENTS):
        housing.face([cen.at(x, y, rec) for x, y in f], [(x, y) for x, y in f], cen.n, None)
    for q in cs.wall([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], -SCREEN_PROUD_M, rec):
        add_shape(housing, q, [tuple(range(4))], lambda p: cen.at(p[0], p[1], p[2]), None, lambda n: None)
    for lk in cockpit["locker"]:  # dark lockers with the placards 5 / 6 (the placard: texture)
        c = V(lk["centre_m"]) - cen.o
        cen.box(lockers, c.dot(cen.right) - lk["size_m"][0] / 2.0, c.dot(cen.right) + lk["size_m"][0] / 2.0,
                c.dot(cen.up) - lk["size_m"][1] / 2.0, c.dot(cen.up) + lk["size_m"][1] / 2.0, con["locker_proud_m"], -SCREEN_PROUD_M)
    return housing, back, under, shelf, lockers, lamps, bezel


class TrayFrame:
    """The tray's key face frame at a point: n = the face normal (the console normal turned up by slope_deg), up = up the face, right = the row."""

    def __init__(self, cockpit, cen):
        (nu, no), (tu, to) = cl.tray_axes(cockpit)
        self.n = (cen.up * nu + cen.n * no).normalized()
        self.up = (cen.up * tu + cen.n * to).normalized()
        self.right = cen.right


def build_button(b, tray):
    """A key on the tray: its face at centre_m square to the tray normal, key_height proud, its foot 1 mm into the tray."""
    c = V(b["centre_m"])
    w, h = b["size_m"]
    mesh = vc.VMesh()
    height = cl.key_height(b) + KEY_EMBED_M
    if b["shape"] == "round":
        vc.tube(mesh, c - tray.n * height, c, 0.5 * w, SIDES, ("btn", b["id"]))
    else:
        vc.box(mesh, c - tray.n * (height / 2.0), tray.n, tray.up, tray.right, (height / 2.0, h / 2.0, w / 2.0))
    return mesh


def build_handle(cockpit, b, panel):
    """The centre handle: a raised pad from the tray face up to centre_m with the T grip (the real recess and rim: texture)."""
    c = V(b["centre_m"])
    w, h = b["size_m"]
    mesh = vc.VMesh()
    u = (c - panel.o).dot(panel.up)
    depth = max((c - panel.o).dot(panel.n) - cl.tray_face_o(cockpit, u) + KEY_EMBED_M, 0.004)
    vc.box(mesh, c - panel.n * (depth / 2.0), panel.n, panel.up, panel.right, (depth / 2.0, h / 2.0, w / 2.0))
    stem = c + panel.n * 0.03
    vc.tube(mesh, c - panel.n * 0.002, stem, 0.008, SIDES, "handle_stem")
    vc.tube(mesh, stem - panel.right * (w * 0.31), stem + panel.right * (w * 0.31), 0.012, SIDES, "handle_bar")
    return mesh


def build_guard(cockpit, g, cen, tray):
    """(cover, rim): the clear cover over a guarded key block (a filleted box standing on the tray face) and its red rim along the top
    perimeter with the hinge barrel on hinge_edge."""
    lat0, lat1 = g["lat"]
    u0, u1 = g["u"]
    sl = math.radians(cockpit["tray"]["slope_deg"])
    um = 0.5 * (u0 + u1)
    foot = cen.at(0.5 * (lat0 + lat1), um, cl.tray_face_o(cockpit, um))  # the face point under the cover's middle
    w, length, hgt = lat1 - lat0, (u1 - u0) / math.cos(sl), g["height"]
    cover, rim = vc.VMesh(), vc.VMesh()
    centre = foot + tray.n * (0.5 * hgt - KEY_EMBED_M)
    verts, faces = cs.rounded_box(w, length, hgt + KEY_EMBED_M, GUARD_RADIUS_M, 2)
    add_shape(cover, verts, faces, lambda p: centre + tray.right * p[0] + tray.up * p[1] + tray.n * p[2], None,
              lambda n: None if max(abs(n.x), abs(n.y), abs(n.z)) > 0.99999 else "guard")
    top = foot + tray.n * (hgt + 0.5 * GUARD_RIM_M[1])
    hw, hl, rw = 0.5 * w, 0.5 * length, 0.5 * GUARD_RIM_M[0]
    for sgn in (-1.0, 1.0):  # rim bars: along the row (top / bottom), across it (ends)
        vc.box(rim, top + tray.up * (sgn * (hl - rw)), tray.n, tray.up, tray.right, (0.5 * GUARD_RIM_M[1], rw, hw))
        vc.box(rim, top + tray.right * (sgn * (hw - rw)), tray.n, tray.up, tray.right, (0.5 * GUARD_RIM_M[1], hl - 2 * rw, rw))
    edge = {"top": tray.up * hl, "bottom": -tray.up * hl, "left": -tray.right * hw, "right": tray.right * hw}[g["hinge_edge"]]
    along = tray.right * hw if g["hinge_edge"] in ("top", "bottom") else tray.up * hl
    hinge = foot + edge + tray.n * (hgt - GUARD_HINGE_RADIUS_M)
    vc.tube(rim, hinge - along, hinge + along, GUARD_HINGE_RADIUS_M, SIDES, ("hinge", g["hinge_edge"]))
    return cover, rim


def build_hits(cockpit):
    """console_hit: one flat quad per key face (cabin_layout.hit_faces), invisible in the game (the pointer's simplified hit geometry)."""
    mesh = vc.VMesh()
    cen = Panel(screens_by_id(cockpit)["centre"])
    for corners in cl.hit_faces(cockpit).values():
        pts = [V(p) for p in corners]
        n = (pts[1] - pts[0]).cross(pts[3] - pts[0])
        mesh.face(pts, [(0, 0), (1, 0), (1, 1), (0, 1)], n if n.dot(cen.n) > 0.0 else -n, None)
    return mesh


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
        for part, mesh in zip(cl.SEAT_PARTS, build_seat(seat, cabin["floor_x_m"], spec["crew_figure"]["figure"]["thigh_raise_deg"])):
            parts[f"seat_{part}_{seat['id']}"] = mesh
    (parts["console_housing"], parts["console_back"], parts["console_under"], parts["console_shelf"], parts["console_lockers"],
     parts["console_lamps"], parts["console_bezel"]) = build_console(cockpit)
    parts["ceiling_ring_light"] = build_ring_light(cabin)
    scr = screens_by_id(cockpit)
    cen = Panel(scr["centre"])
    tray = TrayFrame(cockpit, cen)
    for gid, g in cl.guards(cockpit).items():
        parts[f"guard_{gid}"], parts[f"guard_rim_{gid}"] = build_guard(cockpit, g, cen, tray)
    parts["console_hit"] = build_hits(cockpit)
    for s in cockpit["screen"]:
        parts[f"screen_{s['id']}"] = screen_mesh(s)
    for t in cockpit["tablet"]:
        parts[f"tablet_{t['id']}"] = build_tablet(t)
    for i, light in enumerate(cockpit["light"]):
        parts[f"light_{i}"] = build_light(light, Panel(scr["centre"]))
    for b in cockpit["button"]:
        if b["kind"] == "handle":
            parts["handle"] = build_handle(cockpit, b, cen)
        else:
            parts[cl.button_node(b)] = build_button(b, tray)
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
