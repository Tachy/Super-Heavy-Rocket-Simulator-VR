"""Crew figure in the newer SpaceX EVA suit: gen_crew_figure.py -- --spec <json> --out <dir> writes crew_figure.gltf.

Own rebuild from config/cockpits/crew_figure.toml (spec["crew_figure"]); no third-party mesh. Frame: origin = the seat's hip reference point,
X = up the seat back, Y = the seat's chest normal (forward), Z = the crew's left (figure_layout.py, crew_figure.toml header). Rest pose = seated,
hands on the thighs. Two skinned mesh nodes under the armature `crew_figure_rig`: `crew_figure` = the suit hull of figure_body.py
(anatomical ANSUR II body plus the suit ease; smooth weights across the joints, corrective shape keys per joint, the atlas UV and the
wrinkle-mask UV; the only mesh with morph targets) and `crew_figure_parts` = the rigid parts (helmet, visor, hood, glove lock rings,
gloves with five fingers, boots: one bone each). Surfaces (glTF primitives) = the materials suit | boots, gloves, helmet, collar, visor. Suit detail (stripes, seams, folds) comes from suit_textures.py.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402
import figure_body as fb  # noqa: E402
import figure_layout as fl  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

V = Vector
SIDES = 14  # [E] sides of limb sections
TORSO_SIDES = 20  # [E] sides of the torso sections
HELMET_SIDES = 48  # [E] columns (azimuth) of the helmet shell (a smooth silhouette at close range, the facets crisp)
CAP_STEPS = 3  # [E] rings of a rounded end
HELMET_RINGS = 30  # [E] rows of the helmet shell from the sloped bottom cut to the crown
HELMET_TOP_EL = 89.0  # [E] the columns stop here; a fan closes the crown
VISOR_COLS, VISOR_ROWS = 24, 16
RIM_SIDES = 14  # [E] sides of the rim bead's cross section
PIVOT_SIDES = 20  # [E] sides of the round visor-pivot button
FINGER_SIDES = 8  # [E] sides of a glove finger
PARTS_NODE = "crew_figure_parts"  # the rigid parts' mesh node (no shape keys)


def vec(t):
    return V(t)


class Parts:
    """One VMesh per (bone, material); every part becomes an object with the bone's vertex group (weight 1)."""

    def __init__(self):
        self.meshes = {}

    def mesh(self, bone, material):
        key = (bone, material)
        if key not in self.meshes:
            self.meshes[key] = vc.VMesh()
        return self.meshes[key]


def frame(axis, uref):
    a = V(axis).normalized()
    u = V(uref) - a * V(uref).dot(a)
    u.normalize()
    return a, u, a.cross(u)


def loft(mesh, origin, axis, uref, rings, sides, group, cap0=False, cap1=False):
    """Sections along `axis` from `origin`: rings = [(s, half_u, half_v, centre_u, centre_v)], elliptical, smooth sides, optional flat end caps."""
    o = V(origin)
    a, u, v = frame(axis, uref)
    pts, centres = [], []
    for s, hu, hv, cu, cv in rings:
        c = o + a * s + u * cu + v * cv
        centres.append(c)
        pts.append([c + u * (hu * math.cos(2.0 * math.pi * k / sides)) + v * (hv * math.sin(2.0 * math.pi * k / sides)) for k in range(sides)])
    n = len(rings)
    for i in range(n - 1):
        for k in range(sides):
            k1 = (k + 1) % sides
            quad = [pts[i][k], pts[i][k1], pts[i + 1][k1], pts[i + 1][k]]
            if len({(round(p.x, 7), round(p.y, 7), round(p.z, 7)) for p in quad}) < 3:
                continue
            mid = sum(quad, V((0, 0, 0))) / 4.0
            out = mid - (centres[i] + centres[i + 1]) / 2.0
            if out.length < 1e-9:
                out = a * (1.0 if rings[i + 1][0] > rings[i][0] else -1.0)
            uv = [(k / sides, i / (n - 1)), (k1 / sides, i / (n - 1)), (k1 / sides, (i + 1) / (n - 1)), (k / sides, (i + 1) / (n - 1))]
            mesh.face(quad, uv, out, group)
    for end, flag, sign in ((0, cap0, -1.0), (n - 1, cap1, 1.0)):
        if flag:
            for k in range(sides):
                k1 = (k + 1) % sides
                mesh.face([centres[end], pts[end][k], pts[end][k1]], [(0.5, 0.5), (0.0, 0.0), (1.0, 0.0)], a * sign)


def capsule_rings(length, ru0, rv0, ru1, rv1, cap0=True, cap1=True):
    """Rings of a limb segment from s = 0 to `length` with elliptical ends of the sections (ru, rv); round caps extend one radius beyond."""
    rings = []
    if cap0:
        r = min(ru0, rv0)
        for i in range(CAP_STEPS):
            th = (math.pi / 2.0) * i / CAP_STEPS
            rings.append((-r * math.cos(th), ru0 * math.sin(th), rv0 * math.sin(th), 0.0, 0.0))
    rings.append((0.0, ru0, rv0, 0.0, 0.0))
    rings.append((length, ru1, rv1, 0.0, 0.0))
    if cap1:
        r = min(ru1, rv1)
        for i in range(1, CAP_STEPS + 1):
            th = (math.pi / 2.0) * (1.0 - i / CAP_STEPS)
            rings.append((length + r * math.cos(th), ru1 * math.sin(th), rv1 * math.sin(th), 0.0, 0.0))
    return rings


def limb(mesh, p0, p1, r0, r1, group, sides=SIDES, cap0=True, cap1=True, uref=(0.0, 0.0, 1.0), flat0=False, flat1=False):
    """Capsule from p0 to p1 (circular sections r0 -> r1)."""
    p0, p1 = V(p0), V(p1)
    ref = V(uref)
    if abs((p1 - p0).normalized().dot(ref.normalized())) > 0.95:
        ref = V((0.0, 1.0, 0.0))
    loft(mesh, p0, p1 - p0, ref, capsule_rings((p1 - p0).length, r0, r0, r1, r1, cap0, cap1), sides, group, flat0, flat1)


def blend(rows, s):
    return fl.lerp_table(rows, s)


# ---------- the parts ----------

def build_helmet(cfg, parts):
    h, pose = cfg["helmet"], cfg["pose"]
    c = V((pose["head_centre_up_m"], pose["head_centre_forward_m"], 0.0))
    rim = build_shell(h, parts.mesh("head", "helmet"), c)
    build_hood(cfg, parts.mesh("neck", "collar"), c, rim)
    build_visor(cfg, parts, c)


def shell_at(h, c, el, az):
    """(point, normal) on the helmet shell (figure_layout.helmet_point) in the figure frame."""
    p, n = fl.helmet_point(h, el, az)
    return c + V(p), V(n)


def build_shell(h, mesh, c):
    """Task 5b: the compact faceted shell (figure_layout.helmet_*): columns of constant azimuth from the sloped bottom cut to the crown, one
    smooth group (per-facet groups left a zig-zag of shading along the facet edges, which cross the columns at an angle). Returns the bottom rim [(point, normal)] per column (the hood's top)."""
    azs = [360.0 * k / HELMET_SIDES for k in range(HELMET_SIDES)]
    cols = []
    for az in azs:
        e0 = fl.helmet_bottom_el(h, az)
        cols.append([shell_at(h, c, e0 + (HELMET_TOP_EL - e0) * i / (HELMET_RINGS - 1), az) for i in range(HELMET_RINGS)])
    for k in range(HELMET_SIDES):
        k1 = (k + 1) % HELMET_SIDES
        for i in range(HELMET_RINGS - 1):
            quad = [cols[k][i][0], cols[k1][i][0], cols[k1][i + 1][0], cols[k][i + 1][0]]
            if len({(round(p.x, 7), round(p.y, 7), round(p.z, 7)) for p in quad}) < 3:
                continue
            mid = sum(quad, V((0, 0, 0))) / 4.0
            mesh.face(quad, [(k / HELMET_SIDES, i / HELMET_RINGS), (k1 / HELMET_SIDES, i / HELMET_RINGS), (k1 / HELMET_SIDES, (i + 1) / HELMET_RINGS),
                             (k / HELMET_SIDES, (i + 1) / HELMET_RINGS)], mid - c, "shell")
    top = c + V((fl.helmet_rho(h, (1.0, 0.0, 0.0)), 0.0, 0.0))
    for k in range(HELMET_SIDES):  # close the crown (the columns stop just under the pole)
        k1 = (k + 1) % HELMET_SIDES
        mesh.face([cols[k][-1][0], cols[k1][-1][0], top], [(0.0, 0.0), (1.0, 0.0), (0.5, 1.0)], V((1.0, 0.0, 0.0)), "shell")
    return [col[0] for col in cols]


def build_hood(cfg, mesh, c, rim):
    """The grey knit neck hood (spacex-anzug): from an elliptical base ring on the shoulders (collar_height_m below the helmet centre,
    neck_lean_m behind it, half depth collar_radius_m, half width 0.9 of it) up to the shell's sloped lower edge (3 mm inside it)."""
    h, pose = cfg["helmet"], cfg["pose"]
    base = V((c.x - h["collar_height_m"], c.y - pose["neck_lean_m"], 0.0))
    ry, rz = h["collar_radius_m"], 0.9 * h["collar_radius_m"]
    n = len(rim)
    rings = []
    for t in (0.0, 0.35, 0.7, 1.0):
        ring = []
        for k, (p_top, n_top) in enumerate(rim):
            az = 2.0 * math.pi * k / n
            p_base = base + V((0.0, ry * math.cos(az), rz * math.sin(az)))
            p = p_base.lerp(p_top - n_top * 0.003, t)
            ring.append(p)
        rings.append(ring)
    axis_top = sum((p for p, _ in rim), V((0, 0, 0))) / n
    for i in range(len(rings) - 1):
        for k in range(n):
            k1 = (k + 1) % n
            quad = [rings[i][k], rings[i][k1], rings[i + 1][k1], rings[i + 1][k]]
            mid = sum(quad, V((0, 0, 0))) / 4.0
            axis = base.lerp(axis_top, (i + 0.5) / (len(rings) - 1))
            mesh.face(quad, [(k / n, i / 3.0), (k1 / n, i / 3.0), (k1 / n, (i + 1) / 3.0), (k / n, (i + 1) / 3.0)], mid - axis, "hood")
    for k in range(n):  # flat bottom on the shoulders (hidden in the suit)
        k1 = (k + 1) % n
        mesh.face([base, rings[0][k1], rings[0][k]], [(0.5, 0.5), (1.0, 0.0), (0.0, 0.0)], V((-1.0, 0.0, 0.0)), "hood_base")


def opening_node(h, i, j, inset=0.0):
    """Grid node (i, j) of the face opening (visor_outline): (elevation, azimuth) in degrees; i = 0 the chin tip row, VISOR_ROWS the brow."""
    e0, e1 = fl.visor_el_range(h)
    el = e0 + inset + (e1 - e0 - 2.0 * inset) * i / VISOR_ROWS
    half = fl.visor_half_az(h, e0 + (e1 - e0) * i / VISOR_ROWS)  # the width of the frame's row: an inset bead never reaches past the frame
    half = max(half - inset, 0.4 * half)
    return el, (2.0 * j / VISOR_COLS - 1.0) * half


def visor_bulge(h, i, j):
    """Height of the visor's outer surface over the shell at grid node (i, j): the dome (visor_proud_m) at the centre, visor_edge_m on the whole rim (a smooth bulge)."""
    a = 2.0 * j / VISOR_COLS - 1.0
    b = 2.0 * i / VISOR_ROWS - 1.0
    w = (1.0 - a * a) * (1.0 - b * b)
    return h["visor_edge_m"] + (h["visor_proud_m"] - h["visor_edge_m"]) * w


def build_visor(cfg, parts, c):
    """The closed visor: a forward-domed outer and an inner patch over the shell and a rim, one solid (own surface), plus the white frame bead
    (helmet surface), the black seal inside it and the pivot buttons."""
    h = cfg["helmet"]
    mesh = parts.mesh("head", "visor")
    outer, inner = {}, {}
    for i in range(VISOR_ROWS + 1):
        for j in range(VISOR_COLS + 1):
            p, n = shell_at(h, c, *opening_node(h, i, j))
            bulge = visor_bulge(h, i, j)
            outer[i, j] = p + n * bulge
            inner[i, j] = p + n * (bulge - h["visor_thickness_m"])
    for i in range(VISOR_ROWS):
        for j in range(VISOR_COLS):
            ids = [(i, j), (i, j + 1), (i + 1, j + 1), (i + 1, j)]
            uv = [(j / VISOR_COLS, i / VISOR_ROWS), ((j + 1) / VISOR_COLS, i / VISOR_ROWS), ((j + 1) / VISOR_COLS, (i + 1) / VISOR_ROWS), (j / VISOR_COLS, (i + 1) / VISOR_ROWS)]
            mid_o = sum((outer[k] for k in ids), V((0, 0, 0))) / 4.0
            mid_i = sum((inner[k] for k in ids), V((0, 0, 0))) / 4.0
            mesh.face([outer[k] for k in ids], uv, mid_o - c, "visor_out")
            mesh.face([inner[k] for k in ids], uv, c - mid_i, "visor_in")
    edges = ([((0, j), (0, j + 1)) for j in range(VISOR_COLS)] + [((VISOR_ROWS, j), (VISOR_ROWS, j + 1)) for j in range(VISOR_COLS)]
             + [((i, 0), (i + 1, 0)) for i in range(VISOR_ROWS)] + [((i, VISOR_COLS), (i + 1, VISOR_COLS)) for i in range(VISOR_ROWS)])
    centre = sum(outer.values(), V((0, 0, 0))) / len(outer)
    for a, b in edges:
        mid = (outer[a] + outer[b]) / 2.0
        mesh.face([outer[a], outer[b], inner[b], inner[a]], [(0, 0), (1, 0), (1, 1), (0, 1)], mid - centre)
    # one continuous swept bead each (smooth, shared vertices): the thin white visor frame on the opening's edge and, just inside it, the black
    # seal that lines the face opening (spacex-anzug: a thin dark line between the white shell and the visor)
    loop = ([(0, j) for j in range(VISOR_COLS + 1)] + [(i, VISOR_COLS) for i in range(1, VISOR_ROWS + 1)]
            + [(VISOR_ROWS, j) for j in range(VISOR_COLS - 1, -1, -1)] + [(i, 0) for i in range(VISOR_ROWS - 1, 0, -1)])
    for material, radius, inset in (("helmet", h["rim_radius_m"], 0.0), ("boots", h["liner_radius_m"], h["liner_inset_deg"])):
        pts = {}
        for key in loop:
            p, n = shell_at(h, c, *opening_node(h, *key, inset=inset))
            pts[key] = (p + n * (h["visor_edge_m"] * 0.5), n)
        sweep_bead(parts.mesh("head", material), loop, pts, radius, ("rim", material))
    # the visor-pivot buttons: a low round grey disc on each side of the shell behind the opening, at jaw height (jsc2024e066389 / 066390).
    # Its foot sinks into the shell so the curved surface leaves no gap.
    button = parts.mesh("head", "collar")
    r_b, h_b = h["pivot_button_radius_m"], h["pivot_button_height_m"]
    az_b = fl.visor_half_az(h, h["pivot_el_deg"]) + h["pivot_az_offset_deg"]
    for side in (-1.0, 1.0):
        p, n = shell_at(h, c, h["pivot_el_deg"], side * az_b)
        vc.tube(button, p - n * (0.5 * r_b), p + n * h_b, r_b, PIVOT_SIDES, ("pivot", side), cap0=False, radius1=0.85 * r_b)


def sweep_bead(mesh, loop, pts, radius, group):
    """A round bead of `radius` swept along the closed loop of (point, normal) entries."""
    rings = []
    for k, key in enumerate(loop):
        prev, nxt = loop[k - 1], loop[(k + 1) % len(loop)]
        t = (pts[nxt][0] - pts[prev][0]).normalized()
        nrm = pts[key][1]
        side = t.cross(nrm).normalized()
        rings.append([pts[key][0] + (nrm * math.cos(2.0 * math.pi * m / RIM_SIDES) + side * math.sin(2.0 * math.pi * m / RIM_SIDES)) * radius
                      for m in range(RIM_SIDES)])
    for k in range(len(loop)):
        k1 = (k + 1) % len(loop)
        for m in range(RIM_SIDES):
            m1 = (m + 1) % RIM_SIDES
            quad = [rings[k][m], rings[k1][m], rings[k1][m1], rings[k][m1]]
            mid = sum(quad, V((0, 0, 0))) / 4.0
            mesh.face(quad, [(0, 0), (1, 0), (1, 1), (0, 1)], mid - pts[loop[k]][0], group)


def build_arm(cfg, parts, bones, side):
    """The rigid hand parts of one side: the glove lock ring at the wrist (helmet white), the glove (palm, index, the middle / ring / little
    fingers on the bones fingers, fingers_2, fingers_3, the thumb). The sleeve is the suit hull (figure_body)."""
    suit, hand = cfg["suit"], cfg["hand"]
    b = {n: bones[f"{n}_{side}"] for n in ("hand", "index_1", "index_2", "index_3", "thumb_1", "thumb_2", "fingers", "fingers_2", "fingers_3")}
    f, d, t = fl.hand_frame(cfg, side)
    wrist = V(b["hand"]["head"])
    cuff_axis = (V(b["hand"]["head"]) - V(bones[f"forearm_{side}"]["head"])).normalized()
    loft(parts.mesh(f"forearm_{side}", "helmet"), wrist - cuff_axis * suit["cuff_length_m"], cuff_axis, (0.0, 0.0, 1.0),
         [(0.0, suit["cuff_radius_m"], suit["cuff_radius_m"], 0.0, 0.0), (suit["cuff_length_m"] + 0.004, suit["cuff_radius_m"], suit["cuff_radius_m"], 0.0, 0.0)],
         SIDES, f"cuff_{side}", True, True)
    # palm: elliptical (width along the thumb axis t, thickness along the palm's down d)
    pl = hand["palm_length_m"]
    palm = [(-0.0, hand["palm_half_width_m"] * 0.80, hand["palm_half_thick_m"] * 0.90, 0.0, 0.0),
            (pl * 0.45, hand["palm_half_width_m"], hand["palm_half_thick_m"], 0.0, 0.0),
            (pl, hand["palm_half_width_m"] * 0.98, hand["palm_half_thick_m"] * 0.9, 0.0, 0.0)]
    loft(parts.mesh(f"hand_{side}", "gloves"), wrist, f, t, palm, SIDES, f"palm_{side}", True, True)
    for k in (1, 2, 3):
        bone = b[f"index_{k}"]
        r = hand["index_radius_m"] * (1.0 - 0.1 * (k - 1))
        limb(parts.mesh(f"index_{k}_{side}", "gloves"), bone["head"], bone["tail"], r, r * 0.92, f"index{k}_{side}", FINGER_SIDES, uref=tuple(t))
    # middle, ring and little finger side by side on the shared bones (one curl), toward the little finger from the middle knuckle
    for j, share in enumerate(hand["finger_length_share"]):
        off = V(t) * (-hand["finger_spacing_m"] * j)
        r0 = hand["finger_radius_m"] * (1.0 - 0.06 * j)
        for k, name in enumerate(("fingers", "fingers_2", "fingers_3")):
            bone = b[name]
            head, tail = V(bone["head"]) + off, V(bone["tail"]) + off
            tail = head + (tail - head) * share
            if k > 0:  # the shortened finger's joints follow its own previous phalanx
                head = prev_tail
            r = r0 * (1.0 - 0.1 * k)
            limb(parts.mesh(f"{name}_{side}", "gloves"), head, tail, r, r * 0.92, f"finger{j}{k}_{side}", FINGER_SIDES, uref=tuple(t))
            prev_tail = tail
    for k in (1, 2):
        bone = b[f"thumb_{k}"]
        r = hand["thumb_radius_m"] * (1.0 - 0.12 * (k - 1))
        limb(parts.mesh(f"thumb_{k}_{side}", "gloves"), bone["head"], bone["tail"], r, r * 0.9, f"thumb{k}_{side}", FINGER_SIDES, uref=tuple(d))


def build_leg(cfg, parts, bones, side):
    """The boot: a shaft over the suit's lower leg (rigid on the shin) and the foot (rigid on the foot bone)."""
    suit, body = cfg["suit"], cfg["body"]
    sh, ft = bones[f"shin_{side}"], bones[f"foot_{side}"]
    knee, ankle = V(sh["head"]), V(sh["tail"])
    thigh_len = fl.chain_lengths(cfg)["thigh"]
    rows = fb._fit_rows(body["leg_sections_m"])
    n_rings = 6
    shaft = suit["boot_shaft_fraction"]
    axis = (ankle - knee).normalized()
    rings = []
    for i in range(n_rings + 1):
        f = 1.0 - shaft + shaft * i / n_rings  # from the shaft top to the ankle
        a_half, b_half = fb._section(rows, thigh_len + f * (ankle - knee).length)
        e = suit["ease_shank_m"] + suit["boot_ease_m"]
        rings.append(((f - (1.0 - shaft)) * (ankle - knee).length, a_half + e, b_half + e, 0.0, 0.0))
    rings.append((rings[-1][0] + 0.03, rings[-1][1], rings[-1][2], 0.0, 0.0))
    top = knee + (ankle - knee) * (1.0 - shaft)
    loft(parts.mesh(f"shin_{side}", "boots"), top, axis, (0.0, 0.0, 1.0), rings, SIDES, f"boot_{side}", True, True)
    # foot: along the foot direction from the heel; sections (s from the heel, half width, half height, centre height above the foot line)
    fdir = V(ft["tail"]) - V(ft["head"])
    fdir.normalize()
    heel = V(ft["head"]) - fdir * suit["boot_heel_m"]
    hw, hh = suit["boot_foot_half_width_m"], suit["boot_foot_half_height_m"]
    total = suit["boot_length_m"]
    s_ankle = suit["boot_heel_m"]
    rings = [(0.0, hw * 0.78, hh * 0.80, -0.020, 0.0), (s_ankle * 0.5, hw * 0.92, hh * 0.92, -0.020, 0.0), (s_ankle, hw, hh, -0.020, 0.0),
             (total * 0.55, hw * 1.04, hh * 0.80, -0.030, 0.0), (total * 0.72, hw * 1.04, hh * 0.64, -0.040, 0.0),
             (total * 0.90, hw * 0.82, hh * 0.48, -0.046, 0.0), (total, hw * 0.45, hh * 0.28, -0.052, 0.0)]
    # the loft's v axis = axis x u; with u = the side axis z this is the foot's up (toward the knee): centre_v = the offset above the foot line
    rings = [(s, a, b, 0.0, cv) for s, a, b, cv, _ in rings]
    loft(parts.mesh(f"foot_{side}", "boots"), heel, fdir, (0.0, 0.0, 1.0), rings, SIDES, f"foot_{side}", True, True)


def build_figure(cfg):
    bones = fl.bones(cfg)
    parts = Parts()
    build_helmet(cfg, parts)
    for side in fl.SIDES:
        build_arm(cfg, parts, bones, side)
        build_leg(cfg, parts, bones, side)
    return bones, parts


# ---------- Blender objects ----------

def make_materials(cfg):
    mats = {}
    for name in fl.MATERIALS:
        m = cfg["material"][name]
        mat = bpy.data.materials.new(name)
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes["Principled BSDF"]
        bsdf.inputs["Base Color"].default_value = (*m["albedo"], 1.0)
        bsdf.inputs["Roughness"].default_value = m["roughness"]
        bsdf.inputs["Metallic"].default_value = m["metallic"]
        mat.diffuse_color = (*m["albedo"], 1.0)
        mats[name] = mat
    return mats


def make_armature(bones):
    data = bpy.data.armatures.new("crew_figure_rig")
    rig = bpy.data.objects.new("crew_figure_rig", data)
    bpy.context.scene.collection.objects.link(rig)
    bpy.context.view_layer.objects.active = rig
    bpy.ops.object.mode_set(mode="EDIT")
    edit = {}
    for name in fl.BONE_NAMES:
        eb = data.edit_bones.new(name)
        eb.head, eb.tail = V(bones[name]["head"]), V(bones[name]["tail"])
        edit[name] = eb
    for name in fl.BONE_NAMES:
        if bones[name]["parent"]:
            edit[name].parent = edit[bones[name]["parent"]]
    bpy.ops.object.mode_set(mode="OBJECT")
    return rig


def suit_object(cfg, mats):
    """The suit hull (figure_body.suit_mesh): smooth weights (vertex groups), the atlas UV (UVMap), the wrinkle mask (UV `joint_mask`:
    x = joint slot + 1, 0 = none; glTF y = the mask) and the corrective shape keys. Blender UV v is flipped against glTF / the image rows."""
    m = fb.suit_mesh(cfg)
    h = m["hull"]
    aw, ah = cfg["suit_look"]["atlas_m"]
    mesh = bpy.data.meshes.new("suit_hull")
    mesh.from_pydata([tuple(p) for p in m["rest"]], [], [tuple(f) for f in h.faces])
    mesh.update()
    uv = mesh.uv_layers.new(name="UVMap")
    mask = mesh.uv_layers.new(name="joint_mask")
    li = 0
    for poly, corners in zip(mesh.polygons, h.uv):
        for k, loop_index in enumerate(poly.loop_indices):
            u, v = corners[k]
            uv.data[loop_index].uv = (u / aw, 1.0 - v / ah)
            slot, val = h.mask[mesh.loops[loop_index].vertex_index]
            mask.data[loop_index].uv = ((slot + 1) if slot >= 0 else 0.0, 1.0 - (val if slot >= 0 else 0.0))
            li += 1
    for poly in mesh.polygons:
        poly.use_smooth = True
    ob = bpy.data.objects.new("crew_figure", mesh)
    bpy.context.scene.collection.objects.link(ob)
    ob.data.materials.append(mats["suit"])
    groups = {n: ob.vertex_groups.new(name=n) for n in m["names"]}
    for i, wd in enumerate(h.w):
        for bone, wt in wd.items():
            groups[bone].add([i], wt, "REPLACE")
    ob.shape_key_add(name="Basis", from_mix=False)
    for name, d in m["shapes"].items():
        key = ob.shape_key_add(name=name, from_mix=False)
        co = m["rest"] + d
        key.data.foreach_set("co", co.astype("float32").ravel())
    return ob, len(h.triangles()), m


def assemble(cfg):
    """(rig object, mesh object, triangles): the skinned figure in the current scene."""
    bones, parts = build_figure(cfg)
    mats = make_materials(cfg)
    rig = make_armature(bones)
    suit, tris, _ = suit_object(cfg, mats)
    objs = []
    for (bone, material), mesh in sorted(parts.meshes.items()):
        ob, n = mesh.to_object(f"part_{bone}_{material}")
        tris += n
        ob.data.materials.append(mats[material])
        ob.vertex_groups.new(name=bone).add(list(range(len(ob.data.vertices))), 1.0, "REPLACE")
        objs.append(ob)
    # fix round 1: the rigid parts (helmet, visor, collar, gloves, boots) are their own mesh WITHOUT shape keys; only the suit carries the 16
    # corrective morph targets (a joined mesh exported them on every primitive: 10.5 MB .bin and morph work on the rigid vertices)
    bpy.ops.object.select_all(action="DESELECT")
    for ob in objs:
        ob.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.join()
    rigid_parts = bpy.context.view_layer.objects.active
    rigid_parts.name = PARTS_NODE
    rigid_parts.data.name = PARTS_NODE
    suit.name = "crew_figure"
    suit.data.name = "crew_figure"
    for ob in (suit, rigid_parts):
        ob.parent = rig
        mod = ob.modifiers.new("Armature", "ARMATURE")
        mod.object = rig
    return rig, suit, tris


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    vc.common.clear_scene()
    rig, fig, tris = assemble(spec["crew_figure"])
    vc.export_selected(Path(args["out"]) / "crew_figure.gltf", [rig])
    vc.report_triangles("crew_figure", tris)


if __name__ == "__main__":
    main()
