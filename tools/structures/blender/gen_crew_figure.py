"""Crew figure in the newer SpaceX EVA suit: gen_crew_figure.py -- --spec <json> --out <dir> writes crew_figure.gltf.

Own rebuild from config/cockpits/crew_figure.toml (spec["crew_figure"]); no third-party mesh. Frame: origin = the seat's hip reference point,
X = up the seat back, Y = the seat's chest normal (forward), Z = the crew's left (figure_layout.py, crew_figure.toml header). Rest pose = seated.
One skinned mesh node `crew_figure` (rigid weights, one bone per segment) under the armature `crew_figure_rig`; surfaces (glTF primitives) =
the materials suit, boots, gloves, helmet, visor (the closed copper visor is its own surface). Silhouette only: stripes, seams, patches,
the flag by texture later. The bones are figure_layout.bones(cfg).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402
import figure_layout as fl  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

V = Vector
SIDES = 14  # [E] sides of limb sections
TORSO_SIDES = 20  # [E] sides of the torso / helmet sections
CAP_STEPS = 3  # [E] rings of a rounded end
HELMET_BOTTOM_T = -0.75  # [E] the helmet shell is cut flat at this fraction of its height below the centre (the neck ring closes it)
HELMET_RINGS = 14  # [E] rings of the helmet shell
VISOR_COLS, VISOR_ROWS = 14, 8  # [E] grid of the visor patch


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

def build_torso(cfg, parts, bones):
    fig, ln = cfg["figure"], fl.chain_lengths(cfg)
    rings_cfg = cfg["suit"]["torso_rings_m"]
    origin = V((fig["hip_joint_up_m"], fig["hip_joint_forward_m"], 0.0))
    splits = [rings_cfg[0][0], fl.SPINE_SPLIT_M[0], fl.SPINE_SPLIT_M[1], rings_cfg[-1][0]]
    for bone, s0, s1 in (("hips", splits[0], splits[1]), ("spine", splits[1], splits[2]), ("chest", splits[2], splits[3])):
        ss = sorted({s0, s1, *[r[0] for r in rings_cfg if s0 < r[0] < s1]})
        rings = []
        for s in ss:
            hz, hn, fwd = blend(rings_cfg, s)
            rings.append((s, hn, hz, fwd, 0.0))
        loft(parts.mesh(bone, "suit"), origin, (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), rings, TORSO_SIDES, ("torso", bone), cap0=(bone == "hips"), cap1=(bone == "chest"))


def build_helmet(cfg, parts):
    h, pose = cfg["helmet"], cfg["pose"]
    c = V((pose["head_centre_up_m"], pose["head_centre_forward_m"], 0.0))
    # ellipsoid shell about the centre: axis up (x), u = depth (y), v = width (z)
    ts = [HELMET_BOTTOM_T + (1.0 - HELMET_BOTTOM_T) * i / (HELMET_RINGS - 1) for i in range(HELMET_RINGS)]
    rings = []
    for t in ts:
        r = math.sqrt(max(0.0, 1.0 - t * t))
        rings.append((t * (h["half_above_m"] if t > 0 else h["half_below_m"]), h["half_depth_m"] * r, h["half_width_m"] * r, 0.0, 0.0))
    loft(parts.mesh("head", "helmet"), c, (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), rings, TORSO_SIDES, "helmet", cap0=True)
    # collar: a wider ring under the helmet shell (eva_suit_detail2), slightly flared at the top where the helmet's lower rim sits on it
    top = c.x + HELMET_BOTTOM_T * h["half_below_m"] + 0.02
    cr, ch, nr = h["collar_radius_m"], h["collar_height_m"], cfg["suit"]["neck_radius_m"]
    loft(parts.mesh("neck", "helmet"), V((top - ch, c.y, 0.0)), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
         [(0.0, nr * 1.15, nr * 1.15, 0.0, 0.0), (0.2 * ch, cr * 0.94, cr * 0.94, 0.0, 0.0), (0.7 * ch, cr, cr, 0.0, 0.0), (ch, cr * 1.04, cr * 1.04, 0.0, 0.0)],
         SIDES, "neck_ring", True, True)
    build_visor(cfg, parts, c)


def ellipsoid_point(h, c, el_deg, az_deg):
    """Point and outward unit normal on the helmet ellipsoid at elevation / azimuth (az 0 = forward, positive toward the figure's left)."""
    e, a = math.radians(el_deg), math.radians(az_deg)
    d = V((math.sin(e), math.cos(e) * math.cos(a), math.cos(e) * math.sin(a)))
    rx = h["half_above_m"] if d.x > 0 else h["half_below_m"]
    radii = V((rx, h["half_depth_m"], h["half_width_m"]))
    p = c + V((radii.x * d.x, radii.y * d.y, radii.z * d.z))
    n = V((d.x / radii.x, d.y / radii.y, d.z / radii.z)).normalized()
    return p, n


def visor_bulge(h, i, j):
    """Height of the visor's outer surface over the shell at grid node (i, j): the dome (visor_proud_m) at the centre, visor_edge_m on the whole rim (a smooth bulge)."""
    a = 2.0 * j / VISOR_COLS - 1.0
    b = 2.0 * i / VISOR_ROWS - 1.0
    w = (1.0 - a * a) * (1.0 - b * b)
    return h["visor_edge_m"] + (h["visor_proud_m"] - h["visor_edge_m"]) * w


def build_visor(cfg, parts, c):
    """The closed visor: a forward-domed outer and an inner patch over the helmet ellipsoid and a rim, one solid (own surface), plus the white rim ring (helmet surface)."""
    h = cfg["helmet"]
    mesh = parts.mesh("head", "visor")
    outer, inner, shell_pts = {}, {}, {}
    for i in range(VISOR_ROWS + 1):
        el = h["visor_down_deg"] + (h["visor_up_deg"] - h["visor_down_deg"]) * i / VISOR_ROWS
        for j in range(VISOR_COLS + 1):
            az = -h["visor_half_angle_deg"] + 2.0 * h["visor_half_angle_deg"] * j / VISOR_COLS
            p, n = ellipsoid_point(h, c, el, az)
            bulge = visor_bulge(h, i, j)
            outer[i, j] = p + n * bulge
            inner[i, j] = p + n * (bulge - h["visor_thickness_m"])
            shell_pts[i, j] = p + n * 0.5 * (h["visor_edge_m"] - h["visor_thickness_m"])
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
    # the rim ring: a round bar along the visor's boundary (the white brow band of detail2), in the helmet material
    loop = ([(0, j) for j in range(VISOR_COLS + 1)] + [(i, VISOR_COLS) for i in range(1, VISOR_ROWS + 1)]
            + [(VISOR_ROWS, j) for j in range(VISOR_COLS - 1, -1, -1)] + [(i, 0) for i in range(VISOR_ROWS - 1, 0, -1)])
    rim = parts.mesh("head", "helmet")
    for k, key in enumerate(loop):
        nxt = loop[(k + 1) % len(loop)]
        vc.tube(rim, shell_pts[key], shell_pts[nxt], h["rim_radius_m"], 8, ("rim", k), cap0=False, cap1=False)


def build_arm(cfg, parts, bones, side):
    suit, hand = cfg["suit"], cfg["hand"]
    b = {n: bones[f"{n}_{side}"] for n in ("shoulder", "upper_arm", "forearm", "hand", "index_1", "index_2", "index_3", "thumb_1", "thumb_2", "fingers")}
    ua, fa = b["upper_arm"], b["forearm"]
    limb(parts.mesh(f"upper_arm_{side}", "suit"), ua["head"], ua["tail"], suit["upper_arm_radius_m"][0], suit["upper_arm_radius_m"][1], f"ua_{side}")
    limb(parts.mesh(f"forearm_{side}", "suit"), fa["head"], fa["tail"], suit["forearm_radius_m"][0], suit["forearm_radius_m"][1], f"fa_{side}", cap1=False, flat1=True)
    f, d, t = fl.hand_frame(cfg, side)
    wrist = V(b["hand"]["head"])
    cuff_axis = V(f)
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
        limb(parts.mesh(f"index_{k}_{side}", "gloves"), bone["head"], bone["tail"], r, r * 0.92, f"index{k}_{side}", 10, uref=tuple(t))
    fb = b["fingers"]
    fh, ft = V(fb["head"]), V(fb["tail"])
    loft(parts.mesh(f"fingers_{side}", "gloves"), fh, ft - fh, t,
         capsule_rings((ft - fh).length, hand["fingers_half_width_m"], hand["fingers_radius_m"], hand["fingers_half_width_m"] * 0.9, hand["fingers_radius_m"] * 0.9, False, True),
         SIDES, f"fingers_{side}", cap0=True)
    for k in (1, 2):
        bone = b[f"thumb_{k}"]
        r = hand["thumb_radius_m"] * (1.0 - 0.12 * (k - 1))
        limb(parts.mesh(f"thumb_{k}_{side}", "gloves"), bone["head"], bone["tail"], r, r * 0.9, f"thumb{k}_{side}", 10, uref=tuple(d))


def build_leg(cfg, parts, bones, side):
    suit = cfg["suit"]
    th, sh, ft = bones[f"thigh_{side}"], bones[f"shin_{side}"], bones[f"foot_{side}"]
    limb(parts.mesh(f"thigh_{side}", "suit"), th["head"], th["tail"], suit["thigh_radius_m"][0], suit["thigh_radius_m"][1], f"th_{side}")
    knee, ankle = V(sh["head"]), V(sh["tail"])
    shaft_top = ankle + (knee - ankle) * suit["boot_shaft_fraction"]
    r_top = suit["shin_radius_m"][0] + (suit["shin_radius_m"][1] - suit["shin_radius_m"][0]) * (1.0 - suit["boot_shaft_fraction"])
    limb(parts.mesh(f"shin_{side}", "suit"), knee, shaft_top, suit["shin_radius_m"][0], r_top, f"sh_{side}", cap1=False, flat1=True)
    limb(parts.mesh(f"shin_{side}", "boots"), shaft_top - (shaft_top - ankle).normalized() * 0.0, ankle, r_top + 0.004, suit["shin_radius_m"][1] + 0.004, f"boot_{side}", cap0=False, cap1=False, flat0=True)
    # foot: along the foot direction from the heel; sections (s from the heel, half width, half height, centre height above the foot line)
    fdir = V(ft["tail"]) - V(ft["head"])
    fdir.normalize()
    heel = V(ft["head"]) - fdir * suit["boot_heel_m"]
    hw, hh = suit["boot_foot_half_width_m"], suit["boot_foot_half_height_m"]
    total = cfg["proportion"]["foot_length"] * cfg["figure"]["stature_m"]
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
    build_torso(cfg, parts, bones)
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


def assemble(cfg):
    """(rig object, mesh object, triangles): the skinned figure in the current scene."""
    bones, parts = build_figure(cfg)
    mats = make_materials(cfg)
    rig = make_armature(bones)
    objs, tris = [], 0
    for (bone, material), mesh in sorted(parts.meshes.items()):
        ob, n = mesh.to_object(f"part_{bone}_{material}")
        tris += n
        ob.data.materials.append(mats[material])
        ob.vertex_groups.new(name=bone).add(list(range(len(ob.data.vertices))), 1.0, "REPLACE")
        objs.append(ob)
    bpy.ops.object.select_all(action="DESELECT")
    for ob in objs:
        ob.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.join()
    fig = bpy.context.view_layer.objects.active
    fig.name = "crew_figure"
    fig.data.name = "crew_figure"
    fig.parent = rig
    mod = fig.modifiers.new("Armature", "ARMATURE")
    mod.object = rig
    return rig, fig, tris


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    vc.common.clear_scene()
    rig, fig, tris = assemble(spec["crew_figure"])
    vc.export_selected(Path(args["out"]) / "crew_figure.gltf", [rig])
    vc.report_triangles("crew_figure", tris)


if __name__ == "__main__":
    main()
