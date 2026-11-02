"""The IDA's docking interface, at both ports: idss_passive_ring.py -- --spec <json> --out <dir> writes iss_rings.gltf.

Own drawing. The HCS (flange, seals, hooks, receptacles, guide pins, separators) and the soft-capture ring with its petals are the IDSS IDD Rev E numbers of the
[idss] table in config/vehicles/detail/crew_dragon.toml, drawn by the same functions as Dragon's mechanism (idss_mechanism.py): the system is androgynous. The IDA is the passive
side: ring retracted by the IDD's recess, no actuators. The barrel, PMA and fittings come from [ida] of the ISS detail file. Frame: the station's stage frame (config), so the nodes
stand on the configured ports. Nodes per port n (2, 3): pma_n, ida_n (barrel), ida_n_band, ida_n_face (face, tunnel), ida_n_floor, ida_n_hcs (seals, passive hooks, receptacles, separators),
ida_n_pockets, ida_n_hooks (active hooks, open), ida_n_pins, ida_n_ring (ring, strikers), ida_n_petals, ida_n_fittings.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import idss_mechanism as mech  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402

LOCAL_UP = (0.0, 0.0, -1.0)  # the mechanism's drawing frame: X the port axis, up -Z, as on Dragon
LOCAL_AXIS = (1.0, 0.0, 0.0)
BARREL_SEGMENTS = 64  # [E] sagitta 1 mm at r 0.89
PMA_SEGMENTS = 48  # [E]
FITTING_SIDES = 8  # [E]


def port_matrix(port) -> Matrix:
    """Drawing frame (X axis, Y = axis x up, Z = -up) -> stage frame, about the port's point. A proper rotation: X x Y = Z."""
    x, up = Vector(port["axis"]).normalized(), Vector(port["up"]).normalized()
    y = x.cross(up)
    z = -up
    m = Matrix(((x.x, y.x, z.x, port["position_m"][0]), (x.y, y.y, z.y, port["position_m"][1]), (x.z, y.z, z.z, port["position_m"][2]), (0.0, 0.0, 0.0, 1.0)))
    return m


def phi_radial(phi_deg):
    """Radial direction of the port-frame angle phi (from y = up x axis towards up) in the drawing frame: y_port = -Y, up = -Z."""
    a = math.radians(phi_deg)
    return Vector((0.0, -math.cos(a), -math.sin(a)))


def phi_tangent(phi_deg):
    a = math.radians(phi_deg)
    return Vector((0.0, math.sin(a), -math.cos(a)))


def build_barrel(mesh, band, face, floor, d, ida):
    seg = BARREL_SEGMENTS
    r_t = 0.5 * d["tunnel_diameter_m"]
    r_f, r_b, depth = ida["face_outer_r_m"], ida["barrel_r_m"], ida["barrel_depth_m"]
    chamfer = r_b - r_f
    vc.revolve(face, [([(0.0, r_t), (0.0, r_f)], (1.0, 0.0, 0.0)), ([(0.0, r_t), (-ida["floor_depth_m"], r_t)], "in")], seg, 1.0, "face")
    vc.revolve(floor, [([(-ida["floor_depth_m"], r_t), (-ida["floor_depth_m"], 0.0)], (1.0, 0.0, 0.0))], seg, 1.0, "floor")
    b0, b1 = ida["band_depth_m"]
    vc.revolve(mesh, [([(0.0, r_f), (-chamfer, r_b)], "out"), ([(-chamfer, r_b), (-b1, r_b)], "out"), ([(-b0, r_b), (-depth, r_b)], "out"), ([(-depth, r_b), (-depth, 0.0)], (-1.0, 0.0, 0.0))], seg, r_b, "barrel")
    vc.revolve(band, [([(-b1, r_b), (-b0, r_b)], "out")], seg, r_b, "band")


def build_pma(mesh, ida, port, m):
    """A cone that leans towards the ring axis: rings from the table, centred on the line from the Harmony axis (s = 0, offset 0) to the ring axis; drawn in the drawing frame."""
    table = ida["pma_profile_m"]
    base = ida["pma_base_s_m"]
    offset_dir = port["pma_offset_dir"]  # in the drawing frame: from the ring axis towards the Harmony axis (the table's offset runs from Harmony to the ring)
    full = table[-1][2]
    prev = None
    for s, r, off in table:
        x = s - base
        centre = Vector((x, 0.0, 0.0)) + Vector(offset_dir) * (full - off)
        ring = [centre + vc.radial_dir(2.0 * math.pi * i / PMA_SEGMENTS) * r for i in range(PMA_SEGMENTS)]
        if prev is not None:
            for i in range(PMA_SEGMENTS):
                j = (i + 1) % PMA_SEGMENTS
                a, b = prev[i], prev[j]
                out = vc.radial_dir(2.0 * math.pi * (i + 0.5) / PMA_SEGMENTS)
                arc0, arc1 = i * 0.25, (i + 1) * 0.25
                mesh.face([a, b, ring[j], ring[i]], [(arc0, 1.0), (arc1, 1.0), (arc1, 0.0), (arc0, 0.0)], out, "pma")
        prev = ring


def build_fittings(mesh, ida):
    r_b = ida["barrel_r_m"]
    for phi, depth, r_end in ida["target_rods"]:
        e = phi_radial(phi)
        p0 = Vector((-depth, 0.0, 0.0)) + e * (r_b - 0.01)
        p1 = Vector((-depth, 0.0, 0.0)) + e * r_end
        vc.tube(mesh, p0, p1, 0.022, FITTING_SIDES, ("rod", phi), radius1=0.018)
        vc.tube(mesh, p1, p1 + e * 0.07, 0.045, FITTING_SIDES, ("ball", phi))
    phi, depth, r_end, width = ida["target_plate"]
    e, t = phi_radial(phi), phi_tangent(phi)
    c = Vector((-depth, 0.0, 0.0)) + e * (0.5 * (r_b + r_end))
    vc.box(mesh, c, e, t, Vector(LOCAL_AXIS), (0.5 * (r_end - r_b), 0.5 * width, 0.012))
    phi, depth, r_end, tang, axial = ida["fitting_box"]
    e, t = phi_radial(phi), phi_tangent(phi)
    c = Vector((-depth, 0.0, 0.0)) + e * (0.5 * (r_b + r_end))
    vc.box(mesh, c, e, t, Vector(LOCAL_AXIS), (0.5 * (r_end - r_b), 0.5 * tang, 0.5 * axial))
    for phi in ida["handrail_phi_deg"]:
        e, t = phi_radial(phi), phi_tangent(phi)
        mid = Vector((-ida["handrail_depth_m"], 0.0, 0.0)) + e * ida["handrail_r_m"]
        half = 0.5 * ida["handrail_length_m"]
        vc.tube(mesh, mid - Vector(LOCAL_AXIS) * half, mid + Vector(LOCAL_AXIS) * half, 0.014, 6, ("rail", phi))
        for sign in (-1.0, 1.0):
            foot = mid + Vector(LOCAL_AXIS) * sign * half
            vc.tube(mesh, foot, foot - e * (ida["handrail_r_m"] - r_b + 0.005), 0.012, 6, ("railfoot", phi, sign), cap1=False)


def active_hooks(mesh, d, up, axis):
    """The active hooks stand open on the passive side, ready for the mated pose; same outline and pivot as Dragon's (spec idss block)."""
    outline = [(u / 1000.0, v / 1000.0) for u, v in d["hook_active_mm"]]
    count = int(d["hook_count"])
    pivot_u, pivot_x = d["hook_pivot_m"]
    turn = math.radians(d["hook_turn_deg"])
    for k in range(count):
        az = mech.theta_azimuth(d["hook_first_theta_deg"] + 360.0 * k / count, up, axis)
        e_r, t = vc.radial_dir(az), mech.tangent(az)
        origin = e_r * d["hook_radius_m"] + t * pivot_u + Vector((pivot_x, 0.0, 0.0))

        def place(u, v, w):
            # the view's hook_transform: outline u along +Z, v along +X, turned about the width axis by `turn`, then about X to the azimuth
            uu = u * math.cos(turn) - v * math.sin(turn)
            vv = u * math.sin(turn) + v * math.cos(turn)
            return origin + t * uu + Vector((vv, 0.0, 0.0)) + e_r * w

        mech.extrude(mesh, outline, d["hook_width_m"], place)


def pins(mesh, d, up, axis):
    for theta in d["pin_theta_deg"]:
        az = mech.theta_azimuth(theta, up, axis)
        c = vc.radial_dir(az) * (0.5 * d["pin_circle_diameter_m"])
        r = 0.5 * d["pin_diameter_m"]
        shaft, height = d["pin_shaft_m"], d["pin_height_m"]
        vc.tube(mesh, c, c + Vector((shaft, 0.0, 0.0)), r, mech.PIN_SIDES, ("pin", theta), cap1=False)
        vc.tube(mesh, c + Vector((shaft, 0.0, 0.0)), c + Vector((height, 0.0, 0.0)), r, mech.PIN_SIDES, ("tip", theta), radius1=0.0005, cap0=False)


def petal_frame(mesh, d, ida, az, x_offset):
    """One passive petal: the IDD's 45 deg face (base and tip circles, half angles), drawn as model D and photo D_Iss065e002827 show it, a frame with lightening holes: rim, centre rib and
    a cross member. Bars have the plate's normal thickness; base plate along the ring. Drawn directly on azimuth az."""
    r0, r1 = 0.5 * d["petal_base_diameter_m"], 0.5 * d["petal_tip_diameter_m"]
    height = d["petal_height_m"]
    h0, h1 = math.radians(d["petal_base_half_deg"]), math.radians(d["petal_tip_half_deg"])
    length = math.hypot(height, r0 - r1)
    nx, nr = (r0 - r1) / length, height / length  # cone normal, inward and up
    th, bar = ida["petal_plate_m"], ida["petal_bar_m"]

    def p(s, f, off=0.0):
        """Point on the petal: s 0 base .. 1 tip, f -1 .. 1 across, off along the face normal."""
        r = r0 + (r1 - r0) * s
        half = h0 + (h1 - h0) * s
        a = az + half * f
        return Vector(((r0 - r) * height / (r0 - r1) + x_offset + off * nx, (r - off * nr) * math.cos(a), (r - off * nr) * math.sin(a)))

    def bar_between(a, b):
        """A flat bar of the frame from (s, f) to (s, f), full face width `bar` measured across."""
        pa, pb = p(*a), p(*b)
        along = (pb - pa).normalized()
        n = (p(a[0], a[1], 1.0) - pa).normalized()
        side = along.cross(n).normalized() * (0.5 * bar)
        t = n * th
        top = [pa - side, pa + side, pb + side, pb - side]
        under = [q - t for q in top]
        mesh.face(top, [(0, 0)] * 4, n)
        mesh.face(under, [(0, 0)] * 4, -n)
        for k in range(4):
            q0, q1 = top[k], top[(k + 1) % 4]
            mesh.face([q0, q1, q1 - t, q0 - t], [(0, 0)] * 4, (q0 + q1) - (pa + pb))

    steps = 4
    for side in (-1.0, 1.0):
        for k in range(steps):
            bar_between((k / steps, side), ((k + 1) / steps, side))  # the raking edges
    for k in range(steps * 2):
        f0, f1 = -1.0 + k / steps, -1.0 + (k + 1) / steps
        bar_between((0.0, f0), (0.0, f1))  # base
        bar_between((1.0, f0), (1.0, f1))  # tip
        bar_between((0.55, f0), (0.55, f1))  # cross member
    bar_between((0.0, 0.0), (1.0, 0.0))  # centre rib
    bar_between((0.0, -0.55), (0.55, 0.0))  # diagonals of the lower bay
    bar_between((0.0, 0.55), (0.55, 0.0))
    # rear rib and latch square of the IDD figure (as Dragon's drawing)
    foot_r, foot_d = d["petal_rib_foot_m"]
    e_r, tang = vc.radial_dir(az), mech.tangent(az)
    half = 0.5 * d["petal_thickness_m"]
    rib = [(r1, height), (r0, 0.0), (foot_r, -foot_d)]
    for side in (-1.0, 1.0):
        mesh.face([Vector((x + x_offset, 0.0, 0.0)) + e_r * r + tang * side * half for r, x in rib], [(0, 0)] * 3, tang * side)
    r_l, h_l = d["latch_box_m"]
    x_l = (r0 - r_l) * height / (r0 - r1) + x_offset
    up_in = Vector((nx, 0.0, 0.0)) - e_r * nr
    e_a = (Vector((nr, 0.0, 0.0)) + e_r * nx).normalized()
    vc.box(mesh, Vector((x_l, 0.0, 0.0)) + e_r * r_l + up_in * 0.004, tang, e_a, up_in, (h_l, h_l, 0.004))


def petals(mesh, d, ida, up, axis, x_offset):
    """The three passive petals at the IDD's angles."""
    for theta in d["petal_theta_deg"]:
        petal_frame(mesh, d, ida, mech.theta_azimuth(theta, up, axis), x_offset)


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    d, ida = spec["idss"], spec["detail"]["ida"]
    vc.common.clear_scene()
    root = vc.empty("iss_rings")
    tris = 0
    objects = []
    up, axis = LOCAL_UP, LOCAL_AXIS
    mech.spec_neck_outer = ida["face_outer_r_m"]
    for port in spec["ports"]:
        n = port["number"]
        m = port_matrix(port)
        parts = {name: vc.VMesh() for name in ("pma", "barrel", "band", "face", "floor", "hcs", "pockets", "hooks", "pins", "ring", "petals", "fittings")}
        flange = vc.VMesh()  # the mechanism's own flange and tunnel are replaced by the IDA's face and tunnel (build_barrel); only the metal and pockets remain
        mech.build_flange(flange, parts["hcs"], parts["pockets"], d, up, axis)
        flange.bm.free()
        active_hooks(parts["hooks"], d, up, axis)
        pins(parts["pins"], d, up, axis)
        mech.build_ring(parts["ring"], d, up, axis)
        parts["ring"].bm.transform(Matrix.Translation((-d["scs_recess_m"], 0.0, 0.0)))
        petals(parts["petals"], d, ida, up, axis, -d["scs_recess_m"])
        build_barrel(parts["barrel"], parts["band"], parts["face"], parts["floor"], d, ida)
        build_pma(parts["pma"], ida, port, m)
        build_fittings(parts["fittings"], ida)
        names = {"pma": f"pma_{n}", "barrel": f"ida_{n}", "band": f"ida_{n}_band", "face": f"ida_{n}_face", "floor": f"ida_{n}_floor", "hcs": f"ida_{n}_hcs", "pockets": f"ida_{n}_pockets",
                 "hooks": f"ida_{n}_hooks", "pins": f"ida_{n}_pins", "ring": f"ida_{n}_ring", "petals": f"ida_{n}_petals", "fittings": f"ida_{n}_fittings"}
        for key, mesh in parts.items():
            mesh.bm.transform(m)
            mesh.bm.normal_update()
            if m.determinant() < 0.0:
                raise ValueError("port frame not proper")
            tris += mesh.to_object(names[key], root)[1]
    vc.export_selected(Path(args["out"]) / "iss_rings.gltf", [root])
    vc.report_triangles("iss_rings", tris)


if __name__ == "__main__":
    main()
