"""HLS legs, airlock hatch, elevator track and basket as primitive lists, pure Python (no Blender): ('tube', p0, p1, r0, r1) and
('box', centre, ex, ey, ez, half), every point in the stage's body frame (station X, nose +X, (Y, Z) = r (cos a, sin a)).

Numbers: config/vehicles/detail/starship_hls.toml [landing_leg] / [airlock] / [elevator] (drawing, [E]); the leg placement and its kinematics
are the config's [stage.landing_legs] (azimuths, footprint radius, foot station, stroke, hinge_station_m, hinge_radius_m, knuckle_height_m).

Deployable legs (Task 6b, user 2026-11-04, like Falcon 9's): every node is DRAWN DEPLOYED and posed by the deployment d only (leg_rig, pose_point;
the same kinematics as the core's contact::leg_pose and the view's VehicleModels.hls_leg_pose). In the leg's radial plane:
  `leg_<i>`    the rigid tube hinge H -> knuckle K; turns about H by (1 - d) x (0 - phi): stowed along the hull, tip up.
  `shock_<i>`  the knuckle sleeve (vertical when deployed); with the leg, then about K by (1 - d) x (phi - 180 deg): stowed in the leg's line.
  `stem_<i>`   the piston stem: as the sleeve, slid along its +X (deployed: the station axis) by the leg's compression.
  `foot_<i>`   the pad (+ its ball-joint hub): as the stem, then about the pad pivot Q by (1 - d) x 90 deg: stowed flat over the folded stem.
  `pusher_<i>` drawn along +X with unit length; the view stretches it from its fixed hull anchor to its point on the leg (as Falcon 9's).
  `leg_mount_<i>` hinge clevis and pusher anchor bracket, fixed on the hull.
At d = 1 every pose is the identity: the drawn pad's BOTTOM FACE CENTRE is the core's foot point (station foot_station, radius footprint, azimuth).
"""
import math


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def radial(a_rad):
    return (0.0, math.cos(a_rad), math.sin(a_rad))


def tangent(a_rad):
    return (0.0, -math.sin(a_rad), math.cos(a_rad))


def point(station, a_rad, r):
    return (station, r * math.cos(a_rad), r * math.sin(a_rad))


def foot_point(legs, a_rad):
    return point(legs["foot_station_m"], a_rad, legs["footprint_radius_m"])


NODE_KINDS = ("leg", "shock", "stem", "foot")


def foot_nodes(legs):
    return [{"index": i, "leg_node": f"leg_{i}", "shock_node": f"shock_{i}", "stem_node": f"stem_{i}", "foot_node": f"foot_{i}",
             "pusher_node": f"pusher_{i}", "mount_node": f"leg_mount_{i}", "azimuth_deg": a, "azimuth_rad": math.radians(a)}
            for i, a in enumerate(legs["azimuths_deg"])]


def sleeve_gap_m(d, legs):
    """Free stem between the pad's top and the sleeve: what the config's knuckle height leaves for it (must exceed the stroke)."""
    return legs["knuckle_height_m"] - d["pad_height_m"] - 0.5 * d["sleeve_length_m"]


def knuckle_station_m(legs):
    return legs["foot_station_m"] + legs["knuckle_height_m"]


def leg_rig(d, legs):
    """The rig in the leg's radial plane (station s, radius r): hinge, knuckle, the leg's length and deployed angle phi (from +X towards the
    radial), the foot's and the pad's fold angles and the pad pivot, the pusher's anchor and point, and the stowed thickness that follows
    from the folded geometry (the pad's rim: it lies on the sleeve's outer face + a gap, its bottom face outwards)."""
    hs, hr, R = legs["hinge_station_m"], legs["hinge_radius_m"], legs["footprint_radius_m"]
    ks = knuckle_station_m(legs)
    phi = math.degrees(math.atan2(R - hr, ks - hs))
    h = 0.5 * (d["sleeve_radius_m"] + d["pad_pivot_gap_m"])  # pivot offset: stowed the pad's top face lies 2 h off the leg's axis
    pad_top_s = legs["foot_station_m"] + d["pad_height_m"]
    pad_rim_r = math.hypot(hr + 2.0 * h + d["pad_height_m"], d["pad_radius_m"])
    return {"hinge_station_m": hs, "hinge_radius_m": hr, "knuckle_station_m": ks, "knuckle_radius_m": R,
            "length_m": math.hypot(ks - hs, R - hr), "deployed_angle_deg": phi, "stowed_angle_deg": 0.0,
            "foot_fold_deg": phi - 180.0, "pad_fold_deg": 90.0, "pad_pivot_station_m": pad_top_s + h, "pad_pivot_radius_m": R - h,
            "pusher_anchor_station_m": d["pusher_anchor_station_m"], "pusher_anchor_radius_m": d["pusher_anchor_radius_m"],
            "pusher_leg_u_m": d["pusher_leg_u_m"], "stowed_thickness_m": pad_rim_r - legs["hull_radius_m"]}


def _turn(p, a_rad, pivot_s, pivot_r, angle_deg):
    """`p` turned about the tangent axis through (pivot_s, pivot_r) at azimuth `a_rad` (a positive angle turns +X towards the radial)."""
    er, et = radial(a_rad), tangent(a_rad)
    s, r = p[0] - pivot_s, p[1] * er[1] + p[2] * er[2] - pivot_r
    t = p[1] * et[1] + p[2] * et[2]
    c, n = math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))
    return _add(_add((pivot_s + s * c - r * n, 0.0, 0.0), _mul(er, pivot_r + s * n + r * c)), _mul(et, t))


def pose_point(rig, a_rad, deployment, p, kind="leg", compression_m=0.0):
    """A body-frame point of the deployed drawing of a `kind` node ("leg", "shock", "stem", "foot") in the pose of `deployment` (0 stowed ..
    1 deployed) and the leg's compression (stem and foot slide along the deployed +X). Innermost first: pad fold, slide, foot fold, leg turn."""
    u = 1.0 - min(max(deployment, 0.0), 1.0)
    level = NODE_KINDS.index(kind)
    if level >= 3:
        p = _turn(p, a_rad, rig["pad_pivot_station_m"], rig["pad_pivot_radius_m"], u * rig["pad_fold_deg"])
    if level >= 2:
        p = _add(p, (compression_m, 0.0, 0.0))
    if level >= 1:
        p = _turn(p, a_rad, rig["knuckle_station_m"], rig["knuckle_radius_m"], u * rig["foot_fold_deg"])
    return _turn(p, a_rad, rig["hinge_station_m"], rig["hinge_radius_m"], u * (rig["stowed_angle_deg"] - rig["deployed_angle_deg"]))


def pusher_ends(rig, a_rad, deployment):
    """(anchor, point on the leg) of the pusher in the pose of `deployment`."""
    anchor = point(rig["pusher_anchor_station_m"], a_rad, rig["pusher_anchor_radius_m"])
    a = math.radians(rig["deployed_angle_deg"])
    on_leg = point(rig["hinge_station_m"] + rig["pusher_leg_u_m"] * math.cos(a), a_rad, rig["hinge_radius_m"] + rig["pusher_leg_u_m"] * math.sin(a))
    return anchor, pose_point(rig, a_rad, deployment, on_leg)


def foot_prims(d, legs, a_rad):
    """The pad (frustum, bottom face on the foot point) and the hub of its ball joint (pad top -> pivot)."""
    rig = leg_rig(d, legs)
    f = foot_point(legs, a_rad)
    top = _add(f, (d["pad_height_m"], 0.0, 0.0))
    pivot = point(rig["pad_pivot_station_m"], a_rad, rig["pad_pivot_radius_m"])
    return [("tube", f, top, d["pad_radius_m"], 0.75 * d["pad_radius_m"]),
            ("tube", top, pivot, d["hub_radius_m"], d["hub_radius_m"])]


def stem_prims(d, legs, a_rad):
    """The piston stem from the pad's top up into the knuckle sleeve."""
    f = foot_point(legs, a_rad)
    return [("tube", _add(f, (d["pad_height_m"], 0.0, 0.0)), _add(f, (d["pad_height_m"] + sleeve_gap_m(d, legs) + d["stem_inside_m"], 0.0, 0.0)),
             d["stem_radius_m"], d["stem_radius_m"])]


def sleeve_span(d, legs):
    """(bottom station, top station) of the knuckle sleeve: centred on the knuckle, its bottom the free stem above the pad (more than the stroke)."""
    k = knuckle_station_m(legs)
    return k - 0.5 * d["sleeve_length_m"], k + 0.5 * d["sleeve_length_m"]


def shock_prims(d, legs, a_rad):
    """The knuckle sleeve the stem slides in (vertical, deployed)."""
    bottom, top = sleeve_span(d, legs)
    r_foot = legs["footprint_radius_m"]
    return [("tube", point(bottom, a_rad, r_foot), point(top, a_rad, r_foot), d["sleeve_radius_m"], d["sleeve_radius_m"])]


def leg_prims(d, legs, a_rad):
    """The rigid leg, deployed: the tube from the hinge to the knuckle."""
    hinge = point(legs["hinge_station_m"], a_rad, legs["hinge_radius_m"])
    knuckle = point(knuckle_station_m(legs), a_rad, legs["footprint_radius_m"])
    return [("tube", hinge, knuckle, d["leg_root_radius_m"], d["leg_tip_radius_m"])]


def mount_prims(d, legs, hull_radius_m, a_rad):
    """Fixed on the hull: the hinge clevis under the leg's root and the pusher's anchor bracket (both from inside the skin to the axis radius)."""
    et, er, ex = tangent(a_rad), radial(a_rad), (1.0, 0.0, 0.0)
    embed = d["attach_embed_m"]
    prims = []
    for st, r_axis, half_station, half_width in ((legs["hinge_station_m"], legs["hinge_radius_m"], 0.5 * d["clevis_height_m"], 0.5 * d["clevis_width_m"]),
                                                 (d["pusher_anchor_station_m"], d["pusher_anchor_radius_m"], 0.5 * d["anchor_size_m"], 0.5 * d["anchor_size_m"])):
        r0 = hull_radius_m - embed
        centre = point(st, a_rad, 0.5 * (r0 + r_axis))
        prims.append(("box", centre, ex, er, et, (half_station, 0.5 * (r_axis - r0), half_width)))
    return prims


def pusher_prims(d):
    """The pusher in its own frame: a tube along +X from 0 to 1 m (the view stretches +X to the anchor-to-leg distance)."""
    return [("tube", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), d["pusher_radius_m"], d["pusher_radius_m"])]


def hatch_prims(h, hull_radius_m):
    """Airlock hatch: a door leaf and a raised frame on the hull at (station_from..to, azimuth)."""
    a = math.radians(h["azimuth_deg"])
    ex, er, et = (1.0, 0.0, 0.0), radial(a), tangent(a)
    c0 = h["sill_station_m"] + 0.5 * h["height_m"]
    r = hull_radius_m + 0.5 * h["frame_depth_m"]
    t = h["frame_width_m"]
    prims = [("box", point(c0, a, r + 0.25 * h["frame_depth_m"]), ex, er, et,
              (0.5 * h["height_m"] - t, 0.25 * h["frame_depth_m"], 0.5 * h["width_m"] - t))]  # the leaf, set in the frame
    prims += [("box", point(c0 + s * (0.5 * h["height_m"] - 0.5 * t), a, r), ex, er, et, (0.5 * t, 0.5 * h["frame_depth_m"], 0.5 * h["width_m"])) for s in (-1, 1)]
    for s in (-1, 1):
        centre = _add(point(c0, a, r), _mul(et, s * (0.5 * h["width_m"] - 0.5 * t)))
        prims.append(("box", centre, ex, er, et, (0.5 * h["height_m"], 0.5 * h["frame_depth_m"], 0.5 * t)))
    return prims


def track_prims(e, hull_radius_m):
    """Two rails and a cable down the hull from the hatch's sill to the lower end, on the hull."""
    a = math.radians(e["azimuth_deg"])
    ex, er, et = (1.0, 0.0, 0.0), radial(a), tangent(a)
    s0, s1 = e["track_from_station_m"], e["track_to_station_m"]
    c, half = 0.5 * (s0 + s1), 0.5 * abs(s0 - s1)
    prims = []
    for s in (-1, 1):
        centre = _add(point(c, a, hull_radius_m + e["rail_stand_off_m"] + 0.5 * e["rail_depth_m"]), _mul(et, s * 0.5 * e["rail_gauge_m"]))
        prims.append(("box", centre, ex, er, et, (half, 0.5 * e["rail_depth_m"], 0.5 * e["rail_width_m"])))
        for k in range(int(e["bracket_count"])):  # brackets that hold the rail off the hull
            st = s1 + (s0 - s1) * (k + 0.5) / e["bracket_count"]
            b = _add(point(st, a, hull_radius_m + 0.5 * e["rail_stand_off_m"]), _mul(et, s * 0.5 * e["rail_gauge_m"]))
            prims.append(("box", b, ex, er, et, (0.06, 0.5 * e["rail_stand_off_m"] + 0.02, 0.5 * e["rail_width_m"])))
    prims.append(("tube", point(s0, a, hull_radius_m + e["rail_stand_off_m"] + e["rail_depth_m"] + e["cable_radius_m"]),
                  point(s1, a, hull_radius_m + e["rail_stand_off_m"] + e["rail_depth_m"] + e["cable_radius_m"]), e["cable_radius_m"], e["cable_radius_m"]))
    return prims


def basket_prims(e, hull_radius_m):
    """The basket at its fixed pose at the top of the track: floor, four posts, top frame, wire-mesh walls (rails and bars)."""
    a = math.radians(e["azimuth_deg"])
    ex, er, et = (1.0, 0.0, 0.0), radial(a), tangent(a)
    w, dep, hgt = e["basket_width_m"], e["basket_depth_m"], e["basket_height_m"]
    base = e["basket_bottom_station_m"]
    r0 = hull_radius_m + e["basket_clearance_m"]  # the basket's hull-side face
    rc = r0 + 0.5 * dep

    def at(st, dr, dt):
        return _add(point(st, a, rc), _add(_mul(er, dr), _mul(et, dt)))

    f = e["frame_m"]
    prims = [("box", at(base + 0.5 * f, 0.0, 0.0), ex, er, et, (0.5 * f, 0.5 * dep, 0.5 * w))]  # floor
    for sr in (-1, 1):
        for stt in (-1, 1):
            prims.append(("box", at(base + 0.5 * hgt, sr * (0.5 * dep - 0.5 * f), stt * (0.5 * w - 0.5 * f)), ex, er, et, (0.5 * hgt, 0.5 * f, 0.5 * f)))
    wall_h = e["wall_height_m"]
    for sr in (-1, 1):  # top frame (front and back rails) and the lower and upper wall rails
        for st in (base + hgt - 0.5 * f, base + wall_h - 0.5 * f, base + 0.5 * f + 0.0):
            prims.append(("box", at(st, sr * (0.5 * dep - 0.5 * f), 0.0), ex, er, et, (0.5 * f, 0.5 * f, 0.5 * w)))
    for stt in (-1, 1):
        for st in (base + hgt - 0.5 * f, base + wall_h - 0.5 * f):
            prims.append(("box", at(st, 0.0, stt * (0.5 * w - 0.5 * f)), ex, er, et, (0.5 * f, 0.5 * dep, 0.5 * f)))
    bars = int(e["mesh_bars"])
    bar = e["mesh_bar_m"]
    for sr in (-1, 1):  # vertical bars of the mesh on the two broad walls (the wire mesh itself is too fine to draw: bars at its coarse pitch)
        for k in range(1, bars):
            dt = -0.5 * w + w * k / bars
            prims.append(("box", at(base + 0.5 * wall_h, sr * (0.5 * dep - 0.5 * bar), dt), ex, er, et, (0.5 * wall_h, 0.5 * bar, 0.5 * bar)))
    return prims
