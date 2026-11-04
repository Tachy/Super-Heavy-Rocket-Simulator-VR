"""HLS legs, airlock hatch, elevator track and basket as primitive lists, pure Python (no Blender): ('tube', p0, p1, r0, r1) and
('box', centre, ex, ey, ez, half), every point in the stage's body frame (station X, nose +X, (Y, Z) = r (cos a, sin a)).

Numbers: config/vehicles/detail/starship_hls.toml [landing_leg] / [airlock] / [elevator] (drawing, [E]); the leg placement is the config's
[stage.landing_legs] (azimuths, footprint radius, foot station, stroke), unchanged (user ruling 2026-11-04).
The foot's reference point is the CENTRE OF THE PAD'S BOTTOM FACE = the core's foot point (station foot_station, radius footprint, azimuth): the pad
node `foot_<i>` is posed by translating it along +X (the stroke axis = the station axis, core contact model) by the leg's compression.
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


def foot_nodes(legs):
    return [{"index": i, "leg_node": f"leg_{i}", "foot_node": f"foot_{i}", "azimuth_deg": a, "azimuth_rad": math.radians(a)}
            for i, a in enumerate(legs["azimuths_deg"])]


def foot_prims(d, legs, a_rad):
    """Pad (frustum, bottom face on the foot point) and the piston stem up to inside the sleeve. Moves with the compression."""
    f = foot_point(legs, a_rad)
    top = _add(f, (d["pad_height_m"], 0.0, 0.0))
    stem_top = _add(f, (d["pad_height_m"] + d["sleeve_gap_m"] + d["stem_inside_m"], 0.0, 0.0))
    return [("tube", f, top, d["pad_radius_m"], 0.75 * d["pad_radius_m"]),
            ("tube", top, stem_top, d["stem_radius_m"], d["stem_radius_m"])]


def sleeve_span(d, legs):
    """(bottom station, top station) of the vertical sleeve at the foot: its bottom a `sleeve_gap_m` above the pad (more than the stroke)."""
    bottom = legs["foot_station_m"] + d["pad_height_m"] + d["sleeve_gap_m"]
    return bottom, bottom + d["sleeve_length_m"]


def leg_prims(d, legs, hull_radius_m, a_rad):
    """Sleeve at the foot, main strut and brace to the hull (in the leg's radial plane), a saddle on the hull at each end."""
    bottom, top = sleeve_span(d, legs)
    r_foot = legs["footprint_radius_m"]
    sleeve0, sleeve1 = point(bottom, a_rad, r_foot), point(top, a_rad, r_foot)
    embed = d["attach_embed_m"]
    hull_main = point(d["strut_attach_station_m"], a_rad, hull_radius_m - embed)
    hull_brace = point(d["brace_attach_station_m"], a_rad, hull_radius_m - embed)
    sleeve_mid = point(0.5 * (bottom + top), a_rad, r_foot)
    prims = [("tube", sleeve0, sleeve1, d["sleeve_radius_m"], d["sleeve_radius_m"]),
             ("tube", sleeve1, hull_main, d["sleeve_radius_m"] * 0.9, d["strut_radius_m"]),
             ("tube", sleeve_mid, hull_brace, d["brace_radius_m"], d["brace_radius_m"])]
    et, er, ex = tangent(a_rad), radial(a_rad), (1.0, 0.0, 0.0)
    for st, half_station in ((d["strut_attach_station_m"], 0.5 * d["saddle_height_m"]), (d["brace_attach_station_m"], 0.25 * d["saddle_height_m"])):
        centre = point(st, a_rad, hull_radius_m + 0.5 * d["saddle_thickness_m"] - embed)
        prims.append(("box", centre, ex, er, et, (half_station, 0.5 * d["saddle_thickness_m"] + embed, 0.5 * d["saddle_width_m"])))
    return prims


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
