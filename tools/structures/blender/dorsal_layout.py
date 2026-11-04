"""Dorsal transfer hardware of a ship (tanker probes, depot drogues, connectors): pure-Python layout, no Blender.

Used by gen_dorsal_hardware.py (the mesh), by vehicle_spec / build_models (the manifest block) and by the tests. The numbers come from
the vehicle config (elements, positions, axes) and the `[dorsal]` table of the detail file.

Probe node: origin = the extended TIP (root + cone_depth + extension along the axis, see probe_offsets_m); the config element is the ROOT.
Local part frame: +X is the interface normal (the port's `axis`, out of the vehicle), the part is a body of revolution about it, profiles are
(x, r). Origin: probe = the extended TIP (the config element); drogue = the centre of its MOUTH plane (the config element; the funnel narrows towards -X); connector = the centre of the coupling face (the config position). The generator rotates +X onto the port axis and leaves the
origin, so a game node only translates (probe: by the stroke along the axis).
"""
import math

SKIN_FLAT = (1.0, 0.0, 0.0)  # axial hint: faces whose normal is +X in the local frame


def element_nodes(ports):
    """One entry per element over all ports, in order: node name, kind, port id, element, axis. `probe` for an active port, `drogue` for a passive one."""
    out = []
    for port in ports:
        kind = "probe" if port["soft_capture"] == "active" else "drogue"
        for element in port["elements_m"]:
            out.append({"node": f"{kind}_{len(out)}", "kind": kind, "port": port["id"], "element_m": list(element), "axis": list(port["axis"])})
    return out


def connector_nodes(connectors, axis):
    """Node name `connector_<id without the _transfer suffix>` and the face position per connector."""
    return [{"node": "connector_" + c["id"].removesuffix("_transfer"), "id": c["id"], "position_m": list(c["position_m"]),
             "line_diameter_m": c["line_diameter_m"], "axis": list(axis)} for c in connectors]


def hull_distance_m(hull_radius_m, point_m, axis):
    """Distance t >= 0 from `point_m` against `axis` (t > 0 goes into the hull) to the barrel cylinder about the station axis X."""
    py, pz = point_m[1], point_m[2]
    ay, az = axis[1], axis[2]
    a = ay * ay + az * az
    b = -2.0 * (py * ay + pz * az)
    c = py * py + pz * pz - hull_radius_m ** 2
    if a < 1e-12:
        raise ValueError("axis parallel to the station axis")
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        raise ValueError("axis line misses the barrel")
    return (-b - math.sqrt(disc)) / (2.0 * a)  # the nearer crossing, outside -> inside


def axial_m(point_m, axis):
    return sum(p * a for p, a in zip(point_m, axis))


def probe_profile(d):
    """Tip at x = 0, body towards -x: blunt tip, cone, collar, shaft. Returns [(x, r)] from the tip to the foot."""
    head = d["probe_head_length_m"]
    collar = d["probe_head_collar_m"]
    return [(0.0, d["probe_tip_flat_radius_m"]), (-head, d["probe_head_radius_m"]), (-head - collar, d["probe_head_radius_m"]),
            (-head - collar, d["probe_shaft_radius_m"]), (-d["probe_shaft_length_m"], d["probe_shaft_radius_m"])]


def probe_runs(d):
    prof = probe_profile(d)
    tip_r = prof[0][1]
    foot = prof[-1]
    return [([(0.0, 0.0), (0.0, tip_r)], SKIN_FLAT),
            ([prof[0], prof[1]], "out"), ([prof[1], prof[2]], "out"),
            ([prof[2], prof[3]], (-1.0, 0.0, 0.0)), ([prof[3], prof[4]], "out"),
            ([foot, (foot[0], 0.0)], (-1.0, 0.0, 0.0))]


def probe_head_total_m(d):
    return d["probe_head_length_m"] + d["probe_head_collar_m"]


def drogue_runs(d, stand_length_m=None):
    """Funnel about +x: the MOUTH plane at x = 0 (the config element, core: `probe_drogue_interface.hpp` "drogue mouths in the plane x = 0"),
    the cone narrows to the floor at x = -depth (radius throat), a solid foot under the floor, then a stand (radius throat + wall) down to
    x = -stand_length_m (the barrel, reached by `stand_length`) so the funnel does not float above the hull."""
    depth, mouth, throat = d["drogue_depth_m"], d["drogue_mouth_radius_m"], d["drogue_throat_radius_m"]
    wall, foot = d["drogue_wall_m"], d["drogue_foot_m"]
    outer_mouth = mouth + wall
    neck = throat + wall
    bottom = -(depth + foot) if stand_length_m is None else -stand_length_m
    if bottom > -(depth + foot):
        raise ValueError("drogue stand ends above the funnel's foot")
    return [([(-depth, 0.0), (-depth, throat)], SKIN_FLAT),
            ([(-depth, throat), (0.0, mouth)], "in"),
            ([(0.0, mouth), (0.0, outer_mouth)], SKIN_FLAT),
            ([(0.0, outer_mouth), (-depth - foot, neck)], "out"),
            ([(-depth - foot, neck), (bottom, neck)], "out"),
            ([(bottom, neck), (bottom, 0.0)], (-1.0, 0.0, 0.0))]


def stand_length(d, hull_radius_m, element_m, axis):
    """Distance from a drogue's mouth plane down to the foot of its stand: to the barrel along the port axis plus the hidden embed."""
    return hull_distance_m(hull_radius_m, element_m, axis) + d["connector_embed_m"]


def drogue_cone_radius_m(d, depth_below_mouth_m):
    """Inner radius of the funnel at `depth_below_mouth_m` (0 at the mouth), None outside the funnel."""
    depth = d["drogue_depth_m"]
    if depth_below_mouth_m < 0.0 or depth_below_mouth_m > depth:
        return None
    return d["drogue_mouth_radius_m"] - (d["drogue_mouth_radius_m"] - d["drogue_throat_radius_m"]) * depth_below_mouth_m / depth


def probe_offsets_m(d, hull_radius_m, element_m, axis):
    """Tip positions of a probe along the port axis, relative to its ROOT (the config element, on the interface plane; core
    `probe_drogue_interface.hpp`: a probe stands cone_depth + extension on the plane): stowed (just below the fairing top), seated (a head in its
    drogue's seat = cone_depth, the docked state with the extension retracted to 0) and extended (cone_depth + extension)."""
    stowed = hull_radius_m + d["fairing_height_m"] - d["probe_stowed_clearance_m"] - axial_m(element_m, axis)
    seated = d["drogue_depth_m"]
    extended = d["drogue_depth_m"] + d["probe_extension_m"]
    return {"stowed": stowed, "seated": seated, "extended": extended}


def probe_travel_m(d, hull_radius_m, element_m, axis):
    o = probe_offsets_m(d, hull_radius_m, element_m, axis)
    return o["extended"] - o["stowed"]


def fairing_geometry(d, hull_radius_m, element_m, axis):
    """Local numbers of one probe fairing about the axis line through the element, along +x measured from the EXTENDED tip: top x, floor of the bore, foot x."""
    ext = probe_offsets_m(d, hull_radius_m, element_m, axis)["extended"]
    tip_m = [e + a * ext for e, a in zip(element_m, axis)]
    top = hull_radius_m + d["fairing_height_m"] - axial_m(tip_m, axis)
    floor = -(probe_travel_m(d, hull_radius_m, element_m, axis) + probe_head_total_m(d)) - 0.02
    foot = -(hull_distance_m(hull_radius_m, tip_m, axis) + d["fairing_embed_m"])
    if floor <= foot:
        raise ValueError("fairing bore floor reaches the hull: lower the stroke or head")
    return {"top_x": top, "floor_x": floor, "foot_x": foot}


def fairing_runs(d, g):
    r_out, ch = d["fairing_radius_m"], d["fairing_chamfer_m"]
    bore = d["probe_head_radius_m"] + d["fairing_bore_clearance_m"]
    top = g["top_x"]
    return [([(g["foot_x"], r_out), (top - ch, r_out), (top, r_out - ch)], "out"),
            ([(top, r_out - ch), (top, bore)], SKIN_FLAT),
            ([(top, bore), (g["floor_x"], bore)], "in"),
            ([(g["floor_x"], bore), (g["floor_x"], 0.0)], SKIN_FLAT)]


def connector_runs(d, hull_radius_m, position_m, axis, line_diameter_m):
    """Face at x = 0: coupling nozzle (outer radius line/2 + wall) over a pedestal that reaches into the barrel."""
    r_noz = 0.5 * line_diameter_m + d["connector_nozzle_wall_m"]
    shoulder = -d["connector_nozzle_length_m"]
    foot = -(hull_distance_m(hull_radius_m, position_m, axis) + d["connector_embed_m"])
    r_ped = d["connector_pedestal_radius_m"]
    return [([(0.0, 0.0), (0.0, r_noz)], SKIN_FLAT),
            ([(0.0, r_noz), (shoulder, r_noz)], "out"),
            ([(shoulder, r_noz), (shoulder, r_ped)], SKIN_FLAT),
            ([(shoulder, r_ped), (foot, r_ped)], "out")]


def block(spec_dorsal, hull_radius_m):
    """The manifest entries the game needs (top-level keys of the manifest, see build_models.dorsal_manifest)."""
    d = spec_dorsal["drawing"]
    nodes = element_nodes(spec_dorsal["ports"])
    first = spec_dorsal["ports"][0]
    return {"dorsal_stage_id": spec_dorsal["stage_id"], "dorsal_elements_m": [n["element_m"] for n in nodes],
            "dorsal_nodes": [n["node"] for n in nodes], "dorsal_mechanism": nodes[0]["kind"], "dorsal_axis": list(first["axis"]),
            "probe_stroke_m": probe_travel_m(d, hull_radius_m, nodes[0]["element_m"], first["axis"]),
            "probe_tip_offsets_m": probe_offsets_m(d, hull_radius_m, nodes[0]["element_m"], first["axis"]),
            "probe_seated_extension": (lambda o: (o["seated"] - o["stowed"]) / (o["extended"] - o["stowed"]))(probe_offsets_m(d, hull_radius_m, nodes[0]["element_m"], first["axis"])),
            "dorsal_fairing_height_m": d["fairing_height_m"],
            "dorsal_hull_radius_m": hull_radius_m,
            "dorsal_port": {k: list(first[k]) for k in ("position_m", "axis", "up")},
            "dorsal_drogue": {k: d[k] for k in ("drogue_mouth_radius_m", "drogue_depth_m", "drogue_throat_radius_m")},
            "dorsal_connectors": {c["node"]: {"position_m": c["position_m"], "axis": c["axis"]}
                                  for c in connector_nodes(spec_dorsal["connectors"], first["axis"])}}


def _smoothstep(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def raceway_offset_m(x_m, connectors, d):
    """Drawn sideways offset (metres along the arc, towards increasing azimuth) of the dorsal raceway at station `x_m`.

    The Block 3 raceway tube lies on the dorsal centreline and the config puts both transfer connectors on it (y = 0), so the tube would run into
    the connector pedestals. User ruling 2026-11-04: until decided otherwise, route the tube around the pedestals IN THE DRAWING ONLY (a bump of
    `raceway_detour_m` over each pedestal, plateau over its radius, smooth ramps); physics and config stay. Zero away from the connectors."""
    best = 0.0
    for c in connectors:
        dx = abs(x_m - c["position_m"][0]) - d["connector_pedestal_radius_m"]
        best = max(best, d["raceway_detour_m"] * (1.0 - _smoothstep(dx / d["raceway_detour_ramp_m"])))
    return best


def raceway_stations(first_m, last_m, connectors, d, step_m=0.25):
    """Extra stations for the tube inside the detour bumps (a smooth S needs a short step); sorted, within (first, last)."""
    out = []
    reach = d["connector_pedestal_radius_m"] + d["raceway_detour_ramp_m"]
    for c in connectors:
        x = c["position_m"][0] - reach
        while x <= c["position_m"][0] + reach + 1e-9:
            if first_m < x < last_m:
                out.append(round(x, 6))
            x += step_m
    return sorted(set(out))


def foot_clears_connectors(x_m, half_length_m, half_width_m, connectors, d, gap_m=0.02):
    """True when a raceway foot (box x +- half_length, lateral offset(x) +- half_width) stays `gap_m` outside every connector pedestal circle."""
    off = raceway_offset_m(x_m, connectors, d)
    for c in connectors:
        cx = c["position_m"][0]
        nx = min(max(cx, x_m - half_length_m), x_m + half_length_m)  # nearest point of the box to the pedestal centre
        ny = min(max(0.0, off - half_width_m), off + half_width_m)
        if math.hypot(nx - cx, ny) < d["connector_pedestal_radius_m"] + gap_m:
            return False
    return True
