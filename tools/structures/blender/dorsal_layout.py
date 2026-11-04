"""Dorsal transfer hardware of a ship (tanker probes, depot drogues, connectors): pure-Python layout, no Blender.

Used by gen_dorsal_hardware.py (the mesh), by vehicle_spec / build_models (the manifest block) and by the tests. The numbers come from
the vehicle config (elements, positions, axes) and the `[dorsal]` table of the detail file.

Local part frame: +X is the interface normal (the port's `axis`, out of the vehicle), the part is a body of revolution about it, profiles are
(x, r). Origin: probe = the extended TIP (the config element); drogue = the floor point where the tip rests (the config element, the funnel opens
towards +X); connector = the centre of the coupling face (the config position). The generator rotates +X onto the port axis and leaves the
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


def drogue_runs(d):
    """Funnel about +x: floor at x = 0 (radius throat), cone out to the mouth at x = depth, rim, outer cone and solid foot below the floor."""
    depth, mouth, throat = d["drogue_depth_m"], d["drogue_mouth_radius_m"], d["drogue_throat_radius_m"]
    wall, foot = d["drogue_wall_m"], d["drogue_foot_m"]
    outer_mouth = mouth + wall
    return [([(0.0, 0.0), (0.0, throat)], SKIN_FLAT),
            ([(0.0, throat), (depth, mouth)], "in"),
            ([(depth, mouth), (depth, outer_mouth)], SKIN_FLAT),
            ([(depth, outer_mouth), (-foot, throat + wall)], "out"),
            ([(-foot, throat + wall), (-foot, 0.0)], (-1.0, 0.0, 0.0))]


def fairing_geometry(d, hull_radius_m, element_m, axis):
    """Local (about the element's axis line) numbers of one probe fairing: top x, floor of the bore, foot x. All relative to the element, along +x."""
    top = hull_radius_m + d["fairing_height_m"] - axial_m(element_m, axis)
    floor = -(d["probe_stroke_m"] + probe_head_total_m(d)) - 0.02
    foot = -(hull_distance_m(hull_radius_m, element_m, axis) + d["fairing_embed_m"])
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
            "probe_stroke_m": d["probe_stroke_m"], "dorsal_fairing_height_m": d["fairing_height_m"],
            "dorsal_hull_radius_m": hull_radius_m,
            "dorsal_connectors": {c["node"]: {"position_m": c["position_m"], "axis": c["axis"]}
                                  for c in connector_nodes(spec_dorsal["connectors"], first["axis"])}}
