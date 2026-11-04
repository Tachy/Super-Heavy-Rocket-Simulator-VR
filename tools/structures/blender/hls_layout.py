"""HLS hull parts, pure Python (no Blender): the thruster pods and the nose port as revolve profiles, the manifest block.

Numbers: config/vehicles/detail/starship_hls.toml [landing_thruster] / [nose_port] (drawing, [E]); placement: the config's `landing` engine
group as the core resolves it (vehicle_spec.hls_parts_block mirrors core/src/vehicle/vehicle.cpp). Frames: the stage's body frame (station X,
nose +X). A thruster node's local +X is its THRUST axis (the exhaust leaves along -X); its exit plane is X = 0 on the config position, the bell
and the pod lie behind it (X > 0, towards the hull).
"""
import math


def thruster_nodes(thrusters):
    """One node per config thruster, named by its index in the stage's engine list order of the landing group."""
    return [dict(t, node=f"thruster_{i}") for i, t in enumerate(thrusters)]


def exit_radius_m(t):
    return 0.5 * t["exit_diameter_m"]


def pod_radius_m(d, t):
    return exit_radius_m(t) + d["pod_wall_m"]


def bell_contour(d, t, samples=10):
    """Inner (flow) wall as (X, r) from the exit plane (X = 0) to the throat (X = bell length): the exit radius is the config's."""
    r_e, r_t, length, e = exit_radius_m(t), 0.5 * d["throat_diameter_m"], d["bell_length_m"], d["contour_exponent"]
    return [(length * i / samples, r_t + (r_e - r_t) * ((length - length * i / samples) / length) ** e) for i in range(samples + 1)]


def thruster_runs(d, t):
    """Profiles for vehicle_common.revolve about the thrust axis: exit-plane face, closed bell lip, flow wall, throat plate, pod body and taper."""
    r_e, wall, r_p = exit_radius_m(t), d["wall_m"], pod_radius_m(d, t)
    inner = bell_contour(d, t)
    length = d["bell_length_m"]
    body, taper = d["pod_length_m"], d["pod_taper_m"]
    return [([(0.0, r_e + wall), (0.0, r_p)], (-1.0, 0.0, 0.0)),
            ([(0.0, r_e), (0.0, r_e + wall)], (-1.0, 0.0, 0.0)),
            (inner, "in"),
            ([(length, inner[-1][1]), (length, 0.0)], (-1.0, 0.0, 0.0)),
            ([(0.0, r_p), (body, r_p)], "out"),
            ([(body, r_p), (body + taper, 0.0)], "out")]


def nose_port_runs(d, length_m):
    """Profiles (absolute stations) of the visual-only docking port on the nose tip: collar, rim, recessed face, hub."""
    rc, top = d["radius_m"], length_m + d["height_m"]
    floor = top - d["recess_m"]
    hub_top = floor + 0.03  # [E] the hub stands 3 cm proud of the face (a drawn detail)
    return [([(length_m - d["embed_m"], rc), (top, rc)], "out"),
            ([(top, rc), (top, d["face_radius_m"])], (1.0, 0.0, 0.0)),
            ([(top, d["face_radius_m"]), (floor, d["face_radius_m"])], "in"),
            ([(floor, d["face_radius_m"]), (floor, d["hub_radius_m"])], (1.0, 0.0, 0.0)),
            ([(floor, d["hub_radius_m"]), (hub_top, d["hub_radius_m"])], "out"),
            ([(hub_top, d["hub_radius_m"]), (hub_top, 0.0)], (1.0, 0.0, 0.0))]


def block(hls):
    """Manifest entries (top-level keys): the thruster exit planes (stage-local body frame), the nose port, the textured parts' places."""
    d = hls["drawing"]
    nodes = thruster_nodes(hls["thrusters"])
    return {"hls_stage_id": hls["stage_id"],
            "thruster_exit_m": [{"node": n["node"], "exit_m": list(n["position_m"]), "axis": list(n["axis"]),
                                 "exit_diameter_m": n["exit_diameter_m"], "pod_radius_m": pod_radius_m(d["landing_thruster"], n)} for n in nodes],
            "hls_nose_port": {"visual_only": True, "station_m": hls["length_m"], "height_m": d["nose_port"]["height_m"], "radius_m": d["nose_port"]["radius_m"]},
            "hls_solar_band": {"station_from_m": d["solar_band"]["station_from_m"], "station_to_m": d["solar_band"]["station_to_m"]},
            "hls_windows": {k: d["windows"][k] for k in ("station_m", "azimuth_deg", "count", "pitch_m", "width_m", "height_m")}}
