"""Writer of the geographic `[grid]` header read by core/src/config/lunar_site_config.cpp (`load_terrain_grid_file`).

Shared by geoid.py and earth_terrain.py. The grid is node-registered: line 0 is the north edge, lines go south, sample 0
is the west edge; node (i, j) sits at (north - i * step_lat, west + j * step_lon).
"""

from __future__ import annotations


def toml_value(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'
    return repr(value)


def header_text(comment: str, *, product: str, height_reference: str, dn_scale_m: float, heights: str, lines: int,
                samples: int, north_latitude_deg: float, west_longitude_deg: float, step_latitude_deg: float,
                step_longitude_deg: float, global_longitude: bool, checks: list[tuple[float, float, float]],
                extra: dict | None = None) -> str:
    text = (f"# {comment}\n"
            "[grid]\n"
            f"product = {toml_value(product)}\n"
            'projection = "geographic"\n'
            f"height_reference = {toml_value(height_reference)}\n"
            "sphere_radius_m = 6378137.0\n"
            f"dn_scale_m = {dn_scale_m}\n"
            f"heights = {toml_value(heights)}\n"
            f"lines = {lines}\n"
            f"samples = {samples}\n"
            f"north_latitude_deg = {north_latitude_deg}\n"
            f"west_longitude_deg = {west_longitude_deg}\n"
            f"step_latitude_deg = {step_latitude_deg!r}\n"
            f"step_longitude_deg = {step_longitude_deg!r}\n"
            f"global_longitude = {toml_value(global_longitude)}\n")
    for key, value in (extra or {}).items():
        text += f"{key} = {toml_value(value)}\n"
    for lat, lon, h in checks:
        text += f"\n[[check]]\nlatitude_deg = {lat!r}\nlongitude_deg = {lon!r}\nheight_m = {h!r}\n"
    return text
