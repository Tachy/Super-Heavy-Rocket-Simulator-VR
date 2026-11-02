"""Driver of the headless Blender generators: detail file -> glTF models in data/models/<site>/ with a manifest.

    python tools/structures/build_models.py [--force] [--config <toml>] [--out <dir>] [--blender <exe>]

The manifest holds the input hash (config + generator scripts + Blender version); a matching hash means no work.
Without Blender the driver prints a message and exits 0 (the game then runs with its boxes).

Vehicle models (hull, flaps, grid fin, engines of a vehicle config):

    python tools/structures/build_models.py --vehicle starship_block3 [--force] [--out <dir>] [--blender <exe>]

writes data/models/<vehicle id>/ the same way (spec JSON from `vehicle_spec.py` hashed into the manifest).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hull_textures  # noqa: E402
import detail_file  # noqa: E402
import hull_tiles  # noqa: E402
import mount_textures  # noqa: E402
import vehicle_spec  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / "tools" / "structures" / "blender"
DEFAULT_CONFIG = ROOT / "config" / "structures" / "starbase_olp2.toml"
DEFAULT_OUT = ROOT / "data" / "models" / "starbase_olp2"
BLENDER_DEFAULT = Path(r"C:\Program Files\Blender Foundation\Blender 3.4\blender.exe")
GENERATORS = ("gen_profiles.py", "gen_chopstick.py", "gen_carriage.py", "gen_rails.py", "gen_qd_arm.py", "gen_mount.py", "gen_trench.py")
OUTPUTS = ("profiles.gltf", "chopstick.gltf", "chopstick_minus.gltf", "carriage.gltf", "rails.gltf", "qd_arm.gltf", "mount.gltf", "trench.gltf",
           mount_textures.FILE_NAME, mount_textures.CONCRETE_FILE_NAME, mount_textures.APRON_MASK_FILE_NAME)
MANIFEST = "manifest.json"
VEHICLE_FIXED_OUTPUTS = ("booster.gltf", "ship.gltf", "flap_fwd.gltf", "flap_aft.gltf", "grid_fin.gltf",
                         hull_textures.FILE_NAME, *hull_tiles.FILES.values())
VEHICLE_FRAME = ("Blender axes in glTF (export_yup off) = the config's body frame: X station axis (station 0 at X = 0, "
                 "nose +X), (Y, Z) = r (cos a, sin a), belly azimuth 270 = -Z; game maps body (x, y, z) to render "
                 "(y, z, x). Hull UV = (arc metres at the barrel radius, 1 - station) in Blender = (u, station) in glTF. "
                 "Per-model frames: docstrings of gen_*.py")


def find_blender(explicit: str | None = None) -> Path | None:
    for cand in (explicit, os.environ.get("STARSIM_BLENDER"), shutil.which("blender"), str(BLENDER_DEFAULT)):
        if cand and Path(cand).is_file():
            return Path(cand)
    return None


def blender_command(exe: Path, script: Path, *script_args: str) -> list:
    """Headless run of a generator; a Python exception gives a non-zero exit (without the flag Blender exits 0 on one)."""
    return [str(exe), "-b", "--factory-startup", "--python-exit-code", "1", "-P", str(script), "--", *script_args]


def swap_in(staging: Path, out: Path) -> None:
    """Replace `out` by the finished `staging`; the old directory stays until the new one has its name."""
    old = out.parent / (out.name + ".old")
    if old.exists():
        shutil.rmtree(old)
    if out.exists():
        out.rename(old)
    try:
        staging.rename(out)
    except OSError:
        if old.exists() and not out.exists():
            old.rename(out)
        raise
    shutil.rmtree(old, ignore_errors=True)


def blender_version(blender: Path) -> str:
    out = subprocess.run([str(blender), "-b", "--factory-startup", "--version"], capture_output=True, text=True,
                         check=True).stdout
    m = re.search(r"Blender\s+(\S+)", out)
    return m.group(1) if m else out.strip().splitlines()[0]


def input_hash(config: Path, version: str) -> str:
    h = hashlib.sha256()
    h.update(version.encode())
    h.update(f"site_tower={sorted(detail_file.site_tower().items())}".encode())  # drawn on the physics values, not in the detail file
    for p in [config, SCRIPT_DIR / "common.py", SCRIPT_DIR / "member_geometry.py", SCRIPT_DIR / "mechazilla_layout.py", SCRIPT_DIR / "mount_layout.py",
              Path(mount_textures.__file__), *(SCRIPT_DIR / g for g in GENERATORS)]:
        h.update(p.name.encode())
        h.update(p.read_bytes().replace(b"\r\n", b"\n"))
    return h.hexdigest()


def is_current(out: Path, digest: str) -> bool:
    try:
        manifest = json.loads((out / MANIFEST).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False
    return manifest.get("input_hash") == digest and all((out / f).is_file() for f in OUTPUTS)


def build(config: Path = DEFAULT_CONFIG, out: Path = DEFAULT_OUT, force: bool = False,
          blender: str | None = None) -> int:
    exe = find_blender(blender)
    if exe is None:
        print("build_models: Blender not found (set STARSIM_BLENDER or install Blender 3.4); no models generated")
        return 0
    version = blender_version(exe)
    digest = input_hash(config, version)
    if not force and is_current(out, digest):
        # The game compares the manifest's mtime with the sources': a checkout that makes a source newer than the
        # (still matching) models must not pin the game on its boxes.
        (out / MANIFEST).touch()
        print(f"build_models: models up to date ({out})")
        return 0
    # Generate into a sibling directory and swap it in only when every generator succeeded: a failure must not leave new
    # glTF next to an old manifest (or half-written files in place of the models the game loads).
    staging = out.parent / (out.name + ".new")
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True)
    triangles: dict[str, int] = {}
    try:
        panel_map = mount_textures.write(staging, detail_file.load(config))
    except Exception as exc:
        print(f"build_models: texture step failed ({exc}); {out} left as it was")
        shutil.rmtree(staging, ignore_errors=True)
        return 1
    for gen in GENERATORS:
        cmd = blender_command(exe, SCRIPT_DIR / gen, "--config", str(config), "--out", str(staging))
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:
            sys.stderr.write(res.stdout[-2000:] + res.stderr[-2000:])
            print(f"build_models: {gen} failed (exit {res.returncode}); {out} left as it was")
            shutil.rmtree(staging, ignore_errors=True)
            return 1
        for m in re.finditer(r"^TRIANGLES (\S+) (\d+)$", res.stdout, re.M):
            triangles[m.group(1)] = int(m.group(2))
    missing = [f for f in OUTPUTS if not (staging / f).is_file()]
    if missing:
        print(f"build_models: generators produced no {missing}; {out} left as it was")
        shutil.rmtree(staging, ignore_errors=True)
        return 1
    manifest = {"input_hash": digest, "blender": version, "config": config.name, "files": list(OUTPUTS),
                "triangles": triangles, "mount_map": panel_map,
                "frame": "Blender axes in glTF (export_yup off); per model: mechazilla_layout.py"}
    (staging / MANIFEST).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    swap_in(staging, out)
    print(f"build_models: generated {', '.join(OUTPUTS)} in {out}; triangles {triangles}")
    return 0


def vehicle_sources(table: dict) -> list:
    """Files hashed into a vehicle's manifest, in order: helper scripts, texture bakers, generators (generators.toml)."""
    return [*(SCRIPT_DIR / s for s in table["scripts"]), *(SCRIPT_DIR.parent / f"{b}.py" for b in table["bakers"]),
            *(SCRIPT_DIR / g for g in table["generators"])]


def vehicle_hash(spec_text: str, version: str, table: dict) -> str:
    h = hashlib.sha256()
    h.update(version.encode())
    h.update(spec_text.encode())
    for p in vehicle_sources(table):
        h.update(p.name.encode())
        h.update(p.read_bytes().replace(b"\r\n", b"\n"))
    return h.hexdigest()


def vehicle_outputs(spec: dict, table: dict | None = None) -> tuple:
    if table is not None and "outputs" in table:
        return tuple(table["outputs"])
    return VEHICLE_FIXED_OUTPUTS + tuple(e["file"] for _, e in sorted(spec["engines"].items()))


def falcon9_manifest(spec: dict, digest: str, version: str, triangles: dict, table: dict) -> dict:
    """What the game needs of the Falcon 9's first stage: the leg rig, its deployed foot, the Merlin's exit plane and drawn
    offset (station 0 is the outer Merlins' exit plane: the drawn gimbal point stands one bell length above the physics
    one), part -> material names and the paints."""
    detail = spec["detail"]
    legs, fin = spec["stages"]["first_stage"]["landing_legs"], detail["grid_fin"]
    engines = {eid: {"file": e["file"], "exit_plane_x_m": -e["bell_length_m"], "exit_diameter_m": e["nozzle_exit_diameter_m"],
                     "axial_offset_m_by_stage": {"first_stage": e["bell_length_m"]}}
               for eid, e in sorted(spec["engines"].items())}
    # The Merlin Vacuum's gimbal point IS the second stage's station 0 (no offset); its bell hangs inside the interstage.
    mvac = spec["mvac"]
    engines["merlin_vacuum"] = {"file": mvac["file"], "exit_plane_x_m": -mvac["bell_length_m"],
                                "exit_diameter_m": mvac["nozzle_exit_diameter_m"], "axial_offset_m_by_stage": {"second_stage": 0.0}}
    paint = detail["paint"]
    look = {"no_maps": True, "engine_ids": {"sea_level": "merlin1d", "vacuum": "merlin_vacuum"},
            "interstage_base_m": detail["hull"]["interstage_base_m"], "plume_start_at_exit_diameter": True,
            "fin_stow_standoff_m": fin["stow_standoff_m"],
            "part_materials": {"hull": "paint_hull", "interstage": "paint_interstage", "octaweb": "paint_octaweb",
                               "engine_plate": "paint_engine_plate"},
            "stage_part_materials": {"second_stage": {"hull": "paint_hull2"}},
            "paint": {"paint_hull": {"albedo": paint["hull_albedo"], "roughness": 0.45, "metallic": 0.0},
                      "paint_interstage": {"albedo": paint["interstage_albedo"], "roughness": 0.6, "metallic": 0.0},
                      "paint_octaweb": {"albedo": paint["octaweb_albedo"], "roughness": 0.7, "metallic": 0.1},
                      "paint_leg": {"albedo": paint["leg_albedo"], "roughness": 0.6, "metallic": 0.1},
                      "paint_hull2": {"albedo": paint["stage2_hull_albedo"], "roughness": 0.5, "metallic": 0.0},
                      "paint_engine_plate": {"albedo": paint["engine_plate_albedo"], "roughness": 0.8, "metallic": 0.0}},
            "engine_look": {"vacuum": {"albedo": paint["mvac_albedo"], "roughness": 0.55, "metallic": 0.35,
                                       "rib_slope": 0.0, "rib_top_x_m": -0.1}}}
    return {"input_hash": digest, "blender": version, "vehicle": spec["vehicle_id"], "files": list(vehicle_outputs(spec, table)),
            "triangles": triangles, "frame": VEHICLE_FRAME, "flaps": {}, "engines": engines, "look": look, "leg": spec["leg"],
            "leg_foot_deployed_m": {"station": legs["foot_station_m"], "radius": legs["footprint_radius_m"]}}


def vehicle_manifest(spec: dict, digest: str, version: str, triangles: dict) -> dict:
    """What the game needs besides the glTF: per-flap rotation signs, engine exit planes, the frame statement."""
    flaps = {st_id: [{k: f[k] for k in ("id", "kind", "hinge_azimuth_deg", "station_aft_m", "station_fwd_m", "span_m",
                                          "rotation_sign_about_plus_x", "drawn_hinge_station_aft_m",
                                          "drawn_hinge_lean_deg", "drawn_hinge_radius_aft_m",
                                          "drawn_hinge_station_fwd_m")} for f in st["flaps"]]
             for st_id, st in spec["stages"].items() if st["flaps"]}
    detail = spec["detail"]
    recess = detail["ship"]["engine_recess"]
    # ship_axial_offset_m: the drawn engine sits this far up the station axis from the physics gimbal point (ship only)
    engines = {eid: {"file": e["file"], "exit_plane_x_m": -e["bell_length_m"],
                     "exit_diameter_m": e["nozzle_exit_diameter_m"], "ship_exit_station_m": recess[eid],
                     "ship_axial_offset_m": recess[eid] + e["bell_length_m"]}
               for eid, e in sorted(spec["engines"].items())}
    look = {"ring_pitch_m": detail["hull"]["ring_pitch_m"],  # the shader's per-ring panels and the steel map's period
            "weld_first_m": detail["hull"]["base_chamfer_m"] + 2.0 * detail["hull"]["weld_half_width_m"],
            "ship_barrel_end_m": spec["stages"]["ship"]["length_m"] - spec["stages"]["ship"]["nose_length_m"],
            "ship_belly_azimuth_deg": spec["stages"]["ship"]["belly_azimuth_deg"],
            "catch_fin_azimuths_deg": detail["grid_fin"]["catch_fin_azimuths_deg"],
            "rib_counts": {("rvac" if e["file"].startswith("rvac") else "raptor"): e["rib_count"]
                           for e in spec["engines"].values()},
            "tiles": {k: detail["ship"]["tiles"][k] for k in ("hex_across_flats_m", "gap_m", "coverage_station_m",
                                                               "coverage_half_angle_deg")}}
    return {"input_hash": digest, "blender": version, "vehicle": spec["vehicle_id"], "files": list(vehicle_outputs(spec)),
            "triangles": triangles, "frame": VEHICLE_FRAME, "flaps": flaps, "engines": engines, "look": look}


def build_vehicle(vehicle_id: str, out: Path | None = None, force: bool = False, blender: str | None = None,
                  vehicles_dir: Path = vehicle_spec.VEHICLES) -> int:
    out = out or ROOT / "data" / "models" / vehicle_id
    exe = find_blender(blender)
    if exe is None:
        print("build_models: Blender not found (set STARSIM_BLENDER or install Blender 3.4); no models generated")
        return 0
    table = vehicle_spec.generator_table(vehicle_id)
    spec = vehicle_spec.load(vehicle_id, vehicles_dir)
    if not table["generators"]:
        print(f"build_models: {vehicle_id} has no generators yet; no models generated (the game keeps its cylinders)")
        return 0
    spec_text = vehicle_spec.dumps(spec)
    version = blender_version(exe)
    digest = vehicle_hash(spec_text, version, table)
    outputs = vehicle_outputs(spec, table)
    starship = table["kind"] == "starship"
    try:
        current = json.loads((out / MANIFEST).read_text(encoding="utf-8")).get("input_hash") == digest
    except (OSError, ValueError):
        current = False
    if not force and current and all((out / f).is_file() for f in outputs):
        (out / MANIFEST).touch()
        print(f"build_models: vehicle models up to date ({out})")
        return 0
    staging = out.parent / (out.name + ".new")
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True)
    spec_file = staging.parent / (staging.name + ".spec.json")
    spec_file.write_text(spec_text, encoding="utf-8")
    triangles: dict[str, int] = {}
    try:
        if starship:
            steel_map = hull_textures.write(staging, spec)
            tile_maps = hull_tiles.write(staging, spec, hull_textures.png_bytes)
        for gen in table["generators"]:
            cmd = blender_command(exe, SCRIPT_DIR / gen, "--spec", str(spec_file), "--out", str(staging))
            res = subprocess.run(cmd, capture_output=True, text=True)
            missing = [f for f in outputs if not (staging / f).is_file()] if gen == table["generators"][-1] else []
            if res.returncode != 0 or missing:
                sys.stderr.write(res.stdout[-2000:] + res.stderr[-2000:])
                print(f"build_models: {gen} failed (exit {res.returncode}, missing {missing}); {out} left as it was")
                shutil.rmtree(staging, ignore_errors=True)
                return 1
            for m in re.finditer(r"^TRIANGLES (\S+) (\d+)$", res.stdout, re.M):
                triangles[m.group(1)] = int(m.group(2))
    finally:
        spec_file.unlink(missing_ok=True)
    if starship:
        manifest = vehicle_manifest(spec, digest, version, triangles)
        manifest["look"]["steel_map"] = steel_map
        manifest["look"]["tile_maps"] = tile_maps
    else:
        manifest = falcon9_manifest(spec, digest, version, triangles, table)
    (staging / MANIFEST).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    swap_in(staging, out)
    print(f"build_models: generated {', '.join(outputs)} in {out}; triangles {triangles}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--force", action="store_true", help="regenerate even when the manifest hash matches")
    ap.add_argument("--vehicle", default=None, help="vehicle id: build its models instead of the Pad B ones")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--blender", default=None)
    a = ap.parse_args(argv)
    if a.vehicle:
        return build_vehicle(a.vehicle, a.out, a.force, a.blender)
    return build(a.config, a.out or DEFAULT_OUT, a.force, a.blender)


if __name__ == "__main__":
    sys.exit(main())
