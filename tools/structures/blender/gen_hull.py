"""Hulls of Super Heavy and Starship: plain lathe hull, nose, hot-stage section, top truss, skirt, chine, raceway, pins.

The shapes follow the 1:100 reference model (tools/structures/measure/stl_measure.py; geometry measured, nothing
copied): the ship's drawn ogive is 13.4 m (config physics nose: 17 m), the booster's top has a 20-tooth V truss.

Run: blender -b --factory-startup -P gen_hull.py -- --spec <json> --out <dir>
Writes booster.gltf and ship.gltf. Frame (see vehicle_common): X = station axis with the stage's station 0 at X = 0, the
nose at X = length; (Y, Z) = r (cos a, sin a). Root node = the stage id (identity transform), children: `hull` (the
shell: a plain lathe, no weld rings - welds and dents are the baked steel maps; UV = (arc metres at the
barrel radius, station metres), glTF v is the station), `hot_stage`, `fin_fairings`, `catch_pins`, `chine`,
`raceway` where the stage has them.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

NOSE_SAMPLES = 20  # [E] profile samples along the ogive (chord error < 5 mm on the 17 m / rho 35 m ogive)
MERGE_M = 1e-4  # profile stations closer than this are one
PIN_SIDES = 12  # [E]
RACEWAY_SIDES = 8  # [E]
STRAKE_RAMP_STEPS = 5  # [E] segments of the strake's soft end ramp
PORT_SIDES = 16  # [E] round transfer fittings
FOOT_STEP_M = 2.0  # [E] spacing of the raceway tube's feet (brackets)
SKIRT_SEGMENTS = 64  # [E] the skirt rings are thin flanges: sagitta 5.6 mm at R = 4.5 m
CHINE_STEP_M = 0.5  # [E] sweep step of the chine along the barrel
DOME_SAMPLES = 12  # segments of the booster's forward dome
EMBED_M = 0.01  # [E] details start this far inside the shell (no gap on the curved hull)
X_AXIS = Vector((1.0, 0.0, 0.0))


class Profile:
    """Radius of the hull over the station: chamfer, plain barrel, tangent ogive nose with a blunt tip."""

    def __init__(self, stage, hull):
        self.L, self.R = stage["length_m"], stage["diameter_m"] / 2.0
        # the DRAWN ogive is the 1:100 model's (13.4 m), not the config's physics nose_length_m (17 m): a documented
        # deviation
        self.nose = hull["nose_drawn_length_m"] if stage["nose_length_m"] > 0.0 else 0.0
        self.chamfer = hull["base_chamfer_m"]
        self.tip_r = hull["nose_tip_radius_m"]
        self.cap_h = hull["nose_tip_cap_m"] if self.nose > 0.0 else 0.0
        if self.nose > 0.0:
            R, Ln = self.R, self.nose
            self.rho = (R * R + Ln * Ln) / (2.0 * R)
            # ogive from the tip: r(s) = sqrt(rho^2 - (Ln - s)^2) + R - rho; truncated where r = tip radius, the
            # remaining length is squeezed so that truncated ogive + cap ends exactly at X = length
            self.s_t = Ln - math.sqrt(self.rho ** 2 - (self.rho - R + self.tip_r) ** 2)
        self.x_barrel_end = self.L - self.nose
        # a booster's forward dome (1:100 model photo): the barrel ends at the truss shoulder, a half ellipse closes it
        # below the ring under the ship, inside the physics length
        self.dome_h = hull.get("dome_height_m", 0.0) if self.nose <= 0.0 else 0.0
        if self.dome_h > 0.0:
            self.x_barrel_end = self.L - hull["dome_start_m"]

    def base(self, x):
        """Radius of the shell."""
        if x < self.chamfer:
            return self.R - self.chamfer + x
        if self.dome_h > 0.0 and x > self.x_barrel_end:
            return self.R * math.sqrt(max(1.0 - ((x - self.x_barrel_end) / self.dome_h) ** 2, 0.0))
        if self.nose <= 0.0 or x <= self.x_barrel_end:
            return self.R
        s_new = self.L - x  # distance from the tip along the axis
        s = self.s_t + (s_new - self.cap_h) * (self.nose - self.s_t) / (self.nose - self.cap_h)
        return math.sqrt(max(self.rho ** 2 - (self.nose - s) ** 2, 0.0)) + self.R - self.rho

    def radius(self, x):
        return self.base(x)

    def stations(self):
        if self.dome_h > 0.0:
            return [0.0, self.chamfer, self.x_barrel_end]
        xs = {0.0, self.chamfer, self.x_barrel_end if self.nose > 0 else self.L}  # a straight barrel: its two ends only
        if self.nose > 0.0:
            x_end = self.L - self.cap_h
            for i in range(NOSE_SAMPLES + 1):  # denser toward the tip
                f = 1.0 - (1.0 - i / NOSE_SAMPLES) ** 1.6
                xs.add(self.x_barrel_end + f * (x_end - self.x_barrel_end))
            xs.add(x_end)
        xs = sorted(x for x in xs if 0.0 <= x <= (self.L - self.cap_h) + 1e-9)
        out = []
        for x in xs:
            if not out or x - out[-1] > MERGE_M:
                out.append(x)
        return out

    def runs(self):
        main = [(x, self.radius(x)) for x in self.stations()]
        if self.dome_h > 0.0:
            for i in range(1, DOME_SAMPLES + 1):
                t = 0.5 * math.pi * i / DOME_SAMPLES
                main.append((self.x_barrel_end + self.dome_h * math.sin(t), self.R * math.cos(t)))
            return [([(0.0, 0.0), (0.0, main[0][1])], (-1.0, 0.0, 0.0)), (main[:2], "out"), (main[1:], "out")]
        if self.nose > 0.0:  # blunt tip: a quarter ellipse (axial cap_h, radial tip_r) closes the shell at X = length
            x_e, r_e = main[-1]
            for i in range(1, 9):
                t = 0.5 * math.pi * i / 8.0
                main.append((x_e + self.cap_h * math.sin(t), r_e * math.cos(t)))
        # the chamfer is its own smooth group: one long barrel strip must not inherit the chamfer corner's normal
        runs = [([(0.0, 0.0), (0.0, main[0][1])], (-1.0, 0.0, 0.0)), (main[:2], "out"), (main[1:], "out")]
        if self.nose <= 0.0:
            runs.append(([(self.L, main[-1][1]), (self.L, 0.0)], (1.0, 0.0, 0.0)))
        return runs


def tile_half_angle_deg(tiles, x):
    xs, ys = tiles["coverage_station_m"], tiles["coverage_half_angle_deg"]
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            f = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + f * (ys[i] - ys[i - 1])
    return ys[-1]


def _build_hull(stage, detail, profile):
    mesh = vc.VMesh()
    vc.revolve(mesh, profile.runs(), int(detail["hull"]["segments"]), profile.R, "hull")
    return mesh


def _build_hot_stage(stage, detail, profile):
    """Collar ring at the lower edge of the hot-stage section; the section itself is the V truss (`top_truss`)."""
    hs = detail["booster"]["hot_stage"]
    mesh = vc.VMesh()
    R, L = profile.R, profile.L
    x1 = L - hs["height_m"]
    x0 = x1 - hs["flange_height_m"]
    p = hs["flange_proud_m"]
    seg = int(detail["hull"]["segments"])
    vc.revolve(mesh, [([(x0, R + p), (x1, R + p)], "out"), ([(x1, R + p), (x1, R - EMBED_M)], (1.0, 0.0, 0.0)),
                      ([(x0, R - EMBED_M), (x0, R + p)], (-1.0, 0.0, 0.0))], seg, R, "flange")
    return mesh


def _build_fin_fairings(stage, detail, profile):
    f = detail["booster"]["grid_fin_fairing"]
    gf = stage["grid_fins"]
    span = 2.0 * (gf["radius_m"] - profile.R)
    chord = gf["area_m2"] / span
    mesh = vc.VMesh()
    for az in gf["azimuths_deg"]:
        a = math.radians(az)
        er, et = vc.radial_dir(a), Vector((0.0, -math.sin(a), math.cos(a)))
        half = (chord * f["length_frac"] / 2.0, f["width_m"] / 2.0, f["height_m"] / 2.0)
        vc.box(mesh, vc.pt(gf["station_m"], a, profile.R + f["height_m"] / 2.0 - EMBED_M), X_AXIS, et, er, half)
    return mesh


def _ring_points(x, r, segments):
    return [vc.pt(x, 2.0 * math.pi * i / segments, r) for i in range(segments + 1)]


def _build_top_truss(stage, detail, profile):
    """Open V-truss ring on the booster top (1:100 model: 20 teeth, 18 deg pitch; photos: a zigzag ring)."""
    t = detail["booster"]["top_truss"]
    mesh = vc.VMesh()
    n, R, L, rr = int(t["tooth_count"]), profile.R + 0.04, profile.L, t["tube_radius_m"]
    sub = 3
    pts = _ring_points(L - rr, R, n * sub)
    for k in range(len(pts) - 1):
        vc.tube(mesh, pts[k], pts[k + 1], rr, 6, ("ring", 0), cap0=False, cap1=False)
    for i in range(n):
        a0, a1 = 2.0 * math.pi * i / n, 2.0 * math.pi * (i + 1) / n
        am = 0.5 * (a0 + a1)
        valley = vc.pt(L - t["height_m"], am, R)
        # tapered rods (chopsticks photo 2026-03): slender at the ring, stouter at the valley
        vc.tube(mesh, vc.pt(L - rr, a0, R), valley, rr * t["taper_top"], 6, ("strut", i, 0), radius1=rr * t["taper_foot"])
        vc.tube(mesh, vc.pt(L - rr, a1, R), valley, rr * t["taper_top"], 6, ("strut", i, 1), radius1=rr * t["taper_foot"])
        et = Vector((0.0, -math.sin(am), math.cos(am)))
        foot = vc.pt(L - t["height_m"] - t["post_height_m"], am, R - 0.02)
        vc.tube(mesh, valley, foot, rr, 6, ("post", i))
        for sgn in (-1.0, 1.0):  # forked foot (1:100 model photo): two short prongs on the shoulder
            vc.tube(mesh, foot, foot + et * (sgn * 0.12) - X_AXIS * 0.35, rr * 0.7, 5, ("prong", i, sgn))
    return mesh


def _build_skirt(stage, detail, profile):
    """Engine skirt: base ring, four thin ring flanges, a comb of posts between them, side ports (1:100 model)."""
    s = detail["booster"]["skirt"]
    R, e = profile.R, EMBED_M
    br, rp, rt = s["base_ring_radius_m"], s["ring_proud_m"], s["ring_thickness_m"]
    mesh = vc.VMesh()
    runs = [([(0.0, R - e), (0.0, R + br)], (-1.0, 0.0, 0.0)), ([(0.0, R + br), (0.4, R + br)], "out"),
            ([(0.4, R + br), (0.4, R - e)], (1.0, 0.0, 0.0))]
    for x in s["ring_stations_m"]:
        runs += [([(x - rt / 2, R - e), (x - rt / 2, R + rp)], (-1.0, 0.0, 0.0)),
                 ([(x - rt / 2, R + rp), (x + rt / 2, R + rp)], "out"),
                 ([(x + rt / 2, R + rp), (x + rt / 2, R - e)], (1.0, 0.0, 0.0))]
    vc.revolve(mesh, runs, SKIRT_SEGMENTS, R, "skirt")
    n = int(s["tooth_count"])
    for i in range(n):
        a = 2.0 * math.pi * (i + 0.5) / n
        er, et = vc.radial_dir(a), Vector((0.0, -math.sin(a), math.cos(a)))
        half = (s["tooth_height_m"] / 2.0, 0.06, (s["tooth_proud_m"] + e) / 2.0)
        vc.box(mesh, vc.pt(s["tooth_station_m"], a, R + (s["tooth_proud_m"] - e) / 2.0), X_AXIS, et, er, half)
    n = int(s["port_count"])
    for i in range(n):
        a = math.radians(45.0 + 360.0 * i / n)
        er, et = vc.radial_dir(a), Vector((0.0, -math.sin(a), math.cos(a)))
        half = (s["port_height_m"] / 2.0, s["port_width_m"] / 2.0, (s["port_proud_m"] + e) / 2.0)
        vc.box(mesh, vc.pt(s["port_station_m"], a, R + (s["port_proud_m"] - e) / 2.0), X_AXIS, et, er, half)
    return mesh


def _build_booster_raceway(stage, detail, profile):
    """Thin line rail along the booster and one pill-shaped line fairing per tank section (1:100 model)."""
    rw = detail["booster"]["raceway"]
    a = math.radians(rw["azimuth_deg"])
    mesh = vc.VMesh()
    er, et = vc.radial_dir(a), Vector((0.0, -math.sin(a), math.cos(a)))
    off = math.radians(8.0)
    vc.tube(mesh, vc.pt(2.0, a + off, profile.R + rw["rail_radius_m"] * 0.5),
            vc.pt(profile.L - 4.0, a + off, profile.R + rw["rail_radius_m"] * 0.5), rw["rail_radius_m"], 6,
            ("rail", 0))
    k = 0
    x_top = profile.L - 3.9
    while x_top - rw["pill_length_m"] > 3.0:
        half = (rw["pill_length_m"] / 2.0, rw["pill_width_m"] / 2.0, rw["height_m"] / 2.0)
        xc = x_top - rw["pill_length_m"] / 2.0
        vc.box(mesh, vc.pt(xc, a - off, profile.R + rw["height_m"] / 2.0 - EMBED_M), X_AXIS, et, er, half)
        x_top -= rw["pill_pitch_m"]
        k += 1
    return mesh


def _build_strakes(stage, detail, profile):
    """Long line fairings along the lower booster (1:100 model: four ridges, photo 2026-05: two tall boxes with a
    sloped top at the silhouette): trapezoid section, full height from the lower end, sloped top over `chamfer_m`."""
    sk = detail["booster"]["strakes"]
    mesh = vc.VMesh()
    for ri, (az, x0, x1) in enumerate(zip(sk["ridge_azimuth_deg"], sk["from_m"], sk["to_m"])):
        a = math.radians(az)
        et = Vector((0.0, -math.sin(a), math.cos(a)))
        # soft fairing: a straight ramp from full height down to the hull over `chamfer_m` (no pointed tip: the
        # width stays, only the height dies away flush with the shell, as the sloped end in the photos)
        ramp = [(x1 - sk["chamfer_m"] * (1.0 - j / STRAKE_RAMP_STEPS),
                 sk["height_m"] * (1.0 - j / STRAKE_RAMP_STEPS)) for j in range(STRAKE_RAMP_STEPS + 1)]
        stations = [(x0, sk["height_m"])] + ramp[:-1] + [(x1, 0.03)]
        secs = []
        for x, h in stations:
            t = max(h, 0.03) / sk["height_m"]
            wb, wt = sk["base_width_m"] / 2.0, sk["top_width_m"] / 2.0
            base = profile.R - EMBED_M
            secs.append([vc.pt(x, a, base) - et * wb, vc.pt(x, a, base) + et * wb,
                         vc.pt(x, a, profile.R + h) + et * wt, vc.pt(x, a, profile.R + h) - et * wt])
        for k in range(len(secs) - 1):
            for i in range(4):
                j = (i + 1) % 4
                quad = [secs[k][i], secs[k][j], secs[k + 1][j], secs[k + 1][i]]
                centre = sum((secs[k][m] + secs[k + 1][m] for m in range(4)), Vector((0.0, 0.0, 0.0))) / 8.0
                mid = sum(quad, Vector((0.0, 0.0, 0.0))) / 4.0
                mesh.face(quad, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)], mid - centre)
        for sec, sign in ((secs[0], -1.0), (secs[-1], 1.0)):
            mesh.face(sec, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)], X_AXIS * sign)
    return mesh


def _build_ports(stage, detail, profile):
    """Propellant transfer ports: two blocks on the lee side below the nose (position estimated)."""
    p = detail["ship"]["ports"]
    mesh = vc.VMesh()
    base = math.radians(stage["belly_azimuth_deg"] + p["azimuth_from_belly_deg"])
    n = int(p["port_count"])
    for i in range(n):
        a = base + math.radians(p["spread_deg"]) * (2.0 * i / max(n - 1, 1) - 1.0) if n > 1 else base
        x = p["placed_at_m"]
        r0 = profile.base(x) - EMBED_M
        h = p["port_height_m"]
        # round fitting (photos: a round coupling): collar, neck and a flanged cap, along the local normal
        vc.tube(mesh, vc.pt(x, a, r0), vc.pt(x, a, r0 + h * 0.45), p["port_radius_m"], PORT_SIDES, ("port_collar", i))
        vc.tube(mesh, vc.pt(x, a, r0 + h * 0.45), vc.pt(x, a, r0 + h * 0.8), p["port_radius_m"] * 0.7, PORT_SIDES,
                ("port_neck", i))
        vc.tube(mesh, vc.pt(x, a, r0 + h * 0.8), vc.pt(x, a, r0 + h), p["port_radius_m"] * 0.85, PORT_SIDES,
                ("port_cap", i))
    return mesh


def _build_catch_pins(stage, detail, profile):
    cp, d = stage["catch_pins"], detail["catch_pin"]
    mesh = vc.VMesh()
    r_out = profile.R + cp["protrusion_m"]
    for i, az in enumerate(cp["azimuths_deg"]):
        a = math.radians(az)
        x = cp["station_m"]
        vc.tube(mesh, vc.pt(x, a, profile.R - 0.1), vc.pt(x, a, r_out), d["radius_m"], PIN_SIDES, ("shaft", i))
        vc.tube(mesh, vc.pt(x, a, r_out - d["head_length_m"]), vc.pt(x, a, r_out), d["head_radius_m"], PIN_SIDES,
                ("head", i))
    return mesh


def _build_chine(stage, detail, profile):
    c, tiles = detail["ship"]["chine"], detail["ship"]["tiles"]
    mesh = vc.VMesh()
    x_first = c["first_station_m"]
    x_last = min(c["last_station_m"], profile.L - profile.cap_h - 0.5)
    xs = []
    x = x_first
    while x < x_last:
        xs.append(x)
        x += CHINE_STEP_M
    xs.append(x_last)
    belly = math.radians(stage["belly_azimuth_deg"])

    def section(x, side):
        half = math.radians(tile_half_angle_deg(tiles, x))
        a = belly + side * half
        r = profile.base(x) - EMBED_M
        w = c["width_m"] / 2.0 / max(profile.base(x), 0.5)
        fade = min(1.0, (x - x_first) / c["fade_m"], (x_last - x) / c["fade_m"])
        h = c["height_m"] * max(fade, 0.0) + EMBED_M
        return [vc.pt(x, a - w, r), vc.pt(x, a - w * 0.5, r + h), vc.pt(x, a + w * 0.5, r + h), vc.pt(x, a + w, r)], a

    for side in (-1.0, 1.0):
        for k in range(len(xs) - 1):
            s0, a0 = section(xs[k], side)
            s1, a1 = section(xs[k + 1], side)
            for q in range(3):
                pts = [s0[q], s0[q + 1], s1[q + 1], s1[q]]
                uvs = [(0.0, 1.0 - xs[k]), (0.1, 1.0 - xs[k]), (0.1, 1.0 - xs[k + 1]), (0.0, 1.0 - xs[k + 1])]
                mesh.face(pts, uvs, vc.radial_dir(0.5 * (a0 + a1)), ("chine", side, q))
    return mesh


def _build_raceway(stage, detail, profile):
    rw = detail["ship"]["raceway"]
    a = math.radians(stage["belly_azimuth_deg"] + rw["azimuth_from_belly_deg"])
    mesh = vc.VMesh()
    first, last = rw["first_station_m"], rw["last_station_m"]
    xs = [first + 2.0 * i for i in range(int((min(last, profile.x_barrel_end) - first) / 2.0) + 1)]
    xs += [x for x in profile.stations() if max(first, profile.x_barrel_end) < x < last and x - xs[-1] > 0.2]
    xs.append(last)
    r = rw["radius_m"]
    for k in range(len(xs) - 1):
        p0 = vc.pt(xs[k], a, profile.base(xs[k]) + r * 0.5)
        p1 = vc.pt(xs[k + 1], a, profile.base(xs[k + 1]) + r * 0.5)
        vc.tube(mesh, p0, p1, r, RACEWAY_SIDES, ("raceway", 0), cap0=k == 0, cap1=k == len(xs) - 2)
    er, et = vc.radial_dir(a), Vector((0.0, -math.sin(a), math.cos(a)))
    foot = rw["foot_width_m"] / 2.0  # wider feet: flat saddles under the tube every FOOT_STEP_M
    x = first + 0.5 * FOOT_STEP_M
    while x < last:
        vc.box(mesh, vc.pt(x, a, profile.base(x) + rw["foot_height_m"] / 2.0 - EMBED_M), X_AXIS, et, er,
               (rw["foot_length_m"] / 2.0, foot, rw["foot_height_m"] / 2.0 + EMBED_M / 2.0))
        x += FOOT_STEP_M
    return mesh


def build_stage(spec, stage_id):
    """Builds the stage's objects in the current scene; returns (root object, {node name: triangles})."""
    stage, detail = spec["stages"][stage_id], spec["detail"]
    profile = Profile(stage, detail["hull"])
    root = vc.empty(stage_id)
    parts = {"hull": _build_hull(stage, detail, profile)}
    if stage_id == "booster":
        parts["hot_stage"] = _build_hot_stage(stage, detail, profile)
        parts["top_truss"] = _build_top_truss(stage, detail, profile)
        parts["skirt"] = _build_skirt(stage, detail, profile)
        parts["raceway"] = _build_booster_raceway(stage, detail, profile)
        parts["strakes"] = _build_strakes(stage, detail, profile)
        if stage.get("grid_fins"):
            parts["fin_fairings"] = _build_fin_fairings(stage, detail, profile)
    else:
        parts["chine"] = _build_chine(stage, detail, profile)
        parts["raceway"] = _build_raceway(stage, detail, profile)
        parts["ports"] = _build_ports(stage, detail, profile)
    if stage.get("catch_pins"):
        parts["catch_pins"] = _build_catch_pins(stage, detail, profile)
    tris = {}
    for name, mesh in parts.items():
        tris[name] = mesh.to_object(name, root)[1]
    return root, tris


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    out = Path(args["out"])
    for stage_id in ("booster", "ship"):
        vc.common.clear_scene()
        root, tris = build_stage(spec, stage_id)
        vc.export_selected(out / f"{stage_id}.gltf", [root])
        vc.report_triangles(stage_id, sum(tris.values()))
        if "preview" in args:
            vc.common.render_preview(Path(args["preview"]).with_name(f"{stage_id}_{Path(args['preview']).name}"),
                                     (spec["stages"][stage_id]["length_m"] / 2.0, 0.0, 0.0), 120.0, 35.0, 20.0)


if __name__ == "__main__":
    main()
