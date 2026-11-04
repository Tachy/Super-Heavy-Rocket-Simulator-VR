# Sun elevation at a lunar site over a span of days, against the terrain's horizon in the Sun's azimuth (helper of
# tools/lunar_sun_scan.py, which documents the arguments and reads this output). Needs the lunar terrain data and the SPICE kernels.
# Run: godot --headless --path game --script res://tests/lunar_sun_scan.gd -- --lat=-84.7906 --lon=29.1957 --start=2026-11-02T00:00:00 --days=33 --step_min=60
# Output lines: "scan: <hours since start> <sun elevation deg> <sun azimuth deg> <horizon deg>"
extends SceneTree

## Height above the ground of the evaluation point (m) [E]: the lander's and the eye's order of magnitude; the horizon is tied to the site.
const EYE_HEIGHT_M := 2.0


func _initialize() -> void:
	var args := {}
	for a in OS.get_cmdline_user_args():
		var kv := String(a).trim_prefix("--").split("=", true, 1)
		args[kv[0]] = kv[1] if kv.size() > 1 else ""
	var lat_deg := float(args.get("lat", "-84.7906"))
	var lon_deg := float(args.get("lon", "29.1957"))
	var days := float(args.get("days", "33"))
	var step_s := float(args.get("step_min", "60")) * 60.0
	var root_dir := ProjectSettings.globalize_path("res://").path_join("..").simplify_path()
	var core := StarsimCore.new()
	if not core.open(root_dir.path_join("data/spice")):
		push_error("lunar_sun_scan: no SPICE kernels in data/spice")
		quit(2)
		return
	var present: Dictionary = core.load_terrain(root_dir.path_join("config"))
	if not present.get("moon", false):
		push_error("lunar_sun_scan: no lunar terrain data")
		quit(2)
		return
	var t0: float = core.utc_to_tdb_s(String(args.get("start", "2026-11-02T00:00:00")))
	var ground: Dictionary = core.surface_samples("moon", PackedFloat64Array([lat_deg]), PackedFloat64Array([lon_deg]))
	var height_m := float(ground["height_m"][0]) + EYE_HEIGHT_M
	var lat := deg_to_rad(lat_deg)
	var lon := deg_to_rad(lon_deg)
	var up := Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
	var east := Vector3(-sin(lon), cos(lon), 0.0)
	var north := up.cross(east)
	var point_fixed := up * (Atmosphere.MOON_RADIUS_M + height_m)
	print("scan_site: lat %.4f lon %.4f ground+eye height %.1f m" % [lat_deg, lon_deg, height_m])
	var t := 0.0
	while t <= days * 86400.0:
		var tdb := t0 + t
		var to_fixed: Basis = core.moon_fixed_to_gcrf(tdb).transposed()
		var moon_gcrf: Vector3 = core.body_position_gcrf_m("moon", tdb)
		var sun_gcrf: Vector3 = core.apparent_body_position_gcrf_m("sun", tdb, Vector3.ZERO, Vector3.ZERO)
		var dir := (to_fixed * (sun_gcrf - moon_gcrf) - point_fixed).normalized()
		var az := atan2(dir.dot(east), dir.dot(north))
		var horizon: float = core.horizon_elevation_rad("moon", lat_deg, lon_deg, height_m, az)
		print("scan: %.3f %.4f %.2f %.4f" % [t / 3600.0, rad_to_deg(asin(dir.dot(up))), fposmod(rad_to_deg(az), 360.0), rad_to_deg(horizon)])
		t += step_s
	quit(0)
