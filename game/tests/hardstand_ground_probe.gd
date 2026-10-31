# Terrain (physics ground) heights under the Pad B hardstand outline: along its edges and over its interior, against the site plateau.
# godot --headless --path game --script res://tests/hardstand_ground_probe.gd
# Prints the outline vertices (east / north of the ring centre, ground height, difference to the site height), the road-side edge
# samples, and a least-squares plane over the interior with its residuals.
extends SceneTree

const SITE_LAT_DEG := 25.99677
const SITE_LON_DEG := -97.15804
const WGS84_A_M := 6378137.0
const WGS84_E2 := 0.00669437999014
const EDGE_STEP_M := 10.0
const GRID_STEP_M := 20.0
const OUTLINE := [[-35.7, -74.4], [-1.0, -57.2], [35.7, -34.0], [99.3, -13.9], [152.4, 9.4], [157.9, 9.5], [171.5, 33.2],
		[183.9, 82.9], [178.9, 111.4], [152.3, 128.8], [125.3, 137.3], [65.4, 156.3], [3.2, 155.1], [-29.7, 160.2],
		[-59.8, 147.2], [-63.4, 138.8], [-59.9, 122.0], [-63.4, 102.6], [-45.9, 40.6], [-43.4, -22.4]]
const ROAD_FIRST := 8  ## outline index where the road side starts (895,270 in the sketch) ... to ROAD_LAST
const ROAD_LAST := 14

var _core := StarsimCore.new()


func _initialize() -> void:
	var config_dir := ProjectSettings.globalize_path("res://").path_join("../config").simplify_path()
	var present: Dictionary = _core.load_terrain(config_dir)
	print("terrain present: ", present)
	var site_h := _height(0.0, 0.0)
	print("site (ring centre) ground %.3f m" % site_h)
	for i in OUTLINE.size():
		var p: Array = OUTLINE[i]
		var h := _height(p[0], p[1])
		print("vertex %2d  E %7.1f N %7.1f  r %6.1f  h %8.3f  dh %6.3f" % [i, p[0], p[1], Vector2(p[0], p[1]).length(), h, h - site_h])
	print("-- road side edge samples (index %d .. %d)" % [ROAD_FIRST, ROAD_LAST])
	var road: Array = []
	for i in range(ROAD_FIRST, ROAD_LAST):
		var a := Vector2(OUTLINE[i][0], OUTLINE[i][1])
		var b := Vector2(OUTLINE[i + 1][0], OUTLINE[i + 1][1])
		var n := int(ceil(a.distance_to(b) / EDGE_STEP_M))
		for k in n:
			var q := a.lerp(b, float(k) / n)
			var h := _height(q.x, q.y)
			road.append(h)
			print("road  E %7.1f N %7.1f  h %8.3f  dh %6.3f" % [q.x, q.y, h, h - site_h])
	print("road mean %.3f min %.3f max %.3f" % [_mean(road), road.min(), road.max()])
	# interior grid
	var pts: Array = []
	var poly := PackedVector2Array()
	for p in OUTLINE:
		poly.append(Vector2(p[0], p[1]))
	var x := -70.0
	while x <= 190.0:
		var y := -80.0
		while y <= 165.0:
			if Geometry2D.is_point_in_polygon(Vector2(x, y), poly):
				pts.append(Vector3(x, y, _height(x, y)))
			y += GRID_STEP_M
		x += GRID_STEP_M
	var hs: Array = []
	for p in pts:
		hs.append(p.z)
	print("interior %d samples: mean %.3f min %.3f max %.3f (site %.3f)" % [pts.size(), _mean(hs), hs.min(), hs.max(), site_h])
	# least squares plane h = c + gx x + gy y
	var sxx := 0.0; var sxy := 0.0; var syy := 0.0; var sx := 0.0; var sy := 0.0; var sh := 0.0; var sxh := 0.0; var syh := 0.0
	for p in pts:
		sx += p.x; sy += p.y; sh += p.z; sxx += p.x * p.x; sxy += p.x * p.y; syy += p.y * p.y; sxh += p.x * p.z; syh += p.y * p.z
	var n := float(pts.size())
	var m := Basis(Vector3(sxx, sxy, sx), Vector3(sxy, syy, sy), Vector3(sx, sy, n))
	var sol := m.inverse() * Vector3(sxh, syh, sh)  # (gx, gy, c)
	var worst := 0.0
	for p in pts:
		worst = max(worst, absf(p.z - (sol.z + sol.x * p.x + sol.y * p.y)))
	print("plane: gx %.5f gy %.5f c %.3f  (slope %.3f %%), max residual %.3f m" % [sol.x, sol.y, sol.z, 100.0 * Vector2(sol.x, sol.y).length(), worst])
	quit(0)


func _height(east_m: float, north_m: float) -> float:
	var lat := deg_to_rad(SITE_LAT_DEG)
	var s := sin(lat)
	var w := sqrt(1.0 - WGS84_E2 * s * s)
	var m_per_deg_lat := deg_to_rad(1.0) * WGS84_A_M * (1.0 - WGS84_E2) / (w * w * w)
	var m_per_deg_lon := deg_to_rad(1.0) * WGS84_A_M * cos(lat) / w
	var g: Dictionary = _core.surface_samples("earth", PackedFloat64Array([SITE_LAT_DEG + north_m / m_per_deg_lat]),
			PackedFloat64Array([SITE_LON_DEG + east_m / m_per_deg_lon]))
	return float(g["height_m"][0]) if bool(g["present"]) else NAN


func _mean(a: Array) -> float:
	var s := 0.0
	for v in a:
		s += v
	return s / a.size()
