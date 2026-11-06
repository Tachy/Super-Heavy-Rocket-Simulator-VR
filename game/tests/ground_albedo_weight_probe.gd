extends SceneTree
## Probe (U-4): the wall irradiance the radiance cube's ground bounce gives, against what the imagery's ground gives a vertical
## face at the same height, at Pad B. The cube is drawn at the eye (height h); for a vertical face there the ground bounce is
## the cosine-weighted mean of the ground radiance over the lower hemisphere; with the ground albedo a(r) at the horizontal
## distance r = h / tan(beta) (beta: the depression below the horizon) and the face's cosine cos(beta) cos(phi):
##   E_bounce ~ int a(r(beta)) cos^2(beta) dbeta over 0..90 deg, normalised by pi / 4 (the mean albedo for a uniform ground).
## "truth": the same face's view-factor weighting over the imagery's own pixels (the 30 m crop, 6 km box, four orientations),
## dF = cos(e) cos(g) dA / (pi d^2) for a face element at height h; no profile, no model. "cube": a(r) as sky.gdshader draws it
## (the ring profile, then the blend to the global day texture 1500 .. 6000 m) or, "global", the day texture alone (luminance 0.101).
##   godot --headless --path game --script res://tests/ground_albedo_weight_probe.gd
## Prints a table; exit code 0 (a probe, not a gate).

const LAT_DEG := 25.99677
const LON_DEG := -97.15804
const METRES_PER_DEGREE := 111132.0
const HALF_M := 3000.0
const STEP_M := 30.0
## The global day texture's linear RGB at the site (task 13b).
const GLOBAL_CHANNEL := [0.083, 0.108, 0.083]


func _initialize() -> void:
	var imagery := ImagerySource.new(Paths.data_dir().path_join("imagery/earth/index.toml"))
	var half_lat := HALF_M / METRES_PER_DEGREE
	var half_lon := half_lat / cos(deg_to_rad(LAT_DEG))
	var box := Rect2(LON_DEG - half_lon, LAT_DEG - half_lat, 2.0 * half_lon, 2.0 * half_lat)
	var crop := imagery.crop(box, STEP_M / METRES_PER_DEGREE)
	var image: Image = crop["image"]
	var image_box: Rect2 = crop["box_deg"]
	print("ground_albedo_weight_probe: crop %dx%d, source %s level %d" % [image.get_width(), image.get_height(),
			crop["source_id"], crop["level"]])
	var n := int(2.0 * HALF_M / STEP_M)
	var albedo := PackedFloat64Array()  # per cell: linear luminance
	for j in range(n):
		for i in range(n):
			var x_m := -HALF_M + (i + 0.5) * STEP_M  # east
			var y_m := HALF_M - (j + 0.5) * STEP_M  # north
			var lon := LON_DEG + x_m / (METRES_PER_DEGREE * cos(deg_to_rad(LAT_DEG)))
			var lat := LAT_DEG + y_m / METRES_PER_DEGREE
			var px := clampi(int((lon - image_box.position.x) / image_box.size.x * image.get_width()), 0, image.get_width() - 1)
			var py := clampi(int((image_box.end.y - lat) / image_box.size.y * image.get_height()), 0, image.get_height() - 1)
			var c := image.get_pixel(px, py)
			albedo.append(_luma(Vector3(GroundAlbedoNear._decode(c.r), GroundAlbedoNear._decode(c.g), GroundAlbedoNear._decode(c.b))))
	var rings := GroundAlbedoNear.ring_albedo(imagery, LAT_DEG, LON_DEG)
	var ring_luma := []
	for ring in rings:
		ring_luma.append(snappedf(_luma(ring), 0.0001))
	print("  ring albedo luminance (0-100, 100-250, 250-700, 700-1500 m): %s; global day texture %.3f" % [ring_luma, _luma(Vector3(GLOBAL_CHANNEL[0], GLOBAL_CHANNEL[1], GLOBAL_CHANNEL[2]))])
	print("  eye height h | truth (imagery, wall at h) | cube with the profile (error) | cube, global texture only (error)")
	for h: float in [2.0, 10.0, 40.0, 75.0, 148.0]:
		var truth := _truth(albedo, n, h)
		var cube := _cube(rings, h, true)
		var global := _cube(rings, h, false)
		print("  %6.0f m     | %.4f                     | %.4f  (%+.1f %%)             | %.4f  (%+.1f %%)" % [h, truth, cube,
				100.0 * (cube / truth - 1.0), global, 100.0 * (global / truth - 1.0)])
	quit(0)


## The face's view-factor weighted mean of the imagery's ground (four orientations), wall element at height h.
func _truth(albedo: PackedFloat64Array, n: int, h: float) -> float:
	var sum := 0.0
	var weight := 0.0
	for orient in range(4):
		var normal := Vector2.from_angle(orient * PI / 2.0)
		for j in range(n):
			for i in range(n):
				var x_m := -HALF_M + (i + 0.5) * STEP_M
				var y_m := HALF_M - (j + 0.5) * STEP_M
				var along := normal.x * x_m + normal.y * y_m
				if along <= 0.0:
					continue
				var d2 := x_m * x_m + y_m * y_m + h * h
				var w := along * h / (PI * d2 * d2)
				sum += albedo[j * n + i] * w
				weight += w
	return sum / weight


## The cube's weighted mean luminance for an eye at height h: the profile blended to the global texture (with_profile), or
## the texture alone. The weighted sum of the channels' face albedos.
func _cube(rings: PackedVector3Array, h: float, with_profile: bool) -> float:
	var profile := rings if with_profile else PackedVector3Array()
	var y := 0.0
	var weights := [0.2126, 0.7152, 0.0722]
	for c in range(3):
		y += weights[c] * GroundAlbedoNear.face_bounce_albedo(profile, c, h, GLOBAL_CHANNEL[c])
	return y


func _luma(rgb: Vector3) -> float:
	return 0.2126 * rgb.x + 0.7152 * rgb.y + 0.0722 * rgb.z
