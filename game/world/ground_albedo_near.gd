class_name GroundAlbedoNear
extends RefCounted
## Mean surface albedo of the ground around an Earth eye, as a profile over the horizontal distance and the azimuth from the
## eye, for the radiance cube's ground bounce (follow-ups of the Moon procedural surface plan, U-4; Task 13b measured the
## defect and wrote the first version). The cube's ground is drawn from the global day texture (5400 x 2700: one texel ~ 6.7 km
## at Starbase, its mean over land, bay and Gulf: 0.108 against 0.20 of the ground within a hundred metres), which left a
## wall's ground bounce 30-40 % low: the cube is drawn at the eye, and the light a vertical face there receives from the
## ground comes mostly from the ground near the eye (cosine-weighted over the lower hemisphere, the share of the ground beyond
## a horizontal distance R is ~ h / R of the eye's height h) and mostly from the side the face looks at. Where the imagery
## pyramids cover the eye, the mean of their pixels in each cell of SECTORS azimuth sectors x RING_EDGES_M's annuli around
## the eye is the albedo of the ground there (sky.gdshader interpolates between the cells and fades to the global texture with
## distance); elsewhere no profile exists and the cube is unchanged.
## The imagery is read on a WorkerThreadPool task (two crops of up to 3 x 3 JPEG tiles each).

## [E] Outer edges of the annuli (m, horizontal distance) around the eye: the ground under a low eye (0-25: a face at 2 m
## gets ~ 3/4 of its bounce from within 25 m), the hardstand / pad (25-100), the pad's surroundings, the flats, the beach and
## the bay (up to the box). ~ geometric, as the ground's contribution falls with distance.
const RING_EDGES_M := [0.0, 25.0, 100.0, 250.0, 700.0, 1500.0]
## [E] Azimuth sectors (centred on north, then every 360 / SECTORS deg toward east): at Pad B the ground a face sees differs
## by azimuth (Gulf and beach to the east, flats and bay to the west); ground_albedo_weight_probe.gd measures it.
const SECTORS := 8
## Samples per cell: equal-area radii x angles (equal area: the plain mean of the samples is the cell mean).
const CELL_RADII := 12
const CELL_ANGLES := 16
## [E] Requested pixel size (m) of the crop for the inner rings (0-250 m: NAIP, ~1.2 m after the 3 x 3 tile limit) and of the
## crop for the whole box (Sentinel-2 / NAIP overview): the 25 m annulus needs pixels well below its width.
const INNER_TEXEL_M := 0.6
const INNER_RINGS := 3
const TEXEL_M := 30.0
## [E] A new profile after the eye moved this far (m): half the inner ring's width, so the 0-25 m ring (~ 3/4 of a 2 m eye's
## bounce) is never more than half stale; higher up REFRESH_HEIGHT_FRACTION of the eye's height (an eye at height h gets its
## bounce mostly from within a few h, the near rings matter less). A read is 10-20 ms on a worker.
const REFRESH_MIN_M := 12.5
const REFRESH_HEIGHT_FRACTION := 0.5
## Metres per degree of latitude [S: WGS84 mean].
const METRES_PER_DEGREE := 111132.0

## Off: World draws the cube's ground from the global day texture alone (render_check --ground_albedo_near=off, cost and
## before / after views).
static var enabled := true

var _imagery: ImagerySource
var _last_lat_deg := 0.0
var _last_lon_deg := 0.0
var _value := PackedVector3Array()  # the cell albedos (linear RGB) of the last finished read; empty: none
var _valid := false  # a read has been started for _last_*: _value belongs to it once the task is collected
var _task_id := -1
var _task_result := [PackedVector3Array()]  # [profile, ms]: written by the worker, read only after the task completed
var _task_ms := 0.0  # wall time of the last finished read on the worker (diagnostics)
var _reads := 0


func _init(imagery: ImagerySource) -> void:
	_imagery = imagery


## Number of annuli.
static func ring_count() -> int:
	return RING_EDGES_M.size() - 1


## Number of cells of a profile (sector-major: cell s * ring_count() + k is sector s, ring k).
static func profile_size() -> int:
	return SECTORS * ring_count()


## The cell albedos (linear RGB, profile_size() of them) around the point, empty where the imagery does not cover the whole
## box. The imagery is read on a worker: the profile of the last finished read is returned meanwhile (empty before the first),
## and a new read starts when the point moved refresh_distance_m(height_m) from the last read's point and no read is running.
## height_m: the eye's height (m; over the ellipsoid is good enough for the refresh distance).
func at(lat_deg: float, lon_deg: float, height_m := 0.0) -> PackedVector3Array:
	_collect()
	var moved_m := Vector2((lon_deg - _last_lon_deg) * cos(deg_to_rad(lat_deg)), lat_deg - _last_lat_deg).length() \
			* METRES_PER_DEGREE
	if _task_id < 0 and (not _valid or moved_m >= refresh_distance_m(height_m)):
		_valid = true
		_last_lat_deg = lat_deg
		_last_lon_deg = lon_deg
		_reads += 1
		_task_result = [PackedVector3Array()]
		_task_id = WorkerThreadPool.add_task(_read.bind(lat_deg, lon_deg, _task_result), false, "ground albedo near")
	return _value


## The distance (m) the eye moves before a new profile is read, for an eye at height_m.
static func refresh_distance_m(height_m: float) -> float:
	return maxf(REFRESH_MIN_M, REFRESH_HEIGHT_FRACTION * height_m)


## Number of reads started (tests, diagnostics) and the last read's wall time on the worker (ms).
func reads() -> int:
	return _reads


func last_read_ms() -> float:
	return _task_ms


## Blocks until a running read has finished and takes its profile (tests, shutdown).
func finish() -> PackedVector3Array:
	if _task_id >= 0:
		WorkerThreadPool.wait_for_task_completion(_task_id)
		_take()
	return _value


func _collect() -> void:
	if _task_id >= 0 and WorkerThreadPool.is_task_completed(_task_id):
		WorkerThreadPool.wait_for_task_completion(_task_id)  # joins the finished task (returns at once)
		_take()


func _take() -> void:
	_task_id = -1
	_value = _task_result[0]
	_task_ms = _task_result[1] if _task_result.size() > 1 else 0.0


func _read(lat_deg: float, lon_deg: float, result: Array) -> void:
	var t0 := Time.get_ticks_usec()
	var value := ring_albedo(_imagery, lat_deg, lon_deg)
	result.append(float(Time.get_ticks_usec() - t0) / 1000.0)
	result[0] = value


## Uncached: the mean of the imagery's linear colour in each sector x annulus cell around the point (the colours are the
## calibrated surface reflectance, sRGB encoded: Task 15), empty without imagery covering the whole box. The inner rings come
## from a fine crop where one covers them, else from the box's crop.
static func ring_albedo(imagery: ImagerySource, lat_deg: float, lon_deg: float) -> PackedVector3Array:
	if imagery == null or imagery.is_empty():
		return PackedVector3Array()
	var outer := _crop(imagery, lat_deg, lon_deg, RING_EDGES_M[RING_EDGES_M.size() - 1], TEXEL_M)
	if outer.is_empty():
		return PackedVector3Array()
	var inner := _crop(imagery, lat_deg, lon_deg, RING_EDGES_M[INNER_RINGS], INNER_TEXEL_M)
	if inner.is_empty():
		inner = outer
	var cos_lat := maxf(cos(deg_to_rad(lat_deg)), 0.01)
	var sector_rad := TAU / SECTORS
	var profile := PackedVector3Array()
	for s in range(SECTORS):
		for k in range(ring_count()):
			var crop: Dictionary = inner if k < INNER_RINGS else outer
			var image: Image = crop["image"]
			var image_box: Rect2 = crop["box_deg"]
			var r0: float = RING_EDGES_M[k]
			var r1: float = RING_EDGES_M[k + 1]
			var sum := Vector3.ZERO
			var count := 0
			for ir in range(CELL_RADII):
				var r := sqrt(r0 * r0 + (r1 * r1 - r0 * r0) * (ir + 0.5) / CELL_RADII)  # equal areas
				for ia in range(CELL_ANGLES):
					# azimuth from north toward east, the sector centred on s * sector_rad; every other radius offset
					var azimuth := (s - 0.5 + (ia + 0.5 + 0.5 * (ir % 2)) / CELL_ANGLES) * sector_rad
					var lon := lon_deg + r * sin(azimuth) / (METRES_PER_DEGREE * cos_lat)
					var lat := lat_deg + r * cos(azimuth) / METRES_PER_DEGREE
					var x := int((lon - image_box.position.x) / image_box.size.x * image.get_width())
					var y := int((image_box.end.y - lat) / image_box.size.y * image.get_height())
					if x < 0 or y < 0 or x >= image.get_width() or y >= image.get_height():
						continue
					var c := image.get_pixel(x, y)
					sum += Vector3(_decode(c.r), _decode(c.g), _decode(c.b))
					count += 1
			if count == 0:
				return PackedVector3Array()
			profile.append(sum / float(count))
	return profile


## The imagery crop of the square of half-size half_m around the point at the wanted texel, {} unless one source covers it all
## (tiles are black outside a source's coverage).
static func _crop(imagery: ImagerySource, lat_deg: float, lon_deg: float, half_m: float, texel_m: float) -> Dictionary:
	var half_lat := half_m / METRES_PER_DEGREE
	var half_lon := half_lat / maxf(cos(deg_to_rad(lat_deg)), 0.01)
	var box := Rect2(lon_deg - half_lon, lat_deg - half_lat, 2.0 * half_lon, 2.0 * half_lat)
	var crop := imagery.crop(box, texel_m / METRES_PER_DEGREE)
	if crop.is_empty() or not (crop["coverage_deg"] as Rect2).encloses(box):
		return {}
	return crop


## The profile at the horizontal distance r_m and the azimuth (rad, from north toward east): the cell values held at the rings'
## mid-distances and the sectors' centres, interpolated linearly in between (the same rule as sky.gdshader's
## radiance_local_ground_albedo, which draws it).
static func profile_at(profile: PackedVector3Array, r_m: float, azimuth_rad: float) -> Vector3:
	var u := fposmod(azimuth_rad / (TAU / SECTORS), float(SECTORS))
	var s0 := int(floor(u)) % SECTORS
	var s1 := (s0 + 1) % SECTORS
	return _ring_at(profile, s0, r_m).lerp(_ring_at(profile, s1, r_m), u - floor(u))


static func _ring_at(profile: PackedVector3Array, sector: int, r_m: float) -> Vector3:
	var n := ring_count()
	var base := sector * n
	if r_m <= 0.5 * (RING_EDGES_M[0] + RING_EDGES_M[1]):
		return profile[base]
	for k in range(n - 1):
		var mid0: float = 0.5 * (RING_EDGES_M[k] + RING_EDGES_M[k + 1])
		var mid1: float = 0.5 * (RING_EDGES_M[k + 1] + RING_EDGES_M[k + 2])
		if r_m <= mid1:
			return profile[base + k].lerp(profile[base + k + 1], (r_m - mid0) / (mid1 - mid0))
	return profile[base + n - 1]


## The mean ground albedo (linear RGB; the global texture's `global_albedo` beyond) that a vertical face of azimuth face_rad (its
## normal, from north toward east) at the eye gets its ground bounce from, for an eye at height h over a flat ground: the
## view-factor weighted mean over the lower half-space the face sees, dF ~ sin^2(beta) max(cos(phi - face), 0) dbeta dphi (beta
## from the nadir, the ground point at the horizontal distance r = h tan(beta)), the albedo as sky.gdshader draws it (the
## profile, blended to the texture between fade_near_m and fade_far_m). A diagnostic of the model (probe, radiance_budget.gd).
static func face_albedo(profile: PackedVector3Array, eye_height_m: float, face_rad: float, global_albedo: Vector3,
		fade_near_m := 1500.0, fade_far_m := 6000.0) -> Vector3:
	var local := profile.size() == profile_size()
	if not local:
		return global_albedo
	var beta_steps := 300
	var phi_steps := 48  # over the half circle the face sees
	var sum := Vector3.ZERO
	var weight := 0.0
	for ib in range(beta_steps):
		var beta := (ib + 0.5) / beta_steps * PI / 2.0
		var r := eye_height_m * tan(beta)
		var fade := smoothstep(fade_near_m, fade_far_m, r)
		var w_beta := sin(beta) * sin(beta)
		for ip in range(phi_steps):
			var dphi := ((ip + 0.5) / phi_steps - 0.5) * PI
			var w := w_beta * cos(dphi)
			var a := global_albedo
			if fade < 1.0:
				a = profile_at(profile, r, face_rad + dphi).lerp(global_albedo, fade)
			sum += a * w
			weight += w
	return sum / weight


## sRGB (IEC 61966-2-1) to linear.
static func _decode(c: float) -> float:
	return c / 12.92 if c <= 0.04045 else pow((c + 0.055) / 1.055, 2.4)
