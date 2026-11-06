class_name GroundAlbedoNear
extends RefCounted
## Mean surface albedo of the ground around an Earth eye, as a profile over the distance from the eye, for the radiance cube's
## ground bounce (follow-ups of the Moon procedural surface plan, U-4; Task 13b measured the defect and wrote the first
## version). The cube's ground is drawn from the global day texture (5400 x 2700: one texel ~ 6.7 km at Starbase, its mean over
## land, bay and Gulf: 0.108 against 0.20 of the ground within a hundred metres), which left a wall's ground bounce 30-40 %
## low: the cube is drawn at the eye, and the light a vertical face there receives from the ground comes mostly from the
## ground near the eye (cosine-weighted over the lower hemisphere, the share of the ground beyond a distance R is ~ h / R of
## the eye's height h). Where the imagery pyramids cover the eye, the mean of their pixels in RING_EDGES_M's annuli around
## the eye is the albedo of the ground at that distance (sky.gdshader interpolates between the rings and fades to the global
## texture with distance); elsewhere no profile exists and the cube is unchanged.
## The imagery is read on a WorkerThreadPool task (a crop of up to 3 x 3 JPEG tiles).

## [E] Outer edges of the annuli (m) around the eye: the hardstand / pad (0-100), the pad's surroundings, the flats, the beach
## and the bay (up to the box). ~ geometric, as the ground's contribution falls with distance.
const RING_EDGES_M := [0.0, 100.0, 250.0, 700.0, 1500.0]
## Samples per annulus: equal-area radii x angles (equal area: the plain mean of the samples is the annulus mean).
const RING_RADII := 8
const RING_ANGLES := 48
## [E] Pixel size of the sampled mosaic (m): the 100 m annulus needs ~ a third of it.
const TEXEL_M := 30.0
## [E] A new profile after the eye moved this far (m): a tenth of the outer edge.
const REFRESH_DISTANCE_M := 150.0
## Metres per degree of latitude [S: WGS84 mean].
const METRES_PER_DEGREE := 111132.0

var _imagery: ImagerySource
var _last_lat_deg := 0.0
var _last_lon_deg := 0.0
var _value := PackedVector3Array()  # the ring albedos (linear RGB) of the last finished read; empty: none
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


## The ring albedos (linear RGB, ring_count() of them) around the point, empty where the imagery does not cover the whole
## box. The imagery is read on a worker: the profile of the last finished read is returned meanwhile (empty before the first),
## and a new read starts when the point moved REFRESH_DISTANCE_M from the last read's point and no read is running.
func at(lat_deg: float, lon_deg: float) -> PackedVector3Array:
	_collect()
	var moved_m := Vector2((lon_deg - _last_lon_deg) * cos(deg_to_rad(lat_deg)), lat_deg - _last_lat_deg).length() \
			* METRES_PER_DEGREE
	if _task_id < 0 and (not _valid or moved_m >= REFRESH_DISTANCE_M):
		_valid = true
		_last_lat_deg = lat_deg
		_last_lon_deg = lon_deg
		_reads += 1
		_task_result = [PackedVector3Array()]
		_task_id = WorkerThreadPool.add_task(_read.bind(lat_deg, lon_deg, _task_result), false, "ground albedo near")
	return _value


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


## Uncached: the mean of the imagery's linear colour in each annulus around the point (the colours are the calibrated surface
## reflectance, sRGB encoded: Task 15), empty without imagery covering the whole box.
static func ring_albedo(imagery: ImagerySource, lat_deg: float, lon_deg: float) -> PackedVector3Array:
	if imagery == null or imagery.is_empty():
		return PackedVector3Array()
	var half_m: float = RING_EDGES_M[RING_EDGES_M.size() - 1]
	var half_lat := half_m / METRES_PER_DEGREE
	var half_lon := half_lat / maxf(cos(deg_to_rad(lat_deg)), 0.01)
	var box := Rect2(lon_deg - half_lon, lat_deg - half_lat, 2.0 * half_lon, 2.0 * half_lat)
	var crop := imagery.crop(box, TEXEL_M / METRES_PER_DEGREE)
	if crop.is_empty():
		return PackedVector3Array()
	if not (crop["coverage_deg"] as Rect2).encloses(box):  # tiles are black outside a source's coverage
		return PackedVector3Array()
	var image: Image = crop["image"]
	var image_box: Rect2 = crop["box_deg"]
	var cos_lat := maxf(cos(deg_to_rad(lat_deg)), 0.01)
	var rings := PackedVector3Array()
	for k in range(ring_count()):
		var r0: float = RING_EDGES_M[k]
		var r1: float = RING_EDGES_M[k + 1]
		var sum := Vector3.ZERO
		var count := 0
		for ir in range(RING_RADII):
			var r := sqrt(r0 * r0 + (r1 * r1 - r0 * r0) * (ir + 0.5) / RING_RADII)  # equal areas
			for ia in range(RING_ANGLES):
				var angle := (ia + 0.5 + 0.5 * (ir % 2)) / RING_ANGLES * TAU  # offset every other radius
				var lon := lon_deg + r * cos(angle) / (METRES_PER_DEGREE * cos_lat)
				var lat := lat_deg + r * sin(angle) / METRES_PER_DEGREE
				var x := int((lon - image_box.position.x) / image_box.size.x * image.get_width())
				var y := int((image_box.end.y - lat) / image_box.size.y * image.get_height())
				if x < 0 or y < 0 or x >= image.get_width() or y >= image.get_height():
					continue
				var c := image.get_pixel(x, y)
				sum += Vector3(_decode(c.r), _decode(c.g), _decode(c.b))
				count += 1
		if count == 0:
			return PackedVector3Array()
		rings.append(sum / float(count))
	return rings


## The profile at distance t_m: the ring values held at the rings' mid-distances and interpolated linearly in between (the
## same rule as sky.gdshader's radiance_local_ground_albedo, which draws it).
static func profile_at(rings: PackedVector3Array, t_m: float) -> Vector3:
	var last := ring_count() - 1
	var mid0: float = 0.5 * (RING_EDGES_M[0] + RING_EDGES_M[1])
	if t_m <= mid0:
		return rings[0]
	for k in range(last):
		mid0 = 0.5 * (RING_EDGES_M[k] + RING_EDGES_M[k + 1])
		var mid1: float = 0.5 * (RING_EDGES_M[k + 1] + RING_EDGES_M[k + 2])
		if t_m <= mid1:
			return rings[k].lerp(rings[k + 1], (t_m - mid0) / (mid1 - mid0))
	return rings[last]


## The mean ground albedo (one channel, `component` 0..2 of the rings; the global texture's `global_albedo` beyond) that a vertical
## face at the eye gets its ground bounce from, for an eye at height h over a flat ground: the cosine-weighted mean over the
## lower hemisphere of the albedo at the distance r = h / tan(depression), the albedo as sky.gdshader draws it (the profile,
## blended to the texture between fade_near_m and fade_far_m). A diagnostic of the model (radiance_budget.gd).
static func face_bounce_albedo(rings: PackedVector3Array, component: int, eye_height_m: float, global_albedo: float,
		fade_near_m := 1500.0, fade_far_m := 6000.0) -> float:
	var steps := 4000
	var sum := 0.0
	var weight := 0.0
	for k in range(steps):
		var depression := (k + 0.5) / steps * PI / 2.0
		var r := eye_height_m / tan(depression)
		var local := rings.size() == ring_count()
		var a := global_albedo
		if local:
			a = lerpf(profile_at(rings, r)[component], global_albedo, smoothstep(fade_near_m, fade_far_m, r))
		var w := cos(depression) * cos(depression)
		sum += a * w
		weight += w
	return sum / weight


## sRGB (IEC 61966-2-1) to linear.
static func _decode(c: float) -> float:
	return c / 12.92 if c <= 0.04045 else pow((c + 0.055) / 1.055, 2.4)
