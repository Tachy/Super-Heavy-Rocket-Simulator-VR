class_name AdaptationSurround
extends RefCounted
## The eye adaptation's field beyond the frame (look fixes T4, F4): the AdaptationMeter's histograms hold only the rendered
## frame, so a view up into the noon sky metered the sky alone and doubled the gain (field 2354 against 4642 cd/m^2 with
## the horizon and the ground, gain 3.30 against 1.67). The eye's adaptation state integrates a wider surround than the
## ~50 deg the desktop frame shows (eye movements, the periphery). This class adds the directions within CAP_HALF_ANGLE_DEG
## of the view axis that the frame does not show, each with the luminance an open view would show there, to the frame's
## histograms; the field statistics (AdaptationMeter.field_log_average, field_anchor: the 2 % highlight cut, the 3.7-decade
## range, the anchor percentile) and the rule (Atmosphere.field_adaptation_luminance) then run on the wider field unchanged.
##
## Where the luminance comes from: Godot's sky radiance cube (the sky and the model ground, scene values) is engine-internal
## (SkyRD::sky_get_radiance_texture_rd has no script binding), so a second source stands in for it: the analytic model the
## cube is drawn from, evaluated on the CPU (Atmosphere._surround_luminance: the core's sky luminance along the ray, or the
## model ground lit by the Sun and the sky and seen through the air with the near imagery's albedo, both times the cube's
## cloud factors; near the Moon the regolith of the eye adaptation's Moon term, the sky black). It is sampled into a coarse
## store of directions about the eye's local vertical (STORE_BANDS x STORE_AZIMUTHS cells, refreshed CELLS_PER_FRAME per
## frame; a complete store is swapped in, so a refresh never mixes two states), not drawn again on the GPU.
##
## Enclosed views (the cabin, under a hull): the model knows no walls. The out-of-frame part is weighted by a visibility V
## from how well the model agrees with the frame where both look: V = 1 while the frame's field and the model's field over
## the in-frame directions agree within VISIBILITY_FULL_DECADES, 0 beyond VISIBILITY_NONE_DECADES (smoothstep). Both fields
## are the same statistic (AdaptationMeter.field_log_average with the cut of the whole weight, black included, so a small lit object only one field has - the Earth's disc in a lunar sky - is no disagreement; the 3.7-decade exclusion; the model's
## in-frame directions as a histogram with the frame's black bin). The disagreement in decades is low-passed in time
## (VISIBILITY_TIME_S; a view jump restarts it), so panning across a structure does not fade the surround in and out; a black
## field counts as the histogram's floor (LOG_MIN), so the measure is continuous. In the cabin (field 16 cd/m^2 against
## ~3000 outside) V = 0: the frame alone, as before; under the open sky V = 1.
## Directions the model shows black (space, the lunar sky) add no weight: no signal, as the 3.7-decade exclusion (so the 2 %
## highlight cut keeps its in-frame share of a lit field).

## [E] Half angle of the adaptation field about the view axis: the extent of the monocular visual field from the fixation
## point, ~60 deg superior, ~75 inferior, ~60 nasal, ~100 temporal (standard perimetry, e.g. Traquair's limits; mean 74),
## rounded down to 70. The brief's ~60 deg left the ground out of every view 60 deg up or more (the inferior field reaches
## it): the lunar day seen 60 deg up stayed a black field (gain 12 650, dark adapted) although the sunlit regolith fills
## the lower periphery.
const CAP_HALF_ANGLE_DEG := 70.0
## [E] The weight per solid angle, exp(-theta^2 / (2 sigma^2)) of the angle from the view axis: the continuation of the
## frame's own centre weighting (AdaptationMeter.CENTRE_SIGMA 0.5 half diagonals) beyond its edge - for the desktop frame
## (50 deg vertical, 16:9, half diagonal 43.6 deg; pixel density ~1 / cos^3) that weighting per steradian falls to 0.36 at
## the corner and 0.83 at the top edge, which a Gaussian of 30 deg in angle reproduces (0.36, 0.71); Moon & Spencer 1945's
## surround weight falls with eccentricity as well [S: JOSA 35, 233]. At the cap's edge 0.066.
const SIGMA_DEG := 30.0
## [call] The query directions: rings of QUERY_STEP_DEG about the view axis, azimuths at about the same spacing (~150
## directions; a statistic of a smooth surround - 7 deg cost 0.70 ms per measurement in GDScript, 10 deg about half).
const QUERY_STEP_DEG := 10.0
## [call] The store: elevation bands (7.5 deg; a band edge on the horizon, so sky and ground never share a cell) x
## azimuths (15 deg) about the eye's local vertical; nearest-cell lookup.
const STORE_BANDS := 24
const STORE_AZIMUTHS := 24
## [call] Cells evaluated per frame (a full store in 72 frames, 0.8 s at 90 Hz, inside the view jump's 1 s snap; the light
## changes far slower). 24 per frame cost 0.24 ms of the main thread. After a view jump (reset) one complete store at once.
const CELLS_PER_FRAME := 8
## [call] A head turn below this (Basis entries, ~0.3 deg) reuses the last lookups (combine).
const LOOKUP_REUSE := 0.005
## [call] Visibility: the model's in-frame field within 0.3 decades (x2) of the frame's: open (V = 1); beyond 0.6
## decades (x4): enclosed (V = 0).
const VISIBILITY_FULL_DECADES := 0.3
const VISIBILITY_NONE_DECADES := 0.6
## [call] Time constant (s) of the low-pass on the disagreement in decades: slower than a head turn, as fast as the eye's
## adaptation steps (a cabin entered by walking fades in about this time; a view jump restarts it).
const VISIBILITY_TIME_S := 1.0
## The model's in-frame histogram as integer counts for AdaptationMeter.field_log_average (resolution 1e-6 of the weight).
const MODEL_COUNTS := 1.0e6
## [call] Main-thread time (us) a view jump's complete store may take per frame (a penumbra costs ~56 us per cell, 32 ms in
## all; the frame alone meanwhile). The budget is checked after every CELLS_PER_FRAME cells.
const REBUILD_BUDGET_USEC := 2000
## [call] While the surround is not used (V = 0 in the cabin) the store advances only every this many frames.
const IDLE_REFRESH_EVERY := 8
## The fidelity flag (on in the game; the user argument --frame_only_adaptation meters the frame alone, as before T4).
static var enabled := not OS.get_cmdline_user_args().has("--frame_only_adaptation")

## Diagnostics of the last combine: the visibility, the out-of-frame share of the field's weight, the model's in-frame
## field (cd/m^2; 0 black, -1 none), the frame's own field (cd/m^2).
var last_visibility := 0.0
var last_out_share := 0.0
var last_model_in_cd_m2 := -1.0
var last_frame_field_cd_m2 := -1.0
## Diagnostics: main-thread time spent (us) and calls, refresh and combine (render_check prints the means).
var refresh_usec := 0
var refresh_calls := 0
var combine_usec := 0
var combine_calls := 0

var _query_dirs := PackedVector3Array()  ## head space (-Z forward)
var _query_weights := PackedFloat64Array()  ## w(theta) x solid angle
var _in_frame := PackedByteArray()
var _in_frame_key := []
var _basis := Basis.IDENTITY  ## the complete store's axes (render world): y the local vertical
var _rgb := PackedVector3Array()
var _valid := false
var _build_basis := Basis.IDENTITY
var _build_rgb := PackedVector3Array()
var _build_next := 0
var _build_state: Variant = null
var _store_serial := 0
var _rebuild_now := true
var _lookup_key := []
var _lookup := []  ## [weight in frame, the model's in-frame field (cd/m^2), out-of-frame samples]
var _decades := -1.0  ## the low-passed disagreement of the frame and the model (decades); < 0: none yet
var _last_combine_usec := 0
var _idle_frames := 0


func _init() -> void:
	var step := deg_to_rad(QUERY_STEP_DEG)
	var rings := int(round(CAP_HALF_ANGLE_DEG / QUERY_STEP_DEG))
	var sigma := deg_to_rad(SIGMA_DEG)
	for k in range(rings):
		var lo := k * step
		var hi := (k + 1) * step
		var theta := 0.5 * (lo + hi)
		var count := maxi(4, int(round(TAU * sin(theta) / step)))
		var solid := TAU * (cos(lo) - cos(hi)) / count
		var w := exp(-theta * theta / (2.0 * sigma * sigma)) * solid
		for a in range(count):
			var phi := TAU * (a + 0.5 * (k % 2)) / count
			_query_dirs.append(Vector3(sin(theta) * cos(phi), sin(theta) * sin(phi), -cos(theta)))
			_query_weights.append(w)
	_build_rgb.resize(STORE_BANDS * STORE_AZIMUTHS)


## The weighted integral of the query set (sr): the cap's integral of the weight (tests).
func query_weight_total() -> float:
	var total := 0.0
	for w in _query_weights:
		total += w
	return total


## A view jump: the store belongs to another place; the next refresh builds a complete one at once (576 cells, ~6 ms of
## the main thread, spread over frames by REBUILD_BUDGET_USEC; the frame alone meanwhile).
func reset() -> void:
	_decades = -1.0
	_valid = false
	_build_next = 0
	_build_state = null
	_rebuild_now = true


func is_ready() -> bool:
	return _valid


## Advances the store by `cells` cells (all: a complete store at once). up_render: the eye's local vertical (render world);
## begin: Callable() -> state, called at the start of a store; luminance: Callable(direction_render, state) -> RGB cd/m^2.
func refresh(up_render: Vector3, begin: Callable, luminance: Callable, cells := CELLS_PER_FRAME) -> void:
	var total := STORE_BANDS * STORE_AZIMUTHS
	if _valid and last_visibility <= 0.0 and cells < total:  # the surround is not used: tick slowly
		_idle_frames += 1
		if _idle_frames % IDLE_REFRESH_EVERY != 0:
			return
	var start := Time.get_ticks_usec()
	if _build_next == 0:
		_build_basis = _local_basis(up_render)
		_build_state = begin.call()
	var chunk := mini(cells, total)
	while true:
		var end := mini(total, _build_next + chunk)
		for index in range(_build_next, end):
			_build_rgb[index] = luminance.call(_build_basis * _cell_direction(index), _build_state)
		_build_next = end
		# A view jump's rebuild: more chunks while the frame's budget lasts.
		if not (_rebuild_now and _build_next < total and Time.get_ticks_usec() - start < REBUILD_BUDGET_USEC):
			break
	if _build_next >= total:
		_rebuild_now = false
		_basis = _build_basis
		_rgb = _build_rgb.duplicate()
		_valid = true
		_build_next = 0
		_store_serial += 1
	refresh_usec += Time.get_ticks_usec() - start
	refresh_calls += 1


## The stored luminance (RGB cd/m^2) along a render-world direction (nearest cell); ZERO without a store.
func lookup(direction_render: Vector3) -> Vector3:
	if not _valid:
		return Vector3.ZERO
	return _rgb[_cell_of(_basis.transposed() * direction_render)]


## The frame's histograms (AdaptationMeter layout: photopic then scotopic) with the out-of-frame part of the field added.
## view: {"head": Basis (render world, -Z the view axis), "eyes": [[Basis of the eye in the head's axes, Projection], ...]}
## (both VR eyes: in the frame is in either). Without a store, a view or any lit out-of-frame direction: counts unchanged.
func combine(counts: PackedInt32Array, view: Dictionary, dt_s := -1.0) -> PackedInt32Array:
	var start := Time.get_ticks_usec()
	if dt_s < 0.0:  # the time since the last combine (tests pass it)
		dt_s = minf(float(start - _last_combine_usec) * 1.0e-6, 1.0) if _last_combine_usec > 0 else 1.0
	_last_combine_usec = start
	var out := _combine(counts, view, dt_s)
	combine_usec += Time.get_ticks_usec() - start
	combine_calls += 1
	return out


func _combine(counts: PackedInt32Array, view: Dictionary, dt_s: float) -> PackedInt32Array:
	var bins := AdaptationMeter.HISTOGRAM_BINS
	last_visibility = 0.0
	last_out_share = 0.0
	last_model_in_cd_m2 = -1.0
	last_frame_field_cd_m2 = AdaptationMeter.field_log_average(counts, 0, true)
	if not enabled or not _valid or view.is_empty() or counts.size() != 2 * bins:
		return counts
	var frame_total := 0.0
	for k in range(bins):
		frame_total += float(counts[k])
	if frame_total <= 0.0:
		return counts
	var eyes: Array = view["eyes"]
	_update_in_frame(eyes)
	var to_store: Basis = _basis.transposed() * (view["head"] as Basis)
	var key := [_store_serial, _in_frame_key, to_store.x.snapped(Vector3.ONE * LOOKUP_REUSE),
			to_store.y.snapped(Vector3.ONE * LOOKUP_REUSE), to_store.z.snapped(Vector3.ONE * LOOKUP_REUSE)]
	if key != _lookup_key:
		_lookup_key = key
		_lookup = _look_up(to_store)
	var weight_in: float = _lookup[0]
	last_model_in_cd_m2 = _lookup[1]
	var outside: Dictionary = _lookup[2]
	if weight_in <= 0.0:
		return counts
	var target := decades_between(last_frame_field_cd_m2, last_model_in_cd_m2)
	if target < 0.0:
		return counts
	_decades = target if _decades < 0.0 else lerpf(_decades, target, 1.0 - exp(-dt_s / VISIBILITY_TIME_S))
	last_visibility = visibility_of_decades(_decades)
	if last_visibility <= 0.0 or outside.is_empty():
		return counts
	var scale := frame_total / weight_in * last_visibility
	var out := counts.duplicate()
	var added := 0.0
	for k: int in outside:
		var add := int(round(float(outside[k]) * scale))
		if add <= 0:
			continue
		out[k] += add
		if k < bins:
			added += add
	last_out_share = added / (frame_total + added)
	return out


## The query directions through the store for a head orientation in the store's axes: [weight in frame, lit weight in
## frame, the model's field over the in-frame directions (cd/m^2; the frame's statistic), the lit out-of-frame weight by
## histogram index {bin: weight} (photopic bins, then the scotopic ones at + HISTOGRAM_BINS)].
func _look_up(to_store: Basis) -> Array:
	var black := pow(10.0, AdaptationMeter.LOG_MIN)
	var weight_in := 0.0
	var outside := {}
	var bins := AdaptationMeter.HISTOGRAM_BINS
	var in_bins := PackedFloat64Array()
	in_bins.resize(bins)
	for j in range(_query_dirs.size()):
		var w := _query_weights[j]
		var rgb := _rgb[_cell_of(to_store * _query_dirs[j])]
		var y := rgb.dot(Atmosphere.LUMINANCE_WEIGHTS)
		if _in_frame[j] != 0:
			weight_in += w
			in_bins[AdaptationMeter.bin_of(y)] += w  # black in bin 0, as the frame's
		elif y > black:
			var photopic := AdaptationMeter.bin_of(y)
			var scotopic := bins + AdaptationMeter.bin_of(rgb.dot(EyeAdaptation.SCOTOPIC_WEIGHTS))
			outside[photopic] = float(outside.get(photopic, 0.0)) + w
			outside[scotopic] = float(outside.get(scotopic, 0.0)) + w
	var model_in := -1.0
	if weight_in > 0.0:
		var counts := PackedInt32Array()
		counts.resize(bins)
		for k in range(bins):
			counts[k] = int(round(in_bins[k] / weight_in * MODEL_COUNTS))
		model_in = AdaptationMeter.field_log_average(counts, 0, true)  # as the frame's, for the visibility
	return [weight_in, model_in, outside]


## The disagreement of the frame's field and the model's over the same directions (cd/m^2; 0: black, < 0: none) in
## decades; black counts as the histogram's floor, so a field fading to black grows the measure continuously (both black:
## 0). < 0 without a field.
static func decades_between(frame_cd_m2: float, model_cd_m2: float) -> float:
	if frame_cd_m2 < 0.0 or model_cd_m2 < 0.0:
		return -1.0
	var floor_cd_m2 := pow(10.0, AdaptationMeter.LOG_MIN)
	return absf(log(maxf(frame_cd_m2, floor_cd_m2) / maxf(model_cd_m2, floor_cd_m2))) / log(10.0)


## The visibility (0 .. 1) of a disagreement in decades: open within VISIBILITY_FULL_DECADES, enclosed beyond
## VISIBILITY_NONE_DECADES.
static func visibility_of_decades(decades: float) -> float:
	return 1.0 - smoothstep(VISIBILITY_FULL_DECADES, VISIBILITY_NONE_DECADES, decades)


## The instantaneous visibility from the two fields (no time filter).
static func visibility(frame_cd_m2: float, model_cd_m2: float) -> float:
	var decades := decades_between(frame_cd_m2, model_cd_m2)
	return 0.0 if decades < 0.0 else visibility_of_decades(decades)


## Which query directions a view shows (cached per set of eye orientations and projections).
func _update_in_frame(eyes: Array) -> void:
	var key := _view_key(eyes)
	if key == _in_frame_key and _in_frame.size() == _query_dirs.size():
		return
	_in_frame_key = key
	_in_frame.resize(_query_dirs.size())
	for j in range(_query_dirs.size()):
		var inside := 0
		for eye: Array in eyes:
			if in_view(_query_dirs[j], eye[0], eye[1]):
				inside = 1
				break
		_in_frame[j] = inside


## The eyes' orientations and projections rounded (1e-4): tracking noise does not redo the in-frame flags every time.
static func _view_key(eyes: Array) -> Array:
	var key := []
	for eye: Array in eyes:
		var b: Basis = eye[0]
		var p: Projection = eye[1]
		for v: Vector3 in [b.x, b.y, b.z]:
			key.append(v.snapped(Vector3.ONE * 1.0e-4))
		for c: Vector4 in [p.x, p.y, p.z, p.w]:
			key.append(c.snapped(Vector4.ONE * 1.0e-4))
	return key


## Whether a direction (head axes) falls inside a view of that orientation (head axes) and projection.
static func in_view(direction: Vector3, eye_basis: Basis, projection: Projection) -> bool:
	var local := eye_basis.transposed() * direction
	var clip := projection * Vector4(local.x, local.y, local.z, 0.0)
	return clip.w > 0.0 and absf(clip.x) <= clip.w and absf(clip.y) <= clip.w


## The store's axes for a local vertical: y up, x and z horizontal.
static func _local_basis(up: Vector3) -> Basis:
	var y := up.normalized()
	var helper := Vector3(1, 0, 0) if absf(y.x) < 0.9 else Vector3(0, 0, 1)
	var x := helper.cross(y).normalized()
	return Basis(x, y, x.cross(y))


## The centre direction of a store cell in its axes.
static func _cell_direction(index: int) -> Vector3:
	var band := index / STORE_AZIMUTHS
	var azimuth := TAU * (float(index % STORE_AZIMUTHS) + 0.5) / STORE_AZIMUTHS
	var elevation := PI * ((float(band) + 0.5) / STORE_BANDS - 0.5)
	return Vector3(cos(elevation) * cos(azimuth), sin(elevation), cos(elevation) * sin(azimuth))


## The store cell of a direction in its axes.
static func _cell_of(local: Vector3) -> int:
	var elevation := asin(clampf(local.y, -1.0, 1.0))
	var band := clampi(int(floor((elevation / PI + 0.5) * STORE_BANDS)), 0, STORE_BANDS - 1)
	var azimuth := fposmod(atan2(local.z, local.x), TAU)
	var a := int(floor(azimuth / TAU * STORE_AZIMUTHS)) % STORE_AZIMUTHS
	return band * STORE_AZIMUTHS + a
