# The eye adaptation's field beyond the frame (AdaptationSurround, look fixes T4): headless, exit code 1 on failure.
# godot --headless --path game --script res://tests/adaptation_surround_test.gd
extends SceneTree

const BINS := AdaptationMeter.HISTOGRAM_BINS

var _failures := 0


func _check(name_: String, ok: bool, detail := "") -> void:
	if not ok:
		print("adaptation_surround_test: FAIL %s %s" % [name_, detail])
		_failures += 1


func _near(name_: String, actual: float, expected: float, relative: float) -> void:
	_check(name_, absf(actual - expected) <= relative * absf(expected), "%s, expected %s" % [actual, expected])


## A frame histogram: every sample at one luminance (grey: photopic and scotopic alike), weight n.
func _frame(luminance: float, n: int) -> PackedInt32Array:
	var counts := PackedInt32Array()
	counts.resize(BINS * 2)
	counts[AdaptationMeter.bin_of(luminance)] = n
	counts[BINS + AdaptationMeter.bin_of(luminance * Vector3.ONE.dot(EyeAdaptation.SCOTOPIC_WEIGHTS))] = n
	return counts


## The desktop view: the head looking along `forward` (render world), 50 deg vertical at 16:9.
func _desktop(forward: Vector3) -> Dictionary:
	var projection := Projection.create_perspective(50.0, 16.0 / 9.0, 0.05, 1000.0)
	return {"head": Basis.looking_at(forward, Vector3.UP if absf(forward.y) < 0.99 else Vector3.FORWARD),
			"eyes": [[Basis.IDENTITY, projection]]}


## A noon-like world about the vertical +Y: the sky by elevation (zenith 2000, horizon 5000 cd/m^2), the ground 1500.
func _sky_world(direction: Vector3, _state: Variant) -> Vector3:
	var elevation := asin(clampf(direction.y, -1.0, 1.0))
	if elevation < 0.0:
		return Vector3.ONE * 1500.0
	return Vector3.ONE * lerpf(5000.0, 2000.0, sqrt(elevation / (PI / 2.0)))


func _uniform_world(_direction: Vector3, _state: Variant) -> Vector3:
	return Vector3.ONE * 3000.0


func _black_world(_direction: Vector3, _state: Variant) -> Vector3:
	return Vector3.ZERO


## Lunar day: bright ground, black sky.
func _lunar_world(direction: Vector3, _state: Variant) -> Vector3:
	return Vector3.ONE * 2600.0 if direction.y < 0.0 else Vector3.ZERO


func _filled(world: Callable) -> AdaptationSurround:
	var s := AdaptationSurround.new()
	s.refresh(Vector3.UP, func() -> Variant: return null, world, AdaptationSurround.STORE_BANDS * AdaptationSurround.STORE_AZIMUTHS)
	return s


func _initialize() -> void:
	AdaptationSurround.enabled = true
	# The query set integrates the weight over the cap: 2 pi int_0^cap exp(-t^2 / 2 s^2) sin t dt (Simpson).
	var s := AdaptationSurround.new()
	var sigma := deg_to_rad(AdaptationSurround.SIGMA_DEG)
	var cap := deg_to_rad(AdaptationSurround.CAP_HALF_ANGLE_DEG)
	var n := 600
	var integral := 0.0
	for k in range(n + 1):
		var t := cap * k / n
		var f := exp(-t * t / (2.0 * sigma * sigma)) * sin(t)
		integral += f * (1.0 if k == 0 or k == n else (4.0 if k % 2 == 1 else 2.0))
	integral *= TAU * cap / n / 3.0
	_near("cap weight integral", s.query_weight_total(), integral, 0.01)

	# No store yet: the frame alone.
	var frame := _frame(2000.0, 100000)
	_check("no store: unchanged", s.combine(frame, _desktop(Vector3.UP)) == frame)

	# A uniform world seen as such in the frame: the field stays that luminance (the surround agrees with the frame).
	var uniform := _filled(_uniform_world)
	var combined := uniform.combine(_frame(3000.0, 100000), _desktop(Vector3.FORWARD))
	_near("uniform world: field kept", AdaptationMeter.field_log_average(combined, 0),
			AdaptationMeter.field_log_average(_frame(3000.0, 100000), 0), 1e-9)
	_near("uniform world: visible", uniform.last_visibility, 1.0, 0.0)
	_check("uniform world: out-of-frame weight added", uniform.last_out_share > 0.2 and uniform.last_out_share < 0.8,
			str(uniform.last_out_share))

	# Looking up into a sky darker than its horizon: the surround (the brighter horizon band) raises the field above the
	# frame's own, i.e. the gain falls; looking at the horizon the darker ground and upper sky lower it. The two fields
	# move toward each other.
	var sky := _filled(_sky_world)
	var up_frame := _frame(2100.0, 100000)
	var up_field := AdaptationMeter.field_log_average(sky.combine(up_frame, _desktop(Vector3(0.0, sin(deg_to_rad(60.0)),
			-cos(deg_to_rad(60.0))))), 0)
	var level_frame := _frame(4500.0, 100000)
	var level_field := AdaptationMeter.field_log_average(sky.combine(level_frame, _desktop(Vector3(0.0, 0.087, -0.996))), 0)
	_check("looking up: field rises", up_field > 1.1 * AdaptationMeter.field_log_average(up_frame, 0), str(up_field))
	_check("level: field falls", level_field < 0.95 * AdaptationMeter.field_log_average(level_frame, 0), str(level_field))
	_check("the fields converge", level_field / up_field < 4500.0 / 2100.0, "%s / %s" % [level_field, up_field])

	# Lunar day looking up 60 deg: a black frame, the lit ground a small share (~2 %) of the field's weight. The highlight
	# cut is taken of the lit weight only (user 2026-11-07), so that surround is not removed as a highlight.
	var lunar := _filled(_lunar_world)
	var lunar_combined := lunar.combine(_frame(0.0, 100000), _desktop(Vector3(0.0, sin(deg_to_rad(60.0)),
			-cos(deg_to_rad(60.0)))))
	_check("lunar el 60: ground share small", lunar.last_out_share > 0.0 and lunar.last_out_share < 0.1, str(lunar.last_out_share))
	_near("lunar el 60: field is the ground", AdaptationMeter.field_log_average(lunar_combined, 0), 2600.0, 0.1)
	_near("lunar el 60: anchor is the ground", AdaptationMeter.field_anchor(lunar_combined, 0), 2600.0, 0.1)

	# An enclosed view (a cabin at 16 cd/m^2 under an open-sky model): visibility 0, the frame alone.
	var cabin := _frame(16.0, 100000)
	_check("enclosed: unchanged", uniform.combine(cabin, _desktop(Vector3.FORWARD), 10.0) == cabin)
	_near("enclosed: visibility", uniform.last_visibility, 0.0, 0.0)
	# The visibility's ramp: within x2 open, beyond x4 closed.
	_near("visibility x1.9", AdaptationSurround.visibility(1900.0, 1000.0), 1.0, 0.0)
	_near("visibility x4.1", AdaptationSurround.visibility(4100.0, 1000.0), 0.0, 0.0)
	_near("visibility both black", AdaptationSurround.visibility(0.0, 0.0), 1.0, 0.0)
	_near("visibility frame black only", AdaptationSurround.visibility(0.0, 10.0), 0.0, 0.0)

	# A black surround (space) adds no weight: the frame alone, the 2 % cut keeps its share.
	var black := _filled(_black_world)
	var lit_frame := _frame(3000.0, 100000)
	_check("black surround: unchanged", black.combine(lit_frame, _desktop(Vector3.FORWARD)) == lit_frame)

	# VR: two eyes (each 100 deg, canted 10 deg outward) show more of the cap than one: less out-of-frame weight.
	var eye_projection := Projection.create_perspective(100.0, 1.0, 0.05, 1000.0)
	var left := Basis(Vector3.UP, deg_to_rad(10.0))
	var right := Basis(Vector3.UP, deg_to_rad(-10.0))
	var head := {"head": Basis.IDENTITY, "eyes": [[left, eye_projection]]}
	uniform.combine(_frame(3000.0, 100000), head, 10.0)
	var one_eye := uniform.last_out_share
	head["eyes"] = [[left, eye_projection], [right, eye_projection]]
	uniform.combine(_frame(3000.0, 100000), head, 10.0)
	_check("two eyes show more", uniform.last_out_share < one_eye, "%s vs %s" % [uniform.last_out_share, one_eye])
	_check("in view: the axis", AdaptationSurround.in_view(Vector3.FORWARD, Basis.IDENTITY, eye_projection))
	_check("in view: not behind", not AdaptationSurround.in_view(Vector3.BACK, Basis.IDENTITY, eye_projection))

	# The store's cells: a cell's centre maps back to itself; the horizon is a band edge.
	var bad := 0
	for index in range(AdaptationSurround.STORE_BANDS * AdaptationSurround.STORE_AZIMUTHS):
		if AdaptationSurround._cell_of(AdaptationSurround._cell_direction(index)) != index:
			bad += 1
	_check("cell round trip", bad == 0, str(bad))
	_check("horizon band edge", AdaptationSurround._cell_of(Vector3(1, 0.001, 0).normalized()) / AdaptationSurround.STORE_AZIMUTHS
			!= AdaptationSurround._cell_of(Vector3(1, -0.001, 0).normalized()) / AdaptationSurround.STORE_AZIMUTHS)
	# V is continuous: a frame field sweeping across the ramp (x1 .. x6 of the model's 3000) moves V by a small step per
	# measurement, and a jump of the frame (a panned structure) is low-passed in time, not applied at once.
	var sweep := _filled(_uniform_world)
	var last_v := 1.0
	var max_step := 0.0
	for i in range(0, 121):
		var field := 3000.0 * pow(10.0, 0.9 * i / 120.0)
		sweep.combine(_frame(field, 100000), _desktop(Vector3.FORWARD), 1.0 / 22.0)
		if i > 0:
			max_step = maxf(max_step, absf(sweep.last_visibility - last_v))
		last_v = sweep.last_visibility
	_check("V sweep: small steps", max_step < 0.05, str(max_step))
	_check("V sweep: ends enclosed", last_v < 0.05, str(last_v))
	var pan := _filled(_uniform_world)
	pan.combine(_frame(3000.0, 100000), _desktop(Vector3.FORWARD), 1.0 / 22.0)
	pan.combine(_frame(30000.0, 100000), _desktop(Vector3.FORWARD), 1.0 / 22.0)  # a bright structure enters the frame
	_check("V pan: not instantly closed", pan.last_visibility > 0.7, str(pan.last_visibility))
	for i in range(60):  # ~3 s of it
		pan.combine(_frame(30000.0, 100000), _desktop(Vector3.FORWARD), 1.0 / 22.0)
	_check("V pan: closed after seconds", pan.last_visibility < 0.01, str(pan.last_visibility))
	# A frame field fading to black: the disagreement grows continuously (decades against the histogram floor), no jump to 0.
	var fade_prev := AdaptationSurround.decades_between(3000.0, 3000.0)
	var fade_step := 0.0
	for e in range(0, 100):
		var level := 3000.0 * pow(10.0, -0.1 * e)
		var d := AdaptationSurround.decades_between(level, 3000.0)
		fade_step = maxf(fade_step, absf(d - fade_prev))
		fade_prev = d
	_check("fade to black: continuous", fade_step < 0.11, str(fade_step))
	_near("black frame, lit model: far", AdaptationSurround.decades_between(0.0, 3000.0), 9.48, 0.01)

	# The model's in-frame field is the frame's statistic: a ground 4.5 decades under the sky (0.1 against 3000 cd/m^2)
	# is excluded by the 3.7-decade range, as the frame would (the plain mean would give ~38 cd/m^2).
	var split := _filled(func(direction: Vector3, _state: Variant) -> Vector3:
			return Vector3.ONE * (3000.0 if direction.y > 0.0 else 0.1))
	split.combine(_frame(3000.0, 100000), _desktop(Vector3(0.0, 0.087, -0.996)), 1.0)
	_near("model in frame: range exclusion", split.last_model_in_cd_m2, 3000.0, 0.1)

	# The scotopic histogram gets the out-of-frame content as well.
	var scot := _filled(_uniform_world)
	var scot_in := _frame(3000.0, 100000)
	var scot_out := scot.combine(scot_in, _desktop(Vector3.FORWARD), 1.0)
	var photopic_added := 0
	var scotopic_added := 0
	for k in range(BINS):
		photopic_added += scot_out[k] - scot_in[k]
		scotopic_added += scot_out[BINS + k] - scot_in[BINS + k]
	_check("scotopic content added", scotopic_added > 0 and absi(scotopic_added - photopic_added) <= BINS, "%d vs %d" % [scotopic_added, photopic_added])

	# A view jump's rebuild is spread over frames when the luminance is dear (budget), and complete in the end.
	var slow := AdaptationSurround.new()
	var slow_world := func(_d: Vector3, _st: Variant) -> Vector3:
		OS.delay_usec(60)
		return Vector3.ONE * 3000.0
	var frames := 0
	while not slow.is_ready() and frames < 100:
		slow.refresh(Vector3.UP, func() -> Variant: return null, slow_world)
		frames += 1
	_check("rebuild spread over frames", slow.is_ready() and frames > 4 and frames < 100, str(frames))

	# A reset drops the store (a view jump).
	uniform.reset()
	_check("reset: frame alone", uniform.combine(lit_frame, _desktop(Vector3.FORWARD)) == lit_frame)

	print("adaptation_surround_test: %s" % ("PASS" if _failures == 0 else "%d FAILED" % _failures))
	quit(1 if _failures > 0 else 0)
