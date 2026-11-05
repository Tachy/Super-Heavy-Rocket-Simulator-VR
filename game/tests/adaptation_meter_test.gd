# The scene-referred eye adaptation's field statistic (AdaptationMeter.field_log_average) and the field rule's gain
# (EyeAdaptation.FIELD_MID_GREY): headless, exit code 1 on failure.
# godot --headless --path game --script res://tests/adaptation_meter_test.gd
extends SceneTree

const BINS := AdaptationMeter.HISTOGRAM_BINS

var _failures := 0


func _near(name_: String, actual: float, expected: float, relative: float) -> void:
	if not (absf(actual - expected) <= relative * absf(expected)):
		print("adaptation_meter_test: FAIL %s: %s, expected %s" % [name_, actual, expected])
		_failures += 1


func _histogram(entries: Dictionary) -> PackedInt32Array:  # bin -> weight
	var counts := PackedInt32Array()
	counts.resize(BINS * 2)
	for k in entries:
		counts[k] = entries[k]
	return counts


func _centre(k: int) -> float:
	return pow(10.0, AdaptationMeter.bin_log_centre(k))


func _initialize() -> void:
	var k500 := AdaptationMeter.bin_of(500.0)
	# Bins: a bin's centre falls in that bin; black and below the range in bin 0, above it in the top bin.
	for k in range(1, BINS):
		if AdaptationMeter.bin_of(_centre(k)) != k:
			print("adaptation_meter_test: FAIL bin %d round trip" % k)
			_failures += 1
	_near("black in bin 0", AdaptationMeter.bin_of(0.0), 0.0, 0.0)
	_near("below range in bin 0", AdaptationMeter.bin_of(1.0e-7), 0.0, 0.0)
	_near("above range in the top bin", AdaptationMeter.bin_of(1.0e9), float(BINS - 1), 0.0)
	# One luminance: its bin's centre.
	_near("uniform field", AdaptationMeter.field_log_average(_histogram({k500: 1000})), _centre(k500), 1e-9)
	# The geometric mean of two equal populations.
	var k50 := AdaptationMeter.bin_of(50.0)
	_near("geometric mean", AdaptationMeter.field_log_average(_histogram({k500: 980, k50: 980, BINS - 1: 40})),
			sqrt(_centre(k500) * _centre(k50)), 1e-9)
	# A black sky over half the field does not pull the adaptation down.
	_near("black sky", AdaptationMeter.field_log_average(_histogram({0: 1000, k500: 1000})), _centre(k500), 1e-9)
	# The Sun's disc (1 % of the weight, top bin) is cut.
	_near("sun disc cut", AdaptationMeter.field_log_average(_histogram({BINS - 1: 10, k500: 990})), _centre(k500), 1e-9)
	# More than the cut: the highlight's rest stays in the mean (2 % cut of 5 %: 3 % at the top).
	var mixed := AdaptationMeter.field_log_average(_histogram({BINS - 1: 50, k500: 950}))
	_near("highlight rest", mixed, pow(10.0, (30.0 * AdaptationMeter.bin_log_centre(BINS - 1)
			+ 950.0 * AdaptationMeter.bin_log_centre(k500)) / 980.0), 1e-9)
	# Below the simultaneous range (more than 3.7 decades under the highlights): no signal, not in the mean.
	var k_dim := AdaptationMeter.bin_of(500.0 * pow(10.0, -4.5))
	_near("below the simultaneous range", AdaptationMeter.field_log_average(_histogram({k_dim: 5000, k500: 1000})),
			_centre(k500), 1e-9)
	var k_shade := AdaptationMeter.bin_of(500.0 * pow(10.0, -2.0))
	_near("inside the simultaneous range", AdaptationMeter.field_log_average(_histogram({k_shade: 980, k500: 980, BINS - 1: 40})),
			sqrt(_centre(k500) * _centre(k_shade)), 1e-9)
	# No samples: -1; nothing above black: 0 (a dark field).
	_near("no samples", AdaptationMeter.field_log_average(_histogram({})), -1.0, 0.0)
	_near("dark field", AdaptationMeter.field_log_average(_histogram({0: 1000})), 0.0, 0.0)
	# The scotopic histogram at its offset.
	var both := _histogram({k500: 10, BINS + k50: 10})
	_near("scotopic offset", AdaptationMeter.field_log_average(both, BINS), _centre(k50), 1e-9)

	# The field rule's adaptation luminance L_f / FIELD_MID_GREY (Atmosphere.update): a grey-world field (18 % of the
	# white plate) gives the plate's gain; the Apollo 11 field measured in Task 14 (660 cd/m^2 under a 41 064 cd/m^2
	# plate) 11.2 in place of the plate rule's 1 (Weber range: L_ref x 0.18 / L_f).
	var reference := 41064.0
	var grey := EyeAdaptation.FIELD_MID_GREY
	_near("grey world", EyeAdaptation.cone_gain(grey * 20000.0 / grey, reference), EyeAdaptation.cone_gain(20000.0, reference),
			1e-12)
	_near("Apollo 11 field", EyeAdaptation.cone_gain(660.0 / grey, reference), reference * grey / 660.0, 1e-9)

	# The compute's sizes follow the constants (review M-1): the shared histogram holds both histograms, no placeholder
	# is left, and the placement is before the transparent pass (review I-1: display-referred emitters not counted).
	var source := AdaptationMeter.shader_source()
	_near("shared histogram size", float(source.contains("local_bins[%d]" % (BINS * 2))), 1.0, 0.0)
	_near("no placeholder left", float(source.contains("TOTAL_BINS") or source.contains("GROUP_EDGE")), 0.0, 0.0)
	# Every display-referred emitter (vision_display_referred) draws in the transparent pass (a blend mode), so the
	# meter, before that pass, never counts it: the field does not feed back on the gain.
	var dir := DirAccess.open("res://shaders")
	for file in dir.get_files():
		if not file.ends_with(".gdshader"):
			continue
		var code := FileAccess.get_file_as_string("res://shaders/" + file)
		if code.contains("vision_display_referred("):
			var mode := code.substr(code.find("render_mode"), 200)
			if not mode.contains("blend_"):
				print("adaptation_meter_test: FAIL %s is display-referred but not in the transparent pass" % file)
				_failures += 1
	var meter := AdaptationMeter.new()
	_near("before the transparent pass", float(meter.effect_callback_type),
			float(CompositorEffect.EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT), 0.0)

	print("adaptation_meter_test: %s" % ("PASS" if _failures == 0 else "%d FAILED" % _failures))
	quit(1 if _failures > 0 else 0)
