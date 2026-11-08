# The scene-referred eye adaptation's field statistics (AdaptationMeter.field_log_average, field_anchor) and the field rule's gain
# (EyeAdaptation.FIELD_MID_GREY): headless, exit code 1 on failure.
# godot --headless --path game --script res://tests/adaptation_meter_test.gd
extends SceneTree

const BINS := AdaptationMeter.HISTOGRAM_BINS

var _failures := 0


func _near(name_: String, actual: float, expected: float, relative: float) -> void:
	if not (absf(actual - expected) <= relative * absf(expected)):
		print("adaptation_meter_test: FAIL %s: %s, expected %s" % [name_, actual, expected])
		_failures += 1


func _check_rule(name_: String, ok: bool) -> void:
	if not ok:
		print("adaptation_meter_test: FAIL %s" % name_)
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
	# The cut is 2 % of the lit weight (user 2026-11-07): a lit area of 2.4 % of a mostly black field stays the field, the
	# Sun disc (1 % of the lit weight) in it is still cut.
	_near("1.5 % lit area stays the field (lit-weight cut)", AdaptationMeter.field_log_average(_histogram({0: 9850, k500: 150})), _centre(k500), 1e-9)
	_near("lunar-like small lit share", AdaptationMeter.field_log_average(_histogram({0: 9760, k500: 240})), _centre(k500), 1e-9)
	_near("lunar-like anchor", float(AdaptationMeter.bin_of(AdaptationMeter.field_anchor(_histogram({0: 9760, k500: 240})))), float(k500), 0.0)
	_near("sun disc cut in a black field", AdaptationMeter.field_log_average(_histogram({0: 9000, BINS - 1: 10, k500: 990})), _centre(k500), 1e-9)
	_near("cut of the whole weight: a small lit area is cut", AdaptationMeter.field_log_average(_histogram({0: 9850, k500: 150}), 0, true), 0.0, 0.0)
	# No samples: -1; nothing above black: 0 (a dark field).
	_near("no samples", AdaptationMeter.field_log_average(_histogram({})), -1.0, 0.0)
	_near("dark field", AdaptationMeter.field_log_average(_histogram({0: 1000})), 0.0, 0.0)
	# The Sun alone in a black field (review T5b M3 / I3): the meter masks the disc out (its samples never reach the
	# histograms: the shader skips them), so the frame is black and its field 0 whatever the disc's pixel share; the
	# glare veil (Atmosphere.glare_veil, glare_veil_test) then is the field. The mask's parameters: the Sun at the view
	# axis of a desktop view is the image centre's ray, a missing Sun masks nothing (cosine 2).
	var view := {"head": Basis.IDENTITY, "eyes": [[Basis.IDENTITY, Projection.create_perspective(50.0, 16.0 / 9.0, 0.05, 1000.0)]],
			"sun": Vector3(0, 0, -1), "sun_mask_cos": 0.99999}
	var mask := AdaptationMeter.sun_mask_parameters(view, 0)
	_near("Sun mask: ray at the image centre", float(mask[1]) * float(mask[1]) + float(mask[3]) * float(mask[3]), 0.0, 0.0)
	_near("Sun mask: Sun along -Z", float(mask[6]), -1.0, 1e-6)
	_near("Sun mask: no Sun, no mask", float(AdaptationMeter.sun_mask_parameters({"eyes": view["eyes"]}, 0)[7]), 2.0, 0.0)
	_near("Sun masked: the black frame's field", AdaptationMeter.field_log_average(_histogram({0: 1000})), 0.0, 0.0)
	# The scotopic histogram at its offset.
	var both := _histogram({k500: 10, BINS + k50: 10})
	_near("scotopic offset", AdaptationMeter.field_log_average(both, BINS), _centre(k50), 1e-9)

	# The anchored rule (AdaptationMeter.field_anchor): the ANCHOR_PERCENTILE-th percentile, each bin's
	# weight spread evenly in log across the bin. A uniform field: the anchor inside its bin (at the percentile's place
	# in it), so L_a = L / ANCHOR_WHITE to within the bin's resolution.
	var width := (AdaptationMeter.LOG_MAX - AdaptationMeter.LOG_MIN) / float(BINS - 1)
	var uniform_anchor := AdaptationMeter.field_anchor(_histogram({k500: 1000}))
	_near("anchor uniform field", uniform_anchor,
			pow(10.0, AdaptationMeter.bin_log_centre(k500) + (AdaptationMeter.ANCHOR_PERCENTILE - 0.5) * width), 1e-9)
	_near("anchor uniform field within the bin", float(AdaptationMeter.bin_of(uniform_anchor)), float(k500), 0.0)
	_near("anchor L_a", uniform_anchor / EyeAdaptation.ANCHOR_WHITE, 500.0 / EyeAdaptation.ANCHOR_WHITE,
			pow(10.0, width) - 1.0)
	# A 1 % bright spot (inside the 2 % highlight cut) does not move the anchor.
	var k_bright := AdaptationMeter.bin_of(500.0 * 20.0)
	_near("anchor 1 % spot", AdaptationMeter.field_anchor(_histogram({k500: 990, k_bright: 10})), uniform_anchor, 1e-9)
	# A 5 % spot: 3 % left after the cut, below the 5 % above the percentile - still on the field.
	_near("anchor 5 % spot", float(AdaptationMeter.bin_of(AdaptationMeter.field_anchor(_histogram({k500: 950, k_bright: 50})))),
			float(k500), 0.0)
	# A 10 % bright area (8 % of the 98 % left, above the 5 % share) sets the anchor: in the bright area's bin.
	var area := AdaptationMeter.field_anchor(_histogram({k500: 900, k_bright: 100}))
	_near("anchor 10 % area", float(AdaptationMeter.bin_of(area)), float(k_bright), 0.0)
	# Same exclusions as the field average: black sky ignored, nothing above black -> 0, no samples -> -1, scotopic offset.
	_near("anchor black sky", AdaptationMeter.field_anchor(_histogram({0: 1000, k500: 1000})), uniform_anchor, 1e-9)
	_near("anchor below the simultaneous range", AdaptationMeter.field_anchor(_histogram({k_dim: 50, k500: 1000})),
			uniform_anchor, 1e-9)
	_near("anchor dark field", AdaptationMeter.field_anchor(_histogram({0: 1000})), 0.0, 0.0)
	_near("anchor no samples", AdaptationMeter.field_anchor(_histogram({})), -1.0, 0.0)
	_near("anchor scotopic offset", float(AdaptationMeter.bin_of(AdaptationMeter.field_anchor(both, BINS))), float(k50), 0.0)
	# The anchor is at or above the field's log average (a percentile of the same field above its middle).
	var spread := _histogram({k_shade: 500, k50: 500, k500: 500})
	_near("anchor above the average", float(AdaptationMeter.field_anchor(spread) >= AdaptationMeter.field_log_average(spread)),
			1.0, 0.0)

	# The field rule's adaptation luminance L_f / FIELD_MID_GREY (Atmosphere.update): a grey-world field (Y_b 20 % of the
	# white plate) gives the plate's gain; the Apollo 11 field measured in Task 14 (660 cd/m^2 under a 41 064 cd/m^2
	# plate) 12.4 in place of the plate rule's 1 (Weber range: L_ref x 0.2 / L_f).
	var reference := 41064.0
	var grey := EyeAdaptation.FIELD_MID_GREY
	_near("grey world", EyeAdaptation.cone_gain(grey * 20000.0 / grey, reference), EyeAdaptation.cone_gain(20000.0, reference),
			1e-12)
	_near("Apollo 11 field", EyeAdaptation.cone_gain(660.0 / grey, reference), reference * grey / 660.0, 1e-9)

	# Highlight protection (Atmosphere.FieldRule.PROTECT, rule D before T13, --protect_adaptation): the larger (darker exposure) of the mean-grey
	# and the anchor adaptation luminances, photopic or scotopic alike.
	var key := EyeAdaptation.FIELD_MID_GREY
	var white := EyeAdaptation.ANCHOR_WHITE
	_near("rule field", Atmosphere.field_adaptation_luminance(Atmosphere.FieldRule.MEAN_GREY, 100.0, 1000.0, key), 100.0 / key, 1e-9)
	_near("rule anchor", Atmosphere.field_adaptation_luminance(Atmosphere.FieldRule.ANCHOR, 100.0, 1000.0, key), 1000.0 / white, 1e-9)
	# A bright anchor (sky beside ground): the anchor exposure is the stricter one.
	_near("rule protect, anchor stricter", Atmosphere.field_adaptation_luminance(Atmosphere.FieldRule.PROTECT, 100.0, 1000.0, key),
			1000.0 / white, 1e-9)
	# An anchor close to the field (a uniform grey scene): the mean-grey exposure is the stricter one.
	_near("rule protect, field stricter", Atmosphere.field_adaptation_luminance(Atmosphere.FieldRule.PROTECT, 100.0, 120.0, key),
			100.0 / key, 1e-9)
	_near("rule protect, scotopic pair", Atmosphere.field_adaptation_luminance(Atmosphere.FieldRule.PROTECT, 0.01, 0.2, key),
			0.2 / white, 1e-9)
	# Unified appearance model (look fixes T13): the game's rule is the mean grey with Y_b = 0.2 (CIE 159), no anchor term;
	# the field then renders at Y_b x DAY_EXPOSURE (gain L_ref / L_a, scene value L x DAY_EXPOSURE x gain / L_ref), which the
	# fixed display curve shows at Y_b of display white. The old rules stay selectable.
	_near("T13 Y_b", key, 0.2, 1e-12)
	_check_rule("T13 default rule", Atmosphere._field_rule_from_arguments(PackedStringArray()) == Atmosphere.FieldRule.MEAN_GREY)
	_check_rule("T13 --protect_adaptation", Atmosphere._field_rule_from_arguments(
			PackedStringArray(["--protect_adaptation"])) == Atmosphere.FieldRule.PROTECT)
	_check_rule("T13 --anchored_adaptation", Atmosphere._field_rule_from_arguments(
			PackedStringArray(["--anchored_adaptation"])) == Atmosphere.FieldRule.ANCHOR)
	var field_scene := 660.0 * Atmosphere.DAY_EXPOSURE * EyeAdaptation.cone_gain(660.0 / key, reference) / reference
	_near("T13 field renders at Y_b x DAY_EXPOSURE", field_scene, key * Atmosphere.DAY_EXPOSURE, 1e-9)
	_near("T13 field at Y_b of display white", DisplayTone.display_curve(field_scene, PackedFloat64Array()), key, 1e-9)
	# The white cap (the former gain floor 1): a field brighter than Y_b of the TOA white plate is a white world.
	var capped := EyeAdaptation.new()
	capped.advance(0.5 * reference / key, 0.0, reference, 0.0)
	_near("T13 white cap", capped.gain, 1.0, 1e-12)
	_near("rule protect, dark field", Atmosphere.field_adaptation_luminance(Atmosphere.FieldRule.PROTECT, 0.0, 0.0, key), 0.0, 0.0)

	# The start view (World.start_view_angles): the camera sits 60 deg from the Sun as seen from the Earth's centre
	# (75 % lit), north stays up, for Suns of any declination and right ascension.
	for sun in [Vector3(1.4e11, 0, 0), Vector3(-1.0e11, 9.0e10, 5.0e10), Vector3(0, -1.5e11, -6.0e10), Vector3(2.0e10, 1.4e11, 5.8e10)]:
		var angles := World.start_view_angles(sun, 0.6)
		var c := Frames.from_render(CameraMath.orbit_offset(Basis.IDENTITY, angles.x, angles.y, 1.0))
		_near("start view phase angle", rad_to_deg(c.angle_to(sun)), World.START_VIEW_PHASE_DEG, 1e-9)
		_near("start view lit fraction", (1.0 + cos(c.angle_to(sun))) / 2.0, 0.75, 1e-12)

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
