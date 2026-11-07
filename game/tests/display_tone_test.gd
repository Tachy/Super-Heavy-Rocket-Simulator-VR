# The display tone step and the daylight Hunt factor (DisplayTone, EyeAdaptation.chroma_factor; look fixes T5;
# eye-adaptation.md "Display tone step"). CPU checks of the model the GPU step mirrors (display_tone_check.gd compares the
# GPU against it):
# - grey axis neutral: a grey scene value is written unchanged (AgX then shows its own grey curve), within 1e-5
#   relative (the published matrices' rounding);
# - AgX o the step = the target colour (the pre-distortion is AgX's exact inverse), within 1e-5, below AgX's white;
#   at and above it display white;
# - monotonic: the displayed luminance and OkLab lightness of k x grow with k (grey, the noon sky, a saturated blue,
#   an orange), and the grey curve with the scene luminance;
# - hue preserved: OkLab hue of the displayed colour = the scene's within 0.5 deg over 4000 random colours (seeded,
#   scene luminance 1e-3 .. 8, Hunt factor 1 and 1.64), where the displayed chroma is > 0.01; AgX alone turns the noon
#   sky's hue by > 5 deg (the reason for the step);
# - chroma kept below the gamut knee: the noon sky's B/R through the step = the scene's within 1e-5;
# - the Hunt factor: OkLab chroma x S at the same lightness and hue (below the knee, 1e-5); S = 1 at the display's white,
#   values from CIECAM02's F_L ratio against 100 cd/m^2 (tolerance 1e-3: 3.8e4 -> 1.641, 1e4 -> 1.468, 90 -> 0.991,
#   <= 4 -> 0.766), monotonic, at most CHROMA_FACTOR_MAX; the former reference (flag off) keeps 1 by day and 0.911 at
#   3264; cone_map takes only the part <= 1;
# - the emitters' inverse (vision.gdshaderinc): an opaque display-referred colour through tone_inverse, hunt_inverse and
#   the step is shown as AgX alone showed it, within 3 % per channel at S 1 and 1.64 (not exact: the Hunt inverse keeps
#   the OkLab lightness, the step's curve reads the luminance; the path to white above scene 2); above AgX's white
#   display white.
# Headless: godot --headless --path game --script res://tests/display_tone_test.gd
extends SceneTree

## [call] The published matrices' rounding (OkLab 10 digits) leaves ~2e-6 on the grey axis and the round trip.
const EXACT := 1.0e-5
const HUE_TOLERANCE_DEG := 0.5
const EMITTER_TOLERANCE := 0.03
const SKY := Vector3(0.0869, 0.1426, 0.284)  # [D: look investigation, pad noon el 85 scene value at gain 1.67]

var _failures := 0
var _p := DisplayTone.curve_parameters(1.25, 16.29)  # Godot 4.7's AgX defaults (world.gd keeps them)


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	_check_grey()
	_check_round_trip()
	_check_monotonic()
	_check_hue()
	_check_chroma()
	_check_hunt_factor()
	_check_emitter_inverse()
	print("display_tone_test: %s (%d failures)" % ["PASS" if _failures == 0 else "FAIL", _failures])
	quit(1 if _failures > 0 else 0)


func _check(label: String, ok: bool, detail: String) -> void:
	if not ok:
		_failures += 1
	print("display_tone_test: %-46s %s %s" % [label, detail, "ok" if ok else "FAIL"])


func _check_grey() -> void:
	var worst := 0.0
	var worst_curve := 0.0
	var y := 1.0e-4
	while y < 16.0:
		var grey := Vector3.ONE * y
		worst = maxf(worst, (DisplayTone.apply(grey, 1.6, _p) - grey).length() / grey.length())
		worst_curve = maxf(worst_curve, (DisplayTone.agx(DisplayTone.apply(grey, 1.0, _p), _p)
				- Vector3.ONE * DisplayTone.curve(y, _p)).length())
		y *= 1.25
	_check("grey written unchanged", worst < EXACT, "largest relative error %s" % worst)
	_check("grey shown on AgX's grey curve", worst_curve < EXACT, "largest error %s" % worst_curve)


func _check_round_trip() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = 5
	var worst := 0.0
	var worst_white := 0.0
	for i in range(3000):
		var x := Vector3(rng.randf(), rng.randf(), rng.randf()) * pow(10.0, rng.randf_range(-3.0, 1.6))
		var hunt := 1.0 if i % 2 == 0 else 1.64
		var shown := DisplayTone.agx(DisplayTone.apply(x, hunt, _p), _p)
		if DisplayTone.LUMINANCE_WEIGHTS.dot(x) >= _p[4]:
			worst_white = maxf(worst_white, (shown - Vector3.ONE).abs().length())
		else:
			worst = maxf(worst, (shown - DisplayTone.target(x, hunt, _p)).length())
	_check("AgX o step = target colour", worst < EXACT, "largest error %s" % worst)
	_check("above AgX's white: display white", worst_white < EXACT, "largest error %s" % worst_white)


func _check_monotonic() -> void:
	var ok := true
	var detail := ""
	for c: Vector3 in [Vector3.ONE, SKY / SKY.y, Vector3(0.02, 0.06, 0.6), Vector3(1.0, 0.35, 0.02)]:
		var last_y := -1.0
		var last_l := -1.0
		var k := 1.0e-3
		while k < 40.0:
			var shown := DisplayTone.target(c * k, 1.6, _p)
			var y := DisplayTone.LUMINANCE_WEIGHTS.dot(shown)
			var l := DisplayTone.to_oklab(shown).x
			if y < last_y - 1e-9 or l < last_l - 1e-9:
				ok = false
				detail += " %s at k %.4f (Y %.6f < %.6f or L %.6f < %.6f)" % [c, k, y, last_y, l, last_l]
			last_y = y
			last_l = l
			k *= 1.1
	var last := -1.0
	var x := 1.0e-4
	while x < 20.0:
		var g := DisplayTone.curve(x, _p)
		ok = ok and g >= last
		last = g
		x *= 1.05
	_check("monotonic in the scene level", ok, detail if detail != "" else "4 colours x 1e-3 .. 40, grey curve")


func _check_hue() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = 11
	var worst := 0.0
	var counted := 0
	for i in range(4000):
		var x := Vector3(rng.randf(), rng.randf(), rng.randf()) * pow(10.0, rng.randf_range(-3.0, 0.9))
		var hunt := 1.0 if i % 2 == 0 else 1.64
		var shown := DisplayTone.target(x, hunt, _p)
		var lab := DisplayTone.to_oklab(shown)
		if Vector2(lab.y, lab.z).length() <= 0.01:
			continue
		counted += 1
		worst = maxf(worst, _angle_deg(DisplayTone.hue_angle(shown), DisplayTone.hue_angle(x)))
	_check("OkLab hue preserved", worst < HUE_TOLERANCE_DEG and counted > 1000,
			"largest %.4f deg over %d colours" % [worst, counted])
	var agx_turn := _angle_deg(DisplayTone.hue_angle(DisplayTone.agx(SKY, _p)), DisplayTone.hue_angle(SKY))
	var step_turn := _angle_deg(DisplayTone.hue_angle(DisplayTone.target(SKY, 1.0, _p)), DisplayTone.hue_angle(SKY))
	_check("noon sky: AgX turns the hue, the step not", agx_turn > 5.0 and step_turn < 0.01,
			"AgX %.3f deg, step %.5f deg" % [agx_turn, step_turn])


func _check_chroma() -> void:
	var shown := DisplayTone.target(SKY, 1.0, _p)
	var ratio_error := absf((shown.z / shown.x) / (SKY.z / SKY.x) - 1.0)
	var agx := DisplayTone.agx(SKY, _p)
	_check("noon sky B/R kept", ratio_error < EXACT, "scene %.4f, step %.4f, AgX %.4f" % [SKY.z / SKY.x,
			shown.z / shown.x, agx.z / agx.x])
	var hunted := DisplayTone.to_oklab(DisplayTone.target(SKY, 1.5, _p))
	var plain := DisplayTone.to_oklab(shown)
	var c_ratio := Vector2(hunted.y, hunted.z).length() / Vector2(plain.y, plain.z).length()
	_check("Hunt: OkLab chroma x S, same lightness", absf(c_ratio - 1.5) < EXACT and absf(hunted.x - plain.x) < EXACT,
			"chroma ratio %.7f, lightness %.6f / %.6f" % [c_ratio, hunted.x, plain.x])
	# The highlight end: the chroma falls to 0 toward AgX's white.
	var last := 1.0e9
	var fading := true
	for k: float in [2.0, 4.0, 8.0, 12.0, 16.0]:
		var lab := DisplayTone.to_oklab(DisplayTone.target(SKY / SKY.y * k, 1.0, _p))
		var c := Vector2(lab.y, lab.z).length()
		fading = fading and c <= last
		last = c
	_check("path to white", fading and last < 1e-3, "chroma at scene 16: %s" % last)


func _check_hunt_factor() -> void:
	var expected := {38000.0: 1.641, 10000.0: 1.468, 3264.0: 1.337, 181.0: 1.051, 90.0: 0.991, 4.0: 0.766, 0.1: 0.766}
	var worst := 0.0
	for level: float in expected:
		worst = maxf(worst, absf(EyeAdaptation.chroma_factor(level) - float(expected[level])))
	var unit := EyeAdaptation.chroma_factor(EyeAdaptation.DISPLAY_WHITE_CD_M2)
	_check("S against the display's white", worst < 1e-3 and absf(unit - 1.0) < 1e-9,
			"largest table error %.5f, S(display white) %.9f" % [worst, unit])
	var last := 0.0
	var monotonic := true
	var level := 1.0
	while level < 1.0e7:
		var s := EyeAdaptation.chroma_factor(level)
		monotonic = monotonic and s >= last and s <= EyeAdaptation.CHROMA_FACTOR_MAX
		last = s
		level *= 1.2
	_check("S monotonic, capped", monotonic and absf(last - EyeAdaptation.CHROMA_FACTOR_MAX) < 1e-9,
			"S(1e7) %.4f" % last)
	EyeAdaptation.hunt_display_reference = false
	var old_noon := EyeAdaptation.chroma_factor(38000.0)
	var old_3264 := EyeAdaptation.chroma_factor(3264.0)
	EyeAdaptation.hunt_display_reference = true
	_check("flag off: the former reference", absf(old_noon - 1.0) < 1e-9 and absf(old_3264 - 0.911) < 1e-3,
			"noon %.4f, 3264 %.4f" % [old_noon, old_3264])
	var linear := EyeAdaptation.cone_map(1.6, Basis.IDENTITY)
	var worst_linear := 0.0
	for k in range(3):
		worst_linear = maxf(worst_linear, (linear[k] - Basis.IDENTITY[k]).length())
	_check("cone_map: only the part <= 1", worst_linear < 1e-12 and EyeAdaptation.display_hunt(0.7) == 1.0
			and EyeAdaptation.display_hunt(1.6) == 1.6, "linear map error %s" % worst_linear)


func _check_emitter_inverse() -> void:
	var worst := 0.0
	var worst_dark := 0.0
	for c: Vector3 in [Vector3(1.0, 0.45, 0.1), Vector3(0.9, 0.7, 0.4), Vector3(0.2, 0.4, 1.0), Vector3(0.6, 0.6, 0.6),
			Vector3(0.05, 0.3, 0.05) * 3.0, Vector3(2.0, 1.2, 0.5), Vector3(0.02, 0.01, 0.005)]:
		var before := DisplayTone.agx(c, _p)  # what AgX alone showed
		for hunt: float in [1.0, 1.64]:
			var written := DisplayTone.hunt_inverse(DisplayTone.tone_inverse(c, _p), hunt)
			var shown := DisplayTone.target(written, hunt, _p)
			for k in range(3):
				worst = maxf(worst, absf(shown[k] - before[k]) / maxf(before[k], 0.05))
	var white := DisplayTone.agx(DisplayTone.apply(DisplayTone.tone_inverse(Vector3(40, 30, 20), _p), 1.64, _p), _p)
	worst_dark = (white - Vector3.ONE).abs().length()
	_check("emitters: step o inverse ~ AgX", worst < EMITTER_TOLERANCE and worst_dark < EXACT,
			"largest channel error %.4f, above white %s" % [worst, worst_dark])


static func _angle_deg(a: float, b: float) -> float:
	return absf(rad_to_deg(wrapf(a - b, -PI, PI)))
