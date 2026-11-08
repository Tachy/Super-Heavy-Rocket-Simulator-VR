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
#   display white;
# - the glow bound (fix round 1): no written channel above max(scene's largest, glow threshold), hue and the round trip
#   kept under it, the noon aureole held below the threshold, the grey lift above white, u >= 0 (_check_glow_bound).
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
	# The pre-T13 properties on AgX's grey curve (--display_curve=agx), then the fixed display curve (look fixes T13).
	DisplayTone.display_linear = false
	_check_grey()
	_check_glow_bound()
	DisplayTone.display_linear = true
	_check_display_curve()
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


## The fixed display curve (T13): grey shown on it exactly (AgX o step), linear to the knee (the white plate at display
## white x knee / ..., the field Y_b DAY_EXPOSURE at Y_b), C1 and monotonic at the knee, the inverse exact, white reached.
func _check_display_curve() -> void:
	var worst := 0.0
	var worst_inverse := 0.0
	var y := 1.0e-4
	while y < 16.0:
		worst = maxf(worst, (DisplayTone.agx(DisplayTone.apply(Vector3.ONE * y, 1.0, _p), _p)
				- Vector3.ONE * DisplayTone.display_curve(y, _p)).length())
		var d := DisplayTone.display_curve(y, _p)
		if d < 0.999:
			worst_inverse = maxf(worst_inverse, absf(DisplayTone.display_curve_inverse(d, _p) / y - 1.0))
		y *= 1.25
	_check("T13 grey shown on the display curve", worst < EXACT, "largest error %s" % worst)
	_check("T13 display curve inverse", worst_inverse < 1e-6, "largest relative error %s" % worst_inverse)
	var field := EyeAdaptation.FIELD_MID_GREY * Atmosphere.DAY_EXPOSURE
	_check("T13 field at Y_b of display white", absf(DisplayTone.display_curve(field, _p) - 0.2) < 1e-9,
			"%s" % DisplayTone.display_curve(field, _p))
	var knee := DisplayTone.DISPLAY_KNEE / DisplayTone.DISPLAY_SLOPE
	var h := 1e-6
	var left := (DisplayTone.display_curve(knee, _p) - DisplayTone.display_curve(knee - h, _p)) / h
	var right := (DisplayTone.display_curve(knee + h, _p) - DisplayTone.display_curve(knee, _p)) / h
	_check("T13 display curve C1 at the knee", absf(left - right) < 1e-4 * left, "slopes %s / %s" % [left, right])
	var monotonic := true
	var previous := -1.0
	y = 1.0e-4
	while y < 20.0:
		var d := DisplayTone.display_curve(y, _p)
		monotonic = monotonic and (d > previous or d >= 1.0)
		previous = d
		y *= 1.05
	_check("T13 display curve monotonic, white at AgX's white", monotonic and DisplayTone.display_curve(_p[4], _p) > 0.9999,
			"at white %s" % DisplayTone.display_curve(_p[4], _p))


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


## Glow bound (fix round 1, review I1): with world.gd's glow (threshold 4, strength 1) no written channel exceeds
## max(4, the scene's largest channel, the largest written by the step without the Hunt boost) - over random saturated
## colours (scene luminance 0.05 .. 16, S 1.64) and the noon aureole (scene (1.75, 2.15, 2.35), which wrote 4.9 at S
## 1.58 without the bound); hue still kept (0.5 deg) and AgX o step = target; display-referred emitters at S 1 written
## as without the bound (1e-4 relative: their glow is not cut); above AgX's white the grey lift is
## display white with the largest channel at most doubled; the AgX curve outputs u = OUTSET^-1 d are >= 0 on the whole
## sRGB cube (review M4; reachable guards it).
func _check_glow_bound() -> void:
	var environment := Environment.new()
	environment.glow_enabled = true
	environment.glow_hdr_threshold = 4.0
	environment.glow_strength = 1.0
	var glow := DisplayTone.glow_bound(environment)
	environment.glow_enabled = false
	_check("glow bound: threshold / strength, off = INF", glow == 4.0 and DisplayTone.glow_bound(environment) == INF,
			"bound %s" % glow)
	var rng := RandomNumberGenerator.new()
	rng.seed = 20261107
	var worst_excess := 0.0
	var worst_hue := 0.0
	var worst_round := 0.0
	var count := 0
	var floored := 0
	for i in range(1500):
		var c := Vector3(rng.randf(), rng.randf(), rng.randf())
		c = c * c + Vector3.ONE * 1e-3
		var y := exp(rng.randf_range(log(0.05), log(16.0)))
		var x := c * (y / DisplayTone.LUMINANCE_WEIGHTS.dot(c))
		var written := DisplayTone.apply(x, 1.64, _p, glow)
		# the Hunt boost adds no glow: written <= the threshold, the scene or what the step writes without the boost
		var no_hunt := DisplayTone.apply(x, 1.0, _p)
		var limit := maxf(maxf(glow, maxf(x.x, maxf(x.y, x.z))), maxf(no_hunt.x, maxf(no_hunt.y, no_hunt.z)))
		if maxf(written.x, maxf(written.y, written.z)) > maxf(glow, maxf(x.x, maxf(x.y, x.z))) * (1.0 + EXACT):
			floored += 1  # the bound left less chroma than S = 1: the S = 1 colour (unbounded)
		worst_excess = maxf(worst_excess, maxf(written.x, maxf(written.y, written.z)) / limit - 1.0)
		var shown := DisplayTone.agx(written, _p)
		var shown_target := DisplayTone.target(x, 1.64, _p, glow)
		if y < _p[4]:
			worst_round = maxf(worst_round, (shown - shown_target).length())
		var lab := DisplayTone.to_oklab(shown_target)
		if Vector2(lab.y, lab.z).length() > 0.01:
			worst_hue = maxf(worst_hue, _angle_deg(DisplayTone.hue_angle(shown_target), DisplayTone.hue_angle(x)))
			count += 1
	_check("glow bound: written <= max(4, scene, S = 1)", worst_excess < EXACT,
			"largest excess %s (1500 colours, %d at the S = 1 floor)" % [worst_excess, floored])
	_check("glow bound: hue kept, AgX o step = target", worst_hue < HUE_TOLERANCE_DEG and worst_round < EXACT,
			"hue %.4f deg over %d, round trip %s" % [worst_hue, count, worst_round])
	var aureole := Vector3(1.75, 2.15, 2.35)
	var free := DisplayTone.apply(aureole, 1.58, _p)
	var bounded := DisplayTone.apply(aureole, 1.58, _p, glow)
	var free_max := maxf(free.x, maxf(free.y, free.z))
	var bounded_max := maxf(bounded.x, maxf(bounded.y, bounded.z))
	_check("glow bound: noon aureole below the threshold", free_max > glow and bounded_max <= glow * (1.0 + EXACT),
			"largest channel %.3f unbounded, %.3f bounded" % [free_max, bounded_max])
	var worst_emitter := 0.0
	for c: Vector3 in [Vector3(3.0, 1.2, 0.2), Vector3(4.5, 1.0, 0.1), Vector3(1.0, 0.45, 0.1), Vector3(6.0, 1.5, 0.3)]:
		var x := DisplayTone.tone_inverse(c, _p)
		var written := DisplayTone.apply(x, 1.0, _p, glow)
		worst_emitter = maxf(worst_emitter, (written - DisplayTone.apply(x, 1.0, _p)).length() / c.length())
	_check("glow bound: emitters at S 1 not cut by it", worst_emitter < 1e-4,
			"largest relative change %s" % worst_emitter)
	var worst_white := 0.0
	var worst_lift := 0.0
	for x: Vector3 in [Vector3(40, 18, 3), Vector3(30, 20, 10), Vector3(5, 20, 60), Vector3(17, 17, 17)]:
		var written := DisplayTone.apply(x, 1.64, _p, glow)
		worst_white = maxf(worst_white, (DisplayTone.agx(written, _p) - Vector3.ONE).abs().length())
		worst_lift = maxf(worst_lift, maxf(written.x, maxf(written.y, written.z)) / maxf(x.x, maxf(x.y, x.z)))
	_check("above white: display white, largest x <= 2", worst_white < EXACT and worst_lift <= 2.0,
			"white error %s, largest lift %.3f" % [worst_white, worst_lift])
	# u >= 0 (review M4): OUTSET^-1 has only positive entries, so every d >= 0 of the sRGB cube gives u >= 0 - the
	# explicit test in reachable is a guard; a d with a negative u (outside the cube) is rejected.
	var outset_inverse := DisplayTone.AGX_OUTSET.inverse()
	var smallest := INF
	for k in range(3):
		smallest = minf(smallest, minf(outset_inverse[k].x, minf(outset_inverse[k].y, outset_inverse[k].z)))
	var outside := DisplayTone.AGX_OUTSET * Vector3(0.5, -0.05, 0.5)
	_check("reachable: u >= 0 on the cube, guarded", smallest > 0.0 and not DisplayTone.reachable(outside, _p),
			"smallest OUTSET^-1 entry %.4f" % smallest)


static func _angle_deg(a: float, b: float) -> float:
	return absf(rad_to_deg(wrapf(a - b, -PI, PI)))
