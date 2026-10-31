# Rule test of the eye adaptation through sunset (eye-adaptation.md): the adaptation target and the settled cone gain
# must vary continuously with the Sun's elevation (no on / off gate, no per-sample cutoff inside the aureole). Sweeps the
# Sun from +8 to -18 deg in 0.1 deg steps at the pad and 20 km up, with the Sun's azimuth fixed; the largest change of
# ln(target) and ln(gain) per step is gated. Headless: godot --headless --path game --script res://tests/sunset_adaptation_test.gd
# STARSIM_VERBOSE=1 prints every step.
extends SceneTree

const PAD_LAT_DEG := 25.9967
const PAD_LON_DEG := -97.1543
const SUN_START_DEG := 8.0
const SUN_END_DEG := -18.0
const STEP_DEG := 0.1
## [E] Two gates. (1) The glow term's second difference of ln(glow) per step (a smooth curve has |d2| below 0.1 above the floor; an
## on / off gate or a sample entering a cutoff is a jump of 0.1 and more): MAX_GLOW_D2_LN. (2) The target's and the settled
## gain's largest step of ln per 0.1 deg wherever the glow term takes part (a step between two points that both sit on the
## plate is the Sun setting behind the eye's horizon: physical and steep from altitude): MAX_STEP_LN, above the natural
## fall of the darkest twilight (about 0.3 per step at -13 deg); the hard edges this test was written for were 0.9 to 1.4.
const MAX_GLOW_D2_LN := 0.15  # includes the kink where the floor below cuts in (~0.13)
const MAX_STEP_LN := 0.35
## [E] Luminances are compared as max(value, this) (cd/m^2): the tables' half-float quantisation makes the logarithm of
## values below ~0.1 jitter by 10 % and more from step to step (seen from Sun -10.7 deg on), and the eye there is on the
## cone threshold's flat part anyway. A gate that switches a term on from 0 still shows as a jump from this floor.
const FLOOR_CD_M2 := 0.3

var _core := StarsimCore.new()
var _atmosphere := Atmosphere.new()
var _failures := 0


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	if not _core.open(Paths.data_dir().path_join("spice")):
		push_error("SPICE kernels missing")
		quit(1)
		return
	_atmosphere.start_loading(_core, Paths.config_dir(), Paths.data_dir().path_join("atmosphere"))
	if not _atmosphere.wait_until_loaded():
		push_error("atmosphere tables not loaded: " + _atmosphere.status_text)
		quit(1)
		return
	for height_m in [2.0, 20000.0]:
		_sweep(height_m)
	print("sunset_adaptation_test: %s (%d failures)" % ["PASS" if _failures == 0 else "FAIL", _failures])
	quit(1 if _failures > 0 else 0)


func _sweep(height_m: float) -> void:
	var lat := deg_to_rad(PAD_LAT_DEG)
	var lon := deg_to_rad(PAD_LON_DEG)
	var up := Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
	var north := Vector3(-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat))
	var eye := HeightSource.geodetic_to_fixed(PAD_LAT_DEG, PAD_LON_DEG, height_m)
	var reference := _atmosphere.reference_luminance(1.0)
	var verbose := OS.get_environment("STARSIM_VERBOSE") == "1"
	var targets: Array[float] = []
	var gains: Array[float] = []
	var glows: Array[float] = []
	var on_plate: Array[bool] = []
	var steps := int(round((SUN_START_DEG - SUN_END_DEG) / STEP_DEG))
	for k in range(steps + 1):
		var elevation := SUN_START_DEG - k * STEP_DEG
		var e := deg_to_rad(elevation)
		var sun := (up * sin(e) + north * cos(e)).normalized()
		var target := _atmosphere.adaptation_luminance(eye, sun, 1.0, 1.0)
		var gain := EyeAdaptation.cone_gain(clampf(target, EyeAdaptation.LUMINANCE_FLOOR_CD_M2, reference), reference)
		targets.append(target)
		gains.append(gain)
		glows.append(_atmosphere.last_adaptation_glow_cd_m2)
		on_plate.append(_atmosphere.last_adaptation_down_cd_m2 >= maxf(_atmosphere.last_adaptation_glow_cd_m2,
				_atmosphere.last_adaptation_up_cd_m2))
		if verbose:
			print("  h %6.0f m sun %6.2f deg target %10.4f gain %10.3f (down %.3f up %.3f glow %.3f)" % [height_m,
					elevation, target, gain, _atmosphere.last_adaptation_down_cd_m2, _atmosphere.last_adaptation_up_cd_m2,
					_atmosphere.last_adaptation_glow_cd_m2])
	var worst_step := 0.0
	var worst_step_at := 0.0
	var worst_d2 := 0.0
	var worst_d2_at := 0.0
	for k in range(1, steps + 1):
		var elevation := SUN_START_DEG - k * STEP_DEG
		if not (on_plate[k] and on_plate[k - 1]):
			var d := absf(log(maxf(targets[k], FLOOR_CD_M2) / maxf(targets[k - 1], FLOOR_CD_M2)))
			if targets[k] >= FLOOR_CD_M2 and targets[k - 1] >= FLOOR_CD_M2:
				d = maxf(d, absf(log(gains[k] / gains[k - 1])))
			if d > worst_step:
				worst_step = d
				worst_step_at = elevation
		# (above the fade-out start the glow is faded to nothing under the larger plate: the target step covers it)
		if k >= 2 and elevation <= Atmosphere.TWILIGHT_GLOW_FADE_START_DEG:
			var l0 := log(maxf(glows[k - 2], FLOOR_CD_M2))
			var l1 := log(maxf(glows[k - 1], FLOOR_CD_M2))
			var l2 := log(maxf(glows[k], FLOOR_CD_M2))
			var d2 := absf(l2 - 2.0 * l1 + l0)
			if d2 > worst_d2:
				worst_d2 = d2
				worst_d2_at = elevation
	var ok := worst_step <= MAX_STEP_LN and worst_d2 <= MAX_GLOW_D2_LN
	print("sunset_adaptation_test: %s h %.0f m: largest step of ln target / gain %.3f (at Sun %.1f deg; limit %.2f), "
			% ["ok  " if ok else "FAIL", height_m, worst_step, worst_step_at, MAX_STEP_LN]
			+ "largest second difference of ln glow %.3f (at Sun %.1f deg; limit %.2f)" % [worst_d2, worst_d2_at,
			MAX_GLOW_D2_LN])
	if not ok:
		_failures += 1
