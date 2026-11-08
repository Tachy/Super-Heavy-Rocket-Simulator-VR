# The Sun's disability glare in the eye adaptation (Atmosphere.glare_veil, look fixes T5b fix round 1, user 2026-11-08;
# eye-adaptation.md "Sun glare"): headless, exit code 1 on failure.
# - the form: L_veil = 10 E cos(theta) / theta^2 (Stiles-Holladay, CIE 146:2002; theta deg from the view axis, E lux);
# - a Sun alone in a black frame (its disc masked out by the meter) and the same Sun just outside a narrower frame give
#   the same L_a (tolerance 1e-6: the veil is the field either way), and crossing the frame edge is continuous (24.9 /
#   25.1 deg: within the 1/theta^2 change, 2 %);
# - a Sun hidden in the frame (no masked sample shows the disc) adds nothing; behind the eye (theta > 90 deg) nothing;
# - stars vanish near the Sun: at 20 deg from a 1.27e5 lux Sun the limiting magnitude (Schaefer 1990, as
#   star_visibility.gdshaderinc, the background = L_a) is below -2 (no star but Sirius' -1.46 ... none); with the Sun
#   behind, the black field leaves the dark-sky limit (> 7).
# godot --headless --path game --script res://tests/glare_veil_test.gd
extends SceneTree

const BINS := AdaptationMeter.HISTOGRAM_BINS
## [S: Allen's Astrophysical Quantities] the illuminance of a magnitude-0 star outside the air, lux.
const MAG0_LUX := 2.54e-6

var _failures := 0


func _check(label: String, ok: bool, detail: String) -> void:
	if not ok:
		_failures += 1
	print("glare_veil_test: %-52s %s %s" % [label, detail, "ok" if ok else "FAIL"])


func _initialize() -> void:
	var atmosphere := Atmosphere.new()
	atmosphere.last_sun_normal_lux = 1.27e5
	atmosphere.adaptation_surround.last_visibility = 1.0  # an open view (the surround agrees with the frame)
	var black := PackedInt32Array()
	black.resize(BINS * 2)
	black[0] = 60000  # the frame after the Sun's mask: black space
	black[BINS] = 60000

	# The form at 10 deg.
	var v10 := atmosphere.glare_veil(_view(10.0, 60.0), [0, 0])
	_check("Stiles-Holladay at 10 deg", absf(v10.x - 10.0 * 1.27e5 * cos(deg_to_rad(10.0)) / 100.0) < 1.0e-6 * v10.x,
			"%.1f cd/m^2" % v10.x)

	# Sun alone in the frame (fov 60: 20 deg off axis is inside; the meter's mask shows the disc) against the same Sun
	# just outside a narrower frame (fov 30).
	var inside := atmosphere.glare_veil(_view(20.0, 60.0), [12, 9])
	var in_frame := atmosphere.last_veil_visibility
	var outside := atmosphere.glare_veil(_view(20.0, 30.0), [0, 0])
	var la_inside := _adaptation(black, inside)
	var la_outside := _adaptation(black, outside)
	_check("Sun alone in the frame = Sun just outside", absf(la_inside / la_outside - 1.0) < 1.0e-6,
			"L_a %.1f / %.1f cd/m^2 (veil %.1f, visibility in frame %.2f)" % [la_inside, la_outside, inside.x, in_frame])
	# Across the frame edge (horizontal half angle of a 16:9 frame with a 30 deg vertical fov: 25.2 deg).
	var half := rad_to_deg(atan(tan(deg_to_rad(15.0)) * 16.0 / 9.0))
	var just_in := _adaptation(black, atmosphere.glare_veil(_view(half - 0.1, 30.0), [8, 6]))
	var just_out := _adaptation(black, atmosphere.glare_veil(_view(half + 0.1, 30.0), [0, 0]))
	_check("continuous across the frame edge", absf(just_in / just_out - 1.0) < 0.02,
			"%.1f / %.1f cd/m^2 at %.2f +- 0.1 deg" % [just_in, just_out, half])
	# Hidden behind something in the frame: no masked sample shows the disc.
	var hidden := atmosphere.glare_veil(_view(20.0, 60.0), [12, 0])
	_check("Sun hidden in the frame: no veil", hidden.x == 0.0, "%s" % hidden.x)
	var behind := atmosphere.glare_veil(_view(120.0, 60.0), [0, 0])
	_check("Sun behind the eye: no veil", behind.x == 0.0, "%s" % behind.x)
	var enclosed_atmosphere := Atmosphere.new()
	enclosed_atmosphere.last_sun_normal_lux = 1.27e5
	enclosed_atmosphere.adaptation_surround.last_visibility = 0.0
	var enclosed := enclosed_atmosphere.glare_veil(_view(20.0, 30.0), [0, 0])
	_check("enclosed view, Sun outside the frame: no veil", enclosed.x == 0.0, "%s" % enclosed.x)
	var at_centre := atmosphere.glare_veil(_view(0.0, 60.0), [12, 9])
	_check("Sun at the view axis: finite (angle held at the disc)", is_finite(at_centre.x) and at_centre.x > 1.0e6,
			"%s cd/m^2" % at_centre.x)

	# Stars near the Sun: the limiting magnitude over the adapted background (stars.gdshader takes max(sky, L_a)).
	var limit_near := _limiting_magnitude(la_outside)
	_check("stars vanish 20 deg from the Sun", limit_near < -2.0, "limiting magnitude %.2f (L_a %.0f)" % [limit_near, la_outside])
	var limit_dark := _limiting_magnitude(maxf(_adaptation(black, behind), 0.0))
	_check("Sun behind: dark-sky stars", limit_dark > 7.0, "limiting magnitude %.2f" % limit_dark)
	print("glare_veil_test: %s" % ("PASS" if _failures == 0 else "%d FAILED" % _failures))
	quit(1 if _failures > 0 else 0)


## A desktop view (16:9, vertical fov in deg) looking along -Z with the Sun theta_deg off the axis to the right.
func _view(theta_deg: float, fov_deg: float) -> Dictionary:
	var projection := Projection.create_perspective(fov_deg, 16.0 / 9.0, 0.05, 1000.0)
	var t := deg_to_rad(theta_deg)
	return {"head": Basis.IDENTITY, "eyes": [[Basis.IDENTITY, projection]], "sun": Vector3(sin(t), 0.0, -cos(t))}


## The game's rule (PROTECT, key 0.18) on the field with the veil.
func _adaptation(counts: PackedInt32Array, veil: Vector2) -> float:
	var field := maxf(AdaptationMeter.field_log_average(counts, 0), 0.0) + veil.x  # as Atmosphere._take_field_measurement
	var anchor := maxf(AdaptationMeter.field_anchor(counts, 0), 0.0) + veil.x
	return Atmosphere.field_adaptation_luminance(Atmosphere.FieldRule.PROTECT, maxf(field, 0.0), maxf(anchor, 0.0),
			EyeAdaptation.FIELD_MID_GREY)


## Schaefer 1990's naked-eye threshold (star_visibility.gdshaderinc) as a magnitude.
func _limiting_magnitude(background_cd_m2: float) -> float:
	var b_nl := background_cd_m2 * 314159.27
	var night := 1.0 + sqrt(1.2589e-2 * b_nl)
	var day := 1.0 + sqrt(1.2589e-6 * b_nl)
	var lux := minf(1.5849e-10 * night * night, 4.4668e-9 * day * day) * 10.763910
	return -2.5 * log(lux / MAG0_LUX) / log(10.0)
