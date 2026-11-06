extends SceneTree
## Probe (U-4): the ground albedo a vertical face gets its ground bounce from, as the radiance cube draws it, against an
## independent reference, per face orientation.
## Reference ("truth"): imagery_face_albedo.gd - the face's exact view-factor integral over the imagery at its finest
## resolution (polar grid around the eye, nested crops from ~ 1 m NAIP to ~ 40 m Sentinel-2 at 6 km, the global day texture
## beyond), no rings, no azimuth averaging, no shared sampling with GroundAlbedoNear.ring_albedo.
## Model ("cube"): GroundAlbedoNear.face_albedo - the albedo as sky.gdshader draws it (the sector x ring profile by horizontal
## distance and azimuth, blended to the global day texture 1500 .. 6000 m) under the same view-factor kernel; "azimuth mean":
## the same profile with its sectors averaged (what an azimuth-blind profile gives); "global": the day texture alone.
##   godot --headless --path game --script res://tests/ground_albedo_weight_probe.gd
## Prints a table per probe point; exit code 0 (a probe, not a gate).

const ImageryFaceAlbedo := preload("res://tests/imagery_face_albedo.gd")

## Probe points: Pad B (the mount; within ~ 15 m of it the imagery shows the mount's top and shadow, not ground) and the
## flats 1 km west of it.
const SITES := [["Pad B", 25.99677, -97.15804], ["flats 1 km west of Pad B", 25.99677, -97.16804]]
## The global day texture's linear RGB at the site (task 13b).
const GLOBAL_CHANNEL := [0.083, 0.108, 0.083]
const ORIENTATIONS := 8
const HEIGHTS_M := [2.0, 10.0, 40.0, 75.0, 148.0]


func _initialize() -> void:
	var imagery := ImagerySource.new(Paths.data_dir().path_join("imagery/earth/index.toml"))
	for site in SITES:
		_site(imagery, site[0], site[1], site[2])
	quit(0)


func _site(imagery: ImagerySource, label: String, lat_deg: float, lon_deg: float) -> void:
	print("ground_albedo_weight_probe: %s (%.5f, %.5f)" % [label, lat_deg, lon_deg])
	var reference := ImageryFaceAlbedo.new(imagery, lat_deg, lon_deg, true)
	if reference.crops.is_empty():
		print("  no imagery covers the reference's crops")
		return
	var global_rgb := Vector3(GLOBAL_CHANNEL[0], GLOBAL_CHANNEL[1], GLOBAL_CHANNEL[2])
	var global_luma := ImageryFaceAlbedo.luma(global_rgb)
	var profile := GroundAlbedoNear.ring_albedo(imagery, lat_deg, lon_deg)
	var n := GroundAlbedoNear.ring_count()
	print("  profile (GroundAlbedoNear.ring_albedo), luminance per sector (N, NE, E, ...) and ring (edges %s m):"
			% [GroundAlbedoNear.RING_EDGES_M])
	for s in range(GroundAlbedoNear.SECTORS):
		var row := []
		for k in range(n):
			row.append(snappedf(ImageryFaceAlbedo.luma(profile[s * n + k]), 0.0001))
		print("    sector %d: %s" % [s, row])
	print("  global day texture %.3f" % global_luma)
	var radial := []
	for r: float in [1.0, 3.0, 6.0, 10.0, 15.0, 20.0, 30.0, 50.0, 75.0, 100.0]:
		var m := 0.0
		for ip in range(360):
			m += reference.sample(r * sin(deg_to_rad(ip)), r * cos(deg_to_rad(ip))) / 360.0
		radial.append("%.0f m %.3f" % [r, m])
	print("  imagery azimuth mean at the horizontal distance: %s" % ", ".join(radial))
	var mean_profile := _azimuth_mean(profile)
	var azimuths := []
	for o in range(ORIENTATIONS):
		azimuths.append(o * TAU / ORIENTATIONS)
	print("  h (m) | face azimuth | truth (imagery, exact) | cube (error) | cube, azimuth mean of the profile (error) | global only (error)")
	for h: float in HEIGHTS_M:
		var truths := reference.face_albedo(h, azimuths, global_luma)
		var sum_err := 0.0
		var max_err := 0.0
		var sum_mean_err := 0.0
		for o in range(ORIENTATIONS):
			var truth: float = truths[o]
			var cube := ImageryFaceAlbedo.luma(GroundAlbedoNear.face_albedo(profile, h, azimuths[o], global_rgb))
			var mean := ImageryFaceAlbedo.luma(GroundAlbedoNear.face_albedo(mean_profile, h, azimuths[o], global_rgb))
			var err := 100.0 * (cube / truth - 1.0)
			sum_err += absf(err)
			max_err = maxf(max_err, absf(err))
			sum_mean_err += absf(100.0 * (mean / truth - 1.0))
			print("  %5.0f | %5.0f deg    | %.4f                 | %.4f (%+5.1f %%) | %.4f (%+5.1f %%)                      | %.4f (%+5.1f %%)" % [
					h, rad_to_deg(azimuths[o]), truth, cube, err, mean, 100.0 * (mean / truth - 1.0), global_luma,
					100.0 * (global_luma / truth - 1.0)])
		print("  %5.0f | truth over azimuth %.4f .. %.4f; cube |error| mean %.1f %%, max %.1f %%; azimuth-mean profile |error| mean %.1f %%"
				% [h, truths.min(), truths.max(), sum_err / ORIENTATIONS, max_err, sum_mean_err / ORIENTATIONS])


## The profile with every sector replaced by the mean over the sectors (an azimuth-blind profile, for comparison).
func _azimuth_mean(profile: PackedVector3Array) -> PackedVector3Array:
	var n := GroundAlbedoNear.ring_count()
	var out := profile.duplicate()
	for k in range(n):
		var m := Vector3.ZERO
		for s in range(GroundAlbedoNear.SECTORS):
			m += profile[s * n + k]
		m /= float(GroundAlbedoNear.SECTORS)
		for s in range(GroundAlbedoNear.SECTORS):
			out[s * n + k] = m
	return out
