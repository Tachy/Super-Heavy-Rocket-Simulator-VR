class_name EarthShadowLength
extends RefCounted
## render_check --shadow_length (moon procedural surface plan Task 16): the Pad B tower's Sun shadow on the ground, measured
## in the image against the casters' geometry. Once the view has settled with the Earth's near shadows on, the frame is
## kept (A), the shadows are switched off for a few frames and the frame kept again (B); the shadow is where the radiance
## ratio A / B falls below the midpoint between 1 (lit) and the umbra's ratio (the direct Sun's 50 % contour: the
## geometric edge of a caster wider than the penumbra). Each shadowed pixel's ray is intersected with the site's ground
## plane; the farthest one along the anti-Sun horizontal from the tower's site origin is the measured tip. The predicted
## tip is the largest (P - O).a + h(P) / tan(e) over every vertex of the tower's casting geometry drawn at this camera
## distance (MeshInstance3D / MultiMeshInstance3D under the TowerView with cast_shadow on; the farthest vertex is the
## head's roof, h ~ 147.9 m). Tolerance: 2 shadow-map texels of the split holding the tip (on the ground along the
## anti-Sun line: texel / sin e) + 2 image pixels' ground size; with the PCSS softening on also half the penumbra's ground
## length there (the solar disc's 0.533 deg x the vertex's distance h / sin e, / sin e on the ground). The geometric gate
## is the hard run (--earth_shadow_soft=off: the map's resolution alone); the soft run checks the penumbra on top.
## Beyond the plateau the ground is the terrain: the tip's ground height over the site enters both sides (below).
## View: an eye high above the pad looking (nearly) straight down (--lat / --lon / --eye_height / --elevation=-89.9),
## so a structure's own pixels fall near its footprint and never beyond the ground shadow's tip.

const WAIT_FRAMES := 4
## Pixels whose ground point lies farther than this from the anti-Sun line through the tower (m) are not the tower's.
const LATERAL_LIMIT_M := 120.0

var stage := 0
var frames := 0
var image_on: Image
var passed := false
var report := ""


## Advances the probe by a frame; true once done (report / passed set).
func step(world: World, viewport: Viewport) -> bool:
	if stage == 0:
		image_on = viewport.get_texture().get_image()
		EarthNearShadow.enabled = false
		stage = 1
		return false
	if stage == 1:
		frames += 1
		if frames < WAIT_FRAMES:
			return false
		var image_off := viewport.get_texture().get_image()
		EarthNearShadow.enabled = true
		_evaluate(world, viewport, image_off)
		stage = 2
	return true


func _evaluate(world: World, viewport: Viewport, image_off: Image) -> void:
	if world.towers.is_empty():
		report = "no tower at this site"
		return
	var tower: TowerView = world.towers[0]
	var camera := viewport.get_camera_3d()
	var origin := tower.global_position
	# The ground view's geodetic up (the eye stands above the pad: the same vertical within 1e-5 rad).
	var up := Frames.to_render(world.camera.site_up_gcrf).normalized()
	var to_sun := world.sun_light.global_basis.z.normalized()
	var sin_e := to_sun.dot(up)
	var horizontal := to_sun - up * sin_e
	var tan_e := sin_e / horizontal.length()
	var along_dir := -horizontal.normalized()  # anti-Sun, horizontal
	var side_dir := up.cross(along_dir).normalized()
	# The prediction: every casting vertex drawn at this camera distance.
	var best := -INF
	var best_height := 0.0
	var best_point := Vector3.ZERO
	var vertices := 0
	for node in tower.find_children("*", "GeometryInstance3D", true, false):
		var geometry := node as GeometryInstance3D
		if geometry.cast_shadow == GeometryInstance3D.SHADOW_CASTING_SETTING_OFF or not geometry.is_visible_in_tree():
			continue
		var distance := camera.global_position.distance_to(geometry.global_transform * geometry.get_aabb().get_center())
		if distance < geometry.visibility_range_begin or (geometry.visibility_range_end > 0.0
				and distance >= geometry.visibility_range_end):
			continue
		var mesh: Mesh = null
		var transforms: Array[Transform3D] = []
		if geometry is MeshInstance3D:
			mesh = (geometry as MeshInstance3D).mesh
			transforms.append(geometry.global_transform)
		elif geometry is MultiMeshInstance3D:
			var multimesh := (geometry as MultiMeshInstance3D).multimesh
			if multimesh == null:
				continue
			mesh = multimesh.mesh
			for i in multimesh.instance_count:
				transforms.append(geometry.global_transform * multimesh.get_instance_transform(i))
		if mesh == null:
			continue
		for surface in mesh.get_surface_count():
			var points: PackedVector3Array = mesh.surface_get_arrays(surface)[Mesh.ARRAY_VERTEX]
			for t in transforms:
				for p in points:
					var q := t * p - origin
					var h := q.dot(up)
					var along := q.dot(along_dir) + h / tan_e
					vertices += 1
					if along > best:
						best = along
						best_height = h
						best_point = q
	# The measurement: the shadowed ground pixels (ratio below the 50 % contour of the direct Sun).
	var width := image_off.get_width()
	var height := image_off.get_height()
	var ratios := PackedFloat32Array()
	ratios.resize(width * height)
	var umbra := 1.0
	for y in height:
		for x in width:
			var lum_off := _luminance(image_off.get_pixel(x, y))
			var r := _luminance(image_on.get_pixel(x, y)) / lum_off if lum_off > 1.0e-4 else 1.0
			ratios[y * width + x] = r
	var sorted := ratios.duplicate()
	sorted.sort()
	umbra = sorted[int(0.001 * sorted.size())]  # the deepest shadow (0.1 % quantile: robust to single pixels)
	var threshold := 0.5 * (1.0 + umbra)
	var tip := -INF
	var tip_pixel := Vector2.ZERO
	var shadow_pixels := 0
	var scale := Vector2(viewport.get_visible_rect().size) / Vector2(width, height)
	for y in height:
		for x in width:
			if ratios[y * width + x] >= threshold:
				continue
			var screen := Vector2(x + 0.5, y + 0.5) * scale
			var ray_origin := camera.project_ray_origin(screen)
			var ray := camera.project_ray_normal(screen)
			var denominator := ray.dot(up)
			if absf(denominator) < 1.0e-6:
				continue
			var t := (origin - ray_origin).dot(up) / denominator
			if t <= 0.0:
				continue
			var g := ray_origin + ray * t - origin
			if absf(g.dot(side_dir)) > LATERAL_LIMIT_M:
				continue
			shadow_pixels += 1
			var along := g.dot(along_dir)
			if along > tip:
				tip = along
				tip_pixel = Vector2(x, y)
	# Beyond the pad's plateau (150 m + 50 m apron) the ground is the terrain, not the site's plane: the tip's ground
	# height over the site (HeightSource, the drawn terrain's) moves both the measured point (the tip pixel's ray meets
	# the ground there) and the prediction ((h - dh) / tan e). Three fixed-point passes (the point moves by dh / tan of
	# the ray's slope: converged to mm in a nadir view).
	var dh := 0.0
	var screen_tip := (tip_pixel + Vector2(0.5, 0.5)) * scale
	var tip_ray_origin := camera.project_ray_origin(screen_tip)
	var tip_ray := camera.project_ray_normal(screen_tip)
	for pass_index in 3:
		var ground := tip_ray_origin + tip_ray * ((origin + up * dh - tip_ray_origin).dot(up) / tip_ray.dot(up)) - origin
		tip = ground.dot(along_dir)
		dh = _ground_height_over_site(world, tower, ground)
	best -= dh / tan_e
	# Tolerance: the split's texel at the tip's view depth, the image pixel's ground size there, half the penumbra.
	var tip_world := origin + along_dir * tip
	var depth := -(camera.global_transform.affine_inverse() * tip_world).z
	var projection := camera.get_camera_projection()
	var tan_y := tan(deg_to_rad(0.5 * projection.get_fov())) / projection.get_aspect()
	var range_m := world.earth_near_shadow.range_m
	var texels: Array = MoonNearShadow.split_texels(range_m, projection.get_z_near(), tan_y, projection.get_aspect())
	var split_far: Vector4 = texels[1]
	var texel_v: Vector4 = texels[0]
	var texel := texel_v.w
	for i in 4:
		if depth < split_far[i]:
			texel = texel_v[i]
			break
	var eye_distance := camera.global_position.distance_to(tip_world)
	var pixel_m := eye_distance * 2.0 * tan_y / float(height) / maxf(absf((camera.global_position - tip_world).normalized().dot(up)), 0.1)
	# On the ground along the anti-Sun line a texel and the penumbra are stretched by 1 / sin e (the light's rays meet
	# the ground at e): the texel's footprint texel / sin e, the penumbra theta x (h / sin e) / sin e.
	var caster_to_tip := best_height / sin_e  # the vertex to its shadow on the ground, along the Sun's ray
	var penumbra := 0.0
	if EarthNearShadow.soft:
		penumbra = caster_to_tip * deg_to_rad(MoonNearShadow.solar_disc_diameter_deg(Atmosphere.ASTRONOMICAL_UNIT_M)) / sin_e
	var tolerance := 2.0 * texel / sin_e + 2.0 * pixel_m + 0.5 * penumbra
	var error := tip - best
	passed = shadow_pixels > 0 and absf(error) <= tolerance
	report = ("Sun elevation %.3f deg; ground at the tip %+.2f m over the site; predicted tip %.2f m along the anti-Sun line "
			+ "(vertex %.2f m high, h / tan e = %.2f m, %d vertices); measured %.2f m (pixel %s, %d shadow pixels, umbra ratio %.3f, threshold %.3f); error %+.2f m, "
			+ "tolerance %.2f m (texel %.3f m at depth %.0f m of range %.0f, pixel %.2f m, penumbra %.2f m): %s") % [
			rad_to_deg(atan(tan_e)), dh, best, best_height, best_height / tan_e, vertices, tip, tip_pixel, shadow_pixels,
			umbra, threshold, error, tolerance, texel, depth, range_m, pixel_m, penumbra, "PASS" if passed else "FAIL"]


## The terrain's height (HeightSource, as drawn: the site's plateau rule included) at a ground offset from the tower's
## site (render axes) over the site's height; offsets of a few km: the local east / north plane.
static func _ground_height_over_site(world: World, tower: TowerView, offset: Vector3) -> float:
	var site: Dictionary = {}
	for candidate: Dictionary in world.sites:
		if candidate["id"] == tower.site_id:
			site = candidate
	if site.is_empty() or world._earth_heights == null:
		return 0.0
	var east := offset.dot(Frames.to_render(world.camera.site_east_gcrf).normalized())
	var north := offset.dot(Frames.to_render(world.camera.site_north_gcrf).normalized())
	var lat := float(site["latitude_deg"]) + rad_to_deg(north / World.EARTH_MEAN_RADIUS_M)
	var lon := float(site["longitude_deg"]) + rad_to_deg(east / (World.EARTH_MEAN_RADIUS_M * cos(deg_to_rad(lat))))
	var sample: Dictionary = world._earth_heights.sample_band_limited(PackedFloat64Array([lat]),
			PackedFloat64Array([lon]), 0.0)
	return float((sample["height_m"] as PackedFloat64Array)[0]) - float(site["height_m"])


static func _luminance(c: Color) -> float:
	var l := c.srgb_to_linear()
	return 0.2126 * l.r + 0.7152 * l.g + 0.0722 * l.b
