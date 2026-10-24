# GPU check of the atmosphere shader include (M8 atmosphere, Task 5): renders the sky (shaders/sky.gdshader, nothing
# else in the scene) into an HDR SubViewport for sample views, reads the float pixel on the view direction and compares
# it with the core's CPU evaluation (StarsimCore.atmosphere_sky_luminance x Atmosphere.luminance_scale) within 2 % per
# channel. Linear tone mapper, exposure 1, glow off: the read value is the scene value. Exit code 1 on failure.
# Task 6 adds the Earth's surfaces (terrain_earth.gdshader, earth.gdshader incl. scaled space: lit by the transmitted
# Sun and the sky light, seen through the atmosphere; 3 %). STARSIM_VERBOSE=1 prints the surface samples' parts.
# Needs a GPU (no --headless): godot --path game --script res://tests/atmosphere_check.gd
extends SceneTree

const TOLERANCE := 0.02
## Channels below this fraction of the sample's brightest channel are compared absolutely (at that level).
const RELATIVE_FLOOR := 1e-3
const VIEW_SIZE := 65  ## odd: the centre pixel's ray is the camera axis
const SETTLE_FRAMES := 3
## OLP-1 (config/launch_sites.toml, height [D] ellipsoidal).
const PAD_LAT_DEG := 25.9967
const PAD_LON_DEG := -97.1543
const PAD_HEIGHT_M := -21.11
const MOON_DISTANCE_M := 3.844e8
const AU_M := 149597870700.0
const EPOCH_UTC := "2024-04-08T18:42:00"
## Surface samples (Task 6): a point near Cape Canaveral, the terrain pixel this far west of the camera.
const CAPE_LAT_DEG := 28.45
const CAPE_LON_DEG := -80.6
const CAPE_POINT_DISTANCE_M := 20000.0
const SURFACE_TOLERANCE := 0.03
## The surfaces' albedo: every texel of the bound day texture is this sRGB-8 colour, so the texel the shader reads is
## known exactly (decoded sRGB -> linear on the GPU as Color.srgb_to_linear on the CPU). The real day texture would
## make "the same texel" depend on mip level and anisotropic filtering at the grazing terrain view.
const SURFACE_ALBEDO_8 := Color8(150, 120, 90)
const WGS84_A_M := 6378137.0
const WGS84_FLATTENING := 1.0 / 298.257223563

var _core := StarsimCore.new()
var _atmosphere := Atmosphere.new()
var _viewport := SubViewport.new()
var _camera := Camera3D.new()
var _earth_body := CelestialBody.new()  ## the Earth's ellipsoid mesh with earth.gdshader (scaled space far out)
var _earth_material := ShaderMaterial.new()
var _terrain_parent := Node3D.new()  ## Earth-fixed frame of the terrain quad (unscaled)
var _terrain_quad := MeshInstance3D.new()  ## one camera-facing quad with terrain_earth.gdshader
var _terrain_material := ShaderMaterial.new()
var _earth_to_gcrf := Basis.IDENTITY
var _failures := 0
var _samples := 0


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	if not _core.open(Paths.data_dir().path_join("spice")):
		_fail_and_quit("SPICE kernels missing")
		return
	_earth_to_gcrf = _core.earth_fixed_to_gcrf(_core.utc_to_tdb_s(EPOCH_UTC))  # a real, non-trivial rotation
	if OS.get_cmdline_user_args().has("--f32"):
		_atmosphere.table_format = Image.FORMAT_RGBAF  # compare with the full-float tables
	print("atmosphere_check: table format %s" % ("RGBAF" if _atmosphere.table_format == Image.FORMAT_RGBAF else "RGBAH"))
	_atmosphere.start_loading(_core, Paths.config_dir(), Paths.data_dir().path_join("atmosphere"))
	if not _atmosphere.wait_until_loaded():
		_fail_and_quit("atmosphere tables not loaded: " + _atmosphere.status_text)
		return
	_setup_scene()

	# 1. The pad looking up, Sun 60 deg (and two other directions).
	var pad := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, 2.0)
	var sun_60 := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 180.0, 60.0)
	await _sample("pad, zenith, Sun 60 deg", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 0.0, 90.0), sun_60)
	await _sample("pad, az 90 el 30, Sun 60 deg", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0, 30.0), sun_60)
	await _sample("pad, az 0 el 5, Sun 60 deg", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 0.0, 5.0), sun_60)

	# 2. The pad at sunset looking at the Sun (just above it and a little higher).
	var sun_set := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, 0.5)
	await _sample("sunset, toward the Sun el 1", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, 1.0), sun_set)
	await _sample("sunset, toward the Sun el 5", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 272.0, 5.0), sun_set)
	await _sample("sunset, away from the Sun el 10", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0, 10.0), sun_set)

	# 3. 400 km above the ellipsoid looking at the limb (tangent heights 10 / 30 / 60 km), Sun 30 deg at the camera.
	var orbit := _geodetic(10.0, -60.0, 400000.0)
	var sun_orbit := _local_direction(10.0, -60.0, 90.0, 30.0)
	for tangent_km in [10.0, 30.0, 60.0]:
		await _sample("400 km, limb at %.0f km" % tangent_km, orbit, _limb_direction(orbit, Vector3(0, 0, 1),
				tangent_km * 1000.0), sun_orbit)

	# 4. Moon distance looking at the Earth's limb (Review Focus 2): tangent heights 20 / 50 km on the lit side.
	var far_dir := Vector3(cos(0.3), sin(0.3), 0.0)
	var far := far_dir * MOON_DISTANCE_M
	var side := Vector3(-sin(0.3), cos(0.3), 0.0)  # in the equator plane, perpendicular to the camera direction
	var sun_far := (side * 0.8 + far_dir * 0.6).normalized()  # lights the limb on the `side` edge
	for tangent_km in [20.0, 50.0]:
		await _sample("Moon distance, limb at %.0f km" % tangent_km, far, _limb_direction(far, side,
				tangent_km * 1000.0), sun_far)

	# 5. Pad camera at -21 m ellipsoidal (Review Focus 1): 129 m above the -150 m model ground; grazing rays are the
	# most sensitive to the camera height. The control renders the grazing ray with the altitude uniform forced to the
	# model ground: it must differ, proving the shader takes the camera height from atmosphere_camera_altitude_m (the
	# float64-derived uniform) and not from its float32 position.
	var low := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, PAD_HEIGHT_M)
	var sun_30 := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 120.0, 30.0)
	for elevation in [0.3, 1.0, 3.0, 20.0]:
		await _sample("pad at -21 m, az 90 el %.1f" % elevation, low,
				_local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0, elevation), sun_30)
	print("atmosphere_check: camera altitude above the model ground at -21 m: %.3f m" % _atmosphere.camera_altitude_m)
	await _altitude_control(low, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0, 0.3), sun_30)

	# 6. Ground rays (sky.gdshader ground_radiance: atm_to_point + atm_sun_and_sky, the stretched-normal tilt): from the
	# pad a few km out, from 400 km (atm_to_point entering from space) and from the Moon distance.
	await _ground_sample("pad, ground 2 deg down (~4 km)", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0, -2.0),
			sun_60)
	await _ground_sample("400 km, ground 40 deg down", orbit, _local_direction(10.0, -60.0, 0.0, -40.0), sun_orbit)
	await _ground_sample("Moon distance, Earth's disc", far, (-far_dir + side * 0.01).normalized(), sun_far)

	# 7. Earth surfaces (Task 6): a terrain pixel (terrain_earth.gdshader) 20 km from a camera 2 m above the ground at
	# the Cape, and Earth-disc pixels of the ellipsoid mesh (earth.gdshader) from 400 km and from the Moon distance (the
	# scaled-space mesh): albedo x (sun + sky lux) / pi x T + to-point luminance, 3 %.
	var cape := _geodetic(CAPE_LAT_DEG, CAPE_LON_DEG, 2.0)
	var cape_point := _geodetic(CAPE_LAT_DEG, CAPE_LON_DEG - CAPE_POINT_DISTANCE_M / (111319.49 * cos(deg_to_rad(
			CAPE_LAT_DEG))), 0.0)
	var sun_cape := _local_direction(CAPE_LAT_DEG, CAPE_LON_DEG, 120.0, 40.0)
	await _surface_sample("Cape, terrain 20 km", cape, cape_point, _geodetic_up(cape_point), sun_cape, true)
	var disc_400 := _ellipsoid_hit(orbit, _local_direction(10.0, -60.0, 0.0, -40.0))
	await _surface_sample("400 km, Earth mesh 40 deg down", orbit, disc_400, _geodetic_up(disc_400), sun_orbit, false)
	var disc_far := _ellipsoid_hit(far, (-far_dir + side * 0.01).normalized())
	await _surface_sample("Moon distance, Earth mesh (scaled)", far, disc_far, _geodetic_up(disc_far), sun_far, false)

	print("atmosphere_check: %d samples, %d failures" % [_samples, _failures])
	print("atmosphere_check: %s" % ("PASS" if _failures == 0 else "FAIL"))
	quit(1 if _failures > 0 else 0)


func _setup_scene() -> void:
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	_viewport.use_hdr_2d = true
	_viewport.own_world_3d = true
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(_viewport)
	var sky_material := ShaderMaterial.new()
	sky_material.shader = load("res://shaders/sky.gdshader")
	var sky := Sky.new()
	sky.sky_material = sky_material
	sky.process_mode = Sky.PROCESS_MODE_REALTIME
	var environment := Environment.new()
	environment.background_mode = Environment.BG_SKY
	environment.sky = sky
	environment.tonemap_mode = Environment.TONE_MAPPER_LINEAR
	environment.tonemap_exposure = 1.0
	environment.glow_enabled = false
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_DISABLED
	environment.reflected_light_source = Environment.REFLECTION_SOURCE_DISABLED
	var world_environment := WorldEnvironment.new()
	world_environment.environment = environment
	_viewport.add_child(world_environment)
	_camera.fov = 10.0
	_camera.near = 0.5
	_camera.far = 4.0e7  # the scaled-space Earth sits at CelestialBody.SCALED_SPACE_DISTANCE_M
	_camera.current = true
	_viewport.add_child(_camera)
	_setup_surfaces()


## The Earth's surfaces of Task 6, hidden until a surface sample shows one: the ellipsoid mesh (CelestialBody with
## earth.gdshader) and a terrain quad (terrain_earth.gdshader), both with the constant albedo texture and black night
## lights, materials set up as world.gd does.
func _setup_surfaces() -> void:
	var albedo_texture := _constant_texture(SURFACE_ALBEDO_8)
	var black := _constant_texture(Color(0, 0, 0))
	_earth_material.shader = load("res://shaders/earth.gdshader")
	_earth_material.set_shader_parameter("day_texture", albedo_texture)
	_earth_material.set_shader_parameter("night_texture", black)
	_earth_material.set_shader_parameter("moon_radius_m", 1737400.0)
	_earth_material.set_shader_parameter("day_exposure", Atmosphere.DAY_EXPOSURE)
	_earth_body.setup(WGS84_A_M, WGS84_FLATTENING, _earth_material)
	_earth_body.visible = false
	_viewport.add_child(_earth_body)
	_terrain_material.shader = load("res://shaders/terrain_earth.gdshader")
	_terrain_material.set_shader_parameter("day_texture", albedo_texture)
	_terrain_material.set_shader_parameter("day_texture_bound", true)
	_terrain_material.set_shader_parameter("night_texture", black)
	_terrain_material.set_shader_parameter("moon_radius_m", 1737400.0)
	_terrain_material.set_shader_parameter("day_exposure", Atmosphere.DAY_EXPOSURE)
	_terrain_quad.material_override = _terrain_material
	_terrain_parent.add_child(_terrain_quad)
	_terrain_parent.visible = false
	_viewport.add_child(_terrain_parent)


static func _constant_texture(color: Color) -> ImageTexture:
	var image := Image.create(4, 4, true, Image.FORMAT_RGBA8)
	image.fill(color)
	image.generate_mipmaps()
	return ImageTexture.create_from_image(image)


## A surface pixel: the terrain quad (terrain = true) centred on point_itrf_m and facing the camera, its NORMAL the
## surface normal normal_itrf (the lighting's normal; the quad's geometry only puts a fragment on the camera axis), or
## the ellipsoid mesh (the point is the ray's WGS84 hit). Sun far, the Moon behind the Earth (no eclipse).
## CPU: albedo x (sun + sky lux on the normal) / pi x T + to-point luminance, all x luminance_scale.
func _surface_sample(label: String, camera_itrf_m: Vector3, point_itrf_m: Vector3, normal_itrf: Vector3,
		sun_itrf: Vector3, terrain: bool) -> void:
	var view_itrf := (point_itrf_m - camera_itrf_m).normalized()
	var camera_gcrf := _earth_to_gcrf * camera_itrf_m
	var sun_local := Frames.to_render(sun_itrf * AU_M)
	var moon_local := -Frames.to_render(sun_itrf) * MOON_DISTANCE_M
	RenderingServer.global_shader_parameter_set(&"earth_sun_position_local", sun_local)
	RenderingServer.global_shader_parameter_set(&"earth_moon_position_local", moon_local)
	_earth_material.set_shader_parameter("sun_position_local", sun_local)
	_earth_material.set_shader_parameter("moon_position_local", moon_local)
	_earth_body.visible = not terrain
	_terrain_parent.visible = terrain
	if terrain:
		var center := Frames.to_render(point_itrf_m)
		var distance := (point_itrf_m - camera_itrf_m).length()
		_terrain_parent.transform = Transform3D(Frames.basis_to_render(_earth_to_gcrf), Frames.to_render(-camera_gcrf))
		_terrain_quad.mesh = _facing_quad(Frames.to_render(view_itrf), Frames.to_render(normal_itrf), distance * 0.01)
		_terrain_quad.position = center
		var split := TerrainQuadtree.split_float32(center)
		_terrain_quad.set_instance_shader_parameter(&"vertex_offset", split[0])
		_terrain_quad.set_instance_shader_parameter(&"vertex_offset_lo", split[1])
		var camera_split := TerrainQuadtree.split_float32(Frames.to_render(camera_itrf_m))
		RenderingServer.global_shader_parameter_set(&"earth_terrain_camera_hi", camera_split[0])
		RenderingServer.global_shader_parameter_set(&"earth_terrain_camera_lo", camera_split[1])
	else:
		_earth_body.fixed_to_gcrf = _earth_to_gcrf
		_earth_body.center_gcrf_m = Vector3.ZERO
		_earth_body.place(camera_gcrf)
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	_earth_body.visible = false
	_terrain_parent.visible = false
	var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(camera_itrf_m, point_itrf_m, sun_itrf)
	var lux: Dictionary = _core.atmosphere_illuminance(point_itrf_m, normal_itrf, sun_itrf)
	var linear := SURFACE_ALBEDO_8.srgb_to_linear()
	var albedo := Vector3(linear.r, linear.g, linear.b)
	var transmittance: Vector3 = to_point["transmittance"]
	var light: Vector3 = (lux["sun_lux"] + lux["sky_lux"]) / PI
	var expected: Vector3 = (albedo * light * transmittance + to_point["luminance"]) * _atmosphere.luminance_scale
	if OS.get_environment("STARSIM_VERBOSE") == "1":
		print("atmosphere_check:   surface %s x T %s + in-scatter %s" % [_vec(albedo * light * _atmosphere.luminance_scale),
				_vec(transmittance), _vec(to_point["luminance"] * _atmosphere.luminance_scale)])
		# Sensitivity to the drawn surface's height: the ellipsoid mesh's facets lie up to ~0.5 km below the ellipsoid.
		var lower := point_itrf_m - normal_itrf * 300.0
		var lux_low: Dictionary = _core.atmosphere_illuminance(lower, normal_itrf, sun_itrf)
		var to_low: Dictionary = _core.atmosphere_sky_luminance_to_point(camera_itrf_m, lower, sun_itrf)
		var t_low: Vector3 = to_low["transmittance"]
		print("atmosphere_check:   the same 300 m lower: %s" % _vec((albedo * (lux_low["sun_lux"] + lux_low["sky_lux"]) / PI
				* t_low + to_low["luminance"]) * _atmosphere.luminance_scale))
	_compare(label + " [%.0f km]" % ((point_itrf_m - camera_itrf_m).length() / 1000.0), expected, got,
			SURFACE_TOLERANCE)


## A square of half size `half_m` perpendicular to `view` (centred on the origin), every vertex with the normal `normal`.
static func _facing_quad(view: Vector3, normal: Vector3, half_m: float) -> ArrayMesh:
	var u := view.cross(Vector3.UP if absf(view.y) < 0.9 else Vector3.RIGHT).normalized() * half_m
	var w := view.cross(u).normalized() * half_m
	var vertices := PackedVector3Array([-u - w, u - w, u + w, -u - w, u + w, -u + w])
	var normals := PackedVector3Array()
	for i in range(vertices.size()):
		normals.append(normal)
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh


## The ray's first hit on the WGS84 ellipsoid (float64, in the polar-stretched space where it is a sphere of radius a).
static func _ellipsoid_hit(camera_itrf_m: Vector3, view_itrf: Vector3) -> Vector3:
	var stretch := Atmosphere.POLAR_STRETCH
	var p := Vector3(camera_itrf_m.x, camera_itrf_m.y, camera_itrf_m.z * stretch)
	var v := Vector3(view_itrf.x, view_itrf.y, view_itrf.z * stretch)
	var v_length := v.length()
	v /= v_length
	var along := p.dot(v)
	var b2 := p.length_squared() - along * along
	var t := -along - sqrt(WGS84_A_M * WGS84_A_M - b2)
	var hit := p + v * t
	return Vector3(hit.x, hit.y, hit.z / stretch)


## Geodetic up of a point on (or near) the WGS84 ellipsoid: the gradient (x / a^2, y / a^2, z / b^2).
static func _geodetic_up(point_itrf_m: Vector3) -> Vector3:
	var stretch := Atmosphere.POLAR_STRETCH
	return Vector3(point_itrf_m.x, point_itrf_m.y, point_itrf_m.z * stretch * stretch).normalized()


## Renders the sky for the camera (ITRF m) along view_itrf with the Sun along sun_itrf and compares the centre pixel.
func _sample(label: String, camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3) -> void:
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	var cpu: Dictionary = _core.atmosphere_sky_luminance(camera_itrf_m, view_itrf, sun_itrf)
	var expected: Vector3 = cpu["luminance"] * _atmosphere.luminance_scale
	_compare(label, expected, got)


## Counts and prints one sample: every channel within TOLERANCE (relative; absolute below the floor).
func _compare(label: String, expected: Vector3, got: Vector3, tolerance := TOLERANCE) -> void:
	_samples += 1
	var peak := maxf(expected.x, maxf(expected.y, expected.z))
	var ok := peak > 0.0
	var ratios := PackedStringArray()
	for k in range(3):
		var e := expected[k]
		var g := got[k]
		var ratio := g / e if e > 0.0 else INF
		ratios.append("%.4f" % ratio)
		var floor_value := peak * RELATIVE_FLOOR
		if e >= floor_value:
			ok = ok and absf(ratio - 1.0) <= tolerance
		else:
			ok = ok and absf(g - e) <= tolerance * floor_value
	if not ok:
		_failures += 1
	print("atmosphere_check: %-38s expected %s got %s ratio (%s) %s" % [label, _vec(expected), _vec(got),
			", ".join(ratios), "ok" if ok else "FAIL"])


## A ground ray: the sky shader shows the model ground (albedo GROUND_ALBEDO without a day texture) lit by the Sun and
## the sky, seen through the atmosphere. CPU: the model-ground hit in the stretched space (float64), then
## albedo x (sun + sky lux) / pi x T + in-scattered luminance, all x luminance_scale.
const GROUND_ALBEDO := 0.1  ## sky.gdshader's ground_albedo default
func _ground_sample(label: String, camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3) -> void:
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	var stretch := Atmosphere.POLAR_STRETCH
	var p := Vector3(camera_itrf_m.x, camera_itrf_m.y, camera_itrf_m.z * stretch)
	var v := Vector3(view_itrf.x, view_itrf.y, view_itrf.z * stretch).normalized()
	var radius := float(_atmosphere.parameters["bottom_radius_m"])
	var along := p.dot(v)
	var t := -along - sqrt(along * along - (p.length_squared() - radius * radius))
	var hit := p + v * t
	var point_itrf := Vector3(hit.x, hit.y, hit.z / stretch)
	var normal_itrf := Vector3(hit.x, hit.y, hit.z * stretch).normalized()
	var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(camera_itrf_m, point_itrf, sun_itrf)
	var lux: Dictionary = _core.atmosphere_illuminance(point_itrf, normal_itrf, sun_itrf)
	var transmittance: Vector3 = to_point["transmittance"]
	var light: Vector3 = (lux["sun_lux"] + lux["sky_lux"]) * (GROUND_ALBEDO / PI)
	var expected: Vector3 = (light * transmittance + to_point["luminance"]) * _atmosphere.luminance_scale
	_compare(label + " [ground, %.0f km]" % (t / 1000.0), expected, got)


## Review Focus 1 control: the grazing sample with the camera altitude uniform overridden to 0 (the model ground),
## the float32 position unchanged. The shader must follow the uniform: the forced render matches the CPU for a camera
## on the model ground (2 %), and it moved away from the true render by more than CONTROL_MIN_CHANGE (the true sample
## agrees with the CPU to ~0.1 %, so a change of that size is the uniform's doing, not noise).
const CONTROL_MIN_CHANGE := 0.005
func _altitude_control(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3) -> void:
	var true_value := await _render(camera_itrf_m, view_itrf, sun_itrf)
	var true_altitude := _atmosphere.camera_altitude_m
	var forced := await _render(camera_itrf_m, view_itrf, sun_itrf, 0.0)
	var ground_camera := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, PAD_HEIGHT_M - true_altitude)
	var cpu: Dictionary = _core.atmosphere_sky_luminance(ground_camera, view_itrf, sun_itrf)
	var expected: Vector3 = cpu["luminance"] * _atmosphere.luminance_scale
	var change := absf(forced.y / true_value.y - 1.0) if true_value.y > 0.0 else 0.0
	var match_ratio := forced.y / expected.y if expected.y > 0.0 else INF
	var ok := change > CONTROL_MIN_CHANGE and absf(match_ratio - 1.0) <= TOLERANCE
	_samples += 1
	if not ok:
		_failures += 1
	print(("atmosphere_check: altitude control (uniform forced %.1f -> 0 m): green %s -> %s (change %.2f %%, needs > "
			+ "%.1f %%), CPU on the model ground %s, ratio %.4f %s") % [true_altitude, _num(true_value.y), _num(forced.y),
			change * 100.0, CONTROL_MIN_CHANGE * 100.0, _num(expected.y), match_ratio, "ok" if ok else "FAIL"])


func _render(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, altitude_override := NAN) -> Vector3:
	var camera_gcrf := _earth_to_gcrf * camera_itrf_m
	_atmosphere.update(camera_gcrf, Vector3.ZERO, _earth_to_gcrf, _earth_to_gcrf * sun_itrf * AU_M)
	if not is_nan(altitude_override):
		RenderingServer.global_shader_parameter_set(&"atmosphere_camera_altitude_m", altitude_override)
	var view_render := Frames.to_render(_earth_to_gcrf * view_itrf).normalized()
	var up := Vector3.UP if absf(view_render.y) < 0.9 else Vector3.RIGHT
	_camera.transform = Transform3D(Basis.looking_at(view_render, up), Vector3.ZERO)
	for i in range(SETTLE_FRAMES):
		await RenderingServer.frame_post_draw
	var image := _viewport.get_texture().get_image()
	var c := image.get_pixel(VIEW_SIZE / 2, VIEW_SIZE / 2)
	return Vector3(c.r, c.g, c.b)


static func _geodetic(lat_deg: float, lon_deg: float, height_m: float) -> Vector3:
	return HeightSource.geodetic_to_fixed(lat_deg, lon_deg, height_m)


## ITRF unit vector at azimuth (from north, east positive) and elevation above the geodetic horizon.
static func _local_direction(lat_deg: float, lon_deg: float, azimuth_deg: float, elevation_deg: float) -> Vector3:
	var lat := deg_to_rad(lat_deg)
	var lon := deg_to_rad(lon_deg)
	var up := Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
	var east := Vector3(-sin(lon), cos(lon), 0.0)
	var north := Vector3(-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat))
	var az := deg_to_rad(azimuth_deg)
	var el := deg_to_rad(elevation_deg)
	return (up * sin(el) + (north * cos(az) + east * sin(az)) * cos(el)).normalized()


## The view direction from `camera` grazing the sphere of radius a + tangent_height on the `toward` side (a sphere
## of the equatorial radius: the CPU and the GPU see the same ray, its exact tangent height does not matter).
static func _limb_direction(camera: Vector3, toward: Vector3, tangent_height_m: float) -> Vector3:
	var c := camera.normalized()
	var e := (toward - c * toward.dot(c)).normalized()
	var d := camera.length()
	var radius := 6378137.0 + tangent_height_m
	var tangent := c * (radius * radius / d) + e * (radius * sqrt(1.0 - (radius / d) * (radius / d)))
	return (tangent - camera).normalized()


func _fail_and_quit(message: String) -> void:
	print("atmosphere_check: FAIL - " + message)
	quit(1)


## Five significant digits in scientific form (GDScript's % has no %g).
static func _num(x: float) -> String:
	if x == 0.0 or not is_finite(x):
		return str(x)
	var exponent := floori(log(absf(x)) / log(10.0))
	return "%.4fe%d" % [x / pow(10.0, exponent), exponent]


static func _vec(v: Vector3) -> String:
	return "(%s, %s, %s)" % [_num(v.x), _num(v.y), _num(v.z)]
