class_name Atmosphere
extends RefCounted
## The Earth's precomputed atmosphere on the GPU (M8 atmosphere): loads the core's tables (StarsimCore.load_atmosphere:
## the cache in data/atmosphere, or a first-run precomputation) on a WorkerThreadPool task, uploads them as float
## textures, sets the global shader uniforms of shaders/atmosphere.gdshaderinc and updates the per-frame ones.
## No physics here: the tables and every evaluation come from the core; this only converts frames and units.
##
## Scene units (one exposure, spec "One exposure for everything"): scene value = luminance (cd/m^2) x DAY_EXPOSURE x
## pi / E_v,TOA x 1 / d_AU^2 - a white Lambertian surface under the top-of-atmosphere Sun at 1 AU renders as
## DAY_EXPOSURE, as the Earth shaders' day lighting does.

## Scene value of a white Lambertian surface under the full TOA Sun, the one exposure of the game [call: M8 atmosphere
## spec, "stays 1.5"]. The `day_exposure` uniforms of shaders/earth.gdshader and terrain_earth.gdshader are set from
## it (world.gd); they have no default of their own.
const DAY_EXPOSURE := 1.5
## Astronomical unit (m) [S: IAU 2012 Resolution B2]; the tables are for the Sun at 1 AU.
const ASTRONOMICAL_UNIT_M := 149597870700.0
## a / b of WGS84 (1 / (1 - f), f = 1 / 298.257223563) [S: NIMA TR8350.2]: the model space's polar stretch, as the core.
const POLAR_STRETCH := 1.0 / (1.0 - 1.0 / 298.257223563)
const CACHE_FILE := "earth_atmosphere.bin"

## True once the textures are uploaded and atmosphere_enabled is set.
var ready := false
## A table load is running on a worker thread.
var loading := false
## German status line for the UI while the tables load or compute ("" when idle); the error text after a failure.
var status_text := ""
## AtmosphereParameters of the loaded tables (StarsimCore.load_atmosphere "parameters").
var parameters := {}
## GPU format of the tables, chosen before start_loading. Half float [call, Task 5 fix round 1]: half the VRAM (42 vs 85
## MB) and fetch bandwidth; relative rounding ~5e-4, the GPU check stays within its 2 % (numbers in the Task 5
## report); RGBAF for comparisons (atmosphere_check.gd --f32). The transmittance table is always RGBAF [call, Task 6]:
## its grazing values from the ground (T to the top ~1e-5 and below) fall into the half-float subnormal range, and the
## aerial perspective divides two of them (a terrain pixel 20 km from the Cape camera was 4 % off in blue with RGBAH,
## 0.05 % with RGBAF); +8 MB.
var table_format := Image.FORMAT_RGBAH
## Per-frame values (readable by tests): table luminance (cd/m^2 for the Sun at 1 AU) -> scene value (includes the
## 1 / d_AU^2 of the true Sun distance: do not apply it to a true luminance), camera height above the model ground (stretched
## space), the Sun's Earth-local direction and the camera's Earth-local position.
var luminance_scale := 0.0
var camera_altitude_m := 0.0
var sun_direction_local := Vector3.ZERO
var camera_local_m := Vector3.ZERO

var _core: StarsimCore
var _task_id := -1
var _result := {}  ## written by the worker task, read after it completed
var _textures: Array[Texture] = []  ## keeps the uploaded textures alive


## Starts loading the tables in the background (config_dir/atmosphere/earth_optics.toml, cache in cache_dir).
func start_loading(core: StarsimCore, config_dir: String, cache_dir: String) -> void:
	if loading or ready:
		return
	_core = core
	RenderingServer.global_shader_parameter_set(&"atmosphere_enabled", false)
	var cached := FileAccess.file_exists(cache_dir.path_join(CACHE_FILE))
	status_text = ("Atmosphäre wird geladen ..." if cached
			else "Atmosphäre wird berechnet (einmalig, etwa eine Minute) ...")
	loading = true
	_task_id = WorkerThreadPool.add_task(_load_task.bind(config_dir, cache_dir), false, "atmosphere tables")


## Main thread: uploads the tables once the worker finished. Returns true when the atmosphere is ready.
func poll() -> bool:
	if loading and WorkerThreadPool.is_task_completed(_task_id):
		WorkerThreadPool.wait_for_task_completion(_task_id)
		loading = false
		_upload(_result)
		_result = {}
	return ready


## Blocks until a running load finished and uploads it (tests).
func wait_until_loaded() -> bool:
	if loading:
		WorkerThreadPool.wait_for_task_completion(_task_id)
		loading = false
		_upload(_result)
		_result = {}
	return ready


## Per frame: the camera, the Earth's centre and orientation (fixed -> GCRF) and the apparent Sun position (GCRF, as
## seen from the Earth's centre), all float64. The camera is differenced against the Earth's centre here, in float64.
func update(camera_gcrf_m: Vector3, earth_center_gcrf_m: Vector3, earth_fixed_to_gcrf: Basis,
		sun_apparent_gcrf_m: Vector3) -> void:
	if not poll():
		return
	var gcrf_to_fixed := earth_fixed_to_gcrf.transposed()
	var camera_fixed := gcrf_to_fixed * (camera_gcrf_m - earth_center_gcrf_m)
	camera_local_m = Frames.to_render(camera_fixed)
	var split := TerrainQuadtree.split_float32(camera_local_m)
	var stretched := Vector3(camera_fixed.x, camera_fixed.y, camera_fixed.z * POLAR_STRETCH)
	camera_altitude_m = stretched.length() - float(parameters["bottom_radius_m"])
	var sun_from_earth := sun_apparent_gcrf_m - earth_center_gcrf_m
	var d_au := sun_from_earth.length() / ASTRONOMICAL_UNIT_M
	sun_direction_local = Frames.to_render(gcrf_to_fixed * sun_from_earth).normalized()
	luminance_scale = DAY_EXPOSURE * PI / float(parameters["toa_solar_illuminance_lux"]) / (d_au * d_au)
	# Render world axes are GCRF in render axes; Earth-local = P R^T P^T (the transpose of the body's render basis).
	var world_to_earth := Frames.basis_to_render(earth_fixed_to_gcrf).transposed()
	RenderingServer.global_shader_parameter_set(&"atmosphere_camera_hi", split[0])
	RenderingServer.global_shader_parameter_set(&"atmosphere_camera_lo", split[1])
	RenderingServer.global_shader_parameter_set(&"atmosphere_camera_altitude_m", camera_altitude_m)
	RenderingServer.global_shader_parameter_set(&"atmosphere_world_to_earth", world_to_earth)
	RenderingServer.global_shader_parameter_set(&"atmosphere_sun_direction", sun_direction_local)
	RenderingServer.global_shader_parameter_set(&"atmosphere_luminance_scale", luminance_scale)


## The Sun light of Godot's lit materials (vehicles, structures: world.gd's DirectionalLight3D) at an Earth-fixed point:
## [colour, energy]. Per channel it is the scene value atm_sun_at gives the Earth's surfaces (one exposure): the TOA
## solar illuminance of the tables (its own colour in the RGB luminance space) x the transmittance to the Sun
## (StarsimCore.atmosphere_sun_transmittance: red and dim at sunset, zero below the horizon or in the Earth's shadow) x
## luminance_scale / pi x visible_fraction (the unocculted part of the solar disc, Moon) - a white Lambertian at normal
## incidence renders it; in luminance that is DAY_EXPOSURE / d_AU^2 x T under the TOA Sun. Godot multiplies a
## directional light's energy by pi and its Lambert BRDF divides by pi, so energy = the brightest channel's value and
## colour = the channels over it, sRGB-encoded (Godot linearises light colours). Before the tables are loaded: the
## vacuum Sun (white, DAY_EXPOSURE).
func sun_light_at(position_itrf_m: Vector3, sun_direction_itrf: Vector3, visible_fraction: float) -> Array:
	var scene := Vector3.ONE * DAY_EXPOSURE
	if ready:
		var transmittance := _core.atmosphere_sun_transmittance(position_itrf_m, sun_direction_itrf)
		var solar: Vector3 = parameters["solar_illuminance_rgb"]
		var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
		scene = solar * sun_factor * transmittance * (luminance_scale / PI)
	scene *= visible_fraction
	var peak := maxf(scene.x, maxf(scene.y, scene.z))
	var color := Color.WHITE
	if peak > 0.0:
		color = Color(scene.x / peak, scene.y / peak, scene.z / peak).linear_to_srgb()
	return [color, peak]


func _load_task(config_dir: String, cache_dir: String) -> void:
	var result: Dictionary = _core.load_atmosphere(config_dir, cache_dir)
	if result.get("ok", false):
		# Images are built here (the large copies), textures on the main thread (_upload).
		var t_size: Vector2i = result["transmittance_size"]
		var s_size: Vector3i = result["scattering_size"]
		var i_size: Vector2i = result["irradiance_size"]
		result["transmittance_image"] = _image_2d(result["transmittance"], t_size, Image.FORMAT_RGBAF)
		result["irradiance_image"] = _image_2d(result["irradiance"], i_size)
		result["scattering_images"] = _images_3d(result["scattering"], s_size)
		result["single_mie_images"] = _images_3d(result["single_mie"], s_size)
		for key in ["transmittance", "irradiance", "scattering", "single_mie"]:
			result.erase(key)
	_result = result


func _image_2d(data: PackedFloat32Array, size: Vector2i, format := table_format) -> Image:
	var image := Image.create_from_data(size.x, size.y, false, Image.FORMAT_RGBAF, data.to_byte_array())
	if format != Image.FORMAT_RGBAF:
		image.convert(format)
	return image


## One image per depth slice (z = r), x fastest within a slice (the core's layout).
func _images_3d(data: PackedFloat32Array, size: Vector3i) -> Array[Image]:
	var bytes := data.to_byte_array()
	var slice_bytes := size.x * size.y * 4 * 4
	var images: Array[Image] = []
	for z in range(size.z):
		var image := Image.create_from_data(size.x, size.y, false, Image.FORMAT_RGBAF,
				bytes.slice(z * slice_bytes, (z + 1) * slice_bytes))
		if table_format != Image.FORMAT_RGBAF:
			image.convert(table_format)
		images.append(image)
	return images


func _upload(result: Dictionary) -> void:
	if not result.get("ok", false):
		status_text = str(result.get("error", "Atmosphäre konnte nicht geladen werden"))
		push_error("Atmosphere: " + status_text)
		return
	parameters = result["parameters"]
	var s_size: Vector3i = result["scattering_size"]
	var transmittance := ImageTexture.create_from_image(result["transmittance_image"])
	var irradiance := ImageTexture.create_from_image(result["irradiance_image"])
	var scattering := ImageTexture3D.new()
	scattering.create(table_format, s_size.x, s_size.y, s_size.z, false, result["scattering_images"])
	var single_mie := ImageTexture3D.new()
	single_mie.create(table_format, s_size.x, s_size.y, s_size.z, false, result["single_mie_images"])
	_textures = [transmittance, irradiance, scattering, single_mie]
	var t_size: Vector2i = result["transmittance_size"]
	var i_size: Vector2i = result["irradiance_size"]
	var cells: Vector4i = result["scattering_cells"]  # (nu, mu_s, mu, r)
	var rs := RenderingServer
	rs.global_shader_parameter_set(&"atmosphere_transmittance", transmittance)
	rs.global_shader_parameter_set(&"atmosphere_irradiance", irradiance)
	rs.global_shader_parameter_set(&"atmosphere_scattering", scattering)
	rs.global_shader_parameter_set(&"atmosphere_single_mie", single_mie)
	rs.global_shader_parameter_set(&"atmosphere_bottom_radius_m", float(parameters["bottom_radius_m"]))
	rs.global_shader_parameter_set(&"atmosphere_top_radius_m", float(parameters["top_radius_m"]))
	rs.global_shader_parameter_set(&"atmosphere_mu_s_min", float(parameters["mu_s_min"]))
	rs.global_shader_parameter_set(&"atmosphere_mie_phase_g", float(parameters["mie_phase_g"]))
	rs.global_shader_parameter_set(&"atmosphere_sun_angular_radius_rad", float(parameters["sun_angular_radius_rad"]))
	rs.global_shader_parameter_set(&"atmosphere_sky_radiance_to_luminance", parameters["sky_radiance_to_luminance"])
	var solar: Vector3 = parameters["solar_illuminance_rgb"]
	var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
	rs.global_shader_parameter_set(&"atmosphere_sun_illuminance_lux", solar * sun_factor)
	# A true luminance (cd/m^2, at the Sun's actual distance) -> scene value: luminance_scale without the 1 / d_AU^2
	# (that factor converts the tables' 1 AU values); the star threshold reads the sky's true luminance back with it.
	rs.global_shader_parameter_set(&"atmosphere_scene_per_cd_m2",
			DAY_EXPOSURE * PI / float(parameters["toa_solar_illuminance_lux"]))
	rs.global_shader_parameter_set(&"atmosphere_scattering_size", Vector4(cells.x, cells.y, cells.z, cells.w))
	rs.global_shader_parameter_set(&"atmosphere_table_sizes", Vector4(t_size.x, t_size.y, i_size.x, i_size.y))
	rs.global_shader_parameter_set(&"atmosphere_enabled", true)
	ready = true
	status_text = ""
