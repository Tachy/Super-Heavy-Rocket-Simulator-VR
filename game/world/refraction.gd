class_name Refraction
extends RefCounted
## Astronomical refraction for the camera: the core's lift table (StarsimCore.atmospheric_refraction, ~1 ms) as the
## global shader uniforms of refraction.gdshaderinc (Sun disc, Moon, stars drawn at their refracted directions).
##
## The table depends on the camera's height only: it is rebuilt when the distance from the Earth's centre has changed
## by more than REBUILD_FRACTION of the height (at least REBUILD_MIN_M). The vertical follows every frame: the normal
## of the WGS84 ellipsoid scaled through the camera (within ~f h / a of the geodetic vertical: < 0.01 deg below 400 km).

const REBUILD_MIN_M := 5.0
const REBUILD_FRACTION := 0.005
const EARTH_POLAR_RADIUS_M := 6356752.3142  ## WGS84 (a lower bound of the height for the rebuild distance)
const EARTH_SEMI_MAJOR_AXIS_M := 6378137.0  ## WGS84

var enabled := true  ## false draws the geometric directions (comparison)

var _texture: ImageTexture = null
var _built_at_radius_m := INF
var _grid := Vector4.ZERO


## camera_fixed_m: the camera in ITRF metres; fixed_to_gcrf: the Earth's orientation (CelestialBody.fixed_to_gcrf).
func update(core: StarsimCore, camera_fixed_m: Vector3, fixed_to_gcrf: Basis) -> void:
	var radius_m := camera_fixed_m.length()
	var height_m := maxf(radius_m - EARTH_POLAR_RADIUS_M, 0.0)
	if absf(radius_m - _built_at_radius_m) > maxf(REBUILD_MIN_M, REBUILD_FRACTION * height_m):
		var table: Dictionary = core.atmospheric_refraction(camera_fixed_m)
		if not table.is_empty():
			_built_at_radius_m = radius_m
			var lift: PackedFloat32Array = table["lift_rad"]
			_grid = Vector4.ZERO  # empty table: no refraction (a distant camera)
			if lift.size() >= 2:
				_upload(lift)
				_grid = Vector4(table["min_rad"], table["span_rad"], lift.size(), 1.0)
	var grid := _grid
	grid.w = 1.0 if enabled and _grid.w > 0.0 else 0.0
	var ratio := EARTH_SEMI_MAJOR_AXIS_M / EARTH_POLAR_RADIUS_M
	var up_fixed := Vector3(camera_fixed_m.x, camera_fixed_m.y, camera_fixed_m.z * ratio * ratio).normalized()
	RenderingServer.global_shader_parameter_set(&"refraction_grid", grid)
	RenderingServer.global_shader_parameter_set(&"refraction_up", Frames.to_render(fixed_to_gcrf * up_fixed))


func _upload(lift: PackedFloat32Array) -> void:
	var image := Image.create_from_data(lift.size(), 1, false, Image.FORMAT_RF, lift.to_byte_array())
	if _texture == null or _texture.get_width() != image.get_width():
		_texture = ImageTexture.create_from_image(image)
		RenderingServer.global_shader_parameter_set(&"refraction_lift", _texture)
	else:
		_texture.update(image)
