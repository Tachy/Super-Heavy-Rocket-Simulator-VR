class_name GroundPatch
extends MeshInstance3D
## Fine ellipsoid patch under the camera, child of the Earth body. The global Earth mesh (1.4 deg facets)
## lies up to ~0.6 km below the true surface; this patch follows the WGS84 ellipsoid to ~6 cm so vehicles
## stand on the visible ground. Interim solution until streamed terrain tiles (M8).
## Vertices are stored relative to the patch centre (small float32 values, cm precision).

const SPAN_DEG := 2.0
const CELLS := 160
const RECENTER_DEG := 0.4
const MAX_ALTITUDE_M := 500000.0
const A := 6378137.0
const F := 1.0 / 298.257223563

var _center_lat := INF
var _center_lon := INF
var _height_m := 0.0
## Launch sites ({latitude_deg, longitude_deg, height_m}): near one, the patch is drawn at the site's
## height so that the visible ground matches the physical ground (flat at pad height until terrain, M8).
var sites: Array = []


func setup(shader_material: ShaderMaterial) -> void:
	material_override = shader_material
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF


## camera_fixed_m: camera position in Earth-fixed coordinates (ITRF).
func update_for_camera(camera_fixed_m: Vector3) -> void:
	var lat := rad_to_deg(atan2(camera_fixed_m.z, Vector2(camera_fixed_m.x, camera_fixed_m.y).length()))
	var lon := rad_to_deg(atan2(camera_fixed_m.y, camera_fixed_m.x))
	visible = camera_fixed_m.length() - A < MAX_ALTITUDE_M
	if not visible:
		return
	if absf(lat - _center_lat) < RECENTER_DEG and absf(wrapf(lon - _center_lon, -180.0, 180.0)) < RECENTER_DEG:
		return
	_center_lat = clampf(roundf(lat / RECENTER_DEG) * RECENTER_DEG, -89.0, 89.0)
	_center_lon = roundf(lon / RECENTER_DEG) * RECENTER_DEG
	_height_m = 0.0
	for site: Dictionary in sites:
		if absf(site["latitude_deg"] - lat) < SPAN_DEG and absf(wrapf(site["longitude_deg"] - lon, -180.0, 180.0)) < SPAN_DEG:
			_height_m = site["height_m"]
	_build()


func _build() -> void:
	var center_fixed := geodetic_to_fixed(_center_lat, _center_lon, _height_m)
	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()
	var uvs := PackedVector2Array()
	var indices := PackedInt32Array()
	var half := SPAN_DEG * 0.5
	var lon_span := SPAN_DEG / maxf(cos(deg_to_rad(_center_lat)), 0.2)  # keep cells roughly square
	for i in range(CELLS + 1):
		var lat := clampf(_center_lat + half - SPAN_DEG * i / CELLS, -89.9, 89.9)
		for j in range(CELLS + 1):
			var lon := _center_lon - lon_span * 0.5 + lon_span * j / CELLS
			vertices.append(Frames.to_render(geodetic_to_fixed(lat, lon, _height_m) - center_fixed))
			var la := deg_to_rad(lat)
			var lo := deg_to_rad(lon)
			normals.append(Frames.to_render(Vector3(cos(la) * cos(lo), cos(la) * sin(lo), sin(la))))
			uvs.append(Vector2(fposmod(lon + 180.0, 360.0) / 360.0, (90.0 - lat) / 180.0))
	var row := CELLS + 1
	for i in range(CELLS):
		for j in range(CELLS):
			var k := i * row + j
			indices.append_array([k, k + 1, k + row, k + 1, k + row + 1, k + row])
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_INDEX] = indices
	var patch_mesh := ArrayMesh.new()
	patch_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	mesh = patch_mesh
	position = Frames.to_render(center_fixed)
	set_instance_shader_parameter("vertex_offset", position)


static func geodetic_to_fixed(lat_deg: float, lon_deg: float, height_m := 0.0) -> Vector3:
	var e2 := F * (2.0 - F)
	var la := deg_to_rad(lat_deg)
	var lo := deg_to_rad(lon_deg)
	var n := A / sqrt(1.0 - e2 * sin(la) * sin(la))
	return Vector3((n + height_m) * cos(la) * cos(lo), (n + height_m) * cos(la) * sin(lo),
			(n * (1.0 - e2) + height_m) * sin(la))
