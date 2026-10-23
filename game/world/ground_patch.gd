class_name GroundPatch
extends MeshInstance3D
## Fine ellipsoid patch under the camera, child of the Earth body. The global Earth mesh (1.4 deg facets)
## lies up to ~0.6 km below the true surface; this patch follows the WGS84 ellipsoid to ~6 cm so vehicles
## stand on the visible ground. With the Earth terrain loaded (FlightSim.earth_surface) the vertices follow the physical
## surface (relief, pad plateaus, the sea on the EGM2008 geoid), with extra grid lines across each site's plateau and
## apron so the pad is drawn where the physics has it; without terrain the old flat rule (ellipsoid, or a launch site's
## height near one).
## Vertices are stored relative to the patch centre (small float32 values, cm precision).

const SPAN_DEG := 2.0
const CELLS := 160
const RECENTER_DEG := 0.4
const MAX_ALTITUDE_M := 500000.0
const A := 6378137.0
const F := 1.0 / 298.257223563
const METRES_PER_DEG := A * PI / 180.0
## [E] Grid lines across a site with terrain: every SITE_STEP_M out to its plateau + apron + SITE_MARGIN_M.
const SITE_STEP_M := 20.0
const SITE_MARGIN_M := 100.0
const DEFAULT_PLATEAU_M := 150.0  # a site without plateau_radius_m / plateau_apron_m in launch_sites.toml
const DEFAULT_APRON_M := 50.0

var _center_lat := INF
var _center_lon := INF
var _height_m := 0.0
## Launch sites ({latitude_deg, longitude_deg, height_m[, plateau_radius_m, plateau_apron_m]}): near one, the flat
## rule draws the patch at the site's height so that the visible ground matches the physical ground.
var sites: Array = []
## The flight simulation (FlightSim) whose Earth terrain the vertices follow; null or no terrain: the flat rule.
var flight_sim: Object = null


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


## The surface height above the ellipsoid at a vertex: the Earth terrain when loaded, else the flat rule's height.
func _vertex_height_m(lat_deg: float, lon_deg: float, terrain: bool) -> float:
	if terrain:
		return flight_sim.earth_surface(lat_deg, lon_deg)["height_m"]
	return _height_m


## Grid lines from `low` to `high` (CELLS uniform cells), plus with terrain the refined lines across every site inside
## (`coordinate` "latitude_deg" / "longitude_deg", `deg_per_m` the line spacing per metre there). Sorted ascending.
func _lines(low: float, high: float, terrain: bool, coordinate: String, deg_per_m: float) -> PackedFloat64Array:
	var lines := PackedFloat64Array()
	for k in range(CELLS + 1):
		lines.append(low + (high - low) * k / CELLS)
	if terrain:
		for site: Dictionary in sites:
			var c: float = site[coordinate]
			if coordinate == "longitude_deg":
				c = _center_lon + wrapf(c - _center_lon, -180.0, 180.0)
			var reach_m := float(site.get("plateau_radius_m", DEFAULT_PLATEAU_M)) \
					+ float(site.get("plateau_apron_m", DEFAULT_APRON_M)) + SITE_MARGIN_M
			var steps := ceili(reach_m / SITE_STEP_M)
			for k in range(-steps, steps + 1):
				var x := c + k * SITE_STEP_M * deg_per_m
				if x > low and x < high:
					lines.append(x)
	lines.sort()
	return lines


func _build() -> void:
	var terrain := flight_sim != null and bool(flight_sim.earth_surface(_center_lat, _center_lon)["present"])
	var center_fixed := geodetic_to_fixed(_center_lat, _center_lon, _vertex_height_m(_center_lat, _center_lon, terrain))
	var half := SPAN_DEG * 0.5
	var lon_span := SPAN_DEG / maxf(cos(deg_to_rad(_center_lat)), 0.2)  # keep cells roughly square
	var lats := _lines(_center_lat - half, _center_lat + half, terrain, "latitude_deg", 1.0 / METRES_PER_DEG)
	lats.reverse()  # rows go south
	var lons := _lines(_center_lon - lon_span * 0.5, _center_lon + lon_span * 0.5, terrain, "longitude_deg",
			1.0 / (METRES_PER_DEG * maxf(cos(deg_to_rad(_center_lat)), 0.2)))
	var rows := lats.size()
	var row := lons.size()
	var fixed := PackedVector3Array()  # Earth-fixed, relative to the centre (double)
	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()
	var uvs := PackedVector2Array()
	var indices := PackedInt32Array()
	for i in range(rows):
		var lat := clampf(lats[i], -89.9, 89.9)
		for j in range(row):
			var lon := lons[j]
			var p := geodetic_to_fixed(lat, lon, _vertex_height_m(lat, lon, terrain)) - center_fixed
			fixed.append(p)
			vertices.append(Frames.to_render(p))
			uvs.append(Vector2(fposmod(lon + 180.0, 360.0) / 360.0, (90.0 - lat) / 180.0))
	for i in range(rows):
		for j in range(row):
			var n: Vector3
			if terrain:  # the relief's normal: central differences (one-sided at the edges); rows go south
				var south := fixed[mini(i + 1, rows - 1) * row + j] - fixed[maxi(i - 1, 0) * row + j]
				var east := fixed[i * row + mini(j + 1, row - 1)] - fixed[i * row + maxi(j - 1, 0)]
				n = east.cross(-south).normalized()
			else:
				var la := deg_to_rad(clampf(lats[i], -89.9, 89.9))
				var lo := deg_to_rad(lons[j])
				n = Vector3(cos(la) * cos(lo), cos(la) * sin(lo), sin(la))
			normals.append(Frames.to_render(n))
	for i in range(rows - 1):
		for j in range(row - 1):
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
