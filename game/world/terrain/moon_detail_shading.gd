class_name MoonDetailShading
## Material wiring of the Moon terrain's regolith detail and unresolved-surface statistics (moon_detail.gdshaderinc;
## moon procedural surface plan Task 10): the baked regolith normal maps (data/textures/moon_detail, Task 8), the
## core's statistics (StarsimCore.moon_unresolved_shading) and the rock abundance map, plus the per-node uniforms
## (texture origins, vertex and horizon-grid spacing) that TerrainQuadtree sets through node_parameters.

const TEXTURE_DIR := "textures/moon_detail"
const SCALES := ["s0", "s1", "s2"]
## [D] Hex cell = half a tile: every triangle-grid patch shows a different part of the map, tiles never repeat whole.
const HEX_CELL_PER_TILE := 0.5
## The shader's DETAIL_HEX_PERIOD (cells after which the hash repeats).
const HEX_PERIOD := 256.0
## [D] Rock abundance map grid: 1/8 deg (3.8 km at the equator; the Diviner map is 1/32 deg, smooth at this scale).
const ABUNDANCE_WIDTH := 2880
const ABUNDANCE_HEIGHT := 1440
## Texture weight table (moon_detail.gdshaderinc DETAIL_WEIGHT_*): node spacings 1 mm x 2^(k / 4), k = 0 .. 56.
const WEIGHT_SAMPLES := 57
const WEIGHT_PER_OCTAVE := 4
const WEIGHT_MIN_SPACING_M := 1.0e-3
## [D] Wavelength samples per band for the weight (log-spaced midpoints; < 0.5 % against 4x more).
const WEIGHT_BAND_SAMPLES := 48
## Diagnostics switch (render_check --moon_detail_shading=off): false leaves the detail shading off with the detail on.
static var enabled := true

## Sets the template material's detail uniforms; false (detail_shading off) without the core statistics or with the
## detail switched off. The textures are optional (absent: statistics only, a warning).
static func configure(core: StarsimCore, material: ShaderMaterial, detail_on: bool) -> Dictionary:
	material.set_shader_parameter("detail_shading", false)
	material.set_shader_parameter("detail_textures", false)
	if not detail_on or not enabled:
		return {}
	var stats: Dictionary = core.moon_unresolved_shading()
	if not bool(stats.get("present", false)):
		return {}
	var drawn := PackedFloat32Array(Array(stats["drawn_table"]))
	material.set_shader_parameter("detail_total_slope_variance", float(stats["total_slope_variance"]))
	material.set_shader_parameter("detail_drawn_table", drawn)
	material.set_shader_parameter("detail_drawn_axis", Vector2(float(stats["drawn_table_min_spacing_m"]),
			float(stats["drawn_table_per_octave"])))
	material.set_shader_parameter("detail_rock_law", Vector4(float(stats["rock_cover_coefficient"]),
			float(stats["rock_cover_exponent"]), float(stats["rock_min_diameter_m"]), float(stats["rock_max_diameter_m"])))
	material.set_shader_parameter("detail_rock_shape", Vector4(float(stats["rock_exposed_height_per_diameter"]),
			float(stats["rock_reflectance_ratio"]), float(stats["lit_gain_step_rad"]), float(stats["lit_gain_floor_rad"])))
	material.set_shader_parameter("detail_rock_lit_gain", PackedFloat32Array(Array(stats["rock_lit_gain"])))
	var grid: PackedFloat32Array = core.moon_rock_abundance_grid(ABUNDANCE_WIDTH, ABUNDANCE_HEIGHT)
	var abundance := Image.create_from_data(ABUNDANCE_WIDTH, ABUNDANCE_HEIGHT, false, Image.FORMAT_RF,
			grid.to_byte_array())
	material.set_shader_parameter("detail_rock_abundance", ImageTexture.create_from_image(abundance))
	material.set_shader_parameter("detail_shading", true)
	var tiles := _configure_textures(material)
	stats["tiles"] = tiles
	return stats


## Loads the three regolith normal maps (mipmapped) and their scale table; returns [tile_m, hex_cell_m] per scale
## ([] when absent).
static func _configure_textures(material: ShaderMaterial) -> Array:
	var dir := Paths.data_dir().path_join(TEXTURE_DIR)
	var toml_path := dir.path_join("moon_detail.toml")
	if not FileAccess.file_exists(toml_path):
		push_warning("MoonDetailShading: %s fehlt (python tools/data_pipeline/fetch.py moon_detail) - Regolith ohne "
				% toml_path + "Detailtexturen")
		return []
	var table: Dictionary = TomlLite.load_file(toml_path)
	var scales: Array = table.get("regolith", [])
	var tiles := []
	var params := []
	for i in SCALES.size():
		var entry: Dictionary = {}
		for s: Dictionary in scales:
			if s.get("name", "") == SCALES[i]:
				entry = s
		var path := dir.path_join("regolith_normal_%s.png" % SCALES[i])
		if entry.is_empty() or not FileAccess.file_exists(path):
			push_warning("MoonDetailShading: %s fehlt - Regolith ohne Detailtexturen" % path)
			return []
		material.set_shader_parameter("detail_slope_" + SCALES[i], ImageTexture.create_from_image(slope_image(path)))
		var tile := float(entry["tile_m"])
		params.append(Vector4(tile, tile * HEX_CELL_PER_TILE, float(entry["wavelength_min_m"]),
				float(entry["wavelength_max_m"])))
		tiles.append([tile, tile * HEX_CELL_PER_TILE])
	var hurst := float(table["hurst_exponent"])
	var weights := PackedFloat32Array()
	for p: Vector4 in params:
		for k in WEIGHT_SAMPLES:
			var spacing := WEIGHT_MIN_SPACING_M * pow(2.0, float(k) / WEIGHT_PER_OCTAVE)
			weights.append(texture_weight(p.z, p.w, hurst, spacing))
	material.set_shader_parameter("detail_weight_table", weights)
	material.set_shader_parameter("detail_scale", params)
	material.set_shader_parameter("detail_textures", true)
	return tiles


## Nominal vertex spacing of a level (m): a face's edge R pi / 2 over 64 x 2^level cells (the texture metric; the true
## spacing is 0.84-1 x this).
static func cell_m(level: int, radius_m: float) -> float:
	return radius_m * PI / (2.0 * TerrainNodeBuilder.CELLS * float(1 << level))


## Per-node uniforms: detail_cell_m, detail_origin (per scale: the node corner's texture coordinate modulo the tile and
## its skewed hex coordinate modulo HEX_PERIOD, from the face lattice in float64) and detail_horizon_spacing_m.
static func node_parameters(key: Vector4i, horizon_owner: Vector4i, radius_m: float, tiles: Array) -> Dictionary:
	var cell := cell_m(key.y, radius_m)
	var u0 := float(key.z) * TerrainNodeBuilder.CELLS * cell
	var v0 := float(key.w) * TerrainNodeBuilder.CELLS * cell
	var origins := []
	for t: Array in tiles:
		var tile: float = t[0]
		var hex: float = t[1]
		var skew_u := (u0 - v0 / sqrt(3.0)) / hex
		var skew_v := 2.0 * v0 / sqrt(3.0) / hex
		origins.append(Vector4(fposmod(u0, tile), fposmod(v0, tile), fposmod(skew_u, HEX_PERIOD),
				fposmod(skew_v, HEX_PERIOD)))
	while origins.size() < 3:
		origins.append(Vector4.ZERO)
	return {"detail_cell_m": cell, "detail_origin": origins,
			"detail_horizon_spacing_m": cell_m(horizon_owner.y, radius_m)}


## Amplitude weight of a texture band [lambda_min, lambda_max] (octave height variance ~ lambda^(2 H), the bake's law)
## on a mesh of vertex spacing L: sqrt of the band's height variance the mesh does not draw. The mesh's vertices are
## the L x L box mean (amplitude sinc(pi L / lambda)) linearly interpolated between vertices (a triangle kernel:
## sinc^2), so it keeps sinc^6(pi L / lambda) of a wavelength's variance (aliasing neglected); the texture adds the rest:
## w^2 = sum V (1 - sinc^6) / sum V.
static func texture_weight(lambda_min: float, lambda_max: float, hurst: float, spacing_m: float) -> float:
	var missing := 0.0
	var total := 0.0
	for j in WEIGHT_BAND_SAMPLES:
		var lam := lambda_min * pow(lambda_max / lambda_min, (j + 0.5) / WEIGHT_BAND_SAMPLES)
		var v := pow(lam, 2.0 * hurst)
		var x := PI * spacing_m / lam
		var sinc := sin(x) / x
		var s2 := sinc * sinc
		missing += v * (1.0 - s2 * s2 * s2)
		total += v
	return sqrt(clampf(missing / total, 0.0, 1.0))


## The baked normal map as a two-channel half-float slope map (dh/du, dh/dv) with zero mean, mipmapped: slopes average
## linearly, so a mip texel is the footprint's mean slope (8-bit normal maps would also carry the encoding's offset - a
## zero slope sits on code 128 = +0.0039 - and Godot's rounded 8-bit mips drift by up to ~1 code over the chain).
## OpenGL tangent space: red = +u, green = toward the image top = -v.
static func slope_image(path: String) -> Image:
	var image := Image.load_from_file(path)
	image.convert(Image.FORMAT_RGB8)
	var w := image.get_width()
	var h := image.get_height()
	var src := image.get_data()
	var slopes := PackedFloat32Array()
	slopes.resize(2 * w * h)
	var mean := Vector2.ZERO
	for k in w * h:
		var nx := src[3 * k] / 127.5 - 1.0
		var ny := src[3 * k + 1] / 127.5 - 1.0
		var nz := maxf(src[3 * k + 2] / 127.5 - 1.0, 0.05)
		var sx := -nx / nz
		var sy := ny / nz
		slopes[2 * k] = sx
		slopes[2 * k + 1] = sy
		mean += Vector2(sx, sy)
	mean /= float(w * h)
	for k in w * h:
		slopes[2 * k] -= mean.x
		slopes[2 * k + 1] -= mean.y
	var out := Image.create_from_data(w, h, false, Image.FORMAT_RGF, slopes.to_byte_array())
	out.generate_mipmaps()
	out.convert(Image.FORMAT_RGH)  # half floats: half the texture bandwidth (slopes to ~1e-3 relative)
	return out
