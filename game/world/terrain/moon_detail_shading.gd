class_name MoonDetailShading
## Material wiring of the Moon terrain's regolith detail and unresolved-surface statistics (moon_detail.gdshaderinc;
## moon procedural surface plan Task 10): the baked regolith + crater slope maps (data/textures/moon_detail), the
## core's statistics (StarsimCore.moon_unresolved_shading) and the rock abundance map, plus the per-node uniforms
## (texture origins, vertex and horizon-grid spacing) that TerrainQuadtree sets through node_parameters.

const TEXTURE_DIR := "textures/moon_detail"
const SCALES := ["s0", "s1", "s2", "s3"]
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
## The baked slope maps' size (bake_moon_detail.py REGOLITH_SIZE_PX): four half floats per texel.
const SLOPE_SIZE_PX := 1024
## Diagnostics switch (render_check --moon_detail_shading=off): false leaves the detail shading off with the detail on.
static var enabled := true

## Sets the template material's detail uniforms; false (detail_shading off) without the core statistics or with the
## detail switched off. The textures are optional (absent: statistics only, a warning).
static func configure(core: StarsimCore, material: ShaderMaterial, detail_on: bool) -> Dictionary:
	material.set_shader_parameter("detail_shading", false)
	material.set_shader_parameter("detail_textures", false)
	if not detail_on or not enabled:
		return {}
	var start_us := Time.get_ticks_usec()
	var stats: Dictionary = core.moon_unresolved_shading()
	print("[MoonDetailShading] unresolved statistics in %.0f ms (rock view tables from data/cache when present)" % (
			(Time.get_ticks_usec() - start_us) * 1.0e-3))
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
	material.set_shader_parameter("detail_rock_shadow", PackedFloat32Array(Array(stats["rock_shadow"])))
	# Task 12: the oblique eye's rock terms (RG float texture, the core's table layout) and the fill's albedo.
	var excess := PackedFloat64Array(stats.get("rock_view_excess", PackedFloat64Array()))
	var ratio := PackedFloat64Array(stats.get("rock_view_gain_ratio", PackedFloat64Array()))
	var gains := Array(stats["rock_lit_gain"]).size()
	material.set_shader_parameter("detail_rock_view_present", excess.size() > 0 and excess.size() == ratio.size())
	if excess.size() > 0 and excess.size() == ratio.size():
		var rows := excess.size() / gains
		var view := PackedFloat32Array()
		view.resize(2 * excess.size())
		for k in excess.size():
			view[2 * k] = excess[k]
			view[2 * k + 1] = ratio[k]
		material.set_shader_parameter("detail_rock_view", ImageTexture.create_from_image(
				Image.create_from_data(gains, rows, false, Image.FORMAT_RGF, view.to_byte_array())))
		material.set_shader_parameter("detail_view_step", Vector2(float(stats["view_elevation_step_rad"]),
				float(stats["view_azimuth_step_rad"])))
	material.set_shader_parameter("detail_hemispherical_albedo_mu", hemispherical_albedo_mu(
			PackedFloat64Array(stats.get("hemispherical_albedo", PackedFloat64Array())), float(stats["lit_gain_step_rad"])))
	var grid: PackedFloat32Array = core.moon_rock_abundance_grid(ABUNDANCE_WIDTH, ABUNDANCE_HEIGHT)
	var abundance := Image.create_from_data(ABUNDANCE_WIDTH, ABUNDANCE_HEIGHT, false, Image.FORMAT_RF,
			grid.to_byte_array())
	material.set_shader_parameter("detail_rock_abundance", ImageTexture.create_from_image(abundance))
	material.set_shader_parameter("detail_shading", true)
	var tiles := _configure_textures(material, stats)
	stats["tiles"] = tiles
	return stats


## The fill light's A_h table resampled from the core's elevation grid (k x step_rad, linear between) onto the shader's mu
## grid mu = k / MU_STEPS (moon_detail.gdshaderinc DETAIL_ALBEDO_MU_STEPS; review I5 of Task 12: no asin per lookup).
const MU_STEPS := 90
static func hemispherical_albedo_mu(by_elevation: PackedFloat64Array, step_rad: float) -> PackedFloat32Array:
	var out := PackedFloat32Array()
	out.resize(MU_STEPS + 1)
	out.fill(0.0)
	if by_elevation.size() < 2 or step_rad <= 0.0:
		return out
	for k in MU_STEPS + 1:
		var x := clampf(asin(float(k) / MU_STEPS) / step_rad, 0.0, by_elevation.size() - 1.0)
		var i := mini(int(x), by_elevation.size() - 2)
		out[k] = lerpf(by_elevation[i], by_elevation[i + 1], x - i)
	return out


## Loads the regolith slope maps (mipmapped) and their scale table; returns [tile_m, hex_cell_m] per scale ([] when
## absent).
static func _configure_textures(material: ShaderMaterial, stats: Dictionary) -> Array:
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
		var path := dir.path_join("regolith_slope_%s.rgbah" % SCALES[i])
		var image := slope_image(path)
		if entry.is_empty() or image == null:
			push_warning("MoonDetailShading: %s fehlt oder ist veraltet (python tools/data_pipeline/fetch.py --force "
					% path + "moon_detail) - Regolith ohne Detailtexturen")
			return []
		material.set_shader_parameter("detail_slope_" + SCALES[i], ImageTexture.create_from_image(image))
		var tile := float(entry["tile_m"])
		params.append(Vector4(tile, tile * HEX_CELL_PER_TILE, float(entry["wavelength_min_m"]),
				float(entry["wavelength_max_m"])))
		tiles.append([tile, tile * HEX_CELL_PER_TILE])
	var hurst := float(table["hurst_exponent"])
	var weights := PackedFloat32Array()
	var crater_weights := PackedFloat32Array()
	for i in params.size():
		var p: Vector4 = params[i]
		for k in WEIGHT_SAMPLES:
			var spacing := WEIGHT_MIN_SPACING_M * pow(2.0, float(k) / WEIGHT_PER_OCTAVE)
			weights.append(texture_weight(p.z, p.w, hurst, spacing))
			crater_weights.append(crater_weight(scales_entry(scales, SCALES[i]), stats, spacing))
	material.set_shader_parameter("detail_weight_table", weights)
	material.set_shader_parameter("detail_crater_weight_table", crater_weights)
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
	while origins.size() < SCALES.size():
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


## The scale's entry of moon_detail.toml's [[regolith]] list ({} when absent).
static func scales_entry(scales: Array, name: String) -> Dictionary:
	for s: Dictionary in scales:
		if s.get("name", "") == name:
			return s
	return {}


## Amplitude weight of a scale's crater component (craters of the core's law with D in [crater_min_diameter_m,
## crater_max_diameter_m), moon_detail.toml) on a mesh of vertex spacing L: sqrt of the share of their slope variance
## the mesh does not draw, 1 - the mean of the core's crater_drawn_fraction(D / L) over the band weighted by the
## law's slope variance per ln D (~ D^q, q = 2 - b). The table (stats crater_fraction) is the core's. A scale whose
## crater component also holds the pebbles (bake_moon_detail.py: rocks below MoonRockLayer.PEBBLE_GEOMETRY_LIMIT_M, never
## mesh-drawn, weight 1) gets the variance-combined weight sqrt((w_c^2 V_craters + V_pebbles) / (V_craters +
## V_pebbles)) (heights: crater_only_rms_m, pebble_rms_m).
static func crater_weight(entry: Dictionary, stats: Dictionary, spacing_m: float) -> float:
	var w_c := _crater_only_weight(entry, stats, spacing_m)
	var v_p := pow(float(entry.get("pebble_rms_m", 0.0)), 2.0)
	if v_p <= 0.0:
		return w_c
	var v_c := pow(float(entry.get("crater_only_rms_m", 0.0)), 2.0)
	return sqrt((w_c * w_c * v_c + v_p) / (v_c + v_p))


static func _crater_only_weight(entry: Dictionary, stats: Dictionary, spacing_m: float) -> float:
	var d_lo := float(entry.get("crater_min_diameter_m", 0.0))
	var d_hi := float(entry.get("crater_max_diameter_m", 0.0))
	var table: PackedFloat64Array = stats.get("crater_fraction", PackedFloat64Array())
	if d_hi <= d_lo or table.size() < 2:
		return 0.0
	var lowest := float(stats["crater_fraction_lowest_octaves"])
	var q := float(stats["crater_slope_exponent"])
	var drawn := 0.0
	var total := 0.0
	for j in WEIGHT_BAND_SAMPLES:
		var d := d_lo * pow(d_hi / d_lo, (j + 0.5) / WEIGHT_BAND_SAMPLES)
		var v := pow(d, q)
		var x := 2.0 * (log(d / spacing_m) / log(2.0) + lowest)  # crater_drawn_fraction_table
		var f := 0.0
		if x >= 0.0:
			x = minf(x, table.size() - 1)
			var k := mini(int(x), table.size() - 2)
			f = lerpf(table[k], table[k + 1], x - k)
		drawn += v * f
		total += v
	return sqrt(clampf(1.0 - drawn / total, 0.0, 1.0))


## The baked slope map (bake_moon_detail.py: raw little-endian half floats - regolith dh/du, dh/dv, craters dh/du,
## dh/dv - zero mean, from the float heights) mipmapped; null when absent or of the wrong size. Slopes average linearly,
## so a mip texel is the footprint's mean slope.
static func slope_image(path: String) -> Image:
	if not FileAccess.file_exists(path):
		return null
	var bytes := FileAccess.get_file_as_bytes(path)
	if bytes.size() != SLOPE_SIZE_PX * SLOPE_SIZE_PX * 8:
		return null
	var out := Image.create_from_data(SLOPE_SIZE_PX, SLOPE_SIZE_PX, false, Image.FORMAT_RGBAH, bytes)
	out.generate_mipmaps()
	return out
