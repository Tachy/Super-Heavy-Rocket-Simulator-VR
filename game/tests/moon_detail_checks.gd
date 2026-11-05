class_name MoonDetailChecks
## CPU checks of the Moon regolith detail shading for moon_surface_check.gd (moon procedural surface plan Task 10):
## the summed (mesh + texture) roughness at Guo's 125 mm baseline, the continuity of the residual theta_bar and the
## light factors in the node spacing (no step when a node splits) and the rock abundance grid.

const R := HeightSource.MOON_RADIUS_M
const GUO_SAMPLES := 25  ## [S] Guo et al. 2021: 25 x 25 samples at 5 mm (125 mm window)
const GUO_PIXEL_M := 0.005
const WINDOWS := 96  ## [D] hashed windows per spacing (the median's spread ~ 1.25 / sqrt(n) x sigma ~ 2 %)
const ROUGHNESS_TOLERANCE := 0.10  ## the brief's 4 mm +- 10 %
## Node spacings of levels 22 .. 17 at the site (nominal: R pi / (2 x 64 x 2^level)).
const LEVELS := [22, 21, 20, 19, 18, 17]


## Guo's statistic of mesh + textures at each level's spacing: the median plane-detrended RMS of 25 x 25 samples (5 mm)
## of the node mesh (the core's band-limited heights at its vertices, bilinear between them) plus every texture scale's
## regolith and crater components at their weights (MoonDetailShading.texture_weight / crater_weight; the two
## components of a scale at the same texture position, as the shader fetches them) plus the drawn rocks' tops (the
## core's rocks from PEBBLE_GEOMETRY_LIMIT_M at the samples). Returns lines; failures via check.
static func summed_roughness(core: StarsimCore, site: Vector2, check: Callable) -> void:
	var dir := Paths.data_dir().path_join(MoonDetailShading.TEXTURE_DIR)
	var table: Dictionary = TomlLite.load_file(dir.path_join("moon_detail.toml"))
	var stats: Dictionary = core.moon_surface_statistics()
	var unresolved: Dictionary = core.moon_unresolved_shading()
	var target := float(stats["roughness"]["rms_height_measured_total_m"])
	var maps := []
	var slot := 0
	for entry: Dictionary in table["regolith"]:
		for crater in [false, true]:
			var image := Image.load_from_file(dir.path_join("%s_height_%s.png"
					% ["crater" if crater else "regolith", entry["name"]]))
			image.convert(Image.FORMAT_RF)
			maps.append({"image": image, "pixel_m": float(entry["pixel_m"]), "size": image.get_width(),
					"scale_m": float(entry["crater_height_scale_m" if crater else "height_scale_m"]),
					"lo": float(entry["wavelength_min_m"]), "hi": float(entry["wavelength_max_m"]), "crater": crater,
					"entry": entry, "slot": slot})
		slot += 1
	var centre := CubeSphere.geodetic_direction(site.x, site.y)
	var east := Vector3(0, 0, 1).cross(centre).normalized()
	var north := centre.cross(east)
	# The drawn rocks (D >= MoonRockLayer.PEBBLE_GEOMETRY_LIMIT_M; smaller ones are the texture's pebble bumps) on top:
	# per window the core's rock-top increment at the samples, the same at every level (Task 11 fix round: what is
	# drawn is mesh + textures + rock geometry, and Guo's 4 mm includes the rocks).
	var rock_dh: Array[PackedFloat64Array] = []
	for w in WINDOWS:
		var ox := (_hash(w, 1) - 0.5) * 1000.0
		var oy := (_hash(w, 2) - 0.5) * 1000.0
		var lats := PackedFloat64Array()
		var lons := PackedFloat64Array()
		for b in GUO_SAMPLES:
			for a in GUO_SAMPLES:
				var u := (centre + (east * (ox + a * GUO_PIXEL_M) + north * (oy + b * GUO_PIXEL_M)) / R).normalized()
				var ll := CubeSphere.lat_lon_deg(u)
				lats.append(ll.x)
				lons.append(ll.y)
		var with_rocks: PackedFloat64Array = core.surface_samples_detail("moon", lats, lons, 0.0,
				MoonRockLayer.PEBBLE_GEOMETRY_LIMIT_M)["height_m"]
		var bare: PackedFloat64Array = core.surface_samples_detail("moon", lats, lons, 0.0, INF)["height_m"]
		var dh := PackedFloat64Array()
		for k in with_rocks.size():
			dh.append(with_rocks[k] - bare[k])
		rock_dh.append(dh)
	for level: int in LEVELS:
		var spacing := MoonDetailShading.cell_m(level, R)
		var rms := PackedFloat64Array()
		var weights := []
		for m: Dictionary in maps:
			weights.append(MoonDetailShading.crater_weight(m["entry"], unresolved, spacing) if m["crater"]
					else MoonDetailShading.texture_weight(m["lo"], m["hi"], float(table["hurst_exponent"]), spacing))
		for w in WINDOWS:
			var ox := (_hash(w, 1) - 0.5) * 1000.0
			var oy := (_hash(w, 2) - 0.5) * 1000.0
			# Mesh lattice covering the window, aligned to multiples of the spacing (as a node's vertices).
			var i0 := floori(ox / spacing)
			var j0 := floori(oy / spacing)
			var nv := ceili(GUO_SAMPLES * GUO_PIXEL_M / spacing) + 2
			var lats := PackedFloat64Array()
			var lons := PackedFloat64Array()
			for j in nv:
				for i in nv:
					var u := (centre + (east * ((i0 + i) * spacing) + north * ((j0 + j) * spacing)) / R).normalized()
					var ll := CubeSphere.lat_lon_deg(u)
					lats.append(ll.x)
					lons.append(ll.y)
			var mesh: PackedFloat64Array = core.surface_samples_detail("moon", lats, lons, spacing, INF)["height_m"]
			var z := PackedFloat64Array()
			for b in GUO_SAMPLES:
				for a in GUO_SAMPLES:
					var x := ox + a * GUO_PIXEL_M
					var y := oy + b * GUO_PIXEL_M
					var fx := x / spacing - i0
					var fy := y / spacing - j0
					var ia := floori(fx)
					var jb := floori(fy)
					fx -= ia
					fy -= jb
					var h00 := mesh[jb * nv + ia]
					var h10 := mesh[jb * nv + ia + 1]
					var h01 := mesh[(jb + 1) * nv + ia]
					var h11 := mesh[(jb + 1) * nv + ia + 1]
					var h := lerpf(lerpf(h00, h10, fx), lerpf(h01, h11, fx), fy) + rock_dh[w][b * GUO_SAMPLES + a]
					for k in maps.size():
						if weights[k] > 0.0:
							var sl: int = maps[k]["slot"]
							h += weights[k] * _texture_height(maps[k], x + 37.1 * sl + 101.3 * _hash(w, 3 + sl),
									y + 53.7 * sl + 89.9 * _hash(w, 7 + sl))
					z.append(h)
			rms.append(_detrended_rms(z))
		var sorted := Array(rms)
		sorted.sort()
		var median: float = sorted[sorted.size() / 2]
		var listed := PackedStringArray()
		for k in range(0, maps.size(), 2):
			listed.append("%.2f/%.2f" % [weights[k], weights[k + 1]])
		check.call(absf(median / target - 1.0) < ROUGHNESS_TOLERANCE,
				"summed roughness mesh + textures + drawn rocks, level %d (spacing %.4f m, weights regolith/craters %s): Guo median "
				% [level, spacing, " ".join(listed)]
				+ "%.2f mm against %.1f mm (%+.1f %%, gate 10 %%)" % [1000.0 * median, 1000.0 * target,
				100.0 * (median / target - 1.0)])


## The residual theta_bar and the light factors over node spacings in 1 % steps (a split halves the spacing through the
## morph continuously): the largest change per step stays tiny, no jump.
static func continuity(core: StarsimCore, check: Callable) -> void:
	var worst_theta := 0.0
	var worst_sun := 0.0
	var previous_theta := core.moon_residual_theta_bar(0.01)
	var previous_sun := core.moon_unresolved_light_factors(1.0, INF, 0.01, 20.0, deg_to_rad(7.0), deg_to_rad(7.0)).x
	var d := 0.0101
	while d < 3.0e4:
		var theta := core.moon_residual_theta_bar(d)
		var sun := core.moon_unresolved_light_factors(1.0, INF, d, 20.0, deg_to_rad(7.0), deg_to_rad(7.0)).x
		worst_theta = maxf(worst_theta, absf(theta - previous_theta))
		worst_sun = maxf(worst_sun, absf(sun - previous_sun) / maxf(sun, 1e-6))
		previous_theta = theta
		previous_sun = sun
		d *= 1.01
	print("  info  residual theta_bar: L22 (1 cm) %.2f deg, L18 %.2f, L14 %.2f, L10 %.2f, globe %.2f"
			% [rad_to_deg(core.moon_residual_theta_bar(MoonDetailShading.cell_m(22, R))),
			rad_to_deg(core.moon_residual_theta_bar(MoonDetailShading.cell_m(18, R))),
			rad_to_deg(core.moon_residual_theta_bar(MoonDetailShading.cell_m(14, R))),
			rad_to_deg(core.moon_residual_theta_bar(MoonDetailShading.cell_m(10, R))),
			rad_to_deg(core.moon_residual_theta_bar(1.0e7))])
	check.call(worst_theta < 1.0e-3 and worst_sun < 0.005,
			"fade continuity over the node spacing (1 %% steps, 1 cm - 30 km): theta_bar changes <= %.7f rad, Sun factor "
			% worst_theta + "<= %.2f %% per step (Sun 7 deg, horizon grid 20 m)" % (100.0 * worst_sun))


static func abundance_grid(core: StarsimCore, check: Callable) -> void:
	var grid: PackedFloat32Array = core.moon_rock_abundance_grid(MoonDetailShading.ABUNDANCE_WIDTH,
			MoonDetailShading.ABUNDANCE_HEIGHT)
	var lo := INF
	var hi := -INF
	var finite := true
	for v in grid:
		finite = finite and is_finite(v)
		lo = minf(lo, v)
		hi = maxf(hi, v)
	check.call(finite and lo >= 0.0 and grid.size() == MoonDetailShading.ABUNDANCE_WIDTH * MoonDetailShading.ABUNDANCE_HEIGHT,
			"rock abundance grid %d x %d finite, factor %.2f .. %.2f" % [MoonDetailShading.ABUNDANCE_WIDTH,
			MoonDetailShading.ABUNDANCE_HEIGHT, lo, hi])


static func _texture_height(m: Dictionary, x: float, y: float) -> float:
	var image: Image = m["image"]
	var size: int = m["size"]
	var px := x / float(m["pixel_m"])
	var py := y / float(m["pixel_m"])
	var ix := floori(px)
	var iy := floori(py)
	var fx := px - ix
	var fy := py - iy
	var a := image.get_pixel(posmod(ix, size), posmod(iy, size)).r
	var b := image.get_pixel(posmod(ix + 1, size), posmod(iy, size)).r
	var c := image.get_pixel(posmod(ix, size), posmod(iy + 1, size)).r
	var d := image.get_pixel(posmod(ix + 1, size), posmod(iy + 1, size)).r
	var value := lerpf(lerpf(a, b, fx), lerpf(c, d, fx), fy)  # normalised 16-bit code
	return (value * 65535.0 - 32768.0) / 32767.0 * float(m["scale_m"])


## Plane-detrended RMS of a GUO_SAMPLES^2 window (least squares z = a + b x + c y on the centred grid).
static func _detrended_rms(z: PackedFloat64Array) -> float:
	var n := GUO_SAMPLES
	var c := (n - 1) * 0.5
	var sum := 0.0
	var sx := 0.0
	var sy := 0.0
	var xx := 0.0
	for b in n:
		for a in n:
			var v := z[b * n + a]
			sum += v
			sx += v * (a - c)
			sy += v * (b - c)
			xx += (a - c) * (a - c)
	var mean := sum / (n * n)
	var gx := sx / xx
	var gy := sy / xx
	var ss := 0.0
	for b in n:
		for a in n:
			var r := z[b * n + a] - mean - gx * (a - c) - gy * (b - c)
			ss += r * r
	return sqrt(ss / (n * n))


static func _hash(a: int, b: int) -> float:
	var h := hash(Vector2i(a * 7919 + 17, b * 104729 + 3))
	return float(h & 0xffffff) / float(0x1000000)
