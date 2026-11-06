class_name EarthNearShadow
extends RefCounted
## Near-field Sun shadows on the Earth (moon procedural surface plan Task 16; docs/subsystems/launch-structures.md
## "Sun shadows", terrain-rendering.md). The same technique as the Moon's (MoonNearShadow, Task 12): Godot's directional
## shadow map of the Sun's DirectionalLight3D (World.sun_light), parallel splits (PSSM, 4) around the eye, one shadow pass
## shared by both VR eyes (multiview), softened by PCSS with the solar disc's angular diameter (0.53 deg at 1 AU).
## Casters: the pad structures (tower lattice members, head, chopsticks, QD arm, mount, hardstand), vehicles, drone ship
## (Godot's default; the Earth terrain does not cast: TerrainQuadtree.cast_shadows false - the near relief is the flat
## pad plateau, its own shading comes from the relief normal). Receivers: every lit material (structures, vehicles) and
## terrain_earth.gdshader through its light() (earth_near_shadow.gdshaderinc): the shadow multiplies only the direct Sun
## (already transmitted through the atmosphere and the cloud shadow), never the sky or ground-bounce light.
## Active with the fidelity setting earth_near_shadows (the time panel's "Schatten Erde"), the Moon's near shadows not
## active, the eye within ACTIVE_HEIGHT_M of the Earth's ground; above it the light casts nothing (no cost in orbit).
## Both classes drive the one Sun light: MoonNearShadow.update runs first each frame, this one after it and only while
## the Moon's is off; the directional atlas goes back to the project's when neither is active.

## The persisted setting (user://settings.cfg [graphics] earth_near_shadows, default on; the time panel's check box).
const SETTINGS_FILE := "user://settings.cfg"
## Fidelity setting earth_near_shadows (render_check --earth_near_shadows=off).
static var enabled := load_enabled_setting()
## Diagnostics (render_check --earth_shadow_splits / --earth_shadow_soft / --earth_shadow_atlas): split count (1, 2, 4),
## PCSS softening (off: Godot's PCF only), the atlas size (px).
static var splits := 4
static var soft := true
static var atlas_size := ATLAS_SIZE
static var atlas_16_bit := true  ## 16-bit depth (render_check --earth_shadow_32bit: the former 32-bit atlas)
static var bias := SHADOW_BIAS  ## (render_check --earth_shadow_bias=<bias>,<normal bias>)
static var normal_bias := SHADOW_NORMAL_BIAS
static var soft_quality := SOFT_QUALITY  ## (render_check --earth_shadow_quality=hard|very_low|low|medium|high|ultra)

## [E] Active below this height of the eye over the Earth's ground: vehicles keep their own shadows on themselves while
## climbing out (the same bound as the Moon's ACTIVE_HEIGHT_M); above it the map would only hold a vehicle seen from far.
const ACTIVE_HEIGHT_M := 20000.0
## [E] The vehicles' self-shadows fade out between ACTIVE_HEIGHT_M and this height instead of switching off as a step (the light's
## shadow_opacity falls by a smoothstep; the terrain's own range, 4 km at most, no longer reaches the ground from there).
const FADE_END_HEIGHT_M := 30000.0
## [E] Shadow range (max distance, m of view depth): on the ground at least RANGE_MIN_M, and long enough for the shadow of
## a CASTER_HEIGHT_M caster at the Sun's elevation (h / tan e, at most RANGE_MAX_M) with RANGE_SHADOW_MARGIN, growing
## with the eye's height by RANGE_PER_HEIGHT (a vehicle's shadow on the ground below it).
const RANGE_MIN_M := 500.0
const RANGE_MAX_M := 4000.0
const RANGE_PER_HEIGHT := 4.0
## [D] The tallest caster at the pads: the Pad B tower head's top 147.9 m (launch-structures.md); the stacked vehicle on
## the mount reaches about as high.
const CASTER_HEIGHT_M := 147.9
## [E] The range covers the tallest caster's shadow plus a quarter (the eye stands anywhere around the pad; Godot fades
## the last FADE_START share out).
const RANGE_SHADOW_MARGIN := 1.25
## [E] The range's Sun-dependent part is quantised in steps of 2^(1/4) (19 %): a slowly setting Sun does not resize the
## splits every frame (each resize moves the texel grid: shimmering edges).
const RANGE_STEPS_PER_OCTAVE := 4.0
## [E] Godot's default PSSM splits (share of the range): first split 50 m at the 500 m range.
const SPLITS := Vector3(0.1, 0.2, 0.5)
## [E] The shadow fades out over the range's last 20 % (Godot's fade_start); the terrain's light() uses the same weight.
const FADE_START := 0.8
## [E, measured Task 13b] Directional shadow atlas 8192 px (4096 per split), 16-bit depth: VRAM 8192 x 8192 x 2 B = 128 MB while
## active (computed; 256 MB with the 32-bit atlas of Task 16, which gave the same image: Sun 39 deg shadow length -1.31 m both,
## 23 / 115 pixels of 1.44 M differ by > 8 / > 2 at 19 deg and 3 / 8 at 5.6 deg, single lattice edge pixels). Halving the atlas to
## 4096 doubles the texel (0.22 -> 0.44 m at 500 m) and the shadow edge's error (-1.3 -> -2.9 m at 190 m): kept at 8192.
const ATLAS_SIZE := 8192
## [E, renders Task 16] Bias in Godot's units (shadow_bias x the split's texel; normal bias along the receiver's normal in
## texels): no acne on the hardstand, the hulls and the members at 5-60 deg Sun, contact kept at the tower foot.
const SHADOW_BIAS := 0.03
const SHADOW_NORMAL_BIAS := 1.0
## [E, measured Task 16] Soft-shadow filter (PCSS blocker search + PCF): HIGH. The Moon's LOW left a visible grain in the
## wide penumbrae on the Earth's terrain at the shadow tips (vehicle_mount crops); MEDIUM barely less; HIGH smooth for
## +0.6-0.9 ms in the stereo mock at the pad (5.4-5.8 ms with shadows, 4.2-4.3 off).
const SOFT_QUALITY := RenderingServer.SHADOW_QUALITY_SOFT_HIGH

var active := false
var range_m := 0.0
var _atlas_set := false
var _logged_range_m := 0.0


## Per frame after MoonNearShadow.update: whether the Earth's near shadows apply, the light's shadow settings, the
## terrain shader's global. moon_active: the Moon's near shadows own the light this frame; height_over_ground_m: the eye
## over the Earth's ground (INF far from it); sun_elevation_rad: the Sun's elevation at the eye; sun_distance_m: the eye to
## the Sun (the disc's angular diameter).
func update(sun_light: DirectionalLight3D, moon_active: bool, height_over_ground_m: float, sun_elevation_rad: float,
		sun_distance_m: float) -> void:
	active = enabled and not moon_active and height_over_ground_m < FADE_END_HEIGHT_M and sun_elevation_rad > -0.02
	if not active:
		ShaderGlobals.set_value(&"earth_near_shadow", Vector4(0.0, 0.0, 0.0, 0.0))
		if moon_active:
			_atlas_set = false  # the Moon's settings are on the light and the atlas
			return
		sun_light.shadow_enabled = false
		sun_light.shadow_opacity = 1.0
		if _atlas_set:  # the project's atlas and filter back (as MoonNearShadow does on leaving the Moon)
			RenderingServer.directional_shadow_atlas_set_size(
					int(ProjectSettings.get_setting("rendering/lights_and_shadows/directional_shadow/size", 4096)),
					bool(ProjectSettings.get_setting("rendering/lights_and_shadows/directional_shadow/16_bits", true)))
			RenderingServer.directional_soft_shadow_filter_set_quality(int(ProjectSettings.get_setting(
					"rendering/lights_and_shadows/directional_shadow/soft_shadow_filter_quality", 2)))
			_atlas_set = false
		return
	if not _atlas_set:
		RenderingServer.directional_shadow_atlas_set_size(atlas_size, atlas_16_bit)
		RenderingServer.directional_soft_shadow_filter_set_quality(soft_quality)
		_atlas_set = true
	sun_light.shadow_enabled = true
	sun_light.shadow_opacity = strength(height_over_ground_m)
	range_m = shadow_range_m(height_over_ground_m, sun_elevation_rad)
	sun_light.directional_shadow_mode = DirectionalLight3D.SHADOW_PARALLEL_4_SPLITS if splits >= 4 \
			else (DirectionalLight3D.SHADOW_PARALLEL_2_SPLITS if splits == 2 else DirectionalLight3D.SHADOW_ORTHOGONAL)
	sun_light.directional_shadow_max_distance = range_m
	sun_light.directional_shadow_split_1 = SPLITS.x
	sun_light.directional_shadow_split_2 = SPLITS.y
	sun_light.directional_shadow_split_3 = SPLITS.z
	sun_light.directional_shadow_blend_splits = true
	sun_light.directional_shadow_fade_start = FADE_START
	sun_light.shadow_bias = bias
	sun_light.shadow_normal_bias = normal_bias
	sun_light.shadow_blur = 1.0
	sun_light.light_angular_distance = MoonNearShadow.solar_disc_diameter_deg(sun_distance_m) if soft else 0.0
	ShaderGlobals.set_value(&"earth_near_shadow", Vector4(1.0, FADE_START * range_m, range_m, 0.0))
	if absf(range_m - _logged_range_m) > 0.1 * _logged_range_m:
		print("[EarthNearShadow] range %.0f m (eye %.0f m over the ground, Sun %.1f deg)" % [range_m,
				height_over_ground_m, rad_to_deg(sun_elevation_rad)])
		_logged_range_m = range_m


## The shadows' strength (the light's shadow_opacity) at an eye height: 1 up to ACTIVE_HEIGHT_M, then a smoothstep to 0 at
## FADE_END_HEIGHT_M.
static func strength(height_over_ground_m: float) -> float:
	return 1.0 - smoothstep(ACTIVE_HEIGHT_M, FADE_END_HEIGHT_M, height_over_ground_m)


## The shadow range (m of view depth): the tallest caster's shadow at the Sun's elevation with its margin (quantised, at
## least RANGE_MIN_M), plus RANGE_PER_HEIGHT x the eye's height, at most RANGE_MAX_M.
static func shadow_range_m(height_over_ground_m: float, sun_elevation_rad: float) -> float:
	var tan_e := tan(clampf(sun_elevation_rad, deg_to_rad(1.0), deg_to_rad(89.0)))
	var shadow := maxf(RANGE_SHADOW_MARGIN * CASTER_HEIGHT_M / tan_e, RANGE_MIN_M)
	var octaves := ceilf(log(shadow / RANGE_MIN_M) / log(2.0) * RANGE_STEPS_PER_OCTAVE - 1.0e-6) / RANGE_STEPS_PER_OCTAVE
	shadow = RANGE_MIN_M * pow(2.0, octaves)
	return minf(shadow + RANGE_PER_HEIGHT * maxf(height_over_ground_m, 0.0), RANGE_MAX_M)


## The persisted fidelity setting (default on; a missing or unreadable file: on).
static func load_enabled_setting(path := SETTINGS_FILE) -> bool:
	var config := ConfigFile.new()
	if config.load(path) != OK:
		return true
	return bool(config.get_value("graphics", "earth_near_shadows", true))


static func save_enabled_setting(value: bool, path := SETTINGS_FILE) -> void:
	var config := ConfigFile.new()
	config.load(path)  # keeps the other settings; a missing file starts empty
	config.set_value("graphics", "earth_near_shadows", value)
	config.save(path)
