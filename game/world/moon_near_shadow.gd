class_name MoonNearShadow
extends RefCounted
## Near-field Sun shadows on the Moon (moon procedural surface plan Task 12; docs/subsystems/moon-surface.md
## "Near-field shadows"). Technique: Godot's directional shadow map of the Sun's DirectionalLight3D (World.sun_light):
## parallel-split (PSSM, 4 splits) around the eye, rendered once per frame for both VR eyes (multiview shares the
## light's shadow pass), softened by PCSS with the solar disc's angular diameter (0.53 deg at 1 AU; light_angular_distance).
## Casters: the Moon's terrain nodes (TerrainQuadtree.cast_shadows), the drawn rocks (MoonRockLayer), vehicles and
## structures (Godot's default). Receivers: terrain_moon.gdshader and moon_rock.gdshader (their light(),
## moon_near_shadow.gdshaderinc), every lit material (vehicles). Beyond the range Godot fades the shadow out over its last
## FADE_START share and the receivers blend the statistical shadow share in with the same weight (no step).
## Active only near the Moon's ground with the fidelity setting moon_near_shadows on; elsewhere the Sun light casts no
## shadow (the Earth: unchanged) and the globals say off (the shaders' former expressions).

## Fidelity setting moon_near_shadows (render_check --moon_near_shadows=off; the settings' graphics page).
static var enabled := true
## The fill light from the sunlit terrain around (moon_fill.gdshaderinc), with the shadows (render_check --moon_fill=off).
static var fill_enabled := true
## Diagnostics (render_check --moon_shadow_casters=terrain,rocks / --moon_shadow_splits / --moon_shadow_soft): which
## of the Moon's own geometry casts (read at the terrain's setup / the rock pool's allocation), the PSSM split count
## (1, 2 or 4) and the PCSS softening (off: Godot's PCF only).
static var terrain_casts := true
static var rocks_cast := true
static var splits := 4
static var soft := true
static var soft_quality := SOFT_QUALITY  ## (render_check --moon_shadow_quality=hard|low|medium|high|ultra)

## [E] Active below this height of the eye over the Moon's ground: above it the drawn relief and rocks are too small
## for the map and the horizon map with the statistics carries the shadows.
const ACTIVE_HEIGHT_M := 20000.0
## [E] Shadow range (max distance, m of view depth): RANGE_MIN_M on the ground, growing with the eye's height (a landing
## vehicle's shadow reaches the ground hundreds of metres away at a low Sun), at most RANGE_MAX_M.
const RANGE_MIN_M := 150.0
const RANGE_PER_HEIGHT := 4.0
const RANGE_MAX_M := 4000.0
## [E] Godot's default PSSM splits (share of the range): first split 15 m at the 150 m range (texel ~1 cm at 8192).
const SPLITS := Vector3(0.1, 0.2, 0.5)
## [E] The shadow fades out over the range's last 20 % (Godot's fade_start); the statistics fade in with the same weight.
const FADE_START := 0.8
## [E] Directional shadow atlas: 8192 px (4096 per split), 32-bit depth (the light's depth range spans km at a low Sun).
const ATLAS_SIZE := 8192
## [E, renders 2026-11-05] Bias in Godot's units (shadow_bias x the split's texel; normal bias along the receiver's
## normal in texels): no acne on the terrain at a 3-7 deg Sun, contact kept under 2 m rocks.
const SHADOW_BIAS := 0.03
const SHADOW_NORMAL_BIAS := 1.0
## [E] Soft-shadow filter (PCSS blocker search + PCF): RenderingServer.SHADOW_QUALITY_SOFT_MEDIUM.
const SOFT_QUALITY := RenderingServer.SHADOW_QUALITY_SOFT_LOW

var active := false
var range_m := 0.0
var _atlas_set := false
var _logged_range_m := 0.0


## Per frame after the Sun light is placed: whether the near shadows apply (enabled, the Moon terrain drawn, the eye
## within ACTIVE_HEIGHT_M of the ground), the light's shadow settings, the shaders' globals. height_over_ground_m: the
## eye over the terrain under it; sun_distance_m: the eye to the Sun (the disc's angular diameter); eye_world: the eye in
## the render world (the head centre in VR).
func update(sun_light: DirectionalLight3D, moon_terrain_active: bool, height_over_ground_m: float, sun_distance_m: float,
		eye_world: Vector3, projection := Projection()) -> void:
	active = enabled and moon_terrain_active and height_over_ground_m < ACTIVE_HEIGHT_M
	sun_light.shadow_enabled = active
	ShaderGlobals.set_value(&"moon_eye_world", eye_world)
	if not active:
		ShaderGlobals.set_value(&"moon_near_shadow", Vector4(0.0, 0.0, 0.0, 1.0 if fill_enabled else 0.0))
		return
	if not _atlas_set:
		RenderingServer.directional_shadow_atlas_set_size(ATLAS_SIZE, false)
		RenderingServer.directional_soft_shadow_filter_set_quality(soft_quality)
		_atlas_set = true
	range_m = clampf(RANGE_MIN_M + RANGE_PER_HEIGHT * maxf(height_over_ground_m, 0.0), RANGE_MIN_M, RANGE_MAX_M)
	sun_light.directional_shadow_mode = DirectionalLight3D.SHADOW_PARALLEL_4_SPLITS if splits >= 4 \
			else (DirectionalLight3D.SHADOW_PARALLEL_2_SPLITS if splits == 2 else DirectionalLight3D.SHADOW_ORTHOGONAL)
	sun_light.directional_shadow_max_distance = range_m
	sun_light.directional_shadow_split_1 = SPLITS.x
	sun_light.directional_shadow_split_2 = SPLITS.y
	sun_light.directional_shadow_split_3 = SPLITS.z
	sun_light.directional_shadow_blend_splits = true
	sun_light.directional_shadow_fade_start = FADE_START
	sun_light.shadow_bias = SHADOW_BIAS
	sun_light.shadow_normal_bias = SHADOW_NORMAL_BIAS
	sun_light.shadow_blur = 1.0
	sun_light.light_angular_distance = solar_disc_diameter_deg(sun_distance_m) if soft else 0.0
	ShaderGlobals.set_value(&"moon_near_shadow", Vector4(1.0, FADE_START * range_m, range_m,
			1.0 if fill_enabled else 0.0))
	var near_m := 0.05
	var tan_y := 1.0
	var aspect := 1.0
	if projection != Projection() and projection.is_orthogonal() == false:
		near_m = projection.get_z_near()
		aspect = projection.get_aspect()
		tan_y = tan(deg_to_rad(0.5 * projection.get_fov())) / aspect  # get_fov: the horizontal angle
	var texels := split_texels(range_m, near_m, tan_y, aspect)
	ShaderGlobals.set_value(&"moon_near_shadow_texel", texels[0])
	ShaderGlobals.set_value(&"moon_near_shadow_split", texels[1])
	if absf(range_m - _logged_range_m) > 0.25 * _logged_range_m:
		print("[MoonNearShadow] range %.0f m, split far %s m, texel %s m" % [range_m, texels[1], texels[0]])
		_logged_range_m = range_m


## Each split's shadow-map texel (m) and far view depth (m), as Godot's RendererSceneCull sizes them: the split's
## frustum slice (from the previous split's near with blended splits) of the camera's projection (vertical half-angle
## tangent tan_y, aspect), its 8 corners' bounding sphere about their mean, one texel added on each side, texel =
## 2 r / the split's texture size (half the atlas with 2 or 4 splits). [Vector4 texels, Vector4 split far depths].
static func split_texels(range_m: float, near_m: float, tan_y: float, aspect: float) -> Array:
	var count := 4 if splits >= 4 else (2 if splits == 2 else 1)
	var size := float(ATLAS_SIZE) / (2.0 if count > 1 else 1.0)
	var distances := [near_m]
	var fractions := [SPLITS.x, SPLITS.y, SPLITS.z]
	for i in count - 1:
		distances.append(near_m + fractions[i] * (range_m - near_m) if count == 4 else near_m + SPLITS.x * (range_m - near_m))
	distances.append(range_m)
	var texel := Vector4.ZERO
	var far := Vector4(range_m, range_m, range_m, range_m)
	for i in count:
		var n: float = distances[i - 1 if i > 0 else 0]  # blended splits overlap the previous one
		var f: float = distances[i + 1]
		var corners: Array[Vector3] = []
		for z: float in [n, f]:
			var h := tan_y * z
			var w := h * aspect
			for sx: float in [-1.0, 1.0]:
				for sy: float in [-1.0, 1.0]:
					corners.append(Vector3(sx * w, sy * h, -z))
		var center := Vector3.ZERO
		for c in corners:
			center += c
		center /= 8.0
		var radius := 0.0
		for c in corners:
			radius = maxf(radius, center.distance_to(c))
		radius *= size / (size - 2.0)
		texel[i] = 2.0 * radius / size
		far[i] = f
	for i in range(count, 4):
		texel[i] = texel[count - 1]
	return [texel, far]


## The Sun's apparent angular diameter (deg) at sun_distance_m (0.533 deg at 1 AU).
static func solar_disc_diameter_deg(sun_distance_m: float) -> float:
	return rad_to_deg(2.0 * asin(World.SUN_RADIUS_M / maxf(sun_distance_m, World.SUN_RADIUS_M * 2.0)))
