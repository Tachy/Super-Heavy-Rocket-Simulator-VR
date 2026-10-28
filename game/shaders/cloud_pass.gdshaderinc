// The cloud pass's compute kernels (M8 weather and clouds, Task 8): GLSL 450 only - compiled by CloudEffect
// (game/world/weather/cloud_effect.gd) through CloudShaderPort, never included by a Godot shader. One of CLOUD_MARCH,
// CLOUD_TEMPORAL, CLOUD_UPSAMPLE, CLOUD_OUTLINE, CLOUD_EDGE_REFINE, CLOUD_CENTRE or
// CLOUD_SHAPE_PREFILTER is defined per kernel; CLOUD_MARCH also gets atmosphere.gdshaderinc and
// clouds.gdshaderinc (translated: their `global uniform`s in set 0). With CLOUD_CACHE_FILL (cloud view cache Task 3)
// the march's machinery is compiled without its images and main for cloud_view_cache.gdshaderinc's fill kernel; the
// march and the fill shade a ray by the one function cloud_shade_ray. CLOUD_CACHE_LOOKUP (Task 4) gets the CloudView
// block and the ray helpers below for cloud_view_cache.gdshaderinc's per-eye lookup.
//
// The march and the temporal accumulation run once per frame in the centre view (VR: the head, a frustum covering both
// eyes, CloudCentreView; the desktop: the view itself), the upsample once per eye (VR performance part 2: two marches
// with their own jitter and history showed different noise and blur per eye):
// 1. CLOUD_MARCH at reduced resolution (the quality preset's divisor): one ray per low-resolution pixel through the
//    centre of its block of full-resolution pixels; the scene depth of the block in a checkerboard: the nearest of the
//    block on even pixels, the farthest on odd ones (both depths are represented for the upsampling) - unless the
//    block's distances lie within CLOUD_ONE_SURFACE_SPREAD of each other (one surface: the ground seen from far, a
//    slope): then the mean, the centre ray's own distance (Task 10). The raymarch
//    (steps, light, compositing) as described in cloud_layer.gdshader's header. The deterministic march (the default,
//    cloud_deterministic_march; static clouds Task 3): fixed sample points (cloud_march), an unchanged view gives a
//    bit-identical image. The jittered one (the reference path): the start jitter is a blue-noise value per
//    low-resolution pixel (cloud_noise.hpp, 64 x 64 tiled) offset per frame by the golden ratio (a low-discrepancy
//    sequence in time): a function of the pixel and the frame alone - the same for both eyes. Out: premultiplied
//    colour + the background's transmittance T = 1 - alpha (every image of the pass holds T in .a: its half float
//    resolves the direct beam of the Sun's disk to T ~ 1e-7, where 1 - T had lost it below 2.4e-4; every stage is
//    linear in it), and (march distance used, transmittance-weighted cloud distance). With CLOUD_EDGE_REFINE (the
//    deterministic march's outline pass, static clouds Task 5b) the same kernel runs a second time: pixels on an
//    outline get three fixed coverage-only sub-rays besides the centre ray (alpha: the mean of the four, colour: the
//    centre's per alpha), all others are copied.
// 2. CLOUD_TEMPORAL: accumulation with reprojection, per eye: the pixel's point at its cloud distance (the far sky's
//    1e7 m where there is no cloud: rotation only) moved by the camera's motion (Earth-fixed, float64 on the CPU) into
//    the previous frame's clip space of the same eye; the history there (bilinear) is clamped to the current 3 x 3
//    neighbourhood's min / max (no ghosting) and blended with the new value by the weight 1 / n, n the frames the
//    history holds (+1, its age, kept per pixel beside it) down to the preset's floor weight: a running mean that
//    converges (an exponential mean of weight w keeps a noise floor of sqrt(w / (2 - w)) of one frame's noise: 23 % at
//    0.1), with the floor's memory at most. A history the clamp moved (the content changed) keeps at most
//    CLOUD_CLAMPED_AGE frames; no history (first frame, outside the previous view, after a reset): the new value alone.
//    The deterministic march has nothing to average: the weight is 1 (CloudEffect), the kernel copies the frame.
// 3. CLOUD_UPSAMPLE to full resolution, per eye: the desktop's view (its own centre) takes the four nearest
//    low-resolution pixels, bilinear weights x a depth weight exp(-CLOUD_DEPTH_SHARPNESS |ln(D / D_i)|) (D the
//    full-resolution scene distance, D_i the sample's march distance): a cloud in front of a mountain does not bleed
//    over it and the sky between peaks keeps its cloud. A VR eye reprojects its ray into the centre image - at its
//    scene point, then at the taps' cloud distance (the disparity of the cloud) - and gathers a 4 x 4 tent (soft
//    alpha edges from the march's noise, the same in both eyes; radius 1 - bilinear - with the deterministic march)
//    with the same depth weight. Out: the full-resolution
//    image the composite quad (cloud_layer.gdshader) blends over the scene.

#if defined(CLOUD_MARCH) || defined(CLOUD_TEMPORAL) || defined(CLOUD_UPSAMPLE) || defined(CLOUD_OUTLINE) \
		|| defined(CLOUD_CACHE_LOOKUP)

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

// The view's parameters (CloudEffect): matrices of this frame and the previous frame's clip transform of the same eye.
layout(set = 1, binding = 0, std140) uniform CloudView {
	mat4 inv_projection;       // clip (the renderer's: reversed Z in [0, 1], 1 = near; y down) -> view space
	mat4 view_to_world;        // the eye's transform (render world; the floating origin is the camera)
	mat4 world_to_earth;       // render world axes -> Earth-local axes (rotation)
	mat4 prev_clip_from_earth; // previous frame: Earth-local offset from the previous camera -> this eye's clip
	vec4 sizes;                // full width, full height, low width, low height
	vec4 camera_delta;         // xyz: camera now - camera before (Earth-local, m); w: 1 = history valid
	vec4 params;               // x: frame jitter [0, 1), y: temporal weight, z: debug output, w: march pixel angle [rad]
	vec4 eye_earth;            // xyz: the eye's offset from the head (camera), Earth-local axes (m); 0 on the desktop;
	                           // w: the frame index mod 4096 (the sub-pixel shape offsets' sequence)
	vec4 temporal;             // x: the history's clip - 0: the neighbourhood's min / max; > 0: its mean +- x standard
	                           // deviations (VR: noise inside the range keeps the history; CloudLayer.PRESETS)
} cloud_view;

// Distance [m] from the camera (the head) of a pixel's depth (reversed Z: 1 near, 0 far = no geometry) through a
// projection's inverse and its image size.
float cloud_scene_distance_in(mat4 inv_projection, vec2 size, vec2 pixel_centre, float depth) {
	if (depth <= 0.0) {
		return 1.0e30;
	}
	vec2 ndc = pixel_centre / size * 2.0 - 1.0;  // the renderer's projection: y down (flipped)
	vec4 v = inv_projection * vec4(ndc, depth, 1.0);
	return length(v.xyz / v.w);
}

// Distance [m] from the camera of a full-resolution pixel's depth of this view.
float cloud_scene_distance(vec2 pixel_centre, float depth) {
	return cloud_scene_distance_in(cloud_view.inv_projection, cloud_view.sizes.xy, pixel_centre, depth);
}

// Head-space (view) unit direction of the ray through a full-resolution position.
vec3 cloud_ray_view(vec2 pixel_centre) {
	vec2 ndc = pixel_centre / cloud_view.sizes.xy * 2.0 - 1.0;  // the renderer's projection: y down (flipped)
	// The renderer's per-eye projection already contains the eye offset (inverse: clip -> head space), so one unprojected
	// point is not on the eye's ray. Two depths of the same pixel are: their difference is the eye's ray direction.
	vec4 a = cloud_view.inv_projection * vec4(ndc, 0.5, 1.0);
	vec4 b = cloud_view.inv_projection * vec4(ndc, 1.0, 1.0);
	return normalize(a.xyz / a.w - b.xyz / b.w);
}

// Earth-local unit direction of the ray through a full-resolution position.
vec3 cloud_ray_earth(vec2 pixel_centre) {
	vec3 dir_world = normalize(mat3(cloud_view.view_to_world) * cloud_ray_view(pixel_centre));
	return normalize(mat3(cloud_view.world_to_earth) * dir_world);
}

#endif

// =====================================================================================================================
#if defined(CLOUD_OUTLINE) || defined(CLOUD_EDGE_REFINE)
// The outline pass (static clouds Task 5b; CloudEffect runs it after the deterministic march): the deterministic
// march's binary edge decides each march pixel by its one centre ray, so outlines followed the march grid - stairs and
// beads of one march pixel that the upsample only rounded. CLOUD_OUTLINE copies the march's colour and appends the
// outline pixels to a list; CLOUD_EDGE_REFINE (a march variant, one invocation per list entry: full waves of outline
// pixels - spread over the screen's work groups they had kept whole groups waiting) gives each
// CLOUD_EDGE_SUBRAYS coverage-only rays at fixed rotated-grid offsets: the pixel's alpha is the mean of the centre's
// and theirs (covered fraction in quarters), the colour per alpha the centre's - the same in every frame (no jitter,
// no noise pattern), at the extra cost of the outline pixels alone.
layout(set = 1, binding = 6) uniform sampler2D coarse_color;  // the first march's colour + alpha, nearest
// The outline list: the entries' count (reset to 0 by the CPU every frame), the pixels (x | y << 16).
layout(set = 1, binding = 7, std430) buffer CloudOutlineList {
	uint count;
	uint pixels[];
} cloud_outline;
const uint CLOUD_OUTLINE_GROUP_PIXELS = 64u;  // the work group's invocations (8 x 8): one entry each
// [E] Largest alpha difference to a neighbour that keeps a pixel's single ray: one sub-ray's share of the mean (a
// smaller step is not resolved further by four rays).
const float CLOUD_EDGE_ALPHA_STEP = 0.25;

// The largest alpha difference between the march pixel and its 8 neighbours (= of their T in .a); opaque: the most
// opaque of the 3 x 3 (the least T).
float cloud_outline_spread(ivec2 pixel, ivec2 low, vec4 coarse, out vec4 opaque) {
	float spread = 0.0;
	opaque = coarse;
	for (int j = -1; j <= 1; j++) {
		for (int i = -1; i <= 1; i++) {
			vec4 c = texelFetch(coarse_color, clamp(pixel + ivec2(i, j), ivec2(0), low - 1), 0);
			spread = max(spread, abs(c.a - coarse.a));
			if (c.a < opaque.a) {
				opaque = c;
			}
		}
	}
	return spread;
}
#endif

#ifdef CLOUD_OUTLINE
layout(rgba16f, set = 1, binding = 2) uniform writeonly image2D out_color;  // the refined image

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	ivec2 low = ivec2(cloud_view.sizes.zw);
	if (pixel.x >= low.x || pixel.y >= low.y) {
		return;
	}
	vec4 coarse = texelFetch(coarse_color, pixel, 0);
	imageStore(out_color, pixel, coarse);  // the outline pixels' are overwritten by CLOUD_EDGE_REFINE
	vec4 opaque;
	if (cloud_outline_spread(pixel, low, coarse, opaque) > CLOUD_EDGE_ALPHA_STEP) {
		uint entry = atomicAdd(cloud_outline.count, 1u);
		cloud_outline.pixels[entry] = uint(pixel.x) | (uint(pixel.y) << 16);
	}
}
#endif

// =====================================================================================================================
#ifdef CLOUD_MARCH

#ifndef CLOUD_CACHE_FILL
layout(set = 1, binding = 1) uniform sampler2D scene_depth;  // full resolution, nearest
layout(rgba16f, set = 1, binding = 2) uniform writeonly image2D out_color;
#ifdef CLOUD_EDGE_REFINE
layout(rg32f, set = 1, binding = 3) uniform image2D out_depth;  // read too: the first march's cloud distance
#else
layout(rg32f, set = 1, binding = 3) uniform writeonly image2D out_depth;
#endif
#endif
#ifdef CLOUD_EDGE_REFINE
// [E] The centre ray's colour per alpha serves an outline pixel while its alpha is at least half the pixel's mean
// alpha; a fainter centre (a rim it barely touches) takes the most opaque 3 x 3 neighbour's (alpha > 0.25 on any
// outline: CLOUD_EDGE_ALPHA_STEP) - the cloud's colour per alpha beside it.
const float CLOUD_EDGE_COLOUR_ALPHA_RATIO = 0.5;
// The rotated grid with the centre ray as one of its four rays: x and y each take four distinct eighths of a march
// pixel ({-3, 0, 1, 2}/8, the centre's 0 among them; four distinct positions along any near-horizontal or
// near-vertical edge) and the mean of the four rays is the pixel centre (0, 0) in both axes (no shift of the outline).
// The other three's offsets in march pixels.
const int CLOUD_EDGE_SUBRAYS = 3;
const vec2 CLOUD_EDGE_OFFSETS[3] = vec2[3](vec2(-0.375, 0.125), vec2(0.125, 0.25), vec2(0.25, -0.375));
#endif

// [E] Path toward the light from an in-cloud sample in the near (shape) part (cloud_light_march.z: the preset).
// [E] The coarse and fine steps are fractions of the distance from this distance on (nearer: CLOUD_MIN_STEP_M).
const float CLOUD_NEAR_START_M = 1000.0;
// [E] Smallest step (a camera inside a cloud: the mean free path of a thick cumulus is ~15 m).
const float CLOUD_MIN_STEP_M = 10.0;
// Lower end of the shell: below every ground (the Dead Sea shore -430 m); the field holds layer 0 below 0 m.
const float CLOUD_BOTTOM_M = -500.0;
const float CLOUD_MIN_SCALED_TRANSMITTANCE = 0.01;
// [E] The Sun's disk is seen through the cloud at its physical radiance (its centre 8.67e4 on the scene scale, the
// mean 6.9e4: 3e5 times a lit ground's), so the background transmittance of rays toward it is marched on past the
// scaled criterion (which leaves exp(-tau) ~ 5e-4 of the disk) down to CLOUD_SUN_MIN_TRANSMITTANCE (the disk x that is
// < 1e-4 of a lit ground) - the transmittance alone, without light samples. Those rays: within CLOUD_SUN_RAY_ANGLE of
// the Sun (3 x the disk's radius 4.65e-3: the march pixel and the upsample).
const float CLOUD_SUN_MIN_TRANSMITTANCE = 1.0e-9;
const float CLOUD_SUN_RAY_ANGLE = 0.014;
// [E] A coverage-only ray (cloud_march) ends where its background transmittance falls below this: its alpha is then
// known to 0.01 (the scaled transmittance, which ends the full march, decays more slowly by the transport scale).
const float CLOUD_COVERAGE_MIN_TRANSMITTANCE = 0.01;
// [E] A block whose nearest and farthest scene distances differ by less than this (ln of their ratio) shows one
// surface: its centre ray is marched to the block's mean distance. The checkerboard's nearest depth cut that ray short
// by up to the block's depth extent - from 8 600 km a slanted 2 x 2 block of ground spans several km, so every other
// pixel lost the low clouds in front of the ground (Task 10). An occluder's edge (a mountain against the sky or the
// far terrain) spans far more and keeps the checkerboard. 0.05: the upsampling's depth weight
// exp(-CLOUD_DEPTH_SHARPNESS x 0.05) = 0.82 cannot tell such depths apart either.
const float CLOUD_ONE_SURFACE_SPREAD = 0.05;
// [E] Largest weight of one step in the pixel's scattered light, T_sc (1 - exp(-(1 - f) tau_step)): a heavier step is
// marched again with steps of this weight (one point sample - its light march included - would decide the pixel over
// the whole step: grain at cloud surfaces and at bases seen at a grazing angle; with steps of tau ~1 instead the pass
// cost 6 ms more at the descent's worst second). Behind T_sc < this weight any step is fine. 0.3 saved 4.7 / 5.2 ms of
// the VR proxy's cloud pass at the ground / the deck but pushed cloud_far_check's hand-over fade cases to 1.108 / 1.102
// (bound 1.10): the shapes' radiance dropped; 0.15 keeps them at 1.094 / 1.085.
const float CLOUD_MAX_STEP_WEIGHT = 0.15;
// [E] The air's light at the cloud is re-evaluated along a ray after 500 m or 5 % of the distance (the transmitted
// Sun changes by < 1 % over that at a 10-deg Sun; the eclipse's shadow edge is ~50 km wide).
const float CLOUD_LIGHT_REFRESH_M = 500.0;
const float CLOUD_LIGHT_REFRESH_FRACTION = 0.05;
// [E] The eclipse's factor on the sky light (atm_sky_eclipse_factor: costly - evaluated at every light refresh it took
// the descent's p99 from 14.5 to 25 ms): once per 8 x 8 work group at two points of the group's centre ray, 5 km and
// 150 km away, linear in the distance between them (held beyond). The penumbra spans ~3500 km from first contact to
// totality (~3 % of obscuration per 100 km), the umbra ~200 km: neighbouring pixels share the factor.
const float CLOUD_ECLIPSE_NEAR_M = 5000.0;
const float CLOUD_ECLIPSE_FAR_M = 150000.0;
// [E] Extinction where a silhouette begins (deterministic march): below it a fine step (<= 1 km) carries tau < 0.1, so
// the clear stretch skipped in front of the refined entry holds no visible cloud; a cumulus' rim rises to its in-cloud
// ~0.03-0.1 /m within tens of metres, so the entry sits at the visible outline. Thin cloud below it (cirrus, wisps:
// ~1e-5-1e-4 /m) is marched as before, translucent by its density.
const float CLOUD_SILHOUETTE_EXTINCTION_PER_M = 1.0e-4;
// [E] Bisections of the entry bracket (the step's first half, start to midpoint): 1/32 of the step (the presets' fine
// step 1.2 % of the distance: 0.4 mrad, below the VR centre's 1.8 mrad pixel).
const int CLOUD_SILHOUETTE_BISECTIONS = 4;
// [E] The deterministic march's detail band limit as a fraction of its step: detail level k enters from cells l_k of
// twice this times the step and is full from four times (cloud_detail_state: l_k >= 2 f / 4 f), i.e. from one / two
// steps per cell - the midpoint rule's resolution along the ray. Finer detail sampled at fixed points aliased into
// static salt-and-pepper holes in near cumuli (the jitter's temporal mean had averaged it; Task 3 images).
const float CLOUD_DETAIL_STEP_FRACTION = 0.5;
// [E] Smallest |dh/ds| for the step phase's height shells (a horizontal ray: shells 1e-3 of the step apart, the phase
// a function of the ray alone as for any other ray).
const float CLOUD_PHASE_MIN_CLIMB = 1.0e-3;
// [E] The deterministic march's longest interval between samples of the shapes (static clouds Task 5c), as a fraction
// of the distance over which the ray crosses one texel of the finest shape in the cell (cloud_structure_step: cumulus
// 250 m across / 125 m high, stratiform 1 km / 125 m, cirrus 250 m / 16 m): a longer step - the fine step is 2.5 % of
// the distance (VR), 500 m at 20 km - takes the optical depth of equal sub-steps, each sampled at its midpoint in its
// own field cell; one light sample per step, at the sub-samples' extinction-weighted centre. One sample per step hit or
// skipped thin and broken cloud on long oblique paths with the ray's fixed step phase - static salt-and-pepper at the
// march pixel that the jittered march's history had averaged (the user's t7_cape; gone with 8 x the steps;
// cloud_grain_check.gd). A quarter texel (a horizontal ray through cumulus: every 62.5 m): with half a texel (and the
// detail band limit continuous in the step, fix round 1) the ground view lost 2.4 % of its cover against the 8 x
// reference (RMS difference 0.13), the deck 2.7 %; a whole texel left the Cape's grain at 1.36 x the reference's.
const float CLOUD_STRUCTURE_TEXEL_FRACTION = 0.25;
// [E] At most this many sub-samples per step (longer steps sample at 1 / CLOUD_MAX_SUBSTEPS of the step): 8 held the
// deck view and the cirrus within 0.8 % of the 8 x reference's alpha, 4 within 1.4 % / 1.7 % but left the Cape's alpha
// grain at 1.375 x the reference's (bound 1.25, cloud_grain_check.gd); 5: 1.241 (no margin), 6: 1.16-1.17. 6 saves
// 5.4 / 4.2 ms of the VR proxy's cloud pass at the ground / the deck (4: 9.3 / 8.9; crops t10b_substeps_*).
const int CLOUD_MAX_SUBSTEPS = 6;
shared vec3 cloud_group_eclipse_near;
shared vec3 cloud_group_eclipse_far;

global uniform sampler2D cloud_blue_noise : filter_nearest, repeat_enable;  // 64 x 64 R8 (cloud_noise.hpp)
global uniform int cloud_steps;        // coarse steps spanning the segment (quality preset)
global uniform int cloud_max_steps;    // cap on the steps per pixel (coarse + fine)
global uniform float cloud_fine_step_fraction;  // fine step / distance inside the field's cloudy region
global uniform bool cloud_deterministic_march;  // fidelity flag (CloudLayer.deterministic_march; false: jittered)
// The near / far hand-over (static clouds Task 8; CloudLayer.PRESETS "handover_m"): the march ends at
// cloud_handover_far_m, the far tier (cloud_march's pass 1, the column model cloud_far_column at large footprints)
// takes the ray from cloud_handover_near_m on, blended between.
global uniform float cloud_handover_near_m;
global uniform float cloud_handover_far_m;

// The far tier's shapes (cloud_march's pass 1) fade to the column model between these lateral pixel footprints
// (x start, y end [m]; CloudLayer.PRESETS "far_fade_m") at the ray's reference distance (cloud_far_reference_m): the
// shape's mips stop at 1 km texels (CLOUD_SHAPE_PREFILTER_LEVELS), so beyond ~3.5 km a point sample of them beats with
// the 32 km tile (Task 3's follow-up, spec decision 4) - the speckled band along the limb and the far horizon. The
// footprint is the lateral one (no |cos| of the view: at a grazing view the pixel spans km along the layer). The
// checks: 1e9, 1e9 the shapes everywhere, 0, 0 the column model.
global uniform vec2 cloud_far_pattern_fade_m;
// [E] Seen from above the footprint counts this many times (up to it at a vertical view, x |cos| x it, at least x 1): a
// steep ray crosses the layer within its footprint, so a pixel is one point of the thresholded shapes (from low orbit
// at ~1 km footprints their cumuli flickered per pixel into grain), while a grazing ray averages them over kilometres
// of its path through the layer.
const float CLOUD_FAR_STEEP_FOOTPRINT_FACTOR = 3.0;
// [E] The far tier's light path through the shapes (cloud_far_light_depth): samples, first interval and the intervals'
// ratio - 20 / 52 / 137 / 358 / 936 m, 1.5 km in all (a cumulus' width, so a sample on a sunlit flank is lit through
// its flank, not through the column above it; the first interval the volume's light march's, CloudLayer.PRESETS
// light_first_m; 3 intervals lit a far deck 7 % dark).
const int CLOUD_FAR_LIGHT_SAMPLES = 5;
const float CLOUD_FAR_LIGHT_FIRST_M = 20.0;
const float CLOUD_FAR_LIGHT_RATIO = 2.615;
// [E] The far tier's first guess of the slab's height where the column of its first guess is clear: a low cloud's
// (the next guess takes the column where the ray crosses it).
const float CLOUD_FAR_GUESS_HEIGHT_M = 2000.0;
// [E] The column model's sub-grid cover (cloud_far_cover): taps across the footprint (twice as many along it: 32
// shape reads per pixel, once; 64 with the unaligned reading), the footprint's widening (2: the average spans two pixels, no march-pixel blocks), its
// largest extent along a grazing view (beyond it the average is the layer's mean: the 32 km tile), and the footprints
// over which it fades to the even cover (from a quarter of the tile on, its period would show as a lattice; the
// scaled-space far field beyond 1.5e7 m, with ~17 km pixels, draws the even cover).
const int CLOUD_FAR_COVER_TAPS = 4;
const float CLOUD_FAR_COVER_FOOTPRINT_SCALE = 2.0;
// [E] From footprints of a few km the 32 km tile's period shows as a regular lattice over the hundreds of km of a view
// from orbit: the sub-grid cover mixes in (up to this weight, faded in over these footprints) a second reading of the
// shape, Earth-fixed in a frame turned by 37 deg about (1, 2, 3) and scaled by an irrational-ish 1.37 (a horizontal
// slice of the isotropic noise has the same statistics: the same thresholds and edge mean), whose period does not line
// up with the first.
const float CLOUD_FAR_COVER_UNALIGNED_WEIGHT = 0.5;
const vec2 CLOUD_FAR_COVER_UNALIGNED_M = vec2(1000.0, 3000.0);
const float CLOUD_FAR_COVER_UNALIGNED_SCALE = 1.37;
const mat3 CLOUD_FAR_COVER_UNALIGNED_ROTATION = mat3(0.8130186879, 0.5112918472, -0.2785341274, -0.4537591358,
		0.8561682215, 0.2471408976, 0.3648331945, -0.0745427634, 0.9280841107);
const float CLOUD_FAR_COVER_ALONG_MAX_M = 32000.0;
const vec2 CLOUD_FAR_COVER_FADE_M = vec2(8000.0, 16000.0);
// [E] Behind a march transmittance below this the far tier is not evaluated (< 0.1 % of its light).
const float CLOUD_FAR_MIN_TRANSMITTANCE = 1.0e-3;

// The ray dir (Earth-local unit) from a point offset from the camera (Earth-local, m). The cache fill
// (CLOUD_CACHE_FILL: the anchor, metres to kilometres from the camera): the ray's height is the camera's plus the
// offset's along its own direction (the cloud_height_along form: exact in float32 where |M p| - a would lose ~0.5 m),
// its up the anchor's model up. The per-frame march: the camera's ray (cloud_ray) - its origin is an eye at most a few
// centimetres from the head (cloud_vr_stereo_test), the head's height and up; kept apart at compile time so the
// march's code stays bit-identical (a run-time branch on the offset let the GPU compiler re-associate the camera's
// path: 4 % of the pixels changed, up to 30 % of the mean colour).
CloudRay cloud_ray_from(vec3 dir_local, vec3 origin_offset) {
	CloudRay ray = cloud_ray(dir_local);
#ifdef CLOUD_CACHE_FILL
	float offset_m = length(origin_offset);
	if (offset_m > 0.0) {
		ray.h_c = cloud_height_along(cloud_ray(origin_offset / offset_m), offset_m);
		ray.r_c = CLOUD_A_M + ray.h_c;
		vec3 up_o = normalize((atmosphere_camera_hi + atmosphere_camera_lo + origin_offset) * ATM_STRETCH);
		ray.mu = dot(up_o, dir_local * ATM_STRETCH / ray.scale);
	}
#endif
	return ray;
}

// Scattering source of one light (Sun or Moon) at a sample: J per unit irradiance normal to the light. The Sun's
// optical depth from the light cache's cascades where they hold the sample (cloud_light_cache.gdshaderinc; the hybrid
// ones plus the near part of the path marched through the shapes), blended into the light march over the outermost
// band; elsewhere - and always for the Moon - the light march (one call site: where the cache holds nothing the result
// is the march's, bit for bit).
float cloud_light_source(vec3 light_dir, vec3 normal, vec3 dir, vec3 offset, float height_m, float distance_m,
		vec2 column, float jitter, float pixel_angle, bool sun) {
	vec4 near_w = vec4(0.0);
	vec3 cached = sun ? cloud_cached_light_depth(offset, height_m, light_dir, near_w) : vec3(0.0, 0.0, 1.0);
	float tau_light = cached.x;
	if (cached.y > 0.0) {
		tau_light += cloud_near_light_depth(offset, height_m, distance_m, light_dir, pixel_angle, near_w);
	}
	if (cached.z > 0.0) {
		float marched = cloud_light_optical_depth(offset, height_m, distance_m, light_dir, jitter, pixel_angle,
				pixel_angle);
		tau_light = cached.z >= 1.0 ? marched : tau_light + cached.z * marched;
	}
	if (cloud_scattering_model == 1) {
		return cloud_scattering_source_mc(light_dir, -dir, normal, tau_light, column.x, column.y);
	}
	return cloud_scattering_source_3d(light_dir, -dir, normal, tau_light, column.x, column.y);
}

// The far tier's weight at distance d (0: the march alone, 1: the far tier alone): smoothstep over the hand-over band;
// a step at cloud_handover_far_m for an empty band (the checks: 0 / 0 = the far tier everywhere, both beyond the scene
// = the march everywhere).
float cloud_handover_weight(float d) {
	if (cloud_handover_far_m <= cloud_handover_near_m) {
		return d >= cloud_handover_far_m ? 1.0 : 0.0;
	}
	return smoothstep(cloud_handover_near_m, cloud_handover_far_m, d);
}

// The far slab's heights for the column at lat_lon (x the grid-mean depth, y the largest fraction) seen at the view's
// |cosine| mu: x from below (the depth from its base reaching cloud_far_seen_depth), y from above (from its top).
vec2 cloud_far_slab_heights(vec2 lat_lon, vec2 column, float mu) {
	float seen = cloud_far_seen_depth(column, mu);
	float from_above = cloud_far_depth_height(lat_lon, seen);
	float from_below = cloud_far_depth_height(lat_lon, max(column.x - seen, 0.0));
	return vec2(min(from_below, from_above), from_above);
}

// The far tier's column part (static clouds Task 8; at large pixel footprints, cloud_far_pattern_fade_m): the column
// model of the far field (cloud_far.gdshaderinc) for the ray beyond the hand-over's start from_m up to to_m (the scene
// or the cloud shell). The slab is placed as in the far field - at the height where the column's depth reaches the
// seen light's mean depth (from above: from its top; from below: from its base) - and found along the ray: a first
// guess of its column (the surface the ray meets when it has one, the far field's ground point; else its lowest point
// in the segment), then twice the slab's crossing with the column there: the ray at from_m above the slab descends to
// it, below it climbs to it, inside the slab's heights it is taken where it leaves them (grazing); a crossing in front
// of from_m (a higher layer draws the slab up) is taken at from_m. Out: the crossing's distance (< 0: none) and its
// column (x the grid-mean depth, y the largest fraction).
float cloud_far_slab(CloudRay ray, vec3 dir, vec3 eye, float from_m, float to_m, bool surface, out vec2 column_p) {
	column_p = vec2(0.0);
	if (to_m <= from_m) {
		return -1.0;
	}
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	float guess_m = surface ? to_m : clamp(-ray.r_c * ray.mu / ray.scale, from_m, to_m);
	vec3 q = camera + eye + dir * guess_m;
	vec2 lat_lon = cloud_lat_lon(q);
	vec2 column = cloud_column_mean_depth(lat_lon, 0.0);
	float h_from = cloud_height_along(ray, from_m);
	float d_p = -1.0;
	for (int i = 0; i < 2; i++) {
		vec2 heights = vec2(CLOUD_FAR_GUESS_HEIGHT_M);
		if (column.x > 0.0) {
			heights = cloud_far_slab_heights(lat_lon, column, max(abs(dot(dir, cloud_geodetic_up(q))),
					CLOUD_FAR_MU_MIN));
		}
		float d = -1.0;
		if (h_from >= heights.y) {
			vec2 crossing = cloud_sphere_crossing(ray, heights.y);
			d = crossing.x <= crossing.y ? crossing.x : -1.0;  // descending to the slab seen from above
		} else if (h_from <= heights.x) {
			vec2 crossing = cloud_sphere_crossing(ray, heights.x);
			d = crossing.x <= crossing.y ? crossing.y : -1.0;  // climbing to the slab seen from below
		} else {
			// Inside the slab's heights at from_m: where the ray leaves them (climbing: the top, else the base).
			bool climbing = cloud_height_slope(ray, from_m) >= 0.0;
			vec2 crossing = cloud_sphere_crossing(ray, climbing ? heights.y : heights.x);
			d = crossing.x <= crossing.y ? (climbing ? crossing.y : crossing.x) : -1.0;
		}
		if (!(d >= 0.0 && d < to_m)) {
			break;
		}
		d = max(d, from_m);
		q = camera + eye + dir * d;
		lat_lon = cloud_lat_lon(q);
		column = cloud_column_mean_depth(lat_lon, 0.0);
		d_p = d;
		column_p = cloud_column_mean_depth_smooth(lat_lon, 0.0);  // the drawn column: smooth (no cell lattice)
	}
	return column_p.x > 0.0 ? d_p : -1.0;
}

// The distance at which the far tier's shapes fade to the column model (cloud_far_pattern_fade_m, the pixel footprint
// there): where the ray crosses CLOUD_FAR_GUESS_HEIGHT_M (descending from above, climbing from below; its lowest point
// when it stays above), within [from_m, to_m] - continuous over the pixels (the slab's crossing, from the column there,
// jumped between neighbouring pixels: rectangular steps in the far horizon's band).
float cloud_far_reference_m(CloudRay ray, float from_m, float to_m) {
	vec2 crossing = cloud_sphere_crossing(ray, CLOUD_FAR_GUESS_HEIGHT_M);
	float d;
	if (ray.h_c < CLOUD_FAR_GUESS_HEIGHT_M) {
		d = crossing.y;
	} else {
		d = crossing.x <= crossing.y ? crossing.x : -ray.r_c * ray.mu / ray.scale;
	}
	return clamp(d, from_m, to_m);
}

// The column model's sub-grid cover (static clouds Task 8 fix rounds 1 and 2; spec decision 4: a deterministic
// footprint average of the mip-based cover) of a column whose largest fraction is fraction and whose even cover along
// the view is even (the beam cover, cloud_beam_cover_of): the share of fixed taps over the pixel's footprint on the
// layer where the shapes' prefiltered value exceeds the prefilter threshold of the even cover - the shapes' pattern
// covering the share even of the layer, so the mean over the layer is the even cover's (spec decision 3: the cover and
// the water stay the grid's at every footprint; a share of the fraction's threshold scaled by even / fraction and
// clamped at 1 lost 9-15 % of the cover where whole footprints were covered); the even cover for a deck. The footprint
// is widened by
// CLOUD_FAR_COVER_FOOTPRINT_SCALE (a box of one pixel still aliases): footprint_m across the view, footprint_m / mu
// along its horizontal direction (at most CLOUD_FAR_COVER_ALONG_MAX_M), CLOUD_FAR_COVER_TAPS across and twice as many
// along, read at the mip level of the larger spacing; from CLOUD_FAR_COVER_UNALIGNED_M's footprints on a share comes
// from the shape's second, unaligned reading. It fades to the even cover between CLOUD_FAR_COVER_FADE_M's footprints
// (the 32 km tile's own period would show as a lattice) and is the even cover where the cell at the point holds no
// cloud (its types).
float cloud_far_cover(vec3 offset_local, float height_m, vec3 dir, vec3 up, float mu, float footprint_m,
		float fraction, float even) {
	float fade = 1.0 - smoothstep(CLOUD_FAR_COVER_FADE_M.x, CLOUD_FAR_COVER_FADE_M.y, footprint_m);
	CloudCell cell = cloud_field(cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset_local), height_m);
	if (fade <= 0.0 || fraction >= 1.0 || even >= 1.0 || cell.fraction < CLOUD_MIN_FRACTION
			|| cell.extinction <= 0.0) {
		return even;
	}
	vec3 along = dir - up * dot(dir, up);
	float along_norm = length(along);
	vec3 t1 = along_norm > 1.0e-6 ? along / along_norm
			: normalize(cross(up, abs(up.y) < 0.9 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0)));
	vec3 t2 = cross(up, t1);
	float across_m = footprint_m * CLOUD_FAR_COVER_FOOTPRINT_SCALE;
	float along_m = min(across_m / mu, max(CLOUD_FAR_COVER_ALONG_MAX_M, across_m));
	const int taps_along = 2 * CLOUD_FAR_COVER_TAPS;
	// cloud_far_level of the taps' larger spacing
	float spacing_m = max(along_m / float(taps_along), across_m / float(CLOUD_FAR_COVER_TAPS));
	float level = clamp(log2(spacing_m * CLOUD_SHAPE_TEXELS / CLOUD_SHAPE_TILE_H_M), 0.0, CLOUD_SHAPE_PREFILTER_LEVELS);
	vec2 uv = cloud_prefilter_uv(even, level);
	float threshold = texture(cloud_prefilter_threshold, uv).r;
	vec3 types = cloud_type_weights(cell);
	// The second, unaligned reading's weight (CLOUD_FAR_COVER_UNALIGNED_*).
	float unaligned = CLOUD_FAR_COVER_UNALIGNED_WEIGHT
			* smoothstep(CLOUD_FAR_COVER_UNALIGNED_M.x, CLOUD_FAR_COVER_UNALIGNED_M.y, footprint_m);
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	float sum = 0.0;
	for (int j = 0; j < CLOUD_FAR_COVER_TAPS; j++) {
		for (int i = 0; i < taps_along; i++) {
			vec2 u = (vec2(i, j) + 0.5) / vec2(taps_along, CLOUD_FAR_COVER_TAPS) - 0.5;
			vec3 q = offset_local + t1 * (u.x * along_m) + t2 * (u.y * across_m);
			float covered = 0.0;
			for (int type = 0; type < 3; type++) {
				if (types[type] > 0.0) {
					vec4 s = cloud_shape_prefiltered(cloud_type_shape_coordinates(type, q, height_m), level);
					covered += types[type] * step(threshold, s.r);
				}
			}
			if (unaligned > 0.0) {
				vec4 s = cloud_shape_prefiltered(CLOUD_FAR_COVER_UNALIGNED_ROTATION * (camera + q)
						/ (CLOUD_SHAPE_TILE_H_M * CLOUD_FAR_COVER_UNALIGNED_SCALE), level);
				covered = mix(covered, step(threshold, s.r), unaligned);
			}
			sum += covered;
		}
	}
	return mix(even, sum / float(taps_along * CLOUD_FAR_COVER_TAPS), fade);
}

// The column model at the slab's crossing d_p (cloud_far_slab): the even cover C_v (the beam cover of the column's
// largest fraction at the view's cosine) distributed by the sub-grid cover at the pixel's footprint footprint_m
// (cloud_far_cover; its mean C_v) with the even cover's in-cloud depth (the water follows the cover), the
// plane-parallel slab of the cloudy part's depth (cloud_far_cloudy_radiance: reflected seen from above, transmitted from below), its
// air light at the crossing; premultiplied colour + alpha (coverage_only: alpha alone).
vec4 cloud_far_column(vec3 dir, vec3 eye, float d_p, vec2 column_p, float footprint_m, vec3 eclipse_near,
		vec3 eclipse_far, bool coverage_only) {
	vec3 offset = eye + dir * d_p;
	vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + offset;
	vec3 up = cloud_geodetic_up(p);
	vec3 view_prop = -dir;
	float mu = max(abs(dot(view_prop, up)), CLOUD_FAR_MU_MIN);
	float even = max(cloud_column_beam_cover(column_p, mu), CLOUD_MIN_FRACTION);
	float tau = column_p.x / even;  // the cloudy part's vertical depth
	float cover = cloud_far_cover(offset, cloud_height_of(p), dir, up, mu, footprint_m, column_p.y, even);
	float alpha = cover * (1.0 - exp(-tau / mu));
	if (coverage_only) {
		return vec4(0.0, 0.0, 0.0, alpha);
	}
	float eclipse_x = clamp((d_p - CLOUD_ECLIPSE_NEAR_M) / (CLOUD_ECLIPSE_FAR_M - CLOUD_ECLIPSE_NEAR_M), 0.0, 1.0);
	vec3 radiance = cloud_far_cloudy_radiance(p, up, view_prop, tau, column_p.y, mix(eclipse_near, eclipse_far,
			eclipse_x));
	vec3 air_transmittance;
	vec3 air_light = atm_to_point_offset(offset, air_transmittance);
	return vec4(air_transmittance * radiance * cover + alpha * air_light, alpha);
}

// The far tier's extinction [1/m] (unscaled) at camera + offset_local: up to one shape texel of band limit the
// volume's own (cloud_extinction_in_cell: the shape and the detail band-limited at detail_m - the lobes the march
// draws there), beyond it the shape from its box-filtered mips at the level (cloud_extinction_prefiltered; the detail
// is gone at that band limit, continuous at level 0; deterministic, no sub-pixel offset - spec decision 4); 0 outside
// cloud.
float cloud_far_extinction_at(vec3 offset_local, float height_m, CloudCell cell, float detail_m, float band_m,
		float level) {
	if (cell.fraction < CLOUD_MIN_FRACTION || cell.extinction <= 0.0) {
		return 0.0;
	}
	if (level <= 0.0) {
		float in_cloud;
		return cloud_extinction_in_cell(cell, offset_local, height_m, detail_m, 0.0, in_cloud);
	}
	return cloud_extinction_prefiltered(cell, offset_local, height_m, band_m, level);
}

// The far tier's optical depth toward a light from camera + offset_local (height_m): CLOUD_FAR_LIGHT_SAMPLES
// geometric intervals (ratio CLOUD_FAR_LIGHT_RATIO, about 1.5 km in all) from CLOUD_FAR_LIGHT_FIRST_M on (as the
// volume's light march: the
// cloud right around the sample decides its source's depth - equal intervals put the first sample 250 m out, above a
// deck's top, and lit the far deck as its top: 25 % dark) through the far tier's shapes (the lit and shaded flanks of
// the far cumuli), the rest the column's grid-mean depth above that point along the light (plane-parallel; the light
// march's far part is the grid mean too). No light cache this far (ruling of Task 2: its cascades end the march).
float cloud_far_light_depth(vec3 offset_local, float height_m, vec3 light_dir, float detail_m, float band_m,
		float level) {
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	float tau = 0.0;
	float start = 0.0;
	float length_i = CLOUD_FAR_LIGHT_FIRST_M;
	for (int i = 0; i < CLOUD_FAR_LIGHT_SAMPLES; i++) {
		vec3 q = offset_local + light_dir * (start + 0.5 * length_i);
		float h = cloud_height_of(camera + q);
		CloudCell cell = cloud_field(cloud_lat_lon(camera + q), h);
		tau += cloud_far_extinction_at(q, h, cell, max(detail_m, length_i), band_m, level) * length_i;
		start += length_i;
		length_i *= CLOUD_FAR_LIGHT_RATIO;
	}
	vec3 end = camera + offset_local + light_dir * start;
	float mu_l = max(dot(light_dir, cloud_geodetic_up(end)), CLOUD_ENTRY_COSINE_MIN);
	return tau + cloud_column_mean_depth(cloud_lat_lon(end), cloud_height_of(end)).x / mu_l;
}

// The level of the shape's box-filtered mips whose texel is the band limit band_m (0: the shape itself).
float cloud_far_level(float band_m) {
	return clamp(log2(band_m * CLOUD_SHAPE_TEXELS / CLOUD_SHAPE_TILE_H_M), 0.0, CLOUD_SHAPE_PREFILTER_LEVELS);
}

// A sample's extinction [1/m] (unscaled) at camera + offset_local in its field cell: the march's (far false:
// cloud_extinction_in_cell - the shape prefiltered at the pixel footprint pixel_m only beyond f_s, in the prefilter
// kernel) or the far tier's (true: cloud_far_extinction_at - the shape band-limited at the pixel footprint, beyond a
// shape texel its mips); the detail band-limited at detail_m. in_cloud: the cell's in-cloud mean extinction.
float cloud_sample_extinction_in_cell(bool far, CloudCell cell, vec3 offset_local, float height_m, float detail_m,
		float pixel_m, out float in_cloud) {
	if (!far) {
		return cloud_extinction_in_cell(cell, offset_local, height_m, detail_m, pixel_m, in_cloud);
	}
	in_cloud = cell.fraction >= CLOUD_MIN_FRACTION && cell.extinction > 0.0 ? cell.extinction / cell.fraction : 0.0;
	return cloud_far_extinction_at(offset_local, height_m, cell, detail_m, pixel_m, cloud_far_level(pixel_m));
}

// The same at camera + offset_local with its field cell looked up; out its (lat, lon).
float cloud_sample_extinction(bool far, vec3 offset_local, float height_m, float detail_m, float pixel_m,
		out vec2 lat_lon, out float in_cloud) {
	lat_lon = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset_local);
	return cloud_sample_extinction_in_cell(far, cloud_field(lat_lon, height_m), offset_local, height_m, detail_m,
			pixel_m, in_cloud);
}

// A sample's source of one light (cloud_light_source's J): the march's (far false: the light cache, the light march)
// or the far tier's (true: no cache that far, ruling of Task 2 - the optical depth toward the light from
// cloud_far_light_depth, the shapes band-limited at the pixel footprint there).
float cloud_sample_light_source(bool far, vec3 light_dir, vec3 normal, vec3 dir, vec3 offset, float height_m,
		float distance_m, vec2 column, float jitter, float pixel_angle, bool sun) {
	if (!far) {
		return cloud_light_source(light_dir, normal, dir, offset, height_m, distance_m, column, jitter, pixel_angle,
				sun);
	}
	float band_m = pixel_angle * distance_m;
	float tau_light = cloud_far_light_depth(offset, height_m, light_dir, band_m, band_m, cloud_far_level(band_m));
	if (cloud_scattering_model == 1) {
		return cloud_scattering_source_mc(light_dir, -dir, normal, tau_light, column.x, column.y);
	}
	return cloud_scattering_source_3d(light_dir, -dir, normal, tau_light, column.x, column.y);
}

// The deterministic march's structure step in a cell for a ray climbing at climb (|dh/ds|):
// CLOUD_STRUCTURE_TEXEL_FRACTION of the distance over which the ray crosses one texel of the finest shape present
// (the types' tiles, cloud_type_shape_coordinates; a sheet of cirrus is 16 m per texel high, so a slant ray through it
// samples far more finely than through cumulus).
float cloud_structure_step(CloudCell cell, float climb) {
	vec3 types = cloud_type_weights(cell);
	float across_texel_m = (types[CLOUD_TYPE_CUMULUS] > 0.0 || types[CLOUD_TYPE_CIRRUS] > 0.0 ? CLOUD_SHAPE_TILE_H_M
			: CLOUD_SHAPE_TILE_H_M * CLOUD_STRATIFORM_STRETCH) / CLOUD_SHAPE_TEXELS;
	float up_texel_m = (types[CLOUD_TYPE_CIRRUS] > 0.0 ? CLOUD_ICE_TILE_V_M : CLOUD_SHAPE_TILE_V_M) / CLOUD_SHAPE_TEXELS;
	float across = sqrt(max(1.0 - climb * climb, 0.0));
	return CLOUD_STRUCTURE_TEXEL_FRACTION / max(climb / up_texel_m, across / across_texel_m);
}

// The raymarch of one ray (dir: Earth-local unit; from origin: the eye's or the cache anchor's offset from the camera,
// Earth-local, m) up to scene_m; out: premultiplied colour, alpha, weighted distance,
// and the background's transmittance T = 1 - alpha computed without the cancellation of 1 - alpha (what the image
// stores in .a: a half float resolves T relatively, the Sun's disk down to a slant tau of ~17).
// The deterministic march (cloud_deterministic_march; static clouds Task 3): the first step ends on an Earth-fixed
// height shell (CLOUD_PHASE_MIN_CLIMB), every sample - the view's and the light march's - at its step's midpoint
// (jitter 0.5), and where a sample's extinction crosses CLOUD_SILHOUETTE_EXTINCTION_PER_M from below the entry point
// is bisected (CLOUD_SILHOUETTE_BISECTIONS) between the step's start and the sample, and the in-cloud steps start there
// (the outline found, not sampled: a whole step's depth at a cloud's rim made the silhouette a step's width too big);
// a clear sample whose step ends in the field's cloudy region ends the step there (fine steps follow), and the detail
// is band-limited at the step too (CLOUD_DETAIL_STEP_FRACTION); a step longer than the structure step samples the
// shapes at sub-steps (cloud_structure_step, Task 5c).
// coverage_only (the outline pass's sub-rays, Task 5b fix round 1): the same steps and transmittance without the light
// samples and the air light, ended at CLOUD_COVERAGE_MIN_TRANSMITTANCE (alpha within 0.01 of the full march's, the
// cloud distance nearly its); the colour stays 0.
// The near / far hand-over (static clouds Task 8): two passes of these steps - the march up to cloud_handover_far_m
// with its light cache, the far tier from cloud_handover_near_m to the segment's end with the shapes band-limited at the
// pixel footprint and the far light path - each step weighted by its pass's hand-over weight; composited below.
void cloud_march(vec3 dir, vec3 origin, float scene_m, float jitter, vec3 eclipse_near, vec3 eclipse_far,
		bool coverage_only, out vec3 color, out float alpha, out float transmittance, out float cloud_m) {
	color = vec3(0.0);
	alpha = 0.0;
	transmittance = 1.0;
	cloud_m = 0.0;
	CloudRay ray = cloud_ray_from(dir, origin);
	if (cloud_debug_output == 3) {
		vec2 crossing = cloud_sphere_crossing(ray, cloud_debug_height_m);
		float d_h = crossing.x > 0.0 ? crossing.x : crossing.y;
		if (crossing.x > crossing.y || d_h <= 0.0) {
			return;
		}
		vec3 offset_h = origin + dir * d_h;
		vec2 lat_lon_h = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset_h);
		CloudCell cell = cloud_field(lat_lon_h, cloud_debug_height_m);
		color = vec3(cloud_coverage(cloud_shape_coordinates(offset_h, cloud_debug_height_m), cell.fraction));
		alpha = 1.0;
		transmittance = 0.0;
		return;
	}
	vec2 top = cloud_sphere_crossing(ray, CLOUD_TOP_M);
	if (top.x > top.y || top.y <= 0.0) {
		return;
	}
	float d0 = max(top.x, 0.0);
	float d1 = top.y;
	vec2 bottom = cloud_sphere_crossing(ray, CLOUD_BOTTOM_M);
	if (bottom.x <= bottom.y) {
		if (ray.h_c >= CLOUD_BOTTOM_M) {
			if (bottom.x > 0.0) {
				d1 = min(d1, bottom.x);
			}
		} else {
			d0 = max(d0, bottom.y);
		}
	}
	bool surface = scene_m < d1;  // the ray ends on the scene, not on the cloud shell
	d1 = min(d1, scene_m);
	if (d1 <= d0) {
		return;
	}
	// The near / far hand-over (static clouds Task 8): the march ends at cloud_handover_far_m, the far tier takes the
	// rest (after the loop). The debug outputs (the checks' physical quantities of the volume) march the whole segment;
	// the air shadow's 9-12 check the game's image (the hand-over kept).
	bool handover = cloud_debug_output == 0 || cloud_debug_output >= 9;
	float end_m = d1;
	if (handover) {
		d1 = min(d1, cloud_handover_far_m);
	}

	float k = cloud_transport_scale();
	vec3 sun_dir = normalize(atmosphere_sun_direction);
	bool sun_ray = dot(dir, sun_dir) > cos(CLOUD_SUN_RAY_ANGLE);  // the Sun's disk is behind this ray
	vec3 moon_dir = normalize(atmosphere_moon_direction);
	bool moonlit = atm_moonlit();
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	vec3 eye = origin;  // each eye (the cache: its anchor) marches from its own position
	float pixel_angle = cloud_view.params.w;  // the detail's band limit: footprint = pixel_angle x distance
	float w_tab = cloud_phase.w;
	// The far tier (static clouds Task 8): from the band's start to the segment's end; its shapes fade to the far
	// field's column model where the pixel footprint outgrows their mips (cloud_far_pattern_fade_m, at the ray's
	// reference distance cloud_far_reference_m: continuous over the pixels).
	float far_from_m = max(d0, cloud_handover_near_m);
	float shapes = 1.0;
	if (handover && pixel_angle * end_m * CLOUD_FAR_STEEP_FOOTPRINT_FACTOR > cloud_far_pattern_fade_m.x) {
		float reference_m = cloud_far_reference_m(ray, far_from_m, end_m);
		float steep = abs(dot(dir, cloud_geodetic_up(camera + eye + dir * reference_m)));
		float footprint_m = pixel_angle * reference_m * max(1.0, steep * CLOUD_FAR_STEEP_FOOTPRINT_FACTOR);
		shapes = cloud_far_pattern_fade_m.y > cloud_far_pattern_fade_m.x
				? 1.0 - smoothstep(cloud_far_pattern_fade_m.x, cloud_far_pattern_fade_m.y, footprint_m)
				: (footprint_m >= cloud_far_pattern_fade_m.y ? 0.0 : 1.0);
	}
	// Two passes over the same steps (the march's machinery: coarse steps spanning the segment, fine steps in the
	// field's cloudy region that still reach its end, sub-samples, silhouette entries, dense refinement): pass 0 the
	// march [d0, d1], pass 1 the far tier [far_from_m, end_m] - its shapes band-limited at the pixel footprint
	// (cloud_sample_extinction), its light from cloud_far_light_depth (cloud_sample_light_source), its steps weighted
	// by the hand-over weight, the march's by 1 - it. Their results: radiance, alpha (weighted), alpha-weighted
	// distance.
	vec3 pass_radiance[2] = vec3[](vec3(0.0), vec3(0.0));
	float pass_alpha[2] = float[](0.0, 0.0);
	// 1 - the pass's alpha as T_bg + the hand-over's dropped shares (1 - keep) of the steps' alpha: no cancellation.
	float pass_rest[2] = float[](1.0, 1.0);
	float pass_weighted[2] = float[](0.0, 0.0);
	float transmittance_end = 1.0;    // the march's background transmittance at its end (the debug outputs)
	float transmittance_front = 1.0;  // the march's in front of the hand-over band (the far tier's attenuation)
	int passes = handover && shapes > 0.0 && far_from_m < end_m ? 2 : 1;
	for (int pass = 0; pass < passes; pass++) {
		bool far = pass == 1;
		if (far && transmittance_front <= CLOUD_FAR_MIN_TRANSMITTANCE) {
			break;
		}
		float seg0 = far ? far_from_m : d0;
		float seg1 = far ? end_m : d1;
		float start = max(seg0, CLOUD_NEAR_START_M);
		int n = max(cloud_steps, 1);
		// Coarse step: the fraction of the distance that spans [start, seg1] in n steps; at most one height layer.
		float coarse_rel = seg1 > start ? exp(log(seg1 / start) / float(n)) - 1.0 : 1.0;
		float transmittance_bg = 1.0;   // exp(-tau): the background
		// The pass's alpha: each step's share of the pixel weighted by keep (the hand-over's cross-fade; = 1 - the
		// background's transmittance without a band).
		float march_alpha = 0.0;
		float dropped_alpha = 0.0;      // the steps' alpha the hand-over's keep left out: sum (1 - keep) step_alpha
		float transmittance_sc = 1.0;   // exp(-(1 - f) tau): the scattered light
		// exp(-w_D tau): the lights' scattered light with the transfer table (cloud_scattering_model 1; the table's
		// source leaves the phase function's HG peak in the view ray, core cloud_transfer_view_extinction)
		float transmittance_tab = 1.0;
		vec3 radiance = vec3(0.0);
		float weighted_distance = 0.0;
		float light_at_m = -1.0e30;
		vec3 sun_irradiance = vec3(0.0);
		vec3 sky_irradiance = vec3(0.0);
		vec3 moon_irradiance = vec3(0.0);
		vec3 moon_sky_irradiance = vec3(0.0);
		float dense_step_m = 1.0e30;  // the step limit after a dense sample (CLOUD_MAX_STEP_WEIGHT)
		float carried_tau = 0.0;  // thin cloud in front of an entry, added to the next step (cloud_march's sub-samples)
		float a = seg0;
		// The deterministic step phase: the first step ends where the ray meets the next height shell k s_v above the
		// ellipsoid (s_v: the first coarse step's height extent), so the samples stay put for a static camera and move
		// with the shells, not with the screen, for a moving one. phase_m: that end (-1: none, the jittered march).
		float phase_m = -1.0;
		// the last sample lay below CLOUD_SILHOUETTE_EXTINCTION_PER_M (the next crossing is an entry)
		bool outside = true;
		// the last clear step ended in the field's cloudy region: the next steps are fine ones
		bool cloud_ahead = false;
		if (cloud_deterministic_march) {
			float h0 = cloud_height_along(ray, seg0);
			float slope = cloud_height_slope(ray, seg0);
			float layer0_m = h0 < CLOUD_FINE_TOP_M ? CLOUD_FINE_LAYER_M : CLOUD_COARSE_LAYER_M;
			float step0_m = max(min(start * coarse_rel, layer0_m / max(abs(slope), 1.0e-3)), CLOUD_MIN_STEP_M);
			float shell_m = step0_m * max(abs(slope), CLOUD_PHASE_MIN_CLIMB);
			float f = fract(h0 / shell_m);
			float to_shell = slope >= 0.0 ? 1.0 - f : f;  // of the step, toward the next shell along the ray
			phase_m = seg0 + step0_m * (to_shell > 1.0e-3 ? to_shell : 1.0);
		}
		for (int i = 0; i < cloud_max_steps && a < seg1; i++) {
			float h_a = cloud_height_along(ray, a);
			float climb = abs(cloud_height_slope(ray, a));
			float layer_m = h_a < CLOUD_FINE_TOP_M ? CLOUD_FINE_LAYER_M : CLOUD_COARSE_LAYER_M;
			float step_m = min(max(a, start) * coarse_rel, layer_m / max(climb, 1.0e-3));
			step_m = max(step_m, CLOUD_MIN_STEP_M);
			if (a < phase_m) {
				step_m = min(step_m, phase_m - a);
			}
			// Inside the field's cloudy region the step shrinks to the fine fraction of the distance: the sub-grid
			// shapes are resolved (a coarse step would hit or miss a cumulus edge per pixel: grain).
			float d = a + jitter * step_m;
			vec2 lat_lon = cloud_lat_lon(camera + eye + dir * d);
			CloudCell cell = cloud_field(lat_lon, cloud_height_along(ray, d));
			if (cell.extinction > 0.0 || cloud_ahead) {
				// Never finer than what still reaches the segment's end within the remaining steps (no far cloud is
				// cut).
				float remaining = float(max(cloud_max_steps - i, 1));
				float reach = max(a, start) * (exp(log(max(seg1, start) / max(a, start)) / remaining) - 1.0);
				// The dense-cloud limit overrides the reach (behind an opaque surface little of the segment is seen).
				float fine = min(max(max(max(a, start) * cloud_fine_step_fraction, reach), CLOUD_MIN_STEP_M),
						dense_step_m);
				if (fine < step_m) {
					step_m = fine;
					d = a + jitter * step_m;
					lat_lon = cloud_lat_lon(camera + eye + dir * d);
					cell = cloud_field(lat_lon, cloud_height_along(ray, d));
				}
			}
			float b = min(a + step_m, seg1);
			d = min(d, b);
			float a_this = a;
			a = b;
			if (cell.extinction <= 0.0) {
				dense_step_m = min(2.0 * dense_step_m, 1.0e30);
				outside = true;
				if (phase_m >= 0.0 && b < seg1) {
					// The deterministic march: a clear sample whose step ends in the field's cloudy region (a deck's
					// top, a cumulus layer's) ends the step at the sample and marches on with fine steps - a fixed
					// sample point would skip up to a coarse step of the cloud there in every frame (the jitter only
					// averaged it).
					cloud_ahead = cloud_field(cloud_lat_lon(camera + eye + dir * b),
							cloud_height_along(ray, b)).extinction > 0.0;
					if (cloud_ahead && d > a_this) {
						a = d;
					}
				}
				continue;
			}
			cloud_ahead = false;
			vec3 offset = eye + dir * d;  // from the head: the ray starts at the eye
			float height = cloud_height_along(ray, d);
			float in_cloud;
			float beta;
			float tau_step;  // the step's optical depth (unscaled)
			if (phase_m >= 0.0 && cloud_debug_output != 1 && cloud_debug_output != 2 && cloud_debug_output != 6) {
				// The deterministic march's samples of the shapes (not for the grid-mean debug outputs, which have no
				// sub-grid structure): the step in equal sub-steps of at most the structure step (cloud_structure_step;
				// beyond CLOUD_MAX_SUBSTEPS of them longer), each sampled at its midpoint in its own field cell (a
				// sheet of cirrus a few hundred metres thick changes over a slant step: the step's cell lost ~2 % of
				// its alpha).
				float span_m = b - a_this;
				// (the far tier's shapes are band-limited at the pixel footprint: their structure step scales with the
				// mip)
				float structure_m = cloud_structure_step(cell, climb)
						* (far ? exp2(cloud_far_level(pixel_angle * a_this)) : 1.0);
				float spacing_m = max(structure_m, span_m / float(CLOUD_MAX_SUBSTEPS));
				int subs = clamp(int(ceil(span_m / spacing_m - 1.0e-3)), 1, CLOUD_MAX_SUBSTEPS);
				float sub_m = span_m / float(subs);
				// The detail's band limit: the pixel footprint, at least CLOUD_DETAIL_STEP_FRACTION of the step up to
				// the structure step's spacing - continuous in the step length (with the sub-step itself, which halves
				// where the step crosses the structure step, the band limit jumped there: a contour on near cumuli, fix
				// round 1).
				spacing_m = min(span_m, spacing_m);
				float sum_tau = 0.0;     // of the sub-steps taken
				float sum_tau_d = 0.0;
				float sum_tau_in = 0.0;  // x the sub-samples' in-cloud extinction (their cells')
				float sum_tau_gap = 0.0;  // of the stretch from the last sub-step taken to an entry
				float before = a_this;   // the last sub-sample (the step's start before the first)
				float beta_before = 0.0;
				float beta_taken_max = 0.0;  // the largest extinction of the sub-samples taken
				float taken_m = span_m;      // the step's length (shortened at an entry after cloud)
				int taken = subs;            // the sub-steps taken
				bool entered = false;        // ended at an entry with only thin cloud in front (carried)
				for (int j = 0; j < subs; j++) {
					float d_j = a_this + (float(j) + 0.5) * sub_m;
					float detail_j = max(pixel_angle * d_j, CLOUD_DETAIL_STEP_FRACTION * spacing_m);
					vec2 lat_lon_j;
					float in_cloud_j;
					float beta_j = cloud_sample_extinction(far, eye + dir * d_j, cloud_height_along(ray, d_j), detail_j,
							pixel_angle * d_j, lat_lon_j, in_cloud_j);
					bool entry = outside && beta_j >= CLOUD_SILHOUETTE_EXTINCTION_PER_M;
					outside = beta_j < CLOUD_SILHOUETTE_EXTINCTION_PER_M;
					if (entry) {
						// The silhouette between the last sub-sample and this one: bisected for the threshold
						// (CLOUD_SILHOUETTE_BISECTIONS), then the in-cloud steps start there (the outline found, not
						// sampled).
						float lo = before;
						float hi = d_j;
						float beta_lo = beta_before;
						for (int k = 0; k < CLOUD_SILHOUETTE_BISECTIONS; k++) {
							float mid = 0.5 * (lo + hi);
							vec2 lat_lon_mid;
							float in_cloud_mid;
							float beta_mid = cloud_sample_extinction(far, eye + dir * mid, cloud_height_along(ray, mid),
									detail_j, pixel_angle * mid, lat_lon_mid, in_cloud_mid);
							if (beta_mid >= CLOUD_SILHOUETTE_EXTINCTION_PER_M) {
								hi = mid;
							} else {
								lo = mid;
								beta_lo = beta_mid;
							}
						}
						if (lo > a_this) {
							// The step ends at the entry. In front of it: the sub-steps taken and the stretch from
							// their end to the entry (at the bracket's clear end's extinction). Thin cloud only (all
							// below the threshold) is carried into the next step (dropped, a thin layer went missing
							// with every entry as the ray's step phase fell - grain); cloud the ray left and re-enters
							// within the step is a step of its own with its light (carried, it darkened the next step's
							// light: black specks).
							float gap_m = max(lo - (a_this + float(j) * sub_m), 0.0);
							a = lo;
							if (beta_taken_max < CLOUD_SILHOUETTE_EXTINCTION_PER_M) {
								carried_tau += sum_tau + beta_lo * gap_m;
								entered = true;
							} else {
								sum_tau_gap = beta_lo * gap_m;
								taken_m = float(j) * sub_m + gap_m;
								taken = j;
							}
							break;
						}
					}
					sum_tau += beta_j * sub_m;
					sum_tau_d += beta_j * sub_m * d_j;
					sum_tau_in += beta_j * sub_m * in_cloud_j;
					before = d_j;
					beta_before = beta_j;
					beta_taken_max = max(beta_taken_max, beta_j);
				}
				if (entered) {
					continue;
				}
				if (sum_tau <= 0.0) {
					dense_step_m = min(2.0 * dense_step_m, 1.0e30);  // clear air: the steps grow back
					continue;
				}
				beta = (sum_tau + sum_tau_gap) / taken_m;  // the step's mean extinction
				tau_step = sum_tau + sum_tau_gap;
				if (taken < subs) {
					// A step shortened at an entry: the length of its sub-steps for the step-weight test below (not lo
					// - a_this: its float32 difference far from the camera retried the step until the budget ran out,
					// as noted there).
					step_m = float(taken) * sub_m;
				}
				// The step's light and distance at its extinction-weighted centre (where it scatters); the step's cell.
				in_cloud = sum_tau_in / sum_tau;
				d = sum_tau_d / sum_tau;
				offset = eye + dir * d;
				height = cloud_height_along(ray, d);
			} else {
				// The detail's band limit: the pixel footprint (the jittered march; the grid-mean debug outputs).
				beta = cloud_sample_extinction_in_cell(far, cell, offset, height, pixel_angle * d, pixel_angle * d,
						in_cloud);
				if (beta <= 0.0) {
					dense_step_m = min(2.0 * dense_step_m, 1.0e30);  // clear air: the steps grow back
					outside = true;
					continue;
				}
				tau_step = beta * (b - a_this);
				if (phase_m >= 0.0) {
					bool entry = outside && beta >= CLOUD_SILHOUETTE_EXTINCTION_PER_M;
					outside = beta < CLOUD_SILHOUETTE_EXTINCTION_PER_M;
					if (entry && d > a_this) {
						// The silhouette (the debug outputs' grid mean, as above for the shapes).
						float lo = a_this;
						float hi = d;
						for (int j = 0; j < CLOUD_SILHOUETTE_BISECTIONS; j++) {
							float mid = 0.5 * (lo + hi);
							vec2 lat_lon_mid;
							float in_cloud_mid;
							float beta_mid = cloud_sample_extinction(far, eye + dir * mid, cloud_height_along(ray, mid),
									pixel_angle * mid, pixel_angle * mid, lat_lon_mid, in_cloud_mid);
							if (beta_mid >= CLOUD_SILHOUETTE_EXTINCTION_PER_M) {
								hi = mid;
							} else {
								lo = mid;
							}
						}
						if (lo > a_this) {
							a = lo;  // the in-cloud steps start at the entry (the bracket's clear end)
							continue;
						}
					}
				}
			}
			// A step weighing more than CLOUD_MAX_STEP_WEIGHT in the pixel is done again with steps of that weight. The
			// step's intended length is compared, not b - a_this: far from the camera (a float32 distance of 8.6e6 m
			// has a 1 m ulp) the rounded difference exceeded the 0.1 % margin again and again - the same step was
			// retried until the step budget ran out and the ray never reached the cloud (Task 10: low decks seen from
			// the scaled-space limit, the horizon from orbit).
			float allowed = CLOUD_MAX_STEP_WEIGHT / transmittance_sc;
			float refined = allowed < 1.0 ? max(-log(1.0 - allowed) / (k * beta), CLOUD_MIN_STEP_M) : 1.0e30;
			if (min(step_m, seg1 - a_this) > 1.001 * refined) {
				dense_step_m = refined;
				a = a_this;
				continue;
			}
			// the limit at this density (grows as the pixel turns opaque)
			dense_step_m = max(refined, CLOUD_MIN_STEP_M);
			float tau = tau_step + carried_tau;
			carried_tau = 0.0;
			// Over the hand-over band the step's share of the pixel (its light and its alpha, after the clouds in front
			// of it) is weighted by 1 - the far tier's weight there, the far tier's steps by the weight: a
			// premultiplied cross-fade, linear per cloud. A faded opacity per step compounds over the many steps inside
			// one cloud like a faded density - a thick cloud stayed opaque until the band's end.
			float keep = !handover ? 1.0 : (far ? cloud_handover_weight(d) : 1.0 - cloud_handover_weight(d));
			float scaled_step = exp(-k * tau);
			float true_step = exp(-tau);
			// A ray toward the Sun marched on past the scaled criterion (CLOUD_SUN_MIN_TRANSMITTANCE) carries the
			// background's transmittance alone: its light beyond is dropped as on every other ray (no brighter disc
			// of the last percent around the Sun), and the light samples are saved.
			if (!coverage_only && transmittance_sc >= CLOUD_MIN_SCALED_TRANSMITTANCE) {
				vec3 p = camera + offset;
				vec3 normal = normalize(p * ATM_STRETCH * ATM_STRETCH);  // the ellipsoid's normal (geodetic up)
				vec2 column = cloud_column_depths(cell, lat_lon, height, in_cloud);
				// The light march's jitter decorrelated from step to step (golden-ratio sequence): the samples of one
				// ray average its noise instead of sharing one light-march error. The deterministic march: the light
				// march's midpoints too (the Moon's; the Sun's where no cache holds it).
				float light_jitter = phase_m >= 0.0 ? 0.5 : fract(jitter + float(i) * 0.6180339887);
				vec3 source = vec3(0.0);        // the sky's (scaled problem: attenuated on the scaled depths)
				vec3 light_source = vec3(0.0);  // the Sun's and the Moon's (cloud_light_source)
				if (cloud_debug_output == 2 || cloud_debug_output == 7) {
					light_source = vec3(cloud_sample_light_source(far, sun_dir, normal, dir, offset, height, d, column,
							light_jitter, pixel_angle, true));
				} else {
					// The air's light at the cloud (Sun and sky irradiance), refreshed along the ray every
					// CLOUD_LIGHT_REFRESH_FRACTION of the distance (at least CLOUD_LIGHT_REFRESH_M): it changes over
					// kilometres.
					if (abs(d - light_at_m) > max(CLOUD_LIGHT_REFRESH_M, CLOUD_LIGHT_REFRESH_FRACTION * d)) {
						light_at_m = d;
						sun_irradiance = ATM_PI * atm_sun_at(p) * atm_sunlit(p);
						float eclipse_x = clamp((d - CLOUD_ECLIPSE_NEAR_M)
								/ (CLOUD_ECLIPSE_FAR_M - CLOUD_ECLIPSE_NEAR_M), 0.0, 1.0);
						sky_irradiance = ATM_PI * atm_sky_light(p, normal) * mix(eclipse_near, eclipse_far, eclipse_x);
						if (moonlit) {
							moon_irradiance = ATM_PI * atm_moon_at(p);
							moon_sky_irradiance = ATM_PI * atm_moon_sky_light(p, normal);
						}
					}
					float sky_source = cloud_sky_scattering_source(-dot(dir, normal), column.x, column.y);
					source = sky_irradiance * sky_source;
					if (dot(sun_irradiance, sun_irradiance) > 0.0) {
						light_source += sun_irradiance
								* cloud_sample_light_source(far, sun_dir, normal, dir, offset, height, d, column,
								light_jitter, pixel_angle, true);
					}
					if (moonlit) {
						light_source += moon_irradiance
								* cloud_sample_light_source(far, moon_dir, normal, dir, offset, height, d, column,
								light_jitter, pixel_angle, false);
						source += moon_sky_irradiance * sky_source;
					}
				}
				if (cloud_scattering_model == 1) {
					// The lights' source with the table (exact single scattering + the tabulated Draine part of the
					// higher orders) is attenuated with exp(-w_D tau); the sky's stays in the scaled problem.
					float tab_step = exp(-w_tab * tau);
					radiance += keep * (transmittance_sc * source * (1.0 - scaled_step) / k
							+ transmittance_tab * light_source * (1.0 - tab_step) / w_tab);
					transmittance_tab *= tab_step;
				} else {
					radiance += keep * transmittance_sc * (source + light_source) * (1.0 - scaled_step) / k;
				}
			}
			transmittance_sc *= scaled_step;
			float next_bg = transmittance_bg * true_step;
			float step_alpha = transmittance_bg - next_bg;
			march_alpha += keep * step_alpha;
			dropped_alpha += (1.0 - keep) * step_alpha;
			weighted_distance += keep * d * step_alpha;
			transmittance_bg = next_bg;
			if (!far && d < cloud_handover_near_m) {
				transmittance_front = transmittance_bg;
			}
			if ((transmittance_sc < CLOUD_MIN_SCALED_TRANSMITTANCE
					&& !(sun_ray && transmittance_bg > CLOUD_SUN_MIN_TRANSMITTANCE))
					|| (coverage_only && !sun_ray && transmittance_bg < CLOUD_COVERAGE_MIN_TRANSMITTANCE)) {
				break;
			}
		}

		// Thin cloud in front of an entry the step budget ended at: its transmittance (its light is negligible, tau <
		// 0.05).
		float carried = exp(-carried_tau);
		float carried_keep = !handover ? 1.0 : (far ? cloud_handover_weight(seg1) : 1.0 - cloud_handover_weight(seg1));
		march_alpha += carried_keep * transmittance_bg * (1.0 - carried);
		dropped_alpha += (1.0 - carried_keep) * transmittance_bg * (1.0 - carried);
		transmittance_bg *= carried;
		pass_radiance[pass] = radiance;
		pass_alpha[pass] = handover ? march_alpha : 1.0 - transmittance_bg;
		pass_rest[pass] = handover ? transmittance_bg + dropped_alpha : transmittance_bg;
		pass_weighted[pass] = weighted_distance;
		if (!far) {
			transmittance_end = transmittance_bg;
		}
	}

	if (cloud_debug_output == 1 || cloud_debug_output == 4) {
		color = vec3(transmittance_end);
		alpha = 1.0;
		transmittance = 0.0;
		return;
	}
	if (cloud_debug_output == 2 || cloud_debug_output == 7) {
		color = pass_radiance[0];
		alpha = 1.0;
		transmittance = 0.0;
		return;
	}
	// The composite (static clouds Task 8): the march's image plus, behind the march's clouds in front of the band, the
	// far tier's (faded to the column model at large footprints, weighted by the hand-over weight at the far slab):
	// both march the same clouds, so a cloud in the band - or two behind each other - is the exact linear cross-fade of
	// the two renders; the cloud distance weighted by each part's alpha.
	float behind = handover ? transmittance_front : 0.0;
	vec4 column_part = vec4(0.0);
	float column_m = 0.0;
	if (handover && shapes < 1.0 && behind > CLOUD_FAR_MIN_TRANSMITTANCE) {
		vec2 column_p;
		float d_p = cloud_far_slab(ray, dir, eye, far_from_m, end_m, surface, column_p);
		if (d_p > 0.0) {
			column_part = cloud_handover_weight(d_p)
					* cloud_far_column(dir, eye, d_p, column_p, pixel_angle * d_p, eclipse_near, eclipse_far,
					coverage_only);
			column_m = d_p;
		}
	}
	float near_alpha = pass_alpha[0];
	float far_alpha = mix(column_part.a, pass_alpha[1], shapes);
	alpha = min(near_alpha + behind * far_alpha, 1.0);
	// T = 1 - near_alpha - behind x far_alpha = (near_rest - behind) + behind x far_rest: both terms at the magnitude
	// of T itself.
	float far_rest = mix(1.0 - column_part.a, pass_rest[1], shapes);
	transmittance = clamp((pass_rest[0] - behind) + behind * far_rest, 0.0, 1.0);
	float weighted_m = pass_weighted[0]
			+ behind * ((1.0 - shapes) * column_part.a * column_m + shapes * pass_weighted[1]);
	cloud_m = alpha > 1.0e-6 ? weighted_m / alpha : 0.0;
	if (coverage_only) {
		return;
	}
	if (alpha <= 1.0e-6 && dot(pass_radiance[0], pass_radiance[0]) <= 0.0
			&& dot(pass_radiance[1], pass_radiance[1]) <= 0.0) {
		alpha = 0.0;
		transmittance = 1.0;
		cloud_m = 0.0;
		return;
	}
	for (int pass = 0; pass < passes; pass++) {
		vec3 air_transmittance = vec3(1.0);
		vec3 air_light = vec3(0.0);
		if (pass_alpha[pass] > 1.0e-6) {
			air_light = atm_to_point_offset(eye + dir * (pass_weighted[pass] / pass_alpha[pass]), air_transmittance);
		}
		// Debug output 11: the clouds' own light zeroed, the air in front of them kept (the composite's air weight).
		vec3 part = air_transmittance * pass_radiance[pass] * (cloud_debug_output == 11 ? 0.0 : 1.0)
				+ pass_alpha[pass] * air_light;
		color += pass == 0 ? part : behind * shapes * part;
	}
	color += behind * (1.0 - shapes) * column_part.rgb;
}

// ---- Cloud shadows on the air (static clouds Task 9b, ruling 2026-10-27 "Wolkenschatten auf die Luft"). The air
// light the tables give (atmosphere.gdshaderinc) is the clear sky's: every point of a view ray is lit by the Sun's
// direct beam. Under and beside clouds that beam is cut: a backlit cumulus is seen through the shadowed haze in front
// of it, and the gaps of a broken deck at a low Sun send crepuscular rays across the sky. Physics: of the air light
// only the single scattering of the direct beam (single Rayleigh: atmosphere_single_rayleigh, single Mie:
// atmosphere_single_mie) is weighted by the beam's cloud transmittance T_c at the scattering point; the multiple
// scattering (the sky's diffuse light, the deck's glow) is unchanged. The loss along the view ray,
//     L = sum_i (1 - T_c(p_i)) x [T(x, s_i-1) S1(s_i-1) - T(x, s_i) S1(s_i)] x V(s_i)
// in Bruneton's difference form per segment (as the eclipse's atm_segmented_scattering: unweighted it telescopes to
// the plain single-scattering lookup), S1 = S1_R x P_R + S1_M x P_M x the scene's Mie fade (smoothstep(0, 0.01, mu_s)
// at the CAMERA, as atm_to_point_model and atm_sky: the unweighted sum is then exactly the scene's single
// scattering), V = 1 in front of the clouds' weighted distance and the background's transmittance behind it (the
// composite dims the scene's air behind a cloud by T and adds the air in front of it at weight 1: cloud_march's
// alpha x atm_to_point_offset to the clouds' distance; the segment holding that distance split linearly). T_c from
// the cloud shadow maps (cloud_light.gdshaderinc cloud_shadow_transmittance: the near map's shapes within ~10 km, the
// far map's grid mean to 130 km, the slanted column beyond), averaged over fixed points of the segment
// (CLOUD_AIR_SHADOW_SUBSAMPLES): world space, the same for both VR eyes, no jitter (static). The pass's colour carries
// -L (negative where the air is shadowed); the composite adds it to the scene.
// Segments [E]: 32 over the part of the ray below the highest cloud top of the loaded weather
// (cloud_air_shadow_top_m: environment::cloud_render_top_m; no cloud above it shadows the air: the Sun above the
// point's horizon looks up), at d0 + (d1 - d0) (i / n)^2 - denser near the camera, where the air is densest along a
// low ray and the near map's shapes lie.
global uniform sampler3D atmosphere_single_rayleigh : filter_linear, repeat_disable;
// Fidelity flag (CloudLayer.air_shadow): false keeps the clear sky's air light in front of and between the clouds.
global uniform bool cloud_air_shadow;
// The highest cloud top of the bound snapshots (CloudLayer._bind_slots; 0: no cloud), metres above the ellipsoid.
global uniform float cloud_air_shadow_top_m;
// Checks only (cloud_check.gd section 14; -1 in the game): >= 0 replaces T_c at every point by this value.
global uniform float cloud_debug_air_transmittance;
const int CLOUD_AIR_SHADOW_SEGMENTS = 32;
// [E] T_c per segment: the mean of this many points spread evenly over it (fixed per pixel) - one midpoint aliased the
// near map's cloud shadows into regular stripes across the sky (the segment spacing against the shadows' spacing);
// 3 x 32 points show the rays as 96 segments do, at a third of the table lookups.
const int CLOUD_AIR_SHADOW_SUBSAMPLES = 3;

// The single scattering (unphased Rayleigh, unphased Mie) at (r, mu, mu_s, nu) (atm_combined_scattering's lookup).
void cloud_single_scattering(float r, float rho, float mu, float mu_s, float nu, bool ground, out vec3 rayleigh,
		out vec3 mie) {
	vec4 uvwz = atm_scattering_uvwz(r, rho, mu, mu_s, nu, ground);
	float nu_size = atmosphere_scattering_size.x;
	float tex_coord_x = uvwz.x * (nu_size - 1.0);
	float tex_x = floor(tex_coord_x);
	float lerp_nu = tex_coord_x - tex_x;
	vec3 uvw0 = vec3((tex_x + uvwz.y) / nu_size, uvwz.z, uvwz.w);
	vec3 uvw1 = vec3((tex_x + 1.0 + uvwz.y) / nu_size, uvwz.z, uvwz.w);
	rayleigh = mix(textureLod(atmosphere_single_rayleigh, uvw0, 0.0).rgb,
			textureLod(atmosphere_single_rayleigh, uvw1, 0.0).rgb, lerp_nu);
	mie = mix(textureLod(atmosphere_single_mie, uvw0, 0.0).rgb, textureLod(atmosphere_single_mie, uvw1, 0.0).rgb,
			lerp_nu);
}

// The direct Sun's cloud transmittance at an air point (offset from the camera, Earth-local render axes; up_c the
// camera's model-space up); inside the near map's range the map alone (cloud_shadow_transmittance's weights there: no
// column lookup).
float cloud_air_sun_transmittance(vec3 offset_local, vec3 sun_dir, vec3 up_c) {
	if (cloud_debug_air_transmittance >= 0.0) {
		return cloud_debug_air_transmittance;
	}
	vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + offset_local;
	float height_m = cloud_height_of(p);
	if (height_m > CLOUD_TOP_M) {
		return 1.0;
	}
	if (cloud_shadow_valid && length(offset_local - up_c * dot(offset_local, up_c)) <= cloud_shadow_extent.z) {
		return cloud_shadow_map_transmittance(cloud_shadow_near, cloud_shadow_near_anchor, cloud_shadow_extent.x,
				offset_local);
	}
	float mu0 = dot(cloud_geodetic_up(p), sun_dir);
	return cloud_shadow_transmittance(offset_local, cloud_column_mean_depth(cloud_lat_lon(p), height_m), mu0);
}

// The view ray's geometry for the air shadow: the stretch of the ray below the highest cloud top up to end_m (render
// metres: the scene or the ray's end), the camera's model-space point and angles. False: nothing to shadow - no cloud,
// the ray misses the layer, or the camera lies above the atmosphere's top (r > top radius: the tables' parametrisation
// needs the ray moved to its top-of-atmosphere entry, as atm_sky does; not done - an open point: the loss switches
// off above ~100 km).
struct CloudAirRay {
	float from_m;
	float to_m;
	float scale;  // model metres per render metre along the ray
	float r;
	float rho;
	float mu;
	float mu_s;
	float nu;
	bool ground;
	float phase_r;
	float phase_m;  // times the scene's Mie fade at the camera
};

bool cloud_air_ray(CloudRay ray, vec3 dir, vec3 eye, float end_m, out CloudAirRay a) {
	a.from_m = 0.0;
	a.to_m = 0.0;
	float top_m = min(cloud_air_shadow_top_m, CLOUD_TOP_M);
	if (!cloud_air_shadow || !atmosphere_enabled || top_m <= 0.0) {
		return false;
	}
	vec2 top = cloud_sphere_crossing(ray, top_m);
	if (top.x > top.y || top.y <= 0.0) {
		return false;
	}
	a.from_m = max(top.x, 0.0);
	a.to_m = min(top.y, end_m);
	if (a.to_m <= a.from_m) {
		return false;
	}
	vec3 up;
	atm_point_model(eye, up, a.r, a.rho);
	if (a.r > atmosphere_top_radius_m) {
		return false;
	}
	vec3 v_stretched = dir * ATM_STRETCH;
	a.scale = length(v_stretched);
	vec3 v = v_stretched / a.scale;
	vec3 sun = normalize(atmosphere_sun_direction * ATM_STRETCH);
	a.mu = atm_clamp_cosine(dot(up, v));
	a.mu_s = atm_clamp_cosine(dot(up, sun));
	a.nu = atm_clamp_cosine(dot(v, sun));
	a.ground = atm_ray_intersects_ground(a.r, a.rho, a.mu);
	a.phase_r = atm_rayleigh_phase(a.nu);
	// Bruneton's fade of the single Mie term with the Sun below the horizon, at the camera (atm_to_point_model).
	a.phase_m = atm_mie_phase(atmosphere_mie_phase_g, a.nu) * smoothstep(0.0, 0.01, a.mu_s);
	return true;
}

// T(x, s) S1(s) at the distance at_m (render metres) along the ray: the phased single scattering seen from the camera.
vec3 cloud_air_single_at(CloudAirRay a, float at_m) {
	float R = atmosphere_bottom_radius_m;
	float d = at_m * a.scale;
	float h = clamp(atm_height_along(a.r, a.rho, a.mu, d), 0.0, atmosphere_top_radius_m - R);
	float r_d = R + h;
	float mu_d = atm_clamp_cosine((a.r * a.mu + d) / r_d);
	float mu_s_d = atm_clamp_cosine((a.r * a.mu_s + d * a.nu) / r_d);
	vec3 t_d = d > 0.0 ? atm_transmittance(a.r, a.rho, a.mu, d, a.ground) : vec3(1.0);
	vec3 rayleigh;
	vec3 mie;
	cloud_single_scattering(r_d, atm_rho_from_height(h), mu_d, mu_s_d, a.nu, a.ground, rayleigh, mie);
	return t_d * (rayleigh * a.phase_r + mie * a.phase_m);
}

// The air light the clouds' shadows take away along the ray dir from the eye (offset from the camera) up to end_m,
// scene units, in two parts: `front` in front of the clouds' weighted distance front_m (seen directly) and `behind`
// beyond it (unweighted: the caller multiplies it by the background's transmittance T). Loss = front + T x behind.
// Zero without clouds in the Sun's way.
void cloud_air_shadow_parts(CloudRay ray, vec3 dir, vec3 eye, float end_m, float front_m, out vec3 front,
		out vec3 behind) {
	front = vec3(0.0);
	behind = vec3(0.0);
	CloudAirRay a;
	if (!cloud_air_ray(ray, dir, eye, end_m, a)) {
		return;
	}
	vec3 sun_dir = normalize(atmosphere_sun_direction);
	vec3 up_c = normalize((atmosphere_camera_hi + atmosphere_camera_lo) * ATM_STRETCH);
	vec3 s_prev = vec3(0.0);
	float prev_m = a.from_m;
	float inv_n = 1.0 / float(CLOUD_AIR_SHADOW_SEGMENTS);
	for (int i = 0; i <= CLOUD_AIR_SHADOW_SEGMENTS; i++) {
		float f = float(i) * inv_n;
		float at_m = a.from_m + (a.to_m - a.from_m) * f * f;
		vec3 s_d = cloud_air_single_at(a, at_m);
		if (i > 0) {
			float shadow = 0.0;
			for (int k = 0; k < CLOUD_AIR_SHADOW_SUBSAMPLES; k++) {
				float at_k = mix(prev_m, at_m, (float(k) + 0.5) / float(CLOUD_AIR_SHADOW_SUBSAMPLES));
				shadow += 1.0 - cloud_air_sun_transmittance(eye + dir * at_k, sun_dir, up_c);
			}
			shadow /= float(CLOUD_AIR_SHADOW_SUBSAMPLES);
			if (shadow > 0.0) {
				if (atmosphere_eclipse_segments > 0) {
					shadow *= atm_sunlit(atmosphere_camera_hi + atmosphere_camera_lo + eye
							+ dir * (0.5 * (prev_m + at_m)));
				}
				vec3 piece = shadow * (s_prev - s_d);
				// The segment's part in front of the clouds' distance seen directly, the rest through T: linear in
				// front_m (a step at the midpoint stair-stepped the loss along the clouds' distance contours).
				float seen = clamp((front_m - prev_m) / max(at_m - prev_m, 1.0e-3), 0.0, 1.0);
				front += seen * piece;
				behind += (1.0 - seen) * piece;
			}
		}
		s_prev = s_d;
		prev_m = at_m;
	}
	// No clamp per segment: a piece the tables' interpolation makes negative (long near-horizon segments, the setting
	// Sun) belongs to the telescoping sum - clamped, the sum exceeded the scene's single scattering by up to 8 % (Sun
	// 4 deg, view 1 deg up) and 10-50 % at a 0.3-deg Sun (cloud_check section 14). The parts as a whole are >= 0.
	front = atm_to_scene(max(front, vec3(0.0)));
	behind = atm_to_scene(max(behind, vec3(0.0)));
}

// The loss with the background's transmittance behind_t (see cloud_air_shadow_parts).
vec3 cloud_air_shadow_loss(CloudRay ray, vec3 dir, vec3 eye, float end_m, float front_m, float behind_t) {
	vec3 front;
	vec3 behind;
	cloud_air_shadow_parts(ray, dir, eye, end_m, front_m, front, behind);
	return front + behind_t * behind;
}

// Checks (debug output 8): the scene's single scattering over the air shadow's stretch of the ray, by the endpoint
// form T(x, s0) S1(s0) - T(x, s1) S1(s1) - what the unweighted segment sum must telescope to.
vec3 cloud_air_single_reference(CloudRay ray, vec3 dir, vec3 eye, float end_m) {
	CloudAirRay a;
	if (!cloud_air_ray(ray, dir, eye, end_m, a)) {
		return vec3(0.0);
	}
	return atm_to_scene(cloud_air_single_at(a, a.from_m) - cloud_air_single_at(a, a.to_m));
}

// The work group's eclipse factors on the sky light (CLOUD_ECLIPSE_NEAR_M / _FAR_M) along its centre ray dir from
// origin (offset from the camera): probe points above the ground (a ray that meets it is probed short of it).
void cloud_eclipse_factors(vec3 dir, vec3 origin, out vec3 near_factor, out vec3 far_factor) {
	near_factor = vec3(1.0);
	far_factor = vec3(1.0);
	if (cloud_enabled && atmosphere_enabled && atmosphere_eclipse_segments > 0) {
		vec3 camera_point = atmosphere_camera_hi + atmosphere_camera_lo + origin;
		vec2 ground = cloud_sphere_crossing(cloud_ray_from(dir, origin), 0.0);
		float limit_m = ground.x <= ground.y && ground.x > 0.0 ? ground.x : CLOUD_ECLIPSE_FAR_M;
		near_factor = atm_sky_eclipse_factor(camera_point + dir * min(CLOUD_ECLIPSE_NEAR_M, limit_m));
		far_factor = atm_sky_eclipse_factor(camera_point + dir * min(CLOUD_ECLIPSE_FAR_M, limit_m));
	}
}

// The start jitter of a ray (the deterministic march: the stratified midpoints, 0.5; else blue noise at noise_pixel
// moved per frame) and, with CLOUD_SHAPE_PREFILTER, the sub-pixel shape offset (clouds.gdshaderinc
// cloud_subpixel_offset).
float cloud_ray_jitter(ivec2 noise_pixel) {
	float jitter = 0.5;
	if (!cloud_deterministic_march) {
		jitter = fract(texelFetch(cloud_blue_noise, noise_pixel % 64, 0).r + cloud_view.params.x);
	}
#ifdef CLOUD_SHAPE_PREFILTER
	// The sub-pixel shape offset: blue noise at four shifted texels, moved per frame along the R4 sequence (Roberts
	// 2018: 1 / phi_4^k) of the frame index. The deterministic march reads the footprint's centre (0.5: no offset) - the
	// mips' box filter there is the footprint's mean cover (spec decision 4), the same in every frame.
	if (!cloud_deterministic_march) {
		float frame = cloud_view.eye_earth.w;
		cloud_shape_subpixel = fract(vec4(texelFetch(cloud_blue_noise, (noise_pixel + ivec2(17, 41)) % 64, 0).r,
				texelFetch(cloud_blue_noise, (noise_pixel + ivec2(43, 29)) % 64, 0).r,
				texelFetch(cloud_blue_noise, (noise_pixel + ivec2(5, 53)) % 64, 0).r,
				texelFetch(cloud_blue_noise, (noise_pixel + ivec2(31, 11)) % 64, 0).r)
				+ frame * vec4(0.8566748839, 0.7338918708, 0.6287067210, 0.5385972572));
	}
#endif
	return jitter;
}

// What the images store of a ray's colour and transmittance: the half-float range (a lit cloud seen through a
// dark-adapted eye; negative: the air shadow's loss, cloud_air_shadow_loss), no NaN into the history, and in .a the
// background's transmittance T, not alpha: a half float resolves T relatively (normal down to 6.1e-5, subnormal to
// 6e-8), where 1 - T near 1 had a spacing of 4.9e-4 - the Sun's disk vanished behind a slant tau of 8.
vec4 cloud_stored_color(vec3 color, float transmittance) {
	color = clamp(color, vec3(-65000.0), vec3(65000.0));
	if (any(isnan(color)) || isnan(transmittance)) {
		color = vec3(0.0);
		transmittance = 1.0;
	}
	return vec4(color, clamp(transmittance, 0.0, 1.0));
}

// The shading of one view ray (cloud view cache Task 3: the per-frame march and the cache's fill share it):
// dir_earth (Earth-local unit) from origin_offset (the eye's or the anchor's offset from the camera, Earth-local, m) up
// to scene_m - the march (cloud_ray_jitter at noise_pixel), the air shadow's loss in the colour, the debug outputs.
// Out: the stored colour (cloud_stored_color: premultiplied colour + the background's transmittance T in .a) and the
// alpha-weighted cloud distance.
void cloud_shade_ray(vec3 dir_earth, vec3 origin_offset, float scene_m, ivec2 noise_pixel, vec3 eclipse_near,
		vec3 eclipse_far, out vec4 color_t, out float cloud_m) {
	vec3 color = vec3(0.0);
	float alpha = 0.0;
	float transmittance = 1.0;
	cloud_m = 0.0;
	// The air shadow's loss in the colour: the game's image (debug output 0) and the checks' 9 (the loss alone: the
	// march's colour zeroed) and 10 (as 9; CLOUD_EDGE_REFINE's outline pixels their own).
	bool air = cloud_debug_output == 0 || cloud_debug_output == 9 || cloud_debug_output == 10;
	if (cloud_enabled && atmosphere_enabled) {
		float jitter = cloud_ray_jitter(noise_pixel);
		cloud_march(dir_earth, origin_offset, scene_m, jitter, eclipse_near, eclipse_far, false, color, alpha,
				transmittance, cloud_m);
		if (cloud_debug_output == 9 || cloud_debug_output == 10) {
			color = vec3(0.0);  // the loss alone
		}
		if (air) {
			color -= cloud_air_shadow_loss(cloud_ray_from(dir_earth, origin_offset), dir_earth, origin_offset, scene_m,
					alpha > 1.0e-6 ? cloud_m : 1.0e30, transmittance);
		}
		if (cloud_debug_output == 8) {
			color = cloud_air_single_reference(cloud_ray_from(dir_earth, origin_offset), dir_earth, origin_offset,
					scene_m);
			transmittance = 0.0;
		} else if (cloud_debug_output == 12) {
			color = vec3(cloud_m, alpha, 0.0);
			transmittance = 0.0;
		}
	}
	color_t = cloud_stored_color(color, transmittance);
}

#ifndef CLOUD_CACHE_FILL
#ifdef CLOUD_CENTRE
// The centre view has no depth buffer of its own: its rays take the scene distance from an eye's (VR performance
// part 2).
layout(set = 1, binding = 4) uniform sampler2D scene_depth_right;  // `scene_depth` is the left eye's
layout(set = 1, binding = 5, std140) uniform CloudEyes {
	mat4 clip_from_head[2];  // the eyes' projections (the renderer's: eye offset and rotation inside)
	mat4 inv_projection[2];
	vec4 eye_sizes;          // xy: the eyes' full-resolution size
} cloud_eyes;

// [E] Where a centre ray is projected into an eye to find its depth texels: far, so the texel is the eye's look along
// the same direction; nearer geometry is then read with a parallax of at most IPD/2 / D (the distance from the head
// differs by at most IPD/2: cloud_vr_stereo_test), which the eyes' own upsample (their depth) absorbs at full
// resolution.
const float CLOUD_CENTRE_PROBE_M = 1.0e6;
// [E] Samples per axis of an eye's depth texels under one centre pixel (strided beyond).
const int CLOUD_CENTRE_GATHER_MAX = 6;

// The ndc of a head-space direction's far point in an eye; w <= 0 (behind the eye): far outside.
vec2 cloud_eye_ndc(int eye, vec3 dir_view) {
	vec4 c = cloud_eyes.clip_from_head[eye] * vec4(dir_view * CLOUD_CENTRE_PROBE_M, 1.0);
	return c.w > 0.0 ? c.xy / c.w : vec2(1.0e9);
}

// Nearest, farthest, sum and count of the scene distances under a low-resolution centre pixel: read in the eye whose
// ndc holds the pixel's centre ray (the left one first), over the eye texels its corners project to.
void cloud_block_distances(ivec2 pixel, vec2 scale, inout float near_m, inout float far_m, inout float sum_m,
		inout float count) {
	vec3 dir = cloud_ray_view(min((vec2(pixel) + 0.5) * scale, cloud_view.sizes.xy));
	int eye = all(lessThanEqual(abs(cloud_eye_ndc(0, dir)), vec2(1.0))) ? 0 : 1;
	vec2 size = cloud_eyes.eye_sizes.xy;
	vec2 lo = vec2(1.0e30);
	vec2 hi = vec2(-1.0e30);
	for (int j = 0; j <= 1; j++) {
		for (int i = 0; i <= 1; i++) {
			vec2 corner = min(vec2(pixel + ivec2(i, j)) * scale, cloud_view.sizes.xy);
			vec2 px = (clamp(cloud_eye_ndc(eye, cloud_ray_view(corner)), vec2(-1.0), vec2(1.0)) * 0.5 + 0.5) * size;
			lo = min(lo, px);
			hi = max(hi, px);
		}
	}
	ivec2 full = ivec2(size);
	ivec2 first = clamp(ivec2(floor(lo)), ivec2(0), full - 1);
	ivec2 last = clamp(ivec2(ceil(hi)) - 1, first, full - 1);
	ivec2 stride = max((last - first + CLOUD_CENTRE_GATHER_MAX) / CLOUD_CENTRE_GATHER_MAX, ivec2(1));
	for (int y = first.y; y <= last.y; y += stride.y) {
		for (int x = first.x; x <= last.x; x += stride.x) {
			float depth = eye == 0 ? texelFetch(scene_depth, ivec2(x, y), 0).r
					: texelFetch(scene_depth_right, ivec2(x, y), 0).r;
			float dist = cloud_scene_distance_in(cloud_eyes.inv_projection[eye], size, vec2(x, y) + 0.5, depth);
			near_m = min(near_m, dist);
			far_m = max(far_m, dist);
			sum_m += dist;
			count += 1.0;
		}
	}
}
#else
// Nearest, farthest, sum and count of the scene distances of the block of full-resolution pixels this pixel covers.
void cloud_block_distances(ivec2 pixel, vec2 scale, inout float near_m, inout float far_m, inout float sum_m,
		inout float count) {
	ivec2 full = ivec2(cloud_view.sizes.xy);
	ivec2 first = ivec2(vec2(pixel) * scale);
	ivec2 last = min(ivec2(vec2(pixel + 1) * scale) - 1, full - 1);
	for (int y = first.y; y <= last.y; y++) {
		for (int x = first.x; x <= last.x; x++) {
			float dist = cloud_scene_distance(vec2(x, y) + 0.5, texelFetch(scene_depth, ivec2(x, y), 0).r);
			near_m = min(near_m, dist);
			far_m = max(far_m, dist);
			sum_m += dist;
			count += 1.0;
		}
	}
}
#endif

void main() {
	ivec2 low = ivec2(cloud_view.sizes.zw);
	vec2 scale = cloud_view.sizes.xy / cloud_view.sizes.zw;
#ifdef CLOUD_EDGE_REFINE
	// One outline pixel per invocation (the list's entry); the coverage-only rays need no eclipse factors (no light).
	// (the groups are a 2-D grid: one dimension is capped at 65535 groups)
	uint entry = (gl_WorkGroupID.y * gl_NumWorkGroups.x + gl_WorkGroupID.x) * CLOUD_OUTLINE_GROUP_PIXELS
			+ gl_LocalInvocationIndex;
	if (entry >= cloud_outline.count) {
		return;
	}
	ivec2 pixel = ivec2(cloud_outline.pixels[entry] & 0xffffu, cloud_outline.pixels[entry] >> 16);
	vec4 coarse = texelFetch(coarse_color, pixel, 0);
	vec4 opaque_neighbour;
	cloud_outline_spread(pixel, low, coarse, opaque_neighbour);
#else
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	// The work group's eclipse factors on the sky light (cloud_eclipse_factors), before any invocation leaves.
	if (gl_LocalInvocationIndex == 0u) {
		vec2 group_centre = (vec2(gl_WorkGroupID.xy * gl_WorkGroupSize.xy) + 0.5 * vec2(gl_WorkGroupSize.xy)) * scale;
		vec3 group_dir = cloud_ray_earth(min(group_centre, cloud_view.sizes.xy - 0.5));
		cloud_eclipse_factors(group_dir, cloud_view.eye_earth.xyz, cloud_group_eclipse_near, cloud_group_eclipse_far);
	}
	barrier();
	if (pixel.x >= low.x || pixel.y >= low.y) {
		return;
	}
#endif
	vec2 centre = (vec2(pixel) + 0.5) * scale;
	bool nearest = ((pixel.x + pixel.y) & 1) == 0;
	float near_m = 1.0e30;
	float far_m = 0.0;
	float sum_m = 0.0;
	float count = 0.0;
	cloud_block_distances(pixel, scale, near_m, far_m, sum_m, count);
	float scene_m = nearest ? near_m : far_m;
	if (far_m < 1.0e29 && far_m <= near_m * exp(CLOUD_ONE_SURFACE_SPREAD)) {
		scene_m = sum_m / count;  // one surface: the centre ray's distance
	}
#ifdef CLOUD_EDGE_REFINE
	vec3 color = vec3(0.0);
	float alpha = 0.0;
	float transmittance = 1.0;
	float cloud_m = 0.0;
	// The air shadow's loss in the colour (as cloud_shade_ray).
	bool air = cloud_debug_output == 0 || cloud_debug_output == 9 || cloud_debug_output == 10;
	if (cloud_enabled && atmosphere_enabled) {
		float jitter = cloud_ray_jitter(pixel);
		// The coverage: the mean alpha of the centre ray (the first march's) and CLOUD_EDGE_SUBRAYS coverage-only rays at
		// the rotated grid's fixed offsets (the same in every frame: static); the cloud distance weighted by each ray's
		// alpha (the march's own convention). The colour per alpha: CLOUD_EDGE_COLOUR_ALPHA_RATIO's. Full marches per
		// sub-ray (their light samples) had made the pass ~3x as costly (fix round 1).
		vec4 centre_depth = imageLoad(out_depth, pixel);  // the first march's, this pixel's own texel
		float centre_alpha = 1.0 - coarse.a;  // the images hold T in .a
		float coverage = centre_alpha;
		float transmittance_sum = coarse.a;
		float weighted_m = centre_alpha * centre_depth.y;
		for (int k = 0; k < CLOUD_EDGE_SUBRAYS; k++) {
			vec3 sub_color;
			float sub_alpha;
			float sub_transmittance;
			float sub_m;
			cloud_march(cloud_ray_earth(centre + CLOUD_EDGE_OFFSETS[k] * scale), cloud_view.eye_earth.xyz,
					scene_m, jitter, vec3(1.0), vec3(1.0), true, sub_color, sub_alpha, sub_transmittance, sub_m);
			bool bad = isnan(sub_alpha) || isnan(sub_transmittance);
			sub_alpha = bad ? 0.0 : clamp(sub_alpha, 0.0, 1.0);
			coverage += sub_alpha;
			transmittance_sum += bad ? 1.0 : clamp(sub_transmittance, 0.0, 1.0);
			weighted_m += sub_alpha * sub_m;
		}
		alpha = coverage / float(CLOUD_EDGE_SUBRAYS + 1);
		transmittance = transmittance_sum / float(CLOUD_EDGE_SUBRAYS + 1);
		cloud_m = coverage > 1.0e-6 ? weighted_m / coverage : 0.0;
		vec4 source = centre_alpha >= CLOUD_EDGE_COLOUR_ALPHA_RATIO * alpha ? coarse : opaque_neighbour;
		float source_alpha = 1.0 - source.a;
		// The images' colour carries the air shadow's -L (cloud_air_shadow_parts: front + T x behind): the source's
		// (its own T) taken off before its colour per alpha is scaled, this pixel's (its T) added back; the parts of
		// this pixel's ray stand for the source's (the loss varies smoothly across the outline).
		vec3 dir_centre = cloud_ray_earth(centre);
		vec3 air_front = vec3(0.0);
		vec3 air_behind = vec3(0.0);
		if (air) {
			vec3 eye = cloud_view.eye_earth.xyz;
			cloud_air_shadow_parts(cloud_ray_from(dir_centre, eye), dir_centre, eye, scene_m,
					cloud_m > 0.0 ? cloud_m : 1.0e30, air_front, air_behind);
		}
		vec3 loss_source = air_front + source.a * air_behind;
		vec3 loss_here = air_front + transmittance * air_behind;
		color = (source_alpha > 1.0e-6 ? (source.rgb + loss_source) * (alpha / source_alpha) : vec3(0.0)) - loss_here;
		if (cloud_debug_output == 10) {
			color = -loss_here;
		}
		if (cloud_debug_output == 8) {
			color = cloud_air_single_reference(cloud_ray_from(dir_centre, cloud_view.eye_earth.xyz), dir_centre,
					cloud_view.eye_earth.xyz, scene_m);
			transmittance = 0.0;
		} else if (cloud_debug_output == 12) {
			color = vec3(cloud_m, alpha, 0.0);
			transmittance = 0.0;
		}
	}
	vec4 color_t = cloud_stored_color(color, transmittance);
#else
	// One ray through the pixel's centre from the eye (the centre view: the head).
	vec4 color_t;
	float cloud_m;
	cloud_shade_ray(cloud_ray_earth(centre), cloud_view.eye_earth.xyz, scene_m, pixel, cloud_group_eclipse_near,
			cloud_group_eclipse_far, color_t, cloud_m);
#endif
	imageStore(out_color, pixel, color_t);
	imageStore(out_depth, pixel, vec4(scene_m, cloud_m, 0.0, 0.0));
}
#endif

#endif

// =====================================================================================================================
#ifdef CLOUD_TEMPORAL

layout(set = 1, binding = 1) uniform sampler2D current_color;  // nearest
layout(set = 1, binding = 2) uniform sampler2D current_depth;  // nearest
layout(set = 1, binding = 3) uniform sampler2D history_color;  // linear, clamp
layout(rgba16f, set = 1, binding = 4) uniform writeonly image2D out_history;
layout(set = 1, binding = 5) uniform sampler2D history_age;    // nearest, clamp: frames the history holds
layout(r16f, set = 1, binding = 6) uniform writeonly image2D out_age;

// Distance where a pixel without cloud is reprojected (the far sky: rotation only).
const float CLOUD_SKY_REPROJECT_M = 1.0e7;
// [E] Frames a history keeps when the neighbourhood clamp moved it (changed content: a short memory, no ghost).
const float CLOUD_CLAMPED_AGE = 4.0;

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	ivec2 low = ivec2(cloud_view.sizes.zw);
	if (pixel.x >= low.x || pixel.y >= low.y) {
		return;
	}
	vec4 current = texelFetch(current_color, pixel, 0);
	float weight_floor = cloud_view.params.y;
	if (cloud_view.camera_delta.w < 0.5 || weight_floor >= 1.0) {
		imageStore(out_history, pixel, current);
		imageStore(out_age, pixel, vec4(1.0));
		return;
	}
	// The neighbourhood's range: its min / max, or (clip > 0) its mean +- clip standard deviations (variance clipping,
	// Salvi 2016: the march's noise stays inside, a real change of the content leaves it).
	vec4 lo = current;
	vec4 hi = current;
	vec4 m1 = vec4(0.0);
	vec4 m2 = vec4(0.0);
	for (int dy = -1; dy <= 1; dy++) {
		for (int dx = -1; dx <= 1; dx++) {
			vec4 c = texelFetch(current_color, clamp(pixel + ivec2(dx, dy), ivec2(0), low - 1), 0);
			lo = min(lo, c);
			hi = max(hi, c);
			m1 += c;
			m2 += c * c;
		}
	}
	float clip_gamma = cloud_view.temporal.x;
	if (clip_gamma > 0.0) {
		m1 /= 9.0;
		vec4 sigma = sqrt(max(m2 / 9.0 - m1 * m1, vec4(0.0)));
		lo = m1 - clip_gamma * sigma;
		hi = m1 + clip_gamma * sigma;
	}
	// Reprojection of the pixel's point into the previous frame of this eye.
	vec2 scale = cloud_view.sizes.xy / cloud_view.sizes.zw;
	vec3 dir = cloud_ray_earth((vec2(pixel) + 0.5) * scale);
	float cloud_m = texelFetch(current_depth, pixel, 0).g;
	float distance_m = cloud_m > 0.0 ? cloud_m : CLOUD_SKY_REPROJECT_M;
	vec3 previous_offset = cloud_view.eye_earth.xyz + dir * distance_m + cloud_view.camera_delta.xyz;
	vec4 clip = cloud_view.prev_clip_from_earth * vec4(previous_offset, 1.0);
	vec4 result = current;
	float age = 1.0;
	if (clip.w > 0.0) {
		vec2 ndc = clip.xy / clip.w;
		vec2 uv = ndc * 0.5 + 0.5;
		if (all(greaterThanEqual(uv, vec2(0.0))) && all(lessThanEqual(uv, vec2(1.0)))) {
			vec4 history = texture(history_color, uv);
			vec4 clamped = clamp(history, lo, hi);
			float held = texture(history_age, uv).r;
			if (any(notEqual(clamped, history))) {
				held = min(held, CLOUD_CLAMPED_AGE);
			}
			age = min(held + 1.0, 1.0 / weight_floor);
			result = mix(clamped, current, 1.0 / age);
		}
	}
	imageStore(out_history, pixel, result);
	imageStore(out_age, pixel, vec4(age));
}

#endif

// =====================================================================================================================
#ifdef CLOUD_UPSAMPLE

layout(set = 1, binding = 1) uniform sampler2D scene_depth;    // full resolution, nearest
layout(set = 1, binding = 2) uniform sampler2D low_color;      // the accumulated history (of the centre view), nearest
layout(set = 1, binding = 3) uniform sampler2D low_depth;      // the march distances (of the centre view), nearest
layout(rgba16f, set = 1, binding = 4) uniform writeonly image2D out_image;
// The centre view the history belongs to (CloudEffect): its transform from Earth-local offsets of the head to its clip
// space, its sizes (virtual full, low) and params: x the mode (1 = an eye reads the centre (VR), 0 = the view is the
// centre: the desktop's own pixel grid, the 2 x 2 gather of before), y the centre pixel's angle [rad], z the tent's
// largest radius [centre pixels] (CloudEffect: 1 - bilinear - with the deterministic march, 2 with the jittered one),
// w 1 with the deterministic march (not for debug outputs): the final gather is a Catmull-Rom (bicubic) filter.
layout(set = 1, binding = 5, std140) uniform CloudCentre {
	mat4 clip_from_earth;
	vec4 sizes;
	vec4 params;
} cloud_centre;

// [E] Sharpness of the depth weight: a factor e^-1 per 1/4 of a e-fold distance ratio (|ln(D / D_i)| = 0.25: a
// sample 28 % nearer or farther weighs 37 %); equal distances (the sky, a far cloud deck over the sea) weigh 1.
const float CLOUD_DEPTH_SHARPNESS = 4.0;
// The eyes' tent gather (user 2026-10-26: the march's noise may stay but must end in soft alpha edges, not grain):
// weights max(0, 1 - |d| / R) per axis over the 4 x 4 nearest centre pixels. Display only: the history stays
// unfiltered (a filter fed back would blur more every frame). R spans CLOUD_EDGE_SOFT_M at the cloud's distance - a
// cloud's frayed edge is about as wide in metres near and far, so it narrows on screen with distance (a fixed screen
// radius blurred far clouds as much as near ones, user 2026-10-26) - between bilinear (1) and the soft maximum (2):
// with the VR centre's ~1.8 mrad pixels soft up to ~5.5 km, bilinear beyond ~11 km.
// [E] Width of a cloud's soft edge [m] (the frayed rim of a cumulus; cloud_vr_stereo_test mirrors these three).
const float CLOUD_EDGE_SOFT_M = 20.0;
const float CLOUD_TENT_RADIUS_MIN = 1.0;
const float CLOUD_TENT_RADIUS_MAX = 2.0;
// [E] Taps whose depth weights differ by more than this ratio (one below it times the largest) show two surfaces: the
// gather keeps only its kernel's positive part (static clouds Task 8 fix round 1, the dotted horizon row).
const float CLOUD_DEPTH_MIXED_WEIGHT = 0.5;
// [E] The upsample's colour floor per T (cloud_tent_gather): a tap's T below this counts as this (a body tap with a
// negative colour - none in theory - does not make the floor unbounded).
const float CLOUD_FLOOR_MIN_T = 1.0e-3;
// Distance where an eye pixel without scene geometry is reprojected into the centre (the far sky: rotation only).
const float CLOUD_UPSAMPLE_SKY_M = 1.0e7;

// Catmull-Rom's 1-D weight at distance |x| [low pixels] (Keys 1981, a = -0.5): interpolating (1 at 0, 0 at the other
// texel centres), C1, support 2 - a march pixel's binary edge becomes a ramp of about one march pixel with smooth
// corners instead of bilinear's stair steps; its negative lobes are clamped in cloud_tent_gather.
float cloud_catmull_rom(float x) {
	x = abs(x);
	return x < 1.0 ? (1.5 * x - 2.5) * x * x + 1.0 : (x < 2.0 ? ((-0.5 * x + 2.5) * x - 4.0) * x + 2.0 : 0.0);
}

float cloud_depth_weight(float scene_m, float d) {
	return exp(-CLOUD_DEPTH_SHARPNESS * abs(log(max(scene_m, 1.0) / max(d, 1.0))));
}

// The centre's low-resolution position (texel units, texel centres at +0.5) of an Earth-local offset from the head,
// clamped to the image (an eye's edge pixel outside the centre reads its edge).
vec2 cloud_centre_position(vec3 offset) {
	vec4 clip = cloud_centre.clip_from_earth * vec4(offset, 1.0);
	vec2 ndc = clip.w > 0.0 ? clip.xy / clip.w : vec2(0.0);
	vec2 low = cloud_centre.sizes.zw;
	return clamp((ndc * 0.5 + 0.5) * low, vec2(0.5), low - 0.5);
}

// The tent gather at a centre position: premultiplied colour + alpha weighted by the tent and the depth weight; `cloud`
// gets the alpha-weighted cloud distance of the taps (0 without cloud).
// The tent radius [centre pixels] for a cloud at `cloud_m` (CLOUD_EDGE_SOFT_M; params.y: the centre pixel's angle, z:
// the largest radius, at most CLOUD_TENT_RADIUS_MAX).
float cloud_tent_radius(float cloud_m) {
	float largest = clamp(cloud_centre.params.z, CLOUD_TENT_RADIUS_MIN, CLOUD_TENT_RADIUS_MAX);
	return clamp(CLOUD_EDGE_SOFT_M / (max(cloud_m, 1.0) * max(cloud_centre.params.y, 1.0e-9)), CLOUD_TENT_RADIUS_MIN,
			largest);
}

vec4 cloud_tent_gather(vec2 position, float scene_m, float radius, out float cloud) {
	ivec2 low = ivec2(cloud_centre.sizes.zw);
	ivec2 base = ivec2(floor(position - 0.5)) - 1;
	vec4 sum = vec4(0.0);
	float weights = 0.0;
	vec4 sum_positive = vec4(0.0);  // the kernel's positive part alone (CLOUD_DEPTH_MIXED_WEIGHT)
	// The alpha 1 - T too: the Catmull-Rom's weights cancel (sum w << sum |w|), so the normalised sum of T = 1 taps
	// came out up to ~5e-4 below 1 where the scene depth moves (the sea's waves: the clear sky flickered by one
	// half-float step), while a sum of alpha = 0 stays exactly 0. T below 0.5 is taken from the sum of T (the Sun's disk
	// behind a deck: relative precision), above from 1 - the sum of alpha.
	float sum_alpha = 0.0;
	float sum_alpha_positive = 0.0;
	float weights_positive = 0.0;
	float depth_weight_min = 1.0;
	float depth_weight_max = 0.0;
	float cloud_sum = 0.0;
	float cloud_weights = 0.0;
	vec4 best = vec4(0.0, 0.0, 0.0, 1.0);  // no cloud (T 1)
	float best_error = 1.0e30;
	// The taps' colours and depth weights: the lobes' undershoot floor (below)
	vec4 tap_color[16];
	float tap_depth_weight[16];
	for (int j = 0; j <= 3; j++) {
		for (int i = 0; i <= 3; i++) {
			ivec2 q = clamp(base + ivec2(i, j), ivec2(0), low - 1);
			vec2 d = abs(vec2(base + ivec2(i, j)) + 0.5 - position);
			float tent = radius > 0.0 ? max(0.0, 1.0 - d.x / radius) * max(0.0, 1.0 - d.y / radius)
					: cloud_catmull_rom(d.x) * cloud_catmull_rom(d.y);  // radius 0: Catmull-Rom
			vec4 c = texelFetch(low_color, q, 0);
			vec2 depths = texelFetch(low_depth, q, 0).rg;
			float march_m = min(depths.r, 1.0e20);
			float error = abs(log(max(scene_m, 1.0) / max(march_m, 1.0)));
			float depth_weight = exp(-CLOUD_DEPTH_SHARPNESS * error);
			float w = tent * depth_weight;
			sum += w * c;
			sum_alpha += w * (1.0 - c.a);
			weights += w;
			tap_color[j * 4 + i] = c;
			tap_depth_weight[j * 4 + i] = tent > 0.0 ? depth_weight : -1.0;
			if (tent > 0.0) {
				sum_positive += w * c;
				sum_alpha_positive += w * (1.0 - c.a);
				weights_positive += w;
			}
			if (tent != 0.0) {
				depth_weight_min = min(depth_weight_min, depth_weight);
				depth_weight_max = max(depth_weight_max, depth_weight);
			}
			if (depths.g > 0.0 && w > 0.0) {
				cloud_sum += w * (1.0 - c.a) * depths.g;  // .a: T
				cloud_weights += w * (1.0 - c.a);
			}
			if (tent > 0.0 && error < best_error) {
				best_error = error;
				best = c;
			}
		}
	}
	cloud = cloud_weights > 1.0e-6 ? cloud_sum / cloud_weights : 0.0;
	// Taps of two depths (the march's nearest / farthest checkerboard along a depth edge - the horizon seen from
	// altitude): the Catmull-Rom's negative lobes, renormalised over the taps of one depth, alternate with the
	// checkerboard and dotted the edge's row; its positive part alone (the nearest texels) is used there.
	if (depth_weight_min < CLOUD_DEPTH_MIXED_WEIGHT * depth_weight_max) {
		sum = sum_positive;
		sum_alpha = sum_alpha_positive;
		weights = weights_positive;
	}
	vec4 result = weights > 1.0e-4 ? sum / weights : best;
	if (weights > 1.0e-4 && result.a >= 0.5) {
		result.a = 1.0 - sum_alpha / weights;
	}
	result.a = clamp(result.a, 0.0, 1.0);
	// The Catmull-Rom lobes can over- / undershoot: premultiplied colour >= the lowest tap's (0 without air shadow:
	// the air shadow's loss is negative), T in [0, 1]. The floor from the taps of this pixel's depth only (depth
	// weight within CLOUD_DEPTH_MIXED_WEIGHT of the best): a near object's silhouette against the shadowed sky took
	// the sky's larger loss as its floor. And a tap's negative colour belongs to its T x scene (the loss of the air
	// the scene holds: the sky's -L at T 1, a cloud body's colour >= 0 at T 0), so the floor is also this pixel's T
	// times the taps' lowest colour per T: a body pixel next to the shadowed sky took the sky's -L through the filter
	// (final T x scene + colour < 0 at T ~ 0; cloud_check section 14). Both from the inner 2 x 2 taps (the texels
	// around the pixel): the outer ring's negative colour let the lobes' undershoot next to a bright cloud darken the
	// shadowed sky by 4-12 % of its loss, where without air shadow the floor 0 clamped it.
	vec3 lowest = vec3(0.0);
	vec3 lowest_per_t = vec3(0.0);
	for (int k = 0; k < 16; k++) {
		bool inner = (k & 3) == 1 || (k & 3) == 2;
		inner = inner && (k >> 2 == 1 || k >> 2 == 2);
		if (inner && tap_depth_weight[k] >= CLOUD_DEPTH_MIXED_WEIGHT * depth_weight_max) {
			lowest = min(lowest, tap_color[k].rgb);
			lowest_per_t = min(lowest_per_t, tap_color[k].rgb / max(tap_color[k].a, CLOUD_FLOOR_MIN_T));
		}
	}
	return vec4(max(result.rgb, max(lowest, result.a * lowest_per_t)), result.a);
}

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	ivec2 full = ivec2(cloud_view.sizes.xy);
	if (pixel.x >= full.x || pixel.y >= full.y) {
		return;
	}
	float scene_m = min(cloud_scene_distance(vec2(pixel) + 0.5, texelFetch(scene_depth, pixel, 0).r), 1.0e20);
	if (cloud_centre.params.x > 0.5) {
		// An eye reads the shared centre image at its own ray: first at its scene point, then again at the taps' cloud
		// distance (the cloud's own disparity; cloud_vr_stereo_test).
		vec3 dir = cloud_ray_earth(vec2(pixel) + 0.5);
		vec3 eye = cloud_view.eye_earth.xyz;
		float cloud_m;
		cloud_tent_gather(cloud_centre_position(eye + dir * min(scene_m, CLOUD_UPSAMPLE_SKY_M)), scene_m,
				CLOUD_TENT_RADIUS_MAX, cloud_m);
		float at_m = cloud_m > 0.0 ? min(cloud_m, scene_m) : min(scene_m, CLOUD_UPSAMPLE_SKY_M);
		float radius = cloud_tent_radius(cloud_m > 0.0 ? cloud_m : 1.0);  // no cloud: the largest radius
		if (cloud_centre.params.w > 0.5) {
			radius = 0.0;  // the deterministic march: Catmull-Rom
		}
		imageStore(out_image, pixel, cloud_tent_gather(cloud_centre_position(eye + dir * at_m), scene_m, radius,
				cloud_m));
		return;
	}
	ivec2 low = ivec2(cloud_view.sizes.zw);
	vec2 position = (vec2(pixel) + 0.5) * cloud_view.sizes.zw / cloud_view.sizes.xy - 0.5;
	if (cloud_centre.params.w > 0.5) {
		// The deterministic march: the Catmull-Rom gather on the view's own grid (the centre is the view).
		float cloud_m;
		imageStore(out_image, pixel, cloud_tent_gather(position + 0.5, scene_m, 0.0, cloud_m));
		return;
	}
	ivec2 base = ivec2(floor(position));
	vec2 f = position - vec2(base);
	vec4 sum = vec4(0.0);
	float weights = 0.0;
	vec4 best = vec4(0.0, 0.0, 0.0, 1.0);  // no cloud (T 1)
	float best_error = 1.0e30;
	for (int j = 0; j <= 1; j++) {
		for (int i = 0; i <= 1; i++) {
			ivec2 q = clamp(base + ivec2(i, j), ivec2(0), low - 1);
			vec4 c = texelFetch(low_color, q, 0);
			float d = min(texelFetch(low_depth, q, 0).r, 1.0e20);
			float error = abs(log(max(scene_m, 1.0) / max(d, 1.0)));
			float w = (i == 0 ? 1.0 - f.x : f.x) * (j == 0 ? 1.0 - f.y : f.y) * exp(-CLOUD_DEPTH_SHARPNESS * error);
			sum += w * c;
			weights += w;
			if (error < best_error) {
				best_error = error;
				best = c;
			}
		}
	}
	imageStore(out_image, pixel, weights > 1.0e-4 ? sum / weights : best);
}

#endif
