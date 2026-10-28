// The cloud view cache's kernels (cloud view cache Tasks 3-4): GLSL 450 only - compiled by CloudEffect through
// CloudShaderPort after cloud_pass.gdshaderinc, never included by a Godot shader.
//
// CLOUD_CACHE_FILL (with CLOUD_MARCH: the march's machinery and its set 0, without the march's own images and main):
// one invocation per texel of the picked tiles of one resolution level (CloudViewCache.fill: one dispatch per level,
// x = the 8 x 8 blocks of a tile, y = its entries of the tile list). The texel's direction is
// CloudViewCacheLayout.dir_of's (cube faces +X, -X, +Y, -Y, +Z, -Z of the Earth-local axes, tangent mapping); its ray
// starts at the anchor (CloudView eye_earth.xyz: the anchor's offset from the camera, differenced in float64 on the
// CPU) and ends at the static world seen from there - the ellipsoid / sea crossing, never the frame's depth buffer
// (scene geometry hides the cached clouds per frame). The shading is the march's own (cloud_shade_ray). Out per texel:
// premultiplied colour + the background's transmittance T (RGBA16F) and the alpha-weighted cloud distance (R32F).
// Task 4: an entry may name a fade slot - the texel's old colour is copied there first (the lookup cross-fades from
// it: a refresh at the same level, CloudViewCachePlanner) - and every entry's nearest cloud distance is kept (atomicMin
// of the float bits: positive floats order as their bits) for the schedule's parallax error (read back by the CPU).
//
// CLOUD_CACHE_LOOKUP (cloud view cache Task 4; replaces CLOUD_UPSAMPLE when the cache is on): one invocation per eye
// pixel. The eye's ray (CloudView of the eye) reads the cache from the eye's own position (cloud_cache_lookup: the
// parallax through the cached cloud distance), at the finest level its tile has, cross-faded from the tile's previous
// content over CLOUD_CACHE_CROSSFADE_S; scene geometry nearer than the cloud hides it (the upsample's depth weight).
// Out: the eye's layer of the pass's result (premultiplied colour + T), as the upsample's.
// Task 5: every tile carries the anchor it was rendered from (one of a ring, CloudViewCachePlanner: the cache
// re-anchors progressively) and the lookup reprojects each tile from its own anchor; the fill starts at the near
// shell's radius R_near, and with the near shell on (a cloud within R_near) the frame's march covers [0, the shell
// around the tile's anchor] (CLOUD_NEAR_SHELL: cloud_near_shell_end) and the lookup composites it in front of the
// cache.

#if defined(CLOUD_CACHE_FILL) || defined(CLOUD_CACHE_LOOKUP) || defined(CLOUD_NEAR_SHELL)

const uint CLOUD_CACHE_NO_SLOT = 0xFFFFFFFFu;  // a tile-list entry without a fade slot
// The fade slots' cells (CloudViewCachePlanner.SLOT_*): one full-level tile each, SLOT cells per atlas row.
const uint CLOUD_CACHE_LEVELS = 3u;  // CloudViewCacheLayout.LEVELS

// The unit direction of face coordinates uv (0..1 across the face; beyond: toward the neighbouring face):
// CloudViewCacheLayout.dir_of.
vec3 cloud_cache_dir(uint face, vec2 uv) {
	float s = 2.0 * uv.x - 1.0;
	float t = 2.0 * uv.y - 1.0;
	vec3 d;
	if (face == 0u) {
		d = vec3(1.0, -t, -s);
	} else if (face == 1u) {
		d = vec3(-1.0, -t, s);
	} else if (face == 2u) {
		d = vec3(s, 1.0, t);
	} else if (face == 3u) {
		d = vec3(s, -1.0, -t);
	} else if (face == 4u) {
		d = vec3(s, -t, 1.0);
	} else {
		d = vec3(-s, -t, -1.0);
	}
	return normalize(d);
}

// (x, y, face): the continuous texel coordinates (0..face_size) of a direction in its face (CloudViewCacheLayout
// texel_of).
vec3 cloud_cache_texel(vec3 dir, float face_size) {
	vec3 a = abs(dir);
	float face;
	float s;
	float t;
	float major;
	if (a.x >= a.y && a.x >= a.z) {
		major = a.x;
		face = dir.x > 0.0 ? 0.0 : 1.0;
		s = dir.x > 0.0 ? -dir.z : dir.z;
		t = -dir.y;
	} else if (a.y >= a.z) {
		major = a.y;
		face = dir.y > 0.0 ? 2.0 : 3.0;
		s = dir.x;
		t = dir.y > 0.0 ? dir.z : -dir.z;
	} else {
		major = a.z;
		face = dir.z > 0.0 ? 4.0 : 5.0;
		s = dir.z > 0.0 ? dir.x : -dir.x;
		t = -dir.y;
	}
	float scale = 0.5 * face_size / major;
	return vec3(clamp(vec2(s, t) * scale + 0.5 * face_size, vec2(0.0), vec2(face_size)), face);
}

// The tile of a face's texel position at the full level (full: its texels per face side).
uint cloud_cache_tile_in(vec3 texel, uint full, uint per_side) {
	float tile_texels = float(full / per_side);
	uvec2 xy = uvec2(clamp(ivec2(texel.xy / tile_texels), ivec2(0), ivec2(per_side) - 1));
	return (uint(texel.z) * per_side + xy.y) * per_side + xy.x;
}

// A tile state's fields (CloudViewCachePlanner): x = the shown level + 1 (bits 0-1; 0: none), what it fades from (bits
// 2-4: 0 nothing, 1-3 that level + 1, 4 its fade slot), the fade slot (bits 8-17), the anchor it shows (bits 18-24) and
// the anchor of what it fades from (bits 25-31); y = the fade's start (float bits, s of the planner's clock).
int cloud_cache_state_level(uvec2 state) {
	return int(state.x & 3u) - 1;
}

uint cloud_cache_state_anchor(uvec2 state) {
	return (state.x >> 18u) & 127u;
}

uint cloud_cache_state_from_anchor(uvec2 state) {
	return state.x >> 25u;
}

// The eye ray's exit (m along dir, unit) from the sphere of radius r around an anchor; o: the eye - the anchor
// (inside).
float cloud_cache_sphere_exit(vec3 o, vec3 dir, float r) {
	float b = dot(o, dir);
	return -b + sqrt(max(b * b - dot(o, o) + r * r, 0.0));
}

#endif

// The tile states, the anchor ring and the cached cloud distances, read by the near shell's march and by the lookup
// alike (Task 5 fix round 1: both kernels pick each eye ray's tile and anchor by the one rule, cloud_cache_pick) - the
// same names in both kernels, their bindings each kernel's own.
#ifdef CLOUD_NEAR_SHELL
#define CLOUD_CACHE_STATES_BINDING 8
#define CLOUD_CACHE_ANCHORS_BINDING 9
#define CLOUD_CACHE_DISTANCE_0_BINDING 10
#define CLOUD_CACHE_DISTANCE_1_BINDING 11
#define CLOUD_CACHE_DISTANCE_2_BINDING 12
layout(push_constant, std430) uniform CloudNearShell {
	uvec4 layout_;  // x: the full level's texels per face side, y: tiles per side, z: the current anchor
	vec4 radius;    // x: the near shell's radius R_near (m)
} cloud_near_shell;
#define CLOUD_CACHE_FULL cloud_near_shell.layout_.x
#define CLOUD_CACHE_PER_SIDE cloud_near_shell.layout_.y
#define CLOUD_CACHE_CURRENT cloud_near_shell.layout_.z
#endif

#ifdef CLOUD_CACHE_LOOKUP
#define CLOUD_CACHE_STATES_BINDING 9
#define CLOUD_CACHE_ANCHORS_BINDING 11
#define CLOUD_CACHE_DISTANCE_0_BINDING 5
#define CLOUD_CACHE_DISTANCE_1_BINDING 6
#define CLOUD_CACHE_DISTANCE_2_BINDING 7
layout(push_constant, std430) uniform CloudCacheLookup {
	vec4 clock;    // x: now (s)
	// x: the full level's texels per face side, y: tiles per side, z: fade slots per atlas row, w: bit 0 covering (the
	// march's whole rays ran into out_image this frame), bit 1 the near shell (its result is in out_image), bits 8-15
	// the current anchor
	uvec4 layout_;
} cloud_cache_lookup_block;
#define CLOUD_CACHE_FULL cloud_cache_lookup_block.layout_.x
#define CLOUD_CACHE_PER_SIDE cloud_cache_lookup_block.layout_.y
#define CLOUD_CACHE_CURRENT ((cloud_cache_lookup_block.layout_.w >> 8u) & 127u)
#endif

#if defined(CLOUD_CACHE_LOOKUP) || defined(CLOUD_NEAR_SHELL)

layout(set = 1, binding = CLOUD_CACHE_DISTANCE_0_BINDING) uniform samplerCube cache_distance_0;  // per level, nearest
layout(set = 1, binding = CLOUD_CACHE_DISTANCE_1_BINDING) uniform samplerCube cache_distance_1;
layout(set = 1, binding = CLOUD_CACHE_DISTANCE_2_BINDING) uniform samplerCube cache_distance_2;
layout(set = 1, binding = CLOUD_CACHE_STATES_BINDING, std430) readonly buffer CloudCacheStates {
	uvec2 states[];  // per tile: its shown state (cloud_cache_state_*)
} cloud_cache_states;
layout(set = 1, binding = CLOUD_CACHE_ANCHORS_BINDING, std430) readonly buffer CloudCacheAnchors {
	vec4 offsets[];  // per anchor of the ring: xyz the camera (head) - the anchor, Earth-local (m, float64 differences)
} cloud_cache_anchors;

// [E] Iterations of the parallax lookup (cloud_cache_parallax; cloud_vr_stereo_test mirrors it): two fixed-point steps
// and secant steps - a cloud at 2 km and beyond lands within 1e-8 eye px for the eyes and a camera 50 m off the anchor.
const int CACHE_PARALLAX_ITERATIONS = 3;
// [E] Parallax solves of one eye ray at most until the tile it lands on shows the anchor it was solved from
// (cloud_cache_pick; cloud_vr_stereo_test mirrors it): the current anchor, then the landed tile's - a third solve
// settles a ray that crossed two anchor boundaries; one still unsettled lies between tiles of different anchors.
const int CACHE_ANCHOR_ITERATIONS = 3;
// Distance [m] of a cached texel without cloud: the far sky, rotation only (as CLOUD_UPSAMPLE_SKY_M).
const float CLOUD_CACHE_FAR_M = 1.0e7;

// The tile of a face's texel position at the full level.
uint cloud_cache_tile(vec3 texel) {
	return cloud_cache_tile_in(texel, CLOUD_CACHE_FULL, CLOUD_CACHE_PER_SIDE);
}

float cloud_cache_distance_sample(int level, vec3 q) {
	if (level == 0) {
		return textureLod(cache_distance_0, q, 0.0).r;
	} else if (level == 1) {
		return textureLod(cache_distance_1, q, 0.0).r;
	}
	return textureLod(cache_distance_2, q, 0.0).r;
}

// The cached cloud distance (m, from the anchor; CLOUD_CACHE_FAR_M without cloud or without a level) along q.
float cloud_cache_distance(vec3 q) {
	int level = cloud_cache_state_level(cloud_cache_states.states[cloud_cache_tile(cloud_cache_texel(q,
			float(CLOUD_CACHE_FULL)))]);
	float r = level >= 0 ? cloud_cache_distance_sample(level, q) : 0.0;
	return r > 0.0 ? r : CLOUD_CACHE_FAR_M;
}

// The parallax lookup from one anchor: the distance s along the eye ray (o = the eye - the anchor, d its unit
// direction) whose point the cache holds - the root of g(s) = r(q(s)) - |o + s d| (q(s) the direction of o + s d, r
// the cached distance from the anchor): two fixed-point steps (the ray's point at the cached distance from the anchor),
// then secant steps while both ends saw cloud (cloud_vr_stereo_test mirrors it).
float cloud_cache_parallax(vec3 o, vec3 d) {
	float b = dot(o, d);
	float c = dot(o, o);
	float r = cloud_cache_distance(d);
	float s_prev = -b + sqrt(max(b * b - c + r * r, 0.0));
	r = cloud_cache_distance(normalize(o + d * s_prev));
	float g_prev = r < CLOUD_CACHE_FAR_M ? r - length(o + d * s_prev) : 0.0;
	bool cloud_prev = r < CLOUD_CACHE_FAR_M;
	float s = -b + sqrt(max(b * b - c + r * r, 0.0));
	for (int i = 1; i < CACHE_PARALLAX_ITERATIONS; i++) {
		r = cloud_cache_distance(normalize(o + d * s));
		bool cloud = r < CLOUD_CACHE_FAR_M;
		float g = cloud ? r - length(o + d * s) : 0.0;
		float next = -b + sqrt(max(b * b - c + r * r, 0.0));
		if (cloud && cloud_prev && g != g_prev) {
			float secant = s - g * (s - s_prev) / (g - g_prev);
			next = secant > 0.0 ? secant : next;
		}
		s_prev = s;
		g_prev = g;
		cloud_prev = cloud;
		s = next;
	}
	return s;
}

// The one rule of both kernels (Task 5 fix round 1; cloud_vr_stereo_test mirrors it): the tile an eye ray reads and
// the anchor it is reprojected from. The parallax lookup from the current anchor lands on a tile; where that tile shows
// another anchor, again from that one - until the tile landed on shows the anchor solved from (at most
// CACHE_ANCHOR_ITERATIONS solves). eye_offset: the eye's offset from the head (camera), Earth-local; d: the eye ray.
// Out: the tile, its anchor and state, s (m along the eye ray to the cached point), q (its direction from the anchor)
// and its texel position; stable: the anchor settled. Returns the tile's level, -1 where it has none. Not settled
// (stable false), the ray's point lies between tiles of different anchors (neither holds it): the last tile landed on,
// solved from its own anchor (q just outside it: the colour clamps to its edge) - the kernels that march (the near
// shell, covering) march such a ray whole instead; -1 when q left that tile's face.
int cloud_cache_pick(vec3 eye_offset, vec3 d, out uint tile, out uint anchor, out uvec2 state, out float s, out vec3 q,
		out vec3 texel, out bool stable) {
	float full = float(CLOUD_CACHE_FULL);
	anchor = CLOUD_CACHE_CURRENT;
	stable = false;
	int level = -1;
	for (int k = 0; k < CACHE_ANCHOR_ITERATIONS; k++) {
		vec3 o = cloud_cache_anchors.offsets[anchor].xyz + eye_offset;
		s = cloud_cache_parallax(o, d);
		q = normalize(o + d * s);
		texel = cloud_cache_texel(q, full);
		tile = cloud_cache_tile(texel);
		state = cloud_cache_states.states[tile];
		level = cloud_cache_state_level(state);
		if (level < 0) {
			return -1;
		}
		uint shown = cloud_cache_state_anchor(state);
		if (shown == anchor) {
			stable = true;
			return level;
		}
		anchor = shown;
	}
	vec3 o = cloud_cache_anchors.offsets[anchor].xyz + eye_offset;
	s = cloud_cache_parallax(o, d);
	q = normalize(o + d * s);
	texel = cloud_cache_texel(q, full);
	uint per_side = CLOUD_CACHE_PER_SIDE;
	return uint(texel.z) == tile / (per_side * per_side) ? level : -1;
}

#endif

#ifdef CLOUD_NEAR_SHELL

// The near shell's end of the eye ray (m from the eye; the eye's offset from the head and the ray, Earth-local): its
// exit from the sphere of R_near around the anchor cloud_cache_pick names for the ray - the anchor of the tile the
// lookup reads (that tile's content starts at the sphere: the near shell and the cache meet without a gap or an
// overlap); -1 where the ray reads no level or no settled anchor (the march's whole ray, which the lookup keeps).
float cloud_near_shell_end(vec3 eye_offset, vec3 dir) {
	uint tile;
	uint anchor;
	uvec2 state;
	float s;
	vec3 q;
	vec3 texel;
	bool stable;
	if (cloud_cache_pick(eye_offset, dir, tile, anchor, state, s, q, texel, stable) < 0 || !stable) {
		return -1.0;
	}
	return cloud_cache_sphere_exit(cloud_cache_anchors.offsets[anchor].xyz + eye_offset, dir,
			cloud_near_shell.radius.x);
}

#endif

#ifdef CLOUD_CACHE_FILL

layout(rgba16f, set = 1, binding = 1) uniform restrict imageCube cache_color;  // read too: the fade slot's copy
layout(r32f, set = 1, binding = 2) uniform writeonly imageCube cache_distance;
layout(set = 1, binding = 3, std430) readonly buffer CloudCacheTiles {
	uvec2 tiles[];  // (tile index (CloudViewCacheLayout: (face * tiles_per_side + ty) * tiles_per_side + tx), slot)
} cloud_cache_tiles;
layout(set = 1, binding = 4, std430) buffer CloudCacheNear {
	uint near[];  // per entry: the float bits of its nearest cloud distance (m; +inf: none)
} cloud_cache_near;
layout(rgba16f, set = 1, binding = 5) uniform writeonly image2D cache_slots;  // the fade slots' atlas
layout(push_constant, std430) uniform CloudCacheFill {
	uint face_texels;     // this level's texels per face side
	uint tiles_per_side;
	uint first;           // the level's first entry of the tile list
	uint slots_per_row;   // the fade slots' cells per atlas row
	float start_m;        // the rays' start from the anchor (m; Task 5: the near shell's radius, 0: the whole ray)
} cloud_cache_fill;

const uint CLOUD_CACHE_BLOCK = 8u;  // the work group's side (local_size 8 x 8)
// [E] The sky's distance where the ray from the anchor meets no static surface.
const float CLOUD_CACHE_SKY_M = 1.0e30;
// [E] The ground's depth under an anchor below the ellipsoid (m): a standing eye's height (the ground view's 2 m).
const float CLOUD_CACHE_GROUND_BELOW_M = 2.0;
// The stored end of a ray without a static surface (m): the lookup's far sky (CLOUD_CACHE_FAR_M).
const float CLOUD_CACHE_END_MAX_M = 1.0e7;
// The fade slots' cell side (texels): the full level's tile (CloudViewCacheLayout tile_texels, CloudViewCache).
const uint CLOUD_CACHE_SLOT_TEXELS = 64u;

void main() {
	uint entry = cloud_cache_fill.first + gl_WorkGroupID.y;
	uvec2 job = cloud_cache_tiles.tiles[entry];
	uint tile = job.x;
	uint per_side = cloud_cache_fill.tiles_per_side;
	uint face = tile / (per_side * per_side);
	uvec2 tile_xy = uvec2(tile % per_side, (tile / per_side) % per_side);
	uint tile_texels = cloud_cache_fill.face_texels / per_side;
	uint blocks = tile_texels / CLOUD_CACHE_BLOCK;
	uvec2 block_origin = tile_xy * tile_texels
			+ uvec2(gl_WorkGroupID.x % blocks, gl_WorkGroupID.x / blocks) * CLOUD_CACHE_BLOCK;
	uvec2 texel = block_origin + gl_LocalInvocationID.xy;
	float face_size = float(cloud_cache_fill.face_texels);
	vec3 origin = cloud_view.eye_earth.xyz;
	// The work group's eclipse factors along its block's centre ray (as the march's work groups).
	if (gl_LocalInvocationIndex == 0u) {
		vec3 group_dir = cloud_cache_dir(face, (vec2(block_origin) + 0.5 * float(CLOUD_CACHE_BLOCK)) / face_size);
		cloud_eclipse_factors(group_dir, origin, cloud_group_eclipse_near, cloud_group_eclipse_far);
	}
	barrier();
	vec3 dir = cloud_cache_dir(face, (vec2(texel) + 0.5) / face_size);
	// The static world's end: the ellipsoid (the sea) ahead of the anchor; for an anchor below it (a shore below sea
	// level: Starbase's ground lies ~25 m under the ellipsoid) a sphere CLOUD_CACHE_GROUND_BELOW_M under the anchor -
	// the ground around it, not the air and clouds beyond the planet's limb (their shadow loss lay on the terrain).
	CloudRay anchor_ray = cloud_ray_from(dir, origin);
	vec2 ground = cloud_sphere_crossing(anchor_ray, min(0.0, anchor_ray.h_c - CLOUD_CACHE_GROUND_BELOW_M));
	float scene_m = ground.x <= ground.y && ground.x > 0.0 ? ground.x : CLOUD_CACHE_SKY_M;
	vec4 color_t;
	float cloud_m;
	cloud_march_start_m = cloud_cache_fill.start_m;
	cloud_shade_ray(dir, origin, scene_m, ivec2(texel), cloud_group_eclipse_near, cloud_group_eclipse_far, color_t,
			cloud_m);
	if (job.y != CLOUD_CACHE_NO_SLOT) {
		// The texel's shown colour to its tile's fade slot (the lookup fades from it to the new one).
		uvec2 cell = uvec2(job.y % cloud_cache_fill.slots_per_row, job.y / cloud_cache_fill.slots_per_row);
		imageStore(cache_slots, ivec2(cell * CLOUD_CACHE_SLOT_TEXELS + texel - tile_xy * tile_texels),
				imageLoad(cache_color, ivec3(texel, face)));
	}
	imageStore(cache_color, ivec3(texel, face), color_t);
	// Without cloud the static end's distance, negated (the lookup scales the air's shadow loss to nearer geometry).
	imageStore(cache_distance, ivec3(texel, face),
			vec4(cloud_m > 0.0 ? cloud_m : -min(scene_m, CLOUD_CACHE_END_MAX_M), 0.0, 0.0, 0.0));
	if (cloud_m > 0.0) {
		atomicMin(cloud_cache_near.near[entry], floatBitsToUint(cloud_m));
	}
}

#endif

#ifdef CLOUD_CACHE_LOOKUP

layout(set = 1, binding = 1) uniform sampler2D scene_depth;  // the eye's depth, full resolution, nearest
layout(set = 1, binding = 2) uniform samplerCube cache_color_0;  // per level: colour + T, linear
layout(set = 1, binding = 3) uniform samplerCube cache_color_1;
layout(set = 1, binding = 4) uniform samplerCube cache_color_2;
// (bindings 5 - 7: the cloud distance per level, 9: the tile states, 11: the anchor ring - declared above)
layout(set = 1, binding = 8) uniform sampler2D cache_slots;  // the fade slots' atlas, linear
// read too: while covering, the per-frame march's result is there (the lookup keeps it where a tile has nothing), and
// with the near shell its result (composited in front of the cache)
layout(rgba16f, set = 1, binding = 10) uniform restrict image2D out_image;

// [E] The cross-fade of a tile's new content (a finer level, a refresh): a quarter second is no visible pop and lags a
// moving light by less than its refresh step (CloudViewCacheSchedule.SUN_REFRESH_RAD at time warp).
const float CACHE_CROSSFADE_S = 0.25;
// [E] Sharpness of the depth test (the upsample's CLOUD_DEPTH_SHARPNESS): scene geometry 28 % nearer than the cloud
// leaves 37 % of it.
const float CLOUD_CACHE_DEPTH_SHARPNESS = 4.0;
const uint CLOUD_CACHE_SLOT_TEXELS = 64u;  // as the fill's

// A level's colour + T at q, bilinear inside the tile's level where a neighbouring tile does not hold that level or
// shows another anchor (its texels there are old, empty or seen from elsewhere): the position is clamped to this
// tile's outer texel centres on that side.
vec4 cloud_cache_colour(int level, vec3 q, vec3 texel_full, uint tile) {
	uint anchor = cloud_cache_state_anchor(cloud_cache_states.states[tile]);
	uint full = CLOUD_CACHE_FULL;
	uint per_side = CLOUD_CACHE_PER_SIDE;
	float n = float(full >> (CLOUD_CACHE_LEVELS - 1u - uint(level)));
	float tile_n = n / float(per_side);
	uint face = uint(texel_full.z);
	vec2 t = texel_full.xy * (n / float(full));
	vec2 origin = vec2(uvec2(tile % per_side, (tile / per_side) % per_side)) * tile_n;
	vec2 lo = origin + 0.5;
	vec2 hi = origin + tile_n - 0.5;
	vec2 side = vec2(t.x < lo.x ? -1.0 : (t.x > hi.x ? 1.0 : 0.0), t.y < lo.y ? -1.0 : (t.y > hi.y ? 1.0 : 0.0));
	vec2 c = t;
	if (side != vec2(0.0)) {
		bvec2 clamp_axis = bvec2(false);
		vec2 centre = origin + 0.5 * tile_n;
		for (int k = 0; k < 3; k++) {
			// the side neighbours along x, along y, then the diagonal one
			vec2 step_ = k == 0 ? vec2(side.x, 0.0) : (k == 1 ? vec2(0.0, side.y) : side);
			if ((k == 0 && side.x == 0.0) || (k == 1 && side.y == 0.0)
					|| (k == 2 && (side.x == 0.0 || side.y == 0.0))) {
				continue;
			}
			vec3 beyond = cloud_cache_dir(face, (centre + step_ * tile_n) / n);
			uvec2 other = cloud_cache_states.states[cloud_cache_tile(cloud_cache_texel(beyond, float(full)))];
			if (cloud_cache_state_level(other) < level || cloud_cache_state_anchor(other) != anchor) {
				clamp_axis = bvec2(clamp_axis.x || step_.x != 0.0, clamp_axis.y || step_.y != 0.0);
			}
		}
		c = vec2(clamp_axis.x ? clamp(t.x, lo.x, hi.x) : t.x, clamp_axis.y ? clamp(t.y, lo.y, hi.y) : t.y);
	}
	vec3 p = c == t ? q : cloud_cache_dir(face, c / n);
	if (level == 0) {
		return textureLod(cache_color_0, p, 0.0);
	} else if (level == 1) {
		return textureLod(cache_color_1, p, 0.0);
	}
	return textureLod(cache_color_2, p, 0.0);
}

// The eye ray's cached cloud (premultiplied colour + T) and its distance from the eye (cloud_m; without cloud <= 0:
// minus the cached ray's static end from the anchor, the air's shadow loss alone). eye_offset_earth: the eye's offset
// from the head (camera), Earth-local. Each tile is reprojected from the anchor it was rendered from (Task 5, the
// progressive re-anchor): the tile and the anchor by cloud_cache_pick (the near shell's rule too). The colour: the
// finest level the tile has, cross-faded from what it showed before over CACHE_CROSSFADE_S (what it showed is read at
// the same point of the eye ray seen from its own anchor); a tile without a level shows no cloud (T = 1). stable: the
// pick's anchor settled (else the march, where it ran, keeps the ray: CLOUD_CACHE_LOOKUP's main).
vec4 cloud_cache_sample(vec3 eye_offset_earth, vec3 dir_earth, out float cloud_m, out float fresh, out bool stable);
vec4 cloud_cache_lookup(vec3 eye_offset_earth, vec3 dir_earth, out float cloud_m) {
	float fresh;
	bool stable;
	vec4 sampled = cloud_cache_sample(eye_offset_earth, dir_earth, cloud_m, fresh, stable);
	return mix(vec4(0.0, 0.0, 0.0, 1.0), sampled, fresh);
}

// cloud_cache_lookup without the fade from nothing: fresh is the weight of the returned colour against what shows
// where the tile has nothing (0 without a level, the fade's weight while a first fill fades in, else 1) - T = 1, or the
// covering march's result (CLOUD_CACHE_LOOKUP's main).
vec4 cloud_cache_sample(vec3 eye_offset_earth, vec3 dir_earth, out float cloud_m, out float fresh, out bool stable) {
	vec3 d = dir_earth;
	uint full = CLOUD_CACHE_FULL;
	uint tile;
	uint anchor;
	uvec2 state;
	float s;
	vec3 q;
	vec3 texel;
	int level = cloud_cache_pick(eye_offset_earth, d, tile, anchor, state, s, q, texel, stable);
	cloud_m = 0.0;
	fresh = 0.0;
	if (level < 0) {
		return vec4(0.0, 0.0, 0.0, 1.0);
	}
	fresh = 1.0;
	vec4 to = cloud_cache_colour(level, q, texel, tile);
	float w = clamp((cloud_cache_lookup_block.clock.x - uintBitsToFloat(state.y)) / CACHE_CROSSFADE_S, 0.0, 1.0);
	uint from = (state.x >> 2u) & 7u;
	if (w < 1.0 && from == 0u) {
		fresh = w;  // a first fill fades in from what shows without the cache
	} else if (w < 1.0) {
		// What the tile showed was rendered from its previous anchor: the same point of the eye ray seen from there.
		vec3 q_before = q;
		vec3 texel_before = texel;
		uint from_anchor = cloud_cache_state_from_anchor(state);
		if (from_anchor != cloud_cache_state_anchor(state)) {
			q_before = normalize(cloud_cache_anchors.offsets[from_anchor].xyz + eye_offset_earth + d * s);
			texel_before = cloud_cache_texel(q_before, float(full));
		}
		vec4 before = vec4(0.0, 0.0, 0.0, 1.0);
		if (from >= 1u && from <= CLOUD_CACHE_LEVELS) {
			before = cloud_cache_colour(int(from) - 1, q_before, texel_before, tile);
		} else if (from == 4u) {
			// The slot holds the tile's texels of this level before the refresh (the fill's copy).
			uint slot = (state.x >> 8u) & 1023u;
			uint per_side = CLOUD_CACHE_PER_SIDE;
			float tile_n = float(full >> (CLOUD_CACHE_LEVELS - 1u - uint(level))) / float(per_side);
			vec2 local = texel_before.xy * (tile_n * float(per_side) / float(full))
					- vec2(uvec2(tile % per_side, (tile / per_side) % per_side)) * tile_n;
			uint per_row = cloud_cache_lookup_block.layout_.z;
			vec2 cell = vec2(uvec2(slot % per_row, slot / per_row)) * float(CLOUD_CACHE_SLOT_TEXELS);
			before = textureLod(cache_slots, (cell + clamp(local, vec2(0.5), vec2(tile_n - 0.5)))
					/ vec2(textureSize(cache_slots, 0)), 0.0);
		}
		to = mix(before, to, w);
	}
	float stored = cloud_cache_distance_sample(level, q);
	cloud_m = stored > 0.0 ? s : stored;
	return to;
}

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	if (pixel.x >= int(cloud_view.sizes.x) || pixel.y >= int(cloud_view.sizes.y)) {
		return;
	}
	float scene_m = cloud_scene_distance(vec2(pixel) + 0.5, texelFetch(scene_depth, pixel, 0).r);
	vec3 dir = cloud_ray_earth(vec2(pixel) + 0.5);
	float cloud_m;
	float fresh;
	bool stable;
	vec4 result = cloud_cache_sample(cloud_view.eye_earth.xyz, dir, cloud_m, fresh, stable);
	uint flags = cloud_cache_lookup_block.layout_.w;
	bool covering = (flags & 1u) != 0u;
	bool near_shell = (flags & 2u) != 0u;
	// No level or no settled anchor: the march's result stays (its whole ray there). With the near shell exactly where
	// its kernel marched the whole ray (cloud_near_shell_end: the same pick, no settled anchor - a ray without a level
	// has none); a first fill there (fresh 0 at the frame it is shown) is composited behind the shell's segment, which
	// ended at the sphere (Task 5 fix round 2: keeping that segment alone left the deck's clouds out after a jump).
	if (near_shell ? !stable : ((fresh <= 0.0 || !stable) && covering)) {
		return;
	}
	// Scene geometry in front of the cloud hides it, softly over the distance ratio (the upsample's depth weight). A
	// texel without cloud holds the air's shadow loss along the cached ray to its static end: nearer geometry (a hill
	// before the horizon) keeps the share of its distance.
	if (cloud_m > 0.0 && scene_m < cloud_m) {
		float v = exp(-CLOUD_CACHE_DEPTH_SHARPNESS * log(cloud_m / max(scene_m, 1.0)));
		result = vec4(result.rgb * v, 1.0 - v * (1.0 - result.a));
	} else if (cloud_m < 0.0 && scene_m < -cloud_m) {
		result.rgb *= scene_m / -cloud_m;
	}
	if (near_shell) {
		// The near shell (Task 5) in front of the cache: its march ended at the shell this tile's content starts at
		// (premultiplied: near + T_near x cache). A first fill shows at once here (the march's rays end at the shell:
		// there is no whole-ray result to fade from; the cache beyond R_near differs from it only by its sampling).
		vec4 near = imageLoad(out_image, pixel);
		result = vec4(near.rgb + near.a * result.rgb, near.a * result.a);
	} else if (fresh < 1.0) {
		// A first fill fades in from what shows without the cache (both after the depth test: it is affine, T = 1
		// stays): the covering march's result (already in the output), else no cloud.
		result = mix(covering ? imageLoad(out_image, pixel) : vec4(0.0, 0.0, 0.0, 1.0), result, fresh);
	}
	imageStore(out_image, pixel, result);
}

#endif
