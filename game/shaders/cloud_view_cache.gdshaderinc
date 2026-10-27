// The cloud view cache's fill (cloud view cache Task 3): GLSL 450 only - compiled by CloudEffect through CloudShaderPort
// after cloud_pass.gdshaderinc with CLOUD_MARCH and CLOUD_CACHE_FILL defined (the march's machinery and its set 0,
// without the march's own images and main), never included by a Godot shader.
//
// CLOUD_CACHE_FILL: one invocation per texel of the picked tiles of one resolution level (CloudViewCache.fill: one
// dispatch per level, x = the 8 x 8 blocks of a tile, y = its entries of the tile list). The texel's direction is
// CloudViewCacheLayout.dir_of's (cube faces +X, -X, +Y, -Y, +Z, -Z of the Earth-local axes, tangent mapping); its ray
// starts at the anchor (CloudView eye_earth.xyz: the anchor's offset from the camera, differenced in float64 on the
// CPU) and ends at the static world seen from there - the ellipsoid / sea crossing, never the frame's depth buffer
// (scene geometry hides the cached clouds per frame). The shading is the march's own (cloud_shade_ray). Out per texel:
// premultiplied colour + the background's transmittance T (RGBA16F) and the alpha-weighted cloud distance (R32F).

#ifdef CLOUD_CACHE_FILL

layout(rgba16f, set = 1, binding = 1) uniform writeonly imageCube cache_color;
layout(r32f, set = 1, binding = 2) uniform writeonly imageCube cache_distance;
layout(set = 1, binding = 3, std430) readonly buffer CloudCacheTiles {
	uint tiles[];  // tile indices (CloudViewCacheLayout: (face * tiles_per_side + ty) * tiles_per_side + tx)
} cloud_cache_tiles;
layout(push_constant, std430) uniform CloudCacheFill {
	uint face_texels;     // this level's texels per face side
	uint tiles_per_side;
	uint first;           // the level's first entry of the tile list
	uint pad;
} cloud_cache_fill;

const uint CLOUD_CACHE_BLOCK = 8u;  // the work group's side (local_size 8 x 8)
// [E] The sky's distance where the ray from the anchor meets no static surface.
const float CLOUD_CACHE_SKY_M = 1.0e30;

// The unit direction of face coordinates uv (0..1 across the face): CloudViewCacheLayout.dir_of.
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

void main() {
	uint tile = cloud_cache_tiles.tiles[cloud_cache_fill.first + gl_WorkGroupID.y];
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
	// The static world's end: the ellipsoid (the sea) ahead of the anchor; an anchor below it (a shore below sea level)
	// has none.
	vec2 ground = cloud_sphere_crossing(cloud_ray_from(dir, origin), 0.0);
	float scene_m = ground.x <= ground.y && ground.x > 0.0 ? ground.x : CLOUD_CACHE_SKY_M;
	vec4 color_t;
	float cloud_m;
	cloud_shade_ray(dir, origin, scene_m, ivec2(texel), cloud_group_eclipse_near, cloud_group_eclipse_far, color_t,
			cloud_m);
	imageStore(cache_color, ivec3(texel, face), color_t);
	imageStore(cache_distance, ivec3(texel, face), vec4(cloud_m, 0.0, 0.0, 0.0));
}

#endif
