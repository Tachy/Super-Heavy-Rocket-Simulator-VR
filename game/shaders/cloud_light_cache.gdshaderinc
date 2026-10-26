// The light cache (static clouds, Task 1): the optical depth toward the Sun tau_light (the quantity
// cloud_light_optical_depth returns) at the voxel centres of an Earth-fixed box over the cloud shell, filled once by
// the CLOUD_LIGHT_FILL kernel (CloudLightCache) and only looked up while marching (cloud_pass.gdshaderinc
// cloud_light_source): the light a cloud sample receives depends on the Sun, the weather field and the point - not on
// the camera. GLSL 450 through CloudShaderPort (its `global uniform`s in set 0); include atmosphere.gdshaderinc and
// clouds.gdshaderinc first.
//
// The box (one cascade): horizontal axes east / north of the tangent plane at its anchor (a point on the ground below
// the camera when it was filled), cloud_light_cache_extent.xy wide and centred on the anchor; vertically the height
// above the ellipsoid from cloud_light_cache_extent.z to .w (the shell). A point maps to (its offset from the anchor
// along east, along north, its height) - the voxel of the fill holds exactly that point (the fill raises it by the
// curvature drop r^2 / 2 (a + h)). Voxel values belong to voxel centres; the lookup is trilinear (hardware) between
// them and returns -1.0 for a point outside the centres' range (the caller marches: the result is the march's).
// The fill marches toward the light with a converged, deterministic sample set (CloudLightCache.SAMPLES near + far,
// the stratified midpoints: jitter 0.5), the detail band-limited at least at the voxel's horizontal size (finer
// octaves cannot be held by the voxels; the tables keep their mean).

// ---- Globals (CloudLayer: the fidelity flag; CloudEffect: the rest, from the cache's state on the render thread).
global uniform sampler3D cloud_light_cache : filter_linear, repeat_disable;  // R32F tau_light
global uniform bool cloud_light_cache_enabled;  // fidelity flag (false: the light march everywhere, the reference)
global uniform bool cloud_light_cache_valid;    // the cache holds a complete fill for this frame's Sun
global uniform vec3 cloud_light_cache_origin;   // the anchor - the camera (Earth-local axes, m; float64 on the CPU)
global uniform vec3 cloud_light_cache_east;     // the box's horizontal axes (unit, Earth-local)
global uniform vec3 cloud_light_cache_north;
global uniform vec4 cloud_light_cache_extent;   // (east extent m, north extent m, bottom height m, top height m)
global uniform vec3 cloud_light_cache_size;     // voxels (east, north, up)

// tau_light toward the Sun at camera + offset_local with its height, from the cache; -1.0 where the cache does not
// hold the point (off, not filled, outside the voxel centres).
float cloud_cached_light_depth(vec3 offset_local, float height_m) {
	if (!cloud_light_cache_enabled || !cloud_light_cache_valid) {
		return -1.0;
	}
	vec3 rel = offset_local - cloud_light_cache_origin;
	vec3 f = vec3(dot(rel, cloud_light_cache_east) / cloud_light_cache_extent.x + 0.5,
			dot(rel, cloud_light_cache_north) / cloud_light_cache_extent.y + 0.5,
			(height_m - cloud_light_cache_extent.z) / (cloud_light_cache_extent.w - cloud_light_cache_extent.z));
	vec3 voxel = f * cloud_light_cache_size - 0.5;  // in voxel-centre units: 0 .. size - 1 inside
	if (any(lessThan(voxel, vec3(0.0))) || any(greaterThan(voxel, cloud_light_cache_size - 1.0))) {
		return -1.0;
	}
	return textureLod(cloud_light_cache, f, 0.0).r;
}

// =====================================================================================================================
#ifdef CLOUD_LIGHT_FILL

layout(local_size_x = 4, local_size_y = 4, local_size_z = 4) in;

// The fill's frame (CloudLightCache.fill; Earth-local axes, m).
layout(set = 1, binding = 0, std140) uniform CloudLightFill {
	vec4 origin;   // xyz: the anchor - the camera; w: the detail's footprint floor (the voxel's horizontal size)
	vec4 east;     // xyz: unit; w: the first layer (z) of this dispatch
	vec4 north;    // xyz: unit; w: the end layer (exclusive)
	vec4 up;       // xyz: the anchor's geodetic up
	vec4 extent;   // as cloud_light_cache_extent
	vec4 light;    // xyz: unit direction toward the light
	vec4 march;    // the intervals (as cloud_light_march)
	ivec4 samples; // xy: near, far samples
} cloud_fill;

layout(r32f, set = 1, binding = 1) uniform writeonly image3D out_light_depth;

void main() {
	ivec3 size = imageSize(out_light_depth);
	ivec3 voxel = ivec3(gl_GlobalInvocationID) + ivec3(0, 0, int(cloud_fill.east.w));
	if (voxel.x >= size.x || voxel.y >= size.y || voxel.z >= min(size.z, int(cloud_fill.north.w))) {
		return;
	}
	vec3 f = (vec3(voxel) + 0.5) / vec3(size);
	float x = (f.x - 0.5) * cloud_fill.extent.x;
	float y = (f.y - 0.5) * cloud_fill.extent.y;
	float h = mix(cloud_fill.extent.z, cloud_fill.extent.w, f.z);
	float drop = (x * x + y * y) / (2.0 * (CLOUD_A_M + h));  // the sphere below the tangent plane
	vec3 offset = cloud_fill.origin.xyz + cloud_fill.east.xyz * x + cloud_fill.north.xyz * y
			+ cloud_fill.up.xyz * (h + drop);
	float tau = cloud_light_optical_depth_with(offset, h, 0.0, cloud_fill.light.xyz, 0.5, 0.0, 0.0, cloud_fill.march,
			cloud_fill.samples.xy, cloud_fill.origin.w);
	imageStore(out_light_depth, voxel, vec4(tau, 0.0, 0.0, 0.0));
}

#endif
