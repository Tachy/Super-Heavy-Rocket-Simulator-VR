// Geomorphing of terrain quadtree nodes (TerrainQuadtree), shared by the Earth and Moon terrain shaders.
// A node's vertices slide towards the parent level's surface (CUSTOM0.xyz) and their normals towards the parent level's
// normals (CUSTOM3.xyz) as the camera moves away, so a split or merge never pops: morph is 0 near the camera and 1
// from (1.4 + 0.55) K s on, K = lod_factor (TerrainQuadtree.LOD_FACTOR), s = the node's size. Children appear at K and
// vanish at 1.15 K times their parent's size (about 2 s) from the parent's bounding sphere, where they are fully morphed.

// Camera position relative to the node centre (node-local render axes, metres); set per node and frame.
instance uniform vec3 camera_local = vec3(0.0);
// Edge length of the node (m), CubeSphere.node_size_m.
instance uniform float node_size_m = 1.0;
uniform float lod_factor = 3.0;

// Morph factor of a vertex at node-local position `vertex`.
float terrain_morph_factor(vec3 vertex) {
	float k_s = lod_factor * node_size_m;
	return clamp((length(vertex - camera_local) - 1.4 * k_s) / (0.55 * k_s), 0.0, 1.0);
}
