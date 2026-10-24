// Geomorphing of terrain quadtree nodes (TerrainQuadtree), shared by the Earth and Moon terrain shaders.
// A node's vertices slide towards the parent level's surface (CUSTOM0.xyz) and their normals towards the parent level's
// normals (CUSTOM3.xyz) as the camera moves away: morph is 0 closer than morph_start_m and 1 beyond morph_end_m. The
// quadtree sets both per node (TerrainQuadtree.morph_range_m) so that a node is unmorphed whenever its children appear
// or vanish and a child is fully morphed (= its parent's surface) at that moment: splits and merges never pop.

// Camera position relative to the node centre (node-local render axes, metres); set per node and frame.
instance uniform vec3 camera_local = vec3(0.0);
instance uniform float morph_start_m = 1.0e30;
instance uniform float morph_end_m = 2.0e30;

// Morph factor of a vertex at node-local position `vertex`.
float terrain_morph_factor(vec3 vertex) {
	return clamp((length(vertex - camera_local) - morph_start_m) / (morph_end_m - morph_start_m), 0.0, 1.0);
}
